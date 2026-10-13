#!/usr/bin/env python3
"""Every bench-console command is still routed, on a real board.

WHY THIS EXISTS. handle_line() used to be one chain of 35 `cmd ==` comparisons.
It is now five functions, one per command family, tried in order, with a single
"unknown-cmd" answer left in handle_line. The branches were moved verbatim and
that was checked mechanically -- same 35 comparisons, same order, and a
multiset diff of the bodies showing only the five dispatch calls -- but no host
test covers this file at all: it is BENCH_TEST-only firmware. A command dropped
from the dispatch chain would therefore fail silently, the first time someone
reached for it in the middle of a campaign.

WHAT IT CHECKS, and what it deliberately does not. It asserts every command is
RECOGNISED -- the reply is not "unknown-cmd" -- not that it behaves. That is the
exact failure mode a dispatch refactor can introduce, and it is safe to probe:
most of these commands answer a bare invocation with a status line or a usage
message, and either proves the routing. Checking behaviour is the job of the
campaign scripts next to this one.

The command list is PARSED FROM THE FIRMWARE SOURCE, not copied here, so it
cannot drift: add a command to bench_console.cpp and this sweep covers it.

Needs a board flashed with a --bench build. Run from the repository root:
    python3 tests/bench/console_sweep.py
"""

import re
import subprocess
import sys
import threading
import time
from pathlib import Path

try:
    import serial
except ImportError:
    sys.exit("pyserial missing: pip install pyserial")

REPO = Path(__file__).resolve().parents[2]
CONSOLE_SRC = REPO / "ports/nrf52840/core/interface/bench_console.cpp"

# %STORE is the one command a sweep must not fire: it runs PMU::powerdown() for
# real on a --bench-storage build, and the USB link goes with it. It is reachable
# only through handle_sensors_cmd, which every other command in that family also
# exercises, so skipping it costs no coverage of the dispatch.
SKIP = {"%STORE"}


def commands_from_source():
    """The commands the firmware actually dispatches, in source order."""
    if not CONSOLE_SRC.exists():
        sys.exit(f"cannot read {CONSOLE_SRC}")
    text = CONSOLE_SRC.read_text(encoding="utf-8", errors="replace")
    seen, out = set(), []
    for name in re.findall(r'cmd == "(%[A-Z]+)"', text):
        if name not in seen:
            seen.add(name)
            out.append(name)
    return out


def find_port():
    """First CDC port that opens. Windows-native bench uses COM*, WSL /dev/ttyACM*."""
    import glob

    candidates = sorted(glob.glob("/dev/ttyACM*"))
    if not candidates:
        try:
            from serial.tools import list_ports

            candidates = [p.device for p in list_ports.comports()]
        except Exception:
            candidates = []
    for dev in candidates:
        try:
            serial.Serial(dev, 115200, timeout=0.1, write_timeout=5).close()
            return dev
        except Exception:
            continue
    return None


class Board:
    def __init__(self, port):
        self.ser = serial.Serial(port, 115200, timeout=0.1, write_timeout=5)
        self._lock = threading.Lock()
        self._rx = []
        self._stop = False
        self._t = threading.Thread(target=self._reader, daemon=True)
        self._t.start()

    def _reader(self):
        buf = b""
        while not self._stop:
            try:
                buf += self.ser.read(256)
            except Exception:
                break
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self._rx.append(line.decode("utf-8", "replace").strip())

    def send(self, text, settle=1.2):
        """Send one command and return the lines it produced."""
        with self._lock:
            self._rx.clear()
            self.ser.write((text + "\r\n").encode())
        time.sleep(settle)
        return list(self._rx)

    def close(self):
        self._stop = True
        time.sleep(0.2)
        self.ser.close()


def main():
    commands = commands_from_source()
    print(f"{len(commands)} commands parsed from {CONSOLE_SRC.name}")

    port = find_port()
    if not port:
        sys.exit("no CDC port found — is a --bench board attached and the link up?")
    print(f"port: {port}")

    board = Board(port)
    try:
        # RF guard FIRST, before anything else is sent. A KIM2 that answers will
        # transmit even with no credentials loaded, and a sweep that wakes the
        # Argos path before silencing it would radiate.
        board.send("%AMODE 0", settle=2.0)

        handshake = board.send("%PING", settle=2.0)
        if not any("%BENCH OK" in l for l in handshake):
            sys.exit(f"no %PING answer — not a --bench build? got: {handshake}")
        print("handshake OK\n")

        failures, skipped = [], []
        for cmd in commands:
            if cmd in SKIP:
                skipped.append(cmd)
                print(f"  {cmd:<12} SKIPPED (would power the board down)")
                continue
            # %I2C scans the bus and takes about 30 s.
            lines = board.send(cmd, settle=32.0 if cmd == "%I2C" else 1.2)
            unknown = any("unknown-cmd" in l for l in lines)
            answered = bool(lines)
            if unknown or not answered:
                failures.append((cmd, "unknown-cmd" if unknown else "no reply"))
                print(f"  {cmd:<12} FAIL  {'unknown-cmd' if unknown else 'no reply'}")
            else:
                print(f"  {cmd:<12} ok    {lines[0][:72]}")

        print()
        print(f"{len(commands) - len(failures) - len(skipped)}/{len(commands) - len(skipped)} routed", end="")
        print(f", {len(skipped)} skipped" if skipped else "")
        if failures:
            print("\nNOT ROUTED — the dispatch chain lost these:")
            for cmd, why in failures:
                print(f"  {cmd}: {why}")
            return 1
        print("every command reached a handler")
        return 0
    finally:
        board.close()


if __name__ == "__main__":
    sys.exit(main())
