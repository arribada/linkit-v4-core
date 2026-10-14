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
    """The commands the firmware dispatches, in source order.

    Returns (name, conditional) pairs. `conditional` means the command sits
    inside a build-variant #if, so a board built without that option answers
    "unknown-cmd" and is RIGHT to: %AXLFAIL lives under LORA_MOTION_EXT, off on
    a KIM build. Reported absent on the first bench run before this was handled.
    """
    if not CONSOLE_SRC.exists():
        sys.exit(f"cannot read {CONSOLE_SRC}")
    text = CONSOLE_SRC.read_text(encoding="utf-8", errors="replace")

    # Track preprocessor depth, ignoring the file-level `#ifdef BENCH_TEST` that
    # wraps everything (depth 1 is the whole file, not a variant).
    depth = 0
    seen, out = set(), []
    for line in text.splitlines():
        st = line.strip()
        if re.match(r"#\s*(if|ifdef|ifndef)\b", st):
            depth += 1
        elif re.match(r"#\s*endif\b", st):
            depth = max(0, depth - 1)
        for name in re.findall(r'cmd == "(%[A-Z]+)"', line):
            if name not in seen:
                seen.add(name)
                out.append((name, depth > 1))
    return out


NORDIC_VID = 0x1915  # Nordic Semiconductor, the nRF52840 USB CDC


def find_port():
    """The board's CDC port. Windows-native bench uses COM*, WSL /dev/ttyACM*.

    Ordered by VID, not by port number: on Windows the first port that merely
    OPENS is COM1, a motherboard serial port with nothing behind it, and the
    first bench run talked to it for a while before giving up. A port whose USB
    VID is Nordic's is the board; everything else is a fallback.
    """
    import glob

    ranked = []
    try:
        from serial.tools import list_ports

        for p in list_ports.comports():
            nordic = (p.vid == NORDIC_VID)
            legacy = p.device.upper() in ("COM1", "COM2")  # on-board UARTs, never the tag
            ranked.append((0 if nordic else (2 if legacy else 1), p.device))
    except Exception:
        pass
    for dev in sorted(glob.glob("/dev/ttyACM*")):
        ranked.append((1, dev))

    for _, dev in sorted(ranked):
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

    def send(self, text, settle=1.2, quiet_for=0.4, cap=None):
        """Send one command and return the lines it produced.

        Waits for the reply to go QUIET instead of sleeping a fixed time. A fixed
        delay has to be either too short for %I2C and %SCHED, which answer over
        seconds and many lines, or wasteful for the 30 commands that answer in
        milliseconds — the first bench run hit the short end and read truncated
        replies. This returns once nothing new has arrived for `quiet_for`, with
        `settle` as the floor and `cap` as the ceiling.
        """
        cap = cap if cap is not None else max(settle * 6, 8.0)
        with self._lock:
            self._rx.clear()
            self.ser.write((text + "\r\n").encode())
        t0 = time.time()
        last_len, last_change = 0, t0
        while True:
            time.sleep(0.1)
            now = time.time()
            if len(self._rx) != last_len:
                last_len, last_change = len(self._rx), now
            if now - t0 >= cap:
                break
            if now - t0 >= settle and now - last_change >= quiet_for and last_len:
                break
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

        failures, skipped, absent = [], [], []
        for cmd, conditional in commands:
            if cmd in SKIP:
                skipped.append(cmd)
                print(f"  {cmd:<12} SKIPPED (would power the board down)")
                continue
            # %I2C scans both buses and answers over tens of seconds.
            lines = board.send(cmd, settle=2.0, cap=40.0 if cmd == "%I2C" else 10.0)
            unknown = any("unknown-cmd" in l for l in lines)
            if unknown and conditional:
                # Correct behaviour: the command is compiled out on this variant.
                absent.append(cmd)
                print(f"  {cmd:<12} n/a   not built into this variant")
            elif unknown or not lines:
                failures.append((cmd, "unknown-cmd" if unknown else "no reply"))
                print(f"  {cmd:<12} FAIL  {'unknown-cmd' if unknown else 'no reply'}")
            else:
                print(f"  {cmd:<12} ok    {lines[0][:72]}")

        print()
        testable = len(commands) - len(skipped) - len(absent)
        print(f"{testable - len(failures)}/{testable} routed", end="")
        if absent:
            print(f", {len(absent)} not in this variant", end="")
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
