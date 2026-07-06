#!/usr/bin/env python3
"""
rspb_bench.py — RSPB (SMD-over-SPI) bench observer/validator.

RSPB has no USB and its debug UART is TX-only (UARTE1, P0.11, 921600). So this is
an OBSERVE-only tool: connect a USB-UART adapter's RX to the RSPB debug TX (+ GND),
build/flash the firmware with `./scripts/build_rspb.sh --bench`, upload a config
(via your usual BLE/PyLinkit path), and this reads the 921600 log stream and
validates the compressed TPL duty-cycle behaviour.

Bench firmware (BENCH_TEST + EXTERNAL_WAKEUP):
  - PMU::powerdown() -> soft reset (simulated TPL wake) instead of System OFF, so
    the board stays reachable. Compressed cycle: boot -> work -> reset -> boot ...
  - auto-injects one synthetic GPS fix per "run" boot so the SMD satellite TX
    fires with no antenna.

Usage:
  ./rspb_bench.py                 auto-detect adapter, stream + live validate
  ./rspb_bench.py --port /dev/ttyUSB0
  ./rspb_bench.py --cycles 5      stop after N duty-cycles, print a report
"""
import argparse, datetime, glob, os, re, sys, time

try:
    import serial
except ImportError:
    sys.exit("pyserial not installed. Run: pip install pyserial")

HERE = os.path.dirname(os.path.abspath(__file__))
LOGDIR = os.path.join(HERE, "logs")
BAUD = 921600

MARK = {
    "boot":      re.compile(r"EXTERNAL_WAKEUP: Boot counter = (\d+)"),
    "modulo_pd": re.compile(r"Not our turn to run \(modulo check\)"),
    "reset":     re.compile(r"soft reset \(simulated TPL wake"),
    "inject":    re.compile(r"auto_inject: injecting synthetic fix"),
    "tx_start":  re.compile(r"ArgosTxService: TX START — type=(\S+) mode=(\d)"),
    "tx_ok":     re.compile(r"ArgosTxService: TX SUCCESS — type=(\S+) mode=(\d)"),
    "tx_fail":   re.compile(r"ArgosTxService: TX FAILED"),
    "smd_tx":    re.compile(r"SmdSat.*(TX started on STM32|initiate_tx OK)"),
    "smd_boot":  re.compile(r"SmdSat::.*boot sequence"),
    "blind":     re.compile(r"BLIND KMAC loaded \(retx_nb=(\d+) period=(\d+)s\)"),
    "heartbeat": re.compile(r"0xFF heartbeat"),
}


def autodetect():
    # USB-UART adapters show up as ttyUSB* (FTDI/CP210x) or sometimes ttyACM*.
    for p in sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")):
        return p
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--cycles", type=int, default=0, help="stop after N duty-cycles")
    args = ap.parse_args()

    port = args.port or autodetect()
    if not port:
        sys.exit("No USB-UART adapter found (/dev/ttyUSB* or ttyACM*). "
                 "Plug the adapter (RX<-RSPB TX, GND) and retry, or pass --port.")
    print(f"[rspb] reading {port} @ {args.baud}")
    os.makedirs(LOGDIR, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    logf = open(os.path.join(LOGDIR, f"rspb_{stamp}.log"), "a", buffering=1)

    ser = serial.Serial(port, args.baud, timeout=0.2)

    cycles = 0
    cur = None            # current cycle accumulator
    summary = []
    buf = b""

    def close_cycle():
        nonlocal cur, cycles
        if cur is None:
            return
        cycles += 1
        ran = cur["inject"] or cur["tx_ok"] or cur["tx_start"]
        kind = "RUN" if ran else ("MODULO-SKIP" if cur["modulo_pd"] else "?")
        line = (f"[cycle {cycles}] boot#{cur['boot']} {kind}"
                f" inject={cur['inject']} TX_START={cur['tx_start']}"
                f" TX_OK={cur['tx_ok']} TX_FAIL={cur['tx_fail']}"
                f" SMD_TX={cur['smd_tx']} blind={cur['blind']}"
                f" heartbeat={cur['heartbeat']} reset={cur['reset']}")
        print("\033[1m" + line + "\033[0m", flush=True)
        summary.append((cycles, cur["boot"], kind, cur["tx_ok"], cur["tx_fail"], cur["reset"]))
        cur = None

    def new_cycle(boot):
        return dict(boot=boot, modulo_pd=False, inject=False, tx_start=0, tx_ok=0,
                    tx_fail=0, smd_tx=0, blind=None, heartbeat=False, reset=False)

    try:
        while True:
            chunk = ser.read(1024)
            if chunk:
                buf += chunk
                while True:
                    i = min([k for k in (buf.find(b"\r"), buf.find(b"\n")) if k >= 0], default=-1)
                    if i < 0:
                        break
                    raw, buf = buf[:i], buf[i+1:]
                    if not raw:
                        continue
                    line = raw.decode("utf-8", "replace")
                    logf.write(datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3] + "  " + line + "\n")

                    m = MARK["boot"].search(line)
                    if m:
                        close_cycle()
                        cur = new_cycle(int(m.group(1)))
                        continue
                    if cur is None:
                        # pre-first-boot noise; still surface TX/errors
                        cur = new_cycle(-1)
                    if MARK["modulo_pd"].search(line): cur["modulo_pd"] = True
                    if MARK["inject"].search(line):    cur["inject"] = True
                    if MARK["tx_start"].search(line):  cur["tx_start"] += 1
                    if MARK["tx_ok"].search(line):     cur["tx_ok"] += 1
                    if MARK["tx_fail"].search(line):   cur["tx_fail"] += 1
                    if MARK["smd_tx"].search(line):    cur["smd_tx"] += 1
                    if MARK["heartbeat"].search(line): cur["heartbeat"] = True
                    b = MARK["blind"].search(line)
                    if b: cur["blind"] = (int(b.group(1)), int(b.group(2)))
                    if MARK["reset"].search(line):
                        cur["reset"] = True
                        close_cycle()
                        if args.cycles and cycles >= args.cycles:
                            raise KeyboardInterrupt
    except KeyboardInterrupt:
        pass
    finally:
        close_cycle()
        runs = [s for s in summary if s[2] == "RUN"]
        oks = sum(s[3] for s in summary)
        print(f"\n=== {cycles} cycles observed | {len(runs)} RUN | {oks} TX SUCCESS ===")
        ser.close(); logf.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
