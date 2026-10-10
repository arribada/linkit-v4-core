#!/usr/bin/env python3
"""flash.py — flash the LinkIt V4 bench firmware, the CLAUDE.md way, on Windows or Linux.

    python tests/bench/flash.py                      newest ports/nrf52840/build*/LINKIT/*_merged-*.hex
    python tests/bench/flash.py --hex <path.hex>     an explicit MERGED hex
    python tests/bench/flash.py --check-probe        list probes and processes holding them, flash nothing

Sequence (never a different one): nrfjprog --recover (ONE attempt, hard limit
120 s, reported instead of retried), then --program <merged> --chiperase
--verify --reset, and both 'Verify successful' and 'Run.' must appear. Exit 0
only then. APPROTECT re-arms on these boards and a silent flash failure leaves
the OLD image running.

--recover WIPES the configuration store: re-apply every parameter afterwards.
Replaces flash.sh (bash, defaulted to app_settings, did not check the output).

RADIO DISARM (after a confirmed flash, unless --no-disarm). The wiped store
boots on factory defaults -- ARGOS_MODE=LEGACY, GNSS on, TR_NOM 60 s
(config_store.hpp) -- and a KIM2 that answers transmits on its built-in LDK
RCONF and factory ID even without credentials. So flash.py waits for the CDC
port, sends %AMODE 0 at once (RAM, immediate), enters configuration, writes
ARGOS_MODE=0 + CERT_TX_ENABLE=0 (persisted by PARMW), reads ARGOS_MODE=0 back,
and closes the port. RF is only safe once 'RADIO DISARMED' is printed. Exit 4:
flashed, but the radio could NOT be disarmed (unplug the USB until a harness
can force ARGOS_MODE=0). The board is left in CONFIGURATION.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bench_platform as P  # noqa: E402


def main():
    P.setup_stdio()
    ap = argparse.ArgumentParser(description="flash the merged bench hex (recover + program + verify)")
    ap.add_argument("--hex", help="merged hex (default: newest *_merged-*.hex in the build dir)")
    ap.add_argument("--check-probe", action="store_true", help="only list probes / probe holders")
    ap.add_argument("--log-dir", default=P.temp_dir(), help="where nrfjprog --log writes (default: temp dir)")
    ap.add_argument("--no-disarm", action="store_true",
                    help="do NOT force ARGOS_MODE=0 after flashing (factory defaults = LEGACY, real RF "
                         "on a KIM2 that answers)")
    ap.add_argument("--disarm-only", action="store_true",
                    help="flash nothing: only force ARGOS_MODE=0 on the running board")
    a = ap.parse_args()
    if a.disarm_only:
        disarmed, why = disarm_radio()
        print(("RADIO DISARMED: " if disarmed else "RADIO NOT DISARMED: ") + why)
        return 0 if disarmed else 4

    ids = P.nrfjprog(["--ids"], timeout=30)
    probes = [l.strip() for l in ids.stdout.splitlines() if l.strip().isdigit()]
    holders = P.probe_holders()
    print(f"nrfjprog: {P.nrfjprog_exe()}")
    print(f"probes: {probes or 'NONE'}" + (" (TIMEOUT)" if ids.timed_out else ""))
    if holders:
        print(f"WARNING processes that may hold the probe: {', '.join(holders)} -- close nRF Connect "
              "and kill nrfutil-device before flashing")
    if a.check_probe:
        return 0 if probes else 2
    if not probes:
        print("no J-Link visible to nrfjprog: nothing flashed")
        return 2
    if len(probes) > 1 and not os.environ.get("NRFJPROG_SNR"):
        print("several probes: set NRFJPROG_SNR to the one wired to the tag")
        return 2

    hexpath = a.hex or P.find_merged_hex()
    print(f"image: {hexpath}")
    ok, report = P.flash_merged(hexpath, log_dir=a.log_dir)
    print(("FLASH OK: " if ok else "FLASH FAILED: ") + report)
    if not ok:
        return 1
    if a.no_disarm:
        print("WARNING --no-disarm: the board runs factory defaults (ARGOS_MODE=LEGACY) -- real RF possible")
        return 0
    disarmed, why = disarm_radio()
    print(("RADIO DISARMED: " if disarmed else "RADIO NOT DISARMED: ") + why)
    return 0 if disarmed else 4


def disarm_radio(port_timeout=45.0):
    """Force ARGOS_MODE=0 on the freshly flashed board. (ok, report)."""
    try:
        import kim_bench
    except Exception as e:                      # pyserial missing, ...
        return False, f"cannot load kim_bench ({type(e).__name__}: {e})"
    port = os.environ.get("BENCH_PORT") or P.wait_for_port(port_timeout)
    if not port:
        return False, (f"no CDC port 1915:520F within {port_timeout:.0f} s after 'Run.' -- the board runs "
                       "factory defaults (ARGOS_MODE=LEGACY): unplug the USB, or run kim_bench --run NOW")
    b = None
    try:
        t_end = time.monotonic() + 20.0
        while True:
            try:
                b = kim_bench.Bench(port=port, quiet=True)
                guard = kim_bench.RFGuard(b)
                b.open()
                break
            except Exception as e:              # port enumerated but not yet openable
                if b is not None:
                    try:
                        b.close()
                    except Exception:
                        pass
                    b = None
                if time.monotonic() > t_end:
                    return False, f"cannot open {port}: {type(e).__name__}: {e}"
                time.sleep(1.0)
        # %AMODE 0 first: RAM write, effective at once, works in OPERATIONAL.
        b._send("%AMODE 0\r\n")
        amode = b.expect(r"%AMODE mode=0", 6.0, from_idx=0)
        # First boot after a flash: a %CFG sent during BootState is acknowledged,
        # the board enters ConfigurationState and LEAVES it 0.3 s later on its own
        # (BootState -> Config -> PreOp -> Operational), so every PARMW lands in
        # Operational and is ignored (2026-10-10, SMD board). Wait until the
        # board has settled in OPERATIONAL, then enter config.
        t_settle = time.monotonic() + 60.0
        while time.monotonic() < t_settle and b.get_state() != "OPERATIONAL":
            time.sleep(1.0)
        time.sleep(2.0)
        if not b.enter_config(timeout=20.0):
            return False, f"%CFG not acknowledged on {port} (%AMODE 0 {'acked' if amode else 'NOT acked'})"
        # The DTE is not served at once after %CFG, and the first boot after
        # --recover also formats the config store: both swallowed the single
        # PARMW/PARMR of the first version (reply None, exit 4 on 2026-10-10).
        time.sleep(2.0)
        replies = []
        disarmed = False
        for attempt in range(1, 5):
            if b.get_state() != "CONFIG" and not b.enter_config(timeout=20.0):
                replies.append(f"#{attempt} not in CONFIG")
                time.sleep(3.0)
                continue
            w = b.write_params({"ARGOS_MODE": 0, "CERT_TX_ENABLE": 0}, timeout=15.0, strict=False)
            raw, p = b.read_params(["ARGOS_MODE"], timeout=10.0)
            replies.append(f"#{attempt} PARMW={w.group(0) if w else None!r} PARMR={raw!r}")
            if p.get("ARP01") == "0":
                disarmed = True
                break
            time.sleep(3.0)
        if not disarmed:
            with b._lock:
                tail = [l for _, l in b.history[-15:]]
            return False, ("ARGOS_MODE not read back as 0 after 4 attempts (%AMODE 0 "
                           f"{'acked' if amode else 'NOT acked'}): " + " | ".join(replies)
                           + " || last lines: " + " / ".join(tail))
        if guard.seen:
            return False, (f"ARGOS_MODE=0 is set, but a REAL EMISSION happened before it: "
                           f"{guard.seen.strip()[-100:]}")
        return True, f"ARGOS_MODE=0 + CERT_TX_ENABLE=0 written and read back on {port}; board left in CONFIG"
    except Exception as e:
        return False, f"{type(e).__name__}: {e}"
    finally:
        if b is not None:
            try:
                b.close()
            except Exception:
                pass


if __name__ == "__main__":
    sys.exit(main())
