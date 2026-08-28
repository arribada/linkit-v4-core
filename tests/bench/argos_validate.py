#!/usr/bin/env python3
"""
argos_validate.py — validate Argos LEGACY and BLIND TX modes on a live KIM board.

Uses the bench harness (kim_bench.Bench). Requires --bench firmware + an attached
board (see README). Provisions the KIM2 radio config (RCONF), then drives the real
ArgosTxService TX path and captures the KIM2 AT exchange from the logs:

  LEGACY (BASIC KMAC):  AT+KMAC=1  then  NTRY_PER_MESSAGE × AT+TX   (module sends one copy per AT+TX)
  BLIND  (KMAC ctx):    AT+KMAC=2,<hex>  then  1 × AT+TX            (module owns the retx burst)

Usage:
    ./argos_validate.py provision     # write RCONF + push to module (SATVF force)
    ./argos_validate.py legacy        # configure + trigger + capture LEGACY
    ./argos_validate.py blind         # configure + trigger + capture BLIND
    ./argos_validate.py all           # provision -> legacy -> blind, full report
"""
import sys, time, re
from kim_bench import Bench

# Standard Kineis radio configs (device-independent RF params) + AES key. These
# used to be quoted from a template under template_conf/, which no longer ships
# one, so they live here now. The board keeps its own factory Argos ID
# (HEXID already 3FEBA4 in config); we only provision the missing RADIOCONF.
CREDS = {
    "ARGOS_RADIOCONF_LDK":   "03921FB104B92859209B18ABD009DE96",
    "ARGOS_RADIOCONF_LDA2":  "3D678AF16B5A572078F3DBC95A1104E7",
    "ARGOS_RADIOCONF_VLDA4": "550b4bec21009c7a7b5bebaa937cdb41",
    "ARGOS_RADIOCONF":       "03921fb104b92859209b18abd009de96",
    "ARGOS_ADAPTIVE_MODULATION": "1",
    "ARGOS_SECKEY":          "69B26EE789BD10109328437567F68469",
}

FIX = (-21.0097, 55.2707)


def show(b, since):
    with b._lock:
        for _, line in b.history[since:]:
            print("   " + line)


def provision(b):
    print("\n=== PROVISION RCONF ===")
    assert b.enter_config(), "could not enter config"
    r = b.write_params(CREDS)
    print("PARMW creds:", r.group(0) if r else "no resp")
    # Push config -> module (force=1 re-writes when hardware differs)
    mk = b.mark()
    r = b.dte("SATVF", "1", timeout=20)
    print("SATVF force=1:", r.group(0) if r else "no resp")
    time.sleep(2)
    r = b.dte("SATVF", "0", timeout=20)
    print("SATVF verify :", r.group(0) if r else "no resp")
    show(b, mk)
    return r is not None and r.group(1) == "O"


def cfg_and_trigger(b, params, label, watch_s=75):
    print(f"\n=== {label}: configure + trigger ===")
    if b.get_state() != "CONFIG":
        b.enter_config()
    r = b.write_params(params)
    print("PARMW:", r.group(0) if r else "no resp", "->", params)
    # confirm readback
    _, rp = b.read_params(list(params.keys()))
    print("readback:", rp)
    # Give the device a realistic wall-clock BEFORE leaving config: the injected
    # fix and the Argos scheduler/prepass need real UTC, and the "GNSS-corrected
    # clock" gate wants a plausible time. RTCW takes a unix epoch.
    r = b.dte("RTCW", str(int(time.time())), timeout=6)
    print("RTCW set-clock:", r.group(0) if r else "no resp")
    assert b.exit_config(), "could not exit config"
    time.sleep(1)
    mk = b.mark()
    print(f"injecting fix {FIX} and watching {watch_s}s for KIM2 AT exchange…")
    b.inject_gps(*FIX)
    t = time.time()
    while time.time() - t < watch_s:
        time.sleep(1)
    print(f"--- captured logs for {label} ---")
    show(b, mk)
    return mk


def analyze(b, since, label):
    with b._lock:
        lines = [l for _, l in b.history[since:]]
    unlocked = [l for l in lines if "TX unlocked" in l]
    tx_start = [l for l in lines if "state_transmit_enter: TX START" in l]
    tx_ok    = [l for l in lines if "TX EMITTED OK" in l or "TX SUCCESS" in l]
    blind    = [l for l in lines if "BLIND loaded" in l]
    blind_rej= [l for l in lines if "BLIND (AT+KMAC=2) rejected" in l]
    mods     = sorted(set(m.group(1) for l in lines
                          for m in [re.search(r"state_transmit_enter: mode=(\d)", l)] if m))
    print(f"\n[{label}] unlocked={bool(unlocked)}  TX_START={len(tx_start)}  "
          f"TX_OK={len(tx_ok)}  BLIND_loaded={len(blind)}  modulations={mods}")
    for l in blind[:3]:     print("  BLIND>", l.split(']')[-1].strip())
    for l in blind_rej[:2]: print("  BLIND-REJECT>", l.split(']')[-1].strip())
    for l in tx_ok[:6]:     print("  TX_OK>", l.split(']')[-1].strip())
    blind_window = [l for l in lines if "BLIND — TX window" in l]
    # Verdict
    if label == "LEGACY":
        # TX firing already implies the clock was GNSS-corrected (unlock only logs
        # once per power session). BASIC = no BLIND load, per-message AT+TX.
        ok = len(tx_ok) >= 1 and not blind
        print(f"  ==> LEGACY {'OK' if ok else 'FAIL'}: BASIC KMAC (no BLIND), "
              f"{len(tx_ok)} TX emitted, modulations={mods}")
    elif label == "BLIND":
        # BLIND = module owns the retx burst: AT+KMAC=2 accepted (retx_nb/period)
        # + a long BLIND TX window opens. The RF completion lands after the whole
        # window (retx_nb × period), which may exceed the capture; the KMAC=2 load
        # + window start are the definitive signals.
        ok = bool(blind) and bool(blind_window) and not blind_rej
        for l in blind_window[:2]: print("  WINDOW>", l.split(']')[-1].strip())
        print(f"  ==> BLIND {'OK' if ok else 'FAIL'}: "
              f"{'KMAC=2 module retx burst' if ok else 'no BLIND burst'}"
              f"{' (REJECTED->BASIC)' if blind_rej else ''}, "
              f"{len(tx_ok)} TX completed in window")


def main():
    what = sys.argv[1] if len(sys.argv) > 1 else "all"
    b = Bench(quiet=True)
    b.open()
    if not b.ping():
        print("no bench board"); return 1
    try:
        if what in ("provision", "all"):
            ok = provision(b)
            print("PROVISION:", "OK" if ok else "credentials still mismatch (check logs)")
        if what in ("legacy", "all"):
            mk = cfg_and_trigger(b, {
                "ARGOS_MODE": "2",           # LEGACY
                "ARGOS_BLIND_EN": "0",
                "NTRY_PER_MESSAGE": "2",
                "TR_NOM": "30",
            }, "LEGACY")
            analyze(b, mk, "LEGACY")
        if what in ("blind", "all"):
            mk = cfg_and_trigger(b, {
                "ARGOS_MODE": "2",           # LEGACY regime (BLIND not paired with surfacing)
                "ARGOS_BLIND_EN": "1",
                "ARGOS_BLIND_RETX_NB": "3",
                "ARGOS_BLIND_RETX_PERIOD_S": "60",
            }, "BLIND", watch_s=230)         # module burst = retx_nb × period (~240s)
            analyze(b, mk, "BLIND")
    finally:
        print(f"\nTranscript: {b.logpath}")
        b.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
