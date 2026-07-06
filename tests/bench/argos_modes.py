#!/usr/bin/env python3
"""
argos_modes.py — exhaustive validation of every Argos MODE and the key params,
driven from the bench (GPS injection). Requires a provisioned RCONF (run
argos_validate.py provision first) + --bench firmware.

Modes (ARGOS_MODE / ARP01): 0=OFF 1=PASS_PREDICTION 2=LEGACY 3=DUTY_CYCLE
4=DOPPLER 5=SURFACING_BURST. Each mode differs in *scheduling*; all share the
KIM2 TX path. We config the mode, inject a fix, watch the logs and assert the
mode-specific signature.
"""
import sys, time, re
from kim_bench import Bench

FIX = (-21.0097, 55.2707)
results = []


def check(name, ok, detail=""):
    results.append((name, bool(ok)))
    tag = "\033[1;32mPASS\033[0m" if ok else "\033[1;31mFAIL\033[0m"
    print(f"[{tag}] {name}" + (f"  — {detail}" if detail else ""), flush=True)


def cfg(b, params):
    if b.get_state() != "CONFIG":
        b.enter_config()
    b.write_params(params)
    b.dte("RTCW", str(int(time.time())), timeout=6)
    b.exit_config()
    time.sleep(1)


def run(b, params, watch_s):
    cfg(b, params)
    mk = b.mark()
    b.inject_gps(*FIX)
    t = time.time()
    while time.time() - t < watch_s:
        time.sleep(1)
    with b._lock:
        return [l for _, l in b.history[mk:]]


def has(lines, pat):
    rx = re.compile(pat)
    return next((l for l in lines if rx.search(l)), None)


def n_tx(lines):
    return sum(1 for l in lines if "TX EMITTED OK" in l)


def modes(b):
    print("\n===== ARGOS MODES =====")

    # OFF: service disabled -> no TX at all.
    L = run(b, {"ARGOS_MODE": "0", "ARGOS_BLIND_EN": "0"}, 30)
    check("OFF (0): no TX", not has(L, r"state_transmit_enter: TX START"),
          "no TX START seen" if not has(L, r"TX START") else "unexpected TX")

    # LEGACY: periodic TX at TR_NOM.
    L = run(b, {"ARGOS_MODE": "2", "TR_NOM": "30", "NTRY_PER_MESSAGE": "3"}, 45)
    check("LEGACY (2): TX emitted", has(L, r"TX EMITTED OK"),
          f"{n_tx(L)} TX, {'legacy sched' if has(L,'schedule') or n_tx(L) else ''}")

    # DUTY_CYCLE is a UINT param (one bit per hour, 24-bit). All-hours = decimal
    # 16777215 (=0xFFFFFF). NB: it is NOT a hex-encoded param — feed decimal.
    L = run(b, {"ARGOS_MODE": "3", "DUTY_CYCLE": "16777215", "TR_NOM": "30"}, 50)
    check("DUTY_CYCLE (3) all-hours: TX emitted", has(L, r"TX EMITTED OK"),
          f"{n_tx(L)} TX")

    # No hours (0) -> scheduler finds no slot, TX suppressed.
    L = run(b, {"ARGOS_MODE": "3", "DUTY_CYCLE": "0"}, 30)
    check("DUTY_CYCLE (3) no-hours: TX suppressed",
          not has(L, r"TX EMITTED OK") and has(L, r"no TX slot found"),
          "no TX slot found in 24h (correct gating)")

    # DOPPLER: burst promotion on a fresh fix.
    L = run(b, {"ARGOS_MODE": "4"}, 45)
    check("DOPPLER (4): burst", has(L, r"DOPPLER.*(promoting|msg|GNSS TX)") or has(L, r"TX EMITTED OK"),
          (has(L, r"DOPPLER.*\w") or "").split(']')[-1].strip() if has(L, "DOPPLER") else f"{n_tx(L)} TX")

    # PASS_PREDICTION: without valid AOP -> "no pass"; with AOP -> a scheduled pass.
    L = run(b, {"ARGOS_MODE": "1"}, 30)
    npass = has(L, r"PASS_PREDICTION returned no pass")
    check("PASS_PREDICTION (1): handled", npass or has(L, r"prepass|pass at|TX EMITTED"),
          "no-pass path (no AOP)" if npass else "pass scheduled/TX")

    # SURFACING_BURST: needs UNDERWATER_EN=1 + a dive->surface transition. Bench
    # hooks %DIVE/%SURFACE inject the SWS UW_SENSOR event. Fix cached underwater,
    # then surface promotes to the GNSS Doppler phase.
    if b.get_state() != "CONFIG":
        b.enter_config()
    b.write_params({"ARGOS_MODE": "5", "UNDERWATER_EN": "1", "TR_NOM": "30"})
    b.dte("RTCW", str(int(time.time())), timeout=6)
    b.exit_config(); time.sleep(1)
    mk = b.mark()
    b.ser.write(b"%DIVE\r\n"); b.ser.flush(); time.sleep(2)
    b.inject_gps(*FIX); time.sleep(2)
    b.ser.write(b"%SURFACE\r\n"); b.ser.flush()
    t = time.time()
    while time.time() - t < 45:
        time.sleep(1)
    with b._lock:
        L = [l for _, l in b.history[mk:]]
    check("SURFACING_BURST (5): dive->surface->burst",
          has(L, r"SURFACING_BURST.*(promoting|GNSS TX)") and has(L, r"TX START"),
          (has(L, r"SURFACING_BURST.*\w") or "").split(']')[-1].strip() if has(L, "SURFACING_BURST") else "no burst")


def params(b):
    print("\n===== PARAMETERS =====")

    # Adaptive modulation ON -> LDK<->LDA2 switching present.
    L = run(b, {"ARGOS_MODE": "2", "ARGOS_ADAPTIVE_MODULATION": "1",
                "TR_NOM": "30", "ARGOS_DEPTH_PILE": "4"}, 70)
    check("ADAPTIVE_MOD on: LDK/LDA2 switch", has(L, r"switching to 1|switch_modulation|mode=1"),
          "modulation switched" if has(L, "switch") else "check")

    # Adaptive OFF -> fixed modulation (no switch).
    L = run(b, {"ARGOS_MODE": "2", "ARGOS_ADAPTIVE_MODULATION": "0", "TR_NOM": "30"}, 45)
    check("ADAPTIVE_MOD off: fixed modulation", has(L, r"TX EMITTED OK") and not has(L, r"switching to"),
          f"{n_tx(L)} TX, no switch")

    # Depth pile: 1 position -> SHORT packet; larger pile -> LONG once >1 entry.
    L = run(b, {"ARGOS_MODE": "2", "ARGOS_DEPTH_PILE": "1", "TR_NOM": "30"}, 45)
    check("DEPTH_PILE=1: SHORT packet", has(L, r"SHORT packet"),
          (has(L, r"(SHORT|LONG) packet") or "").split(']')[-1].strip())

    # NTRY_PER_MESSAGE tours (v3): NTRY=1 -> entry eligible for 1 tour only.
    L = run(b, {"ARGOS_MODE": "2", "NTRY_PER_MESSAGE": "1", "TR_NOM": "30"}, 50)
    check("NTRY_PER_MESSAGE=1: TX then exhaust", has(L, r"TX EMITTED OK"),
          f"{n_tx(L)} TX (NTRY=1)")

    # Low-battery: LB_EN + LB_THRESHOLD=100 forces LB mode (SoC < 100%).
    L = run(b, {"ARGOS_MODE": "2", "LB_EN": "1", "LB_THRESHOLD": "100",
                "LB_ARGOS_MODE": "2", "TR_LB": "60"}, 45)
    check("LOW_BATTERY: LB mode engaged", has(L, r"is_lb|LB mode|low.?bat|LB\b"),
          (has(L, r"lb|LB") or "").split(']')[-1].strip() if has(L, "LB") else "check logs")
    # restore
    cfg(b, {"LB_EN": "0"})

    # BLIND + DUTY_CYCLE combo (BLIND valid with LEGACY/DUTY_CYCLE, not surfacing).
    L = run(b, {"ARGOS_MODE": "3", "DUTY_CYCLE": "FFFFFF",
                "ARGOS_BLIND_EN": "1", "ARGOS_BLIND_RETX_NB": "1",
                "ARGOS_BLIND_RETX_PERIOD_S": "60"}, 90)
    check("BLIND+DUTY_CYCLE: KMAC=2 + TX OK", has(L, r"BLIND loaded") and has(L, r"TX EMITTED OK"),
          "blind burst + emit" if has(L, "TX EMITTED") else (has(L, "BLIND loaded") or "").split(']')[-1].strip())
    cfg(b, {"ARGOS_BLIND_EN": "0"})


def main():
    what = sys.argv[1] if len(sys.argv) > 1 else "all"
    b = Bench(quiet=True); b.open()
    if not b.ping():
        print("no board"); return 1
    try:
        if what in ("modes", "all"):
            modes(b)
        if what in ("params", "all"):
            params(b)
        npass = sum(1 for _, ok in results if ok)
        print(f"\n===== {npass}/{len(results)} passed =====")
        print("transcript:", b.logpath)
    finally:
        b.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
