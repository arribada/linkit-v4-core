#!/usr/bin/env python3
"""
argos_deep.py — deeper bench checks: depth pile, message structure, adaptive
modulation. Requires provisioned RCONF (argos_validate.py provision) + --bench fw.

Depth pile (ARGOS_DEPTH_PILE) pools up to N GPS positions into one Argos frame:
  1 position  -> SHORT packet, 96 bits, fits LDK (mode 0)
  >1 position -> LONG packet, LDA2 frame bits, needs LDA2 (mode 1)
Adaptive modulation switches LDK<->LDA2 by payload size.
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


def inject_and_watch(b, n, watch_s, spacing=4):
    mk = b.mark()
    for i in range(n):
        # nudge each fix slightly so entries are distinct positions
        b.inject_gps(FIX[0] + i * 0.01, FIX[1] + i * 0.01)
        time.sleep(spacing)
    t = time.time()
    while time.time() - t < watch_s:
        time.sleep(1)
    with b._lock:
        return [l for _, l in b.history[mk:]]


def packets(lines):
    """Return list of (kind, positions, bits) for each build_gnss_packet log."""
    out = []
    for l in lines:
        m = re.search(r"build_gnss_packet: (SHORT) packet, 1 position, (\d+) bits", l)
        if m:
            out.append(("SHORT", 1, int(m.group(2)))); continue
        m = re.search(r"build_gnss_packet: (LONG) packet, (\d+) position\(s\), (\d+) bits", l)
        if m:
            out.append(("LONG", int(m.group(2)), int(m.group(3))))
    return out


def modes_used(lines):
    return sorted(set(int(m.group(1)) for l in lines
                      for m in [re.search(r"state_transmit_enter: mode=(\d)", l)] if m))


def main():
    b = Bench(quiet=True); b.open()
    if not b.ping():
        print("no board"); return 1
    try:
        print("\n===== DEPTH PILE + MESSAGE STRUCTURE + ADAPTIVE MODULATION =====")

        # DEPTH_PILE=1: every TX is a SHORT single-position packet -> LDK (mode 0).
        cfg(b, {"ARGOS_MODE": "2", "ARGOS_DEPTH_PILE": "1",
                "ARGOS_ADAPTIVE_MODULATION": "1", "TR_NOM": "30", "NTRY_PER_MESSAGE": "5"})
        L = inject_and_watch(b, 3, 65)
        pk = packets(L); md = modes_used(L)
        shorts = [p for p in pk if p[0] == "SHORT"]
        check("DEPTH_PILE=1 -> SHORT 96-bit single-position packets",
              bool(shorts) and all(p[2] == 96 for p in shorts) and not any(p[0] == "LONG" for p in pk),
              f"packets={pk} modes={md}")
        check("DEPTH_PILE=1 -> LDK (mode 0) modulation", md == [0] or (0 in md and 1 not in md),
              f"modes={md}")

        # DEPTH_PILE=4: multiple positions pool into a LONG packet -> LDA2 (mode 1).
        cfg(b, {"ARGOS_DEPTH_PILE": "4"})
        L = inject_and_watch(b, 4, 80)
        pk = packets(L); md = modes_used(L)
        longs = [p for p in pk if p[0] == "LONG"]
        check("DEPTH_PILE=4 -> LONG multi-position packet",
              bool(longs) and max((p[1] for p in longs), default=0) >= 2,
              f"packets={pk} modes={md}")
        check("DEPTH_PILE=4 -> adaptive switch to LDA2 (mode 1)", 1 in md,
              f"modes={md} (LONG {longs[0][2] if longs else '?'} bits > 128 needs LDA2)")

        # Adaptive OFF -> no LDK<->LDA2 switch, fixed master modulation.
        cfg(b, {"ARGOS_DEPTH_PILE": "4", "ARGOS_ADAPTIVE_MODULATION": "0"})
        L = inject_and_watch(b, 2, 55)
        sw = [l for l in L if "switch_modulation" in l or "ensure_modulation: switching" in l]
        check("ADAPTIVE=0 -> no runtime modulation switching",
              not sw and bool([l for l in L if "TX EMITTED OK" in l]),
              f"{len(sw)} switch logs, {sum('TX EMITTED OK' in l for l in L)} TX")

        npass = sum(1 for _, ok in results if ok)
        print(f"\n===== {npass}/{len(results)} passed =====")
        print("transcript:", b.logpath)
    finally:
        b.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
