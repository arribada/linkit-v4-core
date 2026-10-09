#!/usr/bin/env python3
"""cl_aging.py — CloudLocate MEAS20 vs PVT apres vieillissement d'ephemeride.

Hypothese : le MEAS20 (mesures brutes, resolues au cloud) n'a pas besoin
d'ephemeride valide, donc reste rapide apres une plongee > 4 h alors que le PVT
calcule sur la puce s'effondre (~27-32 s mesure en fix-mode). Sequence :
amorce (eph fraiche) -> plongee longue -> a la remontee on chronometre SEPAREMENT
time-to-MEAS20 et time-to-PVT, sans s'arreter au premier des deux.

Cross-plateforme (WSL ou Windows natif) : Board.find_port() trouve le port par VID.
Prerequis : firmware --bench --validation. Duree ~6 h (15 min + 4 h + 2 h de plongee).
Resultats incrementaux : cl_aging_results.csv a cote du script.
"""
import csv, os, re, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from gnss_bench import Board

OUT = os.path.join(HERE, "cl_aging_results.csv")

# CloudLocate MEAS20 a chaque surface, PVT conserve (GNP53=0), mask 7, rail jamais coupe,
# batterie de banc neutralisee. CloudLocate exige ARGOS_MODE=SURFACING_BURST (%AMODE 5) ;
# sans credentials KIM2 les emissions echouent, sans effet sur le GNSS.
CFG = ("GNP02=0,GNP20=0,GNP05=120,GNP09=300,GNP24=1,GNP52=4294967295,UNP20=0,"
       "GNP40=7,LBP02=0,LBP12=0,GNP45=2,GNP46=1,GNP51=1,GNP53=0")
DIVE_MIN = [int(a) for a in sys.argv[1:]] or [15, 240, 120]  # ex. : cl_aging.py 240 120
OBSERVE_AFTER_FIRST_S = 90

RX_PVT   = re.compile(r"GNSS time persisted|reset ntry=\d+->0 \(PVT", re.I)
RX_CL    = re.compile(r"first raw measurement available|broadcasting GNSS_CLOUDLOCATE_READY", re.I)
RX_CLASS = re.compile(r"start_class=(\S+)")
RX_WAKE  = re.compile(r"extint_wake_presync", re.I)
RX_BOOT  = re.compile(r"boot sync at (\d+) baud", re.I)
RX_NOFX  = re.compile(r"retry_counter: NO_FIX|MaxNavSamples|MaxSatSamples|NO raw measurement captured", re.I)
RX_SURF  = re.compile(r"%SURFACE OK surfaced", re.I)


def main():
    b = Board()
    print("port:", b.port, flush=True)
    time.sleep(1.0)
    i = b.mark(); b.send("%AMODE 5"); print("AMODE:", b.wait(r"%AMODE mode=", 6, since=i), flush=True)
    i = b.mark(); b.send("%CFG"); b.wait(r"%CFG OK", 6, since=i); time.sleep(2.0)
    b.send(f"$PARMW#{len(CFG):03X};{CFG}"); time.sleep(1.5)
    i = b.mark(); b.send("%OP"); b.wait(r"%OP OK", 8, since=i); time.sleep(3.0)

    header = os.path.exists(OUT) and os.path.getsize(OUT) > 0
    for n, dmin in enumerate(DIVE_MIN, 1):
        name = f"CL{n}_gap{dmin}min"
        outages0 = b.outages
        i = b.mark(); b.send("%DIVE"); b.wait(r"%DIVE OK", 8, since=i)
        print(f"[{name}] plongee {dmin} min", flush=True)
        time.sleep(dmin * 60)

        i = b.mark(); b.send("%SURFACE")
        t_surf = time.monotonic() - b.t0
        if b.wait(r"%SURFACE OK surfaced", 10, since=i):
            t_surf = next((t for t, l in b.since(i) if RX_SURF.search(l)), t_surf)

        ev, seen = {}, i
        deadline = time.monotonic() + 400
        while time.monotonic() < deadline:
            for t, line in b.since(seen):
                seen += 1
                if "wake" not in ev and RX_WAKE.search(line): ev["wake"] = t
                if "boot" not in ev and RX_BOOT.search(line): ev["boot"] = t
                m = RX_CLASS.search(line)
                if "cls" not in ev and m: ev["cls"] = m.group(1)
                if "cl" not in ev and RX_CL.search(line): ev["cl"] = t
                if "pvt" not in ev and RX_PVT.search(line): ev["pvt"] = t
                if "nofix" not in ev and RX_NOFX.search(line): ev["nofix"] = t
            if "cl" in ev and "pvt" in ev:
                break
            first = min([ev[k] for k in ("cl", "pvt") if k in ev], default=None)
            if first is not None and (time.monotonic() - b.t0) - first > OBSERVE_AFTER_FIRST_S:
                break
            time.sleep(0.3)

        row = {
            "cycle": name, "gap_min": dmin,
            "path": "deep-idle" if "wake" in ev else ("cold-rail" if "boot" in ev else "?"),
            "start_class": ev.get("cls", ""),
            "t_cloudlocate_s": round(ev["cl"] - t_surf, 1) if "cl" in ev else "",
            "t_pvt_s": round(ev["pvt"] - t_surf, 1) if "pvt" in ev else "",
            "cl_minus_pvt_s": round(ev["cl"] - ev["pvt"], 1) if ("cl" in ev and "pvt" in ev) else "",
            "result": ("CL+PVT" if ("cl" in ev and "pvt" in ev) else "CL_ONLY" if "cl" in ev
                       else "PVT_ONLY" if "pvt" in ev else "NO_FIX" if "nofix" in ev else "TIMEOUT"),
        }
        if b.outages != outages0:
            # Lien perdu pendant le cycle : la plongee ou la remontee ont pu ne
            # jamais partir, la ligne ne mesure rien.
            row["result"] = f"INVALID_LINK_LOST({row['result']})"
        with open(OUT, "a", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(row.keys()))
            if not header:
                w.writeheader(); header = True
            w.writerow(row)
        print(f"[{name}] {row}", flush=True)

        time.sleep(15)
        t2 = time.monotonic() + 15
        while time.monotonic() < t2:
            if any(re.search(r"enterbackup|backupidle", l) for _, l in b.since(seen)):
                break
            time.sleep(0.4)

    print("DONE", flush=True)
    b.close()


if __name__ == "__main__":
    main()
