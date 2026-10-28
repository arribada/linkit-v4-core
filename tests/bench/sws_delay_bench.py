#!/usr/bin/env python3
"""Mesure de banc SWS : lecture brute du capteur d'eau salée selon le délai de charge.

POURQUOI. Chaque échantillon SWS alimente l'électrode, attend `sample_delay_us`
en attente active (le condensateur de 100 nF se charge à travers l'eau), puis
lit l'ADC. Le délai part de UNP08 (1 ms par défaut) et s'adapte entre UNP09 et
UNP10 (0,2 à 10 ms par défaut) : il monte de 10 % à chaque recalage « air » tant
que le contraste eau/air dépasse 10. Sur des électrodes propres, il finit au
plafond : 10 ms par seconde sous l'eau, 1 % du temps processeur. L'analyse
WWF-YanaRiba (octobre 2026) en a déduit que la lecture en eau de mer ne dépend
pas du délai (constante de temps bien inférieure à 200 µs), mais sans l'avoir
mesuré. Ce script le mesure, pour choisir le délai à figer.

PROTOCOLE. Un lancement par milieu, la carte en USB :
  1. électrodes dans de l'eau salée (35 g/L) : --medium eau
  2. électrodes à l'air, sèches : --medium air
  3. électrodes tout juste sorties de l'eau, encore mouillées : --medium film
Pour chaque délai, le délai est figé (UNP08 = UNP09 = UNP10), le mode test SWS
relancé (SWSTST : une lecture toutes les 100 ms) et les lectures brutes
recueillies pendant --seconds. Les valeurs d'origine de UNP08/09/10 sont
remises à la fin, même sur erreur. Tout se passe en mode configuration :
aucune émission radio.

    python3 tests/bench/sws_delay_bench.py --medium eau
    python3 tests/bench/sws_delay_bench.py --medium air
    python3 tests/bench/sws_delay_bench.py --medium film --seconds 4
    python3 tests/bench/sws_delay_bench.py --report        tableau des trois milieux

Les CSV vont dans tests/bench/logs/sws_delay_<milieu>.csv.
"""
import argparse
import csv
import os
import re
import statistics as st
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

LOGS = os.path.join(HERE, "logs")
DELAYS_US = [200, 500, 1000, 2000, 5000, 10000]
KEYS = ("UNP08", "UNP09", "UNP10")

# SWSST, poussé à chaque lecture en mode test (dte_handler.cpp, SWSTST_REQ) :
# air, water, threshold, hysteresis, raw, filtered, calibrated, underwater,
# time_in_state, surface_level, contrast_x10, observed_peak, sample_delay_us.
RX_SWSST = re.compile(r"\$O;SWSST#[0-9A-Fa-f]{3};([0-9,]+)")
F_RAW, F_FILTERED, F_UNDERWATER, F_DELAY = 4, 5, 7, 12


def parse_swsst(line):
    """Champs d'une ligne SWSST poussée, ou None."""
    m = RX_SWSST.search(line)
    if not m:
        return None
    f = [int(x) for x in m.group(1).split(",") if x != ""]
    return f if len(f) >= 13 else None


def summarize(samples, delay_us):
    """Statistiques des lectures brutes prises au délai demandé."""
    good = [f for f in samples if f[F_DELAY] == delay_us]
    raws = [f[F_RAW] for f in good]
    wrong = len(samples) - len(good)
    if not raws:
        return {"delay_us": delay_us, "n": 0, "wrong_delay": wrong}
    return {
        "delay_us": delay_us,
        "n": len(raws),
        "mean": round(st.mean(raws), 1),
        "std": round(st.pstdev(raws), 1),
        "min": min(raws),
        "max": max(raws),
        "underwater_pct": round(100.0 * sum(1 for f in good if f[F_UNDERWATER]) / len(good)),
        "wrong_delay": wrong,
    }


def measure(b, delays, seconds):
    rows = []
    for d in delays:
        r = b.dte("PARMW", f"UNP08={d},UNP09={d},UNP10={d}")
        if not r or r.group(1) != "O":
            raise RuntimeError(f"PARMW du délai {d} µs refusé : {r.group(0) if r else 'pas de réponse'}")
        b.dte("SWSTST", "0")
        mk = b.mark()
        r = b.dte("SWSTST", "1")
        if not r or r.group(1) != "O":
            raise RuntimeError("SWSTST 1 refusé")
        time.sleep(seconds)
        b.dte("SWSTST", "0")
        with b._lock:
            lines = [l for _, l in b.history[mk:]]
        samples = [f for f in (parse_swsst(l) for l in lines) if f]
        row = summarize(samples, d)
        rows.append(row)
        if row["n"]:
            print(f"  {d:6d} µs : n={row['n']:3d}  brut moyen {row['mean']:8.1f}  écart-type {row['std']:6.1f}  "
                  f"[{row['min']}..{row['max']}]  sous l'eau {row['underwater_pct']} %", flush=True)
        else:
            print(f"  {d:6d} µs : aucune lecture au bon délai ({row['wrong_delay']} à un autre)", flush=True)
    return rows


def report():
    """Tableau des milieux mesurés, par délai : moyennes et contraste eau / air et eau / film."""
    data = {}
    for medium in ("eau", "air", "film"):
        path = os.path.join(LOGS, f"sws_delay_{medium}.csv")
        if os.path.exists(path):
            with open(path) as f:
                data[medium] = {int(r["delay_us"]): r for r in csv.DictReader(f) if r.get("n") not in ("", "0")}
    if not data:
        print("aucune mesure dans", LOGS)
        return 1
    print("délai µs | eau (moy ± σ) | air (moy ± σ) | film (moy ± σ) | eau/air | eau/film")
    for d in DELAYS_US:
        cells, means = [], {}
        for medium in ("eau", "air", "film"):
            r = data.get(medium, {}).get(d)
            if r:
                means[medium] = float(r["mean"])
                cells.append(f"{float(r['mean']):7.1f} ± {float(r['std']):5.1f}")
            else:
                cells.append("        —      ")
        ratio = lambda a, b: f"{means[a] / means[b]:6.1f}" if a in means and b in means and means[b] > 0 else "   —  "
        print(f"{d:8d} | {cells[0]} | {cells[1]} | {cells[2]} | {ratio('eau', 'air')} | {ratio('eau', 'film')}")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--medium", choices=["eau", "air", "film"])
    ap.add_argument("--delays", default=",".join(str(d) for d in DELAYS_US))
    ap.add_argument("--seconds", type=float, default=8.0, help="durée de recueil par délai (lecture toutes les 100 ms)")
    ap.add_argument("--port", default=os.environ.get("BENCH_PORT"))
    ap.add_argument("--report", action="store_true")
    args = ap.parse_args()
    if args.report:
        return report()
    if not args.medium:
        ap.error("--medium ou --report")

    import kim_bench
    delays = [int(x) for x in args.delays.split(",")]
    b = kim_bench.Bench(port=args.port, quiet=True)
    b.open()
    time.sleep(1.0)
    mk = b.mark()
    b._send("%CFG\r\n")
    b.expect(r"%CFG.*", 5.0, from_idx=mk)
    time.sleep(2.0)
    _, orig = b.read_params(list(KEYS))
    if not all(k in orig for k in KEYS):
        print("lecture de UNP08/09/10 impossible :", orig)
        b.close()
        return 1
    print(f"milieu {args.medium} ; valeurs d'origine : " + ", ".join(f"{k}={orig[k]}" for k in KEYS), flush=True)
    rows = []
    try:
        rows = measure(b, delays, args.seconds)
    finally:
        b.dte("SWSTST", "0")
        r = b.dte("PARMW", ",".join(f"{k}={orig[k]}" for k in KEYS))
        print("valeurs d'origine remises :", r.group(0) if r else "PAS DE RÉPONSE — à remettre à la main", flush=True)
        b.close()
    os.makedirs(LOGS, exist_ok=True)
    path = os.path.join(LOGS, f"sws_delay_{args.medium}.csv")
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["delay_us", "n", "mean", "std", "min", "max", "underwater_pct", "wrong_delay"])
        w.writeheader()
        for r in rows:
            w.writerow(r)
    print("écrit", path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
