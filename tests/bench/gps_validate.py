#!/usr/bin/env python3
"""gps_validate.py — validation banc de l'optimisation GPS (classifieur start_class + TTFF).

Cross-plateforme (Linux/WSL ET Windows natif) : utilise Board.find_port() du harnais
(VID Nordic). Lance depuis la racine du depot OU n'importe ou (chemin robuste).

Scenario : ancrage (froid, BBR perdue apres recover) puis quelques cycles courts
(BBR retenue, eph fraiche). Attendu : start_class=S0_cold au 1er, puis S1_hot/S2/S4
selon ciel ; skip_assist=True (froid: False) ; TTFF hot ~5-9 s par bon ciel.

Prerequis : firmware --bench --validation flashe (lignes [VAL-GNSS]).
"""
import os, re, sys, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gnss_bench import Board

CFG = ("GNP02=0,GNP20=0,GNP05=120,GNP09=300,GNP24=1,GNP52=4294967295,UNP20=0,"
       "GNP40=7,LBP02=0,LBP12=0,GNP45=0")
PLAN = [("ANCRAGE", 0), ("H1", 90), ("H2", 90), ("H3", 90)]

RX_CLASS = re.compile(r"start_class=(\S+) bbr=(\d+) n_eph=(\d+) n_aop=(\d+) n_ano=(\d+) n_alm=(\d+) n_good=(\d+) numSvs=(\d+)")
RX_FIX   = re.compile(r"GNSS time persisted|reset ntry=\d+->0 \(PVT", re.I)
RX_SKIP  = re.compile(r"configure->startreceive", re.I)
RX_ANO   = re.compile(r"state \w+->sendofflinedatabase|injecting \d+ bytes", re.I)
RX_SURF  = re.compile(r"%SURFACE OK surfaced", re.I)


def now(b):
    return time.monotonic() - b.t0


def main():
    b = Board()
    print("port:", b.port, flush=True)
    time.sleep(1.0)
    idx = b.mark(); b.send("%AMODE 0"); b.wait(r"%AMODE mode=", 6, since=idx)
    idx = b.mark(); b.send("%CFG"); b.wait(r"%CFG OK", 6, since=idx); time.sleep(2.0)
    b.send(f"$PARMW#{len(CFG):03X};{CFG}"); time.sleep(1.5)
    idx = b.mark(); b.send("%OP"); b.wait(r"%OP OK", 8, since=idx); time.sleep(3.0)

    for name, dive in PLAN:
        if dive:
            idx = b.mark(); b.send("%DIVE"); b.wait(r"%DIVE OK", 6, since=idx)
            print(f"[{name}] dive {dive}s", flush=True); time.sleep(dive)
        idx = b.mark(); b.send("%SURFACE")
        t_surf = now(b)
        if b.wait(r"%SURFACE OK surfaced", 10, since=idx):
            t_surf = next((t for t, l in b.since(idx) if RX_SURF.search(l)), t_surf)
        cls = None; t_fix = None; skip = False; ano = False; seen = idx
        dl = time.monotonic() + 150
        while time.monotonic() < dl:
            for t, l in b.since(seen):
                seen += 1
                m = RX_CLASS.search(l)
                if m and not cls: cls = m.groups()
                if RX_SKIP.search(l): skip = True
                if RX_ANO.search(l): ano = True
                if RX_FIX.search(l) and not t_fix: t_fix = t
            if t_fix and cls: break
            time.sleep(0.3)
        ttff = round(t_fix - t_surf, 1) if t_fix else None
        print(f"[{name}] start_class={cls} skip_assist={skip} ran_ano={ano} TTFF={ttff}s", flush=True)
        if t_fix: time.sleep(12)
        t2 = time.monotonic() + 12
        while time.monotonic() < t2:
            if any(re.search(r"enterbackup|backupidle", l) for _, l in b.since(seen)): break
            time.sleep(0.4)
    print("DONE", flush=True)
    b.close()


if __name__ == "__main__":
    main()
