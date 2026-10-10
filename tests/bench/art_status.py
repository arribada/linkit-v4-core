#!/usr/bin/env python3
"""art_status.py — statuts de garde de modulation (c5de1408) lus sur la carte, sans emission radio.

Sequence : %CFG -> PARMR UNP26,ARP54,SMP01 -> STATR (tout) -> SATVF force=1 (KIM2 : reecrit et relit
la RCONF maitresse, aucun AT+TX) -> STATR -> %OP. Affiche les reponses DTE brutes et un resume
ART12 (masque d'avertissements) / ART13 (modulation maitresse relue : 0 LDA2, 1 LDK, 2 VLDA4, 3 inconnue).
Attendu sur la carte de banc (maitresse LDK) : ART13=1 apres SATVF ; ART12 non nul si la config
courante demande CloudLocate MEAS20 (bit0), 0 sinon.
"""
import os
import re
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gnss_bench import Board  # noqa: E402

RX_REPLY = re.compile(r"\$([ON]);(PARMR|STATR|SATVF)#[0-9A-Fa-f]{3};([^\r\n]*)")


def dte(b, cmd, wait_s=6.0):
    i = b.mark()
    b.send(cmd)
    t_end = time.monotonic() + wait_s
    while time.monotonic() < t_end:
        for _, line in b.since(i):
            m = RX_REPLY.search(line)
            if m:
                return m.group(0)
        time.sleep(0.2)
    return None


def arts(reply):
    return dict(re.findall(r"(ART1[2-5])=([0-9]+)", reply or ""))


def main():
    b = Board(rf_guard=True)
    print("port:", b.port, flush=True)
    time.sleep(1.0)
    i = b.mark(); b.send("%CFG"); print("CFG:", b.wait(r"%CFG OK", 8, since=i), flush=True)
    time.sleep(2.0)
    p = "UNP26,ARP54,SMP01"
    print("PARMR:", dte(b, f"$PARMR#{len(p):03X};{p}"), flush=True)
    before = dte(b, "$STATR#000;")
    print("STATR avant SATVF:", arts(before), flush=True)
    print("SATVF force=1:", dte(b, "$SATVF#001;1", wait_s=20.0), flush=True)
    time.sleep(1.0)
    after = dte(b, "$STATR#000;")
    a = arts(after)
    print("STATR apres SATVF:", a, flush=True)
    names = {0: "LDA2", 1: "LDK", 2: "VLDA4", 3: "inconnue"}
    if "ART13" in a:
        print(f"RESUME: ART12=0x{int(a.get('ART12', '0')):X} ART13={a['ART13']} ({names.get(int(a['ART13']), '?')})",
              flush=True)
    else:
        print("RESUME: ART12/ART13 absents du STATR (firmware anterieur a c5de1408 ?)", flush=True)
    i = b.mark(); b.send("%OP"); print("OP:", b.wait(r"%OP OK", 10, since=i), flush=True)
    if b.rf_seen:
        print("ATTENTION emission vue:", b.rf_seen, flush=True)
    b.close()


if __name__ == "__main__":
    main()
