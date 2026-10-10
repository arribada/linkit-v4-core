#!/usr/bin/env python3
"""sat_dx1_smoke.py — banc GPS + satellite : Doppler de surface, trames DX1 et fix GNSS sur une vraie carte KIM.

Phase A (UNP26=0, SWS_SENSOR_ENABLE_TX_MODE=OFF) : un cycle plongee/surface de reference. Attendu : trame Doppler historique
  (00 + batterie + 00, completee par le driver a la taille de la modulation). Sert aussi a savoir
  si le KIM2 emet REELLEMENT ("TX EMITTED OK") : dans ce cas la phase B est sautee, car le
  decodeur plateforme non patche lirait les trames DX1 comme de fausses positions.
Phase B (UNP26=2 FORCE, UNDERWATER_EN=1) : N cycles. FORCE et non AUTO (1, defaut) : AUTO
  n'emet DX1 que sur un firmware compile avec DX1_GROUND_DECODER_READY=1, sinon trame historique. Chaque AT+TX est decode contre le contrat
  tools/dx1 (dx1_ref.py) et compare a ce que la sequence %DIVE/%SURFACE doit produire.

Cross-plateforme (Windows natif ou WSL) via Board.find_port(). Prerequis : firmware
--bench --validation. Duree ~25 min. Usage : sat_dx1_smoke.py [--cycles N] [--allow-rf]
"""
import argparse
import os
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "..", "tools", "dx1"))
from gnss_bench import Board  # noqa: E402
import dx1_ref as R  # noqa: E402

BASE = ("GNP02=0,GNP20=0,GNP05=120,GNP09=300,GNP24=1,GNP52=4294967295,UNP20=0,"
        "GNP40=7,LBP02=0,LBP12=0,GNP45=0,ARP54=0,UNP01=1")
DIVE_S, SURF_S = 120, 100

RX_ATTX = re.compile(r"AT\+TX=([0-9A-Fa-f]+)\)")
RX_PREWARM = re.compile(r"PREWARM mode=(\S+) sz=(\d+)")
RX_EMIT = re.compile(r"TX EMITTED OK")
RX_TXFAIL = re.compile(r"TX failed status=(-?\d+)|\+ERROR=(\d+)")
RX_FIX = re.compile(r"GNSS time persisted|reset ntry=\d+->0 \(PVT", re.I)
RX_NOSYNC = re.compile(r"failed to sync", re.I)
RX_SURF = re.compile(r"%SURFACE OK surfaced")


def decode(hexs):
    b = bytes.fromhex(hexs)
    out = {"len_bits": len(b) * 8, "byte0": b[0]}
    if b[0] != 0x00 or len(b) < 3:
        out["kind"] = "not-doppler"
        return out
    out["batt_mv"] = 2700 + 20 * (b[1] >> 1)
    ver = b[2] >> 6
    out["ver"] = ver
    if ver == 0:
        out["kind"] = "legacy" if not any(b[2:]) else "legacy-dirty-tail"
        return out
    out["kind"] = "dx1"
    out["dry"], out["dend"] = (b[2] >> 2) & 7, b[2] & 3
    if len(b) >= 4:
        out["crc_ok"] = R.crc8(b[:3]) == b[3]
    if len(b) == 16:
        word, pos = int.from_bytes(b, "big"), 32
        for name, w in R.RICH_LAYOUT:
            out[name] = (word >> (128 - pos - w)) & ((1 << w) - 1)
            pos += w
        for f in ("s_last", "d_prev", "d_max"):
            out[f + "_s"] = R.em_dec(out[f], 4, 3)
        out["tail_zero"] = not any(b[14:])
    return out


RX_PARMW = re.compile(r"\$([ON]);PARMW#([0-9A-Fa-f]{3});([^\r]*)")


def cfg(b, params):
    """PARMW en ConfigurationState, puis retour en Operational. Echoue BRUYAMMENT si la balise refuse une clef.

    Une clef inconnue ou une valeur hors plage n'invalide pas la trame : la balise applique les autres
    clefs et repond `$N;PARMW#LLL;CLEF1,CLEF2` (dte_handler.cpp, PARMW_REQ). Une erreur de trame repond
    `$N;PARMW#001;<code>` (4 = DATA_LENGTH_MISMATCH, pas une erreur de plage). Sans ce controle, une
    clef renommee (ARP47 -> UNP26) passait en silence et la phase B tournait sur la valeur stockee.
    """
    i = b.mark(); b.send("%CFG"); b.wait(r"%CFG OK", 8, since=i); time.sleep(2.0)
    i = b.mark(); b.send(f"$PARMW#{len(params):03X};{params}")
    # La reponse DTE finit par \r, pas \n : le lecteur n'en fait une ligne qu'a l'arrivee du \n suivant
    # (au plus tard la sortie de %OP). D'ou search() et non match(), et la relecture apres %OP.
    b.wait(r"\$[ON];PARMW#", 6, since=i); time.sleep(1.5)
    j = b.mark(); b.send("%OP"); b.wait(r"%OP OK", 10, since=j); time.sleep(4.0)
    m = next((m for m in (RX_PARMW.search(l) for _, l in b.since(i)) if m), None)
    if m is None:
        b.close()
        raise SystemExit(f"cfg: no PARMW reply for {params!r} -- configuration NOT verified, run aborted")
    if m.group(1) == "N":
        detail = m.group(3).strip()
        what = (f"frame error code {detail}, nothing applied" if detail.isdigit()
                else f"rejected key(s) {detail}, the other keys WERE applied")
        b.close()
        raise SystemExit(f"cfg: PARMW {what} for {params!r} -- run aborted")


def cycle(b, label):
    i = b.mark(); b.send("%DIVE"); b.wait(r"%DIVE OK", 8, since=i)
    time.sleep(DIVE_S)
    i = b.mark(); b.send("%SURFACE")
    t_surf = time.monotonic() - b.t0
    if b.wait(r"%SURFACE OK surfaced", 10, since=i):
        t_surf = next((t for t, l in b.since(i) if RX_SURF.search(l)), t_surf)
    time.sleep(SURF_S)
    res = {"label": label, "tx": [], "prewarm": None, "emitted": 0, "txfail": 0, "fix_s": None, "nosync": 0}
    for t, l in b.since(i):
        m = RX_ATTX.search(l)
        if m:
            res["tx"].append((round(t - t_surf, 1), m.group(1).upper()))
        m = RX_PREWARM.search(l)
        if m and not res["prewarm"]:
            res["prewarm"] = (m.group(1), int(m.group(2)), round(t - t_surf, 1))
        if RX_EMIT.search(l):
            res["emitted"] += 1
        if RX_TXFAIL.search(l):
            res["txfail"] += 1
        if RX_FIX.search(l) and res["fix_s"] is None:
            res["fix_s"] = round(t - t_surf, 1)
        if RX_NOSYNC.search(l):
            res["nosync"] += 1
    print(f"[{label}] prewarm={res['prewarm']} tx={len(res['tx'])} emitted={res['emitted']} "
          f"txfail={res['txfail']} fix_s={res['fix_s']} nosync={res['nosync']}", flush=True)
    for dt, hx in res["tx"]:
        print(f"    +{dt:5.1f}s AT+TX={hx}  {decode(hx)}", flush=True)
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cycles", type=int, default=4)
    ap.add_argument("--allow-rf", action="store_true", help="run phase B even if the KIM2 really emits")
    a = ap.parse_args()
    b = Board()
    print("port:", b.port, flush=True)
    time.sleep(1.0)
    i = b.mark(); b.send("%AMODE 5"); print("AMODE:", b.wait(r"%AMODE mode=", 6, since=i), flush=True)

    cfg(b, BASE + ",UNP26=0")
    ra = cycle(b, "A0_legacy")
    rf = ra["emitted"] > 0
    ok_a = all(decode(h)["kind"] == "legacy" for _, h in ra["tx"]) and len(ra["tx"]) > 0
    print(f"PHASE A: legacy_frames_ok={ok_a} rf_emitting={rf}", flush=True)
    if rf and not a.allow_rf:
        print("PHASE B SKIPPED: the KIM2 really emits; DX1 frames would reach the unpatched platform decoder.",
              flush=True)
        b.close()
        return

    cfg(b, BASE + ",UNP26=2")
    rows = [cycle(b, f"B{n}_dx1") for n in range(1, a.cycles + 1)]
    seqs = []
    for r in rows:
        ds = [decode(h) for _, h in r["tx"]]
        dx = [d for d in ds if d.get("kind") == "dx1"]
        crc = all(d.get("crc_ok", True) for d in dx)
        seqs.append([d.get("surf_seq") for d in dx])
        print(f"SUMMARY {r['label']}: dx1_frames={len(dx)}/{len(ds)} crc_ok={crc} "
              f"surf_seq={[d.get('surf_seq') for d in dx]} s_last_s={[d.get('s_last_s') for d in dx]} "
              f"d_prev_s={[d.get('d_prev_s') for d in dx]} dend={[d.get('dend') for d in dx]} "
              f"latch={[d.get('latch') for d in dx]}", flush=True)
    print("EXPECTED: after the first real dive edge, SURF_SEQ rises by 1 per cycle, S_LAST ~ the surface "
          f"duration (~{SURF_S}+ s, lower-bound decoded), D_PREV ~ {DIVE_S} s, DEND 1, LATCH 1, CRC ok, "
          "all pings of one burst identical except battery/health.", flush=True)
    print("DONE", flush=True)
    b.close()


if __name__ == "__main__":
    main()
