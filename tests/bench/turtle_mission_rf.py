#!/usr/bin/env python3
"""turtle_mission_rf.py -- mission tortue de bout en bout, EMISSION ARGOS REELLE.

Carte : LinkIt V4 + module satellite SMD en UART-AT, firmware
`build_linkitv4_smd.sh --bench --validation --uart`, antenne branchee, maitresse
RCONF provisionnee et relue LDA2 (ART13=0), modulation adaptative OFF. La carte
voit le ciel. Les electrodes SWS sont seches : plongees et emersions sont
injectees par la console de banc (%DIVE / %SURFACE, bench_console.cpp).

EMISSION REELLE : le script refuse de tourner sans DTE_RF_AUTORISE=1 (qui leve
le garde RF de gnss_bench.Board). Il borne lui-meme le nombre de trames par
scenario (--max-tx) et coupe la radio (%AMODE 0) des que la borne est franchie,
puis remet ARGOS_MODE=0 en flash a la fin (sauf --leave-armed).

Tout est lu sur la console USB, dans les lignes que le firmware de validation
emet a niveau INFO (compilees a DEBUG_LEVEL=3, verifiees par
tools/check_bench_log_patterns.py via _fw()) :
  - "[VAL-TX] SmdSatCmdAt::initiate_tx: AT+TX=<hex>)"  la trame REELLEMENT remise
    au module (smd_sat_cmd_at.cpp:557), decodee ici octet par octet ;
  - "[VAL-TX] type=... spacing_s=... burst=... dop_count=..."  fin d emission
    REUSSIE (argos_tx_service.cpp:3064, react(TxComplete) ; sur SMD TxComplete
    n est notifie que sur MAC_TX_DONE, smd_sat.cpp state_transmitting) ;
  - "SmdSat: TXSTART ms=... uptime=N"  (bench) + "%SURFACE OK surfaced uptime=N"
    -> latence emersion -> emission mesuree par l horloge du firmware ;
  - "ArgosTxService::react: KineisEventDeviceError (consecutive=" = echec.
NB : "TX START"/"TX SUCCESS" (argos_tx_service.cpp:3027-3043) sont KIM2
SEULEMENT (#if !ARGOS_SMD) : ils n existent pas sur cette carte.

Sous-commandes
==============
  burst  Profil tortue (template_conf/turtle_gps.cfg converti par
         tools/check_template_conf.to_wire : SURFACING_BURST, adaptatif 0, pile 4,
         NTRY 0, salve 5/10/60 s x6, UW 1, GNSS du template) + gardes de banc
         (LB_THRESHOLD=0, LB_CRITICAL_THRESH=0 : la "batterie" du banc est le rail
         USB ; UNP26=0 ; BLIND 0). Une plongee (--dive-s, 120 s) puis emersion et
         observation (--observe-s, 420 s). Verifie :
           B1 ping 1 = Doppler PREWARM (argos_tx_service.cpp:2596-2634), trame
              historique 4 o sur LDA2 : 00 | BATT7.LB | 00 | 00 (bourrage LDA2,
              modulation_guard.hpp frame_for_modulation), "PREWARM mode=LDA2 sz=24" ;
           B2 latence %SURFACE -> TXSTART (horloge firmware) et -> AT+TX (hote) ;
           B3 espacement Doppler k->k+1 >= min(init+(k-1)*step, max)
              (schedule_surfacing_burst, argos_tx_service.cpp:750-764 ; mesure entre
              deux AT+TX, donc + duree d emission + demarrage : borne haute +12 s) ;
           B4 phase Doppler bornee par SURFACING_BURST_MAX_MSG (dop_count <= 6,
              argos_tx_service.cpp:711-725 et la defense 1123-1139) ;
           B5 fix GNSS reel -> "GNSS fix acquired after" -> premiere trame GNSS
              (type=gnss) : SHORT 12 o sur LDA2 (1 entree eligible -> 1 position,
              build_gnss_packet argos_packet_builder.cpp:432-450), lat/lon decodees,
              non NO_FIX, coherentes entre elles et avec --ref-lat/--ref-lon si
              donnes ; date (jour/heure/minute) a +-15 min de l UTC hote ;
              espacee d au moins init_s de la precedente (garde d espacement 799) ;
           B6 NTRY=0 en SURFACING_BURST = UNE emission par fix (depth_pile.cpp:216-
              218) -> "burst finished" puis battement de presence au TR_NOM
              (+-5 s de gigue, schedule_surfacing_heartbeat 677-685) ;
           B7 toutes les trames ont abouti ([VAL-TX] type= pour chaque AT+TX, aucune
              KineisEventDeviceError, aucun remboursement de credit).
         Les pings Doppler >= 2 portent la DERNIERE position en cache (SHORT
         "short-cached", argos_tx_service.cpp:2880-2953 -- sans limite d age) :
         ils sont decodes et rapportes, pas comptes comme trames GNSS.

  pile   Rotation de la pile de profondeur (ARGOS_DEPTH_PILE=4), positions
         INJECTEES (%GPS, coordonnees distinctes : base + k*0.01 deg) pour que
         chaque trame dise QUELLES positions elle porte. GNSS_TRIGGER_ON_SURFACED=0
         et DLOC_ARG_NOM=24 h pendant le scenario (restaures a la fin) pour que le
         vrai recepteur ne glisse pas de positions parasites ; s il en glisse,
         elles sont etiquetees REAL et traitees en joker. TR_NOM=30 (minimum DTE)
         et gigue 0 pour une cadence previsible. Trois passes (--runs) :
           sb0   SURFACING_BURST, NTRY=0 (le template) : c1 normal ; c2 plongee
                 AVANT l emission GNSS ; c3 normal.
           sbn   SURFACING_BURST, NTRY=--ntry (3) : --cycles (5) cycles, --frames
                 (2) trames GNSS par emersion puis plongee (emersion courte).
           legn  LEGACY, NTRY=--ntry : la MEME sequence (fix injecte a l emersion,
                 2 trames, plongee).
           leg0  (option) LEGACY NTRY=0 = illimite.
         Le verdict ne repose pas sur une sequence ecrite a la main : un MODELE
         Python de DepthPile (portage ligne a ligne de depth_pile.hpp:46-270) est
         pilote par les lignes du firmware lui-meme -- "GPS FIX stored,
         burst_counter=N" (stockage + credit), "NO_FIX dedup", "purged N degraded",
         "build_gnss_packet: SHORT|LONG packet, N position(s)" (un retrieve()),
         "TX ended before reaching the air" (remboursement), "entry:
         OperationalState" (service_init vide la pile, argos_tx_service.cpp:122) --
         et chaque trame AT+TX decodee doit porter EXACTEMENT les positions que le
         modele predit, dans l ordre du LONG (plus recente d abord). Les cliches
         %PILE (type:credit, bench_dump_gps) sont compares au modele.
         m_retrieve_index N EST PAS remis a zero par clear() (depth_pile.hpp:46 ;
         seul remove_if le remet, 125-127) : la phase de la rotation depend de
         l historique. Le modele garde donc les max_index hypotheses de phase
         (2 pour une pile de 4) et ne retient que celles que les trames confirment.

         Semantique derivee du code (pile 4 -> max_messages=3, max_index=2) :
           creneau 0 = les 3 entrees les plus recentes, creneau 1 = les 3
           precedentes (ici la plus ancienne seule) ; retrieve() part de
           m_retrieve_index % 2, saute un creneau sans credit, prend TOUTES les
           entrees creditees du premier creneau eligible (1 -> SHORT, >1 -> LONG,
           <=3), debite 1 credit chacune.
           Credit initial (depth_pile.cpp:205-221) :
             LEGACY/DUTY_CYCLE/PASS_PREDICTION : NTRY=N -> N ; NTRY=0 -> illimite
             SURFACING_BURST (et DOPPLER)       : NTRY=N -> N ; NTRY=0 -> 1 (!)
           Donc avec le template tortue (NTRY=0) la pile ne RE-emet jamais une
           position deja partie : elle ne sert que d arriere pour les fix pas
           encore emis (emersion trop courte). La rotation "a la LEGACY" n existe en
           SURFACING_BURST qu avec NTRY>=2. C est une difference VOULUE (commentaire
           depth_pile.cpp:216 "Surfacing burst: 0 means send once per fix"), pas un
           defaut : le script la rapporte, il ne la force pas au vert.
           Autres differences voulues SB vs LEGACY : NO_FIX ignores en phase
           Doppler (argos_tx_service.cpp:1364-1376), dedupliques sinon, purges au
           premier vrai fix (1446-1456) ; en LEGACY ils occupent leur creneau de
           grille (0xFF). En SB la phase GNSS s arrete quand plus rien n est
           eligible (774-781) ; LEGACY se met en attente (943-950).
         Sequences attendues (modele, --selftest les recalcule ; P1..P5 = fix
         injectes ; [a,b] = LONG plus recent d abord ; phase 0/1 = m_retrieve_index
         pair/impair au depart) :
           sb0                          : [P1] ; (P2 garde son credit) ; [P3,P2]
           sbn/legn NTRY=3, 2 trames/emersion, phase 0 :
             c1 [P1] [P1]  c2 [P2,P1] [P2]  c3 [P3,P2] [P3]  c4 [P4,P3] [P4]
             c5 [P5,P4] [P5]
           phase 1 : identique (sauter un creneau vide realigne la parite ; les
             deux hypotheses restent donc ouvertes, ce qui n invalide rien)
           leg0 = LEGACY NTRY=0 (illimite), phases 0 et 1 :
             c1 [P1] [P1]  c2 [P2,P1] [P2,P1]  c3 [P3,P2,P1] [P3,P2,P1]
             c4 [P1] [P4,P3,P2]  c5 [P2] [P5,P4,P3]   <- l alternance LEGACY :
             creneau 1 (le plus ancien) puis creneau 0 (les 3 plus recents)
         (Ces lignes sont celles que --selftest affiche ; s il y a ecart entre ce
         texte et la sortie de --selftest, c est --selftest qui fait foi.)

  dx1    UNP26=2 (FORCE) sur le profil tortue. --cycles (3) cycles plongee
         (--dive-s) / emersion, plongee des que le ping 1 est parti (une trame
         par cycle). Chaque ping 1 doit etre une vue DX1 de 4 o sur LDA2 :
         00 | BATT7.LB | VER=01 EXT=0 DRY DEND | CRC8(octets 0..2) (dx1_ref.py,
         tools/dx1/README.md), "PREWARM mode=LDA2 sz=32", CRC valide, DEND=1
         (fin de plongee normale : ni dive-timeout ni surface forcee, DEND
         rafraichi a l envoi, argos_tx_service.cpp:2614-2621), DRY=0 (moins de 12 h
         d observation). LATCH/SURF_SEQ/S_LAST/D_PREV ne sont PAS dans la vue LDA2
         (bloc riche = vue LDK 16 o seulement) : SKIP explicite, pas un PASS.

  --plan       ordre, durees, budget RF, cas dte_campaign a jouer avec
               DTE_RF_AUTORISE=1 (limiteur, echouage, refroidissement, modes).
  --selftest   hors ligne : decodeur contre les vecteurs du VRAI constructeur C++
               (argos_tx_test.cpp + un banc compile depuis argos_packet_builder.cpp),
               modele de pile, et rejeu d un journal synthetique par l analyse.
  --replay LOG --scenario X   re-analyse un journal gnss_*.log deja capture.

Sorties : turtle_mission_rf_<horodatage>.jsonl et .csv a cote du script (une
ligne par trame et par verdict), journal brut dans tests/bench/logs/gnss_*.log.

Exemples (Windows, cmd) :
    set PYTHONUTF8=1
    set DTE_RF_AUTORISE=1
    python tests\\bench\\turtle_mission_rf.py --plan
    python tests\\bench\\turtle_mission_rf.py burst --ref-lat 43.61 --ref-lon 3.87
"""
import argparse
import csv
import datetime
import json
import os
import re
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, "tools"))
sys.path.insert(0, os.path.join(REPO, "tools", "dx1"))
import bench_platform as P  # noqa: E402
import dx1_ref as DX1  # noqa: E402

TEMPLATE = os.path.join(REPO, "template_conf", "turtle_gps.cfg")
DTE_PARAMS = os.path.join(REPO, "core", "protocol", "dte_params.cpp")
UINT_MAX = 0xFFFFFFFF


def _fw(lit):
    """Texte LITTERAL d une ligne firmware, echappe pour une regex.

    tools/check_bench_log_patterns.py verifie chaque litteral passe a _fw contre les
    sources, et qu il n est pas DEBUG_TRACE seulement (compile hors du binaire a
    DEBUG_LEVEL=3)."""
    return re.escape(lit)


# ============================================================================
#  Motifs firmware (tous INFO/WARN/ERROR, verifies par check_bench_log_patterns)
# ============================================================================
RX_ATTX = re.compile(_fw("[VAL-TX] SmdSatCmdAt::initiate_tx: AT+TX=") + r"([0-9A-Fa-f]+)\)")
RX_VALTX = re.compile(_fw("[VAL-TX] type=") + r"(\S+) t=(\d+) spacing_s=(\d+) burst=(\w+) dop_count=(\d+)")
RX_TXSTART = re.compile(_fw("SmdSat: TXSTART ms=") + r"\S* uptime=(\d+)")
RX_DEVERR = re.compile(_fw("ArgosTxService::react: KineisEventDeviceError (consecutive="))
RX_REFUND = re.compile(_fw("ArgosTxService: TX ended before reaching the air") + r"\D+(\d+) depth-pile credit")
RX_PREWARM = re.compile(_fw("process_doppler_burst: PREWARM mode=") + r"(\S+) sz=(\d+)")
RX_BUILD = re.compile(_fw("ArgosPacketBuilder::build_gnss_packet: ") + r"(SHORT|LONG) packet, (\d+) position")
RX_NOELIG = re.compile(_fw("ArgosTxService::process_gnss_burst: no entries eligible in depth pile"))
RX_STORED = re.compile(r"GPS (FIX|NO_FIX heartbeat) " + _fw("stored, burst_counter=") + r"(\d+)")
RX_NOFIX_CACHED = re.compile(_fw("DepthPileManager::notify_peer_event: GNSS NO_FIX cached"))
RX_NOFIX_DEDUP = re.compile(_fw("DepthPileManager: NO_FIX dedup"))
RX_PURGED = re.compile(_fw("ArgosTxService::notify_peer_event: purged ") + r"(\d+) degraded entr\w+ from depth pile "
                       r"\(no_fix_included=(\d)\)")
RX_FIXACQ = re.compile(_fw("ArgosTxService::SURFACING_BURST: GNSS fix acquired after ") + r"(\d+)")
RX_GNSS1 = re.compile(_fw("ArgosTxService::SURFACING_BURST: GNSS TX #1 (immediate after fix)"))
RX_BURSTEND = re.compile(_fw("ArgosTxService: burst finished, waiting for the next surfacing"))
RX_DOPLIMIT = re.compile(_fw("ArgosTxService::SURFACING_BURST: Doppler limit reached ") + r"\((\d+)/(\d+)\)")
RX_OPENTRY = re.compile(_fw("entry: OperationalState"))
RX_BOOTENTRY = re.compile(_fw("entry: BootState"))
RX_PVTFIX = re.compile(_fw("first valid GPS fix this session"))
RX_RTCSYNC = re.compile(_fw("M10QAsyncReceiver: GNSS time persisted"))
# Console de banc (bench_console.cpp) -- pas des logs firmware, motifs libres.
RX_SURF = re.compile(r"%SURFACE OK surfaced uptime=(\d+)")
RX_DIVE = re.compile(r"%DIVE OK underwater uptime=(\d+)")
RX_GPSOK = re.compile(r"%GPS OK lat=(-?[\d.]+) lon=(-?[\d.]+)")
RX_PILE = re.compile(r"%PILE (empty|[\d: ]+)")
RX_MARK = re.compile(r"\[turtle\] (RUN|CYCLE|END|CFG|NOTE) ?(.*)")
RX_LINKLOST = re.compile(r"\[bench\] lien s.rie perdu")


# ============================================================================
#  Decodeur de trames (miroir de argos_packet_builder.cpp)
# ============================================================================
LAT_NONE = (1 << 21) - 1
LON_NONE = (1 << 22) - 1
DTL_MIN = {1: 10, 2: 15, 3: 30, 4: 60, 5: 120, 6: 180, 7: 240, 8: 360, 9: 720, 10: 1440,
           11: 1, 12: 2, 13: 5, 14: 20, 15: 45}   # BaseDeltaTimeLoc (base_types.hpp:668)


class _Reader:
    """Lecture MSB d abord, comme PACK_BITS / extract_bits (bitpack.hpp)."""

    def __init__(self, data):
        self.v = int.from_bytes(data, "big")
        self.n = len(data) * 8
        self.pos = 0

    def get(self, w):
        val = (self.v >> (self.n - self.pos - w)) & ((1 << w) - 1)
        self.pos += w
        return val


def dec_lat(raw):
    if raw == LAT_NONE:
        return None
    mag = raw & 0xFFFFF
    return -mag / 1e4 if (raw >> 20) & 1 else mag / 1e4


def dec_lon(raw):
    if raw == LON_NONE:
        return None
    mag = raw & 0x1FFFFF
    return -mag / 1e4 if (raw >> 21) & 1 else mag / 1e4


def _pos(lat_raw, lon_raw):
    if lat_raw == LAT_NONE and lon_raw == LON_NONE:
        return None          # NO_FIX (0xFF...) ou creneau absent
    return (dec_lat(lat_raw), dec_lon(lon_raw))


def decode_short(b):
    """SHORT 96 b (argos_packet_builder.cpp:77-148)."""
    r = _Reader(b[:12])
    d = {"kind": "short", "hdr": r.get(3), "day": r.get(5), "hour": r.get(5), "minute": r.get(6)}
    lat, lon = r.get(21), r.get(22)
    d["speed"], d["ooz"], d["heading"], d["alt"] = r.get(7), r.get(1), r.get(8), r.get(8)
    d["batt_mv"] = 2700 + 20 * r.get(7)
    d["lb"] = r.get(1)
    d["pad"] = r.get(2)
    p = _pos(lat, lon)
    d["positions"] = [p]
    d["nofix"] = p is None
    d["ok"] = d["hdr"] == 0 and d["pad"] == 0 and not any(b[12:])
    return d


def decode_long(b):
    """LONG 192 b sur LDA2 (argos_packet_builder.cpp:340-430), CRC8 a l octet 23."""
    r = _Reader(b[:24])
    d = {"kind": "long", "hdr": r.get(3), "day": r.get(5), "hour": r.get(5), "minute": r.get(6)}
    lat0, lon0 = r.get(21), r.get(22)
    d["speed"], d["ooz"] = r.get(7), r.get(1)
    d["batt_mv"] = 2700 + 20 * r.get(7)
    d["lb"] = r.get(1)
    d["dtl"] = r.get(4)
    d["dtl_min"] = DTL_MIN.get(d["dtl"])
    slots = [_pos(lat0, lon0)]
    for _ in range(2):
        la, lo = r.get(21), r.get(22)
        slots.append(_pos(la, lo))
    d["tail"] = r.get(16)
    d["crc_ok"] = DX1.crc8(bytes(b[:23])) == b[23]
    d["slots"] = slots
    # Sans le nombre de positions (ligne build_gnss_packet), un creneau 0xFF est
    # ambigu : absent ou NO_FIX. Par defaut on coupe les creneaux vides en queue.
    pos = list(slots)
    while len(pos) > 1 and pos[-1] is None:
        pos.pop()
    d["positions"] = pos
    d["ok"] = d["hdr"] == 0 and d["tail"] == 0 and d["crc_ok"]
    return d


def decode_doppler4(b):
    """Doppler sur LDA2 : 3 o historiques bourres a 4, ou vue DX1 de 4 o."""
    d = {"batt_mv": 2700 + 20 * (b[1] >> 1), "lb": b[1] & 1, "ver": b[2] >> 6}
    if d["ver"] == 0:
        d["kind"] = "doppler"
        d["ok"] = b[0] == 0 and b[2] == 0 and b[3] == 0
    else:
        d["kind"] = "dx1"
        d["ext"] = (b[2] >> 5) & 1
        d["dry"] = (b[2] >> 2) & 7
        d["dend"] = b[2] & 3
        d["crc_ok"] = DX1.crc8(bytes(b[:3])) == b[3]
        d["ok"] = b[0] == 0 and d["ver"] == 1 and d["ext"] == 0 and d["crc_ok"]
    d["positions"] = []
    return d


def decode_frame(hexs, master="LDA2"):
    """Decode la charge AT+TX telle que remise au module. Tailles sur LDA2 :
    Doppler 4 o, SHORT 12 o, LONG/fastloc/CloudLocate MEAS20/capteur 24 o."""
    try:
        b = bytes.fromhex(hexs)
    except ValueError:
        return {"kind": "invalid", "ok": False, "positions": []}
    n = len(b)
    d = {"nbytes": n}
    if master == "LDA2":
        if n == 4 and b[0] == 0:
            d.update(decode_doppler4(b))
        elif n == 12:
            d.update(decode_short(b))
        elif n == 24:
            hdr = b[0] >> 5
            if hdr == 0:
                d.update(decode_long(b))
            else:
                d.update({"kind": {1: "sensor", 2: "fastloc", 7: "cloudlocate"}.get(hdr, f"hdr{hdr}"),
                          "ok": DX1.crc8(bytes(b[:23])) == b[23], "positions": []})
        else:
            d.update({"kind": f"unknown{n}", "ok": False, "positions": []})
    else:  # LDK 16 o : Doppler bourre ou SHORT + 4 o nuls (non utilise par ce banc)
        if n == 16 and b[0] == 0 and not any(b[3:]):
            d.update(decode_doppler4(b[:4]))
        elif n == 16 and b[0] == 0 and (b[2] >> 6) == 1:
            d.update(decode_doppler4(b[:4]))
        elif n == 16:
            d.update(decode_short(b))
        else:
            d.update({"kind": f"unknown{n}", "ok": False, "positions": []})
    return d


def packet_datetime(day, hour, minute, ref):
    """Rend la date complete la plus proche de `ref` (UTC) pour jour/heure/minute."""
    best = None
    for dm in (-1, 0, 1):
        y, m = ref.year, ref.month + dm
        if m == 0:
            y, m = y - 1, 12
        elif m == 13:
            y, m = y + 1, 1
        try:
            cand = datetime.datetime(y, m, day, hour, minute, tzinfo=datetime.timezone.utc)
        except ValueError:
            continue
        if best is None or abs((cand - ref).total_seconds()) < abs((best - ref).total_seconds()):
            best = cand
    return best


# ---- encodeur miroir (pour l auto-test et le journal synthetique UNIQUEMENT) --
def _enc_lat(x):
    return int(x * 10000) if x >= 0 else (int((x - 0.00005) * -10000) | (1 << 20))


def _enc_lon(x):
    return int(x * 10000) if x >= 0 else (int((x - 0.00005) * -10000) | (1 << 21))


def _enc_batt(mv):
    return min(127, max(mv - 2700, 0) // 20)


def _enc_speed(x):
    return 0 if x < 0 else min(127, int((3600 * x) / (2 * 1000000)))


def _enc_head(x):
    return 0 if x < 0 else min(254, int(x * (1.0 / 1.42)))


def _enc_alt(x):
    return int(min(254.0, max(0.0, x / 40000)))


def _put(bits, v, w):
    bits.put(v & ((1 << w) - 1), w)


def encode_short(e, ooz=0, lb=0):
    t = datetime.datetime.fromtimestamp(e["t"], datetime.timezone.utc)
    bits = DX1.Bits(12)
    for v, w in ((0, 3), (t.day, 5), (t.hour, 5), (t.minute, 6)):
        _put(bits, v, w)
    if e["valid"]:
        _put(bits, _enc_lat(e["lat"]), 21)
        _put(bits, _enc_lon(e["lon"]), 22)
        _put(bits, _enc_speed(e.get("gspeed", 0)), 7)
        _put(bits, ooz, 1)
        _put(bits, _enc_head(e.get("head", 0)), 8)
        _put(bits, _enc_alt(e.get("hmsl", 0)) if e.get("fix3d") else 255, 8)
    else:
        for v, w in ((UINT_MAX, 21), (UINT_MAX, 22), (0xFF, 7), (ooz, 1), (0xFF, 8), (0xFF, 8)):
            _put(bits, v, w)
    _put(bits, _enc_batt(e.get("batt", 3700)), 7)
    _put(bits, lb, 1)
    return bytes(bits.b)


def encode_long(entries_newest_first, ooz=0, lb=0, dtl=1):
    e0 = entries_newest_first[0]
    t = datetime.datetime.fromtimestamp(e0["t"], datetime.timezone.utc)
    bits = DX1.Bits(24)
    for v, w in ((0, 3), (t.day, 5), (t.hour, 5), (t.minute, 6)):
        _put(bits, v, w)
    if e0["valid"]:
        _put(bits, _enc_lat(e0["lat"]), 21)
        _put(bits, _enc_lon(e0["lon"]), 22)
        _put(bits, _enc_speed(e0.get("gspeed", 0)), 7)
    else:
        for v, w in ((UINT_MAX, 21), (UINT_MAX, 22), (0xFF, 7)):
            _put(bits, v, w)
    for v, w in ((ooz, 1), (_enc_batt(e0.get("batt", 3700)), 7), (lb, 1), (dtl, 4)):
        _put(bits, v, w)
    for i in (1, 2):
        e = entries_newest_first[i] if i < len(entries_newest_first) else None
        if e is None or not e["valid"]:
            _put(bits, UINT_MAX, 21)
            _put(bits, UINT_MAX, 22)
        else:
            _put(bits, _enc_lat(e["lat"]), 21)
            _put(bits, _enc_lon(e["lon"]), 22)
    out = bytearray(bits.b)
    out[23] = DX1.crc8(bytes(out[:23]))
    return bytes(out)


# ============================================================================
#  Modele de la pile (portage de depth_pile.hpp:20-270)
# ============================================================================
class PileModel:
    """DepthPile<GPSLogEntry>, ligne a ligne. Entrees : [label, credit, kind]."""

    def __init__(self, retrieve_index=0, max_size=24):
        self.e = []
        self.max = max_size
        self.ri = retrieve_index
        self.evictions = 0
        self.last = []

    def clear(self):                    # depth_pile.hpp:46 -- ri N EST PAS remis a 0
        self.e = []

    def set_max_size(self, n):          # :53-60
        self.max = max(1, n)
        while len(self.e) > self.max:
            self.e.pop(0)
            self.evictions += 1

    def store(self, label, kind, credit):   # :62-69
        self.e.append([label, credit, kind])
        if len(self.e) > self.max:
            self.e.pop(0)
            self.evictions += 1

    def replace_last_nofix(self, label, credit):   # store_or_replace_last :76-85
        if self.e and self.e[-1][2] == "nofix":
            self.e[-1] = [label, credit, "nofix"]
            return True
        self.store(label, "nofix", credit)
        return False

    def remove_kinds(self, kinds):      # remove_if :114-129
        before = len(self.e)
        self.e = [x for x in self.e if x[2] not in kinds]
        if len(self.e) != before:
            self.ri = 0
        return before - len(self.e)

    def eligible(self):
        return sum(1 for x in self.e if x[1])

    def retrieve(self, depth, max_messages=3):     # :222-270
        max_messages = min(depth, max_messages)
        max_index = (depth + (max_messages - 1)) // max_messages
        size = len(self.e)
        span = min(max_messages, size)
        max_msg_index = self.ri + max_index
        retrieve_index = 0
        eligible = 0
        first = None
        while self.ri < max_msg_index and not eligible:
            retrieve_index = self.ri % max_index
            for k in range(span):
                idx = size - (span * (retrieve_index + 1)) + k
                if 0 <= idx < size and self.e[idx][1]:
                    eligible += 1
                    if first is None:
                        first = idx
            self.ri += 1
        out = []
        if eligible == 1:
            self.e[first][1] -= 1
            out.append(self.e[first])
        elif eligible > 1:
            for k in range(span):
                idx = size - (span * (retrieve_index + 1)) + k
                if 0 <= idx < size and self.e[idx][1]:
                    self.e[idx][1] -= 1
                    out.append(self.e[idx])
        self.last = out
        return [x[0] for x in out]

    def refund(self):                   # :205-220
        for x in self.last:
            if any(x is y for y in self.e):
                x[1] += 1
        self.last = []

    def dump(self):
        """Format de %PILE : "<type>:<credit>" (0=fix, 1=no-fix), plus ancien d abord."""
        return " ".join(f"{0 if k == 'fix' else 1}:{c & UINT_MAX}" for _, c, k in self.e)


def credit_for(mode, ntry):
    """depth_pile.cpp:205-221."""
    if mode in (1, 2, 3):              # PASS_PREDICTION, LEGACY, DUTY_CYCLE
        return UINT_MAX if ntry == 0 else ntry
    return 1 if ntry == 0 else ntry


def frame_order(labels):
    """build_gnss_packet inverse la liste (plus recent d abord) quand > 1 position."""
    return list(reversed(labels)) if len(labels) > 1 else list(labels)


def simulate(mode, ntry, cycles, depth=4, phase=0):
    """Sequence attendue pour un plan [(n_frames|'dive-before'), ...] : un fix par
    emersion, stocke AVANT le premier retrieve, puis n retrieves."""
    m = PileModel(phase)
    m.set_max_size(depth)
    seq = []
    for k, spec in enumerate(cycles, 1):
        m.store(f"P{k}", "fix", credit_for(mode, ntry))
        frames = []
        if spec != "dive-before":
            for _ in range(spec):
                if mode == 5 and not m.eligible():
                    break
                got = m.retrieve(depth)
                if not got:
                    break
                frames.append(frame_order(got))
        seq.append(frames)
    return seq


# ============================================================================
#  Analyse des journaux (vivant et rejeu partagent ce code)
# ============================================================================
def parse_log_file(path):
    """Journal de gnss_bench.Board : "[  t] texte" ; ">>> " = commandes envoyees."""
    out = []
    rx = re.compile(r"^\[\s*([\d.]+)\] (.*)$")
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            m = rx.match(line.rstrip("\r\n"))
            if m and not m.group(2).startswith(">>> "):
                out.append((float(m.group(1)), m.group(2)))
    return out


def extract_frames(lines, master="LDA2"):
    """Une entree par AT+TX, avec ce qui l entoure : construction (build_gnss_packet,
    PREWARM) juste avant, TXSTART et [VAL-TX] type= (succes) ou DeviceError apres."""
    frames = []
    pend_build = pend_prewarm = None
    cur = None
    for idx, (t, l) in enumerate(lines):
        m = RX_BUILD.search(l)
        if m:
            pend_build = (m.group(1), int(m.group(2)))
            continue
        m = RX_PREWARM.search(l)
        if m:
            pend_prewarm = (m.group(1), int(m.group(2)))
            continue
        m = RX_ATTX.search(l)
        if m:
            if cur is not None and cur["status"] == "pending":
                cur["status"] = "no-completion"
            hx = m.group(1).upper()
            cur = {"t": t, "idx": idx, "hex": hx, "dec": decode_frame(hx, master), "build": pend_build,
                   "prewarm": pend_prewarm, "status": "pending", "val": None, "txstart_up": None}
            if pend_build and cur["dec"]["kind"] == "long":
                cur["dec"]["positions"] = cur["dec"]["slots"][:pend_build[1]]
            frames.append(cur)
            pend_build = pend_prewarm = None
            continue
        if cur is None:
            continue
        m = RX_TXSTART.search(l)
        if m and cur["txstart_up"] is None:
            cur["txstart_up"] = int(m.group(1))
            continue
        m = RX_VALTX.search(l)
        if m and cur["status"] == "pending":
            cur["status"] = "ok"
            cur["val"] = {"type": m.group(1), "t": int(m.group(2)), "spacing_s": int(m.group(3)),
                          "burst": m.group(4), "dop_count": int(m.group(5))}
            continue
        if RX_DEVERR.search(l) and cur["status"] == "pending":
            cur["status"] = "device-error"
    return frames


class Results:
    def __init__(self, sink=None):
        self.rows = []
        self.sink = sink

    def add(self, scen, cid, name, verdict, detail, evidence=()):
        row = {"type": "check", "scenario": scen, "id": cid, "name": name, "verdict": verdict,
               "detail": detail, "evidence": list(evidence)[:6]}
        self.rows.append(row)
        if self.sink:
            self.sink.jsonl(row)
        col = {"PASS": "32", "FAIL": "31", "SKIP": "33"}.get(verdict, "36")
        print(f"  [{P.colour(verdict, col)}] {cid} {name}", flush=True)
        print(f"         {detail}", flush=True)
        for e in list(evidence)[:6]:
            print(f"         | {e}", flush=True)

    def summary(self, scen=None):
        rows = [r for r in self.rows if scen is None or r["scenario"] == scen]
        c = {}
        for r in rows:
            c[r["verdict"]] = c.get(r["verdict"], 0) + 1
        return c


def _fmt_pos(p):
    return "NOFIX" if p is None else f"({p[0]:.4f},{p[1]:.4f})"


def frame_summary(f, t0=0.0):
    d = f["dec"]
    s = f"+{f['t'] - t0:6.1f}s {d['kind']:<8} {f['hex']}"
    if d["kind"] in ("short", "long"):
        s += " pos=" + ",".join(_fmt_pos(p) for p in d["positions"])
    if d["kind"] in ("doppler", "dx1", "short", "long"):
        s += f" batt={d['batt_mv']}mV"
    if d["kind"] == "dx1":
        s += f" dry={d['dry']} dend={d['dend']} crc={'ok' if d['crc_ok'] else 'BAD'}"
    if f["val"]:
        s += f" [{f['val']['type']} dop#{f['val']['dop_count']}]"
    s += f" {f['status']}"
    return s


def _interval(cfg, k):
    """Espacement avant le ping Doppler k+1 (argos_tx_service.cpp:750-753)."""
    return min(cfg["init"] + (k - 1) * cfg["step"], cfg["max"])


# ----------------------------------------------------------------------------
def analyse_burst(lines, cfg, res, ref=None, host_utc_at=None):
    """Analyse d un cycle burst a partir de la ligne %SURFACE OK."""
    scen = "burst"
    i_s = next((i for i, (_, l) in enumerate(lines) if RX_SURF.search(l)), None)
    if i_s is None:
        res.add(scen, "B0", "emersion injectee", "FAIL", "aucune ligne %SURFACE OK")
        return []
    t_s, l_s = lines[i_s]
    up_s = int(RX_SURF.search(l_s).group(1))
    after = lines[i_s:]
    frames = extract_frames(after)
    for f in frames:
        print("    " + frame_summary(f, t_s), flush=True)
    t_fix = next((t for t, l in after if RX_FIXACQ.search(l)), None)
    # Fin de salve APRES le fix : si la phase Doppler a atteint MAX_MSG avant le
    # fix, un premier "burst finished" la precede (le fix relance alors la phase
    # GNSS depuis m_awaiting_surfacing, argos_tx_service.cpp:1461).
    t_end = next((t for t, l in after if RX_BURSTEND.search(l) and (t_fix is None or t > t_fix)), None)
    t_doplim = next((t for t, l in after if RX_DOPLIMIT.search(l)), None)
    if not frames:
        res.add(scen, "B1", "ping 1 = Doppler PREWARM", "FAIL", "aucune trame AT+TX apres l emersion")
        return frames

    # B1 -- ping 1
    f1 = frames[0]
    d1 = f1["dec"]
    if t_fix is not None and t_fix < f1["t"]:
        res.add(scen, "B1", "ping 1 = Doppler PREWARM", "SKIP",
                f"le fix (+{t_fix - t_s:.1f} s) a precede le ping 1 : la salve est passee directement en "
                f"phase GNSS, comme le veut schedule_surfacing_burst (pas de Doppler a emettre)")
    else:
        ok = (f1["prewarm"] is not None and f1["prewarm"][0] == "LDA2" and f1["prewarm"][1] == 24
              and d1["kind"] == "doppler" and d1["ok"] and d1["nbytes"] == 4
              and f1["val"] is not None and f1["val"]["type"] == "doppler-prewarm")
        res.add(scen, "B1", "ping 1 = Doppler PREWARM historique 4 o sur LDA2", "PASS" if ok else "FAIL",
                f"prewarm={f1['prewarm']} trame={f1['hex']} kind={d1['kind']} batt={d1.get('batt_mv')} mV "
                f"lb={d1.get('lb')} val={f1['val'] and f1['val']['type']}")

    # B2 -- latence
    lat_fw = (f1["txstart_up"] - up_s) if f1["txstart_up"] else None
    lat_host = f1["t"] - t_s
    ok = lat_host <= 10.0
    res.add(scen, "B2", "latence emersion -> premiere emission", "PASS" if ok else "FAIL",
            f"firmware %SURFACE->TXSTART = {lat_fw if lat_fw is not None else '?'} ms ; hote %SURFACE->AT+TX = "
            f"{lat_host:.2f} s (borne 10 s ; mesures anterieures 1,7-2,9 s)")

    # Phases
    dop = [f for f in frames if f["val"] and f["val"]["burst"] == "on" and f["val"]["type"] != "gnss"
           and (t_fix is None or f["t"] < t_fix)]
    gnss = [f for f in frames if f["val"] and f["val"]["type"] == "gnss"]
    hb = [f for f in frames if t_end is not None and f["t"] > t_end
          and not (f["val"] and f["val"]["type"] == "gnss")]

    # B3 -- espacement Doppler
    bad = []
    for a, b in zip(dop, dop[1:]):
        k = a["val"]["dop_count"]
        want = _interval(cfg, k)
        dt = b["t"] - a["t"]
        if not (want - 1.0 <= dt <= want + 12.0):
            bad.append(f"ping {k}->{k + 1}: {dt:.1f} s, attendu [{want}, {want + 12}] s")
    seq = [f"{b['t'] - a['t']:.1f}" for a, b in zip(dop, dop[1:])]
    want_seq = [str(_interval(cfg, a["val"]["dop_count"])) for a in dop[:-1]]
    if len(dop) < 2:
        res.add(scen, "B3", "espacement progressif de la phase Doppler", "SKIP",
                f"{len(dop)} ping(s) Doppler seulement avant le fix : rien a espacer")
    else:
        res.add(scen, "B3", "espacement progressif de la phase Doppler", "FAIL" if bad else "PASS",
                f"AT+TX->AT+TX = {seq} s ; consigne init+(k-1)*step plafonne = {want_seq} s", bad)

    # B4 -- borne MAX_MSG
    counts = [f["val"]["dop_count"] for f in frames if f["val"] and f["val"]["burst"] == "on"]
    over = [c for c in counts if c > cfg["max_msg"]]
    res.add(scen, "B4", "phase Doppler bornee par SURFACING_BURST_MAX_MSG", "FAIL" if over else "PASS",
            f"dop_count vus = {counts} (max {cfg['max_msg']})"
            + (f" ; limite atteinte a +{t_doplim - t_s:.1f} s avant tout fix" if t_doplim else ""))

    # B5 -- GNSS
    if t_fix is None:
        res.add(scen, "B5", "fix GNSS reel puis trame GNSS", "FAIL",
                "aucun 'GNSS fix acquired after' dans la fenetre (ciel ? GNSS_TRIGGER_ON_SURFACED ?)")
    elif not gnss:
        res.add(scen, "B5", "fix GNSS reel puis trame GNSS", "FAIL",
                f"fix a +{t_fix - t_s:.1f} s mais aucune trame type=gnss")
    else:
        g = gnss[0]
        d = g["dec"]
        prev = [f for f in frames if f["t"] < g["t"]]
        gap = g["t"] - prev[-1]["t"] if prev else None
        probs = []
        # SHORT (12 o) si la pile ne tient que ce fix ; LONG (24 o) si elle garde un
        # fix precedent encore credite (pile 4, NTRY=2) : la plus recente en tete.
        if d["kind"] not in ("short", "long") or d["nbytes"] not in (12, 24) or not d["ok"]:
            probs.append(f"attendu SHORT 12 o ou LONG 24 o valide, vu {d['kind']} {d['nbytes']} o ok={d.get('ok')}")
        p = d["positions"][0] if d.get("positions") else None
        if p is None:
            probs.append("position NO_FIX dans la premiere trame GNSS")
        else:
            if not (-90 <= p[0] <= 90 and -180 <= p[1] <= 180):
                probs.append(f"position hors domaine {p}")
            if ref:
                km = _dist_km(p, ref)
                if km > ref[2]:
                    probs.append(f"a {km:.2f} km de la reference {ref[:2]} (tolerance {ref[2]} km)")
        if host_utc_at and "day" in d:
            utc = host_utc_at(g["t"])
            pdt = packet_datetime(d["day"], d["hour"], d["minute"], utc)
            if pdt is None or abs((pdt - utc).total_seconds()) > 900:
                probs.append(f"date paquet {d['day']}/{d['hour']}:{d['minute']:02d} loin de l UTC hote {utc:%d %H:%M}")
        if gap is not None and gap < cfg["init"] - 1.0:
            probs.append(f"premiere trame GNSS {gap:.1f} s apres la precedente (< init_s={cfg['init']})")
        others = [f["dec"]["positions"][0] for f in frames
                  if f["t"] >= g["t"] and f["dec"]["kind"] == "short" and f["dec"]["positions"]
                  and f["dec"]["positions"][0] is not None]
        spread = max((_dist_km(p, q) for q in others), default=0.0) if p else 0.0
        if spread > 0.5:
            probs.append(f"les trames posterieures portent une autre position (ecart {spread:.2f} km)")
        res.add(scen, "B5", "fix GNSS reel -> trame GNSS (SHORT/LONG) decodee", "FAIL" if probs else "PASS",
                f"fix a +{t_fix - t_s:.1f} s ; trame +{g['t'] - t_s:.1f} s {g['hex']} -> {_fmt_pos(p)} "
                f"date {d.get('day')}/{d.get('hour')}h{d.get('minute', 0):02d} batt={d.get('batt_mv')} mV "
                f"alt={d.get('alt')} ; ecart avec la precedente {gap if gap is None else round(gap, 1)} s", probs)

    # B6 -- NTRY trames GNSS par fix (NTRY=0 en SURFACING_BURST = 1 credit) puis battement au TR_NOM.
    # Templates tortue a NTRY=2 depuis 5224bde2 : 2 trames pour le meme fix.
    ntry = int(cfg.get("ntry") or 0)
    attendu = max(1, ntry)
    if t_fix is not None and gnss:
        probs = []
        if len(gnss) != attendu:
            probs.append(f"{len(gnss)} trames type=gnss pour un fix, attendu {attendu} (NTRY={ntry} en SURFACING_BURST)")
        if t_end is None:
            probs.append("pas de 'burst finished' apres la phase GNSS")
        sp = [round(b["t"] - a["t"], 1) for a, b in zip(hb, hb[1:])]
        bad = [s for s in sp if not (cfg["tr_nom"] - 6 <= s <= cfg["tr_nom"] + 15)]
        if bad:
            probs.append(f"battements espaces de {sp} s, attendu TR_NOM={cfg['tr_nom']} s +-5 s de gigue")
        res.add(scen, "B6", f"NTRY={ntry} : {attendu} trame(s) GNSS par fix, puis battement de presence au TR_NOM",
                "FAIL" if probs else "PASS",
                f"trames GNSS={len(gnss)} ; battements={len(hb)} types={[f['val']['type'] for f in hb if f['val']]} "
                f"espacements={sp} s", probs)
    else:
        res.add(scen, "B6", "NTRY=0 : une trame GNSS par fix, puis battement", "SKIP", "pas de phase GNSS observee")

    # B7 -- statut des emissions
    bad = [f for f in frames if f["status"] != "ok"]
    nref = sum(1 for _, l in after if RX_REFUND.search(l))
    res.add(scen, "B7", "toutes les emissions ont abouti (MAC_TX_DONE)", "FAIL" if (bad or nref) else "PASS",
            f"{len(frames)} AT+TX, {len(frames) - len(bad)} [VAL-TX] de fin, {len(bad)} en echec, "
            f"{nref} remboursement(s) de credit",
            [frame_summary(f, t_s) for f in bad])
    return frames


def _dist_km(p, q):
    import math
    la1, lo1, la2, lo2 = map(math.radians, (p[0], p[1], q[0], q[1]))
    a = math.sin((la2 - la1) / 2) ** 2 + math.cos(la1) * math.cos(la2) * math.sin((lo2 - lo1) / 2) ** 2
    return 6371.0 * 2 * math.asin(min(1.0, math.sqrt(a)))


# ----------------------------------------------------------------------------
class PileReplay:
    """Pilote les modeles (une hypothese de phase par valeur de m_retrieve_index %
    max_index) avec les lignes du firmware et compare chaque trame."""

    def __init__(self, mode, ntry, depth, injected):
        max_messages = min(depth, 3)
        self.max_index = (depth + max_messages - 1) // max_messages
        self.models = {ph: PileModel(ph) for ph in range(self.max_index)}
        for m in self.models.values():
            m.set_max_size(depth)
        self.alive = set(self.models)
        self.mode, self.ntry, self.depth = mode, ntry, depth
        self.injected = injected          # [(label, lat, lon)]
        self.pending_inj = []
        self.real_n = 0
        self.rows = []                    # (t, expected per phase, observed, ok_by_phase)
        self.credit_bad = []
        self.pile_bad = []
        self.pending_nofix = False

    def label_of(self, p):
        if p is None:
            return "NOFIX"
        for lab, la, lo in self.injected:
            if abs(p[0] - la) < 2e-4 and abs(p[1] - lo) < 2e-4:
                return lab
        return "REAL"

    def _store(self, label, kind, credit):
        for m in self.models.values():
            m.set_max_size(self.depth)
            m.store(label, kind, credit)

    def feed(self, idx, t, l, frame_by_idx):
        if self.pending_nofix:
            self.pending_nofix = False
            if RX_NOFIX_DEDUP.search(l):
                for m in self.models.values():
                    m.replace_last_nofix("NOFIX", credit_for(self.mode, self.ntry))
                return
            if not RX_STORED.search(l):
                # SURFACING_BURST : store_or_replace_last a ajoute sans rien journaliser
                self._store("NOFIX", "nofix", credit_for(self.mode, self.ntry))
        m = RX_GPSOK.search(l)
        if m:
            self.pending_inj.append((float(m.group(1)), float(m.group(2))))
            return
        if RX_OPENTRY.search(l):
            for mm in self.models.values():
                mm.clear()
            return
        if RX_NOFIX_CACHED.search(l):
            self.pending_nofix = True
            return
        m = RX_STORED.search(l)
        if m:
            credit = int(m.group(2))
            want = credit_for(self.mode, self.ntry)
            if credit != want:
                self.credit_bad.append(f"+{t:.1f}s credit={credit} attendu {want} (mode={self.mode} NTRY={self.ntry})")
            if m.group(1) == "FIX":
                if self.pending_inj:
                    la, lo = self.pending_inj.pop(0)
                    lab = self.label_of((la, lo))
                else:
                    self.real_n += 1
                    lab = "REAL"
                self._store(lab, "fix", credit)
            else:
                self._store("NOFIX", "nofix", credit)
            return
        m = RX_PURGED.search(l)
        if m:
            kinds = {"nofix", "cl", "fl"} if m.group(2) == "1" else {"cl", "fl"}
            for mm in self.models.values():
                mm.remove_kinds(kinds)
            return
        if RX_REFUND.search(l):
            for mm in self.models.values():
                mm.refund()
            return
        if RX_NOELIG.search(l):
            exp = {ph: mm.retrieve(self.depth) for ph, mm in self.models.items()}
            for ph, e in exp.items():
                if e and ph in self.alive:
                    self.alive.discard(ph)
            self.rows.append({"t": t, "expected": exp, "observed": [], "frame": None, "build": None})
            return
        m = RX_BUILD.search(l)
        if m:
            exp = {ph: frame_order(mm.retrieve(self.depth)) for ph, mm in self.models.items()}
            self.rows.append({"t": t, "expected": exp, "observed": None, "frame": None,
                              "build": (m.group(1), int(m.group(2)))})
            return
        m = RX_ATTX.search(l)
        if m and self.rows and self.rows[-1]["observed"] is None:
            f = frame_by_idx.get(idx)
            row = self.rows[-1]
            row["frame"] = f
            obs = [self.label_of(p) for p in (f["dec"]["positions"] if f else [])]
            row["observed"] = obs
            for ph in list(self.alive):
                if not _labels_match(row["expected"][ph], obs):
                    self.alive.discard(ph)
            return
        m = RX_PILE.search(l)
        if m:
            got = "" if m.group(1) == "empty" else " ".join(m.group(1).split())
            for ph in list(self.alive):
                if self.models[ph].dump() != got:
                    self.pile_bad.append(f"+{t:.1f}s %PILE '{got}' / modele phase {ph} '{self.models[ph].dump()}'")
                    self.alive.discard(ph)


def _labels_match(exp, obs):
    if len(exp) != len(obs):
        return False
    for e, o in zip(exp, obs):
        if e == o or (e.startswith("REAL") and o == "REAL"):
            continue
        return False
    return True


def analyse_pile_run(lines, run, res, sink=None):
    """Analyse une passe du scenario pile (lignes entre RUN et END)."""
    scen = "pile"
    name = run["name"]
    frames = extract_frames(lines)
    # Par indice de ligne, pas par instant : sous Windows time.monotonic() a une
    # resolution de ~16 ms, deux lignes d un meme paquet USB partagent l instant.
    by_idx = {f["idx"]: f for f in frames}
    rp = PileReplay(run["mode"], run["ntry"], run["depth"], run["injected"])
    for idx, (t, l) in enumerate(lines):
        rp.feed(idx, t, l, by_idx)
    t0 = lines[0][0] if lines else 0.0
    pile_frames = [r for r in rp.rows if r["build"] is not None]
    print(f"    {name}: {len(frames)} AT+TX dont {len(pile_frames)} issues de la pile ; phases restantes "
          f"{sorted(rp.alive)} sur {list(range(rp.max_index))}", flush=True)
    for r in rp.rows:
        f = r["frame"]
        exp = " | ".join(f"ph{ph}:{e}" for ph, e in r["expected"].items())
        print(f"      +{r['t'] - t0:6.1f}s obs={r['observed']} attendu {exp} "
              f"{(f['hex'] + ' ' + f['status']) if f else ''}", flush=True)
        if sink:
            sink.jsonl({"type": "pile_frame", "run": name, "t": r["t"], "observed": r["observed"],
                        "expected": {str(k): v for k, v in r["expected"].items()},
                        "hex": f["hex"] if f else None, "status": f["status"] if f else None})
    cached = [f for f in frames if f["val"] and f["val"]["type"] == "short-cached"]
    if run.get("cycles_done", 0) == 0:
        res.add(scen, f"{name}-0", f"{name}: passe jouee", "FAIL", "aucun cycle n a abouti")
        return {"name": name, "seq": [], "alive": []}
    # 1. credit initial = semantique NTRY du mode
    res.add(scen, f"{name}-1", f"{name}: credit initial des entrees (NTRY={run['ntry']}, mode={run['mode']})",
            "FAIL" if rp.credit_bad else "PASS",
            f"attendu {credit_for(run['mode'], run['ntry'])} par entree (depth_pile.cpp:205-221)", rp.credit_bad)
    # 2. chaque trame porte exactement les positions predites
    unanswered = [r for r in pile_frames if r["observed"] is None]
    ok = bool(rp.alive) and not unanswered and bool(pile_frames)
    detail = (f"{len(pile_frames)} trame(s) pile ; hypothese(s) de phase coherente(s) : {sorted(rp.alive) or 'AUCUNE'}"
              f" ; sequence observee {[r['observed'] for r in pile_frames]}")
    if not pile_frames:
        detail = "aucune trame issue de la pile (build_gnss_packet) : rien a comparer"
    res.add(scen, f"{name}-2", f"{name}: positions de chaque trame = modele DepthPile", "PASS" if ok else "FAIL",
            detail, rp.pile_bad + [f"retrieve sans trame a +{r['t'] - t0:.1f}s" for r in unanswered])
    # 3. trames GNSS completes
    bad = [f for f in frames if f["status"] != "ok"]
    res.add(scen, f"{name}-3", f"{name}: emissions abouties", "FAIL" if bad else "PASS",
            f"{len(frames)} AT+TX, {len(bad)} sans [VAL-TX] de fin ; {len(cached)} ping(s) Doppler a position "
            f"en cache (hors pile)", [frame_summary(f, t0) for f in bad])
    if rp.real_n:
        res.add(scen, f"{name}-4", f"{name}: positions non injectees", "INFO",
                f"{rp.real_n} vrai(s) fix GNSS glisse(s) dans la pile (traites en joker)")
    return {"name": name, "seq": [r["observed"] for r in pile_frames], "alive": sorted(rp.alive),
            "mode": run["mode"], "ntry": run["ntry"]}


# ============================================================================
#  Configuration : template tortue -> clefs DTE
# ============================================================================
def param_maps():
    try:
        from kim_bench import load_param_map
        return load_param_map(DTE_PARAMS)
    except SystemExit:
        rx = re.compile(r'\{\s*"([A-Z0-9_]+)"\s*,\s*"([A-Z0-9]+)"')
        n2k, k2n = {}, {}
        for line in open(DTE_PARAMS, encoding="utf-8", errors="replace"):
            m = rx.search(line)
            if m:
                n2k[m.group(1)], k2n[m.group(2)] = m.group(2), m.group(1)
        return n2k, k2n


def turtle_profile():
    """template_conf/turtle_gps.cfg (espace PyLinkit) -> {clef DTE: valeur fil}."""
    import check_template_conf as TC
    import pylinkit_map as PL
    inv = {v: k for k, v in PL.NAMES.items()}
    out, notes = {}, []
    for raw in open(TEMPLATE, encoding="utf-8"):
        line = raw.split("#", 1)[0].strip()
        if "=" not in line or line.startswith("["):
            continue
        nom, val = (s.strip() for s in line.split("=", 1))
        key = inv.get(nom)
        if key is None:
            notes.append(f"{nom}: inconnu de pylinkit_map, ignore")
            continue
        typ = PL.TYPES.get(key)
        if typ in ("TEXT", "DATESTRING", "HEXADECIMAL"):
            continue
        if typ == "ARGOSDUTYCYLE":
            out[key] = int(val, 16)       # firmware ARP18 = UINT 0..0xFFFFFF (dte_params.cpp:51)
            continue
        wire, err = TC.to_wire(key, val)
        if err is not None or wire is None:
            raise SystemExit(f"template {nom}={val!r}: {err}")
        out[key] = int(wire) if float(wire) == int(wire) else wire
    return out, notes


def by_name(**kw):
    n2k, _ = param_maps()
    out = {}
    for k, v in kw.items():
        if k not in n2k:
            raise SystemExit(f"parametre inconnu du firmware: {k}")
        out[n2k[k]] = v
    return out


def burst_profile(extra=None):
    prof, _ = turtle_profile()
    prof.update(by_name(LB_THRESHOLD=0, LB_CRITICAL_THRESH=0, SWS_SENSOR_ENABLE_TX_MODE=0, ARGOS_BLIND_EN=0))
    if extra:
        prof.update(extra)
    return prof


def burst_cfg_from(prof):
    k = by_name(SURFACING_BURST_INIT_S=0, SURFACING_BURST_STEP_S=0, SURFACING_BURST_MAX_S=0,
                SURFACING_BURST_MAX_MSG=0, TR_NOM=0, ARGOS_DEPTH_PILE=0, NTRY_PER_MESSAGE=0, ARGOS_MODE=0)
    names = ("init", "step", "max", "max_msg", "tr_nom", "depth", "ntry", "mode")
    return {name: int(prof[key]) for name, key in zip(names, k.keys())}


# ============================================================================
#  Banc vivant
# ============================================================================
class BenchError(Exception):
    pass


class Sink:
    def __init__(self):
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.base = os.path.join(HERE, f"turtle_mission_rf_{stamp}")
        self._j = P.utf8_open(self.base + ".jsonl", "a", buffering=1)
        self._rows = []

    def jsonl(self, obj):
        try:
            self._j.write(json.dumps(obj, default=str) + "\n")
        except Exception:
            pass

    def frames(self, scen, run, frames, t0):
        prev = None
        for f in frames:
            d = f["dec"]
            row = {"scenario": scen, "run": run, "t_rel_s": round(f["t"] - t0, 2),
                   "dt_prev_s": round(f["t"] - prev, 2) if prev is not None else "", "hex": f["hex"],
                   "nbytes": d.get("nbytes"), "kind": d["kind"], "ok": d.get("ok"),
                   "val_type": f["val"]["type"] if f["val"] else "", "dop_count": f["val"]["dop_count"] if f["val"] else "",
                   "status": f["status"], "positions": ";".join(_fmt_pos(p) for p in d.get("positions", [])),
                   "batt_mv": d.get("batt_mv", ""), "dx1_dry": d.get("dry", ""), "dx1_dend": d.get("dend", ""),
                   "crc_ok": d.get("crc_ok", ""), "prewarm": f["prewarm"] or "", "build": f["build"] or "",
                   "txstart_uptime_ms": f["txstart_up"] or ""}
            self._rows.append(row)
            self.jsonl(dict(row, type="frame"))
            prev = f["t"]

    def close(self):
        if self._rows:
            with P.utf8_open(self.base + ".csv", "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=list(self._rows[0].keys()))
                w.writeheader()
                w.writerows(self._rows)
        try:
            self._j.close()
        except Exception:
            pass


class Rig:
    """gnss_bench.Board (reconnexion USB, journal horodate) + configuration DTE
    verifiee + budget RF."""

    def __init__(self, args):
        from gnss_bench import Board
        # rf_guard=False : DTE_RF_AUTORISE=1 a ete verifie par main().
        self.b = Board(rf_guard=False)
        self.args = args
        self.budget = None
        self._bud_idx = 0
        self._bud_count = 0
        self.rf_stopped = False
        self.t_wall0 = time.time() - (time.monotonic() - self.b.t0)
        self._stop = threading.Event()
        threading.Thread(target=self._budget_watch, daemon=True).start()
        print(f"port: {self.b.port} ; journal brut: {self.b.logpath}", flush=True)

    # ---- temps hote -> UTC
    def utc_at(self, t_rel):
        return datetime.datetime.fromtimestamp(self.t_wall0 + t_rel, datetime.timezone.utc)

    # ---- budget RF
    def arm_budget(self, n):
        self._bud_idx = self.b.mark()
        self._bud_count = 0
        self.budget = n
        self.rf_stopped = False

    def _budget_watch(self):
        while not self._stop.is_set():
            time.sleep(0.4)
            if self.budget is None:
                continue
            new = self.b.since(self._bud_idx)
            self._bud_idx += len(new)
            self._bud_count += sum(1 for _, l in new if RX_ATTX.search(l))
            if self._bud_count > self.budget and not self.rf_stopped:
                self.rf_stopped = True
                try:
                    self.b.send("%AMODE 0")
                except Exception:
                    pass
                self.b._note(f"[turtle] NOTE budget RF depasse ({self._bud_count}>{self.budget}) -- %AMODE 0 envoye")

    def check_rf(self):
        if self.rf_stopped:
            raise BenchError(f"budget RF depasse ({self._bud_count}>{self.budget}) -- scenario interrompu")

    # ---- primitives
    def note(self, text):
        self.b._note(f"[turtle] {text}")

    def mark(self):
        return self.b.mark()

    def cmd(self, text, pat, timeout=8.0):
        i = self.mark()
        self.b.send(text)
        return self.b.wait(pat, timeout, since=i)

    def sleep(self, s):
        end = time.monotonic() + s
        while time.monotonic() < end:
            self.check_rf()
            time.sleep(min(0.5, max(0.0, end - time.monotonic())))

    def wait_line(self, rx, timeout, since, count=1):
        """Attend `count` lignes correspondant a rx depuis `since` ; rend la liste."""
        end = time.monotonic() + timeout
        while True:
            self.check_rf()
            hits = [(t, l) for t, l in self.b.since(since) if rx.search(l)]
            if len(hits) >= count or time.monotonic() > end:
                return hits
            time.sleep(0.3)

    def ping_operational(self, timeout=40):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            l = self.cmd("%PING", r"%BENCH OK state=(\S+)", 2.5)
            if l and "OPERATIONAL" in l:
                return True
            if l and "CONFIG" in l:
                self.cmd("%OP", r"%OP OK", 6)
            time.sleep(1.0)
        raise BenchError("la carte ne repond pas OPERATIONAL a %PING")

    def enter_config(self):
        for _ in range(15):
            i = self.mark()
            self.b.send("%CFG")
            l = self.b.wait(r"%CFG (OK|ERR)", 4, since=i)
            if l and "OK" in l:
                self.b.wait(r"entry: ConfigurationState|already-config", 8, since=i)
                time.sleep(1.5)
                return
            time.sleep(2.0)
        raise BenchError("impossible d entrer en configuration (%CFG)")

    def exit_config(self):
        i = self.mark()
        self.b.send("%OP")
        self.b.wait(r"%OP OK", 10, since=i)
        self.b.wait(RX_OPENTRY.pattern, 30, since=i)
        time.sleep(2.0)
        self.ping_operational()

    def dte(self, cmd, payload, timeout=10.0):
        for _ in range(3):
            i = self.mark()
            self.b.send(f"${cmd}#{len(payload):03X};{payload}")
            l = self.b.wait(rf"\$[ON];{cmd}#", timeout, since=i)
            if l:
                m = re.search(rf"\$([ON]);{cmd}#([0-9A-Fa-f]{{3}});?(.*)", l)
                if m:
                    return m.group(1), m.group(3).strip()
            time.sleep(1.0)
        raise BenchError(f"pas de reponse a ${cmd}")

    def _read(self, cmd, keys):
        out = {}
        for i in range(0, len(keys), 10):
            code, body = self.dte(cmd, ",".join(keys[i:i + 10]))
            if code != "O":
                raise BenchError(f"${cmd} refuse ({body}) pour {keys[i:i + 10]}")
            out.update(dict(re.findall(r"([A-Z]{3}\d\d)=([^,]*)", body)))
        return out

    def parmr(self, keys):
        return self._read("PARMR", list(keys))

    def statr(self, keys):
        return self._read("STATR", list(keys))

    def parmw(self, kv):
        items = [f"{k}={v}" for k, v in kv.items()]
        chunk = []
        for it in items + [None]:
            if it is not None and len(",".join(chunk + [it])) <= 200:
                chunk.append(it)
                continue
            if chunk:
                code, body = self.dte("PARMW", ",".join(chunk))
                if code != "O":
                    raise BenchError(f"PARMW refuse : {body!r} pour {chunk} (une liste de clefs = clefs "
                                     f"rejetees, les autres SONT appliquees ; 4 = DATA_LENGTH_MISMATCH)")
            chunk = [it] if it is not None else []

    def configure(self, kv, label, critical=(), optional=None):
        """N ecrit QUE les clefs qui different (ecrire ARP54 a l identique pourrait
        remettre ART13 a 3, dte_params.cpp:567-570), puis relit tout.

        `optional` : clefs compilees selon la carte (ENABLE_*_SENSOR) -- ecrites
        seulement si la carte les connait."""
        self.note(f"CFG {label}")
        self.enter_config()
        kv = dict(kv)
        for k, v in (optional or {}).items():
            try:
                self.parmr([k])
                kv[k] = v
            except BenchError:
                pass
        cur = self.parmr(list(kv))
        diff = {k: v for k, v in kv.items() if not _same(cur.get(k), v)}
        if diff:
            print(f"    {label}: ecriture de {len(diff)} clef(s) : {diff}", flush=True)
            self.parmw(diff)
        after = self.parmr(list(kv))
        bad = {k: (after.get(k), v) for k, v in kv.items() if not _same(after.get(k), v)}
        st = {}
        try:
            st = self.statr(["ART13", "SYT01"])
        except BenchError as e:
            print(f"    STATR ART13/SYT01 indisponible : {e}", flush=True)
        self.exit_config()
        crit = {k: v for k, v in bad.items() if k in critical or not critical}
        if bad:
            print(f"    relecture differente pour {bad}", flush=True)
        if crit:
            raise BenchError(f"{label}: configuration non appliquee : {crit}")
        return {"written": diff, "readback_mismatch": bad, "statr": st, "before": cur}

    def safe_off(self, persist=True):
        try:
            # Rend l etat sous-marin au vrai detecteur (%DIVE/%SURFACE le figent
            # depuis le firmware du 2026-10-11 ; ancien firmware : %ERR, sans effet).
            self.cmd("%UWREAL", r"%UWREAL OK|%ERR", 6)
        except Exception:
            pass
        try:
            self.cmd("%AMODE 0", r"%AMODE mode=", 6)
            if persist:
                self.enter_config()
                self.parmw(by_name(ARGOS_MODE=0))
                self.exit_config()
        except Exception as e:
            print(f"    mise hors emission incomplete : {e}", flush=True)

    def close(self):
        self._stop.set()
        self.b.close()


def _same(a, b):
    if a is None:
        return False
    try:
        if int(str(a), 0) == int(b):
            return True
        # PARMR peut rendre un masque en hexadecimal (ARP18 FFFFFF).
        return bool(re.fullmatch(r"[0-9A-Fa-f]+", str(a))) and int(str(a), 16) == int(b) \
            and bool(re.search(r"[A-Fa-f]", str(a)))
    except (ValueError, TypeError):
        try:
            return float(a) == float(b)
        except (ValueError, TypeError):
            return str(a).strip() == str(b).strip()


CRITICAL = tuple(by_name(ARGOS_MODE=0, NTRY_PER_MESSAGE=0, ARGOS_DEPTH_PILE=0, ARGOS_ADAPTIVE_MODULATION=0,
                         SURFACING_BURST_INIT_S=0, SURFACING_BURST_STEP_S=0, SURFACING_BURST_MAX_S=0,
                         SURFACING_BURST_MAX_MSG=0, UNDERWATER_EN=0, SWS_SENSOR_ENABLE_TX_MODE=0, TR_NOM=0,
                         GNSS_TRIGGER_ON_SURFACED=0, DLOC_ARG_NOM=0, MIN_SURFACE_CYCLE_INTERVAL_S=0,
                         ARGOS_TX_JITTER_EN=0, GNSS_FASTLOC_MODE=0).keys())


# Emission capteur (process_sensor_burst) au lieu de GNSS si l un de ces modes est
# actif (config_store.hpp:1476-1510) : paquet capteur 24 o et retrieve_gps_single
# (une position, depth_pile.hpp:353) -- ni le format ni la rotation attendus ici.
SENSOR_TX_OFF = {k: 0 for k in ("STP04", "PHP04", "LTP04", "PRP04", "AXP05", "THP06")}


def check_master(rig, info, res, scen):
    art13 = info["statr"].get("ART13")
    if art13 is None:
        res.add(scen, "PRE", "maitresse LDA2 relue (ART13=0)", "SKIP", "ART13 illisible -- tailles LDA2 supposees")
        return
    if str(art13).strip() != "0":
        res.add(scen, "PRE", "maitresse LDA2 relue (ART13=0)", "FAIL",
                f"ART13={art13} (0 LDA2, 1 LDK, 2 VLDA4, 3 inconnu : lancer SATVF). Ce scenario decode des trames "
                f"LDA2 : arret.")
        raise BenchError("maitresse non LDA2")
    res.add(scen, "PRE", "maitresse LDA2 relue (ART13=0)", "PASS", f"ART13={art13}")


def ensure_rtc(rig, res, scen):
    """SURFACING_BURST (phase GNSS) et LEGACY attendent une horloge reglee
    (argos_tx_service.cpp:530-532 et 771-773). Les fix INJECTES ne la reglent pas :
    seul un vrai PVT le fait (m10qasync.cpp, rtc->settime). Si SYT01 est loin de
    l heure hote, amorce par un vrai fix, radio coupee (ARGOS_MODE=0)."""
    rig.enter_config()
    syt = rig.statr(["SYT01"]).get("SYT01")
    rig.exit_config()
    try:
        skew = abs(int(syt) - time.time())
    except (TypeError, ValueError):
        skew = None
    if skew is not None and skew < 120:
        print(f"    RTC regle (ecart hote {skew:.0f} s)", flush=True)
        return
    print(f"    RTC non regle (SYT01={syt}) : fix reel d amorce, ARGOS_MODE=0", flush=True)
    rig.configure(burst_profile(by_name(ARGOS_MODE=0, GNSS_TRIGGER_ON_SURFACED=1)), "amorce RTC (radio OFF)",
                  CRITICAL)
    rig.cmd("%DIVE", r"%DIVE OK", 8)
    rig.sleep(10)
    i = rig.mark()
    rig.cmd("%SURFACE", r"%SURFACE OK", 8)
    hits = rig.wait_line(RX_RTCSYNC, 240, i)
    rig.cmd("%DIVE", r"%DIVE OK", 8)
    rig.enter_config()
    syt = rig.statr(["SYT01"]).get("SYT01")
    rig.exit_config()
    try:
        ok = abs(int(syt) - time.time()) < 120
    except (TypeError, ValueError):
        ok = False
    res.add(scen, "RTC", "horloge reglee par un vrai fix avant la passe", "PASS" if ok else "FAIL",
            f"SYT01={syt} ; " + (hits[0][1] if hits else "aucun 'GNSS time persisted' en 240 s"))


# ---------------------------------------------------------------- scenarios
def run_burst(rig, args, res, sink):
    scen = "burst"
    prof = burst_profile()
    cfg = burst_cfg_from(prof)
    # Phase Doppler (MAX_MSG) + 1 trame GNSS + battements au TR_NOM sur la fenetre.
    per_cycle = cfg["max_msg"] + 1 + args.observe_s // max(1, cfg["tr_nom"] - 5) + 2
    rig.arm_budget(args.max_tx or per_cycle * args.cycles)
    info = rig.configure(prof, "burst: profil tortue", CRITICAL, SENSOR_TX_OFF)
    rig.note("CFG " + json.dumps(cfg))
    check_master(rig, info, res, scen)
    ref = (args.ref_lat, args.ref_lon, args.ref_tol_km) if args.ref_lat is not None else None
    for c in range(1, args.cycles + 1):
        rig.note(f"RUN burst cycle={c}")
        outages = rig.b.outages
        rig.cmd("%DIVE", r"%DIVE OK", 8)
        print(f"  [burst c{c}] plongee {args.dive_s} s", flush=True)
        rig.sleep(args.dive_s)
        i = rig.mark()
        rig.cmd("%SURFACE", r"%SURFACE OK", 8)
        print(f"  [burst c{c}] emersion, observation {args.observe_s} s", flush=True)
        rig.sleep(args.observe_s)
        lines = rig.b.since(i)
        frames = analyse_burst(lines, cfg, res, ref, rig.utc_at)
        sink.frames(scen, f"c{c}", frames, lines[0][0] if lines else 0.0)
        if rig.b.outages != outages:
            res.add(scen, f"LINK{c}", "lien USB stable pendant le cycle", "FAIL",
                    "le port a ete perdu pendant le cycle : mesures a reprendre")
        rig.cmd("%DIVE", r"%DIVE OK", 8)


def run_pile(rig, args, res, sink):
    scen = "pile"
    runs_wanted = [r.strip() for r in args.runs.split(",") if r.strip()]
    base = burst_profile(by_name(TR_NOM=30, ARGOS_TX_JITTER_EN=0, GNSS_TRIGGER_ON_SURFACED=0, DLOC_ARG_NOM=10,
                                 ARGOS_TIME_SYNC_BURST_EN=0, ARGOS_DEPTH_PILE=4, GNSS_FASTLOC_MODE=0))
    restore_keys = list(by_name(GNSS_TRIGGER_ON_SURFACED=0, DLOC_ARG_NOM=0, TR_NOM=0, ARGOS_TX_JITTER_EN=0,
                                ARGOS_MODE=0, NTRY_PER_MESSAGE=0).keys())
    rig.enter_config()
    saved = rig.parmr(restore_keys)
    rig.exit_config()
    rig.note("NOTE pile: valeurs a restaurer " + json.dumps(saved))
    ensure_rtc(rig, res, scen)
    results = {}
    budget_total = 0
    try:
        for name in runs_wanted:
            if name == "sb0":
                spec = dict(name="sb0", mode=5, ntry=0, plan=[1, "dive-before", 1])
            elif name == "sbn":
                spec = dict(name="sbn", mode=5, ntry=args.ntry, plan=[args.frames] * args.cycles)
            elif name == "legn":
                spec = dict(name="legn", mode=2, ntry=args.ntry, plan=[args.frames] * args.cycles)
            elif name == "leg0":
                spec = dict(name="leg0", mode=2, ntry=0, plan=[args.frames] * args.cycles)
            else:
                raise BenchError(f"passe inconnue {name} (sb0,sbn,legn,leg0)")
            spec["depth"] = 4
            n_tx = sum((1 if spec["mode"] == 5 else 0) + (p if p != "dive-before" else 0) for p in spec["plan"])
            # Marge : un ping Doppler 2 si le fix injecte tarde, un battement si la
            # plongee suit de pres la fin de salve.
            budget = n_tx + 2 * len(spec["plan"]) + 4
            budget_total += budget
            rig.arm_budget(budget)
            kv = dict(base)
            kv.update(by_name(ARGOS_MODE=spec["mode"], NTRY_PER_MESSAGE=spec["ntry"]))
            info = rig.configure(kv, f"pile {name}", CRITICAL, SENSOR_TX_OFF)
            if name == runs_wanted[0]:
                check_master(rig, info, res, scen)
            spec["injected"] = []
            i_run = rig.mark()
            rig.note(f"RUN {name} " + json.dumps({k: spec[k] for k in ("mode", "ntry", "plan", "depth")}))
            spec["cycles_done"] = _pile_cycles(rig, args, spec)
            rig.note(f"END {name}")
            lines = rig.b.since(i_run)
            print(f"  [pile {name}] analyse", flush=True)
            results[name] = analyse_pile_run(lines, spec, res, sink)
            sink.frames(scen, name, extract_frames(lines), lines[0][0] if lines else 0.0)
    finally:
        try:
            rig.cmd("%AMODE 0", r"%AMODE mode=", 6)
            rig.enter_config()
            rig.parmw({k: v for k, v in saved.items() if v != ""})
            rig.exit_config()
            print(f"    restaure : {saved}", flush=True)
        except Exception as e:
            print(f"    RESTAURATION INCOMPLETE {saved}: {e}", flush=True)
    compare_pile_runs(results, res, args)


def _pile_cycles(rig, args, spec):
    """Execute le plan ; rend le nombre de cycles menes a terme."""
    done = 0
    rig.cmd("%DIVE", r"%DIVE OK", 8)
    rig.sleep(args.pile_dive_s)
    for k, p in enumerate(spec["plan"], 1):
        rig.note(f"CYCLE {spec['name']} {k} plan={p}")
        lat = round(args.base_lat + 0.01 * k, 4)
        lon = round(args.base_lon + 0.01 * k, 4)
        spec["injected"].append((f"P{k}", lat, lon))
        i = rig.mark()
        rig.cmd("%SURFACE", r"%SURFACE OK", 8)
        if spec["mode"] == 5:
            # Ping 1 (Doppler) d abord : le fix arrive apres, comme sur l animal.
            got = rig.wait_line(RX_VALTX, 40, i)
            if not got:
                print(f"    cycle {k}: pas de ping 1 en 40 s", flush=True)
        j = rig.mark()
        if not rig.cmd(f"%GPS {lat} {lon} 2500 9", r"%GPS OK", 8):
            print(f"    cycle {k}: %GPS sans reponse", flush=True)
        if p == "dive-before":
            time.sleep(0.3)
            rig.cmd("%DIVE", r"%DIVE OK", 8)
            hit = [l for _, l in rig.b.since(j) if RX_ATTX.search(l)]
            if hit:
                print(f"    cycle {k}: une emission est partie avant la plongee : {hit}", flush=True)
            rig.sleep(args.pile_dive_s)
            done += 1
            continue
        end = time.monotonic() + p * (30 + 25) + 40
        while time.monotonic() < end:
            rig.check_rf()
            lines = rig.b.since(j)
            n_g = sum(1 for _, l in lines if RX_VALTX.search(l) and "type=gnss" in l)
            n_fail = sum(1 for _, l in lines if RX_DEVERR.search(l))
            finished = spec["mode"] == 5 and any(RX_BURSTEND.search(l) for _, l in lines)
            if n_g >= p or finished or n_fail >= 2:
                break
            time.sleep(0.5)
        # Plonger AUSSITOT : la trame suivante de la phase GNSS (TR_NOM plus tard)
        # ne doit pas partir. Le cliche %PILE suit la plongee (pile figee).
        rig.cmd("%DIVE", r"%DIVE OK", 8)
        rig.cmd("%PILE", r"%PILE ", 4)
        rig.sleep(args.pile_dive_s)
        done += 1
    return done


def compare_pile_runs(results, res, args):
    scen = "pile"
    sb, lg = results.get("sbn"), results.get("legn")
    if sb and lg:
        same = sb["seq"] == lg["seq"]
        both_model = bool(sb["alive"]) and bool(lg["alive"])
        if same:
            v, d = "PASS", "meme rotation trame pour trame en SURFACING_BURST et en LEGACY (NTRY identique)"
        elif both_model:
            v, d = "INFO", ("rotations differentes mais CHACUNE conforme au modele DepthPile : ecart de phase de "
                            "m_retrieve_index (non remis a zero par clear(), depth_pile.hpp:46) ou arret de la phase "
                            "GNSS SB quand plus rien n est eligible (argos_tx_service.cpp:774-781)")
        else:
            v, d = "FAIL", "rotations differentes et au moins une passe hors modele"
        res.add(scen, "CMP-1", f"rotation SB NTRY={args.ntry} vs LEGACY NTRY={args.ntry}", v,
                f"{d} ; SB={sb['seq']} ; LEGACY={lg['seq']}")
    sb0 = results.get("sb0")
    if sb0:
        want = [["P1"], ["P3", "P2"]]
        ok = sb0["seq"] == want
        res.add(scen, "CMP-2", "template NTRY=0 en SURFACING_BURST : une emission par fix, arriere conserve",
                "PASS" if ok else "FAIL",
                f"observe {sb0['seq']}, attendu {want} (credit 1 par fix, depth_pile.cpp:216-218 ; P2 non emis "
                f"avant la plongee part avec P3). DIFFERENCE VOULUE avec LEGACY NTRY=0 (illimite, rotation "
                f"perpetuelle) : en SB la pile ne re-emet jamais une position deja partie.")


def run_dx1(rig, args, res, sink):
    scen = "dx1"
    prof = burst_profile(by_name(SWS_SENSOR_ENABLE_TX_MODE=2))
    cfg = burst_cfg_from(prof)
    rig.arm_budget(args.max_tx or (2 * args.cycles + 2))
    info = rig.configure(prof, "dx1: profil tortue + UNP26=2", CRITICAL, SENSOR_TX_OFF)
    rig.note("CFG " + json.dumps(cfg))
    check_master(rig, info, res, scen)
    allf = []
    try:
        rig.cmd("%DIVE", r"%DIVE OK", 8)
        for c in range(1, args.cycles + 1):
            rig.note(f"RUN dx1 cycle={c}")
            rig.sleep(args.dive_s)
            i = rig.mark()
            rig.cmd("%SURFACE", r"%SURFACE OK", 8)
            got = rig.wait_line(RX_VALTX, 40, i)
            if not got:
                rig.wait_line(RX_DEVERR, 5, i)
            rig.cmd("%DIVE", r"%DIVE OK", 8)
            time.sleep(1.0)
            fr = extract_frames(rig.b.since(i))
            for f in fr:
                print("    " + frame_summary(f, fr[0]["t"]), flush=True)
            sink.frames(scen, f"c{c}", fr, fr[0]["t"] if fr else 0.0)
            allf.append((c, fr))
    finally:
        try:
            rig.cmd("%AMODE 0", r"%AMODE mode=", 6)
            rig.enter_config()
            rig.parmw(by_name(SWS_SENSOR_ENABLE_TX_MODE=0))
            rig.exit_config()
        except Exception as e:
            print(f"    UNP26 non remis a 0 : {e}", flush=True)
    analyse_dx1(allf, res)


def analyse_dx1(allf, res):
    scen = "dx1"
    firsts = [(c, fr[0]) for c, fr in allf if fr]
    if not firsts:
        res.add(scen, "X1", "ping 1 DX1 a chaque emersion", "FAIL", "aucune trame")
        return
    probs, ev = [], []
    for c, f in firsts:
        d = f["dec"]
        ev.append(f"c{c} {f['hex']} kind={d['kind']} prewarm={f['prewarm']} dry={d.get('dry')} "
                  f"dend={d.get('dend')} crc={d.get('crc_ok')} batt={d.get('batt_mv')} {f['status']}")
        if d["kind"] != "dx1" or d.get("nbytes") != 4:
            probs.append(f"c{c}: pas une vue DX1 4 o ({d['kind']}) -- firmware sans DX1_DOPPLER_EXT_BUILD, "
                         f"UNDERWATER_EN=0 ou UNP26 non applique")
            continue
        if not d["crc_ok"]:
            probs.append(f"c{c}: CRC8 faux")
        if f["prewarm"] != ("LDA2", 32):
            probs.append(f"c{c}: PREWARM {f['prewarm']} au lieu de ('LDA2', 32)")
        if f["status"] != "ok":
            probs.append(f"c{c}: emission {f['status']}")
    res.add(scen, "X1", "ping 1 = vue DX1 4 o sur LDA2, VER=1 EXT=0, CRC8 valide", "FAIL" if probs else "PASS",
            f"{len(firsts)} emersion(s)", probs + ev)
    dends = [f["dec"].get("dend") for _, f in firsts if f["dec"]["kind"] == "dx1"]
    res.add(scen, "X2", "DEND=1 (plongee terminee normalement)", "PASS" if dends and all(x == 1 for x in dends)
            else "FAIL", f"DEND par emersion = {dends} (2 = dive-timeout, 3 = surface forcee/ADC, 0 = inconnu)")
    drys = [f["dec"].get("dry") for _, f in firsts if f["dec"]["kind"] == "dx1"]
    res.add(scen, "X3", "DRY=0 (moins de 12 h d observation)", "PASS" if drys and all(x == 0 for x in drys)
            else "INFO", f"DRY = {drys}")
    res.add(scen, "X4", "LATCH / SURF_SEQ / S_LAST / D_PREV", "SKIP",
            "absents de la vue LDA2 (4 o) : le bloc riche n est porte que par la vue LDK 16 o "
            "(argos_packet_builder.hpp DX1, doppler_view). Non verifiable sur une maitresse LDA2 ; "
            "sat_dx1_smoke.py le couvre sur une maitresse LDK.")


# ============================================================================
#  Plan
# ============================================================================
PLAN = r"""
PLAN D ESSAI -- mission tortue, emission reelle (LinkIt V4 SMD UART-AT, maitresse LDA2)
=====================================================================================
0. Preparation (UNE fois) -- jamais par ce script :
   - flasher selon .claude/CLAUDE.md (nrfjprog --recover puis --program *_merged-*.hex
     --chiperase --verify --reset ; exiger "Verify successful" ET "Run.").
   - --recover EFFACE le magasin : re-provisionner identifiants Argos + RCONF maitresse
     LDA2, lancer SATVF, verifier STATR ART13=0. Le script refuse une maitresse != LDA2.
   - antenne branchee, ciel visible ; DTE_RF_AUTORISE=1 ; PYTHONUTF8=1.

1. turtle_mission_rf.py burst            ~ 10 min   ~ 9 trames, borne 16 (--max-tx)
      1 plongee 120 s + 7 min d emersion : ping 1 PREWARM, salve 5/15/25/35/45 s,
      fix reel, 1 trame GNSS SHORT, puis battement de presence a 60 s.
      Option : --ref-lat/--ref-lon/--ref-tol-km (position connue du banc).
2. turtle_mission_rf.py pile             ~ 35 min   ~ 30 trames (borne par passe)
      sb0 (3 cycles), sbn (5 cycles x 2 trames), legn (5 cycles x 2 trames) ;
      --runs sb0,sbn,legn,leg0 pour ajouter LEGACY illimite. Positions injectees
      (%GPS, base --base-lat/--base-lon + k*0.01 deg) : ce sont de VRAIES trames Argos
      qui portent des positions FICTIVES -- prevenir le depouillement CLS.
3. turtle_mission_rf.py dx1              ~ 5 min    3 trames, borne 8
      UNP26=2 FORCE, 3 cycles, ping 1 DX1 4 o (DRY/DEND/CRC). Le decodeur plateforme
      doit etre patche DX1 (sinon trames lues comme des Doppler "historiques" bizarres).
      UNP26 est remis a 0 a la fin.

4. Cas existants de dte_campaign.py, a jouer avec DTE_RF_AUTORISE=1 (ne PAS les
   dupliquer ici) -- python tests/bench/dte_campaign.py --cases <liste> :
      limiteur de debit  : RL-01, RL-02, RL-03
      echouage / hors-eau: HAUL-02, HAUL-03, HAULED-01
      salve / refroidis. : ARG-03, SURF-01, SURF-02, SURF-03, SURF-04
      format LONG        : MOD-04 (compte aussi "TX SUCCESS", ligne KIM2 seulement :
                           sur SMD ne retenir que build_gnss_packet LONG 192 bits)
      deploiement        : DEP-01, DEP-02, DEP-03, DEP-04, DEP-05
                           puis DEP-06 EN DERNIER (reset usine : re-provisionner
                           identifiants + maitresse + SATVF apres)
   Exemple : python tests/bench/dte_campaign.py --cases RL-01,RL-02,RL-03,HAUL-02,HAUL-03,
             HAULED-01,ARG-03,SURF-01,SURF-02,SURF-03,SURF-04,MOD-04,DEP-01,DEP-02,DEP-03,
             DEP-04,DEP-05
   ATTENTION : ces cas laissent leur propre configuration ; rejouer turtle_mission_rf.py
   APRES eux, pas avant (le script pose tout ce dont il depend, mais pas l inverse).

Budget RF total indicatif : ~60 trames + les cas dte_campaign.
Sorties : tests/bench/turtle_mission_rf_<horodatage>.{jsonl,csv}, journaux bruts
tests/bench/logs/gnss_*.log (rejouables : --replay LOG --scenario burst|pile|dx1).
"""


# ============================================================================
#  Auto-test hors ligne
# ============================================================================
# Vecteurs du VRAI constructeur C++ :
#   (a) tests/src/argos_tx_test.cpp BuildShortGNSSPacket / BuildLongGNSSPacket (passent
#       sur tests/build/TrackerTests) : lon=12.3 lat=44.4 t=1652105502, fixType 2,
#       batt 4200 ; LONG = prefixe + CRC auto-coherent ;
#   (b) banc jetable compile depuis core/services/argos_packet_builder.cpp avec les
#       options de TrackerTests (HEAD 85bd5bc3), t=1760184000 (2025-10-11 12:00 UTC),
#       DELTA_T_10MIN (ou 30MIN + hors zone + LB pour les deux derniers) :
#       a=(43.6012, 3.9001, 3D, hMSL 10 m, 0 mm/s, 3700 mV, cap 0)
#       b=(-21.0097, 55.2707, 3D, 42 m, 1500 mm/s, 4100 mV, cap 123)
#       c=(-33.8571, -151.2153, 2D, 0, 278 mm/s, 2650 mV, cap 359.9)   n=NO_FIX
CPP_VECTORS = [
    ("097166C6600781E00003FE58", [(44.4, 12.3)], dict(day=9, hour=14, minute=11, batt_mv=4200, ooz=0, lb=0)),
    ("097166C6600781E00403FE58", [(44.4, 12.3)], dict(ooz=1, lb=0)),
    ("097166C6600781E00003FE5C", [(44.4, 12.3)], dict(ooz=0, lb=1)),
    ("0B6006A72C02616400000190", [(43.6012, 3.9001)], dict(day=11, hour=12, minute=0, alt=0, batt_mv=3700)),
    ("0B601334B121BC0C11580630", [(-21.0097, 55.2707)], dict(alt=1, heading=86, speed=2, batt_mv=4100)),
    ("0B60152A8BDC4B6403F7FC00", [(-33.8571, -151.2153)], dict(alt=255, heading=253, speed=0, batt_mv=2700)),
    ("0B601FFFFFFFFFFFFBFFFD68", [None], dict(batt_mv=3600)),
    ("0B601334B121BC0C12304D4E5804C2CFFFFFFFFFFF000057", [(-21.0097, 55.2707), (43.6012, 3.9001)], dict(dtl=1)),
    ("0B60152A8BDC4B640000666962437819A9CB0098590000DE",
     [(-33.8571, -151.2153), (-21.0097, 55.2707), (43.6012, 3.9001)], dict(dtl=1, batt_mv=2700)),
    ("0B6006A72C02616401907FFFFFFFFFFD4AA2F712D9000004", [(43.6012, 3.9001), None, (-33.8571, -151.2153)],
     dict(dtl=1)),
    ("0B6006A72C02616401907FFFFFFFFFFFFFFFFFFFFF0000E7", [(43.6012, 3.9001)], dict(dtl=1)),
    ("0B60152A8BDC4B640404E66962437819A9CB009859000068",
     [(-33.8571, -151.2153), (-21.0097, 55.2707), (43.6012, 3.9001)], dict(dtl=3, ooz=1, lb=1)),
    ("0B601334B121BC0C15580634", [(-21.0097, 55.2707)], dict(ooz=1, lb=1)),
]
CPP_LONG_PREFIXES = ["097166C6600781E002584D8CC00F03C7FFFFFFFFFF", "097166C6600781E002584D8CC00F03C1B19801E078"]


def selftest():
    ok = True

    def chk(cond, what):
        nonlocal ok
        print(f"  [{'ok' if cond else 'KO'}] {what}")
        ok = ok and cond

    print("1. decodeur contre les vecteurs du constructeur C++")
    for hx, pos, fields in CPP_VECTORS:
        d = decode_frame(hx)
        got = d["positions"]
        good = d["ok"] and len(got) == len(pos) and all(
            (a is None and b is None) or (a and b and abs(a[0] - b[0]) < 1e-4 and abs(a[1] - b[1]) < 1e-4)
            for a, b in zip(got, pos))
        good = good and all(d.get(k) == v for k, v in fields.items())
        chk(good, f"{hx[:24]}... {d['kind']} -> {[_fmt_pos(p) for p in got]} "
                  f"{ {k: d.get(k) for k in fields} }")
    for pre in CPP_LONG_PREFIXES:
        full = bytearray(bytes.fromhex(pre) + bytes(23 - len(pre) // 2) + b"\0")
        full[23] = DX1.crc8(bytes(full[:23]))
        d = decode_frame(full.hex())
        chk(d["ok"] and d["positions"][0] == (44.4, 12.3), f"LONG argos_tx_test {pre[:20]}... -> "
                                                            f"{[_fmt_pos(p) for p in d['positions']]}")
    print("2. encodeur miroir == C++ (sert au journal synthetique)")
    t = 1760184000
    a = dict(valid=True, lat=43.6012, lon=3.9001, t=t, fix3d=True, hmsl=10000, gspeed=0, batt=3700, head=0)
    b = dict(valid=True, lat=-21.0097, lon=55.2707, t=t, fix3d=True, hmsl=42000, gspeed=1500, batt=4100, head=123.0)
    c = dict(valid=True, lat=-33.8571, lon=-151.2153, t=t, fix3d=False, hmsl=0, gspeed=278, batt=2650, head=359.9)
    n = dict(valid=False, t=t, batt=3600)
    chk(encode_short(a).hex().upper() == CPP_VECTORS[3][0], "SHORT a")
    chk(encode_short(b).hex().upper() == CPP_VECTORS[4][0], "SHORT b")
    chk(encode_short(c).hex().upper() == CPP_VECTORS[5][0], "SHORT c")
    chk(encode_short(n).hex().upper() == CPP_VECTORS[6][0], "SHORT NO_FIX")
    chk(encode_long([b, a]).hex().upper() == CPP_VECTORS[7][0], "LONG [b,a]")
    chk(encode_long([c, b, a]).hex().upper() == CPP_VECTORS[8][0], "LONG [c,b,a]")
    chk(encode_long([a, n, c]).hex().upper() == CPP_VECTORS[9][0], "LONG [a,NOFIX,c]")
    chk(encode_long([c, b, a], 1, 1, 3).hex().upper() == CPP_VECTORS[11][0], "LONG hors zone + LB + 30 min")
    print("3. Doppler / DX1 (dx1_ref.py, vecteurs firmware tests/src/dx1_doppler_test.cpp)")
    chk(decode_frame("00640000")["kind"] == "doppler" and decode_frame("00640000")["batt_mv"] == 3700,
        "Doppler historique 006400 bourre a 4 o (3700 mV)")
    img, _ = DX1.view(DX1.image(3700, False, 0, 1), "LDA2")
    d = decode_frame(img.hex())
    chk(d["kind"] == "dx1" and d["crc_ok"] and d["dend"] == 1 and d["dry"] == 0, f"DX1 LDA2 {img.hex()} -> {d}")
    bad = bytearray(img)
    bad[3] ^= 1
    chk(not decode_frame(bytes(bad).hex())["crc_ok"], "DX1 CRC corrompu detecte")
    print("4. modele DepthPile (sequences attendues)")
    for ph in (0, 1):
        print(f"   phase {ph}:")
        print(f"     sb0  : {simulate(5, 0, [1, 'dive-before', 1], phase=ph)}")
        print(f"     sbn3 : {simulate(5, 3, [2] * 5, phase=ph)}")
        print(f"     legn3: {simulate(2, 3, [2] * 5, phase=ph)}")
        print(f"     leg0 : {simulate(2, 0, [2] * 5, phase=ph)}")
    chk(simulate(5, 0, [1, "dive-before", 1]) == [[["P1"]], [], [["P3", "P2"]]], "sb0 = [P1] ; - ; [P3,P2]")
    chk(simulate(5, 3, [2] * 5) == simulate(2, 3, [2] * 5), "SB NTRY=3 == LEGACY NTRY=3 a interleaving egal")
    leg0 = simulate(2, 0, [2] * 5)
    chk(leg0[3] == [["P1"], ["P4", "P3", "P2"]] and leg0[4] == [["P2"], ["P5", "P4", "P3"]],
        "LEGACY illimite : alternance creneau 1 (plus ancien) / creneau 0 (3 plus recents)")
    print("5. rejeu d un journal synthetique par l analyse (pile sbn, burst)")
    ok = _synthetic_replay(chk) and ok
    print("SELFTEST", "OK" if ok else "ECHEC")
    return 0 if ok else 1


def _synthetic_replay(chk):
    """Fabrique les lignes que le firmware emettrait (formats reels) a partir du
    modele, puis les rend a l analyse : verifie le parsing et l appariement."""
    lines, t = [], 100.0
    t_epoch = 1760184000

    def emit(s, dt=0.5):
        nonlocal t
        t += dt
        lines.append((t, s))

    model = PileModel(1)
    model.set_max_size(4)
    inj = []
    store = {}
    emit("[turtle] RUN sbn {}")
    emit("entry: OperationalState")
    model.clear()
    for k in range(1, 5):
        lat, lon = round(43.6 + 0.01 * k, 4), round(3.9 + 0.01 * k, 4)
        inj.append((f"P{k}", lat, lon))
        store[f"P{k}"] = dict(valid=True, lat=lat, lon=lon, t=t_epoch + 100 * k, fix3d=True, hmsl=10000,
                              batt=3700)
        emit("%SURFACE OK surfaced uptime=1000")
        emit("[VAL-TX] SmdSatCmdAt::initiate_tx: AT+TX=00640000)")
        emit("[VAL-TX] type=doppler-prewarm t=1 spacing_s=0 burst=on dop_count=1 session#=1")
        emit(f"%GPS OK lat={lat:.6f} lon={lon:.6f}")
        emit("DepthPileManager: GPS FIX stored, burst_counter=3 (0xFFFFFFFF=unlimited)")
        model.store(f"P{k}", "fix", 3)
        for _ in range(2):
            labs = frame_order(model.retrieve(4))
            ents = [store[x] for x in labs]
            hx = (encode_short(ents[0]) if len(ents) == 1 else encode_long(ents)).hex().upper()
            emit(f"ArgosPacketBuilder::build_gnss_packet: {'SHORT' if len(ents) == 1 else 'LONG'} packet, "
                 f"{len(ents)} position(s), {96 if len(ents) == 1 else 192} bits")
            emit(f"[VAL-TX] SmdSatCmdAt::initiate_tx: AT+TX={hx})")
            emit("SmdSat: TXSTART ms=1,2,3 uptime=2000 (power_on,...)")
            emit("[VAL-TX] type=gnss t=2 spacing_s=30 burst=on dop_count=1 session#=2", 2.0)
            emit(f"%PILE {model.dump()}")
        emit("%DIVE OK underwater uptime=5000")
    emit("[turtle] END sbn")
    res = Results()
    run = dict(name="sbn", mode=5, ntry=3, depth=4, injected=inj, cycles_done=4)
    out = analyse_pile_run(lines, run, res)
    good = all(r["verdict"] in ("PASS", "INFO") for r in res.rows) and 1 in out["alive"]
    chk(good, f"pile synthetique : verdicts {[(r['id'], r['verdict']) for r in res.rows]} phases {out['alive']}")
    # une trame trafiquee doit etre detectee
    lines2 = [(tt, l.replace(encode_short(store["P1"]).hex().upper(), encode_short(store["P2"]).hex().upper()))
              for tt, l in lines]
    res2 = Results()
    analyse_pile_run(lines2, run, res2)
    chk(any(r["verdict"] == "FAIL" for r in res2.rows), "pile synthetique trafiquee -> FAIL")
    # burst synthetique
    lb, t = [], 0.0

    def e2(s, dt):
        nonlocal t
        t += dt
        lb.append((t, s))

    cfg = dict(init=5, step=10, max=60, max_msg=6, tr_nom=60, depth=4, ntry=0, mode=5)
    fix = dict(valid=True, lat=43.61, lon=3.87, t=int(time.time()) - 20, fix3d=True, hmsl=10000, batt=3900)
    e2("%SURFACE OK surfaced uptime=10000", 0)
    e2("ArgosTxService::process_doppler_burst: PREWARM mode=LDA2 sz=24 age=60000 ms", 1.0)
    e2("[VAL-TX] SmdSatCmdAt::initiate_tx: AT+TX=00640000)", 0.5)
    e2("SmdSat: TXSTART ms=1,2 uptime=11900 (power_on,...)", 0.4)
    e2("[VAL-TX] type=doppler-prewarm t=1 spacing_s=0 burst=on dop_count=1 session#=1", 1.0)
    e2("[VAL-TX] SmdSatCmdAt::initiate_tx: AT+TX=00640000)", 6.5)
    e2("[VAL-TX] type=doppler t=2 spacing_s=7 burst=on dop_count=2 session#=2", 1.0)
    e2("ArgosTxService::SURFACING_BURST: GNSS fix acquired after 2 Doppler messages - switching to GNSS phase", 4.0)
    e2("ArgosPacketBuilder::build_gnss_packet: SHORT packet, 1 position, 96 bits", 3.0)
    e2(f"[VAL-TX] SmdSatCmdAt::initiate_tx: AT+TX={encode_short(fix).hex().upper()})", 0.5)
    e2("[VAL-TX] type=gnss t=3 spacing_s=6 burst=on dop_count=2 session#=3", 1.0)
    e2("ArgosTxService: burst finished, waiting for the next surfacing - presence heartbeat", 0.2)
    for _ in range(2):
        e2(f"[VAL-TX] SmdSatCmdAt::initiate_tx: AT+TX={encode_short(fix).hex().upper()})", 60.0)
        e2("[VAL-TX] type=short-cached t=4 spacing_s=60 burst=off dop_count=2 session#=4", 1.0)
    res3 = Results()
    now = datetime.datetime.now(datetime.timezone.utc)
    analyse_burst(lb, cfg, res3, (43.61, 3.87, 1.0), lambda _t: now)
    chk(all(r["verdict"] in ("PASS", "SKIP") for r in res3.rows),
        f"burst synthetique : {[(r['id'], r['verdict']) for r in res3.rows]}")
    return good


# ============================================================================
#  Rejeu d un journal capture
# ============================================================================
def replay(path, scenario, args):
    lines = parse_log_file(path)
    res = Results()
    cfg = None
    for _, l in lines:
        m = RX_MARK.search(l)
        if m and m.group(1) == "CFG" and m.group(2).startswith("{"):
            try:
                cfg = json.loads(m.group(2))
            except ValueError:
                pass
    if scenario == "burst":
        cfg = cfg or burst_cfg_from(burst_profile())
        starts = [i for i, (_, l) in enumerate(lines) if "[turtle] RUN burst" in l] or [0]
        for k, s in enumerate(starts):
            e = starts[k + 1] if k + 1 < len(starts) else len(lines)
            analyse_burst(lines[s:e], cfg, res, None, None)
    elif scenario == "pile":
        runs = [(i, l) for i, (_, l) in enumerate(lines) if RX_MARK.search(l) and "[turtle] RUN " in l]
        results = {}
        for i, l in runs:
            name, spec = l.split("[turtle] RUN ", 1)[1].split(" ", 1)
            spec = json.loads(spec)
            end = next((j for j in range(i, len(lines)) if f"[turtle] END {name}" in lines[j][1]), len(lines))
            seg = lines[i:end + 1]
            inj = []
            for _, x in seg:
                m = re.search(r"\[turtle\] CYCLE \S+ (\d+)", x)
                if m:
                    k = int(m.group(1))
                    inj.append((f"P{k}", round(args.base_lat + 0.01 * k, 4), round(args.base_lon + 0.01 * k, 4)))
            spec.update(name=name, injected=inj, cycles_done=len(inj))
            results[name] = analyse_pile_run(seg, spec, res)
        compare_pile_runs(results, res, args)
    elif scenario == "dx1":
        starts = [i for i, (_, l) in enumerate(lines) if "[turtle] RUN dx1" in l]
        allf = []
        for k, s in enumerate(starts):
            e = starts[k + 1] if k + 1 < len(starts) else len(lines)
            seg = lines[s:e]
            i_s = next((i for i, (_, l) in enumerate(seg) if RX_SURF.search(l)), 0)
            allf.append((k + 1, extract_frames(seg[i_s:])[:1]))
        analyse_dx1(allf, res)
    return _finish(res)


def _finish(res):
    c = res.summary()
    print("\nRESUME : " + ", ".join(f"{k}={v}" for k, v in sorted(c.items())), flush=True)
    return 1 if c.get("FAIL") else 0


# ============================================================================
def main():
    P.setup_stdio()
    ap = argparse.ArgumentParser(description="Mission tortue SURFACING_BURST, emission Argos reelle (SMD UART-AT)")
    ap.add_argument("scenario", nargs="?", choices=("burst", "pile", "dx1"))
    ap.add_argument("--plan", action="store_true", help="afficher le plan d essai et sortir")
    ap.add_argument("--selftest", action="store_true", help="auto-test hors ligne (aucune carte)")
    ap.add_argument("--replay", metavar="LOG", help="re-analyser un journal tests/bench/logs/gnss_*.log")
    ap.add_argument("--cycles", type=int, default=None, help="burst: 1 ; pile sbn/legn: 5 ; dx1: 3")
    ap.add_argument("--dive-s", type=int, default=None, help="duree de plongee (burst 120 s, dx1 60 s)")
    ap.add_argument("--observe-s", type=int, default=420, help="burst: observation apres emersion (s)")
    ap.add_argument("--ref-lat", type=float, help="burst: latitude connue du banc")
    ap.add_argument("--ref-lon", type=float, help="burst: longitude connue du banc")
    ap.add_argument("--ref-tol-km", type=float, default=1.0)
    ap.add_argument("--runs", default="sb0,sbn,legn", help="pile: passes parmi sb0,sbn,legn,leg0")
    ap.add_argument("--ntry", type=int, default=2, help="pile: NTRY des passes sbn/legn (templates tortue: 2)")
    ap.add_argument("--frames", type=int, default=2, help="pile: trames GNSS par emersion avant plongee")
    ap.add_argument("--pile-dive-s", type=int, default=90,
                    help="pile: plongee entre emersions (garder > 60 s : un fix de moins de 60 s dans la pile "
                         "promeut la salve en phase GNSS des le ping 1, should_promote_doppler_to_gnss)")
    ap.add_argument("--base-lat", type=float, default=43.60)
    ap.add_argument("--base-lon", type=float, default=3.90)
    ap.add_argument("--max-tx", type=int, default=None, help="borne de trames AT+TX du scenario")
    ap.add_argument("--leave-armed", action="store_true", help="ne pas remettre ARGOS_MODE=0 a la fin")
    args = ap.parse_args()

    if args.plan:
        print(PLAN)
        return 0
    if args.selftest:
        return selftest()
    if args.replay:
        if not args.scenario:
            ap.error("--replay exige le scenario (burst|pile|dx1)")
        return replay(args.replay, args.scenario, args)
    if not args.scenario:
        ap.error("scenario requis (burst|pile|dx1), ou --plan / --selftest / --replay")
    if os.environ.get("DTE_RF_AUTORISE") != "1":
        print("REFUS : ce script EMET pour de vrai. Poser DTE_RF_AUTORISE=1 (antenne branchee, emission voulue "
              "par l operateur) puis relancer. Voir --plan.", flush=True)
        return 2
    if args.cycles is None:
        args.cycles = {"burst": 1, "pile": 5, "dx1": 3}[args.scenario]
    if args.dive_s is None:
        args.dive_s = {"burst": 120, "pile": 60, "dx1": 60}[args.scenario]

    sink = Sink()
    res = Results(sink)
    rig = None
    code = 1
    try:
        rig = Rig(args)
        sink.jsonl({"type": "start", "scenario": args.scenario, "args": vars(args), "port": rig.b.port,
                    "log": rig.b.logpath})
        rig.ping_operational()
        {"burst": run_burst, "pile": run_pile, "dx1": run_dx1}[args.scenario](rig, args, res, sink)
        code = _finish(res)
    except BenchError as e:
        print(f"\nARRET : {e}", flush=True)
        res.add(args.scenario, "ABORT", "scenario interrompu", "FAIL", str(e))
        _finish(res)
        code = 1
    except KeyboardInterrupt:
        print("\ninterrompu", flush=True)
        code = 130
    finally:
        if rig is not None:
            rig.budget = None
            rig.safe_off(persist=not args.leave_armed)
            rig.close()
        sink.close()
        print(f"sorties : {sink.base}.jsonl / .csv", flush=True)
    return code


if __name__ == "__main__":
    sys.exit(main())
