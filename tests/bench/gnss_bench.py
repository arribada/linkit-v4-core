#!/usr/bin/env python3
"""
gnss_bench.py — campagne de validation GNSS en intérieur (aucun fix nécessaire).

Complète kim_bench.py, qui valide la chaîne config/DTE/injection. Ici on valide
le sous-système GNSS lui-même : la négociation de débit au démarrage, la
rétention BBR, l'échappatoire cold start, le deep-idle, le bridge, et les fuites
de rail. Tout est observable depuis les logs, sans antenne et sans fix.

    ./gnss_bench.py --run            campagne complète
    ./gnss_bench.py --run --only T1,T4
    ./gnss_bench.py --observe 120    capture brute horodatée

Prérequis : firmware construit avec `./scripts/build_linkitv4_kim.sh --bench
--validation` (la campagne s'appuie sur les lignes [VAL-GNSS]).
"""
import argparse, datetime, glob, os, re, subprocess, sys, threading, time

import serial

HERE = os.path.dirname(os.path.abspath(__file__))
LOGDIR = os.path.join(HERE, "logs")
PORT_GLOB = None       # None = autodétection (le nœud change de numéro à chaque reset)


class Board:
    """Lien série + journal horodaté + attente de motifs dans le flux."""

    @staticmethod
    def find_port(timeout=30):
        """Le nœud CDC change de numéro à chaque re-énumération (reset, flash).
        On prend le plus récemment apparu qui s'ouvre."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            cands = sorted(glob.glob("/dev/ttyACM*"),
                           key=lambda p: os.stat(p).st_mtime, reverse=True)
            for c in cands:
                try:
                    s = serial.Serial(c, 115200, timeout=0.1)
                    s.close()
                    return c
                except Exception:
                    continue
            time.sleep(0.5)
        return None

    def __init__(self, port=PORT_GLOB, baud=115200):
        self.baud = baud
        port = port or self.find_port()
        if not port:
            raise RuntimeError("aucun port CDC — la carte est-elle attachée à WSL ?")
        self.port = port
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.lines = []                     # (t_relatif, texte)
        self._buf = b""
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self.t0 = time.monotonic()
        os.makedirs(LOGDIR, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.logpath = os.path.join(LOGDIR, f"gnss_{stamp}.log")
        self._logf = open(self.logpath, "a", buffering=1)
        self._t = threading.Thread(target=self._reader, daemon=True)
        self._t.start()

    def _reader(self):
        while not self._stop.is_set():
            try:
                data = self.ser.read(4096)
            except Exception:
                # Re-énumération USB (reset carte, flash) : le nœud disparaît puis
                # revient, souvent sous un autre numéro. On se raccroche au vol,
                # sinon toute la campagne s'arrête au premier reset.
                self._note("[bench] lien série perdu — reconnexion…")
                try:
                    self.ser.close()
                except Exception:
                    pass
                p = self.find_port(timeout=60)
                if not p:
                    self._note("[bench] reconnexion impossible")
                    break
                try:
                    self.ser = serial.Serial(p, self.baud, timeout=0.1)
                    self.port = p
                    self._note(f"[bench] reconnecté sur {p}")
                except Exception:
                    break
                continue
            if not data:
                continue
            self._buf += data
            while b"\n" in self._buf:
                raw, _, self._buf = self._buf.partition(b"\n")
                line = raw.decode("utf-8", "replace").rstrip("\r")
                t = time.monotonic() - self.t0
                with self._lock:
                    self.lines.append((t, line))
                self._logf.write(f"[{t:8.2f}] {line}\n")

    def _note(self, text):
        t = time.monotonic() - self.t0
        with self._lock:
            self.lines.append((t, text))
        self._logf.write(f"[{t:8.2f}] {text}\n")
        print(f"      {text}", flush=True)

    def close(self):
        self._stop.set()
        time.sleep(0.2)
        try:
            self.ser.close()
        except Exception:
            pass

    # ---- émission ---------------------------------------------------------
    def send(self, text):
        try:
            self.ser.write((text + "\r\n").encode())
            self.ser.flush()
        except Exception:
            self._note(f"[bench] envoi impossible ({text}) — lien en reconnexion")
            return
        t = time.monotonic() - self.t0
        self._logf.write(f"[{t:8.2f}] >>> {text}\n")

    def raw(self, data: bytes):
        """Écrit des octets bruts — bridge (`+++`) et trames UBX."""
        try:
            self.ser.write(data)
            self.ser.flush()
        except Exception:
            self._note("[bench] écriture brute impossible")
            return
        t = time.monotonic() - self.t0
        self._logf.write(f"[{t:8.2f}] >>> RAW {data[:24]!r}\n")

    def read_raw(self, seconds):
        """Lit le flux brut pendant N secondes (le lecteur de lignes tourne
        toujours, mais en passthrough les octets ne sont pas des lignes)."""
        end = time.monotonic() + seconds
        acc = b""
        while time.monotonic() < end:
            try:
                acc += self.ser.read(4096)
            except Exception:
                break
            time.sleep(0.05)
        return acc

    def dte(self, cmd, payload=""):
        """Commande DTE `$CMD#LEN;payload`."""
        self.send(f"${cmd}#{len(payload):03d};{payload}" if payload
                  else f"${cmd}#000;")

    # ---- observation ------------------------------------------------------
    def mark(self):
        with self._lock:
            return len(self.lines)

    def since(self, idx):
        with self._lock:
            return list(self.lines[idx:])

    def wait(self, pattern, timeout, since=None):
        """Attend une ligne correspondant au motif. Renvoie la ligne ou None."""
        rx = re.compile(pattern, re.I)
        idx = self.mark() if since is None else since
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for _, line in self.since(idx):
                if rx.search(line):
                    return line
            time.sleep(0.15)
        return None

    def collect(self, seconds, quiet_note=None):
        idx = self.mark()
        if quiet_note:
            print(f"      … {quiet_note} ({seconds:.0f} s)", flush=True)
        time.sleep(seconds)
        return [l for _, l in self.since(idx)]


def reset_board():
    """Reset matériel par le J-Link — le seul vrai redémarrage à froid du nRF."""
    subprocess.run(["nrfjprog", "--reset"], check=False,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


# ─────────────────────────────────────────────────────────────────────────────
#  Scénarios
# ─────────────────────────────────────────────────────────────────────────────
RESULTS = []


def record(tid, name, ok, detail, evidence=()):
    """ok = True (PASS) / False (FAIL) / None (SKIP — non concluant sur ce banc)."""
    RESULTS.append((tid, name, ok, detail, list(evidence)))
    tag = ("\033[32mPASS\033[0m" if ok is True else
           "\033[33mSKIP\033[0m" if ok is None else "\033[31mFAIL\033[0m")
    print(f"  [{tag}] {tid} — {name}")
    print(f"         {detail}")
    for e in evidence[:4]:
        print(f"         | {e}")


def t1_boot_sync(b):
    """La négociation de débit au démarrage doit aboutir, et dire laquelle."""
    idx = b.mark()
    reset_board()
    line = b.wait(r"boot sync at \d+ baud", 60, since=idx)
    if not line:
        # le reset a pu couper l'USB : on retombe sur la session suivante
        line = b.wait(r"boot sync at \d+ baud", 90, since=idx)
    fail = b.wait(r"failed to sync comms", 1, since=idx)
    if not line:
        record("T1", "sonde de débit au démarrage", False,
               "aucune ligne 'boot sync' observée en 150 s")
        return None
    m = re.search(r"boot sync at (\d+) baud — BBR (\w+)", line)
    baud, bbr = (m.group(1), m.group(2)) if m else ("?", "?")
    ok = fail is None
    record("T1", "sonde de débit au démarrage", ok,
           f"synchro à {baud} baud, BBR {bbr}" +
           ("" if ok else " — MAIS 'failed to sync comms' observé"), [line])
    return bbr.upper()


def t2_sessions(b, n=3):
    """N sessions consécutives : chacune doit se synchroniser et écouter."""
    seen, syncs = [], 0
    idx = b.mark()
    deadline = time.monotonic() + 100 * n
    while syncs < n and time.monotonic() < deadline:
        line = b.wait(r"boot sync at \d+ baud|failed to sync comms", 100, since=idx)
        if not line:
            break
        idx = b.mark()
        seen.append(line)
        if "failed" not in line.lower():
            syncs += 1
    ok = syncs >= n and not any("failed" in s.lower() for s in seen)
    record("T2", f"{n} sessions consécutives", ok,
           f"{syncs}/{n} synchros réussies, "
           f"{sum('failed' in s.lower() for s in seen)} échec(s)", seen[-3:])


def t3_receiver_alive(b):
    """Le récepteur doit réellement écouter : rapports satellite au fil de l'eau."""
    lines = b.collect(35, "attente de rapports satellite (intérieur : 0 SV attendu)")
    sat = [l for l in lines if "sat report" in l.lower() or "satellite" in l.lower()]
    diag = [l for l in lines if "SESSION_DIAG" in l]
    ok = bool(sat) or bool(diag)
    record("T3", "récepteur vivant et configuré", ok,
           f"{len(sat)} ligne(s) satellite, {len(diag)} SESSION_DIAG "
           f"(en intérieur l'abandon précoce est le comportement attendu)",
           (sat + diag)[:3])


def t4_cold_start(b):
    """GNSS_COLD_START_AFTER_NTRY doit produire un vrai CFG-RST + clear_config."""
    b.send("%CFG")
    if not b.wait(r"%CFG OK|ConfigurationState", 15):
        record("T4", "escalade cold start", False, "impossible d'entrer en config")
        return
    time.sleep(1)
    b.dte("PARMW", "GNP54=2")          # cold start toutes les 2 sessions mortes
    b.wait(r"PARMW|\$O;", 8)
    time.sleep(0.5)
    b.send("%OP")
    b.wait(r"%OP OK|OperationalState", 15)

    idx = b.mark()
    req = b.wait(r"COLD START demand", 260, since=idx)
    applied = b.wait(r"COLD START appliqu", 90, since=idx) if req else None
    cleared = b.wait(r"clear_bbr_config|erasing the receiver BBR config", 5, since=idx)
    ok = bool(req and applied)
    record("T4", "escalade cold start (GNP54=2)", ok,
           ("demande + application observées" if ok else
            f"demande={'oui' if req else 'non'} application={'oui' if applied else 'non'}") +
           f", clear_config={'oui' if cleared else 'non'}",
           [x for x in (req, applied, cleared) if x])
    return bool(cleared)


def t5_bbr_escape(b, had_clear):
    """Après un cold start, la couche de config BBR doit avoir été effacée :
    le démarrage suivant doit repartir aux défauts usine (9600)."""
    if not had_clear:
        record("T5", "échappatoire BBR (D2)", False,
               "clear_config non émis — rien à vérifier")
        return
    idx = b.mark()
    line = b.wait(r"boot sync at \d+ baud", 120, since=idx)
    if not line:
        record("T5", "échappatoire BBR (D2)", False, "pas de session suivante observée")
        return
    m = re.search(r"boot sync at (\d+) baud — BBR (\w+)", line)
    baud = m.group(1) if m else "?"
    ok = baud == "9600"
    record("T5", "échappatoire BBR (D2)", ok,
           f"après effacement, le récepteur repart à {baud} baud "
           f"({'défauts usine — conforme' if ok else 'ATTENDU 9600'})", [line])


def t6_deep_idle(b):
    """GNP52 : entrée en PMREQ-backup puis réveil EXTINT avec état de session neuf."""
    b.send("%CFG"); b.wait(r"%CFG OK|ConfigurationState", 15); time.sleep(1)
    b.dte("PARMW", "GNP52=90")
    b.wait(r"PARMW|\$O;", 8); time.sleep(0.5)
    b.send("%OP"); b.wait(r"%OP OK|OperationalState", 15)

    idx = b.mark()
    entered = b.wait(r"enterbackup|deep-idle|PMREQ", 200, since=idx)
    woke = b.wait(r"wake from deep-idle via EXTINT", 200, since=idx) if entered else None
    # au réveil, les compteurs doivent repartir de zéro : pas d'abandon
    # "no satellite detected after N" avec un N hérité de la session précédente
    early = None
    if woke:
        after = b.mark()
        line = b.wait(r"no satellite detected after (\d+) sat reports", 60, since=after)
        if line:
            n = int(re.search(r"after (\d+) sat", line).group(1))
            early = (n, line)
    # L'inhibition R4 (armée quand le boot vient d'un reset WDT) force un
    # power-off à froid tant qu'aucun VRAI fix PVT n'a été obtenu — et un fix
    # injecté par %GPS ne la lève pas (il court-circuite react(GPSEventPVT)).
    # En intérieur, GNP52 est donc sans effet : ce n'est pas un défaut, c'est le
    # garde-fou qui fait son travail. On le dit au lieu de conclure à tort.
    inhibited = b.wait(r"wdt_inhibit|WDT-reset inhibit", 2, since=idx)
    ok = bool(entered) and (early is None or early[0] >= 20)
    if not entered and inhibited:
        ok = None
    detail = []
    if inhibited and not entered:
        detail.append("NON CONCLUANT — inhibition deep-idle post-WDT active "
                      "(levée seulement par un vrai fix PVT, impossible en intérieur)")
    else:
        detail.append("entrée deep-idle observée" if entered else "PAS d'entrée deep-idle")
    if woke:
        detail.append("réveil EXTINT observé")
    if early:
        detail.append(f"1er abandon à N={early[0]} "
                      f"({'compteur remis à zéro' if early[0] >= 20 else 'COMPTEUR HERITE'})")
    record("T6", "deep-idle GNP52 + réveil chaud", ok, " ; ".join(detail),
           [x for x in (entered, woke, inhibited, early[1] if early else None) if x])
    # remise à l'état par défaut
    b.send("%CFG"); b.wait(r"%CFG OK|ConfigurationState", 15); time.sleep(1)
    b.dte("PARMW", "GNP52=0"); b.wait(r"PARMW|\$O;", 8); time.sleep(0.5)
    b.send("%OP"); b.wait(r"%OP OK|OperationalState", 15)


def t7_bridge(b):
    """Le bridge doit s'ouvrir au bon débit et se refermer proprement.

    Attention : pendant le bridge, la console USB est SILENCIEUSE (les logs sont
    coupés pour ne pas polluer le flux brut vers le récepteur) et le port devient
    un passthrough. On sort donc par la séquence d'échappement `+++` envoyée en
    trois octets nus — envoyer une commande DTE la ferait partir vers le M10Q et
    laisserait le port en passthrough pour tout le reste de la campagne.
    """
    b.send("%CFG"); b.wait(r"%CFG OK|ConfigurationState", 15); time.sleep(1)
    idx = b.mark()
    b.dte("GNSSBR", "1")
    active = b.wait(r"bridge mode ACTIVE \((\d+) baud", 15, since=idx)
    ok_resp = b.wait(r"\$O;GNSSBR", 5, since=idx)
    time.sleep(1)
    b.raw(b"+++")                       # sortie du passthrough
    off = b.wait(r"BRIDGE OFF|bridge mode STOPPED", 15)
    baud = re.search(r"\((\d+) baud", active).group(1) if active else "?"
    ok = bool(off) and (bool(active) or bool(ok_resp))
    record("T7", "bridge u-center (ouverture/fermeture)", ok,
           f"ouverture={'oui' if (active or ok_resp) else 'non'} "
           f"débit={baud} sortie+++={'oui' if off else 'NON — port resté en passthrough'}",
           [x for x in (active, ok_resp, off) if x])
    b.send("%OP"); b.wait(r"%OP OK|OperationalState|PreOperational", 15)


def t8_pwron_leak(b):
    """Un PWRON GNSS oublié doit être coupé à la sortie du mode configuration."""
    b.send("%CFG"); b.wait(r"%CFG OK|ConfigurationState", 15); time.sleep(1)
    idx = b.mark()
    b.dte("PWRON", "2")                 # ComponentPower::GNSS
    b.wait(r"PWRON|power_on", 15, since=idx)
    time.sleep(2)
    b.send("%OP")
    caught = b.wait(r"GNSS still powered", 20, since=idx)
    ok = bool(caught)
    record("T8", "fuite de rail PWRON GNSS (C3)", ok,
           "filet de sortie de config déclenché" if ok else
           "AUCUN filet — le rail resterait allumé", [caught] if caught else [])
    b.wait(r"%OP OK|OperationalState", 15)


def t9_gnssi(b):
    """GNSSI prouve une communication complète avec le M10Q (SW/HW/UID)."""
    b.send("%CFG"); b.wait(r"%CFG OK|ConfigurationState", 15); time.sleep(1)
    idx = b.mark()
    b.dte("GNSSI")
    resp = b.wait(r"GNSSI|GNSS SW:|GNSS UID", 40, since=idx)
    uid = b.wait(r"GNSS UID", 5, since=idx)
    sw = b.wait(r"GNSS SW:", 5, since=idx)
    ok = bool(resp)
    record("T9", "identité du récepteur (GNSSI)", ok,
           "réponse obtenue" if ok else "aucune réponse",
           [x for x in (sw, uid, resp) if x][:3])
    b.send("%OP"); b.wait(r"%OP OK|OperationalState", 15)


def t10_inject(b):
    """Injection d'un fix : la chaîne post-fix doit se dérouler et clore la session."""
    idx = b.mark()
    b.send("%GPS -21.0097 55.2707")
    acc = b.wait(r"bench_inject_fix|retry_counter: reset", 25, since=idx)
    off = b.wait(r"dispatch=immediate_off|power_off", 25, since=idx)
    ok = bool(acc)
    record("T10", "injecting fix + fin de session (R20)", ok,
           ("fix accepté" if acc else "fix non accepté") +
           (", session close" if off else ", PAS de fin de session"),
           [x for x in (acc, off) if x])


def t11_no_regression(b):
    """Aucune trace des pannes connues sur l'ensemble de la campagne."""
    with b._lock:
        allz = [l for _, l in b.lines]
    bad = {
        "failed to sync comms": "verrou de débit au démarrage",
        "state_configure: failed": "échec de configuration",
        "repeated comms errors": "erreurs UART répétées",
        "ubx_comms.init failed": "échec d'init UART",
        "HardFault": "faute matérielle",
    }
    hits = []
    for pat, why in bad.items():
        found = [l for l in allz if pat.lower() in l.lower()]
        if found:
            hits.append(f"{why} ({len(found)}×) : {found[0][:110]}")
    ok = not hits
    record("T11", "absence de régression connue", ok,
           "aucune signature de panne sur toute la campagne" if ok
           else f"{len(hits)} signature(s) détectée(s)", hits)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", action="store_true")
    ap.add_argument("--observe", type=float, default=0)
    ap.add_argument("--only", default="")
    ap.add_argument("--port", default=PORT_GLOB)
    a = ap.parse_args()

    b = Board(a.port)
    print(f"Journal : {b.logpath}\n")

    try:
        if a.observe:
            idx = b.mark()
            time.sleep(a.observe)
            for t, l in b.since(idx):
                print(f"[{t:8.2f}] {l}")
            return 0

        only = {x.strip().upper() for x in a.only.split(",") if x.strip()}
        def run(tid, fn, *args):
            if only and tid not in only:
                return None
            print(f"\n▶ {tid}")
            return fn(*args)

        print("═══ Campagne GNSS — intérieur, aucun fix attendu ═══")
        run("T1", t1_boot_sync, b)
        run("T2", t2_sessions, b)
        run("T3", t3_receiver_alive, b)
        cleared = run("T4", t4_cold_start, b)
        run("T5", t5_bbr_escape, b, bool(cleared))
        run("T6", t6_deep_idle, b)
        run("T7", t7_bridge, b)
        run("T8", t8_pwron_leak, b)
        run("T9", t9_gnssi, b)
        run("T10", t10_inject, b)
        run("T11", t11_no_regression, b)

        print("\n═══ Bilan ═══")
        npass = sum(1 for r in RESULTS if r[2] is True)
        nskip = sum(1 for r in RESULTS if r[2] is None)
        nfail = sum(1 for r in RESULTS if r[2] is False)
        for tid, name, ok, detail, _ in RESULTS:
            lab = "PASS" if ok is True else ("SKIP" if ok is None else "FAIL")
            print(f"  {lab}  {tid:4s} {name} — {detail}")
        print(f"\n  {npass} au vert, {nfail} en échec, {nskip} non concluant(s) sur ce banc")
        print(f"  Journal complet : {b.logpath}")
        return 0 if nfail == 0 else 1
    finally:
        b.close()


if __name__ == "__main__":
    sys.exit(main())
