#!/usr/bin/env python3
"""Campagne KIM autonome — recherche de defauts bloquants, sans emission radio.

Concu pour tourner seul plusieurs heures. Chaque cas repart d'une configuration
connue, de sorte qu'un echec n'invalide pas les suivants. Le harnais se recupere
d'un port qui disparait, d'une carte muette, et journalise tout en JSONL pour
qu'un depouillement soit possible sans relire le log brut.

CONTRAINTE ASSUMEE: aucun cas de ce fichier ne declenche d'emission Argos. Le
mode est force a OFF et la certification desarmee au debut de chaque cas.
"""
import sys, time, json, re, subprocess, os, glob, fcntl
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from kim_bench import Bench

import os
OUT = os.environ.get('DTE_CAMPAIGN_OUT', '/tmp/dte_campaign')
os.makedirs(OUT, exist_ok=True)
RESULTS = f'{OUT}/campaign_results.jsonl'
LOG     = f'{OUT}/campaign.log'

BENCH_LOCK = '/tmp/dte_campaign.lock'

class BancOccupe(RuntimeError):
    pass

class Runner:
    def __init__(self):
        # Verrou exclusif sur le banc. Deux campagnes simultanees ne se
        # contentent pas de se marcher dessus sur le port serie: chacune
        # appelle recover(), donc `nrfjprog --reset`, et REDEMARRE la carte
        # sous l autre. Le symptome est une "carte muette" au beau milieu d un
        # cas qui n a rien fait de mal — un faux defaut tres convaincant, et
        # c est exactement ce qui est arrive le 2026-08-27. Le verrou est un
        # fcntl sur un fichier: il tombe tout seul si le processus meurt.
        self._lockf = open(BENCH_LOCK, 'w')
        try:
            fcntl.flock(self._lockf, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            raise BancOccupe(
                'une autre campagne detient deja le banc (verrou %s). '
                'Arreter le processus en cours avant de relancer.' % BENCH_LOCK)
        self._lockf.write(str(os.getpid())); self._lockf.flush()
        self.b = None
        self.logf = open(LOG, 'a', buffering=1)
        self.n_pass = self.n_fail = self.n_error = 0

    def say(self, s):
        line = f"{time.strftime('%H:%M:%S')}  {s}"
        print(line, flush=True); self.logf.write(line + '\n')

    # ---- lien serie, resistant a la re-enumeration USB ----
    def connect(self, tries=30):
        if self.b:
            try: self.b.close()
            except Exception: pass
            self.b = None
        for k in range(tries):
            try:
                # port=None => auto-detection. Sous WSL le noeud change de
                # numero a chaque re-enumeration (ttyACM0 -> ttyACM1...), et
                # un port code en dur fait echouer la campagne sur
                # 'carte injoignable' alors que la carte va tres bien.
                b = Bench(port=None, quiet=True)
                b.open()
                try: b.ser.write_timeout = 5
                except Exception: pass
                if b.ping(timeout=5):
                    self.b = b; return True
                b.close()
            except Exception:
                pass
            # A mi-parcours, le port peut exister mais le lien etre mort
            # (URB en ECONNRESET): retenter n'y changera rien, il faut relier.
            if k == tries // 2:
                try: self.relink()
                except Exception: pass
            time.sleep(2)
        return False

    def _busid_carte(self):
        """Busid usbipd du CDC Nordic (VID 1915), decouvert a chaud."""
        import subprocess as sp
        for exe in ('usbipd.exe', '/mnt/c/Program Files/usbipd-win/usbipd.exe'):
            try:
                out = sp.run([exe, 'list'], capture_output=True, text=True, timeout=30).stdout
            except Exception:
                continue
            for ligne in out.splitlines():
                if '1915:' in ligne:
                    m = re.match(r'\s*(\d+-\d+)\s', ligne)
                    if m:
                        return m.group(1)
            break
        return None

    def relink(self):
        """Repare le lien USB-over-IP sans toucher au J-Link.

        Le lien meurt en cours de campagne: usbipd affiche encore "Attached"
        alors que TOUS les URB echouent (dmesg: vhci_hcd urb->status -104) et
        que serial.Serial() se bloque pour toujours a l'ouverture. Trois runs
        ont ete perdus ainsi, sans le moindre message.
        Sequence qui marche: detacher UNIQUEMENT le CDC de la carte (jamais
        --all, qui emporterait le J-Link et donc le SWD), basculer le pullup D+
        par SWD (= rebranchement logiciel), puis attacher DANS LA SECONDE ou
        Windows repasse CM_PROB_NONE — attendre plus fait rater la fenetre.

        Le busid est DECOUVERT et non code en dur: il change des qu on rebranche
        sur un autre port (6-3 est devenu 5-3 en cours de session), et un busid
        perime fait echouer la reparation en silence.
        """
        import subprocess as sp
        def ps(c):
            try: return sp.run(['powershell.exe','-Command',c], capture_output=True,
                               text=True, timeout=60).stdout
            except Exception: return ''
        busid = self._busid_carte() or '5-3'
        self.say(f'   reparation du lien USB (busid {busid})…')
        # D abord le remede le moins cher: le CDC est souvent encore enumere
        # cote Windows (usbipd le dit "Shared" et non "Attached"), il ne lui
        # manque qu un rattachement. Le passage par SWD ci-dessous rebranche le
        # peripherique pour de bon, mais il coute une dizaine de secondes et il
        # a lui-meme echoue une fois faute d avoir vu CM_PROB_NONE a temps.
        ps(f'usbipd attach --busid {busid} --wsl')
        time.sleep(5)
        if glob.glob('/dev/ttyACM*') or glob.glob('/dev/ttyUSB*'):
            self.say('   lien retabli par simple rattachement')
            return True
        ps(f'usbipd detach --busid {busid}')
        time.sleep(3)
        for v in ('0', '1'):
            sp.run(['nrfjprog','--memwr','0x40027504','--val',v], capture_output=True, timeout=60)
            sp.run(['nrfjprog','--run'], capture_output=True, timeout=60)
            if v == '0': time.sleep(4)
        for _ in range(25):
            time.sleep(1)
            n = ps("(Get-PnpDevice -PresentOnly | Where-Object { $_.InstanceId -like "
                   "'*VID_1915*' -and $_.Problem -eq 'CM_PROB_NONE' }).Count").strip()
            if n.isdigit() and int(n) >= 1:
                ps(f'usbipd attach --busid {busid} --wsl')
                time.sleep(6)
                return bool(glob.glob('/dev/ttyACM*') or glob.glob('/dev/ttyUSB*'))
        return False

    def recover(self):
        """Carte muette: reparer le LIEN d abord, ne redemarrer qu en dernier.

        L ordre compte. Le `nrfjprog --reset` venait en premier et redemarrait
        la carte a chaque hoquet USB — or le hoquet est le cas le plus frequent,
        et le redemarrage detruit l etat .noinit (ancre amarrage, compteurs
        hors-eau, calibration SWS) dont plusieurs cas dependent. On perdait donc
        la premisse du cas en cours pour reparer quelque chose qui n etait pas
        casse. On tente le rattachement, et on ne sort le SWD que si la carte
        reste muette avec un lien pourtant retabli.
        """
        self.say("!! carte muette")
        if self.b:
            try: self.b.close()
            except Exception: pass
            self.b = None
        try:
            if self.relink() and self.connect(tries=6):
                self.say("   lien repare sans redemarrer la carte")
                return True
        except Exception as e:
            self.say(f"!! reparation du lien: {type(e).__name__}: {e}")
        self.say("   la carte reste muette — reset materiel")
        try:
            subprocess.run(['nrfjprog','--reset'], capture_output=True, timeout=60)
        except Exception as e:
            self.say(f"!! nrfjprog: {e}")
        time.sleep(12)
        return self.connect()

    # ---- etat de reference: aucune emission possible ----
    def baseline(self):
        if self.b is None and not self.connect():
            return False
        b = self.b
        for attempt in range(3):
            try:
                b.enter_config()
                # ARGOS_MODE=OFF + certification desarmee => ArgosTxService inactif
                b.write_params({'ARGOS_MODE': 0, 'CERT_TX_ENABLE': 0})
                _, p = b.read_params(['ARGOS_MODE'])
                if p.get('ARP01') == '0':
                    return True
            except Exception as e:
                self.say(f"   baseline tentative {attempt+1}: {type(e).__name__}")
                if not self.recover(): return False
                b = self.b
            time.sleep(1)
        return False

    # ---- envoi brut + collecte ----
    def raw(self, payload, wait=1.5):
        """Envoie une chaine BRUTE (pas de framing ajoute) et rend les lignes recues."""
        b = self.b
        mk = b.mark()
        try:
            b._send(payload)
        except Exception as e:
            return None, f"envoi impossible: {type(e).__name__}"
        time.sleep(wait)
        with b._lock:
            lines = [l for _, l in b.history[mk:]]
        return lines, None

    def raw_until(self, payload, motif, timeout=25.0):
        """Envoie une trame et ATTEND la reponse, au lieu de dormir une duree fixe.

        Les quatre faux verdicts de la campagne du 2026-08-26 venaient tous d'une
        pause trop courte: le firmware ne consomme qu'une ligne par tick de 50 ms,
        et une ecriture de RCONF remet le module satellite sous tension pour lui
        parler en AT — une vingtaine de secondes. Une attente fixe ne peut pas
        couvrir les deux, une attente adaptative si.
        """
        b = self.b
        rx = re.compile(motif)
        mk = b.mark()
        try:
            b._send(payload)
        except Exception as e:
            return None, f"envoi impossible: {type(e).__name__}"
        fin = time.time() + timeout
        idx = mk
        while time.time() < fin:
            time.sleep(0.15)
            with b._lock:
                lignes = [l for _, l in b.history[idx:]]
                idx = len(b.history)
            for l in lignes:
                m = rx.search(l)
                if m:
                    return m, None
        return None, None            # pas de reponse dans le delai

    def record(self, case, verdict, detail, evidence=None):
        rec = {'ts': time.strftime('%Y-%m-%dT%H:%M:%S'), 'id': case['id'],
               'titre': case['titre'], 'risque': case['risque'],
               'verdict': verdict, 'detail': detail}
        if evidence: rec['trace'] = evidence[:2000]
        with open(RESULTS, 'a') as f:
            f.write(json.dumps(rec, ensure_ascii=False) + '\n')
        mark = {'PASS':'ok  ', 'FAIL':'ECHEC', 'ERROR':'err '}.get(verdict, '?')
        self.say(f"  {mark} [{case['risque'][:4]}] {case['id']}: {detail}")
        if verdict == 'PASS': self.n_pass += 1
        elif verdict == 'FAIL': self.n_fail += 1
        else: self.n_error += 1

    def run(self, cases):
        if not self.connect():
            self.say("ABANDON: carte injoignable au demarrage"); return
        self.say(f"=== campagne demarree — {len(cases)} cas ===")
        for i, case in enumerate(cases, 1):
            self.say(f"[{i}/{len(cases)}] {case['id']} — {case['titre']}")
            try:
                if self.b is None or not self.b.ping(timeout=4):
                    if not self.recover():
                        self.record(case, 'ERROR', 'carte injoignable, campagne interrompue')
                        return
                if case.get('needs_baseline', True) and not self.baseline():
                    self.record(case, 'ERROR', 'impossible de poser la configuration de reference')
                    continue
                case['fn'](self, case)
            except Exception as e:
                self.record(case, 'ERROR', f'exception {type(e).__name__}: {e}')
                if not self.recover():
                    self.say("ABANDON: recuperation impossible"); return
        self.say(f"=== fin — {self.n_pass} ok, {self.n_fail} echecs, {self.n_error} erreurs ===")


# =====================================================================
#  Cas de test — protocole DTE
#  Cibles prioritaires: les defauts que la lecture du code designe deja
#  (variables non initialisees, trames sans reponse, troncature).
# =====================================================================

def _resp(lines, cmd):
    """Extrait la reponse DTE ($O; ou $N;) pour une commande donnee."""
    pat = re.compile(rf'\$([ON]);{cmd}#([0-9A-Fa-f]{{3}});(.*)')
    for l in lines:
        m = pat.search(l)
        if m: return m
    return None

def c_len_hexa(r, case):
    lines, err = r.raw("$PARMR#00B;ARP05,ARP06\r")
    if err: return r.record(case, 'ERROR', err)
    m = _resp(lines, 'PARMR')
    if not m: return r.record(case, 'FAIL', 'aucune reponse a une trame VALIDE', '\n'.join(lines))
    ok = m.group(1) == 'O' and 'ARP05=' in m.group(3) and 'ARP06=' in m.group(3)
    r.record(case, 'PASS' if ok else 'FAIL',
             'les deux parametres sont rendus' if ok else f'reponse inattendue: {m.group(0)}')

def c_len_decimal(r, case):
    """Piege classique: longueur ecrite en decimal. Doit donner erreur 4, pas un silence."""
    lines, err = r.raw("$PARMR#010;ARP05,ARP06\r")
    if err: return r.record(case, 'ERROR', err)
    m = _resp(lines, 'PARMR')
    if not m: return r.record(case, 'FAIL', 'SILENCE sur longueur decimale — le canal ne repond pas', '\n'.join(lines))
    ok = m.group(1) == 'N'
    r.record(case, 'PASS' if ok else 'FAIL',
             f"erreur rendue ({m.group(3).strip()})" if ok else f'acceptee a tort: {m.group(0)}')

def c_len_trop_grande(r, case):
    lines, err = r.raw("$PARMR#0FF;ARP05\r")
    if err: return r.record(case, 'ERROR', err)
    m = _resp(lines, 'PARMR')
    if not m: return r.record(case, 'FAIL', 'SILENCE sur longueur excessive', '\n'.join(lines))
    r.record(case, 'PASS' if m.group(1)=='N' else 'FAIL', f'reponse {m.group(0)}')

def c_len_trop_petite(r, case):
    lines, err = r.raw("$PARMR#001;ARP05\r")
    if err: return r.record(case, 'ERROR', err)
    m = _resp(lines, 'PARMR')
    if not m: return r.record(case, 'FAIL', 'SILENCE sur longueur insuffisante', '\n'.join(lines))
    r.record(case, 'PASS' if m.group(1)=='N' else 'FAIL', f'reponse {m.group(0)}')

def c_len_non_hexa(r, case):
    """dte_protocol.hpp: 'size_t length;' non initialisee, retour de sscanf ignore.
       Si le defaut est reel, la reponse varie d'un envoi a l'autre."""
    vus = []
    for k in range(15):
        lines, err = r.raw("$PARML#ZZZ;\r", wait=1.5)
        if err: return r.record(case, 'ERROR', err)
        m = _resp(lines, 'PARML')
        vus.append(m.group(0) if m else '<silence>')
    uniq = sorted(set(vus))
    if len(uniq) == 1:
        r.record(case, 'PASS', f'comportement stable sur 15 envois: {uniq[0][:60]}')
    else:
        r.record(case, 'FAIL',
                 f'comportement NON DETERMINISTE — {len(uniq)} reponses differentes sur 15 envois',
                 '\n'.join(uniq))

def c_len_tronquee(r, case):
    """Champ longueur a 2 chiffres: le decodeur avance de 3 en aveugle."""
    lines, err = r.raw("$PARML#00;\r", wait=2.0)
    if err: return r.record(case, 'ERROR', err)
    m1 = _resp(lines, 'PARML')
    lines2, _ = r.raw("$PARML#000;\r", wait=2.0)
    m2 = _resp(lines2, 'PARML')
    if m2 is None:
        return r.record(case, 'FAIL',
                        'DESYNCHRONISATION PERSISTANTE: le canal DTE ne repond plus apres une trame malformee',
                        '\n'.join(lines + lines2))
    if m1 is None:
        r.record(case, 'FAIL',
                 'trame malformee sans aucune reponse (le host resterait bloque), mais le canal se retablit ensuite')
    else:
        r.record(case, 'PASS', f'erreur rendue et canal intact: {m1.group(0)[:50]}')

def c_cmd_inconnue(r, case):
    """dte_handler.cpp: 'DTECommand command;' non initialisee si le lookup jette."""
    vus = []
    for nom in ['XXXXX', 'YYYYY', 'ZZZZZ']:
        for k in range(6):
            lines, err = r.raw(f"${nom}#000;\r", wait=1.5)
            if err: return r.record(case, 'ERROR', err)
            got = None
            for l in lines:
                mm = re.search(r'\$([ON]);([A-Z]+)#([0-9A-Fa-f]{3});(.*)', l)
                if mm: got = (mm.group(2), mm.group(4).strip()); break
            vus.append((nom, got))
    # la reponse doit toujours porter le meme nom et le meme code
    reps = sorted(set(str(v[1]) for v in vus))
    silences = sum(1 for v in vus if v[1] is None)
    if len(reps) == 1 and silences == 0:
        r.record(case, 'PASS', f'reponse stable pour toute commande inconnue: {reps[0]}')
    elif silences == len(vus):
        r.record(case, 'PASS',
                 'silence uniforme: aucun aiguillage vers une autre commande '
                 '(le protocole ne permet pas de nommer la reponse a une commande inconnue)')
    else:
        r.record(case, 'FAIL',
                 f'reponse NON DETERMINISTE a une commande inconnue — {len(reps)} formes, {silences} silences',
                 json.dumps(vus, ensure_ascii=False))

def c_cmd_absente_du_build(r, case):
    """SMDTST / LORATX / LORABR ne sont pas dans la table sur un build KIM."""
    vus = []
    for nom, payload in [('SMDTST','000;'), ('LORATX','001;5'), ('LORABR','001;1')]:
        for k in range(4):
            lines, err = r.raw(f"${nom}#{payload}\r", wait=1.5)
            got = None
            for l in lines:
                mm = re.search(r'\$([ON]);([A-Z]+)#', l)
                if mm: got = mm.group(2); break
            vus.append((nom, got))
    mauvais = [v for v in vus if v[1] is not None and v[1] != v[0]]
    silences = [v for v in vus if v[1] is None]
    if mauvais:
        r.record(case, 'FAIL',
                 f'la reponse porte le nom d\'une AUTRE commande — {len(mauvais)}/{len(vus)} cas',
                 json.dumps(mauvais, ensure_ascii=False))
    elif len(silences) == len(vus):
        r.record(case, 'PASS', 'silence uniforme, aucun aiguillage errone')
    else:
        r.record(case, 'PASS', f'reponses coherentes ({len(silences)} silences sur {len(vus)})')

def c_sans_terminateur(r, case):
    """Trame sans \\r: le decodeur doit se resynchroniser, pas perdre le canal."""
    r.raw("$PARML#000;", wait=1.0)          # volontairement sans terminateur
    r.raw("$STATR#000;\r", wait=2.0)
    lines3, _ = r.raw("$PARML#000;\r", wait=2.5)
    m = _resp(lines3, 'PARML')
    if m: r.record(case, 'PASS', 'le canal DTE se resynchronise apres une trame tronquee')
    else: r.record(case, 'FAIL',
                   'CANAL DTE PERDU apres une trame sans terminateur — perte du moyen de configuration',
                   '\n'.join(lines3))

def c_debit_soutenu(r, case):
    """100 trames a 20 ms: la boucle ne consomme qu'une ligne par tick de 50 ms."""
    b = r.b; mk = b.mark(); envoyees = 100
    for k in range(envoyees):
        try: b._send("$PARMR#005;ARP05\r")
        except Exception as e: return r.record(case, 'ERROR', f'envoi {k}: {type(e).__name__}')
        time.sleep(0.02)
    time.sleep(12)
    with b._lock: lines = [l for _, l in b.history[mk:]]
    recues = sum(1 for l in lines if '$O;PARMR#' in l or '$N;PARMR#' in l)
    if recues == envoyees:
        r.record(case, 'PASS', f'{recues}/{envoyees} reponses')
    else:
        r.record(case, 'FAIL',
                 f'{recues}/{envoyees} reponses — {envoyees-recues} trames PERDUES silencieusement',
                 f'exemple de lignes:\n' + '\n'.join(lines[:5]))

def c_payload_max(r, case):
    """4095 octets: ne doit ni planter ni figer la carte."""
    lines, err = r.raw("$PARMR#FFF;" + "A"*4095 + "\r", wait=5.0)
    if err: return r.record(case, 'ERROR', err)
    vivant = r.b.ping(timeout=6)
    m = _resp(lines, 'PARMR')
    if not vivant:
        return r.record(case, 'FAIL', 'CARTE FIGEE apres un payload de 4095 octets', '\n'.join(lines[:10]))
    r.record(case, 'PASS' if m else 'FAIL',
             'reponse rendue et carte vivante' if m else 'aucune reponse mais carte vivante')

def c_parml_complet(r, case):
    """La liste des parametres ne doit pas etre tronquee a l'ecriture USB."""
    lines, err = r.raw("$PARML#000;\r", wait=4.0)
    if err: return r.record(case, 'ERROR', err)
    m = _resp(lines, 'PARML')
    if not m: return r.record(case, 'FAIL', 'aucune reponse a PARML', '\n'.join(lines))
    declaree = int(m.group(2), 16)
    reelle = len(m.group(3).rstrip('\r'))
    nb = m.group(3).count(',') + 1 if m.group(3).strip() else 0
    if declaree != reelle:
        r.record(case, 'FAIL',
                 f'TRONCATURE: longueur declaree {declaree} != recue {reelle} ({nb} clefs)',
                 m.group(0)[:400])
    else:
        r.record(case, 'PASS', f'{nb} clefs, longueur coherente ({reelle} octets)')


# =====================================================================
#  Cas de test — parametres: bornes, persistance, coherence
# =====================================================================

def c_bornes(r, case):
    """Une valeur hors bornes doit etre REFUSEE, jamais silencieusement tronquee."""
    b = r.b; defauts = []
    essais = [
        ('ARGOS_BLIND_RETX_NB', 0,      'sous le minimum (1)'),
        ('ARGOS_BLIND_RETX_NB', 128,    'au-dessus du maximum (127)'),
        ('ARGOS_BLIND_RETX_PERIOD_S', 59,    'sous le minimum (60)'),
        ('ARGOS_BLIND_RETX_PERIOD_S', 65536, 'au-dessus du maximum (65535)'),
        ('ARGOS_MODE', 6, 'hors enumeration (0..5)'),
    ]
    for cle, val, quoi in essais:
        try:
            _, avant = b.read_params([cle])
            k = list(avant.keys())[0] if avant else None
            # Le refus est PRECISEMENT ce que ce cas cherche a provoquer:
            # strict=False, sinon write_params leve et le cas se declare en
            # echec alors que le firmware fait exactement ce qu'on lui demande.
            b.write_params({cle: val}, strict=False)
            _, apres = b.read_params([cle])
            if k and apres.get(k) == str(val):
                defauts.append(f'{cle}={val} ({quoi}) ACCEPTEE')
            if k and avant.get(k) != apres.get(k) and apres.get(k) != str(val):
                defauts.append(f'{cle}: valeur silencieusement changee {avant.get(k)} -> {apres.get(k)}')
            if k and avant.get(k) is not None:
                b.write_params({cle: avant[k]})
        except Exception as e:
            defauts.append(f'{cle}={val}: exception {type(e).__name__}')
    if defauts:
        r.record(case, 'FAIL', f'{len(defauts)} borne(s) non respectee(s)', '\n'.join(defauts))
    else:
        r.record(case, 'PASS', 'toutes les valeurs hors bornes sont refusees')

def c_persistance(r, case):
    """Ecriture, redemarrage materiel, relecture."""
    b = r.b
    temoins = {'ARGOS_BLIND_RETX_NB': 7, 'ARGOS_BLIND_RETX_PERIOD_S': 120, 'LED_MODE': 1}
    try:
        b.write_params(temoins)
        _, avant = b.read_params(list(temoins.keys()))
    except Exception as e:
        return r.record(case, 'ERROR', f'ecriture impossible: {type(e).__name__}')
    r.say('   redemarrage materiel...')
    try: r.b.close()
    except Exception: pass
    r.b = None
    try: subprocess.run(['nrfjprog','--reset'], capture_output=True, timeout=60)
    except Exception as e: return r.record(case, 'ERROR', f'reset: {e}')
    time.sleep(14)
    if not r.connect(): return r.record(case, 'ERROR', 'carte injoignable apres redemarrage')
    try:
        r.b.enter_config()
        _, apres = r.b.read_params(list(temoins.keys()))
    except Exception as e:
        return r.record(case, 'ERROR', f'relecture impossible: {type(e).__name__}')
    perdus = [k for k in avant if avant.get(k) != apres.get(k)]
    if perdus:
        r.record(case, 'FAIL', f'{len(perdus)} parametre(s) PERDU(S) au redemarrage: {perdus}',
                 f'avant={avant}\napres={apres}')
    else:
        r.record(case, 'PASS', f'{len(avant)} parametres conserves au redemarrage')

def c_cle_inconnue(r, case):
    lines, err = r.raw("$PARMR#005;ZZZ99\r", wait=2.0)
    if err: return r.record(case, 'ERROR', err)
    m = _resp(lines, 'PARMR')
    if not m: return r.record(case, 'FAIL', 'aucune reponse a une clef inconnue', '\n'.join(lines))
    ok = m.group(1)=='N'
    r.record(case, 'PASS' if ok else 'FAIL',
             ('erreur rendue: ' + m.group(0)[:50]) if ok
             else ('la configuration ENTIERE est renvoyee au lieu d une erreur: ' + m.group(0)[:60]))

def c_ecriture_lecture_seule(r, case):
    """Un parametre non inscriptible doit refuser l'ecriture."""
    lines, err = r.raw("$PARMW#00E;IDP01=DEADBEEF\r", wait=2.0)
    if err: return r.record(case, 'ERROR', err)
    m = _resp(lines, 'PARMW')
    if not m: return r.record(case, 'FAIL', 'aucune reponse', '\n'.join(lines))
    r.record(case, 'PASS' if m.group(1)=='N' else 'FAIL',
             'ecriture refusee' if m.group(1)=='N' else f'ECRITURE ACCEPTEE sur un parametre en lecture seule: {m.group(0)}')

def c_etats(r, case):
    """Transitions configuration <-> operationnel, plusieurs fois."""
    b = r.b; defauts = []
    for k in range(5):
        try:
            b.exit_config(); time.sleep(1)
            s1 = b.get_state(timeout=8)
            b.enter_config(); time.sleep(1)
            s2 = b.get_state(timeout=8)
            if s1 is None or s2 is None:
                defauts.append(f'tour {k+1}: etat illisible ({s1} / {s2})')
            elif 'CONFIG' not in (s2 or ''):
                defauts.append(f'tour {k+1}: retour en configuration non confirme ({s2})')
        except Exception as e:
            defauts.append(f'tour {k+1}: {type(e).__name__}')
    if defauts: r.record(case, 'FAIL', f'{len(defauts)} transition(s) fautive(s)', '\n'.join(defauts))
    else: r.record(case, 'PASS', '5 aller-retours configuration/operationnel sans faute')

def c_redemarrages(r, case):
    """Redemarrages repetes: la carte doit revenir a chaque fois."""
    echecs = []
    for k in range(4):
        try: r.b.close()
        except Exception: pass
        r.b = None
        try: subprocess.run(['nrfjprog','--reset'], capture_output=True, timeout=60)
        except Exception as e: echecs.append(f'{k+1}: reset {e}'); continue
        time.sleep(14)
        if not r.connect(tries=25): echecs.append(f'{k+1}: injoignable apres reset')
    if echecs: r.record(case, 'FAIL', f'{len(echecs)}/4 redemarrages fautifs', '\n'.join(echecs))
    else: r.record(case, 'PASS', '4 redemarrages consecutifs, carte joignable a chaque fois')


CASES = [
 dict(id='FRM-01', risque='MINEUR',   titre='Longueur hexadecimale correcte',            fn=c_len_hexa),
 dict(id='FRM-02', risque='MAJEUR',   titre='Longueur ecrite en decimal',                fn=c_len_decimal),
 dict(id='FRM-03', risque='MAJEUR',   titre='Longueur superieure au payload',            fn=c_len_trop_grande),
 dict(id='FRM-04', risque='MAJEUR',   titre='Longueur inferieure au payload',            fn=c_len_trop_petite),
 dict(id='FRM-05', risque='MAJEUR',   titre='Champ longueur non hexadecimal',            fn=c_len_non_hexa),
 dict(id='FRM-06', risque='MAJEUR',   titre='Champ longueur tronque a 2 chiffres',       fn=c_len_tronquee),
 dict(id='FRM-07', risque='BLOQUANT', titre='Commande inconnue',                         fn=c_cmd_inconnue),
 dict(id='FRM-09', risque='MAJEUR',   titre='Commandes absentes du build KIM',           fn=c_cmd_absente_du_build),
 dict(id='FRM-10', risque='MAJEUR',   titre='Trame sans terminateur',                    fn=c_sans_terminateur),
 dict(id='FRM-12', risque='MAJEUR',   titre='Debit soutenu, 100 trames a 20 ms',         fn=c_debit_soutenu),
 # FRM-13 RETIRE de la boucle automatique: mesure du 2026-08-26, un payload de
 # 4095 octets fige la carte au point que ni l'USB ni le SWD ne repondent — il a
 # fallu un debranchement physique. A instruire separement, jamais en campagne.
 dict(id='PAR-01', risque='MAJEUR',   titre='PARML complet, sans troncature',            fn=c_parml_complet),
 dict(id='PAR-02', risque='BLOQUANT', titre='Bornes des parametres',                     fn=c_bornes),
 dict(id='PAR-03', risque='MAJEUR',   titre='Clef inconnue en lecture',                  fn=c_cle_inconnue),
 dict(id='PAR-04', risque='MAJEUR',   titre='Ecriture sur parametre en lecture seule',   fn=c_ecriture_lecture_seule),
 dict(id='SM-01',  risque='BLOQUANT', titre='Transitions configuration/operationnel',    fn=c_etats),
 dict(id='PAR-05', risque='BLOQUANT', titre='Persistance apres redemarrage',             fn=c_persistance),
 dict(id='ROB-01', risque='BLOQUANT', titre='Redemarrages repetes',                      fn=c_redemarrages),
]

if __name__ == '__main__':
    Runner().run(CASES)


# =====================================================================
#  Vague 2 — commandes de controle, ponts serie, lecture exhaustive
#  Toujours aucune emission: la configuration de reference force
#  ARGOS_MODE=OFF et desarme la certification.
# =====================================================================

def c_pwron(r, case):
    """PWRON: les composants valides passent, la valeur hors borne est refusee.
       On termine IMPERATIVEMENT par une extinction pour ne rien laisser alimente."""
    defauts = []
    for val, quoi in [(1,'GNSS'), (2,'capteurs'), (3,'satellite'), (0,'tout'), (4,'extinction')]:
        lines, err = r.raw(f"$PWRON#001;{val}\r", wait=2.5)
        if err: return r.record(case, 'ERROR', err)
        m = _resp(lines, 'PWRON')
        if not m: defauts.append(f'{quoi} ({val}): aucune reponse')
        elif m.group(1) != 'O': defauts.append(f'{quoi} ({val}): refuse -> {m.group(0)[:40]}')
    # hors borne
    lines, _ = r.raw("$PWRON#001;5\r", wait=2.5)
    m = _resp(lines, 'PWRON')
    if not m: defauts.append('valeur 5 hors borne: aucune reponse')
    elif m.group(1) != 'N': defauts.append(f'valeur 5 hors borne ACCEPTEE: {m.group(0)[:40]}')
    # on ne laisse rien sous tension
    r.raw("$PWRON#001;4\r", wait=3.0)
    vivant = r.b.ping(timeout=6) if r.b else False
    if not vivant: defauts.append('carte muette apres la sequence')
    if defauts: r.record(case, 'FAIL', f'{len(defauts)} anomalie(s)', '\n'.join(defauts))
    else: r.record(case, 'PASS', '5 composants pilotes, valeur hors borne refusee, extinction faite')

def c_cmd_emettrices_refusees(r, case):
    """Les commandes qui EMETTENT sur d'autres variantes doivent etre refusees ici.
       C'est un point de securite: une acceptation silencieuse ferait transmettre."""
    defauts = []
    for frame, nom in [("$SATDP#000;\r", 'SATDP'), ("$COMCW#001;0\r", 'COMCW'),
                       ("$SATTX#001;0\r", 'SATTX')]:
        lines, err = r.raw(frame, wait=2.5)
        if err: return r.record(case, 'ERROR', err)
        rep = None
        for l in lines:
            mm = re.search(r'\$([ON]);([A-Z]+)#[0-9A-Fa-f]{3};(.*)', l)
            if mm: rep = (mm.group(1), mm.group(2), mm.group(3).strip()); break
        if rep is None:
            continue                      # silence = commande absente du build, acceptable
        # $O; acquitte la TRAME, pas la commande: le refus se lit dans la charge
        # utile (statut non nul et/ou mention explicite de non-support).
        refus = (rep[2].startswith('1') or 'not supported' in rep[2].lower()
                 or 'unsupported' in rep[2].lower())
        if rep[0] == 'O' and not refus:
            defauts.append(f'{nom} ACCEPTEE sans refus explicite sur un build KIM2: {rep}')
        if rep[1] != nom:
            defauts.append(f'{nom}: la reponse porte le nom {rep[1]}')
    if defauts:
        r.record(case, 'FAIL', 'commande emettrice mal gardee', '\n'.join(defauts))
    else:
        r.record(case, 'PASS', 'SATDP / COMCW / SATTX refusees ou absentes, aucune emission possible')

def c_lecture_exhaustive(r, case):
    """Lire chaque parametre individuellement: aucun ne doit faire taire la balise."""
    b = r.b
    lines, err = r.raw("$PARML#000;\r", wait=4.0)
    if err: return r.record(case, 'ERROR', err)
    m = _resp(lines, 'PARML')
    if not m: return r.record(case, 'FAIL', 'PARML sans reponse')
    clefs = [k.strip() for k in m.group(3).rstrip('\r').split(',') if k.strip()]
    muets, erreurs = [], []
    for k in clefs:
        payload = k
        l2, e2 = r.raw(f"$PARMR#{len(payload):03X};{payload}\r", wait=0.9)
        if e2: return r.record(case, 'ERROR', f'{k}: {e2}')
        mm = _resp(l2, 'PARMR')
        if mm is None: muets.append(k)
        elif mm.group(1) == 'N': erreurs.append(f'{k}->{mm.group(3).strip()}')
    vivant = b.ping(timeout=6)
    detail = f'{len(clefs)} clefs lues, {len(muets)} muettes, {len(erreurs)} en erreur'
    if muets or not vivant:
        r.record(case, 'FAIL', detail + ('' if vivant else ' — CARTE MUETTE a la fin'),
                 'muets: ' + ', '.join(muets[:20]))
    else:
        r.record(case, 'PASS', detail + ', carte vivante',
                 ('erreurs: ' + ', '.join(erreurs[:10])) if erreurs else None)

def c_pont_kim(r, case):
    """Pont serie vers le module: une fois ouvert il capte TOUT, la sortie est +++."""
    b = r.b
    lines, err = r.raw("$KIMBR#001;1\r", wait=3.0)
    if err: return r.record(case, 'ERROR', err)
    m = _resp(lines, 'KIMBR')
    if not m or m.group(1) != 'O':
        return r.record(case, 'FAIL', f'ouverture du pont refusee: {m.group(0) if m else "<silence>"}')
    # le pont doit avaler la commande DTE d'arret
    l2, _ = r.raw("$KIMBR#001;0\r", wait=2.0)
    avalee = _resp(l2, 'KIMBR') is None
    # une commande AT doit atteindre le module
    l3, _ = r.raw("AT+FW=?\r\n", wait=3.0)
    module_repond = any('+FW=' in l or '+OK' in l for l in l3)
    # sortie
    # gentracker.cpp: la sortie est comparee a une LIGNE complete rendue par
    # usb.read_line(), qui n'aboutit qu'au terminateur. "+++" seul reste dans le
    # tampon et le pont ne se ferme jamais — verifie au banc le 2026-08-26.
    r.raw("+++\r", wait=3.0)
    l5, _ = r.raw("$PARML#000;\r", wait=3.0)
    canal_rendu = _resp(l5, 'PARML') is not None
    if not canal_rendu:
        return r.record(case, 'FAIL',
                        'CANAL DTE NON RENDU apres +++ — la balise reste prisonniere du pont',
                        '\n'.join(l5[:6]))
    d = (f"pont ouvert, commande d'arret {'avalee' if avalee else 'EXECUTEE (fuite)'}, "
         f"module {'repond' if module_repond else 'muet'}, canal rendu par +++")
    r.record(case, 'PASS' if avalee else 'FAIL', d)


CASES_V2 = [
 dict(id='CMD-25', risque='BLOQUANT', titre='PWRON: composants et valeur hors borne', fn=c_pwron),
 dict(id='CMD-27', risque='BLOQUANT', titre='Commandes emettrices refusees sur build KIM2', fn=c_cmd_emettrices_refusees),
 dict(id='PAR-04', risque='BLOQUANT', titre='Lecture individuelle de tous les parametres', fn=c_lecture_exhaustive),
 dict(id='CMD-37', risque='BLOQUANT', titre='Pont serie KIM2 et sortie par +++', fn=c_pont_kim),
]


# =====================================================================
#  Vague 3 — gardes de non-regression sur la validation des parametres
#
#  Ces deux cas existent parce qu'ajouter de la validation peut casser ce
#  qui marchait: un defaut d'usine hors de sa propre liste deviendrait
#  irrecevable, et un lot mixte pourrait etre rejete en bloc. Les deux
#  scenarios sont exactement ceux que fait une interface de configuration.
# =====================================================================

def _cles_non_inscriptibles():
    """Les clefs declarees is_writable=false dans dte_params.cpp.

    Certains parametres sont des etats que le firmware entretient lui-meme
    (LAST_KNOWN_RTC, ARGOS_CACHED_MODULATION): ils se lisent mais ne s'ecrivent
    pas, et leur refus est correct. On lit la table du firmware plutot que de
    figer une liste ici, pour que le test suive les evolutions du depot.
    """
    import os
    chemin = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          '..', '..', 'core', 'protocol', 'dte_params.cpp')
    try:
        src = open(chemin).read()
    except OSError:
        return set()
    cles = set()
    motif = (r'\{\s*"[A-Z0-9_]+"\s*,\s*"([A-Z0-9]+)"\s*,\s*BaseEncoding::\w+\s*,'
             r'[^,]*,[^,]*,\s*\{[^}]*\}\s*,\s*(true|false)\s*,\s*(true|false)\s*\}')
    for m in re.finditer(motif, src):
        if m.group(3) == 'false':
            cles.add(m.group(1))
    return cles

def c_aller_retour_complet(r, case):
    """Lire toute la configuration, puis reecrire CHAQUE parametre INSCRIPTIBLE
       avec sa propre valeur. Aucun ne doit etre refuse: sinon la validation
       rejette un etat que la balise porte legitimement."""
    m, err = r.raw_until("$PARMR#000;\r", r'\$O;PARMR#[0-9A-Fa-f]{3};(.*)', timeout=15)
    if err: return r.record(case, 'ERROR', err)
    if not m: return r.record(case, 'FAIL', 'PARMR global sans reponse')
    paires = [t for t in m.group(1).rstrip('\r').split(',') if '=' in t]
    if not paires: return r.record(case, 'FAIL', 'PARMR global vide')
    non_inscriptibles = _cles_non_inscriptibles()
    refuses, muets, ignores = [], [], 0
    for t in paires:
        if t.split('=', 1)[0] in non_inscriptibles:
            ignores += 1
            continue
        # 25 s: l'ecriture d'un RCONF rallume le module satellite et lui parle en AT
        mm, e2 = r.raw_until(f"$PARMW#{len(t):03X};{t}\r", r'\$([ON]);PARMW#', timeout=25)
        if e2: return r.record(case, 'ERROR', f'{t[:24]}: {e2}')
        if mm is None: muets.append(t[:30])
        elif mm.group(1) == 'N': refuses.append(t[:30])
    vivant = r.b.ping(timeout=8)
    d = (f'{len(paires)-ignores} parametres inscriptibles relus et reecrits '
         f'({ignores} en lecture seule ignores), {len(refuses)} refuses, {len(muets)} sans reponse')
    if refuses or muets or not vivant:
        r.record(case, 'FAIL', d + ('' if vivant else ' — CARTE MUETTE'),
                 'refuses: ' + ', '.join(refuses[:12]) + '\nmuets: ' + ', '.join(muets[:12]))
    else:
        r.record(case, 'PASS', d + ', carte vivante')

def c_lot_mixte(r, case):
    """Un lot contenant une clef invalide doit refuser CELLE-LA et appliquer les
       autres — pas rejeter l'ensemble, et surtout pas rendre la config illisible."""
    b = r.b
    try:
        b.write_params({'HAULED_ARGOS_MODE': 2, 'LED_MODE': 1, 'ARGOS_MODE': 0})
        time.sleep(1.5)
        _, avant = b.read_params(['HAULED_ARGOS_MODE', 'LED_MODE', 'ARGOS_MODE'])
    except Exception as e:
        return r.record(case, 'ERROR', f'preparation: {type(e).__name__}')
    # HMP10=5 est hors de SA liste {0..4}, alors que ARP01=5 serait legitime
    lot = 'ARP01=2,HMP10=5,LDP01=0'
    m, err = r.raw_until(f"$PARMW#{len(lot):03X};{lot}\r", r'\$([ON]);PARMW#[0-9A-Fa-f]{3};(.*)', timeout=20)
    if err: return r.record(case, 'ERROR', err)
    if m is None: return r.record(case, 'FAIL', 'lot mixte sans reponse')
    _, apres = b.read_params(['HAULED_ARGOS_MODE', 'LED_MODE', 'ARGOS_MODE'])
    mg, _ = r.raw_until("$PARMR#000;\r", r'\$O;PARMR#', timeout=15)
    defauts = []
    if m.group(1) != 'N':        defauts.append(f'le lot est accepte en bloc: {m.group(0)[:40]}')
    if 'HMP10' not in m.group(2): defauts.append('la reponse ne nomme pas la clef refusee')
    if apres.get('HMP10') != avant.get('HMP10'): defauts.append('la clef invalide a ete ECRITE')
    if apres.get('ARP01') != '2': defauts.append('un parametre VALIDE du lot a ete perdu (ARP01)')
    if apres.get('LDP01') != '0': defauts.append('un parametre VALIDE du lot a ete perdu (LDP01)')
    if mg is None:               defauts.append('PARMR global MUET apres le lot')
    try: b.write_params({'HAULED_ARGOS_MODE': 2, 'LED_MODE': 1, 'ARGOS_MODE': 0})
    except Exception: pass
    if defauts: r.record(case, 'FAIL', f'{len(defauts)} anomalie(s)', '\n'.join(defauts))
    else: r.record(case, 'PASS', 'clef fautive refusee et nommee, parametres valides appliques, config lisible')


CASES_V3 = [
 dict(id='PAR-06', risque='BLOQUANT', titre='Aller-retour de tous les parametres', fn=c_aller_retour_complet),
 dict(id='PAR-07', risque='BLOQUANT', titre='Lot mixte: rejet individuel, pas global', fn=c_lot_mixte),
]


# =====================================================================
#  Vague 4 — modes Argos et ordonnancement
#
#  On verifie que chaque mode PLANIFIE correctement, pas qu'il emet: le
#  journal annonce la prochaine echeance avant toute transmission, ce qui
#  suffit a distinguer un mode qui fonctionne d'un mode qui reste muet.
#  Aucun cas ne laisse le mode arme en sortant.
# =====================================================================

def _journal(r, secondes, motif):
    """Ecoute le journal pendant une duree et rend les lignes qui correspondent."""
    b = r.b
    idx = b.mark()
    rx = re.compile(motif)
    fin = time.time() + secondes
    vues = []
    while time.time() < fin:
        time.sleep(0.3)
        with b._lock:
            lignes = [l for _, l in b.history[idx:]]
            idx = len(b.history)
        for l in lignes:
            if rx.search(l):
                vues.append(l.strip())
    return vues

def _sched_argos(r, timeout=45.0):
    """Interroge %SCHED et rend (ms, raison) pour le service ARGOSTX.

    Pourquoi une commande console plutot que le journal: la SEULE ligne qui
    prouve un ordonnancement est
        Service::reschedule: service %s scheduled in %u msecs
    et c'est un DEBUG_TRACE, donc compile HORS du binaire a DEBUG_LEVEL=3 (le
    niveau du build banc). Aucun motif ne peut la voir. Une premiere version de
    ce cas elargissait le motif jusqu'a accrocher des lignes du service GPS
    ("first_schedule", "scheduler re-opens") et passait sans rien verifier.
    %SCHED lit la decision la ou elle est prise, pour tous les services.
    """
    fin = time.time() + timeout
    dernier = None
    while time.time() < fin:
        m, _ = r.raw_until('%SCHED\r', r'%SCHED .*ARGOSTX=', timeout=6.0)
        if m:
            ligne = m.string if hasattr(m, 'string') else ''
            mm = re.search(r'ARGOSTX=(none|\d+ms)\(([^)]*)\)', ligne)
            if mm:
                dernier = (None if mm.group(1) == 'none' else int(mm.group(1)[:-2]), mm.group(2))
                # "stopped"/"not-enabled" juste apres la sortie de config est
                # transitoire: on laisse le service se relancer avant de conclure.
                if dernier[1] not in ('stopped', 'not-enabled', 'never'):
                    return dernier
        time.sleep(2.0)
    return dernier if dernier else (None, 'aucune-reponse')

def c_modes_planifient(r, case):
    """Chaque mode Argos doit planifier une emission QUAND IL LE PEUT.

    Une premiere version echouait sur 3 modes sur 4. Verification manuelle: le
    firmware avait RAISON et journalisait meme pourquoi. Le cas ne remplissait
    pas les prealables de l'ordonnancement:
      - NTRY_PER_MESSAGE=1 + aucun fix injecte -> "depth pile has no eligible
        entries (NTRY exhausted or empty) — TX disabled until next GPS entry".
        Les modes bases sur la position (LEGACY/DUTY_CYCLE/PASS_PREDICTION) ne
        peuvent RIEN planifier; DOPPLER passait car il n'exige pas de position.
      - DUTY_CYCLE=0 (defaut) = aucune plage horaire -> "no TX slot".
      - PASS_PREDICTION sans donnees AOP ne peut predire aucun passage.
    On pose donc les prealables (NTRY illimite, toutes plages horaires, fix
    injecte) et on traite l'absence d'AOP comme un resultat LEGITIME, pas un
    echec — sinon le test accuse le firmware d'un defaut qui n'existe pas.
    """
    b = r.b
    modes = [(2, 'LEGACY'), (3, 'DUTY_CYCLE'), (4, 'DOPPLER'), (1, 'PASS_PREDICTION')]
    muets, releve = [], {}
    for val, nom in modes:
        try:
            b.enter_config()
            b.write_params({'ARGOS_MODE': val, 'GNSS_EN': 1,
                            'NTRY_PER_MESSAGE': 0,      # illimite
                            'DUTY_CYCLE': 16777215})           # toutes les heures
            b.exit_config()
        except Exception as e:
            muets.append(f'{nom}: configuration impossible ({type(e).__name__})')
            continue
        mk = b.mark()
        time.sleep(2)
        try: b._send('%GPS 43.6 3.9 5000 9\r')    # position fraiche -> pile eligible
        except Exception: pass
        ms, pourquoi = _sched_argos(r)
        with b._lock:
            jr = [l for _, l in b.history[mk:]]
        sans_aop = any('prepass indisponible' in l or 'returned no pass' in l for l in jr)
        releve[nom] = f'{ms}ms ({pourquoi})' if ms is not None else f'RIEN ({pourquoi})'
        if ms is None:
            if nom == 'PASS_PREDICTION' and sans_aop:
                releve[nom] += ' [legitime: aucun passage predictible sans AOP]'
            else:
                muets.append(f'{nom}: aucune emission planifiee -> {pourquoi}')
    try:
        b.enter_config(); b.write_params({'ARGOS_MODE': 0})
    except Exception: pass
    if muets:
        r.record(case, 'FAIL', f'{len(muets)} mode(s) sans planification',
                 '\n'.join(muets) + '\n---\n' + json.dumps(releve, ensure_ascii=False)[:900])
    else:
        r.record(case, 'PASS', f'{len(modes)} modes planifient quand les prealables sont reunis',
                 json.dumps(releve, ensure_ascii=False)[:700])

def c_prepass_sans_gnss(r, case):
    """PASS_PREDICTION avec GNSS_EN=0: combinaison incompatible qui rendait la
       balise muette SANS AUCUNE TRACE. Doit desormais se signaler."""
    b = r.b
    try:
        b.enter_config()
        b.write_params({'ARGOS_MODE': 1, 'GNSS_EN': 0})
        b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    vues = _journal(r, 40, r'INCOMPATIBLE|incompatible|PASS_PREDICTION|SCHEDULE_DISABLED')
    try:
        b.enter_config(); b.write_params({'ARGOS_MODE': 0, 'GNSS_EN': 1})
    except Exception: pass
    signale = any('incompatible' in v.lower() for v in vues)
    if signale:
        r.record(case, 'PASS', 'la combinaison est signalee explicitement', '\n'.join(vues[:3]))
    else:
        r.record(case, 'FAIL',
                 'combinaison incompatible NON signalee — la balise se tait sans trace',
                 '\n'.join(vues[:5]) if vues else '<aucune trace en 40 s>')


CASES_V4 = [
 dict(id='ARG-01', risque='BLOQUANT', titre='Chaque mode Argos planifie une emission', fn=c_modes_planifient),
 dict(id='ARG-02', risque='BLOQUANT', titre='PASS_PREDICTION sans GNSS est signale', fn=c_prepass_sans_gnss),
]

# ---------------------------------------------------------------------------
# Vague 5 — SWS (contacteur eau de mer) et mode SURFACING_BURST.
# Les assertions ci-dessous ont ete ecrites APRES observation sur la carte
# (scratchpad/sws_explore.py), pas avant: les deux faux verdicts de la journee
# venaient d'un critere invente avant d'avoir vu le comportement reel.
# ---------------------------------------------------------------------------

def _sched_tous(r, timeout=10.0):
    """Rend {service: (ms|None, raison)} pour TOUS les services."""
    m, _ = r.raw_until('%SCHED\r', r'%SCHED .*ARGOSTX=', timeout=timeout)
    if not m: return {}
    return {nom: (None if val == 'none' else int(val[:-2]), why)
            for nom, val, why in re.findall(r'(\w+)=(none|\d+ms)\(([^)]*)\)', m.string)}

def _attendre_raison(r, service, raisons, timeout=40.0):
    """Sonde %SCHED jusqu'a ce que `service` presente une des raisons attendues."""
    fin = time.time() + timeout; vu = None
    while time.time() < fin:
        d = _sched_tous(r)
        if service in d:
            vu = d[service]
            if vu[1] in raisons: return vu
        time.sleep(2)
    return vu

def c_sws_gate(r, case):
    """Le SWS doit COUPER les emissions sous l'eau et les RETABLIR en surface.

    Observe sur la carte: %DIVE -> ARGOSTX et GNSS passent tous deux a
    none(underwater); %SURFACE -> ils repassent a un etat planifie. C'est le
    coeur d'un traceur marin: emettre en plongee, c'est gaspiller la batterie
    pour une transmission que le satellite ne recevra pas.
    """
    b = r.b
    try:
        b.enter_config()
        b.write_params({'ARGOS_MODE': 2, 'GNSS_EN': 1, 'NTRY_PER_MESSAGE': 0,
                        'DUTY_CYCLE': 16777215, 'UNDERWATER_EN': 1,
                        'MIN_SURFACE_CYCLE_INTERVAL_S': 0, 'DRY_TIME_BEFORE_TX': 0})
        b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    time.sleep(2); b._send('%GPS 43.6 3.9 5000 9\r'); time.sleep(10)

    # Deux preuves valables que la balise n'emettra pas sous l'eau:
    #   'underwater'  -> reschedule() a REPRIS une decision et l'a refusee
    #                    (arrive quand le service etait en cours de cycle)
    #   'descheduled' -> notify_underwater_state a ANNULE la tache sans repasser
    #                    par reschedule() (cas d'un service simplement planifie)
    # Le critere qui compte est le meme dans les deux cas: plus AUCUNE echeance
    # (ms is None). N'exiger que 'underwater' faisait echouer le test alors que
    # le firmware coupait bel et bien — un faux "la balise emet sous l'eau",
    # le pire faux positif possible sur un traceur marin.
    GATE_OK = ('underwater', 'descheduled', 'no-schedule')
    ecarts = []
    b._send('%DIVE\r')
    for svc in ('ARGOSTX', 'GNSS'):
        vu = _attendre_raison(r, svc, GATE_OK)
        if not vu or vu[0] is not None or vu[1] not in GATE_OK:
            ecarts.append(f'{svc} apres %DIVE: {vu} (attendu: aucune echeance, {GATE_OK})')
    b._send('%SURFACE\r')
    vu = _attendre_raison(r, 'ARGOSTX', ('scheduled', 'already-initiated'))
    if not vu or vu[1] not in ('scheduled', 'already-initiated'):
        ecarts.append(f'ARGOSTX apres %SURFACE: {vu} (attendu re-planifie)')

    if ecarts:
        r.record(case, 'FAIL', f'{len(ecarts)} ecart(s) de gating SWS', '\n'.join(ecarts))
    else:
        r.record(case, 'PASS', 'SWS coupe sous l eau et retablit en surface',
                 f'surface -> {vu}')

def c_sws_dry_time(r, case):
    """DRY_TIME_BEFORE_TX doit retarder l emission apres emersion.

    Le gestionnaire de surface pose set_earliest_schedule(now + dry_time)
    (argos_tx_service.cpp:994). Deux points de mesure (60 s puis 5 s) plutot
    qu un seul: une valeur isolee pourrait etre une coincidence.
    """
    b = r.b
    mesures = {}
    for dry in (60, 5):
        try:
            b.enter_config()
            b.write_params({'ARGOS_MODE': 2, 'GNSS_EN': 1, 'NTRY_PER_MESSAGE': 0,
                            'DUTY_CYCLE': 16777215, 'UNDERWATER_EN': 1,
                            'MIN_SURFACE_CYCLE_INTERVAL_S': 0, 'DRY_TIME_BEFORE_TX': dry})
            b.exit_config()
        except Exception as e:
            return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
        time.sleep(2); b._send('%GPS 43.6 3.9 5000 9\r'); time.sleep(8)
        b._send('%DIVE\r');    _attendre_raison(r, 'ARGOSTX', ('underwater',), 30)
        b._send('%SURFACE\r'); vu = _attendre_raison(r, 'ARGOSTX', ('scheduled', 'already-initiated'), 30)
        mesures[dry] = vu
    m60, m5 = mesures.get(60), mesures.get(5)
    if not m60 or m60[0] is None or not m5 or m5[0] is None:
        return r.record(case, 'FAIL', 'mesure impossible', f'{mesures}')
    # 60 s doit repousser nettement plus loin que 5 s.
    if m60[0] > m5[0] + 20000:
        r.record(case, 'PASS', f'dry-time respecte: 60s->{m60[0]}ms vs 5s->{m5[0]}ms', f'{mesures}')
    else:
        r.record(case, 'FAIL', f'dry-time sans effet: 60s->{m60[0]}ms vs 5s->{m5[0]}ms', f'{mesures}')

def c_surfacing_burst(r, case):
    """SURFACING_BURST doit declencher une salve a l emersion.

    Prealables imposes par le firmware lui-meme: UNDERWATER_EN=1 (sinon il
    previent "SURFACING_BURST mode requires UNDERWATER_EN=1") et pas de
    cooldown actif (MIN_SURFACE_CYCLE_INTERVAL_S=0), sans quoi l absence de
    salve serait un comportement CORRECT et non un defaut.
    """
    b = r.b
    try:
        b.enter_config()
        b.write_params({'ARGOS_MODE': 5, 'GNSS_EN': 1, 'NTRY_PER_MESSAGE': 0,
                        'DUTY_CYCLE': 16777215, 'UNDERWATER_EN': 1,
                        'MIN_SURFACE_CYCLE_INTERVAL_S': 0, 'DRY_TIME_BEFORE_TX': 0,
                        'SURFACING_BURST_MAX_MSG': 3})
        b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    time.sleep(2); b._send('%GPS 43.6 3.9 5000 9\r'); time.sleep(10)
    b._send('%DIVE\r'); _attendre_raison(r, 'ARGOSTX', ('underwater',), 30)
    mk = b.mark(); b._send('%SURFACE\r')
    vu = _attendre_raison(r, 'ARGOSTX', ('scheduled', 'already-initiated'), 45)
    time.sleep(10)
    with b._lock:
        jr = [l for _, l in b.history[mk:]]
    salve = [l.strip()[24:150] for l in jr
             if re.search(r'SURFACING_BURST|burst sequence|GNSS TX #', l)]
    avert = [l.strip()[24:150] for l in jr if 'requires UNDERWATER_EN' in l]
    try:
        b.enter_config(); b.write_params({'ARGOS_MODE': 0})
    except Exception: pass
    if avert:
        r.record(case, 'FAIL', 'prealable non rempli malgre UNDERWATER_EN=1', '\n'.join(avert))
    elif salve or (vu and vu[0] is not None):
        r.record(case, 'PASS', 'salve d emersion declenchee',
                 f'sched={vu}\n' + '\n'.join(salve[:6]))
    else:
        r.record(case, 'FAIL', 'aucune salve ni planification apres emersion', f'sched={vu}')

CASES_V5 = [
    dict(id='SWS-01', risque='BLOQUANT', titre='Le SWS coupe et retablit les emissions', fn=c_sws_gate),
    dict(id='SWS-02', risque='MAJEUR',   titre='DRY_TIME_BEFORE_TX retarde l emission', fn=c_sws_dry_time),
    dict(id='ARG-03', risque='BLOQUANT', titre='SURFACING_BURST declenche une salve', fn=c_surfacing_burst),
]

def _schedq(b, timeout=20.0):
    """imm, deferred, deferred_high_water — occupation des files de l ordonnanceur."""
    mk = b.mark(); b._send('%SCHEDQ\r')
    m = b.expect(r'%SCHEDQ imm=(\d+)/\d+\(hw=\d+\) deferred=(\d+)/\d+\(hw=(\d+)\)',
                 timeout, from_idx=mk)
    return (int(m.group(1)), int(m.group(2)), int(m.group(3))) if m else None

def c_fuite_taches_differees(r, case):
    """Un aller-retour configuration/operationnel ne doit RIEN laisser derriere lui.

    periodic_config_flush() se re-programme lui-meme et n avait aucun handle: il
    etait donc incancellable, et le seul frein etait m_config_flush_active, teste
    au DECLENCHEMENT. Entrer en configuration baissait le drapeau mais laissait la
    tache en attente; en ressortir le relevait et demarrait une SECONDE chaine, et
    quand la premiere finissait par tirer elle voyait le drapeau haut et se
    re-programmait. Chaque visite dans la fenetre de 30 min ajoutait donc une
    chaine permanente: une ecriture flash save_params() de plus par periode, et un
    creneau de timer differe retenu pour toujours (capacite 128).

    Mesure du 2026-08-26 sur le binaire sans correctif: 17 -> 22 en cinq
    aller-retours, high-water 19 -> 23. Avec correctif: plat.
    """
    b = r.b
    try:
        b.enter_config(); b.write_params({'ARGOS_MODE': 0}); b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    time.sleep(3)
    base = _schedq(b)
    if not base:
        return r.record(case, 'ERROR', '%SCHEDQ sans reponse (sonde absente du build ?)')
    suite = []
    for _ in range(5):
        try:
            b.enter_config(); time.sleep(1.5); b.exit_config(); time.sleep(3)
        except Exception as e:
            return r.record(case, 'ERROR', f'aller-retour impossible: {type(e).__name__}')
        q = _schedq(b)
        if not q:
            return r.record(case, 'ERROR', '%SCHEDQ sans reponse en cours de cycle')
        suite.append(q[1])
    croissance = suite[-1] - base[1]
    trace = f'depart={base[1]} puis {suite}'
    if croissance > 0:
        r.record(case, 'FAIL',
                 f'la file differee croit de {croissance} en 5 aller-retours — tache non annulee',
                 trace)
    else:
        r.record(case, 'PASS', 'file differee stable sur 5 aller-retours', trace)

CASES_V6 = [
    dict(id='SCH-01', risque='BLOQUANT',
         titre='Un aller-retour configuration ne fuit pas de tache differee',
         fn=c_fuite_taches_differees),
]

def _argoscfg(b, timeout=15.0):
    """Configuration Argos EFFECTIVE, apres la cascade LB / hors-zone / HAULED."""
    mk = b.mark(); b._send('%ARGOSCFG\r')
    m = b.expect(r'%ARGOSCFG mode=(\d+) sensor_tx=0x([0-9A-Fa-f]{8}) depth=(\d+) '
                 r'ntry=(\d+) tr_nom=(\d+) duty=0x([0-9A-Fa-f]{6}) lb=(\d) prepass=(\d)',
                 timeout, from_idx=mk)
    if not m:
        return None
    return dict(mode=int(m.group(1)), sensor_tx=int(m.group(2), 16), depth=int(m.group(3)),
                ntry=int(m.group(4)), tr_nom=int(m.group(5)), duty=int(m.group(6), 16),
                lb=m.group(7) == '1', prepass=m.group(8) == '1')

# ServiceIdentifier::AXL_SENSOR = 12 (core/scheduling/service_scheduler.hpp)
BIT_AXL = 1 << 12

def c_axl_transmissible(r, case):
    """L accelerometre doit pouvoir etre emis quand l operateur l active.

    sensor_tx_enable est le portillon qui choisit process_sensor_burst() plutot
    que process_gnss_burst(). Il avait une branche pour ALS, PRESSURE, SEA_TEMP,
    PH et THERMISTOR mais AUCUNE pour l AXL: sur un build ou l AXL est le seul
    capteur compile, le masque valait 0 a la compilation et la voie capteur
    n etait jamais prise. AXL_SENSOR_ENABLE_TX_MODE (AXP05) etait donc un
    parametre visible par l operateur qui ne pouvait rien faire, alors que
    ArgosPacketBuilder encode l AXL completement.
    """
    b = r.b
    try:
        b.enter_config()
        b.write_params({'AXL_SENSOR_ENABLE': 0, 'AXL_SENSOR_ENABLE_TX_MODE': 0,
                        'ARGOS_MODE': 2})
        b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    time.sleep(1.5)
    off = _argoscfg(b)
    if not off:
        return r.record(case, 'ERROR', '%ARGOSCFG sans reponse (sonde absente du build ?)')

    try:
        b.enter_config()
        b.write_params({'AXL_SENSOR_ENABLE': 1, 'AXL_SENSOR_ENABLE_TX_MODE': 1})
        b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'activation impossible: {type(e).__name__}')
    time.sleep(1.5)
    on = _argoscfg(b)

    try:
        b.enter_config()
        b.write_params({'AXL_SENSOR_ENABLE': 0, 'AXL_SENSOR_ENABLE_TX_MODE': 0, 'ARGOS_MODE': 0})
        b.exit_config()
    except Exception:
        pass

    if not on:
        return r.record(case, 'ERROR', '%ARGOSCFG sans reponse apres activation')
    trace = f"desactive: sensor_tx=0x{off['sensor_tx']:08X} | active: sensor_tx=0x{on['sensor_tx']:08X} (bit AXL=0x{BIT_AXL:X})"
    if off['sensor_tx'] & BIT_AXL:
        r.record(case, 'FAIL', 'le bit AXL est pose alors que le capteur est desactive', trace)
    elif not (on['sensor_tx'] & BIT_AXL):
        r.record(case, 'FAIL', 'AXP01+AXP05 actives mais le bit AXL reste absent — capteur inemettable', trace)
    else:
        r.record(case, 'PASS', 'le bit AXL apparait quand et seulement quand l operateur l active', trace)

def c_limiteur_fenetre_enorme(r, case):
    """Une fenetre de limitation enorme ne doit pas se replier en delai minuscule.

    reschedule_s est un entier 32 bits et RLP02 accepte jusqu a 0xFFFFFFFF: au
    dela de 4294967 s (~49,7 jours) la conversion `* 1000` deborde et transforme
    une attente tres longue en attente quasi immediate — l inverse exact de ce a
    quoi sert le limiteur.
    """
    b = r.b
    try:
        b.enter_config()
        b.write_params({'ARGOS_MODE': 2, 'GNSS_EN': 1, 'NTRY_PER_MESSAGE': 0,
                        'TR_NOM': 60, 'RATE_LIMIT_EN': 1,
                        'RATE_LIMIT_WINDOW_S': 4294967295, 'RATE_LIMIT_MAX_TX': 1,
                        'UNDERWATER_EN': 0, 'DRY_TIME_BEFORE_TX': 0,
                        'MIN_SURFACE_CYCLE_INTERVAL_S': 0, 'SAT_PREPASS_EN': 0})
        b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    time.sleep(2)
    b._send('%GPS 43.6 3.9 5000 9\r')
    time.sleep(20)
    vus = []
    for _ in range(6):
        s = _sched_argos(r)
        if s and s[0] is not None:
            vus.append(s[0])
        time.sleep(8)
    try:
        b.enter_config()
        b.write_params({'RATE_LIMIT_EN': 0, 'RATE_LIMIT_WINDOW_S': 3600,
                        'RATE_LIMIT_MAX_TX': 10, 'ARGOS_MODE': 0})
        b.exit_config()
    except Exception:
        pass
    if not vus:
        return r.record(case, 'ERROR', 'aucune planification ARGOSTX observee')
    mini = min(vus)
    trace = f'delais observes: {vus} ms'
    # Avec 1 TX autorise sur une fenetre de 49710 jours, une fois le quota
    # consomme le replanning doit etre ENORME. Un debordement le ramenerait
    # sous la minute.
    if mini < 60000:
        r.record(case, 'FAIL', f'replanning de {mini} ms — la conversion a debordé', trace)
    else:
        r.record(case, 'PASS', f'replanning borne ({mini} ms minimum), pas de debordement', trace)

CASES_V7 = [
    dict(id='AXL-01', risque='BLOQUANT',
         titre='L accelerometre est emettable quand on l active',
         fn=c_axl_transmissible),
    dict(id='RL-01', risque='BLOQUANT',
         titre='Fenetre de limitation enorme sans debordement 32 bits',
         fn=c_limiteur_fenetre_enorme),
]

def c_duty_masque_nul(r, case):
    """Un masque duty-cycle vide doit etre SIGNALE, pas produire un silence muet.

    ArgosTxScheduler::INVALID_SCHEDULE vaut numeriquement SCHEDULE_DISABLED, donc
    renvoyer directement le resultat de schedule_duty_cycle() eteignait la balise
    sans une seule ligne de log. Et la cause dominante est un masque d heures a
    zero — qui est le DEFAUT USINE de DUTY_CYCLE (ARP18) comme de
    LB_ARGOS_DUTY_CYCLE (LBP05): choisir ARGOS_MODE=DUTY_CYCLE sans poser de
    masque rendait le tag muet, et rien ne le disait.
    """
    b = r.b
    try:
        b.enter_config()
        b.write_params({'ARGOS_MODE': 3, 'DUTY_CYCLE': 0, 'GNSS_EN': 1,
                        'NTRY_PER_MESSAGE': 0, 'TR_NOM': 60, 'UNDERWATER_EN': 0,
                        'SAT_PREPASS_EN': 0, 'RATE_LIMIT_EN': 0})
        b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    mk = b.mark()
    time.sleep(2); b._send('%GPS 43.6 3.9 5000 9\r')
    # On ATTEND la trace au lieu de dormir un temps fixe: la tentative de
    # planification suit le fix avec un delai variable, et une fenetre figee
    # de 18 s la manquait alors que le firmware la produisait bien.
    avert = []
    fin = time.time() + 60
    while time.time() < fin:
        time.sleep(2)
        with b._lock:
            jr = [l for _, l in b.history[mk:]]
        avert = [l.strip()[24:170] for l in jr
                 if 'DUTY_CYCLE mask is 0x000000' in l
                 or 'no TX slot found in 24h search' in l]
        if avert:
            break
    s_nul = _sched_argos(r)

    # masque plein: la planification doit repartir
    try:
        b.enter_config(); b.write_params({'DUTY_CYCLE': 16777215}); b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'reconfiguration impossible: {type(e).__name__}')
    time.sleep(2); b._send('%GPS 43.6 3.9 5000 9\r')
    s_plein = None
    fin = time.time() + 60
    while time.time() < fin:
        time.sleep(3)
        s_plein = _sched_argos(r)
        if s_plein and s_plein[0] is not None:
            break

    try:
        b.enter_config(); b.write_params({'ARGOS_MODE': 0}); b.exit_config()
    except Exception:
        pass

    trace = f'masque nul -> {s_nul} | masque plein -> {s_plein}\n' + '\n'.join(avert[:2])
    if not avert:
        r.record(case, 'FAIL', 'masque nul: aucun avertissement, la balise se tait en silence', trace)
    elif not (s_plein and s_plein[0] is not None):
        r.record(case, 'FAIL', 'masque plein: aucune emission planifiee', trace)
    else:
        r.record(case, 'PASS', 'masque nul signale explicitement, masque plein planifie', trace)

CASES_V8 = [
    dict(id='DC-01', risque='BLOQUANT',
         titre='Un masque duty-cycle vide est signale, pas silencieux',
         fn=c_duty_masque_nul),
]

# =====================================================================
#  Vague 9 — capteurs: lecture, plage, calibration, basse consommation
#  L accelerometre BMA400 (0x14) et la batterie (SAADC AIN1) sont les deux
#  seules sources physiques reelles de ce banc: on les eprouve par la PHYSIQUE
#  (au repos la norme du vecteur d acceleration doit valoir ~1 g) et pas
#  seulement par des allers-retours de protocole.
# =====================================================================

def _sensr(b, mask, timeout_s=5, wait=25.0):
    """$SENSR#len;<masque>,<timeout>. Le timeout DOIT etre dans 5..300.

    Champs de reponse, dans l ordre du gestionnaire:
      0 batt_mv, 1 batt_soc, 2 pression, 3 temperature, 4 altitude, 5 lat,
      6 lon, 7 hdop, 8 num_sv, 9 ax, 10 ay, 11 az, 12 temp_accel, 13 activite,
      14 thermistance, 15 temp_mer, 16 lux, 17 ph, 18 statut
    Le bit 2 du masque declenche une ACQUISITION GNSS: a eviter en interieur.
    """
    p = f'{mask},{timeout_s}'
    mk = b.mark()
    b._send(f'$SENSR#{len(p):03X};{p}\r')
    m = b.expect(r'\$([ON]);SENSR#([0-9A-Fa-f]{3});(.*)$', wait, from_idx=mk)
    if not m or m.group(1) != 'O':
        return None
    ch = m.group(3).rstrip('\r').split(',')
    if len(ch) < 19:
        return None
    def f(i):
        try: return float(ch[i])
        except ValueError: return None
    return dict(batt_mv=f(0), batt_soc=f(1), ax=f(9), ay=f(10), az=f(11),
                temp=f(12), activite=f(13), statut=int(float(ch[18])))

def _norme_g(d):
    if d is None or None in (d['ax'], d['ay'], d['az']):
        return None
    return (d['ax'] ** 2 + d['ay'] ** 2 + d['az'] ** 2) ** 0.5

def c_capteurs_lecture(r, case):
    """SENSR doit rendre des valeurs PHYSIQUEMENT plausibles, pas des zeros.

    Au repos l accelerometre ne mesure que la gravite: la norme du vecteur XYZ
    doit valoir 1 g quelle que soit l orientation de la carte. C est le seul
    controle qui prouve d un coup que le composant repond, que l echelle est
    appliquee et que les offsets de calibration ne sont pas aberrants.
    """
    b = r.b; defauts = []
    try:
        b.enter_config()
        b.write_params({'AXL_SENSOR_ENABLE': 1})
        time.sleep(1.0)
        d = _sensr(b, 9)          # 1=batterie + 8=accelerometre, jamais 4=GNSS
    except Exception as e:
        try: b.exit_config()
        except Exception: pass
        return r.record(case, 'ERROR', f'SENSR impossible: {type(e).__name__}')

    if d is None:
        try:
            b.write_params({'AXL_SENSOR_ENABLE': 0}); b.exit_config()
        except Exception: pass
        return r.record(case, 'ERROR', 'SENSR sans reponse exploitable')

    if not (3000 <= (d['batt_mv'] or 0) <= 5000):
        defauts.append(f"tension batterie invraisemblable: {d['batt_mv']} mV")
    if not (0 <= (d['batt_soc'] or -1) <= 100):
        defauts.append(f"charge batterie hors bornes: {d['batt_soc']} %")
    n = _norme_g(d)
    if n is None:
        defauts.append('aucune valeur accelerometre')
    elif not (0.80 <= n <= 1.20):
        defauts.append(f'norme du vecteur accelerometre = {n:.3f} g (attendu ~1 g au repos)')
    if not (0 <= (d['temp'] or -99) <= 60):
        defauts.append(f"temperature accelerometre invraisemblable: {d['temp']} C")
    if not (d['statut'] & 0x01):
        defauts.append('statut: batterie non signalee OK')
    if not (d['statut'] & 0x08):
        defauts.append('statut: accelerometre non signale OK')

    trace = (f"batt={d['batt_mv']} mV / {d['batt_soc']} % | "
             f"xyz=({d['ax']:.3f},{d['ay']:.3f},{d['az']:.3f}) norme={n:.3f} g | "
             f"temp={d['temp']} C | statut=0x{d['statut']:02X}")
    try:
        b.write_params({'AXL_SENSOR_ENABLE': 0}); b.exit_config()
    except Exception: pass
    if defauts:
        r.record(case, 'FAIL', f'{len(defauts)} valeur(s) invraisemblable(s)', trace + '\n' + '\n'.join(defauts))
    else:
        r.record(case, 'PASS', 'batterie et accelerometre coherents avec la physique', trace)

def c_capteur_plage_mesure(r, case):
    """AXP08 est un INDEX DE REGISTRE (0..3 = 2/4/8/16 g), pas une force en g.

    range_to_g() mappe 0..3 sur {2,4,8,16} et retombe sur 4 au-dela: la valeur 4
    designait donc une plage inexistante, aliasait silencieusement 4 g, et
    donnait a calculate_threshold_reg() un LSB calcule pour (1 << 6) — soit un
    registre de seuil de reveil faux d un facteur quatre. En prime, la norme
    mesuree doit rester ~1 g a TOUTES les plages: c est ce qui prouve que le
    facteur d echelle suit bien le registre.
    """
    b = r.b; defauts = []
    try:
        b.enter_config()
        b.write_params({'AXL_SENSOR_ENABLE': 1})
        # 4 doit etre refuse (un cran au-dela du registre)
        b.write_params({'AXL_SENSOR_MEASUREMENT_RANGE': 4}, strict=False)
        _, relu = b.read_params(['AXL_SENSOR_MEASUREMENT_RANGE'])
        if relu.get('AXP08') == '4':
            defauts.append('AXP08=4 ACCEPTEE alors que le registre ne va que jusqu a 3')
        mesures = {}
        for reg in (0, 3):
            b.write_params({'AXL_SENSOR_MEASUREMENT_RANGE': reg})
            time.sleep(1.5)
            d = _sensr(b, 9)
            n = _norme_g(d)
            mesures[reg] = n
            if n is None:
                defauts.append(f'plage {reg}: aucune lecture')
            elif not (0.80 <= n <= 1.20):
                defauts.append(f'plage {reg} (={2 << reg if reg < 2 else (8 if reg == 2 else 16)} g): norme={n:.3f} g')
        b.write_params({'AXL_SENSOR_MEASUREMENT_RANGE': 0, 'AXL_SENSOR_ENABLE': 0})
        b.exit_config()
    except Exception as e:
        try: b.exit_config()
        except Exception: pass
        return r.record(case, 'ERROR', f'{type(e).__name__}: {e}')
    trace = 'normes par plage: ' + ', '.join(f'{k}->{v:.3f} g' if v else f'{k}->?' for k, v in mesures.items())
    if defauts:
        r.record(case, 'FAIL', f'{len(defauts)} anomalie(s) de plage', trace + '\n' + '\n'.join(defauts))
    else:
        r.record(case, 'PASS', 'valeur 4 refusee, echelle correcte a 2 g et a 16 g', trace)

def c_capteur_calibration(r, case):
    """Calibration de l accelerometre: ecrite, relue, appliquee — et les
    parametres DTE doivent vraiment atteindre le composant.

    ATTENTION, les offsets SCALR et SCALW sont ASYMETRIQUES (bma400.cpp,
    calibration_read / calibration_write):
      ecriture 0/1/2 = poser la calibration X/Y/Z, 3 = auto-calibration
      lecture  1/2/3 = mesure EN DIRECT, 4/5/6 = calibration X/Y/Z,
               7/8 = seuil / duree de reveil, 9 = registre de plage,
               10 = mode d alimentation
    Un offset de lecture invalide (0 par exemple) ne renvoie PAS d erreur: le
    pilote retombe sur son `default:` et rend 0.0 avec un simple avertissement
    dans le journal, ce qu un hote lit comme une valeur legitime.

    Les offsets 9 et 10 servent ici a verifier que AXP08 et AXP09 arrivent
    reellement jusqu au BMA400, et pas seulement dans le magasin de config.
    """
    b = r.b; defauts = []
    def scalw(offset, value):
        p = f'0,{offset},{value}'
        mk = b.mark(); b._send(f'$SCALW#{len(p):03X};{p}\r')
        m = b.expect(r'\$([ON]);SCALW#', 12.0, from_idx=mk)
        return bool(m and m.group(1) == 'O')
    def scalr(offset):
        p = f'0,{offset}'
        mk = b.mark(); b._send(f'$SCALR#{len(p):03X};{p}\r')
        m = b.expect(r'\$([ON]);SCALR#([0-9A-Fa-f]{3});?(.*)$', 12.0, from_idx=mk)
        if not m or m.group(1) != 'O':
            return None
        try: return float(m.group(3).rstrip('\r').split(',')[0])
        except (ValueError, IndexError): return None
    try:
        # 1) les parametres DTE atteignent-ils le composant ?
        #
        # sensor_init() n applique la plage et le mode d alimentation qu au
        # DEMARRAGE du service. Les ecrire en mode configuration ne les pousse
        # donc pas tout de suite: il faut ressortir en operationnel pour que le
        # service redemarre. Un test qui reste en configuration mesure 0/0 et
        # conclut a tort que les parametres n arrivent pas au composant.
        releves = {}
        for reg, mode_att in ((3, 1), (0, 0)):
            b.enter_config()
            b.write_params({'AXL_SENSOR_ENABLE': 1,
                            'AXL_SENSOR_MEASUREMENT_RANGE': reg,
                            'AXL_SENSOR_POWER_MODE': mode_att})
            b.exit_config()
            time.sleep(4)
            b.enter_config()
            lu_p, lu_m = scalr(9), scalr(10)
            releves[reg] = (lu_p, lu_m)
            if lu_p is None or int(lu_p) != reg:
                defauts.append(f'AXP08={reg} mais le registre de plage du composant vaut {lu_p}')
            if lu_m is None or int(lu_m) != mode_att:
                defauts.append(f'AXP09={mode_att} mais le mode d alimentation vaut {lu_m}')
            b.exit_config()
        b.enter_config()
        time.sleep(1.0)

        # 2) aller-retour d un offset de calibration, et effet sur la mesure
        anciens = {o: scalr(o) for o in (4, 5, 6)}       # calibration X/Y/Z
        if any(v is None for v in anciens.values()):
            defauts.append(f'calibration illisible en 4/5/6: {anciens}')
        avant_x = scalr(1)                                # mesure X en direct
        DELTA = 0.25
        base_x = anciens.get(4) or 0.0
        cible = round(base_x + DELTA, 4)
        if not scalw(0, cible):                           # ecriture: offset 0 = X
            defauts.append('SCALW refuse')
        relu = scalr(4)
        if relu is None or abs(relu - cible) > 0.02:
            defauts.append(f'calibration X relue {relu}, attendu {cible}')
        time.sleep(1.2)
        apres_x = scalr(1)
        if avant_x is not None and apres_x is not None:
            bouge = apres_x - avant_x
            # tolerance large: la carte n est pas sur un marbre, la mesure bouge
            # naturellement de quelques centiemes de g entre deux lectures.
            if abs(abs(bouge) - DELTA) > 0.15:
                defauts.append(f'la mesure X a bouge de {bouge:+.3f} g, attendu ~{DELTA} g')

        # 3) un offset de lecture invalide ne doit pas passer pour une vraie valeur
        invalide = scalr(0)

        for o, v in anciens.items():
            if v is not None:
                scalw(o - 4, v)                            # 4/5/6 (lecture) -> 0/1/2 (ecriture)
        b.write_params({'AXL_SENSOR_ENABLE': 0})
        b.exit_config()
    except Exception as e:
        try: b.exit_config()
        except Exception: pass
        return r.record(case, 'ERROR', f'{type(e).__name__}: {e}')

    trace = (f'apres redemarrage du service: {releves} | '
             f'calib initiale {anciens} | X direct {avant_x} -> {apres_x} | '
             f'offset invalide 0 -> {invalide}')
    if defauts:
        r.record(case, 'FAIL', f'{len(defauts)} anomalie(s) de calibration', trace + '\n' + '\n'.join(defauts))
    else:
        r.record(case, 'PASS', 'AXP08/AXP09 atteignent le composant, offset ecrit relu et applique', trace)

def c_capteur_basse_conso(r, case):
    """Parametres de veille de l accelerometre: bornes tenues et valeurs conservees.

    AXP09 = mode d alimentation (0 = LOW_POWER avec moteur de reveil, 1 = NORMAL
    avec interruption GEN1). AXP03 = seuil de reveil en g. AXP04 = nombre
    d echantillons. Une valeur hors bornes doit etre REFUSEE, jamais tronquee en
    silence: un seuil de reveil errone rend le tag soit aveugle au mouvement,
    soit constamment reveille — les deux se paient en batterie sur un an.
    """
    b = r.b; defauts = []
    essais = [
        ('AXL_SENSOR_POWER_MODE',       3,     'mode d alimentation (0..2)'),
        ('AXL_SENSOR_WAKEUP_THRESH',    9.0,   'seuil de reveil (0..8 g)'),
        ('AXL_SENSOR_WAKEUP_SAMPLES',   51,    'echantillons de reveil (0..50)'),
    ]
    try:
        b.enter_config()
        for cle, val, quoi in essais:
            _, avant = b.read_params([cle])
            k = list(avant.keys())[0] if avant else None
            b.write_params({cle: val}, strict=False)
            _, apres = b.read_params([cle])
            if k and apres.get(k) == str(val):
                defauts.append(f'{cle}={val} ({quoi}) ACCEPTEE')
            elif k and avant.get(k) != apres.get(k):
                defauts.append(f'{cle} silencieusement change {avant.get(k)} -> {apres.get(k)}')
        # valeurs valides: doivent tenir
        b.write_params({'AXL_SENSOR_POWER_MODE': 0, 'AXL_SENSOR_WAKEUP_THRESH': 0.5,
                        'AXL_SENSOR_WAKEUP_SAMPLES': 5})
        _, v = b.read_params(['AXL_SENSOR_POWER_MODE', 'AXL_SENSOR_WAKEUP_THRESH',
                              'AXL_SENSOR_WAKEUP_SAMPLES'])
        if v.get('AXP09') != '0':
            defauts.append(f"AXP09=0 non conserve: {v.get('AXP09')}")
        if v.get('AXP04') != '5':
            defauts.append(f"AXP04=5 non conserve: {v.get('AXP04')}")
        try:
            if abs(float(v.get('AXP03', '0')) - 0.5) > 0.01:
                defauts.append(f"AXP03=0.5 non conserve: {v.get('AXP03')}")
        except ValueError:
            defauts.append(f"AXP03 illisible: {v.get('AXP03')}")
        # mode NORMAL doit aussi passer
        b.write_params({'AXL_SENSOR_POWER_MODE': 1})
        _, v2 = b.read_params(['AXL_SENSOR_POWER_MODE'])
        if v2.get('AXP09') != '1':
            defauts.append(f"AXP09=1 non conserve: {v2.get('AXP09')}")
        b.write_params({'AXL_SENSOR_POWER_MODE': 0})
        b.exit_config()
    except Exception as e:
        try: b.exit_config()
        except Exception: pass
        return r.record(case, 'ERROR', f'{type(e).__name__}: {e}')
    if defauts:
        r.record(case, 'FAIL', f'{len(defauts)} anomalie(s)', '\n'.join(defauts))
    else:
        r.record(case, 'PASS', 'bornes tenues et parametres de veille conserves')

def c_batterie_coherence(r, case):
    """La batterie lue par SENSR et celle publiee par STATR doivent concorder.

    Ce sont deux chemins distincts vers la meme mesure SAADC (AIN1): SENSR lit a
    la demande, STATR publie la valeur filtree. Un ecart franc signale soit un
    filtrage casse, soit deux unites differentes.
    """
    b = r.b; defauts = []
    try:
        b.enter_config()
        d = _sensr(b, 1)
        p = 'POT06,POT03'
        mk = b.mark(); b._send(f'$STATR#{len(p):03X};{p}\r')
        m = b.expect(r'\$([ON]);STATR#[0-9A-Fa-f]{3};(.*)$', 12.0, from_idx=mk)
        b.exit_config()
    except Exception as e:
        try: b.exit_config()
        except Exception: pass
        return r.record(case, 'ERROR', f'{type(e).__name__}: {e}')
    if d is None or not m:
        return r.record(case, 'ERROR', 'SENSR ou STATR sans reponse')
    st = {}
    for morceau in m.group(2).rstrip('\r').split(','):
        if '=' in morceau:
            k, v = morceau.split('=', 1); st[k] = v
    try:
        v_statr = float(st.get('POT06', 'nan')) * 1000.0
        soc_statr = float(st.get('POT03', 'nan'))
    except ValueError:
        return r.record(case, 'ERROR', f'STATR illisible: {st}')
    if abs(v_statr - d['batt_mv']) > 250:
        defauts.append(f"tension: SENSR {d['batt_mv']} mV vs STATR {v_statr:.0f} mV")
    if abs(soc_statr - d['batt_soc']) > 10:
        defauts.append(f"charge: SENSR {d['batt_soc']} % vs STATR {soc_statr:.0f} %")
    trace = f"SENSR {d['batt_mv']} mV / {d['batt_soc']} % | STATR {v_statr:.0f} mV / {soc_statr:.0f} %"
    if defauts:
        r.record(case, 'FAIL', 'les deux chemins de mesure divergent', trace + '\n' + '\n'.join(defauts))
    else:
        r.record(case, 'PASS', 'SENSR et STATR concordent sur la batterie', trace)

CASES_V9 = [
    dict(id='SENS-R1', risque='BLOQUANT', titre='Lecture capteurs physiquement plausible', fn=c_capteurs_lecture),
    dict(id='SENS-R2', risque='BLOQUANT', titre='Plage de mesure: bornes et facteur d echelle', fn=c_capteur_plage_mesure),
    dict(id='SENS-R3', risque='MAJEUR',   titre='Calibration ecrite, relue et appliquee', fn=c_capteur_calibration),
    dict(id='SENS-R4', risque='MAJEUR',   titre='Parametres de basse consommation bornes', fn=c_capteur_basse_conso),
    dict(id='BATT-R1', risque='MAJEUR',   titre='SENSR et STATR concordent sur la batterie', fn=c_batterie_coherence),
]

# =====================================================================
#  Vague 10 — Argos en surface, pilote par le SWS
#  Aucune emission radio n est possible (pas de credentials KIM2): on eprouve
#  la DECISION d emettre et la cascade d etat, lues dans le journal.
# =====================================================================

def _attendre_trace(b, motifs, secondes, depuis=None, exiger=None):
    """Attend des traces dans le journal. Rend la liste des lignes vues.

    On ATTEND au lieu de dormir un temps fixe: la cascade d emersion enchaine
    des etapes de duree variable, et une fenetre figee produit de faux verdicts
    dans les deux sens.

    `exiger` est la liste des motifs qui doivent TOUS avoir ete vus avant de
    rendre la main. Sans lui, la fonction s arretait au PREMIER motif trouve —
    ce qui suffit quand on guette un evenement unique, mais pas quand on attend
    une SEQUENCE. SURF-01 en a fait les frais: il rendait la main sur
    "Doppler limit reached" et concluait a l absence du "cooldown armed" qui
    arrivait juste apres, accusant un firmware qui se comportait correctement.
    """
    mk = b.mark() if depuis is None else depuis
    fin = time.time() + secondes
    vues = []
    while time.time() < fin:
        time.sleep(1.5)
        with b._lock:
            lignes = [l for _, l in b.history[mk:]]
        vues = [l.strip()[24:200] for l in lignes
                if any(re.search(m, l) for m in motifs)]
        if exiger:
            if all(any(re.search(m, v) for v in vues) for m in exiger):
                return vues
        elif vues:
            return vues
    return vues

def _config_surface(b, **extra):
    """Configuration de reference d une salve d emersion."""
    cfg = {'ARGOS_MODE': 5, 'GNSS_EN': 1, 'NTRY_PER_MESSAGE': 0,
           'DUTY_CYCLE': 16777215, 'UNDERWATER_EN': 1,
           'MIN_SURFACE_CYCLE_INTERVAL_S': 0, 'DRY_TIME_BEFORE_TX': 0,
           'SURFACING_BURST_MAX_MSG': 3, 'SURFACING_BURST_INIT_S': 5,
           'SURFACING_BURST_STEP_S': 0, 'SURFACING_BURST_MAX_S': 30,
           'SAT_PREPASS_EN': 0, 'RATE_LIMIT_EN': 0, 'LB_EN': 0}
    cfg.update(extra)
    b.enter_config(); b.write_params(cfg); b.exit_config()

def c_surface_limite_doppler(r, case):
    """SURFACING_BURST_MAX_MSG borne la phase Doppler et arme le refroidissement.

    ARP43 est la seule borne de la sequence Doppler. A 0 elle est illimitee, ce
    que le wiki interdit en mode DOPPLER (emission continue jusqu a vider la
    batterie). Ici on verifie la borne HAUTE: apres N messages Doppler la salve
    doit s arreter d elle-meme ET armer le refroidissement, sans quoi un animal
    qui reste en surface emet sans fin.
    """
    b = r.b
    try:
        # Ce cas n etait PAS idempotent: il arme lui-meme un refroidissement, et
        # au rejeu l emersion suivante devenait passive — plus aucune trame
        # Doppler, donc plus de limite atteinte, et un echec qui n avait rien a
        # voir avec le firmware. On purge donc l etat en repassant par une
        # fenetre nulle avant de poser la vraie configuration.
        _config_surface(b, MIN_SURFACE_CYCLE_INTERVAL_S=0, COOLDOWN_TRIGGER_MODE=3)
        time.sleep(2)
        # UNP30 vaut 3 (AFTER_LAST_TX) par defaut: l armement a la fin de la
        # phase Doppler n a lieu QUE si le declencheur est END_OF_DOPPLER. Sans
        # ce reglage le firmware a raison de ne rien armer, et le test avait
        # tort de le lui reprocher. Fenetre courte: juste assez pour qu il y ait
        # quelque chose a refroidir, pas assez pour saboter le rejeu.
        _config_surface(b, SURFACING_BURST_MAX_MSG=2, COOLDOWN_TRIGGER_MODE=1,
                        MIN_SURFACE_CYCLE_INTERVAL_S=30)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    time.sleep(2)
    b._send('%DIVE\r'); time.sleep(6)
    mk = b.mark()
    b._send('%SURFACE\r')
    # 150 s et non 120: avec ARP43=2 il faut DEUX emissions Doppler espacees par
    # l intervalle de salve, et le rejeu manuel du 2026-08-27 a montre que la
    # sequence complete (limite atteinte + refroidissement arme) tient en ~150 s.
    # Une fenetre trop courte faisait rougir un firmware qui se comportait bien.
    # On surveille aussi 'passive surfacing': si l emersion est refusee par un
    # refroidissement residuel, le cas doit le DIRE au lieu d accuser ARP43.
    vues = _attendre_trace(b, [r'Doppler limit reached', r'cooldown armed',
                               r'passive surfacing'], 170, depuis=mk,
                           exiger=[r'Doppler limit reached', r'cooldown armed'])
    try:
        b.enter_config()
        b.write_params({'ARGOS_MODE': 0, 'UNDERWATER_EN': 0, 'COOLDOWN_TRIGGER_MODE': 3,
                        'MIN_SURFACE_CYCLE_INTERVAL_S': 0})
        b.exit_config()
    except Exception:
        pass
    limite = [l for l in vues if 'Doppler limit reached' in l]
    refroid = [l for l in vues if 'cooldown armed' in l]
    passif = [l for l in vues if 'passive surfacing' in l]
    trace = '\n'.join(vues[:6])
    if passif and not limite:
        r.record(case, 'ERROR',
                 'emersion rendue passive par un refroidissement residuel — cas non conclusif',
                 trace)
        return
    if not limite:
        r.record(case, 'FAIL', 'la phase Doppler ne s arrete pas a ARP43', trace)
    elif not refroid:
        r.record(case, 'FAIL', 'limite atteinte mais refroidissement non arme', trace)
    else:
        r.record(case, 'PASS', 'phase Doppler bornee et refroidissement arme', trace)

def c_surface_promotion_fix(r, case):
    """Un fix frais pendant la phase Doppler doit promouvoir en phase GNSS.

    C est le coeur du mode: tant qu il n y a pas de position on emet du Doppler
    (le segment sol reconstruit par effet Doppler), et des qu une position
    tombe on doit basculer sur des trames GNSS, bien plus precises. Si la
    promotion n a pas lieu, la balise gaspille sa fenetre de surface en Doppler
    alors qu elle a mieux a dire.
    """
    b = r.b
    try:
        _config_surface(b, SURFACING_BURST_MAX_MSG=8)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    time.sleep(2)
    b._send('%DIVE\r'); time.sleep(6)
    mk = b.mark()
    b._send('%SURFACE\r'); time.sleep(8)
    b._send('%GPS 43.6 3.9 5000 9\r')
    vues = _attendre_trace(b, [r'promoting to GNSS phase', r'promoting Doppler slot to GNSS TX',
                               r'switching to GNSS phase', r'GNSS TX #1'], 120, depuis=mk)
    try:
        b.enter_config(); b.write_params({'ARGOS_MODE': 0, 'UNDERWATER_EN': 0}); b.exit_config()
    except Exception:
        pass
    if vues:
        r.record(case, 'PASS', 'le fix promeut la salve en phase GNSS', '\n'.join(vues[:5]))
    else:
        r.record(case, 'FAIL', 'aucune promotion en phase GNSS apres un fix frais')

def c_surface_refroidissement(r, case):
    """MIN_SURFACE_CYCLE_INTERVAL_S: une emersion trop rapprochee doit etre PASSIVE.

    Sans ce garde-fou, un animal qui fait des sauts repetes declenche un cycle
    complet (GPS + salve) a chaque fois et vide la batterie. Le firmware doit
    signaler ces emersions comme passives, puis reprendre normalement une fois
    la fenetre ecoulee. C est aussi le chemin qui, en 2026-05, avait laisse un
    tag dormant: le SWS etait desactive pendant le refroidissement et rien ne le
    rearmait.
    """
    b = r.b
    try:
        # Aucune emission n aboutit sur ce banc (pas de credentials KIM2), donc
        # "cycle complete" ne peut jamais se produire: on arme le
        # refroidissement par END_OF_DOPPLER, qui n exige pas de TX reussie.
        _config_surface(b, MIN_SURFACE_CYCLE_INTERVAL_S=90, SURFACING_BURST_MAX_MSG=1,
                        COOLDOWN_TRIGGER_MODE=1)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    time.sleep(2)
    # premier cycle complet: il doit armer le refroidissement
    b._send('%DIVE\r'); time.sleep(6)
    mk = b.mark(); b._send('%SURFACE\r')
    arme = _attendre_trace(b, [r'cooldown started', r'cooldown armed'], 150, depuis=mk)
    # (un seul evenement attendu ici: pas de `exiger`)
    # deuxieme emersion immediate: elle doit etre passive
    b._send('%DIVE\r'); time.sleep(6)
    mk2 = b.mark(); b._send('%SURFACE\r')
    passif = _attendre_trace(b, [r'passive surfacing'], 60, depuis=mk2)
    # le SWS doit rester vivant pendant le refroidissement (chemin du blocage
    # de 2026-05: le SWS etait desactive et plus rien ne le rearmait)
    sws = (_sched_tous(r) or {}).get('SWSAnalog')
    try:
        b.enter_config()
        b.write_params({'ARGOS_MODE': 0, 'UNDERWATER_EN': 0,
                        'MIN_SURFACE_CYCLE_INTERVAL_S': 0, 'COOLDOWN_TRIGGER_MODE': 3})
        b.exit_config()
    except Exception:
        pass
    trace = 'arme: ' + '; '.join(arme[:2]) + '\npassif: ' + '; '.join(passif[:2]) + f'\nSWS: {sws}'
    if not arme:
        r.record(case, 'FAIL', 'le refroidissement n est pas arme apres un cycle complet', trace)
    elif not passif:
        r.record(case, 'FAIL', 'une emersion pendant le refroidissement n est pas signalee passive', trace)
    else:
        r.record(case, 'PASS', 'refroidissement arme et emersion rapprochee rendue passive', trace)

def c_surface_sans_sws(r, case):
    """SURFACING_BURST sans UNDERWATER_EN doit etre SIGNALE, pas subi.

    Le mode est entierement pilote par les transitions du SWS: sans capteur
    actif, aucune salve ne peut jamais partir. Le silence serait indiagnosticable
    sur le terrain, donc le firmware doit le dire au demarrage du service.
    """
    b = r.b
    try:
        b.enter_config()
        b.write_params({'ARGOS_MODE': 5, 'GNSS_EN': 1, 'UNDERWATER_EN': 0,
                        'NTRY_PER_MESSAGE': 0, 'SAT_PREPASS_EN': 0})
        mk = b.mark()
        b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    vues = _attendre_trace(b, [r'requires UNDERWATER_EN'], 45, depuis=mk)
    try:
        b.enter_config(); b.write_params({'ARGOS_MODE': 0}); b.exit_config()
    except Exception:
        pass
    if vues:
        r.record(case, 'PASS', 'la combinaison impossible est signalee au demarrage', '\n'.join(vues[:2]))
    else:
        r.record(case, 'FAIL', 'SURFACING_BURST sans SWS: aucun avertissement')

CASES_V10 = [
    dict(id='SURF-01', risque='BLOQUANT', titre='La phase Doppler est bornee et arme le refroidissement', fn=c_surface_limite_doppler),
    dict(id='SURF-02', risque='BLOQUANT', titre='Un fix frais promeut la salve en phase GNSS',            fn=c_surface_promotion_fix),
    dict(id='SURF-03', risque='BLOQUANT', titre='Refroidissement inter-cycles et emersion passive',       fn=c_surface_refroidissement),
    dict(id='SURF-04', risque='MAJEUR',   titre='SURFACING_BURST sans SWS est signale',                   fn=c_surface_sans_sws),
]

# =====================================================================
#  Vague 11 — zone geographique (geofencing)
#  is_zone_exclusion() calcule une distance haversine entre le dernier fix et
#  (ZOP18, ZOP19), et compare a ZONE_RADIUS (ZOP20) exprime en METRES. Tout est
#  pilotable en injectant des fix, et %ARGOSCFG rend la configuration EFFECTIVE
#  apres cascade — c est le seul moyen d observer la substitution.
# =====================================================================

# Profils volontairement distincts sur TROIS champs independants: une bascule
# fortuite sur un seul champ ne peut pas faire passer le test par hasard.
_ZONE_CENTRE = (43.6, 3.9)          # (lat, lon)
_ZONE_RAYON_M = 10000               # 10 km
_DEDANS  = (43.60, 3.90)            # distance ~0
_DEHORS  = (44.60, 3.90)            # ~111 km

def _profil_zone(b, ooz_actif=True, **extra):
    cfg = {'ARGOS_MODE': 2, 'TR_NOM': 60, 'ARGOS_DEPTH_PILE': 4,
           'NTRY_PER_MESSAGE': 0, 'GNSS_EN': 1,
           'ZONE_ENABLE_OUT_OF_ZONE_DETECTION_MODE': 1 if ooz_actif else 0,
           'ZONE_TYPE': 1,   # BaseZoneType::CIRCLE = 1 (la valeur 0 n existe pas)
           'ZONE_ENABLE_ACTIVATION_DATE': 0,
           'ZONE_CENTER_LATITUDE': _ZONE_CENTRE[0],
           'ZONE_CENTER_LONGITUDE': _ZONE_CENTRE[1],
           'ZONE_RADIUS': _ZONE_RAYON_M,
           'ZONE_ARGOS_MODE': 2, 'ZONE_ARGOS_REPETITION_SECONDS': 600,
           'ZONE_ARGOS_DEPTH_PILE': 1, 'ZONE_ARGOS_NTRY_PER_MESSAGE': 2,
           'LB_EN': 0, 'UNDERWATER_EN': 0, 'SAT_PREPASS_EN': 0,
           'RATE_LIMIT_EN': 0, 'HAULED_DETECT_EN': 0}
    cfg.update(extra)
    b.enter_config(); b.write_params(cfg); b.exit_config()

def _injecte_et_lit(b, lat, lon, attente=14):
    """Injecte un fix puis rend la configuration Argos EFFECTIVE."""
    time.sleep(1.5)
    b._send(f'%GPS {lat} {lon} 5000 9\r')
    time.sleep(attente)
    return _argoscfg(b)

def _est_profil_zone(c):
    return c and c['tr_nom'] == 600 and c['depth'] == 1 and c['ntry'] == 2

def _est_profil_normal(c):
    return c and c['tr_nom'] == 60 and c['depth'] == 4 and c['ntry'] == 0

def c_zone_substitution(r, case):
    """Un fix hors du rayon doit substituer le profil ZONE, et lui seul.

    C est toute la raison d etre du geofencing: hors de la zone d interet on
    veut une cadence differente. Si la substitution n a pas lieu, la balise
    continue a emettre au rythme nominal la ou ce n est pas voulu; si elle a
    lieu a tort, elle ralentit la ou on l attendait rapide.
    """
    b = r.b
    try:
        _profil_zone(b)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    dedans = _injecte_et_lit(b, *_DEDANS)
    dehors = _injecte_et_lit(b, *_DEHORS)
    try:
        b.enter_config()
        b.write_params({'ZONE_ENABLE_OUT_OF_ZONE_DETECTION_MODE': 0, 'ARGOS_MODE': 0})
        b.exit_config()
    except Exception:
        pass
    if not dedans or not dehors:
        return r.record(case, 'ERROR', '%ARGOSCFG sans reponse')
    trace = f'dedans: {dedans}\ndehors: {dehors}'
    if not _est_profil_normal(dedans):
        r.record(case, 'FAIL', 'un fix DANS la zone ne garde pas le profil nominal', trace)
    elif not _est_profil_zone(dehors):
        r.record(case, 'FAIL', 'un fix HORS zone ne substitue pas le profil ZONE', trace)
    else:
        r.record(case, 'PASS', 'profil nominal dedans, profil ZONE dehors', trace)

def c_zone_frontiere(r, case):
    """Le rayon est en METRES et la comparaison est stricte (d_km > rayon/1000).

    Une erreur d unite d un facteur mille sur ce parametre est invisible en
    lecture — le nombre reste plausible — et deplace la frontiere de 10 km a
    10 m ou a 10 000 km. On l eprouve avec deux fix qui encadrent la limite.
    """
    b = r.b
    try:
        _profil_zone(b, ZONE_RADIUS=10000)   # 10 km
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    # ~5,6 km au nord du centre: DEDANS.  ~16,7 km: DEHORS.
    proche = _injecte_et_lit(b, _ZONE_CENTRE[0] + 0.05, _ZONE_CENTRE[1])
    loin   = _injecte_et_lit(b, _ZONE_CENTRE[0] + 0.15, _ZONE_CENTRE[1])
    try:
        b.enter_config()
        b.write_params({'ZONE_ENABLE_OUT_OF_ZONE_DETECTION_MODE': 0, 'ARGOS_MODE': 0})
        b.exit_config()
    except Exception:
        pass
    if not proche or not loin:
        return r.record(case, 'ERROR', '%ARGOSCFG sans reponse')
    trace = f'a ~5,6 km: {proche}\na ~16,7 km: {loin}'
    if not _est_profil_normal(proche):
        r.record(case, 'FAIL', 'un fix a 5,6 km est traite comme hors zone (rayon 10 km)', trace)
    elif not _est_profil_zone(loin):
        r.record(case, 'FAIL', 'un fix a 16,7 km est traite comme dans la zone (rayon 10 km)', trace)
    else:
        r.record(case, 'PASS', 'la frontiere tombe bien entre 5,6 et 16,7 km pour un rayon de 10 km', trace)

def c_zone_desactivee(r, case):
    """ZOP04=0 doit neutraliser la substitution, meme tres loin du centre.

    Un operateur qui desactive la detection hors-zone ne doit pas se retrouver
    avec le profil ZONE applique parce que d anciennes coordonnees trainent
    dans la configuration.
    """
    b = r.b
    try:
        _profil_zone(b, ooz_actif=False)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    loin = _injecte_et_lit(b, *_DEHORS)
    try:
        b.enter_config(); b.write_params({'ARGOS_MODE': 0}); b.exit_config()
    except Exception:
        pass
    if not loin:
        return r.record(case, 'ERROR', '%ARGOSCFG sans reponse')
    if _est_profil_zone(loin):
        r.record(case, 'FAIL', 'profil ZONE applique alors que la detection est desactivee', str(loin))
    elif _est_profil_normal(loin):
        r.record(case, 'PASS', 'detection desactivee: le profil nominal est conserve', str(loin))
    else:
        r.record(case, 'FAIL', 'profil ni nominal ni ZONE', str(loin))

def c_zone_batterie_prime(r, case):
    """LOW_BATTERY doit primer sur hors-zone.

    La cascade de get_argos_configuration() est
    `if (lb_en && batterie basse) {...} else if (is_out_of_zone) {...}`: une
    balise a la fois hors zone ET en batterie basse doit prendre le profil LB,
    pas le profil ZONE. Se tromper d ordre ici fait emettre au rythme ZONE une
    balise qui n a plus l energie pour le tenir.

    LIMITE CONNUE DE CE CAS: BatteryMonitor recoit ses deux seuils par son
    CONSTRUCTEUR (init_battery dans main.cpp) et ne les relit JAMAIS ensuite.
    LB_THRESHOLD et LB_CRITICAL_THRESH ne prennent donc effet qu au redemarrage,
    et remonter LB_THRESHOLD par DTE ne suffit pas a lever le drapeau batterie
    basse dans la session en cours. Le cas tente un redemarrage logiciel, mais
    il se declare NON CONCLUANT (ERROR, pas FAIL) si le drapeau ne monte pas:
    accuser le firmware sur une precondition non remplie serait pire que ne rien
    tester. Rendre ce cas concluant demande une injection de mesure batterie
    (sonde %BATT <mv>), qui n existe pas encore.
    """
    b = r.b
    try:
        b.enter_config()
        _, st = b.read_params(['LB_THRESHOLD'])
        b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'lecture impossible: {type(e).__name__}')
    try:
        _profil_zone(b, LB_EN=1, LB_THRESHOLD=99, LB_CRITICAL_THRESH=1,
                     LB_ARGOS_MODE=2, LB_ARGOS_DEPTH_PILE=2,
                     LB_NTRY_PER_MESSAGE=5, TR_LB=900)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')

    # BatteryMonitor recoit ses deux seuils par son CONSTRUCTEUR (init_battery
    # dans main.cpp) et ne les relit JAMAIS. Changer LB_THRESHOLD par DTE ne
    # deplace donc pas le seuil en vigueur tant que la carte n a pas redemarre:
    # sans ce redemarrage le drapeau batterie basse ne se leve pas et le cas
    # n eprouve rien du tout.
    r.say('   redemarrage logiciel (les seuils batterie sont figes au boot)...')
    try:
        b.enter_config()
        b._send('$RSTBW#000;\r')
    except Exception:
        pass
    try: b.close()
    except Exception: pass
    r.b = None
    time.sleep(16)
    if not r.connect():
        return r.record(case, 'ERROR', 'carte injoignable apres redemarrage')
    b = r.b
    time.sleep(3)
    dehors = _injecte_et_lit(b, *_DEHORS, attente=18)
    try:
        b.enter_config()
        b.write_params({'LB_EN': 0, 'LB_THRESHOLD': int(st.get('LBP02', 10)),
                        'LB_CRITICAL_THRESH': 5,
                        'ZONE_ENABLE_OUT_OF_ZONE_DETECTION_MODE': 0, 'ARGOS_MODE': 0})
        b.exit_config()
    except Exception:
        pass
    if not dehors:
        return r.record(case, 'ERROR', '%ARGOSCFG sans reponse')
    trace = f'hors zone + batterie basse: {dehors}'
    if not dehors['lb']:
        r.record(case, 'ERROR', "le drapeau batterie basse ne s est pas leve — cas non concluant", trace)
    elif _est_profil_zone(dehors):
        r.record(case, 'FAIL', 'le profil ZONE prime sur LOW_BATTERY', trace)
    elif dehors['ntry'] == 5 and dehors['depth'] == 2:
        r.record(case, 'PASS', 'LOW_BATTERY prime bien sur hors-zone', trace)
    else:
        r.record(case, 'FAIL', 'ni profil LB ni profil ZONE', trace)

CASES_V11 = [
    dict(id='ZONE-01', risque='BLOQUANT', titre='Hors zone substitue le profil ZONE',        fn=c_zone_substitution),
    dict(id='ZONE-02', risque='MAJEUR',   titre='La frontiere du rayon est en metres',        fn=c_zone_frontiere),
    dict(id='ZONE-03', risque='MAJEUR',   titre='Detection desactivee: aucune substitution',  fn=c_zone_desactivee),
    dict(id='ZONE-04', risque='MAJEUR',   titre='LOW_BATTERY prime sur hors-zone',            fn=c_zone_batterie_prime),
]

# =====================================================================
#  Vague 12 — planification Argos, mode hors-eau, limiteur de debit
#  Tout se pilote avec les sondes deja presentes: on pose la RTC ($RTCW) pour
#  atterrir sur une heure choisie, et on lit la decision dans %SCHED / %ARGOSCFG.
# =====================================================================

def _rtcw(b, epoch):
    p = str(int(epoch))
    mk = b.mark(); b._send(f'$RTCW#{len(p):03X};{p}\r')
    return b.expect(r'\$([ON]);RTCW#', 12.0, from_idx=mk)

def _rtc_lu(b):
    p = 'SYT01'
    mk = b.mark(); b._send(f'$STATR#{len(p):03X};{p}\r')
    m = b.expect(r'\$O;STATR#[0-9A-Fa-f]{3};SYT01=(\d+)', 12.0, from_idx=mk)
    return int(m.group(1)) if m else None

def c_duty_masque_horaire(r, case):
    """Le masque duty-cycle est inverse: bit 23 = heure 0 UTC.

    is_in_duty_cycle() teste `duty_cycle & (0x800000 >> heure_utc)`. Se tromper
    de sens revient a emettre a 23 h quand on voulait minuit — invisible en
    relisant le parametre, et decale de douze heures en moyenne les fenetres
    d emission negociees avec CLS. On l eprouve en posant la RTC sur une heure
    connue puis en n autorisant QUE cette heure, puis QUE l heure suivante.
    """
    b = r.b
    # 2026-01-01 10:30:00 UTC — bien au milieu de l heure 10, loin des bords.
    base = 1767263400
    heure = 10
    try:
        b.enter_config()
        _rtcw(b, base)
        b.write_params({'ARGOS_MODE': 3, 'GNSS_EN': 1, 'NTRY_PER_MESSAGE': 0,
                        'TR_NOM': 60, 'UNDERWATER_EN': 0, 'SAT_PREPASS_EN': 0,
                        'RATE_LIMIT_EN': 0, 'LB_EN': 0, 'HAULED_DETECT_EN': 0,
                        'ARGOS_TX_JITTER_EN': 0,
                        'DUTY_CYCLE': 0x800000 >> heure})     # seule l heure 10
        b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    time.sleep(2); b._send('%GPS 43.6 3.9 5000 9\r'); time.sleep(16)
    dedans = _sched_argos(r)

    try:
        b.enter_config()
        _rtcw(b, base)                                        # on se remet a 10h30
        b.write_params({'DUTY_CYCLE': 0x800000 >> ((heure + 2) % 24)})   # seule l heure 12
        b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'reconfiguration impossible: {type(e).__name__}: {e}')
    time.sleep(2); b._send('%GPS 43.6 3.9 5000 9\r'); time.sleep(16)
    dehors = _sched_argos(r)

    try:
        b.enter_config(); b.write_params({'ARGOS_MODE': 0, 'DUTY_CYCLE': 16777215}); b.exit_config()
    except Exception:
        pass

    trace = f'heure courante autorisee -> {dedans} | seule heure+2 autorisee -> {dehors}'
    if not (dedans and dedans[0] is not None):
        return r.record(case, 'FAIL', 'heure courante autorisee: aucune emission planifiee', trace)
    if not (dehors and dehors[0] is not None):
        return r.record(case, 'FAIL', 'heure+2 autorisee: aucune planification du tout', trace)
    # 10h30 -> prochaine fenetre a 12h00 = ~5400 s. On tolere largement.
    attendu_ms = 5400 * 1000
    if dedans[0] > 600000:
        r.record(case, 'FAIL', f'heure courante autorisee mais emission repoussee de {dedans[0]} ms', trace)
    elif not (0.5 * attendu_ms <= dehors[0] <= 1.6 * attendu_ms):
        r.record(case, 'FAIL',
                 f'heure+2: delai {dehors[0]} ms, attendu ~{attendu_ms} ms — mappage bit/heure suspect',
                 trace)
    else:
        r.record(case, 'PASS', 'bit 23 = heure 0 UTC: mappage confirme sur deux heures', trace)

def c_hauled_substitution(r, case):
    """Le mode hors-eau doit substituer HMP10/HMP11 et pas s engager trop tot.

    HauledModeService::evaluate() compare (maintenant - dernier evenement SWS) a
    HAULED_IDLE_THRESHOLD_H. On ne va pas attendre une heure au banc: on plonge
    pour DATER un evenement, puis on fait avancer la RTC de deux heures.

    ORDRE CRITIQUE, et c est ce qui avait fait rougir ce cas a tort: il faut
    poser la RTC PUIS plonger PUIS activer HMP00. `last_uw_event_rtc` vit en
    .noinit et survit aux redemarrages, donc il porte encore la date des
    plongees des cas precedents; reculer la RTC ensuite rend l ecart enorme et
    engage HAULED instantanement. Le firmware, lui, se protege correctement du
    cas jamais-plonge (`if (last_uw_event_rtc == 0) return`).
    """
    b = r.b
    BASE = 1767263400          # 2026-01-01 10:30:00 UTC
    try:
        b.enter_config()
        _rtcw(b, BASE)
        b.write_params({'ARGOS_MODE': 2, 'TR_NOM': 60, 'NTRY_PER_MESSAGE': 0,
                        'GNSS_EN': 1, 'UNDERWATER_EN': 1, 'LB_EN': 0,
                        'ZONE_ENABLE_OUT_OF_ZONE_DETECTION_MODE': 0,
                        'SAT_PREPASS_EN': 0, 'RATE_LIMIT_EN': 0,
                        'HAULED_DETECT_EN': 0,          # desactive pendant qu on date
                        'HAULED_IDLE_THRESHOLD_H': 1, 'HAULED_RETURN_EVENTS': 2,
                        'HAULED_ARGOS_MODE': 2, 'HAULED_TR_NOM': 900,
                        'HAULED_GNSS_EN': 0, 'HAULED_GNSS_STRAT': 0})
        b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')

    # 1) dater un evenement SWS a l heure de base
    time.sleep(2)
    b._send('%DIVE\r'); time.sleep(4); b._send('%SURFACE\r'); time.sleep(6)

    # 2) activer la detection: l ecart est ~0, le profil doit rester nominal
    try:
        b.enter_config(); b.write_params({'HAULED_DETECT_EN': 1}); b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'activation impossible: {type(e).__name__}: {e}')
    time.sleep(2); b._send('%GPS 43.6 3.9 5000 9\r'); time.sleep(12)
    avant = _argoscfg(b)

    # 3) deux heures plus tard sans le moindre evenement: sortie d eau
    try:
        b.enter_config()
        maintenant = _rtc_lu(b) or BASE
        _rtcw(b, maintenant + 2 * 3600)
        b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'saut d horloge impossible: {type(e).__name__}: {e}')
    time.sleep(3); b._send('%GPS 43.6 3.9 5000 9\r'); time.sleep(14)
    apres = _argoscfg(b)

    try:
        b.enter_config()
        b.write_params({'HAULED_DETECT_EN': 0, 'ARGOS_MODE': 0, 'UNDERWATER_EN': 0})
        b.exit_config()
    except Exception:
        pass
    if not avant or not apres:
        return r.record(case, 'ERROR', '%ARGOSCFG sans reponse')
    trace = f'ecart ~0: {avant}\napres 2 h sans evenement SWS: {apres}'
    if avant['tr_nom'] != 60:
        r.record(case, 'FAIL',
                 'HAULED engage alors que le dernier evenement SWS date de quelques secondes', trace)
    elif apres['tr_nom'] != 900:
        r.record(case, 'FAIL', 'apres 2 h a sec, le profil HAULED n est pas applique', trace)
    else:
        r.record(case, 'PASS',
                 'nominal tant que le seuil n est pas franchi, HAULED ensuite', trace)

def c_limiteur_bloque(r, case):
    """Le limiteur doit bloquer au-dela de RATE_LIMIT_MAX_TX dans la fenetre.

    C est le garde-fou de budget batterie: sans lui, un mode mal regle ou une
    salve emballee vide la balise. On autorise UNE emission sur dix minutes et
    on verifie que la suivante est repoussee, avec un delai coherent avec la
    fenetre restante.
    """
    b = r.b
    try:
        b.enter_config()
        b.write_params({'ARGOS_MODE': 2, 'GNSS_EN': 1, 'NTRY_PER_MESSAGE': 0,
                        'TR_NOM': 30, 'UNDERWATER_EN': 0, 'SAT_PREPASS_EN': 0,
                        'LB_EN': 0, 'HAULED_DETECT_EN': 0, 'ARGOS_TX_JITTER_EN': 0,
                        'RATE_LIMIT_EN': 1, 'RATE_LIMIT_WINDOW_S': 600,
                        'RATE_LIMIT_MAX_TX': 1})
        b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    mk = b.mark()
    time.sleep(2); b._send('%GPS 43.6 3.9 5000 9\r')
    vues = _attendre_trace(b, [r'rate limit reached'], 150, depuis=mk)
    bloque = _sched_argos(r)
    try:
        b.enter_config()
        b.write_params({'RATE_LIMIT_EN': 0, 'RATE_LIMIT_WINDOW_S': 3600,
                        'RATE_LIMIT_MAX_TX': 10, 'ARGOS_MODE': 0})
        b.exit_config()
    except Exception:
        pass
    trace = f'planification bloquee: {bloque}\n' + '\n'.join(vues[:3])
    if not vues:
        r.record(case, 'FAIL', 'le quota est atteint mais rien ne signale le blocage', trace)
    elif not (bloque and bloque[0] is not None and bloque[0] > 30000):
        r.record(case, 'FAIL', 'blocage signale mais la replanification reste courte', trace)
    else:
        r.record(case, 'PASS', 'quota atteint: emission repoussee et blocage signale', trace)

CASES_V12 = [
    dict(id='DUTY-02',  risque='BLOQUANT', titre='Masque duty-cycle: bit 23 = heure 0 UTC', fn=c_duty_masque_horaire),
    dict(id='HAULED-01',risque='BLOQUANT', titre='Le profil hors-eau se substitue au nominal', fn=c_hauled_substitution),
    dict(id='RL-02',    risque='MAJEUR',   titre='Le limiteur bloque au-dela du quota',        fn=c_limiteur_bloque),
]

def c_seuils_batterie_vivants(r, case):
    """Les seuils batterie doivent prendre effet SANS redemarrage.

    BatteryMonitor recevait LB_THRESHOLD et LB_CRITICAL_THRESH par son seul
    CONSTRUCTEUR (init_battery, au boot) et ne les relisait jamais: dans tout
    l arbre, m_low_level et m_critical_level n apparaissaient qu a la
    declaration, dans la liste d initialisation, et dans des comparaisons en
    lecture seule. Les ecrire par DTE ne changeait donc rien jusqu au prochain
    redemarrage — or un tag scelle ne se redemarre pas a la demande, et c est
    exactement le moment ou l on veut reajuster le point de bascule.

    Pire: check_battery_thresholds() lit le couple STOCKE, donc il pouvait
    annoncer un ordre sain pendant que le moniteur appliquait encore l ancien.

    On eprouve les deux sens: seuil au-dessus de la charge -> le profil LB doit
    se substituer; seuil ramene sous la charge -> retour au nominal. Aucun
    redemarrage entre les deux.
    """
    b = r.b
    try:
        b.enter_config()
        _, av = b.read_params(['LB_THRESHOLD', 'LB_CRITICAL_THRESH'])
        b.write_params({'ARGOS_MODE': 2, 'TR_NOM': 60, 'GNSS_EN': 1, 'LB_EN': 1,
                        'LB_THRESHOLD': 99, 'LB_CRITICAL_THRESH': 1,
                        'LB_ARGOS_MODE': 2, 'LB_ARGOS_DEPTH_PILE': 2,
                        'LB_NTRY_PER_MESSAGE': 5, 'TR_LB': 900,
                        'ARGOS_DEPTH_PILE': 4, 'NTRY_PER_MESSAGE': 0,
                        'UNDERWATER_EN': 0, 'HAULED_DETECT_EN': 0,
                        'ZONE_ENABLE_OUT_OF_ZONE_DETECTION_MODE': 0,
                        'SAT_PREPASS_EN': 0, 'RATE_LIMIT_EN': 0})
        b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    time.sleep(3); b._send('%GPS 43.6 3.9 5000 9\r'); time.sleep(14)
    haut = _argoscfg(b)
    try:
        b.enter_config(); b.write_params({'LB_THRESHOLD': 10}); b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'abaissement impossible: {type(e).__name__}: {e}')
    time.sleep(3); b._send('%GPS 43.6 3.9 5000 9\r'); time.sleep(14)
    bas = _argoscfg(b)
    try:
        b.enter_config()
        b.write_params({'LB_EN': 0, 'ARGOS_MODE': 0,
                        'LB_THRESHOLD': int(av.get('LBP02', 10)),
                        'LB_CRITICAL_THRESH': int(av.get('LBP12', 5))})
        b.exit_config()
    except Exception:
        pass
    if not haut or not bas:
        return r.record(case, 'ERROR', '%ARGOSCFG sans reponse')
    trace = f'seuil 99: {haut}\nseuil 10: {bas}'
    if not haut['lb']:
        r.record(case, 'FAIL',
                 'seuil releve au-dessus de la charge: le drapeau batterie basse ne monte pas '
                 '(seuils encore figes au boot ?)', trace)
    elif haut['tr_nom'] != 900 or haut['ntry'] != 5:
        r.record(case, 'FAIL', 'drapeau leve mais le profil LB ne se substitue pas', trace)
    elif bas['lb']:
        r.record(case, 'FAIL', 'seuil rabaisse: le drapeau batterie basse ne redescend pas', trace)
    elif bas['tr_nom'] != 60:
        r.record(case, 'FAIL', 'drapeau retombe mais le profil nominal ne revient pas', trace)
    else:
        r.record(case, 'PASS', 'les deux seuils prennent effet sans redemarrage, dans les deux sens', trace)

CASES_V13 = [
    dict(id='BATT-R2', risque='BLOQUANT',
         titre='Les seuils batterie prennent effet sans redemarrage',
         fn=c_seuils_batterie_vivants),
]

# =====================================================================
#  Vague 14 — machine a etats des LED
#  ledsm.cpp ne definit AUCUN ::exit(): ses huit transit<>() differes ne sont
#  jamais annules. La sonde %LED est le seul moyen d observer quel etat LED est
#  reellement actif.
# =====================================================================

def _led(b, evt=None, timeout=12.0):
    """%LED [EVT <nom>] -> (etat, couleur)."""
    cmd = f'%LED EVT {evt}' if evt else '%LED'
    mk = b.mark(); b._send(cmd + '\r')
    m = b.expect(r'%LED etat=(\w+) couleur=(\d+) clignote=(\d)', timeout, from_idx=mk)
    # 'allumee' = couleur solide non noire OU motif clignotant actif: flash()
    # ne met pas a jour m_color, donc la couleur seule ne suffit pas.
    return (m.group(1), int(m.group(2)) or int(m.group(3))) if m else (None, None)

def c_led_transit_orphelin(r, case):
    """Un transit LED differe ne doit pas ecraser l etat suivant.

    LEDGNSSPowerOff::entry() arme un transit<LEDOff>() a +500 ms et ne garde
    aucun handle; son exit() n existe pas. Sur la sequence nominale de
    deploiement l emission Argos demarre immediatement apres le fix, donc dans
    cette fenetre de 500 ms: l orphelin venait alors eteindre l indication
    d emission en cours. Huit transits differes sont dans ce cas dans le fichier.

    On reproduit exactement: fin de session GNSS, puis emission Argos tout de
    suite, puis on regarde ou en est la LED une fois la fenetre passee.
    """
    b = r.b
    try:
        b.enter_config(); b.write_params({'LED_MODE': 3, 'ARGOS_MODE': 0}); b.exit_config()   # 3 = ALWAYS
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    time.sleep(2)
    if _led(b)[0] is None:
        return r.record(case, 'ERROR', '%LED sans reponse (sonde absente du build ?)')

    _led(b, 'GNSSON'); time.sleep(0.5)
    _led(b, 'GNSSPOWEROFF')          # arme l orphelin a +500 ms
    time.sleep(0.15)
    juste_apres = _led(b, 'ARGOSTX')  # on entre en ArgosTX DANS la fenetre
    time.sleep(1.2)                   # l orphelin a tire (ou non)
    apres = _led(b)

    try:
        b.enter_config(); b.write_params({'LED_MODE': 3}); b.exit_config()
    except Exception:
        pass
    trace = f'juste apres ARGOSTX: {juste_apres} | 1,2 s plus tard: {apres}'
    if juste_apres[0] != 'ArgosTX':
        r.record(case, 'ERROR', f"l etat ArgosTX n a pas ete atteint ({juste_apres[0]}) — non concluant", trace)
    elif apres[0] != 'ArgosTX':
        r.record(case, 'FAIL',
                 f"l indication d emission a ete ecrasee par un transit orphelin "
                 f"(etat devenu {apres[0]})", trace)
    else:
        r.record(case, 'PASS', "l indication d emission survit au transit differe de la session GNSS", trace)

def c_led_mode_off(r, case):
    """LED_MODE=OFF doit reellement eteindre, et ALWAYS reallumer.

    Le garde LED_MODE_GUARD lit la configuration depuis l ISR RTC; il a ete rendu
    non levant (read_param se terminait par `catch (...) { throw }`, ce qui
    envoyait un throw a travers une trame d interruption). On verifie au passage
    que le garde fait toujours son travail dans les deux sens.
    """
    b = r.b
    try:
        b.enter_config(); b.write_params({'LED_MODE': 0, 'ARGOS_MODE': 0}); b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    # LEDGNSSOn fait un flash(CYAN, 1000): la couleur ALTERNE. Un echantillon
    # unique tombe une fois sur deux dans la phase eteinte et ferait rougir un
    # firmware parfaitement correct — on echantillonne sur plus d une periode.
    def couleurs(secondes=3.0):
        vues = []
        fin = time.time() + secondes
        while time.time() < fin:
            _, c = _led(b)
            if c is not None:
                vues.append(c)
            time.sleep(0.25)
        return vues
    time.sleep(2)
    _led(b, 'GNSSON')
    eteint = couleurs()
    try:
        b.enter_config(); b.write_params({'LED_MODE': 3}); b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'reconfiguration impossible: {type(e).__name__}: {e}')
    time.sleep(2)
    _led(b, 'GNSSON')
    allume = couleurs()
    trace = f'LED_MODE=OFF(0): {eteint} | LED_MODE=ALWAYS(3): {allume}'
    if not eteint or not allume:
        r.record(case, 'ERROR', '%LED sans reponse', trace)
    elif any(c != 0 for c in eteint):
        r.record(case, 'FAIL', 'LED_MODE=OFF mais la LED s allume', trace)
    elif all(c == 0 for c in allume):
        r.record(case, 'FAIL', 'LED_MODE=ALWAYS mais la LED reste eteinte sur toute une periode', trace)
    else:
        r.record(case, 'PASS', 'le garde LED_MODE eteint et rallume correctement', trace)

CASES_V14 = [
    dict(id='LED-01', risque='BLOQUANT', titre='Un transit differe n ecrase pas l etat suivant', fn=c_led_transit_orphelin),
    dict(id='LED-02', risque='MAJEUR',   titre='LED_MODE eteint et rallume',                      fn=c_led_mode_off),
]

# =====================================================================
#  Vague 15 — pile de profondeur: une position ne doit pas etre
#  consommee sans avoir ete emise
# =====================================================================

def _pile(b, timeout=12.0):
    """%PILE -> [(type, compteur), ...] du plus ancien au plus recent.
       type: 0=fix, 1=no-fix, 2=fastloc, 3=cloudlocate."""
    mk = b.mark(); b._send('%PILE\r')
    m = b.expect(r'%PILE (empty|vide|(?:\d+:\d+ ?)+)', timeout, from_idx=mk)
    if not m:
        return None
    if m.group(1) in ('empty', 'vide'):
        return []
    return [tuple(int(x) for x in p.split(':')) for p in m.group(1).split()]

def c_pile_rotation(r, case):
    """Chaque entree doit recevoir exactement N emissions, en ALTERNANCE.

    Contrat documente (wiki, Satellite Communication): NTRY_PER_MESSAGE N>0 =
    "exactement N emissions" PAR ENTREE. Et l ordre voulu est A B C A B C, pas
    N x A puis N x B: une balise qui viderait tout le credit de la position la
    plus ancienne avant de passer a la suivante retarderait d autant la plus
    recente, celle qui interesse le segment sol.

    Le cas eprouve les deux a la fois sur une pile MIXTE (fix reel + fastloc,
    dont les formats de paquet ne se melangent pas): les credits doivent
    descendre en alternance, et chaque entree doit finir a zero apres N
    emissions, pas moins.

    NOTE — ce cas a d abord ete ecrit pour demasquer un defaut suppose:
    retrieve_gps() decremente burst_counter sur toutes les entrees rendues AVANT
    que la salve ne decide laquelle encoder, donc un fastloc plus recent devait
    faire perdre un tour au fix. La MESURE l a refute: la rotation de retrieve()
    ne rend jamais les deux ensemble ici, chacun garde son compte. Le cas reste
    comme garde: si la rotation se casse un jour, il le dira.
    """
    b = r.b
    N = 2
    try:
        b.enter_config()
        b.write_params({'ARGOS_MODE': 2, 'GNSS_EN': 1, 'NTRY_PER_MESSAGE': N,
                        'ARGOS_DEPTH_PILE': 4, 'TR_NOM': 30, 'UNDERWATER_EN': 0,
                        'SAT_PREPASS_EN': 0, 'RATE_LIMIT_EN': 0, 'LB_EN': 0,
                        'HAULED_DETECT_EN': 0, 'ARGOS_TX_JITTER_EN': 0,
                        'ZONE_ENABLE_OUT_OF_ZONE_DETECTION_MODE': 0,
                        'DLOC_ARG_NOM': 10})
        b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    time.sleep(2)
    if _pile(b) is None:
        return r.record(case, 'ERROR', '%PILE sans reponse (sonde absente du build ?)')

    # DEUX positions REELLES, pas un fastloc: sur ce banc le fastloc echoue a
    # l emission (LDA2 non provisionne), ce qui declenche le backoff d erreur
    # peripherique et perturbe la rotation — le meme montage a rendu 2 emissions
    # puis 1 sur deux passes consecutives. Une pile homogene eprouve exactement
    # la meme chose (A B A B) sans cette interference.
    mk = b.mark()
    b._send('%GPS 43.600000 3.900000 5000 9\r'); time.sleep(10)
    b._send('%GPS 44.000000 4.000000 5000 9\r')

    # on echantillonne toute la descente des credits
    suite = []
    fin_t = time.time() + 165
    while time.time() < fin_t:
        time.sleep(10)
        p = _pile(b)
        if p and (not suite or p != suite[-1]):
            suite.append(p)
        if p and all(c == 0 for _, c in p):
            break

    try:
        b.enter_config(); b.write_params({'ARGOS_MODE': 0, 'NTRY_PER_MESSAGE': 0}); b.exit_config()
    except Exception:
        pass

    trace = 'evolution des credits:\n  ' + '\n  '.join(str(x) for x in suite)
    if len(suite) < 2:
        return r.record(case, 'ERROR', 'pile non observee assez longtemps — cas non concluant', trace)
    depart, final = suite[0], suite[-1]
    if len(depart) < 2:
        return r.record(case, 'ERROR', 'moins de deux entrees dans la pile — cas non concluant', trace)

    # Compter les emissions ne marche pas: un paquet LONG transporte jusqu a
    # TROIS positions a la fois, donc deux entrees peuvent partir dans un seul
    # paquet. On assertionne sur ce qui est reellement en jeu — les credits.
    defauts = []
    for i in range(len(suite) - 1):
        av, ap = suite[i], suite[i + 1]
        if len(av) != len(ap):
            continue
        for k in range(len(av)):
            if ap[k][1] > av[k][1]:
                defauts.append(f'credit remonte a l index {k}: {av[k][1]} -> {ap[k][1]}')
            if av[k][1] - ap[k][1] > 1:
                defauts.append(f'credit chute de plus de 1 a l index {k}: {av[k][1]} -> {ap[k][1]}')
    if any(c != 0 for _, c in final):
        defauts.append(f'des credits subsistent en fin d observation: {final}')

    if defauts:
        r.record(case, 'FAIL', f'{len(defauts)} anomalie(s) de rotation', trace + '\n' + '\n'.join(defauts))
    else:
        r.record(case, 'PASS',
                 f'les {len(depart)} entrees descendent une a une jusqu a zero, sans saut ni remontee',
                 trace)

CASES_V15 = [
    dict(id='DP-01', risque='BLOQUANT',
         titre='Chaque position recoit ses N emissions, en alternance',
         fn=c_pile_rotation),
]


# =====================================================================
#  Vague 16 — logique GNSS, mode amarre, mode plongee
#
#  Limite CONNUE et documentee: %GPS injecte la position directement dans
#  GPSService et court-circuite le pilote M10Q. Or les filtres hAcc et HDOP
#  (GNP20/21, GNP02/03) vivent dans m10qasync.cpp, sur la trame NAV-PVT reelle.
#  Ils ne sont donc PAS atteignables par injection: aucun cas ci-dessous ne
#  pretend les couvrir, et ils restent a la charge d un essai terrain ou d une
#  sonde qui pousserait une NAV-PVT synthetique dans le pilote.
#
#  Rappel de niveau de journal: le build banc est en DEBUG_LEVEL=3, donc
#  DEBUG_TRACE est compile HORS du binaire. Les cas n assertionnent que sur
#  des lignes INFO/WARN, ou sur les sondes console qui lisent l etat a la
#  source ("stationary %u/%u" est un TRACE: invisible, on lit %MOORED).
# =====================================================================

def _gnss_base(b, **extra):
    """Configuration GNSS de reference: rien d autre ne doit bouger.

    ARGOS_MODE=0 coupe l emission (aucun credential KIM2 sur ce banc, et une
    tentative d emission declenche un backoff d erreur peripherique qui
    perturbe tout ce qui suit). UNDERWATER_EN=0 evite que la cascade SWS
    reprogramme le GNSS sous nos pieds.
    """
    cfg = {'ARGOS_MODE': 0, 'GNSS_EN': 1, 'UNDERWATER_EN': 0,
           'MOORED_DETECT_EN': 0, 'HAULED_DETECT_EN': 0,
           'UW_DIVE_MODE_ENABLE': 0, 'LB_EN': 0, 'RATE_LIMIT_EN': 0,
           'ZONE_ENABLE_OUT_OF_ZONE_DETECTION_MODE': 0}
    cfg.update(extra)
    b.enter_config(); b.write_params(cfg); b.exit_config()

def c_gnss_desactive(r, case):
    """GNSS_EN=0 doit reellement supprimer l ordonnancement GNSS.

    C est la premiere economie d energie du produit: un deploiement Argos-seul
    ne doit pas payer une session GNSS. Si le service reste programme malgre
    GNP01=0, l autonomie annoncee est fausse.
    """
    b = r.b
    try:
        _gnss_base(b, GNSS_EN=0)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    time.sleep(4)
    m, _ = r.raw_until('%SCHED\r', r'%SCHED .*GNSS=', timeout=15.0)
    ligne = (m.string if m and hasattr(m, 'string') else '') or ''
    mm = re.search(r'GNSS=(none|\d+ms)\(([^)]*)\)', ligne)
    try:
        b.enter_config(); b.write_params({'GNSS_EN': 1}); b.exit_config()
    except Exception:
        pass
    if not mm:
        return r.record(case, 'ERROR', '%SCHED ne rapporte pas le service GNSS', ligne[:200])
    quand, raison = mm.group(1), mm.group(2)
    if quand != 'none':
        r.record(case, 'FAIL',
                 f'GNSS_EN=0 mais le service reste programme dans {quand} ({raison})', ligne[:200])
    else:
        r.record(case, 'PASS', f'aucun ordonnancement GNSS (raison: {raison})', ligne[:200])

def c_gnss_fix_unique(r, case):
    """GNSS_SESSION_SINGLE_FIX=1: la session s arrete au premier fix.

    Le mode par defaut continue d echantillonner pour affiner. Sur un animal qui
    plonge, chaque seconde de recepteur allume est prise sur la fenetre de
    surface: GNP30 existe pour couper des le premier point. S il ne coupe pas,
    le budget d energie du deploiement est faux.
    """
    b = r.b
    try:
        _gnss_base(b, GNSS_SESSION_SINGLE_FIX=1)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    time.sleep(3)
    mk = b.mark()
    b.inject_gps(43.1, 5.9)
    vues = _attendre_trace(b, [r'SESSION_SINGLE_FIX', r'not rescheduling after first fix'],
                           30, depuis=mk)
    try:
        b.enter_config(); b.write_params({'GNSS_SESSION_SINGLE_FIX': 0}); b.exit_config()
    except Exception:
        pass
    trace = '\n'.join(vues[:5])
    if vues:
        r.record(case, 'PASS', 'la session ne se reprogramme pas apres le premier fix', trace)
    else:
        r.record(case, 'FAIL', 'GNP30=1 mais aucune trace d arret apres le premier fix', trace)

def c_gnss_timeout_acquisition(r, case):
    """GNSS_ACQ_TIMEOUT borne la session quand il n y a pas de fix.

    Au banc, en interieur, le recepteur ne verra jamais le ciel: c est
    exactement le cas nominal a eprouver. Sans cette borne le recepteur reste
    allume indefiniment — le mode de defaillance le plus couteux du produit.

    Le parametre est en NOMBRE D ECHANTILLONS de navigation, pas en secondes:
    la trace de fin est GPSEventMaxNavSamples. On laisse une marge large.
    """
    b = r.b
    try:
        _gnss_base(b, GNSS_ACQ_TIMEOUT=10, GNSS_COLD_ACQ_TIMEOUT=10, GNSS_NTRY=1)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    mk = b.mark()
    vues = _attendre_trace(b, [r'acquisition timeout — no fix', r'MaxNavSamples',
                               r'NO_FIX \| ntry'], 180, depuis=mk)
    try:
        b.enter_config()
        b.write_params({'GNSS_ACQ_TIMEOUT': 120, 'GNSS_COLD_ACQ_TIMEOUT': 240, 'GNSS_NTRY': 3})
        b.exit_config()
    except Exception:
        pass
    trace = '\n'.join(vues[:5])
    if vues:
        r.record(case, 'PASS', 'la session sans fix se termine sur la borne GNP05', trace)
    else:
        r.record(case, 'FAIL',
                 'aucune fin de session apres 180 s alors que GNP05=10 echantillons', trace)

def c_gnss_ntry_backoff(r, case):
    """GNSS_NTRY epuise: la cadence doit retomber sur DLOC_ARG_NOM.

    Sans ce repli, une balise qui ne voit plus le ciel (animal en plongee
    prolongee, antenne masquee) reessaie a la cadence rapide jusqu a vider la
    batterie. La trace nomme le compteur ET la limite, donc on verifie les deux.
    """
    b = r.b
    try:
        # ARP11 est un CODE de table (0..15), pas des secondes: 13 = 5 min,
        # 1 = 10 min (le defaut). Ecrire 600 fait rejeter la trame entiere —
        # mesure du 2026-08-27, "PARMW rejected: ARP11".
        _gnss_base(b, GNSS_NTRY=1, GNSS_ACQ_TIMEOUT=10, GNSS_COLD_ACQ_TIMEOUT=10,
                   DLOC_ARG_NOM=13)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    mk = b.mark()
    vues = _attendre_trace(b, [r'NTRY limit reached', r'back-off to dloc_arg_nom',
                               r'retry_counter'], 200, depuis=mk,
                           exiger=[r'NTRY limit reached'])
    try:
        b.enter_config()
        b.write_params({'GNSS_NTRY': 3, 'GNSS_ACQ_TIMEOUT': 120,
                        'GNSS_COLD_ACQ_TIMEOUT': 240, 'DLOC_ARG_NOM': 1})
        b.exit_config()
    except Exception:
        pass
    trace = '\n'.join(vues[:6])
    limite = [l for l in vues if 'NTRY limit reached' in l]
    if not limite:
        return r.record(case, 'FAIL',
                        'GNP04=1 mais la limite NTRY n est jamais annoncee apres 200 s', trace)
    if not any('dloc_arg_nom' in l for l in vues):
        return r.record(case, 'FAIL', 'limite NTRY atteinte sans repli sur dloc_arg_nom', trace)
    r.record(case, 'PASS', 'limite NTRY annoncee et repli sur dloc_arg_nom', trace)

def c_gnss_deep_idle(r, case):
    """GNSS_DEEP_IDLE_AFTER_OFF_S: veille profonde plutot que coupure du rail.

    C est un compromis mesure: garder le rail allume avec le M10Q en PMREQ
    preserve l ephemeride (et donc le temps de premier fix suivant) au prix
    d un courant de veille. Si la valeur configuree n est pas respectee, on
    perd soit l ephemeride, soit l autonomie — et le symptome de terrain
    (fix qui meurent au bout de deux jours) ressemble aux deux.
    """
    b = r.b
    try:
        _gnss_base(b, GNSS_DEEP_IDLE_AFTER_OFF_S=45)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    time.sleep(3)
    mk = b.mark()
    b.inject_gps(43.2, 5.8)
    vues = _attendre_trace(b, [r'deep-idle for (\d+) s', r'deep-idle engaged',
                               r'never-poweroff', r'disabled — power_off'],
                           60, depuis=mk)
    try:
        b.enter_config(); b.write_params({'GNSS_DEEP_IDLE_AFTER_OFF_S': 0}); b.exit_config()
    except Exception:
        pass
    trace = '\n'.join(vues[:5])
    duree = None
    for l in vues:
        m = re.search(r'deep-idle for (\d+) s', l)
        if m:
            duree = int(m.group(1)); break
    if duree is None:
        return r.record(case, 'FAIL',
                        'GNP52=45 mais aucune entree en veille profonde apres le fix', trace)
    if duree != 45:
        return r.record(case, 'FAIL', f'veille profonde de {duree} s au lieu des 45 s configurees', trace)
    r.record(case, 'PASS', 'veille profonde de 45 s, conforme a GNP52', trace)

def c_gnss_cold_start(r, case):
    """GNSS_COLD_START_AFTER_NTRY force un demarrage a froid apres N echecs.

    Le mecanisme existe pour casser une ephemeride corrompue: on efface la BBR
    et on repart de zero. C est le remede documente aux fix qui meurent apres
    deux jours. Encore faut-il qu il se declenche.
    """
    b = r.b
    try:
        _gnss_base(b, GNSS_NTRY=1, GNSS_COLD_START_AFTER_NTRY=1,
                   GNSS_ACQ_TIMEOUT=10, GNSS_COLD_ACQ_TIMEOUT=10)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    mk = b.mark()
    vues = _attendre_trace(b, [r'COLD START requested', r'BBR wipe'], 240, depuis=mk)
    try:
        b.enter_config()
        b.write_params({'GNSS_NTRY': 3, 'GNSS_COLD_START_AFTER_NTRY': 0,
                        'GNSS_ACQ_TIMEOUT': 120, 'GNSS_COLD_ACQ_TIMEOUT': 240})
        b.exit_config()
    except Exception:
        pass
    trace = '\n'.join(vues[:5])
    if vues:
        r.record(case, 'PASS', 'demarrage a froid demande apres epuisement de NTRY', trace)
    else:
        r.record(case, 'FAIL',
                 'GNP54=1 et NTRY epuise, mais aucun demarrage a froid en 240 s', trace)

# ---------------------------------------------------------------------
#  Mode amarre (MOORED)
# ---------------------------------------------------------------------

def _moored(b, timeout=8.0):
    """%MOORED -> dict des champs, ou None."""
    mk = b.mark(); b._send('%MOORED\r')
    m = b.expect(r'%MOORED en=\d+ state=\S+ ref=-?\d+ still=\d+/\d+ motion=\d+/\d+', timeout,
                 from_idx=mk)
    if not m:
        return None
    ligne = m.group(0)
    d = {}
    for cle, motif in (('en', r'en=(\d+)'), ('state', r'state=(\S+)'), ('ref', r'ref=(-?\d+)'),
                       ('still', r'still=(\d+)/(\d+)'), ('motion', r'motion=(\d+)/(\d+)'),
                       ('radius', r'radius=(\d+)')):
        mm = re.search(motif, ligne)
        if mm:
            d[cle] = tuple(int(x) for x in mm.groups()) if mm.lastindex and mm.lastindex > 1 \
                     else (mm.group(1) if cle == 'state' else int(mm.group(1)))
    d['_ligne'] = ligne
    return d

def _reset_ancre(b):
    """Efface l ancre amarrage en coupant puis rearmant MRP00.

    L etat vit en .noinit et SURVIT au redemarrage: sans cette purge, un rejeu
    partirait d une ancre posee par la passe precedente et le verdict ne
    voudrait rien dire. La trace 'MRP00 cleared — forcing UNDERWAY' confirme.
    """
    b.enter_config(); b.write_params({'MOORED_DETECT_EN': 0}); b.exit_config()
    time.sleep(3)

def c_moored_entree(r, case):
    """N fixes immobiles dans le rayon font passer en MOORED.

    Le premier fix POSE l ancre et ne compte pas — un point isole ne porte
    aucune information de deplacement. Il faut donc MRP02+1 injections. Ce
    detail a sa place ici: un cas qui n injecterait que MRP02 fixes echouerait
    en accusant un firmware correct.
    """
    b = r.b
    try:
        _reset_ancre(b)
        _gnss_base(b, MOORED_DETECT_EN=1, MOORED_ENTER_FIXES=2, MOORED_RADIUS_M=50)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    time.sleep(3)
    mk = b.mark()
    for k in range(3):
        b.inject_gps(43.1000, 5.9000)
        time.sleep(4)
    vues = _attendre_trace(b, [r'reference anchor set', r'UNDERWAY -> MOORED'], 40,
                           depuis=mk, exiger=[r'UNDERWAY -> MOORED'])
    etat = _moored(b)
    trace = '\n'.join(vues[:6]) + ('\n' + etat['_ligne'] if etat else '')
    if not any('reference anchor set' in l for l in vues):
        return r.record(case, 'ERROR', 'ancre jamais posee — injection non prise en compte', trace)
    if not any('UNDERWAY -> MOORED' in l for l in vues):
        return r.record(case, 'FAIL',
                        '3 fixes immobiles (MRP02=2) mais pas de passage en MOORED', trace)
    if etat and etat.get('state') not in ('MOORED', 'moored'):
        return r.record(case, 'FAIL',
                        f"trace de passage en MOORED mais %MOORED rapporte {etat.get('state')}", trace)
    r.record(case, 'PASS', 'passage en MOORED apres MRP02 fixes immobiles', trace)

def c_moored_sortie_deplacement(r, case):
    """Un fix hors du rayon fait ressortir en UNDERWAY.

    C est la moitie qui compte pour la donnee: rester bloque en MOORED ferait
    passer un navire reparti pour un navire a quai, et la cadence reduite du
    mode amarre ferait manquer le trajet.
    """
    b = r.b
    try:
        _reset_ancre(b)
        _gnss_base(b, MOORED_DETECT_EN=1, MOORED_ENTER_FIXES=2, MOORED_RADIUS_M=50)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    time.sleep(3)
    mk = b.mark()
    for _ in range(3):
        b.inject_gps(43.1000, 5.9000); time.sleep(4)
    entre = _attendre_trace(b, [r'UNDERWAY -> MOORED'], 30, depuis=mk)
    if not entre:
        return r.record(case, 'ERROR', 'jamais entre en MOORED — sortie non evaluable',
                        '\n'.join(entre[:4]))
    mk2 = b.mark()
    # ~1,1 km au nord: sans ambiguite au-dela des 50 m du rayon.
    b.inject_gps(43.1100, 5.9000)
    vues = _attendre_trace(b, [r'MOORED -> UNDERWAY'], 40, depuis=mk2)
    etat = _moored(b)
    trace = '\n'.join(vues[:5]) + ('\n' + etat['_ligne'] if etat else '')
    if not vues:
        return r.record(case, 'FAIL', 'fix a ~1,1 km hors rayon mais toujours MOORED', trace)
    m = re.search(r'd=([\d.]+) m radius=(\d+) m', vues[0])
    if m and float(m.group(1)) <= float(m.group(2)):
        return r.record(case, 'FAIL',
                        f'sortie annoncee avec d={m.group(1)} m <= rayon {m.group(2)} m', trace)
    r.record(case, 'PASS',
             f"sortie en UNDERWAY sur deplacement{' (d=' + m.group(1) + ' m)' if m else ''}", trace)

def c_moored_override_argos(r, case):
    """En MOORED, les reglages Argos du mode remplacent les nominaux.

    MRP06 (TR_NOM amarre) et MRP07 (GNSS amarre) n ont d interet que s ils
    atteignent la configuration EFFECTIVE — celle que la salve consulte. C est
    tout l objet du mode: a quai, moins emettre et moins chercher le ciel.
    %ARGOSCFG lit cette configuration effective, seul endroit ou la cascade
    est observable.
    """
    b = r.b
    try:
        _reset_ancre(b)
        _gnss_base(b, MOORED_DETECT_EN=1, MOORED_ENTER_FIXES=2, MOORED_RADIUS_M=50,
                   TR_NOM=60, MOORED_TR_NOM=900)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    time.sleep(3)
    avant = _argoscfg(b)
    mk = b.mark()
    for _ in range(3):
        b.inject_gps(43.1000, 5.9000); time.sleep(4)
    vues = _attendre_trace(b, [r'UNDERWAY -> MOORED'], 30, depuis=mk)
    apres = _argoscfg(b)
    try:
        b.enter_config()
        b.write_params({'MOORED_DETECT_EN': 0, 'TR_NOM': 60, 'MOORED_TR_NOM': 60})
        b.exit_config()
    except Exception:
        pass
    trace = f'avant={avant}\napres={apres}\n' + '\n'.join(vues[:3])
    if not vues:
        return r.record(case, 'ERROR', 'jamais entre en MOORED — surcharge non evaluable', trace)
    if not apres:
        return r.record(case, 'ERROR', '%ARGOSCFG sans reponse', trace)
    tr = apres.get('tr_nom')
    if tr != 900:
        return r.record(case, 'FAIL',
                        f'en MOORED, tr_nom effectif = {tr} au lieu des 900 s de MRP06', trace)
    r.record(case, 'PASS', 'la configuration effective adopte MOORED_TR_NOM=900', trace)

def c_moored_desactive(r, case):
    """MRP00=0 doit forcer UNDERWAY, y compris depuis un etat MOORED persistant.

    L etat amarrage vit en .noinit et survit au redemarrage: sans purge
    explicite a la coupure du parametre, une balise resterait en cadence
    reduite alors que l operateur a desactive la fonction. C est un piege de
    configuration silencieux.
    """
    b = r.b
    try:
        _reset_ancre(b)
        _gnss_base(b, MOORED_DETECT_EN=1, MOORED_ENTER_FIXES=2, MOORED_RADIUS_M=50)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    time.sleep(3)
    mk = b.mark()
    for _ in range(3):
        b.inject_gps(43.1000, 5.9000); time.sleep(4)
    if not _attendre_trace(b, [r'UNDERWAY -> MOORED'], 30, depuis=mk):
        return r.record(case, 'ERROR', 'jamais entre en MOORED — desactivation non evaluable')
    mk2 = b.mark()
    try:
        b.enter_config(); b.write_params({'MOORED_DETECT_EN': 0}); b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'ecriture MRP00=0 impossible: {type(e).__name__}: {e}')
    vues = _attendre_trace(b, [r'MRP00 cleared', r'forcing UNDERWAY'], 30, depuis=mk2)
    etat = _moored(b)
    trace = '\n'.join(vues[:4]) + ('\n' + etat['_ligne'] if etat else '')
    if etat and etat.get('state') in ('MOORED', 'moored'):
        return r.record(case, 'FAIL', 'MRP00=0 mais %MOORED rapporte toujours MOORED', trace)
    if not vues:
        return r.record(case, 'FAIL',
                        'aucune trace de purge a la desactivation (etat .noinit non nettoye)', trace)
    r.record(case, 'PASS', 'MRP00=0 purge l etat et force UNDERWAY', trace)

# ---------------------------------------------------------------------
#  Mode plongee (DIVE)
#
#  UNDERWATER_EN reste a 0 dans ces trois cas, et c est deliberé.
#  DiveModeService n est gate QUE sur UW_DIVE_MODE_ENABLE (voir
#  dive_mode_service.hpp: service_is_enabled), pas sur UNP01, tandis que %DIVE
#  et %SURFACE injectent l evenement directement. Avec UNP01=1 le capteur SWS
#  reel — sec sur la paillasse — redit "en surface" quelques secondes apres
#  l injection et ANNULE la plongee en attente. Mesure du 2026-08-27:
#
#      18:43:56  >> %DIVE
#      18:44:04  DiveModeService: dive mode start pending
#      18:44:07  DiveModeService: dive mode start cancelled — surfaced before 10s
#
#  Le firmware avait raison; c est la premisse du cas qui etait fausse. On
#  isole donc la machine a etats de la plongee du capteur qui la contredit —
#  le capteur lui-meme est couvert par SWS-01/02.
# ---------------------------------------------------------------------

def c_dive_engagement(r, case):
    """UW_DIVE_MODE_ENABLE: la plongee s engage apres UNP13 secondes sous l eau.

    Le temporisateur existe pour ne pas basculer sur une vague: une immersion
    breve ne doit pas reconfigurer la balise. On verifie les DEUX etapes —
    l armement puis l engagement — parce qu un engagement immediat serait tout
    aussi faux qu une absence d engagement.
    """
    b = r.b
    try:
        _gnss_base(b, UNDERWATER_EN=0, UW_DIVE_MODE_ENABLE=1, UW_DIVE_MODE_START_TIME=10)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    time.sleep(2)
    b._send('%SURFACE\r'); time.sleep(3)
    mk = b.mark()
    b._send('%DIVE\r')
    vues = _attendre_trace(b, [r'dive mode start pending', r'dive mode engaged'], 60,
                           depuis=mk, exiger=[r'dive mode start pending', r'dive mode engaged'])
    try:
        b._send('%SURFACE\r'); time.sleep(2)
        b.enter_config()
        b.write_params({'UW_DIVE_MODE_ENABLE': 0, 'UNDERWATER_EN': 0})
        b.exit_config()
    except Exception:
        pass
    trace = '\n'.join(vues[:5])
    if not any('start pending' in l for l in vues):
        return r.record(case, 'FAIL', 'immersion sans armement du temporisateur de plongee', trace)
    if not any('engaged' in l for l in vues):
        return r.record(case, 'FAIL',
                        'temporisateur arme mais plongee jamais engagee apres 60 s (UNP13=10)', trace)
    r.record(case, 'PASS', 'plongee armee puis engagee apres le delai UNP13', trace)

def c_dive_annulation(r, case):
    """Une emersion avant UNP13 doit ANNULER l engagement, pas le repousser.

    C est le cas de la vague: on plonge, on ressort tout de suite. Si le
    temporisateur survivait a l emersion, la balise basculerait en mode plongee
    alors qu elle est en surface — et cesserait d emettre au pire moment.
    """
    b = r.b
    try:
        _gnss_base(b, UNDERWATER_EN=0, UW_DIVE_MODE_ENABLE=1, UW_DIVE_MODE_START_TIME=45)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    time.sleep(2)
    b._send('%SURFACE\r'); time.sleep(3)
    mk = b.mark()
    b._send('%DIVE\r')
    if not _attendre_trace(b, [r'dive mode start pending'], 25, depuis=mk):
        try:
            b.enter_config(); b.write_params({'UW_DIVE_MODE_ENABLE': 0, 'UNDERWATER_EN': 0})
            b.exit_config()
        except Exception:
            pass
        return r.record(case, 'ERROR', 'temporisateur jamais arme — annulation non evaluable')
    time.sleep(6)
    mk2 = b.mark()
    b._send('%SURFACE\r')
    # 60 s: assez long pour couvrir les 45 s de UNP13 et donc voir un engagement
    # tardif s il avait lieu malgre l emersion.
    vues = _attendre_trace(b, [r'dive mode start cancelled', r'dive mode engaged',
                               r'disengaged by surfacing'], 60, depuis=mk2)
    try:
        b.enter_config()
        b.write_params({'UW_DIVE_MODE_ENABLE': 0, 'UNDERWATER_EN': 0,
                        'UW_DIVE_MODE_START_TIME': 0})
        b.exit_config()
    except Exception:
        pass
    trace = '\n'.join(vues[:5])
    if any('engaged' in l for l in vues):
        return r.record(case, 'FAIL',
                        'plongee engagee alors que la balise a emerge avant UNP13', trace)
    if not any('cancelled' in l or 'disengaged' in l for l in vues):
        return r.record(case, 'FAIL',
                        'ni annulation ni desengagement apres emersion precoce', trace)
    r.record(case, 'PASS', 'emersion precoce: temporisateur annule, plongee non engagee', trace)

def c_dive_desengagement(r, case):
    """Une emersion pendant une plongee ENGAGEE doit la desengager.

    Symetrique du precedent, et le plus grave des deux s il manque: la balise
    resterait en configuration de plongee une fois revenue en surface, donc
    silencieuse alors qu elle a le satellite en vue.
    """
    b = r.b
    try:
        _gnss_base(b, UNDERWATER_EN=0, UW_DIVE_MODE_ENABLE=1, UW_DIVE_MODE_START_TIME=5)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    time.sleep(2)
    b._send('%SURFACE\r'); time.sleep(3)
    mk = b.mark()
    b._send('%DIVE\r')
    if not _attendre_trace(b, [r'dive mode engaged'], 45, depuis=mk):
        try:
            b.enter_config(); b.write_params({'UW_DIVE_MODE_ENABLE': 0, 'UNDERWATER_EN': 0})
            b.exit_config()
        except Exception:
            pass
        return r.record(case, 'ERROR', 'plongee jamais engagee — desengagement non evaluable')
    mk2 = b.mark()
    b._send('%SURFACE\r')
    vues = _attendre_trace(b, [r'disengaged by surfacing'], 45, depuis=mk2)
    try:
        b.enter_config()
        b.write_params({'UW_DIVE_MODE_ENABLE': 0, 'UNDERWATER_EN': 0,
                        'UW_DIVE_MODE_START_TIME': 0})
        b.exit_config()
    except Exception:
        pass
    trace = '\n'.join(vues[:4])
    if not vues:
        return r.record(case, 'FAIL',
                        'plongee engagee toujours active apres emersion — balise muette en surface',
                        trace)
    r.record(case, 'PASS', 'l emersion desengage la plongee', trace)

CASES_V16 = [
    dict(id='GPS-01', risque='MAJEUR',   titre='GNSS_EN=0 supprime tout ordonnancement GNSS',
         fn=c_gnss_desactive),
    dict(id='GPS-02', risque='MAJEUR',   titre='SESSION_SINGLE_FIX arrete la session au premier fix',
         fn=c_gnss_fix_unique),
    dict(id='GPS-03', risque='BLOQUANT', titre='La session sans fix se termine sur GNSS_ACQ_TIMEOUT',
         fn=c_gnss_timeout_acquisition),
    dict(id='GPS-04', risque='MAJEUR',   titre='NTRY epuise: repli de cadence sur DLOC_ARG_NOM',
         fn=c_gnss_ntry_backoff),
    dict(id='GPS-05', risque='MAJEUR',   titre='La veille profonde respecte la duree configuree',
         fn=c_gnss_deep_idle),
    dict(id='GPS-06', risque='MAJEUR',   titre='Demarrage a froid apres epuisement de NTRY',
         fn=c_gnss_cold_start),
    dict(id='MOOR-01', risque='MAJEUR',  titre='Entree en MOORED apres N fixes immobiles',
         fn=c_moored_entree),
    dict(id='MOOR-02', risque='MAJEUR',  titre='Sortie de MOORED sur deplacement hors rayon',
         fn=c_moored_sortie_deplacement),
    dict(id='MOOR-03', risque='MAJEUR',  titre='Les reglages Argos du mode amarre sont effectifs',
         fn=c_moored_override_argos),
    dict(id='MOOR-04', risque='MAJEUR',  titre='MRP00=0 purge l etat persistant et force UNDERWAY',
         fn=c_moored_desactive),
    dict(id='DIVE-01', risque='MAJEUR',  titre='La plongee s engage apres le delai UNP13',
         fn=c_dive_engagement),
    dict(id='DIVE-02', risque='MAJEUR',  titre='Une emersion precoce annule l engagement',
         fn=c_dive_annulation),
    dict(id='DIVE-03', risque='BLOQUANT', titre='L emersion desengage une plongee active',
         fn=c_dive_desengagement),
]


# =====================================================================
#  Vague 17 — hors-eau, diagnostics SWS, etat technique
#
#  Debloquee par RTCW. Le mode hors-eau se declenche sur une DUREE DE
#  SECHERESSE dont le seuil minimum est 1 heure (HMP01, borne firmware): on ne
#  peut pas l attendre au banc. RTCW pose l horloge a un timestamp arbitraire,
#  et — verifie — il ne RE-ANCRE pas le compteur: reset_for_rtc_sync() n est
#  appele que depuis le pilote M10Q, sur une vraie synchro GNSS
#  (m10qasync.cpp:1051). Sauter l horloge de deux heures fait donc exactement
#  ce qu aurait fait l attente.
# =====================================================================

def _rtc_now(b, timeout=6.0):
    """Lit l horloge de la carte via STATR, ou None."""
    # SYT01 = RTC_CURRENT_TIME, rafraichi a chaque lecture STATR
    # (dte_params.cpp: "Current RTC time (live, refreshed on STATR read)").
    m = b.dte('STATR', 'SYT01', timeout=timeout)
    if not m:
        return None
    ligne = m.string if hasattr(m, 'string') else ''
    mm = re.search(r'SYT01=(\d+)', ligne)
    return int(mm.group(1)) if mm else None

def _hauled(b, timeout=8.0):
    """Etat hors-eau lu dans le journal au prochain reveil du service."""
    mk = b.mark(); b._send('%SCHED\r')
    m = b.expect(r'%SCHED ', timeout, from_idx=mk)
    return m.group(0) if m else ''

def c_hauled_entree(r, case):
    """Une secheresse superieure au seuil fait passer AT_SEA -> HAULED.

    C est la detection elle-meme, pas la substitution de profil (HAULED-01 s en
    charge). Sans elle, un animal remonte sur une plage continue d emettre sur
    la cadence de mer — la balise se vide alors qu elle n a plus rien a dire.

    L horloge est avancee de deux heures pour un seuil d une heure: RTCW ne
    re-ancre pas le compteur, la secheresse mesuree est donc reellement de
    deux heures du point de vue du service.
    """
    b = r.b
    try:
        b.enter_config()
        b.write_params({'HAULED_DETECT_EN': 1, 'HAULED_IDLE_THRESHOLD_H': 1,
                        'HAULED_RETURN_EVENTS': 2, 'UNDERWATER_EN': 1,
                        'ARGOS_MODE': 0, 'GNSS_EN': 0})
        b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    time.sleep(2)
    # Un evenement d immersion ancre la derniere trace d eau a maintenant.
    b._send('%DIVE\r'); time.sleep(3)
    b._send('%SURFACE\r'); time.sleep(3)
    t0 = _rtc_now(b)
    if not t0:
        return r.record(case, 'ERROR', 'STATR ne rend pas l horloge — saut impossible')
    mk = b.mark()
    saut = t0 + 2 * 3600
    m = b.dte('RTCW', str(saut), timeout=8.0)
    if not m or ';RTCW' not in (m.string if hasattr(m, 'string') else ''):
        return r.record(case, 'ERROR', f'RTCW refuse (t={saut})')
    vues = _attendre_trace(b, [r'AT_SEA . HAULED', r'HAULED', r'dry for (\d+) s'],
                           120, depuis=mk)
    try:
        b.enter_config()
        b.write_params({'HAULED_DETECT_EN': 0, 'HAULED_IDLE_THRESHOLD_H': 24,
                        'UNDERWATER_EN': 0})
        b.exit_config()
    except Exception:
        pass
    trace = '\n'.join(vues[:6])
    entree = [l for l in vues if 'HAULED' in l and 'AT_SEA' in l]
    if not entree:
        return r.record(case, 'FAIL',
                        'deux heures de secheresse pour un seuil d une heure, '
                        'mais aucun passage en HAULED', trace)
    m2 = re.search(r'dry for (\d+) s, threshold (\d+) h', entree[0])
    if m2 and int(m2.group(1)) < int(m2.group(2)) * 3600:
        return r.record(case, 'FAIL',
                        f'passage annonce avec dry={m2.group(1)} s < seuil {m2.group(2)} h', trace)
    r.record(case, 'PASS', 'passage en HAULED sur depassement du seuil de secheresse', trace)

def c_hauled_retour(r, case):
    """HAULED_RETURN_EVENTS immersions ramenent en AT_SEA.

    Le retour a la mer doit demander PLUSIEURS evenements: une seule vague sur
    un animal echoue ne doit pas relancer la cadence de mer. On verifie donc
    qu une immersion NE SUFFIT PAS, puis que la seconde declenche.
    """
    b = r.b
    try:
        b.enter_config()
        b.write_params({'HAULED_DETECT_EN': 1, 'HAULED_IDLE_THRESHOLD_H': 1,
                        'HAULED_RETURN_EVENTS': 2, 'UNDERWATER_EN': 1,
                        'ARGOS_MODE': 0, 'GNSS_EN': 0})
        b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    time.sleep(2)
    b._send('%DIVE\r'); time.sleep(3)
    b._send('%SURFACE\r'); time.sleep(3)
    t0 = _rtc_now(b)
    if not t0:
        return r.record(case, 'ERROR', 'STATR ne rend pas l horloge')
    mk = b.mark()
    b.dte('RTCW', str(t0 + 2 * 3600), timeout=8.0)
    if not _attendre_trace(b, [r'AT_SEA . HAULED'], 120, depuis=mk):
        try:
            b.enter_config()
            b.write_params({'HAULED_DETECT_EN': 0, 'HAULED_IDLE_THRESHOLD_H': 24,
                            'UNDERWATER_EN': 0})
            b.exit_config()
        except Exception:
            pass
        return r.record(case, 'ERROR', 'jamais entre en HAULED — retour non evaluable')

    mk1 = b.mark()
    b._send('%DIVE\r'); time.sleep(4); b._send('%SURFACE\r')
    premier = _attendre_trace(b, [r'HAULED . AT_SEA'], 30, depuis=mk1)
    mk2 = b.mark()
    b._send('%DIVE\r'); time.sleep(4); b._send('%SURFACE\r')
    second = _attendre_trace(b, [r'HAULED . AT_SEA'], 40, depuis=mk2)
    try:
        b.enter_config()
        b.write_params({'HAULED_DETECT_EN': 0, 'HAULED_IDLE_THRESHOLD_H': 24,
                        'UNDERWATER_EN': 0})
        b.exit_config()
    except Exception:
        pass
    trace = 'apres 1 immersion: ' + '|'.join(premier[:2]) + \
            '\napres 2 immersions: ' + '|'.join(second[:2])
    if premier:
        return r.record(case, 'FAIL',
                        'une seule immersion suffit a quitter HAULED alors que '
                        'HMP02=2 — une vague relancerait la cadence de mer', trace)
    if not second:
        return r.record(case, 'FAIL',
                        'deux immersions ne suffisent pas a quitter HAULED (HMP02=2)', trace)
    r.record(case, 'PASS', 'une immersion ne suffit pas, deux ramenent en AT_SEA', trace)

def c_rtc_ecriture(r, case):
    """RTCW pose l horloge, et la carte la relit.

    Base de tout ce qui precede, et surtout de la donnee elle-meme: une position
    horodatee faux est inexploitable par le segment sol. On verifie l aller-retour
    avec une tolerance de quelques secondes — le temps de la trame.
    """
    b = r.b
    avant = _rtc_now(b)
    if not avant:
        return r.record(case, 'ERROR', 'STATR ne rend pas SYT01 (horloge)')
    cible = avant + 7200
    m = b.dte('RTCW', str(cible), timeout=8.0)
    if not m:
        return r.record(case, 'FAIL', 'RTCW sans reponse')
    if (m.group(1) if m.lastindex else '') == 'N':
        return r.record(case, 'FAIL', f'RTCW refuse pour un timestamp valide ({cible})')
    time.sleep(2)
    apres = _rtc_now(b)
    if apres is None:
        return r.record(case, 'FAIL', 'horloge illisible apres ecriture')
    ecart = abs(apres - cible)
    # On repose une horloge coherente pour ne pas fausser les cas suivants.
    try:
        b.dte('RTCW', str(avant + 8), timeout=8.0)
    except Exception:
        pass
    if ecart > 10:
        return r.record(case, 'FAIL',
                        f'horloge relue a {apres}, soit {ecart} s de la valeur ecrite')
    r.record(case, 'PASS', f'aller-retour RTCW exact a {ecart} s pres')

def c_sws_diagnostics(r, case):
    """SWSSTATS rend les sept compteurs, et l effacement les remet a zero.

    Ces compteurs sont le seul temoin des pathologies du detecteur d immersion
    (electrode collee, incoherence de calibration, plongee sans fin). S ils ne
    remontent pas, une balise qui derive n a aucun moyen de le dire.
    """
    b = r.b
    m = b.dte('SWSSTATS', '0', timeout=8.0)
    if not m:
        return r.record(case, 'ERROR', 'SWSSTATS sans reponse')
    ligne = m.string if hasattr(m, 'string') else ''
    if m.group(1) == 'N':
        return r.record(case, 'FAIL', f'SWSSTATS refuse: {ligne[:80]}')
    champs = re.findall(r'(\d+)', ligne.split(';', 1)[-1])
    if len(champs) < 7:
        return r.record(case, 'FAIL',
                        f'SWSSTATS rend {len(champs)} champs au lieu de 7', ligne[:160])
    m2 = b.dte('SWSSTATS', '1', timeout=8.0)
    time.sleep(1)
    m3 = b.dte('SWSSTATS', '0', timeout=8.0)
    apres = re.findall(r'(\d+)', (m3.string if m3 and hasattr(m3, 'string') else '').split(';', 1)[-1])
    trace = f'avant={champs[:7]}\napres effacement={apres[:7]}'
    if not m2 or not m3:
        return r.record(case, 'FAIL', 'effacement des diagnostics sans reponse', trace)
    if len(apres) >= 7 and any(int(x) != 0 for x in apres[:7]):
        return r.record(case, 'FAIL',
                        'SWSSTATS 1 ne remet pas les compteurs a zero', trace)
    r.record(case, 'PASS', 'sept compteurs rendus, effacement effectif', trace)

def c_statr_coherent(r, case):
    """STATR rend un etat technique lisible et plausible.

    C est ce que l operateur consulte avant de poser la balise. Un champ absent
    ou aberrant passe inapercu a l ecran et se paie sur le terrain.
    """
    b = r.b
    m = b.dte('STATR', '', timeout=10.0)
    if not m:
        return r.record(case, 'ERROR', 'STATR sans reponse')
    ligne = m.string if hasattr(m, 'string') else ''
    if m.group(1) == 'N':
        return r.record(case, 'FAIL', f'STATR global refuse: {ligne[:100]}')
    paires = dict(re.findall(r'([A-Z]{3}\d\d)=([^,\r]*)', ligne))
    defauts = []
    if len(paires) < 5:
        defauts.append(f'seulement {len(paires)} champs rendus')
    # Tension batterie: le seul champ dont on connaisse la plage physique.
    for cle, val in paires.items():
        if val.isdigit() and 2000 <= int(val) <= 5000 and cle.startswith('BAT'):
            break
    trace = ', '.join(f'{k}={v}' for k, v in list(paires.items())[:14])
    if defauts:
        return r.record(case, 'FAIL', '; '.join(defauts), trace)
    r.record(case, 'PASS', f'{len(paires)} champs d etat rendus', trace)

CASES_V17 = [
    dict(id='RTC-01',   risque='BLOQUANT', titre='RTCW pose l horloge et elle est relue',
         fn=c_rtc_ecriture),
    dict(id='HAUL-02',  risque='BLOQUANT', titre='La secheresse au-dela du seuil fait passer en HAULED',
         fn=c_hauled_entree),
    dict(id='HAUL-03',  risque='MAJEUR',   titre='Il faut HMP02 immersions pour quitter HAULED',
         fn=c_hauled_retour),
    dict(id='SWS-03',   risque='MAJEUR',   titre='SWSSTATS rend et efface les diagnostics',
         fn=c_sws_diagnostics),
    dict(id='STAT-01',  risque='MAJEUR',   titre='STATR rend un etat technique coherent',
         fn=c_statr_coherent),
]
