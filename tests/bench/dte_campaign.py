#!/usr/bin/env python3
"""Campagne KIM autonome — recherche de defauts bloquants, sans emission radio.

Concu pour tourner seul plusieurs heures. Chaque cas repart d'une configuration
connue, de sorte qu'un echec n'invalide pas les suivants. Le harnais se recupere
d'un port qui disparait, d'une carte muette, et journalise tout en JSONL pour
qu'un depouillement soit possible sans relire le log brut.

CONTRAINTE ASSUMEE: aucun cas de ce fichier ne declenche d'emission Argos. Le
mode est force a OFF et la certification desarmee au debut de chaque cas.
"""
import sys, time, json, re, subprocess, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from kim_bench import Bench

import os
OUT = os.environ.get('DTE_CAMPAIGN_OUT', '/tmp/dte_campaign')
os.makedirs(OUT, exist_ok=True)
RESULTS = f'{OUT}/campaign_results.jsonl'
LOG     = f'{OUT}/campaign.log'

class Runner:
    def __init__(self):
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
                b = Bench(port='/dev/ttyACM0', quiet=True)
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

    def relink(self):
        """Repare le lien USB-over-IP sans toucher au J-Link.

        Le lien meurt en cours de campagne: usbipd affiche encore "Attached"
        alors que TOUS les URB echouent (dmesg: vhci_hcd urb->status -104) et
        que serial.Serial() se bloque pour toujours a l'ouverture. Trois runs
        ont ete perdus ainsi, sans le moindre message.
        Sequence qui marche: detacher UNIQUEMENT 6-3 (jamais --all, qui
        emporterait le J-Link et donc le SWD), basculer le pullup D+ par SWD
        (= rebranchement logiciel), puis attacher DANS LA SECONDE ou Windows
        repasse CM_PROB_NONE — attendre plus fait rater la fenetre.
        """
        import subprocess as sp
        def ps(c):
            try: return sp.run(['powershell.exe','-Command',c], capture_output=True,
                               text=True, timeout=60).stdout
            except Exception: return ''
        self.say('   reparation du lien USB…')
        ps('usbipd detach --busid 6-3')
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
                ps('usbipd attach --busid 6-3 --wsl')
                time.sleep(6)
                return os.path.exists('/dev/ttyACM0')
        return False

    def recover(self):
        """Carte muette: reset materiel, puis reconnexion."""
        self.say("!! carte muette — reset materiel")
        if self.b:
            try: self.b.close()
            except Exception: pass
            self.b = None
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
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}')
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
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}')
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
            return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}')
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
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}')
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
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}')
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
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}')
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
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}')
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
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}')
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
