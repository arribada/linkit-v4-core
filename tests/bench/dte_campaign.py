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
            time.sleep(2)
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
            b.write_params({cle: val})
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
