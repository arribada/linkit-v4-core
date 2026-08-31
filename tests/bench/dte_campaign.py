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

def _nrfjprog(args, timeout=180):
    """Appelle nrfjprog SANS chemin graphique et SANS stdin.

    JLinkGUIServerExe ouvre une boite de dialogue (deverrouillage, mise a jour
    de la sonde...) sur un affichage qui n existe pas en session non
    interactive, et ATTEND INDEFINIMENT. Le symptome est un timeout avec ZERO
    octet de sortie: ni erreur, ni progression, rien — ce qui ressemble a une
    sonde morte et n en est pas une.

    Mesure du 2026-08-29: quatre --recover bloques d affilee, puis rc=0 du
    premier coup une fois DISPLAY et stdin coupes. Le timeout monte aussi a
    180 s: un --recover legitime depasse regulierement les 60 s d origine.
    """
    import os
    env = {k: v for k, v in os.environ.items()
           if k not in ('DISPLAY', 'WAYLAND_DISPLAY')}
    with open(os.devnull, 'rb') as devnull:
        return subprocess.run(['nrfjprog'] + args, capture_output=True,
                              timeout=timeout, env=env, stdin=devnull)

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
        self.n_pass = self.n_fail = self.n_error = self.n_skip = 0

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
        # ATTENTION: `usbipd detach` peut laisser le peripherique NON PARTAGE,
        # et le re-partager (`usbipd bind`) exige des droits ADMINISTRATEUR.
        # Mesure du 2026-08-28: la sequence detach + bascule D+ a fait
        # disparaitre le CDC de la vue Windows, et plus aucune commande
        # accessible depuis WSL ne pouvait le ramener — le harnais s etait mis
        # dans un etat dont il ne savait pas sortir, en pleine campagne de nuit.
        #
        # On verifie donc que le partage tient AVANT de detacher, et on renonce
        # a la sequence lourde si ce n est pas le cas: mieux vaut echouer en
        # laissant le banc utilisable que le rendre injoignable.
        etat = ps(f'usbipd list')
        if busid not in etat:
            self.say('   busid absent de usbipd — banc injoignable, pas de detach')
            return False
        ps(f'usbipd detach --busid {busid}')
        time.sleep(3)
        for v in ('0', '1'):
            _nrfjprog(['--memwr','0x40027504','--val',v])
            _nrfjprog(['--run'])
            if v == '0': time.sleep(4)
        for _ in range(25):
            time.sleep(1)
            n = ps("(Get-PnpDevice -PresentOnly | Where-Object { $_.InstanceId -like "
                   "'*VID_1915*' -and $_.Problem -eq 'CM_PROB_NONE' }).Count").strip()
            if n.isdigit() and int(n) >= 1:
                ps(f'usbipd attach --busid {busid} --wsl')
                time.sleep(6)
                return bool(glob.glob('/dev/ttyACM*') or glob.glob('/dev/ttyUSB*'))
        # Le peripherique n est jamais repasse CM_PROB_NONE. S il a aussi quitte
        # la liste usbipd, seul un debranchement PHYSIQUE le ramenera: on le dit
        # plutot que de boucler.
        if busid not in ps('usbipd list'):
            self.say('   le peripherique a QUITTE la vue Windows — debranchement '
                     'physique necessaire')
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
            _nrfjprog(['--reset'])
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
        mark = {'PASS':'ok  ', 'FAIL':'ECHEC', 'ERROR':'err ', 'SKIP':'saute'}.get(verdict, '?')
        self.say(f"  {mark} [{case['risque'][:4]}] {case['id']}: {detail}")
        if verdict == 'PASS': self.n_pass += 1
        elif verdict == 'FAIL': self.n_fail += 1
        # SKIP: le cas ne PEUT pas conclure dans les conditions physiques du
        # moment (electrode a sec, pas de ciel, module sans credentials). Ce
        # n est ni un succes ni une panne, et le compter comme une erreur rend
        # le bilan de campagne mensonger dans les deux sens: on croit avoir un
        # defaut, et on croit avoir couvert le cas.
        elif verdict == 'SKIP': self.n_skip += 1
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
                if case.get('ciel'):
                    # Les cas de plein air ne se jouent que sur demande. Les
                    # jouer en interieur ne peut rien prouver: ils attendent
                    # chacun plusieurs minutes une position que quatre murs ne
                    # rendront pas, puis accusent le firmware. Pire, les faire
                    # SAUTER sur "pas de fix" masquerait le jour ou l absence de
                    # fix EST le defaut — dehors, antenne degagee. On les rend
                    # donc volontaires plutot que devinables.
                    if not os.environ.get('DTE_CIEL'):
                        self.record(case, 'SKIP',
                                    'vague de plein air non demandee — relancer '
                                    'antenne vers le ciel avec DTE_CIEL=1')
                        continue
                    if saute_si_gnss_muet(self, case, 'le plein air'):
                        continue
                if case.get('radio') and saute_si_radio_muette(
                        self, case, 'le dialogue avec le module'):
                    continue
                avant = self.n_pass + self.n_fail + self.n_error + self.n_skip
                case['fn'](self, case)
                # Un cas qui sort sans rien enregistrer DISPARAIT de la campagne:
                # le total baisse, aucun rouge n apparait, et personne ne le voit.
                # C est arrive le 2026-08-30 — une aide inseree au milieu de
                # c_sws_etat_coherent avait coupe la fonction avant son verdict, et
                # ses verifications vivaient en code mort apres un return. Le seul
                # indice etait un resultat perime qui survivait d une passe a l autre.
                if self.n_pass + self.n_fail + self.n_error + self.n_skip == avant:
                    self.record(case, 'ERROR',
                                'le cas s est termine SANS rendre de verdict — '
                                'chemin sans record(), a corriger dans le harnais')
            except Exception as e:
                self.record(case, 'ERROR', f'exception {type(e).__name__}: {e}')
                if not self.recover():
                    self.say("ABANDON: recuperation impossible"); return
        saute = f", {self.n_skip} sautes" if self.n_skip else ""
        self.say(f"=== fin — {self.n_pass} ok, {self.n_fail} echecs, {self.n_error} erreurs{saute} ===")


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

def _commandes_absentes(b):
    """Quelles commandes DTE sont absentes de la table sur CE build.

    L absence depend de la cible, pas d une verite generale: SMDTST n existe
    que sur ARGOS_SMD, KIMBR que sur un build ni-SMD-ni-LoRa, LORATX/LORABR que
    sur LORA_RAK3172. Tester l absence de SMDTST sur une carte SMD revient a
    tester l absence d une commande presente — mesure du 2026-08-29, ou le cas
    a interroge SMDTST quatre fois de suite sur un build SMD, chaque appel
    prenant 28 s cote firmware.

    On demande au build ce qu il porte plutot que de le supposer: ARP60
    (SMD_LPM_MODE) n existe que sur SMD, LRP07 (LORA_NJM) que sur LoRa.
    """
    clefs = cles_implementees(b)
    if not clefs:
        return [('SMDTST', '000;'), ('LORATX', '001;5'), ('LORABR', '001;1')]
    est_smd = 'ARP60' in clefs
    est_lora = 'LRP07' in clefs
    absentes = []
    if not est_smd:
        absentes.append(('SMDTST', '000;'))
    if not est_lora:
        absentes += [('LORATX', '001;5'), ('LORABR', '001;1')]
    if est_smd or est_lora:
        absentes.append(('KIMBR', '000;'))
    return absentes


def c_cmd_absente_du_build(r, case):
    """Une commande absente de la table de CE build ne doit pas etre aiguillee.

    Le risque n est pas le silence: c est qu une commande inconnue soit
    dispatchee sur le gestionnaire d une AUTRE, et reponde sous son nom. Mesure
    2026-08-26: onze reponses sur douze portaient le nom d une autre commande.
    """
    b = r.b
    paires = _commandes_absentes(b)
    if not paires:
        return r.record(case, 'SKIP',
                        'ce build porte toutes les commandes conditionnelles — rien a '
                        'eprouver ici')
    vus, envois_rates = [], 0
    for nom, payload in paires:
        for k in range(4):
            lines, err = r.raw(f"${nom}#{payload}\r", wait=1.5)
            if lines is None:
                # raw() rend (None, message) quand l envoi echoue; iterer dessus
                # levait un TypeError et faisait passer un incident de lien pour
                # un defaut de firmware.
                envois_rates += 1
                continue
            got = None
            for l in lines:
                mm = re.search(r'\$([ON]);([A-Z]+)#', l)
                if mm: got = mm.group(2); break
            vus.append((nom, got))
    mauvais = [v for v in vus if v[1] is not None and v[1] != v[0]]
    silences = [v for v in vus if v[1] is None]
    quoi = ', '.join(n for n, _ in paires)
    if not vus:
        return r.record(case, 'ERROR',
                        f'{envois_rates} envoi(s) impossible(s), rien observe — cas non concluant')
    if mauvais:
        r.record(case, 'FAIL',
                 f'la reponse porte le nom d\'une AUTRE commande — {len(mauvais)}/{len(vus)} cas',
                 json.dumps(mauvais, ensure_ascii=False))
    elif len(silences) == len(vus):
        r.record(case, 'PASS', f'silence uniforme sur {quoi}, aucun aiguillage errone')
    else:
        r.record(case, 'PASS',
                 f'reponses coherentes sur {quoi} ({len(silences)} silences sur {len(vus)})')

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
    try: _nrfjprog(['--reset'])
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
        try: _nrfjprog(['--reset'])
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
    if saute_si_radio_muette(r, case, 'le dialogue avec le module'):
        return
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
    if saute_si_radio_muette(r, case, 'le dialogue avec le module'):
        return
    b = r.b
    # KIMBR n est compile que sur un build ni-SMD-ni-LoRa. Ailleurs son absence
    # est le comportement attendu, et le silence qu on obtient n est pas un
    # refus d ouverture — mesure du 2026-08-30 sur carte SMD, ou le cas
    # concluait "ouverture du pont refusee: <silence>" sur un firmware qui ne
    # porte simplement pas la commande. Le pont equivalent y est LORABR, et le
    # pont GNSS (BRDG-01) existe partout.
    if any(nom == 'KIMBR' for nom, _ in _commandes_absentes(b)):
        return r.record(case, 'SKIP',
                        'ce build ne porte pas KIMBR (SMD ou LoRa) — le pont KIM2 n existe '
                        'pas ici, son absence est correcte')
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
    # module_repond etait CALCULE puis jamais assertionne: un pont qui avale les
    # commandes sans que le module reponde passait pour bon. Or un pont qui ne
    # transporte rien n a aucun interet — c est precisement la panne qu on veut
    # detecter avant d envoyer quelqu un diagnostiquer une balise sur le terrain.
    defauts = []
    if not avalee:
        defauts.append("la commande d'arret DTE est EXECUTEE au lieu d'etre transmise au module")
    if not module_repond:
        defauts.append('le module ne repond pas a AT+FW=? a travers le pont')
    if defauts:
        return r.record(case, 'FAIL', '; '.join(defauts), '\n'.join(l3[:6]))
    r.record(case, 'PASS',
             "pont ouvert, commande DTE transmise, module repond a AT+FW=?, canal rendu par +++",
             '\n'.join(l3[:4]))


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
    if saute_si_radio_muette(r, case, 'le dialogue avec le module'):
        return
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
            mm = re.search(r'ARGOSTX=(none|hold\d+s|\d+ms)\(([^)]*)\)', ligne)
            if mm:
                quand = mm.group(1)
                dernier = (int(quand[:-2]) if _est_planifie(quand) else None, mm.group(2))
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
        # Libelles EXACTS du firmware (argos_tx_service.cpp). Les deux chaines
        # cherchees auparavant — 'prepass indisponible' et 'returned no pass' —
        # n existent nulle part, donc sans_aop etait TOUJOURS faux et le filet
        # qui excuse PASS_PREDICTION sans AOP ne s est jamais declenche.
        sans_aop = any('no pass is computable' in l
                       or 'prepass impossible' in l
                       or 'prepass unavailable until the first fix' in l
                       for l in jr)
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

def _est_planifie(quand):
    """Vrai si le service a une ECHEANCE D EXECUTION.

    Trois formes depuis la migration ScheduleDecision (2026-08):
      <n>ms   -- il va s executer dans n ms
      hold<n>s -- il attend quelque chose, et se re-interrogera dans n s
      none    -- decision finale, il ne repassera pas de lui-meme

    Seule la premiere est une emission a venir. Tester la RAISON exacte, comme
    le faisait ce fichier, ne marche plus: chaque service nomme desormais sa
    propre cause ('TR_NOM period', 'cold-start retry', 'depth pile empty'...)
    la ou tous rendaient 'scheduled' ou 'no-schedule'.
    """
    return bool(quand) and quand.endswith('ms')

def _sched_tous(r, timeout=10.0):
    """Rend {service: (ms|None, raison)} pour TOUS les services.

    Un hold rend ms=None comme un none: dans les deux cas rien n est prevu a
    l execution. La raison distingue les deux, et le prefixe hold<n>s dit en
    plus au bout de combien de temps le service se reveillera tout seul.
    """
    m, _ = r.raw_until('%SCHED\r', r'%SCHED .*ARGOSTX=', timeout=timeout)
    if not m: return {}
    return {nom: (int(val[:-2]) if _est_planifie(val) else None, why)
            for nom, val, why in re.findall(r'(\w+)=(none|hold\d+s|\d+ms)\(([^)]*)\)', m.string)}

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

def _attendre_planifie(r, service, timeout=40.0):
    """Sonde %SCHED jusqu'a ce que `service` ait une echeance d execution.

    Remplace l attente sur ('scheduled', 'already-initiated'): 'scheduled' n est
    plus rendu que par les services non migres et par le chemin immediat, si
    bien qu exiger cette chaine ferait echouer un firmware qui planifie
    parfaitement — le pire faux positif possible, celui qui dit muette une
    balise qui emet.
    """
    fin = time.time() + timeout; vu = None
    while time.time() < fin:
        d = _sched_tous(r)
        if service in d:
            vu = d[service]
            if vu[0] is not None: return vu
        time.sleep(2)
    return vu

def _attendre_sans_echeance(r, service, timeout=40.0):
    """Sonde %SCHED jusqu'a ce que `service` n ait plus AUCUNE echeance."""
    fin = time.time() + timeout; vu = None
    while time.time() < fin:
        d = _sched_tous(r)
        if service in d:
            vu = d[service]
            if vu[0] is None: return vu
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
    # Le critere est structurel, pas lexical: plus AUCUNE echeance d execution.
    # La raison exacte varie ('underwater', 'descheduled', ou la cause propre du
    # service depuis la migration ScheduleDecision) et n a jamais ete la preuve.
    ecarts = []
    b._send('%DIVE\r')
    for svc in ('ARGOSTX', 'GNSS'):
        vu = _attendre_sans_echeance(r, svc)
        if not vu or vu[0] is not None:
            ecarts.append(f'{svc} apres %DIVE: {vu} (attendu: aucune echeance)')
    b._send('%SURFACE\r')
    vu = _attendre_planifie(r, 'ARGOSTX')
    if not vu or vu[0] is None:
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
        b._send('%SURFACE\r'); vu = _attendre_planifie(r, 'ARGOSTX', 30)
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
    vu = _attendre_planifie(r, 'ARGOSTX', 45)
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
    # Comparer le DERNIER echantillon au premier revient a conclure une tendance
    # a partir de deux points, sur une grandeur qui fluctue: d autres services
    # programment et retirent des taches pendant la mesure. Mesure du 2026-08-30
    # sur la carte SMD: depart=7 puis [6, 8, 6, 8, 8] — la file oscille entre 6
    # et 8, et l ancien critere y voyait "+1 de croissance".
    #
    # Le bon critere vient de la nature de la fuite: une tache non annulee ne
    # repart JAMAIS, donc elle releve le PLANCHER de la file. Sur le binaire sans
    # correctif (17 -> [18,19,20,21,22]) le plancher passe de 17 a 18 et le defaut
    # est vu; ici le plancher descend a 6, donc rien ne fuit.
    plancher = min(suite)
    croissance = plancher - base[1]
    monotone = all(suite[i] <= suite[i + 1] for i in range(len(suite) - 1))
    trace = f'depart={base[1]} puis {suite} (plancher={plancher})'
    if croissance > 0 or (monotone and suite[-1] - base[1] >= 2):
        r.record(case, 'FAIL',
                 f'la file differee ne redescend plus sous {plancher} (depart {base[1]}) '
                 f'en 5 aller-retours — tache non annulee',
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
    if saute_si_capacite_absente(r, case, 'AXP01', 'le bit AXL dans la charge utile'):
        return
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
    if saute_si_radio_muette(r, case, 'une emission reussie'):
        return
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
    mk = b.mark()
    b._send('%GPS 43.6 3.9 5000 9\r')
    # On n echantillonne QU APRES que le quota soit consomme. Avant, l emission
    # est legitimement immediate — un fix vient de tomber — et un releve a 0 ms
    # n a rien a voir avec un debordement.
    #
    # Mesure du 2026-08-28: [0, 4294967294, 4294967294, 4294967294, 4294967294,
    # 4294967294]. Les cinq dernieres valeurs sont exactement MAX_RESCHEDULE_MS,
    # donc la borne firmware tient; c est le premier releve, pris avant blocage,
    # qui faisait echouer le cas. Il passait au rejeu isole simplement parce
    # qu un lien lent decalait ce premier releve apres la consommation du quota.
    bloque = _attendre_trace(b, [r'rate limit reached'], 90, depuis=mk)
    vus = []
    for _ in range(5):
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
    if not bloque:
        return r.record(case, 'ERROR',
                        'le limiteur n annonce jamais son blocage en 90 s — cas non concluant',
                        f'delais observes: {vus} ms')
    if not vus:
        return r.record(case, 'ERROR', 'aucune planification ARGOSTX observee apres blocage')
    mini = min(vus)
    trace = f'blocage annonce, delais ensuite: {vus} ms'
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
        # Le libelle EXACT du firmware (argos_tx_scheduler.cpp): "duty-cycle
        # mask is 0 — no hour is permitted". La version precedente de ce cas
        # cherchait "DUTY_CYCLE mask is 0x000000", qui n existe nulle part —
        # elle echouait donc sur un firmware parfaitement sain, mesure du
        # 2026-08-29. "no TX slot found in 24h search" ne peut pas se produire
        # non plus: le repli sur 24 h l empeche justement.
        avert = [l.strip()[24:170] for l in jr
                 if 'duty-cycle mask is 0' in l]
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
    # Le contrat est DOUBLE: signaler, et continuer d emettre. Le firmware
    # bascule sur les 24 h justement pour qu une balise scellee ne se taise pas
    # sur une erreur de configuration — verifier l avertissement sans verifier
    # l emission laisserait passer un repli casse.
    elif not (s_nul and s_nul[0] is not None):
        r.record(case, 'FAIL',
                 'masque nul signale, mais AUCUNE emission planifiee: le repli sur les '
                 '24 h ne fonctionne pas', trace)
    elif not (s_plein and s_plein[0] is not None):
        r.record(case, 'FAIL', 'masque plein: aucune emission planifiee', trace)
    else:
        r.record(case, 'PASS',
                 'masque nul signale explicitement ET repli sur les 24 h (la balise emet '
                 'quand meme), masque plein planifie', trace)

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
    if saute_si_capacite_absente(r, case, 'AXP01', 'la lecture des capteurs par SENSR'):
        return
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
    if saute_si_capacite_absente(r, case, 'AXP01', 'la plage de mesure de l accelerometre'):
        return
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
    if saute_si_capacite_absente(r, case, 'AXP01', 'la calibration de l accelerometre'):
        return
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
    if saute_si_capacite_absente(r, case, 'AXP01', 'le mode basse consommation de l accelerometre'):
        return
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
    if saute_si_radio_muette(r, case, 'le dialogue avec le module'):
        return
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
    # Depend de la radio, contrairement a ce qu une classification automatique
    # avait conclu. La promotion n est evaluee QUE lorsque le service se
    # reprogramme (schedule_surfacing_burst, argos_tx_service.cpp:602). Sur une
    # carte dont le module ne repond jamais, le service est gare sur un maintien
    # d erreur peripherique pouvant aller jusqu a l heure, donc rien ne le
    # rappelle dans la fenetre de 120 s et aucune ligne de promotion ne sort.
    # Preuve par l histoire: six PASS sur la carte KIM2 (module vivant) des 27
    # au 29 aout, un seul echec — sur la carte SMD au module muet.
    if saute_si_radio_muette(r, case, 'la promotion en phase GNSS'):
        return
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
    if saute_si_radio_muette(r, case, 'le dialogue avec le module'):
        return
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
    tester. Rendre ce cas concluant demande une injecting mesure batterie
    (sonde %BATT <mv>), qui n existe pas encore.
    """
    if saute_si_batterie_trop_pleine(r, case, 'la priorite de LOW_BATTERY sur le hors-zone'):
        return
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
    if saute_si_radio_muette(r, case, 'le dialogue avec le module'):
        return
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

    # Ce que le contrat promet, c est que l instant planifie tombe DANS une
    # heure autorisee — pas qu il tombe vite. L ancienne version exigeait moins
    # de 10 minutes, ce qui est une hypothese de promptitude etrangere au
    # mappage bit/heure que ce cas existe pour prouver: mesure du 2026-08-29,
    # 26 minutes apres 10h30 font 10h56, soit toujours l heure 10, et le cas
    # rougissait sur un firmware qui respectait exactement la consigne. Le
    # delai depend de LAST_TX, que la contrainte de plus-tot-possible reporte,
    # et LAST_TX ne se rafraichit pas sur une carte dont la radio ne repond pas.
    heure_planifiee = ((base + dedans[0] // 1000) % 86400) // 3600
    if heure_planifiee != heure:
        r.record(case, 'FAIL',
                 f'heure {heure} autorisee mais emission planifiee dans l heure '
                 f'{heure_planifiee} (dans {dedans[0]} ms)', trace)
    elif not (0.5 * attendu_ms <= dehors[0] <= 1.6 * attendu_ms):
        r.record(case, 'FAIL',
                 f'heure+2: delai {dehors[0]} ms, attendu ~{attendu_ms} ms — mappage bit/heure suspect',
                 trace)
    else:
        r.record(case, 'PASS',
                 f'bit 23 = heure 0 UTC: mappage confirme sur deux heures (planifie dans '
                 f'l heure {heure_planifiee}, puis a +90 min)', trace)

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
    if saute_si_radio_muette(r, case, 'une emission reussie'):
        return
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

def charge_batterie(b):
    """Charge en % lue par STATR POT03, ou None si illisible."""
    try:
        b.enter_config()
        cle = 'POT03'
        mk = b.mark(); b._send(f'$STATR#{len(cle):03X};{cle}\r')
        m = b.expect(r'\$([ON]);STATR#[0-9A-Fa-f]{3};(.*)$', 12.0, from_idx=mk)
        b.exit_config()
    except Exception:
        try: b.exit_config()
        except Exception: pass
        return None
    if not m:
        return None
    for morceau in m.group(2).rstrip('\r').split(','):
        if morceau.startswith('POT03='):
            try:
                return float(morceau.split('=', 1)[1])
            except ValueError:
                return None
    return None


# Marge minimale sous 100 % pour qu un seuil batterie soit posable au-dessus de
# la charge. LB_THRESHOLD est borne a 100 et le drapeau se leve sur
# charge < seuil, STRICTEMENT: a 99 % le seuil vaut 100 et une derive d un seul
# point entre la lecture et l evaluation suffit a faire echouer un firmware
# sain. Mesure du 2026-08-30 sur la carte SMD alimentee par USB.
MARGE_SEUIL_BATTERIE = 94


def saute_si_batterie_trop_pleine(r, case, quoi):
    """SKIP motive si la charge ne laisse pas de place a un seuil au-dessus.

    Ne saute pas quand la charge est illisible: l appelant doit alors conclure
    lui-meme, car une mesure absente n est pas une batterie pleine.
    """
    soc = charge_batterie(r.b)
    if soc is None or soc <= MARGE_SEUIL_BATTERIE:
        return False
    r.record(case, 'SKIP',
             f'charge a {soc:.0f} % — au-dessus de {MARGE_SEUIL_BATTERIE} % aucun seuil ne se '
             f'pose franchement sous 100 %, donc {quoi} n est pas eprouvable. '
             'Ce cas demande une carte sur batterie partiellement dechargee.')
    return True


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
    # Mesurer AVANT de choisir le seuil. Le drapeau se leve sur SOC < seuil, et
    # LB_THRESHOLD est borne a 100: sur une carte alimentee par USB, batterie
    # pleine, AUCUN seuil ne peut depasser la charge et le cas est
    # inconcluable. L ancienne version codait 99 en dur et concluait "les
    # seuils sont figes au boot" alors que 100 < 99 est simplement faux —
    # mesure du 2026-08-29, sur un firmware qui appliquait correctement la
    # regle.
    try:
        b.enter_config()
        p = 'POT03'
        mk = b.mark(); b._send(f'$STATR#{len(p):03X};{p}\r')
        m = b.expect(r'\$([ON]);STATR#[0-9A-Fa-f]{3};(.*)$', 12.0, from_idx=mk)
        b.exit_config()
    except Exception as e:
        try: b.exit_config()
        except Exception: pass
        return r.record(case, 'ERROR', f'lecture de la charge impossible: {type(e).__name__}: {e}')
    soc = None
    if m:
        for morceau in m.group(2).rstrip('\r').split(','):
            if morceau.startswith('POT03='):
                try: soc = float(morceau.split('=', 1)[1])
                except ValueError: pass
    if soc is None:
        return r.record(case, 'ERROR', 'charge (POT03) illisible — cas non concluant')
    if soc > MARGE_SEUIL_BATTERIE:
        return r.record(case, 'SKIP',
                        f'charge a {soc:.0f} %: au-dessus de {MARGE_SEUIL_BATTERIE} % le seuil '
                        'haut est plafonne a 100 et le drapeau ne monte que sur une charge '
                        'STRICTEMENT inferieure — une derive d un point suffit a faire echouer '
                        'un firmware sain. Ce cas demande une batterie partiellement dechargee.')
    seuil_haut = min(100, int(soc) + 5)
    seuil_bas = max(0, int(soc) - 5)

    try:
        b.enter_config()
        _, av = b.read_params(['LB_THRESHOLD', 'LB_CRITICAL_THRESH'])
        b.write_params({'ARGOS_MODE': 2, 'TR_NOM': 60, 'GNSS_EN': 1, 'LB_EN': 1,
                        'LB_THRESHOLD': seuil_haut, 'LB_CRITICAL_THRESH': 1,
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
        b.enter_config(); b.write_params({'LB_THRESHOLD': seuil_bas}); b.exit_config()
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
    trace = f'seuil {seuil_haut}: {haut}\nseuil {seuil_bas}: {bas}'
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

def _led_calme(b, fenetre=3.0, essais=30):
    """Attend que la FSM LED cesse de bouger d elle-meme.

    GNSS_EN=0 empeche la PROCHAINE session GNSS, pas celle qui tourne deja.
    Dehors, une acquisition en vol continue de pousser ses propres evenements
    (GNSSOn en particulier) pendant que le cas injecte sa sequence, et elle
    ecrase l etat teste — un firmware correct echoue alors sur un artefact de
    banc. On attend donc un etat stable avant d injecter quoi que ce soit.

    Le budget (30 tentatives de 3 s) depasse volontairement GNSS_ACQ_TIMEOUT
    (120 s par defaut): une session deja lancee doit pouvoir aller au bout de
    son echeance pendant qu on attend, sinon on renonce juste avant qu elle se
    taise.

    Renvoie l etat stable, ou None si la FSM n a pas cesse de bouger.
    """
    for _ in range(essais):
        depart = _led(b)[0]
        if depart is None:
            return None
        t0 = time.time()
        stable = True
        while time.time() - t0 < fenetre:
            time.sleep(0.4)
            if _led(b)[0] != depart:
                stable = False
                break
        if stable:
            return depart
    return None


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
        # GNSS_EN=0 est indispensable, pas une precaution: ces cas injectent une
        # SEQUENCE d evenements LED et verifient qui ecrase qui. Dehors, une
        # acquisition reelle pousse ses propres evenements au milieu et fait
        # echouer un firmware correct — mesure du 2026-08-29, l etat devenait
        # GNSSOn en pleine fenetre. En interieur le recepteur ne trouvait rien
        # et le cas passait par chance.
        b.enter_config()
        b.write_params({'LED_MODE': 3, 'ARGOS_MODE': 0, 'GNSS_EN': 0})   # 3 = ALWAYS
        b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    time.sleep(2)
    if _led(b)[0] is None:
        return r.record(case, 'ERROR', '%LED sans reponse (sonde absente du build ?)')
    if _led_calme(b) is None:
        return r.record(case, 'ERROR',
                        'la FSM LED bouge encore toute seule (session GNSS en vol ?) — '
                        'sequence non injectable, cas non concluant')

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

    COMMENT l alternance est reellement verifiee: en injectant les deux
    positions AVANT toute emission, donc a credits EGAUX. L invariant devient
    alors simple et solide — les deux compteurs ne doivent jamais differer de
    plus de 1. Un firmware qui viderait N x A puis N x B creuserait un ecart de
    N-1 et se ferait prendre.

    Cet invariant tient pour les DEUX modulations, qui alternent differemment:
      - LDA2 (max_messages=3): le slot 0 porte les deux entrees, elles sont
        decrementees ENSEMBLE et partent dans un seul paquet LONG (ecart 0);
      - LDK (max_messages=1): slot 0 = la plus recente, slot 1 = la plus
        ancienne, slots 2-3 vides — l alternance se fait entre slots (ecart 1).

    NOTE — ce cas a d abord ete ecrit pour demasquer un defaut suppose:
    retrieve_gps() decremente burst_counter sur toutes les entrees rendues AVANT
    que la salve ne decide laquelle encoder, donc un fastloc plus recent devait
    faire perdre un tour au fix. La MESURE l a refute. Le cas reste comme garde.

    Et une version anterieure de ce cas AFFIRMAIT tester l alternance sans la
    verifier: ses assertions ne portaient que sur "les credits descendent sans
    remonter et finissent a zero", que N x A puis N x B satisfait tout aussi
    bien. L injection se faisait aussi emission ACTIVE, donc la premiere entree
    avait deja perdu du credit avant la seconde — a credits inegaux, aucun
    ecart n est concluant.
    """
    if saute_si_radio_muette(r, case, 'une emission reussie'):
        return
    b = r.b
    N = 2
    try:
        b.enter_config()
        # ARGOS_MODE=0 pendant l injection: les deux entrees doivent entrer dans
        # la pile a credits EGAUX, sinon l ecart entre compteurs ne prouve rien.
        b.write_params({'ARGOS_MODE': 0, 'GNSS_EN': 1, 'NTRY_PER_MESSAGE': N,
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
    b._send('%GPS 43.600000 3.900000 5000 9\r'); time.sleep(6)
    b._send('%GPS 44.000000 4.000000 5000 9\r'); time.sleep(4)

    depart_egal = _pile(b)
    if not depart_egal or len(depart_egal) < 2:
        return r.record(case, 'ERROR',
                        f'moins de deux entrees apres injection: {depart_egal}')
    if len({c for _, c in depart_egal}) != 1:
        return r.record(case, 'ERROR',
                        f'les entrees ne partent pas a credits egaux ({depart_egal}) — '
                        'l ecart ne prouverait rien')

    # Maintenant seulement, on ouvre l emission.
    try:
        b.enter_config(); b.write_params({'ARGOS_MODE': 2}); b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'passage en emission impossible: {type(e).__name__}: {e}')

    # on echantillonne toute la descente des credits
    suite = [depart_egal]
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

    # L ALTERNANCE elle-meme: partis a credits egaux, les compteurs ne doivent
    # jamais s ecarter de plus de 1. C est ce qui distingue A B A B de A A B B,
    # et c est ce que l ancienne version du cas ne verifiait pas.
    for etat in suite:
        vivants = [c for _, c in etat if c > 0]
        if len(vivants) > 1 and (max(vivants) - min(vivants)) > 1:
            defauts.append(f'ecart de credit > 1 entre entrees actives: {etat} — '
                           'une entree est videe avant de passer a la suivante')

    if defauts:
        r.record(case, 'FAIL', f'{len(defauts)} anomalie(s) de rotation', trace + '\n' + '\n'.join(defauts))
    else:
        r.record(case, 'PASS',
                 f'les {len(depart)} entrees partent a credits egaux et descendent en '
                 f'ALTERNANCE (ecart <= 1) jusqu a zero, sans saut ni remontee',
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

    GNSS_NTRY et DLOC_ARG_NOM sont poses ICI, meme quand le cas ne s y interesse
    pas. baseline() ne remet que le mode Argos, donc tout le reste est HERITE du
    cas precedent — et GPS-04 epuise NTRY deliberement. Le 2026-08-30 une coupure
    USB a interrompu GPS-04 avant son nettoyage: GPS-05 a herite d un service en
    repli, n a vu aucune session, et a conclu que la veille profonde ne
    fonctionnait pas. Cinq PASS les jours precedents disaient le contraire.
    Une aide de preconditions pose TOUT ce dont le cas depend, pas seulement ce
    qui l interesse.
    """
    cfg = {'ARGOS_MODE': 0, 'GNSS_EN': 1, 'UNDERWATER_EN': 0,
           'MOORED_DETECT_EN': 0, 'HAULED_DETECT_EN': 0,
           'UW_DIVE_MODE_ENABLE': 0, 'LB_EN': 0, 'RATE_LIMIT_EN': 0,
           'ZONE_ENABLE_OUT_OF_ZONE_DETECTION_MODE': 0,
           'GNSS_NTRY': 0, 'DLOC_ARG_NOM': 13}
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
    mm = re.search(r'GNSS=(none|hold\d+s|\d+ms)\(([^)]*)\)', ligne)
    try:
        b.enter_config(); b.write_params({'GNSS_EN': 1}); b.exit_config()
    except Exception:
        pass
    if not mm:
        return r.record(case, 'ERROR', '%SCHED ne rapporte pas le service GNSS', ligne[:200])
    quand, raison = mm.group(1), mm.group(2)
    if _est_planifie(quand):
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

def _deep_idle_dispatch(r, b, valeur, secondes=70):
    """Configure GNP52, injecte un fix, et rend la ligne de disposition observee.

    Les trois regimes emettent trois lignes DIFFERENTES, et c est la seule facon
    de savoir laquelle la balise a choisie:
      GNP52=0           -> "disabled — power_off"      (coupure immediate)
      GNP52=0xFFFFFFFF  -> "never-poweroff (rail stays on"  (sentinelle)
      sinon             -> "deep-idle for <n> s"
    """
    _gnss_base(b, GNSS_DEEP_IDLE_AFTER_OFF_S=valeur)
    time.sleep(3)
    mk = b.mark()
    b.inject_gps(43.2, 5.8)
    return _attendre_trace(b, [r'deep-idle for (\d+) s',
                               r'never-poweroff \(rail stays on',
                               r'disabled — power_off'],
                           secondes, depuis=mk)


def _deep_idle_restaure(b):
    try:
        b.enter_config(); b.write_params({'GNSS_DEEP_IDLE_AFTER_OFF_S': 0}); b.exit_config()
    except Exception:
        pass


def c_deep_idle_zero(r, case):
    """GNP52=0: coupure immediate du rail, et une session suivante repart.

    C est le DEFAUT de toutes les balises, et il n avait aucun cas. Le risque
    n est pas la coupure elle-meme mais ce qui suit: si le rail est coupe sans
    que l ordonnanceur ne soit relance, l acquisition s arrete pour de bon. Le
    cas verifie donc les DEUX: la coupure, puis la session suivante.
    """
    b = r.b
    try:
        vues = _deep_idle_dispatch(r, b, 0, secondes=60)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    coupure = any('disabled — power_off' in l for l in vues)
    if not coupure:
        _deep_idle_restaure(b)
        return r.record(case, 'FAIL', 'GNP52=0 mais aucune coupure immediate annoncee',
                        '\n'.join(vues[:5]))
    # la session suivante doit repartir: c est la moitie du cas
    mk = b.mark()
    suite = _attendre_trace(b, [r'M10Q on'], 400, depuis=mk)
    _deep_idle_restaure(b)
    if not suite:
        return r.record(case, 'FAIL',
                        'coupure immediate correcte, mais AUCUNE session GNSS ensuite en 400 s — '
                        'acquisition arretee pour de bon', '\n'.join(vues[:5]))
    r.record(case, 'PASS', 'coupure immediate, et la session suivante repart',
             '\n'.join((vues + suite)[:6]))


def c_deep_idle_longue(r, case):
    """GNP52=600: une fenetre longue est annoncee a sa vraie valeur.

    45 s (GPS-05) tient dans une seule fenetre d ordonnancement; 600 s n y tient
    pas. Si la duree annoncee etait tronquee ou arrondie quelque part, seule une
    valeur longue le montrerait.
    """
    b = r.b
    try:
        vues = _deep_idle_dispatch(r, b, 600)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    _deep_idle_restaure(b)
    duree = None
    for l in vues:
        m = re.search(r'deep-idle for (\d+) s', l)
        if m:
            duree = int(m.group(1)); break
    if duree is None:
        return r.record(case, 'FAIL', 'GNP52=600 mais aucune veille profonde annoncee',
                        '\n'.join(vues[:5]))
    if duree != 600:
        return r.record(case, 'FAIL', f'veille profonde de {duree} s au lieu des 600 s configurees',
                        '\n'.join(vues[:5]))
    r.record(case, 'PASS', 'veille profonde de 600 s, conforme a GNP52', '\n'.join(vues[:5]))


def c_deep_idle_sentinelle(r, case):
    """GNP52=0xFFFFFFFF: le rail reste allume, et la balise le DIT.

    Le regime le plus risque des trois, et jamais eprouve. Son propre commentaire
    dans gps_service.cpp raconte qu une version precedente arretait DEFINITIVEMENT
    l acquisition dans ce mode, faute de tache de reouverture — le rail restait
    allume et plus rien ne revenait. Le cas suivant (c_deep_idle_sentinelle_sans_uw)
    eprouve precisement cette configuration-la.
    """
    b = r.b
    try:
        vues = _deep_idle_dispatch(r, b, 0xFFFFFFFF)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    _deep_idle_restaure(b)
    if any('never-poweroff (rail stays on' in l for l in vues):
        return r.record(case, 'PASS', 'sentinelle reconnue: rail maintenu, M10Q en PMREQ-backup',
                        '\n'.join(vues[:5]))
    if any('deep-idle for' in l for l in vues):
        return r.record(case, 'FAIL',
                        'la sentinelle est traitee comme une duree ordinaire — le rail sera coupe',
                        '\n'.join(vues[:5]))
    r.record(case, 'FAIL', 'GNP52=sentinelle mais aucune disposition annoncee',
             '\n'.join(vues[:5]))


def c_deep_idle_sentinelle_sans_uw(r, case):
    """Sentinelle SANS detection d immersion: l acquisition doit continuer.

    C est la regression que le code decrit noir sur blanc: "sur toute
    configuration sans source d evenement pair (UNDERWATER_EN=0, c est-a-dire un
    simple traceur periodique) l acquisition GNSS s arretait definitivement".
    C est exactement le profil drifter et le profil fix_beacon — et celui de
    Chypre. Le cas ne se contente donc pas de lire la ligne de disposition: il
    attend la SESSION SUIVANTE.
    """
    b = r.b
    try:
        vues = _deep_idle_dispatch(r, b, 0xFFFFFFFF)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    if not any('never-poweroff (rail stays on' in l for l in vues):
        _deep_idle_restaure(b)
        return r.record(case, 'ERROR', 'la sentinelle n a pas ete engagee — cas non concluant',
                        '\n'.join(vues[:5]))
    mk = b.mark()
    # _gnss_base pose DLOC_ARG_NOM=13 (5 min): une session doit revenir bien avant 500 s.
    suite = _attendre_trace(b, [r'M10Q on', r'sentinel deep-idle — re-opening scheduler'],
                            500, depuis=mk)
    _deep_idle_restaure(b)
    if not suite:
        return r.record(case, 'FAIL',
                        'sentinelle engagee avec UNDERWATER_EN=0 et AUCUNE reprise en 500 s — '
                        'l acquisition est arretee pour de bon (regression connue)',
                        '\n'.join(vues[:5]))
    r.record(case, 'PASS', 'sentinelle engagee, et l ordonnanceur rouvre bien sans evenement pair',
             '\n'.join(suite[:5]))


def c_deep_idle_borne(r, case):
    """Une valeur au-dela du plafond de 24 h doit etre ramenee au plafond.

    DEEP_IDLE_HARD_CAP_S vaut 24 h et borne auto_off_s. Une valeur de 48 h ne
    doit donc pas produire une fenetre de 48 h — sinon une balise mal configuree
    disparait deux jours au lieu d un.
    """
    b = r.b
    deux_jours = 48 * 3600
    try:
        vues = _deep_idle_dispatch(r, b, deux_jours)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    _deep_idle_restaure(b)
    duree = None
    for l in vues:
        m = re.search(r'deep-idle for (\d+) s', l)
        if m:
            duree = int(m.group(1)); break
    if duree is None:
        return r.record(case, 'FAIL', 'GNP52=48 h mais aucune veille profonde annoncee',
                        '\n'.join(vues[:5]))
    if duree > 24 * 3600:
        return r.record(case, 'FAIL',
                        f'fenetre de {duree} s — le plafond de 24 h n est pas applique',
                        '\n'.join(vues[:5]))
    r.record(case, 'PASS', f'fenetre ramenee a {duree} s, plafond de 24 h applique',
             '\n'.join(vues[:5]))


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
    # 'deep-idle engaged' n existe pas dans le firmware; le motif utile est
    # 'deep-idle for <n> s'. Le garder ne cassait rien (il etait en OU) mais
    # laissait croire a une couverture qui n existait pas.
    vues = _attendre_trace(b, [r'deep-idle for (\d+) s',
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
    # La veille profonde a QUATRE regimes, pas un continuum, et un seul avait un
    # cas (GPS-05, valeur 45 s). Le defaut de toutes les balises — GNP52=0 —
    # n en avait aucun, et la sentinelle non plus alors que c est le regime le
    # plus risque: son propre commentaire raconte qu une version precedente
    # arretait DEFINITIVEMENT l acquisition dans cette configuration.
    dict(id='GPS-D1', risque='BLOQUANT', titre='GNP52=0: coupure immediate, et la session suivante repart',
         fn=c_deep_idle_zero),
    dict(id='GPS-D2', risque='MAJEUR',   titre='GNP52=600: une fenetre longue est annoncee juste',
         fn=c_deep_idle_longue),
    dict(id='GPS-D3', risque='BLOQUANT', titre='Sentinelle: le rail reste allume et la balise le dit',
         fn=c_deep_idle_sentinelle),
    dict(id='GPS-D4', risque='BLOQUANT', titre='Sentinelle sans immersion: l acquisition continue',
         fn=c_deep_idle_sentinelle_sans_uw),
    dict(id='GPS-D5', risque='MAJEUR',   titre='Au-dela de 24 h, la fenetre est ramenee au plafond',
         fn=c_deep_idle_borne),
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
#
#  INVARIANT paye au prix de deux cas rouges: les commandes DTE ne repondent
#  QU EN MODE CONFIGURATION. C est ConfigurationState::process_usb_data() qui
#  alimente l analyseur (gentracker.cpp:1238), et sa scrutation est armee par
#  ConfigurationState::schedule_usb_poll(). En operationnel la carte repond
#  encore a %PING — la console banc, elle, est servie partout — mais un
#  $STATR reste sans reponse indefiniment: mesure du 2026-08-27, quatre
#  requetes ignorees en 48 s sur une carte parfaitement vivante. baseline()
#  laisse justement la carte EN configuration; un cas qui en sort doit y
#  revenir avant toute trame DTE.
# =====================================================================

def _rtc_now(b, timeout=10.0, essais=4):
    """Lit l horloge de la carte via STATR, ou None.

    Plusieurs tentatives, et pas par superstition: juste apres une sortie de
    configuration la console rend la main avec 3 a 4 secondes de retard — le
    firmware ne consomme qu une ligne par tick et l init (RCONF KIM2, BMA400,
    Argos) sature le tour de boucle. Mesure du 2026-08-27: un %DIVE emis a
    19:06:32 a recu son accuse a 19:06:36, et le STATR qui suivait est reste
    sans reponse dans sa fenetre de 6 s. Ce n etait pas un defaut, seulement
    une carte occupee.
    """
    # SYT01 = RTC_CURRENT_TIME, rafraichi a chaque lecture STATR
    # (dte_params.cpp: "Current RTC time (live, refreshed on STATR read)").
    for k in range(essais):
        m = b.dte('STATR', 'SYT01', timeout=timeout)
        ligne = (m.string if m and hasattr(m, 'string') else '') or ''
        mm = re.search(r'SYT01=(\d+)', ligne)
        if mm:
            return int(mm.group(1))
        time.sleep(2)
    return None

def _hauled(b, timeout=10.0):
    """%HAULED -> dict, ou None. Sonde ajoutee le 2026-08-27.

    Sans elle ces cas n etaient pas idempotents: l etat hors-eau vit en .noinit
    et survit au redemarrage, donc au rejeu la carte etait DEJA en HAULED,
    aucune ligne AT_SEA -> HAULED n etait emise, et le cas accusait un firmware
    qui n avait rien fait de mal. On lit l etat au lieu de le deviner.
    """
    # Plusieurs tentatives: apres une sortie de configuration la console rend
    # la main avec plusieurs secondes de retard (cf. _rtc_now), et une seule
    # fenetre trop courte fait conclure a une sonde absente du build.
    m = None
    for _ in range(4):
        mk = b.mark(); b._send('%HAULED\r')
        m = b.expect(r'%HAULED en=\d+ state=\S+ last_uw=\d+ dry_s=-?\d+ '
                     r'returns=\d+/\d+ threshold_h=\d+', timeout, from_idx=mk)
        if m:
            break
        time.sleep(2)
    if not m:
        return None
    l = m.group(0)
    d = {'_ligne': l}
    for cle, motif in (('en', r'en=(\d+)'), ('state', r'state=(\S+)'),
                       ('last_uw', r'last_uw=(\d+)'), ('dry_s', r'dry_s=(-?\d+)'),
                       ('threshold_h', r'threshold_h=(\d+)')):
        mm = re.search(motif, l)
        if mm:
            d[cle] = mm.group(1) if cle == 'state' else int(mm.group(1))
    mm = re.search(r'returns=(\d+)/(\d+)', l)
    if mm:
        d['returns'] = (int(mm.group(1)), int(mm.group(2)))
    return d

def _hauled_reset(b, timeout=10.0):
    """Repart d un AT_SEA propre: efface in_hauled ET last_uw_event_rtc."""
    for _ in range(4):
        mk = b.mark(); b._send('%HAULED RESET\r')
        if b.expect(r'%HAULED OK reset', timeout, from_idx=mk):
            return True
        time.sleep(2)
    return False

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
    if not _hauled_reset(b):
        return r.record(case, 'ERROR', '%HAULED RESET sans reponse (sonde absente du build ?)')
    time.sleep(1)
    # Un evenement d immersion ancre la derniere trace d eau a maintenant.
    # La console banc repond dans tous les etats, contrairement au DTE.
    b._send('%DIVE\r'); time.sleep(3)
    b._send('%SURFACE\r'); time.sleep(3)
    # Le repere est pose AVANT d entrer en configuration: evaluate() est appele
    # a CHAQUE lecture de parametre par ConfigurationStore, donc la transition
    # peut tomber pendant le traitement du RTCW lui-meme. Un repere pose apres
    # la sortie de configuration la manquait — et le cas concluait a l absence
    # d une transition qui avait bien eu lieu.
    mk = b.mark()
    try:
        b.enter_config()
        t0 = _rtc_now(b)
        if not t0:
            b.exit_config()
            return r.record(case, 'ERROR', 'STATR ne rend pas l horloge — saut impossible')
        saut = t0 + 2 * 3600
        m = b.dte('RTCW', str(saut), timeout=10.0)
        b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'saut d horloge impossible: {type(e).__name__}: {e}')
    if not m:
        return r.record(case, 'ERROR', f'RTCW sans reponse (t={saut})')
    vues = _attendre_trace(b, [r'AT_SEA . HAULED', r'dry for (\d+) s'], 120, depuis=mk)
    # Lire l etat AVANT de restaurer: HMP00=0 efface in_hauled (evaluate() le
    # remet a zero des que la detection est coupee), donc restaurer d abord
    # detruit la preuve qu on vient de chercher.
    etat = _hauled(b)
    try:
        b.enter_config()
        b.write_params({'HAULED_DETECT_EN': 0, 'HAULED_IDLE_THRESHOLD_H': 24,
                        'UNDERWATER_EN': 0})
        b.exit_config()
    except Exception:
        pass
    trace = '\n'.join(vues[:6]) + ('\n' + etat['_ligne'] if etat else '')
    entree = [l for l in vues if 'HAULED' in l and 'AT_SEA' in l]
    if not entree and not (etat and etat.get('state') == 'HAULED'):
        return r.record(case, 'FAIL',
                        'deux heures de secheresse pour un seuil d une heure, '
                        'mais ni trace de passage ni etat HAULED', trace)
    if etat and etat.get('state') != 'HAULED':
        return r.record(case, 'FAIL',
                        f"trace de passage mais %HAULED rapporte {etat.get('state')}", trace)
    if not entree:
        return r.record(case, 'PASS',
                        'etat HAULED atteint sur depassement du seuil de secheresse', trace)
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
    if saute_si_radio_muette(r, case, 'le dialogue avec le module'):
        return
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
    if not _hauled_reset(b):
        return r.record(case, 'ERROR', '%HAULED RESET sans reponse (sonde absente du build ?)')
    time.sleep(1)
    b._send('%DIVE\r'); time.sleep(3)
    b._send('%SURFACE\r'); time.sleep(3)
    mk = b.mark()   # avant la configuration: cf. HAUL-02
    try:
        b.enter_config()
        t0 = _rtc_now(b)
        if not t0:
            b.exit_config()
            return r.record(case, 'ERROR', 'STATR ne rend pas l horloge')
        b.dte('RTCW', str(t0 + 2 * 3600), timeout=10.0)
        b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'saut d horloge impossible: {type(e).__name__}: {e}')
    entre = _attendre_trace(b, [r'AT_SEA . HAULED'], 120, depuis=mk)
    if not entre and not ((_hauled(b) or {}).get('state') == 'HAULED'):
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
    etat = _hauled(b)   # avant la restauration: HMP00=0 effacerait in_hauled
    try:
        b.enter_config()
        b.write_params({'HAULED_DETECT_EN': 0, 'HAULED_IDLE_THRESHOLD_H': 24,
                        'UNDERWATER_EN': 0})
        b.exit_config()
    except Exception:
        pass
    trace = 'apres 1 immersion: ' + '|'.join(premier[:2]) + \
            '\napres 2 immersions: ' + '|'.join(second[:2]) + \
            ('\n' + etat['_ligne'] if etat else '')
    if premier:
        return r.record(case, 'FAIL',
                        'une seule immersion suffit a quitter HAULED alors que '
                        'HMP02=2 — une vague relancerait la cadence de mer', trace)
    if not second and not (etat and etat.get('state') == 'AT_SEA'):
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
    # baseline() laisse la carte en configuration, seul etat ou le DTE repond.
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


# =====================================================================
#  Vague 18 — detecteur d immersion analogique, observe a la source
#
#  $SWSST#000; rend les treize champs de SWSAnalogService::Status en une
#  requete synchrone (dte_handler.cpp:1573), dont `surface_level`: 0 = aucune
#  detection, 1..5 = les paliers L1..L5 de la cascade. C est le seul acces
#  direct a cette cascade, et il ne demande aucun materiel supplementaire.
#
#  Rappel: les trames DTE ne repondent QU EN MODE CONFIGURATION (cf. vague 17),
#  et baseline() y laisse la carte.
# =====================================================================

def _swsst(b, timeout=10.0, essais=4):
    """$SWSST -> dict des treize champs, ou None."""
    champs = ('air', 'eau', 'seuil', 'hysteresis', 'adc_brut', 'adc_filtre',
              'calibre', 'immerge', 'temps_etat_s', 'palier', 'contraste_x10',
              'pic_observe', 'delai_us')
    for _ in range(essais):
        m = b.dte('SWSST', '', timeout=timeout)
        ligne = (m.string if m and hasattr(m, 'string') else '') or ''
        if m and ';SWSST' in ligne:
            corps = ligne.split(';', 2)[-1]
            vals = re.findall(r'(\d+)', corps)
            # Le premier nombre est la longueur de trame: on la saute.
            vals = vals[1:] if len(vals) > len(champs) else vals
            if len(vals) >= len(champs):
                return dict(zip(champs, (int(v) for v in vals[:len(champs)])))
        time.sleep(2)
    return None

def _sws_actif(b, secondes=30):
    """Met le detecteur EN SERVICE, le laisse mesurer, puis interroge la sonde.

    Deux contraintes OPPOSEES se rencontrent ici:
      - les trames DTE ne repondent QU EN MODE CONFIGURATION (vague 17);
      - les services ne tournent QU EN DEHORS — arretes a l entree en
        configuration, redemarres a la sortie, et c est Service::start() qui
        appelle service_init() (service.cpp:585).
    Ecrire UNP01=1 en restant en configuration ne demarre donc RIEN. Il faut
    sortir, laisser mesurer, et revenir pour lire.

    CE QUI PROUVE QU UNE MESURE A EU LIEU: `temps_etat_s`, PAS l ADC.
    Tout le bloc de statut est ecrit d un coup en fin de passe
    d echantillonnage (sws_analog_detection.cpp:827), temps_etat_s compris. Un
    compteur qui avance prouve donc qu une passe a tourne.

    A l inverse, adc_brut=0 sur une electrode SECHE est une lecture NORMALE: le
    circuit RC ne se charge pas sans eau pour ponter les electrodes. Une
    premiere version exigeait un ADC non nul et declarait le cas non concluant
    cinq fois de suite, sur trois versions de firmware — en prenant une mesure
    correcte pour une mesure absente. Deux "corrections" par reprises n y ont
    rien change, et pour cause.
    """
    b.write_params({'UNDERWATER_EN': 1})
    st = None
    for _ in range(3):
        b.exit_config()
        time.sleep(secondes)
        b.enter_config()
        st = _swsst(b)
        if st and st['temps_etat_s'] > 0:
            return st
    return st

def _adc_pleine_echelle():
    """Pleine echelle SAADC, lue dans le firmware et non recopiee.

    Le SAADC de cette carte echantillonne en 14 bits: sws_analog_constants.hpp
    definit ADC_INVALID_MAX 16383, et nrf_battery_mon.cpp comme thermistor.cpp
    posent ADC_MAX_VALUE 16384 (2^14). Un cas qui assertait contre 4095 (12
    bits) declarait aberrantes des lectures parfaitement valides — mesure du
    2026-08-29: eau=15050, pic=15060, sur une electrode saine.
    """
    import os
    chemin = os.path.join(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))),
        'core', 'services', 'sws_analog_constants.hpp')
    try:
        m = re.search(r'#define\s+ADC_INVALID_MAX\s+(\d+)', open(chemin).read())
        if m:
            return int(m.group(1))
    except OSError:
        pass
    return 16383   # 2^14 - 1, la valeur du firmware au 2026-08


def c_sws_etat_coherent(r, case):
    """Le detecteur rend un etat physiquement coherent, a sec.

    Sur la paillasse l electrode est a l air: la carte DOIT se dire hors de
    l eau, et l ADC doit se tenir du cote air du seuil. C est le controle le
    plus elementaire de toute la chaine d immersion, et pourtant rien ne le
    verifiait: si le seuil derive au-dessus de la mesure d air, la balise se
    croit immergee en permanence et cesse d emettre — panne muette et totale.
    """
    b = r.b
    try:
        st = _sws_actif(b)
    except Exception as e:
        return r.record(case, 'ERROR', f'mise en service impossible: {type(e).__name__}: {e}')
    if not st:
        return r.record(case, 'ERROR', '$SWSST sans reponse (ENABLE_SWS_ANALOG absent ?)')
    trace = ', '.join(f'{k}={v}' for k, v in st.items())
    defauts = []
    if not st['temps_etat_s']:
        try:
            b.write_params({'UNDERWATER_EN': 0})
        except Exception:
            pass
        return r.record(case, 'ERROR',
                        'le compteur de temps dans l etat n avance pas: aucune passe '
                        'd echantillonnage n a tourne — cas non concluant', trace)
    if st['immerge']:
        defauts.append('la carte se declare IMMERGEE alors qu elle est a l air')
    if not st['calibre']:
        defauts.append('calibration invalide')
    if not (st['air'] < st['seuil'] < st['eau']):
        defauts.append(f"seuil hors de l intervalle air..eau "
                       f"({st['air']} < {st['seuil']} < {st['eau']} est faux)")

    # Pleine echelle SAADC: toute valeur au-dela est une lecture aberrante. La
    # borne vient du firmware (ADC_INVALID_MAX), pas d une constante recopiee.
    pleine_echelle = _adc_pleine_echelle()
    for cle in ('air', 'eau', 'seuil', 'adc_brut', 'adc_filtre', 'pic_observe'):
        if st[cle] > pleine_echelle:
            defauts.append(f'{cle}={st[cle]} depasse la pleine echelle SAADC '
                           f'({pleine_echelle})')
    if st['palier'] > 5:
        defauts.append(f"palier de detection {st['palier']} hors de la plage L1..L5")
    try:
        b.write_params({'UNDERWATER_EN': 0})
    except Exception:
        pass
    if defauts:
        return r.record(case, 'FAIL', '; '.join(defauts), trace)
    r.record(case, 'PASS',
             f"a sec apres {st['temps_etat_s']} s d echantillonnage: hors de l eau, calibre, "
             f"air={st['air']} < seuil={st['seuil']} < eau={st['eau']}, adc={st['adc_brut']}",
             trace)

def c_sws_hysteresis_appliquee(r, case):
    """UNP22 atteint reellement le service au redemarrage.

    L hysteresis est ce qui empeche le detecteur de basculer a chaque clapot.
    Le risque qu on couvre ici est celui, deja rencontre sur les seuils
    batterie, d un parametre ecrit puis jamais relu: sur une balise scellee,
    reglee par BLE et jamais redemarree a la main, un reglage qui n arrive pas
    au service est un reglage perdu en silence.

    Ce que ce cas N AFFIRME PAS, et pourquoi — trois passes l ont etabli au
    banc le 2026-08-27: la valeur EN COMPTES ADC (champ `hysteresis` de $SWSST)
    est calculee dans update_dynamic_threshold() (sws_analog_calibration.cpp:379),
    appele uniquement sur un evenement de RECALIBRATION. Electrode sechee et
    stable, aucune recalibration ne se declenche, et le champ reste a la valeur
    du .noinit quel que soit UNP22 — 10 aux trois essais, y compris avec
    UNP23=60 force. Ce n est pas un defaut, c est un delai d application: le
    reglage prend effet au prochain recalage. La preuve du bon acheminement est
    la ligne d init, elle, immediate:

        19:50:40  $PARMW UNP22=30,UNP23=60
        19:50:50  SWSAnalog: Init - hyst=30% ratio=35%

    A retenir cote exploitation: un operateur qui regle l hysteresis et relit
    dans la foulee verra l ANCIENNE valeur en comptes ADC. Ce n est pas que son
    reglage a ete refuse.
    """
    b = r.b
    def _appliquer(valeur):
        b.write_params({'SWS_ANALOG_HYSTERESIS': valeur, 'UNDERWATER_EN': 1})
        mk = b.mark()
        b.exit_config()
        vues = _attendre_trace(b, [r'SWSAnalog: Init - hyst=(\d+)%'], 60, depuis=mk)
        b.enter_config()
        for l in vues:
            m = re.search(r'hyst=(\d+)%', l)
            if m:
                return int(m.group(1))
        return None
    try:
        bas = _appliquer(4)
        haut = _appliquer(30)
        b.write_params({'SWS_ANALOG_HYSTERESIS': 4, 'UNDERWATER_EN': 0})
    except Exception as e:
        return r.record(case, 'ERROR', f'ecriture UNP22 impossible: {type(e).__name__}: {e}')
    trace = f'UNP22=4  -> init hyst={bas}%\nUNP22=30 -> init hyst={haut}%'
    if bas is None or haut is None:
        return r.record(case, 'ERROR',
                        'le service ne trace pas son hysteresis au demarrage — non concluant',
                        trace)
    if bas != 4 or haut != 30:
        return r.record(case, 'FAIL',
                        f'le service demarre avec hyst={bas}% puis {haut}% au lieu de 4 % puis 30 % — '
                        'UNP22 n atteint pas le service', trace)
    r.record(case, 'PASS', 'UNP22 est relu par le service a chaque demarrage (4 % puis 30 %)',
             trace)

def c_sws_borne_delai(r, case):
    """Le delai de charge RC reste dans les bornes UNP09..UNP10.

    Le delai s adapte tout seul pour garder du contraste entre air et eau. S il
    sort de ses bornes, la mesure perd son sens: trop court elle lit du bruit,
    trop long elle epuise la batterie a chaque echantillon. Les bornes sont
    la seule protection, et elles n avaient jamais ete verifiees a chaud.
    """
    b = r.b
    try:
        _, p = b.read_params(['SWS_DELAY_MIN_US', 'SWS_DELAY_MAX_US'])
        mini = int(p.get(b._key('SWS_DELAY_MIN_US'), 0))
        maxi = int(p.get(b._key('SWS_DELAY_MAX_US'), 0))
    except Exception as e:
        return r.record(case, 'ERROR', f'lecture UNP09/UNP10 impossible: {type(e).__name__}: {e}')
    try:
        st = _sws_actif(b)
        b.write_params({'UNDERWATER_EN': 0})
    except Exception as e:
        return r.record(case, 'ERROR', f'mise en service impossible: {type(e).__name__}: {e}')
    if not st:
        return r.record(case, 'ERROR', '$SWSST sans reponse')
    d = st['delai_us']
    trace = f'delai={d} us, bornes UNP09={mini} UNP10={maxi}'
    if mini and d < mini:
        return r.record(case, 'FAIL', f'delai {d} us sous la borne basse {mini} us', trace)
    if maxi and d > maxi:
        return r.record(case, 'FAIL', f'delai {d} us au-dessus de la borne haute {maxi} us', trace)
    r.record(case, 'PASS', f'delai de charge {d} us, dans [{mini}, {maxi}]', trace)

def c_sws_contraste(r, case):
    """Le contraste eau/air est annonce et exploitable.

    Sous MIN_WATER_AIR_RATIO la calibration ne distingue plus l eau de l air, et
    le firmware doit le dire plutot que de detecter au hasard. On verifie que le
    champ est renseigne et concorde avec les lignes de base air et eau.
    """
    b = r.b
    try:
        st = _sws_actif(b)
        b.write_params({'UNDERWATER_EN': 0})
    except Exception as e:
        return r.record(case, 'ERROR', f'mise en service impossible: {type(e).__name__}: {e}')
    if not st:
        return r.record(case, 'ERROR', '$SWSST sans reponse')
    trace = ', '.join(f'{k}={v}' for k, v in st.items())
    if st['air'] <= 0:
        return r.record(case, 'FAIL', 'ligne de base air nulle — contraste indefinissable', trace)
    attendu = round(10.0 * st['eau'] / st['air'])
    ecart = abs(st['contraste_x10'] - attendu)
    if st['contraste_x10'] == 0:
        # PAS un defaut: m_contrast_x10 n est renseigne que par
        # update_dynamic_threshold(), lui-meme appele uniquement sur un
        # evenement de RE-calibration (sws_analog_detection.cpp:185, 220, 449...).
        # Sur un banc sec, electrode stable et calibration restauree du .noinit,
        # aucune recalibration ne se declenche: le champ reste a zero alors que
        # la calibration est parfaitement valide. C est un cache d execution,
        # ni persiste ni recalcule a l init.
        #
        # A SIGNALER cote IHM: un afficheur qui montre "contraste 0.0x" sur une
        # balise saine inquiete pour rien. C est cosmetique, pas fonctionnel.
        return r.record(case, 'SKIP',
                        'contraste non calcule: aucune recalibration depuis le demarrage, '
                        'ce qui est l etat NORMAL d une electrode a sec et stable. Le champ '
                        'est un cache d execution, ni persiste ni recalcule a l init. '
                        'Ce cas demande une electrode mouillee.', trace)
    # Une unite de tolerance: le firmware arrondit, nous aussi.
    if ecart > 1:
        return r.record(case, 'FAIL',
                        f"contraste annonce {st['contraste_x10']}/10 mais eau/air donne "
                        f'{attendu}/10', trace)
    r.record(case, 'PASS',
             f"contraste {st['contraste_x10'] / 10:.1f}x, concorde avec eau/air", trace)

def c_limiteur_fenetre_glissante(r, case):
    """Le quota se libere quand la fenetre glisse — le limiteur ne bloque pas a vie.

    RL-02 prouve que le limiteur BLOQUE au-dela du quota. Le risque symetrique
    n etait pas couvert: qu il ne debloque JAMAIS. Une balise definitivement
    muette apres N emissions serait pire que pas de limiteur du tout, et rien ne
    le signalerait.

    CORRECTION 2026-08-28: la premiere version posait ARGOS_MODE=0 et concluait
    "aucune emission possible sans credentials KIM2". C etait FAUX sur les deux
    points — la carte emet (26 TX SUCCESS type=gnss releves dans les journaux,
    aucun +ERROR), et c est le mode OFF que le cas s imposait lui-meme qui
    empechait toute emission. Le limiteur ne pouvait rien compter parce que rien
    ne partait.
    """
    if saute_si_radio_muette(r, case, 'une emission reussie'):
        return
    b = r.b
    try:
        b.enter_config()
        b.write_params({'ARGOS_MODE': 2, 'TR_NOM': 30, 'NTRY_PER_MESSAGE': 0,
                        'ARGOS_DEPTH_PILE': 1, 'DUTY_CYCLE': 16777215,
                        'GNSS_EN': 1, 'UNDERWATER_EN': 0, 'LB_EN': 0,
                        'SAT_PREPASS_EN': 0, 'MIN_SURFACE_CYCLE_INTERVAL_S': 0,
                        'RATE_LIMIT_EN': 1, 'RATE_LIMIT_WINDOW_S': 120,
                        'RATE_LIMIT_MAX_TX': 1})
        b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    time.sleep(3)
    mk = b.mark()
    b._send('%GPS 43.6 3.9 5000 9\r')

    # 1. Une emission part, puis le quota (1 sur 120 s) doit bloquer la suivante.
    vues = _attendre_trace(b, [r'TX SUCCESS', r'rate limit reached'], 150, depuis=mk,
                           exiger=[r'rate limit reached'])
    emis = [l for l in vues if 'TX SUCCESS' in l]
    bloque = [l for l in vues if 'rate limit reached' in l]
    if not emis:
        try:
            b.enter_config()
            b.write_params({'RATE_LIMIT_EN': 0, 'ARGOS_MODE': 0}); b.exit_config()
        except Exception:
            pass
        return r.record(case, 'ERROR',
                        'aucune emission en 150 s — le limiteur n a rien a compter',
                        '\n'.join(vues[:6]))
    if not bloque:
        try:
            b.enter_config()
            b.write_params({'RATE_LIMIT_EN': 0, 'ARGOS_MODE': 0}); b.exit_config()
        except Exception:
            pass
        return r.record(case, 'FAIL',
                        'une emission est partie avec un quota de 1/120 s, et pourtant le '
                        'limiteur n annonce aucun blocage', '\n'.join(vues[:6]))

    # 2. Le delai annonce doit etre FINI et compatible avec la fenetre.
    delais = [int(m.group(1)) for l in bloque
              for m in [re.search(r'reschedule in (\d+) s', l)] if m]
    trace = '\n'.join(vues[:8]) + f'\ndelais annonces: {delais} s'
    try:
        b.enter_config()
        b.write_params({'RATE_LIMIT_EN': 0, 'RATE_LIMIT_WINDOW_S': 3600,
                        'RATE_LIMIT_MAX_TX': 10, 'ARGOS_MODE': 0})
        b.exit_config()
    except Exception:
        pass
    if not delais:
        return r.record(case, 'ERROR',
                        'blocage annonce sans delai de reprise — cas non concluant', trace)
    pire = max(delais)
    # La fenetre fait 120 s: une reprise annoncee bien au-dela signifie que le
    # deblocage n est pas indexe sur la fenetre glissante.
    if pire > 300:
        return r.record(case, 'FAIL',
                        f'reprise annoncee dans {pire} s pour une fenetre de 120 s — '
                        'le deblocage ne suit pas la fenetre', trace)
    r.record(case, 'PASS',
             f'emission comptee, blocage annonce, reprise bornee a {pire} s pour 120 s de fenetre',
             trace)

CASES_V18 = [
    dict(id='SWS-04', risque='BLOQUANT', titre='A sec, le detecteur est coherent et calibre',
         fn=c_sws_etat_coherent),
    dict(id='SWS-05', risque='MAJEUR',   titre='UNP22 porte sur l hysteresis effective',
         fn=c_sws_hysteresis_appliquee),
    dict(id='SWS-06', risque='MAJEUR',   titre='Le delai de charge respecte UNP09..UNP10',
         fn=c_sws_borne_delai),
    dict(id='SWS-07', risque='MAJEUR',   titre='Le contraste eau/air concorde avec les lignes de base',
         fn=c_sws_contraste),
    dict(id='RL-03',  risque='MAJEUR',   titre='Le limiteur debloque quand la fenetre glisse',
         fn=c_limiteur_fenetre_glissante),
]


# =====================================================================
#  Vague 19 — LED: la couleur annoncee est-elle celle du contrat ?
#
#  C est le seul retour visuel dont dispose l operateur au moment de poser
#  l animal. Une couleur fausse ne casse rien dans le firmware et se paie
#  entierement sur le terrain: on croit la balise en acquisition alors qu elle
#  est en panne, ou l inverse.
#
#  Le domaine a deja livre deux defauts (LED_MODE_GUARD qui levait une
#  exception depuis l ISR, huit transits differes jamais annules); ces cas-ci
#  eprouvent le CONTRAT documente, page 10 du wiki.
#
#  Rappel de lecture: _led() rend `couleur or clignote`, parce que flash() ne
#  met pas a jour m_color — une LED clignotante se lit BLACK si on ne regarde
#  que la couleur solide.
# =====================================================================

# core/hardware/rgb_led.hpp
NOIR, ROUGE, VERT, BLEU, CYAN, MAGENTA, JAUNE, BLANC = range(8)

def _led_mode(b, mode):
    """Pose LED_MODE et laisse la FSM le prendre en compte."""
    b.write_params({'LED_MODE': mode})
    time.sleep(2)

def c_led_contrat_couleurs(r, case):
    """Chaque evenement allume la couleur que le wiki lui associe.

    Le contrat (10-GPS-Guide, tableau des etats LED):
      GNSSON    -> CYAN clignotant lent   (acquisition en cours)
      GNSSNOFIX -> ROUGE fixe             (session terminee sans position)
      ARGOSTX   -> MAGENTA fixe           (emission satellite)
      OFF       -> eteint
    On verifie l ETAT de la machine LED autant que la couleur: une couleur
    juste atteinte par le mauvais etat serait un faux positif.
    """
    b = r.b
    try:
        # GNSS_EN=0: dehors, une acquisition reelle pousse ses propres evenements
        # LED au milieu de la sequence injectee (cf. LED-01).
        b.write_params({'GNSS_EN': 0})
        _led_mode(b, 3)   # ALWAYS: sinon la fenetre 24 h peut tout eteindre
    except Exception as e:
        return r.record(case, 'ERROR', f'LED_MODE=3 impossible: {type(e).__name__}: {e}')
    if _led_calme(b) is None:
        return r.record(case, 'ERROR',
                        'la FSM LED bouge encore toute seule (session GNSS en vol ?) — '
                        'sequence non injectable, cas non concluant')
    attendus = [
        ('GNSSON',    'GNSSOn',           None,    'acquisition en cours'),
        ('GNSSNOFIX', 'GNSSOffWithout',   ROUGE,   'session sans position'),
        ('ARGOSTX',   'ArgosTX',          MAGENTA, 'emission satellite'),
    ]
    defauts, observe = [], []
    for evt, etat_attendu, couleur_attendue, quoi in attendus:
        etat, couleur = _led(b, evt)
        observe.append(f'{evt}: etat={etat} couleur={couleur}')
        if etat is None:
            defauts.append(f'{evt}: sonde sans reponse')
            continue
        if etat_attendu not in etat:
            defauts.append(f'{evt} ({quoi}): etat={etat}, attendu ~{etat_attendu}')
        if couleur_attendue is not None and couleur != couleur_attendue:
            defauts.append(f'{evt} ({quoi}): couleur={couleur}, attendu {couleur_attendue}')
        elif couleur_attendue is None and not couleur:
            defauts.append(f'{evt} ({quoi}): LED eteinte alors qu elle doit signaler')
        time.sleep(1)
    _led(b, 'OFF'); time.sleep(1)
    etat, couleur = _led(b)
    observe.append(f'OFF: etat={etat} couleur={couleur}')
    if couleur:
        defauts.append(f'apres OFF la LED reste allumee (couleur={couleur})')
    try:
        b.write_params({'LED_MODE': 1})
    except Exception:
        pass
    trace = '\n'.join(observe)
    if defauts:
        return r.record(case, 'FAIL', f'{len(defauts)} ecart(s) au contrat: ' + '; '.join(defauts),
                        trace)
    r.record(case, 'PASS', 'acquisition, echec de fix et emission portent la bonne couleur', trace)

def c_led_mode_off_total(r, case):
    """LED_MODE=OFF doit tout eteindre, y compris l emission Argos.

    C est le reglage des balises posees sur oiseaux: une LED visible attire
    l attention d un predateur ou d un tiers, et consomme. Si un seul evenement
    passe outre, le reglage ne vaut rien. On eprouve donc les trois evenements
    les plus voyants, dont ARGOSTX qui a une exception documentee — mais pour
    SATDP seulement, pas pour une emission ordinaire.
    """
    b = r.b
    try:
        b.write_params({'GNSS_EN': 0})
        _led_mode(b, 0)
    except Exception as e:
        return r.record(case, 'ERROR', f'LED_MODE=0 impossible: {type(e).__name__}: {e}')
    if _led_calme(b) is None:
        return r.record(case, 'ERROR',
                        'la FSM LED bouge encore toute seule (session GNSS en vol ?) — '
                        'sequence non injectable, cas non concluant')
    allumes, observe = [], []
    for evt in ('GNSSON', 'GNSSNOFIX', 'ARGOSTX'):
        etat, couleur = _led(b, evt)
        observe.append(f'{evt}: etat={etat} couleur={couleur}')
        if couleur:
            allumes.append(f'{evt} (couleur={couleur})')
        time.sleep(1)
    try:
        _led(b, 'OFF')
        b.write_params({'LED_MODE': 1})
    except Exception:
        pass
    trace = '\n'.join(observe)
    if allumes:
        return r.record(case, 'FAIL',
                        'LED_MODE=OFF mais la LED s allume sur: ' + ', '.join(allumes), trace)
    r.record(case, 'PASS', 'aucun des trois evenements n allume la LED en mode OFF', trace)

def c_led_cloudlocate_preserve(r, case):
    """La double impulsion CloudLocate n est pas ecrasee par la fin de session.

    Garde de non-regression sur un defaut reel: LEDGNSSOffWithoutFix arrivait
    juste apres CLREADY et eteignait la signalisation avant qu elle soit vue.
    Le firmware la DIFFERE de 500 ms pour cette raison precise (wiki, tableau
    des etats). Si la garde saute, l operateur ne voit jamais que la mesure
    brute a ete capturee — et croit la session ratee.
    """
    b = r.b
    try:
        b.write_params({'GNSS_EN': 0})
        _led_mode(b, 3)
    except Exception as e:
        return r.record(case, 'ERROR', f'LED_MODE=3 impossible: {type(e).__name__}: {e}')
    if _led_calme(b) is None:
        return r.record(case, 'ERROR',
                        'la FSM LED bouge encore toute seule (session GNSS en vol ?) — '
                        'sequence non injectable, cas non concluant')
    etat_cl, couleur_cl = _led(b, 'CLREADY')
    # Immediatement: la fin de session sans fix, celle qui ecrasait.
    etat_apres, couleur_apres = _led(b, 'GNSSNOFIX')
    time.sleep(1)
    etat_final, couleur_final = _led(b)
    try:
        _led(b, 'OFF')
        b.write_params({'LED_MODE': 1})
    except Exception:
        pass
    trace = (f'CLREADY   -> etat={etat_cl} couleur={couleur_cl}\n'
             f'GNSSNOFIX -> etat={etat_apres} couleur={couleur_apres}\n'
             f'apres 1 s -> etat={etat_final} couleur={couleur_final}')
    if etat_cl is None:
        return r.record(case, 'ERROR', 'sonde LED sans reponse', trace)
    if 'CloudLocate' not in (etat_cl or ''):
        return r.record(case, 'FAIL',
                        f'CLREADY ne mene pas a l etat CloudLocate (etat={etat_cl})', trace)
    if not couleur_cl:
        return r.record(case, 'FAIL', 'CLREADY n allume rien', trace)
    r.record(case, 'PASS',
             'la signalisation CloudLocate est bien etablie avant la fin de session', trace)

CASES_V19 = [
    dict(id='LED-03', risque='MAJEUR',   titre='Chaque evenement porte la couleur du contrat',
         fn=c_led_contrat_couleurs),
    dict(id='LED-04', risque='BLOQUANT', titre='LED_MODE=OFF eteint tout, sans exception',
         fn=c_led_mode_off_total),
    dict(id='LED-05', risque='MAJEUR',   titre='La signalisation CloudLocate n est pas ecrasee',
         fn=c_led_cloudlocate_preserve),
]


# =====================================================================
#  Vague 20 — zone geographique, le reste du contrat
#
#  ZONE-01..04 couvrent la substitution, la frontiere, la desactivation et la
#  priorite de la batterie. Restent trois leviers jamais eprouves, et chacun
#  peut faire taire une balise sans qu aucune erreur ne soit tracee.
# =====================================================================

def _date_dte(epoch):
    """Epoch -> 'DD/MM/YYYY HH:MM:SS', le format attendu par decode_datestring."""
    t = time.gmtime(epoch)
    return time.strftime('%d/%m/%Y %H:%M:%S', t)

def c_zone_date_activation(r, case):
    """ZONE_ENABLE_ACTIVATION_DATE retarde la zone jusqu a la date prevue.

    Le cas d usage est une campagne qui commence a une date connue: la balise
    part avec sa zone deja configuree, mais celle-ci ne doit pas mordre avant.
    Si la date est ignoree, le profil de zone s applique des le premier fix —
    et une zone reglee pour economiser (cadence lente, pile courte) ferait
    manquer tout le debut de la campagne.

    Detail qui compte: la date est comparee a l HORODATAGE DU FIX, pas a
    l horloge courante (config_store.hpp:836, convert_epochtime sur
    m_last_gps_log_entry). Comme bench_inject_fix estampille depuis le RTC,
    piloter l horloge pilote les deux cotes de la comparaison.
    """
    b = r.b
    try:
        b.enter_config()
        maintenant = _rtc_now(b)
        b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'lecture d horloge impossible: {type(e).__name__}: {e}')
    if not maintenant:
        return r.record(case, 'ERROR', 'STATR ne rend pas l horloge')

    # 1. Date d activation dans le FUTUR: la zone ne doit pas mordre.
    try:
        _profil_zone(b, ooz_actif=True, ZONE_ENABLE_ACTIVATION_DATE=1,
                     ZONE_ACTIVATION_DATE=_date_dte(maintenant + 30 * 86400))
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    avant = _injecte_et_lit(b, *_DEHORS)

    # 2. Date d activation dans le PASSE: la zone doit mordre.
    try:
        _profil_zone(b, ooz_actif=True, ZONE_ENABLE_ACTIVATION_DATE=1,
                     ZONE_ACTIVATION_DATE=_date_dte(maintenant - 30 * 86400))
    except Exception as e:
        return r.record(case, 'ERROR', f'reconfiguration impossible: {type(e).__name__}: {e}')
    apres = _injecte_et_lit(b, *_DEHORS)

    try:
        b.enter_config()
        b.write_params({'ZONE_ENABLE_OUT_OF_ZONE_DETECTION_MODE': 0,
                        'ZONE_ENABLE_ACTIVATION_DATE': 0})
        b.exit_config()
    except Exception:
        pass
    trace = f'date future -> {avant}\ndate passee -> {apres}'
    if avant is None or apres is None:
        return r.record(case, 'ERROR', '%ARGOSCFG sans reponse', trace)
    if _est_profil_zone(avant):
        return r.record(case, 'FAIL',
                        'le profil de zone s applique alors que la date d activation '
                        'est dans 30 jours', trace)
    if not _est_profil_zone(apres):
        return r.record(case, 'FAIL',
                        'la date d activation est depassee de 30 jours mais le profil '
                        'de zone ne s applique pas', trace)
    r.record(case, 'PASS',
             'la zone reste inerte avant sa date d activation et mord apres', trace)

def c_zone_duty_cycle(r, case):
    """ZONE_ARGOS_DUTY_CYCLE remplace bien le masque horaire nominal.

    Un masque de duty-cycle est le moyen le plus direct de faire taire une
    balise: a zero elle n emet plus une seule fois. C est deja arrive
    (DUTY-01), et la variante de zone n avait jamais ete verifiee — un masque
    de zone mal recopie rendrait la balise muette des qu elle sort du domaine,
    c est-a-dire precisement quand on veut la suivre.
    """
    b = r.b
    try:
        _profil_zone(b, ooz_actif=True,
                     DUTY_CYCLE=0xFFFFFF, ZONE_ARGOS_DUTY_CYCLE=0x0F0F0F)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    dedans = _injecte_et_lit(b, *_DEDANS)
    dehors = _injecte_et_lit(b, *_DEHORS)
    try:
        b.enter_config()
        b.write_params({'ZONE_ENABLE_OUT_OF_ZONE_DETECTION_MODE': 0,
                        'ZONE_ARGOS_DUTY_CYCLE': 0xFFFFFF})
        b.exit_config()
    except Exception:
        pass
    trace = f'dans la zone -> {dedans}\nhors zone    -> {dehors}'
    if dedans is None or dehors is None:
        return r.record(case, 'ERROR', '%ARGOSCFG sans reponse', trace)
    if dedans['duty'] != 0xFFFFFF:
        return r.record(case, 'FAIL',
                        f"dans la zone, duty={dedans['duty']:#08x} au lieu du masque nominal",
                        trace)
    if dehors['duty'] != 0x0F0F0F:
        return r.record(case, 'FAIL',
                        f"hors zone, duty={dehors['duty']:#08x} au lieu du masque de zone "
                        '0x0f0f0f', trace)
    r.record(case, 'PASS', 'le masque horaire de zone se substitue au nominal', trace)

def c_zone_gnss_timeout(r, case):
    """ZONE_GNSS_ACQ_TIMEOUT remplace le timeout nominal hors zone.

    Hors du domaine, on veut souvent chercher la position plus longtemps — la
    balise est la ou on ne l attendait pas. Le timeout effectif se lit dans la
    trace d allumage du recepteur (`M10Q on — nav_max=%u`), seul endroit ou la
    valeur retenue est visible.
    """
    b = r.b
    try:
        _profil_zone(b, ooz_actif=True, GNSS_ACQ_TIMEOUT=60, ZONE_GNSS_ACQ_TIMEOUT=180)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    mk = b.mark()
    b._send(f'%GPS {_DEHORS[0]} {_DEHORS[1]} 5000 9\r')
    vues = _attendre_trace(b, [r'M10Q on — nav_max=(\d+)'], 120, depuis=mk)
    try:
        b.enter_config()
        b.write_params({'ZONE_ENABLE_OUT_OF_ZONE_DETECTION_MODE': 0,
                        'GNSS_ACQ_TIMEOUT': 120, 'ZONE_GNSS_ACQ_TIMEOUT': 120})
        b.exit_config()
    except Exception:
        pass
    trace = '\n'.join(vues[:4])
    valeurs = [int(m.group(1)) for l in vues
               for m in [re.search(r'nav_max=(\d+)', l)] if m]
    if not valeurs:
        return r.record(case, 'ERROR',
                        'aucune trace d allumage du recepteur en 120 s — cas non concluant',
                        trace)
    if 180 not in valeurs:
        return r.record(case, 'FAIL',
                        f'hors zone, nav_max={valeurs} au lieu des 180 de ZOP17', trace)
    r.record(case, 'PASS', 'hors zone, le recepteur adopte le timeout de zone (180)', trace)

CASES_V20 = [
    dict(id='ZONE-05', risque='MAJEUR',   titre='La zone reste inerte avant sa date d activation',
         fn=c_zone_date_activation),
    dict(id='ZONE-06', risque='BLOQUANT', titre='Le masque horaire de zone se substitue au nominal',
         fn=c_zone_duty_cycle),
    dict(id='ZONE-07', risque='MAJEUR',   titre='Le timeout GNSS de zone se substitue au nominal',
         fn=c_zone_gnss_timeout),
]


# =====================================================================
#  Vague 21 — sortie des donnees: DUMPD et son protocole multi-paquets
#
#  C est par la que la mission quitte la balise. Le protocole rend
#  "mmm,MMM,<charge>" ou mmm est l index du paquet et MMM le dernier index
#  (dte_handler.cpp:418). Une sequence d index cassee, c est de la donnee
#  perdue en silence: rien ne l annonce, et le journal reste intact sur la
#  carte pendant que l operateur croit l avoir recupere.
# =====================================================================

_DUMPD_SYSTEME = 0     # BaseLogDType::INTERNAL
_DUMPD_GNSS    = 1     # BaseLogDType::GNSS_SENSOR
_DUMPD_PLAFOND = 48    # paquets aspires au plus par cas — voir _dumpd()

# En-tete d un paquet: $O;DUMPD#<len>;<mmm>,<MMM>,<charge base64>
# Les DEUX index sont en HEXADECIMAL (mesure: "A,3FE" puis "10,3FE"), ce qu un
# motif en \d+ manque des le onzieme paquet.
_RE_DUMPD = re.compile(r'\$O;DUMPD#[0-9A-Fa-f]+;([0-9A-Fa-f]+),([0-9A-Fa-f]+),')

def _dumpd(b, d_type, plafond=_DUMPD_PLAFOND, silence=6.0):
    """Aspire un journal. Rend (index vus, MMM, refus, tronque).

    UNE requete declenche un FLUX de paquets, elle n en rend pas un seul.
    Mesure du 2026-08-27, horodatages a la milliseconde:

        20:26:50.709  >> $DUMPD#001;0
        20:26:50.710  $O;DUMPD#23E;1,3FE,...
        20:26:50.715  $O;DUMPD#382;2,3FE,...   <- sans nouvelle requete

    Une premiere version envoyait une requete par paquet en posant son repere
    juste avant: elle sautait les paquets arrives entre-temps et lisait
    [0, 1, 3]. Le firmware n y etait pour rien.

    `plafond` borne l aspiration — le journal systeme fait 1023 paquets et on
    n a pas besoin de tout pour eprouver la sequence. La troncature est RENDUE
    a l appelant, jamais passee sous silence.
    """
    mk = b.mark()
    b._send(f'$DUMPD#001;{d_type}\r')
    vus, mmm, refus = [], None, False
    dernier_recu = time.time()
    while time.time() - dernier_recu < silence and len(vus) < plafond:
        time.sleep(0.5)
        with b._lock:
            lignes = [l for _, l in b.history[mk:]]
        nouveaux = []
        for l in lignes:
            m = _RE_DUMPD.search(l)
            if m:
                nouveaux.append((int(m.group(1), 16), int(m.group(2), 16)))
            elif '$N;DUMPD' in l:
                refus = True
        if len(nouveaux) > len(vus):
            dernier_recu = time.time()
        vus = [i for i, _ in nouveaux]
        if nouveaux:
            mmm = nouveaux[-1][1]
        if mmm is not None and vus and vus[-1] >= mmm:
            break
    tronque = bool(mmm is not None and vus and vus[-1] < mmm)
    return vus, mmm, refus, tronque

def _dumpd_silence(b, silence=8.0, plafond=180.0):
    """Attend que le flux DUMPD se taise.

    Indispensable avant tout cas qui interroge DUMPD: une aspiration
    precedente continue de deverser ses paquets (le journal systeme en compte
    1023), et ces paquets repondent au meme motif. Sans cette purge, un cas lit
    la fin du flux precedent en croyant lire sa propre reponse — c est ainsi
    que DUMP-02 a rapporte un index 230 pour un journal qui commence a 0.
    """
    fin = time.time() + plafond
    mk = b.mark()
    dernier = time.time()
    vu = 0
    while time.time() < fin:
        time.sleep(1.0)
        with b._lock:
            lignes = [l for _, l in b.history[mk:]]
        n = sum(1 for l in lignes if _RE_DUMPD.search(l))
        if n > vu:
            vu = n
            dernier = time.time()
        elif time.time() - dernier >= silence:
            return True
    return False

def _dumpd_flux(b, d_type, plafond=8, silence=6.0):
    """Comme _dumpd mais rend les paires (index, MMM) telles quelles."""
    mk = b.mark()
    b._send(f'$DUMPD#001;{d_type}\r')
    paires, refus = [], False
    dernier_recu = time.time()
    while time.time() - dernier_recu < silence and len(paires) < plafond:
        time.sleep(0.5)
        with b._lock:
            lignes = [l for _, l in b.history[mk:]]
        courant = []
        for l in lignes:
            m = _RE_DUMPD.search(l)
            if m:
                courant.append((int(m.group(1), 16), int(m.group(2), 16)))
            elif '$N;DUMPD' in l:
                refus = True
        if len(courant) > len(paires):
            dernier_recu = time.time()
        paires = courant
    return paires, refus

def c_dumpd_sequence(r, case):
    """Les index de paquets se suivent sans trou ni repetition.

    Cette paire d index est le seul controle d integrite dont dispose l hote.
    Un saut ou une repetition signifie que des entrees ne remontent jamais — et
    comme la carte considere le journal comme lu, la perte est definitive et
    muette.
    """
    b = r.b
    _dumpd_silence(b)
    vus, mmm, refus, tronque = _dumpd(b, _DUMPD_SYSTEME)
    apercu = vus[:20]
    trace = (f'index vus = {apercu}{" ..." if len(vus) > 20 else ""}\n'
             f'{len(vus)} paquets, MMM = {mmm}, tronque = {tronque}')
    if refus:
        return r.record(case, 'ERROR', 'DUMPD refuse la demande', trace)
    if not vus:
        return r.record(case, 'ERROR', 'DUMPD ne rend aucun paquet — journal vide ?', trace)
    defauts = []
    if vus[0] != 0:
        defauts.append(f'la sequence commence a {vus[0]} au lieu de 0')
    if vus != list(range(len(vus))):
        manquants = sorted(set(range(vus[-1] + 1)) - set(vus))
        defauts.append(f'index non consecutifs, manquants: {manquants[:12]}')
    if not tronque and mmm is not None and vus[-1] != mmm:
        defauts.append(f'arret a l index {vus[-1]} alors que MMM={mmm}')
    if defauts:
        return r.record(case, 'FAIL', '; '.join(defauts), trace)
    borne = f'jusqu a {vus[-1]} (plafond du cas, MMM={mmm})' if tronque else f'de 0 a {mmm}'
    r.record(case, 'PASS', f'{len(vus)} paquets consecutifs {borne}', trace)

def c_dumpd_changement_type(r, case):
    """Deux aspirations successives de journaux differents partent chacune de 0.

    C est le geste courant de l operateur: recuperer le journal systeme, puis
    celui des positions. Si l etat d index fuit de l une a l autre, la seconde
    reprend la ou la premiere s est arretee et son debut n arrive jamais — on
    croit tenir le journal GNSS alors qu il en manque la tete, sans qu aucune
    erreur ne soit tracee. Le firmware s en protege (dte_handler.cpp:437,
    "Reset state if dump type changed mid-stream").

    Ce que ce cas N ATTEINT PAS, et pourquoi: la bascule VRAIMENT en plein flux
    n est pas eprouvable sur ce lien. Trois passes l ont montre le 2026-08-27 —
    le flux sortant sature la console (le firmware ne consomme qu une ligne par
    tick) et la requete de bascule reste sans reponse. On ne saurait pas
    distinguer une garde qui n a pas joue d une requete jamais lue, et un cas
    qui ne peut pas distinguer ces deux-la ne vaut rien. On purge donc entre
    les deux aspirations et on eprouve la fuite d etat, qui est le defaut reel.
    """
    b = r.b
    _dumpd_silence(b)
    systeme, refus = _dumpd_flux(b, _DUMPD_SYSTEME, plafond=6)
    if refus or not systeme:
        return r.record(case, 'ERROR', 'premiere aspiration impossible', f'systeme = {systeme}')
    if len(systeme) < 2:
        return r.record(case, 'ERROR',
                        'le journal systeme tient en un paquet — fuite d etat non evaluable',
                        f'systeme = {systeme}')
    mmm_systeme = systeme[-1][1]
    if not _dumpd_silence(b):
        return r.record(case, 'ERROR',
                        'le flux ne se tarit pas — seconde aspiration non isolable',
                        f'systeme = {[i for i, _ in systeme]}')
    gnss, refus2 = _dumpd_flux(b, _DUMPD_GNSS, plafond=4)
    trace = (f'systeme: {[i for i, _ in systeme]} (MMM={mmm_systeme})\n'
             f'GNSS: {gnss[:6]}')
    if refus2:
        return r.record(case, 'ERROR',
                        'le journal GNSS est refuse (vide ?) — cas non evaluable', trace)
    if not gnss:
        return r.record(case, 'ERROR', 'aucun paquet pour le journal GNSS', trace)
    if gnss[0][1] == mmm_systeme:
        return r.record(case, 'ERROR',
                        f'les deux journaux annoncent le meme MMM={mmm_systeme} — '
                        'on ne peut pas prouver qu il s agit bien du second', trace)
    if gnss[0][0] != 0:
        return r.record(case, 'FAIL',
                        f'la seconde aspiration commence a l index {gnss[0][0]}: son debut '
                        'est saute (fuite d etat entre journaux)', trace)
    r.record(case, 'PASS',
             f'la seconde aspiration repart de 0 (journal distinct, MMM={gnss[0][1]})', trace)

def c_dumpd_type_inconnu(r, case):
    """Un type de journal inexistant est refuse, pas servi au hasard.

    Le type est un index dans une table (m_logger_dump). Servir un journal
    voisin pour un index hors table donnerait les donnees d un capteur pour
    celles d un autre — pire qu une erreur, une donnee fausse indiscernable
    d une vraie.
    """
    b = r.b
    # Sans purge, un paquet du flux precedent ($O;DUMPD#...) serait pris pour
    # la reponse et le cas conclurait que le type 99 est accepte.
    _dumpd_silence(b)
    m = b.dte('DUMPD', '99', timeout=10.0)
    ligne = (m.string if m and hasattr(m, 'string') else '') or ''
    if not m:
        return r.record(case, 'ERROR', 'aucune reponse a un type inconnu')
    if m.group(1) != 'N':
        return r.record(case, 'FAIL',
                        f'le type 99 est ACCEPTE et sert des donnees: {ligne[:90]}', ligne[:200])
    r.record(case, 'PASS', 'le type de journal inconnu est refuse', ligne[:160])

CASES_V21 = [
    dict(id='DUMP-01', risque='BLOQUANT', titre='Les index de paquets se suivent de 0 a MMM',
         fn=c_dumpd_sequence),
    dict(id='DUMP-02', risque='MAJEUR',   titre='Changer de journal remet l index a zero',
         fn=c_dumpd_changement_type),
    dict(id='DUMP-03', risque='MAJEUR',   titre='Un type de journal inconnu est refuse',
         fn=c_dumpd_type_inconnu),
]


# =====================================================================
#  Vague 22 — prepasse satellite: le repli doit etre BRUYANT
#
#  Le mode prepasse n emet qu aux passages calcules du satellite. S il ne peut
#  pas les calculer — table AOP absente, perimee, ou aucun passage a portee —
#  la seule issue acceptable est de retomber sur l emission periodique EN LE
#  DISANT. Une balise qui se tait sans trace est indiscernable d une balise en
#  panne, et c est exactement le defaut deja corrige sur le masque de
#  duty-cycle vide.
# =====================================================================

def _en_config(b):
    """Garantit le mode configuration avant d envoyer une trame DTE.

    Invariant paye trois fois: les commandes DTE ne repondent QU EN MODE
    CONFIGURATION (ConfigurationState::process_usb_data). baseline() y laisse la
    carte, alors un cas qui n en sort pas fonctionne par heritage — et casse des
    qu un cas precedent en est sorti. C est ainsi que GNSSI, GNSSBR, DUMPM et
    ERASE se sont declares muets, alors que le diagnostic du 2026-08-28 les a
    vus repondre en DEUX SECONDES chacun, le pont GNSS transportant meme tout le
    flot NMEA du M10Q.

    enter_config() est idempotent cote banc (%CFG rend "already-config"), donc
    l appeler sans condition ne coute rien.
    """
    try:
        b.enter_config()
        return True
    except Exception:
        return False

def _statr(b, cles, timeout=10.0, essais=3):
    """STATR sur une liste de cles -> {cle: valeur}. Mode configuration requis."""
    for _ in range(essais):
        m = b.dte('STATR', ','.join(cles), timeout=timeout)
        ligne = (m.string if m and hasattr(m, 'string') else '') or ''
        paires = dict(re.findall(r'([A-Z]{3}\d\d)=([^,\r]*)', ligne))
        if paires:
            return paires
        time.sleep(2)
    return {}

def c_prepass_repli_bruyant(r, case):
    """Sans AOP exploitable, la prepasse retombe sur le periodique EN LE DISANT.

    Le banc n a pas de table AOP fraiche: c est donc le cas nominal du repli,
    et le seul verdict acceptable est une trace explicite. Un silence serait le
    pire des resultats — la balise ne transmettrait plus et rien ne l indiquerait
    a l operateur.
    """
    b = r.b
    try:
        b.enter_config()
        b.write_params({'ARGOS_MODE': 2, 'SAT_PREPASS_EN': 1, 'TR_NOM': 60,
                        'GNSS_EN': 1, 'NTRY_PER_MESSAGE': 0, 'DUTY_CYCLE': 0xFFFFFF,
                        'UNDERWATER_EN': 0, 'LB_EN': 0, 'RATE_LIMIT_EN': 0,
                        'ZONE_ENABLE_OUT_OF_ZONE_DETECTION_MODE': 0})
        aop = _statr(b, ['PPT01', 'PPT02'])
        b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    mk = b.mark()
    b._send('%GPS 43.6 3.9 5000 9\r')
    # Ces motifs suivent les messages du firmware, traduits en anglais en 2026-08.
    # On vise la partie stable ("prepass ...", "periodic TX") pour qu'une reformulation
    # ne casse pas silencieusement le cas.
    vues = _attendre_trace(b, [r'prepass (?:requested but|impossible)', r'periodic TX',
                               r'no pass is computable'],
                           120, depuis=mk)
    try:
        b.enter_config()
        b.write_params({'SAT_PREPASS_EN': 0, 'ARGOS_MODE': 0})
        b.exit_config()
    except Exception:
        pass
    trace = f'AOP: {aop}\n' + '\n'.join(vues[:5])
    if not vues:
        return r.record(case, 'FAIL',
                        'prepasse demandee sans AOP exploitable, et AUCUNE trace de repli '
                        'en 120 s — la balise se tait en silence', trace)
    r.record(case, 'PASS', 'le repli sur emission periodique est annonce', trace)

def c_aop_statut_coherent(r, case):
    """Le statut AOP annonce concorde avec la decision de repli.

    PPT01 (validite) et PPT02 (age) sont ce que l IHM affiche a l operateur
    avant une pose. S ils annoncent une table valide alors que le firmware
    retombe sur le periodique faute d AOP, l operateur part avec une balise
    dont il croit la prepasse operante.
    """
    b = r.b
    try:
        b.enter_config()
        aop = _statr(b, ['PPT01', 'PPT02', 'PPP08'])
        b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'lecture STATR impossible: {type(e).__name__}: {e}')
    if not aop:
        return r.record(case, 'ERROR', 'STATR ne rend pas le statut AOP')
    trace = ', '.join(f'{k}={v}' for k, v in aop.items())
    valide = aop.get('PPT01', '')
    if valide not in ('0', '1'):
        return r.record(case, 'FAIL', f'PPT01 (validite AOP) vaut {valide!r}, attendu 0 ou 1',
                        trace)
    age = aop.get('PPT02', '')
    if not age.isdigit():
        return r.record(case, 'FAIL', f'PPT02 (age AOP) vaut {age!r}, attendu un entier', trace)
    # Coherence: une table declaree valide ne peut pas etre plus vieille que la
    # limite configuree (PPP08, en jours).
    limite_j = aop.get('PPP08', '')
    if valide == '1' and limite_j.isdigit() and int(limite_j) > 0:
        if int(age) > int(limite_j) * 86400:
            return r.record(case, 'FAIL',
                            f'AOP declaree valide alors qu elle a {int(age) // 86400} jours '
                            f'pour une limite de {limite_j} jours', trace)
    r.record(case, 'PASS',
             f"statut AOP coherent (valide={valide}, age={age} s, limite={limite_j} j)", trace)

CASES_V22 = [
    dict(id='ARG-04', risque='BLOQUANT', titre='Sans AOP, le repli periodique est annonce',
         fn=c_prepass_repli_bruyant),
    dict(id='ARG-05', risque='MAJEUR',   titre='Le statut AOP annonce est coherent',
         fn=c_aop_statut_coherent),
]


# =====================================================================
#  Vague 23 — le compte a rebours vers le brick
#
#  BOOT_RETRY_BEFORE_FACTORY = 3 (gentracker.cpp:252): trois demarrages
#  consecutifs rates declenchent un reset usine, et le reset usine efface les
#  identifiants Argos. Sur une balise scellee, c est definitif — plus aucun
#  moyen de la reprogrammer sur le terrain.
#
#  Le compteur vit en .noinit et n est efface QUE par un demarrage reussi
#  (bootfail_reset(), appele depuis OperationalState::entry). S il cessait de
#  s effacer, rien ne le montrerait: le compte a rebours serait entierement
#  silencieux jusqu au troisieme redemarrage. La sonde %BOOT est le seul point
#  d observation.
# =====================================================================

def _boot(b, timeout=10.0, essais=4):
    """%BOOT -> (echecs, tentatives de reset usine), ou (None, None)."""
    for _ in range(essais):
        mk = b.mark(); b._send('%BOOT\r')
        m = b.expect(r'%BOOT failures=(\d+) factory_attempted=(\d+)', timeout, from_idx=mk)
        if m:
            return int(m.group(1)), int(m.group(2))
        time.sleep(2)
    return None, None

def c_boot_compteur_efface(r, case):
    """Un demarrage reussi remet le compteur d echecs a zero.

    C est la seule chose qui empeche le compte a rebours d avancer. La carte a
    redemarre des dizaines de fois pendant la campagne — chaque reparation de
    lien, chaque reflashage — donc si l effacement ne marchait pas, le compteur
    serait deja bien au-dela de trois et le reset usine aurait eu lieu.
    Le lire a zero apres une session entiere est une preuve de terrain, pas une
    verification de principe.
    """
    b = r.b
    echecs, usine = _boot(b)
    if echecs is None:
        return r.record(case, 'ERROR', '%BOOT sans reponse (sonde absente du build ?)')
    trace = f'failures={echecs} factory_attempted={usine}'
    if echecs > 0:
        return r.record(case, 'FAIL',
                        f'le compteur d echecs vaut {echecs} sur une carte en marche: '
                        'un demarrage reussi ne l efface pas, le reset usine approche', trace)
    if usine:
        return r.record(case, 'FAIL',
                        'une tentative de reset usine est enregistree — les identifiants '
                        'Argos ont pu etre effaces', trace)
    r.record(case, 'PASS', 'compteur d echecs a zero, aucune tentative de reset usine', trace)

def c_boot_survit_redemarrage(r, case):
    """Le compteur est TOUJOURS a zero apres un redemarrage volontaire.

    Le cas precedent constate un etat; celui-ci exerce la boucle complete.
    RSTBW redemarre la carte (DTEAction::RESET); si l effacement n avait lieu
    qu au tout premier demarrage apres flashage, ce cas le verrait.

    Le redemarrage fait re-enumerer le CDC et coupe le lien: la reconnexion
    fait partie du cas, pas d un incident.
    """
    b = r.b
    avant = _boot(b)
    try:
        b.enter_config()
        b.dte('RSTBW', '', timeout=8.0)
    except Exception as e:
        return r.record(case, 'ERROR', f'RSTBW impossible: {type(e).__name__}: {e}')
    time.sleep(6)
    if not r.connect():
        return r.record(case, 'ERROR', 'la carte ne revient pas apres RSTBW')
    b = r.b
    if not b.wait_state('OPERATIONAL', timeout=90):
        return r.record(case, 'ERROR', 'la carte ne repasse pas en OPERATIONAL apres RSTBW')
    apres = _boot(b)
    trace = f'avant: failures={avant[0]} usine={avant[1]}\napres: failures={apres[0]} usine={apres[1]}'
    if apres[0] is None:
        return r.record(case, 'ERROR', '%BOOT sans reponse apres redemarrage', trace)
    if apres[0] > 0:
        return r.record(case, 'FAIL',
                        f'apres un redemarrage volontaire le compteur vaut {apres[0]}: '
                        'le demarrage reussi ne l efface pas', trace)
    r.record(case, 'PASS',
             'le compteur revient a zero apres un redemarrage volontaire', trace)

def c_rstvw_compteur_tx(r, case):
    """RSTVW remet le compteur d emissions a zero, et refuse un index inconnu.

    Le compteur d emissions sert au suivi de consommation du quota satellite.
    Un RSTVW qui accepterait n importe quel index ecraserait une autre variable
    de comptage sans le dire.
    """
    b = r.b
    try:
        b.enter_config()
        m = b.dte('RSTVW', '1', timeout=10.0)
        apres = _statr(b, ['ART02'])
        mauvais = b.dte('RSTVW', '9', timeout=10.0)
        b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'RSTVW impossible: {type(e).__name__}: {e}')
    trace = f'apres RSTVW 1: {apres}\nRSTVW 9: {(mauvais.string if mauvais and hasattr(mauvais, "string") else "")[:80]}'
    if not m or m.group(1) != 'O':
        return r.record(case, 'FAIL', 'RSTVW 1 (compteur d emissions) refuse', trace)
    valeur = apres.get('ART02', '')
    if valeur.isdigit() and int(valeur) != 0:
        return r.record(case, 'FAIL',
                        f'le compteur d emissions vaut {valeur} apres remise a zero', trace)
    if not mauvais:
        return r.record(case, 'ERROR', 'aucune reponse a un index inconnu', trace)
    if mauvais.group(1) != 'N':
        return r.record(case, 'FAIL', 'RSTVW accepte un index inconnu (9)', trace)
    r.record(case, 'PASS',
             'le compteur d emissions est remis a zero, l index inconnu est refuse', trace)

CASES_V23 = [
    dict(id='BOOT-01', risque='BLOQUANT', titre='Un demarrage reussi efface le compteur d echecs',
         fn=c_boot_compteur_efface),
    dict(id='BOOT-02', risque='BLOQUANT', titre='Le compteur reste a zero apres un redemarrage',
         fn=c_boot_survit_redemarrage),
    dict(id='RSTV-01', risque='MAJEUR',   titre='RSTVW remet a zero et refuse un index inconnu',
         fn=c_rstvw_compteur_tx),
]


# =====================================================================
#  Vague 24 — integration PREPASS v4.0 / reception AOP Kineis
#
#  Trois choses a eprouver sur carte, qu aucun test hote ne peut couvrir:
#  que les nouveaux parametres existent vraiment dans le binaire flashe, que
#  le codec AOP refuse l ancien format SANS abimer la table stockee, et que la
#  reception ne s arme pas quand elle ne doit pas.
# =====================================================================

# Capture reelle CLS retrieve-kineis-aop du 2026-08-27T18:31 UTC, 478 octets,
# sha256 fcf7b67e...c332b1 — la meme que le test hote.
_AOP_KINEIS_HEX = 'tests/data/kineis_aop_20260827.hex'

def c_prepass_params_presents(r, case):
    """Les trois parametres PREPASS v4.0 existent et sont bornes.

    PPP10/11/12 remplacent des valeurs qui etaient en dur dans le code. Un
    binaire flashe sans eux accepterait les ecritures en silence (cle inconnue
    -> rejet nomme) ou, pire, les rejetterait sans qu on s en apercoive: la
    balise resterait sur les valeurs d usine sans le dire.
    """
    b = r.b
    defauts = []
    try:
        b.write_params({'PP_MIN_CULMINATION': 10, 'PP_RX_MIN_CULMINATION': 25,
                        'PP_POSITION_MARGIN_KM': 5})
        _, lus = b.read_params(['PP_MIN_CULMINATION', 'PP_RX_MIN_CULMINATION',
                                'PP_POSITION_MARGIN_KM'])
    except Exception as e:
        return r.record(case, 'ERROR', f'ecriture/lecture impossible: {type(e).__name__}: {e}')
    attendu = {'PPP10': '10', 'PPP11': '25', 'PPP12': '5'}
    for cle, val in attendu.items():
        if lus.get(cle) != val:
            defauts.append(f'{cle}={lus.get(cle)} au lieu de {val}')
    # Bornes: la culmination est un angle, 91 n a pas de sens.
    for cle, hors in (('PP_MIN_CULMINATION', 91), ('PP_RX_MIN_CULMINATION', 91),
                      ('PP_POSITION_MARGIN_KM', 101)):
        try:
            b.write_params({cle: hors}, strict=False)
            _, apres = b.read_params([cle])
            k = b._key(cle)
            if apres.get(k) == str(hors):
                defauts.append(f'{cle} accepte la valeur hors borne {hors}')
        except Exception:
            pass
    try:
        b.write_params({'PP_MIN_CULMINATION': 0, 'PP_RX_MIN_CULMINATION': 20,
                        'PP_POSITION_MARGIN_KM': 0})
    except Exception:
        pass
    trace = f'relus: {lus}'
    if defauts:
        return r.record(case, 'FAIL', '; '.join(defauts), trace)
    r.record(case, 'PASS', 'PPP10/11/12 presents, ecrits, relus et bornes', trace)

def c_aop_kineis_accepte(r, case):
    """Une trame AOP tronquee par le transport est refusee, pas ecrite a moitie.

    CE CAS NE PEUT PAS TELEVERSER L AOP PAR USB, et c est une limite du banc,
    pas un defaut du firmware. Mesure du 2026-08-28:

      - NrfUSB::read_line() coupe sur '\r' OU '\n' (nrf_usb.cpp:232). La
        capture Kineis contient deux octets 0x0A, donc la trame arrive tronquee
        et le firmware repond $N;PASPW#001;4 (DATA_LENGTH_MISMATCH).
      - Le chemin BLE, lui, ne vide son tampon que si le DERNIER octet d un
        paquet BLE vaut '\r' (ble_interface.cpp:700). Un 0x0D a l interieur de
        la charge ne declenche rien: le televersement binaire par BLE — celui
        de l IHM — fonctionne.

    Le decodage lui-meme est prouve cote hote par PASPW_REQ_KineisAllcastAop,
    sur cette meme capture, avec plus de soixante assertions.

    Ce qui reste a eprouver sur la carte, et qui vaut d etre eprouve: qu une
    trame tronquee soit refusee PROPREMENT et laisse la table intacte. Une
    troncature acceptee a moitie ecrirait des elements orbitaux partiels, et la
    balise viserait des satellites qui ne sont pas la.
    """
    b = r.b
    import os
    chemin = os.path.join(os.path.dirname(__file__), '..', '..', _AOP_KINEIS_HEX)
    try:
        with open(os.path.normpath(chemin)) as f:
            binaire = bytes.fromhex(f.read().strip())
    except OSError as e:
        return r.record(case, 'ERROR', f'vecteur introuvable: {e}')
    if b'\n' not in binaire and b'\r' not in binaire:
        return r.record(case, 'ERROR',
                        'la capture ne contient ni LF ni CR: elle passerait par USB et ce '
                        'cas ne mesure plus la troncature — le remplacer par un vrai '
                        'televersement')

    avant = _statr(b, ['PPT01', 'ART03'])
    trame = f'$PASPW#{len(binaire):03X};'.encode() + binaire + b'\r'
    mk = b.mark()
    try:
        b.ser.write(trame); b.ser.flush()
    except Exception as e:
        return r.record(case, 'ERROR', f'envoi impossible: {type(e).__name__}: {e}')
    m = b.expect(r'\$([ON]);PASPW#([0-9A-F]{3});(\d*)', timeout=25.0, from_idx=mk)
    ligne = (m.string if m and hasattr(m, 'string') else '') or ''
    apres = _statr(b, ['PPT01', 'ART03'])
    trace = (f'{len(binaire)} octets, {binaire.count(10)} LF, {binaire.count(13)} CR\n'
             f'reponse: {ligne[:70]}\navant={avant}\napres={apres}')
    if not m:
        return r.record(case, 'ERROR', 'PASPW sans reponse', trace)
    if m.group(1) == 'O':
        return r.record(case, 'FAIL',
                        'une trame TRONQUEE par le transport est ACCEPTEE — la table a pu '
                        'etre ecrite partiellement', trace)
    code = m.group(3)
    if code != '4':
        return r.record(case, 'FAIL',
                        f'trame tronquee refusee avec l erreur {code}, attendu 4 '
                        '(DATA_LENGTH_MISMATCH)', trace)
    if avant.get('ART03') and apres.get('ART03') != avant.get('ART03'):
        return r.record(case, 'FAIL',
                        f"refus annonce mais la table a bouge: ART03 {avant.get('ART03')} "
                        f"-> {apres.get('ART03')}", trace)
    r.record(case, 'PASS',
             'trame tronquee par le transport USB refusee en DATA_LENGTH_MISMATCH, '
             'table intacte', trace)

def c_aop_ancien_format_refuse(r, case):
    """L ancien format A-DCS est refuse SANS abimer la table stockee.

    Ce qui compte n est pas l echec mais sa proprete: accepter a moitie une
    trame illisible ecrirait des elements orbitaux faux, et la balise viserait
    des satellites absents — panne silencieuse et durable, puisque plus rien ne
    la corrigerait avant le prochain PASPW.
    """
    b = r.b
    avant = _statr(b, ['PPT01', 'ART03'])
    # Adresse allcast A-DCS (0x0BE5) suivie de remplissage: la forme suffit,
    # le codec doit la rejeter sur l adresse.
    faux = bytes.fromhex('00000BE5') + bytes(20)
    trame = f'$PASPW#{len(faux):03X};'.encode() + faux + b'\r'
    mk = b.mark()
    try:
        b.ser.write(trame); b.ser.flush()
    except Exception as e:
        return r.record(case, 'ERROR', f'envoi impossible: {type(e).__name__}: {e}')
    m = b.expect(r'\$([ON]);PASPW#', timeout=20.0, from_idx=mk)
    apres = _statr(b, ['PPT01', 'ART03'])
    trace = f'avant={avant}\napres={apres}'
    if not m:
        return r.record(case, 'ERROR', 'PASPW sans reponse sur l ancien format', trace)
    if m.group(1) != 'N':
        return r.record(case, 'FAIL',
                        'une trame A-DCS est ACCEPTEE — la table a pu etre ecrasee', trace)
    if avant.get('ART03') and apres.get('ART03') != avant.get('ART03'):
        return r.record(case, 'FAIL',
                        f"refus annonce mais la table a bouge: ART03 {avant.get('ART03')} "
                        f"-> {apres.get('ART03')}", trace)
    r.record(case, 'PASS', 'ancien format refuse, table stockee intacte', trace)

def c_rx_gate_batterie(r, case):
    """La reception AOP ne s arme pas en batterie faible.

    Une fenetre de reception coute jusqu a ARGOS_RX_MAX_WINDOW (15 min par
    defaut) de recepteur alimente. Le profil batterie faible existe pour en
    faire MOINS. La protection etait accidentelle — elle tenait au fait que
    LB_ARGOS_MODE vaut LEGACY par defaut — et un operateur choisissant
    PASS_PREDICTION en batterie faible la faisait disparaitre.

    On observe l ordonnancement du service, seul temoin accessible au banc.
    """
    if saute_si_radio_muette(r, case, 'le dialogue avec le module'):
        return
    b = r.b
    # La reception Argos n existe que sur KIM2. SmdSat::start_receive et
    # stop_receive repondent "Not supported", et main() n instancie
    # ArgosRxService que dans la branche KIM2 — il n y a donc aucun service a
    # observer sur un build SMD, et %SCHED a raison de ne rien rapporter.
    # Mesure du 2026-08-30, ou le cas concluait "sonde ou service absent".
    if any(nom == 'KIMBR' for nom, _ in _commandes_absentes(b)):
        return r.record(case, 'SKIP',
                        'ce build ne porte pas la reception Argos (SmdSat: "Not supported", '
                        'ArgosRxService instancie seulement en KIM2) — rien a observer ici')
    try:
        b.enter_config()
        b.write_params({'ARGOS_RX_EN': 1, 'ARGOS_MODE': 1, 'LB_EN': 1,
                        'LB_ARGOS_MODE': 1, 'CERT_TX_ENABLE': 0})
        b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    time.sleep(4)
    m, _ = r.raw_until('%SCHED\r', r'%SCHED ', timeout=20.0)
    ligne = (m.string if m and hasattr(m, 'string') else '') or ''
    mm = re.search(r'ARGOSRX=(none|hold\d+s|\d+ms)\(([^)]*)\)', ligne)
    try:
        b.enter_config()
        b.write_params({'LB_EN': 0, 'LB_ARGOS_MODE': 2, 'ARGOS_MODE': 0})
        b.exit_config()
    except Exception:
        pass
    if not mm:
        return r.record(case, 'ERROR',
                        '%SCHED ne rapporte pas le service ARGOSRX — sonde ou service absent',
                        ligne[:200])
    quand, raison = mm.group(1), mm.group(2)
    trace = ligne[:220]
    # La batterie du banc est pleine: LB_EN=1 seul ne declenche pas le profil.
    # Ce cas est donc un GARDE de configuration, pas une preuve de terrain — il
    # verifie qu on n arme pas la reception hors PASS_PREDICTION reel.
    if _est_planifie(quand) and 'not-enabled' not in raison and 'stopped' not in raison:
        return r.record(case, 'ERROR',
                        f'la carte est sur batterie pleine (LB inactif): ARGOSRX={quand} '
                        f'({raison}) — cas non concluant sans alimentation pilotable',
                        trace)
    r.record(case, 'PASS', f'reception non armee (raison: {raison})', trace)

CASES_V24 = [
    dict(id='PP-01',  risque='MAJEUR',   titre='PPP10/11/12 presents, ecrits, relus et bornes',
         fn=c_prepass_params_presents),
    dict(id='AOP-01', risque='BLOQUANT', titre='Une trame AOP tronquee est refusee sans abimer la table',
         fn=c_aop_kineis_accepte),
    dict(id='AOP-02', risque='BLOQUANT', titre='L ancien format est refuse sans abimer la table',
         fn=c_aop_ancien_format_refuse),
    dict(id='RX-01',  risque='MAJEUR',   titre='La reception AOP ne s arme pas sans raison',
         fn=c_rx_gate_batterie),
]


# =====================================================================
#  Vague 25 — le reste du port DTE: recuperation et action
#
#  Inventaire du 2026-08-27: sur 37 commandes DTE, 19 etaient couvertes. Cette
#  vague prend celles qui restent et qui ont un sens sur cette carte.
#
#  DELIBEREMENT ABSENTES, et pour des raisons differentes:
#    FACTW           efface les identifiants Argos. Definitif sur balise
#                    scellee. Ne doit pas etre ecrit sans carte sacrifiable.
#    LORATX, LORABR  LORA_RAK3172 est OFF dans le build KIM2: le module n est
#                    pas compile. A couvrir sur la variante LoRa, pas ici.
#    SMDDFU, SMDTST  module SMD absent de cette carte.
#  Les ponts serie (KIMBR, GNSSBR) SONT couverts: un pont se prouve par un
#  aller-retour reel — une commande part, la reponse du module revient, et +++
#  rend le canal DTE. Un pont qui ne transporte rien est la panne qu on veut
#  voir avant d envoyer quelqu un diagnostiquer une balise sur le terrain.
# =====================================================================

def c_profil_aller_retour(r, case):
    """PROFW ecrit le nom de profil, PROFR le relit.

    C est ce qui identifie une balise dans les exports du segment sol. Un nom
    perdu ou tronque rend un jeu de donnees anonyme, et personne ne s en apercoit
    avant le depouillement.
    """
    b = r.b
    nom = 'BENCH-2026-08'
    m = b.dte('PROFW', nom, timeout=10.0)
    if not m:
        return r.record(case, 'ERROR', 'PROFW sans reponse')
    if m.group(1) != 'O':
        ligne = m.string if hasattr(m, 'string') else ''
        return r.record(case, 'FAIL', f'PROFW refuse un nom valide: {ligne[:70]}')
    m2 = b.dte('PROFR', '', timeout=10.0)
    ligne = (m2.string if m2 and hasattr(m2, 'string') else '') or ''
    if not m2:
        return r.record(case, 'ERROR', 'PROFR sans reponse')
    if nom not in ligne:
        return r.record(case, 'FAIL', f'PROFR ne rend pas {nom!r}: {ligne[:90]}', ligne[:200])
    r.record(case, 'PASS', f'nom de profil ecrit et relu ({nom})', ligne[:120])

def c_gnss_info(r, case):
    """GNSSI rend l identite du recepteur.

    Sert au diagnostic avant pose: un recepteur qui ne repond pas a GNSSI ne
    donnera pas de position non plus, et il vaut mieux le savoir sur la paillasse
    que sur l animal. La commande allume le rail GNSS, la reponse peut donc
    tarder.
    """
    b = r.b
    if not _en_config(b):
        return r.record(case, 'ERROR', 'mode configuration inaccessible')
    m = b.dte('GNSSI', '', timeout=30.0)
    ligne = (m.string if m and hasattr(m, 'string') else '') or ''
    if not m:
        return r.record(case, 'ERROR', 'GNSSI sans reponse en 30 s')
    if m.group(1) != 'O':
        return r.record(case, 'FAIL', f'GNSSI refuse — recepteur absent ou muet: {ligne[:80]}',
                        ligne[:200])
    corps = ligne.split(';', 2)[-1].strip()
    if len(corps) < 4:
        return r.record(case, 'FAIL', f'GNSSI rend une identite vide: {ligne[:90]}', ligne[:200])
    r.record(case, 'PASS', f'identite recepteur rendue ({corps[:40]})', ligne[:200])

def c_gnss_almanach(r, case):
    """GNSSA rend l etat de l almanach.

    L almanach conditionne le temps de premier fix. C est la contrepartie
    mesurable de la mort GNSS a deux jours: sans almanach frais, chaque
    acquisition repart de zero.
    """
    b = r.b
    if not _en_config(b):
        return r.record(case, 'ERROR', 'mode configuration inaccessible')
    m = b.dte('GNSSA', '', timeout=30.0)
    ligne = (m.string if m and hasattr(m, 'string') else '') or ''
    if not m:
        return r.record(case, 'ERROR', 'GNSSA sans reponse en 30 s')
    if m.group(1) != 'O':
        return r.record(case, 'FAIL', f'GNSSA refuse: {ligne[:80]}', ligne[:200])
    r.record(case, 'PASS', 'etat d almanach rendu', ligne[:200])

def c_satvf_borne(r, case):
    """SATVF accepte 0/1 et refuse le reste.

    La verification satellite force une emission. Un argument hors borne accepte
    declencherait un comportement non specifie sur un emetteur — c est le genre
    de laxisme qui se paie en credit satellite.
    """
    if saute_si_radio_muette(r, case, 'le dialogue avec le module'):
        return
    b = r.b
    # SATVF verifie les credentials satellite: sans eux il refuse, a juste
    # titre. Sur une carte non provisionnee le cas ne peut donc pas eprouver
    # ses bornes — mesure du 2026-08-30, RADIOCONF et SECKEY vides (seuls
    # DECID/HEXID avaient survecu a une remise a zero de config), et les deux
    # arguments VALIDES etaient refuses en $N;...;5.
    try:
        if not _en_config(b):
            return r.record(case, 'ERROR', 'mode configuration inaccessible')
        _, creds = b.read_params(['ARGOS_RADIOCONF'], timeout=12.0)
        if len((creds.get('IDP14') or '').strip()) != 32:
            return r.record(case, 'SKIP',
                            'carte non provisionnee (ARGOS_RADIOCONF vide): SATVF refuse tout '
                            'argument, y compris les valides — bornes ineprouvables ici')
    except Exception as e:
        return r.record(case, 'ERROR', f'lecture des credentials impossible: {type(e).__name__}: {e}')

    defauts = []
    for val in ('0', '1'):
        m = b.dte('SATVF', val, timeout=25.0)
        if not m:
            defauts.append(f'SATVF {val}: aucune reponse')
        elif m.group(1) != 'O':
            ligne = m.string if hasattr(m, 'string') else ''
            defauts.append(f'SATVF {val} refuse: {ligne[:50]}')
        time.sleep(2)
    m = b.dte('SATVF', '2', timeout=15.0)
    if not m:
        defauts.append('SATVF 2 (hors borne): aucune reponse')
    elif m.group(1) != 'N':
        defauts.append('SATVF 2 hors borne ACCEPTE')
    if defauts:
        return r.record(case, 'FAIL', '; '.join(defauts))
    r.record(case, 'PASS', 'SATVF accepte 0 et 1, refuse 2')

def c_secur_code(r, case):
    """SECUR refuse un code d acces faux.

    C est la seule barriere entre un tiers et la configuration d une balise
    posee. Un SECUR qui accepterait n importe quoi la supprimerait sans bruit.
    """
    b = r.b
    m = b.dte('SECUR', 'DEADBEEF', timeout=10.0)
    ligne = (m.string if m and hasattr(m, 'string') else '') or ''
    if not m:
        return r.record(case, 'ERROR', 'SECUR sans reponse')
    if m.group(1) != 'N':
        return r.record(case, 'FAIL',
                        f'un code d acces arbitraire est ACCEPTE: {ligne[:80]}', ligne[:200])
    r.record(case, 'PASS', 'code d acces faux refuse', ligne[:120])

def c_dumpm_memoire(r, case):
    """DUMPM repond, ou refuse proprement si l acces memoire n est pas cable.

    Commande de diagnostic bas niveau. Ce qui compte est qu elle ne laisse pas la
    console sans reponse: un port DTE muet sur une commande valide est le defaut
    le plus difficile a diagnostiquer sur le terrain.
    """
    b = r.b
    if not _en_config(b):
        return r.record(case, 'ERROR', 'mode configuration inaccessible')
    # 0x20000000 = debut de la RAM: la seule fenetre que DUMPM accepte.
    # Une adresse hors RAM doit etre REFUSEE, pas ignoree — c est le defaut
    # corrige le 2026-08-28, ou le port se taisait sur une commande bien formee.
    m = b.dte('DUMPM', '20000000,10', timeout=15.0)
    ligne = (m.string if m and hasattr(m, 'string') else '') or ''
    if not m:
        return r.record(case, 'FAIL',
                        'DUMPM reste SANS REPONSE — la console est muette sur une commande valide')
    verdict = 'acceptee' if m.group(1) == 'O' else 'refusee proprement'
    r.record(case, 'PASS', f'DUMPM {verdict}', ligne[:160])

def c_erase_journal(r, case):
    """ERASE vide reellement le journal demande, et refuse un type inconnu.

    C est le geste d avant-pose: repartir sur un journal propre. S il ne vidait
    pas, la balise partirait avec la memoire d une autre mission et le
    depouillement melangerait deux jeux de donnees.

    A JOUER EN DERNIER: les autres cas lisent le journal systeme.
    """
    b = r.b
    if not _en_config(b):
        return r.record(case, 'ERROR', 'mode configuration inaccessible')
    m = b.dte('ERASE', '99', timeout=15.0)
    if not m:
        return r.record(case, 'ERROR', 'ERASE sans reponse sur type inconnu')
    if m.group(1) != 'N':
        return r.record(case, 'FAIL', 'ERASE accepte un type de journal inconnu (99)')
    # BaseEraseType: 1 = GNSS_SENSOR, 2 = SYSTEM, 3 = ALL. Le type 2 est donc
    # bien le journal SYSTEME — d ou l obligation de jouer ce cas en dernier,
    # puisque tous les autres lisent ses traces. Le commentaire precedent
    # disait "journal capteurs", ce qui etait faux dans les deux sens.
    _dumpd_silence(b)
    avant, _, _, _ = _dumpd(b, 1, plafond=4)
    # 120 s, pas 25: truncate() est SYNCHRONE et efface un journal d 1 Mo secteur
    # par secteur sur la QSPI. A 300 ms d effacement par secteur dans le pire cas
    # (voir le defaut IS25 de 2026-08), l ordre de grandeur est la minute. Le
    # port ne repond pas pendant ce temps: ce n est pas une panne, mais c est
    # une propriete qu il faut mesurer plutot que supposer, donc on chronometre.
    t0 = time.time()
    m2 = b.dte('ERASE', '2', timeout=120.0)
    duree = time.time() - t0
    ligne = (m2.string if m2 and hasattr(m2, 'string') else '') or ''
    if not m2:
        return r.record(case, 'ERROR',
                        f'ERASE sans reponse sur un type valide apres {duree:.0f} s')
    if m2.group(1) != 'O':
        return r.record(case, 'FAIL', f'ERASE refuse un type valide: {ligne[:70]}', ligne[:200])
    r.record(case, 'PASS', f'type inconnu refuse, effacement accepte en {duree:.1f} s',
             f'journal GNSS avant: {avant[:6]}')

def c_ordonnanceur_complet(r, case):
    """Chaque service actif annonce une raison d ordonnancement lisible.

    Un service qui ne rend ni echeance ni raison est un service dont personne ne
    sait s il tourne. C est exactement l etat "vivant mais inerte" observe sur le
    terrain — la balise repond, le WDT est nourri, et pourtant rien n avance.
    """
    b = r.b
    m, _ = r.raw_until('%SCHED\r', r'%SCHED ', timeout=20.0)
    ligne = (m.string if m and hasattr(m, 'string') else '') or ''
    if not m:
        return r.record(case, 'ERROR', '%SCHED sans reponse')
    services = re.findall(r'(\w+)=(none|hold\d+s|\d+ms)\(([^)]*)\)', ligne)
    if not services:
        return r.record(case, 'FAIL', f'%SCHED ne rend aucun service: {ligne[:120]}', ligne[:250])
    muets = [nom for nom, _, raison in services if not raison.strip()]
    trace = '\n'.join(f'  {n} = {q} ({raison})' for n, q, raison in services)
    if muets:
        return r.record(case, 'FAIL',
                        'services sans raison d ordonnancement: ' + ', '.join(muets), trace)
    r.record(case, 'PASS', f'{len(services)} services annoncent tous une raison', trace)

def c_pont_gnss(r, case):
    """Le pont GNSS transporte reellement le trafic du recepteur.

    C est l outil de diagnostic de dernier recours: quand une balise ne fixe
    plus, le pont donne un acces direct au M10Q. S il ne transporte rien, on
    perd le seul moyen d interroger le recepteur sans demonter la balise.

    Le pont REFUSE en veille profonde (m10qasync.cpp: l UART y est deinitialise
    et ses canaux PPI liberes). On sort donc explicitement le GNSS de veille
    avant d ouvrir, sinon le cas mesurerait le refus au lieu du transport.
    """
    b = r.b
    try:
        b.enter_config()
        b.write_params({'GNSS_EN': 1, 'GNSS_DEEP_IDLE_AFTER_OFF_S': 0})
        b.exit_config()
        time.sleep(6)
        # RETOUR en configuration: $GNSSBR est une trame DTE, et le DTE ne
        # repond qu en configuration. La version precedente l envoyait apres la
        # sortie, donc la carte ne la lisait jamais — et le cas concluait a un
        # pont refuse alors qu il n avait simplement pas ete demande.
        b.enter_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')

    lines, err = r.raw("$GNSSBR#001;1\r", wait=4.0)
    if err:
        return r.record(case, 'ERROR', err)
    m = _resp(lines, 'GNSSBR')
    refus_veille = any('deep-idle' in l for l in lines)
    if not m or m.group(1) != 'O':
        if refus_veille:
            return r.record(case, 'ERROR',
                            'pont refuse: GNSS en veille profonde — cas non concluant',
                            '\n'.join(lines[:5]))
        return r.record(case, 'FAIL',
                        f'ouverture du pont refusee: {m.group(0) if m else "<silence>"}',
                        '\n'.join(lines[:5]))

    # Sonde UBX-MON-VER (classe 0x0A, id 0x04, longueur 0, somme 0x0E 0x34).
    # Le recepteur repond par une trame MON-VER; on ne la decode pas, on veut
    # seulement prouver que des octets FONT LE TRAJET.
    mk = b.mark()
    try:
        b.ser.write(bytes.fromhex('B5620A0400000E34'))
        b.ser.flush()
    except Exception as e:
        return r.record(case, 'ERROR', f'ecriture sur le pont impossible: {type(e).__name__}: {e}')
    time.sleep(6)
    with b._lock:
        recu = [l for _, l in b.history[mk:]]

    # Sortie du pont, puis verification que le canal DTE revient.
    r.raw("+++\r", wait=3.0)
    l5, _ = r.raw("$PARML#000;\r", wait=3.0)
    canal_rendu = _resp(l5, 'PARML') is not None
    try:
        b.enter_config(); b.write_params({'GNSS_EN': 0}); b.exit_config()
    except Exception:
        pass

    trace = f'{len(recu)} lignes recues a travers le pont\n' + '\n'.join(recu[:5])
    if not canal_rendu:
        return r.record(case, 'FAIL',
                        'CANAL DTE NON RENDU apres +++ — la balise reste prisonniere du pont',
                        trace)
    if not recu:
        return r.record(case, 'FAIL',
                        'le pont est ouvert mais AUCUN octet ne remonte du recepteur — '
                        'il ne transporte rien', trace)
    r.record(case, 'PASS',
             f'trafic recepteur transporte ({len(recu)} lignes), canal rendu par +++', trace)

CASES_V25 = [
    dict(id='PROF-01', risque='MAJEUR',   titre='Le nom de profil est ecrit et relu',
         fn=c_profil_aller_retour),
    dict(id='SEC-01',  risque='BLOQUANT', titre='SECUR refuse un code d acces faux',
         fn=c_secur_code),
    dict(id='GNSS-I1', risque='MAJEUR',   titre='GNSSI rend l identite du recepteur',
         fn=c_gnss_info),
    dict(id='GNSS-A1', risque='MAJEUR',   titre='GNSSA rend l etat de l almanach',
         fn=c_gnss_almanach),
    dict(id='SAT-V1',  risque='MAJEUR',   titre='SATVF accepte 0/1 et refuse le reste',
         fn=c_satvf_borne),
    dict(id='MEM-01',  risque='MINEUR',   titre='DUMPM repond au lieu de rester muet',
         fn=c_dumpm_memoire),
    dict(id='BRDG-01', risque='MAJEUR',   titre='Le pont GNSS transporte le trafic du recepteur',
         fn=c_pont_gnss),
    dict(id='SCH-02',  risque='BLOQUANT', titre='Chaque service annonce une raison d ordonnancement',
         fn=c_ordonnanceur_complet),
    # En dernier: efface un journal.
    dict(id='ERAS-01', risque='MAJEUR',   titre='ERASE vide le journal et refuse un type inconnu',
         fn=c_erase_journal),
]


# =====================================================================
#  Vague 26 — PLEIN AIR: ce qu aucune injection ne remplace
#
#  A LANCER ANTENNE VERS LE CIEL. Ces cas ne s executent pas sur la paillasse
#  et se declarent non concluants plutot que rouges s ils ne voient pas de fix.
#
#  Pourquoi ils existent: %GPS injecte la position DIRECTEMENT dans GPSService
#  et court-circuite le pilote M10Q. Or les filtres hAcc et HDOP vivent dans
#  m10qasync.cpp, sur la trame NAV-PVT REELLE (lignes 1061 et 1085). Ils n ont
#  donc JAMAIS ete eprouves — ni au banc, ni en simulation. Un filtre trop
#  strict rejette toutes les positions et la balise se tait sans qu aucune
#  erreur ne soit tracee; un filtre inoperant laisse passer des positions
#  fausses que le segment sol prendra pour vraies. Les deux se paient
#  entierement sur le terrain.
# =====================================================================

def _config_ciel(b, **extra):
    """Configuration d acquisition reelle: aucune injection, aucune emission."""
    # PLAFOND D ACQUISITION AU MAXIMUM (600 = borne DTE). Cette carte perd sa
    # BBR a chaque coupure du rail — le firmware le prevoit ("no cell": le
    # fichier en flash devient le seul vecteur de demarrage a chaud) — donc
    # CHAQUE session est un demarrage a froid absolu. Un froid sans almanach
    # peut demander jusqu a 12,5 min rien que pour le telecharger; 180 s ou
    # 300 s condamnaient la vague exterieure a echouer quel que soit le ciel.
    # CADENCE D ACQUISITION — l oubli le plus couteux de ce harnais. Aucun des
    # deux helpers de ciel ne la remettait, et DP-01 laisse derriere lui
    # DLOC_ARG_NOM=10, qui dans l espace DTE brut vaut 1440 min = 24 HEURES.
    # Tout cas exterieur attendait donc une session programmee le lendemain et
    # concluait "antenne masquee". Mesure du 2026-08-29: OUT-01 n a pas meme vu
    # la ligne "M10Q on" — le recepteur n avait jamais ete allume. Code 13 = 5 min.
    #
    # GNSS_NTRY et l assistance pour la meme raison: GPS-04 epuise les
    # tentatives et GPS-06 force un demarrage a froid (effacement de la BBR du
    # recepteur). Sans remise a zero, un cas exterieur herite d un service en
    # repli et d une acquisition qui repart de zero — ephemeride, almanach,
    # position, heure.
    cfg = {'GNSS_EN': 1, 'ARGOS_MODE': 0, 'UNDERWATER_EN': 0, 'LB_EN': 0,
           'RATE_LIMIT_EN': 0, 'MOORED_DETECT_EN': 0, 'HAULED_DETECT_EN': 0,
           'ZONE_ENABLE_OUT_OF_ZONE_DETECTION_MODE': 0,
           'DLOC_ARG_NOM': 13, 'GNSS_NTRY': 5,
           'GNSS_ASSISTNOW_EN': 1, 'GNSS_ASSISTNOW_OFFLINE_EN': 1,
           'GNSS_ACQ_TIMEOUT': 600, 'GNSS_COLD_ACQ_TIMEOUT': 600,
           'GNSS_HACCFILT_EN': 0, 'GNSS_HDOPFILT_EN': 0,
           'GNSS_DEEP_IDLE_AFTER_OFF_S': 0, 'GNSS_SESSION_SINGLE_FIX': 1}
    cfg.update(extra)
    b.enter_config(); b.write_params(cfg); b.exit_config()

def _attend_fix_reel(b, secondes=330, depuis=None):
    """Attend une VRAIE position. Rend (lignes, hAcc_mm, numSV) ou (lignes, None, None).

    On guette la trace de traitement du pilote, pas l injecting banc: c est
    la seule qui prouve qu une trame NAV-PVT est reellement arrivee.
    """
    motifs = [r'task_process_gnss_data: lat=', r'timeout with degraded PVT',
              r'acquisition timeout — no fix']
    vues = _attendre_trace(b, motifs, secondes, depuis=depuis)
    for l in vues:
        m = re.search(r'hAcc=([\d.]+)', l)
        n = re.search(r'numSV=(\d+)', l)
        if m and 'task_process_gnss_data' in l:
            return vues, float(m.group(1)), int(n.group(1)) if n else 0
    return vues, None, None

def c_ciel_premier_fix(r, case):
    """Une vraie position tombe, avec une precision et un nombre de satellites plausibles.

    C est LE cas qui prouve que le recepteur fonctionne. Tout le reste de la
    campagne GNSS a ete valide par injection: rien n avait encore montre qu une
    trame NAV-PVT reelle traverse le pilote jusqu au journal.
    """
    b = r.b
    try:
        _config_ciel(b)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    mk = b.mark()
    vues, hacc, numsv = _attend_fix_reel(b, 330, depuis=mk)
    trace = '\n'.join(vues[:6])
    if hacc is None:
        degrade = any('degraded PVT' in l for l in vues)
        # Ne pas accuser le ciel quand on ne peut pas trancher: "le recepteur a
        # cherche sans rien trouver" est une question de vue, "le recepteur ne
        # s est jamais allume" est une question de configuration, et les deux
        # envoient chercher le probleme a des endroits opposes.
        allume = any('M10Q on' in l for l in vues)
        if not allume:
            return r.record(case, 'ERROR',
                            'aucune session GNSS en 5 min: le recepteur ne s est PAS allume '
                            '(cadence d acquisition, ou service desactive) — ce n est pas une '
                            'question de ciel', trace)
        return r.record(case, 'ERROR',
                        'le recepteur a cherche sans aboutir en 5 min — vue du ciel, ou '
                        'demarrage a froid recent'
                        + (' (fixes degrades seulement)' if degrade else ''), trace)
    defauts = []
    # 50 m: au-dela, ce n est pas un fix utilisable pour du suivi animal.
    if hacc > 50000:
        defauts.append(f'hAcc={hacc/1000:.1f} m — precision inexploitable')
    if numsv < 4:
        defauts.append(f'numSV={numsv} — un fix 3D en demande au moins 4')
    if defauts:
        return r.record(case, 'FAIL', '; '.join(defauts), trace)
    r.record(case, 'PASS', f'position reelle: hAcc={hacc/1000:.1f} m, {numsv} satellites', trace)

def c_ciel_filtre_hacc(r, case):
    """GNP20/21 rejette reellement une position trop imprecise.

    Le chemin n a jamais ete couvert: il est dans le pilote, sur la trame reelle.
    On serre le seuil sous ce que le ciel donne — la position doit alors etre
    rejetee et la session se terminer sur un PVT degrade. Puis on desserre, et
    la meme position doit passer. Les deux sens comptent: un filtre qui rejette
    tout et un filtre inoperant echouent chacun a une moitie du test.
    """
    b = r.b
    # 1. Reference: quelle precision le ciel donne-t-il ici ?
    try:
        _config_ciel(b)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    mk = b.mark()
    _, hacc, _ = _attend_fix_reel(b, 330, depuis=mk)
    if hacc is None:
        return r.record(case, 'ERROR', 'pas de position de reference — cas non evaluable')

    # 2. Seuil impossible: 1 m alors que le ciel donne hacc. GNP21 est en METRES
    #    (le pilote compare threshold * 1000 aux mm).
    strict = 1
    if hacc / 1000.0 <= 2.0:
        return r.record(case, 'ERROR',
                        f'ciel trop bon (hAcc={hacc/1000:.1f} m): aucun seuil strict '
                        'ne peut le rejeter — cas non concluant')
    try:
        _config_ciel(b, GNSS_HACCFILT_EN=1, GNSS_HACCFILT_THR=strict)
    except Exception as e:
        return r.record(case, 'ERROR', f'reconfiguration impossible: {type(e).__name__}: {e}')
    mk2 = b.mark()
    vues_strict, hacc_strict, _ = _attend_fix_reel(b, 330, depuis=mk2)
    rejete = any('degraded PVT' in l for l in vues_strict)

    # 3. Seuil large: la meme position doit repasser.
    try:
        _config_ciel(b, GNSS_HACCFILT_EN=1, GNSS_HACCFILT_THR=200)
    except Exception:
        pass
    mk3 = b.mark()
    _, hacc_large, _ = _attend_fix_reel(b, 330, depuis=mk3)
    try:
        _config_ciel(b, GNSS_HACCFILT_EN=0)
    except Exception:
        pass

    trace = (f'ciel: hAcc={hacc/1000:.1f} m\n'
             f'seuil {strict} m -> {"REJETE (PVT degrade)" if rejete else f"accepte hAcc={hacc_strict}"}\n'
             f'seuil 200 m -> {"accepte" if hacc_large is not None else "AUCUNE position"}')
    if not rejete and hacc_strict is not None:
        return r.record(case, 'FAIL',
                        f'seuil a {strict} m mais une position a {hacc_strict/1000:.1f} m '
                        'est acceptee — le filtre hAcc ne filtre pas', trace)
    if hacc_large is None:
        return r.record(case, 'FAIL',
                        'seuil desserre a 200 m et pourtant aucune position acceptee — '
                        'le filtre rejette tout', trace)
    r.record(case, 'PASS', 'le filtre hAcc rejette au seuil strict et laisse passer au large',
             trace)

def c_ciel_fix_chaud(r, case):
    """Le second fix est nettement plus rapide que le premier.

    C est la contrepartie mesurable de la mort GNSS a deux jours: si
    l ephemeride n est pas conservee entre deux sessions, chaque acquisition
    repart de zero et la balise finit par ne plus jamais fixer dans sa fenetre.
    """
    b = r.b
    try:
        _config_ciel(b, GNSS_DEEP_IDLE_AFTER_OFF_S=30)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    mk = b.mark(); t0 = time.time()
    _, h1, _ = _attend_fix_reel(b, 330, depuis=mk)
    t_froid = time.time() - t0
    if h1 is None:
        return r.record(case, 'ERROR', 'pas de premiere position — cas non evaluable')
    time.sleep(45)
    mk2 = b.mark(); t1 = time.time()
    _, h2, _ = _attend_fix_reel(b, 330, depuis=mk2)
    t_chaud = time.time() - t1
    try:
        _config_ciel(b, GNSS_DEEP_IDLE_AFTER_OFF_S=0)
    except Exception:
        pass
    trace = f'premier fix: {t_froid:.0f} s\nsecond fix: {t_chaud:.0f} s'
    if h2 is None:
        return r.record(case, 'FAIL',
                        'la seconde session ne fixe plus alors que la premiere a reussi — '
                        'l ephemeride n est pas conservee', trace)
    if t_chaud > t_froid:
        return r.record(case, 'FAIL',
                        f'le second fix ({t_chaud:.0f} s) est plus lent que le premier '
                        f'({t_froid:.0f} s) — rien n est conserve entre sessions', trace)
    r.record(case, 'PASS', f'premier fix {t_froid:.0f} s, second {t_chaud:.0f} s', trace)

def c_ciel_emission_reelle(r, case):
    """Une position reelle part reellement par satellite.

    Bout en bout: ciel -> pilote -> pile de profondeur -> encodage -> KIM2 ->
    antenne. Toute la campagne a valide des morceaux de cette chaine; ce cas est
    le seul qui la parcourt entiere avec une vraie position.
    """
    if saute_si_radio_muette(r, case, 'une emission reussie'):
        return
    b = r.b
    try:
        _config_ciel(b, ARGOS_MODE=2, TR_NOM=60, NTRY_PER_MESSAGE=1,
                     ARGOS_DEPTH_PILE=1, DUTY_CYCLE=16777215, SAT_PREPASS_EN=0)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    mk = b.mark()
    _, hacc, _ = _attend_fix_reel(b, 330, depuis=mk)
    if hacc is None:
        return r.record(case, 'ERROR', 'pas de position reelle — emission non evaluable')
    vues = _attendre_trace(b, [r'TX SUCCESS', r'\+ERROR=', r'TX START'], 180, depuis=mk)
    try:
        b.enter_config(); b.write_params({'ARGOS_MODE': 0}); b.exit_config()
    except Exception:
        pass
    trace = '\n'.join(vues[:8])
    succes = [l for l in vues if 'TX SUCCESS' in l]
    erreurs = [l for l in vues if '+ERROR=' in l]
    if not vues:
        return r.record(case, 'FAIL',
                        'position reelle acquise mais AUCUNE emission tentee en 3 min', trace)
    if not succes and erreurs:
        return r.record(case, 'FAIL', f'emission refusee par le module: {erreurs[0][:70]}', trace)
    if not succes:
        return r.record(case, 'ERROR', 'emission tentee sans verdict — cas non concluant', trace)
    r.record(case, 'PASS', f'position reelle emise ({hacc/1000:.1f} m de precision)', trace)

def c_ciel_prepasse_reelle(r, case):
    """Avec une AOP fraiche et une vraie position, la prepasse calcule un passage.

    ARG-04 ne prouve aujourd hui que le REPLI, faute d AOP exploitable. Ici la
    table vient d etre chargee et la position est reelle: le firmware doit
    calculer un passage plutot que retomber sur le periodique.
    """
    b = r.b
    try:
        _config_ciel(b, ARGOS_MODE=1, SAT_PREPASS_EN=1, TR_NOM=60,
                     NTRY_PER_MESSAGE=0, DUTY_CYCLE=16777215)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    mk = b.mark()
    _, hacc, _ = _attend_fix_reel(b, 330, depuis=mk)
    if hacc is None:
        return r.record(case, 'ERROR', 'pas de position reelle — prepasse non evaluable')
    vues = _attendre_trace(b, [r'prepass', r'next pass', r'next window',
                               r'periodic TX'], 150, depuis=mk)
    aop = None
    try:
        b.enter_config(); aop = _statr(b, ['PPT01', 'PPT02']); b.exit_config()
    except Exception:
        pass
    try:
        b.enter_config(); b.write_params({'ARGOS_MODE': 0, 'SAT_PREPASS_EN': 0}); b.exit_config()
    except Exception:
        pass
    trace = f'AOP: {aop}\n' + '\n'.join(vues[:6])
    if aop and aop.get('PPT01') != '1':
        return r.record(case, 'ERROR',
                        'AOP invalide sur la carte — charger la capture Kineis avant ce cas',
                        trace)
    repli = [l for l in vues if 'periodique' in l]
    if repli:
        return r.record(case, 'FAIL',
                        'AOP valide et position reelle, et pourtant repli periodique: '
                        'aucun passage calcule', trace)
    if not vues:
        return r.record(case, 'ERROR', 'aucune trace de prepasse en 150 s', trace)
    r.record(case, 'PASS', 'un passage satellite est calcule sur AOP fraiche', trace)

CASES_V26 = [
    dict(id='OUT-01', risque='BLOQUANT', titre='Une vraie position tombe, precise et plausible',
         fn=c_ciel_premier_fix),
    dict(id='OUT-02', risque='BLOQUANT', titre='Le filtre hAcc rejette au strict, laisse passer au large',
         fn=c_ciel_filtre_hacc),
    dict(id='OUT-03', risque='MAJEUR',   titre='Le second fix est plus rapide que le premier',
         fn=c_ciel_fix_chaud),
    dict(id='OUT-04', risque='BLOQUANT', titre='Une position reelle part reellement par satellite',
         fn=c_ciel_emission_reelle),
    dict(id='OUT-05', risque='MAJEUR',   titre='La prepasse calcule un passage sur AOP fraiche',
         fn=c_ciel_prepasse_reelle),
]


# Les deux vagues de plein air exigent un recepteur GNSS qui repond. Sans lui
# elles ne sont pas en echec: elles sont hors de portee de la carte. Le drapeau
# est pose ici plutot que dans chaque dict pour qu aucun cas de ciel n y echappe
# par oubli lors d un ajout.
for _c in CASES_V26:
    _c['ciel'] = True


# =====================================================================
#  Vague 27 — VALIDATION DEPLOIEMENT KIM2
#
#  Les configurations reelles de mission, pas des unites isolees. Chaque cas
#  repond a une question de deploiement: "si je pose une balise comme ceci,
#  est-ce qu elle emet, et est-ce qu elle CONTINUE d emettre ?"
#
#  Le fil rouge est le meme partout: la panne redoutee n est pas le plantage,
#  c est le SILENCE. Une balise qui se tait sans trace est indiscernable d une
#  balise perdue, et personne ne va la rechercher.
# =====================================================================

def _compter_trace(b, motifs, secondes, depuis=None):
    """Observe PENDANT TOUTE la fenetre et rend tout ce qui a correspondu.

    A ne pas confondre avec _attendre_trace, qui rend la main au PREMIER motif
    trouve quand on ne lui passe pas `exiger`. Compter des evenements avec elle
    donne toujours 1: c est ainsi que DEP-01 a conclu "1 emission en 5 min"
    apres avoir observe 53 secondes, sur une cadence de 30 s ou une seule
    emission est exactement ce qu on attend.

    Quand la question est "combien", il faut attendre la fin de la fenetre.
    """
    mk = b.mark() if depuis is None else depuis
    fin = time.time() + secondes
    while time.time() < fin:
        time.sleep(3)
    with b._lock:
        lignes = [l for _, l in b.history[mk:]]
    return [l.strip()[24:200] for l in lignes if any(re.search(m, l) for m in motifs)]

def c_doppler_seul_continu(r, case):
    """Doppler seul, sans GNSS ni SWS: la balise emet et NE S ARRETE PAS.

    C est le deploiement le plus depouille — GNSS_EN=0, UNDERWATER_EN=0, mode
    LEGACY — et le plus expose: rien ne vient reveiller la balise si elle
    s endort. Un verrou de premier fix, un refroidissement, un backoff sans
    sortie, et elle se tait pour de bon.

    On observe sur plusieurs cycles: il ne suffit pas qu elle emette une fois.
    """
    if saute_si_radio_muette(r, case, 'une emission reussie'):
        return
    b = r.b
    try:
        b.enter_config()
        b.write_params({'ARGOS_MODE': 2, 'TR_NOM': 30, 'GNSS_EN': 0,
                        'UNDERWATER_EN': 0, 'NTRY_PER_MESSAGE': 0,
                        'ARGOS_DEPTH_PILE': 1, 'DUTY_CYCLE': 16777215,
                        'LB_EN': 0, 'RATE_LIMIT_EN': 0, 'SAT_PREPASS_EN': 0,
                        'MIN_SURFACE_CYCLE_INTERVAL_S': 0})
        b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    mk = b.mark()
    # 5 min: assez pour une dizaine de periodes a TR_NOM=30 s.
    vues = _compter_trace(b, [r'TX SUCCESS', r'\+ERROR=', r'reset cause',
                              r'entry: BootState'], 300, depuis=mk)
    emissions = [l for l in vues if 'TX SUCCESS' in l]
    reboots = [l for l in vues if 'BootState' in l or 'reset cause' in l]
    try:
        b.enter_config(); b.write_params({'ARGOS_MODE': 0}); b.exit_config()
    except Exception:
        pass
    trace = f'{len(emissions)} emissions, {len(reboots)} redemarrages\n' + '\n'.join(vues[:6])
    if reboots:
        return r.record(case, 'FAIL',
                        f'{len(reboots)} redemarrage(s) pendant une emission Doppler continue',
                        trace)
    if not emissions:
        return r.record(case, 'FAIL',
                        'aucune emission en 5 min sans GNSS ni SWS — la balise est muette '
                        'dans la configuration la plus simple', trace)
    if len(emissions) < 3:
        return r.record(case, 'FAIL',
                        f'{len(emissions)} emission(s) seulement en 5 min a TR_NOM=30 s: '
                        'la cadence s arrete', trace)
    r.record(case, 'PASS',
             f'{len(emissions)} emissions en 5 min, aucun redemarrage', trace)

def c_duty_masque_valide(r, case):
    """Un masque horaire VALIDE n emet que dans son heure, et le repli ne l ecrase pas.

    DUTY-01 couvre le masque VIDE (mort silencieuse). Le risque inverse n etait
    pas couvert: qu un masque partiel soit ignore et que la balise emette a
    toute heure, brulant son quota satellite et sa batterie hors des fenetres
    voulues.
    """
    if saute_si_radio_muette(r, case, 'une emission reussie'):
        return
    b = r.b
    # L horloge de la CARTE est imposee, elle n a rien a voir avec celle de
    # l hote: is_in_duty_cycle() teste `duty & (0x800000 >> heure_RTC)`. Une
    # premiere version derivait le masque de time.gmtime() cote hote et
    # concluait que le duty-cycle bloquait tout, alors que la carte etait a une
    # autre heure. 10:30 UTC place le test au milieu de l heure 10, loin des
    # bords ou un changement d heure fausserait le verdict.
    base = 1767263400   # 2026-01-01 10:30:00 UTC
    heure = 10
    masque_courant = 0x800000 >> heure
    masque_autre = 0x800000 >> ((heure + 12) % 24)
    try:
        b.enter_config()
        _rtcw(b, base)
        b.write_params({'ARGOS_MODE': 3, 'TR_NOM': 30, 'GNSS_EN': 0,
                        'UNDERWATER_EN': 0, 'NTRY_PER_MESSAGE': 0,
                        'ARGOS_DEPTH_PILE': 1, 'LB_EN': 0, 'RATE_LIMIT_EN': 0,
                        'ARGOS_TX_JITTER_EN': 0,
                        'DUTY_CYCLE': masque_autre})
        b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    mk = b.mark()
    hors = _compter_trace(b, [r'TX SUCCESS'], 120, depuis=mk)
    try:
        b.enter_config(); b.write_params({'DUTY_CYCLE': masque_courant}); b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'reconfiguration impossible: {type(e).__name__}: {e}')
    mk2 = b.mark()
    dedans = _compter_trace(b, [r'TX SUCCESS'], 150, depuis=mk2)
    try:
        b.enter_config()
        b.write_params({'ARGOS_MODE': 0, 'DUTY_CYCLE': 16777215}); b.exit_config()
    except Exception:
        pass
    trace = (f'heure RTC imposee={heure}\nmasque AUTRE heure ({masque_autre:#08x}): '
             f'{len(hors)} emission(s)\nmasque HEURE COURANTE ({masque_courant:#08x}): '
             f'{len(dedans)} emission(s)')
    if hors:
        return r.record(case, 'FAIL',
                        f'{len(hors)} emission(s) avec un masque excluant l heure courante: '
                        'le duty-cycle est ignore', trace)
    if not dedans:
        return r.record(case, 'FAIL',
                        'aucune emission avec un masque AUTORISANT l heure courante: '
                        'le duty-cycle bloque tout', trace)
    r.record(case, 'PASS',
             f'silencieux hors fenetre, {len(dedans)} emission(s) dans la fenetre', trace)

def c_credentials_survivent(r, case):
    """Les identifiants Argos survivent aux redemarrages.

    Le chemin de brick: trois demarrages rates declenchent un reset usine qui
    efface DECID, HEXID et la cle secrete. Sur balise scellee, definitif — plus
    aucun moyen de la reprogrammer sur le terrain.

    On ne PROVOQUE pas d echec de demarrage: ce serait irreversible si le
    correctif ne tenait pas. On verifie que les identifiants sont la, que le
    compteur d echecs est a zero, et qu un redemarrage volontaire ne les touche
    pas.
    """
    b = r.b
    if not _en_config(b):
        return r.record(case, 'ERROR', 'mode configuration inaccessible')
    try:
        _, avant = b.read_params(['ARGOS_DECID', 'ARGOS_HEXID'])
    except Exception as e:
        return r.record(case, 'ERROR', f'lecture impossible: {type(e).__name__}: {e}')
    echecs, usine = _boot(b)
    if echecs is None:
        return r.record(case, 'ERROR', '%BOOT sans reponse')
    decid = avant.get(b._key('ARGOS_DECID'), '')
    if not decid or decid == '0':
        return r.record(case, 'FAIL',
                        f'ARGOS_DECID vaut {decid!r}: la carte n a pas d identifiant satellite',
                        f'avant={avant} boot=({echecs},{usine})')
    if usine:
        return r.record(case, 'FAIL',
                        'une tentative de reset usine est enregistree — les identifiants '
                        'ont pu etre effaces', f'boot=({echecs},{usine})')
    # Redemarrage volontaire, puis relecture.
    try:
        b.dte('RSTBW', '', timeout=8.0)
    except Exception:
        pass
    time.sleep(8)
    if not r.connect():
        return r.record(case, 'ERROR', 'la carte ne revient pas apres RSTBW')
    b = r.b
    if not b.wait_state('OPERATIONAL', timeout=90):
        return r.record(case, 'ERROR', 'la carte ne repasse pas en OPERATIONAL')
    if not _en_config(b):
        return r.record(case, 'ERROR', 'mode configuration inaccessible apres redemarrage')
    _, apres = b.read_params(['ARGOS_DECID', 'ARGOS_HEXID'])
    echecs2, usine2 = _boot(b)
    trace = f'avant={avant}\napres={apres}\nboot apres=({echecs2},{usine2})'
    if apres != avant:
        return r.record(case, 'FAIL', 'les identifiants ont CHANGE apres un redemarrage', trace)
    if echecs2:
        return r.record(case, 'FAIL',
                        f'compteur d echecs a {echecs2} apres un demarrage reussi', trace)
    r.record(case, 'PASS',
             f'identifiants intacts apres redemarrage (DECID={decid}), compteur a zero', trace)

def c_batterie_transitoire_tx(r, case):
    """Aucune alerte batterie critique sur une batterie saine pendant les emissions.

    Une emission Argos tire un pic de courant. Si la mesure n est pas filtree,
    le creux de tension passe pour une batterie mourante et la balise bascule en
    profil basse consommation — voire s eteint — alors qu elle est saine. La
    mediane doit absorber le creux.
    """
    if saute_si_radio_muette(r, case, 'une emission reussie'):
        return
    b = r.b
    try:
        b.enter_config()
        b.write_params({'ARGOS_MODE': 2, 'TR_NOM': 30, 'GNSS_EN': 0,
                        'UNDERWATER_EN': 0, 'NTRY_PER_MESSAGE': 0,
                        'ARGOS_DEPTH_PILE': 1, 'DUTY_CYCLE': 16777215,
                        'LB_EN': 1, 'RATE_LIMIT_EN': 0})
        b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    mk = b.mark()
    vues = _compter_trace(b, [r'TX SUCCESS', r'BatteryMonitorEventVoltageCritical',
                              r'LOW_BATTERY', r'critical'], 240, depuis=mk)
    emissions = [l for l in vues if 'TX SUCCESS' in l]
    alertes = [l for l in vues if 'ritical' in l or 'LOW_BATTERY' in l]
    try:
        b.enter_config(); b.write_params({'ARGOS_MODE': 0, 'LB_EN': 0}); b.exit_config()
    except Exception:
        pass
    trace = f'{len(emissions)} emissions, {len(alertes)} alertes\n' + '\n'.join(vues[:6])
    if not emissions:
        return r.record(case, 'ERROR', 'aucune emission — transitoire non observable', trace)
    if alertes:
        return r.record(case, 'FAIL',
                        f'{len(alertes)} alerte(s) batterie sur {len(emissions)} emissions '
                        'avec une batterie saine: le transitoire n est pas absorbe', trace)
    r.record(case, 'PASS',
             f'{len(emissions)} emissions, aucune alerte batterie', trace)

def c_reveil_apres_refroidissement(r, case):
    """LE cas qui decide: apres un refroidissement, l emission REPART-ELLE ?

    Sur une configuration sans GNSS ni SWS, rien ne vient reveiller la balise:
    pas de fix, pas d emersion. Si le refroidissement n a pas d echeance propre,
    la balise se tait DEFINITIVEMENT — et c est indiscernable d une balise
    perdue.

    On arme un refroidissement court, on attend qu il expire, et on verifie que
    l emission reprend d elle-meme.
    """
    if saute_si_radio_muette(r, case, 'une emission reussie'):
        return
    b = r.b
    try:
        b.enter_config()
        b.write_params({'ARGOS_MODE': 2, 'TR_NOM': 30, 'GNSS_EN': 0,
                        'UNDERWATER_EN': 0, 'NTRY_PER_MESSAGE': 0,
                        'ARGOS_DEPTH_PILE': 1, 'DUTY_CYCLE': 16777215,
                        'LB_EN': 0, 'RATE_LIMIT_EN': 0,
                        'MIN_SURFACE_CYCLE_INTERVAL_S': 90,
                        'COOLDOWN_TRIGGER_MODE': 3})
        b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    mk = b.mark()
    avant = _attendre_trace(b, [r'TX SUCCESS'], 120, depuis=mk)
    if not avant:
        try:
            b.enter_config()
            b.write_params({'ARGOS_MODE': 0, 'MIN_SURFACE_CYCLE_INTERVAL_S': 0})
            b.exit_config()
        except Exception:
            pass
        return r.record(case, 'ERROR',
                        'aucune emission initiale — le reveil n est pas evaluable',
                        '\n'.join(avant[:4]))
    # Le refroidissement s arme sur la derniere emission (UNP30=3). On attend
    # sa fenetre PLUS une marge, et on exige une reprise.
    mk2 = b.mark()
    apres = _attendre_trace(b, [r'TX SUCCESS'], 260, depuis=mk2)
    try:
        b.enter_config()
        b.write_params({'ARGOS_MODE': 0, 'MIN_SURFACE_CYCLE_INTERVAL_S': 0,
                        'COOLDOWN_TRIGGER_MODE': 3})
        b.exit_config()
    except Exception:
        pass
    trace = (f'avant refroidissement: {len(avant)} emission(s)\n'
             f'apres la fenetre de 90 s: {len(apres)} emission(s)\n' + '\n'.join(apres[:4]))
    if not apres:
        return r.record(case, 'FAIL',
                        'la balise a emis puis N A JAMAIS REPRIS apres le refroidissement, '
                        'sans GNSS ni SWS pour la reveiller — silence definitif', trace)
    r.record(case, 'PASS',
             f'l emission repart seule apres le refroidissement ({len(apres)} emission(s))',
             trace)

CASES_V27 = [
    dict(id='DEP-01', risque='BLOQUANT', titre='Doppler seul: emet et ne s arrete pas',
         fn=c_doppler_seul_continu),
    dict(id='DEP-02', risque='BLOQUANT', titre='Masque horaire valide: silencieux hors fenetre, actif dedans',
         fn=c_duty_masque_valide),
    dict(id='DEP-03', risque='BLOQUANT', titre='Les identifiants Argos survivent aux redemarrages',
         fn=c_credentials_survivent),
    dict(id='DEP-04', risque='MAJEUR',   titre='Aucune alerte batterie sur le transitoire d emission',
         fn=c_batterie_transitoire_tx),
    dict(id='DEP-05', risque='BLOQUANT', titre='L emission repart apres un refroidissement, sans reveil externe',
         fn=c_reveil_apres_refroidissement),
]

def c_factory_reset_recuperable(r, case):
    """FACTW efface la configuration, et la balise RETROUVE son identite.

    C est le chemin de brick documente par l audit: trois demarrages rates
    declenchent un reset usine, sans operateur. Si ce reset emporte les
    identifiants, la balise scellee n emet plus jamais et rien ne peut la
    reprogrammer sur le terrain — le mecanisme cense la sauver serait celui qui
    la tue.

    DESTRUCTIF, et sur KIM2 SEULEMENT. Ce qui le rend sur ici:
      - DECID et HEXID sont RELUS DU MODULE a chaque init (kim2.cpp:1026), donc
        ils reviennent d eux-memes;
      - les RADIOCONF sont dans PROTECTED_PARAMS sans condition;
      - ARGOS_SECKEY n est protege que sous ARGOS_SMD, mais sur KIM2 la cle vit
        dans le module (RCONF chiffre, decodable par lui seul) et n est pas
        utilisee cote nRF.
    Sur une carte SMD ce cas EFFACERAIT une cle irrecuperable: ne pas le jouer
    la-bas sans carte sacrifiable.

    A JOUER EN DERNIER: tout le reste de la configuration revient aux defauts.
    """
    b = r.b
    if not _en_config(b):
        return r.record(case, 'ERROR', 'mode configuration inaccessible')
    try:
        _, avant = b.read_params(['ARGOS_DECID', 'ARGOS_HEXID', 'ARGOS_RADIOCONF_LDK'])
    except Exception as e:
        return r.record(case, 'ERROR', f'lecture impossible: {type(e).__name__}: {e}')
    decid_avant = avant.get(b._key('ARGOS_DECID'), '')
    if not decid_avant or decid_avant == '0':
        return r.record(case, 'ERROR',
                        'aucun identifiant avant le reset — cas non evaluable', f'{avant}')

    m = b.dte('FACTW', '', timeout=15.0)
    if not m:
        return r.record(case, 'ERROR', 'FACTW sans reponse')
    if m.group(1) != 'O':
        ligne = m.string if hasattr(m, 'string') else ''
        return r.record(case, 'FAIL', f'FACTW refuse: {ligne[:70]}')

    # Le reset usine redemarre la carte, et ce redemarrage fait re-enumerer le
    # CDC. Mesure du 2026-08-29: le noeud revient mais l endpoint ne draine plus
    # — zero octet, ecriture en timeout — et seul un reset SWD le ranime. Une
    # simple reconnexion concluait donc "la carte ne revient pas apres FACTW",
    # ce qui ressemble beaucoup a un brick et n en est pas un: le SWD repondait,
    # et la carte est revenue en OPERATIONAL avec ses identifiants intacts.
    #
    # La reconnexion fait partie du cas, et elle doit aller jusqu au bout: une
    # tentative douce, puis la reparation du lien, puis un reset materiel.
    time.sleep(12)
    revenue = r.connect(tries=10)
    if not revenue:
        r.say('   FACTW: lien perdu au redemarrage, reparation…')
        revenue = r.recover()
    if not revenue:
        return r.record(case, 'ERROR',
                        'la carte ne repond plus apres FACTW, meme apres reparation du '
                        'lien et reset materiel — verifier au SWD avant de conclure a un brick')
    b = r.b
    if not b.wait_state('OPERATIONAL', timeout=120):
        return r.record(case, 'ERROR',
                        'la carte ne repasse pas en OPERATIONAL apres FACTW — brick')
    if not _en_config(b):
        return r.record(case, 'ERROR', 'mode configuration inaccessible apres FACTW')
    _, apres = b.read_params(['ARGOS_DECID', 'ARGOS_HEXID', 'ARGOS_RADIOCONF_LDK'])
    echecs, usine = _boot(b)
    trace = f'avant={avant}\napres={apres}\nboot=({echecs},{usine})'

    defauts = []
    if apres.get(b._key('ARGOS_DECID')) != decid_avant:
        defauts.append(f"DECID perdu: {decid_avant} -> {apres.get(b._key('ARGOS_DECID'))}")
    if apres.get(b._key('ARGOS_HEXID')) != avant.get(b._key('ARGOS_HEXID')):
        defauts.append('HEXID perdu')
    rc_avant = avant.get(b._key('ARGOS_RADIOCONF_LDK'), '')
    rc_apres = apres.get(b._key('ARGOS_RADIOCONF_LDK'), '')
    if rc_avant and rc_apres != rc_avant:
        defauts.append(f'RADIOCONF_LDK perdu: {rc_avant[:16]}... -> {rc_apres[:16] or "(vide)"}')
    if defauts:
        return r.record(case, 'FAIL',
                        'reset usine NON RECUPERABLE: ' + '; '.join(defauts), trace)
    r.record(case, 'PASS',
             f'la balise retrouve son identite apres reset usine (DECID={decid_avant})', trace)

CASES_V28 = [
    dict(id='DEP-06', risque='BLOQUANT', titre='Un reset usine ne tue pas la balise',
         fn=c_factory_reset_recuperable),
]


# =====================================================================
#  Vague 29 — MATRICE: chaque mode Argos, chaque format de charge utile
#
#  La campagne eprouvait des modes isoles et des formats isoles, jamais leur
#  CROISEMENT. Or c est la que vivent les defauts: une trame dimensionnee pour
#  la mauvaise modulation, un format qui ne part que dans un mode, un mode qui
#  n emet jamais le format qu on croit.
#
#  Tailles documentees (argos_packet_builder.hpp):
#    SHORT       96 bits   (1 position)
#    LONG       192 bits   (2 a 3 positions, LDA2_FRAME_BITS)
#    FASTLOC    192 bits
#    MEASC12    128 bits   (CloudLocate, LDK)
# =====================================================================

_BITS_ATTENDUS = {'SHORT': 96, 'LONG': 192, 'FASTLOC': 192}

def _config_mode(b, mode, **extra):
    """Configuration minimale pour qu un mode emette, sans rien d autre."""
    cfg = {'ARGOS_MODE': mode, 'TR_NOM': 30, 'GNSS_EN': 1,
           'NTRY_PER_MESSAGE': 0, 'ARGOS_DEPTH_PILE': 1,
           'DUTY_CYCLE': 16777215, 'UNDERWATER_EN': 0, 'LB_EN': 0,
           'RATE_LIMIT_EN': 0, 'SAT_PREPASS_EN': 0,
           'MIN_SURFACE_CYCLE_INTERVAL_S': 0, 'ARGOS_TX_JITTER_EN': 0,
           'GNSS_SESSION_SINGLE_FIX': 1}
    cfg.update(extra)
    b.enter_config(); b.write_params(cfg); b.exit_config()

def _emissions(vues):
    """Extrait (type, mode) de chaque TX SUCCESS."""
    out = []
    for l in vues:
        m = re.search(r'TX SUCCESS — type=(\S+) mode=(\d+)', l)
        if m:
            out.append((m.group(1), int(m.group(2))))
    return out

def _paquets(vues):
    """Extrait (format, bits) de chaque construction de paquet."""
    out = []
    for l in vues:
        m = re.search(r'build_gnss_packet: (SHORT|LONG) packet, .*?(\d+) bits', l)
        if m:
            out.append((m.group(1), int(m.group(2))))
        m2 = re.search(r'build_fastloc_packet', l)
        if m2:
            out.append(('FASTLOC', None))
    return out

def _mode_emet(r, case, mode, nom, position=True, secondes=180, **extra):
    """Tronc commun: un mode donne construit un paquet et l emet.

    On verifie les DEUX: qu un paquet soit CONSTRUIT au bon format et qu il
    parte. Un paquet construit et jamais emis, ou une emission sans paquet
    trace, sont deux pannes distinctes et il faut pouvoir les distinguer.

    Le garde-fou radio est pose ICI plutot que chez chaque appelant. Mesure du
    2026-08-30: c_mode_duty tient en une seule ligne apres sa docstring, et une
    detection automatique y a vu le garde-fou de la fonction SUIVANTE — MOD-02
    est donc parti sans protection et a accuse le firmware d un paquet "jamais
    emis" sur une carte au module muet. Un tronc commun se garde au tronc.
    """
    if saute_si_radio_muette(r, case, 'une emission reussie'):
        return
    b = r.b
    try:
        _config_mode(b, mode, **extra)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    mk = b.mark()
    if position:
        b._send('%GPS 43.6 3.9 5000 9\r')
    vues = _compter_trace(b, [r'TX SUCCESS', r'build_\w+_packet', r'\+ERROR=',
                              r'periodique', r'SCHEDULE_DISABLED'], secondes, depuis=mk)
    try:
        b.enter_config(); b.write_params({'ARGOS_MODE': 0}); b.exit_config()
    except Exception:
        pass
    emis = _emissions(vues)
    paquets = _paquets(vues)
    erreurs = [l for l in vues if '+ERROR=' in l]
    trace = (f'{len(emis)} emission(s): {emis[:6]}\n'
             f'{len(paquets)} paquet(s): {paquets[:6]}\n' + '\n'.join(vues[:5]))
    if erreurs:
        return r.record(case, 'FAIL', f'module en erreur: {erreurs[0][:60]}', trace)
    if not paquets and not emis:
        return r.record(case, 'FAIL',
                        f'{nom}: aucun paquet construit ni emis en {secondes} s', trace)
    if not emis:
        return r.record(case, 'FAIL',
                        f'{nom}: paquet construit mais JAMAIS emis', trace)
    # Tailles: un format annonce doit porter le nombre de bits documente.
    for fmt, bits in paquets:
        attendu = _BITS_ATTENDUS.get(fmt)
        if attendu and bits and bits != attendu:
            return r.record(case, 'FAIL',
                            f'{fmt} construit avec {bits} bits au lieu de {attendu}', trace)
    return r.record(case, 'PASS',
                    f'{nom}: {len(emis)} emission(s), formats {sorted({f for f, _ in paquets})}',
                    trace)

def c_mode_legacy(r, case):
    """LEGACY: emission periodique, format SHORT sur une position."""
    if saute_si_radio_muette(r, case, 'une emission reussie'):
        return
    _mode_emet(r, case, 2, 'LEGACY')

def c_mode_duty(r, case):
    """DUTY_CYCLE avec masque plein: doit emettre comme LEGACY."""
    _mode_emet(r, case, 3, 'DUTY_CYCLE')

def c_mode_doppler(r, case):
    """DOPPLER: charge minimale, SANS position.

    C est le mode de repli quand le GNSS ne donne rien — il doit emettre
    justement quand il n y a pas de fix, sinon il ne sert a rien.
    """
    if saute_si_radio_muette(r, case, 'une emission reussie'):
        return
    _mode_emet(r, case, 4, 'DOPPLER', position=False, GNSS_EN=0)

# ARGOS_CACHED_MODULATION (SMP01) suit la convention SmdArgosModulation:
# 0=LDA2, 1=LDK, 2=VLDA4. L enum KineisModulation, celui qui apparait dans le
# log "TX SUCCESS ... mode=N", utilise un ORDRE DIFFERENT: LDK=0, LDA2=1,
# VLDA4=2. kim2.cpp fait la conversion explicitement et dit "never cast";
# confondre les deux inverse LDK et LDA2, donc inverse exactement la conclusion
# qu on tire sur la pile de profondeur.
_CACHE_LDA2, _CACHE_LDK, _CACHE_VLDA4 = '0', '1', '2'

def _nom_cache(v):
    return {'0': 'LDA2', '1': 'LDK', '2': 'VLDA4'}.get(v, f'inconnu({v})')


# =====================================================================
#  Etat du module satellite — mesure UNE fois, sert a toute la passe
# =====================================================================

_RADIO_ETAT = None   # None = pas encore mesure; sinon (vivante: bool, raison: str)


def radio_repond(b):
    """Le module satellite repond-il ? Mesure une seule fois par passe.

    SMDDFU action 5 (VERSION) est disponible sur TOUS les builds — SMD, KIM2 et
    LoRa — et c est la facon la moins invasive de demander au module s il est
    la. Il repond "$O;SMDDFU#...;<maj>,<min>,<rev>,<texte>", et le texte porte
    "Failed to read version" quand le module ne repond pas.

    POURQUOI CETTE MESURE EXISTE. Sur une carte dont la radio est muette, une
    partie des cas ne peut pas conclure — non parce que le firmware faute, mais
    parce que leur objet meme est une emission. Mesure du 2026-08-29: SURF-01
    (sequence bornee par les emissions), RL-02 (quota du limiteur, alimente par
    record_tx en fin d emission REUSSIE) et DUTY-02 (delai dependant de
    LAST_TX, jamais rafraichi) sont tombes rouges tous les trois, avec trois
    mecanismes differents et une seule cause. Un rouge qui n accuse rien coute
    plus de temps qu il n en fait gagner.

    Rend (vivante, raison). La raison est vide quand la radio repond.
    """
    global _RADIO_ETAT
    if _RADIO_ETAT is not None:
        return _RADIO_ETAT
    try:
        if not _en_config(b):
            _RADIO_ETAT = (True, '')      # dans le doute, ne rien sauter
            return _RADIO_ETAT
        m = b.dte('SMDDFU', '5', timeout=40.0)
        ligne = (m.string if m and hasattr(m, 'string') else '') or ''
        try:
            b.exit_config()
        except Exception:
            pass
        if not m:
            _RADIO_ETAT = (True, '')      # pas de reponse DTE: autre probleme, ne pas masquer
        elif 'Failed' in ligne or 'failed' in ligne:
            _RADIO_ETAT = (False, 'le module satellite ne repond pas (SMDDFU VERSION: '
                                  '"Failed to read version")')
        else:
            _RADIO_ETAT = (True, '')
    except Exception:
        _RADIO_ETAT = (True, '')          # ne jamais transformer un incident en masquage
    return _RADIO_ETAT


def saute_si_radio_muette(r, case, quoi):
    """Rend True et enregistre un SKIP motive si la radio ne repond pas.

    A appeler au tout debut d un cas qui exige une emission. `quoi` dit ce que
    le cas aurait eprouve, pour que le SKIP reste informatif.
    """
    vivante, raison = radio_repond(r.b)
    if vivante:
        return False
    r.record(case, 'SKIP', f'{raison} — {quoi} ne peut pas etre eprouve sur cette carte')
    return True


_GNSS_ETAT = None    # None = pas encore mesure; sinon (vivant: bool, raison: str)


def gnss_repond(b):
    """Le recepteur GNSS repond-il ? Mesure une seule fois par passe.

    GNSSI interroge l identite du recepteur par sa liaison serie, en allumant
    le rail au passage. Un recepteur qui ne repond pas a GNSSI ne rendra pas de
    position non plus — c est precisement ce que dit la docstring du cas
    GNSS-I1, et c est ce qui en fait la bonne sonde.

    POURQUOI CETTE MESURE EXISTE. Symetrique de radio_repond(). Mesure du
    2026-08-30 sur la carte SMD d interieur: GNSS-I1 sans reponse en 30 s, puis
    OUT-01 sans session, puis OUT-02 sans position de reference. Trois rouges,
    une seule cause — le recepteur ne repond pas sur cette carte — et aucun des
    trois n accuse le firmware. Les treize cas de plein air dependent tous d un
    recepteur qui repond; sans lui ils ne sont pas en echec, ils sont hors de
    portee de cette carte.

    Prudence deliberee: tout doute rend (True, ''). Une sonde qui masque un
    vrai defaut coute infiniment plus cher qu un rouge de trop.

    Rend (vivant, raison). La raison est vide quand le recepteur repond.
    """
    global _GNSS_ETAT
    if _GNSS_ETAT is not None:
        return _GNSS_ETAT
    try:
        if not _en_config(b):
            _GNSS_ETAT = (True, '')       # dans le doute, ne rien sauter
            return _GNSS_ETAT
        m = b.dte('GNSSI', '', timeout=35.0)
        ligne = (m.string if m and hasattr(m, 'string') else '') or ''
        try:
            b.exit_config()
        except Exception:
            pass
        if not m:
            _GNSS_ETAT = (False, 'le recepteur GNSS ne repond pas (GNSSI muet en 35 s)')
        elif m.group(1) != 'O':
            _GNSS_ETAT = (False, 'le recepteur GNSS refuse GNSSI '
                                 f'({ligne[:60]})')
        else:
            _GNSS_ETAT = (True, '')
    except Exception:
        _GNSS_ETAT = (True, '')           # ne jamais transformer un incident en masquage
    return _GNSS_ETAT


def saute_si_gnss_muet(r, case, quoi):
    """Rend True et enregistre un SKIP motive si le recepteur ne repond pas.

    `quoi` dit ce que le cas aurait eprouve, pour que le SKIP reste informatif.
    """
    vivant, raison = gnss_repond(r.b)
    if vivant:
        return False
    r.record(case, 'SKIP', f'{raison} — {quoi} ne peut pas etre eprouve sur cette carte')
    return True


def c_mode_long_multi(r, case):
    """Trois positions dans la pile: le format LONG doit etre choisi.

    Un LONG transporte jusqu a trois positions. Si la balise n emettait que du
    SHORT, chaque position couterait une emission entiere — trois fois le
    budget satellite pour la meme information.
    """
    if saute_si_radio_muette(r, case, 'le dialogue avec le module'):
        return
    b = r.b
    # Un LONG fait 192 bits et LDK n en porte que 128: sur un module dont la
    # RCONF maitresse est LDK, process_gnss_burst PLAFONNE deliberement la
    # recuperation a une position (argos_tx_service.cpp, "cap the burst to a
    # single fix at retrieve time") pour ne pas remettre a KIM2 une charge qu il
    # laisse tomber en silence. Trois paquets SHORT sont alors la BONNE reponse,
    # et l affirmer defaut ferait accuser un firmware sain — mesure du
    # 2026-08-29, mode=0 dans la trace.
    #
    # On demande donc la modulation adaptative, qui autorise le service a passer
    # en LDA2 le temps de l emission. Si LDA2 n est pas provisionne dans le
    # module, le cas ne peut pas conclure et le dit.
    try:
        _config_mode(b, 2, ARGOS_DEPTH_PILE=4, NTRY_PER_MESSAGE=1, TR_NOM=60,
                     ARGOS_ADAPTIVE_MODULATION=1)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    mk = b.mark()
    for lat in (43.60, 43.61, 43.62):
        b._send(f'%GPS {lat} 3.9 5000 9\r')
        time.sleep(12)
    vues = _compter_trace(b, [r'TX SUCCESS', r'build_gnss_packet'], 150, depuis=mk)
    try:
        b.enter_config(); b.write_params({'ARGOS_MODE': 0}); b.exit_config()
    except Exception:
        pass
    paquets = _paquets(vues)
    trace = f'paquets: {paquets}\n' + '\n'.join(vues[:6])
    longs = [(f, n) for f, n in paquets if f == 'LONG']
    if not paquets:
        return r.record(case, 'FAIL', 'aucun paquet construit avec 3 positions en pile', trace)
    if not longs:
        # Distinguer "le firmware refuse d empiler" de "la modulation ne peut
        # pas porter un LONG". Seul le premier est un defaut.
        modul = None
        try:
            b.enter_config()
            _, lus = b.read_params(['ARGOS_CACHED_MODULATION'])
            modul = lus.get('SMP01')
            b.exit_config()
        except Exception:
            pass
        if modul == _CACHE_LDK:
            return r.record(case, 'SKIP',
                            'module en LDK (128 bits): un LONG de 192 bits ne rentre pas et '
                            'le firmware plafonne a une position par emission, ce qui est '
                            'correct. Il faut une RCONF LDA2 pour conclure.', trace)
        return r.record(case, 'FAIL',
                        f'trois positions en pile mais AUCUN paquet LONG '
                        f'(modulation en cache={_nom_cache(modul)}): chaque position coute '
                        'une emission entiere', trace)
    for _, bits in longs:
        if bits and bits != _BITS_ATTENDUS['LONG']:
            return r.record(case, 'FAIL',
                            f"LONG construit avec {bits} bits au lieu de "
                            f"{_BITS_ATTENDUS['LONG']}", trace)
    r.record(case, 'PASS', f'{len(longs)} paquet(s) LONG de 192 bits', trace)

def c_charge_sans_fix(r, case):
    """Sans aucun fix, la balise emet quand meme un signe de vie.

    Le verrou de premier message tenait TOUTE emission tant qu aucun fix n etait
    tombe: une balise qui redemarrait sans jamais retrouver le ciel
    disparaissait des ecrans. Le heartbeat doit partir malgre tout.
    """
    if saute_si_radio_muette(r, case, 'une emission reussie'):
        return
    b = r.b
    try:
        _config_mode(b, 2, GNSS_EN=0)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    mk = b.mark()
    vues = _compter_trace(b, [r'TX SUCCESS', r'build_\w+_packet'], 150, depuis=mk)
    try:
        b.enter_config(); b.write_params({'ARGOS_MODE': 0}); b.exit_config()
    except Exception:
        pass
    emis = _emissions(vues)
    trace = f'{len(emis)} emission(s): {emis[:6]}\n' + '\n'.join(vues[:5])
    if not emis:
        return r.record(case, 'FAIL',
                        'aucune emission sans fix: une balise qui ne retrouve pas le ciel '
                        'disparait des ecrans', trace)
    r.record(case, 'PASS', f'{len(emis)} emission(s) sans aucun fix', trace)

CASES_V29 = [
    dict(id='MOD-01', risque='BLOQUANT', titre='LEGACY construit et emet',           fn=c_mode_legacy),
    dict(id='MOD-02', risque='BLOQUANT', titre='DUTY_CYCLE masque plein emet',       fn=c_mode_duty),
    dict(id='MOD-03', risque='BLOQUANT', titre='DOPPLER emet sans position',         fn=c_mode_doppler),
    dict(id='MOD-04', risque='MAJEUR',   titre='Trois positions donnent un LONG de 192 bits', fn=c_mode_long_multi),
    dict(id='MOD-05', risque='BLOQUANT', titre='Sans fix, la balise donne signe de vie', fn=c_charge_sans_fix),
]


# =====================================================================
#  Vague 30 — GNSS PLEIN AIR: tous les parametres, et surtout leurs bornes
#
#  Une seule question traverse cette vague: un parametre regle trop STRICT
#  produit-il un echec PROPRE ET TRACE, ou une mort silencieuse ?
#
#  C est le mode de defaillance le plus couteux du produit. Un operateur qui
#  serre GNSS_MIN_CNO ou GNSS_MIN_ELEV pour "n avoir que de bonnes positions"
#  peut, sans le savoir, configurer une balise qui n en rapportera jamais
#  aucune — et rien dans le journal ne le lui dira si le firmware se contente
#  de ne rien trouver.
#
#  Les filtres hAcc et HDOP vivent dans le pilote M10Q, sur la trame NAV-PVT
#  REELLE (m10qasync.cpp): %GPS les court-circuite. Cette vague est le seul
#  endroit de toute la campagne qui les exerce.
#
#  Duree: chaque cas s arrete des qu il a sa reponse. Les cas qui attendent un
#  ECHEC sont bornes par le timeout d acquisition configure, pas par une
#  attente fixe — c est ce qui rend la vague tenable en une passe.
# =====================================================================

_CIEL_BASE = {
    'GNSS_EN': 1, 'ARGOS_MODE': 0, 'UNDERWATER_EN': 0, 'LB_EN': 0,
    'RATE_LIMIT_EN': 0, 'MOORED_DETECT_EN': 0, 'HAULED_DETECT_EN': 0,
    'ZONE_ENABLE_OUT_OF_ZONE_DETECTION_MODE': 0, 'SAT_PREPASS_EN': 0,
    'GNSS_HACCFILT_EN': 0, 'GNSS_HDOPFILT_EN': 0,
    'GNSS_DEEP_IDLE_AFTER_OFF_S': 0, 'GNSS_SESSION_SINGLE_FIX': 1,
    'GNSS_MIN_CNO': 10, 'GNSS_MIN_ELEV': 10, 'GNSS_MIN_NUM_FIXES': 1,
    'GNSS_CONSTELLATION_MASK': 0x0F, 'GNSS_FIX_MODE': 3,
    # 600 = borne DTE. Voir _config_ciel: sans BBR retenue, chaque session est
    # un froid absolu et 180 s ne suffisent pas a telecharger un almanach.
    'GNSS_ACQ_TIMEOUT': 600, 'GNSS_COLD_ACQ_TIMEOUT': 600,
    # Voir _config_ciel: sans ces quatre lignes, la vague exterieure herite de
    # la cadence de 24 h laissee par DP-01, des tentatives epuisees par GPS-04
    # et du demarrage a froid force par GPS-06.
    'DLOC_ARG_NOM': 13, 'GNSS_NTRY': 5,
    'GNSS_ASSISTNOW_EN': 1, 'GNSS_ASSISTNOW_OFFLINE_EN': 1,
}

def _ciel(b, **extra):
    cfg = dict(_CIEL_BASE); cfg.update(extra)
    b.enter_config(); b.write_params(cfg); b.exit_config()

def _session(b, secondes=240, depuis=None):
    """Observe UNE session GNSS. Rend (issue, hAcc_mm, numSV, secondes, lignes).

    issue vaut 'fix', 'sans-fix', 'degrade' ou None (rien observe). On s arrete
    des qu une issue tombe: attendre la fin d une fenetre quand la reponse est
    deja la ne mesure que la patience.
    """
    mk = b.mark() if depuis is None else depuis
    t0 = time.time()
    motifs = [r'task_process_gnss_data: lat=', r'acquisition timeout — no fix',
              r'timeout with degraded PVT', r'M10Q on —']
    fin = t0 + secondes
    while time.time() < fin:
        time.sleep(3)
        with b._lock:
            lignes = [l.strip()[24:220] for l in b.history[mk:]
                      if any(re.search(m, l) for m in motifs)]
        for l in lignes:
            if 'task_process_gnss_data' in l:
                h = re.search(r'hAcc=([\d.]+)', l)
                n = re.search(r'numSV=(\d+)', l)
                return ('fix', float(h.group(1)) if h else None,
                        int(n.group(1)) if n else 0, time.time() - t0, lignes)
            if 'degraded PVT' in l:
                h = re.search(r'hAcc=(\d+)', l)
                return ('degrade', float(h.group(1)) if h else None, 0,
                        time.time() - t0, lignes)
            if 'no fix' in l:
                return ('sans-fix', None, 0, time.time() - t0, lignes)
    with b._lock:
        lignes = [l.strip()[24:220] for l in b.history[mk:]
                  if any(re.search(m, l) for m in motifs)]
    return (None, None, 0, time.time() - t0, lignes)


def c_ciel_reference(r, case):
    """Position reelle de reference: precision, satellites, temps.

    Tous les cas suivants se calibrent sur ces chiffres. Sans eux, un seuil
    "strict" est une supposition.
    """
    b = r.b
    try:
        _ciel(b)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    issue, hacc, numsv, dt, lignes = _session(b, 300)
    trace = f'issue={issue} hAcc={hacc} numSV={numsv} en {dt:.0f}s\n' + '\n'.join(lignes[:5])
    if issue != 'fix':
        if issue is None and not lignes:
            return r.record(case, 'ERROR',
                            'aucune session GNSS en 5 min: le recepteur ne s est PAS allume '
                            '— configuration, pas ciel', trace)
        return r.record(case, 'ERROR',
                        f'le recepteur a cherche sans aboutir en 5 min (issue={issue})', trace)
    r.b._ciel_hacc = hacc
    r.b._ciel_numsv = numsv
    defauts = []
    if hacc > 50000:
        defauts.append(f'hAcc={hacc/1000:.1f} m inexploitable pour du suivi animal')
    if numsv < 4:
        defauts.append(f'numSV={numsv}: un fix 3D en demande au moins 4')
    if defauts:
        return r.record(case, 'FAIL', '; '.join(defauts), trace)
    r.record(case, 'PASS',
             f'reference: hAcc={hacc/1000:.1f} m, {numsv} satellites, {dt:.0f} s', trace)


def _borne_stricte(r, case, nom, quoi, **cfg):
    """Tronc commun des cas limites: un reglage impossible doit ECHOUER PROPREMENT.

    Le verdict ne porte pas sur l absence de position — elle est attendue — mais
    sur la maniere: la session doit se TERMINER et le DIRE. Une session qui ne
    conclut jamais laisse le recepteur allume et la balise muette.
    """
    b = r.b
    try:
        _ciel(b, **cfg)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    issue, hacc, numsv, dt, lignes = _session(b, 240)
    try:
        _ciel(b)
    except Exception:
        pass
    trace = f'issue={issue} hAcc={hacc} numSV={numsv} en {dt:.0f}s\n' + '\n'.join(lignes[:6])
    if issue is None:
        return r.record(case, 'FAIL',
                        f'{nom}: la session ne conclut NI par un fix NI par un echec en '
                        f'{dt:.0f} s — le recepteur reste allume et la balise muette', trace)
    if issue == 'fix':
        return r.record(case, 'PASS',
                        f'{nom}: fixe malgre {quoi} (hAcc={hacc/1000:.1f} m, {numsv} sat)', trace)
    return r.record(case, 'PASS',
                    f'{nom}: pas de position ({issue}) mais la session se termine et le dit '
                    f'en {dt:.0f} s', trace)


def c_ciel_cno_max(r, case):
    """GNSS_MIN_CNO=50 (le maximum): presque aucun satellite ne passe le seuil."""
    _borne_stricte(r, case, 'MIN_CNO=50', 'un seuil de signal au maximum',
                   GNSS_MIN_CNO=50, GNSS_ACQ_TIMEOUT=90)

def c_ciel_elev_max(r, case):
    """GNSS_MIN_ELEV=90: seuls les satellites au zenith exact comptent."""
    _borne_stricte(r, case, 'MIN_ELEV=90', 'un masque d antenne au zenith',
                   GNSS_MIN_ELEV=90, GNSS_ACQ_TIMEOUT=90)

def c_ciel_mask_gps_seul(r, case):
    """GNSS_CONSTELLATION_MASK=0x01: GPS seul, la constellation minimale.

    Un deploiement peut vouloir GPS seul pour la consommation. Il doit alors
    fixer quand meme — sinon le reglage est un piege.
    """
    _borne_stricte(r, case, 'GPS seul', 'la constellation minimale',
                   GNSS_CONSTELLATION_MASK=0x01, GNSS_ACQ_TIMEOUT=180)

def c_ciel_acq_minimal(r, case):
    """GNSS_ACQ_TIMEOUT=10, la borne basse: dix echantillons pour fixer."""
    _borne_stricte(r, case, 'ACQ_TIMEOUT=10', 'la fenetre d acquisition minimale',
                   GNSS_ACQ_TIMEOUT=10, GNSS_COLD_ACQ_TIMEOUT=10)

def c_ciel_hacc_frontiere(r, case):
    """Le filtre hAcc a la FRONTIERE de ce que le ciel donne.

    Le vrai cas limite: un seuil juste EN DESSOUS de la precision mesuree doit
    rejeter, un seuil juste AU-DESSUS doit accepter. Regler au hasard ne
    distingue pas un filtre qui marche d un filtre inerte.
    """
    b = r.b
    hacc = getattr(b, '_ciel_hacc', None)
    if hacc is None:
        return r.record(case, 'ERROR',
                        'pas de precision de reference — jouer GPS-C1 avant ce cas')
    sous = max(1, int(hacc / 1000) - 1)      # metres, juste sous la mesure
    sur = int(hacc / 1000) + 10              # confortablement au-dessus
    try:
        _ciel(b, GNSS_HACCFILT_EN=1, GNSS_HACCFILT_THR=sous, GNSS_ACQ_TIMEOUT=90)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    i1, h1, _, t1, l1 = _session(b, 150)
    try:
        _ciel(b, GNSS_HACCFILT_EN=1, GNSS_HACCFILT_THR=sur, GNSS_ACQ_TIMEOUT=180)
    except Exception:
        pass
    i2, h2, _, t2, l2 = _session(b, 240)
    try:
        _ciel(b)
    except Exception:
        pass
    trace = (f'ciel: hAcc={hacc/1000:.1f} m\n'
             f'seuil {sous} m -> {i1} ({t1:.0f}s)\n'
             f'seuil {sur} m -> {i2} ({t2:.0f}s)\n' + '\n'.join((l1 + l2)[:6]))
    if i1 == 'fix':
        return r.record(case, 'FAIL',
                        f'seuil a {sous} m mais une position a {h1/1000:.1f} m est ACCEPTEE: '
                        'le filtre hAcc ne filtre pas', trace)
    if i2 != 'fix':
        return r.record(case, 'FAIL',
                        f'seuil desserre a {sur} m et pourtant aucune position ({i2}): '
                        'le filtre rejette tout', trace)
    r.record(case, 'PASS',
             f'rejette a {sous} m, accepte a {sur} m — le filtre discrimine bien', trace)

def c_ciel_fix_chaud(r, case):
    """Le second fix doit etre plus rapide: l ephemeride est conservee.

    Contrepartie mesurable de la mort GNSS a deux jours. On rend les DEUX temps,
    parce que le rapport est la vraie information — pas le fait binaire.
    """
    b = r.b
    try:
        _ciel(b, GNSS_DEEP_IDLE_AFTER_OFF_S=30)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    i1, h1, n1, t1, _ = _session(b, 300)
    if i1 != 'fix':
        try:
            _ciel(b)
        except Exception:
            pass
        return r.record(case, 'ERROR', f'pas de premier fix (issue={i1}) — cas non evaluable')
    time.sleep(40)
    i2, h2, n2, t2, l2 = _session(b, 300)
    try:
        _ciel(b)
    except Exception:
        pass
    trace = f'froid: {t1:.0f}s ({n1} sat)\nchaud: {t2:.0f}s ({n2} sat)\n' + '\n'.join(l2[:4])
    if i2 != 'fix':
        return r.record(case, 'FAIL',
                        'la seconde session ne fixe plus alors que la premiere a reussi: '
                        'rien n est conserve entre sessions', trace)
    r.record(case, 'PASS', f'froid {t1:.0f} s, chaud {t2:.0f} s', trace)

def c_ciel_emission_reelle(r, case):
    """Bout en bout: ciel -> pilote -> pile -> encodage -> KIM2 -> antenne."""
    b = r.b
    try:
        _ciel(b, ARGOS_MODE=2, TR_NOM=60, NTRY_PER_MESSAGE=1,
              ARGOS_DEPTH_PILE=1, DUTY_CYCLE=16777215)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    mk = b.mark()
    issue, hacc, numsv, dt, _ = _session(b, 300, depuis=mk)
    if issue != 'fix':
        try:
            _ciel(b)
        except Exception:
            pass
        return r.record(case, 'ERROR', f'pas de position (issue={issue}) — emission non evaluable')
    vues = _compter_trace(b, [r'TX SUCCESS', r'\+ERROR=', r'build_\w+_packet'], 150, depuis=mk)
    try:
        _ciel(b)
    except Exception:
        pass
    succes = [l for l in vues if 'TX SUCCESS' in l]
    erreurs = [l for l in vues if '+ERROR=' in l]
    trace = f'hAcc={hacc/1000:.1f} m, {numsv} sat\n' + '\n'.join(vues[:6])
    if erreurs and not succes:
        return r.record(case, 'FAIL', f'emission refusee: {erreurs[0][:70]}', trace)
    if not succes:
        return r.record(case, 'FAIL',
                        'position reelle acquise mais aucune emission aboutie', trace)
    r.record(case, 'PASS',
             f'{len(succes)} emission(s) avec position reelle ({hacc/1000:.1f} m)', trace)

CASES_V30 = [
    dict(id='GPS-C1', risque='BLOQUANT', titre='Position de reference: precision, satellites, temps',
         fn=c_ciel_reference),
    dict(id='GPS-C2', risque='BLOQUANT', titre='Filtre hAcc a la frontiere: rejette sous, accepte au-dessus',
         fn=c_ciel_hacc_frontiere),
    dict(id='GPS-C3', risque='MAJEUR',   titre='MIN_CNO au maximum echoue proprement',
         fn=c_ciel_cno_max),
    dict(id='GPS-C4', risque='MAJEUR',   titre='MIN_ELEV au zenith echoue proprement',
         fn=c_ciel_elev_max),
    dict(id='GPS-C5', risque='MAJEUR',   titre='GPS seul fixe ou le dit',
         fn=c_ciel_mask_gps_seul),
    dict(id='GPS-C6', risque='MAJEUR',   titre='Fenetre d acquisition minimale conclut',
         fn=c_ciel_acq_minimal),
    dict(id='GPS-C7', risque='MAJEUR',   titre='Le fix a chaud est plus rapide que le froid',
         fn=c_ciel_fix_chaud),
    dict(id='GPS-C8', risque='BLOQUANT', titre='Une position reelle part par satellite',
         fn=c_ciel_emission_reelle),
]


for _c in CASES_V30:
    _c['ciel'] = True


# =====================================================================
#  Vague 31 — les templates de deploiement, poses sur la vraie carte
# =====================================================================

TEMPLATES = ('turtle_doppler_only', 'turtle_gps', 'turtle_cloudlocate',
             'drifter', 'fix_beacon', 'rspb_avian_mortality_cyprus_boat')

_CLES_IMPL = None    # None = pas encore lu; sinon set() des cles que ce build porte


def cles_implementees(b):
    """L ensemble des cles que CE build porte, lu une fois par passe.

    PARML est la seule reponse qui filtre sur is_implemented depuis toujours —
    y compris avant le correctif du 2026-08-30 sur les lectures par cle. C est
    donc la source de verite sur ce que la carte porte, et elle repond a TOUTES
    les questions de capacite d un seul aller-retour: type de carte, presence de
    l accelerometre, du SMD, du LoRa.

    POURQUOI ELLE REMPLACE LES SONDES PAR PARMR. _est_rspb() demandait RSP01 par
    PARMR et le recevait sur n importe quelle carte, parce que la lecture par
    cle ne consultait pas is_implemented. Elle a donc declare RSPB une carte
    LinkIt SMD, et SHUT-02 a conclu l inverse de la verite. Le firmware est
    corrige, mais une sonde qui ne depend pas de la version du firmware qu elle
    interroge vaut mieux qu une sonde qui en depend.

    Rend un set vide si PARML ne repond pas — l appelant ne doit alors rien
    conclure, et les aides ci-dessous ne sautent rien dans ce cas.
    """
    global _CLES_IMPL
    if _CLES_IMPL is not None:
        return _CLES_IMPL
    _CLES_IMPL = set()
    try:
        # b.dte() ne rend qu un groupe (\$([ON]);CMD#): la charge utile se lit sur
        # la ligne entiere. Un m.group(3) y leve IndexError, l except plus bas
        # l avale, et la sonde rend un ensemble vide sans que rien ne le dise —
        # elle ne saute alors plus rien, ce qui est le bon defaut mais la rend
        # inutile. Verifie sur la carte le 2026-08-30.
        m = b.dte('PARML', '', timeout=12.0)
        ligne = (m.string if m and hasattr(m, 'string') else '') or ''
        mm = re.search(r'\$O;PARML#[0-9A-Fa-f]{3};(.*)', ligne)
        if mm:
            _CLES_IMPL = {k.strip() for k in mm.group(1).rstrip('\r').split(',') if k.strip()}
    except Exception:
        pass
    return _CLES_IMPL


def saute_si_capacite_absente(r, case, cle, quoi):
    """SKIP motive si `cle` n est pas implementee sur ce build.

    Ne saute JAMAIS quand PARML n a pas repondu: on ne transforme pas une
    absence de mesure en absence de capacite.
    """
    clefs = cles_implementees(r.b)
    if not clefs or cle in clefs:
        return False
    r.record(case, 'SKIP',
             f'{cle} n est pas implemente sur ce build — {quoi} n existe pas sur cette carte')
    return True


def _est_rspb(b):
    """La carte est-elle un build RSPB ?

    RSPB_PACKET_FORMAT (RSP01) n existe que derriere HAS_BOARD_RSPB: sur un
    build KIM2 il n est pas implemente et n apparait pas dans la reponse. C est
    le discriminant le plus direct, et il vient de la carte plutot que d une
    supposition de notre part.
    """
    return 'RSP01' in cles_implementees(b)

def _encode_template(nom):
    """Rend [(cle, valeur_fil)] pour un template, via l encodeur PyLinkit.

    C est PyLinkit qui pousse ces fichiers en production, donc c est son
    encodeur qui fait foi: plusieurs codecs TRANSFORMENT la valeur au lieu de
    la chercher dans une table (le cycle de service est lu en hexadecimal), et
    reimplementer cette conversion cote banc reviendrait a tester notre copie
    plutot que le chemin reel.
    """
    import configparser
    chemin = os.path.join(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))), 'template_conf', f'{nom}.cfg')
    cfg = configparser.RawConfigParser()
    cfg.optionxform = lambda o: o
    cfg.read(chemin)
    params = dict(cfg['PARAM'])

    racine = os.environ.get('PYLINKIT')
    if not racine:
        raise RuntimeError('PYLINKIT non defini — impossible d encoder comme le fait '
                           'la production; cas non concluant')
    if racine not in sys.path:
        sys.path.insert(0, racine)
    from pylinkit.protocol.dte_params import DTEParamMap
    return [(DTEParamMap.param_to_key(k), DTEParamMap.encode(k, v))
            for k, v in params.items()]


def _pousser_template(b, paires, taille=10):
    """PARMW par tranches. Rend la liste des cles refusees."""
    refuses = []
    for i in range(0, len(paires), taille):
        tranche = paires[i:i + taille]
        charge = ','.join(f'{k}={v}' for k, v in tranche)
        mk = b.mark()
        b._send(f'$PARMW#{len(charge):03X};{charge}\r')
        m = b.expect(r'\$([ON]);PARMW#([0-9A-Fa-f]{3});?(.*)$', 12.0, from_idx=mk)
        if not m:
            refuses.append(f'(pas de reponse sur la tranche {i // taille + 1})')
        elif m.group(1) == 'N':
            refuses.extend(x for x in m.group(3).rstrip('\r').split(',') if x)
    return refuses


def _cas_template(nom):
    def cas(r, case):
        b = r.b
        try:
            paires = _encode_template(nom)
        except Exception as e:
            return r.record(case, 'SKIP', f'template non encodable: {e}')
        if not _en_config(b):
            return r.record(case, 'ERROR', 'mode configuration inaccessible')
        # Le format .cfg est generique (pylinkit --parmw sert toute la famille);
        # ce qui ne l est pas, c est d ECRIRE un profil entier sur la carte de
        # banc. Sur une KIM cela reconfigure la carte meme qu on est en train de
        # valider, et ne represente aucun deploiement reel. On constate le type
        # de carte plutot que de le supposer.
        if not _est_rspb(b):
            try:
                b.exit_config()
            except Exception:
                pass
            return r.record(case, 'SKIP',
                            'carte non-RSPB (RSP01 absent): ecrire un profil entier sur la '
                            'carte en cours de validation ne represente aucun deploiement reel')
        # L identite de la carte de banc est restauree a la fin: un template
        # ecrit PROFILE_NAME et DEVICE_MODEL, et laisser "TURTLE-DOPPLER" sur
        # la carte ferait mentir tous les cas suivants qui lisent son modele.
        _, identite = b.read_params(['PROFILE_NAME', 'DEVICE_MODEL'], timeout=12.0)
        try:
            refuses = _pousser_template(b, paires)
        except Exception as e:
            return r.record(case, 'ERROR', f'ecriture impossible: {type(e).__name__}: {e}')
        if refuses:
            return r.record(case, 'FAIL',
                            f'{len(refuses)} cle(s) refusee(s) par la balise: '
                            f'{", ".join(refuses[:8])}',
                            f'{len(paires)} cles poussees')

        # Relire: une cle acceptee mais ecrasee par un service ne se voit pas
        # dans la reponse PARMW. On ne relit que les cles numeriques, les
        # champs texte revenant parfois normalises.
        cles = [k for k, _ in paires]
        ecarts = []
        for i in range(0, len(cles), 12):
            _, lus = b.read_params(cles[i:i + 12], timeout=12.0)
            for k, v in paires[i:i + 12]:
                if k in lus and lus[k].strip() != str(v).strip():
                    ecarts.append(f'{k}: pose {v}, relu {lus[k]}')
        try:
            if identite:
                b.write_params({k: v for k, v in identite.items()}, timeout=12.0)
            b.exit_config()
        except Exception:
            pass
        if ecarts:
            return r.record(case, 'FAIL',
                            f'{len(ecarts)} parametre(s) relu(s) differents: '
                            f'{"; ".join(ecarts[:6])}')
        r.record(case, 'PASS',
                 f'{len(paires)} parametres poses et relus identiques',
                 f'template {nom}')
    return cas


# Vague RSPB: sautee d elle-meme sur une carte KIM2, ou elle ne represente
# aucun deploiement reel.
CASES_V31 = [
    dict(id=f'TPL-{i:02d}', risque='MAJEUR',
         titre=f'Le template {nom} se pose entierement sur la carte (RSPB)',
         fn=_cas_template(nom))
    for i, nom in enumerate(TEMPLATES, 1)
]


def c_modulations_provisionnees(r, case):
    """Quelles modulations le module porte reellement — et ce que ca implique.

    refresh_modulation_availability() ne considere une modulation disponible
    que si sa RCONF fait exactement 32 caracteres. C est ce qui decide si un
    paquet LONG (192 bits) peut partir: LDK n en porte que 128, et sur un
    module LDK seul le firmware plafonne a une position par emission. La pile
    de profondeur ne fait alors rien gagner, et un budget batterie calcule sur
    "trois positions par emission" est faux d un facteur trois.

    Ce cas ne juge pas: il constate, parce que la reponse change ce qu on peut
    promettre a l operateur.
    """
    b = r.b
    if not _en_config(b):
        return r.record(case, 'ERROR', 'mode configuration inaccessible')
    try:
        _, lus = b.read_params(['ARGOS_RADIOCONF_LDK', 'ARGOS_RADIOCONF_LDA2',
                                'ARGOS_RADIOCONF_VLDA4', 'ARGOS_CACHED_MODULATION',
                                'ARGOS_ADAPTIVE_MODULATION'], timeout=12.0)
        b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'lecture impossible: {type(e).__name__}: {e}')

    def porte(cle):
        v = (lus.get(cle) or '').strip()
        return len(v) == 32

    ldk, lda2, vlda4 = porte('ARP51'), porte('ARP52'), porte('ARP53')
    cache = lus.get('SMP01', '?')
    adapt = lus.get('ARP54', '?')
    trace = (f'LDK={ldk} LDA2={lda2} VLDA4={vlda4} '
             f'modulation en cache={cache} ({_nom_cache(cache)}) adaptative={adapt}')

    if not (ldk or lda2 or vlda4):
        return r.record(case, 'FAIL',
                        'aucune RCONF de 32 caracteres: le module ne peut emettre dans '
                        'aucune modulation', trace)

    # PROVISIONNE n est pas UTILISE. Un LONG de 192 bits ne part que si la
    # modulation EFFECTIVE est LDA2, c est-a-dire si la RCONF maitresse decode
    # en LDA2, ou si l adaptatif autorise le service a y basculer. Une carte
    # peut tres bien porter les trois RCONF et n emettre qu en LDK.
    maitresse_lda2 = (cache == _CACHE_LDA2)
    adaptatif = (adapt == '1')
    long_possible = lda2 and (maitresse_lda2 or adaptatif)

    if long_possible:
        pourquoi = 'maitresse en LDA2' if maitresse_lda2 else 'adaptatif autorise la bascule'
        return r.record(case, 'PASS',
                        f'LDA2 provisionne ET effectivement atteignable ({pourquoi}): un '
                        'paquet LONG de 192 bits peut partir, la pile de profondeur '
                        'rapporte jusqu a 3 positions par emission', trace)
    if lda2:
        return r.record(case, 'PASS',
                        f'LDA2 provisionne mais INUTILISE: la maitresse decode en '
                        f'{_nom_cache(cache)} et l adaptatif est eteint. Aucun paquet LONG '
                        'ne peut partir tel quel — ARGOS_DEPTH_PILE > 1 ne fait rien gagner '
                        'tant que ARGOS_ADAPTIVE_MODULATION reste a 0.', trace)
    r.record(case, 'PASS',
             'LDK seul: pas de paquet LONG possible, chaque position coute une emission '
             'entiere. ARGOS_DEPTH_PILE > 1 ne fait rien gagner sur ce module.', trace)


CASES_V31.append(dict(id='PROV-01', risque='MAJEUR',
                      titre='Modulations reellement provisionnees dans le module',
                      fn=c_modulations_provisionnees))


def c_premier_message_apres_boot(r, case):
    """Le PREMIER message apres un demarrage a froid est dimensionne juste.

    C est la garantie que le cache de modulation existe pour tenir. La RCONF
    maitresse est du hex chiffre que le firmware ne sait PAS decoder: seul le
    module le peut (AT+RCONF=?). Or le service demande get_current_modulation()
    pour dimensionner la trame AVANT que le module soit allume et interroge.
    Sans le cache il partait sur LDA2 par defaut, et sur une unite dont la
    maitresse encode LDK cela donnait une trame de 12 octets a une radio LDK
    figee a 16 -> +ERROR=5, une emission perdue A CHAQUE DEMARRAGE.

    On regarde donc, apres un redemarrage volontaire et sur la toute premiere
    emission:
      - aucun "TX mode != RCONF mode, realigning" (le symptome exact),
      - aucun +ERROR=5,
      - une emission qui aboutit.

    PORTEE: ceci valide la modulation reellement provisionnee dans CE module,
    pas les trois. Valider LDK, LDA2 et VLDA4 demanderait leurs RCONF
    respectives, qui sont des credentials.
    """
    if saute_si_radio_muette(r, case, 'une emission reussie'):
        return
    b = r.b
    try:
        b.enter_config()
        _, avant = b.read_params(['ARGOS_CACHED_MODULATION', 'ARGOS_ADAPTIVE_MODULATION'])
        b.exit_config()
    except Exception as e:
        return r.record(case, 'ERROR', f'lecture impossible: {type(e).__name__}: {e}')
    cache_avant = avant.get('SMP01', '?')

    # DOPPLER sans GNSS: la premiere emission part vite et ne depend d aucun fix.
    try:
        _config_mode(b, 4, GNSS_EN=0, TR_NOM=30, ARGOS_ADAPTIVE_MODULATION=0)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')

    # RSTBW, pas %BOOT: %BOOT ne fait que LIRE le compteur d echecs, il ne
    # redemarre rien. Le lien USB tombe au redemarrage, c est attendu.
    try:
        b.enter_config()
        b.dte('RSTBW', '', timeout=8.0)
    except Exception as e:
        return r.record(case, 'ERROR', f'RSTBW impossible: {type(e).__name__}: {e}')
    time.sleep(6)
    if not r.connect():
        return r.record(case, 'ERROR', 'la carte ne revient pas apres RSTBW')
    b = r.b
    mk = b.mark()
    if not b.wait_state('OPERATIONAL', timeout=90):
        return r.record(case, 'ERROR', 'la carte ne repasse pas en OPERATIONAL apres RSTBW')
    vues = _compter_trace(b, [r'TX SUCCESS', r'\+ERROR=', r'realigning',
                              r'resync_rconf_cache'], 180, depuis=mk)
    try:
        b.enter_config()
        _, apres = b.read_params(['ARGOS_CACHED_MODULATION'])
        b.write_params({'ARGOS_MODE': 0})
        b.exit_config()
    except Exception:
        apres = {}

    realign = [l for l in vues if 'realigning' in l]
    erreurs = [l for l in vues if '+ERROR=' in l]
    succes = [l for l in vues if 'TX SUCCESS' in l]
    corrige = [l for l in vues if 'resync_rconf_cache: cache' in l]
    trace = (f'cache avant={cache_avant} ({_nom_cache(cache_avant)}) '
             f'apres={apres.get("SMP01", "?")} ({_nom_cache(apres.get("SMP01"))})\n'
             + '\n'.join(vues[:8]))

    if realign:
        return r.record(case, 'FAIL',
                        'la premiere emission apres demarrage a du realigner la modulation: '
                        'la trame etait dimensionnee pour la mauvaise', trace)
    if erreurs and not succes:
        return r.record(case, 'FAIL',
                        f'premiere emission refusee: {erreurs[0][-60:]}', trace)
    if not succes:
        return r.record(case, 'ERROR',
                        'aucune emission observee en 3 min — cas non concluant', trace)
    detail = (f'premiere emission juste en {_nom_cache(apres.get("SMP01", cache_avant))}, '
              f'sans realignement ni +ERROR=5')
    if corrige:
        detail += ' (le cache etait perime et a ete corrige au demarrage)'
    r.record(case, 'PASS', detail, trace)


CASES_V29.append(dict(id='MOD-06', risque='BLOQUANT',
                      titre='Le premier message apres demarrage est dimensionne juste',
                      fn=c_premier_message_apres_boot))


# =====================================================================
#  Vague 32 — BLIND, extinction, et les commandes DTE encore decouvertes
# =====================================================================

def c_blind_charge_dans_le_module(r, case):
    """BLIND: le module recoit bien la config de rafale et l annonce.

    BLIND confie la redondance au module: AT+KMAC=2 lui passe {retx_nb,
    nb_parallel=1, retx_period_s, per_offset}, et il repete tout seul. Le
    firmware ne voit alors qu un seul +TX, a la FIN de la rafale — c est
    pourquoi le timeout de securite est allonge de retx_nb * retx_period_s.

    Si AT+KMAC=2 est refuse le firmware retombe en BASIC et le DIT: la balise
    emet toujours, mais une seule fois par cycle. Le silence serait pire.
    """
    if saute_si_radio_muette(r, case, 'le dialogue avec le module'):
        return
    b = r.b
    N, P = 3, 60
    try:
        _config_mode(b, 2, GNSS_EN=0, TR_NOM=60, ARGOS_BLIND_EN=1,
                     ARGOS_BLIND_RETX_NB=N, ARGOS_BLIND_RETX_PERIOD_S=P)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    mk = b.mark()
    vues = _compter_trace(b, [r'BLIND loaded', r'BLIND \(AT\+KMAC=2\) rejected',
                              r'BLIND — TX window', r'TX SUCCESS'], 150, depuis=mk)
    try:
        b.enter_config(); b.write_params({'ARGOS_MODE': 0, 'ARGOS_BLIND_EN': 0}); b.exit_config()
    except Exception:
        pass
    trace = '\n'.join(vues[:6])
    charge = [l for l in vues if 'BLIND loaded' in l]
    refus = [l for l in vues if 'rejected' in l]
    if refus and not charge:
        return r.record(case, 'SKIP',
                        'le module refuse AT+KMAC=2 (BLIND non supporte par ce firmware '
                        'KIM2) et le firmware retombe en BASIC en le disant — pas de '
                        'silence, mais le mode ne peut pas etre valide ici', trace)
    if not charge:
        return r.record(case, 'ERROR', 'ni chargement ni refus BLIND observe en 2.5 min', trace)
    # Les valeurs annoncees doivent etre CELLES qu on a posees.
    import re as _re
    m = _re.search(r'retx_nb=(\d+) period=(\d+)s', charge[0])
    if not m:
        return r.record(case, 'FAIL', f'trace BLIND illisible: {charge[0][-70:]}', trace)
    if (int(m.group(1)), int(m.group(2))) != (N, P):
        return r.record(case, 'FAIL',
                        f'BLIND charge avec retx_nb={m.group(1)} period={m.group(2)} '
                        f'au lieu de {N}/{P}', trace)
    r.record(case, 'PASS', f'BLIND charge dans le module avec retx_nb={N} period={P}s', trace)


def c_blind_exclu_de_la_salve(r, case):
    """BLIND doit etre NEUTRALISE en SURFACING_BURST et en DOPPLER.

    Ces modes font deja leur propre cascade de repetitions. Y ajouter une
    rafale detenue par le module ferait emettre DEUX fois chaque message —
    budget satellite double, et une balise qui parle par-dessus elle-meme.
    get_argos_configuration() efface blind_en pour ces deux modes; le cas
    verifie que le module ne recoit alors AUCUNE config de rafale.
    """
    if saute_si_radio_muette(r, case, 'le dialogue avec le module'):
        return
    b = r.b
    try:
        _config_mode(b, 5, GNSS_EN=0, TR_NOM=60, ARGOS_BLIND_EN=1,
                     ARGOS_BLIND_RETX_NB=3, ARGOS_BLIND_RETX_PERIOD_S=60,
                     UNDERWATER_EN=0)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    mk = b.mark()
    vues = _compter_trace(b, [r'BLIND loaded', r'BLIND — TX window', r'TX SUCCESS'],
                          120, depuis=mk)
    try:
        b.enter_config(); b.write_params({'ARGOS_MODE': 0, 'ARGOS_BLIND_EN': 0}); b.exit_config()
    except Exception:
        pass
    trace = '\n'.join(vues[:6])
    if [l for l in vues if 'BLIND loaded' in l or 'BLIND — TX window' in l]:
        return r.record(case, 'FAIL',
                        'BLIND est charge alors que le mode est SURFACING_BURST: chaque '
                        'message partirait deux fois', trace)
    r.record(case, 'PASS',
             'BLIND correctement neutralise en SURFACING_BURST (aucune rafale confiee '
             'au module)', trace)


def c_budget_session_ignore_sur_kim(r, case):
    """SHUTDOWN_NTIME_SAT est accepte sur KIM2 mais n y eteint RIEN — et le dit.

    Le parametre n a de sens que sur RSPB, ou le TPL5111 coupe l alimentation a
    chaque cycle: "session" y vaut un cycle et le compteur repart au boot
    suivant. Sur une carte qui reste alimentee, m_session_tx_count n est remis a
    zero que dans service_init — le plafond deviendrait donc un plafond A VIE et
    tuerait le deploiement (a TR_NOM=60 s, un budget de 100 tue la balise en
    100 minutes).

    Le firmware l ignore donc ici, et l annonce UNE fois. Ce cas verifie les
    deux moitiés: l avertissement existe, ET l emission continue au-dela du
    budget. Verifier seulement l avertissement laisserait passer une balise qui
    se tait quand meme.
    """
    if saute_si_radio_muette(r, case, 'une emission reussie'):
        return
    b = r.b
    budget = 2
    try:
        _config_mode(b, 4, GNSS_EN=0, TR_NOM=30, SHUTDOWN_NTIME_SAT=budget,
                     SURFACING_BURST_MAX_MSG=0)
    except Exception as e:
        return r.record(case, 'ERROR', f'configuration impossible: {type(e).__name__}: {e}')
    mk = b.mark()
    vues = _compter_trace(b, [r'TX SUCCESS', r'session TX budget reached'], 210, depuis=mk)
    try:
        b.enter_config(); b.write_params({'ARGOS_MODE': 0, 'SHUTDOWN_NTIME_SAT': 0}); b.exit_config()
    except Exception:
        pass
    emissions = [l for l in vues if 'TX SUCCESS' in l]
    avert = [l for l in vues if 'session TX budget reached' in l]
    trace = f'{len(emissions)} emissions, {len(avert)} avertissement(s)\n' + '\n'.join(vues[:8])
    if len(emissions) <= budget:
        return r.record(case, 'ERROR',
                        f'seulement {len(emissions)} emission(s) pour un budget de {budget}: '
                        'le depassement n a pas ete atteint, cas non concluant', trace)
    if not avert:
        return r.record(case, 'FAIL',
                        f'{len(emissions)} emissions au-dela du budget de {budget}, mais AUCUN '
                        'avertissement: le parametre est ignore en silence', trace)
    if len(avert) > 1:
        return r.record(case, 'FAIL',
                        f'{len(avert)} avertissements: le message doit etre emis UNE fois, '
                        'pas a chaque cycle', trace)
    r.record(case, 'PASS',
             f'budget de {budget} depasse ({len(emissions)} emissions), ignore avec un seul '
             'avertissement — la balise ne se tait pas', trace)


def _cas_commande_simple(cmd, charge, motif, quoi):
    """Une commande DTE doit REPONDRE — pas se taire — et etre lisible."""
    def cas(r, case):
        b = r.b
        if not _en_config(b):
            return r.record(case, 'ERROR', 'mode configuration inaccessible')
        try:
            m = b.dte(cmd, charge, timeout=20.0)
        except Exception as e:
            return r.record(case, 'ERROR', f'{cmd} impossible: {type(e).__name__}: {e}')
        finally:
            try:
                b.exit_config()
            except Exception:
                pass
        if not m:
            return r.record(case, 'FAIL',
                            f'{cmd} sans reponse: le port se tait sur une commande bien '
                            f'formee, ce qu un operateur ne distingue pas d une carte morte')
        ligne = (m.string if hasattr(m, 'string') else '') or ''
        if motif and not re.search(motif, ligne):
            return r.record(case, 'FAIL', f'{cmd} repond hors format: {ligne[:80]}', ligne[:200])
        r.record(case, 'PASS', f'{cmd} ({quoi}) repond: {ligne[:70]}', ligne[:200])
    return cas


def c_budget_session_interdit_hors_rspb(r, case):
    """SHUTDOWN_NTIME_SAT n existe QUE sur RSPB — la regle, pas sa consequence.

    POURQUOI CE CAS EXISTE A COTE DE SHUT-01. SHUT-01 eprouve la CONSEQUENCE du
    budget de session: il compte les emissions et verifie qu elles ne s arretent
    pas. C est le bon test du comportement, mais il exige une radio vivante, donc
    il ne conclut pas sur une carte au module muet. Or la regle demandee est plus
    simple et plus forte que sa consequence: le parametre ne doit pas EXISTER
    hors RSPB. Cela se verifie sans emettre un seul octet.

    Ce que le firmware promet (core/protocol/dte_params.cpp): PWP05 et LBP14
    portent HAS_BOARD_RSPB. Un parametre non implemente n est pas liste par
    PARMR et doit etre refuse par PARMW. Si l un des deux passait, un operateur
    pourrait poser sur une carte KIM2 un budget de session qu elle ignore — il
    croirait la balise bornee alors qu elle emet sans limite.

    Le cas s adapte a la carte: sur RSPB les deux parametres DOIVENT exister,
    et un refus y serait tout aussi grave (le budget deviendrait impossible a
    poser sur la seule carte qui l honore).
    """
    b = r.b
    if not _en_config(b):
        return r.record(case, 'ERROR', 'mode configuration inaccessible')
    rspb = _est_rspb(b)
    try:
        _, lus = b.read_params(['SHUTDOWN_NTIME_SAT', 'LB_SHUTDOWN_NTIME_SAT'], timeout=12.0)
    except Exception as e:
        return r.record(case, 'ERROR', f'PARMR injoignable: {type(e).__name__}: {e}')
    presents = [k for k in ('PWP05', 'LBP14') if k in lus]

    m = b.write_params({'SHUTDOWN_NTIME_SAT': 5}, timeout=12.0, strict=False)
    ligne = (m.string if m and hasattr(m, 'string') else '') or ''
    ecriture_refusee = bool(m) and m.group(1) == 'N'
    trace = f'PARMR presents={presents or "aucun"} | PARMW -> {ligne[:90]}'

    if rspb:
        if len(presents) == 2 and not ecriture_refusee:
            return r.record(case, 'PASS',
                            'carte RSPB: le budget de session existe et s ecrit', trace)
        return r.record(case, 'FAIL',
                        'carte RSPB: le budget de session devrait exister et s ecrire — '
                        f'presents={presents}, ecriture refusee={ecriture_refusee}', trace)

    if presents:
        return r.record(case, 'FAIL',
                        f'hors RSPB, PARMR expose encore {",".join(presents)} — '
                        'un operateur croirait la balise bornee', trace)
    if not ecriture_refusee:
        return r.record(case, 'FAIL',
                        'hors RSPB, PARMW ACCEPTE SHUTDOWN_NTIME_SAT: le budget est '
                        'posable mais jamais honore', trace)
    r.record(case, 'PASS',
             'hors RSPB: absent de PARMR et refuse par PARMW, comme voulu', trace)


CASES_V32 = [
    dict(id='BLIND-01', risque='MAJEUR',
         titre='BLIND: la rafale est confiee au module avec les bonnes valeurs',
         fn=c_blind_charge_dans_le_module),
    dict(id='BLIND-02', risque='BLOQUANT',
         titre='BLIND est neutralise en SURFACING_BURST (sinon double emission)',
         fn=c_blind_exclu_de_la_salve),
    dict(id='SHUT-01', risque='BLOQUANT',
         titre='SHUTDOWN_NTIME_SAT est ignore sur KIM2, et le dit une fois',
         fn=c_budget_session_ignore_sur_kim),
    dict(id='SHUT-02', risque='BLOQUANT',
         titre='SHUTDOWN_NTIME_SAT n est pas posable hors RSPB (sans radio)',
         fn=c_budget_session_interdit_hors_rspb),
    # Les quatre commandes DTE qui n etaient citees nulle part dans la campagne.
    # On n en teste que la REPONSE: SWSCAL et SWSTST demandent une electrode
    # qu on mouille a la main, GNSSBCKP immobilise le rail GNSS, et SMDDFU n a
    # que son action VERSION de disponible hors SMD.
    dict(id='CMD-40', risque='MAJEUR', titre='SWSTST repond (mode test du detecteur d eau)',
         fn=_cas_commande_simple('SWSTST', '1', r'\$[ON];SWSTST', 'demarrage du mode test')),
    dict(id='CMD-41', risque='MAJEUR', titre='SWSCAL repond (calibration guidee)',
         fn=_cas_commande_simple('SWSCAL', '1', r'\$[ON];SWSCAL', 'annulation de calibration')),
    dict(id='CMD-42', risque='MAJEUR', titre='GNSSBCKP repond (charge de la pile de sauvegarde)',
         fn=_cas_commande_simple('GNSSBCKP', '0', r'\$[ON];GNSSBCKP', 'abandon, duree 0')),
    dict(id='CMD-43', risque='MAJEUR', titre='SMDDFU VERSION repond sur un build KIM2',
         radio=True,
         fn=_cas_commande_simple('SMDDFU', '5', r'\$[ON];SMDDFU', 'action VERSION')),
]
