#!/usr/bin/env python3
"""Aspire system.log de la carte et l analyse hors ligne.

POURQUOI CET OUTIL EXISTE
-------------------------
Le lien USB-over-IP ne tient pas plusieurs heures: sur ce banc il decroche
regulierement, et cinq debranchements physiques ont ete necessaires en deux
jours. Un essai d endurance qui dependrait du lien ne finirait jamais.

Mais la balise, elle, journalise sur sa flash externe. Un essai long n a donc
pas besoin du lien PENDANT qu il tourne: on configure, on laisse courir des
heures, on reconnecte, et on aspire. Le lien redevient un simple moyen de
recuperation, pas une condition de l essai.

CE QU IL FAUT SAVOIR SUR DUMPD
------------------------------
Une requete declenche un FLUX de paquets, pas une reponse unique. Les index
sont HEXADECIMAUX. La charge est du base64. Et le DTE ne repond QU EN MODE
CONFIGURATION.
"""
import argparse
import base64
import re
import sys
import time

sys.path.insert(0, __file__.rsplit('/', 1)[0])
from kim_bench import Bench

RE_PKT = re.compile(r'\$O;DUMPD#[0-9A-Fa-f]+;([0-9A-Fa-f]+),([0-9A-Fa-f]+),(\S*)')

# Les noms du protocole, pas les miens. Le type 1 est GNSS_SENSOR et le
# fichier s appelle sensor.log: c est le dump des DONNEES CAPTEUR, et c est la
# que vivent les colonnes ttff / onTime / numSV / hAcc. 'gnss' reste accepte
# comme alias parce que c est ce qu on cherche dedans.
LOGS = {'system': 0, 'sensor': 1, 'gnss': 1, 'sws': 11}


def harvest(b, d_type=0, plafond=4000, silence=12.0):
    """Aspire un journal. Rend (texte, nb_paquets, mmm, tronque)."""
    mk = b.mark()
    b._send(f'$DUMPD#001;{d_type}\r')
    paquets, mmm = {}, None
    dernier = time.time()
    while time.time() - dernier < silence and len(paquets) < plafond:
        time.sleep(1.0)
        with b._lock:
            lignes = [l for _, l in b.history[mk:]]
        avant = len(paquets)
        for l in lignes:
            m = RE_PKT.search(l)
            if m:
                idx = int(m.group(1), 16)
                mmm = int(m.group(2), 16)
                paquets[idx] = m.group(3)
        if len(paquets) > avant:
            dernier = time.time()
        if mmm is not None and mmm in paquets:
            break
    texte = ''
    for i in sorted(paquets):
        try:
            texte += base64.b64decode(paquets[i] + '===').decode('utf-8', 'replace')
        except Exception:
            pass
    tronque = mmm is not None and (mmm + 1) > len(paquets)
    return texte, len(paquets), mmm, tronque


# --- analyses ---------------------------------------------------------------

def analyse(texte):
    """Rend un bilan chiffre de ce que le journal raconte."""
    lignes = texte.splitlines()
    def compte(motif):
        return sum(1 for l in lignes if re.search(motif, l))
    bilan = {
        'lignes':            len(lignes),
        'tx_succes':         compte(r'TX SUCCESS'),
        'tx_erreurs':        compte(r'\+ERROR='),
        'fix_reels':         compte(r'task_process_gnss_data'),
        'sessions_gnss':     compte(r'M10Q on —'),
        'sans_fix':          compte(r'acquisition timeout — no fix'),
        'pvt_degrades':      compte(r'degraded PVT'),
        'redemarrages':      compte(r'entry: BootState'),
        'resets_wdt':        compte(r'soft reset|WDT'),
        'echecs_boot':       compte(r'BootFail: counter'),
        'reset_usine':       compte(r'factory_reset'),
        'batterie_critique': compte(r'VoltageCritical|critical'),
        'backoff':           compte(r'backoff|suspension'),
        'limiteur':          compte(r'rate limit reached'),
    }
    return bilan


def analyse_gnss(texte):
    """Bilan chiffre du dump des donnees capteur (type 1, sensor.log).

    C est un CSV, pas des lignes de trace.

    C est ICI que vivent les temps: le journal systeme ne porte aucun ttff, il
    est une colonne du CSV produit par GPSLogFormatter. Chercher un `ttff=` dans
    system.log ne rend donc jamais rien, et on croit la balise muette sur ses
    performances alors qu elle les journalise ailleurs.

    En-tete: log_datetime,batt_voltage,iTOW,fix_datetime,valid,onTime,ttff,
             fixType,flags,flags2,flags3,numSV,lon,lat,height,hMSL,hAcc,...
    """
    lignes = [l for l in texte.splitlines() if l.strip()]
    if not lignes:
        return {'lignes': 0}
    entete = None
    for l in lignes:
        if l.startswith('log_datetime'):
            entete = [c.strip() for c in l.split(',')]
            break
    if entete is None:
        return {'lignes': len(lignes), 'note': 'en-tete CSV absente — journal tronque ?'}

    idx = {nom: i for i, nom in enumerate(entete)}
    fixes = []
    for l in lignes:
        if l.startswith('log_datetime'):
            continue
        cols = l.split(',')
        if len(cols) < len(entete):
            continue
        fixes.append(cols)

    def col(cols, nom, conv=float):
        try:
            return conv(cols[idx[nom]])
        except (KeyError, IndexError, ValueError):
            return None

    def stats(nom, conv=float, valides_seulement=True):
        vals = []
        for c in fixes:
            if valides_seulement and col(c, 'valid', int) != 1:
                continue
            v = col(c, nom, conv)
            if v is not None:
                vals.append(v)
        if not vals:
            return None
        vals.sort()
        return {'n': len(vals), 'min': vals[0], 'median': vals[len(vals) // 2],
                'max': vals[-1], 'moy': sum(vals) / len(vals)}

    valides = sum(1 for c in fixes if col(c, 'valid', int) == 1)
    bilan = {'entrees': len(fixes), 'positions_valides': valides,
             'positions_rejetees': len(fixes) - valides}
    for nom, cle in (('ttff', 'ttff_ms'), ('onTime', 'temps_allume_ms'),
                     ('numSV', 'satellites'), ('hAcc', 'hAcc_mm'), ('hDOP', 'hDOP')):
        st = stats(nom)
        if st:
            bilan[cle] = st
    return bilan


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--log', default='system', choices=sorted(LOGS),
                    help="'sensor' (alias 'gnss') = sensor.log, le dump des donnees "
                         "capteur, ou vivent les temps de fix")
    ap.add_argument('--out', default='/tmp/harvest.log')
    ap.add_argument('--plafond', type=int, default=4000)
    a = ap.parse_args()

    b = Bench(quiet=True)
    b.open()
    print(f'etat: {b.get_state(timeout=12)}')
    b.enter_config()          # le DTE ne repond qu ici
    time.sleep(1)
    t0 = time.time()
    texte, n, mmm, tronque = harvest(b, LOGS[a.log], a.plafond)
    b.exit_config()
    b.close()

    with open(a.out, 'w') as f:
        f.write(texte)
    print(f'{n} paquets sur {(mmm + 1) if mmm is not None else "?"}, '
          f'{len(texte)} octets en {time.time() - t0:.0f} s'
          f'{" (TRONQUE)" if tronque else ""}')
    print(f'ecrit dans {a.out}')
    print('--- bilan ---')
    fonction = analyse_gnss if LOGS[a.log] == 1 else analyse
    for k, v in fonction(texte).items():
        if isinstance(v, dict):
            print(f'  {k:20} n={v["n"]:<5} min={v["min"]:<10.0f} med={v["median"]:<10.0f} '
                  f'moy={v["moy"]:<10.1f} max={v["max"]:.0f}')
        else:
            print(f'  {k:20} {v}')


if __name__ == '__main__':
    main()
