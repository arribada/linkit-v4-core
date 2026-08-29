#!/usr/bin/env python3
"""Essai d endurance long, independant du lien USB.

POURQUOI EN DEUX TEMPS
----------------------
Le lien USB-over-IP de ce banc ne tient pas plusieurs heures. Un essai qui
resterait connecte pendant qu il tourne finirait toujours par mesurer la
sante du lien plutot que celle de la balise.

On coupe donc l essai en deux gestes:

    endurance.py arm --profil doppler   -> configure, verifie, ferme le port
    (des heures passent, le lien peut tomber, la carte s en moque)
    endurance.py collect --depuis <t0>  -> rouvre, aspire system.log, analyse

Entre les deux, la balise journalise sur sa flash externe. Le lien redevient
un moyen de recuperation, pas une condition de l essai.

CE QU ON MESURE
---------------
Pas "est-ce que ca a marche", mais des chiffres qu on peut comparer a ce que
la configuration promettait: combien d emissions pour combien d heures, a
quelle cadence reelle, avec quels trous. Un ecart de plus de 25 % entre la
cadence attendue et la cadence observee est signale — c est le seuil au-dela
duquel un budget batterie calcule sur le papier devient faux.
"""
import argparse
import json
import os
import re
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from kim_bench import Bench
from log_harvest import harvest, LOGS

ETAT = '/tmp/endurance_etat.json'

# Chaque profil: les parametres a poser, et la cadence d emission attendue en
# secondes. La cadence sert de reference a l analyse, elle n est pas devinee.
PROFILS = {
    # Doppler seul: le cas le plus econome et le plus expose au silence
    # definitif, puisque rien d autre ne reveille la balise.
    'doppler': {
        'params': {
            'ARGOS_MODE': 4, 'TR_NOM': 60, 'NTRY_PER_MESSAGE': 0,
            'ARGOS_DEPTH_PILE': 1, 'DUTY_CYCLE': 16777215,
            'GNSS_EN': 0, 'UNDERWATER_EN': 0,
            # 0 = sequence Doppler NON bornee. C est voulu ICI et seulement ici:
            # on veut un flux continu pour mesurer une cadence. Sur un animal,
            # cette valeur le fait emettre en surface jusqu a vider la batterie.
            'SURFACING_BURST_MAX_MSG': 0,
            'LB_EN': 0, 'RATE_LIMIT_EN': 0, 'LED_MODE': 0,
            'SAT_PREPASS_EN': 0, 'ARGOS_RX_EN': 0,
        },
        'periode_s': 60,
        'quoi': 'Doppler seul, une emission par minute, sans GNSS ni immersion',
    },
    # Doppler + GNSS: la sequence complete acquisition -> emission, celle qui
    # consomme le plus et ou les deux services se disputent l ordonnanceur.
    'gnss': {
        'params': {
            'ARGOS_MODE': 2, 'TR_NOM': 120, 'NTRY_PER_MESSAGE': 0,
            'ARGOS_DEPTH_PILE': 4, 'DUTY_CYCLE': 16777215,
            'GNSS_EN': 1, 'DLOC_ARG_NOM': 13, 'GNSS_ACQ_TIMEOUT': 120,
            'GNSS_COLD_ACQ_TIMEOUT': 300, 'GNSS_SESSION_SINGLE_FIX': 1,
            'UNDERWATER_EN': 0, 'LB_EN': 0, 'RATE_LIMIT_EN': 0, 'LED_MODE': 0,
        },
        'periode_s': 120,
        'quoi': 'LEGACY + GNSS toutes les 5 min, emission toutes les 2 min',
    },
    # Limiteur actif: on verifie qu il borne sans jamais faire taire.
    'limiteur': {
        'params': {
            'ARGOS_MODE': 4, 'TR_NOM': 30, 'NTRY_PER_MESSAGE': 0,
            'ARGOS_DEPTH_PILE': 1, 'DUTY_CYCLE': 16777215,
            'GNSS_EN': 0, 'UNDERWATER_EN': 0, 'LB_EN': 0, 'LED_MODE': 0,
            'RATE_LIMIT_EN': 1, 'RATE_LIMIT_WINDOW_S': 3600,
            'RATE_LIMIT_MAX_TX': 40,
        },
        'periode_s': 90,   # 40 emissions par heure = une toutes les 90 s
        'quoi': 'Doppler a 30 s mais limite a 40 emissions par heure',
    },
}


def arm(profil):
    p = PROFILS[profil]
    b = Bench(quiet=True)
    b.open()
    etat = b.get_state(timeout=12)
    print(f'etat au depart: {etat}')
    b.enter_config()
    time.sleep(1)
    b.write_params(p['params'])
    # Relire ce qu on vient d ecrire: un parametre refuse est annonce par le
    # port, mais un parametre ecrit puis ecrase par un service ne l est pas.
    _, relu = b.read_params(list(p['params'].keys()), timeout=15.0)
    b.exit_config()
    time.sleep(2)
    etat_fin = b.get_state(timeout=12)
    b.close()

    # read_params rend un dict indexe par CLE ('ARP01'), pas par nom: comparer
    # sur les noms ne trouve jamais rien et l ecart passe inapercu. On traduit.
    ecarts = []
    for k, v in p['params'].items():
        cle = b._key(k)
        if cle in relu and str(relu[cle]).strip() != str(v).strip():
            ecarts.append(f'{k} ({cle}): pose {v}, relu {relu[cle]}')
        elif cle not in relu:
            ecarts.append(f'{k} ({cle}): absent de la relecture')

    t0 = time.time()
    with open(ETAT, 'w') as f:
        json.dump({'profil': profil, 't0': t0, 'periode_s': p['periode_s'],
                   'etat_depart': etat_fin}, f)
    print(f"profil '{profil}': {p['quoi']}")
    print(f'etat apres configuration: {etat_fin}')
    if ecarts:
        print('ECARTS de relecture:')
        for e in ecarts:
            print(f'   {e}')
    else:
        print('relecture conforme sur les', len(p['params']), 'parametres')
    print(f'\nt0 = {t0:.0f} ({time.strftime("%H:%M:%S", time.localtime(t0))})')
    print(f'laissez courir, puis: python3 tests/bench/endurance.py collect')
    return 0 if not ecarts else 1


# Un redemarrage volontaire (BootState) n est pas un incident; une remise a
# zero par chien de garde en est un.
RE_TX = re.compile(r'TX SUCCESS')
RE_ERR = re.compile(r'\+ERROR=|TX FAIL|TX ABORT')
RE_BOOT = re.compile(r'entry: BootState')
RE_WDT = re.compile(r'soft reset|Health WDT|WDT:')
RE_BACKOFF = re.compile(r'backoff|suspension|device-error hold')
RE_LIMITE = re.compile(r'rate limit')


def collect(plafond, garder):
    if not os.path.exists(ETAT):
        print(f'aucun essai arme ({ETAT} absent) — lancez "arm" d abord')
        return 2
    with open(ETAT) as f:
        etat = json.load(f)
    ecoule = time.time() - etat['t0']
    heures = ecoule / 3600.0

    b = Bench(quiet=True)
    b.open()
    print(f'etat a l arrivee: {b.get_state(timeout=12)}')
    b.enter_config()
    time.sleep(1)
    texte, n, mmm, tronque = harvest(b, LOGS['system'], plafond)
    b.exit_config()
    b.close()

    with open(garder, 'w') as f:
        f.write(texte)

    lignes = texte.splitlines()
    tx = sum(1 for l in lignes if RE_TX.search(l))
    err = sum(1 for l in lignes if RE_ERR.search(l))
    boots = sum(1 for l in lignes if RE_BOOT.search(l))
    wdt = sum(1 for l in lignes if RE_WDT.search(l))
    backoff = sum(1 for l in lignes if RE_BACKOFF.search(l))
    limite = sum(1 for l in lignes if RE_LIMITE.search(l))

    attendu = int(ecoule / etat['periode_s']) if etat['periode_s'] else 0
    print(f"\nprofil '{etat['profil']}' — {heures:.2f} h ({ecoule:.0f} s)")
    print(f'  journal        {len(texte)} octets, {n} paquets'
          f'{" (TRONQUE — remontez --plafond)" if tronque else ""}')
    print(f'  emissions      {tx}   attendu ~{attendu} '
          f'(une toutes les {etat["periode_s"]} s)')
    if attendu:
        ecart = 100.0 * (tx - attendu) / attendu
        print(f'  ecart          {ecart:+.0f} %')
        cadence = ecoule / tx if tx else 0
        print(f'  cadence reelle {cadence:.0f} s entre emissions')
    print(f'  erreurs TX     {err}')
    print(f'  redemarrages   {boots}')
    print(f'  chien de garde {wdt}')
    print(f'  backoff        {backoff}')
    print(f'  limiteur       {limite}')
    print(f'\njournal complet: {garder}')

    # Verdict: seul le silence est disqualifiant. Un ecart de cadence est un
    # ecart de budget batterie, pas une panne.
    if tronque:
        print('\nINCOMPLET: le journal a ete tronque, les comptes sont des minorants.')
        return 3
    if tx == 0:
        print('\nECHEC: aucune emission sur toute la duree.')
        return 1
    if attendu and abs(tx - attendu) > 0.25 * attendu:
        print('\nA REGARDER: plus de 25 % d ecart entre cadence promise et cadence tenue.')
        return 1
    print('\nOK: la balise a emis sans discontinuer a la cadence attendue.')
    return 0


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    a = sub.add_parser('arm')
    a.add_argument('--profil', default='doppler', choices=sorted(PROFILS))
    c = sub.add_parser('collect')
    c.add_argument('--plafond', type=int, default=8000)
    c.add_argument('--garder', default='/tmp/endurance.log')
    args = ap.parse_args()
    if args.cmd == 'arm':
        return arm(args.profil)
    return collect(args.plafond, args.garder)


if __name__ == '__main__':
    sys.exit(main())
