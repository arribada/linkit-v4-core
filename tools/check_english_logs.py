#!/usr/bin/env python3
"""Le firmware parle anglais. Cette verification l y oblige.

POURQUOI ELLE EXISTE. Le 2026-08-31, l utilisateur a recu un journal de sa
balise ou six messages sur dix etaient en francais: "RTC revenue en arriere",
"injection de %u octets d'assistance", "position trop vieille". Ces journaux
partent chez des partenaires et des integrateurs; la regle du projet est que
tout le firmware — code, commentaires et messages — est en anglais.

Le harnais de banc (tests/bench/) est deliberement en francais et n est donc
pas verifie ici. Seul le firmware l est: core/ et ports/nrf52840/, hors SDK
Nordic et hors repertoires de construction.
"""
import os
import re
import sys

RACINE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Mots francais sans ambiguite dans un contexte anglais. Volontairement courte
# et sure: mieux vaut rater une chaine que crier au loup sur "information" ou
# "position", qui s ecrivent pareil dans les deux langues.
# Compares en MOTS ENTIERS: "persisted" contient "persiste" et "impossible"
# s ecrit pareil dans les deux langues — les inclure ferait crier au loup sur
# du parfait anglais, et une verification qui crie au loup finit ignoree.
MARQUEURS = [
    'ecartee', 'ecartees', 'arriere', 'persistee', 'octets', 'vieille',
    'restauree', 'rayon', 'plafond', 'perdue', 'retenue', 'reveil',
    'provenance', 'aucune', 'aucun', 'ecriture', 'fichier', 'heure',
    'echec', 'erreur', 'demarrage', 'arret', 'lecture', 'derive',
    'synchronisee', 'avance', 'chaine', 'rompue',
    # 'injection' est identique en anglais — l inclure ferait un faux positif.
]

EXCLUS = ('/build/', '/drivers/', '/nRF5_SDK', '/external/', '/libraries/')
CHAINE = re.compile(r'"((?:[^"\\]|\\.){6,})"')


def fichiers():
    for base in ('core', 'ports/nrf52840/core', 'ports/nrf52840/main.cpp',
                 'ports/nrf52840/bsp'):
        chemin = os.path.join(RACINE, base)
        if os.path.isfile(chemin):
            yield chemin
            continue
        for dossier, _, noms in os.walk(chemin):
            if any(x in dossier.replace(os.sep, '/') for x in EXCLUS):
                continue
            for n in noms:
                if n.endswith(('.cpp', '.hpp', '.c', '.h')):
                    yield os.path.join(dossier, n)


def main():
    fautes = []
    for f in fichiers():
        try:
            lignes = open(f, encoding='utf-8', errors='replace').read().split('\n')
        except OSError:
            continue
        for i, ligne in enumerate(lignes, 1):
            for m in CHAINE.finditer(ligne):
                texte = m.group(1)
                bas = texte.lower()
                mots = set(re.findall(r"[a-z']+", bas))
                for mot in MARQUEURS:
                    if mot in mots:
                        fautes.append((os.path.relpath(f, RACINE), i, mot, texte[:88]))
                        break
    if fautes:
        print(f"{len(fautes)} chaine(s) de firmware en francais — le firmware doit etre en anglais:")
        for f, i, mot, texte in fautes:
            print(f"  {f}:{i}  [{mot}]  \"{texte}\"")
        return 1
    print("chaines de firmware: anglais uniquement")
    return 0


if __name__ == '__main__':
    sys.exit(main())
