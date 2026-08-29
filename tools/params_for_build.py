#!/usr/bin/env python3
"""Which DTE parameters a given build actually exposes, and why the others are hidden.

PARML and PARMR filter on the parameter table's `is_implemented` field, so a
board only ever advertises what it can honour. That is the right behaviour, but
it is invisible from the source: the table reads `ENABLE_AXL_SENSOR` and you
have to go and find what the build set it to.

This resolves the expression against a build's own compile flags and prints the
answer, split by the flag that decides it.

    python3 tools/params_for_build.py ports/nrf52840/build/LINKIT
    python3 tools/params_for_build.py ports/nrf52840/build/RSPB --hidden

An operator gets the same list from the device with a full `PARMR` (or `PARML`);
this is for reading it before the board exists, and for telling a missing key
apart from a broken one.
"""
import argparse
import glob
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TABLE = os.path.join(ROOT, "core/protocol/dte_params.cpp")

ENTRY = re.compile(
    r'\{\s*"([A-Z0-9_]+)"\s*,\s*"([A-Z]{2,3}[0-9]{2})"\s*,\s*BaseEncoding::(\w+)\s*,'
    r'\s*(.+?)\s*,\s*(.+?)\s*,\s*\{(.*?)\}\s*,\s*(.+?)\s*,\s*(\w+)\s*\}',
    re.S)


def build_flags(build_dir):
    """-D flags PER TARGET, as {target: {name: value}}.

    Per target, not merged. A build directory can hold several -- tests/build
    carries TrackerTests, RSPBSimulation and TurtleSimulation -- and their
    flags disagree by design. Unioning them yields a set no single binary was
    ever compiled with: EXTERNAL_WAKEUP leaked from the RSPB simulation and
    made this tool claim four parameters that TrackerTests does not expose.

    Make writes them to CMakeFiles/<target>.dir/flags.make, Ninja to
    build.ninja keyed by the object path.
    """
    cibles = {}
    for f in glob.glob(os.path.join(build_dir, "CMakeFiles/*.dir/flags.make")):
        nom = os.path.basename(os.path.dirname(f))[: -len(".dir")]
        d = cibles.setdefault(nom, {})
        for m in re.finditer(r'-D([A-Za-z_][A-Za-z0-9_]*)(?:=([^\s]+))?', open(f).read()):
            d[m.group(1)] = m.group(2) if m.group(2) is not None else "1"

    ninja = os.path.join(build_dir, "build.ninja")
    if os.path.exists(ninja):
        texte = open(ninja, errors="ignore").read()
        # "build CMakeFiles/<target>.dir/...o: ..." then an indented "FLAGS = ..."
        # Ninja splits them: macros land in DEFINES, the rest in FLAGS. Reading
        # only FLAGS silently drops every -DENABLE_*, which is precisely the
        # set that decides what a build exposes.
        for m in re.finditer(r'^build [^:\n]*?CMakeFiles/([^/]+)\.dir/[^:\n]*?:.*?\n'
                             r'((?:\s+\w+ = [^\n]*\n)+)',
                             texte, re.M):
            d = cibles.setdefault(m.group(1), {})
            for f2 in re.finditer(r'-D([A-Za-z_][A-Za-z0-9_]*)(?:=([^\s]+))?', m.group(2)):
                d[f2.group(1)] = f2.group(2) if f2.group(2) is not None else "1"
    return cibles


# Le tableau n utilise pas les drapeaux du build tels quels: dte_params.cpp en
# derive deux (presence d une macro sans valeur), et deux autres ont un defaut
# quand le build ne les passe pas. Sans ces regles, 23 parametres restaient
# "non resolus" alors que le compilateur, lui, sait tres bien les trancher.
DERIVES = {
    "HAS_EXTERNAL_WAKEUP": lambda f: "EXTERNAL_WAKEUP" in f,      # #ifdef, sans valeur
    "HAS_BOARD_RSPB": lambda f: "BOARD_RSPB" in f,                # #ifdef, sans valeur
}
DEFAUTS = {
    "LORA_RAK3172": "0",   # base_types.hpp
    "ARGOS_SMD": "0",      # dte_params.cpp
}


def resolve(expr, flags):
    """Evaluate an is_implemented expression. Returns True/False, or None if unknown."""
    expr = expr.strip()
    if expr == "true":
        return True
    if expr == "false":
        return False

    def valeur(nom):
        if nom in DERIVES:
            return "1" if DERIVES[nom](flags) else "0"
        return flags.get(nom, DEFAUTS.get(nom))

    # (FLAG == 1) or a bare FLAG
    m = re.fullmatch(r'\(\s*([A-Za-z_]\w*)\s*==\s*(\d+)\s*\)', expr)
    if m:
        v = valeur(m.group(1))
        return None if v is None else (v == m.group(2))
    if re.fullmatch(r'[A-Za-z_]\w*', expr):
        v = valeur(expr)
        return None if v is None else (v not in ("0", "false"))
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("build_dir", help="e.g. ports/nrf52840/build/LINKIT")
    ap.add_argument("--hidden", action="store_true", help="list the hidden parameters too")
    ap.add_argument("--target", help="which target, when the build directory holds several")
    ap.add_argument("--self-check", action="store_true",
                    help="compare the prediction against the host suite's real PARMR response")
    a = ap.parse_args()

    cibles = build_flags(a.build_dir)
    if not cibles:
        sys.exit(f"no compile flags found under {a.build_dir} — is it configured and built?")
    if a.target:
        if a.target not in cibles:
            sys.exit(f"target {a.target!r} not in {a.build_dir}: {', '.join(sorted(cibles))}")
        nom_cible, flags = a.target, cibles[a.target]
    elif len(cibles) == 1:
        nom_cible, flags = next(iter(cibles.items()))
    else:
        sys.exit(f"{len(cibles)} targets in {a.build_dir} and their flags differ; "
                 f"pick one with --target: {', '.join(sorted(cibles))}")

    src = open(TABLE).read()
    exposes, caches, inconnus = [], {}, []
    for m in ENTRY.finditer(src):
        nom, key, _enc, _lo, _hi, _perm, impl, _wr = m.groups()
        etat = resolve(impl, flags)
        if etat is True:
            exposes.append((key, nom))
        elif etat is False:
            caches.setdefault(impl.strip(), []).append((key, nom))
        else:
            inconnus.append((key, nom, impl.strip()))

    carte = [k for k in ("BOARD_LINKIT", "BOARD_RSPB", "BOARD_HORIZON") if k in flags]
    total = len(exposes) + sum(len(v) for v in caches.values()) + len(inconnus)
    # Deuxieme filtre, independant de is_implemented: une requete PARMR vide ne
    # rend que les cles de CONFIGURATION (3e caractere 'P'), et STATR les cles
    # d ETAT ('T'). Les compter ensemble donnerait un total qu aucune des deux
    # commandes ne renvoie jamais.
    p_keys = [k for k, _ in exposes if k[2] == "P"]
    t_keys = [k for k, _ in exposes if k[2] == "T"]
    autres = [k for k, _ in exposes if k[2] not in ("P", "T")]
    print(f"{nom_cible}"
          f"{' (' + ', '.join(carte) + ')' if carte else ''}")
    print(f"  {len(exposes)} parametres implementes sur {total}, dont:")
    print(f"    {len(p_keys):3d} rendus par un PARMR vide   (cles de configuration, ...P..)")
    print(f"    {len(t_keys):3d} rendus par STATR           (cles d etat, ...T..)")
    if autres:
        print(f"    {len(autres):3d} ni l un ni l autre: {', '.join(autres)}")
    print()

    if caches:
        print(f"  {sum(len(v) for v in caches.values())} caches, par drapeau:")
        for expr, l in sorted(caches.items(), key=lambda x: -len(x[1])):
            print(f"    {expr:28s} {len(l):3d}  {', '.join(k for k, _ in l)}")
    if inconnus:
        print(f"\n  {len(inconnus)} non resolus (drapeau absent des flags de ce build):")
        for key, nom, expr in inconnus:
            print(f"    {key} {nom} <- {expr}")
    if a.self_check:
        # Le test hote PARMR_REQ_CheckEmptyRequest fige la reponse REELLE d un
        # PARMR vide. La comparer a la prediction est la seule verification que
        # cet outil decrit bien le firmware et pas une idee du firmware.
        chemin = os.path.join(ROOT, "tests/src/dte_handler_test.cpp")
        texte = open(chemin).read()
        i = texte.index("PARMR_REQ_CheckEmptyRequest")
        m = re.search(r'"\$O;PARMR#[0-9A-F]{3};(.*?)\\r"', texte[i:], re.S)
        if not m:
            print("\n  auto-verification: reponse PARMR introuvable dans le test hote")
            return 1
        corps = re.sub(r'"\s*"', '', m.group(1))
        reels = sorted({x.split("=")[0] for x in corps.split(",") if "=" in x})
        reels_p = [k for k in reels if k[2:3] == "P"]
        manque = sorted(set(p_keys) - set(reels_p))
        surplus = sorted(set(reels_p) - set(p_keys))
        print(f"\n  auto-verification contre le PARMR reel du test hote:")
        print(f"    predits {len(p_keys)}, reels {len(reels_p)}")
        if manque:
            print(f"    predits mais absents : {', '.join(manque)}")
        if surplus:
            print(f"    reels mais non predits: {', '.join(surplus)}")
        if manque or surplus:
            print("    -> l outil ne decrit pas ce build; ne pas s y fier tel quel")
            return 1
        print("    concordance exacte")

    if a.hidden and caches:
        print("\n  Detail des caches:")
        for expr, l in sorted(caches.items()):
            print(f"    --- {expr}")
            for key, nom in l:
                print(f"      {key}  {nom}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
