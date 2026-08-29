#!/usr/bin/env python3
"""Validate a template_conf/*.cfg before it ever reaches a device.

Config files here are pushed with `pylinkit_cli config push`, so they are
written in PYLINKIT's space, not the firmware's:

  - PyLinkit's parameter NAMES. Thirty-one of them differ from the firmware's
    name for the same key: ARGOS_DUTY_CYCLE, not DUTY_CYCLE; GNSS_ENABLE, not
    GNSS_EN; GNSS_DELTATIME_ACQ, not DLOC_ARG_NOM.
  - HUMAN values, which PyLinkit encodes to the wire codes itself:
    `ARGOS_MODE = SURFACING_BURST` rather than 5, a depth rather than a depth-
    pile code, and MINUTES rather than an acquisition-period code.

PyLinkit checks neither the names' spelling against the firmware nor any
numeric range: an unknown name is dropped and an out-of-range value is only
refused by the device. This checker closes both gaps by reading
core/protocol/dte_params.cpp for the keys and their limits, and
tools/pylinkit_map.py for the names and the human-value tables.

    python3 tools/check_template_conf.py                       # every template
    python3 tools/check_template_conf.py template_conf/x.cfg   # just one

Set PYLINKIT to a PyLinkit v4 checkout to also verify that the mirror in
tools/pylinkit_map.py still matches that installation.
"""
import glob
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TABLE = os.path.join(ROOT, "core/protocol/dte_params.cpp")
sys.path.insert(0, os.path.join(ROOT, "tools"))
import pylinkit_map as PL  # noqa: E402

ENTRY = re.compile(
    r'\{\s*"([A-Z0-9_]+)"\s*,\s*"([A-Z]{2,3}[0-9]{2})"\s*,\s*BaseEncoding::(\w+)\s*,'
    r'\s*(.+?)\s*,\s*(.+?)\s*,\s*\{(.*?)\}\s*,\s*(.+?)\s*,\s*(\w+)\s*\}',
    re.S)


def _num(tok):
    """Parse a C++ numeric literal; None when it is not a plain number."""
    tok = re.sub(r'^\(\s*(?:double|float|unsigned int|int|uint\d+_t)\s*\)\s*', '',
                 str(tok).strip()).strip()
    tok = tok.rstrip("Uu")
    for conv in (lambda t: int(t, 0), float):
        try:
            return conv(tok)
        except ValueError:
            pass
    return None


def load_firmware():
    """key -> firmware limits for that parameter."""
    out = {}
    for m in ENTRY.finditer(open(TABLE).read()):
        name, key, enc, lo, hi, perm, impl, writable = m.groups()
        out[key] = {
            "fw_name": name,
            "enc": enc,
            "min": _num(lo),
            "max": _num(hi),
            "set": [v for v in (_num(t) for t in perm.split(",") if t.strip()) if v is not None],
            "writable": writable.strip() == "true",
            "gate": None if impl.strip() == "true" else impl.strip(),
        }
    return out


def check_mirror(fw):
    """Report drift between the mirror, the firmware, and a live PyLinkit."""
    problems = []
    for key in PL.NAMES:
        if key not in fw:
            problems.append(f"tools/pylinkit_map.py knows key {key}, the firmware does not")
    for key in fw:
        if key not in PL.NAMES:
            problems.append(f"the firmware has key {key} ({fw[key]['fw_name']}), "
                            f"PyLinkit does not — it cannot be pushed")
    live = os.environ.get("PYLINKIT")
    if live:
        try:
            src = open(os.path.join(live, "pylinkit/protocol/dte_params.py")).read()
        except OSError as e:
            problems.append(f"PYLINKIT={live} unreadable: {e}")
            return problems
        actuel = {k: n for n, k in re.findall(
            r'\[\s*"([A-Z0-9_]+)"\s*,\s*"([A-Z]{2,3}[0-9]{2})"', src)}
        for key, name in sorted(actuel.items()):
            if PL.NAMES.get(key) != name:
                problems.append(f"mirror is stale for {key}: PyLinkit says {name}, "
                                f"mirror says {PL.NAMES.get(key)} — regenerate with "
                                f"tools/gen_pylinkit_map.py")
    return problems


def to_wire(key, value):
    """Turn a template's human value into the wire value, or explain why not.

    Returns (wire_value, error). Exactly one of the two is None.
    """
    typ = PL.TYPES.get(key)
    coded = PL.CODED.get(typ)
    if coded is None:
        if typ == "BOOLEAN":
            n = _num(value)
            if n not in (0, 1):
                return None, f"boolean, got {value!r}"
            return n, None
        if typ in ("TEXT", "DATESTRING", "HEXADECIMAL"):
            return None, None  # nothing numeric to check
        n = _num(value)
        if n is None:
            return None, f"{value!r} is not numeric for a {typ} parameter"
        return n, None

    kind, table = coded
    if kind == "minutes":
        n = _num(value)
        if n is None or int(n) not in table:
            offert = sorted(v for v in table)
            return None, (f"{value!r} is not an acquisition period. PyLinkit takes "
                          f"MINUTES: {offert}")
        return table.index(int(n)), None

    # 'index': the wire code is the position in PyLinkit's table.
    offert = [v for v in table if v != -1]
    brut = _num(value)
    for cand in (value, str(brut if brut is not None and brut == int(brut or 0) else value)):
        try:
            i = table.index(int(cand) if str(cand).lstrip("-").isdigit() else cand)
        except (ValueError, TypeError):
            continue
        if table[i] != -1:
            return i, None
    return None, f"{value!r} is not one of {offert}"


def check(path, fw):
    problems, gated, seen, n = [], [], set(), 0
    par_nom = {n_: k for k, n_ in PL.NAMES.items()}
    fw_par_nom = {v["fw_name"]: k for k, v in fw.items()}

    for lineno, line in enumerate(open(path), 1):
        s = line.strip()
        if not s or s.startswith(("#", ";", "[")) or "=" not in s:
            continue
        nom, _, val = s.partition("=")
        nom, val = nom.strip(), val.strip()
        n += 1
        if nom in seen:
            problems.append(f"{lineno}: {nom}: duplicate key")
        seen.add(nom)

        key = par_nom.get(nom)
        if key is None:
            autre = fw_par_nom.get(nom)
            if autre:
                problems.append(f"{lineno}: {nom}: that is the FIRMWARE name for {autre}; "
                                f"PyLinkit calls it {PL.NAMES[autre]}")
            else:
                problems.append(f"{lineno}: {nom}: unknown parameter")
            continue

        info = fw[key]
        if not info["writable"]:
            problems.append(f"{lineno}: {nom} ({key}): read-only, cannot be set")
            continue
        if info["gate"]:
            gated.append((nom, key, info["gate"]))

        wire, err = to_wire(key, val)
        if err:
            problems.append(f"{lineno}: {nom} ({key}): {err}")
            continue
        if wire is None:
            continue

        if info["set"] and wire not in info["set"]:
            lisible = PL.CODED.get(PL.TYPES.get(key))
            if lisible:
                autorise = [lisible[1][c] for c in sorted(info["set"]) if c < len(lisible[1])]
                problems.append(f"{lineno}: {nom} ({key}): {val!r} is refused here; "
                                f"this parameter only permits {autorise}")
            else:
                problems.append(f"{lineno}: {nom} ({key}): {val!r} encodes to {wire}, "
                                f"outside the permitted set {sorted(info['set'])}")
            continue
        lo, hi = info["min"], info["max"]
        if lo is not None and hi is not None and hi > lo and not (lo <= wire <= hi):
            problems.append(f"{lineno}: {nom} ({key}): {val} outside [{lo}, {hi}]")
    return n, problems, gated


def main():
    fw = load_firmware()
    rc = 0
    drift = check_mirror(fw)
    if drift:
        rc = 1
        print(f"FAIL tools/pylinkit_map.py ({len(drift)} problems)")
        for d in drift:
            print(f"       {d}")

    files = sys.argv[1:] or sorted(glob.glob(os.path.join(ROOT, "template_conf/*.cfg")))
    if not files:
        print("no template to check")
        return rc
    for f in files:
        n, problems, gated = check(f, fw)
        rel = os.path.relpath(f, ROOT)
        if problems:
            rc = 1
            print(f"FAIL {rel}  ({n} keys, {len(problems)} problems)")
            for p in problems:
                print(f"       {p}")
        else:
            print(f"ok   {rel}  ({n} keys)")
        if gated:
            print(f"       note: {len(gated)} key(s) behind a build flag —")
            for nom, key, gate in gated:
                print(f"         {nom} ({key}) requires {gate}")
    print(f"\n{len(fw)} parameters in the firmware table, {len(PL.NAMES)} known to PyLinkit")
    return rc


if __name__ == "__main__":
    sys.exit(main())
