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


def load_live_encoder():
    """PyLinkit's own encoder, when PYLINKIT points at a checkout.

    Always preferred over the mirror: several codecs TRANSFORM the value
    rather than look it up, and only the real one can be trusted with those.
    ARGOSDUTYCYLE is the case that matters -- it takes a hex string, so
    "16777215" silently means 0x16777215 and the device refuses it.
    """
    live = os.environ.get("PYLINKIT")
    if not live:
        return None
    sys.path.insert(0, live)
    try:
        from pylinkit.protocol.dte_params import DTEParamMap
    except Exception as e:
        print(f"note: PYLINKIT={live} could not be imported ({e}); using the mirror")
        return None
    return DTEParamMap

class _Unverifiable(str):
    """Marker: this line needs the real encoder, it is not an error."""


UNVERIFIABLE = _Unverifiable("needs PyLinkit's own encoder")

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


def to_wire(key, value, live=None, nom=None):
    """Turn a template's human value into the wire value, or explain why not.

    Returns (wire_value, error). Exactly one of the two is None.
    """
    typ = PL.TYPES.get(key)
    if live is not None:
        try:
            return _num(live.encode(nom, value)), None
        except Exception as e:
            # A bare KeyError prints only the offending value, which tells the
            # reader nothing about what WAS acceptable. Fill that in from the
            # mirror's table for the same type.
            detail = str(e) or type(e).__name__
            coded = PL.CODED.get(typ)
            if coded:
                offert = sorted(v for v in coded[1] if v != -1)
                unite = "minutes" if coded[0] == "minutes" else "one of"
                detail = f"{detail} — takes {unite} {offert}"
            elif typ in PL.TRANSFORMING:
                detail = f"{detail} — takes {PL.TRANSFORMING[typ]}"
            return None, f"PyLinkit refuses {value!r}: {detail}"
    if typ in PL.TRANSFORMING:
        # No table to check against offline, and guessing would be worse than
        # saying so: these codecs rewrite the value. Not a failure of the FILE
        # -- a limit of this run, reported as such.
        return None, UNVERIFIABLE
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


def coherence(valeurs):
    """Combinations the firmware accepts and that produce a silent beacon.

    Every one of these is a valid config: no parameter is out of range, no key
    is unknown, and the device will happily store it. They are here because a
    file that passes every other check can still describe a tracker that never
    transmits, and that failure is indistinguishable at sea from a lost tag.
    """
    v = valeurs.get
    out = []
    mode = v("ARGOS_MODE")
    if mode == "PASS_PREDICTION" and v("GNSS_ENABLE") == "0":
        out.append("ARGOS_MODE=PASS_PREDICTION with GNSS_ENABLE=0 schedules NOTHING: "
                   "pass prediction is computed from a position. The service logs an "
                   "error and no transmission is ever planned.")
    if mode == "SURFACING_BURST" and v("UW_ENABLE") == "0":
        out.append("ARGOS_MODE=SURFACING_BURST with UW_ENABLE=0: the burst is triggered "
                   "by a surfacing event, and without the water sensor there is none.")
    if v("GNSS_FASTLOC_MODE") == "2" and mode != "SURFACING_BURST":
        out.append(f"GNSS_FASTLOC_MODE=2 (CloudLocate) needs ARGOS_MODE=SURFACING_BURST; "
                   f"with {mode} the raw-measurement capture is never armed.")
    if v("ARGOS_RX_EN") == "1" and mode != "PASS_PREDICTION":
        out.append(f"ARGOS_RX_EN=1 only bites in PASS_PREDICTION; with {mode} the receive "
                   "window never opens and the parameter does nothing.")
    dc = (v("ARGOS_DUTY_CYCLE") or "").strip().upper()
    if dc in ("0", "000000", "00000000"):
        out.append("ARGOS_DUTY_CYCLE is an empty hour mask: the beacon is MUTE, "
                   "twenty-four hours a day.")
    if v("GNSS_ENABLE") == "1" and v("GNSS_HACCFILT_ENABLE") == "1":
        seuil = _num(v("GNSS_HACCFILT_THR"))
        if seuil is not None and seuil <= 5:
            out.append(f"GNSS_HACCFILT_THR={seuil} m is strict enough to reject most fixes "
                       "from a wet antenna, and a rejected fix is not logged as a fault.")
    if v("LB_EN") == "1":
        lb, crit = _num(v("LB_THRESHOLD")), _num(v("LB_CRITICAL_THRESH"))
        if lb is not None and crit is not None and crit >= lb:
            out.append(f"LB_CRITICAL_THRESH ({crit}) is not below LB_THRESHOLD ({lb}): "
                       "the critical shutdown moves up to the low-battery threshold.")
    return out


def check(path, fw, live=None):
    problems, gated, seen, n = [], [], set(), 0
    non_verifies, valeurs = [], {}
    par_nom = {n_: k for k, n_ in PL.NAMES.items()}
    fw_par_nom = {v["fw_name"]: k for k, v in fw.items()}

    for lineno, line in enumerate(open(path), 1):
        s = line.strip()
        if not s or s.startswith(("#", ";", "[")) or "=" not in s:
            continue
        nom, _, val = s.partition("=")
        nom, val = nom.strip(), val.strip()
        n += 1
        valeurs[nom] = val
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

        wire, err = to_wire(key, val, live, nom)
        if isinstance(err, _Unverifiable):
            non_verifies.append((nom, key, PL.TRANSFORMING[PL.TYPES[key]]))
            continue
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
    problems.extend(f"coherence: {c}" for c in coherence(valeurs))
    return n, problems, gated, non_verifies


def main():
    fw = load_firmware()
    live = load_live_encoder()
    rc = 0
    drift = check_mirror(fw)
    if drift:
        rc = 1
        print(f"FAIL tools/pylinkit_map.py ({len(drift)} problems)")
        for d in drift:
            print(f"       {d}")

    non_verifies_total = {}
    files = sys.argv[1:] or sorted(glob.glob(os.path.join(ROOT, "template_conf/*.cfg")))
    if not files:
        print("no template to check")
        return rc
    for f in files:
        n, problems, gated, non_verifies = check(f, fw, live)
        rel = os.path.relpath(f, ROOT)
        if problems:
            rc = 1
            print(f"FAIL {rel}  ({n} keys, {len(problems)} problems)")
            for p in problems:
                print(f"       {p}")
        else:
            print(f"ok   {rel}  ({n} keys)")
        for nom, key, quoi in non_verifies:
            non_verifies_total.setdefault((nom, key, quoi), 0)
            non_verifies_total[(nom, key, quoi)] += 1
        if gated:
            print(f"       note: {len(gated)} key(s) behind a build flag —")
            for nom, key, gate in gated:
                print(f"         {nom} ({key}) requires {gate}")
    if non_verifies_total:
        total = sum(non_verifies_total.values())
        print(f"\n{total} line(s) NOT verified across {len(files)} file(s) — set PYLINKIT "
              f"to a checkout so PyLinkit's own encoder can check them:")
        for (nom, key, quoi), n in sorted(non_verifies_total.items()):
            print(f"  {nom} ({key}) takes {quoi}  [x{n}]")
    source = "PyLinkit's own encoder" if live else "the mirror in tools/pylinkit_map.py"
    print(f"\n{len(fw)} parameters in the firmware table, {len(PL.NAMES)} known to "
          f"PyLinkit; values checked with {source}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
