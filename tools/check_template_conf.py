#!/usr/bin/env python3
"""Validate a template_conf/*.cfg against the firmware parameter table.

Checks, for every key in the file:
  - the parameter exists in core/protocol/dte_params.cpp
  - it is writable (a read-only parameter in a template is a silent no-op)
  - the value is in the permitted set, when the parameter is a code table
  - the value is within [min, max], for plain numeric parameters

Run with no argument to check every file in template_conf/.
"""
import re
import sys
import glob
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TABLE = os.path.join(ROOT, "core/protocol/dte_params.cpp")
PROTOCOL = os.path.join(ROOT, "core/protocol/dte_protocol.hpp")
TYPES = os.path.join(ROOT, "core/protocol/base_types.hpp")

# Coded encodings whose permitted set in the parameter table is checked
# AFTER decoding (every one of them calls DTEEncoder::validate) -- AQPERIOD
# is the exception and runs no such check.
VALIDATED_AFTER_DECODE = {
    "LEDMODE", "ZONETYPE", "GNSSFIXMODE", "GNSSDYNMODEL", "ARGOSPOWER",
    "ARGOSMODE", "DEPTHPILE", "MODULATION", "DEBUGMODE", "UWDETECTSOURCE",
    "PRESSURESENSORFULLSCALE", "PRESSURESENSORLOGGINGMODE",
}

# Coded encodings whose accepted DTE inputs are enumerated by a decode_*
# function rather than by the permitted set in the parameter table. LED_MODE
# is the live example: its set is empty, yet decode_led_mode() accepts only
# 0, 1 and 3 -- a template with LED_MODE=2 is rejected at push time.
DECODERS = {
    "LEDMODE": "decode_led_mode",
    "ZONETYPE": "decode_zone_type",
    "GNSSFIXMODE": "decode_gnss_fix_mode",
    "GNSSDYNMODEL": "decode_gnss_dyn_model",
    "ARGOSPOWER": "decode_power",
    "ARGOSMODE": "decode_mode",
    "DEPTHPILE": "decode_depth_pile",
    "AQPERIOD": "decode_acquisition_period",
    "MODULATION": "decode_argos_modulation",
    "DEBUGMODE": "decode_debug_mode",
    "SENSORENABLETXMODE": "decode_sensor_enable_tx_mode",
    "UWDETECTSOURCE": "decode_underwater_detect_source",
    "PRESSURESENSORFULLSCALE": "decode_pressure_sensor_full_scale",
    "PRESSURESENSORLOGGINGMODE": "decode_pressure_sensor_logging_mode",
}


def load_enums():
    """Numeric value of every enum constant in base_types.hpp."""
    try:
        src = open(TYPES).read()
    except OSError:
        return {}
    out = {}
    for body in re.findall(r'enum class \w+[^{]*\{(.*?)\}', src, re.S):
        nxt = 0
        for item in body.split(","):
            item = re.sub(r'//.*', '', item).strip()
            if not item:
                continue
            m = re.match(r'([A-Za-z_]\w*)\s*(?:=\s*(.+))?$', item)
            if not m:
                continue
            name, val = m.groups()
            if val is not None:
                v = _num(val)
                if v is None:
                    continue
                nxt = int(v)
            out[name] = nxt
            nxt += 1
    return out


def load_decoder_maps():
    """Recover, per decode_* function, the input->decoded-value mapping.

    The distinction matters: for DEPTHPILE the DTE input is a CODE
    (1,2,3,4,8,9,10,11,12) while the parameter table's permitted set holds
    the resulting VALUES (1,2,3,4,8,12,16,20,24). Writing ARGOS_DEPTH_PILE=16
    in a config file is rejected -- the code for a 16-deep pile is 10.
    """
    enums = load_enums()
    try:
        src = open(PROTOCOL).read()
    except OSError:
        return {}
    out = {}
    for enc, fn in DECODERS.items():
        m = re.search(r'\bdecode_\w+\b'.replace(r'\w+', fn[7:]) + r'\s*\(const std::string\s*&\s*\w+\s*\)\s*\{', src)
        if not m:
            continue
        # Walk to the matching closing brace of the function body.
        i, depth = m.end() - 1, 0
        while i < len(src):
            if src[i] == '{':
                depth += 1
            elif src[i] == '}':
                depth -= 1
                if depth == 0:
                    break
            i += 1
        body = src[m.end():i]
        mapping = {}
        for inp, ret in re.findall(r'==\s*"([^"]+)"\s*\)\s*\{?\s*return\s+([^;]+);', body):
            key = _num(inp)
            if key is None:
                continue
            ret = ret.strip()
            enum_name = ret.rsplit("::", 1)[-1].strip()
            val = enums.get(enum_name)
            if val is None:
                val = _num(ret)
            if val is None:
                m2 = re.fullmatch(r'(\d+)\s*\*\s*(\d+)', ret)
                val = int(m2.group(1)) * int(m2.group(2)) if m2 else None
            mapping[key] = val
        if mapping:
            out[enc] = mapping
    return out

ENTRY = re.compile(
    r'\{\s*"([A-Z0-9_]+)"\s*,\s*"([A-Z]{2,3}[0-9]{2})"\s*,\s*BaseEncoding::(\w+)\s*,'
    r'\s*(.+?)\s*,\s*(.+?)\s*,\s*\{(.*?)\}\s*,\s*(.+?)\s*,\s*(\w+)\s*\}',
    re.S)


def _num(tok):
    """Parse a C++ numeric literal from the table; None when not a plain number."""
    tok = tok.strip().rstrip("U").rstrip("u")
    tok = re.sub(r'^\(\s*(?:double|float|unsigned int|int|uint\d+_t)\s*\)\s*', '', tok).strip()
    tok = tok.rstrip("U").rstrip("u")
    try:
        return int(tok, 0)
    except ValueError:
        pass
    try:
        return float(tok)
    except ValueError:
        return None


def load_params():
    src = open(TABLE).read()
    decoder_maps = load_decoder_maps()
    out = {}
    for m in ENTRY.finditer(src):
        name, key, enc, lo, hi, perm, _impl, writable = m.groups()
        codes = [_num(t) for t in perm.split(",") if t.strip()]
        out[name] = {
            "decode": decoder_maps.get(enc),
            "gate": None if _impl.strip() == "true" else _impl.strip(),
            "key": key,
            "enc": enc,
            "min": _num(lo),
            "max": _num(hi),
            "set": [c for c in codes if c is not None],
            "writable": writable.strip() == "true",
        }
    return out


def check(path, params):
    problems = []
    gated = []
    seen = set()
    n = 0
    for lineno, line in enumerate(open(path), 1):
        s = line.strip()
        if not s or s.startswith(("#", ";", "[")) or "=" not in s:
            continue
        k, _, v = s.partition("=")
        k, v = k.strip(), v.strip()
        n += 1
        if k in seen:
            problems.append(f"{lineno}: {k}: duplicate key")
        seen.add(k)
        p = params.get(k)
        if p is None:
            problems.append(f"{lineno}: {k}: unknown parameter")
            continue
        if p["gate"]:
            gated.append((k, p["key"], p["gate"]))
        if not p["writable"]:
            problems.append(f"{lineno}: {k} ({p['key']}): read-only, cannot be set")
            continue
        if p["enc"] in ("TEXT", "DATESTRING", "HEXADECIMAL", "BASE64", "KEY_LIST", "AQPERIOD_LIST"):
            continue
        val = _num(v)
        if val is None:
            problems.append(f"{lineno}: {k}: '{v}' is not numeric for a {p['enc']} parameter")
            continue
        if p["enc"] == "BOOLEAN":
            if val not in (0, 1):
                problems.append(f"{lineno}: {k} ({p['key']}): boolean, got {v}")
            continue
        dec = p["decode"]
        if dec is not None:
            # The value written in the file is the DTE input; check it first.
            if val not in dec:
                problems.append(
                    f"{lineno}: {k} ({p['key']}): {v} is not an accepted {p['enc']} input "
                    f"{sorted(dec)}")
                continue
            # Then the permitted set, which the firmware applies to the
            # DECODED value -- and only for encodings that call validate().
            decoded = dec[val]
            if p["set"] and p["enc"] in VALIDATED_AFTER_DECODE and decoded is not None \
                    and decoded not in p["set"]:
                problems.append(
                    f"{lineno}: {k} ({p['key']}): input {v} decodes to {decoded}, "
                    f"outside the permitted set {sorted(p['set'])}")
            continue
        if p["set"]:
            if val not in p["set"]:
                problems.append(
                    f"{lineno}: {k} ({p['key']}): {v} not in permitted set {sorted(p['set'])}")
            continue
        lo, hi = p["min"], p["max"]
        if lo is not None and hi is not None and hi > lo and not (lo <= val <= hi):
            problems.append(f"{lineno}: {k} ({p['key']}): {v} outside [{lo}, {hi}]")
    return n, problems, gated


def main():
    params = load_params()
    files = sys.argv[1:] or sorted(glob.glob(os.path.join(ROOT, "template_conf/*.cfg")))
    if not files:
        print("no template to check")
        return 0
    rc = 0
    for f in files:
        n, problems, gated = check(f, params)
        rel = os.path.relpath(f, ROOT)
        if problems:
            rc = 1
            print(f"FAIL {rel}  ({n} keys, {len(problems)} problems)")
            for p in problems:
                print(f"       {p}")
        else:
            print(f"ok   {rel}  ({n} keys)")
        if gated:
            # Not an error: these exist, but only on a build whose flag is set.
            # Pushed elsewhere they are reported as not implemented and ignored.
            print(f"       note: {len(gated)} key(s) behind a build flag —")
            for k, key, gate in gated:
                print(f"         {k} ({key}) requires {gate}")
    print(f"\n{len(params)} parameters in the firmware table")
    return rc


if __name__ == "__main__":
    sys.exit(main())
