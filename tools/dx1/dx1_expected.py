#!/usr/bin/env python3
"""Expected platform decodes for every DX1 v3 vector, derived from dx1_ref.py (the source of truth).

For each frames[] view and each scenarios[] hex checkpoint it prints the fields the platform decoder
must return. The field CODES come from the reference model itself (the arguments dx1_ref.image() was
called with, captured by wrapping it), not from re-parsing the bytes; the decoded values are derived
from those codes with dx1_ref.em_dec and the Q4 bucket table. It also asserts that a bit-level parse
of each hex gives back the same codes (layout cross-check).

Usage: python3 dx1_expected.py [--ts OUT.ts]   (default: print JSON)
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dx1_ref as R  # noqa: E402

# ---- capture every image() call made by the reference vector generator
CALLS = {}
_orig_image = R.image


def _image(mv, lb, dry, dend, rich=None):
    img = _orig_image(mv, lb, dry, dend, rich)
    CALLS[R.H(img)] = dict(mv=mv, lb=lb, dry=dry, dend=dend, rich=dict(rich) if rich else None)
    return img


R.image = _image
V = R.vectors()

# vectors file on disk must match a fresh generation
here = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(here, "dx1_vectors_v3.json")) as f:
    ON_DISK = json.load(f)
assert ON_DISK == json.loads(json.dumps(V)), "dx1_vectors_v3.json is stale: regenerate it with dx1_ref.py"

Q4_RANGES = [(0, 0), (1, 1), (2, 2), (3, 3), (4, 4), (5, 5), (6, 6), (7, 7), (8, 15), (16, 31), (32, 63),
             (64, 127), (128, 255), (256, 1023), (1024, 4095), (4096, None)]
for n in range(0, 70000, 7):  # the bucket table agrees with the reference encoder
    lo, hi = Q4_RANGES[R.q4(n)]
    assert lo <= n and (hi is None or n <= hi), n


def bits(b, off, w):
    v = 0
    for i in range(w):
        p = off + i
        v = (v << 1) | ((b[p // 8] >> (7 - p % 8)) & 1 if p // 8 < len(b) else 0)
    return v


def lookup(hexstr):
    """Reference call whose image starts with this view."""
    hits = [c for full, c in CALLS.items() if full.startswith(hexstr)]
    assert hits, hexstr
    # all images sharing a 3/4-byte prefix share mv/lb/dry/dend
    c0 = hits[0]
    for c in hits:
        assert (c["mv"], c["lb"], c["dry"], c["dend"]) == (c0["mv"], c0["lb"], c0["dry"], c0["dend"])
    return CALLS.get(hexstr) or c0


def rich_expected(codes):
    full = {k: codes.get(k, 0) for k, _ in R.RICH_LAYOUT}
    npo = R.em_dec(full["n_surf"], 4, 3)
    delay = R.em_dec(full["delay"], 3, 3)
    q = lambda c: {"min": Q4_RANGES[c][0], "max": Q4_RANGES[c][1]}  # noqa: E731
    return {
        "latch": full["latch"],
        "blind": full["blind"] == 1,
        "partial": full["partial"] == 1,
        "surfaceSeq": full["surf_seq"] or None,
        "resets": full["rst"],
        "pctDry": None if full["pct"] == 0 else full["pct"] - 1,
        "surfacings24h": None if npo is None else npo - 1,
        "surfaceLast_s": R.em_dec(full["s_last"], 4, 3),
        "divePrev_s": R.em_dec(full["d_prev"], 4, 3),
        "diveMax24h_s": R.em_dec(full["d_max"], 4, 3),
        "sampleDelay_us": None if delay is None else delay * 50,
        "contrast_x10": R.em_dec(full["contrast"], 3, 3),
        "diveTimeouts": q(full["dto"]),
        "forcedSurfaces": q(full["fsc"]),
        "stuckRecoveries": q(full["src"]),
        "surfaceMethod": full["method"] or None,
        "delayFloor": full["dfloor"] == 1,
        "adcFault": full["adcf"] == 1,
        "codes": full,
    }


def expected(hexstr, mode):
    c = lookup(hexstr)
    b = bytes.fromhex(hexstr)
    nbits = len(b) * 8
    # layout cross-check: bytes parse back to the reference codes
    assert b[0] == 0 and bits(b, 8, 7) == R.batt(c["mv"]) and bits(b, 15, 1) == (1 if c["lb"] else 0)
    assert bits(b, 16, 2) == 1 and bits(b, 18, 1) == 0
    assert bits(b, 19, 3) == c["dry"] and bits(b, 22, 2) == c["dend"]
    if nbits >= 32:
        assert b[3] == R.crc8(b[:3])
    e = {
        "messageType": "DOPPLER",
        "bitLength": nbits,
        "battery_mV": R.batt(c["mv"]) * 20 + 2700,
        "lowBattery": bool(c["lb"]),
        "dx1Version": 1,
        "dryClass": c["dry"] or None,
        "diveEnd": c["dend"] or None,
    }
    if nbits >= 32:
        e["crcValid"] = True
    rich = c["rich"]
    if mode == "LDK" and rich and rich.get("latch", 0) != 0:
        off = 32
        for name, w in R.RICH_LAYOUT:
            assert bits(b, off, w) == rich.get(name, 0), (hexstr, name)
            off += w
        assert b[14] == 0 and b[15] == 0
        e["rich"] = rich_expected(rich)
    return e


out = []
for fr in V["frames"]:
    for mode in ("VLDA4", "LDA2", "LDK"):
        out.append({"name": f"{fr['name']}/{mode}", "modulation": mode, "hex": fr[mode],
                    "expected": expected(fr[mode], mode)})
for sc in V["scenarios"]:
    for cp in sc["checkpoints"]:
        if not isinstance(cp["value"], str):
            continue  # DEND-only checkpoints (tracker outputs, not frames)
        mode = "LDA2" if len(cp["value"]) == 8 else ("VLDA4" if len(cp["value"]) == 6 else "LDK")
        out.append({"name": f"{sc['name']}/{cp['label']}", "modulation": mode, "hex": cp["value"],
                    "expected": expected(cp["value"], mode)})

if len(sys.argv) > 2 and sys.argv[1] == "--ts":
    with open(sys.argv[2], "w") as f:
        f.write("// ABOUTME: DX1 v3 LinkIt Doppler test vectors with the decode the reference model expects.\n")
        f.write("// ABOUTME: GENERATED from the firmware reference model (dx1_ref.py, dx1_expected.py) - do not edit by hand.\n\n")
        f.write("export interface Dx1Vector {\n  name: string;\n  modulation: 'VLDA4' | 'LDA2' | 'LDK';\n"
                "  hex: string;\n  expected: Record<string, unknown>;\n}\n\n")
        f.write("export const DX1_V3_VECTORS: Dx1Vector[] = ")
        f.write(json.dumps(out, indent=2))
        f.write(";\n")
    print(f"wrote {len(out)} vectors to {sys.argv[2]}")
else:
    print(json.dumps(out, indent=1))
