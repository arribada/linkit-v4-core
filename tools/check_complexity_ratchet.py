#!/usr/bin/env python3
"""No function gets more complex than it already is.

WHY A RATCHET AND NOT A THRESHOLD. The obvious guard -- fail when lizard emits
more warnings than before -- is worse than no guard at all, and that is
measured, not assumed: splitting bench_console.cpp's handle_line() took its
worst function from CCN 118 to 12 and took the FILE from 4 warnings to 8,
because the five functions it became are each counted. A warning-count gate
would have rejected the change that fixed the problem. What actually has to not
regress is the worst function.

So this pins every function already above FLOOR at its current CCN and refuses
to let it grow, while capping anything new at NEW_CEILING. Splitting a monster
is always allowed: the pieces are new functions, and each must come in under
the ceiling.

WHAT THE NUMBERS MEAN. lizard runs on the tree, so every #if branch is counted
at once and the CCN of any single built variant is lower -- differently for each
of the seven. The absolute values are therefore inflated. That does not matter
for a ratchet, which only ever compares this measurement against the same
measurement taken earlier; it does mean the baseline figures are not the
complexity of any binary that was ever flashed.

The thresholds come from the distribution at the time the baseline was taken
(2782 functions): p50=1, p75=4, p90=8, p95=14, p99=32. FLOOR=15 is just above
p95, which keeps the baseline to the ~125 functions worth tracking instead of
all 2782 -- a file that would churn on every rename. NEW_CEILING=25 is roughly
the top 2%: a new function may not join it.

    python3 tools/check_complexity_ratchet.py            # check
    python3 tools/check_complexity_ratchet.py --update    # re-pin the WHOLE tree (see below)
    python3 tools/check_complexity_ratchet.py --tighten   # pin improvements only

--update is a reset, not a remedy: it re-pins every function from the current
tree, so running it to silence one complaint silently accepts every other
regression present at the same time. --tighten is the one that is safe to run
habitually -- it only ever lowers a limit, and it refuses outright while any
regression is outstanding.
"""

import argparse
import csv
import json
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
BASELINE = REPO / "tools" / "complexity_baseline.json"

# Our code only. The excluded trees are the same ones the clang-format rule
# skips: vendored SDK, board support and bootloader are not ours to simplify.
SCAN = [
    "core",
    "ports/nrf52840/core",
    "ports/nrf52840/main.cpp",
]

FLOOR = 15  # functions at or above this are pinned individually
NEW_CEILING = 25  # a function absent from the baseline may not exceed this


def measure():
    """{key: (ccn, file, name)} for every function lizard finds."""
    cmd = ["lizard", *SCAN, "-l", "cpp", "--csv"]
    try:
        out = subprocess.run(cmd, cwd=REPO, capture_output=True, text=True, check=False)
    except FileNotFoundError:
        sys.exit("lizard not found: pip install lizard")
    if not out.stdout.strip():
        sys.exit(f"lizard produced nothing (exit {out.returncode}): {out.stderr[:400]}")

    found = {}
    for row in csv.reader(out.stdout.splitlines()):
        if len(row) < 9:
            continue
        try:
            ccn, params = int(row[1]), int(row[3])
        except ValueError:
            continue  # a header line, if lizard ever grows one
        path, name = row[6], row[7]
        # Keyed on arity rather than the full signature: a renamed parameter
        # would otherwise read as a brand new function and be judged against
        # NEW_CEILING instead of its own history.
        key = f"{path}::{name}/{params}"
        # Same key twice means same name and arity in one file (a template
        # specialisation, or two statics in different anonymous scopes). Keep
        # the worst, so the ratchet cannot be loosened by adding a simple twin.
        if key not in found or ccn > found[key][0]:
            found[key] = (ccn, path, name)
    return found


def load_baseline():
    if not BASELINE.exists():
        return None
    with open(BASELINE, encoding="utf-8") as f:
        return json.load(f)


def write_baseline(found, note):
    pinned = {k: v[0] for k, v in sorted(found.items()) if v[0] >= FLOOR}
    BASELINE.write_text(
        json.dumps(
            {
                "_comment": note,
                "_floor": FLOOR,
                "_new_ceiling": NEW_CEILING,
                "functions": pinned,
            },
            indent=1,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    return len(pinned)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--update", action="store_true", help="rewrite the baseline from the tree")
    ap.add_argument("--tighten", action="store_true", help="pin improvements, keep every limit")
    args = ap.parse_args()

    found = measure()

    if args.update:
        n = write_baseline(found, "Taken with --update. Every function at or above _floor, pinned at its CCN.")
        print(f"baseline rewritten: {n} functions pinned, floor {FLOOR}")
        return 0

    base = load_baseline()
    if base is None:
        n = write_baseline(found, "First baseline.")
        print(f"no baseline yet — written: {n} functions pinned, floor {FLOOR}")
        return 0

    pinned = base["functions"]
    regressions, newcomers, improvements = [], [], []

    for key, (ccn, path, name) in sorted(found.items()):
        if key in pinned:
            if ccn > pinned[key]:
                regressions.append((key, pinned[key], ccn))
            elif ccn < pinned[key]:
                improvements.append((key, pinned[key], ccn))
        elif ccn > NEW_CEILING:
            newcomers.append((key, ccn))

    gone = [k for k in pinned if k not in found]

    if args.tighten:
        # Only lower limits: a regression must still be fixed, not absorbed.
        if regressions:
            print("--tighten refuses to run while there are regressions:")
            for key, was, now in regressions:
                print(f"  {key}: {was} -> {now}")
            return 1
        kept = dict(pinned)
        for key, _, now in improvements:
            kept[key] = now
        for key in gone:
            del kept[key]
        # Every function at or above the floor, not only the ones that broke the
        # ceiling: a new function landing at 21 is under NEW_CEILING and so never
        # failed the check, but without pinning it here it could then grow to 25
        # unremarked. Pinning on each tighten is what makes this a ratchet rather
        # than a fence.
        for key, (ccn, _path, _name) in found.items():
            if key not in kept and ccn >= FLOOR:
                kept[key] = ccn
        base["functions"] = {k: v for k, v in sorted(kept.items()) if v >= FLOOR}
        base["_comment"] = "Tightened with --tighten: improvements pinned at their new value."
        BASELINE.write_text(json.dumps(base, indent=1, sort_keys=True) + "\n", encoding="utf-8")
        newly = len(base["functions"]) - len(pinned) + len(gone)
        print(f"baseline tightened: {len(improvements)} improvement(s) pinned, "
              f"{len(gone)} gone, {newly} function(s) newly pinned")
        return 0

    for key, was, now in improvements:
        print(f"  better  {key}: {was} -> {now}")
    if improvements:
        print(f"  ({len(improvements)} improvement(s) — pin them with --tighten)\n")

    ok = True
    if regressions:
        ok = False
        print("COMPLEXITY REGRESSION — these functions got harder to read:")
        for key, was, now in sorted(regressions, key=lambda r: r[2] - r[1], reverse=True):
            print(f"  {key}: CCN {was} -> {now}  (+{now - was})")
    if newcomers:
        ok = False
        # Deliberately NOT advertising --update here. It rewrites the whole
        # baseline from the tree, so it would absorb every other regression
        # along with this one -- the escape hatch would be a ratchet reset.
        # Splitting the function, or a reviewed hand-edit of the one line in
        # tools/complexity_baseline.json, are the two honest answers.
        print(f"NEW FUNCTION OVER THE CEILING ({NEW_CEILING}) — split it, or pin just this")
        print("line by hand in tools/complexity_baseline.json (do NOT run --update: it")
        print("re-pins the whole tree and would swallow any other regression):")
        for key, ccn in sorted(newcomers, key=lambda r: -r[1]):
            print(f"  {key}: CCN {ccn}")

    if not ok:
        print("\nSplitting a complex function is always allowed: the pieces are new")
        print(f"functions and each only has to come in under {NEW_CEILING}.")
        return 1

    worst = max(((v[0], k) for k, v in found.items()), default=(0, "-"))
    print(f"{len(pinned)} function(s) pinned, none regressed. Worst: {worst[1]} at CCN {worst[0]}")
    if gone:
        print(f"{len(gone)} pinned function(s) no longer found (renamed or removed) — --tighten drops them")
    return 0


if __name__ == "__main__":
    sys.exit(main())
