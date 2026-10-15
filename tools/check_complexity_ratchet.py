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
regression OR any over-ceiling newcomer is outstanding.

ONE KEY CAN HOLD SEVERAL FUNCTIONS. The key is file + name + arity, chosen so a
renamed parameter does not read as a brand new function. Overloads of the same
arity in one file therefore share a key -- react(const SomeEvent &) is the shape
that matters, and m10qasync.cpp has nine of them. Pinning only the worst hid 352
CCN across 43 keys and 238 functions, with each of the others free to grow up to
the pinned maximum unremarked. So the pinned value is the whole sorted list and
the comparison is position by position. An overload ADDED to an existing key is
judged as a newcomer, against NEW_CEILING.

AND THE BASELINE FILE IS REQUIRED. If it is missing this exits non-zero instead
of writing one: creating a baseline from whatever is in the tree and reporting
success is how a guard disables itself, and this repo has already had one do
exactly that inside a git worktree.
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
        # One key can hold SEVERAL functions: overloads of the same arity in one
        # file. react(const SomeEvent &) is the shape that matters -- the event
        # handlers of every driver -- and m10qasync.cpp has nine of them under
        # one key. Keeping only the worst hid 352 CCN across 43 keys and 238
        # functions, so each of the others could have grown up to the pinned
        # maximum with the check green. The value is therefore the whole list,
        # compared position by position.
        found.setdefault(key, {"ccns": [], "path": path, "name": name})["ccns"].append(ccn)
    for v in found.values():
        v["ccns"].sort(reverse=True)
    return found


def _fmt(ccns):
    """A lone CCN prints bare, a key holding overloads prints its list."""
    lst = _as_list(ccns) if not isinstance(ccns, list) else ccns
    return str(lst[0]) if len(lst) == 1 else str(lst)


def load_baseline():
    if not BASELINE.exists():
        return None
    with open(BASELINE, encoding="utf-8") as f:
        return json.load(f)


def _pin_value(ccns):
    """One int for a lone function, the sorted list when a key holds overloads."""
    return ccns[0] if len(ccns) == 1 else list(ccns)


def _as_list(pin):
    """Baselines written before the overload fix hold a bare int."""
    return [pin] if isinstance(pin, int) else list(pin)


def write_baseline(found, note):
    pinned = {k: _pin_value(v["ccns"]) for k, v in sorted(found.items()) if v["ccns"][0] >= FLOOR}
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
        # Deliberately an ERROR, not a convenience. Writing a baseline from
        # whatever happens to be in the tree and exiting 0 is how a guard
        # disables itself: delete the file -- or land in a checkout that never
        # had it -- and the hook and CI go green with no ratchet at all. This
        # repo has already had a guard that silently disabled itself in a
        # worktree. Creating the baseline is a deliberate act: --update.
        print(f"ERROR: {BASELINE.relative_to(REPO)} is missing.")
        print("Without it there is NO complexity ratchet. If this is deliberate -- a first")
        print("baseline, or a reset you intend -- run:")
        print("    python3 tools/check_complexity_ratchet.py --update")
        print("Otherwise restore the file from git: it is tracked.")
        return 1

    pinned = base["functions"]
    regressions, newcomers, improvements = [], [], []

    for key, v in sorted(found.items()):
        ccns = v["ccns"]
        if key in pinned:
            was = sorted(_as_list(pinned[key]), reverse=True)
            # Position by position on the sorted lists, so one overload of a
            # nine-way react() cannot grow behind the worst one.
            overlap = min(len(was), len(ccns))
            if any(ccns[i] > was[i] for i in range(overlap)):
                regressions.append((key, was, ccns))
            elif any(ccns[i] < was[i] for i in range(overlap)) or len(ccns) < len(was):
                improvements.append((key, was, ccns))
            # An overload ADDED to an existing key is a new function and gets
            # the newcomer rule, not the pinned one.
            for extra in ccns[len(was):]:
                if extra > NEW_CEILING:
                    newcomers.append((key, extra))
        else:
            for ccn in ccns:
                if ccn > NEW_CEILING:
                    newcomers.append((key, ccn))

    gone = [k for k in pinned if k not in found]

    if args.tighten:
        # Only lower limits: a regression must still be fixed, not absorbed.
        if regressions:
            print("--tighten refuses to run while there are regressions:")
            for key, was, now in regressions:
                print(f"  {key}: {was} -> {now}")
            return 1
        # Refuse for the same reason it refuses on a regression: a newcomer over
        # the ceiling is a RED check, and pinning it would turn that red green.
        # The docstring promises this command only ever lowers a limit; it has
        # to keep that promise or it is the escape hatch --update was taken out
        # of the failure message for being.
        if newcomers:
            print("--tighten refuses to run while a new function is over the ceiling:")
            for key, ccn in newcomers:
                print(f"  {key}: CCN {ccn} > {NEW_CEILING}")
            print("Split it, or pin that one line by hand if it is genuinely intended.")
            return 1
        kept = dict(pinned)
        for key, _, now in improvements:
            kept[key] = _pin_value(now)
        for key in gone:
            del kept[key]
        # Every function at or above the floor, not only the ones that broke the
        # ceiling: a new function landing at 21 is under NEW_CEILING and so never
        # failed the check, but without pinning it here it could then grow to 25
        # unremarked. Pinning on each tighten is what makes this a ratchet rather
        # than a fence.
        for key, v in found.items():
            if key not in kept and v["ccns"][0] >= FLOOR:
                kept[key] = _pin_value(v["ccns"])
        base["functions"] = {k: v for k, v in sorted(kept.items()) if v >= FLOOR}
        base["_comment"] = "Tightened with --tighten: improvements pinned at their new value."
        BASELINE.write_text(json.dumps(base, indent=1, sort_keys=True) + "\n", encoding="utf-8")
        newly = len(base["functions"]) - len(pinned) + len(gone)
        print(f"baseline tightened: {len(improvements)} improvement(s) pinned, "
              f"{len(gone)} gone, {newly} function(s) newly pinned")
        return 0

    for key, was, now in improvements:
        print(f"  better  {key}: {_fmt(was)} -> {_fmt(now)}")
    if improvements:
        print(f"  ({len(improvements)} improvement(s) — pin them with --tighten)\n")

    ok = True
    if regressions:
        ok = False
        print("COMPLEXITY REGRESSION — these functions got harder to read:")
        for key, was, now in sorted(regressions, key=lambda r: max(now_i - was_i for now_i, was_i in zip(r[2], r[1])),
                                    reverse=True):
            worst = max(now_i - was_i for now_i, was_i in zip(now, was))
            print(f"  {key}: CCN {_fmt(was)} -> {_fmt(now)}  (worst +{worst})")
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

    worst = max(((v["ccns"][0], k) for k, v in found.items()), default=(0, "-"))
    print(f"{len(pinned)} function(s) pinned, none regressed. Worst: {worst[1]} at CCN {worst[0]}")
    if gone:
        print(f"{len(gone)} pinned function(s) no longer found (renamed or removed) — --tighten drops them")
    return 0


if __name__ == "__main__":
    sys.exit(main())
