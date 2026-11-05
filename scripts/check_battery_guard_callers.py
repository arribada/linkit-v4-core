#!/usr/bin/env python3
"""Every transmission service still calls the critical-battery guard.

WHAT THIS PROTECTS. Service::shutdown_if_battery_critical is the one code path
that calls PMU::powerdown() on a tag sealed in resin, with its reed switch under
the glue and no downlink. Firing it wrongly ends the mission; not firing it when
the cell is genuinely flat means a brown-out mid-transmission instead of a
clean stop.

WHY A SOURCE CHECK AND NOT A TEST. The guard's own logic is unit-tested
(tests/src/battery_critical_guard_test.cpp). What no host test can reach is
whether the SERVICES still call it. The guard sits behind
`if (argos_config.is_lb)`, and is_lb is `LB_EN && m_is_battery_level_low`
(config_store.hpp:1230) -- a cached flag, refreshed in exactly one place,
ConfigStoreLFS::update_battery_level (config_store_fs.hpp:455). The host
double, FakeConfigurationStore, overrides update_battery_level() with an empty
body, so that flag is permanently false in every host test and the whole
low-battery regime of both TX services is unreachable from there. Measured:
giving the fake the real body makes 10 existing tests fail, because they rely on
that branch never being taken.

So this is what is checkable today: the call is present, in every service that
transmits. It is the shape of the defect that actually happened -- the two
services carried a copy each, the copies diverged, and a LoRa tag went on
powering down on a transmit-current transient -- except that now the way to
reintroduce it is to stop calling, or to add a third transmission service and
forget.

Run:  python3 scripts/check_battery_guard_callers.py
"""

import re
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
GUARD = "shutdown_if_battery_critical"

# Every service that can start a transmission. A new one belongs here; that is
# the point, and the check says so when the list and the tree disagree.
EXPECTED_CALLERS = [
    "core/services/argos_tx_service.cpp",
    "core/services/lora_tx_service.cpp",
]

# Where the one definition lives.
DEFINITION = "core/services/service.cpp"


def strip_comments(text):
    """Comments out. Without this the check passes on a file that merely MENTIONS
    the guard: lora_tx_service.cpp has a comment naming
    Service::shutdown_if_battery_critical, and a plain substring search was
    satisfied by it after the real call had been deleted. Verified by deleting
    the call and watching the first version of this script report OK."""
    text = re.sub(r"/\*.*?\*/", " ", text, flags=re.S)
    text = re.sub(r"//[^\n]*", "", text)
    return text


def body_of(text, signature_fragment):
    """The brace-matched body of the first function whose header contains the
    fragment. Crude on purpose: no compiler here, and the alternative is to
    grep the whole file and miss which function the call sits in."""
    i = text.find(signature_fragment)
    if i < 0:
        return None
    j = text.find("{", i)
    if j < 0:
        return None
    depth = 0
    for k in range(j, len(text)):
        if text[k] == "{":
            depth += 1
        elif text[k] == "}":
            depth -= 1
            if depth == 0:
                return text[j : k + 1]
    return None


def main():
    problems = []

    # 1. One definition, and it is the shared one.
    #
    # git ls-files, NOT rglob. This repo keeps its git worktrees INSIDE itself
    # at .claude/worktrees/, and a second full clone under .claude/ as well --
    # 2179 .cpp files of other trees. rglob walked all of them, so the moment a
    # worktree was made from a branch that has this function the check saw two
    # definitions and hard-failed, blocking every commit to the TX services.
    # Verified by creating exactly that worktree and watching it happen. This
    # repo has already had a guard disable itself in a worktree
    # (.claude/CLAUDE.md); the opposite failure is no better.
    tracked = subprocess.run(
        ["git", "-C", str(REPO), "ls-files", "-z", "--", "*.cpp"],
        capture_output=True,
        text=True,
        check=False,
    )
    if tracked.returncode != 0:
        print(f"ERROR: git ls-files failed: {tracked.stderr.strip()[:200]}")
        return 1
    definers = []
    for rel in tracked.stdout.split("\0"):
        if not rel or rel.startswith(("libraries/", "ports/nrf52840/drivers/", "tests/")):
            continue
        path = REPO / rel
        if not path.exists():
            continue
        text = strip_comments(path.read_text(encoding="utf-8", errors="replace"))
        if re.search(rf"\b\w+::{GUARD}\s*\(", text):
            definers.append(rel)
    if definers != [DEFINITION]:
        problems.append(
            f"{GUARD} should be defined once, in {DEFINITION}; found: {definers or 'nowhere'}.\n"
            "    Two definitions is how the previous divergence happened."
        )

    # 2. Every expected caller calls it, from its scheduling decision.
    for rel in EXPECTED_CALLERS:
        path = REPO / rel
        if not path.exists():
            problems.append(f"{rel} is gone; update EXPECTED_CALLERS in this script.")
            continue
        text = strip_comments(path.read_text(encoding="utf-8", errors="replace"))
        if f"{GUARD}(" not in text:
            problems.append(
                f"{rel} no longer calls {GUARD}.\n"
                "    That service can now plan a transmission on a confirmed-flat cell."
            )
            continue
        # It has to be in the scheduling decision, not merely somewhere in the
        # file: that is the function whose verdict stops the service.
        body = body_of(text, "::service_next_schedule(")
        if body is None:
            problems.append(f"{rel}: cannot find service_next_schedule() to check the call site.")
        elif f"{GUARD}(" not in body:
            problems.append(
                f"{rel}: {GUARD} is called, but not from service_next_schedule().\n"
                "    Only that function's return value stops the service scheduling a transmission."
            )

    # 3. A transmission service that is not on the list at all.
    for path in sorted((REPO / "core" / "services").glob("*_tx_service.cpp")):
        rel = path.relative_to(REPO).as_posix()
        if rel not in EXPECTED_CALLERS:
            problems.append(
                f"{rel} looks like a transmission service and is not in EXPECTED_CALLERS.\n"
                f"    Either it calls {GUARD} and belongs on the list, or it plans transmissions\n"
                "    without the critical-battery gate."
            )

    if problems:
        print(f"ERROR: the critical-battery guard is not wired as expected ({len(problems)} problem(s)):")
        for p in problems:
            print(f"  - {p}")
        return 1

    print(f"OK -- {GUARD} defined once in {DEFINITION}, called from service_next_schedule() "
          f"in all {len(EXPECTED_CALLERS)} transmission service(s).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
