#!/usr/bin/env python3
"""No push-pull output in a BSP pin table carries a pull resistor.

WHAT THIS PROTECTS. While an output is driven, the pin sets the level, so its
internal pull either does nothing (same level) or conducts VDD over ~13 kOhm
(opposite level). GPS_PWR_EN, SAT_EN and SAT_WKUP were push-pull outputs with a
pull-down, driven HIGH for the whole GNSS deep-idle window or satellite session:
the A/B probe of 2026-10-27 (%PULLAB, LinkIt, PPK2) measured the pull-down of a
HIGH GPS_PWR_EN at 117 +/- 13 uA with USB sharing the supply, ~0.25 mA expected
on battery alone -- about 10 % of a turtle's daily budget, on a pin whose pull
bought nothing: a reset returns PIN_CNF to a disconnected input with no pull, so
the resistor is not there during the reset it was meant to cover either.

WHY A SOURCE CHECK. No host test runs the BSP, and no bench measurement before
that date had the GNSS rail on: the reference sleep figure was taken with the
pin LOW, where the pull-down costs nothing.

Open-drain drives (S0D1, H0D1, D0S1, D0H1) are left alone: there the pull is
what provides the level the pin does not drive.

Run:  python3 scripts/check_gpio_output_pulls.py
"""

import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
BSP_GLOB = "ports/nrf52840/bsp/*/bsp.cpp"

# Drive modes where the pin drives BOTH levels.
PUSH_PULL = {"S0S1", "H0S1", "S0H1", "H0H1"}

ENTRY = re.compile(
    r"/\*\s*(?P<name>GPIO_\w+|GPOI_\w+)\s*\*/\s*\{\s*NRF_GPIO_PIN_MAP\([^)]*\)\s*,"
    r"\s*NRF_GPIO_PIN_DIR_(?P<dir>INPUT|OUTPUT)\s*,"
    r"\s*NRF_GPIO_PIN_INPUT_\w+\s*,"
    r"\s*NRF_GPIO_PIN_(?P<pull>NOPULL|PULLUP|PULLDOWN)\s*,"
    r"\s*NRF_GPIO_PIN_(?P<drive>\w+)\s*,"
)

# Fewer entries than this in a BSP file means the pattern stopped matching the
# table, not that the board lost its pins: fail loudly instead of passing.
MIN_ENTRIES = 15


def main():
    files = sorted(REPO.glob(BSP_GLOB))
    if not files:
        print(f"FAIL: no BSP file matches {BSP_GLOB}")
        return 1
    failures = []
    for path in files:
        text = path.read_text()
        entries = list(ENTRY.finditer(text))
        rel = path.relative_to(REPO)
        if len(entries) < MIN_ENTRIES:
            failures.append(f"{rel}: only {len(entries)} pin entries parsed -- the table format changed, fix the pattern")
            continue
        for m in entries:
            if m["dir"] == "OUTPUT" and m["pull"] != "NOPULL" and m["drive"] in PUSH_PULL:
                line = text.count("\n", 0, m.start()) + 1
                failures.append(f"{rel}:{line}: {m['name']} is a push-pull output ({m['drive']}) with "
                                f"{m['pull']} -- use NRF_GPIO_PIN_NOPULL")
        print(f"{rel}: {len(entries)} pins checked")
    if failures:
        print("\n".join(failures))
        return 1
    print("OK: no push-pull output carries a pull")
    return 0


if __name__ == "__main__":
    sys.exit(main())
