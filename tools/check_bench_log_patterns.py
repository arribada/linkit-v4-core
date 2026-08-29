#!/usr/bin/env python3
"""Check that every firmware log string the bench greps for actually exists.

Three separate false results in one day came from the same mistake: a bench
case waiting on a message the firmware never emits.

  - DC-01 searched for "DUTY_CYCLE mask is 0x000000"; the firmware says
    "duty-cycle mask is 0". The case failed a perfectly sound firmware, and
    on a BLOCKING criterion.
  - endurance.py counted "TX ABORT", which appears nowhere; aborted
    transmissions are logged "aborting fire" or "TX aborted", so a run full
    of them reported zero errors.
  - log_harvest.py looked for "ttff=" in system.log, where it never appears --
    ttff is a column of the sensor-data CSV.

A missing string is silent in the worst direction: the test goes green (or red)
for a reason unrelated to the firmware.

    python3 tools/check_bench_log_patterns.py
"""
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BENCH = [os.path.join(ROOT, "tests/bench", f)
         for f in ("dte_campaign.py", "endurance.py", "log_harvest.py")]
SOURCES = ("core", "ports/nrf52840/core")

# Literals that are not firmware log text: bench console replies, our own
# verdict words, regex fragments, and protocol tokens checked elsewhere.
IGNORE = re.compile(
    r"^(%|\$|PASS$|FAIL$|SKIP$|ERROR$|BLOQUANT|MAJEUR|MINEUR|OPERATIONAL|CONFIGURATION|"
    r"[A-Z]{2,3}[0-9]{2}$|[\W\d]*$)")

# A quoted string is only worth checking when it reads like a log sentence:
# several words, at least one lowercase letter, no format specifiers.
CANDIDATE = re.compile(r"^(?=.*[a-z])(?!.*%[sdulx])[\w\- :=/,'()\.]{12,}$")


def firmware_text():
    out = []
    for rel in SOURCES:
        for dirpath, dirnames, filenames in os.walk(os.path.join(ROOT, rel)):
            dirnames[:] = [d for d in dirnames if d != "build"]
            for fn in filenames:
                if fn.endswith((".cpp", ".hpp", ".c", ".h")):
                    try:
                        out.append(open(os.path.join(dirpath, fn), errors="ignore").read())
                    except OSError:
                        pass
    return "\n".join(out)


def main():
    fw = firmware_text()
    manquants = []
    verifies = 0
    for path in BENCH:
        if not os.path.exists(path):
            continue
        src = open(path).read()
        for m in re.finditer(r"""r?['"]([^'"\\\n]{12,120})['"]""", src):
            lit = m.group(1)
            if IGNORE.match(lit) or not CANDIDATE.match(lit):
                continue
            # Only literals that look like they are being searched for.
            # Le contexte doit etre une RECHERCHE, pas un message de verdict.
            # Sans ce filtre les libelles francais de r.record() remontent en
            # masse et noient les vraies trouvailles.
            ligne = src[max(0, m.start() - 90):m.start()]
            if not re.search(r"(\bin l\b|\bin ligne\b|re\.(search|match|findall|compile)|"
                             r"expect\(|_compter_trace\(|_attendre_trace\(|motifs\s*=)", ligne):
                continue
            if re.search(r"r\.record\(|append\(f?['\"]", ligne):
                continue
            # Fragment recolle entre deux chaines adjacentes, ou texte de
            # commentaire: ce n est pas un motif recherche.
            if re.search(r"[()\[\]{}]|\bif\b|\bhasattr\b|\blen\b", lit):
                continue
            debut_ligne = src.rfind("\n", 0, m.start()) + 1
            if src[debut_ligne:m.start()].lstrip().startswith("#"):
                continue
            verifies += 1
            if lit not in fw:
                lineno = src[:m.start()].count("\n") + 1
                manquants.append((os.path.relpath(path, ROOT), lineno, lit))

    print(f"{verifies} chaine(s) de log verifiee(s) contre les sources firmware")
    if manquants:
        print(f"\n{len(manquants)} INTROUVABLE(S) dans le firmware:")
        for f, n, lit in manquants:
            print(f"  {f}:{n}: {lit!r}")
        return 1
    print("toutes presentes")
    return 0


if __name__ == "__main__":
    sys.exit(main())
