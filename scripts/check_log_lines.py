#!/usr/bin/env python3
"""No log line is longer than a journal entry can hold.

WHAT THIS PROTECTS. A DEBUG_* line is formatted into one LogEntry: 128 bytes,
9 of header, 118 of text and its terminator (MAX_LOG_PAYLOAD, messages.hpp).
Whatever does not fit is cut -- in flash and on the bench console alike, since
both format into that same entry. On 2026-10-27, 53 format strings could not
fit even with every %u printing a single digit: duty-cycle and BLIND
configuration warnings among them, each cut just before the part that says
what to do.

THE RULE. The text of the format string, in UTF-8 bytes ("—" is three, "°" and
"µ" two), plus one byte per conversion, must fit 118 bytes. That is the shortest
the line can ever print, so a failure here is a line that is ALWAYS cut. Values
can still push a line over at run time; keep the fixed text well short of the
limit. The explanation belongs in a comment next to the call.

Covered: DEBUG_ERROR, DEBUG_WARN, DEBUG_INFO, DEBUG_DETAIL with a literal
format, in core/ and ports/nrf52840/ outside the vendor trees.

Run:  python3 scripts/check_log_lines.py
"""

import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
ROOTS = ["core", "ports/nrf52840"]
SKIP = re.compile(r"^ports/nrf52840/(drivers|bootloader|bsp/.*/sdk)/")
PAYLOAD = 118

CALL = re.compile(r'\bDEBUG_(ERROR|WARN|INFO|DETAIL)\s*\(\s*((?:"(?:[^"\\]|\\.)*"\s*)+)[,)]', re.S)
LITERAL = re.compile(r'"((?:[^"\\]|\\.)*)"')
CONVERSION = re.compile(r"%[-+ #0]*(?:\d+|\*)?(?:\.(?:\d+|\*))?(?:hh|h|ll|l|z|j|t|L)?([diouxXeEfgGcsp%])")
ESCAPE = re.compile(r"\\(x[0-9A-Fa-f]{1,2}|[0-7]{1,3}|.)")

# Fewer calls than this means the pattern stopped matching the tree.
MIN_CALLS = 1000


def text_bytes(fmt):
    """Shortest printed length of a format string, in bytes."""
    unescaped = ESCAPE.sub("?", fmt)  # any C escape is one byte
    n = 0
    pos = 0
    for m in CONVERSION.finditer(unescaped):
        n += len(unescaped[pos:m.start()].encode("utf-8"))
        n += 1  # '%%' prints '%', every other conversion prints >= 1 char
        pos = m.end()
    return n + len(unescaped[pos:].encode("utf-8"))


def main():
    calls = 0
    failures = []
    for root in ROOTS:
        for path in sorted((REPO / root).rglob("*")):
            if path.suffix not in (".c", ".cpp", ".h", ".hpp"):
                continue
            rel = path.relative_to(REPO).as_posix()
            if SKIP.match(rel):
                continue
            text = path.read_text(errors="replace")
            for m in CALL.finditer(text):
                calls += 1
                fmt = "".join(LITERAL.findall(m.group(2)))
                n = text_bytes(fmt)
                if n > PAYLOAD:
                    line = text.count("\n", 0, m.start()) + 1
                    failures.append(f"{rel}:{line}: DEBUG_{m.group(1)} prints at least {n} bytes "
                                    f"(> {PAYLOAD}): \"{fmt[:70]}...\"")
    if calls < MIN_CALLS:
        print(f"FAIL: only {calls} DEBUG_* calls parsed -- the pattern stopped matching the tree")
        return 1
    if failures:
        print("\n".join(failures))
        print(f"FAIL: {len(failures)} log line(s) always cut at {PAYLOAD} bytes -- shorten the text, "
              "move the explanation to a comment")
        return 1
    print(f"OK: {calls} log lines fit a journal entry")
    return 0


if __name__ == "__main__":
    sys.exit(main())
