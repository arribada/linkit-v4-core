#!/usr/bin/env python3
"""Check that every relative markdown link in our own docs resolves.

Scope is deliberately narrow: only `[text](path)` links, never file names
mentioned in backticks. Widening it to prose would flag things like
`config.dat`, `SWS.CAL` or `*.hex`, and a check with false positives is a check
that gets switched off.

Vendor trees are never scanned.

Exit 0 if every link resolves.
"""
import os
import re
import sys

ROOTS = ["README.md", "CONTRIBUTING.md", "core", "ports", "tests", "scripts",
         "template_conf", "docs"]
SKIP = ("nRF5_SDK", "libraries/", "_deps", "/build/", "node_modules")
LINK = re.compile(r"\[[^\]]*\]\(([^)\s]+)\)")

docs = []
for root in ROOTS:
    if os.path.isfile(root):
        docs.append(root)
    elif os.path.isdir(root):
        for dirpath, _, names in os.walk(root):
            if any(s in dirpath + "/" for s in SKIP):
                continue
            docs += [os.path.join(dirpath, n) for n in names if n.endswith(".md")]

broken = []
for doc in sorted(docs):
    base = os.path.dirname(doc)
    for m in LINK.finditer(open(doc, encoding="utf8", errors="replace").read()):
        target = m.group(1)
        if target.startswith(("http://", "https://", "mailto:", "#")):
            continue
        target = target.split("#", 1)[0]          # drop the anchor
        if not target:
            continue
        if not os.path.exists(os.path.normpath(os.path.join(base, target))):
            broken.append((doc, m.group(1)))

if not broken:
    print(f"OK -- {len(docs)} markdown files, every relative link resolves.")
    sys.exit(0)

print(f"{len(broken)} broken link(s):")
for doc, target in broken:
    print(f"  {doc}  ->  {target}")
print("\nA README that points at a file which does not exist is worse than no README:"
      "\nit sends the next reader looking for something that was renamed or deleted.")
sys.exit(1)
