#!/usr/bin/env python3
"""Guard the three parallel DTE parameter tables.

A parameter occupies the SAME INDEX in three separate tables:

  A  core/protocol/base_types.hpp        enum class ParamID   name + explicit index
  B  core/protocol/dte_params.cpp        param_map[]          key, encoding, range
  C  core/configuration/config_store.hpp default_params       the default value

B is already protected: dte_params.cpp carries a static_assert against
ParamID::__PARAM_SIZE. C is not, and cannot be: default_params is an aggregate
initialiser, so a missing entry compiles without a word. The slot then becomes
BaseType{} == std::string(""), config_store_fs.hpp sees the variant index
mismatch, and factory-resets that parameter on every boot -- silently, on a
sealed beacon at sea.

This script is the assert the compiler cannot give us.

Exit 0 if the three tables agree. No dependencies, runs in ~0.1 s.
"""
import re
import sys

SENTINELS = {"__PARAM_SIZE", "__NULL_PARAM"}


def braced_block(path, anchor):
    """Return the lines between the brace that opens after `anchor` and its match."""
    lines = open(path, encoding="utf8", errors="replace").read().split("\n")
    try:
        start = next(i for i, l in enumerate(lines) if anchor in l)
    except StopIteration:
        sys.exit(f"{path}: anchor {anchor!r} not found -- has the table been renamed?")
    depth = 0
    for i in range(start, len(lines)):
        depth += lines[i].count("{") - lines[i].count("}")
        if i > start and depth <= 0:
            return lines[start + 1:i]
    sys.exit(f"{path}: unterminated block after {anchor!r}")


enum = [m.group(1)
        for l in braced_block("core/protocol/base_types.hpp", "enum class ParamID")
        if (m := re.match(r"\s*([A-Za-z_]\w*)\s*=\s*\d+", l))
        and m.group(1) not in SENTINELS]

params = [m.group(1)
          for l in braced_block("core/protocol/dte_params.cpp", "const BaseMap param_map[]")
          if (m := re.match(r'\s*\{\s*"([^"]*)"', l))]

defaults = [m.group(1)
            for l in braced_block("core/configuration/config_store.hpp", "default_params {")
            if (m := re.match(r"\s*/\*\s*(?:\[\d+\]\s*)?([A-Za-z_]\w*)", l))]

print(f"ParamID={len(enum)}  param_map={len(params)}  default_params={len(defaults)}")

errors = []

if not (len(enum) == len(params) == len(defaults)):
    errors.append("the three tables do not have the same length")

# Names: ParamID and the /* NAME */ markers of default_params are kept in exact
# sync, so a mismatch here means an entry was inserted at the wrong index --
# which shifts every slot after it and misaligns the on-flash format.
#
# param_map names are NOT compared: they are the DTE-facing textual names and
# legitimately differ for a handful of slots (e.g. _RESERVED_ARGOS_FREQ marks a
# slot that is no longer writable over DTE while ParamID still calls it
# ARGOS_FREQ). Only their COUNT is checked, above.
for i, (e, d) in enumerate(zip(enum, defaults)):
    if e != d:
        errors.append(f"index {i}: ParamID says {e!r} but default_params says {d!r}")

if not errors:
    print(f"OK -- {len(enum)} slots, three tables aligned by index and by name.")
    sys.exit(0)

print("MISMATCH")
for e in errors[:10]:
    print(f"  {e}")
if len(errors) > 10:
    print(f"  ... and {len(errors) - 10} more")
print("""
Adding a parameter means THREE edits, always, in this order:
  1. core/protocol/base_types.hpp        enum class ParamID  (append + bump __PARAM_SIZE)
  2. core/protocol/dte_params.cpp        param_map[]         (append)
  3. core/configuration/config_store.hpp default_params      (append)  <-- the one
     that used to compile silently when forgotten.

Appending at the END of the table does NOT need a config version bump.
See core/configuration/README.md.""")
sys.exit(1)
