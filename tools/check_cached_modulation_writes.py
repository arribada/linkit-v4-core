#!/usr/bin/env python3
"""ARGOS_CACHED_MODULATION (SMP01) has exactly one writer.

WHY. With adaptive modulation off, the firmware trusts the master modulation
it recorded (ART13) only while it came from a genuine read-back of the current
master (ART14). Every SMP01 write therefore has to say what it is: a plain
cache update (escalation, deferred switch, a read-back that is not of the
master) or a genuine master read-back, which also records ART13/ART14. Both go
through ConfigurationStore::write_cached_modulation() /
write_master_modulation_readback() in core/configuration/config_store.hpp.

A future writer that calls write_param(ParamID::ARGOS_CACHED_MODULATION, ...)
directly skips that bookkeeping, and nothing else would notice: the drivers
that hold those writes are nRF-only and have no host test. This check fails
the pre-commit hook and CI instead.

Scans core/ and ports/nrf52840/ (build directories, the Nordic SDK and vendor
libraries excluded) plus tests/ -- a test that pokes SMP01 directly would hide
exactly the bypass this guards against; tests use the helpers too.
"""
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# The one place allowed to write it: the helper itself.
ALLOWED = {os.path.join('core', 'configuration', 'config_store.hpp'): 1}

# Matched against each folder's path RELATIVE to ROOT, never the absolute one:
# a git worktree of this repo lives under .claude/worktrees/<id>/, so an
# absolute match on '/.claude/' excluded every file in it. The scan then found
# nothing, including inside the helper, and the check failed with "expected
# exactly 1 write, found 0" on a tree where nothing was wrong -- loudly here,
# but the same bug in a check that counts offenders would have passed vacuously.
EXCLUDED = ('/build/', '/drivers/', '/nRF5_SDK', '/external/', '/libraries/', '/.claude/')
PATTERN = re.compile(r'write_param\s*(<[^>]*>)?\s*\(\s*ParamID::ARGOS_CACHED_MODULATION\b')


def files():
    for base in ('core', 'ports/nrf52840', 'tests/src', 'tests/mocks', 'tests/fakes'):
        top = os.path.join(ROOT, base)
        for folder, _, names in os.walk(top):
            rel_folder = '/' + os.path.relpath(folder, ROOT).replace(os.sep, '/') + '/'
            if any(x in rel_folder for x in EXCLUDED):
                continue
            for n in names:
                if n.endswith(('.cpp', '.hpp', '.c', '.h')):
                    yield os.path.join(folder, n)


def main():
    found = {}
    offenders = []
    for path in files():
        try:
            text = open(path, encoding='utf-8', errors='replace').read()
        except OSError:
            continue
        for m in PATTERN.finditer(text):
            rel = os.path.relpath(path, ROOT)
            line = text.count('\n', 0, m.start()) + 1
            found[rel] = found.get(rel, 0) + 1
            if rel not in ALLOWED:
                offenders.append('%s:%d' % (rel, line))
    for rel, n in ALLOWED.items():
        if found.get(rel, 0) != n:
            offenders.append('%s: expected exactly %d write in the helper, found %d' % (rel, n, found.get(rel, 0)))
    if offenders:
        print('ARGOS_CACHED_MODULATION written outside ConfigurationStore::write_cached_modulation():')
        for o in offenders:
            print('  ' + o)
        print('Use write_cached_modulation() (cache only) or write_master_modulation_readback() '
              '(genuine read-back of the master, adaptive off).')
        return 1
    print('OK -- ARGOS_CACHED_MODULATION has a single writer')
    return 0


if __name__ == '__main__':
    sys.exit(main())
