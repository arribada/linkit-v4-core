#!/usr/bin/env python3
"""Regenerate tools/pylinkit_map.py from a PyLinkit v4 checkout.

    python3 tools/gen_pylinkit_map.py /path/to/pylinkit-v4 > tools/pylinkit_map.py

Config files in template_conf/ are consumed by PyLinkit, so they are written
in PyLinkit's space: its parameter names, and human values it encodes to the
wire codes itself. The firmware table cannot answer either question, hence
this mirror.
"""
import re
import sys

# PyLinkit derives AQPERIOD from a dict comprehension the plain parser below
# cannot evaluate, so the table is spelled out; it matches
# DTEProtocol::decode_acquisition_period in the firmware.
AQPERIOD_MINUTES = [0, 10, 15, 30, 60, 120, 180, 240, 360, 720, 1440, 1, 2, 5, 20, 45]


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    root = sys.argv[1]
    params = open(f'{root}/pylinkit/protocol/dte_params.py').read()
    types_src = open(f'{root}/pylinkit/protocol/dte_types.py').read()

    entries = re.findall(r'\[\s*"([A-Z0-9_]+)"\s*,\s*"([A-Z]{2,3}[0-9]{2})"\s*,\s*(\w+)', params)
    if not entries:
        sys.exit(f'no parameter entries found in {root}')

    coded = {}
    for m in re.finditer(r'class (\w+)\(\):\n(.*?)(?=\nclass |\Z)', types_src, re.S):
        nom, corps = m.group(1), m.group(2)
        a = re.search(r'allowed\s*=\s*(\[[^\]]*\])', corps, re.S)
        if a:
            try:
                coded[nom] = ('index', eval(a.group(1)))
            except Exception:
                pass
        if nom == 'AQPERIOD':
            coded[nom] = ('minutes', AQPERIOD_MINUTES)

    out = sys.stdout.write
    out('"""PyLinkit\'s view of the parameter table, mirrored into the repository.\n\n')
    out('Config files in template_conf/ are pushed with `pylinkit_cli config push`,\n')
    out("so they are written in PYLINKIT's space, not the firmware's: PyLinkit's\n")
    out("parameter NAMES (31 of them differ from the firmware's for the same key),\n")
    out('and human VALUES that PyLinkit encodes to the wire codes itself -- minutes\n')
    out('for an acquisition period, a depth for the depth pile, "SURFACING_BURST"\n')
    out('rather than 5.\n\n')
    out('Regenerate against a PyLinkit checkout with:\n\n')
    out('    python3 tools/gen_pylinkit_map.py /path/to/pylinkit-v4 > tools/pylinkit_map.py\n\n')
    out('tools/check_template_conf.py cross-checks this mirror against a live\n')
    out('PyLinkit when PYLINKIT points at one, so drift is reported, not assumed away.\n')
    out('"""\n\n')
    out('# key -> PyLinkit parameter name\nNAMES = {\n')
    for n, k, t in entries:
        out(f'    {k!r}: {n!r},\n')
    out('}\n\n# key -> PyLinkit type name\nTYPES = {\n')
    for n, k, t in entries:
        out(f'    {k!r}: {t!r},\n')
    out('}\n\n')
    out('# type -> ("index", [values by wire code]) or ("minutes", [minutes by wire code])\n')
    out('# A -1 entry marks a wire code PyLinkit does not expose.\nCODED = {\n')
    for t in sorted(coded):
        kind, vals = coded[t]
        out(f'    {t!r}: ({kind!r}, {vals!r}),\n')
    out('}\n')


if __name__ == '__main__':
    main()
