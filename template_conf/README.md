# template_conf/

Example DTE configuration files, to be pushed with
`pylinkit_cli config push <file>` or over USB/BLE.

## Currently empty, on purpose

The two files that used to live here — `turtle_tracker.cfg` and
`rspb_mortality.cfg` — were removed in 2026-08. They had drifted badly against
the firmware: **32 of 135 keys** in the turtle file and **47 of 287** in the RSPB
one named parameters that no longer exist (`ARGOS_DUTY_CYCLE`, `GNSS_ENABLE`,
`UW_ENABLE`, `LB_TRESHOLD` — a typo that had fossilised, and others). The
firmware skips unknown keys cleanly and reports them, so nothing broke; but a
quarter of a reference file shipped to operators described settings that were
never applied, which is worse than having no file at all.

New per-application templates will be written here. Two rules for whoever writes
them:

1. **Every key must exist in the firmware.** Check against `name` in
   [`../core/protocol/dte_params.cpp`](../core/protocol/dte_params.cpp), or run:

   ```bash
   python3 - <<'PY'
   import re
   names = set(re.findall(r'^\s*\{\s*"([^"]+)"', open('core/protocol/dte_params.cpp').read(), re.M))
   for l in open('template_conf/YOUR_FILE.cfg'):
       k = l.split('=')[0].strip()
       if k and not l.lstrip().startswith(('#', ';', '[')) and k not in names:
           print('unknown key:', k)
   PY
   ```

2. **No real credentials.** The removed files carried live CLS `ARGOS_RADIOCONF`
   values and an AES key. Templates are examples; provisioning is a separate step.

The parameter reference is the
[Parameters wiki page](https://github.com/arribada/linkit-v4-core/wiki/09-%E2%80%90-Parameters)
and [`../core/protocol/dte_params.cpp`](../core/protocol/dte_params.cpp).
