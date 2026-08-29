# template_conf/

Example DTE configuration files, to be pushed with
`pylinkit_cli config push <file>` or over USB/BLE.

## The templates

| File | Deployment | Argos mode | GNSS |
|---|---|---|---|
| [`turtle_doppler_only.cfg`](turtle_doppler_only.cfg) | Sea turtle, position reconstructed on the ground | `DOPPLER` | off |
| [`turtle_gps.cfg`](turtle_gps.cfg) | Sea turtle, on-board GNSS fixes | `SURFACING_BURST` | every 10 min |
| [`turtle_cloudlocate.cfg`](turtle_cloudlocate.cfg) | Sea turtle, raw measurements positioned in the cloud | `SURFACING_BURST` | raw snapshot |
| [`drifter.cfg`](drifter.cfg) | Drifting buoy, permanently at the surface | `PASS_PREDICTION` | hourly |
| [`fix_beacon.cfg`](fix_beacon.cfg) | Fixed ground station | `PASS_PREDICTION` | daily |
| [`rspb_avian_mortality_cyprus_boat.cfg`](rspb_avian_mortality_cyprus_boat.cfg) | RSPB Cyprus boat tracker, moored/underway | `LEGACY` + `MOORED_*` | every 5 min under way |

Each file carries its rationale inline: why the mode, and — more useful in
practice — which value would silence the beacon. The RSPB one targets an RSPB
build; nine of its keys sit behind `HAS_BOARD_RSPB`, `HAS_EXTERNAL_WAKEUP` or
`ENABLE_MORTALITY_SENSOR` and are reported as not implemented on a LinkIt
build.

## These files are written in PyLinkit's language, not the firmware's

They are pushed with `pylinkit_cli config push`, and PyLinkit sits between the
file and the wire. Two consequences that decide how every line is spelled:

**Parameter NAMES are PyLinkit's.** Thirty-one of them differ from the
firmware's name for the same key — `ARGOS_DUTY_CYCLE` not `DUTY_CYCLE`,
`GNSS_ENABLE` not `GNSS_EN`, `GNSS_DELTATIME_ACQ` not `DLOC_ARG_NOM`,
`UW_ENABLE` not `UNDERWATER_EN`. Reading the firmware table alone will lead you
to the wrong spelling.

**VALUES are human, not wire codes.** PyLinkit encodes them itself:

| Write | Not | Because |
|---|---|---|
| `ARGOS_MODE = SURFACING_BURST` | `5` | PyLinkit maps the name to the code |
| `LED_MODE = 24HRS` | `1` | accepted: `OFF`, `24HRS`, `ALWAYS` |
| `ARGOS_DEPTH_PILE = 16` | `10` | the depth, not the code for it |
| `GNSS_DELTATIME_ACQ = 10` | `1` | **minutes**, not the acquisition-period code |
| `GNSS_DYN_MODEL = SEA` | `5` | names again |
| `ARGOS_DUTY_CYCLE = FFFFFF` | `16777215` | read as **hexadecimal** |

The duty cycle deserves its own line: PyLinkit encodes it with `int(value, 16)`,
so `16777215` means `0x16777215` — 376926741, which the device refuses, leaving
whatever mask was already in place. `FFFFFF` is all twenty-four hours;
`000000` is a mute beacon.

`GNSS_FASTLOC_MODE` and `GNSS_CLOUDLOCATE_FORMAT` are the exception: PyLinkit
passes them through as plain integers, so they keep their numeric values.

The bench harness in [`../tests/bench/`](../tests/bench/) speaks the other
language — raw DTE, firmware names, wire codes — because it drives the device
directly. Do not carry a value across from one to the other.

## Validating a file

```bash
python3 tools/check_template_conf.py                       # every template
python3 tools/check_template_conf.py template_conf/x.cfg   # just one
```

PyLinkit checks neither the spelling of a name against the firmware nor any
numeric range: an unknown name is dropped, and an out-of-range value is only
refused by the device. The checker closes both gaps — it reads
`core/protocol/dte_params.cpp` for the keys and their limits, and
`tools/pylinkit_map.py` for the names and the human-value tables. It reports:

- an unknown parameter name — and, if you used the firmware's name for a
  parameter PyLinkit spells differently, which name to use instead;
- a read-only parameter (silently a no-op in a config file);
- a duplicate key;
- a value that is not one of the accepted words for a coded parameter;
- a value outside the firmware's `[min, max]`;
- a value the firmware permits nowhere — `HAULED_ARGOS_MODE` takes any Argos
  mode except `SURFACING_BURST`, since a hauled device is dry and stationary
  and would transmit exactly zero times;
- keys that only exist behind a build flag (informational);
- drift between `tools/pylinkit_map.py`, the firmware, and — when `PYLINKIT`
  points at a checkout — that installation of PyLinkit.

Set `PYLINKIT` whenever you can:

```bash
PYLINKIT=/path/to/pylinkit-v4 python3 tools/check_template_conf.py
```

Several codecs *transform* the value rather than look it up, and only the real
encoder can be trusted with those — the duty cycle above is exactly such a
case. Without `PYLINKIT` the checker says which lines it could not verify
instead of guessing at them.

Regenerate the mirror after a PyLinkit change:

```bash
python3 tools/gen_pylinkit_map.py /path/to/pylinkit-v4 > tools/pylinkit_map.py
```

## Two rules for whoever writes a new one

1. **Run the checker.** This is not theoretical. The two files that used to
   live here were removed in 2026-08 for naming parameters said not to exist —
   but that audit had been run against the *firmware* table, and most of the
   names it flagged (`ARGOS_DUTY_CYCLE`, `GNSS_ENABLE`, `UW_ENABLE`) are
   perfectly valid PyLinkit names. What it did catch for real was
   `LB_TRESHOLD`, a typo of `LB_THRESHOLD` that both tables agree on and that
   had fossilised across both files. The lesson is the same either way: check
   against the table that actually consumes the file, mechanically.

2. **No real credentials.** The removed files carried live CLS
   `ARGOS_RADIOCONF` values and an AES key. Templates are examples;
   provisioning is a separate step.

The parameter reference is the
[Parameters wiki page](https://github.com/arribada/linkit-v4-core/wiki/09-%E2%80%90-Parameters)
and [`../core/protocol/dte_params.cpp`](../core/protocol/dte_params.cpp).
