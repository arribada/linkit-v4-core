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

## Validating a file

```bash
python3 tools/check_template_conf.py                       # every template
python3 tools/check_template_conf.py template_conf/x.cfg   # just one
```

The checker reads `core/protocol/dte_params.cpp` and `dte_protocol.hpp`, so it
follows the firmware rather than a copy of it. It reports:

- an unknown parameter name;
- a read-only parameter (silently a no-op in a config file);
- a duplicate key;
- a value outside `[min, max]`;
- a value that is not an accepted input for a coded parameter;
- a coded input that decodes to a value outside the permitted set;
- keys that only exist behind a build flag (informational).

### The two traps it exists to catch

**Coded parameters take a code, not the value.** `ARGOS_DEPTH_PILE` accepts
`1 2 3 4 8 9 10 11 12`, which decode to depths `1 2 3 4 8 12 16 20 24`. Writing
`ARGOS_DEPTH_PILE = 16` is rejected at push time; the code for a 16-deep pile
is `10`. `DLOC_ARG_NOM` and `MOORED_DLOC` are the same: `4` means one hour, not
four seconds.

**A permitted set can be narrower than the decoder.** `HAULED_ARGOS_MODE`
decodes `0..5` like any Argos mode, but only `0..4` are permitted — a hauled
device is dry and stationary by definition, so `SURFACING_BURST` there would
transmit exactly zero times.

## Two rules for whoever writes a new one

1. **Every key must exist in the firmware, and every value must be accepted.**
   Run the checker above. This is not theoretical: the two files that used to
   live here were removed in 2026-08 after drifting badly — **32 of 135 keys**
   in the turtle file and **47 of 287** in the RSPB one named parameters that
   no longer existed (`ARGOS_DUTY_CYCLE`, `GNSS_ENABLE`, `UW_ENABLE`,
   `LB_TRESHOLD` — a typo that had fossilised). The firmware skips unknown keys
   cleanly and reports them, so nothing broke; but a quarter of a reference file
   shipped to operators described settings that were never applied.

2. **No real credentials.** The removed files carried live CLS
   `ARGOS_RADIOCONF` values and an AES key. Templates are examples;
   provisioning is a separate step.

The parameter reference is the
[Parameters wiki page](https://github.com/arribada/linkit-v4-core/wiki/09-%E2%80%90-Parameters)
and [`../core/protocol/dte_params.cpp`](../core/protocol/dte_params.cpp).
