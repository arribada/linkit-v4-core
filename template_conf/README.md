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

Each file carries its rationale inline: why the mode, and — more useful in
practice — which value would silence the beacon.

The `.cfg` format itself is generic — `pylinkit --parmw <file>` serves the whole
family (Linkit, Horizon, RSPB). What is **not** generic is writing a whole
template to a device on the bench: the wave that does it (`TPL-*`) reads
`RSPB_PACKET_FORMAT` and skips itself on anything but an RSPB board, because
pushing a full profile to the KIM2 bench board reconfigures the very device
under validation and represents no deployment anyone performs.

The RSPB file additionally has nine keys behind `HAS_BOARD_RSPB`,
`HAS_EXTERNAL_WAKEUP` or `ENABLE_MORTALITY_SENSOR`, reported as not implemented
anywhere else.

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

## The depth pile only pays off on LDA2

`ARGOS_DEPTH_PILE` is the most consequential setting in these files after the
mode itself, and its benefit depends on something no config file can state: how
the module is provisioned.

A LONG Argos packet is **192 bits** and carries up to three positions. **LDK
carries 128**, so a LONG does not fit. On a module transmitting in LDK the
firmware caps the retrieval to **one position per transmission** — deliberately,
and at retrieve time, so a position whose transmission KIM2 would drop does not
also spend one of its redundancy slots. The pile then carries nothing and a
battery budget computed as "three positions per transmission" is wrong by a
factor of three.

The four turtle templates ship `ARGOS_ADAPTIVE_MODULATION = 0` (decision
2026-10), so adaptive modulation is **no longer a way to reach LDA2** for them.
Every frame goes on the modulation of the **master RCONF** (`ARGOS_RADIOCONF`),
and the per-modulation RCONFs (`ARGOS_RADIOCONF_LDK/LDA2/VLDA4`) only matter
for the rare non-adaptive escalation paths. One condition, then:

**The master RCONF decodes to LDA2.** On an LDK master, for these templates:

- GNSS positions go out **one per transmission** (SHORT, 96 bits padded to the
  128-bit LDK frame); a pile of 4 is redundancy, not throughput.
- **CloudLocate MEAS20 is never transmitted.** It is 192 bits, an LDA2-only
  frame. `turtle_cloudlocate.cfg` and `turtle_argos_surface.cfg` use MEAS20
  (`GNSS_CLOUDLOCATE_FORMAT = 1`), so on an LDK master they report no
  CloudLocate at all. The slot is not lost: it falls back to the last cached
  position or to a Doppler (firmware fix 2026-10 — before it, the slot went out
  empty, on every ping and heartbeat while the snapshot was held).
- The Doppler — the low-battery `DOPPLER` fallback included
  (`LB_ARGOS_TX_REPETITION = 600`) — follows the master too: a 4-byte frame on
  LDA2, a 128-bit (16-byte) frame with a zero tail on LDK. Count that in any
  energy estimate, and check that the ground decoder classifies it as a Doppler.

**An empty master is not a fallback, it is a provisioning error.** The factory
value of `ARGOS_RADIOCONF` is empty, and the templates do not set it (no real
credentials here, see below). With an empty master the KIM2 programs a
hard-coded LDK RCONF: the unit transmits as an LDK-master unit with all of the
above, and an SMD unit does not transmit at all. (The KIM2 no longer writes
that default back into `ARGOS_RADIOCONF`, so the master stays visibly empty.)
A unit that only ever ran with adaptive on may never have had its master
written, because adaptive used the per-modulation RCONFs instead.

Before deployment, write the master and read back what it decodes to, then
read the firmware's own verdict:

1. `PARMW` the configuration (template + `ARGOS_RADIOCONF`);
2. `SATVF` — with `force=1` on KIM2, which re-programs the master and reads it
   back (the unforced KIM2 read only sees the module's power-on RCONF and does
   not count); on SMD the dirty credentials are written and read back;
3. `STATR ART12,ART13` — `ART13` is the master modulation from that genuine
   read-back (0 LDA2, 1 LDK, 2 VLDA4, 3 not verified), `ART12` the warning
   bitmask: bit0 the CloudLocate format does not fit the master (MEAS20 needs
   LDA2), bit1 not verified yet, bit2 master empty, bit3 MEAS50 on Argos.
   **`ART12` must read 0.** The firmware warns, it never refuses.

The read-back is persisted (`ART13` with the fingerprint of the master it came
from, `ART14`), so after a reset the firmware puts its modulation cache back to
it by itself; an SMD unit whose master was never read back tries once, at the
next start, per master. Changing `ARGOS_RADIOCONF` or
`ARGOS_ADAPTIVE_MODULATION` over `PARMW` invalidates `ART13` until the next
read-back.

The bench board is exactly the trap: it carries all three per-modulation RCONFs
but its master decodes to LDK, so with adaptive off no LONG packet and no MEAS20
can leave it. The `PROV-01` bench case reads these facts and returns that
verdict for a given board.

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
- **keys that are set but inert in this very configuration** (informational) —
  a GNSS timeout with the receiver off, a low-battery threshold with `LB_EN=0`,
  `ARGOS_BLIND_RETX_PERIOD_S` in `DOPPLER` where the period comes from the
  sequence instead. Neither an error nor a silence: a setting the operator made
  that will do nothing, and that no response from the port would ever mention;
- **combinations that produce a silent beacon** — every parameter in range,
  every name valid, and a tracker that never transmits: `PASS_PREDICTION` with
  the GNSS off, `SURFACING_BURST` without the water sensor, CloudLocate in any
  mode but `SURFACING_BURST`, an empty duty-cycle mask, a critical battery
  threshold above the low one;
- **provisioning prerequisites** (informational) — what the file needs from
  the device that no template can state, such as CloudLocate MEAS20 with
  adaptive off requiring a master RCONF that decodes to LDA2;
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

### Checking it automatically

`tools/hooks/pre-commit` runs the checker whenever a commit touches
`template_conf/`, `dte_params.cpp` or the mirror — the three things that can
put a template out of step. Git does not version its own hooks, so install it
once per clone:

```bash
cp tools/hooks/pre-commit .git/hooks/ && chmod +x .git/hooks/pre-commit
```

### Why this is not done in the firmware

`is_implemented` is fixed at compile time, so `PARML` and `PARMR` already hide
what a build cannot honour — `tools/params_for_build.py` shows exactly what a
given build exposes. Hiding *mode-inert* parameters would be different: the set
of keys returned would then vary with the configuration, and a host that reads,
modifies and writes back would silently drop everything the current mode masks.
A configuration backup taken in one mode would not restore in another. Saying it
here costs nothing and breaks nothing.

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
