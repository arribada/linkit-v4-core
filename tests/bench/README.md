# Autonomous bench — LinkIt V4 KIM

Hardware-in-the-loop test bench. Plug a board in and validate **everything from the
logs** with no magnet, no antenna and no operator: enter config mode, push any
configuration, inject a synthetic GPS position, exercise the Argos TX path, and
flash fixes over SWD — all driven from this directory.

## How it works

The firmware, built with `--bench` (`-DBENCH_TEST=ON`, implies `--debug`), adds a
tiny `%`-prefixed console on the **USB-CDC** link. It coexists with the normal CLS
**DTE** protocol (`$CMD#LEN;payload`) and with the debug log stream on the same
port — the host driver demuxes them.

| Command | Effect |
|---|---|
| `%PING` | Handshake → `%BENCH OK state=<S>` |
| `%STATE` | Current FSM state |
| `%CFG` | **Enter ConfigurationState without the reed magnet** (synthesises the confirmation gesture) |
| `%OP` | Leave config → Operational |
| `%GPS <lat> <lon> [hAcc_mm] [numSV]` | **Inject a synthetic 3D fix** straight into the post-fix pipeline (no antenna) |

Once in config, the full DTE surface is available over USB: `PARMR`/`PARMW` (all
230 params), `STATR`, `SENSR`, `SATVF`, `ARGOSTX`, `GNSSI`, `KIMBR`, …

Firmware pieces: [bench_console.cpp](../../ports/nrf52840/core/interface/bench_console.cpp),
`GPSService::bench_inject_fix()` in [gps_service.cpp](../../core/services/gps_service.cpp),
the `%` route in [gentracker.cpp](../../core/sm/gentracker.cpp), wiring in
[main.cpp](../../ports/nrf52840/main.cpp). All behind `#ifdef BENCH_TEST` — **zero
footprint in production builds**.

## One-time setup (WSL2)

USB isn't native to WSL2; bridge the board's CDC port and the J-Link probe from
Windows with [usbipd-win](https://github.com/dorssel/usbipd-win):

```bash
./wsl_usb.sh list                 # find busids (board CDC = Nordic 1915:xxxx, J-Link = 1366:0101)
./wsl_usb.sh attach <busid>       # attach the board's CDC (and the J-Link)
./wsl_usb.sh keep <busid>         # auto-attach: re-attaches after every flash reset (run in bg)
```
First `attach`/`bind` of each device needs a one-off, elevated
`usbipd bind --busid <id>` on Windows (the script prints the exact line if attach
fails). After binding, `./wsl_usb.sh keep <busid> &` makes the port survive every
flash-induced re-enumeration with no further clicks — this is what makes the
flash→validate→fix→reflash loop fully hands-off.

### Why the firmware needs a bench flag for USB (not just DTR)

The nRF USB-CDC only arms RX / ungates TX on the CDC `PORT_OPEN` event, which the
host raises by asserting **DTR**. Over WSL2/usbip that DTR control transfer is not
reliably forwarded, so the link looks dead (no logs, no `%` replies). The
`BENCH_TEST` build therefore forces the port open once USB is enumerated
(see [nrf_usb.cpp](../../ports/nrf52840/core/hardware/nrf_usb.cpp) `NrfUSB::process()`).
Production keeps strict DTR gating.

## Full loop

```bash
# 1. Build the bench firmware
./scripts/build_linkitv4_kim.sh --bench

# 2. Flash it over SWD (J-Link)
tests/bench/flash.sh --full          # first flash (app+BL+SD, chiperase)
tests/bench/flash.sh                  # subsequent flashes (app-only, fast)

# 3. Validate autonomously
tests/bench/kim_bench.py --detect     # find board + handshake
tests/bench/kim_bench.py --run        # full suite, PASS/FAIL report
tests/bench/kim_bench.py --monitor    # just stream timestamped logs
tests/bench/kim_bench.py --shell      # type % or $ commands by hand
tests/bench/kim_bench.py --gps -21.0097 55.2707   # one-shot fix injection
```

Every session is transcript-logged to `tests/bench/logs/<timestamp>.log`.

## Validation suite (`--run`)

1. handshake (`%PING`)
2. reed bypass → config (`%CFG`)
3. DTE reachable (`STATR`)
4. read device identity (`PARMR`)
5. param write/read round-trip (`PARMW`/`PARMR`)
6. satellite credentials verify (`SATVF` — KIM2 modulation/RCONF health)
7. exit to operational (`%OP`)
8. synthetic GPS fix injection (`%GPS`) → logging pipeline
9. Argos/TX activity after fix *(soft check — regex tuned against the first real capture)*

## Argos TX backoff / suspension suite (`argos_backoff_bench.py --run`)

Separate script, separate run. `kim_bench.py --run` stays the regression
baseline (10/10) and is unchanged; this one is the on-board counterpart to the
device-error tests in `tests/src/argos_tx_test.cpp`. Flash first with a short
deadline, or the probe check waits an hour:

```bash
ARGOS_TX_ERROR_SUSPEND_S=120 ./scripts/build_linkitv4_kim.sh --bench
./tests/bench/flash.sh
python3 tests/bench/kim_bench.py --run            # baseline, expect 10/10
python3 tests/bench/argos_backoff_bench.py --run  # then this
```

What it checks, in order: the console answers and the board is operational ·
`%SCHED` reports an `ARGOSTX` schedule · no suspension is armed at boot · a
`%GPS` injection lifts the first-fix TX gate · the backoff ladder reads 60 s
then 120 s · the third strike arms the deadline the firmware was built with ·
**the suspension does not spin** · the service is parked rather than
rescheduling at zero delay · exactly one probe fires when the deadline passes ·
a GPS session clears the suspension, and clears the earliest-TX floor with it ·
a surface event (`%DIVE`/`%SURFACE`) clears it too · the scheduler queue has not
leaked.

The spin check is the one worth the trip. Until the probe deadline landed, the
suspension completed without a reschedule, which left the safety-net timeout
armed; it fired, rescheduled at zero delay, hit the guard in `service_initiate`
and rearmed itself, so the board woke, logged and skipped once per
`service_next_timeout` forever — an LFS commit every pass, and no transmission.
Nothing in a host test would have made that visible as a *cost*.

**The interesting half only runs if TX actually fails.** On a board with valid
KIM2 credentials the transmissions succeed, there is no error ladder, and those
checks report `SKIP` rather than `FAIL` — that is the correct outcome, not a
gap in the script. To exercise the ladder deliberately, run against a board
whose KIM2 credentials are absent or whose module is disconnected: the TX is
then attempted and refused, which is the fault this machinery exists for.

## Notes / limits

- Bench builds are **debug** (logs on, `DEBUG_NO_WATCHDOG`): not for power-draw testing.
  Never flash a `--release` build to the bench — it silences USB logs (`g_debug_mode=NONE`).
- Log assertions target **INFO/WARN** lines; `DEBUG_TRACE` is filtered at `DEBUG_LEVEL=3`
  (e.g. `task_process_gnss_data` is TRACE — assert on `retry_counter: reset ... (PVT fix)`).
- `%GPS` injects at the `GPSService` layer — it exercises the real post-fix pipeline
  (log → config-store → prepass → Argos scheduling) but bypasses the M10Q and does
  **not** GPS-sync the RTC. A real M10Q read is still available in config via `SENSR`.
- **Argos live TX needs device state the injection can't fake:** KIM2 `RCONF`
  credentials + a GPS-synced RTC. On a board without credentials, `SATVF` returns
  `$N` and the TX service logs `no RCONF configured` / `TX held until first GPS fix`
  — the suite reports this as "TX subsystem alive, TX gated" (correct, not a failure).
  To validate a real transmission, program KIM2 credentials first.

## Console latency after `%OP` — read this before adding a check

Leaving configuration mode starts every service, and the KIM2 bring-up alone is
several seconds of UART round-trips (RCONF read + validate). The bench console is
polled from that same main loop, so its replies queue behind the work: a `%STATE`
sent immediately after `%OP` routinely answers **2 to 5 seconds late**.

That is not a fault. What *is* a fault is not waiting for it: the late reply is
then read as the answer to the NEXT command, and every following check measures
the wrong thing. That is what made the suite report 8-9/10 for months — check 8
failed while its own log line, two lines further down the transcript, proved the
injection had happened.

`Bench.settle_console()` drains that backlog (it pings until the console answers
within a second). Call it after anything that makes the board busy, and give
commands issued in that window a generous timeout. Do not lower them back.

## Live runs

- **2026-07-03**, board Argos ID 4189092 — 10/10 on a real KIM board: reed bypass,
  config R/W (`IDT06=3FEBA4`, model "LinkIt V4"), `PARMW/PARMR` round-trip, `%GPS`
  injection accepted as a PVT. Real M10Q present (`GNSS UID 8FAF580F2E`). Argos TX
  gated — this unit has no KIM2 RCONF credentials.
- **2026-08-28**, same board — 10/10, three consecutive runs, after fixing the
  console-latency desynchronisation described above. The same firmware built from
  `main` scored 8-9/10 on five runs before the harness fix, so the defect was in
  the harness, not the firmware.

---

# RSPB bench (SMD satellite over SPI · UART observe-only)

RSPB differs from KIM: the SMD satellite module talks **SPI** (internal), and the
board has **no USB** — its debug UART is **TX-only (UARTE1, P0.11, 921600)**. So the
RSPB bench is **observe-only**: the firmware drives itself and we watch the log
stream over a **USB-UART adapter**.

### Firmware (`./scripts/build_rspb.sh --bench`, implies --debug)
- `PMU::powerdown()` → **soft reset** ("simulated TPL wake") instead of System OFF,
  so the real TPL5111 can't cut power and drop control. Compressed duty-cycle:
  `boot → work → powerdown → reset → boot …` (state persisted each time, so the
  pseudo-RTC chain + boot-modulo continue exactly as after a real TPL wake).
- **Auto-injects one synthetic GPS fix per "run" boot** (`bench::start_auto_inject`)
  so the SMD satellite TX fires with no antenna.
- All `#ifdef BENCH_TEST` — zero production footprint.

### Wiring
```
USB-UART adapter RX  <---  RSPB debug TX (P0.11)
USB-UART adapter GND <-->  RSPB GND
(adapter TX unused — the debug UART has no RX)
```
Adapter appears as `/dev/ttyUSB0` (FTDI/CP210x) in WSL (attach via `wsl_usb.sh`).

### Run
```bash
./scripts/build_rspb.sh --bench          # build
tests/bench/flash.sh   # (RSPB variant; or nrfjprog --program ...RSPB...merged.hex --chiperase --verify --reset)
# upload a config below via your BLE/PyLinkit path, then:
tests/bench/rspb_bench.py --cycles 5     # observe + validate N duty-cycles
```

### Config sets to upload (you upload; I observe & validate)
Each is a param set; RCONF/SECKEY are inlined in `argos_validate.py` (they used to be
quoted from a template under `template_conf/`, which no longer ships one).

| # | Scenario | Key params | Expected in the log |
|---|---|---|---|
| 1 | Basic LEGACY TX | `ARGOS_MODE=LEGACY`, `ARGOS_DEPTH_PILE=1`, RCONF/SECKEY set | boot → `auto_inject: injecting synthetic fix` → `TX START type=gnss` → `TX SUCCESS` → `simulated TPL wake` |
| 2 | BLIND | `ARGOS_BLIND_EN=1`, `ARGOS_BLIND_RETX_NB=3`, `ARGOS_BLIND_RETX_PERIOD_S=60` | `SmdSat: BLIND KMAC loaded (retx_nb=3 period=60s)` + `TX START/SUCCESS` |
| 3 | Depth pile 4 (LONG) | `ARGOS_DEPTH_PILE=4` | `LONG packet, N position(s)` after a few cycles + adaptive modulation |
| 4 | Boot-modulo duty | `WAKEUP_PERIOD` / modulo params | some cycles `MODULO-SKIP` (`Not our turn to run`), run cycles TX |
| 5 | No-fix heartbeat | (don't provision creds, or block GNSS) | `0xFF heartbeat` grid filler TX |

`rspb_bench.py` prints one line per duty-cycle: `RUN`/`MODULO-SKIP`, inject, TX
START/SUCCESS/FAIL counts, BLIND retx, heartbeat, and the reset that closes it.
