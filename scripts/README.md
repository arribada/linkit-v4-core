# scripts/

Build, setup and utility scripts.

## Setup

| Script | Purpose |
|--------|---------|
| `setup_environment.sh` | Detect / install ARM GCC, nrfutil, nrfjprog, CMake. Generates `build_config.sh` |

## Build

| Script | Output | Comm |
|--------|--------|------|
| `build_linkitv4_kim.sh`  | `ports/nrf52840/build/LINKIT/`      | KIM2 (default) |
| `build_linkitv4_smd.sh`  | `ports/nrf52840/build/LINKIT_SMD/`  | SMD (`-DARGOS_SMD=ON`) |
| `build_linkitv4_lora.sh` | `ports/nrf52840/build/LINKIT_LORA/` | LoRa (`-DLORA_RAK3172=ON`) |
| `build_rspb.sh`          | `ports/nrf52840/build/RSPB/`        | SMD on RSPB board |
| `build_with_bootloader.sh <target> [--clean] [--recover]` | Merged hex (app + bootloader + SoftDevice) | any |
| `build_unit_tests.sh`    | `tests/build/TrackerTests`          | host |

All build scripts source `build_config.sh` (created by `setup_environment.sh`) and use `git describe --dirty` to embed the firmware version.

## Other

| Script | Purpose |
|--------|---------|
| `run_tests.sh` | Build and run the host test suite |
| `log_stack_dump.py` | Decode a hex stack-dump captured from the device using `addr2line` |
| `check_param_tables.py` | Guard the three parallel DTE parameter tables (`ParamID`, `param_map[]`, `default_params`). Compares lengths, and names between the first and third — which catches an entry inserted at the wrong index. Run by the `consistency` CI job. |
| `check_doc_links.py` | Check that every relative markdown link in our own docs resolves. Markdown links only, never file names in backticks — widening it to prose produces false positives, and a check with false positives gets switched off. Run by the `consistency` CI job. |

Both take no arguments, need only `python3`, and run in well under a second:

```bash
python3 scripts/check_param_tables.py && python3 scripts/check_doc_links.py
```

## Adding a new build target

Copy an existing `build_*.sh`, adjust the build dir / CMake flags, and document it in [`../README.md`](../README.md).
