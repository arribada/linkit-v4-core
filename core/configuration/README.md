# core/configuration/

Persistent configuration storage and parameter definitions.

## Files

| File | Role |
|------|------|
| [`config_store.hpp`](config_store.hpp) | `ConfigurationStore` interface + the `default_params` table (**default values**) |
| [`config_store_fs.hpp`](config_store_fs.hpp) | LittleFS-backed implementation (serialization to `config.dat`) |
| [`calibration.hpp`](calibration.hpp) / [`calibration.cpp`](calibration.cpp) | Sensor calibration data (separate from DTE params) |

## The parameter table lives in three places, aligned by index

There is no single source of truth today. A parameter occupies **the same index**
in three tables, and all three must be edited together:

| # | File | Holds |
|---|------|-------|
| A | [`../protocol/base_types.hpp`](../protocol/base_types.hpp) — `enum class ParamID` | the symbolic name and its explicit numeric index |
| B | [`../protocol/dte_params.cpp`](../protocol/dte_params.cpp) — `param_map[]` | DTE 5-char key, encoding, min/max, permitted values, `is_implemented`, `is_writable` |
| C | [`config_store.hpp`](config_store.hpp) — `default_params` | **the default value** |

Only B is protected: [`dte_params.cpp:528`](../protocol/dte_params.cpp) carries a
`static_assert` that its length matches `ParamID::__PARAM_SIZE`. **C has no guard at
all** — a forgotten default compiles silently, the slot becomes `std::string("")`,
and `config_store_fs.hpp` factory-resets that parameter on every boot.

## Adding a new parameter

1. **Append** an entry to `enum class ParamID` in
   [`../protocol/base_types.hpp`](../protocol/base_types.hpp), just before
   `__PARAM_SIZE`, with an explicit index — then bump `__PARAM_SIZE` itself.
2. **Append** the matching `BaseMap` entry to `param_map[]` in
   [`../protocol/dte_params.cpp`](../protocol/dte_params.cpp)
   (DTE key, encoding, min/max, `is_implemented`, `is_writable`).
3. **Append** the default value to `default_params` in
   [`config_store.hpp`](config_store.hpp). *This is the step with no compile-time
   guard — do not skip it.*
4. For conditional parameters, prefer the boolean column
   (`is_implemented = (FEATURE == 1)`) used by B and C over a `#if` in the enum.
5. Mirror the entry in the host tool (PyLinkit `dte_params.py`) and, if the
   parameter is operator-facing, in [`../../template_conf/`](../../template_conf/).

## Do NOT bump the config version for an append

`m_config_version_code` ([`config_store.hpp:173`](config_store.hpp)) is **not** a
changelog counter.

- **Appending at the end of the table needs no bump.** No existing slot moves;
  reading an old `config.dat` simply runs out of entries, the loop applies the
  default and continues.
- **Removing or moving a slot always needs one**, otherwise deserialization
  misaligns and silently factory-resets every parameter after it (this is exactly
  what ARP36/ARP37 caused in 2026-06).

A bump keeps **only** `ARGOS_DECID` / `ARGOS_HEXID` and factory-resets everything
else — including the LoRaWAN credentials (`LORA_DEVEUI`/`APPKEY`/`DEVADDR`/
`APPSKEY`/`NWKSKEY`) of every provisioned unit, which then cannot rejoin its
network, with nothing in the logs to say why. The full reasoning and the version
history are in the comment above the constant.
