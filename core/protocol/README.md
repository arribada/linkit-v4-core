# core/protocol/

DTE (Data Terminal Equipment) protocol — the line-based command interface used over USB, BLE and UART for configuration and diagnostics.

## Files

| File | Role |
|------|------|
| `dte_handler.{cpp,hpp}` | Parses incoming DTE lines, dispatches to commands |
| `dte_commands.{cpp,hpp}` | All DTE command definitions (PARMR, PARMW, CMDR, FACTR, …) |
| `dte_params.cpp` | The `param_map[]` — DTE keys, encodings, ranges, permitted values. **Default values are not here**: they live in [`../configuration/config_store.hpp`](../configuration/config_store.hpp) `default_params`, aligned by index |
| `base_types.hpp` | Type tags used by the param map (`BaseEncoding`, `BaseMap`) |
| `base64.hpp` | Base64 encoding helper (vendor drop-in) |
| `dte_protocol.hpp` | Encoder/decoder + the Argos allcast `PassPredictCodec` |

## Adding a new command

1. Add the command name to the `DTECommand` enum in `dte_commands.hpp`.
2. Add a `DTECommandMap` entry in `dte_commands.cpp` (parser format string + handler).
3. Implement the handler.
4. If the command takes a new parameter, register it in `dte_params.cpp` (see [`../configuration/README.md`](../configuration/README.md)).

Wiki: [DTE Commands](https://github.com/arribada/linkit-v4-core/wiki/06-%E2%80%90-DTE-commands).
