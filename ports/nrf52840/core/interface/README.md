# ports/nrf52840/core/interface/

External communication interfaces (operator-facing, not satellite/LoRa uplink).

| File | Role |
|------|------|
| `ble_interface.{cpp,hpp}` | BLE GATT service (Nordic S140 SoftDevice) — DTE over Bluetooth |
| `usb_interface.hpp` | USB CDC ACM — DTE over USB |
| `ble_stm_ota.{c,h}` | BLE STM OTA service for firmware updates over BLE |

DTE traffic flows from these interfaces into [`../../../../core/protocol/`](../../../../core/protocol/) (`DTEHandler`). When the bridge mode is active, raw bytes pass through to the comm module ([`../hardware/`](../hardware/)) instead.
