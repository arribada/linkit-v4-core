# core/services/

Long-running services that orchestrate tracker behavior. Each service inherits from `Service` and is registered with the cooperative scheduler ([`../scheduling/`](../scheduling/)).

## Categories

| Domain | Files |
|--------|-------|
| **GPS** | `gps_service.{cpp,hpp}` (M10Q async + fastloc) |
| **Argos TX** | `argos_tx_service.{cpp,hpp}`, `argos_packet_builder.{cpp,hpp}`, `depth_pile.{cpp,hpp}` |
| **Argos RX** | `argos_rx_service.{cpp,hpp}` |
| **LoRa TX** | `lora_tx_service.{cpp,hpp}` (mirrors Argos TX behavior) |
| **Underwater** | `uwdetector_service.{cpp,hpp}`, `sws_analog_*.{cpp,hpp}` |
| **Sensors** | `*_sensor_service.hpp` (axl, pressure, thermistor, sea_temp, als, ph, cdt) |
| **Mortality** | `mortality_service.{cpp,hpp}` (RSPB) |
| **Mode classifiers** | `hauled_mode_service.{cpp,hpp}` (hauled-out animal), `moored_mode_service.{cpp,hpp}` (stationary vessel), `rate_limiter.{cpp,hpp}` — static classes, not `Service` subclasses: state in `.noinit` RAM + CRC16, fed from the `ServiceManager::notify_peer_event` funnel, consumed by the `ConfigurationStore` override cascade |
| **Cloudlocate** | `cloudlocate_*.{cpp,hpp}` |
| **Power / Cam** | `low_battery_service.hpp`, `cam_service.hpp` |

## Adding a new service

1. Create `mything_service.{hpp,cpp}` here, inherit from `Service`, implement `service_init()`, `service_term()`, `service_task()`.
2. Instantiate and register it in [`../sm/gentracker.cpp`](../sm/gentracker.cpp) (`operational_state_entry`).
3. Wire any DTE parameters via [`../configuration/`](../configuration/) and [`../protocol/dte_params.cpp`](../protocol/dte_params.cpp).
4. Emit/consume events through `EventEmitter<Listener>` (see [`../hardware/events.hpp`](../hardware/events.hpp)).
