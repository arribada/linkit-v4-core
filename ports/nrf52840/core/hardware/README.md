# ports/nrf52840/core/hardware/

Device drivers — both nRF peripheral wrappers and external sensor/module drivers.

## nRF peripheral wrappers (loose files)

| File | Peripheral |
|------|-----------|
| `nrf_gpio.cpp` | `GPIOPins` implementation |
| `nrf_i2c.{cpp,hpp}` | I2C master |
| `nrf_spim.{cpp,hpp}` | SPI master (DMA) |
| `nrf_uart_async.{cpp,hpp}` | Async UART (libuarte_async wrapper used by KIM2 / SMD / LoRa) |
| `nrf_usb.{cpp,hpp}` | USB CDC |
| `nrf_rtc.{cpp,hpp}` | RTC |
| `nrf_timer.{cpp,hpp}` | Timer |
| `nrf_irq.{cpp,hpp}` | IRQ priority helpers |
| `nrf_pmu.cpp` | Power management, low-power, reset cause |
| `nrf_battery_mon.{cpp,hpp}` | Analog battery monitor |
| `nrf_switch.{cpp,hpp}` | Reed/saltwater switch input |
| `nrf_dfu.cpp` | DFU bootloader entry |
| `nrf_debug_uart.{cpp,hpp}` | Debug log UART (separate from comm UARTs) |
| `nrf_peripheral_power.hpp` | SDK workaround: full POWER-register reset for UARTE / SAADC |
| `nrf_memory_access.hpp`, `nrf_rgb_led.hpp` | Misc |

## External device drivers (one folder per device)

| Folder | Device | Notes |
|--------|--------|-------|
| `kim2/` | CLS KIM2 Argos module | UART AT, 9600 baud (instance 1) |
| `smd_sat/` | Arribada SMD Argos module | SPI, A+ protocol |
| `lora_rak3172/` | RAK3172-SiP LoRa module | UART AT (RUI3), 115200 baud (instance 1) |
| `m10qasync/` | u-blox M10Q GPS | UART async + UBX parser, 460800 baud (instance 0) |
| `bma400/` | Accelerometer | I2C |
| `lps28dfw/` | Pressure / temperature | I2C |
| `ms58xx/` | MS58xx pressure (CDT) | I2C |
| `bar100/` | KELLER bar100 pressure | I2C |
| `stc3117/` | Battery fuel-gauge | I2C |
| `thermistor/` | NTC thermistor | ADC |
| `tsys01/` | Sea temperature | I2C |
| `oem_ph/`, `oem_rtd/`, `ezo_rtd/` | EZO / OEM probes | I2C |
| `ad5933/` | Conductivity AFE | I2C |
| `ltr_303/` | Ambient light | I2C |
| `pressure_sensor/`, `cdt/` | Cross-driver wrappers (model selection) | — |
| `run_cam/` | RunCam camera: `runcam` (button), `runcam_uart_link` (UART, see `core/hardware/runcam_uart_device`) | GPIO pulse / UARTE1 |

## Comm-module driver pattern

Each comm module (KIM2, SMD, LoRa) follows the same two-layer pattern:
- **`*_comm`** — UART/SPI TX/RX, AT response parsing, raw event emission.
- **`*` (top-level)** — state machine, implements `KineisDevice` interface (used by `ArgosTxService`).

## Adding a new sensor

1. Create `mysensor/` here, with `.cpp`/`.hpp` implementing the interface defined in [`../../../../core/hardware/`](../../../../core/hardware/).
2. Reference it in [`../../CMakeLists.txt`](../../CMakeLists.txt) under a `-DENABLE_MYSENSOR=ON` flag.
3. Wire a service in [`../../../../core/services/`](../../../../core/services/).
4. Add a host stub in [`../../../../tests/mocks/`](../../../../tests/mocks/) and a unit test in [`../../../../tests/src/`](../../../../tests/src/).
