# core/hardware/

Pure-virtual hardware abstractions. **No platform-specific code** — every header here defines an interface, the concrete implementation lives in [`../../ports/`](../../ports/).

## Common interfaces

| File | Interface |
|------|-----------|
| `gpio.hpp` | `GPIOPins` (set/clear/pulse, sensor power refcount) |
| `pmu.hpp` | Power management, timestamps, delays, reset cause |
| `rtc.hpp` | Real-time clock |
| `gps.hpp` | GPS receiver |
| `battery.hpp` | Battery monitor |
| `switch.hpp`, `reed.{cpp,hpp}` | Reed switch / saltwater switch input |
| `dfu.hpp` | DFU bootloader entry |
| `cam.hpp` | Camera trigger |
| `runcam_uart_device.{cpp,hpp}` | RunCam over its UART: `RunCamLink` (rail + byte link, implemented by the port) and the portable session logic behind `CAMDevice` |
| `events.hpp` | `EventEmitter<Listener>` template — used everywhere for async notifications |
| `gpio_buzzer.hpp`, `gpio_led.hpp`, `rgb_led.hpp` | Output device interfaces |
| `kineis_device.hpp` | Common interface implemented by KIM2, SMD and LoRa devices |
| `comms.hpp` | Communication module interface (TX/RX, power) |
| `sensor.{cpp,hpp}` | Generic sensor base class |

## Adding a new hardware abstraction

1. Add a pure-virtual interface header here.
2. Implement it under [`../../ports/nrf52840/core/hardware/`](../../ports/nrf52840/core/hardware/) for the embedded target.
3. Add a mock under [`../../tests/mocks/`](../../tests/mocks/) so host tests can use it.
