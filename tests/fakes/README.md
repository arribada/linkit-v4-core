# tests/fakes/

Stub implementations used to make embedded code link and run on the host. Unlike mocks ([`../mocks/`](../mocks/)), fakes do not assert on calls — they just provide a working-enough implementation so the code under test can execute.

## Categories

| File(s) | Purpose |
|---------|---------|
| `nrf_gpio.h`, `nrf_uarte.h`, `nrf_power.h`, `nrf_libuarte_async.h`, `nrfx_*.h` | Stubs of Nordic SDK headers used by drivers we compile into the host binary |
| `nrf_libuarte_async.cpp`, `nrf_uarte.c`, `nrfx_rtc.c` | Function-body stubs for the SDK calls |
| `nrf_err.h`, `sdk_errors.h` | SDK error code constants |
| `nrf_peripheral_power.hpp` | Stub for the SDK peripheral-power workaround |
| `bsp.hpp`, `fake_bsp.cpp` | Fake board-support package — pin numbers, UART configs |
| `fake_*.hpp` | Fake implementations of `core/hardware/` interfaces (battery, switch, RTC, timer, LED, RGB LED, logger, config store, IRQ, memory access, SWS, reed) |
| `memmang.cpp` | Heap accounting stub |

## When the host build breaks after touching firmware code

Most often the new code uses an SDK symbol that no fake provides. Either:
- **Add a stub** here (preferred — keeps the test build green for everyone).
- **Exclude the file from the host build** if it's deeply hardware-dependent (edit [`../CMakeLists.txt`](../CMakeLists.txt) `target_sources`).

The CI host build will catch the breakage.
