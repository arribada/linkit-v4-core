# core/logging/

Logging primitives.

| File | Role |
|------|------|
| `debug.hpp` | `DEBUG_TRACE`, `DEBUG_INFO`, `DEBUG_WARN`, `DEBUG_ERROR` macros (controlled by `DEBUG_LEVEL`) |
| `console_log.hpp` | Console (UART/USB) log sink |
| `fs_log.hpp` | File-backed log sink |
| `sys_log.hpp` | System-event ring buffer |
| `logger.{cpp,hpp}` | Generic logger wrapper |
| `messages.hpp` | Log message type definitions |

## Conventions

- Use `DEBUG_TRACE` for high-volume per-tick info (off by default).
- Use `DEBUG_INFO` for one-shot state transitions and notable events.
- Use `DEBUG_WARN` for recoverable anomalies.
- Use `DEBUG_ERROR` for failures that affect the user-visible behavior.

Verbosity is set at build time via `-DDEBUG_LEVEL=N` (0=off, 4=trace).

ERROR, WARN and INFO also go to `system.log` on LittleFS — whichever of them
`DEBUG_LEVEL` leaves compiled in, since the flash half lives inside the same
macro. That half never throws, and skips the line from interrupt context or
while LittleFS is already busy; the console half always prints it.

`-DDEBUG_NO_SYSTEMLOG=ON` drops the flash half at every level while keeping the
console, which is how a latency-critical path is measured without paying for its
own logs. On a path worth keeping the lines for, `DebugLogger::hold()` buffers
them in RAM and `release()` replays them afterwards instead.
