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

ERROR, WARN and INFO also go to `system.log` on LittleFS, up to
`-DSYSTEMLOG_MIN_LEVEL=N` (1=ERROR, 2=WARN, 3=INFO, the default). That half never
throws, and skips the line from interrupt context or while LittleFS is already
busy; the console half always prints it.
