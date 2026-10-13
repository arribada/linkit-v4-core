/**
 * @file debug.hpp
 * @brief Debug logging macros — level-filtered console + optional system log output.
 */

#pragma once

#include "logger.hpp"

/// @brief Global debug logger singletons (set in main.cpp).
/// Also owns the "hold" window. Every DEBUG_* line commits to system_log on
/// LittleFS, and on a latency-critical path that is not free: measured on the
/// bench 2026-09-03, SURFACING_BURST over SMD UART, surface event to AT+TX
/// accepted, the flash half of the logs cost 0.6 to 0.9 s of a 1.7 s path.
/// DEBUG_TRACE already refuses the system log for exactly this reason (see the
/// note above its macro); this is the same hazard at INFO/WARN/ERROR.
///
/// Dropping the lines would blind the moment most worth diagnosing, so hold()
/// buffers them in RAM and release() replays them once the transmission is
/// away. Console output is untouched throughout.
class DebugLogger {
public:
	static inline Logger *console_log = nullptr;
	static inline Logger *system_log = nullptr;

	static void hold();
	static void release();
	static bool is_held() { return m_held; }
	/// @brief Buffer one line. Drops past capacity and counts the drops, which
	/// release() reports -- a full buffer means the window was left open.
	static void hold_line(char level, const char *fmt, ...);

	/// @brief System-log half of DEBUG_ERROR/WARN/INFO (level 'E', 'W', 'I').
	///
	/// Never throws: a log call sits in catch handlers and in callbacks that
	/// unwind through C frames, where an escaping exception is std::terminate.
	/// A line is skipped from interrupt context and while LittleFS is already
	/// working (a nested log, a flash driver error), as LittleFS is not
	/// re-entrant; the console half still shows it.
	static void syslog(char level, const char *fmt, ...);
	/// @brief Lines not written because the context forbade it.
	static unsigned int syslog_skipped() { return m_syslog_skipped; }
	/// @brief Lines whose write failed (the exception was swallowed).
	static unsigned int syslog_failed() { return m_syslog_failed; }

private:
	static constexpr unsigned int HOLD_MAX_LINES = 24;
	static constexpr unsigned int HOLD_LINE_LEN = 120;
	static inline bool m_held = false;
	static inline unsigned int m_hold_count = 0;
	static inline unsigned int m_hold_dropped = 0;
	static inline char m_hold_level[HOLD_MAX_LINES] = { 0 };
	static inline char m_hold_buf[HOLD_MAX_LINES][HOLD_LINE_LEN] = { { 0 } };

	static inline bool m_writing = false;
	static inline unsigned int m_syslog_skipped = 0;
	static inline unsigned int m_syslog_failed = 0;

	static void write_line(char level, const char *text);
};

/// @brief What a compiled-out DEBUG_* expands to.
///
/// NOT empty. An empty expansion breaks the build at any reduced DEBUG_LEVEL,
/// because the arguments are usually computed for the trace and nothing else:
/// -Werror=unused-variable (and -Werror=unused-but-set-variable) fire on them,
/// and `if (cond) DEBUG_INFO(...);` collapses to `if (cond) ;` which trips
/// -Werror=empty-body. Measured on this tree: -DDEBUG_LEVEL=2 failed with 10
/// such errors across gps_service, service_cooldown, argos_tx_service and
/// kim2 -- i.e. the configuration a deployment would ask for did not compile,
/// and would only have been discovered at deployment time.
///
/// `if (false)` keeps the arguments type-checked and marks them used, while
/// generating no code and never evaluating them, so a compiled-out level still
/// costs nothing at runtime. The do/while gives the macro a statement body so
/// it can follow a bare `if` safely.
/// Arguments are taken BY VALUE, not by reference: several call sites pass
/// fields of packed structs (GPSLogEntry::info.lat and friends), and a
/// reference cannot bind to a packed field -- by-reference parameters turned
/// the unused-variable errors into "cannot bind packed field to double&".
/// Nothing is actually copied: the call is never evaluated.
namespace debug_detail {
template <typename... Args> inline void swallow(Args...) {}
}  // namespace debug_detail
#define DEBUG_NOOP(fmt, ...)                                    \
	do {                                                        \
		if (false) ::debug_detail::swallow(fmt, ##__VA_ARGS__); \
	} while (0)

#ifdef DEBUG_ENABLE

// System-log half of each DEBUG_* line. Severity filtering is DEBUG_LEVEL's
// job alone: each DEBUG_* below is already compiled out under its own level, so
// the flash half disappears with it and needs no second threshold. The only
// question left here is whether the flash sink exists at all, which is
// -DDEBUG_NO_SYSTEMLOG=ON (it drops this half at every level while keeping the
// console, so a latency path can be measured without flash commits).
#ifdef DEBUG_TO_SYSTEMLOG
#define DEBUG_SYSLOG(level, fmt, ...) DebugLogger::syslog(level, fmt, ##__VA_ARGS__)
#else
#define DEBUG_SYSLOG(level, fmt, ...) ((void)0)
#endif

#if (DEBUG_LEVEL >= 1)
#define DEBUG_ERROR(fmt, ...)                                                              \
	do {                                                                                   \
		if (DebugLogger::console_log) DebugLogger::console_log->error(fmt, ##__VA_ARGS__); \
		DEBUG_SYSLOG('E', fmt, ##__VA_ARGS__);                                             \
	} while (0)
#else
#define DEBUG_ERROR(fmt, ...) DEBUG_NOOP(fmt, ##__VA_ARGS__)
#endif

#if (DEBUG_LEVEL >= 2)
#define DEBUG_WARN(fmt, ...)                                                              \
	do {                                                                                  \
		if (DebugLogger::console_log) DebugLogger::console_log->warn(fmt, ##__VA_ARGS__); \
		DEBUG_SYSLOG('W', fmt, ##__VA_ARGS__);                                            \
	} while (0)
#else
#define DEBUG_WARN(fmt, ...) DEBUG_NOOP(fmt, ##__VA_ARGS__)
#endif

#if (DEBUG_LEVEL >= 3)
#define DEBUG_INFO(fmt, ...)                                                              \
	do {                                                                                  \
		if (DebugLogger::console_log) DebugLogger::console_log->info(fmt, ##__VA_ARGS__); \
		DEBUG_SYSLOG('I', fmt, ##__VA_ARGS__);                                            \
	} while (0)
#else
#define DEBUG_INFO(fmt, ...) DEBUG_NOOP(fmt, ##__VA_ARGS__)
#endif

// NOTE: Do not log TRACE level to system log as it can impact timing of some
// time critical areas of code
#if (DEBUG_LEVEL >= 4)
#define DEBUG_TRACE(fmt, ...)                                                              \
	do {                                                                                   \
		if (DebugLogger::console_log) DebugLogger::console_log->trace(fmt, ##__VA_ARGS__); \
	} while (0)
#else
#define DEBUG_TRACE(fmt, ...) DEBUG_NOOP(fmt, ##__VA_ARGS__)
#endif


#else

#define DEBUG_ERROR(fmt, ...) DEBUG_NOOP(fmt, ##__VA_ARGS__)
#define DEBUG_WARN(fmt, ...)  DEBUG_NOOP(fmt, ##__VA_ARGS__)
#define DEBUG_INFO(fmt, ...)  DEBUG_NOOP(fmt, ##__VA_ARGS__)
#define DEBUG_TRACE(fmt, ...) DEBUG_NOOP(fmt, ##__VA_ARGS__)

#endif
