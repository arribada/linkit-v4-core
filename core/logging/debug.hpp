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

private:
	static constexpr unsigned int HOLD_MAX_LINES = 24;
	static constexpr unsigned int HOLD_LINE_LEN = 120;
	static inline bool m_held = false;
	static inline unsigned int m_hold_count = 0;
	static inline unsigned int m_hold_dropped = 0;
	static inline char m_hold_level[HOLD_MAX_LINES] = { 0 };
	static inline char m_hold_buf[HOLD_MAX_LINES][HOLD_LINE_LEN] = { { 0 } };
};

#ifdef DEBUG_ENABLE

#if (DEBUG_LEVEL >= 1)
#ifdef DEBUG_TO_SYSTEMLOG
#define DEBUG_ERROR(fmt, ...)                                                              \
	do {                                                                                   \
		if (DebugLogger::console_log) DebugLogger::console_log->error(fmt, ##__VA_ARGS__); \
		if (DebugLogger::system_log && DebugLogger::system_log->is_ready()) {              \
			if (DebugLogger::is_held())                                                    \
				DebugLogger::hold_line('E', fmt, ##__VA_ARGS__);                            \
			else                                                                           \
				DebugLogger::system_log->error(fmt, ##__VA_ARGS__);                             \
		}                                                                                \
	} while (0)
#else
#define DEBUG_ERROR(fmt, ...)                                                              \
	do {                                                                                   \
		if (DebugLogger::console_log) DebugLogger::console_log->error(fmt, ##__VA_ARGS__); \
	} while (0)
#endif
#else
#define DEBUG_ERROR(fmt, ...)
#endif

#if (DEBUG_LEVEL >= 2)
#ifdef DEBUG_TO_SYSTEMLOG
#define DEBUG_WARN(fmt, ...)                                                              \
	do {                                                                                  \
		if (DebugLogger::console_log) DebugLogger::console_log->warn(fmt, ##__VA_ARGS__); \
		if (DebugLogger::system_log && DebugLogger::system_log->is_ready()) {              \
			if (DebugLogger::is_held())                                                    \
				DebugLogger::hold_line('W', fmt, ##__VA_ARGS__);                            \
			else                                                                           \
				DebugLogger::system_log->warn(fmt, ##__VA_ARGS__);                             \
		}                                                                                \
	} while (0)
#else
#define DEBUG_WARN(fmt, ...)                                                              \
	do {                                                                                  \
		if (DebugLogger::console_log) DebugLogger::console_log->warn(fmt, ##__VA_ARGS__); \
	} while (0)
#endif
#else
#define DEBUG_WARN(fmt, ...)
#endif

#if (DEBUG_LEVEL >= 3)
#ifdef DEBUG_TO_SYSTEMLOG
#define DEBUG_INFO(fmt, ...)                                                              \
	do {                                                                                  \
		if (DebugLogger::console_log) DebugLogger::console_log->info(fmt, ##__VA_ARGS__); \
		if (DebugLogger::system_log && DebugLogger::system_log->is_ready()) {              \
			if (DebugLogger::is_held())                                                    \
				DebugLogger::hold_line('I', fmt, ##__VA_ARGS__);                            \
			else                                                                           \
				DebugLogger::system_log->info(fmt, ##__VA_ARGS__);                             \
		}                                                                                \
	} while (0)
#else
#define DEBUG_INFO(fmt, ...)                                                              \
	do {                                                                                  \
		if (DebugLogger::console_log) DebugLogger::console_log->info(fmt, ##__VA_ARGS__); \
	} while (0)
#endif
#else
#define DEBUG_INFO(fmt, ...)
#endif

// NOTE: Do not log TRACE level to system log as it can impact timing of some
// time critical areas of code
#if (DEBUG_LEVEL >= 4)
#define DEBUG_TRACE(fmt, ...)                                                              \
	do {                                                                                   \
		if (DebugLogger::console_log) DebugLogger::console_log->trace(fmt, ##__VA_ARGS__); \
	} while (0)
#else
#define DEBUG_TRACE(fmt, ...)
#endif


#else

#define DEBUG_ERROR(fmt, ...)
#define DEBUG_WARN(fmt, ...)
#define DEBUG_INFO(fmt, ...)
#define DEBUG_TRACE(fmt, ...)

#endif
