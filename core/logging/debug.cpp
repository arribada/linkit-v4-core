/**
 * @file debug.cpp
 * @brief The system-log hold window (see DebugLogger in debug.hpp).
 */

#include "debug.hpp"

#include <cstdarg>
#include <cstdio>

void DebugLogger::hold() {
	m_held = true;
	m_hold_count = 0;
	m_hold_dropped = 0;
}

void DebugLogger::hold_line(char level, const char *fmt, ...) {
	if (m_hold_count >= HOLD_MAX_LINES) {
		m_hold_dropped++;
		return;
	}
	va_list args;
	va_start(args, fmt);
	vsnprintf(m_hold_buf[m_hold_count], HOLD_LINE_LEN, fmt, args);
	va_end(args);
	m_hold_level[m_hold_count] = level;
	m_hold_count++;
}

void DebugLogger::release() {
	// Clear the flag FIRST: replaying calls the very macros that check it, and a
	// still-held flag would push every replayed line straight back into the
	// buffer instead of the file.
	m_held = false;

	if (!system_log || !system_log->is_ready()) {
		m_hold_count = 0;
		m_hold_dropped = 0;
		return;
	}

	for (unsigned int i = 0; i < m_hold_count; i++) {
		// "%s" and not the buffer as a format string: the held text is already
		// expanded and may well contain a stray % from a payload dump.
		switch (m_hold_level[i]) {
		case 'E':
			system_log->error("%s", m_hold_buf[i]);
			break;
		case 'W':
			system_log->warn("%s", m_hold_buf[i]);
			break;
		default:
			system_log->info("%s", m_hold_buf[i]);
			break;
		}
	}

	if (m_hold_dropped) system_log->warn("DebugLogger: %u held lines dropped (hold window too long)", m_hold_dropped);

	m_hold_count = 0;
	m_hold_dropped = 0;
}
