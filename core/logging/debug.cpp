/**
 * @file debug.cpp
 * @brief The system-log half of DEBUG_* and its hold window (see DebugLogger in debug.hpp).
 */

#include "debug.hpp"
#include "filesystem.hpp"

#include <cstdarg>
#include <cstdio>

#ifndef CPPUTEST
#include "nrf.h"
#endif

static bool in_interrupt() {
#ifdef CPPUTEST
	return false;
#else
	return __get_IPSR() != 0;
#endif
}

void DebugLogger::syslog(char level, const char *fmt, ...) {
	if (!system_log || !system_log->is_ready()) return;

	// Flash is never written from an interrupt, and neither is the hold
	// buffer, which only thread code may touch.
	if (in_interrupt()) {
		m_syslog_skipped++;
		return;
	}

	// Formatted here, at the system log's own payload size, so the text is
	// exactly what Logger would have stored.
	char text[MAX_LOG_PAYLOAD];
	va_list args;
	va_start(args, fmt);
	vsnprintf(text, sizeof(text), fmt, args);
	va_end(args);

	if (m_held)
		hold_line(level, "%s", text);
	else
		write_line(level, text);
}

void DebugLogger::write_line(char level, const char *text) {
	if (m_writing || LFSFileSystem::in_flash_io()) {
		m_syslog_skipped++;
		return;
	}

	m_writing = true;
	try {
		switch (level) {
		case 'E': system_log->error("%s", text); break;
		case 'W': system_log->warn("%s", text); break;
		default: system_log->info("%s", text); break;
		}
	} catch (...) {
		m_syslog_failed++;
	}
	m_writing = false;
}

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
	// The replay writes flash. From an interrupt, leave the window open: the
	// next release() from thread code (Service::service_complete) replays it.
	if (in_interrupt()) return;

	// Clear the flag FIRST: a line logged during the replay must not be
	// appended to the buffer being replayed.
	m_held = false;

	if (!system_log || !system_log->is_ready()) {
		m_hold_count = 0;
		m_hold_dropped = 0;
		return;
	}

	// The held text is already expanded and may well contain a stray % from a
	// payload dump, so it is written as text, never used as a format.
	for (unsigned int i = 0; i < m_hold_count; i++)
		write_line(m_hold_level[i], m_hold_buf[i]);

	if (m_hold_dropped) {
		char text[MAX_LOG_PAYLOAD];
		snprintf(text, sizeof(text), "DebugLogger: %u held lines dropped (hold window too long)", m_hold_dropped);
		write_line('W', text);
	}

	m_hold_count = 0;
	m_hold_dropped = 0;
}
