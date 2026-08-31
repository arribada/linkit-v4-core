/**
 * @file logger.hpp
 * @brief Abstract logger interface + LoggerManager registry.
 */

#pragma once

#include <map>
#include <cstring>
#include <cstdint>
#include <cstdio>
#include <cstdarg>

#include "messages.hpp"

enum LogLevel { LOG_LEVEL_OFF, LOG_LEVEL_ERROR, LOG_LEVEL_WARN, LOG_LEVEL_INFO, LOG_LEVEL_DEBUG };


class LoggerManager;

/// @brief Formats log entries for output (CSV, text, etc.).
class LogFormatter {
public:
	static const char *log_level_str(LogType t);
	virtual ~LogFormatter() {}
	virtual const std::string header() = 0;
	virtual const std::string log_entry(const LogEntry &e) = 0;

	/// @brief How many entries DUMPD may pack into one packet for this log.
	///
	/// The payload is base64-encoded (x4/3) and must fit BASE_MAX_PAYLOAD_LENGTH
	/// (4095 bytes), and several copies of it are live at once on a 64 KB heap
	/// shared with the BLE stack and every service. A formatter with long lines
	/// must therefore pack fewer entries: the GPS one emits 29 CSV fields out of
	/// a 512-byte buffer, so eight of them can reach 4096 raw bytes -- 5461 once
	/// base64-encoded, i.e. OVER the protocol limit, and several times that in
	/// peak heap. Measured on Cyprus 2026-08-31: dumping a 1437-entry GNSS log
	/// died in vApplicationMallocFailedHook, backtrace pvPortMalloc <-
	/// std::string::append.
	///
	/// Derived, not guessed. EVERY formatter in the tree snprintf()s into a
	/// 512-byte buffer, so the historic batch of 8 was worth up to 4096 raw bytes
	/// for ANY log -- over the safe limit, not just for GPS. Overriding one
	/// formatter would have left the trap everywhere else.
	///
	/// A formatter with genuinely shorter lines says so by overriding
	/// max_line_chars() and gets a bigger batch for free.
	virtual unsigned int max_dump_entries() const {
		/// Raw bytes per packet. base64 inflates by 4/3, so 2048 raw -> 2731
		/// encoded, comfortably inside BASE_MAX_PAYLOAD_LENGTH (4095), and it
		/// halves the peak heap against the batch of 8 that failed.
		constexpr unsigned int DUMP_RAW_BUDGET = 2048;
		constexpr unsigned int DUMP_MAX_ENTRIES = 8;
		const unsigned int n = DUMP_RAW_BUDGET / max_line_chars();
		return n < 1 ? 1 : (n > DUMP_MAX_ENTRIES ? DUMP_MAX_ENTRIES : n);
	}

	/// @brief Upper bound on one formatted line, i.e. this formatter's snprintf
	/// buffer. 512 is what every formatter in the tree currently uses.
	virtual unsigned int max_line_chars() const { return 512; }
};

/// @brief Abstract logger — write log entries, query by level (error/warn/info/trace).
class Logger {
private:
	int m_log_level = LOG_LEVEL_DEBUG;
	unsigned int m_unique_id;
	LogFormatter *m_log_formatter;
	const char *m_name;

	static inline void sync_datetime(LogHeader &header);

public:
	Logger(const char *name);
	virtual ~Logger();

	void set_log_level(int level);
	void set_log_formatter(LogFormatter *formatter);
	LogFormatter *get_log_formatter();
	void warn(const char *msg, ...);
	void error(const char *msg, ...);
	void info(const char *msg, ...);
	void trace(const char *msg, ...);
	void show_info();
	unsigned int get_unique_id();
	const char *get_name();

	virtual void create() = 0;
	virtual void write(void *) = 0;
	virtual void read(void *, int index = 0) = 0;
	virtual unsigned int num_entries() = 0;
	virtual bool is_ready() = 0;
	virtual void truncate() = 0;
};


/// @brief Global logger registry — manages all Logger instances.
class LoggerManager {
private:
	static inline unsigned int m_unique_identifier = 0;
	static inline std::map<unsigned int, Logger &> m_map;

public:
	static unsigned int add(Logger &s);
	static void remove(Logger &s);
	static void create();
	static void truncate();
	static Logger *find_by_name(const char *);
	static void show_info();
};
