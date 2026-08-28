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
