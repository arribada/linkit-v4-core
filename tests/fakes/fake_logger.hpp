#pragma once

#include <cstring>
#include "logger.hpp"
#include "messages.hpp"

class FakeLog : public Logger {
private:
	LogEntry m_entry[2048];
	// Initialised here, not only by create()/truncate(): fixtures that build a log
	// with `new` and never call create() read whatever the heap held -- zero in a
	// fresh process (-p), last test's bytes in a single-process run.
	unsigned int m_index = 0;

public:
	FakeLog(const char *name = "Fake") : Logger(name) {}

	void create() { m_index = 0; }

	void truncate() { m_index = 0; }

	bool is_ready() { return true; }

	void write(void *entry) { std::memcpy(&m_entry[m_index++ % 2048], entry, sizeof(LogEntry)); }

	void read(void *entry, int index = 0) { std::memcpy(entry, &m_entry[index % 2048], sizeof(LogEntry)); }

	unsigned int num_entries() { return m_index; }
};
