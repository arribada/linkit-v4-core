// The firmware builds every DEBUG_* line with its system-log half; the host
// build does not. Turn it on for this file only, before anything pulls in
// debug.hpp. No header included here defines inline code that logs, so the
// macros cannot differ between translation units.
#define DEBUG_TO_SYSTEMLOG

#include "debug.hpp"
#include "error.hpp"
#include "filesystem.hpp"

#include <stdexcept>
#include <string>
#include <vector>

#include "CppUTest/TestHarness.h"

namespace {

/// Records what it is given; can be told to fail the way a broken LittleFS does.
class RecordingLog : public Logger {
public:
	enum class Fail { NONE, LFS_INT, ERROR_CODE, STD_EXCEPTION };
	struct Line {
		LogType type;
		std::string text;
	};

	std::vector<Line> lines;
	bool ready = true;
	Fail fail = Fail::NONE;
	void (*on_write)() = nullptr;

	RecordingLog() : Logger("RecordingLog") {}

	void create() override {}
	void truncate() override {}
	bool is_ready() override { return ready; }
	unsigned int num_entries() override { return lines.size(); }
	void read(void *, int) override {}
	void write(void *p) override {
		if (on_write) on_write();
		switch (fail) {
		case Fail::LFS_INT: throw static_cast<int>(LFS_ERR_CORRUPT);
		case Fail::ERROR_CODE: throw ErrorCode::BAD_FILESYSTEM;
		case Fail::STD_EXCEPTION: throw std::runtime_error("write failed");
		case Fail::NONE:
		default: break;
		}
		const auto *e = static_cast<const LogEntry *>(p);
		lines.push_back({ e->header.log_type, reinterpret_cast<const char *>(e->data) });
	}
};

}  // namespace

TEST_GROUP(DebugLogger) {
	Logger *saved_console;
	Logger *saved_system;
	RecordingLog *console;
	RecordingLog *system;

	void setup() {
		saved_console = DebugLogger::console_log;
		saved_system = DebugLogger::system_log;
		console = new RecordingLog;
		system = new RecordingLog;
		DebugLogger::console_log = console;
		DebugLogger::system_log = system;
		DebugLogger::release();  // no window left open by a previous test
	}

	void teardown() {
		DebugLogger::release();
		DebugLogger::console_log = saved_console;
		DebugLogger::system_log = saved_system;
		delete console;
		delete system;
	}
};

TEST(DebugLogger, LinesReachBothHalvesWithTheirLevel) {
	DEBUG_ERROR("err %d", 1);
	DEBUG_WARN("warn %s", "two");
	DEBUG_INFO("info %u", 3u);

	CHECK_EQUAL(3, system->lines.size());
	CHECK_EQUAL(LOG_ERROR, system->lines[0].type);
	STRCMP_EQUAL("err 1", system->lines[0].text.c_str());
	CHECK_EQUAL(LOG_WARN, system->lines[1].type);
	STRCMP_EQUAL("warn two", system->lines[1].text.c_str());
	CHECK_EQUAL(LOG_INFO, system->lines[2].type);
	STRCMP_EQUAL("info 3", system->lines[2].text.c_str());
	CHECK_EQUAL(3, console->lines.size());
}

TEST(DebugLogger, TraceNeverReachesTheSystemLog) {
	DEBUG_TRACE("trace %d", 1);
	CHECK_EQUAL(0, system->lines.size());
	CHECK_EQUAL(1, console->lines.size());
}

// The system log stores what Logger itself would have stored, truncation included.
TEST(DebugLogger, LongLineIsStoredExactlyAsLoggerWouldStoreIt) {
	const std::string long_text(3 * MAX_LOG_PAYLOAD, 'x');
	RecordingLog direct;
	direct.info("%s", long_text.c_str());

	DEBUG_INFO("%s", long_text.c_str());

	CHECK_EQUAL(1, system->lines.size());
	STRCMP_EQUAL(direct.lines[0].text.c_str(), system->lines[0].text.c_str());
	CHECK_EQUAL(MAX_LOG_PAYLOAD - 1, system->lines[0].text.size());
}

TEST(DebugLogger, NotReadyWritesNothing) {
	system->ready = false;
	const unsigned int skipped = DebugLogger::syslog_skipped();
	const unsigned int failed = DebugLogger::syslog_failed();

	DEBUG_ERROR("lost");

	CHECK_EQUAL(0, system->lines.size());
	CHECK_EQUAL(1, console->lines.size());
	CHECK_EQUAL(skipped, DebugLogger::syslog_skipped());
	CHECK_EQUAL(failed, DebugLogger::syslog_failed());
}

// A failing system log must not turn a log line into an exception: DEBUG_ERROR
// sits in catch handlers and in callbacks unwinding through C frames.
TEST(DebugLogger, ThrowingSystemLogDoesNotPropagate) {
	const RecordingLog::Fail kinds[] = { RecordingLog::Fail::LFS_INT, RecordingLog::Fail::ERROR_CODE,
	                                     RecordingLog::Fail::STD_EXCEPTION };
	for (auto kind : kinds) {
		system->fail = kind;
		const unsigned int failed = DebugLogger::syslog_failed();
		try {
			DEBUG_ERROR("error %d", 1);
			DEBUG_WARN("warn %d", 2);
			DEBUG_INFO("info %d", 3);
		} catch (...) {
			FAIL("a DEBUG_* line threw");
		}
		CHECK_EQUAL(failed + 3, DebugLogger::syslog_failed());
	}
	CHECK_EQUAL(0, system->lines.size());
	CHECK_EQUAL(9, console->lines.size());  // the console half is unaffected

	// And the log works again as soon as the filesystem does.
	system->fail = RecordingLog::Fail::NONE;
	DEBUG_INFO("back");
	CHECK_EQUAL(1, system->lines.size());
}

TEST(DebugLogger, ThrowingSystemLogInsideACatchHandlerDoesNotEscape) {
	system->fail = RecordingLog::Fail::LFS_INT;
	bool reached = false;
	try {
		try {
			throw ErrorCode::RESOURCE_NOT_AVAILABLE;
		} catch (...) {
			DEBUG_ERROR("main loop: unhandled unknown exception");
			reached = true;
		}
	} catch (...) {
		FAIL("the log line escaped the handler");
	}
	CHECK_TRUE(reached);
}

// A line logged while the system log is writing (here from inside its own
// write) must not re-enter it.
static void log_from_inside_write() {
	DEBUG_ERROR("nested");
}

TEST(DebugLogger, NestedLineIsSkippedNotReentered) {
	system->on_write = log_from_inside_write;
	const unsigned int skipped = DebugLogger::syslog_skipped();

	DEBUG_INFO("outer");

	CHECK_EQUAL(1, system->lines.size());
	STRCMP_EQUAL("outer", system->lines[0].text.c_str());
	CHECK_EQUAL(skipped + 1, DebugLogger::syslog_skipped());
	CHECK_EQUAL(2, console->lines.size());  // the nested line still shows on the console
}

// --- Hold window: behaviour unchanged ---------------------------------------

TEST(DebugLogger, HoldBuffersAndReleaseReplaysInOrder) {
	DebugLogger::hold();
	CHECK_TRUE(DebugLogger::is_held());

	DEBUG_INFO("one %d", 1);
	DEBUG_WARN("two");
	DEBUG_ERROR("three");
	CHECK_EQUAL(0, system->lines.size());
	CHECK_EQUAL(3, console->lines.size());  // the console is never held

	DebugLogger::release();
	CHECK_FALSE(DebugLogger::is_held());
	CHECK_EQUAL(3, system->lines.size());
	CHECK_EQUAL(LOG_INFO, system->lines[0].type);
	STRCMP_EQUAL("one 1", system->lines[0].text.c_str());
	CHECK_EQUAL(LOG_WARN, system->lines[1].type);
	STRCMP_EQUAL("two", system->lines[1].text.c_str());
	CHECK_EQUAL(LOG_ERROR, system->lines[2].type);
	STRCMP_EQUAL("three", system->lines[2].text.c_str());

	// Nothing is replayed twice.
	DebugLogger::release();
	CHECK_EQUAL(3, system->lines.size());
}

TEST(DebugLogger, ReleaseReportsLinesDroppedPastCapacity) {
	DebugLogger::hold();
	for (int i = 0; i < 26; i++)
		DEBUG_INFO("line %d", i);
	DebugLogger::release();

	CHECK_EQUAL(25, system->lines.size());
	STRCMP_EQUAL("line 23", system->lines[23].text.c_str());
	CHECK_EQUAL(LOG_WARN, system->lines[24].type);
	STRCMP_EQUAL("DebugLogger: 2 held lines dropped (hold window too long)", system->lines[24].text.c_str());
}

TEST(DebugLogger, HeldPercentIsReplayedLiterally) {
	DebugLogger::hold();
	DEBUG_INFO("payload %s", "100%d %s%n");
	DebugLogger::release();

	CHECK_EQUAL(1, system->lines.size());
	STRCMP_EQUAL("payload 100%d %s%n", system->lines[0].text.c_str());
}

TEST(DebugLogger, HoldAgainRestartsTheWindow) {
	DebugLogger::hold();
	DEBUG_INFO("first");
	DebugLogger::hold();
	DEBUG_INFO("second");
	DebugLogger::release();

	CHECK_EQUAL(1, system->lines.size());
	STRCMP_EQUAL("second", system->lines[0].text.c_str());
}

TEST(DebugLogger, NothingIsHeldWhileTheLogIsNotReady) {
	system->ready = false;
	DebugLogger::hold();
	DEBUG_INFO("not ready");
	system->ready = true;
	DebugLogger::release();

	CHECK_EQUAL(0, system->lines.size());
}

TEST(DebugLogger, ReleaseIntoAFailingLogDoesNotThrow) {
	DebugLogger::hold();
	DEBUG_INFO("a");
	DEBUG_INFO("b");
	system->fail = RecordingLog::Fail::LFS_INT;
	const unsigned int failed = DebugLogger::syslog_failed();
	try {
		DebugLogger::release();
	} catch (...) {
		FAIL("release() threw");
	}
	CHECK_FALSE(DebugLogger::is_held());
	CHECK_EQUAL(failed + 2, DebugLogger::syslog_failed());
}

// --- A flash driver that logs from inside a LittleFS operation --------------

namespace {

/// RAM flash whose program step logs, as Is25Flash does on a QSPI error.
class LoggingRamFlash : public RamFlash {
public:
	LoggingRamFlash() : RamFlash(4096, 64, 256) {}
	int prog(lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size) override {
		DEBUG_ERROR("QSPI IO Error %04x", 0);
		return RamFlash::prog(block, off, buffer, size);
	}
};

/// A system log that appends to a LittleFS file, like FsLog.
class LfsSystemLog : public Logger {
public:
	explicit LfsSystemLog(LFSFileSystem *fs) : Logger("LfsSystemLog"), m_fs(fs) {}
	void create() override {}
	void truncate() override {}
	bool is_ready() override { return true; }
	unsigned int num_entries() override { return 0; }
	void read(void *, int) override {}
	void write(void *entry) override {
		LFSFile f(m_fs, "system.log", LFS_O_WRONLY | LFS_O_CREAT | LFS_O_APPEND);
		f.write(entry, sizeof(LogEntry));
	}

private:
	LFSFileSystem *m_fs;
};

}  // namespace

TEST(DebugLogger, LineFromInsideAFlashOperationDoesNotReenterLittleFs) {
	LoggingRamFlash flash;
	LFSFileSystem fs(&flash);
	CHECK_EQUAL(0, fs.format());
	CHECK_EQUAL(0, fs.mount());
	LfsSystemLog log(&fs);
	DebugLogger::system_log = &log;
	const unsigned int skipped = DebugLogger::syslog_skipped();

	// A write that is not a log one (a config save, say): every program step
	// logs from inside it.
	uint8_t data[1000];
	for (unsigned int i = 0; i < sizeof(data); i++)
		data[i] = static_cast<uint8_t>(i * 7);
	{
		LFSFile f(&fs, "config.dat", LFS_O_WRONLY | LFS_O_CREAT);
		CHECK_EQUAL((lfs_ssize_t)sizeof(data), f.write(data, sizeof(data)));
	}
	CHECK_FALSE(LFSFileSystem::in_flash_io());
	CHECK(DebugLogger::syslog_skipped() > skipped);

	// None of those lines went into LittleFS, and the write is intact.
	struct lfs_info info;
	CHECK(fs.stat("system.log", &info) < 0);
	{
		uint8_t back[sizeof(data)] = { 0 };
		LFSFile f(&fs, "config.dat", LFS_O_RDONLY);
		CHECK_EQUAL((lfs_ssize_t)sizeof(back), f.read(back, sizeof(back)));
		MEMCMP_EQUAL(data, back, sizeof(data));
	}

	// Outside a flash operation the system log is written as usual.
	DEBUG_INFO("after");
	DebugLogger::system_log = system;
	LFSFile f(&fs, "system.log", LFS_O_RDONLY);
	CHECK_EQUAL((lfs_soff_t)sizeof(LogEntry), f.size());
	LogEntry e;
	CHECK_EQUAL((lfs_ssize_t)sizeof(e), f.read(&e, sizeof(e)));
	STRCMP_EQUAL("after", reinterpret_cast<const char *>(e.data));
}
