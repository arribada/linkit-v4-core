/**
 * @file scalar_sensor_log_test.cpp
 * @brief CSV output of the four single-channel sensor log formatters.
 *
 * Nothing asserted the CSV these four produce -- checked before writing this:
 * no test in tests/src referenced their formatters or their column names. That
 * output is the ground segment's contract, so the formatters had drifted apart
 * unobserved (the thermistor had a 128-byte line buffer where the other three
 * had 512) and a reformat could have changed a column name or a separator with
 * nothing to notice.
 *
 * These tests pin the exact bytes: column names, the dd/mm/yyyy HH:MM:SS date,
 * the comma, the %f rendering and the CRLF.
 */

#include "als_sensor_service.hpp"
#include "ph_sensor_service.hpp"
#include "sea_temp_sensor_service.hpp"
#include "thermistor_sensor_service.hpp"

#include "CppUTest/TestHarness.h"

TEST_GROUP(ScalarSensorLog){};

namespace {
/// 2026-10-13 14:05:09, a date whose day and month cannot be confused.
LogHeader fixed_header() {
	LogHeader h = {};
	h.day = 13;
	h.month = 10;
	h.year = 2026;
	h.hours = 14;
	h.minutes = 5;
	h.seconds = 9;
	return h;
}
}  // namespace

TEST(ScalarSensorLog, headers_name_their_column) {
	ThermistorLogFormatter thermistor;
	PHLogFormatter ph;
	SeaTempLogFormatter sea_temp;
	ALSLogFormatter als;

	STRCMP_EQUAL("log_datetime,temp\r\n", thermistor.header().c_str());
	STRCMP_EQUAL("log_datetime,pH\r\n", ph.header().c_str());
	STRCMP_EQUAL("log_datetime,sea_temp\r\n", sea_temp.header().c_str());
	STRCMP_EQUAL("log_datetime,lumens\r\n", als.header().c_str());
}

TEST(ScalarSensorLog, thermistor_entry_is_date_then_value) {
	ThermistorLogEntry e = {};
	e.header = fixed_header();
	e.temp = 21.5;

	ThermistorLogFormatter f;
	STRCMP_EQUAL("13/10/2026 14:05:09,21.500000\r\n", f.log_entry(*reinterpret_cast<const LogEntry *>(&e)).c_str());
}

TEST(ScalarSensorLog, ph_entry_is_date_then_value) {
	PHLogEntry e = {};
	e.header = fixed_header();
	e.ph = 8.25;

	PHLogFormatter f;
	STRCMP_EQUAL("13/10/2026 14:05:09,8.250000\r\n", f.log_entry(*reinterpret_cast<const LogEntry *>(&e)).c_str());
}

TEST(ScalarSensorLog, sea_temp_entry_is_date_then_value) {
	SeaTempLogEntry e = {};
	e.header = fixed_header();
	e.sea_temp = 17.125;

	SeaTempLogFormatter f;
	STRCMP_EQUAL("13/10/2026 14:05:09,17.125000\r\n", f.log_entry(*reinterpret_cast<const LogEntry *>(&e)).c_str());
}

TEST(ScalarSensorLog, als_entry_is_date_then_value) {
	ALSLogEntry e = {};
	e.header = fixed_header();
	e.lumens = 1234.5;

	ALSLogFormatter f;
	STRCMP_EQUAL("13/10/2026 14:05:09,1234.500000\r\n", f.log_entry(*reinterpret_cast<const LogEntry *>(&e)).c_str());
}

/// The reason the shared buffer is the larger of the two sizes the copies had.
/// `%f` on a value like this emits over 300 characters; at the thermistor's old
/// 128 bytes the line came out truncated, with no CRLF for a parser to find.
/// snprintf never overflowed, so this was never a fault -- just an unreadable
/// line out of a tag you only get back once.
TEST(ScalarSensorLog, a_pathological_value_is_not_truncated) {
	ThermistorLogEntry e = {};
	e.header = fixed_header();
	e.temp = 1e300;

	ThermistorLogFormatter f;
	std::string line = f.log_entry(*reinterpret_cast<const LogEntry *>(&e));
	CHECK(line.size() > 300);
	// A complete line still ends with CRLF, which is what a parser looks for.
	STRCMP_EQUAL("\r\n", line.substr(line.size() - 2).c_str());
}
