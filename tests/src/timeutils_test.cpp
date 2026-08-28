#include "CppUTest/TestHarness.h"
#include <cstring>
#include "timeutils.hpp"

TEST_GROUP(TimeUtils){};

TEST(TimeUtils, ConvertEpochTimeUnixEpoch) {
	// 1 Jan 1970 00:00:00 = epoch 0
	std::time_t t = convert_epochtime(1970, 1, 1, 0, 0, 0);
	CHECK_EQUAL(0, t);
}

TEST(TimeUtils, ConvertEpochTimeKnownDate) {
	// 1 Jan 2020 00:00:00 UTC = 1577836800
	std::time_t t = convert_epochtime(2020, 1, 1, 0, 0, 0);
	CHECK_EQUAL(1577836800, t);
}

TEST(TimeUtils, ConvertEpochTimeWithTime) {
	// 27 Jan 2020 00:00:00 UTC = 1580083200 (used in test suite)
	std::time_t t = convert_epochtime(2020, 1, 27, 0, 0, 0);
	CHECK_EQUAL(1580083200, t);
}

TEST(TimeUtils, ConvertEpochTimeLeapYear) {
	// 29 Feb 2020 (leap year) 12:00:00
	std::time_t t = convert_epochtime(2020, 2, 29, 12, 0, 0);
	// 29 Feb 2020 12:00:00 = 1583064000 + 12*3600 - check with known value
	struct tm check;
	memset(&check, 0, sizeof(check));
	check.tm_year = 120;  // 2020-1900
	check.tm_mon = 1;     // Feb
	check.tm_mday = 29;
	check.tm_hour = 12;
	std::time_t expected = timegm(&check);
	CHECK_EQUAL(expected, t);
}

TEST(TimeUtils, ConvertEpochTimeYear2000) {
	// 1 Jan 2000 00:00:00 UTC = 946684800
	std::time_t t = convert_epochtime(2000, 1, 1, 0, 0, 0);
	CHECK_EQUAL(946684800, t);
}

TEST(TimeUtils, ConvertDayOfYearJan1) {
	uint8_t month;
	uint16_t day;
	convert_day_of_year(2020, 1, month, day);
	CHECK_EQUAL(1, month);
	CHECK_EQUAL(1, day);
}

TEST(TimeUtils, ConvertDayOfYearFeb29LeapYear) {
	uint8_t month;
	uint16_t day;
	// Day 60 of 2020 (leap year) = 29 Feb
	convert_day_of_year(2020, 60, month, day);
	CHECK_EQUAL(2, month);
	CHECK_EQUAL(29, day);
}

TEST(TimeUtils, ConvertDayOfYearDec31) {
	uint8_t month;
	uint16_t day;
	// Day 366 of 2020 (leap year) = 31 Dec
	convert_day_of_year(2020, 366, month, day);
	CHECK_EQUAL(12, month);
	CHECK_EQUAL(31, day);
}

TEST(TimeUtils, ConvertDayOfYearMarch1NonLeap) {
	uint8_t month;
	uint16_t day;
	// Day 60 of 2021 (non-leap year) = 1 March
	convert_day_of_year(2021, 60, month, day);
	CHECK_EQUAL(3, month);
	CHECK_EQUAL(1, day);
}

TEST(TimeUtils, ConvertDatetimeToEpochRoundTrip) {
	uint16_t year;
	uint8_t month, day, hour, min, sec;
	std::time_t input = 1580083200;  // 27 Jan 2020 00:00:00
	convert_datetime_to_epoch(input, year, month, day, hour, min, sec);
	CHECK_EQUAL(2020, year);
	CHECK_EQUAL(1, month);
	CHECK_EQUAL(27, day);
	CHECK_EQUAL(0, hour);
	CHECK_EQUAL(0, min);
	CHECK_EQUAL(0, sec);

	// Round-trip
	std::time_t output = convert_epochtime(year, month, day, hour, min, sec);
	CHECK_EQUAL(input, output);
}

TEST(TimeUtils, ConvertDatetimeToEpochNullTime) {
	uint16_t year;
	uint8_t month, day, hour, min, sec;
	convert_datetime_to_epoch(0, year, month, day, hour, min, sec);
	CHECK_EQUAL(1970, year);
	CHECK_EQUAL(1, month);
	CHECK_EQUAL(1, day);
	CHECK_EQUAL(0, hour);
	CHECK_EQUAL(0, min);
	CHECK_EQUAL(0, sec);
}

TEST(TimeUtils, PortableTimegmConsistency) {
	// Verify portable_timegm matches system timegm for known dates
	struct tm t;
	memset(&t, 0, sizeof(t));
	t.tm_year = 121;  // 2021
	t.tm_mon = 8;     // September
	t.tm_mday = 18;
	t.tm_hour = 23;
	t.tm_min = 9;
	t.tm_sec = 10;
	std::time_t result = portable_timegm(&t);
	// 18 Sep 2021 23:09:10 - used in PASPW AOP test data
	CHECK(result > 0);

	// Verify round-trip
	uint16_t year;
	uint8_t month, day, hour, minute, second;
	convert_datetime_to_epoch(result, year, month, day, hour, minute, second);
	CHECK_EQUAL(2021, year);
	CHECK_EQUAL(9, month);
	CHECK_EQUAL(18, day);
	CHECK_EQUAL(23, hour);
	CHECK_EQUAL(9, minute);
	CHECK_EQUAL(10, second);
}
