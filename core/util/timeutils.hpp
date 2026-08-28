/**
 * @file timeutils.hpp
 * @brief Time conversion utilities — epoch ↔ datetime, day-of-year, portable timegm.
 */

#pragma once

#include <ctime>
#include <cstdint>
#include <cstring>

// Portable timegm implementation for embedded systems (newlib doesn't provide timegm)
// Converts struct tm (interpreted as UTC) to time_t
static std::time_t portable_timegm(struct tm *tm) {
	// Days in each month (non-leap year)
	static const int days_in_month[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

	int year = tm->tm_year + 1900;
	int is_leap = ((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0);

	// Count days from 1970 to the start of this year
	std::time_t days = 0;
	for (int y = 1970; y < year; y++) {
		days += (((y % 4 == 0) && (y % 100 != 0)) || (y % 400 == 0)) ? 366 : 365;
	}

	// Add days for completed months in the current year
	for (int m = 0; m < tm->tm_mon; m++) {
		days += days_in_month[m];
		if (m == 1 && is_leap) days++;
	}

	// Add days in current month (tm_mday is 1-based)
	days += tm->tm_mday - 1;

	return days * 86400 + tm->tm_hour * 3600 + tm->tm_min * 60 + tm->tm_sec;
}

// Alias for compatibility
#ifndef timegm
#define timegm portable_timegm
#endif

static std::time_t convert_epochtime(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min,
                                     uint8_t sec) {
	struct tm t;
	memset(&t, 0, sizeof(t));
	t.tm_sec = sec;
	t.tm_min = min;
	t.tm_hour = hour;
	t.tm_mday = day;
	t.tm_mon = month - 1;
	t.tm_year = year - 1900;
	std::time_t et = portable_timegm(&t);

	return et;
}

static void convert_day_of_year(const uint16_t year, const uint16_t day_of_year, uint8_t &month, uint16_t &day) {
	std::time_t t = convert_epochtime(year, 1, 1, 0, 0, 0);  // Epoch time at start of year i.e., 1st January
	t += (day_of_year - 1) * 24
	     * 3600;                      // Add day of year in seconds, note: we subtract 1 because day of year is 1...365
	struct tm *tm = std::gmtime(&t);  // Convert back to struct tm

	// Return back to caller the month and day of month
	month = tm->tm_mon + 1;
	day = tm->tm_mday;
}

static void convert_datetime_to_epoch(std::time_t time, uint16_t &year, uint8_t &month, uint8_t &day, uint8_t &hour,
                                      uint8_t &min, uint8_t &sec) {
	std::tm *date_time = std::gmtime(&time);

	if (date_time == nullptr) {
		day = 1;
		month = 1;
		year = 1970;
		hour = 0;
		min = 0;
		sec = 0;
		return;
	}

	day = date_time->tm_mday;
	month = date_time->tm_mon + 1;
	year = date_time->tm_year + 1900;
	hour = date_time->tm_hour;
	min = date_time->tm_min;
	sec = date_time->tm_sec;
}
