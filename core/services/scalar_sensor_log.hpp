/**
 * @file scalar_sensor_log.hpp
 * @brief CSV formatting shared by the single-channel sensor log formatters.
 *
 * Four sensor services -- thermistor, pH, sea temperature and ALS -- each log one
 * double per sample, and each had its own copy of the same fifteen lines: convert
 * the entry header to epoch, gmtime, strftime into "dd/mm/yyyy HH:MM:SS", then
 * snprintf one value after it. Four copies of one CSV writer, and the copies had
 * already drifted apart (see the buffer note below).
 *
 * A formatter is now three lines: name the column, cast the entry, hand over the
 * header and the value. Everything a reader needs to check about the CSV format
 * is in one place, and a fifth scalar sensor costs three lines instead of fifteen.
 */

#pragma once

#include <cstdio>
#include <ctime>
#include <string>

#include "logger.hpp"
#include "messages.hpp"
#include "timeutils.hpp"

/// @brief One CSV line: the entry's timestamp, then its value.
///
/// The header is taken BY VALUE, not by reference. Every one of these log entries
/// is a `__attribute__((packed))` struct, so its `header` member can be
/// misaligned; binding a reference or a pointer to a packed member is what
/// produces "cannot bind packed field" and, where it compiles, unaligned access
/// on ARM. Copying out of a packed struct is fine -- the compiler emits byte
/// loads -- and LogHeader is small enough that the copy costs nothing worth
/// measuring against a flash write.
///
/// The buffers are the LARGER of the two sizes the four copies had settled on.
/// pH, sea temperature and ALS used 512/128; the thermistor used 128/25, which is
/// a truncation risk rather than a style difference: `%f` on a double can emit
/// over 300 characters for a value like 1e300, so a corrupted entry would have
/// produced a silently truncated CSV line. snprintf never overflows, so this was
/// never a memory fault -- only a line a ground-segment parser could not read.
inline std::string scalar_sensor_csv_line(LogHeader header, double value) {
	char entry[512], date[128];

	std::time_t t =
	    convert_epochtime(header.year, header.month, header.day, header.hours, header.minutes, header.seconds);
	std::tm *tm = std::gmtime(&t);
	std::strftime(date, sizeof(date), "%d/%m/%Y %H:%M:%S", tm);

	snprintf(entry, sizeof(entry), "%s,%f\r\n", date, value);
	return std::string(entry);
}

/// @brief The CSV header line for a single-value sensor: "log_datetime,<column>".
inline std::string scalar_sensor_csv_header(const char *column) {
	return std::string("log_datetime,") + column + "\r\n";
}
