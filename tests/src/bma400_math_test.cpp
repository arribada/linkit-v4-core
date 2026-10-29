/**
 * @file bma400_math_test.cpp
 * @brief The BMA400's activity metric and calibration average (July audit M1 and
 * M3, ported 2026-10-29), as the driver computes them (bma400_math.hpp).
 *
 * What the driver adds around them -- 0/0/0 on a read with no fresh sample, the
 * journal line per stuck episode, AXL.CAL kept on a refused run -- needs the
 * sensor and is left to the board (CAMPAGNE 2.15).
 */

#include "bma400/bma400_math.hpp"

#include "CppUTest/TestHarness.h"

TEST_GROUP(Bma400Math) {
	// A run of 200 samples at rest, flat: X and Y read 0.02 g, Z reads 1 g.
	const unsigned int asked = 200;
	const double x_at_rest = 0.02;
	const double z_at_rest = 1.0;
};

// Mortality counts an activity below MORTALITY_ACTIVITY_THRESH as a still animal.
// A sensor that reads 0 g on every axis is broken, not still: always the maximum,
// whatever the range. It used to give 17 at 16 g.
TEST(Bma400Math, StuckSensorReadsMaximumActivityAtEveryRange) {
	for (uint8_t g_range : { 2, 4, 8, 16 })
		CHECK_EQUAL(255, bma400_activity(0.0, 0.0, 0.0, g_range));
	CHECK_TRUE(bma400_reads_stuck(0.05, 0.0, 0.05));
	CHECK_FALSE(bma400_reads_stuck(0.0, 0.0, z_at_rest));
}

// The metric itself is unchanged: 0 at rest, the deviation from 1 g scaled to the range.
TEST(Bma400Math, ActivityIsTheDeviationFromOneG) {
	CHECK_EQUAL(0, bma400_activity(0.0, 0.0, 1.0, 2));
	CHECK_EQUAL(255, bma400_activity(0.0, 0.0, 2.0, 2));
	CHECK_EQUAL(85, bma400_activity(0.0, 0.0, 2.0, 4));
	CHECK_EQUAL(255, bma400_activity(0.0, 0.0, 9.0, 4));  // capped at the range
}

// Half the reads timed out: the offsets are the average of the other half, not
// halved. Z halved to 0.5 g made a board at rest read 1.5 g ever after.
TEST(Bma400Math, CalibrationAveragesOnlyTheSamplesRead) {
	const unsigned int read = asked / 2;
	double ox = -1.0, oy = -1.0, oz = -1.0;
	CHECK_TRUE(
	    bma400_calibration_average(read * x_at_rest, read * x_at_rest, read * z_at_rest, read, asked, ox, oy, oz));
	DOUBLES_EQUAL(x_at_rest, ox, 1e-9);
	DOUBLES_EQUAL(x_at_rest, oy, 1e-9);
	DOUBLES_EQUAL(z_at_rest, oz, 1e-9);
}

// Fewer than half the samples read: refused, and the outputs are untouched, so the
// caller keeps the previous calibration (user decision, 2026-10-29).
TEST(Bma400Math, CalibrationWithTooFewSamplesIsRefused) {
	const unsigned int read = asked / 2 - 1;
	double ox = 0.1, oy = 0.2, oz = 0.9;
	CHECK_FALSE(
	    bma400_calibration_average(read * x_at_rest, read * x_at_rest, read * z_at_rest, read, asked, ox, oy, oz));
	DOUBLES_EQUAL(0.1, ox, 1e-12);
	DOUBLES_EQUAL(0.2, oy, 1e-12);
	DOUBLES_EQUAL(0.9, oz, 1e-12);
	CHECK_FALSE(bma400_calibration_average(0.0, 0.0, 0.0, 0, asked, ox, oy, oz));
}
