/**
 * @file rspb_gauge_test.cpp
 * @brief The STC3117 gauge of the RSPB: the critical verdict drawn from its cell
 * curve, and an error path of ST's C driver (July audit H3 and #7, ported
 * 2026-10-29). Compiled into RSPBSimulation, the build with the RSPB sources.
 *
 * The C++ half of the driver (stc3117_gasgauge.cpp) needs the nRF I2C and the
 * sensor power rail, which no host build has: what it adds around these two is
 * left to the board (CAMPAGNE 2.15).
 */

#include <cstdint>

#include "stc3117/icp402050_ocv.hpp"
extern "C" {
#include "stc3117/Generic_I2C.h"
#include "stc3117/stc311x_gasgauge.h"
}

#include "CppUTest/TestHarness.h"

namespace {
// The I2C bus as the C driver sees it: it answers with zeros, or not at all.
bool s_bus_answers;

int fake_read(int, int, unsigned char *rx, int count) {
	if (!s_bus_answers) return -1;
	for (int i = 0; i < count; i++)
		rx[i] = 0;
	return 0;
}

int fake_write(int, int, unsigned char *, int) {
	return s_bus_answers ? 0 : -1;
}
}  // namespace

TEST_GROUP(RSPBGauge) {
	const uint8_t critical = 5;  // LB_CRITICAL_THRESH (LBP12), its default

	void setup() {
		s_bus_answers = true;
		RegisterI2C_Read(fake_read);
		RegisterI2C_Write(fake_write);
	}

	void teardown() {
		RegisterI2C_Read(nullptr);
		RegisterI2C_Write(nullptr);
	}
};

// A low SOC alone used to power the wake down. A gauge reporting one on a healthy
// cell at every wake made the tag mute; the voltage now has to agree.
TEST(RSPBGauge, CriticalNeedsTheVoltageToAgreeWithTheSoc) {
	// An empty cell: both read low.
	CHECK_TRUE(soc_and_voltage_critical(2, 3350, critical));
	// The gauge reads 2 % on a cell at 3.9 V, 73 % on the curve: vetoed.
	CHECK_FALSE(soc_and_voltage_critical(2, 3900, critical));
	// A SOC above the level is never critical, whatever the voltage under load.
	CHECK_FALSE(soc_and_voltage_critical(40, 3350, critical));
	// LB_CRITICAL_THRESH at 0: nothing is ever critical, as before.
	CHECK_FALSE(soc_and_voltage_critical(0, 3000, 0));
}

// "The voltage is below the critical level" only means something if a lower
// voltage never reads a higher charge. A refined curve has to keep that.
TEST(RSPBGauge, OcvCurveNeverReadsMoreChargeAtLowerVoltage) {
	for (uint16_t mv = 3000; mv < 4300; mv++)
		CHECK_TRUE(estimate_soc_from_voltage(mv) <= estimate_soc_from_voltage(mv + 1));
	CHECK_EQUAL(0, estimate_soc_from_voltage(ICP402050_OCV_MV[0]));
	CHECK_EQUAL(100, estimate_soc_from_voltage(ICP402050_OCV_MV[ICP402050_OCV_POINTS - 1]));
}

// The counter read on a dead bus came back as 65535, past the ready mark, so the
// start believed the gauge was running and never took its communication-error
// branch -- the one that stops the gauge instead of leaving it at ~70 uA.
TEST(RSPBGauge, RunningCounterReportsABusErrorAsNegative) {
	s_bus_answers = false;
	CHECK_TRUE(STC31xx_GetRunningCounter() < 0);

	s_bus_answers = true;
	CHECK_EQUAL(0, STC31xx_GetRunningCounter());
}
