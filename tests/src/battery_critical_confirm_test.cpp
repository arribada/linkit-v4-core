#include "nrf_battery_mon.hpp"

#include "CppUTest/TestHarness.h"

// CriticalBandConfirm decides when a low battery is believed enough to power a
// sealed tag off for good. NrfBatteryMonitor itself is not host-compiled (it
// needs the Nordic SDK), so the policy is pinned here on its own.

namespace {
constexpr uint64_t SEC = 1000ULL;
constexpr uint64_t MIN = 60ULL * SEC;
constexpr uint64_t SPACING = CriticalBandConfirm::MIN_SPACING_MS;
}  // namespace

TEST_GROUP(BatteryCriticalConfirm){};

// The defect this guards against: one TX burst takes a reading before every
// transmission, so three low readings arrive within seconds.
TEST(BatteryCriticalConfirm, OneBurstNeverConfirms) {
	CriticalBandConfirm confirm;
	uint8_t count = 0;
	const uint64_t t0 = 10 * MIN;
	for (uint64_t i = 0; i < 50; i++) {
		CHECK_FALSE(confirm.update(count, true, t0 + i * 2 * SEC));
	}
	CHECK_EQUAL(1, count);
}

TEST(BatteryCriticalConfirm, ThreeSpacedSamplesConfirmAfterAnHour) {
	CriticalBandConfirm confirm;
	uint8_t count = 0;
	const uint64_t t0 = 5 * MIN;
	CHECK_FALSE(confirm.update(count, true, t0));
	CHECK_FALSE(confirm.update(count, true, t0 + SPACING));
	CHECK_EQUAL(2, count);
	CHECK_TRUE(confirm.update(count, true, t0 + 2 * SPACING));
	CHECK_EQUAL(CriticalBandConfirm::SAMPLES, count);
	CHECK_TRUE(2 * SPACING >= 60 * MIN);
}

TEST(BatteryCriticalConfirm, SpacingIsMeasuredFromTheLastCountedSample) {
	CriticalBandConfirm confirm;
	uint8_t count = 0;
	CHECK_FALSE(confirm.update(count, true, 0));
	// Low samples in between neither count nor move the anchor.
	CHECK_FALSE(confirm.update(count, true, 10 * MIN));
	CHECK_FALSE(confirm.update(count, true, 20 * MIN));
	CHECK_FALSE(confirm.update(count, true, SPACING - 1));
	CHECK_EQUAL(1, count);
	CHECK_FALSE(confirm.update(count, true, SPACING));
	CHECK_EQUAL(2, count);
	CHECK_FALSE(confirm.update(count, true, 2 * SPACING - 1));
	CHECK_TRUE(confirm.update(count, true, 2 * SPACING));
}

TEST(BatteryCriticalConfirm, AHealthySampleRestartsTheRun) {
	CriticalBandConfirm confirm;
	uint8_t count = 0;
	CHECK_FALSE(confirm.update(count, true, 0));
	CHECK_FALSE(confirm.update(count, true, SPACING));
	CHECK_EQUAL(2, count);

	// A reading at rest between two bursts: the pack recovered.
	CHECK_FALSE(confirm.update(count, false, SPACING + 5 * MIN));
	CHECK_EQUAL(0, count);

	const uint64_t t1 = SPACING + 10 * MIN;
	CHECK_FALSE(confirm.update(count, true, t1));
	CHECK_EQUAL(1, count);
	CHECK_FALSE(confirm.update(count, true, t1 + SPACING));
	CHECK_TRUE(confirm.update(count, true, t1 + 2 * SPACING));
}

TEST(BatteryCriticalConfirm, StaysConfirmedWhileLowAndClearsOnRecovery) {
	CriticalBandConfirm confirm;
	uint8_t count = 0;
	CHECK_FALSE(confirm.update(count, true, 0));
	CHECK_FALSE(confirm.update(count, true, SPACING));
	CHECK_TRUE(confirm.update(count, true, 2 * SPACING));
	CHECK_TRUE(confirm.update(count, true, 2 * SPACING + 1 * SEC));
	CHECK_TRUE(confirm.update(count, true, 5 * SPACING));
	CHECK_EQUAL(CriticalBandConfirm::SAMPLES, count);

	CHECK_FALSE(confirm.update(count, false, 5 * SPACING + 1 * SEC));
	CHECK_EQUAL(0, count);
	CHECK_FALSE(confirm.update(count, true, 5 * SPACING + 2 * SEC));
}

// The count survives a soft reset (.noinit), the uptime does not: the first
// low sample of a boot only re-anchors.
TEST(BatteryCriticalConfirm, RunCarriedOverAResetNeedsAnotherSpacedSample) {
	uint8_t count = 2;  // as left in .noinit by the previous boot
	CriticalBandConfirm confirm;
	const uint64_t boot = 3 * SEC;
	CHECK_FALSE(confirm.update(count, true, boot));
	CHECK_EQUAL(2, count);
	CHECK_FALSE(confirm.update(count, true, boot + 5 * SEC));
	CHECK_FALSE(confirm.update(count, true, boot + SPACING - 1));
	CHECK_TRUE(confirm.update(count, true, boot + SPACING));
}

TEST(BatteryCriticalConfirm, ConfirmedBeforeAResetIsNotConfirmedOnBoot) {
	uint8_t count = CriticalBandConfirm::SAMPLES;
	CriticalBandConfirm confirm;
	CHECK_FALSE(confirm.update(count, true, 2 * SEC));
	CHECK_EQUAL(CriticalBandConfirm::SAMPLES - 1, count);
	CHECK_FALSE(confirm.update(count, true, 4 * SEC));
	CHECK_TRUE(confirm.update(count, true, 2 * SEC + SPACING));
}

TEST(BatteryCriticalConfirm, OutOfRangeCarriedCountIsCapped) {
	uint8_t count = 200;
	CriticalBandConfirm confirm;
	CHECK_FALSE(confirm.update(count, true, 0));
	CHECK_EQUAL(CriticalBandConfirm::SAMPLES - 1, count);
}

TEST(BatteryCriticalConfirm, RebootLoopNeverConfirms) {
	uint8_t count = 0;
	for (int boot = 0; boot < 100; boot++) {
		CriticalBandConfirm confirm;  // RAM state is lost on every reset
		for (uint64_t t = 1 * SEC; t < SPACING; t += 5 * MIN) {
			CHECK_FALSE(confirm.update(count, true, t));
		}
	}
	CHECK_EQUAL(1, count);
}

TEST(BatteryCriticalConfirm, ClockGoingBackwardsDoesNotCount) {
	CriticalBandConfirm confirm;
	uint8_t count = 0;
	CHECK_FALSE(confirm.update(count, true, 10 * SPACING));
	CHECK_FALSE(confirm.update(count, true, 1 * SEC));
	CHECK_EQUAL(1, count);
}
