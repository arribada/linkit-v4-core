#include "CppUTest/TestHarness.h"

#include "fake_config_store.hpp"
#include "fake_rtc.hpp"
#include "moored_mode_service.hpp"

extern ConfigurationStore *configuration_store;
extern RTC *rtc;

/// Metres -> degrees of latitude, using the same spherical earth radius as
/// haversine.cpp (R = 6371 km). Latitude-only offsets keep the tests free of
/// any cos(lat) longitude scaling.
static double lat_offset_m(double metres) {
	return metres / 111194.926644559;
}

static constexpr double BASE_LAT = 34.7500;  // Limassol-ish
static constexpr double BASE_LON = 33.0300;

TEST_GROUP(MooredMode) {
	FakeConfigurationStore *fake_config_store;
	FakeRTC *fake_rtc;

	void setup() {
		fake_config_store = new FakeConfigurationStore;
		configuration_store = fake_config_store;
		configuration_store->init();
		fake_rtc = new FakeRTC;
		rtc = fake_rtc;
		fake_rtc->settime(1580083200);
		MooredModeService::reset_for_tests();
	}

	void teardown() {
		delete fake_config_store;
		configuration_store = nullptr;
		delete fake_rtc;
		rtc = nullptr;
	}

	/// Enable detection with the production defaults, overridable per test.
	void enable(unsigned int radius_m = 150, unsigned int enter_fixes = 3, unsigned int exit_events = 2,
	            unsigned int holdoff_s = 0) {
		fake_config_store->write_param(ParamID::MOORED_DETECT_EN, (bool)true);
		fake_config_store->write_param(ParamID::MOORED_RADIUS_M, radius_m);
		fake_config_store->write_param(ParamID::MOORED_ENTER_FIXES, enter_fixes);
		fake_config_store->write_param(ParamID::MOORED_EXIT_EVENTS, exit_events);
		fake_config_store->write_param(ParamID::MOORED_AXL_HOLDOFF_S, holdoff_s);
	}

	/// Feed `n` fixes at the very same coordinates, stationary and at rest.
	void feed_still(unsigned int n, std::time_t t0 = 1000) {
		for (unsigned int i = 0; i < n; i++)
			MooredModeService::on_gnss_fix(BASE_LAT, BASE_LON, 0, t0 + i);
	}

	/// Anchor + exactly enough stationary fixes to engage MOORED (default cfg).
	void moor_it() {
		feed_still(4);  // 1 plants the anchor, 3 count toward MRP02
		CHECK_TRUE(MooredModeService::is_moored());
	}
};

// === Master switch =========================================================

TEST(MooredMode, DisabledNeverMoors) {
	// MOORED_DETECT_EN defaults to false: no amount of evidence may engage it,
	// and no state may accumulate. This is the guarantee that turtle and RSPB
	// deployments are byte-identical.
	feed_still(20);
	CHECK_FALSE(MooredModeService::is_moored());
	CHECK_FALSE(MooredModeService::has_reference());
	CHECK_EQUAL(0, (int)MooredModeService::stationary_fixes());
}

TEST(MooredMode, DisablingForcesUnderway) {
	enable();
	moor_it();
	fake_config_store->write_param(ParamID::MOORED_DETECT_EN, (bool)false);
	MooredModeService::evaluate(2000);
	CHECK_FALSE(MooredModeService::is_moored());
}

// === Entering MOORED =======================================================

TEST(MooredMode, FirstFixOnlyPlantsTheAnchor) {
	enable();
	// A single point carries no displacement information: it must set the
	// reference and count for nothing.
	MooredModeService::on_gnss_fix(BASE_LAT, BASE_LON, 0, 1000);
	CHECK_TRUE(MooredModeService::has_reference());
	CHECK_EQUAL(0, (int)MooredModeService::stationary_fixes());
	CHECK_FALSE(MooredModeService::is_moored());
}

TEST(MooredMode, MoorsAfterEnterFixes) {
	enable(150, 3);
	feed_still(3);  // anchor + 2 stationary
	CHECK_FALSE(MooredModeService::is_moored());
	feed_still(1, 2000);
	CHECK_TRUE(MooredModeService::is_moored());
}

TEST(MooredMode, GnssNoiseInsideRadiusStillMoors) {
	enable(150, 3);
	// +/- 5 m of receiver noise around the berth must not reset the counter.
	MooredModeService::on_gnss_fix(BASE_LAT, BASE_LON, 0, 1000);
	MooredModeService::on_gnss_fix(BASE_LAT + lat_offset_m(5), BASE_LON, 0, 1060);
	MooredModeService::on_gnss_fix(BASE_LAT - lat_offset_m(4), BASE_LON, 0, 1120);
	MooredModeService::on_gnss_fix(BASE_LAT + lat_offset_m(3), BASE_LON, 0, 1180);
	CHECK_TRUE(MooredModeService::is_moored());
}

TEST(MooredMode, OneFixOutsideRadiusResetsTheCounter) {
	enable(150, 3);
	feed_still(3);  // anchor + 2
	MooredModeService::on_gnss_fix(BASE_LAT + lat_offset_m(200), BASE_LON, 0, 1200);
	CHECK_FALSE(MooredModeService::is_moored());
	CHECK_EQUAL(0, (int)MooredModeService::stationary_fixes());
	// And the anchor moved with the vessel, so the next still run is measured
	// from where it actually stopped.
	feed_still(3, 1300);
	CHECK_FALSE(MooredModeService::is_moored());  // those 3 are 200 m from the NEW anchor
}

TEST(MooredMode, RadiusIsFlooredAtTenMetres) {
	// LittleFS does not checksum file DATA, so a corrupted 0 can reach the
	// classifier even though DTE clamps writes to >= 10 m. A 0 radius would
	// make every fix "movement" and silently kill the economy.
	enable(0, 2);
	fake_config_store->write_param(ParamID::MOORED_RADIUS_M, 0U);
	MooredModeService::on_gnss_fix(BASE_LAT, BASE_LON, 0, 1000);
	MooredModeService::on_gnss_fix(BASE_LAT + lat_offset_m(5), BASE_LON, 0, 1060);
	MooredModeService::on_gnss_fix(BASE_LAT + lat_offset_m(3), BASE_LON, 0, 1120);
	CHECK_TRUE(MooredModeService::is_moored());
}

// === Leaving MOORED on GNSS evidence =======================================

TEST(MooredMode, ExitsOnDisplacement) {
	enable();
	moor_it();
	MooredModeService::on_gnss_fix(BASE_LAT + lat_offset_m(500), BASE_LON, 0, 3000);
	CHECK_FALSE(MooredModeService::is_moored());
}

TEST(MooredMode, ExitsOnGroundSpeedEvenWithoutDisplacement) {
	enable();
	moor_it();
	// Same coordinates, but the receiver reports 2 m/s: the vessel is under way
	// and the next fix simply hasn't caught up yet.
	MooredModeService::on_gnss_fix(BASE_LAT, BASE_LON, 2000, 3000);
	CHECK_FALSE(MooredModeService::is_moored());
}

TEST(MooredMode, SpeedBelowThresholdDoesNotExit) {
	enable();
	moor_it();
	MooredModeService::on_gnss_fix(BASE_LAT, BASE_LON, MooredModeService::UNDERWAY_SPEED_MMS, 3000);
	CHECK_TRUE(MooredModeService::is_moored());  // strictly greater-than
}

TEST(MooredMode, SlowDriftAccumulatesAgainstFixedAnchorAndExits) {
	// The property the whole design hangs on: the reference is NOT re-centred
	// while stationary. A vessel creeping 30 m per fix stays "inside the
	// radius" relative to its previous position forever, but accumulates
	// against the anchor and eventually crosses it.
	enable(150, 3);
	MooredModeService::on_gnss_fix(BASE_LAT, BASE_LON, 0, 1000);  // anchor
	for (unsigned int step = 1; step <= 3; step++)                // +30, +60, +90
		MooredModeService::on_gnss_fix(BASE_LAT + lat_offset_m(30.0 * step), BASE_LON, 0, 1000 + step);
	CHECK_TRUE(MooredModeService::is_moored());

	MooredModeService::on_gnss_fix(BASE_LAT + lat_offset_m(120), BASE_LON, 0, 1004);
	CHECK_TRUE(MooredModeService::is_moored());
	MooredModeService::on_gnss_fix(BASE_LAT + lat_offset_m(149), BASE_LON, 0, 1005);
	CHECK_TRUE(MooredModeService::is_moored());
	MooredModeService::on_gnss_fix(BASE_LAT + lat_offset_m(180), BASE_LON, 0, 1006);
	CHECK_FALSE(MooredModeService::is_moored());  // drifted off the mooring
}

TEST(MooredMode, ReMoorsAtTheNewBerth) {
	enable(150, 3);
	moor_it();
	MooredModeService::on_gnss_fix(BASE_LAT + lat_offset_m(900), BASE_LON, 0, 3000);
	CHECK_FALSE(MooredModeService::is_moored());
	// The anchor followed; three still fixes at the new spot re-engage.
	for (unsigned int i = 0; i < 3; i++)
		MooredModeService::on_gnss_fix(BASE_LAT + lat_offset_m(900), BASE_LON, 0, 3100 + i);
	CHECK_TRUE(MooredModeService::is_moored());
}

// === Leaving MOORED on accelerometer evidence ==============================

TEST(MooredMode, MotionEventsExitAfterExitEvents) {
	enable(150, 3, 2, 0);
	moor_it();
	MooredModeService::on_motion_event(2000);
	CHECK_TRUE(MooredModeService::is_moored());  // one wave is not a departure
	MooredModeService::on_motion_event(2010);
	CHECK_FALSE(MooredModeService::is_moored());
}

TEST(MooredMode, MotionEventsIgnoredWhenAlreadyUnderway) {
	enable(150, 3, 2, 0);
	MooredModeService::on_motion_event(1000);
	MooredModeService::on_motion_event(1010);
	MooredModeService::on_motion_event(1020);
	CHECK_FALSE(MooredModeService::is_moored());
	CHECK_EQUAL(0, (int)MooredModeService::motion_events());
}

TEST(MooredMode, HoldoffSuppressesRepeatedMotionExits) {
	// Swell trips the accelerometer over and over. Without the hold-off each
	// trip costs a GNSS acquisition (via GNSS_TRIGGER_ON_AXL_WAKEUP) and the
	// economy evaporates.
	enable(150, 3, 2, 900);
	moor_it();
	MooredModeService::on_motion_event(2000);
	MooredModeService::on_motion_event(2010);
	CHECK_FALSE(MooredModeService::is_moored());  // first exit at t=2010

	// Re-moor at the same berth.
	feed_still(3, 2100);
	CHECK_TRUE(MooredModeService::is_moored());

	// Still inside the 900 s window: wake-ups must be swallowed.
	MooredModeService::on_motion_event(2200);
	MooredModeService::on_motion_event(2300);
	MooredModeService::on_motion_event(2400);
	CHECK_TRUE(MooredModeService::is_moored());

	// Past the window: the accelerometer is trusted again.
	MooredModeService::on_motion_event(2010 + 901);
	MooredModeService::on_motion_event(2010 + 902);
	CHECK_FALSE(MooredModeService::is_moored());
}

TEST(MooredMode, HoldoffZeroAllowsBackToBackExits) {
	enable(150, 3, 1, 0);
	moor_it();
	MooredModeService::on_motion_event(2000);
	CHECK_FALSE(MooredModeService::is_moored());
	feed_still(3, 2100);
	CHECK_TRUE(MooredModeService::is_moored());
	MooredModeService::on_motion_event(2200);
	CHECK_FALSE(MooredModeService::is_moored());
}

// === Robustness ============================================================

TEST(MooredMode, RtcRollbackDoesNotFreezeTheHoldoffForever) {
	// A WDT reset can bring the RTC back to the virtual epoch while .noinit
	// still holds real-epoch timestamps. Left alone, the hold-off comparison
	// would suppress every accelerometer exit for ~50 years — the tracker would
	// stay moored while the boat sailed away.
	enable(150, 3, 1, 900);
	moor_it();
	MooredModeService::on_motion_event(1580083200);  // exit, stamps a real epoch
	CHECK_FALSE(MooredModeService::is_moored());

	// RTC rolls back to the virtual frame.
	MooredModeService::evaluate(100);
	feed_still(4, 200);
	CHECK_TRUE(MooredModeService::is_moored());

	// Hold-off is now measured from t=100, not from the 2020 timestamp.
	MooredModeService::on_motion_event(100 + 901);
	CHECK_FALSE(MooredModeService::is_moored());
}

TEST(MooredMode, RestoreStateKeepsAValidCrcState) {
	enable();
	moor_it();
	MooredModeService::restore_state();  // CRC matches -> state survives
	CHECK_TRUE(MooredModeService::is_moored());
}

TEST(MooredMode, StatePublishedToReadOnlyParam) {
	enable();
	moor_it();
	CHECK_EQUAL(1U, fake_config_store->read_param<unsigned int>(ParamID::MOORED_STATE));
	MooredModeService::on_gnss_fix(BASE_LAT + lat_offset_m(600), BASE_LON, 0, 3000);
	CHECK_EQUAL(0U, fake_config_store->read_param<unsigned int>(ParamID::MOORED_STATE));
}

TEST(MooredMode, DistanceToReferenceIsNegativeBeforeFirstFix) {
	enable();
	DOUBLES_EQUAL(-1.0, MooredModeService::distance_to_reference_m(BASE_LAT, BASE_LON), 0.001);
	MooredModeService::on_gnss_fix(BASE_LAT, BASE_LON, 0, 1000);
	DOUBLES_EQUAL(100.0, MooredModeService::distance_to_reference_m(BASE_LAT + lat_offset_m(100), BASE_LON), 1.0);
}
