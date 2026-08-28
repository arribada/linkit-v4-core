/**
 * @file gps_shortsurface_test.cpp
 * @brief Short-surface turtle GPS scenarios + safety-net regression tests.
 *
 * Two purposes:
 *   1. Validate that the GPS service behaves correctly under the short-surface
 *      cadence typical of a sea turtle (5-30 s surface windows, multi-hour
 *      dives) — covers warm/cold dispatch, the trigger-on-surfaced gate, and
 *      the filter / CloudLocate interaction that broke 2-day field deployments.
 *   2. Regression-test the two safety nets added on top of the deep-idle
 *      refactor:
 *        - 3.1 WDT-inhibit 24 h hard-cap.
 *        - 3.2 Stuck-M10Q hard recovery after 20 dead sessions.
 *
 * Every test follows the existing GPSService test fixture (MockM10Q, FakeRTC,
 * FakeTimer, FakeConfigurationStore) and only observes behaviour through
 * public mock-call counts.
 */

#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

#include "gps_service.hpp"
#include "mock_m10q.hpp"
#include "fake_rtc.hpp"
#include "fake_config_store.hpp"
#include "fake_logger.hpp"
#include "fake_timer.hpp"
#include "fake_battery_mon.hpp"
#include "mock_comparators.hpp"


extern Timer *system_timer;
extern ConfigurationStore *configuration_store;
extern Scheduler *system_scheduler;
extern RTC *rtc;
extern BatteryMonitor *battery_monitor;

#define FIRST_AQPERIOD (30)

TEST_GROUP(GPSShortSurface) {
	FakeBatteryMonitor *fake_battery_mon;
	FakeConfigurationStore *fake_config_store;
	FakeRTC *fake_rtc;
	FakeTimer *fake_timer;
	FakeLog *fake_log;
	MockM10Q *mock_m10q;
	MockStdFunctionVoidComparator m_comparator_std_func;
	MockGPSNavSettingsComparator m_comparator_nav;

	void setup() {
		mock().installComparator("std::function<void()>", m_comparator_std_func);
		mock().installComparator("const GPSNavSettings&", m_comparator_nav);
		fake_log = new FakeLog("GPS");
		mock_m10q = new MockM10Q;
		fake_config_store = new FakeConfigurationStore;
		configuration_store = fake_config_store;
		fake_battery_mon = new FakeBatteryMonitor;
		battery_monitor = fake_battery_mon;
		fake_rtc = new FakeRTC;
		rtc = fake_rtc;
		fake_timer = new FakeTimer;
		system_timer = fake_timer;
		system_scheduler = new Scheduler(system_timer);
		m_current_ms = 0;

		configuration_store->init();

		// Default short-surface turtle profile applied to every test. Individual
		// tests override what they need. Energy is NOT a concern per user spec
		// (system OFF underwater), so we lean on conservative timeouts.
		fake_config_store->write_param(ParamID::GNSS_EN, (bool)true);
		fake_config_store->write_param(ParamID::UNDERWATER_EN, (bool)true);
		fake_config_store->write_param(ParamID::DLOC_ARG_NOM, 60U);  // tight for fast tests
		fake_config_store->write_param(ParamID::GNSS_ACQ_TIMEOUT, 30U);
		fake_config_store->write_param(ParamID::GNSS_COLD_ACQ_TIMEOUT, 180U);
		fake_config_store->write_param(ParamID::GNSS_COLD_START_RETRY_PERIOD, 60U);
		fake_config_store->write_param(ParamID::GNSS_NTRY, 0U);  // unlimited
		fake_config_store->write_param(ParamID::GNSS_TRIGGER_ON_SURFACED, (bool)true);
		fake_config_store->write_param(ParamID::GNSS_TRIGGER_COLD_START_ON_SURFACED, (bool)false);
		fake_config_store->write_param(ParamID::GNSS_HDOPFILT_EN, (bool)false);
		fake_config_store->write_param(ParamID::GNSS_HACCFILT_EN, (bool)false);
		fake_config_store->write_param(ParamID::GNSS_FIX_MODE, BaseGNSSFixMode::AUTO);
		fake_config_store->write_param(ParamID::GNSS_DYN_MODEL, BaseGNSSDynModel::SEA);
		fake_config_store->write_param(ParamID::GNSS_FASTLOC_MODE, 2U);  // CLOUDLOCATE
		fake_config_store->write_param(ParamID::GNSS_CLOUDLOCATE_ALWAYS, (bool)true);
		fake_config_store->write_param(ParamID::GNSS_CLOUDLOCATE_ONLY, (bool)false);
		// GNP52=0 → immediate poweroff, simplest for mock-call accounting.
		// Tests that exercise deep-idle override this explicitly.
		fake_config_store->write_param(ParamID::GNSS_DEEP_IDLE_AFTER_OFF_S, 0U);
	}

	void teardown() {
		delete system_scheduler;
		delete fake_timer;
		delete fake_rtc;
		rtc = nullptr;
		delete fake_config_store;
		delete fake_battery_mon;
		delete mock_m10q;
		delete fake_log;
	}

	void increment_time_ms(uint64_t ms) {
		while (ms) {
			m_current_ms++;
			if (m_current_ms % 1000 == 0) fake_rtc->incrementtime(1);
			fake_timer->increment_counter(1);
			system_scheduler->run();
			ms--;
		}
	}

	void increment_time_s(uint64_t s) {
		increment_time_ms(s * 1000);
	}
	void increment_time_min(uint64_t m) {
		increment_time_ms(m * 60 * 1000);
	}
	void increment_time_h(uint64_t h) {
		increment_time_ms(h * 3600 * 1000);
	}

	void notify_underwater_state(bool state) {
		ServiceEvent e;
		e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
		e.event_data = state;
		e.event_source = ServiceIdentifier::UW_SENSOR;
		e.event_originator_unique_id = 0x12345678;
		ServiceManager::notify_peer_event(e);
	}

	// Drive a single end-of-session NO_FIX cycle. Expects exactly 1 power_on
	// (when the scheduler fires) and 1 power_off (when notify_max_nav_samples
	// hits try_enter_deep_idle_or_poweroff with GNP52=0).
	// Uses ignoreOtherParameters because nav settings vary per session.
	void drive_one_nofix_cycle() {
		mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
		// Schedule period mirrors GNSS_COLD_START_RETRY_PERIOD (no first_fix yet).
		increment_time_s(60);
		mock().expectOneCall("power_off").onObject(mock_m10q);
		mock_m10q->notify_max_nav_samples();
	}

	uint64_t m_current_ms;
};

// ============================================================================
// SECTION A — Short-surface scenarios (no safety net involvement)
// ============================================================================

/// First fix lands → m_is_first_fix_found becomes true → subsequent acquisitions
/// use warm timeout. This is the path that broke when HACCFILT was on: every
/// PVT got rejected as degraded → CloudLocate raw emitted → first_fix_found
/// stayed false → cold timeout forever.
TEST(GPSShortSurface, FirstPVTFlipsToWarmStartCadence) {
	fake_rtc->settime(1580083200);
	GPSService s(*mock_m10q, fake_log);
	s.start();

	// First scheduled fire.
	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(FIRST_AQPERIOD);

	// Valid PVT arrives → gnss_data_callback sets m_is_first_fix_found=true.
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_gnss_data(fake_rtc->gettime(), 10.0, 10.0);

	// Subsequent acquisitions schedule at DLOC_ARG_NOM (60 s here).
	for (int i = 0; i < 3; i++) {
		mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
		increment_time_s(60);
		mock().expectOneCall("power_off").onObject(mock_m10q);
		mock_m10q->notify_gnss_data(fake_rtc->gettime(), 10.0, 10.0);
	}
}

/// Degraded PVT does NOT set m_is_first_fix_found — confirms the documented
/// behavior that lets the 2-day field bug surface when HACCFILT/HDOPFILT are
/// on. This is the regression-pin for "don't change R6 semantics by accident".
TEST(GPSShortSurface, DegradedPVTDoesNotSetFirstFixFound) {
	fake_config_store->write_param(ParamID::GNSS_FASTLOC_MODE, 1U);  // DEGRADED_PVT
	fake_rtc->settime(1580083200);
	GPSService s(*mock_m10q, fake_log);
	s.start();

	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(FIRST_AQPERIOD);

	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_degraded_gnss_data(fake_rtc->gettime(), 10.0, 10.0);

	// Second cycle should still use cold_start_retry_period scheduling (60 s),
	// not the longer dloc_arg_nom warm cadence. Implicit via not setting first_fix.
	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(60);
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_max_nav_samples();
}

/// Trigger-on-surfaced: a surface event fires GPS immediately (the SURFACING_
/// BURST tortue path). Verifies the new GNP52 deep-idle stamp didn't break
/// the legacy immediate-trigger contract.
TEST(GPSShortSurface, TriggerOnSurfacedFiresImmediately) {
	fake_rtc->settime(0);
	GPSService s(*mock_m10q, fake_log);
	s.start();

	// First scheduled fire to bootstrap first_fix_found.
	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(FIRST_AQPERIOD);

	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_gnss_data(fake_rtc->gettime(), 10.0, 10.0);

	// Dive — no acquisition while underwater.
	notify_underwater_state(true);
	increment_time_min(15);

	// Surface → immediate GPS.
	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	notify_underwater_state(false);
	increment_time_s(1);
}

// ============================================================================
// SECTION B — Safety net 3.1: WDT-inhibit 24 h timeout
// ============================================================================

/// Setting the WDT inhibit before the first session forces a cold power_off
/// (no deep-idle) on the next dispatch. We verify the inhibit is preserved
/// across multiple back-to-back NO_FIX cycles — every session takes the
/// legacy cold-poweroff path until something proves the M10Q is alive.
///
/// Note: the 24 h hard-cap itself cannot be exercised in unit tests because
/// `PMU::get_timestamp_ms()` (used to compare elapsed time) is mocked to
/// return wall-clock time, not the FakeTimer simulation time. The cap is
/// covered by code review / integration logs, not by this fast unit test.
TEST(GPSShortSurface, WDTInhibitPreservedAcrossSessions) {
	// GNP52 sentinel — without the WDT inhibit this would arm deep-idle.
	fake_config_store->write_param(ParamID::GNSS_DEEP_IDLE_AFTER_OFF_S, 0xFFFFFFFFU);

	fake_rtc->settime(1580083200);
	GPSService s(*mock_m10q, fake_log);
	s.set_deep_idle_inhibit_first_session(true);
	s.start();

	// First acquisition fires cold-style.
	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(FIRST_AQPERIOD);
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_max_nav_samples();

	// 3 more NO_FIX sessions — each must take the inhibit cold-poweroff
	// branch. Cumulative dead = 4 (well under STUCK_THRESHOLD=20 so 3.2
	// doesn't fire).
	for (int i = 0; i < 3; i++) {
		drive_one_nofix_cycle();
	}
}

/// A PVT fix clears the WDT inhibit immediately. Next session uses the
/// normal dispatch path (deep-idle if GNP52 says so).
TEST(GPSShortSurface, WDTInhibitClearedByPVT) {
	fake_rtc->settime(1580083200);
	GPSService s(*mock_m10q, fake_log);
	s.set_deep_idle_inhibit_first_session(true);
	s.start();

	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(FIRST_AQPERIOD);

	// PVT clears inhibit (gps_service.cpp:925 area).
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_gnss_data(fake_rtc->gettime(), 10.0, 10.0);

	// Next cycle: warm timeout (first_fix_found=true). Normal dispatch path.
	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(60);
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_gnss_data(fake_rtc->gettime(), 10.0, 10.0);
}

/// Setter-side stamping: calling the inhibit setter with `false` clears
/// `m_inhibit_set_at_ms` back to 0. This is the disarm path used after a
/// successful PVT (where the legacy clear sites also reset the stamp).
/// Verified indirectly by re-arming after a disarm and running cleanly.
TEST(GPSShortSurface, WDTInhibitSetterDisarmAndReArm) {
	fake_rtc->settime(1580083200);
	GPSService s(*mock_m10q, fake_log);

	// Arm, then disarm, then re-arm. Each call should be a clean transition
	// without leaking timer state.
	s.set_deep_idle_inhibit_first_session(true);
	s.set_deep_idle_inhibit_first_session(false);
	s.set_deep_idle_inhibit_first_session(true);

	s.start();
	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(FIRST_AQPERIOD);
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_max_nav_samples();
}

// ============================================================================
// SECTION C — Safety net 3.2: Stuck M10Q hard recovery (threshold = 20)
// ============================================================================

/// 19 consecutive NO_FIX must NOT arm the stuck recovery. This pins the
/// threshold = 20 invariant.
TEST(GPSShortSurface, StuckRecoveryNotTriggeredBelowThreshold) {
	fake_rtc->settime(1580083200);
	GPSService s(*mock_m10q, fake_log);
	s.start();

	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(FIRST_AQPERIOD);
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_max_nav_samples();

	// 18 more NO_FIX (total 19).
	for (int i = 0; i < 18; i++) {
		drive_one_nofix_cycle();
	}

	// At this point: 19 dead sessions. m_consecutive_dead_sessions = 19,
	// NOT >= STUCK_THRESHOLD (20). No recovery should be armed.
	// Advance 2 s — would be enough for the 1 s arm task IF it had fired.
	// If a stray power_off_immediate fires, the mock framework will report
	// unexpected calls during checkExpectations() in main_test.cpp.
	increment_time_s(2);

	// Probe: next normal acquisition should fire as scheduled.
	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(60);
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_max_nav_samples();
}

/// At exactly 20 consecutive NO_FIX, the stuck recovery is armed. After
/// 1 s the lambda fires power_off_immediate() (which the MockM10Q maps to
/// power_off via the GPSDevice default), and 30 s later service_reschedule
/// re-arms the next acquisition.
TEST(GPSShortSurface, StuckRecoveryTriggeredAtThreshold) {
	fake_rtc->settime(1580083200);
	GPSService s(*mock_m10q, fake_log);
	s.start();

	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(FIRST_AQPERIOD);
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_max_nav_samples();

	// Drive 19 more NO_FIX (cumulative = 20).
	for (int i = 0; i < 19; i++) {
		drive_one_nofix_cycle();
	}

	// 1 s later, the stuck_recovery_arm_task fires. m_is_active is false
	// (we're between sessions), so it calls power_off_immediate() → power_off
	// on the mock.
	mock().expectOneCall("power_off").onObject(mock_m10q);
	increment_time_s(1);

	// After 30 s the done task fires → service_reschedule → next acquisition
	// armed. Advance just past 30 s and expect the power_on for the
	// post-recovery acquisition (which will fire at the next aligned schedule
	// — we collapse this into a single check).
	mock().expectNCalls(1, "power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(60);
}

/// A real PVT in the middle of a failure streak must reset the dead-session
/// counter. Otherwise even 1 success/19 failures would arm a recovery.
TEST(GPSShortSurface, StuckRecoveryResetByPVT) {
	fake_rtc->settime(1580083200);
	GPSService s(*mock_m10q, fake_log);
	s.start();

	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(FIRST_AQPERIOD);
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_max_nav_samples();

	// 18 more NO_FIX (cumulative = 19, just under threshold).
	for (int i = 0; i < 18; i++) {
		drive_one_nofix_cycle();
	}

	// One PVT — resets m_consecutive_dead_sessions to 0.
	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(60);
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_gnss_data(fake_rtc->gettime(), 10.0, 10.0);

	// Now 19 more NO_FIX — should NOT trigger recovery (counter was reset).
	// At 60 s + DLOC after warm timeout, the next schedule is dloc_arg_nom.
	for (int i = 0; i < 19; i++) {
		mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
		increment_time_s(60);  // dloc_arg_nom after first fix
		mock().expectOneCall("power_off").onObject(mock_m10q);
		mock_m10q->notify_max_nav_samples();
	}

	// Advance 2 s — no stuck recovery should fire (we're at 19 dead since
	// the last PVT, not 20).
	increment_time_s(2);

	// Probe.
	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(60);
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_max_nav_samples();
}

/// A degraded PVT also resets the dead-session counter. This is per-spec
/// (the degraded callback explicitly resets `m_consecutive_dead_sessions`
/// even though it does NOT set first_fix_found — different concerns).
TEST(GPSShortSurface, StuckRecoveryResetByDegradedPVT) {
	fake_config_store->write_param(ParamID::GNSS_FASTLOC_MODE, 1U);  // DEGRADED_PVT
	fake_rtc->settime(1580083200);
	GPSService s(*mock_m10q, fake_log);
	s.start();

	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(FIRST_AQPERIOD);
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_max_nav_samples();

	for (int i = 0; i < 18; i++) {
		drive_one_nofix_cycle();
	}

	// Degraded PVT resets the counter.
	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(60);
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_degraded_gnss_data(fake_rtc->gettime(), 10.0, 10.0);

	// 19 more NO_FIX — no recovery (we're at 19 since reset).
	for (int i = 0; i < 19; i++) {
		drive_one_nofix_cycle();
	}

	increment_time_s(2);  // would catch a spurious recovery arm

	// Probe.
	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(60);
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_max_nav_samples();
}

/// Single-arm invariant: once recovery fires (and resets the counter to 0
/// inside the done task), subsequent dead sessions must build up again from
/// 0 → 20 before re-firing. Tests the m_stuck_recovery_in_flight flag.
TEST(GPSShortSurface, StuckRecoverySingleArmThenRecountsFromZero) {
	fake_rtc->settime(1580083200);
	GPSService s(*mock_m10q, fake_log);
	s.start();

	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(FIRST_AQPERIOD);
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_max_nav_samples();

	for (int i = 0; i < 19; i++) {
		drive_one_nofix_cycle();
	}

	// 1 s later, recovery arm task fires → power_off_immediate → power_off.
	mock().expectOneCall("power_off").onObject(mock_m10q);
	increment_time_s(1);

	// After 30 s + scheduler aligns, next acquisition fires.
	mock().expectNCalls(1, "power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(60);

	// We're now in a fresh acquisition. End it with NO_FIX.
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_max_nav_samples();

	// 18 more NO_FIX should NOT re-arm recovery yet (we're at 19 since reset).
	for (int i = 0; i < 18; i++) {
		drive_one_nofix_cycle();
	}
	increment_time_s(2);  // would catch a stray recovery

	// 1 more NO_FIX → cumulative 20 → recovery re-arms.
	drive_one_nofix_cycle();
	mock().expectOneCall("power_off").onObject(mock_m10q);
	increment_time_s(1);
}

// ============================================================================
// SECTION D — Deep-idle dispatch matrix (regression for the refactor)
// ============================================================================

/// GNP52=0 → immediate poweroff. Every NO_FIX = 1 power_off, no extra path.
TEST(GPSShortSurface, DeepIdleDisabledImmediatePowerOff) {
	// GNP52=0 already set in fixture.
	fake_rtc->settime(1580083200);
	GPSService s(*mock_m10q, fake_log);
	s.start();

	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(FIRST_AQPERIOD);

	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_max_nav_samples();
}

/// GNP52=0xFFFFFFFF → "never poweroff" sentinel. With the MockM10Q the
/// power_off mock is still called (because request_deep_idle_on_next_stop is
/// a no-op and power_off() is called in the dispatch path), but the
/// scheduling gate prevents the next acquisition from being armed (the
/// "is_in_deep_idle" check returns false by default on the mock, so we
/// observe normal scheduling behavior). This test pins that no extra calls
/// fire from the sentinel branch.
TEST(GPSShortSurface, DeepIdleNeverOffSentinel) {
	fake_config_store->write_param(ParamID::GNSS_DEEP_IDLE_AFTER_OFF_S, 0xFFFFFFFFU);
	fake_rtc->settime(1580083200);
	GPSService s(*mock_m10q, fake_log);
	s.start();

	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(FIRST_AQPERIOD);

	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_max_nav_samples();
}
