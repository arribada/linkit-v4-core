#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

#include "dive_mode_service.hpp"
#include "fake_config_store.hpp"
#include "fake_timer.hpp"
#include "fake_switch.hpp"
#include "fake_rtc.hpp"
#include "scheduler.hpp"


extern Timer *system_timer;
extern ConfigurationStore *configuration_store;
extern Scheduler *system_scheduler;
extern RTC *rtc;


TEST_GROUP(DiveMode) {
	FakeConfigurationStore *fake_config_store;
	FakeTimer *fake_timer;
	FakeRTC *fake_rtc;
	FakeSwitch fake_switch;

	void setup() {
		fake_timer = new FakeTimer;
		system_timer = fake_timer;
		fake_config_store = new FakeConfigurationStore;
		configuration_store = fake_config_store;
		system_scheduler = new Scheduler(system_timer);
		// Own the global rtc: notify_peer_event() dereferences `rtc`. Without this
		// DiveMode inherits a DANGLING `rtc` left by an earlier test that deleted
		// its FakeRTC without nulling the global → SIGSEGV. (Firmware guards with
		// `if (rtc && ...)`; this is purely test global-state hygiene.)
		fake_rtc = new FakeRTC;
		rtc = fake_rtc;
		configuration_store->init();
		system_timer->start();
		fake_switch.resume();
	}

	void teardown() {
		delete system_scheduler;
		delete fake_timer;
		delete fake_config_store;
		delete fake_rtc;
		rtc = nullptr;  // never leave a dangling global for the next test
	}

	void notify_underwater_state(bool state) {
		ServiceEvent e;
		e.event_type = ServiceEventType::SERVICE_LOG_UPDATED, e.event_data = state,
		e.event_source = ServiceIdentifier::UW_SENSOR;
		e.event_originator_unique_id = 0x12345678;
		ServiceManager::notify_peer_event(e);
	}

	void advance_time(unsigned int ms) {
		fake_timer->increment_counter(ms);
		system_scheduler->run();
	}
};


TEST(DiveMode, DiveModeEngagedAndDisengagedBySurfacing) {
	unsigned int start_period = 10;
	bool dive_mode_en = true;

	configuration_store->write_param(ParamID::UW_DIVE_MODE_ENABLE, dive_mode_en);
	configuration_store->write_param(ParamID::UW_DIVE_MODE_START_TIME, start_period);

	DiveModeService s(fake_switch);
	s.start();
	notify_underwater_state(true);
	CHECK_EQUAL(1000 * start_period, s.get_last_schedule());
	advance_time(s.get_last_schedule());
	CHECK_TRUE(fake_switch.is_paused());
	notify_underwater_state(false);
	CHECK_FALSE(fake_switch.is_paused());
	s.stop();
	CHECK_FALSE(fake_switch.is_paused());
}

TEST(DiveMode, DiveModeHasNoEffectWhenDisabled) {
	unsigned int start_period = 10;
	bool dive_mode_en = false;

	configuration_store->write_param(ParamID::UW_DIVE_MODE_ENABLE, dive_mode_en);
	configuration_store->write_param(ParamID::UW_DIVE_MODE_START_TIME, start_period);

	DiveModeService s(fake_switch);
	s.start();
	notify_underwater_state(true);
	CHECK_EQUAL(Service::SCHEDULE_DISABLED, s.get_last_schedule());
	CHECK_FALSE(fake_switch.is_paused());
	s.stop();
	CHECK_FALSE(fake_switch.is_paused());
}

TEST(DiveMode, DiveModeDisengagedIfServiceStopped) {
	unsigned int start_period = 10;
	bool dive_mode_en = true;

	configuration_store->write_param(ParamID::UW_DIVE_MODE_ENABLE, dive_mode_en);
	configuration_store->write_param(ParamID::UW_DIVE_MODE_START_TIME, start_period);

	DiveModeService s(fake_switch);
	s.start();
	notify_underwater_state(true);
	CHECK_EQUAL(1000 * start_period, s.get_last_schedule());
	advance_time(s.get_last_schedule());
	CHECK_TRUE(fake_switch.is_paused());
	s.stop();
	CHECK_FALSE(fake_switch.is_paused());
}


/// Disabling UW_DIVE_MODE_ENABLE while the state is Engaged must NOT leave
/// the reed paused at the next surface. The previous gating on
/// `service_is_enabled()` would silently drop surface events if the user
/// disabled dive mode mid-engaged via DTE — leaving the magnet inert.
TEST(DiveMode, DiveModeDisengagesEvenIfDisabledMidCycle) {
	unsigned int start_period = 10;

	configuration_store->write_param(ParamID::UW_DIVE_MODE_ENABLE, (bool)true);
	configuration_store->write_param(ParamID::UW_DIVE_MODE_START_TIME, start_period);

	DiveModeService s(fake_switch);
	s.start();

	// Engage normally
	notify_underwater_state(true);
	advance_time(s.get_last_schedule());
	CHECK_TRUE(fake_switch.is_paused());

	// User disables dive mode via DTE while Engaged
	configuration_store->write_param(ParamID::UW_DIVE_MODE_ENABLE, (bool)false);

	// Next surface must still resume the reed despite param being now false
	notify_underwater_state(false);
	CHECK_FALSE(fake_switch.is_paused());

	s.stop();
	CHECK_FALSE(fake_switch.is_paused());
}


/// Surfacing BEFORE the start timer fires must cancel the pending state and
/// must NOT pause the reed when the timer eventually fires. Otherwise the
/// magnet stays inert at the surface — see dive_mode_service.hpp StartPending
/// branch.
TEST(DiveMode, DiveModeCancelledByEarlySurfacing) {
	unsigned int start_period = 10;
	bool dive_mode_en = true;

	configuration_store->write_param(ParamID::UW_DIVE_MODE_ENABLE, dive_mode_en);
	configuration_store->write_param(ParamID::UW_DIVE_MODE_START_TIME, start_period);

	DiveModeService s(fake_switch);
	s.start();

	// Dive → StartPending, timer armed for 10 s
	notify_underwater_state(true);
	CHECK_EQUAL(1000 * start_period, s.get_last_schedule());
	CHECK_FALSE(fake_switch.is_paused());

	// Surface BEFORE timer fires (only half the period elapsed)
	advance_time(start_period * 1000 / 2);
	CHECK_FALSE(fake_switch.is_paused());
	notify_underwater_state(false);
	CHECK_FALSE(fake_switch.is_paused());

	// Let the originally-pending timer fire — must NOT engage dive mode now
	// because we cancelled the StartPending state on early surface.
	advance_time(start_period * 1000);
	CHECK_FALSE(fake_switch.is_paused());

	// Subsequent dives must still work normally (state machine reset to Idle)
	notify_underwater_state(true);
	advance_time(start_period * 1000);
	CHECK_TRUE(fake_switch.is_paused());
	notify_underwater_state(false);
	CHECK_FALSE(fake_switch.is_paused());

	s.stop();
}


TEST(DiveMode, DiveModeRunsMultipleTimes) {
	unsigned int start_period = 10;
	bool dive_mode_en = true;

	configuration_store->write_param(ParamID::UW_DIVE_MODE_ENABLE, dive_mode_en);
	configuration_store->write_param(ParamID::UW_DIVE_MODE_START_TIME, start_period);

	DiveModeService s(fake_switch);
	s.start();

	for (unsigned int i = 0; i < 10; i++) {
		notify_underwater_state(true);
		CHECK_EQUAL(1000 * start_period, s.get_last_schedule());
		advance_time(s.get_last_schedule());
		CHECK_TRUE(fake_switch.is_paused());
		notify_underwater_state(false);
		CHECK_FALSE(fake_switch.is_paused());
	}

	s.stop();
	CHECK_FALSE(fake_switch.is_paused());
}
