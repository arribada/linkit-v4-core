#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

#include "cam_service.hpp"
#include "axl_sensor_service.hpp"
#include "fake_cam.hpp"
#include "fake_config_store.hpp"
#include "fake_logger.hpp"
#include "fake_rtc.hpp"
#include "fake_timer.hpp"
#include "fake_battery_mon.hpp"
#include "scheduler.hpp"

extern Timer *system_timer;
extern ConfigurationStore *configuration_store;
extern Scheduler *system_scheduler;
extern RTC *rtc;
extern BatteryMonitor *battery_monitor;

static_assert(sizeof(CAMLogEntry) == sizeof(LogEntry), "CAMLogEntry must overlay LogEntry");

/*
 * CAMService drives a camera whose power sequences take 2.1 s each and run
 * asynchronously on the scheduler (see CAMDevice). What is checked here is
 * the service's side of that contract: when a sequence is requested, that the
 * scheduler is free meanwhile, how a session completes, what the triggers do
 * to a camera that is already filming, and every path that must leave the
 * camera off (stop, timeout, device error).
 */
TEST_GROUP(CAMService) {
	static constexpr unsigned int PERIOD_ON_S = 60;
	static constexpr unsigned int PERIOD_OFF_S = 300;
	static constexpr unsigned int SEQ_MS = FakeCAMDevice::SEQUENCE_MS;
	// What service_next_schedule_in_ms() arms for each half of the cycle.
	static constexpr unsigned int OFF_WAIT_MS = PERIOD_OFF_S * 1000 + SEQ_MS;
	static constexpr unsigned int ON_WAIT_MS = PERIOD_ON_S * 1000 + SEQ_MS;

	FakeConfigurationStore *fake_config_store;
	FakeTimer *fake_timer;
	FakeLog *fake_logger;
	FakeRTC *fake_rtc;
	FakeBatteryMonitor *fake_battery_monitor;
	unsigned int num_log_events;

	void setup() {
		ServiceManager::reset();
		fake_timer = new FakeTimer;
		system_timer = fake_timer;
		fake_rtc = new FakeRTC;
		rtc = fake_rtc;
		fake_rtc->settime(1580083200);  // 27/01/2020 00:00:00
		fake_config_store = new FakeConfigurationStore;
		configuration_store = fake_config_store;
		configuration_store->init();
		fake_logger = new FakeLog;
		fake_logger->create();
		fake_battery_monitor = new FakeBatteryMonitor;
		battery_monitor = fake_battery_monitor;
		fake_battery_monitor->set_values(/*level*/ 100, /*mv*/ 4100, /*is_low*/ false);
		system_scheduler = new Scheduler(system_timer);
		fake_timer->start();
		num_log_events = 0;

		configuration_store->write_param(ParamID::CAM_ENABLE, true);
		configuration_store->write_param(ParamID::CAM_PERIOD_ON, PERIOD_ON_S);
		configuration_store->write_param(ParamID::CAM_PERIOD_OFF, PERIOD_OFF_S);
		configuration_store->write_param(ParamID::CAM_TRIGGER_ON_SURFACED, false);
		configuration_store->write_param(ParamID::CAM_TRIGGER_ON_AXL_WAKEUP, false);
		configuration_store->write_param(ParamID::LB_EN, false);
	}

	void teardown() {
		delete system_scheduler;
		delete fake_timer;
		delete fake_config_store;
		delete fake_logger;
		delete fake_rtc;
		delete fake_battery_monitor;
		system_scheduler = nullptr;
		system_timer = nullptr;
		rtc = nullptr;
		configuration_store = nullptr;
		battery_monitor = nullptr;
	}

	/// @brief Advance the fake clock one millisecond at a time, running the
	/// scheduler after each tick, so a task posted by a task fires on time.
	void run_for(unsigned int ms) {
		for (unsigned int i = 0; i < ms; i++) {
			fake_timer->increment_counter(1);
			system_scheduler->run();
		}
	}

	void start(CAMService & s) {
		s.start([this](ServiceEvent &event) {
			if (event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) num_log_events++;
		});
	}

	/// @brief Run a full power-on: wait out the OFF period, then the sequence.
	void bring_camera_on(FakeCAMDevice & drv) {
		run_for(OFF_WAIT_MS);
		CHECK_EQUAL(1, drv.on_requests);
		run_for(SEQ_MS);
		CHECK_TRUE(drv.is_powered_on());
	}

	void notify_underwater_state(bool submerged) {
		ServiceEvent e;
		e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
		e.event_source = ServiceIdentifier::UW_SENSOR;
		e.event_data = submerged;
		e.event_originator_unique_id = 0x12345678;
		ServiceManager::notify_peer_event(e);
	}

	void notify_axl_wakeup() {
		ServiceSensorData sensor_data;
		sensor_data.port[AXLSensorPort::WAKEUP_TRIGGERED] = 1.0;
		ServiceEvent e;
		e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
		e.event_source = ServiceIdentifier::AXL_SENSOR;
		e.event_data = sensor_data;
		e.event_originator_unique_id = 0x12345678;
		ServiceManager::notify_peer_event(e);
	}

	CAMLogEntry log_entry(unsigned int index) {
		CAMLogEntry entry;
		fake_logger->read(&entry, (int)index);
		return entry;
	}
};

TEST(CAMService, DisabledByParam) {
	FakeCAMDevice drv;
	CAMService s(drv, fake_logger);
	configuration_store->write_param(ParamID::CAM_ENABLE, false);

	start(s);
	run_for(2 * OFF_WAIT_MS);

	CHECK_EQUAL(0, drv.on_requests);
	CHECK_EQUAL(0, fake_logger->num_entries());
	s.stop();
}

TEST(CAMService, PeriodOnZeroDisables) {
	FakeCAMDevice drv;
	CAMService s(drv, fake_logger);
	configuration_store->write_param(ParamID::CAM_PERIOD_ON, 0U);

	start(s);
	run_for(2 * OFF_WAIT_MS);

	CHECK_EQUAL(0, drv.on_requests);
	s.stop();
}

TEST(CAMService, PeriodicCycleAlternatesOnAndOff) {
	FakeCAMDevice drv;
	CAMService s(drv, fake_logger);

	start(s);

	// The cycle starts with the OFF period; nothing before it elapses.
	run_for(OFF_WAIT_MS - 1);
	CHECK_EQUAL(0, drv.on_requests);
	run_for(1);
	CHECK_EQUAL(1, drv.on_requests);
	// The session is open but the device only reports ON after its sequence.
	CHECK_TRUE(drv.is_sequence_running());
	CHECK_FALSE(drv.is_powered_on());
	CHECK_EQUAL(0, fake_logger->num_entries());
	run_for(SEQ_MS - 1);
	CHECK_FALSE(drv.is_powered_on());
	run_for(1);
	CHECK_TRUE(drv.is_powered_on());
	CHECK_EQUAL(1, fake_logger->num_entries());
	CHECK_EQUAL(1, num_log_events);
	CAMLogEntry on = log_entry(0);
	CHECK_EQUAL((unsigned int)CAMEventType::ON, (unsigned int)on.info.event_type);
	CHECK_EQUAL(1, (unsigned int)on.info.counter);
	CHECK_EQUAL(4100, (unsigned int)on.info.batt_voltage);
	CHECK_EQUAL(LOG_CAM, (unsigned int)on.header.log_type);

	// ON period, then the OFF sequence.
	run_for(ON_WAIT_MS - 1);
	CHECK_EQUAL(0, drv.off_requests);
	run_for(1);
	CHECK_EQUAL(1, drv.off_requests);
	run_for(SEQ_MS);
	CHECK_FALSE(drv.is_powered_on());
	CHECK_EQUAL(2, fake_logger->num_entries());
	CAMLogEntry off = log_entry(1);
	CHECK_EQUAL((unsigned int)CAMEventType::OFF, (unsigned int)off.info.event_type);
	CHECK_EQUAL(1, (unsigned int)off.info.counter);

	// And around again: the next ON comes after another OFF period.
	run_for(OFF_WAIT_MS);
	CHECK_EQUAL(2, drv.on_requests);
	run_for(SEQ_MS);
	CHECK_EQUAL(2, (unsigned int)log_entry(2).info.counter);

	s.stop();
}

TEST(CAMService, SchedulerKeepsRunningDuringPowerSequence) {
	FakeCAMDevice drv;
	CAMService s(drv, fake_logger);
	bool other_task_ran = false;

	start(s);
	run_for(OFF_WAIT_MS);
	CHECK_EQUAL(1, drv.on_requests);

	// Something else posted for the middle of the 2.1 s sequence must run
	// there, not after it: that is the whole point of the asynchronous device.
	system_scheduler->post_task_prio([&other_task_ran]() { other_task_ran = true; }, "Other",
	                                 Scheduler::DEFAULT_PRIORITY, SEQ_MS / 2);
	run_for(SEQ_MS / 2);
	CHECK_TRUE(other_task_ran);
	CHECK_FALSE(drv.is_powered_on());
	run_for(SEQ_MS / 2);
	CHECK_TRUE(drv.is_powered_on());

	s.stop();
}

TEST(CAMService, SurfacingStartsCaptureWhenTriggerEnabled) {
	FakeCAMDevice drv;
	CAMService s(drv, fake_logger);
	configuration_store->write_param(ParamID::CAM_TRIGGER_ON_SURFACED, true);

	start(s);
	run_for(1000);  // well inside the OFF period
	CHECK_EQUAL(0, drv.on_requests);

	notify_underwater_state(true);  // dive: nothing
	run_for(1);
	CHECK_EQUAL(0, drv.on_requests);

	notify_underwater_state(false);  // surface: capture now
	run_for(1);
	CHECK_EQUAL(1, drv.on_requests);
	run_for(SEQ_MS);
	CHECK_TRUE(drv.is_powered_on());

	// The triggered capture then lasts one ON period, like a scheduled one.
	run_for(ON_WAIT_MS - 1);
	CHECK_EQUAL(0, drv.off_requests);
	run_for(1);
	CHECK_EQUAL(1, drv.off_requests);

	s.stop();
}

TEST(CAMService, SurfacingIgnoredWhenTriggerDisabled) {
	FakeCAMDevice drv;
	CAMService s(drv, fake_logger);

	start(s);
	run_for(1000);
	notify_underwater_state(false);
	run_for(1);
	CHECK_EQUAL(0, drv.on_requests);

	// The periodic schedule is untouched.
	run_for(OFF_WAIT_MS - 1001);
	CHECK_EQUAL(1, drv.on_requests);

	s.stop();
}

TEST(CAMService, SurfacingDoesNotCutARunningCapture) {
	FakeCAMDevice drv;
	CAMService s(drv, fake_logger);
	configuration_store->write_param(ParamID::CAM_TRIGGER_ON_SURFACED, true);

	start(s);
	bring_camera_on(drv);

	run_for(1000);
	notify_underwater_state(false);
	run_for(1);
	CHECK_EQUAL(0, drv.off_requests);
	CHECK_TRUE(drv.is_powered_on());

	// The pending OFF still fires exactly when it was armed.
	run_for(ON_WAIT_MS - 1001 - 1);
	CHECK_EQUAL(0, drv.off_requests);
	run_for(1);
	CHECK_EQUAL(1, drv.off_requests);

	s.stop();
}

TEST(CAMService, AxlWakeupStartsCaptureWhenTriggerEnabled) {
	FakeCAMDevice drv;
	CAMService s(drv, fake_logger);
	configuration_store->write_param(ParamID::CAM_TRIGGER_ON_AXL_WAKEUP, true);

	start(s);
	run_for(1000);
	notify_axl_wakeup();
	run_for(1);
	CHECK_EQUAL(1, drv.on_requests);

	s.stop();
}

TEST(CAMService, AxlWakeupIgnoredWhenTriggerDisabled) {
	FakeCAMDevice drv;
	CAMService s(drv, fake_logger);

	start(s);
	run_for(1000);
	notify_axl_wakeup();
	run_for(1);
	CHECK_EQUAL(0, drv.on_requests);

	s.stop();
}

TEST(CAMService, AxlWakeupDoesNotCutARunningCapture) {
	FakeCAMDevice drv;
	CAMService s(drv, fake_logger);
	configuration_store->write_param(ParamID::CAM_TRIGGER_ON_AXL_WAKEUP, true);

	start(s);
	bring_camera_on(drv);

	// On a moving animal wake-ups are frequent: none of them may toggle the
	// camera off, and none may disturb the OFF already armed.
	for (unsigned int i = 0; i < 5; i++) {
		run_for(1000);
		notify_axl_wakeup();
		run_for(1);
	}
	CHECK_EQUAL(0, drv.off_requests);
	CHECK_TRUE(drv.is_powered_on());
	run_for(ON_WAIT_MS - 5 * 1001 - 1);
	CHECK_EQUAL(0, drv.off_requests);
	run_for(1);
	CHECK_EQUAL(1, drv.off_requests);

	s.stop();
}

TEST(CAMService, StopPowersOffBlockingWhileFilming) {
	FakeCAMDevice drv;
	CAMService s(drv, fake_logger);

	start(s);
	bring_camera_on(drv);
	CHECK_EQUAL(1, fake_logger->num_entries());

	// Leaving Operational: the camera must be really off when stop() returns,
	// which only the blocking sequence guarantees.
	s.stop();
	CHECK_EQUAL(1, drv.blocking_off_calls);
	CHECK_FALSE(drv.is_powered_on());
	CHECK_EQUAL(0, drv.off_requests);
	// No session was open, so nothing is logged and nothing is re-armed.
	CHECK_EQUAL(1, fake_logger->num_entries());
	run_for(2 * OFF_WAIT_MS);
	CHECK_EQUAL(1, drv.on_requests);
}

TEST(CAMService, StopDuringPowerSequenceClosesTheSession) {
	FakeCAMDevice drv;
	CAMService s(drv, fake_logger);

	start(s);
	run_for(OFF_WAIT_MS);
	CHECK_EQUAL(1, drv.on_requests);
	CHECK_TRUE(drv.is_sequence_running());

	s.stop();
	CHECK_EQUAL(1, drv.blocking_off_calls);
	CHECK_FALSE(drv.is_powered_on());
	CHECK_FALSE(drv.is_sequence_running());
	// One OFF entry closes the interrupted session -- and only one: the
	// device's own power-off event must not add a second completion.
	CHECK_EQUAL(1, fake_logger->num_entries());
	CHECK_EQUAL((unsigned int)CAMEventType::OFF, (unsigned int)log_entry(0).info.event_type);
	run_for(2 * OFF_WAIT_MS);
	CHECK_EQUAL(1, drv.on_requests);
}

TEST(CAMService, LostSequenceIsRecoveredByTimeout) {
	FakeCAMDevice drv;
	CAMService s(drv, fake_logger);
	drv.complete_sequences = false;  // the sequence's tasks are lost

	start(s);
	run_for(OFF_WAIT_MS);
	CHECK_EQUAL(1, drv.on_requests);

	// Nothing arrives; the safety-net timeout cancels the session.
	run_for(30000 - 1);
	CHECK_EQUAL(0, drv.blocking_off_calls);
	run_for(1);
	CHECK_EQUAL(1, drv.blocking_off_calls);
	CHECK_FALSE(drv.is_sequence_running());
	CHECK_EQUAL(1, fake_logger->num_entries());
	CHECK_EQUAL((unsigned int)CAMEventType::OFF, (unsigned int)log_entry(0).info.event_type);

	// And the cycle resumes from the OFF period.
	drv.complete_sequences = true;
	run_for(OFF_WAIT_MS - 1);
	CHECK_EQUAL(1, drv.on_requests);
	run_for(1);
	CHECK_EQUAL(2, drv.on_requests);
	run_for(SEQ_MS);
	CHECK_TRUE(drv.is_powered_on());

	s.stop();
}

TEST(CAMService, DeviceErrorPowersOffAndRestartsFromOffPeriod) {
	FakeCAMDevice drv;
	CAMService s(drv, fake_logger);

	start(s);
	bring_camera_on(drv);
	run_for(1000);

	drv.raise_error();
	CHECK_EQUAL(1, drv.blocking_off_calls);
	CHECK_FALSE(drv.is_powered_on());

	// The OFF that was armed against a filming camera is dropped; the next
	// event is a power-on after a full OFF period, not a no-op toggle.
	run_for(ON_WAIT_MS);
	CHECK_EQUAL(0, drv.off_requests);
	CHECK_EQUAL(1, drv.on_requests);
	run_for(OFF_WAIT_MS - ON_WAIT_MS);
	CHECK_EQUAL(2, drv.on_requests);

	s.stop();
}

TEST(CAMService, LowBatteryFollowsLbCamEn) {
	FakeCAMDevice drv;
	CAMService s(drv, fake_logger);
	configuration_store->write_param(ParamID::LB_EN, true);
	configuration_store->write_param(ParamID::LB_CAM_EN, false);
	fake_battery_monitor->set_values(/*level*/ 5, /*mv*/ 3300, /*is_low*/ true);

	// Low battery, LB_CAM_EN off (the default): the camera stays off.
	start(s);
	run_for(2 * OFF_WAIT_MS);
	CHECK_EQUAL(0, drv.on_requests);
	s.stop();

	// Same battery, LB_CAM_EN on: the camera runs through low battery.
	configuration_store->write_param(ParamID::LB_CAM_EN, true);
	start(s);
	run_for(OFF_WAIT_MS);
	CHECK_EQUAL(1, drv.on_requests);
	s.stop();
}
