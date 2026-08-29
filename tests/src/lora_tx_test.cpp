#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

#include "mock_kineis_device.hpp"
#include "fake_rtc.hpp"
#include "fake_config_store.hpp"
#include "fake_timer.hpp"
#include "fake_battery_mon.hpp"
#include "timeutils.hpp"
#include "scheduler.hpp"
#include "lora_tx_service.hpp"

extern Timer *system_timer;
extern ConfigurationStore *configuration_store;
extern Scheduler *system_scheduler;
extern RTC *rtc;
extern BatteryMonitor *battery_monitor;

/// LoRaTxService has been compiled into TrackerTests for some time
/// (tests/CMakeLists.txt) without a single test to its name -- including the
/// device-error suspension that ArgosTxService is being asked to copy, and
/// whose header records an 18 h field outage as the reason it exists.
///
/// This group covers that machinery only: the backoff below the strike limit,
/// the time-bounded suspension above it, the single probe dispatch that ends
/// the suspension, and the events that end it sooner. Packet content, jitter
/// and the burst modes are deliberately out of scope here.
///
/// Every test below counts `send` strictly and calls mock().ignoreOtherCalls()
/// for the rest of the device chatter. That is deliberate and it is not a
/// weakening: CppUTest still fails a SURPLUS call to a function that carries
/// expectations, so "expect exactly N sends" remains a proof that no further
/// transmission occurred -- while stop_send/power_off counts, which depend on
/// the wake cadence rather than on the behaviour under test, stay out of it.
TEST_GROUP(LoRaTxService) {
	FakeBatteryMonitor *fake_battery_monitor;
	FakeConfigurationStore *fake_config_store;
	MockKineisDevice *mock_device;
	FakeRTC *fake_rtc;
	FakeTimer *fake_timer;

	void setup() {
		// Same clean-slate discipline as the Argos group: ServiceManager holds
		// static registration and cooldown state that would otherwise leak
		// between tests in the same process.
		ServiceManager::reset();
		fake_battery_monitor = new FakeBatteryMonitor;
		battery_monitor = fake_battery_monitor;
		mock_device = new MockKineisDevice;
		fake_config_store = new FakeConfigurationStore;
		configuration_store = fake_config_store;
		configuration_store->init();
		fake_rtc = new FakeRTC;
		rtc = fake_rtc;
		fake_rtc->settime(1580083200);  // 27/01/2020 00:00:00
		fake_timer = new FakeTimer;
		system_timer = fake_timer;
		system_scheduler = new Scheduler(system_timer);
		fake_timer->start();

		unsigned int no_cooldown = 0U;
		fake_config_store->write_param(ParamID::MIN_SURFACE_CYCLE_INTERVAL_S, no_cooldown);
	}

	void teardown() {
		delete system_scheduler;
		delete fake_timer;
		delete fake_rtc;
		delete fake_config_store;
		delete mock_device;
		delete fake_battery_monitor;
		system_scheduler = nullptr;
		system_timer = nullptr;
		rtc = nullptr;
		configuration_store = nullptr;
		battery_monitor = nullptr;
	}

	GPSLogEntry make_gps_location(std::time_t t) {
		GPSLogEntry log{};
		log.info.valid = true;
		log.info.lon = 11.8768;
		log.info.lat = -33.8232;
		log.info.fixType = 3;
		log.info.batt_voltage = 4200;
		log.info.schedTime = t;

		uint16_t year;
		uint8_t month, day, hour, min, sec;
		convert_datetime_to_epoch(t, year, month, day, hour, min, sec);
		log.header.year = log.info.year = year;
		log.header.month = log.info.month = month;
		log.header.day = log.info.day = day;
		log.header.hours = log.info.hour = hour;
		log.header.minutes = log.info.min = min;
		log.header.seconds = log.info.sec = sec;
		return log;
	}

	void inject_gps_location(std::time_t t) {
		ServiceEvent e;
		e.event_source = ServiceIdentifier::GNSS_SENSOR;
		e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
		e.event_data = make_gps_location(t);
		e.event_originator_unique_id = 0x12345678;
		ServiceManager::notify_peer_event(e);
		configuration_store->notify_gps_location(std::get<GPSLogEntry>(e.event_data));
	}

	void notify_underwater_state(bool state) {
		ServiceEvent e;
		e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
		e.event_data = state;
		e.event_source = ServiceIdentifier::UW_SENSOR;
		e.event_originator_unique_id = 0x12345678;
		ServiceManager::notify_peer_event(e);
	}

	void configure_plain_legacy(unsigned int tr_nom_s = 10) {
		fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::LEGACY);
		fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);
		fake_config_store->write_param(ParamID::GNSS_EN, true);
		fake_config_store->write_param(ParamID::LB_EN, false);
		fake_config_store->write_param(ParamID::TR_NOM, tr_nom_s);
		fake_config_store->write_param(ParamID::ARGOS_TX_JITTER_EN, false);
	}

	void configure_surfacing_burst(unsigned int max_msg = 0) {
		fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::SURFACING_BURST);
		fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);
		fake_config_store->write_param(ParamID::GNSS_EN, true);
		fake_config_store->write_param(ParamID::LB_EN, false);
		fake_config_store->write_param(ParamID::TR_NOM, (unsigned int)60);
		fake_config_store->write_param(ParamID::ARGOS_TX_JITTER_EN, false);
		fake_config_store->write_param(ParamID::UNDERWATER_EN, (bool)true);
		fake_config_store->write_param(ParamID::DRY_TIME_BEFORE_TX, (unsigned int)0);
		fake_config_store->write_param(ParamID::SURFACING_BURST_INIT_S, (unsigned int)5);
		fake_config_store->write_param(ParamID::SURFACING_BURST_STEP_S, (unsigned int)10);
		fake_config_store->write_param(ParamID::SURFACING_BURST_MAX_S, (unsigned int)60);
		fake_config_store->write_param(ParamID::SURFACING_BURST_MAX_MSG, max_msg);
	}

	/// Drive `count` failed transmissions. Expectations must already be set.
	void fail_n_transmissions(LoRaTxService & serv, std::time_t & t, unsigned int count) {
		for (unsigned int i = 0; i < count; i++) {
			t += serv.get_last_schedule();
			fake_rtc->settime(t / 1000);
			fake_timer->set_counter(t);
			system_scheduler->run();
			mock_device->notify(KineisEventDeviceError({}));
		}
	}
};

// Below the strike limit LoRa behaves exactly like Argos: a capped doubling
// backoff, applied by the service rescheduling itself. No peer event needed --
// which matters because a moored/periodic LoRa tracker has no surfacing events
// at all.
TEST(LoRaTxService, DeviceErrorBelowThresholdBacksOffAndReschedulesItself) {
	configure_plain_legacy();

	LoRaTxService serv(*mock_device);
	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	serv.start();
	inject_gps_location(t / 1000);

	mock().expectNCalls(2, "send").onObject(mock_device).ignoreOtherParameters();
	mock().ignoreOtherCalls();

	fail_n_transmissions(serv, t, 1);
	CHECK_EQUAL(60000U, serv.get_last_schedule());

	fail_n_transmissions(serv, t, 1);
	CHECK_EQUAL(120000U, serv.get_last_schedule());
	mock().checkExpectations();
}

// At the strike limit LoRa arms a deadline instead of stopping dead. While it
// holds, service_initiate refuses to dispatch -- so no `send` may occur, and
// the strict mock enforces that.
TEST(LoRaTxService, SuspensionHoldsForTheProbePeriod) {
	configure_plain_legacy();

	LoRaTxService serv(*mock_device);
	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	serv.start();
	inject_gps_location(t / 1000);

	// Three failures, then ten wake-ups spread over half an hour -- well
	// inside the one-hour probe period, so none of them may transmit.
	mock().expectNCalls(3, "send").onObject(mock_device).ignoreOtherParameters();
	mock().ignoreOtherCalls();

	fail_n_transmissions(serv, t, 3);

	for (unsigned int k = 0; k < 10; k++) {
		t += 180000;  // 3 min
		fake_rtc->settime(t / 1000);
		fake_timer->set_counter(t);
		system_scheduler->run();
	}
	mock().checkExpectations();
}

// ...and the deadline expires. This is the whole point of the design, and the
// half Argos does not have: one dispatch is let through as a probe, without a
// reboot, a surfacing, or a GPS session.
TEST(LoRaTxService, ProbeTransmitsOnceTheSuspensionElapses) {
	configure_plain_legacy();

	LoRaTxService serv(*mock_device);
	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	serv.start();
	inject_gps_location(t / 1000);

	// Three failures, then the probe.
	mock().expectNCalls(3 + 1, "send").onObject(mock_device).ignoreOtherParameters();
	mock().ignoreOtherCalls();

	fail_n_transmissions(serv, t, 3);

	// Past DEVICE_ERROR_PROBE_PERIOD_S (LORA_TX_ERROR_SUSPEND_S, 3600 s by
	// default, which is what the host build gets).
	t += 3700000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();

	// The probe failed too, so the suspension is re-armed rather than lifted:
	// the next wake-ups must not transmit again.
	mock_device->notify(KineisEventDeviceError({}));

	for (unsigned int k = 0; k < 5; k++) {
		t += 180000;
		fake_rtc->settime(t / 1000);
		fake_timer->set_counter(t);
		system_scheduler->run();
	}
	mock().checkExpectations();
}

// A probe that succeeds clears the count outright, so the service returns to
// its nominal cadence instead of creeping back through the backoff ladder.
TEST(LoRaTxService, SuccessfulProbeClearsTheSuspensionCompletely) {
	configure_plain_legacy();

	LoRaTxService serv(*mock_device);
	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	serv.start();
	inject_gps_location(t / 1000);

	mock().expectNCalls(3 + 1 + 1, "send").onObject(mock_device).ignoreOtherParameters();
	mock().ignoreOtherCalls();

	fail_n_transmissions(serv, t, 3);

	// The probe, and it works.
	t += 3700000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock_device->notify(KineisEventTxComplete({}));

	// Back to the nominal period, not to another suspension.
	CHECK_COMPARE(serv.get_last_schedule(), <, 60000U);

	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock().checkExpectations();
}

// A surfacing tracker does not have to wait out the hour. The UW branch clears
// the count and the base class reschedules, exactly as on Argos.
TEST(LoRaTxService, SurfaceEventEndsTheSuspensionEarly) {
	configure_plain_legacy();
	fake_config_store->write_param(ParamID::DRY_TIME_BEFORE_TX, (unsigned int)0);

	LoRaTxService serv(*mock_device);
	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	serv.start();
	inject_gps_location(t / 1000);

	mock().expectNCalls(3 + 1, "send").onObject(mock_device).ignoreOtherParameters();
	mock().ignoreOtherCalls();

	fail_n_transmissions(serv, t, 3);

	// Down and up, well inside the probe period.
	notify_underwater_state(true);
	t += 600000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	notify_underwater_state(false);

	t += 60000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock().checkExpectations();
}

// "Consecutive" must mean consecutive here too: a transmission that works
// resets the ladder, so failures separated by successes cannot accumulate
// into a suspension.
TEST(LoRaTxService, SuccessfulTxClearsTheErrorCount) {
	configure_plain_legacy();

	LoRaTxService serv(*mock_device);
	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	serv.start();
	inject_gps_location(t / 1000);

	mock().expectNCalls(3, "send").onObject(mock_device).ignoreOtherParameters();
	mock().ignoreOtherCalls();

	fail_n_transmissions(serv, t, 1);
	CHECK_EQUAL(60000U, serv.get_last_schedule());

	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock_device->notify(KineisEventTxComplete({}));

	fail_n_transmissions(serv, t, 1);
	CHECK_EQUAL(60000U, serv.get_last_schedule());
	mock().checkExpectations();
}

// SURFACING_BURST, freshly deployed, no dive yet: m_awaiting_surfacing is still
// false and m_is_surfacing_burst has never been set, so the mode used to fall
// off the end of the dispatch and answer SCHEDULE_DISABLED. A tag was therefore
// mute from power-on until its first dive -- and on a mooring or a slow animal
// that can be days, indistinguishable from a dead radio. Argos has always sent
// Doppler at the legacy rate in exactly this situation; LoRa now sends status.
TEST(LoRaTxService, SurfacingBurstBeforeAnyDiveStillSendsAStatusHeartbeat) {
	configure_surfacing_burst();

	LoRaTxService serv(*mock_device);
	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("send").onObject(mock_device).ignoreOtherParameters();
	mock().ignoreOtherCalls();

	serv.start();

	// A real delay, not the sentinel: something is planned.
	CHECK(serv.get_last_schedule() != Service::SCHEDULE_DISABLED);

	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();

	mock().checkExpectations();
}

// End of a status burst on SURFACING_BURST_MAX_MSG. The service sets
// m_awaiting_surfacing and used to answer SCHEDULE_DISABLED in the same breath
// -- so the heartbeat that flag is supposed to allow only materialised if
// something else happened to re-evaluate the service, and that flag is
// precisely what stops anything else from happening. The heartbeat is now
// scheduled in the same pass.
TEST(LoRaTxService, BurstEndingOnMaxMessagesKeepsTheHeartbeatInTheSamePass) {
	configure_surfacing_burst(2);  // two status messages, then the burst is over

	LoRaTxService serv(*mock_device);
	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	// Two burst messages, then one heartbeat at TR_NOM. A third send is what
	// proves the heartbeat was scheduled in the same pass as the burst end;
	// a strict count is also what would catch the burst running on.
	mock().expectNCalls(3, "send").onObject(mock_device).ignoreOtherParameters();
	mock().ignoreOtherCalls();

	// A cached position, put straight into the store without notifying the
	// service: process_status_burst silences the whole burst after its first
	// ping when it has nothing at all to say ("STATUS-PURE ... silencing
	// further pings"), which would end the burst long before max_msg. The
	// cache keeps the progressive burst alive; not notifying the service keeps
	// m_has_gnss_fix_since_surfacing false, so we stay in the status phase.
	GPSLogEntry cached = make_gps_location(t / 1000);
	cached.info.event_type = GPSEventType::FIX;
	configuration_store->notify_gps_location(cached);

	serv.start();
	notify_underwater_state(true);
	notify_underwater_state(false);  // surfacing -> status burst begins

	// Burn the two allowed status messages.
	for (unsigned int k = 0; k < 2; k++) {
		t += serv.get_last_schedule();
		fake_rtc->settime(t / 1000);
		fake_timer->set_counter(t);
		system_scheduler->run();
		mock_device->notify(KineisEventTxComplete({}));
	}

	// The burst is over and no surfacing has happened since. Walk past TR_NOM
	// on the wall clock -- deliberately NOT via get_last_schedule(), which only
	// a Run decision updates and which would therefore read as a stale delay
	// exactly in the case this test exists to catch.
	t += 120000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock_device->notify(KineisEventTxComplete({}));

	mock().checkExpectations();
}
