#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

#include "mock_kineis_device.hpp"
#include "fake_rtc.hpp"
#include "fake_config_store.hpp"
#include "fake_timer.hpp"
#include "fake_battery_mon.hpp"
#include "scheduler.hpp"
#include "argos_rx_service.hpp"

extern Timer *system_timer;
extern ConfigurationStore *configuration_store;
extern Scheduler *system_scheduler;
extern RTC *rtc;
extern BatteryMonitor *battery_monitor;

/*
 * ArgosRxService arrived with the PREPASS v4.0 / Kineis downlink integration and
 * had no unit test at all -- it was compiled into the suite and never exercised.
 * It is the one service that POWERS THE RECEIVER for minutes at a time
 * (ARGOS_RX_MAX_WINDOW defaults to 15 minutes), so what it refuses to do matters
 * as much as what it does.
 */
TEST_GROUP(ArgosRxService) {
	FakeBatteryMonitor *fake_battery_monitor;
	FakeConfigurationStore *fake_config_store;
	MockKineisDevice *mock_kineis;
	FakeRTC *fake_rtc;
	FakeTimer *fake_timer;

	void setup() {
		ServiceManager::reset();
		fake_battery_monitor = new FakeBatteryMonitor;
		battery_monitor = fake_battery_monitor;
		mock_kineis = new MockKineisDevice;
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
	}

	void teardown() {
		delete system_scheduler;
		delete fake_timer;
		delete fake_rtc;
		delete fake_config_store;
		delete mock_kineis;
		delete fake_battery_monitor;
		system_scheduler = nullptr;
		system_timer = nullptr;
		rtc = nullptr;
		configuration_store = nullptr;
		battery_monitor = nullptr;
	}

	/// @brief Put the store in the one configuration where RX is meant to run.
	void configure_rx_enabled() {
		fake_config_store->write_param(ParamID::ARGOS_RX_EN, true);
		fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::PASS_PREDICTION);
		fake_config_store->write_param(ParamID::CERT_TX_ENABLE, false);
		fake_config_store->write_param(ParamID::LB_EN, false);
	}

	/// @brief Reads the enable decision directly.
	///
	/// Going through start() and get_last_schedule() cannot tell "refused by the
	/// gate" from "willing, but no satellite pass to schedule yet" -- both leave
	/// SCHEDULE_DISABLED. A test that cannot tell those apart would pass on a
	/// service that refuses everything. service_is_enabled() is protected, so a
	/// test-only subclass reads it for what it is.
	class Testable : public ArgosRxService {
	public:
		explicit Testable(KineisDevice &d) : ArgosRxService(d) {}
		bool enabled() { return service_is_enabled(); }
	};
};

TEST(ArgosRxService, DisabledWhenArgosRxEnIsFalse) {
	configure_rx_enabled();
	fake_config_store->write_param(ParamID::ARGOS_RX_EN, false);
	Testable serv(*mock_kineis);
	CHECK_FALSE(serv.enabled());
	mock().checkExpectations();
}

TEST(ArgosRxService, DisabledOutsidePassPredictionMode) {
	/*
	 * The downlink window is computed from the satellite pass table, so it only
	 * means anything in PASS_PREDICTION. In LEGACY or DUTY_CYCLE there is no
	 * pass to align on and opening a receive window would just burn current.
	 */
	configure_rx_enabled();
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::LEGACY);
	Testable serv(*mock_kineis);
	CHECK_FALSE(serv.enabled());
	mock().checkExpectations();
}

TEST(ArgosRxService, DisabledDuringCertification) {
	/*
	 * Certification drives the transmitter under a controlled protocol. A
	 * receive window opening underneath it would take the module away
	 * mid-measurement and invalidate the run.
	 */
	configure_rx_enabled();
	fake_config_store->write_param(ParamID::CERT_TX_ENABLE, true);
	Testable serv(*mock_kineis);
	CHECK_FALSE(serv.enabled());
	mock().checkExpectations();
}

TEST(ArgosRxService, DisabledOnLowBatteryEvenWhenLbModeIsPassPrediction) {
	/*
	 * THE case this file was written for.
	 *
	 * Low battery substitutes ARGOS_MODE with LB_ARGOS_MODE, whose default is
	 * LEGACY -- so RX stops, but only as a SIDE EFFECT of the mode no longer
	 * being PASS_PREDICTION. LB_ARGOS_MODE accepts PASS_PREDICTION (value 1)
	 * like any other mode, and choosing it is not far-fetched: "keep prepass on
	 * low battery, it transmits less". With that setting the incidental
	 * protection disappears and the receiver powers up for ARGOS_RX_MAX_WINDOW
	 * -- fifteen minutes by default -- on a battery already declared low.
	 *
	 * The whole point of the LB profile is to do LESS. This asserts the gate is
	 * explicit rather than accidental.
	 */
	configure_rx_enabled();
	fake_config_store->write_param(ParamID::LB_EN, true);
	fake_config_store->write_param(ParamID::LB_ARGOS_MODE, BaseArgosMode::PASS_PREDICTION);
	fake_battery_monitor->set_values(/*level*/ 1, /*mv*/ 3200, /*is_low*/ true);
	// The store latches the low-battery verdict; the fake exposes it directly
	// rather than making the test drive a sampling cycle.
	fake_config_store->set_is_battery_level_low(true);

	Testable serv(*mock_kineis);
	CHECK_FALSE(serv.enabled());
	mock().checkExpectations();
}

TEST(ArgosRxService, EnabledInNominalConfiguration) {
	/*
	 * The counterpart of the four refusals above: with everything in its
	 * intended state the service must actually arm. Without this, a gate that
	 * refused unconditionally would pass every other test in this file.
	 */
	configure_rx_enabled();
	fake_battery_monitor->set_values(/*level*/ 100, /*mv*/ 4200, /*is_low*/ false);
	Testable serv(*mock_kineis);
	CHECK_TRUE(serv.enabled());
	mock().checkExpectations();
}
