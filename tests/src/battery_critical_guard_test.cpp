/**
 * @file battery_critical_guard_test.cpp
 * @brief The one guard that can end a mission: Service::shutdown_if_battery_critical.
 *
 * ArgosTxService and LoRaTxService each carried their own copy of this block,
 * and the copies had already diverged with consequences: the hardening that
 * made the check require a CONFIRMED verdict went into ArgosTxService and was
 * forgotten in LoRaTxService, so LoRa tags went on powering down on a
 * transmit-current transient. Neither copy had a test.
 *
 * It calls PMU::powerdown() on a device whose reed switch is under the glue, so
 * the case that matters is the one that must NOT fire: a single low reading.
 */

#include "service.hpp"
#include "fake_battery_mon.hpp"
#include "fake_config_store.hpp"
#include "fake_rtc.hpp"
#include "fake_timer.hpp"

#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

extern Timer *system_timer;
extern ConfigurationStore *configuration_store;
extern BatteryMonitor *battery_monitor;
extern RTC *rtc;

namespace {
/// Exposes the protected guard, and nothing else.
class GuardProbe : public Service {
public:
	GuardProbe() : Service(ServiceIdentifier::UNKNOWN, "GuardProbe") {}
	using Service::shutdown_if_battery_critical;

private:
	void service_init() override {}
	void service_term() override {}
	bool service_is_enabled() override { return false; }
	unsigned int service_next_schedule_in_ms() override { return SCHEDULE_DISABLED; }
	void service_initiate() override {}
};
}  // namespace

TEST_GROUP(BatteryCriticalGuard) {
	FakeConfigurationStore *fake_config_store;
	FakeBatteryMonitor *fake_battery;
	FakeTimer *fake_timer;
	FakeRTC *fake_rtc;
	GuardProbe *probe;

	void setup() {
		ServiceManager::reset();
		fake_timer = new FakeTimer;
		system_timer = fake_timer;
		fake_rtc = new FakeRTC;
		rtc = fake_rtc;
		fake_rtc->settime(1580083200);
		fake_config_store = new FakeConfigurationStore;
		fake_config_store->init();
		configuration_store = fake_config_store;
		fake_battery = new FakeBatteryMonitor;
		battery_monitor = fake_battery;
		probe = new GuardProbe;

		unsigned int critical = 10;
		configuration_store->write_param(ParamID::LB_CRITICAL_THRESH, critical);
	}

	void teardown() {
		delete probe;
		delete fake_battery;
		delete fake_config_store;
		delete fake_rtc;
		delete fake_timer;
		mock().checkExpectations();
		mock().clear();
	}
};

/// Outside low-battery mode the guard does nothing at all -- not even a sample.
/// The flag the callers pass is `LB_EN && SOC < LB_THRESHOLD`, so this is the
/// state of every tag on a default config, where LB_EN is false.
TEST(BatteryCriticalGuard, outside_low_battery_mode_it_does_not_even_sample) {
	mock().expectNoCall("powerdown");
	fake_battery->set_values(1, 3000, true, true);  // as bad as it gets

	CHECK_FALSE(probe->shutdown_if_battery_critical(false).has_value());
	CHECK_EQUAL(0u, fake_battery->update_calls);
}

TEST(BatteryCriticalGuard, a_healthy_battery_carries_on) {
	mock().expectNoCall("powerdown");
	fake_battery->set_values(80, 4000, false, false);

	CHECK_FALSE(probe->shutdown_if_battery_critical(true).has_value());
	CHECK_EQUAL(1u, fake_battery->update_calls);
}

/// THE case. The state of charge is under the threshold but the monitor has not
/// confirmed it over BATT_CONFIRM_SAMPLES consecutive readings, which is what a
/// transmit-current spike or a cold boot looks like. Shutting down here is the
/// defect that reached the field.
TEST(BatteryCriticalGuard, an_unconfirmed_low_reading_does_not_power_down) {
	mock().expectNoCall("powerdown");
	fake_battery->set_values(3, 3300, true, /* is_critical (confirmed) */ false);

	CHECK_FALSE(probe->shutdown_if_battery_critical(true).has_value());
}

/// The mirror case: confirmed critical, but the level is at or above the
/// configured threshold. Both conditions are required.
TEST(BatteryCriticalGuard, confirmed_critical_above_the_threshold_does_not_power_down) {
	mock().expectNoCall("powerdown");
	fake_battery->set_values(10, 3300, true, true);  // threshold is 10, so 10 is not below it

	CHECK_FALSE(probe->shutdown_if_battery_critical(true).has_value());
}

TEST(BatteryCriticalGuard, confirmed_critical_below_the_threshold_powers_down) {
	mock().expectOneCall("powerdown");
	fake_battery->set_values(3, 3300, true, true);

	auto decision = probe->shutdown_if_battery_critical(true);
	CHECK_TRUE(decision.has_value());
	// Returned so the caller stops scheduling: on a real tag powerdown() has
	// already happened by now, but the bench builds suppress it and the service
	// would otherwise go on planning transmissions.
	CHECK_FALSE(decision->comes_back_on_its_own());
}

/// A flash that refuses the save before the cut must not stop the cut: on RSPB
/// the MCU_DONE pulse in powerdown() is what makes the TPL5111 cut the power.
TEST(BatteryCriticalGuard, a_failed_save_before_the_cut_still_powers_down) {
	mock().expectOneCall("powerdown");
	fake_battery->set_values(3, 3300, true, true);
	fake_config_store->set_fail_saves(true);

	CHECK_TRUE(probe->shutdown_if_battery_critical(true).has_value());
}

/// The threshold is read on every call, not captured once: LB_CRITICAL_THRESH
/// is writable over DTE and a sealed tag cannot be rebooted to pick it up.
TEST(BatteryCriticalGuard, the_threshold_is_read_fresh_each_time) {
	mock().expectNoCall("powerdown");
	fake_battery->set_values(15, 3600, true, true);
	CHECK_FALSE(probe->shutdown_if_battery_critical(true).has_value());

	mock().checkExpectations();
	mock().clear();
	mock().expectOneCall("powerdown");

	unsigned int raised = 20;
	configuration_store->write_param(ParamID::LB_CRITICAL_THRESH, raised);
	CHECK_TRUE(probe->shutdown_if_battery_critical(true).has_value());
}
