/**
 * @file battery_sampling_test.cpp
 * @brief When BatteryMonitor::update() takes a sample, and when it reuses one.
 *
 * Every get_argos_configuration() / get_gnss_configuration() refreshes the
 * battery, and the services call them at each scheduling step. With a sample
 * per call, a simulated turtle day took 4023 samples, 13 of them between the
 * surfacing and the first frame -- each a 500 ms busy-wait on KIM before the
 * conversion. The rule under test: a routine refresh reuses a sample younger
 * than SAMPLE_MAX_AGE_MS; a forced read, a younger max age asked by the caller,
 * a changed threshold pair or a rejected attempt sample sooner.
 */

#include "battery.hpp"
#include "fake_battery_mon.hpp"
#include "fake_timer.hpp"

#include "CppUTest/TestHarness.h"

extern Timer *system_timer;

namespace {
/// Records the critical event, which must not depend on which call sampled.
class CriticalListener : public BatteryMonitorEventListener {
public:
	unsigned int events = 0;
	void react(BatteryMonitorEventVoltageCritical const &) override { events++; }
};

/// A monitor whose attempts fail, like an ADC conversion reading 0 mV.
class RejectingBatteryMonitor : public BatteryMonitor {
public:
	RejectingBatteryMonitor() : BatteryMonitor(10, 5) {}
	unsigned int attempts = 0;
	bool reject = true;

private:
	void internal_update() override {
		attempts++;
		m_sample_rejected = reject;
	}
};

constexpr uint64_t MINUTE_MS = 60ULL * 1000ULL;
}  // namespace

TEST_GROUP(BatterySampling) {
	FakeTimer *fake_timer;
	FakeBatteryMonitor *battery;

	void setup() {
		fake_timer = new FakeTimer;
		system_timer = fake_timer;
		fake_timer->start();
		fake_timer->set_counter(1000);
		battery = new FakeBatteryMonitor;
	}

	void teardown() {
		delete battery;
		system_timer = nullptr;
		delete fake_timer;
	}

	void advance(uint64_t ms) {
		fake_timer->set_counter(fake_timer->get_counter() + ms);
	}
};

TEST(BatterySampling, FirstUpdateSamples) {
	battery->update();
	CHECK_EQUAL(1U, battery->update_calls);
}

TEST(BatterySampling, YoungSampleIsReused) {
	battery->update();
	advance(BatteryMonitor::SAMPLE_MAX_AGE_MS - 1);
	for (int i = 0; i < 50; i++)
		battery->update();
	CHECK_EQUAL(1U, battery->update_calls);
}

TEST(BatterySampling, SampleOlderThanMaxAgeIsRefreshed) {
	battery->update();
	advance(BatteryMonitor::SAMPLE_MAX_AGE_MS);
	battery->update();
	CHECK_EQUAL(2U, battery->update_calls);
}

TEST(BatterySampling, CallerCanAskForAYoungerSample) {
	battery->update();
	advance(10000);
	battery->update(10000);
	CHECK_EQUAL(2U, battery->update_calls);
	advance(9999);
	battery->update(10000);
	CHECK_EQUAL(2U, battery->update_calls);
}

TEST(BatterySampling, ForcedReadAlwaysSamples) {
	battery->update();
	CHECK_TRUE(battery->update_forced());
	CHECK_TRUE(battery->update_forced());
	CHECK_EQUAL(3U, battery->update_calls);
	// ...and counts as the latest sample for the routine refresh.
	advance(MINUTE_MS);
	battery->update();
	CHECK_EQUAL(3U, battery->update_calls);
}

// update(0) is what Service::service_update_battery_now() asks for: the CLS
// woodbuoy's fresh sample at every GNSS session (BATTERY_SAMPLE_EACH_GNSS_SESSION).
TEST(BatterySampling, ZeroMaxAgeAlwaysSamples) {
	battery->update();
	battery->update(0);
	battery->update(0);
	CHECK_EQUAL(3U, battery->update_calls);
	// ...a routine refresh right after reuses it, as after a forced read.
	battery->update();
	CHECK_EQUAL(3U, battery->update_calls);
}

TEST(BatterySampling, ChangedThresholdsSampleAtOnceSamePairDoesNot) {
	battery->update();
	battery->set_thresholds(10, 5);  // the pair the monitor was built with
	battery->update();
	CHECK_EQUAL(1U, battery->update_calls);
	battery->set_thresholds(30, 5);
	battery->update();
	CHECK_EQUAL(2U, battery->update_calls);
	battery->update();
	CHECK_EQUAL(2U, battery->update_calls);
}

TEST(BatterySampling, RejectedAttemptIsRetriedSoonNotEveryCall) {
	RejectingBatteryMonitor rejecting;
	rejecting.update();
	CHECK_EQUAL(1U, rejecting.attempts);
	for (int i = 0; i < 20; i++)
		rejecting.update();  // no busy-wait storm on a broken ADC
	CHECK_EQUAL(1U, rejecting.attempts);
	advance(5 * MINUTE_MS);
	rejecting.reject = false;
	rejecting.update();
	CHECK_EQUAL(2U, rejecting.attempts);
	// A good sample brings the routine age back.
	advance(5 * MINUTE_MS);
	rejecting.update();
	CHECK_EQUAL(2U, rejecting.attempts);
}

TEST(BatterySampling, CriticalEventFiresEvenWhenTheSampleIsReused) {
	CriticalListener listener;
	battery->subscribe(listener);
	battery->update();
	battery->set_values(3, 3000, true, true);
	battery->update();  // young sample reused: the edge is still seen
	CHECK_EQUAL(1U, battery->update_calls);
	CHECK_EQUAL(1U, listener.events);
	battery->update();
	CHECK_EQUAL(1U, listener.events);  // one event per falling edge
	battery->unsubscribe(listener);
}
