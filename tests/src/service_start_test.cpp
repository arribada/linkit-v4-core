#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

#include "service.hpp"
#include "scheduler.hpp"
#include "error.hpp"
#include "fake_config_store.hpp"
#include "fake_rtc.hpp"
#include "fake_timer.hpp"

extern Timer *system_timer;
extern ConfigurationStore *configuration_store;
extern Scheduler *system_scheduler;
extern RTC *rtc;

/// A service that can be told to fail its start, or its peer handler.
class FlakyService : public Service {
public:
	explicit FlakyService(const char *name) : Service(ServiceIdentifier::UNKNOWN, name) {}

	unsigned int init_calls = 0;
	unsigned int term_calls = 0;
	unsigned int init_failures_left = 0;  ///< next service_init() calls that throw
	bool init_always_fails = false;
	unsigned int peer_events = 0;
	bool peer_fails = false;

	void notify_peer_event(ServiceEvent &) override {
		peer_events++;
		if (peer_fails) throw ErrorCode::RESOURCE_NOT_AVAILABLE;
	}

private:
	void service_init() override {
		init_calls++;
		if (init_always_fails) throw ErrorCode::RESOURCE_NOT_AVAILABLE;
		if (init_failures_left) {
			init_failures_left--;
			throw std::runtime_error("init failed");
		}
	}
	void service_term() override { term_calls++; }
	bool service_is_enabled() override { return false; }
	unsigned int service_next_schedule_in_ms() override { return SCHEDULE_DISABLED; }
	void service_initiate() override {}
};

TEST_GROUP(ServiceStart) {
	FakeConfigurationStore *fake_config_store;
	FakeTimer *fake_timer;
	FakeRTC *fake_rtc;

	void setup() {
		ServiceManager::reset();
		fake_timer = new FakeTimer;
		system_timer = fake_timer;
		fake_rtc = new FakeRTC;
		rtc = fake_rtc;
		fake_rtc->settime(1580083200);
		fake_config_store = new FakeConfigurationStore;
		configuration_store = fake_config_store;
		configuration_store->init();
		system_scheduler = new Scheduler(system_timer);
		fake_timer->start();
	}

	void teardown() {
		ServiceManager::reset();
		delete system_scheduler;
		delete fake_timer;
		delete fake_config_store;
		delete fake_rtc;
		system_scheduler = nullptr;
		system_timer = nullptr;
		rtc = nullptr;
		configuration_store = nullptr;
	}

	/// Advance the clock in 100 ms steps, running the scheduler at each one.
	void run_until(uint64_t t_ms) {
		while (fake_timer->get_counter() < t_ms) {
			const uint64_t next = fake_timer->get_counter() + 100;
			fake_timer->set_counter(next < t_ms ? next : t_ms);
			system_scheduler->run();
		}
	}
};

TEST(ServiceStart, ThrowingStartDoesNotStopTheOthers) {
	FlakyService a("A"), b("B"), c("C");
	b.init_always_fails = true;

	ServiceManager::startall();

	CHECK_TRUE(a.is_started());
	CHECK_FALSE(b.is_started());
	CHECK_TRUE(c.is_started());
	CHECK_EQUAL(1u, c.init_calls);
	// The half-done start is undone like a stop: its term ran once.
	CHECK_EQUAL(1u, b.term_calls);
	CHECK_EQUAL(0u, a.term_calls);
}

TEST(ServiceStart, FailedStartIsRetriedWithGrowingDelayUntilItWorks) {
	FlakyService b("B");
	b.init_failures_left = 3;

	CHECK_FALSE(b.start_or_retry());
	CHECK_EQUAL(1u, b.init_calls);

	// Retries 5 s, 10 s, then 20 s after each failure.
	run_until(4900);
	CHECK_EQUAL(1u, b.init_calls);
	run_until(5000);
	CHECK_EQUAL(2u, b.init_calls);
	run_until(14900);
	CHECK_EQUAL(2u, b.init_calls);
	run_until(15000);
	CHECK_EQUAL(3u, b.init_calls);
	CHECK_FALSE(b.is_started());
	run_until(34900);
	CHECK_EQUAL(3u, b.init_calls);
	run_until(35000);
	CHECK_EQUAL(4u, b.init_calls);
	CHECK_TRUE(b.is_started());

	// Started: no further attempt.
	run_until(4 * 3600 * 1000);
	CHECK_EQUAL(4u, b.init_calls);
}

TEST(ServiceStart, FailedStartIsNeverAbandoned) {
	FlakyService b("B");
	b.init_always_fails = true;
	b.start_or_retry();

	// The delay stops growing at one hour: every later hour still tries.
	run_until(24ull * 3600 * 1000);
	const unsigned int calls_after_a_day = b.init_calls;
	CHECK_TRUE(calls_after_a_day > 10);
	run_until(26ull * 3600 * 1000);
	CHECK_EQUAL(calls_after_a_day + 2, b.init_calls);
	CHECK_EQUAL(b.init_calls, b.term_calls);
}

TEST(ServiceStart, StopCancelsThePendingRetry) {
	FlakyService b("B");
	b.init_always_fails = true;
	ServiceManager::startall();
	CHECK_EQUAL(1u, b.init_calls);

	// Leaving Operational must not leave a start armed behind it.
	ServiceManager::stopall();
	run_until(3 * 3600 * 1000);
	CHECK_EQUAL(1u, b.init_calls);
}

TEST(ServiceStart, NewStartallBeginsAgainFromTheShortestDelay) {
	FlakyService b("B");
	b.init_always_fails = true;
	ServiceManager::startall();
	run_until(5000 + 10000 + 20000);
	CHECK_EQUAL(4u, b.init_calls);
	ServiceManager::stopall();

	const uint64_t t0 = fake_timer->get_counter();
	ServiceManager::startall();
	CHECK_EQUAL(5u, b.init_calls);
	run_until(t0 + 5000);
	CHECK_EQUAL(6u, b.init_calls);
}

TEST(ServiceStart, ThrowingPeerDoesNotStopTheBroadcast) {
	FlakyService origin("ORIGIN"), p1("P1"), p2("P2"), p3("P3");
	p2.peer_fails = true;

	ServiceEvent e;
	e.event_type = ServiceEventType::SERVICE_ACTIVE;
	e.event_source = ServiceIdentifier::ARGOS_TX;
	e.event_originator_unique_id = origin.get_unique_id();
	ServiceManager::notify_peer_event(e);

	CHECK_EQUAL(0u, origin.peer_events);
	CHECK_EQUAL(1u, p1.peer_events);
	CHECK_EQUAL(1u, p2.peer_events);
	CHECK_EQUAL(1u, p3.peer_events);
}

TEST(ServiceStart, ThrowingClassifierDoesNotStopTheBroadcast) {
	FlakyService origin("ORIGIN"), p1("P1");

	// An underwater event must carry a bool: the hauled classifier's
	// std::get<bool> throws on anything else.
	ServiceEvent e;
	e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
	e.event_source = ServiceIdentifier::UW_SENSOR;
	e.event_data = GPSLogEntry{};
	e.event_originator_unique_id = origin.get_unique_id();
	ServiceManager::notify_peer_event(e);

	CHECK_EQUAL(1u, p1.peer_events);
}
