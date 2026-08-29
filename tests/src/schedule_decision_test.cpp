#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

#include "schedule_decision.hpp"
#include "service.hpp"
#include "scheduler.hpp"
#include "fake_timer.hpp"
#include "fake_rtc.hpp"

extern Timer *system_timer;
extern Scheduler *system_scheduler;
extern RTC *rtc;

/// The smallest thing that is a Service. It exists to reach the default
/// `service_next_schedule()` — the wrapper every service that has not migrated
/// still goes through — and to make its mapping assertable.
class FakeSchedulingService : public Service {
public:
	FakeSchedulingService()
	    : Service(ServiceIdentifier::GNSS_SENSOR, "FAKE") {}

	unsigned int m_next_ms = 0;

	/// Reaches the protected default wrapper from a test.
	ScheduleDecision decide() { return service_next_schedule(); }

private:
	void service_init() override {}
	void service_term() override {}
	bool service_is_enabled() override { return true; }
	unsigned int service_next_schedule_in_ms() override { return m_next_ms; }
	void service_initiate() override {}
};

/// A service that holds. Counts how often the framework asks it again — which
/// is the only thing the hold kinds actually promise.
class FakeHoldingService : public Service {
public:
	FakeHoldingService()
	    : Service(ServiceIdentifier::GNSS_SENSOR, "HOLD") {}

	unsigned int m_asked = 0;
	const char *m_why = "waiting for something";
	unsigned int m_bound_s = 60;

protected:
	ScheduleDecision service_next_schedule() override {
		m_asked++;
		return ScheduleDecision::hold_for_event(m_bound_s, m_why);
	}

private:
	void service_init() override {}
	void service_term() override {}
	bool service_is_enabled() override { return true; }
	unsigned int service_next_schedule_in_ms() override { return SCHEDULE_DISABLED; }
	void service_initiate() override {}
};

TEST_GROUP(ScheduleDecision) {
	FakeTimer *fake_timer;
	FakeRTC *fake_rtc;

	void setup() {
		ServiceManager::reset();
		fake_rtc = new FakeRTC;
		rtc = fake_rtc;
		fake_rtc->settime(1652105502);
		fake_timer = new FakeTimer;
		system_timer = fake_timer;
		system_scheduler = new Scheduler(system_timer);
		fake_timer->start();
	}

	void teardown() {
		ServiceManager::reset();
		delete system_scheduler;
		delete fake_timer;
		delete fake_rtc;
		system_scheduler = nullptr;
		system_timer = nullptr;
		rtc = nullptr;
	}

	void advance_to(uint64_t ms) {
		fake_timer->set_counter(ms);
		fake_rtc->settime(1652105502 + (std::time_t)(ms / 1000));
		system_scheduler->run();
	}
};

// The four kinds carry what the single sentinel could not: which of them it is,
// and why. `reason()` is what reaches the field log and the bench %SCHED report.
TEST(ScheduleDecision, EachKindCarriesItsValueAndItsReason) {
	const ScheduleDecision r = ScheduleDecision::run(1500, "nominal period");
	CHECK_TRUE(ScheduleDecision::Kind::Run == r.kind());
	CHECK_EQUAL(1500U, r.delay_ms());
	STRCMP_EQUAL("nominal period", r.reason());
	CHECK_TRUE(r.comes_back_on_its_own());

	const ScheduleDecision h = ScheduleDecision::hold_until(1652105502U, "surface-cycle cooldown");
	CHECK_TRUE(ScheduleDecision::Kind::HoldUntil == h.kind());
	CHECK_EQUAL(1652105502U, h.epoch_s());
	CHECK_TRUE(h.comes_back_on_its_own());

	const ScheduleDecision o = ScheduleDecision::off("ARGOS_MODE=OFF");
	CHECK_TRUE(ScheduleDecision::Kind::Off == o.kind());
	STRCMP_EQUAL("ARGOS_MODE=OFF", o.reason());
	// The only kind that legitimately leaves a service with nothing pending.
	CHECK_FALSE(o.comes_back_on_its_own());
}

// The point of the whole type. A hold cannot be unbounded, because there is no
// way to write one: no public constructor, and the only factory for this kind
// takes the bound. A caller's zero and a caller's forever are both clamped into
// something the framework can honour, rather than trusted.
TEST(ScheduleDecision, AnEventHoldIsAlwaysBounded) {
	const ScheduleDecision zero = ScheduleDecision::hold_for_event(0, "waiting for the first Argos TX");
	CHECK_TRUE(ScheduleDecision::Kind::HoldForEvent == zero.kind());
	CHECK_EQUAL(ScheduleDecision::MIN_HOLD_S, zero.max_hold_s());

	const ScheduleDecision forever = ScheduleDecision::hold_for_event(0xFFFFFFFFU, "waiting for a surfacing");
	CHECK_EQUAL(ScheduleDecision::MAX_HOLD_S, forever.max_hold_s());

	const ScheduleDecision ok = ScheduleDecision::hold_for_event(120, "depth pile empty");
	CHECK_EQUAL(120U, ok.max_hold_s());
	CHECK_TRUE(ok.comes_back_on_its_own());
}

// A null reason would reach a %s in a log line. Never accept one.
TEST(ScheduleDecision, ReasonIsNeverNull) {
	STRCMP_EQUAL("unspecified", ScheduleDecision::off(nullptr).reason());
}

// The compatibility contract, and the reason 746 tests keep passing without a
// single assertion changing: a service that has not migrated goes through the
// default wrapper, and SCHEDULE_DISABLED still means exactly what it meant.
//
// "no-schedule" is not a free choice of words. Service::reschedule() already
// produced that literal for the bench report, and tests/bench/dte_campaign.py
// matches it by exact string equality — so the wrapper has to keep saying it.
TEST(ScheduleDecision, LegacyServicesMapThroughTheDefaultWrapperUnchanged) {
	FakeSchedulingService s;

	s.m_next_ms = 4200;
	const ScheduleDecision run = s.decide();
	CHECK_TRUE(ScheduleDecision::Kind::Run == run.kind());
	CHECK_EQUAL(4200U, run.delay_ms());
	STRCMP_EQUAL("scheduled", run.reason());

	s.m_next_ms = Service::SCHEDULE_DISABLED;
	const ScheduleDecision off = s.decide();
	CHECK_TRUE(ScheduleDecision::Kind::Off == off.kind());
	STRCMP_EQUAL("no-schedule", off.reason());
}

// A zero delay is a legitimate "run now", not a disabled schedule. The sentinel
// is 0xFFFFFFFF precisely so that every real delay stays expressible; the
// wrapper must not confuse the two.
TEST(ScheduleDecision, ZeroDelayIsRunNowNotDisabled) {
	FakeSchedulingService s;
	s.m_next_ms = 0;
	const ScheduleDecision d = s.decide();
	CHECK_TRUE(ScheduleDecision::Kind::Run == d.kind());
	CHECK_EQUAL(0U, d.delay_ms());
}

// THE guarantee. A service that holds for an event owns nothing to run — but
// the framework comes back on its own, without any peer event, within the bound
// the hold declared. This is what a bare SCHEDULE_DISABLED could not express,
// and its absence is what left beacons mute for whole deployments.
TEST(ScheduleDecision, AnEventHoldIsAlwaysReEvaluatedWithinItsBound) {
	FakeHoldingService s;
	s.m_bound_s = 60;
	s.start();

	CHECK_EQUAL(1U, s.m_asked);  // asked once at start, and it held
	// A hold is not a run: nothing is scheduled to execute.
	CHECK_TRUE(Service::SCHEDULE_DISABLED == s.get_last_schedule());

	// No event, no peer, nothing external. Just time.
	advance_to(61 * 1000);
	CHECK_EQUAL(2U, s.m_asked);
}

// ...and a service that keeps answering the same hold must not become a poll.
// The delay doubles on each identical answer, so a permanently-held service
// converges to a rare wake instead of the wake/log/skip loop this type exists
// to remove.
TEST(ScheduleDecision, RepeatingTheSameHoldBacksOffInsteadOfPolling) {
	FakeHoldingService s;
	s.m_bound_s = 60;
	s.start();
	CHECK_EQUAL(1U, s.m_asked);

	advance_to(61 * 1000);
	CHECK_EQUAL(2U, s.m_asked);

	// Second identical answer: the next re-evaluation is 120 s out, not 60.
	advance_to(61 * 1000 + 61 * 1000);
	CHECK_EQUAL(2U, s.m_asked);  // not yet — it doubled

    advance_to(61 * 1000 + 121 * 1000);
	CHECK_EQUAL(3U, s.m_asked);
}

// A different reason is a different situation: the streak restarts, so a
// service alternating between two real conditions is not punished for it.
TEST(ScheduleDecision, ADifferentReasonRestartsTheBackoff) {
	FakeHoldingService s;
	s.m_bound_s = 60;
	s.start();

	advance_to(61 * 1000);
	CHECK_EQUAL(2U, s.m_asked);

	// The second identical answer pushed the next check to 120 s out, so nothing
	// can be observed before then whatever the reason becomes.
	s.m_why = "a different reason entirely";
	advance_to(61 * 1000 + 121 * 1000);
	CHECK_EQUAL(3U, s.m_asked);

	// That third answer carried a NEW reason, so the streak restarted: the next
	// check is back to the base bound rather than 240 s.
	advance_to(61 * 1000 + 121 * 1000 + 61 * 1000);
	CHECK_EQUAL(4U, s.m_asked);
}
