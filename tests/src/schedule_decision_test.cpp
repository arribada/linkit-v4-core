#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

#include "schedule_decision.hpp"
#include "service.hpp"

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

TEST_GROUP(ScheduleDecision){void setup(){ServiceManager::reset();
}
void teardown() {
	ServiceManager::reset();
}
}
;

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
