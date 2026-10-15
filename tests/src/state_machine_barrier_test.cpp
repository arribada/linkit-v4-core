/**
 * @file state_machine_barrier_test.cpp
 * @brief The driver state-machine diagnostic barrier.
 *
 * The four drivers it is used by -- SmdSat, KIM2Device, M10QAsyncReceiver,
 * LoRaDevice -- are nRF-port-only, so none of them is reachable from a host
 * test. The barrier is deliberately a hardware-free header so the part with
 * behaviour in it is covered here instead of only compiled.
 *
 * There is very little behaviour left, and that is the result of two wrong
 * versions rather than of a small ambition: see the file header of
 * core/hardware/state_machine_barrier.hpp. What remains to pin is exactly two
 * things -- that every exception type the firmware throws gets named, and that
 * reporting does NOT consume the exception, so the caller's `throw;` still
 * reaches main.cpp. The second is the one that matters, because a barrier that
 * silently swallowed would look identical in a log.
 */

#include "state_machine_barrier.hpp"

#include "linux_timer.hpp"
#include "scheduler.hpp"

#include "CppUTest/TestHarness.h"

TEST_GROUP(StateMachineBarrier){};

/// Every type the firmware actually throws is named, and the last-resort catch
/// takes everything else. That last one decides whether a log line says
/// anything useful at all about an exception nobody anticipated.
TEST(StateMachineBarrier, every_thrown_type_is_named) {
	const char *got = nullptr;

	try {
		throw ErrorCode::CONFIG_STORE_CORRUPTED;
	} catch (...) {
		got = StateMachineBarrier::describe_current_exception();
	}
	STRCMP_EQUAL("ErrorCode", got);

	try {
		std::variant<int, double> v = 1;
		(void)std::get<double>(v);
	} catch (...) {
		got = StateMachineBarrier::describe_current_exception();
	}
	STRCMP_EQUAL("bad_variant_access", got);

	try {
		throw std::out_of_range("index");
	} catch (...) {
		got = StateMachineBarrier::describe_current_exception();
	}
	STRCMP_EQUAL("out_of_range", got);

	try {
		throw std::runtime_error("boom");
	} catch (...) {
		got = StateMachineBarrier::describe_current_exception();
	}
	STRCMP_EQUAL("std::exception", got);

	try {
		throw 42;  // nothing in the firmware throws this; the barrier still holds
	} catch (...) {
		got = StateMachineBarrier::describe_current_exception();
	}
	STRCMP_EQUAL("unknown exception", got);
}

/// report() must leave the exception in flight. describe_current_exception()
/// works by rethrowing internally to match the type, so getting that wrong --
/// letting one of those inner catches escape, or failing to re-match -- would
/// turn the barrier into the silent swallow the previous version was.
TEST(StateMachineBarrier, reporting_does_not_consume_the_exception) {
	StateMachineBarrier b("Test");
	bool rethrow_arrived = false;

	try {
		try {
			throw ErrorCode::SPI_COMMS_ERROR;
		} catch (...) {
			b.report(3);
			throw;  // exactly what the four dispatchers do
		}
	} catch (ErrorCode e) {
		rethrow_arrived = true;
		CHECK_EQUAL((int)ErrorCode::SPI_COMMS_ERROR, (int)e);
	}

	CHECK_TRUE(rethrow_arrived);
}

/// Reporting twice on the same exception is still safe and the type survives: a
/// nested dispatch could report on the way out of each level.
TEST(StateMachineBarrier, reporting_twice_keeps_the_type) {
	StateMachineBarrier b("Test");
	const char *first = nullptr;
	const char *second = nullptr;

	try {
		throw std::out_of_range("x");
	} catch (...) {
		first = StateMachineBarrier::describe_current_exception();
		b.report(1);
		second = StateMachineBarrier::describe_current_exception();
	}

	STRCMP_EQUAL("out_of_range", first);
	STRCMP_EQUAL("out_of_range", second);
}

// ===========================================================================
// The link that carries the whole design, and the one the host build could not
// see before: main.cpp is not compiled here, so nothing observed that an
// exception out of a driver dispatch reaches it.
//
// What IS observable is the mechanism. Scheduler::run() (scheduler.hpp:168)
// calls task.m_func() bare, with no try/catch and with its InterruptLock
// already out of scope, so an exception from a task body propagates out of
// run() -- which is what puts it in front of main.cpp:1687's catch(ErrorCode)
// and makes it an ErrorEvent.
//
// If someone ever wraps Scheduler::run() in a catch, every driver's diagnostic
// becomes a line about a fault nothing acts on, and this test is what says so.
// ===========================================================================

TEST_GROUP(StateMachineBarrierPropagation) {
	Timer *timer = nullptr;
	Scheduler *sched = nullptr;

	void setup() {
		timer = new LinuxTimer;
		timer->start();
		sched = new Scheduler(timer);
	}
	void teardown() {
		sched->clear_all();
		timer->stop();
		delete sched;
		delete timer;
	}
};

namespace {
/// Shaped like the four real dispatchers: work in a try, report, rethrow.
struct FakeDriver {
	StateMachineBarrier barrier{ "FakeDriver" };
	Scheduler *sched = nullptr;
	unsigned int dispatches = 0;
	bool handler_throws = true;

	void arm() {
		sched->post_task_prio([this]() { dispatch(); }, "FakeDriver", Scheduler::DEFAULT_PRIORITY, 0);
	}

	void dispatch() {
		dispatches++;
		try {
			if (handler_throws) throw ErrorCode::SPI_COMMS_ERROR;
		} catch (...) {
			barrier.report(7);
			throw;
		}
	}
};
}  // namespace

TEST(StateMachineBarrierPropagation, a_throw_escapes_the_scheduler_as_its_original_type) {
	FakeDriver drv;
	drv.sched = sched;
	drv.arm();

	bool escaped_as_errorcode = false;
	try {
		sched->run();
	} catch (ErrorCode e) {
		escaped_as_errorcode = true;
		// The TYPE matters, not just the escape: main.cpp turns catch(ErrorCode)
		// into an ErrorEvent carrying that code, and only that branch feeds the
		// FSM's error path.
		CHECK_EQUAL((int)ErrorCode::SPI_COMMS_ERROR, (int)e);
	} catch (...) {
		FAIL("escaped as something other than ErrorCode: main.cpp would catch it, but not as an ErrorEvent");
	}

	CHECK_TRUE(escaped_as_errorcode);
	CHECK_EQUAL(1u, drv.dispatches);
}

/// And the happy path is untouched: a dispatch that does not throw runs,
/// returns, and run() reports that it ran. The barrier costs nothing here.
TEST(StateMachineBarrierPropagation, a_clean_dispatch_is_unaffected) {
	FakeDriver drv;
	drv.sched = sched;
	drv.handler_throws = false;
	drv.arm();

	CHECK_TRUE(sched->run());
	CHECK_EQUAL(1u, drv.dispatches);
}
