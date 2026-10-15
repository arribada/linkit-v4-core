/**
 * @file state_machine_barrier_test.cpp
 * @brief The driver state-machine exception barrier.
 *
 * The three drivers it protects -- SmdSat, KIM2Device, M10QAsyncReceiver -- are
 * nRF-port-only, so none of them is reachable from a host test. The barrier is
 * deliberately a hardware-free header so the part with the actual policy in it
 * (what gets logged, how long until the next tick, when the run resets) is
 * covered here instead of only compiled.
 */

#include "state_machine_barrier.hpp"

#include <optional>

#include "CppUTest/TestHarness.h"

TEST_GROUP(StateMachineBarrier){};

TEST(StateMachineBarrier, a_fresh_barrier_has_no_run) {
	StateMachineBarrier b("Test");
	CHECK_EQUAL(0u, b.consecutive());
}

/// The first occurrences retry quickly: a one-off throw should cost a session
/// almost nothing.
TEST(StateMachineBarrier, the_first_occurrences_retry_quickly) {
	for (unsigned int n = 1; n <= StateMachineBarrier::LOGGED_OCCURRENCES; n++) {
		CHECK_EQUAL(StateMachineBarrier::RETRY_DELAY_MS, StateMachineBarrier::retry_delay_ms(n));
		CHECK_TRUE(StateMachineBarrier::shall_log(n));
	}
}

/// Past that the throw looks deterministic, so it backs off and goes quiet --
/// a DEBUG_ERROR line also costs ~170 ms in the flash system log.
TEST(StateMachineBarrier, a_persistent_throw_backs_off_and_stops_logging) {
	const unsigned int first_quiet = StateMachineBarrier::LOGGED_OCCURRENCES + 1;

	CHECK_TRUE(StateMachineBarrier::shall_announce_quiet(first_quiet));
	CHECK_FALSE(StateMachineBarrier::shall_log(first_quiet));
	CHECK_EQUAL(StateMachineBarrier::QUIET_DELAY_MS, StateMachineBarrier::retry_delay_ms(first_quiet));

	// Exactly one announcement, not one per tick.
	CHECK_FALSE(StateMachineBarrier::shall_announce_quiet(first_quiet + 1));
	CHECK_FALSE(StateMachineBarrier::shall_log(first_quiet + 1));
	CHECK_EQUAL(StateMachineBarrier::QUIET_DELAY_MS, StateMachineBarrier::retry_delay_ms(first_quiet + 1));

	// Past ESCALATE_AFTER the delay stops being the question -- the caller is
	// told to rethrow instead. See the escalation tests below.
	CHECK_TRUE(StateMachineBarrier::shall_escalate(StateMachineBarrier::ESCALATE_AFTER));
	CHECK_FALSE(StateMachineBarrier::shall_escalate(StateMachineBarrier::ESCALATE_AFTER - 1));
}

/// THE behaviour this barrier exists to get right, and the one the first version
/// got wrong. A run that keeps throwing is not absorbed for ever: on_exception
/// returns nullopt, the caller rethrows, and the fault reaches main.cpp's catch
/// -> ErrorEvent -> the ErrorState ladder (gentracker.cpp:155-210), which resets
/// and retries with a growing backoff. Swallowing it instead removed the only
/// escalation a wedged driver had, and a reset is the stronger remedy: it
/// re-initialises the peripheral, which ticking the same state again cannot.
TEST(StateMachineBarrier, a_persistent_fault_is_handed_back_not_absorbed) {
	StateMachineBarrier b("Test");
	std::optional<unsigned int> delay;

	// Everything up to the threshold is absorbed, so a transient costs nothing.
	for (unsigned int n = 1; n < StateMachineBarrier::ESCALATE_AFTER; n++) {
		try {
			throw ErrorCode::SPI_COMMS_ERROR;
		} catch (...) {
			delay = b.on_exception(2);
		}
		CHECK_TRUE(delay.has_value());
	}

	// The one that tips it over asks the caller to rethrow.
	try {
		throw ErrorCode::SPI_COMMS_ERROR;
	} catch (...) {
		delay = b.on_exception(2);
	}
	CHECK_FALSE(delay.has_value());
	CHECK_EQUAL(StateMachineBarrier::ESCALATE_AFTER, b.consecutive());
}

/// Escalation is latching only while the run continues: a dispatch that
/// completes clears it, so an intermittent fault that recovers is never
/// escalated however many bursts it has had.
TEST(StateMachineBarrier, a_clean_tick_cancels_a_pending_escalation) {
	StateMachineBarrier b("Test");

	for (unsigned int n = 1; n < StateMachineBarrier::ESCALATE_AFTER; n++) {
		try {
			throw ErrorCode::SPI_COMMS_ERROR;
		} catch (...) {
			b.on_exception(2);
		}
	}
	b.on_tick_completed();

	// Back to square one: this throw is absorbed, not escalated.
	std::optional<unsigned int> delay;
	try {
		throw ErrorCode::SPI_COMMS_ERROR;
	} catch (...) {
		delay = b.on_exception(2);
	}
	CHECK_TRUE(delay.has_value());
	CHECK_EQUAL(StateMachineBarrier::RETRY_DELAY_MS, *delay);
}

TEST(StateMachineBarrier, the_run_counts_up_and_the_delay_follows) {
	StateMachineBarrier b("Test");

	for (unsigned int n = 1; n <= StateMachineBarrier::LOGGED_OCCURRENCES; n++) {
		std::optional<unsigned int> delay;
		try {
			throw ErrorCode::SPI_COMMS_ERROR;
		} catch (...) {
			delay = b.on_exception(4);
		}
		CHECK_EQUAL(n, b.consecutive());
		CHECK_TRUE(delay.has_value());
		CHECK_EQUAL(StateMachineBarrier::RETRY_DELAY_MS, *delay);
	}

	std::optional<unsigned int> delay;
	try {
		throw ErrorCode::SPI_COMMS_ERROR;
	} catch (...) {
		delay = b.on_exception(4);
	}
	CHECK_TRUE(delay.has_value());
	CHECK_EQUAL(StateMachineBarrier::QUIET_DELAY_MS, *delay);
}

/// A tick that completes ends the run, so an intermittent fault keeps getting
/// the fast retry and a fresh log line rather than inheriting the quiet state
/// of some earlier burst.
TEST(StateMachineBarrier, a_clean_tick_ends_the_run) {
	StateMachineBarrier b("Test");

	for (int i = 0; i < 9; i++) {
		try {
			throw ErrorCode::RESOURCE_NOT_AVAILABLE;
		} catch (...) {
			b.on_exception(1);
		}
	}
	CHECK_EQUAL(9u, b.consecutive());

	b.on_tick_completed();
	CHECK_EQUAL(0u, b.consecutive());

	std::optional<unsigned int> delay;
	try {
		throw ErrorCode::RESOURCE_NOT_AVAILABLE;
	} catch (...) {
		delay = b.on_exception(1);
	}
	CHECK_EQUAL(1u, b.consecutive());
	CHECK_TRUE(delay.has_value());
	CHECK_EQUAL(StateMachineBarrier::RETRY_DELAY_MS, *delay);
}

/// Every type the firmware actually throws is named, and the last-resort catch
/// takes everything else -- that is the one that decides whether the exception
/// reaches std::terminate.
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
