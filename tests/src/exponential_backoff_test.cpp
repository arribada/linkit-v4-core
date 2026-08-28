#include "exponential_backoff.hpp"

#include "CppUTest/TestHarness.h"

// The failure this pins is unreachable through the state machines: both TX
// services suspend at DEVICE_ERROR_MAX_CONSECUTIVE (3), so the FSM can never
// drive the attempt count into the range where `base << (attempt - 1)` wraps.
// The bug would only appear the day that suspension changes -- which is exactly
// why it has to be pinned here rather than through the services.
TEST_GROUP(ExponentialBackoff){};

TEST(ExponentialBackoff, DoublesThenSaturates) {
	const unsigned int BASE = 60000;  // 1 min
	const unsigned int CAP = 600000;  // 10 min

	// The sequence itself. Nothing here is shortened relative to the shift.
	CHECK_EQUAL(60000U, Backoff::doubling_capped(BASE, CAP, 1));
	CHECK_EQUAL(120000U, Backoff::doubling_capped(BASE, CAP, 2));
	CHECK_EQUAL(240000U, Backoff::doubling_capped(BASE, CAP, 3));
	CHECK_EQUAL(480000U, Backoff::doubling_capped(BASE, CAP, 4));

	// 60000 << 4 is 960000, above the cap: attempt 5 is where it saturates, and
	// it stays there.
	CHECK_EQUAL(CAP, Backoff::doubling_capped(BASE, CAP, 5));
	CHECK_EQUAL(CAP, Backoff::doubling_capped(BASE, CAP, 6));
	CHECK_EQUAL(CAP, Backoff::doubling_capped(BASE, CAP, 100));

	// attempt 0 is not a real input, but it must not underflow into a huge loop
	// or a zero delay.
	CHECK_EQUAL(BASE, Backoff::doubling_capped(BASE, CAP, 0));
}

// The whole point: a backoff must never come back SHORTER as the failures pile
// up. With a 60 s base, `base << (attempt - 1)` returns exactly 0 from attempt 28
// -- an immediate retry after twenty-eight consecutive hardware failures -- and
// from attempt 33 the shift count reaches the width of the type, which is
// undefined behaviour.
TEST(ExponentialBackoff, NeverWrapsToASmallerDelay) {
	const unsigned int BASE = 60000;
	const unsigned int CAP = 600000;

	unsigned int prev = 0;
	for (unsigned int n = 1; n <= 1000; n++) {
		unsigned int ms = Backoff::doubling_capped(BASE, CAP, n);
		CHECK_COMPARE(ms, >=, prev);  // monotonic
		CHECK_COMPARE(ms, >=, BASE);  // never shorter than the first attempt
		CHECK_COMPARE(ms, <=, CAP);   // never above the ceiling
		prev = ms;
	}
}

// Holds for any constants, not just the ones the services happen to use today.
// This is what replaces the previous "clamp the shift at 16", which was only
// correct while BASE_MS stayed at 60000.
TEST(ExponentialBackoff, HoldsForOtherConstants) {
	// A base large enough that `base << 16` would itself overflow.
	const unsigned int BIG = 300000;
	const unsigned int CAP = 1000000;
	CHECK_EQUAL(BIG, Backoff::doubling_capped(BIG, CAP, 1));
	CHECK_EQUAL(600000U, Backoff::doubling_capped(BIG, CAP, 2));
	CHECK_EQUAL(CAP, Backoff::doubling_capped(BIG, CAP, 3));
	CHECK_EQUAL(CAP, Backoff::doubling_capped(BIG, CAP, 500));

	// A cap below the base: the caller asked for something contradictory, and
	// the ceiling wins rather than the floor.
	CHECK_EQUAL(1000U, Backoff::doubling_capped(60000, 1000, 1));

	// It is constexpr, so a wrong sequence can be caught at build time too.
	static_assert(Backoff::doubling_capped(60000, 600000, 4) == 480000, "");
	static_assert(Backoff::doubling_capped(60000, 600000, 40) == 600000, "");
}
