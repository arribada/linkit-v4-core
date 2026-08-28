#pragma once

/**
 * @file exponential_backoff.hpp
 * @brief Doubling backoff that saturates instead of wrapping.
 *
 * The obvious spelling, `base << (attempt - 1)` followed by a cap, is wrong for
 * a reason that is invisible at the call site: the shift overflows unsigned int
 * long before the cap can act on the result, and an overflowed backoff wraps to a
 * SMALL value. With the 60 s base both TX services use (60000 = 2^5 x 1875, so
 * the product hits zero once the shift reaches 27) it returns exactly 0 from the
 * 28th attempt -- retry immediately, on a device that has just failed twenty-eight
 * times in a row -- and from the 33rd the shift count reaches the width of the
 * type, which is undefined behaviour rather than merely a wrong number.
 *
 * Both TX services carried that spelling. Each is safe only by an invariant that
 * lives elsewhere in its own function (a suspension that caps the attempt count),
 * which is exactly the kind of guarantee that disappears when someone changes the
 * suspension.
 *
 * This is a pure function of three numbers. It is shared so the sequence has ONE
 * definition and ONE test; it does not couple the services, which keep their own
 * constants.
 */

namespace Backoff {

/// @brief `base_ms`, doubled once per attempt beyond the first, never above `cap_ms`.
///
/// Produces exactly the same sequence as `base_ms << (attempt - 1)` capped at
/// `cap_ms`, for every `attempt` that expression can represent -- nothing is
/// shortened. It simply stops doubling once the cap is reached, so there is no
/// shift to bound and no magic constant tied to the current value of `base_ms`.
///
/// @param base_ms  delay for the first attempt
/// @param cap_ms   ceiling; the result never exceeds it
/// @param attempt  1-based attempt number. 0 is treated as 1.
constexpr unsigned int doubling_capped(unsigned int base_ms, unsigned int cap_ms, unsigned int attempt) {
	unsigned int ms = base_ms;
	for (unsigned int i = 1; i < attempt && ms < cap_ms; i++)
		ms <<= 1;
	return (ms > cap_ms) ? cap_ms : ms;
}

}  // namespace Backoff
