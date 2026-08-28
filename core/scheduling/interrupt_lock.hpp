/**
 * @file interrupt_lock.hpp
 * @brief RAII interrupt lock — disables IRQs on construction, restores on destruction.
 *
 * nRF52840: uses SoftDevice critical region (sd_nvic_critical_region_enter/exit).
 * Linux (tests): uses std::recursive_mutex.
 */

#pragma once

#include <cstdint>

/// @brief RAII guard that disables interrupts for the scope lifetime.
/// Non-copyable. Supports nesting via SoftDevice critical region API.
class InterruptLock {
public:
	/// @brief Enter critical region (disable IRQs / lock mutex).
	InterruptLock();

	InterruptLock(const InterruptLock &) = delete;
	InterruptLock &operator=(const InterruptLock &) = delete;

	/// @brief Exit critical region (restore IRQs / unlock mutex).
	~InterruptLock();

private:
	uint8_t m_nested = 0;  ///< SoftDevice nesting counter (unused on Linux port).
};