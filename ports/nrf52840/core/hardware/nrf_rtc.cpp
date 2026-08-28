/**
 * @file nrf_rtc.cpp
 * @brief nRF52840 RTC wall-clock implementation (8 Hz, 24-bit counter with overflow tracking).
 *
 * The nRF RTC is a 24-bit counter (overflows every ~2 097 152 s ≈ 24 days at 8 Hz).
 * Overflow events increment a software counter to extend to 64-bit ticks.
 * A compare event at the midpoint catches any missed overflow during reads.
 *
 * Thread safety:
 *  - g_overflow_count (32-bit) is written only from the RTC ISR — atomic on Cortex-M4.
 *  - g_stamp64 (64-bit) is written only from the RTC ISR — not atomic, but the
 *    ISR is the sole writer and callers use InterruptLock before reading.
 *  - gettime()/settime()/getuptime() are called under InterruptLock.
 */

#include "nrf_rtc.hpp"
#include "drv_rtc.h"
#include "interrupt_lock.hpp"
#include "bsp.hpp"
#include "debug.hpp"

static constexpr uint32_t RTC_FREQ_HZ = 8;               ///< Lowest available RTC frequency
static constexpr int64_t TICKS_PER_OVERFLOW = 16777216;  ///< 2^24 (24-bit counter)
static constexpr int64_t SECONDS_PER_OVERFLOW = TICKS_PER_OVERFLOW / RTC_FREQ_HZ;

static int64_t g_timestamp_offset;
static volatile uint32_t g_overflow_count;
static volatile uint64_t g_stamp64;

/// @brief 64-bit tick count from 24-bit counter + overflow tracking.
/// @note Called under InterruptLock from getuptime(), or from ISR (sole writer of g_stamp64).
static uint64_t current_ticks() {
	uint64_t now = drv_rtc_counter_get(&BSP::RTC_Inits[RTC_DATE_TIME].rtc) + (g_overflow_count * TICKS_PER_OVERFLOW);

	// Detect missed overflow: if 'now' is behind the last recorded stamp,
	// the 24-bit counter has wrapped but the overflow ISR hasn't fired yet.
	if (now < g_stamp64) now += TICKS_PER_OVERFLOW;

	return now;
}

/// @brief RTC ISR — tracks overflows and midpoint stamps for wrap detection.
static void rtc_event_handler(drv_rtc_t const *const p_instance) {
	if (drv_rtc_overflow_pending(p_instance))
		g_overflow_count = g_overflow_count + 1;
	else if (drv_rtc_compare_pending(p_instance, 0))
		g_stamp64 = current_ticks();
}

void NrfRTC::init() {
	m_is_set = false;
	g_overflow_count = 0;
	g_timestamp_offset = 0;
	g_stamp64 = 0;

	const drv_rtc_config_t rtc_config = { .prescaler = RTC_FREQ_TO_PRESCALER(RTC_FREQ_HZ),
	                                      .interrupt_priority = BSP::RTC_Inits[RTC_DATE_TIME].irq_priority };

	ret_code_t err = drv_rtc_init(&BSP::RTC_Inits[RTC_DATE_TIME].rtc, &rtc_config, rtc_event_handler);
	if (err != NRF_SUCCESS) {
		DEBUG_ERROR("RTC init failed: 0x%08X", err);
		return;
	}

	drv_rtc_overflow_enable(&BSP::RTC_Inits[RTC_DATE_TIME].rtc, true);
	drv_rtc_compare_set(&BSP::RTC_Inits[RTC_DATE_TIME].rtc, 0, RTC_COUNTER_COUNTER_Msk >> 1, true);
	drv_rtc_start(&BSP::RTC_Inits[RTC_DATE_TIME].rtc);
}

void NrfRTC::uninit() {
	drv_rtc_stop(&BSP::RTC_Inits[RTC_DATE_TIME].rtc);
}

int64_t NrfRTC::getuptime() {
	InterruptLock lock;
	return current_ticks() / RTC_FREQ_HZ;
}

std::time_t NrfRTC::gettime() {
	InterruptLock lock;
	return getuptime() + g_timestamp_offset;
}

void NrfRTC::settime(std::time_t time) {
	InterruptLock lock;
	g_timestamp_offset = static_cast<int64_t>(time) - getuptime();
	m_is_set = true;
}

bool NrfRTC::is_set() {
	return m_is_set;
}
