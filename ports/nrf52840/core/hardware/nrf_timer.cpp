/**
 * @file nrf_timer.cpp
 * @brief nRF52840 hardware timer — schedule-based task execution from RTC ISR.
 *
 * RTC clock = 32768 Hz, prescaler = 33 → ~992.97 Hz (~1.007 ms per tick).
 * 24-bit counter overflows every ~16.9 million ticks (~16 888 seconds ≈ 4.7 hours).
 * Software overflow tracking extends to 64-bit ticks.
 *
 * Schedules are stored in a time-sorted etl::list (fixed capacity, no heap).
 * The RTC compare0 interrupt fires for the next due schedule; overflow interrupt
 * tracks the 24-bit wrap.  Compare1 fires at the midpoint for wrap detection.
 */

#include <cstdint>
#include "nrf_timer.hpp"
#include "nrf_delay.h"
#include "drv_rtc.h"
#include "interrupt_lock.hpp"
#include "bsp.hpp"
#include "debug.hpp"
#include "etl/list.h"

// ═══════════════════════════════════════════════════════
//  Tick ↔ millisecond conversion
// ═══════════════════════════════════════════════════════

/// @name RTC timer constants
/// @{
static constexpr uint16_t RTC_TIMER_PRESCALER = 32;       ///< nRF prescaler register value (divider = 33)
static constexpr uint64_t FREQ_NUMERATOR      = 992969ULL;  ///< Effective frequency × 1000 (32768/33 ≈ 992.97 Hz)
static constexpr uint64_t FREQ_DENOMINATOR    = 1000000ULL;
static constexpr uint32_t TICKS_PER_OVERFLOW  = 16777216;   ///< 2^24 (24-bit counter wrap)
/// @}

/// @brief Convert 64-bit RTC ticks to milliseconds (integer arithmetic, no float).
/// @param ticks  64-bit tick count from the RTC counter.
/// @return Equivalent time in milliseconds.
static constexpr uint64_t ticks_to_ms(uint64_t ticks) {
	return (ticks * FREQ_DENOMINATOR) / FREQ_NUMERATOR;
}

/// @brief Convert milliseconds to 64-bit RTC ticks (integer arithmetic, no float).
/// @param ms  Time in milliseconds.
/// @return Equivalent tick count.
static constexpr uint64_t ms_to_ticks(uint64_t ms) {
	return (ms * FREQ_NUMERATOR) / FREQ_DENOMINATOR;
}

// ═══════════════════════════════════════════════════════
//  64-bit tick tracking (same pattern as nrf_rtc.cpp)
// ═══════════════════════════════════════════════════════

static volatile uint32_t g_overflow_count;  ///< Number of 24-bit counter overflows since init
static volatile uint64_t g_stamp64;         ///< Midpoint stamp for overflow detection

/// @brief 64-bit tick count.  Must be called under InterruptLock or from ISR.
/// @return Current tick count (24-bit counter + overflow tracking).
static uint64_t current_ticks()
{
	InterruptLock lock;
	uint64_t now = drv_rtc_counter_get(&BSP::RTC_Inits[RTC_TIMER].rtc)
	             + (static_cast<uint64_t>(g_overflow_count) * TICKS_PER_OVERFLOW);

	// Detect missed overflow (counter wrapped but ISR hasn't fired yet)
	if (now < g_stamp64)
		now += TICKS_PER_OVERFLOW;

	return now;
}


// ═══════════════════════════════════════════════════════
//  Schedule list (fixed-capacity, no heap)
// ═══════════════════════════════════════════════════════

/// @brief A pending scheduled callback with its target time and unique ID.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnull-dereference"
struct Schedule {
	stdext::inplace_function<void(), INPLACE_FUNCTION_SIZE_TIMER> m_func;  ///< Callback (runs in ISR context)
	std::optional<unsigned int> m_id;   ///< Unique handle for cancel_schedule()
	uint64_t m_target_ticks;            ///< Absolute tick count when this fires
};
#pragma GCC diagnostic pop

/// @brief Time-sorted list of pending schedules (fixed capacity, no heap).
static etl::list<Schedule, MAX_NUM_TIMERS> g_schedules;

/// @brief Monotonic ID counter — wraps at UINT_MAX, skips 0 (reserved for invalid handle).
static unsigned int g_unique_id;


// ═══════════════════════════════════════════════════════
//  RTC ISR — overflow tracking + schedule dispatch
// ═══════════════════════════════════════════════════════

/// @brief Set compare0 to the next due schedule, or disable if empty.
static void setup_compare_interrupt()
{
	if (g_schedules.empty()) {
		drv_rtc_compare_disable(&BSP::RTC_Inits[RTC_TIMER].rtc, 0);
		return;
	}

	uint64_t next_overflow_ticks = (static_cast<uint64_t>(g_overflow_count) + 1) * TICKS_PER_OVERFLOW;
	uint64_t next_schedule_ticks = g_schedules.front().m_target_ticks;

	// Ensure compare is at least 5 ticks in the future (hardware minimum)
	uint64_t now = current_ticks();
	if (next_schedule_ticks <= now + 5)
		next_schedule_ticks = now + 5;

	if (next_schedule_ticks < next_overflow_ticks) {
		uint32_t compare_value = next_schedule_ticks % TICKS_PER_OVERFLOW;
		drv_rtc_compare_set(&BSP::RTC_Inits[RTC_TIMER].rtc, 0, compare_value, true);
	}
}

/// @brief RTC ISR handler — processes overflows, due schedules, and midpoint stamps.
/// @note Schedule callbacks execute in this ISR context — keep them short.
static void rtc_event_handler(drv_rtc_t const * const p_instance)
{
	if (drv_rtc_overflow_pending(p_instance)) {
		g_overflow_count = g_overflow_count + 1;
	} else if (drv_rtc_compare_pending(p_instance, 0)) {
		// Fire all due schedules. Re-derive the front each iteration and erase the
		// fired node BEFORE its callback: callbacks run in THIS ISR and may call
		// cancel_schedule() (LED transitions cancel timer handles) which erases an
		// arbitrary node; a live iterator held across the callback would dangle
		// -> use-after-free on the next pass.
		while (!g_schedules.empty()) {
			auto it = g_schedules.begin();
			if (it->m_target_ticks > current_ticks())
				break;  // List is sorted — no more due

			Schedule sched = *it;
			g_schedules.erase(it);
			if (sched.m_func)
				sched.m_func();
		}
	} else if (drv_rtc_compare_pending(p_instance, 1)) {
		g_stamp64 = current_ticks();
	}

	setup_compare_interrupt();
}


// ═══════════════════════════════════════════════════════
//  NrfTimer implementation
// ═══════════════════════════════════════════════════════

void NrfTimer::init()
{
	m_start_ticks = 0;
	g_overflow_count = 0;
	g_stamp64 = 0;

	drv_rtc_config_t rtc_config = {
		.prescaler          = RTC_TIMER_PRESCALER,
		.interrupt_priority = BSP::RTC_Inits[RTC_TIMER].irq_priority
	};

	ret_code_t err = drv_rtc_init(&BSP::RTC_Inits[RTC_TIMER].rtc, &rtc_config, rtc_event_handler);
	if (err != NRF_SUCCESS) {
		DEBUG_ERROR("NrfTimer: RTC init failed (0x%08X)", err);
		return;
	}

	drv_rtc_compare_set(&BSP::RTC_Inits[RTC_TIMER].rtc, 1, RTC_COUNTER_COUNTER_Msk >> 1, true);
	drv_rtc_overflow_enable(&BSP::RTC_Inits[RTC_TIMER].rtc, true);
}

void NrfTimer::uninit()
{
	stop();
	nrf_delay_us(50);  // Wait for RTC peripheral to actually stop
	g_schedules.clear();
}

uint64_t NrfTimer::get_counter()
{
	InterruptLock lock;
	return ticks_to_ms(current_ticks() - m_start_ticks);
}

Timer::TimerHandle NrfTimer::add_schedule(
	stdext::inplace_function<void(), INPLACE_FUNCTION_SIZE_TIMER> const &task_func,
	uint64_t target_count_ms)
{
	Schedule schedule;
	schedule.m_func = task_func;
	schedule.m_target_ticks = ms_to_ticks(target_count_ms) + m_start_ticks;

	TimerHandle handle;

	{
		InterruptLock lock;

		schedule.m_id = g_unique_id;

		// Insert in sorted order (earliest first)
		auto iter = g_schedules.begin();
		while (iter != g_schedules.end() && iter->m_target_ticks <= schedule.m_target_ticks)
			++iter;

		// ETL assert fires if list is full → etl_error_handler → reset in release,
		// blink in debug.  This is intentional: a full schedule list means the
		// system is in a broken state and a reset is safer than silent failure.
		g_schedules.insert(iter, schedule);
		handle = g_unique_id;

		g_unique_id++;
		if (g_unique_id == 0) g_unique_id = 1;  // Skip 0 (collision with invalid handle)
	}

	drv_rtc_irq_trigger(&BSP::RTC_Inits[RTC_TIMER].rtc);
	return handle;
}

void NrfTimer::cancel_schedule(TimerHandle &handle)
{
	if (!handle.has_value())
		return;

	InterruptLock lock;

	for (auto iter = g_schedules.begin(); iter != g_schedules.end(); ++iter) {
		if (iter->m_id == *handle) {
			g_schedules.erase(iter);
			handle.reset();
			drv_rtc_irq_trigger(&BSP::RTC_Inits[RTC_TIMER].rtc);
			return;
		}
	}
}

void NrfTimer::start()
{
	m_start_ticks = current_ticks();
	drv_rtc_start(&BSP::RTC_Inits[RTC_TIMER].rtc);
}

void NrfTimer::stop()
{
	drv_rtc_stop(&BSP::RTC_Inits[RTC_TIMER].rtc);
}
