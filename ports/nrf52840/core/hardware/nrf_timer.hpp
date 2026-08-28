#pragma once

/**
 * @file nrf_timer.hpp
 * @brief nRF52840 hardware timer for schedule-based task execution (~1 ms resolution).
 *
 * Uses a 24-bit RTC peripheral at ~993 Hz (32768 / 33) with overflow tracking
 * to provide a 64-bit millisecond counter.  Schedules are stored in a sorted
 * etl::list and serviced from the RTC compare interrupt.
 *
 * @note Schedule callbacks execute in RTC ISR context — they must be short
 *       and non-blocking (set a flag, post a scheduler task, toggle GPIO).
 *
 * Singleton — only one timer instance exists.
 */

#include "timer.hpp"

class NrfTimer final : public Timer {
public:
	static NrfTimer &get_instance() {
		static NrfTimer instance;
		return instance;
	}

	/// @brief Configure RTC peripheral, overflow and compare interrupts.
	void init();

	/// @brief Stop RTC and clear all pending schedules.
	void uninit();

	/// @brief Milliseconds elapsed since start() was called.
	uint64_t get_counter() override;

	/**
	 * @brief Schedule a callback at an absolute millisecond timestamp.
	 * @param task_func  Callback to execute (runs in RTC ISR context — keep short).
	 * @param target_count  Absolute time in ms (from get_counter() epoch).
	 * @return Handle for cancel_schedule(), or nullopt if the schedule list is full.
	 */
	TimerHandle add_schedule(stdext::inplace_function<void(), INPLACE_FUNCTION_SIZE_TIMER> const &task_func,
	                         uint64_t target_count) override;

	/// @brief Cancel a pending schedule.  Invalidates @p handle on success.
	void cancel_schedule(TimerHandle &handle) override;

	/// @brief Start the RTC counter and record the tick origin for get_counter().
	void start() override;

	/// @brief Stop the RTC counter (schedules remain pending until uninit).
	void stop() override;

	/// @brief Current / peak / max pending-schedule occupancy (g_schedules).
	unsigned int schedules_size() const override;
	unsigned int schedules_high_water() const override;
	unsigned int schedules_capacity() const override;

private:
	uint64_t m_start_ticks = 0;  ///< Tick count at start() — subtracted in get_counter()
	NrfTimer() = default;
	NrfTimer(NrfTimer const &) = delete;
	void operator=(NrfTimer const &) = delete;
};
