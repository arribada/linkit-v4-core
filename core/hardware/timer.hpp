#pragma once

/**
 * @file timer.hpp
 * @brief Abstract hardware timer interface for scheduled callbacks.
 *
 * Provides millisecond-resolution scheduling via add_schedule() / cancel_schedule().
 * TimerHandle is an optional<unsigned int> — nullopt means invalid/cancelled.
 */

#include <cstdint>
#include <optional>
#include "inplace_function.hpp"

// 2026-06: 48 -> 96. The hardware-timer schedule list (g_schedules) is the
// BINDING limit for all *deferred* scheduler tasks (every post_task_prio with
// delay>0 consumes one slot here, 1:1). Unlike MAX_NUM_TASKS (immediate queue,
// doubled 64->128 earlier), this pool was never widened. Realistic field peak is
// ~25-35; 96 gives comfortable headroom above the drop-storm reset backstop.
// Cost ~76 B/entry (.bss). NOT tied to RTC channels (software list; RTC uses
// only compare0/compare1) — safe to raise. See MemoryMonitorService high-water log.
#define MAX_NUM_TIMERS 96  ///< Maximum concurrent pending schedules

#ifndef INPLACE_FUNCTION_SIZE_TIMER
#define INPLACE_FUNCTION_SIZE_TIMER 48  ///< Inline storage for schedule callbacks (bytes)
#endif

class Timer {
public:
	/// @brief Opaque handle to a pending schedule.  nullopt = invalid/cancelled.
	using TimerHandle = std::optional<unsigned int>;

	virtual ~Timer() = default;

	/// @brief Milliseconds since start().
	virtual uint64_t get_counter() = 0;

	/**
	 * @brief Schedule a callback at an absolute millisecond timestamp.
	 * @param task_func     Callback to execute when target_count is reached.
	 * @param target_count  Absolute time in ms (compared against get_counter()).
	 * @return Handle for cancellation, or nullopt if the schedule list is full.
	 */
	virtual TimerHandle add_schedule(stdext::inplace_function<void(), INPLACE_FUNCTION_SIZE_TIMER> const &task_func,
	                                 uint64_t target_count) = 0;

	/// @brief Cancel a pending schedule.  Resets @p handle to nullopt.
	virtual void cancel_schedule(TimerHandle &handle) = 0;

	/// @brief Start the underlying hardware counter.
	virtual void start() = 0;

	/// @brief Stop the underlying hardware counter.
	virtual void stop() = 0;

	/// @name Occupancy instrumentation (high-water diagnostics)
	/// Default no-op implementations (return 0 / capacity) so non-instrumented
	/// Timer backends (test fakes, other ports) need no changes. NrfTimer
	/// overrides these to expose the real g_schedules occupancy.
	/// @{
	/// @brief Current number of pending schedules.
	virtual unsigned int schedules_size() const { return 0; }
	/// @brief Peak number of pending schedules observed since boot.
	virtual unsigned int schedules_high_water() const { return 0; }
	/// @brief Maximum capacity of the schedule pool.
	virtual unsigned int schedules_capacity() const { return MAX_NUM_TIMERS; }
	/// @}
};
