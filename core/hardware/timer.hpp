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

// Must be >= the scheduler's MAX_NUM_TASKS (64): each deferred task posts one
// timer schedule, so a smaller timer list makes add_schedule ETL-assert (reset)
// at 48 BEFORE the scheduler's own full()-guard (sized to MAX_NUM_TASKS) can drop
// gracefully — making that guard dead code. Keep the two aligned.
#define MAX_NUM_TIMERS 64  ///< Maximum concurrent pending schedules (>= MAX_NUM_TASKS)

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
	virtual TimerHandle add_schedule(stdext::inplace_function<void(), INPLACE_FUNCTION_SIZE_TIMER> const &task_func, uint64_t target_count) = 0;

	/// @brief Cancel a pending schedule.  Resets @p handle to nullopt.
	virtual void cancel_schedule(TimerHandle &handle) = 0;

	/// @brief Start the underlying hardware counter.
	virtual void start() = 0;

	/// @brief Stop the underlying hardware counter.
	virtual void stop() = 0;
};
