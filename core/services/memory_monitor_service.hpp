/**
 * @file memory_monitor_service.hpp
 * @brief Memory monitor service — periodic heap/stack usage logging (every 12h).
 */

#pragma once

#include "service.hpp"
#include "memmang.hpp"
#include "scheduler.hpp"  // Scheduler occupancy getters + g_scheduler_drop_count
#include "timer.hpp"      // Timer::schedules_* occupancy getters

extern Scheduler *system_scheduler;
extern Timer *system_timer;

/// @brief Periodic memory monitor — logs heap stats and max stack usage.
class MemoryMonitorService : public Service {
public:
	MemoryMonitorService() : Service(ServiceIdentifier::MEMORY_MONITOR, "MEMORY") {}

	void service_initiate() {
		HeapStats_t heap_stats = MEMMANG::heap_stats();
		DEBUG_INFO("MemoryMonitorService: HEAP: %d min | %d free | %d freeblk | %d allocs | %d frees",
		           heap_stats.xMinimumEverFreeBytesRemaining, heap_stats.xAvailableHeapSpaceInBytes,
		           heap_stats.xNumberOfFreeBlocks, heap_stats.xNumberOfSuccessfulAllocations,
		           heap_stats.xNumberOfSuccessfulFrees);
		DEBUG_INFO("MemoryMonitorService: STACK: %u max", MEMMANG::max_stack_usage());

		// Scheduler / hardware-timer pool occupancy (high-water diagnostics).
		// The deferred pool is backed 1:1 by the hardware-timer schedule list
		// (g_schedules) — the tightest cap, so it's the one to watch. high-water
		// marks are cumulative since boot, so even this 12h sampling captures the
		// true peak. Routine line = TRACE; escalates to WARN/ERROR if any pool
		// peaked near its cap or a task was ever dropped (drops also trigger the
		// drop-storm soft-reset net independently).
		unsigned int t_cur = system_scheduler ? system_scheduler->tasks_size() : 0;
		unsigned int t_hw = system_scheduler ? system_scheduler->tasks_high_water() : 0;
		unsigned int d_cur = system_scheduler ? system_scheduler->deferred_size() : 0;
		unsigned int d_hw = system_scheduler ? system_scheduler->deferred_high_water() : 0;
		unsigned int s_cap = Scheduler::capacity();
		unsigned int g_cur = system_timer ? system_timer->schedules_size() : 0;
		unsigned int g_hw = system_timer ? system_timer->schedules_high_water() : 0;
		unsigned int g_cap = system_timer ? system_timer->schedules_capacity() : 0;
		unsigned int drops = g_scheduler_drop_count;

		DEBUG_TRACE(
		    "MemoryMonitorService: SCHED tasks=%u/%u-hw deferred=%u/%u-hw cap=%u | timer=%u/%u-hw cap=%u | drops=%u",
		    t_cur, t_hw, d_cur, d_hw, s_cap, g_cur, g_hw, g_cap, drops);

		auto pct = [](unsigned int v, unsigned int cap) -> unsigned int { return cap ? (v * 100u / cap) : 0u; };
		unsigned int worst = pct(t_hw, s_cap);
		unsigned int dp = pct(d_hw, s_cap);
		if (dp > worst) worst = dp;
		unsigned int gp = pct(g_hw, g_cap);
		if (gp > worst) worst = gp;
		if (drops > 0 || worst >= 90) {
			DEBUG_ERROR("MemoryMonitorService: SCHED pool CRITICAL — peak %u%% of cap, drops=%u (tasks_hw=%u/%u "
			            "deferred_hw=%u/%u timer_hw=%u/%u)",
			            worst, drops, t_hw, s_cap, d_hw, s_cap, g_hw, g_cap);
		} else if (worst >= 75) {
			DEBUG_WARN("MemoryMonitorService: SCHED pool HIGH — peak %u%% of cap (tasks_hw=%u/%u deferred_hw=%u/%u "
			           "timer_hw=%u/%u)",
			           worst, t_hw, s_cap, d_hw, s_cap, g_hw, g_cap);
		}

		service_complete();
	}

	unsigned int service_next_schedule_in_ms() override {
		// Run every 12 hours
		return 12 * 3600 * 1000;
	}

	bool service_is_enabled() override { return true; }
	bool service_is_usable_underwater() override { return true; }
	void service_init() override {}
	void service_term() override {}
};
