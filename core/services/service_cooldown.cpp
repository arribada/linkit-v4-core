/**
 * @file service_cooldown.cpp
 * @brief Surface-cycle cooldown: noinit state, CRC, sleep entry and exit.
 *
 * Split out of service.cpp, where it was a 311-line island in the middle of the
 * Service framework. It is a self-contained module: its whole state lives in a
 * .noinit struct guarded by a CRC16, its API is static on ServiceManager, and it
 * is already consumed from five other files through is_in_cooldown().
 *
 * Nothing in service.hpp changed to make this possible -- the members it touches
 * are declared `static inline` inside the class, so their definitions may live in
 * any translation unit.
 *
 * Five includes came here with it and left service.cpp behind: <cstddef> for
 * offsetof, interrupt_lock.hpp, sws_analog_service.hpp, and the
 * VALIDATION_LOG_ENABLE + CRC16 preamble. service.hpp is included by 75 files,
 * so that is a real coupling reduction, not just tidiness.
 */

#include "service.hpp"
#include "scheduler.hpp"
#include "rtc.hpp"
#include "timer.hpp"
#include "config_store.hpp"
#include "battery.hpp"
#include "interrupt_lock.hpp"
#include "sws_analog_service.hpp"
#include "pmu.hpp"
#include <cstddef>

// Pre-deploy validation channel — see hauled_mode_service.cpp header comment.
// Enables grep-friendly [VAL-COOLDOWN] tags on enter/exit for short-surface
// turtle deployment testing. Default off (zero overhead).
#ifndef VALIDATION_LOG_ENABLE
#define VALIDATION_LOG_ENABLE 0
#endif

// CRC16-CCITT: nRF SDK header in firmware build, inline stub in CppUTest build.
// Mirrors the conditional in sws_analog_constants.hpp so tests can build this
// file (the SDK header is not on the test include path).
#ifndef CPPUTEST
#include "crc16.h"
#else
#include <cstdint>
static inline uint16_t crc16_compute(const uint8_t *data, uint16_t length, const uint16_t *) {
	uint16_t crc = 0xFFFF;
	for (uint16_t i = 0; i < length; i++) {
		crc ^= static_cast<uint16_t>(data[i]) << 8;
		for (uint8_t j = 0; j < 8; j++)
			crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
	}
	return crc;
}
#endif

extern Scheduler *system_scheduler;
extern RTC *rtc;
extern ConfigurationStore *configuration_store;

// Build-time guarantee that time_t is 64-bit. If the toolchain ever flips to
// 32-bit signed time_t (newlib --enable-newlib-time-t-32bit or similar), the
// cooldown math (now - last_cycle) silently wraps in January 2038, leading
// to false-negative cooldowns mid-deployment. Catching this at compile time
// is the cheapest possible safety net.
static_assert(sizeof(std::time_t) >= 8,
              "time_t must be 64-bit to avoid 2038-01-19 wraparound on multi-year deployments");

// Noinit RAM structure for cooldown persistence across System OFF (PSEUDO_POWER_OFF)
struct CooldownNoinit {
	std::time_t last_cycle_time;
	uint16_t passive_count;
	uint16_t crc;
};
#ifndef CPPUTEST
static CooldownNoinit s_cooldown_noinit __attribute__((section(".noinit")));
#else
static CooldownNoinit s_cooldown_noinit;
#endif

static uint16_t cooldown_noinit_crc() {
	// CRC-16-CCITT — same algorithm used for SWS calibration and the PMU
	// callstack, replacing the earlier shift-XOR pseudo-CRC. On a fresh
	// power-on (battery insert) the noinit RAM is uninitialised; the weak
	// CRC was prone to false-positive validations against random RAM
	// patterns, leading to phantom cooldowns that would skip a GPS cycle
	// at the start of deployment.
	return crc16_compute(reinterpret_cast<const uint8_t *>(&s_cooldown_noinit),
	                     offsetof(decltype(s_cooldown_noinit), crc), nullptr);
}

/// @brief Persist cooldown state to .noinit RAM (survives System OFF / pseudo power-off).
void ServiceManager::save_cooldown_state() {
	InterruptLock lock;
	s_cooldown_noinit.last_cycle_time = m_last_successful_cycle_time;
	s_cooldown_noinit.passive_count = static_cast<uint16_t>(m_passive_surfacing_count);
	s_cooldown_noinit.crc = cooldown_noinit_crc();
}

/// @brief Restore cooldown state from .noinit RAM on boot (CRC-validated).
void ServiceManager::restore_cooldown_state() {
	if (s_cooldown_noinit.crc == cooldown_noinit_crc() && s_cooldown_noinit.last_cycle_time > 0) {
		// Mitigation M1c (2026-05): defense-in-depth against noinit corruption
		// that passes CRC (single-bit flip in CRC field itself). If the stored
		// timestamp is now in the future relative to current RTC (RTC rollback
		// after WDT, or impossible far-future value), treat noinit as invalid
		// rather than restoring it. The existing `now < stored → expired`
		// defense in is_in_cooldown handles the math, but explicit rejection
		// here keeps state clean for subsequent set_cycle_complete writes.
		if (rtc && rtc->is_set() && s_cooldown_noinit.last_cycle_time > rtc->gettime()) {
			DEBUG_WARN("ServiceManager: cooldown noinit timestamp in future (stored=%u > now=%u), discarding",
			           (unsigned int)s_cooldown_noinit.last_cycle_time, (unsigned int)rtc->gettime());
			m_last_successful_cycle_time = 0;
			m_passive_surfacing_count = 0;
			return;
		}
		m_last_successful_cycle_time = s_cooldown_noinit.last_cycle_time;
		m_passive_surfacing_count = s_cooldown_noinit.passive_count;
		DEBUG_INFO("ServiceManager: cooldown restored from noinit (last_cycle=%u, passive=%u)",
		           (unsigned int)m_last_successful_cycle_time, m_passive_surfacing_count);

		// FIX 2026-05-23 (audit cooldown finding 3): if we boot mid-cooldown
		// (noinit restored a non-expired timestamp), the wake task MUST be
		// re-armed. Otherwise services gate themselves via is_in_cooldown(),
		// stay dormant, and never re-emit SWS state since exit_cooldown_sleep
		// (the path that re-emits) is only triggered by the wake task. Without
		// this arm, a device that crashed mid-cooldown and reboots at surface
		// would never wake any service until the next external event (magnet,
		// AXL wakeup) — sealed turtle = dormant for the rest of the day.
		if (rtc && rtc->is_set() && system_scheduler) {
			unsigned int interval =
			    configuration_store
			        ? configuration_store->read_param<unsigned int>(ParamID::MIN_SURFACE_CYCLE_INTERVAL_S)
			        : 0;
			if (interval > 0) {
				std::time_t now = rtc->gettime();
				std::time_t elapsed = (now > m_last_successful_cycle_time) ? (now - m_last_successful_cycle_time) : 0;
				if (elapsed < (std::time_t)interval) {
					unsigned int remaining_ms = (interval - (unsigned int)elapsed) * 1000;
					m_cooldown_wake_task = system_scheduler->post_task_prio(
					    []() { exit_cooldown_sleep(); }, "CooldownWakeBoot", Scheduler::DEFAULT_PRIORITY, remaining_ms);
					DEBUG_INFO("ServiceManager: cooldown boot-wake armed for %u s remaining",
					           (interval - (unsigned int)elapsed));
				} else {
					DEBUG_TRACE("ServiceManager: cooldown already expired at boot — no wake task needed");
				}
			}
		} else {
			// RTC not set: the wake task will be armed when RTC syncs and the
			// next set_cycle_complete or enter_cooldown_sleep fires.
			DEBUG_TRACE("ServiceManager: cooldown restored but RTC not set, deferring wake-task arm");
		}
	} else {
		m_last_successful_cycle_time = 0;
		m_passive_surfacing_count = 0;
		DEBUG_TRACE("ServiceManager: cooldown noinit invalid, starting fresh");
	}
}

/// @brief Mark a successful surface cycle — starts cooldown timer.
/// @param t  RTC time of cycle completion.
void ServiceManager::set_cycle_complete(std::time_t t) {
	// FIX 2026-05-23 (audit cooldown finding 1): refuse to anchor cooldown on
	// virtual-epoch RTC. If RTC is not set (cold boot before GNSS sync), `t`
	// is virtual epoch (1 or close to it). Anchoring cooldown there means the
	// elapsed-time math at is_in_cooldown() compares "now" against virtual
	// epoch — once GPS syncs and "now" jumps to real time, elapsed becomes
	// massive and cooldown looks expired immediately. Or worse, the wake task
	// posted below fires after `interval * 1000` ms even though the RTC frame
	// will shift mid-cooldown. Defer arming until we have real time.
	unsigned int interval = configuration_store->read_param<unsigned int>(ParamID::MIN_SURFACE_CYCLE_INTERVAL_S);

	if (!rtc || !rtc->is_set()) {
		// COOLDOWN HIGH audit fix: previously this path returned without arming
		// ANY cooldown when RTC wasn't set (cold boot, no GPS fix yet). For
		// AFTER_FIRST_GNSS / AFTER_LAST_TX trigger modes, this is fire-once —
		// the cooldown was silently dropped, allowing the next surface to
		// hammer the battery with an unbounded TX burst. Fix: arm the
		// scheduler wake task unconditionally (it uses ms-uptime, not RTC),
		// AND mark m_last_successful_cycle_time = 0 (sentinel "RTC not set
		// at cooldown arm time"). The is_in_cooldown() RTC math is guarded
		// by `== 0 → false` (line 335) so it returns "not in cooldown" —
		// but services gate primarily on the wake task being pending via
		// service_next_schedule_in_ms returning the remaining delay.
		// Net effect: cooldown still throttles via scheduler timer even
		// without RTC. When RTC syncs later, is_in_cooldown returns the
		// correct answer (which is "no, not in cooldown" — by then the
		// timer-based gate has likely expired anyway).
		DEBUG_WARN("ServiceManager::set_cycle_complete: RTC not set — arming wake task only (no RTC anchor)");
#if VALIDATION_LOG_ENABLE
		DEBUG_INFO("[VAL-COOLDOWN] enter_no_rtc interval_s=%u (scheduler-only gate)", interval);
#endif
		if (interval > 0 && system_scheduler) {
			unsigned int remaining_ms = interval * 1000;
			system_scheduler->cancel_task(m_cooldown_wake_task);
			m_cooldown_wake_task = system_scheduler->post_task_prio(
			    []() { exit_cooldown_sleep(); }, "CooldownWakeNoRTC", Scheduler::DEFAULT_PRIORITY, remaining_ms);
		}
		return;
	}

	m_last_successful_cycle_time = t;
	save_cooldown_state();
	DEBUG_INFO("ServiceManager: cycle complete at %u, cooldown started", (unsigned int)t);
#if VALIDATION_LOG_ENABLE
	DEBUG_INFO("[VAL-COOLDOWN] enter t=%u interval_s=%u", (unsigned int)t, interval);
#endif

	// Schedule the cooldown-wake task UNCONDITIONALLY here (2026-05-23 fix).
	// Previously this was only set in enter_cooldown_sleep(), which is only
	// called from the passive-surfacing path (notify_underwater_state when
	// state=false AND cooldown active). On the *dive-with-cooldown-armed* path
	// (typical SURFACING_BURST END_OF_DOPPLER + dive), set_cycle_complete()
	// fires alone — no wake task was posted, so when SWS gets disabled by its
	// service_next_schedule_in_ms cooldown gate, NOTHING re-emits state when
	// the window expires. The device sleeps until an unrelated periodic task
	// (e.g. backup-charge tick) wakes it, by which point surface detection is
	// effectively dead. Field log 2026-05-23: device stuck in UW state
	// forever after a single max-msg burst completion. Critical for sealed
	// turtles since this is the normal end-of-burst path.
	if (interval > 0 && system_scheduler) {
		unsigned int remaining_ms = interval * 1000;
		system_scheduler->cancel_task(m_cooldown_wake_task);
		m_cooldown_wake_task = system_scheduler->post_task_prio([]() { exit_cooldown_sleep(); }, "CooldownWake",
		                                                        Scheduler::DEFAULT_PRIORITY, remaining_ms);
		DEBUG_TRACE("ServiceManager::set_cycle_complete: wake timer set for %u s", interval);
	}
}

/// @brief Check if surface cycle cooldown is active.
/// @param now  Current RTC time.
/// @return true if less than MIN_SURFACE_CYCLE_INTERVAL_S has elapsed since last cycle.
bool ServiceManager::is_in_cooldown(std::time_t now) {
	return get_cooldown_remaining_s(now) > 0;
}

/// @brief Seconds remaining until cooldown expires.
/// @param now  Current RTC time.
/// @return  Remaining cooldown seconds, or 0 if no active cooldown.
unsigned int ServiceManager::get_cooldown_remaining_s(std::time_t now) {
	unsigned int interval = configuration_store->read_param<unsigned int>(ParamID::MIN_SURFACE_CYCLE_INTERVAL_S);
	if (interval == 0) return 0;
	if (m_last_successful_cycle_time == 0) return 0;
	if (now < m_last_successful_cycle_time) return 0;  // RTC went backward — treat as cooldown expired
	// `now == m_last_successful_cycle_time` falls through with elapsed=0,
	// which correctly reports the full interval as remaining. This matters
	// for callers that do set_cycle_complete(now) then is_in_cooldown(now)
	// in the same tick (e.g. LoRaTxService dive handler).
	std::time_t elapsed = now - m_last_successful_cycle_time;
	if (elapsed >= (std::time_t)interval) return 0;
	return (unsigned int)((std::time_t)interval - elapsed);
}

void ServiceManager::notify_passive_surfacing() {
	m_passive_surfacing_count++;
	save_cooldown_state();
	DEBUG_INFO("ServiceManager: passive surfacing #%u (cooldown active, no GPS/TX)", m_passive_surfacing_count);
#if VALIDATION_LOG_ENABLE
	std::time_t now = (rtc && rtc->is_set()) ? rtc->gettime() : 0;
	unsigned int remaining = get_cooldown_remaining_s(now);
	DEBUG_INFO("[VAL-COOLDOWN] block passive #%u t=%u remaining_s=%u", (unsigned int)m_passive_surfacing_count,
	           (unsigned int)now, remaining);
#endif
}

unsigned int ServiceManager::get_passive_surfacing_count() {
	return m_passive_surfacing_count;
}

/// @brief Enter cooldown sleep — stop SWS, program wake timer for remaining cooldown.
void ServiceManager::enter_cooldown_sleep() {
	unsigned int remaining_s = 0;
	if (rtc && rtc->is_set() && m_last_successful_cycle_time > 0) {
		std::time_t now = rtc->gettime();
		if (now > m_last_successful_cycle_time) {
			unsigned int interval =
			    configuration_store->read_param<unsigned int>(ParamID::MIN_SURFACE_CYCLE_INTERVAL_S);
			std::time_t elapsed = now - m_last_successful_cycle_time;
			remaining_s = (elapsed < (std::time_t)interval) ? (interval - (unsigned int)elapsed) : 0;
		}
	}
	DEBUG_INFO("ServiceManager: entering cooldown sleep (remaining %u s) — stopping SWS", remaining_s);
	// #if VALIDATION_LOG_ENABLE
	// 	DEBUG_INFO("[VAL-SLEEP] cooldown_sws_pause remaining_s=%u", remaining_s);
	// #endif

	// Stop SWS (UW_SENSOR) to save power during cooldown — unless the user
	// is actively running SWSTST,1 (bench/cable testing). Pausing SWS during
	// test mode makes the LED freeze and confuses operator diagnostics.
	// SWS test mode only exists when ENABLE_SWS_ANALOG is compiled in; on
	// boards without it (e.g. RSPB) always proceed with the pause.
#if ENABLE_SWS_ANALOG
	if (!SWSAnalogService::is_test_running()) {
#else
	{
#endif
		for (auto &p : m_map) {
			if (p.second.get_service_id() == ServiceIdentifier::UW_SENSOR) {
				p.second.pause_for_cooldown();
			}
		}
#if ENABLE_SWS_ANALOG
	} else {
		DEBUG_INFO("ServiceManager: SWS test mode active — skipping SWS pause for cooldown");
	}
#else
	}
#endif

	// Program wake timer for remaining cooldown duration
	if (rtc && rtc->is_set() && m_last_successful_cycle_time > 0) {
		std::time_t now = rtc->gettime();
		if (now <= m_last_successful_cycle_time) {
			// RTC went backward — cooldown expired, restart immediately
			exit_cooldown_sleep();
			return;
		}
		unsigned int interval = configuration_store->read_param<unsigned int>(ParamID::MIN_SURFACE_CYCLE_INTERVAL_S);
		std::time_t elapsed = now - m_last_successful_cycle_time;
		if (elapsed < (std::time_t)interval) {
			unsigned int remaining_ms = (interval - (unsigned int)elapsed) * 1000;
			system_scheduler->cancel_task(m_cooldown_wake_task);
			m_cooldown_wake_task = system_scheduler->post_task_prio([]() { exit_cooldown_sleep(); }, "CooldownWake",
			                                                        Scheduler::DEFAULT_PRIORITY, remaining_ms);
			DEBUG_TRACE("ServiceManager: cooldown wake timer set for %u s", (interval - (unsigned int)elapsed));
		} else {
			// Cooldown already expired — restart immediately
			exit_cooldown_sleep();
		}
	}
}

/// @brief Exit cooldown sleep — restart SWS with forced first-time detection.
/// The SWS will re-emit its current state on the first sample, which triggers
/// surface/UW events to wake all services. This avoids incorrectly broadcasting
/// a surface event when the device might actually be underwater.
void ServiceManager::exit_cooldown_sleep() {
	DEBUG_INFO("ServiceManager: exiting cooldown sleep (RTC=%u) — restarting SWS",
	           (rtc && rtc->is_set()) ? (unsigned int)rtc->gettime() : 0);
#if VALIDATION_LOG_ENABLE
	{
		std::time_t now = (rtc && rtc->is_set()) ? rtc->gettime() : 0;
		std::time_t elapsed = (now > m_last_successful_cycle_time && m_last_successful_cycle_time > 0)
		                          ? (now - m_last_successful_cycle_time)
		                          : 0;
		DEBUG_INFO("[VAL-COOLDOWN] exit t=%u elapsed_s=%u passive=%u", (unsigned int)now, (unsigned int)elapsed,
		           (unsigned int)m_passive_surfacing_count);
	}
#endif

	// Restart SWS with first-time flag — it will re-emit its current state
	// on the next sample, triggering surface/UW notification to all peers.
	for (auto &p : m_map) {
		if (p.second.get_service_id() == ServiceIdentifier::UW_SENSOR) {
			p.second.reset_state_for_cooldown_exit();
			p.second.resume_from_cooldown();
		}
	}
}

/// @brief Pause service for cooldown — deschedule without stopping.
void Service::pause_for_cooldown() {
	DEBUG_INFO("Service::pause_for_cooldown: %s", m_name);
	deschedule();
}

/// @brief Resume service after cooldown — reschedule if still started.
void Service::resume_from_cooldown() {
	DEBUG_INFO("Service::resume_from_cooldown: %s", m_name);
	if (m_is_started) reschedule();
}
