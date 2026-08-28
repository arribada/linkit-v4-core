/**
 * @file service.cpp
 * @brief Service framework — Service lifecycle + ServiceManager orchestration + cooldown.
 */

#include "service.hpp"
#include "scheduler.hpp"
#include "rtc.hpp"
#include "timeutils.hpp"
#include "timer.hpp"
#include "config_store.hpp"
#include "battery.hpp"
#include "interrupt_lock.hpp"
#include "sws_analog_service.hpp"
#include "../sm/error.hpp"
#include "pmu.hpp"
#include "rate_limiter.hpp"
#include "hauled_mode_service.hpp"
#include "moored_mode_service.hpp"
#if ENABLE_AXL_SENSOR
#include "axl_sensor_service.hpp"  // AXLSensorPort::WAKEUP_TRIGGERED
#endif
#include <cstddef>
#ifdef BENCH_TEST
#include <cstdio>
#include <string>
#endif
#include <stdexcept>
#include <variant>

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

extern Timer *system_timer;
extern Scheduler *system_scheduler;
extern RTC *rtc;
extern BatteryMonitor *battery_monitor;

/// @brief Register a service and assign a unique ID.
/// @param s  Service to register.
/// @return Unique ID for this service instance.
unsigned int ServiceManager::add(Service &s) {
	m_map.insert({ m_unique_identifier, s });
	DEBUG_TRACE("ServiceManager::add: service=%s added id=%u", s.get_name(), m_unique_identifier);
	return m_unique_identifier++;
}

/// @brief Unregister a service.
void ServiceManager::remove(Service &s) {
	DEBUG_TRACE("ServiceManager::remove: service=%s added", s.get_name());
	m_map.erase(s.get_unique_id());
}

/// @brief TEST-ONLY: wipe all ServiceManager static state back to construction
/// defaults. See declaration. Not called from production code.
void ServiceManager::reset() {
	m_map.clear();
	m_unique_identifier = 0;
	m_data_notification_callback = nullptr;
	m_last_successful_cycle_time = 0;
	m_passive_surfacing_count = 0;
	m_cooldown_wake_task = Scheduler::TaskHandle{};
}


/// @brief Start all registered services (called on FSM transition to Operational).
/// @param data_notification_callback  Global event callback for FSM.
void ServiceManager::startall(std::function<void(ServiceEvent &)> data_notification_callback) {
	m_data_notification_callback = data_notification_callback;
	restore_cooldown_state();
	RateLimiter::restore_state();
	HauledModeService::restore_state();
	MooredModeService::restore_state();
	for (auto const &p : m_map) {
		DEBUG_TRACE("ServiceManager::startall: starting %s id=%u", p.second.get_name(), p.first);
		p.second.start(data_notification_callback);
	}
}

/// @brief Stop all registered services (called on FSM transition out of Operational).
///
/// Per-service hardening for the sealed-deployment case: each Service::stop()
/// can call deschedule(), service_cancel(), notify_service_inactive() and
/// service_term() — any of which can throw (variant access, scheduler queue)
/// OR hang (I2C device unresponsive during sensor term, BLE SoftDevice quirk,
/// SMD SPI cascade). Without protection here, the *first* misbehaving service
/// aborts the whole stop sequence, leaving services N+1..K still scheduled.
/// Result: stale tasks fire after the FSM has already transited to e.g.
/// ConfigurationState or OffState, mutating shared state from a context where
/// they shouldn't run.
///
/// Defense in depth:
/// 1. WDT kick before every service stop — a 15-min budget per service rather
///    than per stopall. On a sealed turtle, a single hung service must not
///    consume the budget the *next* service needs to clean up.
/// 2. try/catch per service — exception in one stop() doesn't abort the loop.
///
/// We can't protect against true hangs (no preemption), but the WDT kick
/// pattern means a hang reaches the 15-min cap at the offending service, not
/// after partial-stop of N-1 services. A WDT reset then puts the device back
/// in BootState with a fresh start, which is recoverable; half-stopped
/// services persisting into the next FSM state are not.
void ServiceManager::stopall() {
	// Cancel any pending cooldown-wake task. Otherwise a stale wake lambda
	// queued during an active cooldown could fire after we transition back
	// into Operational and undo a fresh cooldown's SWS pause.
	system_scheduler->cancel_task(m_cooldown_wake_task);
	for (auto const &p : m_map) {
		PMU::kick_watchdog();
		try {
			p.second.stop();
		} catch (const std::exception &e) {
			DEBUG_ERROR("ServiceManager::stopall: %s stop() threw std::exception: %s", p.second.get_name(), e.what());
		} catch (...) {
			DEBUG_ERROR("ServiceManager::stopall: %s stop() threw unknown — continuing", p.second.get_name());
		}
	}
	PMU::kick_watchdog();
}

/// @brief Broadcast a peer event to all services except the originator.
/// @param event  Service event to broadcast.
void ServiceManager::notify_peer_event(ServiceEvent &event) {
	// HauledModeService funnel — feeds every UW transition (dive/surface) into
	// the hauled classifier. Hooked here rather than in a Service subclass so
	// there's a single ground-truth dispatch site (Plan 1 step 3).
	if (event.event_source == ServiceIdentifier::UW_SENSOR
	    && event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
		std::time_t now = (rtc && rtc->is_set()) ? rtc->gettime() : 0;
		if (now > 0) {
			HauledModeService::on_underwater_event(std::get<bool>(event.event_data), now);
		}
	}

	// MooredModeService funnel (2026-08) — same rationale, two sources of
	// evidence. Both are no-ops unless MOORED_DETECT_EN is set, so a build that
	// never enables the feature pays one enum compare per event.
	//
	// Only GPSEventType::FIX feeds the classifier: FASTLOC / CLOUDLOCATE /
	// degraded PVT carry accuracy far coarser than a sensible MOORED_RADIUS_M
	// (hundreds of metres to kilometres), and would read as movement on a
	// vessel that never left its berth.
	if (event.event_source == ServiceIdentifier::GNSS_SENSOR
	    && event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
		if (auto *gps = std::get_if<GPSLogEntry>(&event.event_data)) {
			if (gps->info.valid && gps->info.event_type == GPSEventType::FIX) {
				std::time_t now = (rtc && rtc->is_set()) ? rtc->gettime() : 0;
				MooredModeService::on_gnss_fix(gps->info.lat, gps->info.lon, gps->info.gSpeed, now);
			}
		}
	}
#if ENABLE_AXL_SENSOR
	// Accelerometer wake-up: the cheap movement oracle between two GNSS points.
	// Needs a real RTC — the hold-off that keeps swell from burning the budget
	// is measured in wall-clock seconds, and running it against the virtual
	// pre-fix epoch would make the debounce meaningless.
	if (event.event_source == ServiceIdentifier::AXL_SENSOR
	    && event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
		if (auto *sensor = std::get_if<ServiceSensorData>(&event.event_data)) {
			if (sensor->port[AXLSensorPort::WAKEUP_TRIGGERED]) {
				std::time_t now = (rtc && rtc->is_set()) ? rtc->gettime() : 0;
				if (now > 0) MooredModeService::on_motion_event(now);
			}
		}
	}
#endif

	for (auto const &p : m_map) {
		if (p.first != event.event_originator_unique_id) p.second.notify_peer_event(event);
	}
}

unsigned int ServiceManager::get_unique_id(const char *name) {
	for (auto const &p : m_map) {
		if (std::string(p.second.get_name()) == std::string(name)) return p.second.get_unique_id();
	}

	throw ErrorCode::RESOURCE_NOT_AVAILABLE;
}

Logger *ServiceManager::get_logger(ServiceIdentifier service_id) {
	for (auto const &p : m_map) {
		if (p.second.get_service_id() == service_id && p.second.get_logger()) return p.second.get_logger();
	}

	return nullptr;
}

/// @brief Inject an event directly to the FSM callback (bypasses peer broadcast).
/// @param event  Event to inject.
void ServiceManager::inject_event(ServiceEvent &event) {
	if (m_data_notification_callback) m_data_notification_callback(event);
}

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

/// @brief Constructor — register with ServiceManager, init state.
/// @param service_id  Unique service identifier.
/// @param name        Debug name (string literal).
/// @param logger      Optional persistent logger.
Service::Service(ServiceIdentifier service_id, const char *name, Logger *logger) {
	m_is_started = false;
	m_name = name;
	m_is_underwater = false;
	m_service_id = service_id;
	m_logger = logger;
	m_last_schedule = Service::SCHEDULE_DISABLED;
	m_unique_id = ServiceManager::add(*this);
}

Service::~Service() {
	ServiceManager::remove(*this);
}

unsigned int Service::get_unique_id() {
	return m_unique_id;
}
const char *Service::get_name() {
	return m_name;
}
ServiceIdentifier Service::get_service_id() {
	return m_service_id;
}
Logger *Service::get_logger() {
	return m_logger;
}
void Service::set_logger(Logger *logger) {
	m_logger = logger;
}

/// @brief GNSS MED #4 audit fix impl — cancel the rescheduler's safety-net
/// timeout. Used by derived services that short-circuit service_initiate.
void Service::cancel_safety_timeout() {
	if (system_scheduler) system_scheduler->cancel_task(m_task_timeout);
}

/// @brief Start the service — init, register callback, schedule first execution.
/// @param data_notification_callback  Global event callback.
void Service::start(std::function<void(ServiceEvent &)> data_notification_callback) {
	DEBUG_TRACE("Service::start: service %s started", m_name);
	m_is_started = true;
	m_is_initiated = false;
	m_data_notification_callback = data_notification_callback;
	m_last_schedule = Service::SCHEDULE_DISABLED;
	service_init();
	reschedule();
}

/// @brief Stop the service — cancel active task, notify inactive, terminate.
void Service::stop() {
	DEBUG_TRACE("Service::stop: service %s stopped (is_started=%u)", m_name, (unsigned int)m_is_started);
	if (m_is_started) {
		m_is_started = false;
		deschedule();
		service_cancel();
		if (m_is_initiated) notify_service_inactive();
		m_is_initiated = false;
		service_term();
	}
}

unsigned int Service::get_last_schedule() {
	return m_last_schedule;
}

bool Service::is_underwater_deferred() {
	return m_is_underwater;
}

/// @brief Handle underwater state change — deschedule when submerged, reschedule on surfacing.
/// @param state  true = submerged, false = surfaced.
void Service::notify_underwater_state(bool state) {
	if (service_is_usable_underwater()) return;  // Don't care since the sensor can be used underwater
	//DEBUG_TRACE("Service::notify_underwater_state: service %s notify UW %u", m_name, state);
	m_is_underwater = state;
	if (m_is_underwater) {
		deschedule();
		service_cancel();
		if (m_is_initiated) notify_service_inactive();
		m_is_initiated = false;
	} else {
		// Check cooldown: skip reschedule if a successful cycle completed recently
		if (rtc && rtc->is_set() && ServiceManager::is_in_cooldown(rtc->gettime())) {
			// Log passive surfacing and enter cooldown sleep once per surfacing event
			if (m_service_id == ServiceIdentifier::GNSS_SENSOR) {
				ServiceManager::notify_passive_surfacing();
				ServiceManager::enter_cooldown_sleep();  // Stop SWS + program wake timer
			}
			DEBUG_INFO("Service::notify_underwater_state: service %s skipped (cooldown active)", m_name);
			return;
		}
		bool immediate;
		if (service_is_triggered_on_surfaced(immediate)) reschedule(immediate);
	}
}

/// @brief Handle peer events — routes UW state changes and triggered events.
/// @param event  Peer service event. May be overridden by subclass for custom handling.
void Service::notify_peer_event(ServiceEvent &event) {
	//DEBUG_TRACE("Service::notify_peer_event: src=%u type=%u", (unsigned int)event.event_source, (unsigned int)event.event_type);
	bool immediate = true;
	if (event.event_source == ServiceIdentifier::UW_SENSOR && event.event_type == ServiceEventType::SERVICE_LOG_UPDATED)
		notify_underwater_state(std::get<bool>(event.event_data));
	else if (service_is_triggered_on_event(event, immediate)) {
		reschedule(immediate);
	}
};

bool Service::is_started() {
	return m_is_started;
}

bool Service::is_initiated() {
	return m_is_initiated;
}

void Service::service_reschedule(bool immediate) {
	reschedule(immediate);
}

bool Service::service_is_scheduled() {
	return m_last_schedule != Service::SCHEDULE_DISABLED;
}

void Service::service_log(ServiceEventData *event_data, void *entry) {
	if (m_logger && entry != nullptr) m_logger->write(entry);
	if (event_data) notify_log_updated(*event_data);
}

/// @brief Mark service as complete — log, notify inactive, optionally reschedule.
/// @param event_data       Optional event payload for peer notification.
/// @param entry            Optional raw log entry for persistent logger.
/// @param shall_reschedule true to reschedule after completion.
void Service::service_complete(ServiceEventData *event_data, void *entry, bool shall_reschedule) {
	DEBUG_TRACE("Service::service_complete: service %s", m_name);
	if (!m_is_initiated) {
		if (!m_is_started) {
			// Service was stopped (e.g., state transition during async sensor read) — expected, ignore silently
			DEBUG_TRACE("Service::service_complete: service %s async completion after stop (ignored)", m_name);
		} else {
			DEBUG_WARN("Service::service_complete: service %s completed without being initiated", m_name);
		}
		return;
	}
	m_is_initiated = false;
	notify_service_inactive();
	service_log(event_data, entry);
	if (shall_reschedule) reschedule();
}

void Service::service_set_log_header_time(LogHeader &header, std::time_t time) {
	uint16_t year;
	uint8_t month, day, hour, min, sec;

	convert_datetime_to_epoch(time, year, month, day, hour, min, sec);

	header.year = year;
	header.month = month;
	header.day = day;
	header.hours = hour;
	header.minutes = min;
	header.seconds = sec;
}

void Service::service_active() {
	notify_service_active();
}

std::time_t Service::service_current_time() {
	return rtc->gettime();
}

bool Service::service_is_time_known() {
	return rtc->is_set();
}

uint64_t Service::service_current_timer() {
	return system_timer->get_counter();
}

void Service::service_set_time(std::time_t t) {
	rtc->settime(t);
}

void Service::service_update_battery() {
	return battery_monitor->update();
}

uint16_t Service::service_get_voltage() {
	return battery_monitor->get_voltage();
}

uint8_t Service::service_get_level() {
	return battery_monitor->get_level();
}

bool Service::service_is_battery_level_low() {
	return battery_monitor->is_battery_low();
}

#ifdef BENCH_TEST
/// Record the scheduling decision for the bench console. Deliberately placed on
/// EVERY branch of reschedule(), including those that plan nothing: a test that
/// only sees the success path cannot tell "this mode scheduled an emission" from
/// "this mode scheduled nothing and I am reading the previous mode's value".
#define BENCH_SCHED_NOTE(ms, why)  \
	do {                           \
		m_bench_sched_ms = (ms);   \
		m_bench_sched_why = (why); \
	} while (0)

/// @brief Bench-only: one-line schedule report over ALL registered services.
std::string ServiceManager::bench_schedule_report() {
	std::string out;
	for (auto const &kv : m_map) {
		Service &s = kv.second;
		char buf[96];
		if (s.bench_sched_ms() == Service::SCHEDULE_DISABLED)
			snprintf(buf, sizeof(buf), "%s=none(%s) ", s.bench_name(), s.bench_sched_why());
		else
			snprintf(buf, sizeof(buf), "%s=%ums(%s) ", s.bench_name(), s.bench_sched_ms(), s.bench_sched_why());
		out += buf;
	}
	return out;
}
#else
#define BENCH_SCHED_NOTE(ms, why) \
	do {                          \
	} while (0)
#endif

/// @brief Internal: compute next schedule, post task to scheduler, arm timeout.
/// @param immediate  true to schedule with 0 delay.
void Service::reschedule(bool immediate) {
	DEBUG_TRACE("Service::reschedule: service %s", m_name);
	deschedule();
	// Underwater short-circuit: re-arming here is wasted work and creates a
	// log-spam loop. Sequence: notify_underwater_state(true) calls deschedule()
	// then service_cancel() — and for services like GPSService, service_cancel()
	// invokes service_complete() with shall_reschedule=true, which lands back
	// here. The m_task_period lambda below already gates service_initiate() on
	// m_is_underwater, so the task would be a no-op, BUT it still arms a fresh
	// m_task_timeout that fires ~70 s later, runs service_cancel() + reschedule()
	// again, and the cycle repeats every ~70 s until resurface. The fix is to
	// skip arming entirely: notify_underwater_state(false) calls reschedule()
	// through the normal path at resurface (after clearing m_is_underwater).
	if (m_is_underwater) {
		DEBUG_TRACE("Service::reschedule: service %s skipped (underwater)", m_name);
		BENCH_SCHED_NOTE(SCHEDULE_DISABLED, "underwater");
		return;
	}
	if (is_started()) {
		if (service_is_enabled()) {
			unsigned int next_schedule = immediate ? 0 : service_next_schedule_in_ms();
			if (!m_is_initiated) {
				if (next_schedule != SCHEDULE_DISABLED) {
					DEBUG_TRACE("Service::reschedule: service %s scheduled in %u msecs", m_name, next_schedule);
					BENCH_SCHED_NOTE(next_schedule, "scheduled");
					m_last_schedule = next_schedule;
					m_task_period = system_scheduler->post_task_prio(
					    [this]() {
						    // Barrier against unhandled exceptions in service code.
						    // `service_initiate()` is virtual and runs user-defined logic that
						    // may throw (e.g. ConfigStore::CONFIG_STORE_CORRUPTED, ErrorCode
						    // enums, std::out_of_range from at(), bad_variant_access, etc.).
						    // Without this catch, the exception escapes the scheduler task
						    // runner and reaches std::terminate → __verbose_terminate_handler
						    // → abort(), which on this platform hangs in fputc until WDT.
						    // Catching here logs the failure and lets the service be
						    // rescheduled on the next event/timer instead of bricking the FW.
						    try {
							    unsigned int timeout_ms = service_next_timeout();
							    DEBUG_TRACE("Service::reschedule: service %s time out in %u msecs", m_name, timeout_ms);
							    if (timeout_ms) {
								    m_task_timeout = system_scheduler->post_task_prio(
								        [this]() {
									        try {
										        DEBUG_TRACE("Service::reschedule: service %s timed out", m_name);
										        service_cancel();
										        if (m_is_initiated) notify_service_inactive();
										        m_is_initiated = false;
										        reschedule();
									        } catch (ErrorCode e) {
										        DEBUG_ERROR("Service::reschedule: timeout ErrorCode=%d in service %s — "
												            "recovering",
												            (int)e, m_name);
										        m_is_initiated = false;
									        } catch (const std::exception &e) {
										        DEBUG_ERROR("Service::reschedule: timeout std::exception in service "
												            "%s: %s — recovering",
												            m_name, e.what());
										        m_is_initiated = false;
									        } catch (...) {
										        DEBUG_ERROR("Service::reschedule: timeout unknown exception in service "
												            "%s — recovering",
												            m_name);
										        m_is_initiated = false;
									        }
								        },
								        "ServiceTimeoutPeriod", Scheduler::DEFAULT_PRIORITY, timeout_ms);
							    }

							    if (!m_is_underwater) {
								    DEBUG_TRACE("Service::reschedule: service %s active", m_name);
								    m_is_initiated = true;
								    if (service_is_active_on_initiate()) notify_service_active();
								    service_initiate();
							    } else {
								    DEBUG_TRACE("Service::reschedule: service %s can't run underwater", m_name);
							    }
						    } catch (ErrorCode e) {
							    // Firmware-thrown enum (CONFIG_STORE_CORRUPTED, RESOURCE_NOT_AVAILABLE, etc.)
							    DEBUG_ERROR("Service::reschedule: ErrorCode=%d in service %s task — recovering", (int)e,
								            m_name);
							    // Cancel the safety-net timeout we just armed: otherwise the service
							    // is stuck waiting for `timeout_ms` before any retry. (See QA review B2.)
							    system_scheduler->cancel_task(m_task_timeout);
							    m_is_initiated = false;
						    } catch (const std::bad_variant_access &e) {
							    // Type mismatch when reading config_store params (variant<>).
							    DEBUG_ERROR("Service::reschedule: bad_variant_access in service %s task (config type "
								            "mismatch?) — recovering",
								            m_name);
							    system_scheduler->cancel_task(m_task_timeout);
							    m_is_initiated = false;
						    } catch (const std::out_of_range &e) {
							    // Index out of range — typically from std::array::at() or vector::at().
							    DEBUG_ERROR("Service::reschedule: out_of_range in service %s task (%s) — recovering",
								            m_name, e.what());
							    system_scheduler->cancel_task(m_task_timeout);
							    m_is_initiated = false;
						    } catch (const std::exception &e) {
							    // Any other std exception (bad_alloc, runtime_error, …)
							    DEBUG_ERROR("Service::reschedule: std::exception in service %s task: %s — recovering",
								            m_name, e.what());
							    system_scheduler->cancel_task(m_task_timeout);
							    m_is_initiated = false;
						    } catch (...) {
							    // Last-resort barrier. Without this, the exception escapes the
							    // scheduler runner and propagates up to terminate() →
							    // __verbose_terminate_handler → abort() → fputc-hang → WDT
							    // (15 min on this board). Log and recover instead.
							    DEBUG_ERROR("Service::reschedule: unknown exception in service %s task — recovering",
								            m_name);
							    system_scheduler->cancel_task(m_task_timeout);
							    m_is_initiated = false;
						    }
					    },
					    "ServicePeriod", Scheduler::DEFAULT_PRIORITY, next_schedule);
				} else {
					DEBUG_TRACE("Service::reschedule: service %s schedule currently disabled", m_name);
					BENCH_SCHED_NOTE(SCHEDULE_DISABLED, "no-schedule");
				}
			} else {
				DEBUG_TRACE("Service::reschedule: service %s already initiated", m_name);
				BENCH_SCHED_NOTE(next_schedule, "already-initiated");
			}
		} else {
			DEBUG_TRACE("Service::reschedule: service %s is not enabled", m_name);
			BENCH_SCHED_NOTE(SCHEDULE_DISABLED, "not-enabled");
		}
	} else {
		DEBUG_TRACE("Service::reschedule: service %s is stopped", m_name);
		BENCH_SCHED_NOTE(SCHEDULE_DISABLED, "stopped");
	}
}

/// @brief Cancel all pending tasks (period + timeout).
void Service::deschedule() {
	system_scheduler->cancel_task(m_task_timeout);
	system_scheduler->cancel_task(m_task_period);
	// Cancellation is a DECISION that is observable on the bench. Without this
	// line, a dive that cancels a service which is merely "scheduled" (and not
	// "initiated") triggers no pass through reschedule(): the bench value stayed
	// frozen on "scheduled" and an SWS gating test wrongly concluded that the
	// beacon was still transmitting underwater.
	BENCH_SCHED_NOTE(SCHEDULE_DISABLED, "descheduled");
	m_last_schedule = Service::SCHEDULE_DISABLED;
}

/// @brief Broadcast SERVICE_LOG_UPDATED event to all peers via callback.
/// @param data  Event payload (GPS fix, sensor data, etc.).
void Service::notify_log_updated(ServiceEventData &data) {
	if (m_data_notification_callback) {
		ServiceEvent e;
		e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
		e.event_source = m_service_id;
		e.event_data = data;
		e.event_originator_unique_id = m_unique_id;
		m_data_notification_callback(e);
	}
}

/// @brief Broadcast SERVICE_ACTIVE event to all peers.
void Service::notify_service_active() {
	if (m_data_notification_callback) {
		ServiceEvent e;
		e.event_type = ServiceEventType::SERVICE_ACTIVE;
		e.event_source = m_service_id;
		e.event_originator_unique_id = m_unique_id;
		m_data_notification_callback(e);
	}
}

/// @brief Broadcast SERVICE_INACTIVE event to all peers.
void Service::notify_service_inactive() {
	if (m_data_notification_callback) {
		ServiceEvent e;
		e.event_type = ServiceEventType::SERVICE_INACTIVE;
		e.event_source = m_service_id;
		e.event_originator_unique_id = m_unique_id;
		m_data_notification_callback(e);
	}
}

/// @brief Broadcast a custom ServiceEvent type to all peers (no data payload).
/// @param type  Event type (e.g. GNSS_CLOUDLOCATE_READY).
void Service::notify_service_event(ServiceEventType type) {
	if (m_data_notification_callback) {
		ServiceEvent e;
		e.event_type = type;
		e.event_source = m_service_id;
		e.event_originator_unique_id = m_unique_id;
		m_data_notification_callback(e);
	}
}
