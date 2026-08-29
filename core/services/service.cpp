/**
 * @file service.cpp
 * @brief Service framework — Service lifecycle + ServiceManager orchestration.
 * The cooldown lives in service_cooldown.cpp.
 */

#include "service.hpp"
#include "scheduler.hpp"
#include "rtc.hpp"
#include "timeutils.hpp"
#include "timer.hpp"
#include "config_store.hpp"
#include "battery.hpp"
#include "../sm/error.hpp"
#include "pmu.hpp"
#include "rate_limiter.hpp"
#include "hauled_mode_service.hpp"
#include "moored_mode_service.hpp"
#if ENABLE_AXL_SENSOR
#include "axl_sensor_service.hpp"  // AXLSensorPort::WAKEUP_TRIGGERED
#endif
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

/// @brief Default: speak for the services that have not migrated yet.
///
/// Every SCHEDULE_DISABLED becomes off("no-schedule"), which is exactly what
/// reschedule() did with the sentinel before this type existed, and exactly the
/// string the bench report already produces. Behaviour is unchanged until a
/// service overrides this and starts saying what it actually means.
ScheduleDecision Service::service_next_schedule() {
	const unsigned int ms = service_next_schedule_in_ms();
	if (ms == SCHEDULE_DISABLED) return ScheduleDecision::off("no-schedule");
	return ScheduleDecision::run(ms, "scheduled");
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
#define BENCH_SCHED_NOTE(ms, why)     \
	do {                              \
		m_bench_sched_ms = (ms);      \
		m_bench_sched_why = (why);    \
		m_bench_sched_hold_s = 0;     \
	} while (0)

/// @brief A hold: no deadline to run, but a deadline to think again.
#define BENCH_SCHED_HOLD(hold_s, why)        \
	do {                                     \
		m_bench_sched_ms = SCHEDULE_DISABLED; \
		m_bench_sched_why = (why);           \
		m_bench_sched_hold_s = (hold_s);     \
	} while (0)

/// @brief Bench-only: one-line schedule report over ALL registered services.
std::string ServiceManager::bench_schedule_report() {
	std::string out;
	for (auto const &kv : m_map) {
		Service &s = kv.second;
		char buf[96];
		if (s.bench_sched_hold_s())
			snprintf(buf, sizeof(buf), "%s=hold%us(%s) ", s.bench_name(), s.bench_sched_hold_s(),
			         s.bench_sched_why());
		else if (s.bench_sched_ms() == Service::SCHEDULE_DISABLED)
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
#define BENCH_SCHED_HOLD(hold_s, why) \
	do {                              \
	} while (0)
#endif

/// @brief Internal: compute next schedule, post task to scheduler, arm timeout.
/// @param immediate  true to schedule with 0 delay.
void Service::reschedule(bool immediate) {
	DEBUG_TRACE("Service::reschedule: service %s", m_name);
	// Keep the safety-net timeout when a session of ours is already in flight.
	//
	// This used to deschedule() unconditionally, cancelling BOTH tasks, and then
	// return a few lines down at the m_is_initiated test without re-arming
	// anything. m_task_timeout is armed in exactly one place -- run_scheduled_task
	// -- so a reschedule() arriving mid-session destroyed that session's only
	// framework net, permanently, and the service stayed initiated with nothing
	// pending if the device never answered.
	//
	// Not hypothetical, and not confined to one service: GPSService holds an
	// initiated hardware session for tens of seconds to several minutes and is
	// wired straight into this path -- Service::notify_peer_event ->
	// service_is_triggered_on_event -> reschedule(immediate), which
	// GPSService answers true to on every accelerometer wake-up when
	// GNSS_TRIGGER_ON_AXL_WAKEUP is set (the moored-mode configuration). On a
	// moving animal that is most sessions. The M10Q driver arms its own timeouts
	// while receiving, so the usual cost is the loss of defence in depth rather
	// than a hang; the states that upload the assistance database arm none, and
	// there this timeout is the only thing that can end the session.
	deschedule(!m_is_initiated);

	// Underwater short-circuit: re-arming here is wasted work and creates a
	// log-spam loop. Sequence: notify_underwater_state(true) calls deschedule()
	// then service_cancel() — and for services like GPSService, service_cancel()
	// invokes service_complete() with shall_reschedule=true, which lands back
	// here. run_scheduled_task() already gates service_initiate() on
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
	if (!is_started()) {
		DEBUG_TRACE("Service::reschedule: service %s is stopped", m_name);
		BENCH_SCHED_NOTE(SCHEDULE_DISABLED, "stopped");
		return;
	}
	if (!service_is_enabled()) {
		DEBUG_TRACE("Service::reschedule: service %s is not enabled", m_name);
		BENCH_SCHED_NOTE(SCHEDULE_DISABLED, "not-enabled");
		return;
	}

	// Computed BEFORE the m_is_initiated test, deliberately. This is virtual and
	// the overrides have side effects — ArgosTxService::service_next_schedule_in_ms
	// picks m_scheduled_task and m_scheduled_mode, arms the cooldown, refreshes
	// the prepass status. Moving it below the test would silently stop all of
	// that from happening on an already-initiated service.
	// `immediate` still bypasses the virtual entirely, and still reports
	// "scheduled": that bypass is load-bearing (GPSService documents that an
	// immediate reschedule deliberately skips its deep-idle gate, and Argos
	// relies on it not re-picking m_scheduled_task), and the string is matched
	// verbatim by tests/bench/dte_campaign.py.
	const ScheduleDecision decision = immediate ? ScheduleDecision::run(0, "scheduled") : service_next_schedule();

	// The legacy value this decision would have been before the type existed.
	// Only a Run carries a delay; every other kind read as SCHEDULE_DISABLED,
	// and the bench report must keep saying exactly that.
	[[maybe_unused]] const unsigned int legacy_ms =
	    (decision.kind() == ScheduleDecision::Kind::Run) ? decision.delay_ms() : SCHEDULE_DISABLED;

	if (m_is_initiated) {
		DEBUG_TRACE("Service::reschedule: service %s already initiated", m_name);
		BENCH_SCHED_NOTE(legacy_ms, "already-initiated");
		return;
	}

	if (decision.kind() == ScheduleDecision::Kind::Run) {
		m_hold_reason = nullptr;
		m_hold_streak = 0;
		DEBUG_TRACE("Service::reschedule: service %s scheduled in %u msecs", m_name, decision.delay_ms());
		BENCH_SCHED_NOTE(decision.delay_ms(), decision.reason());
		// m_last_schedule is written by Run and by nothing else. It means "when
		// is the next RUN", not "do I own a task" -- several tests use it as a
		// virtual clock (t += get_last_schedule()) and would silently jump time
		// by a hold's length if a hold wrote into it.
		m_last_schedule = decision.delay_ms();
		m_task_period = system_scheduler->post_task_prio([this]() { run_scheduled_task(); }, "ServicePeriod",
		                                                 Scheduler::DEFAULT_PRIORITY, decision.delay_ms());
		return;
	}

	if (decision.kind() == ScheduleDecision::Kind::Off) {
		m_hold_reason = nullptr;
		m_hold_streak = 0;
		DEBUG_TRACE("Service::reschedule: service %s off (%s)", m_name, decision.reason());
		BENCH_SCHED_NOTE(SCHEDULE_DISABLED, decision.reason());
		return;
	}

	// A hold. The service owns nothing to run, but the framework WILL come back
	// -- that is the whole difference with Off, and the reason a temporary
	// condition can no longer become permanent.
	post_hold_reevaluation(decision);
}

/// @brief Arm the re-evaluation that ends a hold, and keep repeats from polling.
///
/// Reuses m_task_period: this is only reachable when !m_is_initiated, and
/// deschedule() at the top of every reschedule() already cancels it, so an
/// event-driven reschedule -- the primary way out of a hold -- clears it for
/// free. No extra handle per service, and no second task to leak.
void Service::post_hold_reevaluation(const ScheduleDecision &decision) {
	unsigned int delay_s;
	if (decision.kind() == ScheduleDecision::Kind::HoldUntil) {
		const std::time_t now = (rtc && rtc->is_set()) ? rtc->gettime() : 0;
		const std::time_t until = (std::time_t)decision.epoch_s();
		// An instant already past, or an unusable clock, becomes the floor
		// rather than zero: a hold must never turn into a spin.
		delay_s = (now > 0 && until > now) ? (unsigned int)(until - now) : ScheduleDecision::MIN_HOLD_S;
		m_hold_reason = nullptr;  // a deadline is its own bound; no streak needed
		m_hold_streak = 0;
	} else {
		delay_s = decision.max_hold_s();
		// Same hold twice in a row: double the wait. Pointer identity on a
		// static literal, so this costs one compare.
		if (m_hold_reason == decision.reason()) {
			if (m_hold_streak < 16) m_hold_streak++;
			const unsigned int scaled = delay_s << (m_hold_streak > 20 ? 20 : m_hold_streak);
			delay_s = (scaled > HOLD_STREAK_CEILING_S || scaled < delay_s) ? HOLD_STREAK_CEILING_S : scaled;
			if (delay_s >= HOLD_STREAK_CEILING_S) {
				DEBUG_ERROR("Service::reschedule: service %s has answered \"%s\" %u times running — still holding, "
				            "now re-checking only every %u s",
				            m_name, decision.reason(), (unsigned int)m_hold_streak, delay_s);
			}
		} else {
			m_hold_reason = decision.reason();
			m_hold_streak = 0;
		}
	}

	DEBUG_INFO("Service::reschedule: service %s holding (%s) — re-evaluating in %u s", m_name, decision.reason(),
	           delay_s);
	BENCH_SCHED_HOLD(delay_s, decision.reason());
	// m_last_schedule deliberately left as deschedule() set it: a hold is not a
	// run, and service_is_scheduled() must keep meaning "a run is pending".
	m_task_period = system_scheduler->post_task_prio([this]() { reschedule(); }, "ServiceHoldReeval",
	                                                 Scheduler::DEFAULT_PRIORITY, (uint64_t)delay_s * 1000ULL);
}

/// @brief Body of the scheduled-period task: arm the timeout, then initiate.
void Service::run_scheduled_task() {
	// Barrier against unhandled exceptions in service code. service_initiate()
	// is virtual and runs user-defined logic that may throw (ErrorCode enums,
	// std::out_of_range from at(), bad_variant_access, …). Without this catch
	// the exception escapes the scheduler task runner and reaches
	// std::terminate → __verbose_terminate_handler → abort(), which on this
	// platform hangs in fputc until the watchdog fires 15 minutes later.
	try {
		unsigned int timeout_ms = service_next_timeout();
		DEBUG_TRACE("Service::reschedule: service %s time out in %u msecs", m_name, timeout_ms);
		if (timeout_ms) {
			m_task_timeout = system_scheduler->post_task_prio(
			    [this]() { on_service_timeout(); }, "ServiceTimeoutPeriod", Scheduler::DEFAULT_PRIORITY, timeout_ms);
		}

		if (!m_is_underwater) {
			DEBUG_TRACE("Service::reschedule: service %s active", m_name);
			m_is_initiated = true;
			if (service_is_active_on_initiate()) notify_service_active();
			service_initiate();
		} else {
			DEBUG_TRACE("Service::reschedule: service %s can't run underwater", m_name);
		}
	} catch (...) {
		// Cancel the safety-net timeout we just armed: otherwise the service is
		// stuck waiting for timeout_ms before any retry. (See QA review B2.)
		handle_task_exception("task", true);
	}
}

/// @brief Body of the timeout task: cancel the service and re-arm it.
void Service::on_service_timeout() {
	try {
		DEBUG_TRACE("Service::reschedule: service %s timed out", m_name);
		service_cancel();
		if (m_is_initiated) notify_service_inactive();
		m_is_initiated = false;
		reschedule();
	} catch (...) {
		// No cancel here: this IS m_task_timeout, and it has already fired.
		handle_task_exception("timeout", false);
	}
}

/// @brief Report the exception currently being handled, then recover.
void Service::handle_task_exception(const char *where, bool cancel_timeout) {
	try {
		throw;  // re-match the live exception; UB if called outside a catch
	} catch (ErrorCode e) {
		// Firmware-thrown enum (CONFIG_STORE_CORRUPTED, RESOURCE_NOT_AVAILABLE, …)
		DEBUG_ERROR("Service::reschedule: %s ErrorCode=%d in service %s — recovering", where, (int)e, m_name);
	} catch (const std::bad_variant_access &) {
		// Type mismatch when reading config_store params (variant<>).
		DEBUG_ERROR("Service::reschedule: %s bad_variant_access in service %s (config type mismatch?) — recovering",
		            where, m_name);
	} catch (const std::out_of_range &e) {
		// Index out of range — typically from std::array::at() or vector::at().
		DEBUG_ERROR("Service::reschedule: %s out_of_range in service %s (%s) — recovering", where, m_name, e.what());
	} catch (const std::exception &e) {
		// Any other std exception (bad_alloc, runtime_error, …)
		DEBUG_ERROR("Service::reschedule: %s std::exception in service %s: %s — recovering", where, m_name, e.what());
	} catch (...) {
		// Last-resort barrier. Without this the exception escapes the scheduler
		// runner and propagates up to terminate() → abort() → fputc-hang → WDT.
		DEBUG_ERROR("Service::reschedule: %s unknown exception in service %s — recovering", where, m_name);
	}

	if (cancel_timeout) system_scheduler->cancel_task(m_task_timeout);
	m_is_initiated = false;

	// Recover a schedule. At this point the service owns nothing: the period
	// task has already fired -- it is what threw -- and the timeout has either
	// just been cancelled or already run. Before this, a single transient throw
	// out of service_initiate() left the service inert until an unrelated peer
	// event happened along, and on a configuration that has no such event (a
	// periodic tracker with no underwater sensor) until a watchdog reset the
	// device. GPSService reaches this from ordinary config reads and from the
	// battery gauge on a wedged I2C bus, so it is not a theoretical path.
	//
	// Deferred rather than immediate on purpose: service_next_schedule_in_ms()
	// is virtual and is itself a plausible source of the throw, so calling
	// reschedule() straight from this handler could throw right back out of it.
	// The inner catch is the same argument one level down.
	if (m_is_started) {
		m_task_exception_retry = system_scheduler->post_task_prio(
		    [this]() {
			    try {
				    reschedule();
			    } catch (...) {
				    DEBUG_ERROR("Service::handle_task_exception: recovery reschedule threw for service %s", m_name);
			    }
		    },
		    "ServiceExceptionRetry", Scheduler::DEFAULT_PRIORITY, EXCEPTION_RETRY_MS);
	}
}

/// @brief Cancel all pending tasks (period + timeout).
void Service::deschedule(bool cancel_timeout) {
	if (cancel_timeout) system_scheduler->cancel_task(m_task_timeout);
	system_scheduler->cancel_task(m_task_period);
	// A real schedule supersedes any pending exception recovery.
	system_scheduler->cancel_task(m_task_exception_retry);
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
