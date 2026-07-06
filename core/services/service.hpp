/**
 * @file service.hpp
 * @brief Service framework — base class for all periodic services + ServiceManager orchestrator.
 */

#pragma once

#include <functional>
#include <variant>
#include <ctime>

#include "service_scheduler.hpp"
#include "scheduler.hpp"
#include "logger.hpp"
#include "base_types.hpp"
#include "config_store.hpp"

extern ConfigurationStore *configuration_store;

/// @brief Base class for all periodic services (GPS, Argos TX/RX, sensors, SWS, camera, etc.).
class Service {
public:
	static constexpr unsigned int SCHEDULE_DISABLED = 0xFFFFFFFF;  ///< Returned by service_next_schedule_in_ms to disable scheduling.

	Service(ServiceIdentifier service_id, const char *name, Logger *logger = nullptr);
	virtual ~Service();
	unsigned int get_unique_id();
	const char *get_name();
	ServiceIdentifier get_service_id();
	Logger *get_logger();
	void set_logger(Logger *logger);
	void start(std::function<void(ServiceEvent&)> data_notification_callback = nullptr);
	void stop();
	virtual void notify_peer_event(ServiceEvent& event);
	bool is_started();
	unsigned int get_last_schedule();
	bool is_underwater_deferred();
	bool is_initiated();

private:
	bool m_is_started = false;
	const char *m_name = nullptr;
	bool m_is_underwater = false;
	bool m_is_initiated = false;
	Scheduler::TaskHandle m_task_period;
	Scheduler::TaskHandle m_task_timeout;
	std::function<void(ServiceEvent&)> m_data_notification_callback;
	ServiceIdentifier m_service_id = ServiceIdentifier::UNKNOWN;
	unsigned int m_unique_id = 0;
	Logger *m_logger = nullptr;
	unsigned int m_last_schedule = SCHEDULE_DISABLED;

	void reschedule(bool immediate = false);
	void deschedule();
	void notify_log_updated(ServiceEventData& data);
	void notify_service_active();
	void notify_service_inactive();
	void notify_underwater_state(bool state);

protected:
	/// @brief GNSS MED #4 audit fix: derived services that short-circuit
	/// service_initiate (e.g. rate-limit early return) need to cancel the
	/// safety-net timeout posted by `reschedule()`. Otherwise the timeout
	/// fires ~120 s later, runs `service_cancel` + another reschedule, and
	/// leaks scheduler slots over long deployments. Protected so only
	/// derived service classes can use it. Implementation in service.cpp
	/// since `system_scheduler` is a global not visible from this header.
	void cancel_safety_timeout();

	/// @brief Emit a custom ServiceEvent on the peer-notification bus.
	/// Used for non-standard event types that don't fit ACTIVE/INACTIVE/
	/// LOG_UPDATED semantics (e.g. SERVICE_CLOUDLOCATE_READY = "raw
	/// measurement available mid-acquisition, GPS still running").
	void notify_service_event(ServiceEventType type);

private:

protected:
	// === Virtual interface — subclasses must/may override ===

	/// @brief Initialize service hardware/state (called once at start).
	virtual void service_init() = 0;
	/// @brief Terminate service (called once at stop).
	virtual void service_term() = 0;
	/// @brief Check if this service is enabled in config.
	/// @return true if service should be scheduled.
	virtual bool service_is_enabled() = 0;
	/// @brief Compute delay until next execution.
	/// @return Delay in ms, or SCHEDULE_DISABLED.
	virtual unsigned int service_next_schedule_in_ms() = 0;
	/// @brief Execute the service task (called when scheduled time arrives).
	virtual void service_initiate() = 0;
	/// @brief Cancel an active service task (e.g., GPS acquisition timeout).
	/// @return true if the task was active and cancelled.
	virtual bool service_cancel() { return false; }
	/// @brief Safety timeout after which service_cancel is called automatically.
	/// @return Timeout in ms, or 0 for no timeout.
	virtual unsigned int service_next_timeout() { return 0; }
	/// @brief Check if this service should reschedule on surfacing.
	/// @param[out] immediate  true if service should fire immediately.
	/// @return true if surfacing triggers this service.
	virtual bool service_is_triggered_on_surfaced(bool&) { return false; }
	/// @brief Check if this service can operate underwater (default: no).
	/// @return true if service should keep running when submerged.
	virtual bool service_is_usable_underwater() { return false; }
	/// @brief Check if a peer event should trigger this service.
	/// @param[in] event      Peer event.
	/// @param[out] immediate true if service should fire immediately.
	/// @return true if this event triggers rescheduling.
	virtual bool service_is_triggered_on_event(ServiceEvent&, bool&) { return false; }
	/// @brief Check if service is active after initiate (for async services).
	/// @return true if initiate starts an async operation (default: true).
	virtual bool service_is_active_on_initiate() { return true; }

	// === Protected helpers for subclasses ===

	/// @brief Check if this service has a pending schedule.
	bool service_is_scheduled();
	/// @brief Force reschedule (e.g., after config change or peer event).
	/// @param immediate  true to schedule with 0 delay.
	void service_reschedule(bool immediate = false);
	/// @brief Write log entry and notify peers of updated data.
	/// @param event_data  Optional event payload for peer notification.
	/// @param entry       Optional raw log entry for persistent logger.
	void service_log(ServiceEventData *event_data = nullptr, void *entry = nullptr);
	/// @brief Mark service as complete — log, notify inactive, optionally reschedule.
	/// @param event_data  Optional event payload.
	/// @param entry       Optional raw log entry.
	/// @param reschedule  true to reschedule after completion (default: true).
	void service_complete(ServiceEventData *event_data = nullptr, void *entry = nullptr, bool reschedule = true);
#ifdef BENCH_TEST
	/// @brief Bench-only: force the service into the "initiated/started" state so a
	/// synthetic completion (e.g. GPSService::bench_inject_fix) runs the FULL
	/// service_complete() path — including service_log() → SERVICE_LOG_UPDATED
	/// broadcast — instead of bailing on "!m_is_initiated". Zero prod footprint.
	void bench_force_initiated() { m_is_started = true; m_is_initiated = true; }
#endif
	/// @brief Notify peers that this service is now active.
	void service_active();
	/// @brief Fill log header date/time fields from epoch time.
	/// @param[out] header  Log header to fill.
	/// @param time         Epoch time (seconds).
	void service_set_log_header_time(LogHeader& header, std::time_t time);
	/// @brief Get current RTC time.
	/// @return Epoch time in seconds.
	std::time_t service_current_time();
	/// @brief Check if RTC has been set (e.g., by GNSS fix).
	bool service_is_time_known();
	/// @brief Set RTC time (e.g., from GNSS fix).
	void service_set_time(std::time_t);
	/// @brief Sample battery ADC/gauge.
	void service_update_battery();
	/// @brief Get last battery voltage (mV).
	uint16_t service_get_voltage();
	/// @brief Get last battery level (0-100%).
	uint8_t service_get_level();
	/// @brief Check if battery is below low threshold.
	bool service_is_battery_level_low();
	/// @brief Get current hardware timer counter (ms).
	uint64_t service_current_timer();
	template <typename T> T& service_read_param(ParamID param_id) {
		return configuration_store->read_param<T>(param_id);
	}
	template <typename T> void service_write_param(ParamID param_id, T& value) {
		configuration_store->write_param(param_id, value);
	}

public:
	// Cooldown sleep: pause/resume service externally (used by ServiceManager)
	void pause_for_cooldown();
	void resume_from_cooldown();
	virtual void reset_state_for_cooldown_exit() {}  ///< Override to force state re-emission after cooldown
};

/// @brief Global service orchestrator — manages all Service instances, peer events, cooldown.
class ServiceManager
{
private:
	static inline std::function<void(ServiceEvent&)> m_data_notification_callback = nullptr;
	static inline unsigned int m_unique_identifier = 0;
	static inline std::map<unsigned int, Service&> m_map;
	static inline std::time_t m_last_successful_cycle_time = 0;
	static inline unsigned int m_passive_surfacing_count = 0;

public:
	static unsigned int add(Service& s);
	static void remove(Service& s);
	/// @brief Clear ALL static state (registered services, id counter, cooldown
	///        and cycle bookkeeping). TEST-ONLY: production never resets the
	///        ServiceManager. Lets a unit-test fixture guarantee a clean global
	///        between tests so a leftover registration or deferred continuation
	///        from a prior test cannot bleed into the next one.
	static void reset();
	static void startall(std::function<void(ServiceEvent&)> data_notification_callback = nullptr);
	static void stopall();
	static void notify_underwater_state(bool state);
	static void notify_peer_event(ServiceEvent& event);
	static void inject_event(ServiceEvent& event);
	static Logger *get_logger(ServiceIdentifier service_id);
	static unsigned int get_unique_id(const char *name);
	static void set_cycle_complete(std::time_t t);
	static bool is_in_cooldown(std::time_t now);
	/// @brief Seconds remaining in the current cooldown window, or 0 if no
	/// active cooldown (disabled, or already expired).
	static unsigned int get_cooldown_remaining_s(std::time_t now);
	static void notify_passive_surfacing();
	static unsigned int get_passive_surfacing_count();
	static void save_cooldown_state();
	static void restore_cooldown_state();
	static void enter_cooldown_sleep();
	static void exit_cooldown_sleep();

private:
	static inline Scheduler::TaskHandle m_cooldown_wake_task;
};
