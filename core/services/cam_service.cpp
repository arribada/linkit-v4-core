/**
 * @file cam_service.cpp
 * @brief Camera service — periodic on/off cycling, event handling, capture logging.
 */

#include <cstring>

#include "cam_service.hpp"
#include "config_store.hpp"
#include "scheduler.hpp"
#if ENABLE_AXL_SENSOR
#include "axl_sensor_service.hpp"
#endif

extern ConfigurationStore *configuration_store;
extern Scheduler *system_scheduler;

static constexpr unsigned int MS_PER_SEC = 1000;

/// @brief Init: reset state (members already initialized in-class).
void CAMService::service_init() {
	m_is_active = false;
	m_is_pwr_on = false;
}

/// @brief Terminate: drop a pending exception recovery. The device itself was
/// powered off by service_cancel(), which Service::stop() calls first.
void CAMService::service_term() {
	system_scheduler->cancel_task(m_task_recovery);
}

/// @brief Enabled if CAM_ENABLE param is set, with low-battery override via LB_CAM_EN.
/// @details When `LB_EN` is on and the battery is below threshold, the camera follows
/// `LB_CAM_EN` (default false) instead of `CAM_ENABLE`. Default low-battery behavior is
/// therefore "cam OFF in LB" — preserves the budget for satellite/LoRa TX. Set
/// `LB_CAM_EN=true` to keep the camera running through LB.
/// @return true if camera is enabled.
bool CAMService::service_is_enabled() {
	if (service_read_param<bool>(ParamID::LB_EN) && service_is_battery_level_low()) {
		return service_read_param<bool>(ParamID::LB_CAM_EN);
	}
	return service_read_param<bool>(ParamID::CAM_ENABLE);
}

/// @brief Compute next schedule — alternates between period_on and period_off.
/// @return Delay in ms until next power toggle, or SCHEDULE_DISABLED if period_on is 0.
unsigned int CAMService::service_next_schedule_in_ms() {
	const std::time_t now = service_current_time();
	const std::time_t period_on = service_read_param<unsigned int>(ParamID::CAM_PERIOD_ON);
	const std::time_t period_off = service_read_param<unsigned int>(ParamID::CAM_PERIOD_OFF);
	if (period_on == 0) {
		return Service::SCHEDULE_DISABLED;
	}

	const std::time_t next_schedule = m_is_pwr_on ? period_on : period_off;

	// The UTC instant of the toggle being armed — what a log entry's schedTime means.
	m_next_schedule = now + next_schedule;

	DEBUG_TRACE("CAMService::reschedule: period_on=%u period_off=%u now=%u next=%u next_state=%u",
	            (unsigned int)period_on, (unsigned int)period_off, (unsigned int)now, (unsigned int)next_schedule,
	            (unsigned int)!m_is_pwr_on);

	// The power sequence is charged to the period: a session only completes
	// PWR_DELAY + PWR_BUTT_DELAY ms after the toggle is requested.
	return (unsigned int)(next_schedule * MS_PER_SEC) + PWR_BUTT_DELAY + PWR_DELAY;
}

/// @brief Toggle camera power — if on, turn off; if off, turn on.
///
/// Starts the device's asynchronous sequence and returns; the session ends in
/// react() when the device reports the new state.
void CAMService::service_initiate() {
	m_is_active = true;
	if (m_device.is_powered_on()) {
		DEBUG_TRACE("CAMService::service_initiate => PWR OFF");
		m_device.power_off();
	} else {
		DEBUG_TRACE("CAMService::service_initiate => PWR ON");
		m_device.power_on();
	}
}

/// @brief Cancel active camera — power off (blocking), log, re-arm.
///
/// Reached from Service::stop() when the FSM leaves Operational, from the
/// safety-net timeout if a sequence never completed, and from a device error.
/// All mean "make sure the camera is really off before going further", so this
/// is the one place that blocks for the button sequence. m_is_active is
/// cleared FIRST so the CAMEventPowerOff raised by the device is not turned
/// into a second completion by react().
///
/// Mid-session the OFF entry closes it and service_complete() re-arms the
/// schedule. Between sessions (device error while filming) the pending toggle
/// was armed against a camera that is now off, so it is re-armed from the OFF
/// period instead. Both re-arms are no-ops once Service::stop() has run.
/// @return true if camera was active and cancelled.
bool CAMService::service_cancel() {
	DEBUG_TRACE("CAMService::service_cancel");
	if (!m_is_active) return false;

	m_is_active = false;
	m_device.power_off_blocking();
	m_is_pwr_on = false;
	if (service_is_initiated()) {
		CAMLogEntry log_entry = invalid_log_entry();
		ServiceEventData event_data = log_entry;
		service_complete(&event_data, &log_entry);
	} else {
		service_reschedule();
	}
	return true;
}

/// @brief Safety net for a sequence whose completion never arrives.
///
/// A power sequence is two scheduler tasks; if one is ever dropped (queue
/// saturation) the session would otherwise stay initiated for good with the
/// camera possibly left ON. The framework then runs service_cancel(). Far
/// longer than the 2.1 s sequence on purpose: a scheduler busy with a blocking
/// modem or flash operation must not trip it.
/// @return Timeout in ms.
unsigned int CAMService::service_next_timeout() {
	return SEQUENCE_TIMEOUT_MS;
}

/// @brief Camera is usable underwater (waterproof housing).
///
/// Consequence: Service::notify_underwater_state() returns before doing
/// anything for this service, so a dive never cancels a running session and a
/// resurfacing never reaches service_is_triggered_on_surfaced(). The surfacing
/// trigger is handled in notify_peer_event() for that reason.
/// @return Always true.
bool CAMService::service_is_usable_underwater() {
	return true;
}

/// @brief Peer events: surfacing edge for CAM_TRIGGER_ON_SURFACED, then the
/// framework's usual routing (AXL wake-up via service_is_triggered_on_event).
void CAMService::notify_peer_event(ServiceEvent &event) {
	if (event.event_source == ServiceIdentifier::UW_SENSOR
	    && event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
		const bool *submerged = std::get_if<bool>(&event.event_data);
		if (submerged != nullptr && !*submerged && !m_device.is_powered_on()
		    && service_read_param<bool>(ParamID::CAM_TRIGGER_ON_SURFACED)) {
			DEBUG_TRACE("CAMService: surfaced — immediate capture");
			service_reschedule(true);
		}
	}
	Service::notify_peer_event(event);
}

/// @brief Build a camera log entry with OFF status (error or cancel).
/// @return CAMLogEntry with event_type=OFF and current battery/time.
CAMLogEntry CAMService::invalid_log_entry() {
	DEBUG_INFO("CAMService::invalid_log_entry");

	CAMLogEntry cam_entry;
	memset(&cam_entry, 0, sizeof(cam_entry));

	cam_entry.header.log_type = LOG_CAM;

	populate_cam_log_with_time(cam_entry, service_current_time());

	service_update_battery();
	cam_entry.info.batt_voltage = service_get_voltage();
	cam_entry.info.event_type = CAMEventType::OFF;
	cam_entry.info.counter = (uint16_t)m_device.get_num_captures();
	cam_entry.info.schedTime = m_next_schedule;

	return cam_entry;
}

/// @brief Log camera state change (ON/OFF) with battery and capture count.
/// @param state  true = ON, false = OFF.
void CAMService::task_process_cam_data(bool state) {
	DEBUG_TRACE("CAMService::task_process_cam_data");

	CAMLogEntry cam_entry;
	memset(&cam_entry, 0, sizeof(cam_entry));

	cam_entry.header.log_type = LOG_CAM;

	populate_cam_log_with_time(cam_entry, service_current_time());

	service_update_battery();
	cam_entry.info.batt_voltage = service_get_voltage();

	cam_entry.info.schedTime = m_next_schedule;

	if (state)
		cam_entry.info.event_type = CAMEventType::ON;
	else
		cam_entry.info.event_type = CAMEventType::OFF;

	// The log field is 16-bit: the count wraps after 65535 power-ons, which is
	// the flash record format and not worth widening for.
	cam_entry.info.counter = (uint16_t)m_device.get_num_captures();

	DEBUG_INFO("CAMService::task_process_cam_data: batt=%lfV state=%u count=%u",
	           (double)cam_entry.info.batt_voltage / 1000, (unsigned int)cam_entry.info.event_type,
	           (unsigned int)cam_entry.info.counter);

	ServiceEventData event_data = cam_entry;
	service_complete(&event_data, &cam_entry, true);
}

/// @brief Complete the session from an asynchronous device event, behind a barrier.
///
/// task_process_cam_data() ends in service_complete() -> reschedule() ->
/// service_next_schedule_in_ms(), which reads the config store and can throw.
/// We run here from the device's scheduler task, outside the try/catch of
/// Service::run_scheduled_task(), so nothing else would catch it. Recover the
/// way the framework's own barrier does: close the session, then re-arm a
/// schedule once, a little later. If that retry throws too, the service stays
/// idle until the next peer event rather than storming the log.
void CAMService::complete_with_log(bool state) {
	try {
		task_process_cam_data(state);
	} catch (...) {
		DEBUG_ERROR("CAMService: exception completing %s — recovering", state ? "power-on" : "power-off");
		if (service_is_initiated()) {
			// Not reached service_complete(): close the session ourselves, or
			// every later reschedule() bails with "already initiated".
			try {
				service_complete(nullptr, nullptr, false);
			} catch (...) {
				// Nothing left to close; the retry below still runs.
			}
		}
		system_scheduler->cancel_task(m_task_recovery);
		m_task_recovery = system_scheduler->post_task_prio(
		    [this]() {
			    try {
				    service_reschedule();
			    } catch (...) {
				    DEBUG_ERROR("CAMService: recovery reschedule failed — idle until next peer event");
			    }
		    },
		    "CAMServiceRecovery", Scheduler::DEFAULT_PRIORITY, RECOVERY_RETRY_MS);
	}
}

/// @brief Camera error — treated as a cancel: blocking power-off, OFF log entry.
void CAMService::react(const CAMEventError &) {
	if (!m_is_active) return;
	DEBUG_TRACE("CAMService::react(CAMEventError)");
	service_cancel();
}

/// @brief Camera power-on sequence completed — update state, log, reschedule.
void CAMService::react(const CAMEventPowerOn &) {
	if (!m_is_active) return;
	DEBUG_TRACE("CAMService::react(CAMEventOn)");
	m_is_pwr_on = true;
	complete_with_log(true);
}

/// @brief Camera power-off sequence completed — update state, log, reschedule.
void CAMService::react(const CAMEventPowerOff &) {
	if (!m_is_active) return;
	DEBUG_TRACE("CAMService::react(CAMEventOff)");
	m_is_pwr_on = false;
	complete_with_log(false);
}

/// @brief Fill log entry header with date/time fields.
/// @param[out] entry  Log entry to populate.
/// @param time        Epoch time (seconds).
void CAMService::populate_cam_log_with_time(CAMLogEntry &entry, std::time_t time) {
	service_set_log_header_time(entry.header, time);
}

/// @brief Check if AXL wakeup event should start a capture.
///
/// Only STARTS one. Without the is_powered_on() test the trigger toggled, so a
/// wake-up arriving while the camera was filming cut the recording short — and
/// on a moving animal wake-ups are frequent.
/// @param event       Incoming peer event.
/// @param[out] immediate  Set to true if camera should fire immediately.
/// @return true if this event triggers camera rescheduling.
bool CAMService::service_is_triggered_on_event(ServiceEvent &event, bool &immediate) {
#if ENABLE_AXL_SENSOR
	if (event.event_source == ServiceIdentifier::AXL_SENSOR
	    && event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
		// Check if AXL wakeup was triggered by reading the ServiceSensorData
		auto *sensor_data = std::get_if<ServiceSensorData>(&event.event_data);
		if (sensor_data && sensor_data->port[AXLSensorPort::WAKEUP_TRIGGERED]) {
			immediate = !m_device.is_powered_on() && service_read_param<bool>(ParamID::CAM_TRIGGER_ON_AXL_WAKEUP);
			return immediate;
		}
	}
#else
	(void)event;
	(void)immediate;
#endif

	return false;
}
