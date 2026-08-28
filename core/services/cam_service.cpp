/**
 * @file cam_service.cpp
 * @brief Camera service — periodic on/off cycling, event handling, capture logging.
 */

#include "cam_service.hpp"
#include "config_store.hpp"
#include "scheduler.hpp"
#include "runcam.hpp"
#if ENABLE_AXL_SENSOR
#include "axl_sensor_service.hpp"
#endif

extern ConfigurationStore *configuration_store;

static constexpr unsigned int MS_PER_SEC = 1000;

/// @brief Init: reset state (members already initialized in-class).
void CAMService::service_init() {
	m_is_active = false;
	m_num_captures = 0;
	m_is_pwr_on = false;
}

/// @brief Terminate: no-op (device powered off by service_cancel).
void CAMService::service_term() {}

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
	std::time_t now = service_current_time();
	std::time_t period_on = service_read_param<unsigned int>(ParamID::CAM_PERIOD_ON);
	std::time_t period_off = service_read_param<unsigned int>(ParamID::CAM_PERIOD_OFF);
	std::time_t next_schedule = 0;
	if (period_on == 0) {
		return Service::SCHEDULE_DISABLED;
	}

	if (m_is_pwr_on)
		next_schedule = period_on;
	else
		next_schedule = period_off;

	DEBUG_TRACE("CAMService::reschedule: period_on=%u period_off=%u now=%u next=%u next_state=%u",
	            (unsigned int)period_on, (unsigned int)period_off, (unsigned int)now, (unsigned int)next_schedule,
	            !m_is_pwr_on);
	(void)now;  // Suppress unused variable warning when DEBUG_TRACE is disabled

	// Find the time in milliseconds until this schedule
	//return (next_schedule - now) * MS_PER_SEC;
	return (next_schedule * MS_PER_SEC) + PWR_BUTT_DELAY + PWR_DELAY;
}

/// @brief Toggle camera power — if on, turn off; if off, turn on.
void CAMService::service_initiate() {
	m_is_active = true;
	m_next_schedule = service_current_timer();
	m_wakeup_time = service_current_timer();
	if (m_device.is_powered_on()) {
		DEBUG_TRACE("CAMService::service_initiate => PWR OFF");
		m_device.power_off();
	} else {
		DEBUG_TRACE("CAMService::service_initiate => PWR ON");
		m_device.power_on();
	}
}

/// @brief Cancel active camera — power off and log.
/// @return true if camera was active and cancelled.
bool CAMService::service_cancel() {
	// Cleanly terminate
	DEBUG_TRACE("CAMService::service_cancel");

	if (m_is_active) {
		m_is_active = false;
		m_device.power_off();
		CAMLogEntry log_entry = invalid_log_entry();
		ServiceEventData event_data = log_entry;
		service_complete(&event_data, &log_entry);
		return true;
	}

	return false;
}

/// @brief No timeout managed for camera — returns 0 (ServiceManager handles).
/// @return Always 0.
unsigned int CAMService::service_next_timeout() {
	return 0;
}

/// @brief Trigger camera on surfacing if CAM_TRIGGER_ON_SURFACED is set.
/// @param[out] immediate  true if camera should fire immediately on surfacing.
/// @return Always true (reschedule on surface).
bool CAMService::service_is_triggered_on_surfaced(bool &immediate) {
	immediate = service_read_param<bool>(ParamID::CAM_TRIGGER_ON_SURFACED);
	return true;
}

/// @brief Camera is usable underwater (waterproof housing).
/// @return Always true.
bool CAMService::service_is_usable_underwater() {
	return true;
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

	cam_entry.info.counter = m_device.get_num_captures();

	DEBUG_INFO("CAMService::task_process_cam_data: batt=%lfV state=%u count=%u",
	           (double)cam_entry.info.batt_voltage / 1000, (unsigned int)cam_entry.info.event_type,
	           (unsigned int)cam_entry.info.counter);

	ServiceEventData event_data = cam_entry;
	service_complete(&event_data, &cam_entry, true);
}

/// @brief Camera error — power off and complete service.
void CAMService::react(const CAMEventError &) {
	if (!m_is_active) return;
	DEBUG_TRACE("CAMService::react(CAMEventError)");
	m_device.power_off();
	CAMLogEntry log_entry = invalid_log_entry();
	ServiceEventData event_data = log_entry;
	service_complete(&event_data, &log_entry);
}

/// @brief Camera power on confirmed — update state, log.
/// @note m_device.power_on() is safe to re-call (RunCam has POWERED_ON guard).
void CAMService::react(const CAMEventPowerOn &) {
	DEBUG_TRACE("CAMService::react(CAMEventOn)");
	m_is_pwr_on = true;
	m_device.power_on();
	task_process_cam_data(m_is_pwr_on);
}

/// @brief Camera power off confirmed — update state, log, increment capture count.
/// @note m_device.power_off() is safe to re-call (RunCam has POWERED_OFF guard).
void CAMService::react(const CAMEventPowerOff &) {
	DEBUG_TRACE("CAMService::react(CAMEventOff)");
	m_is_pwr_on = false;
	m_device.power_off();
	task_process_cam_data(m_is_pwr_on);
	m_num_captures++;
}

/// @brief Fill log entry header with date/time fields.
/// @param[out] entry  Log entry to populate.
/// @param time        Epoch time (seconds).
void CAMService::populate_cam_log_with_time(CAMLogEntry &entry, std::time_t time) {
	service_set_log_header_time(entry.header, time);
}

/// @brief Check if AXL wakeup event should trigger camera.
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
			bool trigger_on_axl = service_read_param<bool>(ParamID::CAM_TRIGGER_ON_AXL_WAKEUP);
			immediate = trigger_on_axl;
			return trigger_on_axl;
		}
	}
#else
	(void)event;
	(void)immediate;
#endif

	return false;
}
