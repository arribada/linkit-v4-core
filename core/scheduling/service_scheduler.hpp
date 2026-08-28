/**
 * @file service_scheduler.hpp
 * @brief Service event types, identifiers, and inter-service communication structures.
 */

#pragma once

#include <functional>
#include <variant>

#include "messages.hpp"

/// @brief Event types emitted by services and consumed by peers / ServiceManager.
enum class ServiceEventType {
	SERVICE_ACTIVE,          ///< Service has started (e.g., GPS power on)
	SERVICE_INACTIVE,        ///< Service has completed (e.g., GPS power off)
	SERVICE_LOG_UPDATED,     ///< Service has written a new log entry
	GNSS_CLOUDLOCATE_READY,  ///< GPS captured 1st raw measurement during active acquisition
	                         ///< (GPS keeps running — this is just a heads-up so peers like
	                         ///< LoRa can fire an early CloudLocate TX without waiting for
	                         ///< full PVT timeout)
	GNSS_OFF_DEEP_IDLE,      ///< GPS session ended, dispatched to deep-idle (rail on,
	                         ///< M10Q in PMREQ-backup). LED dispatcher renders as a brief
	                         ///< double-blink RED. Emitted only on no-fix paths.
	GNSS_OFF_POWEROFF,       ///< GPS session ended, full power-off (rail cut).
	                         ///< LED dispatcher renders as a fast blink RED. Emitted
	                         ///< only on no-fix paths.
	GNSS_ON,                 ///< GPS acquisition started (legacy)
	ARGOS_TX_START,          ///< Argos TX started (legacy)
	ARGOS_TX_END,            ///< Argos TX completed (legacy)
	SENSOR_LOG_UPDATED       ///< Sensor data logged (legacy)
};

/// @brief Sensor data payload for SERVICE_LOG_UPDATED events.
struct ServiceSensorData {
	double port[6] = {};  ///< Up to 6 sensor ports (0-initialized to avoid garbage reads)
};

using ServiceEventData = std::variant<bool, GPSLogEntry, ServiceSensorData, CAMLogEntry>;

/// @brief Unique identifier for each service instance.
enum class ServiceIdentifier : unsigned int {
	UNKNOWN,
	ARGOS_TX,
	ARGOS_RX,
	LORA_TX,
	GNSS_SENSOR,
	CDT_SENSOR,
	PRESSURE_SENSOR,
	UW_SENSOR,
	ALS_SENSOR,
	AIR_TEMP_SENSOR,
	SEA_TEMP_SENSOR,
	PH_SENSOR,
	AXL_SENSOR,
	MEMORY_MONITOR,
	DIVE_MODE,
	CAM_SENSOR,
	THERMISTOR_SENSOR,
	MORTALITY
};

/// @brief Inter-service event — carries type, data, source, and originator ID.
struct ServiceEvent {
	ServiceEventType event_type = ServiceEventType::SERVICE_INACTIVE;
	ServiceEventData event_data;
	ServiceIdentifier event_source = ServiceIdentifier::UNKNOWN;
	unsigned int event_originator_unique_id = 0;
};

/// @deprecated Legacy scheduler interface — kept for test compatibility. Use Service subclass instead.
class ServiceScheduler {
public:
	virtual ~ServiceScheduler() {}
	virtual void start(std::function<void(ServiceEvent &)> data_notification_callback = nullptr) = 0;
	virtual void stop() = 0;
	virtual void notify_underwater_state(bool state) = 0;
	virtual void notify_sensor_log_update() = 0;
};
