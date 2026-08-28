/**
 * @file sensor_service.hpp
 * @brief Abstract sensor service — periodic sampling, TX aggregation (mean/median/oneshot).
 *
 * Base class for all sensor services (ALS, PH, CDT, pressure, sea_temp, thermistor, AXL).
 * Handles background sampling triggered by GNSS active/complete events, progressive
 * aggregation, and log persistence.
 */

#pragma once

#include <algorithm>
#include <cstdint>
#include <numeric>
#include <vector>

#include "sensor.hpp"
#include "service.hpp"
#include "logger.hpp"

/// @brief Base class for sensor services — periodic sampling with TX aggregation modes.
class SensorService : public Service {
public:
	/// @param sensor   Hardware sensor instance.
	/// @param service  Service identifier for this sensor type.
	/// @param name     Service name (for debug logging).
	/// @param logger   Optional persistent logger.
	SensorService(Sensor &sensor, ServiceIdentifier service, const char *name, Logger *logger)
	    : Service(service, name, logger),
	      m_sensor(sensor) {}
	virtual ~SensorService() {}

protected:
	Sensor &m_sensor;

	/// @brief Read sensor, aggregate samples, log entry. Called by service_initiate and wakeup events.
	/// @param reschedule      true to reschedule after completion.
	/// @param gnss_shutdown   true if GNSS just completed — force terminal state.
	void sensor_handler(bool reschedule = true, bool gnss_shutdown = false) {
		try {
			if (m_sensor_background_active) {
				m_sample_number++;
				for (unsigned int chan = 0; chan < safe_num_channels(); chan++) {
					if (m_samples[chan].size() < sensor_max_samples() + 1)
						m_samples[chan].push_back(m_sensor.read(chan));
				}

				// Update progressive ready value after each sample (Option 3: best-effort aggregation)
				// This ensures we always have a valid value to report even if GNSS completes early
				update_ready_value();

				if (service_is_scheduled()) {
					service_complete(nullptr, nullptr,
					                 !gnss_shutdown
					                     && (sensor_enable_tx_mode() != BaseSensorEnableTxMode::ONESHOT
					                         && m_sample_number < sensor_max_samples()));
				}

				if (gnss_shutdown || m_sample_number >= sensor_max_samples()) {
					DEBUG_TRACE("SensorService: %s: terminal state reached (%u/%u samples)", get_name(),
					            m_sample_number, sensor_max_samples());
					// Use the progressive ready value (already up-to-date)
					LogEntry e;
					ServiceEventData data = m_ready_value;
					sensor_populate_log_entry(&e, m_ready_value);
					service_log(&data, &e);
					m_sensor_background_active = false;
				}
			} else {
				// Not in TX background mode — periodic log acquisition
				ServiceSensorData sensor;
				for (unsigned int chan = 0; chan < safe_num_channels(); chan++)
					sensor.port[chan] = m_sensor.read(chan);
				LogEntry e;
				ServiceEventData data = sensor;
				sensor_populate_log_entry(&e, sensor);
				service_log(&data, &e);
				service_complete(nullptr, nullptr, reschedule);
			}
		} catch (ErrorCode e) {
			DEBUG_ERROR("SensorService: %s: Failed to read sensor [%04X]", get_name(), (unsigned int)e);
			if (!m_sensor_background_active) {
				service_complete(nullptr, nullptr, reschedule);
			} else {
				m_sensor_background_active = false;
				service_complete(nullptr, nullptr, reschedule);
			}
		}
	}

private:
	static constexpr unsigned int MAX_SENSOR_CHANNELS = 6;
	std::vector<double> m_samples[MAX_SENSOR_CHANNELS];
	unsigned int m_sample_number = 0;
	bool m_sensor_background_active = false;
	ServiceSensorData m_ready_value = {};
	bool m_ready_valid = false;

	double compute_mean_samples(std::vector<double> &v) {
		if (v.empty()) return 0.0;
		return std::reduce(v.begin(), v.end()) / v.size();
	}
	double compute_median_samples(std::vector<double> &v) {
		if (v.empty()) return 0.0;
		size_t n = v.size();
		auto mid = v.begin() + n / 2;
		std::nth_element(v.begin(), mid, v.end());
		if (n % 2 == 0) {
			auto lower = std::max_element(v.begin(), mid);
			return (*mid + *lower) / 2.0;
		}
		return *mid;
	}
	double compute_oneshot_samples(std::vector<double> &v) {
		if (v.empty()) return 0.0;
		return v.at(0);
	}

	// Progressive aggregation: update ready value after each sample
	// so we always have a best-effort value available for TX
	void update_ready_value() {
		for (unsigned int chan = 0; chan < safe_num_channels(); chan++) {
			switch (sensor_enable_tx_mode()) {
			case BaseSensorEnableTxMode::ONESHOT:
				m_ready_value.port[chan] = compute_oneshot_samples(m_samples[chan]);
				break;
			case BaseSensorEnableTxMode::MEAN: m_ready_value.port[chan] = compute_mean_samples(m_samples[chan]); break;
			case BaseSensorEnableTxMode::MEDIAN:
				m_ready_value.port[chan] = compute_median_samples(m_samples[chan]);
				break;
			default:
			case BaseSensorEnableTxMode::OFF: break;
			}
		}
		m_ready_valid = true;
	}

	void notify_peer_event(ServiceEvent &e) override {
		if (sensor_enable_tx_mode() != BaseSensorEnableTxMode::OFF) {
			handle_peer_event(e);
		}
		Service::notify_peer_event(e);
	}

	void handle_peer_event(ServiceEvent &e) {
		if (e.event_source == ServiceIdentifier::GNSS_SENSOR) {
			if (e.event_type == ServiceEventType::SERVICE_ACTIVE) {
				DEBUG_TRACE("SensorService: %s: GNSS active - start sampling", get_name());
				m_sensor_background_active = true;
				m_sample_number = 0;
				reset_samples();
				service_reschedule(true);
			} else if (e.event_type == ServiceEventType::SERVICE_LOG_UPDATED
			           || e.event_type == ServiceEventType::SERVICE_INACTIVE) {
				if (m_sensor_background_active) {
					DEBUG_TRACE("SensorService: %s: GNSS complete - force stop sampling", get_name());
					sensor_handler(false, true);
				}
			}
		}
	}

	void service_initiate() override { sensor_handler(); }

	unsigned int service_next_schedule_in_ms() override {
		if (m_sensor_background_active) {
			if (m_sample_number == 0) return 0U;
			return sensor_tx_periodic();
		} else {
			return sensor_periodic();
		}
	}

	bool service_is_enabled() override { return sensor_is_enabled(); }

	void service_init() override {
		if (sensor_num_channels() > MAX_SENSOR_CHANNELS) {
			DEBUG_ERROR("SensorService: %s: num_channels %u exceeds MAX_SENSOR_CHANNELS %u", get_name(),
			            sensor_num_channels(), MAX_SENSOR_CHANNELS);
		}
		m_sensor_background_active = false;
		m_sample_number = 0;
		reset_samples();
		unsigned int max_s = sensor_max_samples();
		if (max_s > 0) {
			for (unsigned int chan = 0; chan < safe_num_channels(); chan++)
				m_samples[chan].reserve(max_s);
		}
		sensor_init();
	}

	void reset_samples() {
		for (unsigned int chan = 0; chan < safe_num_channels(); chan++)
			m_samples[chan].clear();
		m_ready_valid = false;
	}

	void service_term() override { sensor_term(); }
	bool service_is_usable_underwater() override { return sensor_is_usable_underwater(); }
	unsigned int service_next_timeout() override { return 0U; }

	// === Virtual interface — subclasses must/may override ===

	/// @brief Init sensor hardware (called once at service start).
	virtual void sensor_init() {}
	/// @brief Terminate sensor hardware (called once at service stop).
	virtual void sensor_term() {}
	/// @brief Check if this sensor is enabled in config.
	/// @return true if the sensor param (e.g. ALS_SENSOR_ENABLE) is set.
	virtual bool sensor_is_enabled() { return false; }
	/// @brief TX aggregation mode (OFF/ONESHOT/MEAN/MEDIAN).
	/// @return Mode from config param.
	virtual BaseSensorEnableTxMode sensor_enable_tx_mode() { return BaseSensorEnableTxMode::OFF; }
	/// @brief Populate a log entry from sensor data (pure virtual — subclass must implement).
	/// @param e     Log entry buffer (reinterpret_cast to sensor-specific LogEntry).
	/// @param data  Sensor reading (port[0..N] = channel values).
	virtual void sensor_populate_log_entry(LogEntry *e, ServiceSensorData &data) = 0;
	/// @brief Sampling period in ms (from config param, 0 = disabled).
	/// @return Period in ms, or SCHEDULE_DISABLED.
	virtual unsigned int sensor_periodic() { return 1000U; }
	/// @brief TX aggregation sampling period in ms.
	/// @return Period in ms between TX samples.
	virtual unsigned int sensor_tx_periodic() { return 1000U; }
	/// @brief Max number of TX aggregation samples per burst.
	/// @return Max samples (0 = unlimited).
	virtual unsigned int sensor_max_samples() { return 1U; }
	/// @brief Number of data channels (e.g. 1 for ALS, 3 for CDT, 6 for AXL).
	/// @return Channel count.
	virtual unsigned int sensor_num_channels() { return 1U; }

	// Bounded channel count to prevent out-of-bounds access on m_samples[]
	unsigned int safe_num_channels() {
		unsigned int n = sensor_num_channels();
		return (n > MAX_SENSOR_CHANNELS) ? MAX_SENSOR_CHANNELS : n;
	}
	virtual bool sensor_is_usable_underwater() { return true; }
};
