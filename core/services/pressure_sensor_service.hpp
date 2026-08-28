/**
 * @file pressure_sensor_service.hpp
 * @brief Pressure sensor service — periodic sampling, barometric altitude, UW threshold logging.
 */

#pragma once

#include "logger.hpp"
#include "messages.hpp"
#include "sensor_service.hpp"
#include "timeutils.hpp"
#include "error.hpp"
#include <cmath>

/// @brief Log entry for pressure sensor (pressure, temperature, altitude).
struct __attribute__((packed)) PressureLogEntry {
	LogHeader header;
	union {
		struct {
			double pressure;
			double temperature;
			double altitude;
		};
		uint8_t data[MAX_LOG_PAYLOAD];
	};
};

class PressureLogFormatter : public LogFormatter {
public:
	const std::string header() override { return "log_datetime,pressure,temperature,altitude\r\n"; }
	const std::string log_entry(const LogEntry &e) override {
		char entry[512], d1[128];
		const auto *log = reinterpret_cast<const PressureLogEntry *>(&e);
		std::time_t t;
		std::tm *tm;

		t = convert_epochtime(log->header.year, log->header.month, log->header.day, log->header.hours,
		                      log->header.minutes, log->header.seconds);
		tm = std::gmtime(&t);
		std::strftime(d1, sizeof(d1), "%d/%m/%Y %H:%M:%S", tm);

		// Convert to CSV
		snprintf(entry, sizeof(entry), "%s,%f,%f,%.2f\r\n", d1, log->pressure, log->temperature, log->altitude);
		return std::string(entry);
	}
};

enum class PressureSensorPort : unsigned int {
	PRESSURE,
	TEMPERATURE,
};


class PressureSensorService : public SensorService {
public:
	PressureSensorService(Sensor &sensor, Logger *logger = nullptr)
	    : SensorService(sensor, ServiceIdentifier::PRESSURE_SENSOR, "PRESSURE", logger),
	      m_last_pressure(0) {}

private:
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
	double m_last_pressure;

#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
	void sensor_populate_log_entry(LogEntry *e, ServiceSensorData &data) override {
		auto *log = reinterpret_cast<PressureLogEntry *>(e);
		log->pressure = data.port[static_cast<unsigned int>(PressureSensorPort::PRESSURE)];
		log->temperature = data.port[static_cast<unsigned int>(PressureSensorPort::TEMPERATURE)];

		// Compute barometric altitude: altitude = 44330 * (1 - (P/P0)^(1/5.255))
		// Sensor pressure is in bars, 1 bar = 1000 hPa
		double sea_level_hpa = 1013.25;
		m_sensor.calibration_read(sea_level_hpa, 0);
		double pressure_hpa = log->pressure * 1000.0;
		if (sea_level_hpa > 0.0 && pressure_hpa > 0.0) {
			log->altitude = 44330.0 * (1.0 - std::pow(pressure_hpa / sea_level_hpa, 1.0 / 5.255));
		} else {
			log->altitude = 0.0;
		}
		DEBUG_INFO("PressureSensorService: pressure=%.4f bar | temp=%.2f C | altitude=%.2f m", log->pressure,
		           log->temperature, log->altitude);

		// Check pressure logging mode
		BasePressureSensorLoggingMode mode =
		    service_read_param<BasePressureSensorLoggingMode>(ParamID::PRESSURE_SENSOR_LOGGING_MODE);
		if (mode == BasePressureSensorLoggingMode::ALWAYS) {
			service_set_log_header_time(log->header, service_current_time());
		} else if (mode == BasePressureSensorLoggingMode::UW_THRESHOLD) {
			double uw_threshold = service_read_param<double>(ParamID::UNDERWATER_DETECT_THRESH);
			if ((m_last_pressure < uw_threshold && log->pressure >= uw_threshold)
			    || (m_last_pressure >= uw_threshold && log->pressure < uw_threshold)) {
				// Trigger criteria of submerged or surfaced is met
				m_last_pressure = log->pressure;
				service_set_log_header_time(log->header, service_current_time());
			} else {
				// Don't log if trigger criteria is not met
				m_last_pressure = log->pressure;
				throw ErrorCode::RESOURCE_NOT_AVAILABLE;
			}
		}
	}
#pragma GCC diagnostic pop

	unsigned int sensor_max_samples() override {
		return service_read_param<unsigned int>(ParamID::PRESSURE_SENSOR_ENABLE_TX_MAX_SAMPLES);
	}

	unsigned int sensor_num_channels() override { return 2U; }

	bool sensor_is_enabled() override { return service_read_param<bool>(ParamID::PRESSURE_SENSOR_ENABLE); }

	unsigned int sensor_periodic() override {
		unsigned int schedule = 1000 * service_read_param<unsigned int>(ParamID::PRESSURE_SENSOR_PERIODIC);
		return schedule == 0 ? Service::SCHEDULE_DISABLED : schedule;
	}

	unsigned int sensor_tx_periodic() override {
		return service_read_param<unsigned int>(ParamID::PRESSURE_SENSOR_ENABLE_TX_SAMPLE_PERIOD);
	}

	bool sensor_is_usable_underwater() override { return true; }

	void sensor_init() override {
		unsigned int fs = static_cast<unsigned int>(
		    service_read_param<BasePressureSensorFullScale>(ParamID::PRESSURE_SENSOR_FULL_SCALE));
		m_sensor.set_full_scale(fs);
	}

	BaseSensorEnableTxMode sensor_enable_tx_mode() override {
		return service_read_param<BaseSensorEnableTxMode>(ParamID::PRESSURE_SENSOR_ENABLE_TX_MODE);
	}
};
