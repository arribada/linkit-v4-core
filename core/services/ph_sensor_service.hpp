/**
 * @file ph_sensor_service.hpp
 * @brief pH sensor service — periodic pH sampling, logging, TX aggregation.
 */

#pragma once

#include "sensor_service.hpp"
#include "scalar_sensor_log.hpp"
#include "logger.hpp"
#include "messages.hpp"
#include "timeutils.hpp"

/// @brief Log entry for pH sensor.
struct __attribute__((packed)) PHLogEntry {
	LogHeader header;
	union {
		double ph;
		uint8_t data[MAX_LOG_PAYLOAD];
	};
};


class PHLogFormatter : public LogFormatter {
public:
	const std::string header() override { return scalar_sensor_csv_header("pH"); }
	const std::string log_entry(const LogEntry &e) override {
		const auto *log = reinterpret_cast<const PHLogEntry *>(&e);
		return scalar_sensor_csv_line(log->header, log->ph);
	}
};


class PHSensorService : public SensorService {
public:
	PHSensorService(Sensor &sensor, Logger *logger)
	    : SensorService(sensor, ServiceIdentifier::PH_SENSOR, "PH", logger) {}

private:
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
	void sensor_populate_log_entry(LogEntry *e, ServiceSensorData &data) override {
		auto *ph = reinterpret_cast<PHLogEntry *>(e);
		ph->ph = data.port[0];
		service_set_log_header_time(ph->header, service_current_time());
	}
#pragma GCC diagnostic pop

	unsigned int sensor_max_samples() override {
		return service_read_param<unsigned int>(ParamID::PH_SENSOR_ENABLE_TX_MAX_SAMPLES);
	}

	unsigned int sensor_num_channels() override { return 1U; }

	bool sensor_is_enabled() override { return service_read_param<bool>(ParamID::PH_SENSOR_ENABLE); }

	unsigned int sensor_periodic() override {
		unsigned int schedule = 1000 * service_read_param<unsigned int>(ParamID::PH_SENSOR_PERIODIC);
		return schedule == 0 ? Service::SCHEDULE_DISABLED : schedule;
	}

	unsigned int sensor_tx_periodic() override {
		return service_read_param<unsigned int>(ParamID::PH_SENSOR_ENABLE_TX_SAMPLE_PERIOD);
	}

	bool sensor_is_usable_underwater() override { return true; }

	BaseSensorEnableTxMode sensor_enable_tx_mode() override {
		return service_read_param<BaseSensorEnableTxMode>(ParamID::PH_SENSOR_ENABLE_TX_MODE);
	}
};
