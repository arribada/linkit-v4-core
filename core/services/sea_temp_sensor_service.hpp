/**
 * @file sea_temp_sensor_service.hpp
 * @brief Sea temperature sensor service — periodic sampling via RTD or TSYS01.
 */

#pragma once

#include "sensor_service.hpp"
#include "scalar_sensor_log.hpp"
#include "logger.hpp"
#include "messages.hpp"
#include "timeutils.hpp"

/// @brief Log entry for sea temperature sensor.
struct __attribute__((packed)) SeaTempLogEntry {
	LogHeader header;
	union {
		double sea_temp;
		uint8_t data[MAX_LOG_PAYLOAD];
	};
};

class SeaTempLogFormatter : public LogFormatter {
public:
	const std::string header() override { return scalar_sensor_csv_header("sea_temp"); }
	const std::string log_entry(const LogEntry &e) override {
		const auto *log = reinterpret_cast<const SeaTempLogEntry *>(&e);
		return scalar_sensor_csv_line(log->header, log->sea_temp);
	}
};


class SeaTempSensorService : public SensorService {
public:
	SeaTempSensorService(Sensor &sensor, Logger *logger)
	    : SensorService(sensor, ServiceIdentifier::SEA_TEMP_SENSOR, "SEA_TEMP", logger) {}

private:
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
	void sensor_populate_log_entry(LogEntry *e, ServiceSensorData &data) override {
		auto *log = reinterpret_cast<SeaTempLogEntry *>(e);
		log->sea_temp = data.port[0];
		service_set_log_header_time(log->header, service_current_time());
	}
#pragma GCC diagnostic pop

	unsigned int sensor_max_samples() override {
		return service_read_param<unsigned int>(ParamID::SEA_TEMP_SENSOR_ENABLE_TX_MAX_SAMPLES);
	}

	unsigned int sensor_num_channels() override { return 1U; }

	bool sensor_is_enabled() override { return service_read_param<bool>(ParamID::SEA_TEMP_SENSOR_ENABLE); }

	unsigned int sensor_periodic() override {
		unsigned int schedule = 1000 * service_read_param<unsigned int>(ParamID::SEA_TEMP_SENSOR_PERIODIC);
		return schedule == 0 ? Service::SCHEDULE_DISABLED : schedule;
	}

	unsigned int sensor_tx_periodic() override {
		return service_read_param<unsigned int>(ParamID::SEA_TEMP_SENSOR_ENABLE_TX_SAMPLE_PERIOD);
	}

	bool sensor_is_usable_underwater() override { return true; }

	BaseSensorEnableTxMode sensor_enable_tx_mode() override {
		return service_read_param<BaseSensorEnableTxMode>(ParamID::SEA_TEMP_SENSOR_ENABLE_TX_MODE);
	}
};
