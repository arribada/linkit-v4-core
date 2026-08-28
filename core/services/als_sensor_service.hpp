/**
 * @file als_sensor_service.hpp
 * @brief Ambient Light Sensor service — periodic lux sampling, logging, TX aggregation.
 */

#pragma once

#include "logger.hpp"
#include "messages.hpp"
#include "sensor_service.hpp"
#include "timeutils.hpp"

/// @brief Log entry for ALS sensor (lux value).
struct __attribute__((packed)) ALSLogEntry {
	LogHeader header;
	union {
		double lumens;
		uint8_t data[MAX_LOG_PAYLOAD];
	};
};

/// @brief CSV log formatter for ALS entries (used by DUMPD command).
class ALSLogFormatter : public LogFormatter {
public:
	const std::string header() override { return "log_datetime,lumens\r\n"; }
	const std::string log_entry(const LogEntry &e) override {
		char entry[512], d1[128];
		const auto *log = reinterpret_cast<const ALSLogEntry *>(&e);
		std::time_t t;
		std::tm *tm;

		t = convert_epochtime(log->header.year, log->header.month, log->header.day, log->header.hours,
		                      log->header.minutes, log->header.seconds);
		tm = std::gmtime(&t);
		std::strftime(d1, sizeof(d1), "%d/%m/%Y %H:%M:%S", tm);

		snprintf(entry, sizeof(entry), "%s,%f\r\n", d1, log->lumens);
		return std::string(entry);
	}
};

/// @brief ALS sensor service — reads ambient light, logs, aggregates for TX.
class ALSSensorService : public SensorService {
public:
	/// @param sensor  ALS hardware sensor instance.
	/// @param logger  Optional logger for persistent storage.
	ALSSensorService(Sensor &sensor, Logger *logger = nullptr)
	    : SensorService(sensor, ServiceIdentifier::ALS_SENSOR, "ALS", logger) {}

private:
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
	/// @brief Populate log entry from sensor data.
	/// @param e     Log entry buffer (reinterpreted as ALSLogEntry).
	/// @param data  Sensor reading (port[0] = lumens).
	void sensor_populate_log_entry(LogEntry *e, ServiceSensorData &data) override {
		auto *log = reinterpret_cast<ALSLogEntry *>(e);
		log->lumens = data.port[0];
		service_set_log_header_time(log->header, service_current_time());
	}
#pragma GCC diagnostic pop

	/// @brief Max TX samples per burst (from ALS_SENSOR_ENABLE_TX_MAX_SAMPLES param).
	/// @return Max samples, or 0 for unlimited.
	unsigned int sensor_max_samples() override {
		return service_read_param<unsigned int>(ParamID::ALS_SENSOR_ENABLE_TX_MAX_SAMPLES);
	}

	/// @brief Number of data channels (1 = lumens).
	/// @return Always 1.
	unsigned int sensor_num_channels() override { return 1U; }

	/// @brief Check if ALS sensor is enabled in config.
	/// @return true if ALS_SENSOR_ENABLE is set.
	bool sensor_is_enabled() override { return service_read_param<bool>(ParamID::ALS_SENSOR_ENABLE); }

	/// @brief Sampling period in ms (from ALS_SENSOR_PERIODIC param, seconds → ms).
	/// @return Period in ms, or SCHEDULE_DISABLED if 0.
	unsigned int sensor_periodic() override {
		unsigned int schedule = 1000 * service_read_param<unsigned int>(ParamID::ALS_SENSOR_PERIODIC);
		return schedule == 0 ? Service::SCHEDULE_DISABLED : schedule;
	}

	/// @brief TX aggregation period in ms (from ALS_SENSOR_ENABLE_TX_SAMPLE_PERIOD param).
	/// @return Period in ms.
	unsigned int sensor_tx_periodic() override {
		return service_read_param<unsigned int>(ParamID::ALS_SENSOR_ENABLE_TX_SAMPLE_PERIOD);
	}

	/// @brief TX aggregation mode (OFF/ONESHOT/MEAN/MEDIAN).
	/// @return Mode from ALS_SENSOR_ENABLE_TX_MODE param.
	BaseSensorEnableTxMode sensor_enable_tx_mode() override {
		return service_read_param<BaseSensorEnableTxMode>(ParamID::ALS_SENSOR_ENABLE_TX_MODE);
	}
};
