/**
 * @file thermistor_sensor_service.hpp
 * @brief Thermistor temperature sensor service — periodic sampling, wakeup threshold detection.
 */

#pragma once

#include "sensor_service.hpp"
#include "logger.hpp"
#include "messages.hpp"
#include "timeutils.hpp"

/// @brief Log entry for thermistor sensor (body temperature).
struct __attribute__((packed)) ThermistorLogEntry {
	LogHeader header;
	union {
		double temp;
		uint8_t data[MAX_LOG_PAYLOAD];
	};
};

class ThermistorLogFormatter : public LogFormatter {
public:
	const std::string header() override { return "log_datetime,temp\r\n"; }
	const std::string log_entry(const LogEntry &e) override {
		char entry[128], d1[25];
		const auto *log = reinterpret_cast<const ThermistorLogEntry *>(&e);
		std::time_t t;
		std::tm *tm;

		t = convert_epochtime(log->header.year, log->header.month, log->header.day, log->header.hours,
		                      log->header.minutes, log->header.seconds);
		tm = std::gmtime(&t);
		std::strftime(d1, sizeof(d1), "%d/%m/%Y %H:%M:%S", tm);

		// Convert to CSV
		snprintf(entry, sizeof(entry), "%s,%f\r\n", d1, log->temp);
		return std::string(entry);
	}
};


class ThermistorSensorService : public SensorService {
public:
	ThermistorSensorService(Sensor &sensor, Logger *logger)
	    : SensorService(sensor, ServiceIdentifier::THERMISTOR_SENSOR, "THERMISTOR", logger) {}

private:
	unsigned int count_threshold_value_crossed = 0;

	bool sensor_trigger_event(double value) {
		unsigned int wakeup_samples = service_read_param<unsigned int>(ParamID::THERMISTOR_SENSOR_WAKEUP_SAMPLES);
		if ((wakeup_samples != 0) && (service_read_param<bool>(ParamID::THERMISTOR_SENSOR_ENABLE))) {
			if (value > service_read_param<double>(ParamID::THERMISTOR_SENSOR_WAKEUP_THRESH)) {
				count_threshold_value_crossed++;
				DEBUG_TRACE("ThermistorSensorService: %s: threshold value crossed %d (samples required : %d) ",
				            get_name(), count_threshold_value_crossed, wakeup_samples);
				if (count_threshold_value_crossed >= wakeup_samples) {
					count_threshold_value_crossed = 0;
					return true;
				}
			}
		}
		return false;
	}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
	void sensor_populate_log_entry(LogEntry *e, ServiceSensorData &data) override {
		auto *log = reinterpret_cast<ThermistorLogEntry *>(e);
		log->temp = data.port[0];
		service_set_log_header_time(log->header, service_current_time());
		if (sensor_trigger_event(log->temp)) {
			DEBUG_TRACE("ThermistorSensorService: %s: Thermistor wakeup event handler", get_name());

			ServiceEvent event;
			event.event_type = ServiceEventType::GNSS_ON;
			event.event_source = ServiceIdentifier::THERMISTOR_SENSOR;
			event.event_data = true;
			ServiceManager::notify_peer_event(event);
		}
	}
#pragma GCC diagnostic pop

	unsigned int sensor_max_samples() override {
		return service_read_param<unsigned int>(ParamID::THERMISTOR_SENSOR_ENABLE_TX_MAX_SAMPLES);
	}

	unsigned int sensor_num_channels() override { return 1U; }

	bool sensor_is_enabled() override { return service_read_param<bool>(ParamID::THERMISTOR_SENSOR_ENABLE); }

	unsigned int sensor_periodic() override {
		unsigned int schedule = 1000 * service_read_param<unsigned int>(ParamID::THERMISTOR_SENSOR_PERIODIC);
		return schedule == 0 ? Service::SCHEDULE_DISABLED : schedule;
	}

	unsigned int sensor_tx_periodic() override {
		return service_read_param<unsigned int>(ParamID::THERMISTOR_SENSOR_ENABLE_TX_SAMPLE_PERIOD);
	}

	bool sensor_is_usable_underwater() override { return true; }

	BaseSensorEnableTxMode sensor_enable_tx_mode() override {
		return service_read_param<BaseSensorEnableTxMode>(ParamID::THERMISTOR_SENSOR_ENABLE_TX_MODE);
	}
};
