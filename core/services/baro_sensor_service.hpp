#pragma once

#include "logger.hpp"
#include "messages.hpp"
#include "sensor_service.hpp"
#include "timeutils.hpp"

#include "bsp.hpp"
#include "gpio.hpp"

struct __attribute__((packed)) BAROLogEntry {
	LogHeader header;
	union {
		double digital_value;
		uint8_t data[MAX_LOG_PAYLOAD];
	};
};

class BAROLogFormatter : public LogFormatter {
public:
    const std::string header() override {
        return "log_datetime,digital_value\r\n";
    }
    const std::string log_entry(const LogEntry& e) override {
        char entry[512], d1[128];
        const BAROLogEntry *log = (const BAROLogEntry *)&e;
        std::time_t t;
        std::tm *tm;

        t = convert_epochtime(log->header.year, log->header.month, log->header.day, log->header.hours, log->header.minutes, log->header.seconds);
        tm = std::gmtime(&t);
        std::strftime(d1, sizeof(d1), "%d/%m/%Y %H:%M:%S", tm);

        // Convert to CSV
        snprintf(entry, sizeof(entry), "%s,%d\r\n",
                d1,
                log->digital_value);
        return std::string(entry);
    }
};


class BAROSensorService : public SensorService {
public:
    BAROSensorService (Sensor& sensor, Logger *logger = nullptr) : SensorService(sensor, ServiceIdentifier::BARO_SENSOR, "BARO", logger) {}

private:
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
    void sensor_populate_log_entry(LogEntry *e, ServiceSensorData& data) override {
		BAROLogEntry *log = (BAROLogEntry *)e;
        log->digital_value = data.port[0];
		service_set_log_header_time(log->header, service_current_time());
    }
#pragma GCC diagnostic pop

	unsigned int sensor_max_samples() override {
		return service_read_param<unsigned int>(ParamID::BARO_SENSOR_ENABLE_TX_MAX_SAMPLES);
	}

	unsigned int sensor_num_channels() override { return 1U; }

	bool sensor_is_enabled() override {
		return service_read_param<bool>(ParamID::BARO_SENSOR_ENABLE);
	}

	unsigned int sensor_periodic() override {
		unsigned int schedule =
				1000 * service_read_param<unsigned int>(ParamID::BARO_SENSOR_PERIODIC);
		return schedule == 0 ? Service::SCHEDULE_DISABLED : schedule;
	}

	unsigned int sensor_tx_periodic() override {
		return service_read_param<unsigned int>(ParamID::BARO_SENSOR_ENABLE_TX_SAMPLE_PERIOD);
	}

	BaseSensorEnableTxMode sensor_enable_tx_mode() override {
		return service_read_param<BaseSensorEnableTxMode>(ParamID::BARO_SENSOR_ENABLE_TX_MODE);
	}

	/*-- BARO POWER SUPPLY MANAGEMENT --*/
	bool m_baroEnabled = false;

	void baro_enable_supply(bool enable) {
		if(enable)
		{
			GPIOPins::set(BARO_PWR_EN);
			PMU::delay_ms(2000);
		}
		else
		{
  			GPIOPins::clear(BARO_PWR_EN);
		}
	}

	void service_initiate() override {
		if (!m_baroEnabled)
		{
			DEBUG_TRACE("BAROSensorService::service_initiate: ENABLE baro power supply");
			baro_enable_supply(true);
			m_baroEnabled = true;
		}
		SensorService::sensor_handler();
	}

	void sensor_service_terminate(void) override {
		baro_enable_supply(false);
		m_baroEnabled = false;
		DEBUG_TRACE("BAROSensorService::sensor_service_terminate => Disable baro power supply");
	}
};
