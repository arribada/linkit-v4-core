#pragma once

/**
 * @file pressure_sensor.hpp
 * @brief Abstract pressure sensor device interface + PressureSensor high-level class.
 *
 * PressureSensorDevice: abstract I2C device (Bar100, MS58xx, LPS28DFW).
 * PressureSensor: high-level Sensor class with sea-level calibration and full-scale modes.
 */

#include <cstdint>
#include <functional>
#include <string>
#include "sensor.hpp"
#include "bsp.hpp"
#include "error.hpp"


class PressureSensorDevice {
public:
	virtual ~PressureSensorDevice() = default;
	virtual void read(double &temperature, double &pressure) = 0;
	virtual void set_full_scale(unsigned int mode) { (void)mode; }
};

/// @brief Dummy pressure sensor for testing — always returns 0.
class PressureSensorDummyDevice : public PressureSensorDevice {
public:
	void read(double &t, double &p) override {
		t = 0;
		p = 0;
	}
};

/**
 * Calibration write offsets (SCALW device_id=1):
 *   0 = sea level pressure (hPa), default 1013.25
 *
 * Full scale modes (set via PRESSURE_SENSOR_FULL_SCALE param):
 *   0 = 1260 hPa (surface only, better precision)
 *   1 = 4060 hPa (surface + underwater)
 */
class PressureSensor : public Sensor {
public:
	PressureSensor(PressureSensorDevice &device) : Sensor("PRS"), m_device(device), m_cal("PRS") {
		try {
			m_sea_level_hpa = m_cal.read(0);
		} catch (...) {
			m_sea_level_hpa = 1013.25;
		}
		try {
			m_temp_offset = m_cal.read(1);
		} catch (...) {
			m_temp_offset = 0.0;
		}
	}
	double read(unsigned int channel = 0) {
		if (0 == channel) {
			m_device.read(m_last_temperature, m_last_pressure);
			return m_last_pressure;
		} else if (1 == channel) {
			return m_last_temperature + m_temp_offset;
		}
		throw ErrorCode::BAD_SENSOR_CHANNEL;
	}

	void calibration_write(const double value, const unsigned int offset) override {
		if (offset == 0) {
			m_sea_level_hpa = value;
			m_cal.write(0, value);
			m_cal.save();
			DEBUG_TRACE("PressureSensor: sea_level_hpa calibrated to %.2f", m_sea_level_hpa);
		} else if (offset == 1) {
			m_temp_offset = value;
			m_cal.write(1, value);
			m_cal.save();
			DEBUG_TRACE("PressureSensor: temp_offset calibrated to %.2f", m_temp_offset);
		}
	}

	void calibration_read(double &value, const unsigned int offset) override {
		if (offset == 0) {
			value = m_sea_level_hpa;
		} else if (offset == 1) {
			value = m_temp_offset;
		} else {
			value = 0.0;
		}
	}

	void set_full_scale(unsigned int mode) { m_device.set_full_scale(mode); }

private:
	double m_last_pressure = 0.0;
	double m_last_temperature = 0.0;
	double m_sea_level_hpa = 1013.25;
	double m_temp_offset = 0.0;
	PressureSensorDevice &m_device;
	Calibration m_cal;
};
