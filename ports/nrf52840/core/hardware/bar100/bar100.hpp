#pragma once

/**
 * @file bar100.hpp
 * @brief Blue Robotics Bar100 pressure/temperature sensor driver (I2C).
 *
 * The Bar100 is a high-pressure (up to 100 bar) depth sensor used for
 * deep-dive profiling.  Reads pressure and temperature via I2C.
 * Supports PA (vented gauge), PR (sealed gauge), and PAA (absolute) modes.
 */

#include <cstdint>
#include "pressure_sensor.hpp"
#include "bsp.hpp"

class Bar100 : public PressureSensorDevice {
public:
	/// @param bus      I2C bus index.
	/// @param address  7-bit I2C address.
	/// @throws ErrorCode::I2C_COMMS_ERROR if device not responding.
	Bar100(unsigned int bus, unsigned char address);

	void read(double &temperature, double &pressure) override;

private:
	unsigned int m_bus;
	unsigned char m_addr;
	double m_pmin;         ///< Factory min pressure (bar)
	double m_pmax;         ///< Factory max pressure (bar)
	double m_mode_offset;  ///< Offset for PA/PR/PAA mode (bar)

	/// @brief Send a register command, optionally read back data after a delay.
	void command(uint8_t reg, uint8_t *read_buffer = nullptr, unsigned int length = 0, unsigned int delay_ms = 0);
};
