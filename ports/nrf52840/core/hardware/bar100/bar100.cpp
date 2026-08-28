/**
 * @file bar100.cpp
 * @brief Blue Robotics Bar100 pressure/temperature sensor — I2C driver.
 */

#include <cstring>
#include "bar100.hpp"
#include "nrf_i2c.hpp"
#include "error.hpp"
#include "pmu.hpp"
#include "debug.hpp"

/// @name Bar100 register addresses
/// @{
static constexpr uint8_t CUST_ID0 = 0x00;
static constexpr uint8_t SCALING0 = 0x12;
static constexpr uint8_t SCALING1 = 0x13;
static constexpr uint8_t SCALING2 = 0x14;
static constexpr uint8_t SCALING3 = 0x15;
static constexpr uint8_t SCALING4 = 0x16;
static constexpr uint8_t REQUEST = 0xAC;
static constexpr uint8_t BUSY = 0x20;
/// @}

/// @brief Max time to wait for measurement completion (ms).
static constexpr unsigned int READ_TIMEOUT_MS = 10;


void Bar100::command(uint8_t reg, uint8_t *read_buffer, unsigned int length, unsigned int delay_ms) {
	NrfI2C::write(m_bus, m_addr, &reg, 1, false);
	if (length > 0) {
		PMU::delay_ms(delay_ms);
		NrfI2C::read(m_bus, m_addr, read_buffer, length);
	}
}

/// @brief Read factory scaling registers and determine pressure mode.
Bar100::Bar100(unsigned int bus, unsigned char addr)
    : m_bus(bus),
      m_addr(addr),
      m_pmin(0),
      m_pmax(0),
      m_mode_offset(0) {
	DEBUG_TRACE("Bar100(%u, 0x%02x)", bus, static_cast<unsigned int>(addr));

	uint8_t buf[3];

	command(CUST_ID0, buf, sizeof(buf), 1);
	[[maybe_unused]] uint16_t custid0 = buf[2] | static_cast<uint16_t>(buf[1]) << 8;
	DEBUG_TRACE("Bar100: CUST_ID0=%04X", static_cast<unsigned int>(custid0));

	// Determine pressure mode from SCALING0 register
	command(SCALING0, buf, sizeof(buf), 1);
	uint8_t mode = buf[2] & 0x3;
	DEBUG_TRACE("Bar100: PA_MODE=%u", static_cast<unsigned int>(mode));

	if (mode == 0) {
		m_mode_offset = 1.01325;  // PA: vented gauge, zero at atmospheric
	} else if (mode == 1) {
		m_mode_offset = 1.0;  // PR: sealed gauge, zero at 1.0 bar
	} else {
		m_mode_offset = 0;  // PAA: absolute, zero at vacuum
	}

	// Read factory pmin/pmax as IEEE 754 float (big-endian across 2 registers each)
	auto read_float = [&](uint8_t reg_hi, uint8_t reg_lo) -> float {
		uint32_t raw = 0;
		command(reg_hi, buf, sizeof(buf), 1);
		raw = static_cast<uint32_t>(buf[1]) << 24 | static_cast<uint32_t>(buf[2]) << 16;
		command(reg_lo, buf, sizeof(buf), 1);
		raw |= static_cast<uint32_t>(buf[1]) << 8 | static_cast<uint32_t>(buf[2]);
		float result;
		std::memcpy(&result, &raw, sizeof(result));  // Safe type punning (no UB)
		return result;
	};

	m_pmin = read_float(SCALING1, SCALING2);
	m_pmax = read_float(SCALING3, SCALING4);

	DEBUG_TRACE("Bar100: pmin=%f pmax=%f", m_pmin, m_pmax);
}

/// @brief Request a measurement, wait for completion, decode pressure and temperature.
/// @param[out] temperature  Temperature in degrees Celsius.
/// @param[out] pressure     Pressure in bar (with mode offset applied).
/// @throws ErrorCode::RESOURCE_NOT_AVAILABLE if sensor is busy after timeout.
void Bar100::read(double &temperature, double &pressure) {
	uint8_t buf[5];

	command(REQUEST);

	// Poll BUSY flag with timeout
	unsigned int timeout = READ_TIMEOUT_MS;
	do {
		PMU::delay_ms(1);
		NrfI2C::read(m_bus, m_addr, buf, 1);
	} while ((buf[0] & BUSY) && --timeout > 0);

	if (buf[0] & BUSY) throw ErrorCode::RESOURCE_NOT_AVAILABLE;

	// Read measurement result (status + 2 bytes pressure + 2 bytes temperature)
	NrfI2C::read(m_bus, m_addr, buf, sizeof(buf));

	uint16_t pressure16 = static_cast<uint16_t>(buf[1]) << 8 | buf[2];
	uint16_t temperature16 = static_cast<uint16_t>(buf[3]) << 8 | buf[4];

	temperature = (((temperature16 / 16.0) - 24.0) * 0.05) - 50.0;
	pressure = (pressure16 - 16384.0) * ((m_pmax - m_pmin) / 32768.0) + m_pmin + m_mode_offset;

	DEBUG_TRACE("Bar100: status=%02x p16=%04x t16=%04x p=%f t=%f", static_cast<unsigned int>(buf[0]),
	            static_cast<unsigned int>(pressure16), static_cast<unsigned int>(temperature16), pressure, temperature);
}
