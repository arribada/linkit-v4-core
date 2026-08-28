/**
 * @file oem_ph.cpp
 * @brief Atlas Scientific OEM pH sensor — I2C big-endian register driver.
 */

#include <cstdint>
#include "oem_ph.hpp"
#include "nrf_i2c.hpp"
#include "bsp.hpp"
#include "error.hpp"
#include "gpio.hpp"
#include "nrf_delay.h"
#include "debug.hpp"

/// @brief Init: LED off, hibernate → wake, confirm device present.
OEM_PH_Sensor::OEM_PH_Sensor() : Sensor("PH") {
	readReg<uint8_t>(RegAddr::LED_CTRL);
}

template <typename T> void OEM_PH_Sensor::writeReg(RegAddr address, T value) {
	uint8_t buffer[5];
	buffer[0] = static_cast<uint8_t>(address);

	// Reverse byte order (little-endian ARM → big-endian I2C)
	for (size_t i = 0; i < sizeof(T); ++i)
		buffer[i + 1] = reinterpret_cast<uint8_t *>(&value)[sizeof(T) - 1 - i];

	NrfI2C::write(OEM_PH_DEVICE, OEM_PH_DEVICE_ADDR, buffer, sizeof(T) + 1, false);
}

template <typename T> T OEM_PH_Sensor::readReg(RegAddr address) {
	uint8_t buffer[4];
	T little_endian = 0;

	NrfI2C::write(OEM_PH_DEVICE, OEM_PH_DEVICE_ADDR, reinterpret_cast<const uint8_t *>(&address), sizeof(address),
	              false);
	NrfI2C::read(OEM_PH_DEVICE, OEM_PH_DEVICE_ADDR, buffer, sizeof(T));

	// Reverse byte order (big-endian I2C → little-endian ARM)
	for (size_t i = 0; i < sizeof(T); ++i)
		reinterpret_cast<uint8_t *>(&little_endian)[i] = buffer[sizeof(T) - 1 - i];

	return little_endian;
}

/// @brief Read pH: wake device, apply temperature compensation, trigger reading, poll DRDY (5s timeout).
/// @param offset  Unused (only channel 0).
/// @return pH value (e.g. 7.123).
/// @throws ErrorCode::I2C_COMMS_ERROR on timeout.
double OEM_PH_Sensor::read(unsigned int) {
	// Turn off the LED when sampling
	writeReg<uint8_t>(RegAddr::LED_CTRL, static_cast<uint8_t>(0x00));

	// Set the last temperature reading
	set_temperature_if_set();

	// Put the device into active mode
	writeReg(RegAddr::ACTIVE_HIBERNATE, static_cast<uint8_t>(0x01));

	// Poll the device waiting for a new reading (timeout: 5 seconds)
	{
		unsigned int timeout = 5000;
		while (!readReg<uint8_t>(RegAddr::NEW_READING_AVAILABLE)) {
			nrf_delay_ms(1);
			if (--timeout == 0) {
				DEBUG_ERROR("OEM_PH: reading timeout");
				throw ErrorCode::I2C_COMMS_ERROR;
			}
		}
	}
	uint32_t reading_u32 = readReg<uint32_t>(RegAddr::PH_READING);

	// Convert reading to uint16_t and return it
	return static_cast<double>(reading_u32) / 1000.0;
}

/// @brief Calibration commands: 0=mid(pH7), 1=low(pH4), 2=high(pH10), 3=clear, 4=set temp compensation.
/// @param calibration_data    Temperature value for offset=4 (× 100), unused otherwise.
/// @param calibration_offset  SCALW offset (0-4).
void OEM_PH_Sensor::calibration_write(const double calibration_data, const unsigned int calibration_offset) {
	// Calibration_offset
	// 0 = Reset all calibration values
	// 1 = 7.0 pH Midpoint
	// 2 = 4.0 pH Low point
	// 3 = 10.0 pH High point
	// 4 = Temperature (this is only applied when reading the sensor value)

	// NOTE: the calibration must be done in this order!

	// Set the last temperature reading
	set_temperature_if_set();

	switch (calibration_offset) {
	case 0:
		writeReg<uint8_t>(RegAddr::CALIBRATION_REQUEST, 1U);  // Clear Calibration
		break;

	case 1:
		writeReg<uint32_t>(RegAddr::CALIBRATION, 7000U);      // 7 pH
		writeReg<uint8_t>(RegAddr::CALIBRATION_REQUEST, 3U);  // Midpoint Calibration
		break;

	case 2:
		writeReg<uint32_t>(RegAddr::CALIBRATION, 4000U);      // 4 pH
		writeReg<uint8_t>(RegAddr::CALIBRATION_REQUEST, 2U);  // Low point Calibration
		break;

	case 3:
		writeReg<uint32_t>(RegAddr::CALIBRATION, 10000U);     // 10 pH
		writeReg<uint8_t>(RegAddr::CALIBRATION_REQUEST, 4U);  // High point Calibration
		break;

	case 4:
		// The OEM device expects 10E2 units, so we convert to 10E2 here
		m_supplied_temperature = static_cast<uint16_t>(calibration_data * 100);
		break;

	default: throw ErrorCode::RESOURCE_NOT_AVAILABLE;
	}
}

/// @brief Write user-supplied temperature compensation to the sensor (if previously set via SCALW offset 4).
void OEM_PH_Sensor::set_temperature_if_set() {
	// Read current calibration status
	uint8_t cnf = readReg<uint8_t>(RegAddr::CALIBRATION_CONFIRMATION);
	DEBUG_TRACE("CALIBRATION_CONFIRMATION=%02x", cnf);

	// WARNING: The temperature compensation feature does not work.  The code is
	// disabled to activate this as it results in frozen values reported
	// from the device.
#if 0
    if (m_supplied_temperature.has_value()) {
        // There is a limitation in the OEM PH sensor device which means that it is not permitted to change
    	// the reference temperature during a calibration procedure.
        // We do not allow the TC register to be updated unless fully calibrated or if no calibration points have
    	// been set (i.e., after calibration has been cleared).  This avoids this register being changed
        // during a calibration procedure.
        if (cnf == 0x0 || cnf == 0x7)
        	writeReg<uint16_t>(RegAddr::TEMPERATURE_COMPENSATION, m_supplied_temperature.value());
    }
#endif
}
