/**
 * @file oem_rtd.cpp
 * @brief Atlas Scientific OEM RTD temperature sensor — I2C big-endian register driver.
 */

#include <cstdint>
#include <array>
#include "oem_rtd.hpp"
#include "nrf_i2c.hpp"
#include "bsp.hpp"
#include "error.hpp"
#include "gpio.hpp"
#include "debug.hpp"
#include "nrf_delay.h"

OEM_RTD_Sensor::OEM_RTD_Sensor() : Sensor("RTD") {
	readReg<uint8_t>(RegAddr::LED_CTRL);
}

/// @brief Write a big-endian register value via I2C.
/// @param address  Register address.
/// @param value    Value to write (byte-swapped to big-endian before sending).
template <typename T> void OEM_RTD_Sensor::writeReg(RegAddr address, T value) {
	std::array<uint8_t, 1 + sizeof(T)> buffer;
	buffer[0] = static_cast<uint8_t>(address);

	// Reverse byte order (little-endian ARM → big-endian I2C)
	for (size_t i = 0; i < sizeof(T); ++i)
		buffer[i + 1] = reinterpret_cast<uint8_t *>(&value)[sizeof(T) - 1 - i];

	NrfI2C::write(OEM_RTD_DEVICE, OEM_RTD_DEVICE_ADDR, buffer.data(), buffer.size(), false);
}

template <typename T> T OEM_RTD_Sensor::readReg(RegAddr address) {
	T big_endian;
	T little_endian;

	NrfI2C::write(OEM_RTD_DEVICE, OEM_RTD_DEVICE_ADDR, reinterpret_cast<const uint8_t *>(&address), sizeof(address),
	              false);
	NrfI2C::read(OEM_RTD_DEVICE, OEM_RTD_DEVICE_ADDR, reinterpret_cast<uint8_t *>(&big_endian), sizeof(T));

	// Reverse byte order (big-endian I2C → little-endian ARM)
	for (size_t i = 0; i < sizeof(T); ++i)
		reinterpret_cast<uint8_t *>(&little_endian)[i] = reinterpret_cast<uint8_t *>(&big_endian)[sizeof(T) - 1 - i];

	return little_endian;
}

/// @brief Read temperature: wake device, trigger reading, poll for result, sleep.
/// @param offset  Unused (only channel 0).
/// @return Temperature in °C (value / 1000.0).
/// @throws ErrorCode::I2C_COMMS_ERROR on timeout (5s).
double OEM_RTD_Sensor::read(unsigned int) {
	// Turn off the LED when sampling
	writeReg(RegAddr::LED_CTRL, static_cast<uint8_t>(0x00));

	// Put the device into active mode
	writeReg(RegAddr::ACTIVE_HIBERNATE, static_cast<uint8_t>(0x01));

	// Check calibration status for debug purposes
	uint8_t cnf = readReg<uint8_t>(RegAddr::CALIBRATION_CONFIRMATION);
	DEBUG_TRACE("CALIBRATION_CONFIRMATION: %02x", cnf);

	// Poll the device waiting for a new reading (timeout: 5 seconds)
	{
		unsigned int timeout = 5000;
		while (!readReg<uint8_t>(RegAddr::NEW_READING_AVAILABLE)) {
			nrf_delay_ms(1);
			if (--timeout == 0) {
				DEBUG_ERROR("OEM_RTD: reading timeout");
				throw ErrorCode::I2C_COMMS_ERROR;
			}
		}
	}

	int32_t reading_u32 = readReg<int32_t>(RegAddr::RTD_READING);

	return static_cast<double>(reading_u32) / 1000.0;
}

/// @brief Calibration commands: 0=calibrate at known temp, 1=clear calibration.
/// @param value              Temperature for calibration (offset=0), unused for offset=1.
/// @param calibration_offset 0=calibrate, 1=clear.
void OEM_RTD_Sensor::calibration_write(const double, const unsigned int calibration_offset) {
	// We always calibrate to 0C based on ice melting in water temperature
	writeReg<uint32_t>(RegAddr::CALIBRATION, calibration_offset == 2 ? 100U : 0U);

	// The calibration offset may be set to:
	// 0=>Clear calibration
	// 1=>Single point calibration at 0C
	// 2=>Single point calibration at 100C
	switch (calibration_offset) {
	case 0:
		writeReg<uint8_t>(RegAddr::CALIBRATION_REQUEST, 1U);  // Clear calibration
		break;
	case 1:
	case 2:
		writeReg<uint8_t>(RegAddr::CALIBRATION_REQUEST, 2U);  // Single point
		break;
	default: throw ErrorCode::RESOURCE_NOT_AVAILABLE;
	}
}
