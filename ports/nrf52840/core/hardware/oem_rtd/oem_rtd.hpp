#pragma once

/**
 * @file oem_rtd.hpp
 * @brief Atlas Scientific OEM RTD temperature sensor driver (I2C, big-endian registers).
 *
 * Calibration write offsets (SCALW):
 *   0: Calibrate at known temperature (value used)
 *   1: Clear calibration
 */

#include <cstdint>
#include "sensor.hpp"

class OEM_RTD_Sensor : public Sensor {
public:
	/// @throws ErrorCode::I2C_COMMS_ERROR if device not found on I2C bus.
	OEM_RTD_Sensor();

	/// @param offset  Unused (only channel 0 = temperature × 1000).
	/// @return Temperature in °C (e.g. 25.123).
	/// @throws ErrorCode::I2C_COMMS_ERROR on timeout.
	double read(unsigned int offset) override;

	/// @param value   Temperature for calibration (offset=0), unused for offset=1.
	/// @param offset  0=calibrate at value, 1=clear calibration.
	void calibration_write(const double value, const unsigned int offset) override;

private:
	enum class RegAddr : uint8_t {
		DEVICE_TYPE = 0x00,
		FIRMWARE_VERSION = 0x01,
		ADDRESS_LOCK_UNLOCK = 0x02,
		ADDRESS = 0x03,
		INTERRUPT_CTRL = 0x04,
		LED_CTRL = 0x05,
		ACTIVE_HIBERNATE = 0x06,
		NEW_READING_AVAILABLE = 0x07,
		CALIBRATION = 0x08,
		CALIBRATION_REQUEST = 0x0C,
		CALIBRATION_CONFIRMATION = 0x0D,
		RTD_READING = 0x0E,
	};

	template <typename T> T readReg(RegAddr address);

	template <typename T> void writeReg(RegAddr address, T value);
};
