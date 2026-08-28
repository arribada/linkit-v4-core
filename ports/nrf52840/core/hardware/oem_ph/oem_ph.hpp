#pragma once

/**
 * @file oem_ph.hpp
 * @brief Atlas Scientific OEM pH sensor driver (I2C, big-endian registers).
 *
 * Calibration write offsets (SCALW):
 *   0: Calibrate mid-point (pH 7.0)
 *   1: Calibrate low-point (pH 4.0)
 *   2: Calibrate high-point (pH 10.0)
 *   3: Clear calibration
 *   4: Set temperature compensation (value × 100, e.g. 25.00°C = 2500)
 */

#include <cstdint>
#include <optional>
#include "sensor.hpp"

class OEM_PH_Sensor : public Sensor {
public:
	/// @throws ErrorCode::I2C_COMMS_ERROR if device not found on I2C bus.
	OEM_PH_Sensor();

	/// @param offset  Unused (only channel 0 = pH reading × 1000).
	/// @return pH value (e.g. 7.123).
	/// @throws ErrorCode::I2C_COMMS_ERROR on timeout.
	double read(unsigned int offset) override;

	/// @param value   Calibration value (interpretation depends on offset).
	/// @param offset  SCALW offset (see table above).
	void calibration_write(const double value, const unsigned int offset) override;

private:
	/// @brief Write temperature compensation to sensor if previously set.
	void set_temperature_if_set();

	std::optional<uint16_t> m_supplied_temperature;  ///< User-supplied temperature compensation (× 100)

	enum class RegAddr : uint8_t {
		DEVICE_TYPE = 0x00,       // Read-only
		FIRMWARE_VERSION = 0x01,  // Read-only
		ADDRESS_LOCK_UNLOCK = 0x02,
		ADDRESS = 0x03,
		INTERRUPT_CTRL = 0x04,
		LED_CTRL = 0x05,
		ACTIVE_HIBERNATE = 0x06,
		NEW_READING_AVAILABLE = 0x07,
		CALIBRATION = 0x08,
		CALIBRATION_REQUEST = 0x0C,
		CALIBRATION_CONFIRMATION = 0x0D,  // Read-only
		TEMPERATURE_COMPENSATION = 0x0E,
		TEMPERATURE_CONFIRMATION = 0x12,  // Read-only
		PH_READING = 0x16                 // Read-only
	};

	template <typename T> T readReg(RegAddr address);

	template <typename T> void writeReg(RegAddr address, T value);
};
