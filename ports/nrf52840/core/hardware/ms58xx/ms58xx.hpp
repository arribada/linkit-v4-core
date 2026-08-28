#pragma once

/**
 * @file ms58xx.hpp
 * @brief MS5803/MS5837 pressure sensor driver (I2C, Measurement Specialties).
 *
 * Supports MS5803-14BA (14 bar), MS5837-30BA (30 bar), and MS5837-02BA (2 bar)
 * variants.  Variant-specific second-order compensation is selected at construction.
 * PROM calibration coefficients are CRC-checked.
 */

#include <cstdint>
#include <functional>
#include <string>
#include "pressure_sensor.hpp"
#include "bsp.hpp"
#include "error.hpp"

class MS58xxLL : public PressureSensorDevice {
public:
	/// @param bus      I2C bus index.
	/// @param address  7-bit I2C address.
	/// @param variant  Variant string ("MS5803_14BA", "MS5837_30BA", "MS5837_02BA").
	/// @throws ErrorCode::I2C_COMMS_ERROR if PROM CRC check fails.
	MS58xxLL(unsigned int bus, unsigned char address, const std::string &variant);

	/// @param[out] temperature  Temperature in °C.
	/// @param[out] pressure     Pressure in bar.
	void read(double &temperature, double &pressure) override;

private:
	unsigned int m_bus;
	unsigned char m_addr;
	enum MS58xxCommand : uint8_t {
		RESET = (0x1E),     // ADC reset command
		PROM = (0xA0),      // PROM location
		ADC_READ = (0x00),  // ADC read command
		ADC_CONV = (0x40),  // ADC conversion command
		ADC_D1 = (0x00),    // ADC D1 conversion
		ADC_D2 = (0x10),    // ADC D2 conversion
		ADC_256 = (0x00),   // ADC resolution=256
		ADC_512 = (0x02),   // ADC resolution=512
		ADC_1024 = (0x04),  // ADC resolution=1024
		ADC_2048 = (0x06),  // ADC resolution=2048
		ADC_4096 = (0x08),  // ADC resolution=4096
		ADC_8192 = (0x0A),  // ADC resolution=8192
	};
	uint16_t C[8] = {};  ///< PROM calibration coefficients (CRC-checked)
	std::function<void(const int32_t, const int32_t, const uint16_t *, double &, double &)>
	    m_convert;  ///< Variant-specific conversion function
	MS58xxCommand m_resolution;
	unsigned int m_prom_size;
	unsigned int m_crc_offset;
	unsigned int m_crc_mask;
	unsigned int m_crc_shift;

	void send_command(uint8_t command);
	void read_coeffs();
	uint32_t sample_adc(uint8_t measurement);
	void check_coeffs();
};
