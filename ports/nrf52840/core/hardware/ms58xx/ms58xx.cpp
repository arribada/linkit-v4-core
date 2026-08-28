/**
 * @file ms58xx.cpp
 * @brief MS5803/MS5837 pressure sensor — variant-specific conversion + PROM CRC.
 */

#include "ms58xx.hpp"
#include "bsp.hpp"
#include "nrf_i2c.hpp"
#include "error.hpp"
#include "pmu.hpp"
#include "debug.hpp"

/// @brief MS5837-30BA conversion: first + second order temperature/pressure compensation.
/// @param D1  Raw pressure ADC value.
/// @param D2  Raw temperature ADC value.
/// @param C   PROM calibration coefficients (C[0]..C[7]).
/// @param[out] temperature  Temperature in °C.
/// @param[out] pressure     Pressure in bar.
static void read_ms5837_30bar(int32_t D1, int32_t D2, const uint16_t *C, double &temperature, double &pressure) {
	int32_t TEMP, dT;
	int64_t OFF, SENS, T2, OFF2, SENS2;

	// First order
	dT = D2 - (static_cast<int32_t>(C[5]) << 8);
	TEMP = ((static_cast<int64_t>(dT) * C[6]) >> 23) + 2000;
	OFF = (static_cast<int64_t>(C[2]) << 16) + ((C[4] * static_cast<int64_t>(dT)) >> 7);
	SENS = (static_cast<int64_t>(C[1]) << 15) + ((C[3] * static_cast<int64_t>(dT)) >> 8);

	// Second order
	if (TEMP < 2000) {
		T2 = 3 * ((static_cast<int64_t>(dT) * dT) >> 33);
		OFF2 = 3 * ((TEMP - 2000) * (TEMP - 2000)) >> 1;
		SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) >> 3;
		if (TEMP < -1500) {
			OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
			SENS2 = SENS2 + 4 * ((TEMP + 1500) * (TEMP + 1500));
		}
	} else {
		T2 = 2 * ((static_cast<int64_t>(dT) * dT) >> 37);
		OFF2 = 1 * ((TEMP - 2000) * (TEMP - 2000)) >> 4;
		SENS2 = 0;
	}

	TEMP -= T2;
	OFF -= OFF2;
	SENS -= SENS2;

	pressure = static_cast<double>((((SENS * D1) >> 21) - OFF) >> 13) / 10000.0;
	temperature = TEMP / 100.0;

	DEBUG_TRACE("MS58xx 30bar: %f bar @ %f °C", pressure, temperature);
}

/// @brief MS5803-14BA conversion: first + second order compensation with low/very-low temp branches.
/// @param D1  Raw pressure ADC value.
/// @param D2  Raw temperature ADC value.
/// @param C   PROM calibration coefficients.
/// @param[out] temperature  Temperature in °C.
/// @param[out] pressure     Pressure in bar.
static void read_ms5803_14bar(int32_t D1, int32_t D2, const uint16_t *C, double &temperature, double &pressure) {
	int32_t TEMP, dT;
	int64_t OFF, SENS, T2, OFF2, SENS2;

	// First order
	dT = D2 - (static_cast<int32_t>(C[5]) << 8);
	TEMP = ((static_cast<int64_t>(dT) * C[6]) >> 23) + 2000;
	OFF = (static_cast<int64_t>(C[2]) << 16) + ((C[4] * static_cast<int64_t>(dT)) >> 7);
	SENS = (static_cast<int64_t>(C[1]) << 15) + ((C[3] * static_cast<int64_t>(dT)) >> 8);

	// Second order
	if (TEMP < 2000) {
		T2 = 3 * ((static_cast<int64_t>(dT) * dT) >> 33);
		OFF2 = 3 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
		SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) >> 3;
		if (TEMP < -1500) {
			OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
			SENS2 = SENS2 + 4 * ((TEMP + 1500) * (TEMP + 1500));
		}
	} else {
		T2 = 7 * (static_cast<uint64_t>(dT) * dT) / (1ULL << 37);
		OFF2 = ((TEMP - 2000) * (TEMP - 2000)) / 16;
		SENS2 = 0;
	}

	TEMP -= T2;
	OFF -= OFF2;
	SENS -= SENS2;

	pressure = static_cast<double>((((SENS * D1) >> 21) - OFF) >> 15) / 10000.0;
	temperature = TEMP / 100.0;

	DEBUG_TRACE("MS58xx 14bar: %f bar @ %f °C", pressure, temperature);
}

/// @brief MS5837-02BA conversion: higher precision (2 bar range), different second-order coefficients.
/// @param D1  Raw pressure ADC value.
/// @param D2  Raw temperature ADC value.
/// @param C   PROM calibration coefficients.
/// @param[out] temperature  Temperature in °C.
/// @param[out] pressure     Pressure in bar.
static void read_ms5837_02bar(int32_t D1, int32_t D2, const uint16_t *C, double &temperature, double &pressure) {
	int32_t TEMP, dT;
	int64_t OFF, SENS, T2, OFF2, SENS2;

	// First order (different bit shifts than 30bar variant)
	dT = D2 - (static_cast<int32_t>(C[5]) << 8);
	TEMP = ((static_cast<int64_t>(dT) * C[6]) >> 23) + 2000;
	OFF = (static_cast<int64_t>(C[2]) << 17) + ((C[4] * static_cast<int64_t>(dT)) >> 6);
	SENS = (static_cast<int64_t>(C[1]) << 16) + ((C[3] * static_cast<int64_t>(dT)) >> 7);

	// Second order (MS5837-02BA datasheet coefficients: T2=3, OFF2=3, SENS2=63)
	if (TEMP < 2000) {
		T2 = 3 * ((static_cast<int64_t>(dT) * dT) >> 35);
		OFF2 = 3 * ((TEMP - 2000) * (TEMP - 2000)) >> 3;
		SENS2 = 63 * ((TEMP - 2000) * (TEMP - 2000)) >> 5;
	} else {
		T2 = 0;
		OFF2 = 0;
		SENS2 = 0;
	}

	TEMP -= T2;
	OFF -= OFF2;
	SENS -= SENS2;

	pressure = static_cast<double>((((SENS * D1) >> 21) - OFF) >> 15) / 10000.0;
	temperature = TEMP / 100.0;

	DEBUG_TRACE("MS58xx 02bar: %f bar @ %f °C", pressure, temperature);
}


// ═══════════════════════════════════════════════════════
//  MS58xxLL implementation
// ═══════════════════════════════════════════════════════

/// @brief Detect variant, configure PROM layout, read and CRC-check coefficients.
/// @param bus      I2C bus index.
/// @param addr     7-bit I2C address.
/// @param variant  Variant string ("MS5803_14BA", "MS5837_30BA", "MS5837_02BA").
/// @throws ErrorCode::NOT_IMPLEMENTED if variant is not supported.
/// @throws ErrorCode::I2C_CRC_FAILURE if PROM CRC check fails.
MS58xxLL::MS58xxLL(unsigned int bus, unsigned char addr, const std::string &variant)
    : m_bus(bus),
      m_addr(addr),
      m_resolution(MS58xxCommand::ADC_4096) {
	DEBUG_TRACE("MS58xxLL(bus=%u, addr=0x%02x, variant=%s)", bus, static_cast<unsigned int>(addr), variant.c_str());

	if (variant == "MS5837_02BA") {
		m_convert = read_ms5837_02bar;
		m_crc_offset = 0;
		m_prom_size = 7;
		m_crc_mask = 0x0FFF;
		m_crc_shift = 12;
		C[7] = 0;
	} else if (variant == "MS5837_30BA") {
		m_convert = read_ms5837_30bar;
		m_crc_offset = 0;
		m_prom_size = 7;
		m_crc_mask = 0x0FFF;
		m_crc_shift = 12;
	} else if (variant == "MS5803_14BA") {
		m_convert = read_ms5803_14bar;
		m_prom_size = 8;
		m_crc_offset = 7;
		m_crc_mask = 0xFF00;
		m_crc_shift = 0;
	} else {
		DEBUG_ERROR("MS58xxLL: unsupported variant '%s'", variant.c_str());
		throw ErrorCode::NOT_IMPLEMENTED;
	}

	read_coeffs();
	check_coeffs();
}

/// @brief Read pressure and temperature using the variant-specific conversion function.
/// @param[out] temperature  Temperature in °C.
/// @param[out] pressure     Pressure in bar.
void MS58xxLL::read(double &temperature, double &pressure) {
	auto D1 = static_cast<int32_t>(sample_adc(MS58xxCommand::ADC_D1));
	auto D2 = static_cast<int32_t>(sample_adc(MS58xxCommand::ADC_D2));
	m_convert(D1, D2, C, temperature, pressure);
}

/// @brief Send a single-byte I2C command.
/// @param command  Command byte.
void MS58xxLL::send_command(uint8_t command) {
	NrfI2C::write(m_bus, m_addr, &command, sizeof(command), false);
}

/// @brief Reset device and read PROM calibration coefficients via I2C.
void MS58xxLL::read_coeffs() {
	DEBUG_TRACE("MS58xxLL: reading PROM (%u words)", m_prom_size);

	send_command(static_cast<uint8_t>(MS58xxCommand::RESET));
	PMU::delay_ms(10);

	for (uint32_t i = 0; i < m_prom_size; ++i) {
		uint8_t read_buffer[2];

		send_command(static_cast<uint8_t>(MS58xxCommand::PROM) | (i * 2));
		NrfI2C::read(m_bus, m_addr, read_buffer, 2);
		C[i] = static_cast<uint16_t>(read_buffer[0]) << 8 | read_buffer[1];
	}
}

/// @brief Sample one ADC channel (D1=pressure or D2=temperature).
/// @param measurement  ADC_D1 or ADC_D2.
/// @return 24-bit ADC result.
uint32_t MS58xxLL::sample_adc(uint8_t measurement) {
	uint8_t cmd =
	    static_cast<uint8_t>(MS58xxCommand::ADC_CONV) | static_cast<uint8_t>(MS58xxCommand::ADC_4096) | measurement;
	send_command(cmd);
	PMU::delay_ms(10);  // 4096 OSR conversion time: max 8.22 ms
	send_command(static_cast<uint8_t>(MS58xxCommand::ADC_READ));

	uint8_t buf[3];
	NrfI2C::read(m_bus, m_addr, buf, 3);
	return static_cast<uint32_t>(buf[0]) << 16 | static_cast<uint32_t>(buf[1]) << 8 | buf[2];
}

/// @brief Verify PROM CRC (4-bit CRC per MS58xx datasheet).
/// @throws ErrorCode::I2C_CRC_FAILURE if CRC mismatch.
void MS58xxLL::check_coeffs() {
	uint32_t n_rem = 0;
	uint16_t crc_temp = C[m_crc_offset];

	C[m_crc_offset] &= m_crc_mask;

	for (uint32_t i = 0; i < 16; ++i) {
		if (i % 2 == 1)
			n_rem ^= C[i >> 1] & 0x00FF;
		else
			n_rem ^= C[i >> 1] >> 8;

		for (uint32_t j = 8; j > 0; j--) {
			if (n_rem & 0x8000)
				n_rem = (n_rem << 1) ^ 0x3000;
			else
				n_rem = n_rem << 1;
		}
	}

	n_rem = 0x000F & (n_rem >> 12);
	C[m_crc_offset] = crc_temp;

	uint8_t actual_crc = n_rem ^ 0x00;
	if (actual_crc != static_cast<uint8_t>(crc_temp >> m_crc_shift)) {
		DEBUG_ERROR("MS58xxLL: CRC failure (expected %02X, got %02X)",
		            static_cast<unsigned int>(crc_temp >> m_crc_shift), static_cast<unsigned int>(actual_crc));
		throw ErrorCode::I2C_CRC_FAILURE;
	}
}
