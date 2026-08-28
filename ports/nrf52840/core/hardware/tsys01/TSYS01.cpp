/**
 * @file TSYS01.cpp
 * @brief TSYS01 driver.
 */

#include <cmath>

#include "TSYS01.hpp"
#include "nrf_i2c.hpp"
#include "debug.hpp"
#include "pmu.hpp"
#include "bsp.hpp"

#define TSYS01_ADDR          0x77
#define TSYS01_RESET         0x1E
#define TSYS01_ADC_READ      0x00
#define TSYS01_ADC_TEMP_CONV 0x48
#define TSYS01_PROM_READ     0XA0

TSYS01::TSYS01() : Sensor("TSYS01") {
	init();
}

/// @brief Reset device and read 8 PROM calibration coefficients.
/// @return true on success (I2C errors propagate as exceptions from NrfI2C).
bool TSYS01::init() {
	write_command(TSYS01_RESET);
	PMU::delay_ms(10);

	uint8_t received_bytes[2] = {};
	for (uint8_t i = 0; i < 8; i++) {
		write_command(TSYS01_PROM_READ + i * 2);
		NrfI2C::read(TSYS01_DEVICE, TSYS01_ADDR, received_bytes, 2);
		C[i] = static_cast<uint16_t>(received_bytes[0]) << 8 | received_bytes[1];
	}

	// Sanity check: at least one coefficient should be non-zero
	bool valid = false;
	for (uint8_t i = 1; i <= 5; i++) {
		if (C[i] != 0) {
			valid = true;
			break;
		}
	}
	if (!valid) {
		DEBUG_ERROR("TSYS01: PROM coefficients all zero — device not responding correctly");
		return false;
	}

	return true;
}

/// @brief Trigger ADC conversion, read 24-bit result, calculate temperature.
/// @param port  Unused (only channel 0).
/// @return Temperature in °C.
double TSYS01::read(unsigned int port) {
	write_command(TSYS01_ADC_TEMP_CONV);

	PMU::delay_ms(10);  // Max conversion time per datasheet

	write_command(TSYS01_ADC_READ);

	uint8_t received_bytes[3] = { 0 };
	NrfI2C::read(TSYS01_DEVICE, TSYS01_ADDR, received_bytes, 3);
	D1 = static_cast<uint32_t>(received_bytes[0]) << 16 | static_cast<uint32_t>(received_bytes[1]) << 8
	     | received_bytes[2];

	calculate();
	DEBUG_TRACE("TSYS01: temp=%.3f °C", static_cast<double>(TEMP));

	return static_cast<double>(TEMP);
}

/// @brief Load datasheet test case coefficients for calculation verification.
void TSYS01::readTestCase() {
	C[0] = 0;
	C[1] = 28446;  //0xA2 K4
	C[2] = 24926;  //0XA4 k3
	C[3] = 36016;  //0XA6 K2
	C[4] = 32791;  //0XA8 K1
	C[5] = 40781;  //0XAA K0
	C[6] = 0;
	C[7] = 0;

	D1 = 9378708.0f;

	adc = D1 / 256;

	calculate();
}

/// @brief Compute temperature from ADC value and PROM coefficients (4th-order polynomial).
void TSYS01::calculate() {
	adc = D1 / 256.0;

	TEMP = (-2) * float(C[1]) / 1000000000000000000000.0f * float(pow(adc, 4))
	       + 4 * float(C[2]) / 10000000000000000.0f * float(pow(adc, 3))
	       + (-2) * float(C[3]) / 100000000000.0f * float(pow(adc, 2)) + 1 * float(C[4]) / 1000000.0f * float(adc)
	       + (-1.5f) * float(C[5]) / 100;
}

/// @brief Return last computed temperature in °C.
/// @return Temperature in °C.
float TSYS01::temperature() {
	return TEMP;
}

/// @brief Send a single-byte I2C command to the TSYS01.
/// @param command  Command byte (reset, ADC convert, PROM read, etc.).
void TSYS01::write_command(uint8_t command) {
	NrfI2C::write(TSYS01_DEVICE, TSYS01_ADDR, &command, 1, false);
}