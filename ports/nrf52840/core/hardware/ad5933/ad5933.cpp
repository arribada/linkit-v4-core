/**
 * @file ad5933.cpp
 * @brief AD5933 impedance network analyser — I2C low-level driver.
 */

#include <cmath>

#include "bsp.hpp"
#include "ad5933.hpp"
#include "nrf_i2c.hpp"
#include "error.hpp"
#include "pmu.hpp"
#include "debug.hpp"

/// @brief Max settling time register value (9-bit + 2-bit multiplier).
static constexpr unsigned int MAX_SETTLING = 2047;

/// @brief IQ data-ready polling timeout (ms).  Worst-case sweep with 128 settling cycles.
static constexpr unsigned int IQ_TIMEOUT_MS = 500;


AD5933LL::AD5933LL(unsigned int bus, unsigned char addr) : m_bus(bus), m_addr(addr), m_gain_setting(0) {
	DEBUG_TRACE("AD5933LL(%u, 0x%02x)", bus, static_cast<unsigned int>(addr));
	powerdown();
}

uint8_t AD5933LL::read_reg(Reg reg) {
	uint8_t r = static_cast<uint8_t>(reg);
	uint8_t value;
	NrfI2C::write(m_bus, m_addr, &r, sizeof(r), false);
	NrfI2C::read(m_bus, m_addr, &value, sizeof(value));
	return value;
}

void AD5933LL::write_reg(Reg reg, uint8_t value) {
	uint8_t buffer[2] = { static_cast<uint8_t>(reg), value };
	NrfI2C::write(m_bus, m_addr, buffer, sizeof(buffer), false);
}

/// @brief Configure and start an impedance measurement at the given frequency.
/// @param frequency  Excitation frequency in Hz.
/// @param vrange     Output voltage range and PGA gain.
void AD5933LL::start(unsigned int frequency, VRange vrange) {
	DEBUG_TRACE("AD5933LL::start");
	reset();
	gain(vrange);
	set_settling_times(128);
	set_start_frequency(frequency);
	set_frequency_increment(1);
	set_number_of_increments(0);
	standby();
	initialize();
	PMU::delay_ms(10);
	startsweep();
}

void AD5933LL::stop() {
	DEBUG_TRACE("AD5933LL::stop");
	powerdown();
}

/**
 * @brief Measure impedance by averaging multiple IQ readings.
 * @param averaging  Number of readings to average.
 * @param gain       Calibration gain factor (1 / (magnitude × gain) = impedance).
 * @return Averaged impedance in ohms, or 0 if data not ready.
 */
double AD5933LL::get_impedence(unsigned int averaging, double gain) {
	DEBUG_TRACE("AD5933LL::get_impedence");
	double impedence = 0;
	for (unsigned int i = 0; i < averaging; i++) {
		if (!wait_iq_data_ready()) {
			DEBUG_WARN("AD5933: IQ data timeout during averaging (sample %u/%u)", i, averaging);
			return 0;
		}
		double magnitude =
		    std::sqrt(std::pow(static_cast<double>(read_real()), 2) + std::pow(static_cast<double>(read_imag()), 2));
		if (magnitude > 0) impedence += 1.0 / (magnitude * gain);
	}
	impedence /= averaging;

	DEBUG_TRACE("AD5933LL::get_impedence() = %lf", impedence);
	return impedence;
}

/// @brief Read raw real and imaginary DFT components.
void AD5933LL::get_real_imaginary(int16_t &real, int16_t &imag) {
	DEBUG_TRACE("AD5933LL::get_real_imaginary");
	if (!wait_iq_data_ready()) {
		DEBUG_WARN("AD5933: IQ data timeout in get_real_imaginary");
		real = 0;
		imag = 0;
		return;
	}
	real = read_real();
	imag = read_imag();
	DEBUG_TRACE("AD5933LL::get_real_imaginary() = %d|%d", static_cast<int>(real), static_cast<int>(imag));
#ifdef DEBUG_AD5933
	dump_regs();
#endif
}

/// @brief Set the excitation start frequency register (assumes 16.776 MHz oscillator).
void AD5933LL::set_start_frequency(unsigned int v) {
	uint64_t code = ((static_cast<uint64_t>(v) << 27) + 2097000) / 4194000;
	DEBUG_TRACE("AD5933LL::set_start_frequency: code=%08x", static_cast<unsigned int>(code));
	write_reg(Reg::START_FREQ_23_16, code >> 16);
	write_reg(Reg::START_FREQ_15_8, code >> 8);
	write_reg(Reg::START_FREQ_7_0, code);
}

void AD5933LL::set_number_of_increments(unsigned int num) {
	write_reg(Reg::NUM_INC_15_8, num >> 8);
	write_reg(Reg::NUM_INC_7_0, num);
}

/// @brief Set frequency increment register (assumes 16.776 MHz oscillator).
void AD5933LL::set_frequency_increment(unsigned int v) {
	uint64_t code = ((static_cast<uint64_t>(v) << 27) + 2097000) / 4194000;
	write_reg(Reg::FREQ_INC_23_16, code >> 16);
	write_reg(Reg::FREQ_INC_15_8, code >> 8);
	write_reg(Reg::FREQ_INC_7_0, code);
}

/// @brief Set settling time cycles with automatic multiplier (1×/2×/4×).
void AD5933LL::set_settling_times(unsigned int settling) {
	uint8_t decode = 0;
	if (settling > MAX_SETTLING) settling = MAX_SETTLING;

	if (settling > 1023) {
		decode = 3;  // 4× multiplier
		settling >>= 2;
	} else if (settling > 511) {
		decode = 1;  // 2× multiplier
		settling >>= 1;
	}

	write_reg(Reg::SETTLING_15_8, (settling >> 8) | (decode << 1));
	write_reg(Reg::SETTLING_7_0, settling);
}

void AD5933LL::initialize() {
	write_reg(Reg::CONTROL_HIGH, static_cast<uint8_t>(CtrlHigh::INIT_START_FREQ) | m_gain_setting);
}

void AD5933LL::reset() {
	write_reg(Reg::CONTROL_LOW, static_cast<uint8_t>(CtrlLow::RESET));
}

void AD5933LL::standby() {
	write_reg(Reg::CONTROL_HIGH, static_cast<uint8_t>(CtrlHigh::STANDBY) | m_gain_setting);
}

/// @brief Configure output voltage range and PGA gain.
void AD5933LL::gain(VRange setting) {
	switch (setting) {
	case VRange::V1_GAIN1X:
		m_gain_setting = static_cast<uint8_t>(CtrlHigh::PGA_GAIN_1X) | static_cast<uint8_t>(CtrlHigh::OPV_1V);
		break;
	case VRange::V2_GAIN1X:
		m_gain_setting = static_cast<uint8_t>(CtrlHigh::PGA_GAIN_1X) | static_cast<uint8_t>(CtrlHigh::OPV_2V);
		break;
	case VRange::V200MV_GAIN1X:
		m_gain_setting = static_cast<uint8_t>(CtrlHigh::PGA_GAIN_1X) | static_cast<uint8_t>(CtrlHigh::OPV_200MV);
		break;
	case VRange::V400MV_GAIN1X:
		m_gain_setting = static_cast<uint8_t>(CtrlHigh::PGA_GAIN_1X) | static_cast<uint8_t>(CtrlHigh::OPV_400MV);
		break;
	case VRange::V1_GAIN0_5X: m_gain_setting = static_cast<uint8_t>(CtrlHigh::OPV_1V); break;
	case VRange::V2_GAIN0_5X: m_gain_setting = static_cast<uint8_t>(CtrlHigh::OPV_2V); break;
	case VRange::V200MV_GAIN0_5X: m_gain_setting = static_cast<uint8_t>(CtrlHigh::OPV_200MV); break;
	case VRange::V400MV_GAIN0_5X: m_gain_setting = static_cast<uint8_t>(CtrlHigh::OPV_400MV); break;
	default: m_gain_setting = 0; break;
	}

	write_reg(Reg::CONTROL_HIGH, m_gain_setting);
}

void AD5933LL::startsweep() {
	write_reg(Reg::CONTROL_HIGH, static_cast<uint8_t>(CtrlHigh::START_SWEEP) | m_gain_setting);
}

uint8_t AD5933LL::status() {
	return read_reg(Reg::STATUS);
}

int16_t AD5933LL::read_real() {
	uint8_t low = read_reg(Reg::REAL_7_0);
	uint8_t high = read_reg(Reg::REAL_15_8);
	return static_cast<int16_t>(static_cast<int>(high) << 8 | low);
}

int16_t AD5933LL::read_imag() {
	uint8_t low = read_reg(Reg::IMAG_7_0);
	uint8_t high = read_reg(Reg::IMAG_15_8);
	return static_cast<int16_t>(static_cast<int>(high) << 8 | low);
}

/// @brief Wait for IQ data to become valid in the status register.
/// @return true if data ready, false on timeout.
bool AD5933LL::wait_iq_data_ready() {
	for (unsigned int ms = 0; ms < IQ_TIMEOUT_MS; ms++) {
		if (status() & static_cast<uint8_t>(Status::VALID_IQ)) return true;
		PMU::delay_ms(1);
	}
	DEBUG_WARN("AD5933: wait_iq_data_ready timeout (%u ms)", IQ_TIMEOUT_MS);
	return false;
}

void AD5933LL::powerdown() {
	write_reg(Reg::CONTROL_HIGH, static_cast<uint8_t>(CtrlHigh::POWER_DOWN));
}

void AD5933LL::dump_regs() {
	for (unsigned int i = 0x80; i < 0x98; i++) {
		[[maybe_unused]] uint8_t value = read_reg(static_cast<Reg>(i));
		DEBUG_TRACE("reg[%02x]=%02x", i, static_cast<unsigned int>(value));
	}
}
