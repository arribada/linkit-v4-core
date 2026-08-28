/**
 * @file thermistor.cpp
 * @brief NTC thermistor sensor — SAADC reading + Beta equation conversion.
 */

#include <cstdint>
#include <cmath>

#include "thermistor.hpp"
#include "nrf_peripheral_power.hpp"
#include "nrfx_saadc.h"
#include "bsp.hpp"
#include "error.hpp"
#include "debug.hpp"
#include "nrf_delay.h"
#include "gpio.hpp"

/// @name ADC constants
/// @{
static constexpr int ADC_MAX_VALUE = 16384;   ///< 2^14 (14-bit resolution)
static constexpr float ADC_REFERENCE = 0.6f;  ///< Internal reference voltage (V)
/// @}

static void nrfx_saadc_event_handler(nrfx_saadc_evt_t const *p_event) {
	(void)p_event;
}

Thermistor::Thermistor(uint8_t adc_channel) : Sensor("THERMISTOR"), m_cal(Calibration("THERMISTOR")) {
	DEBUG_INFO("THERMISTOR::Init: ADC Channel %u", adc_channel);
	adc_calibration();
	try {
		m_offset_temp = m_cal.read(static_cast<unsigned int>(CalibrationPoint::TEMP_THRESHOLD));
		DEBUG_TRACE("THERMISTOR::Init: Reading calibration offset %lf", m_offset_temp);
	} catch (...) {
		DEBUG_WARN("THERMISTOR::Init: Failed to read calibration offset");
		m_offset_temp = 0.0;
		m_cal.write(static_cast<unsigned int>(CalibrationPoint::TEMP_THRESHOLD), 0.0);
		m_cal.save();
	}
	m_adc_channel = adc_channel;
}

/// @brief Run SAADC offset calibration with 200 ms timeout.
void Thermistor::adc_calibration() {
	DEBUG_TRACE("THERMISTOR::%s: Calibrating ADC...", __func__);
	nrfx_saadc_init(&BSP::ADC_Inits.config, nrfx_saadc_event_handler);
	nrfx_saadc_calibrate_offset();

	const uint32_t timeout_ms = 200;
	uint32_t elapsed = 0;
	while (nrfx_saadc_is_busy()) {
		nrf_delay_ms(1);
		if (++elapsed >= timeout_ms) {
			DEBUG_WARN("THERMISTOR::%s: ADC calibration timeout!", __func__);
			break;
		}
	}

	nrfx_saadc_uninit();
	nrf_peripheral_power_reset(NRF_SAADC_BASE_ADDR);  // Errata 241: prevent 400 µA idle leak
	m_is_init = true;
	DEBUG_INFO("THERMISTOR::%s: Calibration done!", __func__);
}

/// @brief Sample SAADC and return voltage in millivolts.
/// @return ADC reading in mV.
float Thermistor::sample_adc() {
	if (!m_is_init) {
		adc_calibration();
	}

	SensorsPowerGuard power_guard;  // Acquire VSENSORS power for ADC reading

	nrf_saadc_value_t raw = 0;

	nrfx_saadc_init(&BSP::ADC_Inits.config, nrfx_saadc_event_handler);
	nrfx_saadc_channel_init(m_adc_channel, &BSP::ADC_Inits.channel_config[m_adc_channel]);
	nrfx_saadc_sample_convert(m_adc_channel, &raw);
	nrfx_saadc_uninit();
	nrf_peripheral_power_reset(NRF_SAADC_BASE_ADDR);  // Errata 241: prevent 400 µA idle leak

	float adc_voltage = (static_cast<float>(raw) / static_cast<float>(ADC_MAX_VALUE))
	                    * static_cast<float>(ADC_REFERENCE) * (1.0f / static_cast<float>(ADC_GAIN));

	return adc_voltage * 1000.0f;  // Return millivolts
}

/// @brief Convert ADC millivolts to temperature using Beta equation with range-dependent B-values.
/// @param adc  ADC reading in millivolts.
/// @return Temperature in °C (with calibration offset), or NAN on invalid reading.
double Thermistor::convert_temp(float adc) {
	// Convert ADC value (mV) to Voltage (V)
	double v = static_cast<double>(adc) / 1000.0;

	// Guard against invalid ADC readings
	if (v < 0.001) {
		DEBUG_ERROR("Thermistor: ADC voltage near 0");
		return NAN;
	}
	if (v > 3.3) v = 3.3;

	// Voltage divider: NTC thermistor on top, 10k fixed resistor on bottom
	constexpr double R_BOTTOM = 10000.0;  // 10k fixed resistor
	double r_therm = R_BOTTOM * ((3.3 / v) - 1.0);

	// NTC thermistor properties
	constexpr double R0 = 10000.0;  // Resistance at 25C
	constexpr double T0 = 298.15;   // 25C in Kelvin

	// Dynamically select B-value based on resistance range
	double B = 3434.0;
	if (r_therm > 12000.0) {
		B = 3380.0;  // Cold (<25C)
	} else if (r_therm > 8000.0) {
		B = 3434.0;  // Warm (25-50C)
	} else {
		B = 3455.0;  // Hot (>50C)
	}

	// Beta equation: 1/T = 1/T0 + (1/B) * ln(R_therm / R0)
	double tempK = 1.0 / ((1.0 / T0) + (1.0 / B) * log(r_therm / R0));
	double tempC = tempK - 273.15;

	tempC = tempC + m_offset_temp;

	// Reject physically implausible results. A clamped v≈3.3 V (ADC glitch /
	// disconnected divider) degenerates the Beta math to ≈ -273 °C, and other
	// glitches can produce wild values that would feed mortality/logging. A bird
	// tracker realistically sees ≈ -50..+80 °C, so treat anything outside
	// [-55, 125] °C as a bad reading → read() converts NAN into a skipped sample.
	if (tempC < -55.0 || tempC > 125.0) {
		DEBUG_ERROR("Thermistor: implausible temp %.1f C (v=%.3f) — rejecting", tempC, v);
		return NAN;
	}
	return tempC;
}

/// @brief Read temperature in °C (sample ADC + convert + apply offset).
/// @param offset  Unused (only channel 0).
/// @return Temperature in °C.
/// @throws ErrorCode::I2C_COMMS_ERROR if ADC reading is invalid.
double Thermistor::read(unsigned int offset) {
	(void)offset;
	float adc = sample_adc();
	double temperature = convert_temp(adc);
	if (std::isnan(temperature)) {
		throw ErrorCode::I2C_COMMS_ERROR;
	}
	DEBUG_INFO("THERMISTOR::%s: TempC = %.5lf", __func__, temperature);
	m_last_temperature = temperature;
	return temperature;
}

/// @brief Average 10 samples at a known temperature to compute calibration offset.
/// @param target_temp_c  Known reference temperature in °C.
/// @return Calibration offset = target - measured (°C).
double Thermistor::find_calibration_point(double target_temp_c) {
	// target_temp_c is in degrees Celsius (e.g., 24.0 = 24.0°C)
	double target_c = target_temp_c;

	// Temporarily disable offset to measure raw temperature
	double saved_offset = m_offset_temp;
	m_offset_temp = 0.0;

	constexpr unsigned int num_samples = 10;
	double total_temperature = 0.0;

	for (unsigned int i = 0; i < num_samples; ++i) {
		total_temperature += read(0);
		nrf_delay_ms(100);
	}

	double average_raw = total_temperature / num_samples;

	// offset = target - measured (added to raw readings to correct)
	double calibration_offset = target_c - average_raw;

	DEBUG_INFO("THERMISTOR::calibrate: target=%.3f C | measured=%.3f C | offset=%.3f C", target_c, average_raw,
	           calibration_offset);

	// Restore offset (will be replaced when calibration is applied)
	m_offset_temp = saved_offset;

	return calibration_offset;
}

/// @brief Calibration commands: 0=reset offset, 1=calibrate at known temperature.
/// @param value   Target temperature for offset=1, unused for offset=0.
/// @param offset  0=reset, 1=calibrate.
void Thermistor::calibration_write(const double value, const unsigned int offset) {
	if (offset == 0) {
		// Reset calibration
		m_offset_temp = 0.0;
		m_cal.write(static_cast<unsigned int>(CalibrationPoint::TEMP_THRESHOLD), 0.0);
		m_cal.save();
		DEBUG_INFO("THERMISTOR: Calibration reset");
	} else if (offset == 1) {
		// Calibrate: value is known temperature in degrees Celsius (e.g., 24.0 = 24.0°C)
		double calibration_value = find_calibration_point(value);
		m_offset_temp = calibration_value;
		m_cal.write(static_cast<unsigned int>(CalibrationPoint::TEMP_THRESHOLD), calibration_value);
		m_cal.save();
		DEBUG_INFO("THERMISTOR: Calibration saved offset=%.3f C", calibration_value);
	}
}

/// @brief Persist calibration offset to flash.
void Thermistor::calibration_save(bool force) {
	m_cal.save(force);
}

/// @brief Read calibration offset (offset=0) or last temperature (offset=1).
/// @param[out] value  Read-back value.
/// @param offset      0=calibration offset, 1=last temperature.
void Thermistor::calibration_read(double &value, const unsigned int offset) {
	if (0 == offset) {
		try {
			value = m_cal.read(static_cast<unsigned int>(CalibrationPoint::TEMP_THRESHOLD));
		} catch (...) {
			value = 0.0;
		}
	} else if (1 == offset) {
		// Read current temperature (live sensor reading)
		value = m_last_temperature;
	} else {
		value = 0.0;
	}
}
