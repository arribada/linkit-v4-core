#pragma once

/**
 * @file thermistor.hpp
 * @brief NTC thermistor temperature sensor driver (nRF SAADC).
 *
 * Reads a 10K NTC thermistor via the nRF52840 SAADC through a voltage divider
 * (NTC on top, 10K fixed on bottom).  Temperature is computed using the Beta
 * equation with range-dependent B-values for improved accuracy.
 *
 * Read channels:
 *   0: Temperature in °C (with calibration offset applied)
 *
 * Calibration write offsets (SCALW):
 *   0: Reset calibration offset to 0
 *   1: Calibrate at known temperature (value = target °C, averages 10 samples)
 *
 * Calibration read offsets (SCALR):
 *   0: Stored calibration offset (°C)
 *   1: Last measured temperature (°C, cached)
 */

#include <cstdint>
#include "sensor.hpp"

class Thermistor : public Sensor {
public:
	/// @param adc_channel  BSP ADC channel index for the thermistor.
	/// @throws ErrorCode if ADC calibration fails.
	Thermistor(uint8_t adc_channel);

	/// @param offset  Channel (only 0 = temperature).
	/// @return Temperature in °C.
	/// @throws ErrorCode::I2C_COMMS_ERROR if ADC reading is invalid (misleading name, it's an ADC error).
	double read(unsigned int offset) override;

	/// @param value   Target temperature for calibration (offset=1), unused for offset=0.
	/// @param offset  0=reset, 1=calibrate at value.
	void calibration_write(const double value, const unsigned int offset) override;

	/// @param force  If true, write even if no changes detected.
	void calibration_save(bool force) override;

	/// @param[out] value  0=calibration offset, 1=last temperature.
	/// @param offset      SCALR offset.
	void calibration_read(double &value, const unsigned int offset) override;

private:
	Calibration m_cal;                ///< Persistent calibration file (THERMISTOR.CAL)
	uint8_t m_adc_channel;            ///< SAADC channel index
	bool m_is_init = false;           ///< True after ADC calibration
	double m_last_temperature = 0.0;  ///< Cached last reading (°C)
	double m_offset_temp = 0.0;       ///< Calibration offset (°C, added to raw reading)

	/// @brief Run SAADC offset calibration (200 ms timeout).
	void adc_calibration();

	/// @brief Sample ADC and return voltage in millivolts.
	/// @return ADC reading in mV.
	float sample_adc();

	/// @brief Convert ADC millivolts to temperature using Beta equation.
	/// @param adc_mv  ADC reading in millivolts.
	/// @return Temperature in °C (with offset applied), or NAN on error.
	double convert_temp(float adc_mv);

	/// @brief Average 10 samples to compute calibration offset for a known temperature.
	/// @param target_temp_c  Known reference temperature in °C.
	/// @return Calibration offset = target - measured (°C).
	double find_calibration_point(double target_temp_c);

	/// @brief Calibration point indices.
	enum class CalibrationPoint : unsigned int {
		TEMP_THRESHOLD = 0  ///< Temperature offset in °C
	};
};
