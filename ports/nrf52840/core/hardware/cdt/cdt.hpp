#pragma once

/**
 * @file cdt.hpp
 * @brief CDT (Conductivity-Depth-Temperature) sensor driver.
 *
 * Combines an AD5933 impedance analyser (conductivity) with a pressure
 * sensor (depth + temperature).  The AD5933 injects 90 kHz through water
 * between two electrodes; impedance → conductivity via polynomial calibration.
 *
 * Read channels:
 *   0: Calibrated conductivity (µS/cm)
 *   1: Pressure (hPa) — triggers a new pressure/temperature reading
 *   2: Temperature (°C, cached from last pressure read)
 *
 * Calibration write offsets (SCALW):
 *   0: Reset calibration
 *   2: CA coefficient
 *   3: CB coefficient
 *   4: CC coefficient
 *   5: Save calibration to file
 *   6: Gain factor
 *   7: Start AD5933 at frequency (value = Hz)
 *   8: Stop AD5933
 *
 * Calibration read offsets (SCALR):
 *   0: CA     1: CB     2: CC     3: Gain factor
 *   4: Real IQ component     5: Imaginary IQ component
 *   6: Impedance using calibrated gain
 */

#include "pressure_sensor.hpp"
#include "ad5933.hpp"

class CDT : public Sensor {
public:
	/// @param device  Pressure sensor for depth and temperature readings.
	/// @param ad5933  Impedance analyser for conductivity measurement.
	CDT(PressureSensorDevice &device, AD5933 &ad5933);

	/// @param offset  Channel: 0=conductivity (µS/cm), 1=pressure (hPa), 2=temperature (°C).
	/// @return Sensor value for the requested channel.
	double read(unsigned int offset) override;

	/// @param value   Calibration value to write (interpretation depends on offset).
	/// @param offset  SCALW offset (see table above).
	void calibration_write(const double value, const unsigned int offset) override;

	/// @param[out] value  Read-back value (interpretation depends on offset).
	/// @param offset      SCALR offset (see table above).
	void calibration_read(double &value, const unsigned int offset) override;

	/// @param force  If true, write even if no changes detected.
	void calibration_save(bool force) override;

private:
	Calibration m_cal;               ///< Persistent calibration file (CDT.CAL)
	PressureSensorDevice &m_device;  ///< Pressure sensor for depth + temperature
	AD5933 &m_ad5933;                ///< Impedance analyser for conductivity
	double m_last_temperature = 0;   ///< Cached temperature from last pressure read (°C)
	double m_last_pressure = 0;      ///< Cached pressure from last read (hPa)
	int16_t m_last_imaginary = 0;    ///< Cached imaginary IQ from last SCALR read

	/// @brief Calibration polynomial coefficients.
	/// conductivity = 1000 × (CA × x² + CB × x + CC) where x = (1/impedance) × 1000
	enum class CalibrationPoint : unsigned int {
		GAIN_FACTOR = 0,  ///< AD5933 gain factor (1 / (magnitude × known_impedance))
		CA = 1,           ///< Quadratic coefficient
		CB = 2,           ///< Linear coefficient
		CC = 3            ///< Constant offset
	};

	/// @brief Measure impedance at 90 kHz, convert to conductivity using calibration.
	/// @return Conductivity in µS/cm, or 0 if impedance measurement failed.
	double read_calibrated_conductivity();
};
