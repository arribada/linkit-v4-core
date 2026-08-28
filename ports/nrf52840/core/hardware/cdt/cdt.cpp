/**
 * @file cdt.cpp
 * @brief CDT sensor — conductivity via AD5933 impedance + depth/temperature via pressure sensor.
 */

#include "cdt.hpp"
#include "debug.hpp"

CDT::CDT(PressureSensorDevice &device, AD5933 &ad5933)
    : Sensor("CDT"),
      m_cal(Calibration("CDT")),
      m_device(device),
      m_ad5933(ad5933) {}

/// @brief Read sensor channel: 0=conductivity, 1=pressure (triggers read), 2=temperature (cached).
double CDT::read(unsigned int offset) {
	switch (offset) {
	case 0: return read_calibrated_conductivity();
	case 1: m_device.read(m_last_temperature, m_last_pressure); return m_last_pressure;
	case 2: return m_last_temperature;
	default: return 0;
	}
}

/// @brief Write calibration value or trigger AD5933 action.
void CDT::calibration_write(const double value, const unsigned int offset) {
	switch (offset) {
	case 0:
		DEBUG_TRACE("CDT: reset calibration");
		m_cal.reset();
		break;
	case 2:
		DEBUG_TRACE("CDT: CA = %lf", value);
		m_cal.write(static_cast<unsigned int>(CalibrationPoint::CA), value);
		break;
	case 3:
		DEBUG_TRACE("CDT: CB = %lf", value);
		m_cal.write(static_cast<unsigned int>(CalibrationPoint::CB), value);
		break;
	case 4:
		DEBUG_TRACE("CDT: CC = %lf", value);
		m_cal.write(static_cast<unsigned int>(CalibrationPoint::CC), value);
		break;
	case 5:
		DEBUG_TRACE("CDT: saving calibration");
		m_cal.save();
		break;
	case 6:
		DEBUG_TRACE("CDT: gain_factor = %lf", value);
		m_cal.write(static_cast<unsigned int>(CalibrationPoint::GAIN_FACTOR), value);
		break;
	case 7:
		DEBUG_TRACE("CDT: power on AD5933 f=%u Hz", static_cast<unsigned int>(value));
		m_ad5933.start(static_cast<unsigned int>(value), VRange::V400MV_GAIN1X);
		break;
	case 8:
		DEBUG_TRACE("CDT: power off AD5933");
		m_ad5933.stop();
		break;
	default: DEBUG_WARN("CDT: invalid calibration_write offset %u", offset); break;
	}
}

/// @brief Read calibration value, IQ data, or impedance.
void CDT::calibration_read(double &value, const unsigned int offset) {
	switch (offset) {
	case 0: value = m_cal.read(static_cast<unsigned int>(CalibrationPoint::CA)); break;
	case 1: value = m_cal.read(static_cast<unsigned int>(CalibrationPoint::CB)); break;
	case 2: value = m_cal.read(static_cast<unsigned int>(CalibrationPoint::CC)); break;
	case 3: value = m_cal.read(static_cast<unsigned int>(CalibrationPoint::GAIN_FACTOR)); break;
	case 4: {
		// Read real + imaginary IQ from AD5933 (must be called before offset 5)
		int16_t real, imag;
		m_ad5933.get_real_imaginary(real, imag);
		DEBUG_TRACE("CDT: IQ real=%d imag=%d", static_cast<int>(real), static_cast<int>(imag));
		value = static_cast<double>(real);
		m_last_imaginary = imag;
		break;
	}
	case 5: value = static_cast<double>(m_last_imaginary); break;
	case 6:
		value = m_ad5933.get_impedence(1, m_cal.read(static_cast<unsigned int>(CalibrationPoint::GAIN_FACTOR)));
		break;
	default:
		DEBUG_WARN("CDT: invalid calibration_read offset %u", offset);
		value = 0;
		break;
	}
}

/// @brief Persist calibration to flash.
void CDT::calibration_save(bool force) {
	m_cal.save(force);
}

/// @brief Measure conductivity: start AD5933 at 90 kHz, read impedance, apply polynomial.
double CDT::read_calibrated_conductivity() {
	double gain_factor;
	try {
		gain_factor = m_cal.read(static_cast<unsigned int>(CalibrationPoint::GAIN_FACTOR));
	} catch (...) {
		DEBUG_TRACE("CDT: gain factor missing — using default");
		gain_factor = 10.95E-6;
	}

	double CA, CB, CC;
	try {
		CA = m_cal.read(static_cast<unsigned int>(CalibrationPoint::CA));
		CB = m_cal.read(static_cast<unsigned int>(CalibrationPoint::CB));
		CC = m_cal.read(static_cast<unsigned int>(CalibrationPoint::CC));
	} catch (...) {
		DEBUG_TRACE("CDT: CA/CB/CC missing — using defaults");
		CA = 0.0011;
		CB = 0.9095;
		CC = 0.4696;
	}

	m_ad5933.start(90000, VRange::V400MV_GAIN1X);
	double impedance = m_ad5933.get_impedence(2, gain_factor);
	m_ad5933.stop();

	// Guard against division by zero (AD5933 timeout or disconnected electrodes)
	if (impedance <= 0) {
		DEBUG_WARN("CDT: impedance <= 0 (%.4f) — returning 0", impedance);
		return 0;
	}

	double conduino_value = (1.0 / impedance) * 1000.0;
	double conductivity = 1000.0 * (CA * conduino_value * conduino_value + CB * conduino_value + CC);

	DEBUG_TRACE("CDT: impedance=%.2f conduino=%.2f conductivity=%.2f µS/cm", impedance, conduino_value, conductivity);

	return conductivity;
}
