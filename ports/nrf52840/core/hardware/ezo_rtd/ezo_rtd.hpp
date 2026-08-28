#pragma once

/**
 * @file ezo_rtd.hpp
 * @brief Atlas Scientific EZO-RTD temperature sensor driver (I2C).
 *
 * The EZO-RTD is a precision RTD (Resistance Temperature Detector) circuit
 * for PT-100/PT-1000 probes.  Communication is via I2C with ASCII commands.
 * The device has a sleep mode (~1 µA) and requires a wakeup sequence.
 *
 * Read channels:
 *   0: Temperature in °C (triggers wakeup → read → sleep, 600 ms blocking)
 *
 * Calibration write offsets (SCALW):
 *   0: Clear calibration
 *   1: Calibrate at 0°C
 *   2: Calibrate at 100°C
 *   3: Calibrate at arbitrary temperature (value = °C)
 *   4: LED Find mode (blink LED)
 *   5: Factory reset
 *   6+: Exit calibration (sleep)
 */

#include <cstdint>
#include "sensor.hpp"

class EZO_RTD_Sensor : public Sensor {
public:
	/// @brief I2C response codes from the EZO circuit.
	enum class ResponseCode {
		SUCCESS,  ///< 0x01 — command executed successfully
		ERROR,    ///< 0x02 — command failed
		BUSY,     ///< 0xFE — processing, try again
		NODATA,   ///< 0xFF — no data available
		UNKNOWN
	};

	/// @throws ErrorCode::I2C_COMMS_ERROR if device not detected on I2C bus.
	EZO_RTD_Sensor();

	/// @brief Read temperature in °C (600 ms blocking for ADC conversion).
	/// @param offset  Unused (only channel 0).
	/// @return Temperature in °C (-126 to 1254 range).
	/// @throws ErrorCode::I2C_COMMS_ERROR if reading is invalid or out of range.
	double read(unsigned int offset) override;

	/// @brief Calibration commands — see SCALW offset table above.
	/// @param calibration_data    Temperature value for offset 3, ignored otherwise.
	/// @param calibration_offset  SCALW offset (0-6+).
	void calibration_write(const double calibration_data, const unsigned int calibration_offset) override;

private:
	bool m_is_calibrating;  ///< True during calibration — prevents sleep after read

	/// @brief Send two I2C reads to wake the EZO from sleep mode.
	void wakeup();

	/// @brief Send the "Sleep" command to enter low-power mode (~1 µA).
	void sleep();

	/// @brief Send an ASCII command string via I2C.
	/// @param command  Null-terminated command (e.g. "R", "Cal,100", "Sleep").
	void write_command(const char *command);

	/// @brief Read I2C response and optionally extract ASCII data.
	/// @param[out] response  Buffer for response data (null-terminated, min 20 bytes), or nullptr.
	/// @return Response code from the EZO circuit.
	ResponseCode read_response(char *response = nullptr);

	/// @brief Poll for SUCCESS response with timeout (100 ms).
	/// @param[out] response  Buffer for response data, or nullptr.
	/// @throws ErrorCode::I2C_COMMS_ERROR on timeout.
	void wait_response(char *response = nullptr);

	/// @brief Convert response code to printable string for debug logs.
	/// @param resp  Response code.
	/// @return Static string (e.g. "SUCCESS", "BUSY").
	static const char *response_code_to_str(ResponseCode resp);
};
