/**
 * @file ezo_rtd.cpp
 * @brief Atlas Scientific EZO-RTD temperature sensor — I2C ASCII command driver.
 */

#include <cstdint>
#include <cstdlib>
#include <cerrno>
#include <cstring>

#include "nrf_i2c.hpp"
#include "bsp.hpp"
#include "error.hpp"
#include "gpio.hpp"
#include "debug.hpp"
#include "ezo_rtd.hpp"
#include "pmu.hpp"

/// @brief Max command/response buffer size (EZO commands are short ASCII strings).
static constexpr unsigned int EZO_CMD_BUF_SIZE = 32;

/// @brief Max I2C response buffer size (status byte + ASCII data).
static constexpr unsigned int EZO_RESP_BUF_SIZE = 20;

/// @brief Max retries for wait_response() polling (1 ms each).
static constexpr unsigned int EZO_RESPONSE_MAX_RETRIES = 100;


EZO_RTD_Sensor::EZO_RTD_Sensor() : Sensor("RTD"), m_is_calibrating(false) {
	// Wakeup to confirm device is present on I2C bus (throws if not)
	wakeup();
	sleep();
}

/// @brief Wake from sleep by sending two I2C reads (EZO wakeup protocol).
void EZO_RTD_Sensor::wakeup() {
	read_response();
	read_response();
}

/// @brief Enter deep sleep (~1 µA).
void EZO_RTD_Sensor::sleep() {
	write_command("Sleep");
}

/// @brief Read temperature: wakeup → "R" command → 600 ms wait → parse ASCII response.
/// @param offset  Unused (only channel 0).
/// @return Temperature in °C.
/// @throws ErrorCode::I2C_COMMS_ERROR if response is invalid or out of range.
double EZO_RTD_Sensor::read(unsigned int) {
	char response[EZO_RESP_BUF_SIZE];

	wakeup();
	write_command("R");
	PMU::delay_ms(600);  // EZO-RTD ADC conversion time
	wait_response(response);

	if (!m_is_calibrating) sleep();

	// Parse ASCII temperature string
	char *end;
	errno = 0;
	double value = strtod(response, &end);

	// Validate: no parse error and within EZO-RTD range (-126°C to 1254°C)
	if (errno == 0 && response != end && value >= -126 && value <= 1254) return value;

	throw ErrorCode::I2C_COMMS_ERROR;
}

/// @brief Calibration commands via SCALW offsets.
/// @param temperature         Temperature value for single-point calibration (offset 3).
/// @param calibration_offset  0=clear, 1=cal@0°C, 2=cal@100°C, 3=cal@value, 4=find, 5=factory, 6+=exit.
void EZO_RTD_Sensor::calibration_write(const double temperature, const unsigned int calibration_offset) {
	wakeup();
	m_is_calibrating = true;

	switch (calibration_offset) {
	case 0:
		write_command("Cal,clear");
		PMU::delay_ms(300);
		break;
	case 1:
		write_command("Cal,0");
		PMU::delay_ms(600);
		break;
	case 2:
		write_command("Cal,100");
		PMU::delay_ms(600);
		break;
	case 3:
		if (temperature >= -126 && temperature <= 1254) {
			char command[EZO_CMD_BUF_SIZE];
			snprintf(command, sizeof(command), "Cal,%f", temperature);
			write_command(command);
			PMU::delay_ms(600);
		} else {
			m_is_calibrating = false;
			throw ErrorCode::RESOURCE_NOT_AVAILABLE;
		}
		break;
	case 4:
		write_command("Find");
		PMU::delay_ms(300);
		break;
	case 5:
		write_command("Factory");
		m_is_calibrating = false;
		return;
	default:
		// offset >= 6: exit calibration mode
		sleep();
		m_is_calibrating = false;
		return;
	}

	wait_response();
}

/// @brief Send ASCII command via I2C.
/// @param command  Null-terminated command string.
/// @note Data is copied to a RAM buffer because the nRF TWIM DMA cannot read from flash.
void EZO_RTD_Sensor::write_command(const char *command) {
	char data[EZO_CMD_BUF_SIZE];
	strncpy(data, command, sizeof(data) - 1);
	data[sizeof(data) - 1] = '\0';
	DEBUG_TRACE("EZO_RTD: cmd=%s", command);
	NrfI2C::write(EZO_RTD_DEVICE, EZO_RTD_DEVICE_ADDR, reinterpret_cast<const uint8_t *>(data), strlen(data), false);
}

/// @brief Convert response code enum to printable string.
/// @param resp  Response code.
/// @return Static string.
const char *EZO_RTD_Sensor::response_code_to_str(ResponseCode resp) {
	switch (resp) {
	case ResponseCode::SUCCESS: return "SUCCESS";
	case ResponseCode::ERROR: return "ERROR";
	case ResponseCode::BUSY: return "BUSY";
	case ResponseCode::NODATA: return "NODATA";
	default: return "UNKNOWN";
	}
}

/// @brief Read I2C response: first byte = status code, remaining bytes = ASCII data.
/// @param[out] response  Buffer for ASCII data (min EZO_RESP_BUF_SIZE), or nullptr to ignore data.
/// @return EZO response code.
EZO_RTD_Sensor::ResponseCode EZO_RTD_Sensor::read_response(char *response) {
	uint8_t bytes[EZO_RESP_BUF_SIZE] = {};

	NrfI2C::read(EZO_RTD_DEVICE, EZO_RTD_DEVICE_ADDR, bytes, sizeof(bytes));

	ResponseCode resp;
	switch (bytes[0]) {
	case 0x01: resp = ResponseCode::SUCCESS; break;
	case 0x02: resp = ResponseCode::ERROR; break;
	case 0xFE: resp = ResponseCode::BUSY; break;
	case 0xFF: resp = ResponseCode::NODATA; break;
	default: resp = ResponseCode::UNKNOWN; break;
	}

	if (resp == ResponseCode::SUCCESS && response != nullptr) {
		unsigned int i;
		for (i = 1; i < sizeof(bytes) && bytes[i]; i++)
			response[i - 1] = static_cast<char>(bytes[i]);
		response[i - 1] = '\0';
		DEBUG_TRACE("EZO_RTD: resp=%s data=%s", response_code_to_str(resp), response);
	} else {
		DEBUG_TRACE("EZO_RTD: resp=%s", response_code_to_str(resp));
	}

	return resp;
}

/// @brief Poll for SUCCESS with bounded timeout.
/// @param[out] response  Buffer for response data, or nullptr.
/// @throws ErrorCode::I2C_COMMS_ERROR if no SUCCESS after EZO_RESPONSE_MAX_RETRIES attempts.
void EZO_RTD_Sensor::wait_response(char *response) {
	for (unsigned int retries = 0; retries < EZO_RESPONSE_MAX_RETRIES; retries++) {
		if (read_response(response) == ResponseCode::SUCCESS) return;
		PMU::delay_ms(1);
	}

	DEBUG_WARN("EZO_RTD: wait_response timeout (%u ms)", EZO_RESPONSE_MAX_RETRIES);
	throw ErrorCode::I2C_COMMS_ERROR;
}
