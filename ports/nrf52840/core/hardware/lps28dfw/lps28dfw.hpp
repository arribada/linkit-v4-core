#pragma once

/**
 * @file lps28dfw.hpp
 * @brief LPS28DFW absolute pressure sensor driver (I2C, ST Micro).
 *
 * The LPS28DFW is a high-precision barometric pressure sensor with two
 * full-scale ranges: 1260 hPa (surface, higher precision) and 4060 hPa
 * (underwater, wider range).  Uses one-shot mode with 32-sample averaging.
 *
 * Registers are volatile — re-applied from m_mode on every read() since
 * VSENSORS power may have been cycled between reads.
 *
 * Used on RSPB board as the default pressure sensor (preferred over MS58xx).
 */

#include <cstdint>
#include "pressure_sensor.hpp"
#include "lps28dfw_reg.h"

class LPS28DFW : public PressureSensorDevice {
public:
	/// @param bus      I2C bus index.
	/// @param address  7-bit I2C address (default 0x5C).
	/// @throws ErrorCode::I2C_COMMS_ERROR if WHOAMI check fails.
	LPS28DFW(unsigned int bus, unsigned char address = 0x5C);

	/// @brief Probe device and configure one-shot mode with 32-sample averaging.
	/// @return true on success, false if WHOAMI mismatch or I2C error.
	[[nodiscard]] bool init();

	/// @brief Returns true if init() completed successfully.
	[[nodiscard]] bool is_initialized() const { return m_initialized; }

	/// @brief Trigger one-shot measurement, poll DRDY, read pressure and temperature.
	/// @param[out] temperature  Temperature in degrees Celsius.
	/// @param[out] pressure     Pressure in bar (converted from hPa).
	/// @throws ErrorCode::I2C_COMMS_ERROR if trigger or data read fails.
	void read(double &temperature, double &pressure) override;

	/// @brief Select full-scale range.
	/// @param mode  0 = 1260 hPa (surface, better precision), 1 = 4060 hPa (underwater).
	void set_full_scale(unsigned int mode) override;

private:
	unsigned int m_bus;
	unsigned char m_addr;
	bool m_initialized = false;

	stmdev_ctx_t m_ctx;    ///< ST driver context (I2C read/write function pointers)
	lps28dfw_md_t m_mode;  ///< Cached mode config (re-applied on every read after power cycle)

	/// @name I2C platform callbacks for ST driver
	/// @{
	/// @param handle  Pointer to LPS28DFW instance (this).
	/// @param reg     Register address.
	/// @param bufp    Data buffer.
	/// @param len     Number of bytes.
	/// @return 0 on success, -1 on I2C error.
	static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
	static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
	/// @}
};
