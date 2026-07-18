/**
 * @file lps28dfw.cpp
 * @brief LPS28DFW pressure sensor — I2C wrapper over ST driver library.
 */

#include <cstring>
#include "lps28dfw.hpp"
#include "nrf_i2c.hpp"
#include "pmu.hpp"
#include "debug.hpp"
#include "gpio.hpp"

/// @brief DRDY polling timeout in ms (32-avg one-shot ≈ 20 ms typical).
static constexpr unsigned int DRDY_TIMEOUT_MS = 50;

/// @brief Max I2C write buffer (register address + data).
static constexpr size_t MAX_WRITE_BUF = 32;


LPS28DFW::LPS28DFW(unsigned int bus, unsigned char address)
	: m_bus(bus), m_addr(address)
{
	m_ctx.write_reg = platform_write;
	m_ctx.read_reg = platform_read;
	m_ctx.handle = this;

	m_initialized = init();
	if (!m_initialized) {
		DEBUG_ERROR("LPS28DFW: init failed (bus=%u addr=0x%02X)", bus, address);
		throw ErrorCode::I2C_COMMS_ERROR;
	}
}

/// @brief Probe WHOAMI, configure one-shot + 32-avg + 1260 hPa default.
/// @return true on success.
bool LPS28DFW::init()
{
	SensorsPowerGuard power_guard;
	DEBUG_TRACE("LPS28DFW::init bus=%u addr=0x%02X", m_bus, m_addr);

	lps28dfw_id_t id;
	if (lps28dfw_id_get(&m_ctx, &id) != 0 || id.whoami != LPS28DFW_ID) {
		DEBUG_ERROR("LPS28DFW: WHOAMI mismatch or read error");
		return false;
	}

	if (lps28dfw_init_set(&m_ctx, LPS28DFW_DRV_RDY) != 0)
		return false;

	m_mode.odr = LPS28DFW_ONE_SHOT;
	m_mode.avg = LPS28DFW_32_AVG;
	m_mode.lpf = LPS28DFW_LPF_DISABLE;
	m_mode.fs  = LPS28DFW_1260hPa;

	if (lps28dfw_mode_set(&m_ctx, &m_mode) != 0)
		return false;

	return true;
}

/// @brief Trigger one-shot, poll DRDY (50 ms timeout), read temperature + pressure.
/// @param[out] temperature  Temperature in °C.
/// @param[out] pressure     Pressure in bar (hPa / 1000).
/// @throws ErrorCode::I2C_COMMS_ERROR on I2C failure.
void LPS28DFW::read(double& temperature, double& pressure)
{
	if (!m_initialized) {
		DEBUG_ERROR("LPS28DFW::read called but not initialized");
		throw ErrorCode::I2C_COMMS_ERROR;
	}

	SensorsPowerGuard power_guard;

	// Re-apply config after power cycle (registers are volatile)
	if (lps28dfw_init_set(&m_ctx, LPS28DFW_DRV_RDY) != 0 ||
	    lps28dfw_mode_set(&m_ctx, &m_mode) != 0) {
		DEBUG_ERROR("LPS28DFW::read: re-apply config failed");
		throw ErrorCode::I2C_COMMS_ERROR;
	}

	if (lps28dfw_trigger_sw(&m_ctx, &m_mode) != 0) {
		DEBUG_ERROR("LPS28DFW::read: trigger_sw failed");
		throw ErrorCode::I2C_COMMS_ERROR;
	}

	// Poll DRDY with bounded timeout
	lps28dfw_stat_t status;
	unsigned int retries = DRDY_TIMEOUT_MS;
	do {
		PMU::delay_ms(1);
		if (lps28dfw_status_get(&m_ctx, &status) != 0) {
			DEBUG_WARN("LPS28DFW::read: status_get failed, proceeding anyway");
			break;
		}
	} while (!status.drdy_pres && --retries > 0);

	if (retries == 0) {
		// Conversion never completed. Do NOT fall through to data_get(): it would
		// read stale/zero PRESS_OUT/TEMP_OUT and return a fabricated 0 as a valid
		// sample. Fail the read so SensorService drops it (like the other paths).
		DEBUG_WARN("LPS28DFW::read: DRDY timeout (%u ms) — failing read", DRDY_TIMEOUT_MS);
		throw ErrorCode::I2C_COMMS_ERROR;
	}

	lps28dfw_data_t data;
	if (lps28dfw_data_get(&m_ctx, &m_mode, &data) != 0) {
		DEBUG_ERROR("LPS28DFW::read: data_get failed");
		throw ErrorCode::I2C_COMMS_ERROR;
	}

	temperature = static_cast<double>(data.heat.deg_c);
	pressure = static_cast<double>(data.pressure.hpa) / 1000.0;  // hPa → bar

	DEBUG_TRACE("LPS28DFW: %.2f °C | %.5f bar", temperature, pressure);
}

/// @brief Select full-scale range (applied on next read).
/// @param mode  0 = 1260 hPa (surface), 1 = 4060 hPa (underwater).
void LPS28DFW::set_full_scale(unsigned int mode)
{
	m_mode.fs = (mode == 1) ? LPS28DFW_4060hPa : LPS28DFW_1260hPa;
	DEBUG_TRACE("LPS28DFW: full_scale=%s", (mode == 1) ? "4060hPa" : "1260hPa");
}

/// @brief I2C write callback for ST driver: [reg][data...].
/// @return 0 on success, -1 on error.
int32_t LPS28DFW::platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
	auto *self = static_cast<LPS28DFW *>(handle);

	if (len == 0) return 0;
	if (static_cast<size_t>(len) + 1 > MAX_WRITE_BUF) {
		DEBUG_ERROR("LPS28DFW: write len %u exceeds buffer", len);
		return -1;
	}

	uint8_t buffer[MAX_WRITE_BUF];
	buffer[0] = reg;
	memcpy(&buffer[1], bufp, len);

	try {
		NrfI2C::write(self->m_bus, self->m_addr, buffer, len + 1, false);
	} catch (...) {
		return -1;
	}
	return 0;
}

/// @brief I2C read callback for ST driver: write reg addr (no stop), then read data.
/// @return 0 on success, -1 on error.
int32_t LPS28DFW::platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	auto *self = static_cast<LPS28DFW *>(handle);

	try {
		NrfI2C::write(self->m_bus, self->m_addr, &reg, sizeof(reg), true);
		NrfI2C::read(self->m_bus, self->m_addr, bufp, len);
	} catch (...) {
		return -1;
	}
	return 0;
}
