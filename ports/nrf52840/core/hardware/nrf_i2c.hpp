#pragma once

/**
 * @file nrf_i2c.hpp
 * @brief nRF52840 TWIM (I2C master) driver with async transfers, timeout, and bus recovery.
 *
 * All operations are async with a bounded timeout (default 100 ms).
 * On failure, the driver attempts up to 3 retries with automatic bus
 * recovery (clock stretching -> full bus reset).  Per-bus statistics
 * track timeouts, errors, and recovery events for field diagnostics.
 *
 * Two API flavours:
 *  - read()/write(): throw ErrorCode::I2C_COMMS_ERROR on failure (legacy)
 *  - read_safe()/write_safe(): return bool (preferred for graceful handling)
 */

#include <cstdint>
#include "bsp.hpp"

/// @name I2C driver tuning constants
/// @{
static constexpr uint32_t I2C_OPERATION_TIMEOUT_MS = 100;  ///< Max wait per async transfer
static constexpr uint8_t I2C_MAX_RETRIES = 3;              ///< Retries before giving up
static constexpr uint8_t I2C_BUS_RECOVERY_CYCLES = 9;      ///< SCL clock pulses for recovery
static constexpr uint8_t I2C_RECOVERY_MAX_ATTEMPTS = 3;    ///< Clock-stretch attempts before full reset
/// @}

/// @brief Per-bus I2C statistics for field diagnostics.
struct I2CStats {
	uint32_t timeouts;
	uint32_t errors;
	uint32_t bus_recoveries;
	uint32_t total_operations;
	uint32_t successful_operations;
};

class NrfI2C {
private:
	static inline bool m_is_enabled[BSP::I2C_TOTAL_NUMBER] = { false };
	static inline volatile bool m_transfer_done[BSP::I2C_TOTAL_NUMBER] = { false };
	static inline volatile bool m_transfer_error[BSP::I2C_TOTAL_NUMBER] = { false };
	static inline I2CStats m_stats[BSP::I2C_TOTAL_NUMBER] = {};

	static bool wait_for_transfer(uint8_t bus, uint32_t timeout_ms);
	static bool is_bus_stuck(uint8_t bus);
	static bool clock_stretch_recovery(uint8_t bus);
	static bool full_bus_reset(uint8_t bus);

	/// @brief Uninit + reinit TWIM peripheral with async event handler.
	/// @param bus  I2C bus index.
	/// @return true if reinit succeeded.
	static bool reinit_bus(uint8_t bus);

	/**
	 * @brief Common retry loop for read_safe/write_safe.
	 * @param bus       I2C bus index.
	 * @param address   7-bit I2C slave address.
	 * @param buffer    TX or RX data buffer.
	 * @param length    Number of bytes.
	 * @param is_read   true for RX, false for TX.
	 * @param no_stop   true to suppress I2C STOP (TX only, for repeated start).
	 * @param op_name   Operation name for debug logs ("read" or "write").
	 * @return true on success, false after all retries exhausted.
	 */
	static bool transfer_with_retry(uint8_t bus, uint8_t address, uint8_t *buffer, unsigned int length, bool is_read,
	                                bool no_stop, const char *op_name);

public:
	/// @brief ISR callback — called from TWIM event handlers (C-style).
	/// @param bus    I2C bus index that completed the transfer.
	/// @param error  true if NACK, overrun, or bus error occurred.
	static void event_handler(uint8_t bus, bool error);

	static void init(void);
	static void uninit(void);
	static void disable(uint8_t bus);

	/// @name Throwing API (legacy) — throw ErrorCode::I2C_COMMS_ERROR on failure
	/// @{
	static void read(uint8_t bus, uint8_t address, uint8_t *buffer, unsigned int length);
	static void write(uint8_t bus, uint8_t address, const uint8_t *buffer, unsigned int length, bool no_stop);
	/// @}

	/// @name Non-throwing API — return true on success, false on failure
	/// @{
	[[nodiscard]] static bool read_safe(uint8_t bus, uint8_t address, uint8_t *buffer, unsigned int length);
	[[nodiscard]] static bool write_safe(uint8_t bus, uint8_t address, const uint8_t *buffer, unsigned int length,
	                                     bool no_stop);
	/// @}

	static bool recover_bus(uint8_t bus);

	static uint8_t num_buses(void);
	static bool is_enabled(uint8_t bus);
	static const I2CStats &get_stats(uint8_t bus);
	static void reset_stats(uint8_t bus);
};
