/**
 * @file nrf_uart_async.hpp
 * @brief Common async UART driver for nRF52840 — deferred RX pattern.
 *
 * Base class for all UART-based communication layers (KIM2, LoRa RAK3172, SMD AT).
 * Handles init/deinit, ISR buffering, deferred RX processing, and TX busy management.
 *
 * ISR context: only memcpy into fixed buffer + rx_free (no heap allocation).
 * Main context: process_rx() drains ISR buffer, accumulates lines, calls on_rx_line().
 *
 * Subclasses implement:
 *   - on_rx_line(std::string& line)  — protocol-specific line parsing
 *   - on_rx_error(unsigned int)      — protocol-specific error handling
 */

#pragma once

#include <cstdint>
#include <string>
#include "nrf_libuarte_async.h"

class NrfUartAsync {
public:
	/// @param uart_instance  BSP UARTAsync_Inits index (0 or 1).
	explicit NrfUartAsync(unsigned int uart_instance);
	virtual ~NrfUartAsync();

	// Non-copyable
	NrfUartAsync(const NrfUartAsync &) = delete;
	NrfUartAsync &operator=(const NrfUartAsync &) = delete;

	/// @brief Init UART peripheral and start RX.
	/// @param baudrate_override  If non-zero, override BSP baudrate (e.g. NRF_UARTE_BAUDRATE_115200).
	/// @throws ErrorCode::RESOURCE_NOT_AVAILABLE on failure.
	void init(uint32_t baudrate_override = 0);

	/// @brief Uninit UART peripheral + Nordic SDK POWER register workaround.
	void deinit();

	/// @brief Check if UART is initialized.
	bool is_init() const { return m_is_init; }

	/// @brief Send raw bytes via UART (non-blocking).
	/// @return true if TX started, false if busy or not initialized.
	bool send_raw(const uint8_t *data, size_t len);

	/// @brief Send a string (convenience wrapper).
	bool send_string(const std::string &str);

	/// @brief Process ISR-buffered RX data in main context.
	/// Must be called periodically (e.g., in polling loops or state machine ticks).
	void process_rx();

	// ISR callbacks — called from UART interrupt handler, do NOT call directly.
	void isr_handle_tx_done();
	void isr_handle_rx_data(uint8_t *buffer, uint16_t length);
	void isr_handle_error(unsigned int error_type);

protected:
	/// @brief Called for each complete line received (stripped of CR/LF).
	/// Runs in main context (safe for heap/string operations).
	virtual void on_rx_line(std::string &line) = 0;

	/// @brief Called on UART error (deferred from ISR).
	/// @param error_type  0x100=alloc, 0x200=overrun, other=HW errorsrc.
	virtual void on_rx_error(unsigned int error_type) = 0;

	unsigned int m_uart_instance;
	bool m_is_rx_started;
	bool m_is_send_busy;
	std::string m_tx_buffer;

private:
	bool m_is_init;

	// Config snapshot (for baudrate override)
	nrf_libuarte_async_config_t m_uart_config;

	// Main-context line accumulator
	std::string m_rx_buffer;
	std::string m_line_buffer;

	// ISR-safe buffer: written by ISR, drained by process_rx() under InterruptLock
	static constexpr size_t ISR_BUF_SIZE = 256;
	static constexpr size_t MAX_RX_BUFFER_SIZE = 512;
	uint8_t m_isr_buf[ISR_BUF_SIZE];
	volatile uint16_t m_isr_buf_len;
	volatile bool m_isr_error;
	volatile unsigned int m_isr_error_type;
	volatile bool m_isr_overflow;

	void process_rx_lines();
};
