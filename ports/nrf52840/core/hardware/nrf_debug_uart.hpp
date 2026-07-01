#pragma once

/**
 * @file nrf_debug_uart.hpp
 * @brief TX-only debug UART on UARTE1 (921600 baud, no flow control).
 *
 * Used on RSPB boards where DEBUG_UART_TX_PIN is defined.
 * All methods are static — only one debug UART instance exists.
 */

#include <cstdint>

class NrfDebugUart {
public:
	/// @brief Initialise UARTE1 for TX-only debug output.
	/// @param tx_pin  GPIO pin number for UART TX.
	static void init(uint32_t tx_pin);

	/// @brief Blocking write to the debug UART.
	/// @param data  Pointer to the data buffer.
	/// @param len   Number of bytes to transmit (ignored if <= 0).
	static void write(const char *data, int len);

	/// @brief Returns true if init() completed successfully.
	static bool is_init();

private:
	static bool m_is_init;
};
