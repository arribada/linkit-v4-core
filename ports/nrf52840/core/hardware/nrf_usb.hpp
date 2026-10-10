#pragma once

/**
 * @file nrf_usb.hpp
 * @brief USB CDC ACM interface for debug output and DTE command input.
 *
 * Provides a virtual serial port over USB.  TX is blocking (queued + flushed).
 * RX uses a ring buffer filled from the USB ISR; read_line() returns a
 * complete line when available.
 *
 * All methods are static — only one USB CDC instance exists.
 */

#include <cstddef>
#include <string>

static constexpr size_t NRF_USB_RX_BUFFER_SIZE = 4128;  ///< Ring buffer size (matches BLE buffer)

class NrfUSB final {
public:
	/// @brief Init USB stack, CDC ACM class, poll for port open (up to 500 ms).
	static void init();

	/// @brief Blocking write to the CDC ACM TX endpoint.
	static int write(char *ptr, int len);

	/// @brief Read available raw bytes from the ring buffer.
	static int read(char *ptr, int max_len);

	/// @brief Read a complete line (terminated by \\r or \\n).  Returns empty if none available.
	static std::string read_line();

	/// @brief True if any unread data is in the ring buffer.
	static bool has_data();

	/// @brief Process pending USB events (call from main loop).
	static void process();

	/// @brief Called from USB event handler when port opens/closes.
	static void set_port_open(bool is_open);

	/// @brief True if the USB CDC port is currently open.
	static bool is_port_open() { return m_port_open; }

	/// @brief Bytes dropped because the TX ring was full (host not reading).
	static uint32_t tx_dropped();

	/// @brief Called from USB ISR when data arrives — stores in ring buffer.
	static void on_rx_data(const char *data, size_t len);

private:
	static bool m_port_open;
	static char m_rx_buffer[NRF_USB_RX_BUFFER_SIZE];
	static volatile size_t m_rx_write_idx;  ///< Next write position (updated from ISR)
	static volatile size_t m_rx_read_idx;   ///< Next read position (updated from thread)
	static volatile bool m_line_ready;      ///< True if at least one complete line is buffered
};
