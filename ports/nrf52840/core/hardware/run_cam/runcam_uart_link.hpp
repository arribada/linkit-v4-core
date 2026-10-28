/**
 * @file runcam_uart_link.hpp
 * @brief RunCamLink on LinkIt V4: CAM_PWR_EN for the rail, UARTE1 for the camera's UART.
 *
 * CAM_CONTROL=UART only, which CMake allows on an SMD SPI build alone: UARTE1 is
 * the satellite slot's UART, and on that build nothing else opens it (pins and
 * baud rate in bsp.cpp, UARTAsync_Inits[CAM_UART_INSTANCE]). The camera answers
 * in binary, so NrfUartAsync's line framing is bypassed through on_rx_data().
 */

#pragma once

#include "nrf_uart_async.hpp"
#include "runcam_uart_device.hpp"

class NrfRunCamLink : public RunCamLink, private NrfUartAsync {
public:
	NrfRunCamLink();

	void set_rail(bool on) override;
	bool open() override;
	void close() override;
	bool is_open() const override;
	bool send(const uint8_t *data, size_t len) override;
	size_t receive(uint8_t *buf, size_t max) override;

	/// @brief UART errors and dropped bytes since boot. Framing errors while the
	/// camera boots are expected: its TX is not driven until its UART is up.
	unsigned int rx_errors() const override { return m_rx_errors; }
	void describe(char *out, size_t out_size) const override;

private:
	/// One whole batch of NrfUartAsync's ISR buffer (its ISR_BUF_SIZE). receive()
	/// only pulls a batch in once this is empty, so nothing is ever dropped here:
	/// what overflows does so in the ISR buffer, which keeps the oldest bytes —
	/// hence the device reading its answer right after each request.
	static constexpr size_t RX_BUF_SIZE = 256;
	uint8_t m_rx[RX_BUF_SIZE] = {};
	size_t m_rx_len = 0;
	unsigned int m_rx_errors = 0;

	void on_rx_data(const uint8_t *data, size_t len) override;
	void on_rx_line(std::string &) override {}
	void on_rx_error(unsigned int error_type) override;
	/// The answers carry their own CRC: what came before an error is kept.
	bool keeps_rx_data_on_error() const override { return true; }
};
