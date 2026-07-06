/**
 * @file smd_sat_cmd_at.hpp
 * @brief AT/UART implementation of SmdSatCmd.
 *
 * Uses NrfUartAsync for UART communication with SMD module.
 * AT command format: AT+CMD=<params>\r\n
 * Response format:   +<RESP>=<data>\r\n then +OK\r\n or +ERROR=<code>\r\n
 */

#pragma once

#include "smd_sat_cmd.hpp"
#include "nrf_uart_async.hpp"
#include <cstdint>
#include <string>
#include <optional>

class SmdSatCmdAt : public SmdSatCmd, public NrfUartAsync {
public:
	SmdSatCmdAt(unsigned int uart_instance = 1);
	~SmdSatCmdAt() override;

	// Transport lifecycle
	void init() override;
	void deinit() override;

	// Basic communication
	bool ping() override;
	void sync() override;

	// Identification & credentials
	void read_id(uint32_t *id) override;
	void set_id(uint32_t id) override;
	void read_address(smd_uint8_array_t *address) override;
	void set_address(smd_uint8_array_t *address) override;
	void read_seckey(smd_uint8_array_t *seckey) override;
	void set_seckey(smd_uint8_array_t *seckey) override;
	void read_serial(smd_uint8_array_t *serial) override;

	// Radio configuration
	void set_radio_conf(smd_uint8_array_t *radio_conf) override;
	void read_radio_conf(SmdArgosModulation *modulation) override;
	bool save_radio_conf() override;
	void read_rconf_raw(uint8_t *rconf_raw, uint16_t *len) override;

	// KMAC
	void load_kmac_profil(uint8_t profile, const uint8_t* ctx = nullptr, uint8_t ctx_len = 0) override;
	void read_kmac(uint8_t *profile) override;
	void get_kmac_status(uint8_t *status) override;

	// Power / LPM
	void read_lpm(uint8_t *lpm_mode) override;
	void write_lpm(uint8_t *lpm_mode) override;

	// TCXO
	void read_tcxo_warmup(uint32_t *time_ms) override;
	void write_tcxo_warmup(uint32_t time_ms) override;
	void set_tcxo_warmup_internal(uint32_t time_s) override;
	void set_tcxo_control(bool state) override;

	// TX
	bool initiate_tx(const KineisPacket& payload) override;
	bool is_tx_finished() override;
	bool is_tx_in_progress() override;
	bool is_tx_successful() override { return m_tx_complete && m_tx_status == 0; }

	// Status
	void get_status(uint8_t *status) override;
	void read_spimac_state(uint8_t *spi_state, uint8_t *mac_state) override;

	// Version / firmware info
	void read_version(uint8_t *version) override;
	void print_firmware_version() override;
	void read_firmware_info(uint8_t *info, uint16_t *len) override;

	// CW / Prepass / UTC date
	void read_cw(uint8_t *cw_data, uint16_t *len) override;
	void write_cw(const uint8_t *cw_data, uint16_t len) override;
	void read_prepassen(uint8_t *prepass_data, uint16_t *len) override;
	void write_prepassen(const uint8_t *prepass_data, uint16_t len) override;
	void read_udate(uint8_t *udate_data, uint16_t *len) override;
	void write_udate(const uint8_t *udate_data, uint16_t len) override;

	// DFU (via AT+BOOT + AT+DFU= commands)
	bool dfu_supported() override { return true; }
	bool is_dfu_mode() const override { return m_dfu_mode; }
	bool dfu_enter() override;
	bool dfu_exit() override;
	bool dfu_get_bootloader_info(SmdDfuInfo *info) override;
	SmdDfuResponse dfu_erase() override;
	SmdDfuResponse dfu_write_chunk(uint32_t addr, const uint8_t *data, uint16_t len) override;
	SmdDfuResponse dfu_read_chunk(uint32_t addr, uint8_t *data, uint16_t len) override;
	SmdDfuResponse dfu_verify(uint32_t crc32) override;
	SmdDfuResponse dfu_jump() override;
	uint32_t calculate_crc32(const uint8_t *data, size_t len) override;

	// Debug / test
	std::string run_command_test() override;

protected:
	/// @brief Parse a complete RX line and update response state (SMD AT protocol).
	void on_rx_line(std::string& line) override;

	/// @brief Handle UART error.
	void on_rx_error(unsigned int error_type) override;

private:
	// Response state (set by on_rx_line, read by blocking send)
	volatile bool m_resp_ok;
	volatile bool m_resp_error;
	volatile bool m_resp_data_ready;
	std::string m_resp_data;     // Parsed response data (value after +KEY=)
	std::string m_resp_key;      // Response key (e.g. "ID", "ADDR", "TX")

	// TX async state
	volatile bool m_tx_complete;     // +TX= response received
	volatile uint16_t m_tx_status;   // TX error code from +TX= response
	volatile bool m_tx_in_progress;  // AT+TX fired, awaiting +OK/+ERROR/+TX= (async, non-blocking)

	// DFU state
	bool m_dfu_mode;
	std::string m_dfu_resp_data;

	// Low-level AT operations (use NrfUartAsync for UART)
	bool send_at(const std::string& cmd, uint16_t timeout_ms = 2000);
	bool send_at_with_data(const std::string& cmd, std::string& response_data, uint16_t timeout_ms = 2000);
	bool send_dfu(uint8_t cmd_id, const std::string& hex_data = "", uint16_t timeout_ms = 5000);
	bool send_dfu_with_data(uint8_t cmd_id, const std::string& hex_data,
	                        std::string& response_data, uint16_t timeout_ms = 5000);

	// Response parsing
	void parse_response(const std::string& line);

	// Hex conversion helpers
	static std::string bytes_to_hex(const uint8_t *data, uint16_t len);
	static uint16_t hex_to_bytes(const std::string& hex, uint8_t *data, uint16_t max_len);
};
