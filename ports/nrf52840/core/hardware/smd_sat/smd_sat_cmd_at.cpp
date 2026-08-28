/**
 * @file smd_sat_cmd_at.cpp
 * @brief AT/UART command layer for SMD satellite module.
 *
 * UART lifecycle and deferred RX are handled by NrfUartAsync base class.
 * This file implements: SMD AT response parsing and command interface.
 */

#include "smd_sat_cmd_at.hpp"
#include "debug.hpp"
#include "pmu.hpp"
#include "binascii.hpp"
#include "error.hpp"  // ErrorCode (RESOURCE_NOT_AVAILABLE) thrown by the credential commands

#include "nrf_delay.h"

#include <cstring>
#include <cstdio>
#include <algorithm>

// ============================================================================
// Constructor / Destructor — delegate to NrfUartAsync
// ============================================================================

SmdSatCmdAt::SmdSatCmdAt(unsigned int uart_instance)
    : NrfUartAsync(uart_instance),
      m_resp_ok(false),
      m_resp_error(false),
      m_resp_data_ready(false),
      m_tx_complete(false),
      m_tx_status(0xFFFF),
      m_tx_in_progress(false),
      m_dfu_mode(false) {}

SmdSatCmdAt::~SmdSatCmdAt() {
	NrfUartAsync::deinit();
}

// ============================================================================
// Transport lifecycle — delegate to NrfUartAsync
// ============================================================================

void SmdSatCmdAt::init() {
	NrfUartAsync::init();  // Uses BSP default baudrate
	m_dfu_mode = false;
}

void SmdSatCmdAt::deinit() {
	NrfUartAsync::deinit();
}

// ============================================================================
// NrfUartAsync callbacks — protocol parsing
// ============================================================================

void SmdSatCmdAt::on_rx_line(std::string &line) {
	parse_response(line);
}

void SmdSatCmdAt::on_rx_error(unsigned int error_type) {
	DEBUG_WARN("SmdSatCmdAt: UART error type=%02x", error_type);
}

// ============================================================================
// Response parsing
// ============================================================================

void SmdSatCmdAt::parse_response(const std::string &line) {
	// All responses start with '+'
	if (line.empty() || line[0] != '+') return;

	// +OK
	if (line == "+OK") {
		m_resp_ok = true;
		return;
	}

	// +ERROR=<code>
	if (line.compare(0, 7, "+ERROR=") == 0) {
		m_resp_error = true;
		m_resp_ok = true;  // Unblock the (synchronous) wait loop
		// If an async AT+TX is in flight, surface the rejection as a FAILED TX
		// completion so the state machine's is_tx_finished()/is_tx_successful()
		// poll fails fast, instead of waiting out the full TX timeout budget.
		if (m_tx_in_progress) {
			m_tx_status = 0xFFFF;  // non-zero → is_tx_successful() == false
			m_tx_complete = true;
			m_tx_in_progress = false;
		}
		DEBUG_WARN("SmdSatCmdAt: %s", line.c_str());
		return;
	}

	// +BOOT=OK
	if (line == "+BOOT=OK") {
		m_resp_ok = true;
		return;
	}

	// +DFU=OK[,<data>] or +DFU=ERR,<error>
	if (line.compare(0, 5, "+DFU=") == 0) {
		std::string dfu_resp = line.substr(5);
		if (dfu_resp.compare(0, 2, "OK") == 0) {
			m_resp_ok = true;
			if (dfu_resp.size() > 3 && dfu_resp[2] == ',') {
				m_dfu_resp_data = dfu_resp.substr(3);
			} else {
				m_dfu_resp_data.clear();
			}
		} else if (dfu_resp.compare(0, 3, "ERR") == 0) {
			m_resp_error = true;
			m_resp_ok = true;
			DEBUG_WARN("SmdSatCmdAt: DFU error: %s", dfu_resp.c_str());
		}
		return;
	}

	// +TX=<status>,<data> (async TX completion)
	if (line.compare(0, 4, "+TX=") == 0) {
		std::string tx_resp = line.substr(4);
		size_t comma = tx_resp.find(',');
		if (comma != std::string::npos) {
			try {
				m_tx_status = std::stoi(tx_resp.substr(0, comma));
			} catch (...) {
				m_tx_status = 0xFFFF;
			}
		}
		m_tx_complete = true;
		m_tx_in_progress = false;  // TX resolved (completion received)
		// Also set resp_ok since AT+TX waits for MAC acceptance
		m_resp_ok = true;
		return;
	}

	// Generic +KEY=<data> response
	size_t eq = line.find('=');
	if (eq != std::string::npos) {
		m_resp_key = line.substr(1, eq - 1);  // Strip leading '+'
		m_resp_data = line.substr(eq + 1);
		m_resp_data_ready = true;
	}
}

// ============================================================================
// Low-level AT operations — use NrfUartAsync for UART
// ============================================================================

bool SmdSatCmdAt::send_at(const std::string &cmd, uint16_t timeout_ms) {
	m_resp_ok = false;
	m_resp_error = false;
	m_resp_data_ready = false;

	if (!send_string(cmd + "\r\n")) return false;

	// Wait for +OK or +ERROR, draining ISR buffer each tick
	while (!m_resp_ok && timeout_ms > 0) {
		PMU::delay_ms(1);
		NrfUartAsync::process_rx();
		timeout_ms--;
	}

	if (timeout_ms == 0) {
		DEBUG_WARN("SmdSatCmdAt::send_at: timeout for '%s'", cmd.c_str());
		return false;
	}

	return !m_resp_error;
}

bool SmdSatCmdAt::send_at_with_data(const std::string &cmd, std::string &response_data, uint16_t timeout_ms) {
	m_resp_ok = false;
	m_resp_error = false;
	m_resp_data_ready = false;
	m_resp_data.clear();

	if (!send_string(cmd + "\r\n")) return false;

	while (!m_resp_ok && timeout_ms > 0) {
		PMU::delay_ms(1);
		NrfUartAsync::process_rx();
		timeout_ms--;
	}

	if (timeout_ms == 0 || m_resp_error) return false;

	response_data = m_resp_data;
	return true;
}

bool SmdSatCmdAt::send_dfu(uint8_t cmd_id, const std::string &hex_data, uint16_t timeout_ms) {
	std::string cmd = "AT+DFU=" + std::to_string(cmd_id);
	if (!hex_data.empty()) {
		cmd += "," + hex_data;
	}

	m_resp_ok = false;
	m_resp_error = false;
	m_dfu_resp_data.clear();

	if (!send_string(cmd + "\r\n")) return false;

	while (!m_resp_ok && timeout_ms > 0) {
		if (timeout_ms % 500 == 0) PMU::kick_watchdog();
		PMU::delay_ms(1);
		NrfUartAsync::process_rx();
		timeout_ms--;
	}

	return !m_resp_error && (timeout_ms > 0);
}

bool SmdSatCmdAt::send_dfu_with_data(uint8_t cmd_id, const std::string &hex_data, std::string &response_data,
                                     uint16_t timeout_ms) {
	if (!send_dfu(cmd_id, hex_data, timeout_ms)) return false;
	response_data = m_dfu_resp_data;
	return true;
}

// ============================================================================
// Hex conversion helpers
// ============================================================================

std::string SmdSatCmdAt::bytes_to_hex(const uint8_t *data, uint16_t len) {
	std::string hex;
	hex.reserve(len * 2);
	for (uint16_t i = 0; i < len; i++) {
		char buf[3];
		snprintf(buf, sizeof(buf), "%02x", data[i]);
		hex += buf;
	}
	return hex;
}

uint16_t SmdSatCmdAt::hex_to_bytes(const std::string &hex, uint8_t *data, uint16_t max_len) {
	uint16_t len = hex.size() / 2;
	if (len > max_len) len = max_len;
	for (uint16_t i = 0; i < len; i++) {
		unsigned int val;
		if (sscanf(hex.c_str() + i * 2, "%2x", &val) != 1) return i;  // Return number of bytes successfully converted
		data[i] = static_cast<uint8_t>(val);
	}
	return len;
}

// ============================================================================
// Basic communication
// ============================================================================

bool SmdSatCmdAt::ping() {
	return send_at("AT+PING=?");
}

void SmdSatCmdAt::sync() {
	// No sync needed for UART
}

// ============================================================================
// Identification & credentials
// ============================================================================

void SmdSatCmdAt::read_id(uint32_t *id) {
	std::string data;
	if (send_at_with_data("AT+ID=?", data) && !data.empty()) {
		try {
			*id = std::stoul(data);
		} catch (...) {
			*id = 0;
			throw ErrorCode::RESOURCE_NOT_AVAILABLE;
		}
	} else {
		throw ErrorCode::RESOURCE_NOT_AVAILABLE;
	}
}

void SmdSatCmdAt::set_id(uint32_t id) {
	if (!send_at("AT+ID=" + std::to_string(id))) {
		throw ErrorCode::RESOURCE_NOT_AVAILABLE;
	}
}

void SmdSatCmdAt::read_address(smd_uint8_array_t *address) {
	std::string data;
	if (send_at_with_data("AT+ADDR=?", data) && !data.empty()) {
		hex_to_bytes(data, address->p_data, address->size);
	} else {
		throw ErrorCode::RESOURCE_NOT_AVAILABLE;
	}
}

void SmdSatCmdAt::set_address(smd_uint8_array_t *address) {
	std::string hex = bytes_to_hex(address->p_data, address->size);
	if (!send_at("AT+ADDR=" + hex)) {
		throw ErrorCode::RESOURCE_NOT_AVAILABLE;
	}
}

void SmdSatCmdAt::read_seckey(smd_uint8_array_t *seckey) {
	std::string data;
	if (send_at_with_data("AT+SECKEY=?", data) && !data.empty()) {
		hex_to_bytes(data, seckey->p_data, seckey->size);
	} else {
		throw ErrorCode::RESOURCE_NOT_AVAILABLE;
	}
}

void SmdSatCmdAt::set_seckey(smd_uint8_array_t *seckey) {
	std::string hex = bytes_to_hex(seckey->p_data, seckey->size);
	if (!send_at("AT+SECKEY=" + hex)) {
		throw ErrorCode::RESOURCE_NOT_AVAILABLE;
	}
}

void SmdSatCmdAt::read_serial(smd_uint8_array_t *serial) {
	std::string data;
	if (send_at_with_data("AT+SN=?", data) && !data.empty()) {
		uint16_t len = std::min(static_cast<uint16_t>(data.size()), serial->size);
		memcpy(serial->p_data, data.c_str(), len);
		serial->size = len;
	} else {
		throw ErrorCode::RESOURCE_NOT_AVAILABLE;
	}
}

// ============================================================================
// Radio configuration
// ============================================================================

void SmdSatCmdAt::set_radio_conf(smd_uint8_array_t *radio_conf) {
	std::string hex = bytes_to_hex(radio_conf->p_data, radio_conf->size);
	if (!send_at("AT+RCONF=" + hex)) {
		throw ErrorCode::RESOURCE_NOT_AVAILABLE;
	}
}

void SmdSatCmdAt::read_radio_conf(SmdArgosModulation *modulation) {
	std::string data;
	if (send_at_with_data("AT+RCONF=?", data) && !data.empty()) {
		// Response: <min_freq>,<max_freq>,<rf_level>,<modulation>
		// Extract modulation string (last field)
		size_t last_comma = data.rfind(',');
		if (last_comma != std::string::npos) {
			std::string mod_str = data.substr(last_comma + 1);
			if (mod_str == "LDA2" || mod_str == "LDA2L") {
				*modulation = ARGOS_MOD_LDA2;
			} else if (mod_str == "LDK") {
				*modulation = ARGOS_MOD_LDK;
			} else if (mod_str == "VLDA4") {
				*modulation = ARGOS_MOD_VLDA4;
			} else {
				*modulation = ARGOS_MOD_LDA2;  // Default
			}
		}
	} else {
		throw ErrorCode::RESOURCE_NOT_AVAILABLE;
	}
}

bool SmdSatCmdAt::save_radio_conf() {
	return send_at("AT+SAVE_RCONF");
}

void SmdSatCmdAt::read_rconf_raw(uint8_t *rconf_raw, uint16_t *len) {
	std::string data;
	if (send_at_with_data("AT+RCONFRAW=?", data) && !data.empty()) {
		*len = hex_to_bytes(data, rconf_raw, SMDSAT_CMD_READ_RCONF_RAW_LEN);
	} else {
		throw ErrorCode::RESOURCE_NOT_AVAILABLE;
	}
}

// ============================================================================
// KMAC
// ============================================================================

void SmdSatCmdAt::load_kmac_profil(uint8_t profile, const uint8_t *ctx, uint8_t ctx_len) {
	std::string cmd = "AT+KMAC=" + std::to_string(profile);
	if (ctx != nullptr && ctx_len > 0) {
		// Append the packed profile config as hex: "AT+KMAC=<id>,<hex>"
		cmd += "," + Binascii::hexlify(std::string(reinterpret_cast<const char *>(ctx), ctx_len));
	}
	if (!send_at(cmd)) {
		throw ErrorCode::RESOURCE_NOT_AVAILABLE;
	}
}

void SmdSatCmdAt::read_kmac(uint8_t *profile) {
	std::string data;
	if (send_at_with_data("AT+KMAC=?", data) && !data.empty()) {
		// Response: <profile_id>,<config_hex>
		try {
			*profile = std::stoi(data.substr(0, data.find(',')));
		} catch (...) {
			*profile = 0;
		}
	} else {
		throw ErrorCode::RESOURCE_NOT_AVAILABLE;
	}
}

void SmdSatCmdAt::get_kmac_status(uint8_t *status) {
	// AT interface has no direct KMAC status query
	// After KMAC load, status is OK if no error was returned
	*status = MAC_OK;
}

// ============================================================================
// Power / LPM
// ============================================================================

void SmdSatCmdAt::read_lpm(uint8_t *lpm_mode) {
	std::string data;
	if (send_at_with_data("AT+LPM=?", data) && !data.empty()) {
		try {
			*lpm_mode = static_cast<uint8_t>(std::stoul(data, nullptr, 16));
		} catch (...) {
			*lpm_mode = 0;
		}
	} else {
		throw ErrorCode::RESOURCE_NOT_AVAILABLE;
	}
}

void SmdSatCmdAt::write_lpm(uint8_t *lpm_mode) {
	char buf[8];
	snprintf(buf, sizeof(buf), "0x%02X", *lpm_mode);
	if (!send_at(std::string("AT+LPM=") + buf)) {
		throw ErrorCode::RESOURCE_NOT_AVAILABLE;
	}
}

// ============================================================================
// TCXO
// ============================================================================

void SmdSatCmdAt::read_tcxo_warmup(uint32_t *time_ms) {
	std::string data;
	if (send_at_with_data("AT+TCXO_WU=?", data) && !data.empty()) {
		try {
			*time_ms = std::stoul(data);
		} catch (...) {
			*time_ms = 0;
		}
	} else {
		throw ErrorCode::RESOURCE_NOT_AVAILABLE;
	}
}

void SmdSatCmdAt::write_tcxo_warmup(uint32_t time_ms) {
	if (!send_at("AT+TCXO_WU=" + std::to_string(time_ms))) {
		throw ErrorCode::RESOURCE_NOT_AVAILABLE;
	}
}

void SmdSatCmdAt::set_tcxo_warmup_internal(uint32_t time_s) {
	write_tcxo_warmup(time_s * 1000);
}

void SmdSatCmdAt::set_tcxo_control(bool state) {
	(void)state;
	// No AT equivalent for TCXO control — managed internally by SMD firmware
}

// ============================================================================
// TX
// ============================================================================

bool SmdSatCmdAt::initiate_tx(const KineisPacket &payload) {
	DEBUG_TRACE("SmdSatCmdAt::%s: Size: %u", __func__, payload.size());

	std::string hex = Binascii::hexlify(payload);

	m_tx_complete = false;
	m_tx_status = 0xFFFF;
	m_resp_error = false;
	m_resp_ok = false;
	m_tx_in_progress = true;  // arm +ERROR / +TX= async surfacing

	// Fire AT+TX NON-BLOCKING (was a 5 s busy-wait via send_at, which froze the
	// cooperative scheduler). The +OK (MAC acceptance) and the async
	// +TX=<status> completion are surfaced through is_tx_finished()/
	// is_tx_successful(), polled by the shared state machine — same contract as
	// the SPI backend, same async UART transport principle as KIM2.
	if (!send_string("AT+TX=" + hex + "\r\n")) {
		m_tx_in_progress = false;
		return false;
	}
	return true;
}

bool SmdSatCmdAt::is_tx_finished() {
	// C2 fix: initiate_tx() fires AT+TX non-blocking and the +TX=<status> completion
	// arrives asynchronously — it is parsed ONLY by process_rx(). The state machine
	// polls this accessor without otherwise running a send_at* (the only other
	// process_rx caller), so drain here or the completion is never parsed and every
	// TX times out.
	NrfUartAsync::process_rx();
	return m_tx_complete;
}

bool SmdSatCmdAt::is_tx_in_progress() {
	NrfUartAsync::process_rx();  // parse any pending async +TX= completion first
	return !m_tx_complete;
}

// ============================================================================
// Status
// ============================================================================

void SmdSatCmdAt::get_status(uint8_t *status) {
	// No direct AT equivalent — return OK if ping succeeds
	*status = ping() ? 0x00 : 0xFF;
}

void SmdSatCmdAt::read_spimac_state(uint8_t *spi_state, uint8_t *mac_state) {
	// No real AT MAC-state register — synthesize it. C1 fix: report TX_IN_PROGRESS
	// ONLY while a TX is actually in flight; once a TX completed report its verdict;
	// OTHERWISE (idle / no TX pending — e.g. during state_load_kmac at boot) report
	// MAC_OK so the mac_ready gate can pass (mirrors get_kmac_status()). The old code
	// returned MAC_TX_IN_PROGRESS whenever !m_tx_complete, which at boot (no TX ever
	// done) made load_kmac's gate permanently unreachable -> KMAC never loaded, TX
	// never started, per-session dead-end into cooldown.
	if (spi_state) *spi_state = 0;
	if (mac_state) {
		if (m_tx_in_progress)
			*mac_state = MAC_TX_IN_PROGRESS;
		else if (m_tx_complete)
			*mac_state = (m_tx_status == 0) ? MAC_TX_DONE : MAC_TX_TIMEOUT;
		else
			*mac_state = MAC_OK;  // idle / ready
	}
}

// ============================================================================
// Version / firmware info
// ============================================================================

void SmdSatCmdAt::read_version(uint8_t *version) {
	std::string data;
	if (send_at_with_data("AT+VERSION=?", data) && !data.empty()) {
		uint16_t len = std::min(static_cast<uint16_t>(data.size()), static_cast<uint16_t>(63));
		memcpy(version, data.c_str(), len);
		version[len] = 0;
	}
}

void SmdSatCmdAt::print_firmware_version() {
	std::string data;
	if (send_at_with_data("AT+FW=?", data)) {
		DEBUG_INFO("SmdSatCmdAt: FW=%s", data.c_str());
	}
}

void SmdSatCmdAt::read_firmware_info(uint8_t *info, uint16_t *len) {
	std::string data;
	if (send_at_with_data("AT+FW=?", data) && !data.empty()) {
		uint16_t copy_len = std::min(static_cast<uint16_t>(data.size()), *len);
		memcpy(info, data.c_str(), copy_len);
		*len = copy_len;
	} else {
		*len = 0;
	}
}

// ============================================================================
// CW / Prepass / UTC date
// ============================================================================

void SmdSatCmdAt::read_cw(uint8_t *cw_data, uint16_t *len) {
	(void)cw_data;
	*len = 0;
	// CW is set-only via AT+CW=<mode>,<freq>,<power>[,<period>]
}

void SmdSatCmdAt::write_cw(const uint8_t *cw_data, uint16_t len) {
	// CW data format: mode(1) + freq(4) + power(2) + optional period(2)
	if (len < 7) return;

	uint8_t mode = cw_data[0];
	int32_t freq = (int32_t)((cw_data[1]) | (cw_data[2] << 8) | (cw_data[3] << 16) | (cw_data[4] << 24));
	uint16_t power = cw_data[5] | (cw_data[6] << 8);

	char cmd[64];
	if (len >= 9) {
		uint16_t period = cw_data[7] | (cw_data[8] << 8);
		snprintf(cmd, sizeof(cmd), "AT+CW=%02x,%ld,%u,%u", mode, (long)freq, power, period);
	} else {
		snprintf(cmd, sizeof(cmd), "AT+CW=%02x,%ld,%u", mode, (long)freq, power);
	}

	send_at(cmd);
}

void SmdSatCmdAt::read_prepassen(uint8_t *prepass_data, uint16_t *len) {
	(void)prepass_data;
	*len = 0;
	// Prepass is a stub in AT firmware (always returns 0)
}

void SmdSatCmdAt::write_prepassen(const uint8_t *prepass_data, uint16_t len) {
	(void)prepass_data;
	(void)len;
	// Stub — AT+PREPASS_EN is not implemented in firmware
}

void SmdSatCmdAt::read_udate(uint8_t *udate_data, uint16_t *len) {
	(void)udate_data;
	*len = 0;
	// AT+UDATE is a stub
}

void SmdSatCmdAt::write_udate(const uint8_t *udate_data, uint16_t len) {
	(void)udate_data;
	(void)len;
	// AT+UDATE is a stub
}

// ============================================================================
// DFU (via AT+BOOT + AT+DFU= commands)
// ============================================================================
// Flow: AT+BOOT → reboot into bootloader → AT+DFU=<cmd_id>[,<hex_data>]

bool SmdSatCmdAt::dfu_enter() {
	DEBUG_INFO("SmdSatCmdAt::%s: Sending AT+BOOT", __func__);

	// Send AT+BOOT to reboot into bootloader
	if (!send_at("AT+BOOT=", 3000)) {
		DEBUG_ERROR("SmdSatCmdAt::%s: AT+BOOT failed", __func__);
		return false;
	}

	// Wait for bootloader to start
	nrf_delay_ms(1000);
	PMU::kick_watchdog();

	// PING bootloader
	std::string resp;
	for (int attempt = 0; attempt < 10; attempt++) {
		if (send_dfu_with_data(1, "", resp, 2000)) {  // DFU PING = cmd_id 1
			DEBUG_INFO("SmdSatCmdAt::%s: Bootloader ready: %s", __func__, resp.c_str());
			m_dfu_mode = true;
			return true;
		}
		nrf_delay_ms(500);
		PMU::kick_watchdog();
	}

	DEBUG_ERROR("SmdSatCmdAt::%s: Bootloader not responding", __func__);
	return false;
}

bool SmdSatCmdAt::dfu_exit() {
	if (!m_dfu_mode) return true;

	// JUMP = cmd_id 8
	if (send_dfu(8, "", 3000)) {
		m_dfu_mode = false;
		nrf_delay_ms(1000);
		return true;
	}
	return false;
}

bool SmdSatCmdAt::dfu_get_bootloader_info(SmdDfuInfo *info) {
	if (!m_dfu_mode || !info) return false;

	// GET_INFO = cmd_id 2
	std::string resp;
	if (!send_dfu_with_data(2, "", resp, 2000)) return false;

	// Parse response: hex data containing version, app_start, max_size, page_size
	uint8_t data[32] = { 0 };
	uint16_t len = hex_to_bytes(resp, data, sizeof(data));

	if (len >= 15) {
		info->version_major = data[0];
		info->version_minor = data[1];
		info->version_patch = data[2];
		info->app_start_addr = data[3] | (data[4] << 8) | (data[5] << 16) | (data[6] << 24);
		info->app_max_size = data[7] | (data[8] << 8) | (data[9] << 16) | (data[10] << 24);
		info->flash_page_size = data[11] | (data[12] << 8) | (data[13] << 16) | (data[14] << 24);

		DEBUG_INFO("SmdSatCmdAt::%s: BL v%u.%u.%u | app=0x%08X | max=%u | page=%u", __func__, info->version_major,
		           info->version_minor, info->version_patch, info->app_start_addr, info->app_max_size,
		           info->flash_page_size);
	}

	return true;
}

SmdDfuResponse SmdSatCmdAt::dfu_erase() {
	DEBUG_INFO("SmdSatCmdAt::%s: Erasing flash...", __func__);
	PMU::kick_watchdog();

	// ERASE = cmd_id 3, long timeout (flash erase ~3s)
	if (send_dfu(3, "", 10000)) {
		DEBUG_INFO("SmdSatCmdAt::%s: Erase OK", __func__);
		return DFU_RSP_OK;
	}
	return DFU_RSP_ERROR;
}

SmdDfuResponse SmdSatCmdAt::dfu_write_chunk(uint32_t addr, const uint8_t *data, uint16_t len) {
	if (!data || len == 0 || len > SMDSAT_DFU_CHUNK_SIZE) {
		return DFU_RSP_SIZE_ERROR;
	}

	// WRITE = cmd_id 4, payload: <addr_4B_LE><data>
	// Build hex: 4 bytes addr (LE) + data
	uint8_t header[4] = { static_cast<uint8_t>(addr & 0xFF), static_cast<uint8_t>((addr >> 8) & 0xFF),
	                      static_cast<uint8_t>((addr >> 16) & 0xFF), static_cast<uint8_t>((addr >> 24) & 0xFF) };

	std::string hex = bytes_to_hex(header, 4) + bytes_to_hex(data, len);

	if (send_dfu(4, hex, 3000)) {
		return DFU_RSP_OK;
	}
	return DFU_RSP_ERROR;
}

SmdDfuResponse SmdSatCmdAt::dfu_read_chunk(uint32_t addr, uint8_t *data, uint16_t len) {
	if (!data || len == 0) return DFU_RSP_SIZE_ERROR;

	// READ = cmd_id 5, payload: <addr_4B_LE><len_2B_LE>
	uint8_t req[6] = { static_cast<uint8_t>(addr & 0xFF),         static_cast<uint8_t>((addr >> 8) & 0xFF),
	                   static_cast<uint8_t>((addr >> 16) & 0xFF), static_cast<uint8_t>((addr >> 24) & 0xFF),
	                   static_cast<uint8_t>(len & 0xFF),          static_cast<uint8_t>((len >> 8) & 0xFF) };

	std::string resp;
	if (send_dfu_with_data(5, bytes_to_hex(req, 6), resp, 3000)) {
		hex_to_bytes(resp, data, len);
		return DFU_RSP_OK;
	}
	return DFU_RSP_ERROR;
}

SmdDfuResponse SmdSatCmdAt::dfu_verify(uint32_t crc32) {
	// VERIFY = cmd_id 6, payload: <crc32_4B_LE>
	uint8_t crc_data[4] = { static_cast<uint8_t>(crc32 & 0xFF), static_cast<uint8_t>((crc32 >> 8) & 0xFF),
	                        static_cast<uint8_t>((crc32 >> 16) & 0xFF), static_cast<uint8_t>((crc32 >> 24) & 0xFF) };

	if (send_dfu(6, bytes_to_hex(crc_data, 4), 5000)) {
		DEBUG_INFO("SmdSatCmdAt::%s: CRC OK", __func__);
		return DFU_RSP_OK;
	}
	return DFU_RSP_ERROR;
}

SmdDfuResponse SmdSatCmdAt::dfu_jump() {
	DEBUG_INFO("SmdSatCmdAt::%s: Jumping to app...", __func__);

	// JUMP = cmd_id 8
	send_dfu(8, "", 3000);
	m_dfu_mode = false;
	nrf_delay_ms(SMDSAT_DFU_RESET_DELAY_MS);

	return DFU_RSP_OK;
}

uint32_t SmdSatCmdAt::calculate_crc32(const uint8_t *data, size_t len) {
	return spi_crc32_mpeg2(data, len);
}

// ============================================================================
// Debug / test
// ============================================================================

std::string SmdSatCmdAt::run_command_test() {
	DEBUG_INFO("SmdSatCmdAt::%s: === AT COMMAND TEST START ===", __func__);

	struct CmdTest {
		const char *cmd;
		const char *name;
	};

	CmdTest tests[] = {
		{ "AT+PING=?", "PING" }, { "AT+VERSION=?", "VERSION" }, { "AT+FW=?", "FW" },
		{ "AT+ID=?", "ID" },     { "AT+ADDR=?", "ADDR" },       { "AT+SECKEY=?", "SECKEY" },
		{ "AT+SN=?", "SN" },     { "AT+RCONF=?", "RCONF" },     { "AT+RCONFRAW=?", "RCONFRAW" },
		{ "AT+KMAC=?", "KMAC" }, { "AT+LPM=?", "LPM" },         { "AT+TCXO_WU=?", "TCXO_WU" },
	};

	uint8_t pass_count = 0;
	uint8_t num_tests = sizeof(tests) / sizeof(tests[0]);
	std::string fails;

	for (uint8_t i = 0; i < num_tests; i++) {
		std::string data;
		bool ok = send_at_with_data(tests[i].cmd, data);

		if (ok) {
			pass_count++;
			DEBUG_INFO("  PASS: %s = %s", tests[i].name, data.c_str());
		} else {
			if (!fails.empty()) fails += " ";
			fails += tests[i].name;
			DEBUG_ERROR("  FAIL: %s", tests[i].name);
		}
		nrf_delay_ms(50);
	}

	std::string result = std::to_string(pass_count) + "/" + std::to_string(num_tests);
	if (fails.empty()) {
		result += " ALL OK";
	} else {
		result += " FAIL:" + fails;
	}

	DEBUG_INFO("SmdSatCmdAt::%s: === TEST RESULT: %s ===", __func__, result.c_str());
	return result;
}
