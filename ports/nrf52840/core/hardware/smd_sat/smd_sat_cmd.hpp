/**
 * @file smd_sat_cmd.hpp
 * @brief Abstract command interface for SMD satellite module.
 *
 * Two implementations:
 *   - SmdSatCmdSpi (smd_sat_cmd_spi.cpp) — SPI Protocol A+
 *   - SmdSatCmdAt  (smd_sat_cmd_at.cpp)  — AT/UART commands
 * CMake selects which .cpp to compile.
 */

#pragma once

#include <cstdint>
#include <string>

#include "smd_sat_registers.hpp"
#include "kineis_device.hpp"

class SmdSatCmd {
public:
	virtual ~SmdSatCmd() = default;

	// ========================================================================
	// Transport lifecycle
	// ========================================================================
	virtual void init() = 0;
	virtual void deinit() = 0;

	// ========================================================================
	// Basic communication
	// ========================================================================
	virtual bool ping() = 0;
	virtual void sync() = 0;

	// ========================================================================
	// Identification & credentials
	// ========================================================================
	virtual void read_id(uint32_t *id) = 0;
	virtual void set_id(uint32_t id) = 0;
	virtual void read_address(smd_uint8_array_t *address) = 0;
	virtual void set_address(smd_uint8_array_t *address) = 0;
	virtual void read_seckey(smd_uint8_array_t *seckey) = 0;
	virtual void set_seckey(smd_uint8_array_t *seckey) = 0;
	virtual void read_serial(smd_uint8_array_t *serial) = 0;

	// ========================================================================
	// Radio configuration
	// ========================================================================
	virtual void set_radio_conf(smd_uint8_array_t *radio_conf) = 0;
	virtual void read_radio_conf(SmdArgosModulation *modulation) = 0;
	virtual bool save_radio_conf() = 0;
	virtual void read_rconf_raw(uint8_t *rconf_raw, uint16_t *len) = 0;

	// ========================================================================
	// KMAC (Key-based Message Authentication Code)
	// ========================================================================
	// Load a KMAC MAC profile. ctx (optional) = packed profile config bytes sent
	// as hex after the profile id: "AT+KMAC=<profile>,<hex(ctx)>" (BLIND uses it).
	virtual void load_kmac_profil(uint8_t profile, const uint8_t *ctx = nullptr, uint8_t ctx_len = 0) = 0;
	virtual void read_kmac(uint8_t *profile) = 0;
	virtual void get_kmac_status(uint8_t *status) = 0;

	// ========================================================================
	// Power / LPM
	// ========================================================================
	virtual void read_lpm(uint8_t *lpm_mode) = 0;
	virtual void write_lpm(uint8_t *lpm_mode) = 0;

	// ========================================================================
	// TCXO
	// ========================================================================
	virtual void read_tcxo_warmup(uint32_t *time_ms) = 0;
	virtual void write_tcxo_warmup(uint32_t time_ms) = 0;
	virtual void set_tcxo_warmup_internal(uint32_t time_s) = 0;
	virtual void set_tcxo_control(bool state) = 0;

	// ========================================================================
	// TX
	// ========================================================================
	virtual bool initiate_tx(const KineisPacket &payload) = 0;
	virtual bool is_tx_finished() = 0;
	virtual bool is_tx_in_progress() = 0;
	/// @brief After is_tx_finished() returns true, returns whether the TX
	/// actually succeeded (MAC_TX_DONE) vs. failed (MAC_TX_TIMEOUT/MAC_ERROR).
	/// Lets the upper layer distinguish "TX hardware completed" from
	/// "TX hardware completed AND was actually transmitted." Critical for
	/// AFTER_LAST_TX cooldown arming — without this distinction, a failed TX
	/// erroneously arms cooldown the same as a successful one.
	virtual bool is_tx_successful() = 0;

	// ========================================================================
	// Status
	// ========================================================================
	virtual void get_status(uint8_t *status) = 0;
	virtual void read_spimac_state(uint8_t *spi_state, uint8_t *mac_state) = 0;

	// ========================================================================
	// Argos MAC message counter (MC) — OPTIONAL.
	// Default = unsupported (returns false) so a module firmware without the
	// command degrades gracefully (the SMD keeps auto-managing its own MC; the
	// nRF simply does not hold MC across repeats). Override in the transport
	// (SPI = SMDSAT_CMD_READ_MC/WRITE_MC; AT = "AT+MC").
	// Contract: *mc / mc is the 9-bit (0-511) counter the NEXT transmission
	// will use; set_message_counter sets that next MC.
	// ========================================================================
	virtual bool read_message_counter(uint16_t *mc) {
		(void)mc;
		return false;
	}
	virtual bool set_message_counter(uint16_t mc) {
		(void)mc;
		return false;
	}

	// ========================================================================
	// Version / firmware info
	// ========================================================================
	virtual void read_version(uint8_t *version) = 0;
	virtual void print_firmware_version() = 0;
	virtual void read_firmware_info(uint8_t *info, uint16_t *len) = 0;

	// ========================================================================
	// CW / Prepass / UTC date
	// ========================================================================
	virtual void read_cw(uint8_t *cw_data, uint16_t *len) = 0;
	virtual void write_cw(const uint8_t *cw_data, uint16_t len) = 0;
	virtual void read_prepassen(uint8_t *prepass_data, uint16_t *len) = 0;
	virtual void write_prepassen(const uint8_t *prepass_data, uint16_t len) = 0;
	virtual void read_udate(uint8_t *udate_data, uint16_t *len) = 0;
	virtual void write_udate(const uint8_t *udate_data, uint16_t len) = 0;

	// ========================================================================
	// DFU (Device Firmware Update) — SPI-only, AT returns false/error
	// ========================================================================
	virtual bool dfu_supported() { return false; }
	virtual bool is_dfu_mode() const { return false; }
	virtual bool dfu_enter() { return false; }
	virtual bool dfu_exit() { return false; }
	virtual bool dfu_get_bootloader_info(SmdDfuInfo *info) {
		(void)info;
		return false;
	}
	virtual SmdDfuResponse dfu_erase() { return DFU_RSP_ERROR; }
	virtual SmdDfuResponse dfu_write_chunk(uint32_t addr, const uint8_t *data, uint16_t len) {
		(void)addr;
		(void)data;
		(void)len;
		return DFU_RSP_ERROR;
	}
	virtual SmdDfuResponse dfu_read_chunk(uint32_t addr, uint8_t *data, uint16_t len) {
		(void)addr;
		(void)data;
		(void)len;
		return DFU_RSP_ERROR;
	}
	virtual SmdDfuResponse dfu_verify(uint32_t crc32) {
		(void)crc32;
		return DFU_RSP_ERROR;
	}
	virtual SmdDfuResponse dfu_jump() { return DFU_RSP_ERROR; }
	virtual uint32_t calculate_crc32(const uint8_t *data, size_t len) {
		(void)data;
		(void)len;
		return 0;
	}

	// ========================================================================
	// Debug / test
	// ========================================================================
	virtual std::string run_command_test() { return "not implemented"; }
};
