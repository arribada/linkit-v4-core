/**
 * @file smd_sat_cmd_spi.hpp
 * @brief SPI Protocol A+ implementation of SmdSatCmd.
 */

#pragma once

#include "smd_sat_cmd.hpp"
#include "nrf_spim.hpp"
struct SpiAplusResponse {
    uint8_t seq;
    SpiAplusStatus status;
    uint8_t data[SPI_PROTOCOL_APLUS_MAX_DATA_LEN];
    uint16_t data_len;
    bool valid;
    uint8_t raw_status;   // real STATUS byte from the frame, preserved even when CRC fails
    bool crc_error;       // true when the frame was structurally valid but the CRC8 mismatched
};

// Protocol mode enumeration
enum class SpiProtocolMode {
    LEGACY,
    APLUS,
    AUTO_DETECT
};

// ============================================================================
// SmdSatCmdSpi — SPI Protocol A+ implementation of SmdSatCmd
// ============================================================================
class SmdSatCmdSpi : public SmdSatCmd {
public:
	SmdSatCmdSpi();
	~SmdSatCmdSpi() override;

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
	bool is_tx_successful() override { return m_last_tx_status == MAC_TX_DONE; }

	// Status
	void get_status(uint8_t *status) override;
	void read_spimac_state(uint8_t *spi_state, uint8_t *mac_state) override;

	// Argos MAC message counter (MC) — SMDSAT_CMD_READ_MC / WRITE_MC.
	// Return false if the module firmware lacks the command (INVALID_CMD).
	bool read_message_counter(uint16_t *mc) override;
	bool set_message_counter(uint16_t mc) override;

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

	// DFU
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
	std::string run_tx_flow_test();
	uint8_t poll_tx_result(uint32_t timeout_ms);

	// SPI-specific: direct access to NrfSPIM (needed by SmdSat for power management)
	NrfSPIM* get_spim() const { return m_nrf_spim; }
	void set_spim(NrfSPIM* spim) { m_nrf_spim = spim; }

private:
	NrfSPIM *m_nrf_spim;

	// Protocol A+ state
	SpiProtocolMode m_protocol_mode;
	uint8_t m_sequence_number;  // Wraps at 256 — matches Zephyr argos-smd-driver behavior
	bool m_protocol_detected;
	bool m_seq_reset_attempted; // Prevents infinite retry loop on INVALID_CMD

	// DFU state
	bool m_dfu_mode;
	SmdDfuInfo m_dfu_info;

	// Last TX status — populated by is_tx_finished() when it returns true.
	// Used by is_tx_successful() to distinguish MAC_TX_DONE (real success)
	// from MAC_TX_TIMEOUT / MAC_ERROR (failures). 0 = no TX yet.
	uint8_t m_last_tx_status = 0;

	// Legacy SPI
	void read_byte(uint8_t *byte_read);
	void send_command(uint8_t command);
	void send_command(const uint8_t *tx_data, uint8_t *rx_data, uint16_t size);

	// Protocol A+ core
	uint16_t build_aplus_frame(uint8_t *frame, uint8_t cmd, const uint8_t *data, uint16_t data_len);
	bool parse_aplus_response(const uint8_t *rx_buffer, uint16_t rx_len, SpiAplusResponse *response);
	bool send_command_aplus(uint8_t command, const uint8_t *tx_data, uint16_t tx_len,
	                        SpiAplusResponse *response);
	bool send_command_auto(uint8_t command, const uint8_t *tx_data = nullptr, uint16_t tx_len = 0,
	                       uint8_t *rx_data = nullptr, uint16_t *rx_len = nullptr);
	bool send_command_2phase(uint8_t req_cmd, uint8_t write_cmd,
	                         const uint8_t *data, uint16_t len);

	// DFU low-level
	SmdDfuResponse dfu_send_command(uint8_t cmd, const uint8_t *data = nullptr, uint16_t data_len = 0,
	                                uint8_t *response_data = nullptr, uint16_t *response_len = nullptr);
	SmdDfuResponse dfu_send_with_retry(uint8_t cmd, const uint8_t *data = nullptr, uint16_t data_len = 0,
	                                   uint8_t *response_data = nullptr, uint16_t *response_len = nullptr);
	SmdDfuResponse dfu_ping_bl();
	SmdDfuResponse dfu_get_info(SmdDfuInfo *info);
	SmdDfuResponse dfu_reset();
	SmdDfuResponse dfu_get_status(uint8_t *status);
	SmdDfuResponse dfu_abort();
	SmdDfuResponse dfu_set_header(const uint8_t *header);
};
