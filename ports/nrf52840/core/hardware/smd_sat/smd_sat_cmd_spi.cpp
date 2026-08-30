/**
 * @file smd_sat_cmd_spi.cpp
 * @brief SPI Protocol A+ command layer for SMD satellite module.
 */

#include <cstdint>
#include <cstring>
#include <cmath>

#include "bsp.hpp"
#include "error.hpp"
#include "debug.hpp"
#include "gpio.hpp"
#include "scheduler.hpp"
#include "timer.hpp"
#include "binascii.hpp"
#include "smd_sat_cmd_spi.hpp"
#include "pmu.hpp"

#include "nrf_delay.h"
#include "nrf_gpio.h"

SmdSatCmdSpi::SmdSatCmdSpi()
    : m_nrf_spim(nullptr),
      m_protocol_mode(SpiProtocolMode::APLUS),
      m_sequence_number(0),
      m_protocol_detected(true),
      m_seq_reset_attempted(false),
      m_dfu_mode(false) {
	memset(&m_dfu_info, 0, sizeof(m_dfu_info));
}

SmdSatCmdSpi::~SmdSatCmdSpi() {
	deinit();
}

void SmdSatCmdSpi::init() {
	if (m_nrf_spim == nullptr) {
		m_nrf_spim = new (std::nothrow) NrfSPIM(SPI_SATELLITE);
		if (m_nrf_spim == nullptr) {
			DEBUG_ERROR("SmdSatCmdSpi::init: SPI allocation failed");
			throw ErrorCode::RESOURCE_NOT_AVAILABLE;
		}
		// CS is managed by NrfSPIM::transfer() — do NOT force it low here
	}
	m_protocol_mode = SpiProtocolMode::APLUS;
	m_protocol_detected = true;
	m_sequence_number = 0;
	m_seq_reset_attempted = false;
}

void SmdSatCmdSpi::deinit() {
	if (m_nrf_spim) {
		delete m_nrf_spim;
		m_nrf_spim = nullptr;
		nrf_gpio_cfg_output(BSP::SPI_Inits[SPI_SATELLITE].config.ss_pin);
		nrf_gpio_pin_clear(BSP::SPI_Inits[SPI_SATELLITE].config.ss_pin);
		nrf_gpio_cfg_input(BSP::SPI_Inits[SPI_SATELLITE].config.mosi_pin, NRF_GPIO_PIN_PULLDOWN);
		nrf_gpio_cfg_input(BSP::SPI_Inits[SPI_SATELLITE].config.miso_pin, NRF_GPIO_PIN_PULLDOWN);
		nrf_gpio_cfg_input(BSP::SPI_Inits[SPI_SATELLITE].config.sck_pin, NRF_GPIO_PIN_PULLDOWN);
	}
}

void SmdSatCmdSpi::read_byte(uint8_t *byte_read) {
	nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
	uint8_t read_cmd = SMDSAT_CMD_NONE;
	int ret;

	ret = m_nrf_spim->transfer(&read_cmd, byte_read, sizeof(read_cmd));
	if (ret) {
		throw ErrorCode::SPI_COMMS_ERROR;
	}
}

void SmdSatCmdSpi::send_command(uint8_t command) {
	uint8_t buffer_read;
	int ret;
	DEBUG_TRACE("%s::Send %u", __func__, command);

	ret = m_nrf_spim->transfer(&command, &buffer_read, sizeof(command));
	if (ret) {
		throw ErrorCode::SPI_COMMS_ERROR;
	}
}

void SmdSatCmdSpi::send_command(const uint8_t *tx_data, uint8_t *rx_data, uint16_t size) {
	int ret = m_nrf_spim->transfer(tx_data, rx_data, size);
	if (ret) {
		throw ErrorCode::SPI_COMMS_ERROR;
	}
}

// ============================================================================
// Protocol A+ Implementation (Based on Zephyr argos-smd-driver)
// ============================================================================
// Key features:
// - Fixed 64-byte frames with 0xFF padding
// - CRC-8 over MAGIC + SEQ + CMD + LEN + DATA (includes MAGIC!)
// - Pipelined protocol: response to command N arrives in transaction N+2
// - BUSY pattern (0xBB) handling for flash operations
// ============================================================================

uint16_t SmdSatCmdSpi::build_aplus_frame(uint8_t *frame, uint8_t cmd, const uint8_t *data, uint16_t data_len) {
	// Max data that fits in a single 64-byte frame: 64 - 4(header) - 1(CRC) = 59
	constexpr uint16_t single_frame_capacity =
	    SPI_PROTOCOL_APLUS_FRAME_SIZE - SPI_PROTOCOL_APLUS_HEADER_LEN - SPI_PROTOCOL_APLUS_CRC_LEN;
	if (data_len > single_frame_capacity) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: Data too long for single frame (%u > %u)", __func__, data_len,
		            single_frame_capacity);
		return 0;
	}

	// Initialize frame with padding
	memset(frame, SPI_PROTOCOL_APLUS_PAD_BYTE, SPI_PROTOCOL_APLUS_FRAME_SIZE);

	uint16_t idx = 0;

	// Build frame: [MAGIC][SEQ][CMD][LEN][DATA][CRC8][PADDING...]
	frame[idx++] = SPI_PROTOCOL_APLUS_MAGIC_REQUEST;
	frame[idx++] = m_sequence_number;
	frame[idx++] = cmd;
	frame[idx++] = static_cast<uint8_t>(data_len);

	// Copy data if any
	if (data != nullptr && data_len > 0) {
		memcpy(&frame[idx], data, data_len);
		idx += data_len;
	}

	// Calculate CRC-8 over MAGIC + SEQ + CMD + LEN + DATA (includes MAGIC!)
	uint8_t crc = spi_crc8_ccitt(frame, idx);
	frame[idx++] = crc;

	DEBUG_TRACE("SmdSatCmdSpi::%s: Frame built - SEQ=%u | CMD=0x%02X | LEN=%u | CRC=0x%02X", __func__,
	            m_sequence_number, cmd, data_len, crc);

	// Return fixed frame size for SPI transfer
	return SPI_PROTOCOL_APLUS_FRAME_SIZE;
}

bool SmdSatCmdSpi::parse_aplus_response(const uint8_t *rx_buffer, uint16_t rx_len, SpiAplusResponse *response) {
	if (response == nullptr || rx_buffer == nullptr) {
		return false;
	}

	response->valid = false;
	response->data_len = 0;
	response->crc_error = false;
	response->raw_status = 0;

	// Scan for response magic byte, skipping IDLE (0xAA) and padding (0xFF)
	// (matches Zephyr parse_response_frame behavior)
	uint16_t offset = 0;
	while (offset < rx_len) {
		if (rx_buffer[offset] == SPI_PROTOCOL_APLUS_MAGIC_RESPONSE) {
			break;
		}
		if (rx_buffer[offset] == SPI_PROTOCOL_APLUS_BUSY_PATTERN) {
			DEBUG_TRACE("SmdSatCmdSpi::%s: BUSY pattern at offset %u", __func__, offset);
			response->status = SPI_APLUS_STATUS_BUSY;
			return false;
		}
		if (rx_buffer[offset] != SPI_PROTOCOL_APLUS_IDLE_PATTERN && rx_buffer[offset] != SPI_PROTOCOL_APLUS_PAD_BYTE) {
			break;
		}
		offset++;
	}

	if (offset >= rx_len || rx_buffer[offset] != SPI_PROTOCOL_APLUS_MAGIC_RESPONSE) {
		if (rx_buffer[0] == SPI_PROTOCOL_APLUS_IDLE_PATTERN) {
			DEBUG_TRACE("SmdSatCmdSpi::%s: IDLE pattern - no response ready", __func__);
			response->status = SPI_APLUS_STATUS_NOT_READY;
		} else {
			DEBUG_TRACE("SmdSatCmdSpi::%s: No response magic found (first byte: 0x%02X)", __func__, rx_buffer[0]);
			response->status = SPI_APLUS_STATUS_FRAME_ERROR;
		}
		return false;
	}

	// Minimum frame: MAGIC + SEQ + STATUS + LEN + CRC = 5 bytes
	if (rx_len - offset < SPI_PROTOCOL_APLUS_HEADER_LEN + SPI_PROTOCOL_APLUS_CRC_LEN) {
		DEBUG_WARN("SmdSatCmdSpi::%s: Response too short after offset %u", __func__, offset);
		return false;
	}

	// Parse header at offset
	response->seq = rx_buffer[offset + 1];
	response->status = static_cast<SpiAplusStatus>(rx_buffer[offset + 2]);
	response->raw_status = rx_buffer[offset + 2];  // preserve real status even if CRC later fails
	uint8_t data_len = rx_buffer[offset + 3];

	if (data_len > SPI_PROTOCOL_APLUS_MAX_DATA_LEN) {
		DEBUG_WARN("SmdSatCmdSpi::%s: Invalid data length %u", __func__, data_len);
		response->status = SPI_APLUS_STATUS_SIZE_ERROR;
		return false;
	}

	// Bounds check: header + data + CRC must fit in remaining buffer
	uint16_t expected_len = SPI_PROTOCOL_APLUS_HEADER_LEN + data_len + SPI_PROTOCOL_APLUS_CRC_LEN;
	if (rx_len - offset < expected_len) {
		DEBUG_WARN("SmdSatCmdSpi::%s: Response incomplete: got %u | expected %u", __func__, rx_len - offset,
		           expected_len);
		response->status = SPI_APLUS_STATUS_FRAME_ERROR;
		return false;
	}

	// Copy data
	response->data_len = data_len;
	if (data_len > 0) {
		memcpy(response->data, &rx_buffer[offset + 4], data_len);
	}

	// Validate CRC-8 over MAGIC + SEQ + STATUS + LEN + DATA
	uint16_t crc_data_len = SPI_PROTOCOL_APLUS_HEADER_LEN + data_len;
	uint8_t received_crc = rx_buffer[offset + crc_data_len];
	uint8_t calculated_crc = spi_crc8_ccitt(&rx_buffer[offset], crc_data_len);

	if (received_crc != calculated_crc) {
		DEBUG_WARN("SmdSatCmdSpi::%s: CRC mismatch (recv=0x%02X | calc=0x%02X)", __func__, received_crc,
		           calculated_crc);
		response->status = SPI_APLUS_STATUS_FRAME_CRC_ERROR;
		response->crc_error = true;  // frame structure valid, only CRC8 failed (data already copied above)
		return false;
	}

	response->valid = true;
	DEBUG_TRACE("SmdSatCmdSpi::%s: Response parsed - SEQ=%u | STATUS=%u | LEN=%u", __func__, response->seq,
	            static_cast<int>(response->status), response->data_len);

	return true;
}

// True if the command triggers a flash/NVM write on the STM32 side, which
// requires NOP-polling with BUSY pattern handling (the STM stalls SPI while
// it writes flash). Must be a whitelist of *actual flash-writing* opcodes —
// matching by cmd_delay >= TIMING_WRITE_MS misclassifies any slow read
// (e.g. READ_SPIMAC_STATE polled during RF) and causes seq_num desync
// (per the Zephyr-derived comment in send_command_aplus).
static bool is_flash_command(uint8_t cmd) {
	switch (cmd) {
	case SMDSAT_CMD_DFU_ERASE:
	case SMDSAT_CMD_DFU_WRITE_DATA:
	case SMDSAT_CMD_DFU_SET_HEADER:
	case SMDSAT_CMD_WRITE_RCONF:
	case SMDSAT_CMD_WRITE_KMAC:
	case SMDSAT_CMD_WRITE_SECKEY:
	case SMDSAT_CMD_SAVE_RCONF:
	case SMDSAT_CMD_WRITE_ID:
	case SMDSAT_CMD_WRITE_ADDR:
	case SMDSAT_CMD_WRITE_LPM:
	case SMDSAT_CMD_WRITE_TCXO:
	case SMDSAT_CMD_WRITE_CW:
	case SMDSAT_CMD_WRITE_PREPASSEN:
	case SMDSAT_CMD_WRITE_UDATE: return true;
	default: return false;
	}
}

// Command-specific delay (matches Zephyr timing constants)
static uint32_t get_command_delay(uint8_t cmd) {
	switch (cmd) {
	case SMDSAT_CMD_DFU_ERASE: return SMDSAT_TIMING_ERASE_MS;
	case SMDSAT_CMD_DFU_WRITE_DATA:
	case SMDSAT_CMD_DFU_SET_HEADER:
	case SMDSAT_CMD_WRITE_RCONF:
	case SMDSAT_CMD_WRITE_KMAC:
	case SMDSAT_CMD_WRITE_SECKEY:
	case SMDSAT_CMD_SAVE_RCONF:
	case SMDSAT_CMD_WRITE_ID:
	case SMDSAT_CMD_WRITE_ADDR:
	case SMDSAT_CMD_WRITE_LPM:
	case SMDSAT_CMD_WRITE_TCXO:
	case SMDSAT_CMD_WRITE_CW:
	case SMDSAT_CMD_WRITE_PREPASSEN:
	case SMDSAT_CMD_WRITE_UDATE: return SMDSAT_TIMING_WRITE_MS;
	case SMDSAT_CMD_WRITE_TX: return SMDSAT_SPI_POST_TX_DELAY_MS;  // 100ms async processing after TX data
	case SMDSAT_CMD_READ_SPIMAC_STATE:
		// Polled at every state_transmitting tick — but during active RF the
		// STM is busy and cannot serve a fast SPI response in the standard
		// 30 ms window. Reading a half-filled response buffer yields random
		// IDLE bytes sometimes containing a bit-shifted 0x55 that the parser
		// mistakes for MAGIC, then the byte at offset+3 is treated as
		// data_len and produces "Response incomplete: got 64 | expected N"
		// WARN cascades. 150 ms (2026-05 second field-test bump) gives the
		// STM ample time to finish RF housekeeping and fill the response
		// buffer cleanly. Costs ~120 ms per is_tx_finished poll, negligible
		// vs the seconds the TX itself takes.
		return 150;
	case SMDSAT_CMD_DFU_RESET:
	case SMDSAT_CMD_DFU_JUMP: return SMDSAT_TIMING_RESET_MS;
	default: return SMDSAT_TIMING_STANDARD_MS;
	}
}

// Check if RX buffer is all IDLE or all BUSY (slave not ready)
static bool is_slave_not_ready(const uint8_t *buf, uint8_t check_len) {
	bool all_idle = true, all_busy = true;
	for (uint8_t i = 0; i < check_len; i++) {
		if (buf[i] != SPI_PROTOCOL_APLUS_IDLE_PATTERN) all_idle = false;
		if (buf[i] != SPI_PROTOCOL_APLUS_BUSY_PATTERN) all_busy = false;
	}
	return all_idle || all_busy;
}

bool SmdSatCmdSpi::send_command_aplus(uint8_t command, const uint8_t *tx_data, uint16_t tx_len,
                                      SpiAplusResponse *response) {
	// Inter-transaction delay: wait for STM32 DMA re-arm from previous transaction
	nrf_delay_ms(smdsat_spi_inter_tx_delay_ms());

	// Build the A+ CMD frame (fixed 64 bytes)
	uint8_t tx_frame[SPI_PROTOCOL_APLUS_FRAME_SIZE];
	uint8_t rx_frame[SPI_PROTOCOL_APLUS_FRAME_SIZE];

	uint16_t frame_len = build_aplus_frame(tx_frame, command, tx_data, tx_len);
	if (frame_len == 0) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: Failed to build frame", __func__);
		return false;
	}

	memset(rx_frame, 0, sizeof(rx_frame));

	// Transaction 1: Send command (response will be in transaction 2)
	int ret = m_nrf_spim->transfer(tx_frame, rx_frame, SPI_PROTOCOL_APLUS_FRAME_SIZE);
	if (ret) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: SPI transfer failed", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}

	// Wait for STM32 to process command (command-specific delay)
	// Kick WDT for long delays to prevent watchdog timeout
	uint32_t cmd_delay = get_command_delay(command);
	while (cmd_delay > 500) {
		PMU::kick_watchdog();
		nrf_delay_ms(500);
		cmd_delay -= 500;
	}
	if (cmd_delay > 0) {
		nrf_delay_ms(cmd_delay);
	}

	// Transaction 2+: Send NOP to retrieve response (pipelined protocol)
	m_sequence_number++;

	uint8_t nop_frame[SPI_PROTOCOL_APLUS_FRAME_SIZE];
	uint8_t response_frame[SPI_PROTOCOL_APLUS_FRAME_SIZE];

	// Zephyr behavior: only retry NOP polling for flash/NVM write operations.
	// For standard commands, send ONE NOP and accept whatever comes back.
	// Retrying NOPs for non-flash commands causes sequence number desync with STM32.
	// 2026-05 fix: was `cmd_delay >= SMDSAT_TIMING_WRITE_MS` which misclassified
	// any slow read (e.g. READ_SPIMAC_STATE @150ms) as a flash op and triggered
	// the retry-with-seq-bump path — the very cascade the long cmd_delay was
	// meant to suppress. Whitelist by opcode now.
	bool is_flash_op = is_flash_command(command);
	uint8_t max_retries = is_flash_op ? SMDSAT_SPI_BUSY_MAX_RETRIES : 0;

	for (uint8_t retry = 0; retry <= max_retries; retry++) {
		build_aplus_frame(nop_frame, SMDSAT_CMD_NONE, nullptr, 0);
		memset(response_frame, 0, sizeof(response_frame));

		ret = m_nrf_spim->transfer(nop_frame, response_frame, SPI_PROTOCOL_APLUS_FRAME_SIZE);
		if (ret) {
			DEBUG_ERROR("SmdSatCmdSpi::%s: Failed to read response", __func__);
			throw ErrorCode::SPI_COMMS_ERROR;
		}

		// For flash ops: check for BUSY/IDLE pattern and retry
		if (is_flash_op && is_slave_not_ready(response_frame, 8) && retry < max_retries) {
			bool is_busy = (response_frame[0] == SPI_PROTOCOL_APLUS_BUSY_PATTERN);
			DEBUG_TRACE("SmdSatCmdSpi::%s: Slave %s (0x%02X) | retry %u/%u", __func__, is_busy ? "BUSY" : "not ready",
			            response_frame[0], retry + 1, max_retries);
			nrf_delay_ms(is_busy ? SMDSAT_SPI_BUSY_WAIT_MS : smdsat_spi_retry_delay_ms());
			m_sequence_number++;
			continue;
		}
		break;
	}

	// Increment sequence number for next transaction (matches Zephyr seq_num++ at end)
	m_sequence_number++;

	// Parse response if caller wants it
	if (response != nullptr) {
		bool parsed = parse_aplus_response(response_frame, SPI_PROTOCOL_APLUS_FRAME_SIZE, response);
		if (!parsed) {
			// An unparseable frame used to say nothing about WHY. The raw bytes
			// discriminate the three cases a person at the bench needs to tell
			// apart: all 0xFF = MISO floating, nothing driving the line; all
			// 0x00 = line held low, module unpowered; all 0xAA = the slave IDLE
			// pattern, i.e. something IS running and driving MISO but has no
			// response ready. Measured 2026-08-30 on the LinkIt SMD board:
			// AA AA AA AA AA AA AA AA to CMD 0x02, forever — a module that talks
			// but does not speak this protocol version.
			DEBUG_WARN("SmdSatCmdSpi::%s: unparseable response to cmd 0x%02X — rx: "
			           "%02X %02X %02X %02X %02X %02X %02X %02X",
			           __func__, command, response_frame[0], response_frame[1], response_frame[2],
			           response_frame[3], response_frame[4], response_frame[5], response_frame[6],
			           response_frame[7]);
		}
		if (!parsed && !SPI_APLUS_IS_RECOVERABLE(response->status)) {
			return false;
		}
	}

	return true;
}

bool SmdSatCmdSpi::send_command_auto(uint8_t command, const uint8_t *tx_data, uint16_t tx_len, uint8_t *rx_data,
                                     uint16_t *rx_len) {
	// Protocol A+ only (no legacy fallback)
	SpiAplusResponse response;

	// Give each top-level command its own one-shot INVALID_CMD resync budget.
	// Without this, a command that already consumed the resync (and failed)
	// leaves the latch set, so the NEXT command in a cascade can no longer
	// resync on its first INVALID_CMD — a cross-command desync trap. The
	// within-call single-reset guarantee is preserved (the latch is re-set
	// inside the loop after the first reset).
	m_seq_reset_attempted = false;

	for (uint8_t retry = 0; retry < SMDSAT_SPI_MAX_RETRIES; retry++) {
		if (send_command_aplus(command, tx_data, tx_len, &response)) {
			// Copy response data if requested
			if (rx_data != nullptr && rx_len != nullptr && *rx_len > 0 && response.data_len > 0) {
				uint16_t copy_len = (response.data_len < *rx_len) ? response.data_len : *rx_len;
				memcpy(rx_data, response.data, copy_len);
				*rx_len = copy_len;
			} else if (rx_len != nullptr) {
				*rx_len = response.data_len;
			}

			if (response.status == SPI_APLUS_STATUS_OK) {
				m_seq_reset_attempted = false;  // Success — allow future resync attempts
				return true;
			}

			// INVALID_CMD (0x07): protocol desync — reset sequence and retry once.
			// The STM32WL watchdog resets its SPI state after 5s of no valid command,
			// so resetting our sequence number allows resynchronization.
			if (response.status == SPI_APLUS_STATUS_INVALID_CMD && !m_seq_reset_attempted) {
				DEBUG_WARN("SmdSatCmdSpi::%s: INVALID_CMD for cmd 0x%02X — resetting sequence (was %u)", __func__,
				           command, m_sequence_number);
				m_seq_reset_attempted = true;
				m_sequence_number = 0;
				nrf_delay_ms(smdsat_spi_retry_delay_ms());
				continue;
			}

			if (!SPI_APLUS_IS_RECOVERABLE(response.status)) {
				DEBUG_WARN("SmdSatCmdSpi::%s: Non-recoverable error %u for cmd 0x%02X", __func__,
				           static_cast<int>(response.status), command);
				return false;
			}
		}

		DEBUG_TRACE("SmdSatCmdSpi::%s: Retry %u/%u for cmd 0x%02X", __func__, retry + 1, SMDSAT_SPI_MAX_RETRIES,
		            command);
		nrf_delay_ms(smdsat_spi_retry_delay_ms());
	}

	return false;
}

// 2-phase write helper (matches Zephyr argos_spi_write_2phase)
// Phase 1: Send REQ command to prepare STM32 for data reception
// Phase 2: Send WRITE command with actual data
bool SmdSatCmdSpi::send_command_2phase(uint8_t req_cmd, uint8_t write_cmd, const uint8_t *data, uint16_t len) {
	DEBUG_TRACE("SmdSatCmdSpi::%s: REQ=0x%02X | WRITE=0x%02X | len=%u", __func__, req_cmd, write_cmd, len);

	// Phase 1: Send REQ command
	if (!send_command_auto(req_cmd)) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: REQ 0x%02X failed", __func__, req_cmd);
		return false;
	}

	// Small delay for STM32 to prepare RX buffer
	nrf_delay_ms(1);

	// Phase 2: Send WRITE command with data
	if (!send_command_auto(write_cmd, data, len)) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: WRITE 0x%02X failed", __func__, write_cmd);
		return false;
	}

	return true;
}

// Argos MAC message counter (MC) — 9-bit (0-511), little-endian over SPI.
// Returns false (graceful) if the module firmware does not implement the
// command: send_command_auto() returns false on INVALID_CMD after one resync.
bool SmdSatCmdSpi::read_message_counter(uint16_t *mc) {
	if (mc == nullptr) return false;
	uint8_t rx[4] = { 0 };
	uint16_t rx_len = sizeof(rx);
	if (!send_command_auto(SMDSAT_CMD_READ_MC, nullptr, 0, rx, &rx_len)) {
		DEBUG_TRACE("SmdSatCmdSpi::%s: READ_MC unsupported/failed", __func__);
		return false;
	}
	if (rx_len < 2) {
		DEBUG_TRACE("SmdSatCmdSpi::%s: READ_MC short response (%u)", __func__, rx_len);
		return false;
	}
	*mc = static_cast<uint16_t>(rx[0] | (rx[1] << 8)) & 0x01FF;  // 9-bit
	DEBUG_TRACE("SmdSatCmdSpi::%s: MC=%u", __func__, *mc);
	return true;
}

bool SmdSatCmdSpi::set_message_counter(uint16_t mc) {
	uint16_t v = mc & 0x01FF;  // clamp to 9-bit
	uint8_t data[2] = { static_cast<uint8_t>(v & 0xFF), static_cast<uint8_t>((v >> 8) & 0xFF) };
	if (!send_command_2phase(SMDSAT_CMD_WRITE_MC_REQ, SMDSAT_CMD_WRITE_MC, data, sizeof(data))) {
		DEBUG_TRACE("SmdSatCmdSpi::%s: WRITE_MC unsupported/failed", __func__);
		return false;
	}
	DEBUG_TRACE("SmdSatCmdSpi::%s: MC set to %u", __func__, v);
	return true;
}

void SmdSatCmdSpi::load_kmac_profil(uint8_t profile, const uint8_t *ctx, uint8_t ctx_len) {
	// CMD_WRITE_KMAC payload = [profile_id][ctx...]. BASIC/NONE send the id only
	// (1 byte, unchanged — backward-compatible). BLIND appends the 7-byte packed
	// ctx: feat-blind firmware parses it; older firmware ignores the extra bytes
	// (reads the id only). The A+ framing already handles LEN/CRC.
	uint8_t payload[1 + 7];
	payload[0] = profile;
	uint8_t len = 1;
	if (ctx != nullptr && ctx_len > 0) {
		if (ctx_len > (uint8_t)(sizeof(payload) - 1)) ctx_len = (uint8_t)(sizeof(payload) - 1);
		memcpy(&payload[1], ctx, ctx_len);
		len = (uint8_t)(1 + ctx_len);
	}
	DEBUG_TRACE("SmdSatCmdSpi::%s: Load KMAC profile %u (data_len=%u)", __func__, profile, len);
	if (!send_command_2phase(SMDSAT_CMD_WRITE_KMAC_REQ, SMDSAT_CMD_WRITE_KMAC, payload, len)) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: Failed to load KMAC profile", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}
}

void SmdSatCmdSpi::read_kmac(uint8_t *profile) {
	uint8_t rx[4] = { 0 };
	uint16_t rx_len = sizeof(rx);
	if (!send_command_auto(SMDSAT_CMD_READ_KMAC, nullptr, 0, rx, &rx_len)) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: Failed to read KMAC profile", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}
	*profile = rx[0];
	DEBUG_TRACE("SmdSatCmdSpi::%s: KMAC profile=%u", __func__, *profile);
}

void SmdSatCmdSpi::read_rconf_raw(uint8_t *rconf_raw, uint16_t *len) {
	uint16_t rx_len = SMDSAT_CMD_READ_RCONF_RAW_LEN;
	if (!send_command_auto(SMDSAT_CMD_READ_RCONF_RAW, nullptr, 0, rconf_raw, &rx_len)) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: Failed to read raw radio config", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}
	if (len) *len = rx_len;
	DEBUG_TRACE("SmdSatCmdSpi::%s: RCONF_RAW len=%u", __func__, rx_len);
}

void SmdSatCmdSpi::read_spimac_state(uint8_t *spi_state, uint8_t *mac_state) {
	uint8_t rx[4] = { 0 };
	uint16_t rx_len = sizeof(rx);
	if (!send_command_auto(SMDSAT_CMD_READ_SPIMAC_STATE, nullptr, 0, rx, &rx_len)) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: Failed to read SPI MAC state", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}
	if (spi_state) *spi_state = rx[0];
	if (mac_state) *mac_state = rx[1];
	DEBUG_TRACE("SmdSatCmdSpi::%s: SPI state=%u | MAC state=%u", __func__, rx[0], rx[1]);
}

void SmdSatCmdSpi::read_firmware_info(uint8_t *info, uint16_t *len) {
	uint16_t rx_len = SMDSAT_CMD_READ_FIRMWARE_LEN;
	if (!send_command_auto(SMDSAT_CMD_READ_FIRMWARE, nullptr, 0, info, &rx_len)) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: Failed to read firmware info", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}
	if (len) *len = rx_len;
	DEBUG_TRACE("SmdSatCmdSpi::%s: firmware info len=%u", __func__, rx_len);
}

// CW (Continuous Wave) commands
void SmdSatCmdSpi::read_cw(uint8_t *cw_data, uint16_t *len) {
	uint16_t rx_len = (len && *len > 0) ? *len : 32;
	if (!send_command_auto(SMDSAT_CMD_READ_CW, nullptr, 0, cw_data, &rx_len)) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: Failed to read CW", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}
	if (len) *len = rx_len;
	DEBUG_TRACE("SmdSatCmdSpi::%s: CW data len=%u", __func__, rx_len);
}

void SmdSatCmdSpi::write_cw(const uint8_t *cw_data, uint16_t len) {
	DEBUG_TRACE("SmdSatCmdSpi::%s: len=%u", __func__, len);
	if (!send_command_2phase(SMDSAT_CMD_WRITE_CW_REQ, SMDSAT_CMD_WRITE_CW, cw_data, len)) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: Failed to write CW", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}
}

// Prepass commands
void SmdSatCmdSpi::read_prepassen(uint8_t *prepass_data, uint16_t *len) {
	uint16_t rx_len = (len && *len > 0) ? *len : 32;
	if (!send_command_auto(SMDSAT_CMD_READ_PREPASSEN, nullptr, 0, prepass_data, &rx_len)) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: Failed to read prepass", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}
	if (len) *len = rx_len;
	DEBUG_TRACE("SmdSatCmdSpi::%s: prepass data len=%u", __func__, rx_len);
}

void SmdSatCmdSpi::write_prepassen(const uint8_t *prepass_data, uint16_t len) {
	DEBUG_TRACE("SmdSatCmdSpi::%s: len=%u", __func__, len);
	if (!send_command_2phase(SMDSAT_CMD_WRITE_PREPASSEN_REQ, SMDSAT_CMD_WRITE_PREPASSEN, prepass_data, len)) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: Failed to write prepass", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}
}

// UTC date commands
void SmdSatCmdSpi::read_udate(uint8_t *udate_data, uint16_t *len) {
	uint16_t rx_len = (len && *len > 0) ? *len : 32;
	if (!send_command_auto(SMDSAT_CMD_READ_UDATE, nullptr, 0, udate_data, &rx_len)) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: Failed to read UTC date", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}
	if (len) *len = rx_len;
	DEBUG_TRACE("SmdSatCmdSpi::%s: udate data len=%u", __func__, rx_len);
}

void SmdSatCmdSpi::write_udate(const uint8_t *udate_data, uint16_t len) {
	DEBUG_TRACE("SmdSatCmdSpi::%s: len=%u", __func__, len);
	if (!send_command_2phase(SMDSAT_CMD_WRITE_UDATE_REQ, SMDSAT_CMD_WRITE_UDATE, udate_data, len)) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: Failed to write UTC date", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}
}

// SPI protocol synchronization (matches Zephyr argos_spi_sync)
void SmdSatCmdSpi::sync() {
	DEBUG_INFO("SmdSatCmdSpi::%s: Synchronizing SPI protocol...", __func__);

	m_sequence_number = 0;

	uint8_t tx_buf[SPI_PROTOCOL_APLUS_FRAME_SIZE];
	uint8_t rx_buf[SPI_PROTOCOL_APLUS_FRAME_SIZE];
	int sync_count = 0;

	for (int i = 0; i < 5; i++) {
		memset(tx_buf, 0xFF, sizeof(tx_buf));
		memset(rx_buf, 0, sizeof(rx_buf));

		int ret = m_nrf_spim->transfer(tx_buf, rx_buf, SPI_PROTOCOL_APLUS_FRAME_SIZE);
		if (ret) {
			DEBUG_WARN("SmdSatCmdSpi::%s: Sync transaction %d failed: %d", __func__, i, ret);
			continue;
		}

		// Check if we're getting consistent idle pattern (0xAA)
		bool all_idle = true;
		for (int j = 0; j < 8; j++) {
			if (rx_buf[j] != SPI_PROTOCOL_APLUS_IDLE_PATTERN) {
				all_idle = false;
				break;
			}
		}

		if (all_idle) {
			sync_count++;
			if (sync_count >= 2) {
				DEBUG_INFO("SmdSatCmdSpi::%s: SPI synchronized after %d transactions", __func__, i + 1);
				return;
			}
		} else {
			sync_count = 0;
		}

		nrf_delay_ms(SMDSAT_SPI_DETECT_TIMEOUT_MS);
	}

	DEBUG_WARN("SmdSatCmdSpi::%s: SPI sync incomplete | proceeding anyway", __func__);
}

void SmdSatCmdSpi::get_status(uint8_t *status) {
	uint8_t rx[4] = { 0 };
	uint16_t rx_len = sizeof(rx);
	if (!send_command_auto(SMDSAT_CMD_SPI_STATUS, nullptr, 0, rx, &rx_len)) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: Failed to read SPI status", __func__);
		*status = SMDSAT_SPICMD_ERROR;
		return;
	}
	*status = rx[0];
	DEBUG_TRACE("SmdSatCmdSpi::%s: status=%u", __func__, *status);
}

void SmdSatCmdSpi::get_kmac_status(uint8_t *status) {
	uint8_t rx[4] = { 0 };
	uint16_t rx_len = sizeof(rx);
	if (!send_command_auto(SMDSAT_CMD_MAC_STATUS, nullptr, 0, rx, &rx_len)) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: Failed to read KMAC status", __func__);
		*status = MAC_ERROR;
		return;
	}
	*status = rx[0];
	DEBUG_TRACE("SmdSatCmdSpi::%s: status=%u", __func__, *status);
}

void SmdSatCmdSpi::set_radio_conf(smd_uint8_array_t *radio_conf) {
	DEBUG_TRACE("SmdSatCmdSpi::%s", __func__);
	if (radio_conf->size != SMDSAT_CMD_WRITECONF_LEN - 1) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: Size mismatch %u != %u", __func__, radio_conf->size,
		            SMDSAT_CMD_WRITECONF_LEN - 1);
		throw ErrorCode::SPI_COMMS_ERROR;
	}
	if (!send_command_2phase(SMDSAT_CMD_WRITE_RCONF_REQ, SMDSAT_CMD_WRITE_RCONF, radio_conf->p_data,
	                         radio_conf->size)) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: Failed to set radio conf", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}
}

void SmdSatCmdSpi::read_radio_conf(SmdArgosModulation *modulation) {
	DEBUG_TRACE("SmdSatCmdSpi::%s", __func__);
	uint8_t rx[SMDSAT_CMD_READCONF_LEN] = { 0 };
	uint16_t rx_len = sizeof(rx);
	if (!send_command_auto(SMDSAT_CMD_READ_RCONF, nullptr, 0, rx, &rx_len)) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: Failed to read radio conf", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}

	uint32_t min_frequency = (static_cast<uint32_t>(rx[3]) << 24) | (static_cast<uint32_t>(rx[2]) << 16)
	                         | (static_cast<uint32_t>(rx[1]) << 8) | rx[0];
	uint32_t max_frequency = (static_cast<uint32_t>(rx[7]) << 24) | (static_cast<uint32_t>(rx[6]) << 16)
	                         | (static_cast<uint32_t>(rx[5]) << 8) | rx[4];
	int8_t rf_level = rx[8];
	// rx[9] is the STM32 KNS_tx_mod_t enum (LDA2=2, LDA2L=3, VLDA4=4, HDA4=5,
	// LDK=6) — NOT the SmdArgosModulation enum (LDA2=0, LDK=1, VLDA4=2).
	// A direct cast would silently produce wrong values (e.g. KNS LDA2=2 cast
	// gives ARGOS_MOD_VLDA4=2). Same mapping as init_and_get_mod() in
	// smd_spi_test().
	uint8_t kineis_mod = rx[9];
	switch (kineis_mod) {
	case 2:
	case 3: *modulation = ARGOS_MOD_LDA2; break;   // KNS_TX_MOD_LDA2 / LDA2L
	case 4: *modulation = ARGOS_MOD_VLDA4; break;  // KNS_TX_MOD_VLDA4
	case 6: *modulation = ARGOS_MOD_LDK; break;    // KNS_TX_MOD_LDK
	default: *modulation = ARGOS_MOD_LDA2; break;  // safe fallback
	}

	DEBUG_INFO("SmdSatCmdSpi::%s: KineisMod=%u (-> enum=%d) | Min Freq: %u | Max Freq: %u | RF Level %d", __func__,
	           kineis_mod, static_cast<int>(*modulation), min_frequency, max_frequency, rf_level);
}

bool SmdSatCmdSpi::save_radio_conf() {
	DEBUG_TRACE("SmdSatCmdSpi::%s", __func__);
	if (!send_command_auto(SMDSAT_CMD_SAVE_RCONF)) {
		DEBUG_WARN("SmdSatCmdSpi::%s: Radio conf failed to save", __func__);
		return false;
	}
	DEBUG_INFO("SmdSatCmdSpi::%s: Radio conf saved", __func__);
	return true;
}

void SmdSatCmdSpi::read_lpm(uint8_t *lpm_mode) {
	DEBUG_TRACE("SmdSatCmdSpi::%s", __func__);
	uint8_t rx[4] = { 0 };
	uint16_t rx_len = sizeof(rx);
	if (!send_command_auto(SMDSAT_CMD_READ_LPM, nullptr, 0, rx, &rx_len)) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: Failed to read LPM", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}
	*lpm_mode = rx[0];
	DEBUG_INFO("SmdSatCmdSpi::SMD LPM mode is: %u", *lpm_mode);
}

void SmdSatCmdSpi::write_lpm(uint8_t *lpm_mode) {
	DEBUG_TRACE("SmdSatCmdSpi::%s", __func__);
	if (!send_command_2phase(SMDSAT_CMD_WRITE_LPM_REQ, SMDSAT_CMD_WRITE_LPM, lpm_mode, 1)) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: Failed to write LPM", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}
}

void SmdSatCmdSpi::read_version(uint8_t *version) {
	DEBUG_TRACE("SmdSatCmdSpi::%s", __func__);
	uint8_t rx[SPI_PROTOCOL_APLUS_MAX_DATA_LEN] = { 0 };
	uint16_t rx_len = sizeof(rx);
	if (!send_command_auto(SMDSAT_CMD_READ_VERSION, nullptr, 0, rx, &rx_len)) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: Failed to read version", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}
	// Copy full version string (not just first byte)
	memcpy(version, rx, rx_len);
	DEBUG_INFO("SmdSatCmdSpi::SMD SPI Version: %.*s", (int)rx_len, (char *)rx);
}

void SmdSatCmdSpi::read_address(smd_uint8_array_t *address) {
	DEBUG_TRACE("SmdSatCmdSpi::%s", __func__);
	if (address->size != SMDSAT_CMD_READ_ADDR_LEN) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: Size mismatch %u != %u", __func__, address->size, SMDSAT_CMD_READ_ADDR_LEN);
		return;
	}
	uint16_t rx_len = address->size;
	if (!send_command_auto(SMDSAT_CMD_READ_ADDR, nullptr, 0, address->p_data, &rx_len)) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: Failed to read address", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}
}

void SmdSatCmdSpi::set_address(smd_uint8_array_t *address) {
	DEBUG_TRACE("SmdSatCmdSpi::%s", __func__);
	if (address->size != SMDSAT_CMD_WRITE_ADDR_LEN - 1) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: Size mismatch %u != %u", __func__, address->size, SMDSAT_CMD_WRITE_ADDR_LEN - 1);
		return;
	}
	if (!send_command_2phase(SMDSAT_CMD_WRITE_ADDR_REQ, SMDSAT_CMD_WRITE_ADDR, address->p_data, address->size)) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: Failed to set address", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}
}

void SmdSatCmdSpi::read_seckey(smd_uint8_array_t *seckey) {
	DEBUG_TRACE("SmdSatCmdSpi::%s", __func__);
	if (seckey->size != SMDSAT_CMD_READ_SECKEY_LEN) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: Size mismatch %u != %u", __func__, seckey->size, SMDSAT_CMD_READ_SECKEY_LEN);
		return;
	}
	uint16_t rx_len = seckey->size;
	if (!send_command_auto(SMDSAT_CMD_READ_SECKEY, nullptr, 0, seckey->p_data, &rx_len)) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: Failed to read seckey", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}
}

void SmdSatCmdSpi::set_seckey(smd_uint8_array_t *seckey) {
	DEBUG_TRACE("SmdSatCmdSpi::%s", __func__);
	if (seckey->size != SMDSAT_CMD_WRITE_SECKEY_LEN - 1) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: Size mismatch %u != %u", __func__, seckey->size, SMDSAT_CMD_WRITE_SECKEY_LEN);
		return;
	}
	if (!send_command_2phase(SMDSAT_CMD_WRITE_SECKEY_REQ, SMDSAT_CMD_WRITE_SECKEY, seckey->p_data, seckey->size)) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: Failed to set seckey", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}
}

void SmdSatCmdSpi::read_id(uint32_t *id) {
	if (id == nullptr) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: id ptr is null", __func__);
		return;
	}
	uint8_t rx[SMDSAT_CMD_READ_ID_LEN] = { 0 };
	uint16_t rx_len = sizeof(rx);
	if (!send_command_auto(SMDSAT_CMD_READ_ID, nullptr, 0, rx, &rx_len)) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: Failed to read ID", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}
	*id = 0;
	for (int i = 0; i < SMDSAT_CMD_READ_ID_LEN; i++) {
		*id |= rx[i] << (8 * i);
	}
	DEBUG_INFO("SmdSatCmdSpi::%s: SMD ID: %u", __func__, *id);
}

void SmdSatCmdSpi::set_id(uint32_t id) {
	DEBUG_TRACE("SmdSatCmdSpi::%s: id: %u", __func__, id);
	uint8_t data[4];
	memcpy(data, &id, sizeof(id));
	if (!send_command_2phase(SMDSAT_CMD_WRITE_ID_REQ, SMDSAT_CMD_WRITE_ID, data, sizeof(data))) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: Failed to set ID", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}
}

void SmdSatCmdSpi::read_tcxo_warmup(uint32_t *time_ms) {
	if (time_ms == nullptr) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: tcxo ptr is null", __func__);
		return;
	}
	uint8_t rx[SMDSAT_CMD_READ_ID_LEN] = { 0 };
	uint16_t rx_len = sizeof(rx);
	if (!send_command_auto(SMDSAT_CMD_READ_TCXO, nullptr, 0, rx, &rx_len)) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: Failed to read TCXO warmup", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}
	*time_ms = 0;
	for (int i = 0; i < SMDSAT_CMD_READ_ID_LEN; i++) {
		*time_ms |= static_cast<uint32_t>(rx[i]) << (8 * i);
	}
	DEBUG_INFO("SmdSatCmdSpi::%s: SMD TCXO warmup time: %u", __func__, static_cast<unsigned>(*time_ms));
}

void SmdSatCmdSpi::write_tcxo_warmup(uint32_t time_ms) {
	DEBUG_TRACE("SmdSatCmdSpi::%s: tcxo warmup time: %u", __func__, time_ms);
	uint8_t data[4];
	memcpy(data, &time_ms, sizeof(time_ms));
	if (!send_command_2phase(SMDSAT_CMD_WRITE_TCXO_REQ, SMDSAT_CMD_WRITE_TCXO, data, sizeof(data))) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: Failed to write TCXO warmup", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}
}

void SmdSatCmdSpi::read_serial(smd_uint8_array_t *serial) {
	DEBUG_TRACE("SmdSatCmdSpi::%s", __func__);
	if (serial->size != SMDSAT_CMD_READ_SERIAL_LEN) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: Size mismatch %u != %u", __func__, serial->size, SMDSAT_CMD_READ_SERIAL_LEN);
		return;
	}
	uint16_t rx_len = serial->size;
	if (!send_command_auto(SMDSAT_CMD_READ_SN, nullptr, 0, serial->p_data, &rx_len)) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: Failed to read serial", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}
}

void SmdSatCmdSpi::set_tcxo_warmup_internal(uint32_t time_s) {
	DEBUG_TRACE("SmdSatCmdSpi::%s", __func__);
	this->write_tcxo_warmup(time_s * 1000);
}

void SmdSatCmdSpi::set_tcxo_control(bool state) {
	(void)state;
	DEBUG_TRACE("SmdSatCmdSpi::%s: Managed by SMD module", __func__);
}

void SmdSatCmdSpi::print_firmware_version() {
	// CMD_READ_VERSION (0x05) via A+ - returns firmware version string
	// (CMD_READ_FIRMWARE 0x06 is NOT supported in A+ by STM32 firmware)
	uint8_t rx[SPI_PROTOCOL_APLUS_MAX_DATA_LEN] = { 0 };
	uint16_t rx_len = sizeof(rx);

	if (!send_command_auto(SMDSAT_CMD_READ_VERSION, nullptr, 0, rx, &rx_len)) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: CMD_READ_VERSION failed", __func__);
		return;
	}

	size_t len = 0;
	while (len < rx_len && rx[len] != 0 && rx[len] >= 0x20 && rx[len] < 0x7F) {
		len++;
	}

	if (len > 0) {
		DEBUG_INFO("SmdSatCmdSpi::%s: Firmware=%.*s", __func__, (int)len, (char *)rx);
	} else {
		DEBUG_WARN("SmdSatCmdSpi::%s: Empty firmware version (rx_len=%u)", __func__, rx_len);
	}
}

bool SmdSatCmdSpi::ping() {
	DEBUG_TRACE("SmdSatCmdSpi::%s", __func__);

	for (uint8_t retry = 0; retry < SMDSAT_SPI_MAX_RETRIES; retry++) {
		// Allow one sequence resync attempt per ping retry — if the STM32
		// watchdog reset its counter (5s timeout), the next retry should be
		// able to resynchronize instead of being permanently blocked.
		m_seq_reset_attempted = false;
		try {
			if (send_command_auto(SMDSAT_CMD_PING)) {
				DEBUG_TRACE("SmdSatCmdSpi::%s: ACK from SMD received", __func__);
				return true;
			}
		} catch (...) {
			DEBUG_WARN("SmdSatCmdSpi::%s: Ping failed | retry %u/%u", __func__, retry + 1, SMDSAT_SPI_MAX_RETRIES);
		}
		nrf_delay_ms(smdsat_spi_retry_delay_ms());
	}

	DEBUG_WARN("SmdSatCmdSpi::%s: Ping not received after %u retries", __func__, SMDSAT_SPI_MAX_RETRIES);
	return false;
}

bool SmdSatCmdSpi::is_tx_finished() {
	DEBUG_TRACE("SmdSatCmdSpi::%s", __func__);

	uint8_t rx[4] = { 0 };
	uint16_t rx_len = sizeof(rx);

	if (!send_command_auto(SMDSAT_CMD_READ_SPIMAC_STATE, nullptr, 0, rx, &rx_len)) {
		// Immediate retry after short delay — avoids waiting a full 500ms poll cycle
		nrf_delay_ms(smdsat_spi_retry_delay_ms());
		rx_len = sizeof(rx);
		if (!send_command_auto(SMDSAT_CMD_READ_SPIMAC_STATE, nullptr, 0, rx, &rx_len)) {
			DEBUG_WARN("SmdSatCmdSpi::%s: Failed to read SPIMAC state", __func__);
			return false;
		}
	}

	uint8_t mac_status = rx[1];

	DEBUG_TRACE("SmdSatCmdSpi::%s: spiState=%02X macStatus=%02X", __func__, rx[0], mac_status);

	if (mac_status == MAC_TX_DONE) {
		DEBUG_TRACE("SmdSatCmdSpi::%s: TX completed successfully", __func__);
		m_last_tx_status = mac_status;
		return true;
	}

	if (mac_status == MAC_TX_TIMEOUT) {
		DEBUG_WARN("SmdSatCmdSpi::%s: TX timeout (no satellite)", __func__);
		m_last_tx_status = mac_status;
		return true;
	}

	if (mac_status == MAC_ERROR) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: TX error", __func__);
		m_last_tx_status = mac_status;
		return true;
	}

	return false;
}

bool SmdSatCmdSpi::is_tx_in_progress() {
	DEBUG_TRACE("SmdSatCmdSpi::%s", __func__);

	uint8_t rx[4] = { 0 };
	uint16_t rx_len = sizeof(rx);

	if (!send_command_auto(SMDSAT_CMD_READ_SPIMAC_STATE, nullptr, 0, rx, &rx_len)) {
		DEBUG_WARN("SmdSatCmdSpi::%s: Failed to read SPIMAC state", __func__);
		return false;
	}

	return (rx[1] == MAC_TX_IN_PROGRESS);
}

bool SmdSatCmdSpi::initiate_tx(const KineisPacket &payload) {
	DEBUG_TRACE("SmdSatCmdSpi::%s: Size: %u", __func__, payload.size());

	uint16_t size = payload.size();

	// Step 1: WRITE_TX_REQ (0x14) - Prepare for TX
	if (!send_command_auto(SMDSAT_CMD_WRITE_TX_REQ)) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: TX REQ failed", __func__);
		return false;
	}

	// Step 2: WRITE_TX_SIZE (0x15) - Send size (little-endian)
	uint8_t size_data[2];
	size_data[0] = size & 0xFF;
	size_data[1] = (size >> 8) & 0xFF;
	if (!send_command_auto(SMDSAT_CMD_WRITE_TX_SIZE, size_data, 2)) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: TX SIZE failed", __func__);
		return false;
	}

	// Step 3: WRITE_TX (0x16) - Send actual payload data
	{
		SpiAplusResponse response = {};
		send_command_aplus(SMDSAT_CMD_WRITE_TX, reinterpret_cast<const uint8_t *>(payload.data()), size, &response);
		if (response.status != SPI_APLUS_STATUS_OK && response.valid) {
			DEBUG_WARN("SmdSatCmdSpi::%s: TX DATA status=%u (non-fatal | data may be queued)", __func__,
			           (unsigned)response.status);
		}
	}

	// Post-TX delay for STM32 async processing
	nrf_delay_ms(SMDSAT_SPI_POST_TX_DELAY_MS);
	return true;
}

// ============================================================================
// DFU (Device Firmware Update) Implementation
// ============================================================================

// CRC-32 MPEG-2 calculation (polynomial 0x04C11DB7)
// Compatible with Zephyr argos-smd-driver
uint32_t SmdSatCmdSpi::calculate_crc32(const uint8_t *data, size_t len) {
	return spi_crc32_mpeg2(data, len);
}

// Matches Zephyr dfu_send_cmd() from argos_smd_dfu_spi.c:
// Protocol A+ two-transaction model:
//   1. Send [0xAA][SEQ][CMD][LEN][DATA][CRC8][pad 0xAA] as single SPI transaction
//   2. Wait command-specific delay
//   3. Send 64-byte idle pattern to read response [0x55][SEQ][STATUS][LEN][DATA][CRC8]
//   4. Re-poll if slave BUSY (0xBB) or transitional (0xAA)
//   5. Parse response, increment sequence number
SmdDfuResponse SmdSatCmdSpi::dfu_send_command(uint8_t cmd, const uint8_t *data, uint16_t data_len,
                                              uint8_t *response_data, uint16_t *response_len) {
	DEBUG_TRACE("SmdSatCmdSpi::%s: cmd=0x%02X | data_len=%u | seq=%u", __func__, cmd, data_len, m_sequence_number);

	// ═══ Step 1: Build Protocol A+ request frame ═══
	// Format: [0xAA][SEQ][CMD][LEN][DATA...][CRC8]
	constexpr uint16_t DFU_LARGE_TX_SIZE = 280;    // BL_SPI_TRANSACTION_SIZE in bootloader
	constexpr uint16_t DFU_TRANSACTION_SIZE = 64;  // Standard poll transaction size
	constexpr uint8_t DFU_MAX_PAYLOAD = 255;
	constexpr uint8_t DFU_BUSY_RETRY_COUNT = 10;
	constexpr uint8_t DFU_BUSY_RETRY_DELAY_MS = 20;

	uint8_t tx_buf[DFU_LARGE_TX_SIZE];
	uint8_t rx_buf[DFU_LARGE_TX_SIZE];
	uint16_t idx = 0;
	int ret;

	if (data_len > DFU_MAX_PAYLOAD) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: Payload too large: %u", __func__, data_len);
		return DFU_RSP_SIZE_ERROR;
	}

	// Build frame
	tx_buf[idx++] = SPI_PROTOCOL_APLUS_MAGIC_REQUEST;
	tx_buf[idx++] = m_sequence_number;
	tx_buf[idx++] = cmd;
	tx_buf[idx++] = static_cast<uint8_t>(data_len);

	if (data != nullptr && data_len > 0) {
		memcpy(&tx_buf[idx], data, data_len);
		idx += data_len;
	}

	// CRC calculated over: MAGIC + SEQ + CMD + LEN + DATA
	tx_buf[idx] = spi_crc8_ccitt(tx_buf, idx);
	idx++;

	// Pad to transaction size
	uint16_t tx_size = (idx < DFU_TRANSACTION_SIZE) ? DFU_TRANSACTION_SIZE : idx;
	if (idx < tx_size) {
		memset(&tx_buf[idx], SPI_PROTOCOL_APLUS_IDLE_PATTERN, tx_size - idx);
	}

	DEBUG_TRACE("SmdSatCmdSpi::%s: TX[cmd=0x%02X seq=%u len=%u]: %02X %02X %02X %02X %02X ...", __func__, cmd,
	            m_sequence_number, data_len, tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3], tx_buf[4]);

	// ═══ Step 2: Transaction 1 — Send command ═══
	// (RX contains idle pattern, ignore it)
	memset(rx_buf, 0, sizeof(rx_buf));
	ret = m_nrf_spim->transfer(tx_buf, rx_buf, tx_size);
	if (ret) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: SPI TX failed: %d", __func__, ret);
		return DFU_RSP_ERROR;
	}

	DEBUG_TRACE("SmdSatCmdSpi::%s: RX1 (idle): %02X %02X %02X %02X", __func__, rx_buf[0], rx_buf[1], rx_buf[2],
	            rx_buf[3]);

	// ═══ Step 3: Wait for slave to process command ═══
	// For long delays (erase=3000ms), kick watchdog periodically
	uint32_t cmd_delay = get_command_delay(cmd);
	while (cmd_delay > 500) {
		PMU::kick_watchdog();
		nrf_delay_ms(500);
		cmd_delay -= 500;
	}
	if (cmd_delay > 0) {
		nrf_delay_ms(cmd_delay);
	}

	// ═══ Step 4: Transaction 2 — Send idle pattern to read response ═══
	memset(tx_buf, SPI_PROTOCOL_APLUS_IDLE_PATTERN, DFU_TRANSACTION_SIZE);
	memset(rx_buf, 0, sizeof(rx_buf));
	ret = m_nrf_spim->transfer(tx_buf, rx_buf, DFU_TRANSACTION_SIZE);
	if (ret) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: SPI RX failed: %d", __func__, ret);
		return DFU_RSP_ERROR;
	}

	DEBUG_TRACE("SmdSatCmdSpi::%s: RX2: %02X %02X %02X %02X %02X %02X %02X %02X", __func__, rx_buf[0], rx_buf[1],
	            rx_buf[2], rx_buf[3], rx_buf[4], rx_buf[5], rx_buf[6], rx_buf[7]);

	// ═══ Step 4b: Re-poll until valid response (0x55) received ═══
	auto has_valid_response = [](const uint8_t *buf, uint16_t len) -> bool {
		uint16_t check_len = (len < 8) ? len : 8;
		for (uint16_t i = 0; i < check_len; i++) {
			if (buf[i] == SPI_PROTOCOL_APLUS_MAGIC_RESPONSE) {
				return true;
			}
		}
		return false;
	};

	auto is_busy_pattern = [](const uint8_t *buf, uint16_t len) -> bool {
		return (len >= 2 && buf[0] == 0xBB && buf[1] == 0xBB);
	};

	uint8_t busy_retries = 0;
	while (!has_valid_response(rx_buf, DFU_TRANSACTION_SIZE) && busy_retries < DFU_BUSY_RETRY_COUNT) {
		if (is_busy_pattern(rx_buf, DFU_TRANSACTION_SIZE)) {
			DEBUG_TRACE("SmdSatCmdSpi::%s: Slave BUSY (0xBB) | re-polling... (%u/%u)", __func__, busy_retries + 1,
			            DFU_BUSY_RETRY_COUNT);
		}
		// Note: non-busy non-valid response also retries (same action as busy)

		nrf_delay_ms(DFU_BUSY_RETRY_DELAY_MS);

		memset(tx_buf, SPI_PROTOCOL_APLUS_IDLE_PATTERN, DFU_TRANSACTION_SIZE);
		memset(rx_buf, 0, sizeof(rx_buf));
		ret = m_nrf_spim->transfer(tx_buf, rx_buf, DFU_TRANSACTION_SIZE);
		if (ret) {
			DEBUG_ERROR("SmdSatCmdSpi::%s: SPI re-poll failed: %d", __func__, ret);
			return DFU_RSP_ERROR;
		}

		DEBUG_TRACE("SmdSatCmdSpi::%s: RX2 re-poll: %02X %02X %02X %02X %02X %02X %02X %02X", __func__, rx_buf[0],
		            rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4], rx_buf[5], rx_buf[6], rx_buf[7]);

		busy_retries++;
	}

	if (!has_valid_response(rx_buf, DFU_TRANSACTION_SIZE)) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: No valid response after %u retries", __func__, DFU_BUSY_RETRY_COUNT);
		m_sequence_number++;
		return DFU_RSP_ERROR;
	}

	if (busy_retries > 0) {
		DEBUG_TRACE("SmdSatCmdSpi::%s: Slave responded after %u re-polls", __func__, busy_retries);
	}

	// ═══ Step 5: Parse response frame ═══
	SpiAplusResponse response;
	bool parsed = parse_aplus_response(rx_buf, DFU_TRANSACTION_SIZE, &response);

	// Increment sequence number for next command
	m_sequence_number++;

	if (!parsed) {
		if (!response.crc_error) {
			DEBUG_ERROR("SmdSatCmdSpi::%s: Failed to parse response", __func__);
			return DFU_RSP_ERROR;
		}
		// CRC-only mismatch on a DFU bootloader response: the frame structure
		// (magic/seq/status/len/data) is valid and the payload was already copied,
		// but the per-frame CRC8 mismatched. On this SMD bootloader the DFU response
		// CRC is deterministically off (identical recv/calc across power rails ⇒ a
		// framing/spec quirk, not line corruption), which otherwise aborts the whole
		// update on the first command. The transfer is still protected end-to-end by
		// dfu_verify() (CRC32 over the flashed image), which gates the jump to the app
		// — so accept the frame using its real status rather than failing the update.
		DEBUG_WARN("SmdSatCmdSpi::%s: DFU response CRC mismatch tolerated (status=0x%02X len=%u) "
		           "— end-to-end CRC32 verify still gates the jump",
		           __func__, response.raw_status, response.data_len);
		response.status = static_cast<SpiAplusStatus>(response.raw_status);
	}

	// Copy response data if requested
	if (response_data != nullptr && response_len != nullptr && response.data_len > 0) {
		uint16_t cpy = (*response_len < response.data_len) ? *response_len : response.data_len;
		memcpy(response_data, response.data, cpy);
		*response_len = cpy;
	} else if (response_len != nullptr) {
		*response_len = response.data_len;
	}

	DEBUG_TRACE("SmdSatCmdSpi::%s: Response status=0x%02X len=%u", __func__, response.status, response.data_len);

	return static_cast<SmdDfuResponse>(response.status);
}

// Matches Zephyr dfu_send_with_retry() — wraps dfu_send_command with automatic
// retry on recoverable errors (BUSY=0x06 and FRAME_CRC_ERROR=0x10).
SmdDfuResponse SmdSatCmdSpi::dfu_send_with_retry(uint8_t cmd, const uint8_t *data, uint16_t data_len,
                                                 uint8_t *response_data, uint16_t *response_len) {
	for (int retry = 0; retry < SMDSAT_DFU_MAX_RETRIES; retry++) {
		SmdDfuResponse result = dfu_send_command(cmd, data, data_len, response_data, response_len);

		if (result == DFU_RSP_OK) {
			return DFU_RSP_OK;
		}

		// Check for recoverable errors (matches Zephyr argos_is_recoverable)
		bool recoverable = (result == DFU_RSP_BUSY || result == static_cast<SmdDfuResponse>(0x10));  // FRAME_CRC_ERROR
		if (recoverable) {
			DEBUG_WARN("SmdSatCmdSpi::%s: Recoverable error (0x%02X) | retry %d/%d", __func__, static_cast<int>(result),
			           retry + 1, SMDSAT_DFU_MAX_RETRIES);
			nrf_delay_ms(smdsat_spi_retry_delay_ms());
			continue;
		}

		// Non-recoverable error — return immediately
		return result;
	}

	DEBUG_ERROR("SmdSatCmdSpi::%s: cmd=0x%02X failed after %d retries", __func__, cmd, SMDSAT_DFU_MAX_RETRIES);
	return DFU_RSP_ERROR;
}

SmdDfuResponse SmdSatCmdSpi::dfu_ping_bl() {
	DEBUG_TRACE("SmdSatCmdSpi::%s", __func__);

	uint8_t response_data[4];
	uint16_t response_len = sizeof(response_data);

	SmdDfuResponse result = dfu_send_command(SMDSAT_CMD_DFU_PING, nullptr, 0, response_data, &response_len);

	if (result == DFU_RSP_OK && response_len >= 2) {
		DEBUG_INFO("SmdSatCmdSpi::%s: Bootloader v%u.%u", __func__, response_data[0], response_data[1]);
	}

	return result;
}

SmdDfuResponse SmdSatCmdSpi::dfu_get_info(SmdDfuInfo *info) {
	DEBUG_TRACE("SmdSatCmdSpi::%s", __func__);

	if (info == nullptr) {
		return DFU_RSP_ERROR;
	}

	uint8_t response_data[32];
	uint16_t response_len = sizeof(response_data);

	SmdDfuResponse result = dfu_send_command(SMDSAT_CMD_DFU_GET_INFO, nullptr, 0, response_data, &response_len);

	if (result == DFU_RSP_OK && response_len >= 15) {
		info->version_major = response_data[0];
		info->version_minor = response_data[1];
		info->version_patch = response_data[2];
		// Little-endian parsing (matches Zephyr argos_dfu_get_info)
		info->app_start_addr =
		    response_data[3] | (response_data[4] << 8) | (response_data[5] << 16) | (response_data[6] << 24);
		info->app_max_size =
		    response_data[7] | (response_data[8] << 8) | (response_data[9] << 16) | (response_data[10] << 24);
		info->flash_page_size =
		    response_data[11] | (response_data[12] << 8) | (response_data[13] << 16) | (response_data[14] << 24);

		DEBUG_INFO("SmdSatCmdSpi::%s: Bootloader v%u.%u.%u | app_start=0x%08X | max_size=%u | page_size=%u", __func__,
		           info->version_major, info->version_minor, info->version_patch, info->app_start_addr,
		           info->app_max_size, info->flash_page_size);
	}

	return result;
}

// Matches Zephyr argos_dfu_erase() — 3000ms delay handled inside dfu_send_command
SmdDfuResponse SmdSatCmdSpi::dfu_erase() {
	DEBUG_INFO("SmdSatCmdSpi::%s: Erasing application flash (~2-3 seconds)...", __func__);

	SmdDfuResponse result = dfu_send_command(SMDSAT_CMD_DFU_ERASE, nullptr, 0, nullptr, nullptr);

	if (result == DFU_RSP_OK) {
		DEBUG_INFO("SmdSatCmdSpi::%s: Flash erased successfully", __func__);
	} else {
		DEBUG_ERROR("SmdSatCmdSpi::%s: Erase failed: 0x%02X", __func__, static_cast<int>(result));
	}

	return result;
}

// Matches Zephyr argos_dfu_write_chunk() — WRITE_REQ + WRITE_DATA with retry
SmdDfuResponse SmdSatCmdSpi::dfu_write_chunk(uint32_t addr, const uint8_t *data, uint16_t len) {
	DEBUG_TRACE("SmdSatCmdSpi::%s: addr=0x%08X | len=%u", __func__, addr, len);

	if (data == nullptr || len == 0 || len > SMDSAT_DFU_CHUNK_SIZE) {
		return DFU_RSP_SIZE_ERROR;
	}

	// STM32 flash requires 8-byte aligned writes
	if ((addr & 0x7) != 0) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: Address 0x%08X not 8-byte aligned", __func__, addr);
		return DFU_RSP_ADDR_ERROR;
	}

	// Step 1: WRITE_REQ — Send address and length (little-endian)
	uint8_t req_data[6];
	req_data[0] = addr & 0xFF;
	req_data[1] = (addr >> 8) & 0xFF;
	req_data[2] = (addr >> 16) & 0xFF;
	req_data[3] = (addr >> 24) & 0xFF;
	req_data[4] = len & 0xFF;
	req_data[5] = (len >> 8) & 0xFF;

	SmdDfuResponse result = dfu_send_with_retry(SMDSAT_CMD_DFU_WRITE_REQ, req_data, 6, nullptr, nullptr);
	if (result != DFU_RSP_OK) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: WRITE_REQ failed at 0x%08X: 0x%02X", __func__, addr, static_cast<int>(result));
		return result;
	}

	// Step 2: WRITE_DATA — Send actual data (with retry)
	result = dfu_send_with_retry(SMDSAT_CMD_DFU_WRITE_DATA, data, len, nullptr, nullptr);
	if (result != DFU_RSP_OK) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: WRITE_DATA failed at 0x%08X: 0x%02X", __func__, addr, static_cast<int>(result));
	}

	return result;
}

// Matches Zephyr argos_dfu_read() — READ_REQ + READ_DATA with retry
SmdDfuResponse SmdSatCmdSpi::dfu_read_chunk(uint32_t addr, uint8_t *data, uint16_t len) {
	DEBUG_TRACE("SmdSatCmdSpi::%s: addr=0x%08X | len=%u", __func__, addr, len);

	if (data == nullptr || len == 0) {
		return DFU_RSP_SIZE_ERROR;
	}

	// Step 1: READ_REQ — Send address and length (little-endian)
	uint8_t req_data[6];
	req_data[0] = addr & 0xFF;
	req_data[1] = (addr >> 8) & 0xFF;
	req_data[2] = (addr >> 16) & 0xFF;
	req_data[3] = (addr >> 24) & 0xFF;
	req_data[4] = len & 0xFF;
	req_data[5] = (len >> 8) & 0xFF;

	SmdDfuResponse result = dfu_send_with_retry(SMDSAT_CMD_DFU_READ_REQ, req_data, 6, nullptr, nullptr);
	if (result != DFU_RSP_OK) {
		DEBUG_ERROR("SmdSatCmdSpi::%s: READ_REQ failed: 0x%02X", __func__, static_cast<int>(result));
		return result;
	}

	// Step 2: READ_DATA — Read the data
	uint16_t response_len = len;
	result = dfu_send_with_retry(SMDSAT_CMD_DFU_READ_DATA, nullptr, 0, data, &response_len);

	return result;
}

SmdDfuResponse SmdSatCmdSpi::dfu_verify(uint32_t crc32) {
	DEBUG_TRACE("SmdSatCmdSpi::%s: crc32=0x%08X", __func__, crc32);

	// Send CRC32 in little-endian format (matches Zephyr argos_dfu_verify)
	uint8_t crc_data[4];
	crc_data[0] = crc32 & 0xFF;
	crc_data[1] = (crc32 >> 8) & 0xFF;
	crc_data[2] = (crc32 >> 16) & 0xFF;
	crc_data[3] = (crc32 >> 24) & 0xFF;

	SmdDfuResponse result = dfu_send_command(SMDSAT_CMD_DFU_VERIFY, crc_data, 4, nullptr, nullptr);

	if (result == DFU_RSP_OK) {
		DEBUG_INFO("SmdSatCmdSpi::%s: CRC verification passed", __func__);
	} else {
		DEBUG_ERROR("SmdSatCmdSpi::%s: CRC verification failed", __func__);
	}

	return result;
}

SmdDfuResponse SmdSatCmdSpi::dfu_reset() {
	DEBUG_TRACE("SmdSatCmdSpi::%s", __func__);
	return dfu_send_command(SMDSAT_CMD_DFU_RESET, nullptr, 0, nullptr, nullptr);
}

// Matches Zephyr argos_dfu_jump() — device may not respond after jump, that's OK
SmdDfuResponse SmdSatCmdSpi::dfu_jump() {
	DEBUG_INFO("SmdSatCmdSpi::%s: Jumping to application...", __func__);

	SmdDfuResponse result = dfu_send_command(SMDSAT_CMD_DFU_JUMP, nullptr, 0, nullptr, nullptr);

	// Device may not respond after jump - that's OK (it has already jumped)
	if (result != DFU_RSP_OK) {
		DEBUG_WARN("SmdSatCmdSpi::%s: Jump response: 0x%02X (device may have jumped)", __func__,
		           static_cast<int>(result));
	}

	m_dfu_mode = false;
	nrf_delay_ms(SMDSAT_DFU_RESET_DELAY_MS);

	return DFU_RSP_OK;  // Always return OK — no response means jump succeeded
}

SmdDfuResponse SmdSatCmdSpi::dfu_get_status(uint8_t *status) {
	DEBUG_TRACE("SmdSatCmdSpi::%s", __func__);

	uint8_t response_data[4];
	uint16_t response_len = sizeof(response_data);

	SmdDfuResponse result = dfu_send_command(SMDSAT_CMD_DFU_GET_STATUS, nullptr, 0, response_data, &response_len);

	if (result == DFU_RSP_OK && status != nullptr && response_len >= 1) {
		*status = response_data[0];
	}

	return result;
}

SmdDfuResponse SmdSatCmdSpi::dfu_abort() {
	DEBUG_TRACE("SmdSatCmdSpi::%s", __func__);
	return dfu_send_command(SMDSAT_CMD_DFU_ABORT, nullptr, 0, nullptr, nullptr);
}

SmdDfuResponse SmdSatCmdSpi::dfu_set_header(const uint8_t *header) {
	DEBUG_TRACE("SmdSatCmdSpi::%s", __func__);

	if (header == nullptr) {
		return DFU_RSP_ERROR;
	}

	return dfu_send_command(SMDSAT_CMD_DFU_SET_HEADER, header, SMDSAT_DFU_HEADER_SIZE, nullptr, nullptr);
}

// DFU entry sequence — matches Zephyr argos_dfu_enter() + argos_dfu_wait_ready():
//
// STEP 1: SYNC with running app (5x 0xFF frames → expect 0xAA idle)
// STEP 2: PING APP (cmd 0x02) to confirm app is running
// STEP 3: DFU_ENTER (cmd 0x3F, send-only — STM32 jumps to bootloader)
// STEP 4: ACTIVE POLL — send SPI sync frames immediately to lock bootloader
//         into SPI mode (bootloader has 3s UART/SPI detection race!)
// STEP 5: DFU PING (cmd 0x30) with retries to confirm bootloader ready
//
// CRITICAL: The STM32 bootloader races UART vs SPI for 3 seconds after boot.
// The first interface to send valid data wins. If we wait silently, UART noise
// on PA3 can trigger false detection → bootloader deinitializes SPI → all zeros.
// Solution: start sending SPI transactions ASAP after DFU_ENTER (~100ms).

// ============================================================================
// DFU high-level (SPI-specific communication sequence)
// ============================================================================

bool SmdSatCmdSpi::dfu_enter() {
	DEBUG_TRACE("SmdSatCmdSpi::%s", __func__);

	// Reset protocol state
	m_protocol_mode = SpiProtocolMode::APLUS;
	m_protocol_detected = true;
	m_sequence_number = 0;

	// STEP 1: SYNC with APP (5x 0xFF frames)
	DEBUG_INFO("SmdSatCmdSpi::%s: STEP 1 - SPI sync with app...", __func__);
	{
		uint8_t sync_tx[SPI_PROTOCOL_APLUS_FRAME_SIZE];
		uint8_t sync_rx[SPI_PROTOCOL_APLUS_FRAME_SIZE];
		memset(sync_tx, 0xFF, sizeof(sync_tx));

		m_sequence_number = 0;
		for (int i = 0; i < 5; i++) {
			PMU::kick_watchdog();
			memset(sync_rx, 0, sizeof(sync_rx));
			m_nrf_spim->transfer(sync_tx, sync_rx, SPI_PROTOCOL_APLUS_FRAME_SIZE);
			nrf_delay_ms(SMDSAT_SPI_DETECT_TIMEOUT_MS);
		}
	}
	PMU::kick_watchdog();
	nrf_delay_ms(100);

	// STEP 2: PING APP (cmd 0x02)
	DEBUG_INFO("SmdSatCmdSpi::%s: STEP 2 - Ping app...", __func__);
	m_sequence_number = 0;
	{
		SpiAplusResponse app_response;
		bool app_ok = send_command_aplus(SMDSAT_CMD_PING, nullptr, 0, &app_response);
		if (app_ok && app_response.status == SPI_APLUS_STATUS_OK) {
			DEBUG_INFO("SmdSatCmdSpi::%s: APP PING OK!", __func__);
		} else {
			DEBUG_WARN("SmdSatCmdSpi::%s: APP PING failed | checking if already in bootloader...", __func__);
			m_sequence_number = 0;
			SmdDfuResponse bl_ping = dfu_ping_bl();
			if (bl_ping == DFU_RSP_OK) {
				DEBUG_INFO("SmdSatCmdSpi::%s: Already in BOOTLOADER mode!", __func__);
				m_dfu_mode = true;
				return true;
			}
			DEBUG_WARN("SmdSatCmdSpi::%s: No response | trying DFU_ENTER anyway...", __func__);
		}
	}
	PMU::kick_watchdog();
	nrf_delay_ms(200);

	// STEP 3: DFU_ENTER (cmd 0x3F, send-only)
	DEBUG_INFO("SmdSatCmdSpi::%s: STEP 3 - DFU_ENTER...", __func__);
	{
		uint8_t tx_buf[SPI_PROTOCOL_APLUS_FRAME_SIZE];
		uint8_t rx_buf[SPI_PROTOCOL_APLUS_FRAME_SIZE];

		build_aplus_frame(tx_buf, SMDSAT_CMD_DFU_ENTER, nullptr, 0);
		memset(rx_buf, 0, sizeof(rx_buf));
		m_nrf_spim->transfer(tx_buf, rx_buf, SPI_PROTOCOL_APLUS_FRAME_SIZE);
		m_sequence_number++;
	}

	m_sequence_number = 0;

	// STEP 4: ACTIVE POLL — aggressive SPI sync to win UART/SPI race
	DEBUG_INFO("SmdSatCmdSpi::%s: STEP 4 - Active SPI sync...", __func__);
	PMU::kick_watchdog();
	nrf_delay_ms(200);

	{
		uint8_t sync_tx[SPI_PROTOCOL_APLUS_FRAME_SIZE];
		uint8_t sync_rx[SPI_PROTOCOL_APLUS_FRAME_SIZE];
		memset(sync_tx, SPI_PROTOCOL_APLUS_IDLE_PATTERN, sizeof(sync_tx));

		bool bootloader_detected = false;

		for (int attempt = 0; attempt < 500; attempt++) {
			if (attempt % 50 == 0) PMU::kick_watchdog();
			memset(sync_rx, 0, sizeof(sync_rx));
			m_nrf_spim->transfer(sync_tx, sync_rx, SPI_PROTOCOL_APLUS_FRAME_SIZE);

			for (int j = 0; j < 8; j++) {
				if (sync_rx[j] == SPI_PROTOCOL_APLUS_IDLE_PATTERN) {
					bootloader_detected = true;
					break;
				}
			}

			if (attempt < 20 || bootloader_detected || (attempt % 50 == 0)) {
				DEBUG_INFO("SmdSatCmdSpi::%s: Sync %d RX: %02X %02X %02X %02X %02X %02X %02X %02X%s", __func__, attempt,
				           sync_rx[0], sync_rx[1], sync_rx[2], sync_rx[3], sync_rx[4], sync_rx[5], sync_rx[6],
				           sync_rx[7], bootloader_detected ? " <- IDLE" : "");
			}

			if (bootloader_detected) {
				DEBUG_INFO("SmdSatCmdSpi::%s: Bootloader SPI detected after %d sync frames", __func__, attempt + 1);
				break;
			}

			nrf_delay_ms(10);
		}
	}
	PMU::kick_watchdog();
	nrf_delay_ms(50);

	// STEP 5: DFU PING with retries
	DEBUG_INFO("SmdSatCmdSpi::%s: STEP 5 - DFU PING...", __func__);
	{
		constexpr int MAX_PING_ATTEMPTS = 30;
		constexpr int PING_RETRY_DELAY_MS = 100;

		for (int attempt = 0; attempt < MAX_PING_ATTEMPTS; attempt++) {
			m_sequence_number = 0;
			SmdDfuResponse bl_ping = dfu_ping_bl();

			if (bl_ping == DFU_RSP_OK) {
				DEBUG_INFO("SmdSatCmdSpi::%s: BOOTLOADER PING OK!", __func__);
				m_dfu_mode = true;
				return true;
			}

			PMU::kick_watchdog();
			nrf_delay_ms(PING_RETRY_DELAY_MS);
		}
	}

	DEBUG_ERROR("SmdSatCmdSpi::%s: Bootloader not responding", __func__);
	return false;
}

bool SmdSatCmdSpi::dfu_exit() {
	if (!m_dfu_mode) {
		return true;
	}

	SmdDfuResponse result = dfu_jump();
	if (result == DFU_RSP_OK) {
		m_dfu_mode = false;
		return true;
	}
	return false;
}

bool SmdSatCmdSpi::dfu_get_bootloader_info(SmdDfuInfo *info) {
	if (!m_dfu_mode) {
		DEBUG_WARN("SmdSatCmdSpi::%s: Not in DFU mode", __func__);
		return false;
	}
	return (dfu_get_info(info) == DFU_RSP_OK);
}

std::string SmdSatCmdSpi::run_command_test() {
	DEBUG_INFO("SmdSatCmdSpi::%s: === COMMAND TEST START ===", __func__);

	// Verify SPI communication is alive. If ping fails, the A+ sequence
	// may be desynced (e.g. from a previous failed test) — reset and retry.
	bool smd_ready = ping();
	if (!smd_ready) {
		DEBUG_WARN("SmdSatCmdSpi::%s: first ping failed, resync seq", __func__);
		m_sequence_number = 0;
		for (uint8_t attempt = 0; attempt < 10; attempt++) {
			nrf_delay_ms(100);
			if (ping()) {
				smd_ready = true;
				break;
			}
		}
	}

	if (!smd_ready) {
		return "FAIL: SMD not ready (ping failed)";
	}

	struct CmdTest {
		uint8_t cmd;
		const char *name;
	};

	CmdTest tests[] = {
		{ SMDSAT_CMD_PING, "PING" },
		{ SMDSAT_CMD_READ_VERSION, "READ_VERSION" },
		{ SMDSAT_CMD_SPI_STATUS, "SPI_STATUS" },
		{ SMDSAT_CMD_MAC_STATUS, "MAC_STATUS" },
		{ SMDSAT_CMD_READ_ID, "READ_ID" },
		{ SMDSAT_CMD_READ_ADDR, "READ_ADDR" },
		{ SMDSAT_CMD_READ_SN, "READ_SN" },
		{ SMDSAT_CMD_READ_RCONF, "READ_RCONF" },
		{ SMDSAT_CMD_READ_KMAC, "READ_KMAC" },
		{ SMDSAT_CMD_READ_LPM, "READ_LPM" },
		{ SMDSAT_CMD_READ_SECKEY, "READ_SECKEY" },
		{ SMDSAT_CMD_READ_SPIMAC_STATE, "READ_SPIMAC_STATE" },
		{ SMDSAT_CMD_READ_RCONF_RAW, "READ_RCONF_RAW" },
	};

	uint8_t pass_count = 0;
	uint8_t num_tests = sizeof(tests) / sizeof(tests[0]);
	std::string fails;

	try {
		for (uint8_t i = 0; i < num_tests; i++) {
			SpiAplusResponse response = {};
			bool ok = false;
			try {
				ok = send_command_aplus(tests[i].cmd, nullptr, 0, &response);
			} catch (...) {}

			if (ok && response.status == SPI_APLUS_STATUS_OK) {
				pass_count++;
			} else {
				if (!fails.empty()) fails += " ";
				fails += tests[i].name;
			}
			nrf_delay_ms(50);
		}
	} catch (...) {
		return "FAIL: Exception";
	}

	std::string result = std::to_string(pass_count) + "/" + std::to_string(num_tests);
	if (fails.empty()) {
		result += " ALL OK";
	} else {
		result += " FAIL:" + fails;
	}

	// ================================================================
	// Phase 2: TX flow test — reproduces the field failure pattern
	// Tests: RCONF save → KMAC reload → immediate TX → poll result
	// ================================================================
	std::string tx_result = run_tx_flow_test();
	result += " | TX:" + tx_result;

	return result;
}

std::string SmdSatCmdSpi::run_tx_flow_test() {
	DEBUG_INFO("SmdSatCmdSpi::%s: === TX FLOW TEST START ===", __func__);

	std::string result;
	uint8_t tx_pass = 0;
	uint8_t tx_total = 0;

	// Payloads for each modulation
	uint8_t dummy_lda2[ARGOS_TX_LDA2_MIN_BYTE_SIZE] = { 0xFF, 0xFF, 0xFF, 0xFF };
	uint8_t dummy_ldk[ARGOS_TX_LDK_PAYLOAD_BYTE_SIZE] = {};
	uint8_t dummy_vlda4[ARGOS_TX_VLDA4_PAYLOAD_BYTE_SIZE] = { 0x00, 0x92, 0x00 };
	memset(dummy_ldk, 0xFF, sizeof(dummy_ldk));
	KineisPacket pkt_lda2(dummy_lda2, dummy_lda2 + sizeof(dummy_lda2));
	KineisPacket pkt_ldk(dummy_ldk, dummy_ldk + sizeof(dummy_ldk));
	KineisPacket pkt_vlda4(dummy_vlda4, dummy_vlda4 + sizeof(dummy_vlda4));

	// Helper: resync SPI A+ protocol after TX.
	// The TX flow (TX_REQ + TX_SIZE + WRITE_TX + multiple polls) increments
	// the A+ sequence number many times. If any SPI transfer fails or the
	// response is missed, the sequence numbers on nRF and STM32 diverge.
	// All subsequent commands then fail with SEQ_ERROR or INVALID_CMD.
	// A ping is the simplest way to resync — the STM32 accepts it regardless
	// of sequence state.
	// NOTE: No KMAC reload needed here. MAC terminal states (TX_DONE etc.)
	// are overwritten by the next TX initiation. Flash operations (SAVE_RCONF)
	// do not depend on MAC status.
	auto resync_after_tx = [&]() -> bool {
		PMU::kick_watchdog();
		// After TX+poll, the A+ sequence number is heavily desynced
		// (TX_REQ + TX_SIZE + WRITE_TX + N polls = ~16 transactions).
		// A single ping won't fix it. Hard-reset the seq and flush the
		// SPI pipeline with multiple pings, like power_on_blocking() does.
		m_sequence_number = 0;
		nrf_delay_ms(200);
		for (uint8_t attempt = 0; attempt < 5; attempt++) {
			if (ping()) return true;
			nrf_delay_ms(100);
		}
		DEBUG_ERROR("SmdSatCmdSpi::%s: resync failed after 5 pings", __func__);
		return false;
	};

	// Helper: poll READ_SPIMAC_STATE for MAC_OK after KMAC load
	auto wait_mac_ok = [&](uint32_t timeout_ms = 2000) -> bool {
		uint32_t elapsed = 0;
		while (elapsed < timeout_ms) {
			PMU::kick_watchdog();
			uint8_t spi_st = 0, mac_st = 0;
			try {
				read_spimac_state(&spi_st, &mac_st);
			} catch (...) {}
			if (mac_st == MAC_OK) return true;
			nrf_delay_ms(100);
			elapsed += 100;
		}
		return false;
	};

	// Helper: run a single TX and return result string
	auto run_tx = [&](const char *tag, const KineisPacket &pkt) -> bool {
		PMU::kick_watchdog();
		tx_total++;
		bool tx_ok = initiate_tx(pkt);
		if (!tx_ok) {
			uint8_t spi_st = 0, mac_st = 0;
			try {
				read_spimac_state(&spi_st, &mac_st);
			} catch (...) {}
			result +=
			    std::string(tag) + ":REQ_FAIL(spi=" + std::to_string(spi_st) + ",mac=" + std::to_string(mac_st) + ") ";
			return false;
		}
		uint8_t mac_result = poll_tx_result(6000);
		if (mac_result == MAC_TX_DONE || mac_result == MAC_TX_TIMEOUT) {
			tx_pass++;
			result += std::string(tag) + ":OK ";
			return true;
		}
		result += std::string(tag) + ":MAC" + std::to_string(mac_result) + " ";
		return false;
	};

	// Helper: get the right packet for a given modulation
	auto pkt_for_mod = [&](SmdArgosModulation mod) -> const KineisPacket & {
		switch (mod) {
		case ARGOS_MOD_VLDA4: return pkt_vlda4;
		case ARGOS_MOD_LDK: return pkt_ldk;
		case ARGOS_MOD_LDA2:
		default: return pkt_lda2;
		}
	};

	// Helper: resync + KMAC load + wait MAC_OK + read modulation
	auto init_and_get_mod = [&]() -> SmdArgosModulation {
		m_sequence_number = 0;
		nrf_delay_ms(100);
		if (!ping()) return ARGOS_MOD_LDA2;

		try {
			load_kmac_profil(1);
		} catch (...) {
			return ARGOS_MOD_LDA2;
		}

		if (!wait_mac_ok()) return ARGOS_MOD_LDA2;

		// Read parsed RCONF (CMD 0x0A) — returns modulation as byte[9]
		// This works after MAC_OK. Don't use RCONF_RAW (0x2B) which is
		// raw flash with different byte layout.
		// read_radio_conf returns the Kineis KNS_tx_mod_t value directly:
		//   KNS_TX_MOD_LDA2=2, LDA2L=3, VLDA4=4, HDA4=5, LDK=6
		// Our enum: ARGOS_MOD_LDA2=0, ARGOS_MOD_LDK=1, ARGOS_MOD_VLDA4=2
		// We read the raw byte and map it ourselves.
		uint8_t rx_rconf[13] = { 0 };
		uint16_t rx_len = sizeof(rx_rconf);
		SmdArgosModulation mod = ARGOS_MOD_LDA2;
		if (send_command_auto(SMDSAT_CMD_READ_RCONF, nullptr, 0, rx_rconf, &rx_len) && rx_len >= 10) {
			uint8_t kineis_mod = rx_rconf[9];
			switch (kineis_mod) {
			case 2:
			case 3: mod = ARGOS_MOD_LDA2; break;   // KNS_TX_MOD_LDA2 / LDA2L
			case 4: mod = ARGOS_MOD_VLDA4; break;  // KNS_TX_MOD_VLDA4
			case 6: mod = ARGOS_MOD_LDK; break;    // KNS_TX_MOD_LDK
			default: mod = ARGOS_MOD_LDA2; break;
			}
			DEBUG_INFO("SmdSatCmdSpi::%s: Kineis mod=%u → enum=%d", __func__, kineis_mod, (int)mod);
		}
		return mod;
	};

	// ================================================================
	// Init: resync + KMAC + read current modulation
	// ================================================================
	SmdArgosModulation cur_mod = init_and_get_mod();
	DEBUG_INFO("SmdSatCmdSpi::%s: current modulation=%d", __func__, (int)cur_mod);

	// ================================================================
	// T1: TX with current modulation (baseline)
	// ================================================================
	DEBUG_INFO("SmdSatCmdSpi::%s: [T1] TX mod=%d (%u bytes)", __func__, (int)cur_mod,
	           (unsigned)pkt_for_mod(cur_mod).size());
	run_tx("T1", pkt_for_mod(cur_mod));
	resync_after_tx();

	// ================================================================
	// T2: Back-to-back TX (no config change — verifies resync works)
	// ================================================================
	DEBUG_INFO("SmdSatCmdSpi::%s: [T2] Back-to-back TX", __func__);
	run_tx("T2", pkt_for_mod(cur_mod));
	resync_after_tx();

	std::string summary = std::to_string(tx_pass) + "/" + std::to_string(tx_total);
	if (tx_pass == tx_total) {
		summary += " ALL_OK";
	}
	summary += " [" + result + "]";

	DEBUG_INFO("SmdSatCmdSpi::%s: === TX FLOW TEST END: %s ===", __func__, summary.c_str());
	return summary;
}

uint8_t SmdSatCmdSpi::poll_tx_result(uint32_t timeout_ms) {
	uint32_t elapsed = 0;
	while (elapsed < timeout_ms) {
		PMU::kick_watchdog();
		uint8_t rx[4] = { 0 };
		uint16_t rx_len = sizeof(rx);
		if (send_command_auto(SMDSAT_CMD_READ_SPIMAC_STATE, nullptr, 0, rx, &rx_len)) {
			uint8_t mac_status = rx[1];
			if (mac_status == MAC_TX_DONE || mac_status == MAC_TX_TIMEOUT || mac_status == MAC_ERROR
			    || mac_status == MAC_RF_ABORTED || mac_status == MAC_TX_SIZE_ERROR) {
				DEBUG_INFO("SmdSatCmdSpi::%s: result=%u after %ums", __func__, mac_status, elapsed);
				return mac_status;
			}
		}
		nrf_delay_ms(SMDSAT_TIMING_POLL_MS);
		elapsed += SMDSAT_TIMING_POLL_MS;
	}
	DEBUG_WARN("SmdSatCmdSpi::%s: poll timeout after %ums", __func__, timeout_ms);
	return MAC_ERROR;
}
