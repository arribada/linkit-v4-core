#ifndef SMD_SAT_REGISTER_HPP
#define SMD_SAT_REGISTER_HPP

#ifndef DEFAULT_TCXO_WARMUP_TIME_SECONDS
#define DEFAULT_TCXO_WARMUP_TIME_SECONDS 2
#endif

// ============================================================================
// SMD SAT SPI Protocol A+ Configuration
// ============================================================================
// Based on Zephyr argos-smd-driver implementation
//
// Frame Format (64 bytes fixed, full-duplex):
// Request:  [MAGIC 0xAA][SEQ][CMD][LEN][DATA 0-250][CRC8][PADDING 0xFF...]
// Response: [MAGIC 0x55][SEQ][STATUS][LEN][DATA 0-250][CRC8][PADDING 0xFF...]
//
// Pipelined Protocol: Response to command N arrives in transaction N+2
// Send CMD, then send NOP (0x00) to retrieve response
// ============================================================================

// Protocol A+ magic bytes and patterns
#define SPI_PROTOCOL_APLUS_MAGIC_REQUEST  0xAA
#define SPI_PROTOCOL_APLUS_MAGIC_RESPONSE 0x55
#define SPI_PROTOCOL_APLUS_BUSY_PATTERN   0xBB  // Slave processing flash ops
#define SPI_PROTOCOL_APLUS_IDLE_PATTERN   0xAA  // Slave idle / padding
#define SPI_PROTOCOL_APLUS_PAD_BYTE       0xFF  // Frame padding

// Protocol A+ frame configuration (fixed 64-byte transactions)
#define SPI_PROTOCOL_APLUS_FRAME_SIZE     64   // Fixed frame size for SPI
#define SPI_PROTOCOL_APLUS_MAX_DATA_LEN   250  // Max payload in frame
#define SPI_PROTOCOL_APLUS_HEADER_LEN     4    // MAGIC + SEQ + CMD/STATUS + LEN
#define SPI_PROTOCOL_APLUS_CRC_LEN        1
#define SPI_PROTOCOL_APLUS_MAX_FRAME_LEN \
	(SPI_PROTOCOL_APLUS_HEADER_LEN + SPI_PROTOCOL_APLUS_MAX_DATA_LEN + SPI_PROTOCOL_APLUS_CRC_LEN)

// Protocol A+ unified status codes (from Zephyr driver)
typedef enum {
	SPI_APLUS_STATUS_OK = 0x00,               // Success
	SPI_APLUS_STATUS_ERROR = 0x01,            // General error
	SPI_APLUS_STATUS_CRC_ERROR = 0x02,        // Payload CRC failed
	SPI_APLUS_STATUS_ADDR_ERROR = 0x03,       // Invalid address
	SPI_APLUS_STATUS_SIZE_ERROR = 0x04,       // Invalid size
	SPI_APLUS_STATUS_FLASH_ERROR = 0x05,      // Flash operation failed
	SPI_APLUS_STATUS_BUSY = 0x06,             // Recoverable - retry
	SPI_APLUS_STATUS_INVALID_CMD = 0x07,      // Unknown command
	SPI_APLUS_STATUS_TIMEOUT = 0x08,          // Operation timeout
	SPI_APLUS_STATUS_NOT_READY = 0x09,        // Not ready
	SPI_APLUS_STATUS_INVALID_HEADER = 0x0A,   // Invalid header
	SPI_APLUS_STATUS_VERIFY_ERROR = 0x0B,     // Verification failed
	SPI_APLUS_STATUS_FRAME_CRC_ERROR = 0x10,  // Frame CRC failed - recoverable
	SPI_APLUS_STATUS_SEQ_ERROR = 0x11,        // Sequence mismatch
	SPI_APLUS_STATUS_FRAME_ERROR = 0x12       // Frame format error
} SpiAplusStatus;

// Check if status is recoverable (should retry)
// NOT_READY and FRAME_ERROR are transient after RCONF/KMAC operations
#define SPI_APLUS_IS_RECOVERABLE(s)                                                                               \
	((s) == SPI_APLUS_STATUS_BUSY || (s) == SPI_APLUS_STATUS_FRAME_CRC_ERROR || (s) == SPI_APLUS_STATUS_NOT_READY \
	 || (s) == SPI_APLUS_STATUS_FRAME_ERROR)

// Protocol detection: legacy commands are in range 0x01-0x2A
#define SPI_PROTOCOL_IS_LEGACY_CMD(byte) ((byte) >= 0x01 && (byte) <= 0x2A)

// CRC-8 CCITT calculation (polynomial 0x07)
// NOTE: CRC is calculated over MAGIC + SEQ + CMD + LEN + DATA (includes MAGIC!)
static inline uint8_t spi_crc8_ccitt(const uint8_t *data, uint16_t len) {
	uint8_t crc = 0x00;  // Initial value
	for (uint16_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (uint8_t j = 0; j < 8; j++) {
			if (crc & 0x80) {
				crc = (crc << 1) ^ 0x07;  // Polynomial 0x07
			} else {
				crc <<= 1;
			}
		}
	}
	return crc;
}

// CRC-32 MPEG-2 calculation (polynomial 0x04C11DB7)
// Used for firmware verification
static inline uint32_t spi_crc32_mpeg2(const uint8_t *data, size_t len) {
	uint32_t crc = 0xFFFFFFFF;
	for (size_t i = 0; i < len; i++) {
		crc ^= (uint32_t)data[i] << 24;
		for (int bit = 0; bit < 8; bit++) {
			if (crc & 0x80000000) {
				crc = (crc << 1) ^ 0x04C11DB7;
			} else {
				crc <<= 1;
			}
		}
	}
	return crc;
}

// ============================================================================
// SMD SAT SPI Timing Configuration (from Zephyr driver)
// ============================================================================

// Command timing delays
#define SMDSAT_TIMING_STANDARD_MS         30    // Standard commands
#define SMDSAT_TIMING_WRITE_MS            100   // Flash/NVM write (matches Zephyr ARGOS_TIMING_WRITE_MS)
#define SMDSAT_TIMING_ERASE_MS            3000  // Flash erase (critical!)
#define SMDSAT_TIMING_RESET_MS            100   // Reset/jump commands
#define SMDSAT_TIMING_POLL_MS             500   // General polling interval (DFU, etc.)
// SMDSAT_TIMING_TX_POLL_MS removed — replaced by smdsat_timing_tx_poll_ms() accessor (FAST/SAFE pair below).

// =====================================================================
// FAST / SAFE timing profiles for SMD safety fallback.
//
// Selection layers (resolved at compile time inside SmdSat accessors):
//   1. SMDSAT_USE_SAFE_TIMINGS=1 → SAFE forced for every accessor, no
//      autofallback. Robustness-first deployment.
//   2. SMDSAT_USE_SAFE_TIMINGS=0 + SMDSAT_AUTOFALLBACK_ENABLED=1 → start
//      FAST, runtime can flip to SAFE on repeated SPI errors and persist
//      the state in flash. Best balance — default for SMD builds.
//   3. SMDSAT_USE_SAFE_TIMINGS=0 + SMDSAT_AUTOFALLBACK_ENABLED=0 → FAST
//      hard-coded, no runtime branch. Pure-FAST bench mode.
//
// FAST = aggressive 2026-05 reductions; SAFE = v4.1.4 known-good baseline.
// Raw values are listed below so diffs make the trade-off visible.
// =====================================================================
// 2026-05 second field test: 5/20 ms triggered `parse_aplus_response: Response
// incomplete` cascades on one device — STM couldn't serve fast SPI polls
// during active RF emission. After a second bump request from field testing,
// FAST is now equal to SAFE for these two delays — primary first-TX latency
// gains come from POWER_ON / LOAD_KMAC / BOOT_DELAY / VDD_DISCHARGE which
// remain reduced. Autofallback still in place if these are still too aggressive.
#define SMDSAT_SPI_INTER_TX_DELAY_FAST_MS 15
#define SMDSAT_SPI_INTER_TX_DELAY_SAFE_MS 15
#define SMDSAT_SPI_RETRY_DELAY_FAST_MS    50
#define SMDSAT_SPI_RETRY_DELAY_SAFE_MS    50
#define SMDSAT_SPI_BOOT_DELAY_FAST_MS \
	30  // 2026-05 re-test: was 50ms. STM32WL SPI ready ~30ms after reset per datasheet — 30ms = 1x margin, ping retry loop covers edge cases.
#define SMDSAT_SPI_BOOT_DELAY_SAFE_MS      100
#define SMDSAT_DELAY_POWER_ON_FAST_MS      20
#define SMDSAT_DELAY_POWER_ON_SAFE_MS      50
#define SMDSAT_DELAY_LOAD_KMAC_FAST_MS     50
#define SMDSAT_DELAY_LOAD_KMAC_SAFE_MS     150
#define SMDSAT_VDD_DISCHARGE_FAST_MS       50
#define SMDSAT_VDD_DISCHARGE_SAFE_MS       100
// 2026-05 second field test: 75 ms triggered an `is_tx_finished: Failed to
// read SPIMAC state` cascade on at least one device (STM couldn't service
// the SPI poll during ongoing RF). Reverted FAST to 200 ms — same as SAFE.
// Keep the pair structure for the autofallback infrastructure and a future
// retest under controlled conditions.
#define SMDSAT_TIMING_TX_POLL_FAST_MS      200
#define SMDSAT_TIMING_TX_POLL_SAFE_MS      200
// First-TX TCXO settle when warmup forced to 0 (state_transmit_pending_exit).
// 2026-05 cold-reboot bisect: 100 ms broke (STM not finished at first poll);
// 150 ms under retest.
#define SMDSAT_FIRST_TX_BASE_DELAY_FAST_MS 150
#define SMDSAT_FIRST_TX_BASE_DELAY_SAFE_MS 200

#ifndef SMDSAT_USE_SAFE_TIMINGS
#define SMDSAT_USE_SAFE_TIMINGS 0
#endif
#ifndef SMDSAT_AUTOFALLBACK_ENABLED
#define SMDSAT_AUTOFALLBACK_ENABLED 0
#endif

// Runtime degraded-mode flag, shared between SmdSat (mutator) and SmdSatCmdSpi
// (reader for the SPI-level delays). Defined in smd_sat.cpp. Only meaningful
// when SMDSAT_AUTOFALLBACK_ENABLED == 1; ignored by the inline accessors below
// in every other build configuration.
extern bool g_smdsat_use_safe_timings;

// Inline accessors that resolve at compile time when autofallback is disabled
// (the compiler folds away the runtime branch and the static-time choice is
// the only path emitted). When autofallback is enabled, the accessors read
// g_smdsat_use_safe_timings on each call — single branch, no flash trip.
static inline unsigned int smdsat_spi_inter_tx_delay_ms() {
#if SMDSAT_USE_SAFE_TIMINGS
	return SMDSAT_SPI_INTER_TX_DELAY_SAFE_MS;
#elif SMDSAT_AUTOFALLBACK_ENABLED && (SMDSAT_SPI_INTER_TX_DELAY_FAST_MS != SMDSAT_SPI_INTER_TX_DELAY_SAFE_MS)
	return g_smdsat_use_safe_timings ? SMDSAT_SPI_INTER_TX_DELAY_SAFE_MS : SMDSAT_SPI_INTER_TX_DELAY_FAST_MS;
#else
	return SMDSAT_SPI_INTER_TX_DELAY_FAST_MS;
#endif
}
static inline unsigned int smdsat_spi_retry_delay_ms() {
#if SMDSAT_USE_SAFE_TIMINGS
	return SMDSAT_SPI_RETRY_DELAY_SAFE_MS;
#elif SMDSAT_AUTOFALLBACK_ENABLED && (SMDSAT_SPI_RETRY_DELAY_FAST_MS != SMDSAT_SPI_RETRY_DELAY_SAFE_MS)
	return g_smdsat_use_safe_timings ? SMDSAT_SPI_RETRY_DELAY_SAFE_MS : SMDSAT_SPI_RETRY_DELAY_FAST_MS;
#else
	return SMDSAT_SPI_RETRY_DELAY_FAST_MS;
#endif
}
static inline unsigned int smdsat_spi_boot_delay_ms() {
#if SMDSAT_USE_SAFE_TIMINGS
	return SMDSAT_SPI_BOOT_DELAY_SAFE_MS;
#elif SMDSAT_AUTOFALLBACK_ENABLED && (SMDSAT_SPI_BOOT_DELAY_FAST_MS != SMDSAT_SPI_BOOT_DELAY_SAFE_MS)
	return g_smdsat_use_safe_timings ? SMDSAT_SPI_BOOT_DELAY_SAFE_MS : SMDSAT_SPI_BOOT_DELAY_FAST_MS;
#else
	return SMDSAT_SPI_BOOT_DELAY_FAST_MS;
#endif
}
static inline unsigned int smdsat_delay_power_on_ms() {
#if SMDSAT_USE_SAFE_TIMINGS
	return SMDSAT_DELAY_POWER_ON_SAFE_MS;
#elif SMDSAT_AUTOFALLBACK_ENABLED && (SMDSAT_DELAY_POWER_ON_FAST_MS != SMDSAT_DELAY_POWER_ON_SAFE_MS)
	return g_smdsat_use_safe_timings ? SMDSAT_DELAY_POWER_ON_SAFE_MS : SMDSAT_DELAY_POWER_ON_FAST_MS;
#else
	return SMDSAT_DELAY_POWER_ON_FAST_MS;
#endif
}
static inline unsigned int smdsat_delay_load_kmac_ms() {
#if SMDSAT_USE_SAFE_TIMINGS
	return SMDSAT_DELAY_LOAD_KMAC_SAFE_MS;
#elif SMDSAT_AUTOFALLBACK_ENABLED && (SMDSAT_DELAY_LOAD_KMAC_FAST_MS != SMDSAT_DELAY_LOAD_KMAC_SAFE_MS)
	return g_smdsat_use_safe_timings ? SMDSAT_DELAY_LOAD_KMAC_SAFE_MS : SMDSAT_DELAY_LOAD_KMAC_FAST_MS;
#else
	return SMDSAT_DELAY_LOAD_KMAC_FAST_MS;
#endif
}
static inline unsigned int smdsat_vdd_discharge_ms() {
#if SMDSAT_USE_SAFE_TIMINGS
	return SMDSAT_VDD_DISCHARGE_SAFE_MS;
#elif SMDSAT_AUTOFALLBACK_ENABLED && (SMDSAT_VDD_DISCHARGE_FAST_MS != SMDSAT_VDD_DISCHARGE_SAFE_MS)
	return g_smdsat_use_safe_timings ? SMDSAT_VDD_DISCHARGE_SAFE_MS : SMDSAT_VDD_DISCHARGE_FAST_MS;
#else
	return SMDSAT_VDD_DISCHARGE_FAST_MS;
#endif
}
static inline unsigned int smdsat_timing_tx_poll_ms() {
#if SMDSAT_USE_SAFE_TIMINGS
	return SMDSAT_TIMING_TX_POLL_SAFE_MS;
#elif SMDSAT_AUTOFALLBACK_ENABLED && (SMDSAT_TIMING_TX_POLL_FAST_MS != SMDSAT_TIMING_TX_POLL_SAFE_MS)
	return g_smdsat_use_safe_timings ? SMDSAT_TIMING_TX_POLL_SAFE_MS : SMDSAT_TIMING_TX_POLL_FAST_MS;
#else
	return SMDSAT_TIMING_TX_POLL_FAST_MS;
#endif
}
static inline unsigned int smdsat_first_tx_base_delay_ms() {
#if SMDSAT_USE_SAFE_TIMINGS
	return SMDSAT_FIRST_TX_BASE_DELAY_SAFE_MS;
#elif SMDSAT_AUTOFALLBACK_ENABLED && (SMDSAT_FIRST_TX_BASE_DELAY_FAST_MS != SMDSAT_FIRST_TX_BASE_DELAY_SAFE_MS)
	return g_smdsat_use_safe_timings ? SMDSAT_FIRST_TX_BASE_DELAY_SAFE_MS : SMDSAT_FIRST_TX_BASE_DELAY_FAST_MS;
#else
	return SMDSAT_FIRST_TX_BASE_DELAY_FAST_MS;
#endif
}

#define SMDSAT_SPI_BUSY_WAIT_MS        100  // BUSY pattern (0xBB) = flash write in progress
#define SMDSAT_SPI_DETECT_TIMEOUT_MS   10   // SPI activity detection
#define SMDSAT_SPI_POST_TX_DELAY_MS    100  // Async processing delay
#define SMDSAT_DELAY_TICK_INTERRUPT_MS (10)
#define SMDSAT_DELAY_CMD_MS \
	SMDSAT_TIMING_STANDARD_MS  // Pre-SPI-command pacing — keep aligned to STM32 cmd_delay (30 ms)
#define SMDSAT_DELAY_STATE_TICK_MS \
	(15)  // State-machine reschedule delay between state ticks (pure scheduler latency, no SPI involvement). Was 30 ms (alias of SMDSAT_DELAY_CMD_MS); split 2026-05.
#define SMDSAT_DELAY_CMD_TX            (1000)  // First poll delay after initiate_tx
#define SMDSAT_DELAY_RST_MS            (100)

// SPI retry configuration
#define SMDSAT_SPI_MAX_RETRIES         (3)
#define SMDSAT_SPI_READY_TIMEOUT_MS    (100)
#define SMDSAT_SPI_BUSY_MAX_RETRIES    (10)  // Max retries on BUSY pattern
#define SMDSAT_SPI_BUSY_RETRY_DELAY_MS (50)  // Delay between BUSY retries (matches Zephyr FLASH_WRITE_RETRY_DELAY_MS)
#define TX_FREQUENCY_ARGOS_2_3_BAND_START 401.62

#define SMDSAT_CMD_WRITERCONF_LEN         33
#define SMDSAT_CMD_WRITELPM_LEN           2
#define SMDSAT_CMD_WRITEKMAC_LEN          2
#define SMDSAT_CMD_WRITETX_LEN            3
#define SMDSAT_CMD_WRITECONF_LEN          17
#define SMDSAT_CMD_READCONF_LEN           13
#define SMDSAT_CMD_READ_FIRMWARE_LEN      128
#define SMDSAT_CMD_READ_SERIAL_LEN        16
#define SMDSAT_CMD_WRITE_LPM_LEN          2
#define SMDSAT_CMD_WRITE_ID_LEN           5
#define SMDSAT_CMD_WRITE_TCXO_LEN         5
#define SMDSAT_CMD_READ_ADDR_LEN          4
#define SMDSAT_CMD_READ_ID_LEN            4
#define SMDSAT_CMD_WRITE_ADDR_LEN         (SMDSAT_CMD_READ_ADDR_LEN + 1)
#define SMDSAT_CMD_READ_SECKEY_LEN        16
#define SMDSAT_CMD_WRITE_SECKEY_LEN       (SMDSAT_CMD_READ_SECKEY_LEN + 1)
#define SMDSAT_CMD_READ_RCONF_RAW_LEN     16
#define SMDSAT_CMD_READ_SPIMAC_STATE_LEN  2
#define SMDSAT_CMD_READ_KMAC_LEN          2

typedef enum {
	ARGOS_MOD_LDA2,  // 0
	ARGOS_MOD_LDK,   // 1
	ARGOS_MOD_VLDA4  // 2
} SmdArgosModulation;

typedef enum {
	MAC_UNKNOWN = 0x00,
	MAC_OK = 0x01,
	MAC_TX_DONE = 0x02,
	MAC_TX_SIZE_ERROR = 0x03,
	MAC_TXACK_DONE = 0x04,
	MAC_TX_TIMEOUT = 0x05,
	MAC_TXACK_TIMEOUT = 0x06,
	MAC_RX_ERROR = 0x07,
	MAC_RX_TIMEOUT = 0x08,
	MAC_ERROR = 0x09,
	MAC_TX_IN_PROGRESS = 0x0A,
	MAC_RX_RECEIVED = 0x0B,
	MAC_SAT_DETECTED = 0x0C,
	MAC_SAT_LOST = 0x0D,
	MAC_RF_ABORTED = 0x0E
} SMDSAT_MACSTATUS;

typedef enum {
	SMDSAT_SPICMD_INIT,
	SMDSAT_SPICMD_IDLE,
	SMDSAT_SPICMD_PROCESS_CMD,
	SMDSAT_SPICMD_WAITING_RX,
	SMDSAT_SPICMD_WAITING_TX,
	SMDSAT_SPICMD_WAITING_MAC_EVT,
	SMDSAT_SPICMD_ERROR,
	SMDSAT_SPICMD_STATUS_MAX_LEN
} SMDSAT_SPISTATUS;

typedef enum {
	SMDSAT_CMD_NONE = 0x00,
	SMDSAT_CMD_READ = 0x01,
	SMDSAT_CMD_PING = 0x02,
	SMDSAT_CMD_MAC_STATUS = 0x03,
	SMDSAT_CMD_SPI_STATUS = 0x04,
	SMDSAT_CMD_READ_VERSION = 0x05,
	SMDSAT_CMD_READ_FIRMWARE = 0x06,
	SMDSAT_CMD_READ_ADDR = 0x07,
	SMDSAT_CMD_READ_ID = 0x08,
	SMDSAT_CMD_READ_SN = 0x09,
	SMDSAT_CMD_READ_RCONF = 0x0A,
	SMDSAT_CMD_WRITE_RCONF_REQ = 0x0B,
	SMDSAT_CMD_WRITE_RCONF = 0x0C,
	SMDSAT_CMD_SAVE_RCONF = 0x0D,
	SMDSAT_CMD_READ_KMAC = 0x0E,
	SMDSAT_CMD_WRITE_KMAC_REQ = 0x0F,
	SMDSAT_CMD_WRITE_KMAC = 0x10,
	SMDSAT_CMD_READ_LPM = 0x11,
	SMDSAT_CMD_WRITE_LPM_REQ = 0x12,
	SMDSAT_CMD_WRITE_LPM = 0x13,
	SMDSAT_CMD_WRITE_TX_REQ = 0x14,
	SMDSAT_CMD_WRITE_TX_SIZE = 0x15,
	SMDSAT_CMD_WRITE_TX = 0x16,
	SMDSAT_CMD_READ_CW = 0x17,
	SMDSAT_CMD_WRITE_CW_REQ = 0x18,
	SMDSAT_CMD_WRITE_CW = 0x19,
	SMDSAT_CMD_READ_PREPASSEN = 0x1A,
	SMDSAT_CMD_WRITE_PREPASSEN_REQ = 0x1B,
	SMDSAT_CMD_WRITE_PREPASSEN = 0x1C,
	SMDSAT_CMD_READ_UDATE = 0x1D,
	SMDSAT_CMD_WRITE_UDATE_REQ = 0x1E,
	SMDSAT_CMD_WRITE_UDATE = 0x1F,
	SMDSAT_CMD_WRITE_ID_REQ = 0x20,
	SMDSAT_CMD_WRITE_ID = 0x21,
	SMDSAT_CMD_WRITE_ADDR_REQ = 0x22,
	SMDSAT_CMD_WRITE_ADDR = 0x23,
	SMDSAT_CMD_READ_SECKEY = 0x24,
	SMDSAT_CMD_WRITE_SECKEY_REQ = 0x25,
	SMDSAT_CMD_WRITE_SECKEY = 0x26,
	SMDSAT_CMD_READ_SPIMAC_STATE = 0x27,
	SMDSAT_CMD_READ_TCXO = 0x28,
	SMDSAT_CMD_WRITE_TCXO_REQ = 0x29,
	SMDSAT_CMD_WRITE_TCXO = 0x2A,
	SMDSAT_CMD_READ_RCONF_RAW = 0x2B,  // Raw radio config (16 bytes from flash)
	// Argos MAC message counter (MC), 9-bit (0-511). OPTIONAL — must match the
	// opcodes added on the SMD (STM32WL) side. If the module firmware does not
	// implement them it returns INVALID_CMD and the nRF degrades gracefully
	// (MC hold disabled, SMD keeps auto-managing its own MC).
	SMDSAT_CMD_READ_MC = 0x2C,       // Read MC -> 2 bytes little-endian
	SMDSAT_CMD_WRITE_MC_REQ = 0x2D,  // 2-phase set MC: REQ
	SMDSAT_CMD_WRITE_MC = 0x2E,      // 2-phase set MC: DATA (2 bytes LE)
	SMDSAT_CMD_READ_KCFG = 0x2F,     // Read Kineis stack config bitmap (uint32 LE, read-only)
	SMDSAT_SPICMD_MAX_COUNT = 0x30,

	// ========================================================================
	// DFU Bootloader Commands (0x30-0x3F)
	// ========================================================================
	// These commands are used for firmware update via SPI
	SMDSAT_CMD_DFU_PING = 0x30,        // Test connection, returns version
	SMDSAT_CMD_DFU_GET_INFO = 0x31,    // Get bootloader info (version, addresses)
	SMDSAT_CMD_DFU_ERASE = 0x32,       // Erase application area (~2-3 sec)
	SMDSAT_CMD_DFU_WRITE_REQ = 0x33,   // Prepare write: addr(4) + len(2)
	SMDSAT_CMD_DFU_WRITE_DATA = 0x34,  // Send firmware data chunk
	SMDSAT_CMD_DFU_READ_REQ = 0x35,    // Prepare read: addr(4) + len(2)
	SMDSAT_CMD_DFU_READ_DATA = 0x36,   // Read flash data
	SMDSAT_CMD_DFU_VERIFY = 0x37,      // Verify firmware CRC32
	SMDSAT_CMD_DFU_RESET = 0x38,       // System reset
	SMDSAT_CMD_DFU_JUMP = 0x39,        // Jump to application
	SMDSAT_CMD_DFU_GET_STATUS = 0x3A,  // Get current DFU status
	SMDSAT_CMD_DFU_ABORT = 0x3B,       // Abort DFU transfer
	SMDSAT_CMD_DFU_SET_HEADER = 0x3C,  // Write application header (256 bytes)
	// Reserved: 0x3D, 0x3E
	SMDSAT_CMD_DFU_ENTER = 0x3F  // Enter DFU mode (from application)
} SMDSAT_SPICMD;

// ============================================================================
// DFU Response Codes
// ============================================================================
typedef enum {
	DFU_RSP_OK = 0x00,              // Success
	DFU_RSP_ERROR = 0x01,           // General error
	DFU_RSP_CRC_ERROR = 0x02,       // CRC verification failed
	DFU_RSP_ADDR_ERROR = 0x03,      // Invalid address
	DFU_RSP_SIZE_ERROR = 0x04,      // Invalid size
	DFU_RSP_FLASH_ERROR = 0x05,     // Flash write/erase error
	DFU_RSP_BUSY = 0x06,            // Operation in progress
	DFU_RSP_INVALID_CMD = 0x07,     // Invalid command
	DFU_RSP_TIMEOUT = 0x08,         // Operation timeout
	DFU_RSP_NOT_READY = 0x09,       // Bootloader not ready
	DFU_RSP_INVALID_HEADER = 0x0A,  // Invalid application header
	DFU_RSP_VERIFY_ERROR = 0x0B     // Firmware verification failed
} SmdDfuResponse;

// ============================================================================
// DFU Configuration Constants (from Zephyr driver)
// ============================================================================
#define SMDSAT_DFU_CHUNK_SIZE        248   // Bytes per write chunk (8-byte aligned for STM32)
#define SMDSAT_DFU_HEADER_SIZE       256   // Application header size
#define SMDSAT_DFU_ERASE_TIMEOUT_MS  3000  // Flash erase timeout (critical!)
#define SMDSAT_DFU_WRITE_TIMEOUT_MS  100   // Flash write timeout per chunk
#define SMDSAT_DFU_VERIFY_TIMEOUT_MS 2000  // CRC verification timeout
#define SMDSAT_DFU_RESET_DELAY_MS    100   // Delay after reset command
#define SMDSAT_DFU_MAX_RETRIES       3     // Max retries per operation
#define SMDSAT_DFU_READY_TIMEOUT_MS  5000  // Wait for bootloader ready

// STM32WL55 Flash Layout
#define SMDSAT_DFU_APP_START_ADDR    0x08000000    // Application start
#define SMDSAT_DFU_APP_MAX_SIZE      (204 * 1024)  // 204 KB (102 pages)
#define SMDSAT_DFU_BOOTLOADER_ADDR   0x08033000    // Bootloader (32 KB)
#define SMDSAT_DFU_FLASH_USER_ADDR   0x0803B000    // User flash (preserved)
#define SMDSAT_DFU_FLASH_END_ADDR    0x08040000    // End of flash

// DFU Bootloader info structure (matches Zephyr argos_bl_info)
struct SmdDfuInfo {
	uint8_t version_major;
	uint8_t version_minor;
	uint8_t version_patch;
	uint32_t app_start_addr;
	uint32_t app_max_size;
	uint32_t flash_page_size;
};

// Max TX payload sizes per modulation (in bytes)
#define ARGOS_TX_LDA2_PAYLOAD_BYTE_SIZE  24
#define ARGOS_TX_LDK_PAYLOAD_BYTE_SIZE   16
#define ARGOS_TX_VLDA4_PAYLOAD_BYTE_SIZE 3

// LDA2 valid message lengths in bits: 36, 68, 100, 132, 164, 196
// Corresponding byte sizes: 4, 8, 12, 16, 20, 24 (multiples of 4)
#define ARGOS_TX_LDA2_SIZE_STEP          4
#define ARGOS_TX_LDA2_MIN_BYTE_SIZE      4

// Default radio configurations per modulation (stored in SMD flash via PARMW ARP51/ARP52/ARP53
// or SATVF force-write). These are NOT used at runtime — the user's radio conf is used instead.
// Kept here as reference if a factory reset of the radio conf is needed.
// Format: freq_min, freq_max, rf_level, modulation (16 bytes each)
//   LDA2:  401620000-401680000, 27dBm, max payload 24 bytes
#define SMDSAT_DEFAULT_RCONF_LDA2        "3d678af16b5a572078f3dbc95a1104e7"
//   LDK:   402895000-402990000, 27dBm, max payload 16 bytes
#define SMDSAT_DEFAULT_RCONF_LDK         "03921fb104b92859209b18abd009de96"
//   VLDA4: 401625000-401635000, 27dBm, max payload 3 bytes
#define SMDSAT_DEFAULT_RCONF_VLDA4       "550b4bec21009c7a7b5bebaa937cdb41"

// Local helper type for SPI data transfers
struct smd_uint8_array_t {
	uint16_t size;
	uint8_t *p_data;
};

#endif
