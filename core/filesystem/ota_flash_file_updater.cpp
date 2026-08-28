/**
 * @file ota_flash_file_updater.cpp
 * @brief Flash-backed OTA file updater — MCU firmware, ARTIC, GPS config, SMD DFU.
 */

#include "ota_flash_file_updater.hpp"
#include "error.hpp"
#include "crc32.hpp"
#include "debug.hpp"
#include "pmu.hpp"

// LED feedback for DFU progress
#include "rgb_led.hpp"
#include "ledsm.hpp"
extern RGBLed *status_led;
using led_handle = LEDState;

// SMD DFU support (only when SMD satellite module is enabled)
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
#include "smd_sat.hpp"
#include "dte_protocol.hpp"
#include "ble_service.hpp"
extern SmdSat *smd_sat_instance;  // Defined in main.cpp when SMD is used
extern BLEService *ble_service;   // Defined in main.cpp
#endif

// Flash header for firmware image in external flash
static constexpr const uint32_t FLASH_HEADER_SIZE = sizeof(lfs_size_t) + sizeof(uint32_t);

// SMD firmware header: [size:4B BE][stm32_crc32:4B BE]
static constexpr const uint32_t SMD_FW_HEADER_SIZE = 8;


/// @brief Construct OTA updater with flash target region.
/// @param filesystem             LittleFS instance for file-based updates (ARTIC, GPS, SMD).
/// @param flash_if               Raw flash interface for MCU firmware (QSPI reserved region).
/// @param reserved_block_offset  Start block for MCU firmware in external flash.
/// @param reserved_blocks        Number of blocks reserved for MCU firmware.
OTAFlashFileUpdater::OTAFlashFileUpdater(LFSFileSystem *filesystem, FlashInterface *flash_if,
                                         lfs_off_t reserved_block_offset, lfs_size_t reserved_blocks) {
	m_filesystem = filesystem;
	m_flash_if = flash_if;
	m_reserved_block_offset = reserved_block_offset;
	m_reserved_blocks = reserved_blocks;
	m_file_id = OTAFileIdentifier::MCU_FIRMWARE;
	m_file = nullptr;
	m_file_size = 0;
}

/// @brief Destructor — close any open file (except MCU firmware which uses raw flash).
OTAFlashFileUpdater::~OTAFlashFileUpdater() {
	if (m_file_id != OTAFileIdentifier::MCU_FIRMWARE) delete m_file;
	m_file = nullptr;
}

/// @brief Start a new OTA transfer — erase target, write header, validate size.
/// @param file_id  Target file type (MCU, ARTIC, GPS, SMD).
/// @param length   Total firmware size in bytes (must be 4-byte aligned).
/// @param crc32    Expected CRC32 for verification after transfer.
/// @throws OTA_TRANSFER_ALREADY_IN_PROGRESS, OTA_TRANSFER_BAD_FILE_SIZE, OTA_TRANSFER_FLASH_ERROR.
void OTAFlashFileUpdater::start_file_transfer(OTAFileIdentifier file_id, lfs_size_t length, uint32_t crc32) {
	if (m_file_size) {
		DEBUG_ERROR("OTAFlashFileUpdater::start_file_transfer: transfer already in progress");
		throw ErrorCode::OTA_TRANSFER_ALREADY_IN_PROGRESS;
	}

	if (length == 0 || length & 3 || length > (m_reserved_blocks * m_flash_if->m_block_size) - FLASH_HEADER_SIZE) {
		DEBUG_ERROR("OTAFlashFileUpdater::start_file_transfer: bad transfer size %u bytes", length);
		throw ErrorCode::OTA_TRANSFER_BAD_FILE_SIZE;
	}

	switch (file_id) {
	case OTAFileIdentifier::ARTIC_FIRMWARE:
		DEBUG_INFO("OTAFlashFileUpdater::start_file_transfer: ARTIC_FIRMWARE");
		m_filesystem->remove("artic_firmware.dat");
		m_file = new LFSFile(m_filesystem, "artic_firmware.dat", LFS_O_WRONLY | LFS_O_CREAT);
		break;
	case OTAFileIdentifier::MCU_FIRMWARE:
		DEBUG_INFO("OTAFlashFileUpdater::start_file_transfer: MCU_FIRMWARE");
		// Erase reserved region of external flash
		for (unsigned int i = 0; i < m_reserved_blocks; i++) {
			uint8_t buffer[256];
			if (m_flash_if->read(m_reserved_block_offset + i, 0, buffer, 256))
				throw ErrorCode::OTA_TRANSFER_FLASH_ERROR;
			for (unsigned int j = 0; j < 256; j++) {
				if (buffer[j] != 0xFF) {
					if (m_flash_if->erase(i + m_reserved_block_offset)) throw ErrorCode::OTA_TRANSFER_FLASH_ERROR;
					break;
				}
			}
		}

		// Write the header information into the start of flash
		if (m_flash_if->prog(m_reserved_block_offset, 0, &length, sizeof(length)) || m_flash_if->sync()
		    || m_flash_if->prog(m_reserved_block_offset, sizeof(length), &crc32, sizeof(crc32)) || m_flash_if->sync())
			throw ErrorCode::OTA_TRANSFER_FLASH_ERROR;
		break;
	case OTAFileIdentifier::GPS_CONFIG:
		DEBUG_INFO("OTAFlashFileUpdater::start_file_transfer: GPS_CONFIG");
		m_filesystem->remove("gps_config.dat");
		m_file = new LFSFile(m_filesystem, "gps_config.dat", LFS_O_WRONLY | LFS_O_CREAT);
		break;
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
#if defined(SMD_UART) && (SMD_UART == 1)
	case OTAFileIdentifier::SMD_FIRMWARE_UART:
		DEBUG_INFO("OTAFlashFileUpdater::start_file_transfer: SMD_FIRMWARE_UART");
#else
	case OTAFileIdentifier::SMD_FIRMWARE_SPI: DEBUG_INFO("OTAFlashFileUpdater::start_file_transfer: SMD_FIRMWARE_SPI");
#endif
		m_filesystem->remove("smd_firmware.dat");
		m_file = new LFSFile(m_filesystem, "smd_firmware.dat", LFS_O_WRONLY | LFS_O_CREAT);
		{
			uint8_t header[8];
			header[0] = (length >> 24) & 0xFF;
			header[1] = (length >> 16) & 0xFF;
			header[2] = (length >> 8) & 0xFF;
			header[3] = length & 0xFF;
			header[4] = (crc32 >> 24) & 0xFF;
			header[5] = (crc32 >> 16) & 0xFF;
			header[6] = (crc32 >> 8) & 0xFF;
			header[7] = crc32 & 0xFF;
			m_file->write(header, 8);
		}
		break;
#endif
		// Reject: wrong transport for this build, or no SMD at all
#if !defined(ARGOS_SMD) || (ARGOS_SMD != 1)
	case OTAFileIdentifier::SMD_FIRMWARE_UART:
	case OTAFileIdentifier::SMD_FIRMWARE_SPI:
#elif defined(SMD_UART) && (SMD_UART == 1)
	case OTAFileIdentifier::SMD_FIRMWARE_SPI:
#else
	case OTAFileIdentifier::SMD_FIRMWARE_UART:
#endif
		throw ErrorCode::OTA_TRANSFER_INVALID_FILE_ID;
		break;
	default: throw ErrorCode::OTA_TRANSFER_INVALID_FILE_ID; break;
	}
	DEBUG_INFO("OTAFlashFileUpdater::start_file_transfer: m_file_id=%u | m_file_size=%u crc32=%08x",
	           static_cast<unsigned>(file_id), length, crc32);
	m_file_id = file_id;
	m_file_size = length;
	m_crc32 = crc32;
	m_crc32_calc = 0xFFFFFFFF;  // Initialize CRC to 0xFFFFFFFF for streaming calculation
	m_file_bytes_received = 0;
}

/// @brief Write a chunk of OTA data — appends to flash/file and updates streaming CRC.
/// @throws OTA_TRANSFER_NOT_STARTED, OTA_TRANSFER_OVERFLOW, OTA_TRANSFER_FLASH_ERROR.
void OTAFlashFileUpdater::write_file_data(void *const data, lfs_size_t length) {
	if (m_file_size == 0) throw ErrorCode::OTA_TRANSFER_NOT_STARTED;

	if (m_file_bytes_received + length > m_file_size) {
		abort_file_transfer();
		throw ErrorCode::OTA_TRANSFER_OVERFLOW;
	}

	if (m_file_id != OTAFileIdentifier::MCU_FIRMWARE) {
		if (!m_file) throw ErrorCode::OTA_TRANSFER_NOT_STARTED;
		m_file->write(data, length);
	} else {
		std::vector<uint8_t> aligned_buffer;
		aligned_buffer.resize(length);
		std::memcpy(&aligned_buffer[0], data, length);
		if (m_flash_if->prog(
		        0, (m_reserved_block_offset * m_flash_if->m_block_size) + m_file_bytes_received + FLASH_HEADER_SIZE,
		        &aligned_buffer[0], length)
		    || m_flash_if->sync())
			throw ErrorCode::OTA_TRANSFER_FLASH_ERROR;
	}

	m_file_bytes_received += length;

	DEBUG_TRACE("OTAFlashFileUpdater::write_file_data: %u/%u", m_file_bytes_received, m_file_size);

	// Update CRC (streaming mode - no finalization yet)
	CRC32::checksum_update((unsigned char *)data, length, m_crc32_calc);
}

/// @brief Abort transfer — delete partial file or erase flash header.
void OTAFlashFileUpdater::abort_file_transfer() {
	if (m_file_size != 0) {
		if (m_file_id != OTAFileIdentifier::MCU_FIRMWARE) {
			delete m_file;
			m_file = nullptr;
		} else {
			// Erase the first block to ensure firmware update header is erased
			if (m_flash_if->erase(m_reserved_block_offset)) throw ErrorCode::OTA_TRANSFER_FLASH_ERROR;
		}
	}
	m_file_size = 0;
}

/// @brief Complete transfer — verify all bytes received, finalize and check CRC32.
/// @note SMD firmware skips OTA CRC (STM32 bootloader verifies with its own CRC).
/// @throws OTA_TRANSFER_NOT_STARTED, OTA_TRANSFER_INCOMPLETE, OTA_TRANSFER_CRC_ERROR.
void OTAFlashFileUpdater::complete_file_transfer() {
	if (m_file_size == 0) throw ErrorCode::OTA_TRANSFER_NOT_STARTED;

	if (m_file_bytes_received < m_file_size) {
		DEBUG_ERROR("OTAFlashFileUpdater:: not all bytes received");
		abort_file_transfer();
		throw ErrorCode::OTA_TRANSFER_INCOMPLETE;
	}

#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
	// For SMD firmware files, skip OTA CRC verification
	// The SMD bootloader will verify using the STM32 CRC in the file header
	if (m_file_id == OTAFileIdentifier::SMD_FIRMWARE_UART || m_file_id == OTAFileIdentifier::SMD_FIRMWARE_SPI) {
		DEBUG_INFO("OTAFlashFileUpdater:: SMD firmware - skipping OTA CRC (bootloader will verify)");
		return;
	}
#endif

	// Finalize CRC calculation
	CRC32::checksum_finalize(m_crc32_calc);

	DEBUG_TRACE("OTAFlashFileUpdater::complete_file_transfer: CRC calc=0x%08X expected=0x%08X", m_crc32_calc, m_crc32);

	if (m_crc32_calc != m_crc32) {
		DEBUG_ERROR("OTAFlashFileUpdater:: CRC failure (calc=0x%08X expected=0x%08X)", m_crc32_calc, m_crc32);
		led_handle::dispatch<SetLEDOTAFailed>({});
		abort_file_transfer();
		throw ErrorCode::OTA_TRANSFER_CRC_ERROR;
	}
}

/// @brief Apply the firmware update — MCU: reboot into bootloader; SMD: stream DFU via SPI/UART.
void OTAFlashFileUpdater::apply_file_update() {
	DEBUG_TRACE("OTAFlashFileUpdater::apply_file_update");
	if (m_file_id == OTAFileIdentifier::MCU_FIRMWARE) {
		DEBUG_INFO("OTAFlashFileUpdater::apply_file_update: resetting device to apply firmware update");
		// Signal OTA success with green LED before reset
		led_handle::dispatch<SetLEDOTASuccess>({});
		// Record (SD-safe) that fw was updated so the next boot can detect it.
		// A direct NRF_POWER->GPREGRET2 write here faults: the SoftDevice (BLE) is
		// active during OTA and owns the POWER peripheral. This flag does NOT drive
		// the bootloader's apply (that reads the QSPI staged image) — only the app.
		PMU::set_firmware_updated_flag();
		// Delay to show green LED and allow BLE status notification
		PMU::delay_ms(1500);
		PMU::reset(false);
		// Not reached — device reboots, bootloader applies firmware from QSPI flash
	}
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
	else if (m_file_id == OTAFileIdentifier::SMD_FIRMWARE_UART || m_file_id == OTAFileIdentifier::SMD_FIRMWARE_SPI) {
		// SMD firmware update
		// File format: [size:4B LE][stm32_crc32:4B LE][firmware_data]
		DEBUG_INFO("OTAFlashFileUpdater::apply_file_update: SMD DFU via %s",
		           m_file_id == OTAFileIdentifier::SMD_FIRMWARE_SPI ? "SPI" : "UART");

		delete m_file;  // Close the file first
		m_file = nullptr;

		// Open for reading
		LFSFile fw_file(m_filesystem, "smd_firmware.dat", LFS_O_RDONLY);
		lfs_soff_t fw_file_size = fw_file.size();

		if (fw_file_size < static_cast<lfs_soff_t>(SMD_FW_HEADER_SIZE)) {
			DEBUG_ERROR("OTAFlashFileUpdater::apply_file_update: SMD firmware file too small");
			m_file_size = 0;
			return;
		}

		// Read header [size:4B BE][stm32_crc32:4B BE]
		uint8_t header[SMD_FW_HEADER_SIZE];
		fw_file.read(header, SMD_FW_HEADER_SIZE);

		// Parse big-endian values (matches BLE OTA protocol)
		uint32_t fw_size = (static_cast<uint32_t>(header[0]) << 24) | (static_cast<uint32_t>(header[1]) << 16)
		                   | (static_cast<uint32_t>(header[2]) << 8) | (static_cast<uint32_t>(header[3]));
		uint32_t stm32_crc32 = (static_cast<uint32_t>(header[4]) << 24) | (static_cast<uint32_t>(header[5]) << 16)
		                       | (static_cast<uint32_t>(header[6]) << 8) | (static_cast<uint32_t>(header[7]));

		DEBUG_INFO("OTAFlashFileUpdater: SMD firmware size=%u | STM32 CRC32=0x%08X", fw_size, stm32_crc32);

		// Verify size matches
		if (fw_size != static_cast<uint32_t>(fw_file_size - SMD_FW_HEADER_SIZE)) {
			DEBUG_ERROR("OTAFlashFileUpdater: SMD firmware size mismatch: header=%u | actual=%u", fw_size,
			            static_cast<uint32_t>(fw_file_size - SMD_FW_HEADER_SIZE));
			m_file_size = 0;
			return;
		}

		// Check if SMD instance is available
		if (!smd_sat_instance) {
			DEBUG_ERROR("OTAFlashFileUpdater: SMD satellite instance not available");
			m_file_size = 0;
			return;
		}

		// Keep SMD running — dfu_enter() sends CMD 0x3F to the running app
		// which triggers a software reset into bootloader mode.
		// No hardware power-off/reset needed for DFU.
		DEBUG_INFO("OTAFlashFileUpdater: Starting SMD DFU (streamed, SMD stays running)...");

		// LED feedback: alternate BLUE/WHITE during DFU
		if (status_led) status_led->flash_alternate(RGBLedColor::BLUE, RGBLedColor::WHITE, 250);

		SmdDfuResponse result = smd_sat_instance->firmware_update(&fw_file, fw_size, stm32_crc32, [](uint8_t percent) {
			DEBUG_INFO("SMD DFU progress: %u%%", percent);
			PMU::kick_watchdog();
		});

		if (result == DFU_RSP_OK) {
			DEBUG_INFO("OTAFlashFileUpdater: SMD DFU completed successfully");
			led_handle::dispatch<SetLEDOTASuccess>({});

			// Send firmware version back to pylinkit via BLE
			std::string new_version = smd_sat_instance->get_new_firmware_version();
			if (!new_version.empty()) {
				DEBUG_INFO("OTAFlashFileUpdater: New SMD firmware version: %s", new_version.c_str());
			}
			std::string resp = DTEEncoder::encode(DTECommand::SMDDFU_RESP,
			                                      (unsigned int)0,    // error_code: OK
			                                      (unsigned int)0,    // status: success
			                                      (bool)false,        // dfu_mode: exited DFU
			                                      (unsigned int)100,  // progress: 100%
			                                      new_version);       // info: firmware version
			if (ble_service) ble_service->write(resp);

			// Remove firmware file after successful update
			m_filesystem->remove("smd_firmware.dat");
		} else {
			DEBUG_ERROR("OTAFlashFileUpdater: SMD DFU failed with error %d", result);
			led_handle::dispatch<SetLEDOTAFailed>({});

			// Notify pylinkit of DFU failure
			std::string resp =
			    DTEEncoder::encode(DTECommand::SMDDFU_RESP,
				                   (unsigned int)0,  // error_code: OK (command was valid)
				                   (unsigned int)1,  // status: failure
				                   (bool)false,      // dfu_mode
				                   (unsigned int)0,  // progress
				                   std::string("DFU failed: error " + std::to_string(static_cast<int>(result))));
			if (ble_service) ble_service->write(resp);
		}

		// Restore LED to BLE connected state after 3s feedback
		PMU::delay_ms(3000);
		led_handle::dispatch<SetLEDConfigConnected>({});
	}
#endif
	else {
		delete m_file;
		m_file = nullptr;
	}
	m_file_size = 0;
}
