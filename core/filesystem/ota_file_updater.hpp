/**
 * @file ota_file_updater.hpp
 * @brief Abstract OTA file update interface — start, write, complete, apply firmware updates.
 */

#pragma once

#include <cstdint>
#include "filesystem.hpp"

/// @brief OTA file type identifiers (sent by pylinkit during BLE OTA).
// clang-format off
enum class OTAFileIdentifier {
	MCU_FIRMWARE      = 0,  ///< nRF52840 firmware update
	ARTIC_FIRMWARE    = 1,  ///< ARTIC-R2 firmware update //Deprecated - no longer supported, but keep identifier for backward compatibility with older pylinkit versions
	GPS_CONFIG        = 2,  ///< GPS configuration file
	SMD_FIRMWARE_UART = 3,  ///< SMD firmware update via UART (AT commands)
	SMD_FIRMWARE_SPI  = 4   ///< SMD firmware update via SPI (Protocol A+)
};
// clang-format on

/// @brief Abstract OTA file updater — manages file transfer lifecycle.
class OTAFileUpdater {
public:
	virtual ~OTAFileUpdater() {}

	/// @brief Start a new file transfer (erase target, validate size).
	virtual void start_file_transfer(OTAFileIdentifier file_id, const lfs_size_t length, const uint32_t crc32) = 0;

	/// @brief Write a chunk of file data (streaming, CRC updated incrementally).
	virtual void write_file_data(void *const data, lfs_size_t length) = 0;

	/// @brief Abort an in-progress transfer and clean up.
	virtual void abort_file_transfer() = 0;

	/// @brief Has a transfer been started but not fully received yet?
	///
	/// Exists to separate two states that look identical from the outside. A
	/// successful complete_file_transfer() deliberately does NOT clear the
	/// transfer state -- apply_file_update() still needs it, and for
	/// MCU_FIRMWARE that call is DEFERRED onto the scheduler. So "a transfer
	/// exists" is true both for an upload interrupted midway and for one that
	/// is finished and merely waiting to be applied. Aborting the first is
	/// necessary (it otherwise leaks an open file handle and refuses every
	/// later transfer); aborting the second ERASES the staged firmware header
	/// and silently loses the update.
	///
	/// Default false so that an updater which cannot tell the two apart is
	/// never aborted on someone else's behalf.
	virtual bool is_transfer_incomplete() const { return false; }

	/// @brief Complete transfer — verify CRC, finalize file.
	virtual void complete_file_transfer() = 0;

	/// @brief Apply the update (reboot for MCU, DFU for SMD, etc.).
	virtual void apply_file_update() = 0;
};
