/**
 * @file nrf_dfu.cpp
 * @brief Nordic DFU settings implementation — stages OTA updates for the bootloader.
 */

#include "dfu.hpp"
#include "nrf_dfu_settings.h"
#include "nrf_dfu_types.h"
#include "debug.hpp"

/// @brief Load and validate the DFU settings page from internal flash.
void DFU::initialise() {
	ret_code_t ret = nrf_dfu_settings_init(true);
	if (ret != NRF_SUCCESS) {
		DEBUG_ERROR("DFU settings init failed: 0x%08X", ret);
	}
}

/// @brief Write bank_1 settings for external flash OTA and persist to flash.
void DFU::write_ext_flash_dfu_settings(uint32_t src_addr, uint32_t image_size, uint32_t crc) {
	DEBUG_TRACE("DFU::write_ext_flash_dfu_settings: updating");
	s_dfu_settings.bank_1.image_size = image_size;
	s_dfu_settings.bank_1.image_crc = crc;
	s_dfu_settings.bank_1.bank_code = NRF_DFU_BANK_VALID_EXT_APP;
	s_dfu_settings.progress.update_start_address = src_addr;
	s_dfu_settings.write_offset = 0;

	ret_code_t ret = nrf_dfu_settings_write_and_backup(nullptr);
	if (ret != NRF_SUCCESS) {
		DEBUG_ERROR("DFU settings write failed: 0x%08X", ret);
	}
}

/// @brief Check if a validated firmware update is staged in bank_1.
bool DFU::update_pending() {
	return s_dfu_settings.bank_1.bank_code == NRF_DFU_BANK_VALID_EXT_APP;
}
