/**
 * @file smd_dfu_image.hpp
 * @brief Check of the staged SMD firmware image before a DFU starts.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "filesystem.hpp"
#include "smd_sat_registers.hpp"

/// @brief Does the staged SMD image match the CRC the host announced?
///
/// CRC-32/MPEG-2, the algorithm the SMD bootloader applies in dfu_verify(), over
/// @p size bytes from the file's current position. The position is restored, so
/// the DFU then streams the very bytes that were checked. A short read counts as
/// a mismatch.
///
/// Run it BEFORE dfu_erase(). The bootloader only checks the CRC once the new
/// image is written, and by then the old application is erased: a staged image
/// damaged in the external flash (writes lost during the BLE transfer,
/// 2026-10-30) left the module without an application, and a module in that state
/// answered nothing on SPI, application and bootloader alike.
inline bool smd_dfu_staged_image_ok(File &file, size_t size, uint32_t expected_crc) {
	const lfs_soff_t start = file.tell();
	if (start < 0) return false;

	uint8_t buf[SMDSAT_DFU_CHUNK_SIZE];
	uint32_t crc = 0xFFFFFFFF;
	size_t remaining = size;
	bool complete = true;
	while (remaining > 0) {
		const lfs_size_t n = (remaining > sizeof(buf)) ? sizeof(buf) : static_cast<lfs_size_t>(remaining);
		if (file.read(buf, n) != static_cast<lfs_ssize_t>(n)) {
			complete = false;
			break;
		}
		crc = spi_crc32_mpeg2_update(crc, buf, n);
		remaining -= n;
	}

	if (file.seek(start, LFS_SEEK_SET) != start) return false;
	return complete && crc == expected_crc;
}
