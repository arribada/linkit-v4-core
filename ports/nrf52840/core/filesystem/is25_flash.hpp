#pragma once

/**
 * @file is25_flash.hpp
 * @brief IS25LP128F QSPI NOR flash driver for LittleFS.
 *
 * Provides a FlashInterface implementation backed by the ISSI IS25LP128F
 * 128 Mbit (16 MB) QSPI NOR flash.  All public read/prog/erase/sync
 * operations are wrapped with reference-counted power management so the
 * QSPI peripheral is only active while an operation is in progress.
 */

#include "filesystem.hpp"

#define IS25_BLOCK_COUNT   (4096)    ///< Total 4 KB blocks (16 MB / 4 KB)
#define IS25_BLOCK_SIZE    (4*1024)  ///< Erase granularity: 4 KB sector
#define IS25_PAGE_SIZE     (256)     ///< Program page size: 256 bytes

class Is25Flash : public FlashInterface {
public:
	// FlashInterface(block_size, blocks, page_size) — pass in the declared order
	// (harmless today only because IS25_BLOCK_SIZE == IS25_BLOCK_COUNT == 4096).
	Is25Flash() : FlashInterface(IS25_BLOCK_SIZE, IS25_BLOCK_COUNT, IS25_PAGE_SIZE),
		m_is_init(false), m_power_ref_count(0) {}

	/**
	 * @brief Initialise the IS25LP128F: QSPI peripheral, device ID check, QSPI mode enable.
	 * @return true on success, false if QSPI init or device ID verification failed.
	 */
	[[nodiscard]] bool init();

	/// @brief Returns true if init() completed successfully.
	bool is_init() const { return m_is_init; }

private:
	bool m_is_init;
	unsigned int m_power_ref_count;

	/// @name LittleFS FlashInterface overrides (called via LFS callbacks)
	/// @{
	int read(lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size) override;
	int prog(lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size) override;
	int erase(lfs_block_t block) override;
	int sync() override;
	/// @}

	/// @name Internal operations (no power management, no init guard)
	/// @{
	int _read(lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size);
	int _prog(lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size);
	int _erase(lfs_block_t block);
	int _sync();
	/// @}

	/// @brief Fast program without sync or read-back verification (for OTA transfers).
	int _prog_fast(lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size);

	/// @brief Wake QSPI peripheral and IS25 from deep power-down.
	void _power_up_hw();

	/// @brief Sync pending operations, enter deep power-down, uninit QSPI, float pins.
	void _power_down_hw();

public:
	/**
	 * @brief Fast program for OTA — no sync, no read-back verification.
	 * @note Caller must manage power_up/power_down manually for performance.
	 */
	int prog_fast(lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size);

	/**
	 * @brief Reference-counted QSPI power management.
	 *
	 * Nested power_up/power_down calls are safe.  The QSPI peripheral is
	 * only initialised on the first power_up and shut down when the last
	 * power_down brings the count to zero.
	 * @{
	 */
	void power_up() override;
	void power_down() override;
	/// @}
};
