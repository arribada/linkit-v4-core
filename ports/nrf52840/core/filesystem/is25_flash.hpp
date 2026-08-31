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

#include <cstdint>

#include "filesystem.hpp"

#define IS25_BLOCK_COUNT (4096)      ///< Total 4 KB blocks (16 MB / 4 KB)
#define IS25_BLOCK_SIZE  (4 * 1024)  ///< Erase granularity: 4 KB sector
#define IS25_PAGE_SIZE   (256)       ///< Program page size: 256 bytes

class Is25Flash : public FlashInterface {
public:
	Is25Flash()
	    : FlashInterface(IS25_BLOCK_COUNT, IS25_BLOCK_SIZE, IS25_PAGE_SIZE),
	      m_is_init(false),
	      m_power_ref_count(0) {}

	/**
	 * @brief Initialise the IS25LP128F: QSPI peripheral, device ID check, QSPI mode enable.
	 *
	 * Retries the whole sequence up to INIT_MAX_ATTEMPTS times, issuing a JEDEC
	 * software reset to the die between attempts.  See init_attempt() for why a
	 * plain single-shot init is not enough after an interrupted write.
	 *
	 * @return true on success, false if the device could not be brought up.
	 */
	[[nodiscard]] bool init();

	/// @brief Returns true if init() completed successfully.
	bool is_init() const { return m_is_init; }

private:
	bool m_is_init;
	volatile unsigned int m_power_ref_count;  ///< Shared with ISR — see power_up()

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

	/// @name Bring-up and recovery
	/// @{

	/// @brief One full bring-up attempt (QSPI init, wake, ID check, QE bit).
	/// @param attempt  1-based attempt number; attempts after the first begin
	///                 with a software reset of the die.
	/// @return true on success.  On failure the QSPI peripheral is left
	///         uninitialised and power-cycled, ready for the next attempt.
	bool init_attempt(unsigned int attempt);

	/// @brief Poll RDSR until the write-in-progress bit clears.
	/// @param timeout_us   Maximum time to wait.
	/// @param elapsed_us   Out: time actually spent waiting.
	/// @return true if WIP is clear, false on timeout or transfer error.
	bool wait_wip_clear(uint32_t timeout_us, uint32_t &elapsed_us);

	/// @brief JEDEC software reset of the die (RSTEN then RST).
	/// @note Requires the QSPI peripheral to be initialised.
	void software_reset();
	/// @}

public:
#ifdef BENCH_TEST
	/// @brief Bench probe: read the status register (bit0 WIP, bit6 QE).
	uint8_t bench_status();

	/// @brief Bench probe: exercise the recovery path on real silicon.
	///
	/// Issues RSTEN+RST and checks the die answers its JEDEC ID afterwards.
	/// Non-destructive: quad mode is re-asserted if the reset cleared it, so the
	/// filesystem keeps working after the probe.
	/// @param jedec   Out: the three ID bytes read back after the reset.
	/// @param status  Out: the status register read back after the reset.
	/// @return true if the die identified correctly after the software reset.
	bool bench_software_reset(uint8_t jedec[3], uint8_t &status);
#endif

private:
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
