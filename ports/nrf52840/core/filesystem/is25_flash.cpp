/**
 * @file is25_flash.cpp
 * @brief IS25LP128F QSPI NOR flash driver implementation.
 */

#include <cstddef>
#include "IS25LP128F.hpp"
#include "nrf_peripheral_power.hpp"
#include "bsp.hpp"
#include "debug.hpp"
#include "gpio.hpp"
#include "interrupt_lock.hpp"
#include "is25_flash.hpp"
#include "nrf_delay.h"


namespace {

/// @brief Longest internal write cycle we may have to ride out at boot.
///
/// A reset (watchdog, brown-out, operator) in the middle of a LittleFS sector
/// erase leaves the die self-timing that erase from its own charge pump.  It
/// survives the CPU reset, and until it finishes the flash answers nothing but
/// RDSR — a JEDEC ID read returns garbage.  IS25LP128F specifies 300 ms max for
/// a 4 KB sector erase (the granularity we use, see _sync()), so the 50 ms this
/// file used to allow was six times too short: the boot that followed any
/// interrupted write — a satellite AOP table being saved, a log flush, a config
/// save — failed on a perfectly healthy part and dropped main() into a
/// permanent red blink loop.  Field occurrence 2026-08-25.  2 s leaves margin
/// for a hot die and for the 64 KB block erase path.
constexpr uint32_t WIP_BOOT_TIMEOUT_US = 2000000;

/// @brief WRSR (status register write) is quick — 15 ms typical worst case.
constexpr uint32_t WRSR_TIMEOUT_US = 50000;

/// @brief RDSR poll interval while waiting for WIP.
constexpr uint32_t WIP_POLL_US = 50;

/// @brief Settling time after a software reset (tRST is tens of us; be generous).
constexpr uint32_t RESET_RECOVERY_MS = 10;

/// @brief Full bring-up attempts before init() gives up.
constexpr unsigned int INIT_MAX_ATTEMPTS = 3;

/// @brief Common cinstr scaffolding: IO3 high (it doubles as RESET# while the
/// die is in SPI mode), no automatic WIP wait (we do our own, bounded).
inline nrf_qspi_cinstr_conf_t cinstr_base() {
	nrf_qspi_cinstr_conf_t config;
	config.opcode = 0;
	config.length = NRF_QSPI_CINSTR_LEN_1B;
	config.io2_level = false;
	config.io3_level = true;
	config.wipwait = false;
	config.wren = false;
	return config;
}

}  // namespace


bool Is25Flash::wait_wip_clear(uint32_t timeout_us, uint32_t &elapsed_us) {
	nrf_qspi_cinstr_conf_t config = cinstr_base();
	uint8_t rx_buffer[3];

	config.opcode = IS25LP128F::RDSR;
	config.length = NRF_QSPI_CINSTR_LEN_2B;

	elapsed_us = 0;
	for (;;) {
		rx_buffer[0] = 0xFF;
		if (nrfx_qspi_cinstr_xfer(&config, nullptr, rx_buffer) != NRFX_SUCCESS) return false;

		if (!(rx_buffer[0] & IS25LP128F::STATUS_WIP)) return true;

		if (elapsed_us >= timeout_us) return false;

		nrf_delay_us(WIP_POLL_US);
		elapsed_us += WIP_POLL_US;
	}
}


void Is25Flash::software_reset() {
	// RSTEN then RST. This is the only in-band recovery available to us: the
	// IS25 shares the main 3V3 rail (no dedicated load switch to cycle) and its
	// hardware RESET# on IO3 is disabled once the non-volatile QE bit is set —
	// which it is on every device that has booted at least once.
	nrf_qspi_cinstr_conf_t config = cinstr_base();

	config.opcode = IS25LP128F::RSTEN;
	(void)nrfx_qspi_cinstr_xfer(&config, nullptr, nullptr);

	config.opcode = IS25LP128F::RST;
	(void)nrfx_qspi_cinstr_xfer(&config, nullptr, nullptr);

	nrf_delay_ms(RESET_RECOVERY_MS);
}


bool Is25Flash::init_attempt(unsigned int attempt) {
	nrf_qspi_cinstr_conf_t config;
	uint8_t rx_buffer[3];
	uint8_t tx_buffer[1];
	uint32_t waited = 0;

	// If setting the SO/io1 pin to an input is not done then the nrfx_qspi_init() timesout when the board
	// is reprogrammed using a JLink. It is unclear why this happens but having this in place causes no harm.
	// This was found through experimentation and it is possible the root cause is merely masked by this.
	nrf_gpio_cfg_input(BSP::QSPI_Inits[BSP::QSPI_0].config.pins.io1_pin, NRF_GPIO_PIN_PULLDOWN);

	nrfx_err_t ret = nrfx_qspi_init(&BSP::QSPI_Inits[BSP::QSPI_0].config, nullptr, nullptr);
	if (ret != NRFX_SUCCESS) {
		// nrfx_qspi_init() sets m_cb.state = INITIALIZED *before* it waits for
		// the peripheral to be ready, so a NRFX_ERROR_TIMEOUT leaves the driver
		// claiming it is up. Without this uninit, attempts 2 and 3 would get
		// NRFX_ERROR_INVALID_STATE straight away and the software-reset
		// recovery below would never run — the retry loop would be decorative.
		// INVALID_PARAM is the one failure that returns before the state is
		// set; uninit would then be operating on an uninitialised driver.
		if (ret != NRFX_ERROR_INVALID_PARAM) nrfx_qspi_uninit();
		nrf_peripheral_power_reset(NRF_QSPI_BASE_ADDR);
		DEBUG_ERROR("IS25LP128F QSPI initialisation failure - %d", ret);
		return false;
	}

	config = cinstr_base();

	// Issue a wake-up command in case the device is in deep sleep
	config.opcode = IS25LP128F::RDPD;
	config.length = NRF_QSPI_CINSTR_LEN_1B;
	nrfx_qspi_cinstr_xfer(&config, nullptr, nullptr);

	nrf_delay_ms(1);

	// Retries start by resetting the die: whatever made the previous attempt
	// fail (stuck state machine, stale QPI mode, half-decoded command) is
	// exactly what a software reset is for.
	if (attempt > 1) software_reset();

	// Ride out any program/erase the die is still finishing from before we
	// booted. This MUST come before the JEDEC ID read: while WIP is set the
	// part answers RDSR and nothing else, so the ID check below would fail on a
	// perfectly good device and take the whole board down with it.
	if (!wait_wip_clear(WIP_BOOT_TIMEOUT_US, waited)) {
		nrfx_qspi_uninit();
		nrf_peripheral_power_reset(NRF_QSPI_BASE_ADDR);
		DEBUG_ERROR("IS25LP128F: WIP still set after %lu us — die busy or unresponsive", (unsigned long)waited);
		return false;
	}
	if (waited) DEBUG_WARN("IS25LP128F: rode out an interrupted write/erase (%lu us)", (unsigned long)waited);

	// Read and check the SPI device ID matches the expected value
	config.opcode = IS25LP128F::RDJDID;
	config.length = NRF_QSPI_CINSTR_LEN_4B;
	config.wren = false;

	rx_buffer[0] = rx_buffer[1] = rx_buffer[2] = 0;
	nrfx_qspi_cinstr_xfer(&config, nullptr, rx_buffer);

	if (rx_buffer[0] != IS25LP128F::MANUFACTURER_ID || rx_buffer[1] != IS25LP128F::MEMORY_TYPE_ID
	    || rx_buffer[2] != IS25LP128F::CAPACITY_ID) {
		nrfx_qspi_uninit();
		nrf_peripheral_power_reset(NRF_QSPI_BASE_ADDR);
		DEBUG_ERROR("IS25LP128F not correctly identified (read %02X %02X %02X, expected %02X %02X %02X)", rx_buffer[0],
		            rx_buffer[1], rx_buffer[2], IS25LP128F::MANUFACTURER_ID, IS25LP128F::MEMORY_TYPE_ID,
		            IS25LP128F::CAPACITY_ID);
		return false;
	}

	// Switch to QSPI mode. QE lives in the NON-VOLATILE status register, so on
	// every boot after the first it is already set: skip the write. That spares
	// the register a program cycle per boot and — more to the point — removes a
	// ~15 ms window per boot during which a reset would leave WIP stuck.
	//
	// The skip condition deliberately checks the WHOLE register, not just QE.
	// The unconditional write this replaces put SR = STATUS_QE, which also
	// CLEARED the block-protect bits (BP0..BP3) and SRWD. Skipping on "QE set"
	// alone would let a latched block-protect survive every boot while init()
	// still reported success — the filesystem would then fail to write with no
	// explanation. Re-normalise unless the register is exactly what we want.
	constexpr uint8_t SR_PROTECT_MASK = IS25LP128F::STATUS_BP0 | IS25LP128F::STATUS_BP1 | IS25LP128F::STATUS_BP2
	                                    | IS25LP128F::STATUS_BP3 | IS25LP128F::STATUS_SRWD;
	config.opcode = IS25LP128F::RDSR;
	config.length = NRF_QSPI_CINSTR_LEN_2B;
	config.wren = false;
	rx_buffer[0] = 0;
	nrfx_qspi_cinstr_xfer(&config, nullptr, rx_buffer);

	if (!(rx_buffer[0] & IS25LP128F::STATUS_QE) || (rx_buffer[0] & SR_PROTECT_MASK)) {
		if (rx_buffer[0] & SR_PROTECT_MASK)
			DEBUG_WARN("IS25LP128F: status register had protection bits set (%02X) — normalising", rx_buffer[0]);
		config.opcode = IS25LP128F::WRSR;
		tx_buffer[0] = IS25LP128F::STATUS_QE;
		config.length = NRF_QSPI_CINSTR_LEN_2B;
		config.wren = true;
		nrfx_qspi_cinstr_xfer(&config, tx_buffer, nullptr);

		// Wait for QSPI mode to be programmed (bounded — WRSR typically < 15 ms)
		if (!wait_wip_clear(WRSR_TIMEOUT_US, waited)) {
			nrfx_qspi_uninit();
			nrf_peripheral_power_reset(NRF_QSPI_BASE_ADDR);
			DEBUG_ERROR("IS25LP128F WRSR timeout (WIP stuck after %lu us)", (unsigned long)waited);
			return false;
		}
	}

	// init() manages QSPI directly, use hardware-level power down
	_power_down_hw();

	m_is_init = true;
	return true;
}


#ifdef BENCH_TEST

uint8_t Is25Flash::bench_status() {
	power_up();
	nrf_qspi_cinstr_conf_t config = cinstr_base();
	uint8_t rx[3] = { 0, 0, 0 };
	config.opcode = IS25LP128F::RDSR;
	config.length = NRF_QSPI_CINSTR_LEN_2B;
	(void)nrfx_qspi_cinstr_xfer(&config, nullptr, rx);
	power_down();
	return rx[0];
}

bool Is25Flash::bench_software_reset(uint8_t jedec[3], uint8_t &status) {
	power_up();

	software_reset();

	nrf_qspi_cinstr_conf_t config = cinstr_base();
	uint8_t rx[3] = { 0, 0, 0 };

	config.opcode = IS25LP128F::RDJDID;
	config.length = NRF_QSPI_CINSTR_LEN_4B;
	(void)nrfx_qspi_cinstr_xfer(&config, nullptr, rx);
	jedec[0] = rx[0];
	jedec[1] = rx[1];
	jedec[2] = rx[2];

	// A reset returns the die to its power-on state. QE is non-volatile (the
	// runtime power_up path never re-writes it, yet quad reads keep working),
	// but re-assert it if it did come back clear — otherwise this probe would
	// leave the filesystem unable to read until the next reboot.
	config.opcode = IS25LP128F::RDSR;
	config.length = NRF_QSPI_CINSTR_LEN_2B;
	config.wren = false;
	rx[0] = 0;
	(void)nrfx_qspi_cinstr_xfer(&config, nullptr, rx);
	status = rx[0];

	if (!(status & IS25LP128F::STATUS_QE)) {
		uint8_t tx[1] = { IS25LP128F::STATUS_QE };
		uint32_t waited = 0;
		DEBUG_WARN("IS25LP128F bench: software reset cleared QE — re-asserting");
		config.opcode = IS25LP128F::WRSR;
		config.length = NRF_QSPI_CINSTR_LEN_2B;
		config.wren = true;
		(void)nrfx_qspi_cinstr_xfer(&config, tx, nullptr);
		(void)wait_wip_clear(WRSR_TIMEOUT_US, waited);
	}

	power_down();

	return jedec[0] == IS25LP128F::MANUFACTURER_ID && jedec[1] == IS25LP128F::MEMORY_TYPE_ID
	       && jedec[2] == IS25LP128F::CAPACITY_ID;
}

#endif  // BENCH_TEST


bool Is25Flash::init() {
	for (unsigned int attempt = 1; attempt <= INIT_MAX_ATTEMPTS; attempt++) {
		if (init_attempt(attempt)) {
			if (attempt > 1) DEBUG_WARN("IS25LP128F: recovered on attempt %u/%u", attempt, INIT_MAX_ATTEMPTS);
			return true;
		}

		if (attempt < INIT_MAX_ATTEMPTS)
			DEBUG_WARN("IS25LP128F: bring-up attempt %u/%u failed — retrying with a software reset", attempt,
			           INIT_MAX_ATTEMPTS);
	}

	DEBUG_ERROR("IS25LP128F: bring-up failed after %u attempts", INIT_MAX_ATTEMPTS);
	return false;
}


/**
 * @brief Internal read — QSPI must be powered up by caller.
 * @param block   LFS block number.
 * @param off     Byte offset within the block.
 * @param buffer  Destination buffer (must be word-aligned).
 * @param size    Number of bytes to read (must be multiple of 4, max 0x3FFFF).
 * @return LFS_ERR_OK on success, LFS_ERR_IO or LFS_ERR_INVAL on failure.
 */
int Is25Flash::_read(lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size) {
	if (((intptr_t)buffer & 3) || (size & 3)) return LFS_ERR_INVAL;

	int ret_sync = _sync();
	if (ret_sync != LFS_ERR_OK) return ret_sync;

	nrfx_err_t ret = nrfx_qspi_read(buffer, size, block * m_block_size + off);
	if (ret != NRFX_SUCCESS) {
		DEBUG_ERROR("QSPI IO Error %04x", ret);
		return LFS_ERR_IO;
	}

	return LFS_ERR_OK;
}

/**
 * @brief Internal program with sync and read-back verification.
 * @param block   LFS block number.
 * @param off     Byte offset within the block.
 * @param buffer  Source data buffer (must be word-aligned).
 * @param size    Number of bytes to write (must be multiple of 4, max IS25_PAGE_SIZE).
 * @return LFS_ERR_OK, LFS_ERR_IO, LFS_ERR_NOMEM, or LFS_ERR_CORRUPT.
 * @note Verification is limited to IS25_PAGE_SIZE (256 bytes).
 */
int Is25Flash::_prog(lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size) {
	int ret_sync;

	if (((intptr_t)buffer & 3) || (size & 3)) return LFS_ERR_INVAL;

	ret_sync = _sync();
	if (ret_sync != LFS_ERR_OK) return ret_sync;

	nrfx_err_t ret_write = nrfx_qspi_write(buffer, size, block * m_block_size + off);
	if (ret_write != NRFX_SUCCESS) {
		DEBUG_ERROR("QSPI IO Error %04x", ret_write);
		return LFS_ERR_IO;
	}

	// Wait for the write to complete before verifying
	ret_sync = _sync();
	if (ret_sync != LFS_ERR_OK) return ret_sync;

	// Read-back verification
	uint8_t read_buffer[IS25_PAGE_SIZE];

	if (size > sizeof(read_buffer)) {
		DEBUG_ERROR("QSPI Flash prog verification buffer too small | need size of %lu", size);
		return LFS_ERR_NOMEM;
	}

	int ret_read = _read(block, off, &read_buffer[0], size);
	if (ret_read != LFS_ERR_OK) return ret_read;

	if (memcmp(reinterpret_cast<const uint8_t *>(buffer), &read_buffer[0], size)) {
		DEBUG_ERROR("QSPI Flash prog reported a bad write");
#if (DEBUG_LEVEL >= 1)
		for (unsigned int i = 0; i < size; i++)
			printf("%02X", static_cast<const uint8_t *>(buffer)[i]);
		printf("\r\n");
		for (unsigned int i = 0; i < size; i++)
			printf("%02X", read_buffer[i]);
		printf("\r\n");
#endif
		return LFS_ERR_CORRUPT;
	}

	return LFS_ERR_OK;
}

/**
 * @brief Fast program without sync or read-back verification — for OTA transfers.
 * @note Caller must manage power and sync externally.
 */
int Is25Flash::_prog_fast(lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size) {
	if (((intptr_t)buffer & 3) || (size & 3)) return LFS_ERR_INVAL;

	nrfx_err_t ret_write = nrfx_qspi_write(buffer, size, block * m_block_size + off);
	if (ret_write != NRFX_SUCCESS) {
		DEBUG_ERROR("QSPI IO Error %04x", ret_write);
		return LFS_ERR_IO;
	}

	return LFS_ERR_OK;
}

/** @brief Internal erase — erases one 4 KB sector. */
int Is25Flash::_erase(lfs_block_t block) {
	int ret_sync = _sync();
	if (ret_sync != LFS_ERR_OK) return ret_sync;

	nrfx_err_t ret_erase = nrfx_qspi_erase(NRF_QSPI_ERASE_LEN_4KB, block * m_block_size);
	if (ret_erase != NRFX_SUCCESS) {
		DEBUG_ERROR("QSPI IO Error %04x", ret_erase);
		return LFS_ERR_IO;
	}

	return LFS_ERR_OK;
}

/**
 * @brief Wait for any in-progress write/erase to complete.
 * @return LFS_ERR_OK on success, LFS_ERR_IO on timeout (300 ms).
 */
int Is25Flash::_sync() {
	nrfx_err_t ret;

	// Deliberately left at 300 ms — the IS25LP128F maximum for a 4 KB sector
	// erase — even though that leaves no margin. This wait is REACHABLE FROM
	// SOFTDEVICE INTERRUPT CONTEXT: a BLE OTA write lands in
	// stm_ota_event_handler (NRF_SDH_DISPATCH_MODEL = INTERRUPT) and runs
	// ota_updater->write_file_data() straight through to LittleFS. Raising the
	// budget would extend a busy-wait inside the radio's own IRQ and put the
	// BLE link at risk. A rare timeout here surfaces as LFS_ERR_IO, which the
	// caller handles; a one-second stall in the SoftDevice IRQ does not.
	// The bring-up path has its own, much longer budget — see WIP_BOOT_TIMEOUT_US.
	constexpr uint32_t WAIT_TIME_US = 10;
	constexpr uint32_t WAIT_ATTEMPTS = 30000;

	uint32_t remaining_attempts = WAIT_ATTEMPTS;
	do {
		ret = nrfx_qspi_mem_busy_check();
		if (ret == NRFX_SUCCESS) break;

		nrf_delay_us(WAIT_TIME_US);
	} while (--remaining_attempts);

	if (ret != NRFX_SUCCESS) {
		DEBUG_ERROR("QSPI IO Sync %04x", ret);
		return LFS_ERR_IO;
	}

	return LFS_ERR_OK;
}

/// @name Public LFS wrappers — guard with init check and power management
/// @{

int Is25Flash::read(lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size) {
	if (!m_is_init) return LFS_ERR_IO;
	power_up();
	int ret = _read(block, off, buffer, size);
	power_down();
	return ret;
}

int Is25Flash::prog(lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size) {
	if (!m_is_init) return LFS_ERR_IO;
	power_up();
	int ret = _prog(block, off, buffer, size);
	power_down();
	return ret;
}

int Is25Flash::erase(lfs_block_t block) {
	if (!m_is_init) return LFS_ERR_IO;
	power_up();
	int ret = _erase(block);
	power_down();
	return ret;
}

int Is25Flash::sync() {
	if (!m_is_init) return LFS_ERR_IO;
	power_up();
	int ret = _sync();
	power_down();
	return ret;
}

int Is25Flash::prog_fast(lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size) {
	if (!m_is_init) return LFS_ERR_IO;
	// NO power management here — caller must handle it for performance. That
	// now also means the VSYS interlock: power_up()/power_down() are what keep
	// the rail at 3.3 V, and this die must never be programmed at the 2.3 V
	// idle rail. Currently unused; wrap any new caller in power_up/power_down.
	return _prog_fast(block, off, buffer, size);
}

/// @}

/** @brief Wake QSPI peripheral and send RDPD (Release from Deep Power-Down). */
void Is25Flash::_power_up_hw() {
	nrfx_err_t ret = nrfx_qspi_init(&BSP::QSPI_Inits[BSP::QSPI_0].config, nullptr, nullptr);
	if (ret != NRFX_SUCCESS) {
		DEBUG_ERROR("QSPI power_up init failure - %d", ret);
		return;
	}

	const nrf_qspi_cinstr_conf_t config = { .opcode = IS25LP128F::RDPD,  // Wake from deep power mode
	                                        .length = NRF_QSPI_CINSTR_LEN_1B,
	                                        .io2_level = false,
	                                        .io3_level = true,
	                                        .wipwait = false,
	                                        .wren = false };

	nrfx_qspi_cinstr_xfer(&config, nullptr, nullptr);

	nrf_delay_us(5);
}

/**
 * @brief Sync, send DP (Deep Power-Down), uninit QSPI, and float all QSPI pins.
 *
 * After this call the QSPI peripheral draws zero current.  CS is held high
 * to ensure the flash chip stays deselected.  All other pins are pulled down.
 */
void Is25Flash::_power_down_hw() {
	// Wait for any write/erase to finish before sleeping the die.
	//
	// NOT _sync(): its 300 ms budget is capped by the SoftDevice IRQ path (a BLE
	// OTA write reaches it from the radio's own interrupt -- see _sync), and a
	// 4 KB sector erase already spends all of it. This path only ever runs from
	// the main loop, so it can afford the same 2 s the bring-up path uses.
	//
	// It matters because of what happens next: the caller releases the flash
	// interlock, and PMU::reduce_power_rails() then takes VSYS down to 2.3 V --
	// the die's absolute minimum -- about 250 ms later. Putting a die that is
	// still programming to sleep and then browning it out is what left WIP armed
	// for good on 2026-08-25 and 2026-08-29. With no load switch and no usable
	// RESET#, that state is unrecoverable.
	uint32_t waited = 0;
	if (!wait_wip_clear(WIP_BOOT_TIMEOUT_US, waited))
		DEBUG_ERROR("IS25LP128F: WIP still set after %lu us at power-down -- die left mid-program",
		            (unsigned long)waited);

	const nrf_qspi_cinstr_conf_t config = { .opcode = IS25LP128F::DP,  // Enter deep power mode
	                                        .length = NRF_QSPI_CINSTR_LEN_1B,
	                                        .io2_level = false,
	                                        .io3_level = true,
	                                        .wipwait = false,
	                                        .wren = false };

	nrfx_qspi_cinstr_xfer(&config, nullptr, nullptr);

	nrf_delay_us(3);

	// Errata 122: QSPI draws excess current after TASKS_ACTIVATE unless these
	// registers are written before uninit.  Must be done while QSPI is still active.
	*reinterpret_cast<volatile uint32_t *>(NRF_QSPI_BASE_ADDR + 0x010) = 1;
	*reinterpret_cast<volatile uint32_t *>(NRF_QSPI_BASE_ADDR + 0x054) = 1;

	nrfx_qspi_uninit();
	nrf_peripheral_power_reset(NRF_QSPI_BASE_ADDR);  // Full peripheral reset

	// Float QSPI pins to minimize leakage current
	nrf_gpio_cfg_output(BSP::QSPI_Inits[BSP::QSPI_0].config.pins.csn_pin);
	nrf_gpio_pin_set(BSP::QSPI_Inits[BSP::QSPI_0].config.pins.csn_pin);
	nrf_gpio_cfg_input(BSP::QSPI_Inits[BSP::QSPI_0].config.pins.sck_pin, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(BSP::QSPI_Inits[BSP::QSPI_0].config.pins.io0_pin, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(BSP::QSPI_Inits[BSP::QSPI_0].config.pins.io1_pin, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(BSP::QSPI_Inits[BSP::QSPI_0].config.pins.io2_pin, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(BSP::QSPI_Inits[BSP::QSPI_0].config.pins.io3_pin, NRF_GPIO_PIN_PULLDOWN);
}

/// The reference count is shared with SOFTDEVICE INTERRUPT CONTEXT: a BLE OTA
/// write reaches LittleFS -- and so prog() and power_up() -- straight out of
/// BleInterface::stm_ota_event_handler, which the file itself documents as
/// running in interrupt context (NRF_SDH_DISPATCH_MODEL = 0 in sdk_config.h).
/// A bare ++/-- is a read-modify-write and silently loses one when the ISR
/// lands in the middle. A lost increment lets power_down() reach zero while the
/// main loop is still programming, which releases the rail interlock mid-erase
/// -- precisely the brick the interlock exists to prevent.
///
/// The lock covers ONLY the counter and the flag. The hardware transitions stay
/// outside it: _power_down_hw() can wait up to 2 s for WIP, and disabling
/// interrupts for that long would break the SoftDevice.
void Is25Flash::power_up() {
	bool bring_up;
	{
		InterruptLock lock;
		// Interlock asserted BEFORE the count moves: reduce_power_rails() runs
		// from the scheduler and must never see the flash in service at 2.3 V.
		GPIOPins::set_flash_busy(true);
		// Explicit read-modify-write: ++/-- on a volatile is deprecated in C++20
		// (-Werror=volatile). Safe here — the whole sequence is inside the lock.
		const unsigned int prev = m_power_ref_count;
		m_power_ref_count = prev + 1;
		bring_up = (prev == 0);
	}
	if (bring_up) _power_up_hw();
}

void Is25Flash::power_down() {
	bool shut_down;
	{
		InterruptLock lock;
		if (m_power_ref_count == 0) return;
		const unsigned int next = m_power_ref_count - 1;
		m_power_ref_count = next;
		shut_down = (next == 0);
	}
	if (shut_down) {
		_power_down_hw();
		// Release the interlock only if nobody re-acquired the flash while the
		// die was going to sleep -- otherwise we would clear a flag that an
		// interrupt has just legitimately raised.
		InterruptLock lock;
		if (m_power_ref_count == 0) GPIOPins::set_flash_busy(false);
	}
}
