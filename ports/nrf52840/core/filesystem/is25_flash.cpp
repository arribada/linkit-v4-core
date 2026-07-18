/**
 * @file is25_flash.cpp
 * @brief IS25LP128F QSPI NOR flash driver implementation.
 */

#include <cstddef>
#include "IS25LP128F.hpp"
#include "nrf_peripheral_power.hpp"
#include "bsp.hpp"
#include "debug.hpp"
#include "is25_flash.hpp"
#include "nrf_delay.h"


bool Is25Flash::init()
{
	nrf_qspi_cinstr_conf_t config;
	uint8_t status;
	uint8_t rx_buffer[3];
	uint8_t tx_buffer[1];

	// If setting the SO/io1 pin to an input is not done then the nrfx_qspi_init() timesout when the board
	// is reprogrammed using a JLink. It is unclear why this happens but having this in place causes no harm.
	// This was found through experimentation and it is possible the root cause is merely masked by this.
	nrf_gpio_cfg_input(BSP::QSPI_Inits[BSP::QSPI_0].config.pins.io1_pin, NRF_GPIO_PIN_PULLDOWN);

	nrfx_err_t ret = nrfx_qspi_init(&BSP::QSPI_Inits[BSP::QSPI_0].config, nullptr, nullptr);
	if (ret != NRFX_SUCCESS)
	{
		DEBUG_ERROR("IS25LP128F QSPI initialisation failure - %d", ret);
		return false;
	}

	config.io2_level = false;
	// Keep IO3 high during transfers as this is the reset line in SPI mode.
	// We'll make use of this line once the FLASH chip is in QSPI mode.
	config.io3_level = true;
	config.wipwait   = false;

	// Wake the device (RDPD) and verify its JEDEC ID. A cold power-up can leave
	// the NOR still completing its own power-on for a few ms, so a single read
	// can transiently mis-identify. Retry a few times with a longer settle
	// before giving up — on a sealed EXTERNAL_WAKEUP board a false "not
	// identified" costs an entire wake cycle (and the caller historically spun
	// forever feeding the watchdog, draining the battery to death).
	constexpr int JEDEC_MAX_ATTEMPTS = 3;
	bool identified = false;
	for (int attempt = 0; attempt < JEDEC_MAX_ATTEMPTS && !identified; attempt++)
	{
		// Issue a wake-up command in case the device is in deep sleep
		config.opcode = IS25LP128F::RDPD;
		config.length = NRF_QSPI_CINSTR_LEN_1B;
		config.wren = false;
		nrfx_qspi_cinstr_xfer(&config, nullptr, nullptr);

		nrf_delay_ms(5);  // settle: RDPD wake + NOR power-on completion

		// Read and check the SPI device ID matches the expected value
		config.opcode = IS25LP128F::RDJDID;
		config.length = NRF_QSPI_CINSTR_LEN_4B;
		config.wren = false;
		nrfx_qspi_cinstr_xfer(&config, nullptr, rx_buffer);

		identified = (rx_buffer[0] == IS25LP128F::MANUFACTURER_ID &&
		              rx_buffer[1] == IS25LP128F::MEMORY_TYPE_ID &&
		              rx_buffer[2] == IS25LP128F::CAPACITY_ID);
		if (!identified)
			DEBUG_WARN("IS25LP128F ID mismatch (attempt %d/%d): %02x %02x %02x",
			           attempt + 1, JEDEC_MAX_ATTEMPTS,
			           rx_buffer[0], rx_buffer[1], rx_buffer[2]);
	}

	if (!identified)
	{
		nrfx_qspi_uninit();
		nrf_peripheral_power_reset(NRF_QSPI_BASE_ADDR);
		DEBUG_ERROR("IS25LP128F not correctly identified after %d attempts", JEDEC_MAX_ATTEMPTS);
		return false;
	}

	// Switch to QSPI mode
	config.opcode = IS25LP128F::WRSR;
	tx_buffer[0] = IS25LP128F::STATUS_QE;
	config.length = NRF_QSPI_CINSTR_LEN_2B;
	config.wren = true;
	nrfx_qspi_cinstr_xfer(&config, tx_buffer, nullptr);

	// Wait for QSPI mode to be programmed (bounded — WRSR typically < 15 ms)
	config.opcode = IS25LP128F::RDSR;
	config.length = NRF_QSPI_CINSTR_LEN_2B;
	config.wren = false;

	constexpr uint32_t WRSR_TIMEOUT_US = 50000;  // 50 ms — well above WRSR max (15 ms)
	constexpr uint32_t WRSR_POLL_US    = 10;
	uint32_t elapsed = 0;
	do
	{
		nrfx_qspi_cinstr_xfer(&config, nullptr, rx_buffer);
		status = rx_buffer[0];
		if (!(status & IS25LP128F::STATUS_WIP))
			break;
		nrf_delay_us(WRSR_POLL_US);
		elapsed += WRSR_POLL_US;
	} while (elapsed < WRSR_TIMEOUT_US);

	if (status & IS25LP128F::STATUS_WIP)
	{
		nrfx_qspi_uninit();
		nrf_peripheral_power_reset(NRF_QSPI_BASE_ADDR);
		DEBUG_ERROR("IS25LP128F WRSR timeout (WIP stuck after %lu us)", elapsed);
		return false;
	}

	// init() manages QSPI directly, use hardware-level power down
	_power_down_hw();

	m_is_init = true;
	return true;
}

/**
 * @brief Internal read — QSPI must be powered up by caller.
 * @param block   LFS block number.
 * @param off     Byte offset within the block.
 * @param buffer  Destination buffer (must be word-aligned).
 * @param size    Number of bytes to read (must be multiple of 4, max 0x3FFFF).
 * @return LFS_ERR_OK on success, LFS_ERR_IO or LFS_ERR_INVAL on failure.
 */
int Is25Flash::_read(lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size)
{
	if (((intptr_t)buffer & 3) || (size & 3))
		return LFS_ERR_INVAL;

	int ret_sync = _sync();
	if (ret_sync != LFS_ERR_OK)
		return ret_sync;

	nrfx_err_t ret = nrfx_qspi_read(buffer, size, block * m_block_size + off);
	if (ret != NRFX_SUCCESS)
	{
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
int Is25Flash::_prog(lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size)
{
	int ret_sync;

	if (((intptr_t)buffer & 3) || (size & 3))
		return LFS_ERR_INVAL;

	ret_sync = _sync();
	if (ret_sync != LFS_ERR_OK)
		return ret_sync;

	nrfx_err_t ret_write = nrfx_qspi_write(buffer, size, block * m_block_size + off);
	if (ret_write != NRFX_SUCCESS)
	{
		DEBUG_ERROR("QSPI IO Error %04x", ret_write);
		return LFS_ERR_IO;
	}

	// Wait for the write to complete before verifying
	ret_sync = _sync();
	if (ret_sync != LFS_ERR_OK)
		return ret_sync;

	// Read-back verification
	uint8_t read_buffer[IS25_PAGE_SIZE];

	if (size > sizeof(read_buffer))
	{
		DEBUG_ERROR("QSPI Flash prog verification buffer too small | need size of %lu", size);
		return LFS_ERR_NOMEM;
	}

	int ret_read = _read(block, off, &read_buffer[0], size);
	if (ret_read != LFS_ERR_OK)
		return ret_read;

	if (memcmp(reinterpret_cast<const uint8_t *>(buffer), &read_buffer[0], size))
	{
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
int Is25Flash::_prog_fast(lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size)
{
	if (((intptr_t)buffer & 3) || (size & 3))
		return LFS_ERR_INVAL;

	nrfx_err_t ret_write = nrfx_qspi_write(buffer, size, block * m_block_size + off);
	if (ret_write != NRFX_SUCCESS)
	{
		DEBUG_ERROR("QSPI IO Error %04x", ret_write);
		return LFS_ERR_IO;
	}

	return LFS_ERR_OK;
}

/** @brief Internal erase — erases one 4 KB sector. */
int Is25Flash::_erase(lfs_block_t block)
{
	int ret_sync = _sync();
	if (ret_sync != LFS_ERR_OK)
		return ret_sync;

	nrfx_err_t ret_erase = nrfx_qspi_erase(NRF_QSPI_ERASE_LEN_4KB, block * m_block_size);
	if (ret_erase != NRFX_SUCCESS)
	{
		DEBUG_ERROR("QSPI IO Error %04x", ret_erase);
		return LFS_ERR_IO;
	}

	return LFS_ERR_OK;
}

/**
 * @brief Wait for any in-progress write/erase to complete.
 * @return LFS_ERR_OK on success, LFS_ERR_IO on timeout (300 ms).
 */
int Is25Flash::_sync()
{
	nrfx_err_t ret;

	// Expected max wait: 300 ms (worst-case 4 KB erase)
	constexpr uint32_t WAIT_TIME_US = 10;
	constexpr uint32_t WAIT_ATTEMPTS = 30000;

	uint32_t remaining_attempts = WAIT_ATTEMPTS;
	do
	{
		ret = nrfx_qspi_mem_busy_check();
		if (ret == NRFX_SUCCESS)
			break;

		nrf_delay_us(WAIT_TIME_US);
	} while (--remaining_attempts);

	if (ret != NRFX_SUCCESS)
	{
		DEBUG_ERROR("QSPI IO Sync %04x", ret);
		return LFS_ERR_IO;
	}

	return LFS_ERR_OK;
}

/// @name Public LFS wrappers — guard with init check and power management
/// @{

int Is25Flash::read(lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size)
{
	if (!m_is_init)
		return LFS_ERR_IO;
	power_up();
	int ret = _read(block, off, buffer, size);
	power_down();
	return ret;
}

int Is25Flash::prog(lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size)
{
	if (!m_is_init)
		return LFS_ERR_IO;
	power_up();
	int ret = _prog(block, off, buffer, size);
	power_down();
	return ret;
}

int Is25Flash::erase(lfs_block_t block)
{
	if (!m_is_init)
		return LFS_ERR_IO;
	power_up();
	int ret = _erase(block);
	power_down();
	return ret;
}

int Is25Flash::sync()
{
	if (!m_is_init)
		return LFS_ERR_IO;
	power_up();
	int ret = _sync();
	power_down();
	return ret;
}

int Is25Flash::prog_fast(lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size)
{
	if (!m_is_init)
		return LFS_ERR_IO;
	// NO power management here — caller must handle it for performance
	return _prog_fast(block, off, buffer, size);
}

/// @}

/** @brief Wake QSPI peripheral and send RDPD (Release from Deep Power-Down). */
void Is25Flash::_power_up_hw()
{
	nrfx_err_t ret = nrfx_qspi_init(&BSP::QSPI_Inits[BSP::QSPI_0].config, nullptr, nullptr);
	if (ret != NRFX_SUCCESS)
	{
		DEBUG_ERROR("QSPI power_up init failure - %d", ret);
		return;
	}

	const nrf_qspi_cinstr_conf_t config =
	{
		.opcode = IS25LP128F::RDPD, // Wake from deep power mode
		.length = NRF_QSPI_CINSTR_LEN_1B,
		.io2_level = false,
		.io3_level = true,
		.wipwait = false,
		.wren = false
	};

	nrfx_qspi_cinstr_xfer(&config, nullptr, nullptr);

	nrf_delay_us(5);
}

/**
 * @brief Sync, send DP (Deep Power-Down), uninit QSPI, and float all QSPI pins.
 *
 * After this call the QSPI peripheral draws zero current.  CS is held high
 * to ensure the flash chip stays deselected.  All other pins are pulled down.
 */
void Is25Flash::_power_down_hw()
{
	// Wait for any writes/erases to complete before sleeping
	_sync();

	const nrf_qspi_cinstr_conf_t config =
	{
		.opcode = IS25LP128F::DP, // Enter deep power mode
		.length = NRF_QSPI_CINSTR_LEN_1B,
		.io2_level = false,
		.io3_level = true,
		.wipwait = false,
		.wren = false
	};

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

void Is25Flash::power_up()
{
	if (m_power_ref_count++ == 0)
		_power_up_hw();
}

void Is25Flash::power_down()
{
	if (m_power_ref_count == 0)
		return;
	if (--m_power_ref_count == 0)
		_power_down_hw();
}
