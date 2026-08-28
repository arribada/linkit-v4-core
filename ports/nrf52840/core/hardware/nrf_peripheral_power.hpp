#pragma once

/**
 * @file nrf_peripheral_power.hpp
 * @brief nRF52840 peripheral POWER register workarounds for current leak errata.
 *
 * Multiple nRF52840 errata cause excess idle current (400-900 µA) when
 * peripherals are disabled without toggling their POWER register:
 *
 *  - Errata 89:  TWIM/SPIM + GPIOTE → 400 µA
 *  - Errata 122: QSPI → high current after TASKS_ACTIVATE
 *  - Errata 195: SPIM3 → 900 µA after disable
 *  - Errata 241: SAADC → 400 µA after disable
 *  - UARTE1 Nordic SDK bug: HF clk and DMA bus not released
 *
 * Call nrf_peripheral_power_reset() AFTER nrfx_xxx_uninit() for any
 * TWIM, SPIM, SAADC, QSPI, or UARTE peripheral.
 *
 * @see https://infocenter.nordicsemi.com/topic/errata_nRF52840_EngA/ERR/nRF52840/EngineeringA/latest/anomaly_840_89.html
 * @see https://docs.nordicsemi.com/bundle/errata_nRF52840_Rev3/page/ERR/nRF52840/Rev3/latest/anomaly_840_241.html
 */

#include <cstdint>

/**
 * @brief Toggle peripheral POWER register to force full hardware reset.
 *
 * Sequence: POWER=0 (off) → dummy read (bus sync) → POWER=1 (on).
 * This resets all internal state including HF clock and DMA bus references.
 *
 * @param base_address  Peripheral base address (e.g. 0x40003000 for TWIM0).
 */
static inline void nrf_peripheral_power_reset(uint32_t base_address) {
	volatile auto *power = reinterpret_cast<volatile uint32_t *>(base_address + 0xFFC);
	*power = 0;    // Power OFF — resets all peripheral state
	(void)*power;  // Dummy read — bus synchronization barrier
	*power = 1;    // Power ON — peripheral back to initial state
}

/// @name Common peripheral base addresses for power reset
/// @{
static constexpr uint32_t NRF_SAADC_BASE_ADDR = 0x40007000;
static constexpr uint32_t NRF_QSPI_BASE_ADDR = 0x40029000;
static constexpr uint32_t NRF_UARTE0_BASE_ADDR = 0x40002000;
static constexpr uint32_t NRF_UARTE1_BASE_ADDR = 0x40028000;
/// @}
