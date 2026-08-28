#pragma once
#include <cstdint>
// Test stub — no-op for nrf_peripheral_power_reset and base address constants
static inline void nrf_peripheral_power_reset(uint32_t) {}
static constexpr uint32_t NRF_SAADC_BASE_ADDR = 0x40007000;
static constexpr uint32_t NRF_QSPI_BASE_ADDR = 0x40029000;
static constexpr uint32_t NRF_UARTE0_BASE_ADDR = 0x40002000;
static constexpr uint32_t NRF_UARTE1_BASE_ADDR = 0x40028000;
