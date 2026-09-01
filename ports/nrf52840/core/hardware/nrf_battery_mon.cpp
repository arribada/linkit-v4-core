/**
 * @file nrf_battery_mon.cpp
 * @brief nRF52840 SAADC-based battery voltage monitor implementation.
 */

#include <cstdint>

#include "nrf_battery_mon.hpp"
#include "nrfx_saadc.h"
#include "nrf_peripheral_power.hpp"
#include "nrf_delay.h"
#include "bsp.hpp"
#include "error.hpp"
#include "debug.hpp"
#include "gpio.hpp"
#include "pmu.hpp"

#ifndef CPPUTEST
#include "crc16.h"
#else
#define crc16_compute(x, y, z)  0xFFFF
#endif

// Default voltage divider gain if not defined in BSP
#ifndef V_DIV_GAIN
#define V_DIV_GAIN 1.0f
#endif

/// @name SOC hysteresis thresholds (percentage points)
/// @{
static constexpr uint8_t CRITICAL_SOC_HYSTERESIS = 3;
static constexpr uint8_t LOW_BATT_SOC_THRESHOLD  = 5;
/// @}

/// @name ADC conversion constants
/// @{
static constexpr int     ADC_MAX_VALUE = 16384;    ///< 2^14 (14-bit resolution)
static constexpr float   ADC_REFERENCE = 0.6f;     ///< Internal reference voltage (V)
/// @}

/// @name Battery discharge profiles
/// Each chemistry defines its own voltage range and SOC LUT (11 entries, 100 mV steps,
/// index 0 = max_mv → 100 %, index 10 = min_mv → 0 %). max_mv − min_mv must equal 1000.
/// @{
static constexpr unsigned int BATT_LUT_ENTRIES = 11;

struct BatteryProfile {
	uint16_t min_mv;
	uint16_t max_mv;
	uint16_t step_mv;
	uint8_t  soc_lut[BATT_LUT_ENTRIES];
};

static constexpr BatteryProfile battery_profiles[] = {
	// S18650_2600              — Li-ion 3.2-4.2 V
	{ 3200, 4200, 100, { 100, 91, 79, 62, 42, 12,  2,  0, 0, 0, 0 } },
	// CGR18650_2250            — Li-ion 3.2-4.2 V
	{ 3200, 4200, 100, { 100, 93, 84, 75, 64, 52, 22,  9, 0, 0, 0 } },
	// NCR18650_3100_3400       — Li-ion 3.2-4.2 V
	{ 3200, 4200, 100, { 100, 94, 83, 59, 50, 33, 15,  6, 0, 0, 0 } },
	// ALKALINE_3S2P — 6x LR20 3S2P, 2.4-4.7 V, 230 mV step
	// Index:        4700  4470  4240  4010  3780  3550  3320  3090  2860  2630  2400  mV
	// (per cell)    1.567 1.490 1.413 1.337 1.260 1.183 1.107 1.030 0.953 0.877 0.800 V
	{ 2400, 4700, 230, { 100, 89, 79, 63, 48, 34, 21, 13, 7, 3, 0 } },
	// LS17500_2P (Li-SOCl2 plateau profile, 2 cells in parallel, 2.7-3.7 V).
	// Index:        3700 3600 3500 3400 3300 3200 3100 3000 2900 2800 2700  mV
	// Plateau 100 % until 3.55 V then progressive drop to the cliff at 3.0 V.
	// Voltage-based SOC for Li-SOCl2 is intrinsically coarse (flat discharge);
	// trust BATT_VOLTAGE more than BATT_SOC for fine-grained tracking.
	//
	// Topology in use on this board: 2x LS17500 (parallel) // 70F supercap on Vbatt rail.
	// - Total cell capacity ~7.2 Ah at 3.6 V; supercap absorbs Argos/LoRa TX bursts so a
	//   ~1 A pulse for 1 s only drops Vbatt by ~14 mV → reads here reflect cell SOC, not sag.
	// - End-of-life reserve: ≈ ½·70·(3.5²−2.5²) = 210 J ≈ 0.058 Wh on the supercap, i.e. ~23 min
	//   of graceful-shutdown time at 50 µA sleep after the cells truly die.
	// - Supercap recharge from cells takes ~6 min for a full 1 V refill (200 mA continuous,
	//   2 cells) — don't burst TX too fast when SOC is low or the cap won't recover.
	{ 2700, 3700, 100, { 100, 99, 95, 80, 50, 20,  5,  1, 0, 0, 0 } },
};

static_assert(sizeof(battery_profiles) / sizeof(battery_profiles[0]) ==
              static_cast<unsigned>(BATT_CHEM_LS17500_2P) + 1,
              "battery_profiles[] must have one entry per BatteryChemistry enum value");
/// @}

static void nrfx_saadc_event_handler(nrfx_saadc_evt_t const *p_event)
{
	(void)p_event;
}

/// @name  Filtered SOC values persisted in .noinit RAM to survive soft resets.
/// 	   Flags are set only after 3 consecutive readings below the thresholds.
/// @{
static constexpr unsigned int FILTER_DEPTH = 3;

struct BatteryFilterState {
	uint8_t  soc_history[FILTER_DEPTH];   ///< 3 last SOC values, from oldest to newest
	uint8_t  is_low;                      ///< Memorized low flag
	uint8_t  is_critical;                 ///< Memorized critical flag
};
/// @}

static_assert(sizeof(BatteryFilterState) == FILTER_DEPTH + 2,
              "BatteryFilterState must be tightly packed — CRC covers raw bytes");

static __attribute__((section(".noinit"))) BatteryFilterState m_filter;
static __attribute__((section(".noinit"))) uint16_t m_crc;

/// @brief Maximum attempts for SAADC calibration busy-wait (each ~10 us).
static constexpr uint32_t ADC_CAL_TIMEOUT = 10000;

static bool all_below(const uint8_t *history, uint8_t threshold)
{
	for (unsigned int i = 0; i < FILTER_DEPTH; i++)
		if (history[i] >= threshold)
			return false;
	return true;
}

static bool all_at_or_above(const uint8_t *history, uint8_t threshold)
{
	for (unsigned int i = 0; i < FILTER_DEPTH; i++)
		if (history[i] < threshold)
			return false;
	return true;
}

NrfBatteryMonitor::NrfBatteryMonitor(uint8_t adc_channel,
		BatteryChemistry chem,
		uint8_t critical_level,
		uint8_t low_level)
		: BatteryMonitor(low_level, critical_level)
{
	// One-time SAADC calibration (retained until power reset, survives init/uninit)
	nrfx_saadc_init(&BSP::ADC_Inits.config, nrfx_saadc_event_handler);
	nrfx_saadc_calibrate_offset();

	DEBUG_TRACE("Enter ADC calibration...");
	uint32_t cal_attempts = ADC_CAL_TIMEOUT;
	while (nrfx_saadc_is_busy() && --cal_attempts) {
		nrf_delay_us(10);
	}

	if (cal_attempts == 0) {
		DEBUG_ERROR("ADC calibration timeout");
	} else {
		DEBUG_TRACE("ADC calibration complete");
	}

	nrfx_saadc_uninit();
	nrf_peripheral_power_reset(NRF_SAADC_BASE_ADDR);  // Errata 241: prevent 400 µA idle leak

	m_adc_channel = adc_channel;
	m_is_init = false;
	m_chem = chem;
}

/**
 * @brief Sample battery voltage via SAADC.
 *
 * Enables the battery read circuit (if present), waits for the RC settling
 * time defined by BAT_ADC_SETTLE_MS in the BSP, samples one conversion,
 * then powers down the SAADC to minimise sleep current.
 *
 * @return Raw voltage in millivolts (before divider gain correction).
 */
float NrfBatteryMonitor::sample_adc()
{
	nrf_saadc_value_t raw = 0;

#ifdef BAT_READ_ENABLE
	GPIOPins::set(BAT_READ_ENABLE);
	PMU::delay_ms(BAT_ADC_SETTLE_MS);
#endif

	// Init/uninit SAADC per sample to reduce sleep current
	nrfx_saadc_init(&BSP::ADC_Inits.config, nrfx_saadc_event_handler);
	nrfx_saadc_channel_init(m_adc_channel, &BSP::ADC_Inits.channel_config[m_adc_channel]);

	nrfx_saadc_sample_convert(m_adc_channel, &raw);

	nrfx_saadc_uninit();
	nrf_peripheral_power_reset(NRF_SAADC_BASE_ADDR);  // Errata 241: prevent 400 µA idle leak
#ifdef BAT_READ_ENABLE
	GPIOPins::clear(BAT_READ_ENABLE);
#endif

	return (static_cast<float>(raw)) / ((ADC_GAIN / ADC_REFERENCE) * ADC_MAX_VALUE) * 1000.0f;
}

bool NrfBatteryMonitor::is_plausible(uint16_t mv) const
{
	unsigned int chem = static_cast<unsigned int>(m_chem);
	if (chem >= sizeof(battery_profiles) / sizeof(battery_profiles[0]))
		chem = 0;

	const BatteryProfile& p = battery_profiles[chem];
	// Reject 0 mV (failed conversion) and non consistent values
	return (mv >= (p.min_mv / 2)) && (mv <= (p.max_mv + 300));
}

/**
 * @brief Periodic update — sample ADC, apply SOC hysteresis, persist to .noinit RAM.
 *
 * SOC hysteresis prevents rapid toggling of critical/low flags near the threshold:
 *  - Once SOC drops below critical_level, it must recover to (critical_level + 3%)
 *  - Once SOC drops below low_level, it must recover to (low_level + 5%)
 */
void NrfBatteryMonitor::internal_update()
{
	uint16_t mv = convert_voltage(sample_adc());
	if (!is_plausible(mv)) {
		DEBUG_WARN("Battery sample rejected: %u mV", mv);
		return;
	}

	uint8_t  level = convert_level(mv);

	// Check CRC of previously stored filtered values (.noinit RAM)
	uint16_t crc = crc16_compute(reinterpret_cast<const uint8_t *>(&m_filter), sizeof(m_filter), nullptr);

	if (crc != m_crc) {
		// CRC mismatch (first boot or power-on) — seed with fresh values
		for (unsigned int i = 0; i < FILTER_DEPTH; i++)
			m_filter.soc_history[i] = level;
		m_filter.is_low      = 0;
		m_filter.is_critical = 0;
	} else {
		// Previous values valid — shift history and append new reading
		for (unsigned int i = 0; i < FILTER_DEPTH - 1; i++)
			m_filter.soc_history[i] = m_filter.soc_history[i + 1];
		m_filter.soc_history[FILTER_DEPTH - 1] = level;
	}

	// --- Critical ---
	if (!m_filter.is_critical) {
		if (all_below(m_filter.soc_history, m_critical_level))
			m_filter.is_critical = 1;
	} else {
		if (all_at_or_above(m_filter.soc_history,
		                    m_critical_level + CRITICAL_SOC_HYSTERESIS))
			m_filter.is_critical = 0;
	}

	// --- Low ---
	if (!m_filter.is_low) {
		if (all_below(m_filter.soc_history, m_low_level))
			m_filter.is_low = 1;
	} else {
		if (all_at_or_above(m_filter.soc_history,
		                    m_low_level + LOW_BATT_SOC_THRESHOLD))
			m_filter.is_low = 0;
	}

	// Update CRC
	m_crc = crc16_compute(reinterpret_cast<const uint8_t *>(&m_filter), sizeof(m_filter), nullptr);

	// Apply to base class members
	m_last_voltage_mv = mv;
	m_last_level = level;

	// Set flags (both based on filtered SOC, not raw)
	m_is_critical_voltage = (m_filter.is_critical != 0);
	m_is_low_level = (m_filter.is_low != 0);

	DEBUG_TRACE("Battery update: %u mV, %u%% SOC, low=%d, critical=%d",
	           m_last_voltage_mv, m_last_level, m_filter.is_low, m_filter.is_critical);
}

/**
 * @brief Convert millivolts to SOC percentage via LUT with linear interpolation.
 * @param mv Battery voltage in millivolts.
 * @return SOC percentage (0-100).
 */
uint8_t NrfBatteryMonitor::convert_level(uint16_t mv)
{
	unsigned int chem = static_cast<unsigned int>(m_chem);
	if (chem >= sizeof(battery_profiles) / sizeof(battery_profiles[0]))
		chem = 0;

	const BatteryProfile& profile = battery_profiles[chem];

	if (mv >= profile.max_mv) return profile.soc_lut[0];
	if (mv <= profile.min_mv) return profile.soc_lut[BATT_LUT_ENTRIES - 1];

	const unsigned idx 		= (profile.max_mv - mv) / profile.step_mv;
	const uint8_t  upper    = profile.soc_lut[idx];
	const uint8_t  lower    = profile.soc_lut[idx + 1];
	const uint16_t upper_mv = profile.max_mv - (idx * profile.step_mv);
	const float    t        = static_cast<float>(upper_mv - mv) / static_cast<float>(profile.step_mv);

	return static_cast<uint8_t>(upper + t * (static_cast<float>(lower) - upper) + 0.5f);
}

/**
 * @brief Apply voltage divider gain to convert raw ADC millivolts to actual battery voltage.
 * @param adc_mv Raw ADC reading in millivolts.
 * @return Battery voltage in millivolts.
 */
uint16_t NrfBatteryMonitor::convert_voltage(float adc_mv)
{
#ifdef BATTERY_NOT_FITTED
	unsigned int chem = static_cast<unsigned int>(m_chem);
	if (chem >= sizeof(battery_profiles) / sizeof(battery_profiles[0]))
		chem = 0;
	return battery_profiles[chem].max_mv;
#else
	return static_cast<uint16_t>(adc_mv * V_DIV_GAIN);
#endif
}
