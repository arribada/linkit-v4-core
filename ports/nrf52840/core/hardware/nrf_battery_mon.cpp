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
#define crc16_compute(x, y, z) 0xFFFF
#endif

// Default voltage divider gain if not defined in BSP
#ifndef V_DIV_GAIN
#define V_DIV_GAIN 1.0f
#endif

/// @name SOC hysteresis thresholds (percentage points)
/// @{
static constexpr uint8_t CRITICAL_SOC_HYSTERESIS = 3;
static constexpr uint8_t LOW_BATT_THRESHOLD = 5;
/// @}

/// @name ADC conversion constants
/// @{
static constexpr int ADC_MAX_VALUE = 16384;   ///< 2^14 (14-bit resolution)
static constexpr float ADC_REFERENCE = 0.6f;  ///< Internal reference voltage (V)
/// @}

/// @name Battery discharge profiles
/// Each chemistry defines its own voltage range and SOC LUT (11 entries,
/// index 0 = max_mv → 100 %, index 10 = min_mv → 0 %).
///
/// The step between LUT entries used to be hardcoded at 100 mV, which forced
/// every chemistry into a 1000 mV span. An alkaline pack covers 2000 mV, so the
/// step is now per-profile and the invariant is checked below rather than
/// stated in a comment.
/// @{
static constexpr unsigned int BATT_LUT_ENTRIES = 11;

struct BatteryProfile {
	uint16_t min_mv;
	uint16_t max_mv;
	uint16_t step_mv;  ///< Voltage between adjacent LUT entries
	uint8_t soc_lut[BATT_LUT_ENTRIES];
};

static constexpr BatteryProfile battery_profiles[] = {
	// S18650_2600              — Li-ion 3.2-4.2 V
	{ 3200, 4200, 100, { 100, 91, 79, 62, 42, 12, 2, 0, 0, 0, 0 } },
	// CGR18650_2250            — Li-ion 3.2-4.2 V
	{ 3200, 4200, 100, { 100, 93, 84, 75, 64, 52, 22, 9, 0, 0, 0 } },
	// NCR18650_3100_3400       — Li-ion 3.2-4.2 V
	{ 3200, 4200, 100, { 100, 94, 83, 59, 50, 33, 15, 6, 0, 0, 0 } },
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
	{ 2700, 3700, 100, { 100, 99, 95, 80, 50, 20, 5, 1, 0, 0, 0 } },
	// ALKALINE_3S2P — 6× Energizer EN95 (LR20 / D cell) wired 3S2P, 2.7-4.7 V.
	// Index:        4700 4500 4300 4100 3900 3700 3500 3300 3100 2900 2700  mV
	// (per cell)    1.567 1.500 1.433 1.367 1.300 1.233 1.167 1.100 1.033 0.967 0.900 V
	//
	// Cell data: Energizer EN95 Industrial product datasheet (ANSI-13A / IEC-LR20),
	// 1.5 V nominal, discharge curves plotted 1.6 V down to 0.8 V, capacity rated
	// on "continuous discharge to 0.8 volts at 21 °C".
	//
	// The pack floor is 2.7 V (0.900 V/cell), NOT the 2.4 V the cell chemistry
	// would allow. Below 2.7 V on Vbatt the nRF's own POFCON comparator raises
	// NRF_EVT_POWER_FAILURE_WARNING (nrf_pmu.cpp arms THRESHOLD_V27), and the
	// IS25 flash is specified down to 2.30 V with nothing left in hand. Reporting
	// a few percent of remaining charge through that window would be a gauge that
	// lies exactly where it matters, so 0 % is pinned to the system floor and the
	// last sliver of cell capacity is deliberately left unused. Same reasoning,
	// same number, as the LS17500 profile above.
	//
	// SOC curve: the datasheet discharge shape rescaled so 100 % is a fresh cell
	// at 1.567 V and 0 % is the 0.900 V/cell floor. Alkaline slopes continuously
	// rather than sitting on a plateau, so voltage-based SOC is more meaningful
	// here than it is for Li-SOCl2 — but the slope steepens under load, so a
	// reading taken during a TX burst will under-report.
	{ 2700, 4700, 200, { 100, 90, 81, 68, 54, 41, 28, 17, 10, 4, 0 } },
};

static_assert(sizeof(battery_profiles) / sizeof(battery_profiles[0])
                  == static_cast<unsigned>(BATT_CHEM_ALKALINE_3S2P) + 1,
              "battery_profiles[] must have one entry per BatteryChemistry enum value");

/// @brief Every profile's span must be exactly the LUT's reach.
///
/// convert_level() indexes the LUT as (max_mv - mv) / step_mv, so a profile
/// whose span is not (BATT_LUT_ENTRIES - 1) * step_mv either walks off the end
/// of the table or never reaches its last entry. This used to be a sentence in
/// a comment ("max_mv - min_mv must equal 1000"); it is cheaper to have the
/// compiler check it, and it is what makes adding a chemistry safe.
static constexpr bool battery_profiles_are_consistent() {
	for (const auto &p : battery_profiles) {
		if (p.step_mv == 0) return false;
		if (p.max_mv <= p.min_mv) return false;
		if (static_cast<unsigned int>(p.max_mv - p.min_mv) != (BATT_LUT_ENTRIES - 1) * static_cast<unsigned int>(p.step_mv))
			return false;
	}
	return true;
}
static_assert(battery_profiles_are_consistent(),
              "every battery profile must satisfy max_mv - min_mv == (BATT_LUT_ENTRIES - 1) * step_mv");
/// @}

static void nrfx_saadc_event_handler(nrfx_saadc_evt_t const *p_event) {
	(void)p_event;
}

/// @brief Consecutive confirmations before the low / critical flags are raised.
///
/// Entering either band used to take a single sample. That was survivable on
/// Li-ion, whose internal resistance is low enough that a TX burst barely moves
/// the terminal voltage. An alkaline D pack sags far harder under the same
/// ~1 A pulse, so one unlucky sample taken mid-burst would declare a healthy
/// pack critical and shut the tag down. Leaving a band is still governed by the
/// SOC hysteresis below — this only debounces the way IN.
static constexpr uint8_t BATT_CONFIRM_SAMPLES = 3;

/// @brief Filtered battery state persisted in .noinit RAM to survive soft resets.
///
/// Field order is chosen so the struct has NO padding: the CRC is computed over
/// its raw bytes, and an uninitialised padding byte would make the checksum
/// differ from one boot to the next, silently defeating the persistence it is
/// there to protect. The static_assert below is what keeps that true.
struct BatteryNoinit {
	uint16_t voltage_mv;    ///< Last accepted reading
	uint16_t filtered_soc;  ///< SOC after recovery hysteresis — what the flags are derived from
	uint8_t low_confirm;    ///< Consecutive samples below m_low_level, saturating
	uint8_t crit_confirm;   ///< Consecutive samples below m_critical_level, saturating
};

static_assert(sizeof(BatteryNoinit) == 6, "BatteryNoinit must be padding-free — the CRC covers raw bytes");

static __attribute__((section(".noinit"))) volatile BatteryNoinit m_batt_noinit;

/// @brief CRC16 of m_batt_noinit — detects uninitialised .noinit after power-on.
static __attribute__((section(".noinit"))) volatile uint16_t m_crc;

/// @brief CRC over the .noinit block. Wrapped because the cast is unpleasant and
/// the two call sites must agree byte for byte.
static uint16_t batt_noinit_crc() {
	return crc16_compute(reinterpret_cast<const uint8_t *>(const_cast<const BatteryNoinit *>(&m_batt_noinit)),
	                     sizeof(m_batt_noinit), nullptr);
}

/// @brief Maximum attempts for SAADC calibration busy-wait (each ~10 us).
static constexpr uint32_t ADC_CAL_TIMEOUT = 10000;


NrfBatteryMonitor::NrfBatteryMonitor(uint8_t adc_channel, BatteryChemistry chem, uint8_t critical_level,
                                     uint8_t low_level)
    : BatteryMonitor(low_level, critical_level) {
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
float NrfBatteryMonitor::sample_adc() {
	nrf_saadc_value_t raw = 0;

#ifdef BAT_READ_ENABLE
	GPIOPins::set(BAT_READ_ENABLE);
	PMU::delay_ms(BAT_ADC_SETTLE_MS);
#endif

	// Init/uninit SAADC per sample to reduce sleep current
	nrfx_saadc_init(&BSP::ADC_Inits.config, nrfx_saadc_event_handler);
	nrfx_saadc_channel_init(m_adc_channel, &BSP::ADC_Inits.channel_config[m_adc_channel]);

	// Median of several conversions, not one.
	//
	// This is a high-impedance divider -- the BSP calls it that -- and a single
	// conversion is enough for a noise spike or a transient sag (a TX current
	// pulse, a rail switching) to read tens of millivolts low. That matters here
	// more than anywhere else in the firmware: the critical-battery path latches
	// on one sample and ends in OffState, so a single bad conversion could take
	// a healthy beacon out of service for good. The median rejects an outlier
	// outright, where a mean would still be dragged by it.
	//
	// Sampling inside the same init/uninit window is nearly free: the costs here
	// are the peripheral init and the BAT_ADC_SETTLE_MS delay, both already
	// paid above. Filtering the conversion rather than requiring several
	// consecutive updates also keeps the response immediate -- update() is
	// driven by the services, so "wait for three more" could mean hours on a
	// battery that is genuinely collapsing.
	constexpr unsigned int ADC_MEDIAN_SAMPLES = 5;
	nrf_saadc_value_t samples[ADC_MEDIAN_SAMPLES] = {};
	for (unsigned int i = 0; i < ADC_MEDIAN_SAMPLES; i++) {
		nrfx_saadc_sample_convert(m_adc_channel, &samples[i]);
	}
	// Insertion sort: five elements, no <algorithm> pulled into a driver.
	for (unsigned int i = 1; i < ADC_MEDIAN_SAMPLES; i++) {
		nrf_saadc_value_t key = samples[i];
		unsigned int j = i;
		while (j > 0 && samples[j - 1] > key) {
			samples[j] = samples[j - 1];
			j--;
		}
		samples[j] = key;
	}
	raw = samples[ADC_MEDIAN_SAMPLES / 2];

	nrfx_saadc_uninit();
	nrf_peripheral_power_reset(NRF_SAADC_BASE_ADDR);  // Errata 241: prevent 400 µA idle leak
#ifdef BAT_READ_ENABLE
	GPIOPins::clear(BAT_READ_ENABLE);
#endif

	return (static_cast<float>(raw)) / ((ADC_GAIN / ADC_REFERENCE) * ADC_MAX_VALUE) * 1000.0f;
}

/**
 * @brief Periodic update — sample ADC, apply SOC hysteresis, persist to .noinit RAM.
 *
 * SOC hysteresis prevents rapid toggling of critical/low flags near the threshold:
 *  - Once SOC drops below critical_level, it must recover to (critical_level + 3%)
 *  - Once SOC drops below low_level, it must recover to (low_level + 5%)
 */
void NrfBatteryMonitor::internal_update() {
	uint16_t mv = convert_voltage(sample_adc());

	// A failed conversion reads 0 mV, which convert_level() would faithfully
	// report as a flat battery and hand straight to the critical shutdown. Drop
	// the sample instead and keep the last good one: everything downstream reads
	// m_last_voltage_mv / m_last_level, so a rejected sample is simply a cycle
	// where the gauge did not move.
	if (!is_plausible(mv)) {
		m_consecutive_rejects++;
		if (m_consecutive_rejects >= MAX_CONSECUTIVE_REJECTS) {
			// Past this point the reported voltage is stale, not merely
			// unchanged, and nothing else in the system can tell the difference.
			DEBUG_ERROR("NrfBatteryMonitor: %u consecutive implausible samples (last %u mV) — "
			            "reported battery state is STALE",
			            m_consecutive_rejects, mv);
		} else {
			DEBUG_WARN("NrfBatteryMonitor: implausible sample rejected (%u mV)", mv);
		}
		return;
	}
	m_consecutive_rejects = 0;

	uint8_t level = convert_level(mv);

	// Check CRC of previously stored filtered values (.noinit RAM)
	uint16_t crc = batt_noinit_crc();
	if (crc == m_crc) {
		// Previous values valid — apply hysteresis
		m_batt_noinit.voltage_mv = mv;
		if (m_batt_noinit.filtered_soc < m_critical_level) {
			if (level >= (m_critical_level + CRITICAL_SOC_HYSTERESIS)) m_batt_noinit.filtered_soc = level;
		} else if (m_batt_noinit.filtered_soc < m_low_level) {
			if (level >= (m_low_level + LOW_BATT_THRESHOLD)) m_batt_noinit.filtered_soc = level;
		} else {
			m_batt_noinit.filtered_soc = level;
		}
	} else {
		// CRC mismatch (first boot or power-on) — seed with fresh values
		m_batt_noinit.voltage_mv = mv;
		m_batt_noinit.filtered_soc = level;
		m_batt_noinit.low_confirm = 0;
		m_batt_noinit.crit_confirm = 0;
	}

	// Debounce ENTRY into each band. The counters run on the raw level, not the
	// filtered one: the filtered value is latched low once a band is entered, so
	// counting on it could never fall back and the debounce would be one-way.
	// Read-modify-write through locals: the block is volatile (it lives in
	// .noinit and must survive a soft reset untouched by the optimiser), and a
	// compound ++ on a volatile lvalue is deprecated in C++20 precisely because
	// the read and the write are two separate accesses.
	uint8_t crit_confirm = m_batt_noinit.crit_confirm;
	uint8_t low_confirm = m_batt_noinit.low_confirm;

	if (level < m_critical_level) {
		if (crit_confirm < BATT_CONFIRM_SAMPLES) crit_confirm++;
	} else {
		crit_confirm = 0;
	}
	if (level < m_low_level) {
		if (low_confirm < BATT_CONFIRM_SAMPLES) low_confirm++;
	} else {
		low_confirm = 0;
	}

	m_batt_noinit.crit_confirm = crit_confirm;
	m_batt_noinit.low_confirm = low_confirm;

	// Update CRC
	m_crc = batt_noinit_crc();

	// Apply to base class members
	m_last_voltage_mv = mv;
	m_last_level = level;

	// Flags: the filtered SOC says we are in the band, the counter says we have
	// seen it often enough to act on it.
	m_is_critical_voltage = (m_batt_noinit.filtered_soc < m_critical_level) && (crit_confirm >= BATT_CONFIRM_SAMPLES);
	m_is_low_level = (m_batt_noinit.filtered_soc < m_low_level) && (low_confirm >= BATT_CONFIRM_SAMPLES);

	DEBUG_TRACE("NrfBatteryMonitor: %u mV, %u%% (filtered %u%%), low=%u critical=%u", m_last_voltage_mv, m_last_level,
	            static_cast<unsigned int>(m_batt_noinit.filtered_soc), static_cast<unsigned int>(m_is_low_level),
	            static_cast<unsigned int>(m_is_critical_voltage));
}

/**
 * @brief Convert millivolts to SOC percentage via LUT with linear interpolation.
 * @param mv Battery voltage in millivolts.
 * @return SOC percentage (0-100).
 */
uint8_t NrfBatteryMonitor::convert_level(uint16_t mv) {
	unsigned int chem = static_cast<unsigned int>(m_chem);
	if (chem >= sizeof(battery_profiles) / sizeof(battery_profiles[0])) chem = 0;

	const BatteryProfile &profile = battery_profiles[chem];

	// Clamp first, so the index arithmetic below only ever runs on a voltage
	// that is inside the profile's span. The old form derived the index with a
	// hardcoded /100 and then repaired out-of-range results afterwards, which
	// tied every chemistry to a 100 mV step.
	if (mv >= profile.max_mv) return profile.soc_lut[0];
	if (mv <= profile.min_mv) return profile.soc_lut[BATT_LUT_ENTRIES - 1];

	// (max_mv - mv) is now in (0, span), so idx is in [0, BATT_LUT_ENTRIES - 2]
	// and idx + 1 stays in range — guaranteed by the span static_assert above.
	const unsigned int idx = (profile.max_mv - mv) / profile.step_mv;
	const uint8_t upper = profile.soc_lut[idx];
	const uint8_t lower = profile.soc_lut[idx + 1];
	const uint16_t upper_mv = profile.max_mv - (idx * profile.step_mv);
	const float t = static_cast<float>(upper_mv - mv) / static_cast<float>(profile.step_mv);

	// +0.5 rounds instead of truncating: the old cast lost up to a full percent
	// on every reading, always in the pessimistic direction.
	return static_cast<uint8_t>(static_cast<float>(upper) + t * (static_cast<float>(lower) - static_cast<float>(upper))
	                            + 0.5f);
}

bool NrfBatteryMonitor::is_plausible(uint16_t mv) const {
	unsigned int chem = static_cast<unsigned int>(m_chem);
	if (chem >= sizeof(battery_profiles) / sizeof(battery_profiles[0])) chem = 0;

	const BatteryProfile &profile = battery_profiles[chem];

	// Half the floor below, 300 mV above. A failed conversion reads 0 mV, which
	// convert_level would faithfully turn into 0 % and hand to the critical
	// shutdown; that is the case worth catching. The window is otherwise left
	// wide on purpose — a battery below its profile floor is a real state we
	// must still be able to report, and a fresh pack can sit slightly above its
	// nominal maximum.
	return (mv >= (profile.min_mv / 2)) && (mv <= (profile.max_mv + 300));
}

/**
 * @brief Apply voltage divider gain to convert raw ADC millivolts to actual battery voltage.
 * @param adc_mv Raw ADC reading in millivolts.
 * @return Battery voltage in millivolts.
 */
uint16_t NrfBatteryMonitor::convert_voltage(float adc_mv) {
#ifdef BATTERY_NOT_FITTED
	unsigned int chem = static_cast<unsigned int>(m_chem);
	if (chem >= sizeof(battery_profiles) / sizeof(battery_profiles[0])) chem = 0;
	return battery_profiles[chem].max_mv;
#else
	return static_cast<uint16_t>(adc_mv * V_DIV_GAIN);
#endif
}
