/**
 * @file sws_analog_constants.hpp
 * @brief SWS analog internal constants — shared between service, calibration, and detection .cpp files.
 * @note This is an internal header — do not include from outside core/services/.
 */

#pragma once

// ADC constants
#define ADC_REFERENCE_V                 0.6f
#define ADC_GAIN_1_6                    (1.0f / 6.0f)
// Valid ADC range: 0..16383 (14-bit SAADC).
// ADC value 0 is a legitimate reading (e.g. dry air, open pin — no current through water).
// Errors (SAADC init failure, conversion failure) return ADC_READ_ERROR (UINT16_MAX),
// which is outside the 14-bit range and rejected by is_value_valid().
#define ADC_INVALID_MAX                 16383
#define ADC_READ_ERROR                  UINT16_MAX

// Default configuration values
#define DEFAULT_HYSTERESIS_PERCENT      4
#define DEFAULT_CALIB_INTERVAL_SEC      3600
#define DEFAULT_MAX_DIVE_TIME_SEC       7200

// Detection tuning
#define DEFAULT_THRESHOLD_RATIO_PERCENT 35
#define DEFAULT_ALPHA_PERCENT           19
#define DEFAULT_MIN_DRY_SAMPLES         1  // Immediate surface on threshold crossing

// Water baseline protection
#define MIN_WATER_AIR_RATIO             3

// Minimum air baseline floor: prevents adaptive DOWN from collapsing to near-zero.
// With 14-bit ADC, a clean dry electrode reads ~50-200 ADC with RC circuit.
#define AIR_BASELINE_FLOOR              50

// Minimum gap between threshold_current and threshold_air (ADC counts).
// Prevents false UW triggers from noise when air/water baselines are close
// (e.g. stale calibration with water=108 and air=50).
#define THRESHOLD_MIN_ABOVE_AIR         20

// SWS.CAL offsets for Calibration class persistence
// 0 = manual water hint (SCALW/SWSCAL)
// 1 = manual air hint (SCALW/SWSCAL)
// 2 = running water baseline (auto-updated on state transitions)
// 3 = running air baseline (auto-updated on state transitions)
// 4 = observed peak ADC
#define CAL_OFFSET_HINT_WATER           0
#define CAL_OFFSET_HINT_AIR             1
#define CAL_OFFSET_RUN_WATER            2
#define CAL_OFFSET_RUN_AIR              3
#define CAL_OFFSET_PEAK                 4

// Surface baseline adaptation
#define SURFACE_ADAPT_THRESHOLD         1.3f
#define MIN_SURFACE_TIME_FOR_ADAPT      10

// ═══════════════════════════════════════════════════════
//  MULTI-LEVEL SURFACE DETECTION
//
//  All levels require: underwater ≥ 1s AND proximity guard OK
//
//  Level 1 - INSTANT (1 sample)
//    Drop > 4% from previous raw → immediate surface
//    Use case: sharp water exit, any electrode condition
//
//  Level 2 - FAST (2 samples)
//    2 consecutive raw drops, cumulative > 3% → surface
//    Use case: gradual exit where individual drops < 3% each
//
//  Level 3 - TREND (3+ MA3 decreases)
//    MA3 decreasing 3+ times, total MA3 drop > 4% → surface
//    Use case: heavy biofouling, very slow drying, noisy signal
//
//  Level 4 - ABSOLUTE (variable)
//    filtered < water_baseline × 85% → surface   (L4_DROP_PERCENT = 15)
//    Use case: moderate biofouling, well-calibrated baselines
//
//  Level 5 - SAFETY NET (>10s underwater)
//    Drop from dive peak > 10% → surface
//    Use case: fallback when baselines have drifted
// ═══════════════════════════════════════════════════════

// Level 1
#define L1_DROP_PERCENT                 4  // Drop from recent peak threshold (%)

// Level 2
#define L2_DROP_PERCENT                 3  // Cumulative 2-sample raw drop (%)
#define L2_MIN_CONSECUTIVE              2  // Minimum consecutive raw drops
#define L2_MIN_STEP_PERCENT             2  // Each individual step must be ≥ 2% (filters drift)

// Level 3
#define L3_MIN_CONSECUTIVE              3  // Consecutive MA3 decreases
#define L3_DROP_PERCENT                 4  // Total MA3 drop from trend start (%)

// Level 4
// 15% threshold (was 8%) — gives biofouling stage transitions a wider margin
// where the water baseline lags the actual water level. Combined with the
// 2-consecutive-sample requirement, this avoids spurious surfacing during
// the EMA convergence period after a salinity / biofouling change.
#define L4_DROP_PERCENT                 15  // Drop from water baseline (%)

// Level 5
#define L5_DROP_PERCENT                 10  // Cumulative drop from peak (%)
#define L5_MIN_TIME_SEC                 10  // Minimum time underwater before L5

// Safety
// 0 = L-overrides can fire on the very first post-dive sample (instant surface).
// Protection against rapid state flapping is provided downstream by
// m_surface_lockout_remaining (UNP25 UW_MIN_SURFACE_TIME, default 5s) which
// blocks any return to UW for that duration after a L-override fires.
// The air baseline EMA pull on L-override is bounded by AIR_RECALIB_MAX_RATIO
// (0.70 × water) so an isolated false trigger cannot collapse the threshold.
#define OVERRIDE_MIN_TIME_SEC 1  // Minimum underwater time before any override — 1-sample backstop independent of UNP25
#define SURFACE_LOCKOUT_DURATION_SEC  30
#define MAX_CONSECUTIVE_DIVE_TIMEOUTS 3    // Force surface after N timeouts without any surface detection
#define GUIDED_CALIB_TIMEOUT_TICKS    300  // 300 ticks × 1s = 5 minutes max for guided calibration

// Test mode auto-stop timeout default lives in sws_analog_service.hpp
// (SWSAnalogService::TEST_TIMEOUT_DEFAULT_MS) so that reset_noinit_data()
// can use it without including this internal-only constants header.

// Stuck-state recovery: when air baseline collapses below floor for N consecutive
// samples while at surface, force a fresh calibration from current ADC readings.
// Catches the dry-electrode death spiral where periodic Air recalib pulls air → 0
// before the dive-timeout escalation (≥6h) can kick in.
#define AIR_COLLAPSE_RECOVERY_SAMPLES 5

// Adaptive sample delay (µs) — defaults, overridden by UNP09/UNP10 params at init
// No series R on SWS line: cap charges through GPIO impedance (~100Ω) + water R.
// Salt water (1K): τ = 1K × 100nF = 100µs → 500µs = 5τ → 99% charge
// Tap water (50K): τ = 50K × 100nF = 5ms → 1ms = 0.2τ → 18% charge (enough for detection)
// Biofouling (>100K): τ > 10ms → need longer delay → adaptive increases up to max
// Air (>1M): τ > 100ms → stays near 0 at any delay
#define SAMPLE_DELAY_MIN_US_DEFAULT   200    // Floor: salt water fully charges in ~500µs
#define SAMPLE_DELAY_MAX_US_DEFAULT   10000  // Ceiling: biofouled electrodes need longer charge
#define SAMPLE_DELAY_DEFAULT_US       1000   // Default: 1ms (good balance clean electrode)

// Air baseline recovery: when air drops below this, readings are likely invalid
// (RC circuit not charging enough at current delay). Force delay UP to recover.
#define AIR_BASELINE_RECOVER          150
#define CONTRAST_LOW_THRESHOLD        50   // contrast_x10 < 5.0x → reduce delay
#define CONTRAST_HIGH_THRESHOLD       100  // contrast_x10 > 10.0x → increase delay

// Proximity guard: L-overrides blocked if value > water_baseline * guard%
// Adaptive: relaxes when contrast is low (biofouling) to allow small-gap detection
#define PROXIMITY_GUARD_PERCENT       95  // Default: must drop below 95% of peak
#define PROXIMITY_GUARD_BIOFOULING    99  // Relaxed: when contrast < 5x, allow 1% gap

// Air recalibration on L-override: conservative EMA for gradual adaptation.
// At water exit, filtered value lags (MA2) and is near water baseline → small weight
// prevents ratcheting air upward on repeated rapid transitions.
#define AIR_RECALIB_EMA_WEIGHT        0.15f
// Hard safety cap: air must never exceed 70% of water.
// Guarantees threshold_high stays below ~86% of water baseline,
// so actual water readings (even 10-15% below baseline) still cross the threshold.
#define AIR_RECALIB_MAX_RATIO         0.70f

// Test mode (DTE SWSTST,1) fast sampling — bench/cable testing.
// Overrides UNP03/UNP04 while m_test_mode is true; reverted at SWSTST,0 or
// after m_test_timeout_ms (default 1h) auto-stop, so a deployed unit can
// never drain battery at this rate.
#define SWS_TEST_MODE_SAMPLE_MS       100

// Guided calibration parameters
#define CALIB_NUM_SAMPLES             5     // samples per phase (air/water)
#define CALIB_STABILITY_THRESHOLD     3     // consecutive stable readings to start sampling
#define CALIB_STABILITY_TOLERANCE     500   // ADC counts variation allowed for "stable"
#define CALIB_SAMPLE_INTERVAL_MS      1000  // 1s sampling during guided calibration

// ═══════════════════════════════════════════════════════
//  CRC16 — usage pattern (audit 2026-05 R-DOC-02)
//
//  When computing a CRC over a struct that contains its own CRC field,
//  ALWAYS use offsetof(StructType, crc_field_name) for the `length`
//  parameter to EXCLUDE the CRC field from the input data. Otherwise the
//  CRC depends on its own value, which makes validation impossible.
//
//  Example (correct):
//      m_calib.crc = crc16_compute((const uint8_t *)&m_calib,
//                                   offsetof(CalibrationData, crc), nullptr);
//
//  Anti-example (BUG — historically present, fixed in commit f1ea2ed4):
//      m_calib.crc = crc16_compute((const uint8_t *)&m_calib,
//                                   sizeof(m_calib), nullptr);  // includes 'crc' itself
//
//  When the CRC is stored OUTSIDE the data (e.g. m_observed_peak_adc and
//  m_observed_peak_crc as separate variables), `sizeof(data)` is correct
//  because the CRC field is not part of `data`. Same for buffers like
//  m_filtered_values where the CRC lives in a separate member.
// ═══════════════════════════════════════════════════════

// CRC16 stub for test builds
#ifndef CPPUTEST
#include "crc16.h"
#else
#include <cstdint>
static inline uint16_t crc16_compute(const uint8_t *data, uint16_t length, const uint16_t *) {
	uint16_t crc = 0xFFFF;
	for (uint16_t i = 0; i < length; i++) {
		crc ^= static_cast<uint16_t>(data[i]) << 8;
		for (uint8_t j = 0; j < 8; j++)
			crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
	}
	return crc;
}
#endif
