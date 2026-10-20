#pragma once

/**
 * @file tpl_clock.hpp
 * @brief TPL5111 wake period: what THIS board's resistor really gives, learnt from GNSS fixes.
 *
 * The TPL5111 period is set by the resistor on DELAY/M_DRV, read once when the
 * TPL powers up (TI SNAS659B §7.5.1), and runs from one DRVn rising edge to the
 * next. WAKEUP_PERIOD (PWP04) is the period that resistor was CHOSEN for; the
 * period it actually gives differs, by a large FIXED part and a small varying one
 * (SNAS659B §6.5, 1 h interval):
 *   - resistor tolerance: T follows a quadratic of REXT, so a 1 % resistor moves T
 *     by ~2.3 % at 1 h (the 221 k // 287 k pair of Table 3 included)      FIXED
 *   - setting accuracy, REXT excluded: +/-0.6 %                            FIXED
 *   - oscillator: +/-0.5 % at 25 C, +/-0.24 % over life                    FIXED
 *   - temperature: +/-100 ppm/C typ, +/-400 max                            VARIES
 *   - supply: +/-0.4 %/V (a Li-ion from 4.2 to 3.0 V: ~0.5 %)              VARIES slowly
 * Worst case ~5 % per hop before calibration. Each GNSS fix gives the true time:
 * dividing the chain's error by the hops it spans measures the fixed part, after
 * which only the varying part remains.
 */

#include <cstdint>
#include <ctime>

namespace TplClock {

/// Tolerance per hop with the nominal period: the whole budget above.
static constexpr unsigned int TOLERANCE_PERMILLE = 50;
/// Tolerance per hop once calibrated: temperature (400 ppm/C max over a tag's
/// range) and supply drift, with margin.
static constexpr unsigned int CALIBRATED_TOLERANCE_PERMILLE = 20;
/// A measurement further than this from the nominal is not a resistor tolerance
/// (a WAKEUP_PERIOD that does not match the fitted resistor): never learnt.
static constexpr unsigned int MAX_DEVIATION_PERMILLE = 100;
/// A measurement this far from the current calibration restarts it instead of
/// being averaged in (the resistor or the nominal changed).
static constexpr unsigned int RESET_JUMP_PERMILLE = 30;
/// Longest chain a measurement is taken over.
static constexpr uint64_t MAX_HOPS = 10000;

inline uint64_t deviation_permille(uint64_t a, uint64_t b) {
	const uint64_t d = (a > b) ? (a - b) : (b - a);
	return (b == 0) ? UINT64_MAX : (d * 1000ULL) / b;
}

/// @brief Is `measured_ms` a usable calibration of `nominal_s`?
inline bool is_calibrated(uint32_t nominal_s, uint32_t measured_ms) {
	return nominal_s > 0 && measured_ms > 0
	       && deviation_permille(measured_ms, (uint64_t)nominal_s * 1000ULL) <= MAX_DEVIATION_PERMILLE;
}

/// @brief Period the pseudo-RTC chain adds per hop, in whole seconds: the
/// measured one when there is a usable calibration, the nominal otherwise.
inline uint32_t chain_period_s(uint32_t nominal_s, uint32_t measured_ms) {
	return is_calibrated(nominal_s, measured_ms) ? (measured_ms + 500U) / 1000U : nominal_s;
}

/// @brief Tolerance per hop matching chain_period_s().
inline unsigned int tolerance_permille(uint32_t nominal_s, uint32_t measured_ms) {
	return is_calibrated(nominal_s, measured_ms) ? CALIBRATED_TOLERANCE_PERMILLE : TOLERANCE_PERMILLE;
}

/// @brief Period actually given by the TPL, measured at a GNSS fix.
///
/// @param chain_period_ms period the chain added per hop since the anchor
/// @param pseudo_boot     this run's boot instant as the chain computed it
/// @param true_boot       this run's boot instant according to GNSS
/// @param anchor          LAST_TIME_SYNC: wall time of the previous trusted setting,
///                        taken during a run shorter than half a period
/// @return the measured period in ms, 0 when there is nothing to measure (no
///         anchor, no hop since it, an implausible chain)
inline uint32_t measure_period_ms(uint32_t chain_period_ms, std::time_t pseudo_boot, std::time_t true_boot,
                                  std::time_t anchor) {
	if (chain_period_ms == 0 || anchor <= 0 || pseudo_boot <= anchor) return 0;
	const uint64_t elapsed_ms = (uint64_t)(pseudo_boot - anchor) * 1000ULL;
	// The anchor sits inside a run, the boot instants at its start: rounding to
	// the nearest whole hop absorbs that offset.
	const uint64_t hops = (elapsed_ms + chain_period_ms / 2) / chain_period_ms;
	if (hops == 0 || hops > MAX_HOPS) return 0;
	const int64_t err_ms = (int64_t)(pseudo_boot - true_boot) * 1000LL;
	const int64_t measured = (int64_t)chain_period_ms - err_ms / (int64_t)hops;
	return (measured > 0 && measured <= (int64_t)UINT32_MAX) ? (uint32_t)measured : 0;
}

/// @brief Fold a measurement into the calibration.
/// @return the new calibration (ms), unchanged when the measurement is unusable
inline uint32_t update_calibration_ms(uint32_t nominal_s, uint32_t current_ms, uint32_t measured_ms) {
	if (measured_ms == 0 || !is_calibrated(nominal_s, measured_ms)) return current_ms;
	if (!is_calibrated(nominal_s, current_ms) || deviation_permille(measured_ms, current_ms) > RESET_JUMP_PERMILLE)
		return measured_ms;
	// Gentle smoothing: temperature and supply move it, one fix carries the
	// one-second quantum of the RTC.
	return (uint32_t)((3ULL * current_ms + measured_ms) / 4ULL);
}

}  // namespace TplClock
