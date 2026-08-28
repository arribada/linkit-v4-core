/**
 * @file sws_analog_calibration.cpp
 * @brief SWS analog — calibration data, flash persistence, air/water baselines, threshold.
 */

#include <cstdint>
#include "sws_analog_service.hpp"
#include "sws_analog_constants.hpp"
#include "debug.hpp"
#include "pmu.hpp"
#include "gpio.hpp"
#include "rgb_led.hpp"

// LED feedback for the guided calibration walk-through (GREEN=air, BLUE=water).
extern RGBLed *status_led;

// CRC16 provided by sws_analog_constants.hpp

// ═══════════════════════════════════════════════════════
//  MANUAL CALIBRATION + FLASH PERSISTENCE
// ═══════════════════════════════════════════════════════

/// @brief DTE SCALW: write manual calibration value (water=offset 0, air=offset 1).
/// @param value   ADC value to store.
/// @param offset  Calibration offset (0=water, 1=air).
void SWSAnalogService::calibration_write(const double value, const unsigned int offset) {
	m_manual_calib.write(offset, value);
	m_manual_calib.save(true);
	DEBUG_INFO("SWSAnalog: Manual calib write offset=%u value=%.0f (saved)", offset, value);
}

/// @brief DTE SCALR: read manual calibration value.
/// @param[out] value  ADC value at offset.
/// @param offset      Calibration offset (0=water, 1=air).
void SWSAnalogService::calibration_read(double &value, const unsigned int offset) {
	try {
		value = m_manual_calib.read(offset);
	} catch (...) {
		value = 0.0;
	}
}

/// @brief Persist manual calibration data to SWS.CAL file.
/// @param force  true to save even if not changed.
void SWSAnalogService::calibration_save(bool force) {
	m_manual_calib.save(force);
}

// ═══════════════════════════════════════════════════════
//  FLASH PERSISTENCE — via SWS.CAL (Calibration class)
//  Running calibration stored at offsets 2-4, survives hard resets.
// ═══════════════════════════════════════════════════════

/// @brief Save running calibration (air/water/peak) to flash for hard reset persistence.
void SWSAnalogService::save_calibration_to_flash() {
	if (!s_instance) return;

	s_instance->m_manual_calib.write(CAL_OFFSET_RUN_WATER, (double)m_calib.threshold_water);
	s_instance->m_manual_calib.write(CAL_OFFSET_RUN_AIR, (double)m_calib.threshold_air);
	s_instance->m_manual_calib.write(CAL_OFFSET_PEAK, (double)m_observed_peak_adc);
	s_instance->m_manual_calib.save(true);
	m_last_flash_save_time = PMU::get_timestamp_ms() / 1000;
	DEBUG_TRACE("SWSAnalog: Calibration saved to SWS.CAL (air=%u water=%u peak=%u)", m_calib.threshold_air,
	            m_calib.threshold_water, m_observed_peak_adc);
}

/// @brief Save calibration with 60s debounce to avoid flash wear on rapid transitions.
void SWSAnalogService::save_calibration_to_flash_debounced() {
	uint64_t now_sec = PMU::get_timestamp_ms() / 1000;
	if ((now_sec - m_last_flash_save_time) >= FLASH_SAVE_MIN_INTERVAL_SEC) {
		save_calibration_to_flash();
	}
}

/// @brief Load running calibration from flash (SWS.CAL offsets 2-4).
/// @return true if valid calibration was restored from flash.
bool SWSAnalogService::load_calibration_from_flash() {
	if (!s_instance) return false;

	double water = 0, air = 0, peak = 0;
	try {
		water = s_instance->m_manual_calib.read(CAL_OFFSET_RUN_WATER);
		air = s_instance->m_manual_calib.read(CAL_OFFSET_RUN_AIR);
		peak = s_instance->m_manual_calib.read(CAL_OFFSET_PEAK);
	} catch (...) {
		return false;
	}

	// Sanity check
	if (air <= 0 || water <= 0 || water <= air || water > ADC_INVALID_MAX) {
		return false;
	}

	// Baseline floor: a properly connected, dry electrode reads at least
	// AIR_BASELINE_FLOOR (~50 ADC on 14-bit SAADC). Flash data below this is
	// either a leftover from a factory/open-circuit state or a disconnected
	// electrode during a past calibration run — trust neither. Fall back to
	// a fresh in-air calibration instead of propagating bad baselines.
	if ((uint16_t)air < AIR_BASELINE_FLOOR) {
		DEBUG_WARN("SWSAnalog: stored air=%u < floor %u — discarding flash calibration", (unsigned)(uint16_t)air,
		           (unsigned)AIR_BASELINE_FLOOR);
		return false;
	}
	// Contrast floor: water-air gap must leave enough room above
	// THRESHOLD_MIN_ABOVE_AIR (20) plus the 10 ADC minimum hysteresis (30
	// total) so that threshold_high can live between the two baselines and
	// actual underwater readings can cross it.
	static constexpr uint16_t MIN_WATER_AIR_GAP = 30;
	if ((uint16_t)(water - air) < MIN_WATER_AIR_GAP) {
		DEBUG_WARN("SWSAnalog: stored water-air gap %u < %u — discarding flash calibration",
		           (unsigned)(uint16_t)(water - air), (unsigned)MIN_WATER_AIR_GAP);
		return false;
	}

	m_calib.threshold_air = (uint16_t)air;
	m_calib.threshold_water = (uint16_t)water;
	update_dynamic_threshold();
	m_calib.is_calibrated = true;
	m_calib.last_calibration_time = PMU::get_timestamp_ms() / 1000;
	m_calib.crc = crc16_compute((const uint8_t *)&m_calib, offsetof(SWSAnalogService::CalibrationData, crc), nullptr);

	// F-SWS-4 audit fix: extend peak validity check beyond ADC_INVALID_MAX.
	// A partially-corrupted SWS.CAL (LittleFS atomicity edge at brown-out, the
	// very scenario CLAUDE.md §6.4 addresses) could leave `peak` in a value
	// that's syntactically valid (>0, <=ADC_INVALID_MAX) but absurd vs the
	// water baseline (e.g. peak=200 with water=15000 — would pin the dynamic
	// threshold cap below actual underwater readings forever).
	//
	// Coherence rule: peak should be in [water/2, water*5]. Outside that
	// range it's almost certainly corruption — fall back to water baseline
	// (same path as peak=0). Logged as WARN so post-deploy forensics can
	// see the corruption signature.
	bool peak_coherent = (peak >= (m_calib.threshold_water / 2)) && (peak <= (m_calib.threshold_water * 5));
	if (peak > 0 && peak <= ADC_INVALID_MAX && peak_coherent) {
		m_observed_peak_adc = (uint16_t)peak;
	} else {
		// Peak is 0, invalid, or incoherent vs water baseline. Seed from
		// water baseline so update_dynamic_threshold's cap logic doesn't pin
		// threshold_high below actual underwater readings. Use water itself —
		// conservative, the cap will be re-learned from real ADC samples.
		if (peak > 0 && peak <= ADC_INVALID_MAX && !peak_coherent) {
			DEBUG_WARN(
			    "SWSAnalog: stored peak=%u INCOHERENT vs water=%u (SWS.CAL partial corruption?) — seeding from water",
			    (unsigned)peak, m_calib.threshold_water);
		} else {
			DEBUG_INFO("SWSAnalog: stored peak invalid — seeding from water baseline (%u)", m_calib.threshold_water);
		}
		m_observed_peak_adc = m_calib.threshold_water;
	}
	m_observed_peak_crc = crc16_compute((const uint8_t *)&m_observed_peak_adc, sizeof(m_observed_peak_adc), nullptr);

	DEBUG_INFO("SWSAnalog: Calibration restored from SWS.CAL - air=%u water=%u thresh=%u peak=%u",
	           m_calib.threshold_air, m_calib.threshold_water, m_calib.threshold_current, m_observed_peak_adc);
	return true;
}

// ═══════════════════════════════════════════════════════
//  CALIBRATION INTERNALS — validate, air/water baseline, threshold, filter, delay
// ═══════════════════════════════════════════════════════

/// @brief Validate noinit RAM calibration via CRC16. Falls back to flash if invalid.
/// @return true if calibration data is valid and ready for use.
bool SWSAnalogService::validate_calibration_data() {
	uint16_t calculated_crc =
	    crc16_compute((const uint8_t *)&m_calib, offsetof(SWSAnalogService::CalibrationData, crc), nullptr);
	if (calculated_crc != m_calib.crc) {
		DEBUG_WARN("SWSAnalog: CRC mismatch (calc=%u stored=%u)", calculated_crc, m_calib.crc);
		return false;
	}
	if (m_calib.threshold_air > ADC_INVALID_MAX) {
		return false;
	}
	if (m_calib.threshold_water > ADC_INVALID_MAX) {
		return false;
	}
	if (m_calib.threshold_air >= m_calib.threshold_water) {
		return false;
	}
	return m_calib.is_calibrated;
}

/// @brief Perform initial air baseline calibration (first boot or after CRC failure).
/// Reads N samples, uses manual hint or observed peak to estimate water baseline.
void SWSAnalogService::calibrate_air_baseline() {
	// Check for manual calibration from SWS.CAL (set via $SCALW,8,offset,value)
	// Offset 0 = expected water ADC, Offset 1 = expected air ADC
	double hint_water = 0.0, hint_air = 0.0;
	calibration_read(hint_water, 0);
	calibration_read(hint_air, 1);

	if (hint_water > 0 && hint_air > 0 && hint_water > hint_air) {
		m_calib.threshold_air = (uint16_t)hint_air;
		m_calib.threshold_water = (uint16_t)hint_water;

		update_dynamic_threshold();
		m_calib.is_calibrated = true;
		m_calib.last_calibration_time = PMU::get_timestamp_ms() / 1000;
		m_calib.crc =
		    crc16_compute((const uint8_t *)&m_calib, offsetof(SWSAnalogService::CalibrationData, crc), nullptr);

		DEBUG_INFO("SWSAnalog: Calib - air=%u water=%u thresh=%u (from SWS.CAL)", m_calib.threshold_air,
		           m_calib.threshold_water, m_calib.threshold_current);
		save_calibration_to_flash();
		return;
	}

	// Auto-calibration: sample the ADC
	const int NUM_SAMPLES = 10;
	uint32_t sum = 0;
	int valid_count = 0;
	uint16_t min_val = 0xFFFF, max_val = 0;

	for (int i = 0; i < NUM_SAMPLES; i++) {
		uint16_t value = read_analog_sws();
		if (is_value_valid(value)) {
			sum += value;
			valid_count++;
			if (value < min_val) min_val = value;
			if (value > max_val) max_val = value;
		}
		PMU::delay_ms(100);
	}

	if (valid_count == 0) {
		DEBUG_ERROR("SWSAnalog: No valid samples during calibration");
		return;
	}

	uint16_t avg = (uint16_t)(sum / valid_count);

	// Heuristic: if avg > WATER_DETECT_HEURISTIC, we're likely in water already.
	// With 14-bit ADC (0-16383): dry electrode reads ~100-500, water reads ~8000-15000.
	// Threshold at ~50% of full scale to distinguish reliably.
	const uint16_t WATER_DETECT_HEURISTIC = 7000;

	if (avg > WATER_DETECT_HEURISTIC) {
		// Started calibration IN WATER — swap logic
		DEBUG_WARN("SWSAnalog: Calibration in water detected (avg=%u > %u)", avg, WATER_DETECT_HEURISTIC);
		m_calib.threshold_water = avg;
		// Estimate air as 1/3 of water (conservative)
		m_calib.threshold_air = avg / 3;
		if (m_calib.threshold_air < AIR_BASELINE_FLOOR) m_calib.threshold_air = AIR_BASELINE_FLOOR;
	} else {
		// Normal: started in air. Floor at AIR_BASELINE_FLOOR so the proactive
		// stuck-state recovery (section 6b in detection) doesn't fire on a
		// disconnected/zero-ADC boot — this is a clean baseline, not a collapse.
		m_calib.threshold_air = avg;
		if (m_calib.threshold_air < AIR_BASELINE_FLOOR) m_calib.threshold_air = AIR_BASELINE_FLOOR;
		if (m_calib.threshold_water == 0 || m_calib.threshold_water <= m_calib.threshold_air) {
			// Priority order for water estimate:
			// 1. Manual hint from SWS.CAL (only water provided)
			// 2. Observed peak from noinit (previous session)
			// 3. Heuristic: air×3 (fallback)
			if (hint_water > m_calib.threshold_air) {
				m_calib.threshold_water = (uint16_t)hint_water;
				DEBUG_INFO("SWSAnalog: Water from manual hint %u", m_calib.threshold_water);
			} else if (m_observed_peak_adc > m_calib.threshold_air * 2
			           && m_observed_peak_adc >= (uint16_t)(m_calib.threshold_air * MIN_WATER_AIR_RATIO)) {
				m_calib.threshold_water = m_observed_peak_adc;
				DEBUG_INFO("SWSAnalog: Water from observed peak %u", m_observed_peak_adc);
			} else {
				// Heuristic: water ≈ air × 3, capped at air × 5 to avoid
				// unreachable thresholds when air is high (wet electrode startup).
				m_calib.threshold_water = m_calib.threshold_air * 3;
				uint32_t max_water_estimate = (uint32_t)m_calib.threshold_air * 5;
				if (max_water_estimate > ADC_INVALID_MAX) max_water_estimate = ADC_INVALID_MAX;
				if (m_calib.threshold_water > (uint16_t)max_water_estimate)
					m_calib.threshold_water = (uint16_t)max_water_estimate;
			}
			if (m_calib.threshold_water > ADC_INVALID_MAX) m_calib.threshold_water = ADC_INVALID_MAX;
		}
	}

	update_dynamic_threshold();
	m_calib.is_calibrated = true;
	m_calib.last_calibration_time = PMU::get_timestamp_ms() / 1000;
	m_calib.crc = crc16_compute((const uint8_t *)&m_calib, offsetof(SWSAnalogService::CalibrationData, crc), nullptr);

	DEBUG_INFO("SWSAnalog: Calib - air=%u water=%u thresh=%u (started_in_%s)", m_calib.threshold_air,
	           m_calib.threshold_water, m_calib.threshold_current, avg > WATER_DETECT_HEURISTIC ? "WATER" : "AIR");

	save_calibration_to_flash();
}

/// @brief Update water baseline using EMA when underwater state is confirmed.
/// @param value  Current filtered ADC reading.
void SWSAnalogService::calibrate_water_baseline(uint16_t value) {
	if (!is_value_valid(value)) return;

	// Relaxed guard when water baseline is still estimated (no real peaks observed yet).
	// Estimated water (air*3) can be wildly off with dirty pins → allow full adaptation.
	bool water_is_estimated = (m_observed_peak_adc == 0);

	// EC-1: Aggressive alpha during first deployment for fast convergence.
	// When water is still estimated (peak==0), use alpha=0.50 for the first 5 samples
	// so water baseline converges quickly on the first real immersion.
	// After 5 samples (or once peak is established), revert to normal alpha.
	float alpha;
	if (water_is_estimated && m_fast_convergence_count < 5) {
		alpha = 0.50f;
	} else {
		alpha = m_alpha_percent / 100.0f;
	}

	uint16_t threshold_with_margin = m_calib.threshold_current + m_calib.hysteresis_value;
	uint16_t min_expected_water = water_is_estimated ? (uint16_t)(m_calib.threshold_air * MIN_WATER_AIR_RATIO)
	                                                 : (uint16_t)(m_calib.threshold_water * 0.85f);

	// Only apply air ratio guard when air baseline is reasonable (< 3000 ADC).
	// If air is high (e.g. from corrupted calibration), skip this check to allow recovery.
	bool air_ratio_ok =
	    (m_calib.threshold_air >= 3000) || (value >= (uint16_t)(m_calib.threshold_air * MIN_WATER_AIR_RATIO));

	// Water must exceed threshold and maintain minimum ratio to air.
	// No absolute ADC floor: allows operation in fresh/brackish water where
	// ADC values can be well below 2000.
	bool ok = (value > threshold_with_margin) && air_ratio_ok
	          && (value >= min_expected_water || value > m_calib.threshold_water);

	if (ok) {
		uint16_t new_water = (uint16_t)(alpha * value + (1.0f - alpha) * m_calib.threshold_water);
		// Cap at observed peak only when peak is established (>= value/2):
		// a stale peak from air calibration would otherwise pin water below
		// real underwater readings, blocking convergence on first immersion.
		if (m_observed_peak_adc >= (uint16_t)(value / 2) && new_water > m_observed_peak_adc) {
			new_water = m_observed_peak_adc;
		}
		m_calib.threshold_water = new_water;
		update_dynamic_threshold();
		m_calib.crc =
		    crc16_compute((const uint8_t *)&m_calib, offsetof(SWSAnalogService::CalibrationData, crc), nullptr);

		// EC-1: Track fast convergence progress
		if (water_is_estimated && m_fast_convergence_count < 5) {
			m_fast_convergence_count++;
			DEBUG_INFO("SWSAnalog: Fast convergence %u/5 alpha=%.2f water=%u", m_fast_convergence_count, (double)alpha,
			           new_water);
		}
	}
}

/// @brief Recompute threshold from air/water baselines with configurable ratio + hysteresis.
void SWSAnalogService::update_dynamic_threshold() {
	// Dynamic ratio: low contrast (biofouling) → threshold closer to water
	float contrast =
	    (m_calib.threshold_air > 0) ? (float)m_calib.threshold_water / (float)m_calib.threshold_air : 10.0f;
	m_contrast_x10 = (uint16_t)(contrast * 10.0f);

	float ratio;
	if (contrast >= 8.0f) {
		ratio = m_threshold_ratio_percent / 100.0f;  // Clean: 35%
	} else if (contrast >= 4.0f) {
		ratio = 0.50f;  // Moderate biofouling: midpoint
	} else {
		// Very low contrast (wet electrodes or severe biofouling):
		// threshold closer to air to ensure water readings can cross it.
		// Multi-level detection handles false surface prevention.
		ratio = 0.40f;
	}

	uint16_t range = m_calib.threshold_water - m_calib.threshold_air;
	m_calib.threshold_current = m_calib.threshold_air + (uint16_t)(range * ratio);

	// Enforce minimum gap between threshold and air baseline to prevent
	// false UW triggers from noise when baselines are close (stale calibration).
	uint16_t min_thresh = m_calib.threshold_air + THRESHOLD_MIN_ABOVE_AIR;
	if (m_calib.threshold_current < min_thresh) m_calib.threshold_current = min_thresh;

	m_calib.hysteresis_value = (uint16_t)(m_calib.threshold_current * m_hysteresis_percent / 100.0f);
	if (m_calib.hysteresis_value < 10) m_calib.hysteresis_value = 10;

	// Cap threshold+hysteresis at observed peak ADC so we never exceed actual readings.
	// B7: only trust the peak as a cap when it's plausible water (peak >= water/2).
	// A stale peak that has decayed below water (long surface period or noise spikes)
	// would otherwise pin threshold_high to peak*0.9, collapsing the threshold below
	// legitimate underwater readings and preventing dive detection.
	if (m_observed_peak_adc > 0 && m_calib.threshold_water > 0
	    && m_observed_peak_adc >= (uint16_t)(m_calib.threshold_water / 2)) {
		uint16_t max_thresh_high = m_observed_peak_adc;
		if (m_calib.threshold_current + m_calib.hysteresis_value > max_thresh_high) {
			// Adjust: keep threshold and reduce hysteresis, or lower threshold
			if (m_calib.threshold_current >= max_thresh_high) {
				m_calib.threshold_current = (uint16_t)(max_thresh_high * 0.90f);
				m_calib.hysteresis_value = (uint16_t)(max_thresh_high * 0.05f);
			} else {
				m_calib.hysteresis_value = max_thresh_high - m_calib.threshold_current;
			}
			if (m_calib.hysteresis_value < 10) m_calib.hysteresis_value = 10;
		}
	}

	DEBUG_TRACE("SWSAnalog: thresh=%u contrast=%.1f ratio=%.0f%% hyst=%u peak=%u", m_calib.threshold_current,
	            (double)contrast, (double)(ratio * 100), m_calib.hysteresis_value, m_observed_peak_adc);
}

/// @brief Add ADC value to MA2 history buffer and return filtered result.
/// @param value  New raw ADC reading.
/// @return Moving average of the last ADC_HISTORY_SIZE samples.
uint16_t SWSAnalogService::add_to_history_and_filter(uint16_t value) {
	m_adc_history[m_adc_history_idx] = value;
	m_adc_history_idx = (m_adc_history_idx + 1) % ADC_HISTORY_SIZE;
	if (m_adc_history_count < ADC_HISTORY_SIZE) m_adc_history_count++;

	uint32_t sum = 0;
	for (int i = 0; i < m_adc_history_count; i++) {
		sum += m_adc_history[i];
	}
	return (uint16_t)(sum / m_adc_history_count);
}

// Accepts 0..16383 (full 14-bit range).  Rejects ADC_READ_ERROR (UINT16_MAX)
// and any value above the 14-bit max (hardware fault or saturation).
/// @brief Check if ADC value is within valid 14-bit range.
/// @param value  ADC reading to validate.
/// @return true if 0 <= value <= ADC_INVALID_MAX.
bool SWSAnalogService::is_value_valid(uint16_t value) const {
	return (value <= ADC_INVALID_MAX);
}

/// @brief Adjust RC charge delay based on water/air contrast ratio.
/// Low contrast (biofouling) → increase delay. High contrast (clean) → decrease delay.
///
/// Frozen during guided-calibration sampling phases so all CALIB_NUM_SAMPLES
/// readings averaged into the air/water result use the same RC charge time.
/// Without this guard, an EMA pull on m_calib.threshold_air mid-sampling
/// (e.g. coherence recalib in section 1b of detector_state) would change
/// m_sample_delay_us between samples and bias the average.
void SWSAnalogService::adjust_sample_delay() {
	if (m_calib_phase == CalibPhase::AIR_SAMPLING || m_calib_phase == CalibPhase::WATER_SAMPLING) return;
	if (m_calib.threshold_air == 0) return;
	uint16_t contrast = m_contrast_x10;
	uint32_t old_delay = m_sample_delay_us;

	// GUARD: if air baseline is near-zero, readings are likely invalid because
	// the RC circuit doesn't charge enough at the current delay. Force delay UP
	// to allow proper charging — this breaks the death spiral where low delay →
	// zero readings → air drops → contrast inflates → delay stays low.
	if (m_calib.threshold_air < AIR_BASELINE_RECOVER) {
		if (m_sample_delay_us < m_delay_max_us) {
			m_sample_delay_us = m_sample_delay_us * 11 / 10;  // +10%
			if (m_sample_delay_us > m_delay_max_us) m_sample_delay_us = m_delay_max_us;
		}
	}
	// Normal adaptive: reduce on low contrast (biofouling), increase on high contrast
	else if (contrast < CONTRAST_LOW_THRESHOLD && m_sample_delay_us > m_delay_min_us) {
		m_sample_delay_us = m_sample_delay_us * 3 / 4;  // -25%
		if (m_sample_delay_us < m_delay_min_us) m_sample_delay_us = m_delay_min_us;
	} else if (contrast > CONTRAST_HIGH_THRESHOLD && m_sample_delay_us < m_delay_max_us) {
		m_sample_delay_us = m_sample_delay_us * 11 / 10;  // +10%
		if (m_sample_delay_us > m_delay_max_us) m_sample_delay_us = m_delay_max_us;
	}

	if (old_delay != m_sample_delay_us) {
		DEBUG_TRACE("SWSAnalog: Adaptive delay %uus -> %uus (contrast=%u.%u)", old_delay, m_sample_delay_us,
		            contrast / 10, contrast % 10);
		// F-SWS-7 audit fix: persist the converged delay to noinit so a soft
		// reset (WDT/POR) restores the right value instead of falling back
		// to UNP08 default. CRC protects against torn writes mid-update.
		m_sample_delay_us_noinit = m_sample_delay_us;
		m_sample_delay_us_crc =
		    crc16_compute((const uint8_t *)&m_sample_delay_us_noinit, sizeof(m_sample_delay_us_noinit), nullptr);
	}
}

// should_recalibrate() is in sws_analog_detection.cpp (called by detector_state)

// ═══════════════════════════════════════════════════════
//  GUIDED CALIBRATION STATE MACHINE
// ═══════════════════════════════════════════════════════

/// @brief Advance the guided-calibration state machine by one sample.
///
/// Called once per detector tick from SWSAnalogService::detector_state(), which
/// is where a fresh ADC reading is available. It used to be the last 183 lines
/// of that 1014-line function; the detection logic and the calibration walk-through
/// have nothing in common beyond needing the same sample.
///
/// Does nothing while m_calib_phase is IDLE. The timeout counter, however, runs
/// for every phase except IDLE and DONE -- that is what rescues a calibration
/// the operator walked away from.
///
/// @param raw_value  the ADC sample for this tick
void SWSAnalogService::tick_guided_calibration(uint16_t raw_value) {
	if (m_calib_phase != CalibPhase::IDLE && m_calib_phase != CalibPhase::DONE) {
		m_calib_timeout_ticks++;
		if (m_calib_timeout_ticks >= GUIDED_CALIB_TIMEOUT_TICKS) {
			DEBUG_WARN("SWSAnalog: Guided calibration TIMEOUT after %u ticks — cancelling", m_calib_timeout_ticks);
			cancel_guided_calibration();
			if (m_calib_notify) {
				CalibResult r = { 0, 0, 0 };  // status 0 = timeout
				m_calib_notify(r);
			}
		}
	}
	if (m_calib_phase != CalibPhase::IDLE) {
		switch (m_calib_phase) {
		case CalibPhase::AIR_WAITING:
			// Waiting for stable readings in air before sampling.
			// Use UINT16_MAX as "no prior sample" sentinel (outside valid
			// 14-bit ADC range 0..16383) — the previous `prev > 0` proxy
			// mis-classified a legitimate raw=0 reading (disconnected
			// electrode, uncharged cap) as "no prior sample" and the
			// stable_count could never advance → stuck until the 5 min
			// GUIDED_CALIB_TIMEOUT_TICKS fired.
			if (m_calib_prev_value != UINT16_MAX) {
				uint16_t delta = (raw_value > m_calib_prev_value) ? (raw_value - m_calib_prev_value)
				                                                  : (m_calib_prev_value - raw_value);
				if (delta < CALIB_STABILITY_TOLERANCE) {
					m_calib_stable_count++;
				} else {
					m_calib_stable_count = 0;
				}
			}
			m_calib_prev_value = raw_value;
			if (m_calib_stable_count >= CALIB_STABILITY_THRESHOLD) {
				m_calib_phase = CalibPhase::AIR_SAMPLING;
				m_calib_sum = 0;
				m_calib_count = 0;
				if (status_led) status_led->flash(RGBLedColor::GREEN, 150);
				DEBUG_INFO("SWSAnalog: Guided calib — sampling AIR");
			}
			break;

		case CalibPhase::AIR_SAMPLING:
			m_calib_sum += raw_value;
			m_calib_count++;
			if (m_calib_count >= CALIB_NUM_SAMPLES) {
				m_calib_air_result = (uint16_t)(m_calib_sum / m_calib_count);
				if (status_led) status_led->set(RGBLedColor::GREEN);
				DEBUG_INFO("SWSAnalog: Guided calib — AIR=%u — now place in WATER", m_calib_air_result);
				// EC-4: Non-blocking pause — transition via state machine tick
				m_calib_phase = CalibPhase::AIR_DONE_PAUSE;
				m_calib_stable_count = 0;
				m_calib_prev_value = UINT16_MAX;  // sentinel: no prior sample yet
			}
			break;

		case CalibPhase::AIR_DONE_PAUSE:
			// EC-4: 1 tick pause (1s via service_next_schedule_in_ms) replaces PMU::delay_ms(1000)
			m_calib_phase = CalibPhase::WATER_WAITING;
			if (status_led) status_led->flash(RGBLedColor::BLUE, 500);
			break;

		case CalibPhase::WATER_WAITING:
			// Wait for readings significantly above air baseline.
			// Same UINT16_MAX sentinel pattern as AIR_WAITING — needed here
			// even though water raw is typically >0, because the phase
			// entry resets prev to UINT16_MAX and the first sample must
			// not be misclassified.
			if (raw_value > m_calib_air_result * 2) {
				if (m_calib_prev_value != UINT16_MAX) {
					uint16_t delta = (raw_value > m_calib_prev_value) ? (raw_value - m_calib_prev_value)
					                                                  : (m_calib_prev_value - raw_value);
					if (delta < CALIB_STABILITY_TOLERANCE) {
						m_calib_stable_count++;
					} else {
						m_calib_stable_count = 0;
					}
				}
			} else {
				m_calib_stable_count = 0;
			}
			m_calib_prev_value = raw_value;
			if (m_calib_stable_count >= CALIB_STABILITY_THRESHOLD) {
				m_calib_phase = CalibPhase::WATER_SAMPLING;
				m_calib_sum = 0;
				m_calib_count = 0;
				if (status_led) status_led->flash(RGBLedColor::BLUE, 150);
				DEBUG_INFO("SWSAnalog: Guided calib — sampling WATER");
			}
			break;

		case CalibPhase::WATER_SAMPLING:
			m_calib_sum += raw_value;
			m_calib_count++;
			if (m_calib_count >= CALIB_NUM_SAMPLES) {
				m_calib_water_result = (uint16_t)(m_calib_sum / m_calib_count);
				DEBUG_INFO("SWSAnalog: Guided calib — WATER=%u", m_calib_water_result);

				// Validate: water must be significantly above air
				bool success = (m_calib_water_result > m_calib_air_result * 2);
				m_calib_water_success = success;
				if (success) {
					// Save to SWS.CAL (batch writes, single flash serialize)
					m_manual_calib.write(CAL_OFFSET_HINT_WATER, (double)m_calib_water_result);
					m_manual_calib.write(CAL_OFFSET_HINT_AIR, (double)m_calib_air_result);
					m_manual_calib.save(true);

					// Apply immediately
					m_calib.threshold_air = m_calib_air_result;
					m_calib.threshold_water = m_calib_water_result;
					update_dynamic_threshold();
					m_calib.is_calibrated = true;
					m_calib.last_calibration_time = PMU::get_timestamp_ms() / 1000;
					m_calib.crc = crc16_compute((const uint8_t *)&m_calib,
					                            offsetof(SWSAnalogService::CalibrationData, crc), nullptr);
					save_calibration_to_flash();
					DEBUG_INFO(
					    "SWSAnalog: Guided calib SUCCESS — air=%u water=%u thresh=%u (waiting resurface for ACK)",
					    m_calib_air_result, m_calib_water_result, m_calib.threshold_current);
				} else {
					DEBUG_WARN("SWSAnalog: Guided calib FAILED — water=%u not >> air=%u (waiting resurface for ACK)",
					           m_calib_water_result, m_calib_air_result);
				}

				// Defer the WHITE/RED result flash + GUI notify until the tag is
				// back in air: BLE is blocked by water, so firing the notify
				// here would silently drop the ACK and the GUI would never see
				// the calibration outcome. Hold BLUE solid as "phase done,
				// remove from water" cue. Global GUIDED_CALIB_TIMEOUT_TICKS
				// (5 min) still applies via cancel_guided_calibration if the
				// operator never resurfaces.
				if (status_led) status_led->set(RGBLedColor::BLUE);
				m_calib_phase = CalibPhase::WAIT_RESURFACE_FOR_ACK;
				m_calib_count = 0;
			}
			break;

		case CalibPhase::WAIT_RESURFACE_FOR_ACK:
			// Wait for raw to drop back below half of the water reading — at
			// that point the tag is clearly out of water and BLE is back up,
			// so the GUI notify will actually reach the host.
			// Failure case: m_calib_water_result might be 0 if validation
			// failed because both readings were near-zero; fall back to a
			// small absolute threshold to avoid stuck-forever.
			{
				uint16_t resurface_thresh = m_calib_water_result > 0 ? (uint16_t)(m_calib_water_result / 2) : 1000;
				if (raw_value < resurface_thresh) {
					if (status_led) {
						status_led->flash(m_calib_water_success ? RGBLedColor::WHITE : RGBLedColor::RED, 200);
					}
					if (m_calib_notify) {
						CalibResult r = { (uint8_t)(m_calib_water_success ? 1 : 2), m_calib_air_result,
						                  m_calib_water_result };
						m_calib_notify(r);
					}
					DEBUG_INFO("SWSAnalog: Guided calib — resurfaced (raw=%u<%u), ACK sent", raw_value,
					           resurface_thresh);
					m_calib_phase = CalibPhase::COMPLETION_PAUSE;
					m_calib_count = 0;  // reused as tick counter for COMPLETION_PAUSE
				}
			}
			break;

		case CalibPhase::COMPLETION_PAUSE:
			// Non-blocking 3s pause keeps the WHITE (success) or RED (fail)
			// flash visible long enough for a bench operator to read it.
			// Then hand the LED back to whatever set up the callback — the
			// DTE handler dispatches the proper ledsm event there
			// (SetLEDConfigConnected for SWSCAL), so the LED returns to
			// its pre-calibration state instead of staying frozen on the
			// success/failure flash or going off.
			m_calib_count++;
			if (m_calib_count >= 3) {
				m_calib_phase = CalibPhase::DONE;
				m_test_mode = false;
				if (s_instance) s_instance->stop();
				if (m_on_test_stop) m_on_test_stop();
			}
			break;

		case CalibPhase::IDLE:
		case CalibPhase::DONE:
		default: break;
		}
	}
}
