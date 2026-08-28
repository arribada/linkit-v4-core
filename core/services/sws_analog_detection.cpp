/**
 * @file sws_analog_detection.cpp
 * @brief SWS analog — multi-level surface detection, safety timeouts, detector_state.
 */

#include <cstdint>
#include "sws_analog_service.hpp"
#include "sws_analog_constants.hpp"
#include "debug.hpp"
#include "pmu.hpp"
#include "gpio.hpp"
#include "rgb_led.hpp"

extern RGBLed *status_led;

/// @brief Check if calibration interval has elapsed and recalibration is needed.
/// @return true if m_calib_interval_sec has passed since last calibration.
///
/// Audit 2026-05 R-CODE-06: explicit underflow guard. After a reset,
/// PMU::get_timestamp_ms() restarts at 0 while m_calib.last_calibration_time
/// persists in noinit RAM with the value from the previous session. Without
/// this guard, the subtraction underflows uint64 and the wrap-around result
/// happens to be >= m_calib_interval_sec — i.e. recalibration fires
/// "correctly" through undefined behavior. Make the intent explicit.
bool SWSAnalogService::should_recalibrate() const {
	if (m_calib_interval_sec == 0) return false;
	uint64_t now_sec = PMU::get_timestamp_ms() / 1000;
	if (now_sec < m_calib.last_calibration_time) {
		// Post-reboot clock regression — force a recalibration explicitly.
		return true;
	}
	uint64_t elapsed = now_sec - m_calib.last_calibration_time;
	return (elapsed >= m_calib_interval_sec);
}

/// @brief Check safety timeout — force surface if max dive time exceeded.
/// @param current_state  Current underwater state (true=submerged).
/// @return true if timeout override forces a state change.
bool SWSAnalogService::check_safety_timeouts(bool current_state) {
	uint64_t now = PMU::get_timestamp_ms() / 1000;
	if (m_last_state_change_time == 0) {
		m_last_state_change_time = now;
		m_time_in_current_state = 0;
	} else {
		m_time_in_current_state = now - m_last_state_change_time;
	}

	if (current_state && m_max_dive_time_sec > 0 && m_time_in_current_state >= m_max_dive_time_sec) {
		DEBUG_WARN("SWSAnalog: Max dive time %us | recalibrating", m_time_in_current_state);
		return true;
	}
	return false;
}

// ═══════════════════════════════════════════════════════
//  MAIN DETECTION: 5-LEVEL SURFACE DETECTION
// ═══════════════════════════════════════════════════════

/// @brief Main detection logic — read ADC, filter, 5-level surface detection, calibrate, log.
/// @return true if underwater, false if at surface.
bool SWSAnalogService::detector_state() {
	// === 0. TIMING INSTRUMENTATION (2026-05 latency investigation) ===
	// Measures the actual scheduler cadence between two detector_state() calls.
	// Nominal: SAMPLING_UNDER_FREQ (default 1000 ms) under water, SAMPLING_SURF_FREQ
	// (10000 ms) at surface, or SWS_TEST_MODE_SAMPLE_MS (100 ms) in test mode.
	// A delta significantly larger than the nominal period indicates the scheduler
	// was held by another task (FsLog flush, SPI burst, etc.). Disabled by default;
	// enable by setting SWS_TIMING_LOG_ENABLE=1 at build time (see definition below).
#ifndef SWS_TIMING_LOG_ENABLE
#define SWS_TIMING_LOG_ENABLE 0
#endif
#if SWS_TIMING_LOG_ENABLE
	uint64_t sample_t0_ms = PMU::get_timestamp_ms();
	uint32_t sample_delta_ms = (m_last_sample_ms == 0) ? 0 : static_cast<uint32_t>(sample_t0_ms - m_last_sample_ms);
	m_last_sample_ms = sample_t0_ms;
#endif

	// === 1. READ ADC ===
	uint16_t raw_value = read_analog_sws();
	if (!is_value_valid(raw_value)) {
		DEBUG_WARN("SWSAnalog: Invalid ADC %u", raw_value);
		// Stuck-failure escape: if the SAADC silicon or SWS frontend dies
		// while underwater, returning m_current_state forever pins the
		// device in "underwater" and blocks all TX. After
		// MAX_CONSECUTIVE_INVALID_ADC reads we force-flip to surface so
		// the device at least keeps transmitting. We DO NOT reset the
		// counter here on purpose — only a single valid read clears it.
		if (m_consecutive_invalid_adc < UINT16_MAX) {
			m_consecutive_invalid_adc++;
		}
		// F-SWS-6 audit fix: periodic heartbeat log every 10 invalid reads
		// makes slow-bleed silicon/frontend degradation visible in post-deploy
		// forensics instead of being silent until the safety trigger fires.
		if (m_consecutive_invalid_adc > 0 && m_consecutive_invalid_adc % 10 == 0) {
			DEBUG_WARN("SWSAnalog: still bleeding invalid ADC reads (count=%u)", m_consecutive_invalid_adc);
		}
		if (m_consecutive_invalid_adc == MAX_CONSECUTIVE_INVALID_ADC) {
			DEBUG_ERROR("SWSAnalog: %u consecutive invalid ADC reads — forcing state=surface to keep TX alive",
			            m_consecutive_invalid_adc);
			return false;  // force surface
		}
		if (m_consecutive_invalid_adc > MAX_CONSECUTIVE_INVALID_ADC) {
			// Stay in forced-surface until ADC recovers
			return false;
		}
		return m_current_state;
	}
	if (m_consecutive_invalid_adc > 0) {
		DEBUG_INFO("SWSAnalog: ADC recovered after %u invalid reads", m_consecutive_invalid_adc);
		m_consecutive_invalid_adc = 0;
	}

	// === 1b. COHERENCE CHECK (first sample + continuous) ===
	// Detects calibration mismatch vs actual ADC readings.
	// First sample: catches stale noinit data.
	// Continuous: catches environment changes (e.g. device moved from indoor to outdoor).
	{
		bool calib_incoherent = false;
		// Track which coherence case fired so the recalib branch knows whether
		// we are now in water (case 1) or in air (case 2). Without this, the
		// generic raw>7000 heuristic would assume "in air" for moderate-salinity
		// water (ADC=4800) and set water=raw*3 — driving the threshold above
		// actual readings and blocking detection.
		bool incoherent_in_water = false;

		if (!m_first_sample_done) {
			m_first_sample_done = true;

			// F-SWS-3 audit fix: explicitly seed m_prev_raw with the current
			// reading on the first sample. Without this seed, m_prev_raw is
			// 0-initialised and L1/L2 (lines ~253, ~261) skip due to the
			// `prev_raw > 0` guard for the entire first sample's check. With
			// the seed, the existing line ~213 (`prev_raw = m_prev_raw`)
			// captures the same-sample raw_value for L1/L2 — delta computes
			// to 0 so L1/L2 still does not falsely fire on sample #1, but the
			// m_prev_raw state is properly initialised. Defense-in-depth
			// against future code changes that might re-order the prev_raw
			// capture relative to L1/L2 checks.
			m_prev_raw = raw_value;

			// Case 1: Stored air is low, but reading is way above water → wrong medium
			if (raw_value > m_calib.threshold_current + m_calib.hysteresis_value * 3
			    && raw_value > m_calib.threshold_water * 1.3f) {
				DEBUG_WARN("SWSAnalog: Coherence fail - raw=%u >> water=%u, recalibrating", raw_value,
				           m_calib.threshold_water);
				calib_incoherent = true;
				incoherent_in_water = true;
			}
			// Case 2: Stored air is high (calibrated in water), but reading is low → in air
			else if (raw_value < m_calib.threshold_air * 0.5f && m_calib.threshold_air > 5000) {
				DEBUG_WARN("SWSAnalog: Coherence fail - raw=%u << air=%u, recalibrating", raw_value,
				           m_calib.threshold_air);
				calib_incoherent = true;
			}
		}

		// EC-5: Continuous coherence — require 3 consecutive samples > water×2
		// before adapting. Prevents splash/wave from corrupting water baseline.
		// Runs in both states: at SURF it catches environment changes
		// (deployment moved), at UW it catches salinity ramps that the EMA
		// alpha=0.19 in calibrate_water_baseline would converge to too slowly.
		// Note: time_in_state is *not* gated here — a state change resets the
		// timer and would otherwise reset the coherence count mid-ramp.
		if (!calib_incoherent && m_first_sample_done && m_calib.threshold_water > 0
		    && raw_value > m_calib.threshold_water * 2) {
			m_coherence_high_count++;
			if (m_coherence_high_count >= 3) {
				inc_diag(m_diag.coherence_recalib_count);
				DEBUG_WARN("SWSAnalog: Continuous coherence - raw=%u >> water=%u (%u consecutive), adapting water",
				           raw_value, m_calib.threshold_water, m_coherence_high_count);
				// Cap at observed peak only when peak is established (>= raw/2):
				// a stale peak from air calibration would otherwise pin water below
				// real underwater readings.
				uint16_t new_water = raw_value;
				if (m_observed_peak_adc >= (uint16_t)(raw_value / 2) && new_water > m_observed_peak_adc) {
					new_water = m_observed_peak_adc;
				}
				m_calib.threshold_water = new_water;
				update_dynamic_threshold();
				m_calib.crc =
				    crc16_compute((const uint8_t *)&m_calib, offsetof(SWSAnalogService::CalibrationData, crc), nullptr);
				m_coherence_high_count = 0;
				// Flash save deferred to next state transition (section 10) to avoid
				// excessive writes during rapid convergence
			}
		} else {
			m_coherence_high_count = 0;
		}

		if (calib_incoherent) {
			inc_diag(m_diag.coherence_recalib_count);
			memset(&m_calib, 0, sizeof(m_calib));
			if (incoherent_in_water || raw_value > 7000) {
				// Case 1 OR raw is unambiguously water (ADC > heuristic):
				// raw IS water, estimate air at raw/3.
				m_calib.threshold_water = raw_value;
				m_calib.threshold_air = raw_value / 3;
			} else {
				// Case 2 (low reading, was calibrated in water): raw is air.
				m_calib.threshold_air = raw_value;
				m_calib.threshold_water = raw_value * 3;
				if (m_observed_peak_adc > 0 && m_calib.threshold_water > m_observed_peak_adc)
					m_calib.threshold_water = m_observed_peak_adc;
				if (m_calib.threshold_water > ADC_INVALID_MAX) m_calib.threshold_water = ADC_INVALID_MAX;
			}
			// Apply AIR_BASELINE_FLOOR — coherence recalib can be triggered with
			// raw≈0 (disconnected electrode) which would set air=0 and trip the
			// section 6b stuck-state recovery on the very next sample.
			if (m_calib.threshold_air < AIR_BASELINE_FLOOR) m_calib.threshold_air = AIR_BASELINE_FLOOR;
			if (m_calib.threshold_water <= m_calib.threshold_air * MIN_WATER_AIR_RATIO)
				m_calib.threshold_water = m_calib.threshold_air * MIN_WATER_AIR_RATIO;
			update_dynamic_threshold();
			m_calib.is_calibrated = true;
			m_calib.last_calibration_time = PMU::get_timestamp_ms() / 1000;
			m_calib.crc =
			    crc16_compute((const uint8_t *)&m_calib, offsetof(SWSAnalogService::CalibrationData, crc), nullptr);
			DEBUG_TRACE("SWSAnalog: Recalib from coherence - air=%u water=%u thresh=%u", m_calib.threshold_air,
			            m_calib.threshold_water, m_calib.threshold_current);
			save_calibration_to_flash();
		}
	}

	// Save prev_raw BEFORE filtering overwrites history
	uint16_t prev_raw = m_prev_raw;
	m_prev_raw = raw_value;

	uint16_t filtered_value = add_to_history_and_filter(raw_value);

	// === 2. TIME TRACKING ===
	bool timeout_override = check_safety_timeouts(m_current_state);

	// === 2b. HEARTBEAT WARN (audit 2026-05 R-CODE-04) ===
	// Log a WARNING when no state change has occurred for an unusually long
	// time. Pure observability — no automatic action because a turtle can
	// legitimately stay submerged or at the surface for many hours.
	// Uses persistent system.log (DEBUG_WARN) for post-deployment forensics.
	{
		uint32_t threshold = m_current_state ? m_heartbeat_warn_uw_sec : m_heartbeat_warn_surf_sec;
		if (threshold > 0 && m_time_in_current_state >= threshold) {
			uint64_t now_sec = PMU::get_timestamp_ms() / 1000;
			if (m_last_heartbeat_warn_time == 0
			    || (now_sec - m_last_heartbeat_warn_time) >= HEARTBEAT_WARN_REPEAT_SEC) {
				DEBUG_WARN("SWSAnalog: heartbeat — no state change for %llus (state=%s)",
				           (unsigned long long)m_time_in_current_state, m_current_state ? "UW" : "SURF");
				m_last_heartbeat_warn_time = now_sec;
			}
		}
	}

	// === 3. LEVEL 1 & 2: FAST RAW DROP (uses raw, not filtered, for speed) ===
	uint8_t surface_level = 0;

	// Proximity guard: blocks L-overrides if still close to water peak.
	// Adaptive: relaxes to 99% when contrast < 5x (biofouling).
	uint16_t proximity_ref = m_calib.threshold_water;
	if (m_peak_adc_since_underwater > proximity_ref) proximity_ref = m_peak_adc_since_underwater;
	uint8_t guard_pct = (m_contrast_x10 < 50) ? PROXIMITY_GUARD_BIOFOULING : PROXIMITY_GUARD_PERCENT;
	bool proximity_ok = (proximity_ref == 0) || (filtered_value < (uint16_t)(proximity_ref * guard_pct / 100.0f));

	if (m_current_state && prev_raw > 0
#if OVERRIDE_MIN_TIME_SEC > 0
	    && m_time_in_current_state >= OVERRIDE_MIN_TIME_SEC
#endif
	    && proximity_ok) {

		// LEVEL 1: Sudden single-sample drop from previous raw
		// Uses prev_raw (not recent_peak) to avoid false triggers from gradual drift
		if (prev_raw > 0 && raw_value < prev_raw) {
			uint16_t single_drop_pct = (uint16_t)((uint32_t)(prev_raw - raw_value) * 100 / prev_raw);
			if (single_drop_pct >= L1_DROP_PERCENT) {
				surface_level = 1;
			}
		}

		// LEVEL 2: Two consecutive significant raw drops with cumulative threshold
		// Each step must exceed L2_MIN_STEP_PERCENT to filter gradual drift/salinity changes
		if (surface_level == 0) {
			if (raw_value < prev_raw) {
				uint16_t step_pct = (uint16_t)((uint32_t)(prev_raw - raw_value) * 100 / prev_raw);
				if (step_pct >= L2_MIN_STEP_PERCENT) {
					if (m_consecutive_raw_drops == 0) {
						m_drop_reference = prev_raw;
					}
					m_consecutive_raw_drops++;

					uint16_t cumul_pct = m_drop_reference
					                         ? static_cast<uint16_t>(static_cast<uint32_t>(m_drop_reference - raw_value)
					                                                 * 100 / m_drop_reference)
					                         : 0;
					if (m_consecutive_raw_drops >= L2_MIN_CONSECUTIVE && cumul_pct >= L2_DROP_PERCENT) {
						surface_level = 2;
					}
				} else {
					m_consecutive_raw_drops = 0;  // Drift, not a real drop
				}
			} else {
				m_consecutive_raw_drops = 0;
			}
		}
	} else if (!m_current_state) {
		m_consecutive_raw_drops = 0;
	}

	// === 4. LEVEL 3: TREND MA3 (consecutive MA3 decreases → electrode drying) ===
	m_trend_buffer[m_trend_buffer_idx] = filtered_value;
	m_trend_buffer_idx = (m_trend_buffer_idx + 1) % TREND_MA_SIZE;
	if (m_trend_buffer_count < TREND_MA_SIZE) m_trend_buffer_count++;

	uint16_t current_ma3 = filtered_value;
	if (m_trend_buffer_count >= TREND_MA_SIZE) {
		uint32_t ma_sum = 0;
		for (int i = 0; i < TREND_MA_SIZE; i++)
			ma_sum += m_trend_buffer[i];
		current_ma3 = (uint16_t)(ma_sum / TREND_MA_SIZE);
	}

	if (surface_level == 0 && m_current_state && m_prev_ma3 > 0 && m_trend_buffer_count >= TREND_MA_SIZE &&
#if OVERRIDE_MIN_TIME_SEC > 0
	    m_time_in_current_state >= OVERRIDE_MIN_TIME_SEC &&
#endif
	    proximity_ok) {

		if (current_ma3 < m_prev_ma3) {
			// MA3 decreased
			if (m_ma3_trend_count == 0) {
				m_ma3_trend_start = m_prev_ma3;  // record start of trend
			}
			m_ma3_trend_count++;
		} else {
			// Allow 1 flat/increase without full reset (noise tolerance)
			if (m_ma3_trend_count > 0) m_ma3_trend_count--;
		}

		if (m_ma3_trend_count >= L3_MIN_CONSECUTIVE && m_ma3_trend_start > 0) {
			uint16_t ma3_drop = (uint16_t)((uint32_t)(m_ma3_trend_start - current_ma3) * 100 / m_ma3_trend_start);
			if (ma3_drop >= L3_DROP_PERCENT) {
				surface_level = 3;
			}
		}
	} else if (!m_current_state) {
		m_ma3_trend_count = 0;
		m_ma3_trend_start = 0;
	}

	m_prev_ma3 = current_ma3;

	// === 5. LEVEL 4 & 5: RELATIVE AND CUMULATIVE DROP ===
	if (m_current_state) {
		if (filtered_value > m_peak_adc_since_underwater) m_peak_adc_since_underwater = filtered_value;

		// Recent peak: decays 2%/sample toward reading (prevents drift false-trigger)
		if (raw_value > m_recent_peak || m_recent_peak == 0) {
			m_recent_peak = raw_value;
		} else {
			// Decay: recent_peak moves 2% toward current reading each sample
			// Slow decay preserves sensitivity with long sampling periods (10s)
			m_recent_peak = (uint16_t)(m_recent_peak * 0.98f + raw_value * 0.02f);
		}
	}

	if (surface_level == 0 && m_current_state &&
#if OVERRIDE_MIN_TIME_SEC > 0
	    m_time_in_current_state >= OVERRIDE_MIN_TIME_SEC &&
#endif
	    proximity_ok) {

		// LEVEL 4: Drop relative to water baseline. Require:
		//  (a) 2 consecutive filtered samples below water*(1-L4_DROP%) — filters
		//      out single-sample noise during biofouling stage transitions
		//      where the water baseline lags the actual water level.
		//  (b) raw_value < threshold_high — confirms raw is in the "air zone"
		//      (below the dive-detection threshold). Without this guard, L4
		//      fires falsely when water_baseline is stale and raw is still
		//      water-level (biofouling reduced actual conductivity → real
		//      readings sit below the stale water_baseline×0.85 but well above
		//      threshold_high).
		const uint16_t threshold_high_local = m_calib.threshold_current + m_calib.hysteresis_value;
		if (m_calib.threshold_water > 0) {
			uint16_t water_thresh = (uint16_t)(m_calib.threshold_water * (1.0f - L4_DROP_PERCENT / 100.0f));
			if (filtered_value < water_thresh && raw_value < threshold_high_local) {
				m_l4_consecutive_below++;
				if (m_l4_consecutive_below >= 2) {
					surface_level = 4;
				}
			} else {
				m_l4_consecutive_below = 0;
			}
		}

		// LEVEL 5: Cumulative drop from peak during this dive.
		// Same raw guard as L4: raw must be in the air zone for L5 to fire.
		// Otherwise stale peaks (after biofouling reduction) trigger L5 falsely.
		// EC-6: Guard against underflow if filtered_value > peak.
		if (surface_level == 0 && m_peak_adc_since_underwater > 0 && filtered_value < m_peak_adc_since_underwater
		    && raw_value < threshold_high_local && m_time_in_current_state > L5_MIN_TIME_SEC) {
			uint16_t drop = (uint16_t)((uint32_t)(m_peak_adc_since_underwater - filtered_value) * 100
			                           / m_peak_adc_since_underwater);
			if (drop >= L5_DROP_PERCENT) {
				surface_level = 5;
			}
		}
	}

	// === 6. SURFACE BASELINE TRACKING (blocked during lockout) ===
	if (!m_current_state && m_time_in_current_state > MIN_SURFACE_TIME_FOR_ADAPT && m_surface_lockout_remaining == 0) {
		// B2: reject sub-floor readings — they likely come from a dry/disconnected
		// electrode where the RC circuit hasn't charged. Letting them in pulls
		// the buffer average toward zero and corrupts the periodic Air recalib.
		if (filtered_value < m_calib.threshold_current && filtered_value >= AIR_BASELINE_FLOOR) {
			m_surface_readings[m_surface_readings_idx] = filtered_value;
			m_surface_readings_idx = (m_surface_readings_idx + 1) % SURFACE_BUFFER_SIZE;
			if (m_surface_readings_count < SURFACE_BUFFER_SIZE) m_surface_readings_count++;
		}

		if (m_surface_readings_count >= SURFACE_BUFFER_SIZE / 2) {
			uint32_t sum = 0;
			for (int i = 0; i < m_surface_readings_count; i++)
				sum += m_surface_readings[i];
			uint16_t avg = (uint16_t)(sum / m_surface_readings_count);

			if (should_recalibrate()) {
				// B1: clamp to AIR_BASELINE_FLOOR. Without this, dry-electrode
				// surface readings (~0) can collapse air baseline to 0, breaking
				// threshold computation and trapping the system at false-surface.
				[[maybe_unused]] uint16_t old = m_calib.threshold_air;
				uint16_t new_air = avg;
				if (new_air < AIR_BASELINE_FLOOR) new_air = AIR_BASELINE_FLOOR;
				m_calib.threshold_air = new_air;
				update_dynamic_threshold();
				m_calib.last_calibration_time = PMU::get_timestamp_ms() / 1000;
				m_calib.crc =
				    crc16_compute((const uint8_t *)&m_calib, offsetof(SWSAnalogService::CalibrationData, crc), nullptr);
				m_surface_readings_count = 0;
				m_surface_readings_idx = 0;
				DEBUG_TRACE("SWSAnalog: Air recalib %u -> %u%s", old, m_calib.threshold_air,
				            (avg < AIR_BASELINE_FLOOR) ? " (floored)" : "");
				adjust_sample_delay();
			} else if (avg > (uint16_t)(m_calib.threshold_air * SURFACE_ADAPT_THRESHOLD)
			           && avg < m_calib.threshold_current) {
				// B3: when air baseline is at/below floor, the SURFACE_ADAPT_THRESHOLD
				// gate becomes 0 and any noise sample fires this branch — rounding
				// back to 0. Force air to AIR_BASELINE_FLOOR first to break the loop.
				[[maybe_unused]] uint16_t old = m_calib.threshold_air;
				uint16_t new_air;
				if (m_calib.threshold_air < AIR_BASELINE_FLOOR) {
					new_air = AIR_BASELINE_FLOOR;
				} else {
					new_air = (uint16_t)(m_calib.threshold_air * 0.9f + avg * 0.1f);
					if (new_air < AIR_BASELINE_FLOOR) new_air = AIR_BASELINE_FLOOR;
				}
				m_calib.threshold_air = new_air;
				update_dynamic_threshold();
				m_calib.crc =
				    crc16_compute((const uint8_t *)&m_calib, offsetof(SWSAnalogService::CalibrationData, crc), nullptr);
				DEBUG_TRACE("SWSAnalog: Adaptive air UP %u -> %u", old, m_calib.threshold_air);
				adjust_sample_delay();
			} else if (avg < (uint16_t)(m_calib.threshold_air * 0.70f)) {
				// Downward adaptation: air was too high (wet electrode calibration)
				//
				// Guard against runaway drift toward zero:
				// 1. Block if avg itself is implausibly low (< 2× floor) — likely
				//    ADC noise or uncharged RC circuit, not real air readings.
				// 2. Block if air baseline is already close to actual readings.
				bool avg_too_low = (avg < AIR_BASELINE_FLOOR * 2);
				bool air_already_low = (m_calib.threshold_air < avg * 2);
				if (avg_too_low || air_already_low) {
					DEBUG_TRACE("SWSAnalog: Adaptive air DOWN blocked (avg=%u air=%u floor=%u)", avg,
					            m_calib.threshold_air, AIR_BASELINE_FLOOR);
				} else {
					[[maybe_unused]] uint16_t old = m_calib.threshold_air;
					uint16_t new_air = (uint16_t)(m_calib.threshold_air * 0.80f + avg * 0.20f);
					if (new_air < AIR_BASELINE_FLOOR) new_air = AIR_BASELINE_FLOOR;
					m_calib.threshold_air = new_air;
					update_dynamic_threshold();
					m_calib.crc = crc16_compute((const uint8_t *)&m_calib,
					                            offsetof(SWSAnalogService::CalibrationData, crc), nullptr);
					m_surface_readings_count = 0;
					m_surface_readings_idx = 0;
					DEBUG_TRACE("SWSAnalog: Adaptive air DOWN %u -> %u", old, m_calib.threshold_air);
					adjust_sample_delay();
				}
			}
		}
	} else if (m_current_state) {
		m_surface_readings_count = 0;
		m_surface_readings_idx = 0;
	}

	// === 6b. STUCK-STATE RECOVERY (proactive) ===
	// If air baseline has collapsed below floor while we believe we're at surface,
	// the device is in the dry-electrode death spiral. Force a clean recalibration
	// from current ADC readings before the dive-timeout escalation (≥6h) is needed.
	// Only triggers if the safety-net fixes (B1/B2/B3) somehow leak — defense in depth.
	if (!m_current_state && m_calib.threshold_air < AIR_BASELINE_FLOOR) {
		m_air_collapse_count++;
		if (m_air_collapse_count >= AIR_COLLAPSE_RECOVERY_SAMPLES) {
			inc_diag(m_diag.stuck_recovery_count);
			uint16_t old_air = m_calib.threshold_air;
			uint16_t old_water = m_calib.threshold_water;
			uint16_t old_peak = m_observed_peak_adc;

			// Re-seed air from filtered_value (current air reading), floored.
			uint16_t recovered_air = (filtered_value >= AIR_BASELINE_FLOOR) ? filtered_value : AIR_BASELINE_FLOOR;
			m_calib.threshold_air = recovered_air;
			// Ensure water keeps a sane gap above air; never lower it.
			if (m_calib.threshold_water <= recovered_air * MIN_WATER_AIR_RATIO) {
				m_calib.threshold_water = recovered_air * MIN_WATER_AIR_RATIO;
				if (m_calib.threshold_water > ADC_INVALID_MAX) m_calib.threshold_water = ADC_INVALID_MAX;
			}
			// Reset peak — it has decayed below water and is corrupting threshold cap.
			m_observed_peak_adc = 0;
			m_consecutive_spike_rejects = 0;

			update_dynamic_threshold();
			m_calib.last_calibration_time = PMU::get_timestamp_ms() / 1000;
			m_calib.crc =
			    crc16_compute((const uint8_t *)&m_calib, offsetof(SWSAnalogService::CalibrationData, crc), nullptr);
			m_observed_peak_crc =
			    crc16_compute((const uint8_t *)&m_observed_peak_adc, sizeof(m_observed_peak_adc), nullptr);
			// Debounced: a stuck-low electrode (raw≈0 = valid surface) triggers
			// this recovery every ~50 s and would otherwise burn ~1700 flash
			// writes/day, exhausting the 100k-cycle endurance in ~60 days.
			save_calibration_to_flash_debounced();

			m_surface_readings_count = 0;
			m_surface_readings_idx = 0;
			m_air_collapse_count = 0;
			DEBUG_WARN("SWSAnalog: Stuck recovery — air collapsed (%u) | air %u->%u water %u->%u peak %u->0",
			           AIR_COLLAPSE_RECOVERY_SAMPLES, old_air, m_calib.threshold_air, old_water,
			           m_calib.threshold_water, old_peak);
		}
	} else {
		m_air_collapse_count = 0;
	}

	// === 7. STATE DETERMINATION ===
	bool new_state = m_current_state;
	uint16_t threshold_high = m_calib.threshold_current + m_calib.hysteresis_value;
	uint16_t threshold_low = (m_calib.hysteresis_value >= m_calib.threshold_current)
	                             ? 0
	                             : m_calib.threshold_current - m_calib.hysteresis_value;
	// Prevent underflow: ensure threshold_low stays above air baseline
	if (threshold_low <= m_calib.threshold_air && m_calib.threshold_current > m_calib.threshold_air) {
		threshold_low = m_calib.threshold_air + 1;
	}

	// Basic threshold detection.
	//
	// Asymmetric filtering for latency:
	//  - UW entry (going into water) uses `filtered_value` (MA2) for noise
	//    rejection. False UW detection is costly (suppresses surface-mode
	//    services, drains transmit budget).
	//  - Surface entry (going out of water) uses `raw_value` to shave the
	//    1-sample MA2 lag. False surface here is bounded by:
	//      • DEFAULT_MIN_DRY_SAMPLES counter (1 sample minimum)
	//      • Hysteresis (threshold_low = threshold_current - hyst)
	//      • Downstream m_surface_lockout protecting against flapping.
	//    On a UNP03=10s configuration this saves up to one full sampling
	//    period (~10s) of detection latency in the slow-exit case where L1
	//    is gated by the proximity guard.
	if (filtered_value > threshold_high) {
		new_state = true;
		m_consecutive_samples = 0;

		// Don't update water baseline during surface lockout (readings are surface, not water)
		if (m_surface_lockout_remaining == 0) {
			calibrate_water_baseline(filtered_value);
		}
	} else if (raw_value < threshold_low) {
		if (m_current_state) {
			m_consecutive_samples++;
			if (m_consecutive_samples >= DEFAULT_MIN_DRY_SAMPLES) new_state = false;
		} else {
			new_state = false;
			m_consecutive_samples = 0;
		}
	} else {
		// Hysteresis zone — maintain current state
		m_consecutive_samples = 0;
	}

	// === 7b. UPDATE OBSERVED PEAK ADC ===
	// Placed after calibrate_water_baseline() so water_is_estimated (peak==0)
	// is correctly evaluated on the first underwater samples.
	//
	// EC-3: Anti-spike filter + slow decay + coherence guard.
	// - Reject isolated spikes: new peak must be within 120% of current peak (once established)
	// - Slow EMA decay (0.999) — only when raw is plausibly water (B5)
	// - Coherence guard: reset peak if it exceeds water baseline × 3
	{
		bool peak_updated = false;

		if (raw_value > m_observed_peak_adc) {
			// B4: "first_water_contact" must compare to a stable water reference,
			// not threshold_current (which collapses if air baseline drifts).
			// A peak below half the water baseline is stale (e.g. decayed during
			// a long surface period) — accept new readings unconditionally to
			// re-converge instead of trapping real water samples as "spikes".
			uint16_t stale_ref =
			    (m_calib.threshold_water > 0) ? (uint16_t)(m_calib.threshold_water / 2) : m_calib.threshold_current;
			bool first_water_contact = (m_observed_peak_adc < stale_ref);
			if (m_observed_peak_adc == 0 || first_water_contact || raw_value <= (uint32_t)m_observed_peak_adc * 6 / 5) {
				m_observed_peak_adc = raw_value;
				peak_updated = true;
				m_consecutive_spike_rejects = 0;
			} else {
				m_consecutive_spike_rejects++;
				if (m_consecutive_spike_rejects >= 10) {
					inc_diag(m_diag.spike_reject_count);
					// Peak is stuck far below actual readings — force reset
					DEBUG_WARN("SWSAnalog: Peak stuck (10 consecutive rejects) raw=%u >> peak=%u, resetting peak",
					           raw_value, m_observed_peak_adc);
					m_observed_peak_adc = raw_value;
					m_calib.threshold_water = raw_value;
					update_dynamic_threshold();
					m_calib.crc = crc16_compute((const uint8_t *)&m_calib,
					                            offsetof(SWSAnalogService::CalibrationData, crc), nullptr);
					save_calibration_to_flash();
					peak_updated = true;
					m_consecutive_spike_rejects = 0;
				} else {
					DEBUG_WARN("SWSAnalog: Peak spike rejected raw=%u >> peak=%u", raw_value, m_observed_peak_adc);
				}
			}
		} else if (m_observed_peak_adc > 0) {
			// B5: only decay when raw is plausibly an underwater reading.
			// At surface raw≈0; decaying peak toward 0 each surface sample makes
			// peak collapse below water baseline over long surface periods, which
			// then corrupts threshold_high via update_dynamic_threshold's cap.
			// Floor the decay target at water_baseline so peak can't drop into
			// surface-noise territory.
			bool raw_is_water = (m_calib.threshold_water > 0 && raw_value > (uint16_t)(m_calib.threshold_water / 2));
			if (raw_is_water) {
				uint16_t decayed = (uint16_t)(m_observed_peak_adc * 0.999f + raw_value * 0.001f);
				if (decayed < m_calib.threshold_water) decayed = m_calib.threshold_water;
				if (decayed != m_observed_peak_adc) {
					m_observed_peak_adc = decayed;
					peak_updated = true;
				}
			}
		}

		// Coherence guard: peak should not grossly exceed water baseline. The
		// ratio is loose (×5) because at first water contact peak accepts any
		// value (B4) and water grows on a slow EMA — they take a few samples
		// to converge. Tightening to ×3 fired spuriously and pinned peak to
		// a not-yet-converged water, blocking detection.
		if (m_observed_peak_adc > 0 && m_calib.threshold_water > 0
		    && m_observed_peak_adc > (uint32_t)m_calib.threshold_water * 5) {
			inc_diag(m_diag.peak_incoherent_count);
			DEBUG_WARN("SWSAnalog: Peak incoherent peak=%u > water×5=%u, resetting", m_observed_peak_adc,
			           m_calib.threshold_water * 5);
			m_observed_peak_adc = m_calib.threshold_water;
			peak_updated = true;
		}

		if (peak_updated) {
			m_observed_peak_crc =
			    crc16_compute((const uint8_t *)&m_observed_peak_adc, sizeof(m_observed_peak_adc), nullptr);
		}
	}

	// Apply multi-level surface override
	if (surface_level > 0 && m_current_state && new_state) {
		new_state = false;
		m_consecutive_samples = 0;

		// Recalibrate air: gentle EMA toward the raw value (the fresh air-level
		// reading that triggered the L-override, not the MA2-filtered value
		// which still carries water-level lag from the previous sample). Using
		// filtered_value caused air to drift several thousand ADC counts after
		// a few dive/surface cycles, eventually saturating at water * 0.7.
		// Hard cap at AIR_RECALIB_MAX_RATIO of water keeps threshold_high below
		// actual underwater readings.
		[[maybe_unused]] uint16_t old_air = m_calib.threshold_air;
		uint16_t new_air =
		    (uint16_t)(m_calib.threshold_air * (1.0f - AIR_RECALIB_EMA_WEIGHT) + raw_value * AIR_RECALIB_EMA_WEIGHT);
		uint16_t hard_cap = (uint16_t)(m_calib.threshold_water * AIR_RECALIB_MAX_RATIO);
		if (new_air > hard_cap) new_air = hard_cap;
		if (new_air < AIR_BASELINE_FLOOR) new_air = AIR_BASELINE_FLOOR;
		if (new_air != m_calib.threshold_air && new_air < m_calib.threshold_water) {
			m_calib.threshold_air = new_air;
			update_dynamic_threshold();
			m_calib.crc =
			    crc16_compute((const uint8_t *)&m_calib, offsetof(SWSAnalogService::CalibrationData, crc), nullptr);
			DEBUG_TRACE("SWSAnalog: SURFACE L%u | air recalib %u -> %u | thresh=%u", surface_level, old_air,
			            m_calib.threshold_air, m_calib.threshold_current);

			adjust_sample_delay();
		}

		// EC-2: respect user-configured min surface time. UW_MIN_SURFACE_TIME=0
		// explicitly disables the post-L-override lockout (test scenarios and
		// applications that want immediate re-dive detection). The hysteresis
		// gap (threshold_low) already handles MA2 lag at the transition, so a
		// lockout is not strictly necessary for stability. Default config is 5s.
		m_surface_lockout_remaining = m_min_surface_time_sec;

		DEBUG_TRACE("SWSAnalog: SURFACE L%u | raw=%u filt=%u ma3=%u air=%u water=%u lockout=%us", surface_level,
		            raw_value, filtered_value, current_ma3, m_calib.threshold_air, m_calib.threshold_water,
		            m_surface_lockout_remaining);
	}

	// === 8. MAX DIVE TIMEOUT — escalating response ===
	// First timeouts: recalibrate water baseline (legitimate long dives).
	// After MAX_CONSECUTIVE_DIVE_TIMEOUTS without surface: force surface + lockout
	// and clear stuck-state corruption so the next dive can be detected.
	if (timeout_override) {
		inc_diag(m_diag.dive_timeout_count);
		m_consecutive_dive_timeouts++;
		uint16_t old_water = m_calib.threshold_water;

		// Always recalibrate water baseline from current reading
		if (is_value_valid(filtered_value) && filtered_value > m_calib.threshold_air * 2) {
			m_calib.threshold_water = filtered_value;
			update_dynamic_threshold();
			m_calib.crc =
			    crc16_compute((const uint8_t *)&m_calib, offsetof(SWSAnalogService::CalibrationData, crc), nullptr);
			save_calibration_to_flash();
		}

		if (m_consecutive_dive_timeouts >= MAX_CONSECUTIVE_DIVE_TIMEOUTS) {
			inc_diag(m_diag.force_surface_count);
			// B6: also clear corruptable state — peak may have decayed/spiked,
			// spike-reject counter may be mid-cycle, surface-readings buffer
			// may have stale entries from before the dive. Without this, the
			// forced surface succeeds but the next real dive is silently
			// blocked by a stuck anti-spike peak.
			new_state = false;
			m_surface_lockout_remaining = SURFACE_LOCKOUT_DURATION_SEC;
			m_consecutive_dive_timeouts = 0;
			// Re-seed peak from the just-recalibrated water baseline so
			// update_dynamic_threshold's cap doesn't collapse threshold_high.
			m_observed_peak_adc = m_calib.threshold_water;
			m_consecutive_spike_rejects = 0;
			m_observed_peak_crc =
			    crc16_compute((const uint8_t *)&m_observed_peak_adc, sizeof(m_observed_peak_adc), nullptr);
			m_surface_readings_count = 0;
			m_surface_readings_idx = 0;
			update_dynamic_threshold();
			m_calib.crc =
			    crc16_compute((const uint8_t *)&m_calib, offsetof(SWSAnalogService::CalibrationData, crc), nullptr);
			save_calibration_to_flash();
			DEBUG_WARN("SWSAnalog: Dive timeout escalation — forcing surface | water %u -> %u | air=%u | peak reset",
			           old_water, m_calib.threshold_water, m_calib.threshold_air);
		} else {
			// Reset timer for next interval
			m_last_state_change_time = PMU::get_timestamp_ms() / 1000;
			DEBUG_WARN("SWSAnalog: Dive timeout %u/%u — recalib | water %u -> %u | air=%u", m_consecutive_dive_timeouts,
			           MAX_CONSECUTIVE_DIVE_TIMEOUTS, old_water, m_calib.threshold_water, m_calib.threshold_air);
		}
	}

	// === 8b. PER-SAMPLE TIMING LOG (2026-05 investigation) ===
	// Records each sample with full context: timestamp, scheduler-cadence delta,
	// raw + filtered ADC, all thresholds, decided new_state and surface_level (L1-L5).
	// Cost: 1 DEBUG_INFO (= 1 LittleFS commit) per sample, so ~50-300 ms added per
	// sample. Disabled by default — flip SWS_TIMING_LOG_ENABLE in section 0 above
	// to 1 to re-enable for bench / field debugging.
#if SWS_TIMING_LOG_ENABLE
	DEBUG_INFO("[SWS-T t=%lu d=%u] raw=%u filt=%u thr=%u air=%u water=%u state=%s L=%u peak=%u",
	           static_cast<unsigned long>(sample_t0_ms), sample_delta_ms, raw_value, filtered_value,
	           m_calib.threshold_current, m_calib.threshold_air, m_calib.threshold_water, new_state ? "UW" : "SURF",
	           surface_level, m_observed_peak_adc);
#endif

	// === 9. SURFACE LOCKOUT (time-based, not sample-count-based) ===
	// Lockout duration is in seconds; compared against actual time in surface state.
	// !m_current_state guards against premature clear on the L-override call
	// (where m_time_in_current_state still reflects the old UW state).
	if (m_surface_lockout_remaining > 0) {
		if (!m_current_state && m_time_in_current_state >= m_surface_lockout_remaining) {
			m_surface_lockout_remaining = 0;
		} else if (new_state) {
			new_state = false;
		}
	}

	// === 10. STATE CHANGE ===
	if (new_state != m_current_state) {
		m_last_state_change_time = PMU::get_timestamp_ms() / 1000;
		m_time_in_current_state = 0;
		m_consecutive_samples = 0;
		m_peak_adc_since_underwater = 0;
		m_recent_peak = 0;
		m_consecutive_dive_timeouts = 0;  // Reset escalation on any state change
		m_last_heartbeat_warn_time = 0;   // R-CODE-04: re-arm WARN for new state period

		// Reset fast drop tracking
		m_consecutive_raw_drops = 0;
		m_drop_reference = 0;
		m_l4_consecutive_below = 0;

		// Reset trend tracking
		m_trend_buffer_count = 0;
		m_trend_buffer_idx = 0;
		m_ma3_trend_count = 0;
		m_ma3_trend_start = 0;
		m_prev_ma3 = 0;

#if SWS_TIMING_LOG_ENABLE
		DEBUG_INFO("[SWS-CHG t=%lu d=%u] %s -> %s | L=%u raw=%u filt=%u thresh=%u air=%u water=%u",
		           static_cast<unsigned long>(sample_t0_ms), sample_delta_ms, m_current_state ? "UW" : "SURF",
		           new_state ? "UW" : "SURF", surface_level, raw_value, filtered_value, m_calib.threshold_current,
		           m_calib.threshold_air, m_calib.threshold_water);
#else
		DEBUG_TRACE("SWSAnalog: %s -> %s | raw=%u filt=%u thresh=%u air=%u water=%u", m_current_state ? "UW" : "SURF",
		            new_state ? "UW" : "SURF", raw_value, filtered_value, m_calib.threshold_current,
		            m_calib.threshold_air, m_calib.threshold_water);
#endif

		// Persist calibration to flash on state transitions — debounced to reduce flash wear
		save_calibration_to_flash_debounced();

		// Test mode: override LED to show SWS state.
		// Convention: BLUE = underwater, GREEN = surface (matches ledsm).
		// The ledsm dispatcher is suppressed in test mode (see gentracker.cpp)
		// so this set is not overwritten by LEDSurfaceDetected/LEDDiveDetected.
		if (m_test_mode && status_led) {
			if (new_state)
				status_led->set(RGBLedColor::BLUE);  // UNDERWATER
			else
				status_led->set(RGBLedColor::GREEN);  // SURFACE
		}
	}

	// === 11. STATUS PUSH ===
	m_status.threshold_air = m_calib.threshold_air;
	m_status.threshold_water = m_calib.threshold_water;
	m_status.threshold_current = m_calib.threshold_current;
	m_status.hysteresis = m_calib.hysteresis_value;
	m_status.last_raw_adc = raw_value;
	m_status.last_filtered_adc = filtered_value;
	m_status.is_calibrated = m_calib.is_calibrated;
	m_status.is_underwater = new_state;
	m_status.time_in_state_sec = (uint32_t)m_time_in_current_state;
	m_status.surface_level = surface_level;
	m_status.contrast_x10 = m_contrast_x10;
	m_status.observed_peak = m_observed_peak_adc;
	m_status.sample_delay_us = m_sample_delay_us;

	if (m_test_mode && m_status_notify) {
		m_status_notify(m_status);
	}

#if ENABLE_SWS_LOG
	// === 12. PERSISTENT LOG ===
	if (m_sws_logger) {
		SWSLogEntry sws_entry = {};
		service_set_log_header_time(sws_entry.header, service_current_time());
		sws_entry.header.log_type = LOG_UNDERWATER;
		sws_entry.header.payload_size = sizeof(SWSLogEntry) - sizeof(LogHeader);
		sws_entry.raw_adc = raw_value;
		sws_entry.filtered_adc = filtered_value;
		sws_entry.threshold = m_calib.threshold_current;
		sws_entry.hysteresis = m_calib.hysteresis_value;
		sws_entry.air = m_calib.threshold_air;
		sws_entry.water = m_calib.threshold_water;
		sws_entry.calibrated = m_calib.is_calibrated ? 1 : 0;
		sws_entry.underwater = new_state ? 1 : 0;
		sws_entry.time_in_state = (uint16_t)m_time_in_current_state;
		sws_entry.surface_level = surface_level;
		sws_entry.contrast_x10 = m_contrast_x10;
		sws_entry.observed_peak = m_observed_peak_adc;
		sws_entry.sample_delay_us = (uint16_t)m_sample_delay_us;
		m_sws_logger->write(&sws_entry);
	}
#endif

	// === 12b. TEST MODE AUTO-STOP ===
	// Battery-drain safety net: SWSTST,1 without a follow-up SWSTST,0 would
	// otherwise sample + push LED indefinitely. Skip while guided calibration
	// is running (it manages its own 5-min timeout and clears m_test_mode at
	// completion). Setting m_test_mode=false is sufficient — the next
	// scheduler cycle re-evaluates service_is_enabled() (= UNDERWATER_EN).
	// We do not call stop_test_mode() here because Service::stop() would
	// deschedule us mid service_initiate().
	if (m_test_mode && m_test_timeout_ms > 0 && m_calib_phase == CalibPhase::IDLE) {
		uint64_t elapsed = PMU::get_timestamp_ms() - m_test_mode_start_time;
		if (elapsed >= m_test_timeout_ms) {
			DEBUG_WARN("SWSAnalog: Test mode auto-stop after %llu ms (timeout %llu ms)", (unsigned long long)elapsed,
			           (unsigned long long)m_test_timeout_ms);
			m_test_mode = false;
			if (m_on_test_stop) m_on_test_stop();
		}
	}

	// === 13. GUIDED CALIBRATION STATE MACHINE ===
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

	return new_state;
}
