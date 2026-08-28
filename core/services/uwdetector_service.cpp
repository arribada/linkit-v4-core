/**
 * @file uwdetector_service.cpp
 * @brief Underwater detector — multi-sample state detection with dry/wet hysteresis.
 */

#include "uwdetector_service.hpp"
#include "debug.hpp"
#include "pmu.hpp"

// 2026-05 latency investigation toggle. Enable by setting UW_TIMING_LOG_ENABLE=1
// at build to add per-tick [UW-CHG]/[UW-BCAST] timing logs (with absolute
// timestamps + detector_state() duration + broadcast latency). Disabled by
// default — each log costs one LittleFS commit (~50-300 ms) per emit.
#ifndef UW_TIMING_LOG_ENABLE
#define UW_TIMING_LOG_ENABLE 0
#endif

/// @brief Sample the detector, accumulate dry/wet counts, emit state change on terminal iteration.
void UWDetectorService::service_initiate() {
#if UW_TIMING_LOG_ENABLE
	uint64_t uw_t0_ms = PMU::get_timestamp_ms();
#endif
	bool new_state;
	DEBUG_TRACE("UWDetectorService: m_sample_iteration=%u dry=%u", m_sample_iteration, m_dry_count);

	// Sample the switch
	new_state = detector_state();
#if UW_TIMING_LOG_ENABLE
	uint32_t detector_ms = static_cast<uint32_t>(PMU::get_timestamp_ms() - uw_t0_ms);
#endif
	m_sample_iteration++;

	DEBUG_TRACE("UWDetectorService: state=%u", new_state);

	if (new_state) {
		m_pending_state = true;
	} else {
		if (++m_dry_count >= m_min_dry_samples) {
			DEBUG_TRACE("UWDetectorService: terminate early as dry=%u", m_dry_count);
			m_sample_iteration = m_max_samples;  // Terminate early
			m_pending_state = false;
		}
	}

	if (m_sample_iteration >= m_max_samples) {
		// If we reached the terminal number of iterations then the SWS state must have been
		// set for all sampling iterations -- we treat SWS as definitively set.

		DEBUG_TRACE("UWDetectorService: terminal iterations reached: state=%u", m_pending_state);

		m_sample_iteration = 0;
		m_dry_count = 0;

		if (m_pending_state != m_current_state || m_is_first_time) {
#if UW_TIMING_LOG_ENABLE
			DEBUG_INFO("[UW-CHG t=%lu det=%u ms] state changed: state=%u (sample_iter=%u dry=%u max=%u min_dry=%u)",
			           static_cast<unsigned long>(uw_t0_ms), detector_ms, (unsigned int)m_pending_state,
			           m_sample_iteration, m_dry_count, m_max_samples, m_min_dry_samples);
#else
			DEBUG_INFO("UWDetectorService: state changed: state=%u", (unsigned int)m_pending_state);
#endif
			m_is_first_time = false;
			m_current_state = m_pending_state;
			ServiceEventData event = m_pending_state;
			service_complete(&event);
#if UW_TIMING_LOG_ENABLE
			// Logs the moment broadcast completes — peer service reactions happen
			// synchronously inside service_complete via the data_notification_callback,
			// so this timestamp tells us when peers have finished reacting.
			DEBUG_INFO("[UW-BCAST t=%lu spent=%lu ms] broadcast complete",
			           static_cast<unsigned long>(PMU::get_timestamp_ms()),
			           static_cast<unsigned long>(PMU::get_timestamp_ms() - uw_t0_ms));
#endif
		} else {
			DEBUG_TRACE("UWDetectorService: state unchanged");
			service_complete();
		}
		m_pending_state = false;
	} else {
		service_complete();
	}
}

/// @brief Init: load UW detection params from config (thresholds, sample counts, gaps).
void UWDetectorService::service_init() {
	m_is_first_time = true;
	m_period_underwater_ms =
	    static_cast<unsigned int>(service_read_param<double>(ParamID::SAMPLING_UNDER_FREQ) * 1000.0);
	m_period_surface_ms = static_cast<unsigned int>(service_read_param<double>(ParamID::SAMPLING_SURF_FREQ) * 1000.0);
	m_activation_threshold = service_read_param<double>(ParamID::UNDERWATER_DETECT_THRESH);
	m_sample_gap = service_read_param<unsigned int>(ParamID::UW_SAMPLE_GAP);
	m_enable_sample_delay = service_read_param<unsigned int>(ParamID::UW_PIN_SAMPLE_DELAY_US);
	m_max_samples = service_read_param<unsigned int>(ParamID::UW_MAX_SAMPLES);
	m_min_dry_samples = service_read_param<unsigned int>(ParamID::UW_MIN_DRY_SAMPLES);
	m_sample_iteration = 0;
	m_dry_count = 0;
	m_pending_state = false;
}

/// @brief Terminate: no-op.
void UWDetectorService::service_term() {};

/// @brief Next schedule: sample_gap between iterations, or surface/underwater period.
/// @return Delay in ms until next sample, or 0 for first-time immediate.
unsigned int UWDetectorService::service_next_schedule_in_ms() {
	if (m_sample_iteration)
		return m_sample_gap;
	else
		return m_is_first_time ? 0 : (m_current_state ? m_period_underwater_ms : m_period_surface_ms);
}

/// @brief UW detector must run underwater (it IS the underwater detector).
/// @return Always true.
bool UWDetectorService::service_is_usable_underwater() {
	return true;
}
