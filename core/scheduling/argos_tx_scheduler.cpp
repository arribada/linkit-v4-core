/**
 * @file argos_tx_scheduler.cpp
 * @brief Argos TX scheduler — duty cycle, legacy, periodic scheduling with jitter.
 */

#include "argos_tx_scheduler.hpp"
#include "debug.hpp"
#include "rtc.hpp"

extern "C" {
	#include "previpass.h"
}

extern RTC *rtc;

/// @brief Constructor — reset all optional state, seed RNG.
ArgosTxScheduler::ArgosTxScheduler() :
		m_rand(std::mt19937()) {
	m_last_schedule_abs.reset();
	m_curr_schedule_abs.reset();
	m_earliest_schedule.reset();
	m_location.reset();
}

/// @brief Reset scheduler state and re-seed RNG.
/// @param seed  RNG seed (typically Argos device ID for reproducible jitter).
void ArgosTxScheduler::reset(unsigned int seed) {
	m_earliest_schedule.reset();
	m_last_schedule_abs.reset();
	m_location.reset();
	m_rand.seed(seed);
}

/// @brief Check if a given time falls within an active duty cycle hour.
/// @param time_ms     Absolute time in ms.
/// @param duty_cycle  24-bit bitmask (bit 23 = hour 0 UTC, bit 0 = hour 23 UTC).
/// @return true if the hour is active in the duty cycle.
bool ArgosTxScheduler::is_in_duty_cycle(uint64_t time_ms, unsigned int duty_cycle)
{
	// Note that duty cycle is a bit-field comprising 24 bits as follows:
	// 23 22 21 20 19 18 17 16 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00  bit
	// 0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 21 21 22 23  hour (UTC)
	uint64_t msec_of_day = (time_ms % (SECONDS_PER_DAY * MSECS_PER_SECOND));
	unsigned int hour_of_day = msec_of_day / (SECONDS_PER_HOUR * MSECS_PER_SECOND);
	return (duty_cycle & (0x800000 >> hour_of_day));
}

/// @brief Generate random TX jitter in [min, max] ms.
/// @param jitter_en  If false, returns 0 (no jitter).
/// @param min        Minimum jitter in ms (default -5000).
/// @param max        Maximum jitter in ms (default +5000).
/// @return Jitter value in ms, or 0 if disabled.
int ArgosTxScheduler::compute_random_jitter(bool jitter_en, int min, int max) {
	if (jitter_en) {
		std::uniform_int_distribution<int> dist(min, max);
		int jitter = dist(m_rand);
		DEBUG_TRACE("ArgosTxScheduler::compute_random_jitter: jitter=%d", jitter);
		return jitter;
	} else {
		return 0;
	}
}

/// @brief Compute next periodic TX time within duty cycle, advancing by period + jitter.
/// @param period_ms    TX interval in ms.
/// @param jitter_en    Enable random jitter on each period.
/// @param duty_cycle   24-bit hourly bitmask (0xFFFFFF = always on).
/// @param now_ms       Current time in ms.
/// @throws ErrorCode::RESOURCE_NOT_AVAILABLE if no slot found in 24h.
void ArgosTxScheduler::schedule_periodic(unsigned int period_ms, bool jitter_en, unsigned int duty_cycle, uint64_t now_ms) {

	uint64_t start_time;

	// Sealed-device guard (audit 2026-07): the 24 h search loop below advances
	// by period_ms — with 0 it never terminates AND never kicks the WDT, so a
	// corrupted stored TR_NOM (DTE clamps writes to >=30 s but LittleFS does
	// not checksum file DATA) would spin until the WDT resets, reboot to the
	// same value, and after 3 such resets trigger a factory reset. Fail the
	// schedule cleanly instead — callers already treat this exception as
	// INVALID_SCHEDULE and re-arm on the next GPS entry. Zero effect on any
	// valid period (>=30 s): the loop is never entered with period_ms==0.
	if (period_ms == 0) {
		DEBUG_ERROR("ArgosTxScheduler::schedule_periodic: period_ms=0 (corrupt config?) — TX schedule aborted");
		m_curr_schedule_abs.reset();
		throw ErrorCode::RESOURCE_NOT_AVAILABLE;
	}

	DEBUG_TRACE("ArgosTxScheduler::schedule_periodic: now=%llu last=%llu tr=%u jitter=%u", now_ms,
			m_last_schedule_abs.has_value() ? m_last_schedule_abs.value() : 0,
			period_ms, jitter_en);

	// Handle the case where earliest TX time has been set
	while (m_earliest_schedule.has_value())
	{
		DEBUG_TRACE("ArgosTxScheduler::schedule_periodic: earliest TX constraint: earliest=%llu now=%llu", m_earliest_schedule.value(), now_ms);

		// If earliest TX is later than last known schedule
		if (m_earliest_schedule.value() >= now_ms) {
			// If we already had a schedule that is later then use that instead
			start_time = m_earliest_schedule.value();
			if (m_last_schedule_abs.has_value() &&
				start_time < m_last_schedule_abs.value())
				start_time = m_last_schedule_abs.value();

			// Check if start_time is in the duty cycle
			if (is_in_duty_cycle(start_time, duty_cycle)) {
				m_curr_schedule_abs = start_time;
				return;
			} else {
				// Not in duty cycle, so allow a new schedule to be computed
				break;
			}
		} else {
			// Earliest TX has elapsed so reset the last schedule to now and allow new schedule
			// to be computed based on duty cycle
			m_earliest_schedule.reset();
		}
		break;
	}

	// A new schedule is required

	// Set schedule based on last TR_NOM point (if there is one)
	if (!m_last_schedule_abs.has_value()) {
		// Use now as the initial TR_NOM -- we don't allow
		// a -ve jitter amount in this case to avoid a potential -ve overflow
		start_time = now_ms + compute_random_jitter(jitter_en, 0);
	}
	else
	{
		// Advance by TR_NOM + TX jitter if we have a previous TR_NOM
		// It should be safe to allow a -ve jitter because TR_NOM is always larger
		// than the jitter amount
		start_time = m_last_schedule_abs.value() + period_ms + compute_random_jitter(jitter_en);

		// Since we only project 24 hours forwards make sure that the start_time is within
		// 24 hours of the current time
		if ((start_time + (MSECS_PER_SECOND * SECONDS_PER_DAY)) < now_ms)
			start_time = now_ms;
	}

	DEBUG_TRACE("ArgosTxScheduler::schedule_periodic: starting @ %llu", start_time);

	// We iterate forwards from the candidate start_time until we find a TR_NOM that
	// falls inside a permitted hour of transmission.  The maximum span we search is 24 hours.
	uint64_t elapsed_time = 0;
	while (elapsed_time <= (MSECS_PER_SECOND * SECONDS_PER_DAY)) {
		if (is_in_duty_cycle(start_time, duty_cycle) && start_time >= now_ms) {
			DEBUG_INFO("ArgosTxScheduler::schedule_periodic: TX scheduled @ %llu (in %llu ms)", start_time, start_time - now_ms);
			m_curr_schedule_abs = start_time;
			return;
		} else {
			start_time += period_ms;
			elapsed_time += period_ms;
		}
	}

	DEBUG_ERROR("ArgosTxScheduler::schedule_periodic: no TX slot found in 24h search (duty_cycle=0x%06X period=%u ms) — TX schedule aborted", duty_cycle, period_ms);
	m_curr_schedule_abs.reset();
	throw ErrorCode::RESOURCE_NOT_AVAILABLE;
}

/// @brief Epoch of the next satellite pass — READ ONLY.
std::time_t ArgosTxScheduler::next_pass_epoch(ArgosConfig& config, BasePassPredict& pass_predict,
                                              std::time_t now) {
	if (!m_location.has_value() || pass_predict.num_records == 0)
		return 0;

	std::time_t start_time = now;
	std::time_t stop_time = start_time + static_cast<std::time_t>(24 * SECONDS_PER_HOUR);
	struct tm *p_tm = std::gmtime(&start_time);
	struct tm tm_start = *p_tm;
	p_tm = std::gmtime(&stop_time);
	struct tm tm_stop = *p_tm;

	PredictionPassConfiguration_t pp_config = {
		static_cast<float>(m_location.value().latitude),
		static_cast<float>(m_location.value().longitude),
		{ static_cast<uint16_t>(1900 + tm_start.tm_year), static_cast<uint8_t>(tm_start.tm_mon + 1),
		  static_cast<uint8_t>(tm_start.tm_mday), static_cast<uint8_t>(tm_start.tm_hour),
		  static_cast<uint8_t>(tm_start.tm_min), static_cast<uint8_t>(tm_start.tm_sec) },
		{ static_cast<uint16_t>(1900 + tm_stop.tm_year), static_cast<uint8_t>(tm_stop.tm_mon + 1),
		  static_cast<uint8_t>(tm_stop.tm_mday), static_cast<uint8_t>(tm_stop.tm_hour),
		  static_cast<uint8_t>(tm_stop.tm_min), static_cast<uint8_t>(tm_stop.tm_sec) },
		static_cast<float>(config.prepass_min_elevation),
		static_cast<float>(config.prepass_max_elevation),
		static_cast<float>(config.prepass_min_duration) / 60.0f,
		config.prepass_max_passes,
		static_cast<float>(config.prepass_linear_margin) / 60.0f,
		config.prepass_comp_step,
		0, // minCulmination  // TODO : ADD DTE PARAMETER
		0, // position error, // TODO : ADD DTE PARAMETER
		true // includeCurrentPass
	};
	SatelliteNextPassPrediction_t next_pass;
	if (PREVIPASS_compute_next_pass(&pp_config, pass_predict.records,
	                                pass_predict.num_records, &next_pass))
		return static_cast<std::time_t>(next_pass.epoch);
	return 0;
}

/// @brief Schedule next TX using satellite pass prediction (PREVIPASS).
/// @return Delay in ms until next TX, or INVALID_SCHEDULE if no pass found.
unsigned int ArgosTxScheduler::schedule_prepass(ArgosConfig& config, BasePassPredict& pass_predict,
                                                std::time_t now) {
	DEBUG_TRACE("ArgosTxScheduler::schedule_prepass");

	// We must have a previous GPS location to proceed
	if (!m_location.has_value()) {
		DEBUG_WARN("ArgosTxScheduler::schedule_prepass: no known GPS location, cannot predict passes — TX disabled until next GPS fix");
		m_last_schedule_abs.reset();
		return INVALID_SCHEDULE;
	}

	uint64_t now_ms = static_cast<uint64_t>(now) * MSECS_PER_SECOND;
	uint64_t start_time_ms = now_ms;

	// If we were deferred by UW event then recompute using earliest TX as start
	if (m_earliest_schedule.has_value() && m_earliest_schedule.value() > now_ms) {
		DEBUG_TRACE("ArgosTxScheduler::schedule_prepass: deferred start, earliest TX in %.3f s",
		            static_cast<double>(m_earliest_schedule.value() - now_ms) / MSECS_PER_SECOND);
		start_time_ms = m_earliest_schedule.value();
	}

	// Set start and end time of the prepass search — 24 hour window
	std::time_t start_time = start_time_ms / MSECS_PER_SECOND;
	std::time_t stop_time = start_time + static_cast<std::time_t>(24 * SECONDS_PER_HOUR);
	struct tm *p_tm = std::gmtime(&start_time);
	struct tm tm_start = *p_tm;
	p_tm = std::gmtime(&stop_time);
	struct tm tm_stop = *p_tm;

	DEBUG_TRACE("ArgosTxScheduler::schedule_prepass: window start=%llu now=%llu stop=%llu",
	           static_cast<unsigned long long>(start_time), static_cast<unsigned long long>(now),
	           static_cast<unsigned long long>(stop_time));

	PredictionPassConfiguration_t pp_config = {
		static_cast<float>(m_location.value().latitude),
		static_cast<float>(m_location.value().longitude),
		{ static_cast<uint16_t>(1900 + tm_start.tm_year), static_cast<uint8_t>(tm_start.tm_mon + 1),
		  static_cast<uint8_t>(tm_start.tm_mday), static_cast<uint8_t>(tm_start.tm_hour),
		  static_cast<uint8_t>(tm_start.tm_min), static_cast<uint8_t>(tm_start.tm_sec) },
		{ static_cast<uint16_t>(1900 + tm_stop.tm_year), static_cast<uint8_t>(tm_stop.tm_mon + 1),
		  static_cast<uint8_t>(tm_stop.tm_mday), static_cast<uint8_t>(tm_stop.tm_hour),
		  static_cast<uint8_t>(tm_stop.tm_min), static_cast<uint8_t>(tm_stop.tm_sec) },
		static_cast<float>(config.prepass_min_elevation),
		static_cast<float>(config.prepass_max_elevation),
		static_cast<float>(config.prepass_min_duration) / 60.0f,
		config.prepass_max_passes,
		static_cast<float>(config.prepass_linear_margin) / 60.0f,
		config.prepass_comp_step,
		0, // minCulmination  // TODO : ADD DTE PARAMETER
		0, // position error, // TODO : ADD DTE PARAMETER
		true // includeCurrentPass
	};
	SatelliteNextPassPrediction_t next_pass;

	while (PREVIPASS_compute_next_pass_with_status(&pp_config,
		pass_predict.records,
		pass_predict.num_records,
		SAT_DNLK_OFF,
		SAT_UPLK_ON_KINEIS_V1, // TODO : Manage all satellites for LDA2 on legacy RF band
		&next_pass))
		{
		uint64_t schedule = 0;

		// Ensure at least TR_NOM from last TX
		if (m_last_schedule_abs.has_value())
			schedule = std::max(schedule,
			                    m_last_schedule_abs.value() + static_cast<uint64_t>(config.tx_interval_s) * MSECS_PER_SECOND);

		// Advance to at least the prepass epoch
		schedule = std::max(static_cast<uint64_t>(next_pass.epoch) * MSECS_PER_SECOND, schedule);

		// Apply TX jitter
		schedule += compute_random_jitter(config.argos_tx_jitter_en);

		// Ensure schedule is at least start_time and current time
		schedule = std::max(start_time_ms, schedule);
		schedule = std::max(now_ms, schedule);

		// Check we don't transmit past the end of the prepass window
		if ((schedule + ARGOS_TX_MARGIN_MSECS) < (static_cast<uint64_t>(next_pass.epoch) + next_pass.duration) * MSECS_PER_SECOND) {
			DEBUG_INFO("ArgosTxScheduler::schedule_prepass: scheduled in %.3f secs (hex_id=%01x ul=%u)",
			           static_cast<double>(schedule - now_ms) / MSECS_PER_SECOND,
			           static_cast<unsigned>(next_pass.satHexId),
			           static_cast<unsigned>(next_pass.uplinkStatus));
			m_curr_schedule_abs = schedule;
			// 2026-08 — no more overwriting of the modulation here. This line
			// forced LDA2 unconditionally, cancelling the
			// `adaptive ? LDA2 : resolve_non_adaptive_modulation()` that the
			// service had just evaluated a handful of lines above. The
			// modulation fix of 2026-05-25 did correctly handle the
			// service for PASS_PREDICTION — its own comment says so —
			// but nobody had seen that the scheduler overwrote the result
			// just afterwards. Consequence: in pass prediction mode, an
			// LDK + adaptive-disabled configuration still went out in
			// LDA2, the frame was sized for the wrong modulation, and
			// ensure_modulation rewrote the master RCONF on every transmission.
			return static_cast<unsigned int>(schedule - now_ms);
		}

		// Window too late — advance search to next pass
		DEBUG_TRACE("ArgosTxScheduler::schedule_prepass: window too late, advancing");
		start_time = static_cast<std::time_t>(next_pass.epoch) + next_pass.duration;
		p_tm = std::gmtime(&start_time);
		tm_start = *p_tm;
		pp_config.start = { static_cast<uint16_t>(1900 + tm_start.tm_year),
		                     static_cast<uint8_t>(tm_start.tm_mon + 1),
		                     static_cast<uint8_t>(tm_start.tm_mday),
		                     static_cast<uint8_t>(tm_start.tm_hour),
		                     static_cast<uint8_t>(tm_start.tm_min),
		                     static_cast<uint8_t>(tm_start.tm_sec) };
	}

	DEBUG_ERROR("ArgosTxScheduler::schedule_prepass: no satellite pass found in 24h window (stale AOP or elevation config?) — TX disabled until next GPS fix");
	return INVALID_SCHEDULE;
}

/// @brief Schedule next TX in duty cycle mode.
/// @param config  Argos configuration (tx_interval, jitter, duty_cycle bitmask).
/// @param now     Current RTC time.
/// @return Delay in ms until next TX, or INVALID_SCHEDULE.
unsigned int ArgosTxScheduler::schedule_duty_cycle(ArgosConfig& config, std::time_t now) {
	try {
		schedule_periodic((config.tx_interval_s * MSECS_PER_SECOND), config.argos_tx_jitter_en, config.duty_cycle, ((uint64_t)now * MSECS_PER_SECOND));
		return m_curr_schedule_abs.value() - (now * MSECS_PER_SECOND);
	} catch(...) {
		return INVALID_SCHEDULE;
	}
}

/// @brief Schedule next TX in legacy mode (24h duty cycle, every hour active).
/// @param config  Argos configuration (tx_interval, jitter).
/// @param now     Current RTC time.
/// @return Delay in ms until next TX, or INVALID_SCHEDULE.
unsigned int ArgosTxScheduler::schedule_legacy(ArgosConfig& config, std::time_t now) {
	try {
		schedule_periodic((config.tx_interval_s * MSECS_PER_SECOND), config.argos_tx_jitter_en, DUTYCYCLE_24HRS, (now * MSECS_PER_SECOND));
		return m_curr_schedule_abs.value() - (now * MSECS_PER_SECOND);
	} catch(...) {
		return INVALID_SCHEDULE;
	}
}

/// @brief Set earliest allowed TX time (e.g., after SWS dry_time_before_tx).
/// @param earliest  Earliest TX epoch time (seconds).
void ArgosTxScheduler::set_earliest_schedule(std::time_t earliest) {
	DEBUG_TRACE("ArgosTxScheduler::set_earliest_schedule: t=%llu", static_cast<unsigned long long>(earliest));
	m_earliest_schedule = (uint64_t)earliest * MSECS_PER_SECOND;
}

/// @brief Update last known GPS position for prepass computation.
/// @param lon  Longitude in degrees.
/// @param lat  Latitude in degrees.
void ArgosTxScheduler::set_last_location(double lon, double lat) {
	DEBUG_TRACE("ArgosTxScheduler::set_last_location: lon=%f lat=%f", lon, lat);
	m_location = Location(lon, lat);
}

/// @brief Force next TX at absolute time t (used for time sync burst).
/// @param t  TX epoch time (seconds).
void ArgosTxScheduler::schedule_at(std::time_t t) {
	DEBUG_TRACE("ArgosTxScheduler::schedule_at: t=%llu", static_cast<unsigned long long>(t));
	m_curr_schedule_abs = (uint64_t)t * MSECS_PER_SECOND;
}

/// @brief Notify that TX completed — saves current schedule as last for TR_NOM calculation.
void ArgosTxScheduler::notify_tx_complete() {
	DEBUG_TRACE("ArgosTxScheduler::notify_tx_complete: TR_NOM anchor=%llu",
	            m_curr_schedule_abs.has_value() ? m_curr_schedule_abs.value() : 0);
	m_last_schedule_abs = m_curr_schedule_abs;
}
