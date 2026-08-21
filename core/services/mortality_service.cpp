/**
 * @file mortality_service.cpp
 * @brief Bird mortality detection — daily confidence evaluation from activity, temperature, GPS.
 */

#include "mortality_service.hpp"
#include "sensor.hpp"

extern Scheduler *system_scheduler;

// Sensor port indices (must match axl_sensor_service.hpp / sensor.hpp)
static constexpr unsigned int AXL_PORT_ACTIVITY = 4;
static constexpr unsigned int AXL_PORT_WAKEUP   = 5;

/// @brief Constructor — init state as ALIVE, reset session data.
/// @param logger  Optional persistent logger for mortality state.
MortalityService::MortalityService(Logger *logger)
	: Service(ServiceIdentifier::MORTALITY, "MORTALITY", logger)
{
	memset(&m_state, 0, sizeof(m_state));
	m_state.status = MortalityStatus::ALIVE;
	reset_session_data();
}

/// @brief Clear session-local sensor data (between daily evaluations).
void MortalityService::reset_session_data()
{
	m_has_activity = false;
	m_has_temperature = false;
	m_has_gps = false;
	m_session_activity = 0;
	m_session_body_temp = 0.0;
	m_session_lat = 0.0;
	m_session_lon = 0.0;
	m_session_gps_speed = 0;
}

/// @brief Init: restore persisted state from FsLog, reset session.
void MortalityService::service_init()
{
	reset_session_data();
	// H4: session-scoped flags — reset once per wake, NOT per evaluation
	// (reset_session_data also runs at the end of each evaluate_mortality).
	m_gps_attempted = false;
	m_no_fix_counted = false;
	m_fallback_evaluated = false;

	// Restore persisted state from log. The read is wrapped: FsLog::read opens an
	// LFSFile whose ctor throws on a corrupt/worn/missing chunk, and service_init
	// runs from ServiceManager::startall(), which — unlike stopall() — has NO
	// per-service exception barrier. An escaping throw would abort startup of every
	// later service, propagate to the main-loop catch → ErrorState → boot-fail
	// counter, and on a sealed RSPB tag (soft reset does not cut power) spin a
	// battery-draining reset loop. Fail SAFE to a fresh ALIVE state instead
	// (design priority 1: bounded recovery, never thrash).
	if (get_logger() && get_logger()->num_entries() > 0) {
		try {
			MortalityLogEntry last_entry;
			get_logger()->read(&last_entry, get_logger()->num_entries() - 1);
			m_state = last_entry.info;
			DEBUG_INFO("MortalityService: Restored state: confidence=%u%% days=%u status=%u",
					m_state.confidence, m_state.consecutive_days, (unsigned int)m_state.status);
		} catch (...) {
			memset(&m_state, 0, sizeof(m_state));
			m_state.status = MortalityStatus::ALIVE;
			DEBUG_ERROR("MortalityService: state restore failed (corrupt log) — starting fresh");
		}
	} else {
		memset(&m_state, 0, sizeof(m_state));
		m_state.status = MortalityStatus::ALIVE;
		DEBUG_INFO("MortalityService: No prior state, starting fresh");
	}
}

/// @brief Terminate: clear session-local data. Persistent state (confidence,
/// consecutive_days, status, no-fix streak) is already flushed to FsLog on each
/// evaluate_mortality() / no-fix increment, so there is nothing to persist here.
void MortalityService::service_term()
{
	reset_session_data();
}

/// @brief Enabled if MORTALITY_ENABLE param is set and board supports it.
/// @return true if mortality detection is active.
bool MortalityService::service_is_enabled()
{
#if ENABLE_MORTALITY_SENSOR
	return service_read_param<bool>(ParamID::MORTALITY_ENABLE);
#else
	return false;
#endif
}

/// @brief Schedule daily evaluation (24h period).
/// @return 24 hours in ms.
unsigned int MortalityService::service_next_schedule_in_ms()
{
	// Event-driven only — no periodic scheduling
	return SCHEDULE_DISABLED;
}

/// @brief Run daily mortality evaluation — compute confidence, log, persist.
void MortalityService::service_initiate()
{
	// Should not be called (schedule disabled), but handle gracefully
	service_complete();
}

/// @brief Cancel — no-op (evaluation is instant).
/// @return Always false.
bool MortalityService::service_cancel()
{
	return false;
}

/// @brief Collect sensor data from peer events (AXL activity, thermistor temp, GPS fix).
/// @param event  Peer service event.
void MortalityService::notify_peer_event(ServiceEvent& event)
{
#if ENABLE_MORTALITY_SENSOR
	if (!service_is_enabled())
		return;

	// Collect AXL activity data
	if (event.event_source == ServiceIdentifier::AXL_SENSOR &&
		event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
		auto *sensor_data = std::get_if<ServiceSensorData>(&event.event_data);
		if (sensor_data) {
			m_session_activity = static_cast<uint8_t>(sensor_data->port[AXL_PORT_ACTIVITY]);
			m_has_activity = true;
			DEBUG_TRACE("MortalityService: AXL activity=%u", m_session_activity);
		}
	}

	// Collect thermistor temperature data
	if (event.event_source == ServiceIdentifier::THERMISTOR_SENSOR &&
		event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
		auto *sensor_data = std::get_if<ServiceSensorData>(&event.event_data);
		if (sensor_data) {
			m_session_body_temp = sensor_data->port[0];
			m_has_temperature = true;
			DEBUG_TRACE("MortalityService: body_temp=%.1f", m_session_body_temp);
		}
	}

	// Collect GPS position data
	if (event.event_source == ServiceIdentifier::GNSS_SENSOR &&
		event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
		auto *gps_log = std::get_if<GPSLogEntry>(&event.event_data);
		if (gps_log) {
			m_gps_attempted = true;  // H4: GPS ran this session (fix or not)
			if (gps_log->info.valid) {
				m_session_lat = gps_log->info.lat;
				m_session_lon = gps_log->info.lon;
				m_session_gps_speed = gps_log->info.gSpeed;
				m_has_gps = true;
				DEBUG_TRACE("MortalityService: GPS lat=%.4f lon=%.4f speed=%d mm/s",
						m_session_lat, m_session_lon, m_session_gps_speed);
			} else if (!m_no_fix_counted) {
				// H4: no valid fix this session — extend the no-fix streak and
				// persist it so it accumulates across TPL5111 power cuts. Once
				// the streak reaches MORTALITY_NO_FIX_FALLBACK_SESSIONS the
				// evaluation below runs without a fix (partial stationarity).
				m_no_fix_counted = true;
				// Persist ONLY when the streak actually changes: once it
				// saturates at UINT8_MAX, rewriting the 128-byte MortalityLogEntry
				// to flash on every subsequent no-fix wake is pure flash wear +
				// battery cost for no state change (L25).
				if (m_state.no_fix_sessions < UINT8_MAX) {
					m_state.no_fix_sessions++;
					DEBUG_INFO("MortalityService: no GPS fix this session (no-fix streak=%u)",
							m_state.no_fix_sessions);
					persist_state();
				}
			}
		}
	}

	// Decide whether to evaluate this session:
	//  - Normal: a valid GPS fix + at least one other sensor (evaluates per fix).
	//  - Fallback (H4): after N consecutive no-fix sessions, evaluate on
	//    activity+temperature + partial stationarity (evaluate_mortality awards
	//    MORTALITY_NO_FIX_GPS_SCORE when m_has_gps is false). Guarded to run at
	//    most once per session so it doesn't over-weight the EMA.
	//
	// The fallback gates on BOTH sensors (activity AND temperature), not either:
	// AXL and thermistor arrive as two separate peer events, and evaluate_mortality
	// latches m_fallback_evaluated + resets the session flags, so whichever event
	// fires the evaluation is the only sensor scored. With an OR gate the fallback
	// fired on the first-arriving sensor and could never combine both, capping the
	// session score at 40+0+15=55 (or 0+30+15=45) — below the 80 needed to advance
	// consecutive_days. A dead bird under dense canopy (never fixes GPS) would then
	// reach SUSPECTED but NEVER CONFIRMED, defeating H4's entire purpose. Requiring
	// both sensors lets the fallback score the intended 40+30+15=85 once activity
	// and temperature are collected, regardless of peer-event ordering.
	bool have_other_sensor = (m_has_activity || m_has_temperature);
	if (have_other_sensor && m_has_gps) {
		evaluate_mortality();
	} else if (m_has_activity && m_has_temperature && !m_fallback_evaluated &&
			MORTALITY_NO_FIX_FALLBACK_SESSIONS > 0 &&
			m_state.no_fix_sessions >= MORTALITY_NO_FIX_FALLBACK_SESSIONS) {
		m_fallback_evaluated = true;
		evaluate_mortality();
	}
#else
	(void)event;
#endif
}

/// @brief Check if all 3 sensor inputs (activity, temperature, GPS) have been received.
/// @return true if all inputs are available for evaluation.
bool MortalityService::all_inputs_collected() const
{
	return m_has_activity && m_has_temperature && m_has_gps;
}

/// @brief Monotonic per-day key used to detect a calendar-day boundary between
///        evaluations. Returns a strictly increasing (year, day-of-year) index
///        rather than the bare tm_yday, so that:
///          - a fresh unit (last_eval_epoch == 0) evaluating on Jan 1 UTC is not
///            aliased with the epoch==0 "never evaluated" sentinel, and
///          - the day counter cannot collide across a New-Year boundary on a
///            multi-year sealed deployment (yday resets to 0 every Jan 1).
///        Only equality of consecutive keys matters to the caller, so the exact
///        magnitude is irrelevant; *366 (> max yday) guarantees no cross-year
///        collision. epoch==0 keeps returning 0 as the "never evaluated" sentinel.
unsigned int MortalityService::eval_day_index(std::time_t epoch) const
{
	if (epoch == 0) return 0;
	struct tm *t = gmtime(&epoch);
	return t ? static_cast<unsigned int>((t->tm_year + 1900) * 366 + t->tm_yday) : 0;
}

/// @brief Compute mortality confidence (0-100%) from activity, body temp, GPS stationarity.
void MortalityService::evaluate_mortality()
{
#if ENABLE_MORTALITY_SENSOR
	unsigned int activity_thresh = service_read_param<unsigned int>(ParamID::MORTALITY_ACTIVITY_THRESH);
	double temp_thresh = service_read_param<double>(ParamID::MORTALITY_TEMP_THRESH);
	unsigned int gps_distance_thresh = service_read_param<unsigned int>(ParamID::MORTALITY_GPS_DISTANCE_THRESH);
	unsigned int confirm_days = service_read_param<unsigned int>(ParamID::MORTALITY_CONFIRM_DAYS);

	// --- Score calculation ---
	unsigned int activity_score = 0;
	if (m_has_activity && m_session_activity < activity_thresh) {
		activity_score = 40;
	}

	unsigned int temp_score = 0;
	if (m_has_temperature && m_session_body_temp < temp_thresh) {
		temp_score = 30;
	}

	unsigned int gps_score = 0;
	if (m_has_gps) {
		// Check stationarity: distance from last known position + low speed
		bool has_prior_position = (m_state.last_lat != 0.0 || m_state.last_lon != 0.0);
		if (has_prior_position) {
			double distance_km = haversine_distance(m_state.last_lon, m_state.last_lat,
					m_session_lon, m_session_lat);
			double distance_m = distance_km * 1000.0;
			bool is_stationary = (distance_m < (double)gps_distance_thresh) &&
					(m_session_gps_speed < 100); // < 100 mm/s = 0.36 km/h
			if (is_stationary) {
				gps_score = 30;
			}
		}
		// Update last known position
		m_state.last_lat = m_session_lat;
		m_state.last_lon = m_session_lon;
		// H4: a valid fix breaks the no-fix streak.
		m_state.no_fix_sessions = 0;
	} else {
		// H4 fallback: reached only after MORTALITY_NO_FIX_FALLBACK_SESSIONS
		// consecutive no-fix sessions (see the trigger in notify_peer_event).
		// Treat the sustained inability to fix as WEAK stationarity evidence and
		// award partial credit so activity + temperature can still drive
		// confidence toward CONFIRMED for a bird that never sees the sky.
		gps_score = MORTALITY_NO_FIX_GPS_SCORE;
	}

	unsigned int session_score = activity_score + temp_score + gps_score;

	// --- Exponential moving average ---
	unsigned int old_confidence = m_state.confidence;
	m_state.confidence = static_cast<uint8_t>((old_confidence * 7 + session_score * 3) / 10);
	if (m_state.confidence > 100) m_state.confidence = 100;

	DEBUG_INFO("MortalityService: score=%u (act=%u temp=%u gps=%u) confidence=%u%% (was %u%%)",
			session_score, activity_score, temp_score, gps_score,
			m_state.confidence, old_confidence);

	// --- Day boundary check ---
	std::time_t now = service_current_time();
	unsigned int current_day = eval_day_index(now);
	unsigned int last_day = eval_day_index(static_cast<std::time_t>(m_state.last_eval_epoch));

	if (now > 0 && now <= static_cast<std::time_t>(UINT32_MAX) && current_day != last_day) {
		m_state.last_eval_epoch = static_cast<uint32_t>(now);
		if (m_state.confidence >= 80) {
			if (m_state.consecutive_days < 255)
				m_state.consecutive_days++;
			DEBUG_INFO("MortalityService: consecutive_days++ = %u", m_state.consecutive_days);
		} else {
			if (m_state.consecutive_days > 0)
				m_state.consecutive_days--;
			DEBUG_INFO("MortalityService: consecutive_days-- = %u", m_state.consecutive_days);
		}
	}

	// --- Status determination ---
	if (m_state.consecutive_days >= confirm_days) {
		m_state.status = MortalityStatus::CONFIRMED;
	} else if (m_state.confidence >= 50) {
		m_state.status = MortalityStatus::SUSPECTED;
	} else {
		m_state.status = MortalityStatus::ALIVE;
	}

	// --- Duty cycle adaptation (opt-in, requires EXTERNAL_WAKEUP for BOOT_COUNTER_MODULO) ---
#ifdef EXTERNAL_WAKEUP
	unsigned int duty_modulo = service_read_param<unsigned int>(ParamID::MORTALITY_DUTY_CYCLE_MODULO);

	// BOOT_COUNTER_MODULO must be >= 2 (see boot_count_check_modulo): a modulo of
	// 1 would make the device power down on nearly every wake and silence the tag
	// exactly when a confirmed-dead bird should be beaconing. MTP06's DTE range is
	// 0-100, so a 1 is reachable — clamp it defensively before it reaches the
	// duty-cycle write. 0 keeps its "never adapt" meaning (guarded below).
	if (duty_modulo == 1)
		duty_modulo = 2;

	unsigned int current_modulo = service_read_param<unsigned int>(ParamID::BOOT_COUNTER_MODULO);
	unsigned int original = service_read_param<unsigned int>(ParamID::MORTALITY_ORIGINAL_MODULO);

	if (duty_modulo > 0 && m_state.status == MortalityStatus::CONFIRMED) {
		// Apply (or RE-apply) the dead-bird cadence. Gating on the actual
		// modulo value rather than the status EDGE self-heals the case where
		// a TPL5111 power cut dropped the (RAM-only) modulo write while the
		// CONFIRMED state (FsLog) persisted (M2, 2026-07): the next
		// evaluation sees current_modulo != duty_modulo and re-applies it.
		if (current_modulo != duty_modulo) {
			if (original == 0)
				service_write_param(ParamID::MORTALITY_ORIGINAL_MODULO, current_modulo);
			service_write_param(ParamID::BOOT_COUNTER_MODULO, duty_modulo);
			DEBUG_INFO("MortalityService: CONFIRMED — duty cycle adapted to modulo=%u", duty_modulo);
		}
	} else if (original > 0) {
		// Restore the original cadence whenever we are NOT holding the dead
		// cadence. This covers two exits, and the restore lives OUTSIDE the
		// `duty_modulo > 0` guard on purpose:
		//  - H5 recovery: status walked CONFIRMED -> SUSPECTED -> ALIVE (restore
		//    on ANY transition out of CONFIRMED, not just the atomic ->ALIVE edge,
		//    else the dead cadence sticks once a bird improves through SUSPECTED).
		//  - Feature disabled after adaptation (operator writes MTP06=0): nesting
		//    this under `duty_modulo > 0` previously stranded the tag at the dead
		//    cadence forever — "disable" became a one-way trap.
		service_write_param(ParamID::BOOT_COUNTER_MODULO, original);
		unsigned int zero = 0U;
		service_write_param(ParamID::MORTALITY_ORIGINAL_MODULO, zero);
		DEBUG_INFO("MortalityService: not holding dead cadence — restored duty cycle modulo=%u", original);
	}
#endif

	// --- Update session data for log ---
	if (m_has_activity)
		m_state.last_activity = m_session_activity;
	if (m_has_temperature) {
		// Clamp before the unsigned cast: body temp is in °C and can be sub-zero
		// (cold carcass / winter), which would otherwise wrap to ~65000 in this
		// uint16 log field (L4). Store 0 for negatives — logging only.
		m_state.last_body_temp = (m_session_body_temp > 0.0)
			? static_cast<uint16_t>(m_session_body_temp) : 0;
	}

	// --- Persist to flash ---
	persist_state();

	// Reset session data for next collection
	reset_session_data();

	DEBUG_INFO("MortalityService: status=%u confidence=%u%% days=%u",
			(unsigned int)m_state.status, m_state.confidence, m_state.consecutive_days);
#endif
}

/// @brief Write current mortality state to FsLog for persistence across power cycles.
void MortalityService::persist_state()
{
	MortalityLogEntry entry;
	memset(&entry, 0, sizeof(entry));
	entry.header.log_type = LOG_MORTALITY;
	service_set_log_header_time(entry.header, service_current_time());
	entry.info = m_state;
	service_log(nullptr, &entry);
}
