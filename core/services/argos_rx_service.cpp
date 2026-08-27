/**
 * @file argos_rx_service.cpp
 * @brief Argos RX service + scheduler — downlink AOP reception and pass predict update.
 */

#include "argos_rx_service.hpp"
#include "config_store.hpp"
#include "binascii.hpp"
#include "dte_protocol.hpp"

extern ConfigurationStore *configuration_store;

/// @brief Shortest RX window worth powering the receiver for. A pass already
///        under way when the window opens is reported by PREVIPASS with only
///        its remaining seconds; below this threshold we look for the next one.
static constexpr unsigned int ARGOS_RX_MIN_WINDOW_SECS = 60;

/// @brief Bound on the number of candidate passes examined per scheduling call.
static constexpr unsigned int ARGOS_RX_MAX_PASS_CANDIDATES = 16;

/// @brief Age beyond which a campaign that never completed is committed with
///        whatever it has.
///
///        The commit normally waits for every satellite declared by the
///        constellation status to have an orbit bulletin. That is the right
///        rule, but it assumes the two always agree. If a satellite is listed
///        as operational while its AOP is never broadcast, the campaign waits
///        for a message that will never come and no update is ever written.
///        Past this age the satellites we do have are committed and the others
///        keep their previous bulletin.
static constexpr unsigned int ARGOS_RX_CAMPAIGN_MAX_SECS = 2 * 24 * 3600;

/// @brief Construct Argos RX service with a KineisDevice backend.
/// @param device  KineisDevice for RX operations (SMD/KIM2).
ArgosRxService::ArgosRxService(KineisDevice& device) : Service(ServiceIdentifier::ARGOS_RX, "ARGOSRX"), m_kineis(device) {
}

/// @brief Init: set TCXO warmup, subscribe to KineisDevice events.
void ArgosRxService::service_init() {
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);
	m_kineis.set_tcxo_warmup_time(argos_config.argos_tcxo_warmup_time);
	m_kineis.subscribe(*this);
}

/// @brief Terminate: unsubscribe from KineisDevice events.
void ArgosRxService::service_term() {
	m_kineis.unsubscribe(*this);
}

/// @brief Enabled only in PASS_PREDICTION mode with ARGOS_RX_EN and no cert TX.
/// @return true if RX reception should be scheduled.
bool ArgosRxService::service_is_enabled() {
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);
	return (argos_config.argos_rx_en && argos_config.mode == BaseArgosMode::PASS_PREDICTION && !argos_config.cert_tx_enable);
}

/// @brief Compute next RX window using PREVIPASS.
/// @return Delay in ms until RX start, or SCHEDULE_DISABLED.
unsigned int ArgosRxService::service_next_schedule_in_ms() {
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);
	BasePassPredict& pass_predict = configuration_store->read_pass_predict();
	std::time_t now = service_current_time();
	return m_sched.schedule(argos_config, pass_predict, now, m_timeout, m_mode);
}

/// @brief Start RX — power on receiver in scheduled mode.
void ArgosRxService::service_initiate() {
	DEBUG_INFO("ArgosRxService::service_initiate: starting RX");
	// Opens the acceptance window. The two decoder maps are deliberately NOT
	// cleared here: a campaign may need several passes to be completed, and
	// what was decoded during the previous windows is exactly what we want to
	// keep. They are cleared on commit only.
	m_rx_window_open = true;
	m_kineis.start_receive(m_mode);
	m_cumulative_rx_time = 0;
}

/// @brief Cancel active RX — stop receiver.
/// @return true if RX was stopped.
bool ArgosRxService::service_cancel() {
	m_rx_window_open = false;
	return m_kineis.stop_receive();
}

/// @brief RX timeout — duration of the scheduled RX window.
/// @return Timeout in ms (set by scheduler during scheduling).
unsigned int ArgosRxService::service_next_timeout() {
	return m_timeout;
}

/// @brief Handle peer events — GPS fix updates location, UW surfacing sets earliest schedule.
/// @param e  Peer service event.
void ArgosRxService::notify_peer_event(ServiceEvent& e) {

	if (e.event_source == ServiceIdentifier::GNSS_SENSOR &&
		e.event_type == ServiceEventType::SERVICE_LOG_UPDATED)
	{
		// Update location information if we got a valid fix
		GPSLogEntry& gps = std::get<GPSLogEntry>(e.event_data);
		if (gps.info.valid) {
			DEBUG_TRACE("ArgosRxService::notify_peer_event: updated GPS location");
			m_sched.set_location(gps.info.lon, gps.info.lat);
			if (!service_is_scheduled())
				service_reschedule();
		}
	} else if (e.event_source == ServiceIdentifier::UW_SENSOR && e.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
		if (std::get<bool>(e.event_data) == false) {
			ArgosConfig argos_config;
			configuration_store->get_argos_configuration(argos_config);
			std::time_t earliest_schedule = service_current_time() + argos_config.dry_time_before_tx;
			m_sched.set_earliest_schedule(earliest_schedule);
		}
	}

	Service::notify_peer_event(e);
}

/// @brief Reschedule on surfacing (non-immediate — wait for earliest schedule).
/// @param[out] immediate  Always false for RX (no immediate trigger).
/// @return true (always reschedule on surface).
bool ArgosRxService::service_is_triggered_on_surfaced(bool& immediate) {
	immediate = false;
	return true;
}

/// @brief Downlink packet received — decode AOP and merge into pass predict.
/// @param e  RX packet event with hex-encoded payload.
void ArgosRxService::react(KineisEventRxPacket const& e) {
	// Frames still in flight when the window was closed (the module keeps
	// delivering for a few seconds after AT+DL=0, and the commit has just
	// cleared the maps) would re-seed the decoder with a partial picture that
	// would then leak into the next campaign.
	if (!m_rx_window_open) {
		DEBUG_TRACE("ArgosRxService: late RX packet ignored (window closed)");
		return;
	}

	// Increment RX counter (RAM only — flash deferred to periodic flush / powerdown)
	configuration_store->increment_rx_counter();

	// Attempt to decode the queue of packets. The constellation status
	// tracking follows the CS message set (counter/index/total) so that we can
	// tell a complete constellation picture from a partial one.
	BasePassPredict pass_predict;
	PassPredictCodec::decode(m_orbit_params_map, m_constellation_status_map,
			&m_constellation_status_tracking, e.packet, pass_predict);

	// Check to see if any new AOP records were found
	update_pass_predict(pass_predict);
}

/// @brief Device error — cancel RX and complete service.
void ArgosRxService::react(KineisEventDeviceError const&) {
	DEBUG_TRACE("ArgosRxService::react: KineisEventDeviceError");
	if (service_cancel())
		service_complete();
}

/// @brief Power off — flush cumulative RX time to config store (seconds).
void ArgosRxService::react(KineisEventPowerOff const&) {
	if (m_cumulative_rx_time) {
		DEBUG_INFO("ArgosRxService::react: KineisEventPowerOff: cumulative_rx=%u ms", m_cumulative_rx_time);
		configuration_store->increment_rx_time((m_cumulative_rx_time + 999) / 1000); // Stored in seconds
		// RAM updated — flash deferred to periodic flush / powerdown
		m_cumulative_rx_time = 0;
	}
}

/// @brief RX stopped — accumulate RX-on time for statistics.
/// @param e  Event with rx_time in ms.
void ArgosRxService::react(KineisEventRxStopped const& e) {
	m_cumulative_rx_time += e.rx_time;
	m_rx_window_open = false;
}

/// @brief Merge new AOP records into existing pass predict database and persist to flash.
/// @param new_pass_predict  Decoded AOP records from downlink packet(s).
void ArgosRxService::update_pass_predict(BasePassPredict& new_pass_predict) {

	BasePassPredict existing_pass_predict;
	unsigned int num_updated_records = 0;

	// Read in the existing pass predict database
	existing_pass_predict = configuration_store->read_pass_predict();

	// Iterate over new candidate records
	for (unsigned int i = 0; i < new_pass_predict.num_records; i++) {
		bool operational = new_pass_predict.records[i].downlinkStatus ||
				new_pass_predict.records[i].uplinkStatus;
		bool has_aop = new_pass_predict.records[i].bulletin.year != 0;
		unsigned int j = 0;

		DEBUG_TRACE("ArgosRxService::update_pass_predict: hexid=%02x dl=%u ul=%u aop=%u",
				(unsigned int)new_pass_predict.records[i].satHexId,
				(unsigned int)new_pass_predict.records[i].downlinkStatus,
				(unsigned int)new_pass_predict.records[i].uplinkStatus,
				(unsigned int)has_aop);

		for (; j < existing_pass_predict.num_records; j++) {
			// Check for existing hex ID match
			if (new_pass_predict.records[i].satHexId != existing_pass_predict.records[j].satHexId)
				continue;
			if (operational && has_aop) {
				num_updated_records++;
				existing_pass_predict.records[j] = new_pass_predict.records[i];
			} else if (!operational) {
				// Satellite out of service: keep the orbit params, drop the status
				num_updated_records++;
				existing_pass_predict.records[j].downlinkStatus = new_pass_predict.records[i].downlinkStatus;
				existing_pass_predict.records[j].uplinkStatus = new_pass_predict.records[i].uplinkStatus;
			}
			break;
		}

		// If we reached the end of the existing database then this is a new hex ID, so
		// add it to the end of the existing database
		if (j == existing_pass_predict.num_records) {
			if (operational && has_aop) {
				if (existing_pass_predict.num_records < MAX_AOP_SATELLITE_ENTRIES) {
					existing_pass_predict.records[j] = new_pass_predict.records[i];
					existing_pass_predict.num_records++;
					num_updated_records++;
				} else {
					DEBUG_WARN("ArgosRxService::update_pass_predict: database full, hexid=%02x dropped",
							(unsigned int)new_pass_predict.records[i].satHexId);
				}
			} else if (!operational) {
				// Unknown and out of service: nothing to store, but nothing
				// missing either as far as completeness is concerned
				num_updated_records++;
			}
		}
	}

	DEBUG_INFO("ArgosRxService::update_pass_predict: received=%u/%u constellation_status=%s",
			num_updated_records, new_pass_predict.num_records,
			m_constellation_status_tracking.is_complete() ? "complete" : "partial");

	// Normal case: commit once the whole constellation status set has been
	// received and every satellite it declares has been matched with an orbit
	// bulletin, so the database is never written with holes.
	const bool campaign_complete = m_constellation_status_tracking.is_complete() &&
			new_pass_predict.num_records &&
			num_updated_records == new_pass_predict.num_records;

	// Safety valve: a campaign that drags on is committed with what it has.
	// Waiting forever for one satellite would keep the whole database frozen,
	// including the bulletins that were correctly received and are ageing.
	const std::time_t now = service_current_time();
	bool campaign_expired = false;

	if (m_campaign_started == 0)
		m_campaign_started = now;
	else if (!campaign_complete && num_updated_records &&
			 (now - m_campaign_started) > (std::time_t)ARGOS_RX_CAMPAIGN_MAX_SECS) {
		campaign_expired = true;
		DEBUG_WARN("ArgosRxService::update_pass_predict: campaign running for %llu s "
				"with %u/%u satellites, committing what we have",
				(unsigned long long)(now - m_campaign_started),
				num_updated_records, new_pass_predict.num_records);
	}

	if (campaign_complete || campaign_expired) {
		// A complete status set is authoritative: the NT states that a
		// satellite previously operational and absent from the current CS
		// messages is to be considered decommissioned. Drop it from the
		// database, otherwise it lingers with an operational status and an
		// ageing bulletin, and the scheduler keeps booking windows on a
		// satellite that is no longer there. Only on a complete set — on the
		// expiry path the picture may be partial, and only when the merge did
		// not truncate, otherwise "absent" would just mean "did not fit".
		if (campaign_complete && new_pass_predict.num_records < MAX_AOP_SATELLITE_ENTRIES) {
			unsigned int kept = 0;

			for (unsigned int i = 0; i < existing_pass_predict.num_records; i++) {
				bool declared = false;

				for (unsigned int j = 0; j < new_pass_predict.num_records; j++) {
					if (existing_pass_predict.records[i].satHexId ==
							new_pass_predict.records[j].satHexId) {
						declared = true;
						break;
					}
				}
				if (!declared) {
					DEBUG_WARN("ArgosRxService::update_pass_predict: hexid=%02x no longer "
							"declared by the constellation, removed from the database",
							(unsigned int)existing_pass_predict.records[i].satHexId);
					continue;
				}
				if (kept != i)
					existing_pass_predict.records[kept] = existing_pass_predict.records[i];
				kept++;
			}
			existing_pass_predict.num_records = static_cast<uint8_t>(kept);
		}

		// Silence the radio FIRST, before touching flash. What follows — writing
		// the database, then the parameters, then the pass prediction recomputed
		// by service_complete() — takes seconds during which the main loop never
		// returns to the UART. Leaving the module in DL mode through that means
		// about one allcast message per second piling up in a buffer nobody
		// drains: the RX overflows, lines are truncated and merged, and the
		// AT+DL=0 that eventually goes out lands on a saturated link. Measured at
		// four seconds in the field.
		//
		// Order matters within these two lines too: stop_receive() is synchronous
		// and dispatches the frames still queued in the driver, which would call
		// back into react() and re-enter this very function. Closing the
		// acceptance window first makes those late frames a no-op.
		m_rx_window_open = false;
		service_cancel();

		DEBUG_INFO("ArgosRxService::update_pass_predict: committing %u AOP records", num_updated_records);
		configuration_store->write_pass_predict(existing_pass_predict);
		std::time_t new_aop_time = service_current_time();
		configuration_store->write_param(ParamID::ARGOS_AOP_DATE, new_aop_time);
		configuration_store->save_params();
		m_orbit_params_map.clear();
		m_constellation_status_map.clear();
		m_constellation_status_tracking.reset();
		m_campaign_started = 0;
		service_complete();
	}
}

/// @brief Constructor — reset location, zero earliest schedule.
ArgosRxScheduler::ArgosRxScheduler() : m_earliest_schedule(0) {
	m_location.reset();
}

/// @brief Find next downlink RX window using PREVIPASS (SAT_DNLK_ON filter).
/// @param argos_config    Argos configuration (prepass params, AOP update period).
/// @param pass_predict    AOP satellite database.
/// @param now             Current RTC time.
/// @param[out] timeout    RX window duration in ms.
/// @param[out] mode       Modulation for the RX window.
/// @return Delay in ms until RX start, or SCHEDULE_DISABLED.
unsigned int ArgosRxScheduler::schedule(ArgosConfig& argos_config, BasePassPredict& pass_predict, std::time_t now, unsigned int &timeout, KineisModulation& mode) {
	if (!m_location.has_value()) {
		DEBUG_TRACE("ArgosRxService::schedule: can't schedule as last location/time is not known");
		return Service::SCHEDULE_DISABLED;
	}

	// Update earliest schedule according to configuration and current time
	set_earliest_schedule(argos_config.last_aop_update + (SECONDS_PER_DAY * argos_config.argos_rx_aop_update_period));
	set_earliest_schedule(now);

	std::time_t start_time = m_earliest_schedule;
	std::time_t stop_time = start_time + (std::time_t)SECONDS_PER_DAY;

	DEBUG_TRACE("ArgosRxService::service_next_schedule_in_ms: searching window start=%llu stop=%llu",
			(unsigned long long)start_time, (unsigned long long)stop_time);

	// The search start moves forward when a candidate pass turns out to be
	// almost over, so the configuration is rebuilt on each iteration.
	auto build_config = [&](std::time_t from) {
		struct tm *p_tm = std::gmtime(&from);
		struct tm tm_start = *p_tm;
		p_tm = std::gmtime(&stop_time);
		struct tm tm_stop = *p_tm;

		PredictionPassConfiguration_t cfg = {
			(float)m_location.value().latitude,
			(float)m_location.value().longitude,
			{ (uint16_t)(1900 + tm_start.tm_year), (uint8_t)(tm_start.tm_mon + 1), (uint8_t)tm_start.tm_mday, (uint8_t)tm_start.tm_hour, (uint8_t)tm_start.tm_min, (uint8_t)tm_start.tm_sec },
			{ (uint16_t)(1900 + tm_stop.tm_year), (uint8_t)(tm_stop.tm_mon + 1), (uint8_t)tm_stop.tm_mday, (uint8_t)tm_stop.tm_hour, (uint8_t)tm_stop.tm_min, (uint8_t)tm_stop.tm_sec },
			(float)argos_config.prepass_min_elevation,        //< Minimum elevation of passes [0, 90]
			(float)argos_config.prepass_max_elevation,        //< Maximum elevation of passes  [maxElevation >= < minElevation]
			(float)argos_config.prepass_min_duration / 60.0f,  //< Minimum duration (minutes)
			argos_config.prepass_max_passes,                  //< Maximum number of passes per satellite (#)
			(float)argos_config.prepass_linear_margin / 60.0f, //< Linear time margin (in minutes/6months)
			argos_config.prepass_comp_step,                    //< Computation step (seconds)
			// PPP11, default 20 deg: the downlink window is a fixed cost, and a
			// grazing pass spends it for a link that will not close.
			(float)argos_config.prepass_rx_min_culmination,
			(float)argos_config.prepass_position_margin_km,   // PPP12
			true // includeCurrentPass
		};
		return cfg;
	};

	PredictionPassConfiguration_t pp_config = build_config(start_time);
	SatelliteNextPassPrediction_t next_pass;

	for (unsigned int attempt = 0; attempt < ARGOS_RX_MAX_PASS_CANDIDATES; attempt++) {

		if (!PREVIPASS_compute_next_pass_with_status(
				&pp_config,
				pass_predict.records,
				pass_predict.num_records,
				SAT_DNLK_ON,
				SAT_UPLK_ON_KINEIS_V1,
				&next_pass))
			break;

		// Set initial start/end points based on this discovered window
		std::time_t start = start_time, end = next_pass.epoch + next_pass.duration;

		// Advance to at least the prepass epoch position
		start = std::max((std::time_t)next_pass.epoch, start);

		DEBUG_INFO("ArgosRxScheduler::schedule_prepass: sat=%02x dl=%u e=%llu t=%llu [%llu %llu %llu]",
					(unsigned int)next_pass.satHexId,
					(unsigned int)next_pass.downlinkStatus,
					(unsigned long long)m_earliest_schedule,
					(unsigned long long)now,
					(unsigned long long)start_time,
					(unsigned long long)next_pass.epoch,
					(unsigned long long)end);

		// A pass already under way when the window opens only leaves its tail.
		// Powering the receiver for a few seconds yields nothing — the module
		// needs to acquire the satellite, and a full allcast cycle is 16
		// messages at roughly one per second — so skip to the next candidate
		// instead of burning a window on it.
		if (end <= start || (unsigned int)(end - start) < ARGOS_RX_MIN_WINDOW_SECS) {
			DEBUG_TRACE("ArgosRxScheduler::schedule_prepass: sat=%02x window too short (%d secs), skipping",
					(unsigned int)next_pass.satHexId, (int)(end - start));
			// Strictly monotonic: a candidate whose end is already behind the
			// current search start would otherwise be returned again and again,
			// burning the candidate budget without moving forward.
			start_time = std::max(end + (std::time_t)1, start_time + (std::time_t)1);
			if (start_time >= stop_time)
				break;
			pp_config = build_config(start_time);
			continue;
		}

		// Check we don't schedule off the end of the computed window
		if ((start + ARGOS_RX_MARGIN_MSECS) < end) {
			// We're good to go for this schedule, compute relative delay until the epoch arrives
			mode = KineisModulation::LDK;
			DEBUG_INFO("ArgosRxScheduler::schedule_prepass: scheduled in %llu secs | timeout %u secs",
					(unsigned long long)(start - now), (unsigned int)(end - start));
			timeout = (end - start) * MSECS_PER_SECOND;
			return (start - now) * MSECS_PER_SECOND;
		} else {
			break;
		}
	}

	DEBUG_ERROR("ArgosRxService::schedule: failed to find DL RX window");
	return Service::SCHEDULE_DISABLED;
}

/// @brief Advance earliest RX time (only moves forward, never backward).
/// @param t  Candidate earliest epoch time (seconds).
void ArgosRxScheduler::set_earliest_schedule(std::time_t t) {
	if (t > m_earliest_schedule) {
		DEBUG_TRACE("ArgosRxScheduler::set_earliest_schedule: new earliest: %llu", t);
		m_earliest_schedule = t;
	}
}

/// @brief Update last known GPS position for PREVIPASS computation.
/// @param lon  Longitude in degrees.
/// @param lat  Latitude in degrees.
void ArgosRxScheduler::set_location(double lon, double lat) {
	m_location = Location(lon, lat);
}
