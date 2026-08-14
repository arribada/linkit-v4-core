/**
 * @file argos_rx_service.cpp
 * @brief Argos RX service + scheduler — downlink AOP reception and pass predict update.
 */

#include "argos_rx_service.hpp"
#include "config_store.hpp"
#include "binascii.hpp"
#include "dte_protocol.hpp"

extern ConfigurationStore *configuration_store;

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
	m_kineis.start_receive(m_mode);
	m_cumulative_rx_time = 0;
}

/// @brief Cancel active RX — stop receiver.
/// @return true if RX was stopped.
bool ArgosRxService::service_cancel() {
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
	DEBUG_INFO("ArgosRxService::react(KineisEventRxPacket): packet=%s length=%u", Binascii::hexlify(e.packet).c_str(), e.size_bits);

	// Increment RX counter (RAM only — flash deferred to periodic flush / powerdown)
	configuration_store->increment_rx_counter();

	// Attempt to decode the queue of packets
	BasePassPredict pass_predict;
	PassPredictCodec::decode(m_orbit_params_map, m_constellation_status_map, e.packet, pass_predict);

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
		unsigned int j = 0;

		for (; j < existing_pass_predict.num_records; j++) {
			// Check for existing hex ID match
			if (new_pass_predict.records[i].satHexId == existing_pass_predict.records[j].satHexId) {
				DEBUG_TRACE("ArgosRxService::update_pass_predict: hexid=%01x dl=%u ul=%u aop=%u",
						(unsigned int)new_pass_predict.records[i].satHexId,
						(unsigned int)new_pass_predict.records[i].downlinkStatus,
						(unsigned int)new_pass_predict.records[i].uplinkStatus,
						(unsigned int)new_pass_predict.records[i].bulletin.year ? 1: 0
						);
				if ((new_pass_predict.records[i].downlinkStatus || new_pass_predict.records[i].uplinkStatus) &&
						new_pass_predict.records[i].bulletin.year) {
					num_updated_records++;
					existing_pass_predict.records[j] = new_pass_predict.records[i];
				} else if (!new_pass_predict.records[i].downlinkStatus && !new_pass_predict.records[i].uplinkStatus) {
					num_updated_records++;
					existing_pass_predict.records[j].downlinkStatus = new_pass_predict.records[i].downlinkStatus;
					existing_pass_predict.records[j].uplinkStatus = new_pass_predict.records[i].uplinkStatus;
				}
				break;
			}
		}

		// If we reached the end of the existing database then this is a new hex ID, so
		// add it to the end of the existing database
		if (j == existing_pass_predict.num_records &&
			existing_pass_predict.num_records < MAX_AOP_SATELLITE_ENTRIES) {
			if ((new_pass_predict.records[i].downlinkStatus || new_pass_predict.records[i].uplinkStatus) &&
					new_pass_predict.records[i].bulletin.year) {
				existing_pass_predict.records[j] = new_pass_predict.records[i];
				existing_pass_predict.num_records++;
				num_updated_records++;
			} else if (!new_pass_predict.records[i].downlinkStatus && !new_pass_predict.records[i].uplinkStatus) {
				existing_pass_predict.records[j].downlinkStatus = new_pass_predict.records[i].downlinkStatus;
				existing_pass_predict.records[j].uplinkStatus = new_pass_predict.records[i].uplinkStatus;
				existing_pass_predict.num_records++;
				num_updated_records++;
			}
		}
	}

	DEBUG_INFO("ArgosRxService::update_pass_predict: received=%u required=%u", num_updated_records, std::max(existing_pass_predict.num_records, new_pass_predict.num_records));

	// Check if we received a sufficient number of records
	if (num_updated_records == new_pass_predict.num_records && num_updated_records >= existing_pass_predict.num_records) {
		DEBUG_INFO("ArgosRxService::update_pass_predict: committing %u AOP records", num_updated_records);
		configuration_store->write_pass_predict(existing_pass_predict);
		std::time_t new_aop_time = service_current_time();
		configuration_store->write_param(ParamID::ARGOS_AOP_DATE, new_aop_time);
		configuration_store->save_params();
		m_orbit_params_map.clear();
		m_constellation_status_map.clear();
		service_cancel();
		service_complete();
	}
}

/// @brief Constructor — reset location, zero earliest schedule.
ArgosRxScheduler::ArgosRxScheduler() : m_earliest_schedule(0) {
	m_location.reset();
}

/// @brief Find next downlink RX window using PREVIPASS (SAT_DNLK_ON_WITH_A3 filter).
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
	struct tm *p_tm = std::gmtime(&start_time);
	struct tm tm_start = *p_tm;
	p_tm = std::gmtime(&stop_time);
	struct tm tm_stop = *p_tm;

	DEBUG_TRACE("ArgosRxService::service_next_schedule_in_ms: searching window start=%llu stop=%llu", start_time, stop_time);

	PredictionPassConfiguration_t pp_config = {
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
		20, // minCulmination  // TODO : ADD DTE PARAMETER
		0, // position error, // TODO : ADD DTE PARAMETER
		true // includeCurrentPass
	};
	SatelliteNextPassPrediction_t next_pass;

	while (PREVIPASS_compute_next_pass_with_status(
    	&pp_config,
		pass_predict.records,
		pass_predict.num_records,
		SAT_DNLK_ON,
		SAT_UPLK_ON_KINEIS_V1,
		&next_pass)) {

		// Set initial start/end points based on this discovered window
		std::time_t start = start_time, end = next_pass.epoch + next_pass.duration;

		// Advance to at least the prepass epoch position
		start = std::max((std::time_t)next_pass.epoch, start);

		DEBUG_INFO("ArgosRxScheduler::schedule_prepass: sat=%01x dl=%u e=%llu t=%llu [%llu %llu %llu]",
					(unsigned int)next_pass.satHexId,
					(unsigned int)next_pass.downlinkStatus,
					m_earliest_schedule,
					now,
					start_time,
					next_pass.epoch,
					end);

		// Check we don't schedule off the end of the computed window
		if ((start + ARGOS_RX_MARGIN_MSECS) < end) {
			// We're good to go for this schedule, compute relative delay until the epoch arrives
			mode = KineisModulation::LDK;
			DEBUG_INFO("ArgosRxScheduler::schedule_prepass: scheduled for %llu secs from now | timeout %u secs", start - now, end - start);
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
