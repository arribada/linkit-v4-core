/**
 * @file depth_pile.cpp
 * @brief Depth pile manager — GPS entry cache with burst counter management.
 */

#include "depth_pile.hpp"
#include "debug.hpp"
#include "config_store.hpp"
#include "scheduler.hpp"

extern ConfigurationStore *configuration_store;
extern Scheduler *system_scheduler;

/// @brief Constructor — load sensor TX enable bitmask from config.
DepthPileManager::DepthPileManager() {
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);
	m_sensor_tx_enable = argos_config.sensor_tx_enable | (1 << (int)ServiceIdentifier::GNSS_SENSOR);
	m_sensor_tx_current = 0;
}

/// @brief Handle peer events — cache GPS/sensor data, trigger depth pile update when all sensors ready.
/// @param e  Peer service event (GPS fix, ALS, PH, pressure, sea_temp, thermistor, AXL).
void DepthPileManager::notify_peer_event(ServiceEvent& e) {

	if (e.event_source == ServiceIdentifier::GNSS_SENSOR &&
		e.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
		GPSLogEntry& entry = std::get<GPSLogEntry>(e.event_data);
		if (!entry.info.valid) {
			// NO_FIX entries carry no position — they become 0xFF markers in
			// the transmitted packet. v3 behavior (restored 2026-07): in
			// LEGACY/DUTY_CYCLE/PASS_PREDICTION they ARE cached — the 0xFF
			// packet doubles as an "alive" heartbeat and the entry occupies its
			// slot on the uniform delta_time_loc grid, keeping multi-position
			// packet dating correct. Only exception: SURFACING_BURST with
			// fastloc enabled (GNP45 != OFF), where process_doppler_burst()
			// emits the DEGRADED_PVT / CLOUDLOCATE replacement packet instead.
			unsigned int fastloc_mode = configuration_store->read_param<unsigned int>(ParamID::GNSS_FASTLOC_MODE);
			// Use the EFFECTIVE Argos mode from the per-regime cascade
			// (resolves LB / OUT_OF_ZONE / HAULED overrides), matching what
			// update_depth_pile and ArgosTxService act on — NOT the raw ARGOS_MODE slot
			// (the NORMAL-regime value), which would mis-gate when per-regime modes differ.
			ArgosConfig cfg;
			configuration_store->get_argos_configuration(cfg);
			if (fastloc_mode != (unsigned int)BaseFastlocMode::OFF &&
			    cfg.mode == BaseArgosMode::SURFACING_BURST) {
				DEBUG_INFO("DepthPileManager::notify_peer_event: skip NO_FIX (fastloc GNP45=%u + SURFACING_BURST)",
				           fastloc_mode);
				return;  // Do NOT cache, do NOT mark ready — fastloc will handle fallback
			}
			DEBUG_WARN("DepthPileManager::notify_peer_event: GNSS NO_FIX cached — will TX as 0xFF heartbeat (grid filler)");
		} else {
			DEBUG_TRACE("DepthPileManager::notify_peer_event: GNSS cache set");
		}
		m_gps_cache = entry;
		m_sensor_tx_current |= (1 << (int)ServiceIdentifier::GNSS_SENSOR);
	} else if (e.event_source == ServiceIdentifier::ALS_SENSOR &&
			e.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
		DEBUG_TRACE("DepthPileManager::notify_peer_event: ALS cache set");
		ServiceSensorData& entry = std::get<ServiceSensorData>(e.event_data);
		m_als_cache.port[0] = entry.port[0];
		m_sensor_tx_current |= (1 << (int)ServiceIdentifier::ALS_SENSOR);
	} else if (e.event_source == ServiceIdentifier::PH_SENSOR &&
			e.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
		DEBUG_TRACE("DepthPileManager::notify_peer_event: PH cache set");
		ServiceSensorData& entry = std::get<ServiceSensorData>(e.event_data);
		m_ph_cache.port[0] = (unsigned int)(entry.port[0] * 1000U);
		m_sensor_tx_current |= (1 << (int)ServiceIdentifier::PH_SENSOR);
	} else if (e.event_source == ServiceIdentifier::PRESSURE_SENSOR &&
			e.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
		DEBUG_TRACE("DepthPileManager::notify_peer_event: PRESSURE cache set");
		ServiceSensorData& entry = std::get<ServiceSensorData>(e.event_data);
		m_pressure_cache.port[0] = (unsigned int)(entry.port[0] * 1000U);
		m_pressure_cache.port[1] = (unsigned int)((entry.port[1] + 40.0) * 100U);
		m_sensor_tx_current |= (1 << (int)ServiceIdentifier::PRESSURE_SENSOR);
	} else if (e.event_source == ServiceIdentifier::SEA_TEMP_SENSOR &&
			e.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
		DEBUG_TRACE("DepthPileManager::notify_peer_event: SEA_TEMP cache set");
		ServiceSensorData& entry = std::get<ServiceSensorData>(e.event_data);
		// Encoding: (°C + 126) × 100, same step (0.01°C) and pattern as
		// pressure_temp / AXL temp / thermistor. The offset +126 °C reflects the
		// EZO-RTD valid range floor (-126 °C, cf ezo_rtd.cpp). Multiplier × 100
		// keeps the result inside the 14-bit field (max 16383) for typical
		// marine temps; a previous × 1000 truncated silently via PACK_BITS for
		// every value > -109.6 °C — i.e. all real sea readings. Decoder:
		//   °C = encoded / 100 − 126.
		m_sea_temp_cache.port[0] = (unsigned int)((entry.port[0] + 126.0) * 100U);
		m_sensor_tx_current |= (1 << (int)ServiceIdentifier::SEA_TEMP_SENSOR);
#if ENABLE_THERMISTOR_SENSOR
	} else if (e.event_source == ServiceIdentifier::THERMISTOR_SENSOR &&
			e.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
		DEBUG_TRACE("DepthPileManager::notify_peer_event: THERMISTOR cache set");
		ServiceSensorData& entry = std::get<ServiceSensorData>(e.event_data);
		m_sea_temp_cache.port[0] = (unsigned int)((entry.port[0] + 40.0) * 100U);
		m_sensor_tx_current |= (1 << (int)ServiceIdentifier::THERMISTOR_SENSOR);
#endif
#if ENABLE_AXL_SENSOR
	} else if (e.event_source == ServiceIdentifier::AXL_SENSOR &&
			e.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
		DEBUG_TRACE("DepthPileManager::notify_peer_event: AXL cache set");
		ServiceSensorData& entry = std::get<ServiceSensorData>(e.event_data);
		// Read configured g-range (register value 0-3) and convert to actual g value
		unsigned int range_reg = configuration_store->read_param<unsigned int>(ParamID::AXL_SENSOR_MEASUREMENT_RANGE);
		static const double g_range_table[] = { 2.0, 4.0, 8.0, 16.0 };
		double g_range = (range_reg < 4) ? g_range_table[range_reg] : 16.0;
		m_axl_cache.port[0] = (unsigned int)((entry.port[0] + 40.0) * 100U);                  // Temperature (-40 to 85°C)
		m_axl_cache.port[1] = (unsigned int)((entry.port[1] + g_range) * 1000U);               // X axis
		m_axl_cache.port[2] = (unsigned int)((entry.port[2] + g_range) * 1000U);               // Y axis
		m_axl_cache.port[3] = (unsigned int)((entry.port[3] + g_range) * 1000U);               // Z axis
		m_axl_cache.port[4] = (unsigned int)(entry.port[4]);                                   // Activity (0-255)
		m_sensor_tx_current |= (1 << (int)ServiceIdentifier::AXL_SENSOR);
#endif
	} else if (e.event_source == ServiceIdentifier::GNSS_SENSOR &&
			e.event_type == ServiceEventType::SERVICE_INACTIVE) {

		// If we didn't yet gather all the expected inputs then we start
		// a timeout to force dummy values into the depth pile
		if ((m_sensor_tx_current & m_sensor_tx_enable) != m_sensor_tx_enable) {
			DEBUG_TRACE("DepthPileManager: GNSS inactive, sensors pending (curr=%08x enable=%08x) — arming dummy-fill timeout", m_sensor_tx_current, m_sensor_tx_enable);
			m_timeout_task = system_scheduler->post_task_prio([this]() {
				DEBUG_WARN("DepthPileManager: sensor timeout — filling missing sensor with 0xFF dummy values: curr=%08x enable=%08x", m_sensor_tx_current, m_sensor_tx_enable);
				if (((1 << (int)ServiceIdentifier::ALS_SENSOR) & m_sensor_tx_enable) &&
					((1 << (int)ServiceIdentifier::ALS_SENSOR) & m_sensor_tx_current) == 0) {
					m_als_cache.port[0] = 0xFFFFFFFF;
					m_sensor_tx_current |= (1 << (int)ServiceIdentifier::ALS_SENSOR);
				} else if (((1 << (int)ServiceIdentifier::PH_SENSOR) & m_sensor_tx_enable) &&
						((1 << (int)ServiceIdentifier::PH_SENSOR) & m_sensor_tx_current) == 0) {
					m_ph_cache.port[0] = 0xFFFFFFFF;
					m_sensor_tx_current |= (1 << (int)ServiceIdentifier::PH_SENSOR);
				} else if (((1 << (int)ServiceIdentifier::PRESSURE_SENSOR) & m_sensor_tx_enable) &&
						((1 << (int)ServiceIdentifier::PRESSURE_SENSOR) & m_sensor_tx_current) == 0) {
					m_pressure_cache.port[0] = 0xFFFFFFFF;
					m_pressure_cache.port[1] = 0xFFFFFFFF;
					m_sensor_tx_current |= (1 << (int)ServiceIdentifier::PRESSURE_SENSOR);
				} else if (((1 << (int)ServiceIdentifier::SEA_TEMP_SENSOR) & m_sensor_tx_enable) &&
						((1 << (int)ServiceIdentifier::SEA_TEMP_SENSOR) & m_sensor_tx_current) == 0) {
					m_sea_temp_cache.port[0] = 0xFFFFFFFF;
					m_sensor_tx_current |= (1 << (int)ServiceIdentifier::SEA_TEMP_SENSOR);
#if ENABLE_THERMISTOR_SENSOR
				} else if (((1 << (int)ServiceIdentifier::THERMISTOR_SENSOR) & m_sensor_tx_enable) &&
						((1 << (int)ServiceIdentifier::THERMISTOR_SENSOR) & m_sensor_tx_current) == 0) {
					m_sea_temp_cache.port[0] = 0xFFFFFFFF;
					m_sensor_tx_current |= (1 << (int)ServiceIdentifier::THERMISTOR_SENSOR);
#endif
#if ENABLE_AXL_SENSOR
				} else if (((1 << (int)ServiceIdentifier::AXL_SENSOR) & m_sensor_tx_enable) &&
						((1 << (int)ServiceIdentifier::AXL_SENSOR) & m_sensor_tx_current) == 0) {
					m_axl_cache.port[0] = 0xFFFFFFFF;
					m_axl_cache.port[1] = 0xFFFFFFFF;
					m_axl_cache.port[2] = 0xFFFFFFFF;
					m_axl_cache.port[3] = 0xFFFFFFFF;
					m_axl_cache.port[4] = 0xFF;
					m_sensor_tx_current |= (1 << (int)ServiceIdentifier::AXL_SENSOR);
#endif
				}
				update_depth_pile();
			}, "DepthPileTimeout", Scheduler::DEFAULT_PRIORITY, 2000U);
		}
	}

	update_depth_pile();
}

/// @brief Store cached GPS/sensor data into depth piles with burst counter from config.
void DepthPileManager::update_depth_pile() {

	if ((m_sensor_tx_current & m_sensor_tx_enable) == m_sensor_tx_enable) {

		// Cancel inactivity timeout
		system_scheduler->cancel_task(m_timeout_task);

		// Get the required burst counter
		ArgosConfig argos_config;
		configuration_store->get_argos_configuration(argos_config);

		// Apply ARGOS_DEPTH_PILE as the effective cap for every pile. Older
		// entries (pending retries included) are evicted when the cap shrinks
		// or when a new fix arrives on a full pile — see DepthPile::set_max_size().
		unsigned int max_size = (unsigned int)argos_config.depth_pile;
		m_gps_depth_pile.set_max_size(max_size);
		m_als_depth_pile.set_max_size(max_size);
		m_ph_depth_pile.set_max_size(max_size);
		m_pressure_depth_pile.set_max_size(max_size);
		m_sea_temp_depth_pile.set_max_size(max_size);
#if ENABLE_AXL_SENSOR
		m_axl_depth_pile.set_max_size(max_size);
#endif

		unsigned int burst_counter;
		if (argos_config.mode == BaseArgosMode::DUTY_CYCLE ||
			argos_config.mode == BaseArgosMode::LEGACY ||
			argos_config.mode == BaseArgosMode::PASS_PREDICTION) {
			// LEGACY / DUTY_CYCLE / PASS_PREDICTION: NTRY_PER_MESSAGE (ARP19) is
			// the number of times each depth pile entry is transmitted — N full
			// sweeps of the pile (A, B, C, A, B, C, ...) then the entry goes
			// inert. 0 = unlimited: keep cycling the pile until newer fixes
			// evict the entries (FIFO). A no-fix cycle caches a NO_FIX entry
			// (see notify_peer_event) transmitted as a 0xFF heartbeat that
			// also fills its delta_time_loc grid slot (v3 behavior).
			burst_counter = (argos_config.ntry_per_message == 0) ? UINT_MAX : argos_config.ntry_per_message;
		} else if (argos_config.ntry_per_message == 0) {
			// Surfacing burst: 0 means send once per fix
			burst_counter = 1;
		} else {
			burst_counter = argos_config.ntry_per_message;
		}
		DEBUG_TRACE("DepthPileManager: mode=%u ntry_per_message=%u -> burst_counter=%u, pile max_size=%u",
		            (unsigned int)argos_config.mode, argos_config.ntry_per_message, burst_counter, max_size);

		// Synchronously update the depth piles
		if (m_sensor_tx_current & (1 << (int)ServiceIdentifier::GNSS_SENSOR)) {
			bool is_planned_mode = (argos_config.mode == BaseArgosMode::LEGACY ||
			                        argos_config.mode == BaseArgosMode::DUTY_CYCLE ||
			                        argos_config.mode == BaseArgosMode::PASS_PREDICTION);
			if (m_gps_cache.info.event_type == GPSEventType::NO_FIX && !is_planned_mode) {
				// NO_FIX dedup (SURFACING_BURST & co only): if the cache is
				// NO_FIX and the last pile entry is also NO_FIX, replace rather
				// than append — keep one fresh heartbeat timestamp instead of
				// wasting airtime. NOT applied to LEGACY/DUTY_CYCLE/
				// PASS_PREDICTION (v3 behavior): there every NO_FIX entry
				// occupies its own slot on the uniform delta_time_loc grid so
				// multi-position packet dating stays exact.
				bool replaced = m_gps_depth_pile.store_or_replace_last(m_gps_cache, burst_counter,
					[](const GPSLogEntry& prev) {
						return prev.info.event_type == GPSEventType::NO_FIX;
					});
				if (replaced) {
					DEBUG_INFO("DepthPileManager: NO_FIX dedup — replaced previous NO_FIX (heartbeat refreshed)");
				}
			} else {
				m_gps_depth_pile.store(m_gps_cache, burst_counter);
				DEBUG_INFO("DepthPileManager: GPS %s stored, burst_counter=%u (0xFFFFFFFF=unlimited)",
				           m_gps_cache.info.valid ? "FIX" : "NO_FIX heartbeat", burst_counter);
			}
		}
		if (m_sensor_tx_current & (1 << (int)ServiceIdentifier::ALS_SENSOR)) {
			// Store the entry into the depth pile
			m_als_depth_pile.store(m_als_cache, burst_counter);
		}
		if (m_sensor_tx_current & (1 << (int)ServiceIdentifier::PH_SENSOR)) {
			// Store the entry into the depth pile
			m_ph_depth_pile.store(m_ph_cache, burst_counter);
		}
		if (m_sensor_tx_current & (1 << (int)ServiceIdentifier::PRESSURE_SENSOR)) {
			// Store the entry into the depth pile
			m_pressure_depth_pile.store(m_pressure_cache, burst_counter);
		}
		if (m_sensor_tx_current & (1 << (int)ServiceIdentifier::SEA_TEMP_SENSOR)) {
			// Store the entry into the depth pile
			m_sea_temp_depth_pile.store(m_sea_temp_cache, burst_counter);
		}
#if ENABLE_THERMISTOR_SENSOR
		if (m_sensor_tx_current & (1 << (int)ServiceIdentifier::THERMISTOR_SENSOR)) {
			// Thermistor uses the sea_temp depth pile slot
			m_sea_temp_depth_pile.store(m_sea_temp_cache, burst_counter);
		}
#endif
#if ENABLE_AXL_SENSOR
		if (m_sensor_tx_current & (1 << (int)ServiceIdentifier::AXL_SENSOR)) {
			// Store the AXL entry into the depth pile
			m_axl_depth_pile.store(m_axl_cache, burst_counter);
		}
#endif
		m_sensor_tx_current = 0;
	}
}
