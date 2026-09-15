/**
 * @file lora_tx_service.cpp
 * @brief LoRa TX service — scheduling, burst preparation, TX event handling.
 */

#include <climits>
#include <algorithm>

#include "lora_tx_service.hpp"
#include "gps.hpp"
#include "messages.hpp"
#include "timeutils.hpp"
#include "exponential_backoff.hpp"
#include "binascii.hpp"
#include "debug.hpp"
#include "moored_mode_service.hpp"
#if defined(LORA_MOTION_EXT) && (LORA_MOTION_EXT == 1)
#include "axl_sensor_service.hpp"
#endif

extern ConfigurationStore *configuration_store;
extern Scheduler *system_scheduler;
extern GPSDevice *gps_device;

LoRaTxService::LoRaTxService(KineisDevice &device) : Service(ServiceIdentifier::LORA_TX, "LORATX"), m_device(device) {}

void LoRaTxService::service_init() {
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);

	m_device.subscribe(*this);

	// Warn if SURFACING_BURST mode is configured without underwater detection
	if (argos_config.mode == BaseArgosMode::SURFACING_BURST && !argos_config.underwater_en) {
		DEBUG_WARN("LoRaTxService: SURFACING_BURST mode requires UNDERWATER_EN=1 — burst will not trigger without SWS");
	}

	// Jitter seed: shared derivation with ArgosTxService (see
	// ConfigurationStore::get_tx_jitter_seed). On a LoRa build that resolves to
	// the DevEUI — neither Argos ID param is ever written here, and seeding
	// from them would give every unit mt19937(0): identical jitter, and a
	// fleet-wide constant offset instead of the per-unit spread that keeps
	// units from transmitting on top of each other.
	m_sched.reset(configuration_store->get_tx_jitter_seed());
	m_depth_pile_manager.clear();
	m_is_first_tx = true;
	m_is_tx_pending = false;
	m_session_tx_count = 0;
	m_consecutive_device_errors = 0;
	m_is_surfacing_burst = false;
	m_awaiting_surfacing = false;
	m_has_gnss_fix_since_surfacing = false;
	m_first_gnss_tx_sent = false;
	m_status_burst_count = 0;
	m_last_tx_had_gps = false;
	m_cooldown_armed = false;
	m_cloudlocate_ready_pending = false;
#if defined(LORA_MOTION_EXT) && (LORA_MOTION_EXT == 1)
	m_motion_in_flight = 0;
#endif
#if defined(LORA_LINKCHECK) && (LORA_LINKCHECK == 1)
	m_inflight_encoded.clear();
	DEBUG_INFO("LoRaTxService: LinkCheck store-and-forward — a position is spent only once the network heard it");
#endif

	DEBUG_TRACE("LoRaTxService::service_init: initialized");
}

void LoRaTxService::service_term() {
	m_device.unsubscribe(*this);
	// 2026-05-25 Fix #10c: mirror ArgosTxService and GPSService — cut LoRa
	// rail unconditionally on service_term. Previously this method only
	// unsubscribed the listener, leaving the RAK3172 powered (potentially
	// mid-TX/standby) for the OffState path's PMU::powerdown delay window
	// (~OFF_LED_PERIOD_MS). Worse, on ConfigurationState / Error transits
	// the RAK could stay alive consuming SAT rail current with no service
	// to manage it. power_off_immediate() cancels in-flight TX, calls
	// power_off_enter (rail cut + UART deinit + pin park), so the next
	// LoRaTxService init starts from a clean cold-boot.
	m_device.power_off_immediate();
}

bool LoRaTxService::service_is_enabled() {
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);
	return (argos_config.mode != BaseArgosMode::OFF);
}

/// @brief Translate a LoRaTxScheduler answer, which uses INVALID_SCHEDULE for
/// "could not compute a slot" -- numerically the same value as SCHEDULE_DISABLED,
/// which is exactly how a computation failure used to pass for a deliberate stop.
static ScheduleDecision from_scheduler(unsigned int ms, const char *ran_why, const char *failed_why) {
	if (ms == LoRaTxScheduler::INVALID_SCHEDULE) return ScheduleDecision::hold_for_event(600, failed_why);
	return ScheduleDecision::run(ms, ran_why);
}

/// @brief Burst over, still no surfacing: presence heartbeat at the nominal rate.
/// Mirror of ArgosTxService::schedule_surfacing_heartbeat.
ScheduleDecision LoRaTxService::schedule_surfacing_heartbeat(ArgosConfig &argos_config, std::time_t now) {
	DEBUG_INFO("LoRaTxService: burst finished, waiting for the next surfacing — status heartbeat at the "
	           "nominal period meanwhile (not a fault).");
	m_scheduled_task = [this]() { process_status_burst(); };
	return from_scheduler(m_sched.schedule_legacy(argos_config, now), "surfacing heartbeat",
	                      "surfacing heartbeat: no slot computable");
}

/// @brief Legacy entry point, superseded by service_next_schedule().
/// Still required while the base declares it pure -- deliberately, so that a new
/// service implementing neither cannot compile and be silently off for ever.
unsigned int LoRaTxService::service_next_schedule_in_ms() {
	return SCHEDULE_DISABLED;
}

ScheduleDecision LoRaTxService::schedule_at_decision(std::time_t now, std::time_t t, const char *why) {
	const std::time_t effective = m_sched.schedule_at(t);
	return ScheduleDecision::run((effective > now) ? (unsigned int)(effective - now) * 1000U : 0U, why);
}

/// @brief Decide what LoRa does next: transmit, wait on a named gate, or stay off.
ScheduleDecision LoRaTxService::service_next_schedule() {
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);
	std::time_t now = service_current_time();

	DEBUG_TRACE("LoRaTxService::service_next_schedule_in_ms");

	// Critical battery check: immediate powerdown, no transmission.
	//
	// Meme garde que ArgosTxService : on consulte le verdict CONFIRME du
	// moniteur, pas un relevé brut. m_is_critical_voltage n'est pose qu'apres
	// BATT_CONFIRM_SAMPLES = 3 echantillons consecutifs sous le seuil, ce qui est
	// ce qui rend l'action sure : un echantillon isole, pris pendant une pointe de
	// courant d'emission ou juste apres un demarrage a froid, lit tres au-dessous
	// de l'etat reel. Cette branche appelle PMU::powerdown(), et sur une balise
	// scellee c'est la fin de la mission -- le reed est sous la colle.
	// Ce chemin avait ete oublie lors du durcissement d'ArgosTxService : une
	// balise LoRa s'eteignait encore sur un transitoire.
	if (argos_config.is_lb) {
		service_update_battery();
		unsigned int critical_level = configuration_store->read_param<unsigned int>(ParamID::LB_CRITICAL_THRESH);
		unsigned int current_soc = service_get_level();
		if (current_soc < critical_level && service_is_battery_critical()) {
			DEBUG_INFO("LoRaTxService: CRITICAL battery SOC %u%% < %u%% (confirmed) - shutdown", current_soc,
			           critical_level);
			configuration_store->save_params();
			PMU::powerdown();
			return ScheduleDecision::off("critical battery — powering down");
		}
	}

	if (argos_config.mode == BaseArgosMode::OFF) {
		return ScheduleDecision::off("ARGOS_MODE=OFF");
	}

	// The first-message lock is gone, as it went on the Argos side in 2026-08.
	//
	// It held back EVERY transmission -- presence heartbeat included -- for as
	// long as no fix had set the clock. A tag whose receiver was broken or whose
	// sky was blocked therefore vanished from the network entirely: no position,
	// and no sign of life either. Losing the position is the receiver's fault;
	// losing the beacon as well is ours.
	//
	// One nuance LoRa needs and Argos did not. Argos kept its time-sync burst
	// conditional because that burst CARRIES the time; LoRa has no equivalent,
	// so there is nothing to hold back on those grounds. What it does have is a
	// depth pile whose entries carry the instant they were taken. So with the
	// clock still unset, prefer the status heartbeat: build_status_packet is a
	// 14-bit header -- packet type, flags, battery -- with no timestamp in it at
	// all, and is therefore correct on any clock.
	const bool clock_unset = argos_config.gnss_en && !service_is_time_known();
	if (clock_unset) {
		DEBUG_INFO("LoRaTxService: clock not set yet — sending the status heartbeat instead of holding TX back "
		           "(depth-pile entries carry their own timestamps and are kept for later).");
	}

	// If depth pile has eligible entries, schedule GPS or sensor burst
	if (!clock_unset && m_depth_pile_manager.eligible()) {
		if (argos_config.sensor_tx_enable) {
			m_scheduled_task = [this]() { process_sensor_burst(); };
		} else {
			m_scheduled_task = [this]() { process_gps_burst(); };
		}
	} else {
		// No GPS data available, send status heartbeat
		m_scheduled_task = [this]() { process_status_burst(); };
	}

	if (argos_config.mode == BaseArgosMode::DUTY_CYCLE) {
		if (m_is_first_tx && m_depth_pile_manager.eligible()) {
			return schedule_at_decision(now, now, "first TX, depth pile ready");
		}
		return from_scheduler(m_sched.schedule_duty_cycle(argos_config, now), "duty-cycle window",
		                      "duty cycle: no window computable");
	}
	if (argos_config.mode == BaseArgosMode::LEGACY) {
		if (m_is_first_tx && m_depth_pile_manager.eligible()) {
			return schedule_at_decision(now, now, "first TX, depth pile ready");
		}
		return from_scheduler(m_sched.schedule_legacy(argos_config, now), "TR_NOM period",
		                      "legacy: no slot computable");
	}
	if (argos_config.mode == BaseArgosMode::SURFACING_BURST) {
		// Phase 1: Status heartbeat burst (battery level) until GNSS fix
		if (m_is_surfacing_burst && !m_has_gnss_fix_since_surfacing) {
			// Check max message limit (0 = unlimited)
			unsigned int burst_max_msg =
			    configuration_store->read_param<unsigned int>(ParamID::SURFACING_BURST_MAX_MSG);
			if (burst_max_msg > 0 && m_status_burst_count >= burst_max_msg) {
				DEBUG_INFO("LoRaTxService::SURFACING_BURST: max status messages reached (%u/%u)", m_status_burst_count,
				           burst_max_msg);
				// Arm cooldown if trigger mode is END_OF_DOPPLER (status burst ends
				// because max_msg reached without GNSS fix) — parity with Argos.
				unsigned int trigger = configuration_store->read_param<unsigned int>(ParamID::COOLDOWN_TRIGGER_MODE);
				if (trigger == (unsigned int)BaseCooldownTrigger::END_OF_DOPPLER && !m_cooldown_armed) {
					m_cooldown_armed = true;
					DEBUG_INFO("LoRaTxService: cooldown armed (END_OF_DOPPLER, max msg)");
				}
				m_is_surfacing_burst = false;
				m_awaiting_surfacing = true;
				// Heartbeat now, in this same pass. Setting the flag and then
				// answering "nothing" left the beacon mute until something else
				// happened to re-evaluate it -- and the flag itself is what
				// stops anything else from happening.
				return schedule_surfacing_heartbeat(argos_config, now);
			}

			m_scheduled_task = [this]() { process_status_burst(); };

			// First message: normally immediate, but if FASTLOC_MODE=CLOUDLOCATE
			// (and no cached position is available) defer to give the GPS time
			// to capture the raw measurement before our 1st TX. The raw-ready
			// notification (notify_peer_event below) pre-empts this delay via
			// service_reschedule(true), so the actual 1st TX fires the moment
			// raw is available — typically 5-15 s after surface. The full
			// timeout only fires as a safety fallback if GPS never emits raw,
			// at which point we send a STATUS ping (raw=false branch in
			// process_status_burst).
			//
			// When a cached position (last GPS fix or last Fastloc) is
			// available, we skip the defer and fire immediately at surface —
			// the 1st TX packet content is "prepared underwater" from the
			// cache, and we don't need to wait for live data.
			if (m_status_burst_count == 0) {
				const GPSLogEntry &cached_gps = configuration_store->get_last_gps_entry();
				const GPSLogEntry &cached_fl = configuration_store->get_last_fastloc_entry();
				bool have_cached_position =
				    (cached_gps.info.valid && cached_gps.info.event_type == GPSEventType::FIX)
				    || (cached_fl.info.valid && cached_fl.info.event_type == GPSEventType::FASTLOC);

				if (have_cached_position) {
					DEBUG_INFO("LoRaTxService::SURFACING_BURST: status #1 immediate (cached position available)");
					return schedule_at_decision(now, now, "surfacing burst: status #1, cached position");
				}

				unsigned int fastloc_mode = configuration_store->read_param<unsigned int>(ParamID::GNSS_FASTLOC_MODE);
				// Defer 1st STATUS only when a CloudLocate raw can actually arrive:
				//   - FASTLOC_MODE == CLOUDLOCATE (CloudLocate enabled)
				//   - argos_config.gnss_en (GPS hardware will run)
				// Without the gnss_en check, a deployment with GNSS_EN=0 would
				// burn the full GNSS_ACQ_TIMEOUT on every surface waiting for
				// a raw that can never come.
				if (fastloc_mode == (unsigned int)BaseFastlocMode::CLOUDLOCATE && argos_config.gnss_en) {
					unsigned int wait_s = configuration_store->read_param<unsigned int>(ParamID::GNSS_ACQ_TIMEOUT);
					if (wait_s == 0) wait_s = 30;  // Safety floor — should never fire (param min is 10)
					DEBUG_INFO(
					    "LoRaTxService::SURFACING_BURST: status #1 deferred up to %u s waiting for CloudLocate raw",
					    wait_s);
					return schedule_at_decision(now, now + wait_s, "surfacing burst: waiting for a CloudLocate raw");
				}
				DEBUG_INFO("LoRaTxService::SURFACING_BURST: status #%u (immediate, no cache)",
				           m_status_burst_count + 1);
				return schedule_at_decision(now, now, "surfacing burst: status #1");
			}

			// Progressive interval: init + (count-1) * step, capped at max
			unsigned int interval_s =
			    argos_config.surfacing_burst_init_s + (m_status_burst_count - 1) * argos_config.surfacing_burst_step_s;
			if (interval_s > argos_config.surfacing_burst_max_s) interval_s = argos_config.surfacing_burst_max_s;

			// Demoted to TRACE: fires on every progressive ping in the burst.
			// Burst start ("status #1 immediate") and end ("max messages
			// reached" / "STATUS-PURE silencing") are the meaningful markers.
			DEBUG_TRACE("LoRaTxService::SURFACING_BURST: status #%u in %u s", m_status_burst_count + 1, interval_s);
			return schedule_at_decision(now, now + interval_s, "surfacing burst: progressive status");
		}

		// Phase 2: GNSS fix available — TX position immediately, then TR_NOM
		if (m_has_gnss_fix_since_surfacing) {
			if (!m_depth_pile_manager.eligible()) {
				DEBUG_TRACE("LoRaTxService::SURFACING_BURST: GNSS phase but no eligible entries");
				m_is_surfacing_burst = false;
				m_awaiting_surfacing = true;
				m_has_gnss_fix_since_surfacing = false;
				m_first_gnss_tx_sent = false;
				return schedule_surfacing_heartbeat(argos_config, now);
			}
			if (argos_config.sensor_tx_enable) {
				m_scheduled_task = [this]() { process_sensor_burst(); };
			} else {
				m_scheduled_task = [this]() { process_gps_burst(); };
			}

			// First GNSS TX is immediate after fix, then use tx_interval_s
			// Note: m_first_gnss_tx_sent is set in service_initiate(), not here.
			if (!m_first_gnss_tx_sent) {
				DEBUG_INFO("LoRaTxService::SURFACING_BURST: GNSS fix — TX immediate");
				return schedule_at_decision(now, now, "surfacing burst: first TX after fix");
			}
			return from_scheduler(m_sched.schedule_legacy(argos_config, now), "surfacing burst: GNSS phase",
			                      "surfacing burst: no slot computable");
		}

		// Burst ended — wait for the next surfacing event, but keep the presence
		// heartbeat going rather than returning SCHEDULE_DISABLED. Same reasoning
		// as ArgosTxService::schedule_surfacing_heartbeat: returning disabled
		// leaves the service owning nothing, and only a dive→surface transition
		// or a new valid fix clears the flag -- so an animal that stopped diving
		// and is not getting fixes went silent for the rest of the deployment.
		// A status heartbeat at the nominal period is what the beacon has to say
		// meanwhile, and it costs nothing while the animal is actually diving
		// because the dive deschedules the service outright.
		if (m_awaiting_surfacing) return schedule_surfacing_heartbeat(argos_config, now);

		// Not yet surfaced (boot): status heartbeat at the legacy rate, as Argos
		// sends its Doppler at the legacy rate in the same situation. A tag that
		// has not dived yet is usually one that has just been deployed; being
		// silent until the first dive is indistinguishable from a dead radio.
		m_scheduled_task = [this]() { process_status_burst(); };
		return from_scheduler(m_sched.schedule_legacy(argos_config, now), "not surfaced yet — legacy-rate status",
		                      "not surfaced yet: no slot computable");
	}

	return ScheduleDecision::off("ARGOS_MODE has no LoRa equivalent");
}

void LoRaTxService::service_initiate() {
	DEBUG_TRACE("LoRaTxService::service_initiate");

	if (m_consecutive_device_errors >= DEVICE_ERROR_MAX_CONSECUTIVE) {
		// Time-bounded, not permanent. Once the deadline passes, let exactly one
		// dispatch through as a probe by stepping the counter back below the
		// threshold: if it succeeds, TxComplete clears the counter outright; if
		// it fails, react() puts it back at MAX and arms a fresh deadline. That
		// keeps the battery guard (no free-running retry storm) while making the
		// suspension recoverable without a reboot.
		std::time_t now = service_current_time();
		// DEVICE_ERROR_PROBE_PERIOD_S == 0 (bench builds): never hold a dispatch
		// back. The capped exponential backoff already spaces retries to at most
		// one per DEVICE_ERROR_BACKOFF_MAX_MS, which is the guard that matters.
		if (DEVICE_ERROR_PROBE_PERIOD_S && now < m_device_error_suspend_until) {
			DEBUG_WARN("LoRaTxService::service_initiate: skipping TX — %u consecutive errors, "
			           "suspended for another %llu s",
			           m_consecutive_device_errors, (unsigned long long)(m_device_error_suspend_until - now));
			service_complete(nullptr, nullptr, false);
			return;
		}
		DEBUG_INFO("LoRaTxService::service_initiate: suspension elapsed — probing with one TX");
		m_consecutive_device_errors = DEVICE_ERROR_MAX_CONSECUTIVE - 1;
	}

	m_is_first_tx = false;
	m_is_tx_pending = true;

	// Track status burst count in SURFACING_BURST phase 1. Incremented HERE —
	// before process_status_burst() runs — so that after this point
	// m_status_burst_count is the index of the *current* TX (1-based).
	// Convention used by logs:
	//   - service_schedule() reads the counter BEFORE this increment, so it logs
	//     `count + 1` to refer to the upcoming TX (e.g. "status #1 (immediate)").
	//   - process_status_burst() reads the counter AFTER this increment, so it
	//     logs `count` directly to refer to the current TX.
	if (m_is_surfacing_burst && !m_has_gnss_fix_since_surfacing) {
		m_status_burst_count++;
	}

	// Mark first GNSS TX as sent only when actually executing (not during scheduling)
	if (m_has_gnss_fix_since_surfacing && !m_first_gnss_tx_sent) {
		m_first_gnss_tx_sent = true;
	}

	// Defensive: if m_scheduled_task wasn't set in the current code path
	// (e.g., reschedule(immediate=true) which bypasses service_next_schedule_in_ms),
	// pick a sane default based on current state instead of calling an empty
	// std::function (which throws bad_function_call).
	if (!m_scheduled_task) {
		DEBUG_WARN("LoRaTxService::service_initiate: m_scheduled_task empty — falling back to status burst");
		if (m_has_gnss_fix_since_surfacing && m_depth_pile_manager.eligible()) {
			ArgosConfig argos_config;
			configuration_store->get_argos_configuration(argos_config);
			if (argos_config.sensor_tx_enable) {
				m_scheduled_task = [this]() { process_sensor_burst(); };
			} else {
				m_scheduled_task = [this]() { process_gps_burst(); };
			}
		} else {
			m_scheduled_task = [this]() { process_status_burst(); };
		}
	}
	m_scheduled_task();
}

bool LoRaTxService::service_is_active_on_initiate() {
	return false;
}

bool LoRaTxService::service_cancel() {
	DEBUG_TRACE("LoRaTxService::service_cancel: pending=%u", m_is_tx_pending);
	bool is_pending = m_is_tx_pending;
	m_is_tx_pending = false;
	// Cancel any pending pre-warm — service is being cancelled, no next TX coming.
	system_scheduler->cancel_task(m_burst_prewarm_task);
	// Drop any pending CloudLocate-ready trigger — the TX it would have fired
	// won't happen on this cancel path.
	m_cloudlocate_ready_pending = false;
	m_device.stop_send();

	// A TX that ends without transmitting: give the depth pile back what
	// retrieve() debited before send() was ever reached. Same reasoning as
	// ArgosTxService::service_cancel(). m_inflight_gps is emptied by TxComplete,
	// so a cancel after a real transmission gives nothing back, and the eviction
	// snapshot keeps the address matching honest.
	if (is_pending && !m_inflight_reached_air && !m_inflight_gps.empty()
	    && m_depth_pile_manager.gps_evictions() == m_inflight_evictions
#if defined(LORA_LINKCHECK) && (LORA_LINKCHECK == 1)
	    // A NO_FIX marker replaced in place keeps its slot's address, so after any
	    // GNSS log the addresses may name a newer entry: same rule as apply_link_check.
	    && m_pile_generation == m_inflight_generation
#endif
	) {
		const unsigned int n = m_depth_pile_manager.refund_gps(m_inflight_gps);
		if (n) {
			DEBUG_WARN("LoRaTxService: TX ended before reaching the air — %u depth-pile credit(s) given back", n);
		}
	}
#if defined(LORA_MOTION_EXT) && (LORA_MOTION_EXT == 1)
	// A frame that reached the air can still see its TX_DONE after this cancel
	// (the safety-net timeout firing mid-transmit): keep its count for that late
	// TxComplete. A frame that never went out reported nothing.
	if (!m_inflight_reached_air) m_motion_in_flight = 0;
#endif
#if defined(LORA_LINKCHECK) && (LORA_LINKCHECK == 1)
	m_inflight_encoded.clear();
#endif
	m_inflight_gps.clear();
	m_inflight_reached_air = false;

	return is_pending;
}

unsigned int LoRaTxService::service_next_timeout() {
	// Last-resort safety net. It must outlast every timeout the driver applies
	// within one dispatch, or it pre-empts the driver's own recovery instead of
	// backing it up. Worst-case chain for a single service_initiate():
	//   power_on  ~6 s   (1 s boot wait + up to 8 AT ping retries)
	//   configure ~5 s   (16 steps, plus a 3 s reboot when AT+NWM changes)
	//   joining   146 s  (JOIN_WINDOW_MS, derived from JOIN_ATTEMPTS/INTERVAL in
	//                     lora_rak3172.hpp; OTAA only, and capped at one window
	//                     per packet by m_join_attempted)
	//   transmit  150 s  (lora_rak3172.cpp tx_timeout_ms[DR0])
	// ≈ 307 s at DR0, ≈ 182 s at the DR3 default. 360 s clears both.
	// LORA_LINKCHECK builds add the AT+LINKCHECK round-trip (2 s at worst) and the
	// post-TX_DONE listening window (LINKCHECK_WAIT_MS, 3 s): ≈ 312 s, still clear.
	//
	// Was 300 s, sized against the old flat 90 s join timeout. That timeout was
	// shorter than the module's own 8-attempt cycle and has been corrected to
	// cover it, which pushed the worst-case chain past 300 s — so this net had
	// to move with it or it would start pre-empting the join it supervises.
	//
	// The previous 60 s was shorter than the DR0 and DR1 TX windows on their
	// own. What that costs, precisely: firing mid-TX runs service_cancel() →
	// stop_send(), which clears m_packet_buffer while the module is still
	// transmitting and m_state stays `transmit`. TxComplete then lands on a
	// de-initiated service ("completed without being initiated"), and — the
	// part that actually loses data — the reschedule that follows calls send()
	// again, which early-returns on `m_state == transmit` and drops the new
	// packet with nothing but a DEBUG_WARN. The service is left with no
	// completion at all for that cycle.
	//
	// Not a factor here: an AT+SEND deferred by ETSI duty-cycle enforcement is
	// bounded by send_AT's own 2 s command timeout if the module withholds OK,
	// or by initiate_timeout(tx_timeout_ms[dr]) if it accepts and defers. This
	// net never arbitrates that.
	return 360000;
}

bool LoRaTxService::service_is_triggered_on_surfaced(bool &immediate) {
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);
	// In SURFACING_BURST mode, reschedule immediately on surfacing
	immediate = (argos_config.mode == BaseArgosMode::SURFACING_BURST);
	return true;
}

void LoRaTxService::notify_peer_event(ServiceEvent &e) {
#if defined(LORA_MOTION_EXT) && (LORA_MOTION_EXT == 1)
	// MOTION block: every accelerometer wake-up counts, whatever the moored
	// classifier makes of it (hold-off, burst window, already under way).
	if (e.event_source == ServiceIdentifier::AXL_SENSOR && e.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
		const auto *sensor = std::get_if<ServiceSensorData>(&e.event_data);
		if (sensor && sensor->port[AXLSensorPort::WAKEUP_TRIGGERED]) {
			if (m_motion_wakeups < 0xFF) m_motion_wakeups++;
			m_motion_last_wakeup_rtc = service_is_time_known() ? service_current_time() : 0;
		}
	}
#endif

	// During SURFACING_BURST status phase, CloudLocate/Fastloc/NO_FIX entries are already
	// sent directly in process_status_burst() — skip depth pile to avoid double transmission.
	bool skip_depth_pile = false;
	if (m_is_surfacing_burst && !m_has_gnss_fix_since_surfacing && e.event_source == ServiceIdentifier::GNSS_SENSOR
	    && e.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
		GPSLogEntry &gps = std::get<GPSLogEntry>(e.event_data);
		if (gps.info.event_type == GPSEventType::CLOUDLOCATE || gps.info.event_type == GPSEventType::FASTLOC
		    || gps.info.event_type == GPSEventType::NO_FIX) {
			skip_depth_pile = true;
		}
	}

	if (!skip_depth_pile) {
		m_depth_pile_manager.notify_peer_event(e);
#if defined(LORA_LINKCHECK) && (LORA_LINKCHECK == 1)
		if (e.event_source == ServiceIdentifier::GNSS_SENSOR && e.event_type == ServiceEventType::SERVICE_LOG_UPDATED)
			m_pile_generation++;
#endif
	}

	if (e.event_source == ServiceIdentifier::GNSS_SENSOR && e.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
		GPSLogEntry &entry = std::get<GPSLogEntry>(e.event_data);
		ArgosConfig argos_config;
		configuration_store->get_argos_configuration(argos_config);

		// Real GPS fix: purge degraded entries and handle phase transition
		if (entry.info.valid && entry.info.event_type != GPSEventType::FASTLOC) {
			unsigned int purged = m_depth_pile_manager.purge_non_fix_entries();
			if (purged) {
				DEBUG_INFO("LoRaTxService::notify_peer_event: purged %u non-fix entries from depth pile", purged);
			}

			if (argos_config.mode == BaseArgosMode::SURFACING_BURST) {
				if ((m_is_surfacing_burst || m_awaiting_surfacing) && !m_has_gnss_fix_since_surfacing) {
					DEBUG_INFO(
					    "LoRaTxService::SURFACING_BURST: GNSS fix after %u status messages — switching to GPS phase",
					    m_status_burst_count);
					m_has_gnss_fix_since_surfacing = true;
					m_awaiting_surfacing = false;
					m_first_gnss_tx_sent = false;

					// Arm cooldown if trigger mode is END_OF_DOPPLER (status burst
					// ends naturally because a GNSS fix arrived) — parity with Argos.
					// Guard against a delayed fix arriving during an already-active
					// cooldown (rare race: GPS in flight when cooldown started +
					// surface bounce sets m_is_surfacing_burst).
					unsigned int trigger =
					    configuration_store->read_param<unsigned int>(ParamID::COOLDOWN_TRIGGER_MODE);
					if (trigger == (unsigned int)BaseCooldownTrigger::END_OF_DOPPLER && !m_cooldown_armed
					    && !ServiceManager::is_in_cooldown(service_current_time())) {
						m_cooldown_armed = true;
						DEBUG_INFO("LoRaTxService: cooldown armed (END_OF_DOPPLER, GNSS fix)");
					}
					service_reschedule();
				}
			} else {
				// LEGACY / DUTY_CYCLE: GNSS fix → TX immediately, reset TR_NOM timer
				DEBUG_INFO("LoRaTxService: GNSS fix — reschedule for immediate TX");
				m_is_first_tx = true;
				service_reschedule();
			}
		}
	} else if (e.event_source == ServiceIdentifier::UW_SENSOR
	           && e.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
		ArgosConfig argos_config;
		configuration_store->get_argos_configuration(argos_config);
		if (std::get<bool>(e.event_data) == true) {
			// Activate cooldown on dive if armed during this surfacing session
			// (parity with ArgosTxService). The cooldown timer starts now; the
			// interval `MIN_SURFACE_CYCLE_INTERVAL_S` is measured from this point.
			if (m_cooldown_armed) {
				ServiceManager::set_cycle_complete(service_current_time());
				m_cooldown_armed = false;
			}

			// Dive: reset surfacing burst state
			if (argos_config.mode == BaseArgosMode::SURFACING_BURST) {
				m_is_surfacing_burst = false;
				m_awaiting_surfacing = false;
				m_status_burst_count = 0;
				m_has_gnss_fix_since_surfacing = false;
				m_first_gnss_tx_sent = false;
			}

			// Underwater power management:
			//   - In cooldown (MIN_SURFACE_CYCLE_INTERVAL_S > 0 AND still
			//     within the window): keep the module off for the whole dive;
			//     no TX will fire until the window expires, so warming the
			//     radio would just waste the dive's budget.
			//   - Not in cooldown (either disabled or expired): warm up the
			//     module now. Boot + configure + (OTAA) join runs during the
			//     dive so the next surface burst dispatches the first Doppler
			//     in <10 ms (fast standby wake) instead of ~3 s (cold boot) —
			//     matches user requirement "don't lose time on first fix".
			// NOTE: `ServiceManager::is_in_cooldown` returns false when
			// `MIN_SURFACE_CYCLE_INTERVAL_S == 0`, so we don't need a separate
			// "disabled" check. It also correctly reports the just-set cycle
			// as in-cooldown (now == last_cycle → elapsed=0 < interval), so
			// set-then-check works in the same tick.
			bool in_cooldown = ServiceManager::is_in_cooldown(service_current_time());
			if (in_cooldown) {
				DEBUG_INFO("LoRaTxService: dive + cooldown active — keeping module off");
				m_device.power_off_immediate();  // idempotent if already off
				// Schedule a delayed warm-up at cooldown end so the module is
				// configured + in standby before the next surfacing. Priority:
				// fast first doppler TX on every surfacing, regardless of
				// whether the previous cycle left a cooldown active.
				reschedule_cooldown_warm_up();
			} else {
				DEBUG_INFO("LoRaTxService: dive, no cooldown — warming up module for next surface");
				m_device.warm_up_for_tx();
			}
		} else {
			// Surface: a genuinely new transmission opportunity — the tag may
			// now be under a gateway it was not under during the last dive. Clear
			// the device-error suspension so a dive that burned all three strikes
			// does not carry its penalty into this surface. Without this the
			// counter only ever cleared on service_init or a successful TX, so a
			// latched turtle stayed silent for the whole session even though it
			// kept surfacing.
			if (m_consecutive_device_errors) {
				DEBUG_INFO("LoRaTxService: surface — clearing %u device error(s), TX re-armed",
				           m_consecutive_device_errors);
				m_consecutive_device_errors = 0;
				m_device_error_suspend_until = 0;
			}

			// Any pending "cooldown-end warm-up" task from a prior
			// dive is left armed — if the cooldown expires while the user is
			// passively surfacing (no TX allowed yet), the task will warm up
			// the module exactly when the cooldown ends so that the moment a
			// TX becomes permitted the first dispatch is fast. Task is
			// idempotent (warm_up_for_tx is a no-op on a running module).
			// Known limit (review 2026-09): this bound only gates decisions that
			// go through the scheduler. The immediate paths — reschedule(true)
			// from GNSS_CLOUDLOCATE_READY, and the surfacing trigger's
			// immediate=true which fires status ping #1 — short-circuit in
			// Service::reschedule and never consult it. So ping #1 goes out at
			// surface regardless of dry time; pings #2+ and the fix-driven first
			// TX honour it. Gating the immediate paths means touching the shared
			// Service base (Argos first-TX-fast rides the same short-circuit) —
			// not done on purpose.
			std::time_t earliest_schedule = service_current_time() + argos_config.dry_time_before_tx;
			m_sched.set_earliest_schedule(earliest_schedule);

			// Arm cooldown immediately if trigger mode is AT_SURFACE (parity with Argos).
			// Skip arming if a cooldown is already active — otherwise a passive
			// surface bounce during cooldown would re-arm m_cooldown_armed, and
			// the next dive would call set_cycle_complete(now) which resets the
			// cooldown timer, extending it indefinitely under repeated bounces.
			unsigned int trigger = configuration_store->read_param<unsigned int>(ParamID::COOLDOWN_TRIGGER_MODE);
			if (trigger == (unsigned int)BaseCooldownTrigger::AT_SURFACE
			    && !ServiceManager::is_in_cooldown(service_current_time())) {
				m_cooldown_armed = true;
				DEBUG_INFO("LoRaTxService: cooldown armed (AT_SURFACE)");
			}

			// Only enter surfacing burst state if cooldown is not active —
			// otherwise the base class will skip reschedule and no TX fires,
			// so logging "starting status burst" would be misleading.
			if (argos_config.mode == BaseArgosMode::SURFACING_BURST
			    && !ServiceManager::is_in_cooldown(service_current_time())) {
				m_is_surfacing_burst = true;
				m_awaiting_surfacing = false;
				m_status_burst_count = 0;
				m_has_gnss_fix_since_surfacing = false;
				m_first_gnss_tx_sent = false;
				m_is_first_tx = true;
				// Reset CloudLocate-ready trigger — fresh surface, GPS will
				// re-emit GNSS_CLOUDLOCATE_READY when it captures the first raw.
				m_cloudlocate_ready_pending = false;
				// `Service::notify_peer_event` below will trigger `reschedule(true)`
				// (immediate=true for SURFACING_BURST). When immediate=true the base
				// class SKIPS `service_next_schedule_in_ms()` — which is where
				// `m_scheduled_task` is normally assigned. Pre-assign here so
				// `service_initiate()` doesn't call an empty std::function and
				// throw `bad_function_call`.
				m_scheduled_task = [this]() { process_status_burst(); };
				DEBUG_INFO("LoRaTxService::SURFACING_BURST: surface detected — starting status burst");
			}
		}
	}

	// CloudLocate-ready notification from GPS: GPS captured its first raw
	// measurement mid-acquisition. If we're in SURFACING_BURST Phase 1 (status
	// pings) and haven't fired a CloudLocate-quality TX yet, reschedule
	// immediately. process_status_burst() will then take the CloudLocate
	// branch (m_status_burst_count > 0, has_raw_measurement() true) and send
	// the CloudLocate payload instead of a plain status. This shortcuts the
	// "1st ping is status because raw isn't ready" fallback we'd otherwise hit.
	//
	// Edge case: if a TX is already in flight (`m_is_tx_pending == true`), we
	// can't reschedule synchronously — we'd corrupt the FSM. Instead we set
	// `m_cloudlocate_ready_pending` and let `react(KineisEventTxComplete)`
	// trigger the immediate reschedule when the current TX finishes. This
	// guarantees the "immediately after" semantics even when raw arrives mid-TX.
	if (e.event_source == ServiceIdentifier::GNSS_SENSOR && e.event_type == ServiceEventType::GNSS_CLOUDLOCATE_READY) {
		if (!m_is_surfacing_burst || m_has_gnss_fix_since_surfacing) {
			DEBUG_TRACE("LoRaTxService::notify_peer_event: GNSS_CLOUDLOCATE_READY but not in burst phase 1");
		} else if (m_is_tx_pending) {
			DEBUG_INFO("LoRaTxService::notify_peer_event: GNSS_CLOUDLOCATE_READY during in-flight TX — deferring to TX "
			           "complete");
			m_cloudlocate_ready_pending = true;
		} else {
			DEBUG_INFO("LoRaTxService::notify_peer_event: GNSS_CLOUDLOCATE_READY — rescheduling early CloudLocate TX");
			m_scheduled_task = [this]() { process_status_burst(); };
			service_reschedule(true);
			return;  // Skip base reschedule below — we just did it.
		}
	}

	Service::notify_peer_event(e);
}

unsigned int LoRaTxService::get_max_payload_bytes() {
#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
	unsigned int dr = configuration_store->read_param<unsigned int>(ParamID::LORA_DR);
#else
	unsigned int dr = 0;
#endif
	return LoRaPayloadLimits::max_payload_for_dr((uint8_t)dr);
}

void LoRaTxService::process_gps_burst() {
	DEBUG_TRACE("LoRaTxService::process_gps_burst");

	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);

	unsigned int max_payload = get_max_payload_bytes();
#if defined(LORA_MOTION_EXT) && (LORA_MOTION_EXT == 1)
	// The MOTION block rides after the GPS frame: size the frame against what is left.
	max_payload -= LoRaPacketBuilder::MOTION_EXT_BYTES;
#endif
	unsigned int max_entries = LoRaPacketBuilder::max_gps_entries(max_payload);

	// Retrieve GPS entries from depth pile, using the LoRa-specific per-slot cap
	// (Argos default is 3 to fit the LDA2 24-byte frame; LoRa can hold many more).
	std::vector<GPSLogEntry *> v =
	    m_depth_pile_manager.retrieve_gps((unsigned int)argos_config.depth_pile, max_entries);
	// Note the debit as soon as it happens: retrieve() has already spent a credit
	// on every entry, and LoRa reaches m_device.send() through several branches.
	// Recording here rather than at each send covers them all, and service_cancel()
	// hands the credits back if the frame never reaches the air.
	m_inflight_gps = v;
	m_inflight_reached_air = false;
	m_inflight_evictions = m_depth_pile_manager.gps_evictions();
#if defined(LORA_LINKCHECK) && (LORA_LINKCHECK == 1)
	m_inflight_encoded.clear();
	m_inflight_generation = m_pile_generation;
#endif

	if (v.size()) {
		KineisPacket packet;
		unsigned int size_bits;

		// CloudLocate entries: send as dedicated CloudLocate packet
		if (v.back()->info.event_type == GPSEventType::CLOUDLOCATE) {
			const uint8_t *overlay = reinterpret_cast<const uint8_t *>(&v.back()->info.lon);
			uint8_t format_id = overlay[0];
			const uint8_t *blob = &overlay[1];
			unsigned int blob_size = (format_id == (uint8_t)BaseCloudLocateFormat::MEASC12) ? 12 : 20;
			packet = LoRaPacketBuilder::build_cloudlocate_packet(
			    blob, blob_size, format_id, argos_config.is_lb, v.back()->info.batt_voltage, size_bits,
			    (uint32_t)convert_epochtime(v.back()->header.year, v.back()->header.month, v.back()->header.day,
				                            v.back()->header.hours, v.back()->header.minutes,
				                            v.back()->header.seconds));
#if defined(LORA_LINKCHECK) && (LORA_LINKCHECK == 1)
			m_inflight_encoded.assign(1, v.back());
#endif
			// Fastloc entries: send as unified sensor packet (GPS + fastloc quality metadata)
		} else if (v.back()->info.event_type == GPSEventType::FASTLOC) {
			packet = LoRaPacketBuilder::build_sensor_packet(v.back(), nullptr, nullptr, nullptr, nullptr, nullptr,
			                                                argos_config.is_out_of_zone, argos_config.is_lb, size_bits);
#if defined(LORA_MOTION_EXT) && (LORA_MOTION_EXT == 1)
			append_motion_ext(packet, size_bits);
#endif
#if defined(LORA_LINKCHECK) && (LORA_LINKCHECK == 1)
			m_inflight_encoded.assign(1, v.back());
#endif
		} else {
			// Filter out any CloudLocate/fastloc entries that may be mixed in
			v.erase(std::remove_if(v.begin(), v.end(),
			                       [](const GPSLogEntry *e) {
				                       return e->info.event_type == GPSEventType::CLOUDLOCATE
				                              || e->info.event_type == GPSEventType::FASTLOC;
			                       }),
			        v.end());
			if (v.empty()) {
				DEBUG_WARN("LoRaTxService::process_gps_burst: all entries filtered (mixed types)");
				m_is_tx_pending = false;
				service_complete();
				return;
			}
			// Trim to max entries that fit in payload
			if (v.size() > max_entries) v.resize(max_entries);
#if defined(LORA_LINKCHECK) && (LORA_LINKCHECK == 1)
			// build_gps_packet encodes the oldest of these, up to what its count field holds.
			const std::size_t encodable = (1U << LoRaPacketBuilder::BITS_GPS_COUNT) - 1U;
			m_inflight_encoded.assign(v.begin(), v.begin() + std::min(v.size(), encodable));
#endif

			packet = LoRaPacketBuilder::build_gps_packet(v, argos_config.is_out_of_zone, argos_config.is_lb,
			                                             max_payload, size_bits);
#if defined(LORA_MOTION_EXT) && (LORA_MOTION_EXT == 1)
			append_motion_ext(packet, size_bits);
#endif
		}

		// Demoted to TRACE: per-TX payload dump on the burst hot path (~50-300 ms
		// LFS commit per emit). TX completion event already logs the outcome.
		DEBUG_TRACE("LoRaTxService::process_gps_burst: data=%s sz=%u bits", Binascii::hexlify(packet).c_str(),
		            size_bits);
		m_last_tx_had_gps = true;
		m_device.send(KineisModulation::LDA2, packet, size_bits);
	} else {
		DEBUG_WARN("LoRaTxService::process_gps_burst: no eligible entries in depth pile");
		if (m_is_surfacing_burst || m_has_gnss_fix_since_surfacing) {
			DEBUG_INFO("LoRaTxService::process_gps_burst: ending GNSS phase (depth pile exhausted)");
			m_is_surfacing_burst = false;
			m_has_gnss_fix_since_surfacing = false;
			m_first_gnss_tx_sent = false;
			m_awaiting_surfacing = true;
		}
		m_is_tx_pending = false;
		service_complete();
	}
}

void LoRaTxService::process_sensor_burst() {
	DEBUG_TRACE("LoRaTxService::process_sensor_burst");

	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);

	GPSLogEntry *gps = m_depth_pile_manager.retrieve_gps_single((unsigned int)argos_config.depth_pile);
	// Same debit, one entry: retrieve_gps_single() goes through retrieve(depth, 1).
	m_inflight_gps.clear();
	if (gps != nullptr) m_inflight_gps.push_back(gps);
	m_inflight_reached_air = false;
	m_inflight_evictions = m_depth_pile_manager.gps_evictions();
#if defined(LORA_LINKCHECK) && (LORA_LINKCHECK == 1)
	// Sensor bursts keep today's accounting: their sensor piles are not refunded.
	m_inflight_encoded.clear();
	m_inflight_generation = m_pile_generation;
#endif

	if (gps != nullptr) {
		// CloudLocate entries: send as dedicated CloudLocate packet
		if (gps->info.event_type == GPSEventType::CLOUDLOCATE) {
			const uint8_t *overlay = reinterpret_cast<const uint8_t *>(&gps->info.lon);
			uint8_t format_id = overlay[0];
			const uint8_t *blob = &overlay[1];
			unsigned int blob_size = (format_id == (uint8_t)BaseCloudLocateFormat::MEASC12) ? 12 : 20;
			unsigned int size_bits;
			KineisPacket packet = LoRaPacketBuilder::build_cloudlocate_packet(
			    blob, blob_size, format_id, service_is_battery_level_low(), gps->info.batt_voltage, size_bits,
			    (uint32_t)convert_epochtime(gps->header.year, gps->header.month, gps->header.day, gps->header.hours,
				                            gps->header.minutes, gps->header.seconds));
			// Demoted to TRACE: per-TX payload dump (~50-300 ms LFS commit).
			DEBUG_TRACE("LoRaTxService::process_sensor_burst: CloudLocate fmt=%u data=%s", format_id,
			            Binascii::hexlify(packet).c_str());
			m_last_tx_had_gps = true;
			m_device.send(KineisModulation::LDA2, packet, size_bits);
			return;
		}

		// Fastloc entries are handled transparently by build_sensor_packet
		// (the fastloc flag bit is set automatically when event_type == FASTLOC)
		unsigned int size_bits;
		KineisPacket packet = LoRaPacketBuilder::build_sensor_packet(
		    gps,
		    m_depth_pile_manager.retrieve_sensor_single((unsigned int)argos_config.depth_pile,
			                                            ServiceIdentifier::ALS_SENSOR),
		    m_depth_pile_manager.retrieve_sensor_single((unsigned int)argos_config.depth_pile,
			                                            ServiceIdentifier::PH_SENSOR),
		    m_depth_pile_manager.retrieve_sensor_single((unsigned int)argos_config.depth_pile,
			                                            ServiceIdentifier::PRESSURE_SENSOR),
#ifdef BOARD_RSPB
		    m_depth_pile_manager.retrieve_sensor_single((unsigned int)argos_config.depth_pile,
			                                            ServiceIdentifier::THERMISTOR_SENSOR),
#else
		    m_depth_pile_manager.retrieve_sensor_single((unsigned int)argos_config.depth_pile,
			                                            ServiceIdentifier::SEA_TEMP_SENSOR),
#endif
#if ENABLE_AXL_SENSOR
		    m_depth_pile_manager.retrieve_sensor_single((unsigned int)argos_config.depth_pile,
			                                            ServiceIdentifier::AXL_SENSOR),
#else
		    nullptr,
#endif
		    argos_config.is_out_of_zone, argos_config.is_lb, size_bits);
#if defined(LORA_MOTION_EXT) && (LORA_MOTION_EXT == 1)
		append_motion_ext(packet, size_bits);
#endif

		// Demoted to TRACE: per-TX payload dump (~50-300 ms LFS commit).
		DEBUG_TRACE("LoRaTxService::process_sensor_burst: data=%s sz=%u bits", Binascii::hexlify(packet).c_str(),
		            size_bits);
		m_last_tx_had_gps = true;
		m_device.send(KineisModulation::LDA2, packet, size_bits);
	} else {
		DEBUG_WARN("LoRaTxService::process_sensor_burst: no eligible entries in depth pile");
		if (m_is_surfacing_burst || m_has_gnss_fix_since_surfacing) {
			DEBUG_INFO("LoRaTxService::process_sensor_burst: ending GNSS phase (depth pile exhausted)");
			m_is_surfacing_burst = false;
			m_has_gnss_fix_since_surfacing = false;
			m_first_gnss_tx_sent = false;
			m_awaiting_surfacing = true;
		}
		m_is_tx_pending = false;
		service_complete();
	}
}

void LoRaTxService::process_status_burst() {
	DEBUG_TRACE("LoRaTxService::process_status_burst");

	service_update_battery();
	unsigned int size_bits;

	// Progressive CloudLocate: when running a SURFACING_BURST and raw GNSS
	// measurements are available, replace the status payload with a CloudLocate
	// packet. The `m_status_burst_count > 0` check gates legacy LEGACY/DOPPLER
	// modes (where the counter is never incremented), not the first ping of
	// a burst.
	unsigned int fastloc_mode = configuration_store->read_param<unsigned int>(ParamID::GNSS_FASTLOC_MODE);
	if (fastloc_mode == (unsigned int)BaseFastlocMode::CLOUDLOCATE && m_status_burst_count > 0 && gps_device
	    && gps_device->has_raw_measurement()) {
		GNSSRawMeasurement raw = gps_device->get_raw_measurement();
		unsigned int cl_format = configuration_store->read_param<unsigned int>(ParamID::GNSS_CLOUDLOCATE_FORMAT);

		const uint8_t *blob = nullptr;
		unsigned int blob_size = 0;
		uint8_t format_id = 0;

		// STRICT format policy (2026-06): use ONLY the operator-configured format
		// (MEAS50 is available live on LoRa). No cross-format fallback — if the
		// configured format wasn't produced, blob stays null and the CL TX is
		// skipped, rather than silently emitting a different format.
		if (cl_format == (unsigned int)BaseCloudLocateFormat::MEAS50 && raw.has_meas50) {
			blob = raw.meas50;
			blob_size = 50;
			format_id = (uint8_t)BaseCloudLocateFormat::MEAS50;
		} else if (cl_format == (unsigned int)BaseCloudLocateFormat::MEAS20 && raw.has_meas20) {
			blob = raw.meas20;
			blob_size = 20;
			format_id = (uint8_t)BaseCloudLocateFormat::MEAS20;
		} else if (cl_format == (unsigned int)BaseCloudLocateFormat::MEASC12 && raw.has_measc12) {
			blob = raw.measc12;
			blob_size = 12;
			format_id = (uint8_t)BaseCloudLocateFormat::MEASC12;
		}
		if (!blob) {
			DEBUG_WARN("LoRaTxService::process_status_burst: configured CloudLocate format %u not available "
			           "(measc12=%u meas20=%u meas50=%u) — skipping CL TX",
			           cl_format, (unsigned)raw.has_measc12, (unsigned)raw.has_meas20, (unsigned)raw.has_meas50);
		}

		if (blob) {
			// Size vs DR is guaranteed at boot by LoRaDevice::load_config_from_store()
			// which forces DR ≥ 3 when MEAS50 is selected. MEAS20/MEASC12 fit any DR.
			KineisPacket packet =
			    LoRaPacketBuilder::build_cloudlocate_packet(blob, blob_size, format_id, service_is_battery_level_low(),
				                                            service_get_voltage(), size_bits, raw.capture_time);
			// Demoted to TRACE: per-ping payload dump (~50-300 ms LFS commit).
			DEBUG_TRACE("LoRaTxService::process_status_burst: CLOUDLOCATE #%u fmt=%u sz=%u data=%s",
			            m_status_burst_count, format_id, blob_size, Binascii::hexlify(packet).c_str());
			m_last_tx_had_gps = true;
			m_device.send(KineisModulation::LDA2, packet, size_bits);
			return;
		}
	}

	// Progressive fastloc: when running a SURFACING_BURST and the GPS has a
	// degraded PVT available, replace the status payload with a fastloc sensor
	// packet. As above, `m_status_burst_count > 0` gates legacy modes, not the
	// first ping of a burst. Same logic as Argos process_doppler_burst().
	if (fastloc_mode >= (unsigned int)BaseFastlocMode::DEGRADED_PVT && m_status_burst_count > 0 && gps_device
	    && gps_device->has_degraded_pvt()) {
		GNSSData degraded = gps_device->get_degraded_pvt();

		// Build a temporary GPSLogEntry from the degraded PVT
		GPSLogEntry fastloc_entry{};
		fastloc_entry.header.log_type = LOG_GPS;
		fastloc_entry.info.lat = degraded.lat;
		fastloc_entry.info.lon = degraded.lon;
		fastloc_entry.info.height = degraded.height;
		fastloc_entry.info.hMSL = degraded.hMSL;
		fastloc_entry.info.hAcc = degraded.hAcc;
		fastloc_entry.info.vAcc = degraded.vAcc;
		fastloc_entry.info.velN = degraded.velN;
		fastloc_entry.info.velE = degraded.velE;
		fastloc_entry.info.velD = degraded.velD;
		fastloc_entry.info.gSpeed = degraded.gSpeed;
		fastloc_entry.info.headMot = degraded.headMot;
		fastloc_entry.info.sAcc = degraded.sAcc;
		fastloc_entry.info.headAcc = degraded.headAcc;
		fastloc_entry.info.pDOP = degraded.pDOP;
		fastloc_entry.info.vDOP = degraded.vDOP;
		fastloc_entry.info.hDOP = degraded.hDOP;
		fastloc_entry.info.headVeh = degraded.headVeh;
		fastloc_entry.info.fixType = degraded.fixType;
		fastloc_entry.info.numSV = degraded.numSV;
		fastloc_entry.info.ttff = degraded.ttff;
		fastloc_entry.info.onTime = degraded.ttff;
		fastloc_entry.info.batt_voltage = service_get_voltage();
		fastloc_entry.info.schedTime = service_current_time();
		fastloc_entry.info.valid = true;
		fastloc_entry.info.event_type = GPSEventType::FASTLOC;

		KineisPacket packet =
		    LoRaPacketBuilder::build_sensor_packet(&fastloc_entry, nullptr, nullptr, nullptr, nullptr, nullptr, false,
			                                       service_is_battery_level_low(), size_bits);
#if defined(LORA_MOTION_EXT) && (LORA_MOTION_EXT == 1)
		append_motion_ext(packet, size_bits);
#endif

		// Demoted to TRACE: per-ping payload dump (~50-300 ms LFS commit).
		DEBUG_TRACE("LoRaTxService::process_status_burst: FASTLOC #%u hAcc=%um numSV=%u data=%s", m_status_burst_count,
		            degraded.hAcc, degraded.numSV, Binascii::hexlify(packet).c_str());
		m_last_tx_had_gps = true;
		m_device.send(KineisModulation::LDA2, packet, size_bits);
		return;
	}

	// Priority 3: Most recent cached position between last GPS fix and last
	// Fastloc / degraded PVT (any past surface). Used when no live CloudLocate
	// raw or degraded PVT is available this surface. Keeps every ping carrying
	// a position rather than burning TX on pure status.
	//
	// "Most recent wins" comparison uses LogHeader UTC timestamp set at entry
	// creation (service_set_log_header_time). build_sensor_packet handles
	// FASTLOC vs FIX transparently via gps->info.event_type.
	//
	// `m_status_burst_count > 0` gates the legacy LEGACY/DUTY_CYCLE/DOPPLER
	// modes, where the counter is never incremented (it only advances inside a
	// SURFACING_BURST phase-1) — so on a periodic tracker this branch was
	// unreachable and every heartbeat fell through to STATUS-PURE, i.e. battery
	// volts and nothing else.
	//
	// That is exactly wrong for a moored vessel: the depth pile is empty
	// precisely BECAUSE the GNSS cadence was stretched, and the one thing the
	// operator wants from an hourly heartbeat is "still here, at this position".
	// So allow the cached position through when the moored classifier is
	// engaged and MOORED_TX_LAST_POS is set. Both are false by default, so no
	// existing LoRa deployment changes behaviour, and the frame is the same
	// SENSOR packet the backend already decodes.
	bool moored_heartbeat =
	    MooredModeService::is_moored() && configuration_store->read_param<bool>(ParamID::MOORED_TX_LAST_POS);
	if (m_status_burst_count > 0 || moored_heartbeat) {
		const GPSLogEntry &cached_gps = configuration_store->get_last_gps_entry();
		const GPSLogEntry &cached_fl = configuration_store->get_last_fastloc_entry();
		bool gps_ok = (cached_gps.info.valid && cached_gps.info.event_type == GPSEventType::FIX);
		bool fl_ok = (cached_fl.info.valid && cached_fl.info.event_type == GPSEventType::FASTLOC);

		const GPSLogEntry *pick = nullptr;
		if (gps_ok && fl_ok) {
			std::time_t t_gps =
			    convert_epochtime(cached_gps.header.year, cached_gps.header.month, cached_gps.header.day,
				                  cached_gps.header.hours, cached_gps.header.minutes, cached_gps.header.seconds);
			std::time_t t_fl =
			    convert_epochtime(cached_fl.header.year, cached_fl.header.month, cached_fl.header.day,
				                  cached_fl.header.hours, cached_fl.header.minutes, cached_fl.header.seconds);
			pick = (t_fl > t_gps) ? &cached_fl : &cached_gps;
		} else if (gps_ok) {
			pick = &cached_gps;
		} else if (fl_ok) {
			pick = &cached_fl;
		}

		if (pick) {
			GPSLogEntry entry = *pick;
			entry.info.batt_voltage = service_get_voltage();  // freshen battery field
			KineisPacket packet = LoRaPacketBuilder::build_sensor_packet(
			    &entry, nullptr, nullptr, nullptr, nullptr, nullptr, false, service_is_battery_level_low(), size_bits);
#if defined(LORA_MOTION_EXT) && (LORA_MOTION_EXT == 1)
			append_motion_ext(packet, size_bits);
#endif
			// Demoted to TRACE: per-ping payload dump (~50-300 ms LFS commit).
			DEBUG_TRACE("LoRaTxService::process_status_burst: %s #%u lat=%lf lon=%lf data=%s",
			            (pick->info.event_type == GPSEventType::FIX) ? "CACHED_GPS" : "CACHED_FASTLOC",
			            m_status_burst_count, entry.info.lat, entry.info.lon, Binascii::hexlify(packet).c_str());
			m_last_tx_had_gps = true;
			m_device.send(KineisModulation::LDA2, packet, size_bits);
			return;
		}
	}

	// Priority 5: Pure status fallback — no live CloudLocate/degraded PVT and
	// no cached position exist anywhere. Send one status ping then silence
	// further pings this surface (continuing progressive status spam when we
	// can never carry a position is wasteful). The next surfacing event will
	// re-arm the burst.
	KineisPacket packet =
	    LoRaPacketBuilder::build_status_packet(service_get_voltage(), service_is_battery_level_low(), size_bits);
#if defined(LORA_MOTION_EXT) && (LORA_MOTION_EXT == 1)
	append_motion_ext(packet, size_bits);
#endif

	DEBUG_INFO(
	    "LoRaTxService::process_status_burst: STATUS-PURE #%u data=%s sz=%u bits (no cache, silencing further pings)",
	    m_status_burst_count, Binascii::hexlify(packet).c_str(), size_bits);
	m_last_tx_had_gps = false;
	m_device.send(KineisModulation::LDA2, packet, size_bits);

	// Silence the burst after this status-pure ping. Mirror the cooldown
	// arming used by the max-msg-reached branch in service_next_schedule_in_ms.
	if (m_is_surfacing_burst && !m_has_gnss_fix_since_surfacing) {
		unsigned int trigger = configuration_store->read_param<unsigned int>(ParamID::COOLDOWN_TRIGGER_MODE);
		if (trigger == (unsigned int)BaseCooldownTrigger::END_OF_DOPPLER && !m_cooldown_armed) {
			m_cooldown_armed = true;
			DEBUG_INFO("LoRaTxService: cooldown armed (END_OF_DOPPLER, status-pure no cache)");
		}
		m_is_surfacing_burst = false;
		m_awaiting_surfacing = true;
	}
}

void LoRaTxService::react(KineisEventTxStarted const &) {
	DEBUG_TRACE("LoRaTxService::react: KineisEventTxStarted");
	// On air: the try is genuinely used, there is nothing to give back.
	m_inflight_reached_air = true;
	service_active();
}

void LoRaTxService::react([[maybe_unused]] KineisEventTxComplete const &e) {
	DEBUG_TRACE("LoRaTxService::react: KineisEventTxComplete");
#if defined(LORA_LINKCHECK) && (LORA_LINKCHECK == 1)
	apply_link_check(e.link_check);
#endif
	// Transmitted: the credits paid for a real emission. Drop the note so a later
	// cancel cannot hand them back.
	m_inflight_gps.clear();
	m_inflight_reached_air = false;
	m_is_tx_pending = false;
	m_consecutive_device_errors = 0;
#if defined(LORA_MOTION_EXT) && (LORA_MOTION_EXT == 1)
	bool motion_delivered = true;
#if defined(LORA_LINKCHECK) && (LORA_LINKCHECK == 1)
	// No gateway heard that frame: the wake-ups it reported reached nobody and ride again.
	motion_delivered = (e.link_check != 1);
#endif
	// The wake-ups that frame reported are delivered; keep any counted since it was built.
	if (motion_delivered) m_motion_wakeups = (uint8_t)(m_motion_wakeups - std::min(m_motion_wakeups, m_motion_in_flight));
	m_motion_in_flight = 0;
#endif

	// Increment TX counter
	configuration_store->increment_tx_counter();
	m_session_tx_count++;

	// Update last TX date time
	std::time_t t = service_current_time();
	configuration_store->write_param(ParamID::LAST_TX, t);

	// Counters updated in RAM — flash persistence deferred to periodic flush / powerdown

	// Check session TX limit
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);
	if (argos_config.shutdown_ntime_sat > 0 && m_session_tx_count >= argos_config.shutdown_ntime_sat) {
#if defined(BOARD_RSPB)
		DEBUG_ERROR("LoRaTxService: session TX budget reached (%u/%u) — powering down ON PURPOSE. The TPL5111 duty "
		            "cycle restores power and the counter resets on the next boot.",
		            m_session_tx_count, argos_config.shutdown_ntime_sat);
		configuration_store->save_params();  // Flush before shutdown
		PMU::powerdown();
		return;
#else
		// RSPB only, same reasoning as ArgosTxService: m_session_tx_count is
		// reset in service_init and nowhere else, so on a board that stays
		// powered a per-session budget silently becomes a one-shot lifetime cap.
		if (!m_ntime_sat_ignored_logged) {
			m_ntime_sat_ignored_logged = true;
			DEBUG_WARN("LoRaTxService: session TX budget reached (%u/%u) but SHUTDOWN_NTIME_SAT only applies to RSPB, "
			           "whose duty cycle power-cycles the board. Ignored here. Transmission continues.",
			           m_session_tx_count, argos_config.shutdown_ntime_sat);
		}
#endif
	}

	// Cooldown arming based on trigger mode — parity with ArgosTxService.
	// The cooldown timer actually starts on the next UW event (dive), not here.
	// Cooldown guard: skip re-arming if a cooldown is already active — a TX
	// firing during cooldown (only possible in DUTY_CYCLE / LEGACY modes;
	// SURFACING_BURST is already gated) would re-set m_cooldown_armed, and
	// the next dive's set_cycle_complete(now) would reset the cooldown timer,
	// creeping it forward by one full interval per cycle. Parity with the
	// AT_SURFACE / END_OF_DOPPLER branches handled in notify_peer_event.
	{
		unsigned int trigger = configuration_store->read_param<unsigned int>(ParamID::COOLDOWN_TRIGGER_MODE);
		bool cooldown_active = ServiceManager::is_in_cooldown(service_current_time());
		if (trigger == (unsigned int)BaseCooldownTrigger::AFTER_LAST_TX) {
			// Mode 3: arm on every GNSS or Doppler (status-burst) TX.
			// Bool arming is idempotent — the effective anchor is the dive event.
			// Log only on first transition per cycle to avoid spam on long bursts.
			bool qualifies_gnss = m_last_tx_had_gps;
			bool qualifies_dopper = m_is_surfacing_burst && !m_last_tx_had_gps;
			if ((qualifies_gnss || qualifies_dopper) && !m_cooldown_armed && !cooldown_active) {
				m_cooldown_armed = true;
				DEBUG_INFO("LoRaTxService: cooldown armed (AFTER_LAST_TX, reason=%s)",
				           qualifies_gnss ? "GNSS" : "DOPPLER");
			} else if ((qualifies_gnss || qualifies_dopper) && !cooldown_active) {
				// Subsequent qualifying TX — keep armed, refresh effective arm reason at TRACE.
				DEBUG_TRACE("LoRaTxService: cooldown re-armed (AFTER_LAST_TX, reason=%s)",
				            qualifies_gnss ? "GNSS" : "DOPPLER");
			}
		} else if (trigger == (unsigned int)BaseCooldownTrigger::AFTER_FIRST_GNSS) {
			// Mode 2: arm after first GNSS TX only
			if (m_last_tx_had_gps && !m_cooldown_armed && !cooldown_active) {
				m_cooldown_armed = true;
				DEBUG_INFO("LoRaTxService: cooldown armed (AFTER_FIRST_GNSS)");
			}
		}
		// Modes 0 (AT_SURFACE) and 1 (END_OF_DOPPLER) handled in notify_peer_event
	}

	m_sched.notify_tx_complete();
	service_complete();

	// Surface-idle power management: if the surfacing burst is complete and
	// we're waiting for the next surfacing cycle (m_awaiting_surfacing was
	// just set by service_next_schedule_in_ms), cut module power.
	//
	// IMPORTANT: this react() runs synchronously from inside the LoRa device
	// FSM's state_transmit(), which after returning here will still execute
	// LORA_STATE_CHANGE(transmit, idle). If we called power_off_immediate
	// synchronously it would set m_state=power_off, but the FSM would then
	// overwrite it with idle → standby, leaving the driver thinking it's in
	// standby while SAT_PWR_EN is already low — the next start_device would
	// try to wake via AT ping on a powered-off module, fail, and cold-boot
	// (measured: +7 s on first-TX after surface). Defer to the scheduler so
	// the power-off runs after the FSM finishes its transmit→idle→standby
	// walk; from standby, power_off_immediate cleanly moves the state to
	// power_off and the subsequent warm-up/dive paths see the correct state.
	if (m_awaiting_surfacing) {
		DEBUG_INFO("LoRaTxService: burst complete — scheduling LoRa module power-off (surface idle)");
		system_scheduler->post_task_prio(
		    [this]() {
			    DEBUG_INFO("LoRaTxService: powering off LoRa module (post-burst, deferred)");
			    m_device.power_off_immediate();
		    },
		    "LoRaPostBurstOff", Scheduler::DEFAULT_PRIORITY, 500);
		// Burst is ending — make sure no pre-warm fires after the rail is cut,
		// and drop any pending CloudLocate-ready trigger (no next TX coming).
		system_scheduler->cancel_task(m_burst_prewarm_task);
		m_cloudlocate_ready_pending = false;
	} else if (m_cloudlocate_ready_pending) {
		// Edge case: GNSS_CLOUDLOCATE_READY arrived during the TX we just
		// completed. service_complete() already scheduled the next TX at the
		// normal burst interval (e.g. +5 s). Override that — fire the next
		// TX immediately so the CloudLocate goes out right away, as the
		// user expects, instead of waiting for the normal timer tick.
		DEBUG_INFO("LoRaTxService: consuming pending CloudLocate-ready trigger — rescheduling immediate");
		m_cloudlocate_ready_pending = false;
		m_scheduled_task = [this]() { process_status_burst(); };
		service_reschedule(true);
		// Skip pre-warm: we just rescheduled to fire ASAP, no time to pre-warm
		// anyway. Module wake from standby/power_off happens inside m_kineis.send().
	} else {
		// Burst continues normally. In lp_mode=0, schedule a pre-warm so the
		// next TX fires on time (otherwise the 2.5 s boot stacks on the user
		// interval). No-op in lp_mode=1 (standby keeps the module ready).
		schedule_burst_prewarm();
	}
}

/// @brief (Re)schedule the cooldown-end warm-up task.
/// Cancels any previously scheduled warm-up, then — if the device is off and
/// we're actually in an active cooldown window — schedules a task that fires
/// when the cooldown expires. The task drives the module through power_on →
/// configure → standby so the first TX of the next surfacing is fast.
void LoRaTxService::reschedule_cooldown_warm_up() {
	system_scheduler->cancel_task(m_cooldown_warm_up_task);

	unsigned int remaining_s = ServiceManager::get_cooldown_remaining_s(service_current_time());
	if (remaining_s == 0) {
		return;  // No active cooldown — nothing to schedule.
	}

	DEBUG_INFO("LoRaTxService: scheduling module warm-up in %u s (cooldown end)", remaining_s);
	m_cooldown_warm_up_task = system_scheduler->post_task_prio(
	    [this]() {
		    DEBUG_INFO("LoRaTxService: cooldown expired — warming up LoRa module for next surface");
		    m_device.warm_up_for_tx();
	    },
	    "LoRaCooldownWarmUp", Scheduler::DEFAULT_PRIORITY, remaining_s * 1000);
}

/// @brief Arm intra-burst pre-warm task for next TX (lp_mode=0 only).
///
/// In lp_mode=0 (shutdown), the LoRa device's state_idle transitions to
/// state_power_off after every TX (rail cut, 0 µA). The next TX would then
/// pay a ~2.5 s cold-boot penalty stacked on top of the user's interval
/// (interval becomes user_interval + 2.5 s).
///
/// To preserve the user-intended burst timing, we schedule a pre-warm task
/// at (next_TX_time - BURST_PRE_WARM_DURATION_MS). The task calls
/// warm_up_for_tx() which boots the module asynchronously; by the time
/// service_initiate fires m_kineis.send(), the module is in standby and
/// the wake is ~10 ms.
///
/// No-op when lp_mode=1 (standby is already ready), when not in
/// SURFACING_BURST mode, when burst is ending, or when the computed
/// pre-warm delay would be negative (next TX too soon).
void LoRaTxService::schedule_burst_prewarm() {
	system_scheduler->cancel_task(m_burst_prewarm_task);

	// Only pre-warm in shutdown mode — standby already keeps the module ready.
	unsigned int lp_mode = configuration_store->read_param<unsigned int>(ParamID::LORA_LP_MODE);
	if (lp_mode != 0) return;

	// Only pre-warm during active SURFACING_BURST.
	if (!m_is_surfacing_burst || m_awaiting_surfacing) return;

	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);
	if (argos_config.mode != BaseArgosMode::SURFACING_BURST) return;

	// Compute the interval to the NEXT TX, mirroring service_next_schedule_in_ms.
	// `m_status_burst_count` at this point has already been incremented for the
	// TX that just completed; service_next_schedule_in_ms will use the same
	// counter to compute (count) * step, which is the interval to TX N+1.
	unsigned int interval_s =
	    argos_config.surfacing_burst_init_s + m_status_burst_count * argos_config.surfacing_burst_step_s;
	if (interval_s > argos_config.surfacing_burst_max_s) interval_s = argos_config.surfacing_burst_max_s;

	unsigned int interval_ms = interval_s * 1000U;
	if (interval_ms <= BURST_PRE_WARM_DURATION_MS) {
		// Interval shorter than boot budget — can't compensate, next TX will
		// just be late by ~2.5 s. Better than failing to schedule pre-warm.
		DEBUG_TRACE("LoRaTxService: skipping pre-warm — interval %u ms < %u ms budget", interval_ms,
		            BURST_PRE_WARM_DURATION_MS);
		return;
	}

	unsigned int prewarm_delay_ms = interval_ms - BURST_PRE_WARM_DURATION_MS;
	DEBUG_INFO("LoRaTxService: scheduling pre-warm in %u ms (next TX in %u s, budget %u ms)", prewarm_delay_ms,
	           interval_s, BURST_PRE_WARM_DURATION_MS);
	m_burst_prewarm_task = system_scheduler->post_task_prio(
	    [this]() {
		    DEBUG_INFO("LoRaTxService: pre-warm — booting LoRa module for next burst TX");
		    m_device.warm_up_for_tx();
	    },
	    "LoRaBurstPreWarm", Scheduler::DEFAULT_PRIORITY, prewarm_delay_ms);
}

void LoRaTxService::react(KineisEventDeviceError const &) {
	// H3 fix: only count/act on a DeviceError from an IN-FLIGHT TX. service_cancel()
	// returns whether a TX was pending (and clears the flag). A DeviceError raised
	// during a warm-up/join with no TX pending must NOT ratchet
	// m_consecutive_device_errors — otherwise repeated warm-up failures across dives
	// climb to DEVICE_ERROR_MAX_CONSECUTIVE and service_initiate permanently suspends
	// the service without ever attempting a TX (the counter only resets on a
	// successful TxComplete or service_init).
	if (!service_cancel()) {
		DEBUG_TRACE("LoRaTxService::react: KineisEventDeviceError (no TX pending — not counted)");
		return;
	}
	m_consecutive_device_errors++;
	DEBUG_WARN("LoRaTxService::react: KineisEventDeviceError (consecutive=%u/%u)", m_consecutive_device_errors,
	           DEVICE_ERROR_MAX_CONSECUTIVE);
	{
		if (DEVICE_ERROR_PROBE_PERIOD_S && m_consecutive_device_errors >= DEVICE_ERROR_MAX_CONSECUTIVE) {
			m_device_error_suspend_until = service_current_time() + DEVICE_ERROR_PROBE_PERIOD_S;
			DEBUG_ERROR("LoRaTxService: %u consecutive device errors — suspending TX for %u s, "
			            "then one probe (a surface event clears it sooner)",
			            m_consecutive_device_errors, DEVICE_ERROR_PROBE_PERIOD_S);
			service_complete(nullptr, nullptr, false);  // no reschedule
		} else {
			// Exponential backoff: 1 min, 2 min, 4 min... saturating at
			// DEVICE_ERROR_BACKOFF_MAX_MS (10 min).
			//
			// Doubling with a cap-bounded loop rather than `BASE_MS << (n - 1)`:
			// the shift overflows unsigned int long before the cap below can act
			// on it, and an overflowed backoff wraps to a SMALL value -- with this
			// 60 s base it returns exactly 0 from the 28th consecutive error, i.e.
			// retry immediately, on a beacon that has just failed twenty-eight
			// times in a row. The exact inversion of what a backoff is for.
			//
			// Unlike ArgosTxService, this is not a latent problem here: the
			// suspension above is conditional on DEVICE_ERROR_PROBE_PERIOD_S, so
			// with LORA_TX_ERROR_SUSPEND_S=0 the counter climbs without bound and
			// this branch keeps being taken. The shift was already reachable.
			//
			// The loop produces the identical sequence (verified against the
			// shift for every n it can represent), so nothing is shortened; it
			// simply cannot wrap, and it needs no magic bound tied to the
			// current value of BASE_MS.
			unsigned int backoff_ms = Backoff::doubling_capped(
			    DEVICE_ERROR_BACKOFF_BASE_MS, DEVICE_ERROR_BACKOFF_MAX_MS, m_consecutive_device_errors);
			DEBUG_WARN("LoRaTxService: backoff %u ms before next TX attempt", backoff_ms);
			m_sched.set_earliest_schedule(service_current_time() + backoff_ms / 1000);
			service_complete();
		}
	}
}

#if defined(LORA_MOTION_EXT) && (LORA_MOTION_EXT == 1)
void LoRaTxService::append_motion_ext(KineisPacket &packet, unsigned int &size_bits) {
	const std::time_t now = service_is_time_known() ? service_current_time() : 0;
	LoRaMotionExt motion{};
	motion.moored = MooredModeService::is_moored();
	motion.axl_holdoff = (now != 0) && MooredModeService::axl_holdoff_active(now);
	motion.wakeups = m_motion_wakeups;
	motion.min_since_wakeup = LoRaPacketBuilder::encode_minutes_since(now, m_motion_last_wakeup_rtc);
	motion.min_since_axl_exit = LoRaPacketBuilder::encode_minutes_since(now, MooredModeService::last_axl_exit_rtc());
	LoRaPacketBuilder::append_motion_ext(packet, size_bits, motion);
	// Nothing is normally in flight here. If a frame that reached the air was
	// cancelled and its TX_DONE may still land, keep the smaller count, so that
	// late TxComplete cannot consume wake-ups this frame reports but the old one
	// never carried: at worst some are reported twice, never lost.
	m_motion_in_flight = m_motion_in_flight ? std::min(m_motion_in_flight, motion.wakeups) : motion.wakeups;
	DEBUG_TRACE("LoRaTxService: MOTION moored=%u holdoff=%u wakeups=%u since_wakeup=%u min since_axl_exit=%u min",
	            (unsigned)motion.moored, (unsigned)motion.axl_holdoff, (unsigned)motion.wakeups,
	            (unsigned)motion.min_since_wakeup, (unsigned)motion.min_since_axl_exit);
}
#endif

#if defined(LORA_LINKCHECK) && (LORA_LINKCHECK == 1)
/// @brief Settle the positions of the frame that just completed on the network's
/// verdict, before react() drops the in-flight note.
///
/// Every credit retrieve() took is given back first, then:
/// - heard (0): every position the frame carried is spent, whatever credits NTRY
///   left on it -- the network has them. Positions retrieved but left out of the
///   frame (the 4-bit count field, at DR4/DR5) keep theirs;
/// - no answer (1): nothing more. No gateway heard the frame, or its answer was
///   lost on the way down: an outage spends nothing and the pile keeps the newest
///   ARGOS_DEPTH_PILE positions for when coverage returns. A downlink that never
///   gets through costs what an outage costs -- full frames, repeats the IHM merges.
///
/// Anything uncertain keeps today's accounting, one credit per frame:
///  - no verdict (-1);
///  - a frame that never reached TxStarted: a late TX_DONE after a cancel would
///    otherwise settle a batch that never left the device;
///  - a pile evicted or reshaped while in flight: the pointers may name other
///    positions;
///  - NTRY_PER_MESSAGE 0, which already means "replay until evicted";
///  - sensor TX enabled: the sensor piles are not refunded alongside the GPS one.
void LoRaTxService::apply_link_check(int8_t link_check) {
	const std::vector<GPSLogEntry *> encoded = std::move(m_inflight_encoded);
	m_inflight_encoded.clear();
	if (link_check < 0 || !m_inflight_reached_air || m_inflight_gps.empty()
	    || m_depth_pile_manager.gps_evictions() != m_inflight_evictions || m_pile_generation != m_inflight_generation) {
		return;
	}
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);
	if (argos_config.ntry_per_message == 0 || argos_config.sensor_tx_enable != 0) return;

	[[maybe_unused]] const unsigned int kept = m_depth_pile_manager.refund_gps(m_inflight_gps);
	if (link_check == 1) {
		DEBUG_TRACE("LoRaTxService: no LinkCheck answer - %u position(s) kept for the next frame", kept);
		return;
	}
	[[maybe_unused]] const unsigned int heard = m_depth_pile_manager.debit_gps_extra(encoded, UINT_MAX);
	DEBUG_TRACE("LoRaTxService: uplink heard - %u position(s) delivered", heard);
}
#endif
