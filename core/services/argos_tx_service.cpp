/**
 * @file argos_tx_service.cpp
 * @brief Argos TX service — scheduling, burst preparation, TX event handling.
 */

#include <climits>
#include <cstdint>
#include <algorithm>

#include "argos_tx_service.hpp"
#include "gps.hpp"
#include "messages.hpp"
#include "timeutils.hpp"
#include "exponential_backoff.hpp"
#include "binascii.hpp"
#include "debug.hpp"
#include "pmu.hpp"
#include "rate_limiter.hpp"
#include "rgb_led.hpp"

// VLDA4 is not permitted on the KIM2 backend.
//
// The module gates it behind a 27 dBm regulatory check and refuses any send()
// that asks for it (kim2.cpp, KIM2Device::send). That refusal arrives as a
// KineisEventDeviceError and is indistinguishable from a real transmit fault,
// so three of them suspend TX for an hour -- over and over, on a condition that
// will never change on its own. Deciding it at selection time instead means the
// modulation is simply never chosen, and the fallbacks that already exist route
// around it. SMD carries no such restriction.
#if !defined(ARGOS_SMD) || (ARGOS_SMD != 1)
static constexpr bool VLDA4_PERMITTED = false;  // KIM2
#else
static constexpr bool VLDA4_PERMITTED = true;  // SMD
#endif

// Pre-deploy validation channel — see hauled_mode_service.cpp header comment.
// Enables grep-friendly [VAL-TX] tags on every TX completion with type + spacing
// from previous [VAL-TX]. Critical for short-surface Doppler validation campaigns.
#ifndef VALIDATION_LOG_ENABLE
#define VALIDATION_LOG_ENABLE 0
#endif
extern ConfigurationStore *configuration_store;
extern Scheduler *system_scheduler;
extern GPSDevice *gps_device;
extern RGBLed *status_led;
#if defined(BOARD_RSPB) && ENABLE_MORTALITY_SENSOR
#include "mortality_service.hpp"

extern MortalityService *mortality_service;
#endif


/// @brief Report a duty-cycle schedule that came back unusable, then pass it on.
///
/// ArgosTxScheduler::INVALID_SCHEDULE is numerically SCHEDULE_DISABLED, so
/// returning it straight from schedule_duty_cycle() silently switched the
/// beacon off with no log line at all. The dominant cause is an all-zero hour
/// mask -- which is the FACTORY DEFAULT of both DUTY_CYCLE (ARP18) and
/// LB_ARGOS_DUTY_CYCLE (LBP05): selecting ARGOS_MODE=DUTY_CYCLE without also
/// setting a mask mutes the tag, and nothing said so. The behaviour is
/// deliberately unchanged -- an explicit mask is the operator's decision -- but
/// the log now names the reason.
/// @brief Turn a scheduler's answer into a decision.
///
/// ArgosTxScheduler::INVALID_SCHEDULE and Service::SCHEDULE_DISABLED are the
/// same number, so "I could not compute a slot" and "I am switched off" used to
/// be the same answer -- and the service layer treated both as permanent. That
/// is the path the zero duty-cycle mask took all the way to a beacon that never
/// transmitted again. A failed computation is a reason to look again later, not
/// a reason to stop, so it becomes a bounded hold.
static ScheduleDecision from_scheduler(unsigned int ms, const char *ran_why, const char *failed_why) {
	if (ms == ArgosTxScheduler::INVALID_SCHEDULE) return ScheduleDecision::hold_for_event(600, failed_why);
	return ScheduleDecision::run(ms, ran_why);
}

static unsigned int report_duty_cycle_schedule(unsigned int schedule, const ArgosConfig &cfg) {
	if (schedule != ArgosTxScheduler::INVALID_SCHEDULE) return schedule;

	// A zero mask no longer reaches here: ArgosTxScheduler::schedule_periodic
	// treats "no hour permitted" as no restriction and transmits on the nominal
	// period instead, with its own DEBUG_ERROR naming the parameter. What is
	// left is a non-zero mask whose enabled hours the period cannot land on.
	DEBUG_WARN("ArgosTxService: no duty-cycle slot found within 24 h "
	           "(mask=0x%06X, tx_interval=%u s) — no transmission scheduled",
	           cfg.duty_cycle & 0xFFFFFF, cfg.tx_interval_s);
	return schedule;
}

ArgosTxService::ArgosTxService(KineisDevice &device)
    : Service(ServiceIdentifier::ARGOS_TX, "ARGOSTX"),
      m_kineis(device) {}

/// @brief Init: subscribe to KineisDevice events, load config, set TCXO/LPM.
void ArgosTxService::service_init() {
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);

	m_kineis.subscribe(*this);
	m_kineis.set_tcxo_warmup_time(argos_config.argos_tcxo_warmup_time);

	// 2026-08 — seed the scheduler's location from the last PERSISTED fix,
	// instead of waiting for the next one.
	//
	// ArgosTxScheduler's `m_location` is an optional reset to empty at every
	// construction, and it was only fed by notify_peer_event, hence only by a
	// fix obtained AFTER startup. Consequence: after a reset, schedule_prepass
	// bailed out on "no known GPS location, cannot predict passes — TX
	// disabled until next GPS fix" and the beacon stayed mute in
	// PASS_PREDICTION, while its last known position had been available in
	// flash the whole time. A position from the day before is still largely
	// usable to predict satellite visibility.
	{
		const GPSLogEntry &last_gps = configuration_store->get_last_gps_entry();
		if (last_gps.info.valid) {
			m_sched.set_last_location(last_gps.info.lon, last_gps.info.lat);
			DEBUG_INFO("ArgosTxService: prepass position seeded from the persisted last fix (lon=%f lat=%f)",
			           last_gps.info.lon, last_gps.info.lat);
		} else {
			DEBUG_INFO("ArgosTxService: no persisted position — prepass unavailable until the first fix");
		}
	}

	// Set SMD LPM mode from configuration (written to SMD at every boot via SPI)
	uint8_t lpm = static_cast<uint8_t>(configuration_store->read_param<unsigned int>(ParamID::SMD_LPM_MODE));
	m_kineis.set_lpm_mode(lpm);

#if !defined(ARGOS_SMD) || (ARGOS_SMD != 1)
	// KIM2 ONLY (not compiled for SMD): seed the modulation cache from the live
	// radio at startup so the FIRST non-adaptive TX is sized for the master
	// RCONF's true modulation — even on a fresh / stale-cache module (avoids the
	// aborted first TX). NON-ADAPTIVE only: in adaptive mode the service drives
	// the modulation per cycle via switch_modulation(), so the cache is not used
	// for frame sizing and must not be force-seeded here.
	if (!argos_config.adaptive_modulation) {
		m_kineis.resync_rconf_cache();
	}
#endif

	// Warn if SURFACING_BURST mode is configured without underwater detection
	if (argos_config.mode == BaseArgosMode::SURFACING_BURST && !argos_config.underwater_en) {
		DEBUG_WARN(
		    "ArgosTxService: SURFACING_BURST mode requires UNDERWATER_EN=1 — burst will not trigger without SWS");
	}

	// Position-less: LEGACY and DUTY_CYCLE with GNSS_EN=0 fall back to a
	// Doppler-only TX (no position on air, except a cached fix via REUSE_LAST).
	if ((argos_config.mode == BaseArgosMode::LEGACY || argos_config.mode == BaseArgosMode::DUTY_CYCLE)
	    && !argos_config.gnss_en) {
		DEBUG_WARN("ArgosTxService: %s with GNSS_EN=0 — TX will be Doppler-only without position",
		           argos_config.mode == BaseArgosMode::LEGACY ? "LEGACY" : "DUTY_CYCLE");
	}

	// PASS_PREDICTION + GNSS_EN=0 is an INCOMPATIBLE combination, not a
	// degraded mode: pass prediction needs a position to compute, and
	// service_next_schedule_in_ms() has NO branch for that case — it falls
	// through to a SCHEDULE_DISABLED that was silent. The beacon therefore
	// stayed mute, without the slightest trace. We flag it loudly here; the
	// blocking is done upstream, on the configuration interface side.
	if (argos_config.mode == BaseArgosMode::PASS_PREDICTION && !argos_config.gnss_en) {
		DEBUG_ERROR("ArgosTxService: INCOMPATIBLE CONFIGURATION — PASS_PREDICTION requires GNSS_EN=1 "
		            "(pass prediction is computed from a position). NO transmission will be "
		            "scheduled while this combination is in place.");
		if (status_led) status_led->flash(RGBLedColor::RED, 200);
	}

	// BLIND: say once what the operator actually gets. Asking for it and
	// silently not getting it is the failure mode this reports — and the
	// aligned period matters too, because in DOPPLER what goes on air is the
	// sequence's cadence, not ARP46.
	if (configuration_store->read_param<bool>(ParamID::ARGOS_BLIND_EN)) {
		if (argos_config.mode == BaseArgosMode::SURFACING_BURST) {
			DEBUG_WARN("ArgosTxService: BLIND requested but REFUSED in SURFACING_BURST — that mode paces its own "
			           "Doppler cascade and GNSS phase, so a module-owned burst would transmit every message "
			           "twice. Transmitting in BASIC MAC.");
		} else if (argos_config.mode == BaseArgosMode::DOPPLER && !argos_config.blind_en) {
			DEBUG_WARN("ArgosTxService: BLIND requested but REFUSED in DOPPLER — the sequence settles at "
			           "SURFACING_BURST_MAX_S=%u s, below the %u s the module needs between two repeats. Raise "
			           "SURFACING_BURST_MAX_S to at least %u s, or leave BLIND off. Transmitting in BASIC MAC.",
			           argos_config.surfacing_burst_max_s, ArgosConfig::BLIND_MIN_RETX_PERIOD_S,
			           ArgosConfig::BLIND_MIN_RETX_PERIOD_S);
		} else if (argos_config.blind_en) {
			DEBUG_INFO("ArgosTxService: BLIND active — retx_nb=%u period=%u s%s", argos_config.blind_retx_nb,
			           argos_config.blind_retx_period_s,
			           argos_config.mode == BaseArgosMode::DOPPLER
			               ? " (aligned on SURFACING_BURST_MAX_S, not ARGOS_BLIND_RETX_PERIOD_S)"
			               : "");
		}
	}

	DEBUG_INFO("ArgosTxService::service_init: Argos ID=%u", (unsigned int)argos_config.argos_id);
	// Jitter seed: shared derivation with LoRaTxService (see
	// ConfigurationStore::get_tx_jitter_seed) so both services jitter off the
	// same per-unit value, and a build that provisions no Argos ID still gets
	// a unique one instead of mt19937(0).
	m_sched.reset(configuration_store->get_tx_jitter_seed());
	m_depth_pile_manager.clear();
	m_is_first_tx = true;
	m_is_tx_pending = false;
	m_tcxo_skip_on_next_tx = false;
	m_session_tx_count = 0;
	m_is_surfacing_burst = false;
	m_doppler_burst_count = 0;
	m_has_gnss_fix_since_surfacing = false;
	m_first_gnss_tx_sent = false;
	m_last_tx_had_gps = false;
	m_cooldown_armed = false;
	m_doppler_seq_count = 0;
	m_doppler_pause_until_rtc = 0;
	m_last_preconfig_mod = KineisModulation::LDA2;
	m_modulation_preconfig.reset();
	m_consecutive_device_errors = 0;
	m_device_error_hold_until = 0;
	m_is_underwater = false;
	m_prepared_doppler_packet.clear();
	m_prepared_doppler_size_bits = 0;
	m_prepared_at_ms = 0;

	// Snapshot which per-mod RCONFs are provisioned. Used by adaptive bursts
	// to skip a TX cleanly when the fallback modulation can't hold the packet
	// — better than KIM2's silent payload-too-long drop + 30 s service timeout.
	refresh_modulation_availability();
}

/// @brief Terminate: power off device immediately.
void ArgosTxService::service_term() {
	m_kineis.unsubscribe(*this);
	// FIX 2026-05-23 (audit boot finding 5): mirror GPSService R2 — cut the
	// Kineis (SMD/KIM) power state unconditionally on teardown. Without this,
	// a service_term firing during an in-flight TX could leave the radio
	// rail in an indeterminate state (e.g., SMD STM32WL still in TX phase
	// when the host service drops its KineisEventListener subscription).
	// The radio device's own cleanup is best-effort but not guaranteed
	// idempotent under all race conditions. set_idle_timeout(0) tells the
	// driver to fall through to power-off as soon as the current TX (if any)
	// finishes, instead of holding the rail for the configured surfacing
	// burst max.
	m_kineis.set_idle_timeout(0);
	m_kineis.power_off_immediate();
	// Reset session state so a subsequent service_init starts clean (covers
	// the rare path where service_term + service_init fire back-to-back
	// without a full FSM teardown, e.g., DFU rollback).
	m_is_surfacing_burst = false;
	m_awaiting_surfacing = false;
	m_doppler_burst_count = 0;
	m_first_gnss_tx_sent = false;
	m_has_gnss_fix_since_surfacing = false;
	m_cooldown_armed = false;
	m_doppler_seq_count = 0;
	m_doppler_pause_until_rtc = 0;
	m_is_tx_pending = false;
	// Defensive: clear pre-warm state so a hypothetical service_term-without-
	// service_init sequence doesn't leak stale prep across the next session.
	m_is_underwater = false;
	m_prepared_doppler_packet.clear();
	m_prepared_doppler_size_bits = 0;
	m_prepared_at_ms = 0;
}

/// @brief Enabled if Argos mode is not OFF (respects cert TX override).
bool ArgosTxService::service_is_enabled() {
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);
	return (argos_config.mode != BaseArgosMode::OFF || argos_config.cert_tx_enable);
}

/// @brief Compute next TX schedule based on mode (cert/legacy/duty/prepass/surfacing).
/// @return Delay in ms until next TX, or SCHEDULE_DISABLED if TX is off.
/// @brief Legacy entry point, superseded by service_next_schedule().
/// Still required while the base declares it pure — deliberately, so that a new
/// service implementing neither cannot compile and be silently off for ever.
unsigned int ArgosTxService::service_next_schedule_in_ms() {
	return SCHEDULE_DISABLED;
}

ScheduleDecision ArgosTxService::service_next_schedule() {
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);
	std::time_t now = service_current_time();

	DEBUG_TRACE("ArgosTxService::service_next_schedule_in_ms");

	// Cooldown gate (2026-05): refuse to schedule any Argos TX while
	// MIN_SURFACE_CYCLE_INTERVAL_S is still running. Matches the gate added
	// to GPSService::service_next_schedule_in_ms — without it, the boot path
	// (Service::start → reschedule → here) would compute a fresh TX schedule
	// inside the cooldown window, defeating the whole cooldown mechanism on
	// any reset that lands mid-cooldown. SWS re-emits state when cooldown
	// expires and rewakes us via notify_underwater_state.
	if (ServiceManager::is_in_cooldown(now)) {
		return ScheduleDecision::hold_until(
		    (uint32_t)(now + (std::time_t)ServiceManager::get_cooldown_remaining_s(now)), "surface-cycle cooldown");
	}

	// Rolling-window rate limit (Plan 1 step 2). Applies to ALL TX cycles
	// including SURFACING_BURST's first ping — battery priority over the
	// §5.3 first-TX-fast objective, by explicit user decision. Disabled by
	// default (RATE_LIMIT_EN = false → returns false without reading config).
	{
		unsigned int reschedule_s = 0;
		if (RateLimiter::is_blocked(now, reschedule_s)) {
			DEBUG_INFO("ArgosTxService: rate limit reached, reschedule in %u s", reschedule_s);
			m_sched.schedule_at(now + (std::time_t)reschedule_s);
			// Bound the conversion: reschedule_s is 32-bit and RLP02
			// (RATE_LIMIT_WINDOW_S) accepts up to 0xFFFFFFFF, so any value above
			// 4294967 s (~49.7 days) wraps in `* 1000` and turns a very long
			// wait into a near-immediate one -- the exact inversion of what the
			// limiter is for. Clamp instead: SCHEDULE_DISABLED is 0xFFFFFFFF and
			// must stay reachable only as the "nothing planned" sentinel.
			constexpr unsigned int MAX_RESCHEDULE_MS = 0xFFFFFFFEu;
			unsigned int reschedule_ms =
			    (reschedule_s > MAX_RESCHEDULE_MS / 1000u) ? MAX_RESCHEDULE_MS : reschedule_s * 1000u;
			return ScheduleDecision::run(reschedule_ms, "rate limited");
		}
	}

	// --- Prepass gating, orthogonal to the mode (2026-08) --------------------
	// For any mode OTHER than PASS_PREDICTION (which handles its own alignment),
	// we set a "not before the next satellite window" floor through the same
	// mechanism as the SWS dry-time. Transmitting outside a window means
	// spending battery on a frame that no satellite hears.
	// Any failure (AOP missing/expired, no computable pass, window beyond
	// SAT_PREPASS_MAX_WAIT_S) still lets the mode transmit PERIODICALLY: we
	// never leave the beacon mute.
	if (argos_config.prepass_en && argos_config.mode != BaseArgosMode::PASS_PREDICTION
	    && argos_config.mode != BaseArgosMode::OFF) {
		unsigned int age_s = 0;
		std::time_t aos = 0;
		AopStatus etat = aop_status(argos_config, now, age_s);
		if (etat != AopStatus::USABLE) {
			DEBUG_WARN("ArgosTxService: prepass requested but %s (age=%u s) — falling back to periodic TX",
			           aop_status_text(etat), age_s);
		} else {
			BasePassPredict &pp = configuration_store->read_pass_predict();
			aos = m_sched.next_pass_epoch(argos_config, pp, now);
			if (aos <= 0) {
				DEBUG_WARN("ArgosTxService: prepass requested but no pass is computable — falling back to periodic TX");
			} else if (argos_config.prepass_max_wait_s
			           && aos > now + static_cast<std::time_t>(argos_config.prepass_max_wait_s)) {
				DEBUG_INFO("ArgosTxService: next window in %lld s > max wait %u s — falling back to periodic TX",
				           static_cast<long long>(aos - now), argos_config.prepass_max_wait_s);
			} else {
				DEBUG_INFO("ArgosTxService: TX aligned on the next satellite window (in %lld s)",
				           static_cast<long long>(aos - now));
				m_sched.set_earliest_schedule(aos);
			}
		}
		refresh_prepass_status(argos_config, now, aos);
	}

	// Refresh provisioned-modulation mask in case PARMW edited an RCONF since
	// the last cycle. Cheap (3 string-length checks); only logs on change.
	refresh_modulation_availability();

	// Critical battery check: immediate powerdown, no transmission
	if (argos_config.is_lb) {
		service_update_battery();
		unsigned int critical_level = configuration_store->read_param<unsigned int>(ParamID::LB_CRITICAL_THRESH);
		unsigned int current_soc = service_get_level();
		if (current_soc < critical_level) {
			DEBUG_INFO("ArgosTxService: CRITICAL battery SOC %u%% < %u%% - shutdown", current_soc, critical_level);
			configuration_store->save_params();
			PMU::powerdown();
			return ScheduleDecision::off("battery critical — powering down");
		}
	}

	// Certification TX. This is a bench/lab mode: the operator sets a fixed
	// payload and modulation and the beacon repeats it, so it deliberately
	// bypasses the ARGOS_MODE dispatch below -- CERT_TX_ENABLE is what makes
	// service_is_enabled() return true even when ARGOS_MODE is OFF, and
	// get_gnss_configuration() turns GNSS off while it is set.
	//
	// It still sits AFTER the critical-battery and rate-limit gates above: a
	// certification run must not be the thing that flattens a cell.
	if (argos_config.cert_tx_enable) {
		m_scheduled_mode = (KineisModulation)argos_config.cert_tx_modulation;
		m_scheduled_task = [this]() { process_certification_burst(); };

		// First TX fires immediately so the operator sees something at once;
		// afterwards, one every CERT_TX_REPETITION seconds.
		unsigned int period_s = m_is_first_tx ? 0 : argos_config.cert_tx_repetition;
		m_sched.schedule_at(now + (std::time_t)period_s);

		// Same bound as the rate limiter above, for the same reason: CTP04
		// accepts up to 0xFFFFFFFF, and any value over 4294967 s (~49.7 days)
		// wraps in `* 1000` and turns a very long wait into a near-immediate
		// one. SCHEDULE_DISABLED is 0xFFFFFFFF and must stay reachable only as
		// the "nothing planned" sentinel.
		constexpr unsigned int MAX_CERT_PERIOD_MS = 0xFFFFFFFEu;
		unsigned int period_ms = (period_s > MAX_CERT_PERIOD_MS / 1000u) ? MAX_CERT_PERIOD_MS : period_s * 1000u;
		DEBUG_INFO("ArgosTxService: certification TX in %u ms (mode=%s, payload=%u chars)", period_ms,
		           argos_modulation_to_string(argos_config.cert_tx_modulation),
		           (unsigned)argos_config.cert_tx_payload.size());
		return ScheduleDecision::run(period_ms, "certification burst");
	}

	if (argos_config.mode == BaseArgosMode::OFF) return ScheduleDecision::off("ARGOS_MODE=OFF");
	if (argos_config.mode == BaseArgosMode::DOPPLER) return schedule_doppler(argos_config, now);
	if (argos_config.mode == BaseArgosMode::SURFACING_BURST) return schedule_surfacing_burst(argos_config, now);
	if (!argos_config.gnss_en) return schedule_without_gnss(argos_config, now);
	if (!service_is_time_known()) {
		return ScheduleDecision::hold_for_event(600, "waiting for a fix to set the clock");
	}
	return schedule_with_gnss(argos_config, now);
}

/// @brief DOPPLER: progressive Doppler sequence, bounded by SURFACING_BURST_MAX_MSG.
/// Split out of service_next_schedule_in_ms(), which tested argos_config.mode
/// nine times in 489 lines. The gates it runs first -- cooldown, rate limit,
/// prepass, critical battery, certification -- are NOT repeated here.
ScheduleDecision ArgosTxService::schedule_doppler(ArgosConfig &argos_config, std::time_t now) {
	// DOPPLER burst pattern (2026-05): sequence of up to
	// SURFACING_BURST_MAX_MSG messages with progressive spacing
	// (surfacing_burst_init_s + (n-1)*step_s, capped at max_s). Between
	// sequences: tx_interval_s pause (0 = next sequence chains
	// immediately, effectively continuous progressive Doppler).
	// Auto-triggered, works identically with UW=0 or UW=1 (the DOPPLER
	// branch has no UW gate). Reuses surfacing_burst_* params and
	// SURFACING_BURST_MAX_MSG — no new DTE params. The SURFACING_BURST
	// branch below is untouched.
	//
	// FastLoc / GNSS auto-promotion preserved: a fresh fix (<60 s) in
	// the depth pile turns this slot into a GNSS Argos packet via
	// process_gnss_burst (caller routes by mode of the latest entry).
	if (should_promote_doppler_to_gnss(60)) {
		DEBUG_INFO("ArgosTxService::DOPPLER mode: fresh FastLoc/FIX — promoting to GNSS TX");
		m_scheduled_task = [this]() { process_gnss_burst(); };
	} else {
		m_scheduled_task = [this]() { process_doppler_burst(); };
	}
	m_scheduled_mode = argos_config.adaptive_modulation ? adaptive_doppler_modulation() : resolve_non_adaptive_modulation();

	// Inter-sequence pause guard. If a reschedule fires while we are
	// supposed to be paused (e.g., UW surfaced event, GPS log update),
	// honor the remaining pause instead of restarting the sequence.
	if (m_doppler_pause_until_rtc != 0 && now < m_doppler_pause_until_rtc) {
		unsigned int remaining_s = (unsigned int)(m_doppler_pause_until_rtc - now);
		DEBUG_TRACE("ArgosTxService::DOPPLER: in pause, %u s remaining", remaining_s);
		m_sched.schedule_at(m_doppler_pause_until_rtc);
		return ScheduleDecision::run(remaining_s * 1000, "doppler inter-sequence pause");
	}

	// This mode skips the backoff ladder on purpose (see the note further down),
	// but never the suspension. The two are not interchangeable: the ladder only
	// spaces retries, whereas the suspension is what stops a wedged radio, and
	// the guard in service_initiate() reschedules through here. Leave the
	// suspension out and that guard lands back on the burst cadence, hits the
	// guard again a few seconds later and spins -- the same wake/log/skip loop
	// the probe deadline was added to kill, only faster.
	if (m_consecutive_device_errors >= DEVICE_ERROR_MAX_CONSECUTIVE) {
		bool held = false;
		unsigned int delay_ms = apply_device_error_hold(0, now, held);
		if (held) return ScheduleDecision::run(delay_ms, "device-error hold");
	}
	m_doppler_pause_until_rtc = 0;

	unsigned int max_msg = configuration_store->read_param<unsigned int>(ParamID::SURFACING_BURST_MAX_MSG);

	// End of sequence. m_doppler_seq_count is post-incremented in
	// service_initiate, so reaching max_msg here means the Nth msg
	// already TX'd. Reset count and arm the inter-sequence pause.
	if (max_msg > 0 && m_doppler_seq_count >= max_msg) {
		unsigned int inter_s = argos_config.tx_interval_s;
		DEBUG_INFO("ArgosTxService::DOPPLER: sequence end (%u/%u), %s", m_doppler_seq_count, max_msg,
		           inter_s == 0 ? "chaining next sequence" : "pausing then next sequence");
		m_doppler_seq_count = 0;
		if (inter_s > 0) {
			m_doppler_pause_until_rtc = now + (std::time_t)inter_s;
			m_sched.schedule_at(m_doppler_pause_until_rtc);
			return ScheduleDecision::run(inter_s * 1000, "doppler sequence done — pausing");
		}
		// inter_s == 0: chain straight into next sequence — fall
		// through to the count==0 path below (which spacing-guards
		// the first msg against being too close to the last TX).
	}

	// Within sequence. count == 0 = first msg, immediate (spacing-guarded).
	if (m_doppler_seq_count == 0) {
		DEBUG_TRACE("ArgosTxService::DOPPLER: msg #1 (immediate)");
		unsigned int delay_ms = apply_spacing_guard(0, argos_config.surfacing_burst_init_s, now);
		if (delay_ms == 0) m_sched.schedule_at(now);
		return ScheduleDecision::run(delay_ms, "doppler #1");
	}

	// Subsequent msg: progressive interval capped at max_s.
	//
	// Deliberately NOT clamped by apply_device_error_hold, unlike SURFACING_BURST.
	// A backoff only earns its keep if something can end it early, and this mode
	// has nothing to offer: no surfacing event and no GNSS fix, so a hold here
	// would just push messages out of the sequence with no way to bring them
	// back. The strike counter still climbs and the suspension in
	// service_initiate() still stops a wedged radio -- what is skipped is the
	// spacing between messages of one sequence, which is the mode's own cadence.
	unsigned int interval_s =
	    argos_config.surfacing_burst_init_s + (m_doppler_seq_count - 1) * argos_config.surfacing_burst_step_s;
	if (interval_s > argos_config.surfacing_burst_max_s) interval_s = argos_config.surfacing_burst_max_s;
	DEBUG_TRACE("ArgosTxService::DOPPLER: msg #%u in %u s", m_doppler_seq_count + 1, interval_s);
	m_sched.schedule_at(now + (std::time_t)interval_s);
	return ScheduleDecision::run(interval_s * 1000, "doppler sequence");
}

/// @brief SURFACING_BURST: Doppler phase, then GNSS phase once a fix lands.
/// Split out of service_next_schedule_in_ms(), which tested argos_config.mode
/// nine times in 489 lines. The gates it runs first -- cooldown, rate limit,
/// prepass, critical battery, certification -- are NOT repeated here.
/// @brief Presence heartbeat while a surfacing burst waits for the next dive.
///
/// The three places that set m_awaiting_surfacing used to return
/// SCHEDULE_DISABLED, which leaves the service owning nothing at all. Only two
/// things clear that flag: a dive→surface transition, or a NEW VALID GPS fix. A
/// NO_FIX clears neither. So an animal that stopped diving -- hauled out,
/// stranded, a failed underwater sensor, or simply a long stay at the surface --
/// and that is not getting fixes went silent for the rest of the deployment,
/// indistinguishable on the screens from a dead radio.
///
/// A position-less Doppler is what the beacon has to say in that state, and it
/// is enough: the ground segment rebuilds a Doppler position from the reception
/// instants alone. At the NOMINAL period, deliberately, not the burst cadence --
/// SURFACING_BURST_MAX_MSG caps the fast burst at each surfacing and that cap is
/// still honoured; this is the slow heartbeat underneath it. And it costs
/// nothing while the animal is actually diving, because
/// notify_underwater_state(true) deschedules the service outright.
ScheduleDecision ArgosTxService::schedule_surfacing_heartbeat(ArgosConfig &argos_config, std::time_t now) {
	DEBUG_INFO("ArgosTxService: burst finished, waiting for the next surfacing — presence heartbeat at the nominal "
	           "period meanwhile (not a fault).");
	m_scheduled_task = [this]() { process_doppler_burst(); };
	m_scheduled_mode =
	    argos_config.adaptive_modulation ? adaptive_doppler_modulation() : resolve_non_adaptive_modulation();
	return from_scheduler(m_sched.schedule_legacy(argos_config, now), "surfacing heartbeat",
	                      "surfacing heartbeat: no slot computable");
}

ScheduleDecision ArgosTxService::schedule_surfacing_burst(ArgosConfig &argos_config, std::time_t now) {
	// 2026-05-25 modulation fix: SURFACING_BURST was unconditionally
	// hardcoded to LDA2, ignoring both `argos_config.adaptive_modulation`
	// and `resolve_non_adaptive_modulation()` (which returns the user's
	// configured ARGOS_MOD_DEFAULT, e.g. LDK).
	//
	// Symptom observed 2026-05-25: user configured LDK + adaptive=OFF.
	// SMD STM32 flash correctly held LDK (write_credentials_from_config
	// wrote + saved master RCONF, ARGOS_CACHED_MODULATION=1=LDK). But
	// every TX hit "TX mode 0 != current modulation 1 — call
	// switch_modulation() first" because m_scheduled_mode was LDA2.
	// ensure_modulation() then overwrote the saved LDK RCONF with LDA2
	// at runtime, defeating the user's config silently AND causing a
	// per-TX flash write to the STM32 (wear + latency).
	//
	// Fix: match the pattern used by DUTY_CYCLE / LEGACY / PASS_PREDICTION
	// further down. With adaptive=OFF, m_scheduled_mode now equals the
	// saved RCONF modulation → ensure_modulation() is a no-op → no
	// per-TX flash write → user's LDK config is honored AND persisted.
	m_scheduled_mode = argos_config.adaptive_modulation ? KineisModulation::LDA2 : resolve_non_adaptive_modulation();

	// Phase 1: Doppler burst with progressive intervals until GNSS fix
	if (m_is_surfacing_burst && !m_has_gnss_fix_since_surfacing) {
		// Check max Doppler message limit (0 = unlimited)
		unsigned int max_msg = configuration_store->read_param<unsigned int>(ParamID::SURFACING_BURST_MAX_MSG);
		if (max_msg > 0 && m_doppler_burst_count >= max_msg) {
			DEBUG_INFO("ArgosTxService::SURFACING_BURST: Doppler limit reached (%u/%u), stopping burst",
			           m_doppler_burst_count, max_msg);
			// Arm cooldown if trigger mode is END_OF_DOPPLER (max messages reached without fix)
			unsigned int trigger = configuration_store->read_param<unsigned int>(ParamID::COOLDOWN_TRIGGER_MODE);
			if (trigger == (unsigned int)BaseCooldownTrigger::END_OF_DOPPLER && !m_cooldown_armed) {
				m_cooldown_armed = true;
				DEBUG_INFO("ArgosTxService: cooldown armed (END_OF_DOPPLER, max msg)");
			}
			m_is_surfacing_burst = false;
			m_awaiting_surfacing = true;
			m_first_gnss_tx_sent = false;
			return schedule_surfacing_heartbeat(argos_config, now);
		}

		// FastLoc priority (2026-05): if a fresh FastLoc / FIX is in the
		// depth pile, promote to GNSS phase immediately rather than
		// wasting this slot on a position-less Doppler. Fall through
		// to phase 2 below; do NOT execute the Doppler scheduling.
		if (should_promote_doppler_to_gnss(60)) {
			DEBUG_INFO("ArgosTxService::SURFACING_BURST: fresh FastLoc/FIX in pile — promoting to GNSS phase");
			m_has_gnss_fix_since_surfacing = true;
			// Fall through to "Phase 2" below.
		} else {
			m_scheduled_task = [this]() { process_doppler_burst(); };

			// First message is immediate (0 delay) — but apply spacing
			// guard if a prior TX (e.g. from a prior session or a back-
			// to-back transition) is too recent.
			if (m_doppler_burst_count == 0) {
				DEBUG_TRACE("ArgosTxService::SURFACING_BURST: Doppler #%u (immediate)", m_doppler_burst_count + 1);
				bool held = false;
				unsigned int delay_ms = apply_spacing_guard(0, argos_config.surfacing_burst_init_s, now);
				delay_ms = apply_device_error_hold(delay_ms, now, held);
				if (delay_ms == 0) m_sched.schedule_at(now);
				return ScheduleDecision::run(delay_ms, "surfacing burst: doppler #1");
			}

			// Progressive interval: init + (count-1) * step, capped at max
			unsigned int interval_s =
			    argos_config.surfacing_burst_init_s + (m_doppler_burst_count - 1) * argos_config.surfacing_burst_step_s;
			if (interval_s > argos_config.surfacing_burst_max_s) interval_s = argos_config.surfacing_burst_max_s;

			// Demoted to TRACE: per progressive ping. Burst start/end markers
			// stay at INFO; intermediate scheduling is verbose forensics.
			DEBUG_TRACE("ArgosTxService::SURFACING_BURST: Doppler #%u in %u s", m_doppler_burst_count + 1, interval_s);
			bool held = false;
			unsigned int delay_ms = apply_device_error_hold(interval_s * 1000, now, held);
			// The helper re-anchors the scheduler itself when it clamps; only
			// anchor here when it did not. `held` says so explicitly rather than
			// inferring it from the returned value.
			if (!held) m_sched.schedule_at(now + interval_s);
			return ScheduleDecision::run(delay_ms, "surfacing burst: doppler");
		}
		// Promote branch fell through: continue to Phase 2 below.
	}

	// Phase 2: GNSS fix available — switch to normal GNSS TX with tx_interval_s
	if (m_has_gnss_fix_since_surfacing) {
		if (!service_is_time_known()) {
			return ScheduleDecision::hold_for_event(600, "surfacing burst: GNSS phase but the clock is not set");
		}
		if (m_depth_pile_manager.eligible() == 0) {
			DEBUG_TRACE("ArgosTxService::SURFACING_BURST: GNSS phase but no eligible entries");
			m_is_surfacing_burst = false;
			m_awaiting_surfacing = true;
			m_has_gnss_fix_since_surfacing = false;
			m_first_gnss_tx_sent = false;
			return schedule_surfacing_heartbeat(argos_config, now);
		}

		if (argos_config.sensor_tx_enable) {
			m_scheduled_task = [this]() { process_sensor_burst(); };
		} else {
			m_scheduled_task = [this]() { process_gnss_burst(); };
		}

		// First GNSS TX is immediate after fix, then use tx_interval_s
		// Note: m_first_gnss_tx_sent is set in service_initiate(), not here,
		// because scheduling can be called while a TX is still in progress.
		if (!m_first_gnss_tx_sent) {
			DEBUG_INFO("ArgosTxService::SURFACING_BURST: GNSS TX #1 (immediate after fix)");
			// Spacing guard (2026-05): if a Doppler TX just completed
			// seconds ago and the GPS fix arrived right after, firing
			// the first GNSS TX immediately would put 2 TX back-to-back
			// (TCXO drift + CLS rate-limit risk). Defer to at least
			// surfacing_burst_init_s after the previous TX.
			unsigned int delay_ms = apply_spacing_guard(0, argos_config.surfacing_burst_init_s, now);
			// Normally a no-op here: a GNSS fix is one of the events that clears
			// the hold, and reaching phase 2 usually means one just landed. It is
			// not guaranteed though -- should_promote_doppler_to_gnss() can enter
			// this phase off a pile entry with no fresh peer event -- so honour it.
			bool held = false;
			delay_ms = apply_device_error_hold(delay_ms, now, held);
			if (delay_ms == 0) m_sched.schedule_at(now);
			return ScheduleDecision::run(delay_ms, "surfacing burst: first GNSS TX");
		}

		// Demoted to TRACE: per Phase-2 ping. The "GNSS TX #1" INFO at burst
		// promotion already marks the entry; per-ping interval is verbose.
		DEBUG_TRACE("ArgosTxService::SURFACING_BURST: GNSS TX in %u s", argos_config.tx_interval_s);
		return from_scheduler(m_sched.schedule_legacy(argos_config, now), "surfacing burst: GNSS phase",
		                      "surfacing burst: no slot computable");
	}

	// Burst ended — wait for the next surfacing event.
	//
	// This used to return SCHEDULE_DISABLED, leaving the service owning nothing
	// at all. Only two things clear the flag: a dive→surface transition, or a
	// NEW VALID GPS fix. A NO_FIX does not, so a tag that had stopped diving and
	// was not getting fixes stayed silent indefinitely -- and did so without a
	// single line in the log, not even at TRACE, which makes it
	// indistinguishable from a dead radio on a recovered device.
	if (m_awaiting_surfacing) return schedule_surfacing_heartbeat(argos_config, now);

	// Not yet surfaced (boot): send Doppler at legacy rate
	m_scheduled_task = [this]() { process_doppler_burst(); };
	return from_scheduler(m_sched.schedule_legacy(argos_config, now), "not surfaced yet — legacy-rate doppler",
	                      "not surfaced yet: no slot computable");
}

/// @brief GNSS_EN=0: only DUTY_CYCLE and LEGACY can schedule without a position.
/// Split out of service_next_schedule_in_ms(), which tested argos_config.mode
/// nine times in 489 lines. The gates it runs first -- cooldown, rate limit,
/// prepass, critical battery, certification -- are NOT repeated here.
ScheduleDecision ArgosTxService::schedule_without_gnss(ArgosConfig &argos_config, std::time_t now) {
	// BaseGnssStrategy::REUSE_LAST (Plan 1 follow-up): no GPS power-on
	// but the TX uses the most recent cached fix from the depth pile
	// (peek without consume, age-checked vs GNSS_REUSE_FIX_MAX_AGE_S).
	// If no usable cached entry, process_gnss_burst_from_cached()
	// internally falls back to a Doppler-only TX so the cycle still
	// happens. For FRESH and OFF, the existing process_doppler_burst
	// path is preserved exactly (no behavior change).
	if (argos_config.gnss_strategy == BaseGnssStrategy::REUSE_LAST) {
		m_scheduled_task = [this]() { process_gnss_burst_from_cached(); };
	} else if (should_promote_doppler_to_gnss(60)) {
		// FastLoc priority (2026-05): a fresh FastLoc / FIX in the
		// pile is more useful than a position-less Doppler. Use
		// process_gnss_burst — it auto-selects the FastLoc packet
		// builder for FASTLOC entries (argos_tx_service.cpp ~1020).
		DEBUG_INFO("ArgosTxService: fresh FastLoc/FIX in pile — promoting Doppler slot to GNSS TX");
		m_scheduled_task = [this]() { process_gnss_burst(); };
	} else {
		m_scheduled_task = [this]() { process_doppler_burst(); };
	}
	if (argos_config.mode == BaseArgosMode::DUTY_CYCLE) {
		m_scheduled_mode =
		    argos_config.adaptive_modulation ? adaptive_doppler_modulation() : resolve_non_adaptive_modulation();
		return from_scheduler(report_duty_cycle_schedule(m_sched.schedule_duty_cycle(argos_config, now), argos_config),
		                      "duty cycle", "duty cycle: no slot computable");
	}
	if (argos_config.mode == BaseArgosMode::LEGACY) {
		m_scheduled_mode =
		    argos_config.adaptive_modulation ? adaptive_doppler_modulation() : resolve_non_adaptive_modulation();
		return from_scheduler(m_sched.schedule_legacy(argos_config, now), "legacy period",
		                      "legacy: no slot computable");
	}
	// Only DUTY_CYCLE and LEGACY know how to do without the GNSS. Any
	// other mode landing here — PASS_PREDICTION in practice — has NO
	// scheduling branch and exits disabled. It used to be a bare
	// `return`: no trace, mute beacon, and nothing to diagnose it with
	// in the field. The incompatibility is reported at startup in
	// service_init(); we say it again here so the log states WHY
	// no transmission is scheduled any more.
	DEBUG_ERROR("ArgosTxService: mode %d with GNSS_EN=0 — no scheduling is possible, "
	            "TX disabled (incompatible configuration)",
	            static_cast<int>(argos_config.mode));
	if (status_led) status_led->flash(RGBLedColor::RED, 200);
	return ScheduleDecision::off("this mode requires GNSS_EN=1");
}

/// @brief GNSS_EN=1: time-sync burst, then DUTY_CYCLE / LEGACY / PASS_PREDICTION.
/// Split out of service_next_schedule_in_ms(), which tested argos_config.mode
/// nine times in 489 lines. The gates it runs first -- cooldown, rate limit,
/// prepass, critical battery, certification -- are NOT repeated here.
ScheduleDecision ArgosTxService::schedule_with_gnss(ArgosConfig &argos_config, std::time_t now) {
	// 2026-08 — THE FIRST-MESSAGE LOCK HAS BEEN REMOVED.
	//
	// It held back EVERY transmission, presence heartbeat included, as
	// long as no valid GPS fix had landed since power-on. A beacon that
	// restarted without ever getting a fix back — broken receiver,
	// blocked sky — therefore vanished for good from the screens:
	// neither position nor sign of life. That is precisely what would
	// have happened to the field tag of 2026-08-22 if a WDT or an OTA
	// had occurred during the three days of GNSS outage.
	//
	// The original rationale (v3 parity: "time known" implied "the GNSS
	// set the time") does not hold: in Doppler, LEGACY or DUTY_CYCLE the
	// beacon transmits at its period and it is the ground segment that
	// rebuilds the position from the reception instants — its clock does
	// not enter the computation. Only pass prediction needs a correct
	// time and position, and it has its own guard in
	// ArgosTxScheduler::schedule_prepass.
	//
	// RSPB was already exempt from this lock, for exactly that reason.
	//
	// What does stay conditional, on the other hand, is the time-sync
	// burst: it CARRIES the time, so transmitting it on a clock the GNSS
	// has not corrected would amount to broadcasting a wrong date.
	if (m_is_first_tx && argos_config.time_sync_burst_en && m_gps_fix_corrected_clock) {
		// Provisional modulation, like LEGACY/DUTY_CYCLE further down:
		// adaptive -> LDA2 (process_time_sync_burst switches back to LDK
		// depending on size), non-adaptive -> master RCONF. Before 2026-06-25
		// it was LDA2 forced unconditionally (cf. user point).
		m_scheduled_mode =
		    argos_config.adaptive_modulation ? KineisModulation::LDA2 : resolve_non_adaptive_modulation();
		m_scheduled_task = [this]() { process_time_sync_burst(); };
		m_sched.schedule_at(now);
		return ScheduleDecision::run(0, "time-sync burst");
	}
	if (m_depth_pile_manager.eligible() == 0) {
		DEBUG_INFO("ArgosTxService: depth pile has no eligible entries (NTRY exhausted or empty) — TX disabled "
		           "until next GPS entry");
		// The pile is RAM-only, so it is empty at every boot. Answering Off here
		// is what deadlocked a tag whose only filler -- the GNSS service -- was
		// itself waiting on a transmission that could then never happen.
		return ScheduleDecision::hold_for_event(600, "depth pile empty");
	}
	if (argos_config.mode == BaseArgosMode::DUTY_CYCLE) {
		// Non-adaptive: honor the master RCONF's actual modulation
		// (decoded from AT+RCONF=? at init on KIM2; LDA2 on SMD).
		// Adaptive: default to LDA2; process_*_burst will switch to
		// LDK if the payload fits (96/128-bit packets).
		m_scheduled_mode =
		    argos_config.adaptive_modulation ? KineisModulation::LDA2 : resolve_non_adaptive_modulation();
#ifdef BOARD_RSPB
		if (argos_config.adaptive_modulation && argos_config.sensor_tx_enable) {
			unsigned int pkt_fmt = configuration_store->read_param<unsigned int>(ParamID::RSPB_PACKET_FORMAT);
			if (pkt_fmt == 1) m_scheduled_mode = KineisModulation::LDK;
		}
#endif
		if (argos_config.sensor_tx_enable) {
			m_scheduled_task = [this]() { process_sensor_burst(); };
		} else {
			m_scheduled_task = [this]() { process_gnss_burst(); };
		}
		return from_scheduler(report_duty_cycle_schedule(m_sched.schedule_duty_cycle(argos_config, now), argos_config),
		                      "duty cycle", "duty cycle: no slot computable");
	}
	if (argos_config.mode == BaseArgosMode::LEGACY) {
		m_scheduled_mode =
		    argos_config.adaptive_modulation ? KineisModulation::LDA2 : resolve_non_adaptive_modulation();
#ifdef BOARD_RSPB
		if (argos_config.adaptive_modulation && argos_config.sensor_tx_enable) {
			unsigned int pkt_fmt = configuration_store->read_param<unsigned int>(ParamID::RSPB_PACKET_FORMAT);
			if (pkt_fmt == 1) m_scheduled_mode = KineisModulation::LDK;
		}
#endif
		if (argos_config.sensor_tx_enable) {
			m_scheduled_task = [this]() { process_sensor_burst(); };
		} else {
			m_scheduled_task = [this]() { process_gnss_burst(); };
		}
		return from_scheduler(m_sched.schedule_legacy(argos_config, now), "legacy period",
		                      "legacy: no slot computable");
	}
	if (argos_config.mode == BaseArgosMode::PASS_PREDICTION) {
		m_scheduled_mode =
		    argos_config.adaptive_modulation ? KineisModulation::LDA2 : resolve_non_adaptive_modulation();
		if (argos_config.sensor_tx_enable) {
			m_scheduled_task = [this]() { process_sensor_burst(); };
		} else {
			m_scheduled_task = [this]() { process_gnss_burst(); };
		}
		// AOP unusable (missing or expired) -> fall back to PERIODIC.
		// We used to return SCHEDULE_DISABLED: the beacon stayed mute
		// until the next GPS entry, with nothing to signal it. Staying
		// silent is the worse of the two evils; better to transmit
		// blind than not at all.
		unsigned int age_s = 0;
		AopStatus etat = aop_status(argos_config, now, age_s);
		if (etat != AopStatus::USABLE) {
			DEBUG_WARN("ArgosTxService: prepass impossible — %s (age=%u s, limit=%u d) — falling back to periodic TX",
			           aop_status_text(etat), age_s, argos_config.aop_max_age_days);
			refresh_prepass_status(argos_config, now, 0);
			return from_scheduler(m_sched.schedule_legacy(argos_config, now), "prepass fallback: periodic",
			                      "prepass fallback: no slot computable");
		}
		BasePassPredict &pass_predict = configuration_store->read_pass_predict();
		// m_scheduled_mode is resolved just above and is no longer
		// touched by the scheduler.
		unsigned int schedule = m_sched.schedule_prepass(argos_config, pass_predict, now);
		if (schedule == ArgosTxScheduler::INVALID_SCHEDULE) {
			DEBUG_WARN("ArgosTxService: no pass is computable — falling back to periodic TX");
			refresh_prepass_status(argos_config, now, 0);
			return from_scheduler(m_sched.schedule_legacy(argos_config, now), "prepass fallback: periodic",
			                      "prepass fallback: no slot computable");
		}
		// Safeguard: a window too far away must not block transmissions
		// for hours. Beyond SAT_PREPASS_MAX_WAIT_S we transmit in
		// periodic mode (0 = no safeguard).
		static constexpr unsigned int MS_PER_S = 1000;
		if (argos_config.prepass_max_wait_s && schedule > argos_config.prepass_max_wait_s * MS_PER_S) {
			DEBUG_INFO("ArgosTxService: next window in %u s > max wait %u s — falling back to periodic TX",
			           schedule / MS_PER_S, argos_config.prepass_max_wait_s);
			refresh_prepass_status(argos_config, now, now + schedule / MS_PER_S);
			return from_scheduler(m_sched.schedule_legacy(argos_config, now), "prepass fallback: periodic",
			                      "prepass fallback: no slot computable");
		}
		refresh_prepass_status(argos_config, now, now + schedule / MS_PER_S);
		return ScheduleDecision::run(schedule, "aligned on the next satellite pass");
	}
	return ScheduleDecision::off("unhandled ARGOS_MODE");
}


/// @brief Execute scheduled TX — run the prepared burst task (cert/gnss/sensor/doppler).
/// Called by ServiceManager when the scheduled time arrives.
void ArgosTxService::service_initiate() {
	DEBUG_TRACE("ArgosTxService::service_initiate");

	// Skip TX if the device has failed too many consecutive times this session.
	// Second copy of the guard in react(KineisEventDeviceError); this one
	// catches the wake-ups that the un-cancelled safety-net timeout keeps
	// generating while suspended. See the long note on that branch for why
	// those wake-ups exist and why removing them is not a local change.
	if (m_consecutive_device_errors >= DEVICE_ERROR_MAX_CONSECUTIVE) {
		std::time_t now = service_current_time();
		if (DEVICE_ERROR_PROBE_PERIOD_S && now < m_device_error_hold_until) {
			DEBUG_WARN("ArgosTxService::service_initiate: skipping TX — %u consecutive device errors, "
			           "suspended for another %llu s",
			           m_consecutive_device_errors, (unsigned long long)(m_device_error_hold_until - now));
			// Reschedule rather than complete-without-reschedule. The latter
			// leaves the safety-net timeout armed, and that timeout is what
			// used to spin this guard once per service_next_timeout().
			cancel_safety_timeout();
			m_sched.set_earliest_schedule(m_device_error_hold_until);
			service_complete();
			return;
		}
		// Deadline passed: step the counter back below the threshold so exactly
		// one dispatch goes out as a probe. If it succeeds, react(TxComplete)
		// clears the counter outright; if it fails, react() puts it back at MAX
		// and arms a fresh deadline.
		DEBUG_INFO("ArgosTxService::service_initiate: suspension elapsed — probing with one TX");
		m_consecutive_device_errors = DEVICE_ERROR_MAX_CONSECUTIVE - 1;
	}

	// The device cannot transmit and receive at the same time. Firing a TX now
	// would retrieve a depth pile entry -- consuming one of its NTRY redundancy
	// slots -- build a packet, hand it to a module locked in DL, and time out
	// 30 s later having emitted nothing. The cycle then repeats until the pile
	// is exhausted.
	//
	// Reception has priority by design (no preemption), so simply defer: no
	// entry is retrieved, no credit is spent, and the TX resumes on its own
	// once the window closes.
	if (m_kineis.is_receiving()) {
		std::time_t now = service_current_time();
		DEBUG_INFO("ArgosTxService::service_initiate: device is receiving, deferring TX by %u s "
		           "(no depth pile entry consumed)",
		           ARGOS_TX_RX_BUSY_DEFER_S);
		m_sched.set_earliest_schedule(now + (std::time_t)ARGOS_TX_RX_BUSY_DEFER_S);
		service_complete(nullptr, nullptr, true);
		return;
	}

	// Re-check gates that service_next_schedule_in_ms enforces. The framework
	// (Service::reschedule in service.cpp:586) schedules a task that fires
	// service_initiate() DIRECTLY after the delay returned by
	// service_next_schedule_in_ms — without re-running it. So a check that
	// fired and returned a reschedule (rate-limit at line 139, or in this
	// function once the burst max_msg branch runs) gets bypassed on the
	// next fire. Field log 2026-05-23 caught this: rate-limit returned
	// 45 s, burst counter was at 3/3, the rescheduled task fired
	// process_doppler_burst() and pushed counter to 4 then 5 — the user-
	// configured SURFACING_BURST_MAX_MSG=3 silently exceeded.
	//
	// Defense: re-evaluate rate-limit + burst-max here. If either says no,
	// abort and let the next reschedule re-arm cleanly.
	{
		std::time_t now = service_current_time();
		unsigned int rl_reschedule_s = 0;
		if (RateLimiter::is_blocked(now, rl_reschedule_s)) {
			DEBUG_INFO("ArgosTxService::service_initiate: rate-limited (reschedule_s=%u), aborting fire",
			           rl_reschedule_s);
			m_sched.schedule_at(now + (std::time_t)rl_reschedule_s);
			service_complete(nullptr, nullptr, true);
			return;
		}
		// Burst-max defense: if we're inside a SURFACING_BURST and already
		// hit max_msg, do NOT fire another Doppler. Mirrors line 191 in
		// service_next_schedule_in_ms. Only applies in Phase 1 (no GNSS fix
		// yet); Phase 2 (GNSS) is bounded by depth-pile eligibility.
		if (m_is_surfacing_burst && !m_has_gnss_fix_since_surfacing) {
			unsigned int max_msg = configuration_store->read_param<unsigned int>(ParamID::SURFACING_BURST_MAX_MSG);
			if (max_msg > 0 && m_doppler_burst_count >= max_msg) {
				DEBUG_INFO("ArgosTxService::service_initiate: Doppler limit reached (%u/%u), aborting fire",
				           m_doppler_burst_count, max_msg);
				unsigned int trigger = configuration_store->read_param<unsigned int>(ParamID::COOLDOWN_TRIGGER_MODE);
				if (trigger == (unsigned int)BaseCooldownTrigger::END_OF_DOPPLER && !m_cooldown_armed) {
					m_cooldown_armed = true;
					DEBUG_INFO("ArgosTxService: cooldown armed (END_OF_DOPPLER, max msg via initiate)");
				}
				m_is_surfacing_burst = false;
				m_awaiting_surfacing = true;
				m_first_gnss_tx_sent = false;
				service_complete(nullptr, nullptr, false);  // no reschedule — wait for next surface event
				return;
			}
		}
	}

	m_is_first_tx = false;
	m_is_tx_pending = true;

	// Skip TCXO warmup on first TX after surfacing from underwater
	if (m_tcxo_skip_on_next_tx) {
		DEBUG_TRACE("ArgosTxService::service_initiate: TCXO warmup skipped (first TX after submerge)");
		m_kineis.set_tcxo_warmup_time(0);
	}

	// Apply deferred modulation switch (cached while SMD was powered off)
	if (m_modulation_preconfig.has_value()) {
		DEBUG_INFO("ArgosTxService::service_initiate: applying deferred modulation switch to %d",
		           (int)m_modulation_preconfig.value());
		ensure_modulation(m_modulation_preconfig.value());
		m_modulation_preconfig.reset();
	}

	// Adaptive modulation pre-switch for LEGACY/DUTY_CYCLE/DOPPLER modes.
	// In these modes we switch RCONF + reload KMAC at init time (no timing
	// constraint). For SURFACING_BURST, the switch is done at TX complete
	// or during process to avoid KMAC reload at boot.
	//
	// Skip the pre-switch when the burst processor will re-decide modulation
	// from payload size (GNSS-only bursts on non-RSPB in LEGACY/DUTY_CYCLE/
	// PASS_PREDICTION): m_scheduled_mode is provisionally LDA2 at scheduling
	// time, but process_gnss_burst flips to LDK for single-fix short packets
	// (96 bits). Without this guard the device would switch LDA2 → then LDK
	// in succession, costing an extra RCONF+KMAC cycle (~280 ms) per TX.
	// RSPB already nails m_scheduled_mode at scheduling via RSPB_PACKET_FORMAT
	// so the pre-switch is accurate there.
	if (!m_modulation_preconfig.has_value()) {
		ArgosConfig argos_config;
		configuration_store->get_argos_configuration(argos_config);
		if (argos_config.adaptive_modulation && argos_config.mode != BaseArgosMode::SURFACING_BURST) {
			bool burst_may_override_mode = false;
#ifndef BOARD_RSPB
			burst_may_override_mode =
			    argos_config.gnss_en
			    && (argos_config.mode == BaseArgosMode::LEGACY || argos_config.mode == BaseArgosMode::DUTY_CYCLE
			        || argos_config.mode == BaseArgosMode::PASS_PREDICTION);
#endif
			if (!burst_may_override_mode && m_kineis.get_current_modulation() != m_scheduled_mode) {
				DEBUG_INFO("ArgosTxService::service_initiate: adaptive pre-switch to %s",
				           argos_modulation_to_string((BaseArgosModulation)m_scheduled_mode));
				ensure_modulation(m_scheduled_mode);
			}
		}
	}

	// Track Doppler burst count for scheduling interval calculation. Incremented
	// HERE — before process_doppler_burst() runs — so that after this point
	// m_doppler_burst_count is the index of the *current* TX (1-based).
	// Convention used by logs:
	//   - service_schedule() reads the counter BEFORE this increment, so it logs
	//     `count + 1` to refer to the upcoming TX (e.g. "Doppler #1 (immediate)").
	//   - process_doppler_burst() reads the counter AFTER this increment, so it
	//     logs `count` directly to refer to the current TX.
	// Note: only incremented in SURFACING_BURST mode — in legacy DOPPLER mode the
	// counter stays 0, which gates out CloudLocate/Fastloc payload substitution.
	if (m_is_surfacing_burst && !m_has_gnss_fix_since_surfacing) {
		m_doppler_burst_count++;
	}

	// DOPPLER burst counter (2026-05). Post-increment so process_doppler_burst
	// sees the 1-based index of the current TX. Bounded by SURFACING_BURST_MAX_MSG
	// in service_next_schedule_in_ms which resets to 0 at end of sequence.
	{
		ArgosConfig ac;
		configuration_store->get_argos_configuration(ac);
		if (ac.mode == BaseArgosMode::DOPPLER) {
			m_doppler_seq_count++;
		}
	}

	// Mark first GNSS TX as sent only when actually executing (not during scheduling).
	// Check m_has_gnss_fix (not m_is_surfacing_burst) because the GPS fix can arrive
	// after the burst ended (m_awaiting_surfacing path).
	if (m_has_gnss_fix_since_surfacing && !m_first_gnss_tx_sent) {
		m_first_gnss_tx_sent = true;
	}

	m_scheduled_task();
}

/// @brief Returns true — TX is active during initiate (async completion via events).
/// @return Always true (TX completes asynchronously via KineisDevice events).
bool ArgosTxService::service_is_active_on_initiate() {
	return false;
}

/// @brief Cancel pending TX — power off device and stop send.
/// @return true if TX was pending and cancelled, false otherwise.
bool ArgosTxService::service_cancel() {
	DEBUG_TRACE("ArgosTxService::service_cancel: pending=%u", m_is_tx_pending);
	bool is_pending = m_is_tx_pending;
	m_is_tx_pending = false;
	m_kineis.stop_send();
	return is_pending;
}

/// @brief TX timeout — TCXO warmup + 60s margin for satellite module response.
/// @return Timeout in ms after which TX is considered failed.
unsigned int ArgosTxService::service_next_timeout() {
	// Safety timeout: if KineisEventTxComplete/DeviceError never arrives, the
	// service framework cancels and reschedules.
	// Base budget: power-on(2s)+KMAC(1s)+setup(1s)+TCXO(5s)+TX(3s)+margin(18s)=30s
	unsigned int timeout_ms = 30000;
	// BLIND: the module bursts retx_nb copies retx_period_s apart and only reports
	// the completing +TX at the END of the whole burst. Extend the safety timeout
	// to cover the burst so it does not cancel a legitimately in-progress blind TX.
	// Capped at 2 h (keep retx_nb * retx_period_s under 2 h). blind_en here is the
	// effective, mode-gated flag (false in SURFACING_BURST/DOPPLER).
	ArgosConfig cfg;
	configuration_store->get_argos_configuration(cfg);
	if (cfg.blind_en) {
		unsigned int rn = (cfg.blind_retx_nb < 1) ? 1 : cfg.blind_retx_nb;
		uint64_t burst_ms = (uint64_t)rn * (uint64_t)cfg.blind_retx_period_s * 1000ULL;
		if (burst_ms > 7200000ULL) burst_ms = 7200000ULL;
		timeout_ms += (unsigned int)burst_ms;
	}
	return timeout_ms;
}

/// @brief Trigger reschedule on surfacing (for surfacing burst mode).
/// @param[out] immediate  Set to true if TX should fire immediately on surfacing.
/// @return true if this service should be rescheduled when device surfaces.
bool ArgosTxService::service_is_triggered_on_surfaced(bool &immediate) {
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);
	// In SURFACING_BURST mode, reschedule immediately (Doppler burst starts at T=0)
	immediate = (argos_config.mode == BaseArgosMode::SURFACING_BURST);
	return true;  // Re-schedule us on a surfaced event
}

/// @brief Handle peer service events (GPS fix, sensor data, underwater state, surfacing).
/// @param e  Event from another service (GPS, SWS, sensors, etc.).
void ArgosTxService::notify_peer_event(ServiceEvent &e) {
	//DEBUG_TRACE("ArgosTxService::notify_peer_event: (%u|%u)", e.event_source, e.event_type);

	// Background refresh of the pre-warmed Doppler packet: any peer event
	// fired while underwater (sensor sample, GPS tick, etc.) is a chance to
	// re-sample the battery and rebuild the payload if it has aged past 1h.
	// Skipped if we never prepared one (boot underwater, non-SURFACING_BURST).
	if (m_is_underwater && m_prepared_at_ms != 0) {
		uint64_t now_ms = PMU::get_timestamp_ms();
		if (now_ms - m_prepared_at_ms >= PREPARED_DOPPLER_REFRESH_MS) {
			DEBUG_TRACE("ArgosTxService::notify_peer_event: refreshing pre-warmed Doppler packet (>1h underwater)");
			prepare_doppler_packet();
		}
	}

	// During SURFACING_BURST Doppler phase, CloudLocate/Fastloc/NO_FIX entries are already
	// sent directly in process_doppler_burst() — skip depth pile to avoid double transmission.
	// Only real GPS fixes should enter the depth pile for the GNSS phase.
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
	}

	if (e.event_source == ServiceIdentifier::GNSS_SENSOR && e.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
		GPSLogEntry &entry = std::get<GPSLogEntry>(e.event_data);

		// Terrestrial equivalent of the surface-event reset (see UW_SENSOR
		// branch below). A new GPS session is a fresh TX opportunity, so clear
		// the 3-strike device-error suspension here. Without this, a land
		// tracker (RSPB) that never dives has only TWO ways to clear
		// m_consecutive_device_errors: a successful TX (impossible while the
		// radio is wedged) or a full reboot. A *transient* KIM2/SMD fault — e.g.
		// a battery brown-out at module power-on — would therefore suspend TX
		// PERMANENTLY until someone power-cycles the device. Re-arming once per
		// GPS session lets the next session retry; cost is bounded to
		// ≤ DEVICE_ERROR_MAX_CONSECUTIVE power-on attempts per session, which is
		// negligible next to the GPS fix that just ran. Harmless for underwater
		// units (their GPS sessions only occur at surface, already reset above).
		// Field case 2026-06-30: a KIM2 AT+PING brown-out at ~01:00 left an RSPB
		// unit TX-suspended for 6.5 h until a manual reboot, despite the battery
		// recovering within the hour.
		// A REAL FIX, not merely a session. Every GNSS session ends with a log
		// update, NO_FIX ones included, and a tag under poor sky can produce
		// those every few minutes. Clearing on any of them would cap the
		// 3-strike suspension at the GNSS retry cadence: the beacon would keep
		// retrying a wedged radio for as long as the sky stayed bad, which is
		// the opposite of what the suspension is for. A fix is also what the
		// clear is worth having for -- there is now something new to transmit.
		if (m_consecutive_device_errors > 0 && entry.info.valid) {
			DEBUG_INFO("ArgosTxService: clearing %u-error suspension on a new GPS fix — fresh TX opportunity",
			           m_consecutive_device_errors);
			m_consecutive_device_errors = 0;
			m_device_error_hold_until = 0;
			// Clearing the counter is not enough on its own. A suspension parks
			// the service on a probe deadline up to DEVICE_ERROR_PROBE_PERIOD_S
			// out, and that floor outlives the counter: without dropping it and
			// rescheduling here, the beacon would sit out the rest of the probe
			// period with a fresh fix in hand. Scoped to the error case on
			// purpose -- an unconditional reschedule on every GPS log would
			// change the cadence of every mode.
			m_sched.set_earliest_schedule(service_current_time());
			// NOT while a TX of ours is in flight. Service::reschedule() runs
			// deschedule() -- which cancels the safety-net timeout -- BEFORE it
			// tests m_is_initiated and returns; calling it mid-TX therefore
			// disarms that timeout and re-arms nothing, leaving the service
			// initiated for good if the module never answers. The completing TX
			// reschedules on its own, and the floor is already dropped.
			if (!is_initiated()) service_reschedule();
		}

		// First-message gate: a real GPS position fix is what corrects the RTC,
		// so it unlocks Argos TX for this session on non-RSPB builds (see
		// m_gps_fix_corrected_clock). NO_FIX (valid=false), FASTLOC and
		// CLOUDLOCATE do NOT correct the clock and must not unlock.
		if (entry.info.valid && entry.info.event_type != GPSEventType::FASTLOC
		    && entry.info.event_type != GPSEventType::CLOUDLOCATE && !m_gps_fix_corrected_clock) {
			m_gps_fix_corrected_clock = true;
			DEBUG_INFO("ArgosTxService: first valid GPS fix this session — TX unlocked");
		}

		// Update last known location (real fix only — fastloc is too inaccurate for scheduling)
		if (entry.info.valid && entry.info.event_type != GPSEventType::FASTLOC) {
			DEBUG_TRACE("ArgosTxService::notify_peer_event: updated GPS location");
			m_sched.set_last_location(entry.info.lon, entry.info.lat);

			// A real GPS fix supersedes degraded CloudLocate/Fastloc entries in
			// the depth pile (all modes — they must not replay stale/blob data
			// once a real position exists). The NO_FIX 0xFF heartbeats are only
			// purged in SURFACING_BURST: in LEGACY/DUTY_CYCLE/PASS_PREDICTION
			// (v3 behavior) they are delta_time_loc grid fillers that must keep
			// occupying their slot for multi-position dating.
			{
				ArgosConfig purge_cfg;
				configuration_store->get_argos_configuration(purge_cfg);
				bool include_no_fix = (purge_cfg.mode == BaseArgosMode::SURFACING_BURST);
				unsigned int purged = m_depth_pile_manager.purge_non_fix_entries(include_no_fix);
				if (purged) {
					DEBUG_INFO("ArgosTxService::notify_peer_event: purged %u degraded entr%s from depth pile "
					           "(no_fix_included=%u)",
					           purged, purged == 1 ? "y" : "ies", (unsigned int)include_no_fix);
				}
			}

			// SURFACING_BURST: GNSS fix received — switch to GNSS phase.
			// Works during active burst OR after burst ended (awaiting surfacing):
			// a real GPS fix always deserves to be transmitted.
			if ((m_is_surfacing_burst || m_awaiting_surfacing) && !m_has_gnss_fix_since_surfacing) {
				DEBUG_INFO("ArgosTxService::SURFACING_BURST: GNSS fix acquired after %u Doppler messages - switching "
				           "to GNSS phase",
				           m_doppler_burst_count);
				m_has_gnss_fix_since_surfacing = true;
				m_awaiting_surfacing = false;

				// Arm cooldown if trigger mode is END_OF_DOPPLER (Doppler phase
				// ends on GNSS fix). Guard against a delayed fix arriving during
				// an already-active cooldown (rare race: GPS in flight when
				// cooldown started + surface bounce sets m_is_surfacing_burst).
				unsigned int trigger = configuration_store->read_param<unsigned int>(ParamID::COOLDOWN_TRIGGER_MODE);
				if (trigger == (unsigned int)BaseCooldownTrigger::END_OF_DOPPLER && !m_cooldown_armed
				    && !ServiceManager::is_in_cooldown(service_current_time())) {
					m_cooldown_armed = true;
					DEBUG_INFO("ArgosTxService: cooldown armed (END_OF_DOPPLER, GNSS fix)");
				}
				service_reschedule();
				Service::notify_peer_event(e);
				return;
			}
		}

		// Reschedule the service
		if (!service_is_scheduled()) {
			DEBUG_TRACE("ArgosTxService::notify_peer_event: rescheduling as no existing schedule");
			service_reschedule();
		}

	} else if (e.event_source == ServiceIdentifier::UW_SENSOR
	           && e.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
		if (std::get<bool>(e.event_data) == true) {
			// Device went underwater:
			// 1. Cache TCXO=0 for next surfacing (RAM only, sent via SPI at next boot)
			m_tcxo_skip_on_next_tx = true;
			m_kineis.set_tcxo_warmup_time(0);
			// 2. Kill SMD and restore default idle timeout
			m_kineis.set_idle_timeout(1000);
			m_kineis.power_off_immediate();

			// Activate cooldown on dive if armed during this surfacing session
			if (m_cooldown_armed) {
				ServiceManager::set_cycle_complete(service_current_time());
				m_cooldown_armed = false;
			}

			// Reset surfacing burst state on dive
			m_is_surfacing_burst = false;
			m_awaiting_surfacing = false;
			m_doppler_burst_count = 0;
			m_has_gnss_fix_since_surfacing = false;
			m_first_gnss_tx_sent = false;
			m_is_underwater = true;

			// Pre-warm the first surfacing-burst Doppler packet now (SMD already
			// off, battery & ADC available) so the surface event skips the ADC
			// read + packet build on its critical path. No-op outside
			// SURFACING_BURST mode. Refreshed below if we stay underwater >1h.
			prepare_doppler_packet();

			// Adaptive modulation: the modulation switch to VLDA4 should have been
			// done at TX complete time while the SMD was still on. If for some reason
			// the SMD is still in the wrong modulation (e.g. error recovery, first boot),
			// cache VLDA4 as fallback so it gets applied at next power-on.
			{
				ArgosConfig ac;
				configuration_store->get_argos_configuration(ac);
				if (ac.adaptive_modulation && ac.mode == BaseArgosMode::SURFACING_BURST) {
					KineisModulation deferred = adaptive_doppler_modulation();
					if (m_kineis.get_current_modulation() != deferred) {
						m_modulation_preconfig = deferred;
						DEBUG_INFO("ArgosTxService::UW: modulation %d cached for the next surfacing (not "
						           "pre-switched)",
						           (int)deferred);
					} else {
						m_modulation_preconfig.reset();
						DEBUG_TRACE("ArgosTxService::UW: VLDA4 already active, no deferred switch needed");
					}
				}
			}
		} else {
			// Device surfaced
			m_is_underwater = false;
			// Reset the session-suspension counter so a transient burst of
			// failures earlier in the deployment can't permanently kill TX.
			// Without this reset m_consecutive_device_errors only clears on
			// service_init (boot) — on a multi-year single-boot deployment,
			// 3 early errors would suspend TX forever even after the SmdSat
			// 30-min cooldown expires and autofallback flips to SAFE.
			if (m_consecutive_device_errors > 0) {
				DEBUG_INFO("ArgosTxService: clearing %u-error suspension on surface event — fresh session",
				           m_consecutive_device_errors);
				m_consecutive_device_errors = 0;
				m_device_error_hold_until = 0;
				// No reschedule needed here, unlike the GNSS branch: the
				// earliest-TX floor is overwritten with dry_time_before_tx a
				// few lines below, and Service::notify_underwater_state(false)
				// reschedules us through service_is_triggered_on_surfaced().
			}
			ArgosConfig argos_config;
			configuration_store->get_argos_configuration(argos_config);
			std::time_t earliest_schedule = service_current_time() + argos_config.dry_time_before_tx;
			m_sched.set_earliest_schedule(earliest_schedule);

			// Arm cooldown immediately if trigger mode is AT_SURFACE.
			// Skip arming if a cooldown is already active — otherwise a passive
			// surface bounce during cooldown would re-arm m_cooldown_armed, and
			// the next dive would call set_cycle_complete(now) which resets the
			// cooldown timer, extending it indefinitely under repeated bounces.
			unsigned int trigger = configuration_store->read_param<unsigned int>(ParamID::COOLDOWN_TRIGGER_MODE);
			if (trigger == (unsigned int)BaseCooldownTrigger::AT_SURFACE
			    && !ServiceManager::is_in_cooldown(service_current_time())) {
				m_cooldown_armed = true;
				DEBUG_INFO("ArgosTxService: cooldown armed (AT_SURFACE)");
			}

			// Activate surfacing burst mode — only when cooldown is not active.
			// During an active cooldown the base class will skip reschedule
			// anyway (no TX will fire), so setting burst state + logging
			// "starting Doppler burst sequence" would be misleading and waste
			// no-op state churn on every passive bounce.
			if (argos_config.mode == BaseArgosMode::SURFACING_BURST
			    && !ServiceManager::is_in_cooldown(service_current_time())) {
				m_is_surfacing_burst = true;
				m_awaiting_surfacing = false;
				m_doppler_burst_count = 0;
				m_has_gnss_fix_since_surfacing = false;
				m_first_gnss_tx_sent = false;
				// Keep SMD alive between burst pings — default 1s idle timeout
				// is too short for 5-30s Doppler intervals, causing shutdown+reboot
				// failures on the 3rd TX
				m_kineis.set_idle_timeout((argos_config.surfacing_burst_max_s + 10) * 1000);
				m_scheduled_task = [this]() { process_doppler_burst(); };
				// Outside adaptive mode the modulation comes from the master RCONF, as
				// on the scheduling path of the same mode further up. An LDA2 hardcoded
				// here cancelled the operator's setting.
				m_scheduled_mode =
				    argos_config.adaptive_modulation ? adaptive_doppler_modulation() : resolve_non_adaptive_modulation();
				// Demoted to TRACE: the canonical state-change marker is
				// "UWDetectorService: state changed: state=0" emitted in the same
				// broadcast cascade. This log added ~50-300 ms LFS commit on the
				// surfacing critical path with no actionable info beyond the state
				// change itself.
				DEBUG_TRACE("ArgosTxService::SURFACING_BURST: surface detected - starting Doppler burst sequence");
			}
		}
	}

	// CloudLocate-ready notification from GPS: mirror of the LoRa path.
	// Triggers an early Doppler-burst tick so the next TX uses CloudLocate
	// (via the count>0 + has_raw_measurement check in process_doppler_burst)
	// instead of waiting for the normal surfacing_burst timer. Edge case
	// "raw arrives during in-flight TX" is NOT handled here (no pending
	// flag — Argos keeps it simple per user request); on that path the
	// CloudLocate just fires at the next normal timer tick, with a small
	// timing penalty vs the LoRa "immediately after" guarantee.
	if (e.event_source == ServiceIdentifier::GNSS_SENSOR && e.event_type == ServiceEventType::GNSS_CLOUDLOCATE_READY) {
		if (m_is_surfacing_burst && !m_has_gnss_fix_since_surfacing && !m_is_tx_pending) {
			// service_reschedule(true) means "fire now" and skips
			// service_next_schedule_in_ms() entirely, so none of the scheduling
			// gates run -- including the device-error hold. That makes this a
			// third way out of a backoff, next to the GNSS fix and the surface
			// event, and an unintended one: a CloudLocate becoming ready seconds
			// after a failed TX would transmit straight through the hold. Defer
			// to the normal path instead; it still fires at the burst cadence
			// once the hold lifts, and the CloudLocate stays in the pile.
			if (m_device_error_hold_until != 0 && service_current_time() < m_device_error_hold_until) {
				DEBUG_INFO("ArgosTxService::notify_peer_event: GNSS_CLOUDLOCATE_READY during a device-error hold — "
				           "deferring to the normal schedule");
				return;
			}
			DEBUG_INFO("ArgosTxService::notify_peer_event: GNSS_CLOUDLOCATE_READY — rescheduling early CloudLocate TX");
			m_scheduled_task = [this]() { process_doppler_burst(); };
			service_reschedule(true);
			return;
		}
		DEBUG_TRACE(
		    "ArgosTxService::notify_peer_event: GNSS_CLOUDLOCATE_READY but not in burst phase 1 or TX in flight");
	}

	Service::notify_peer_event(e);
}

// ============================================================================
// Adaptive modulation helpers
// ============================================================================

/// @brief Refresh the m_modulation_avail_mask snapshot from config_store.
/// Called at service_init and at the top of every scheduling cycle so a
/// runtime PARMW edit on one of the per-mod RCONFs takes effect on the next
/// TX without requiring a reboot. A modulation is "provisioned" iff its
/// RCONF is a non-empty 32-char hex string (ensure_modulation()'s validity
/// rule). The mask is purely advisory — ensure_modulation() still re-checks
/// at switch time, so a stale mask only delays the skip decision by one
/// cycle, never causes a wrong TX.
void ArgosTxService::refresh_modulation_availability() {
	ArgosConfig cfg;
	configuration_store->get_argos_configuration(cfg);
	auto valid = [](const std::string &s) { return !s.empty() && s.size() == 32; };
	uint8_t prev = m_modulation_avail_mask;
	m_modulation_avail_mask = 0;
	if (valid(cfg.radioconf_ldk)) m_modulation_avail_mask |= (1u << 0);
	if (valid(cfg.radioconf_lda2)) m_modulation_avail_mask |= (1u << 1);
	// Provisioned is not the same as permitted: an operator can hold a perfectly
	// valid VLDA4 RCONF on a backend that will refuse to transmit it.
	if (VLDA4_PERMITTED && valid(cfg.radioconf_vlda4)) m_modulation_avail_mask |= (1u << 2);
	if (prev != m_modulation_avail_mask) {
		DEBUG_INFO("ArgosTxService: modulation availability mask=0x%02X (LDK=%u LDA2=%u VLDA4=%u)",
		           m_modulation_avail_mask, (m_modulation_avail_mask >> 0) & 1, (m_modulation_avail_mask >> 1) & 1,
		           (m_modulation_avail_mask >> 2) & 1);
	}
}

/// @brief Can this modulation carry this frame, on this backend?
/// @param mode          candidate modulation.
/// @param payload_bits  frame size.
///
/// Size is not the only reason a modulation can be unusable: KIM2 refuses VLDA4
/// outright, whatever the frame. Folding both reasons into one predicate means
/// the fallback already written for the size case -- switch to LDA2 and
/// reprogram the module through ensure_modulation() -- also covers the
/// permission case, instead of every call site growing a second branch that
/// forgets to reprogram.
bool ArgosTxService::can_transmit_on(KineisModulation mode, unsigned int payload_bits) {
	if (mode == KineisModulation::VLDA4 && !VLDA4_PERMITTED) return false;
	return size_fits_modulation(payload_bits, mode);
}

bool ArgosTxService::is_modulation_provisioned(KineisModulation mode) const {
	switch (mode) {
	case KineisModulation::LDK: return (m_modulation_avail_mask >> 0) & 1;
	case KineisModulation::LDA2: return (m_modulation_avail_mask >> 1) & 1;
	case KineisModulation::VLDA4: return (m_modulation_avail_mask >> 2) & 1;
	default: return false;
	}
}

/// @brief Whether a payload of @p payload_bits will fit @p mode (per the
/// KIM2/SMD send() max-size table). LDA2 has the largest budget, so when an
/// ensure_modulation() switch fails and we'd fall back to "current", this is
/// what tells the burst processor whether the fallback is viable or whether
/// the TX must be skipped to avoid KIM2's silent payload-too-long drop.
bool ArgosTxService::size_fits_modulation(unsigned int payload_bits, KineisModulation mode) {
	switch (mode) {
	case KineisModulation::LDK: return payload_bits <= 128;
	case KineisModulation::LDA2: return payload_bits <= 192;
	case KineisModulation::VLDA4: return payload_bits <= 24;
	default: return false;
	}
}

/// @brief Modulation honored by the device when adaptive is OFF.
/// On KIM2, state_init parses AT+RCONF=? and updates m_current_rconf_mode to
/// the modulation actually encoded in the master RCONF (which is encrypted
/// hex and not locally decodable). On SMD, m_modulation stays at the LDA2
/// constructor default — SMD users see today's behavior (master RCONF must
/// encode LDA2). On first cold boot of KIM2 before init has run, returns the
/// LDA2 default; the first TX may fail if the master encodes a different
/// modulation, but state_init will update the cache and subsequent TXs work.
KineisModulation ArgosTxService::resolve_non_adaptive_modulation() {
	// Deliberately reports what the module actually has loaded, VLDA4 included.
	// The non-adaptive burst paths rely on that to decide they need NOT
	// reprogram; substituting a different modulation here would make them send
	// under the wrong RCONF. The VLDA4 ban is applied by can_transmit_on(),
	// which routes through the fallback that does call ensure_modulation().
	return m_kineis.get_current_modulation();
}

/// @brief Modulation the adaptive Doppler phase transmits on.
/// VLDA4 where it is permitted, LDA2 where it is not. Adaptive mode picked
/// VLDA4 unconditionally at six separate sites before this existed.
KineisModulation ArgosTxService::adaptive_doppler_modulation() const {
	return VLDA4_PERMITTED ? KineisModulation::VLDA4 : KineisModulation::LDA2;
}

/// @brief Get RCONF hex string for a given modulation from config store.
/// @param mode  Target modulation (LDK, LDA2, VLDA4).
/// @return 32-char hex RCONF string, or empty if not configured.
std::string ArgosTxService::get_rconf_for_modulation(KineisModulation mode) {
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);
	switch (mode) {
	case KineisModulation::LDK: return argos_config.radioconf_ldk;
	case KineisModulation::VLDA4: return argos_config.radioconf_vlda4;
	case KineisModulation::LDA2:
	default: return argos_config.radioconf_lda2;
	}
}

/// @brief Switch RCONF on KineisDevice if current modulation doesn't match target.
/// @param target  Desired modulation for next TX.
/// @return true if modulation is ready, false if switch failed.
bool ArgosTxService::ensure_modulation(KineisModulation target) {
	if (m_kineis.get_current_modulation() == target) {
		return true;
	}
	std::string rconf = get_rconf_for_modulation(target);
	if (rconf.empty() || rconf.size() != 32) {
		DEBUG_ERROR("ArgosTxService::ensure_modulation: invalid RCONF for mode %d (len=%u)", (int)target,
		            (unsigned)rconf.size());
		return false;
	}
	DEBUG_INFO("ArgosTxService::ensure_modulation: switching to %d", (int)target);
	return m_kineis.switch_modulation(target, rconf);
}

/// @brief Build and send certification TX burst from config payload.
void ArgosTxService::process_certification_burst() {
	DEBUG_TRACE("ArgosTxService::process_certification_burst");
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);
	// The modulation is settled first because the frame is built to fit it: the
	// three modulations carry different sizes and CERT_TX_PAYLOAD is whatever the
	// operator typed.
	KineisModulation cert_mode = (KineisModulation)argos_config.cert_tx_modulation;
	// No silent substitution here, unlike the operational bursts: a certification
	// frame sent on a modulation the operator did not ask for certifies nothing.
	// Say what is wrong and transmit nothing -- and, crucially, take no strike
	// for it, since this is a configuration error and not a device fault.
	if (cert_mode == KineisModulation::VLDA4 && !VLDA4_PERMITTED) {
		DEBUG_ERROR("ArgosTxService::process_certification_burst: CERT_TX_MODULATION is VLDA4, which this backend does "
		            "not permit — TX aborted (pick LDK or LDA2)");
		service_complete();
		return;
	}
	unsigned int size_bits;
	KineisPacket packet =
	    ArgosPacketBuilder::build_certification_packet(argos_config.cert_tx_payload, cert_mode, size_bits);
	// Demoted to TRACE: per-TX payload dump (~50-300 ms LFS commit).
	DEBUG_TRACE("ArgosTxService::process_certification_burst: mode=%s data=%s sz=%u",
	            argos_modulation_to_string(argos_config.cert_tx_modulation), Binascii::hexlify(packet).c_str(),
	            size_bits);
	// Every other burst programmes the module before transmitting; this one did
	// not, so a certification frame could go out under whatever RCONF the last
	// ordinary TX happened to leave loaded.
	if (!ensure_modulation(cert_mode)) {
		DEBUG_ERROR("ArgosTxService::process_certification_burst: no valid RCONF for %s - TX aborted",
		            argos_modulation_to_string(argos_config.cert_tx_modulation));
		service_complete(nullptr, nullptr, true);
		return;
	}
	m_last_val_tx_type = "cert";
	m_kineis.send(cert_mode, packet, size_bits);
}

/// @brief Send immediate time sync burst using most recent GPS fix.
void ArgosTxService::process_time_sync_burst() {
	DEBUG_TRACE("ArgosTxService::process_time_sync_burst");
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);
	unsigned int size_bits;
	std::vector<GPSLogEntry *> v = m_depth_pile_manager.retrieve_gps_latest();
	if (v.size()) {
		KineisPacket packet = ArgosPacketBuilder::build_gnss_packet(v, argos_config.is_out_of_zone, argos_config.is_lb,
		                                                            argos_config.delta_time_loc, size_bits);
		// Modulation: same adaptive + payload-size policy as process_gnss_burst
		// (2026-06-25). Before, LDA2 was forced unconditionally, which
		// over-provisioned the time-sync packet (always a single latest fix =
		// 96 bits) and ignored the master RCONF in non-adaptive. The gnss_burst
		// fix (adaptive selection + size_fits_modulation guard + non-adaptive
		// fallback, cf. process_gnss_burst) avoids handing KIM2 a frame of the
		// wrong length (+ERROR=5) or a silent oversize drop. Device-agnostic
		// (safe on the SMD side too).
		if (argos_config.adaptive_modulation) {
			m_scheduled_mode = (size_bits <= 128) ? KineisModulation::LDK : KineisModulation::LDA2;
			if (!ensure_modulation(m_scheduled_mode)) {
				DEBUG_WARN("ArgosTxService::process_time_sync_burst: modulation switch failed, using current");
				m_scheduled_mode = m_kineis.get_current_modulation();
				if (!can_transmit_on(m_scheduled_mode, size_bits)) {
					DEBUG_ERROR("ArgosTxService::process_time_sync_burst: payload %u bits doesn't fit fallback mod %d "
					            "— skipping TX",
					            size_bits, (int)m_scheduled_mode);
					service_complete();
					return;
				}
			}
		} else {
			// Non-adaptive: keep the master RCONF modulation (fixed at scheduling
			// via resolve_non_adaptive_modulation()). If it does not hold the packet
			// (e.g. VLDA4 master = 24 b), fall back to LDA2 if provisioned, else a
			// clean skip rather than a silent KIM2 oversize drop.
			if (!can_transmit_on(m_scheduled_mode, size_bits)) {
				if (can_transmit_on(KineisModulation::LDA2, size_bits)
				    && ensure_modulation(KineisModulation::LDA2)) {
					DEBUG_WARN("ArgosTxService::process_time_sync_burst: %u bits don't fit master mod %d — falling "
					           "back to LDA2",
					           size_bits, (int)m_scheduled_mode);
					m_scheduled_mode = KineisModulation::LDA2;
				} else {
					DEBUG_ERROR(
					    "ArgosTxService::process_time_sync_burst: %u bits fit no provisioned modulation — skipping TX",
					    size_bits);
					service_complete();
					return;
				}
			}
		}
		// Demoted to TRACE: per-TX payload dump.
		DEBUG_TRACE("ArgosTxService::process_time_sync_burst: mode=%s data=%s sz=%u",
		            argos_modulation_to_string((BaseArgosModulation)m_scheduled_mode),
		            Binascii::hexlify(packet).c_str(), size_bits);
		m_last_tx_had_gps = true;
		m_last_val_tx_type = "tsync";
		m_kineis.send(m_scheduled_mode, packet, size_bits);
	} else {
		// No eligible entries for transmission in the depth pile, so send a doppler burst instead
		DEBUG_WARN("ArgosTxService::process_time_sync_burst: no entries eligible in depth pile");
		service_complete();
	}
}

/// @brief Build and send sensor packet (GPS + optional ALS/PH/pressure/temp/AXL).
void ArgosTxService::process_sensor_burst() {
	DEBUG_TRACE("ArgosTxService::process_sensor_burst");
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);
	unsigned int size_bits;
	GPSLogEntry *gps = m_depth_pile_manager.retrieve_gps_single((unsigned int)argos_config.depth_pile);
	if (gps != nullptr) {
		// If GPS entry is a CloudLocate, send CloudLocate packet (extract blob from overlay)
		if (gps->info.event_type == GPSEventType::CLOUDLOCATE) {
			const uint8_t *overlay = reinterpret_cast<const uint8_t *>(&gps->info.lon);
			uint8_t format_id = overlay[0];
			// STRICT format guard (2026-06): skip CloudLocate TX when the stored
			// format is the 0xFF sentinel (configured format not produced this
			// session) or otherwise invalid — never transmit a mismatched/empty
			// CloudLocate packet.
			if (format_id != (uint8_t)BaseCloudLocateFormat::MEASC12
			    && format_id != (uint8_t)BaseCloudLocateFormat::MEAS20) {
				DEBUG_WARN(
				    "ArgosTxService::process_sensor_burst: CloudLocate format 0x%02X unavailable/invalid — skipping TX",
				    format_id);
				service_complete();
				return;
			}
			const uint8_t *blob = &overlay[1];
			unsigned int blob_size = (format_id == (uint8_t)BaseCloudLocateFormat::MEASC12) ? 12 : 20;

			uint32_t cl_capture =
			    (uint32_t)convert_epochtime(gps->header.year, gps->header.month, gps->header.day, gps->header.hours,
				                            gps->header.minutes, gps->header.seconds);
			KineisPacket packet = ArgosPacketBuilder::build_cloudlocate_packet(
			    blob, blob_size, format_id, gps->info.batt_voltage, argos_config.is_lb, cl_capture,
			    (uint32_t)service_current_time());
			size_bits = ArgosPacketBuilder::cloudlocate_packet_bits(format_id);

			// Modulation policy:
			//   adaptive=ON  → pick optimal mod from format (MEASC12→LDK, MEAS20→LDA2)
			//                  and switch the SMD accordingly.
			//   adaptive=OFF → respect the master modulation actually configured in
			//                  the SMD (NOT m_scheduled_mode — line 207 hardcodes
			//                  LDA2 in SURFACING_BURST regardless of master). Query
			//                  the live SMD state so send() always matches.
			//                  Padding fits LDA2 (24 B); the LDK micro-optimization
			//                  for MEASC12 is opt-in via ARGOS_AD_MOD.
			if (argos_config.adaptive_modulation) {
				m_scheduled_mode = (format_id == (uint8_t)BaseCloudLocateFormat::MEASC12) ? KineisModulation::LDK
				                                                                          : KineisModulation::LDA2;
				if (!ensure_modulation(m_scheduled_mode)) {
					DEBUG_WARN("ArgosTxService::process_sensor_burst: CloudLocate modulation switch failed");
					m_scheduled_mode = m_kineis.get_current_modulation();
					if (!can_transmit_on(m_scheduled_mode, size_bits)) {
						DEBUG_ERROR("ArgosTxService::process_sensor_burst: CloudLocate payload %u bits doesn't fit "
						            "fallback mod %d — skipping TX",
						            size_bits, (int)m_scheduled_mode);
						service_complete();
						return;
					}
				}
			} else {
				m_scheduled_mode = m_kineis.get_current_modulation();
				if (!can_transmit_on(m_scheduled_mode, size_bits)) {
					DEBUG_ERROR("ArgosTxService::process_sensor_burst: CloudLocate payload %u bits doesn't fit master "
					            "mod %d (ARGOS_AD_MOD=0) — skipping TX",
					            size_bits, (int)m_scheduled_mode);
					service_complete();
					return;
				}
			}
			// Demoted to TRACE: per-TX payload dump.
			DEBUG_TRACE("ArgosTxService::process_sensor_burst: CloudLocate fmt=%u mode=%s data=%s", format_id,
			            argos_modulation_to_string((BaseArgosModulation)m_scheduled_mode),
			            Binascii::hexlify(packet).c_str());
			m_last_tx_had_gps = true;
			m_last_val_tx_type = "cloudloc";
			m_kineis.send(m_scheduled_mode, packet, size_bits);
			return;
		}

		// If GPS entry is a fastloc (degraded fix), send fastloc packet instead of sensor packet
		if (gps->info.event_type == GPSEventType::FASTLOC) {
			KineisPacket packet = ArgosPacketBuilder::build_fastloc_packet(gps, argos_config.is_lb);
			size_bits = ArgosPacketBuilder::FASTLOC_PACKET_BITS;
			m_scheduled_mode = KineisModulation::LDA2;
			if (argos_config.adaptive_modulation) {
				if (!ensure_modulation(m_scheduled_mode)) {
					DEBUG_WARN("ArgosTxService::process_sensor_burst: fastloc modulation switch failed, using current");
					m_scheduled_mode = m_kineis.get_current_modulation();
					if (!can_transmit_on(m_scheduled_mode, size_bits)) {
						DEBUG_ERROR("ArgosTxService::process_sensor_burst: fastloc payload %u bits doesn't fit "
						            "fallback mod %d — skipping TX",
						            size_bits, (int)m_scheduled_mode);
						service_complete();
						return;
					}
				}
			} else {
				// Non-adaptive: 192 bits, only LDA2 carries them. That is the
				// legitimate safety escalation — but it was never APPLIED here,
				// and we left in LDA2 on a module still on the master RCONF.
				if (!ensure_modulation(KineisModulation::LDA2)) {
					DEBUG_ERROR("ArgosTxService::process_sensor_burst: fastloc %u bits need LDA2, not provisioned — "
					            "skipping TX",
					            size_bits);
					service_complete();
					return;
				}
			}
			// Demoted to TRACE: per-TX payload dump.
			DEBUG_TRACE("ArgosTxService::process_sensor_burst: fastloc mode=%s data=%s sz=%u",
			            argos_modulation_to_string((BaseArgosModulation)m_scheduled_mode),
			            Binascii::hexlify(packet).c_str(), size_bits);
			m_last_tx_had_gps = true;
			m_last_val_tx_type = "fastloc";
			m_kineis.send(m_scheduled_mode, packet, size_bits);
			return;
		}

		KineisPacket packet;
#ifdef BOARD_RSPB
		// RSPB uses dedicated packet format with compact AXL + mortality confidence
		unsigned int mort_conf = 0;
#if ENABLE_MORTALITY_SENSOR
		if (mortality_service) mort_conf = mortality_service->get_confidence();
#endif
		ServiceSensorData *pressure = m_depth_pile_manager.retrieve_sensor_single((unsigned int)argos_config.depth_pile,
		                                                                          ServiceIdentifier::PRESSURE_SENSOR);
		ServiceSensorData *thermistor = m_depth_pile_manager.retrieve_sensor_single(
		    (unsigned int)argos_config.depth_pile, ServiceIdentifier::THERMISTOR_SENSOR);
		ServiceSensorData *axl = nullptr;
#if ENABLE_AXL_SENSOR
		axl = m_depth_pile_manager.retrieve_sensor_single((unsigned int)argos_config.depth_pile,
		                                                  ServiceIdentifier::AXL_SENSOR);
#endif
		unsigned int pkt_fmt = configuration_store->read_param<unsigned int>(ParamID::RSPB_PACKET_FORMAT);
		if (pkt_fmt == 1) {
			m_scheduled_mode = KineisModulation::LDK;
			packet = ArgosPacketBuilder::build_rspb_short_packet(
			    gps, pressure, thermistor, axl, argos_config.is_out_of_zone, argos_config.is_lb, mort_conf, size_bits);
		} else {
			m_scheduled_mode = KineisModulation::LDA2;
			packet = ArgosPacketBuilder::build_rspb_long_packet(
			    gps, pressure, thermistor, axl, argos_config.is_out_of_zone, argos_config.is_lb, mort_conf, size_bits);
		}

		// Adaptive modulation: switch RCONF to match packet modulation
		if (argos_config.adaptive_modulation) {
			if (!ensure_modulation(m_scheduled_mode)) {
				DEBUG_WARN("ArgosTxService::process_sensor_burst: RSPB modulation switch failed, using current");
				m_scheduled_mode = m_kineis.get_current_modulation();
				if (!can_transmit_on(m_scheduled_mode, size_bits)) {
					DEBUG_ERROR("ArgosTxService::process_sensor_burst: RSPB payload %u bits doesn't fit fallback mod "
					            "%d — skipping TX",
					            size_bits, (int)m_scheduled_mode);
					service_complete();
					return;
				}
			}
		}
#else
		// Generic sensor packet for LinkIt V4 (all sensors, no RSPB-specific packing)
		// Demoted to TRACE: per-TX payload dump on hot path.
		DEBUG_TRACE("TX_RAW: SENS lat=%.6f lon=%.6f hAcc=%u nSV=%u hDOP=%.1f batt=%umV", gps->info.lat, gps->info.lon,
		            gps->info.hAcc, gps->info.numSV, (double)gps->info.hDOP, (unsigned)gps->info.batt_voltage);
		m_scheduled_mode = KineisModulation::LDA2;
		packet = ArgosPacketBuilder::build_sensor_packet(
		    gps,
		    m_depth_pile_manager.retrieve_sensor_single((unsigned int)argos_config.depth_pile,
			                                            ServiceIdentifier::ALS_SENSOR),
		    m_depth_pile_manager.retrieve_sensor_single((unsigned int)argos_config.depth_pile,
			                                            ServiceIdentifier::PH_SENSOR),
		    m_depth_pile_manager.retrieve_sensor_single((unsigned int)argos_config.depth_pile,
			                                            ServiceIdentifier::PRESSURE_SENSOR),
		    m_depth_pile_manager.retrieve_sensor_single((unsigned int)argos_config.depth_pile,
			                                            ServiceIdentifier::SEA_TEMP_SENSOR),
#if ENABLE_AXL_SENSOR
		    m_depth_pile_manager.retrieve_sensor_single((unsigned int)argos_config.depth_pile,
			                                            ServiceIdentifier::AXL_SENSOR),
#else
		    nullptr,
#endif
		    argos_config.is_out_of_zone, argos_config.is_lb, size_bits);

		// Adaptive modulation for generic sensor packet
		if (argos_config.adaptive_modulation) {
			// Sensor packet fits in LDK (128 bits)?
			if (size_bits <= 128) {
				m_scheduled_mode = KineisModulation::LDK;
			}
			if (!ensure_modulation(m_scheduled_mode)) {
				DEBUG_WARN("ArgosTxService::process_sensor_burst: modulation switch failed, using current");
				m_scheduled_mode = m_kineis.get_current_modulation();
				if (!can_transmit_on(m_scheduled_mode, size_bits)) {
					DEBUG_ERROR("ArgosTxService::process_sensor_burst: sensor payload %u bits doesn't fit fallback mod "
					            "%d — skipping TX",
					            size_bits, (int)m_scheduled_mode);
					service_complete();
					return;
				}
			}
		} else {
			// This processor was the only one in the file without a non-adaptive
			// branch: the LDA2 set further up shipped as-is, without any
			// ensure_modulation programming the module. Same policy as
			// process_time_sync_burst and process_gnss_burst — the master RCONF
			// prevails, and we escalate to LDA2 only if the packet does not fit it.
			m_scheduled_mode = resolve_non_adaptive_modulation();
			if (!can_transmit_on(m_scheduled_mode, size_bits)) {
				if (can_transmit_on(KineisModulation::LDA2, size_bits)
				    && ensure_modulation(KineisModulation::LDA2)) {
					DEBUG_WARN(
					    "ArgosTxService::process_sensor_burst: %u bits don't fit master mod %d — falling back to LDA2",
					    size_bits, (int)m_scheduled_mode);
					m_scheduled_mode = KineisModulation::LDA2;
				} else {
					DEBUG_ERROR(
					    "ArgosTxService::process_sensor_burst: %u bits fit no provisioned modulation — skipping TX",
					    size_bits);
					service_complete();
					return;
				}
			}
		}
#endif
		// Demoted to TRACE: per-TX payload dump.
		DEBUG_TRACE("ArgosTxService::process_sensor_burst: mode=%s data=%s sz=%u",
		            argos_modulation_to_string((BaseArgosModulation)m_scheduled_mode),
		            Binascii::hexlify(packet).c_str(), size_bits);
		m_last_tx_had_gps = true;
		m_last_val_tx_type = "sensor";
		m_kineis.send(m_scheduled_mode, packet, size_bits);
	} else {
		DEBUG_WARN("ArgosTxService::process_sensor_burst: no entries eligible in depth pile");
		if (m_is_surfacing_burst || m_has_gnss_fix_since_surfacing) {
			DEBUG_INFO("ArgosTxService::process_sensor_burst: ending surfacing burst (depth pile exhausted)");
			m_is_surfacing_burst = false;
			m_awaiting_surfacing = true;
			m_has_gnss_fix_since_surfacing = false;
			m_first_gnss_tx_sent = false;
		}
		service_complete();
	}
}

/// @brief Build and send GNSS packet (short or long based on depth pile entries).
void ArgosTxService::process_gnss_burst() {
	DEBUG_TRACE("ArgosTxService::process_gnss_burst");
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);
	unsigned int size_bits;
	// Non-adaptive (legacy): the master RCONF modulation is fixed for this TX.
	// If it can't hold a multi-fix LONG packet (e.g. LDK = 128b < LDA2's 192b),
	// cap the burst to a single fix at retrieve time so we build a SHORT packet
	// that fits — rather than handing send() an oversize payload that KIM2 drops
	// silently (DEBUG_ERROR + bare return, no event → 30 s timeout stall, repeats
	// every cycle on an LDK master). Capping at retrieve (not after build) avoids
	// wasting a burst_counter redundancy slot on fixes we wouldn't transmit.
	// User policy 2026-06-17: keep the master modulation, limit the fix count;
	// LDA2 fallback (below) only kicks in when even a single fix doesn't fit.
	std::vector<GPSLogEntry *> v;
	if (!argos_config.adaptive_modulation
	    && !size_fits_modulation(ArgosPacketBuilder::LONG_PACKET_BITS, m_scheduled_mode)) {
		v = m_depth_pile_manager.retrieve_gps((unsigned int)argos_config.depth_pile, 1);
	} else {
		v = m_depth_pile_manager.retrieve_gps((unsigned int)argos_config.depth_pile);
	}
	if (v.size()) {
		KineisPacket packet;

		// Check if the latest entry is a CloudLocate
		if (v.back()->info.event_type == GPSEventType::CLOUDLOCATE) {
			const uint8_t *overlay = reinterpret_cast<const uint8_t *>(&v.back()->info.lon);
			uint8_t format_id = overlay[0];
			// STRICT format guard (2026-06): skip CloudLocate TX on 0xFF sentinel
			// (configured format not produced) or any invalid format byte.
			if (format_id != (uint8_t)BaseCloudLocateFormat::MEASC12
			    && format_id != (uint8_t)BaseCloudLocateFormat::MEAS20) {
				DEBUG_WARN(
				    "ArgosTxService::process_gnss_burst: CloudLocate format 0x%02X unavailable/invalid — skipping TX",
				    format_id);
				service_complete();
				return;
			}
			const uint8_t *blob = &overlay[1];
			unsigned int blob_size = (format_id == (uint8_t)BaseCloudLocateFormat::MEASC12) ? 12 : 20;
			uint32_t cl_capture =
			    (uint32_t)convert_epochtime(v.back()->header.year, v.back()->header.month, v.back()->header.day,
				                            v.back()->header.hours, v.back()->header.minutes, v.back()->header.seconds);
			packet = ArgosPacketBuilder::build_cloudlocate_packet(blob, blob_size, format_id,
			                                                      v.back()->info.batt_voltage, argos_config.is_lb,
			                                                      cl_capture, (uint32_t)service_current_time());
			size_bits = ArgosPacketBuilder::cloudlocate_packet_bits(format_id);
			// Modulation policy: see process_sensor_burst for full rationale.
			//   adaptive=ON  → pick optimal mod from format, switch SMD.
			//   adaptive=OFF → use live SMD modulation, NOT m_scheduled_mode
			//                  (which line 207 hardcodes to LDA2 in SURFACING_BURST).
			if (argos_config.adaptive_modulation) {
				m_scheduled_mode = (format_id == (uint8_t)BaseCloudLocateFormat::MEASC12) ? KineisModulation::LDK
				                                                                          : KineisModulation::LDA2;
				if (!ensure_modulation(m_scheduled_mode)) {
					DEBUG_WARN("ArgosTxService::process_gnss_burst: CloudLocate modulation switch failed");
					m_scheduled_mode = m_kineis.get_current_modulation();
					if (!can_transmit_on(m_scheduled_mode, size_bits)) {
						DEBUG_ERROR("ArgosTxService::process_gnss_burst: CloudLocate payload %u bits doesn't fit "
						            "fallback mod %d — skipping TX",
						            size_bits, (int)m_scheduled_mode);
						service_complete();
						return;
					}
				}
			} else {
				m_scheduled_mode = m_kineis.get_current_modulation();
				if (!can_transmit_on(m_scheduled_mode, size_bits)) {
					DEBUG_ERROR("ArgosTxService::process_gnss_burst: CloudLocate payload %u bits doesn't fit master "
					            "mod %d (ARGOS_AD_MOD=0) — skipping TX",
					            size_bits, (int)m_scheduled_mode);
					service_complete();
					return;
				}
			}
			// Demoted to TRACE: per-TX payload dump.
			DEBUG_TRACE("ArgosTxService::process_gnss_burst: CloudLocate fmt=%u mode=%s data=%s", format_id,
			            argos_modulation_to_string((BaseArgosModulation)m_scheduled_mode),
			            Binascii::hexlify(packet).c_str());
			m_last_tx_had_gps = true;
			m_last_val_tx_type = "cloudloc";
			m_kineis.send(m_scheduled_mode, packet, size_bits);
			return;
		}

		// A fastloc entry produces a full LDA2 frame (FASTLOC_PACKET_BITS equals
		// LDA2_FRAME_BITS = 192 bits). In adaptive we therefore aim for LDA2
		// straight away. In NON-adaptive we do not touch the modulation resolved
		// at scheduling: the master RCONF prevails, and the size guard of the
		// `else` block further down performs the escalation to LDA2 by calling
		// ensure_modulation() — hence by actually programming the module, which
		// the direct overwrite did not do.
		if (v.back()->info.event_type == GPSEventType::FASTLOC) {
			packet = ArgosPacketBuilder::build_fastloc_packet(v.back(), argos_config.is_lb);
			size_bits = ArgosPacketBuilder::FASTLOC_PACKET_BITS;
			if (argos_config.adaptive_modulation) m_scheduled_mode = KineisModulation::LDA2;
		} else {
			// Filter out any CloudLocate/fastloc entries that may be mixed in
			v.erase(std::remove_if(v.begin(), v.end(),
			                       [](const GPSLogEntry *e) {
				                       return e->info.event_type == GPSEventType::CLOUDLOCATE
				                              || e->info.event_type == GPSEventType::FASTLOC;
			                       }),
			        v.end());
			if (v.empty()) {
				DEBUG_WARN("ArgosTxService::process_gnss_burst: all entries filtered (mixed types)");
				service_complete();
				return;
			}
			for (unsigned int i = 0; i < v.size(); i++) {
				// Demoted to TRACE: per-entry payload dump in GNSS burst hot path.
				DEBUG_TRACE("TX_RAW: GNSS[%u] lat=%.6f lon=%.6f hAcc=%u nSV=%u hDOP=%.1f batt=%umV", i, v[i]->info.lat,
				            v[i]->info.lon, v[i]->info.hAcc, v[i]->info.numSV, (double)v[i]->info.hDOP,
				            (unsigned)v[i]->info.batt_voltage);
			}
			packet = ArgosPacketBuilder::build_gnss_packet(v, argos_config.is_out_of_zone, argos_config.is_lb,
			                                               argos_config.delta_time_loc, size_bits);
		}

		// Adaptive modulation: a SHORT packet (96 bits) fits in LDK, a LONG
		// (192) requires LDA2. Careful: the FASTLOC packet is 192 bits too
		// (FASTLOC_PACKET_BITS = LDA2_FRAME_BITS), contrary to what this
		// comment used to say — which is why the size guard below did not catch
		// it when the modulation was overwritten to LDA2 just before.
		if (argos_config.adaptive_modulation) {
			m_scheduled_mode = (size_bits <= 128) ? KineisModulation::LDK : KineisModulation::LDA2;
			if (!ensure_modulation(m_scheduled_mode)) {
				DEBUG_WARN("ArgosTxService::process_gnss_burst: modulation switch failed, using current");
				m_scheduled_mode = m_kineis.get_current_modulation();
				if (!can_transmit_on(m_scheduled_mode, size_bits)) {
					DEBUG_ERROR("ArgosTxService::process_gnss_burst: GNSS payload %u bits doesn't fit fallback mod %d "
					            "— skipping TX",
					            size_bits, (int)m_scheduled_mode);
					service_complete();
					return;
				}
			}
		} else {
			// Non-adaptive: m_scheduled_mode is the master RCONF modulation. The
			// retrieve above already capped the burst to a single fix when the
			// master modulation can't hold a LONG packet, so the common LDK case
			// now ships a 96-bit SHORT packet that fits. If the built packet STILL
			// doesn't fit (e.g. master is VLDA4 = 24b — even a single 96-bit fix
			// overflows), fall back to LDA2 when it's provisioned; otherwise skip
			// this TX cleanly rather than letting KIM2 send() drop it silently.
			// User policy 2026-06-17: keep master modulation; on overflow → LDA2.
			if (!can_transmit_on(m_scheduled_mode, size_bits)) {
				if (can_transmit_on(KineisModulation::LDA2, size_bits)
				    && ensure_modulation(KineisModulation::LDA2)) {
					DEBUG_WARN(
					    "ArgosTxService::process_gnss_burst: %u bits don't fit master mod %d — falling back to LDA2",
					    size_bits, (int)m_scheduled_mode);
					m_scheduled_mode = KineisModulation::LDA2;
				} else {
					DEBUG_ERROR(
					    "ArgosTxService::process_gnss_burst: %u bits fit no provisioned modulation — skipping TX",
					    size_bits);
					service_complete();
					return;
				}
			}
		}

		// Demoted to TRACE: per-TX payload dump.
		DEBUG_TRACE("ArgosTxService::process_gnss_burst: mode=%s data=%s sz=%u",
		            argos_modulation_to_string((BaseArgosModulation)m_scheduled_mode),
		            Binascii::hexlify(packet).c_str(), size_bits);
		m_last_tx_had_gps = true;
		// fastloc fallback uses LDA2 + 96-bit packet → still attribute as "gnss"
		// at this site; the inner fastloc branch above already tagged "fastloc".
		m_last_val_tx_type = (v.back()->info.event_type == GPSEventType::FASTLOC) ? "fastloc" : "gnss";
		m_kineis.send(m_scheduled_mode, packet, size_bits);
	} else {
		DEBUG_WARN("ArgosTxService::process_gnss_burst: no entries eligible in depth pile");
		if (m_is_surfacing_burst || m_has_gnss_fix_since_surfacing) {
			DEBUG_INFO("ArgosTxService::process_gnss_burst: ending surfacing burst (depth pile exhausted)");
			m_is_surfacing_burst = false;
			m_awaiting_surfacing = true;
			m_has_gnss_fix_since_surfacing = false;
			m_first_gnss_tx_sent = false;
		}
		service_complete();
	}
}

/// @brief REUSE_LAST GNSS burst — TX a GNSS Argos packet from the most recent
/// cached depth-pile fix WITHOUT powering the GPS. Used by HAULED mode (and
/// later by Plan 2 sequencer phases) when battery matters more than positional
/// freshness. Falls back to process_doppler_burst() if no usable fix.
void ArgosTxService::process_gnss_burst_from_cached() {
	DEBUG_TRACE("ArgosTxService::process_gnss_burst_from_cached");

	GPSLogEntry cached;
	if (!read_cached_last_fix(cached)) {
		DEBUG_INFO("ArgosTxService::process_gnss_burst_from_cached: no usable cached fix — falling back to Doppler");
		process_doppler_burst();
		return;
	}

	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);

	unsigned int age_s = compute_gps_log_age_seconds(cached, service_current_time());
	DEBUG_INFO("ArgosTxService::process_gnss_burst_from_cached: lat=%.6f lon=%.6f hAcc=%u age=%u s", cached.info.lat,
	           cached.info.lon, cached.info.hAcc, age_s);

	// build_gnss_packet expects a vector<GPSLogEntry*>. Use a stack-local
	// vector pointing at our cached copy; the builder is read-only.
	std::vector<GPSLogEntry *> v;
	v.push_back(&cached);

	unsigned int size_bits;
	KineisPacket packet = ArgosPacketBuilder::build_gnss_packet(v, argos_config.is_out_of_zone, argos_config.is_lb,
	                                                            argos_config.delta_time_loc, size_bits);

	// Adaptive modulation: same logic as process_gnss_burst (single-entry
	// packet always fits LDK at 96/128 bits, so we prefer LDK for power).
	if (argos_config.adaptive_modulation) {
		m_scheduled_mode = (size_bits <= 128) ? KineisModulation::LDK : KineisModulation::LDA2;
		if (!ensure_modulation(m_scheduled_mode)) {
			DEBUG_WARN("ArgosTxService::process_gnss_burst_from_cached: modulation switch failed, using current");
			m_scheduled_mode = m_kineis.get_current_modulation();
			if (!can_transmit_on(m_scheduled_mode, size_bits)) {
				DEBUG_ERROR("ArgosTxService::process_gnss_burst_from_cached: packet %u bits doesn't fit fallback mod "
				            "%d — skipping TX",
				            size_bits, (int)m_scheduled_mode);
				service_complete();
				return;
			}
		}
	} else {
		// Non-adaptive: keep the master RCONF modulation. REUSE_LAST is always a
		// single fix (96 bits), which fits LDK/LDA2; only a VLDA4 master (24b)
		// can't hold it — fall back to LDA2 if provisioned, else skip cleanly
		// instead of a silent KIM2 oversize drop. User policy 2026-06-17.
		if (!can_transmit_on(m_scheduled_mode, size_bits)) {
			if (can_transmit_on(KineisModulation::LDA2, size_bits) && ensure_modulation(KineisModulation::LDA2)) {
				DEBUG_WARN("ArgosTxService::process_gnss_burst_from_cached: %u bits don't fit master mod %d — falling "
				           "back to LDA2",
				           size_bits, (int)m_scheduled_mode);
				m_scheduled_mode = KineisModulation::LDA2;
			} else {
				DEBUG_ERROR("ArgosTxService::process_gnss_burst_from_cached: %u bits fit no provisioned modulation — "
				            "skipping TX",
				            size_bits);
				service_complete();
				return;
			}
		}
	}

	// Demoted to TRACE: per-TX payload dump.
	DEBUG_TRACE("ArgosTxService::process_gnss_burst_from_cached: REUSE_LAST TX mode=%s data=%s sz=%u",
	            argos_modulation_to_string((BaseArgosModulation)m_scheduled_mode), Binascii::hexlify(packet).c_str(),
	            size_bits);
	m_last_tx_had_gps = true;
	m_last_val_tx_type = "reuse_last";
	m_kineis.send(m_scheduled_mode, packet, size_bits);
}

/// @brief Pre-build the first surfacing-burst Doppler packet while underwater.
/// Samples the battery and builds the Doppler payload now so the first TX at
/// surface skips the ADC read + packet build on its critical path. Only acts in
/// SURFACING_BURST mode; on other modes the prepared state is cleared.
void ArgosTxService::prepare_doppler_packet() {
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);
	if (argos_config.mode != BaseArgosMode::SURFACING_BURST) {
		m_prepared_doppler_packet.clear();
		m_prepared_doppler_size_bits = 0;
		m_prepared_at_ms = 0;
		return;
	}

	service_update_battery();
	unsigned int size_bits = 0;
	KineisPacket packet;
#if defined(BOARD_RSPB) && ENABLE_MORTALITY_SENSOR
	unsigned int mort_conf = 0;
	uint8_t activity = 0;
	if (mortality_service) {
		mort_conf = mortality_service->get_confidence();
		activity = mortality_service->get_last_activity();
	}
	packet = ArgosPacketBuilder::build_rspb_doppler_packet(service_get_level(), activity, mort_conf, size_bits);
#else
	packet = ArgosPacketBuilder::build_doppler_packet(service_get_voltage(), service_is_battery_level_low(), size_bits);
#endif

	m_prepared_doppler_packet = packet;
	m_prepared_doppler_size_bits = size_bits;
	// 2026-05-25 modulation fix (companion to argos_tx_service.cpp:225):
	// non-adaptive path was hardcoded to LDA2. The prewarm send (line 1369)
	// deliberately SKIPS ensure_modulation() to keep the surfacing first-TX
	// latency minimal — meaning whatever mode lands here ships exactly as-is.
	// With the hardcoded LDA2 + a non-LDA2 master RCONF (e.g. user-configured
	// LDK), the first ping fired LDA2 while SmdSat held LDK → "TX mode 0 !=
	// current modulation 1" WARN per surface (observed in v4.1.8-1 logs).
	//
	// Using resolve_non_adaptive_modulation() (which returns the cached
	// modulation from m_kineis — already aligned with the saved master RCONF)
	// lets the prewarm path ship in the *correct* modulation with zero extra
	// latency, zero ensure_modulation() call, zero STM32 flash write. The
	// SMD already persists the master modulation across reboots via
	// write_credentials_from_config + save_radio_conf — nothing to add.
	m_prepared_doppler_mode =
	    argos_config.adaptive_modulation ? adaptive_doppler_modulation() : resolve_non_adaptive_modulation();
	m_prepared_at_ms = PMU::get_timestamp_ms();
}

/// @brief Build and send Doppler burst (24-bit, no GPS — or RSPB Doppler with mortality).
void ArgosTxService::process_doppler_burst() {
	DEBUG_TRACE("ArgosTxService::process_doppler_burst");
	unsigned int size_bits;

	// Pre-warm fast path: first Doppler of a surfacing burst can ship the
	// payload built while underwater — skip the ADC read + packet build on
	// the surface critical path. m_doppler_burst_count is post-increment from
	// service_initiate(), so == 1 means this is the first ping. Anything else
	// (legacy DOPPLER mode, count > 1, missing prep, mode changed since prep,
	// stale prep older than the refresh window) falls through to the normal
	// build below. The freshness guard rejects any prep that survived from a
	// previous session or aged past the underwater refresh window.
	{
		ArgosConfig pw_cfg;
		configuration_store->get_argos_configuration(pw_cfg);
		uint64_t prep_age_ms = (m_prepared_at_ms != 0) ? (PMU::get_timestamp_ms() - m_prepared_at_ms) : UINT64_MAX;
		if (m_is_surfacing_burst && m_doppler_burst_count == 1 && pw_cfg.mode == BaseArgosMode::SURFACING_BURST
		    && !m_prepared_doppler_packet.empty() && m_prepared_at_ms != 0
		    && prep_age_ms < PREPARED_DOPPLER_REFRESH_MS) {
			KineisModulation tx_mode = m_prepared_doppler_mode;
			if (pw_cfg.adaptive_modulation) {
				tx_mode = adaptive_doppler_modulation();
				if (!ensure_modulation(tx_mode)) {
					DEBUG_WARN(
					    "ArgosTxService::process_doppler_burst: pre-warmed modulation switch failed, using current");
					tx_mode = m_kineis.get_current_modulation();
				}
			}
			DEBUG_TRACE("ArgosTxService::process_doppler_burst: PREWARM mode=%s sz=%u age=%lu ms",
			            argos_modulation_to_string((BaseArgosModulation)tx_mode), m_prepared_doppler_size_bits,
			            static_cast<unsigned long>(prep_age_ms));
			m_last_tx_had_gps = false;
			KineisPacket prepared = m_prepared_doppler_packet;
			unsigned int prepared_bits = m_prepared_doppler_size_bits;
			m_prepared_doppler_packet.clear();
			m_prepared_doppler_size_bits = 0;
			m_prepared_at_ms = 0;
			m_last_val_tx_type = "doppler-prewarm";
			m_kineis.send(tx_mode, prepared, prepared_bits);
			return;
		}
	}

	service_update_battery();
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);

	// Progressive CloudLocate: when running a SURFACING_BURST and raw GNSS
	// measurements are available, replace the Doppler payload with a CloudLocate
	// packet. The `m_doppler_burst_count > 0` check is what gates this off in
	// legacy DOPPLER mode (where the counter is never incremented — see
	// service_initiate()) — it does NOT protect the first ping of a burst.
	unsigned int fastloc_mode = configuration_store->read_param<unsigned int>(ParamID::GNSS_FASTLOC_MODE);
	if (fastloc_mode == (unsigned int)BaseFastlocMode::CLOUDLOCATE && m_doppler_burst_count > 0 && gps_device
	    && gps_device->has_raw_measurement()) {
		GNSSRawMeasurement raw = gps_device->get_raw_measurement();
		unsigned int cl_format = configuration_store->read_param<unsigned int>(ParamID::GNSS_CLOUDLOCATE_FORMAT);

		// STRICT format policy (2026-06): use ONLY the operator-configured format.
		// No cross-format fallback — if the configured format wasn't produced this
		// session, blob stays null and the CloudLocate TX is skipped (see `if (blob)`
		// below) rather than silently emitting a different format the operator did
		// not select. Format is independent of the Argos modulation.
		const uint8_t *blob = nullptr;
		unsigned int blob_size = 0;
		uint8_t format_id = 0;
		if (cl_format == (unsigned int)BaseCloudLocateFormat::MEASC12 && raw.has_measc12) {
			blob = raw.measc12;
			blob_size = 12;
			format_id = (uint8_t)BaseCloudLocateFormat::MEASC12;
		} else if (cl_format == (unsigned int)BaseCloudLocateFormat::MEAS20 && raw.has_meas20) {
			blob = raw.meas20;
			blob_size = 20;
			format_id = (uint8_t)BaseCloudLocateFormat::MEAS20;
		}
		if (!blob) {
			DEBUG_WARN("ArgosTxService::process_doppler_burst: configured CloudLocate format %u not available "
			           "(measc12=%u meas20=%u) — skipping CL TX",
			           cl_format, (unsigned)raw.has_measc12, (unsigned)raw.has_meas20);
		}

		if (blob) {
			// Demoted to TRACE: per-ping TX_RAW dump on the surfacing-burst hot
			// path adds ~50-300 ms of LFS commit per emit.
			DEBUG_TRACE("TX_RAW: CL fmt=%u sz=%u batt=%umV blob=%s", format_id, blob_size,
			            (unsigned)service_get_voltage(),
			            Binascii::hexlify(std::string((const char *)blob, blob_size)).c_str());
			KineisPacket packet = ArgosPacketBuilder::build_cloudlocate_packet(
			    blob, blob_size, format_id, service_get_voltage(), argos_config.is_lb, raw.capture_time,
			    (uint32_t)service_current_time());
			size_bits = ArgosPacketBuilder::cloudlocate_packet_bits(format_id);

			// Modulation policy: see process_sensor_burst for full rationale.
			//   adaptive=ON  → pick optimal mod from format, switch SMD.
			//   adaptive=OFF → use live SMD modulation. m_scheduled_mode is NOT
			//                  reliable here — line 207 hardcodes LDA2 in
			//                  SURFACING_BURST regardless of master config.
			KineisModulation tx_mode;
			if (argos_config.adaptive_modulation) {
				tx_mode = (format_id == (uint8_t)BaseCloudLocateFormat::MEASC12) ? KineisModulation::LDK
				                                                                 : KineisModulation::LDA2;
				if (!ensure_modulation(tx_mode)) {
					DEBUG_WARN("ArgosTxService::process_doppler_burst: CloudLocate modulation switch failed");
					tx_mode = m_kineis.get_current_modulation();
					if (!size_fits_modulation(size_bits, tx_mode)) {
						DEBUG_ERROR("ArgosTxService::process_doppler_burst: CloudLocate payload %u bits doesn't fit "
						            "fallback mod %d — skipping TX",
						            size_bits, (int)tx_mode);
						service_complete();
						return;
					}
				}
			} else {
				tx_mode = m_kineis.get_current_modulation();
				if (!size_fits_modulation(size_bits, tx_mode)) {
					DEBUG_ERROR("ArgosTxService::process_doppler_burst: CloudLocate payload %u bits doesn't fit master "
					            "mod %d (ARGOS_AD_MOD=0) — skipping TX",
					            size_bits, (int)tx_mode);
					service_complete();
					return;
				}
			}

			DEBUG_TRACE("ArgosTxService::process_doppler_burst: CLOUDLOCATE #%u fmt=%u sz=%u mode=%s",
			            m_doppler_burst_count, format_id, blob_size,
			            argos_modulation_to_string((BaseArgosModulation)tx_mode));
			m_last_tx_had_gps = true;
			m_last_val_tx_type = "cloudloc-surf";
			m_kineis.send(tx_mode, packet, size_bits);
			return;
		}
	}

	// Progressive fastloc: when running a SURFACING_BURST and the GPS has a
	// degraded PVT available, replace the Doppler payload with a fastloc packet.
	// The position improves over time as the GPS refines its fix. As above, the
	// `m_doppler_burst_count > 0` check gates legacy DOPPLER mode, not the first
	// ping of a burst.
	if (fastloc_mode >= (unsigned int)BaseFastlocMode::DEGRADED_PVT && m_doppler_burst_count > 0 && gps_device
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
		fastloc_entry.info.onTime = degraded.ttff;  // Best approximation of GPS on time
		fastloc_entry.info.batt_voltage = service_get_voltage();
		fastloc_entry.info.schedTime = service_current_time();
		fastloc_entry.info.valid = true;
		fastloc_entry.info.event_type = GPSEventType::FASTLOC;

		// Demoted to TRACE: per-ping TX_RAW dump on the surfacing-burst hot
		// path adds ~50-300 ms of LFS commit per emit.
		DEBUG_TRACE("TX_RAW: FLOC lat=%.6f lon=%.6f hAcc=%u nSV=%u hDOP=%.1f batt=%umV", degraded.lat, degraded.lon,
		            degraded.hAcc, degraded.numSV, (double)degraded.hDOP, (unsigned)service_get_voltage());
		KineisPacket packet = ArgosPacketBuilder::build_fastloc_packet(&fastloc_entry, argos_config.is_lb);
		size_bits = ArgosPacketBuilder::FASTLOC_PACKET_BITS;

		KineisModulation tx_mode = KineisModulation::LDA2;
		if (argos_config.adaptive_modulation) {
			if (!ensure_modulation(tx_mode)) {
				DEBUG_WARN("ArgosTxService::process_doppler_burst: fastloc modulation switch failed, using current");
				tx_mode = m_kineis.get_current_modulation();
				if (!size_fits_modulation(size_bits, tx_mode)) {
					DEBUG_ERROR("ArgosTxService::process_doppler_burst: fastloc payload %u bits doesn't fit fallback "
					            "mod %d — skipping TX",
					            size_bits, (int)tx_mode);
					service_complete();
					return;
				}
			}
		} else {
			// Non-adaptive: same safety escalation as the three sibling branches
			// of this processor (prewarm, CloudLocate, cached position), which do
			// apply it. Without it the surface ping left in LDA2 on a module still
			// on the master.
			if (!ensure_modulation(KineisModulation::LDA2)) {
				DEBUG_ERROR(
				    "ArgosTxService::process_doppler_burst: fastloc %u bits need LDA2, not provisioned — skipping TX",
				    size_bits);
				service_complete();
				return;
			}
		}

		DEBUG_TRACE("ArgosTxService::process_doppler_burst: FASTLOC #%u hAcc=%um numSV=%u mode=%s data=%s",
		            m_doppler_burst_count, degraded.hAcc, degraded.numSV,
		            argos_modulation_to_string((BaseArgosModulation)tx_mode), Binascii::hexlify(packet).c_str());
		m_last_tx_had_gps = true;
		m_last_val_tx_type = "fastloc-degr";
		m_kineis.send(tx_mode, packet, size_bits);
		return;
	}

	// Cached position branch — used only on ping #2+ (m_doppler_burst_count > 1)
	// when no live CloudLocate raw or degraded PVT is available this surface.
	// Sends the most-recent cached position between last GPS fix and last
	// Fastloc/degraded PVT (RAM cache populated by gps_service).
	//
	// Ping #1 is intentionally skipped to preserve the §5 priority-3 fast
	// first-TX path: the existing pre-warmed Doppler (lines 1324-1357) stays
	// untouched. This is the prudent "Approach A" — no change to the surface
	// critical-path latency, position is added starting at ping #2.
	//
	// Falls through to standard Doppler if cache is empty or modulation
	// constraints prevent TX.
	if (m_doppler_burst_count > 1) {
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
			bool is_fastloc = (entry.info.event_type == GPSEventType::FASTLOC);

			KineisPacket cached_packet =
			    is_fastloc
			        ? ArgosPacketBuilder::build_fastloc_packet(&entry, argos_config.is_lb)
			        : ArgosPacketBuilder::build_short_packet(&entry, argos_config.is_out_of_zone, argos_config.is_lb);
			unsigned int cached_size_bits =
			    is_fastloc ? ArgosPacketBuilder::FASTLOC_PACKET_BITS : ArgosPacketBuilder::SHORT_PACKET_BITS;

			// Modulation: adaptive forces LDA2 (universal for both 96-bit short
			// and 192-bit fastloc). Non-adaptive trusts the master modulation;
			// fall through to Doppler if the cached payload doesn't fit
			// (e.g. VLDA4 master with 96-bit short_packet — short doesn't fit 24).
			KineisModulation cached_mode = KineisModulation::LDA2;
			bool can_send = true;
			if (argos_config.adaptive_modulation) {
				if (!ensure_modulation(cached_mode)) {
					DEBUG_WARN(
					    "ArgosTxService::process_doppler_burst: cached-pos modulation switch failed, trying current");
					cached_mode = m_kineis.get_current_modulation();
					if (!size_fits_modulation(cached_size_bits, cached_mode)) {
						DEBUG_WARN("ArgosTxService::process_doppler_burst: cached-pos %u bits doesn't fit fallback mod "
						           "%d — falling through to Doppler",
						           cached_size_bits, (int)cached_mode);
						can_send = false;
					}
				}
			} else {
				cached_mode = resolve_non_adaptive_modulation();
				if (!size_fits_modulation(cached_size_bits, cached_mode)) {
					DEBUG_WARN("ArgosTxService::process_doppler_burst: cached-pos %u bits doesn't fit master mod %d "
					           "(ARGOS_AD_MOD=0) — falling through to Doppler",
					           cached_size_bits, (int)cached_mode);
					can_send = false;
				}
			}

			if (can_send) {
				// Demoted to TRACE: per-ping payload dump on hot path.
				DEBUG_TRACE("ArgosTxService::process_doppler_burst: %s #%u lat=%lf lon=%lf mode=%s data=%s",
				            is_fastloc ? "CACHED_FASTLOC" : "CACHED_GPS", m_doppler_burst_count, entry.info.lat,
				            entry.info.lon, argos_modulation_to_string((BaseArgosModulation)cached_mode),
				            Binascii::hexlify(cached_packet).c_str());
				m_last_tx_had_gps = true;
				m_last_val_tx_type = is_fastloc ? "fastloc-cached" : "short-cached";
				m_kineis.send(cached_mode, cached_packet, cached_size_bits);
				return;
			}
			// else fall through to standard Doppler below
		}
	}

	// Standard Doppler packet (first ping, or fastloc not available)
	KineisPacket packet;

#if defined(BOARD_RSPB) && ENABLE_MORTALITY_SENSOR
	// RSPB Doppler: battery SOC + activity + mortality (Type 6)
	unsigned int mort_conf = 0;
	uint8_t activity = 0;
	if (mortality_service) {
		mort_conf = mortality_service->get_confidence();
		activity = mortality_service->get_last_activity();
	}
	// Demoted to TRACE: redundant on the surfacing-burst hot path. The
	// "process_doppler_burst" log below already carries the data hex; battery
	// voltage / soc visible via dedicated DTE param query.
	DEBUG_TRACE("TX_RAW: DOPP soc=%u activity=%u mortality=%u", service_get_level(), activity, mort_conf);
	packet = ArgosPacketBuilder::build_rspb_doppler_packet(service_get_level(), activity, mort_conf, size_bits);
#else
	// Standard Doppler: battery voltage only
	DEBUG_TRACE("TX_RAW: DOPP batt=%umV low_batt=%u", (unsigned)service_get_voltage(),
	            service_is_battery_level_low() ? 1U : 0U);
	packet = ArgosPacketBuilder::build_doppler_packet(service_get_voltage(), service_is_battery_level_low(), size_bits);
#endif

	// Adaptive modulation: Doppler = 24 bits = VLDA4.
	// Non-adaptive: honor the master RCONF's modulation (LDK/LDA2/VLDA4 all
	// accept a 24-bit Doppler payload — KIM2 pads per modulation). LDA2
	// fallback is for SMD users (m_modulation stays at LDA2) and the very
	// first KIM2 boot before state_init has read back the actual modulation.
	KineisModulation tx_mode;
	if (argos_config.adaptive_modulation) {
		tx_mode = adaptive_doppler_modulation();
		if (!ensure_modulation(tx_mode)) {
			DEBUG_WARN("ArgosTxService::process_doppler_burst: modulation switch failed, using current");
			tx_mode = m_kineis.get_current_modulation();
		}
	} else {
		tx_mode = resolve_non_adaptive_modulation();
	}

	// Demoted to TRACE: METRIC-SURF and METRIC-FIRST-TX already mark the burst
	// boundaries with absolute timestamps. This per-TX dump adds ~50-300 ms LFS
	// commit on the critical surfacing path.
	DEBUG_TRACE("ArgosTxService::process_doppler_burst: mode=%s data=%s sz=%u",
	            argos_modulation_to_string((BaseArgosModulation)tx_mode), Binascii::hexlify(packet).c_str(), size_bits);
	m_last_tx_had_gps = false;
	m_last_val_tx_type = "doppler";
	m_kineis.send(tx_mode, packet, size_bits);
}

/// @brief TX started event — notify service manager that TX is in progress.
void ArgosTxService::react(KineisEventTxStarted const &) {
	DEBUG_TRACE("ArgosTxService::react: KineisEventTxStarted");
#if !defined(ARGOS_SMD) || (ARGOS_SMD != 1)
	// KIM2 only: surface the burst type + modulation + running session TX count
	// so the operator can see WHAT is being sent and HOW MANY this session.
	DEBUG_INFO("ArgosTxService: TX START — type=%s mode=%d session_tx#=%u", m_last_val_tx_type,
	           static_cast<int>(m_scheduled_mode), m_session_tx_count + 1);
#endif
	service_active();
}

/// @brief TX complete event — update counters, manage surfacing burst, complete service.
void ArgosTxService::react(KineisEventTxComplete const &) {
	DEBUG_TRACE("ArgosTxService::react: KineisEventTxComplete");
#if !defined(ARGOS_SMD) || (ARGOS_SMD != 1)
	// KIM2 only: clear, visible confirmation that the Argos message went out.
	DEBUG_INFO("ArgosTxService: TX SUCCESS — type=%s mode=%d session_tx#=%u", m_last_val_tx_type,
	           static_cast<int>(m_scheduled_mode), m_session_tx_count + 1);
#endif
	m_is_tx_pending = false;
	m_consecutive_device_errors = 0;
	m_device_error_hold_until = 0;

#if VALIDATION_LOG_ENABLE
	{
		std::time_t now_v = service_current_time();
		// Spacing in seconds from the previous TX complete. 0 if first TX of
		// the session (m_last_val_tx_t == 0) — distinguishable from a true
		// 0 s gap, which is physically impossible (TX takes ~360-720 ms).
		unsigned int spacing_s = 0;
		if (m_last_val_tx_t > 0 && now_v > m_last_val_tx_t)
			spacing_s = static_cast<unsigned int>(now_v - m_last_val_tx_t);
		// `burst=on` means the TX is inside a SURFACING_BURST sequence and
		// m_doppler_burst_count was incremented by service_initiate. `burst=off`
		// means the TX fired outside the burst (legacy DOPPLER mode, or first
		// TX after boot that runs before the surface event has propagated to
		// ArgosTxService — counter stays 0). The previous TX seen at boot in
		// the field log on 2026-05-23 with burst#=0 was such a pre-surface-event
		// "legacy" TX, not an off-by-one bug.
		DEBUG_INFO("[VAL-TX] type=%s t=%u spacing_s=%u burst=%s dop_count=%u session#=%u", m_last_val_tx_type,
		           (unsigned int)now_v, spacing_s, m_is_surfacing_burst ? "on" : "off", m_doppler_burst_count,
		           m_session_tx_count + 1);
		m_last_val_tx_t = now_v;
	}
#endif

	// Restore TCXO warmup after first TX post-submerge
	if (m_tcxo_skip_on_next_tx) {
		ArgosConfig argos_config;
		configuration_store->get_argos_configuration(argos_config);
		DEBUG_TRACE("ArgosTxService::react: restoring TCXO warmup to %u s", argos_config.argos_tcxo_warmup_time);
		m_kineis.set_tcxo_warmup_time(argos_config.argos_tcxo_warmup_time);
		m_tcxo_skip_on_next_tx = false;
	}

	// BLIND: this single +TX corresponds to a module-owned burst of retx_nb copies
	// on air. Count them all toward the airtime/session safety budgets so
	// NTIME_SAT and the rate limiter keep bounding actual airtime.
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);
	unsigned int tx_copies = (argos_config.blind_en && argos_config.blind_retx_nb > 0) ? argos_config.blind_retx_nb : 1;

	// Increment TX counter
	configuration_store->increment_tx_counter();
	m_session_tx_count += tx_copies;

	// Update last TX date time
	std::time_t t = service_current_time();
	configuration_store->write_param(ParamID::LAST_TX, t);

	// Counters updated in RAM — flash persistence deferred to periodic flush / powerdown

	// Check session TX limit (SHUTDOWN_NTIME_SAT / LB_SHUTDOWN_NTIME_SAT).
	//
	// Read "session" precisely: m_session_tx_count is reset in service_init only
	// -- at boot, or on re-entry into Operational -- and by nothing else. Not by
	// a dive, not by a surfacing, not by the end of a GNSS session. On a board
	// whose duty cycle reboots it (RSPB, where the TPL5111 cuts power and the
	// MCU_DONE line ends each cycle) that is the intended meaning and the budget
	// genuinely resets every cycle. On a board that stays powered, "session"
	// means the whole deployment, and this becomes a lifetime cap: at TR_NOM=60 s
	// a budget of 100 ends the beacon 100 minutes in.
	//
	// So it is logged at ERROR, with the count and the parameter that did it.
	// This is a deliberate stop, not a fault -- but it is indistinguishable from
	// a dead beacon afterwards, and the log is the only thing that can tell the
	// two apart on a recovered device.
	if (argos_config.shutdown_ntime_sat > 0 && m_session_tx_count >= argos_config.shutdown_ntime_sat) {
#if defined(BOARD_RSPB)
		DEBUG_ERROR("ArgosTxService: session TX budget reached (%u/%u) — powering down ON PURPOSE "
		            "(SHUTDOWN_NTIME_SAT / LB_SHUTDOWN_NTIME_SAT). The TPL5111 duty cycle restores power and the "
		            "counter resets on the next boot.",
		            m_session_tx_count, argos_config.shutdown_ntime_sat);
		configuration_store->save_params();  // Flush before shutdown
		PMU::powerdown();
		return;
#else
		// RSPB only. There the TPL5111 cuts power and MCU_DONE ends each duty
		// cycle, so "session" really is one cycle and the budget resets on the
		// next boot. On a board that stays powered, m_session_tx_count is reset
		// in service_init and nowhere else -- so a parameter documented as a
		// per-session cap would silently become a one-shot lifetime cap and end
		// the deployment. At TR_NOM=60 s a budget of 100 would kill the beacon
		// 100 minutes in. Honour the parameter where it means what it says,
		// ignore it where it does not, and say so once.
		if (!m_ntime_sat_ignored_logged) {
			m_ntime_sat_ignored_logged = true;
			DEBUG_WARN("ArgosTxService: session TX budget reached (%u/%u) but SHUTDOWN_NTIME_SAT only applies to "
			           "RSPB, whose duty cycle power-cycles the board. Ignored here — it would be a lifetime cap, "
			           "not a per-session one. Transmission continues.",
			           m_session_tx_count, argos_config.shutdown_ntime_sat);
		}
#endif
	}

	// Post-TX adaptive modulation: pre-switch to VLDA4 while SMD is still
	// powered on so the *next surfacing's* Doppler #1 boots with RCONF already
	// in VLDA4 → no KMAC reload, no deferred RCONF apply in state_load_kmac.
	// The first ping at surface must be as fast as possible (user requirement).
	//
	// Note: SmdSat::state_transmitting transitions directly to `stopped` after
	// every TX (smd_sat.cpp:639), so SMD is fully powered off during inter-TX
	// gaps. The flash write done here persists in STM32WL flash and survives
	// the power-off; the next power-on auto-inits MAC from flash (no SPI
	// KMAC reload needed when RCONF hasn't changed — see state_load_kmac).
	//
	// Guards (skip the pre-switch):
	//   - next_likely_lda2: degraded PVT waiting for better fix → next TX is
	//     fastloc in LDA2, don't churn to VLDA4 in between.
	//   - fix_just_arrived: GNSS fix landed during this (Doppler/CL/FLOC) TX
	//     and the first GNSS TX is about to fire immediately. Pre-switching
	//     would be undone 0 ms later when process_gnss_burst() ensures LDK.
	//     Narrow window (<1 s between TX-complete and next TX-start) — low
	//     risk of dive interruption here.
	//
	// Every other post-TX path pre-switches unconditionally when current is
	// not VLDA4 — including every GNSS-phase TX. Yes, this costs one flash
	// write per GNSS TX (VLDA4 written here, LDK re-written at the next
	// ensure_modulation()), but it GUARANTEES that any dive mid-burst leaves
	// the STM32WL flash in VLDA4, so the next surfacing's first ping skips
	// the deferred-RCONF path entirely.
	if (argos_config.adaptive_modulation && argos_config.mode == BaseArgosMode::SURFACING_BURST) {
		bool next_likely_lda2 = (gps_device && gps_device->has_degraded_pvt() && !m_has_gnss_fix_since_surfacing
		                         && m_doppler_burst_count > 0);
		bool fix_just_arrived = (m_has_gnss_fix_since_surfacing && !m_first_gnss_tx_sent);

		KineisModulation next_doppler = adaptive_doppler_modulation();
		if (!next_likely_lda2 && !fix_just_arrived && m_kineis.get_current_modulation() != next_doppler) {
			DEBUG_INFO("ArgosTxService::react: pre-switch to modulation %d for next Doppler", (int)next_doppler);
			ensure_modulation(next_doppler);
		}
	}

	// Cooldown arming based on trigger mode.
	// The cooldown timer actually starts on the next UW event (dive), not here.
	// Cooldown guard: skip re-arming if a cooldown is already active — otherwise
	// a TX that fires during cooldown (only possible in DUTY_CYCLE / LEGACY /
	// PASS_PREDICTION; SURFACING_BURST is already gated) would re-set
	// m_cooldown_armed, and the next dive's set_cycle_complete(now) would
	// reset the cooldown timer — creeping it forward by one full interval on
	// each cycle. Parity with the AT_SURFACE / END_OF_DOPPLER branches.
	{
		unsigned int trigger = configuration_store->read_param<unsigned int>(ParamID::COOLDOWN_TRIGGER_MODE);
		bool cooldown_active = ServiceManager::is_in_cooldown(service_current_time());
		if (trigger == (unsigned int)BaseCooldownTrigger::AFTER_LAST_TX) {
			// Mode 3: arm on every TX complete (timer restarts each time)
			if ((m_last_tx_had_gps || m_is_surfacing_burst) && !cooldown_active) {
				m_cooldown_armed = true;
			}
		} else if (trigger == (unsigned int)BaseCooldownTrigger::AFTER_FIRST_GNSS) {
			// Mode 2: arm after first GNSS TX only
			if (m_last_tx_had_gps && !m_cooldown_armed && !cooldown_active) {
				m_cooldown_armed = true;
				DEBUG_INFO("ArgosTxService: cooldown armed (AFTER_FIRST_GNSS)");
			}
		}
		// Modes 0 (AT_SURFACE) and 1 (END_OF_DOPPLER) are handled elsewhere
	}

	// Record the completed TX in the rolling-window rate limiter (Plan 1
	// step 2). No-op if disabled. Placed AFTER cooldown arming so the rate
	// limiter records every TX irrespective of cooldown trigger mode.
	for (unsigned int i = 0; i < tx_copies; i++)
		RateLimiter::record_tx(t);

	// Track wall-clock uptime of this TX completion for the spacing guard
	// (2026-05). Uses monotonic uptime, not RTC, so it survives RTC rollback
	// (cold boot virtual epoch, GNSS sync jumps). The next service_next_-
	// schedule_in_ms() that would return "immediate" (Doppler #1, first GNSS
	// TX after fix, etc.) clamps to at least surfacing_burst_init_s seconds
	// after this point — protects TCXO stability + CLS rate-limit + battery.
	m_last_tx_uptime_ms = service_current_timer();

	m_sched.notify_tx_complete();
	service_complete();
}

/// @brief Device error event — increment backoff counter, complete service with error.
void ArgosTxService::react(KineisEventDeviceError const &) {
	// Called exactly once, here at the top, because it does two jobs: it reports
	// whether a TX of OURS was in flight, and it clears the flag + stops the
	// send. Every branch below reads the result rather than calling it again.
	const bool was_our_tx_in_flight = service_cancel();

	// Distinguish "device cooldown reject" from a real device error. The SmdSat
	// 30-min autofallback cooldown is itself the recovery path — counting it as
	// a device error would burn the 3-strike session budget within ~3 surface
	// events and suspend TX for the rest of the session, even though TX would
	// have worked again on its own after the cooldown expired. Reschedule past
	// the cooldown (no exponential backoff, no error increment).
	unsigned int cooldown_ms = m_kineis.cooldown_remaining_ms();
	if (cooldown_ms > 0) {
		DEBUG_WARN("ArgosTxService::react: TX rejected by device cooldown (%u min left) — not counted",
		           cooldown_ms / 60000);
#if VALIDATION_LOG_ENABLE
		DEBUG_INFO("[VAL-SAT] argos_react_skip_cooldown remaining_ms=%u (no error increment)", cooldown_ms);
#endif
		if (was_our_tx_in_flight) {
			unsigned int backoff_s = (cooldown_ms / 1000) + 1;  // +1 s margin
			// Same two consumers as the exponential backoff below. This branch
			// is SmdSat-only (KIM2 has no cooldown notion), and without the hold
			// it was the one place where a SURFACING_BURST on SMD behaved
			// differently from the same burst on KIM2: the module was refusing
			// for up to 30 min and the burst kept pinging at 5 s regardless.
			m_device_error_hold_until = service_current_time() + (std::time_t)backoff_s;
			m_sched.set_earliest_schedule(m_device_error_hold_until);
			service_complete();
		}
		return;
	}

	// Only a failure of OUR transmission counts. KineisDevice::notify broadcasts
	// to every subscriber, and ArgosTxService is not the only one: on the KIM2
	// build ArgosRxService is handed the same device object, so a refused
	// downlink, a rejected AT+DL or an error during reception all arrive here.
	// None of them is a transmit fault, and with no TX pending none of them even
	// produced a visible effect — they just ratcheted the counter until the
	// guard in service_initiate() started refusing to transmit. Three failed RX
	// windows could mute a healthy transmitter.
	//
	// LoRaTxService has had this guard since its own version of the problem
	// (repeated warm-up failures across dives climbing to the strike limit); it
	// is the same test, on purpose, so the three backends behave alike.
	//
	// The cost is that a genuine TX error arriving AFTER the safety timeout has
	// already run service_cancel() is dropped. That is the right trade: the
	// timeout has by then cancelled and rescheduled the TX anyway, so the late
	// error would only be double-counting a failure already handled.
	if (!was_our_tx_in_flight) {
		DEBUG_TRACE("ArgosTxService::react: KineisEventDeviceError with no TX of ours in flight — not counted");
		return;
	}

	m_consecutive_device_errors++;
	DEBUG_WARN("ArgosTxService::react: KineisEventDeviceError (consecutive=%u/%u)", m_consecutive_device_errors,
	           DEVICE_ERROR_MAX_CONSECUTIVE);
#if !defined(ARGOS_SMD) || (ARGOS_SMD != 1)
	// KIM2 only: surface the burst type + modulation on a TX failure.
	DEBUG_WARN("ArgosTxService: TX FAILED — type=%s mode=%d (consecutive=%u/%u, will backoff/retry)",
	           m_last_val_tx_type, static_cast<int>(m_scheduled_mode), m_consecutive_device_errors,
	           DEVICE_ERROR_MAX_CONSECUTIVE);
#endif

	// Restore TCXO warmup if it was skipped
	if (m_tcxo_skip_on_next_tx) {
		ArgosConfig argos_config;
		configuration_store->get_argos_configuration(argos_config);
		m_kineis.set_tcxo_warmup_time(argos_config.argos_tcxo_warmup_time);
		m_tcxo_skip_on_next_tx = false;
	}

	// On error, force RCONF reload on next power-on as safety measure.
	// The SMD may have been in an inconsistent state.
	{
		ArgosConfig ac;
		configuration_store->get_argos_configuration(ac);
		if (ac.adaptive_modulation && ac.mode == BaseArgosMode::SURFACING_BURST) {
			KineisModulation target = m_has_gnss_fix_since_surfacing ? KineisModulation::LDK : adaptive_doppler_modulation();
			m_modulation_preconfig = target;
			DEBUG_INFO("ArgosTxService::react: error recovery — caching RCONF for modulation %d", (int)target);
		}
	}

	{
		if (m_consecutive_device_errors >= DEVICE_ERROR_MAX_CONSECUTIVE) {
			// Max errors reached: stop transmitting, but on a deadline rather
			// than on the arrival of an outside event. Four things end it --
			// the probe below, a new GPS session (notify_peer_event GNSS
			// branch), a surface event (UW branch), or a reboot -- and only
			// the first exists on a land tracker that has GNSS turned off.
			// The 2026-06-30 field case, a KIM2 brown-out that cost 6.5 h of
			// silence, is what the GPS-session clear was written for; the
			// deadline is what covers the configurations it cannot reach.
			//
			// service_complete() WITH a reschedule, and an explicit
			// cancel_safety_timeout(), are both load-bearing. Completing
			// without a reschedule skips deschedule(), the only path that
			// cancels the safety-net timeout armed just before
			// service_initiate(); the timeout then outlived the suspension,
			// fired, rescheduled at zero delay, hit the guard in
			// service_initiate() and rearmed itself -- a wake/log/skip loop
			// once per service_next_timeout() that transmitted nothing and
			// cost an LFS commit per pass.
			m_device_error_hold_until = service_current_time() + (std::time_t)DEVICE_ERROR_PROBE_PERIOD_S;
			DEBUG_ERROR("ArgosTxService: %u consecutive device errors — suspending TX for %u s, then one probe "
			            "(a GPS session or a surface event clears it sooner)",
			            m_consecutive_device_errors, DEVICE_ERROR_PROBE_PERIOD_S);
			// Sleep on the deadline instead of completing without a reschedule:
			// see the note on DEVICE_ERROR_PROBE_PERIOD_S for what the latter
			// did to the safety-net timeout.
			cancel_safety_timeout();
			m_sched.set_earliest_schedule(m_device_error_hold_until);
			service_complete();
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
			// This is safe today only by an invariant that lives thirteen lines
			// above: the branch is unreachable unless the counter is below
			// DEVICE_ERROR_MAX_CONSECUTIVE (3), so the shift never exceeded 2.
			// Give this service a probe period the way LoRaTxService has one and
			// the counter climbs without bound -- and nothing here would say so.
			//
			// The loop produces the identical sequence (verified against the
			// shift for every n it can represent), so nothing is shortened; it
			// simply cannot wrap, and it needs no magic bound tied to the
			// current value of BASE_MS.
			unsigned int backoff_ms = Backoff::doubling_capped(
			    DEVICE_ERROR_BACKOFF_BASE_MS, DEVICE_ERROR_BACKOFF_MAX_MS, m_consecutive_device_errors);
			DEBUG_WARN("ArgosTxService: backoff %u ms before next TX attempt", backoff_ms);
			// Two consumers, one instant. set_earliest_schedule reaches the
			// periodic and prepass schedulers; m_device_error_hold_until reaches
			// the burst paths, which schedule themselves and never read that
			// floor. Setting only the first is what made this ladder a log line
			// in SURFACING_BURST.
			m_device_error_hold_until = service_current_time() + (std::time_t)(backoff_ms / 1000);
			m_sched.set_earliest_schedule(m_device_error_hold_until);
			service_complete();
		}
	}
}

// === BaseGnssStrategy::REUSE_LAST plumbing =================================
// Kept together and free of member state so they stay unit-testable in
// isolation. All of them are on the live scheduling path -- the note that used
// to say "currently unused at runtime" was written before the consumers landed
// and has been wrong since.

unsigned int ArgosTxService::compute_gps_log_age_seconds(const GPSLogEntry &entry, std::time_t now) {
	// LogHeader year=0 is the cold-boot / unset RTC sentinel — never trust it.
	if (entry.header.year == 0) return UINT_MAX;
	std::time_t entry_time = convert_epochtime(entry.header.year, entry.header.month, entry.header.day,
	                                           entry.header.hours, entry.header.minutes, entry.header.seconds);
	// Future-dated entries indicate either RTC roll-back or corruption — reject
	// rather than reporting age=0 which would falsely qualify as "fresh".
	if (now < entry_time) return UINT_MAX;
	return (unsigned int)(now - entry_time);
}

unsigned int ArgosTxService::apply_spacing_guard(unsigned int proposed_delay_ms, unsigned int min_spacing_s,
                                                 std::time_t now) {
	if (m_last_tx_uptime_ms == 0) return proposed_delay_ms;  // no prior TX, no guard needed
	uint64_t now_uptime_ms = service_current_timer();
	uint64_t earliest_ms = m_last_tx_uptime_ms + (uint64_t)min_spacing_s * 1000;
	uint64_t proposed_uptime_ms = now_uptime_ms + proposed_delay_ms;
	if (proposed_uptime_ms >= earliest_ms) return proposed_delay_ms;  // OK as-is
	unsigned int deferred_ms = (unsigned int)(earliest_ms - now_uptime_ms);
	DEBUG_INFO("ArgosTxService: TX deferred %u ms (intra-burst spacing guard, last TX %u ms ago)", deferred_ms,
	           (unsigned int)(now_uptime_ms - m_last_tx_uptime_ms));
	// Re-anchor scheduler at the deferred RTC time so the scheduler doesn't
	// fire "early" relative to our spacing.
	m_sched.schedule_at(now + (std::time_t)(deferred_ms / 1000) + 1);
	return deferred_ms;
}

/// @brief Clamp a burst schedule to a pending device-error backoff/suspension.
/// @param proposed_delay_ms  what the burst cadence asked for.
/// @param now                current RTC epoch second.
/// @return the delay the caller should return, in ms.
unsigned int ArgosTxService::apply_device_error_hold(unsigned int proposed_delay_ms, std::time_t now, bool &held) {
	held = false;
	if (m_device_error_hold_until == 0) return proposed_delay_ms;
	if (now >= m_device_error_hold_until) {
		// Elapsed. Drop it here rather than leaving a stale instant behind: the
		// periodic path self-clears inside the scheduler, the burst path has no
		// equivalent.
		m_device_error_hold_until = 0;
		return proposed_delay_ms;
	}
	unsigned int hold_ms = (unsigned int)(m_device_error_hold_until - now) * 1000u;
	if (hold_ms <= proposed_delay_ms) return proposed_delay_ms;  // burst is already slower
	DEBUG_INFO("ArgosTxService: burst TX held %u ms by the device-error backoff (cadence asked %u ms)", hold_ms,
	           proposed_delay_ms);
	m_sched.schedule_at(m_device_error_hold_until);
	held = true;
	return hold_ms;
}


const char *ArgosTxService::aop_status_text(AopStatus e) {
	switch (e) {
	case AopStatus::NO_RECORD: return "no AOP record (never provisioned)";
	case AopStatus::RTC_UNSET: return "RTC not set (no reliable time)";
	case AopStatus::NO_DATE: return "bulletin date missing or later than the current time";
	case AopStatus::EXPIRED: return "bulletin expired";
	case AopStatus::USABLE: return "usable";
	default: return "unknown status";
	}
}

bool ArgosTxService::aop_is_usable(const ArgosConfig &config, std::time_t now, unsigned int &age_s) {
	return aop_status(config, now, age_s) == AopStatus::USABLE;
}

/// @brief Are the AOP usable (present, dated, not expired)?
ArgosTxService::AopStatus ArgosTxService::aop_status(const ArgosConfig &config, std::time_t now, unsigned int &age_s) {
	age_s = 0;
	BasePassPredict &pp = configuration_store->read_pass_predict();
	if (pp.num_records == 0) return AopStatus::NO_RECORD;
	// last_aop_update = ARGOS_AOP_DATE, set when a PASPW is received.
	// Its factory value is old (Oct. 2021): a beacon that was never provisioned
	// is therefore considered expired as soon as the age limit is enabled, and
	// falls back to periodic — that is the safe behaviour, not an accident.
	// Without a reliable time no window can be computed — and above all the AOP
	// age is meaningless. A distinct cause because the remedy is one too: set
	// the RTC ($RTCW or a GNSS fix), not re-upload the AOP.
	if (!service_is_time_known()) return AopStatus::RTC_UNSET;
	if (config.last_aop_update <= 0 || now <= config.last_aop_update) return AopStatus::NO_DATE;
	std::time_t age = now - config.last_aop_update;
	age_s = static_cast<unsigned int>(age);
	if (config.aop_max_age_days == 0) return AopStatus::USABLE;  // 0 = no expiry
	static constexpr std::time_t SECS_PER_DAY = 24 * 60 * 60;
	return (age < static_cast<std::time_t>(config.aop_max_age_days) * SECS_PER_DAY) ? AopStatus::USABLE
	                                                                                : AopStatus::EXPIRED;
}

/// @brief Update the statuses readable through STATR (PPT01..PPT04).
void ArgosTxService::refresh_prepass_status(const ArgosConfig &config, std::time_t now, std::time_t next_pass_epoch) {
	unsigned int age_s = 0;
	bool valid = aop_is_usable(config, now, age_s);
	try {
		configuration_store->write_param(ParamID::SAT_AOP_VALID, valid);
		configuration_store->write_param(ParamID::SAT_AOP_AGE_S, age_s);
		if (next_pass_epoch > 0)
			configuration_store->write_param(ParamID::SAT_NEXT_PASS_TS, static_cast<unsigned int>(next_pass_epoch));
	} catch (...) {
		// A status is information, never a reason to prevent a TX.
		DEBUG_WARN("ArgosTxService::refresh_prepass_status: could not write the status parameters");
	}
}

bool ArgosTxService::should_promote_doppler_to_gnss(unsigned int max_age_s) {
	GPSLogEntry *latest = m_depth_pile_manager.peek_gps_latest_any();
	if (!latest) return false;
	// Only "positional" entries are useful — CLOUDLOCATE handled separately
	// in SURFACING_BURST path via process_gnss_burst itself.
	bool is_positional =
	    (latest->info.event_type == GPSEventType::FIX || latest->info.event_type == GPSEventType::UPDATE
	     || latest->info.event_type == GPSEventType::FASTLOC);
	if (!is_positional) return false;
	unsigned int age = compute_gps_log_age_seconds(*latest, service_current_time());
	if (age > max_age_s) return false;
	return true;
}

bool ArgosTxService::read_cached_last_fix(GPSLogEntry &out) {
	unsigned int max_age_s = configuration_store->read_param<unsigned int>(ParamID::GNSS_REUSE_FIX_MAX_AGE_S);
	if (max_age_s == 0) return false;  // reuse disabled
	GPSLogEntry *cached = m_depth_pile_manager.peek_gps_latest_any();
	if (!cached) return false;
	// REUSE_LAST semantically means "TX with the last KNOWN-GOOD position".
	// Only FIX/UPDATE entries qualify:
	//   - FASTLOC = degraded fix from a short surface where the device didn't
	//     have time for a full PVT solution. In HAULED context (the primary
	//     REUSE_LAST consumer) the user has time to wait for a real fix
	//     before going hauled — TX'ing a stale FASTLOC days/weeks later
	//     would mislead downstream tracking.
	//   - CLOUDLOCATE = raw measurements, not a position; format-specific
	//     packet builder (build_cloudlocate_packet) not wired here.
	//   - NO_FIX = no position at all, nothing to reuse.
	// Falls back to process_doppler_burst (no position) when this returns
	// false. The FastLoc-priority path used in non-REUSE_LAST modes (see
	// should_promote_doppler_to_gnss) covers the "use FastLoc opportunistically"
	// use case separately.
	if (cached->info.event_type != GPSEventType::FIX && cached->info.event_type != GPSEventType::UPDATE) return false;
	unsigned int age = compute_gps_log_age_seconds(*cached, service_current_time());
	if (age > max_age_s) return false;
	out = *cached;
	return true;
}
