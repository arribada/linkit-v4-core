/**
 * @file gps_service.cpp
 * @brief GNSS acquisition service — scheduling, fix processing, fastloc, CloudLocate.
 */

#include <cstdint>
#include <climits>
#include "gps_service.hpp"
#include "config_store.hpp"
#include "scheduler.hpp"
#include "rtc.hpp"
#include "pmu.hpp"
#include "rate_limiter.hpp"

extern RTC *rtc;
#if ENABLE_AXL_SENSOR
#include "axl_sensor_service.hpp"
#endif

extern ConfigurationStore *configuration_store;
extern Scheduler *system_scheduler;

static constexpr unsigned int MS_PER_SEC = 1000;
static constexpr unsigned int FIRST_AQPERIOD_SEC = 30;  ///< Accelerated first fix schedule
static constexpr unsigned int SERVICE_SAFETY_MARGIN_S =
    30;  ///< Extra margin on top of acquisition_timeout for Service watchdog
static constexpr unsigned int DEEP_IDLE_HARD_CAP_S =
    24 * 3600;  ///< R5: ceiling on any single deep-idle window (see try_enter_deep_idle_or_poweroff)

/// @brief Copy GNSSData fields into GPSLogEntry (avoids 30-line duplication).
static void copy_gnss_to_log(const GNSSData &src, GPSLogEntry &dst) {
	dst.info.iTOW = src.iTOW;
	dst.info.year = src.year;
	dst.info.month = src.month;
	dst.info.day = src.day;
	dst.info.hour = src.hour;
	dst.info.min = src.min;
	dst.info.sec = src.sec;
	dst.info.valid = src.valid;
	dst.info.fixType = src.fixType;
	dst.info.flags = src.flags;
	dst.info.flags2 = src.flags2;
	dst.info.flags3 = src.flags3;
	dst.info.numSV = src.numSV;
	dst.info.lon = src.lon;
	dst.info.lat = src.lat;
	dst.info.height = src.height;
	dst.info.hMSL = src.hMSL;
	dst.info.hAcc = src.hAcc;
	dst.info.vAcc = src.vAcc;
	dst.info.velN = src.velN;
	dst.info.velE = src.velE;
	dst.info.velD = src.velD;
	dst.info.gSpeed = src.gSpeed;
	dst.info.headMot = src.headMot;
	dst.info.sAcc = src.sAcc;
	dst.info.headAcc = src.headAcc;
	dst.info.pDOP = src.pDOP;
	dst.info.vDOP = src.vDOP;
	dst.info.hDOP = src.hDOP;
	dst.info.headVeh = src.headVeh;
	dst.info.ttff = src.ttff;
}


/// @brief Init: reset fix state, warn about fastloc mode.
void GPSService::service_init() {
	m_is_active = false;
	m_is_first_fix_found = false;
	m_is_first_schedule = true;
	m_num_gps_fixes = 0;
	m_backup_active = false;
	m_pending_backup_duration_s = 0;
	m_underwater = false;
	m_backup_exit_task = {};
	m_backup_retry_task = {};
	// Safety-net 3.2: reset stuck-recovery state on init (idempotent across
	// service restart cycles — Service framework calls init/term in pairs).
	m_consecutive_dead_sessions = 0;
	m_wdt_cold_start_tried = false;
	m_stuck_recovery_in_flight = false;
	m_stuck_recovery_arm_task = {};
	m_stuck_recovery_done_task = {};

	// Safety-nets 3.5 + 3.6: arm health WDT (24h, any GPS event) and
	// no-PVT WDT (7d, real PVT only). Both are re-armed by the relevant
	// callbacks. Each fires PMU::reset(false) — defensive against the
	// "tag alive but inert" scenarios that hardware WDT cannot catch
	// (catch-loop kicks the WDT, scheduler still runs but service is dead).
	m_health_wdt_task = {};
	m_no_pvt_wdt_task = {};
	arm_health_wdt();
	arm_no_pvt_wdt();
	// 2026-05 deep-idle refactor: no more periodic backup-charge scheduler.
	// Recharge happens implicitly via the deep-idle-after-off window now.

	// Warn user about fastloc mode impact on sensor messages
	unsigned int fastloc_mode = configuration_store->read_param<unsigned int>(ParamID::GNSS_FASTLOC_MODE);
	if (fastloc_mode == (unsigned int)BaseFastlocMode::DEGRADED_PVT) {
		DEBUG_WARN("GPSService: FASTLOC_MODE=DEGRADED_PVT — sensor messages disabled during GPS fallback");
	} else if (fastloc_mode == (unsigned int)BaseFastlocMode::CLOUDLOCATE) {
		unsigned int cl_format = configuration_store->read_param<unsigned int>(ParamID::GNSS_CLOUDLOCATE_FORMAT);
		if (cl_format == (unsigned int)BaseCloudLocateFormat::MEASC12) {
			DEBUG_INFO("GPSService: FASTLOC_MODE=CLOUDLOCATE format=MEASC12 — sensor messages included in fallback");
		} else {
			DEBUG_WARN("GPSService: FASTLOC_MODE=CLOUDLOCATE format=%u — sensor messages disabled during GPS fallback",
			           cl_format);
		}
	}
}

/// @brief End-of-session dispatch (2026-05 deep-idle refactor). Reads
/// GNSS_DEEP_IDLE_AFTER_OFF_S and either cuts the rail immediately (param=0,
/// current default behavior — no regression for unconfigured devices), enters
/// deep-idle indefinitely (param=0xFFFFFFFF — rail always on, M10Q sleeping),
/// or enters deep-idle for the specified duration with an auto-poweroff timer.
///
/// Called by every "GPS session done" path (NO_FIX timeout, PVT received,
/// MaxNavSamples / MaxSatSamples / Error, service_cancel). Replaces the
/// scattered `m_device.power_off()` calls. service_term() bypasses this and
/// calls power_off() directly per R2 robustness — never deep-idle on teardown.
///
/// R4 inhibit: if the boot was a WDT reset, the first session post-boot
/// MUST cold-power-off (no deep-idle) to prove the cold path works before
/// re-engaging the optimization. Cleared after first valid fix.
#ifndef VALIDATION_LOG_ENABLE
#define VALIDATION_LOG_ENABLE 0
#endif
#if VALIDATION_LOG_ENABLE
#define VAL_GNSS(fmt, ...) DEBUG_INFO("[VAL-GNSS] " fmt, ##__VA_ARGS__)
#else
#define VAL_GNSS(fmt, ...) \
	do {                   \
	} while (0)
#endif

/// @brief R4 inhibit setter — stamp the arm time so the 24h hard-cap in
/// try_enter_deep_idle_or_poweroff() can force-clear if no PVT/CloudLocate.
void GPSService::set_deep_idle_inhibit_first_session(bool inhibit) {
	m_deep_idle_inhibit_first_session = inhibit;
	m_inhibit_set_at_ms = inhibit ? PMU::get_timestamp_ms() : 0;
}

/// @brief Is the beacon demonstrably still doing its job?
/// See the declaration for why this gates both GNSS watchdogs.
bool GPSService::beacon_is_transmitting(unsigned int within_s) const {
	if (!rtc || !rtc->is_set()) return false;
	std::time_t last_tx = 0;
	try {
		// std::time_t, not unsigned int: that is the alternative LAST_TX holds
		// (config_store.hpp declares it static_cast<std::time_t>(0U), and both TX
		// services write a std::time_t). read_param is std::get<T> behind a
		// catch-all that rethrows CONFIG_STORE_CORRUPTED, so asking for the wrong
		// alternative throws every time -- and these watchdog lambdas are posted
		// straight to the scheduler, outside Service::run_scheduled_task's
		// barrier, so it would reach the main loop and land in ErrorState. Spell
		// it std::time_t and not long long: it is long long on the target and
		// long on the host build.
		last_tx = configuration_store->read_param<std::time_t>(ParamID::LAST_TX);
	} catch (...) {
		// The evidence is unreadable. Report "working" rather than "silent":
		// resetting a beacon because a parameter read failed is exactly the
		// failure this gate exists to prevent, and an unreadable config store is
		// not something a GNSS watchdog should be adjudicating.
		DEBUG_ERROR("GPSService: LAST_TX unreadable — assuming the beacon works, no reset");
		return true;
	}
	if (last_tx <= 0) return false;
	std::time_t now = rtc->gettime();
	if (now < last_tx) return false;  // clock moved backwards; prove nothing
	return (unsigned int)(now - last_tx) <= within_s;
}

/// @brief Safety net 3.5 — re-arm the GPS-event health watchdog.
/// Fires after HEALTH_WDT_HOURS without ANY GPS event (PVT, degraded,
/// CloudLocate, NO_FIX). Soft reset to recover the tag.
void GPSService::arm_health_wdt() {
	system_scheduler->cancel_task(m_health_wdt_task);
	m_health_wdt_task = system_scheduler->post_task_prio(
	    [this]() {
		    // Everything below runs inside a try. This task is posted straight to
		    // the scheduler, not through Service::run_scheduled_task, so it has no
		    // exception barrier of its own: any throw in here reaches the main
		    // loop and becomes an ErrorState reset, attributed to a corrupt config
		    // store rather than to GNSS. A safety net must not be able to do that.
		    try {
			    constexpr unsigned int WINDOW_S = HEALTH_WDT_HOURS * 3600u;

			    // A tag with GNSS deliberately off produces no GPS event, ever. That
			    // is a configuration, not a fault, and resetting it every 24 h for
			    // the length of the deployment would be this net breaking a working
			    // beacon. Log at INFO -- an operator reading a recovered SYS log must
			    // not find an ERROR here and go looking for a receiver problem.
			    GNSSConfig gnss_config;
			    configuration_store->get_gnss_configuration(gnss_config);
			    if (!gnss_config.enable) {
				    DEBUG_INFO("GPS Health WDT: no GPS event in %u h, and GNSS_EN=0 — expected, not a fault. No reset; "
					           "re-arming. Clears by itself if GNSS is re-enabled.",
					           HEALTH_WDT_HOURS);
				    arm_health_wdt();
				    return;
			    }

			    // The receiver is silent but the beacon is transmitting: whatever is
			    // wrong is confined to the GNSS side, and a device reset is the wrong
			    // tool for it -- it costs the RAM state, a flash write and a forced
			    // cold GNSS path, to fix something a cold start addresses directly.
			    // Escalate on the GNSS only, and let the net re-arm: if the beacon
			    // stops transmitting too, the next firing takes the reset branch.
			    if (beacon_is_transmitting(WINDOW_S)) {
				    // ONE cold start, not one per firing. A cold start wipes the
				    // receiver's backup RAM -- ephemeris, almanac, position, time --
				    // and repeating that every 24 h is the documented cause of fixes
				    // dying after a couple of days on this platform: it destroys the
				    // AssistNow-Autonomous state the receiver needs to recover. Try
				    // it once, then wait for evidence it helped. The flag clears on
				    // any GPS event, alongside the dead-session counter.
				    if (!m_wdt_cold_start_tried) {
					    m_wdt_cold_start_tried = true;
					    m_force_cold_start = true;
					    DEBUG_WARN("GPS Health WDT: no GPS event in %u h, but the beacon transmitted within that "
						           "window — it is working. No reset; forcing ONE cold start on the next "
						           "acquisition and re-arming.",
						           HEALTH_WDT_HOURS);
				    } else {
					    DEBUG_WARN("GPS Health WDT: no GPS event in %u h, beacon still transmitting, and a cold "
						           "start has already been tried since the last GPS event — no reset and no "
						           "further action; re-arming. Clears on any GPS event.",
						           HEALTH_WDT_HOURS);
				    }
				    arm_health_wdt();
				    return;
			    }

			    DEBUG_ERROR("GPS Health WDT: no GPS event and no transmission in %u h — nothing is working, soft reset",
				            HEALTH_WDT_HOURS);
			    // 2026-06: snapshot the (free-running, crystal-accurate) RTC to flash
			    // right before the soft reset so the post-reset restore from
			    // LAST_KNOWN_RTC does NOT jump time backward by up to the 30-min
			    // periodic-flush gap. A backward RTC makes injected MGA-INI-TIME wrong
			    // and breaks AssistNow — a reset loop would march time backward and
			    // never recover. Persisting here bounds the loss to ~0.
			    if (rtc && rtc->is_set() && rtc->gettime() > 0)
				    configuration_store->write_param(ParamID::LAST_KNOWN_RTC,
					                                 static_cast<unsigned int>(rtc->gettime()));
			    PMU::save_stack(PMULogType::WDT);
			    PMU::reset(false);
		    } catch (...) {
			    DEBUG_ERROR("GPS Health WDT: exception while deciding — no reset, re-arming");
			    arm_health_wdt();
		    }
	    },
	    "GPSHealthWDT", Scheduler::DEFAULT_PRIORITY, (uint64_t)HEALTH_WDT_HOURS * 3600ULL * (uint64_t)MS_PER_SEC);
}

/// @brief Safety net 3.6 — re-arm the no-real-PVT watchdog.
/// Fires after NO_PVT_WDT_DAYS without a real PVT (CloudLocate / degraded
/// don't count). Soft reset to recover from filter-trapped CloudLocate-only
/// mode where m_is_first_fix_found stays false indefinitely.
void GPSService::arm_no_pvt_wdt() {
	system_scheduler->cancel_task(m_no_pvt_wdt_task);
	m_no_pvt_wdt_task = system_scheduler->post_task_prio(
	    [this]() {
		    // Same barrier as the health watchdog above, for the same reason.
		    try {
			    constexpr unsigned int WINDOW_S = NO_PVT_WDT_DAYS * 24u * 3600u;

			    // A tag with GNSS off produces no real PVT by design, exactly as it
			    // produces no GPS event. The health watchdog got this gate and this
			    // one did not, which was an oversight rather than a decision.
			    // get_gnss_configuration is the right call: it already folds in the
			    // low-battery override, HAULED_GNSS_STRAT=OFF, the MOORED narrowing
			    // and certification forcing GNSS off.
			    GNSSConfig gnss_config;
			    configuration_store->get_gnss_configuration(gnss_config);
			    if (!gnss_config.enable) {
				    DEBUG_INFO("GPS No-PVT WDT: no real PVT in %u days, and GNSS is disabled — expected, not a "
					           "fault. No reset; re-arming.",
					           NO_PVT_WDT_DAYS);
				    arm_no_pvt_wdt();
				    return;
			    }

			    // A CloudLocate deployment never produces a real PVT: the raw
			    // measurement IS the product, resolved cloud-side.
			    //
			    // Test the MODE, not GNSS_CLOUDLOCATE_ONLY. That parameter is a
			    // session-termination optimisation -- stop at the first raw
			    // measurement instead of waiting for the PVT -- while what declares
			    // a CloudLocate deployment is GNSS_FASTLOC_MODE, which is what every
			    // other site in this file tests. Gating on the optimisation alone
			    // left an ordinary CloudLocate tag, one that simply waits for the
			    // PVT before falling back, unexempted and on its way to a reset.
			    const bool cloudlocate_mode = configuration_store->read_param<unsigned int>(ParamID::GNSS_FASTLOC_MODE)
			                                  == (unsigned int)BaseFastlocMode::CLOUDLOCATE;
			    const bool cloudlocate_only = configuration_store->read_param<bool>(ParamID::GNSS_CLOUDLOCATE_ONLY);
			    if (cloudlocate_mode || cloudlocate_only) {
				    DEBUG_INFO("GPS No-PVT WDT: no real PVT in %u days, but this is a CloudLocate deployment "
					           "(FASTLOC_MODE=CLOUDLOCATE:%u CLOUDLOCATE_ONLY:%u) — expected, not a fault. No reset; "
					           "re-arming.",
					           NO_PVT_WDT_DAYS, (unsigned int)cloudlocate_mode, (unsigned int)cloudlocate_only);
				    arm_no_pvt_wdt();
				    return;
			    }

			    // Same rule as the health watchdog: do not reset a beacon that is
			    // demonstrably useful. A missing PVT with transmissions still going
			    // out is a GNSS-side problem, and a reset does not fix a filter that
			    // is rejecting every fix -- a cold start might.
			    if (beacon_is_transmitting(WINDOW_S)) {
				    // Bounded the same way as the health watchdog, for the same
				    // reason: repeated backup-RAM wipes make the next fix harder.
				    if (!m_wdt_cold_start_tried) {
					    m_wdt_cold_start_tried = true;
					    m_force_cold_start = true;
					    DEBUG_WARN("GPS No-PVT WDT: no real PVT in %u days, but the beacon is still transmitting. "
						           "No reset; forcing ONE cold start on the next acquisition and re-arming.",
						           NO_PVT_WDT_DAYS);
				    } else {
					    DEBUG_WARN("GPS No-PVT WDT: no real PVT in %u days, beacon still transmitting, and a cold "
						           "start has already been tried since the last GPS event — no reset and no "
						           "further action; re-arming.",
						           NO_PVT_WDT_DAYS);
				    }
				    arm_no_pvt_wdt();
				    return;
			    }

			    DEBUG_ERROR("GPS No-PVT WDT: no real PVT and no transmission in %u days — soft reset", NO_PVT_WDT_DAYS);
			    // 2026-06: persist the accurate RTC before reset (see Health WDT) so the
			    // restore doesn't rewind time and poison AssistNow after the reset.
			    if (rtc && rtc->is_set() && rtc->gettime() > 0)
				    configuration_store->write_param(ParamID::LAST_KNOWN_RTC,
					                                 static_cast<unsigned int>(rtc->gettime()));
			    PMU::save_stack(PMULogType::WDT);
			    PMU::reset(false);
		    } catch (...) {
			    DEBUG_ERROR("GPS No-PVT WDT: exception while deciding — no reset, re-arming");
			    arm_no_pvt_wdt();
		    }
	    },
	    "GPSNoPvtWDT", Scheduler::DEFAULT_PRIORITY, (uint64_t)NO_PVT_WDT_DAYS * 24ULL * 3600ULL * (uint64_t)MS_PER_SEC);
}

void GPSService::try_enter_deep_idle_or_poweroff() {
	if (m_deep_idle_inhibit_first_session) {
		// 24 h hard-cap (safety net 3.1): if the inhibit was armed by a WDT
		// reset but no PVT and no CloudLocate ever fires (M10Q hardware
		// degraded, antenna issue, BBR loss), the inhibit would persist for
		// the rest of the deployment and deep-idle would be disabled forever
		// — meaning V_BCKP drains and the M10Q can never warm-start again.
		// Force-clearing the inhibit after 24 h lets deep-idle re-engage so
		// V_BCKP gets recharged via the normal window. Worst case is identical
		// to current behaviour (inhibit would still be active); best case is
		// automatic recovery.
		constexpr uint64_t INHIBIT_HARD_CAP_MS = 24ULL * 3600ULL * 1000ULL;
		uint64_t now_ms = PMU::get_timestamp_ms();
		// Defensive: if the setter was bypassed (legacy code path) the stamp
		// may still be zero. Initialise on first observation.
		if (m_inhibit_set_at_ms == 0) m_inhibit_set_at_ms = now_ms;
		if (now_ms - m_inhibit_set_at_ms > INHIBIT_HARD_CAP_MS) {
			DEBUG_WARN("GPSService: WDT inhibit > 24 h without proof — force-clearing");
			VAL_GNSS("dispatch=wdt_inhibit_force_clear");
			m_deep_idle_inhibit_first_session = false;
			m_inhibit_set_at_ms = 0;
			// Fall through to normal dispatch below — DO NOT return
		} else {
			DEBUG_INFO("GPSService::try_enter_deep_idle_or_poweroff: WDT-reset inhibit — cold power_off");
			VAL_GNSS("dispatch=wdt_inhibit_cold_off");
			m_last_dispatch_was_deep_idle = false;
			m_device.power_off();
			// HIGH GNSS-AUDIT #2 fix: do NOT clear the inhibit here. This path
			// fires on every end-of-session including pure timeouts (MaxNavSamples,
			// MaxSatSamples, Error, service_cancel) where the M10Q may never have
			// really responded — clearing the inhibit on a 0-satellite timeout
			// would re-engage deep-idle on the next session without ever proving
			// the cold path works (R4 robustness invariant violated).
			//
			// The inhibit is now cleared ONLY at the two sites that prove the
			// M10Q is actually alive and the cold path succeeded:
			//   - `react(GPSEventPVT)` — a valid PVT means cold power-on + UART +
			//     nav engine all working.
			//   - `react(GPSEventCloudLocateReady)` — raw measurement means UART +
			//     nav up, even though no PVT.
			// Both already clear the flag inline (see lines ~760 and ~840).
			VAL_GNSS("dispatch=wdt_inhibit_pending_proof (no PVT yet)");
			return;
		}
	}

	unsigned int deep_idle_s = configuration_store->read_param<unsigned int>(ParamID::GNSS_DEEP_IDLE_AFTER_OFF_S);

	// Always cancel any previously-armed auto-off timer. We're starting a fresh
	// disposition: either we re-arm (deep idle, any duration) or we don't need
	// it (immediate poweroff).
	system_scheduler->cancel_task(m_deep_idle_auto_off_task);

	if (deep_idle_s == 0) {
		DEBUG_TRACE("GPSService::try_enter_deep_idle_or_poweroff: disabled — power_off");
		VAL_GNSS("dispatch=immediate_off GNP52=0");
		m_last_dispatch_was_deep_idle = false;
		m_device.power_off();
		m_deep_idle_started_at_ms = 0;
		return;
	}

	// CRITICAL #1 audit fix: instead of calling `enter_deep_idle()` directly
	// (which always refused because state != idle at end-of-session), arm the
	// device-side intent flag THEN call power_off(). The power-down chain
	// runs through stopreceive → fetchdatabase → poweroff, and at the final
	// step `state_poweroff()` reroutes to `enterbackup` (keeping VDD on)
	// because the intent flag is set. Net result: M10Q ends up in
	// PMREQ-backup with V_BCKP charging, exactly the deep-idle goal.
	m_device.request_deep_idle_on_next_stop();
	m_device.power_off();
	m_last_dispatch_was_deep_idle = true;
	VAL_GNSS("dispatch=deep_idle_armed_via_poweroff_chain GNP52=%u", deep_idle_s);

	m_deep_idle_started_at_ms = PMU::get_timestamp_ms();  // R5 timestamp

	// The sentinel (0xFFFFFFFF = "never power the rail off") now arms this task
	// too, because the scheduling gate in service_next_schedule() holds for as
	// long as deep-idle is engaged. That hold is bounded at the 24 h hard cap,
	// so the framework does come back on its own; this task remains the prompt
	// exit, at the end of the actual GNP52 window rather than a day later. With no
	// auto-off task, nothing was left to re-enter that function, so on any
	// configuration without a peer-event source (UNDERWATER_EN=0, i.e. a plain
	// periodic tracker) GNSS acquisition stopped permanently — the old code
	// assumed a surfacing event would always come back. The R5 hard cap lives
	// inside that same function, so it could not rescue it either.
	//
	// But the sentinel task must NOT cut the rail (see the lambda): the M10Q
	// holds its ephemeris in PMREQ-backup powered off VDD, and on a board with
	// no V_BCKP coin cell that rail IS the only thing preserving a warm start.
	// Cutting it would throw away precisely what the sentinel exists to keep.
	m_deep_idle_is_sentinel = (deep_idle_s == 0xFFFFFFFFU);

	// Clamp keeps `* MS_PER_SEC` below inside unsigned int: GNP52 accepts up to
	// 0xFFFFFFFF and anything above ~4.29e6 s would overflow the multiply.
	unsigned int auto_off_s = (deep_idle_s > DEEP_IDLE_HARD_CAP_S) ? DEEP_IDLE_HARD_CAP_S : deep_idle_s;

	if (m_deep_idle_is_sentinel) {
		DEBUG_INFO("GPSService::try_enter_deep_idle_or_poweroff: never-poweroff (rail stays on, M10Q in PMREQ-backup, "
		           "scheduler re-opens in %u s)",
		           auto_off_s);
		VAL_GNSS("dispatch=never_off GNP52=sentinel gate_reopen_s=%u", auto_off_s);
	} else {
		DEBUG_INFO("GPSService::try_enter_deep_idle_or_poweroff: deep-idle for %u s (rail on, M10Q in PMREQ-backup, "
		           "auto-off timer armed)",
		           auto_off_s);
		VAL_GNSS("dispatch=deep_idle_engaged duration_s=%u", auto_off_s);
	}
	m_deep_idle_auto_off_task = system_scheduler->post_task_prio(
	    [this]() {
		    if (m_deep_idle_is_sentinel) {
			    // Rail stays up — the operator asked for exactly that, and on a
			    // board without a V_BCKP coin cell it is the only thing holding the
			    // ephemeris. Clearing the timestamp is enough to re-open the
			    // scheduling gate (it tests `started_at_ms > 0`); the next
			    // acquisition then wakes the M10Q in place with an EXTINT pulse
			    // rather than a rail cycle, keeping the warm start
			    // (m10qasync.cpp, power_on() backupidle branch). An M10Q that has
			    // genuinely wedged is already caught by WAKE_FAIL_FAST_FALLBACK.
			    DEBUG_INFO("GPSService: sentinel deep-idle — re-opening scheduler, rail left on");
			    VAL_GNSS("sentinel_gate_reopened");
			    m_deep_idle_started_at_ms = 0;
		    } else {
			    DEBUG_INFO("GPSService: deep-idle auto-poweroff timer fired");
			    VAL_GNSS("auto_off_timer_fired");
			    // HIGH GNSS-AUDIT #2 follow-up: `power_off()` is a no-op when users==0
			    // (the deep-idle dispatch already decremented users). Use the dedicated
			    // `poweroff_from_deep_idle()` which unconditionally tears down the rail
			    // from the backupidle state. Without this, GNP52 duration was
			    // effectively infinite — rail stayed on forever after the dispatch.
			    m_device.poweroff_from_deep_idle();
			    m_deep_idle_started_at_ms = 0;
		    }

		    // 2026-05-25 deep-idle scheduling gate exit: the gate in
		    // service_next_schedule holds for the whole GNP52 window, so no
		    // acquisition is armed while it runs. Now that the window
		    // is over (m_deep_idle_started_at_ms cleared), explicitly call
		    // service_reschedule so the framework re-arms the next acquisition
		    // through the normal path. Without this, the service would idle until
		    // the next peer event (dive/surface) — fine for most flows but a
		    // regression for time-based scheduling on a stationary device.
		    service_reschedule(false);
	    },
	    "GPSDeepIdleAutoOff", Scheduler::DEFAULT_PRIORITY, auto_off_s * MS_PER_SEC);
}

/// @brief Terminate GNSS service. UNCONDITIONAL power-off (R2 robustness from
/// deep-idle refactor): never trust that prior state left the rail clean. Cuts
/// rail even if state == idle/poweroff — safe no-op in those cases.
void GPSService::service_term() {
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
	system_scheduler->cancel_task(m_defer_gnss_timeout_task);
#endif
	system_scheduler->cancel_task(m_initiate_retry_task);
	system_scheduler->cancel_task(m_backup_exit_task);
	system_scheduler->cancel_task(m_backup_retry_task);
	system_scheduler->cancel_task(m_deep_idle_auto_off_task);
	// Safety-net 3.2: cancel any in-flight stuck-recovery tasks. If the service
	// is being torn down (OffState, ConfigurationState, OTA, BatteryCritical),
	// a pending power_off_immediate or service_reschedule could conflict with
	// the teardown chain. Idempotent — cancel on an invalid handle is a no-op.
	system_scheduler->cancel_task(m_stuck_recovery_arm_task);
	system_scheduler->cancel_task(m_stuck_recovery_done_task);
	m_stuck_recovery_in_flight = false;
	m_consecutive_dead_sessions = 0;
	m_wdt_cold_start_tried = false;
	// Safety-nets 3.5 + 3.6: cancel the health/no-PVT watchdogs on teardown.
	// Service framework re-creates the service from scratch on next init, so
	// dangling reset timers would fire after the rebuild — wrong behavior.
	system_scheduler->cancel_task(m_health_wdt_task);
	system_scheduler->cancel_task(m_no_pvt_wdt_task);
	m_pending_backup_duration_s = 0;
	if (m_backup_active) {
		m_device.exit_backup_charge_mode();
		m_backup_active = false;
	}
	// 2026-05-25 Fix #10: unconditional hard rail-cut via `power_off_immediate`.
	//
	// Previous behaviour (poweroff_from_deep_idle / power_off branch) is a
	// graceful cascade — fine when the M10Q is in a stable state, but fatal
	// when called mid-boot (state=poweron/configure, UART transactions
	// pending). A 2026-05-25 field log captured a 15-min hang because
	// service_term fired during an M10Q cold-boot triggered by an Argos TX
	// gate release just before a reed power-off gesture. ServiceManager::
	// stopall waited synchronously on a service that never returned;
	// OffState never entered; device stayed in System ON (no reed wake)
	// until WDT reset.
	//
	// power_off_immediate() bypasses the cascade and rail-cuts directly.
	// V_BCKP / BBR is lost — acknowledged trade-off (user spec: "BBR
	// doesn't matter at power-down, we want to cut everything"). Next
	// boot starts from a true power-on reset and the constructor path
	// re-initialises all internal state cleanly. is_in_deep_idle() check
	// is no longer needed because power_off_immediate is state-agnostic
	// (it cancels FSM tasks and cuts rail regardless of current state).
	m_device.power_off_immediate();
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
	m_defer_gnss_until_argos_first_tx = false;
#endif
	// GNSS LOW #6 audit fix: reset deep-idle bookkeeping for defensive hygiene.
	// Stale m_deep_idle_started_at_ms could lie about deep-idle duration after
	// a ConfigurationState round-trip. Currently latent (R5 check is gated
	// by is_in_deep_idle() which returns false post-teardown) but future
	// code reading the timestamp without that guard would see invalid state.
	m_deep_idle_started_at_ms = 0;
}

/// @brief Enabled if GNSS_EN is set in config (respects LB/zone overrides).
/// @return true if GNSS acquisition is enabled.
bool GPSService::service_is_enabled() {
	GNSSConfig gnss_config;
	configuration_store->get_gnss_configuration(gnss_config);
	bool enabled = gnss_config.enable;

	// R5 robustness side-effect (2026-05-23 audit finding 3): if the operator
	// disables GNSS_EN while deep-idle is engaged (rail on, M10Q in PMREQ-
	// backup), the Service framework stops calling service_next_schedule_in_ms
	// — the natural place where the R5 24 h hygiene cap fires. Without this
	// hook, the rail could stay on forever on a disabled device, slowly
	// draining the battery via the M10Q backup current.
	//
	// We do the cleanup here because service_is_enabled() is called frequently
	// by the framework. Yes, this violates the "pure query" idiom — but a
	// stuck rail on a sealed turtle is a worse outcome than a side-effect
	// in a frequently-called check.
	if (!enabled && m_device.is_in_deep_idle()) {
		DEBUG_INFO("GPSService::service_is_enabled: GNSS disabled while in deep-idle — cutting rail");
		m_device.power_off();
		m_deep_idle_started_at_ms = 0;
		system_scheduler->cancel_task(m_deep_idle_auto_off_task);
	}

	return enabled;
}

/// @brief Legacy entry point, superseded by service_next_schedule().
/// Still required while the base declares it pure -- deliberately, so that a new
/// service implementing neither cannot compile and be silently off for ever.
unsigned int GPSService::service_next_schedule_in_ms() {
	return SCHEDULE_DISABLED;
}

/// @brief Decide what GNSS does next: acquire, wait on a named gate, or stay off.
/// @return A Run aligned to the UTC 00:00 grid, a bounded hold naming what holds
/// us, or Off when the configuration itself asks for no periodic acquisition.
ScheduleDecision GPSService::service_next_schedule() {
	GNSSConfig gnss_config;
	configuration_store->get_gnss_configuration(gnss_config);

	// R5 robustness (2026-05 deep-idle refactor): if the M10Q has been in
	// deep-idle longer than a 24 h hard cap, force a rail-cycle. Defensive
	// against stale-state scenarios where the M10Q drifted into an
	// unresponsive mode (V_BCKP exhausted + BBR lost, soft hang). The
	// datasheet says PMREQ-backup is indefinitely stable, but for a 1-year
	// sealed turtle a daily prophylactic cycle is cheap insurance. The
	// cycle happens transparently — `power_off()` here, the next iteration
	// of this function will see state == poweroff and proceed normally,
	// then `power_on()` will cold-boot at the upcoming acquisition.
	// !m_deep_idle_is_sentinel: the auto-off lambda (see below) already carries
	// this guard. Without it here, the 24 h prophylactic cycle cuts the rail on a
	// "never cut" configuration -- and on a board with no V_BCKP cell that rail IS
	// the only thing holding the ephemeris.
	if (m_device.is_in_deep_idle() && m_deep_idle_started_at_ms > 0 && !m_deep_idle_is_sentinel) {
		uint64_t now_ms = PMU::get_timestamp_ms();
		constexpr uint64_t DEEP_IDLE_HARD_CAP_MS = static_cast<uint64_t>(DEEP_IDLE_HARD_CAP_S) * MS_PER_SEC;
		if (now_ms - m_deep_idle_started_at_ms > DEEP_IDLE_HARD_CAP_MS) {
			DEBUG_WARN("GPSService: deep-idle exceeded 24 h hard-cap (%llu ms) — forcing M10Q rail-cycle",
			           (unsigned long long)(now_ms - m_deep_idle_started_at_ms));
			m_device.power_off();
			m_deep_idle_started_at_ms = 0;
			system_scheduler->cancel_task(m_deep_idle_auto_off_task);
			// Fall through to the normal scheduling path.
		}
	}

	// 2026-05-25 deep-idle scheduling gate. Without this, Service::reschedule's
	// UTC-aligned next_schedule (cold_start_retry_period = 10 s) fires INSIDE
	// the GNP52 window, service_initiate cancels the auto-off timer at
	// gps_service.cpp:412 and calls m_device.power_on() which pulses EXTINT
	// and wakes the M10Q from PMREQ-backup — defeating deep-idle entirely.
	//
	// Observed 2026-05-25 on bench: GNP52=300s → 3 s of true backup (~10 µA)
	// then ~2 mA for the remaining 297 s, repeating every 10 s as the
	// scheduler kept retrying. The hard-cap branch above already cleared
	// m_deep_idle_started_at_ms in pathological cases, so checking both flags
	// (device state + non-zero timestamp) means a hard-cap-triggered fall-
	// through reaches the normal scheduling path naturally.
	//
	// Exit paths from this gate:
	//   - Any GNP52 > 0: m_deep_idle_auto_off_task fires at the end of the
	//     window (clamped to DEEP_IDLE_HARD_CAP_S) and triggers a
	//     service_reschedule — see the lambda in
	//     try_enter_deep_idle_or_poweroff — so normal scheduling resumes.
	//     This includes the sentinel: it used to arm no timer at all, which
	//     left this gate with no exit other than a peer event. Fine for a
	//     surfacing tracker, fatal for anything running UNDERWATER_EN=0,
	//     where no peer event ever comes and GNSS stopped for good.
	//   - Peer events (surface, AXL) still pre-empt the window, via
	//     reschedule(immediate=true) which BYPASSES this gate entirely
	//     (Service::reschedule skips service_next_schedule_in_ms when
	//     immediate is set).
	//   - Underwater / cooldown: handled by separate gates further down.
	if (m_device.is_in_deep_idle() && m_deep_idle_started_at_ms > 0) {
		// Bounded by the hard cap itself: if the auto-off task were ever lost,
		// the re-evaluation brings us back to the branch above, which cuts the
		// rail. That branch used to be unreachable without an outside caller.
		return ScheduleDecision::hold_for_event(DEEP_IDLE_HARD_CAP_S, "GNSS in deep idle");
	}

	// Cooldown gate (2026-05): refuse to schedule a GPS acquisition while
	// MIN_SURFACE_CYCLE_INTERVAL_S is still running. Without this guard, the
	// boot path (Service::start → reschedule → here) computes a UTC-aligned
	// next schedule that fires inside the cooldown window — symptom: GPS
	// launching right after a watchdog/POR reset that lands mid-cooldown.
	// Same trap on the AXL-wakeup / Argos-RX / first-fix retry reschedule
	// paths, all of which route through here without otherwise touching
	// is_in_cooldown(). When cooldown expires, SWS re-emits its state and
	// notify_underwater_state(false) rewakes us via the normal path.
	if (rtc && rtc->is_set() && ServiceManager::is_in_cooldown(service_current_time())) {
		const std::time_t now_s = service_current_time();
		return ScheduleDecision::hold_until(
		    (uint32_t)(now_s + (std::time_t)ServiceManager::get_cooldown_remaining_s(now_s)),
		    "surface-cycle cooldown");
	}

	// Rate-limit gate (2026-05-23): if Argos TX is rate-limited, acquiring
	// a GPS fix now is pure waste — no TX can fire to actually use the fix
	// until the window expires. Field log on 2026-05-23 showed GPS running
	// a full 70 s cold-acq session during a 45 s rate-limit reschedule —
	// 100 % wasted current. Symptom from sealed deploy POV: shorter
	// effective battery. Mirror the same RateLimiter::is_blocked check
	// ArgosTxService uses (argos_tx_service.cpp:139); return reschedule_in_s
	// matching the rate-limit window so GPS re-checks when it could be useful.
	{
		unsigned int rl_reschedule_s = 0;
		if (rtc && rtc->is_set() && RateLimiter::is_blocked(service_current_time(), rl_reschedule_s)) {
			DEBUG_INFO("GPSService::service_next_schedule_in_ms: rate-limited, reschedule in %u s", rl_reschedule_s);
			// Bound the conversion: rl_reschedule_s is 32-bit and RLP02
			// (RATE_LIMIT_WINDOW_S) accepts up to 0xFFFFFFFF, so any value above
			// 4294967 s (~49.7 days) wraps in `* 1000` and turns a very long
			// wait into a near-immediate one -- the exact inversion of what the
			// limiter is for. Clamp instead: SCHEDULE_DISABLED is 0xFFFFFFFF and
			// must stay reachable only as the "nothing planned" sentinel.
			constexpr unsigned int MAX_RESCHEDULE_MS = 0xFFFFFFFEu;
			unsigned int rl_reschedule_ms =
			    (rl_reschedule_s > MAX_RESCHEDULE_MS / 1000u) ? MAX_RESCHEDULE_MS : rl_reschedule_s * 1000u;
			return ScheduleDecision::run(rl_reschedule_ms, "Argos rate limit");
		}
	}

#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
	// Gate: while waiting for the first Argos TX to complete, no GPS
	// scheduling at all. The gate-arming code in notify_peer_event UW=0
	// calls service_reschedule(false), which routes through here — and
	// returning SCHEDULE_DISABLED here is what actually cancels the
	// UTC-aligned boot-time schedule. The ArgosTx SERVICE_INACTIVE
	// handler clears the gate then calls service_reschedule(immediate)
	// which routes back here and now returns a real delay. The bound below is
	// the second net, not the release: only that handler can clear the flag.
	if (m_defer_gnss_until_argos_first_tx) {
		// m_defer_gnss_timeout_task is the primary release -- it clears the flag,
		// which this bound cannot do. The bound is the second net: without a
		// flag clear, a re-evaluation lands right back here.
		return ScheduleDecision::hold_for_event(DEFER_GNSS_MAX_S + 10,
		                                        "deferred until the first Argos TX");
	}
#endif

	// Single fix mode: don't reschedule GNSS after first successful fix
	if (m_is_first_fix_found && configuration_store->read_param<bool>(ParamID::GNSS_SESSION_SINGLE_FIX)) {
		DEBUG_INFO("GPSService: GNSS_SESSION_SINGLE_FIX enabled | not rescheduling after first fix");
		return ScheduleDecision::off("GNSS_SESSION_SINGLE_FIX, fix already obtained");
	}

	// GNSS_NTRY: max consecutive failed acquisitions per cycle (0=unlimited).
	// Once the counter reaches the limit, back off to dloc_arg_nom
	// (GNSS_DELTATIME_ACQ). Do NOT reset here — the counter is only reset at
	// (a) end of the GNSS_DELTATIME_ACQ period, when service_initiate fires
	// the back-off attempt, and (b) a new surfacing event.
	unsigned int gnss_ntry = configuration_store->read_param<unsigned int>(ParamID::GNSS_NTRY);
	bool ntry_exhausted = (gnss_ntry > 0 && m_cold_start_ntry >= gnss_ntry);
	if (ntry_exhausted) {
		DEBUG_INFO("GPSService::retry_counter: NTRY limit reached (%u/%u) | back-off to dloc_arg_nom",
		           m_cold_start_ntry, gnss_ntry);
	}

	std::time_t now = service_current_time();
	std::time_t aq_period;
	if (m_is_first_schedule) {
		aq_period = FIRST_AQPERIOD_SEC;
	} else if (ntry_exhausted) {
		// NTRY exhausted: back off to dloc_arg_nom regardless of first_fix state
		aq_period = gnss_config.dloc_arg_nom;
	} else {
		aq_period = m_is_first_fix_found ? gnss_config.dloc_arg_nom : gnss_config.cold_start_retry_period;
	}

	const char *const aq_why = m_is_first_schedule    ? "first schedule"
	                           : ntry_exhausted       ? "NTRY back-off to GNSS_DELTATIME_ACQ"
	                           : m_is_first_fix_found ? "GNSS_DELTATIME_ACQ"
	                                                  : "cold-start retry";

	DEBUG_INFO("GPSService::retry_counter: ntry=%u limit=%u exhausted=%u aq_period=%us (%s)", m_cold_start_ntry,
	           gnss_ntry, (unsigned)ntry_exhausted, (unsigned)aq_period, aq_why);

	if (aq_period == 0) {
		DEBUG_INFO("GPSService: aq_period=0 (config) — GNSS periodic scheduling disabled");
		return ScheduleDecision::off("GNSS acquisition period is 0");
	}

	// Find the next schedule time aligned to UTC 00:00
	std::time_t next_schedule = now - (now % aq_period) + aq_period;

	DEBUG_TRACE("GPSService::reschedule: is_first=%u first_fix=%u cold=%u aqperiod=%u now=%u next=%u",
	            (unsigned int)m_is_first_schedule, (unsigned int)m_is_first_fix_found,
	            (unsigned int)gnss_config.cold_start_retry_period, (unsigned int)aq_period, (unsigned int)now,
	            (unsigned int)next_schedule);

	DEBUG_INFO("GPSService: next GNSS acquisition in %u s (UTC-aligned, period=%u s)",
	           (unsigned int)(next_schedule - now), (unsigned int)aq_period);

	// Find the time in milliseconds until this schedule (cast to uint64_t to prevent
	// overflow when aq_period > 4294 seconds, since the result is truncated to unsigned int)
	uint64_t delay_ms = static_cast<uint64_t>(next_schedule - now) * MS_PER_SEC;
	return ScheduleDecision::run((delay_ms > UINT32_MAX) ? UINT32_MAX : static_cast<unsigned int>(delay_ms),
	                             aq_why);
}

/// @brief Start GNSS acquisition — configure nav settings, power on M10Q.
void GPSService::service_initiate() {
	// HIGH #3 audit fix: detect re-entry during a previous session's teardown.
	// Triggered by peer SERVICE_INACTIVE notifications fired synchronously
	// from service_complete() inside react(GPSEventCloudLocateReady) — those
	// notifications can route back through this path before the M10Q state
	// machine has reached idle. Without this gate, the re-entry runs power_on()
	// against a transient state (stopreceive/fetchdatabase/poweroff) → races
	// with the pending teardown ticks → UART double-init / use-after-deinit.
	//
	// Strategy: defer by 200 ms — long enough for the teardown chain to settle
	// (state_poweroff() either cuts the rail or routes to enterbackup) and the
	// device to reach idle/backupidle. The framework's service_complete handler
	// will then naturally reschedule us through the standard path.
	if (m_is_active) {
		DEBUG_WARN(
		    "GPSService::service_initiate: re-entry during active session (peer reschedule race) — deferring 200 ms");
#if VALIDATION_LOG_ENABLE
		DEBUG_INFO("[VAL-GNSS] service_initiate_deferred reason=already_active");
#endif
		// Bounded, cancellable, and it cannot throw out of the scheduler.
		//
		// The handle used to be discarded, which made this the one GPS task
		// service_term() could not cancel -- and it calls power_on() further
		// down, so a deferral posted just before teardown could re-power the
		// M10Q rail on a service that had been shut down. It also called
		// service_initiate() straight from a scheduler task, outside the
		// try/catch that Service::run_scheduled_task wraps around it, so a
		// throw here reached the task runner and terminated.
		if (++m_initiate_retry_count > INITIATE_RETRY_MAX) {
			DEBUG_ERROR("GPSService::service_initiate: still active after %u deferrals — giving up on this slot",
			            INITIATE_RETRY_MAX);
			m_initiate_retry_count = 0;
			return;
		}
		m_initiate_retry_task = system_scheduler->post_task_prio(
		    [this]() {
			    if (!is_started()) return;  // torn down while we were waiting
			    try {
				    service_initiate();
			    } catch (...) {
				    DEBUG_ERROR("GPSService: exception from the deferred service_initiate — dropping this slot");
			    }
		    },
		    "GPSServiceInitiateRetry", Scheduler::DEFAULT_PRIORITY, 200);
		return;
	}

	m_initiate_retry_count = 0;  // past the re-entry gate: this slot is ours

	// 2026-05 deep-idle refactor: cancel any pending auto-poweroff. We're about
	// to acquire a fresh fix — the disposition will be re-armed at end of
	// session via try_enter_deep_idle_or_poweroff().
	system_scheduler->cancel_task(m_deep_idle_auto_off_task);
	m_deep_idle_started_at_ms = 0;  // R5: leaving deep-idle (entering acquisition)

	// Per-session GNSS diagnostic (2026-06) — one consolidated field-visible line
	// to root-cause the "fixes die after ~2 days" issue without device access.
	//   - rtc_set / rtc : injected time validity & value. ANO/AssistNow are useless
	//     with a wrong time; comparing rtc vs real elapsed across sessions reveals a
	//     drifting / non-syncing host clock (hypothesis H1).
	//   - batt_mv / lb  : low-battery mode switches GNSS to LB_* params and can
	//     throttle/disable acquisition around day 2 (hypothesis H3).
	//   - anow / ano    : AssistNow Online / Offline enabled? cross-check with the
	//     `ano_sent msgs=..` line from the M10Q driver to confirm ANO is applied.
	//   - first_fix / ntry / dead : a rising dead-session count = receiver stuck on
	//     stale assistance (H2), the trigger for GNSS_COLD_START_AFTER_NTRY.
	{
		bool rtc_set = (rtc && rtc->is_set());
		unsigned int rtc_now = rtc_set ? (unsigned int)rtc->gettime() : 0;
		service_update_battery();
		unsigned int batt_mv = service_get_voltage();
		bool lb = service_is_battery_level_low();
		bool anow = configuration_store->read_param<bool>(ParamID::GNSS_ASSISTNOW_EN);
		bool ano = configuration_store->read_param<bool>(ParamID::GNSS_ASSISTNOW_OFFLINE_EN);
		DEBUG_INFO(
		    "GPSService: SESSION_DIAG rtc_set=%u rtc=%u batt_mv=%u lb=%u anow=%u ano=%u first_fix=%u ntry=%u dead=%u",
		    (unsigned)rtc_set, rtc_now, batt_mv, (unsigned)lb, (unsigned)anow, (unsigned)ano,
		    (unsigned)m_is_first_fix_found, m_cold_start_ntry, m_consecutive_dead_sessions);
		VAL_GNSS("session_diag rtc_set=%u rtc=%u batt_mv=%u lb=%u anow=%u ano=%u first_fix=%u ntry=%u dead=%u",
		         (unsigned)rtc_set, rtc_now, batt_mv, (unsigned)lb, (unsigned)anow, (unsigned)ano,
		         (unsigned)m_is_first_fix_found, m_cold_start_ntry, m_consecutive_dead_sessions);
	}

	// FIX 2026-05-23 (audit finding cross #2): re-check rate-limit at the
	// service_initiate entry. service_next_schedule_in_ms gates rate-limited
	// sessions, but when the framework's reschedule task fires (Service::
	// reschedule posts a delayed task that calls service_initiate DIRECTLY,
	// without re-running service_next_schedule_in_ms — see service.cpp:586),
	// the gate check is bypassed. Without this re-check, a rate-limited GPS
	// would still power-on at the rescheduled time, burning a full
	// cold-acq window of current for a fix that can't be TX'd.
	{
		unsigned int rl_reschedule_s = 0;
		if (rtc && rtc->is_set() && RateLimiter::is_blocked(service_current_time(), rl_reschedule_s)) {
			DEBUG_INFO("GPSService::service_initiate: rate-limited (reschedule_s=%u), aborting fire", rl_reschedule_s);
			// GNSS MED #4 audit fix: cancel the safety-net timeout posted
			// by Service::reschedule() just before this call. Otherwise it
			// fires ~120 s later, triggers service_cancel + a redundant
			// reschedule, and leaks scheduler slots. Over a year-long
			// deployment with rate limiting active, these accumulate.
			cancel_safety_timeout();
			// Caller (Service::reschedule lambda) does not expect us to
			// re-schedule ourselves — but if we just return, no further
			// task fires. service_complete with shall_reschedule=true so
			// the framework calls reschedule() which routes back through
			// service_next_schedule_in_ms (which will return the same
			// reschedule_s and re-arm the timer correctly).
			m_is_active = false;
			service_complete(nullptr, nullptr, true);
			return;
		}
	}

	// If a backup-cell charge session is in flight (active or pending retry),
	// abort it synchronously so the device returns to idle before we request a
	// normal acquisition. Cancel both the auto-exit timer (active case) and the
	// retry scheduler (pending case) — only one will be armed but cancelling a
	// fresh handle is a no-op.
	if (m_backup_active || m_pending_backup_duration_s > 0) {
		DEBUG_INFO("GPSService::service_initiate: aborting backup-charge to start GNSS acquisition");
		system_scheduler->cancel_task(m_backup_exit_task);
		system_scheduler->cancel_task(m_backup_retry_task);
		m_pending_backup_duration_s = 0;
		if (m_backup_active) {
			m_device.exit_backup_charge_mode();
			m_backup_active = false;
		}
	}

	GNSSConfig gnss_config;
	configuration_store->get_gnss_configuration(gnss_config);

	// End of GNSS_DELTATIME_ACQ back-off: if we arrive here with the counter
	// still at/above NTRY, the previous schedule was a dloc_arg_nom back-off
	// that has now expired. Reset the counter so this new attempt starts
	// fresh. (The other reset trigger is a surfacing event — see
	// service_is_triggered_on_surfaced.)
	unsigned int gnss_ntry = configuration_store->read_param<unsigned int>(ParamID::GNSS_NTRY);
	if (gnss_ntry > 0 && m_cold_start_ntry >= gnss_ntry) {
		DEBUG_INFO("GPSService::retry_counter: reset ntry=%u->0 (end of GNSS_DELTATIME_ACQ back-off)",
		           m_cold_start_ntry);
		m_cold_start_ntry = 0;
	}

	GPSNavSettings nav_settings = {
		gnss_config.fix_mode,           gnss_config.dyn_model,
		gnss_config.assistnow_enable,   gnss_config.assistnow_offline_enable,
		gnss_config.hdop_filter_enable, gnss_config.hdop_filter_threshold,
		gnss_config.hacc_filter_enable, gnss_config.hacc_filter_threshold,
	};

	nav_settings.num_consecutive_fixes = gnss_config.min_num_fixes;
	nav_settings.sat_tracking = true;

	// Force a true cold start (CFG-RST navBbrMask=0xFFFF) when requested — wipes
	// any stale/corrupt BBR (ephemeris/almanac/pos/time/clock) before the M10Q
	// re-seeds from injected time/pos + ANO (done later in the same configure
	// sequence). One-shot: consume and clear here so the next acquisition goes
	// back to a normal hot start unless re-requested.
	bool force_cold = m_force_cold_start;
	m_force_cold_start = false;
	nav_settings.cold_start = force_cold;
	if (force_cold) DEBUG_INFO("GPSService::service_initiate: COLD START requested — BBR wipe + cold timeout");

	// Adaptive timeout: use shorter timeout when assistance data is available.
	// A forced cold start always uses the full cold-start budget — a wiped
	// receiver needs the time to (re)download broadcast ephemeris.
	if (force_cold) {
		nav_settings.max_nav_samples = gnss_config.acquisition_timeout_cold_start;
	} else if (m_is_first_fix_found) {
		nav_settings.max_nav_samples = gnss_config.acquisition_timeout;
	} else {
		// Check if ANO data is available and fresh — if so, use warm timeout
		unsigned int stale_threshold_s = gnss_config.ano_stale_days ? gnss_config.ano_stale_days * 24 * 3600 : 0;
		GNSSAlmanacStatus ano_status = m_device.get_almanac_status(stale_threshold_s);
		if (ano_status.file_present && !ano_status.stale && ano_status.valid_records > 0) {
			DEBUG_TRACE("GPSService: ANO data fresh (%u records) | using warm timeout", ano_status.valid_records);
			nav_settings.max_nav_samples = gnss_config.acquisition_timeout;
		} else {
			nav_settings.max_nav_samples = gnss_config.acquisition_timeout_cold_start;
		}
	}
	// Cold start: enable all constellations (GPS+GAL+GLO+BDS) to maximize visible SVs
	nav_settings.constellation_mask = (m_is_first_fix_found && !force_cold) ? gnss_config.constellation_mask
	                                                                        : (gnss_config.constellation_mask | 0x0F);
	nav_settings.orbmaxerr = gnss_config.orbmaxerr;
	nav_settings.min_cno = gnss_config.min_cno;
	nav_settings.min_elev = gnss_config.min_elev;
	nav_settings.ano_stale_threshold_s = gnss_config.ano_stale_days ? gnss_config.ano_stale_days * 24 * 3600 : 0;

	// CloudLocate: enable MEAS message capture when ALL of:
	//   - fastloc mode is CLOUDLOCATE
	//   - ARGOS_MODE is SURFACING_BURST (LEGACY mode raw-measurement-via-TR_NOM
	//     is not yet wired up — TODO: add a dedicated GUI behavior for LEGACY
	//     CloudLocate timing before re-enabling)
	//   - AND one of:
	//     - we don't already have a real fix in this surface session (default
	//       battery-friendly behavior — once a normal fix is obtained,
	//       subsequent acquisitions should not waste a full cold_acq_timeout
	//       on raw measurement capture)
	//     - OR GNSS_CLOUDLOCATE_ALWAYS=true (override: capture raw-meas at
	//       every surface, including warm. Use case: short-surface tortue
	//       where warm fix often misses the 30 s timeout and we want a
	//       CloudLocate fallback for every surface. Costs the full
	//       cold_acq_timeout each surface even when a real fix arrives.)
	unsigned int fastloc_mode = configuration_store->read_param<unsigned int>(ParamID::GNSS_FASTLOC_MODE);
	BaseArgosMode argos_mode = configuration_store->read_param<BaseArgosMode>(ParamID::ARGOS_MODE);
	bool cloudlocate_always = configuration_store->read_param<bool>(ParamID::GNSS_CLOUDLOCATE_ALWAYS);
	nav_settings.cloudlocate_enable = (fastloc_mode == (unsigned int)BaseFastlocMode::CLOUDLOCATE)
	                                  && (argos_mode == BaseArgosMode::SURFACING_BURST)
	                                  && (!m_is_first_fix_found || cloudlocate_always);

	m_next_schedule = service_current_time();
	m_is_first_schedule = false;
	m_wakeup_time = service_current_timer();
	m_is_active = true;
	DEBUG_INFO("GPSService: M10Q on — nav_max=%u cold=%u cl=%u const=0x%02x",
	           (unsigned int)nav_settings.max_nav_samples, (unsigned int)force_cold,
	           (unsigned int)nav_settings.cloudlocate_enable, (unsigned int)nav_settings.constellation_mask);
	m_device.power_on(nav_settings);
}

/// @brief Cancel active acquisition — power off device, log invalid entry.
/// @return true if acquisition was active and cancelled.
bool GPSService::service_cancel() {
	// Cleanly terminate
	DEBUG_TRACE("GPSService::service_cancel");

	if (m_is_active) {
		DEBUG_WARN("GPSService::service_cancel: active acquisition force-cancelled (safety timeout or teardown) — "
		           "logging NO_FIX");
		m_is_active = false;
		try_enter_deep_idle_or_poweroff();  // 2026-05 deep-idle dispatch
		GPSLogEntry log_entry = invalid_log_entry();
		ServiceEventData event_data = log_entry;
		service_complete(&event_data, &log_entry);
		// LED dispatch: signal which path the dispatcher took. Drives the
		// LEDGNSSDeepIdle (double-blink red) vs LEDGNSSPowerOff (fast blink
		// red) state in the LED FSM. No-fix path only; the fix path keeps
		// the existing LEDGNSSOffWithFix (green) without overlay.
		notify_service_event(m_last_dispatch_was_deep_idle ? ServiceEventType::GNSS_OFF_DEEP_IDLE
		                                                   : ServiceEventType::GNSS_OFF_POWEROFF);
		return true;
	}

	return false;
}

/// @brief Safety timeout — acquisition timeout + 30s margin.
/// @return Timeout in ms after which acquisition is force-cancelled.
unsigned int GPSService::service_next_timeout() {
	// Safety net in case the device never responds (e.g., hardware failure)
	// Must exceed the GNSS acquisition timeout to avoid premature termination
	GNSSConfig gnss_config;
	configuration_store->get_gnss_configuration(gnss_config);
	// 2026-08: the safety net has to cover the budget service_initiate() will
	// ACTUALLY give the driver. A forced cold start (the
	// GNSS_COLD_START_AFTER_NTRY or GNSS_TRIGGER_COLD_START_ON_SURFACED escalation)
	// asks for acquisition_timeout_cold_start even when a fix has already been
	// obtained; without this test we armed the net on acquisition_timeout (+30 s)
	// and cancelled the session at 150 s while the driver was aiming for 530 s. The
	// receiver we had just wiped therefore never had time to re-download its
	// ephemeris: the recovery escalation could not succeed.
	// m_force_cold_start is still armed here -- Service::reschedule() computes this
	// timeout BEFORE calling service_initiate(), which is the only consumer.
	bool cold_budget = !m_is_first_fix_found || m_force_cold_start;
	unsigned int timeout_s = cold_budget ? gnss_config.acquisition_timeout_cold_start : gnss_config.acquisition_timeout;
	return (timeout_s + SERVICE_SAFETY_MARGIN_S) * MS_PER_SEC;
}

/// @brief Trigger immediate GNSS on surfacing if configured.
/// @param[out] immediate  Set to true if GNSS should fire immediately on surfacing.
/// @return true (always reschedule on surface).
bool GPSService::service_is_triggered_on_surfaced(bool &immediate) {
	GNSSConfig gnss_config;
	configuration_store->get_gnss_configuration(gnss_config);
	immediate = gnss_config.trigger_on_surfaced;
	if (m_cold_start_ntry != 0) {
		// Demoted to TRACE: fires on every surface cycle after a NO_FIX (which is
		// frequent). Adds ~50-300 ms LFS commit on the surfacing critical path.
		// Counter value is visible in the "retry_counter: ntry=N limit=N..." log
		// emitted by service_next_schedule_in_ms when the new GNSS attempt is scheduled.
		DEBUG_TRACE("GPSService::retry_counter: reset ntry=%u->0 (surfacing)", m_cold_start_ntry);
	}
	m_cold_start_ntry = 0;  // Reset retry counter on each surfacing
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
	if (m_defer_gnss_until_argos_first_tx) {
		// Demoted to TRACE: redundant with "GPSService: surfaced — GNSS deferred..."
		// which already marks the gate arming.
		DEBUG_TRACE("GPSService: surfacing trigger DEFERRED — waiting for first Argos TX");
		return false;
	}
#endif
	return true;
}

/// @brief GNSS is not usable underwater (RF blocked by water).
/// @return Always false.
bool GPSService::service_is_usable_underwater() {
	return false;
}

/// @brief Build a GPS log entry with NO_FIX status (timeout or error).
/// @return GPSLogEntry with valid=false and current battery/time.
GPSLogEntry GPSService::invalid_log_entry() {
	DEBUG_TRACE("GPSService::invalid_log_entry");
	m_cold_start_ntry++;
	unsigned int ntry_limit = configuration_store->read_param<unsigned int>(ParamID::GNSS_NTRY);
	DEBUG_INFO("GPSService::retry_counter: NO_FIX | ntry=%u/%s", m_cold_start_ntry,
	           ntry_limit == 0 ? "unlimited" : std::to_string(ntry_limit).c_str());

	// Safety-net 3.5: a NO_FIX path is still a GPS event — re-arm health WDT.
	// NOT 3.6 — NO_FIX is the opposite of a real PVT, so we don't want to
	// re-arm the no-PVT WDT (otherwise an infinitely-failing tag would never
	// trigger the safety net 3.6 reset).
	arm_health_wdt();

	// Stuck-M10Q recovery (safety-net 3.2): count consecutive end-of-session
	// paths that produce neither a PVT nor a CloudLocate. After STUCK_THRESHOLD
	// dead sessions, arm a hard rail-cycle to flush any latched M10Q state.
	// The counter is reset by the three success callbacks (gnss_data_callback,
	// gnss_degraded_callback, gnss_cloudlocate_callback). Single-arm via
	// m_stuck_recovery_in_flight so we don't queue multiple recoveries.
	//
	// A session the DIVE cut short is not a GNSS failure, and neither the count
	// nor the escalations below apply to it. notify_underwater_state sets the
	// underwater flag before calling service_cancel(), so this reads true for a
	// dive and false for an ordinary acquisition timeout.
	//
	// Counting them was actively harmful rather than merely inaccurate: every
	// Nth dead session forces a COLD START, which wipes the backup RAM and
	// destroys the ephemeris -- making the next fix HARDER, on a receiver that
	// was working fine and simply went under water. On an animal that dives
	// through most of its acquisition windows, nothing else drove the counter.
	//
	// Skipping rather than clearing the counter on surfacing: clearing would
	// also erase the genuine consecutive failures accumulated from one surface
	// session to the next, which is exactly what the counter is for.
	//
	// Note the whole block is skipped, not just the increment: with the counter
	// left at 0, `m_consecutive_dead_sessions % cold_after_ntry` is 0 too, so
	// guarding the increment alone would have forced a cold start on every
	// dive-cut session -- the opposite of the intent.
	if (service_is_underwater()) {
		DEBUG_TRACE("GPSService: session cut short by a dive — not counted as a dead session");
	} else {
		m_consecutive_dead_sessions++;

		// Auto cold-start escalation (GNSS_COLD_START_AFTER_NTRY). When enabled
		// (>0), every Nth consecutive dead session requests a TRUE cold start (BBR
		// wipe) on the next acquisition — flushes stale/corrupt ephemeris/almanac
		// that a plain rail-cycle (which preserves V_BCKP/BBR) would NOT clear.
		// Fires well before the heavier STUCK_THRESHOLD rail-cycle and re-fires
		// every N sessions while still stuck. Counter resets on any fix, so healthy
		// tags never reach N — no regression when the param is left at 0 (default).
		unsigned int cold_after_ntry = service_read_param<unsigned int>(ParamID::GNSS_COLD_START_AFTER_NTRY);
		if (cold_after_ntry > 0 && (m_consecutive_dead_sessions % cold_after_ntry) == 0) {
			m_force_cold_start = true;
			DEBUG_WARN("GPSService: %u consecutive dead sessions >= NTRY %u — next acquisition COLD START (BBR wipe)",
			           m_consecutive_dead_sessions, cold_after_ntry);
			VAL_GNSS("dispatch=cold_start_after_ntry sessions=%u ntry=%u", m_consecutive_dead_sessions,
			         cold_after_ntry);
		}

		if (m_consecutive_dead_sessions >= STUCK_THRESHOLD && !m_stuck_recovery_in_flight) {
			m_stuck_recovery_in_flight = true;
			VAL_GNSS("dispatch=stuck_recovery_armed sessions=%u", m_consecutive_dead_sessions);
			DEBUG_WARN("GPSService: %u consecutive dead sessions — scheduling hard rail-cycle",
			           m_consecutive_dead_sessions);
			// Fire after current invalidate completes (~1 s), then 30 s rail-down
			// for a clean M10Q hardware reset before normal scheduling resumes.
			m_stuck_recovery_arm_task = system_scheduler->post_task_prio(
			    [this]() {
				    // Guard: if a new acquisition is in flight when we'd yank the
				    // rail, skip this recovery cycle. The counter is NOT reset so the
				    // next dead session will re-arm naturally.
				    if (m_is_active) {
					    VAL_GNSS("dispatch=stuck_recovery_skipped_active");
					    m_stuck_recovery_in_flight = false;
					    return;
				    }
				    VAL_GNSS("dispatch=stuck_recovery_rail_down");
				    DEBUG_WARN("GPSService: stuck recovery — power_off_immediate, 30 s rail-down");
				    m_device.power_off_immediate();
				    m_stuck_recovery_done_task = system_scheduler->post_task_prio(
				        [this]() {
					        VAL_GNSS("dispatch=stuck_recovery_done");
					        DEBUG_INFO("GPSService: stuck recovery — rail-down complete, rescheduling");
					        m_stuck_recovery_in_flight = false;
					        m_consecutive_dead_sessions = 0;
					        m_wdt_cold_start_tried = false;  // fresh chance after recovery
					        service_reschedule(false);
				        },
				        "GPSStuckRecoveryDone", Scheduler::DEFAULT_PRIORITY, 30 * MS_PER_SEC);
			    },
			    "GPSStuckRecoveryArmed", Scheduler::DEFAULT_PRIORITY, 1 * MS_PER_SEC);
		}
	}

	GPSLogEntry gps_entry{};

	gps_entry.header.log_type = LOG_GPS;

	service_set_log_header_time(gps_entry.header, service_current_time());

	service_update_battery();
	gps_entry.info.batt_voltage = service_get_voltage();
	gps_entry.info.event_type = GPSEventType::NO_FIX;
	gps_entry.info.valid = false;
	gps_entry.info.onTime = service_current_timer() - m_wakeup_time;
	gps_entry.info.schedTime = m_next_schedule;

	return gps_entry;
}

/// @brief Process valid GNSS fix — log entry, notify config store, complete service.
void GPSService::task_process_gnss_data() {
	DEBUG_TRACE("GPSService::task_process_gnss_data");
	if (m_cold_start_ntry != 0) {
		DEBUG_INFO("GPSService::retry_counter: reset ntry=%u->0 (PVT fix)", m_cold_start_ntry);
	}
	m_cold_start_ntry = 0;

	GPSLogEntry gps_entry{};
	gps_entry.header.log_type = LOG_GPS;
	service_set_log_header_time(gps_entry.header, service_current_time());

	service_update_battery();
	gps_entry.info.batt_voltage = service_get_voltage();
	copy_gnss_to_log(m_gnss_data.data, gps_entry);
	gps_entry.info.onTime = service_current_timer() - m_wakeup_time;

	if (m_num_gps_fixes == 1) {
		// For the very first fix, we need to compute the scheduled GPS time since the RTC
		// wasn't set beforehand
		std::time_t fix_time = convert_epochtime(gps_entry.info.year, gps_entry.info.month, gps_entry.info.day,
		                                         gps_entry.info.hour, gps_entry.info.min, gps_entry.info.sec);
		gps_entry.info.schedTime = fix_time - (gps_entry.info.onTime / MS_PER_SEC);
	} else {
		gps_entry.info.schedTime = m_next_schedule;
	}

	gps_entry.info.event_type = GPSEventType::FIX;
	gps_entry.info.valid = true;

	// Demoted to TRACE: per-fix payload dump (~50-300 ms LFS commit). The
	// GPSLogEntry itself is persisted via the log queue, so the lat/lon are
	// already retrievable. retry_counter reset (line above) marks the event.
	DEBUG_TRACE("GPSService::task_process_gnss_data: lat=%lf lon=%lf hDOP=%lf hAcc=%lf numSV=%u batt=%lfV ",
	            gps_entry.info.lat, gps_entry.info.lon, static_cast<double>(gps_entry.info.hDOP),
	            static_cast<double>(gps_entry.info.hAcc), gps_entry.info.numSV,
	            (double)gps_entry.info.batt_voltage / 1000);

	// Notify configuration store that we have a new valid GPS fix
	configuration_store->notify_gps_location(gps_entry);

	// Anchor LED HRS_24 RTC cutoff on the first valid GNSS fix.
	// The cutoff (RTC epoch at which the HRS_24 window expires) is persisted
	// in flash so it survives TPL5111 hard shutdowns on EXTERNAL_WAKEUP boards.
	// 0 = unset sentinel; we write it once and ledsm.cpp reads it back to
	// gate LEDs in HRS_24 mode. RTC is set by the M10Q driver from this same
	// fix's date/time fields, so service_current_time() is the GNSS time.
	//
	// Only relevant on EXTERNAL_WAKEUP boards (RSPB) — ledsm.cpp gates the
	// cutoff read with the same #ifdef, so on regular boards (LINKIT) the
	// value would never be read. Writing it anyway burns a flash erase/write
	// cycle and a save_params() round-trip on every fresh deployment for
	// nothing. Mirror the #ifdef here to keep the symmetry tight.
#ifdef EXTERNAL_WAKEUP
	if (service_is_time_known()) {
		std::time_t led_cutoff = configuration_store->read_param<std::time_t>(ParamID::LED_HRS24_RTC_CUTOFF);
		if (led_cutoff == 0) {
			led_cutoff = service_current_time() + 24 * 3600;
			configuration_store->write_param(ParamID::LED_HRS24_RTC_CUTOFF, led_cutoff);
			configuration_store->save_params();
			DEBUG_INFO("GPSService: LED HRS_24 cutoff anchored at RTC=%u", (unsigned int)led_cutoff);
		}
	}
#endif

	ServiceEventData event_data = gps_entry;
	service_complete(&event_data, &gps_entry);
	// LED dispatch: fix path. Drives LEDGNSSDeepIdle / LEDGNSSPowerOff entry,
	// which read the latched fix-validity static and render GREEN for the
	// fix case (vs RED for no-fix). Sequenced AFTER service_complete so the
	// LED FSM's SERVICE_LOG_UPDATED handler latches valid=true first.
	notify_service_event(m_last_dispatch_was_deep_idle ? ServiceEventType::GNSS_OFF_DEEP_IDLE
	                                                   : ServiceEventType::GNSS_OFF_POWEROFF);
}

/// @brief Process degraded GNSS fix (fastloc) — log entry with FASTLOC event type.
void GPSService::task_process_degraded_gnss_data() {
	DEBUG_TRACE("GPSService::task_process_degraded_gnss_data");
	DEBUG_INFO("GPSService::retry_counter: reset ntry=%u->0 (DEGRADED_PVT success)", m_cold_start_ntry);
	m_cold_start_ntry = 0;

	GPSLogEntry gps_entry{};
	gps_entry.header.log_type = LOG_GPS;
	service_set_log_header_time(gps_entry.header, service_current_time());

	service_update_battery();
	gps_entry.info.batt_voltage = service_get_voltage();
	copy_gnss_to_log(m_gnss_data.data, gps_entry);
	gps_entry.info.onTime = service_current_timer() - m_wakeup_time;
	gps_entry.info.schedTime = m_next_schedule;

	// Mark as fastloc (degraded position — valid but low quality)
	gps_entry.info.event_type = GPSEventType::FASTLOC;
	gps_entry.info.valid = true;

	// Demoted to TRACE: per-fastloc payload dump. The entry is persisted via
	// the log queue and broadcast via service_complete; lat/lon are not lost.
	DEBUG_TRACE("GPSService::task_process_degraded_gnss_data: lat=%lf lon=%lf hAcc=%u numSV=%u fixType=%u",
	            gps_entry.info.lat, gps_entry.info.lon, gps_entry.info.hAcc, gps_entry.info.numSV,
	            gps_entry.info.fixType);

	// Cache for Phase-1 surfacing burst: when next surface fires before a
	// fresh fix is available, this entry is used as the 1st-ping payload.
	configuration_store->notify_fastloc_location(gps_entry);

	ServiceEventData event_data = gps_entry;
	service_complete(&event_data, &gps_entry);
	// LED dispatch: degraded-fix path (GPSLogEntry has info.valid based on
	// the degraded data — sometimes valid, sometimes not). Color picked by
	// the LED FSM from the latched fix-validity static.
	notify_service_event(m_last_dispatch_was_deep_idle ? ServiceEventType::GNSS_OFF_DEEP_IDLE
	                                                   : ServiceEventType::GNSS_OFF_POWEROFF);
}

/// @brief Process CloudLocate raw measurement — overlay blob into GPSLogEntry position fields.
void GPSService::task_process_cloudlocate_data() {
	DEBUG_TRACE("GPSService::task_process_cloudlocate_data");
	// Note (QA review R6): CloudLocate emits raw measurements without an on-device
	// position. Resetting m_cold_start_ntry here treats the raw capture as a
	// "successful" cold-start outcome — chains of CloudLocate captures will
	// therefore never trigger the NTRY back-off to dloc_arg_nom. This is
	// intentional: CloudLocate is the deployed fallback strategy, so we should
	// keep trying every cold_start_retry_period rather than giving up.
	DEBUG_INFO("GPSService::retry_counter: reset ntry=%u->0 (CLOUDLOCATE success)", m_cold_start_ntry);
	m_cold_start_ntry = 0;

	GPSLogEntry gps_entry{};
	gps_entry.header.log_type = LOG_GPS;
	service_set_log_header_time(gps_entry.header, service_current_time());

	service_update_battery();
	gps_entry.info.batt_voltage = service_get_voltage();
	gps_entry.info.onTime = service_current_timer() - m_wakeup_time;
	gps_entry.info.schedTime = m_next_schedule;
	gps_entry.info.event_type = GPSEventType::CLOUDLOCATE;
	gps_entry.info.valid = false;  // No on-device position

	// Overlay raw measurement blob into unused position fields (lat, lon, height, hMSL)
	// Layout: byte 0 = format ID, bytes 1-N = blob data
	// lat(8) + lon(8) + height(4) + hMSL(4) = 24 bytes available
	static_assert(offsetof(GPSInfo, hAcc) - offsetof(GPSInfo, lon) >= 21,
	              "GPSInfo overlay space too small for MEAS20 + format byte");
	uint8_t *overlay = reinterpret_cast<uint8_t *>(&gps_entry.info.lon);
	std::memset(overlay, 0, 24);

	unsigned int cl_format = configuration_store->read_param<unsigned int>(ParamID::GNSS_CLOUDLOCATE_FORMAT);
	// STRICT format policy (2026-06): store EXACTLY the operator-configured format.
	// Format and Argos modulation are INDEPENDENT operator choices (you can run
	// MEASC12 on LDA2 or MEAS20 on LDK if you want), so a silent cross-format
	// substitution (the old MEAS20->MEASC12 fallback) would (a) put a format on
	// air the operator didn't select and (b) silently re-introduce MEASC12's
	// position-hint dependency. If the configured format wasn't produced by the
	// receiver this session, store sentinel 0xFF — the TX path skips it instead
	// of sending a mismatched or empty packet.
	if (cl_format == (unsigned int)BaseCloudLocateFormat::MEASC12 && m_raw_measurement.has_measc12) {
		overlay[0] = (uint8_t)BaseCloudLocateFormat::MEASC12;
		std::memcpy(&overlay[1], m_raw_measurement.measc12, 12);
		DEBUG_INFO("GPSService::task_process_cloudlocate_data: stored MEASC12 (12 bytes)");
	} else if (cl_format == (unsigned int)BaseCloudLocateFormat::MEAS20 && m_raw_measurement.has_meas20) {
		overlay[0] = (uint8_t)BaseCloudLocateFormat::MEAS20;
		std::memcpy(&overlay[1], m_raw_measurement.meas20, 20);
		DEBUG_INFO("GPSService::task_process_cloudlocate_data: stored MEAS20 (20 bytes)");
	} else {
		overlay[0] = 0xFF;  // sentinel: configured format not produced this session
		DEBUG_WARN("GPSService::task_process_cloudlocate_data: configured format %u UNAVAILABLE (have measc12=%u "
		           "meas20=%u) — no CloudLocate TX",
		           cl_format, (unsigned)m_raw_measurement.has_measc12, (unsigned)m_raw_measurement.has_meas20);
		VAL_GNSS("cloudlocate_format_unavailable want=%u measc12=%u meas20=%u", cl_format,
		         (unsigned)m_raw_measurement.has_measc12, (unsigned)m_raw_measurement.has_meas20);
	}

	ServiceEventData event_data = gps_entry;
	service_complete(&event_data, &gps_entry);
	// LED dispatch: CloudLocate path. info.valid is typically false (no
	// on-device position), so LED FSM renders the RED variant — but the
	// sleep-depth distinction (double-blink vs fast blink) still applies.
	notify_service_event(m_last_dispatch_was_deep_idle ? ServiceEventType::GNSS_OFF_DEEP_IDLE
	                                                   : ServiceEventType::GNSS_OFF_POWEROFF);
}

/// @brief Acquisition timeout (max nav samples reached) — power off, log invalid.
void GPSService::react(const GPSEventMaxNavSamples &) {
	if (!m_is_active) return;
	DEBUG_INFO("GPSService::react(GPSEventMaxNavSamples): acquisition timeout — no fix, ending session");
	m_is_active = false;
	try_enter_deep_idle_or_poweroff();  // 2026-05 deep-idle dispatch
	GPSLogEntry log_entry = invalid_log_entry();
	ServiceEventData event_data = log_entry;
	service_complete(&event_data, &log_entry);
	notify_service_event(m_last_dispatch_was_deep_idle ? ServiceEventType::GNSS_OFF_DEEP_IDLE
	                                                   : ServiceEventType::GNSS_OFF_POWEROFF);
}

/// @brief No signal acquired (max sat samples) — power off, log invalid.
void GPSService::react(const GPSEventMaxSatSamples &) {
	if (!m_is_active) return;
	DEBUG_INFO("GPSService::react(GPSEventMaxSatSamples): no satellites acquired — no fix, ending session");
	m_is_active = false;
	try_enter_deep_idle_or_poweroff();  // 2026-05 deep-idle dispatch
	GPSLogEntry log_entry = invalid_log_entry();
	ServiceEventData event_data = log_entry;
	service_complete(&event_data, &log_entry);
	notify_service_event(m_last_dispatch_was_deep_idle ? ServiceEventType::GNSS_OFF_DEEP_IDLE
	                                                   : ServiceEventType::GNSS_OFF_POWEROFF);
}

/// @brief GPS hardware error — power off, log invalid.
void GPSService::react(const GPSEventError &) {
	if (!m_is_active) return;
	DEBUG_WARN("GPSService::react(GPSEventError): GNSS device error — aborting session, logging NO_FIX");
	m_is_active = false;
	try_enter_deep_idle_or_poweroff();  // 2026-05 deep-idle dispatch
	GPSLogEntry log_entry = invalid_log_entry();
	ServiceEventData event_data = log_entry;
	service_complete(&event_data, &log_entry);
	notify_service_event(m_last_dispatch_was_deep_idle ? ServiceEventType::GNSS_OFF_DEEP_IDLE
	                                                   : ServiceEventType::GNSS_OFF_POWEROFF);
}

/// @brief Device powered off — clear backup-charge flag if it was set
/// (the M10 emits this on exit_backup_charge_mode too, and on async failure of
/// state_enterbackup which bails to poweroff).
void GPSService::react(const GPSEventPowerOff &) {
	if (m_backup_active) {
		DEBUG_WARN("GPSService: backup-charge aborted by unexpected device power-off");
		m_backup_active = false;
		system_scheduler->cancel_task(m_backup_exit_task);
		// Fire stop callback so the FSM can recover (async hardware failure path).
		// Move semantics make this idempotent vs. backup_charge_stop_internal.
		auto cb = std::move(m_on_backup_charge_stop);
		m_on_backup_charge_start = nullptr;
		m_on_backup_charge_stop = nullptr;
		if (cb) cb();
	}
}

/// @brief Valid PVT fix received — power off, process and log.
void GPSService::react(const GPSEventPVT &e) {
	if (!m_is_active) return;
	m_is_active = false;
	// R4 clear: a valid fix proves the cold path works; release the WDT inhibit
	// so subsequent sessions can use the deep-idle fast-path.
	if (m_deep_idle_inhibit_first_session) {
		DEBUG_INFO("GPSService::react(GPSEventPVT): first clean fix — releasing deep-idle WDT inhibit");
		m_deep_idle_inhibit_first_session = false;
		m_inhibit_set_at_ms = 0;  // safety-net 3.1: reset 24h hard-cap timer
	}
	try_enter_deep_idle_or_poweroff();  // 2026-05 deep-idle dispatch
	gnss_data_callback(e.data);
}

/// @brief Degraded PVT received (fastloc fallback) — emit if DEGRADED_PVT mode enabled.
void GPSService::react(const GPSEventPVTDegraded &e) {
	if (!m_is_active) return;
	m_is_active = false;
	try_enter_deep_idle_or_poweroff();  // 2026-05 deep-idle dispatch

	// Only emit fastloc if mode is DEGRADED_PVT (1) or higher
	unsigned int fastloc_mode = configuration_store->read_param<unsigned int>(ParamID::GNSS_FASTLOC_MODE);
	if (fastloc_mode >= (unsigned int)BaseFastlocMode::DEGRADED_PVT) {
		DEBUG_INFO("GPSService::react(GPSEventPVTDegraded): fastloc hAcc=%u fixType=%u numSV=%u", e.data.hAcc,
		           e.data.fixType, e.data.numSV);
		gnss_degraded_callback(e.data);
	} else {
		DEBUG_INFO("GPSService::react(GPSEventPVTDegraded): fastloc disabled, treating as no fix");
		GPSLogEntry log_entry = invalid_log_entry();
		ServiceEventData event_data = log_entry;
		service_complete(&event_data, &log_entry);
		notify_service_event(m_last_dispatch_was_deep_idle ? ServiceEventType::GNSS_OFF_DEEP_IDLE
		                                                   : ServiceEventType::GNSS_OFF_POWEROFF);
	}
}

/// @brief First raw CloudLocate measurement available mid-acquisition — broadcast
/// to peer services so they can fire an early TX without waiting for the full
/// PVT timeout. GPS keeps running (NOT terminated here, unlike
/// react(GPSEventRawMeasurement)). One-shot per acquisition: the M10Q driver
/// has its own guard so this fires once per power_on().
void GPSService::react(const GPSEventCloudLocateReady &e) {
	if (!m_is_active) return;
	unsigned int fastloc_mode = configuration_store->read_param<unsigned int>(ParamID::GNSS_FASTLOC_MODE);
	if (fastloc_mode != (unsigned int)BaseFastlocMode::CLOUDLOCATE) {
		DEBUG_TRACE("GPSService::react(GPSEventCloudLocateReady): CLOUDLOCATE mode disabled — ignoring");
		return;
	}
	DEBUG_INFO("GPSService::react(GPSEventCloudLocateReady): broadcasting GNSS_CLOUDLOCATE_READY");
	notify_service_event(ServiceEventType::GNSS_CLOUDLOCATE_READY);

	// FAST3b (2026-05 deep-idle refactor): terminate the GNSS session as soon
	// as a raw measurement is available — don't wait for a full PVT fix or the
	// GNSS_ACQ_TIMEOUT. For tortue short-surface deployments where the
	// CloudLocate raw data IS the position payload (resolved cloud-side from
	// the Argos upload), waiting for a full PVT just burns battery.
	//
	// Drops typical session 30-120 s → 5-15 s. Trade-off: no local lat/lon
	// (no GPSEventPVT will fire), so no LAST_KNOWN_POS update for the next
	// session's MGA-INI-POS_LLH hint. Acceptable when the deployment relies
	// on cloud-side processing of raw measurements.
	if (configuration_store->read_param<bool>(ParamID::GNSS_CLOUDLOCATE_ONLY)) {
		DEBUG_INFO("GPSService::react(GPSEventCloudLocateReady): CLOUDLOCATE_ONLY — power-off, no PVT wait");
		m_is_active = false;

		// FIX 2026-05-23 (audit finding 5): reset NTRY counter so we don't
		// accumulate "failures" — capturing a raw measurement IS progress for
		// CLOUDLOCATE_ONLY deployments (the position is computed cloud-side
		// from this raw). Without this reset, two consecutive sessions that
		// timeout-without-PVT (which is the NORMAL CLOUDLOCATE_ONLY behavior)
		// would push NTRY into back-off and stretch GNSS_DELTATIME_ACQ for the
		// next acquisition — wrong incentive given a raw IS what we wanted.
		m_cold_start_ntry = 0;

		// R4 clear: even though CLOUDLOCATE_ONLY produces no PVT, we did
		// successfully reach the M10Q (raw meas implies UART OK + nav up).
		// That's enough to prove the cold path works post-WDT.
		if (m_deep_idle_inhibit_first_session) {
			DEBUG_INFO("GPSService::react(GPSEventCloudLocateReady): raw OK — releasing WDT inhibit");
			m_deep_idle_inhibit_first_session = false;
			m_inhibit_set_at_ms = 0;  // safety-net 3.1: reset 24h hard-cap timer
		}

		try_enter_deep_idle_or_poweroff();
		// Build a synthetic CloudLocate log entry from the raw measurement so
		// peer services get the same SERVICE_LOG_UPDATED hook they'd get on a
		// normal raw-meas-driven end-of-session (via gnss_cloudlocate_callback).
		gnss_cloudlocate_callback(e.data);
	}
}

/// @brief Raw GNSS measurement received (CloudLocate fallback) — emit if CLOUDLOCATE mode.
void GPSService::react(const GPSEventRawMeasurement &e) {
	if (!m_is_active) return;
	m_is_active = false;
	try_enter_deep_idle_or_poweroff();  // 2026-05 deep-idle dispatch

	unsigned int fastloc_mode = configuration_store->read_param<unsigned int>(ParamID::GNSS_FASTLOC_MODE);
	if (fastloc_mode == (unsigned int)BaseFastlocMode::CLOUDLOCATE) {
		// Demoted to TRACE: per-session size dump. The CloudLocate event log
		// already names which format was emitted.
		DEBUG_TRACE("GPSService::react(GPSEventRawMeasurement): CloudLocate measc12=%u meas20=%u meas50=%u",
		            e.data.has_measc12, e.data.has_meas20, e.data.has_meas50);
		gnss_cloudlocate_callback(e.data);
	} else {
		DEBUG_INFO("GPSService::react(GPSEventRawMeasurement): CloudLocate disabled, treating as no fix");
		GPSLogEntry log_entry = invalid_log_entry();
		ServiceEventData event_data = log_entry;
		service_complete(&event_data, &log_entry);
	}
}

/// @brief Valid fix callback — store data, mark first fix, process.
/// @param data  GNSS PVT data from M10Q driver.
void GPSService::gnss_data_callback(GNSSData data) {
	// Mark first fix flag
	m_gnss_data.data = data;
	m_is_first_fix_found = true;
	// Safety-net 3.2: M10Q produced a PVT → reset stuck-recovery state.
	m_consecutive_dead_sessions = 0;
	m_wdt_cold_start_tried = false;
	m_stuck_recovery_in_flight = false;
	system_scheduler->cancel_task(m_stuck_recovery_arm_task);
	system_scheduler->cancel_task(m_stuck_recovery_done_task);
	// Safety-nets 3.5 + 3.6: real PVT — re-arm BOTH watchdogs.
	arm_health_wdt();
	arm_no_pvt_wdt();
	m_num_gps_fixes++;
	task_process_gnss_data();
}

#ifdef BENCH_TEST
void GPSService::bench_inject_fix(double lat, double lon, uint32_t hAcc_mm, uint8_t numSV) {
	GNSSData d = {};
	d.fixType = 3;   // 3D fix
	d.valid = 0x07;  // validDate | validTime | fullyResolved
	d.numSV = numSV ? numSV : 8;
	d.lat = lat;
	d.lon = lon;
	d.height = 10000;  // 10 m (mm)
	d.hMSL = 10000;
	d.hAcc = hAcc_mm ? hAcc_mm : 2500;  // default 2.5 m
	d.vAcc = 3500;
	d.pDOP = 1.5f;
	d.hDOP = 1.0f;
	d.vDOP = 1.2f;
	d.ttff = 2000;  // 2 s simulated TTFF
	// Stamp the fix instant from the RTC so downstream time-of-fix logic and the
	// CSV log carry a coherent timestamp (mirrors generate_fake_fix()).
	if (rtc && rtc->is_set()) {
		std::time_t now = rtc->gettime();
		struct tm *t = gmtime(&now);
		if (t) {
			d.year = (uint16_t)(t->tm_year + 1900);
			d.month = (uint8_t)(t->tm_mon + 1);
			d.day = (uint8_t)t->tm_mday;
			d.hour = (uint8_t)t->tm_hour;
			d.min = (uint8_t)t->tm_min;
			d.sec = (uint8_t)t->tm_sec;
		}
	}
	DEBUG_WARN("GPSService::bench_inject_fix: lat=%f lon=%f hAcc=%umm numSV=%u", lat, lon, (unsigned)d.hAcc,
	           (unsigned)d.numSV);
	// Short-circuit any real acquisition still in flight so a later real PVT
	// doesn't double-log on top of the injected one.
	m_is_active = false;
	// R20: end the session the way the real paths do (react(GPSEventPVT)), or the
	// receiver stays in `receive` with the rail powered and the next power_on takes
	// m_num_power_on to 2 -- after which no power_off cuts anything.
	try_enter_deep_idle_or_poweroff();
	// Mark the GNSS service initiated so gnss_data_callback → task_process_gnss_data
	// → service_complete() runs its FULL path instead of bailing on
	// "!m_is_initiated". That full path calls service_log() → notify_log_updated()
	// which broadcasts SERVICE_LOG_UPDATED carrying the valid GPS entry to peers.
	// ArgosTxService consumes it to set m_gps_fix_corrected_clock ("TX unlocked")
	// and DepthPileManager stores the fix. Without this the injected fix logs but
	// never unlocks Argos TX — so LEGACY/BLIND could not be validated on the bench.
	bench_force_initiated();
	gnss_data_callback(d);
}

/// @brief Fill a GNSSData's date/time from the RTC (shared by the bench injectors).
static void bench_stamp_time(GNSSData &d) {
	if (rtc && rtc->is_set()) {
		std::time_t now = rtc->gettime();
		struct tm *t = gmtime(&now);
		if (t) {
			d.year = (uint16_t)(t->tm_year + 1900);
			d.month = (uint8_t)(t->tm_mon + 1);
			d.day = (uint8_t)t->tm_mday;
			d.hour = (uint8_t)t->tm_hour;
			d.min = (uint8_t)t->tm_min;
			d.sec = (uint8_t)t->tm_sec;
		}
	}
}

void GPSService::bench_inject_fastloc(double lat, double lon, uint32_t hAcc_mm, uint8_t numSV) {
	GNSSData d = {};
	d.fixType = 2;  // 2D / degraded fix
	d.valid = 0x07;
	d.numSV = numSV ? numSV : 4;
	d.lat = lat;
	d.lon = lon;
	d.height = 10000;
	d.hMSL = 10000;
	d.hAcc = hAcc_mm ? hAcc_mm : 50000;  // degraded: 50 m default
	d.vAcc = 60000;
	d.pDOP = 5.0f;
	d.hDOP = 4.0f;
	d.vDOP = 4.0f;
	d.ttff = 5000;
	bench_stamp_time(d);
	DEBUG_WARN("GPSService::bench_inject_fastloc: lat=%f lon=%f hAcc=%umm numSV=%u", lat, lon, (unsigned)d.hAcc,
	           (unsigned)d.numSV);
	m_is_active = false;
	try_enter_deep_idle_or_poweroff();  // R20: cf. bench_inject_fix
	bench_force_initiated();
	gnss_degraded_callback(d);
}

void GPSService::bench_inject_cloudlocate() {
	GNSSRawMeasurement raw;  // ctor zeroes buffers + clears has_* flags
	raw.has_measc12 = true;
	for (unsigned int i = 0; i < sizeof(raw.measc12); i++)
		raw.measc12[i] = (uint8_t)(0xC0 + i);  // deterministic dummy MEASC12 blob
	if (rtc && rtc->is_set()) raw.capture_time = (uint32_t)rtc->gettime();
	DEBUG_WARN("GPSService::bench_inject_cloudlocate: MEASC12 raw measurement injected");
	m_is_active = false;
	try_enter_deep_idle_or_poweroff();  // R20: cf. bench_inject_fix
	bench_force_initiated();
	gnss_cloudlocate_callback(raw);
}

void GPSService::bench_inject_nofix() {
	DEBUG_WARN("GPSService::bench_inject_nofix: forcing NO_FIX end-of-session");
	m_is_active = false;
	try_enter_deep_idle_or_poweroff();  // R20: cf. bench_inject_fix
	bench_force_initiated();
	GPSLogEntry log_entry = invalid_log_entry();
	ServiceEventData event_data = log_entry;
	service_complete(&event_data, &log_entry);
}
#endif

/// @brief Degraded fix callback — store data (does NOT set first_fix_found).
/// @param data  Degraded PVT data (valid but low quality).
void GPSService::gnss_degraded_callback(GNSSData data) {
	m_gnss_data.data = data;
	// Don't set m_is_first_fix_found — degraded fix should not change cold start behavior
	// Safety-net 3.2: M10Q produced a degraded PVT → reset stuck-recovery state.
	m_consecutive_dead_sessions = 0;
	m_wdt_cold_start_tried = false;
	m_stuck_recovery_in_flight = false;
	system_scheduler->cancel_task(m_stuck_recovery_arm_task);
	system_scheduler->cancel_task(m_stuck_recovery_done_task);
	// Safety-net 3.5: GPS produced *some* event — re-arm health WDT.
	// NOT 3.6 — degraded does not count as a real PVT for that watchdog.
	arm_health_wdt();
	task_process_degraded_gnss_data();
}

/// @brief CloudLocate callback — store raw measurement blob for TX.
/// @param data  Raw GNSS measurement snapshot (MEASC12/MEAS20/MEAS50).
void GPSService::gnss_cloudlocate_callback(GNSSRawMeasurement data) {
	m_raw_measurement = data;
	// Don't set m_is_first_fix_found — CloudLocate does not provide on-device position
	// Safety-net 3.2: M10Q produced a raw measurement → reset stuck-recovery state.
	m_consecutive_dead_sessions = 0;
	m_wdt_cold_start_tried = false;
	m_stuck_recovery_in_flight = false;
	system_scheduler->cancel_task(m_stuck_recovery_arm_task);
	system_scheduler->cancel_task(m_stuck_recovery_done_task);
	// Safety-net 3.5: GPS produced *some* event — re-arm health WDT.
	// NOT 3.6 — CloudLocate does not count as a real PVT for that watchdog.
	arm_health_wdt();
	task_process_cloudlocate_data();
}

/// @brief Check if an AXL wakeup event should trigger an immediate GNSS acquisition.
/// @param event       Incoming peer event.
/// @param[out] immediate  Set to true if GNSS should fire immediately.
/// @return true if this event should trigger GNSS rescheduling.
bool GPSService::service_is_triggered_on_event(ServiceEvent &event, bool &immediate) {
#if ENABLE_AXL_SENSOR
	if (event.event_source == ServiceIdentifier::AXL_SENSOR
	    && event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
		// Check if AXL wakeup was triggered by reading the ServiceSensorData
		auto *sensor_data = std::get_if<ServiceSensorData>(&event.event_data);
		if (sensor_data && sensor_data->port[AXLSensorPort::WAKEUP_TRIGGERED]) {
			bool trigger_on_axl = service_read_param<bool>(ParamID::GNSS_TRIGGER_ON_AXL_WAKEUP);
			immediate = trigger_on_axl;
			return trigger_on_axl;
		}
	}
#else
	(void)event;
	(void)immediate;
#endif

	return false;
}

/// @brief Handle peer events — trigger cold start on surfacing if configured.
/// @param e  Peer service event (UW_SENSOR surfacing, etc.).
void GPSService::notify_peer_event(ServiceEvent &e) {
	//DEBUG_TRACE("GPSService::notify_peer_event: (%u|%u)", e.event_source, e.event_type);
	if (e.event_source == ServiceIdentifier::UW_SENSOR && e.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
		bool now_underwater = std::get<bool>(e.event_data);
		m_underwater = now_underwater;
		// Check for surfacing condition
		if (!now_underwater) {
			// Check if we need to trigger a cold start
			if (service_read_param<bool>(ParamID::GNSS_TRIGGER_COLD_START_ON_SURFACED)) {
				DEBUG_TRACE("GPSService: cold start required on surfaced");
				m_is_first_fix_found = false;
				// Request a REAL cold start: wipe the M10Q BBR (navBbrMask=0xFFFF)
				// at the next power-on, not just the cold timeout/constellation
				// path. Without this the receiver keeps stale ephemeris/almanac in
				// BBR and never actually re-acquires from scratch.
				m_force_cold_start = true;
			}
			// 2026-05 deep-idle refactor: the GNSS_BCKP_CHARGE_UW_ONLY abort
			// branch was removed here. With deep-idle, the rail-on window is
			// scoped to a per-session timer (GNSS_DEEP_IDLE_AFTER_OFF_S), not
			// a surface-vs-submerged condition. A manual DTE GNSSBCKP_REQ is
			// the only path that should engage backup-charge while surfaced
			// and the operator explicitly asked for it — don't auto-abort.
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
			// Arm the gate: GNSS power-on will wait until ArgosTxService emits
			// SERVICE_INACTIVE (first satellite TX done) to free CPU for the burst.
			// Safety: only arm if Argos will actually TX — otherwise SERVICE_INACTIVE
			// is never emitted and the gate would block GNSS forever. Cooldown is
			// the second silent path: during cooldown ArgosTxService skips setting
			// m_is_surfacing_burst so no SERVICE_INACTIVE ever fires; the gate
			// would stay armed for the whole surface cycle and starve GPS.
			{
				ArgosConfig argos_config;
				configuration_store->get_argos_configuration(argos_config);
				bool argos_will_tx = ((argos_config.mode != BaseArgosMode::OFF) || argos_config.cert_tx_enable)
				                     && !ServiceManager::is_in_cooldown(service_current_time());
				if (argos_will_tx) {
					m_defer_gnss_until_argos_first_tx = true;
					// Bounded, because the release event may never come — see the
					// note on m_defer_gnss_timeout_task.
					system_scheduler->cancel_task(m_defer_gnss_timeout_task);
					m_defer_gnss_timeout_task = system_scheduler->post_task_prio(
					    [this]() {
						    if (!m_defer_gnss_until_argos_first_tx) return;
						    DEBUG_WARN("GPSService: no Argos TX within %u s of surfacing — lifting the GNSS deferral "
							           "and acquiring. The gate is armed on the TX mode, not on the TX service "
							           "having anything to send.",
							           DEFER_GNSS_MAX_S);
						    m_defer_gnss_until_argos_first_tx = false;
						    if (is_started()) service_reschedule(false);
					    },
					    "GPSDeferGnssTimeout", Scheduler::DEFAULT_PRIORITY, (uint64_t)DEFER_GNSS_MAX_S * MS_PER_SEC);
					// GNSS MED #5 audit fix: also cancel any pending deep-idle
					// auto-off timer. Without this, if GNP52 < surfacing-burst
					// duration, the auto-off fires mid-burst → calls power_off
					// → next acquisition is cold-start (defeats deep-idle).
					// The deep-idle disposition is re-armed at end of the
					// next session via try_enter_deep_idle_or_poweroff.
					system_scheduler->cancel_task(m_deep_idle_auto_off_task);
					m_deep_idle_started_at_ms = 0;
					// Cancel any pending GPS schedule so the gate actually defers
					// the next power_on. WITHOUT this, a UTC-boundary schedule
					// that was queued at boot (gps_service.cpp:157 aligns the
					// next_schedule on the next aq_period UTC boundary, which
					// can be 0-30 s away) fires BEFORE ArgosTx completes the
					// first surfacing-burst TX — exactly the contention the
					// gate is meant to avoid. service_is_triggered_on_surfaced
					// returns false (gate armed) so the base class doesn't
					// re-schedule, but it also doesn't cancel the existing
					// task. Force a reschedule here; service_next_schedule
					// answers a bounded hold while the gate is armed, so the
					// reschedule cancels the pending acquisition without arming
					// a new one. The ArgosTx SERVICE_INACTIVE handler below
					// reschedules properly once TX is done (or has errored out).
					service_reschedule(false);
					DEBUG_INFO("GPSService: surfaced — GNSS deferred until first Argos TX completes");
				}
			}
#endif
		} else {
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
			// Reset gate on submerge so a future surfacing re-arms cleanly.
			m_defer_gnss_until_argos_first_tx = false;
			system_scheduler->cancel_task(m_defer_gnss_timeout_task);
#endif
		}
	}

#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
	// Release the gate when Argos TX completes (any SERVICE_INACTIVE from ARGOS_TX),
	// and manually trigger the deferred GNSS reschedule.
	if (e.event_source == ServiceIdentifier::ARGOS_TX && e.event_type == ServiceEventType::SERVICE_INACTIVE
	    && m_defer_gnss_until_argos_first_tx && !m_underwater) {
		m_defer_gnss_until_argos_first_tx = false;
		system_scheduler->cancel_task(m_defer_gnss_timeout_task);
		DEBUG_INFO("GPSService: first Argos TX done — releasing GNSS gate, rescheduling now");
		Service::notify_peer_event(e);  // let base process the event normally
		bool immediate;
		if (service_is_triggered_on_surfaced(immediate)) {
			service_reschedule(immediate);
		}
		return;
	}
#endif

	Service::notify_peer_event(e);
}

/// @brief Public entry-point: start/stop a V_BCKP coin-cell charge session.
bool GPSService::request_backup_charge(unsigned int duration_s) {
	if (duration_s == 0) {
		DEBUG_INFO("GPSService::request_backup_charge: stop requested");
		backup_charge_stop_internal();
		return true;
	}

	// Refuse if a GNSS acquisition is currently in progress — backup charge
	// and active acquisition are mutually exclusive (M10 device state machine).
	if (m_is_active) {
		DEBUG_WARN("GPSService::request_backup_charge: GNSS acquisition active — refused");
		return false;
	}

	// Already active: just refresh the auto-exit timer (extend / shorten).
	if (m_backup_active) {
		system_scheduler->cancel_task(m_backup_exit_task);
		m_backup_exit_task = system_scheduler->post_task_prio(
		    [this]() {
			    DEBUG_INFO("GPSService: backup-charge auto-exit (duration elapsed)");
			    backup_charge_stop_internal();
		    },
		    "GPSBackupChargeExit", Scheduler::DEFAULT_PRIORITY, duration_s * MS_PER_SEC);
		return true;
	}

	if (m_device.enter_backup_charge_mode()) {
		// Hardware happy path: M10 was in idle, switched to enterbackup straight away.
		m_backup_active = true;
		DEBUG_INFO("GPSService::request_backup_charge: starting (duration=%us)", duration_s);
		if (m_on_backup_charge_start) m_on_backup_charge_start();
		m_backup_exit_task = system_scheduler->post_task_prio(
		    [this]() {
			    DEBUG_INFO("GPSService: backup-charge auto-exit (duration elapsed)");
			    backup_charge_stop_internal();
		    },
		    "GPSBackupChargeExit", Scheduler::DEFAULT_PRIORITY, duration_s * MS_PER_SEC);
		return true;
	}

	// Hardware refused. Two different recovery strategies depending on the caller:
	//
	// 1. DTE path (user sent GNSSBCKP via BLE — m_on_backup_charge_start is set by
	//    ConfigurationState): the user expects the device to enter silent charging mode
	//    NOW. We fire the start callback immediately (BLE will cut after 200 ms) and
	//    poll enter_backup_charge_mode every 250 ms for up to 5 s. If still failing
	//    after that, abort cleanly via the stop callback (transit OperationalState).
	//
	// 2. Periodic algorithm path (backup_charge_periodic_fire — no callbacks set):
	//    just fail. The next periodic tick will try again. We MUST NOT call power_off
	//    here because the GNSS service may still be cycling, and we'd disrupt the
	//    acquisition schedule.
	if (!m_on_backup_charge_start) {
		DEBUG_INFO("GPSService::request_backup_charge: no DTE callbacks set, hardware busy — request skipped");
		return false;
	}

	DEBUG_INFO(
	    "GPSService::request_backup_charge: DTE path, hardware busy — start charge in 'pending' mode (duration=%us)",
	    duration_s);
	m_pending_backup_duration_s = duration_s;
	m_on_backup_charge_start();  // BLE cuts now even though charge hasn't physically started
	schedule_backup_charge_retry(0);
	return true;
}

/// @brief Poll enter_backup_charge_mode every 250 ms until success or attempts exhausted.
/// Called only from the DTE path of request_backup_charge. Aborts cleanly via the stop
/// callback after MAX_ATTEMPTS (~5 s), ensuring the device never freezes — the FSM
/// transits back to OperationalState which restarts services and BLE.
void GPSService::schedule_backup_charge_retry(unsigned int attempt) {
	static constexpr unsigned int MAX_ATTEMPTS = 20;  // 20 * 250 ms = 5 s
	static constexpr unsigned int RETRY_DELAY_MS = 250;

	if (attempt >= MAX_ATTEMPTS) {
		DEBUG_ERROR("GPSService::backup_charge_retry: exhausted %u attempts — abandoning", attempt);
		m_pending_backup_duration_s = 0;
		auto cb = std::move(m_on_backup_charge_stop);
		m_on_backup_charge_start = nullptr;
		m_on_backup_charge_stop = nullptr;
		if (cb) cb();
		return;
	}

	m_backup_retry_task = system_scheduler->post_task_prio(
	    [this, attempt]() {
		    if (m_pending_backup_duration_s == 0) return;  // cancelled (manual stop / reed switch / etc.)
		    unsigned int d = m_pending_backup_duration_s;
		    if (m_device.enter_backup_charge_mode()) {
			    m_pending_backup_duration_s = 0;
			    m_backup_active = true;
			    DEBUG_INFO("GPSService::backup_charge_retry: started after %u attempts (duration=%us)", attempt, d);
			    m_backup_exit_task = system_scheduler->post_task_prio(
			        [this]() {
				        DEBUG_INFO("GPSService: backup-charge auto-exit (duration elapsed)");
				        backup_charge_stop_internal();
			        },
			        "GPSBackupChargeExit", Scheduler::DEFAULT_PRIORITY, d * MS_PER_SEC);
		    } else {
			    schedule_backup_charge_retry(attempt + 1);
		    }
	    },
	    "BackupChargeRetry", Scheduler::DEFAULT_PRIORITY, RETRY_DELAY_MS);
}

/// @brief Internal: stop the charge session if active. Idempotent.
/// Handles three states: active charge (exit_backup_charge_mode), pending retry
/// (cancel safety timer), or neither (no-op apart from clearing callbacks).
void GPSService::backup_charge_stop_internal() {
	system_scheduler->cancel_task(m_backup_exit_task);
	system_scheduler->cancel_task(m_backup_retry_task);
	m_pending_backup_duration_s = 0;
	if (m_backup_active) {
		m_device.exit_backup_charge_mode();
		m_backup_active = false;
	}
	// Fire and clear stop callback (cleared before calling to be re-entrant safe).
	auto cb = std::move(m_on_backup_charge_stop);
	m_on_backup_charge_start = nullptr;
	m_on_backup_charge_stop = nullptr;
	if (cb) cb();
}

void GPSService::set_backup_charge_callbacks(std::function<void()> on_start, std::function<void()> on_stop) {
	m_on_backup_charge_start = std::move(on_start);
	m_on_backup_charge_stop = std::move(on_stop);
}

// 2026-05 deep-idle refactor: the periodic backup-charge scheduler is gone.
// Recharging the V_BCKP coin cell now happens implicitly during the deep-idle
// window after each GPS session (configured via GNSS_DEEP_IDLE_AFTER_OFF_S).
// No more dedicated `backup_charge_schedule_next` / `backup_charge_periodic_fire`
// — those functions used the removed GNSS_BCKP_CHARGE_{INT,DUR,UW_ONLY} params.
// The DTE GNSSBCKP_REQ command (request_backup_charge) is still available for
// manual operator-triggered charge sessions and now delegates to the deep-idle
// driver path internally.
