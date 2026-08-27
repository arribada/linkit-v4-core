/**
 * @file ledsm.cpp
 * @brief LED state machine — entry handlers for each tracker state → RGB color/flash pattern.
 */

#include "ledsm.hpp"
#include "debug.hpp"
#include "config_store.hpp"
#include "rgb_led.hpp"
#include "rtc.hpp"

extern RGBLed *status_led;
extern Timer *system_timer;
extern ConfigurationStore *configuration_store;
extern RTC *rtc;

// GNSS MED #3 audit fix: alias used by the deferred LEDGNSSOffWithoutFix
// re-dispatch (lambda inside system_timer->add_schedule). Same pattern as
// in gentracker.cpp / ota_flash_file_updater.cpp.
using led_handle = LEDState;

namespace {

constexpr unsigned long long LED_HRS_24_MS = 24ULL * 3600ULL * 1000ULL;

// === LED freeze safety (2026-05) ===========================================
// Defense against a stuck FSM (e.g. an Argos TX that never returns its
// completion event) leaving the LED solid for hours. Each non-exempt state
// entry() arms a deadline; the next state entry that runs disarms / re-arms
// it. If the deadline expires the safety fires and forces the LED off — purely
// visual, the FSM stays in its current state. EXEMPT (disarmed explicitly):
// config modes (BLE GUI connected), battery-critical warning, OTA success,
// all pre-operational states — all "intentionally solid" states the operator
// needs to keep seeing.
//
// Timeout history:
//   - Initial: 5 s (too short, fires during normal SMD Argos TX).
//   - Bumped to 10 s to cover full SMD Argos TX (TCXO 5 s + RF ≈ 3 s + margin).
//   - 2026-05-23 (first bump): 130 s after field log showed it firing 3×
//     during a 5-min operational session. Root cause: LEDGNSSOn legitimately
//     stays armed for the full GPS acquisition session, which can reach
//     GNSS_COLD_ACQ_TIMEOUT (default 120 s). 130 s = 120 s cold acq + 10 s
//     margin. Sufficient for the default config but not for users running
//     the param at its allowed maximum.
//   - 2026-05-23 (second bump, post-audit): 650 s to cover GNSS_COLD_ACQ_
//     TIMEOUT at its full configurable range (10-600 s per dte_params.cpp).
//     A genuinely-frozen LED FSM still recovers within 10.8 min, well below
//     the 15-min system watchdog. The safety is a catastrophic-freeze net,
//     not a tight UX timer.
//
// Future option: per-state timeouts (LEDArgosTX=10 s tight, LEDGNSSOn=650 s
// generous, LEDDFUUpdate=120 s). Not implemented — keeping the simple
// global cap is preferable to per-state tuning unless a real freeze case
// shows up that needs the tighter recovery.
static constexpr uint64_t LED_FREEZE_TIMEOUT_MS = 650000;
static Timer::TimerHandle s_led_freeze_handle;

static void arm_led_freeze_safety() {
	if (!system_timer || !status_led) return;
	if (s_led_freeze_handle.has_value())
		system_timer->cancel_schedule(s_led_freeze_handle);
	s_led_freeze_handle = system_timer->add_schedule([]() {
		DEBUG_WARN("LED freeze safety: %llu s without FSM transition — forcing LED off",
		           (unsigned long long)(LED_FREEZE_TIMEOUT_MS / 1000));
		if (status_led) status_led->off();
		s_led_freeze_handle.reset();
	}, system_timer->get_counter() + LED_FREEZE_TIMEOUT_MS);
}

static void disarm_led_freeze_safety() {
	if (!system_timer) return;
	if (s_led_freeze_handle.has_value()) {
		system_timer->cancel_schedule(s_led_freeze_handle);
		s_led_freeze_handle.reset();
	}
}

// Cert/calibration mode override: the operator-visible MAGENTA LED on every
// TX must be visible regardless of LED_MODE (e.g. LED_MODE=OFF for a battery
// audit run). LED_MODE_GUARD applies during deployment, not during a manual
// cert test the operator is actively watching.
static bool cert_tx_active() {
	if (!configuration_store) return false;
	try {
		return configuration_store->read_param<bool>(ParamID::CERT_TX_ENABLE);
	} catch (...) {
		return false;
	}
}

/// @name LED signalling for a BLIND burst
/// @{
/// In BLIND the burst is paced by the module itself and lasts minutes
/// (measured: 123 s for retx_nb=3, period=60). A steady MAGENTA would stay lit
/// for the whole of it -- for nothing, and against the very battery life we are
/// trying to gain by letting the module sleep. Mark the two EDGES of the
/// sequence instead: two short pulses when it opens, one long pulse when it
/// closes, visually distinct from each other.
static constexpr unsigned int BLIND_BLINK_ON_MS  = 120;  ///< length of one opening pulse
static constexpr unsigned int BLIND_BLINK_GAP_MS = 150;  ///< silence between the two
static constexpr unsigned int BLIND_BLINK_END_MS = 700;  ///< closing pulse

/// @brief Is the BLIND MAC profile the one that will actually transmit?
///
/// Query the EFFECTIVE configuration, not the raw parameter: the effective one
/// accounts for the regimes (low battery, out of zone, hauled out) and clears
/// blind_en for SURFACING_BURST and DOPPLER, which pace their burst on the nRF
/// side. Without this the LED would announce a module burst where there is none.
static bool blind_tx_active() {
	if (!configuration_store) return false;
	try {
		ArgosConfig cfg;
		configuration_store->get_argos_configuration(cfg);
		return cfg.blind_en;
	} catch (...) {
		return false;
	}
}

/// @brief Two short pulses -- the burst starts, the module takes over.
static void blind_blink_sequence_start() {
	status_led->set(RGBLedColor::MAGENTA);
	system_timer->add_schedule([]() {
		status_led->off();
		system_timer->add_schedule([]() {
			status_led->set(RGBLedColor::MAGENTA);
			system_timer->add_schedule([]() {
				status_led->off();
			}, system_timer->get_counter() + BLIND_BLINK_ON_MS);
		}, system_timer->get_counter() + BLIND_BLINK_GAP_MS);
	}, system_timer->get_counter() + BLIND_BLINK_ON_MS);
}
/// @}

/// @brief Returns true while we are still inside the 24-hour LED window.
///
/// On regular boards (single boot, no hard power-cycle), `system_timer` uptime
/// is monotonic and accurately tracks the time since deployment.
///
/// On EXTERNAL_WAKEUP boards (TPL5111-driven hard shutdowns), `system_timer`
/// resets to 0 at every wake-up — so the uptime check would never expire and
/// LEDs would stay on for the entire mission, draining the battery.
/// We use the RTC cutoff stored in `LED_HRS24_RTC_CUTOFF` — set once by
/// GPSService to (first valid GNSS RTC + 24h). RTC survives TPL hard shutdowns
/// via `LAST_KNOWN_RTC` (PWP06) restored at every boot, so this gives a real
/// 24-hours-since-deployment window.
///
/// Fallback (EXTERNAL_WAKEUP, no GNSS fix yet OR RTC unknown): keep LEDs on.
/// Better visible-on-bench during early provisioning than silent-on-deployment.
static bool led_24h_window_active() {
#ifdef EXTERNAL_WAKEUP
	if (rtc && rtc->is_set()) {
		// Same reasoning as led_mode_allows_light(): reached from the RTC ISR,
		// and read_param() always rethrows on a bad read.
		std::time_t led_cutoff;
		try {
			led_cutoff = configuration_store->read_param<std::time_t>(ParamID::LED_HRS24_RTC_CUTOFF);
		} catch (...) {
			return false;
		}
		if (led_cutoff != 0) {
			return rtc->gettime() < led_cutoff;
		}
	}
	return true;  // No anchor yet — keep LEDs visible during provisioning
#else
	return system_timer->get_counter() < LED_HRS_24_MS;
#endif
}

}  // namespace


/// @brief Are the LEDs allowed to light right now?
///
/// MUST NOT THROW. Every LED state's entry() can reach this, and those entry()
/// handlers run in the RTC INTERRUPT: ledsm.cpp defers its blink and transition
/// steps through system_timer->add_schedule(), and nrf_timer.cpp runs those
/// callbacks straight from rtc_event_handler ("Schedule callbacks execute in
/// this ISR context - keep them short"). ConfigurationStore::read_param() ends
/// in `catch (...) { throw CONFIG_STORE_CORRUPTED; }`, so it ALWAYS rethrows on
/// a bad read -- and a C++ throw crossing an interrupt frame reaches
/// std::terminate, the same way the BMA400 callbacks did through the Bosch C
/// library. Turning a recoverable config problem into a dead sealed device, for
/// the sake of repainting an LED, is not a trade worth making: swallow it and
/// keep the LEDs dark instead.
static bool led_mode_allows_light() {
	try {
		BaseLEDMode mode = configuration_store->read_param<BaseLEDMode>(ParamID::LED_MODE);
		if (mode == BaseLEDMode::ALWAYS)
			return true;
		if (mode == BaseLEDMode::HRS_24)
			return led_24h_window_active();
		return false;
	} catch (...) {
		return false;
	}
}

// Macro for determining LED mode state
#define LED_MODE_GUARD if (led_mode_allows_light())


void LEDOff::entry() {
	DEBUG_TRACE("LEDOff: entry");
	// Disarm: WHITE-solid while magnet engaged must stay visible to operator
	// (no time bound — operator releases magnet when ready). For BLACK (off),
	// the disarm is harmless. We also accept that a magnet held silently for
	// >5 s could be cleaned by the safety, but that path is uncommon and the
	// operator UX cost of having the LED disappear is higher.
	disarm_led_freeze_safety();
	if (m_is_magnet_engaged)
		status_led->set(RGBLedColor::WHITE);
	else if (m_is_battery_critical)
		transit<LEDBatteryCritical>();
	else
		status_led->off();
}

void LEDBoot::entry() {
	DEBUG_TRACE("LEDBoot: entry");
	arm_led_freeze_safety();
	status_led->flash(RGBLedColor::WHITE, 125);
}

void LEDPowerDown::entry() {
	DEBUG_TRACE("LEDPowerDown: entry");
	arm_led_freeze_safety();
	m_is_battery_critical = false;
	// Always flash white during powerdown countdown, regardless of magnet
	status_led->flash(RGBLedColor::WHITE, 50);
}

void LEDError::entry() {
	DEBUG_TRACE("LEDError: entry");
	arm_led_freeze_safety();
	m_is_battery_critical = false;
	if (m_is_magnet_engaged)
		status_led->set(RGBLedColor::WHITE);
	else
		status_led->flash(RGBLedColor::RED);
}

void LEDPreOperationalPending::entry() {
	DEBUG_TRACE("LEDPreOperationalPending: entry");
	// Pre-operational = magnet activation UX. Operator must see GREEN until
	// they release the magnet. Exempted from the 5 s freeze safety per the
	// "operational mode only" scope of the user spec.
	disarm_led_freeze_safety();
	m_is_battery_critical = false;
	status_led->set(RGBLedColor::GREEN);
}

void LEDPreOperationalError::entry() {
	DEBUG_TRACE("LEDPreOperationalError: entry");
	disarm_led_freeze_safety();  // Pre-operational — exempt per spec
	m_is_battery_critical = false;
	if (m_is_magnet_engaged)
		status_led->set(RGBLedColor::WHITE);
	else
		status_led->flash(RGBLedColor::RED);
}

void LEDPreOperationalBatteryNominal::entry() {
	DEBUG_TRACE("LEDPreOperationalBatteryNominal: entry");
	disarm_led_freeze_safety();  // Pre-operational — exempt per spec
	m_is_battery_critical = false;
	if (m_is_magnet_engaged)
		status_led->set(RGBLedColor::WHITE);
	else
		status_led->flash(RGBLedColor::GREEN);
}

void LEDPreOperationalBatteryLow::entry() {
	DEBUG_TRACE("LEDPreOperationalBatteryLow: entry");
	disarm_led_freeze_safety();  // Pre-operational — exempt per spec
	m_is_battery_critical = false;
	if (m_is_magnet_engaged)
		status_led->set(RGBLedColor::WHITE);
	else
		status_led->flash(RGBLedColor::YELLOW);
}

void LEDConfigPending::entry() {
	DEBUG_TRACE("LEDConfigPending: entry");
	disarm_led_freeze_safety();  // BLE GUI connect path — exempt per spec
	m_is_battery_critical = false;
	status_led->set(RGBLedColor::BLUE);
}

void LEDConfigNotConnected::entry() {
	DEBUG_TRACE("LEDConfigNotConnected: entry");
	disarm_led_freeze_safety();  // Config mode — exempt per spec
	m_is_battery_critical = false;
	if (m_is_magnet_engaged)
		status_led->set(RGBLedColor::WHITE);
	else
		status_led->flash(RGBLedColor::BLUE);
}

void LEDConfigConnected::entry() {
	DEBUG_TRACE("LEDConfigConnected: entry");
	disarm_led_freeze_safety();  // BLE GUI connected — exempt per spec (user request)
	m_is_battery_critical = false;
	if (m_is_magnet_engaged)
		status_led->set(RGBLedColor::WHITE);
	else
		status_led->set(RGBLedColor::BLUE);
}

void LEDGNSSOn::entry() {
	DEBUG_TRACE("LEDGNSSOn: entry");
	arm_led_freeze_safety();
	m_is_gnss_on = true;
	if (m_is_magnet_engaged)
		status_led->set(RGBLedColor::WHITE);
	else {
		LED_MODE_GUARD {
			status_led->flash(RGBLedColor::CYAN, 1000);
		} else {
			status_led->off();
		}
	}
}

void LEDGNSSOffWithFix::entry() {
	DEBUG_TRACE("LEDGNSSOffWithFix: entry");
	arm_led_freeze_safety();
	if (m_is_magnet_engaged)
		status_led->set(RGBLedColor::WHITE);
	else {
		if (m_is_gnss_on) {
			LED_MODE_GUARD {
				status_led->set(RGBLedColor::GREEN);
			} else {
				status_led->off();
			}
		}
		system_timer->add_schedule([this]() {
			// Has the state changed since this was armed? ledsm.cpp defines NO
			// ::exit() anywhere, so a deferred transit is never cancelled: it fires no
			// matter what, including when the FSM has moved to another state in the
			// meantime. On the nominal deployment sequence the Argos transmission
			// follows the fix immediately, so the orphan left by the end of the GNSS
			// session was switching off the transmit indication. Check here rather
			// than weigh down the lifecycle of 28 states.
			if (!is_in_state<LEDGNSSOffWithFix>()) return;
			if (is_in_state<LEDConfigNotConnected>())
				transit<LEDConfigNotConnected>();
			else
				transit<LEDOff>();
		}, system_timer->get_counter() + 3000);
	}
	m_is_gnss_on = false;
}

void LEDGNSSOffWithoutFix::entry() {
	DEBUG_TRACE("LEDGNSSOffWithoutFix: entry");
	arm_led_freeze_safety();
	// 2026-05-24 LED refactor: this state remains as a LEGACY fallback only —
	// no-fix sessions now route through LEDGNSSDeepIdle / LEDGNSSPowerOff (set
	// by GPSService::notify_service_event with GNSS_OFF_DEEP_IDLE or
	// GNSS_OFF_POWEROFF). Path is kept compiling for any caller still
	// dispatching SetLEDGNSSOffWithoutFix directly, with duration halved
	// from 3000 ms → 1500 ms per operator feedback "current red too long".
	if (is_in_state<LEDGNSSCloudLocateReady>()) {
		DEBUG_TRACE("LEDGNSSOffWithoutFix: deferring 500ms — CloudLocateReady CYAN double-blink in progress");
		system_timer->add_schedule([]() {
			led_handle::dispatch<SetLEDGNSSOffWithoutFix>({});
		}, system_timer->get_counter() + 500);
		return;
	}
	if (m_is_magnet_engaged)
		status_led->set(RGBLedColor::WHITE);
	else {
		if (m_is_gnss_on) {
			LED_MODE_GUARD {
				status_led->set(RGBLedColor::RED);
			} else {
				status_led->off();
			}
		}
		system_timer->add_schedule([this]() {
			// Has the state changed since this was armed? ledsm.cpp defines NO
			// ::exit() anywhere, so a deferred transit is never cancelled: it fires no
			// matter what, including when the FSM has moved to another state in the
			// meantime. On the nominal deployment sequence the Argos transmission
			// follows the fix immediately, so the orphan left by the end of the GNSS
			// session was switching off the transmit indication. Check here rather
			// than weigh down the lifecycle of 28 states.
			if (!is_in_state<LEDGNSSOffWithoutFix>()) return;
			if (is_in_state<LEDConfigNotConnected>())
				transit<LEDConfigNotConnected>();
			else
				transit<LEDOff>();
		}, system_timer->get_counter() + 1500);
	}
	m_is_gnss_on = false;
}

// 2026-05-24: end-of-session deep-idle indicator (rail stays on, M10Q in
// PMREQ-backup). Double-blink for ~500 ms then auto-transit to LEDOff.
// Color picks from latched m_last_gnss_fix_valid:
//   fix → GREEN (mirrors legacy LEDGNSSOffWithFix semantics)
//   no fix → RED (mirrors legacy LEDGNSSOffWithoutFix, shortened from 3 s)
// Mirrors LEDGNSSCloudLocateReady's double-blink pattern (120 ms alternate)
// for visual consistency — operator learns "double-blink = transient state
// transition". Replaces legacy LEDGNSSOff{With,Without}Fix solid 3 s flashes
// on the deep-idle dispatch path.
void LEDGNSSDeepIdle::entry() {
	DEBUG_TRACE("LEDGNSSDeepIdle: entry (fix=%d)", m_last_gnss_fix_valid ? 1 : 0);
	arm_led_freeze_safety();
	if (m_is_magnet_engaged) {
		status_led->set(RGBLedColor::WHITE);
	} else {
		LED_MODE_GUARD {
			RGBLedColor color = m_last_gnss_fix_valid ? RGBLedColor::GREEN
			                                          : RGBLedColor::RED;
			status_led->flash_alternate(color, RGBLedColor::BLACK, 120);
		} else {
			status_led->off();
		}
	}
	system_timer->add_schedule([this]() {
		// Has the state changed since this was armed? ledsm.cpp defines NO
		// ::exit() anywhere, so a deferred transit is never cancelled: it fires no
		// matter what, including when the FSM has moved to another state in the
		// meantime. On the nominal deployment sequence the Argos transmission
		// follows the fix immediately, so the orphan left by the end of the GNSS
		// session was switching off the transmit indication. Check here rather
		// than weigh down the lifecycle of 28 states.
		if (!is_in_state<LEDGNSSDeepIdle>()) return;
		if (is_in_state<LEDConfigNotConnected>())
			transit<LEDConfigNotConnected>();
		else
			transit<LEDOff>();
	}, system_timer->get_counter() + 500);
	m_is_gnss_on = false;
}

// 2026-05-24: end-of-session full power-off indicator (rail cut, M10Q cold-
// boot next session). Fast-blink for ~500 ms (50 ms period = 10 Hz cadence)
// — heavier-weight than the deep-idle double-blink so the operator can
// distinguish the two sleep depths at a glance. Same fix/no-fix color logic:
//   fix → GREEN fast-blink
//   no fix → RED fast-blink
void LEDGNSSPowerOff::entry() {
	DEBUG_TRACE("LEDGNSSPowerOff: entry (fix=%d)", m_last_gnss_fix_valid ? 1 : 0);
	arm_led_freeze_safety();
	if (m_is_magnet_engaged) {
		status_led->set(RGBLedColor::WHITE);
	} else {
		LED_MODE_GUARD {
			RGBLedColor color = m_last_gnss_fix_valid ? RGBLedColor::GREEN
			                                          : RGBLedColor::RED;
			status_led->flash(color, 50);
		} else {
			status_led->off();
		}
	}
	system_timer->add_schedule([this]() {
		// Has the state changed since this was armed? ledsm.cpp defines NO
		// ::exit() anywhere, so a deferred transit is never cancelled: it fires no
		// matter what, including when the FSM has moved to another state in the
		// meantime. On the nominal deployment sequence the Argos transmission
		// follows the fix immediately, so the orphan left by the end of the GNSS
		// session was switching off the transmit indication. Check here rather
		// than weigh down the lifecycle of 28 states.
		if (!is_in_state<LEDGNSSPowerOff>()) return;
		if (is_in_state<LEDConfigNotConnected>())
			transit<LEDConfigNotConnected>();
		else
			transit<LEDOff>();
	}, system_timer->get_counter() + 500);
	m_is_gnss_on = false;
}

// 2026-05 deep-idle refactor FAST3c: visual marker for "raw CloudLocate
// measurement ready". Double-blink CYAN (~120 ms on, 80 ms off, repeat —
// total ~400 ms) distinguishes this from the steady CYAN flash of
// LEDGNSSOn. After the double-blink, return to LEDGNSSOn (GPS still
// active waiting for full PVT) or LEDOff (CLOUDLOCATE_ONLY just ended
// the session). LED_MODE_GUARD respected — if the user disabled the LED
// for deployment, the entry call is a no-op.
void LEDGNSSCloudLocateReady::entry() {
	DEBUG_TRACE("LEDGNSSCloudLocateReady: entry");
	arm_led_freeze_safety();
	if (m_is_magnet_engaged) {
		status_led->set(RGBLedColor::WHITE);
	} else {
		LED_MODE_GUARD {
			// flash_alternate(on_color, off_color, period_ms) produces a
			// repeating on→off pattern. With 120 ms on / off period, two
			// quick blinks span ~400 ms before the transit_back fires at
			// 500 ms. Falls through to LEDGNSSOn / LEDOff cleanly.
			status_led->flash_alternate(RGBLedColor::CYAN, RGBLedColor::BLACK, 120);
		} else {
			status_led->off();
		}
	}
	// Schedule transit back. If GPS is still active (CLOUDLOCATE_ONLY=false),
	// resume LEDGNSSOn so the operator sees the GPS is still acquiring.
	// Otherwise go to LEDOff (the GNSS session ended).
	system_timer->add_schedule([this]() {
		// Has the state changed since this was armed? ledsm.cpp defines NO
		// ::exit() anywhere, so a deferred transit is never cancelled: it fires no
		// matter what, including when the FSM has moved to another state in the
		// meantime. On the nominal deployment sequence the Argos transmission
		// follows the fix immediately, so the orphan left by the end of the GNSS
		// session was switching off the transmit indication. Check here rather
		// than weigh down the lifecycle of 28 states.
		if (!is_in_state<LEDGNSSCloudLocateReady>()) return;
		if (m_is_gnss_on)
			transit<LEDGNSSOn>();
		else if (is_in_state<LEDConfigNotConnected>())
			transit<LEDConfigNotConnected>();
		else
			transit<LEDOff>();
	}, system_timer->get_counter() + 500);
}

void LEDArgosTX::entry() {
	DEBUG_TRACE("LEDArgosTX: entry");
	arm_led_freeze_safety();
	if (m_is_magnet_engaged)
		status_led->set(RGBLedColor::WHITE);
	else if (cert_tx_active()) {
		// Doppler / cert calibration: operator is watching this run actively;
		// MAGENTA per TX regardless of LED_MODE so the visual cue is always there.
		status_led->set(RGBLedColor::MAGENTA);
	}
	else {
		LED_MODE_GUARD {
			if (blind_tx_active())
				blind_blink_sequence_start();
			else
				status_led->set(RGBLedColor::MAGENTA);
		} else {
			status_led->off();
		}
	}
}

void LEDArgosTXComplete::entry() {
	DEBUG_TRACE("LEDArgosTXComplete: entry");
	arm_led_freeze_safety();
	if (m_is_magnet_engaged)
		status_led->set(RGBLedColor::WHITE);
	else {
		// In BLIND, mark the close with a long pulse before handing back -- that
		// is what tells it apart from the two short opening pulses. The exit
		// delay has to cover the pulse, otherwise the state transition would
		// switch it off before it is visible.
		unsigned int hold_ms = 50;  // anti-race margin on the tick, nominal case
		bool blind = blind_tx_active();
		if (blind) {
			LED_MODE_GUARD {
				status_led->set(RGBLedColor::MAGENTA);
				hold_ms = BLIND_BLINK_END_MS;
			} else {
				status_led->off();
			}
		} else {
			status_led->off();
		}
		system_timer->add_schedule([this]() {
			// Has the state changed since this was armed? ledsm.cpp defines NO
			// ::exit() anywhere, so a deferred transit is never cancelled: it fires no
			// matter what, including when the FSM has moved to another state in the
			// meantime. On the nominal deployment sequence the Argos transmission
			// follows the fix immediately, so the orphan left by the end of the GNSS
			// session was switching off the transmit indication. Check here rather
			// than weigh down the lifecycle of 28 states.
			if (!is_in_state<LEDArgosTXComplete>()) return;
			status_led->off();
			if (m_is_gnss_on)
				transit<LEDGNSSOn>();
			else {
				if (is_in_state<LEDConfigNotConnected>())
					transit<LEDConfigNotConnected>();
				else
					transit<LEDOff>();
			}
		}, system_timer->get_counter() + hold_ms);
	}
}

void LEDBatteryCritical::entry() {
	DEBUG_TRACE("LEDBatteryCritical: entry");
	disarm_led_freeze_safety();  // Terminal warning — must stay visible
	m_is_battery_critical = true;
	if (m_is_magnet_engaged)
		status_led->set(RGBLedColor::WHITE);
	else {
		status_led->set(RGBLedColor::YELLOW);
	}
}

void LEDDFUUpdate::entry() {
	DEBUG_TRACE("LEDDFUUpdate: entry");
	arm_led_freeze_safety();
	status_led->flash_alternate(RGBLedColor::BLUE, RGBLedColor::WHITE, 250);
}

void LEDOTASuccess::entry() {
	DEBUG_TRACE("LEDOTASuccess: entry");
	disarm_led_freeze_safety();  // Terminal post-update state — must stay visible
	status_led->set(RGBLedColor::GREEN);
}

void LEDOTAFailed::entry() {
	DEBUG_TRACE("LEDOTAFailed: entry");
	arm_led_freeze_safety();
	status_led->flash(RGBLedColor::RED, 200);
}

void LEDFirmwareApplied::entry() {
	DEBUG_TRACE("LEDFirmwareApplied: entry");
	arm_led_freeze_safety();
	status_led->flash(RGBLedColor::GREEN, 150);
}

void LEDConfirmConfig::entry() {
	DEBUG_TRACE("LEDConfirmConfig: entry");
	arm_led_freeze_safety();
	status_led->flash(RGBLedColor::BLUE, 50);
}

void LEDConfirmExitConfig::entry() {
	DEBUG_TRACE("LEDConfirmExitConfig: entry");
	arm_led_freeze_safety();
	status_led->flash(RGBLedColor::GREEN, 50);
}

void LEDConfirmPowerOff::entry() {
	DEBUG_TRACE("LEDConfirmPowerOff: entry");
	arm_led_freeze_safety();
	status_led->flash(RGBLedColor::RED, 50);
}

void LEDSurfaceDetected::entry() {
	DEBUG_TRACE("LEDSurfaceDetected: entry");
	arm_led_freeze_safety();
	LED_MODE_GUARD {
		status_led->set(RGBLedColor::GREEN);
	} else {
		status_led->off();
	}
	system_timer->add_schedule([this]() {
		// Has the state changed since this was armed? ledsm.cpp defines NO
		// ::exit() anywhere, so a deferred transit is never cancelled: it fires no
		// matter what, including when the FSM has moved to another state in the
		// meantime. On the nominal deployment sequence the Argos transmission
		// follows the fix immediately, so the orphan left by the end of the GNSS
		// session was switching off the transmit indication. Check here rather
		// than weigh down the lifecycle of 28 states.
		if (!is_in_state<LEDSurfaceDetected>()) return;
		transit<LEDOff>();
	}, system_timer->get_counter() + 100);
}

void LEDDiveDetected::entry() {
	DEBUG_TRACE("LEDDiveDetected: entry");
	arm_led_freeze_safety();
	LED_MODE_GUARD {
		status_led->set(RGBLedColor::BLUE);
	} else {
		status_led->off();
	}
	system_timer->add_schedule([this]() {
		// Has the state changed since this was armed? ledsm.cpp defines NO
		// ::exit() anywhere, so a deferred transit is never cancelled: it fires no
		// matter what, including when the FSM has moved to another state in the
		// meantime. On the nominal deployment sequence the Argos transmission
		// follows the fix immediately, so the orphan left by the end of the GNSS
		// session was switching off the transmit indication. Check here rather
		// than weigh down the lifecycle of 28 states.
		if (!is_in_state<LEDDiveDetected>()) return;
		transit<LEDOff>();
	}, system_timer->get_counter() + 100);
}
