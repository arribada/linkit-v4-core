/**
 * @file ledsm.hpp
 * @brief LED state machine — maps tracker states to RGB LED patterns (solid, flash, alternate).
 */

#pragma once

#include "tinyfsm.hpp"
#include "timer.hpp"

/// @brief True while a reed-switch confirmation gesture is pending (fast
/// blue/red/green blink awaiting the operator's 2nd gesture). Defined in
/// gentracker.cpp; forwards to GenTracker::is_confirmation_gesture_pending().
/// While true, the LED FSM gates out transient/background LED events so the
/// confirmation prompt is not interrupted by the boot white, the magnet-engaged
/// white, or a GNSS/Argos/dive/surface flash. Declared here (not via a
/// GenTracker include) to keep the LED FSM decoupled from the main FSM.
bool led_confirmation_gesture_pending();

/// @name LED state events (dispatched by GenTracker FSM)
/// @{
struct SetLEDOff : tinyfsm::Event { };
struct SetLEDMagnetEngaged : tinyfsm::Event { };
struct SetLEDMagnetDisengaged : tinyfsm::Event { };
struct SetLEDBoot : tinyfsm::Event { };
struct SetLEDPowerDown : tinyfsm::Event { };
struct SetLEDError : tinyfsm::Event { };
struct SetLEDPreOperationalPending : tinyfsm::Event { };
struct SetLEDPreOperationalError : tinyfsm::Event { };
struct SetLEDPreOperationalBatteryNominal : tinyfsm::Event { };
struct SetLEDPreOperationalBatteryLow : tinyfsm::Event { };
struct SetLEDConfigPending : tinyfsm::Event { };
struct SetLEDConfigNotConnected : tinyfsm::Event { };
struct SetLEDConfigConnected : tinyfsm::Event { };
struct SetLEDGNSSOn : tinyfsm::Event { };
struct SetLEDGNSSOffWithFix : tinyfsm::Event { };
struct SetLEDGNSSOffWithoutFix : tinyfsm::Event { };
// 2026-05 deep-idle refactor FAST3c: visual marker when the M10Q has captured
// its first raw CloudLocate measurement mid-session. Double-blink CYAN
// distinguishes from LEDGNSSOn (steady CYAN flash) so bench operators can see
// when raw measurements are ready without waiting for full PVT.
struct SetLEDGNSSCloudLocateReady : tinyfsm::Event { };
// 2026-05-24: end-of-session sleep-depth indicators. Replace the legacy
// LEDGNSSOffWithoutFix (RED solid 3 s) with two distinct short patterns so
// the operator can tell at a glance whether the M10Q went to deep-idle
// (PMREQ-backup, rail on, fast wake) or full power-off (rail cut, cold
// boot on next surface). Both ~500 ms total.
struct SetLEDGNSSDeepIdle : tinyfsm::Event { };
struct SetLEDGNSSPowerOff : tinyfsm::Event { };
struct SetLEDArgosTX : tinyfsm::Event { };
struct SetLEDArgosTXComplete : tinyfsm::Event { };
struct SetLEDBatteryCritical : tinyfsm::Event { };
struct SetLEDDFUUpdate : tinyfsm::Event { };
struct SetLEDOTASuccess : tinyfsm::Event { };
struct SetLEDOTAFailed : tinyfsm::Event { };
struct SetLEDFirmwareApplied : tinyfsm::Event { };
struct SetLEDConfirmConfig : tinyfsm::Event { };
struct SetLEDConfirmExitConfig : tinyfsm::Event { };
struct SetLEDConfirmPowerOff : tinyfsm::Event { };
struct SetLEDSurfaceDetected : tinyfsm::Event { };
struct SetLEDDiveDetected : tinyfsm::Event { };

class LEDOff;
class LEDBoot;
class LEDPowerDown;
class LEDError;
class LEDPreOperationalPending;
class LEDPreOperationalBatteryNominal;
class LEDPreOperationalBatteryLow;
class LEDPreOperationalError;
class LEDConfigPending;
class LEDConfigNotConnected;
class LEDConfigConnected;
class LEDGNSSOn;
class LEDGNSSOffWithFix;
class LEDGNSSOffWithoutFix;
class LEDGNSSCloudLocateReady;   // 2026-05 deep-idle refactor FAST3c
class LEDGNSSDeepIdle;           // 2026-05-24: post-session deep-idle indicator
class LEDGNSSPowerOff;           // 2026-05-24: post-session full power-off indicator
class LEDArgosTX;
class LEDArgosTXComplete;
class LEDBatteryCritical;
class LEDDFUUpdate;
class LEDOTASuccess;
class LEDOTAFailed;
class LEDFirmwareApplied;
class LEDConfirmConfig;
class LEDConfirmExitConfig;
class LEDConfirmPowerOff;
class LEDSurfaceDetected;
class LEDDiveDetected;


/// @}

/// @brief LED FSM base — dispatches events to LED state subclasses.
class LEDState : public tinyfsm::Fsm<LEDState> {
public:
	/// 2026-05-24: latched fix validity from the most recent GPS SERVICE_LOG_UPDATED.
	/// Read by LEDGNSSDeepIdle / LEDGNSSPowerOff entry handlers to pick the
	/// indicator color: GREEN when the just-ended session had a valid fix,
	/// RED otherwise. Set by gentracker's SERVICE_LOG_UPDATED handler BEFORE
	/// the GNSS_OFF_DEEP_IDLE / GNSS_OFF_POWEROFF event that drives the LED
	/// transit, so the entry handler reads a consistent value. Public so
	/// gentracker (outside the FSM) can latch it directly without needing
	/// a dedicated tinyfsm event.
	static inline bool m_last_gnss_fix_valid = false;
protected:
	static inline bool m_is_battery_critical = false;
	static inline bool m_is_gnss_on = false;
	static inline bool m_is_magnet_engaged = false;
public:
	// --- Confirmation-gesture LED priority (2026-07) --------------------------
	// When the operator triggers a reed confirmation gesture, the LED fast-blinks
	// (BLUE=enter-config, RED=power-off, GREEN=exit-config) and waits for the 2nd
	// gesture (release + re-engage). That prompt must NOT be stomped by transient
	// background LED events firing in the same window — the boot white
	// (SetLEDBoot / boot→preop SetLEDOff), the magnet-engaged white, or a
	// GNSS/Argos/dive/surface flash — otherwise the operator loses the visual cue
	// mid-gesture (esp. when config is triggered right at startup, where the boot
	// LED sequence overlaps the confirm blink). So all transient/background events
	// route through transit_unless_confirming(), which no-ops while a confirmation
	// is pending. The three Confirm events (to show/escalate the prompt) and the
	// safety-critical Error / BatteryCritical / DFU / OTA events bypass the gate
	// and transit directly. This never blocks a real confirmation RESOLUTION: every
	// resolution path in GenTracker clears m_confirmation_pending BEFORE dispatching
	// the resolved-state LED, so the gate is already open by then.
	void react(SetLEDOff const &) { transit_unless_confirming<LEDOff>(); }
	// Magnet engage/disengage: ALWAYS track m_is_magnet_engaged (downstream states
	// colour on it), but suppress the LED repaint (enter()) while confirming so the
	// blink is untouched — the tracked state is applied when the gesture resolves.
	void react(SetLEDMagnetEngaged const &) { if (!m_is_magnet_engaged) { m_is_magnet_engaged = true; if (!led_confirmation_gesture_pending()) enter(); } }
	void react(SetLEDMagnetDisengaged const &) { if (m_is_magnet_engaged) { m_is_magnet_engaged = false; if (!led_confirmation_gesture_pending()) enter(); } }
	void react(SetLEDBoot const &) { transit_unless_confirming<LEDBoot>(); }
	void react(SetLEDPowerDown const &) { transit_unless_confirming<LEDPowerDown>(); }
	void react(SetLEDError const &) { transit<LEDError>(); }
	void react(SetLEDPreOperationalPending const &) { transit_unless_confirming<LEDPreOperationalPending>(); }
	void react(SetLEDPreOperationalError const &) { transit_unless_confirming<LEDPreOperationalError>(); }
	void react(SetLEDPreOperationalBatteryNominal const &) { transit_unless_confirming<LEDPreOperationalBatteryNominal>(); }
	void react(SetLEDPreOperationalBatteryLow const &) { transit_unless_confirming<LEDPreOperationalBatteryLow>(); }
	void react(SetLEDConfigPending const &) { transit_unless_confirming<LEDConfigPending>(); }
	void react(SetLEDConfigNotConnected const &) { transit_unless_confirming<LEDConfigNotConnected>(); }
	void react(SetLEDConfigConnected const &) { transit_unless_confirming<LEDConfigConnected>(); }
	void react(SetLEDGNSSOn const &) { transit_unless_confirming<LEDGNSSOn>(); }
	void react(SetLEDGNSSOffWithFix const &) { transit_unless_confirming<LEDGNSSOffWithFix>(); }
	void react(SetLEDGNSSOffWithoutFix const &) { transit_unless_confirming<LEDGNSSOffWithoutFix>(); }
	void react(SetLEDGNSSCloudLocateReady const &) { transit_unless_confirming<LEDGNSSCloudLocateReady>(); }
	void react(SetLEDGNSSDeepIdle const &) { transit_unless_confirming<LEDGNSSDeepIdle>(); }
	void react(SetLEDGNSSPowerOff const &) { transit_unless_confirming<LEDGNSSPowerOff>(); }
	void react(SetLEDArgosTX const &) { transit_unless_confirming<LEDArgosTX>(); }
	void react(SetLEDArgosTXComplete const &) { transit_unless_confirming<LEDArgosTXComplete>(); }
	void react(SetLEDBatteryCritical const &) { transit<LEDBatteryCritical>(); }
	void react(SetLEDDFUUpdate const &) { transit<LEDDFUUpdate>(); }
	void react(SetLEDOTASuccess const &) { transit<LEDOTASuccess>(); }
	void react(SetLEDOTAFailed const &) { transit<LEDOTAFailed>(); }
	void react(SetLEDFirmwareApplied const &) { transit<LEDFirmwareApplied>(); }
	void react(SetLEDConfirmConfig const &) { transit<LEDConfirmConfig>(); }
	void react(SetLEDConfirmExitConfig const &) { transit<LEDConfirmExitConfig>(); }
	void react(SetLEDConfirmPowerOff const &) { transit<LEDConfirmPowerOff>(); }
	void react(SetLEDSurfaceDetected const &) { transit_unless_confirming<LEDSurfaceDetected>(); }
	void react(SetLEDDiveDetected const &) { transit_unless_confirming<LEDDiveDetected>(); }

	virtual void entry(void) {}
	virtual void exit(void) {}

protected:
	/// Transit to @p S unless a reed confirmation gesture is pending, in which
	/// case swallow the event so the active confirm blink is preserved. Used by
	/// all transient/background LED events; critical and Confirm events call
	/// transit<> directly. See the confirmation-gesture priority note above.
	template<typename S>
	void transit_unless_confirming() {
		if (led_confirmation_gesture_pending()) return;
		transit<S>();
	}
};


class LEDOff : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};


class LEDBoot : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDPowerDown : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDError : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDPreOperationalPending : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDPreOperationalError : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDPreOperationalBatteryNominal : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDPreOperationalBatteryLow : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDConfigPending : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDConfigNotConnected : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDConfigConnected : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDGNSSOn : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDGNSSOffWithFix : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDGNSSOffWithoutFix : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

// 2026-05 deep-idle refactor FAST3c: distinct visual pattern when the first
// CloudLocate raw measurement arrives mid-session. Double-blink CYAN to
// distinguish from the steady CYAN flash of LEDGNSSOn — bench operator
// instantly knows raw measurements are ready (which can be uploaded via
// Argos for cloud-side position resolution even without a full PVT fix).
// After the double-blink, the state machine transitions back to LEDGNSSOn
// (if GPS still active) or LEDOff (if GNSS_CLOUDLOCATE_ONLY terminated the
// session). Transition handled inside the entry() via a scheduled task.
class LEDGNSSCloudLocateReady : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

// 2026-05-24: end-of-session deep-idle indicator (rail stays on, M10Q in
// PMREQ-backup). Double-blink RED for ~500 ms then auto-transit to LEDOff.
// Distinct from LEDGNSSPowerOff (fast blink) — operator can tell at a
// glance which sleep depth was engaged after a no-fix session.
class LEDGNSSDeepIdle : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

// 2026-05-24: end-of-session full power-off indicator (rail cut, M10Q will
// cold-boot next session). Fast blink RED for ~500 ms then auto-transit to
// LEDOff. Heavier shutdown signal than the deep-idle double-blink.
class LEDGNSSPowerOff : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDArgosTX : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDArgosTXComplete : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDBatteryCritical : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDDFUUpdate : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDOTASuccess : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDOTAFailed : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDFirmwareApplied : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDConfirmConfig : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDConfirmExitConfig : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDConfirmPowerOff : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDSurfaceDetected : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDDiveDetected : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};
