/**
 * @file gentracker.hpp
 * @brief GenTracker FSM — main state machine: Off → PreOp → Config → Operational → Error.
 *
 * Uses TinyFSM. Reed switch confirmation gesture drives state transitions.
 * Not board-specific despite the name — used by all tracker variants.
 */

#pragma once

#include "tinyfsm.hpp"
#include "scheduler.hpp"
#include "error.hpp"
#include "ble_service.hpp"
#include "reed.hpp"
#include "service.hpp"
#include "battery.hpp"

/// @brief Reed switch gesture event for FSM transitions.
struct ReedSwitchEvent : tinyfsm::Event { ReedSwitchGesture state; };
/// @brief Error event — triggers transition to ErrorState.
struct ErrorEvent : tinyfsm::Event { ErrorCode error_code; };

/// @brief Main FSM base class — handles reed switch, watchdog, config flush.
class GenTracker : public tinyfsm::Fsm<GenTracker>
{
public:
	void react(tinyfsm::Event const &);
	void react(ReedSwitchEvent const &event);
	void react(ErrorEvent const &event);
	virtual void entry(void);
	virtual void exit(void);
	void set_ble_device_name();

	static void kick_watchdog();
	static void periodic_config_flush();
	static inline bool m_config_flush_active = false;
	static void notify_bad_filesystem_error();

	/// @brief True while a reed-switch confirmation gesture is awaiting the
	/// operator's 2nd gesture (or its re-engage timeout). Read by the LED state
	/// machine to give the confirmation blink (fast BLUE=config / RED=power-off /
	/// GREEN=exit-config) priority: while this is true the LED FSM ignores
	/// transient/background LED events (boot white, magnet-engaged white, GNSS,
	/// Argos, dive/surface) so the confirmation prompt is not stomped mid-gesture.
	/// Safe because every confirmation RESOLUTION path clears m_confirmation_pending
	/// BEFORE dispatching the resolved-state LED — so a real resolution is never
	/// gated. See ledsm.cpp / led_confirmation_gesture_pending().
	static bool is_confirmation_gesture_pending() {
		return m_confirmation_pending != ConfirmationPending::NONE;
	}

protected:
	enum class ConfirmationPending { NONE, ENTER_CONFIG, EXIT_CONFIG, POWEROFF };
	static inline ConfirmationPending m_confirmation_pending = ConfirmationPending::NONE;
	static inline bool m_awaiting_re_engage = false;
	static inline Scheduler::TaskHandle m_confirmation_timeout_task;
	// Re-engage window after RELEASE. Bumped 2000→3000 ms (2026-06) to give the
	// operator margin when the device is busy/sluggish (the gesture pipeline is
	// scheduler-driven; see reed.cpp REED_GESTURE_PRIORITY). Strictly more
	// lenient — never rejects a re-engage that the old 2 s window accepted — and
	// well within PreOperationalState's 20 s stuck-reed escape.
	static constexpr unsigned int CONFIRMATION_TIMEOUT_MS = 3000;
	void cancel_confirmation();
};


/// @brief Boot state — init hardware, check reset cause, transition to Off or PreOp.
class BootState : public GenTracker
{
public:
	void entry() override;
	void exit() override;
};

/// @brief Off state — device sleeping, periodic LED blink, reed switch wakes to PreOp.
class OffState : public GenTracker
{
private:
	static constexpr unsigned int OFF_LED_PERIOD_MS = 5000;
	Scheduler::TaskHandle m_off_state_task;

public:
	void entry() override;
	void exit() override;
};

/// @brief PreOperational — init config, mount filesystem, transition to Operational.
class PreOperationalState : public GenTracker
{
private:
#ifdef EXTERNAL_WAKEUP
	static constexpr unsigned int TRANSIT_PERIOD_MS = 100;
#else
	static constexpr unsigned int TRANSIT_PERIOD_MS = 5000;
#endif
	Scheduler::TaskHandle m_preop_state_task;

	// Stuck-reed escape: if a confirmation gesture is pending but the magnet
	// never RELEASEs (manufacturing residue / shorted switch / hardware
	// short), the device would otherwise stay in PreOp forever drawing
	// ~5-20 mA — catastrophic for a sealed turtle. We re-arm the transit
	// task and force Operational after PREOP_STUCK_REED_MAX_MS elapsed.
	unsigned int m_preop_stuck_reed_ticks = 0;
	static constexpr unsigned int PREOP_STUCK_REED_MAX_MS = 20000;  // 20 s

	/// @brief Self-reposting transit tick — advances the stuck-reed escape counter
	///        and re-checks the 20 s force-transit on EVERY fire, then transits to
	///        Operational (confirmation cleared, or escape elapsed) or re-arms itself.
	void preop_transit_tick();

public:
	void entry() override;
	void exit() override;
};

/// @brief Operational — all services running, GPS/TX/sensors active, battery monitoring.
class OperationalState : public GenTracker, public BatteryMonitorEventListener
{
private:
	void service_event_handler(ServiceEvent &e);

public:
	void entry() override;
	void exit() override;
	void react(BatteryMonitorEventVoltageCritical const &) override;
};

/// @brief Configuration — BLE/USB DTE interface active, accepts commands, OTA updates.
class ConfigurationState : public GenTracker
{
private:
	static constexpr unsigned int BLE_INACTIVITY_TIMEOUT_MS = 20 * 60 * 1000;  ///< 20 min for OTA
	static constexpr unsigned int USB_POLL_INTERVAL_MS = 50;
	Scheduler::TaskHandle m_ble_inactivity_timeout_task;
	Scheduler::TaskHandle m_usb_poll_task;
	Scheduler::TaskHandle m_backup_charge_blink_task;
	/// MED #6 audit fix: track the BackupChargeStopBLE task handle so it
	/// can be cancelled in `exit()`. Previously this task was posted with
	/// `[this]` capture but its handle was discarded — if the operator
	/// transit'd out of ConfigurationState within the 200 ms window, the
	/// lambda would fire with a dangling `this` pointer (dead state object).
	Scheduler::TaskHandle m_backup_charge_stop_ble_task;
	bool m_backup_charge_mode = false;

	/// @brief Periodic yellow LED blink (200 ms on, 10 s off) during DTE-triggered
	/// backup-charge. Visual heartbeat so the user can see the device is still alive
	/// in the silent charging state. Self-reschedules until m_backup_charge_mode = false.
	void backup_charge_blink_fire();
	int on_ble_event(BLEServiceEvent&);
	void on_ble_inactivity_timeout();
	void restart_inactivity_timeout();
	void process_received_data();
	void process_usb_data();
	void schedule_usb_poll();

public:
	void entry() override;
	void exit() override;
};

/// @brief Battery critical — services stopped, 2-min timeout before powerdown.
class BatteryCriticalState : public GenTracker
{
private:
	static constexpr unsigned int BATTERY_CRITICAL_TIMEOUT_MS = 2 * 60 * 1000;
	Scheduler::TaskHandle m_transit_task;
public:
	void entry() override;
	void exit() override;
};

/// @brief Error state — RED LED, 30s delay before powerdown.
class ErrorState : public GenTracker
{
private:
	Scheduler::TaskHandle m_shutdown_task;
public:
	void entry();
	void exit();
};
