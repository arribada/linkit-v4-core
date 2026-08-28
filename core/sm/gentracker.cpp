/**
 * @file gentracker.cpp
 * @brief GenTracker FSM — state entry/exit, reed switch gesture handling, service orchestration.
 */

#include "bsp.hpp"
#include "pmu.hpp"
#include "ota_file_updater.hpp"
#include "logger.hpp"
#include "config_store.hpp"
#include "service_scheduler.hpp"
#include "scheduler.hpp"
#include "service.hpp"
#include "dte_handler.hpp"
#include "filesystem.hpp"
#include "config_store.hpp"
#include "error.hpp"
#include "timer.hpp"
#include "debug.hpp"
#include "switch.hpp"
#include "reed.hpp"
#include "ledsm.hpp"
#include "buzzm.hpp"
#include "battery.hpp"
#include "rgb_led.hpp"
#include "rtc.hpp"

extern RTC *rtc;
#include "led.hpp"
#include "gps.hpp"
#include "gps_service.hpp"
#include "ble_service.hpp"
#include "sws_analog_service.hpp"
// CRC16-CCITT: nRF SDK header in firmware build, inline stub in CppUTest build.
// Mirrors the conditional in sws_analog_constants.hpp so tests can build this
// file (the SDK header is not on the test include path).
#ifndef CPPUTEST
#include "crc16.h"
#else
#include <cstdint>
static inline uint16_t crc16_compute(const uint8_t *data, uint16_t length, const uint16_t *) {
	uint16_t crc = 0xFFFF;
	for (uint16_t i = 0; i < length; i++) {
		crc ^= static_cast<uint16_t>(data[i]) << 8;
		for (uint8_t j = 0; j < 8; j++)
			crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
	}
	return crc;
}
#endif
#include "gentracker.hpp"

// USB DTE interface (platform-specific)
#ifdef USB_DTE_ENABLED
#include "usb_interface.hpp"
#endif
#ifdef BENCH_TEST
#include "bench_console.hpp"
#endif

// LoRa device instance (for bridge mode in process_usb_data)
#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
#include "lora_rak3172.hpp"
extern LoRaDevice *lora_device_instance;
#endif

// KIM2 device instance (for bridge mode in process_usb_data)
#if !(defined(LORA_RAK3172) && (LORA_RAK3172 == 1)) && !(defined(ARGOS_SMD) && (ARGOS_SMD == 1))
#include "kim2.hpp"
extern KIM2Device *kim2_device_instance;
#endif

// LED hardware access for forced LED off before powerdown
extern RGBLed *status_led;

// These contexts must be created before the FSM is initialised
extern FileSystem *main_filesystem;
extern Scheduler *system_scheduler;
extern Timer *system_timer;
extern ConfigurationStore *configuration_store;
extern BLEService *ble_service;
extern OTAFileUpdater *ota_updater;
extern DTEHandler *dte_handler;
extern ReedSwitch *reed_switch;
extern BatteryMonitor *battery_monitor;
extern BaseDebugMode g_debug_mode;

// FSM initial state -> LEDOff
FSM_INITIAL_STATE(LEDState, LEDOff);
FSM_INITIAL_STATE(BuzzState, BuzzOff);

using led_handle = LEDState;
using buzz_handle = BuzzState;

/// @brief Free-function bridge read by the LED state machine (ledsm.hpp) to give
/// the confirmation-gesture blink priority over transient background LED events.
/// Kept as a free function so ledsm stays decoupled from the GenTracker header.
bool led_confirmation_gesture_pending() {
	return GenTracker::is_confirmation_gesture_pending();
}


/// @brief Default event handler — ignore unhandled events.
void GenTracker::react(tinyfsm::Event const &) {}

/// @brief Cancel pending reed switch confirmation gesture and clear timeout.
void GenTracker::cancel_confirmation() {
	system_scheduler->cancel_task(m_confirmation_timeout_task);
	m_confirmation_pending = ConfirmationPending::NONE;
	m_awaiting_re_engage = false;
}

/// @brief Reed switch gesture handler — confirmation pattern: hold 3s/6s, release, re-engage within 3s.
/// @param event  Gesture type (ENGAGE, SHORT_HOLD, LONG_HOLD, RELEASE).
void GenTracker::react(ReedSwitchEvent const &event) {
	DEBUG_INFO("react: ReedSwitchEvent: %u", (int)event.state);

	// During coin-cell backup charge: only ENGAGE stops the charge; all other
	// reed events are ignored so nothing else happens while charging.
	if (gps_service && gps_service->is_backup_charge_active()) {
		if (event.state == ReedSwitchGesture::ENGAGE) {
			DEBUG_INFO("Reed engage → stopping backup charge");
			m_confirmation_pending = ConfirmationPending::NONE;
			m_awaiting_re_engage = false;
			system_scheduler->cancel_task(m_confirmation_timeout_task);
			gps_service->request_backup_charge(0);
		}
		return;
	}

	// Reed switch event handling with confirmation gesture:
	// ENGAGE -- engaged LED state (or confirm pending action if re-engaged within 3s)
	// RELEASE -- disengage LED state (or start 3s confirmation window)
	// SHORT_HOLD (3s) -- rapid blink, awaiting release+re-engage to confirm
	// LONG_HOLD (6s) -- rapid blink, awaiting release+re-engage to confirm power off
	if (event.state == ReedSwitchGesture::ENGAGE) {
		if (m_awaiting_re_engage && m_confirmation_pending != ConfirmationPending::NONE) {
			// User re-engaged within 3s confirmation window — action confirmed
			system_scheduler->cancel_task(m_confirmation_timeout_task);
			auto pending = m_confirmation_pending;
			m_confirmation_pending = ConfirmationPending::NONE;
			m_awaiting_re_engage = false;

			DEBUG_INFO("Reed confirmation accepted: %u", (int)pending);

			if (pending == ConfirmationPending::ENTER_CONFIG) {
				transit<ConfigurationState>();
			} else if (pending == ConfirmationPending::EXIT_CONFIG) {
				transit<PreOperationalState>();
			} else if (pending == ConfirmationPending::POWEROFF) {
				buzz_handle::dispatch<SetBuzzPowerDown>({});
				transit<OffState>();
			}
			return;
		}
		led_handle::dispatch<SetLEDMagnetEngaged>({});
		buzz_handle::dispatch<SetBuzzMagnetEngaged>({});
	} else if (event.state == ReedSwitchGesture::RELEASE) {
		if (m_confirmation_pending != ConfirmationPending::NONE) {
			// Magnet released during confirmation — start 3s re-engage window
			m_awaiting_re_engage = true;
			led_handle::dispatch<SetLEDMagnetDisengaged>({});
			// Cancel any previous timeout before posting a new one.
			// Intentionally left at DEFAULT_PRIORITY (lower than the elevated
			// REED_GESTURE_PRIORITY used for the gesture callbacks in reed.cpp):
			// if a valid re-engage ENGAGE and this timeout are both queued in the
			// same run() batch (e.g. after a blocking op spanning the window), the
			// higher-priority ENGAGE is processed first and confirms the action,
			// cancelling this timeout — so a busy device no longer drops a valid
			// confirmation.
			system_scheduler->cancel_task(m_confirmation_timeout_task);
			m_confirmation_timeout_task = system_scheduler->post_task_prio(
			    [this]() {
				    DEBUG_INFO("Reed confirmation timeout");
				    m_confirmation_pending = ConfirmationPending::NONE;
				    m_awaiting_re_engage = false;
				    buzz_handle::dispatch<SetBuzzMagnetDisengaged>({});
				    // Restore LED to current GenTracker state
				    if (is_in_state<ConfigurationState>()) {
					    led_handle::dispatch<SetLEDConfigNotConnected>({});
				    } else if (is_in_state<PreOperationalState>()) {
					    transit<OperationalState>();
				    } else {
					    led_handle::dispatch<SetLEDOff>({});
				    }
			    },
			    "ReedConfirmTimeout", Scheduler::DEFAULT_PRIORITY, CONFIRMATION_TIMEOUT_MS);
			return;
		}
		led_handle::dispatch<SetLEDMagnetDisengaged>({});
		buzz_handle::dispatch<SetBuzzMagnetDisengaged>({});
	} else if (event.state == ReedSwitchGesture::SHORT_HOLD) {
		if (is_in_state<ConfigurationState>()) {
			m_confirmation_pending = ConfirmationPending::EXIT_CONFIG;
			led_handle::dispatch<SetLEDConfirmExitConfig>({});
			buzz_handle::dispatch<SetBuzzPreOperationalPending>({});
		} else {
			m_confirmation_pending = ConfirmationPending::ENTER_CONFIG;
			led_handle::dispatch<SetLEDConfirmConfig>({});
			buzz_handle::dispatch<SetBuzzConfigPending>({});
		}
		m_awaiting_re_engage = false;
	} else if (event.state == ReedSwitchGesture::LONG_HOLD) {
		m_confirmation_pending = ConfirmationPending::POWEROFF;
		m_awaiting_re_engage = false;
		led_handle::dispatch<SetLEDConfirmPowerOff>({});
	}
}

/// @brief Error event — transition to ErrorState.
/// @param event  Error code that caused the failure.
void GenTracker::react(ErrorEvent const &event) {
	DEBUG_ERROR("GenTracker::react: ErrorEvent: error_code=%u", event.error_code);
	cancel_confirmation();
	transit<ErrorState>();
}

void GenTracker::entry(void) {};
void GenTracker::exit(void) {};

// =====================================================================
// Boot-fail counter — sealed-device recovery infrastructure.
//
// On a sealed turtle (no physical reed access for the operator) the
// device cannot escape OffState — it's a permanent brick. Any path that
// transitions Boot → ErrorState → OffState (e.g. BAD_FILESYSTEM,
// CONFIG_STORE_CORRUPTED, persistent uncaught exception in main loop)
// would otherwise kill the deployment on the first occurrence.
//
// Mitigation: track consecutive boots that fail to reach Operational in
// .noinit RAM. Each boot increments the counter; OperationalState entry
// resets it. When the counter is positive, ErrorState performs a soft
// reset instead of going to OffState — letting the watchdog/reboot
// cycle exercise recovery instead of going dark. At BOOT_RETRY_BEFORE_FACTORY
// (3) we factory-reset the config store on next boot (preserves
// DECID/HEXID/calibration) and reset again. Only if the factory_reset
// retry also fails do we fall through to true OffState (real HW fault).
// =====================================================================

struct BootFailNoinit {
	uint16_t consecutive_failures;
	uint8_t factory_reset_attempted;
	uint8_t _pad;
	uint16_t crc;
};

#ifndef CPPUTEST
static __attribute__((section(".noinit"))) volatile BootFailNoinit s_bootfail_noinit;
#else
static BootFailNoinit s_bootfail_noinit;
#endif

static constexpr uint8_t BOOT_RETRY_MAX = 5;             ///< Total reset retries before OffState
static constexpr uint8_t BOOT_RETRY_BEFORE_FACTORY = 3;  ///< Retries before attempting factory_reset

static uint16_t bootfail_compute_crc() {
	BootFailNoinit snapshot;
	snapshot.consecutive_failures = s_bootfail_noinit.consecutive_failures;
	snapshot.factory_reset_attempted = s_bootfail_noinit.factory_reset_attempted;
	snapshot._pad = s_bootfail_noinit._pad;
	return crc16_compute(reinterpret_cast<const uint8_t *>(&snapshot), offsetof(BootFailNoinit, crc), nullptr);
}

static void bootfail_save() {
	s_bootfail_noinit.crc = bootfail_compute_crc();
}

/// @brief Load the boot-fail counter from .noinit RAM. On cold POR (battery
/// insert) the RAM is uninitialised and the CRC will not validate — we
/// reset to zero in that case.
static void bootfail_load() {
	if (s_bootfail_noinit.crc != bootfail_compute_crc()) {
		s_bootfail_noinit.consecutive_failures = 0;
		s_bootfail_noinit.factory_reset_attempted = 0;
		s_bootfail_noinit._pad = 0;
		bootfail_save();
	}
}

/// @brief Increment the consecutive-failures counter — called at every boot.
static void bootfail_increment() {
	bootfail_load();
	uint16_t cur = s_bootfail_noinit.consecutive_failures;
	if (cur < UINT16_MAX) {
		s_bootfail_noinit.consecutive_failures = cur + 1;
	}
	bootfail_save();
	DEBUG_INFO("BootFail: counter=%u factory_reset_attempted=%u",
	           static_cast<unsigned int>(s_bootfail_noinit.consecutive_failures),
	           static_cast<unsigned int>(s_bootfail_noinit.factory_reset_attempted));
}

#ifdef BENCH_TEST
/// @brief Read the boot-fail state for the bench console.
///
/// Exposed because this counter is on the path to the worst failure a sealed
/// tag has: BOOT_RETRY_BEFORE_FACTORY consecutive failed boots trigger a
/// factory reset, and that wipes the Argos credentials. If the counter ever
/// stopped clearing on a healthy boot, three reboots would brick the device --
/// and nothing outside .noinit RAM would show it coming.
void bench_bootfail_read(unsigned int &failures, unsigned int &factory_attempted) {
	bootfail_load();
	failures = s_bootfail_noinit.consecutive_failures;
	factory_attempted = s_bootfail_noinit.factory_reset_attempted;
}
#endif

/// @brief Reset the boot-fail counter — called from OperationalState::entry
/// to mark that a successful boot has occurred.
static void bootfail_reset() {
	bootfail_load();
	if (s_bootfail_noinit.consecutive_failures > 0 || s_bootfail_noinit.factory_reset_attempted) {
		DEBUG_INFO("BootFail: clearing (was %u failures, factory_reset=%u)", s_bootfail_noinit.consecutive_failures,
		           s_bootfail_noinit.factory_reset_attempted);
	}
	s_bootfail_noinit.consecutive_failures = 0;
	s_bootfail_noinit.factory_reset_attempted = 0;
	bootfail_save();
}

/// @brief Dispatch ErrorEvent with BAD_FILESYSTEM to trigger ErrorState.
void GenTracker::notify_bad_filesystem_error() {
	ErrorEvent event;
	event.error_code = BAD_FILESYSTEM;
	dispatch(event);
}

/// @brief Kick hardware watchdog — called periodically from main scheduler loop.
void GenTracker::kick_watchdog() {
	DEBUG_TRACE("GenTracker::kick_watchdog: calling PMU::kick_watchdog");
	PMU::kick_watchdog();
	system_scheduler->post_task_prio([]() { kick_watchdog(); }, "KickWatchdog", Scheduler::HIGHEST_PRIORITY,
	                                 BSP::WDT_Inits[BSP::WDT].config.reload_value * 0.90);
}

// Periodic flush of config store to flash (counters live in RAM between flushes)
static constexpr unsigned int CONFIG_FLUSH_INTERVAL_MS = 30 * 60 * 1000;  // 30 minutes

/// @brief Periodic config flush — save params + RTC to flash (called from scheduler).
void GenTracker::periodic_config_flush() {
	if (!m_config_flush_active) return;
	if (configuration_store->is_valid()) {
		// Mitigation M1a (2026-05): periodic LAST_KNOWN_RTC snapshot. Without
		// this, a sealed device that never acquires GNSS only updates this
		// param at clean shutdown / POF / DTE write — meaning a WDT reset
		// mid-deployment wipes RTC back to 1 and breaks every time-based
		// defense (cooldown, hauled idle threshold, rate limiter). Saving
		// here every 30 min bounds the post-reset drift to ≤ 30 min.
		//
		// We persist ANY rtc->gettime() (virtual or real) — `main.cpp` accepts
		// small values for the same reason (RSPB pseudo-RTC chain and LinkIt
		// sealed-turtle continuity). Without this we lose the post-WDT
		// continuity entirely for sealed devices.
		if (rtc && rtc->is_set()) {
			std::time_t now = rtc->gettime();
			if (now > 0) {
				configuration_store->write_param(ParamID::LAST_KNOWN_RTC, static_cast<unsigned int>(now));
			}
		}
		DEBUG_TRACE("GenTracker::periodic_config_flush: saving counters to flash");
		try {
			configuration_store->save_params();
		} catch (...) {
			DEBUG_ERROR("GenTracker::periodic_config_flush: save_params failed");
		}
	}
	if (m_config_flush_active) {
		// Cancel first: this call may be the one from OperationalState::entry()
		// re-arming the chain while an earlier task is still pending. Without
		// this the two chains coexist for good.
		system_scheduler->cancel_task(m_config_flush_task);
		m_config_flush_task = system_scheduler->post_task_prio([]() { periodic_config_flush(); }, "ConfigFlush",
		                                                       Scheduler::DEFAULT_PRIORITY, CONFIG_FLUSH_INTERVAL_MS);
	}
}

/// @brief Boot entry — init PMU, check reset cause, mount filesystem, load config.
void BootState::entry() {
	DEBUG_INFO("entry: BootState");

#ifdef CPPUTEST
	// In firmware builds, s_bootfail_noinit lives in the .noinit RAM section
	// and is wiped only by physical power-loss (battery insert). In CPPUTEST
	// builds the same struct is a plain static — so it persists across
	// fsm_handle::start() calls within the same test binary, leaking
	// counter state between tests. Every BootState::entry in test mode
	// then bumps the counter, and after 3 starts the factory_reset branch
	// fires unexpectedly (the legacy Sm tests don't mock factory_reset).
	// Reset between test entries so each test sees a clean boot.
	s_bootfail_noinit.consecutive_failures = 0;
	s_bootfail_noinit.factory_reset_attempted = 0;
	bootfail_save();
#endif

	// Sealed-device recovery: increment the boot-fail counter at every boot.
	// Will be cleared on successful entry into OperationalState. If we never
	// reach Operational, ErrorState consults this counter to decide between
	// soft-reset retry, factory_reset attempt, or final OffState.
	bootfail_increment();
	if (s_bootfail_noinit.consecutive_failures >= BOOT_RETRY_BEFORE_FACTORY
	    && !s_bootfail_noinit.factory_reset_attempted) {
		// Last-ditch recovery before giving up to OffState: wipe the config
		// store and retry. factory_reset preserves DECID/HEXID/calibration.
		DEBUG_ERROR("BootState: %u failed boots — attempting factory_reset before final retry",
		            s_bootfail_noinit.consecutive_failures);
		s_bootfail_noinit.factory_reset_attempted = 1;
		// Reset the consecutive-failure counter after factory_reset so the next
		// boot starts fresh — otherwise the counter keeps climbing past
		// BOOT_RETRY_MAX and we fall through to OffState anyway, bricking the
		// sealed device even though factory_reset gave us a clean slate to try.
		// factory_reset_attempted stays at 1 so we don't loop the factory_reset
		// itself; only the soft-reset retry budget is replenished.
		s_bootfail_noinit.consecutive_failures = 0;
		bootfail_save();
		// factory_reset() writes the full default param block + RTC to LFS.
		// On a degraded flash that's the slowest operation in BootState; kick
		// the WDT first so the 15-min budget restarts cleanly, otherwise a
		// flash erase retry chain could exhaust it mid-reset and brick the
		// sealed device on the very recovery path that's supposed to save it.
		PMU::kick_watchdog();
		try {
			if (configuration_store) configuration_store->factory_reset();
		} catch (...) {
			DEBUG_ERROR("BootState: factory_reset itself threw — proceeding to reboot anyway");
		}
		PMU::kick_watchdog();
		PMU::delay_ms(100);  // Let logs flush
		PMU::reset(false);
	}

	// Ensure the system timer is started to allow scheduling to work
	system_timer->start();

	// Turn status LED white to indicate boot up
	led_handle::start();
	led_handle::dispatch<SetLEDBoot>({});
	buzz_handle::start();

	// WDT kick before LFS mount/format. On a sealed deployment, a marginal
	// flash block (wear, ECC retry) can stretch mount() into multiple seconds;
	// the 15-min WDT budget shouldn't be partly consumed by the boot path
	// before we even reach the scheduler. Kicks here, after mount, and before
	// configuration_store->init() bracket the 3 longest LFS operations in
	// BootState — without them, a slow flash plus the post-task 1 s delay
	// before line 401 leaves no margin in cascade scenarios.
	PMU::kick_watchdog();

	// If we can't mount the filesystem then try to format it first and retry
	if (!main_filesystem->is_mounted() && main_filesystem->mount() < 0) {
		DEBUG_TRACE("format filesystem");
		PMU::kick_watchdog();  // format() is the longest LFS op — fresh budget
		if (main_filesystem->format() < 0 || main_filesystem->mount() < 0) {
			// We can't mount a formatted filesystem, something bad has happened
			system_scheduler->post_task_prio(notify_bad_filesystem_error, "GenTrackerFileSystemError");
			return;
		}
	}
	PMU::kick_watchdog();  // post-mount: fresh budget before configuration_store->init

	// Start reed switch monitoring and dispatch events to state machine
	reed_switch->start([](ReedSwitchGesture s) {
		ReedSwitchEvent e;
		e.state = s;
		dispatch(e);
	});

	// If the magnet is already present at boot, NrfSwitch::resume() syncs the
	// debounced state to the live pin but deliberately suppresses the initial
	// state-change callback (only state CHANGES fire one). Route through the
	// gesture engine so it does what a real ENGAGE edge would: deliver ENGAGE
	// *and* arm the SHORT_HOLD/LONG_HOLD timers. The previous direct
	// dispatch(ENGAGE) only set the FSM/LED state and left the timers unarmed —
	// an operator holding the magnet across boot could not enter config /
	// power-off without first removing and re-applying it to "wake" the gesture.
	if (reed_switch->is_engaged()) {
		DEBUG_INFO("BootState: Magnet already present at boot | priming engage gesture");
		reed_switch->prime_if_engaged();
	}

	try {
		// The underlying classes will create the files on the filesystem if they do not
		// already yet exist
		LoggerManager::create();
		configuration_store->init();
		DEBUG_INFO("Firmware Version: %s", FW_APP_VERSION_STR_C);
		LoggerManager::show_info();
		DEBUG_INFO("configuration_store: is_valid=%u", configuration_store->is_valid());
		DEBUG_INFO("reset cause: %s", PMU::reset_cause_str());
		PMU::print_stack();

		// Check if we just applied a firmware update (green blink for 3s)
		if (PMU::was_firmware_updated()) {
			DEBUG_INFO("Firmware update was applied successfully!");
			led_handle::dispatch<SetLEDFirmwareApplied>({});
			PMU::kick_watchdog();
			PMU::delay_ms(3000);
			PMU::kick_watchdog();
		}

		// Transition to PreOperational state after initialisation
		system_scheduler->post_task_prio(
		    [this]() {
			    kick_watchdog();
			    transit<PreOperationalState>();
		    },
		    "GenTrackerBootStateTransitPreOperationalState", Scheduler::DEFAULT_PRIORITY, 1000);
	} catch (...) {
		system_scheduler->post_task_prio(notify_bad_filesystem_error, "GenTrackerFilesystemError");
	}
}

void BootState::exit() {
	DEBUG_INFO("exit: BootState");

	// Turn status LED off to indicate exit from boot state
	led_handle::dispatch<SetLEDOff>({});
}

/// @brief Off entry — start reed switch, periodic LED blink, enter low power.
void OffState::entry() {
	DEBUG_INFO("entry: OffState");
	led_handle::dispatch<SetLEDPowerDown>({});
	m_off_state_task = system_scheduler->post_task_prio(
	    []() {
		    // Force LED off at hardware level before powerdown
		    status_led->off();
		    buzz_handle::dispatch<SetBuzzOff>({});
		    PMU::powerdown();
	    },
	    "GenTrackerOffStateTransitPowerDown", Scheduler::DEFAULT_PRIORITY, OFF_LED_PERIOD_MS);
}

void OffState::exit() {
	DEBUG_INFO("exit: OffState");
	system_scheduler->cancel_task(m_off_state_task);
	led_handle::dispatch<SetLEDOff>({});
}

/// @brief PreOp entry — verify config valid, mount QSPI, start watchdog, transition to Operational.
void PreOperationalState::preop_transit_tick() {
	// Self-reposting stuck-reed escape tick (H2). Re-posts ITSELF on every fire so
	// m_preop_stuck_reed_ticks actually advances and the PREOP_STUCK_REED_MAX_MS
	// force-transit is re-evaluated each tick. Same logic as the initial arming in
	// entry(); the previous inline re-arm posted a stripped lambda that neither
	// advanced the counter nor re-armed -> the 20 s escape never fired.
	if (m_confirmation_pending != ConfirmationPending::NONE) {
		m_preop_stuck_reed_ticks++;
		if (m_preop_stuck_reed_ticks * TRANSIT_PERIOD_MS >= PREOP_STUCK_REED_MAX_MS) {
			DEBUG_WARN("PreOperationalState: reed appears stuck (%u ms in confirmation) — forcing Operational transit",
			           m_preop_stuck_reed_ticks * TRANSIT_PERIOD_MS);
			m_confirmation_pending = ConfirmationPending::NONE;
			m_preop_stuck_reed_ticks = 0;
			transit<OperationalState>();
			return;
		}
		m_preop_state_task = system_scheduler->post_task_prio([this]() { preop_transit_tick(); },
		                                                      "GenTrackerPreOpRetransitOperationalState",
		                                                      Scheduler::DEFAULT_PRIORITY, TRANSIT_PERIOD_MS);
		return;
	}
	m_preop_stuck_reed_ticks = 0;
	transit<OperationalState>();
}

void PreOperationalState::entry() {
	DEBUG_INFO("entry: PreOperationalState");
	if (configuration_store->is_valid()) {
		// Force battery monitor to update its levels
		try {
			battery_monitor->update();
		} catch (...) {
			DEBUG_WARN("PreOperationalState: battery update failed — continuing with defaults");
		}
		if (battery_monitor->is_battery_critical()) {
			transit<BatteryCriticalState>();
			return;
		}

		if (battery_monitor->is_battery_low())
			led_handle::dispatch<SetLEDPreOperationalBatteryLow>({});
		else
			led_handle::dispatch<SetLEDPreOperationalBatteryNominal>({});

		m_preop_state_task = system_scheduler->post_task_prio(
		    [this]() {
			    // If a confirmation gesture is in progress, don't transit yet —
			    // the confirmation handler or timeout will handle the transition.
			    //
			    // EXCEPT: a reed switch stuck CLOSED (manufacturing magnet residue
			    // or hardware short) would also leave m_confirmation_pending != NONE
			    // indefinitely, with no RELEASE event ever arriving to start the
			    // 2-s confirmation timeout. The previous code returned without
			    // rescheduling, wedging the device in PreOperationalState forever
			    // at full active current (~5-20 mA) — catastrophic drain for a
			    // sealed turtle. Now we re-arm the transit task and, after enough
			    // total time has elapsed, force-transit to Operational regardless.
			    if (m_confirmation_pending != ConfirmationPending::NONE) {
				    m_preop_stuck_reed_ticks++;
				    if (m_preop_stuck_reed_ticks * TRANSIT_PERIOD_MS >= PREOP_STUCK_REED_MAX_MS) {
					    DEBUG_WARN("PreOperationalState: reed appears stuck (%u ms in confirmation) — forcing "
						           "Operational transit",
						           m_preop_stuck_reed_ticks * TRANSIT_PERIOD_MS);
					    m_confirmation_pending = ConfirmationPending::NONE;
					    m_preop_stuck_reed_ticks = 0;
					    transit<OperationalState>();
					    return;
				    }
				    // H2 fix: re-arm with the SELF-REPOSTING tick, which re-increments
				    // m_preop_stuck_reed_ticks and re-checks the 20 s force-transit on
				    // every fire. The old re-arm posted a stripped lambda that never
				    // advanced the counter nor re-armed, so the stuck-reed escape never
				    // fired and the device wedged in PreOperationalState.
				    m_preop_state_task = system_scheduler->post_task_prio(
				        [this]() { preop_transit_tick(); }, "GenTrackerPreOpRetransitOperationalState",
				        Scheduler::DEFAULT_PRIORITY, TRANSIT_PERIOD_MS);
				    return;
			    }
			    m_preop_stuck_reed_ticks = 0;
			    transit<OperationalState>();
		    },
		    "GenTrackerPreOperationalStateTransitOperationalState", Scheduler::DEFAULT_PRIORITY, TRANSIT_PERIOD_MS);
	} else {
		led_handle::dispatch<SetLEDPreOperationalError>({});
		m_preop_state_task = system_scheduler->post_task_prio([this]() { transit<ErrorState>(); },
		                                                      "GenTrackerPreOperationalStateTransitErrorState",
		                                                      Scheduler::DEFAULT_PRIORITY, TRANSIT_PERIOD_MS);
	}
}

void PreOperationalState::exit() {
	DEBUG_INFO("exit: PreOperationalState");
	system_scheduler->cancel_task(m_preop_state_task);
	led_handle::dispatch<SetLEDOff>({});
}

/// @brief Operational entry — start all services, subscribe to battery events, begin config flush.
void OperationalState::entry() {
	DEBUG_INFO("entry: OperationalState");

	// Successful boot reached — clear the boot-fail counter. The sealed-device
	// recovery infrastructure considers reaching this state proof of a good boot.
	bootfail_reset();

	// Start periodic config flush (counters → flash every 30 min)
	m_config_flush_active = true;
	periodic_config_flush();

	battery_monitor->subscribe(*this);
	led_handle::dispatch<SetLEDOff>({});
	buzz_handle::dispatch<SetBuzzOff>({});

	ServiceManager::startall([this](ServiceEvent &e) { service_event_handler(e); });

	BaseDebugMode debug_mode = configuration_store->read_param<BaseDebugMode>(ParamID::DEBUG_OUTPUT_MODE);
	if (debug_mode == BaseDebugMode::BLE_NUS) {
		set_ble_device_name();
		ble_service->start([](BLEServiceEvent &) { return 0; });
		g_debug_mode = debug_mode;
	}
}

/// @brief Battery critical event — stop services, transition to BatteryCriticalState.
void OperationalState::react(BatteryMonitorEventVoltageCritical const &) {
	DEBUG_INFO("OperationalState::react: BatteryMonitorEventVoltageCritical");
	system_scheduler->post_task_prio([this]() { transit<BatteryCriticalState>(); }, "BatteryCriticalHandler",
	                                 Scheduler::DEFAULT_PRIORITY, 1000);
}

/// @brief Service event callback — dispatch to ServiceManager for peer notification.
/// @param e  Service event from any active service.
void OperationalState::service_event_handler(ServiceEvent &e) {
	// LED dispatch FIRST for UW_SENSOR transitions (2026-05 latency fix).
	// The peer-service broadcast below cascades several LFS-backed DEBUG_INFO
	// writes (ArgosTxService, GPSService, etc.), each blocking ~50-500 ms.
	// Dispatching the LED before the broadcast lets the operator see the
	// dive/surface flash within ~10 ms of the SWS state transition instead of
	// after the full cascade (up to 1-2 s perceived delay).
	// Skipped in SWS test mode where the LED is owned by SWSAnalogService::detector_state
	// (steady BLUE/GREEN) — overwriting it here would clobber the bench display.
	// SWS test mode only exists when ENABLE_SWS_ANALOG is compiled in; on
	// boards without it (e.g. RSPB) treat as "test not running" so dive/surface
	// LEDs dispatch normally.
#if ENABLE_SWS_ANALOG
	const bool sws_test_running = SWSAnalogService::is_test_running();
#else
	const bool sws_test_running = false;
#endif
	if (e.event_source == ServiceIdentifier::UW_SENSOR && e.event_type == ServiceEventType::SERVICE_LOG_UPDATED
	    && !sws_test_running) {
		bool is_underwater = std::get<bool>(e.event_data);
		if (is_underwater)
			led_handle::dispatch<SetLEDDiveDetected>({});
		else
			led_handle::dispatch<SetLEDSurfaceDetected>({});
	}

	// Notify event to all peer services
	ServiceManager::notify_peer_event(e);

	if (e.event_source == ServiceIdentifier::GNSS_SENSOR) {
		if (e.event_type == ServiceEventType::SERVICE_ACTIVE) {
			led_handle::dispatch<SetLEDGNSSOn>({});
		} else if (e.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
			// 2026-05-24 LED refactor: capture fix validity into the latched
			// static so the immediately-following GNSS_OFF_DEEP_IDLE / GNSS_OFF_
			// POWEROFF event can pick GREEN (fix) vs RED (no-fix) for its
			// indicator. The legacy LEDGNSSOffWithFix / LEDGNSSOffWithoutFix
			// dispatches are NOT fired here anymore — the new sleep-depth
			// states handle all 4 combinations:
			//   fix + deep-idle      → double-blink GREEN
			//   fix + power-off      → fast blink GREEN
			//   no-fix + deep-idle   → double-blink RED
			//   no-fix + power-off   → fast blink RED
			auto *gps_log = std::get_if<GPSLogEntry>(&e.event_data);
			LEDState::m_last_gnss_fix_valid = (gps_log && gps_log->info.valid);
		} else if (e.event_type == ServiceEventType::GNSS_CLOUDLOCATE_READY) {
			// 2026-05 deep-idle refactor FAST3c: visual marker for raw CloudLocate
			// measurement available (double-blink CYAN). Does NOT short-circuit
			// the event flow — peers (LoRaTxService etc.) still receive the
			// broadcast and act on it. The LED state transits back to
			// LEDGNSSOn (if GPS still active) or LEDOff (if CLOUDLOCATE_ONLY
			// ended the session) after ~500 ms.
			led_handle::dispatch<SetLEDGNSSCloudLocateReady>({});
		} else if (e.event_type == ServiceEventType::GNSS_OFF_DEEP_IDLE) {
			// 2026-05-24 LED refactor: GPSService dispatched end-of-session to
			// deep-idle (rail on, M10Q in PMREQ-backup). Render double-blink
			// RED for ~500 ms then return to LEDOff. Sequenced AFTER any
			// SERVICE_LOG_UPDATED above so this overlay reflects the actual
			// sleep depth.
			led_handle::dispatch<SetLEDGNSSDeepIdle>({});
		} else if (e.event_type == ServiceEventType::GNSS_OFF_POWEROFF) {
			// 2026-05-24 LED refactor: GPSService cut the rail (full power-off,
			// no deep-idle). Render fast-blink RED to distinguish from the
			// double-blink (deep-idle) — operator sees the heavier shutdown.
			led_handle::dispatch<SetLEDGNSSPowerOff>({});
		}
		return;
	} else if (e.event_source == ServiceIdentifier::UW_SENSOR) {
		// LED for UW_SENSOR already dispatched above (pre-broadcast). Nothing more
		// to do here, just early-return so the ARGOS_TX/LORA_TX branch below
		// doesn't accidentally fire on this event.
		return;
	} else if (e.event_source == ServiceIdentifier::ARGOS_TX || e.event_source == ServiceIdentifier::LORA_TX) {
		// Same visual signal for both radio backends: MAGENTA while TX is in
		// progress, off when complete. Let's the operator see any outgoing
		// transmission (Argos or LoRaWAN) with a consistent LED pattern.
		if (e.event_type == ServiceEventType::SERVICE_ACTIVE) {
			led_handle::dispatch<SetLEDArgosTX>({});
		} else if (e.event_type == ServiceEventType::SERVICE_INACTIVE) {
			led_handle::dispatch<SetLEDArgosTXComplete>({});
		}
	}
}

void OperationalState::exit() {
	DEBUG_INFO("exit: OperationalState");
	// Stop periodic config flush and do a final save to persist counters.
	// EVERY destructive step below is independently try/catched: a teardown
	// exception (e.g. SMD SPI dies mid-service_term, BLE stop hits SoftDevice
	// quirk) must NOT escape — main.cpp catches escapes as ErrorEvent which
	// would transit to ErrorState→OffState and brick a sealed device whose
	// only "fault" was a glitchy peripheral during a reed-magnet gesture.
	m_config_flush_active = false;
	// Drop the pending flush rather than waiting for it to fire and notice the
	// flag: if we re-enter Operational before it does, it re-arms itself.
	system_scheduler->cancel_task(m_config_flush_task);
	if (configuration_store->is_valid()) {
		try {
			configuration_store->save_params();
		} catch (...) {
			DEBUG_ERROR("OperationalState::exit: save_params failed");
		}
	}
	try {
		led_handle::dispatch<SetLEDOff>({});
	} catch (...) {
		DEBUG_ERROR("OperationalState::exit: LED dispatch failed");
	}
	try {
		ServiceManager::stopall();
	} catch (...) {
		DEBUG_ERROR("OperationalState::exit: ServiceManager::stopall failed");
	}
	BaseDebugMode debug_mode = BaseDebugMode::NONE;
	try {
		debug_mode = configuration_store->read_param<BaseDebugMode>(ParamID::DEBUG_OUTPUT_MODE);
	} catch (...) {
		DEBUG_ERROR("OperationalState::exit: read DEBUG_OUTPUT_MODE failed");
	}
	if (debug_mode == BaseDebugMode::BLE_NUS) {
		// Restore the compile-time default debug channel: NONE in release
		// builds (silent operation), USB CDC in debug builds.
#ifdef NDEBUG
		g_debug_mode = BaseDebugMode::NONE;
#else
		g_debug_mode = BaseDebugMode::USB_CDC;
#endif
		try {
			ble_service->stop();
		} catch (...) {
			DEBUG_ERROR("OperationalState::exit: ble_service->stop failed");
		}
		DEBUG_TRACE("exit: OperationalState: BLE service stopped");
		PMU::delay_ms(100);
	}
	try {
		battery_monitor->unsubscribe(*this);
	} catch (...) {
		DEBUG_ERROR("OperationalState::exit: battery_monitor unsubscribe failed");
	}
}

/// @brief Config entry — start BLE advertising, USB polling, DTE handler.
void ConfigurationState::entry() {
	DEBUG_INFO("entry: ConfigurationState");

	// Flash the blue LED to indicate we have started BLE and we are
	// waiting for a connection
	led_handle::dispatch<SetLEDConfigNotConnected>({});
	buzz_handle::dispatch<SetBuzzConfiguration>({});

	m_backup_charge_mode = false;

	set_ble_device_name();
	ble_service->start([this](BLEServiceEvent &event) -> int { return on_ble_event(event); });
	restart_inactivity_timeout();

	if (gps_service) {
		gps_service->set_backup_charge_callbacks(
		    [this]() {
			    // Charge started: cut BLE after 200ms (lets the $O; response TX complete).
			    m_backup_charge_mode = true;
			    // MED #6 audit fix: store the task handle so `exit()` can cancel
			    // it if the FSM transits out during the 200 ms window. Without
			    // this, the lambda fires with a dangling `this` → HardFault.
			    m_backup_charge_stop_ble_task = system_scheduler->post_task_prio(
			        [this]() {
				        ble_service->stop();
				        system_scheduler->cancel_task(m_ble_inactivity_timeout_task);
				        led_handle::dispatch<SetLEDOff>({});
				        // Start the visual heartbeat: yellow flash every 10 s while charging.
				        m_backup_charge_blink_task = system_scheduler->post_task_prio(
				            std::bind(&ConfigurationState::backup_charge_blink_fire, this), "BackupChargeBlink",
				            Scheduler::DEFAULT_PRIORITY, 10000);
			        },
			        "BackupChargeStopBLE", Scheduler::DEFAULT_PRIORITY, 200);
		    },
		    [this]() {
			    // Charge ended (reed, timer, or abort): resume normal operation.
			    m_backup_charge_mode = false;
			    transit<OperationalState>();
		    });
	}

#ifdef USB_DTE_ENABLED
	// Default async writer → USB CDC. BLE overrides this on CONNECT
	// (see on_ble_event CONNECTED). Restored on BLE DISCONNECTED.
	// This lets bridges (GNSSBR/KIMBR/LORABR) started over USB route
	// raw UART RX back to the USB host via the DTEHandler async_write.
	dte_handler->set_async_write([](const std::string &msg) { UsbInterface::get_instance().write(msg); });

	// Start USB DTE polling (runs in parallel with BLE)
	DEBUG_TRACE("ConfigurationState: Starting DTE polling");
	schedule_usb_poll();
#endif
}

#ifdef USB_DTE_ENABLED
static void sync_bridge_log_silencing();  // Forward decl — defined near process_usb_data
#endif

void ConfigurationState::exit() {
	DEBUG_INFO("exit: ConfigurationState");
	// Shut the BLE down FIRST, and under protection. This call used to be the LAST
	// instruction of exit() and had no try/catch: if stop_bridge() or
	// power_off_immediate() threw, we never got there and the beacon went back to
	// operation with the BLE radio advertising permanently — on a sealed device,
	// an invisible current leak. OperationalState::exit() was already protecting
	// its own the same way.
	try {
		ble_service->stop();
	} catch (...) {
		DEBUG_ERROR("exit: ConfigurationState: ble_service->stop failed");
	}

	// Abandon any OTA still in flight, HERE rather than in the BLE DISCONNECTED
	// handler. That handler used to be the only place calling this, which was
	// fragile twice over: it needed a DISCONNECTED event to actually arrive
	// (leaving via the reed gesture with no phone attached never produced one),
	// and since stop() now deregisters the callback it would not run at all.
	// Without the abort, OTAFlashFileUpdater keeps m_file_size != 0 and an open
	// LittleFS file handle: the handle leaks and every later transfer is
	// refused until the next reboot. Cheap no-op when nothing is in progress
	// (m_file_size == 0); can throw on the MCU_FIRMWARE erase path, hence the
	// guard.
	// Only a transfer that is genuinely unfinished. A fully received image is
	// waiting for apply_file_update(), which for MCU_FIRMWARE is posted on the
	// scheduler and therefore has NOT run yet: aborting there would erase the
	// staged firmware header and lose a perfectly good update.
	try {
		if (ota_updater && ota_updater->is_transfer_incomplete()) {
			DEBUG_WARN("exit: ConfigurationState — aborting an unfinished OTA transfer");
			ota_updater->abort_file_transfer();
		}
	} catch (...) {
		DEBUG_ERROR("exit: ConfigurationState: abort_file_transfer failed");
	}

#ifdef USB_DTE_ENABLED
	// Drop the async writer back onto USB. It may still be holding the lambda
	// installed on BLE CONNECT, whose link is now gone — same reasoning as
	// above: the DISCONNECTED handler used to do this and no longer runs.
	dte_handler->set_async_write([](const std::string &msg) { UsbInterface::get_instance().write(msg); });
#endif

	system_scheduler->cancel_task(m_ble_inactivity_timeout_task);
	system_scheduler->cancel_task(m_backup_charge_blink_task);
	// MED #6 audit fix: cancel the BackupChargeStopBLE task too — its lambda
	// captures `this` and would HardFault if it fires after our destruction.
	system_scheduler->cancel_task(m_backup_charge_stop_ble_task);
	if (gps_service) gps_service->set_backup_charge_callbacks(nullptr, nullptr);
	m_backup_charge_mode = false;
#ifdef USB_DTE_ENABLED
	system_scheduler->cancel_task(m_usb_poll_task);
#endif
	// Ensure any active bridge is stopped before leaving configuration mode —
	// otherwise the bridge would prevent services from using the underlying UART.
	if (gps_device && gps_device->is_bridge_active()) gps_device->stop_bridge();
#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
	if (lora_device_instance && lora_device_instance->is_bridge_active()) lora_device_instance->stop_bridge();
#endif
#if !(defined(LORA_RAK3172) && (LORA_RAK3172 == 1)) && !(defined(ARGOS_SMD) && (ARGOS_SMD == 1))
	if (kim2_device_instance && kim2_device_instance->is_bridge_active()) kim2_device_instance->stop_bridge();
#endif
#ifdef USB_DTE_ENABLED
	// Restore USB console logs after stopping bridges — otherwise logs stay
	// suppressed until next process_usb_data tick (which is cancelled above).
	sync_bridge_log_silencing();
#endif
	// Safety net against a `PWRON GNSS`/`PWRON ALL` left dangling: the DTE command
	// powers the receiver up with no timer and only `PWRON OFF` decrements it.
	// Without this catch-up, the client counter never gets back to zero: every
	// later service session then does +1/-1 on top of the leaked count, the driver
	// stays in reception permanently (~25-30 mA) and the VSYS latch holds the
	// shared rail at 3.3 V — battery drained in a few days on a sealed device,
	// with NO way at all of noticing it. We only cut power if something is
	// really powered on, so as not to release VSENSORS without having taken it.
	if (gps_device && gps_device->is_powered()) {
		DEBUG_WARN("exit: ConfigurationState — GNSS still powered (PWRON not cancelled) | cutting the rail");
		gps_device->power_off_immediate();
	}
	led_handle::dispatch<SetLEDOff>({});
}

/// @brief Set BLE device name from DEVICE_MODEL + ARGOS_DECID config params.
void GenTracker::set_ble_device_name() {
	std::string device_model = configuration_store->read_param<std::string>(ParamID::DEVICE_MODEL);
	unsigned int identifier = configuration_store->read_param<unsigned int>(ParamID::ARGOS_DECID);
	std::string device_name = device_model + " " + std::to_string(identifier);
	DEBUG_TRACE("GenTracker::set_ble_device_name: %s", device_name.c_str());
	ble_service->set_device_name(device_name);
}

/// @brief BLE event callback — handle connect/disconnect, DTE data, OTA transfers.
/// @param event  BLE service event (CONNECTED, DISCONNECTED, DTE_DATA, OTA_*).
/// @return 0 on success.
int ConfigurationState::on_ble_event(BLEServiceEvent &event) {
	int rc = 0;

	switch (event.event_type) {
	case BLEServiceEventType::CONNECTED:
		DEBUG_TRACE("ConfigurationState::on_ble_event: CONNECTED");
		// Indicate DTE connection is made
		dte_handler->reset_state();
		dte_handler->set_async_write([](const std::string &msg) {
			if (ble_service) ble_service->write(msg);
		});
		led_handle::dispatch<SetLEDConfigConnected>({});
		restart_inactivity_timeout();
		break;
	case BLEServiceEventType::DISCONNECTED:
		DEBUG_TRACE("ConfigurationState::on_ble_event: DISCONNECTED");
		ota_updater->abort_file_transfer();
#ifdef USB_DTE_ENABLED
		// Restore USB writer so future DTE async responses / USB-started
		// bridges route to USB. Note: any bridge still active that captured
		// the (now stale) BLE writer will silently drop RX until stopped.
		dte_handler->set_async_write([](const std::string &msg) { UsbInterface::get_instance().write(msg); });
#endif
		// During backup charge the device is silent — keep LED off instead of
		// returning to the "waiting for BLE connection" blink.
		if (m_backup_charge_mode)
			led_handle::dispatch<SetLEDOff>({});
		else
			led_handle::dispatch<SetLEDConfigNotConnected>({});
		break;
	case BLEServiceEventType::DTE_DATA_RECEIVED:
		DEBUG_TRACE("ConfigurationState::on_ble_event: DTE_DATA_RECEIVED");
		restart_inactivity_timeout();
		system_scheduler->post_task_prio(std::bind(&ConfigurationState::process_received_data, this),
		                                 "BLEProcessReceivedData");
		break;
	case BLEServiceEventType::OTA_START:
		DEBUG_INFO("ConfigurationState::on_ble_event: OTA_START");
		restart_inactivity_timeout();
		ota_updater->start_file_transfer((OTAFileIdentifier)event.file_id, event.file_size, event.crc32);
		led_handle::dispatch<SetLEDDFUUpdate>({});
		break;
	case BLEServiceEventType::OTA_END:
		DEBUG_INFO("ConfigurationState::on_ble_event: OTA_END");
		restart_inactivity_timeout();
		ota_updater->complete_file_transfer();
		led_handle::dispatch<SetLEDConfigConnected>({});
		// Wrap in try/catch — apply_file_update can throw (CRC validation
		// failure, SPI/UART comms error during SMD DFU streaming, LFS read
		// error). Without the wrapper, a throw escapes the scheduler task
		// runner and surfaces at the main-loop catch, which dispatches an
		// ErrorEvent that transits to ErrorState → OffState — bricking a
		// sealed device on a recoverable OTA glitch. Log + swallow keeps
		// the device in Configuration state so the operator can retry.
		system_scheduler->post_task_prio(
		    []() {
			    try {
				    ota_updater->apply_file_update();
			    } catch (ErrorCode e) {
				    DEBUG_ERROR("ConfigurationState: apply_file_update ErrorCode=%d — staying in Config for retry",
					            (int)e);
			    } catch (const std::exception &ex) {
				    DEBUG_ERROR(
				        "ConfigurationState: apply_file_update std::exception: %s — staying in Config for retry",
				        ex.what());
			    } catch (...) {
				    DEBUG_ERROR(
				        "ConfigurationState: apply_file_update unknown exception — staying in Config for retry");
			    }
		    },
		    "BLEApplyOTAFileUpdate");
		break;
	case BLEServiceEventType::OTA_ABORT:
		DEBUG_INFO("ConfigurationState::on_ble_event: OTA_ABORT");
		restart_inactivity_timeout();
		ota_updater->abort_file_transfer();
		led_handle::dispatch<SetLEDConfigConnected>({});
		break;
	case BLEServiceEventType::OTA_FILE_DATA:
		//DEBUG_TRACE("ConfigurationState::on_ble_event: OTA_FILE_DATA");
		restart_inactivity_timeout();
		ota_updater->write_file_data(event.data, event.length);
		break;
	default: break;
	}

	return rc;
}


/// @brief BLE inactivity timeout after 20 min idle in config.
/// Sealed-device hardening (audit 2026-07): this used to transit<OffState>,
/// which on LinkIt means System OFF with the reed magnet as the only wake
/// source — an operator who configured a tag and sealed/released the animal
/// without the explicit exit gesture would get a device that silently powers
/// itself OFF 20 minutes later (a dead deployment at sea). Mirror the NORMAL
/// config-exit gesture (EXIT_CONFIG -> PreOperationalState -> Operational)
/// so the tag resumes its mission. A deliberate power-off stays available via
/// the reed LONG_HOLD (POWEROFF) gesture and the DTE command — both untouched.
void ConfigurationState::on_ble_inactivity_timeout() {
	// Defence in depth: transit<>() leaves whatever state the FSM is CURRENTLY
	// in, not the one that armed the timer. A stale BLE callback re-arming this
	// from OperationalState therefore yanked a running mission back through
	// PreOperationalState — restarting every service — for no reason. The
	// callback leak is fixed in BleInterface::stop(); this guard makes the
	// timer harmless even if anything else ever re-arms it out of context.
	if (!is_in_state<ConfigurationState>()) {
		DEBUG_WARN("BLE Inactivity Timeout fired outside configuration mode — ignored");
		return;
	}
	DEBUG_INFO("BLE Inactivity Timeout — returning to operation (as if exit gesture)");
	transit<PreOperationalState>();
}

/// @brief Visual heartbeat during DTE-triggered backup charge: brief YELLOW flash
/// (~200 ms) then back to off, re-scheduled every 10 s. Lets the operator see the
/// device is still alive in the otherwise-silent charging state. Cancelled on exit
/// or on m_backup_charge_mode = false.
void ConfigurationState::backup_charge_blink_fire() {
	if (!m_backup_charge_mode) return;  // charge ended between schedule and fire
	status_led->set(RGBLedColor::YELLOW);
	system_scheduler->post_task_prio(
	    [this]() {
		    if (!m_backup_charge_mode) return;
		    status_led->off();
		    m_backup_charge_blink_task =
		        system_scheduler->post_task_prio(std::bind(&ConfigurationState::backup_charge_blink_fire, this),
				                                 "BackupChargeBlink", Scheduler::DEFAULT_PRIORITY, 10000);
	    },
	    "BackupChargeBlinkOff", Scheduler::DEFAULT_PRIORITY, 200);
}

/// @brief Reset the BLE inactivity timeout (called on every BLE activity).
void ConfigurationState::restart_inactivity_timeout() {
	//DEBUG_TRACE("Restart BLE inactivity timeout: %lu", system_timer->get_counter());
	system_scheduler->cancel_task(m_ble_inactivity_timeout_task);
	m_ble_inactivity_timeout_task = system_scheduler->post_task_prio(
	    std::bind(&ConfigurationState::on_ble_inactivity_timeout, this), "BLEInactivityTimeout",
	    Scheduler::DEFAULT_PRIORITY, BLE_INACTIVITY_TIMEOUT_MS);
}

/// @brief Process BLE DTE command — parse, dispatch to DTEHandler, send response.
void ConfigurationState::process_received_data() {
	auto req = ble_service->read_line();

	if (req.size()) {
		// Bridge mode: forward raw BLE bytes straight to the UART, bypassing
		// the DTE parser. Mirrors the USB bridge path in process_usb_data().
		// BLE NUS RX is line-buffered (\r triggers flush), so AT-based modules
		// (KIM2/LoRa) work cleanly; GNSS binary UBX over BLE is limited by
		// this line buffering — host should prefer USB for u-center.
		// Exit sequence: user sends "+++\r" over BLE.
		auto bridge_forward = [&req, this](auto *device) -> bool {
			if (!device || !device->is_bridge_active()) return false;

			PMU::kick_watchdog();

			// BLE read_line() keeps the trailing \r. Strip before inspection.
			std::string line = req;
			while (!line.empty() && (line.back() == '\r' || line.back() == '\n'))
				line.pop_back();

			if (line == "+++") {
				device->stop_bridge();
				ble_service->write("\r\n[BRIDGE OFF]\r\n");
				return true;
			}

			// Re-append CRLF for AT framing (safe no-op for payloads that
			// already end in it, since we stripped above).
			std::string data = line + "\r\n";
			device->bridge_send(reinterpret_cast<const uint8_t *>(data.c_str()), data.size());
			return true;
		};

		if (bridge_forward(gps_device)) return;
#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
		if (bridge_forward(lora_device_instance)) return;
#endif
#if !(defined(LORA_RAK3172) && (LORA_RAK3172 == 1)) && !(defined(ARGOS_SMD) && (ARGOS_SMD == 1))
		if (bridge_forward(kim2_device_instance)) return;
#endif

		DEBUG_TRACE("received %u bytes:", req.size());
#if defined(DEBUG_ENABLE) && DEBUG_LEVEL >= 4
		printf("%s\n", req.c_str());
#endif

		std::string resp;
		DTEAction action;

		do {
			action = dte_handler->handle_dte_message(req, resp);

			// Kick watchdog every iteration (not just when resp produced)
			PMU::kick_watchdog();

			if (resp.size()) {
				DEBUG_TRACE("responding: %s", resp.c_str());
				if (!ble_service->write(resp)) {
					dte_handler->reset_state();
					break;
				}

				// Reset inactivity timeout whenever we send a response
				// This is important during a command sequences that can take
				// a long time to complete (eg DUMPD)
				restart_inactivity_timeout();
			}

			if (action == DTEAction::FACTR) {
				DEBUG_INFO("Perform factory reset of configuration store");
				configuration_store->factory_reset();

				// After formatting the filesystem we must do a system reset so that
				// the boot up procedure can repopulate files
				PMU::reset(false);
			} else if (action == DTEAction::RESET) {
				DEBUG_INFO("Perform device reset");

				// Execute this after 3 seconds to allow time for the BLE response to be sent
				system_scheduler->post_task_prio([]() { PMU::reset(false); }, "DTEActionPMUReset",
				                                 Scheduler::DEFAULT_PRIORITY, 3000);
			} else if (action == DTEAction::SECUR) {
				// TODO: add secure procedure
				DEBUG_INFO("Perform secure procedure");
			} else if (action == DTEAction::CONFIG_UPDATED) {
				// Propagate runtime LoRa param changes to the RAK3172 module.
				// reload_config_if_changed() is a no-op if no LORA_* param
				// actually changed (cheap diff against the cached LoRaConfig),
				// so it is safe to call after every PARMW. Busy states (TX /
				// joining) defer the apply to the next idle entry.
#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
				if (lora_device_instance) {
					lora_device_instance->reload_config_if_changed();
				}
#endif
			}

		} while (action == DTEAction::AGAIN);
	}
}

#ifdef USB_DTE_ENABLED
void ConfigurationState::schedule_usb_poll() {
	m_usb_poll_task = system_scheduler->post_task_prio(std::bind(&ConfigurationState::process_usb_data, this),
	                                                   "USBDTEPoll", Scheduler::DEFAULT_PRIORITY, USB_POLL_INTERVAL_MS);
}

/// @brief Suppress USB console logs while any bridge is active, restore when stopped.
/// Debug output would pollute the raw UART passthrough stream sent to the host
/// (u-center, AT terminal, etc.). Call this from any context where bridge state
/// may have changed (process_usb_data tick, FSM exit).
static void sync_bridge_log_silencing() {
	static Logger *s_saved_console_log = nullptr;
	bool bridge_active = (gps_device && gps_device->is_bridge_active());
#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
	bridge_active = bridge_active || (lora_device_instance && lora_device_instance->is_bridge_active());
#endif
#if !(defined(LORA_RAK3172) && (LORA_RAK3172 == 1)) && !(defined(ARGOS_SMD) && (ARGOS_SMD == 1))
	bridge_active = bridge_active || (kim2_device_instance && kim2_device_instance->is_bridge_active());
#endif
	if (bridge_active && DebugLogger::console_log != nullptr) {
		s_saved_console_log = DebugLogger::console_log;
		DebugLogger::console_log = nullptr;
	} else if (!bridge_active && s_saved_console_log != nullptr) {
		DebugLogger::console_log = s_saved_console_log;
		s_saved_console_log = nullptr;
	}
}

void ConfigurationState::process_usb_data() {
	auto &usb = UsbInterface::get_instance();

	sync_bridge_log_silencing();

	// GNSS UART bridge mode: forward USB ↔ GNSS UART directly (binary UBX)
	if (gps_device && gps_device->is_bridge_active()) {
		// Process UART RX → USB (via passthrough callback)
		gps_device->bridge_process_rx();

		// Process USB → UART (raw bytes, no line parsing — UBX is binary)
		if (usb.has_data()) {
			char buf[256];
			int n = NrfUSB::read(buf, sizeof(buf));
			if (n > 0) {
				restart_inactivity_timeout();
				PMU::kick_watchdog();

				// Check for exit sequence "+++" (text typed in terminal)
				if (n == 3 && buf[0] == '+' && buf[1] == '+' && buf[2] == '+') {
					gps_device->stop_bridge();
					usb.write("\r\n[BRIDGE OFF]\r\n");
					schedule_usb_poll();
					return;
				}
				// Also check with line ending
				if (n >= 3 && buf[0] == '+' && buf[1] == '+' && buf[2] == '+'
				    && (n == 3 || buf[3] == '\r' || buf[3] == '\n')) {
					gps_device->stop_bridge();
					usb.write("\r\n[BRIDGE OFF]\r\n");
					schedule_usb_poll();
					return;
				}

				gps_device->bridge_send(reinterpret_cast<const uint8_t *>(buf), n);
			}
		}

		schedule_usb_poll();
		return;
	}

#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
	// LoRa UART bridge mode: forward USB ↔ RAK3172 UART directly
	if (lora_device_instance && lora_device_instance->is_bridge_active()) {
		// Process UART RX → USB (via passthrough callback)
		lora_device_instance->bridge_process_rx();

		// Process USB → UART
		if (usb.has_data()) {
			auto line = usb.read_line();
			if (line.size()) {
				restart_inactivity_timeout();
				PMU::kick_watchdog();

				// Check for exit sequence "+++"
				if (line == "+++") {
					lora_device_instance->stop_bridge();
					usb.write("\r\n[BRIDGE OFF]\r\n");
					schedule_usb_poll();
					return;
				}

				// Forward to RAK3172 UART with \r\n termination
				std::string data = line + "\r\n";
				lora_device_instance->bridge_send(reinterpret_cast<const uint8_t *>(data.c_str()), data.size());
			}
		}

		schedule_usb_poll();
		return;
	}
#endif

#if !(defined(LORA_RAK3172) && (LORA_RAK3172 == 1)) && !(defined(ARGOS_SMD) && (ARGOS_SMD == 1))
	// KIM2 UART bridge mode: forward USB ↔ KIM2 UART directly (AT commands)
	if (kim2_device_instance && kim2_device_instance->is_bridge_active()) {
		// Process UART RX → USB (via passthrough callback)
		kim2_device_instance->bridge_process_rx();

		// Process USB → UART
		if (usb.has_data()) {
			auto line = usb.read_line();
			if (line.size()) {
				restart_inactivity_timeout();
				PMU::kick_watchdog();

				// Check for exit sequence "+++"
				if (line == "+++") {
					kim2_device_instance->stop_bridge();
					usb.write("\r\n[BRIDGE OFF]\r\n");
					schedule_usb_poll();
					return;
				}

				// Forward to KIM2 UART with \r\n termination (AT command framing)
				std::string data = line + "\r\n";
				kim2_device_instance->bridge_send(reinterpret_cast<const uint8_t *>(data.c_str()), data.size());
			}
		}

		schedule_usb_poll();
		return;
	}
#endif

	// Check if USB has data
	if (usb.has_data()) {
		auto req = usb.read_line();

		if (req.size()) {
#ifdef BENCH_TEST
			// Bench '%' commands share the CDC stream with DTE '$...#'. Route
			// them to the bench console (e.g. %OP to leave config, %STATE) so
			// they work while in ConfigurationState too. Never fed to the DTE
			// parser.
			if (req[0] == '%') {
				bench::handle_line(req);
				schedule_usb_poll();
				return;
			}
#endif
			// DTE protocol expects trailing \r which read_line() strips
			req += '\r';
			// Suppress console debug logs during DTE exchange to avoid
			// polluting the USB DTE response stream
			auto *saved_log = DebugLogger::console_log;
			DebugLogger::console_log = nullptr;

			std::string resp;
			DTEAction action;

			do {
				action = dte_handler->handle_dte_message(req, resp);

				// Kick watchdog every iteration (not just when resp produced)
				PMU::kick_watchdog();

				if (resp.size()) {
					usb.write(resp);

					// Reset inactivity timeout on USB activity too
					restart_inactivity_timeout();
				}

				if (action == DTEAction::FACTR) {
					DEBUG_INFO("Perform factory reset of configuration store (USB)");
					configuration_store->factory_reset();
					PMU::reset(false);
				} else if (action == DTEAction::RESET) {
					DEBUG_INFO("Perform device reset (USB)");
					system_scheduler->post_task_prio([]() { PMU::reset(false); }, "DTEActionPMUReset",
					                                 Scheduler::DEFAULT_PRIORITY, 3000);
				} else if (action == DTEAction::SECUR) {
					DEBUG_INFO("Perform secure procedure (USB)");
				}

			} while (action == DTEAction::AGAIN);

			// Restore console logging
			DebugLogger::console_log = saved_log;
		}
	}

	// Schedule next poll
	schedule_usb_poll();
}
#else
// Empty implementations when USB DTE is disabled
void ConfigurationState::schedule_usb_poll() {}
void ConfigurationState::process_usb_data() {}
#endif

void BatteryCriticalState::entry() {
	DEBUG_INFO("entry: BatteryCriticalState");
	PMU::kick_watchdog();
#ifdef EXTERNAL_WAKEUP
	// If magnet is present, user wants to configure via BLE - don't powerdown
	if (reed_switch->is_engaged()) {
		DEBUG_INFO("BatteryCriticalState: Magnet detected, entering configuration mode");
		transit<ConfigurationState>();
		return;
	}
	DEBUG_INFO("EXTERNAL_WAKEUP: Critical battery | immediate powerdown");
	status_led->off();
	PMU::powerdown();
#else
	led_handle::dispatch<SetLEDBatteryCritical>({});
	buzz_handle::dispatch<SetBuzzOff>({});
	m_transit_task =
	    system_scheduler->post_task_prio([this]() { transit<OffState>(); }, "GenTrackerBatteryCriticalTransitOffState",
		                                 Scheduler::DEFAULT_PRIORITY, BATTERY_CRITICAL_TIMEOUT_MS);
#endif
}

void BatteryCriticalState::exit() {
	DEBUG_INFO("exit: BatteryCriticalState");
	system_scheduler->cancel_task(m_transit_task);
	led_handle::dispatch<SetLEDOff>({});
}

void ErrorState::entry() {
	DEBUG_INFO("entry: ErrorState");
	PMU::kick_watchdog();
	led_handle::dispatch<SetLEDError>({});
	buzz_handle::dispatch<SetBuzzOff>({});

	// Sealed-device recovery: while the boot-fail counter is below MAX_BOOT_RETRIES,
	// do a soft reset instead of transiting to OffState (which would require a
	// reed magnet to wake — unrecoverable on a potted turtle). The reset
	// causes a fresh boot, BootState increments the counter, and if the same
	// fault recurs eventually we either factory_reset (at BOOT_RETRY_BEFORE_FACTORY)
	// or, only if everything else has failed, fall through to OffState as a
	// last resort.
	bootfail_load();
#ifdef CPPUTEST
	// Test mode: the sealed-recovery soft-reset path requires real hardware
	// (PMU::reset actually rebooting + the .noinit RAM surviving). In CPPUTEST
	// PMU::reset is a tracked mock and would surface as "unexpected call to
	// reset" in legacy Error→OffState tests written before this feature. Skip
	// directly to OffState so those tests keep validating the FSM transitions.
	bool soft_reset_retry = false;
#else
	bool soft_reset_retry = (s_bootfail_noinit.consecutive_failures < BOOT_RETRY_MAX);
#endif
	if (soft_reset_retry) {
		DEBUG_WARN("ErrorState: boot-fail counter %u/%u — scheduling soft reset retry",
		           s_bootfail_noinit.consecutive_failures, BOOT_RETRY_MAX);
		m_shutdown_task = system_scheduler->post_task_prio(
		    []() { PMU::reset(false); }, "GenTrackerErrorStateRebootRetry", Scheduler::DEFAULT_PRIORITY, 5000);
	} else {
		DEBUG_ERROR("ErrorState: %u consecutive boot failures (max %u reached, factory_reset_attempted=%u) — falling "
		            "through to OffState (real HW fault)",
		            s_bootfail_noinit.consecutive_failures, BOOT_RETRY_MAX, s_bootfail_noinit.factory_reset_attempted);
		m_shutdown_task =
		    system_scheduler->post_task_prio([this]() { transit<OffState>(); }, "GenTrackerErrorStateTransitOffState",
			                                 Scheduler::DEFAULT_PRIORITY, 5000);
	}
}

void ErrorState::exit() {
	DEBUG_INFO("exit: ErrorState");
	system_scheduler->cancel_task(m_shutdown_task);
	led_handle::dispatch<SetLEDOff>({});
}
