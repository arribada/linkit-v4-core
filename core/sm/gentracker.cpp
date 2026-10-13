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
#ifndef EXTERNAL_WAKEUP
	// Re-entering would restart the wait, and errors arriving faster than the
	// backoff would then postpone the retry reset for ever.
	if (is_in_state<ErrorState>()) return;
#endif
	cancel_confirmation();
	transit<ErrorState>();
}

void GenTracker::entry(void) {};
void GenTracker::exit(void) {};

// =====================================================================
// Boot-fail counter — sealed-device recovery infrastructure.
//
// A boot that never reaches OperationalState or ConfigurationState counts as
// failed. Both states prove the same things: the flash mounted, the config
// store parsed, the scheduler runs and the DTE answers. Clearing in
// configuration too keeps an operator's back-to-back USB sessions from
// walking the tag into a factory reset it never earned. The record lives in
// .noinit RAM under a CRC: it survives a reset, and a cold power-up starts
// it from zero.
//
// LinkIt: OffState is System OFF, which only the magnet wakes — the end of
// a glued tag. Boot failures therefore never lead there. ErrorState sleeps
// for a delay that grows with the count (BOOT_BACKOFF_S), then resets to
// retry, for ever. The one factory_reset waits for a failure that is still
// there after FACTORY_RESET_MIN_BACKOFF_S of those delays, so a transient
// fault (cold flash, brown-outs) cannot cost the mission configuration. A
// boot that follows an unplanned reset (crash, watchdog, brown-out) is
// spaced out the same way, so a crash loop neither drains the battery nor
// skips the delays.
//
// RSPB (EXTERNAL_WAKEUP): OffState is a TPL5111 power cut and the timer
// powers the board up again, so the original policy stands: a few quick
// retries, one factory_reset, then OffState.
//
// factory_reset keeps PROTECTED_PARAMS (config_store_fs.hpp): identifiers,
// calibration and the provisioning secrets (ARGOS_SECKEY, the RADIOCONFs,
// the LoRaWAN keys). It loses the mission config — modes, periods,
// thresholds — which is what the recovery is for.
// =====================================================================

struct BootFailNoinit {
	uint16_t consecutive_failures;
	uint8_t factory_reset_attempted;
	uint8_t reset_planned;      ///< 1: the previous boot ended in ErrorState's own retry reset
	uint32_t backoff_waited_s;  ///< ErrorState waits completed since the last good boot
	uint16_t crc;
};

#ifndef CPPUTEST
static __attribute__((section(".noinit"))) volatile BootFailNoinit s_bootfail_noinit;
#else
static BootFailNoinit s_bootfail_noinit;
#endif

#ifdef EXTERNAL_WAKEUP
static constexpr uint8_t BOOT_RETRY_MAX = 5;             ///< Total reset retries before OffState
static constexpr uint8_t BOOT_RETRY_BEFORE_FACTORY = 5;  ///< Retries before attempting factory_reset
#else
/// ErrorState wait before the retry reset, by failure count. The last one repeats for ever.
static constexpr uint32_t BOOT_BACKOFF_S[] = { 5, 30, 2 * 60, 10 * 60, 60 * 60 };
/// factory_reset needs both: enough failed boots, and enough time spent failing.
static constexpr uint16_t FACTORY_RESET_MIN_FAILURES = 5;
static constexpr uint32_t FACTORY_RESET_MIN_BACKOFF_S = 30 * 60;
/// A boot after an unplanned reset waits first from this count on. Lower, an
/// operator's double reset, or a magnet still held across a power-off, would
/// cost a wait.
static constexpr uint16_t UNPLANNED_RESET_BACKOFF_FAILURES = 3;
#endif

static uint16_t bootfail_compute_crc() {
	BootFailNoinit snapshot;
	snapshot.consecutive_failures = s_bootfail_noinit.consecutive_failures;
	snapshot.factory_reset_attempted = s_bootfail_noinit.factory_reset_attempted;
	snapshot.reset_planned = s_bootfail_noinit.reset_planned;
	snapshot.backoff_waited_s = s_bootfail_noinit.backoff_waited_s;
	return crc16_compute(reinterpret_cast<const uint8_t *>(&snapshot), offsetof(BootFailNoinit, crc), nullptr);
}

static void bootfail_save() {
	s_bootfail_noinit.crc = bootfail_compute_crc();
}

static void bootfail_clear() {
	s_bootfail_noinit.consecutive_failures = 0;
	s_bootfail_noinit.factory_reset_attempted = 0;
	s_bootfail_noinit.reset_planned = 0;
	s_bootfail_noinit.backoff_waited_s = 0;
	bootfail_save();
}

/// @brief Load the boot-fail counter from .noinit RAM. On cold POR (battery
/// insert) the RAM is uninitialised and the CRC will not validate — we
/// reset to zero in that case.
static void bootfail_load() {
	if (s_bootfail_noinit.crc != bootfail_compute_crc()) bootfail_clear();
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
/// Exposed because this counter is on the path to a factory reset, which
/// discards the mission configuration. If it ever stopped clearing on a
/// healthy boot, ordinary reboots would add up towards one -- and nothing
/// outside .noinit RAM would show it coming.
void bench_bootfail_read(unsigned int &failures, unsigned int &factory_attempted) {
	bootfail_load();
	failures = s_bootfail_noinit.consecutive_failures;
	factory_attempted = s_bootfail_noinit.factory_reset_attempted;
}
#endif

#ifdef CPPUTEST
static bool s_bootfail_test_keep = false;

void gentracker_test_bootfail_keep(bool keep) {
	s_bootfail_test_keep = keep;
}

void gentracker_test_bootfail_set(const BootFailTestState &state) {
	s_bootfail_noinit.consecutive_failures = static_cast<uint16_t>(state.failures);
	s_bootfail_noinit.factory_reset_attempted = state.factory_reset_attempted ? 1 : 0;
	s_bootfail_noinit.reset_planned = state.reset_planned ? 1 : 0;
	s_bootfail_noinit.backoff_waited_s = state.backoff_waited_s;
	bootfail_save();
}

BootFailTestState gentracker_test_bootfail_get() {
	bootfail_load();
	return { s_bootfail_noinit.consecutive_failures, s_bootfail_noinit.factory_reset_attempted != 0,
	         s_bootfail_noinit.backoff_waited_s, s_bootfail_noinit.reset_planned != 0 };
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
	bootfail_clear();
}

/// @brief True when this boot should make the one factory_reset attempt.
static bool bootfail_factory_reset_due() {
	if (s_bootfail_noinit.factory_reset_attempted) return false;
#ifdef EXTERNAL_WAKEUP
	return s_bootfail_noinit.consecutive_failures >= BOOT_RETRY_BEFORE_FACTORY;
#else
	return s_bootfail_noinit.consecutive_failures >= FACTORY_RESET_MIN_FAILURES
	       && s_bootfail_noinit.backoff_waited_s >= FACTORY_RESET_MIN_BACKOFF_S;
#endif
}

/// @brief The one factory_reset attempt, then a reset to boot on it.
static void bootfail_factory_reset_and_reboot() {
	DEBUG_ERROR("BootFail: %u failed boots — attempting factory_reset, once, before retrying",
	            s_bootfail_noinit.consecutive_failures);
	s_bootfail_noinit.factory_reset_attempted = 1;
	// The retries after it start from the shortest delay again (and, on RSPB,
	// with a full budget before OffState). The flag alone keeps the
	// factory_reset itself from repeating.
	s_bootfail_noinit.consecutive_failures = 0;
	s_bootfail_noinit.backoff_waited_s = 0;
	bootfail_save();
	// factory_reset() writes the full default param block + RTC to LFS.
	// On a degraded flash that's the slowest operation here; kick the WDT
	// first so its whole budget is available, otherwise a flash erase retry
	// chain could exhaust it mid-reset, on the very path meant to recover.
	PMU::kick_watchdog();
	try {
		if (configuration_store) configuration_store->factory_reset();
	} catch (...) {
		DEBUG_ERROR("BootFail: factory_reset itself threw — proceeding to reboot anyway");
	}
	PMU::kick_watchdog();
	PMU::delay_ms(100);  // Let logs flush
	PMU::reset(false);
}

#ifndef EXTERNAL_WAKEUP
/// @brief ErrorState wait for this failure count. 0 means the error came after
/// a good boot, and gets the shortest wait.
static uint32_t bootfail_backoff_s(uint16_t failures) {
	constexpr size_t steps = sizeof(BOOT_BACKOFF_S) / sizeof(BOOT_BACKOFF_S[0]);
	const size_t i = (failures == 0) ? 0 : failures - 1u;
	return BOOT_BACKOFF_S[(i < steps) ? i : steps - 1];
}

/// @brief Read, and clear, the mark ErrorState leaves before its retry reset.
static bool bootfail_take_retry_reset_mark() {
	bootfail_load();
	const bool planned = s_bootfail_noinit.reset_planned != 0;
	s_bootfail_noinit.reset_planned = 0;
	bootfail_save();
	return planned;
}

/// @brief Record a completed ErrorState wait, right before its retry reset.
static void bootfail_note_retry_reset(uint32_t waited_s) {
	bootfail_load();
	const uint32_t total = s_bootfail_noinit.backoff_waited_s + waited_s;
	s_bootfail_noinit.backoff_waited_s = (total < waited_s) ? UINT32_MAX : total;
	s_bootfail_noinit.reset_planned = 1;
	bootfail_save();
}
#endif

/// @brief Clear the boot-fail counter from another translation unit.
///
/// ConfigurationState lives in gentracker_config_state.cpp and needs the same
/// clear as OperationalState. Kept as a thin wrapper so the counter and its
/// CRC stay private to this file.
void gentracker_bootfail_reset() {
	bootfail_reset();
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

/// @brief Set BLE device name from DEVICE_MODEL + ARGOS_DECID config params.
void GenTracker::set_ble_device_name() {
	std::string device_model = configuration_store->read_param<std::string>(ParamID::DEVICE_MODEL);
	unsigned int identifier = configuration_store->read_param<unsigned int>(ParamID::ARGOS_DECID);
	std::string device_name = device_model + " " + std::to_string(identifier);
	DEBUG_TRACE("GenTracker::set_ble_device_name: %s", device_name.c_str());
	ble_service->set_device_name(device_name);
}

/// @brief Boot entry — init PMU, check reset cause, mount filesystem, load config.
void BootState::entry() {
	DEBUG_INFO("entry: BootState");

#ifdef CPPUTEST
	// A plain static here, not .noinit RAM: without this every test would
	// inherit the previous one's count. Tests that simulate resets keep it.
	if (!s_bootfail_test_keep) bootfail_clear();
#endif

	// Counted as failed until OperationalState or ConfigurationState clears it.
#ifndef EXTERNAL_WAKEUP
	const bool after_retry_reset = bootfail_take_retry_reset_mark();
#endif
	bootfail_increment();
#ifdef EXTERNAL_WAKEUP
	if (bootfail_factory_reset_due()) {
		bootfail_factory_reset_and_reboot();
		return;  // unreachable on target; the host-test mock does return
	}
#endif

	// Ensure the system timer is started to allow scheduling to work
	system_timer->start();

	// Turn status LED white to indicate boot up
	led_handle::start();
	led_handle::dispatch<SetLEDBoot>({});
	buzz_handle::start();

#ifndef EXTERNAL_WAKEUP
	// The previous failed boot died (crash, watchdog, brown-out) instead of
	// ending in ErrorState's retry: wait as ErrorState would before touching
	// the flash again, or a crash loop would run flat out and never back off.
	if (!after_retry_reset && s_bootfail_noinit.consecutive_failures >= UNPLANNED_RESET_BACKOFF_FAILURES) {
		DEBUG_WARN("BootState: %u failed boots, the last one without a retry reset — backing off first",
		           s_bootfail_noinit.consecutive_failures);
		system_scheduler->post_task_prio([this]() { transit<ErrorState>(); }, "GenTrackerBootStateBackoff");
		return;
	}
#endif

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
		// Raw registers, not just the decoded enum: a field return is diagnosed
		// by reading this line. Bit 16 = OFF (System OFF wake), 20 = VBUS,
		// 18 = DIF (debugger), 3 = LOCKUP, 2 = SREQ, 1 = DOG, 0 = RESETPIN.
		DEBUG_INFO("reset cause: %s (RESETREAS=0x%08x GPREGRET=0x%02x)", PMU::reset_cause_str(),
		           (unsigned int)PMU::reset_reason_raw(), (unsigned int)PMU::boot_gpregret());
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

	// Start periodic config flush (counters → flash every 30 min)
	m_config_flush_active = true;
	periodic_config_flush();

	battery_monitor->subscribe(*this);
	led_handle::dispatch<SetLEDOff>({});
	buzz_handle::dispatch<SetBuzzOff>({});

	ServiceManager::startall([this](ServiceEvent &e) { service_event_handler(e); });

	try {
		BaseDebugMode debug_mode = configuration_store->read_param<BaseDebugMode>(ParamID::DEBUG_OUTPUT_MODE);
		if (debug_mode == BaseDebugMode::BLE_NUS) {
			set_ble_device_name();
			ble_service->start([](BLEServiceEvent &) { return 0; });
			g_debug_mode = debug_mode;
		}
	} catch (...) {
		// Optional debug output: not worth a reboot.
		DEBUG_ERROR("OperationalState::entry: BLE debug output not started");
	}

	// Proof of a good boot, so cleared last: a throw above leaves the count
	// running, and a deterministic fault escalates through ErrorState instead
	// of looping on the shortest retry for ever.
	bootfail_reset();
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
	// Re-sample before committing. OffState is a one-way door on a sealed device
	// -- the reed under the glue can never restart it -- and the reading that put
	// us here may have been a transient: an Argos TX current spike, or a sample
	// taken before the pack recovered. battery_monitor->is_battery_critical() is
	// the CONFIRMED verdict (3 consecutive samples, nrf_battery_mon.cpp:349), so
	// if it has cleared during these two minutes the tag goes back to work
	// instead of ending its mission on one bad reading.
	m_transit_task = system_scheduler->post_task_prio(
	    [this]() {
		    if (battery_monitor && !battery_monitor->is_battery_critical()) {
			    DEBUG_INFO("BatteryCriticalState: battery no longer critical on re-check — resuming operation");
			    transit<OperationalState>();
			    return;
		    }
		    transit<OffState>();
	    },
	    "GenTrackerBatteryCriticalTransitOffState", Scheduler::DEFAULT_PRIORITY, BATTERY_CRITICAL_TIMEOUT_MS);
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

	bootfail_load();
#ifndef EXTERNAL_WAKEUP
	// Decided here, not in BootState: only now has the fault shown it outlived
	// the waits behind it. Deciding at boot would discard the configuration
	// without first checking whether the last wait had cured it.
	if (bootfail_factory_reset_due()) {
		bootfail_factory_reset_and_reboot();
		return;  // unreachable on target; the host-test mock does return
	}
	// Never OffState: on LinkIt only the magnet wakes it, and a glued tag has
	// none. Sleep through a delay that grows with the failures, then reset and
	// boot again. See the boot-fail counter above.
	m_backoff_s = bootfail_backoff_s(s_bootfail_noinit.consecutive_failures);
	m_backoff_left_ms = m_backoff_s * 1000;
	DEBUG_WARN("ErrorState: boot-fail counter %u — scheduling soft reset retry in %u s",
	           s_bootfail_noinit.consecutive_failures, m_backoff_s);
	backoff_wait(ERROR_LED_MS);
#else
	// RSPB: a few soft-reset retries, then OffState, which the TPL5111 powers
	// up again at its next period.
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
#endif
}

#ifndef EXTERNAL_WAKEUP
/// @brief Wait the next slice of the backoff, at most max_step_ms long.
void ErrorState::backoff_wait(unsigned int max_step_ms) {
	const unsigned int step_ms = (m_backoff_left_ms < max_step_ms) ? m_backoff_left_ms : max_step_ms;
	m_backoff_left_ms -= step_ms;
	m_shutdown_task = system_scheduler->post_task_prio([this]() { backoff_tick(); }, "GenTrackerErrorStateBackoff",
	                                                   Scheduler::DEFAULT_PRIORITY, step_ms);
}

/// @brief End of a slice: feed the watchdog, then wait on or reset to retry.
void ErrorState::backoff_tick() {
	PMU::kick_watchdog();
	if (m_backoff_left_ms == 0) {
		DEBUG_INFO("ErrorState: waited %u s — rebooting to retry", m_backoff_s);
		bootfail_note_retry_reset(m_backoff_s);
		PMU::reset(false);
		return;
	}
	// Nobody sees the LED of a glued tag: dark for the rest of the wait.
	led_handle::dispatch<SetLEDOff>({});
	// Slices well inside the watchdog period, so the wait feeds it by itself
	// (nothing else may be scheduled when the boot failed early).
	backoff_wait(BSP::WDT_Inits[BSP::WDT].config.reload_value / 2);
}
#endif

void ErrorState::exit() {
	DEBUG_INFO("exit: ErrorState");
	system_scheduler->cancel_task(m_shutdown_task);
	led_handle::dispatch<SetLEDOff>({});
}
