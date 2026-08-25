/**
 * @file main.cpp
 * @brief Application entry point — 8-phase init, FSM start, infinite scheduler loop.
 *
 * Init phases: peripherals → power-on check → storage → battery → core services
 * → communication (SWS + satellite/LoRa + GPS) → sensors → runtime services.
 */

// --- Nordic SDK ---
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log_redirect.h"
#include "nrfx_spim.h"

// --- BSP & GPIO ---
#include "bsp.hpp"
#include "gpio.hpp"
#include "gpio_buzzer.hpp"

// --- Core system (FSM, scheduler, config, debug, RTC, timer) ---
#include "gentracker.hpp"
#include "config_store_fs.hpp"
#include "debug.hpp"
#include "console_log.hpp"
#include "nrf_rtc.hpp"
#include "nrf_timer.hpp"
#include "nrf_switch.hpp"
#include "reed.hpp"
#include "nrf_rgb_led.hpp"
#include "nrf_i2c.hpp"
#include "nrf_usb.hpp"
#include "nrf_memory_access.hpp"
#include "is25_flash.hpp"
#include "fs_log.hpp"
#include "sys_log.hpp"
#include "dte_handler.hpp"
#include "ble_interface.hpp"
#include "ota_flash_file_updater.hpp"
#ifdef DEBUG_UART_TX_PIN
#include "nrf_debug_uart.hpp"
#endif
#ifdef BENCH_TEST
#include "bench_console.hpp"
#endif

// --- Battery monitor ---
#if defined(BATTERY_MONITOR_ANALOG)
#include "nrf_battery_mon.hpp"
#elif defined(BATTERY_MONITOR_FAKE)
#include "fake_battery_mon.hpp"
#elif defined(BATTERY_MONITOR_STC3117)
#include "stc3117_gasgauge.hpp"
#endif

// --- Communication backends (mutually exclusive) ---
#include "m10qasync.hpp"
#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
#include "lora_rak3172.hpp"
#elif defined(ARGOS_SMD) && (ARGOS_SMD == 1)
#include "smd_sat.hpp"
#if defined(SMD_UART) && (SMD_UART == 1)
#include "smd_sat_cmd_at.hpp"
#else
#include "smd_sat_cmd_spi.hpp"
#endif
#else
#include "kim2.hpp"
#endif

// --- Sensor drivers ---
#if ENABLE_ALS_SENSOR
#include "ltr_303.hpp"
#endif
#if ENABLE_PH_SENSOR
#include "oem_ph.hpp"
#endif
#if ENABLE_SEA_TEMP_SENSOR
#include "oem_rtd.hpp"
#include "ezo_rtd.hpp"
#include "TSYS01.hpp"
#endif
#if ENABLE_CDT_SENSOR || ENABLE_PRESSURE_SENSOR
#include "ms58xx.hpp"
#include "bar100.hpp"
#include "lps28dfw.hpp"
#endif
#if ENABLE_CDT_SENSOR
#include "cdt.hpp"
#include "ad5933.hpp"
#endif
#if ENABLE_AXL_SENSOR
#include "bma400.hpp"
#endif
#if ENABLE_THERMISTOR_SENSOR
#include "thermistor.hpp"
#endif
#ifdef CAM_PWR_EN
#include "runcam.hpp"
#endif

// --- Services ---
#include "gps_service.hpp"
#include "argos_tx_service.hpp"
#include "argos_rx_service.hpp"
#include "memory_monitor_service.hpp"
#include "dive_mode_service.hpp"
#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
#include "lora_tx_service.hpp"
#endif
#if ENABLE_SWS_ANALOG
#include "sws_analog_service.hpp"
#endif
#if ENABLE_PRESSURE_SENSOR
#include "pressure_sensor_service.hpp"
#endif
#if ENABLE_ALS_SENSOR
#include "als_sensor_service.hpp"
#endif
#if ENABLE_PH_SENSOR
#include "ph_sensor_service.hpp"
#endif
#if ENABLE_SEA_TEMP_SENSOR
#include "sea_temp_sensor_service.hpp"
#endif
#if ENABLE_CDT_SENSOR
#include "cdt_sensor_service.hpp"
#endif
#if ENABLE_AXL_SENSOR
#include "axl_sensor_service.hpp"
#endif
#if ENABLE_THERMISTOR_SENSOR
#include "thermistor_sensor_service.hpp"
#endif
#include "cam_service.hpp"
#if ENABLE_MORTALITY_SENSOR
#include "mortality_service.hpp"
#endif

// --- Utilities ---
#include <algorithm>
#include <cstddef>
#include <cstring>
#include "heap.h"
#include "etl/error_handler.h"

/// @name Global peripheral pointers
/// @brief Assigned during init phases, valid for the entire application lifetime.
///        All point to static objects allocated in .bss (not heap).
/// @{
FileSystem *main_filesystem;              ///< LittleFS filesystem on IS25 external flash
ConfigurationStore *configuration_store;  ///< Parameter persistence (206 params via LFS)
BLEService *ble_service;                  ///< BLE NUS interface for DTE and debug
OTAFileUpdater *ota_updater;              ///< Firmware OTA via BLE or flash file
MemoryAccess *memory_access;              ///< RAM read/write access for DTE DUMPD command
Timer *system_timer;                      ///< Hardware timer (NrfTimer, 1 ms resolution)
Scheduler *system_scheduler;              ///< Task scheduler wrapping system_timer
RGBLed *status_led;                       ///< On-board RGB status LED
ReedSwitch *reed_switch;                  ///< Reed switch with confirmation gesture logic
DTEHandler *dte_handler;                  ///< DTE protocol command dispatcher (PARMR/PARMW/STAT/DUMPD)
RTC *rtc;                                 ///< Real-time clock (NrfRTC, epoch seconds)
BatteryMonitor *battery_monitor;          ///< Battery voltage/level monitor (variant-dependent)
GPSDevice *gps_device;                    ///< GNSS receiver (u-blox M10Q)
GPSService *gps_service = nullptr;        ///< GNSS service singleton (set by phase 6 if M10Q detected)
KineisDevice *kineis_device_instance = nullptr;  ///< Satellite TX device (SMD, KIM2, or LoRa)
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
SmdSat *smd_sat_instance = nullptr;       ///< SMD satellite instance (needed for DFU OTA)
#endif
#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
LoRaDevice *lora_device_instance = nullptr;  ///< LoRa RAK3172 instance (needed for LORATX DTE)
#endif
#if !(defined(LORA_RAK3172) && (LORA_RAK3172 == 1)) && !(defined(ARGOS_SMD) && (ARGOS_SMD == 1))
KIM2Device *kim2_device_instance = nullptr;  ///< KIM2 instance (needed for KIMBR DTE bridge)
#endif
#if ENABLE_MORTALITY_SENSOR
MortalityService *mortality_service = nullptr;  ///< Bird mortality detection service (RSPB only)
#endif
Buzzer *buzzer_ctl;                       ///< Optional piezo buzzer (board-dependent)
/// @}

/// @brief Active debug output channel (UART, USB CDC, BLE NUS, or NONE).
///        Defaults to UART on RSPB, USB CDC on LinkIt V4 — but ONLY in debug
///        builds. Release builds (`NDEBUG` defined by `CMAKE_BUILD_TYPE=Release`)
///        default to NONE so the device stays silent in the field — no USB/UART
///        traffic underwater or in storage mode, no current drawn by the debug
///        peripheral, and no risk of a host enumerating the CDC port when the
///        epoxied tag is plugged into anything.
///        Can still be overridden at runtime via DTE if needed for debugging.
#ifdef NDEBUG
BaseDebugMode g_debug_mode = BaseDebugMode::NONE;
#elif defined(BOARD_RSPB)
BaseDebugMode g_debug_mode = BaseDebugMode::UART;
#else
BaseDebugMode g_debug_mode = BaseDebugMode::USB_CDC;
#endif

/// @brief Guards _write() from outputting before debug peripheral is ready.
static bool m_is_debug_init = false;

// =====================================================================
// Safety net 3.4 — Exception storm detector (2026-06)
// =====================================================================
// Defensive auto-recovery: if the main loop catches >EXC_STORM_THRESHOLD
// exceptions inside EXC_STORM_WINDOW_MS, force a soft reset. The existing
// catch chain already kicks the watchdog every iteration, so a service
// that throws on every dispatch (e.g. a corrupted config-store read) would
// otherwise loop forever, kicking the WDT and never letting the hardware
// WDT firing recover the device. This is one of the failure modes that
// turns sealed turtles into permanent bricks.
//
// Worst case: identical to current behaviour (no exceptions = never fires).
// Best case: tag self-recovers via soft reset → cold boot → boot-fail
// counter logic kicks in if persistent.
static unsigned int s_exc_storm_count = 0;
static uint64_t     s_exc_storm_window_start_ms = 0;
static constexpr unsigned int EXC_STORM_THRESHOLD = 100;
static constexpr uint64_t     EXC_STORM_WINDOW_MS = 60ULL * 1000ULL;  // 1 min

static void gps_service_exception_storm_check() {
	uint64_t now_ms = PMU::get_timestamp_ms();
	if (s_exc_storm_window_start_ms == 0 ||
	    now_ms - s_exc_storm_window_start_ms > EXC_STORM_WINDOW_MS) {
		// Outside window: reset counter and start a new window.
		s_exc_storm_window_start_ms = now_ms;
		s_exc_storm_count = 0;
	}
	s_exc_storm_count++;
	if (s_exc_storm_count >= EXC_STORM_THRESHOLD) {
		DEBUG_ERROR("Exception storm: %u exceptions in <%llu ms — forcing soft reset",
		            s_exc_storm_count, (unsigned long long)EXC_STORM_WINDOW_MS);
		PMU::save_stack(PMULogType::ETL);
		PMU::reset(false);
	}
}

// =====================================================================
// Safety net 3.7 — Scheduler queue drop detector (2026-06)
// =====================================================================
// `g_scheduler_drop_count` is incremented inside Scheduler::schedule_now /
// schedule_deferred whenever the queue is full and a task is dropped. A
// dropped task can be CRITICAL — e.g. the deep-idle auto-poweroff timer or
// a safety-net watchdog — and a silent drop leads to the rail-on / brick
// pattern observed in 2026-06 field deployments.
//
// Strategy: snapshot the counter every main loop iteration. If it grows
// past SCHED_DROP_THRESHOLD inside SCHED_DROP_WINDOW_MS, force a soft
// reset. The queue was already doubled to 128 in scheduler.hpp, so legit
// load should never trip this — only a pathological storm does.
//
// Worst case (no drops): zero impact, just one comparison per iteration.
// Best case (genuine drop storm): tag reboots cleanly before V_BCKP drains.
static unsigned int s_sched_drop_last_observed = 0;
static unsigned int s_sched_drop_in_window = 0;
static uint64_t     s_sched_drop_window_start_ms = 0;
static constexpr unsigned int SCHED_DROP_THRESHOLD = 10;
static constexpr uint64_t     SCHED_DROP_WINDOW_MS = 60ULL * 1000ULL;  // 1 min

static void scheduler_drop_storm_check() {
	if (g_scheduler_drop_count == s_sched_drop_last_observed) return;  // no new drops
	unsigned int delta = g_scheduler_drop_count - s_sched_drop_last_observed;
	s_sched_drop_last_observed = g_scheduler_drop_count;

	uint64_t now_ms = PMU::get_timestamp_ms();
	if (s_sched_drop_window_start_ms == 0 ||
	    now_ms - s_sched_drop_window_start_ms > SCHED_DROP_WINDOW_MS) {
		s_sched_drop_window_start_ms = now_ms;
		s_sched_drop_in_window = 0;
	}
	s_sched_drop_in_window += delta;
	if (s_sched_drop_in_window >= SCHED_DROP_THRESHOLD) {
		DEBUG_ERROR("Scheduler drop storm: %u drops in <%llu ms — forcing soft reset",
		            s_sched_drop_in_window, (unsigned long long)SCHED_DROP_WINDOW_MS);
		PMU::save_stack(PMULogType::ETL);
		PMU::reset(false);
	}
}

FSM_INITIAL_STATE(GenTracker, BootState)

/// @brief Number of IS25 flash blocks reserved for OTA firmware images (last 1 MB).
#define OTA_UPDATE_RESERVED_BLOCKS ((1024 * 1024) / IS25_BLOCK_SIZE)

/// @brief Reed switch debounce time in ms.
/// @note Increased from 25 ms to 250 ms to filter noise from VSENSORS rail switching.
#define REED_SWITCH_DEBOUNCE_TIME_MS    250


/**
 * @brief Infinite fault indicator — blinks the RGB LED and kicks the watchdog forever.
 *
 * Uses NrfRGBLed::set_color_raw() for bare-metal GPIO access (no timer/scheduler dependency).
 *
 * Fault color map:
 *   - RED 50 ms      → HardFault
 *   - YELLOW 50 ms   → MemManagement
 *   - MAGENTA 50 ms  → Stack overflow (__stack_chk_fail)
 *   - CYAN 50 ms     → Malloc failure
 *   - RED 200 ms     → ETL assertion
 *
 * @param color    LED color identifying the fault type.
 * @param delay_ms Blink half-period in milliseconds (default 50 ms).
 */
[[noreturn]] static void fault_blink_loop(RGBLedColor color, unsigned int delay_ms = 50) {
	for (;;) {
#ifdef GPIO_LED_REG
		GPIOPins::set(GPIO_LED_REG);
#endif
		NrfRGBLed::set_color_raw(BSP::GPIO::GPIO_LED_RED, BSP::GPIO::GPIO_LED_GREEN, BSP::GPIO::GPIO_LED_BLUE, color);
		nrf_delay_ms(delay_ms);
		NrfRGBLed::set_color_raw(BSP::GPIO::GPIO_LED_RED, BSP::GPIO::GPIO_LED_GREEN, BSP::GPIO::GPIO_LED_BLUE, RGBLedColor::BLACK);
		nrf_delay_ms(delay_ms);
		PMU::kick_watchdog();
	}
}

extern "C" void HardFault_Handler() {
#ifdef NDEBUG
	for (;;) { PMU::save_stack(PMULogType::HARDFAULT); PMU::reset(false); }
#else
	fault_blink_loop(RGBLedColor::RED);
#endif
}

extern "C" void MemoryManagement_Handler(void) {
#ifdef NDEBUG
	for (;;) { PMU::save_stack(PMULogType::MMAN); PMU::reset(false); }
#else
	fault_blink_loop(RGBLedColor::YELLOW);
#endif
}

extern "C" {
	/// @brief Stack canary value for -fstack-protector.  Fixed constant — see review note.
	void *__stack_check_guard = (void*)0xDEADBEEF;
	/// @brief Called by -fstack-protector when the canary is corrupted.
	void __wrap___stack_chk_fail(void) {
#ifdef NDEBUG
		PMU::save_stack(PMULogType::STACK);
		PMU::reset(false);
#else
		fault_blink_loop(RGBLedColor::MAGENTA);
#endif
	}
}

extern "C" void vApplicationMallocFailedHook() {
#ifdef NDEBUG
	for (;;) { PMU::save_stack(PMULogType::MALLOC); PMU::reset(false); }
#else
	fault_blink_loop(RGBLedColor::CYAN);
#endif
}

/** @brief ETL library error callback — logs the exception then resets or blinks.
 *  @param e  ETL exception with file/line info.
 */
void etl_error_handler(const etl::exception& e)
{
	DEBUG_TRACE("ETL error: %s in %s : %u", e.what(), e.file_name(), e.line_number());
#ifdef NDEBUG
	for (;;) { PMU::save_stack(PMULogType::ETL); PMU::reset(false); }
#else
	fault_blink_loop(RGBLedColor::RED, 200);
#endif
}

/**
 * @brief Low-level write override — routes stdout/printf to the active debug channel.
 *
 * Overrides the weak newlib _write() so that printf / std::cout output is directed
 * to UART, USB CDC, or BLE NUS depending on g_debug_mode.
 *
 * @param file  File descriptor (unused — all output goes to debug channel).
 * @param ptr   Data buffer to write.
 * @param len   Number of bytes to write.
 * @return Always returns @p len (bytes accepted, even if output is suppressed).
 * @note Safe to call from ISR context only for UART and USB CDC paths.
 *       BLE NUS path checks __get_IPSR() and skips if in interrupt context.
 */
extern "C" int _write(int file, char *ptr, int len)
{
#ifdef DEBUG_UART_TX_PIN
	if (g_debug_mode == BaseDebugMode::UART && NrfDebugUart::is_init())
		NrfDebugUart::write(ptr, len);
	else
#endif
	if (g_debug_mode == BaseDebugMode::USB_CDC && m_is_debug_init)
		NrfUSB::write(ptr, len);
	else if (ble_service && !__get_IPSR() && g_debug_mode == BaseDebugMode::BLE_NUS) {
		ble_service->write_best_effort(std::string(ptr, len));
	}
	return len;
}


// ═══════════════════════════════════════════════════════════════
// Init sub-functions
//
// Each phase creates static objects (in .bss/.data) and assigns
// global pointers.  This keeps main() readable and reduces its
// stack frame — static objects are visible in the linker map.
// ═══════════════════════════════════════════════════════════════

/** @brief References returned by init_peripherals() for use in subsequent init phases. */
struct InitContext {
	NrfSwitch& reed;   ///< Reed switch — needed by power-on check, gesture, and dive mode
	Is25Flash& flash;   ///< External NOR flash — needed by LFS and OTA updater
};

/**
 * @brief Phase 1 — Initialise core peripherals.
 *
 * Sets up: PMU, watchdog, GPIO, RTC, hardware timer, debug output (UART/USB/BLE),
 * NFC pin check, RGB LED, external LED, reed switch, buzzer, BLE stack, IS25 flash.
 *
 * @return InitContext with references to reed switch and flash for subsequent phases.
 */
static InitContext init_peripherals()
{
	// Storage-mode wake filter: if we just woke from PSEUDO_POWER_OFF without
	// a magnet held, the device enters System OFF mode immediately (deepest
	// sleep ~0.4 µA + Hall switch ~1.7 µA = ~2 µA total). Must run BEFORE
	// PMU::initialise() (which clears RESETREAS / GPREGRET) and BEFORE
	// start_watchdog() (WDT would tick in System OFF and reset the chip).
	// No-op on boards without PSEUDO_POWER_OFF (RSPB).
	PMU::storage_off_check();

	PMU::initialise();
#ifndef DEBUG_NO_WATCHDOG
	PMU::start_watchdog();
	PMU::kick_watchdog();
#endif
	GPIOPins::initialise();
	etl::error_handler::set_callback<etl_error_handler>();

#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
	// Encapsulated-device safety: a soft reset (OTA, watchdog, nrfjprog --reset,
	// PMU::reset) leaves the STM32WL co-processor running with state on its VDD
	// caps — its SPI sequence number doesn't match what the freshly-booted nRF
	// expects, producing INVALID_CMD / "Response incomplete" cascades on the
	// first session. POR + cap discharge here forces a clean STM32WL boot.
	// Skipped on POWER_ON resets where the STM was off anyway.
	//
	// Use drive_low() (= cfg_output then clear) for SAT_RESET — the BSP entry
	// is DIR_INPUT (commented "for probe flashing"), so init_pin+clear would
	// NOT actively drive the line. drive_low overrides to OUTPUT and forces
	// the pin LOW, holding the STM32WL in reset while VDD discharges.
	if (PMU::reset_cause() != ResetCause::POWER_ON) {
		GPIOPins::clear(SAT_PWR_EN);     // Cut STM32WL VDD (BSP=OUTPUT)
#ifdef SAT_RESET
		GPIOPins::drive_low(SAT_RESET);  // Force STM32WL into reset (overrides BSP=INPUT)
#endif
#ifdef SMD_VPA_PIN
		GPIOPins::drive_low(SMD_VPA_PIN);
#endif
		PMU::delay_ms(500);               // Drain VDD caps — STM32WL needs ~50-200ms typically; 500ms is safe margin
#ifdef SAT_RESET
		GPIOPins::release_to_highz(SAT_RESET);  // Restore high-Z so SmdSat::power_on can manage normally
#endif
	}
#endif

	rtc = &NrfRTC::get_instance();
	NrfRTC::get_instance().init();

	DEBUG_TRACE("Timer...");
	system_timer = &NrfTimer::get_instance();
	NrfTimer::get_instance().init();

	// Init debug output (UART, USB CDC, BLE, or NONE).
	//
	// In release builds (NDEBUG), `g_debug_mode` defaults to NONE so no logs
	// flow over UART/USB during operational/underwater states — see the
	// declaration of `g_debug_mode` for rationale.
	//
	// USB CDC peripheral, however, is ALWAYS initialised on LinkIt regardless
	// of `g_debug_mode` because `USB_DTE_ENABLED` makes ConfigurationState
	// rely on USB for DTE access (magnet-gesture bench configuration). The
	// `_write()` routing checks `g_debug_mode` so no log bytes are pushed to
	// USB when in NONE mode — the peripheral is just enumerable for DTE.
	m_is_debug_init = true;
	static ConsoleLog console_log;
	DebugLogger::console_log = (g_debug_mode == BaseDebugMode::NONE) ? nullptr : &console_log;
#ifdef DEBUG_UART_TX_PIN
	// RSPB: UART is the primary debug channel. Only init when actively
	// requested (so release-mode NONE leaves UART pins inactive → no power).
	if (g_debug_mode == BaseDebugMode::UART) {
		NrfDebugUart::init(DEBUG_UART_TX_PIN);
	} else if (g_debug_mode != BaseDebugMode::NONE) {
		// Edge case: RSPB explicitly configured for USB CDC or BLE NUS debug.
		NrfUSB::init();
	}
#else
	// LinkIt: USB is shared between debug output and DTE — always init,
	// even when g_debug_mode == NONE, so ConfigurationState (magnet-gesture
	// DTE access) keeps working in release builds.
	NrfUSB::init();
#endif
	setvbuf(stdout, NULL, _IONBF, 0);
	nrf_log_redirect_init();

	// Verify NFC pins are configured as GPIO (P0.09/P0.10 used for SPI CS and SWS_OUT)
	{
		uint32_t nfcpins = NRF_UICR->NFCPINS;
		if ((nfcpins & UICR_NFCPINS_PROTECT_Msk) == (UICR_NFCPINS_PROTECT_NFC << UICR_NFCPINS_PROTECT_Pos)) {
			DEBUG_ERROR("UICR: NFC pins still in NFC mode (0x%08X) — P0.09/P0.10 unusable as GPIO!", nfcpins);
		} else {
			DEBUG_INFO("UICR: NFC pins configured as GPIO (0x%08X)", nfcpins);
		}
	}

	DEBUG_TRACE("RGB LED...");
	static NrfRGBLed nrf_status_led("STATUS", BSP::GPIO::GPIO_LED_RED, BSP::GPIO::GPIO_LED_GREEN, BSP::GPIO::GPIO_LED_BLUE, RGBLedColor::WHITE);
	status_led = &nrf_status_led;

#ifdef SMD_FLASH_HOLD
	// ───────────────── TEST MODE: hold the SMD powered for flashing ─────────────────
	// Powers the STM32WL (SAT_PWR_EN high) with the PA regulator OFF (VPA driven low)
	// and releases SAT_RESET to high-Z so an external SWD probe fully owns the STM32WL
	// reset line. The nRF then PARKS here forever (feeding the watchdog) and does
	// NOTHING else — no Argos TX, no I2C recovery, no power cycling — so the supply
	// never sees the TX current spikes that trip the board reset supervisor and reboot
	// the nRF ("Hard Reset" / RESETPIN). Result: a stable, low-current SMD rail you can
	// flash without the board resetting.
	//
	// Enable with -DSMD_FLASH_HOLD=1 (RSPB build). REMOVE for normal operation.
	DEBUG_WARN("SMD_FLASH_HOLD: powering SMD + parking — flash the STM32WL now (MAGENTA LED)");
#ifdef SMD_VPA_PIN
	GPIOPins::drive_low(SMD_VPA_PIN);        // PA regulator OFF — no TX current spikes
#endif
#ifdef SAT_RESET
	GPIOPins::release_to_highz(SAT_RESET);   // high-Z: SWD probe controls STM32WL reset
#endif
	GPIOPins::set(SAT_PWR_EN);               // STM32WL VDD ON, held indefinitely
	status_led->set(RGBLedColor::MAGENTA);   // visual marker: flash-hold mode active
	for (;;) {
#ifndef DEBUG_NO_WATCHDOG
		PMU::kick_watchdog();
#endif
		PMU::delay_ms(200);
	}
#endif // SMD_FLASH_HOLD

	DEBUG_TRACE("Reed switch...");
	static NrfSwitch nrf_reed_switch(BSP::GPIO::GPIO_REED_SW, REED_SWITCH_DEBOUNCE_TIME_MS, REED_SWITCH_ACTIVE_STATE);

#ifdef BUZZER_EN_PIN
	DEBUG_TRACE("Buzzer...");
	static Buzzer buzzer(BUZZER_EN_PIN);
	buzzer_ctl = &buzzer;
#else
	buzzer_ctl = nullptr;
#endif

	DEBUG_TRACE("BLE...");
	BleInterface::get_instance().init();

	DEBUG_TRACE("IS25 flash...");
	static Is25Flash is25_flash;
	if (!is25_flash.init()) {
		DEBUG_ERROR("IS25 flash init failed — cannot continue without filesystem");
		fault_blink_loop(RGBLedColor::RED, 100);
	}

	return {nrf_reed_switch, is25_flash};
}


/**
 * @brief Phase 2 — Power-on reed switch gesture check.
 *
 * On power-on reset, the user must hold a magnet on the reed switch for 3 seconds
 * to confirm intentional boot.  Without this gesture the device powers down
 * immediately, preventing accidental wake from battery insertion or transient
 * power glitches.
 *
 * Two code paths depending on board type:
 *  - PSEUDO_POWER_OFF (LinkIt V4): timer-based, handles both real and pseudo power-on.
 *  - Non-PSEUDO (RSPB): simple polling countdown for true power-on reset only.
 *
 * @param nrf_reed_switch  Hardware reed switch instance for state/callback registration.
 */
static void init_power_on_check(NrfSwitch& nrf_reed_switch)
{
	DEBUG_TRACE("PMU Reset Cause = %s", PMU::reset_cause_str());

#ifdef POWER_ON_RESET_REQUIRES_REED_SWITCH
#ifdef PSEUDO_POWER_OFF
	// PSEUDO_POWER_OFF (LinkIt V4): The nRF52 has no true power switch — it uses
	// VSYS_SEL to latch its own supply via a load switch. "Power off" is a soft
	// shutdown (System OFF mode) with VSYS_SEL de-asserted. A reed switch GPIO
	// event wakes the chip, which sees "Pseudo Power On Reset" instead of a real
	// "Power On Reset". This branch handles both reset causes.
	{
		SensorsPowerGuard power_guard;
		NrfI2C::init();
		PMU::hardware_version(); // Side-effect: I2C probes for hardware detection
#if ENABLE_SEA_TEMP_SENSOR
		{
			try {
				EZO_RTD_Sensor rtd; // Puts the device into standby mode
			} catch (...) {}
		}
#endif
		NrfI2C::uninit();
	}
	bool is_linkit_v4 = (strncmp(PMU::hardware_version(), "linkit-v4", 9) == 0);

	ResetCause cause = PMU::reset_cause();
	if ((is_linkit_v4 && cause == ResetCause::PSEUDO_POWER_ON) ||
		(!is_linkit_v4 && (cause == ResetCause::POWER_ON ||
				cause == ResetCause::PSEUDO_POWER_ON))) {

		volatile bool power_on_ready = false;
		system_timer->start();
		Timer::TimerHandle timer_handle;

		if (nrf_reed_switch.get_state()) {
			status_led->set(RGBLedColor::WHITE);
			BUZZER_ON(buzzer_ctl);
			timer_handle = system_timer->add_schedule([&power_on_ready]() {
				DEBUG_TRACE("Reed switch 3s period elapsed");
				power_on_ready = true;
			}, system_timer->get_counter() + 3000);
		} else {
			status_led->off();
			BUZZER_OFF(buzzer_ctl);
		}

		nrf_reed_switch.start([&timer_handle, &power_on_ready](bool state) {
			system_timer->cancel_schedule(timer_handle);
			if (state) {
				DEBUG_TRACE("Reed State: %u", state);
				GPIOPins::set(VSYS_SEL);
				status_led->set(RGBLedColor::WHITE);
				BUZZER_ON(buzzer_ctl);
				timer_handle = system_timer->add_schedule([&power_on_ready]() {
					DEBUG_TRACE("Reed switch 3s period elapsed");
					power_on_ready = true;
				}, system_timer->get_counter() + 3000);
			} else {
				status_led->off();
				GPIOPins::clear(VSYS_SEL);
				BUZZER_OFF(buzzer_ctl);
			}
		});

		GPIOPins::clear(VSYS_SEL);
		// H1 fix: bound the wait. If the 3 s magnet-hold is never completed (magnet
		// released early, or reed chatter that keeps cancelling the timer), don't spin
		// forever feeding the WDT — give up and power down, mirroring the RSPB #else
		// branch's give-up semantics. 30 s is well beyond a deliberate 3 s hold and
		// still allows a few retry attempts.
		uint64_t power_on_deadline = system_timer->get_counter() + 30000;
		while (!power_on_ready) {
			PMU::kick_watchdog();
			PMU::run();
			if (system_timer->get_counter() >= power_on_deadline) {
				DEBUG_TRACE("Power-on reed gesture not completed within 30s — powering down");
				PMU::powerdown();
				break;  // defensive: if powerdown() returns, stop spinning
			}
		}

		GPIOPins::set(VSYS_SEL);
		PMU::kick_watchdog();
		nrf_reed_switch.stop();
		BUZZER_BEEP_COUNT(buzzer_ctl,200,200,2);
	}
#else
	// Without PSEUDO_POWER_OFF (RSPB/gentracker): The board has a real power path
	// (TPL5111 or physical switch), so only true "Power On Reset" is checked with
	// a simple 3-second polling countdown — no timer/callback needed.
	if (PMU::reset_cause() == ResetCause::POWER_ON) {
		unsigned int countdown = 3000;
		DEBUG_TRACE("Enter Power On Reed Switch Check");
		while (countdown) {
			if (GPIOPins::value(BSP::GPIO_REED_SW) != REED_SWITCH_ACTIVE_STATE)
				break;
			PMU::delay_ms(1);
			countdown--;
		}

		if (countdown) {
			DEBUG_TRACE("Reed Switch Inactive -- Powering Down...");
			PMU::powerdown();
		}
		DEBUG_TRACE("Exiting Power On Reed Switch Check");
	}
#endif // PSEUDO_POWER_OFF
#endif // POWER_ON_RESET_REQUIRES_REED_SWITCH
}


/**
 * @brief Phase 3 — Storage and configuration.
 *
 * Sets up: scheduler, I2C bus, reed gesture wrapper, LittleFS filesystem
 * (mount or format+mount), configuration store, RTC restoration from flash,
 * and TPL5111 boot counter / modulo check (EXTERNAL_WAKEUP boards).
 *
 * @param nrf_reed_switch  Reed switch for boot-counter magnet override check.
 * @param is25_flash       External flash device for LFS backing store.
 * @return Reference to the mounted LFSFileSystem (static, lives in .bss).
 */
static LFSFileSystem& init_storage(NrfSwitch& nrf_reed_switch, Is25Flash& is25_flash)
{
	DEBUG_TRACE("Scheduler...");
	static Scheduler scheduler(system_timer);
	system_scheduler = &scheduler;

	// VSENSORS must be on during I2C init to prevent bus appearing stuck
	{
		SensorsPowerGuard power_guard;
		NrfI2C::init();
	}

	DEBUG_TRACE("Reed gesture...");
	static ReedSwitch reed_gesture_switch(nrf_reed_switch);
	reed_switch = &reed_gesture_switch;

	DEBUG_TRACE("LFS filesystem...");
	static LFSFileSystem lfs_file_system(&is25_flash, IS25_BLOCK_COUNT - OTA_UPDATE_RESERVED_BLOCKS);
	main_filesystem = &lfs_file_system;

	DEBUG_TRACE("Mount LFS filesystem...");
#ifdef FORCE_FORMAT_FILESYSTEM
	DEBUG_WARN("FORCE_FORMAT_FILESYSTEM enabled - formatting filesystem!");
	if (main_filesystem->format() < 0 || main_filesystem->mount() < 0)
	{
		DEBUG_ERROR("Failed to format LFS filesystem");
		PMU::powerdown();
	}
#else
	if (main_filesystem->mount() < 0)
	{
		DEBUG_TRACE("Format LFS filesystem...");
		if (main_filesystem->format() < 0 || main_filesystem->mount() < 0)
		{
			DEBUG_ERROR("Failed to format LFS filesystem");
			PMU::powerdown();
		}
	}
#endif

	DEBUG_TRACE("Configuration store...");
	static LFSConfigurationStore store(lfs_file_system);
	configuration_store = &store;
	configuration_store->init();

	// Restore last known RTC from flash (enables MGA-ANO assistance and faster GNSS TTFF)
	{
		unsigned int last_rtc = configuration_store->read_param<unsigned int>(ParamID::LAST_KNOWN_RTC);

		// Anti-corruption guard (2026-05, audit mitigation C): a POF / brown-out
		// mid-LFS-write or a single-byte bit flip in the param block can leave
		// LAST_KNOWN_RTC at an impossible future value. Restoring such a value
		// would put the RTC decades ahead and break every time-based defense
		// (cooldown, hauled, rate limiter, ANO staleness).
		//
		// We deliberately do NOT reject "small" values (< 2000-01-01) because:
		//   - On RSPB / EXTERNAL_WAKEUP, the pseudo-RTC chain legitimately
		//     stores small values (sum of WAKEUP_PERIOD * boot_count) before
		//     the first GNSS sync.
		//   - On sealed LinkIt V4 without GNSS, M1a periodic save persists the
		//     virtual RTC (uptime-based, also small). Rejecting these would
		//     defeat the entire post-WDT continuity mechanism.
		// Field-deployed corruption tends to flip bits in the high half of the
		// value, producing far-future values. The upper bound catches those.
		// Tighter defense is the LFS file-level integrity (LittleFS CRC).
		constexpr unsigned int RTC_MAX_VALID = 4102444800U; // 2100-01-01
		if (last_rtc > RTC_MAX_VALID) {
			DEBUG_WARN("Suspicious LAST_KNOWN_RTC=%u (post-2100), treating as corrupt", last_rtc);
			last_rtc = 0;  // force virtual RTC fallback
		}
#ifdef EXTERNAL_WAKEUP
		// Pseudo RTC: advance the last known RTC by WAKEUP_PERIOD at each boot.
		unsigned int wakeup_period = configuration_store->read_param<unsigned int>(ParamID::WAKEUP_PERIOD);
		if (last_rtc > 0 && wakeup_period > 0) {
			unsigned int pseudo_rtc = last_rtc + wakeup_period;
			configuration_store->write_param(ParamID::LAST_KNOWN_RTC, pseudo_rtc);
			rtc->settime(static_cast<std::time_t>(pseudo_rtc));
			// Chaine pseudo-RTC: c'est une extrapolation, pas une synchro.
			rtc->note_source(RtcSource::RESTORED);
			DEBUG_INFO("EXTERNAL_WAKEUP: Pseudo RTC set to %u (last=%u + period=%u)", pseudo_rtc, last_rtc, wakeup_period);
		} else {
			DEBUG_INFO("EXTERNAL_WAKEUP: No pseudo RTC available (last_rtc=%u, wakeup_period=%u)", last_rtc, wakeup_period);
		}

		// TPL5111 boot counter management
		unsigned int boot_counter = configuration_store->boot_count_increment();
		DEBUG_INFO("EXTERNAL_WAKEUP: Boot counter = %u", boot_counter);

		// Check if this is not our turn to run based on modulo
		if (boot_counter > 0 && !configuration_store->boot_count_check_modulo(boot_counter)) {
#ifdef BENCH_TEST
			// Bench: never modulo-powerdown — powerdown is suppressed anyway
			// (see PMU::powerdown), and we want every boot to reach the work +
			// Argos-TX path so the SMD SPI can be exercised and observed.
			DEBUG_INFO("EXTERNAL_WAKEUP: BENCH_TEST — modulo skip suppressed, running this boot");
#else
			if (nrf_reed_switch.get_state()) {
				DEBUG_INFO("EXTERNAL_WAKEUP: Magnet detected | skipping modulo powerdown");
			} else {
				DEBUG_INFO("EXTERNAL_WAKEUP: Not our turn to run (modulo check) | powering down");
				status_led->flash(RGBLedColor::RED, 100);
				PMU::delay_ms(300);
				status_led->off();
				PMU::powerdown();
			}
#endif
		}
		DEBUG_INFO("EXTERNAL_WAKEUP: Our turn to run | continuing boot");
#else
		// Non-TPL5111 boards: restore last known RTC directly
		if (last_rtc > 0) {
			rtc->settime(static_cast<std::time_t>(last_rtc));
			// RESTORED et non GNSS: le temps passe hors tension est inconnu, donc
			// l'erreur de cette heure ne se borne pas. Sans cette distinction,
			// is_set() renvoyait « oui » et le firmware annoncait cette valeur au
			// recepteur a +/- 2 s — mesure au banc le 2026-08-25 avec 52 jours
			// d'ecart.
			rtc->note_source(RtcSource::RESTORED);
			DEBUG_INFO("Restored LAST_KNOWN_RTC = %u (provenance: restauree)", last_rtc);
		}
#endif

		// Virtual RTC fallback (2026-05): sealed turtle deployments may go
		// days / weeks before a first GNSS fix, and LAST_KNOWN_RTC stays at 0
		// until then. Without an `is_set()` RTC, every time-based defense
		// (cooldown, hauled, rate limiter, sequencer) is silently dead —
		// they all gate on `rtc->is_set()`. Initialize to 1 (epoch + 1 s) so
		// relative-time math works from boot; GPSService::on_fix() will
		// overwrite with real UTC the moment a fix arrives. Code paths that
		// need absolute UTC (PASS_PREDICTION, log human-readable timestamps)
		// already independently validate the time horizon — they'll see a
		// "1970" RTC and behave defensively until sync.
		if (rtc && !rtc->is_set()) {
			rtc->settime(static_cast<std::time_t>(1));
			DEBUG_INFO("Cold-first-boot: RTC initialized to virtual epoch (1) — sync on first GNSS");
		}
	}

	return lfs_file_system;
}


/**
 * @brief Phase 4 — Battery monitor initialisation.
 *
 * Selects the appropriate battery monitor implementation based on CMake flags:
 *  - BATTERY_MONITOR_ANALOG: nRF SAADC-based (LinkIt V4)
 *  - BATTERY_MONITOR_STC3117: I2C fuel gauge (RSPB)
 *  - BATTERY_MONITOR_FAKE: fixed 4.1 V stub (testing)
 */
static void init_battery()
{
	DEBUG_TRACE("Battery monitor...");
	uint8_t critical_batt_level = static_cast<uint8_t>(configuration_store->read_param<unsigned int>(ParamID::LB_CRITICAL_THRESH));
	uint8_t low_batt_level = static_cast<uint8_t>(configuration_store->read_param<unsigned int>(ParamID::LB_THRESHOLD));

#if defined(BATTERY_MONITOR_ANALOG)
#ifdef BATTERY_ADC
	// Chemistry selected at compile time via -DBATTERY_CHEMISTRY=<enum_name>
	// (CMake option BATTERY_CHEMISTRY). Defaults to NCR18650 Li-ion.
#ifndef BATTERY_CHEMISTRY
#define BATTERY_CHEMISTRY BATT_CHEM_NCR18650_3100_3400
#endif
	static NrfBatteryMonitor nrf_battery_monitor(BATTERY_ADC, BATTERY_CHEMISTRY,
			critical_batt_level, low_batt_level);
	battery_monitor = &nrf_battery_monitor;
#else
	static BatteryMonitor stub_battery_monitor(low_batt_level, critical_batt_level);
	battery_monitor = &stub_battery_monitor;
#endif
#elif defined(BATTERY_MONITOR_FAKE)
	DEBUG_INFO("Using FAKE battery monitor (always 4.1V)");
	static FakeBatteryMonitor fake_battery_monitor(critical_batt_level, low_batt_level);
	battery_monitor = &fake_battery_monitor;
#elif defined(BATTERY_MONITOR_STC3117)
	DEBUG_INFO("Using STC3117 fuel gauge battery monitor");
	static GaugeBatteryMonitor stc3117_battery_monitor(critical_batt_level, low_batt_level);
	battery_monitor = &stc3117_battery_monitor;
#else
	#error "No battery monitor type defined! Set BATTERY_MONITOR_TYPE in CMake"
#endif
}


/**
 * @brief Phase 5 — Core services (system log, DTE handler, OTA updater, memory access).
 *
 * @param lfs_file_system  Mounted filesystem for system.log and OTA staging.
 * @param is25_flash       Raw flash device for OTA block-level writes.
 */
static void init_core_services(LFSFileSystem& lfs_file_system, Is25Flash& is25_flash)
{
	DEBUG_INFO("Creating log files...");
	static SysLogFormatter sys_log_formatter;
	static FsLog fs_system_log(&lfs_file_system, "system.log", 1024*1024);
	fs_system_log.set_log_formatter(&sys_log_formatter);
	DebugLogger::system_log = &fs_system_log;

	DEBUG_TRACE("RAM access...");
	static NrfMemoryAccess nrf_memory_access;
	memory_access = &nrf_memory_access;

	DEBUG_TRACE("DTE handler...");
	static DTEHandler dte_handler_local;
	dte_handler = &dte_handler_local;

	DEBUG_TRACE("OTA updater...");
	ble_service = &BleInterface::get_instance();
	static OTAFlashFileUpdater ota_flash_file_updater(&lfs_file_system, &is25_flash, IS25_BLOCK_COUNT - OTA_UPDATE_RESERVED_BLOCKS, OTA_UPDATE_RESERVED_BLOCKS);
	ota_updater = &ota_flash_file_updater;
}


/**
 * @brief Phase 6 — Communication backends and GPS.
 *
 * Initialises the salt-water switch (SWS) service, the satellite/LoRa TX backend
 * (mutually exclusive: SMD, KIM2, or LoRa RAK3172), and the GNSS receiver (M10Q).
 * Each backend's log file is created here alongside the device.
 *
 * @param lfs_file_system  Mounted filesystem for sensor.log and SWS log.
 */
static void init_communication(LFSFileSystem& lfs_file_system)
{
#if ENABLE_SWS_ANALOG
	DEBUG_INFO(">> Creating services...");
	static SWSAnalogService sws_analog;

#if ENABLE_SWS_LOG
	DEBUG_TRACE("SWS Log...");
	static SWSLogFormatter sws_log_formatter;
	static FsLog sws_log(&lfs_file_system, "SWS", 1024*1024);
	sws_log.set_log_formatter(&sws_log_formatter);
	SWSAnalogService::set_sws_logger(&sws_log);
#endif
#endif

#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
	DEBUG_TRACE("LoRa RAK3172...");
	try {
		static LoRaDevice lora_rak3172;
		lora_device_instance = &lora_rak3172;
		static LoRaTxService lora_tx_service(lora_rak3172);
	} catch (...) {
		DEBUG_INFO("LoRa RAK3172 not detected");
		lora_device_instance = nullptr;
	}
#elif defined(ARGOS_SMD) && (ARGOS_SMD == 1)
	DEBUG_TRACE("SMD Satellite...");
	try {
#if defined(SMD_UART) && (SMD_UART == 1)
		static SmdSatCmdAt smd_cmd;
#else
		static SmdSatCmdSpi smd_cmd;
#endif
		static SmdSat argos_smd(smd_cmd);
		smd_sat_instance = &argos_smd;  // Store pointer for SMD DFU OTA
		kineis_device_instance = &argos_smd;
		static ArgosTxService argos_tx_service(argos_smd);
	} catch (...) {
		DEBUG_INFO("SMD not detected");
		smd_sat_instance = nullptr;
		kineis_device_instance = nullptr;
	}
#else
	DEBUG_TRACE("KIM2...");
	try {
		static KIM2Device kim2;
		kineis_device_instance = &kim2;
		kim2_device_instance = &kim2;
		static ArgosTxService argos_tx_service(kim2);
	} catch (...) {
		DEBUG_INFO("KIM2 not detected");
		kineis_device_instance = nullptr;
		kim2_device_instance = nullptr;
	}
#endif

	DEBUG_TRACE("GPS M10Q ...");
	static GPSLogFormatter fs_sensor_log_formatter;
	static FsLog fs_sensor_log(&lfs_file_system, "sensor.log", 1024*1024);
	fs_sensor_log.set_log_formatter(&fs_sensor_log_formatter);

	try {
		static M10QAsyncReceiver m10q_gnss;
		gps_device = &m10q_gnss;
		static GPSService gps_service_instance(m10q_gnss, &fs_sensor_log);
		gps_service = &gps_service_instance;
	} catch (...) {
		DEBUG_INFO("GPS M10Q not detected");
	}
}


/**
 * @brief Phase 7 — Sensor detection, log creation, and service registration.
 *
 * Auto-detects I2C sensors (pressure, CDT, ALS, pH, sea temperature, accelerometer,
 * thermistor, camera) and creates their log files and SensorService instances.
 * Pressure sensor detection uses placement new into static aligned storage to avoid
 * heap allocation.  Each sensor that fails detection logs an INFO and is skipped.
 *
 * @param lfs_file_system  Mounted filesystem for per-sensor log files.
 */
static void init_sensors(LFSFileSystem& lfs_file_system)
{
#if ENABLE_PRESSURE_SENSOR || ENABLE_CDT_SENSOR
	DEBUG_TRACE("Pressure Sensor...");
	PressureSensorDevice *pressure_sensor_devices[BSP::I2C_TOTAL_NUMBER] = {nullptr};
#ifndef DUMMY_PRESSURE_SENSOR
	// Static storage for pressure sensors — one per I2C bus, no heap allocation
	static constexpr size_t PressureStorageSize = std::max({sizeof(LPS28DFW), sizeof(Bar100), sizeof(MS58xxLL)});
	static constexpr size_t PressureStorageAlign = std::max({alignof(LPS28DFW), alignof(Bar100), alignof(MS58xxLL)});
	struct alignas(PressureStorageAlign) PressureStorage { std::byte data[PressureStorageSize]; };
	static PressureStorage pressure_storage[BSP::I2C_TOTAL_NUMBER];

	// Sensor detection order — LPS28DFW is default for RSPB board
#if defined(BOARD_RSPB)
	static constexpr unsigned int i2caddr[4] = { LPS28DFW_ADDRESS, MS5803_ADDRESS, MS5837_ADDRESS, BAR100_ADDRESS };
	static const char* const variant[4] = { "LPS28DFW", MS5803_VARIANT, MS5837_VARIANT, "BAR100-R3-RP" };
#else
#ifndef LPS28DFW_ADDRESS
#define LPS28DFW_ADDRESS 0x5C
#endif
	static constexpr unsigned int i2caddr[4] = { MS5803_ADDRESS, MS5837_ADDRESS, BAR100_ADDRESS, LPS28DFW_ADDRESS };
	static const char* const variant[4] = { MS5803_VARIANT, MS5837_VARIANT, "BAR100-R3-RP", "LPS28DFW" };
#endif

	for (unsigned int i = 0; i < BSP::I2C_TOTAL_NUMBER; i++) {
		for (unsigned int j = 0; j < 4; j++) {
			try {
				PressureSensorDevice *device;
#if defined(BOARD_RSPB)
				if (j == 0) {
					device = new (&pressure_storage[i]) LPS28DFW(i, i2caddr[j]);
				} else if (j == 3) {
					device = new (&pressure_storage[i]) Bar100(i, i2caddr[j]);
				} else {
					device = new (&pressure_storage[i]) MS58xxLL(i, i2caddr[j], variant[j]);
				}
#else
				if (j == 2) {
					device = new (&pressure_storage[i]) Bar100(i, i2caddr[j]);
				} else if (j == 3) {
					device = new (&pressure_storage[i]) LPS28DFW(i, i2caddr[j]);
				} else {
					device = new (&pressure_storage[i]) MS58xxLL(i, i2caddr[j], variant[j]);
				}
#endif
				pressure_sensor_devices[i] = device;
				DEBUG_TRACE("%s: found on i2cbus=%u i2caddr=0x%02x", variant[j], i, i2caddr[j]);
				break;
			} catch (...) {
				DEBUG_INFO("Nothing detected on i2cbus=%u i2caddr=0x%02x", i, i2caddr[j]);
				pressure_sensor_devices[i] = nullptr;
			}
		}
	}
#else
	static PressureSensorDummyDevice dummy_pressure;
	pressure_sensor_devices[0] = &dummy_pressure;
#endif

#if ENABLE_CDT_SENSOR
	DEBUG_TRACE("AD5933...");
	AD5933 *ad5933_devices[BSP::I2C_TOTAL_NUMBER];
	struct alignas(AD5933LL) AD5933Storage { std::byte data[sizeof(AD5933LL)]; };
	static AD5933Storage ad5933_storage[BSP::I2C_TOTAL_NUMBER];
	for (unsigned int i = 0; i < BSP::I2C_TOTAL_NUMBER; i++) {
		try {
			ad5933_devices[i] = new (&ad5933_storage[i]) AD5933LL(i, AD5933_ADDRESS);
			DEBUG_TRACE("AD5933: found on i2cbus=%u i2caddr=0x%02x", i, AD5933_ADDRESS);
		} catch (...) {
			DEBUG_INFO("AD5933: not detected on i2cbus=%u i2caddr=0x%02x", i, AD5933_ADDRESS);
			ad5933_devices[i] = nullptr;
		}
	}

	static CDTLogFormatter cdt_sensor_log_formatter;
	static FsLog cdt_sensor_log(&lfs_file_system, "CDT", 1024*1024);
	cdt_sensor_log.set_log_formatter(&cdt_sensor_log_formatter);
#endif

#if ENABLE_PRESSURE_SENSOR
	static PressureLogFormatter pressure_sensor_log_formatter;
	static FsLog pressure_sensor_log(&lfs_file_system, "PRESSURE", 1024*1024);
	pressure_sensor_log.set_log_formatter(&pressure_sensor_log_formatter);
#endif

	bool cdt_present = false;
	bool standalone_pressure = false;
	// Iterate twice to allow flags to be set
	for (unsigned int x = 0; x < 2; x++) {
		for (unsigned int i = 0; i < BSP::I2C_TOTAL_NUMBER; i++) {
#if ENABLE_CDT_SENSOR
			if (!cdt_present && ad5933_devices[i] && pressure_sensor_devices[i]) {
				DEBUG_TRACE("CDT on bus %u...", i);
				cdt_present = true;
				static CDT cdt(*pressure_sensor_devices[i], *ad5933_devices[i]);
				static CDTSensorService cdt_sensor_service(cdt, &cdt_sensor_log);
			} else
#endif
#if ENABLE_PRESSURE_SENSOR
			if (!standalone_pressure && pressure_sensor_devices[i]) {
				DEBUG_TRACE("Standalone Pressure Sensor on bus %u...", i);
				standalone_pressure = true;
				static PressureSensor pressure_sensor(*pressure_sensor_devices[i]);
				static PressureSensorService pressure_sensor_service(pressure_sensor, &pressure_sensor_log);
			}
#endif
			(void)i; // Suppress unused warning when both disabled
		}

		if (standalone_pressure && cdt_present)
			break;
	}
#endif // ENABLE_PRESSURE_SENSOR || ENABLE_CDT_SENSOR

#if ENABLE_ALS_SENSOR
	DEBUG_TRACE("LTR303...");
	static ALSLogFormatter als_sensor_log_formatter;
	static FsLog als_sensor_log(&lfs_file_system, "ALS", 1024*1024);
	als_sensor_log.set_log_formatter(&als_sensor_log_formatter);
	try {
		static LTR303 ltr303;
		static ALSSensorService als_sensor_service(ltr303, &als_sensor_log);
	} catch (...) {
		DEBUG_INFO("LTR303: not detected");
	}
#endif

#if ENABLE_PH_SENSOR
	DEBUG_TRACE("OEM PH...");
	static PHLogFormatter ph_sensor_log_formatter;
	static FsLog ph_sensor_log(&lfs_file_system, "PH", 1024*1024);
	ph_sensor_log.set_log_formatter(&ph_sensor_log_formatter);
	try {
		static OEM_PH_Sensor ph;
		static PHSensorService ph_sensor_service(ph, &ph_sensor_log);
	} catch (...) {
		DEBUG_INFO("OEM PH: not detected");
	}
#endif

#if ENABLE_SEA_TEMP_SENSOR
	static SeaTempLogFormatter rtd_sensor_log_formatter;
	static FsLog rtd_sensor_log(&lfs_file_system, "RTD", 1024*1024);
	rtd_sensor_log.set_log_formatter(&rtd_sensor_log_formatter);

	DEBUG_TRACE("EZO RTD...");
	try {
		static EZO_RTD_Sensor rtd;
		static SeaTempSensorService rtd_sensor_service(rtd, &rtd_sensor_log);
	} catch (ErrorCode e) {
		DEBUG_INFO("EZO RTD: not detected [%04X]", e);
	}

	static SeaTempLogFormatter tsys01_sensor_log_formatter;
	static FsLog tsys01_sensor_log(&lfs_file_system, "TSYS01", 1024*1024);
	tsys01_sensor_log.set_log_formatter(&tsys01_sensor_log_formatter);

	DEBUG_TRACE("TSYS01...");
	try {
		static TSYS01 tsys01;
		static SeaTempSensorService tsys01_sensor_service(tsys01, &tsys01_sensor_log);
	} catch (ErrorCode e) {
		DEBUG_INFO("TSYS01: not detected [%04X]", e);
	}
#endif

#if ENABLE_AXL_SENSOR
	DEBUG_TRACE("BMA400...");
	static AXLLogFormatter axl_sensor_log_formatter;
	static FsLog axl_sensor_log(&lfs_file_system, "AXL", 1024*1024);
	axl_sensor_log.set_log_formatter(&axl_sensor_log_formatter);
	try {
		static BMA400 bma400;
		static AXLSensorService axl_sensor_service(bma400, &axl_sensor_log);
	} catch (...) {
		DEBUG_INFO("BMA400: not detected");
	}
#endif

#if ENABLE_THERMISTOR_SENSOR
	DEBUG_TRACE("Thermistor NTC...");
	static ThermistorLogFormatter thermistor_sensor_log_formatter;
	static FsLog thermistor_sensor_log(&lfs_file_system, "THERMISTOR", 1024*1024);
	thermistor_sensor_log.set_log_formatter(&thermistor_sensor_log_formatter);
	try {
		static Thermistor thermistor(THERMISTOR_ADC);
		static ThermistorSensorService thermistor_sensor_service(thermistor, &thermistor_sensor_log);
	} catch (...) {
		DEBUG_INFO("Thermistor: not detected");
	}
#endif

#ifdef CAM_PWR_EN
	DEBUG_TRACE("RunCam...");
	static CAMLogFormatter cam_sensor_log_formatter;
	static FsLog cam_sensor_log(&lfs_file_system, "CAM", 1024*1024);
	cam_sensor_log.set_log_formatter(&cam_sensor_log_formatter);
	try {
		static RunCam run_cam;
		static CAMService cam_service(run_cam, &cam_sensor_log);
	} catch (...) {
		DEBUG_INFO("RunCam: not detected");
	}
#endif

#if ENABLE_MORTALITY_SENSOR
	DEBUG_TRACE("Mortality detection...");
	static MortalityLogFormatter mortality_log_formatter;
	static FsLog mortality_log(&lfs_file_system, "MORTALITY", 64*1024);
	mortality_log.set_log_formatter(&mortality_log_formatter);
	static MortalityService mortality_svc(&mortality_log);
	mortality_service = &mortality_svc;
#endif

	(void)lfs_file_system; // Suppress unused warning when no sensors enabled
}


/**
 * @brief Phase 8 — Runtime services (memory monitor, dive mode, shutdown timer).
 *
 * Starts background services that run during the operational lifetime:
 *  - MemoryMonitorService: periodic heap/stack usage reporting.
 *  - DiveModeService: reed-switch-triggered dive/surface mode toggle.
 *  - EXTERNAL_WAKEUP shutdown timer: powers down after configured seconds (TPL5111 boards).
 *
 * @param nrf_reed_switch  Hardware reed switch for DiveModeService.
 */
static void init_runtime(NrfSwitch& nrf_reed_switch)
{
	DEBUG_TRACE("Memory monitor...");
	static MemoryMonitorService memory_monitor_service;

	DEBUG_TRACE("Dive mode monitor...");
	static DiveModeService dive_mode_service(nrf_reed_switch);

	// R4 robustness (2026-05 deep-idle refactor): if the boot was a WDT reset,
	// inhibit the deep-idle fast-path for the first GPS session post-boot.
	// Force cold-cycle to prove the cold path works before re-engaging the
	// optimization. Cleared in GPSService::react(GPSEventPVT) on first fix.
	if (gps_service && PMU::reset_cause() == ResetCause::WDT_RESET) {
		DEBUG_INFO("R4: WDT reset detected — inhibiting deep-idle for first GPS session");
		gps_service->set_deep_idle_inhibit_first_session(true);
	}

#ifdef EXTERNAL_WAKEUP
	// TPL5111 shutdown timer — powerdown after SHUTDOWN_TIMER seconds
	{
		unsigned int shutdown_timer = configuration_store->read_param<unsigned int>(ParamID::SHUTDOWN_TIMER);
		if (shutdown_timer > 0) {
			DEBUG_INFO("EXTERNAL_WAKEUP: Shutdown timer scheduled for %u seconds", shutdown_timer);
			system_scheduler->post_task_prio([]() {
				DEBUG_INFO("EXTERNAL_WAKEUP: Shutdown timer expired | powering down");
				PMU::powerdown();
			}, "SHUTDOWN_TIMER", Scheduler::DEFAULT_PRIORITY, shutdown_timer * 1000);
		} else {
			DEBUG_INFO("EXTERNAL_WAKEUP: Shutdown timer disabled (SHUTDOWN_TIMER=0)");
		}
	}
#endif
}


/**
 * @brief Application entry point.
 *
 * Runs the eight init phases in order, starts the GenTracker FSM, then enters
 * the infinite scheduler loop with deep-idle power management.
 *
 * @note This function never returns.  All init-phase objects are static and
 *       live in .bss — the main() stack frame only holds the loop locals.
 */
int main()
{
	// R1 robustness (2026-05 deep-idle refactor): force the GPS rail LOW
	// before ANYTHING else runs. With deep-idle enabled, the rail can be left
	// ON across hard faults / WDT resets — this 1-instruction invariant cuts
	// it on every boot before init_peripherals even runs. Same for GPS_RST
	// (drive low to assert reset) and GPS_EXT_INT (high-Z so we don't push
	// edges into a still-coming-up M10Q). Uses raw nrf_gpio API since BSP::
	// hasn't been initialized yet at this point.
	{
		auto& pwr_en   = BSP::GPIO_Inits[BSP::GPIO::GPIO_GPS_PWR_EN];
		auto& gps_rst  = BSP::GPIO_Inits[BSP::GPIO::GPIO_GPS_RST];
		auto& ext_int  = BSP::GPIO_Inits[BSP::GPIO::GPIO_GPS_EXT_INT];
		nrf_gpio_cfg_output(pwr_en.pin_number);
		nrf_gpio_pin_clear(pwr_en.pin_number);
		nrf_gpio_cfg_output(gps_rst.pin_number);
		nrf_gpio_pin_clear(gps_rst.pin_number);
		nrf_gpio_cfg_default(ext_int.pin_number);  // high-Z input
	}

	auto [reed, flash] = init_peripherals();
	init_power_on_check(reed);

	auto& lfs = init_storage(reed, flash);
	init_battery();
	init_core_services(lfs, flash);
	init_communication(lfs);
	init_sensors(lfs);
	init_runtime(reed);

	DEBUG_TRACE("Entering main SM...");
	GenTracker::start();

#ifdef BENCH_TEST
	// Bench-test hooks (debug/bench builds only).
#ifdef EXTERNAL_WAKEUP
	// RSPB: no USB, debug UART is TX-only — no interactive console. Auto-inject a
	// fix each (reset-simulated) TPL duty-cycle so the SMD satellite TX runs; the
	// host observes over the 921600 debug UART. Configs are uploaded out-of-band.
	bench::start_auto_inject();
#else
	// LinkIt (KIM/SMD/LoRa): interactive '%'-prefixed USB-CDC console.
	bench::start_poll();
#endif
#endif

	// Power rail management: cut peripheral power rails when no task is due soon.
	// Threshold tuned from the original 5000 ms → 250 ms. The lower the threshold,
	// the more aggressively VSYS toggles 3.3V↔1.8V around short scheduler idles.
	// Rationale per scenario:
	//   - SWS analog detector multi-sample (UW_SAMPLE_GAP = 1000 ms) : both 1000 ms
	//     and 250 ms thresholds let reduce_power_rails() fire between samples.
	//   - SWS underwater fast-sample (period ~500 ms) : 1000 ms threshold blocks
	//     bascule (500 < 1000) → VSYS stuck at 3.3V the whole dive. 250 ms lets
	//     the ~500 ms idle window between samples trigger the bascule.
	//   - Generic services scheduled every 300-900 ms : same story.
	// Cost: each bascule cycle adds ~2 ms PMU::delay_ms in restore_power_rails().
	// At a 500 ms cycle that's 0.4 % time overhead — negligible vs the ~8 µA saved
	// while at 1.8V. The TPS63901 SEL pin supports unlimited switching.
	static constexpr uint64_t IDLE_POWER_SAVE_THRESHOLD_MS = 250;
	static bool power_rails_reduced = false;

#ifndef VALIDATION_LOG_ENABLE
#define VALIDATION_LOG_ENABLE 0
#endif
// #if VALIDATION_LOG_ENABLE
// 	// Sleep-optim cumulative accounting. Reset on first VAL_SLEEP summary tick.
// 	// `reduced_ms` = time spent with VSYS at 1.8 V (peripherals dark).
// 	// `active_ms`  = time spent with VSYS at 3.3 V (running scheduler tasks).
// 	// Ratio reduced/(reduced+active) is the effective sleep-optim duty cycle:
// 	// > 0.9 in steady state on the bench (no surface activity).
// 	static uint64_t s_val_reduced_total_ms = 0;
// 	static uint64_t s_val_active_total_ms = 0;
// 	static uint64_t s_val_last_enter_reduce_ms = 0;
// 	static uint64_t s_val_last_exit_reduce_ms = 0;
// 	static uint64_t s_val_last_summary_ms = 0;
// 	static uint64_t s_val_summary_reduce_count = 0;
// 	static constexpr uint64_t VAL_SLEEP_SUMMARY_PERIOD_MS = 60000;  // 1 min
// #endif

	// The scheduler should run forever.  Any run-time exceptions should be handled and passed to FSM.
	while (true)
	{
		try {
#ifdef DEBUG_UART_TX_PIN
			if (g_debug_mode != BaseDebugMode::UART)
#endif
				NrfUSB::process();

			// Restore power rails BEFORE running scheduler tasks —
			// peripherals need 3.3V before any SPI/I2C/UART access.
			if (power_rails_reduced) {
				uint64_t remaining = system_scheduler->ms_until_next_task();
				if (remaining <= IDLE_POWER_SAVE_THRESHOLD_MS) {
					PMU::restore_power_rails();
					power_rails_reduced = false;
					// DEBUG_TRACE("IDLE_POWER_SAVE: exit (%llu ms remaining)", remaining);  // log retiré : trop verbeux (pollue même en Verbose)
// #if VALIDATION_LOG_ENABLE
// 					uint64_t now_ms = PMU::get_timestamp_ms();
// 					uint64_t this_reduce_ms = (s_val_last_enter_reduce_ms > 0 &&
// 					                           now_ms > s_val_last_enter_reduce_ms)
// 					                          ? (now_ms - s_val_last_enter_reduce_ms) : 0;
// 					s_val_reduced_total_ms += this_reduce_ms;
// 					s_val_last_exit_reduce_ms = now_ms;
// 					DEBUG_INFO("[VAL-SLEEP] exit_reduce reduced_ms=%llu remaining_ms=%llu",
// 					           this_reduce_ms, remaining);
// #endif
				}
			}

			system_scheduler->run();

			// Reduce power rails when no tasks are due for a while
			uint64_t idle_ms = system_scheduler->ms_until_next_task();
			if (idle_ms > IDLE_POWER_SAVE_THRESHOLD_MS && !power_rails_reduced) {
				// DEBUG_TRACE("IDLE_POWER_SAVE: enter (%llu ms idle)", idle_ms);  // log retiré : trop verbeux (pollue même en Verbose)
				power_rails_reduced = true;
				PMU::reduce_power_rails();
// #if VALIDATION_LOG_ENABLE
// 				uint64_t now_ms = PMU::get_timestamp_ms();
// 				uint64_t this_active_ms = (s_val_last_exit_reduce_ms > 0 &&
// 				                            now_ms > s_val_last_exit_reduce_ms)
// 				                          ? (now_ms - s_val_last_exit_reduce_ms) : 0;
// 				s_val_active_total_ms += this_active_ms;
// 				s_val_last_enter_reduce_ms = now_ms;
// 				s_val_summary_reduce_count++;
// 				DEBUG_INFO("[VAL-SLEEP] enter_reduce idle_ms=%llu active_ms=%llu",
// 				           idle_ms, this_active_ms);
// 				// Periodic cumulative summary — emits ratio + total counts.
// 				// Logs every minute regardless of activity, so a flat curve in
// 				// the log timeline reveals a stuck-awake regression.
// 				if (s_val_last_summary_ms == 0) s_val_last_summary_ms = now_ms;
// 				if (now_ms - s_val_last_summary_ms >= VAL_SLEEP_SUMMARY_PERIOD_MS) {
// 					uint64_t total = s_val_reduced_total_ms + s_val_active_total_ms;
// 					unsigned int duty_pct = total > 0
// 					                        ? (unsigned int)((s_val_reduced_total_ms * 100) / total)
// 					                        : 0;
// 					DEBUG_INFO("[VAL-SLEEP] summary reduced_ms=%llu active_ms=%llu duty=%u%% cycles=%llu",
// 					           s_val_reduced_total_ms, s_val_active_total_ms, duty_pct,
// 					           s_val_summary_reduce_count);
// 					s_val_last_summary_ms = now_ms;
// 				}
// #endif
			}

			// Safety watchdog kick — reduces dependency on scheduled task
			PMU::kick_watchdog();

			// Safety net 3.7 (2026-06): check scheduler drop counter and reset
			// if persistent saturation pattern detected. Cheap — usually one
			// integer comparison (no new drops) per loop iteration.
			scheduler_drop_storm_check();

			PMU::run();
		} catch (ErrorCode e) {
			gps_service_exception_storm_check();
			ErrorEvent event;
			event.error_code = e;
			GenTracker::dispatch(event);
		} catch (const std::exception& ex) {
			// std::bad_alloc, std::bad_function_call, std::out_of_range, etc.
			// Without this catch, the exception would reach std::terminate ->
			// __verbose_terminate_handler -> abort() -> hang in fputc until
			// the 15-min watchdog reset cycle. Convert to ErrorEvent so the
			// FSM can react cleanly (and now, via the boot-fail counter,
			// trigger factory_reset retry if these become persistent).
			DEBUG_ERROR("main loop: unhandled std::exception: %s", ex.what());
			gps_service_exception_storm_check();
			ErrorEvent event;
			event.error_code = ErrorCode::RESOURCE_NOT_AVAILABLE;
			GenTracker::dispatch(event);
		} catch (...) {
			// Last-resort catch for SoftDevice asserts, mid-throw exceptions,
			// or non-derived thrown types. Same rationale as above.
			DEBUG_ERROR("main loop: unhandled unknown exception");
			gps_service_exception_storm_check();
			ErrorEvent event;
			event.error_code = ErrorCode::RESOURCE_NOT_AVAILABLE;
			GenTracker::dispatch(event);
		}
	}
}
