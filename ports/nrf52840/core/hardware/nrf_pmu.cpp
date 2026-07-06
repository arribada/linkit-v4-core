/**
 * @file nrf_pmu.cpp
 * @brief nRF52840 PMU — watchdog, reset cause, power-down, deep idle, crash trace.
 */

#include <cstring>
#include "bsp.hpp"
#include "pmu.hpp"
#include "gpio.hpp"
#include "nrf_timer.hpp"
#include "nrf_nvic.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_delay.h"
#include "nrfx_wdt.h"
#include "nrf_power.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_soc.h"
#include "nrfx_twim.h"
#include "cm_backtrace.h"
#include "debug.hpp"
#include "nrf_rgb_led.hpp"
#include "crc16.h"

#include "rtc.hpp"
#include "config_store.hpp"
#include "service.hpp"
extern RTC *rtc;
extern ConfigurationStore *configuration_store;
extern RGBLed *status_led;

static uint32_t m_reset_cause = 0;  ///< Raw RESETREAS register + pseudo power-off flag

#ifdef SOFTDEVICE_PRESENT
/// @brief SoftDevice power-failure warning handler — captures cooldown state before brown-out.
///
/// On `NRF_EVT_POWER_FAILURE_WARNING` (POFCON V27 = 2.7 V on Vbatt) the device has only the
/// supercap energy left. The previous implementation called `configuration_store->save_params()`
/// here, which performs an LFS journal write of ~100 ms. If the brown-out is caused by a deep
/// discharge (supercap drained), that write can be truncated and corrupt the params block.
///
/// LFS is journaling, so the file system itself doesn't brick, but partial writes can lose the
/// last 30 minutes of param updates AND in rare cases leave a half-written record that the
/// reader then rejects on next boot.
///
/// New policy:
///   - Save ONLY cooldown state to `.noinit` RAM (atomic, no flash, ~µs)
///   - Capture RTC into the param store IN RAM (write_param updates RAM only, no flash)
///   - Skip the flash sync entirely — rely on the 30-min periodic flush (gentracker.cpp:202)
///     and the clean-shutdown save in `PMU::powerdown()` for durable persistence
///
/// Net effect: on POF, the device loses up to 30 min of param updates (acceptable) but never
/// corrupts the params block (catastrophic).
static void pof_soc_evt_handler(uint32_t evt_id, void * p_context) {
	(void)p_context;
	if (evt_id == NRF_EVT_POWER_FAILURE_WARNING) {
		ServiceManager::save_cooldown_state();
		// write_param updates RAM only — durable persistence depends on the
		// next clean-shutdown save_params or periodic flush. Trade-off
		// accepted: lose <=30 min of RTC vs. risk corrupting all 227 params.
		if (configuration_store && rtc && rtc->is_set()) {
			configuration_store->write_param(ParamID::LAST_KNOWN_RTC,
				static_cast<unsigned int>(rtc->gettime()));
		}
	}
}
NRF_SDH_SOC_OBSERVER(m_pof_soc_observer, 0, pof_soc_evt_handler, NULL);
#endif
static bool m_firmware_was_updated = false;  ///< Set if GPREGRET2 was 0x01 at boot (OTA applied)

/// @name Crash trace storage (.noinit RAM — survives soft reset, not power-on)
/// @{
static __attribute__((section(".noinit"))) volatile uint32_t m_callstack[8];  ///< Saved PC backtrace
static __attribute__((section(".noinit"))) volatile PMULogType m_type;        ///< Crash type
static __attribute__((section(".noinit"))) volatile uint16_t m_crc;           ///< CRC16 guard
/// @}

/// @brief Spare bit in RESETREAS to distinguish pseudo power-off (via GPREGRET) from real SREQ.
#define POWER_RESETREAS_PSEUDO_POWER_OFF  0x80000000

/// @brief Storage-mode wake filter — see header for full rationale.
/// Must be called BEFORE initialise() (which clears RESETREAS / GPREGRET)
/// and BEFORE start_watchdog() (WDT keeps running in System OFF and would
/// reset the chip on timeout, defeating the purpose).
void PMU::storage_off_check() {
#ifdef PSEUDO_POWER_OFF
	// Read RESETREAS / GPREGRET directly — initialise() hasn't cleared them yet.
	uint32_t resetreas = NRF_POWER->RESETREAS;
	uint32_t gpregret  = NRF_POWER->GPREGRET;

	bool was_pseudo_power_off =
		(resetreas & POWER_RESETREAS_SREQ_Msk) && (gpregret == 0x80);

	if (!was_pseudo_power_off) {
		return;  // Not a wake from storage — proceed to normal init
	}

	// Apply BSP config for the reed pin (PULLDOWN input on LinkIt V4).
	GPIOPins::init_pin(BSP::GPIO_REED_SW);

	// Tiny settle delay — pulldown needs a few µs to discharge the pin cap.
	nrf_delay_us(50);

	if (GPIOPins::value(BSP::GPIO_REED_SW) == REED_SWITCH_ACTIVE_STATE) {
		// Magnet held at boot — likely a real user gesture. Let the normal
		// init_power_on_check flow take over (3 s held-check + buzzer + LED).
		return;
	}

	// No magnet at boot. We're going to System OFF. First, drop VSYS_SEL
	// so the TPS63901 buck-boost outputs 1.8 V instead of 3.3 V. Same trick
	// as prepare_for_deep_idle (see this file ~L348) — saves ~8 µA of
	// quiescent. Safe here because:
	//   - All other peripheral rails (GPS / SAT / sensors) were cut by
	//     powerdown() before the soft reset that brought us here.
	//   - The nRF52840 operates from 1.7 V to 3.6 V on VDD (datasheet).
	//   - LED is off in storage state, so the ~3 V Vf is not required.
	GPIOPins::init_pin(BSP::GPIO_VSYS_SEL);     // apply BSP config (open-drain S0D1)
	GPIOPins::clear(BSP::GPIO_VSYS_SEL);        // SEL low → VSYS = 1.8 V

	// Configure REED_SW as a SENSE wake source (PORT event on level match).
	// init_pin set it to GPIOTE TOGGLE in the BSP — we override with SENSE
	// since GPIOTE peripherals stop in System OFF, only SENSE wakes.
	nrf_gpio_cfg_sense_input(BSP::GPIO_Inits[BSP::GPIO_REED_SW].pin_number,
		NRF_GPIO_PIN_PULLDOWN,
		(REED_SWITCH_ACTIVE_STATE != 0) ? NRF_GPIO_PIN_SENSE_HIGH
		                                : NRF_GPIO_PIN_SENSE_LOW);

	// Clear retention registers so the next wake reports the true reset cause
	// (POWER_ON for battery insert, or GPIO for the SENSE wake we just armed).
	// Without this clear, every cold boot via Hall would re-detect
	// "PSEUDO_POWER_OFF + magnet released" and bounce back into System OFF.
	NRF_POWER->GPREGRET = 0;
	NRF_POWER->RESETREAS = NRF_POWER->RESETREAS;  // write-1-to-clear

	// Clear any latched DETECT events on both GPIO ports. Without this clear,
	// stale latch bits from before the reset can interfere with the SENSE
	// wake mechanism — particularly important since we just reconfigured the
	// reed pin SENSE polarity.
	NRF_P0->LATCH = NRF_P0->LATCH;  // w1c — clears all latched bits
	NRF_P1->LATCH = NRF_P1->LATCH;

	// Enter System OFF. The CPU is powered off — instructions after this
	// write may still execute briefly (until the power gate cuts the CPU
	// supply) but their side effects are lost.
	//
	// IMPORTANT: do NOT precede this with __WFE() — WFE would put the CPU
	// to sleep BEFORE the SYSTEMOFF write, which means the write never
	// happens. The chip would just sit in System ON Idle (CPU off but
	// peripherals on, ~10 µA at 1.8 V) instead of true System OFF (~0.5 µA).
	NRF_POWER->SYSTEMOFF = 1;
	__DSB();  // make sure the write completes before the for-loop below

	// Failsafe: if SYSTEMOFF was rejected by hardware (e.g. NFC field detect
	// active, debug interface connected), drop into a tight WFI loop with
	// VSYS at 1.8 V — minimises wasted current until the next reset or wake.
	for (;;) { __WFI(); }
#endif // PSEUDO_POWER_OFF
}

/// @brief Read reset cause, configure DCDC, power-on-failure threshold, clear retention registers.
void PMU::initialise() {
#ifdef POWER_CONTROL_PIN
	GPIOPins::set(BSP::GPIO_POWER_CONTROL);
#endif

	m_reset_cause = NRF_POWER->RESETREAS;
	NRF_POWER->RESETREAS = 0xFFFFFFFF; // Clear down

	// Apply pseudo power off flag if GPREGRET is set
	if (NRF_POWER->GPREGRET)
		m_reset_cause |= POWER_RESETREAS_PSEUDO_POWER_OFF;
	else
		m_reset_cause &= ~POWER_RESETREAS_PSEUDO_POWER_OFF;
	NRF_POWER->GPREGRET = 0; // Clear down

	// Check GPREGRET2 for firmware update flag (set before OTA reset)
	m_firmware_was_updated = (NRF_POWER->GPREGRET2 == 0x01);
	NRF_POWER->GPREGRET2 = 0; // Clear down

	nrf_pwr_mgmt_init();

	NRF_POWER->POFCON = (POWER_POFCON_POF_Enabled << POWER_POFCON_POF_Pos) |
	                     (POWER_POFCON_THRESHOLD_V27 << POWER_POFCON_THRESHOLD_Pos);

#ifdef SOFTDEVICE_PRESENT
	if (nrf_sdh_is_enabled())
	{
		sd_power_dcdc0_mode_set(NRF_POWER_DCDC_ENABLE);
		sd_power_pof_enable(true);
		sd_power_pof_threshold_set(NRF_POWER_THRESHOLD_V27);
	}
	else
#endif
	{
		nrf_power_dcdcen_set(true);
	}
}

/// @brief Trigger an immediate system reset via the SoftDevice NVIC.
void PMU::reset(bool /*dfu_mode*/) {
	sd_nvic_SystemReset();
}

/// @brief Set GPREGRET2 bit0 (OTA-applied flag) in an SD-safe way.
void PMU::set_firmware_updated_flag() {
#ifdef SOFTDEVICE_PRESENT
	if (nrf_sdh_is_enabled()) {
		// POWER is SD-protected while the SoftDevice is enabled — a direct
		// NRF_POWER->GPREGRET2 write faults (HardFault -> "app: Fatal error").
		// Go through the SoftDevice API instead.
		sd_power_gpregret_clr(1, 0xFF);
		sd_power_gpregret_set(1, 0x01);
		return;
	}
#endif
	NRF_POWER->GPREGRET2 = 0x01;
}

// Pre-deploy validation channel — see hauled_mode_service.cpp header comment.
// Marks terminal sleep (powerdown / System OFF). Default off (zero overhead).
#ifndef VALIDATION_LOG_ENABLE
#define VALIDATION_LOG_ENABLE 0
#endif

/// @brief Shut down device — save state, cut power rails, then reset or infinite sleep.
void PMU::powerdown() {
// #if VALIDATION_LOG_ENABLE
// 	DEBUG_INFO("[VAL-SLEEP] powerdown t=%u uptime_ms=%llu",
// 	           (rtc && rtc->is_set()) ? (unsigned int)rtc->gettime() : 0,
// 	           (unsigned long long)PMU::get_timestamp_ms());
// #endif
	// Ensure all power control pins are turned off before shutdown
// #ifdef CAM_PWR_EN
// 	DEBUG_TRACE("Powering off CAM");
// 	GPIOPins::clear(CAM_PWR_EN);
// 	GPIOPins::clear(CAM_PWR_BUTT);
// #endif

	// Persist cooldown state to noinit RAM before shutdown
	ServiceManager::save_cooldown_state();

	// Persist current RTC for pseudo RTC chain on next boot
	if (configuration_store && rtc && rtc->is_set()) {
		configuration_store->write_param(ParamID::LAST_KNOWN_RTC,
			static_cast<unsigned int>(rtc->gettime()));
		configuration_store->save_params();
		DEBUG_TRACE("PMU::powerdown: Saved LAST_KNOWN_RTC = %u",
			static_cast<unsigned int>(rtc->gettime()));
	}

#ifdef BENCH_TEST
	// Bench: EVERY power-off path below cuts VDD and we lose host/UART/SWD access.
	// On RSPB the TPL5111 latch drops power the moment the MCU stops holding it
	// (MCU_DONE pulse OR the brown-out that follows a soft reset on a marginal
	// rail), so even a "simulated wake" soft reset loses the board. Instead,
	// SUPPRESS powerdown entirely: keep the MCU running so the TPL keeps power
	// latched, leave MCU_DONE de-asserted, and return so the scheduler keeps
	// driving GNSS + Argos-TX cycles. State was already persisted above. The
	// board never powers down and stays reachable indefinitely for validation.
	DEBUG_WARN("PMU::powerdown: BENCH_TEST — powerdown SUPPRESSED (staying alive, TPL power held)");
#ifdef MCU_DONE_PIN
	GPIOPins::clear(MCU_DONE_PIN);  // ensure DONE stays de-asserted (high-Z on D0S1)
#endif
	return;
#endif

	// Shut down all peripherals before power-off (all boards)
	NrfRGBLed::set_color_raw(BSP::GPIO::GPIO_LED_RED, BSP::GPIO::GPIO_LED_GREEN, BSP::GPIO::GPIO_LED_BLUE, RGBLedColor::BLACK);
#ifdef SENSORS_PWR_PIN
	GPIOPins::clear(SENSORS_PWR_PIN);
#endif
#ifdef GPS_POWER
	GPIOPins::clear(GPS_POWER);
#endif
#ifdef GPS_RST
	GPIOPins::set(GPS_RST);
#endif
#ifdef SAT_PWR_EN
	GPIOPins::clear(SAT_PWR_EN);
#endif
#ifdef SAT_RESET
	GPIOPins::set(SAT_RESET);
#endif
#ifdef SMD_VPA_PIN
	GPIOPins::drive_low(SMD_VPA_PIN);
#endif
#ifdef SWS_ENABLE_PIN
	GPIOPins::clear(SWS_ENABLE_PIN);
#endif

#ifdef VSYS_SEL
	GPIOPins::clear(VSYS_SEL);
#endif

#if defined(POWER_CONTROL_PIN)
	// Cut power control pin — on LinkIt V4 this de-asserts the VSYS latch.
	// If the hardware pull-up keeps it HIGH (pseudo power-off), the soft reset
	// below handles the actual shutdown. On boards with a real load switch,
	// this cuts VDD directly.
	DEBUG_TRACE("Attempt power off using power control pin");
	GPIOPins::clear(POWER_CONTROL_PIN);
	PMU::delay_ms(1000);  // Wait for load switch to take effect (if real power-off)
#endif

#if defined(PSEUDO_POWER_OFF)
	DEBUG_TRACE("Pseudo power off (soft reset with GPREGRET)");
	NRF_POWER->GPREGRET = 0x80;
	sd_nvic_SystemReset();
#endif

#if defined(EXTERNAL_WAKEUP) && defined(MCU_DONE_PIN)
	// Signal TPL5111 to cut power by pulsing MCU_DONE
	DEBUG_TRACE("External wakeup enabled | set MCU_DONE and wait for TPL5111 power cut");
	GPIOPins::set(MCU_DONE_PIN);
	PMU::delay_ms(100); // Allow time for TPL5111 to respond
	GPIOPins::clear(MCU_DONE_PIN);
#endif

	// Fallback: if no power-off mechanism succeeded, sleep briefly then reset.
	// On RSPB, TPL5111 should have cut power by now — if not, a reset will
	// retry the boot sequence (modulo check will powerdown again).
	DEBUG_WARN("PMU::powerdown: No power-off mechanism took effect — resetting in 2s");
	PMU::delay_ms(2000);
	sd_nvic_SystemReset();
}

/// @brief Enter WFE (Wait For Event) — CPU sleeps until next interrupt.
void PMU::run() {
	nrf_pwr_mgmt_run();
}

void PMU::delay_ms(unsigned ms)
{
	nrf_delay_ms(ms);
}

void PMU::delay_us(unsigned us)
{
	nrf_delay_us(us);
}

/// @brief Init WDT, allocate one channel, enable.  Also inits CmBacktrace for crash diagnostics.
void PMU::start_watchdog()
{
	nrfx_wdt_init(&BSP::WDT_Inits[BSP::WDT].config, PMU::watchdog_handler);
	// Channel ID is discarded as we don't care which channel ID is allocated
	nrfx_wdt_channel_id id;
	nrfx_wdt_channel_alloc(&id);
	nrfx_wdt_enable();
	cm_backtrace_init("", "", "");
}

/// @brief Feed all WDT channels to prevent reset.
void PMU::kick_watchdog()
{
	nrfx_wdt_feed();
}

/// @brief Decode RESETREAS register into a ResetCause enum.
ResetCause PMU::reset_cause()
{
	if (m_reset_cause & NRF_POWER_RESETREAS_RESETPIN_MASK)
		return ResetCause::HARD_RESET;
	else if (m_reset_cause & NRF_POWER_RESETREAS_DOG_MASK)
		return ResetCause::WDT_RESET;
	else if (m_reset_cause & NRF_POWER_RESETREAS_SREQ_MASK)
		if (m_reset_cause & POWER_RESETREAS_PSEUDO_POWER_OFF)
			return ResetCause::PSEUDO_POWER_ON;
		else
			return ResetCause::SOFT_RESET;
	else
		return ResetCause::POWER_ON;
}

/// @brief Human-readable string for the current reset cause.
const char* PMU::reset_cause_str()
{
	switch (reset_cause()) {
	case ResetCause::HARD_RESET:      return "Hard Reset";
	case ResetCause::WDT_RESET:       return "WDT Reset";
	case ResetCause::SOFT_RESET:      return "Soft Reset";
	case ResetCause::PSEUDO_POWER_ON: return "Pseudo Power On Reset";
	case ResetCause::POWER_ON:        return "Power On Reset";
	default:                          return "Unknown";
	}
}

/// @brief Return the first 32 bits of the nRF FICR device ID (unique per chip).
uint32_t PMU::device_identifier()
{
	return NRF_FICR->DEVICEID[0];
}

/// @brief Board + variant identifier string (compile-time, e.g. "linkit-v4-smd").
const char *PMU::hardware_version()
{
#if defined(BOARD_RSPB)
	return "rspb-v1";
#elif defined(ARGOS_SMD) && (ARGOS_SMD == 1)
	return "linkit-v4-smd";
#elif defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
	return "linkit-v4-lora";
#else
	return "linkit-v4-kim";
#endif
}

/// @brief WDT ISR callback — captures callstack before reset.
void PMU::watchdog_handler() {
	save_stack(PMULogType::WDT);
}

/// @brief Capture callstack via CmBacktrace into .noinit RAM + CRC16.
void PMU::save_stack(PMULogType type) {
	m_type = type;
	uint32_t lr = reinterpret_cast<uint32_t>(__builtin_return_address(0));
	uint32_t sp = reinterpret_cast<uint32_t>(__builtin_frame_address(0));
	cm_backtrace_fault(lr, sp, const_cast<uint32_t *>(m_callstack), sizeof(m_callstack) / sizeof(m_callstack[0]));
	m_crc = crc16_compute(reinterpret_cast<const uint8_t *>(const_cast<const uint32_t *>(m_callstack)), sizeof(m_callstack), nullptr);
}

/// @brief Convert PMULogType crash enum to a printable string.
static const char *reset_type_to_string(PMULogType t) {
	switch (t) {
	case PMULogType::WDT:
		return "WDT";
	case PMULogType::HARDFAULT:
		return "HARDFAULT";
	case PMULogType::ETL:
		return "ETL";
	case PMULogType::MMAN:
		return "MMAN";
	case PMULogType::STACK:
		return "STACK";
	case PMULogType::MALLOC:
		return "MALLOC";
	default:
		return "UNKNOWN";
	}
}

/// @brief Milliseconds since boot via NrfTimer.
uint64_t PMU::get_timestamp_ms() {
	return NrfTimer::get_instance().get_counter();
}

/// @brief Read MCU die temperature in °C (whole degrees, sign-preserving).
/// Uses sd_temp_get() — SoftDevice returns temperature in 0.25 °C steps.
/// Returns a 25 °C sentinel on error so callers can treat the read as
/// non-blocking ("if temp is low enough to matter we'll know; if the read
/// fails we behave as if warm").
int PMU::get_die_temperature_c() {
	int32_t raw = 0;
	uint32_t err = sd_temp_get(&raw);
	if (err != NRF_SUCCESS)
		return 25;
	// raw is in 0.25 °C steps — divide rounding toward zero is fine here,
	// since callers compare against a coarse threshold (5 °C).
	return raw / 4;
}

/// @brief Print saved crash trace if CRC is valid, then invalidate to avoid re-printing.
void PMU::print_stack() {
	// Check CRC matches
	if (m_crc == crc16_compute(reinterpret_cast<const uint8_t *>(const_cast<const uint32_t *>(m_callstack)), sizeof(m_callstack), nullptr))
	{
		DEBUG_INFO("PMU post-reset trace available");
		DEBUG_INFO("PMU reset type: %s", reset_type_to_string(m_type));
		for (unsigned int i = 0; i < (sizeof(m_callstack) / sizeof(m_callstack[0])); i++)
			DEBUG_INFO("PMU PC[%u] = %08x", i, static_cast<unsigned int>(m_callstack[i]));
	} else {
		DEBUG_TRACE("PMU post-reset trace unavailable (crc=%04x)", static_cast<unsigned int>(m_crc));
	}

	m_crc = 0;
	memset(const_cast<uint32_t *>(m_callstack), 0, sizeof(m_callstack));
}

/// @brief True if GPREGRET2 indicated a firmware update was applied before this boot.
bool PMU::was_firmware_updated() {
	return m_firmware_was_updated;
}

/// @brief Cut peripheral power rails during idle (VSYS → 1.8V, POWER_CONTROL off).
void PMU::reduce_power_rails() {
	// FIXME (RSPB only): External I2C pull-ups R21/R24 (4.7K) are connected to
	// DCDC_3V3 instead of VSENSORS. When VSENSORS is OFF, ~1.3mA backfeeds
	// through sensor ESD diodes into the unpowered sensor side.
	// Workaround: keep SENSORS_PWR_PIN always ON while nRF is running.
	// This wastes ~50µA idle on the I2C sensors but avoids the 1.3mA backfeed.
	// Fix in next PCB revision: connect R21/R24 to VSENSORS rail.
#if defined(BOARD_RSPB) && defined(SENSORS_PWR_PIN)
	if (!GPIOPins::get_sensors_pwr_state())
		GPIOPins::set(SENSORS_PWR_PIN);
#endif

	// RSPB: POWER_CONTROL_PIN controls the main board power rail (not just SMD).
	// Clearing it in deep idle saves ~0.5-0.9mA from the STM32WL SMD idle current
	// plus any other peripherals on the rail. The TPL5111 also manages this pin,
	// but we get better power savings by cutting it ourselves during idle periods.
#if defined(BOARD_RSPB) && defined(POWER_CONTROL_PIN)
	GPIOPins::clear(POWER_CONTROL_PIN);
#endif

	// LinkIt V4: Switch VSYS to 2.3V during deep idle to reduce nRF52840 core
	// and DCDC quiescent current. Only safe when all peripherals are powered off
	// (GPS, SMD, sensors all use separate power rails at 3.3V and are not affected).
	// The nRF52840 DCDC operates down to 1.8V with full functionality (CPU, RAM, BLE, RTC).
#if defined(VSYS_SEL) && !defined(BOARD_RSPB)
	// Only switch to 2.3V if LED is off — LED forward voltage (~3V) requires 3.3V rail
	if (!GPIOPins::get_sensors_pwr_state() &&
	    status_led && status_led->get_state() == RGBLedColor::BLACK && !status_led->is_flashing()) {
		// Lower the POF brownout threshold BELOW the idle rail BEFORE dropping VSYS:
		// POFCON is armed at 2.7V, but the idle rail is 2.3V, so at 2.7V the comparator
		// would assert POFWARN continuously in deep idle (CPU wakes / cooldown-save churn,
		// and pre-fix a HardFault via the NULL nrf_drv_power handler). restore_power_rails()
		// raises it back to 2.7V once VSYS is back at 3.3V (where brownout-under-load matters).
#ifdef SOFTDEVICE_PRESENT
		if (nrf_sdh_is_enabled())
			sd_power_pof_threshold_set(NRF_POWER_THRESHOLD_V20);  // 2.0 V (< 2.3 V idle rail)
#endif
		GPIOPins::clear(VSYS_SEL);  // Switch to 2.3V
	}
#endif
}

/// @brief Restore peripheral power rails before scheduled tasks (VSYS → 3.3V).
void PMU::restore_power_rails() {
	// LinkIt V4: Restore VSYS to 3.3V before any peripheral access.
	// Must settle before SPI/I2C/UART transactions with 3.3V peripherals.
#if defined(VSYS_SEL) && !defined(BOARD_RSPB)
	GPIOPins::set(VSYS_SEL);  // Switch to 3.3V
	PMU::delay_ms(2);  // Allow VSYS rail to stabilize from 2.3V → 3.3V
	// Rail is back at 3.3V: restore the full 2.7V POF brownout threshold
	// (lowered to 2.0V in reduce_power_rails for the 2.3V idle rail).
#ifdef SOFTDEVICE_PRESENT
	if (nrf_sdh_is_enabled())
		sd_power_pof_threshold_set(NRF_POWER_THRESHOLD_V27);  // back to 2.7 V
#endif
#endif

	// RSPB: restore board power rail before any peripheral access
#if defined(BOARD_RSPB) && defined(POWER_CONTROL_PIN)
	GPIOPins::set(POWER_CONTROL_PIN);
	PMU::delay_ms(10);  // Allow power rail stabilization before SPI/I2C access
#endif
	// RSPB: SENSORS_PWR stays ON (never cut while nRF is running — see FIXME above)
}
