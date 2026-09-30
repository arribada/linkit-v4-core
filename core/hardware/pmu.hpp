#pragma once

/**
 * @file pmu.hpp
 * @brief Power Management Unit — watchdog, reset, power-down, deep idle, crash trace.
 */

#include <cstdint>

/// @brief Crash type identifier stored in .noinit RAM for post-reset diagnostics.
enum PMULogType { WDT, HARDFAULT, ETL, MMAN, STACK, MALLOC };

/// @brief Reset cause determined from NRF_POWER->RESETREAS + GPREGRET.
enum class ResetCause {
	POWER_ON,
	HARD_RESET,
	WDT_RESET,
	SOFT_RESET,
	PSEUDO_POWER_ON,
};

class PMU {
private:
	static void watchdog_handler(void);

public:
	static void initialise();
	/// @param dfu_mode  Reserved (not currently used — always does a normal reset).
	static void reset(bool dfu_mode);

	/// @brief Shut down the device — saves RTC, cuts power rails, enters System OFF or infinite sleep.
	static void powerdown();

	/// @brief Enter WFE (Wait For Event) — CPU sleeps until next interrupt/event.
	static void run();

	static void delay_ms(unsigned ms);
	static void delay_us(unsigned us);
	static void start_watchdog();
	static void kick_watchdog();

	static ResetCause reset_cause();
	static const char *reset_cause_str();
	/// @brief Raw RESETREAS captured at boot (before it is cleared), for diagnostics.
	/// Bit 16 = OFF (System OFF wake), 20 = VBUS, 18 = DIF, 3 = LOCKUP, 2 = SREQ,
	/// 1 = DOG, 0 = RESETPIN. Bit 31 is our own storage-marker flag.
	///
	/// The decoded enum is lossy by design, and the storage work showed how much:
	/// a System OFF wake and a battery insertion both read POWER_ON, so a field
	/// return could not be told apart from a fresh install. Logging the registers
	/// makes the boot path recoverable from a single line.
	static uint32_t reset_reason_raw();
	/// @brief Raw GPREGRET captured at boot (before it is cleared).
	static uint32_t boot_gpregret();
	static const char *hardware_version();
	static uint32_t device_identifier();

	/// @brief Capture callstack into .noinit RAM (called from fault handlers).
	/// @param type  Crash type identifier (WDT, HARDFAULT, ETL, etc.).
	static void save_stack(PMULogType type);

	/// @brief Print saved callstack if CRC is valid, then invalidate.
	static void print_stack();

	/// @brief Type of the crash that preceded this boot, latched by print_stack()
	/// before it invalidates the trace, plus the faulting PC.
	///
	/// print_stack() consumes the .noinit trace: it logs it once at boot then
	/// clears it. A board that crashes with no host attached — the ordinary case
	/// on the bench, since USB re-enumerates after the reset — therefore left
	/// nothing to interrogate afterwards, and RESETREAS alone cannot tell a
	/// HardFault from any other soft reset (both report SREQ). This keeps the
	/// verdict readable for the rest of the boot.
	///
	/// @return "none" if this boot followed no recorded crash.
	static const char *last_crash_str();

	static uint64_t get_timestamp_ms();
	static bool was_firmware_updated();

	/// @brief Record (in GPREGRET2, surviving the imminent reset) that an OTA
	/// firmware update was applied, so the next boot can detect it. MUST be used
	/// instead of a direct NRF_POWER->GPREGRET2 write: while the SoftDevice is
	/// enabled (BLE active during OTA) the POWER peripheral is SD-protected and a
	/// direct write faults ("app: Fatal error"). Routes through sd_power_gpregret
	/// when the SD is up. Does NOT affect the bootloader's apply decision (that is
	/// driven by the QSPI staged-image header), only the app's post-boot flag.
	static void set_firmware_updated_flag();

	/// @brief Read the MCU die temperature in °C (whole degrees, sign-preserving).
	/// Falls back to a sentinel safe value (25 °C) when the temperature
	/// source is unavailable, so callers don't need to handle missing data.
	static int get_die_temperature_c();

	/// @brief Cut peripheral power rails when no task is due soon (> 5 s).
	static void reduce_power_rails();

	/// @brief Restore peripheral power rails before next scheduled task.
	static void restore_power_rails();

	/// @brief Storage-mode wake filter (LinkIt V4 only).
	///
	/// Called at the very beginning of `main()` BEFORE `initialise()` and
	/// `start_watchdog()`. Acts on either storage marker in `GPREGRET` — `0x80`
	/// "powerdown just asked for storage", `0x81` "asleep in storage, this boot
	/// is a wake from it" — provided the reset was not RESETPIN or watchdog.
	/// With no magnet held it then:
	///   - Drops `VSYS_SEL` to switch the buck-boost from 3.3 V to 2.3 V
	///     (same trick as `prepare_for_deep_idle`)
	///   - Configures `REED_SW` as a SENSE wake-up source
	///   - Writes `GPREGRET=0x81` (clears only `RESETREAS`) so the SENSE wake
	///     still decodes as `PSEUDO_POWER_ON` and re-arms the 3 s magnet gesture
	///   - Enters `NRF_POWER->SYSTEMOFF` (deepest sleep, wake = chip reset)
	///
	/// With a magnet held the two markers part ways, and that is the point of
	/// having two. On `0x81` the magnet is the operator waking a stored tag, so
	/// this returns and lets the normal `init_power_on_check` 3 s gesture decide.
	/// On `0x80` the magnet is the one that just confirmed the power-off and is
	/// still on the case; returning would let it satisfy that same 3 s hold ~9.5 s
	/// later and turn the power-off into a power-on, so this waits (bounded, with
	/// a magenta blink asking for the magnet) for it to be taken away first.
	///
	/// Returns immediately if neither marker is present. Must be called before
	/// `initialise()` because:
	///   - `initialise()` clears `RESETREAS` / `GPREGRET`
	///   - The watchdog must NOT be running before System OFF (it would
	///     tick in System OFF and reset the chip on timeout)
	static void storage_off_check();
};
