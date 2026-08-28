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
	static const char *hardware_version();
	static uint32_t device_identifier();

	/// @brief Capture callstack into .noinit RAM (called from fault handlers).
	/// @param type  Crash type identifier (WDT, HARDFAULT, ETL, etc.).
	static void save_stack(PMULogType type);

	/// @brief Print saved callstack if CRC is valid, then invalidate.
	static void print_stack();

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
	/// `start_watchdog()`. If the device just woke from `PSEUDO_POWER_OFF`
	/// (soft reset with `GPREGRET=0x80`) AND no magnet is currently held
	/// on the reed switch, this function:
	///   - Drops `VSYS_SEL` to switch the buck-boost from 3.3 V to 1.8 V
	///     (same trick as `prepare_for_deep_idle`)
	///   - Configures `REED_SW` as a SENSE wake-up source
	///   - Clears `GPREGRET` / `RESETREAS` so the next wake reports cleanly
	///   - Enters `NRF_POWER->SYSTEMOFF` (deepest sleep, wake = chip reset)
	///
	/// Returns immediately if reset cause is not pseudo-power-off, or if the
	/// magnet IS held (lets the normal `init_power_on_check` 3 s gesture
	/// take over). Must be called before `initialise()` because:
	///   - `initialise()` clears `RESETREAS` / `GPREGRET`
	///   - The watchdog must NOT be running before System OFF (it would
	///     tick in System OFF and reset the chip on timeout)
	static void storage_off_check();
};
