#pragma once

/**
 * @file gpio.hpp
 * @brief GPIO abstraction and VSENSORS reference-counted power management.
 *
 * GPIOPins wraps the nRF GPIO HAL with BSP pin mapping.  Pin indices
 * correspond to BSP::GPIO enum values — never raw nRF pin numbers.
 *
 * VSENSORS power rail is reference-counted: multiple sensors can
 * acquire power independently; the rail stays on until ALL release.
 */

#include <cstdint>

class GPIOPins {
public:
	/// @brief Configure all GPIO pins from the BSP init table.
	static void initialise();

	/// @brief Re-apply BSP config for a single pin.
	static void init_pin(uint32_t pin);

	/// @name Basic pin operations (pin = BSP::GPIO enum index)
	/// @{
	static void set(uint32_t pin);
	static void clear(uint32_t pin);
	static void toggle(uint32_t pin);
	static uint32_t value(uint32_t pin);
	static void disable(uint32_t pin);  ///< Float pin (nrf_gpio_cfg_default)
	static void enable(uint32_t pin);   ///< Restore BSP configuration
	/// @}

	/// @name VSENSORS power management (reference-counted)
	/// @{
	static void acquire_sensors_pwr();  ///< Increment refcount, power on if was 0
	static void release_sensors_pwr();  ///< Decrement refcount, power off if reaches 0
	static bool get_sensors_pwr_state();
	static uint8_t get_sensors_pwr_refcount();
	/// @brief "The GNSS UART is in service" interlock (2026-08).
	///
	/// PMU::reduce_power_rails() drops VSYS to 2.3 V as soon as the scheduler is
	/// idle for more than 250 ms. In the `receive` state the only queued task is
	/// the 5 s NAV watchdog, so the rail toggles between every NAV-PVT, with the
	/// GNSS rail powered and the UART at 460800. The existing interlock
	/// (`!get_sensors_pwr_state()`) is DEAD code on LinkIt, where SENSORS_PWR_PIN
	/// does not exist and the refcount stays at 0 for life.
	///
	/// Taken and released by UBXComms::init()/deinit(), i.e. exactly the lifetime
	/// of the UART -- one single point, impossible to desynchronise.
	static void set_gnss_uart_active(bool active);
	static bool is_gnss_uart_active();

	/// @brief Force a VSENSORS off→on power-cycle to release a wedged I2C slave,
	/// WITHOUT changing the reference count (the rail ends powered). Used by the
	/// I2C bus-recovery path. No-op if the rail is currently off. Resets every
	/// sensor on VSENSORS (driving SCL/SDA low during the off-window so the
	/// always-on bus pull-ups can't back-power them); it does NOT reset devices on
	/// VBAT (e.g. the STC3117 gauge — only a full board power cycle clears those).
	static void power_cycle_sensors();
	/// @}

	/**
	 * @brief Drive pin LOW for duration_ms, then release to high-impedance.
	 * @note Useful for SAT_RESET: allows nRF to reset SMD while keeping pin
	 *       free for external probe flashing.
	 */
	static void pulse_low_then_release(uint32_t pin, uint32_t duration_ms);

	/// @brief Configure pin as output and drive LOW.
	static void drive_low(uint32_t pin);

	/// @brief Release pin to high-impedance (input/disconnected).
	static void release_to_highz(uint32_t pin);

	/// @brief Release an open-drain output while HOLDING a defined high level,
	///        through the internal pull-up (~13 kOhm).
	///
	/// release_to_highz() goes through nrf_gpio_cfg_default(), which returns the pin
	/// to an input WITHOUT a pull: the line floats. That is acceptable where an
	/// external pull exists, not where there is none. Use case: the KIM2 module's
	/// USR_NRST pin, which its datasheet declares "should not be left floating" and
	/// for which the linkit-v4 board provides no external pull. It stays an input
	/// (rather than a driven output) so an SWD probe wired on the same net can still
	/// pull the reset without a conflict.
	static void release_to_pullup(uint32_t pin);

private:
	static uint8_t m_sensors_pwr_refcount;
	static bool m_gnss_uart_active;

	/// @brief Disconnect sensor I2C/interrupt pins to prevent backfeed when VSENSORS off.
	static void disconnect_sensor_pins();

	/// @brief Reconnect sensor interrupt pins after VSENSORS powers on.
	static void reconnect_sensor_pins();
};

/**
 * @brief RAII guard for VSENSORS power — automatically releases on scope exit.
 *
 * Usage:
 * @code
 *   {
 *       SensorsPowerGuard guard;
 *       // ... use I2C sensors ...
 *   }  // VSENSORS released here if refcount reaches 0
 * @endcode
 */
class SensorsPowerGuard {
public:
	SensorsPowerGuard() { GPIOPins::acquire_sensors_pwr(); }
	~SensorsPowerGuard() { GPIOPins::release_sensors_pwr(); }
	SensorsPowerGuard(const SensorsPowerGuard &) = delete;
	SensorsPowerGuard &operator=(const SensorsPowerGuard &) = delete;
};
