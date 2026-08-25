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
	static void disable(uint32_t pin);   ///< Float pin (nrf_gpio_cfg_default)
	static void enable(uint32_t pin);    ///< Restore BSP configuration
	/// @}

	/// @name VSENSORS power management (reference-counted)
	/// @{
	static void acquire_sensors_pwr();   ///< Increment refcount, power on if was 0
	static void release_sensors_pwr();   ///< Decrement refcount, power off if reaches 0
	static bool get_sensors_pwr_state();
	static uint8_t get_sensors_pwr_refcount();
	/// @brief Verrou "l'UART GNSS est en service" (2026-08).
	///
	/// PMU::reduce_power_rails() fait descendre VSYS a 2,3 V des que le
	/// scheduler est libre plus de 250 ms. En etat `receive` la seule tache en
	/// file est le chien de garde NAV de 5 s: la bascule tombe donc entre
	/// chaque NAV-PVT, rail GNSS allume et UART a 460800. L'interverrouillage
	/// existant (`!get_sensors_pwr_state()`) est du code MORT sur LinkIt, ou
	/// SENSORS_PWR_PIN n'existe pas et le refcount reste a 0 a vie.
	///
	/// Pose/retire par UBXComms::init()/deinit(), c'est-a-dire exactement la
	/// duree de vie de l'UART — un seul point, impossible a desynchroniser.
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
	SensorsPowerGuard(const SensorsPowerGuard&) = delete;
	SensorsPowerGuard& operator=(const SensorsPowerGuard&) = delete;
};
