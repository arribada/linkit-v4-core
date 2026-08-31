/**
 * @file nrf_gpio.cpp
 * @brief nRF52840 GPIO implementation with VSENSORS power management.
 */

#include <cstdint>

#include "gpio.hpp"
#include "bsp.hpp"
#include "debug.hpp"
#include "nrf_delay.h"
#include "nrf_i2c.hpp"

uint8_t GPIOPins::m_sensors_pwr_refcount = 0;
bool GPIOPins::m_gnss_uart_active = false;
bool GPIOPins::m_flash_busy = false;
bool GPIOPins::m_config_mode_active = false;

// Build-time flag (build_rspb.sh: RSPB_VSENSORS_FLOOR, default 1).
//   1 = keep the RSPB VSENSORS refcount floor: never cut VSENSORS, since the I2C
//       pull-ups (R21/R24) live on it and cutting it kills the whole bus.
//   0 = allow VSENSORS to be cut — to test whether the pull-ups really depend on
//       VSENSORS (bus dies on cut) or whether the earlier failures were purely the
//       log flood (bus survives).
#ifndef RSPB_VSENSORS_FLOOR
#define RSPB_VSENSORS_FLOOR 1
#endif

/// @brief Configure all BSP GPIO pins and set safe initial state for all power rails.
void GPIOPins::initialise() {
	for (unsigned int i = 0; i < static_cast<unsigned int>(BSP::GPIO::GPIO_TOTAL_NUMBER); i++) {
		nrf_gpio_cfg(BSP::GPIO_Inits[i].pin_number, BSP::GPIO_Inits[i].dir, BSP::GPIO_Inits[i].input,
		             BSP::GPIO_Inits[i].pull, BSP::GPIO_Inits[i].drive, BSP::GPIO_Inits[i].sense);
	}

	// Tri-state UART pins — controlled at driver level, not GPIO init
	nrf_gpio_cfg_default(BSP::UARTAsync_Inits[0].config.rx_pin);
	nrf_gpio_cfg_default(BSP::UARTAsync_Inits[0].config.tx_pin);
#if !defined(BOARD_RSPB)
	nrf_gpio_cfg_default(BSP::UARTAsync_Inits[1].config.rx_pin);
	nrf_gpio_cfg_default(BSP::UARTAsync_Inits[1].config.tx_pin);
#endif

	// Ensure power-off state for peripherals controlling power rails
	clear(GPS_POWER);
	release_to_highz(GPS_RST);  // Disconnect (ext pull-up, GPS off)

	// SMD satellite module — force OFF at init.
	// Hold RESET LOW before cutting power to prevent STM32WL boot on residual VDD.
	init_pin(SAT_RESET);
	clear(SAT_RESET);
	clear(SAT_PWR_EN);
#if (defined(ARGOS_SMD) && (ARGOS_SMD == 1)) || (defined(LORA_RAK3172) && (LORA_RAK3172 == 1))
	release_to_highz(SAT_RESET);  // Disconnect (ext pull-up, SMD off)
#else
	// KIM2: there is no external pull on nRESET-SAT (the net only reaches USR_NRST,
	// the J15 debug connector and our open-drain P0.31). Releasing the pin here
	// would leave it FLOATING on a powered-down module. It is held LOW instead --
	// reset asserted, module inert -- and it is the driver that releases it to the
	// internal pull once the module is powered.
#endif
#ifdef SMD_VPA_PIN
	// Drive VPA LOW at boot to prevent floating regulator enable
	drive_low(SMD_VPA_PIN);
#endif

	// RSPB: SMD UART pins (P0.26/P0.14) not used in SPI mode — pulldown to prevent float
#if defined(BOARD_RSPB) && !defined(SMD_UART)
	nrf_gpio_cfg_input(NRF_GPIO_PIN_MAP(0, 26), NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(NRF_GPIO_PIN_MAP(0, 14), NRF_GPIO_PIN_PULLDOWN);
#endif

	clear(SWS_ENABLE_PIN);
#ifdef GPIO_AG_PWR_PIN
	set(GPIO_AG_PWR_PIN);  // BMA400 must be ON to avoid leakage current (sleep = 200 nA)
#endif
#ifdef SENSORS_PWR_PIN
	// FIXME (RSPB): I2C pull-ups R21/R24 are on DCDC_3V3, not VSENSORS.
	// When VSENSORS is OFF, ~1.3 mA backfeeds through sensor ESD diodes.
	// Workaround: power ON VSENSORS immediately at boot and keep it ON.
	// Fix in next PCB revision: move R21/R24 to VSENSORS rail.
#if defined(BOARD_RSPB)
	set(SENSORS_PWR_PIN);
	m_sensors_pwr_refcount = 1;  // Prevent release_sensors_pwr() from cutting it
	reconnect_sensor_pins();
#else
	clear(SENSORS_PWR_PIN);
	disconnect_sensor_pins();
#endif
#endif
#ifdef ADC_ENABLE
	set(ADC_ENABLE);  // Enable ADC to avoid leakage current (I2C pull-up)
#endif

#ifdef CAM_PWR_EN
	DEBUG_TRACE("Initializing CAM pins: CAM_PWR_EN=%u (P%u.%u) | CAM_PWR_BUTT=%u (P%u.%u)", CAM_PWR_EN,
	            (BSP::GPIO_Inits[CAM_PWR_EN].pin_number >> 5), (BSP::GPIO_Inits[CAM_PWR_EN].pin_number & 0x1F),
	            CAM_PWR_BUTT, (BSP::GPIO_Inits[CAM_PWR_BUTT].pin_number >> 5),
	            (BSP::GPIO_Inits[CAM_PWR_BUTT].pin_number & 0x1F));
	clear(CAM_PWR_EN);
	clear(CAM_PWR_BUTT);
	// Force re-enable GPIO configuration to ensure pins are properly configured
	enable(CAM_PWR_EN);
	enable(CAM_PWR_BUTT);
	clear(CAM_PWR_EN);
	clear(CAM_PWR_BUTT);
	DEBUG_TRACE("CAM pins cleared and configured");
#endif

#ifdef VSYS_SEL
	set(VSYS_SEL);
#endif
}

void GPIOPins::init_pin(uint32_t pin) {
	nrf_gpio_cfg(BSP::GPIO_Inits[pin].pin_number, BSP::GPIO_Inits[pin].dir, BSP::GPIO_Inits[pin].input,
	             BSP::GPIO_Inits[pin].pull, BSP::GPIO_Inits[pin].drive, BSP::GPIO_Inits[pin].sense);
}

void GPIOPins::set(uint32_t pin) {
	nrf_gpio_pin_set(BSP::GPIO_Inits[pin].pin_number);
}

void GPIOPins::clear(uint32_t pin) {
	nrf_gpio_pin_clear(BSP::GPIO_Inits[pin].pin_number);
}

void GPIOPins::toggle(uint32_t pin) {
	nrf_gpio_pin_toggle(BSP::GPIO_Inits[pin].pin_number);
}

uint32_t GPIOPins::value(uint32_t pin) {
	return nrf_gpio_pin_read(BSP::GPIO_Inits[pin].pin_number);
}

void GPIOPins::enable(uint32_t pin) {
	nrf_gpio_cfg(BSP::GPIO_Inits[pin].pin_number, BSP::GPIO_Inits[pin].dir, BSP::GPIO_Inits[pin].input,
	             BSP::GPIO_Inits[pin].pull, BSP::GPIO_Inits[pin].drive, BSP::GPIO_Inits[pin].sense);
}

void GPIOPins::disable(uint32_t pin) {
	nrf_gpio_cfg_default(BSP::GPIO_Inits[pin].pin_number);
}

// ═══════════════════════════════════════════════════════
//  VSENSORS power management (reference-counted)
// ═══════════════════════════════════════════════════════

/**
 * @brief Disconnect sensor I2C and interrupt pins when VSENSORS is off.
 *
 * Prevents backfeed current through sensor ESD diodes and floating
 * interrupt lines triggering spurious GPIOTE events.
 */
void GPIOPins::disconnect_sensor_pins() {
#ifdef SENSORS_PWR_PIN
	// I2C pins — explicit disconnect (workaround for nRF52840 TWIM pin release errata)
#ifdef ONBOARD_I2C_BUS
	nrf_gpio_cfg_default(BSP::I2C_Inits[ONBOARD_I2C_BUS].twim_config.scl);
	nrf_gpio_cfg_default(BSP::I2C_Inits[ONBOARD_I2C_BUS].twim_config.sda);
#endif
	// Sensor interrupt pins — disconnect input to prevent floating
#ifdef BMA400_WAKEUP_PIN
	disable(BMA400_WAKEUP_PIN);
#endif
#ifdef PRESSURE_WAKEUP_PIN
	disable(PRESSURE_WAKEUP_PIN);
#endif
#endif
}

/// @brief Reconnect sensor interrupt pins after VSENSORS powers on.
/// @note I2C bus pins are reconnected by NrfI2C::init(), not here.
void GPIOPins::reconnect_sensor_pins() {
#ifdef SENSORS_PWR_PIN
#ifdef BMA400_WAKEUP_PIN
	enable(BMA400_WAKEUP_PIN);
#endif
#ifdef PRESSURE_WAKEUP_PIN
	enable(PRESSURE_WAKEUP_PIN);
#endif
#endif
}

void GPIOPins::power_cycle_sensors() {
#ifdef SENSORS_PWR_PIN
	// Time the rail is held off — long enough to drain the VSENSORS decoupling
	// caps below the sensors' POR threshold so a wedged slave is forced to reset.
	static constexpr uint32_t SENSORS_PWR_CYCLE_OFF_MS = 200;

	// Only meaningful when the rail is actually powered: a powered slave can wedge
	// the bus; an unpowered one cannot. Use the physical OUT latch, NOT the
	// refcount — init() (and the boot probe) run from acquire_sensors_pwr() before
	// the count is incremented, yet the rail is already driven high there.
	if (nrf_gpio_pin_out_read(BSP::GPIO_Inits[SENSORS_PWR_PIN].pin_number) == 0) {
		DEBUG_TRACE("GPIOPins::power_cycle_sensors: VSENSORS off | nothing to cycle");
		return;
	}

	DEBUG_TRACE("GPIOPins::power_cycle_sensors: cycling VSENSORS to release a wedged I2C slave");
	disconnect_sensor_pins();  // release nRF's I2C/INT pin config + INT lines

#ifdef ONBOARD_I2C_BUS
	// The I2C pull-ups likely sit on an always-on rail (DCDC3V3). If so, merely
	// cutting VSENSORS leaves SCL/SDA pulled to 3V3, which back-powers the VSENSORS
	// sensors through their ESD diodes — they never fully reset. Drive BOTH lines
	// LOW for the off-window to bleed that parasitic path so the sensors see a true
	// power-down. Harmless if the pull-ups are instead on VSENSORS (lines are dead
	// anyway). I2C is open-drain, so driving low is benign for the VBAT STC3117 too.
	const uint32_t scl = BSP::I2C_Inits[ONBOARD_I2C_BUS].twim_config.scl;
	const uint32_t sda = BSP::I2C_Inits[ONBOARD_I2C_BUS].twim_config.sda;
	nrf_gpio_cfg_output(scl);
	nrf_gpio_pin_clear(scl);
	nrf_gpio_cfg_output(sda);
	nrf_gpio_pin_clear(sda);
#endif

	clear(SENSORS_PWR_PIN);                  // cut the rail
	nrf_delay_ms(SENSORS_PWR_CYCLE_OFF_MS);  // drain caps → POR the VSENSORS sensors

#ifdef ONBOARD_I2C_BUS
	// Release SCL/SDA before power returns so the pull-ups bring them up cleanly.
	nrf_gpio_cfg_default(scl);
	nrf_gpio_cfg_default(sda);
#endif

	set(SENSORS_PWR_PIN);     // restore
	nrf_delay_ms(50);         // power-up stabilization (matches acquire)
	reconnect_sensor_pins();  // INT pins back; I2C pins reinit by caller
#endif
}

void GPIOPins::acquire_sensors_pwr() {
#ifdef SENSORS_PWR_PIN
	if (m_sensors_pwr_refcount == UINT8_MAX) {
		DEBUG_ERROR("GPIOPins::acquire_sensors_pwr: refcount overflow — capped at %u", m_sensors_pwr_refcount);
		return;
	}
	if (m_sensors_pwr_refcount == 0) {
		DEBUG_TRACE("GPIOPins::acquire_sensors_pwr: Powering ON VSENSORS (refcount 0->1)");
		set(SENSORS_PWR_PIN);
		nrf_delay_ms(50);  // 50 ms for sensor power-up stabilization
		reconnect_sensor_pins();
		// Reinit I2C bus (was disabled when VSENSORS went off)
		if (!NrfI2C::is_enabled(0)) {
			NrfI2C::init();
		}
	} else {
		DEBUG_TRACE("GPIOPins::acquire_sensors_pwr: refcount %u->%u (already on)", m_sensors_pwr_refcount,
		            m_sensors_pwr_refcount + 1);
		// NOTE: no per-acquire bus re-init here. A wedged bus is left DISABLED
		// (fast-fail) so it never storms recover_bus + 100 ms timeouts on every
		// read — that would block the cooperative scheduler (reed switch, main
		// loop). The bus re-inits cleanly at the next boot; a stuck VBAT STC3117
		// needs a battery power-cycle to release SCL regardless of firmware.
	}
	m_sensors_pwr_refcount++;
#endif
}

void GPIOPins::release_sensors_pwr() {
#ifdef SENSORS_PWR_PIN
	if (m_sensors_pwr_refcount == 0) {
		DEBUG_WARN("GPIOPins::release_sensors_pwr: refcount already 0 | ignoring release");
		return;
	}
#if defined(BOARD_RSPB) && RSPB_VSENSORS_FLOOR
	// RSPB: the I2C pull-ups (R21/R24) live on VSENSORS, so VSENSORS MUST stay
	// powered the entire time the nRF runs — cutting it kills the whole I2C bus
	// (SCL/SDA float low → every sensor NACKs). The boot pin sets refcount=1 as a
	// floor, but an acquire/release imbalance (GPS/SMD/DTE PWRON-OFF paths) can
	// still drive it to 0 and cut the rail. Clamp the floor HARD at 1: never let a
	// release take VSENSORS down on RSPB. (Backfeed is a non-issue now that the
	// pull-ups are on VSENSORS, not DCDC_3V3.)
	if (m_sensors_pwr_refcount <= 1) {
		DEBUG_TRACE("GPIOPins::release_sensors_pwr: RSPB floor — keeping VSENSORS ON (refcount stays 1)");
		return;
	}
#endif
	m_sensors_pwr_refcount--;
	if (m_sensors_pwr_refcount == 0) {
		DEBUG_TRACE("GPIOPins::release_sensors_pwr: Powering OFF VSENSORS (refcount 1->0)");
		// 1. Uninit I2C bus BEFORE cutting power
		NrfI2C::uninit();
		// 2. Disconnect ALL sensor-related pins to prevent backfeed current
		disconnect_sensor_pins();
		// 3. Cut VSENSORS power
		clear(SENSORS_PWR_PIN);
		nrf_delay_ms(50);  // Power supply discharge stabilization
	} else {
		DEBUG_TRACE("GPIOPins::release_sensors_pwr: refcount %u->%u (still in use)", m_sensors_pwr_refcount + 1,
		            m_sensors_pwr_refcount);
	}
#endif
}

void GPIOPins::set_gnss_uart_active(bool active) {
	m_gnss_uart_active = active;
}

bool GPIOPins::is_gnss_uart_active() {
	return m_gnss_uart_active;
}

void GPIOPins::set_flash_busy(bool busy) {
	m_flash_busy = busy;
}

bool GPIOPins::is_flash_busy() {
	return m_flash_busy;
}

void GPIOPins::set_config_mode_active(bool active) {
	m_config_mode_active = active;
}

bool GPIOPins::is_config_mode_active() {
	return m_config_mode_active;
}

bool GPIOPins::get_sensors_pwr_state() {
	return m_sensors_pwr_refcount > 0;
}

uint8_t GPIOPins::get_sensors_pwr_refcount() {
	return m_sensors_pwr_refcount;
}

// ═══════════════════════════════════════════════════════
//  Pin control helpers
// ═══════════════════════════════════════════════════════

void GPIOPins::pulse_low_then_release(uint32_t pin, uint32_t duration_ms) {
	uint32_t pin_number = BSP::GPIO_Inits[pin].pin_number;

	nrf_gpio_cfg_output(pin_number);
	nrf_gpio_pin_clear(pin_number);

	nrf_delay_ms(duration_ms);

	// Release to high-impedance — allows external probe to control pin
	nrf_gpio_cfg_default(pin_number);
}

void GPIOPins::drive_low(uint32_t pin) {
	uint32_t pin_number = BSP::GPIO_Inits[pin].pin_number;
	nrf_gpio_cfg_output(pin_number);
	nrf_gpio_pin_clear(pin_number);
}

void GPIOPins::release_to_highz(uint32_t pin) {
	nrf_gpio_cfg_default(BSP::GPIO_Inits[pin].pin_number);
}

void GPIOPins::release_to_pullup(uint32_t pin) {
	nrf_gpio_cfg_input(BSP::GPIO_Inits[pin].pin_number, NRF_GPIO_PIN_PULLUP);
}
