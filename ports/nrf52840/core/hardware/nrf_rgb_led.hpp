#pragma once

/**
 * @file nrf_rgb_led.hpp
 * @brief nRF52840 RGB LED driver (active-low, timer-based flash).
 *
 * LEDs are active-low: GPIOPins::clear() turns ON, GPIOPins::set() turns OFF.
 * Flash is driven by the hardware Timer, not the Scheduler — safe from ISR context.
 *
 * Two APIs:
 *  - Instance methods (set/flash/off): use Timer for flashing, require system_timer.
 *  - set_color_raw() (static): bare-metal GPIO, no timer dependency — safe for
 *    fault handlers and ISR context.
 */

#include <cstdint>

#include "timer.hpp"
#include "rgb_led.hpp"
#include "gpio.hpp"
#include "interrupt_lock.hpp"

extern Timer *system_timer;

class NrfRGBLed : public RGBLed {
private:
	int m_pin_red;
	int m_pin_green;
	int m_pin_blue;
	RGBLedColor m_color;
	RGBLedColor m_flash_color;
	RGBLedColor m_flash_color_alt;
	bool m_is_flashing;
	bool m_flash_state;
	unsigned int m_flash_interval;
	const char *m_name;
	Timer::TimerHandle m_led_timer;

	void toggle_led() {
		InterruptLock lock;
		if (!m_is_flashing) return;
		set_color_raw(m_pin_red, m_pin_green, m_pin_blue, m_flash_state ? m_flash_color : m_flash_color_alt);
		m_flash_state = !m_flash_state;
		m_led_timer = system_timer->add_schedule(
		    [this]() {
			    if (m_is_flashing) toggle_led();
		    },
		    system_timer->get_counter() + m_flash_interval);
	}

public:
	/**
	 * @brief Bare-metal LED color set — no timer/interrupt dependency.
	 *
	 * Safe for fault handlers and ISR context.  Active-low: clear = ON, set = OFF.
	 *
	 * @param pin_red    BSP GPIO index for red LED.
	 * @param pin_green  BSP GPIO index for green LED.
	 * @param pin_blue   BSP GPIO index for blue LED.
	 * @param color      Color to display.
	 */
	static void set_color_raw(int pin_red, int pin_green, int pin_blue, RGBLedColor color) {
		bool r = false, g = false, b = false;
		switch (color) {
		case RGBLedColor::BLACK: break;
		case RGBLedColor::RED: r = true; break;
		case RGBLedColor::GREEN: g = true; break;
		case RGBLedColor::BLUE: b = true; break;
		case RGBLedColor::CYAN:
			g = true;
			b = true;
			break;
		case RGBLedColor::MAGENTA:
			r = true;
			b = true;
			break;
		case RGBLedColor::YELLOW:
			r = true;
			g = true;
			break;
		case RGBLedColor::WHITE:
			r = true;
			g = true;
			b = true;
			break;
		default: break;
		}
		// Active-low: clear = LED ON, set = LED OFF
		r ? GPIOPins::clear(pin_red) : GPIOPins::set(pin_red);
		g ? GPIOPins::clear(pin_green) : GPIOPins::set(pin_green);
		b ? GPIOPins::clear(pin_blue) : GPIOPins::set(pin_blue);
	}

	NrfRGBLed(const char *name, int red, int green, int blue, RGBLedColor color = RGBLedColor::BLACK)
	    : m_pin_red(red),
	      m_pin_green(green),
	      m_pin_blue(blue),
	      m_color(color),
	      m_flash_color(RGBLedColor::BLACK),
	      m_flash_color_alt(RGBLedColor::BLACK),
	      m_is_flashing(false),
	      m_flash_state(false),
	      m_flash_interval(500),
	      m_name(name),
	      m_led_timer{} {
		set(color);
	}

	void set(RGBLedColor color) override {
		InterruptLock lock;
		system_timer->cancel_schedule(m_led_timer);
		m_is_flashing = false;
		m_color = color;
		set_color_raw(m_pin_red, m_pin_green, m_pin_blue, color);
	}

	void off() override { set(RGBLedColor::BLACK); }

	void flash(RGBLedColor color, unsigned int interval_ms = 500) override {
		InterruptLock lock;
		system_timer->cancel_schedule(m_led_timer);
		m_flash_interval = interval_ms;
		m_flash_color = color;
		m_flash_color_alt = RGBLedColor::BLACK;
		m_is_flashing = true;
		m_flash_state = true;
		toggle_led();
	}

	void flash_alternate(RGBLedColor color1, RGBLedColor color2, unsigned int interval_ms = 250) override {
		InterruptLock lock;
		system_timer->cancel_schedule(m_led_timer);
		m_flash_interval = interval_ms;
		m_flash_color = color1;
		m_flash_color_alt = color2;
		m_is_flashing = true;
		m_flash_state = true;
		toggle_led();
	}

	bool is_flashing() override { return m_is_flashing; }

	RGBLedColor get_state() override { return m_color; }
};
