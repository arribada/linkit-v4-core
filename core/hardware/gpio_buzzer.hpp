/**
 * @file gpio_buzzer.hpp
 * @brief GPIO-backed buzzer — on/off/beep sequences via hardware timer.
 */

#pragma once

#include "gpio.hpp"
#include "timer.hpp"
#include "interrupt_lock.hpp"

extern Timer *system_timer;

#define BUZZER_ON(instance)           \
	{                                 \
		if (instance) instance->on(); \
	}
#define BUZZER_OFF(instance)           \
	{                                  \
		if (instance) instance->off(); \
	}
#define BUZZER_BEEP_INFINITE(instance, t_on, t_off)         \
	{                                                       \
		if (instance) instance->beep_infinite(t_on, t_off); \
	}
#define BUZZER_BEEP_COUNT(instance, t_on, t_off, count)         \
	{                                                           \
		if (instance) instance->beep_count(t_on, t_off, count); \
	}

/// @brief Buzzer driven by a GPIO pin with timer-based beep sequences.
class Buzzer : public GPIOPins {
public:
	Buzzer(int pin) {
		m_pin = pin;
		off();
	}
	~Buzzer() { off(); };

	void off() {
		m_is_beeping = false;
		system_timer->cancel_schedule(m_timer_task);
		GPIOPins::clear(m_pin);
	}

	void on() {
		m_is_beeping = false;
		system_timer->cancel_schedule(m_timer_task);
		GPIOPins::set(m_pin);
	}

	void beep_infinite(unsigned int time_on_ms, unsigned int time_off_ms) {
		beep_sequence_manager(time_on_ms, time_off_ms, true, 0);
	}

	void beep_count(unsigned int time_on_ms, unsigned int time_off_ms, unsigned int nb_repetitions) {
		beep_sequence_manager(time_on_ms, time_off_ms, false, nb_repetitions);
	}

private:
	int m_pin;
	bool m_is_beeping = false;
	bool m_beep_state = false;
	unsigned int m_beep_on_time = 0;
	unsigned int m_beep_off_time = 0;
	bool m_beep_sequence_infinite = false;
	unsigned int m_beep_sequence_max = 0;
	unsigned int m_beep_count = 0;
	Timer::TimerHandle m_timer_task;

	void beep_sequence_manager(unsigned int time_on_ms, unsigned int time_off_ms, bool is_infinite,
	                           unsigned int nb_repetitions) {
		InterruptLock lock;
		system_timer->cancel_schedule(m_timer_task);

		m_beep_on_time = time_on_ms;
		m_beep_off_time = time_off_ms;
		m_beep_sequence_infinite = is_infinite;
		m_beep_sequence_max = nb_repetitions;
		m_beep_count = 0;

		m_is_beeping = true;
		m_beep_state = false;  // Start with OFF state to identify changing from "ON continue" to "sequence"
		toggle_buzzer();
	}

	void toggle_buzzer(void) {
		InterruptLock lock;
		unsigned int next_timeout_ms = 0;
		if (!m_is_beeping) return;
		if (m_beep_state) {
			m_beep_count++;
			if (!m_beep_sequence_infinite && m_beep_count > m_beep_sequence_max) {
				off();
				return;
			}
			next_timeout_ms = m_beep_on_time;
			GPIOPins::set(m_pin);
		} else {
			next_timeout_ms = m_beep_off_time;
			GPIOPins::clear(m_pin);
		}
		m_beep_state = !m_beep_state;
		m_timer_task = system_timer->add_schedule(
		    [this]() {
			    if (m_is_beeping) toggle_buzzer();
		    },
		    system_timer->get_counter() + next_timeout_ms);
	}
};
