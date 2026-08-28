/**
 * @file switch.hpp
 * @brief Abstract switch/button interface with hysteresis and pause/resume.
 */

#pragma once

#include <functional>

/// @brief Abstract switch — state change callback with configurable hysteresis.
class Switch {
protected:
	std::function<void(bool)> m_state_change_handler;
	int m_pin;
	unsigned int m_hysteresis_time_ms;
	int m_current_state = 0;
	bool m_active_state = true;
	double m_activation_threshold = 0.0;
	bool m_is_paused = true;

public:
	Switch(int pin, unsigned int hysteresis_time_ms, bool active_state = true) {
		m_pin = pin;
		m_hysteresis_time_ms = hysteresis_time_ms;
		m_current_state = false;
		m_active_state = active_state;
		m_is_paused = true;
	}
	virtual ~Switch() {}
	bool get_state() { return m_current_state; }
	virtual void start(std::function<void(bool)> func) { m_state_change_handler = func; }
	virtual void stop() { m_state_change_handler = nullptr; }
	virtual void pause() { m_is_paused = true; }
	virtual void resume() { m_is_paused = false; }
	virtual bool is_paused() { return m_is_paused; }
};
