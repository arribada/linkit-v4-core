/**
 * @file runcam.cpp
 * @brief RunCam camera — GPIO power control (LDO + button simulation).
 */

#include "runcam.hpp"
#include "bsp.hpp"
#include "gpio.hpp"
#include "debug.hpp"
#include "pmu.hpp"

RunCam::RunCam() {
	DEBUG_TRACE("RunCam: init (LDO controlled)");
}

RunCam::~RunCam() {
	power_off();
}

/// @brief Power off: hold power button, release, cut LDO.
void RunCam::power_off() {
	if (m_state == State::POWERED_OFF) return;

	DEBUG_TRACE("RunCam: power_off");
	GPIOPins::set(CAM_PWR_BUTT);
	PMU::delay_ms(PWR_BUTT_DELAY);
	GPIOPins::clear(CAM_PWR_BUTT);
	PMU::delay_ms(PWR_DELAY);
	GPIOPins::clear(CAM_PWR_EN);
	m_state = State::POWERED_OFF;
	notify<CAMEventPowerOff>({});
}

/// @brief Power on: enable LDO, simulate button press, increment capture counter.
void RunCam::power_on() {
	if (m_state == State::POWERED_ON) return;

	DEBUG_TRACE("RunCam: power_on");
	GPIOPins::set(CAM_PWR_EN);
	PMU::delay_ms(PWR_DELAY);
	GPIOPins::set(CAM_PWR_BUTT);
	PMU::delay_ms(PWR_BUTT_DELAY);
	GPIOPins::clear(CAM_PWR_BUTT);
	m_state = State::POWERED_ON;
	m_num_captures++;
	notify<CAMEventPowerOn>({});
}

bool RunCam::is_powered_on() {
	return m_state == State::POWERED_ON;
}

unsigned int RunCam::get_num_captures() {
	return m_num_captures;
}
