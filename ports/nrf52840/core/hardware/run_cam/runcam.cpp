/**
 * @file runcam.cpp
 * @brief RunCam camera — GPIO power control (LDO + button simulation).
 */

#include "runcam.hpp"
#include "bsp.hpp"
#include "gpio.hpp"
#include "debug.hpp"
#include "pmu.hpp"

extern Scheduler *system_scheduler;

RunCam::RunCam() {
	DEBUG_TRACE("RunCam: init (LDO controlled)");
}

RunCam::~RunCam() {
	power_off_blocking();
}

/// @brief Power on: enable LDO, then press the button for PWR_BUTT_DELAY.
///
/// Two deferred steps chained on the scheduler — nothing blocks. Each lambda
/// captures only `this`, which fits INPLACE_FUNCTION_SIZE_SCHEDULER.
void RunCam::power_on() {
	if (m_state != State::POWERED_OFF) return;  // already on, or a sequence is running

	DEBUG_TRACE("RunCam: power_on");
	m_state = State::POWERING_ON;
	GPIOPins::set(CAM_PWR_EN);
	m_task = system_scheduler->post_task_prio(
	    [this]() {
		    GPIOPins::set(CAM_PWR_BUTT);
		    m_task = system_scheduler->post_task_prio([this]() { finish_power_on(); }, "RunCamPowerOnRelease",
			                                          Scheduler::DEFAULT_PRIORITY, PWR_BUTT_DELAY);
	    },
	    "RunCamPowerOnPress", Scheduler::DEFAULT_PRIORITY, PWR_DELAY);
}

void RunCam::finish_power_on() {
	GPIOPins::clear(CAM_PWR_BUTT);
	m_state = State::POWERED_ON;
	m_num_captures++;
	notify_power_on();
}

/// @brief Power off: press the button for PWR_BUTT_DELAY, release, cut LDO.
void RunCam::power_off() {
	if (m_state != State::POWERED_ON) return;  // already off, or a sequence is running

	DEBUG_TRACE("RunCam: power_off");
	m_state = State::POWERING_OFF;
	GPIOPins::set(CAM_PWR_BUTT);
	m_task = system_scheduler->post_task_prio(
	    [this]() {
		    GPIOPins::clear(CAM_PWR_BUTT);
		    m_task = system_scheduler->post_task_prio([this]() { finish_power_off(); }, "RunCamPowerOffRail",
			                                          Scheduler::DEFAULT_PRIORITY, PWR_DELAY);
	    },
	    "RunCamPowerOffRelease", Scheduler::DEFAULT_PRIORITY, PWR_BUTT_DELAY);
}

void RunCam::finish_power_off() {
	GPIOPins::clear(CAM_PWR_EN);
	m_state = State::POWERED_OFF;
	notify_power_off();
}

/// @brief Blocking power-off for the shutdown path (see CAMDevice).
///
/// Busy-waits with PMU::delay_ms: the caller is about to stop the scheduler
/// or enter System OFF, so an asynchronous sequence could never complete, and
/// the camera would lose its rail mid-write.
void RunCam::power_off_blocking() {
	if (system_scheduler) system_scheduler->cancel_task(m_task);
	if (m_state == State::POWERED_OFF) return;

	DEBUG_TRACE("RunCam: power_off (blocking)");
	// Whatever step the asynchronous sequence had reached, start from a
	// released button so the press below is a clean edge the module sees.
	GPIOPins::clear(CAM_PWR_BUTT);
	PMU::delay_ms(PWR_DELAY);
	GPIOPins::set(CAM_PWR_BUTT);
	PMU::delay_ms(PWR_BUTT_DELAY);
	GPIOPins::clear(CAM_PWR_BUTT);
	PMU::delay_ms(PWR_DELAY);
	GPIOPins::clear(CAM_PWR_EN);
	m_state = State::POWERED_OFF;
	notify_power_off();
}

bool RunCam::is_powered_on() {
	return m_state == State::POWERED_ON;
}

unsigned int RunCam::get_num_captures() {
	return m_num_captures;
}

/// @brief Deliver the completion event from inside a scheduler task.
///
/// The listener (CAMService) logs and reschedules itself in react(), and that
/// path reads the config store, which can throw. We are running from our own
/// scheduler task, outside Service::run_scheduled_task()'s barrier, so an
/// escaping exception would reach std::terminate. The device state is already
/// final at this point; the listener owns its own recovery.
void RunCam::notify_power_on() {
	try {
		notify<CAMEventPowerOn>({});
	} catch (...) {
		DEBUG_ERROR("RunCam: exception in power-on listener — ignored");
	}
}

void RunCam::notify_power_off() {
	try {
		notify<CAMEventPowerOff>({});
	} catch (...) {
		DEBUG_ERROR("RunCam: exception in power-off listener — ignored");
	}
}
