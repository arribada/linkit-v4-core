/**
 * @file runcam.hpp
 * @brief RunCam camera driver — power control via GPIO.
 *
 * The camera is powered through an LDO (CAM_PWR_EN) and started / stopped by
 * simulating a press on its power button (CAM_PWR_BUTT). A press is a
 * PWR_BUTT_DELAY-long hold, and the module needs that whole hold on power-off
 * to close its SD card file. The sequences are therefore long (2.1 s), and are
 * run as a chain of scheduler tasks so nothing else waits for them; see
 * CAMDevice for the contract.
 */

#pragma once

#include <cstdint>
#include "cam.hpp"
#include "scheduler.hpp"

class RunCam : public CAMDevice {
public:
	RunCam();
	~RunCam();
	void power_off() override;
	void power_on() override;
	void power_off_blocking() override;
	bool is_powered_on() override;
	unsigned int get_num_captures() override;

private:
	enum class State { POWERED_OFF, POWERING_ON, POWERED_ON, POWERING_OFF };
	State m_state = State::POWERED_OFF;
	unsigned int m_num_captures = 0;  ///< Power-on count since boot
	Scheduler::TaskHandle m_task;     ///< Pending step of the running sequence, if any

	void finish_power_on();
	void finish_power_off();
	void notify_power_on();
	void notify_power_off();
};
