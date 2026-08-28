/**
 * @file runcam.hpp
 * @brief runcam driver.
 */

#pragma once

#include <cstdint>
#include "cam.hpp"
#include "scheduler.hpp"

/**
 * @file runcam.hpp
 * @brief RunCam camera driver — power control via GPIO.
 */

static constexpr unsigned int PWR_BUTT_DELAY = 2000;  ///< Power button hold time (ms)
static constexpr unsigned int PWR_DELAY = 100;        ///< Power rail stabilization delay (ms)

class RunCam : public CAMDevice {
public:
	RunCam();
	~RunCam();
	void power_off() override;
	void power_on() override;
	bool is_powered_on() override;
	unsigned int get_num_captures() override;

private:
	enum class State { POWERED_OFF, POWERED_ON };
	State m_state = State::POWERED_OFF;
	unsigned int m_num_captures = 0;  ///< Capture counter (reset on power cycle)
};