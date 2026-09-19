#pragma once

#include "cam.hpp"
#include "scheduler.hpp"

extern Scheduler *system_scheduler;

/// @brief Behavioural fake of a CAMDevice, honouring the asynchronous contract
/// of cam.hpp the way RunCam does: a power request flips to an intermediate
/// state, and the final state plus its event arrive PWR_DELAY + PWR_BUTT_DELAY
/// ms later through the scheduler. The GPIO side is replaced by counters.
class FakeCAMDevice : public CAMDevice {
public:
	unsigned int on_requests = 0;         ///< power_on() calls that started a sequence
	unsigned int off_requests = 0;        ///< power_off() calls that started a sequence
	unsigned int blocking_off_calls = 0;  ///< power_off_blocking() calls that found the camera not off
	bool complete_sequences = true;       ///< false: a started sequence never completes (lost task)

	static constexpr unsigned int SEQUENCE_MS = PWR_DELAY + PWR_BUTT_DELAY;

	void power_on() override {
		if (m_state != State::POWERED_OFF) return;
		on_requests++;
		m_state = State::POWERING_ON;
		if (!complete_sequences) return;
		m_task = system_scheduler->post_task_prio(
		    [this]() {
			    m_state = State::POWERED_ON;
			    m_num_captures++;
			    notify<CAMEventPowerOn>({});
		    },
		    "FakeCamPowerOn", Scheduler::DEFAULT_PRIORITY, SEQUENCE_MS);
	}

	void power_off() override {
		if (m_state != State::POWERED_ON) return;
		off_requests++;
		m_state = State::POWERING_OFF;
		if (!complete_sequences) return;
		m_task = system_scheduler->post_task_prio(
		    [this]() {
			    m_state = State::POWERED_OFF;
			    notify<CAMEventPowerOff>({});
		    },
		    "FakeCamPowerOff", Scheduler::DEFAULT_PRIORITY, SEQUENCE_MS);
	}

	void power_off_blocking() override {
		system_scheduler->cancel_task(m_task);
		if (m_state == State::POWERED_OFF) return;
		blocking_off_calls++;
		m_state = State::POWERED_OFF;
		notify<CAMEventPowerOff>({});
	}

	bool is_powered_on() override { return m_state == State::POWERED_ON; }
	unsigned int get_num_captures() override { return m_num_captures; }

	bool is_sequence_running() { return m_state == State::POWERING_ON || m_state == State::POWERING_OFF; }
	void raise_error() { notify<CAMEventError>({}); }

private:
	enum class State { POWERED_OFF, POWERING_ON, POWERED_ON, POWERING_OFF };
	State m_state = State::POWERED_OFF;
	unsigned int m_num_captures = 0;
	Scheduler::TaskHandle m_task;
};
