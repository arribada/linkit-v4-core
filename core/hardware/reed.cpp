/**
 * @file reed.cpp
 * @brief Reed switch gesture detection — scheduler-based hold timers.
 */

#include "reed.hpp"
#include "timer.hpp"
#include "debug.hpp"
#include "scheduler.hpp"

extern Scheduler *system_scheduler;

// Reed gesture tasks run at an elevated scheduler priority (lower number =
// higher priority; 0 = watchdog, 7 = DEFAULT for everything else). Rationale:
// gesture detection and confirmation are time-sensitive and user-facing, but
// were posted at DEFAULT priority — so when the main loop is congested with
// service tasks (or has just returned from a long blocking op such as a KIM2
// AT busy-wait or a GNSS transaction), the ENGAGE/RELEASE/SHORT_HOLD/LONG_HOLD
// callbacks were processed LATE. Late feedback makes the operator overshoot the
// 3 s window into the 6 s (POWEROFF) window, and a valid re-engage can be
// out-queued by the confirmation-timeout task. Priority 2 keeps these ahead of
// routine service tasks (but below the watchdog) so feedback is prompt and a
// queued re-engage ENGAGE outranks the DEFAULT-priority confirmation timeout
// (see GenTracker::react(ReedSwitchEvent), RELEASE branch) within a run() batch.
// NOTE: this changes only the *processing order* once run() executes — the hold
// timers still FIRE at exactly 3 s/6 s (hardware-timer driven), so gesture
// semantics are unchanged.
static constexpr unsigned int REED_GESTURE_PRIORITY = 2;

ReedSwitch::ReedSwitch(Switch &sw, unsigned int short_hold_period_ms, unsigned int long_hold_period_ms) : m_switch(sw) {
	m_short_hold_period_ms = short_hold_period_ms;
	m_long_hold_period_ms = long_hold_period_ms;
}

ReedSwitch::~ReedSwitch() {
	stop();
}

void ReedSwitch::start(std::function<void(ReedSwitchGesture)> func) {
	m_user_callback = func;
	m_switch.start([this](bool state) { switch_state_handler(state); });
}

void ReedSwitch::stop() {
	m_switch.stop();
	system_scheduler->cancel_task(m_task);
}

void ReedSwitch::switch_state_handler(bool state) {
	if (state) {
		DEBUG_TRACE("ReedSwitch::switch_state_handler: ENGAGE");

		if (m_user_callback) {
			system_scheduler->post_task_prio([this]() { m_user_callback(ReedSwitchGesture::ENGAGE); },
			                                 "ReedSwitchUserCallback", REED_GESTURE_PRIORITY);
		}

		// Start hold timers
		m_task = system_scheduler->post_task_prio(
		    [this]() {
			    if (m_user_callback) {
				    system_scheduler->post_task_prio([this]() { m_user_callback(ReedSwitchGesture::SHORT_HOLD); },
					                                 "ReedSwitchUserCallback", REED_GESTURE_PRIORITY);
			    }

			    m_task = system_scheduler->post_task_prio(
			        [this]() {
				        if (m_user_callback) {
					        system_scheduler->post_task_prio(
					            [this]() { m_user_callback(ReedSwitchGesture::LONG_HOLD); }, "ReedSwitchUserCallback",
					            REED_GESTURE_PRIORITY);
				        }
			        },
			        "ShortHoldEventHandler", REED_GESTURE_PRIORITY, m_short_hold_period_ms);
		    },
		    "LongHoldEventHandler", REED_GESTURE_PRIORITY, m_long_hold_period_ms - m_short_hold_period_ms);

	} else {
		DEBUG_TRACE("ReedSwitch::switch_state_handler: RELEASE");

		system_scheduler->cancel_task(m_task);

		if (m_user_callback) {
			system_scheduler->post_task_prio([this]() { m_user_callback(ReedSwitchGesture::RELEASE); },
			                                 "ReedSwitchUserCallback", REED_GESTURE_PRIORITY);
		}
	}
}
