/**
 * @file reed.hpp
 * @brief Reed switch gesture detector — engage, short hold, long hold, release.
 */

#pragma once

#include <functional>
#include "switch.hpp"
#include "timer.hpp"
#include "scheduler.hpp"

enum class ReedSwitchGesture { ENGAGE, SHORT_HOLD, LONG_HOLD, RELEASE };


/// @brief Reed switch with timed gesture detection (short hold 3s, long hold 6s).
class ReedSwitch {
private:
	Switch &m_switch;
	uint64_t m_last_trigger_time = 0;
	unsigned int m_short_hold_period_ms;
	unsigned int m_long_hold_period_ms;
	Scheduler::TaskHandle m_task = {};

	void switch_state_handler(bool);

protected:
	std::function<void(ReedSwitchGesture)> m_user_callback;

public:
	ReedSwitch(Switch &sw, unsigned int short_hold_period_ms = 3000, unsigned int long_hold_period_ms = 6000);
	~ReedSwitch();
	void start(std::function<void(ReedSwitchGesture)> func);
	void stop();
	bool is_engaged() { return m_switch.get_state(); }

	/// @brief Prime the gesture engine when the magnet is ALREADY engaged at the
	/// moment of start() — typically present at boot.
	///
	/// NrfSwitch::resume() syncs the debounced state to the live pin but
	/// deliberately suppresses the initial state-change callback (only state
	/// CHANGES fire one). A magnet held continuously across boot therefore never
	/// runs switch_state_handler(), so ENGAGE is not delivered through the gesture
	/// path AND the SHORT_HOLD/LONG_HOLD timers are never armed — making the
	/// config / power-off gestures impossible until the operator removes and
	/// re-applies the magnet. Call this right after start() so an at-boot magnet
	/// behaves identically to a runtime engage: ENGAGE is dispatched and the hold
	/// timers are armed. No-op if the switch is not currently engaged.
	void prime_if_engaged() {
		if (m_switch.get_state()) switch_state_handler(true);
	}
};
