/**
 * @file buzzm.cpp
 * @brief Buzzer state machine — entry handlers for each tracker state → beep patterns.
 */

#include "buzzm.hpp"
#include "debug.hpp"

extern Buzzer *buzzer_ctl;


void BuzzOff::entry() {
	if (buzzer_ctl) {
		DEBUG_TRACE("BuzzOff: entry");
		if (m_is_magnet_engaged)
			buzzer_ctl->on();
		else
			buzzer_ctl->off();
	}
}

void BuzzPowerDown::entry() {
	if (buzzer_ctl) {
		DEBUG_TRACE("BuzzPowerDown: entry");
		if (m_is_magnet_engaged)
			buzzer_ctl->on();
		else
			buzzer_ctl->beep_count(100, 100, 25);  // 5 Secs
	}
}

void BuzzPreOperationalPending::entry() {
	if (buzzer_ctl) {
		DEBUG_TRACE("BuzzPreOperationalPending: entry");
		if (m_is_magnet_engaged)
			buzzer_ctl->beep_infinite(500, 500);
		else
			buzzer_ctl->beep_count(200, 200, 2);
	}
}

void BuzzPreOperationalPending::exit() {
	if (buzzer_ctl) {
		DEBUG_TRACE("BuzzPreOperationalPending: exit");
		buzzer_ctl->off();
	}
}

void BuzzConfigPending::entry() {
	if (buzzer_ctl) {
		DEBUG_TRACE("BuzzConfigPending: entry");
		buzzer_ctl->beep_infinite(500, 500);
	}
}

void BuzzConfigPending::exit() {
	if (buzzer_ctl) {
		DEBUG_TRACE("BuzzConfigPending: exit");
		buzzer_ctl->off();
	}
}

void BuzzConfiguration::entry() {
	if (buzzer_ctl) {
		DEBUG_TRACE("BuzzConfiguration: entry");
		if (m_is_magnet_engaged)
			buzzer_ctl->on();
		else
			buzzer_ctl->beep_count(200, 200, 4);
	}
}
