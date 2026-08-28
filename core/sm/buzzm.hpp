/**
 * @file buzzm.hpp
 * @brief Buzzer state machine — maps tracker states to buzzer patterns (beep sequences).
 */

#pragma once

#include "tinyfsm.hpp"
#include "timer.hpp"
#include "gpio_buzzer.hpp"

/// @name Buzzer state events (dispatched by GenTracker FSM)
/// @{
struct SetBuzzOff : tinyfsm::Event {};
struct SetBuzzMagnetEngaged : tinyfsm::Event {};
struct SetBuzzMagnetDisengaged : tinyfsm::Event {};
struct SetBuzzPowerDown : tinyfsm::Event {};
struct SetBuzzPreOperationalPending : tinyfsm::Event {};
struct SetBuzzConfigPending : tinyfsm::Event {};
struct SetBuzzConfiguration : tinyfsm::Event {};

class BuzzOff;
class BuzzPowerDown;
class BuzzPreOperationalPending;
class BuzzConfigPending;
class BuzzConfiguration;


/// @}

/// @brief Buzzer FSM base — dispatches events to buzzer state subclasses.
class BuzzState : public tinyfsm::Fsm<BuzzState> {
protected:
	static inline bool m_is_magnet_engaged = false;

public:
	void react(SetBuzzOff const &) { transit<BuzzOff>(); }
	void react(SetBuzzMagnetEngaged const &) {
		if (!m_is_magnet_engaged) {
			m_is_magnet_engaged = true;
			enter();
		}
	}
	void react(SetBuzzMagnetDisengaged const &) {
		if (m_is_magnet_engaged) {
			m_is_magnet_engaged = false;
			enter();
		}
	}
	void react(SetBuzzPowerDown const &) { transit<BuzzPowerDown>(); }
	void react(SetBuzzPreOperationalPending const &) { transit<BuzzPreOperationalPending>(); }
	void react(SetBuzzConfigPending const &) { transit<BuzzConfigPending>(); }
	void react(SetBuzzConfiguration const &) { transit<BuzzConfiguration>(); }

	virtual void entry(void) {}
	virtual void exit(void) {}
};


class BuzzOff : public BuzzState {
public:
	void entry() override;
	void exit() override {};
};


class BuzzPowerDown : public BuzzState {
public:
	void entry() override;
	void exit() override {};
};

class BuzzPreOperationalPending : public BuzzState {
public:
	void entry() override;
	void exit() override;
};

class BuzzConfigPending : public BuzzState {
public:
	void entry() override;
	void exit() override;
};

class BuzzConfiguration : public BuzzState {
public:
	void entry() override;
	void exit() override {};
};
