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
// boot chime (two 200 ms beeps). Dispatched by BootState::entry ONLY on a
// user-initiated (pseudo) power-on
struct SetBuzzBoot : tinyfsm::Event {};
struct SetBuzzMagnetEngaged : tinyfsm::Event {};
struct SetBuzzMagnetDisengaged : tinyfsm::Event {};
struct SetBuzzPowerDown : tinyfsm::Event {};
// power-off confirmation gesture — two audible phases before the
// confirmed power-down trill, so the operator can tell by ear which step of the
// gesture the device is in (mirrors the LED going fast-RED at LONG_HOLD).
//   LONG_HOLD, magnet still on  -> SetBuzzPowerDownPending      (continuous tone)
//   magnet released, 3 s window -> SetBuzzPowerDownAwaitConfirm (200 ms / 600 ms)
//   magnet re-applied, confirmed-> SetBuzzPowerDown             (fast trill, 5 s)
struct SetBuzzPowerDownPending : tinyfsm::Event {};
struct SetBuzzPowerDownAwaitConfirm : tinyfsm::Event {};
struct SetBuzzPreOperationalPending : tinyfsm::Event {};
struct SetBuzzConfigPending : tinyfsm::Event {};
struct SetBuzzConfiguration : tinyfsm::Event {};

class BuzzOff;
class BuzzBoot;
class BuzzPowerDown;
class BuzzPowerDownPending;
class BuzzPowerDownAwaitConfirm;
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
	void react(SetBuzzBoot const &) { transit<BuzzBoot>(); }
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
	void react(SetBuzzPowerDownPending const &) { transit<BuzzPowerDownPending>(); }
	void react(SetBuzzPowerDownAwaitConfirm const &) { transit<BuzzPowerDownAwaitConfirm>(); }
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


/// @brief Boot chime — two short beeps meaning "boot confirmed, magnet can be released".
///
/// Stays silent once the two beeps are done, and does NOT hand the buzzer back on its
/// own: BuzzOff::entry() would re-raise the magnet-detected continuous tone while the
/// operator is still holding the magnet, which was heard as a spurious short beep
/// trailing the chime once they let go. The buzzer is claimed by the next state that
/// has something to signal (Operational/Error/BatteryCritical → SetBuzzOff, or a reed
/// gesture). The magnet flag keeps being tracked in the meantime.
class BuzzBoot : public BuzzState {
public:
	void entry() override;
	void exit() override;
};

class BuzzPowerDown : public BuzzState {
public:
	void entry() override;
	void exit() override {};
};

/// @brief Power-off armed (LONG_HOLD reached, magnet still applied).
/// Continuous tone — same "hard" signal as magnet detection, and a clear break
/// from the slow 500/500 beeping of BuzzConfigPending heard 3 s earlier.
class BuzzPowerDownPending : public BuzzState {
public:
	void entry() override;
	void exit() override;
};

/// @brief Power-off awaiting confirmation (magnet released, 3 s re-engage window).
/// Short beep / long silence (200 ms / 600 ms) — "re-apply the magnet to confirm".
class BuzzPowerDownAwaitConfirm : public BuzzState {
public:
	void entry() override;
	void exit() override;
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
