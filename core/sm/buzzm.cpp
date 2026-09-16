/**
 * @file buzzm.cpp
 * @brief Buzzer state machine — entry handlers for each tracker state → beep patterns.
 */

#include "buzzm.hpp"
#include "debug.hpp"

extern Buzzer *buzzer_ctl;
extern Timer *system_timer;

namespace {

// === Stuck-magnet buzzer safety (2026-09) ==================================
// BuzzPowerDownPending holds a CONTINUOUS tone for as long as the power-off
// confirmation is armed, and that arming ends only on a reed RELEASE. A reed
// stuck CLOSED (manufacturing magnet residue, shorted switch) therefore means a
// permanent drone: PreOperationalState escapes after PREOP_STUCK_REED_MAX_MS
// (20 s) and silences the buzzer on the way to Operational, but
// ConfigurationState has no such escape — the tag would sound off until the
// battery dies.
//
// Same philosophy as ledsm.cpp's LED freeze safety: PURELY audible. We only
// switch the buzzer off; the FSM stays in BuzzPowerDownPending and the LED keeps
// its fast RED blink, so the gesture still resolves normally if the magnet is
// eventually released (RELEASE -> BuzzPowerDownAwaitConfirm re-arms the sound).
constexpr unsigned int BUZZ_STUCK_MAGNET_TIMEOUT_MS = 30000;
Timer::TimerHandle s_buzz_stuck_handle;

// === Boot chime ============================================================
// Two 200 ms beeps
// Timeline of beep_count(200, 200, 2): 200 off | 200 ON | 200 off | 200 ON | 200 off,
// then the driver stops itself with off() at t = 1000 ms. Nothing is scheduled on top
// of it: BuzzBoot does NOT hand the buzzer back when the chime ends — see the comment
// in entry() for the trailing-beep this removes.
constexpr unsigned int BOOT_CHIME_ON_MS = 200;
constexpr unsigned int BOOT_CHIME_OFF_MS = 200;
constexpr unsigned int BOOT_CHIME_COUNT = 2;

/// One-shot latch. BuzzState::react(SetBuzzMagnet*) replays entry() on the current
/// state, and BootState::entry primes the engage gesture right after arming the chime,
/// so entry() is re-run on virtually every power-on (and again on the release). Without
/// this the chime would restart on each magnet edge instead of playing once.
bool s_boot_chime_played = false;

void disarm_buzz_stuck_safety() {
	if (!system_timer) return;
	if (s_buzz_stuck_handle.has_value()) {
		system_timer->cancel_schedule(s_buzz_stuck_handle);
		s_buzz_stuck_handle.reset();
	}
}

void arm_buzz_stuck_safety() {
	if (!system_timer || !buzzer_ctl) return;
	// Re-arming (e.g. entry() replayed by a magnet event) must not leak the
	// previous schedule.
	disarm_buzz_stuck_safety();
	s_buzz_stuck_handle = system_timer->add_schedule(
	    []() {
		    DEBUG_WARN("Buzzer stuck-magnet safety: %u s of continuous tone — forcing buzzer off",
		               BUZZ_STUCK_MAGNET_TIMEOUT_MS / 1000);
		    if (buzzer_ctl) buzzer_ctl->off();
		    s_buzz_stuck_handle.reset();
	    },
	    system_timer->get_counter() + BUZZ_STUCK_MAGNET_TIMEOUT_MS);
}

}  // namespace


void BuzzOff::entry() {
	if (buzzer_ctl) {
		DEBUG_TRACE("BuzzOff: entry");
		if (m_is_magnet_engaged)
			buzzer_ctl->on();
		else
			buzzer_ctl->off();
	}
}

/// @brief Two short beeps = "boot confirmed".
///
/// Re-entrant by design: BuzzState::react(SetBuzzMagnet*) replays entry() on the
/// current state, and BootState::entry primes the engage gesture right after arming
/// this chime — so a magnet event lands mid-chime on virtually every power-on. The
/// in-flight guard keeps the chime from restarting instead of playing once.
void BuzzBoot::entry() {
	if (!buzzer_ctl) return;
	if (s_boot_chime_played) {
		// Magnet edge replaying entry() — stay silent, do not replay the chime.
		DEBUG_TRACE("BuzzBoot: entry (chime already played)");
		return;
	}
	DEBUG_TRACE("BuzzBoot: entry");
	s_boot_chime_played = true;
	// beep_sequence_manager() cancels any pending toggle and clears the pin
	// synchronously, so this also cuts the power-on hold tone if it is still ringing.
	buzzer_ctl->beep_count(BOOT_CHIME_ON_MS, BOOT_CHIME_OFF_MS, BOOT_CHIME_COUNT);
	// Deliberately NO transit back to BuzzOff when the chime ends. BuzzOff::entry()
	// turns the buzzer ON whenever the magnet flag is set, and at that point in the
	// boot the operator IS still holding the magnet — they only let go once they hear
	// this chime. Handing the buzzer back therefore restarted the continuous
	// magnet-detected tone right after the two beeps, and their release cut it a few
	// hundred ms later: audibly a spurious short beep trailing the chime. Staying in
	// BuzzBoot keeps the buzzer silent until something that actually has a signal to
	// give claims it — OperationalState / ErrorState / BatteryCritical
	// (SetBuzzOff) or a reed gesture (SetBuzzConfigPending, SetBuzzPowerDownPending).
	// The magnet flag keeps being tracked meanwhile, so whichever state takes over
	// sees the correct magnet state.
}

void BuzzBoot::exit() {
	if (buzzer_ctl) {
		DEBUG_TRACE("BuzzBoot: exit");
		buzzer_ctl->off();
	}
}

void BuzzPowerDown::entry() {
	if (buzzer_ctl) {
		DEBUG_TRACE("BuzzPowerDown: entry");
		// This state is only ever entered on a CONFIRMED power-off, i.e. right after the
		// operator re-applied the magnet
		buzzer_ctl->beep_count(100, 100, 25);  // 5 Secs
	}
}

/// Phase 1 of the power-off gesture: LONG_HOLD reached, magnet still applied.
/// Continuous tone (magnet-independent) — breaks away from the slow 500/500
/// beeping of BuzzConfigPending heard at SHORT_HOLD 3 s earlier.
void BuzzPowerDownPending::entry() {
	if (buzzer_ctl) {
		DEBUG_TRACE("BuzzPowerDownPending: entry");
		buzzer_ctl->on();
		// Unbounded tone — only a reed RELEASE ends this state. Arm the
		// stuck-magnet cut-off (audible only, the FSM is untouched).
		arm_buzz_stuck_safety();
	}
}

void BuzzPowerDownPending::exit() {
	disarm_buzz_stuck_safety();
	if (buzzer_ctl) {
		DEBUG_TRACE("BuzzPowerDownPending: exit");
		buzzer_ctl->off();
	}
}

/// Phase 2: magnet released, 3 s re-engage window open (CONFIRMATION_TIMEOUT_MS).
/// Short beep / long silence — prompts the operator to re-apply the magnet.
void BuzzPowerDownAwaitConfirm::entry() {
	if (buzzer_ctl) {
		DEBUG_TRACE("BuzzPowerDownAwaitConfirm: entry");
		buzzer_ctl->beep_infinite(200, 600);
	}
}

void BuzzPowerDownAwaitConfirm::exit() {
	if (buzzer_ctl) {
		DEBUG_TRACE("BuzzPowerDownAwaitConfirm: exit");
		buzzer_ctl->off();
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
