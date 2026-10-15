/**
 * @file state_machine_barrier.hpp
 * @brief Absorbs a transient throw out of a device state handler, and escalates
 *        a persistent one to the firmware's own recovery ladder.
 *
 * WHAT THIS PROTECTS. The radio and GNSS drivers all tick the same way: a
 * `state_machine()` dispatch posted as a scheduler task, re-armed either at the
 * end of the dispatch (SmdSat) or by the handler itself through
 * `run_state_machine()` (KIM2, M10Q, LoRa). An exception out of a state handler
 * skips that re-arm, so the driver never ticks again: the module goes inert
 * with the rest of the firmware running normally around it. That is what this
 * fixes.
 *
 * WHAT IT DOES *NOT* FIX, CONTRARY TO WHAT THIS COMMENT FIRST CLAIMED. The
 * exception did not reach std::terminate. ports/nrf52840/main.cpp:1621 wraps the
 * whole loop, `system_scheduler->run()` at :1647 included, in
 * catch(ErrorCode) / catch(const std::exception &) / catch(...), and that catch
 * has been there all along -- its own comment describes the std::terminate ->
 * abort() -> fputc -> 15-minute-watchdog path as the thing it exists to
 * prevent. An escaping driver exception therefore became an ErrorEvent, and
 * ErrorState handled it. Swallowing it here REMOVED that escalation, which is
 * why this barrier now hands a persistent fault back.
 *
 * TRANSIENT VERSUS PERSISTENT. Up to ESCALATE_AFTER consecutive exceptions are
 * absorbed: the state is kept, the tick is re-armed, and a clean dispatch
 * resets the count. That is the case worth absorbing -- a single throw used to
 * cost the whole session for nothing. Past that the driver is not having bad
 * luck, it is broken, and on_exception() returns nullopt so the caller
 * rethrows. The fault then reaches main.cpp's catch, becomes an ErrorEvent, and
 * walks the ladder that core/sm/gentracker.cpp:155-210 already defines.
 *
 * WHY ESCALATING IS SAFE ON A SEALED TAG. Checked in that policy block before
 * writing this, because a tag that disables itself is lost. On LinkIt, "boot
 * failures therefore never lead there [OffState]": ErrorState waits
 * BOOT_BACKOFF_S (5 s, 30 s, 2 min, 10 min, then 60 min repeating), resets, and
 * retries for ever. On RSPB, OffState is a TPL5111 power cut that its own timer
 * undoes. Neither board can be walked into a permanent stop by this path. The
 * single factory_reset on the way needs five failures AND 30 minutes of
 * accumulated backoff, and it keeps PROTECTED_PARAMS -- identifiers,
 * calibration and the provisioning secrets.
 *
 * And a reset is the stronger remedy anyway: it re-initialises a wedged
 * peripheral, which ticking the same state machine again cannot, and the ladder
 * spaces the attempts out instead of making 120 of them an hour.
 *
 * WHAT IT STILL DOES NOT DO. It does not force a state transition. The
 * state-change macros run `_exit()` then `_enter()` handlers, either of which is
 * as able to throw as the handler that just did, so driving a transition from
 * inside a catch block would widen the failure rather than contain it.
 *
 * The log is rationed, not the retries: a DEBUG_ERROR line also goes to the
 * flash system log at roughly 170 ms a line.
 *
 * Header-only and hardware-free, so the host tests compile it as-is (the four
 * drivers that use it are nRF-port-only). See tests/src/state_machine_barrier_test.cpp.
 */

#pragma once

#include <exception>
#include <optional>
#include <stdexcept>
#include <variant>

#include "debug.hpp"
#include "error.hpp"

class StateMachineBarrier {
public:
	/// Occurrences that get a DEBUG_ERROR line each. The one after them says
	/// the log is going quiet, then nothing more is logged.
	static constexpr unsigned int LOGGED_OCCURRENCES = 3;
	/// Re-arm delay while still logging: long enough not to spin, short enough
	/// that a transient throw costs the session almost nothing.
	static constexpr unsigned int RETRY_DELAY_MS = 1000;
	/// Re-arm delay once the throw looks deterministic.
	static constexpr unsigned int QUIET_DELAY_MS = 30000;
	/// Consecutive exceptions after which the fault stops being absorbed and is
	/// handed to the firmware's recovery ladder. Reaching it takes about 33 s
	/// (three fast retries, then one slow one), so a burst of transients does
	/// not cost a reset while a broken driver does not sit there for hours.
	static constexpr unsigned int ESCALATE_AFTER = 5;

	/// @param driver  Name used in the log lines, e.g. "SmdSat".
	explicit StateMachineBarrier(const char *driver) : m_driver(driver), m_consecutive(0) {}

	/// @brief Call after a dispatch that returned normally.
	void on_tick_completed() { m_consecutive = 0; }

	/// @brief Delay to re-arm the tick with, for a given run of exceptions.
	/// @param consecutive  Number of exceptions INCLUDING the one being handled.
	static unsigned int retry_delay_ms(unsigned int consecutive) {
		return (consecutive > LOGGED_OCCURRENCES) ? QUIET_DELAY_MS : RETRY_DELAY_MS;
	}

	/// @brief Whether this run has gone on long enough to stop absorbing it.
	static bool shall_escalate(unsigned int consecutive) { return consecutive >= ESCALATE_AFTER; }

	/// @brief Whether this occurrence still gets a line of its own.
	static bool shall_log(unsigned int consecutive) { return consecutive <= LOGGED_OCCURRENCES; }

	/// @brief Whether this occurrence is the one that announces the log going quiet.
	static bool shall_announce_quiet(unsigned int consecutive) { return consecutive == LOGGED_OCCURRENCES + 1; }

	/// @brief Report the exception being handled and say what to do with it.
	///
	/// MUST be called from inside a `catch (...)` block: it re-throws to match
	/// the live exception against the types the firmware actually throws, which
	/// is undefined behaviour with no exception in flight. Same contract as
	/// Service::handle_task_exception.
	///
	/// @param state  The state value the dispatch was in, for the log line.
	/// @return Delay in ms to re-arm the tick with, or nullopt when the caller
	///         must `throw;` so the fault reaches main.cpp's catch and the
	///         ErrorState ladder. A caller that ignores the nullopt turns a
	///         broken driver back into a silent infinite retry.
	std::optional<unsigned int> on_exception(int state) {
		m_consecutive++;

		if (shall_escalate(m_consecutive)) {
			// Deliberately an ERROR even though the quiet rule would suppress it:
			// this is the last line the driver writes before the reset, and it is
			// the only thing that will explain the reset afterwards.
			DEBUG_ERROR("%s::state_machine: %s in state %d, %u in a row — escalating, no longer absorbed", m_driver,
			            describe_current_exception(), state, m_consecutive);
			return std::nullopt;
		}

		if (shall_log(m_consecutive)) {
			DEBUG_ERROR("%s::state_machine: %s in state %d (%u) — state kept, retry in %u ms", m_driver,
			            describe_current_exception(), state, m_consecutive, retry_delay_ms(m_consecutive));
		} else if (shall_announce_quiet(m_consecutive)) {
			// One line, then silence: see the flash-log cost in the file header.
			DEBUG_WARN("%s::state_machine: still throwing in state %d — no longer logging, retrying every %u ms",
			           m_driver, state, QUIET_DELAY_MS);
		}

		return retry_delay_ms(m_consecutive);
	}

	/// @brief Length of the current run of exceptions (0 after a clean tick).
	unsigned int consecutive() const { return m_consecutive; }

	/// @brief Short name for the exception in flight. Same `catch (...)` contract
	///        as on_exception(); returns a literal, never allocates.
	static const char *describe_current_exception() {
		try {
			throw;  // re-match the live exception
		} catch (ErrorCode) {
			// Firmware-thrown enum. Not formatted with its value here: these are
			// thrown from deep inside the drivers and the state is the useful part.
			return "ErrorCode";
		} catch (const std::bad_variant_access &) {
			return "bad_variant_access";
		} catch (const std::out_of_range &) {
			return "out_of_range";
		} catch (const std::exception &) {
			return "std::exception";
		} catch (...) {
			return "unknown exception";
		}
	}

private:
	const char *m_driver;
	unsigned int m_consecutive;
};
