/**
 * @file state_machine_barrier.hpp
 * @brief Keeps a device state machine alive when one of its state handlers
 *        throws, instead of letting the exception kill the firmware.
 *
 * WHAT THIS PROTECTS. The satellite and GNSS drivers all tick the same way: a
 * `state_machine()` dispatch posted as a scheduler task, re-armed either at the
 * end of the dispatch (SmdSat) or by the handler itself through
 * `run_state_machine()` (KIM2, M10Q). An exception out of a state handler
 * therefore does two things, both bad:
 *
 *   1. it escapes the scheduler task runner and reaches std::terminate ->
 *      __verbose_terminate_handler -> abort(), which on this platform hangs in
 *      fputc until the watchdog resets the device 15 minutes later;
 *   2. it skips the re-arm, so even if something did catch it further out, the
 *      driver would never tick again -- the module goes inert with the rest of
 *      the firmware running normally around it.
 *
 * This is not a hypothetical path. The BMA400 reboot loop was exactly this:
 * callbacks threw C++ exceptions through a C library, hit std::terminate and
 * bypassed the try/catch that looked like it covered them. Service code already
 * has its barrier (Service::run_scheduled_task / handle_task_exception); the
 * drivers below it had none.
 *
 * WHAT IT DOES NOT DO. It does not force a state transition. The state-change
 * macros run `_exit()` then `_enter()` handlers, either of which is as able to
 * throw as the handler that just did, so driving a transition from inside a
 * catch block would widen the failure rather than contain it. The machine stays
 * in its state and is simply ticked again, more slowly.
 *
 * WHY IT KEEPS RETRYING FOREVER. These tags are sealed and have no downlink: a
 * driver that gives up takes the satellite link or the fix with it for the rest
 * of the deployment. A handler that throws deterministically becomes a slow,
 * quiet retry loop rather than a dead module. The log is what gets rationed,
 * not the retries -- a DEBUG_ERROR line also goes to the flash system log at
 * roughly 170 ms a line, so a loop that logged every tick would cost seconds
 * per minute and fill the log with one repeated message.
 *
 * Header-only and hardware-free, so the host tests compile it as-is (the three
 * drivers that use it are nRF-port-only). See tests/src/state_machine_barrier_test.cpp.
 */

#pragma once

#include <exception>
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

	/// @param driver  Name used in the log lines, e.g. "SmdSat".
	explicit StateMachineBarrier(const char *driver) : m_driver(driver), m_consecutive(0) {}

	/// @brief Call after a dispatch that returned normally.
	void on_tick_completed() { m_consecutive = 0; }

	/// @brief Delay to re-arm the tick with, for a given run of exceptions.
	/// @param consecutive  Number of exceptions INCLUDING the one being handled.
	static unsigned int retry_delay_ms(unsigned int consecutive) {
		return (consecutive > LOGGED_OCCURRENCES) ? QUIET_DELAY_MS : RETRY_DELAY_MS;
	}

	/// @brief Whether this occurrence still gets a line of its own.
	static bool shall_log(unsigned int consecutive) { return consecutive <= LOGGED_OCCURRENCES; }

	/// @brief Whether this occurrence is the one that announces the log going quiet.
	static bool shall_announce_quiet(unsigned int consecutive) { return consecutive == LOGGED_OCCURRENCES + 1; }

	/// @brief Report the exception being handled and say when to tick again.
	///
	/// MUST be called from inside a `catch (...)` block: it re-throws to match
	/// the live exception against the types the firmware actually throws, which
	/// is undefined behaviour with no exception in flight. Same contract as
	/// Service::handle_task_exception.
	///
	/// @param state  The state value the dispatch was in, for the log line.
	/// @return Delay in ms to re-arm the state machine tick with.
	unsigned int on_exception(int state) {
		m_consecutive++;

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
