/**
 * @file state_machine_barrier.hpp
 * @brief Names the driver, the state and the exception type when a device state
 *        handler throws, then lets the exception continue on its way.
 *
 * It changes no control flow. That is the point, and it is the third version of
 * this file: the first two did change it, and both were wrong. The history is
 * kept because the reasoning is what matters, not the result.
 *
 * WHAT THE FIRST VERSION CLAIMED, AND WHY IT WAS WRONG. It said an exception out
 * of a state handler reached std::terminate -> abort() -> a hang in fputc until
 * the watchdog fired, and that it also skipped the re-arm so the module went
 * inert. Neither holds. ports/nrf52840/main.cpp:1621 wraps the whole loop,
 * `system_scheduler->run()` at :1647 included, in catch(ErrorCode) /
 * catch(const std::exception &) / catch(...), and has done all along -- its own
 * comment describes the terminate path as the thing it exists to prevent.
 * Scheduler::run() (scheduler.hpp:168) calls task.m_func() bare, with its
 * InterruptLock already out of scope, so the exception propagates out cleanly,
 * becomes an ErrorEvent, and ErrorState resets the device. The module does not
 * go inert: it comes back from the reset with everything re-initialised.
 *
 * WHY THE SECOND VERSION WAS WORSE. It absorbed up to five consecutive throws,
 * re-arming the tick each time, and escalated only after that. The state-change
 * macros in all four drivers are shaped
 *
 *     m_state = y;  state_##x##_exit();  state_##y##_enter();
 *
 * so a throw anywhere inside a transition leaves the NEW state selected with its
 * entry invariants never established. Re-ticking then runs a handler whose
 * _enter() did not happen. The concrete cost, found by review: SmdSat reaching
 * `transmitting` that way holds `transmit_pending`'s poll budget of 3 instead of
 * the one state_transmitting_enter() computes, so the host abandons a live Argos
 * transmission after about 1.4 s instead of 5 s -- or instead of minutes, on a
 * BLIND burst whose window is sized in retx periods. Absorbing bought nothing,
 * because the reset it avoided is the thing that actually repairs a wedged
 * driver, and it cost a corrupted-state re-entry.
 *
 * SO THIS VERSION ONLY LOGS. One line, naming the driver, the state value and
 * the exception type, and then the caller rethrows. That is strictly more than
 * the firmware had before -- main.cpp's handler sees the exception with no idea
 * which driver or which state produced it -- and it is exactly as safe, because
 * the control flow is unchanged.
 *
 * Both entry paths are covered and neither is a surprise. A dispatch posted as a
 * scheduler task lands in main.cpp's catch. A dispatch called synchronously --
 * SmdSat::power_on() does this at smd_sat.cpp:305 -- lands in whichever service
 * called it, inside Service::run_scheduled_task's own barrier, which logs and
 * reschedules the service. Both were already true before this file existed.
 *
 * ONE COST, PAID ON PURPOSE. On that synchronous path
 * Service::handle_task_exception writes its own DEBUG_ERROR, so one exception
 * now costs TWO flash system-log lines instead of one, at roughly 170 ms each.
 * That is strictly worse than before this file existed, and it is the trade:
 * the service's line says a service threw, this one says WHICH DRIVER and in
 * WHICH STATE, which is the information whose absence made the original
 * investigation hard. On the scheduler-posted path the device resets, so the
 * count is bounded by the reset; on the synchronous one it is not, and a
 * driver throwing on every power_on attempt pays 340 ms a time. If that ever
 * matters more than the diagnosis, the line to drop is this one, not the
 * service's.
 *
 * Header-only and hardware-free, so the host tests compile it as-is (the four
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
	/// @param driver  Name used in the log line, e.g. "SmdSat".
	explicit StateMachineBarrier(const char *driver) : m_driver(driver) {}

	/// @brief Name what is being thrown and from where. The caller then rethrows.
	///
	/// MUST be called from inside a `catch (...)` block: it re-throws internally
	/// to match the live exception against the types the firmware actually
	/// throws, which is undefined behaviour with no exception in flight. Same
	/// contract as Service::handle_task_exception.
	///
	/// DEBUG_ERROR, so it survives every DEBUG_LEVEL the shipped variants use
	/// except 0, and so it reaches the flash system log where a recovered tag
	/// can be asked about it afterwards. One line per occurrence is affordable
	/// precisely because the device is about to reset.
	///
	/// @param state  The state value the dispatch was in, for the log line.
	void report(int state) const {
		DEBUG_ERROR("%s::state_machine: %s in state %d — propagating", m_driver, describe_current_exception(), state);
	}

	/// @brief Short name for the exception in flight. Same `catch (...)`
	///        contract as report(); returns a literal, never allocates.
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
};
