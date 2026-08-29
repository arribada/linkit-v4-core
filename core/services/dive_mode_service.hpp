/**
 * @file dive_mode_service.hpp
 * @brief Dive mode service — pauses reed switch after sustained submersion to prevent accidental triggers.
 */

#pragma once

#include "service.hpp"
#include "switch.hpp"

/// @brief Dive mode — disables reed switch when submerged for configurable duration.
class DiveModeService : public Service {
public:
	enum class DiveState { Idle, StartPending, Engaged };

	DiveModeService(Switch &reed)
	    : Service(ServiceIdentifier::DIVE_MODE, "DIVE", nullptr),
	      m_reed(reed),
	      m_dive_state(DiveState::Idle) {}

	void notify_peer_event(ServiceEvent &e) {
		// Check for UW event
		if (e.event_source != ServiceIdentifier::UW_SENSOR || e.event_type != ServiceEventType::SERVICE_LOG_UPDATED) {
			return;
		}

		// Get UW state (0=>surfaced, 1=>submerged)
		bool uw_state = std::get<bool>(e.event_data);

		// SAFETY UNCONDITIONAL: surface always disengages a previously-paused
		// reed. We process this BEFORE the `service_is_enabled()` gate so that
		// disabling UW_DIVE_MODE_ENABLE mid-Engaged (DTE config change) cannot
		// leave the magnet inert. Idempotent if state was already not Engaged.
		if (!uw_state && m_dive_state == DiveState::Engaged) {
			DEBUG_INFO("DiveModeService: dive mode disengaged by surfacing event");
			m_dive_state = DiveState::Idle;
			m_reed.resume();
			return;
		}
		if (!uw_state && m_dive_state == DiveState::StartPending) {
			// Surfaced BEFORE the start timer fired — cancel pending state.
			// Without this, the timer fires later (still at the surface) and
			// pauses the reed, leaving the magnet inert until the next dive
			// completes a full cycle. Reed was never paused yet, so no resume
			// is needed; the pending scheduled task will fire harmlessly
			// because service_initiate() only acts when state == StartPending.
			DEBUG_INFO("DiveModeService: dive mode start cancelled — surfaced before %us start timer",
			           service_read_param<unsigned int>(ParamID::UW_DIVE_MODE_START_TIME));
			m_dive_state = DiveState::Idle;
			return;
		}

		// Dive event — only meaningful when dive mode is enabled
		if (!service_is_enabled()) {
			return;
		}

		if (uw_state && m_dive_state == DiveState::Idle) {
			// Enter start pending state and reschedule the service which will call us
			// back after the dive mode start time period has elapsed
			DEBUG_INFO("DiveModeService: dive mode start pending");
			m_dive_state = DiveState::StartPending;
			service_reschedule();
		}
	}

private:
	Switch &m_reed;
	DiveState m_dive_state;

protected:
	void service_initiate() {
		// If dive state is start pending then engage the dive state
		// and pause any reed switch activity
		if (m_dive_state == DiveState::StartPending) {
			DEBUG_INFO("DiveModeService: dive mode engaged");
			m_dive_state = DiveState::Engaged;
			m_reed.pause();
		}
		service_complete();
	}

	unsigned int service_next_schedule_in_ms() override {
		// If dive mode start is pending then return the timer for scheduling
		// when to engage dive mode
		if (m_dive_state == DiveState::StartPending) {
			// Clamp before the multiply, in 64 bits. UNP13 is a delay in
			// SECONDS and the scheduler wants milliseconds, so `* 1000`
			// overflowed an unsigned int above 4294967 s (~49.7 days): a dive
			// scheduled seven weeks out engaged 0.7 s later, and engaging pauses
			// the reed switch -- the operator loses magnet control at the exact
			// moment he believes nothing is armed yet. The DTE bound now stops
			// such a value arriving, but a device configured before that bound
			// still holds one in flash, so the clamp belongs here too.
			constexpr uint64_t MAX_START_DELAY_S = 86400;  // one day, as elsewhere
			uint64_t start_s = service_read_param<unsigned int>(ParamID::UW_DIVE_MODE_START_TIME);
			if (start_s > MAX_START_DELAY_S) {
				DEBUG_WARN("DiveModeService: UW_DIVE_MODE_START_TIME=%llu s is beyond the %llu s ceiling — clamped",
				           (unsigned long long)start_s, (unsigned long long)MAX_START_DELAY_S);
				start_s = MAX_START_DELAY_S;
			}
			return (unsigned int)(start_s * 1000u);
		} else {
			return SCHEDULE_DISABLED;
		}
	}

	bool service_is_enabled() override { return service_read_param<bool>(ParamID::UW_DIVE_MODE_ENABLE); }

	bool service_is_usable_underwater() override { return true; }

	void service_init() override { m_dive_state = DiveState::Idle; }

	void service_term() override {
		// Restore reed to default-active on shutdown, regardless of whether
		// UW_DIVE_MODE_ENABLE is still true. If the param was toggled off
		// while the state was Engaged, the previous code would have skipped
		// resume() and left the magnet inert until next reboot.
		if (m_dive_state == DiveState::Engaged) {
			m_reed.resume();
		}
		m_dive_state = DiveState::Idle;
	}
};
