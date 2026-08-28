/**
 * @file uwdetector_service.hpp
 * @brief Underwater detector service — periodic SWS/pressure sampling to detect submersion.
 */

#pragma once

#include <functional>
#include "scheduler.hpp"
#include "service.hpp"

/// @brief Underwater detector — samples a switch/sensor to determine surface vs submerged state.
class UWDetectorService : public Service {
public:
	/// @param name  Service name (default "UWDetector").
	UWDetectorService(const char *name = "UWDetector") : Service(ServiceIdentifier::UW_SENSOR, name) {}
	virtual ~UWDetectorService() {}

protected:
	double m_activation_threshold = 0.0;
	unsigned int m_enable_sample_delay = 0;
	bool m_current_state = false;

	/// @brief Read the current detector state (subclasses override for SWS/pressure).
	/// @return true if submerged.
	virtual bool detector_state() { return false; }
	virtual bool service_is_enabled() = 0;
	void service_init() override;

private:
	bool m_is_first_time = true;
	bool m_pending_state = false;
	unsigned int m_period_underwater_ms = 0;
	unsigned int m_period_surface_ms = 0;
	unsigned int m_sample_iteration = 0;
	unsigned int m_sample_gap = 0;
	unsigned int m_min_dry_samples = 0;
	unsigned int m_dry_count = 0;

protected:
	unsigned int m_max_samples = 0;

	void service_term() override;
	unsigned int service_next_schedule_in_ms() override;
	void service_initiate() override;
	bool service_is_usable_underwater() override;
	// FIX 2026-05-23 (audit SWS finding 2.1 / UWD finding 3.1):
	// reset the in-progress sample iteration state too. Previously only
	// m_is_first_time was set, leaving m_sample_iteration / m_dry_count /
	// m_pending_state at their mid-cycle values from before the cooldown.
	// After a long cooldown (minutes/hours) the early samples are stale —
	// conditions may have flipped surface↔UW during the pause — and
	// continuing the iteration could produce a wrong terminal verdict.
	// Force a fresh cycle from sample 0 on every cooldown-exit.
	void reset_state_for_cooldown_exit() override {
		m_is_first_time = true;
		m_sample_iteration = 0;
		m_dry_count = 0;
		m_pending_state = false;
	}
};
