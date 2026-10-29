/**
 * @file hauled_mode_service.hpp
 * @brief Hauled-vs-at-sea mode classifier (Plan 1 step 3).
 *
 * Auto-detects whether the animal is hauled out (dry on the SWS for a long
 * time) or at sea, based on how long it has been continuously out of the
 * water. A long dive is not a dry spell. When hauled, `ConfigurationStore`
 * substitutes the `HAULED_*` parameter variants (Argos mode, TR_NOM, GNSS_EN)
 * in place of the base ones — cloning the LOW_BATTERY override pattern.
 *
 * Detection is asymmetric to provide hysteresis on a sandy / surf boundary:
 *   AT_SEA → HAULED : dry now, and (now - dry_since) > HAULED_IDLE_THRESHOLD_H * 3600
 *   HAULED → AT_SEA : HAULED_RETURN_EVENTS consecutive UW=true events
 * A device that has never been in the water stays AT_SEA.
 *
 * State (in_hauled / counters / stamps) persists in .noinit RAM with CRC16, so
 * a soft reset resumes the classifier where it left off; HAULED cannot engage
 * after a reset until the detector has reported again. A cold boot
 * conservatively resets to AT_SEA — the device waits for evidence before
 * substituting battery-saving HAULED parameters.
 *
 * Priority below LOW_BATTERY (existing) and above any future sequencer
 * (`AT_SEA_SEQUENCED`, Plan 2): when hauled, the sequencer is paused.
 *
 * Hooks (single funnel via the existing UW event broadcast):
 *  - `ServiceManager::notify_peer_event()` → `on_underwater_event()`
 *  - `ServiceManager::startall()`          → `restore_state()`
 *  - `ConfigurationStore::get_*_configuration()` → `evaluate(now)` + `is_hauled()`
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <ctime>

class HauledModeService {
public:
	// Restore noinit RAM state. Call at boot (ServiceManager::startall).
	static void restore_state();

	// Single funnel point for UW events. Called from
	// ServiceManager::notify_peer_event when an SWS event lands.
	// `submerged` = true on dive (UW=wet), false on surfacing.
	// `now` is the RTC timestamp of the event.
	static void on_underwater_event(bool submerged, std::time_t now);

	// Re-evaluate AT_SEA → HAULED transition. Called from ConfigurationStore
	// before every parameter read so the override engages without a separate
	// service tick. Cheap: a few param reads + one subtraction. No-op if
	// `HAULED_DETECT_EN = false` or RTC not set.
	// Reads `now` from the global RTC; the explicit-arg overload is for tests.
	static void evaluate();
	static void evaluate(std::time_t now);

	// Public state accessors.
	static bool is_hauled();
	static std::time_t last_uw_event_rtc();  // last dive; 0 = never in the water
	static std::time_t dry_since_rtc();      // start of the current dry spell; 0 = in the water or unknown
	static unsigned uw_events_since_hauled();

	// Mitigation K (2026-05): re-baseline timestamp when the RTC is synced
	// from GNSS (jump from virtual epoch ~1 to real UTC ~1.7e9). Called by
	// GPSService right after rtc->settime(real_t). Without this, the dry-time
	// computation `(now - dry_since_rtc)` would jump by ~50 years,
	// falsely engaging HAULED on the very next evaluate() call.
	// Preserves `in_hauled` (logical state stays valid) and
	// `uw_events_since_hauled`; only the non-zero stamps are shifted to `now`.
	static void reset_for_rtc_sync(std::time_t now);

	// Visible-for-tests: clear state directly.
	static void reset_for_tests();

#ifdef CPPUTEST
	// Drop the RAM-only state as a reset does; .noinit is kept.
	static void simulate_reset_for_tests();
	// Raw .noinit image, to corrupt it or plant another layout.
	static uint8_t *noinit_bytes_for_tests(std::size_t &size);
#endif
};
