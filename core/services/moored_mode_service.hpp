/**
 * @file moored_mode_service.hpp
 * @brief Moored-vs-underway mode classifier (Cyprus boat tracker).
 *
 * Auto-detects whether the vessel is stationary (alongside, at anchor, on a
 * mooring buoy) or under way, from two cheap sources of evidence:
 *   - GNSS displacement from a fixed reference anchor (haversine), and
 *   - BMA400 wake-on-motion interrupts, when the accelerometer is fitted.
 *
 * When moored, `ConfigurationStore` substitutes the `MOORED_*` parameter
 * variants (GNSS acquisition period, TX interval, GNSS enable) in place of the
 * base ones — cloning the LOW_BATTERY / HAULED override pattern. Priority
 * cascade: LOW_BATTERY > HAULED > MOORED > OUT_OF_ZONE > NORMAL.
 *
 * Detection is asymmetric, and deliberately biased toward *leaving* the
 * battery-saving state:
 *   UNDERWAY -> MOORED : MOORED_ENTER_FIXES consecutive fixes inside
 *                        MOORED_RADIUS_M of the reference anchor.
 *   MOORED -> UNDERWAY : one fix outside the radius, OR a fix reporting
 *                        gSpeed > UNDERWAY_SPEED_MMS, OR MOORED_EXIT_EVENTS
 *                        accelerometer wake-ups (rate-limited by
 *                        MOORED_AXL_HOLDOFF_S).
 *
 * The reference anchor is NOT re-centred while stationary. That is the whole
 * point: a slow drift (a vessel swinging on its anchor, a current) accumulates
 * against a fixed point and eventually crosses the radius on its own. Re-
 * centring on every fix would let the anchor creep indefinitely and the tracker
 * would follow a drifting boat all the way out of the bay while reporting
 * "moored".
 *
 * A false exit is cheap by construction: it costs ONE GNSS acquisition, which
 * re-observes the vessel as stationary and re-enters MOORED after
 * MOORED_ENTER_FIXES fixes. That asymmetry is why the accelerometer threshold
 * can be set aggressively without risking the battery budget.
 *
 * State persists in .noinit RAM with CRC16, so a soft reset resumes the
 * classifier where it left off. A cold boot conservatively resets to UNDERWAY —
 * the device waits for evidence before substituting battery-saving parameters.
 *
 * Hooks (single funnel via the existing peer-event broadcast):
 *  - `ServiceManager::startall()`                 -> `restore_state()`
 *  - `ServiceManager::notify_peer_event()`        -> `on_gnss_fix()` / `on_motion_event()`
 *  - `ConfigurationStore::get_*_configuration()`  -> `evaluate()` + `is_moored()`
 */

#pragma once

#include <cstdint>
#include <ctime>

class MooredModeService {
public:
	/// Ground speed at or above which a fix is treated as "manifestly under
	/// way", regardless of how far it landed from the reference anchor. 1 m/s
	/// is ~3.6 km/h ~ 1.9 kn — below any realistic transit speed, above GNSS
	/// velocity noise on a stationary receiver. Same spirit as the 100 mm/s
	/// stationarity gate in MortalityService, loosened for a vessel.
	static constexpr int32_t UNDERWAY_SPEED_MMS = 1000;

	/// Floor applied to MOORED_RADIUS_M at read time. DTE clamps writes to
	/// >= 10 m, but LittleFS does not checksum file DATA: a corrupted 0 would
	/// make every fix "movement" and silently disable the economy. Not fatal,
	/// but the floor keeps the behaviour predictable.
	static constexpr unsigned int MIN_RADIUS_M = 10;

	// Restore noinit RAM state. Call at boot (ServiceManager::startall).
	static void restore_state();

	// Funnel for valid GNSS fixes. Called from
	// ServiceManager::notify_peer_event when a GNSS_SENSOR log event carries a
	// valid FIX. No-op when MOORED_DETECT_EN is false, so a deployment that
	// never enables the feature accumulates no state at all.
	static void on_gnss_fix(double lat, double lon, int32_t gspeed_mms, std::time_t now);

	// Funnel for accelerometer wake-ups. Called from
	// ServiceManager::notify_peer_event on an AXL_SENSOR log event whose
	// WAKEUP_TRIGGERED port is set. Only counts while MOORED — a wake-up while
	// already under way carries no information.
	static void on_motion_event(std::time_t now);

	// Re-evaluate housekeeping (feature disabled -> force exit; RTC rollback ->
	// re-baseline). Called from ConfigurationStore before every parameter read
	// so a DTE toggle takes effect without a separate service tick. Cheap: one
	// param read plus a short-window cache. Reads `now` from the global RTC;
	// the explicit-arg overload is for tests.
	static void evaluate();
	static void evaluate(std::time_t now);

	// Public state accessors.
	static bool         is_moored();
	static bool         has_reference();
	static unsigned int stationary_fixes();
	static unsigned int motion_events();
	static double       reference_lat();
	static double       reference_lon();

	/// Great-circle distance in metres from the reference anchor to the given
	/// point, or -1.0 when no reference has been established yet. Diagnostic
	/// only (bench console) — the classifier computes its own distance.
	static double distance_to_reference_m(double lat, double lon);

	// Visible-for-tests: clear state directly.
	static void reset_for_tests();
};
