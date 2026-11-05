/**
 * @file rate_limiter.hpp
 * @brief Sliding-window TX rate limiter (Plan 1 step 2).
 *
 * Hard cap on Argos TX activity over a rolling time window — designed as a
 * battery defense applied uniformly to every TX (including the first ping of
 * a SURFACING_BURST cycle, per user decision in Plan 1). State persists in
 * .noinit RAM (CRC16-validated) so the budget survives a soft reset; a real
 * power cut clears the ring, which is acceptable: the next bounded burst
 * post-boot is already throttled by GNSS/surface latency.
 *
 * Wired into ArgosTxService at two sites:
 *  - `service_next_schedule_in_ms()`  — gate before scheduling
 *  - `react(KineisEventTxComplete)`   — record successful TX
 *
 * DTE params: `RATE_LIMIT_EN` (RLP01), `RATE_LIMIT_WINDOW_S` (RLP02),
 * `RATE_LIMIT_MAX_TX` (RLP03). On by default since 2026-11-04: 30 TX per
 * 3600 s (it was off). A full window also stops GPSService from starting a
 * session, since no TX could use the fix.
 */

#pragma once

#include <cstdint>
#include <ctime>

class RateLimiter {
public:
	// Hard upper bound on `RATE_LIMIT_MAX_TX`. Sets the noinit RAM footprint
	// (128 × time_t = 1024 bytes + CRC). Beyond this, the cap is rejected at
	// DTE write time (BaseMap range). 128 covers wide-range deployments
	// including multi-day rolling windows with dense surfacing patterns.
	// head/count remain uint8_t (saturate at MAX_CAP, max 255 OK for 128).
	static constexpr unsigned int MAX_CAP = 128;

	// Re-read state from .noinit RAM. Call on boot, before first push/query.
	// CRC mismatch (cold boot, RAM noise) zeros the ring.
	static void restore_state();

	// Push the timestamp of a TX completion. No-op if the limiter is disabled
	// (RATE_LIMIT_EN=false) or the cap is 0.
	static void record_tx(std::time_t now);

	// Query whether a new TX would breach the cap. Always returns false when
	// the limiter is disabled. When true, `reschedule_in_s` is set to the
	// number of seconds until the oldest entry inside the window expires
	// (i.e. when the next TX would again be allowed).
	static bool is_blocked(std::time_t now, unsigned int &reschedule_in_s);

	// Mitigation L (2026-05): clear the ring buffer when the RTC is synced
	// from GNSS. Pre-sync timestamps are in the virtual RTC frame (uptime
	// since boot) and become meaningless once the RTC jumps to real epoch
	// seconds. Without this, the `ts > now → skip` defense filters them all
	// implicitly, but explicit reset is cleaner: future TX records start
	// counting against the cap from the real-time baseline.
	static void reset_for_rtc_sync();

	// Visible-for-testing accessors. Not stable API.
	static unsigned int count_in_window(std::time_t now, unsigned int window_s);
	static void reset_for_tests();
};
