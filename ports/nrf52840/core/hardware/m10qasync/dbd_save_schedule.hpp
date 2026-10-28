/**
 * @file dbd_save_schedule.hpp
 * @brief When the M10Q driver writes the navigation database it fetched (MGA-DBD) to flash.
 */

#pragma once

#include <algorithm>
#include <cstdint>

/// @brief The deadline of the flash copy of the fetched MGA-DBD.
///
/// Writing the dump is a LittleFS commit that holds the cooperative scheduler for
/// 0.5 to 5.5 s (bench 2026-10-12). The RAM copy is what the next session replays;
/// flash is only the copy that survives a power cut -- every TPL cycle on RSPB, a
/// reset on LinkIt. So the write waits for a quiet moment, within two bounds:
/// - never sooner than MIN_DELAY_MS after the fix: right after it the scheduler
///   is busy sending the position frame (b65e7657);
/// - never later than MAX_DELAY_MS after the FIRST fix whose dump is not on flash
///   yet. A later fix replaces the data but does not push the deadline back:
///   otherwise a GNSS session every few minutes would postpone the write forever.
///
/// In between, the driver writes it when told the moment is quiet (a dive,
/// GPSDevice::persist_soon()), and before anything that ends the run or reuses
/// the buffer (power-down, OFF, configuration, watchdog reset, next upload).
class DbdSaveSchedule {
public:
	static constexpr uint64_t MIN_DELAY_MS = 15000;
	/// Past the surfacing burst of the turtle templates (6 messages, 5 s then
	/// +10 s up to 60 s: about 2 to 3 minutes) and short enough that a reset
	/// rarely costs the freshest dump.
	static constexpr uint64_t MAX_DELAY_MS = 5 * 60 * 1000;
	/// Retry when the receiver is busy at the due time.
	static constexpr uint64_t RETRY_MS = 15000;

	/// @brief A session with a fix left its dump in RAM, not on flash.
	/// @return The uptime (ms) at which to write it.
	uint64_t on_fetched(uint64_t now_ms) {
		if (!m_unsaved) {
			m_unsaved = true;
			m_deadline_ms = now_ms + MAX_DELAY_MS;
		}
		return std::max(m_deadline_ms, now_ms + MIN_DELAY_MS);
	}

	/// @brief The dump reached flash: the next fix starts a new window.
	void saved() { m_unsaved = false; }

	bool unsaved() const { return m_unsaved; }
	uint64_t deadline_ms() const { return m_deadline_ms; }

private:
	bool m_unsaved = false;
	uint64_t m_deadline_ms = 0;
};
