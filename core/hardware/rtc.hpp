#pragma once

/**
 * @file rtc.hpp
 * @brief Abstract real-time clock interface (epoch seconds) + provenance de l'heure.
 */

#include <cstdint>
#include <ctime>

/// @brief Where the time the RTC currently holds came from.
///
/// Without that distinction, `is_set()` answers "yes" both for a satellite-
/// synchronised time and for a value restored from flash that may be weeks
/// behind -- and the firmware announced both to the GNSS receiver with the same
/// +/- 2 s confidence. Measured on the bench on 2026-08-25: after a reset,
/// `LAST_KNOWN_RTC` restored a time 52 days old, and the assistance data saved
/// three minutes earlier was thrown away by the anti-rollback guard.
enum class RtcSource : uint8_t {
	NONE = 0,  ///< jamais posee, ou horloge virtuelle de repli
	RESTORED,  ///< relue du flash au demarrage — erreur NON bornable
	PSEUDO,    ///< chaine pseudo-RTC TPL5111 — erreur BORNEE par la periode
	OPERATOR,  ///< posee a la main en configuration ($RTCW)
	GNSS,      ///< synchronisee sur un PVT valide
};

class RTC {
public:
	virtual ~RTC() = default;
	virtual std::time_t gettime() = 0;           ///< Current epoch time (seconds since 1970)
	virtual void settime(std::time_t time) = 0;  ///< Set wall-clock time (typically from GNSS fix)
	virtual bool is_set() = 0;                   ///< True if settime() has been called at least once

	// ─────────────────────────────────────────────────────────────────────
	//  Provenance et derive (2026-08) — logique commune, pas de portage.
	// ─────────────────────────────────────────────────────────────────────

	/// Minimum window between two GNSS synchronisations for a drift measurement to
	/// mean anything. Below it, the one-second quantum dominates the result.
	static constexpr std::time_t MIN_DRIFT_WINDOW_S = 900;

	/// Drift assumed while no measurement is available. An uncompensated
	/// 32.768 kHz watch crystal typically holds +/- 20 ppm over the temperature
	/// range a tag sees.
	static constexpr int32_t DEFAULT_DRIFT_PPM = 20;

	/// Credibility bound on a drift measurement: beyond it, that is not a crystal
	/// drifting, it is a clock that jumped.
	static constexpr int32_t MAX_CREDIBLE_PPM = 200;

	RtcSource source() const { return m_source; }
	int32_t drift_ppm() const { return m_drift_ppm; }

	/// @brief The value to persist in LAST_KNOWN_RTC.
	///
	/// On a TPL5111 board (EXTERNAL_WAKEUP) the next boot rebuilds its clock as
	/// LAST_KNOWN_RTC + WAKEUP_PERIOD. The TPL5111 period runs from one DRV rising
	/// edge to the next -- DONE only shortens the high time (TI SNAS659B, timer
	/// mode) -- so the right base is the instant THIS run booted, not the instant
	/// it powers down. Persisting the power-down time pushed the chain ahead by the
	/// length of every working run (GNSS plus the transmissions: minutes), on top
	/// of the resistor tolerance it was meant to carry alone.
	///
	/// @param now        current RTC time
	/// @param uptime_ms  time since this boot (PMU::get_timestamp_ms())
	static std::time_t persisted_time(std::time_t now, uint64_t uptime_ms) {
#ifdef EXTERNAL_WAKEUP
		const std::time_t up_s = static_cast<std::time_t>(uptime_ms / 1000ULL);
		return (now > up_s) ? (now - up_s) : now;
#else
		(void)uptime_ms;
		return now;
#endif
	}

	/// Fixed part of the pseudo-RTC uncertainty: boot-latency jitter between the
	/// DRV edge and the timestamp origin, and the accuracy of the anchor itself
	/// ($RTCW typed from a PC is a few seconds at best).
	static constexpr unsigned int PSEUDO_BASE_UNC_S = 10;

	/// @brief Honest uncertainty of a pseudo-RTC value, or 0 if it cannot be bounded.
	///
	/// The TPL5111 period is set by a resistor: each hop adds up to its tolerance
	/// (TplClock: 5 % with the nominal period, 2 % once the board's real period has
	/// been learnt from GNSS fixes), and the hops add up. What bounds the error is
	/// therefore the time elapsed since the last TRUSTED setting (a GNSS fix or
	/// $RTCW, persisted in LAST_TIME_SYNC), not the length of the current hop: the
	/// one-hop bound used before claimed +/-3 min after a day without a fix, when the
	/// clock could be a whole hour out.
	///
	/// @param pseudo_now          the pseudo-RTC value just computed
	/// @param last_sync           LAST_TIME_SYNC (0 = never)
	/// @param tolerance_permille  tolerance per hop
	/// @return seconds, or 0 when there is no usable anchor (never synchronised, an
	///         anchor in the future, or a bound past what MGA-INI-TIME can carry)
	static unsigned int pseudo_uncertainty_s(std::time_t pseudo_now, std::time_t last_sync,
	                                         unsigned int tolerance_permille) {
		if (last_sync <= 0 || pseudo_now < last_sync) return 0;
		const unsigned long elapsed = static_cast<unsigned long>(pseudo_now - last_sync);
		const unsigned long unc = PSEUDO_BASE_UNC_S + (elapsed * tolerance_permille + 999UL) / 1000UL;
		return (unc > 65535UL) ? 0u : static_cast<unsigned int>(unc);
	}

	/// @brief Uncertainty of the pseudo-RTC value (RSPB / TPL5111), as computed by
	/// pseudo_uncertainty_s().
	///
	/// On a TPL5111 board the time spent unpowered is NOT unknown: it is the wake
	/// period, set by a resistor and exposed as a parameter. The error is therefore
	/// bounded by the timer's tolerance -- accumulated over every hop since the last
	/// trusted setting -- and it would be a shame to give up time assistance on that
	/// ground alone.
	///
	/// CAUTION: this bound only holds if the wake really came from the TPL. A manual
	/// wake (magnet), a watchdog or a software reset break the chain -- the caller
	/// must then declare RESTORED rather than PSEUDO.
	void set_pseudo_uncertainty_s(unsigned int u) { m_pseudo_unc_s = u; }

	/// @brief Can a time set during THIS run anchor the TPL5111 chain?
	///
	/// Only if the run started on a TPL edge: the next wake then comes one period
	/// after this boot. A run started by anything else (magnet, watchdog, software
	/// reset, OTA) has no known phase against the TPL timer, so a fix or a $RTCW
	/// taken during it says nothing about when the next TPL wake will come. Set at
	/// boot by the TPL5111 board code; always true elsewhere.
	void set_chain_anchorable(bool anchorable) { m_chain_anchorable = anchorable; }
	bool chain_anchorable() const { return m_chain_anchorable; }

	/// @brief Value to persist in LAST_TIME_SYNC for a trusted setting at `now`:
	/// `now`, or 0 ("no anchor") when this run cannot anchor the chain.
	unsigned int anchor_value(std::time_t now) const {
		return m_chain_anchorable ? static_cast<unsigned int>(now) : 0u;
	}

	/// @brief Age, in seconds, of the last time that was set.
	unsigned int age_s() {
		if (m_source == RtcSource::NONE) return 0;
		std::time_t now = gettime();
		return (now > m_set_at) ? static_cast<unsigned int>(now - m_set_at) : 0u;
	}

	/// @brief Declare where the time just set came from.
	/// To be called just AFTER settime().
	void note_source(RtcSource src) {
		m_source = src;
		m_set_at = gettime();
	}

	/// @brief Set a GNSS time and measure the crystal drift on the way.
	///
	/// @param prev the time the RTC held just before (what it believed)
	/// @param now  the true time, from the PVT
	///
	/// The gap between the two, over the time elapsed since the previous
	/// synchronisation, IS the drift. No probe to add: both values are already read
	/// at the synchronisation point.
	void note_gnss_sync(std::time_t prev, std::time_t now) {
		constexpr std::time_t RTC_MIN_REAL = 946684800;  // 2000-01-01

		if (m_source == RtcSource::GNSS && m_last_gnss_sync >= RTC_MIN_REAL && prev >= RTC_MIN_REAL
		    && now >= RTC_MIN_REAL) {
			std::time_t elapsed = now - m_last_gnss_sync;
			if (elapsed >= MIN_DRIFT_WINDOW_S) {
				long err = static_cast<long>(prev - now);
				long ppm = (err * 1000000L) / static_cast<long>(elapsed);
				if (ppm <= MAX_CREDIBLE_PPM && ppm >= -MAX_CREDIBLE_PPM) {
					// Gentle smoothing: a single measurement carries the one-second
					// quantum, the average converges over a few sessions.
					m_drift_ppm = (m_drift_ppm == 0)
					                  ? static_cast<int32_t>(ppm)
					                  : static_cast<int32_t>((3 * static_cast<long>(m_drift_ppm) + ppm) / 4);
				}
			}
		}

		m_source = RtcSource::GNSS;
		m_set_at = now;
		m_last_gnss_sync = now;
	}

	/// @brief Honest uncertainty on the current time, in seconds.
	/// @return 0 = do NOT inject time assistance (the error cannot be bounded).
	///
	/// This is the value that must fill the `tAccS` field of MGA-INI-TIME.
	/// Announcing 2 s on a restored time means asserting a false constraint to the
	/// receiver, which it will then use to narrow its search.
	unsigned int time_accuracy_s() {
		unsigned int base;
		if (m_source == RtcSource::GNSS) {
			base = 2;
		} else if (m_source == RtcSource::OPERATOR) {
			base = 60;
		} else if (m_source == RtcSource::PSEUDO) {
			// 2026-10: bound accumulated since the last trusted setting (see
			// pseudo_uncertainty_s), no longer the length of one hop.
			base = (m_pseudo_unc_s > 0) ? m_pseudo_unc_s : 60;
		} else {
			// NONE or RESTORED: the time spent unpowered is unknown, so the error
			// cannot be bounded. Better to assert nothing to the receiver.
			return 0;
		}
		int32_t d = (m_drift_ppm < 0) ? -m_drift_ppm : m_drift_ppm;
		if (d == 0) d = DEFAULT_DRIFT_PPM;
		unsigned long acc = base + (static_cast<unsigned long>(d) * static_cast<unsigned long>(age_s())) / 1000000UL;
		return (acc > 65535UL) ? 65535u : static_cast<unsigned int>(acc);
	}

protected:
	RtcSource m_source = RtcSource::NONE;
	std::time_t m_set_at = 0;          ///< heure a laquelle la pose a eu lieu
	std::time_t m_last_gnss_sync = 0;  ///< derniere synchro GNSS (pour la derive)
	int32_t m_drift_ppm = 0;           ///< derive MESUREE, 0 tant qu'inconnue
	unsigned int m_pseudo_unc_s = 0;   ///< incertitude d'un bond pseudo-RTC
	bool m_chain_anchorable = true;    ///< see set_chain_anchorable()
};
