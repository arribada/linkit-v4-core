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

	/// @brief Uncertainty of one hop of the pseudo-RTC chain (RSPB / TPL5111).
	///
	/// On a TPL5111 board the time spent unpowered is NOT unknown: it is the wake
	/// period, set by a resistor and exposed as a parameter. The error is therefore
	/// bounded by the timer's tolerance, and it would be a shame to give up time
	/// assistance on that ground alone.
	///
	/// CAUTION: this bound only holds if the wake really came from the TPL. A manual
	/// wake (magnet), a watchdog or a software reset break the chain -- the caller
	/// must then declare RESTORED rather than PSEUDO.
	void set_pseudo_uncertainty_s(unsigned int u) { m_pseudo_unc_s = u; }

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
			// Bound for ONE hop of the chain. It underestimates after several TPL
			// cycles without a single fix, for want of a persisted anchor on the last
			// synchronisation point -- documented, and of no consequence on a BBR
			// board where the injection is skipped anyway.
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
};
