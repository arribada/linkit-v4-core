#pragma once

/**
 * @file rtc.hpp
 * @brief Abstract real-time clock interface (epoch seconds) + provenance de l'heure.
 */

#include <cstdint>
#include <ctime>

/// @brief D'ou vient l'heure que porte la RTC en ce moment.
///
/// Sans cette distinction, `is_set()` repond « oui » aussi bien pour une heure
/// synchronisee sur satellite que pour une valeur restauree du flash qui peut
/// avoir des semaines de retard — et le firmware annoncait les deux au recepteur
/// GNSS avec la meme confiance de +/- 2 s. Mesure au banc le 2026-08-25: apres un
/// reset, `LAST_KNOWN_RTC` a repose une heure vieille de 52 jours, et l'assistance
/// sauvegardee trois minutes plus tot a ete jetee par la garde anti-recul.
enum class RtcSource : uint8_t {
	NONE = 0,   ///< jamais posee, ou horloge virtuelle de repli
	RESTORED,   ///< relue du flash au demarrage — erreur NON bornable
	PSEUDO,     ///< chaine pseudo-RTC TPL5111 — erreur BORNEE par la periode
	OPERATOR,   ///< posee a la main en configuration ($RTCW)
	GNSS,       ///< synchronisee sur un PVT valide
};

class RTC {
public:
	virtual ~RTC() = default;
	virtual std::time_t gettime() = 0;          ///< Current epoch time (seconds since 1970)
	virtual void settime(std::time_t time) = 0;  ///< Set wall-clock time (typically from GNSS fix)
	virtual bool is_set() = 0;                   ///< True if settime() has been called at least once

	// ─────────────────────────────────────────────────────────────────────
	//  Provenance et derive (2026-08) — logique commune, pas de portage.
	// ─────────────────────────────────────────────────────────────────────

	/// Fenetre minimale entre deux synchros GNSS pour que la mesure de derive
	/// ait un sens. En dessous, le quantum d'une seconde domine le resultat.
	static constexpr std::time_t MIN_DRIFT_WINDOW_S = 900;

	/// Derive supposee tant qu'aucune mesure n'est disponible. Un quartz
	/// horloger 32,768 kHz non compense tient typiquement +/- 20 ppm sur la
	/// plage de temperature d'une balise.
	static constexpr int32_t DEFAULT_DRIFT_PPM = 20;

	/// Borne de credibilite d'une mesure de derive: au-dela, ce n'est plus un
	/// quartz qui derive, c'est une horloge qui a saute.
	static constexpr int32_t MAX_CREDIBLE_PPM = 200;

	RtcSource source() const { return m_source; }
	int32_t drift_ppm() const { return m_drift_ppm; }

	/// @brief Incertitude d'un bond de la chaine pseudo-RTC (RSPB / TPL5111).
	///
	/// Sur une carte a TPL5111, le temps hors tension N'EST PAS inconnu: il vaut
	/// la periode de reveil, fixee par une resistance et exposee en parametre.
	/// L'erreur se borne donc a la tolerance du minuteur, et il serait dommage
	/// de se priver d'assistance temporelle sur ce seul motif.
	///
	/// ATTENTION: cette borne ne vaut que si le reveil vient bien du TPL. Un
	/// reveil manuel (aimant), un WDT ou un reset logiciel cassent la chaine —
	/// l'appelant doit alors declarer RESTORED et non PSEUDO.
	void set_pseudo_uncertainty_s(unsigned int u) { m_pseudo_unc_s = u; }

	/// @brief Age, en secondes, de la derniere pose d'heure.
	unsigned int age_s() {
		if (m_source == RtcSource::NONE)
			return 0;
		std::time_t now = gettime();
		return (now > m_set_at) ? static_cast<unsigned int>(now - m_set_at) : 0u;
	}

	/// @brief Declare d'ou vient l'heure qui vient d'etre posee.
	/// A appeler juste APRES settime().
	void note_source(RtcSource src) {
		m_source = src;
		m_set_at = gettime();
	}

	/// @brief Pose une heure GNSS et mesure la derive du quartz au passage.
	///
	/// @param prev heure que portait la RTC juste avant (sa croyance)
	/// @param now  heure vraie, issue du PVT
	///
	/// L'ecart entre les deux, rapporte au temps ecoule depuis la synchro
	/// precedente, EST la derive. Aucune sonde a ajouter: les deux valeurs sont
	/// deja lues au point de synchronisation.
	void note_gnss_sync(std::time_t prev, std::time_t now) {
		constexpr std::time_t RTC_MIN_REAL = 946684800;  // 2000-01-01

		if (m_source == RtcSource::GNSS && m_last_gnss_sync >= RTC_MIN_REAL &&
		    prev >= RTC_MIN_REAL && now >= RTC_MIN_REAL) {
			std::time_t elapsed = now - m_last_gnss_sync;
			if (elapsed >= MIN_DRIFT_WINDOW_S) {
				long err = static_cast<long>(prev - now);
				long ppm = (err * 1000000L) / static_cast<long>(elapsed);
				if (ppm <= MAX_CREDIBLE_PPM && ppm >= -MAX_CREDIBLE_PPM) {
					// Lissage doux: une mesure isolee porte le quantum d'une
					// seconde, la moyenne converge sur quelques sessions.
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

	/// @brief Incertitude honnete sur l'heure courante, en secondes.
	/// @return 0 = ne PAS injecter d'assistance temporelle (erreur non bornable).
	///
	/// C'est cette valeur qui doit remplir le champ `tAccS` de MGA-INI-TIME.
	/// Annoncer 2 s sur une heure restauree revient a affirmer au recepteur une
	/// contrainte fausse qu'il va utiliser pour restreindre sa recherche.
	unsigned int time_accuracy_s() {
		// if/else et non switch: le projet compile avec -Werror=switch-enum ET
		// -Werror=switch-default, ce qui rend tout switch sur enum class verbeux
		// pour rien ici.
		unsigned int base;
		if (m_source == RtcSource::GNSS) {
			base = 2;
		} else if (m_source == RtcSource::OPERATOR) {
			base = 60;
		} else if (m_source == RtcSource::PSEUDO) {
			// Borne d'UN bond de la chaine. Elle sous-estime apres plusieurs
			// cycles TPL sans le moindre fix, faute d'un ancrage persiste du
			// dernier point de synchro — documente, et sans consequence sur une
			// carte a BBR ou l'injection est de toute facon sautee.
			base = (m_pseudo_unc_s > 0) ? m_pseudo_unc_s : 60;
		} else {
			// NONE ou RESTORED: le temps passe hors tension est inconnu, donc
			// l'erreur ne se borne pas. Mieux vaut ne rien affirmer au recepteur.
			return 0;
		}
		int32_t d = (m_drift_ppm < 0) ? -m_drift_ppm : m_drift_ppm;
		if (d == 0)
			d = DEFAULT_DRIFT_PPM;
		unsigned long acc = base +
			(static_cast<unsigned long>(d) * static_cast<unsigned long>(age_s())) / 1000000UL;
		return (acc > 65535UL) ? 65535u : static_cast<unsigned int>(acc);
	}

protected:
	RtcSource   m_source         = RtcSource::NONE;
	std::time_t m_set_at         = 0;   ///< heure a laquelle la pose a eu lieu
	std::time_t m_last_gnss_sync = 0;   ///< derniere synchro GNSS (pour la derive)
	int32_t     m_drift_ppm      = 0;   ///< derive MESUREE, 0 tant qu'inconnue
	unsigned int m_pseudo_unc_s  = 0;   ///< incertitude d'un bond pseudo-RTC
};
