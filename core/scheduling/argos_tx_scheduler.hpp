/**
 * @file argos_tx_scheduler.hpp
 * @brief Argos TX scheduler — duty cycle, legacy, prepass scheduling with jitter.
 */

#pragma once

#include <cstdint>
#include <ctime>
#include <optional>
#include <random>

#include "config_store.hpp"
#include "kineis_device.hpp"

/// @brief Argos TX scheduling logic — computes next TX time based on mode.
class ArgosTxScheduler {
private:
	struct Location {
		double longitude;
		double latitude;
		Location(double x, double y) : longitude(x), latitude(y) {}
	};
	std::optional<uint64_t> m_last_schedule_abs;
	std::optional<uint64_t> m_curr_schedule_abs;
	std::optional<uint64_t> m_earliest_schedule;
	std::mt19937 m_rand;
	std::optional<Location> m_location;

	static constexpr unsigned int SECONDS_PER_MINUTE   = 60;
	static constexpr unsigned int MINUTES_PER_HOUR     = 60;
	static constexpr unsigned int HOURS_PER_DAY        = 24;
	static constexpr unsigned int MSECS_PER_SECOND     = 1000;
	static constexpr unsigned int SECONDS_PER_HOUR     = MINUTES_PER_HOUR * SECONDS_PER_MINUTE;
	static constexpr unsigned int SECONDS_PER_DAY      = HOURS_PER_DAY * SECONDS_PER_HOUR;
	static constexpr unsigned int DUTYCYCLE_24HRS      = 0xFFFFFFU;
	static constexpr unsigned int ARGOS_TX_MARGIN_MSECS = 0;

	int compute_random_jitter(bool jitter_en, int min = -5000, int max = 5000);
	void schedule_periodic(unsigned int period_ms, bool jitter_en, unsigned int duty_cycle, uint64_t now_ms);

public:
	static constexpr unsigned int INVALID_SCHEDULE = static_cast<unsigned int>(-1);

	/// @brief Check if a given time falls within an active duty cycle hour.
	static bool is_in_duty_cycle(uint64_t time_ms, unsigned int duty_cycle);

	ArgosTxScheduler();

	/// @brief Schedule next TX in duty cycle mode.
	/// @return Delay in ms until next TX, or INVALID_SCHEDULE.
	unsigned int schedule_duty_cycle(ArgosConfig& config, std::time_t now);

	/// @brief Schedule next TX in legacy mode (24h duty cycle).
	unsigned int schedule_legacy(ArgosConfig& config, std::time_t now);

	/// @brief Schedule next TX using satellite pass prediction (PREVIPASS).
	/// @param config         Argos configuration (prepass params, tx_interval, jitter).
	/// @param pass_predict   AOP satellite pass prediction database.
	/// @param now            Current RTC time.
	/// @return Delay in ms until next TX, or INVALID_SCHEDULE.
	///
	/// 2026-08 — le parametre de sortie `scheduled_mode` a ete SUPPRIME. Il
	/// n'etait jamais relu par l'ordonnanceur, seulement ecrit, et il ecrasait
	/// sans condition la modulation que le service venait de resoudre a partir
	/// de la configuration utilisateur. Choisir la modulation est une decision
	/// de politique: elle appartient au service, comme pour LEGACY, DUTY_CYCLE
	/// et SURFACING_BURST.
	unsigned int schedule_prepass(ArgosConfig& config, BasePassPredict& pass_predict,
	                              std::time_t now);

	/// @brief Set earliest allowed TX time (e.g., after SWS dry_time_before_tx).
	void set_earliest_schedule(std::time_t t);

	/// @brief Update last known GPS position for prepass computation.
	void set_last_location(double lon, double lat);

	/// @brief Get absolute time of last schedule (ms).
	unsigned int get_last_schedule();

	/// @brief Reset scheduler state and seed RNG.
	void reset(unsigned int seed);

	/// @brief Force next TX at absolute time t.
	void schedule_at(std::time_t t);

	/// @brief Notify that TX completed — updates last schedule timestamp.
	void notify_tx_complete();
};
