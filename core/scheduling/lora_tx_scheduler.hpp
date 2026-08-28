/**
 * @file lora_tx_scheduler.hpp
 * @brief LoRa TX scheduler — duty cycle, legacy scheduling with jitter.
 */

#pragma once

#include <cstdint>
#include <ctime>
#include <optional>
#include <random>

#include "config_store.hpp"

/// @brief LoRa TX scheduling logic — computes next TX time based on mode.
class LoRaTxScheduler {
private:
	std::optional<uint64_t> m_last_schedule_abs;
	std::optional<uint64_t> m_curr_schedule_abs;
	std::optional<uint64_t> m_earliest_schedule;
	std::mt19937 m_rand;

	static constexpr unsigned int MSECS_PER_SECOND = 1000;
	static constexpr unsigned int SECONDS_PER_HOUR = 3600;
	static constexpr unsigned int SECONDS_PER_DAY = 86400;
	static constexpr unsigned int DUTYCYCLE_24HRS = 0xFFFFFFU;

	int compute_random_jitter(bool jitter_en, int min = -5000, int max = 5000);
	void schedule_periodic(unsigned int period_ms, bool jitter_en, unsigned int duty_cycle, uint64_t now_ms);

public:
	static constexpr unsigned int INVALID_SCHEDULE = static_cast<unsigned int>(-1);

	/// @brief Check if a given time falls within an active duty cycle hour.
	static bool is_in_duty_cycle(uint64_t time_ms, unsigned int duty_cycle);

	LoRaTxScheduler();

	/// @brief Schedule next TX in duty cycle mode.
	/// @param config  Argos configuration (tx_interval, jitter, duty_cycle).
	/// @param now     Current RTC time.
	/// @return Delay in ms until next TX, or INVALID_SCHEDULE.
	unsigned int schedule_duty_cycle(ArgosConfig &config, std::time_t now);

	/// @brief Schedule next TX in legacy mode (24h duty cycle).
	unsigned int schedule_legacy(ArgosConfig &config, std::time_t now);

	/// @brief Set earliest allowed TX time.
	void set_earliest_schedule(std::time_t t);

	/// @brief Reset scheduler and re-seed RNG.
	/// @param seed  RNG seed (typically device ID).
	void reset(unsigned int seed);

	/// @brief Force next TX at absolute time t.
	void schedule_at(std::time_t t);

	/// @brief Notify TX completed — saves timestamp for TR_NOM.
	void notify_tx_complete();
};
