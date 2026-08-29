/**
 * @file lora_tx_scheduler.cpp
 * @brief LoRa TX scheduler — duty cycle, legacy, periodic scheduling with jitter.
 */

#include "lora_tx_scheduler.hpp"
#include "debug.hpp"
#include "error.hpp"

LoRaTxScheduler::LoRaTxScheduler() : m_rand(std::mt19937()) {
	m_last_schedule_abs.reset();
	m_curr_schedule_abs.reset();
	m_earliest_schedule.reset();
}

void LoRaTxScheduler::reset(unsigned int seed) {
	m_earliest_schedule.reset();
	m_last_schedule_abs.reset();
	m_rand.seed(seed);
}

bool LoRaTxScheduler::is_in_duty_cycle(uint64_t time_ms, unsigned int duty_cycle) {
	// Duty cycle is a 24-bit field, one bit per hour of the day
	uint64_t msec_of_day = (time_ms % (SECONDS_PER_DAY * MSECS_PER_SECOND));
	unsigned int hour_of_day = msec_of_day / (SECONDS_PER_HOUR * MSECS_PER_SECOND);
	return (duty_cycle & (0x800000 >> hour_of_day));
}

int LoRaTxScheduler::compute_random_jitter(bool jitter_en, int min, int max) {
	if (jitter_en) {
		std::uniform_int_distribution<int> dist(min, max);
		int jitter = dist(m_rand);
		DEBUG_TRACE("LoRaTxScheduler::compute_random_jitter: jitter=%d", jitter);
		return jitter;
	} else {
		return 0;
	}
}

void LoRaTxScheduler::schedule_periodic(unsigned int period_ms, bool jitter_en, unsigned int duty_cycle,
                                        uint64_t now_ms) {
	uint64_t start_time;

	// Sealed-device guard (audit 2026-07): the 24 h search loop below advances
	// by period_ms — with 0 it never terminates AND never kicks the WDT, so a
	// corrupted stored TR_NOM (DTE clamps writes to >=30 s but LittleFS does
	// not checksum file DATA) would spin until the WDT resets, x3 -> factory
	// reset. Fail cleanly instead — both callers catch this and return
	// INVALID_SCHEDULE. (Mirrors the same guard in ArgosTxScheduler.)
	// A mask of zero permits NO hour at all, so the 24 h search below finds
	// nothing and throws, and the caller turns that into INVALID_SCHEDULE, which
	// Service::reschedule treats as "post nothing". Zero is also the factory
	// default, so selecting DUTY_CYCLE and forgetting the mask is a one-step
	// mistake that silences the beacon with no visible symptom. Same fallback
	// and same reasoning as ArgosTxScheduler: transmitting on the nominal period
	// is a configuration to correct, a mute sealed tag is a lost deployment.
	if (duty_cycle == 0) {
		DEBUG_ERROR("LoRaTxScheduler::schedule_periodic: duty-cycle mask is 0 — no hour is permitted. Check "
		            "ARGOS_DUTY_CYCLE (or LB_ARGOS_DUTY_CYCLE). Falling back to the nominal period so the beacon "
		            "still transmits.");
		duty_cycle = 0xFFFFFFu;  // all 24 hours
	}

	if (period_ms == 0) {
		DEBUG_ERROR("LoRaTxScheduler::schedule_periodic: period_ms=0 (corrupt config?) — TX schedule aborted");
		m_curr_schedule_abs.reset();
		throw ErrorCode::RESOURCE_NOT_AVAILABLE;
	}

	DEBUG_TRACE("LoRaTxScheduler::schedule_periodic: now=%llu last=%llu tr=%u jitter=%u", now_ms,
	            m_last_schedule_abs.has_value() ? m_last_schedule_abs.value() : 0, period_ms, jitter_en);

	// Handle earliest TX time
	while (m_earliest_schedule.has_value()) {
		DEBUG_TRACE("LoRaTxScheduler::schedule_periodic: earliest TX is in %llu", m_earliest_schedule.value() - now_ms);

		if (m_earliest_schedule.value() >= now_ms) {
			start_time = m_earliest_schedule.value();
			if (m_last_schedule_abs.has_value() && start_time < m_last_schedule_abs.value())
				start_time = m_last_schedule_abs.value();

			if (is_in_duty_cycle(start_time, duty_cycle)) {
				m_curr_schedule_abs = start_time;
				return;
			} else {
				break;
			}
		} else {
			m_earliest_schedule.reset();
		}
		break;
	}

	// Compute new schedule
	if (!m_last_schedule_abs.has_value()) {
		start_time = now_ms + compute_random_jitter(jitter_en, 0);
	} else {
		start_time = m_last_schedule_abs.value() + period_ms + compute_random_jitter(jitter_en);
		if ((start_time + (MSECS_PER_SECOND * SECONDS_PER_DAY)) < now_ms) start_time = now_ms;
	}

	DEBUG_TRACE("LoRaTxScheduler::schedule_periodic: starting @ %llu", start_time);

	// Iterate forward to find a schedule within duty cycle (max 24h search)
	uint64_t elapsed_time = 0;
	while (elapsed_time <= (MSECS_PER_SECOND * SECONDS_PER_DAY)) {
		if (is_in_duty_cycle(start_time, duty_cycle) && start_time >= now_ms) {
			DEBUG_TRACE("LoRaTxScheduler::schedule_periodic: found schedule @ %llu", start_time);
			m_curr_schedule_abs = start_time;
			return;
		} else {
			start_time += period_ms;
			elapsed_time += period_ms;
		}
	}

	DEBUG_ERROR("LoRaTxScheduler::schedule_periodic: no schedule found!");
	m_curr_schedule_abs.reset();
	throw ErrorCode::RESOURCE_NOT_AVAILABLE;
}

unsigned int LoRaTxScheduler::schedule_duty_cycle(ArgosConfig &config, std::time_t now) {
	try {
		schedule_periodic((config.tx_interval_s * MSECS_PER_SECOND), config.argos_tx_jitter_en, config.duty_cycle,
		                  ((uint64_t)now * MSECS_PER_SECOND));
		return m_curr_schedule_abs.value() - (now * MSECS_PER_SECOND);
	} catch (...) {
		return INVALID_SCHEDULE;
	}
}

unsigned int LoRaTxScheduler::schedule_legacy(ArgosConfig &config, std::time_t now) {
	try {
		schedule_periodic((config.tx_interval_s * MSECS_PER_SECOND), config.argos_tx_jitter_en, DUTYCYCLE_24HRS,
		                  (now * MSECS_PER_SECOND));
		return m_curr_schedule_abs.value() - (now * MSECS_PER_SECOND);
	} catch (...) {
		return INVALID_SCHEDULE;
	}
}

void LoRaTxScheduler::set_earliest_schedule(std::time_t earliest) {
	DEBUG_TRACE("LoRaTxScheduler::set_earliest_schedule: t=%llu", earliest);
	m_earliest_schedule = (uint64_t)earliest * MSECS_PER_SECOND;
}

void LoRaTxScheduler::schedule_at(std::time_t t) {
	DEBUG_TRACE("LoRaTxScheduler::schedule_at: t=%llu", t);
	m_curr_schedule_abs = (uint64_t)t * MSECS_PER_SECOND;
}

void LoRaTxScheduler::notify_tx_complete() {
	m_last_schedule_abs = m_curr_schedule_abs;
}
