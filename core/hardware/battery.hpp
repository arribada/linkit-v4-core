/**
 * @file battery.hpp
 * @brief Abstract battery monitor — voltage, level, low/critical detection with hysteresis.
 */

#pragma once

#include <cstdint>
#include "events.hpp"
#include "pmu.hpp"

struct BatteryMonitorEventVoltageCritical {};

class BatteryMonitorEventListener {
public:
	virtual ~BatteryMonitorEventListener() {}
	virtual void react(BatteryMonitorEventVoltageCritical const &) {};
};

/// @brief Abstract battery monitor with voltage/level tracking and critical-voltage events.
class BatteryMonitor : public EventEmitter<BatteryMonitorEventListener> {
protected:
	uint16_t m_last_voltage_mv;
	uint8_t m_last_level;
	uint8_t m_critical_level;
	uint8_t m_low_level;
	bool m_is_low_level;
	bool m_is_critical_voltage;

private:
	bool m_is_critical_voltage_last;
	bool m_has_sample = false;      ///< At least one sample taken since boot
	bool m_sample_due = false;      ///< Next update() must sample whatever the age (thresholds changed)
	uint64_t m_last_sample_ms = 0;  ///< PMU::get_timestamp_ms() of the last sample

	void actuate_events() {
		// Hysteresis: fire critical event on falling edge only,
		// require recovery above critical+hysteresis to re-arm
		if (m_is_critical_voltage && !m_is_critical_voltage_last) {
			notify<BatteryMonitorEventVoltageCritical>({});
			m_is_critical_voltage_last = true;
		} else if (m_is_critical_voltage_last && !m_is_critical_voltage) {
			// Only clear the flag when voltage recovers above hysteresis band
			// Subclasses set m_is_critical_voltage with hysteresis already applied
			m_is_critical_voltage_last = false;
		}
	}
	virtual void internal_update() {}

protected:
	/// @brief Set by internal_update() when it kept nothing from this attempt
	/// (implausible reading, gauge not answering). The last good values stand,
	/// and the next routine update() retries after REJECTED_RETRY_MS instead of
	/// SAMPLE_MAX_AGE_MS -- otherwise a glitch on the first sample after boot
	/// would leave 0 mV in every packet for half an hour.
	bool m_sample_rejected = false;
	/// Retry period after a rejected attempt: soon enough for a glitch, not a
	/// busy-wait every minute on a broken ADC.
	static constexpr uint64_t REJECTED_RETRY_MS = 5ULL * 60ULL * 1000ULL;

	/// @brief Take a sample now, whatever the age of the last one.
	void sample() {
		m_sample_rejected = false;
		internal_update();
		m_last_sample_ms = PMU::get_timestamp_ms();
		m_has_sample = true;
		m_sample_due = false;
		actuate_events();
	}

public:
	/// @brief Age under which a routine update() reuses the last sample.
	///
	/// get_argos_configuration() and get_gnss_configuration() refresh the
	/// battery on every call, and the services call them at each step of their
	/// scheduling. Without this, one turtle surfacing took 44 samples (13 before
	/// its first frame) and a day 4023 -- each a BAT_ADC_SETTLE_MS busy-wait
	/// before the conversion, 500 ms on KIM, 100 ms on SMD and LoRa: 33 minutes a
	/// day of the main loop spinning on KIM. A cell does not move in minutes.
	///
	/// 30 minutes is CriticalBandConfirm::MIN_SPACING_MS: every sample taken is
	/// one the critical confirmation counts, and the low-band debounce (three
	/// consecutive samples) spans an hour, as it was meant to, instead of the
	/// one or two seconds three back-to-back samples around a TX burst take.
	static constexpr uint64_t SAMPLE_MAX_AGE_MS = 30ULL * 60ULL * 1000ULL;

	BatteryMonitor(uint8_t low_level, uint8_t critical_level)
	    : m_last_voltage_mv(0),
	      m_last_level(0),
	      m_critical_level(critical_level),
	      m_low_level(low_level),
	      m_is_low_level(false),
	      m_is_critical_voltage(false),
	      m_is_critical_voltage_last(false) {}
	virtual ~BatteryMonitor() {}
	/// @brief Last measured voltage in mV.
	uint16_t get_voltage() { return m_last_voltage_mv; }
	/// @brief Last computed battery level (0-100%).
	uint8_t get_level() { return m_last_level; }
	/// @brief True if level is below low_level threshold.
	/// @brief Re-apply the configured SOC thresholds.
	///
	/// They used to be set ONLY by the constructor, from init_battery() at boot,
	/// and nothing in the tree ever wrote them again: LB_THRESHOLD (LBP02) and
	/// LB_CRITICAL_THRESH (LBP12) written over DTE therefore had no effect until
	/// the next reboot -- and a sealed tag cannot be rebooted on demand. Worse,
	/// ConfigurationStore::check_battery_thresholds() reads the STORED pair, so
	/// it could report a healthy ordering while the monitor was still enforcing
	/// the old one. Both boards are affected: NrfBatteryMonitor and the STC3117
	/// gas gauge share this base and both compare against these two members.
	///
	/// The flags are computed when a sample is taken, so a CHANGED pair makes the
	/// next update() sample at once instead of applying it up to
	/// SAMPLE_MAX_AGE_MS later. The same pair, pushed again on every refresh,
	/// costs nothing.
	void set_thresholds(uint8_t low_level, uint8_t critical_level) {
		if (low_level != m_low_level || critical_level != m_critical_level) m_sample_due = true;
		m_low_level = low_level;
		m_critical_level = critical_level;
	}

	bool is_battery_low() { return m_is_low_level; }
	/// @brief True if voltage is below critical threshold (with hysteresis).
	bool is_battery_critical() { return m_is_critical_voltage; }
	/// @brief Sample ADC/gauge and fire events if thresholds crossed -- unless the
	/// last sample is younger than max_age_ms, which is then reused as is.
	/// @param max_age_ms  Oldest sample the caller accepts. The default suits
	///        every routine refresh; an operator reading the battery over the
	///        DTE asks for a younger one.
	void update(uint64_t max_age_ms = SAMPLE_MAX_AGE_MS) {
		if (m_has_sample && !m_sample_due) {
			const uint64_t now = PMU::get_timestamp_ms();
			const uint64_t limit_ms =
			    (m_sample_rejected && REJECTED_RETRY_MS < max_age_ms) ? REJECTED_RETRY_MS : max_age_ms;
			if (now >= m_last_sample_ms && now - m_last_sample_ms < limit_ms) {
				// No sample, but still the edge check: it costs nothing, and the
				// critical event must not depend on which call took the sample.
				actuate_events();
				return;
			}
		}
		sample();
	}
	/// @brief Force a fresh sample (bypass any cache) and return whether the
	/// underlying device actually responded. Default: no separate device —
	/// samples now and always reports success. Overridden by gauges (e.g.
	/// STC3117) so callers like the SENSR DTE command get a real pass/fail.
	virtual bool update_forced() {
		sample();
		return true;
	}
	virtual int shutdown() { return 0; }  // Optional shutdown for fuel gauges
};
