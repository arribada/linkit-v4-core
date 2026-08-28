/**
 * @file battery.hpp
 * @brief Abstract battery monitor — voltage, level, low/critical detection with hysteresis.
 */

#pragma once

#include <cstdint>
#include "events.hpp"

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

public:
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
	void set_thresholds(uint8_t low_level, uint8_t critical_level) {
		m_low_level = low_level;
		m_critical_level = critical_level;
	}

	bool is_battery_low() { return m_is_low_level; }
	/// @brief True if voltage is below critical threshold (with hysteresis).
	bool is_battery_critical() { return m_is_critical_voltage; }
	/// @brief Sample ADC/gauge and fire events if thresholds crossed.
	void update() {
		internal_update();
		actuate_events();
	}
	/// @brief Force a fresh sample (bypass any cache) and return whether the
	/// underlying device actually responded. Default: no separate device —
	/// behaves like update() and always reports success. Overridden by gauges
	/// (e.g. STC3117) so callers like the SENSR DTE command get a real pass/fail.
	virtual bool update_forced() {
		update();
		return true;
	}
	virtual int shutdown() { return 0; }  // Optional shutdown for fuel gauges
};
