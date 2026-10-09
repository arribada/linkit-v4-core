#pragma once

#include "battery.hpp"

class FakeBatteryMonitor : public BatteryMonitor {
public:
	FakeBatteryMonitor() : BatteryMonitor(10, 5) {}
	/// Number of battery samples taken (BatteryMonitor::update() calls). Lets a
	/// test prove that nothing touched the ADC on a latency-critical path.
	unsigned int update_calls = 0;
	void internal_update() override { update_calls++; }
	void set_values(uint8_t level = 100, uint16_t mv = 4200, bool is_low = false, bool is_critical = false) {
		m_last_voltage_mv = mv;
		m_last_level = level;
		m_is_critical_voltage = is_critical;
		m_is_low_level = is_low;
	}
};
