/**
 * @file fake_battery_mon.cpp
 * @brief Stub battery monitor — always reports 4.1 V / 100%.
 */

#include "fake_battery_mon.hpp"
#include "debug.hpp"

static constexpr uint16_t FAKE_BATTERY_MV = 4100;   ///< Fixed voltage: 4.1 V
static constexpr uint8_t FAKE_BATTERY_LEVEL = 100;  ///< Fixed level: 100%

FakeBatteryMonitor::FakeBatteryMonitor(uint8_t critical_level, uint8_t low_level)
    : BatteryMonitor(low_level, critical_level) {
	DEBUG_INFO("FakeBatteryMonitor: Initialized (always returns %u mV)", FAKE_BATTERY_MV);

	m_last_voltage_mv = FAKE_BATTERY_MV;
	m_last_level = FAKE_BATTERY_LEVEL;
	m_is_low_level = false;
	m_is_critical_voltage = false;
}

void FakeBatteryMonitor::internal_update() {
	m_last_voltage_mv = FAKE_BATTERY_MV;
	m_last_level = FAKE_BATTERY_LEVEL;
	m_is_low_level = false;
	m_is_critical_voltage = false;
}
