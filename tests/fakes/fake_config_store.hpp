#pragma once

#include <algorithm>
#include "battery.hpp"
#include "config_store.hpp"

extern BatteryMonitor *battery_monitor;

class FakeConfigurationStore : public ConfigurationStore {
protected:
	/// A save that fails as a worn or full flash does: LittleFS throws its error code.
	void serialize_config() {
		if (m_fail_saves) throw -5;  // LFS_ERR_IO
		m_saved_count++;
	}
	bool m_fail_saves = false;
	void serialize_zone() {}
	/// Asks the monitor for a refresh exactly like LFSConfigurationStore, so a test
	/// counting samples (FakeBatteryMonitor::update_calls) sees every getter that
	/// takes one. This body was empty, and that is how get_argos_configuration()
	/// sampled the battery 4023 times a simulated day with no test noticing. The
	/// store's own copies (level, voltage, low flag) stay where tests set them:
	/// copying the monitor's flags in makes ten tests take the low-battery branch
	/// they were never written for (see tools/check_battery_guard_callers.py).
	void update_battery_level() {
		if (battery_monitor) battery_monitor->update();
	}
	unsigned int m_saved_count;

public:
	FakeConfigurationStore() {
		m_saved_count = 0;
		m_is_battery_level_low = false;
	}
	BasePassPredict m_pass_predict;
	void init() { std::copy(std::begin(default_params), std::end(default_params), m_params.begin()); }
	bool is_valid() { return true; }
	bool is_zone_valid() { return true; }
	void factory_reset() {}
	BasePassPredict &read_pass_predict() { return m_pass_predict; }
	void write_pass_predict(BasePassPredict &value) { m_pass_predict = value; }
	unsigned int get_saved_count() { return m_saved_count; }
	void set_fail_saves(bool fail) { m_fail_saves = fail; }
	void set_battery_level(unsigned int level) { m_battery_level = level; }
	void set_is_battery_level_low(bool state) { m_is_battery_level_low = state; }

	// Test helper — returns the raw BaseType variant for a given ParamID so
	// callers can dispatch on variant.index() (e.g. config-file loaders that
	// need to coerce textual values into the correct variant slot).
	BaseType get_param_variant(ParamID p) { return m_params.at((unsigned int)p); }
};
