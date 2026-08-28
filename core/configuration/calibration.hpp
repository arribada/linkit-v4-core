/**
 * @file calibration.hpp
 * @brief Sensor calibration persistence — per-offset double values stored in LittleFS.
 */

#pragma once

#include <string>
#include <map>
#include "filesystem.hpp"
#include "debug.hpp"

class Calibratable;

/// @brief Global registry for all Calibratable instances.
class CalibratableManager {
private:
	static inline std::map<std::string, Calibratable &> m_map;

public:
	/// @brief Register a calibratable instance by name.
	static void add(Calibratable &s, const char *name);

	/// @brief Unregister a calibratable instance.
	static void remove(Calibratable &s);

	/// @brief Find a calibratable by name.
	/// @throws std::out_of_range if not found.
	static Calibratable &find_by_name(const char *name);

	/// @brief Save all registered calibratables to flash.
	static void save_all(bool force = false);

	/// @brief Clear the registry (used during shutdown/reset).
	static void clear();
};

/// @brief Base class for objects that support calibration read/write/save.
class Calibratable {
public:
	Calibratable(const char *name = "Calibratable") { CalibratableManager::add(*this, name); }
	virtual ~Calibratable() { CalibratableManager::remove(*this); }
	virtual void calibration_write(const double, const unsigned int) {};
	virtual void calibration_read(double &value, const unsigned int) { value = 0.0; };
	virtual void calibration_save(bool) {};
};

/// @brief Persistent calibration store — maps offset→double, serialized to a .CAL file.
class Calibration {
public:
	Calibration(const char *name);
	~Calibration();

	/// @brief Read calibration value at offset.
	/// @throws std::out_of_range if offset not found.
	double read(unsigned int offset);

	/// @brief Write calibration value at offset (marks dirty).
	void write(unsigned int offset, double value);

	/// @brief Clear all calibration data (marks dirty).
	void reset();

	/// @brief Save to flash if changed (or if force=true).
	void save(bool force = false);

private:
	std::map<unsigned int, double> m_map;
	std::string m_filename;
	bool m_has_changed;

	void deserialize();
	void serialize();
};
