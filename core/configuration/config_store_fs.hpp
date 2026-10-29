/**
 * @file config_store_fs.hpp
 * @brief LittleFS-backed configuration store — serialization/deserialization of all parameters.
 */

#pragma once

#include <cctype>
#include <cstring>
#include <algorithm>
#include <vector>
#include "config_store.hpp"
#include "filesystem.hpp"
#include "debug.hpp"
#include "battery.hpp"
#include "dte_params.hpp"
#include "calibration.hpp"


extern BatteryMonitor *battery_monitor;

/// @brief Flash power held for one scope: power_down() runs on every way out, an
/// exception included. A power_up() whose power_down() an exception skipped left
/// the IS25's reference count up for good, its "busy" interlock raised, and VSYS
/// at 3.3 V in every idle until the next reset (2026-07 audit, robust/storage).
/// Scopes nest: the IS25 counts references.
class FlashPowerScope {
public:
	explicit FlashPowerScope(FileSystem &fs) : m_fs(fs) { m_fs.power_up(); }
	~FlashPowerScope() { m_fs.power_down(); }
	FlashPowerScope(const FlashPowerScope &) = delete;
	FlashPowerScope &operator=(const FlashPowerScope &) = delete;

private:
	FileSystem &m_fs;
};

class LFSConfigurationStore : public ConfigurationStore {
protected:
	bool m_is_pass_predict_valid;
	bool m_is_config_valid;
	BasePassPredict m_pass_predict;

	// Serialization format is as follows:
	//
	// <KEY><BINARY_DATA_ZERO_PADDED_TO_128_BYTES>
	//
	// one record per implemented slot, in table order. <KEY> (fixed length
	// of 5, e.g. "IDT01") is what places a record when it is read back:
	// deserialize_config() puts it in the slot that carries that key, and
	// skips a record whose key no slot of this build carries.

	/// @brief Serialize a single config entry (key + zero-padded binary data) to file.
	bool serialize_config_entry(LFSFile &f, unsigned int index) {
		if (index >= MAX_CONFIG_ITEMS) return false;
		const BaseMap *entry = &param_map[index];
		struct Serializer {
			uint8_t entry_buffer[BASE_TEXT_MAX_LENGTH];
			void operator()(std::string &s) {
				std::memset(entry_buffer, 0, sizeof(entry_buffer));
				std::memcpy(entry_buffer, s.data(), s.size());
			};
			void operator()(unsigned int &s) {
				std::memset(entry_buffer, 0, sizeof(entry_buffer));
				std::memcpy(entry_buffer, &s, sizeof(s));
			};
			void operator()(int &s) {
				std::memset(entry_buffer, 0, sizeof(entry_buffer));
				std::memcpy(entry_buffer, &s, sizeof(s));
			};
			void operator()(double &s) {
				std::memset(entry_buffer, 0, sizeof(entry_buffer));
				std::memcpy(entry_buffer, &s, sizeof(s));
			};
			void operator()(std::time_t &s) {
				std::memset(entry_buffer, 0, sizeof(entry_buffer));
				std::memcpy(entry_buffer, &s, sizeof(s));
			};
			void operator()(BaseArgosMode &s) {
				std::memset(entry_buffer, 0, sizeof(entry_buffer));
				std::memcpy(entry_buffer, &s, sizeof(s));
			};
			void operator()(BaseArgosPower &s) {
				std::memset(entry_buffer, 0, sizeof(entry_buffer));
				std::memcpy(entry_buffer, &s, sizeof(s));
			};
			void operator()(BaseDepthPile &s) {
				std::memset(entry_buffer, 0, sizeof(entry_buffer));
				std::memcpy(entry_buffer, &s, sizeof(s));
			};
			void operator()(bool &s) {
				std::memset(entry_buffer, 0, sizeof(entry_buffer));
				std::memcpy(entry_buffer, &s, sizeof(s));
			};
			void operator()(BaseGNSSFixMode &s) {
				std::memset(entry_buffer, 0, sizeof(entry_buffer));
				std::memcpy(entry_buffer, &s, sizeof(s));
			};
			void operator()(BaseGNSSDynModel &s) {
				std::memset(entry_buffer, 0, sizeof(entry_buffer));
				std::memcpy(entry_buffer, &s, sizeof(s));
			};
			void operator()(BaseLEDMode &s) {
				std::memset(entry_buffer, 0, sizeof(entry_buffer));
				std::memcpy(entry_buffer, &s, sizeof(s));
			};
			void operator()(BaseZoneType &s) {
				std::memset(entry_buffer, 0, sizeof(entry_buffer));
				std::memcpy(entry_buffer, &s, sizeof(s));
			};
			void operator()(BaseArgosModulation &s) {
				std::memset(entry_buffer, 0, sizeof(entry_buffer));
				std::memcpy(entry_buffer, &s, sizeof(s));
			};
			void operator()(BaseUnderwaterDetectSource &s) {
				std::memset(entry_buffer, 0, sizeof(entry_buffer));
				std::memcpy(entry_buffer, &s, sizeof(s));
			};
			void operator()(BaseDebugMode &s) {
				std::memset(entry_buffer, 0, sizeof(entry_buffer));
				std::memcpy(entry_buffer, &s, sizeof(s));
			};
			void operator()(BasePressureSensorLoggingMode &s) {
				std::memset(entry_buffer, 0, sizeof(entry_buffer));
				std::memcpy(entry_buffer, &s, sizeof(s));
			};
			void operator()(BasePressureSensorFullScale &s) {
				std::memset(entry_buffer, 0, sizeof(entry_buffer));
				std::memcpy(entry_buffer, &s, sizeof(s));
			};
			void operator()(BaseSensorEnableTxMode &s) {
				std::memset(entry_buffer, 0, sizeof(entry_buffer));
				std::memcpy(entry_buffer, &s, sizeof(s));
			};
			void operator()(BaseRawData &) {};
		} s;
		std::visit(s, m_params.at(index));

		return f.write((void *)entry->key.data(), entry->key.size()) == (lfs_ssize_t)entry->key.size()
		       && f.write(s.entry_buffer, sizeof(s.entry_buffer)) == sizeof(s.entry_buffer);
	}

	/// @brief Deserialize a single config entry from file and validate key match.
	bool deserialize_config_entry(LFSFile &f, unsigned int index) {
		if (index >= MAX_CONFIG_ITEMS) return false;
		const BaseMap *entry = &param_map[index];
		uint8_t entry_buffer[KEY_LENGTH + BASE_TEXT_MAX_LENGTH];

		// Read entry from file
		if (note_io(f.read(entry_buffer, KEY_LENGTH + BASE_TEXT_MAX_LENGTH)) != KEY_LENGTH + BASE_TEXT_MAX_LENGTH)
			return false;

		// Check param key matches
		if (std::string((const char *)entry_buffer, KEY_LENGTH) != entry->key) return false;

		uint8_t *param_value = &entry_buffer[KEY_LENGTH];

		switch (entry->encoding) {
		case BaseEncoding::DECIMAL: {
			int value;
			std::memcpy(&value, param_value, sizeof(value));
			m_params.at(index) = value;
			break;
		}
		case BaseEncoding::AQPERIOD:
		case BaseEncoding::HEXADECIMAL:
		case BaseEncoding::UINT: {
			unsigned int value;
			std::memcpy(&value, param_value, sizeof(value));
			m_params.at(index) = value;
			break;
		}
		case BaseEncoding::TEXT: {
			std::string value((const char *)param_value, strnlen((const char *)param_value, BASE_TEXT_MAX_LENGTH));
			m_params.at(index) = value;
			break;
		}
		case BaseEncoding::DATESTRING: {
			std::time_t value;
			std::memcpy(&value, param_value, sizeof(value));
			m_params.at(index) = value;
			break;
		}
		case BaseEncoding::BOOLEAN: {
			bool value;
			std::memcpy(&value, param_value, sizeof(value));
			m_params.at(index) = value;
			break;
		}
		case BaseEncoding::ARGOSFREQ:
		case BaseEncoding::FLOAT: {
			double value;
			std::memcpy(&value, param_value, sizeof(value));
			m_params.at(index) = value;
			break;
		}
		case BaseEncoding::DEPTHPILE: {
			BaseDepthPile value;
			std::memcpy(&value, param_value, sizeof(value));
			m_params.at(index) = value;
			break;
		}
		case BaseEncoding::ARGOSMODE: {
			BaseArgosMode value;
			std::memcpy(&value, param_value, sizeof(value));
			m_params.at(index) = value;
			break;
		}
		case BaseEncoding::ARGOSPOWER: {
			BaseArgosPower value;
			std::memcpy(&value, param_value, sizeof(value));
			m_params.at(index) = value;
			break;
		}
		case BaseEncoding::UWDETECTSOURCE: {
			BaseUnderwaterDetectSource value;
			std::memcpy(&value, param_value, sizeof(value));
			m_params.at(index) = value;
			break;
		}
		case BaseEncoding::GNSSFIXMODE: {
			BaseGNSSFixMode value;
			std::memcpy(&value, param_value, sizeof(value));
			m_params.at(index) = value;
			break;
		}
		case BaseEncoding::GNSSDYNMODEL: {
			BaseGNSSDynModel value;
			std::memcpy(&value, param_value, sizeof(value));
			m_params.at(index) = value;
			break;
		}
		case BaseEncoding::LEDMODE: {
			BaseLEDMode value;
			std::memcpy(&value, param_value, sizeof(value));
			m_params.at(index) = value;
			break;
		}
		case BaseEncoding::ZONETYPE: {
			BaseZoneType value;
			std::memcpy(&value, param_value, sizeof(value));
			m_params.at(index) = value;
			break;
		}
		case BaseEncoding::MODULATION: {
			BaseArgosModulation value;
			std::memcpy(&value, param_value, sizeof(value));
			m_params.at(index) = value;
			break;
		}
		case BaseEncoding::DEBUGMODE: {
			BaseDebugMode value;
			std::memcpy(&value, param_value, sizeof(value));
			m_params.at(index) = value;
			break;
		}
		case BaseEncoding::PRESSURESENSORLOGGINGMODE: {
			BasePressureSensorLoggingMode value;
			std::memcpy(&value, param_value, sizeof(value));
			m_params.at(index) = value;
			break;
		}
		case BaseEncoding::PRESSURESENSORFULLSCALE: {
			BasePressureSensorFullScale value;
			std::memcpy(&value, param_value, sizeof(value));
			m_params.at(index) = value;
			break;
		}
		case BaseEncoding::SENSORENABLETXMODE: {
			BaseSensorEnableTxMode value;
			std::memcpy(&value, param_value, sizeof(value));
			m_params.at(index) = value;
			break;
		}
		case BaseEncoding::KEY_LIST:
		case BaseEncoding::KEY_VALUE_LIST:
		case BaseEncoding::BASE64:
		default: return false;
		}

		return true;
	}

	/// @brief The slot whose key a record carries, or -1 when this build's table has none.
	static int slot_of_key(const char *key) {
		for (unsigned int i = 0; i < MAX_CONFIG_ITEMS; i++) {
			const BaseKey &k = param_map[i].key;
			if (k.size() == KEY_LENGTH && std::memcmp(k.data(), key, KEY_LENGTH) == 0) return (int)i;
		}
		return -1;
	}

	/// @brief Version mismatch: recover the identifiers and every protected field,
	/// each by its KEY. Everything else keeps its default.
	/// @param first_record  offset of the first record, just past the version field
	void recover_protected_params(LFSFile &f, lfs_soff_t first_record) {
		// Recover every protected field, identified by its KEY and not by its
		// position in the file.
		//
		// This used to read POSITIONALLY: two calls straight after the version
		// field, which land on records 0 and 1. That recovered exactly
		// ARGOS_DECID and ARGOS_HEXID, because those two happen to sit first in
		// param_map, and silently lost the thirteen entries of
		// PROTECTED_PARAMS, which are scattered. Those thirteen are ALL the
		// credentials -- the Argos secret key, the four RADIOCONF and the eight
		// LoRaWAN keys -- and config_store.hpp:159-195 already states what
		// losing them costs: "would leave every provisioned LoRa unit unable to
		// rejoin its network, with nothing in the logs to say why". The
		// protection was declared and simply never wired to the path that needs
		// it.
		//
		// Reading by key also makes the recovery correct for the very situation
		// a version mismatch implies: records may have been added, removed or
		// reordered, so no position can be trusted. deserialize_config_entry()
		// consumes one record and accepts it only when its key matches the index
		// asked for, so rewinding and walking forward finds a param wherever it
		// now sits. A few hundred fixed-size records per param, on a path that
		// runs once per firmware version change.
		const ParamID recover[] = {
			ParamID::ARGOS_DECID,
			ParamID::ARGOS_HEXID,
		};
		unsigned int recovered = 0, lost = 0;

		auto recover_by_key = [&](ParamID id) {
			if (note_io(f.seek(first_record)) != first_record) return false;
			for (unsigned int record = 0; record < MAX_CONFIG_ITEMS; record++) {
				if (deserialize_config_entry(f, (unsigned int)id)) return true;
			}
			return false;
		};

		for (ParamID id : recover) {
			if (recover_by_key(id))
				recovered++;
			else {
				lost++;
				DEBUG_WARN("deserialize_config: failed to recover %s", param_map[(unsigned int)id].name.c_str());
			}
		}
		for (ParamID id : PROTECTED_PARAMS) {
			if (!param_map[(unsigned int)id].is_implemented) continue;
			if (recover_by_key(id))
				recovered++;
			else {
				lost++;
				DEBUG_WARN("deserialize_config: failed to recover %s", param_map[(unsigned int)id].name.c_str());
			}
		}
		// Console only, despite the general rule that a DEBUG_WARN also lands
		// in the flash system log. This runs from configuration_store->init()
		// at main.cpp:956 and DebugLogger::system_log is not attached until
		// main.cpp:1143, 185 lines later -- so on a sealed tag, the one event
		// you would most want a record of leaves no trace a recovered device
		// can be asked about. Moving it would need the counts carried out to
		// main; not done here.
		DEBUG_WARN("deserialize_config: version mismatch — %u protected param(s) recovered, %u lost", recovered, lost);

		m_requires_serialization = true;
	}

	/// @brief Read every record of a file in this build's version into the slot
	/// that carries its key. @p placed says which slots received one.
	/// @param first_record  offset of the first record, just past the version field
	void read_records_by_key(LFSFile &f, lfs_soff_t first_record, bool (&placed)[MAX_CONFIG_ITEMS]) {
		// Each record goes to the slot that carries its KEY, wherever it sits.
		//
		// The file holds one record per slot the WRITING build implements, in
		// table order, and builds differ: the camera slots exist only with
		// ENABLE_CAM_SENSOR, SECKEY only on SMD, PWP05/LBP14 only on RSPB since
		// f4ba6c16 (on every board before). This loop used to take one record per
		// slot and to drop the slot when the key did not match -- consuming the
		// record anyway, so one record more or less shifted every later one: they
		// all came back at their defaults, and init() saved that for good. That is
		// how a camera build reflashed without the camera lost 71 parameters from
		// slot 172 on, the master RCONF and LAST_KNOWN_RTC among them (bench,
		// 2026-10-27), and how every v4.1.9 -> v4.1.10 upgrade lost 17.
		//
		// A file in this build's layout -- every boot but the first after a
		// change -- costs what it always did: one read and one key comparison per
		// record. A record that is not the expected one costs a key lookup and
		// two more reads, and the file is then saved in this build's layout.
		constexpr lfs_soff_t RECORD = KEY_LENGTH + BASE_TEXT_MAX_LENGTH;
		unsigned int skipped = 0, moved = 0;
		unsigned int expected = 0;  // the slot of the next record, when the file is this build's
		const lfs_soff_t file_size = note_io(f.size());
		for (lfs_soff_t at = first_record; at + RECORD <= file_size; at += RECORD) {
			while (expected < MAX_CONFIG_ITEMS && !param_map[expected].is_implemented)
				expected++;
			if (expected < MAX_CONFIG_ITEMS && deserialize_config_entry(f, expected)) {
				placed[expected++] = true;
				continue;
			}
			// Not the expected record, or past the last slot: whose is it?
			char key[KEY_LENGTH];
			if (note_io(f.seek(at)) != at || note_io(f.read(key, KEY_LENGTH)) != KEY_LENGTH) break;
			const int slot = slot_of_key(key);
			if (slot < 0 || !param_map[slot].is_implemented || placed[slot]) {
				skipped++;  // a slot this build does not have, or a second record of one
			} else {
				if (note_io(f.seek(at)) == at && deserialize_config_entry(f, (unsigned int)slot)) placed[slot] = true;
				if ((unsigned int)slot != expected) moved++;
				// Forward only: every placed slot stays below `expected`, so the
				// fast path never lands on one already placed.
				if ((unsigned int)slot >= expected) expected = (unsigned int)slot + 1;
			}
			if (note_io(f.seek(at + RECORD)) != at + RECORD) break;
		}
		if (skipped || moved) {
			DEBUG_WARN("deserialize_config: %u record(s) of slots this build lacks skipped, %u placed by key", skipped,
			           moved);
			m_requires_serialization = true;
		}
	}

	/// @brief Read full configuration from config.dat, recover protected fields on version mismatch.
	/// @brief Read config.dat into m_params. Throws the LittleFS error of an open or
	/// of any read that failed: a file the flash did not deliver is not a file to
	/// rebuild from the defaults.
	void deserialize_config() {
		DEBUG_TRACE("ConfigurationStoreLFS::deserialize_config");
		m_read_error = 0;
		fault_inject_read();
		LFSFile f(&m_filesystem, "config.dat", LFS_O_RDONLY);

		unsigned int config_version_code = 0;
		const lfs_soff_t first_record = (lfs_soff_t)sizeof(config_version_code);

		// Read configuration version field
		if (note_io(f.read(&config_version_code, sizeof(config_version_code))) != sizeof(config_version_code)
		    || config_version_code != m_config_version_code) {
			raise_read_error();
			DEBUG_WARN(
			    "deserialize_config: configuration version mismatch; try to recover protected & reset all others");
			recover_protected_params(f, first_record);
			raise_read_error();
			return;
		}

		bool placed[MAX_CONFIG_ITEMS] = {};
		read_records_by_key(f, first_record, placed);
		raise_read_error();

		for (unsigned int i = 0; i < MAX_CONFIG_ITEMS; i++) {
			// Skip non-implemented/disabled parameters (reserved slots, disabled sensors)
			if (!param_map[i].is_implemented) continue;

			if (!placed[i]) {
				DEBUG_WARN("deserialize_config: unable to deserialize param %s - resetting...",
				           param_map[i].name.c_str());
				// Reset parameter to factory default
				m_params.at(i) = default_params[i];
				m_requires_serialization = true;
				continue;
			}

			// Check variant index (type) matches default parameters
			if (m_params.at(i).index() != default_params[i].index()) {
				DEBUG_WARN("deserialize_config: param %s variant index mismatch expected %u but got %u - resetting...",
				           param_map[i].name.c_str(), default_params[i].index(), m_params.at(i).index());
				// Reset parameter to factory default
				m_params.at(i) = default_params[i];
				m_requires_serialization = true;
				continue;
			}
		}

		m_is_config_valid = true;
	}

	/// @brief Write full configuration to config.dat (versioned, all implemented params).
	void serialize_config() override {
		DEBUG_TRACE("ConfigurationStoreLFS::serialize_config");
		FlashPowerScope power(m_filesystem);
		LFSFile f(&m_filesystem, "config.dat", LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);

		// Write configuration version field
		if (f.write((void *)&m_config_version_code, sizeof(m_config_version_code)) != sizeof(m_config_version_code)) {
			throw CONFIG_STORE_CORRUPTED;
		}

		for (unsigned int i = 0; i < MAX_CONFIG_ITEMS; i++) {
			// Skip non-implemented/disabled parameters (reserved slots, disabled sensors)
			if (!param_map[i].is_implemented) continue;

			// Check variant index (type) matches default parameter
			if (m_params.at(i).index() != default_params[i].index()) {
				DEBUG_WARN("serialize_config: param %s variant index mismatch expected %u but got %u - resetting...",
				           param_map[i].name.c_str(), default_params[i].index(), m_params.at(i).index());
				// Reset parameter to back to factory default
				m_params.at(i) = default_params[i];
			}

			if (!serialize_config_entry(f, i)) {
				DEBUG_ERROR("serialize_config: failed to serialize param %u", i);
				throw CONFIG_STORE_CORRUPTED;
			}
		}

		m_is_config_valid = true;

		DEBUG_TRACE("ConfigurationStoreLFS::serialize_config: saved new file config.data");
	}

	/// @brief Read Argos pass prediction data from pass_predict.dat.
	///
	/// A file that is missing, of another version, short, corrupt or with more
	/// records than the table holds is replaced by the built-in set, on flash. A
	/// read that FAILS (LFS_ERR_IO, which may be transient) keeps the built-in set
	/// in RAM only: rewriting the file there destroyed an uploaded AOP that the
	/// next boot could have read (2026-07 audit, robust/storage). The record count
	/// bound matters on its own: PREVIPASS walks the records up to num_records, in
	/// a table of MAX_AOP_SATELLITE_ENTRIES.
	void deserialize_prepass() {
		DEBUG_TRACE("ConfigurationStoreLFS::deserialize_prepass");
		int error = 0;
		lfs_ssize_t got = 0;
		try {
			LFSFile f(&m_filesystem, "pass_predict.dat", LFS_O_RDONLY);
			got = f.read(&m_pass_predict, sizeof(m_pass_predict));
			if (got < 0) error = (int)got;
		} catch (int e) {
			error = e;
		}
		if (error == LFS_ERR_IO) {
			DEBUG_WARN("deserialize_prepass: pass_predict.dat unreadable - built-in AOP set in RAM, file kept");
			m_pass_predict = default_prepass;
			m_is_pass_predict_valid = true;
			return;
		}
		if (error == 0 && got == (lfs_ssize_t)sizeof(m_pass_predict)
		    && m_pass_predict.version_code == m_config_version_code_aop
		    && m_pass_predict.num_records <= MAX_AOP_SATELLITE_ENTRIES) {
			m_is_pass_predict_valid = true;
			return;
		}
		DEBUG_WARN("deserialize_prepass: pass_predict.dat missing or unusable (%d) - built-in AOP set written", error);
		create_default_prepass();
	}

	/// @brief Write pass prediction data to pass_predict.dat.
	void serialize_pass_predict() {
		DEBUG_TRACE("ConfigurationStoreLFS::serialize_pass_predict");
		LFSFile f(&m_filesystem, "pass_predict.dat", LFS_O_CREAT | LFS_O_WRONLY | LFS_O_TRUNC);
		m_pass_predict.version_code = m_config_version_code_aop;
		m_is_pass_predict_valid = false;
		m_is_pass_predict_valid = f.write(&m_pass_predict, sizeof(m_pass_predict)) == sizeof(m_pass_predict);
		if (!m_is_pass_predict_valid) {
			DEBUG_ERROR("serialize_pass_predict: failed to serialize pass predict");
			throw CONFIG_STORE_CORRUPTED;
		}
	}

	/// @brief Reset pass prediction to factory default AOP data.
	void create_default_prepass() {
		DEBUG_TRACE("ConfigurationStoreLFS::create_default_prepass");
		write_pass_predict((BasePassPredict &)default_prepass);
	}

	/// @brief Refresh cached battery level/voltage from BatteryMonitor.
	void update_battery_level() override {
		// Push the configured thresholds BEFORE sampling: the monitor takes them
		// through its constructor only, so without this a DTE change to LBP02 or
		// LBP12 would not take effect until the next reboot. This is the natural
		// place -- it already runs on every refresh and it is the one call site
		// that holds both the config store and the monitor.
		try {
			battery_monitor->set_thresholds(
			    static_cast<uint8_t>(read_param<unsigned int>(ParamID::LB_THRESHOLD)),
			    static_cast<uint8_t>(read_param<unsigned int>(ParamID::LB_CRITICAL_THRESH)));
		} catch (...) {
			// Store unreadable: keep whatever the monitor already had rather than
			// letting a battery refresh throw into its callers.
		}
		battery_monitor->update();
		m_battery_level = battery_monitor->get_level();
		m_battery_voltage = battery_monitor->get_voltage();
		m_is_battery_level_low = battery_monitor->is_battery_low();
	}

private:
	FileSystem &m_filesystem;
	bool m_requires_serialization;
	/// @brief First LittleFS error a read of config.dat met, 0 if none: see note_io().
	int m_read_error = 0;
	/// @brief Set by main.cpp once the storage failure budget is spent (C2).
	bool m_last_resort_allowed = false;
	/// @brief What the last resort did with the unreadable file, null if it did not
	/// run: logged by main.cpp once the system log is attached, after init().
	const char *m_last_resort_outcome = nullptr;

	static constexpr unsigned int CONFIG_READ_RETRIES = 3;
	static constexpr unsigned int CONFIG_READ_RETRY_DELAY_MS = 50;  // as main.cpp's mount retries
	static constexpr const char *RADIO_IDS_PENDING = "radio_ids.todo";

	/// @brief Keep the first LittleFS error of a read: a short read is a missing
	/// record (an old or truncated file), a negative one a flash that did not answer.
	lfs_ssize_t note_io(lfs_ssize_t r) {
		if (r < 0 && !m_read_error) m_read_error = (int)r;
		return r;
	}

	void raise_read_error() const {
		if (m_read_error) throw m_read_error;
	}

	/// @brief FAULT_INJECT=6 (validation firmware): config.dat reads as unreadable.
	void fault_inject_read() const {
#if defined(FAULT_INJECT) && (FAULT_INJECT == 6)
		if (fault_inject_unreadable) throw (int)LFS_ERR_IO;
#endif
	}

	/// @brief Read config.dat, again on an error: a passing IS25 fault clears in
	/// milliseconds. @return 0, LFS_ERR_NOENT when there is no file, or the error
	/// that outlived the retries.
	int read_config() {
		int error = 0;
		for (unsigned int attempt = 0; attempt <= CONFIG_READ_RETRIES; attempt++) {
			if (attempt) PMU::delay_ms(CONFIG_READ_RETRY_DELAY_MS);
			std::copy(std::begin(default_params), std::end(default_params), m_params.begin());
			m_requires_serialization = false;
			try {
				deserialize_config();
				return 0;
			} catch (int e) {
				error = e;
			}
			if (error == LFS_ERR_NOENT) return error;
			DEBUG_WARN("config.dat: read failed (%d), attempt %u of %u", error, attempt + 1, CONFIG_READ_RETRIES + 1);
		}
		return error;
	}

	/// @brief The last resort, once the storage budget is spent and config.dat is
	/// still unreadable (user decision 2026-10-29): keep the file for an analysis
	/// if there is room, start again from defaults that still communicate, and let
	/// the boot take the radio identity back from the module.
	void start_over(int error) {
		DEBUG_ERROR("config.dat unreadable (%d) after every retry: starting over", error);
		quarantine_unreadable_config();
		std::copy(std::begin(default_params), std::end(default_params), m_params.begin());
		apply_survival_profile();
		mark_radio_credentials_pending();
		m_requires_serialization = true;
	}

	/// @brief config.dat becomes config.bad if the file system keeps a quarter of
	/// its blocks free with it; otherwise it is removed. Only the latest is kept.
	void quarantine_unreadable_config() {
		lfs_t *lfs = static_cast<lfs_t *>(m_filesystem.get_private_data());
		m_filesystem.remove("config.bad");
		// lfs_fs_size() walks every file: a bad block inside config.dat makes it
		// fail too, and a file whose cost cannot be measured is not kept.
		const lfs_ssize_t used = lfs_fs_size(lfs);
		const lfs_size_t blocks = lfs->cfg->block_count;
		const bool room = used >= 0 && (blocks - (lfs_size_t)used) * 4 >= blocks;
		if (room && lfs_rename(lfs, "config.dat", "config.bad") == 0) {
			m_last_resort_outcome = "kept as config.bad";
			return;
		}
		m_filesystem.remove("config.dat");
		m_last_resort_outcome = used < 0 ? "removed (free space unreadable)"
		                        : room   ? "removed (could not be renamed)"
		                                 : "removed (no room to keep it)";
	}

	/// @brief Factory defaults, minus what would drain a tag nobody can reconfigure:
	/// one Argos TX every 20 min instead of every minute, one GNSS fix an hour
	/// instead of every 10 min, and the low-battery mode on.
	void apply_survival_profile() {
		m_params.at((unsigned int)ParamID::TR_NOM) = 1200U;
		m_params.at((unsigned int)ParamID::DLOC_ARG_NOM) = 60U * 60U;
		m_params.at((unsigned int)ParamID::LB_EN) = true;
	}

	void mark_radio_credentials_pending() {
		try {
			LFSFile f(&m_filesystem, RADIO_IDS_PENDING, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);
		} catch (int e) {
			DEBUG_ERROR("%s not created (%d): the module identity will not be taken back", RADIO_IDS_PENDING, e);
		}
	}

	/// @brief What the boot-fail recovery keeps: the configuration it has just
	/// rewritten, the calibrations it has just saved, and the last resort's records.
	static bool kept_through_recovery(const std::string &name) {
		static const std::string cal = ".CAL";
		return name == "config.dat" || name == "config.bad" || name == RADIO_IDS_PENDING
		       || (name.size() > cal.size() && name.compare(name.size() - cal.size(), cal.size(), cal) == 0);
	}

	/// @brief Every other file at the root goes, one by one: logs, the AOP, the GNSS
	/// database, the OTA images. One that refuses does not stop the rest.
	void remove_state_files() {
		lfs_t *lfs = static_cast<lfs_t *>(m_filesystem.get_private_data());
		std::vector<std::string> doomed;  // never remove while the directory is open
		lfs_dir_t dir;
		if (lfs_dir_open(lfs, &dir, "/") < 0) return;
		struct lfs_info info;
		while (lfs_dir_read(lfs, &dir, &info) > 0)
			if (info.type == LFS_TYPE_REG && !kept_through_recovery(info.name)) doomed.emplace_back(info.name);
		lfs_dir_close(lfs, &dir);
		for (const std::string &name : doomed)
			m_filesystem.remove(name.c_str());
	}

	static bool is_hex(const std::string &s) {
		return !s.empty() && std::all_of(s.begin(), s.end(), [](char c) { return std::isxdigit((unsigned char)c); });
	}

	static bool is_hex_of(const std::string &s, size_t length) { return s.size() == length && is_hex(s); }

	/// @brief OTAA needs DEVEUI, APPEUI and APPKEY; ABP its DEVADDR.
	static bool lora_identity_usable(const std::string &deveui, const std::string &appeui, const std::string &appkey,
	                                 const std::string &devaddr, unsigned int njm) {
		if (njm == 1) return is_hex_of(deveui, 16) && is_hex_of(appeui, 16) && is_hex_of(appkey, 32);
		return is_hex_of(devaddr, 8);
	}

	/// @brief Parameters a factory_reset must NOT destroy.
	///
	/// The identifiers, and — added after the 2026-08 safety-net audit — the
	/// provisioning secrets. factory_reset is a last-ditch recovery from a
	/// corrupt config store, reached automatically after three boots that never
	/// make it to Operational, with no operator involved. It used to preserve
	/// DECID and HEXID only, so ARGOS_SECKEY and the RADIOCONFs came back as
	/// std::string(""), and a beacon that cannot build a valid frame is not a
	/// beacon. On a potted tag that is unrecoverable: the very path meant to
	/// save the device was the one guaranteed to end it.
	///
	/// Keeping them is the safer side of the trade. If the corruption really is
	/// in a credential, preserving it leaves a beacon that MIGHT transmit
	/// instead of one that certainly cannot, and the variant-index repair below
	/// still fixes a wrong type. Everything else — modes, periods, thresholds —
	/// still returns to defaults, which is what the recovery is for.
	/// The guards mirror base_types.hpp: these enumerators only exist on the
	/// builds that own the corresponding radio.
	static constexpr ParamID PROTECTED_PARAMS[] = {
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
		ParamID::ARGOS_SECKEY,
#endif
		ParamID::ARGOS_RADIOCONF,
		ParamID::ARGOS_RADIOCONF_LDK,
		ParamID::ARGOS_RADIOCONF_LDA2,
		ParamID::ARGOS_RADIOCONF_VLDA4,
#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
		ParamID::LORA_DEVEUI,
		ParamID::LORA_APPEUI,
		ParamID::LORA_APPKEY,
		ParamID::LORA_DEVADDR,
		ParamID::LORA_APPSKEY,
		ParamID::LORA_NWKSKEY,
		// Join mode and band are provisioning, not mission settings: a tag
		// deployed in ABP, or on a band other than the build default, would keep
		// its keys through the recovery and still be unable to reach its network.
		ParamID::LORA_NJM,
		ParamID::LORA_BAND,
#endif
	};

	/// @brief True for the identifiers and the provisioning secrets a factory_reset keeps.
	static bool is_protected_param(unsigned int index) {
		if (index <= (unsigned int)ParamID::ARGOS_HEXID) return true;
		for (ParamID id : PROTECTED_PARAMS)
			if ((unsigned int)id == index) return true;
		return false;
	}

	/// @brief What the boot-fail recovery keeps on top of PROTECTED_PARAMS: the
	/// battery and duty-cycle safety. At their defaults a recovered RSPB had no
	/// shutdown timer and no session TX budget, so nothing ended a wake, woke on
	/// modulo 2 whatever it was set to, and had no low-battery mode (H6, July
	/// audit). The session TX budgets are implemented on RSPB only and never reach
	/// the file elsewhere; the TPL5111 ones exist on EXTERNAL_WAKEUP builds only.
	/// FACTR still resets them.
	static constexpr ParamID SAFETY_PARAMS[] = {
		ParamID::LB_EN,
		ParamID::LB_THRESHOLD,
		ParamID::LB_CRITICAL_THRESH,
		ParamID::LB_SHUTDOWN_NTIME_SAT,
		ParamID::SHUTDOWN_NTIME_SAT,
#ifdef EXTERNAL_WAKEUP
		ParamID::SHUTDOWN_TIMER,
		ParamID::BOOT_COUNTER_MODULO,
		ParamID::WAKEUP_PERIOD,
		ParamID::WAKEUP_PERIOD_MEASURED_MS,
#endif
	};

	static bool is_safety_param(unsigned int index) {
		for (ParamID id : SAFETY_PARAMS)
			if ((unsigned int)id == index) return true;
		return false;
	}

	/// @brief Write config.dat for factory_reset: every parameter back to its default
	/// except the identifiers and the provisioning secrets.
	///
	/// The file holds every implemented parameter, in order, exactly like a normal
	/// save. It used to be written sparse (identifiers, then the secrets) while the
	/// reader took records by position: each secret landed in the wrong slot, was
	/// rejected, and came back empty on the next boot, so a tag recovered by the
	/// boot-fail path could no longer join or build a frame. The reader now places
	/// records by key; a full file still reads on its fast path, and still reads on
	/// a firmware older than that after a downgrade.
	/// Power is left to the caller (factory_reset brackets it). @p keep_safety:
	/// SAFETY_PARAMS are kept too (the boot-fail recovery).
	void serialize_protected_config(bool keep_safety) {
		for (unsigned int i = 0; i < MAX_CONFIG_ITEMS; i++) {
			const bool kept = is_protected_param(i) || (keep_safety && is_safety_param(i));
			if (!kept || m_params.at(i).index() != default_params[i].index()) m_params.at(i) = default_params[i];
		}

		LFSFile f(&m_filesystem, "config.dat", LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);
		if (f.write((void *)&m_config_version_code, sizeof(m_config_version_code)) != sizeof(m_config_version_code))
			throw CONFIG_STORE_CORRUPTED;
		for (unsigned int i = 0; i < MAX_CONFIG_ITEMS; i++) {
			if (!param_map[i].is_implemented) continue;
			if (!serialize_config_entry(f, i)) {
				DEBUG_ERROR("serialize_protected_config: failed to serialize param %u", i);
				throw CONFIG_STORE_CORRUPTED;
			}
		}

		DEBUG_INFO("ConfigurationStoreLFS::serialize_protected_config: identifiers and provisioning secrets kept "
		           "across the factory reset");
		if (keep_safety) DEBUG_INFO("ConfigurationStoreLFS: battery and duty-cycle safety parameters kept as well");
	}

	/// @brief Persist all sensor calibration data to flash.
	void save_calibration_data() { CalibratableManager::save_all(true); }

public:
	LFSConfigurationStore(FileSystem &filesystem)
	    : m_is_pass_predict_valid(false),
	      m_is_config_valid(false),
	      m_filesystem(filesystem) {}

	/// @brief Init: deserialize config + prepass from flash, create defaults if missing.
	/// @throws CONFIG_STORE_CORRUPTED on unrecoverable flash error, and the LittleFS
	/// error of a config.dat that could not be read: the file is left as it is, for
	/// the next boot to read again (main.cpp: storage_unusable()).
	void init() override {
		m_requires_serialization = false;
		m_is_pass_predict_valid = false;
		m_is_config_valid = false;
		const bool last_resort = m_last_resort_allowed;
		m_last_resort_allowed = false;  // one shot: main.cpp decides it for its own call

		// Keep flash powered during entire init sequence to avoid per-I/O power cycling
		FlashPowerScope power(m_filesystem);

		// Only an ABSENT file starts from the defaults. Every LittleFS error used to
		// read as "no file", and a passing read error on the IS25 rewrote the
		// credentials no downlink can restore (C1, robust/storage).
		const int error = read_config();
		if (error == LFS_ERR_NOENT) {
			DEBUG_WARN("No configuration file so creating a default file");
			m_requires_serialization = true;
		} else if (error) {
			if (!last_resort) throw error;
			start_over(error);
		}

		if (m_requires_serialization) serialize_config();

		if (!m_is_config_valid) {
			throw CONFIG_STORE_CORRUPTED;  // This is a non-recoverable error
		}

		// Read in prepass file
		try {
			deserialize_prepass();
		} catch (int e) {
			DEBUG_WARN("AOP file does not exist or is corrupted - resetting AOP file");
			create_default_prepass();
		}
	}

	bool is_valid() override { return m_is_config_valid; }

	/// @brief Factory reset: format flash, preserve DECID/HEXID + calibration data.
	void factory_reset() override { format_and_restore(false); }

	/// @brief The boot-fail recovery's factory reset: SAFETY_PARAMS kept as well, and
	/// no format (C5, user decision 2026-10-29). It runs on a tag already failing to
	/// boot, often for want of power, so where a cut is most likely, and a cut between
	/// the format and the rewrite left an empty file system: the identity was gone for
	/// good. LittleFS commits a rewritten file on close, so a cut now leaves the old
	/// config.dat or the new one, whole. What must start again from nothing is the
	/// state, not the file system: the mount went through, or this boot would not be
	/// here. FACTR, with an operator present, keeps the format.
	void factory_reset_for_recovery() override {
		FlashPowerScope power(m_filesystem);
		serialize_protected_config(true);
		save_calibration_data();
		remove_state_files();
		m_is_config_valid = false;
		m_is_pass_predict_valid = false;
	}

	/// @brief main.cpp, before init(): the storage failure budget is spent, so an
	/// unreadable config.dat may be started over (see start_over()).
	void allow_last_resort(bool allowed) { m_last_resort_allowed = allowed; }
#if defined(FAULT_INJECT) && (FAULT_INJECT == 6)
	/// @brief VALIDATION FIRMWARE ONLY: config.dat reads as unreadable (LFS_ERR_IO).
	bool fault_inject_unreadable = false;
#endif
	const char *last_resort_outcome() const override { return m_last_resort_outcome; }

	bool radio_credentials_pending() override {
		FlashPowerScope power(m_filesystem);
		struct lfs_info info;
		return m_filesystem.stat(RADIO_IDS_PENDING, &info) == 0;
	}

	void radio_credentials_done() override {
		FlashPowerScope power(m_filesystem);
		m_filesystem.remove(RADIO_IDS_PENDING);
	}

	bool adopt_argos_credentials(unsigned int dec_id, unsigned int address, const std::string &seckey,
	                             const std::string &radioconf) override {
		if (!is_hex(radioconf) || radioconf.size() % 2) return false;
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
		if (seckey.size() != 32 || !is_hex(seckey)) return false;
		write_param(ParamID::ARGOS_SECKEY, seckey);
#else
		(void)seckey;
#endif
		write_param(ParamID::ARGOS_DECID, dec_id);
		write_param(ParamID::ARGOS_HEXID, address);
		write_param(ParamID::ARGOS_RADIOCONF, radioconf);
		save_params();
		radio_credentials_done();
		return true;
	}

#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
	/// The session keys the module keeps are not overwritten while the configuration
	/// has none (lora_rak3172.cpp skips the empty ones).
	bool adopt_lora_credentials(const std::string &deveui, const std::string &appeui, const std::string &appkey,
	                            const std::string &devaddr, unsigned int njm) override {
		if (!lora_identity_usable(deveui, appeui, appkey, devaddr, njm)) return false;
		write_param(ParamID::LORA_NJM, njm);
		if (is_hex(deveui)) write_param(ParamID::LORA_DEVEUI, deveui);
		if (is_hex(appeui)) write_param(ParamID::LORA_APPEUI, appeui);
		if (is_hex(appkey)) write_param(ParamID::LORA_APPKEY, appkey);
		if (is_hex(devaddr)) write_param(ParamID::LORA_DEVADDR, devaddr);
		save_params();
		radio_credentials_done();
		return true;
	}
#endif

private:
	void format_and_restore(bool keep_safety) {
		FlashPowerScope power(m_filesystem);
		m_filesystem.umount();
		m_filesystem.format();
		m_filesystem.mount();
		serialize_protected_config(keep_safety);  // Recover "protected" parameters
		save_calibration_data();
		m_is_config_valid = false;
		m_is_pass_predict_valid = false;
	}

public:
	BasePassPredict &read_pass_predict() override {
		if (m_is_pass_predict_valid) {
			return m_pass_predict;
		}
		throw CONFIG_DOES_NOT_EXIST;
	}

	void write_pass_predict(BasePassPredict &value) override {
		FlashPowerScope power(m_filesystem);
		m_pass_predict = value;
		serialize_pass_predict();
	}
};
