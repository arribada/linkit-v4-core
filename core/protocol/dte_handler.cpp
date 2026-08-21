/**
 * @file dte_handler.cpp
 * @brief DTE command handler — dispatches PARMR/PARMW/DUMPD/STAT/etc to system components.
 */

#include "dte_handler.hpp"
#include "ledsm.hpp"
#include "gps_service.hpp"
#include "argos_tx_service.hpp"
#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
#include "lora_tx_service.hpp"
#endif
#include "scheduler.hpp"
#include "pmu.hpp"
#include "rgb_led.hpp"
#include <cmath>

extern RGBLed *status_led;

extern Scheduler *system_scheduler;

DTEHandler::DTEHandler() {
	m_dumpd_NNN = 0;
	m_dumpd_mmm = 0;
	m_dumpd_d_type = 0xFFFFFFFF;  // Invalid initial value
	m_sat_device_active = false;
	m_lora_tx_active = false;
	m_doppler_cal_active = false;
	m_doppler_cal_first_tx = false;
	m_sattx_needs_restore = false;
	m_sattx_restore_modulation = KineisModulation::LDA2;
	m_gnssi_pending = false;
	m_gps_subscribed = false;
}

DTEHandler::~DTEHandler() {}

void DTEHandler::set_async_write(std::function<void(const std::string&)> fn) { m_async_write = fn; }

void DTEHandler::reset_state() {
	m_dumpd_NNN = 0;
	m_dumpd_mmm = 0;
	m_dumpd_d_type = 0xFFFFFFFF;
}

// Fix #37: Shared helper for PARMR_REQ and STATR_REQ
std::string DTEHandler::read_params_by_filter(int error_code, std::vector<ParamID>& params, char filter_char, DTECommand resp_cmd) {

	if (error_code) {
		return DTEEncoder::encode(resp_cmd, error_code);
	}

	// Check special case where params is zero length => retrieve all matching key types
	if (params.size() == 0) {
		for (unsigned int i = 0; i < param_map_size; i++) {
			if (param_map[i].is_implemented &&
				param_map[i].key[2] == filter_char)
				params.push_back((ParamID)i);
		}
	}

	std::vector<ParamValue> param_values;
	for (unsigned int i = 0; i < params.size(); i++) {
		try {
			BaseType x = configuration_store->read_param<BaseType>(params[i]);
			param_values.push_back({params[i], x});
		} catch (...) {
			DEBUG_WARN("DTEHandler::read_params_by_filter: failed to read param %u (key=%s)",
			           static_cast<unsigned>(params[i]), param_map[static_cast<unsigned>(params[i])].key.c_str());
		}
	}

	return DTEEncoder::encode(resp_cmd, param_values);
}

#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
// Fix #38: Shared helper for SMD null-check pattern
std::string DTEHandler::smd_not_available_error(const char* func_name, DTECommand resp_cmd) {
	DEBUG_ERROR("DTEHandler::%s: SMD satellite instance not available", func_name);
	return DTEEncoder::encode(resp_cmd, (int)DTEError::INCORRECT_DATA);
}
#endif

std::string DTEHandler::PARML_REQ(int error_code) {

	if (error_code) {
		return DTEEncoder::encode(DTECommand::PARML_RESP, error_code);
	}

	// Build up a list of all implemented parameters
	std::vector<ParamID> params;
	for (unsigned int i = 0; i < param_map_size; i++) {
		if (param_map[i].is_implemented) {
			params.push_back(static_cast<ParamID>(i));
		}
	}

	return DTEEncoder::encode(DTECommand::PARML_RESP, params);
}

std::string DTEHandler::PARMW_REQ(int error_code, std::vector<ParamValue>& param_values, std::vector<std::string>& rejected_keys, DTEAction& action) {

	if (error_code) {
		return DTEEncoder::encode(DTECommand::PARMW_RESP, error_code);
	}

#if !defined(ARGOS_SMD) || (ARGOS_SMD != 1)
	bool rconf_edited = false;  // KIM2 only: re-seed modulation cache if a RCONF param changes
#endif

	for (unsigned int i = 0; i < param_values.size(); i++) {
		if (param_map[(int)param_values[i].param].is_writable) {
			// SMP00 (SMD_DEGRADED_MODE): writable via DTE for manual clear
			// after a confirmed root-cause fix, but only value 0 is accepted.
			// Value 1 (engage SAFE) must remain under the autofallback's
			// exclusive control — the device is in the best position to
			// detect cascade failures and engage SAFE timings. Allowing
			// PARMW SMP00=1 would let an operator manually trigger the
			// "degraded" state, bypassing the success-count / trust-window
			// re-test machinery that auto-clears it.
			if (param_values[i].param == ParamID::SMD_DEGRADED_MODE) {
				try {
					unsigned int v = std::get<unsigned int>(param_values[i].value);
					if (v != 0) {
						DEBUG_WARN("DTEHandler::PARMW_REQ: SMP00 only accepts value 0 (manual clear) — rejected");
						rejected_keys.push_back(param_map[(int)param_values[i].param].key);
						continue;
					}
				} catch (const std::bad_variant_access&) {
					// Type mismatch — let configuration_store handle / reject
				}
			}
			configuration_store->write_param(param_values[i].param, param_values[i].value);
#if !defined(ARGOS_SMD) || (ARGOS_SMD != 1)
			if (param_values[i].param == ParamID::ARGOS_RADIOCONF ||
			    param_values[i].param == ParamID::ARGOS_RADIOCONF_LDK ||
			    param_values[i].param == ParamID::ARGOS_RADIOCONF_LDA2 ||
			    param_values[i].param == ParamID::ARGOS_RADIOCONF_VLDA4) {
				rconf_edited = true;
			}
#endif
		} else {
			DEBUG_WARN("DTEHandler::PARMW_REQ: not writing read-only attribute %s", param_map[(int)param_values[i].param].name.c_str());
			rejected_keys.push_back(param_map[(int)param_values[i].param].key);
		}
	}

#if !defined(ARGOS_SMD) || (ARGOS_SMD != 1)
	// KIM2 ONLY (not compiled for SMD): a RCONF edit changes the radio's
	// modulation. Our firmware can't decode the raw (encrypted) ARGOS_RADIOCONF
	// hex by itself — only the KIM module can — so we program the new RCONF to
	// the module and read it back (AT+RCONF=?, which returns the decoded
	// freq/power/modulation) to learn the modulation, then re-seed the cache.
	// The FIRST TX after the edit is then sized for the right modulation.
	// MUST run BEFORE save_params() so the re-seeded ARGOS_CACHED_MODULATION is
	// flushed to flash and survives the next reboot.
	if (rconf_edited && kineis_device_instance) {
		kineis_device_instance->resync_rconf_cache();
	}
#endif

	// Save all the parameters (incl. the re-seeded modulation cache above)
	configuration_store->save_params();

	// Notify configuration updated action
	action = DTEAction::CONFIG_UPDATED;

	// If some params were rejected, return partial success with rejected key list
	// Format: $N;PARMW#LEN;key1,key2,...\r  (valid params were still written)
	if (!rejected_keys.empty()) {
		std::string payload;
		for (unsigned int i = 0; i < rejected_keys.size(); i++) {
			if (i > 0) payload += ",";
			payload += rejected_keys[i];
		}
		char header[16];
		snprintf(header, sizeof(header), "$N;PARMW#%03X;", (unsigned int)payload.size());
		return std::string(header) + payload + "\r";
	}

	return DTEEncoder::encode(DTECommand::PARMW_RESP, DTEError::OK);
}

std::string DTEHandler::PARMR_REQ(int error_code, std::vector<ParamID>& params) {
	return read_params_by_filter(error_code, params, 'P', DTECommand::PARMR_RESP);
}

std::string DTEHandler::STATR_REQ(int error_code, std::vector<ParamID>& params) {
	// Refresh live RTC value before reading status params
	if (rtc) {
		configuration_store->write_param(ParamID::RTC_CURRENT_TIME, static_cast<unsigned int>(rtc->gettime()));
	}
	return read_params_by_filter(error_code, params, 'T', DTECommand::STATR_RESP);
}

std::string DTEHandler::PROFW_REQ(int error_code, std::vector<BaseType>& arg_list) {

	if (!error_code) {
		if (arg_list.size() < 1)
			return DTEEncoder::encode(DTECommand::PROFW_RESP, (int)DTEError::MISSING_ARGUMENT);
		configuration_store->write_param(ParamID::PROFILE_NAME, arg_list[0]);
	}

	return DTEEncoder::encode(DTECommand::PROFW_RESP, error_code);
}

std::string DTEHandler::PROFR_REQ(int error_code) {

	if (error_code) {
		return DTEEncoder::encode(DTECommand::PROFR_RESP, error_code);
	}

	return DTEEncoder::encode(DTECommand::PROFR_RESP, error_code, configuration_store->read_param<std::string>(ParamID::PROFILE_NAME));
}

std::string DTEHandler::SECUR_REQ(int error_code, std::vector<BaseType>& arg_list) {
	// Default access code - can be changed or made configurable
	// Using ARGOS ID as access code for device-specific security
	static constexpr unsigned int SECUR_ACCESS_CODE = 0x12345678;

	if (error_code) {
		return DTEEncoder::encode(DTECommand::SECUR_RESP, error_code);
	}

	// Validate access code
	if (arg_list.empty()) {
		return DTEEncoder::encode(DTECommand::SECUR_RESP, (int)DTEError::MISSING_ARGUMENT);
	}

	unsigned int provided_code = std::get<unsigned int>(arg_list[0]);

	// Read device ARGOS ID to use as dynamic access code
	unsigned int device_argos_id = configuration_store->read_param<unsigned int>(ParamID::ARGOS_DECID);

	// Accept either the static code or the device's ARGOS ID as valid access codes
	if (provided_code != SECUR_ACCESS_CODE && provided_code != device_argos_id) {
		DEBUG_WARN("DTEHandler::SECUR_REQ: Invalid access code provided (0x%08X)", provided_code);
		return DTEEncoder::encode(DTECommand::SECUR_RESP, (int)DTEError::INVALID_ACCESS_CODE);
	}

	DEBUG_INFO("DTEHandler::SECUR_REQ: Access granted");
	return DTEEncoder::encode(DTECommand::SECUR_RESP, (int)DTEError::OK);
}

std::string DTEHandler::RSTVW_REQ(int error_code, std::vector<BaseType>& arg_list) {

	if (error_code) {
		return DTEEncoder::encode(DTECommand::RSTVW_RESP, error_code);
	}

	if (arg_list.size() < 1)
		return DTEEncoder::encode(DTECommand::RSTVW_RESP, (int)DTEError::MISSING_ARGUMENT);

	unsigned int variable_id = std::get<unsigned int>(arg_list[0]);
	unsigned int zero = 0;

	if (variable_id == 1) {
		// TX_COUNTER
		configuration_store->write_param(ParamID::TX_COUNTER, zero);
	} else if (variable_id == 2) {
		// BOOT_COUNTER
#ifdef EXTERNAL_WAKEUP
		configuration_store->write_param(ParamID::BOOT_COUNTER, zero);
#else
		error_code = (int)DTEError::INCORRECT_DATA;
#endif
	} else if (variable_id == 3) {
		// RX_COUNTER
		configuration_store->write_param(ParamID::ARGOS_RX_COUNTER, zero);
	} else if (variable_id == 4) {
		// RX_TIME
		configuration_store->write_param(ParamID::ARGOS_RX_TIME, zero);
	} else {
		// Invalid variable ID
		error_code = (int)DTEError::INCORRECT_DATA;
	}

	if (!error_code) {
		configuration_store->save_params();
	}

	return DTEEncoder::encode(DTECommand::RSTVW_RESP, error_code);
}

std::string DTEHandler::RSTBW_REQ(int error_code) {

	return DTEEncoder::encode(DTECommand::RSTBW_RESP, error_code);

}

std::string DTEHandler::FACTW_REQ(int error_code) {

	return DTEEncoder::encode(DTECommand::FACTW_RESP, error_code);

}

std::string DTEHandler::DUMPM_REQ(int error_code, std::vector<BaseType>& arg_list) {

	if (error_code) {
		return DTEEncoder::encode(DTECommand::DUMPM_RESP, error_code);
	}

	if (!memory_access) {
		return DTEEncoder::encode(DTECommand::DUMPM_RESP, (int)DTEError::INCORRECT_DATA);
	}

	if (arg_list.size() < 2)
		return DTEEncoder::encode(DTECommand::DUMPM_RESP, (int)DTEError::MISSING_ARGUMENT);

	unsigned int address = std::get<unsigned int>(arg_list[0]);
	unsigned int length = std::get<unsigned int>(arg_list[1]);
	BaseRawData raw = {
		.ptr = memory_access->get_physical_address(address, length),
		.length = length,
		.str = ""
	};

	return DTEEncoder::encode(DTECommand::DUMPM_RESP, error_code, raw);
}

std::string DTEHandler::PASPW_REQ(int error_code, std::vector<BaseType>& arg_list) {

	while (!error_code) {
		BasePassPredict pass_predict;
		std::string paspw_bits = std::get<std::string>(arg_list[0]);
		try {
			PassPredictCodec::decode(paspw_bits, pass_predict);
		} catch (ErrorCode e) {
			error_code = (int)DTEError::INCORRECT_DATA;
			break;  // Do not write configuration store
		}

		// If the number of records is zero then return an incorrect data error and flag this
		// back to the user without updating the configuration store
		if (pass_predict.num_records == 0) {
			DEBUG_ERROR("DTEHandler::PASPW_REQ: no AOP records | not updating the config store");
			error_code = (int)DTEError::INCORRECT_DATA;
			break;
		}

		// Scan through all the records and find the entry whose bulletin date
		// is the most recent, skipping any non-operational satellites
		std::time_t argos_aop_date = 0;
		for (unsigned int i = 0; i < pass_predict.num_records; i++)
		{
			if (pass_predict.records[i].bulletin.year) {
				std::time_t t = convert_epochtime(pass_predict.records[i].bulletin.year, pass_predict.records[i].bulletin.month, pass_predict.records[i].bulletin.day, pass_predict.records[i].bulletin.hour, pass_predict.records[i].bulletin.minute, pass_predict.records[i].bulletin.second);
				if (t > argos_aop_date)
					argos_aop_date = t;
			}
		}

		if (argos_aop_date)
		{
			// Update configuration store
			configuration_store->write_pass_predict(pass_predict);

			// Set to most recent AOP bulletin record
			configuration_store->write_param(ParamID::ARGOS_AOP_DATE, argos_aop_date);

			// Save configuration to commit AOPDATE
			configuration_store->save_params();

			// Log that the PASPW has been updated
			struct tm tm_buf;
			auto time = gmtime_r(&argos_aop_date, &tm_buf);
			char buff[256];
			if (time)
				std::strftime(buff, sizeof(buff), "%d/%m/%Y %H:%M:%S", time);
			else
				std::snprintf(buff, sizeof(buff), "(gmtime failed)");
			DEBUG_INFO("DTEHandler:PASPW_REQ: saving PASPW with #AOPs=%u ARGOS_AOP_DATE=%s", (unsigned int)pass_predict.num_records, buff);
			break;
		} else {
			DEBUG_ERROR("DTEHandler::PASPW_REQ: no valid AOP records | not updating the config store");
			error_code = (int)DTEError::INCORRECT_DATA;
			break;
		}
	}

	return DTEEncoder::encode(DTECommand::PASPW_RESP, error_code);
}

std::string DTEHandler::DUMPD_REQ(int error_code, std::vector<BaseType>& arg_list, DTEAction& action) {

	action = DTEAction::NONE;

	// A bit of explanation here.  The protocol for DUMPD sends back a payload format of
	// mmm,MMM,<payload>
	// where mmm is a packet index and MMM is the maximum packet index whose value
	// is NNN-1, where NNN is the total number of packets to send.
	if (error_code) {
		// Reset state variables back to zero if an error arises
		m_dumpd_NNN = 0;
		m_dumpd_mmm = 0;
		m_dumpd_d_type = 0xFFFFFFFF;
		return DTEEncoder::encode(DTECommand::DUMPD_RESP, error_code);
	}

	Logger *logger = nullptr;

	// Extract the d_type parameter from arg_list to determine which log file to use
	unsigned int d_type = std::get<unsigned int>(arg_list[0]);

	// Reset state if dump type changed mid-stream (prevents state leak)
	if (d_type != m_dumpd_d_type) {
		m_dumpd_NNN = 0;
		m_dumpd_mmm = 0;
		m_dumpd_d_type = d_type;
	}

	try {
		logger = LoggerManager::find_by_name(m_logger_dump.at(d_type).c_str());
	} catch (...) {
		// Ignore any exceptions -- the logger will be nullptr
	}

	// Either invalid log file or the logger doesn't exist
	if (logger == nullptr) {
		error_code = (int)DTEError::INCORRECT_DATA;
		return DTEEncoder::encode(DTECommand::DUMPD_RESP, error_code);
	}

	// Get the log formatter
	LogFormatter *formatter;
	formatter = logger->get_log_formatter();

	// Check to see if this is the first item
	unsigned int total_entries = logger->num_entries();
	if (0 == m_dumpd_NNN) {
		m_dumpd_NNN = (total_entries + (DTE_HANDLER_MAX_LOG_DUMP_ENTRIES-1)) / DTE_HANDLER_MAX_LOG_DUMP_ENTRIES;
		// Special case where log file is empty we set NNN to 1 and will send an empty payload
		m_dumpd_NNN = m_dumpd_NNN == 0 ? 1 : m_dumpd_NNN;
		m_dumpd_mmm = 0;
	}

	LogEntry log_entry;
	BaseRawData raw_data;
	unsigned int start_index = m_dumpd_mmm * DTE_HANDLER_MAX_LOG_DUMP_ENTRIES;
	unsigned int num_entries = (start_index < total_entries) ? std::min(total_entries - start_index, DTE_HANDLER_MAX_LOG_DUMP_ENTRIES) : 0;
	raw_data.ptr = nullptr;
	raw_data.length = 0;

	// Set CSV header line if this is the first packet output
	if (0 == m_dumpd_mmm)
		raw_data.str.append(formatter->header());

	for (unsigned int i = 0; i < num_entries; i++) {
		logger->read(&log_entry, i + start_index);
		// Concatenate formatted log entry
		raw_data.str.append(formatter->log_entry(log_entry));
	}

	// Note that MMM=NNN-1
	std::string msg = DTEEncoder::encode(DTECommand::DUMPD_RESP, error_code, m_dumpd_mmm, m_dumpd_NNN - 1, raw_data);

	m_dumpd_mmm++; // Increment in readiness for next iteration
	if (m_dumpd_mmm == m_dumpd_NNN) {
		// Sequence complete - reset all state
		m_dumpd_NNN = 0;
		m_dumpd_mmm = 0;
		m_dumpd_d_type = 0xFFFFFFFF;
	} else {
		action = DTEAction::AGAIN;  // Inform caller that we need another iteration
	}

	return msg;
}

std::string DTEHandler::ERASE_REQ(int error_code, std::vector<BaseType>& arg_list) {

	if (error_code) {
		return DTEEncoder::encode(DTECommand::ERASE_RESP, error_code);
	}

	DEBUG_TRACE("Processing ERASE");

	// Extract the d_type parameter from arg_list to determine which log file(s) to erase
	unsigned int d_type = std::get<unsigned int>(arg_list[0]);

	if (d_type == (unsigned int)BaseEraseType::ALL) {
		// Truncate all loggers
		LoggerManager::truncate();
	} else {
		Logger *logger = nullptr;

		try {
			logger = LoggerManager::find_by_name(m_logger_erase.at(d_type).c_str());
		} catch (...) {
			// Ignore any exceptions -- the logger will be nullptr
		}

		if (logger)
		{
			DEBUG_TRACE("Truncating log %s", logger->get_name());
			logger->truncate();
		}
		else
		{
			error_code = (int)DTEError::INCORRECT_DATA;
		}
	}
	return DTEEncoder::encode(DTECommand::ERASE_RESP, error_code);
}

std::string DTEHandler::SCALW_REQ(int error_code, std::vector<BaseType>& arg_list) {

	if (error_code) {
		return DTEEncoder::encode(DTECommand::SCALW_RESP, error_code);
	}

	if (arg_list.size() < 3)
		return DTEEncoder::encode(DTECommand::SCALW_RESP, (int)DTEError::MISSING_ARGUMENT);

	// Extract the device_id parameter from arg_list to determine which device to calibrate
	unsigned int device_id = std::get<unsigned int>(arg_list[0]);

	// Extract the calibration offset parameter from arg_list to determine which sensor offset to calibrate
	unsigned int offset = std::get<unsigned int>(arg_list[1]);

	// Extract the calibration value parameter from arg_list to use
	double value = std::get<double>(arg_list[2]);

	try {
		const char *name = m_scalx.at(device_id).c_str();
		DEBUG_TRACE("Calibrating device %s...", name);
		Calibratable& s = CalibratableManager::find_by_name(name);
		s.calibration_write(value, offset);
	} catch (...) {
		DEBUG_TRACE("Device calibration failed");
		error_code = (int)DTEError::INCORRECT_DATA;
	}

	return DTEEncoder::encode(DTECommand::SCALW_RESP, error_code);
}

std::string DTEHandler::SCALR_REQ(int error_code, std::vector<BaseType>& arg_list) {
	if (error_code) {
		return DTEEncoder::encode(DTECommand::SCALR_RESP, error_code);
	}

	if (arg_list.size() < 2)
		return DTEEncoder::encode(DTECommand::SCALR_RESP, (int)DTEError::MISSING_ARGUMENT);

	// Extract the device_id parameter from arg_list to determine which device to calibrate
	unsigned int device_id = std::get<unsigned int>(arg_list[0]);

	// Extract the calibration offset parameter from arg_list to determine which sensor offset to calibrate
	unsigned int offset = std::get<unsigned int>(arg_list[1]);

	double value;

	try {
		const char *name = m_scalx.at(device_id).c_str();
		DEBUG_TRACE("Read device %s calibration setting %u...", name, offset);
		Calibratable& s = CalibratableManager::find_by_name(name);
		s.calibration_read(value, offset);
	} catch (...) {
		DEBUG_TRACE("Device calibration read failed");
		error_code = (int)DTEError::INCORRECT_DATA;
	}

	if (error_code) {
		return DTEEncoder::encode(DTECommand::SCALR_RESP, error_code);
	}

	return DTEEncoder::encode(DTECommand::SCALR_RESP, error_code, value);
}

std::string DTEHandler::SMDDFU_REQ(int error_code, std::vector<BaseType>& arg_list) {
	if (error_code) {
		return DTEEncoder::encode(DTECommand::SMDDFU_RESP, error_code);
	}

	unsigned int action = std::get<unsigned int>(arg_list[0]);
	unsigned int status = 0;
	bool dfu_mode = false;
	unsigned int progress = 0;
	std::string info = "";

	// VERSION action is build-agnostic: returns the firmware/identity string of
	// whichever comm module is active (SMD / KIM2 / LoRa). Other actions (DFU,
	// INFO, STATUS, UPDATE) remain SMD-only.
	if ((SmdDfuAction)action == SmdDfuAction::VERSION) {
		std::string version;
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
		if (!smd_sat_instance) {
			return smd_not_available_error("SMDDFU_REQ", DTECommand::SMDDFU_RESP);
		}
		if (smd_sat_instance->is_dfu_mode()) {
			return DTEEncoder::encode(DTECommand::SMDDFU_RESP, (unsigned int)0,
			                          (unsigned int)1, true, (unsigned int)0,
			                          std::string("SMD in DFU mode - exit first"));
		}
		version = smd_sat_instance->get_firmware_version();
#elif defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
		if (!lora_device_instance) {
			return DTEEncoder::encode(DTECommand::SMDDFU_RESP, (unsigned int)0,
			                          (unsigned int)1, false, (unsigned int)0,
			                          std::string("LoRa module not available"));
		}
		version = lora_device_instance->get_firmware_version();
#else
		if (!kim2_device_instance) {
			return DTEEncoder::encode(DTECommand::SMDDFU_RESP, (unsigned int)0,
			                          (unsigned int)1, false, (unsigned int)0,
			                          std::string("KIM2 module not available"));
		}
		version = kim2_device_instance->get_firmware_version();
#endif
		status = version.empty() ? 1u : 0u;
		info   = version.empty() ? std::string("Failed to read version") : version;
		return DTEEncoder::encode(DTECommand::SMDDFU_RESP, (unsigned int)0,
		                          status, dfu_mode, progress, info);
	}

#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
	if (!smd_sat_instance) {
		return smd_not_available_error("SMDDFU_REQ", DTECommand::SMDDFU_RESP);
	}

	switch ((SmdDfuAction)action) {
	case SmdDfuAction::VERSION:
		// Handled above — should not reach here.
		break;

	case SmdDfuAction::ENTER:
		if (smd_sat_instance->dfu_enter()) {
			status = 0;
			dfu_mode = true;
		} else {
			status = 1;
		}
		break;

	case SmdDfuAction::EXIT:
		if (smd_sat_instance->dfu_exit()) {
			status = 0;
			dfu_mode = false;
		} else {
			status = 1;
		}
		break;

	case SmdDfuAction::STATUS:
		status = 0;
		dfu_mode = smd_sat_instance->is_dfu_mode();
		break;

	case SmdDfuAction::INFO:
		dfu_mode = smd_sat_instance->is_dfu_mode();
		if (dfu_mode) {
			SmdDfuInfo dfu_info;
			if (smd_sat_instance->dfu_get_bootloader_info(&dfu_info)) {
				status = 0;
				info = "BL v" + std::to_string(dfu_info.version_major) + "." +
				       std::to_string(dfu_info.version_minor);
			} else {
				status = 1;
			}
		} else {
			status = 1;
			info = "Not in DFU mode";
		}
		break;

	case SmdDfuAction::UPDATE:
		// Firmware update is handled via OTA file transfer, not this command
		// This action just triggers the update process using previously uploaded firmware
		status = 1;
		info = "Use OTA to upload firmware first";
		break;

	default:
		return DTEEncoder::encode(DTECommand::SMDDFU_RESP, (int)DTEError::INCORRECT_DATA);
	}

	return DTEEncoder::encode(DTECommand::SMDDFU_RESP, (unsigned int)0, status, dfu_mode, progress, info);
#else
	// Non-SMD builds: only VERSION action (handled above) is supported.
	(void)status; (void)dfu_mode; (void)progress; (void)info;
	return DTEEncoder::encode(DTECommand::SMDDFU_RESP, (int)DTEError::INCORRECT_DATA);
#endif
}

/// @brief COMCW_REQ — Continuous Wave RF test for SMD / LoRa modules.
/// Args: mode (0=stop, 1=start), freq_hz, power_dbm, duration_s.
/// Returns status code + info string via COMCW_RESP.
std::string DTEHandler::COMCW_REQ(int error_code, std::vector<BaseType>& arg_list) {
	if (error_code) {
		return DTEEncoder::encode(DTECommand::COMCW_RESP, error_code);
	}
	if (arg_list.empty()) {
		return DTEEncoder::encode(DTECommand::COMCW_RESP, (int)DTEError::MISSING_ARGUMENT);
	}

	unsigned int mode       = std::get<unsigned int>(arg_list[0]);
	unsigned int freq_hz    = (arg_list.size() > 1) ? std::get<unsigned int>(arg_list[1]) : 0U;
	unsigned int power_dbm  = (arg_list.size() > 2) ? std::get<unsigned int>(arg_list[2]) : 0U;
	unsigned int duration_s = (arg_list.size() > 3) ? std::get<unsigned int>(arg_list[3]) : 0U;

	unsigned int status = 0;
	std::string info = "";
	bool ok = false;

#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
	if (!smd_sat_instance) {
		return DTEEncoder::encode(DTECommand::COMCW_RESP, (unsigned int)0,
		                          (unsigned int)1, std::string("SMD not available"));
	}
	if (mode == 0) {
		ok = smd_sat_instance->cw_stop();
		info = ok ? "CW stop OK" : "CW stop failed";
	} else {
		if (arg_list.size() < 3) {
			return DTEEncoder::encode(DTECommand::COMCW_RESP, (int)DTEError::MISSING_ARGUMENT);
		}
		ok = smd_sat_instance->cw_start(freq_hz,
		                                 static_cast<uint16_t>(power_dbm),
		                                 static_cast<uint16_t>(duration_s));
		info = ok ? "CW start OK" : "CW start failed";
	}
#elif defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
	if (!lora_device_instance) {
		return DTEEncoder::encode(DTECommand::COMCW_RESP, (unsigned int)0,
		                          (unsigned int)1, std::string("LoRa not available"));
	}
	if (mode == 0) {
		ok = lora_device_instance->cw_stop();
		info = ok ? "CW stop OK (module reset)" : "CW stop failed";
	} else {
		if (arg_list.size() < 3) {
			return DTEEncoder::encode(DTECommand::COMCW_RESP, (int)DTEError::MISSING_ARGUMENT);
		}
		ok = lora_device_instance->cw_start(freq_hz,
		                                     static_cast<uint16_t>(power_dbm),
		                                     static_cast<uint16_t>(duration_s));
		info = ok ? "CW start OK" : "CW start failed";
	}
#else
	(void)mode; (void)freq_hz; (void)power_dbm; (void)duration_s;
	return DTEEncoder::encode(DTECommand::COMCW_RESP, (unsigned int)0,
	                          (unsigned int)1,
	                          std::string("CW not supported on this module"));
#endif

	status = ok ? 0u : 1u;
	return DTEEncoder::encode(DTECommand::COMCW_RESP, (unsigned int)0, status, info);
}

#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
std::string DTEHandler::SMDTST_REQ(int error_code, std::vector<BaseType>& arg_list) {
	(void)arg_list;
	if (error_code) {
		return DTEEncoder::encode(DTECommand::SMDTST_RESP, error_code);
	}

	if (!smd_sat_instance) {
		return smd_not_available_error("SMDTST_REQ", DTECommand::SMDTST_RESP);
	}

	std::string test_result = smd_sat_instance->smd_spi_test();
	if (test_result.empty()) {
		return DTEEncoder::encode(DTECommand::SMDTST_RESP, (int)DTEError::INCORRECT_DATA);
	}

	return DTEEncoder::encode(DTECommand::SMDTST_RESP, (unsigned int)0, test_result);
}
#endif

std::string DTEHandler::SATVF_REQ(int error_code, std::vector<BaseType>& arg_list) {

	if (error_code) {
		return DTEEncoder::encode(DTECommand::SATVF_RESP, error_code);
	}

	// force: 0 = read-only verify, 1 = re-write from config store when hardware differs
	unsigned int force = 0;
	if (!arg_list.empty()) {
		force = std::get<unsigned int>(arg_list[0]);
	}

	try {
#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
		// LoRa module: read credentials from RAK3172 via AT commands
		if (!lora_device_instance) {
			DEBUG_ERROR("SATVF_REQ: no LoRa device available");
			return DTEEncoder::encode(DTECommand::SATVF_RESP, (int)DTEError::INCORRECT_DATA);
		}

		auto hw = lora_device_instance->read_lora_credentials();
		if (!hw.read_ok) {
			DEBUG_ERROR("SATVF_REQ: failed to read LoRa credentials from module");
			return DTEEncoder::encode(DTECommand::SATVF_RESP, (int)DTEError::INCORRECT_DATA);
		}

		// Lambda: compare hardware snapshot with config store (case-insensitive hex)
		auto to_upper = [](std::string s) { for (auto& c : s) c = toupper(c); return s; };
		auto compare = [&](const LoRaDevice::LoRaCredentials& h) {
			std::string cfg_deveui  = configuration_store->read_param<std::string>(ParamID::LORA_DEVEUI);
			std::string cfg_appeui  = configuration_store->read_param<std::string>(ParamID::LORA_APPEUI);
			std::string cfg_appkey  = configuration_store->read_param<std::string>(ParamID::LORA_APPKEY);
			std::string cfg_devaddr = configuration_store->read_param<std::string>(ParamID::LORA_DEVADDR);
			unsigned int cfg_njm    = configuration_store->read_param<unsigned int>(ParamID::LORA_NJM);
			bool m = (h.njm == cfg_njm);
			if (h.njm == 1) {
				// OTAA: DEVEUI is part of the join procedure, must match.
				m = m && (to_upper(h.deveui) == to_upper(cfg_deveui));
				m = m && (to_upper(h.appeui) == to_upper(cfg_appeui));
				m = m && (to_upper(h.appkey) == to_upper(cfg_appkey));
			} else {
				// ABP: DEVADDR selects the session, session keys encrypt. DEVEUI
				// is only a device identifier with no LoRaWAN function, so we
				// only enforce its match when the user explicitly set one in
				// LRP01 (non-empty and non-zero). This avoids spurious
				// mismatches when the RAK3172 reports 0000...0000 and the user
				// never wrote a DEVEUI (common ABP provisioning).
				m = m && (to_upper(h.devaddr) == to_upper(cfg_devaddr));
				if (!cfg_deveui.empty() &&
				    cfg_deveui != "0000000000000000") {
					m = m && (to_upper(h.deveui) == to_upper(cfg_deveui));
				}
			}
			return m;
		};

		bool match = compare(hw);
		unsigned int forced = 0;

		if (force == 1 && !match) {
			DEBUG_INFO("SATVF(LoRa): force=1, hardware differs — writing credentials from config store");
			if (lora_device_instance->write_credentials_from_config()) {
				forced = 1;
				// Re-read to confirm post-write state
				hw = lora_device_instance->read_lora_credentials();
				if (hw.read_ok) {
					match = compare(hw);
				} else {
					DEBUG_WARN("SATVF(LoRa): re-read after force-write failed");
					match = false;
				}
			} else {
				DEBUG_ERROR("SATVF(LoRa): force-write failed");
			}
		}

		// Build response fields: hw_id=0 (N/A for LoRa), hw_addr=NJM,
		// hw_seckey=credential summary, hw_rconf=mode info, match, forced
		std::string cred_summary = "NJM=" + std::to_string(hw.njm) +
			" DEVEUI=" + hw.deveui;
		std::string mode_info;
		if (hw.njm == 1) {
			cred_summary += " APPEUI=" + hw.appeui + " APPKEY=" + hw.appkey;
			mode_info = "OTAA";
		} else {
			cred_summary += " DEVADDR=" + hw.devaddr;
			mode_info = "ABP";
		}

		DEBUG_INFO("SATVF(LoRa): %s match=%d forced=%u", cred_summary.c_str(), match, forced);

		return DTEEncoder::encode(DTECommand::SATVF_RESP,
			(unsigned int)0,
			(unsigned int)0,             // hw_id (N/A for LoRa)
			(unsigned int)hw.njm,        // hw_addr repurposed as NJM mode
			cred_summary,                // hw_seckey repurposed as credential summary
			mode_info,                   // hw_rconf repurposed as mode info
			(unsigned int)(match ? 1U : 0U),
			forced);
#else
		// Satellite module (SMD or KIM2): read credentials from hardware
		if (!kineis_device_instance) {
			DEBUG_ERROR("SATVF_REQ: no satellite device available");
			return DTEEncoder::encode(DTECommand::SATVF_RESP, (int)DTEError::INCORRECT_DATA);
		}

		unsigned int hw_id = 0, hw_addr = 0;
		std::string hw_seckey, hw_rconf;
		kineis_device_instance->read_credentials(&hw_id, &hw_addr, &hw_seckey, &hw_rconf);

		unsigned int cfg_id = configuration_store->read_param<unsigned int>(ParamID::ARGOS_DECID);
		unsigned int cfg_addr = configuration_store->read_param<unsigned int>(ParamID::ARGOS_HEXID);

		bool match = (hw_id == cfg_id && hw_addr == cfg_addr);
		unsigned int forced = 0;

#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
		if (force == 1 && !match) {
			DEBUG_INFO("SATVF(SMD): force=1, hardware differs — marking credentials dirty");
			configuration_store->mark_credentials_dirty();
			// read_credentials() handles dirty flag: writes config creds to SMD + reloads KMAC, then re-reads
			hw_id = 0; hw_addr = 0; hw_seckey.clear(); hw_rconf.clear();
			kineis_device_instance->read_credentials(&hw_id, &hw_addr, &hw_seckey, &hw_rconf);
			match = (hw_id == cfg_id && hw_addr == cfg_addr);
			forced = 1;
		}
#else
		// KIM2: ID/ADDR are burned in (cannot be written), but force=1 still
		// re-programs the configured master RCONF and re-seeds the modulation
		// cache (the unforced read above only sees the module's power-on RCONF,
		// not the configured master). Lets an operator force a sync after editing
		// the RCONF without rebooting.
		if (force == 1 && kineis_device_instance) {
			DEBUG_INFO("SATVF(KIM2): force=1 — re-programming RCONF + re-seeding modulation cache");
			kineis_device_instance->resync_rconf_cache();
			configuration_store->save_params();  // persist the re-seeded modulation cache across reboot
			hw_id = 0; hw_addr = 0; hw_seckey.clear(); hw_rconf.clear();
			kineis_device_instance->read_credentials(&hw_id, &hw_addr, &hw_seckey, &hw_rconf);
			match = (hw_id == cfg_id && hw_addr == cfg_addr);
			forced = 1;
		}
#endif

		DEBUG_INFO("SATVF: hw_id=%u cfg_id=%u hw_addr=0x%08X cfg_addr=0x%08X match=%d forced=%u",
		           hw_id, cfg_id, hw_addr, cfg_addr, match, forced);

		return DTEEncoder::encode(DTECommand::SATVF_RESP,
			(unsigned int)0,
			(unsigned int)hw_id,
			(unsigned int)hw_addr,
			hw_seckey,
			hw_rconf,
			(unsigned int)(match ? 1U : 0U),
			forced);
#endif
	} catch (...) {
		error_code = (int)DTEError::INCORRECT_DATA;
	}

	return DTEEncoder::encode(DTECommand::SATVF_RESP, error_code);
}

std::string DTEHandler::ARGOSTX_REQ(int error_code, std::vector<BaseType>& arg_list) {
	if (error_code) {
		return DTEEncoder::encode(DTECommand::ARGOSTX_RESP, error_code);
	}

	// Two modes:
	//   Stored: SATTX,<modulation>,<size>[,<tcxo>]            — uses stored radioconf (ARP51/52/53)
	//   Custom: SATTX,<modulation>,<radioconf>,<size>[,<tcxo>] — radioconf hex string passed directly
	if (arg_list.size() < 2) {
		return DTEEncoder::encode(DTECommand::ARGOSTX_RESP, (int)DTEError::MISSING_ARGUMENT);
	}

	// Extract and validate modulation (0=>LDK, 1=>LDA2, 2=>VLDA4)
	unsigned int mod_val = std::get<unsigned int>(arg_list[0]);
	if (mod_val > 2) {
		DEBUG_WARN("DTEHandler::ARGOSTX_REQ: invalid modulation %u", mod_val);
		return DTEEncoder::encode(DTECommand::ARGOSTX_RESP, (int)DTEError::INCORRECT_DATA);
	}
	KineisModulation modulation = (KineisModulation)mod_val;

	// Max packet size per modulation: LDK=16 bytes (128 bits), LDA2/VLDA4=28 bytes (224 bits)
	unsigned int max_bytes = (modulation == KineisModulation::LDK) ? 16 : 28;

	std::string rconf;
	unsigned int num_bytes = 0;
	unsigned int tcxo_time = 0;

	// arg[1] is always TEXT from decoder — detect mode by string length:
	// radioconf is 32 hex chars, size is 1-2 decimal digits
	std::string arg2 = std::get<std::string>(arg_list[1]);
	bool custom_rconf = (arg2.length() > 2);

	if (custom_rconf) {
		// Custom: SATTX,<modulation>,<radioconf>,<size>[,<tcxo>]
		rconf = arg2;
		if (arg_list.size() < 3) {
			return DTEEncoder::encode(DTECommand::ARGOSTX_RESP, (int)DTEError::MISSING_ARGUMENT);
		}
		num_bytes = std::get<unsigned int>(arg_list[2]);
		tcxo_time = arg_list.size() > 3 ? std::get<unsigned int>(arg_list[3]) : 0;
	} else {
		// Stored: SATTX,<modulation>,<size>[,<tcxo>]
		num_bytes = std::stoul(arg2);
		tcxo_time = arg_list.size() > 2 ? std::get<unsigned int>(arg_list[2]) : 0;
	}

	// Validate packet size
	if (num_bytes == 0 || num_bytes > max_bytes) {
		DEBUG_WARN("DTEHandler::ARGOSTX_REQ: invalid size %u for modulation %u (max=%u)",
			num_bytes, mod_val, max_bytes);
		return DTEEncoder::encode(DTECommand::ARGOSTX_RESP, (int)DTEError::INCORRECT_DATA);
	}

	DEBUG_INFO("DTEHandler::ARGOSTX_REQ: mod=%u size=%u tcxo=%u custom_rconf=%u",
		mod_val, num_bytes, tcxo_time, custom_rconf);

	try {
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
		if (!smd_sat_instance) {
			return smd_not_available_error("ARGOSTX_REQ", DTECommand::ARGOSTX_RESP);
		}

		// If not already active then subscribe to events and setup a sufficiently
		// long idle period before the driver shuts off argos power
		if (!m_sat_device_active) {
			smd_sat_instance->subscribe(*this);
			smd_sat_instance->set_idle_timeout(30000);
			m_sat_device_active = true;
		}

		if (!custom_rconf) {
			// Load stored radioconf for the requested modulation
			ArgosConfig argos_config;
			configuration_store->get_argos_configuration(argos_config);
			switch (modulation) {
				case KineisModulation::LDK:  rconf = argos_config.radioconf_ldk; break;
				case KineisModulation::VLDA4: rconf = argos_config.radioconf_vlda4; break;
				case KineisModulation::LDA2:
				default:                      rconf = argos_config.radioconf_lda2; break;
			}
		}

		if (rconf.empty()) {
			DEBUG_WARN("DTEHandler::ARGOSTX_REQ: empty radioconf for modulation %u", mod_val);
			return DTEEncoder::encode(DTECommand::ARGOSTX_RESP, (int)DTEError::INCORRECT_DATA);
		}

		// Save current modulation before switching for SATTX
		KineisModulation prev_mod = smd_sat_instance->get_current_modulation();
		smd_sat_instance->switch_modulation(modulation, rconf);
		m_sattx_restore_modulation = prev_mod;
		m_sattx_needs_restore = true;

		smd_sat_instance->set_tcxo_warmup_time(tcxo_time);
		KineisPacket packet(num_bytes, 0xFF);
		smd_sat_instance->send(modulation, packet, 8 * num_bytes);

		// No immediate response — wait for KineisEventTxComplete/DeviceError
		// which will send the async response via react() callbacks
		return {};
#else
		// KIM2 (and any generic KineisDevice). Mirrors the SMD branch above via the
		// generic interface — the validated SMD #if branch is untouched. LoRa builds
		// don't set kineis_device_instance (they use LORATX_REQ), so this stays a
		// clean "no device" there.
		if (!kineis_device_instance) {
			DEBUG_WARN("DTEHandler::ARGOSTX_REQ: No satellite device available");
			return DTEEncoder::encode(DTECommand::ARGOSTX_RESP, (int)DTEError::INCORRECT_DATA);
		}

		// Subscribe for the async TX result and keep the module powered long enough
		// to transmit (released again on KineisEventPowerOff — see react()).
		if (!m_sat_device_active) {
			kineis_device_instance->subscribe(*this);
			kineis_device_instance->set_idle_timeout(30000);
			m_sat_device_active = true;
		}

		if (custom_rconf) {
			// Custom test RCONF supplied — program it for the requested modulation.
			// KIM2 RCONF is RAM-resident and re-applied from config at the next
			// power-on, so this does not persist past the session (no restore needed).
			if (rconf.length() != 32) {
				DEBUG_WARN("DTEHandler::ARGOSTX_REQ: invalid radioconf length %u",
					static_cast<unsigned>(rconf.length()));
				return DTEEncoder::encode(DTECommand::ARGOSTX_RESP, (int)DTEError::INCORRECT_DATA);
			}
			kineis_device_instance->switch_modulation(modulation, rconf);
		}
		// Stored case: no switch — KIM2's state machine applies its configured
		// (master / per-mode) RCONF in state_init and realigns in state_transmit_enter;
		// a modulation that doesn't match the configured RCONF fails cleanly (logged +
		// async DeviceError), which is the correct feedback for a test command.

		kineis_device_instance->set_tcxo_warmup_time(tcxo_time);
		KineisPacket packet(num_bytes, 0xFF);
		kineis_device_instance->send(modulation, packet, 8 * num_bytes);

		// No immediate response — KineisEventTxComplete / DeviceError send the async
		// ARGOSTX_RESP via the react() callbacks below.
		return {};
#endif
	} catch (...) {
		error_code = (int)DTEError::INCORRECT_DATA;
	}

	return DTEEncoder::encode(DTECommand::ARGOSTX_RESP, error_code);
}

#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
std::string DTEHandler::LORATX_REQ(int error_code, std::vector<BaseType>& arg_list) {
	if (error_code) {
		return DTEEncoder::encode(DTECommand::LORATX_RESP, error_code);
	}

	if (arg_list.size() < 1) {
		return DTEEncoder::encode(DTECommand::LORATX_RESP, (int)DTEError::MISSING_ARGUMENT);
	}

	unsigned int num_bytes = std::get<unsigned int>(arg_list[0]);

	DEBUG_INFO("DTEHandler::LORATX_REQ: size=%u bytes", num_bytes);

	if (!lora_device_instance) {
		DEBUG_WARN("DTEHandler::LORATX_REQ: LoRa device not available");
		return DTEEncoder::encode(DTECommand::LORATX_RESP, (int)DTEError::INCORRECT_DATA);
	}

	// Validate payload size against configured data rate limit
	uint8_t dr = (uint8_t)configuration_store->read_param<unsigned int>(ParamID::LORA_DR);
	unsigned int max_bytes = LoRaPayloadLimits::max_payload_for_dr(dr);
	if (num_bytes == 0 || num_bytes > max_bytes) {
		DEBUG_WARN("DTEHandler::LORATX_REQ: size %u exceeds DR%u max %u bytes", num_bytes, dr, max_bytes);
		return DTEEncoder::encode(DTECommand::LORATX_RESP, (int)DTEError::INCORRECT_DATA);
	}

	try {
		if (!m_sat_device_active) {
			lora_device_instance->subscribe(*this);
			m_sat_device_active = true;
		}
		m_lora_tx_active = true;

		// Build test payload (all 0xFF bytes)
		KineisPacket packet(num_bytes, 0xFF);
		lora_device_instance->send(KineisModulation::LDA2, packet, 8 * num_bytes);

		// Async response — wait for KineisEventTxComplete/DeviceError
		return {};
	} catch (...) {
		m_lora_tx_active = false;
		error_code = (int)DTEError::INCORRECT_DATA;
	}

	return DTEEncoder::encode(DTECommand::LORATX_RESP, error_code);
}

std::string DTEHandler::LORABR_REQ(int error_code, std::vector<BaseType>& arg_list) {
	if (error_code) {
		return DTEEncoder::encode(DTECommand::LORABR_RESP, error_code);
	}

	if (arg_list.size() < 1) {
		return DTEEncoder::encode(DTECommand::LORABR_RESP, (int)DTEError::MISSING_ARGUMENT);
	}

	unsigned int action = std::get<unsigned int>(arg_list[0]);

	if (!lora_device_instance) {
		DEBUG_WARN("DTEHandler::LORABR_REQ: LoRa device not available");
		return DTEEncoder::encode(DTECommand::LORABR_RESP, (int)DTEError::INCORRECT_DATA);
	}

	if (action == 1) {
		// Start bridge: raw UART RX → USB CDC via async_write
		auto write_fn = m_async_write;
		lora_device_instance->start_bridge([write_fn](const uint8_t* data, size_t len) {
			if (write_fn) {
				write_fn(std::string(reinterpret_cast<const char*>(data), len));
			}
		});
		DEBUG_INFO("DTEHandler::LORABR_REQ: bridge STARTED — type +++ to exit");
	} else {
		lora_device_instance->stop_bridge();
		DEBUG_INFO("DTEHandler::LORABR_REQ: bridge STOPPED");
	}

	return DTEEncoder::encode(DTECommand::LORABR_RESP, (int)DTEError::OK);
}

#endif

#if !(defined(LORA_RAK3172) && (LORA_RAK3172 == 1)) && !(defined(ARGOS_SMD) && (ARGOS_SMD == 1))

std::string DTEHandler::KIMBR_REQ(int error_code, std::vector<BaseType>& arg_list) {
	if (error_code) {
		return DTEEncoder::encode(DTECommand::KIMBR_RESP, error_code);
	}

	if (arg_list.size() < 1) {
		return DTEEncoder::encode(DTECommand::KIMBR_RESP, (int)DTEError::MISSING_ARGUMENT);
	}

	unsigned int action = std::get<unsigned int>(arg_list[0]);

	if (!kim2_device_instance) {
		DEBUG_WARN("DTEHandler::KIMBR_REQ: KIM2 device not available");
		return DTEEncoder::encode(DTECommand::KIMBR_RESP, (int)DTEError::INCORRECT_DATA);
	}

	if (action == 1) {
		// Start bridge: raw UART RX → USB CDC via async_write
		auto write_fn = m_async_write;
		kim2_device_instance->start_bridge([write_fn](const uint8_t* data, size_t len) {
			if (write_fn) {
				write_fn(std::string(reinterpret_cast<const char*>(data), len));
			}
		});
		DEBUG_INFO("DTEHandler::KIMBR_REQ: bridge STARTED — type +++ to exit");
	} else {
		kim2_device_instance->stop_bridge();
		DEBUG_INFO("DTEHandler::KIMBR_REQ: bridge STOPPED");
	}

	return DTEEncoder::encode(DTECommand::KIMBR_RESP, (int)DTEError::OK);
}

#endif


std::string DTEHandler::SENSR_REQ(int error_code, std::vector<BaseType>& arg_list) {
	// Response values (initialized to defaults/invalid)
	unsigned int batt_mv = 0;
	unsigned int batt_soc = 0;
	double pressure = 0.0;
	double temperature = 0.0;
	double altitude = 0.0;
	double lat = 0.0;
	double lon = 0.0;
	double hdop = 99.9;
	unsigned int num_sv = 0;
	double accel_x = 0.0;
	double accel_y = 0.0;
	double accel_z = 0.0;
	double accel_temp = 0.0;
	unsigned int activity = 0;
	double thermistor_temp = 0.0;
	double sea_temp_c = 0.0;
	double als_lux = 0.0;
	double ph_value = 0.0;

	// Status bitmask: bit set = sensor OK, bit clear = sensor unavailable
	// Bit 0=battery, 1=pressure, 2=gnss, 3=accel, 4=thermistor, 5=sea_temp, 6=als, 7=ph
	// Upper byte: bit 8=not_compiled (sensor not in build), bit 9=not_enabled (disabled in config)
	// Per-sensor status: 0x01=OK, 0x02=not_compiled, 0x03=not_enabled, 0x04=read_error
	unsigned int sensor_status = 0;  // Each bit = sensor OK

	if (error_code) {
		return DTEEncoder::encode(DTECommand::SENSR_RESP, error_code,
			batt_mv, batt_soc, pressure, temperature, altitude, lat, lon, hdop, num_sv,
			accel_x, accel_y, accel_z, accel_temp, activity, thermistor_temp,
			sea_temp_c, als_lux, ph_value, sensor_status);
	}

	// Parse parameters
	if (arg_list.size() < 2) {
		return DTEEncoder::encode(DTECommand::SENSR_RESP, (int)DTEError::MISSING_ARGUMENT,
			batt_mv, batt_soc, pressure, temperature, altitude, lat, lon, hdop, num_sv,
			accel_x, accel_y, accel_z, accel_temp, activity, thermistor_temp,
			sea_temp_c, als_lux, ph_value, sensor_status);
	}

	unsigned int sensors_mask = std::get<unsigned int>(arg_list[0]);
	unsigned int timeout_s = std::get<unsigned int>(arg_list[1]);
	(void)timeout_s;  // Currently unused - for future GNSS acquisition

	DEBUG_INFO("DTEHandler::SENSR_REQ: sensors_mask=0x%02X timeout=%us", sensors_mask, timeout_s);

	// RSPB: the I2C pull-up rail (VSENSORS) is gated by POWER_CONTROL, which
	// deep-idle reduce_power_rails() cuts while the device sits idle waiting for a
	// BLE/DTE command in ConfigurationState — so the bus is DEAD when a SENSR
	// arrives (works fine in OperationalState where the scheduler keeps the rail
	// up). Restore the board rail (POWER_CONTROL high + settle) before any read so
	// the live diagnostic works off-mission. Cheap and safe on other boards (no-op
	// unless VSYS_SEL/POWER_CONTROL defined).
	PMU::restore_power_rails();

	// Read battery (bit 0 = 0x01). Force a FRESH read (bypass cache + re-probe)
	// and report the REAL result, so SENSR is a clean live diagnostic — e.g. run
	// it with the SMD powered (PWRON SATELLITE) vs off to confirm whether an
	// unpowered SMD on the shared I2C bus is what mutes the gauge.
	if (sensors_mask & 0x01) {
		if (battery_monitor) {
			bool batt_ok = battery_monitor->update_forced();
			batt_mv = battery_monitor->get_voltage();
			batt_soc = battery_monitor->get_level();
			if (batt_ok) {
				sensor_status |= (1 << 0);  // Battery read OK (gauge answered)
				DEBUG_TRACE("SENSR: Battery %umV | %u%% (read OK)", batt_mv, batt_soc);
			} else {
				// Bit stays clear = read error. Value returned is stale/default.
				DEBUG_WARN("SENSR: Battery read FAILED (STC no ACK) — %umV is stale/default", batt_mv);
			}
		} else {
			DEBUG_WARN("SENSR: Battery monitor not available");
		}
	}

	// Read pressure sensor (bit 1 = 0x02)
	if (sensors_mask & 0x02) {
#if ENABLE_PRESSURE_SENSOR
		try {
			Sensor& prs = SensorManager::find_by_name("PRS");
			pressure = prs.read(0);
			temperature = prs.read(1);
			double sea_level_hpa = 1013.25;
			prs.calibration_read(sea_level_hpa, 0);
			double pressure_hpa = pressure * 1000.0;
			if (sea_level_hpa > 0.0 && pressure_hpa > 0.0) {
				altitude = 44330.0 * (1.0 - std::pow(pressure_hpa / sea_level_hpa, 1.0 / 5.255));
			}
			sensor_status |= (1 << 1);  // Pressure OK
			DEBUG_TRACE("SENSR: Pressure %.4f bar | Temp %.2f C | Alt %.2f m", pressure, temperature, altitude);
		} catch (...) {
			DEBUG_WARN("SENSR: Pressure sensor read error");
		}
#else
		DEBUG_WARN("SENSR: Pressure sensor not compiled in this build");
#endif
	}

	// Read GNSS (bit 2 = 0x04)
	if (sensors_mask & 0x04) {
		const GPSLogEntry& gps_entry = configuration_store->get_last_gps_entry();
		if (gps_entry.info.valid) {
			lat = gps_entry.info.lat;
			lon = gps_entry.info.lon;
			hdop = gps_entry.info.hDOP;
			num_sv = gps_entry.info.numSV;
			sensor_status |= (1 << 2);  // GNSS OK
			DEBUG_TRACE("SENSR: GNSS lat=%.6f lon=%.6f hdop=%.1f numSV=%u", lat, lon, hdop, num_sv);
		} else {
			DEBUG_WARN("SENSR: No valid GNSS fix available");
		}
	}

	// Read accelerometer (bit 3 = 0x08)
	if (sensors_mask & 0x08) {
#if ENABLE_AXL_SENSOR
		try {
			Sensor& axl = SensorManager::find_by_name("AXL");
			accel_x = axl.read(1);
			accel_y = axl.read(2);
			accel_z = axl.read(3);
			accel_temp = axl.read(0);
			activity = (unsigned int)axl.read(4);
			sensor_status |= (1 << 3);  // AXL OK
			DEBUG_TRACE("SENSR: Accel X=%.3f Y=%.3f Z=%.3f T=%.1f activity=%u",
				accel_x, accel_y, accel_z, accel_temp, activity);
		} catch (...) {
			DEBUG_WARN("SENSR: Accelerometer read error");
		}
#else
		DEBUG_WARN("SENSR: Accelerometer not compiled in this build");
#endif
	}

	// Read thermistor (bit 4 = 0x10)
	if (sensors_mask & 0x10) {
#if ENABLE_THERMISTOR_SENSOR
		try {
			Sensor& therm = SensorManager::find_by_name("THERMISTOR");
			thermistor_temp = therm.read(0);
			sensor_status |= (1 << 4);  // Thermistor OK
			DEBUG_TRACE("SENSR: Thermistor %.2f C", thermistor_temp);
		} catch (...) {
			DEBUG_WARN("SENSR: Thermistor read error");
		}
#else
		DEBUG_WARN("SENSR: Thermistor not compiled in this build");
#endif
	}

	// Read sea temperature (bit 5 = 0x20)
	if (sensors_mask & 0x20) {
#if ENABLE_SEA_TEMP_SENSOR
		try {
			Sensor& sea = SensorManager::find_by_name("SEA_TEMP");
			sea_temp_c = sea.read(0);
			sensor_status |= (1 << 5);  // Sea temp OK
			DEBUG_TRACE("SENSR: Sea temp %.3f C", sea_temp_c);
		} catch (...) {
			DEBUG_WARN("SENSR: Sea temperature read error");
		}
#else
		DEBUG_WARN("SENSR: Sea temperature not compiled in this build");
#endif
	}

	// Read ambient light (bit 6 = 0x40)
	if (sensors_mask & 0x40) {
#if ENABLE_ALS_SENSOR
		try {
			Sensor& als = SensorManager::find_by_name("ALS");
			als_lux = als.read(0);
			sensor_status |= (1 << 6);  // ALS OK
			DEBUG_TRACE("SENSR: ALS %.1f lux", als_lux);
		} catch (...) {
			DEBUG_WARN("SENSR: ALS read error");
		}
#else
		DEBUG_WARN("SENSR: ALS not compiled in this build");
#endif
	}

	// Read pH (bit 7 = 0x80)
	if (sensors_mask & 0x80) {
#if ENABLE_PH_SENSOR
		try {
			Sensor& ph = SensorManager::find_by_name("PH");
			ph_value = ph.read(0);
			sensor_status |= (1 << 7);  // pH OK
			DEBUG_TRACE("SENSR: pH %.3f", ph_value);
		} catch (...) {
			DEBUG_WARN("SENSR: pH read error");
		}
#else
		DEBUG_WARN("SENSR: pH not compiled in this build");
#endif
	}

	DEBUG_INFO("DTEHandler::SENSR_REQ: sensor_status=0x%02X (requested=0x%02X)", sensor_status, sensors_mask);

	return DTEEncoder::encode(DTECommand::SENSR_RESP, (int)DTEError::OK,
		batt_mv, batt_soc, pressure, temperature, altitude, lat, lon, hdop, num_sv,
		accel_x, accel_y, accel_z, accel_temp, activity, thermistor_temp,
		sea_temp_c, als_lux, ph_value, sensor_status);
}

// PWRON handler - Power on/off components
std::string DTEHandler::PWRON_REQ(int error_code, std::vector<BaseType>& arg_list) {
	if (error_code) {
		return DTEEncoder::encode(DTECommand::PWRON_RESP, error_code);
	}

	// Parse component parameter
	if (arg_list.size() < 1) {
		return DTEEncoder::encode(DTECommand::PWRON_RESP, (int)DTEError::MISSING_ARGUMENT);
	}

	unsigned int component = std::get<unsigned int>(arg_list[0]);
	DEBUG_INFO("DTEHandler::PWRON_REQ: component=%u", component);

	switch ((ComponentPower)component) {
	case ComponentPower::ALL:
		// Power on all components via drivers
		DEBUG_TRACE("PWRON: Powering ON all components");
		if (gps_device) {
			GPSNavSettings nav = {};
			gps_device->power_on(nav);
		}
		GPIOPins::acquire_sensors_pwr();
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
		GPIOPins::set(SAT_PWR_EN);
#elif defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
		GPIOPins::set(SAT_PWR_EN);
#endif
		break;

	case ComponentPower::GNSS:
		// Power on GNSS via M10Q driver (triggers full state machine: configure, etc.)
		DEBUG_TRACE("PWRON: Powering ON GNSS");
		if (gps_device) {
			GPSNavSettings nav = {};
			gps_device->power_on(nav);
		} else {
			return DTEEncoder::encode(DTECommand::PWRON_RESP, (int)DTEError::INCORRECT_DATA);
		}
		break;

	case ComponentPower::SENSORS:
		// Power on sensors
		DEBUG_TRACE("PWRON: Powering ON sensors");
		GPIOPins::acquire_sensors_pwr();
		break;

	case ComponentPower::SATELLITE:
		// Power on satellite module (requires VSENSORS for stable power rail)
		DEBUG_TRACE("PWRON: Powering ON satellite module");
		GPIOPins::acquire_sensors_pwr();
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
		GPIOPins::set(SAT_PWR_EN);
#elif defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
		GPIOPins::set(SAT_PWR_EN);
#else
		GPIOPins::set(SAT_PWR_EN);
		GPIOPins::set(SAT_EXTWAKEUP);
#endif
		break;

	case ComponentPower::OFF:
		// Power off all components via drivers
		DEBUG_TRACE("PWRON: Powering OFF all components");
		if (gps_device) {
			gps_device->power_off();
		}
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
		GPIOPins::clear(SAT_PWR_EN);
#elif defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
		GPIOPins::clear(SAT_PWR_EN);
#else
		GPIOPins::clear(SAT_PWR_EN);
		GPIOPins::clear(SAT_EXTWAKEUP);
#endif
		GPIOPins::release_sensors_pwr();
		break;

	default:
		return DTEEncoder::encode(DTECommand::PWRON_RESP, (int)DTEError::VALUE_OUT_OF_RANGE);
	}

	return DTEEncoder::encode(DTECommand::PWRON_RESP, (int)DTEError::OK);
}

std::string DTEHandler::GNSSBCKP_REQ(int error_code, std::vector<BaseType>& arg_list) {
	if (error_code) {
		return DTEEncoder::encode(DTECommand::GNSSBCKP_RESP, error_code);
	}
	if (arg_list.size() < 1) {
		return DTEEncoder::encode(DTECommand::GNSSBCKP_RESP, (int)DTEError::MISSING_ARGUMENT);
	}
	if (!gps_service) {
		return DTEEncoder::encode(DTECommand::GNSSBCKP_RESP, (int)DTEError::INCORRECT_DATA);
	}
	unsigned int duration_s = std::get<unsigned int>(arg_list[0]);
	DEBUG_INFO("DTEHandler::GNSSBCKP_REQ: duration_s=%u", duration_s);
	if (!gps_service->request_backup_charge(duration_s))
		return DTEEncoder::encode(DTECommand::GNSSBCKP_RESP, (int)DTEError::INCORRECT_DATA);
	return DTEEncoder::encode(DTECommand::GNSSBCKP_RESP, (int)DTEError::OK);
}

std::string DTEHandler::SWSST_REQ(int error_code) {
	if (error_code) {
		return DTEEncoder::encode(DTECommand::SWSST_RESP, error_code);
	}

#if ENABLE_SWS_ANALOG
	auto st = SWSAnalogService::get_status();

	return DTEEncoder::encode(DTECommand::SWSST_RESP, (int)DTEError::OK,
		(unsigned int)st.threshold_air,
		(unsigned int)st.threshold_water,
		(unsigned int)st.threshold_current,
		(unsigned int)st.hysteresis,
		(unsigned int)st.last_raw_adc,
		(unsigned int)st.last_filtered_adc,
		(unsigned int)st.is_calibrated,
		(unsigned int)st.is_underwater,
		(unsigned int)st.time_in_state_sec,
		(unsigned int)st.surface_level,
		(unsigned int)st.contrast_x10,
		(unsigned int)st.observed_peak,
		(unsigned int)st.sample_delay_us);
#else
	return DTEEncoder::encode(DTECommand::SWSST_RESP, (int)DTEError::PARAM_KEY_UNRECOGNISED);
#endif
}

std::string DTEHandler::SWSTST_REQ(int error_code, std::vector<BaseType>& arg_list) {
	if (error_code) {
		return DTEEncoder::encode(DTECommand::SWSTST_RESP, error_code);
	}

	if (arg_list.size() < 1) {
		return DTEEncoder::encode(DTECommand::SWSTST_RESP, (int)DTEError::MISSING_ARGUMENT);
	}

	unsigned int action = std::get<unsigned int>(arg_list[0]);

#if ENABLE_SWS_ANALOG
	if (action == 1) {
		// Start test mode and register async push callback
		auto write_fn = m_async_write;
		SWSAnalogService::set_status_notify([write_fn](const SWSAnalogService::Status& st) {
			if (write_fn) {
				write_fn(DTEEncoder::encode(DTECommand::SWSST_RESP, (int)DTEError::OK,
					(unsigned int)st.threshold_air,
					(unsigned int)st.threshold_water,
					(unsigned int)st.threshold_current,
					(unsigned int)st.hysteresis,
					(unsigned int)st.last_raw_adc,
					(unsigned int)st.last_filtered_adc,
					(unsigned int)st.is_calibrated,
					(unsigned int)st.is_underwater,
					(unsigned int)st.time_in_state_sec,
					(unsigned int)st.surface_level,
					(unsigned int)st.contrast_x10,
					(unsigned int)st.observed_peak,
					(unsigned int)st.sample_delay_us));
			}
		});
		SWSAnalogService::set_on_test_stop([]() {
			// Restore normal LED: flash blue (ConfigNotConnected default)
			if (status_led)
				status_led->flash(RGBLedColor::BLUE);
		});
		SWSAnalogService::start_test_mode();
	} else {
		SWSAnalogService::stop_test_mode();
		SWSAnalogService::clear_status_notify();
		SWSAnalogService::clear_on_test_stop();
	}

	unsigned int running = SWSAnalogService::is_test_running() ? 1U : 0U;
	return DTEEncoder::encode(DTECommand::SWSTST_RESP, (int)DTEError::OK, running);
#else
	(void)action;
	return DTEEncoder::encode(DTECommand::SWSTST_RESP, (int)DTEError::PARAM_KEY_UNRECOGNISED);
#endif
}

std::string DTEHandler::SWSSTATS_REQ(int error_code, std::vector<BaseType>& arg_list) {
	if (error_code) {
		return DTEEncoder::encode(DTECommand::SWSSTATS_RESP, error_code);
	}

	if (arg_list.size() < 1) {
		return DTEEncoder::encode(DTECommand::SWSSTATS_RESP, (int)DTEError::MISSING_ARGUMENT);
	}

	unsigned int action = std::get<unsigned int>(arg_list[0]);

#if ENABLE_SWS_ANALOG
	if (action == 1) {
		SWSAnalogService::clear_diagnostics();
	}

	auto d = SWSAnalogService::get_diagnostics();
	return DTEEncoder::encode(DTECommand::SWSSTATS_RESP, (int)DTEError::OK,
		(unsigned int)d.stuck_recovery_count,
		(unsigned int)d.coherence_recalib_count,
		(unsigned int)d.dive_timeout_count,
		(unsigned int)d.force_surface_count,
		(unsigned int)d.spike_reject_count,
		(unsigned int)d.peak_incoherent_count,
		(unsigned int)d.saadc_init_retry_count);
#else
	(void)action;
	return DTEEncoder::encode(DTECommand::SWSSTATS_RESP, (int)DTEError::PARAM_KEY_UNRECOGNISED);
#endif
}

std::string DTEHandler::SWSCAL_REQ(int error_code, std::vector<BaseType>& arg_list) {
	if (error_code) {
		return DTEEncoder::encode(DTECommand::SWSCAL_RESP, error_code);
	}

	if (arg_list.size() < 1) {
		return DTEEncoder::encode(DTECommand::SWSCAL_RESP, (int)DTEError::MISSING_ARGUMENT);
	}

	unsigned int action = std::get<unsigned int>(arg_list[0]);

#if ENABLE_SWS_ANALOG
	if (action == 1) {
		if (SWSAnalogService::is_test_running()) {
			// Stop any running test mode first
			SWSAnalogService::stop_test_mode();
			SWSAnalogService::clear_status_notify();
		}
		// Capture `this` (not a snapshot of m_async_write) so the notify uses
		// whichever writer is current when the calibration completes (~10 s
		// later). If BLE flaps mid-calibration, the dte_handler's async_write
		// has already been switched to USB by the gentracker handler — using
		// the snapshot would silently route the completion frame to the dead
		// channel and the GUI never sees status=1.
		SWSAnalogService::set_guided_calib_notify([this](const SWSAnalogService::CalibResult& r) {
			if (m_async_write) {
				m_async_write(DTEEncoder::encode(DTECommand::SWSCAL_RESP, (int)DTEError::OK,
					(unsigned int)r.status,
					(unsigned int)r.air,
					(unsigned int)r.water));
			}
		});
		SWSAnalogService::set_on_test_stop([]() {
			// Hand the LED back to ledsm. SWSCAL only makes sense in
			// ConfigurationState with BLE attached, so dispatch the
			// connected-config event — ledsm decides the actual visual
			// (solid BLUE, white if magnet engaged, etc.). Without this
			// re-dispatch, ledsm has no event to react to and the LED
			// stays on the post-calibration WHITE/RED flash forever.
			LEDState::dispatch(SetLEDConfigConnected{});
		});
		SWSAnalogService::start_guided_calibration();
	} else {
		SWSAnalogService::cancel_guided_calibration();
		SWSAnalogService::clear_guided_calib_notify();
	}

	auto r = SWSAnalogService::get_guided_calibration_result();
	return DTEEncoder::encode(DTECommand::SWSCAL_RESP, (int)DTEError::OK,
		(unsigned int)r.status,
		(unsigned int)r.air,
		(unsigned int)r.water);
#else
	(void)action;
	return DTEEncoder::encode(DTECommand::SWSCAL_RESP, (int)DTEError::PARAM_KEY_UNRECOGNISED);
#endif
}

std::string DTEHandler::GNSSBR_REQ(int error_code, std::vector<BaseType>& arg_list) {
	if (error_code) {
		return DTEEncoder::encode(DTECommand::GNSSBR_RESP, error_code);
	}

	if (arg_list.size() < 1) {
		return DTEEncoder::encode(DTECommand::GNSSBR_RESP, (int)DTEError::MISSING_ARGUMENT);
	}

	unsigned int action = std::get<unsigned int>(arg_list[0]);

	if (!gps_device) {
		DEBUG_WARN("DTEHandler::GNSSBR_REQ: GNSS device not available");
		return DTEEncoder::encode(DTECommand::GNSSBR_RESP, (int)DTEError::INCORRECT_DATA);
	}

	if (action == 1) {
		// Start bridge: raw UART RX → USB CDC via async_write
		auto write_fn = m_async_write;
		gps_device->start_bridge([write_fn](const uint8_t* data, size_t len) {
			if (write_fn) {
				write_fn(std::string(reinterpret_cast<const char*>(data), len));
			}
		});
		DEBUG_INFO("DTEHandler::GNSSBR_REQ: bridge STARTED — type +++ to exit");
	} else {
		gps_device->stop_bridge();
		DEBUG_INFO("DTEHandler::GNSSBR_REQ: bridge STOPPED");
	}

	return DTEEncoder::encode(DTECommand::GNSSBR_RESP, (int)DTEError::OK);
}

// Helper to format and return GNSSI response from cached info
static std::string format_gnssi_response(const GNSSDeviceInfo& info) {
	char uid_hex[11];
	snprintf(uid_hex, sizeof(uid_hex), "%02X%02X%02X%02X%02X",
		info.uniqueId[0], info.uniqueId[1], info.uniqueId[2],
		info.uniqueId[3], info.uniqueId[4]);

	return DTEEncoder::encode(DTECommand::GNSSI_RESP, (int)DTEError::OK,
		std::string(uid_hex),
		std::string(info.swVersion),
		std::string(info.hwVersion));
}

std::string DTEHandler::GNSSI_REQ(int error_code) {
	if (error_code) {
		return DTEEncoder::encode(DTECommand::GNSSI_RESP, error_code);
	}

	if (!gps_device) {
		return DTEEncoder::encode(DTECommand::GNSSI_RESP, (int)DTEError::INCORRECT_DATA);
	}

	// If info is already cached, return immediately
	auto info = gps_device->get_device_info();
	if (info.valid) {
		return format_gnssi_response(info);
	}

	// Autonomous mode: power on GNSS, wait for device info via async callback
	DEBUG_INFO("DTEHandler::GNSSI_REQ: info not cached, powering on GNSS autonomously");

	if (!m_gps_subscribed) {
		gps_device->subscribe(*this);
		m_gps_subscribed = true;
	}

	m_gnssi_pending = true;
	GNSSConfig gnss_config;
	configuration_store->get_gnss_configuration(gnss_config);
	GPSNavSettings nav = {
		gnss_config.fix_mode,
		gnss_config.dyn_model,
		gnss_config.assistnow_enable,
		gnss_config.assistnow_offline_enable,
		gnss_config.hdop_filter_enable,
		gnss_config.hdop_filter_threshold,
		gnss_config.hacc_filter_enable,
		gnss_config.hacc_filter_threshold,
	};
	gps_device->power_on(nav);

	// No immediate response — GPSEventDeviceInfoReady will trigger async response
	return {};
}

void DTEHandler::react(const GPSEventDeviceInfoReady&) {
	if (!m_gnssi_pending) return;
	m_gnssi_pending = false;

	DEBUG_INFO("DTEHandler::react: GPSEventDeviceInfoReady — sending GNSSI response");

	auto info = gps_device->get_device_info();
	std::string resp;
	if (info.valid) {
		resp = format_gnssi_response(info);
	} else {
		resp = DTEEncoder::encode(DTECommand::GNSSI_RESP, (int)DTEError::INCORRECT_DATA);
	}

	if (m_async_write) {
		m_async_write(resp);
	}

	// Power off GNSS now that we have the info
	gps_device->power_off();
}

void DTEHandler::react(const GPSEventError&) {
	if (!m_gnssi_pending) return;
	m_gnssi_pending = false;

	DEBUG_WARN("DTEHandler::react: GPSEventError — GNSSI failed");

	if (m_async_write) {
		m_async_write(DTEEncoder::encode(DTECommand::GNSSI_RESP, (int)DTEError::INCORRECT_DATA));
	}

	// Power off GNSS to reset state machine back to idle
	gps_device->power_off();
}

std::string DTEHandler::GNSSA_REQ(int error_code) {
	if (error_code) {
		return DTEEncoder::encode(DTECommand::GNSSA_RESP, error_code);
	}

	if (!gps_device) {
		return DTEEncoder::encode(DTECommand::GNSSA_RESP, (int)DTEError::INCORRECT_DATA);
	}

	auto status = gps_device->get_almanac_status();

	return DTEEncoder::encode(DTECommand::GNSSA_RESP, (int)DTEError::OK,
		(unsigned int)(status.file_present ? 1 : 0),
		(unsigned int)status.file_size,
		(unsigned int)status.total_records,
		(unsigned int)status.valid_records,
		(unsigned int)(status.stale ? 1 : 0));
}

std::string DTEHandler::RTCW_REQ(int error_code, std::vector<BaseType>& arg_list) {
	if (error_code) {
		return DTEEncoder::encode(DTECommand::RTCW_RESP, error_code);
	}

	if (arg_list.size() < 1) {
		return DTEEncoder::encode(DTECommand::RTCW_RESP, (int)DTEError::MISSING_ARGUMENT);
	}

	unsigned int timestamp = std::get<unsigned int>(arg_list[0]);

	if (timestamp == 0) {
		return DTEEncoder::encode(DTECommand::RTCW_RESP, (int)DTEError::VALUE_OUT_OF_RANGE);
	}

	if (!rtc) {
		return DTEEncoder::encode(DTECommand::RTCW_RESP, (int)DTEError::INCORRECT_DATA);
	}

	// Set the RTC to the provided unix timestamp
	rtc->settime(static_cast<std::time_t>(timestamp));
	DEBUG_INFO("DTEHandler::RTCW_REQ: RTC set to %u", timestamp);

	// Also update LAST_KNOWN_RTC so the pseudo RTC chain continues from this value
	configuration_store->write_param(ParamID::LAST_KNOWN_RTC, timestamp);
	configuration_store->save_params();
	DEBUG_TRACE("DTEHandler::RTCW_REQ: LAST_KNOWN_RTC updated to %u", timestamp);

	return DTEEncoder::encode(DTECommand::RTCW_RESP, (int)DTEError::OK);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wswitch-enum"

DTEAction DTEHandler::handle_dte_message(const std::string& req, std::string& resp) {
	DTECommand command;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	std::vector<std::string> rejected_keys;
	unsigned int error_code = (unsigned int)DTEError::OK;
	DTEAction action = DTEAction::NONE;

	if (!configuration_store) {
		DEBUG_ERROR("DTEHandler: configuration_store is null");
		return action;
	}

	try {
		if (!DTEDecoder::decode(req, command, error_code, arg_list, params, param_values, rejected_keys))
			return action;
	} catch (ErrorCode e) {

		switch (e) {
		case ErrorCode::DTE_PROTOCOL_MESSAGE_TOO_LARGE:
			error_code = (unsigned int)DTEError::MESSAGE_TOO_LARGE;
			break;
		case ErrorCode::DTE_PROTOCOL_PARAM_KEY_UNRECOGNISED:
			error_code = (unsigned int)DTEError::PARAM_KEY_UNRECOGNISED;
			break;
		case ErrorCode::DTE_PROTOCOL_UNEXPECTED_ARG:
			error_code = (unsigned int)DTEError::UNEXPECTED_ARGUMENT;
			break;
		case ErrorCode::DTE_PROTOCOL_VALUE_OUT_OF_RANGE:
			error_code = (unsigned int)DTEError::VALUE_OUT_OF_RANGE;
			break;
		case ErrorCode::DTE_PROTOCOL_MISSING_ARG:
			error_code = (unsigned int)DTEError::MISSING_ARGUMENT;
			break;
		case ErrorCode::DTE_PROTOCOL_BAD_FORMAT:
			error_code = (unsigned int)DTEError::BAD_FORMAT;
			break;
		case ErrorCode::DTE_PROTOCOL_PAYLOAD_LENGTH_MISMATCH:
			error_code = (unsigned int)DTEError::DATA_LENGTH_MISMATCH;
			break;
		case ErrorCode::DTE_PROTOCOL_UNKNOWN_COMMAND:
			error_code = (unsigned int)DTEError::INCORRECT_COMMAND;
			break;
		default:
			DEBUG_ERROR("DTEHandler: unhandled ErrorCode %u from decoder", (unsigned int)e);
			error_code = (unsigned int)DTEError::BAD_FORMAT;
			break;
		}
	}

	try {
	switch(command) {
	case DTECommand::PARML_REQ:
		resp = PARML_REQ(error_code);
		break;
	case DTECommand::PARMW_REQ:
		resp = PARMW_REQ(error_code, param_values, rejected_keys, action);
		break;
	case DTECommand::PARMR_REQ:
		resp = PARMR_REQ(error_code, params);
		break;
	case DTECommand::STATR_REQ:
		resp = STATR_REQ(error_code, params);
		break;
	case DTECommand::PROFW_REQ:
		resp = PROFW_REQ(error_code, arg_list);
		break;
	case DTECommand::PROFR_REQ:
		resp = PROFR_REQ(error_code);
		break;
	case DTECommand::SECUR_REQ:
		resp = SECUR_REQ(error_code, arg_list);
		if (!error_code) action = DTEAction::SECUR;
		break;
	case DTECommand::RSTVW_REQ:
		resp = RSTVW_REQ(error_code, arg_list);
		break;
	case DTECommand::RSTBW_REQ:
		resp = RSTBW_REQ(error_code);
		if (!error_code) action = DTEAction::RESET;
		break;
	case DTECommand::FACTW_REQ:
		resp = FACTW_REQ(error_code);
		if (!error_code) action = DTEAction::FACTR;
		break;
	case DTECommand::DUMPM_REQ:
		resp = DUMPM_REQ(error_code, arg_list);
		break;
	case DTECommand::PASPW_REQ:
		resp = PASPW_REQ(error_code, arg_list);
		break;
	case DTECommand::DUMPD_REQ:
		resp = DUMPD_REQ(error_code, arg_list, action);
		break;
	case DTECommand::ERASE_REQ:
		resp = ERASE_REQ(error_code, arg_list);
		break;
	case DTECommand::SCALW_REQ:
		resp = SCALW_REQ(error_code, arg_list);
		break;
	case DTECommand::SCALR_REQ:
		resp = SCALR_REQ(error_code, arg_list);
		break;
	case DTECommand::ARGOSTX_REQ:
		resp = ARGOSTX_REQ(error_code, arg_list);
		break;
	case DTECommand::SENSR_REQ:
		resp = SENSR_REQ(error_code, arg_list);
		break;
	case DTECommand::PWRON_REQ:
		resp = PWRON_REQ(error_code, arg_list);
		break;
	case DTECommand::GNSSBCKP_REQ:
		resp = GNSSBCKP_REQ(error_code, arg_list);
		break;
	case DTECommand::SWSST_REQ:
		resp = SWSST_REQ(error_code);
		break;
	case DTECommand::SWSTST_REQ:
		resp = SWSTST_REQ(error_code, arg_list);
		break;
	case DTECommand::SWSCAL_REQ:
		resp = SWSCAL_REQ(error_code, arg_list);
		break;
	case DTECommand::SWSSTATS_REQ:
		resp = SWSSTATS_REQ(error_code, arg_list);
		break;
	case DTECommand::GNSSBR_REQ:
		resp = GNSSBR_REQ(error_code, arg_list);
		break;
	case DTECommand::GNSSI_REQ:
		resp = GNSSI_REQ(error_code);
		break;
	case DTECommand::GNSSA_REQ:
		resp = GNSSA_REQ(error_code);
		break;
	case DTECommand::RTCW_REQ:
		resp = RTCW_REQ(error_code, arg_list);
		break;
	case DTECommand::SMDDFU_REQ:
		// VERSION action works for SMD/KIM2/LoRa builds; other actions SMD-only
		resp = SMDDFU_REQ(error_code, arg_list);
		break;
	case DTECommand::COMCW_REQ:
		resp = COMCW_REQ(error_code, arg_list);
		break;
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
	case DTECommand::SMDTST_REQ:
		resp = SMDTST_REQ(error_code, arg_list);
		break;
#endif
	case DTECommand::SATVF_REQ:
		resp = SATVF_REQ(error_code, arg_list);
		break;
	case DTECommand::SATDP_REQ:
		resp = SATDP_REQ(error_code, arg_list);
		break;
#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
	case DTECommand::LORATX_REQ:
		resp = LORATX_REQ(error_code, arg_list);
		break;
	case DTECommand::LORABR_REQ:
		resp = LORABR_REQ(error_code, arg_list);
		break;
#endif
#if !(defined(LORA_RAK3172) && (LORA_RAK3172 == 1)) && !(defined(ARGOS_SMD) && (ARGOS_SMD == 1))
	case DTECommand::KIMBR_REQ:
		resp = KIMBR_REQ(error_code, arg_list);
		break;
#endif
	default:
		break;
	}
	} catch (...) {
		DEBUG_ERROR("DTEHandler: unexpected exception in command handler");
		resp.clear();
	}

	return action;
}

#pragma GCC diagnostic pop

// KineisEventListener: async TX result notification
void DTEHandler::react(KineisEventTxComplete const& ) {
	DEBUG_INFO("DTEHandler::react: KineisEventTxComplete");
	if (m_doppler_cal_first_tx) {
		// First SATDP TX succeeded — respond OK, then schedule periodic TX
		LEDState::dispatch(SetLEDArgosTXComplete{});
		m_doppler_cal_first_tx = false;
		if (m_async_write)
			m_async_write(DTEEncoder::encode(DTECommand::SATDP_RESP, (int)DTEError::OK));
		schedule_doppler_cal_tx();
	} else if (m_doppler_cal_active) {
		// Periodic Doppler TX completed — schedule next one
		LEDState::dispatch(SetLEDArgosTXComplete{});
		schedule_doppler_cal_tx();
	} else {
		// Regular ARGOSTX/LORATX async response
#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
		DTECommand resp_cmd = m_lora_tx_active ? DTECommand::LORATX_RESP : DTECommand::ARGOSTX_RESP;
		m_lora_tx_active = false;
#else
		DTECommand resp_cmd = DTECommand::ARGOSTX_RESP;
#endif
		// Restore modulation after SATTX if it was temporarily switched
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
		if (m_sattx_needs_restore && smd_sat_instance) {
			ArgosConfig argos_config;
			configuration_store->get_argos_configuration(argos_config);
			std::string rconf;
			switch (m_sattx_restore_modulation) {
				case KineisModulation::LDK:   rconf = argos_config.radioconf_ldk; break;
				case KineisModulation::LDA2:  rconf = argos_config.radioconf_lda2; break;
				case KineisModulation::VLDA4: rconf = argos_config.radioconf_vlda4; break;
				default:                      rconf = argos_config.radioconf_lda2; break;
			}
			if (!rconf.empty())
				smd_sat_instance->switch_modulation(m_sattx_restore_modulation, rconf);
			m_sattx_needs_restore = false;
		}
#endif
		if (m_async_write) {
			std::string resp = DTEEncoder::encode(resp_cmd, (int)DTEError::OK);
			DEBUG_TRACE("DTEHandler::react: async responding: %s", resp.c_str());
			m_async_write(resp);
		}
	}
}

void DTEHandler::react(KineisEventDeviceError const& ) {
	DEBUG_WARN("DTEHandler::react: KineisEventDeviceError");
	if (m_doppler_cal_first_tx) {
		// First SATDP TX failed — respond error, abort calibration
		LEDState::dispatch(SetLEDArgosTXComplete{});
		m_doppler_cal_active = false;
		m_doppler_cal_first_tx = false;
		if (m_async_write)
			m_async_write(DTEEncoder::encode(DTECommand::SATDP_RESP, (int)DTEError::INCORRECT_DATA));
	} else if (m_doppler_cal_active) {
		// Periodic Doppler TX failed — retry on next cycle
		LEDState::dispatch(SetLEDArgosTXComplete{});
		DEBUG_WARN("DTEHandler: SATDP periodic TX failed | will retry");
		schedule_doppler_cal_tx();
	} else {
		// Regular ARGOSTX/LORATX async error response
#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
		DTECommand resp_cmd = m_lora_tx_active ? DTECommand::LORATX_RESP : DTECommand::ARGOSTX_RESP;
		m_lora_tx_active = false;
#else
		DTECommand resp_cmd = DTECommand::ARGOSTX_RESP;
#endif
		if (m_async_write) {
			std::string resp = DTEEncoder::encode(resp_cmd, (int)DTEError::INCORRECT_DATA);
			DEBUG_TRACE("DTEHandler::react: async responding: %s", resp.c_str());
			m_async_write(resp);
		}
	}
}

// KineisEventListener: handle power off to release subscription
void DTEHandler::react(KineisEventPowerOff const& ) {
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
	if (m_sat_device_active && !m_doppler_cal_active) {
		m_sat_device_active = false;
		smd_sat_instance->set_idle_timeout(3000);
		smd_sat_instance->unsubscribe(*this);
	}
#else
	// KIM2 (generic KineisDevice): release the ARGOSTX_REQ subscription once the
	// module powers down, so DTEHandler stops reacting to ArgosTxService's normal
	// TX events (which would otherwise emit spurious ARGOSTX_RESP).
	if (m_sat_device_active && !m_doppler_cal_active && kineis_device_instance) {
		m_sat_device_active = false;
		kineis_device_instance->set_idle_timeout(3000);
		kineis_device_instance->unsubscribe(*this);
	}
#endif
}

std::string DTEHandler::SATDP_REQ(int error_code, std::vector<BaseType>& arg_list) {
	(void)arg_list;

	if (error_code) {
		return DTEEncoder::encode(DTECommand::SATDP_RESP, error_code);
	}

	if (m_doppler_cal_active) {
		return DTEEncoder::encode(DTECommand::SATDP_RESP, (int)DTEError::INCORRECT_DATA);
	}

#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
	if (!smd_sat_instance) {
		return smd_not_available_error("SATDP_REQ", DTECommand::SATDP_RESP);
	}

	// Subscribe to satellite events if not already active
	if (!m_sat_device_active) {
		smd_sat_instance->subscribe(*this);
		m_sat_device_active = true;
	}
	smd_sat_instance->set_idle_timeout(0);  // Keep satellite powered on indefinitely

	// Build and send the first Doppler packet
	unsigned int size_bits;
	unsigned int batt_mv = battery_monitor ? battery_monitor->get_voltage() : 3700;
	bool is_lb = battery_monitor ? battery_monitor->is_battery_low() : false;
	KineisPacket packet = ArgosPacketBuilder::build_doppler_packet(batt_mv, is_lb, size_bits);

	DEBUG_INFO("DTEHandler::SATDP_REQ: sending first Doppler packet (%u bits)", size_bits);
	// SATDP bypasses ArgosTxService → the operational state machine never sees a
	// SERVICE_ACTIVE event for ARGOS_TX, so the LEDArgosTX (MAGENTA) dispatch in
	// gentracker.cpp is skipped. Dispatch it here so the bench operator sees the
	// visual cue on every Doppler-cal TX, same as a deployed Argos send.
	LEDState::dispatch(SetLEDArgosTX{});
	smd_sat_instance->send(KineisModulation::LDA2, packet, size_bits);

	m_doppler_cal_active = true;
	m_doppler_cal_first_tx = true;

	// No immediate response — wait for KineisEventTxComplete/DeviceError
	return {};
#else
	DEBUG_WARN("DTEHandler::SATDP_REQ: No satellite device available");
	return DTEEncoder::encode(DTECommand::SATDP_RESP, (int)DTEError::INCORRECT_DATA);
#endif
}

void DTEHandler::schedule_doppler_cal_tx() {
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
	ArgosConfig config;
	configuration_store->get_argos_configuration(config);
	unsigned int delay_ms = config.tx_interval_s * 1000;

	DEBUG_TRACE("DTEHandler::schedule_doppler_cal_tx: next TX in %u ms", delay_ms);

	system_scheduler->post_task_prio(
		[this]() {
			if (!m_doppler_cal_active || !smd_sat_instance) return;
			unsigned int size_bits;
			unsigned int batt_mv = battery_monitor ? battery_monitor->get_voltage() : 3700;
			bool is_lb = battery_monitor ? battery_monitor->is_battery_low() : false;
			KineisPacket packet = ArgosPacketBuilder::build_doppler_packet(batt_mv, is_lb, size_bits);
			DEBUG_TRACE("DTEHandler: SATDP periodic TX (%u bits)", size_bits);
			LEDState::dispatch(SetLEDArgosTX{});
			smd_sat_instance->send(KineisModulation::LDA2, packet, size_bits);
		},
		"SATDPPeriodicTx",
		Scheduler::DEFAULT_PRIORITY,
		delay_ms
	);
#endif
}
