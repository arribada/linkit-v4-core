/**
 * @file dte_handler.hpp
 * @brief DTE command handler — dispatches parsed commands to config store, sensors, GPS, satellite.
 */

#pragma once

#include <algorithm>
#include <functional>
#include <string>
#include <map>

#include "dte_protocol.hpp"
#include "config_store.hpp"
#include "memory_access.hpp"
#include "timeutils.hpp"
#include "calibration.hpp"
#include "sensor.hpp"
#include "battery.hpp"
#include "gpio.hpp"
#include "bsp.hpp"
#include "kineis_device.hpp"
#if ENABLE_SWS_ANALOG
#include "sws_analog_service.hpp"
#endif
#include "gps.hpp"
#include "rtc.hpp"

#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
#include "smd_sat.hpp"
extern SmdSat *smd_sat_instance;
#endif
extern KineisDevice *kineis_device_instance;

#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
#include "lora_rak3172.hpp"
extern LoRaDevice *lora_device_instance;
#endif

#if !(defined(LORA_RAK3172) && (LORA_RAK3172 == 1)) && !(defined(ARGOS_SMD) && (ARGOS_SMD == 1))
#include "kim2.hpp"
extern KIM2Device *kim2_device_instance;
#endif

// External sensor access for SENSR command
extern BatteryMonitor *battery_monitor;
extern GPSDevice *gps_device;
extern RTC *rtc;
class GPSService; // forward decl — full definition included in gps_service.hpp where needed
extern GPSService *gps_service;

// This governs the maximum number of log entries we can read out in a single request
#define DTE_HANDLER_MAX_LOG_DUMP_ENTRIES          8U

enum class DTEAction {
	NONE,    // Default action is none
	AGAIN,   // More data is remaining in a multi-message response, call the handler again with the same input message
	RESET,   // Deferred action since DTE must respond first before reset can be performed
	FACTR,   // Deferred action since DTE must respond first before factory reset can be performed
	SECUR,   // DTE service must be notified when SECUR is requested to grant privileges for OTA FW commands
	CONFIG_UPDATED,  // Notified on a successful PARMW
};

enum class DTEError {
	OK = 0,
	INCORRECT_COMMAND = 1,
	NO_LENGTH_DELIMITER = 2,
	NO_DATA_DELIMITER = 3,
	DATA_LENGTH_MISMATCH = 4,
	INCORRECT_DATA = 5,
	// Extended error codes for better diagnostics
	PARAM_KEY_UNRECOGNISED = 6,
	VALUE_OUT_OF_RANGE = 7,
	MISSING_ARGUMENT = 8,
	BAD_FORMAT = 9,
	MESSAGE_TOO_LARGE = 10,
	UNEXPECTED_ARGUMENT = 11,
	INVALID_ACCESS_CODE = 12
};

// The DTEHandler requires access to the following system objects that are extern declared
extern ConfigurationStore *configuration_store;
extern MemoryAccess *memory_access;

class DTEHandler : public KineisEventListener, public GPSEventListener {
private:
	// Tables
	static inline std::map<unsigned int, std::string> m_logger_dump = {
		{(unsigned int)BaseLogDType::INTERNAL, "system.log"},
		{(unsigned int)BaseLogDType::GNSS_SENSOR, "sensor.log"},
		{(unsigned int)BaseLogDType::ALS_SENSOR, "ALS"},
		{(unsigned int)BaseLogDType::PH_SENSOR, "PH"},
		{(unsigned int)BaseLogDType::RTD_SENSOR, "RTD"},
		{(unsigned int)BaseLogDType::CDT_SENSOR, "CDT"},
		{(unsigned int)BaseLogDType::CAM_SENSOR, "CAM"},
		{(unsigned int)BaseLogDType::AXL_SENSOR, "AXL"},
		{(unsigned int)BaseLogDType::PRESSURE_SENSOR, "PRESSURE"},
		{(unsigned int)BaseLogDType::THERMISTOR_SENSOR, "THERMISTOR"},
		{(unsigned int)BaseLogDType::TSYS01_SENSOR, "TSYS01"},
		{(unsigned int)BaseLogDType::SWS_LOG, "SWS"},
		{(unsigned int)BaseLogDType::MORTALITY, "MORTALITY"},
	};
	static inline std::map<unsigned int, std::string> m_logger_erase = {
		{(unsigned int)BaseEraseType::GNSS_SENSOR, "sensor.log"},
		{(unsigned int)BaseEraseType::SYSTEM, "system.log"},
		{(unsigned int)BaseEraseType::ALS_SENSOR, "ALS"},
		{(unsigned int)BaseEraseType::PH_SENSOR, "PH"},
		{(unsigned int)BaseEraseType::RTD_SENSOR, "RTD"},
		{(unsigned int)BaseEraseType::CDT_SENSOR, "CDT"},
		{(unsigned int)BaseEraseType::CAM_SENSOR, "CAM"},
		{(unsigned int)BaseEraseType::AXL_SENSOR, "AXL"},
		{(unsigned int)BaseEraseType::PRESSURE_SENSOR, "PRESSURE"},
		{(unsigned int)BaseEraseType::THERMISTOR_SENSOR, "THERMISTOR"},
		{(unsigned int)BaseEraseType::TSYS01_SENSOR, "TSYS01"},
		{(unsigned int)BaseEraseType::SWS_LOG, "SWS"},
		{(unsigned int)BaseEraseType::MORTALITY, "MORTALITY"},
	};
	static inline std::map<unsigned int, std::string> m_scalx = {
		{0, "AXL"},
		{1, "PRS"},
		{2, "ALS"},
		{3, "PH"},
		{4, "RTD"},
		{5, "CDT"},
		{6, "MCP47X6"},
		{7, "THERMISTOR"},
		{8, "SWS"},
	};
	unsigned int m_dumpd_NNN = 0;
	unsigned int m_dumpd_mmm = 0;
	unsigned int m_dumpd_d_type = 0;  ///< Track current dump type to detect mid-stream changes
	bool m_sat_device_active = false;
	bool m_lora_tx_active = false;         ///< Current async TX is LoRa (vs Argos/SMD)
	bool m_doppler_cal_active = false;     ///< Periodic Doppler TX mode active (until reset)
	bool m_doppler_cal_first_tx = false;   ///< Waiting for first TX result
	bool m_sattx_needs_restore = false;    ///< SATTX temporarily switched modulation
	KineisModulation m_sattx_restore_modulation = KineisModulation::LDA2;
	bool m_gnssi_pending = false;          ///< Waiting for GNSS device info (autonomous GNSSI)
	bool m_gps_subscribed = false;         ///< Subscribed to GPS events
	std::function<void(const std::string&)> m_async_write;

	void schedule_doppler_cal_tx();

	// Helper: read params by filter character (used by PARMR_REQ and STATR_REQ)
	static std::string read_params_by_filter(int error_code, std::vector<ParamID>& params, const std::vector<std::string>& rejected_keys, char filter_char, DTECommand resp_cmd);

#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
	// Helper: return error response when SMD instance is not available
	static std::string smd_not_available_error(const char* func_name, DTECommand resp_cmd);
#endif

public:
	DTEHandler();
	virtual ~DTEHandler();

	void set_async_write(std::function<void(const std::string&)> fn);
	void reset_state();

	static std::string PARML_REQ(int error_code);
	static std::string PARMW_REQ(int error_code, std::vector<ParamValue>& param_values, std::vector<std::string>& rejected_keys, DTEAction& action);
	static std::string PARMR_REQ(int error_code, std::vector<ParamID>& params, const std::vector<std::string>& rejected_keys);
	static std::string STATR_REQ(int error_code, std::vector<ParamID>& params, const std::vector<std::string>& rejected_keys);
	static std::string PROFW_REQ(int error_code, std::vector<BaseType>& arg_list);
	static std::string PROFR_REQ(int error_code);
	static std::string SECUR_REQ(int error_code, std::vector<BaseType>& arg_list);
	static std::string RSTVW_REQ(int error_code, std::vector<BaseType>& arg_list);
	static std::string RSTBW_REQ(int error_code);
	static std::string FACTW_REQ(int error_code);
	static std::string DUMPM_REQ(int error_code, std::vector<BaseType>& arg_list);
	static std::string PASPW_REQ(int error_code, std::vector<BaseType>& arg_list);
	std::string DUMPD_REQ(int error_code, std::vector<BaseType>& arg_list, DTEAction& action);
	static std::string ERASE_REQ(int error_code, std::vector<BaseType>& arg_list);
	static std::string SCALW_REQ(int error_code, std::vector<BaseType>& arg_list);
	static std::string SCALR_REQ(int error_code, std::vector<BaseType>& arg_list);
	std::string ARGOSTX_REQ(int error_code, std::vector<BaseType>& arg_list);
	static std::string SENSR_REQ(int error_code, std::vector<BaseType>& arg_list);
	std::string PWRON_REQ(int error_code, std::vector<BaseType>& arg_list);
	static std::string GNSSBCKP_REQ(int error_code, std::vector<BaseType>& arg_list);
	static std::string SWSST_REQ(int error_code);
	std::string SWSTST_REQ(int error_code, std::vector<BaseType>& arg_list);
	std::string SWSCAL_REQ(int error_code, std::vector<BaseType>& arg_list);
	static std::string SWSSTATS_REQ(int error_code, std::vector<BaseType>& arg_list);
	std::string GNSSBR_REQ(int error_code, std::vector<BaseType>& arg_list);
	std::string GNSSI_REQ(int error_code);
	static std::string GNSSA_REQ(int error_code);
	static std::string RTCW_REQ(int error_code, std::vector<BaseType>& arg_list);

	std::string SATDP_REQ(int error_code, std::vector<BaseType>& arg_list);

	static std::string SMDDFU_REQ(int error_code, std::vector<BaseType>& arg_list);
	static std::string COMCW_REQ(int error_code, std::vector<BaseType>& arg_list);
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
	static std::string SMDTST_REQ(int error_code, std::vector<BaseType>& arg_list);
#endif
	std::string SATVF_REQ(int error_code, std::vector<BaseType>& arg_list);

#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
	std::string LORATX_REQ(int error_code, std::vector<BaseType>& arg_list);
	std::string LORABR_REQ(int error_code, std::vector<BaseType>& arg_list);
#endif
#if !(defined(LORA_RAK3172) && (LORA_RAK3172 == 1)) && !(defined(ARGOS_SMD) && (ARGOS_SMD == 1))
	std::string KIMBR_REQ(int error_code, std::vector<BaseType>& arg_list);
#endif

	DTEAction handle_dte_message(const std::string& req, std::string& resp);

	// KineisEventListener overrides
	void react(KineisEventTxComplete const&) override;
	void react(KineisEventDeviceError const&) override;
	void react(KineisEventPowerOff const&) override;

	// GPSEventListener overrides
	void react(const GPSEventDeviceInfoReady&) override;
	void react(const GPSEventError&) override;
};
