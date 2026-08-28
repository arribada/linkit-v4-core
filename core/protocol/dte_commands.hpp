/**
 * @file dte_commands.hpp
 * @brief DTE command definitions — command enum, command map, sensor/DFU types.
 */

#pragma once

#include "base_types.hpp"


#define RESP_CMD_BASE 0x80

enum class DTECommand {
	PARML_REQ,
	PARMR_REQ,
	PARMW_REQ,
	PROFR_REQ,
	PROFW_REQ,
	PASPW_REQ,
	SECUR_REQ,
	DUMPM_REQ,
	DUMPD_REQ,
	RSTVW_REQ,
	RSTBW_REQ,
	FACTW_REQ,
	STATR_REQ,
	ERASE_REQ,
	SCALW_REQ,
	ARGOSTX_REQ,
	SCALR_REQ,
	SENSR_REQ,     // Sensor/GNSS read command with timeout
	PWRON_REQ,     // Power on/off components
	SWSST_REQ,     // SWS analog calibration status read
	SATDP_REQ,     // Satellite Doppler calibration (periodic TX until reset)
	GNSSI_REQ,     // GNSS device info (unique ID, SW/HW version)
	GNSSA_REQ,     // GNSS almanac file validation
	RTCW_REQ,      // RTC manual write (set time before GNSS fix)
	SWSTST_REQ,    // SWS test mode start/stop
	SWSCAL_REQ,    // SWS guided calibration (LED-assisted air/water measurement)
	SWSSTATS_REQ,  // SWS persistent diagnostic counters read / clear (audit 2026-05 R-MON-02)
	GNSSBR_REQ,    // GNSS UART bridge/passthrough (u-center access)
	GNSSBCKP_REQ,  // GNSS backup-cell charge mode (rail ON, M10 in PMREQ-backup)
	SMDDFU_REQ,    // Comm module command: VERSION (all builds), DFU actions (SMD only)
	COMCW_REQ,     // Comm module Continuous Wave test (SMD / LoRa; KIM2 not supported)
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
	SMDTST_REQ,  // SMD satellite module SPI applicative test
#endif
	SATVF_REQ,  // Satellite credentials verify (SMD + KIM2 + LoRa), optional force re-write
#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
	LORATX_REQ,  // LoRa test transmission
	LORABR_REQ,  // LoRa UART bridge/passthrough (RUI3 AT access)
#endif
#if !(defined(LORA_RAK3172) && (LORA_RAK3172 == 1)) && !(defined(ARGOS_SMD) && (ARGOS_SMD == 1))
	KIMBR_REQ,  // KIM2 UART bridge/passthrough (raw AT command access)
#endif
	__NUM_REQ,
	PARML_RESP = RESP_CMD_BASE,
	PARMR_RESP,
	PARMW_RESP,
	PROFR_RESP,
	PROFW_RESP,
	PASPW_RESP,
	SECUR_RESP,
	DUMPM_RESP,
	DUMPD_RESP,
	RSTVW_RESP,
	RSTBW_RESP,
	FACTW_RESP,
	STATR_RESP,
	ERASE_RESP,
	SCALW_RESP,
	ARGOSTX_RESP,
	SCALR_RESP,
	SENSR_RESP,     // Sensor/GNSS read response
	PWRON_RESP,     // Power on/off response
	SWSST_RESP,     // SWS analog calibration status response
	SATDP_RESP,     // Satellite Doppler calibration response
	GNSSI_RESP,     // GNSS device info response
	GNSSA_RESP,     // GNSS almanac file validation response
	RTCW_RESP,      // RTC manual write response
	SWSTST_RESP,    // SWS test mode response
	SWSCAL_RESP,    // SWS guided calibration response
	SWSSTATS_RESP,  // SWS persistent diagnostic counters response
	GNSSBR_RESP,    // GNSS UART bridge/passthrough response
	GNSSBCKP_RESP,  // GNSS backup-cell charge mode response
	SMDDFU_RESP,    // Comm module response (VERSION: all builds, DFU: SMD only)
	COMCW_RESP,     // Comm module Continuous Wave response
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
	SMDTST_RESP,  // SMD satellite module SPI applicative test response
#endif
	SATVF_RESP,  // Satellite credentials verify response (SMD + KIM2 + LoRa)
#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
	LORATX_RESP,  // LoRa test transmission response
	LORABR_RESP,  // LoRa UART bridge/passthrough response
#endif
#if !(defined(LORA_RAK3172) && (LORA_RAK3172 == 1)) && !(defined(ARGOS_SMD) && (ARGOS_SMD == 1))
	KIMBR_RESP,  // KIM2 UART bridge/passthrough response
#endif
	__NUM_RESP
};

// Sensor types for SENSR command (bitmask)
enum class SensrType : unsigned int {
	BATTERY = 0x01,     // Read battery voltage and SOC
	PRESSURE = 0x02,    // Read pressure sensor
	GNSS = 0x04,        // Trigger GNSS acquisition with timeout
	ACCEL = 0x08,       // Read accelerometer (BMA400)
	THERMISTOR = 0x10,  // Read thermistor temperature
	ALL = 0x1F          // Read all sensors
};

// Component power types (for PWRON command)
enum class ComponentPower {
	ALL = 0,        // All components
	GNSS = 1,       // GNSS module
	SENSORS = 2,    // Sensors power
	SATELLITE = 3,  // Satellite module
	OFF = 4         // Power off all components
};

// SMD DFU action types (for SMDDFU command)
enum class SmdDfuAction {
	ENTER = 0,   // Enter DFU mode
	EXIT = 1,    // Exit DFU mode (jump to application)
	STATUS = 2,  // Get DFU status
	UPDATE = 3,  // Perform firmware update (requires firmware data)
	INFO = 4,    // Get bootloader info
	VERSION = 5  // Get application firmware version
};

struct DTECommandMap {
	std::string name;
	DTECommand command;
	std::vector<BaseMap> prototype;
	unsigned int min_args = 0;  // 0 = all args required (default); >0 = trailing args are optional
};

extern const DTECommandMap command_map[];
extern const size_t command_map_size;
