/**
 * @file dte_params.cpp
 * @brief DTE parameter map — 220+ parameters with keys, encoding, constraints, permissions.
 */

#include "dte_params.hpp"

// Helper for EXTERNAL_WAKEUP which is defined as a flag (no value)
#ifdef EXTERNAL_WAKEUP
#define HAS_EXTERNAL_WAKEUP true
#else
#define HAS_EXTERNAL_WAKEUP false
#endif

// Helper for BOARD_RSPB
#ifdef BOARD_RSPB
#define HAS_BOARD_RSPB true
#else
#define HAS_BOARD_RSPB false
#endif

// ARGOS_SMD is only defined on targets with SMD satellite module
#ifndef ARGOS_SMD
#define ARGOS_SMD 0
#endif

// LORA_RAK3172 is only defined on targets with LoRa RAK3172 module
#ifndef LORA_RAK3172
#define LORA_RAK3172 0
#endif

// one line per parameter; column alignment is how this table is read
// clang-format off
const BaseMap param_map[] = {
	{ "ARGOS_DECID", "IDP12", BaseEncoding::UINT, 0U, 0xFFFFFFFFU, {}, true, true },
	{ "ARGOS_HEXID", "IDT06", BaseEncoding::HEXADECIMAL, 0U, 0xFFFFFFFFU, {}, true, true },
	{ "DEVICE_MODEL", "IDT02", BaseEncoding::TEXT, "", "", {}, true, true },
	{ "FW_APP_VERSION", "IDT03", BaseEncoding::TEXT, "", "", {}, true, false },
	{ "LAST_TX", "ART01", BaseEncoding::DATESTRING, 0, 0, {}, true, false },
	{ "TX_COUNTER", "ART02", BaseEncoding::UINT, 0U, 0U, {}, true, false },
	{ "BATT_SOC", "POT03", BaseEncoding::UINT, 0U, 100U, {}, true, false },
	{ "_RESERVED_7", "", BaseEncoding::UINT, 0, 0, {}, false, false },  // Obsolete: was LAST_FULL_CHARGE_DATE — never written by firmware, no reliable hardware "charge complete" signal on this platform; user-side bookkeeping if needed
	{ "PROFILE_NAME", "IDP11", BaseEncoding::TEXT, "", "", {}, true, true },
	{ "_RESERVED_9", "", BaseEncoding::UINT, 0, 0, {}, false, false },  // Reserved: AOP status is part of PASPW allcast data, not a DTE param
	{ "ARGOS_AOP_DATE", "ART03", BaseEncoding::DATESTRING, 0, 0, {}, true, false },
	{ "_RESERVED_ARGOS_FREQ", "", BaseEncoding::ARGOSFREQ, 401.6200, 401.6800, {}, false, false },  // Obsolete: RADIOCONF controls frequency
	{ "_RESERVED_ARGOS_POWER", "", BaseEncoding::ARGOSPOWER, 0, 0, { 0U, 1U, 2U, 3U }, false, false },  // Obsolete: RADIOCONF controls power
	{ "TR_NOM", "ARP05", BaseEncoding::UINT, 30U, 1200U, {}, true, true },
	{ "ARGOS_MODE", "ARP01", BaseEncoding::ARGOSMODE, 0, 0, { 0U, 1U, 2U, 3U, 4U, 5U }, true, true },
	{ "NTRY_PER_MESSAGE", "ARP19", BaseEncoding::UINT, 0U, 86400U, {}, true, true },
	{ "DUTY_CYCLE", "ARP18", BaseEncoding::UINT, 0U, 0xFFFFFFU, {}, true, true },
	{ "GNSS_EN", "GNP01", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	{ "DLOC_ARG_NOM", "ARP11", BaseEncoding::AQPERIOD, 0, 0, { 0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U, 10U, 11U, 12U, 13U, 14U, 15U }, true, true },
	{ "ARGOS_DEPTH_PILE", "ARP16", BaseEncoding::DEPTHPILE, 0U, 0U, {1U, 2U, 3U, 4U, 8U, 12U, 16U, 20U, 24U}, true, true },
	{ "_RESERVED_20", "", BaseEncoding::UINT, 0, 0, {}, false, false },  // Was GPS_CONST_SELECT — replaced by GNP40
	{ "_RESERVED_21", "", BaseEncoding::UINT, 0, 0, {}, false, false },  // Was GLONASS_CONST_SELECT — replaced by GNP40
	{ "GNSS_HDOPFILT_EN", "GNP02", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	{ "GNSS_HDOPFILT_THR", "GNP03", BaseEncoding::UINT, 2U, 15U, {}, true, true },
	{ "GNSS_ACQ_TIMEOUT", "GNP05", BaseEncoding::UINT, 10U, 600U, {}, true, true },
	{ "GNSS_NTRY", "GNP04", BaseEncoding::UINT, 0U, 255U, {}, true, true },
	{ "UNDERWATER_EN", "UNP01", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	{ "DRY_TIME_BEFORE_TX", "UNP02", BaseEncoding::UINT, 0U, 0xFFFFFFFFU, {}, true, true },
	{ "SAMPLING_UNDER_FREQ", "UNP03", BaseEncoding::FLOAT, (double)0.1, (double)86400.0, {}, true, true },
	{ "LB_EN", "LBP01", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	{ "LB_THRESHOLD", "LBP02", BaseEncoding::UINT, 0U, 100U, {}, true, true },
	{ "_RESERVED_LB_ARGOS_POWER", "", BaseEncoding::ARGOSPOWER, 0, 0, { 0U, 1U, 2U, 3U }, false, false },  // Obsolete: RADIOCONF controls power
	{ "TR_LB", "ARP06", BaseEncoding::UINT, 30U, 1200U, {}, true, true },
	{ "LB_ARGOS_MODE", "LBP04", BaseEncoding::ARGOSMODE, 0U, 0U, { 0U, 1U, 2U, 3U, 4U, 5U }, true, true },
	{ "LB_ARGOS_DUTY_CYCLE", "LBP05", BaseEncoding::UINT, 0U, 0xFFFFFFU, {}, true, true },
	{ "LB_GNSS_EN", "LBP06", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	{ "DLOC_ARG_LB", "ARP12", BaseEncoding::AQPERIOD, 0, 0, { 0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U, 10U, 11U, 12U, 13U, 14U, 15U }, true, true },
	{ "LB_GNSS_HDOPFILT_THR", "LBP07", BaseEncoding::UINT, 2U, 15U, {}, true, true },
	{ "LB_ARGOS_DEPTH_PILE", "LBP08", BaseEncoding::DEPTHPILE, 0U, 0U, {1U, 2U, 3U, 4U, 8U, 12U, 16U, 20U, 24U}, true, true },
	{ "LB_GNSS_ACQ_TIMEOUT", "LBP09", BaseEncoding::UINT, 10U, 600U, {}, true, true },
	{ "SAMPLING_SURF_FREQ", "UNP04", BaseEncoding::FLOAT, (double)0.1, (double)86400.0, {}, true, true },
    { "PP_MIN_ELEVATION", "PPP01", BaseEncoding::FLOAT, 0.0, 90.0, {}, true, true },
	{ "PP_MAX_ELEVATION", "PPP02", BaseEncoding::FLOAT, 0.0, 90.0, {}, true, true },
    { "PP_MIN_DURATION", "PPP03", BaseEncoding::UINT, 20U, 3600U, {}, true, true },
	{ "PP_MAX_PASSES", "PPP04", BaseEncoding::UINT, 1U, 10000U, {}, true, true },
	{ "PP_LINEAR_MARGIN", "PPP05", BaseEncoding::UINT, 1U, 3600U, {}, true, true },
	{ "PP_COMP_STEP", "PPP06", BaseEncoding::UINT, 1U, 1000U, {}, true, true },
	{ "GNSS_COLD_ACQ_TIMEOUT", "GNP09", BaseEncoding::UINT, 10U, 600U, {}, true, true },
	{ "GNSS_FIX_MODE", "GNP10", BaseEncoding::GNSSFIXMODE, 0U, 0U, {1U, 2U, 3U}, true, true },
	{ "GNSS_DYN_MODEL", "GNP11", BaseEncoding::GNSSDYNMODEL, 0U, 0U, {0U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U, 10U}, true, true },
	{ "GNSS_HACCFILT_EN", "GNP20", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	{ "GNSS_HACCFILT_THR", "GNP21", BaseEncoding::UINT, 0U, 100000U, {}, true, true },
	{ "GNSS_MIN_NUM_FIXES", "GNP22", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, true, true },
	{ "GNSS_COLD_START_RETRY_PERIOD", "GNP23", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, true, true },
	{ "ARGOS_TIME_SYNC_BURST_EN", "ARP30", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	{ "LED_MODE", "LDP01", BaseEncoding::LEDMODE, 0U, 0U, {}, true, true },
	{ "ARGOS_TX_JITTER_EN", "ARP31", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	{ "ARGOS_RX_EN", "ARP32", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	{ "ARGOS_RX_MAX_WINDOW", "ARP33", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, true, true },
	{ "ARGOS_RX_AOP_UPDATE_PERIOD", "ARP34", BaseEncoding::UINT, 0U, 0xFFFFFFFFU, {}, true, true },
	{ "ARGOS_RX_COUNTER", "ART10", BaseEncoding::UINT, 0U, 0U, {}, true, false },
	{ "ARGOS_RX_TIME", "ART11", BaseEncoding::UINT, 0U, 0U, {}, true, false },
	{ "GNSS_ASSISTNOW_EN", "GNP24", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	{ "LB_GNSS_HACCFILT_THR", "LBP10", BaseEncoding::UINT, 0U, 100000U, {}, true, true },
	{ "LB_NTRY_PER_MESSAGE", "LBP11", BaseEncoding::UINT, 0U, 86400U, {}, true, true },

	//////////////////////////
	// ZONE FILE
	//////////////////////////
		// BaseZoneType has exactly one member, CIRCLE = 1, and decode_zone_type()
	// throws on anything else. Listing 0 as permitted advertised a zone type
	// that does not exist: a host reading this table would offer it, and the
	// write would then be rejected by the decoder for no reason the operator
	// can see.
	{ "ZONE_TYPE", "ZOP01", BaseEncoding::ZONETYPE, 0, 0, { 1U }, true, true },
	{ "ZONE_ENABLE_OUT_OF_ZONE_DETECTION_MODE", "ZOP04", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	{ "ZONE_ENABLE_ACTIVATION_DATE", "ZOP05", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	{ "ZONE_ACTIVATION_DATE", "ZOP06", BaseEncoding::DATESTRING, 0, 0, {}, true, true },
	{ "ZONE_ARGOS_DEPTH_PILE", "ZOP08", BaseEncoding::DEPTHPILE, 0U, 0U, {1U, 2U, 3U, 4U, 8U, 12U, 16U, 20U, 24U}, true, true },
	{ "_RESERVED_70", "", BaseEncoding::ARGOSPOWER, 0U, 0U, { 0U, 1U, 2U, 3U }, false, false },
	{ "ZONE_ARGOS_REPETITION_SECONDS", "ZOP10", BaseEncoding::UINT, 30U, 1200U, {}, true, true },
	{ "ZONE_ARGOS_MODE", "ZOP11", BaseEncoding::ARGOSMODE, 0U, 0U, { 0U, 1U, 2U, 3U, 4U, 5U }, true, true },
	{ "ZONE_ARGOS_DUTY_CYCLE", "ZOP12", BaseEncoding::UINT, 0U, 0xFFFFFFU, {}, true, true },
	{ "ZONE_ARGOS_NTRY_PER_MESSAGE", "ZOP13", BaseEncoding::UINT, 0U, 86400U, {}, true, true },
	{ "ZONE_GNSS_DELTA_ARG_LOC_ARGOS_SECONDS", "ZOP14", BaseEncoding::AQPERIOD, 0, 0, { 0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U, 10U, 11U, 12U, 13U, 14U, 15U }, true, true },
	{ "ZONE_GNSS_HDOPFILT_THR", "ZOP15", BaseEncoding::UINT, 2U, 15U, {}, true, true },
	{ "ZONE_GNSS_HACCFILT_THR", "ZOP16", BaseEncoding::UINT, 0U, 100000U, {}, true, true },
	{ "ZONE_GNSS_ACQ_TIMEOUT", "ZOP17", BaseEncoding::UINT, 10U, 600U, {}, true, true },
	{ "ZONE_CENTER_LONGITUDE", "ZOP18", BaseEncoding::FLOAT, -180.0, 180.0, {}, true, true },
	{ "ZONE_CENTER_LATITUDE", "ZOP19", BaseEncoding::FLOAT, -90.0, 90.0, {}, true, true },
	{ "ZONE_RADIUS", "ZOP20", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, true, true },

	//////////////////////////
	// CERTIFICATION
	//////////////////////////
	{ "CERT_TX_ENABLE", "CTP01", BaseEncoding::BOOLEAN, 0, 0, { }, true, true },
	{ "CERT_TX_PAYLOAD", "CTP02", BaseEncoding::TEXT, "", "", {}, true, true },
	{ "CERT_TX_MODULATION", "CTP03", BaseEncoding::MODULATION, 0, 0, { 0U, 1U, 2U }, true, true },
	{ "CERT_TX_REPETITION", "CTP04", BaseEncoding::UINT, 2U, 0xFFFFFFFFU, { }, true, true },

	// HW version identification
	{ "HW_VERSION", "IDT04", BaseEncoding::TEXT, "", "", {}, true, false },

	// Battery voltage
	{ "BATT_VOLTAGE", "POT06", BaseEncoding::FLOAT, 0.0, 12.0, {}, true, false },

	// [88-91] TPL5111 power management parameters (slots always reserved)
	{ "SHUTDOWN_TIMER", "PWP01", BaseEncoding::UINT, 0U, 86400U, {}, HAS_EXTERNAL_WAKEUP, true },
	{ "BOOT_COUNTER", "PWP02", BaseEncoding::UINT, 0U, 0U, {}, HAS_EXTERNAL_WAKEUP, false },
	{ "BOOT_COUNTER_MODULO", "PWP03", BaseEncoding::UINT, 2U, 1000U, {}, HAS_EXTERNAL_WAKEUP, true },
	{ "WAKEUP_PERIOD", "PWP04", BaseEncoding::UINT, 0U, 86400U, {}, HAS_EXTERNAL_WAKEUP, true },

	// TCXO warmup period
	{ "ARGOS_TCXO_WARMUP_TIME", "ARP35", BaseEncoding::UINT, 0U, 30U, {}, true, true },

	{ "DEVICE_DECID", "IDT10", BaseEncoding::UINT, 0U, 0U, {}, true, false },
	{ "GNSS_TRIGGER_ON_SURFACED", "GNP25", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	{ "GNSS_TRIGGER_ON_AXL_WAKEUP", "GNP26", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	{ "__RESERVED", "", BaseEncoding::UWDETECTSOURCE, 0, 0, {}, false, false }, // was UNP10 UNDERWATER_DETECT_SOURCE (only SWS supported)
	{ "UNDERWATER_DETECT_THRESH", "UNP11", BaseEncoding::FLOAT, (double)0.0, (double)0.0, {}, true, true },

	// [98-100] pH sensor (slots always reserved)
	{ "PH_SENSOR_ENABLE", "PHP01", BaseEncoding::BOOLEAN, 0, 0, {}, ENABLE_PH_SENSOR, true },
	{ "PH_SENSOR_PERIODIC", "PHP02", BaseEncoding::UINT, 0U, 0U, {}, ENABLE_PH_SENSOR, true },
	{ "PH_SENSOR_VALUE", "PHP03", BaseEncoding::FLOAT, (double)0.0, (double)0.0, {}, ENABLE_PH_SENSOR, true },
	// [101-103] Sea temperature sensor (slots always reserved)
	{ "SEA_TEMP_SENSOR_ENABLE", "STP01", BaseEncoding::BOOLEAN, 0, 0, {}, ENABLE_SEA_TEMP_SENSOR, true },
	{ "SEA_TEMP_SENSOR_PERIODIC", "STP02", BaseEncoding::UINT, 0U, 0U, {}, ENABLE_SEA_TEMP_SENSOR, true },
	{ "SEA_TEMP_SENSOR_VALUE", "STP03", BaseEncoding::FLOAT, (double)0.0, (double)0.0, {}, ENABLE_SEA_TEMP_SENSOR, true },
	// [104-106] ALS sensor (slots always reserved)
	{ "ALS_SENSOR_ENABLE", "LTP01", BaseEncoding::BOOLEAN, 0, 0, {}, ENABLE_ALS_SENSOR, true },
	{ "ALS_SENSOR_PERIODIC", "LTP02", BaseEncoding::UINT, 0U, 0U, {}, ENABLE_ALS_SENSOR, true },
	{ "ALS_SENSOR_VALUE", "LTP03", BaseEncoding::FLOAT, (double)0.0, (double)0.0, {}, ENABLE_ALS_SENSOR, true },
	// [107-111] CDT sensor (slots always reserved)
	{ "CDT_SENSOR_ENABLE", "CDP01", BaseEncoding::BOOLEAN, 0, 0, {}, ENABLE_CDT_SENSOR, true },
	{ "CDT_SENSOR_PERIODIC", "CDP02", BaseEncoding::UINT, 0U, 0U, {}, ENABLE_CDT_SENSOR, true },
	{ "CDT_SENSOR_CONDUCTIVITY", "CDP03", BaseEncoding::FLOAT, (double)0.0, (double)0.0, {}, ENABLE_CDT_SENSOR, true },
	{ "CDT_SENSOR_DEPTH", "CDP04", BaseEncoding::FLOAT, (double)0.0, (double)0.0, {}, ENABLE_CDT_SENSOR, true },
	{ "CDT_SENSOR_TEMPERATURE", "CDP05", BaseEncoding::FLOAT, (double)0.0, (double)0.0, {}, ENABLE_CDT_SENSOR, true },
	// [112-116] Thermistor sensor (slots always reserved)
	{ "THERMISTOR_SENSOR_ENABLE", "THP01", BaseEncoding::BOOLEAN, 0, 0, {}, ENABLE_THERMISTOR_SENSOR, true },
	{ "THERMISTOR_SENSOR_PERIODIC", "THP02", BaseEncoding::UINT, 0U, 0U, {}, ENABLE_THERMISTOR_SENSOR, true },
	{ "THERMISTOR_SENSOR_VALUE", "THP03", BaseEncoding::FLOAT, (double)0.0, (double)0.0, {}, ENABLE_THERMISTOR_SENSOR, true },
	{ "THERMISTOR_SENSOR_WAKEUP_THRESH", "THP04", BaseEncoding::FLOAT, (double)0.0, (double)0.0, {}, ENABLE_THERMISTOR_SENSOR, true },
	{ "THERMISTOR_SENSOR_WAKEUP_SAMPLES", "THP05", BaseEncoding::UINT, 0U, 0xFFFFFFFFU, {}, ENABLE_THERMISTOR_SENSOR, true },
	// [117] External LED
	// [117] Reserved: was EXT_LED_MODE / LDP02 — external LED indicator from
	// the Icoteq Horizon / Artic-R2 era. EXT_LED_PIN is not wired on LinkIt V4
	// or RSPB, so the param had no visible effect. Slot kept reserved (hidden
	// from DTE) to preserve the param array indexing for flash-persisted
	// configs from earlier firmware.
	{ "_RESERVED_117", "", BaseEncoding::UINT, 0, 0, {}, false, false },
	// [118-123] Accelerometer sensor (slots always reserved)
	{ "AXL_SENSOR_ENABLE", "AXP01", BaseEncoding::BOOLEAN, 0, 0, {}, ENABLE_AXL_SENSOR, true },
	{ "AXL_SENSOR_PERIODIC", "AXP02", BaseEncoding::UINT, 0U, 0U, {}, ENABLE_AXL_SENSOR, true },
	{ "AXL_SENSOR_WAKEUP_THRESH", "AXP03", BaseEncoding::FLOAT, (double)0.0, (double)8.0, {}, ENABLE_AXL_SENSOR, true },
	{ "AXL_SENSOR_WAKEUP_SAMPLES", "AXP04", BaseEncoding::UINT, 0U, 50U, {}, ENABLE_AXL_SENSOR, true },
		// 0..3 = BMA400_RANGE_2G/4G/8G/16G. The upper bound was 4, one past the
	// end: the value is a REGISTER INDEX, not a g-force. range_to_g() maps
	// 0..3 to {2,4,8,16} and falls back to 4 for anything above, so AXP08=4
	// designated a range that does not exist, silently aliased 4 g, and fed
	// calculate_threshold_reg() an LSB computed for (1 << 6) -- putting the
	// wakeup threshold register out by a factor of four.
	{ "AXL_SENSOR_MEASUREMENT_RANGE", "AXP08", BaseEncoding::UINT, 0U, 3U, {}, ENABLE_AXL_SENSOR, true },
	{ "AXL_SENSOR_POWER_MODE", "AXP09", BaseEncoding::UINT, 0U, 2U, {}, ENABLE_AXL_SENSOR, true },
	// [124-125] Pressure sensor (slots always reserved)
	{ "PRESSURE_SENSOR_ENABLE", "PRP01", BaseEncoding::BOOLEAN, 0, 0, {}, ENABLE_PRESSURE_SENSOR, true },
	{ "PRESSURE_SENSOR_PERIODIC", "PRP02", BaseEncoding::UINT, 0U, 0U, {}, ENABLE_PRESSURE_SENSOR, true },

	// DEBUG
	{ "DEBUG_OUTPUT_MODE", "DBP01", BaseEncoding::DEBUGMODE, 0, 0, {}, true, true },

	// GNSS ANO
	{ "GNSS_ASSISTNOW_OFFLINE_EN", "GNP27", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },

	// Extended UW parameters
	{ "UW_MAX_SAMPLES", "UNP05", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, true, true },
	{ "UW_MIN_DRY_SAMPLES", "UNP06", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, true, true },
	{ "UW_SAMPLE_GAP", "UNP07", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, true, true },
	{ "UW_PIN_SAMPLE_DELAY_US", "UNP08", BaseEncoding::UINT, 50U, 30000U, {}, true, true },

	// [132] GNSS constellation bitmask, [133] AssistNow orbit max error
	{ "GNSS_CONSTELLATION_MASK", "GNP40", BaseEncoding::UINT, 0x01U, 0x3FU, {}, true, true },
	{ "GNSS_ORBMAXERR", "GNP41", BaseEncoding::UINT, 10U, 1000U, {}, true, true },
	{ "SWS_ANALOG_HYSTERESIS", "UNP22", BaseEncoding::UINT, 0U, 50U, {}, true, true },
	{ "SWS_ANALOG_CALIB_INTERVAL", "UNP23", BaseEncoding::UINT, 60U, 0xFFFFFFFFU, {}, true, true },
	{ "UW_MAX_DIVE_TIME", "UNP24", BaseEncoding::UINT, 0U, 0xFFFFFFFFU, {}, true, true },
	{ "UW_MIN_SURFACE_TIME", "UNP25", BaseEncoding::UINT, 0U, 0xFFFFFFFFU, {}, true, true },

	// Dive mode
	{ "UW_DIVE_MODE_ENABLE", "UNP12", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	// Seconds, turned into a millisecond scheduler delay: above 4294967 the
	// `* 1000` overflowed an unsigned int and a dive scheduled weeks out
	// engaged instantly, pausing the reed switch with it. One day, like the
	// other delays that bound how long a sealed device may stay silent.
	{ "UW_DIVE_MODE_START_TIME", "UNP13", BaseEncoding::UINT, 0U, 86400U, {}, true, true },

	// [140] GNSS min C/N0, [141] GNSS min elevation (remaining slots reserved)
	{ "GNSS_MIN_CNO", "GNP42", BaseEncoding::UINT, 0U, 50U, {}, true, true },
	{ "GNSS_MIN_ELEV", "GNP43", BaseEncoding::UINT, 0U, 90U, {}, true, true },
	{ "__RESERVED", "", BaseEncoding::UINT, 0U, 0U, {}, false, false },
	{ "__RESERVED", "", BaseEncoding::UINT, 0U, 0U, {}, false, false },
	{ "__RESERVED", "", BaseEncoding::UINT, 0U, 0U, {}, false, false },

	// Critical battery threshold
	{ "LB_CRITICAL_THRESH", "LBP12", BaseEncoding::UINT, 0U, 100U, {}, true, true },

	// [146] Pressure sensor logging mode (slot always reserved)
	{ "PRESSURE_SENSOR_LOGGING_MODE", "PRP03", BaseEncoding::PRESSURESENSORLOGGINGMODE, 0, 0, {}, ENABLE_PRESSURE_SENSOR, true },
	// [147] GNSS trigger cold start on surfaced
	{ "GNSS_TRIGGER_COLD_START_ON_SURFACED", "GNP28", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	// [148-150] Sea temp TX options (slots always reserved)
	{ "SEA_TEMP_SENSOR_ENABLE_TX_MODE", "STP04", BaseEncoding::SENSORENABLETXMODE, 0, 0, {}, ENABLE_SEA_TEMP_SENSOR, true },
	{ "SEA_TEMP_SENSOR_ENABLE_TX_MAX_SAMPLES", "STP05", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, ENABLE_SEA_TEMP_SENSOR, true },
	{ "SEA_TEMP_SENSOR_ENABLE_TX_SAMPLE_PERIOD", "STP06", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, ENABLE_SEA_TEMP_SENSOR, true },
	// [151-153] pH TX options (slots always reserved)
	{ "PH_SENSOR_ENABLE_TX_MODE", "PHP04", BaseEncoding::SENSORENABLETXMODE, 0, 0, {}, ENABLE_PH_SENSOR, true },
	{ "PH_SENSOR_ENABLE_TX_MAX_SAMPLES", "PHP05", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, ENABLE_PH_SENSOR, true },
	{ "PH_SENSOR_ENABLE_TX_SAMPLE_PERIOD", "PHP06", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, ENABLE_PH_SENSOR, true },
	// [154-156] ALS TX options (slots always reserved)
	{ "ALS_SENSOR_ENABLE_TX_MODE", "LTP04", BaseEncoding::SENSORENABLETXMODE, 0, 0, {}, ENABLE_ALS_SENSOR, true },
	{ "ALS_SENSOR_ENABLE_TX_MAX_SAMPLES", "LTP05", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, ENABLE_ALS_SENSOR, true },
	{ "ALS_SENSOR_ENABLE_TX_SAMPLE_PERIOD", "LTP06", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, ENABLE_ALS_SENSOR, true },
	// [157-159] Pressure TX options (slots always reserved)
	{ "PRESSURE_SENSOR_ENABLE_TX_MODE", "PRP04", BaseEncoding::SENSORENABLETXMODE, 0, 0, {}, ENABLE_PRESSURE_SENSOR, true },
	{ "PRESSURE_SENSOR_ENABLE_TX_MAX_SAMPLES", "PRP05", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, ENABLE_PRESSURE_SENSOR, true },
	{ "PRESSURE_SENSOR_ENABLE_TX_SAMPLE_PERIOD", "PRP06", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, ENABLE_PRESSURE_SENSOR, true },
	// [160-162] AXL TX options (slots always reserved)
	{ "AXL_SENSOR_ENABLE_TX_MODE", "AXP05", BaseEncoding::SENSORENABLETXMODE, 0, 0, {}, ENABLE_AXL_SENSOR, true },
	{ "AXL_SENSOR_ENABLE_TX_MAX_SAMPLES", "AXP06", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, ENABLE_AXL_SENSOR, true },
	{ "AXL_SENSOR_ENABLE_TX_SAMPLE_PERIOD", "AXP07", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, ENABLE_AXL_SENSOR, true },
	// [163-165] Thermistor TX options (slots always reserved)
	{ "THERMISTOR_SENSOR_ENABLE_TX_MODE", "THP06", BaseEncoding::SENSORENABLETXMODE, 0, 0, {}, ENABLE_THERMISTOR_SENSOR, true },
	{ "THERMISTOR_SENSOR_ENABLE_TX_MAX_SAMPLES", "THP07", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, ENABLE_THERMISTOR_SENSOR, true },
	{ "THERMISTOR_SENSOR_ENABLE_TX_SAMPLE_PERIOD", "THP08", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, ENABLE_THERMISTOR_SENSOR, true },
	// [166-171] Camera sensor (slots always reserved)
	{ "CAM_ENABLE", "CAP01", BaseEncoding::BOOLEAN, 0, 0, {}, ENABLE_CAM_SENSOR, true },
	{ "CAM_TRIGGER_ON_SURFACED", "CAP02", BaseEncoding::BOOLEAN, 0, 0, {}, ENABLE_CAM_SENSOR, true },
	{ "CAM_TRIGGER_ON_AXL_WAKEUP", "CAP03", BaseEncoding::BOOLEAN, 0, 0, {}, ENABLE_CAM_SENSOR, true },
	{ "CAM_PERIOD_ON", "CAP04", BaseEncoding::UINT, 0U, 0xFFFFFFFFU, {}, ENABLE_CAM_SENSOR, true },
	{ "CAM_PERIOD_OFF", "CAP05", BaseEncoding::UINT, 0U, 0xFFFFFFFFU, {}, ENABLE_CAM_SENSOR, true },
	{ "LB_CAM_EN", "LBP13", BaseEncoding::BOOLEAN, 0, 0, {}, ENABLE_CAM_SENSOR, true },
	// [172-173] Satellite credentials (slots always reserved)
	{ "ARGOS_SECKEY", "IDP13", BaseEncoding::TEXT, "", "", {}, (ARGOS_SMD == 1), true },
	{ "ARGOS_RADIOCONF", "IDP14", BaseEncoding::TEXT, "", "", {}, true, true },
	// [174-176] Session shutdown control (slots always reserved)
	{ "SHUTDOWN_NTIME_SAT", "PWP05", BaseEncoding::UINT, 0U, 65535U, {}, true, true },
	{ "LB_SHUTDOWN_NTIME_SAT", "LBP14", BaseEncoding::UINT, 0U, 65535U, {}, true, true },
	{ "GNSS_SESSION_SINGLE_FIX", "GNP30", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	// [177] Pressure sensor full scale (slots always reserved)
	{ "PRESSURE_SENSOR_FULL_SCALE", "PRP07", BaseEncoding::PRESSURESENSORFULLSCALE, 0, 0, {}, ENABLE_PRESSURE_SENSOR, true },
	// [178] GNSS token (e.g. u-blox AssistNow/CloudLocate token)
	{ "GNSS_TOKEN", "GNP31", BaseEncoding::TEXT, "", "", {}, true, true },
	// [179] Last known RTC timestamp for pseudo RTC at boot
	{ "LAST_KNOWN_RTC", "PWP06", BaseEncoding::UINT, 0U, 0U, {}, true, false },
	// [180] Current RTC time (live, refreshed on STATR read)
	{ "RTC_CURRENT_TIME", "SYT01", BaseEncoding::UINT, 0U, 0U, {}, true, false },
	// [181-194] LoRa RAK3172 parameters (slots always reserved)
	{ "LORA_DEVEUI", "LRP01", BaseEncoding::TEXT, "", "", {}, (LORA_RAK3172 == 1), true },
	{ "LORA_APPEUI", "LRP02", BaseEncoding::TEXT, "", "", {}, (LORA_RAK3172 == 1), true },
	{ "LORA_APPKEY", "LRP03", BaseEncoding::TEXT, "", "", {}, (LORA_RAK3172 == 1), true },
	{ "LORA_DEVADDR", "LRP04", BaseEncoding::TEXT, "", "", {}, (LORA_RAK3172 == 1), true },
	{ "LORA_APPSKEY", "LRP05", BaseEncoding::TEXT, "", "", {}, (LORA_RAK3172 == 1), true },
	{ "LORA_NWKSKEY", "LRP06", BaseEncoding::TEXT, "", "", {}, (LORA_RAK3172 == 1), true },
	{ "LORA_NJM", "LRP07", BaseEncoding::UINT, 0U, 1U, {}, (LORA_RAK3172 == 1), true },
	{ "LORA_BAND", "LRP08", BaseEncoding::UINT, 0U, 12U, {}, (LORA_RAK3172 == 1), true },
	{ "LORA_CLASS", "LRP09", BaseEncoding::UINT, 0U, 2U, {}, (LORA_RAK3172 == 1), true },
	{ "LORA_DR", "LRP10", BaseEncoding::UINT, 0U, 15U, {}, (LORA_RAK3172 == 1), true },
	{ "LORA_ADR", "LRP11", BaseEncoding::BOOLEAN, 0, 0, {}, (LORA_RAK3172 == 1), true },
	{ "LORA_TXP", "LRP12", BaseEncoding::UINT, 0U, 14U, {}, (LORA_RAK3172 == 1), true },
	{ "LORA_CFM", "LRP13", BaseEncoding::BOOLEAN, 0, 0, {}, (LORA_RAK3172 == 1), true },
	{ "LORA_FPORT", "LRP14", BaseEncoding::UINT, 1U, 223U, {}, (LORA_RAK3172 == 1), true },
	{ "LORA_LP_MODE", "LRP15", BaseEncoding::UINT, 0U, 1U, {}, (LORA_RAK3172 == 1), true },
	// [196-198] Surfacing burst parameters (slots always reserved)
	{ "SURFACING_BURST_INIT_S", "ARP40", BaseEncoding::UINT, 1U, 120U, {}, true, true },
	{ "SURFACING_BURST_STEP_S", "ARP41", BaseEncoding::UINT, 0U, 60U, {}, true, true },
	{ "SURFACING_BURST_MAX_S", "ARP42", BaseEncoding::UINT, 5U, 600U, {}, true, true },
	// [199-205] Mortality detection parameters (slots always reserved)
	{ "MORTALITY_ENABLE", "MTP01", BaseEncoding::BOOLEAN, 0, 0, {}, ENABLE_MORTALITY_SENSOR, true },
	{ "MORTALITY_ACTIVITY_THRESH", "MTP02", BaseEncoding::UINT, 0U, 255U, {}, ENABLE_MORTALITY_SENSOR, true },
	{ "MORTALITY_TEMP_THRESH", "MTP03", BaseEncoding::FLOAT, 0.0, 60.0, {}, ENABLE_MORTALITY_SENSOR, true },
	{ "MORTALITY_GPS_DISTANCE_THRESH", "MTP04", BaseEncoding::UINT, 0U, 10000U, {}, ENABLE_MORTALITY_SENSOR, true },
	{ "MORTALITY_CONFIRM_DAYS", "MTP05", BaseEncoding::UINT, 1U, 30U, {}, ENABLE_MORTALITY_SENSOR, true },
	{ "MORTALITY_DUTY_CYCLE_MODULO", "MTP06", BaseEncoding::UINT, 0U, 100U, {}, ENABLE_MORTALITY_SENSOR, true },
	{ "MORTALITY_ORIGINAL_MODULO", "MTP07", BaseEncoding::UINT, 0U, 100U, {}, ENABLE_MORTALITY_SENSOR, false },
	// [206] RSPB packet format (RSPB board only)
	{ "RSPB_PACKET_FORMAT", "RSP01", BaseEncoding::UINT, 0U, 1U, { 0U, 1U }, HAS_BOARD_RSPB, true },
	// [207-209] Per-modulation radio configurations
	{ "ARGOS_RADIOCONF_LDK", "ARP51", BaseEncoding::TEXT, std::string(""), std::string(""), {}, true, true },
	{ "ARGOS_RADIOCONF_LDA2", "ARP52", BaseEncoding::TEXT, std::string(""), std::string(""), {}, true, true },
	{ "ARGOS_RADIOCONF_VLDA4", "ARP53", BaseEncoding::TEXT, std::string(""), std::string(""), {}, true, true },
	// [210] Adaptive modulation: auto-select LDK/LDA2/VLDA4 based on packet size
	{ "ARGOS_ADAPTIVE_MODULATION", "ARP54", BaseEncoding::BOOLEAN, false, false, {}, true, true },
	// [211] Surface cycle cooldown: min seconds between successful GPS+TX cycles (0=disabled)
	{ "MIN_SURFACE_CYCLE_INTERVAL_S", "UNP20", BaseEncoding::UINT, 0U, 86400U, {}, true, true },
	// [212] Surfacing burst Doppler limit: 0=unlimited, else stop Doppler phase after N messages
	{ "SURFACING_BURST_MAX_MSG", "ARP43", BaseEncoding::UINT, 0U, 255U, {}, true, true },
	// [213] Cooldown trigger mode: when to arm the cooldown timer
	{ "COOLDOWN_TRIGGER_MODE", "UNP30", BaseEncoding::UINT, 0U, 3U, { 0U, 1U, 2U, 3U }, true, true },
	// [214] SMD LPM mode bitmap: idle sleep between TX (module is powered off between sessions)
	{ "SMD_LPM_MODE", "ARP60", BaseEncoding::UINT, 0x01U, 0x1FU, {}, (ARGOS_SMD == 1), true },
	// [215-216] SWS adaptive sample delay bounds (µs)
	{ "SWS_DELAY_MIN_US", "UNP09", BaseEncoding::UINT, 50U, 15000U, {}, true, true },
	{ "SWS_DELAY_MAX_US", "UNP10", BaseEncoding::UINT, 100U, 30000U, {}, true, true },
	// [217] GNSS ANO staleness threshold in days (0=never discard stale ANO data)
	{ "GNSS_ANO_STALE_DAYS", "GNP44", BaseEncoding::UINT, 0U, 365U, {}, true, true },
	// [218] Fastloc mode: 0=OFF, 1=DEGRADED_PVT (degraded GPS fix), 2=CLOUDLOCATE (raw GNSS measurements)
	{ "GNSS_FASTLOC_MODE", "GNP45", BaseEncoding::UINT, 0U, 2U, {}, true, true },
	// [219] CloudLocate format: 0=MEASC12 (12B, sensor compatible), 1=MEAS20 (20B), 2=MEAS50 (50B, LoRa only)
	{ "GNSS_CLOUDLOCATE_FORMAT", "GNP46", BaseEncoding::UINT, 0U, 2U, {}, true, true },
	// [220] AXL FIFO batch mode enable
	{ "AXL_FIFO_ENABLE", "AXP10", BaseEncoding::BOOLEAN, {}, {}, {}, ENABLE_AXL_SENSOR, true },
	// [221] AXL FIFO sample count per batch (1-170)
	{ "AXL_FIFO_SAMPLE_COUNT", "AXP11", BaseEncoding::UINT, 1U, 170U, {}, ENABLE_AXL_SENSOR, true },
	// [222] LED HRS_24 RTC cutoff — auto-set by GPSService on first valid GNSS
	// fix to (now+24h). Used by ledsm.cpp on EXTERNAL_WAKEUP boards (RSPB) to
	// give a true 24h-since-deployment window across TPL5111 hard shutdowns.
	// User can override via DTE PARMW for re-deployment scenarios.
	{ "LED_HRS24_RTC_CUTOFF", "LDP03", BaseEncoding::DATESTRING, 0, 0, {}, true, true },
	// [223] RESERVED — former ARGOS_TX_NO_FIX_POLICY (ARP36, removed 2026-07: the
	// v3 no-fix behavior is hardwired in LEGACY/DUTY_CYCLE/PASS_PREDICTION — a
	// no-fix cycle TXs a 0xFF heartbeat/grid-filler; repetitions per
	// NTRY_PER_MESSAGE). Before that, GNSS_BCKP_CHARGE_INT (GNP47).
	{ "_RESERVED_223", "", BaseEncoding::UINT, 0, 0, {}, false, false },
	// [224] RESERVED — former ARGOS_LAST_KNOWN_MAX_AGE_S (ARP37, removed 2026-07
	// with the LAST_KNOWN policy). Before that, GNSS_BCKP_CHARGE_DUR (GNP48).
	{ "_RESERVED_224", "", BaseEncoding::UINT, 0, 0, {}, false, false },
	// [225] RESERVED — former GNSS_BCKP_CHARGE_UW_ONLY (GNP49), deep-idle refactor.
	// Reserved for flash-layout compat; key="" + settable=false -> PARAM_KEY_NOT_FOUND.
	{ "_RESERVED_225", "", BaseEncoding::BOOLEAN, 0, 0, {}, false, false },
	// [226] SMD degraded-mode flag — 0 = FAST timings (default), 1 = SAFE timings (auto-engaged
	// by SmdSat after SMD_MAX_CONSECUTIVE_ERRORS SPI errors when SMDSAT_AUTOFALLBACK is built in).
	// Persisted across reboot so a watchdog reset in degraded mode does not lose the SAFE state.
	// Read-only at DTE; written exclusively by SmdSat itself.
	// SMD_DEGRADED_MODE: read mostly via DTE for diagnostic; writable to allow
	// manual clear after a confirmed root-cause fix (e.g. HW repair, FW update
	// addressing the original cascade). DTE writes ONLY accept 0 — engagement
	// stays under autofallback control. min/max kept at 0/1 so firmware
	// `write_param(1)` from `degraded_mode_engage()` continues to work.
	//
	// Do NOT use permitted_values={0U} here. The DTE protocol's validate()
	// runs during PARMW decoding and throws DTE_PROTOCOL_VALUE_OUT_OF_RANGE
	// when the value is not in the list. The exception escapes the per-param
	// try/catch in dte_handler.cpp's PARMW_REQ (decoder runs BEFORE the loop),
	// kills the whole DTE command handler and breaks subsequent PARMR/PARMW.
	// When a GUI reads the full config and writes it back unchanged, it sends
	// SMP00 with its current value (which is 1 if autofallback engaged SAFE)
	// — the read-modify-write round trip must not jam the DTE channel just
	// because that slot is now 1.
	//
	// The "only accept 0 on PARMW" policy is enforced in DTEHandler::PARMW_REQ,
	// which rejects the slot cleanly via rejected_keys without
	// throwing — non-zero writes are logged + reported, every other slot in
	// the batch still applies.
	//
	// Runtime flag `g_smdsat_use_safe_timings` is re-synced from this param at
	// the next `SmdSat::send()` call (lazy sync).
	{ "SMD_DEGRADED_MODE", "SMP00", BaseEncoding::UINT, 0U, 1U, {}, true, true },
	// [227] Cached SMD modulation (0=LDA2, 1=LDK, 2=VLDA4) — mirrors what's actually
	// programmed in STM32WL flash. Written by SmdSat after the credentials-dirty path
	// reads back the master RCONF. On boot, SmdSat loads this so the FIRST surface-burst
	// TX uses the correct modulation without needing a runtime SPI READ_RCONF.
	// Read-only at DTE; written exclusively by SmdSat itself.
	{ "ARGOS_CACHED_MODULATION", "SMP01", BaseEncoding::UINT, 0U, 2U, {}, true, false },
	// [228] GNSS REUSE_LAST max fix age — see ParamID comment.
	// 0 = disable reuse (always fall back to OFF/Doppler). Default 24h.
	{ "GNSS_REUSE_FIX_MAX_AGE_S", "GNP50", BaseEncoding::UINT, 0U, 0xFFFFFFFFU, {}, true, true },
	// [229..231] Rolling-window TX rate limiter (Plan 1 step 2).
	// Disabled by default. RATE_LIMIT_MAX_TX is bounded by RateLimiter::MAX_CAP
	// (128) to keep the noinit ring buffer at a fixed 1024-byte footprint.
	{ "RATE_LIMIT_EN",        "RLP01", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	{ "RATE_LIMIT_WINDOW_S",  "RLP02", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, true, true },
	{ "RATE_LIMIT_MAX_TX",    "RLP03", BaseEncoding::UINT, 0U, 128U, {}, true, true },
	// [232..238] Hauled-vs-at-sea mode (Plan 1 step 3).
	// Detection params (HMP00..02) and override params (HMP10..13). Disabled
	// by default — HAULED_* substitutes mode/TR/gnss_en when engaged.
	{ "HAULED_DETECT_EN",          "HMP00", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	{ "HAULED_IDLE_THRESHOLD_H",   "HMP01", BaseEncoding::UINT,    1U, 0xFFFFU, {}, true, true },
	{ "HAULED_RETURN_EVENTS",      "HMP02", BaseEncoding::UINT,    1U, 0xFFU,   {}, true, true },
	// HMP10 deliberately excludes SURFACING_BURST (5): in HAULED the device is
	// dry/stationary by definition, so a dive-event-triggered burst can never
	// fire — using it as the hauled mode would silently produce zero TX.
	// LEGACY / DUTY_CYCLE / DOPPLER / PASS_PREDICTION are valid choices.
	// Defensive fallback: config_store HAULED branch auto-promotes any legacy
	// SURFACING_BURST value to LEGACY at read time.
	{ "HAULED_ARGOS_MODE",         "HMP10", BaseEncoding::ARGOSMODE, 0, 0, { 0U, 1U, 2U, 3U, 4U }, true, true },
	{ "HAULED_TR_NOM",             "HMP11", BaseEncoding::UINT,    1U, 0xFFFFFFFFU, {}, true, true },
	// HMP12 (HAULED_GNSS_EN): GPS acquisition enable WHEN HAULED. IMPORTANT:
	// this param is ONLY consulted by config_store when HMP13 == FRESH (0).
	// For HMP13 == REUSE_LAST (1) or OFF (2), config_store FORCES gnss_en=false
	// regardless of HMP12 — the cache/Doppler path is used instead. So:
	//   HMP13=0 (FRESH)      + HMP12=true  → GPS WAKES at every HAULED TX
	//   HMP13=0 (FRESH)      + HMP12=false → GPS OFF (Doppler-only TX)
	//   HMP13=1 (REUSE_LAST) + HMP12=*     → GPS OFF, TX uses cached fix
	//                                        (falls back to Doppler if cache
	//                                        > GNP50 = GNSS_REUSE_FIX_MAX_AGE_S)
	//   HMP13=2 (OFF)        + HMP12=*     → GPS OFF, Doppler-only (no cache)
	// A run-time WARN fires if HMP12=true is set with HMP13≠FRESH so the
	// operator sees the ambiguous combo in the post-deploy log.
	{ "HAULED_GNSS_EN",            "HMP12", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	// HAULED_GNSS_STRAT: 0=FRESH (acquire as usual, gated by HMP12),
	//                    1=REUSE_LAST (no GPS, TX uses last cached fix from
	//                                  depth pile, max age = GNP50),
	//                    2=OFF (no GPS, no cache lookup — pure Doppler).
	// Default 1 (REUSE_LAST) — battery-optimal for sealed turtle deployment.
	{ "HAULED_GNSS_STRAT",         "HMP13", BaseEncoding::UINT,    0U, 2U, {}, true, true },
	// [239] CloudLocate always-on: capture raw GNSS measurements on every
	// SURFACING_BURST surface, not just before the first fix. Useful for
	// devices with short surface windows where warm GPS fixes often miss
	// the 30 s timeout — CloudLocate raw-meas gives a cloud-side position
	// fallback at every surface. Costs ~30 s of GPS-on per surface (full
	// cold_acq_timeout) even if a real fix arrives early, since raw-meas
	// collection runs the full window.
	{ "GNSS_CLOUDLOCATE_ALWAYS",   "GNP51", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	// [240] GNSS deep-idle after power-off (seconds). 0=disabled (immediate
	// poweroff, current behavior); 0xFFFFFFFF=never poweroff (rail always on,
	// M10Q in PMREQ-backup); else=duration before auto-poweroff. Replaces the
	// deprecated GNSS_BCKP_CHARGE_* family. See plan doc in
	// .claude/plans/gnss-deep-idle-refactor.md.
	{ "GNSS_DEEP_IDLE_AFTER_OFF_S", "GNP52", BaseEncoding::UINT, 0U, 0xFFFFFFFFU, {}, true, true },
	// [241] FAST3b: terminate GNSS session as soon as first raw CloudLocate
	// measurement arrives, no waiting for full PVT. Only meaningful when
	// GNSS_FASTLOC_MODE=CLOUDLOCATE. Off by default.
	{ "GNSS_CLOUDLOCATE_ONLY",      "GNP53", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	// [242] After N consecutive GNSS sessions with no fix, force a true cold
	// start (BBR wipe) on the next acquisition. 0 = disabled (default).
	{ "GNSS_COLD_START_AFTER_NTRY", "GNP54", BaseEncoding::UINT, 0U, 255U, {}, true, true },
	// [243] BLIND MAC profile enable — module-owned retransmission burst; default off = BASIC
	{ "ARGOS_BLIND_EN",             "ARP44", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	// [244] BLIND retx_nb — module retransmissions per blind burst (KMAC ctx; int8 on module -> cap 127)
	{ "ARGOS_BLIND_RETX_NB",        "ARP45", BaseEncoding::UINT, 1U, 127U, {}, true, true },
	// [245] BLIND retx_period_s — interval (s) between module retransmissions (KMAC ctx; module range 60..65535)
	{ "ARGOS_BLIND_RETX_PERIOD_S",  "ARP46", BaseEncoding::UINT, 60U, 65535U, {}, true, true },
	// Orthogonal prepass + statuses (2026-08). PPP0x keys = configuration
	// (PARMR), PPT0x = read-only status (STATR, filters on key[2]=='T').
	{ "SAT_PREPASS_EN", "PPP07", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	{ "SAT_AOP_MAX_AGE_DAYS", "PPP08", BaseEncoding::UINT, 1U, 90U, {}, true, true },
	{ "SAT_PREPASS_MAX_WAIT_S", "PPP09", BaseEncoding::UINT, 0U, 604800U, {}, true, true },
	{ "SAT_AOP_VALID", "PPT01", BaseEncoding::BOOLEAN, 0, 0, {}, true, false },
	{ "SAT_AOP_AGE_S", "PPT02", BaseEncoding::UINT, 0U, 0xFFFFFFFFU, {}, true, false },
	{ "SAT_NEXT_PASS_TS", "PPT03", BaseEncoding::UINT, 0U, 0xFFFFFFFFU, {}, true, false },
	{ "SAT_LAST_PASS_TS", "PPT04", BaseEncoding::UINT, 0U, 0xFFFFFFFFU, {}, true, false },
	// [253..262] Moored-vs-underway mode (2026-08, Cyprus boat tracker).
	// Detection params (MRP00..04), override params (MRP05..08), read-only
	// status (MRT01). Disabled by default — MOORED_* substitutes the GNSS
	// period / TX interval / GNSS enable only while the classifier is engaged.
	{ "MOORED_DETECT_EN",     "MRP00", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	// MRP01 floor is 10 m and MooredModeService clamps to the same value: below
	// that, GNSS noise alone on a stationary receiver reads as movement and the
	// tracker never settles. 150 m suits a vessel swinging on its mooring.
	{ "MOORED_RADIUS_M",      "MRP01", BaseEncoding::UINT, 10U, 10000U, {}, true, true },
	{ "MOORED_ENTER_FIXES",   "MRP02", BaseEncoding::UINT, 1U, 0xFFU, {}, true, true },
	{ "MOORED_EXIT_EVENTS",   "MRP03", BaseEncoding::UINT, 1U, 0xFFU, {}, true, true },
	// MRP04 = 0 disables the anti-flapping hold-off entirely. Only sane on a
	// bench: on the water, swell trips the accelerometer and every trip costs a
	// GNSS acquisition, which is exactly the expense this feature exists to avoid.
	// Capped at 24 h — a hold-off longer than the moored GNSS period is pointless,
	// the scheduled acquisition catches the movement first.
	{ "MOORED_AXL_HOLDOFF_S", "MRP04", BaseEncoding::UINT, 0U, 86400U, {}, true, true },
	// MRP05 uses the same AQPERIOD code table as DLOC_ARG_NOM (ARP11), which
	// already offers 1 h / 2 h / 3 h / 4 h / 6 h / 12 h / 24 h.
	{ "MOORED_DLOC",          "MRP05", BaseEncoding::AQPERIOD, 0, 0, { 0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U, 10U, 11U, 12U, 13U, 14U, 15U }, true, true },
	// MRP06 shares TR_NOM's 30 s floor (a shorter TX interval is meaningless
	// here) but not its 1200 s ceiling — the whole point is to go well beyond it.
	//
	// The 86400 s (24 h) ceiling is NOT cosmetic. LoRaTxScheduler::schedule_periodic
	// walks forward in period_ms steps under `elapsed_time <= 24 h`, so with a
	// period above 24 h a single step overshoots the budget and the search exits
	// through the "no schedule found" throw -> INVALID_SCHEDULE -> no TX planned
	// at all. That path is only reachable when the candidate start time lands in
	// the past (an RTC jump forward on GNSS sync is the realistic trigger), and it
	// could never happen before because TR_NOM's own DTE maximum is 1200 s. This
	// ceiling keeps the widened range on the safe side of that loop. It also bounds
	// the worst-case silence of a sealed tracker to one day, which is a property
	// worth having on its own.
	{ "MOORED_TR_NOM",        "MRP06", BaseEncoding::UINT, 30U, 86400U, {}, true, true },
	// MRP07=false makes the accelerometer the ONLY way out of MOORED: a vessel
	// under tow, or drifting on a failed mooring with little vibration, would
	// keep reporting "moored" from its last anchor. Leave true unless the
	// battery budget genuinely cannot afford one fix per MRP05.
	{ "MOORED_GNSS_EN",       "MRP07", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	// MRP08: LoRa only. Sends the cached position in the moored heartbeat
	// instead of a battery-only STATUS frame — same SENSOR packet format, no
	// backend change.
	{ "MOORED_TX_LAST_POS",   "MRP08", BaseEncoding::BOOLEAN, 0, 0, {}, (LORA_RAK3172 == 1), true },
	// MRT01: read-only status (STATR filters on key[2]=='T'). The only way to
	// ask a sealed tracker whether it believes it is moored.
	{ "MOORED_STATE",         "MRT01", BaseEncoding::UINT, 0U, 1U, {}, true, false },
	// Culmination is the HIGHEST elevation a pass reaches, at its middle; it
	// says how good the pass is, where PP_MIN_ELEVATION (PPP01) only says when
	// the satellite becomes visible. Bounded at 90 because that is the zenith.
	{ "PP_MIN_CULMINATION",    "PPP10", BaseEncoding::UINT, 0U, 90U, {}, true, true },
	{ "PP_RX_MIN_CULMINATION", "PPP11", BaseEncoding::UINT, 0U, 90U, {}, true, true },
	// 100 km ceiling: past that the widened visibility circle predicts passes
	// the beacon cannot actually use.
	{ "PP_POSITION_MARGIN_KM", "PPP12", BaseEncoding::UINT, 0U, 100U, {}, true, true },
};
// clang-format on

const size_t param_map_size = sizeof(param_map) / sizeof(param_map[0]);
static_assert(sizeof(param_map) / sizeof(param_map[0]) == (size_t)ParamID::__PARAM_SIZE,
              "param_map size must match ParamID::__PARAM_SIZE");
