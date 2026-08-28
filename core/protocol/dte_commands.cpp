/**
 * @file dte_commands.cpp
 * @brief DTE command map — command name → prototype → handler dispatch table.
 */

#include "dte_commands.hpp"

// clang-format off
// Every argument below is a PROTOTYPE slot -- what the parser should expect on
// the wire -- not a stored parameter. Three of BaseMap's fields are therefore
// meaningless here and carry the same value in all 111 of them: `key` is "",
// `is_implemented` and `is_writable` are false. Only the name, the encoding and
// the accepted range say anything.
//
// Spelling those three out per argument cost ten lines apiece and buried the
// four that matter. ARG() states exactly the four.
#define ARG(NAME, ENC, MIN, MAX)                                                                    \
	{                                                                                               \
		.name = NAME, .key = "", .encoding = BaseEncoding::ENC, .min_value = MIN, .max_value = MAX, \
		.permitted_values = {}, .is_implemented = false, .is_writable = false                       \
	}

// The only argument in the table that constrains its values to a set.
#define ARG_PV(NAME, ENC, MIN, MAX, ...)                                                            \
	{                                                                                               \
		.name = NAME, .key = "", .encoding = BaseEncoding::ENC, .min_value = MIN, .max_value = MAX, \
		.permitted_values = __VA_ARGS__, .is_implemented = false, .is_writable = false              \
	}


const DTECommandMap command_map[] = {
	{
		.name = "PARML",
		.command = DTECommand::PARML_REQ,
		.prototype = 
		{
		}
	},
	{
		.name = "PARMR",
		.command = DTECommand::PARMR_REQ,
		.prototype = 
		{
			ARG("keys", KEY_LIST, 0, 0)
		}
	},
	{
		.name = "PARMW",
		.command = DTECommand::PARMW_REQ,
		.prototype = 
		{
			ARG("key_values", KEY_VALUE_LIST, 0, 0)
		}
	},
	{
		.name = "PROFR",
		.command = DTECommand::PROFR_REQ,
		.prototype = 
		{
		}
	},
	{
		.name = "PROFW",
		.command = DTECommand::PROFW_REQ,
		.prototype = 
		{
			ARG("profile_name", TEXT, 1, 128)
		}
	},
	{
		.name = "PASPW",
		.command = DTECommand::PASPW_REQ,
		.prototype = 
		{
			ARG("prepass_file", BASE64, 0, 0)
		}
	},
	{
		.name = "SECUR",
		.command = DTECommand::SECUR_REQ,
		.prototype = 
		{
				ARG("accesscode", HEXADECIMAL, 0U, 0U),
		}
	},
	{
		.name = "DUMPM",
		.command = DTECommand::DUMPM_REQ,
		.prototype = 
		{
			ARG("start_address", HEXADECIMAL, 0U, 0U),
			ARG("length", HEXADECIMAL, 0U, 0x500U)
		}
	},
	{
		.name = "DUMPD",
		.command = DTECommand::DUMPD_REQ,
		.prototype =
		{
			// Updated: includes MORTALITY(12)
			ARG("d_type", HEXADECIMAL, 0U, 12U),
		}
	},
	{
		.name = "RSTVW",
		.command = DTECommand::RSTVW_REQ,
		.prototype =
		{
				ARG_PV("index", HEXADECIMAL, 0U, 0U, { 1U, 2U, 3U, 4U }),
		}
	},
	{
		.name = "RSTBW",
		.command = DTECommand::RSTBW_REQ,
		.prototype = 
		{
		}
	},
	{
		.name = "FACTW",
		.command = DTECommand::FACTW_REQ,
		.prototype = 
		{
		}
	},
	{
		.name = "STATR",
		.command = DTECommand::STATR_REQ,
		.prototype =
		{
			ARG("keys", KEY_LIST, 0, 0)
		}
	},
	{
		.name = "ERASE",
		.command = DTECommand::ERASE_REQ,
		.prototype =
		{
			// Updated: includes MORTALITY(14)
			ARG("log_type", UINT, 1U, 14U)
		}
	},
	{
		.name = "SCALW",
		.command = DTECommand::SCALW_REQ,
		.prototype =
		{
			// Updated: includes SWS (device_id=8, see m_scalx in dte_handler.hpp)
			ARG("sensor", UINT, 0U, 8U),
			ARG("offset", UINT, 0U, 0U),
			ARG("value", FLOAT, 0.0, 0.0)
		}
	},
	{
		.name = "SATTX",
		.command = DTECommand::ARGOSTX_REQ,
		.prototype =
		{
			ARG("modulation", UINT, 0U, 2U),
			// Stored mode: size (decimal string, e.g. "24")
			// Custom mode: radioconf (32-char hex string)
			ARG("radioconf_or_size", TEXT, 0U, 0U),
			// Stored mode: tcxo (optional)
			// Custom mode: size
			ARG("size_or_tcxo", UINT, 0U, 0U),
			// Custom mode only: tcxo (optional)
			ARG("tcxo", UINT, 0U, 0U)
		},
		.min_args = 2  // modulation + radioconf_or_size required; rest optional
	},
	{
		.name = "SCALR",
		.command = DTECommand::SCALR_REQ,
		.prototype =
		{
			// Updated: includes SWS (device_id=8, see m_scalx in dte_handler.hpp)
			ARG("sensor", UINT, 0U, 8U),
			ARG("offset", UINT, 0U, 0U)
		}
	},
	// SENSR - Sensor/GNSS read command
	// Usage: $SENSR,<sensors_bitmask>,<gnss_timeout_s>*<checksum>\r\n
	// sensors_bitmask: 1=battery, 2=pressure, 4=GNSS, 8=accel, 15=all
	// Response: $SENSR,<status>,<batt_mv>,<batt_soc>,<pressure_bar>,<temp_c>,<altitude_m>,<lat>,<lon>,<hdop>,<num_sv>,<accel_x>,<accel_y>,<accel_z>,<accel_temp>*<checksum>\r\n
	{
		.name = "SENSR",
		.command = DTECommand::SENSR_REQ,
		.prototype =
		{
			// Bitmask: 1=battery, 2=pressure, 4=GNSS, 8=accel, 16=thermistor, 32=sea_temp, 64=ALS, 128=pH
			ARG("sensors", UINT, 1U, 255U),
			// GNSS timeout in seconds (5-300s)
			ARG("timeout", UINT, 5U, 300U)
		}
	},
	// PWRON - Power on/off components command
	// Usage: $PWRON#001;<component>\r
	// Components: 0=all, 1=gnss, 2=sensors, 3=satellite, 4=off
	// Response: $O;PWRON#000;\r or $N;PWRON#00x;<error>\r
	{
		.name = "PWRON",
		.command = DTECommand::PWRON_REQ,
		.prototype =
		{
			// 0=all, 1=gnss, 2=sensors, 3=satellite, 4=off
			ARG("component", UINT, 0U, 4U)
		}
	},
	// SWSST - SWS analog calibration status read (no arguments)
	{
		.name = "SWSST",
		.command = DTECommand::SWSST_REQ,
		.prototype = {}
	},
	// SATDP - Satellite Doppler calibration (no arguments)
	// Usage: $SATDP#000;\r
	// Starts periodic Doppler TX at TR_NOM interval until device reset
	{
		.name = "SATDP",
		.command = DTECommand::SATDP_REQ,
		.prototype = {}
	},
	// GNSSI - GNSS device info (no arguments)
	// Usage: $GNSSI#000;\r
	// Returns unique ID, SW version, HW version from GNSS module
	{
		.name = "GNSSI",
		.command = DTECommand::GNSSI_REQ,
		.prototype = {}
	},
	// GNSSA - GNSS almanac file validation (no arguments)
	// Usage: $GNSSA#000;\r
	// Returns almanac file presence, size, record counts, staleness
	{
		.name = "GNSSA",
		.command = DTECommand::GNSSA_REQ,
		.prototype = {}
	},
	// RTCW - RTC manual write (set time before GNSS fix)
	// Usage: $RTCW#00A;<unix_timestamp>\r
	// Sets the RTC to the given unix timestamp value
	{
		.name = "RTCW",
		.command = DTECommand::RTCW_REQ,
		.prototype =
		{
			// No max limit for unix timestamp
			ARG("timestamp", UINT, 0U, 0U)
		}
	},
	// SWSTST - SWS test mode start/stop
	// Usage: $SWSTST#001;1\r (start) or $SWSTST#001;0\r (stop)
	// Response: $O;SWSTST#001;<running>\r
	{
		.name = "SWSTST",
		.command = DTECommand::SWSTST_REQ,
		.prototype =
		{
			// 0=stop, 1=start
			ARG("action", UINT, 0U, 1U)
		}
	},
	// SWSCAL - SWS guided calibration (LED-assisted air/water measurement)
	// Usage: $SWSCAL#001;1\r (start) or $SWSCAL#001;0\r (cancel)
	// Response: $O;SWSCAL#003;<status>,<air>,<water>\r
	{
		.name = "SWSCAL",
		.command = DTECommand::SWSCAL_REQ,
		.prototype =
		{
			// 0=cancel, 1=start
			ARG("action", UINT, 0U, 1U)
		}
	},
	// SWSSTATS - SWS persistent diagnostic counters (audit 2026-05 R-MON-02)
	// Usage: $SWSSTATS#001;0\r (read) or $SWSSTATS#001;1\r (clear + read)
	// Response: $O;SWSSTATS#007;<stuck_rec>,<coh_recalib>,<dive_to>,<force_surf>,
	//                          <spike_rej>,<peak_incoh>,<saadc_retry>\r
	// Counters survive soft reset (noinit RAM) but reset on cold reset / power-on.
	// Saturate at 65535 (no wrap) so a recovered tracker exposes a true minimum.
	{
		.name = "SWSSTATS",
		.command = DTECommand::SWSSTATS_REQ,
		.prototype =
		{
			// 0=read, 1=clear+read
			ARG("action", UINT, 0U, 1U)
		}
	},
	// GNSSBR - GNSS UART bridge/passthrough mode (direct u-blox access via USB)
	// Usage: $GNSSBR#001;1\r (start) — exit by typing +++
	{
		.name = "GNSSBR",
		.command = DTECommand::GNSSBR_REQ,
		.prototype =
		{
			// 0=stop, 1=start
			ARG("action", UINT, 0U, 1U)
		}
	},
	// GNSSBCKP - GNSS backup-cell charge mode (rail ON, M10 in deep sleep)
	// Usage: $GNSSBCKP#001;<duration_s>\r   (duration_s in seconds; 0 = abort)
	// Response: $O;GNSSBCKP#000;\r
	//
	// IMPORTANT: position in this array must match GNSSBCKP_REQ's enum index
	// in DTECommand. DTEEncoder::encode() indexes command_map[] by enum value
	// to look up the response name, so a misordered entry corrupts every
	// downstream response (RTCW, SWSTST, SWSCAL, GNSSBR, ...).
	{
		.name = "GNSSBCKP",
		.command = DTECommand::GNSSBCKP_REQ,
		.prototype =
		{
			ARG("duration_s", UINT, 0U, 86400U)
		}
	},
	// SMDDFU command - always available: VERSION action (5) works on SMD/KIM2/LoRa
	// builds; other actions (enter/exit/status/update/info) are SMD-only.
	// Usage: $SMDDFU#001;<action>\r
	// Actions: 0=enter, 1=exit, 2=status, 3=update, 4=info, 5=version
	{
		.name = "SMDDFU",
		.command = DTECommand::SMDDFU_REQ,
		.prototype =
		{
			// 0=enter, 1=exit, 2=status, 3=update, 4=info, 5=version
			ARG("action", UINT, 0U, 5U)
		}
	},
	// COMCW command - Continuous Wave RF test (SMD / LoRa). KIM2 returns "unsupported".
	// Usage: $COMCW#00N;<mode>[,<freq_hz>,<power_dbm>,<duration_s>]\r
	// mode: 0=stop, 1=start
	// freq_hz: carrier frequency (Hz)
	// power_dbm: TX power (dBm)
	// duration_s: 1-65535 s (LoRa requires non-zero; SMD allows 0=until stop)
	{
		.name = "COMCW",
		.command = DTECommand::COMCW_REQ,
		.prototype =
		{
			ARG("mode", UINT, 0U, 1U),
			ARG("freq_hz", UINT, 0U, 4294967295U),
			ARG("power_dbm", UINT, 0U, 30U),
			ARG("duration_s", UINT, 0U, 65535U),
		},
		.min_args = 1  // mode is required; freq/power/duration optional (only for start)
	},
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
	// SMD SPI applicative test - tests all A+ protocol read commands
	// Usage: $SMDTST#000;\r
	{
		.name = "SMDTST",
		.command = DTECommand::SMDTST_REQ,
		.prototype = {}
	},
#endif
	// Satellite credentials verify — read-back from hardware, compare with config store.
	// Required `force` arg (0 or 1): when 1 and hardware differs, credentials are written
	// from the config store to the satellite module, then re-read.
	// Supported on SMD (dirty flag → state_load_kmac) and LoRa (AT_SET_* sequence).
	// KIM2 has no write path; `force` is ignored on KIM2 builds.
	// Usage: $SATVF#001;0\r   or   $SATVF#001;1\r
	{
		.name = "SATVF",
		.command = DTECommand::SATVF_REQ,
		.prototype =
		{
			ARG("force", UINT, 0U, 1U)
		}
	},
#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
	// LoRa test transmission
	// Usage: $LORATX#001;<size>\r
	{
		.name = "LORATX",
		.command = DTECommand::LORATX_REQ,
		.prototype =
		{
			ARG("size", UINT, 1U, 222U)
		}
	},
	// LoRa UART bridge/passthrough mode (direct RUI3 AT access via USB)
	// Usage: $LORABR#001;1\r (start) — exit by typing +++
	{
		.name = "LORABR",
		.command = DTECommand::LORABR_REQ,
		.prototype =
		{
			// 0=stop, 1=start
			ARG("action", UINT, 0U, 1U)
		}
	},
#endif
#if !(defined(LORA_RAK3172) && (LORA_RAK3172 == 1)) && !(defined(ARGOS_SMD) && (ARGOS_SMD == 1))
	// KIM2 UART bridge/passthrough mode (direct AT command access via USB)
	// Usage: $KIMBR#001;1\r (start) — exit by typing +++
	{
		.name = "KIMBR",
		.command = DTECommand::KIMBR_REQ,
		.prototype =
		{
			// 0=stop, 1=start
			ARG("action", UINT, 0U, 1U)
		}
	},
#endif
	{
		.name = "PARML",
		.command = DTECommand::PARML_RESP,
		.prototype =
		{
			ARG("keys", KEY_LIST, 0, 0)
		}
	},
	{
		.name = "PARMR",
		.command = DTECommand::PARMR_RESP,
		.prototype = 
		{
			ARG("key_values", KEY_VALUE_LIST, 0, 0)
		}
	},
	{
		.name = "PARMW",
		.command = DTECommand::PARMW_RESP,
		.prototype = 
		{
		}
	},
	{
		.name = "PROFR",
		.command = DTECommand::PROFR_RESP,
		.prototype = 
		{
			ARG("profile_name", TEXT, "", "")
		}
	},
	{
		.name = "PROFW",
		.command = DTECommand::PROFW_RESP,
		.prototype = 
		{
		}
	},
	{
		.name = "PASPW",
		.command = DTECommand::PASPW_RESP,
		.prototype = 
		{
		}
	},
	{
		.name = "SECUR",
		.command = DTECommand::SECUR_RESP,
		.prototype = 
		{
		}
	},
	{
		.name = "DUMPM",
		.command = DTECommand::DUMPM_RESP,
		.prototype = 
		{
			ARG("data", BASE64, 0, 0)
		}
	},
	{
		.name = "DUMPD",
		.command = DTECommand::DUMPD_RESP,
		.prototype = 
		{
			ARG("mmm", HEXADECIMAL, 0U, 0xFFFU),
			ARG("MMM", HEXADECIMAL, 0U, 0xFFFU),
			ARG("data", BASE64, 0, 0)
		}
	},
	{
		.name = "RSTVW",
		.command = DTECommand::RSTVW_RESP,
		.prototype =
		{
		}
	},
	{
		.name = "RSTBW",
		.command = DTECommand::RSTBW_RESP,
		.prototype = 
		{
		}
	},
	{
		.name = "FACTW",
		.command = DTECommand::FACTW_RESP,
		.prototype = 
		{
		}
	},
	{
		.name = "STATR",
		.command = DTECommand::STATR_RESP,
		.prototype =
		{
			ARG("key_values", KEY_VALUE_LIST, 0, 0)
		}
	},
	{
		.name = "ERASE",
		.command = DTECommand::ERASE_RESP,
		.prototype =
		{
		}
	},
	{
		.name = "SCALW",
		.command = DTECommand::SCALW_RESP,
		.prototype =
		{
		}
	},
	{
		.name = "SATTX",
		.command = DTECommand::ARGOSTX_RESP,
		.prototype =
		{
		}
	},
	{
		.name = "SCALR",
		.command = DTECommand::SCALR_RESP,
		.prototype =
		{
			ARG("value", FLOAT, 0.0, 0.0)
		}
	},
	// SENSR response - sensor readings
	{
		.name = "SENSR",
		.command = DTECommand::SENSR_RESP,
		.prototype =
		{
			ARG("batt_mv", UINT, 0U, 0U),
			ARG("batt_soc", UINT, 0U, 100U),
			ARG("pressure", FLOAT, 0.0, 0.0),
			ARG("temperature", FLOAT, 0.0, 0.0),
			ARG("altitude", FLOAT, 0.0, 0.0),
			ARG("lat", FLOAT, 0.0, 0.0),
			ARG("lon", FLOAT, 0.0, 0.0),
			ARG("hdop", FLOAT, 0.0, 0.0),
			ARG("num_sv", UINT, 0U, 0U),
			ARG("accel_x", FLOAT, 0.0, 0.0),
			ARG("accel_y", FLOAT, 0.0, 0.0),
			ARG("accel_z", FLOAT, 0.0, 0.0),
			ARG("accel_temp", FLOAT, 0.0, 0.0),
			// Activity level 0-255
			ARG("activity", UINT, 0U, 255U),
			ARG("thermistor_temp", FLOAT, 0.0, 0.0),
			ARG("sea_temp", FLOAT, 0.0, 0.0),
		ARG("als_lux", FLOAT, 0.0, 0.0),
		ARG("ph", FLOAT, 0.0, 0.0),
		ARG("sensor_status", UINT, 0U, 0xFFU)
	}
	},
	// PWRON response - simple acknowledgement
	{
		.name = "PWRON",
		.command = DTECommand::PWRON_RESP,
		.prototype = {}
	},
	// SWSST response - SWS calibration status values
	{
		.name = "SWSST",
		.command = DTECommand::SWSST_RESP,
		.prototype =
		{
			ARG("air", UINT, 0U, 0U),
			ARG("water", UINT, 0U, 0U),
			ARG("threshold", UINT, 0U, 0U),
			ARG("hysteresis", UINT, 0U, 0U),
			ARG("raw_adc", UINT, 0U, 0U),
			ARG("filtered_adc", UINT, 0U, 0U),
			ARG("calibrated", BOOLEAN, 0U, 1U),
			ARG("underwater", BOOLEAN, 0U, 1U),
			ARG("time_in_state", UINT, 0U, 0U),
			ARG("surface_level", UINT, 0U, 5U),
			ARG("contrast_x10", UINT, 0U, 0U),
			ARG("observed_peak", UINT, 0U, 0U),
			ARG("sample_delay_us", UINT, 0U, 0U)
		}
	},
	// SATDP response - simple OK/error acknowledgement
	{
		.name = "SATDP",
		.command = DTECommand::SATDP_RESP,
		.prototype = {}
	},
	// GNSSI response - unique ID, SW version, HW version
	{
		.name = "GNSSI",
		.command = DTECommand::GNSSI_RESP,
		.prototype =
		{
			ARG("unique_id", TEXT, "", ""),
			ARG("sw_version", TEXT, "", ""),
			ARG("hw_version", TEXT, "", "")
		}
	},
	// GNSSA response - almanac file status
	{
		.name = "GNSSA",
		.command = DTECommand::GNSSA_RESP,
		.prototype =
		{
			ARG("present", UINT, 0U, 1U),
			ARG("file_size", UINT, 0U, 0U),
			ARG("total_records", UINT, 0U, 0U),
			ARG("valid_records", UINT, 0U, 0U),
			ARG("stale", UINT, 0U, 1U)
		}
	},
	// RTCW response - simple acknowledgement
	{
		.name = "RTCW",
		.command = DTECommand::RTCW_RESP,
		.prototype = {}
	},
	// SWSTST response - SWS test mode running state
	{
		.name = "SWSTST",
		.command = DTECommand::SWSTST_RESP,
		.prototype =
		{
			ARG("running", UINT, 0U, 1U)
		}
	},
	// SWSCAL response - guided calibration result
	{
		.name = "SWSCAL",
		.command = DTECommand::SWSCAL_RESP,
		.prototype =
		{
			// 0=in progress, 1=success, 2=failed, 3=cancelled
			ARG("status", UINT, 0U, 3U),
			ARG("air", UINT, 0U, 16383U),
			ARG("water", UINT, 0U, 16383U)
		}
	},
	// SWSSTATS response - persistent diagnostic counters (audit 2026-05 R-MON-02)
	{
		.name = "SWSSTATS",
		.command = DTECommand::SWSSTATS_RESP,
		.prototype =
		{
			ARG("stuck_recovery", UINT, 0U, 65535U),
			ARG("coherence_recalib", UINT, 0U, 65535U),
			ARG("dive_timeout", UINT, 0U, 65535U),
			ARG("force_surface", UINT, 0U, 65535U),
			ARG("spike_reject", UINT, 0U, 65535U),
			ARG("peak_incoherent", UINT, 0U, 65535U),
			ARG("saadc_init_retry", UINT, 0U, 65535U)
		}
	},
	// GNSSBR response - simple acknowledgement
	{
		.name = "GNSSBR",
		.command = DTECommand::GNSSBR_RESP,
		.prototype = {}
	},
	// GNSSBCKP response - simple acknowledgement.
	// Position in this array must match GNSSBCKP_RESP's enum index — see the
	// matching note on the GNSSBCKP_REQ entry above.
	{
		.name = "GNSSBCKP",
		.command = DTECommand::GNSSBCKP_RESP,
		.prototype = {}
	},
	// SMDDFU response — always available (VERSION action works for all builds)
	// Response: $SMDDFU,<status>,<dfu_mode>,<progress>[,<info>]*<checksum>\r\n
	{
		.name = "SMDDFU",
		.command = DTECommand::SMDDFU_RESP,
		.prototype =
		{
			// DFU response status code
			ARG("status", UINT, 0U, 0xFFU),
			// True if in DFU mode
			ARG("dfu_mode", BOOLEAN, 0U, 1U),
			// Progress percentage 0-100
			ARG("progress", UINT, 0U, 100U),
			// Additional info (bootloader version, etc.)
			ARG("info", TEXT, "", "")
		}
	},
	// COMCW response — status code + optional info string
	{
		.name = "COMCW",
		.command = DTECommand::COMCW_RESP,
		.prototype =
		{
			ARG("status", UINT, 0U, 0xFFU),
			ARG("info", TEXT, "", ""),
		}
	},
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
	// SMDTST response - applicative SPI test results
	{
		.name = "SMDTST",
		.command = DTECommand::SMDTST_RESP,
		.prototype =
		{
			ARG("info", TEXT, "", "")
		}
	},
#endif
	{
		.name = "SATVF",
		.command = DTECommand::SATVF_RESP,
		.prototype =
		{
			ARG("hw_id", UINT, (unsigned int)0, (unsigned int)0),
			ARG("hw_addr", UINT, (unsigned int)0, (unsigned int)0),
			ARG("hw_seckey", TEXT, "", ""),
			ARG("hw_rconf", TEXT, "", ""),
			ARG("match", UINT, (unsigned int)0, (unsigned int)0),
			ARG("forced", UINT, (unsigned int)0, (unsigned int)0)
		}
	},
#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
	{
		.name = "LORATX",
		.command = DTECommand::LORATX_RESP,
		.prototype = {}
	},
	{
		.name = "LORABR",
		.command = DTECommand::LORABR_RESP,
		.prototype = {}
	},
#endif
#if !(defined(LORA_RAK3172) && (LORA_RAK3172 == 1)) && !(defined(ARGOS_SMD) && (ARGOS_SMD == 1))
	{
		.name = "KIMBR",
		.command = DTECommand::KIMBR_RESP,
		.prototype = {}
	},
#endif
};

// clang-format on

const size_t command_map_size = sizeof(command_map) / sizeof(command_map[0]);
