/**
 * @file config_store.hpp
 * @brief Abstract configuration store — 220+ DTE parameters, zone/LB logic, GNSS/Argos config.
 */

#pragma once

#include <array>
#include <type_traits>
#include <ctime>
#include <cmath>

#include "base_types.hpp"
#include "error.hpp"
#include "debug.hpp"
#include "messages.hpp"
#include "haversine.hpp"
#include "timeutils.hpp"
#include "pmu.hpp"
#include "sensor.hpp"
#include "service_scheduler.hpp"
#include "hauled_mode_service.hpp"
#include "moored_mode_service.hpp"

#if VALIDATION_LOG_ENABLE
// The [VAL-*] traces in this header timestamp using the global RTC. Without
// this declaration, a build with -DVALIDATION_LOG_ENABLE=1 fails on
// "'rtc' was not declared in this scope" — and since the build scripts did not
// check make's return code, the failure still produced flash commands pointing
// at a stale binary.
#include "rtc.hpp"
extern RTC *rtc;
#endif

// Default TPL5111 external-wakeup period (seconds) — MUST match the board's
// TPL5111 timing resistor. Overridable at build time (build_rspb.sh: WAKEUP_PERIOD).
// Still runtime-editable via DTE param WAKEUP_PERIOD (key PWP04, is_writable=true).
#ifndef WAKEUP_PERIOD_DEFAULT
#define WAKEUP_PERIOD_DEFAULT 6300U
#endif

static constexpr unsigned int MAX_CONFIG_ITEMS = (unsigned int)ParamID::__PARAM_SIZE;

struct GNSSConfig {
	bool enable;
	bool hdop_filter_enable;
	unsigned int hdop_filter_threshold;
	bool hacc_filter_enable;
	unsigned int hacc_filter_threshold;
	unsigned int acquisition_timeout_cold_start;
	unsigned int acquisition_timeout;
	unsigned int dloc_arg_nom;
	bool underwater_en;
	uint16_t battery_voltage;
	BaseGNSSFixMode fix_mode;
	BaseGNSSDynModel dyn_model;
	bool is_out_of_zone;
	bool is_lb;
	unsigned int min_num_fixes;
	unsigned int cold_start_retry_period;
	bool assistnow_enable;
	bool trigger_on_surfaced;
	bool assistnow_offline_enable;
	unsigned int constellation_mask;
	unsigned int orbmaxerr;
	unsigned int min_cno;
	unsigned int min_elev;
	unsigned int ano_stale_days;
};

struct ArgosConfig {
	unsigned int tx_counter;
	double frequency;
	BaseArgosPower power;
	unsigned int tx_interval_s;
	BaseArgosMode mode;
	unsigned int ntry_per_message;
	unsigned int duty_cycle;
	BaseDepthPile depth_pile;
	BaseDeltaTimeLoc delta_time_loc;
	unsigned int dry_time_before_tx;
	unsigned int surfacing_burst_init_s;
	unsigned int surfacing_burst_step_s;
	unsigned int surfacing_burst_max_s;
	unsigned int argos_id;
	bool underwater_en;
	double prepass_min_elevation;
	double prepass_max_elevation;
	unsigned int prepass_min_duration;
	unsigned int prepass_max_passes;
	unsigned int prepass_linear_margin;
	unsigned int prepass_comp_step;
	/// @name PREPASS v4.0 filters, hardcoded when the engine was imported
	/// @{
	unsigned int prepass_min_culmination;     ///< deg, TX path (0 = off)
	unsigned int prepass_rx_min_culmination;  ///< deg, AOP downlink window
	unsigned int prepass_position_margin_km;  ///< km, beacon position uncertainty
	/// @}
	bool is_out_of_zone;
	bool is_lb;
	bool time_sync_burst_en;
	bool argos_tx_jitter_en;
	bool argos_rx_en;
	unsigned int argos_rx_max_window;
	bool gnss_en;
	// GNSS sourcing strategy. Default FRESH = existing behavior (acquire fresh
	// fix, then build packet from depth-pile entry). Set to REUSE_LAST only in
	// the HAULED branch of get_argos_configuration; all other branches keep
	// FRESH so existing GNSS paths are byte-identical to pre-Plan-1 code.
	BaseGnssStrategy gnss_strategy;
	unsigned int argos_rx_aop_update_period;
	std::time_t last_aop_update;
	bool prepass_en;                  // prepass gating, independent of the mode
	unsigned int aop_max_age_days;    // beyond this: AOP stale -> periodic fallback
	unsigned int prepass_max_wait_s;  // max wait without a window (0 = unlimited)
	bool cert_tx_enable;
	std::string cert_tx_payload;
	BaseArgosModulation cert_tx_modulation;
	unsigned int cert_tx_repetition;
	unsigned int argos_tcxo_warmup_time;
	unsigned int sensor_tx_enable;
	unsigned int shutdown_ntime_sat;
	bool adaptive_modulation;
	std::string radioconf_ldk;
	std::string radioconf_lda2;
	std::string radioconf_vlda4;
	// BLIND MAC profile (module-owned retransmission). Global (not per-regime):
	// blind_en selects KMAC BLIND vs BASIC; blind_retx_nb/period_s are the module
	// burst config. NTRY_PER_MESSAGE stays the nRF-side count of blind sequences.
	bool blind_en;
	unsigned int blind_retx_nb;
	unsigned int blind_retx_period_s;
	/// @brief Plancher de periode BLIND, en secondes.
	/// Le module ne sait pas repeter plus vite que cela; c est aussi le minimum
	/// DTE d'ARP46. Sert de critere de compatibilite avec la cadence d'un mode
	/// qui rythme deja sa propre sequence.
	static constexpr unsigned int BLIND_MIN_RETX_PERIOD_S = 60;
};

enum class ConfigMode {
	NORMAL,
	LOW_BATTERY,
	OUT_OF_ZONE,
	HAULED,  // Plan 1 step 3 — substitutes HAULED_* override params
	MOORED   // 2026-08 — substitutes MOORED_* override params (vessel stationary)
	         // (Plan 2 will add AT_SEA_SEQUENCED below this, between HAULED and base.)
};


class ConfigurationStore {
protected:
	// 0x1F (2026-07): ARP36/ARP37 removed from the serialized set (slots 223/224
	// back to reserved). The June builds (b8a4946e+) serialized them WITHOUT a
	// version bump, so config.dat generations with and without those records
	// share 0x1E — sequential deserialization would misalign and silently
	// factory-reset every param after slot 222. Bumping forces the documented
	// recovery path instead: keep ARGOS_DECID/ARGOS_HEXID, reset the rest,
	// reprovision after upgrade.
	// 0x20 (2026-07): +ARGOS_BLIND_EN/RETX_NB/RETX_PERIOD_S (slots 243-245, blind MAC profile).
	// 0x20 KEPT in 2026-08 despite the addition of slots 246-252 (orthogonal
	// prepass + statuses). A bump is NOT needed here and would be harmful:
	// it only keeps ARGOS_DECID/ARGOS_HEXID and resets EVERYTHING else to
	// factory values, which would force reprovisioning every deployed beacon.
	// The addition is made AT THE END of the table, so no existing slot moves;
	// when re-reading an old file, entries 246+ land on an end of file,
	// deserialize_config_entry() returns false, the loop applies the default
	// value and CONTINUES (it does not stop), then reserializes.
	// The bump stays mandatory if an existing slot is REMOVED or MOVED:
	// that is what shifted all the following ones with ARP36/37 in 2026-06.
	// 0x20 KEPT again in 2026-08 for slots 253-262 (moored-vs-underway mode,
	// MRP00..MRP08 + MRT01), same reasoning: pure append, no slot moves. It
	// matters more here than usual — the LoRa builds keep their LoRaWAN
	// credentials (LORA_DEVEUI/APPKEY/DEVADDR/APPSKEY/NWKSKEY) in this file, and
	// the recovery path preserves only ARGOS_DECID/ARGOS_HEXID. A needless bump
	// would leave every provisioned LoRa unit unable to rejoin its network, with
	// nothing in the logs to say why.
	static inline const unsigned int m_config_version_code = 0x1c07e800 | 0x20;
	static inline const unsigned int m_config_version_code_aop = 0x1c07e800 | 0x03;
	// one line per ParamID slot; the alignment IS the index map
	// clang-format off
	static inline const BaseType default_params[] = {
		/* ARGOS_DECID */ 0U,
		/* ARGOS_HEXID */ 0U,
		/* DEVICE_MODEL */ DEVICE_MODEL_NAME,
		/* FW_APP_VERSION */ FW_APP_VERSION_STR,
		/* LAST_TX */ static_cast<std::time_t>(0U),
		/* TX_COUNTER */ 0U,
		/* BATT_SOC */ 0U,
		/* _RESERVED_7 (was LAST_FULL_CHARGE_DATE) */ static_cast<std::time_t>(0U),
		/* PROFILE_NAME */ std::string("FACTORY"),
		/* _RESERVED_9 */ 0U,
		/* ARGOS_AOP_DATE */ static_cast<std::time_t>(1633646474U),
		/* ARGOS_FREQ */ 401.65,
		/* ARGOS_POWER */ BaseArgosPower::POWER_350_MW,
		/* TR_NOM */ 60U,

		/* ARGOS_MODE */ BaseArgosMode::LEGACY,
		/* NTRY_PER_MESSAGE */ 0U,
		/* DUTY_CYCLE */ 0U,
		/* GNSS_EN */ (bool)true,
		/* DLOC_ARG_NOM */ 10*60U,

		/* ARGOS_DEPTH_PILE */ BaseDepthPile::DEPTH_PILE_16,
		/* _RESERVED_20 */ 0U,
		/* _RESERVED_21 */ 0U,
		// Seuils de qualite GNSS releves le 2026-08-31 (hAcc 5 -> 50 m, hDOP 2 -> 5),
		// pour les trois familles: nominal, batterie basse, hors zone.
		//
		// POURQUOI. Sur 13 positions reelles mesurees a La Reunion les 29 et 30 aout,
		// la MEILLEURE etait a 13,4 m et le meilleur hDOP a 1,97. Aux anciens defauts
		// (5 m / 2), AUCUNE des 13 ne passait: une balise sortie d usine allumait son
		// recepteur, obtenait une position, et la jetait — 13 fois sur 13, en
		// rapportant NO_FIX. Et comme GNSS_FASTLOC_MODE vaut 0 par defaut, il n y a
		// meme pas de repli degrade: la position est perdue, pas seulement declassee.
		// A 50 m / 5 — les valeurs que portent deja les templates turtle_gps et rspb —
		// 3 des 13 passent. Ce n est pas genereux, c est simplement realiste.
		//
		// Ne change QUE les balises remises a zero ou nouvellement provisionnees:
		// celles du terrain gardent leur valeur enregistree. Aucun bump de version de
		// configuration n est requis — aucun emplacement ne bouge (voir la regle plus
		// haut), et un bump inutile effacerait les identifiants LoRaWAN.
		/* GNSS_HDOPFILT_EN */ (bool)true,
		/* GNSS_HDOPFILT_THR */ 5U,
		/* GNSS_ACQ_TIMEOUT */ 120U,
		/* GNSS_NTRY */ 0U, // 0 = unlimited retries; otherwise cap before backing off to dloc_arg_nom (see gps_service.cpp)
		/* UNDERWATER_EN */ (bool)false,
		/* DRY_TIME_BEFORE_TX */ 0U,
		/* SAMPLING_UNDER_FREQ */ (double)1.0,
		/* LB_EN */ (bool)false,
		/* LB_THRESHOLD */ 10U,
		/* LB_ARGOS_POWER */ BaseArgosPower::POWER_350_MW,
		/* TR_LB */ 240U,
		/* LB_ARGOS_MODE */ BaseArgosMode::LEGACY,
		/* LB_ARGOS_DUTY_CYCLE */ 0U,
		/* LB_GNSS_EN */ (bool)true,
		/* DLOC_ARG_LB */ 60*60U,
		/* LB_GNSS_HDOPFILT_THR */ 5U,
		/* LB_ARGOS_DEPTH_PILE */ BaseDepthPile::DEPTH_PILE_1,
		/* LB_GNSS_ACQ_TIMEOUT */ 120U,
		/* SAMPLING_SURF_FREQ */ (double)10.0,
		/* PP_MIN_ELEVATION */ 15.0,
		/* PP_MAX_ELEVATION */ 90.0,
		/* PP_MIN_DURATION */ 30U,
		/* PP_MAX_PASSES */ 1000U,
		/* PP_LINEAR_MARGIN */ 300U,
		/* PP_COMP_STEP */ 10U,
		/* GNSS_COLD_ACQ_TIMEOUT */ 530U,
		/* GNSS_FIX_MODE */ BaseGNSSFixMode::AUTO,
		/* GNSS_DYN_MODEL */ BaseGNSSDynModel::PORTABLE,
		/* GNSS_HACCFILT_EN */ (bool)true,
		/* GNSS_HACCFILT_THR */ 50U,
		/* GNSS_MIN_NUM_FIXES */ 1U,
		/* GNSS_COLD_START_RETRY_PERIOD */ 60U,
		/* ARGOS_TIME_SYNC_BURST_EN */ (bool)true,
		/* LED_MODE */ BaseLEDMode::HRS_24,
		/* ARGOS_TX_JITTER_EN */ (bool)true,
		/* ARGOS_RX_EN */ (bool)true,
		/* ARGOS_RX_MAX_WINDOW */ 15U*60U,
		/* ARGOS_RX_AOP_UPDATE_PERIOD */ 90U,
		/* ARGOS_RX_COUNTER */ 0U,
		/* ARGOS_RX_TIME */ 0U,
		/* GNSS_ASSISTNOW_EN */ (bool)true,
		/* LB_GNSS_HACCFILT_THR */ 50U,
		/* LB_NTRY_PER_MESSAGE */ 4U,

		/* ZONE_TYPE */ BaseZoneType::CIRCLE,
		/* ZONE_ENABLE_OUT_OF_ZONE_DETECTION_MODE */ (bool)false,
		/* ZONE_ENABLE_ACTIVATION_DATE */ (bool)true,
		/* ZONE_ACTIVATION_DATE */ static_cast<std::time_t>(1577836800U), // 01/01/2020 00:00:00
		/* ZONE_ARGOS_DEPTH_PILE */ BaseDepthPile::DEPTH_PILE_1,
		/* _RESERVED_70 */ BaseArgosPower::POWER_350_MW,

		/* ZONE_ARGOS_REPETITION_SECONDS */ 240U,
		/* ZONE_ARGOS_MODE */ BaseArgosMode::LEGACY,

		/* ZONE_ARGOS_DUTY_CYCLE */ 0xFFFFFFU,
		/* ZONE_ARGOS_NTRY_PER_MESSAGE */ 0U,
		/* ZONE_GNSS_DELTA_ARG_LOC_ARGOS_SECONDS */ 3600U,
		/* ZONE_GNSS_HDOPFILT_THR */ 5U,
		/* ZONE_GNSS_HACCFILT_THR */ 50U,
		/* ZONE_GNSS_ACQ_TIMEOUT */ 240U,
		/* ZONE_CENTER_LONGITUDE */ -123.3925,
		/* ZONE_CENTER_LATITUDE */ -48.8752,
		/* ZONE_RADIUS */ 1000U,
		/* CERT_TX_ENABLE */ (bool)false,
		/* CERT_TX_PAYLOAD */ std::string("FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF"), // 27 bytes for long payload
		/* CERT_TX_MODULATION */ BaseArgosModulation::A2,
		/* CERT_TX_REPETITION */ 60U,
		/* HW_VERSION */ std::string(""),
		/* BATT_VOLTAGE */ (double)0,
		/* [88] SHUTDOWN_TIMER */ 0U,
		/* [89] BOOT_COUNTER */ 0U,
		/* [90] BOOT_COUNTER_MODULO */ 2U,
		/* [91] WAKEUP_PERIOD */ (unsigned int)(WAKEUP_PERIOD_DEFAULT),  // cast: -D passes a bare int; keep the BaseType variant on unsigned (index) or deserialize mismatches + reset-crashes
		/* [92] ARGOS_TCXO_WARMUP_TIME */ 5U,
		/* DEVICE_DECID */ 0U,
		/* GNSS_TRIGGER_ON_SURFACED */ (bool)true,
		/* GNSS_TRIGGER_ON_AXL_WAKEUP */ (bool)false,
		/* UNDERWATER_DETECT_SOURCE */ BaseUnderwaterDetectSource::SWS,
		/* [97] UNDERWATER_DETECT_THRESH */ (double)1.1,
		/* [98] PH_SENSOR_ENABLE */ (bool)false,
		/* [99] PH_SENSOR_PERIODIC */ 0U,
		/* [100] PH_SENSOR_VALUE */ (double)0.0,
		/* [101] SEA_TEMP_SENSOR_ENABLE */ (bool)false,
		/* [102] SEA_TEMP_SENSOR_PERIODIC */ 0U,
		/* [103] SEA_TEMP_SENSOR_VALUE */ (double)0.0,
		/* [104] ALS_SENSOR_ENABLE */ (bool)false,
		/* [105] ALS_SENSOR_PERIODIC */ 0U,
		/* [106] ALS_SENSOR_VALUE */ (double)0.0,
		/* [107] CDT_SENSOR_ENABLE */ (bool)false,
		/* [108] CDT_SENSOR_PERIODIC */ 0U,
		/* [109] CDT_SENSOR_CONDUCTIVITY_VALUE */ (double)0.0,
		/* [110] CDT_SENSOR_DEPTH_VALUE */ (double)0.0,
		/* [111] CDT_SENSOR_TEMPERATURE_VALUE */ (double)0.0,
		/* [112] THERMISTOR_SENSOR_ENABLE */ (bool)false,
		/* [113] THERMISTOR_SENSOR_PERIODIC */ 0U,
		/* [114] THERMISTOR_SENSOR_VALUE */ (double)0.0,
		/* [115] THERMISTOR_SENSOR_WAKEUP_THRESH */ (double)0.0,
		/* [116] THERMISTOR_SENSOR_WAKEUP_SAMPLES */ 0U,
		/* [117] _RESERVED_117 (was EXT_LED_MODE) */ BaseLEDMode::ALWAYS,  // Slot kept for flash-layout backward compat; no code reads it.
		/* [118] AXL_SENSOR_ENABLE */ (bool)false,
		/* [119] AXL_SENSOR_PERIODIC */ 0U,
		/* [120] AXL_SENSOR_WAKEUP_THRESH */ (double)0.0,
		/* [121] AXL_SENSOR_WAKEUP_SAMPLES */ 5U,
		/* [122] AXL_SENSOR_MEASUREMENT_RANGE */ 0U,
		/* [123] AXL_SENSOR_POWER_MODE */ 0U,
		/* [124] PRESSURE_SENSOR_ENABLE */ (bool)false,
		/* [125] PRESSURE_SENSOR_PERIODIC */ 0U,
		/* DEBUG_OUTPUT_MODE */ BaseDebugMode::USB_CDC,  // Default: USB CDC (was UART on Linkit V3)
		/* GNSS_ASSISTNOW_OFFLINE_EN */ (bool)false,
		/* UW_MAX_SAMPLES */ 1U,
		/* UW_MIN_DRY_SAMPLES */ 1U,
		/* UW_SAMPLE_GAP */ 1000U,
		/* UW_PIN_SAMPLE_DELAY_US */ 1000U,  // 1 ms initial RC charge — preserves pre-rename behavior (was UNP08=1 ms before rename to µs)
		/* GNSS_CONSTELLATION_MASK */ 0x0FU,  // GPS|GAL|GLO|BDS (M10Q factory default)
		/* GNSS_ORBMAXERR */ 300U,
		/* SWS_ANALOG_HYSTERESIS */ 4U,
		/* SWS_ANALOG_CALIB_INTERVAL */ 3600U,
		/* UW_MAX_DIVE_TIME */ 7200U,
		/* UW_MIN_SURFACE_TIME */ 5U,
		/* UW_DIVE_MODE_ENABLE */ (bool)false,
		/* UW_DIVE_MODE_START_TIME */ 0U,
		/* GNSS_MIN_CNO */ 10U,
		/* GNSS_MIN_ELEV */ 10U,
		/* __RESERVED_142 */ 0U,
		/* __RESERVED_143 */ 0U,
		/* __RESERVED_144 */ 0U,
		/* [145] LB_CRITICAL_THRESH */ 5U,
		/* [146] PRESSURE_SENSOR_LOGGING_MODE */ BasePressureSensorLoggingMode::ALWAYS,
		/* [147] GNSS_TRIGGER_COLD_START_ON_SURFACED */ (bool)false,
		/* [148] SEA_TEMP_SENSOR_ENABLE_TX_MODE */ BaseSensorEnableTxMode::OFF,
		/* [149] SEA_TEMP_SENSOR_ENABLE_TX_MAX_SAMPLES */ 1U,
		/* [150] SEA_TEMP_SENSOR_ENABLE_TX_SAMPLE_PERIOD */ 1000U,
		/* [151] PH_SENSOR_ENABLE_TX_MODE */ BaseSensorEnableTxMode::OFF,
		/* [152] PH_SENSOR_ENABLE_TX_MAX_SAMPLES */ 1U,
		/* [153] PH_SENSOR_ENABLE_TX_SAMPLE_PERIOD */ 1000U,
		/* [154] ALS_SENSOR_ENABLE_TX_MODE */ BaseSensorEnableTxMode::OFF,
		/* [155] ALS_SENSOR_ENABLE_TX_MAX_SAMPLES */ 1U,
		/* [156] ALS_SENSOR_ENABLE_TX_SAMPLE_PERIOD */ 1000U,
		/* [157] PRESSURE_SENSOR_ENABLE_TX_MODE */ BaseSensorEnableTxMode::OFF,
		/* [158] PRESSURE_SENSOR_ENABLE_TX_MAX_SAMPLES */ 1U,
		/* [159] PRESSURE_SENSOR_ENABLE_TX_SAMPLE_PERIOD */ 1000U,
		/* [160] AXL_SENSOR_ENABLE_TX_MODE */ BaseSensorEnableTxMode::OFF,
		/* [161] AXL_SENSOR_ENABLE_TX_MAX_SAMPLES */ 1U,
		/* [162] AXL_SENSOR_ENABLE_TX_SAMPLE_PERIOD */ 1000U,
		/* [163] THERMISTOR_SENSOR_ENABLE_TX_MODE */ BaseSensorEnableTxMode::OFF,
		/* [164] THERMISTOR_SENSOR_ENABLE_TX_MAX_SAMPLES */ 1U,
		/* [165] THERMISTOR_SENSOR_ENABLE_TX_SAMPLE_PERIOD */ 1000U,
		/* [166] CAM_ENABLE */ (bool)false,
		/* [167] CAM_TRIGGER_ON_SURFACED */ (bool)false,
		/* [168] CAM_TRIGGER_ON_AXL_WAKEUP */ (bool)false,
		/* [169] CAM_PERIOD_ON */ 1U * 60U,
		/* [170] CAM_PERIOD_OFF */ 5U * 60U,
		/* [171] LB_CAM_EN */ (bool)false,
		/* [172] ARGOS_SECKEY */ std::string(""),
		/* [173] ARGOS_RADIOCONF */ std::string(""),
		/* [174] SHUTDOWN_NTIME_SAT */ 0U,
		/* [175] LB_SHUTDOWN_NTIME_SAT */ 0U,
		/* [176] GNSS_SESSION_SINGLE_FIX */ (bool)false,
		/* [177] PRESSURE_SENSOR_FULL_SCALE */ BasePressureSensorFullScale::FS_1260,
		/* [178] GNSS_TOKEN */ std::string(""),
		/* [179] LAST_KNOWN_RTC */ 0U,
		/* [180] RTC_CURRENT_TIME */ 0U,
		/* [181] LORA_DEVEUI */ std::string(""),
		/* [182] LORA_APPEUI */ std::string(""),
		/* [183] LORA_APPKEY */ std::string(""),
		/* [184] LORA_DEVADDR */ std::string(""),
		/* [185] LORA_APPSKEY */ std::string(""),
		/* [186] LORA_NWKSKEY */ std::string(""),
		/* [187] LORA_NJM */ 1U,          // Default: OTAA
		/* [188] LORA_BAND */ 4U,         // Default: EU868
		/* [189] LORA_CLASS */ 0U,        // Default: Class A
		/* [190] LORA_DR */ 3U,           // Default: SF9/125kHz (best speed/range for marine)
		/* [191] LORA_ADR */ (bool)false, // Default: ADR OFF (mandatory for mobile devices)
		/* [192] LORA_TXP */ 0U,          // Default: Max TX power
		/* [193] LORA_CFM */ (bool)false,  // Default: Unconfirmed messages
		/* [194] LORA_FPORT */ 2U,        // Default: Application port 2
		/* [195] LORA_LP_MODE */ 1U,      // Default: 1=standby (fast wake ~10ms), 0=shutdown (0µA, slow wake ~2.5s)
		/* [196] SURFACING_BURST_INIT_S */ 5U,
		/* [197] SURFACING_BURST_STEP_S */ 1U,
		/* [198] SURFACING_BURST_MAX_S */ 30U,
		/* [199] MORTALITY_ENABLE */ (bool)false,
		/* [200] MORTALITY_ACTIVITY_THRESH */ 10U,
		/* [201] MORTALITY_TEMP_THRESH */ (double)25.0,
		/* [202] MORTALITY_GPS_DISTANCE_THRESH */ 50U,
		/* [203] MORTALITY_CONFIRM_DAYS */ 3U,
		/* [204] MORTALITY_DUTY_CYCLE_MODULO */ 0U,
		/* [205] MORTALITY_ORIGINAL_MODULO */ 0U,
		/* [206] RSPB_PACKET_FORMAT */ 0U,  // 0=RSPB_LONG (LDA2), 1=RSPB_SHORT (LDK)
		/* [207] ARGOS_RADIOCONF_LDK */ std::string("03921fb104b92859209b18abd009de96"),
		/* [208] ARGOS_RADIOCONF_LDA2 */ std::string("2c93600d6be3bac0ccfe9047c02c058e"),
		/* [209] ARGOS_RADIOCONF_VLDA4 */ std::string("550b4bec21009c7a7b5bebaa937cdb41"),
		/* [210] ARGOS_ADAPTIVE_MODULATION */ (bool)false,
		/* [211] MIN_SURFACE_CYCLE_INTERVAL_S */ 2700U,  // 45 min default cooldown
		/* [212] SURFACING_BURST_MAX_MSG */ 0U,  // 0 = unlimited Doppler messages per surfacing
		/* [213] COOLDOWN_TRIGGER_MODE */ 3U,  // 3=AFTER_LAST_TX (backward compatible)
		/* [214] SMD_LPM_MODE */ 0x01U,  // 0x01=NONE (safest, host cuts power)
		/* [215] SWS_DELAY_MIN_US */ 200U,    // Adaptive sample delay floor (µs)
		/* [216] SWS_DELAY_MAX_US */ 10000U,  // Adaptive sample delay ceiling (µs)
		/* [217] GNSS_ANO_STALE_DAYS */ 5U,   // ANO staleness threshold: 5 days (0=never discard)
		/* [218] GNSS_FASTLOC_MODE */ 0U,             // 0=OFF, 1=DEGRADED_PVT, 2=CLOUDLOCATE
		/* [219] GNSS_CLOUDLOCATE_FORMAT */ 0U,        // 0=MEASC12, 1=MEAS20, 2=MEAS50
		/* [220] AXL_FIFO_ENABLE */ (bool)false,       // false=single sample, true=FIFO batch averaging
		/* [221] AXL_FIFO_SAMPLE_COUNT */ 50U,         // 1-170 samples per batch
		/* [222] LED_HRS24_RTC_CUTOFF */ static_cast<std::time_t>(0U),  // 0=unset, auto-set by GPSService at first valid fix to (now+24h)
		/* [223] _RESERVED_223 */ 0U,                  // Was ARGOS_TX_NO_FIX_POLICY (removed 2026-07 — no-fix behavior hardwired); before that GNSS_BCKP_CHARGE_INT
		/* [224] _RESERVED_224 */ 0U,                  // Was ARGOS_LAST_KNOWN_MAX_AGE_S (removed 2026-07 with LAST_KNOWN policy); before that GNSS_BCKP_CHARGE_DUR
		/* [225] _RESERVED_225 */ (bool)false,         // Was GNSS_BCKP_CHARGE_UW_ONLY — deprecated 2026-05, slot reserved for flash compat
		/* [226] SMD_DEGRADED_MODE */ 0U,              // 0 = FAST timings (default); 1 = SAFE (set by SmdSat::degraded_mode_engage)
		/* [227] ARGOS_CACHED_MODULATION */ 0U,        // 0 = LDA2 (default), 1 = LDK, 2 = VLDA4 (mirrors SmdArgosModulation enum)
		/* [228] GNSS_REUSE_FIX_MAX_AGE_S */ 86400U,   // 24 h: cached depth-pile fix older than this falls back to Doppler-only (REUSE_LAST → OFF)
		/* [229] RATE_LIMIT_EN */ (bool)false,         // Plan 1 step 2 — disabled by default
		/* [230] RATE_LIMIT_WINDOW_S */ 3600U,         // 60 min sliding window
		/* [231] RATE_LIMIT_MAX_TX */ 10U,             // 10 TX max inside the window
		/* [232] HAULED_DETECT_EN */ (bool)false,      // Plan 1 step 3 — disabled by default
		/* [233] HAULED_IDLE_THRESHOLD_H */ 24U,       // 24 h dry → HAULED
		/* [234] HAULED_RETURN_EVENTS */ 3U,           // 3 dives → AT_SEA
		/* [235] HAULED_ARGOS_MODE */ BaseArgosMode::LEGACY,
		/* [236] HAULED_TR_NOM */ 7200U,               // 2 h interval when hauled
		/* [237] HAULED_GNSS_EN */ (bool)false,        // GNSS off when hauled by default
		/* [238] HAULED_GNSS_STRAT */ 1U,              // BaseGnssStrategy::REUSE_LAST (stored as uint per BaseMap)
		/* [239] GNSS_CLOUDLOCATE_ALWAYS */ (bool)false,  // false=CloudLocate only on cold-start surfaces (default, battery-friendly); true=raw-meas captured at every SURFACING_BURST surface (short-surface turtle fallback)
		/* [240] GNSS_DEEP_IDLE_AFTER_OFF_S */ 0U,     // 0=disabled (immediate poweroff, default — no behavior change); 0xFFFFFFFF=never poweroff (rail always on, M10Q in deep-idle); else=duration in seconds. Replaces deprecated GNSS_BCKP_CHARGE_* (slots 223-225).
		/* [241] GNSS_CLOUDLOCATE_ONLY */ (bool)false, // FAST3b: when true + FASTLOC_MODE=CLOUDLOCATE, end GNSS session on first raw measurement (skip full PVT wait). Off by default to preserve current behavior.
		/* [242] GNSS_COLD_START_AFTER_NTRY */ 0U,     // 0=disabled (default, no regression). N>0=force a real cold start (BBR wipe + cold timeout) after N consecutive GNSS sessions with no fix, to recover a receiver stuck on stale assistance.
		/* [243] ARGOS_BLIND_EN */ (bool)false,        // BASIC by default (no behavior change)
		/* [244] ARGOS_BLIND_RETX_NB */ 4U,            // module retransmissions per blind burst
		/* [245] ARGOS_BLIND_RETX_PERIOD_S */ 60U,     // seconds between module retransmissions
		/* [246] SAT_PREPASS_EN */ (bool)false,        // off by default: no behavior change
		/* [247] SAT_AOP_MAX_AGE_DAYS */ 14U,          // conservative while waiting for the official Kineis value
		/* [248] SAT_PREPASS_MAX_WAIT_S */ 0U,         // 0 = no safety guard (current behavior)
		/* [249] SAT_AOP_VALID */ (bool)false,
		/* [250] SAT_AOP_AGE_S */ 0U,
		/* [251] SAT_NEXT_PASS_TS */ 0U,
		/* [252] SAT_LAST_PASS_TS */ 0U,
		/* [253] MOORED_DETECT_EN */ (bool)false,   // off by default: no behaviour change for turtle / RSPB deployments
		/* [254] MOORED_RADIUS_M */ 150U,           // a vessel swinging on its mooring stays well inside 150 m
		/* [255] MOORED_ENTER_FIXES */ 3U,          // 3 consecutive stationary fixes before believing it
		/* [256] MOORED_EXIT_EVENTS */ 2U,          // 2 accelerometer wake-ups to leave (1 would trip on a single wave)
		/* [257] MOORED_AXL_HOLDOFF_S */ 900U,      // 15 min: bounds wake-up-driven GNSS acquisitions in swell
		/* [258] MOORED_DLOC */ 3600U,              // 1 h GNSS acquisition period while moored (AQPERIOD code table)
		/* [259] MOORED_TR_NOM */ 3600U,            // 1 h TX interval while moored
		/* [260] MOORED_GNSS_EN */ (bool)true,      // keep fixing: GNSS is what detects a slow drift off the mooring
		/* [261] MOORED_TX_LAST_POS */ (bool)true,  // moored heartbeat carries the last known position (LoRa)
		/* [262] MOORED_STATE */ 0U,                // read-only mirror, written by MooredModeService
		/* [263] PP_MIN_CULMINATION */ 0U,         // TX: keep every pass the elevation filter accepted (previous hardcoded value)
		/* [264] PP_RX_MIN_CULMINATION */ 20U,     // RX: a downlink needs a good pass — a grazing one wastes the whole window (previous hardcoded value)
		/* [265] PP_POSITION_MARGIN_KM */ 0U,      // no position uncertainty by default
	};
	// clang-format on

	// The one guard the compiler can give us on this table. default_params used to
	// be a std::array<BaseType, MAX_CONFIG_ITEMS>: an aggregate initialiser with one
	// entry missing compiled WITHOUT A WORD, the absent slot became BaseType{} ==
	// std::string(""), and config_store_fs.hpp then factory-reset that parameter on
	// every boot -- silently, on a sealed beacon. Letting the compiler count the
	// initialisers turns that into a build failure.
	//
	// This holds in EVERY build configuration: __PARAM_SIZE is a fixed literal, and
	// neither this table nor param_map[] carries a #if. The optional sensors are
	// omitted from the ParamID enum only, and every remaining member keeps an
	// explicit index, so disabling one shifts nothing.
	static_assert(sizeof(default_params) / sizeof(default_params[0]) == MAX_CONFIG_ITEMS,
	              "default_params must have exactly one entry per ParamID slot");
	// hand-aligned AOP record, one field per line
	// clang-format off
	static inline const BasePassPredict default_prepass = {
		/* version_code */ m_config_version_code_aop,
		/* num_records */  25,
		{
			{ 0x0B, (SatDownlinkStatus_t)1, (SatUplinkStatus_t)4, { 2026, 8, 10, 4, 46, 56 }, 7006.705, 97.897, 214.435, -24.328,  97.3098,  0.0 },
			{ 0x16, (SatDownlinkStatus_t)1, (SatUplinkStatus_t)4, { 2026, 8, 10, 4, 22, 19 }, 7006.705, 97.897, 220.586, -24.328,  97.3098,  0.0 },
			{ 0x1D, (SatDownlinkStatus_t)0, (SatUplinkStatus_t)4, { 2026, 8, 10, 5, 10, 45 }, 7006.705, 97.897, 208.480, -24.328,  97.3098,  0.0 },
			{ 0x2C, (SatDownlinkStatus_t)1, (SatUplinkStatus_t)4, { 2026, 8, 10, 3, 58, 26 }, 7006.705, 97.897, 226.558, -24.328,  97.3098,  0.0 },
			{ 0x31, (SatDownlinkStatus_t)1, (SatUplinkStatus_t)4, { 2026, 8, 10, 5, 14, 28 }, 7029.538, 98.132, 240.368, -24.445,  97.7859,  0.0 },
			{ 0x3A, (SatDownlinkStatus_t)1, (SatUplinkStatus_t)4, { 2026, 8, 10, 4, 41, 57 }, 7029.498, 97.970, 244.901, -24.447,  97.7851, -6.8 },
			{ 0x45, (SatDownlinkStatus_t)1, (SatUplinkStatus_t)4, { 2026, 8, 10, 4,  9, 31 }, 7029.583, 98.127, 255.580, -24.446,  97.7868,  0.0 },
			{ 0x4E, (SatDownlinkStatus_t)0, (SatUplinkStatus_t)4, { 2026, 8, 10, 5, 26,  8 }, 7022.100, 97.932, 233.381, -24.408,  97.6309, -8.4 },
			{ 0x53, (SatDownlinkStatus_t)1, (SatUplinkStatus_t)4, { 2026, 8, 10, 5, 10, 14 }, 7017.951, 97.899, 236.432, -24.387,  97.5444, -7.4 },
			{ 0x58, (SatDownlinkStatus_t)1, (SatUplinkStatus_t)4, { 2026, 8, 10, 4, 44, 21 }, 7033.361, 98.047, 281.837, -24.466,  97.8655,  0.0 },
			{ 0x62, (SatDownlinkStatus_t)1, (SatUplinkStatus_t)4, { 2026, 8, 10, 4, 19, 27 }, 7033.361, 98.047, 288.060, -24.466,  97.8655,  0.0 },
			{ 0x74, (SatDownlinkStatus_t)1, (SatUplinkStatus_t)4, { 2026, 8, 10, 5, 32, 46 }, 7033.361, 98.047, 269.732, -24.466,  97.8655,  0.0 },
			{ 0x7F, (SatDownlinkStatus_t)1, (SatUplinkStatus_t)4, { 2026, 8, 10, 5,  8, 11 }, 7033.361, 98.047, 275.880, -24.466,  97.8655,  0.0 },
			{ 0x81, (SatDownlinkStatus_t)1, (SatUplinkStatus_t)4, { 2026, 8, 10, 4, 38, 49 }, 7033.236, 97.921, 316.416, -24.467,  97.8630,  0.0 },
			{ 0x8A, (SatDownlinkStatus_t)0, (SatUplinkStatus_t)4, { 2026, 8, 10, 4, 14,  3 }, 7033.236, 97.921, 322.610, -24.467,  97.8630,  0.0 },
			{ 0x9C, (SatDownlinkStatus_t)1, (SatUplinkStatus_t)4, { 2026, 8, 10, 5, 27, 30 }, 7033.236, 97.921, 304.245, -24.467,  97.8630,  0.0 },
			{ 0xA6, (SatDownlinkStatus_t)1, (SatUplinkStatus_t)4, { 2026, 8, 10, 5,  2, 59 }, 7033.236, 97.921, 310.375, -24.467,  97.8630,  0.0 },
			{ 0xAD, (SatDownlinkStatus_t)1, (SatUplinkStatus_t)4, { 2026, 8, 10, 4, 18, 53 }, 7033.123, 97.904, 356.502, -24.466,  97.8617,  0.0 },
			{ 0xBB, (SatDownlinkStatus_t)1, (SatUplinkStatus_t)4, { 2026, 8, 10, 5, 24,  2 }, 7033.123, 97.904, 340.214, -24.466,  97.8617,  0.0 },
			{ 0xCF, (SatDownlinkStatus_t)1, (SatUplinkStatus_t)4, { 2026, 8, 10, 4, 50, 26 }, 7033.123, 97.904, 348.613, -24.466,  97.8617,  0.0 },
			{ 0xF1, (SatDownlinkStatus_t)0, (SatUplinkStatus_t)3, { 2026, 8, 10, 5, 26,  5 }, 7129.183, 98.401, 172.883, -24.968,  99.8714, -1.1 },
			{ 0xF2, (SatDownlinkStatus_t)0, (SatUplinkStatus_t)3, { 2026, 8, 10, 4,  2, 17 }, 7115.065, 98.360, 300.421, -24.894,  99.5748,  0.0 },
			{ 0xF9, (SatDownlinkStatus_t)0, (SatUplinkStatus_t)1, { 2026, 8, 10, 4, 18,  8 }, 7200.006, 98.652, 249.390, -25.341, 101.3622,  0.0 },
			{ 0xFB, (SatDownlinkStatus_t)1, (SatUplinkStatus_t)1, { 2026, 8, 10, 4, 42, 11 }, 7199.813, 98.666, 251.570, -25.340, 101.3582,  0.0 },
			{ 0xFD, (SatDownlinkStatus_t)1, (SatUplinkStatus_t)1, { 2026, 8, 10, 5, 28, 51 }, 7161.616, 98.561,   7.186, -25.138, 100.5531,  0.0 },
		}
	};
	// clang-format on

	std::array<BaseType, MAX_CONFIG_ITEMS> m_params;
	bool m_credentials_dirty = true;  // true on first boot to ensure initial write
	uint8_t m_battery_level = 0;
	uint16_t m_battery_voltage = 0;
	bool m_is_battery_level_low = false;
	GPSLogEntry m_last_gps_log_entry;
	GPSLogEntry m_last_fastloc_log_entry;
	ConfigMode m_last_config_mode;

	/// @brief HM-2 audit fix: track config-mode transitions with an explicit
	/// HAULED-exit log. Previously the cascade logged "NORMAL/OOZ/LB detected"
	/// when exiting HAULED but never named the exit transition itself, making
	/// post-deploy forensics ambiguous (was it a fresh boot or a HAULED exit?).
	/// Helper called by every transition site in get_gnss/argos_configuration.
	/// Idempotent: only logs on actual change.
	void mark_config_mode(ConfigMode new_mode) {
		if (m_last_config_mode == new_mode) return;
		if (m_last_config_mode == ConfigMode::HAULED) {
			DEBUG_INFO("ConfigurationStore: HAULED mode EXITED -> %s",
			           new_mode == ConfigMode::LOW_BATTERY   ? "LOW_BATTERY"
			           : new_mode == ConfigMode::OUT_OF_ZONE ? "OUT_OF_ZONE"
			           : new_mode == ConfigMode::MOORED      ? "MOORED"
			           : new_mode == ConfigMode::NORMAL      ? "NORMAL"
			                                                 : "?");
#if VALIDATION_LOG_ENABLE
			DEBUG_INFO("[VAL-HAULED] mode_exit t=%u to=%d", rtc && rtc->is_set() ? (unsigned int)rtc->gettime() : 0U,
			           (int)new_mode);
#endif
		}
		switch (new_mode) {
		case ConfigMode::LOW_BATTERY: DEBUG_INFO("ConfigurationStore: LOW_BATTERY mode detected"); break;
		case ConfigMode::OUT_OF_ZONE: DEBUG_INFO("ConfigurationStore: OUT_OF_ZONE mode detected"); break;
		case ConfigMode::NORMAL: DEBUG_INFO("ConfigurationStore: NORMAL mode detected"); break;
		case ConfigMode::HAULED: /* logged inline as "engaged (GNSS)" / "engaged (Argos)" */ break;
		case ConfigMode::MOORED: /* logged inline as "engaged (GNSS)" / "engaged (Argos)" */ break;
		default: break;
		}
		m_last_config_mode = new_mode;
	}

	virtual void serialize_config() = 0;
	virtual void update_battery_level() = 0;

private:
	static const inline unsigned int SECONDS_PER_MINUTE = 60;
	static const inline unsigned int SECONDS_PER_HOUR = 3600;

	BaseDeltaTimeLoc calc_delta_time_loc(unsigned int dloc_arg_nom) {
		if (dloc_arg_nom >= (24 * SECONDS_PER_HOUR)) {
			return BaseDeltaTimeLoc::DELTA_T_24HR;
		} else if (dloc_arg_nom >= (12 * SECONDS_PER_HOUR)) {
			return BaseDeltaTimeLoc::DELTA_T_12HR;
		} else if (dloc_arg_nom >= (6 * SECONDS_PER_HOUR)) {
			return BaseDeltaTimeLoc::DELTA_T_6HR;
		} else if (dloc_arg_nom >= (4 * SECONDS_PER_HOUR)) {
			return BaseDeltaTimeLoc::DELTA_T_4HR;
		} else if (dloc_arg_nom >= (3 * SECONDS_PER_HOUR)) {
			return BaseDeltaTimeLoc::DELTA_T_3HR;
		} else if (dloc_arg_nom >= (2 * SECONDS_PER_HOUR)) {
			return BaseDeltaTimeLoc::DELTA_T_2HR;
		} else if (dloc_arg_nom >= (1 * SECONDS_PER_HOUR)) {
			return BaseDeltaTimeLoc::DELTA_T_1HR;
		} else if (dloc_arg_nom >= (45 * SECONDS_PER_MINUTE)) {
			return BaseDeltaTimeLoc::DELTA_T_45MIN;
		} else if (dloc_arg_nom >= (30 * SECONDS_PER_MINUTE)) {
			return BaseDeltaTimeLoc::DELTA_T_30MIN;
		} else if (dloc_arg_nom >= (20 * SECONDS_PER_MINUTE)) {
			return BaseDeltaTimeLoc::DELTA_T_20MIN;
		} else if (dloc_arg_nom >= (15 * SECONDS_PER_MINUTE)) {
			return BaseDeltaTimeLoc::DELTA_T_15MIN;
		} else if (dloc_arg_nom >= (10 * SECONDS_PER_MINUTE)) {
			return BaseDeltaTimeLoc::DELTA_T_10MIN;
		} else if (dloc_arg_nom >= (5 * SECONDS_PER_MINUTE)) {
			return BaseDeltaTimeLoc::DELTA_T_5MIN;
		} else if (dloc_arg_nom >= (2 * SECONDS_PER_MINUTE)) {
			return BaseDeltaTimeLoc::DELTA_T_2MIN;
		} else {
			return BaseDeltaTimeLoc::DELTA_T_1MIN;
		}
	}

public:
	ConfigurationStore() {
		m_last_gps_log_entry.info.valid = 0;      // Mark last GPS entry as invalid
		m_last_fastloc_log_entry.info.valid = 0;  // Mark last Fastloc entry as invalid
		m_last_config_mode = ConfigMode::NORMAL;
	}

	virtual ~ConfigurationStore() {}

	/// @brief Initialize config store — deserialize from flash or create defaults.
	virtual void init() = 0;

	/// @brief Check if configuration is valid (successfully loaded from flash).
	virtual bool is_valid() = 0;

	/// @brief Factory reset — reformat flash, preserve protected params (DECID, HEXID).
	virtual void factory_reset() = 0;

	/// @brief Read Argos pass prediction data from flash.
	virtual BasePassPredict &read_pass_predict() = 0;

	/// @brief Write Argos pass prediction data to flash.
	virtual void write_pass_predict(BasePassPredict &value) = 0;

	/// @brief Warn if the low-battery threshold pair cannot do its job.
	///
	/// Both LB_THRESHOLD (LBP02) and LB_CRITICAL_THRESH (LBP12) are percentages
	/// of state-of-charge accepting 0..100 independently, so nothing in the
	/// parameter table catches an inconsistent pair.
	///
	/// What is actually at stake: the ASYNCHRONOUS shutdown path
	/// (BatteryMonitor::actuate_events -> BatteryMonitorEventVoltageCritical ->
	/// BatteryCriticalState) compares against LB_CRITICAL_THRESH directly and
	/// keeps working whatever these two are set to. What depends on the pair is
	/// the SYNCHRONOUS pre-transmission guard in ArgosTxService/LoRaTxService,
	/// which refuses to start a high-current transmission on an already-critical
	/// battery. That guard sits inside `if (argos_config.is_lb)`, and
	/// `is_lb = LB_EN && SOC < LB_THRESHOLD` — so it only ever runs when LB mode
	/// is enabled AND the low-battery band has been entered first.
	///
	/// Hence: silent when LB_EN is off (the guard is inactive by design, and
	/// LB_EN defaults to false — warning there would be noise on a default
	/// config). When LB_EN is on, LB_THRESHOLD must be strictly greater than
	/// LB_CRITICAL_THRESH, and the two failure modes are reported distinctly
	/// because they are not the same mistake.
	///
	/// Called at boot (init_battery) and after every DTE config write. This is a
	/// warning: the write is not blocked.
	/// @return true when the pair can do its job (or LB mode is off).
	bool check_battery_thresholds() {
		bool lb_en;
		unsigned int lb, crit;
		try {
			lb_en = read_param<bool>(ParamID::LB_EN);
			lb = read_param<unsigned int>(ParamID::LB_THRESHOLD);
			crit = read_param<unsigned int>(ParamID::LB_CRITICAL_THRESH);
		} catch (...) {
			return true;  // store not readable yet — not our problem to report
		}

		if (!lb_en) return true;  // LB mode disabled: the gated guard is inactive by design

		if (lb > crit) return true;

		if (lb < crit)
			DEBUG_WARN("ConfigurationStore: LB_THRESHOLD (LBP02=%u%%) is BELOW "
			           "LB_CRITICAL_THRESH (LBP12=%u%%) — the pre-transmission critical-battery "
			           "guard only runs once the low-battery band is entered, so it will not "
			           "fire until %u%% instead of %u%%",
			           lb, crit, lb, crit);
		else
			DEBUG_WARN("ConfigurationStore: LB_THRESHOLD (LBP02) equals LB_CRITICAL_THRESH "
			           "(LBP12=%u%%) — there is no low-battery band at all, so LB mode never "
			           "takes effect before the critical shutdown",
			           crit);
		return false;
	}

	/// @brief Read a configuration parameter by ID.
	/// @tparam T  Expected parameter type (e.g., unsigned int, bool, std::string).
	/// @throws CONFIG_STORE_CORRUPTED if store is invalid or type mismatch.
	template <typename T> T &read_param(ParamID param_id) {
		try {
			bool b_is_valid = false;

			// These parameters must always be accessible
			if (param_id == ParamID::BATT_SOC) {
				update_battery_level();
				m_params.at((unsigned)param_id) = (unsigned int)m_battery_level;
				b_is_valid = true;
			} else if (param_id == ParamID::FW_APP_VERSION) {
				m_params.at((unsigned)param_id) = FW_APP_VERSION_STR;
				b_is_valid = true;
			} else if (param_id == ParamID::HW_VERSION) {
				m_params.at((unsigned)param_id) = PMU::hardware_version();
				b_is_valid = true;
			} else if (param_id == ParamID::ARGOS_DECID) {
				b_is_valid = true;
			} else if (param_id == ParamID::ARGOS_HEXID) {
				b_is_valid = true;
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
			} else if (param_id == ParamID::ARGOS_SECKEY) {
				b_is_valid = true;
#endif
			} else if (param_id == ParamID::ARGOS_RADIOCONF) {
				b_is_valid = true;
#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
			} else if (param_id == ParamID::LORA_DEVEUI || param_id == ParamID::LORA_APPEUI
			           || param_id == ParamID::LORA_APPKEY || param_id == ParamID::LORA_DEVADDR
			           || param_id == ParamID::LORA_APPSKEY || param_id == ParamID::LORA_NWKSKEY) {
				b_is_valid = true;
#endif
			} else if (param_id == ParamID::DEVICE_MODEL) {
				m_params.at((unsigned)param_id) = DEVICE_MODEL_NAME;
				b_is_valid = true;
			} else if (param_id == ParamID::BATT_VOLTAGE) {
				update_battery_level();
				m_params.at((unsigned)param_id) = (double)m_battery_voltage / 1000.0;
				b_is_valid = true;
			} else if (param_id == ParamID::DEVICE_DECID) {
				m_params.at((unsigned)param_id) = (unsigned int)PMU::device_identifier();
				b_is_valid = true;
			}
#if ENABLE_ALS_SENSOR
			else if (param_id == ParamID::ALS_SENSOR_VALUE) {
				try {
					Sensor &s = SensorManager::find_by_name("ALS");
					m_params.at((unsigned)param_id) = s.read(1);
				} catch (...) {
					m_params.at((unsigned)param_id) = (double)std::nan("");
				}
				b_is_valid = true;
			}
#endif
#if ENABLE_PH_SENSOR
			else if (param_id == ParamID::PH_SENSOR_VALUE) {
				try {
					Sensor &s = SensorManager::find_by_name("PH");
					m_params.at((unsigned)param_id) = s.read();
				} catch (...) {
					m_params.at((unsigned)param_id) = (double)std::nan("");
				}
				b_is_valid = true;
			}
#endif
#if ENABLE_SEA_TEMP_SENSOR
			else if (param_id == ParamID::SEA_TEMP_SENSOR_VALUE) {
				// Sea temp sensor can be either RTD or TSYS01
				try {
					try {
						Sensor &s = SensorManager::find_by_name("RTD");
						m_params.at((unsigned)param_id) = s.read();
					} catch (...) {
						Sensor &s = SensorManager::find_by_name("TSYS01");
						m_params.at((unsigned)param_id) = s.read();
					}
				} catch (...) {
					m_params.at((unsigned)param_id) = (double)std::nan("");
				}
				b_is_valid = true;
			}
#endif
#if ENABLE_THERMISTOR_SENSOR
			else if (param_id == ParamID::THERMISTOR_SENSOR_VALUE) {
				try {
					Sensor &s = SensorManager::find_by_name("THERMISTOR");
					m_params.at((unsigned)param_id) = s.read();
				} catch (...) {
					m_params.at((unsigned)param_id) = (double)std::nan("");
				}
				b_is_valid = true;
			}
#endif
#if ENABLE_CDT_SENSOR
			else if (param_id == ParamID::CDT_SENSOR_CONDUCTIVITY_VALUE) {
				try {
					Sensor &s = SensorManager::find_by_name("CDT");
					m_params.at((unsigned)param_id) = s.read(0);
				} catch (...) {
					m_params.at((unsigned)param_id) = (double)std::nan("");
				}
				b_is_valid = true;
			} else if (param_id == ParamID::CDT_SENSOR_DEPTH_VALUE) {
				try {
					Sensor &s = SensorManager::find_by_name("CDT");
					m_params.at((unsigned)param_id) = s.read(1);
				} catch (...) {
					m_params.at((unsigned)param_id) = (double)std::nan("");
				}
				b_is_valid = true;
			} else if (param_id == ParamID::CDT_SENSOR_TEMPERATURE_VALUE) {
				try {
					Sensor &s = SensorManager::find_by_name("CDT");
					m_params.at((unsigned)param_id) = s.read(2);
				} catch (...) {
					m_params.at((unsigned)param_id) = (double)std::nan("");
				}
				b_is_valid = true;
			}
#endif
			else {
				b_is_valid = is_valid();
			}

			if (b_is_valid) {
				if constexpr (std::is_same<T, BaseType>::value) {
					return m_params.at((unsigned)param_id);
				} else {
					return std::get<T>(m_params.at((unsigned)param_id));
				};
			} else {
				throw CONFIG_STORE_CORRUPTED;
			}
		} catch (...) {
			throw CONFIG_STORE_CORRUPTED;
		}
	}

	/// @brief Write a configuration parameter by ID.
	/// @tparam T  Parameter value type.
	/// @note Marks credentials dirty if DECID/HEXID/SECKEY/RADIOCONF changes.
	template <typename T> void write_param(ParamID param_id, const T &value) {
		try {
			if (is_valid()) {
				m_params.at((unsigned)param_id) = value;
				// Mark credentials dirty when credential params change
				if (param_id == ParamID::ARGOS_DECID || param_id == ParamID::ARGOS_HEXID ||
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
				    param_id == ParamID::ARGOS_SECKEY ||
#endif
				    param_id == ParamID::ARGOS_RADIOCONF) {
					m_credentials_dirty = true;
				}
			} else
				throw CONFIG_STORE_CORRUPTED;
		} catch (...) {
			throw CONFIG_STORE_CORRUPTED;
		}
	}

	/// @brief Check if credential params have been modified since last SMD write.
	bool is_credentials_dirty() const { return m_credentials_dirty; }

	/// @brief Clear credentials dirty flag (called after SMD credential write).
	void clear_credentials_dirty() { m_credentials_dirty = false; }

	/// @brief Force credentials re-push to satellite module on next read/TX.
	/// @note Used by SATVF force-write path to re-trigger state_load_kmac without
	///       rewriting individual params.
	void mark_credentials_dirty() { m_credentials_dirty = true; }

	/// @brief Persist all parameters to flash.
	void save_params() {
		try {
			serialize_config();
		} catch (...) {
			throw CONFIG_STORE_CORRUPTED;
		}
	}

	/// @brief Update cached last GPS fix (used for zone exclusion calculation).
	void notify_gps_location(GPSLogEntry &gps_location) { m_last_gps_log_entry = gps_location; }

	/// @brief Get the last known GPS fix.
	const GPSLogEntry &get_last_gps_entry() const { return m_last_gps_log_entry; }

	/// @brief Update cached last Fastloc / degraded-PVT position.
	/// Used by Phase-1 surfacing burst to TX the most recent known position
	/// when no live fix is available this surface.
	void notify_fastloc_location(GPSLogEntry &gps_location) { m_last_fastloc_log_entry = gps_location; }

	/// @brief Get the last known Fastloc / degraded-PVT position.
	const GPSLogEntry &get_last_fastloc_entry() const { return m_last_fastloc_log_entry; }

	/// @brief Check if device is outside the configured zone (haversine distance).
	bool is_zone_exclusion() {
		if (read_param<bool>(ParamID::ZONE_ENABLE_OUT_OF_ZONE_DETECTION_MODE)
		    && read_param<BaseZoneType>(ParamID::ZONE_TYPE) == BaseZoneType::CIRCLE
		    && m_last_gps_log_entry.info.valid) {
			DEBUG_TRACE("ConfigurationStore::is_zone_exclusion: enabled with valid GPS fix");

			if (!read_param<bool>(ParamID::ZONE_ENABLE_ACTIVATION_DATE)
			    || (read_param<std::time_t>(ParamID::ZONE_ACTIVATION_DATE) <= convert_epochtime(
			            m_last_gps_log_entry.info.year, m_last_gps_log_entry.info.month, m_last_gps_log_entry.info.day,
			            m_last_gps_log_entry.info.hour, m_last_gps_log_entry.info.min, 0))) {
				// Compute distance between two points of longitude and latitude using haversine formula
				double d_km = haversine_distance(read_param<double>(ParamID::ZONE_CENTER_LONGITUDE),
				                                 read_param<double>(ParamID::ZONE_CENTER_LATITUDE),
				                                 m_last_gps_log_entry.info.lon, m_last_gps_log_entry.info.lat);

				// Check if outside zone radius for exclusion parameter triggering
				if (d_km > ((double)read_param<unsigned int>(ParamID::ZONE_RADIUS) / (double)1000)) {
					DEBUG_TRACE("ConfigurationStore::is_zone_exclusion: activation criteria met | d_km = %f", d_km);
					return true;
				}
				DEBUG_TRACE("ConfigurationStore::is_zone_exclusion: activation criteria not met | d_km = %f", d_km);
				return false;
			}
		}

		DEBUG_TRACE("ConfigurationStore::is_zone_exclusion: activation criteria not met");
		return false;
	}

	/// @brief Populate GNSSConfig struct from current params (handles NORMAL/LB/ZONE modes).
	void get_gnss_configuration(GNSSConfig &gnss_config) {
		auto cert_tx_enable = read_param<bool>(ParamID::CERT_TX_ENABLE);
		auto lb_en = read_param<bool>(ParamID::LB_EN);
		update_battery_level();

		gnss_config.battery_voltage = m_battery_voltage;
		gnss_config.is_out_of_zone = is_zone_exclusion();
		gnss_config.is_lb = false;

		// Predict whether the HAULED override is going to engage. Used to gate
		// "NORMAL/OUT_OF_ZONE mode detected" logs in the cascade below — without
		// this guard, every call alternates m_last_config_mode NORMAL → HAULED
		// and logs both transitions on each tick (2-line spam ~1 Hz on the
		// observed field log of 2026-05-23).
		HauledModeService::evaluate();
		bool hauled_will_override = !(lb_en && m_is_battery_level_low) && HauledModeService::is_hauled()
		                            && read_param<bool>(ParamID::HAULED_DETECT_EN);

		// Same prediction for the MOORED override (2026-08). Priority sits below
		// HAULED and above OUT_OF_ZONE: a vessel lying still far from its zone
		// should stay economical, so MOORED wins over the zone variant.
		MooredModeService::evaluate();
		bool moored_will_override = !(lb_en && m_is_battery_level_low) && !hauled_will_override
		                            && MooredModeService::is_moored() && read_param<bool>(ParamID::MOORED_DETECT_EN);

		if (lb_en && m_is_battery_level_low) {
			// Use LB mode which takes priority
			gnss_config.is_lb = true;
			gnss_config.enable = read_param<bool>(ParamID::LB_GNSS_EN);
			gnss_config.dloc_arg_nom = read_param<unsigned int>(ParamID::DLOC_ARG_LB);
			gnss_config.acquisition_timeout = read_param<unsigned int>(ParamID::LB_GNSS_ACQ_TIMEOUT);
			gnss_config.acquisition_timeout_cold_start = read_param<unsigned int>(ParamID::GNSS_COLD_ACQ_TIMEOUT);
			gnss_config.hdop_filter_enable = read_param<bool>(ParamID::GNSS_HDOPFILT_EN);
			gnss_config.hdop_filter_threshold = read_param<unsigned int>(ParamID::LB_GNSS_HDOPFILT_THR);
			gnss_config.hacc_filter_enable = read_param<bool>(ParamID::GNSS_HACCFILT_EN);
			gnss_config.hacc_filter_threshold = read_param<unsigned int>(ParamID::LB_GNSS_HACCFILT_THR);
			gnss_config.underwater_en = read_param<bool>(ParamID::UNDERWATER_EN);
			gnss_config.fix_mode = read_param<BaseGNSSFixMode>(ParamID::GNSS_FIX_MODE);
			gnss_config.dyn_model = read_param<BaseGNSSDynModel>(ParamID::GNSS_DYN_MODEL);
			gnss_config.min_num_fixes = read_param<unsigned int>(ParamID::GNSS_MIN_NUM_FIXES);
			gnss_config.cold_start_retry_period = read_param<unsigned int>(ParamID::GNSS_COLD_START_RETRY_PERIOD);
			gnss_config.assistnow_enable = read_param<bool>(ParamID::GNSS_ASSISTNOW_EN);
			gnss_config.trigger_on_surfaced = read_param<bool>(ParamID::GNSS_TRIGGER_ON_SURFACED);
			gnss_config.assistnow_offline_enable = read_param<bool>(ParamID::GNSS_ASSISTNOW_OFFLINE_EN);
			gnss_config.constellation_mask = read_param<unsigned int>(ParamID::GNSS_CONSTELLATION_MASK);
			gnss_config.orbmaxerr = read_param<unsigned int>(ParamID::GNSS_ORBMAXERR);
			gnss_config.min_cno = read_param<unsigned int>(ParamID::GNSS_MIN_CNO);
			gnss_config.min_elev = read_param<unsigned int>(ParamID::GNSS_MIN_ELEV);
			gnss_config.ano_stale_days = read_param<unsigned int>(ParamID::GNSS_ANO_STALE_DAYS);

			mark_config_mode(ConfigMode::LOW_BATTERY);

		} else if (gnss_config.is_out_of_zone) {
			gnss_config.enable = read_param<bool>(ParamID::GNSS_EN);
			gnss_config.dloc_arg_nom = read_param<unsigned int>(ParamID::ZONE_GNSS_DELTA_ARG_LOC_ARGOS_SECONDS);
			gnss_config.hdop_filter_enable = read_param<bool>(ParamID::GNSS_HDOPFILT_EN);
			gnss_config.hacc_filter_enable = read_param<bool>(ParamID::GNSS_HACCFILT_EN);
			gnss_config.hacc_filter_threshold = read_param<unsigned int>(ParamID::ZONE_GNSS_HACCFILT_THR);
			gnss_config.underwater_en = read_param<bool>(ParamID::UNDERWATER_EN);
			gnss_config.acquisition_timeout = read_param<unsigned int>(ParamID::ZONE_GNSS_ACQ_TIMEOUT);
			gnss_config.acquisition_timeout_cold_start = read_param<unsigned int>(ParamID::GNSS_COLD_ACQ_TIMEOUT);
			gnss_config.hdop_filter_threshold = read_param<unsigned int>(ParamID::ZONE_GNSS_HDOPFILT_THR);
			gnss_config.fix_mode = read_param<BaseGNSSFixMode>(ParamID::GNSS_FIX_MODE);
			gnss_config.dyn_model = read_param<BaseGNSSDynModel>(ParamID::GNSS_DYN_MODEL);
			gnss_config.min_num_fixes = read_param<unsigned int>(ParamID::GNSS_MIN_NUM_FIXES);
			gnss_config.cold_start_retry_period = read_param<unsigned int>(ParamID::GNSS_COLD_START_RETRY_PERIOD);
			gnss_config.assistnow_enable = read_param<bool>(ParamID::GNSS_ASSISTNOW_EN);
			gnss_config.trigger_on_surfaced = read_param<bool>(ParamID::GNSS_TRIGGER_ON_SURFACED);
			gnss_config.assistnow_offline_enable = read_param<bool>(ParamID::GNSS_ASSISTNOW_OFFLINE_EN);
			gnss_config.constellation_mask = read_param<unsigned int>(ParamID::GNSS_CONSTELLATION_MASK);
			gnss_config.orbmaxerr = read_param<unsigned int>(ParamID::GNSS_ORBMAXERR);
			gnss_config.min_cno = read_param<unsigned int>(ParamID::GNSS_MIN_CNO);
			gnss_config.min_elev = read_param<unsigned int>(ParamID::GNSS_MIN_ELEV);
			gnss_config.ano_stale_days = read_param<unsigned int>(ParamID::GNSS_ANO_STALE_DAYS);

			if (!hauled_will_override && !moored_will_override) mark_config_mode(ConfigMode::OUT_OF_ZONE);

		} else {
			// Use default params
			gnss_config.enable = read_param<bool>(ParamID::GNSS_EN);
			gnss_config.dloc_arg_nom = read_param<unsigned int>(ParamID::DLOC_ARG_NOM);
			gnss_config.acquisition_timeout = read_param<unsigned int>(ParamID::GNSS_ACQ_TIMEOUT);
			gnss_config.acquisition_timeout_cold_start = read_param<unsigned int>(ParamID::GNSS_COLD_ACQ_TIMEOUT);
			gnss_config.hdop_filter_enable = read_param<bool>(ParamID::GNSS_HDOPFILT_EN);
			gnss_config.hdop_filter_threshold = read_param<unsigned int>(ParamID::GNSS_HDOPFILT_THR);
			gnss_config.hacc_filter_enable = read_param<bool>(ParamID::GNSS_HACCFILT_EN);
			gnss_config.hacc_filter_threshold = read_param<unsigned int>(ParamID::GNSS_HACCFILT_THR);
			gnss_config.underwater_en = read_param<bool>(ParamID::UNDERWATER_EN);
			gnss_config.fix_mode = read_param<BaseGNSSFixMode>(ParamID::GNSS_FIX_MODE);
			gnss_config.dyn_model = read_param<BaseGNSSDynModel>(ParamID::GNSS_DYN_MODEL);
			gnss_config.min_num_fixes = read_param<unsigned int>(ParamID::GNSS_MIN_NUM_FIXES);
			gnss_config.cold_start_retry_period = read_param<unsigned int>(ParamID::GNSS_COLD_START_RETRY_PERIOD);
			gnss_config.assistnow_enable = read_param<bool>(ParamID::GNSS_ASSISTNOW_EN);
			gnss_config.trigger_on_surfaced = read_param<bool>(ParamID::GNSS_TRIGGER_ON_SURFACED);
			gnss_config.assistnow_offline_enable = read_param<bool>(ParamID::GNSS_ASSISTNOW_OFFLINE_EN);
			gnss_config.constellation_mask = read_param<unsigned int>(ParamID::GNSS_CONSTELLATION_MASK);
			gnss_config.orbmaxerr = read_param<unsigned int>(ParamID::GNSS_ORBMAXERR);
			gnss_config.min_cno = read_param<unsigned int>(ParamID::GNSS_MIN_CNO);
			gnss_config.min_elev = read_param<unsigned int>(ParamID::GNSS_MIN_ELEV);
			gnss_config.ano_stale_days = read_param<unsigned int>(ParamID::GNSS_ANO_STALE_DAYS);

			if (!hauled_will_override && !moored_will_override) mark_config_mode(ConfigMode::NORMAL);
		}

		// HAULED override (Plan 1 step 3) — applied after the LB/OoZ/NORMAL
		// cascade has filled the rest of gnss_config. `hauled_will_override`
		// computed at the top of this function tells us whether to engage.
		// Priority cascade: LOW_BATTERY > HAULED > OUT_OF_ZONE > NORMAL.
		// HAULED_GNSS_STRAT=OFF forces GNSS off (Doppler-only); FRESH and
		// REUSE_LAST both leave GNSS on (REUSE_LAST routes through cached fix
		// in the TX path, no GPS power-on).
		if (hauled_will_override) {
			unsigned int strat = read_param<unsigned int>(ParamID::HAULED_GNSS_STRAT);
			if (strat == (unsigned int)BaseGnssStrategy::OFF) {
				gnss_config.enable = false;
			} else {
				gnss_config.enable = read_param<bool>(ParamID::HAULED_GNSS_EN);
			}
			if (m_last_config_mode != ConfigMode::HAULED) {
				DEBUG_INFO("ConfigurationStore: HAULED mode engaged (GNSS)");
#if VALIDATION_LOG_ENABLE
				DEBUG_INFO("[VAL-HAULED] mode_enter t=%u src=GNSS",
				           rtc && rtc->is_set() ? (unsigned int)rtc->gettime() : 0U);
#endif
				m_last_config_mode = ConfigMode::HAULED;
			}
		}

		// MOORED override (2026-08) — same post-cascade shape as HAULED above:
		// rewrites only the two fields that actually differ rather than
		// duplicating the 25-line parameter block a fifth time.
		//
		// `enable` is ANDed, never assigned: MOORED_GNSS_EN can only narrow the
		// GNSS enable, so a deployment running GNSS_EN=0 (or certification mode)
		// can never be switched back on by the moored branch.
		if (moored_will_override) {
			gnss_config.dloc_arg_nom = read_param<unsigned int>(ParamID::MOORED_DLOC);
			gnss_config.enable = gnss_config.enable && read_param<bool>(ParamID::MOORED_GNSS_EN);
			if (m_last_config_mode != ConfigMode::MOORED) {
				DEBUG_INFO("ConfigurationStore: MOORED mode engaged (GNSS) — acquisition period %u s",
				           gnss_config.dloc_arg_nom);
#if VALIDATION_LOG_ENABLE
				DEBUG_INFO("[VAL-MOORED] mode_enter t=%u src=GNSS dloc=%u",
				           rtc && rtc->is_set() ? (unsigned int)rtc->gettime() : 0U, gnss_config.dloc_arg_nom);
#endif
				m_last_config_mode = ConfigMode::MOORED;
			}
		}

		// Disable GNSS if certification TX is enabled
		if (cert_tx_enable) {
			DEBUG_TRACE("ConfigurationStore::get_gnss_configuration: disable GNSS as TX certification mode is set");
			gnss_config.enable = false;
		}
	}

	/// @brief Device-unique seed for the TX-jitter RNGs.
	///
	/// Single source of truth for every scheduler that jitters its TX times
	/// (ArgosTxScheduler, LoRaTxScheduler), so two services on the same unit
	/// stay aligned and two units never share a sequence.
	///
	/// Each build variant is seeded from the identity it actually provisions:
	///  - LoRa: LORA_DEVEUI. The Argos IDs are never written on a LoRa-only
	///    build, so reaching for them first would hand every unit in the fleet
	///    mt19937(0) -- identical jitter, and a +/-5 s anti-collision spread
	///    that degenerates into a fleet-wide constant offset. DevEUI is the
	///    identity the network already knows this device by, and it is
	///    provisioned (or derived from FICR by the driver) on every unit.
	///  - Argos: ARGOS_DECID, then ARGOS_HEXID. SMD provisions DECID (it is the
	///    ID it transmits) and KIM2 writes DECID+HEXID together at init.
	///  - Last resort: PMU::device_identifier() (MCU FICR), unique per chip and
	///    always readable, so the seed is never 0 whatever the provisioning
	///    state.
	///
	/// Kept separate from ArgosConfig::argos_id on purpose: that field is also
	/// logged as the human-facing "Argos ID", and must keep reporting the real
	/// provisioned ID (0 included) rather than a synthesised fallback.
	unsigned int get_tx_jitter_seed() {
#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
		// FNV-1a fold of the 16-hex-char DevEUI. Folding is required (64 bits ->
		// 32) and must keep the LOW-order digits significant: DevEUIs in a batch
		// are typically allocated as a contiguous block sharing a vendor prefix,
		// so a naive "parse the first 8 hex chars" would collapse a whole fleet
		// onto one seed -- the very failure this function exists to prevent.
		const std::string &deveui = read_param<std::string>(ParamID::LORA_DEVEUI);
		if (!deveui.empty()) {
			unsigned int hash = 2166136261U;  // FNV offset basis
			for (char c : deveui) {
				hash ^= (unsigned char)c;
				hash *= 16777619U;  // FNV prime
			}
			if (hash) return hash;
		}
#endif
		unsigned int seed = read_param<unsigned int>(ParamID::ARGOS_DECID);
		if (seed) return seed;
		seed = read_param<unsigned int>(ParamID::ARGOS_HEXID);
		if (seed) return seed;
		return (unsigned int)PMU::device_identifier();
	}

	/// @brief Populate ArgosConfig struct from current params (handles NORMAL/LB/ZONE modes).
	void get_argos_configuration(ArgosConfig &argos_config) {
		auto lb_en = read_param<bool>(ParamID::LB_EN);
		update_battery_level();

		argos_config.is_out_of_zone = is_zone_exclusion();
		argos_config.is_lb = false;
		// Default GNSS strategy = FRESH preserves byte-identical pre-Plan-1
		// behavior for every non-HAULED path. Only the HAULED override branch
		// (below) sets it to REUSE_LAST / OFF when the user requests.
		argos_config.gnss_strategy = BaseGnssStrategy::FRESH;
		// BLIND MAC profile — global params (read once, apply to all regimes).
		argos_config.blind_en = read_param<bool>(ParamID::ARGOS_BLIND_EN);
		argos_config.blind_retx_nb = read_param<unsigned int>(ParamID::ARGOS_BLIND_RETX_NB);
		argos_config.blind_retx_period_s = read_param<unsigned int>(ParamID::ARGOS_BLIND_RETX_PERIOD_S);
		// Predict whether HAULED override will engage. Used to gate
		// "NORMAL/OUT_OF_ZONE mode detected" logs in the cascade so they
		// don't alternate with "HAULED mode engaged" each tick. Single
		// HauledModeService::evaluate() call shared with get_gnss_configuration
		// would be ideal but the two functions are called independently.
		HauledModeService::evaluate();
		bool hauled_will_override = !(lb_en && m_is_battery_level_low) && HauledModeService::is_hauled()
		                            && read_param<bool>(ParamID::HAULED_DETECT_EN);

		// Same prediction for the MOORED override — see get_gnss_configuration().
		MooredModeService::evaluate();
		bool moored_will_override = !(lb_en && m_is_battery_level_low) && !hauled_will_override
		                            && MooredModeService::is_moored() && read_param<bool>(ParamID::MOORED_DETECT_EN);

		// Power and frequency are controlled by RADIOCONF on SMD devices.
		// These fields are kept for legacy (non-SMD) scheduler compatibility.
		argos_config.power = BaseArgosPower::POWER_350_MW;
		argos_config.frequency = 401.65;

		if (lb_en && m_is_battery_level_low) {
			argos_config.is_lb = true;
			argos_config.gnss_en = read_param<bool>(ParamID::LB_GNSS_EN);
			argos_config.last_aop_update = read_param<std::time_t>(ParamID::ARGOS_AOP_DATE);
			argos_config.prepass_en = read_param<bool>(ParamID::SAT_PREPASS_EN);
			argos_config.aop_max_age_days = read_param<unsigned int>(ParamID::SAT_AOP_MAX_AGE_DAYS);
			argos_config.prepass_max_wait_s = read_param<unsigned int>(ParamID::SAT_PREPASS_MAX_WAIT_S);
			argos_config.argos_rx_aop_update_period = read_param<unsigned int>(ParamID::ARGOS_RX_AOP_UPDATE_PERIOD);
			argos_config.argos_rx_max_window = read_param<unsigned int>(ParamID::ARGOS_RX_MAX_WINDOW);
			argos_config.argos_rx_en = read_param<bool>(ParamID::ARGOS_RX_EN);
			argos_config.argos_tx_jitter_en = read_param<bool>(ParamID::ARGOS_TX_JITTER_EN);
			argos_config.time_sync_burst_en = read_param<bool>(ParamID::ARGOS_TIME_SYNC_BURST_EN);
			argos_config.tx_counter = read_param<unsigned int>(ParamID::TX_COUNTER);
			argos_config.mode = read_param<BaseArgosMode>(ParamID::LB_ARGOS_MODE);
			argos_config.depth_pile = read_param<BaseDepthPile>(ParamID::LB_ARGOS_DEPTH_PILE);
			argos_config.duty_cycle = read_param<unsigned int>(ParamID::LB_ARGOS_DUTY_CYCLE);
			argos_config.ntry_per_message = read_param<unsigned int>(ParamID::LB_NTRY_PER_MESSAGE);
			argos_config.tx_interval_s = read_param<unsigned int>(ParamID::TR_LB);
			argos_config.dry_time_before_tx = read_param<unsigned int>(ParamID::DRY_TIME_BEFORE_TX);
			argos_config.underwater_en = read_param<bool>(ParamID::UNDERWATER_EN);
			argos_config.argos_id = read_param<unsigned int>(ParamID::ARGOS_DECID);
			argos_config.prepass_min_elevation = read_param<double>(ParamID::PP_MIN_ELEVATION);
			argos_config.prepass_max_elevation = read_param<double>(ParamID::PP_MAX_ELEVATION);
			argos_config.prepass_min_duration = read_param<unsigned int>(ParamID::PP_MIN_DURATION);
			argos_config.prepass_max_passes = read_param<unsigned int>(ParamID::PP_MAX_PASSES);
			argos_config.prepass_linear_margin = read_param<unsigned int>(ParamID::PP_LINEAR_MARGIN);
			argos_config.prepass_comp_step = read_param<unsigned int>(ParamID::PP_COMP_STEP);
			argos_config.prepass_min_culmination = read_param<unsigned int>(ParamID::PP_MIN_CULMINATION);
			argos_config.prepass_rx_min_culmination = read_param<unsigned int>(ParamID::PP_RX_MIN_CULMINATION);
			argos_config.prepass_position_margin_km = read_param<unsigned int>(ParamID::PP_POSITION_MARGIN_KM);
			unsigned int delta_time_loc = read_param<unsigned int>(ParamID::DLOC_ARG_LB);
			argos_config.delta_time_loc = calc_delta_time_loc(delta_time_loc);
			argos_config.shutdown_ntime_sat = read_param<unsigned int>(ParamID::LB_SHUTDOWN_NTIME_SAT);
			argos_config.surfacing_burst_init_s = read_param<unsigned int>(ParamID::SURFACING_BURST_INIT_S);
			argos_config.surfacing_burst_step_s = read_param<unsigned int>(ParamID::SURFACING_BURST_STEP_S);
			argos_config.surfacing_burst_max_s = read_param<unsigned int>(ParamID::SURFACING_BURST_MAX_S);
			mark_config_mode(ConfigMode::LOW_BATTERY);
		} else if (argos_config.is_out_of_zone) {
			argos_config.gnss_en = read_param<bool>(ParamID::GNSS_EN);
			argos_config.last_aop_update = read_param<std::time_t>(ParamID::ARGOS_AOP_DATE);
			argos_config.prepass_en = read_param<bool>(ParamID::SAT_PREPASS_EN);
			argos_config.aop_max_age_days = read_param<unsigned int>(ParamID::SAT_AOP_MAX_AGE_DAYS);
			argos_config.prepass_max_wait_s = read_param<unsigned int>(ParamID::SAT_PREPASS_MAX_WAIT_S);
			argos_config.argos_rx_aop_update_period = read_param<unsigned int>(ParamID::ARGOS_RX_AOP_UPDATE_PERIOD);
			argos_config.argos_rx_max_window = read_param<unsigned int>(ParamID::ARGOS_RX_MAX_WINDOW);
			argos_config.argos_rx_en = read_param<bool>(ParamID::ARGOS_RX_EN);
			argos_config.argos_tx_jitter_en = read_param<bool>(ParamID::ARGOS_TX_JITTER_EN);
			argos_config.time_sync_burst_en = read_param<bool>(ParamID::ARGOS_TIME_SYNC_BURST_EN);
			argos_config.tx_counter = read_param<unsigned int>(ParamID::TX_COUNTER);
			argos_config.mode = read_param<BaseArgosMode>(ParamID::ZONE_ARGOS_MODE);
			argos_config.depth_pile = read_param<BaseDepthPile>(ParamID::ZONE_ARGOS_DEPTH_PILE);
			argos_config.duty_cycle = read_param<unsigned int>(ParamID::ZONE_ARGOS_DUTY_CYCLE);
			argos_config.ntry_per_message = read_param<unsigned int>(ParamID::ZONE_ARGOS_NTRY_PER_MESSAGE);
			argos_config.tx_interval_s = read_param<unsigned int>(ParamID::ZONE_ARGOS_REPETITION_SECONDS);
			argos_config.dry_time_before_tx = read_param<unsigned int>(ParamID::DRY_TIME_BEFORE_TX);
			argos_config.underwater_en = read_param<bool>(ParamID::UNDERWATER_EN);
			argos_config.argos_id = read_param<unsigned int>(ParamID::ARGOS_DECID);
			argos_config.prepass_min_elevation = read_param<double>(ParamID::PP_MIN_ELEVATION);
			argos_config.prepass_max_elevation = read_param<double>(ParamID::PP_MAX_ELEVATION);
			argos_config.prepass_min_duration = read_param<unsigned int>(ParamID::PP_MIN_DURATION);
			argos_config.prepass_max_passes = read_param<unsigned int>(ParamID::PP_MAX_PASSES);
			argos_config.prepass_linear_margin = read_param<unsigned int>(ParamID::PP_LINEAR_MARGIN);
			argos_config.prepass_comp_step = read_param<unsigned int>(ParamID::PP_COMP_STEP);
			argos_config.prepass_min_culmination = read_param<unsigned int>(ParamID::PP_MIN_CULMINATION);
			argos_config.prepass_rx_min_culmination = read_param<unsigned int>(ParamID::PP_RX_MIN_CULMINATION);
			argos_config.prepass_position_margin_km = read_param<unsigned int>(ParamID::PP_POSITION_MARGIN_KM);
			argos_config.delta_time_loc =
			    calc_delta_time_loc(read_param<unsigned int>(ParamID::ZONE_GNSS_DELTA_ARG_LOC_ARGOS_SECONDS));
			argos_config.shutdown_ntime_sat = read_param<unsigned int>(ParamID::SHUTDOWN_NTIME_SAT);
			argos_config.surfacing_burst_init_s = read_param<unsigned int>(ParamID::SURFACING_BURST_INIT_S);
			argos_config.surfacing_burst_step_s = read_param<unsigned int>(ParamID::SURFACING_BURST_STEP_S);
			argos_config.surfacing_burst_max_s = read_param<unsigned int>(ParamID::SURFACING_BURST_MAX_S);

			if (!hauled_will_override && !moored_will_override) mark_config_mode(ConfigMode::OUT_OF_ZONE);
		} else {
			// Use default params
			argos_config.gnss_en = read_param<bool>(ParamID::GNSS_EN);
			argos_config.last_aop_update = read_param<std::time_t>(ParamID::ARGOS_AOP_DATE);
			argos_config.prepass_en = read_param<bool>(ParamID::SAT_PREPASS_EN);
			argos_config.aop_max_age_days = read_param<unsigned int>(ParamID::SAT_AOP_MAX_AGE_DAYS);
			argos_config.prepass_max_wait_s = read_param<unsigned int>(ParamID::SAT_PREPASS_MAX_WAIT_S);
			argos_config.argos_rx_aop_update_period = read_param<unsigned int>(ParamID::ARGOS_RX_AOP_UPDATE_PERIOD);
			argos_config.argos_rx_max_window = read_param<unsigned int>(ParamID::ARGOS_RX_MAX_WINDOW);
			argos_config.argos_rx_en = read_param<bool>(ParamID::ARGOS_RX_EN);
			argos_config.argos_tx_jitter_en = read_param<bool>(ParamID::ARGOS_TX_JITTER_EN);
			argos_config.time_sync_burst_en = read_param<bool>(ParamID::ARGOS_TIME_SYNC_BURST_EN);
			argos_config.tx_counter = read_param<unsigned int>(ParamID::TX_COUNTER);
			argos_config.mode = read_param<BaseArgosMode>(ParamID::ARGOS_MODE);
			argos_config.depth_pile = read_param<BaseDepthPile>(ParamID::ARGOS_DEPTH_PILE);
			argos_config.duty_cycle = read_param<unsigned int>(ParamID::DUTY_CYCLE);
			argos_config.ntry_per_message = read_param<unsigned int>(ParamID::NTRY_PER_MESSAGE);
			argos_config.tx_interval_s = read_param<unsigned int>(ParamID::TR_NOM);
			argos_config.dry_time_before_tx = read_param<unsigned int>(ParamID::DRY_TIME_BEFORE_TX);
			argos_config.underwater_en = read_param<bool>(ParamID::UNDERWATER_EN);
			argos_config.argos_id = read_param<unsigned int>(ParamID::ARGOS_DECID);
			argos_config.prepass_min_elevation = read_param<double>(ParamID::PP_MIN_ELEVATION);
			argos_config.prepass_max_elevation = read_param<double>(ParamID::PP_MAX_ELEVATION);
			argos_config.prepass_min_duration = read_param<unsigned int>(ParamID::PP_MIN_DURATION);
			argos_config.prepass_max_passes = read_param<unsigned int>(ParamID::PP_MAX_PASSES);
			argos_config.prepass_linear_margin = read_param<unsigned int>(ParamID::PP_LINEAR_MARGIN);
			argos_config.prepass_comp_step = read_param<unsigned int>(ParamID::PP_COMP_STEP);
			argos_config.prepass_min_culmination = read_param<unsigned int>(ParamID::PP_MIN_CULMINATION);
			argos_config.prepass_rx_min_culmination = read_param<unsigned int>(ParamID::PP_RX_MIN_CULMINATION);
			argos_config.prepass_position_margin_km = read_param<unsigned int>(ParamID::PP_POSITION_MARGIN_KM);
			unsigned int delta_time_loc = read_param<unsigned int>(ParamID::DLOC_ARG_NOM);
			argos_config.delta_time_loc = calc_delta_time_loc(delta_time_loc);
			argos_config.shutdown_ntime_sat = read_param<unsigned int>(ParamID::SHUTDOWN_NTIME_SAT);
			argos_config.surfacing_burst_init_s = read_param<unsigned int>(ParamID::SURFACING_BURST_INIT_S);
			argos_config.surfacing_burst_step_s = read_param<unsigned int>(ParamID::SURFACING_BURST_STEP_S);
			argos_config.surfacing_burst_max_s = read_param<unsigned int>(ParamID::SURFACING_BURST_MAX_S);
			if (!hauled_will_override && !moored_will_override) mark_config_mode(ConfigMode::NORMAL);
		}

		// HAULED override (Plan 1) — see matching logic in
		// get_gnss_configuration() above. Overrides mode / TR_NOM / gnss_en
		// only; everything else inherits from the LB/OoZ/NORMAL cascade.
		if (hauled_will_override) {
			argos_config.mode = read_param<BaseArgosMode>(ParamID::HAULED_ARGOS_MODE);
			// SURFACING_BURST requires UW transitions to fire — meaningless in
			// HAULED (the animal isn't diving). DTE write now rejects this
			// value (HMP10 allowed_values restricted to {0,1,2,3,4}); this
			// guard catches legacy configs persisted before the restriction
			// and silently rolls back to LEGACY to avoid a TX-less hauled mode.
			if (argos_config.mode == BaseArgosMode::SURFACING_BURST) {
				DEBUG_WARN("ConfigurationStore: HAULED_ARGOS_MODE=SURFACING_BURST auto-promoted to LEGACY");
				argos_config.mode = BaseArgosMode::LEGACY;
			}
			argos_config.tx_interval_s = read_param<unsigned int>(ParamID::HAULED_TR_NOM);
			// HMP13 = strategy. Translate to (gnss_strategy, gnss_en):
			//   FRESH      → gnss_en = HMP12 (acquire as usual)
			//   REUSE_LAST → gnss_en = false (GPS stays OFF, battery saved);
			//                ArgosTxService dispatch sees strategy and routes
			//                to process_gnss_burst_from_cached() instead of
			//                process_doppler_burst()
			//   OFF        → gnss_en = false; dispatch falls through to
			//                process_doppler_burst() (no cached lookup)
			unsigned int strat = read_param<unsigned int>(ParamID::HAULED_GNSS_STRAT);
			argos_config.gnss_strategy = static_cast<BaseGnssStrategy>(strat);
			if (strat == (unsigned int)BaseGnssStrategy::REUSE_LAST) {
				argos_config.gnss_en = false;  // No acquisition; TX uses cached fix
			} else if (strat == (unsigned int)BaseGnssStrategy::OFF) {
				argos_config.gnss_en = false;
			} else {  // FRESH
				argos_config.gnss_en = read_param<bool>(ParamID::HAULED_GNSS_EN);
			}
			if (m_last_config_mode != ConfigMode::HAULED) {
				DEBUG_INFO("ConfigurationStore: HAULED mode engaged (Argos)");
#if VALIDATION_LOG_ENABLE
				DEBUG_INFO("[VAL-HAULED] mode_enter t=%u src=Argos",
				           rtc && rtc->is_set() ? (unsigned int)rtc->gettime() : 0U);
#endif
				// HM-1 audit fix: warn about ambiguous config combos. HMP12
				// (HAULED_GNSS_EN) is ONLY honored when HMP13 (HAULED_GNSS_STRAT)
				// = FRESH. For REUSE_LAST and OFF the strategy forces gnss_en=false
				// regardless of HMP12. Users frequently misread this as "enable
				// GPS AND use last fix", but the actual behavior is "GPS stays
				// off, TX uses cache only". Fired once on HAULED entry only
				// (gated by the !=HAULED check above) — no log spam.
				if (strat != (unsigned int)BaseGnssStrategy::FRESH && read_param<bool>(ParamID::HAULED_GNSS_EN)) {
					DEBUG_WARN("ConfigurationStore: HMP12=HAULED_GNSS_EN=true but HMP13=%s "
					           "(non-FRESH) ignores it — GPS will stay OFF during HAULED. "
					           "Set HMP13=FRESH if you want HMP12 to enable GPS, or set "
					           "HMP12=false to remove this warning.",
					           strat == (unsigned int)BaseGnssStrategy::REUSE_LAST ? "REUSE_LAST" : "OFF");
				}
				m_last_config_mode = ConfigMode::HAULED;
			}
		}

		// MOORED override (2026-08) — see matching logic in
		// get_gnss_configuration() above. Stretches the TX interval and mirrors
		// the GNSS enable; deliberately does NOT touch `mode`.
		//
		// Not offering a MOORED_ARGOS_MODE is a decision, not an omission: the
		// only thing a stationary vessel needs is a longer interval, and a mode
		// override is a well-known way to silence a tracker by accident (set it
		// to OFF, or to SURFACING_BURST on a hull that never dives, and the
		// boat goes quiet with no error anywhere). The interval alone cannot do
		// that — MRP06's floor is 30 s and the heartbeat always fires.
		if (moored_will_override) {
			argos_config.tx_interval_s = read_param<unsigned int>(ParamID::MOORED_TR_NOM);
			argos_config.gnss_en = argos_config.gnss_en && read_param<bool>(ParamID::MOORED_GNSS_EN);
			if (m_last_config_mode != ConfigMode::MOORED) {
				DEBUG_INFO("ConfigurationStore: MOORED mode engaged (Argos/LoRa) — TX interval %u s",
				           argos_config.tx_interval_s);
#if VALIDATION_LOG_ENABLE
				DEBUG_INFO("[VAL-MOORED] mode_enter t=%u src=Argos tr=%u",
				           rtc && rtc->is_set() ? (unsigned int)rtc->gettime() : 0U, argos_config.tx_interval_s);
#endif
				m_last_config_mode = ConfigMode::MOORED;
			}
		}

		// Extract certification tx params
		argos_config.cert_tx_enable = read_param<bool>(ParamID::CERT_TX_ENABLE);
		argos_config.cert_tx_modulation = read_param<BaseArgosModulation>(ParamID::CERT_TX_MODULATION);
		argos_config.cert_tx_payload = read_param<std::string>(ParamID::CERT_TX_PAYLOAD);
		argos_config.cert_tx_repetition = read_param<unsigned int>(ParamID::CERT_TX_REPETITION);
		argos_config.argos_tcxo_warmup_time = read_param<unsigned int>(ParamID::ARGOS_TCXO_WARMUP_TIME);

		// Mark GNSS disabled if certification is set
		if (argos_config.cert_tx_enable) argos_config.gnss_en = false;

		// BLIND against the modes that pace their own burst. argos_config.blind_en
		// is the mode-aware source of truth for the drivers and the TX service.
		//
		// SURFACING_BURST: never, and not negotiable. It runs a progressive
		// Doppler cascade AND a GNSS phase, both paced by the nRF; a
		// module-owned retx on top would put every message on air twice —
		// double the satellite budget, and a beacon talking over itself.
		//
		// DOPPLER: the operator's choice, under one condition. BLIND hands the
		// repetition to the module, which cannot repeat faster than
		// BLIND_MIN_RETX_PERIOD_S. The Doppler sequence spaces its own messages
		// by surfacing_burst_init_s and grows to surfacing_burst_max_s. If that
		// settled cadence is tighter than the BLIND floor the two cannot be
		// reconciled — the module would still be repeating the previous message
		// when the nRF schedules the next — so BLIND is refused rather than run
		// half-honoured. Otherwise the module's period is ALIGNED on the
		// sequence's own cadence instead of ARP46: that is what makes the two
		// coherent, and it is why ARP46 is not what ends up on air here.
		//
		// ArgosTxService::service_init reports the outcome once, so an operator
		// who asked for BLIND and did not get it reads why.
		if (argos_config.mode == BaseArgosMode::SURFACING_BURST) {
			argos_config.blind_en = false;
		} else if (argos_config.mode == BaseArgosMode::DOPPLER && argos_config.blind_en) {
			if (argos_config.surfacing_burst_max_s < ArgosConfig::BLIND_MIN_RETX_PERIOD_S)
				argos_config.blind_en = false;
			else
				argos_config.blind_retx_period_s = argos_config.surfacing_burst_max_s;
		}

		// Adaptive modulation configuration
		argos_config.adaptive_modulation = read_param<bool>(ParamID::ARGOS_ADAPTIVE_MODULATION);
		argos_config.radioconf_ldk = read_param<std::string>(ParamID::ARGOS_RADIOCONF_LDK);
		argos_config.radioconf_lda2 = read_param<std::string>(ParamID::ARGOS_RADIOCONF_LDA2);
		argos_config.radioconf_vlda4 = read_param<std::string>(ParamID::ARGOS_RADIOCONF_VLDA4);

		// Set sensor TX enable based on configuration
		argos_config.sensor_tx_enable = 0;
		if (argos_config.gnss_en) {
#if ENABLE_ALS_SENSOR
			argos_config.sensor_tx_enable |=
			    (int)(read_param<bool>(ParamID::ALS_SENSOR_ENABLE)
			          && read_param<BaseSensorEnableTxMode>(ParamID::ALS_SENSOR_ENABLE_TX_MODE)
			                 != BaseSensorEnableTxMode::OFF)
			    << (int)ServiceIdentifier::ALS_SENSOR;
#endif
#if ENABLE_PRESSURE_SENSOR
			argos_config.sensor_tx_enable |=
			    (int)(read_param<bool>(ParamID::PRESSURE_SENSOR_ENABLE)
			          && read_param<BaseSensorEnableTxMode>(ParamID::PRESSURE_SENSOR_ENABLE_TX_MODE)
			                 != BaseSensorEnableTxMode::OFF)
			    << (int)ServiceIdentifier::PRESSURE_SENSOR;
#endif
#if ENABLE_SEA_TEMP_SENSOR
			argos_config.sensor_tx_enable |=
			    (int)(read_param<bool>(ParamID::SEA_TEMP_SENSOR_ENABLE)
			          && read_param<BaseSensorEnableTxMode>(ParamID::SEA_TEMP_SENSOR_ENABLE_TX_MODE)
			                 != BaseSensorEnableTxMode::OFF)
			    << (int)ServiceIdentifier::SEA_TEMP_SENSOR;
#endif
#if ENABLE_PH_SENSOR
			argos_config.sensor_tx_enable |=
			    (int)(read_param<bool>(ParamID::PH_SENSOR_ENABLE)
			          && read_param<BaseSensorEnableTxMode>(ParamID::PH_SENSOR_ENABLE_TX_MODE)
			                 != BaseSensorEnableTxMode::OFF)
			    << (int)ServiceIdentifier::PH_SENSOR;
#endif
#if ENABLE_AXL_SENSOR
			// The accelerometer was missing from this mask while every other
			// sensor had a branch. sensor_tx_enable is what selects
			// process_sensor_burst() over process_gnss_burst(), so on a build
			// where AXL is the only compiled sensor the mask was a compile-time
			// 0 and the sensor path was never taken -- AXL_SENSOR_ENABLE_TX_MODE
			// (AXP05) was an operator-visible parameter that could not do
			// anything, even though ArgosPacketBuilder encodes AXL fully
			// (SENSOR_PACKET_MASK_AXL, X/Y/Z, activity, temperature rules).
			// Both AXP01 and AXP05 default to false/OFF, so this changes nothing
			// until an operator asks for it explicitly.
			argos_config.sensor_tx_enable |=
			    (int)(read_param<bool>(ParamID::AXL_SENSOR_ENABLE)
			          && read_param<BaseSensorEnableTxMode>(ParamID::AXL_SENSOR_ENABLE_TX_MODE)
			                 != BaseSensorEnableTxMode::OFF)
			    << (int)ServiceIdentifier::AXL_SENSOR;
#endif
#if ENABLE_THERMISTOR_SENSOR
			argos_config.sensor_tx_enable |=
			    (int)(read_param<bool>(ParamID::THERMISTOR_SENSOR_ENABLE)
			          && read_param<BaseSensorEnableTxMode>(ParamID::THERMISTOR_SENSOR_ENABLE_TX_MODE)
			                 != BaseSensorEnableTxMode::OFF)
			    << (int)ServiceIdentifier::THERMISTOR_SENSOR;
#endif
		}
	}

	void increment_tx_counter() {
		unsigned int tx_counter = read_param<unsigned int>(ParamID::TX_COUNTER) + 1;
		write_param(ParamID::TX_COUNTER, tx_counter);
	}

	void increment_rx_counter() {
		unsigned int rx_counter = read_param<unsigned int>(ParamID::ARGOS_RX_COUNTER) + 1;
		write_param(ParamID::ARGOS_RX_COUNTER, rx_counter);
	}

	void increment_rx_time(unsigned int inc) {
		unsigned int rx_time = read_param<unsigned int>(ParamID::ARGOS_RX_TIME) + inc;
		write_param(ParamID::ARGOS_RX_TIME, rx_time);
	}

#ifdef EXTERNAL_WAKEUP
	// Boot counter management for TPL5111 periodic wakeup
	unsigned int boot_count_increment() {
		unsigned int boot_counter = read_param<unsigned int>(ParamID::BOOT_COUNTER);
		unsigned int boot_counter_modulo = read_param<unsigned int>(ParamID::BOOT_COUNTER_MODULO);
		// Protection against corrupted counter value exceeding modulo bounds
		if (boot_counter > (boot_counter_modulo + 1)) {
			boot_counter = 0;
		} else {
			boot_counter++;
		}
		write_param(ParamID::BOOT_COUNTER, boot_counter);
		save_params();
		return boot_counter;
	}

	unsigned int boot_count_clear() {
		unsigned int boot_counter = 0;
		write_param(ParamID::BOOT_COUNTER, boot_counter);
		save_params();
		return boot_counter;
	}

	unsigned int boot_count_read() { return read_param<unsigned int>(ParamID::BOOT_COUNTER); }

	// Check if this boot is our turn to run based on modulo
	// Returns true if (boot_counter % modulo == 0), meaning it's our turn to run
	bool boot_count_check_modulo(unsigned int boot_counter) {
		unsigned int modulo = read_param<unsigned int>(ParamID::BOOT_COUNTER_MODULO);

		// Protection: modulo must be >= 2 to avoid running every boot (modulo=1)
		// or division by zero (modulo=0). If misconfigured, always allow boot.
		if (modulo < 2) {
			DEBUG_WARN("BOOT_COUNTER_MODULO=%u invalid (must be >=2) | allowing boot", modulo);
			return false;
		}

		if (boot_counter % modulo == 0) {
			boot_count_clear();
			return true;  // It's our turn to run
		}

		return false;  // Not our turn, caller should shutdown
	}
#endif
};
