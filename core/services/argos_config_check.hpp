/**
 * @file argos_config_check.hpp
 * @brief Configuration-time Argos warnings (ARGOS_CFG_WARNINGS, ART12).
 *
 * The firmware NEVER refuses a configuration; it reports, once, what will not
 * work. The operator's GUI enforces the rules (an LDA2 master for CloudLocate
 * MEAS20); this is the safety net for every other path into a tag:
 *
 *   PARMW -> SATVF (force=1 on KIM2) -> STATR ART12,ART13 -> warn if ART12 != 0
 *
 * The evaluator is pure and compiled on EVERY build, LoRa included (the host
 * unit tests are LoRa builds). Only its call sites are gated: the end of PARMW,
 * the end of SATVF and STATR on Argos builds, and ArgosTxService::service_init.
 * Never from a driver read-back point: those sit on the surfacing path, and a
 * DEBUG_WARN also costs a ~170 ms flash-log commit.
 */

#pragma once

#include <cstdint>

#include "argos_packet_builder.hpp"  // CloudLocate frame sizes
#include "config_store.hpp"
#include "debug.hpp"
#include "messages.hpp"  // MAX_LOG_PAYLOAD
#include "modulation_guard.hpp"

namespace ArgosConfigCheck {

/// @name ARGOS_CFG_WARNINGS bits
/// @{
constexpr uint32_t CL_EXCEEDS_MASTER = 1U << 0;     ///< CloudLocate frame does not fit the verified master
constexpr uint32_t CL_MASTER_UNVERIFIED = 1U << 1;  ///< CloudLocate, adaptive off, master modulation not verified
constexpr uint32_t MASTER_RCONF_EMPTY = 1U << 2;    ///< IDP14 empty (adaptive off, or any SMD build)
constexpr uint32_t CL_FORMAT_NOT_ARGOS = 1U << 3;   ///< MEAS50 on an Argos build (exclusive of bit0/bit1)
/// @}

struct Inputs {
	bool gnss_en;               ///< GNSS_EN (GNP01)
	unsigned int fastloc_mode;  ///< GNSS_FASTLOC_MODE (GNP45)
	unsigned int cl_format;     ///< GNSS_CLOUDLOCATE_FORMAT (GNP46)
	bool adaptive;              ///< ARGOS_ADAPTIVE_MODULATION (ARP54)
	bool master_empty;          ///< ARGOS_RADIOCONF (IDP14) empty
	unsigned int master_mod;    ///< ART13 if trusted, else ModulationGuard::CODE_UNKNOWN
	bool is_smd;                ///< SMD backend (SPI or UART), else KIM2
};

/// @brief Payload bits of the configured CloudLocate format on Argos.
constexpr unsigned int cloudlocate_bits(unsigned int cl_format) {
	return cl_format == static_cast<unsigned int>(BaseCloudLocateFormat::MEASC12)
	           ? ArgosPacketBuilder::CLOUDLOCATE_MEASC12_BITS
			   : ArgosPacketBuilder::CLOUDLOCATE_MEAS20_BITS;
}

/// @brief The warning mask. Pure.
inline uint32_t evaluate(const Inputs &in) {
	uint32_t mask = 0;
	const bool cloudlocate = in.gnss_en && in.fastloc_mode == static_cast<unsigned int>(BaseFastlocMode::CLOUDLOCATE);
	if (cloudlocate) {
		if (in.cl_format == static_cast<unsigned int>(BaseCloudLocateFormat::MEAS50)) {
			// Handled first and alone: MEAS50 is never sent over Argos, so whether it
			// would "fit" the master is meaningless (its size would read as MEAS20's).
			mask |= CL_FORMAT_NOT_ARGOS;
		} else if (!in.adaptive) {
			// Adaptive ON switches to LDA2 for MEAS20 by itself: nothing to report.
			const std::optional<KineisModulation> master = ModulationGuard::from_code(in.master_mod);
			if (!master.has_value())
				mask |= CL_MASTER_UNVERIFIED;
			else if (cloudlocate_bits(in.cl_format) > ModulationGuard::max_payload_bits(master.value()))
				mask |= CL_EXCEEDS_MASTER;
		}
	}
	// KIM2 with adaptive ON uses the per-modulation RCONFs and is not silenced by
	// an empty master; SMD refuses to transmit without one whatever ARP54 says.
	if (in.master_empty && (!in.adaptive || in.is_smd)) mask |= MASTER_RCONF_EMPTY;
	return mask;
}

/// @name Log lines (English, one per evaluation, at most MAX_LOG_PAYLOAD - 1)
/// @{
constexpr char MSG_EMPTY_KIM2[] =
    "ArgosConfigCheck: IDP14 empty - KIM2 will transmit on its built-in LDK RCONF; MEAS20 cannot be sent";
constexpr char MSG_EMPTY_SMD[] =
    "ArgosConfigCheck: IDP14 empty - SMD will not transmit until a master RCONF is provisioned";
constexpr char MSG_MEAS20_ON_LDK[] = "ArgosConfigCheck: CloudLocate MEAS20 needs an LDA2 master RCONF, master reads "
                                     "back LDK (adaptive off) - never sent";
constexpr char MSG_MEAS20_ON_VLDA4[] = "ArgosConfigCheck: CloudLocate MEAS20 needs an LDA2 master RCONF, master reads "
                                       "back VLDA4 (adaptive off) - never sent";
constexpr char MSG_MEASC12_ON_VLDA4[] =
    "ArgosConfigCheck: CloudLocate MEASC12 does not fit master RCONF VLDA4 (adaptive off) - never sent";
constexpr char MSG_MEAS50[] =
    "ArgosConfigCheck: CloudLocate MEAS50 is LoRa only - no CloudLocate will be sent over Argos";
constexpr char MSG_UNVERIFIED_MEAS20[] =
    "ArgosConfigCheck: CloudLocate MEAS20 needs an LDA2 master RCONF; master modulation not verified yet - run SATVF";
constexpr char MSG_UNVERIFIED_MEASC12[] =
    "ArgosConfigCheck: CloudLocate MEASC12 needs an LDK/LDA2 master RCONF; master not verified yet - run SATVF";
/// @}

static_assert(sizeof(MSG_EMPTY_KIM2) - 1 <= MAX_LOG_PAYLOAD - 1, "log line truncated");
static_assert(sizeof(MSG_EMPTY_SMD) - 1 <= MAX_LOG_PAYLOAD - 1, "log line truncated");
static_assert(sizeof(MSG_MEAS20_ON_LDK) - 1 <= MAX_LOG_PAYLOAD - 1, "log line truncated");
static_assert(sizeof(MSG_MEAS20_ON_VLDA4) - 1 <= MAX_LOG_PAYLOAD - 1, "log line truncated");
static_assert(sizeof(MSG_MEASC12_ON_VLDA4) - 1 <= MAX_LOG_PAYLOAD - 1, "log line truncated");
static_assert(sizeof(MSG_MEAS50) - 1 <= MAX_LOG_PAYLOAD - 1, "log line truncated");
static_assert(sizeof(MSG_UNVERIFIED_MEAS20) - 1 <= MAX_LOG_PAYLOAD - 1, "log line truncated");
static_assert(sizeof(MSG_UNVERIFIED_MEASC12) - 1 <= MAX_LOG_PAYLOAD - 1, "log line truncated");

/// @brief The line for the MOST severe bit of @p mask, nullptr for 0.
/// Order: empty master (the tag may not transmit at all), then a CloudLocate
/// that can never be sent, then MEAS50, then "not verified yet".
inline const char *message_for(uint32_t mask, const Inputs &in) {
	if (mask & MASTER_RCONF_EMPTY) return in.is_smd ? MSG_EMPTY_SMD : MSG_EMPTY_KIM2;
	if (mask & CL_EXCEEDS_MASTER) {
		if (in.cl_format == static_cast<unsigned int>(BaseCloudLocateFormat::MEASC12)) return MSG_MEASC12_ON_VLDA4;
		return in.master_mod == ModulationGuard::CODE_VLDA4 ? MSG_MEAS20_ON_VLDA4 : MSG_MEAS20_ON_LDK;
	}
	if (mask & CL_FORMAT_NOT_ARGOS) return MSG_MEAS50;
	if (mask & CL_MASTER_UNVERIFIED) {
		return in.cl_format == static_cast<unsigned int>(BaseCloudLocateFormat::MEASC12) ? MSG_UNVERIFIED_MEASC12
		                                                                                 : MSG_UNVERIFIED_MEAS20;
	}
	return nullptr;
}

/// @brief Gather the inputs from the store. The master modulation is ART13
/// only when it is trusted for the CURRENT master -- never SMP01 nor the live
/// driver modulation, which an escalation or an unforced KIM2 SATVF can bend.
inline Inputs inputs_from(ConfigurationStore &store, bool is_smd) {
	Inputs in{};
	in.gnss_en = store.read_param<bool>(ParamID::GNSS_EN);
	in.fastloc_mode = store.read_param<unsigned int>(ParamID::GNSS_FASTLOC_MODE);
	in.cl_format = store.read_param<unsigned int>(ParamID::GNSS_CLOUDLOCATE_FORMAT);
	in.adaptive = store.read_param<bool>(ParamID::ARGOS_ADAPTIVE_MODULATION);
	in.master_empty = store.read_param<std::string>(ParamID::ARGOS_RADIOCONF).empty();
	in.master_mod = store.trusted_master_modulation();
	in.is_smd = is_smd;
	return in;
}

struct Refresh {
	uint32_t mask = 0;     ///< mask now stored in ART12
	bool changed = false;  ///< differs from the ART12 stored before (the caller decides on save_params)
	bool logged = false;   ///< one DEBUG_WARN was emitted (changed AND non-zero)
};

/// @brief Evaluate, store ART12 (RAM) and log ONE line only when the mask
/// differs from the stored one -- nothing when it returns to 0. Never throws.
inline Refresh refresh(ConfigurationStore &store, bool is_smd) {
	Refresh r;
	try {
		const Inputs in = inputs_from(store, is_smd);
		r.mask = evaluate(in);
		const unsigned int prev = store.read_param<unsigned int>(ParamID::ARGOS_CFG_WARNINGS);
		if (r.mask != prev) {
			r.changed = true;
			store.write_param(ParamID::ARGOS_CFG_WARNINGS, static_cast<unsigned int>(r.mask));
			const char *msg = message_for(r.mask, in);
			if (msg) {
				DEBUG_WARN("%s", msg);
				r.logged = true;
			}
		}
	} catch (...) {
		// Store unreadable: report nothing rather than throw into a DTE reply or
		// a service start.
	}
	return r;
}

}  // namespace ArgosConfigCheck
