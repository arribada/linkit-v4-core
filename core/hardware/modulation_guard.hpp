/**
 * @file modulation_guard.hpp
 * @brief Pure helpers that keep the Argos modulation state right after a reset
 *        (adaptive modulation OFF): master-RCONF fingerprints, the trust rule,
 *        the one-attempt-per-master rule, read-back decoding and late framing.
 *
 * Header-only and hardware-free so the host unit tests compile it as-is. The
 * drivers (kim2.cpp, smd_sat*.cpp) are nRF-port-only; every decision they take
 * about "is this a genuine read-back of the master" or "how is this frame
 * padded" lives here so it has host coverage.
 *
 * Codes: ARGOS_CACHED_MODULATION (SMP01) and ARGOS_MASTER_MODULATION (ART13)
 * share the SmdArgosModulation convention, which is NOT KineisModulation's
 * order: 0 = LDA2, 1 = LDK, 2 = VLDA4, and 3 = unknown (ART13 only).
 */

#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "crc32.hpp"
#include "kineis_device.hpp"  // KineisModulation, KineisPacket

namespace ModulationGuard {

/// @name SMP01 / ART13 codes (SmdArgosModulation order)
/// @{
constexpr unsigned int CODE_LDA2 = 0;
constexpr unsigned int CODE_LDK = 1;
constexpr unsigned int CODE_VLDA4 = 2;
constexpr unsigned int CODE_UNKNOWN = 3;  ///< ART13 only: no genuine read-back of the current master
/// @}

/// @brief KineisModulation -> SMP01/ART13 code. Explicit map, never a cast.
inline unsigned int to_code(KineisModulation m) {
	switch (m) {
	case KineisModulation::LDK: return CODE_LDK;
	case KineisModulation::VLDA4: return CODE_VLDA4;
	case KineisModulation::LDA2:
	default: return CODE_LDA2;
	}
}

/// @brief SMP01/ART13 code -> KineisModulation, nullopt for 3 (unknown) or junk.
inline std::optional<KineisModulation> from_code(unsigned int code) {
	switch (code) {
	case CODE_LDA2: return KineisModulation::LDA2;
	case CODE_LDK: return KineisModulation::LDK;
	case CODE_VLDA4: return KineisModulation::VLDA4;
	default: return std::nullopt;
	}
}

/// @brief Fingerprint of a master RCONF (ARGOS_RADIOCONF, IDP14) as stored.
/// CRC32 of the hex text, case-folded so "AB" and "ab" are the same master,
/// forced odd so a real RCONF never fingerprints to 0. Empty -> 0, which is
/// also the factory default of ART14/ART15: an empty master is never trusted
/// and never resynced.
inline uint32_t rconf_fingerprint(const std::string &rconf_hex) {
	if (rconf_hex.empty()) return 0;
	uint32_t crc = 0xFFFFFFFFU;
	for (char c : rconf_hex) {
		uint8_t b = static_cast<uint8_t>((c >= 'A' && c <= 'F') ? (c - 'A' + 'a') : c);
		CRC32::checksum_update(&b, 1, crc);
	}
	CRC32::checksum_finalize(crc);
	return crc | 1U;
}

/// @brief Can the stored master modulation (ART13) be believed for the
/// master RCONF configured NOW? Only when it came from a genuine read-back of
/// exactly this master (ART14 == fp(IDP14)), with adaptive modulation off.
inline bool modulation_cache_trusted(unsigned int art13, uint32_t art14, const std::string &master_rconf,
                                     bool adaptive) {
	if (adaptive || master_rconf.empty() || art13 > CODE_VLDA4) return false;
	return art14 == rconf_fingerprint(master_rconf);
}

/// @brief SMD only: run the init-time resync (power the module, write the
/// credentials, read the master back)? At most once per master fingerprint,
/// whatever its outcome -- ART15 is persisted BEFORE the module is powered, so
/// an attempt that resets the MCU is never repeated -- and never on a low
/// battery.
inline bool should_attempt_init_resync(bool trusted, const std::string &master_rconf, uint32_t art15,
                                       bool battery_low) {
	if (trusted || master_rconf.empty() || battery_low) return false;
	return art15 != rconf_fingerprint(master_rconf);
}

/// @brief SMD SPI READ_RCONF byte 9 (STM32 KNS_tx_mod_t) -> SMP01 code.
/// LDA2=2, LDA2L=3, VLDA4=4, HDA4=5, LDK=6. HDA4 and anything else are NOT
/// recognised: the caller keeps its previous modulation instead of the silent
/// LDA2 the driver used to substitute.
inline std::optional<unsigned int> smd_kns_code_to_code(uint8_t kns) {
	switch (kns) {
	case 2:
	case 3: return CODE_LDA2;
	case 4: return CODE_VLDA4;
	case 6: return CODE_LDK;
	default: return std::nullopt;
	}
}

/// @brief SMD AT "AT+RCONF=?" data -> SMP01 code. The reply is
/// "<min_freq>,<max_freq>,<rf_level>,<modulation>": the modulation is the last
/// field. No comma, an empty field or an unknown name are NOT recognised.
inline std::optional<unsigned int> smd_at_rconf_reply_to_code(const std::string &reply) {
	const size_t last_comma = reply.rfind(',');
	if (last_comma == std::string::npos) return std::nullopt;
	std::string mod = reply.substr(last_comma + 1);
	while (!mod.empty() && (mod.back() == '\r' || mod.back() == '\n' || mod.back() == ' '))
		mod.pop_back();
	if (mod == "LDA2" || mod == "LDA2L") return CODE_LDA2;
	if (mod == "LDK") return CODE_LDK;
	if (mod == "VLDA4") return CODE_VLDA4;
	return std::nullopt;
}

/// @brief Largest payload, in bits, a modulation carries (the one size table
/// shared by the service guards and the drivers' framing).
inline unsigned int max_payload_bits(KineisModulation m) {
	switch (m) {
	case KineisModulation::LDK: return 128;
	case KineisModulation::LDA2: return 192;
	case KineisModulation::VLDA4: return 24;
	default: return 0;
	}
}

/// @brief Frame a RAW payload for the modulation the radio is actually on.
///
/// LDK is fixed at 16 B and VLDA4 at 3 B; LDA2 rounds up to 4 B, minimum 4 B,
/// maximum 24 B. The same rules as the SMD send() padding and the KIM2 stuffing
/// bits, so a frame re-cut here is what send() would have built had it known
/// the modulation. Bits past @p raw_bits are zeroed.
///
/// @return nullopt when the payload does not fit: the caller refuses the frame
///         (no RF). Never truncates.
inline std::optional<KineisPacket> frame_for_modulation(const KineisPacket &raw, unsigned int raw_bits,
                                                        KineisModulation actual) {
	const unsigned int max_bits = max_payload_bits(actual);
	if (max_bits == 0 || raw_bits > max_bits) return std::nullopt;
	unsigned int total_bits = max_bits;
	if (actual == KineisModulation::LDA2) {
		total_bits = ((raw_bits + 31U) / 32U) * 32U;
		if (total_bits < 32U) total_bits = 32U;
	}
	KineisPacket out(total_bits / 8U, '\0');
	const unsigned int full_bytes = raw_bits / 8U;
	for (unsigned int i = 0; i < full_bytes && i < raw.size(); i++)
		out[i] = raw[i];
	const unsigned int rem = raw_bits % 8U;
	if (rem && full_bytes < raw.size()) {
		const uint8_t keep = static_cast<uint8_t>(0xFFU << (8U - rem));
		out[full_bytes] = static_cast<char>(static_cast<uint8_t>(raw[full_bytes]) & keep);
	}
	return out;
}

}  // namespace ModulationGuard
