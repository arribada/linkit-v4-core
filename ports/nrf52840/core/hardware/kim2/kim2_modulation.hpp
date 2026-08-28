/**
 * @file kim2_modulation.hpp
 * @brief Pure KIM2 modulation-name decoding + read-back validation.
 *
 * Header-only, hardware-free logic so it can be compiled into the host unit
 * tests (the KIM2Device driver in kim2.cpp is nRF-port-only and excluded from
 * the test build). These functions back the read-back validation that prevents
 * the +ERROR=5 "12-byte LDA2 frame on an LDK radio" latch: KIM2 caches the
 * modulation the module ACTUALLY reports (AT+RCONF=?), never the requested tag.
 */

#pragma once

#include "kineis_device.hpp"  // KineisModulation
#include <string>
#include <optional>

namespace KIM2 {

/// @brief Map a modulation name string ("LDK"/"LDA2"/"VLDA4") returned by
///        AT+RCONF=? to the KineisModulation enum.
/// @return The enum value, or std::nullopt for names not in our enum
///         (LDA2L / HDA4 / UNKNOWN) — the caller then keeps its previous value
///         rather than guessing.
inline std::optional<KineisModulation> mod_from_name(const std::string &name) {
	if (name == "LDK") return KineisModulation::LDK;
	if (name == "LDA2") return KineisModulation::LDA2;
	if (name == "VLDA4") return KineisModulation::VLDA4;
	return std::nullopt;
}

/// @brief Verdict of comparing a requested modulation against the modulation
///        the module actually reports back.
struct ModulationVerdict {
	bool recognized;          ///< Read-back name maps to a known modulation
	KineisModulation actual;  ///< Decoded read-back modulation (valid iff recognized)
	bool mismatch;            ///< recognized AND actual != requested
};

/// @brief Decide whether the module's read-back modulation matches what we
///        asked for. When @c mismatch is true the RCONF for @p requested
///        physically encodes a DIFFERENT modulation (mislabeled per-mod RCONF,
///        or a non-adaptive master that decodes to another mod) — the caller
///        must cache @c actual (the truth) and refuse/abort rather than frame
///        the payload for @p requested on a radio that is on @c actual.
/// @note  An unrecognized read-back (recognized=false) is treated as "no
///        decision" (mismatch=false): we cannot prove a conflict, so the caller
///        keeps its current modulation — same conservatism as @c mod_from_name.
inline ModulationVerdict verify_modulation(KineisModulation requested, const std::string &readback_name) {
	ModulationVerdict v{ false, requested, false };
	std::optional<KineisModulation> actual = mod_from_name(readback_name);
	if (actual.has_value()) {
		v.recognized = true;
		v.actual = actual.value();
		v.mismatch = (actual.value() != requested);
	}
	return v;
}

}  // namespace KIM2
