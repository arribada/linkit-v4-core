/**
 * @file kim2_rx.hpp
 * @brief Pure KIM2 RX-side helpers: firmware capability gate and allcast
 *        payload conversion.
 *
 * Header-only, hardware-free logic so it can be compiled into the host unit
 * tests (the KIM2Device driver in kim2.cpp is nRF-port-only and excluded from
 * the test build), same split as kim2_modulation.hpp.
 *
 * The KIM2 answers AT+FW=? with a free-form build string, e.g.
 *   +FW=77b50fc_0x1ETrx_gui,Dec 05 2025_14:16:19
 * i.e. a git hash, a build tag and a C-style __DATE__ / __TIME__ pair. There is
 * no semantic version field in it, so the RX capability gate keys off the BUILD
 * DATE: downlink reception is only exercised on firmware built from Dec 2025
 * onwards. Should a later firmware expose a real version number, only
 * fw_supports_rx() below has to change.
 */

#pragma once

#include <string>

namespace KIM2 {

/// @brief Build date extracted from an AT+FW=? response.
struct FwBuildDate {
	bool valid = false;  ///< True when a "Mon DD YYYY" pattern was found
	unsigned year = 0;
	unsigned month = 0;  ///< 1..12
	unsigned day = 0;    ///< 1..31
};

/// @name Minimum firmware build date allowing downlink reception
/// @{
static constexpr unsigned KIM2_RX_MIN_FW_YEAR = 2026;
static constexpr unsigned KIM2_RX_MIN_FW_MONTH = 7;
/// @}

/**
 * @brief Extract the C __DATE__ style build date from a +FW= payload.
 *
 * Scans for the first "Mon" abbreviation followed by a 1-2 digit day and a
 * 4-digit year, separated by spaces, underscores or dashes. Case-sensitive on
 * purpose: a lower-case git hash cannot produce a false match.
 *
 * @param fw  Payload of the +FW= response (prefix already stripped).
 * @return Decoded date; @c valid is false when no date pattern was found.
 */
inline FwBuildDate parse_fw_build_date(const std::string &fw) {
	static const char *MONTHS[12] = {
		"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
	};
	FwBuildDate out;

	for (size_t i = 0; i + 3 <= fw.size(); i++) {
		for (unsigned m = 0; m < 12; m++) {
			if (fw.compare(i, 3, MONTHS[m]) != 0) continue;

			size_t p = i + 3;
			unsigned day = 0, year = 0, year_digits = 0;

			while (p < fw.size() && (fw[p] == ' ' || fw[p] == '_' || fw[p] == '-'))
				p++;
			const size_t day_start = p;
			while (p < fw.size() && fw[p] >= '0' && fw[p] <= '9')
				day = day * 10 + static_cast<unsigned>(fw[p++] - '0');
			if (p == day_start || day == 0 || day > 31) continue;

			while (p < fw.size() && (fw[p] == ' ' || fw[p] == '_' || fw[p] == '-'))
				p++;
			while (p < fw.size() && fw[p] >= '0' && fw[p] <= '9') {
				year = year * 10 + static_cast<unsigned>(fw[p++] - '0');
				year_digits++;
			}
			if (year_digits != 4 || year < 2000 || year > 2199) continue;

			out.valid = true;
			out.year = year;
			out.month = m + 1;
			out.day = day;
			return out;
		}
	}
	return out;
}

/**
 * @brief Is this firmware recent enough to run downlink reception?
 *
 * @param fw  Payload of the +FW= response.
 * @return true when the build date is >= Dec 2025. An unparseable string is
 *         rejected: we would rather not drive AT+RX on a firmware we cannot
 *         identify.
 */
inline bool fw_supports_rx(const std::string &fw) {
	const FwBuildDate d = parse_fw_build_date(fw);

	if (!d.valid) return false;
	if (d.year != KIM2_RX_MIN_FW_YEAR) return d.year > KIM2_RX_MIN_FW_YEAR;
	return d.month >= KIM2_RX_MIN_FW_MONTH;
}

/**
 * @brief Convert a +DL_ALLCAST= hex payload into raw bytes.
 *
 * The KIM2 prints ceil(size_bits / 4) nibbles, so the string length is ODD for
 * any message whose useful size is not a multiple of 8 bits — 75 nibbles for
 * the 297-bit AOP MultiSat and Constellation Status messages, 25 for the
 * 99-bit ones. Dropping that last nibble would drop the final bit of the FCS
 * and every such frame would then be rejected by the decoder, so the payload is
 * completed to a whole byte here. The zero padding added is exactly what the
 * protocol specifies after the FCS.
 *
 * @param hex        Payload as printed by the module.
 * @param size_bits  Receives the useful size in bits (4 bits per nibble).
 * @return Raw bytes, or an empty string if a non-hex character is found.
 */
inline std::string allcast_to_bytes(const std::string &hex, unsigned int &size_bits) {
	std::string out;
	unsigned int nibbles = 0;
	bool have_high = false;
	unsigned char high = 0;

	size_bits = 0;
	out.reserve((hex.size() + 1) / 2);
	for (size_t i = 0; i < hex.size(); i++) {
		const char c = hex[i];
		unsigned char v;

		if (c >= '0' && c <= '9')
			v = static_cast<unsigned char>(c - '0');
		else if (c >= 'a' && c <= 'f')
			v = static_cast<unsigned char>(c - 'a' + 10);
		else if (c >= 'A' && c <= 'F')
			v = static_cast<unsigned char>(c - 'A' + 10);
		else
			return std::string();

		nibbles++;
		if (!have_high) {
			high = v;
			have_high = true;
		} else {
			out.push_back(static_cast<char>((high << 4) | v));
			have_high = false;
		}
	}
	/* Odd nibble count: complete the byte with the protocol's zero padding. */
	if (have_high) out.push_back(static_cast<char>(high << 4));

	size_bits = nibbles * 4;
	return out;
}

}  // namespace KIM2
