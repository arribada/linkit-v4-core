#pragma once

/**
 * @file packet_field_encoding.hpp
 * @brief Field encodings shared by the Argos and LoRa packet builders.
 *
 * These six conversions and the scale factors behind them are a CONTRACT WITH A
 * DECODER THAT LIVES OUTSIDE THIS REPOSITORY. Both builders must produce exactly
 * the same bits for the same input, which is why they used to be duplicated
 * byte-for-byte in argos_packet_builder.cpp and lora_packet_builder.cpp, each
 * carrying a comment saying "kept identical to the other one".
 *
 * Two identical copies stay identical only until someone edits one. They now
 * live here once, and each builder keeps its own one-line `convert_*` wrapper so
 * that a deliberate divergence would have to be written explicitly rather than
 * happening by omission.
 *
 * DO NOT "fix" anything in here. Not the rounding, not the `- 0.00005` on the
 * negative branch, not the asymmetry between latitude and longitude sign bits.
 * Every one of them is what the ground segment expects.
 */

#include <algorithm>

namespace PacketField {

// === Scale factors ==========================================================
static constexpr unsigned int MM_PER_METER = 1000;
static constexpr unsigned int MM_PER_KM = 1000000;
static constexpr unsigned int MV_PER_UNIT = 20;
static constexpr unsigned int METRES_PER_UNIT = 40;
static constexpr double DEGREES_PER_UNIT = 1.0 / 1.42;  ///< ~0.704 deg/unit
static constexpr unsigned int SECONDS_PER_HOUR = 3600;
static constexpr unsigned int MIN_ALTITUDE = 0;
static constexpr unsigned int MAX_ALTITUDE = 254;
static constexpr unsigned int INVALID_ALTITUDE = 255;
static constexpr unsigned int REF_BATT_MV = 2700;
static constexpr unsigned int LON_LAT_RESOLUTION = 10000;
static constexpr int NEG_LON_LAT_RESOLUTION = -10000;
static constexpr unsigned int BITS_PER_BYTE = 8;

// === Conversions ============================================================

/// @brief Ground speed (mm/s) to 7-bit encoding, 2 km/h per unit.
inline unsigned int speed(double x) {
	if (x < 0) return 0;
	return std::min(127u, static_cast<unsigned int>((SECONDS_PER_HOUR * x) / (2 * MM_PER_KM)));
}

/// @brief Battery voltage (mV) to 7-bit encoding, 20 mV/unit, offset 2700 mV.
///
/// DEAD ZONE BELOW 2700 mV: every voltage <= 2700 mV encodes to 0, so a dying
/// 2.0 V cell is indistinguishable from a healthy-ish 2.7 V one in this field
/// alone. That is deliberate -- the reference matches the Argos builder for
/// cross-platform decoder compatibility, and the `is_low_battery` flag in the
/// packet header carries the critical-state indication.
///
/// DECODER GUIDANCE:
///   - decoded mV = 2700 + encoded * 20
///   - encoded == 0 && is_low_battery == 1  -> "<= 2.7 V (CRITICAL)"
///   - encoded == 127                       -> ">= 5.24 V"
inline unsigned int battery_voltage(unsigned int mv) {
	return std::min(127u, static_cast<unsigned int>(std::max(static_cast<int>(mv) - static_cast<int>(REF_BATT_MV), 0))
	                          / MV_PER_UNIT);
}

/// @brief Latitude (degrees) to 21-bit unsigned; bit 20 is the sign.
inline unsigned int latitude(double x) {
	if (x >= 0)
		return static_cast<unsigned int>(x * LON_LAT_RESOLUTION);
	else
		return static_cast<unsigned int>((x - 0.00005) * NEG_LON_LAT_RESOLUTION) | (1u << 20);
}

/// @brief Longitude (degrees) to 22-bit unsigned; bit 21 is the sign.
inline unsigned int longitude(double x) {
	if (x >= 0)
		return static_cast<unsigned int>(x * LON_LAT_RESOLUTION);
	else
		return static_cast<unsigned int>((x - 0.00005) * NEG_LON_LAT_RESOLUTION) | (1u << 21);
}

/// @brief Heading (degrees) to 8-bit encoding, ~0.704 deg/unit.
/// Clamped to 254: 255 is the invalid-fix sentinel and must stay unambiguous.
inline unsigned int heading(double x) {
	if (x < 0) return 0;
	return std::min(254u, static_cast<unsigned int>(x * DEGREES_PER_UNIT));
}

/// @brief Altitude (mm MSL) to 8-bit encoding, 40 m/unit, clamped 0..254.
inline unsigned int altitude(double x) {
	return static_cast<unsigned int>(
	    std::min(static_cast<double>(MAX_ALTITUDE),
		         std::max(static_cast<double>(MIN_ALTITUDE), x / (MM_PER_METER * METRES_PER_UNIT))));
}

}  // namespace PacketField
