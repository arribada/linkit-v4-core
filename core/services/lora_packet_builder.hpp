/**
 * @file lora_packet_builder.hpp
 * @brief LoRa packet builder — GPS, sensor, status, CloudLocate packets for LoRaWAN TX.
 *
 * All methods are static — no state, no dependencies on Service framework.
 * Conversion functions share the same formulas as ArgosPacketBuilder.
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "messages.hpp"
#include "kineis_device.hpp"
#include "config_store.hpp"
#include "service.hpp"

/// @brief LoRa payload limits by data rate (EU868).
struct LoRaPayloadLimits {
	static constexpr unsigned int DR0_MAX_BYTES = 51;   ///< SF12/125kHz
	static constexpr unsigned int DR1_MAX_BYTES = 51;   ///< SF11/125kHz
	static constexpr unsigned int DR2_MAX_BYTES = 51;   ///< SF10/125kHz
	static constexpr unsigned int DR3_MAX_BYTES = 115;  ///< SF9/125kHz
	static constexpr unsigned int DR4_MAX_BYTES = 222;  ///< SF8/125kHz
	static constexpr unsigned int DR5_MAX_BYTES = 222;  ///< SF7/125kHz

	/// @brief Get max payload size for a given data rate.
	/// @param dr  Data rate (0-5).
	/// @return Max payload bytes, or DR0 as safe fallback.
	static unsigned int max_payload_for_dr(uint8_t dr) {
		constexpr unsigned int table[] = { DR0_MAX_BYTES, DR1_MAX_BYTES, DR2_MAX_BYTES,
		                                   DR3_MAX_BYTES, DR4_MAX_BYTES, DR5_MAX_BYTES };
		return (dr < 6) ? table[dr] : DR0_MAX_BYTES;
	}
};

/// @brief LoRa packet builder — constructs GPS, sensor, status, CloudLocate packets.
class LoRaPacketBuilder {
public:
	// === Packet type identifiers (3 bits) ===
	static constexpr uint8_t PKT_TYPE_GPS_SINGLE = 0b000;
	static constexpr uint8_t PKT_TYPE_GPS_MULTI = 0b001;
	static constexpr uint8_t PKT_TYPE_SENSOR = 0b010;
	static constexpr uint8_t PKT_TYPE_STATUS = 0b011;
	static constexpr uint8_t PKT_TYPE_CLOUDLOCATE = 0b100;

	// === Field bit widths ===
	static constexpr unsigned int BITS_PKT_TYPE = 3;
	static constexpr unsigned int BITS_FLAGS = 4;
	static constexpr unsigned int BITS_VOLTAGE = 7;
	static constexpr unsigned int BITS_GPS_COUNT = 4;
	static constexpr unsigned int BITS_DELTA_T_MIN = 16;  ///< Per-entry minutes-back, GPS_MULTI v2
	static constexpr unsigned int BITS_DAY = 5;
	static constexpr unsigned int BITS_HOUR = 5;
	static constexpr unsigned int BITS_MIN = 6;
	static constexpr unsigned int BITS_LATITUDE = 21;
	static constexpr unsigned int BITS_LONGITUDE = 22;
	static constexpr unsigned int BITS_SPEED = 7;
	static constexpr unsigned int BITS_HEADING = 8;
	static constexpr unsigned int BITS_ALTITUDE = 8;
	static constexpr unsigned int BITS_NUMSV = 4;
	static constexpr unsigned int BITS_SENSOR_MASK = 6;
	static constexpr unsigned int BITS_CL_FORMAT = 2;

	// Sensor field bit widths
	static constexpr unsigned int BITS_ALS = 17;
	static constexpr unsigned int BITS_PH = 14;
	static constexpr unsigned int BITS_PRESSURE = 15;
	static constexpr unsigned int BITS_PRESS_TEMP = 14;
	static constexpr unsigned int BITS_SEA_TEMP = 14;
	static constexpr unsigned int BITS_AXL_TEMP = 14;
	static constexpr unsigned int BITS_AXL_AXIS = 15;
	static constexpr unsigned int BITS_AXL_ACT = 8;

	// Fastloc quality field bit widths
	static constexpr unsigned int BITS_FIXTYPE = 2;
	static constexpr unsigned int BITS_HACC = 16;
	static constexpr unsigned int BITS_VACC = 16;
	static constexpr unsigned int BITS_PDOP = 8;
	static constexpr unsigned int BITS_HDOP = 8;
	static constexpr unsigned int BITS_ONTIME = 10;

	// === Composite sizes ===
	static constexpr unsigned int BITS_HEADER = BITS_PKT_TYPE + BITS_FLAGS + BITS_VOLTAGE;
	static constexpr unsigned int BITS_FASTLOC_QUALITY =
	    BITS_FIXTYPE + BITS_HACC + BITS_VACC + BITS_PDOP + BITS_HDOP + BITS_ONTIME;
	static constexpr unsigned int BITS_GPS_FULL = BITS_DAY + BITS_HOUR + BITS_MIN + BITS_LATITUDE + BITS_LONGITUDE
	                                              + BITS_SPEED + BITS_HEADING + BITS_ALTITUDE + BITS_NUMSV;
	/// @brief GPS_MULTI v2 delta entry: lat + lon + speed + per-entry delta_t (minutes back).
	static constexpr unsigned int BITS_GPS_DELTA = BITS_LATITUDE + BITS_LONGITUDE + BITS_SPEED + BITS_DELTA_T_MIN;
	static constexpr unsigned int DELTA_T_MIN_MAX = 0xFFFFu;  ///< Sentinel for "≥ ~45 days"
	static constexpr unsigned int BITS_AXL_FULL = BITS_AXL_TEMP + (3 * BITS_AXL_AXIS) + BITS_AXL_ACT;
	static constexpr unsigned int BITS_PRESSURE_FULL = BITS_PRESSURE + BITS_PRESS_TEMP;
	static constexpr unsigned int BITS_PER_BYTE = 8;

	// === Encoding constants ===
	static constexpr unsigned int FIXTYPE_3D = 3;
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

	// === Conversion helpers ===

	/// @brief Convert altitude (mm MSL) to 8-bit encoding (40m/unit).
	static unsigned int convert_altitude(double x);
	/// @brief Convert heading (degrees) to 8-bit encoding (~0.704 deg/unit).
	static unsigned int convert_heading(double x);
	/// @brief Convert ground speed (mm/s) to 7-bit encoding.
	static unsigned int convert_speed(double x);
	/// @brief Encode latitude as 21-bit unsigned (bit 20 = sign).
	static unsigned int convert_latitude(double x);
	/// @brief Encode longitude as 22-bit unsigned (bit 21 = sign).
	static unsigned int convert_longitude(double x);
	/// @brief Encode battery voltage as 7-bit (20mV/unit, offset 2700mV).
	static unsigned int convert_battery_voltage(unsigned int battery_voltage);

	/// @brief Compute max GPS entries for a given payload size.
	/// @param max_payload_bytes  Max payload in bytes (from DR limit).
	/// @return Number of GPS entries that fit.
	static unsigned int max_gps_entries(unsigned int max_payload_bytes);

	// === Packet builders ===

	/// @brief Build GPS packet (single or multi-fix, GPS_MULTI v2 layout).
	///
	/// Multi-fix layout: entry[0] is the MOST RECENT fix (full timestamp).
	/// Subsequent entries carry their position plus a 16-bit `DELTA_T_MIN`
	/// field giving how many minutes earlier they occurred relative to the
	/// previous entry. The input vector `v` is expected in oldest-first order
	/// (as produced by `DepthPile::retrieve()`); the builder reverses it
	/// internally to emit newest-first on the wire.
	///
	/// @warning **Mutates `v`** — the first `min(v.size(), max_entries, 15)`
	///          elements are reversed in place. Caller must not rely on the
	///          original element order after the call. Safe to discard the
	///          vector immediately after invocation, as `LoRaTxService` does.
	static KineisPacket build_gps_packet(std::vector<GPSLogEntry *> &v, bool is_out_of_zone, bool is_low_battery,
	                                     unsigned int max_payload_bytes, unsigned int &size_bits);

	/// @brief Build sensor packet (GPS + optional ALS/PH/pressure/sea_temp/AXL).
	static KineisPacket build_sensor_packet(GPSLogEntry *gps, ServiceSensorData *als, ServiceSensorData *ph,
	                                        ServiceSensorData *pressure, ServiceSensorData *sea_temp,
	                                        ServiceSensorData *axl, bool is_out_of_zone, bool is_low_battery,
	                                        unsigned int &size_bits);

	/// @brief Build status packet (battery + flags only, minimal size).
	static KineisPacket build_status_packet(unsigned int battery_voltage, bool is_low_battery, unsigned int &size_bits);

	/// @brief Build CloudLocate raw GNSS measurement packet.
	/// @param capture_rtc  RTC epoch (s) at snapshot capture. 0 = unknown → no time
	///                     field (backward compatible — shorter legacy packet).
	static KineisPacket build_cloudlocate_packet(const uint8_t *blob, unsigned int blob_size, uint8_t format_id,
	                                             bool is_low_battery, unsigned int battery_voltage,
	                                             unsigned int &size_bits, uint32_t capture_rtc = 0);
};
