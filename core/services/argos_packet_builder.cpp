/**
 * @file argos_packet_builder.cpp
 * @brief Argos packet builder — encoding helpers and all build_* methods.
 */

#include "argos_packet_builder.hpp"
#include "bitpack.hpp"
#include "crc8.hpp"
#include "binascii.hpp"
#include "timeutils.hpp"
#include "debug.hpp"
#include "battery.hpp"
#include <algorithm>

extern BatteryMonitor *battery_monitor;

static constexpr unsigned int MS_PER_SEC = 1000;

// Embed CRC8 at byte 23 of an LDA2 frame, computed over the first 184 bits (bytes 0-22).
// LDA2 frames are 24 bytes total; LDK and VLDA4 do not need this — the SMD/KIM2 module
// adds CRC8 on those modulations but leaves LDA2 user payload untouched on air.
static void apply_lda2_crc8(KineisPacket& packet) {
	packet.resize(ArgosPacketBuilder::LDA2_FRAME_BYTES, 0);
	unsigned char crc = CRC8::checksum(packet, ArgosPacketBuilder::LDA2_DATA_BITS);
	packet[ArgosPacketBuilder::LDA2_FRAME_BYTES - 1] = static_cast<char>(crc);
}

/// @brief Convert ground speed (mm/s) to 7-bit Argos encoding.
/// @param x  Ground speed in mm/s (expected non-negative, GPS contract).
/// @return Encoded speed (0-127). 127 = max representable (~254 km/h) AND is
///         also the invalid-fix sentinel — decoder must use the `valid` flag
///         from the packet header to disambiguate. Kept identical to
///         LoRaPacketBuilder for cross-platform decoder compatibility.
unsigned int ArgosPacketBuilder::convert_speed(double x) {
	if (x < 0) return 0;
	return std::min(127u, static_cast<unsigned int>((SECONDS_PER_HOUR * x) / (2 * MM_PER_KM)));
}

/// @brief Convert battery voltage (mV) to 7-bit encoding (20mV/unit, offset 2700mV).
/// @param battery_voltage  Voltage in mV.
/// @return Encoded battery (0-127).
unsigned int ArgosPacketBuilder::convert_battery_voltage(unsigned int battery_voltage) {
	return std::min(127u, static_cast<unsigned int>(std::max(static_cast<int>(battery_voltage) - static_cast<int>(REF_BATT_MV), 0)) / MV_PER_UNIT);
}

/// @brief Encode latitude as 21-bit unsigned (bit 20 = sign for negative).
/// @param x  Latitude in degrees.
/// @return 21-bit encoded latitude.
unsigned int ArgosPacketBuilder::convert_latitude(double x) {
	if (x >= 0)
		return static_cast<unsigned int>(x * LON_LAT_RESOLUTION);
	else
		return static_cast<unsigned int>((x - 0.00005) * NEG_LON_LAT_RESOLUTION) | (1u << 20);
}

/// @brief Encode longitude as 22-bit unsigned (bit 21 = sign for negative).
/// @param x  Longitude in degrees.
/// @return 22-bit encoded longitude.
unsigned int ArgosPacketBuilder::convert_longitude(double x) {
	if (x >= 0)
		return static_cast<unsigned int>(x * LON_LAT_RESOLUTION);
	else
		return static_cast<unsigned int>((x - 0.00005) * NEG_LON_LAT_RESOLUTION) | (1u << 21);
}

/// @brief Convert heading (degrees) to 8-bit Argos encoding (~0.704 deg/unit).
/// @param x  Heading in degrees (expected [0, 360], GPS contract).
/// @return Encoded heading (0-254). 255 is reserved for the invalid-fix
///         sentinel — valid headings are clamped to 254 to keep the encoding
///         unambiguous against the sentinel. Kept identical to
///         LoRaPacketBuilder for cross-platform decoder compatibility.
unsigned int ArgosPacketBuilder::convert_heading(double x) {
	if (x < 0) return 0;
	return std::min(254u, static_cast<unsigned int>(x * DEGREES_PER_UNIT));
}

/// @brief Convert altitude (mm MSL) to 8-bit encoding (40m/unit, clamped 0-254).
/// @param x  Altitude in mm above MSL.
/// @return Encoded altitude (0-254, 255=invalid).
unsigned int ArgosPacketBuilder::convert_altitude(double x) {
	return static_cast<unsigned int>(std::min(static_cast<double>(MAX_ALTITUDE), std::max(static_cast<double>(MIN_ALTITUDE), x / (MM_PER_METER * METRES_PER_UNIT))));
}

KineisPacket ArgosPacketBuilder::build_short_packet(GPSLogEntry* gps_entry,
		bool is_out_of_zone,
		bool is_low_battery
		) {

	DEBUG_TRACE("ArgosPacketBuilder::build_short_packet");
	unsigned int base_pos = 0;
	KineisPacket packet;

	// Reserve required number of bytes
	packet.assign(SHORT_PACKET_BYTES, 0);

	// Payload bytes
	PACK_BITS(SHORT_PACKET_HEADER, packet, base_pos, 3);

	// Use scheduled GPS time as day/hour/min
	uint16_t year;
	uint8_t month, day, hour, min, sec;
	convert_datetime_to_epoch(gps_entry->info.schedTime, year, month, day, hour, min, sec);
	PACK_BITS(day, packet, base_pos, 5);

	DEBUG_TRACE("ArgosPacketBuilder::build_short_packet: day=%u", (unsigned int)day);
	PACK_BITS(hour, packet, base_pos, 5);
	DEBUG_TRACE("ArgosPacketBuilder::build_short_packet: hour=%u", (unsigned int)hour);
	PACK_BITS(min, packet, base_pos, 6);
	DEBUG_TRACE("ArgosPacketBuilder::build_short_packet: min=%u", (unsigned int)min);

	if (gps_entry->info.valid) {
		unsigned int lat = convert_latitude(gps_entry->info.lat);
		PACK_BITS(lat, packet, base_pos, 21);
		DEBUG_TRACE("ArgosPacketBuilder::build_short_packet: lat=%u (%lf)", lat, gps_entry->info.lat);
		unsigned int lon = convert_longitude(gps_entry->info.lon);
		PACK_BITS(lon, packet, base_pos, 22);
		DEBUG_TRACE("ArgosPacketBuilder::build_short_packet: lon=%u (%lf)", lon, gps_entry->info.lon);
		unsigned int gspeed = convert_speed((double)gps_entry->info.gSpeed);
		PACK_BITS((unsigned int)gspeed, packet, base_pos, 7);
		DEBUG_TRACE("ArgosPacketBuilder::build_short_packet: speed=%u (%lf)", (unsigned int)gspeed, (double)gps_entry->info.gSpeed);

		// OUTOFZONE_FLAG
		PACK_BITS(is_out_of_zone, packet, base_pos, 1);
		DEBUG_TRACE("ArgosPacketBuilder::build_short_packet: is_out_of_zone=%u", is_out_of_zone);

		unsigned int heading = convert_heading(gps_entry->info.headMot);
		PACK_BITS(heading, packet, base_pos, 8);
		DEBUG_TRACE("ArgosPacketBuilder::build_short_packet: heading=%u", heading);
		if (gps_entry->info.fixType == FIXTYPE_3D) {
			unsigned int altitude = convert_altitude((double)gps_entry->info.hMSL);
			DEBUG_TRACE("ArgosPacketBuilder::build_short_packet: altitude=%d (x 40m)", altitude);
			PACK_BITS(altitude, packet, base_pos, 8);
		} else {
			DEBUG_WARN("ArgosPacketBuilder::build_short_packet: altitude not available without 3D fix");
			PACK_BITS(INVALID_ALTITUDE, packet, base_pos, 8);
		}
	} else {
		DEBUG_TRACE("ArgosPacketBuilder::build_short_packet: lat/lon no fix");
		PACK_BITS(0xFFFFFFFF, packet, base_pos, 21);
		PACK_BITS(0xFFFFFFFF, packet, base_pos, 22);
		PACK_BITS(0xFF, packet, base_pos, 7);
		PACK_BITS(is_out_of_zone, packet, base_pos, 1);
		DEBUG_TRACE("ArgosPacketBuilder::build_short_packet: is_out_of_zone=%u", (unsigned int)is_out_of_zone);
		PACK_BITS(0xFF, packet, base_pos, 8);
		PACK_BITS(0xFF, packet, base_pos, 8);
	}

	unsigned int batt = convert_battery_voltage((unsigned int)gps_entry->info.batt_voltage);
	PACK_BITS(batt, packet, base_pos, 7);
	DEBUG_TRACE("ArgosPacketBuilder::build_short_packet: voltage=%u (%u)", (unsigned int)batt, (unsigned int)gps_entry->info.batt_voltage);

	// LOWBATERY_FLAG
	PACK_BITS(is_low_battery, packet, base_pos, 1);
	DEBUG_TRACE("ArgosPacketBuilder::build_short_packet: is_lb=%u", is_low_battery);

	return packet;
}

KineisPacket ArgosPacketBuilder::build_fastloc_packet(GPSLogEntry* gps_entry,
		bool is_low_battery) {

	DEBUG_TRACE("ArgosPacketBuilder::build_fastloc_packet");
	unsigned int base_pos = 0;
	KineisPacket packet;

	packet.assign(FASTLOC_PACKET_BYTES, 0);

	// Header (3 bits) — type 010 = fastloc
	PACK_BITS(FASTLOC_PACKET_HEADER, packet, base_pos, 3);

	// Timestamp (16 bits)
	uint16_t year;
	uint8_t month, day, hour, min, sec;
	convert_datetime_to_epoch(gps_entry->info.schedTime, year, month, day, hour, min, sec);
	PACK_BITS(day, packet, base_pos, 5);
	PACK_BITS(hour, packet, base_pos, 5);
	PACK_BITS(min, packet, base_pos, 6);

	// Position (43 bits)
	unsigned int lat = convert_latitude(gps_entry->info.lat);
	PACK_BITS(lat, packet, base_pos, 21);
	DEBUG_TRACE("ArgosPacketBuilder::build_fastloc_packet: lat=%u (%lf)", lat, gps_entry->info.lat);
	unsigned int lon = convert_longitude(gps_entry->info.lon);
	PACK_BITS(lon, packet, base_pos, 22);
	DEBUG_TRACE("ArgosPacketBuilder::build_fastloc_packet: lon=%u (%lf)", lon, gps_entry->info.lon);

	// Speed + heading (15 bits)
	unsigned int gspeed = convert_speed((double)gps_entry->info.gSpeed);
	PACK_BITS(gspeed, packet, base_pos, 7);
	unsigned int heading = convert_heading(gps_entry->info.headMot);
	PACK_BITS(heading, packet, base_pos, 8);

	// Altitude (8 bits)
	if (gps_entry->info.fixType == FIXTYPE_3D) {
		PACK_BITS(convert_altitude((double)gps_entry->info.hMSL), packet, base_pos, 8);
	} else {
		PACK_BITS(INVALID_ALTITUDE, packet, base_pos, 8);
	}

	// Battery (8 bits)
	unsigned int batt = convert_battery_voltage((unsigned int)gps_entry->info.batt_voltage);
	PACK_BITS(batt, packet, base_pos, 7);
	PACK_BITS(is_low_battery, packet, base_pos, 1);

	// Quality metadata (64 bits)
	unsigned int fixType = std::min((unsigned int)gps_entry->info.fixType, 3U);
	PACK_BITS(fixType, packet, base_pos, 2);

	unsigned int numSV = std::min((unsigned int)gps_entry->info.numSV, 15U);
	PACK_BITS(numSV, packet, base_pos, 4);

	// hAcc in meters (16 bits, 0-65535m)
	unsigned int hAcc_m = std::min((unsigned int)(gps_entry->info.hAcc / MM_PER_METER), 65535U);
	PACK_BITS(hAcc_m, packet, base_pos, 16);
	DEBUG_TRACE("ArgosPacketBuilder::build_fastloc_packet: hAcc=%um", hAcc_m);

	// vAcc in meters (16 bits, 0-65535m)
	unsigned int vAcc_m = std::min((unsigned int)(gps_entry->info.vAcc / MM_PER_METER), 65535U);
	PACK_BITS(vAcc_m, packet, base_pos, 16);

	// pDOP × 10 (8 bits, 0-25.5)
	unsigned int pdop = std::min((unsigned int)(gps_entry->info.pDOP * 10.0f), 255U);
	PACK_BITS(pdop, packet, base_pos, 8);

	// hDOP × 10 (8 bits, 0-25.5)
	unsigned int hdop = std::min((unsigned int)(gps_entry->info.hDOP * 10.0f), 255U);
	PACK_BITS(hdop, packet, base_pos, 8);

	// GPS on time in seconds (10 bits, 0-1023)
	unsigned int ontime_s = std::min((unsigned int)(gps_entry->info.onTime / MS_PER_SEC), 1023U);
	PACK_BITS(ontime_s, packet, base_pos, 10);

	// Reserved (27 bits) — zero-filled for future use
	// Total: 3+16+43+15+8+8+2+4+16+16+8+8+10+27 = 184 data bits + 8-bit CRC = 192 bits

	// LDA2 firmware-embedded CRC8 at byte 23 (modem does not add CRC for LDA2).
	apply_lda2_crc8(packet);

	DEBUG_INFO("ArgosPacketBuilder::build_fastloc_packet: fixType=%u numSV=%u hAcc=%um pDOP=%.1f batt=%u",
	           fixType, numSV, hAcc_m, (double)gps_entry->info.pDOP, (unsigned int)gps_entry->info.batt_voltage);

	return packet;
}

unsigned int ArgosPacketBuilder::cloudlocate_packet_bits(uint8_t format_id) {
	if (format_id == (uint8_t)BaseCloudLocateFormat::MEASC12)
		return CLOUDLOCATE_MEASC12_BITS;
	return CLOUDLOCATE_MEAS20_BITS;
}

KineisPacket ArgosPacketBuilder::build_cloudlocate_packet(const uint8_t* blob, unsigned int blob_size,
		uint8_t format_id, unsigned int battery_voltage, bool is_low_battery,
		uint32_t capture_rtc, uint32_t now_rtc) {

	DEBUG_TRACE("ArgosPacketBuilder::build_cloudlocate_packet: format=%u blob_size=%u", format_id, blob_size);
	unsigned int total_bits = cloudlocate_packet_bits(format_id);
	unsigned int total_bytes = (total_bits + 7) / 8;

	KineisPacket packet;
	packet.assign(total_bytes, 0);
	unsigned int base_pos = 0;

	// Header (3 bits) — type 111 = CloudLocate
	PACK_BITS(CLOUDLOCATE_PACKET_HEADER, packet, base_pos, 3);

	// Format (2 bits): 00=MEASC12, 01=MEAS20
	PACK_BITS((unsigned int)format_id, packet, base_pos, 2);

	// Raw GNSS measurement blob
	for (unsigned int i = 0; i < blob_size; i++) {
		PACK_BITS((unsigned int)blob[i], packet, base_pos, 8);
	}

	// Battery voltage (7 bits) + low battery (1 bit)
	unsigned int batt = convert_battery_voltage(battery_voltage);
	PACK_BITS(batt, packet, base_pos, 7);
	PACK_BITS(is_low_battery ? 1U : 0U, packet, base_pos, 1);

	// Optional capture-time field (2026-06). BACKWARD COMPATIBLE: only packed when
	// capture_rtc != 0; legacy frames leave this region zero so the 1-bit "time
	// present" flag reads 0 and old/new decoders treat them as time-less.
	//   MEASC12/LDK : flag(1) + seconds-of-day(17)  — full HH:MM:SS, fits the 19 free bits.
	//   MEAS20/LDA2 : flag(1) + age(10, capture→TX s) — fits the 11 bits before CRC8.
	// The cloud reconstructs the absolute instant: date comes from the Argos
	// Doppler pass; seconds-of-day (MEASC12) or (reception_time − age) (MEAS20)
	// gives the precise time-of-measurement, removing the cache/TX-delay error.
	if (capture_rtc != 0) {
		PACK_BITS(1U, packet, base_pos, CLOUDLOCATE_TIME_FLAG_BITS);  // time present
		if (format_id == (uint8_t)BaseCloudLocateFormat::MEAS20) {
			unsigned int age = (now_rtc > capture_rtc) ? (now_rtc - capture_rtc) : 0U;
			if (age > ((1U << CLOUDLOCATE_AGE_BITS) - 1U)) age = (1U << CLOUDLOCATE_AGE_BITS) - 1U;
			PACK_BITS(age, packet, base_pos, CLOUDLOCATE_AGE_BITS);
		} else {
			unsigned int sod = (unsigned int)(capture_rtc % 86400U);
			PACK_BITS(sod, packet, base_pos, CLOUDLOCATE_SOD_BITS);
		}
	}

	// Remaining bits are zero-padded (already zeroed by assign).
	// LDA2 (MEAS20) requires firmware-embedded CRC8 at byte 23; LDK (MEASC12) does not.
	if (format_id == (uint8_t)BaseCloudLocateFormat::MEAS20) {
		apply_lda2_crc8(packet);
	}

	DEBUG_INFO("CL_PKT: fmt=%u sz=%u batt=%u t_present=%u data=%s",
	           format_id, blob_size, battery_voltage, (unsigned)(capture_rtc != 0), Binascii::hexlify(packet).c_str());

	return packet;
}

KineisPacket ArgosPacketBuilder::build_long_packet(std::vector<GPSLogEntry*> &gps_entries,
		bool is_out_of_zone,
		bool is_low_battery,
		BaseDeltaTimeLoc delta_time_loc) {
	unsigned int base_pos = 0;
	KineisPacket packet;

	DEBUG_TRACE("ArgosPacketBuilder::build_long_packet: gps_entries: %u", gps_entries.size());

	// Reserve full LDA2 frame (24 bytes); CRC8 lands at byte 23 at the end.
	packet.assign(LONG_PACKET_BYTES, 0);

	// 3-bit type header — value 000 is shared with Short Packet but disambiguated
	// by the LDA2 24-byte frame size on the receiver side.
	PACK_BITS(LONG_PACKET_HEADER, packet, base_pos, 3);

	// This will set the log time for the GPS entry based on when it was scheduled
	uint16_t year;
	uint8_t month, day, hour, min, sec;
	convert_datetime_to_epoch(gps_entries[0]->info.schedTime, year, month, day, hour, min, sec);

	PACK_BITS(day, packet, base_pos, 5);
	DEBUG_TRACE("ArgosPacketBuilder::build_long_packet: day=%u", (unsigned int)day);
	PACK_BITS(hour, packet, base_pos, 5);
	DEBUG_TRACE("ArgosPacketBuilder::build_long_packet: hour=%u", (unsigned int)hour);
	PACK_BITS(min, packet, base_pos, 6);
	DEBUG_TRACE("ArgosPacketBuilder::build_long_packet: min=%u", (unsigned int)min);

	// First GPS entry
	if (gps_entries[0]->info.valid) {
		PACK_BITS(convert_latitude(gps_entries[0]->info.lat), packet, base_pos, 21);
		DEBUG_TRACE("ArgosPacketBuilder::build_long_packet: lat=%u (%lf)", convert_latitude(gps_entries[0]->info.lat), gps_entries[0]->info.lat);
		PACK_BITS(convert_longitude(gps_entries[0]->info.lon), packet, base_pos, 22);
		DEBUG_TRACE("ArgosPacketBuilder::build_long_packet: lon=%u (%lf)", convert_longitude(gps_entries[0]->info.lon), gps_entries[0]->info.lon);
		unsigned int gspeed = convert_speed(gps_entries[0]->info.gSpeed);
		PACK_BITS((unsigned int)gspeed, packet, base_pos, 7);
		DEBUG_TRACE("ArgosPacketBuilder::build_long_packet: speed=%u", (unsigned int)gspeed);
	} else {
		DEBUG_TRACE("ArgosPacketBuilder::build_long_packet: lat/lon[0] no fix");
		PACK_BITS(0xFFFFFFFF, packet, base_pos, 21);
		PACK_BITS(0xFFFFFFFF, packet, base_pos, 22);
		PACK_BITS(0xFF, packet, base_pos, 7);
	}

	// OUTOFZONE_FLAG
	PACK_BITS(is_out_of_zone, packet, base_pos, 1);
	DEBUG_TRACE("ArgosPacketBuilder::build_long_packet: is_out_of_zone=%u", is_out_of_zone);

	unsigned int batt = convert_battery_voltage(gps_entries[0]->info.batt_voltage);
	PACK_BITS(batt, packet, base_pos, 7);
	DEBUG_TRACE("ArgosPacketBuilder::build_long_packet: voltage=%u (%u)", (unsigned int)batt, (unsigned int)gps_entries[0]->info.batt_voltage);

	// LOWBATERY_FLAG
	PACK_BITS(is_low_battery, packet, base_pos, 1);
	DEBUG_TRACE("ArgosPacketBuilder::build_long_packet: is_lb=%u", (unsigned int)is_low_battery);

	// Delta time loc
	PACK_BITS((unsigned int)delta_time_loc, packet, base_pos, 4);
	DEBUG_TRACE("ArgosPacketBuilder::build_long_packet: delta_time_loc=%u", (unsigned int)delta_time_loc);

	// Subsequent GPS entries
	for (unsigned int i = 1; i < MAX_GPS_ENTRIES_IN_PACKET; i++) {
		if (gps_entries.size() <= i) {
			DEBUG_TRACE("ArgosPacketBuilder::build_long_packet: lat/lon[%u] not present", i);
			PACK_BITS(0xFFFFFFFF, packet, base_pos, 21);
			PACK_BITS(0xFFFFFFFF, packet, base_pos, 22);
		} else if (0 == gps_entries[i]->info.valid) {
			DEBUG_TRACE("ArgosPacketBuilder::build_long_packet: lat/lon[%u] no fix", i);
			PACK_BITS(0xFFFFFFFF, packet, base_pos, 21);
			PACK_BITS(0xFFFFFFFF, packet, base_pos, 22);
		} else {
			PACK_BITS(convert_latitude(gps_entries[i]->info.lat), packet, base_pos, 21);
			DEBUG_TRACE("ArgosPacketBuilder::build_long_packet: lat[%u]=%u (%lf)", i, convert_latitude(gps_entries[i]->info.lat), gps_entries[i]->info.lat);
			PACK_BITS(convert_longitude(gps_entries[i]->info.lon), packet, base_pos, 22);
			DEBUG_TRACE("ArgosPacketBuilder::build_long_packet: lon[%u]=%u (%lf)", i, convert_longitude(gps_entries[i]->info.lon), gps_entries[i]->info.lon);
		}
	}

	// Legacy LONG format: decoders date subsequent positions as
	// date(GPS[i]) = base - i*delta_time_loc (uniform spacing assumed).
	// Bits 168+ stay 0. (The 2026-06 "v2 skip field" extension was removed
	// 2026-07 — decoders expect the plain legacy frame.)

	// LDA2 firmware-embedded CRC8 at byte 23 (modem does not add CRC for LDA2).
	apply_lda2_crc8(packet);

	return packet;
}

KineisPacket ArgosPacketBuilder::build_gnss_packet(std::vector<GPSLogEntry*> &v,
		bool is_out_of_zone,
		bool is_low_battery,
		BaseDeltaTimeLoc delta_time_loc,
		unsigned int &size_bits) {
	if (v.empty()) {
		DEBUG_ERROR("ArgosPacketBuilder::build_gnss_packet: empty vector");
		size_bits = 0;
		return {};
	} else if (v.size() > 1) {
		std::reverse(v.begin(), v.end()); // Reverse to most-recent-first: gps_entries[0]=newest (date header anchor)
		size_bits = LONG_PACKET_BITS;
		DEBUG_INFO("ArgosPacketBuilder::build_gnss_packet: LONG packet, %u position(s), %u bits",
		           (unsigned int)v.size(), size_bits);
		return build_long_packet(v, is_out_of_zone, is_low_battery, delta_time_loc);
	} else {
		size_bits = SHORT_PACKET_BITS;
		DEBUG_INFO("ArgosPacketBuilder::build_gnss_packet: SHORT packet, 1 position, %u bits", size_bits);
		return build_short_packet(v[0], is_out_of_zone, is_low_battery);
	}
}

KineisPacket ArgosPacketBuilder::build_certification_packet(std::string cert_tx_payload, unsigned int &size_bits) {

	// Convert from ASCII hex to a real binary buffer
	KineisPacket packet = Binascii::unhexlify(cert_tx_payload);

	DEBUG_TRACE("ArgosPacketBuilder::build_certification_packet: TX payload size %u bytes", packet.size());

	// Check the size to determine the packet #bits to send in payload
	if (packet.size() > SHORT_PACKET_BYTES) {
		DEBUG_TRACE("ArgosPacketBuilder::build_certification_packet: using long packet");
		size_bits = LONG_PACKET_BITS;
		packet.resize(LONG_PACKET_BYTES);
	} else {
		DEBUG_TRACE("ArgosPacketBuilder::build_certification_packet: using short packet");
		size_bits = SHORT_PACKET_BITS;
		packet.resize(SHORT_PACKET_BYTES);
	}

	return packet;
}

KineisPacket ArgosPacketBuilder::build_doppler_packet(unsigned int batt_voltage, bool is_low_battery, unsigned int &size_bits) {
	DEBUG_TRACE("ArgosPacketBuilder::build_doppler_packet");
	unsigned int base_pos = 0;
	KineisPacket packet;

	// Reserve required number of bytes
	packet.assign(DOPPLER_PACKET_BYTES, 0);

	// Payload bytes
	// PACK_BITS(0, packet, base_pos, 8);  // Zero CRC field (computed later)

	unsigned int last_known_pos = 0;
	PACK_BITS(last_known_pos, packet, base_pos, 8);
	DEBUG_TRACE("ArgosPacketBuilder::build_doppler_packet: last_known_pos=%u", (unsigned int)last_known_pos);

	unsigned int batt = convert_battery_voltage(batt_voltage);
	PACK_BITS(batt, packet, base_pos, 7);
	DEBUG_TRACE("ArgosPacketBuilder::build_doppler_packet: voltage=%u (%u)", (unsigned int)batt, (unsigned int)batt_voltage);

	// LOWBATERY_FLAG
	PACK_BITS(is_low_battery, packet, base_pos, 1);
	DEBUG_TRACE("ArgosPacketBuilder::build_doppler_packet: is_lb=%u", (unsigned int)is_low_battery);

	// CRC8 is handled by the satellite module (SMD/KIM2)

	size_bits = DOPPLER_PACKET_BITS;

	return packet;
}

KineisPacket ArgosPacketBuilder::build_rspb_doppler_packet(
		unsigned int battery_soc,
		unsigned int activity,
		unsigned int mortality_confidence,
		unsigned int &size_bits) {
	DEBUG_TRACE("ArgosPacketBuilder::build_rspb_doppler_packet");
	unsigned int base_pos = 0;
	KineisPacket packet;

	packet.assign(RSPB_DOPPLER_PACKET_BYTES, 0);

	// Header (3 bits) — Type 6 = RSPB Doppler
	PACK_BITS(RSPB_DOPPLER_HEADER, packet, base_pos, 3);

	// Battery SOC (7 bits, 0-100%)
	unsigned int soc = (battery_soc > 100) ? 100 : battery_soc;
	PACK_BITS(soc, packet, base_pos, 7);
	DEBUG_TRACE("ArgosPacketBuilder::build_rspb_doppler_packet: soc=%u%%", soc);

	// Activity (7 bits, 0-127 — original 0-255 divided by 2)
	unsigned int act = (activity > 255) ? 127 : (activity / 2);
	PACK_BITS(act, packet, base_pos, 7);
	DEBUG_TRACE("ArgosPacketBuilder::build_rspb_doppler_packet: activity=%u (raw=%u)", act, activity);

	// Mortality confidence (7 bits, 0-100%)
	unsigned int mort = (mortality_confidence > 100) ? 100 : mortality_confidence;
	PACK_BITS(mort, packet, base_pos, 7);
	DEBUG_TRACE("ArgosPacketBuilder::build_rspb_doppler_packet: mortality=%u%%", mort);

	size_bits = RSPB_DOPPLER_PACKET_BITS;

	return packet;
}

KineisPacket ArgosPacketBuilder::build_sensor_packet(GPSLogEntry* gps_entry,
		ServiceSensorData *als_sensor,
		ServiceSensorData *ph_sensor,
		ServiceSensorData *pressure_sensor,
		ServiceSensorData *sea_temp_sensor,
		ServiceSensorData *axl_sensor,
		bool is_out_of_zone, bool is_low_battery,
		unsigned int& size_bits) {

	DEBUG_TRACE("ArgosPacketBuilder::build_sensor_packet");
	unsigned int base_pos = 0;
	KineisPacket packet;

	// Reserve required number of bytes
	packet.assign(SENSOR_PACKET_BYTES, 0);

	// 3-bit type header — Type 1 (001) discriminates sensor packets from long packets
	// (both are 24-byte LDA2 frames).
	PACK_BITS(SENSOR_PACKET_HEADER, packet, base_pos, 3);

	// Use scheduled GPS time as day/hour/min
	uint16_t year;
	uint8_t month, day, hour, min, sec;
	convert_datetime_to_epoch(gps_entry->info.schedTime, year, month, day, hour, min, sec);
	PACK_BITS(day, packet, base_pos, 5);

	DEBUG_TRACE("ArgosPacketBuilder::build_sensor_packet: day=%u", (unsigned int)day);
	PACK_BITS(hour, packet, base_pos, 5);
	DEBUG_TRACE("ArgosPacketBuilder::build_sensor_packet: hour=%u", (unsigned int)hour);
	PACK_BITS(min, packet, base_pos, 6);
	DEBUG_TRACE("ArgosPacketBuilder::build_sensor_packet: min=%u", (unsigned int)min);

	if (gps_entry->info.valid) {
		unsigned int lat = convert_latitude(gps_entry->info.lat);
		PACK_BITS(lat, packet, base_pos, 21);
		DEBUG_TRACE("ArgosPacketBuilder::build_sensor_packet: lat=%u (%lf)", lat, gps_entry->info.lat);
		unsigned int lon = convert_longitude(gps_entry->info.lon);
		PACK_BITS(lon, packet, base_pos, 22);
		DEBUG_TRACE("ArgosPacketBuilder::build_sensor_packet: lon=%u (%lf)", lon, gps_entry->info.lon);
		unsigned int gspeed = convert_speed((double)gps_entry->info.gSpeed);
		PACK_BITS((unsigned int)gspeed, packet, base_pos, 7);
		DEBUG_TRACE("ArgosPacketBuilder::build_sensor_packet: speed=%u (%lf)", (unsigned int)gspeed, (double)gps_entry->info.gSpeed);

		// OUTOFZONE_FLAG
		PACK_BITS(is_out_of_zone, packet, base_pos, 1);
		DEBUG_TRACE("ArgosPacketBuilder::build_sensor_packet: is_out_of_zone=%u", is_out_of_zone);
	} else {
		DEBUG_TRACE("ArgosPacketBuilder::build_sensor_packet: lat/lon no fix");
		PACK_BITS(0xFFFFFFFF, packet, base_pos, 21);
		PACK_BITS(0xFFFFFFFF, packet, base_pos, 22);
		PACK_BITS(0xFF, packet, base_pos, 7);
		PACK_BITS(is_out_of_zone, packet, base_pos, 1);
		DEBUG_TRACE("ArgosPacketBuilder::build_sensor_packet: is_out_of_zone=%u", (unsigned int)is_out_of_zone);
	}

	// VOLTAGE
	unsigned int batt = convert_battery_voltage((unsigned int)gps_entry->info.batt_voltage);
	PACK_BITS(batt, packet, base_pos, 7);
	DEBUG_TRACE("ArgosPacketBuilder::build_sensor_packet: voltage=%u (%u)", (unsigned int)batt, (unsigned int)gps_entry->info.batt_voltage);

	// LOWBATERY_FLAG
	PACK_BITS(is_low_battery, packet, base_pos, 1);
	DEBUG_TRACE("ArgosPacketBuilder::build_sensor_packet: is_lb=%u", is_low_battery);

	// 5-bit sensor mask describing which sensors are present (decoder reads this to know
	// which fields follow). MSB-first: ALS, PH, Pressure, SeaTemp, AXL.
	unsigned int sensor_mask = 0;
	if (als_sensor != nullptr)        sensor_mask |= SENSOR_PACKET_MASK_ALS;
	if (ph_sensor != nullptr)         sensor_mask |= SENSOR_PACKET_MASK_PH;
	if (pressure_sensor != nullptr)   sensor_mask |= SENSOR_PACKET_MASK_PRESSURE;
	if (sea_temp_sensor != nullptr)   sensor_mask |= SENSOR_PACKET_MASK_SEATEMP;
	if (axl_sensor != nullptr)        sensor_mask |= SENSOR_PACKET_MASK_AXL;
	PACK_BITS(sensor_mask, packet, base_pos, SENSOR_PACKET_MASK_BITS);
	DEBUG_TRACE("ArgosPacketBuilder::build_sensor_packet: sensor_mask=0x%02X", sensor_mask);

	// Add ALS sensor data
	if (als_sensor != nullptr) {
		DEBUG_TRACE("ArgosPacketBuilder::build_sensor_packet: als=%05X", (unsigned int)als_sensor->port[0]);
		PACK_BITS((unsigned int)als_sensor->port[0], packet, base_pos, 17);
	}
	if (ph_sensor != nullptr) {
		DEBUG_TRACE("ArgosPacketBuilder::build_sensor_packet: ph=%04X", (unsigned int)ph_sensor->port[0]);
		PACK_BITS((unsigned int)ph_sensor->port[0], packet, base_pos, 14);
	}
	if (pressure_sensor != nullptr) {
		DEBUG_TRACE("ArgosPacketBuilder::build_sensor_packet: pbar=%04X ptemp=%04X",
				(unsigned int)pressure_sensor->port[0],
				(unsigned int)pressure_sensor->port[1]);
		PACK_BITS((unsigned int)pressure_sensor->port[0], packet, base_pos, 15);
		PACK_BITS((unsigned int)pressure_sensor->port[1], packet, base_pos, 14);
	}
	if (sea_temp_sensor != nullptr) {
		DEBUG_TRACE("ArgosPacketBuilder::build_sensor_packet: sea_temp=%06X", (unsigned int)sea_temp_sensor->port[0]);
		PACK_BITS((unsigned int)sea_temp_sensor->port[0], packet, base_pos, 21);
	}

	// Add AXL (accelerometer) sensor data.
	// port[0] = temperature (14 bits), port[1-3] = X/Y/Z (15 bits each), port[4] = activity (8 bits).
	//
	// AXL temperature inclusion rule (deterministic from sensor mask — decoder uses same rule):
	//   - If another temperature source is present in the packet (Pressure has its own temp,
	//     or SeaTemp/Thermistor sensor is set), AXL temperature is dropped to avoid redundancy.
	//   - If no other temperature source is set, AXL temperature is included.
	//
	// If the resulting AXL data still doesn't fit the 184-bit data budget, activity LSBs are
	// truncated last (XYZ data preserved as the primary AXL signal).
	if (axl_sensor != nullptr) {
		DEBUG_TRACE("ArgosPacketBuilder::build_sensor_packet: axl_temp=%04X X=%04X Y=%04X Z=%04X activity=%02X",
				(unsigned int)axl_sensor->port[0],
				(unsigned int)axl_sensor->port[1],
				(unsigned int)axl_sensor->port[2],
				(unsigned int)axl_sensor->port[3],
				(unsigned int)axl_sensor->port[4]);
		const bool has_other_temp = (pressure_sensor != nullptr) || (sea_temp_sensor != nullptr);
		const bool axl_with_temp = !has_other_temp;
		if (axl_with_temp) {
			PACK_BITS((unsigned int)axl_sensor->port[0], packet, base_pos, 14);  // Temperature
		} else {
			DEBUG_TRACE("ArgosPacketBuilder::build_sensor_packet: AXL temp dropped (other temp source present)");
		}
		// Pack XYZ + activity, truncating activity LSBs if budget exhausted.
		unsigned int budget_left = (base_pos < SENSOR_PACKET_MAX_TX_BITS) ?
				(SENSOR_PACKET_MAX_TX_BITS - base_pos) : 0;
		auto pack_capped = [&](unsigned int value, unsigned int width) {
			unsigned int n = std::min(width, budget_left);
			if (n) {
				PACK_BITS(value >> (width - n), packet, base_pos, n);
				budget_left -= n;
			}
			if (n < width) {
				DEBUG_WARN("ArgosPacketBuilder::build_sensor_packet: AXL field truncated %u→%u bits", width, n);
			}
		};
		pack_capped((unsigned int)axl_sensor->port[1], 15);  // X
		pack_capped((unsigned int)axl_sensor->port[2], 15);  // Y
		pack_capped((unsigned int)axl_sensor->port[3], 15);  // Z
		pack_capped((unsigned int)axl_sensor->port[4], 8);   // Activity
	}

	size_bits = base_pos;

	if (size_bits > SENSOR_PACKET_MAX_TX_BITS) {
		DEBUG_WARN("ArgosPacketBuilder::build_sensor_packet: packet %u bits exceeds max %u data bits | too many sensors enabled | truncating",
				size_bits, SENSOR_PACKET_MAX_TX_BITS);
		size_bits = SENSOR_PACKET_MAX_TX_BITS;
	}

	// Always emit a full 24-byte LDA2 frame and embed CRC8 at byte 23.
	apply_lda2_crc8(packet);
	size_bits = LDA2_FRAME_BITS;

	return packet;
}

// ============================================================================
// RSPB Dedicated Packet Builders
// ============================================================================

// Common RSPB packing: header + time + GPS + battery (shared by long and short)
static unsigned int pack_rspb_common(KineisPacket &packet, unsigned int header,
		GPSLogEntry* gps_entry, bool is_out_of_zone, bool is_low_battery) {
	unsigned int base_pos = 0;

	// 3-bit packet type header
	PACK_BITS(header, packet, base_pos, 3);

	// Time
	uint16_t year;
	uint8_t month, day, hour, min, sec;
	convert_datetime_to_epoch(gps_entry->info.schedTime, year, month, day, hour, min, sec);
	PACK_BITS(day, packet, base_pos, 5);
	PACK_BITS(hour, packet, base_pos, 5);
	PACK_BITS(min, packet, base_pos, 6);

	// GPS
	if (gps_entry->info.valid) {
		unsigned int lat = ArgosPacketBuilder::convert_latitude(gps_entry->info.lat);
		PACK_BITS(lat, packet, base_pos, 21);
		unsigned int lon = ArgosPacketBuilder::convert_longitude(gps_entry->info.lon);
		PACK_BITS(lon, packet, base_pos, 22);
		unsigned int gspeed = ArgosPacketBuilder::convert_speed((double)gps_entry->info.gSpeed);
		PACK_BITS(gspeed, packet, base_pos, 7);
		PACK_BITS(is_out_of_zone, packet, base_pos, 1);
	} else {
		PACK_BITS(0xFFFFFFFF, packet, base_pos, 21);
		PACK_BITS(0xFFFFFFFF, packet, base_pos, 22);
		PACK_BITS(0xFF, packet, base_pos, 7);
		PACK_BITS(is_out_of_zone, packet, base_pos, 1);
	}

	// Battery
	unsigned int batt = ArgosPacketBuilder::convert_battery_voltage((unsigned int)gps_entry->info.batt_voltage);
	PACK_BITS(batt, packet, base_pos, 7);
	PACK_BITS(is_low_battery, packet, base_pos, 1);

	return base_pos;
}

KineisPacket ArgosPacketBuilder::build_rspb_long_packet(GPSLogEntry* gps_entry,
		ServiceSensorData *pressure_sensor,
		ServiceSensorData *thermistor_sensor,
		ServiceSensorData *axl_sensor,
		bool is_out_of_zone, bool is_low_battery,
		unsigned int mortality_confidence,
		unsigned int &size_bits) {

	DEBUG_TRACE("ArgosPacketBuilder::build_rspb_long_packet");
	KineisPacket packet;
	packet.assign(RSPB_LONG_PACKET_BYTES, 0);

	unsigned int base_pos = pack_rspb_common(packet, RSPB_LONG_HEADER, gps_entry, is_out_of_zone, is_low_battery);

	// Pressure: value (15 bits) + temperature (14 bits)
	if (pressure_sensor != nullptr) {
		PACK_BITS((unsigned int)pressure_sensor->port[0], packet, base_pos, 15);
		PACK_BITS((unsigned int)pressure_sensor->port[1], packet, base_pos, 14);
	} else {
		PACK_BITS(0, packet, base_pos, 15);
		PACK_BITS(0, packet, base_pos, 14);
	}

	// Thermistor body temperature (14 bits)
	if (thermistor_sensor != nullptr) {
		PACK_BITS((unsigned int)thermistor_sensor->port[0], packet, base_pos, 14);
	} else {
		PACK_BITS(0, packet, base_pos, 14);
	}

	// AXL X/Y/Z (15 bits each) + activity (8 bits)
	if (axl_sensor != nullptr) {
		PACK_BITS((unsigned int)axl_sensor->port[1], packet, base_pos, 15);  // X
		PACK_BITS((unsigned int)axl_sensor->port[2], packet, base_pos, 15);  // Y
		PACK_BITS((unsigned int)axl_sensor->port[3], packet, base_pos, 15);  // Z
		PACK_BITS((unsigned int)axl_sensor->port[4], packet, base_pos, 8);   // Activity
	} else {
		PACK_BITS(0, packet, base_pos, 15);
		PACK_BITS(0, packet, base_pos, 15);
		PACK_BITS(0, packet, base_pos, 15);
		PACK_BITS(0, packet, base_pos, 8);
	}

	// Mortality confidence (7 bits, 0-100%)
	unsigned int conf = (mortality_confidence > 100) ? 100 : mortality_confidence;
	PACK_BITS(conf, packet, base_pos, 7);

	// LDA2 firmware-embedded CRC8 at byte 23 (modem does not add CRC for LDA2).
	apply_lda2_crc8(packet);
	size_bits = LDA2_FRAME_BITS;

	DEBUG_INFO("ArgosPacketBuilder::build_rspb_long_packet: %u data bits + CRC | %s",
			RSPB_LONG_PACKET_DATA_BITS, Binascii::hexlify(packet).c_str());
	return packet;
}

KineisPacket ArgosPacketBuilder::build_rspb_short_packet(GPSLogEntry* gps_entry,
		ServiceSensorData *pressure_sensor,
		ServiceSensorData *thermistor_sensor,
		ServiceSensorData *axl_sensor,
		bool is_out_of_zone, bool is_low_battery,
		unsigned int mortality_confidence,
		unsigned int &size_bits) {

	DEBUG_TRACE("ArgosPacketBuilder::build_rspb_short_packet");
	KineisPacket packet;
	packet.assign(RSPB_SHORT_PACKET_BYTES, 0);

	unsigned int base_pos = pack_rspb_common(packet, RSPB_SHORT_HEADER, gps_entry, is_out_of_zone, is_low_battery);

	// Pressure value only (15 bits) — NO temperature (saves 14 bits for LDK)
	if (pressure_sensor != nullptr) {
		PACK_BITS((unsigned int)pressure_sensor->port[0], packet, base_pos, 15);
	} else {
		PACK_BITS(0, packet, base_pos, 15);
	}

	// Thermistor body temperature (14 bits)
	if (thermistor_sensor != nullptr) {
		PACK_BITS((unsigned int)thermistor_sensor->port[0], packet, base_pos, 14);
	} else {
		PACK_BITS(0, packet, base_pos, 14);
	}

	// AXL activity only (8 bits)
	if (axl_sensor != nullptr) {
		PACK_BITS((unsigned int)axl_sensor->port[4], packet, base_pos, 8);
	} else {
		PACK_BITS(0, packet, base_pos, 8);
	}

	// Mortality confidence (7 bits, 0-100%)
	unsigned int conf = (mortality_confidence > 100) ? 100 : mortality_confidence;
	PACK_BITS(conf, packet, base_pos, 7);

	size_bits = base_pos;
	packet.resize((size_bits+7)/8);

	DEBUG_INFO("ArgosPacketBuilder::build_rspb_short_packet: %u bits | %s",
			size_bits, Binascii::hexlify(packet).c_str());
	return packet;
}

// ArgosTxScheduler
