/**
 * @file lora_packet_builder.cpp
 * @brief LoRa packet builder — GPS, sensor, status, CloudLocate encoding.
 */

#include "lora_packet_builder.hpp"
#include "bitpack.hpp"
#include "packet_field_encoding.hpp"
#include "binascii.hpp"
#include "timeutils.hpp"
#include "debug.hpp"
#include <algorithm>

/// @brief Convert ground speed (mm/s) to 7-bit encoding.
/// @param x  Ground speed in mm/s (expected non-negative, GPS contract).
/// @return Encoded speed (0-127). 127 = max representable (~254 km/h) AND is
///         also the invalid-fix sentinel — decoder must use the `valid` flag
///         from the packet header to disambiguate.
unsigned int LoRaPacketBuilder::convert_speed(double x) {
	return PacketField::speed(x);
}

/// @brief Convert battery voltage (mV) to 7-bit encoding (20mV/unit, offset 2700mV).
/// @param battery_voltage  Voltage in mV.
/// @return Encoded battery (0-127). Encoding formula: max(0, mV-2700)/20.
///         Range covered: 2700-5240 mV in 20 mV steps.
///
/// IMPORTANT — DEAD ZONE BELOW 2700 mV:
///   All voltages ≤ 2700 mV encode to 0. A 2.0 V battery (dying) is
///   indistinguishable from a 2.7 V battery (just below ref) in the encoded
///   value. This is intentional — keeping the 2700 mV reference matches the
///   Argos packet builder for cross-platform decoder compatibility, and the
///   `is_low_battery` flag in the packet header provides the critical-state
///   indication when the encoded voltage is at the floor.
///
/// DECODER GUIDANCE:
///   - Decoded voltage = 2700 + (encoded * 20) mV
///   - If `encoded == 0 && is_low_battery == 1` → display "≤ 2.7 V (CRITICAL)"
///   - If `encoded == 127` → display "≥ 5.24 V"
///   - Otherwise → display 2700 + (encoded * 20) mV
unsigned int LoRaPacketBuilder::convert_battery_voltage(unsigned int battery_voltage) {
	return PacketField::battery_voltage(battery_voltage);
}

/// @brief Encode latitude as 21-bit unsigned (bit 20 = sign for negative).
/// @param x  Latitude in degrees.
/// @return 21-bit encoded latitude.
unsigned int LoRaPacketBuilder::convert_latitude(double x) {
	return PacketField::latitude(x);
}

/// @brief Encode longitude as 22-bit unsigned (bit 21 = sign for negative).
/// @param x  Longitude in degrees.
/// @return 22-bit encoded longitude.
unsigned int LoRaPacketBuilder::convert_longitude(double x) {
	return PacketField::longitude(x);
}

/// @brief Convert heading (degrees) to 8-bit encoding (~0.704 deg/unit).
/// @param x  Heading in degrees (expected [0, 360], GPS contract).
/// @return Encoded heading (0-254). 255 is reserved for the invalid-fix
///         sentinel — valid headings are clamped to 254 to keep the encoding
///         unambiguous against the sentinel.
unsigned int LoRaPacketBuilder::convert_heading(double x) {
	return PacketField::heading(x);
}

/// @brief Convert altitude (mm MSL) to 8-bit encoding (40m/unit, clamped 0-254).
/// @param x  Altitude in mm above MSL.
/// @return Encoded altitude (0-254, 255=invalid).
unsigned int LoRaPacketBuilder::convert_altitude(double x) {
	return PacketField::altitude(x);
}

unsigned int LoRaPacketBuilder::max_gps_entries(unsigned int max_payload_bytes) {
	unsigned int max_bits = max_payload_bytes * BITS_PER_BYTE;
	// GPS_MULTI v2 overhead: header(14) + count(4) + first_gps_full(86) = 104 bits
	unsigned int overhead_bits = BITS_HEADER + BITS_GPS_COUNT + BITS_GPS_FULL;
	if (max_bits < overhead_bits) return 0;
	// Each additional delta entry is 66 bits (lat + lon + speed + delta_t_min)
	unsigned int remaining = max_bits - overhead_bits;
	return 1 + remaining / BITS_GPS_DELTA;
}

KineisPacket LoRaPacketBuilder::build_gps_packet(std::vector<GPSLogEntry *> &v, bool is_out_of_zone,
                                                 bool is_low_battery, unsigned int max_payload_bytes,
                                                 unsigned int &size_bits) {
	DEBUG_TRACE("LoRaPacketBuilder::build_gps_packet: %u entries, max=%u bytes", v.size(), max_payload_bytes);

	unsigned int max_entries = max_gps_entries(max_payload_bytes);
	unsigned int num_entries = std::min((unsigned int)v.size(), std::min(max_entries, 15U));

	// `retrieve()` produces oldest-first; GPS_MULTI v2 emits newest-first so
	// entry[0] carries the absolute timestamp and subsequent entries encode
	// minutes-back deltas. Reverse only the working window we actually emit.
	if (num_entries > 0) std::reverse(v.begin(), v.begin() + num_entries);

	// Compute packet size — GPS_MULTI v2 has no global delta_time field.
	size_bits = BITS_HEADER + BITS_GPS_COUNT + BITS_GPS_FULL;
	if (num_entries > 1) size_bits += (num_entries - 1) * BITS_GPS_DELTA;

	unsigned int packet_bytes = (size_bits + 7) / 8;
	KineisPacket packet;
	packet.assign(packet_bytes, 0);

	unsigned int base_pos = 0;

	// Header
	uint8_t pkt_type = (num_entries <= 1) ? PKT_TYPE_GPS_SINGLE : PKT_TYPE_GPS_MULTI;
	PACK_BITS((unsigned int)pkt_type, packet, base_pos, BITS_PKT_TYPE);

	// Flags (4 bits): out_of_zone, low_battery, valid, fastloc=0
	bool valid = (num_entries > 0 && v[0]->info.valid);
	unsigned int flags =
	    ((is_out_of_zone ? 1U : 0U) << 3) | ((is_low_battery ? 1U : 0U) << 2) | ((valid ? 1U : 0U) << 1);
	PACK_BITS(flags, packet, base_pos, BITS_FLAGS);

	// Voltage from newest entry (or 0)
	unsigned int batt = 0;
	if (num_entries > 0) batt = convert_battery_voltage((unsigned int)v[0]->info.batt_voltage);
	PACK_BITS(batt, packet, base_pos, BITS_VOLTAGE);

	// GPS count
	PACK_BITS(num_entries, packet, base_pos, BITS_GPS_COUNT);

	// Newest entry — full record with absolute timestamp
	if (num_entries > 0) {
		GPSLogEntry *gps = v[0];
		uint16_t year;
		uint8_t month, day, hour, min, sec;
		convert_datetime_to_epoch(gps->info.schedTime, year, month, day, hour, min, sec);

		PACK_BITS((unsigned int)day, packet, base_pos, BITS_DAY);
		PACK_BITS((unsigned int)hour, packet, base_pos, BITS_HOUR);
		PACK_BITS((unsigned int)min, packet, base_pos, BITS_MIN);

		if (gps->info.valid) {
			unsigned int lat = convert_latitude(gps->info.lat);
			PACK_BITS(lat, packet, base_pos, BITS_LATITUDE);
			unsigned int lon = convert_longitude(gps->info.lon);
			PACK_BITS(lon, packet, base_pos, BITS_LONGITUDE);
			unsigned int speed = convert_speed((double)gps->info.gSpeed);
			PACK_BITS(speed, packet, base_pos, BITS_SPEED);
			unsigned int heading = convert_heading(gps->info.headMot);
			PACK_BITS(heading, packet, base_pos, BITS_HEADING);
			if (gps->info.fixType == FIXTYPE_3D) {
				unsigned int alt = convert_altitude((double)gps->info.hMSL);
				PACK_BITS(alt, packet, base_pos, BITS_ALTITUDE);
			} else {
				PACK_BITS((unsigned int)INVALID_ALTITUDE, packet, base_pos, BITS_ALTITUDE);
			}
			PACK_BITS(std::min((unsigned int)gps->info.numSV, 15U), packet, base_pos, BITS_NUMSV);
		} else {
			// Invalid fix: fill with max values
			PACK_BITS(0x1FFFFF, packet, base_pos, BITS_LATITUDE);
			PACK_BITS(0x3FFFFF, packet, base_pos, BITS_LONGITUDE);
			PACK_BITS(0x7F, packet, base_pos, BITS_SPEED);
			PACK_BITS(0xFF, packet, base_pos, BITS_HEADING);
			PACK_BITS((unsigned int)INVALID_ALTITUDE, packet, base_pos, BITS_ALTITUDE);
			PACK_BITS(0U, packet, base_pos, BITS_NUMSV);
		}
	}

	// Older entries — lat + lon + speed + per-entry minutes-back delta.
	// delta is computed against the PREVIOUS entry (one step closer to "now").
	for (unsigned int i = 1; i < num_entries; i++) {
		GPSLogEntry *gps = v[i];
		if (gps->info.valid) {
			unsigned int lat = convert_latitude(gps->info.lat);
			PACK_BITS(lat, packet, base_pos, BITS_LATITUDE);
			unsigned int lon = convert_longitude(gps->info.lon);
			PACK_BITS(lon, packet, base_pos, BITS_LONGITUDE);
			unsigned int speed = convert_speed((double)gps->info.gSpeed);
			PACK_BITS(speed, packet, base_pos, BITS_SPEED);
		} else {
			PACK_BITS(0x1FFFFF, packet, base_pos, BITS_LATITUDE);
			PACK_BITS(0x3FFFFF, packet, base_pos, BITS_LONGITUDE);
			PACK_BITS(0x7F, packet, base_pos, BITS_SPEED);
		}

		std::time_t prev_t = v[i - 1]->info.schedTime;
		std::time_t curr_t = v[i]->info.schedTime;
		unsigned int delta_t_min;
		if (prev_t <= curr_t) {
			// Out-of-order or equal timestamps — emit zero rather than a huge wraparound.
			delta_t_min = 0;
		} else {
			std::time_t delta_s = prev_t - curr_t;
			std::time_t delta_m = delta_s / 60;
			delta_t_min = (delta_m >= (std::time_t)DELTA_T_MIN_MAX) ? DELTA_T_MIN_MAX : (unsigned int)delta_m;
		}
		PACK_BITS(delta_t_min, packet, base_pos, BITS_DELTA_T_MIN);
	}

	DEBUG_INFO("LoRaPacketBuilder::build_gps_packet: entries=%u data=%s sz=%u bits", num_entries,
	           Binascii::hexlify(packet).c_str(), size_bits);

	return packet;
}

KineisPacket LoRaPacketBuilder::build_sensor_packet(GPSLogEntry *gps, ServiceSensorData *als, ServiceSensorData *ph,
                                                    ServiceSensorData *pressure, ServiceSensorData *sea_temp,
                                                    ServiceSensorData *axl, bool is_out_of_zone, bool is_low_battery,
                                                    unsigned int &size_bits) {
	DEBUG_TRACE("LoRaPacketBuilder::build_sensor_packet");

	// Detect fastloc
	bool is_fastloc = (gps != nullptr && gps->info.event_type == GPSEventType::FASTLOC);

	// Compute presence bitmask
	uint8_t mask = 0;
	if (gps != nullptr) mask |= 0x20;       // bit 5
	if (als != nullptr) mask |= 0x10;       // bit 4
	if (ph != nullptr) mask |= 0x08;        // bit 3
	if (pressure != nullptr) mask |= 0x04;  // bit 2
	if (sea_temp != nullptr) mask |= 0x02;  // bit 1
	if (axl != nullptr) mask |= 0x01;       // bit 0

	// Compute total size
	size_bits = BITS_HEADER + BITS_SENSOR_MASK;
	if (gps) size_bits += BITS_GPS_FULL;
	if (is_fastloc) size_bits += BITS_FASTLOC_QUALITY;
	if (als) size_bits += BITS_ALS;
	if (ph) size_bits += BITS_PH;
	if (pressure) size_bits += BITS_PRESSURE_FULL;
	if (sea_temp) size_bits += BITS_SEA_TEMP;
	if (axl) size_bits += BITS_AXL_FULL;

	unsigned int packet_bytes = (size_bits + 7) / 8;
	KineisPacket packet;
	packet.assign(packet_bytes, 0);

	unsigned int base_pos = 0;

	// Header
	PACK_BITS((unsigned int)PKT_TYPE_SENSOR, packet, base_pos, BITS_PKT_TYPE);

	// Flags (4 bits): out_of_zone, low_battery, valid, fastloc
	bool valid = (gps != nullptr && gps->info.valid);
	unsigned int flags = ((is_out_of_zone ? 1U : 0U) << 3) | ((is_low_battery ? 1U : 0U) << 2)
	                     | ((valid ? 1U : 0U) << 1) | (is_fastloc ? 1U : 0U);
	PACK_BITS(flags, packet, base_pos, BITS_FLAGS);

	// Voltage
	unsigned int batt = 0;
	if (gps) batt = convert_battery_voltage((unsigned int)gps->info.batt_voltage);
	PACK_BITS(batt, packet, base_pos, BITS_VOLTAGE);

	// Sensor presence bitmask
	PACK_BITS((unsigned int)mask, packet, base_pos, BITS_SENSOR_MASK);

	// GPS data (full entry with timestamp)
	if (gps) {
		uint16_t year;
		uint8_t month, day, hour, min, sec;
		convert_datetime_to_epoch(gps->info.schedTime, year, month, day, hour, min, sec);

		PACK_BITS((unsigned int)day, packet, base_pos, BITS_DAY);
		PACK_BITS((unsigned int)hour, packet, base_pos, BITS_HOUR);
		PACK_BITS((unsigned int)min, packet, base_pos, BITS_MIN);

		if (gps->info.valid) {
			unsigned int lat = convert_latitude(gps->info.lat);
			PACK_BITS(lat, packet, base_pos, BITS_LATITUDE);
			unsigned int lon = convert_longitude(gps->info.lon);
			PACK_BITS(lon, packet, base_pos, BITS_LONGITUDE);
			unsigned int speed = convert_speed((double)gps->info.gSpeed);
			PACK_BITS(speed, packet, base_pos, BITS_SPEED);
			unsigned int heading = convert_heading(gps->info.headMot);
			PACK_BITS(heading, packet, base_pos, BITS_HEADING);
			if (gps->info.fixType == FIXTYPE_3D) {
				unsigned int alt = convert_altitude((double)gps->info.hMSL);
				PACK_BITS(alt, packet, base_pos, BITS_ALTITUDE);
			} else {
				PACK_BITS((unsigned int)INVALID_ALTITUDE, packet, base_pos, BITS_ALTITUDE);
			}
			PACK_BITS(std::min((unsigned int)gps->info.numSV, 15U), packet, base_pos, BITS_NUMSV);
		} else {
			PACK_BITS(0x1FFFFF, packet, base_pos, BITS_LATITUDE);
			PACK_BITS(0x3FFFFF, packet, base_pos, BITS_LONGITUDE);
			PACK_BITS(0x7F, packet, base_pos, BITS_SPEED);
			PACK_BITS(0xFF, packet, base_pos, BITS_HEADING);
			PACK_BITS((unsigned int)INVALID_ALTITUDE, packet, base_pos, BITS_ALTITUDE);
			PACK_BITS(0U, packet, base_pos, BITS_NUMSV);
		}
	}

	// Fastloc quality metadata (appended after GPS data when fastloc flag is set)
	if (is_fastloc) {
		unsigned int fixType = std::min((unsigned int)gps->info.fixType, 3U);
		PACK_BITS(fixType, packet, base_pos, BITS_FIXTYPE);
		unsigned int hAcc_m = std::min((unsigned int)(gps->info.hAcc / MM_PER_METER), 65535U);
		PACK_BITS(hAcc_m, packet, base_pos, BITS_HACC);
		unsigned int vAcc_m = std::min((unsigned int)(gps->info.vAcc / MM_PER_METER), 65535U);
		PACK_BITS(vAcc_m, packet, base_pos, BITS_VACC);
		unsigned int pdop = std::min((unsigned int)(gps->info.pDOP * 10.0f), 255U);
		PACK_BITS(pdop, packet, base_pos, BITS_PDOP);
		unsigned int hdop = std::min((unsigned int)(gps->info.hDOP * 10.0f), 255U);
		PACK_BITS(hdop, packet, base_pos, BITS_HDOP);
		unsigned int ontime_s = std::min((unsigned int)(gps->info.onTime / 1000U), 1023U);
		PACK_BITS(ontime_s, packet, base_pos, BITS_ONTIME);
	}

	// ALS sensor
	if (als) {
		PACK_BITS((unsigned int)als->port[0], packet, base_pos, BITS_ALS);
	}

	// PH sensor
	if (ph) {
		PACK_BITS((unsigned int)ph->port[0], packet, base_pos, BITS_PH);
	}

	// Pressure sensor (pressure + temperature)
	if (pressure) {
		PACK_BITS((unsigned int)pressure->port[0], packet, base_pos, BITS_PRESSURE);
		PACK_BITS((unsigned int)pressure->port[1], packet, base_pos, BITS_PRESS_TEMP);
	}

	// Sea temperature / Thermistor
	if (sea_temp) {
		PACK_BITS((unsigned int)sea_temp->port[0], packet, base_pos, BITS_SEA_TEMP);
	}

	// Accelerometer
	if (axl) {
		PACK_BITS((unsigned int)axl->port[0], packet, base_pos, BITS_AXL_TEMP);
		PACK_BITS((unsigned int)axl->port[1], packet, base_pos, BITS_AXL_AXIS);
		PACK_BITS((unsigned int)axl->port[2], packet, base_pos, BITS_AXL_AXIS);
		PACK_BITS((unsigned int)axl->port[3], packet, base_pos, BITS_AXL_AXIS);
		PACK_BITS((unsigned int)axl->port[4], packet, base_pos, BITS_AXL_ACT);
	}

	DEBUG_INFO("LoRaPacketBuilder::build_sensor_packet: mask=0x%02x data=%s sz=%u bits", mask,
	           Binascii::hexlify(packet).c_str(), size_bits);

	return packet;
}

KineisPacket LoRaPacketBuilder::build_status_packet(unsigned int battery_voltage, bool is_low_battery,
                                                    unsigned int &size_bits) {
	DEBUG_TRACE("LoRaPacketBuilder::build_status_packet");

	size_bits = BITS_HEADER;                          // 14 bits
	unsigned int packet_bytes = (size_bits + 7) / 8;  // 2 bytes
	KineisPacket packet;
	packet.assign(packet_bytes, 0);

	unsigned int base_pos = 0;

	PACK_BITS((unsigned int)PKT_TYPE_STATUS, packet, base_pos, BITS_PKT_TYPE);

	// Flags (4 bits): out_of_zone=0, low_battery, valid=0, fastloc=0
	unsigned int flags = (is_low_battery ? 1U : 0U) << 2;
	PACK_BITS(flags, packet, base_pos, BITS_FLAGS);

	unsigned int batt = convert_battery_voltage(battery_voltage);
	PACK_BITS(batt, packet, base_pos, BITS_VOLTAGE);

	DEBUG_INFO("LoRaPacketBuilder::build_status_packet: data=%s sz=%u bits", Binascii::hexlify(packet).c_str(),
	           size_bits);

	return packet;
}

KineisPacket LoRaPacketBuilder::build_cloudlocate_packet(const uint8_t *blob, unsigned int blob_size, uint8_t format_id,
                                                         bool is_low_battery, unsigned int battery_voltage,
                                                         unsigned int &size_bits, uint32_t capture_rtc) {
	DEBUG_TRACE("LoRaPacketBuilder::build_cloudlocate_packet: format=%u blob_size=%u", format_id, blob_size);

	// type(3) + format(2) + flags(4) + voltage(7) + blob(blob_size*8)
	// Optional trailing capture-time field: flag(1) + seconds-of-day(17). LoRa is
	// length-delimited, so legacy (time-less) packets are simply shorter and the
	// decoder reads the time only if extra bits remain — backward compatible.
	size_bits = BITS_PKT_TYPE + BITS_CL_FORMAT + BITS_FLAGS + BITS_VOLTAGE + (blob_size * 8);
	unsigned int time_bits = (capture_rtc != 0) ? 18U : 0U;  // 1 flag + 17 seconds-of-day
	size_bits += time_bits;
	unsigned int packet_bytes = (size_bits + 7) / 8;
	KineisPacket packet;
	packet.assign(packet_bytes, 0);

	unsigned int base_pos = 0;

	// Packet type (3 bits)
	PACK_BITS((unsigned int)PKT_TYPE_CLOUDLOCATE, packet, base_pos, BITS_PKT_TYPE);

	// Format (2 bits): 00=MEASC12, 01=MEAS20, 10=MEAS50
	PACK_BITS((unsigned int)format_id, packet, base_pos, BITS_CL_FORMAT);

	// Flags (4 bits): out_of_zone=0, low_battery, valid=0, reserved=0
	unsigned int flags = (is_low_battery ? 1U : 0U) << 2;
	PACK_BITS(flags, packet, base_pos, BITS_FLAGS);

	// Battery voltage (7 bits)
	unsigned int batt = convert_battery_voltage(battery_voltage);
	PACK_BITS(batt, packet, base_pos, BITS_VOLTAGE);

	// Raw GNSS measurement blob
	for (unsigned int i = 0; i < blob_size; i++) {
		PACK_BITS((unsigned int)blob[i], packet, base_pos, 8);
	}

	// Optional capture-time: flag(1) + seconds-of-day(17). LoRa has room for all
	// formats (incl. MEAS50). The cloud combines the day (from elsewhere) with
	// seconds-of-day to get the exact measurement instant.
	if (capture_rtc != 0) {
		PACK_BITS(1U, packet, base_pos, 1);
		PACK_BITS((unsigned int)(capture_rtc % 86400U), packet, base_pos, 17);
	}

	DEBUG_INFO("LoRaPacketBuilder::build_cloudlocate_packet: format=%u blob_size=%u t_present=%u data=%s sz=%u bits",
	           format_id, blob_size, (unsigned)(capture_rtc != 0), Binascii::hexlify(packet).c_str(), size_bits);

	return packet;
}

// ============================================================================
// LoRaTxScheduler
// ============================================================================
