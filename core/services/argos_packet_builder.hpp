/**
 * @file argos_packet_builder.hpp
 * @brief Argos packet builder — constructs short, long, Doppler, sensor, RSPB, CloudLocate packets.
 *
 * All methods are static — no state, no dependencies on Service framework.
 * Used by ArgosTxService to build packets before sending via KineisDevice.
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "messages.hpp"
#include "kineis_device.hpp"
#include "config_store.hpp"
#include "service.hpp"
#include "dive_behaviour_tracker.hpp"  // Dx1Behaviour (plain data, every target)

/// @brief Argos packet encoding constants and static builders.
class ArgosPacketBuilder {
public:
	// === Packet format constants ===

	// LDA2 frame layout (applies to long, sensor, fastloc, RSPB long, CloudLocate MEAS20):
	// 24 bytes total = 23 bytes data (184 bits) + 1 byte CRC8 at byte 23.
	// LDK and VLDA4 frames have CRC integrated by the SMD/KIM2 module — only LDA2
	// requires the firmware to compute and embed CRC8 inside the payload.
	static constexpr unsigned int LDA2_FRAME_BITS = 192;
	static constexpr unsigned int LDA2_FRAME_BYTES = 24;
	static constexpr unsigned int LDA2_DATA_BITS = 184;  ///< Bits before CRC8

	// Short packet (LDK, modem-handled CRC)
	static constexpr unsigned int SHORT_PACKET_HEADER = 0b000;
	static constexpr unsigned int SHORT_PACKET_BITS = 96;
	static constexpr unsigned int SHORT_PACKET_PAYLOAD_BITS = 94;
	static constexpr unsigned int SHORT_PACKET_BYTES = 12;

	// Long packet (LDA2, firmware-embedded CRC8 at byte 23).
	// Shares the 3-bit header value 000 with Short Packet (and with the legacy
	// Doppler, byte0 = 0x00) — receivers discriminate by frame size + modulation.
	// Sizes ON AIR, after the backend pads (KIM2 and SMD agree in bytes): LDK
	// frames are always 16 B; LDA2 frames are rounded up to a multiple of 4 B
	// (min 4, max 24); VLDA4 frames are 3 B. Hence:
	//   Short (96 b)       : 12 B on LDA2, 16 B on LDK (zero tail)
	//   Long  (192 b)      : 24 B, LDA2 only
	//   Doppler (24 b)     :  4 B on LDA2, 16 B on LDK (13 zero bytes), 3 B on VLDA4
	//   CloudLocate MEAS20 : 24 B, LDA2 only
	// A Short is therefore NOT identified by "12 B" alone: on an LDK master it is
	// a 16 B frame, like a padded Doppler.
	static constexpr unsigned int LONG_PACKET_HEADER = 0b000;
	static constexpr unsigned int LONG_PACKET_BITS = LDA2_FRAME_BITS;
	static constexpr unsigned int LONG_PACKET_PAYLOAD_BITS = LDA2_DATA_BITS;
	static constexpr unsigned int LONG_PACKET_BYTES = LDA2_FRAME_BYTES;

	// Fastloc degraded PVT packet (LDA2, firmware-embedded CRC8 at byte 23)
	static constexpr unsigned int FASTLOC_PACKET_HEADER = 0b010;
	static constexpr unsigned int FASTLOC_PACKET_BITS = LDA2_FRAME_BITS;
	static constexpr unsigned int FASTLOC_PACKET_BYTES = LDA2_FRAME_BYTES;

	// CloudLocate packet (Type 7): header(3) + format(2) + blob(variable) + battery(8)
	// MEAS20 variant goes on LDA2 with firmware CRC8; MEASC12 stays on LDK (modem CRC).
	static constexpr unsigned int CLOUDLOCATE_PACKET_HEADER = 0b111;
	static constexpr unsigned int CLOUDLOCATE_MEASC12_BITS = 128;  // LDK
	static constexpr unsigned int CLOUDLOCATE_MEASC12_BYTES = 16;
	static constexpr unsigned int CLOUDLOCATE_MEAS20_BITS = LDA2_FRAME_BITS;
	static constexpr unsigned int CLOUDLOCATE_MEAS20_BYTES = LDA2_FRAME_BYTES;
	// Optional capture-time field (2026-06) packed into the previously zero
	// padding, gated by a 1-bit "time present" flag. BACKWARD COMPATIBLE: legacy
	// frames zero-pad that region, so flag=0 means "no time" → old frames decode
	// unchanged. MEASC12/LDK has room for full seconds-of-day (17b); MEAS20/LDA2
	// only fits a 10-bit age (capture→TX seconds, ~17 min) before the CRC8.
	static constexpr unsigned int CLOUDLOCATE_TIME_FLAG_BITS = 1;
	static constexpr unsigned int CLOUDLOCATE_SOD_BITS = 17;  // seconds-of-day 0..86399 (MEASC12 + LoRa)
	static constexpr unsigned int CLOUDLOCATE_AGE_BITS = 10;  // capture→TX age 0..1023 s (MEAS20/LDA2)
	/// First age the 10-bit field can no longer represent honestly. 1023 is itself
	/// excluded because it is indistinguishable from a saturated value, so a frame
	/// carrying it could not be told apart from one that is arbitrarily older. At or
	/// above this, build_cloudlocate_packet withholds the "time present" flag and
	/// the Argos live path gives the TX slot back to a Doppler ping.
	static constexpr unsigned int CLOUDLOCATE_AGE_MAX_S = (1U << CLOUDLOCATE_AGE_BITS) - 1U;  // 1023 s

	// Sensor packet (Type 1, LDA2). Always emitted as a full 24-byte LDA2 frame so the
	// CRC sits at byte 23. Adaptive LDK fallback for tiny sensor packets is therefore
	// disabled — sensor packets always go on LDA2 now.
	//
	// Layout: header(3) + base(75) + sensor_mask(5) + sensor_data(variable) + zero-pad + CRC8.
	// The 5-bit mask at offset 78 makes the packet self-describing, so decoders don't
	// need an external sensor-config DB. Bit order (MSB first):
	//   bit 4 = ALS, bit 3 = PH, bit 2 = Pressure, bit 1 = SeaTemp/Thermistor, bit 0 = AXL.
	// AXL state (full vs no_temp) is inferred deterministically from the remaining
	// budget after non-AXL sensors are packed (see apply_axl_layout in the .cpp).
	static constexpr unsigned int SENSOR_PACKET_HEADER = 0b001;
	static constexpr unsigned int SENSOR_PACKET_BYTES = LDA2_FRAME_BYTES;
	static constexpr unsigned int SENSOR_PACKET_MAX_TX_BYTES = LDA2_FRAME_BYTES;
	static constexpr unsigned int SENSOR_PACKET_MAX_TX_BITS = LDA2_DATA_BITS;
	static constexpr unsigned int SENSOR_PACKET_MASK_BITS = 5;
	static constexpr unsigned int SENSOR_PACKET_MASK_ALS = 1u << 4;
	static constexpr unsigned int SENSOR_PACKET_MASK_PH = 1u << 3;
	static constexpr unsigned int SENSOR_PACKET_MASK_PRESSURE = 1u << 2;
	static constexpr unsigned int SENSOR_PACKET_MASK_SEATEMP = 1u << 1;
	static constexpr unsigned int SENSOR_PACKET_MASK_AXL = 1u << 0;

	// Doppler packet (24 bits, VLDA4)
	// byte0 = 0x00 is the LinkIt Doppler marker, FROZEN forever, and so is
	// byte1 = BATT7 | LB. The legacy frame is those two bytes plus byte2 = 0x00.
	static constexpr unsigned int DOPPLER_PACKET_BITS = 24;
	static constexpr unsigned int DOPPLER_PACKET_PAYLOAD_BITS = 24;
	static constexpr unsigned int DOPPLER_PACKET_BYTES = 3;

	// DX1 Doppler extension (turtle SWS health + behaviour), selected by
	// SWS_SENSOR_ENABLE_TX_MODE (UNP26, see dx1_effective()). ONE 16-byte
	// image is built; each modulation sends a prefix of it (doppler_view), so
	// a modulation change can never cut a field in half or leave a stale CRC:
	//   byte2      VER(2)=01 | EXT(1)=0 | DRY(3) | DEND(2)
	//   byte3      CRC8 over bytes 0..2       (views of 32 bits and more)
	//   bits 32-111 rich block                (LDK view only)
	//   bits 112-127 reserved, 0x00 (Phase 2)
	// The byte vectors in tests/src/dx1_doppler_test.cpp are the wire contract the
	// ground decoder must reproduce.
	static constexpr unsigned int DOPPLER_DX1_VER = 1;
	static constexpr unsigned int DOPPLER_DX1_IMAGE_BYTES = 16;
	static constexpr unsigned int DOPPLER_DX1_IMAGE_BITS = 128;
	static constexpr unsigned int DOPPLER_DX1_VLDA4_BYTES = 3;
	static constexpr unsigned int DOPPLER_DX1_LDA2_BYTES = 4;
	static constexpr unsigned int DOPPLER_DX1_LDK_BYTES = 16;
	static constexpr unsigned int DOPPLER_DX1_CRC_BYTE = 3;
	static constexpr unsigned int DOPPLER_DX1_RICH_BIT = 32;  ///< first bit of the rich block
	static constexpr unsigned int DOPPLER_DX1_RICH_BITS = 80;
	static constexpr unsigned int DOPPLER_DX1_DEND_MASK = 0x03;  ///< DEND = low 2 bits of byte 2
	static constexpr uint32_t DOPPLER_DX1_OBS_MIN_S = 43200;     ///< PCT/DRY unknown below 12 h observed

	/// @brief Already-encoded DX1 field codes (0 = unknown / absent everywhere).
	/// Plain data: every value is a code, never a physical quantity, so the
	/// builder only packs.
	struct DopplerExt {
		uint8_t dry_class = 0;    ///< byte2 DRY, 0..7
		uint8_t dive_end = 0;     ///< byte2 DEND, 0..3
		uint8_t latch = 0;        ///< 0 absent, 1 dive edge, 2 no dive edge since reset
		uint8_t blind = 0;        ///< 1 bit
		uint8_t partial = 0;      ///< 1 bit
		uint16_t surf_seq = 0;    ///< 9 bits, 0 none, 1..511
		uint8_t rst = 0;          ///< 3 bits, saturating 7
		uint8_t pct = 0;          ///< 7 bits, pct_dry_code()
		uint8_t n_surf = 0;       ///< E4M3(n + 1)
		uint8_t s_last = 0;       ///< E4M3 seconds
		uint8_t d_prev = 0;       ///< E4M3 seconds
		uint8_t d_max = 0;        ///< E4M3 seconds
		uint8_t delay = 0;        ///< E3M3 of 50-us units
		uint8_t contrast = 0;     ///< E3M3 of contrast x10
		uint8_t dto = 0;          ///< Q4
		uint8_t fsc = 0;          ///< Q4
		uint8_t src = 0;          ///< Q4
		uint8_t surf_method = 0;  ///< 3 bits
		uint8_t dfloor = 0;       ///< 1 bit
		uint8_t adcf = 0;         ///< 1 bit
	};

	/// @brief Floor quasi-log code: exact below 2^M, then E exponent bits and M
	/// mantissa bits, saturating at the all-ones code. @p v must be >= 1 (code 0
	/// is reserved for "unknown" and is the caller's to emit).
	static unsigned int dx1_em_encode(uint32_t v, unsigned int e_bits, unsigned int m_bits);
	/// @brief 4-bit count code: 0-7 exact, then 8/16/32/64/128/256/1024/4096 floors.
	static unsigned int dx1_q4_encode(uint32_t n);
	/// @brief PCT_DRY code: 0 unknown (obs < 12 h), else 1 + round(100 dry/obs).
	static unsigned int dx1_pct_code(uint32_t dry_s, uint32_t obs_s);
	/// @brief DRY class: 0 unknown, 1..7 at per-mille bounds 20/40/80/160/320/640.
	static unsigned int dx1_dry_class(uint32_t dry_s, uint32_t obs_s);
	/// @brief Fill the SWS health codes of @p ext (SAMPLE_DELAY, CONTRAST, DTO,
	/// FSC, SRC, DFLOOR, ADCF) from raw SWS values. Pure integer encoding.
	static void dx1_encode_health(DopplerExt &ext, uint32_t sample_delay_us, uint16_t contrast_x10, bool is_calibrated,
	                              uint16_t threshold_air, uint32_t dive_timeout_count, uint32_t force_surface_count,
	                              uint32_t stuck_recovery_count, uint32_t delay_min_us, uint32_t delay_max_us,
	                              bool adc_fault);
	/// @brief Fill the behaviour codes of @p ext (DRY, LATCH, BLIND, PARTIAL,
	/// SURF_SEQ, PCT_DRY, N_SURF, S_LAST, D_PREV, D_MAX, SURF_METHOD) from the
	/// raw values the tracker latched.
	static void dx1_encode_behaviour(DopplerExt &ext, const Dx1Behaviour &b);
	/// @brief CRC8 of the first three bytes (crc8.hpp algorithm), no allocation.
	static uint8_t dx1_crc8_head(const KineisPacket &img);
	/// @brief Rewrite DEND in byte 2 and, if the frame has a byte 3, its CRC.
	/// RAM only, no allocation: this is the ONE thing DX1 adds to the surfacing
	/// path. No-op on a frame shorter than 3 bytes.
	static void dx1_refresh_dive_end(KineisPacket &img, unsigned int dive_end);
	/// @brief Shrink a DX1 image to the prefix the modulation carries and return
	/// its size in bits (VLDA4 3 B/24, LDA2 4 B/32, LDK 16 B/128). A non-DX1 packet
	/// (@p is_dx1 false) is left untouched and @p in_bits returned unchanged.
	/// DX1-ness is the caller's explicit flag, never inferred from the length.
	static unsigned int doppler_view(KineisPacket &img, KineisModulation mode, unsigned int in_bits, bool is_dx1);

	// RSPB Doppler (Type 6, VLDA4)
	static constexpr unsigned int RSPB_DOPPLER_HEADER = 0b110;
	static constexpr unsigned int RSPB_DOPPLER_PACKET_BITS = 24;
	static constexpr unsigned int RSPB_DOPPLER_PACKET_BYTES = 3;

	// RSPB dedicated packet formats (bird tracker with mortality)
	static constexpr unsigned int RSPB_LONG_HEADER = 0b100;
	static constexpr unsigned int RSPB_SHORT_HEADER = 0b101;
	static constexpr unsigned int RSPB_LONG_PACKET_DATA_BITS = 181;         ///< Useful bits before CRC
	static constexpr unsigned int RSPB_LONG_PACKET_BITS = LDA2_FRAME_BITS;  ///< Full LDA2 frame (data + CRC)
	static constexpr unsigned int RSPB_LONG_PACKET_BYTES = LDA2_FRAME_BYTES;
	static constexpr unsigned int RSPB_SHORT_PACKET_BITS = 122;
	static constexpr unsigned int RSPB_SHORT_PACKET_BYTES = 16;

	// === Encoding constants ===

	static constexpr unsigned int FIXTYPE_3D = 3;
	static constexpr unsigned int MM_PER_METER = 1000;
	static constexpr unsigned int MM_PER_KM = 1000000;
	static constexpr unsigned int MV_PER_UNIT = 20;
	static constexpr unsigned int METRES_PER_UNIT = 40;
	static constexpr double DEGREES_PER_UNIT = 1.0 / 1.42;  ///< ~0.704 deg/unit
	static constexpr unsigned int BITS_PER_BYTE = 8;
	static constexpr unsigned int MIN_ALTITUDE = 0;
	static constexpr unsigned int MAX_ALTITUDE = 254;
	static constexpr unsigned int INVALID_ALTITUDE = 255;
	static constexpr unsigned int REF_BATT_MV = 2700;
	static constexpr unsigned int LON_LAT_RESOLUTION = 10000;
	static constexpr int NEG_LON_LAT_RESOLUTION = -10000;
	static constexpr unsigned int MAX_GPS_ENTRIES_IN_PACKET = 3;  ///< Reduced from 4 to fit CRC8 in 24-byte LDA2 frame
	static constexpr unsigned int SECONDS_PER_HOUR = 3600;

	// === Conversion helpers ===

	static unsigned int convert_altitude(double x);
	static unsigned int convert_heading(double x);
	static unsigned int convert_speed(double x);
	static unsigned int convert_latitude(double x);
	static unsigned int convert_longitude(double x);
	static unsigned int convert_battery_voltage(unsigned int battery_voltage);

	// === Packet builders ===

	/// @brief Build short GPS packet (1 fix, 96 bits).
	static KineisPacket build_short_packet(GPSLogEntry *v, bool is_out_of_zone, bool is_low_battery);

	/// @brief Build long GPS packet (up to 4 fixes, 224 bits).
	static KineisPacket build_long_packet(std::vector<GPSLogEntry *> &v, bool is_out_of_zone, bool is_low_battery,
	                                      BaseDeltaTimeLoc delta_time_loc);

	/// @brief Build fastloc degraded PVT packet (192 bits).
	static KineisPacket build_fastloc_packet(GPSLogEntry *v, bool is_low_battery);

	/// @brief Build CloudLocate raw measurement packet.
	/// @param capture_rtc  RTC epoch (s) when the snapshot was captured. 0 = unknown
	///                     → no time field emitted (legacy layout, flag=0).
	/// @param now_rtc      RTC epoch (s) at TX time (used to compute MEAS20 age).
	static KineisPacket build_cloudlocate_packet(const uint8_t *blob, unsigned int blob_size, uint8_t format_id,
	                                             unsigned int battery_voltage, bool is_low_battery,
	                                             uint32_t capture_rtc = 0, uint32_t now_rtc = 0);

	/// @brief Get CloudLocate packet size in bits for a given format.
	static unsigned int cloudlocate_packet_bits(uint8_t format_id);

	/// @brief Build GNSS packet (auto-selects short or long based on entry count).
	static KineisPacket build_gnss_packet(std::vector<GPSLogEntry *> &v, bool is_out_of_zone, bool is_low_battery,
	                                      BaseDeltaTimeLoc delta_time_loc, unsigned int &size_bits);

	/// @brief Build certification TX packet from a hex payload, sized to the
	///        modulation it will be transmitted on.
	/// @param cert_tx_payload  operator-supplied hex string (CERT_TX_PAYLOAD).
	/// @param mode             modulation the frame will go out on.
	/// @param[out] size_bits   frame size handed to the device.
	///
	/// The three modulations do not carry the same frame -- VLDA4 holds three
	/// bytes, LDK twelve, LDA2 twelve or twenty-four -- and CERT_TX_PAYLOAD is
	/// the one payload in the firmware that comes straight from an operator with
	/// nothing between it and the radio. Sizing it here means an over-long
	/// payload is truncated rather than refused by the module, because that
	/// refusal counts as a device error and three of them suspend TX.
	static KineisPacket build_certification_packet(std::string cert_tx_payload, KineisModulation mode,
	                                               unsigned int &size_bits);

	/// @brief Build Doppler packet (24 bits, no GPS).
	/// @param ext  nullptr (default): the legacy 3-byte / 24-bit frame, bit-exact.
	///             Otherwise the 16-byte / 128-bit DX1 image; send it through
	///             doppler_view() for the modulation actually used.
	static KineisPacket build_doppler_packet(unsigned int battery, bool is_low_battery, unsigned int &size_bits,
	                                         const DopplerExt *ext = nullptr);

	/// @brief Build sensor packet (GPS + optional ALS/PH/pressure/temp/AXL).
	static KineisPacket build_sensor_packet(GPSLogEntry *v, ServiceSensorData *als_sensor, ServiceSensorData *ph_sensor,
	                                        ServiceSensorData *pressure_sensor, ServiceSensorData *sea_temp_sensor,
	                                        ServiceSensorData *axl_sensor, bool is_out_of_zone, bool is_low_battery,
	                                        unsigned int &size_bits);

	/// @brief Build RSPB long packet (LDA2, bird tracker with mortality).
	static KineisPacket build_rspb_long_packet(GPSLogEntry *gps, ServiceSensorData *pressure_sensor,
	                                           ServiceSensorData *thermistor_sensor, ServiceSensorData *axl_sensor,
	                                           bool is_out_of_zone, bool is_low_battery,
	                                           unsigned int mortality_confidence, unsigned int &size_bits);

	/// @brief Build RSPB short packet (LDK, bird tracker with mortality).
	static KineisPacket build_rspb_short_packet(GPSLogEntry *gps, ServiceSensorData *pressure_sensor,
	                                            ServiceSensorData *thermistor_sensor, ServiceSensorData *axl_sensor,
	                                            bool is_out_of_zone, bool is_low_battery,
	                                            unsigned int mortality_confidence, unsigned int &size_bits);

	/// @brief Build RSPB Doppler packet (VLDA4, mortality + activity).
	static KineisPacket build_rspb_doppler_packet(unsigned int battery_soc, unsigned int activity,
	                                              unsigned int mortality_confidence, unsigned int &size_bits);
};
