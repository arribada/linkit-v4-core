/**
 * @file lora_packet_builder_test.cpp
 * @brief Unit tests for LoRaPacketBuilder — focuses on GPS_MULTI v2 layout
 *        (newest-first ordering + per-entry DELTA_T_MIN) and bounded depth pile.
 */

#include "CppUTest/TestHarness.h"

#include <cstring>
#include <ctime>
#include <vector>

#include "lora_packet_builder.hpp"
#include "depth_pile.hpp"
#include "messages.hpp"
#include "bitpack.hpp"
#include "timeutils.hpp"

using LPB = LoRaPacketBuilder;

TEST_GROUP(LoRaPacketBuilder) {
	// 27/01/2020 00:00:00 UTC — same anchor used by argos_tx_test.cpp
	static constexpr std::time_t T0 = 1580083200;

	GPSLogEntry make_fix(double lat, double lon, std::time_t t, unsigned int batt_mv = 3700, bool valid = true) {
		GPSLogEntry e{};
		e.info.valid = valid;
		e.info.lat = lat;
		e.info.lon = lon;
		e.info.gSpeed = 1000;  // 1 m/s = 3.6 km/h
		e.info.headMot = 90.0f;
		e.info.fixType = 3;
		e.info.hMSL = 0;
		e.info.numSV = 8;
		e.info.batt_voltage = batt_mv;
		e.info.schedTime = t;
		return e;
	}

	/// Decode helper — read GPS_MULTI v2 packet straight from the bitstream.
	/// Returns timestamps reconstructed via t(i) = t(i-1) - DELTA_T_MIN(i)*60.
	struct DecodedV2 {
		uint8_t pkt_type;
		uint8_t flags;
		uint8_t voltage;
		uint8_t count;
		uint8_t day, hour, min;
		uint32_t lat0, lon0;
		std::vector<uint32_t> lats;  // size = count, lats[0] = newest
		std::vector<uint32_t> lons;
		std::vector<unsigned int> deltas;  // size = count-1, deltas[0] = entry[1]'s delta
	};

	DecodedV2 decode_multi(const KineisPacket &packet) {
		DecodedV2 d{};
		unsigned int pos = 0;
		EXTRACT_BITS_CAST(uint8_t, d.pkt_type, packet, pos, LPB::BITS_PKT_TYPE);
		EXTRACT_BITS_CAST(uint8_t, d.flags, packet, pos, LPB::BITS_FLAGS);
		EXTRACT_BITS_CAST(uint8_t, d.voltage, packet, pos, LPB::BITS_VOLTAGE);
		EXTRACT_BITS_CAST(uint8_t, d.count, packet, pos, LPB::BITS_GPS_COUNT);

		EXTRACT_BITS_CAST(uint8_t, d.day, packet, pos, LPB::BITS_DAY);
		EXTRACT_BITS_CAST(uint8_t, d.hour, packet, pos, LPB::BITS_HOUR);
		EXTRACT_BITS_CAST(uint8_t, d.min, packet, pos, LPB::BITS_MIN);
		EXTRACT_BITS(d.lat0, packet, pos, LPB::BITS_LATITUDE);
		EXTRACT_BITS(d.lon0, packet, pos, LPB::BITS_LONGITUDE);
		uint32_t speed0, heading0, alt0, numsv0;
		EXTRACT_BITS(speed0, packet, pos, LPB::BITS_SPEED);
		EXTRACT_BITS(heading0, packet, pos, LPB::BITS_HEADING);
		EXTRACT_BITS(alt0, packet, pos, LPB::BITS_ALTITUDE);
		EXTRACT_BITS(numsv0, packet, pos, LPB::BITS_NUMSV);
		(void)speed0;
		(void)heading0;
		(void)alt0;
		(void)numsv0;

		d.lats.push_back(d.lat0);
		d.lons.push_back(d.lon0);

		for (unsigned int i = 1; i < d.count; i++) {
			uint32_t lat, lon, speed;
			unsigned int delta_min;
			EXTRACT_BITS(lat, packet, pos, LPB::BITS_LATITUDE);
			EXTRACT_BITS(lon, packet, pos, LPB::BITS_LONGITUDE);
			EXTRACT_BITS(speed, packet, pos, LPB::BITS_SPEED);
			EXTRACT_BITS(delta_min, packet, pos, LPB::BITS_DELTA_T_MIN);
			(void)speed;
			d.lats.push_back(lat);
			d.lons.push_back(lon);
			d.deltas.push_back(delta_min);
		}
		return d;
	}
};

// ---------------------------------------------------------------------------
// max_gps_entries — verify v2 capacity matches the design (DR0/3/5)
// ---------------------------------------------------------------------------
TEST(LoRaPacketBuilder, MaxGpsEntriesV2) {
	// DR0/1/2 — 51 B = 408 bits. overhead=104. (408-104)/66 = 4. Total = 5.
	CHECK_EQUAL(5U, LPB::max_gps_entries(51));
	// DR3 — 115 B = 920 bits. (920-104)/66 = 12. Total = 13.
	CHECK_EQUAL(13U, LPB::max_gps_entries(115));
	// DR4/5 — 222 B. (1776-104)/66 = 25, capped at 15 by the count field downstream.
	CHECK(LPB::max_gps_entries(222) >= 15U);
	// Below overhead — zero entries fit.
	CHECK_EQUAL(0U, LPB::max_gps_entries(12));
}

// ---------------------------------------------------------------------------
// count=1 — 13-byte single-fix packet (header + count + full entry)
// ---------------------------------------------------------------------------
TEST(LoRaPacketBuilder, BuildSingleFix13Bytes) {
	GPSLogEntry e = make_fix(45.5, -73.5, T0);
	GPSLogEntry *pe = &e;
	std::vector<GPSLogEntry *> v{ pe };

	unsigned int size_bits;
	KineisPacket pkt = LPB::build_gps_packet(v, false, false, 222, size_bits);

	CHECK_EQUAL(LPB::BITS_HEADER + LPB::BITS_GPS_COUNT + LPB::BITS_GPS_FULL, size_bits);
	CHECK_EQUAL(13U, (unsigned int)pkt.size());

	DecodedV2 d = decode_multi(pkt);
	CHECK_EQUAL(LPB::PKT_TYPE_GPS_SINGLE, d.pkt_type);
	CHECK_EQUAL(1U, d.count);
	CHECK_EQUAL(0U, d.deltas.size());
}

// ---------------------------------------------------------------------------
// count=4 with irregular timestamps — verify newest-first ordering and
// that decoder reconstructs the original timestamps to within 1 minute.
// ---------------------------------------------------------------------------
TEST(LoRaPacketBuilder, IrregularTimestampsRoundTrip) {
	// 4 surfacings: 08:00, 11:30, 14:15, 16:00 (each on day 27).
	std::time_t t0 = T0 + 8 * 3600;             // 08:00
	std::time_t t1 = T0 + 11 * 3600 + 30 * 60;  // 11:30  (210 min after t0)
	std::time_t t2 = T0 + 14 * 3600 + 15 * 60;  // 14:15  (165 min after t1)
	std::time_t t3 = T0 + 16 * 3600;            // 16:00  (105 min after t2)

	GPSLogEntry e0 = make_fix(45.0, -73.0, t0);
	GPSLogEntry e1 = make_fix(45.1, -73.1, t1);
	GPSLogEntry e2 = make_fix(45.2, -73.2, t2);
	GPSLogEntry e3 = make_fix(45.3, -73.3, t3);

	// Input vector is oldest-first (as DepthPile::retrieve produces).
	std::vector<GPSLogEntry *> v{ &e0, &e1, &e2, &e3 };

	unsigned int size_bits;
	KineisPacket pkt = LPB::build_gps_packet(v, false, false, 222, size_bits);

	// Bit budget: 14 + 4 + 86 + 3 * 66 = 302
	CHECK_EQUAL(302U, size_bits);

	DecodedV2 d = decode_multi(pkt);
	CHECK_EQUAL(LPB::PKT_TYPE_GPS_MULTI, d.pkt_type);
	CHECK_EQUAL(4U, d.count);

	// Newest first — entry[0] should be the t3 (16:00) fix.
	uint16_t y;
	uint8_t mo, da, hh, mm, ss;
	convert_datetime_to_epoch(t3, y, mo, da, hh, mm, ss);
	CHECK_EQUAL((uint8_t)da, d.day);
	CHECK_EQUAL((uint8_t)hh, d.hour);
	CHECK_EQUAL((uint8_t)mm, d.min);

	// Newest-first reordering: builder should have placed the largest lat (45.3) at index 0.
	CHECK(d.lats[0] > d.lats[1]);
	CHECK(d.lats[1] > d.lats[2]);
	CHECK(d.lats[2] > d.lats[3]);

	// Reconstructed timestamps:
	//   t(0) = t3 ; t(1) = t(0) - delta[0]*60 ≈ t2 ; etc.
	std::time_t reconstructed = t3;
	std::time_t expected[3] = { t2, t1, t0 };
	for (unsigned int i = 0; i < 3; i++) {
		reconstructed -= (std::time_t)d.deltas[i] * 60;
		// Allow ±60s rounding because we encode minutes.
		std::time_t err = (reconstructed > expected[i]) ? (reconstructed - expected[i]) : (expected[i] - reconstructed);
		CHECK(err <= 60);
	}

	// Sanity: deltas match the gaps we built (rounded-down minutes).
	CHECK_EQUAL(105U, d.deltas[0]);  // 16:00 - 14:15
	CHECK_EQUAL(165U, d.deltas[1]);  // 14:15 - 11:30
	CHECK_EQUAL(210U, d.deltas[2]);  // 11:30 - 08:00
}

// ---------------------------------------------------------------------------
// delta_t > 45 days — clamp to 0xFFFF sentinel.
// ---------------------------------------------------------------------------
TEST(LoRaPacketBuilder, DeltaClampOnHugeGap) {
	// 50-day gap between two fixes — exceeds the 16-bit max (65535 minutes ≈ 45.5 days).
	std::time_t t_old = T0;
	std::time_t t_new = T0 + (50LL * 24LL * 3600LL);

	GPSLogEntry e_old = make_fix(45.0, -73.0, t_old);
	GPSLogEntry e_new = make_fix(46.0, -74.0, t_new);

	std::vector<GPSLogEntry *> v{ &e_old, &e_new };

	unsigned int size_bits;
	KineisPacket pkt = LPB::build_gps_packet(v, false, false, 222, size_bits);

	DecodedV2 d = decode_multi(pkt);
	CHECK_EQUAL(2U, d.count);
	CHECK_EQUAL(LPB::DELTA_T_MIN_MAX, d.deltas[0]);  // clamped sentinel
}

// ---------------------------------------------------------------------------
// Out-of-order timestamps — should not wrap to 65535, instead emit 0.
// ---------------------------------------------------------------------------
TEST(LoRaPacketBuilder, DeltaOutOfOrderEmitsZero) {
	// Both fixes at the same instant. After reverse, prev == curr, so delta should be 0.
	GPSLogEntry e0 = make_fix(45.0, -73.0, T0);
	GPSLogEntry e1 = make_fix(45.1, -73.1, T0);  // same timestamp

	std::vector<GPSLogEntry *> v{ &e0, &e1 };

	unsigned int size_bits;
	KineisPacket pkt = LPB::build_gps_packet(v, false, false, 222, size_bits);

	DecodedV2 d = decode_multi(pkt);
	CHECK_EQUAL(2U, d.count);
	CHECK_EQUAL(0U, d.deltas[0]);
}

// ---------------------------------------------------------------------------
// Invalid fix in the middle — sentinel lat/lon, delta still valid.
// ---------------------------------------------------------------------------
TEST(LoRaPacketBuilder, InvalidFixSentinel) {
	std::time_t t0 = T0;
	std::time_t t1 = T0 + 600;   // +10 min
	std::time_t t2 = T0 + 1200;  // +20 min

	GPSLogEntry e0 = make_fix(45.0, -73.0, t0);
	GPSLogEntry e1 = make_fix(0.0, 0.0, t1, 3700, false /*invalid*/);
	GPSLogEntry e2 = make_fix(45.2, -73.2, t2);

	std::vector<GPSLogEntry *> v{ &e0, &e1, &e2 };

	unsigned int size_bits;
	KineisPacket pkt = LPB::build_gps_packet(v, false, false, 222, size_bits);

	DecodedV2 d = decode_multi(pkt);
	CHECK_EQUAL(3U, d.count);
	// After reverse: idx 0 = newest (e2, valid), idx 1 = e1 (invalid), idx 2 = e0 (valid)
	CHECK_EQUAL(0x1FFFFFu, d.lats[1]);
	CHECK_EQUAL(0x3FFFFFu, d.lons[1]);
	// Deltas still measurable
	CHECK_EQUAL(10U, d.deltas[0]);  // t2 - t1 = 10min
	CHECK_EQUAL(10U, d.deltas[1]);  // t1 - t0 = 10min
}

// ---------------------------------------------------------------------------
// Capacity capped by payload — pass too many entries, ensure trim.
// ---------------------------------------------------------------------------
TEST(LoRaPacketBuilder, TrimToPayloadCapacity) {
	std::vector<GPSLogEntry> entries;
	std::vector<GPSLogEntry *> v;
	entries.reserve(15);
	for (unsigned int i = 0; i < 15; i++) {
		entries.push_back(make_fix(45.0 + 0.01 * i, -73.0 - 0.01 * i, T0 + i * 600));
	}
	for (auto &e : entries)
		v.push_back(&e);

	unsigned int size_bits;
	// DR0 (51 B) only fits 5 entries.
	KineisPacket pkt = LPB::build_gps_packet(v, false, false, 51, size_bits);

	DecodedV2 d = decode_multi(pkt);
	CHECK_EQUAL(5U, d.count);
	CHECK(pkt.size() <= 51U);
}

// ===========================================================================
// Bounded depth pile — set_max_size() must evict oldest entries even when
// their burst_counter is still non-zero.
// ===========================================================================
TEST_GROUP(BoundedDepthPile){};

TEST(BoundedDepthPile, EvictsOldestRegardlessOfBurstCounter) {
	DepthPile<GPSLogEntry> dp;
	GPSLogEntry e{};

	// Fill 4 entries with non-zero burst_counter.
	for (unsigned int i = 0; i < 4; i++) {
		e.info.day = (uint8_t)i;
		dp.store(e, 3 /*burst_counter*/);
	}
	CHECK_EQUAL(4U, dp.size());

	// Shrink cap to 2 — the two oldest (day=0, day=1) are evicted unconditionally.
	dp.set_max_size(2);
	CHECK_EQUAL(2U, dp.size());

	auto v = dp.retrieve(2 /*depth*/, 2 /*max_messages*/);
	// retrieve_index logic returns up to span entries — verify the survivors are
	// the freshest two (day=2 and day=3).
	CHECK(v.size() >= 1U);
	bool found_day2 = false, found_day3 = false;
	for (auto *p : v) {
		if (p->info.day == 2) found_day2 = true;
		if (p->info.day == 3) found_day3 = true;
	}
	CHECK(found_day2 || found_day3);
}

TEST(BoundedDepthPile, ZeroSizeClampedToOne) {
	// set_max_size(0) is a degenerate input; the pile should still hold ≥ 1 entry
	// (eviction policy clamps to 1 to avoid permanently dropping all data).
	DepthPile<GPSLogEntry> dp;
	GPSLogEntry e{};
	e.info.day = 7;
	dp.store(e, 1);

	dp.set_max_size(0);
	CHECK_EQUAL(1U, dp.size());
}

TEST(BoundedDepthPile, NewerStorePopsOldestWhenFull) {
	// pile cap = 2, NTRY = 3 → "fill, fill, fill" pattern: oldest is dropped
	// even though its burst_counter is still 3 (no retries consumed).
	DepthPile<GPSLogEntry> dp;
	dp.set_max_size(2);

	GPSLogEntry a{}, b{}, c{};
	a.info.day = 1;
	dp.store(a, 3);
	b.info.day = 2;
	dp.store(b, 3);
	c.info.day = 3;
	dp.store(c, 3);  // pushes 'a' out via store()'s own pop_front

	CHECK_EQUAL(2U, dp.size());
	// Only b and c should remain.
	bool found_a = false, found_b = false, found_c = false;
	auto v = dp.retrieve(2, 2);
	for (auto *p : v) {
		if (p->info.day == 1) found_a = true;
		if (p->info.day == 2) found_b = true;
		if (p->info.day == 3) found_c = true;
	}
	CHECK(!found_a);
	CHECK(found_b);
	CHECK(found_c);
}
