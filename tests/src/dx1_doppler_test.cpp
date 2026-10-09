/**
 * @file dx1_doppler_test.cpp
 * @brief DX1 turtle Doppler extension: codecs, frames, behaviour tracker, and
 *        the ArgosTx prewarm / standard paths.
 *
 * Every expected byte string below comes from the DX1 v3 reference model
 * (tools/dx1/dx1_ref.py, vectors tools/dx1/dx1_vectors_v3.json). They are the wire contract the
 * ground decoder is written against: a mismatch here is a beacon the platform
 * will misread, not a test to "update".
 */

#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

#include "mock_kineis_device.hpp"
#include "fake_rtc.hpp"
#include "fake_config_store.hpp"
#include "fake_timer.hpp"
#include "fake_battery_mon.hpp"
#include "binascii.hpp"
#include "scheduler.hpp"
#include "argos_tx_service.hpp"
#include "argos_packet_builder.hpp"
#include "dive_behaviour_tracker.hpp"
#include "sws_analog_service.hpp"
#include "crc8.hpp"

#include <string>
#include <vector>

extern Timer *system_timer;
extern ConfigurationStore *configuration_store;
extern Scheduler *system_scheduler;
extern RTC *rtc;
extern BatteryMonitor *battery_monitor;

using APB = ArgosPacketBuilder;

namespace {

// ------------------------------------------------------------ reference decode
uint32_t em_decode(uint32_t c, unsigned int e_bits, unsigned int m_bits) {
	(void)e_bits;
	uint32_t base = 1U << m_bits;
	uint32_t e = c >> m_bits, m = c & (base - 1U);
	return e == 0 ? m : (base + m) << (e - 1U);
}

std::string hex(const std::string &s) {
	return Binascii::hexlify(s);
}

std::string view_hex(const KineisPacket &img, KineisModulation mode, unsigned int expect_bits) {
	KineisPacket v = img;
	unsigned int bits = APB::doppler_view(v, mode, APB::DOPPLER_DX1_IMAGE_BITS, true);
	CHECK_EQUAL(expect_bits, bits);
	CHECK_EQUAL(expect_bits / 8, (unsigned int)v.size());
	return hex(v);
}

void check_crc(const KineisPacket &p) {
	CHECK_TRUE(p.size() >= 4);
	CHECK_EQUAL((unsigned int)CRC8::checksum(p, 24), (unsigned int)(uint8_t)p[3]);
}

// The vectors' scenario health block: delay 1000 us, contrast 15.0x, floor 200,
// ceiling 10000, counters 0 (the model passes the counters to DEND only).
void scenario_health(APB::DopplerExt &ext) {
	APB::dx1_encode_health(ext, 1000, 150, true, 1, 0, 0, 0, 200, 10000, false);
}

struct Counters {
	uint32_t dto = 0;
	uint32_t fsc = 0;
	bool adcf = false;
};

/// Mirror of dx1_ref.build(): latch + DEND from the tracker, fixed health, RST 0.
std::string build_view(uint64_t now, unsigned int mv, bool lb, Counters c, KineisModulation mode) {
	APB::DopplerExt ext;
	APB::dx1_encode_behaviour(ext, DiveBehaviourTracker::latch(now));
	ext.dive_end = (uint8_t)DiveBehaviourTracker::dive_end(c.dto, c.fsc, c.adcf);
	ext.rst = 0;
	scenario_health(ext);
	unsigned int bits = 0;
	KineisPacket img = APB::build_doppler_packet(mv, lb, bits, &ext);
	CHECK_EQUAL(128U, bits);
	bits = APB::doppler_view(img, mode, bits, true);
	return hex(img);
}

}  // namespace

// =============================================================== codecs
TEST_GROUP(Dx1Codec){};

TEST(Dx1Codec, Crc8MatchesStandardCheckValueAndHeadHelper) {
	std::string s = "123456789";
	CHECK_EQUAL(244U, (unsigned int)CRC8::checksum(s, 72));
	// The non-allocating head helper is the same CRC as CRC8::checksum(x, 24).
	for (unsigned int a = 0; a < 256; a += 7) {
		for (unsigned int b = 0; b < 256; b += 13) {
			std::string x = { (char)0x00, (char)a, (char)b, (char)0x00 };
			CHECK_EQUAL((unsigned int)CRC8::checksum(x, 24), (unsigned int)APB::dx1_crc8_head(x));
		}
	}
}

TEST(Dx1Codec, E4M3E3M3Q4PctDryMatchReference) {
	static const uint32_t kE4M3[][3] = { { 1, 1, 1 },
	                                     { 7, 7, 7 },
	                                     { 8, 8, 8 },
	                                     { 15, 15, 15 },
	                                     { 16, 16, 16 },
	                                     { 60, 31, 60 },
	                                     { 90, 35, 88 },
	                                     { 1200, 65, 1152 },
	                                     { 3480, 77, 3328 },
	                                     { 21600, 98, 20480 },
	                                     { 245760, 127, 245760 },
	                                     { 300000, 127, 245760 } };
	static const uint32_t kE3M3[][3] = { { 4, 4, 4 },      { 20, 18, 20 },   { 200, 44, 192 },
	                                     { 600, 57, 576 }, { 960, 63, 960 }, { 2000, 63, 960 } };
	static const uint32_t kQ4[][2] = { { 0, 0 },     { 7, 7 },     { 8, 8 },     { 15, 8 },
	                                   { 16, 9 },    { 255, 12 },  { 256, 13 },  { 1023, 13 },
	                                   { 1024, 14 }, { 4095, 14 }, { 4096, 15 }, { 65535, 15 } };
	static const uint32_t kPct[][3] = { { 5789, 86400, 8 }, { 2600, 86400, 4 }, { 86400, 86400, 101 },
	                                    { 0, 86400, 1 },    { 100, 43199, 0 },  { 21600, 43200, 51 } };
	static const uint32_t kDry[][3] = { { 0, 86400, 1 },     { 1727, 86400, 1 },  { 1728, 86400, 2 },
	                                    { 3456, 86400, 3 },  { 5789, 86400, 3 },  { 13824, 86400, 5 },
	                                    { 27648, 86400, 6 }, { 55296, 86400, 7 }, { 86400, 86400, 7 },
	                                    { 100, 43199, 0 } };
	for (const auto &r : kE4M3) {
		CHECK_EQUAL(r[1], APB::dx1_em_encode(r[0], 4, 3));
		CHECK_EQUAL(r[2], em_decode(r[1], 4, 3));
	}
	for (const auto &r : kE3M3) {
		CHECK_EQUAL(r[1], APB::dx1_em_encode(r[0], 3, 3));
		CHECK_EQUAL(r[2], em_decode(r[1], 3, 3));
	}
	for (const auto &r : kQ4)
		CHECK_EQUAL(r[1], APB::dx1_q4_encode(r[0]));
	for (const auto &r : kPct)
		CHECK_EQUAL(r[2], APB::dx1_pct_code(r[0], r[1]));
	for (const auto &r : kDry)
		CHECK_EQUAL(r[2], APB::dx1_dry_class(r[0], r[1]));
}

TEST(Dx1Codec, LegacyFrameIsBitExactAndNeverViewed) {
	unsigned int bits = 0;
	KineisPacket p = APB::build_doppler_packet(3700, false, bits);
	CHECK_EQUAL(24U, bits);
	STRCMP_EQUAL("006400", hex(p).c_str());
	p = APB::build_doppler_packet(2800, true, bits);
	CHECK_EQUAL(24U, bits);
	STRCMP_EQUAL("000B00", hex(p).c_str());
	// A non-DX1 packet goes through doppler_view untouched on every modulation.
	for (KineisModulation m : { KineisModulation::VLDA4, KineisModulation::LDA2, KineisModulation::LDK }) {
		KineisPacket q = APB::build_doppler_packet(3700, false, bits);
		CHECK_EQUAL(24U, APB::doppler_view(q, m, bits, false));
		STRCMP_EQUAL("006400", hex(q).c_str());
	}
}

// =============================================================== frames
TEST_GROUP(Dx1Frame){};

namespace {
struct FrameVec {
	const char *name;
	unsigned int mv;
	bool lb;
	uint8_t dry, dend;
	bool rich;
	APB::DopplerExt rich_codes;  // already-encoded rich codes
	const char *vlda4, *lda2, *ldk;
};

APB::DopplerExt codes(uint8_t latch, uint8_t blind, uint8_t partial, uint16_t seq, uint8_t rst, uint8_t pct, uint8_t n,
                      uint8_t s_last, uint8_t d_prev, uint8_t d_max, uint8_t delay, uint8_t contrast, uint8_t dto,
                      uint8_t fsc, uint8_t src, uint8_t dfloor, uint8_t adcf, uint8_t method) {
	APB::DopplerExt e;
	e.latch = latch;
	e.blind = blind;
	e.partial = partial;
	e.surf_seq = seq;
	e.rst = rst;
	e.pct = pct;
	e.n_surf = n;
	e.s_last = s_last;
	e.d_prev = d_prev;
	e.d_max = d_max;
	e.delay = delay;
	e.contrast = contrast;
	e.dto = dto;
	e.fsc = fsc;
	e.src = src;
	e.dfloor = dfloor;
	e.adcf = adcf;
	e.surf_method = method;
	return e;
}
}  // namespace

TEST(Dx1Frame, EveryReferenceFrameOnEveryView) {
	const FrameVec frames[] = {
		{ "compact_3700_d3_e1",
		  3700,
		  false,
		  3,
		  1,
		  false,
		  {},
		  "00644D",
		  "00644D45",
		  "00644D45000000000000000000000000" },
		{ "compact_unknown", 3700, false, 0, 0, false, {}, "006440", "00644066", "00644066000000000000000000000000" },
		{ "compact_3600_d2_e3",
		  3600,
		  false,
		  2,
		  3,
		  false,
		  {},
		  "005A4B",
		  "005A4B78",
		  "005A4B78000000000000000000000000" },
		{ "compact_2800lb_d7_e0",
		  2800,
		  true,
		  7,
		  0,
		  false,
		  {},
		  "000B5C",
		  "000B5C04",
		  "000B5C04000000000000000000000000" },
		{ "rich_nominal", 3700, false, 3, 1, true, codes(1, 0, 0, 59, 0, 8, 31, 35, 65, 77, 18, 41, 0, 0, 0, 0, 0, 2),
		  "00644D", "00644D45", "00644D4541D8107D1C19A95200080000" },
		{ "rich_forced", 3600, false, 2, 3, true, codes(1, 0, 0, 300, 2, 4, 18, 33, 98, 98, 18, 39, 6, 2, 1, 0, 0, 6),
		  "005A4B", "005A4B78", "005A4B78496208490E2C494EC4380000" },
		{ "rich_surface_lock_adcf", 2800, true, 7, 3, true,
		  codes(1, 0, 0, 511, 7, 101, 1, 0, 0, 0, 4, 24, 0, 0, 8, 1, 1, 1), "000B5F", "000B5F0D",
		  "000B5F0D4FFFCA040000023001070000" },
	};
	for (const auto &f : frames) {
		APB::DopplerExt ext = f.rich ? f.rich_codes : APB::DopplerExt{};
		ext.dry_class = f.dry;
		ext.dive_end = f.dend;
		unsigned int bits = 0;
		KineisPacket img = APB::build_doppler_packet(f.mv, f.lb, bits, &ext);
		CHECK_EQUAL(128U, bits);
		STRCMP_EQUAL_TEXT(f.vlda4, view_hex(img, KineisModulation::VLDA4, 24).c_str(), f.name);
		STRCMP_EQUAL_TEXT(f.lda2, view_hex(img, KineisModulation::LDA2, 32).c_str(), f.name);
		STRCMP_EQUAL_TEXT(f.ldk, view_hex(img, KineisModulation::LDK, 128).c_str(), f.name);
	}
}

// The rich frames again, this time from RAW values through the real encoders.
TEST(Dx1Frame, RichFramesFromRawValues) {
	struct Raw {
		unsigned int mv;
		bool lb;
		uint8_t dend, rst;
		Dx1Behaviour b;
		uint32_t delay_us;
		uint16_t contrast_x10;
		uint32_t dto, fsc, src;
		bool adcf;
		const char *ldk;
	};
	auto beh = [](uint16_t seq, uint32_t dry, uint32_t obs, uint32_t n, uint32_t s_last, uint32_t d_prev,
	              uint32_t d_max, uint8_t method) {
		Dx1Behaviour b;
		b.latch = 1;
		b.surf_seq = seq;
		b.dry_s = dry;
		b.obs_s = obs;
		b.n_known = true;
		b.n_surf = n;
		b.s_last_s = s_last;
		b.d_prev_s = d_prev;
		b.d_max_s = d_max;
		b.surf_method = method;
		return b;
	};
	const Raw raws[] = {
		{ 3700, false, 1, 0, beh(59, 5789, 86400, 59, 90, 1200, 3480, 2), 1000, 150, 0, 0, 0, false,
		  "00644D4541D8107D1C19A95200080000" },
		{ 3600, false, 3, 2, beh(300, 2600, 86400, 20, 75, 21600, 21600, 6), 1000, 120, 6, 2, 1, false,
		  "005A4B78496208490E2C494EC4380000" },
		{ 2800, true, 3, 7, beh(511, 86400, 86400, 0, 0, 0, 0, 1), 200, 32, 0, 0, 9, true,
		  "000B5F0D4FFFCA040000023001070000" },
	};
	for (const auto &r : raws) {
		APB::DopplerExt ext;
		APB::dx1_encode_behaviour(ext, r.b);
		ext.dive_end = r.dend;
		ext.rst = r.rst;
		APB::dx1_encode_health(ext, r.delay_us, r.contrast_x10, true, 1, r.dto, r.fsc, r.src, 200, 10000, r.adcf);
		unsigned int bits = 0;
		KineisPacket img = APB::build_doppler_packet(r.mv, r.lb, bits, &ext);
		STRCMP_EQUAL(r.ldk, hex(img).c_str());
	}
}

TEST(Dx1Frame, ByteInvariantsExhaustive) {
	for (unsigned int mv = 2700; mv <= 5240; mv += 20) {
		for (int lb = 0; lb < 2; lb++) {
			unsigned int legacy_bits;
			KineisPacket legacy = APB::build_doppler_packet(mv, lb != 0, legacy_bits);
			for (uint8_t dry = 0; dry < 8; dry++) {
				for (uint8_t dend = 0; dend < 4; dend++) {
					APB::DopplerExt ext;
					ext.dry_class = dry;
					ext.dive_end = dend;
					unsigned int bits;
					KineisPacket img = APB::build_doppler_packet(mv, lb != 0, bits, &ext);
					CHECK_EQUAL(0, img[0]);
					CHECK_EQUAL(legacy[1], img[1]);
					CHECK_EQUAL(1U, ((uint8_t)img[2]) >> 6);
					check_crc(img);
					CHECK_EQUAL(0, img[14]);
					CHECK_EQUAL(0, img[15]);
				}
			}
		}
	}
}

TEST(Dx1Frame, RefreshDiveEndRewritesTwoBitsAndCrcOnly) {
	APB::DopplerExt ext;
	ext.dry_class = 3;
	ext.dive_end = 1;
	unsigned int bits;
	KineisPacket img = APB::build_doppler_packet(3600, false, bits, &ext);
	KineisPacket before = img;
	APB::dx1_refresh_dive_end(img, 3);
	STRCMP_EQUAL("005A4F", hex(img.substr(0, 3)).c_str());
	check_crc(img);
	CHECK_TRUE(img.substr(4) == before.substr(4));
	// A 3-byte view has no CRC byte to refresh.
	KineisPacket v = before;
	APB::doppler_view(v, KineisModulation::VLDA4, bits, true);
	APB::dx1_refresh_dive_end(v, 2);
	STRCMP_EQUAL("005A4E", hex(v).c_str());
}

// =============================================================== tracker
TEST_GROUP(Dx1Tracker) {
	static constexpr uint64_t T0 = 7000000ULL;  // tracker reset at a non-zero uptime
	static constexpr uint64_t S = 1000ULL;

	void setup() {
		DiveBehaviourTracker::test_simulate_boot(true);
		DiveBehaviourTracker::reset(T0);
	}
	void teardown() {
		SWSAnalogService::reset_noinit_data();
		DiveBehaviourTracker::reset(0);
	}
	static void uw(bool state, uint64_t t, Counters c = {}, uint8_t level = 0) {
		DiveBehaviourTracker::on_uw_event(state, T0 + t, c.dto, c.fsc, c.adcf, level);
	}
	static std::string ldk(uint64_t t, unsigned int mv = 3700, Counters c = {}) {
		return build_view(T0 + t, mv, false, c, KineisModulation::LDK);
	}
};

TEST(Dx1Tracker, ScenarioCycles20min90s) {
	uint64_t t = 0;
	uw(true, t);  // boot re-emission underwater: not an edge
	STRCMP_EQUAL("00644161900000000000095200000000", ldk(t).c_str());
	unsigned int dives = 0;
	while (t < 30ULL * 3600ULL * S) {
		t += 1200 * S;
		uw(false, t, {}, 2);
		if (dives == 0) STRCMP_EQUAL("00644161900000000000095200000000", ldk(t).c_str());
		if (dives == 1) STRCMP_EQUAL("00644161500800011800095200080000", ldk(t).c_str());
		t += 90 * S;
		uw(true, t);
		dives++;
		if (dives == 1) STRCMP_EQUAL("00644161500800011800095200080000", ldk(t).c_str());
		if (dives == 2) STRCMP_EQUAL("00644161501000011C10095200080000", ldk(t).c_str());
	}
	CHECK_EQUAL(108360000ULL, t);
	STRCMP_EQUAL("00644D4542A010811C18295200080000", ldk(t).c_str());
	STRCMP_EQUAL("00644D45", build_view(T0 + t, 3700, false, {}, KineisModulation::LDA2).c_str());
}

TEST(Dx1Tracker, ScenarioForcedSurface) {
	uint64_t t = 0;
	uw(false, t);  // boot at the surface: no edge
	t += 60 * S;
	uw(true, t);  // surface not known -> dive start not real
	t += 1000 * S;
	uw(false, t, {}, 1);  // real surface edge
	CHECK_EQUAL(1U, DiveBehaviourTracker::dive_end(0, 0, false));
	t += 120 * S;
	uw(true, t);
	CHECK_EQUAL(1180000ULL, t);
	STRCMP_EQUAL("00644161500800013800095200040000", ldk(t).c_str());
	CHECK_EQUAL(2U, DiveBehaviourTracker::dive_end(1, 0, false));  // in-progress dive after a timeout
	t += 21600 * S;
	uw(false, t, { 3, 1, false }, 0);  // forced surface (escalation)
	CHECK_EQUAL(3U, DiveBehaviourTracker::dive_end(3, 1, false));
	STRCMP_EQUAL("005A4340", build_view(T0 + t, 3600, false, { 3, 1, false }, KineisModulation::LDA2).c_str());
	t += 30 * S;
	uw(true, t, { 3, 1, false });  // SWS re-detects water: closes a forced episode
	STRCMP_EQUAL("005A414E500800000620095200180000", ldk(t, 3600, { 3, 1, false }).c_str());
	t += 600 * S;
	uw(false, t, { 3, 1, false }, 3);
	t += 60 * S;
	uw(true, t, { 3, 1, false });  // D_PREV unknown (start followed a forced episode)
	CHECK_EQUAL(23470000ULL, t);
	STRCMP_EQUAL("005A414E50100000F8000952000C0000", ldk(t, 3600, { 3, 1, false }).c_str());
}

TEST(Dx1Tracker, ScenarioCooldownBlind) {
	uint64_t t = 0;
	uw(true, t);
	t += 1200 * S;
	uw(false, t, {}, 2);
	t += 90 * S;
	uw(true, t);
	t += 1200 * S;
	uw(false, t, {}, 2);
	t += 90 * S;
	uw(true, t);
	STRCMP_EQUAL("00644161501000011C10095200080000", ldk(t).c_str());
	DiveBehaviourTracker::on_blind_start(T0 + t + 10 * S);
	uw(false, t + 500 * S, {}, 2);  // ignored while blind
	DiveBehaviourTracker::on_blind_end(T0 + t + 2710 * S);
	t += 2710 * S;
	uw(true, t);  // re-emission after blind: state set, not an edge
	t += 1200 * S;
	uw(false, t, {}, 4);
	t += 90 * S;
	uw(true, t);  // D_PREV unknown, S_LAST 90, BLIND 1
	CHECK_EQUAL(6580000ULL, t);
	STRCMP_EQUAL("00644161701800011800095200100000", ldk(t).c_str());
	DiveBehaviourTracker::on_blind_end(T0 + t + 5);  // no-op without start
	STRCMP_EQUAL("00644161701800011800095200100000", ldk(t).c_str());
}

// Same scenario as the reference cycles, but fed through the real funnel
// (ServiceManager::notify_peer_event) with the uptime clock and the SWS
// status/diagnostics the funnel reads.
TEST(Dx1Tracker, FunnelDrivesTheTrackerLikeTheModel) {
	ServiceManager::reset();
	FakeTimer timer;
	system_timer = &timer;
	timer.set_counter(T0);
	DiveBehaviourTracker::reset();  // reset() reads the uptime clock
	auto event = [&timer](bool state, uint64_t t, uint8_t level) {
		SWSAnalogService::Status st{};
		st.surface_level = level;
		SWSAnalogService::test_set_status(st);
		timer.set_counter(T0 + t);
		ServiceEvent e;
		e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
		e.event_data = state;
		e.event_source = ServiceIdentifier::UW_SENSOR;
		e.event_originator_unique_id = 0x12345678;
		ServiceManager::notify_peer_event(e);
	};
	uint64_t t = 0;
	event(true, t, 0);
	for (int i = 0; i < 2; i++) {
		t += 1200 * S;
		event(false, t, 2);
		t += 90 * S;
		event(true, t, 0);
	}
	STRCMP_EQUAL("00644161501000011C10095200080000", ldk(t).c_str());
	system_timer = nullptr;
}

TEST(Dx1Tracker, TestModeEventsAreIgnoredAndFlagTheWindow) {
	ServiceManager::reset();
	auto event = [](bool state) {
		ServiceEvent e;
		e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
		e.event_data = state;
		e.event_source = ServiceIdentifier::UW_SENSOR;
		e.event_originator_unique_id = 0x12345678;
		ServiceManager::notify_peer_event(e);
	};
	FakeTimer timer;
	system_timer = &timer;
	timer.set_counter(T0);
	DiveBehaviourTracker::reset();
	uint64_t t = 0;
	event(true);
	t += 1200 * S;
	timer.set_counter(T0 + t);
	event(false);
	t += 90 * S;
	timer.set_counter(T0 + t);
	event(true);  // first real dive edge: SEQ 1
	const Dx1Behaviour latched = DiveBehaviourTracker::latch(T0 + t);
	CHECK_EQUAL(1U, latched.surf_seq);

	SWSAnalogService::start_test_mode();  // no instance: only the flag
	t += 100 * S;
	timer.set_counter(T0 + t);
	event(false);
	t += 100 * S;
	timer.set_counter(T0 + t);
	event(true);
	CHECK_TRUE(DiveBehaviourTracker::State::UNKNOWN == DiveBehaviourTracker::test_state());
	const Dx1Behaviour during = DiveBehaviourTracker::latch(T0 + t);  // no edge was counted
	CHECK_EQUAL(latched.surf_seq, during.surf_seq);
	CHECK_EQUAL(latched.s_last_s, during.s_last_s);
	CHECK_EQUAL(latched.obs_s, during.obs_s);
	CHECK_EQUAL(0U, DiveBehaviourTracker::dive_end(0, 0, false));  // state lost: DEND unknown
	SWSAnalogService::reset_noinit_data();                         // test mode off

	t += 100 * S;
	timer.set_counter(T0 + t);
	event(true);  // first event after the test: a state, not an edge
	CHECK_EQUAL(1U, DiveBehaviourTracker::latch(T0 + t).surf_seq);
	t += 1200 * S;
	timer.set_counter(T0 + t);
	event(false);
	t += 90 * S;
	timer.set_counter(T0 + t);
	event(true);
	Dx1Behaviour b = DiveBehaviourTracker::latch(T0 + t);
	CHECK_EQUAL(2U, b.surf_seq);
	CHECK_EQUAL(1U, b.blind);     // the test window is flagged
	CHECK_EQUAL(0U, b.d_prev_s);  // the dive spanning the test has no known start
	system_timer = nullptr;
}

TEST(Dx1Tracker, SurfaceEdgeIsStoreOnly) {
	uw(true, 0);
	uw(false, 1200 * S, {}, 2);
	uw(true, 1290 * S);
	const std::string latched = ldk(1290 * S);
	Dx1Behaviour before = DiveBehaviourTracker::latch(T0 + 1290 * S);
	uw(false, 2490 * S, { 0, 0, false }, 5);
	Dx1Behaviour after = DiveBehaviourTracker::latch(T0 + 2490 * S);
	CHECK_EQUAL(before.surf_seq, after.surf_seq);
	CHECK_EQUAL(before.obs_s, after.obs_s);
	CHECK_EQUAL(before.s_last_s, after.s_last_s);
	STRCMP_EQUAL(latched.c_str(), ldk(2490 * S).c_str());
}

TEST(Dx1Tracker, DiveEndCases) {
	uw(true, 0, { 5, 2, false });
	CHECK_EQUAL(1U, DiveBehaviourTracker::dive_end(5, 2, false));
	CHECK_EQUAL(2U, DiveBehaviourTracker::dive_end(6, 2, false));
	CHECK_EQUAL(3U, DiveBehaviourTracker::dive_end(9, 3, false));
	CHECK_EQUAL(3U, DiveBehaviourTracker::dive_end(5, 2, true));
	CHECK_EQUAL(0U, DiveBehaviourTracker::dive_end(4, 2, false));  // counter went down: cold boot / clear
	uw(false, 600 * S, { 6, 2, false }, 1);
	CHECK_EQUAL(2U, DiveBehaviourTracker::dive_end(99, 99, true));  // SURF: the surface-edge values rule
}

TEST(Dx1Tracker, RstCountsWarmBootsSaturatingAtSeven) {
	DiveBehaviourTracker::test_simulate_boot(true);
	DiveBehaviourTracker::reset(0);
	CHECK_EQUAL(0U, DiveBehaviourTracker::rst_count());
	DiveBehaviourTracker::reset(0);  // a second reset in the same boot changes nothing
	CHECK_EQUAL(0U, DiveBehaviourTracker::rst_count());
	for (unsigned int i = 1; i <= 9; i++) {
		DiveBehaviourTracker::test_simulate_boot(false);
		DiveBehaviourTracker::reset(0);
		CHECK_EQUAL(i < 7 ? i : 7U, DiveBehaviourTracker::rst_count());
	}
	DiveBehaviourTracker::test_simulate_boot(true);
	DiveBehaviourTracker::reset(0);
	CHECK_EQUAL(0U, DiveBehaviourTracker::rst_count());
}

// =============================================================== ArgosTx
TEST_GROUP(Dx1ArgosTx) {
	FakeBatteryMonitor *fake_battery_monitor;
	FakeConfigurationStore *fake_config_store;
	MockKineisDevice *mock_kineis;
	FakeRTC *fake_rtc;
	FakeTimer *fake_timer;
	uint64_t t = 1652105502000ULL;

	void setup() {
		ServiceManager::reset();
		SWSAnalogService::reset_noinit_data();
		fake_battery_monitor = new FakeBatteryMonitor;
		battery_monitor = fake_battery_monitor;
		fake_battery_monitor->set_values(100, 3700);
		mock_kineis = new MockKineisDevice;
		fake_config_store = new FakeConfigurationStore;
		configuration_store = fake_config_store;
		configuration_store->init();
		fake_rtc = new FakeRTC;
		rtc = fake_rtc;
		fake_timer = new FakeTimer;
		system_timer = fake_timer;
		system_scheduler = new Scheduler(system_timer);
		fake_timer->start();
		set_time(t);
		fake_config_store->write_param(ParamID::PP_MIN_ELEVATION, 5.0);
		fake_config_store->write_param(ParamID::MIN_SURFACE_CYCLE_INTERVAL_S, 0U);
		fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::SURFACING_BURST);
		fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);
		fake_config_store->write_param(ParamID::ARGOS_HEXID, (unsigned int)0x01234567U);
		fake_config_store->write_param(ParamID::GNSS_EN, true);
		fake_config_store->write_param(ParamID::LB_EN, false);
		fake_config_store->write_param(ParamID::TR_NOM, (unsigned int)60);
		fake_config_store->write_param(ParamID::ARGOS_TX_JITTER_EN, false);
		fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, false);
		fake_config_store->write_param(ParamID::UNDERWATER_EN, (bool)true);
		fake_config_store->write_param(ParamID::DRY_TIME_BEFORE_TX, (unsigned int)0);
		fake_config_store->write_param(ParamID::SURFACING_BURST_INIT_S, (unsigned int)5);
		fake_config_store->write_param(ParamID::SURFACING_BURST_STEP_S, (unsigned int)10);
		fake_config_store->write_param(ParamID::SURFACING_BURST_MAX_S, (unsigned int)60);
		DiveBehaviourTracker::test_simulate_boot(true);
		DiveBehaviourTracker::reset();
		mock().ignoreOtherCalls();
	}

	void teardown() {
		delete system_scheduler;
		delete fake_timer;
		delete fake_rtc;
		delete fake_config_store;
		delete mock_kineis;
		delete fake_battery_monitor;
		system_scheduler = nullptr;
		system_timer = nullptr;
		rtc = nullptr;
		configuration_store = nullptr;
		battery_monitor = nullptr;
		SWSAnalogService::reset_noinit_data();
		DiveBehaviourTracker::reset(0);
		ServiceManager::reset();
	}

	void set_time(uint64_t ms) {
		t = ms;
		fake_rtc->settime((std::time_t)(ms / 1000));
		fake_timer->set_counter(ms);
	}

	void uw(bool state) {
		ServiceEvent e;
		e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
		e.event_data = state;
		e.event_source = ServiceIdentifier::UW_SENSOR;
		e.event_originator_unique_id = 0x12345678;
		ServiceManager::notify_peer_event(e);
	}

	void gate(bool on) {
		fake_config_store->write_param(ParamID::ARGOS_DOPPLER_EXT_EN, on);
	}

	/// Dive, stay down @p down_s, surface, run the scheduler: the first send
	/// is the prewarm path.
	void dive_and_surface(unsigned int down_s = 1200) {
		uw(true);
		set_time(t + down_s * 1000ULL);
		uw(false);
		system_scheduler->run();
	}

	/// Complete the in-flight TX and run the next one (standard path).
	void next_ping() {
		mock_kineis->notify(KineisEventTxComplete({}));
		set_time(t + 60000ULL);
		system_scheduler->run();
	}
};

TEST(Dx1ArgosTx, GateOffIsLegacyOnEveryModulation) {
	for (KineisModulation m : { KineisModulation::LDA2, KineisModulation::LDK, KineisModulation::VLDA4 }) {
		for (int variant = 0; variant < 2; variant++) {
			// variant 0: gate off; variant 1: gate on but UNDERWATER_EN off.
			gate(variant == 1);
			fake_config_store->write_param(ParamID::UNDERWATER_EN, (bool)(variant == 0));
			mock_kineis->test_set_current_modulation(m);
			ArgosTxService serv(*mock_kineis);
			serv.start();
			mock_kineis->send_count = 0;
			dive_and_surface();
			CHECK_EQUAL(1U, mock_kineis->send_count);
			CHECK_EQUAL(24U, mock_kineis->last_size_bits);
			STRCMP_EQUAL("006400", hex(mock_kineis->last_packet).c_str());
			next_ping();
			CHECK_EQUAL(2U, mock_kineis->send_count);
			CHECK_EQUAL(24U, mock_kineis->last_size_bits);
			STRCMP_EQUAL("006400", hex(mock_kineis->last_packet).c_str());
			serv.stop();
		}
	}
}

TEST(Dx1ArgosTx, PrewarmCarriesTheViewOfTheModulation) {
	struct {
		KineisModulation m;
		unsigned int bits;
		const char *hex;
	} cases[] = {
		{ KineisModulation::LDA2, 32, "00644161" },
		{ KineisModulation::LDK, 128, "00644161900000000000000000000000" },
		{ KineisModulation::VLDA4, 24, "006441" },
	};
	gate(true);
	for (const auto &c : cases) {
		DiveBehaviourTracker::reset();
		mock_kineis->test_set_current_modulation(c.m);
		ArgosTxService serv(*mock_kineis);
		serv.start();
		mock_kineis->send_count = 0;
		uw(true);
		set_time(t + 1200000ULL);
		const unsigned int updates = fake_battery_monitor->update_calls;
		uw(false);
		system_scheduler->run();
		CHECK_EQUAL(updates, fake_battery_monitor->update_calls);  // prewarm path, not a fresh build
		CHECK_EQUAL(1U, mock_kineis->send_count);
		CHECK_EQUAL(c.bits, mock_kineis->last_size_bits);
		STRCMP_EQUAL(c.hex, hex(mock_kineis->last_packet).c_str());
		if (c.bits >= 32) check_crc(mock_kineis->last_packet);
		serv.stop();
	}
}

TEST(Dx1ArgosTx, PrewarmRefreshesDiveEndAfterAForcedSurface) {
	gate(true);
	mock_kineis->test_set_current_modulation(KineisModulation::LDK);
	ArgosTxService serv(*mock_kineis);
	serv.start();
	uw(true);                  // image built at the dive: DEND 1, FSC 0
	set_time(t + 1800000ULL);  // inside the 1-h prewarm freshness window
	SWSAnalogService::Diagnostics d{};
	d.force_surface_count = 1;  // the escalation forces the surface...
	d.dive_timeout_count = 3;
	SWSAnalogService::test_set_diagnostics(d);
	uw(false);  // ...right before the surface event
	system_scheduler->run();
	CHECK_EQUAL(1U, mock_kineis->send_count);
	CHECK_EQUAL(128U, mock_kineis->last_size_bits);
	const KineisPacket p1 = mock_kineis->last_packet;
	CHECK_EQUAL(3U, ((uint8_t)p1[2]) & 0x03);  // DEND 3 already in ping 1
	check_crc(p1);
	// Health fields are those of the dive-time image: FSC still Q4(0) in ping 1.
	STRCMP_EQUAL("0064436F900000000000000000000000", hex(p1).c_str());

	// Ping 2 (standard path): same behaviour bits, health read now (DTO 3, FSC 1).
	next_ping();
	CHECK_EQUAL(2U, mock_kineis->send_count);
	const KineisPacket p2 = mock_kineis->last_packet;
	CHECK_EQUAL(128U, mock_kineis->last_size_bits);
	CHECK_EQUAL((unsigned int)(uint8_t)p1[2], (unsigned int)(uint8_t)p2[2]);
	CHECK_TRUE(p1.substr(4, 6) == p2.substr(4, 6));  // bits 32-79 (behaviour) unchanged
	check_crc(p2);
	unsigned int pos = 95;
	CHECK_EQUAL(3U, extract_bits(p2, pos, 4));      // DTO
	CHECK_EQUAL(1U, extract_bits(p2, pos + 4, 4));  // FSC
	serv.stop();
}

TEST(Dx1ArgosTx, NothingTouchesTheBatteryBetweenSurfaceAndFirstSend) {
	gate(true);
	mock_kineis->test_set_current_modulation(KineisModulation::LDA2);
	ArgosTxService serv(*mock_kineis);
	serv.start();
	uw(true);
	// The battery moves underwater with no peer event to notice it.
	fake_battery_monitor->set_values(100, 3600);
	set_time(t + 1200000ULL);
	const unsigned int updates = fake_battery_monitor->update_calls;
	uw(false);
	system_scheduler->run();
	CHECK_EQUAL(1U, mock_kineis->send_count);
	CHECK_EQUAL(updates, fake_battery_monitor->update_calls);         // no ADC, no prepare
	STRCMP_EQUAL("00644161", hex(mock_kineis->last_packet).c_str());  // dive-time battery byte
	serv.stop();
}

TEST(Dx1ArgosTx, UwSensorEventsOnlyRefreshFromAnUnderwaterSampleEnd) {
	gate(true);
	mock_kineis->test_set_current_modulation(KineisModulation::LDA2);
	ArgosTxService serv(*mock_kineis);
	serv.start();
	uw(true);
	fake_battery_monitor->set_values(100, 3600);  // inputs changed -> a qualifying event rebuilds
	unsigned int updates = fake_battery_monitor->update_calls;
	ServiceEvent e;
	e.event_source = ServiceIdentifier::UW_SENSOR;
	e.event_originator_unique_id = 0x12345678;
	e.event_type = ServiceEventType::SERVICE_ACTIVE;
	ServiceManager::notify_peer_event(e);
	CHECK_EQUAL(updates, fake_battery_monitor->update_calls);
	e.event_type = ServiceEventType::SERVICE_INACTIVE;  // status says surface: no rebuild
	ServiceManager::notify_peer_event(e);
	CHECK_EQUAL(updates, fake_battery_monitor->update_calls);
	SWSAnalogService::Status st{};
	st.is_underwater = true;
	SWSAnalogService::test_set_status(st);
	ServiceManager::notify_peer_event(e);  // end of an underwater sample: rebuild
	CHECK_EQUAL(updates + 1, fake_battery_monitor->update_calls);
	serv.stop();
}

TEST(Dx1ArgosTx, LatchedAdcFaultSampleEndDoesNotRefresh) {
	// An invalid ADC read returns before the status push, so the status keeps
	// the last valid "underwater" while the detector forces the surface: the
	// INACTIVE of the read that latches the fault is the surfacing one.
	SWSAnalogService sws;  // outlives serv; teardown clears s_instance
	gate(true);
	mock_kineis->test_set_current_modulation(KineisModulation::LDA2);
	ArgosTxService serv(*mock_kineis);
	serv.start();
	uw(true);
	SWSAnalogService::Status st{};
	st.is_underwater = true;  // stale: the last valid sample's
	SWSAnalogService::test_set_status(st);
	ServiceEvent e;
	e.event_source = ServiceIdentifier::UW_SENSOR;
	e.event_originator_unique_id = 0x12345678;
	e.event_type = ServiceEventType::SERVICE_INACTIVE;

	// Partial failure, below the latch: still underwater, refresh unchanged.
	SWSAnalogService::test_set_consecutive_invalid_adc(SWSAnalogService::test_max_consecutive_invalid_adc() - 1);
	CHECK_FALSE(SWSAnalogService::adc_fault_active());
	fake_battery_monitor->set_values(100, 3600);
	unsigned int updates = fake_battery_monitor->update_calls;
	ServiceManager::notify_peer_event(e);
	CHECK_EQUAL(updates + 1, fake_battery_monitor->update_calls);

	// The read that latches the fault forces the surface: no rebuild.
	SWSAnalogService::test_set_consecutive_invalid_adc(SWSAnalogService::test_max_consecutive_invalid_adc());
	CHECK_TRUE(SWSAnalogService::adc_fault_active());
	fake_battery_monitor->set_values(100, 3500);
	updates = fake_battery_monitor->update_calls;
	ServiceManager::notify_peer_event(e);
	CHECK_EQUAL(updates, fake_battery_monitor->update_calls);
	// ...nor on the hourly backstop.
	set_time(t + 3600000ULL);
	ServiceManager::notify_peer_event(e);
	CHECK_EQUAL(updates, fake_battery_monitor->update_calls);
	serv.stop();
}

TEST(Dx1ArgosTx, ModulationMismatchSendsTheViewTheModuleFrames) {
	gate(true);
	mock_kineis->test_set_current_modulation(KineisModulation::LDK);  // prepared on LDK
	ArgosTxService serv(*mock_kineis);
	serv.start();
	uw(true);
	set_time(t + 1200000ULL);
	mock_kineis->test_set_current_modulation(KineisModulation::LDA2);  // module now frames LDA2
	const unsigned int updates = fake_battery_monitor->update_calls;
	uw(false);
	system_scheduler->run();
	CHECK_EQUAL(updates, fake_battery_monitor->update_calls);  // prewarm path
	CHECK_EQUAL(1U, mock_kineis->send_count);
	CHECK_EQUAL(32U, mock_kineis->last_size_bits);
	CHECK_EQUAL(4U, (unsigned int)mock_kineis->last_packet.size());
	check_crc(mock_kineis->last_packet);
	serv.stop();
}
