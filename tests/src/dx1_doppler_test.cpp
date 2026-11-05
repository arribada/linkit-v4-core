/**
 * @file dx1_doppler_test.cpp
 * @brief DX1 turtle Doppler extension: codecs, frames, behaviour tracker, and
 *        the ArgosTx prewarm / standard paths.
 *
 * Every expected byte string below comes from the DX1 v3 reference model
 * (dx1_ref.py and its vectors, kept out of the repository). They are the wire contract the
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
#include "gps.hpp"

#include <algorithm>
#include <string>
#include <vector>

extern Timer *system_timer;
extern ConfigurationStore *configuration_store;
extern Scheduler *system_scheduler;
extern RTC *rtc;
extern BatteryMonitor *battery_monitor;
extern GPSDevice *gps_device;  // null unless a test installs a receiver
// mock_pmu.cpp: what PMU::get_die_temperature_q4() returns, and how often it ran.
extern bool g_pmu_die_temp_ok;
extern int32_t g_pmu_die_temp_q4;
extern unsigned int g_pmu_die_temp_reads;

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

#if DX1_DOPPLER_EXT_BUILD
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
#endif

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

// The tracker and the ArgosTx paths exist only in a DIVE_INFO_MESSAGE build; the
// codecs and the gate truth table below run in both.
#if DX1_DOPPLER_EXT_BUILD
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

#endif  // DX1_DOPPLER_EXT_BUILD

// =============================================================== dive frame
// DX1 dive frame (24-byte LDA2 frame, 2026-11). Every expected byte string is
// from the reference model (dx1_ref.py: dive_frames and the dive_* scenarios).
namespace {
bool dive_crcs_ok(const KineisPacket &p) {
	return p.size() == APB::DIVE_FRAME_BYTES && (uint8_t)p[3] == CRC8::checksum(p, 24)
	       && (uint8_t)p[23] == CRC8::checksum(p, 184);
}
}  // namespace

TEST_GROUP(Dx1DiveCodec){};

TEST(Dx1DiveCodec, BinAndTemperatureCodesMatchTheModel) {
	// tl_code: 0 unknown below 1 h observed, else 1 + nearest 2 % step.
	CHECK_EQUAL(1U, APB::dx1_bin_code(0, 7200));
	CHECK_EQUAL(51U, APB::dx1_bin_code(7200, 7200));
	CHECK_EQUAL(26U, APB::dx1_bin_code(3600, 7200));
	CHECK_EQUAL(1U, APB::dx1_bin_code(1, 3600));
	CHECK_EQUAL(0U, APB::dx1_bin_code(0, 3599));
	CHECK_EQUAL(2U, APB::dx1_bin_code(144, 7200));
	CHECK_EQUAL(2U, APB::dx1_bin_code(143, 7200));
	CHECK_EQUAL(0U, APB::dx1_bin_code(0, 0));
	// temp_code: floor(q4 / 2) + 21 in 1..127, 0 unknown.
	CHECK_EQUAL(1U, APB::dx1_temp_code(true, -40));    // -10.0 °C
	CHECK_EQUAL(1U, APB::dx1_temp_code(true, -41));    // below the range: clamped
	CHECK_EQUAL(21U, APB::dx1_temp_code(true, 0));     // 0.0 °C
	CHECK_EQUAL(71U, APB::dx1_temp_code(true, 100));   // 25.0 °C
	CHECK_EQUAL(20U, APB::dx1_temp_code(true, -1));    // -0.25 °C rounds DOWN to -0.5
	CHECK_EQUAL(19U, APB::dx1_temp_code(true, -3));    // -0.75 °C -> -1.0
	CHECK_EQUAL(127U, APB::dx1_temp_code(true, 212));  // 53.0 °C
	CHECK_EQUAL(127U, APB::dx1_temp_code(true, 300));
	CHECK_EQUAL(1U, APB::dx1_temp_code(true, INT32_MIN));
	CHECK_EQUAL(127U, APB::dx1_temp_code(true, INT32_MAX));
	CHECK_EQUAL(0U, APB::dx1_temp_code(false, 100));
}

TEST(Dx1DiveCodec, EveryReferenceFrame) {
	struct Vec {
		const char *name;
		unsigned int mv;
		bool lb;
		uint8_t dry, dend;
		uint16_t surf_seq;
		uint8_t dive[5], s_last, pct, n_surf, d_max, temp, bins[12], phase;
		const char *hex;
	} vecs[] = {
		{ "dive_empty_3700",
		  3700,
		  false,
		  3,
		  1,
		  0,
		  { 0, 0, 0, 0, 0 },
		  0,
		  0,
		  0,
		  0,
		  0,
		  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
		  0,
		  "00646DA50000000000000000000000000000000000000000" },
		{ "dive_nominal",
		  3700,
		  false,
		  3,
		  1,
		  59,
		  { 67, 66, 65, 70, 0 },
		  35,
		  8,
		  31,
		  77,
		  77,
		  { 4, 1, 51, 0, 26, 2, 1, 43, 1, 51, 2, 0 },
		  1,
		  "00646DA51DC385063004620FCD9A203980D040D60E610103" },
		{ "dive_saturated_lowbatt",
		  2800,
		  true,
		  7,
		  3,
		  511,
		  { 127, 98, 1, 16, 15 },
		  127,
		  101,
		  1,
		  127,
		  1,
		  { 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51 },
		  0,
		  "000B7FEDFFFFC40480FFF940FF039E79E79E79E79E79E683" },
	};
	for (const auto &v : vecs) {
		APB::DiveFrameExt e;
		e.dry_class = v.dry;
		e.dive_end = v.dend;
		e.surf_seq = v.surf_seq;
		for (unsigned int k = 0; k < 5; k++)
			e.dive[k] = v.dive[k];
		e.s_last = v.s_last;
		e.pct = v.pct;
		e.n_surf = v.n_surf;
		e.d_max = v.d_max;
		e.temp = v.temp;
		for (unsigned int k = 0; k < 12; k++)
			e.bins[k] = v.bins[k];
		e.phase = v.phase;
		unsigned int bits = 0;
		KineisPacket p = APB::build_dive_frame(v.mv, v.lb, e, bits);
		CHECK_EQUAL(APB::DIVE_FRAME_BITS, bits);
		STRCMP_EQUAL_TEXT(v.hex, hex(p).c_str(), v.name);
		CHECK_TRUE_TEXT(dive_crcs_ok(p), v.name);
		CHECK_EQUAL(0U, (unsigned int)(uint8_t)p[0]);  // Doppler marker: no GNSS frame has day 0
		CHECK_EQUAL(1U, extract_bits(p, 16, 2));       // VER
		CHECK_EQUAL(1U, extract_bits(p, 18, 1));       // EXT: dive frame
		CHECK_EQUAL((unsigned int)v.surf_seq, extract_bits(p, 32, 9));
		CHECK_EQUAL((unsigned int)v.temp, extract_bits(p, 104, 7));
		CHECK_EQUAL((unsigned int)v.bins[11], extract_bits(p, 111 + 66, 6));
		CHECK_EQUAL((unsigned int)v.phase, extract_bits(p, 183, 1));
	}
}

TEST(Dx1DiveCodec, EncodeDiveTakesTheLatchedCodesAndTheData) {
	Dx1Behaviour b;
	b.latch = 1;
	b.dry_s = 5789;
	b.obs_s = 86400;
	b.n_known = true;
	b.n_surf = 59;
	b.s_last_s = 90;
	b.d_max_s = 3480;
	Dx1DiveData d;
	d.surf_seq = 60;
	d.dive_s[0] = 1500;
	d.dive_s[1] = 0;  // unknown stays 0
	d.dive_s[4] = 21600;
	d.temp_valid = true;
	d.temp_q4 = 112;
	d.bin_dry_s[0] = 450;
	d.bin_obs_s[0] = 7200;
	d.bin_obs_s[1] = 3599;  // under 1 h observed: unknown
	d.phase = 1;
	APB::DiveFrameExt e;
	APB::dx1_encode_dive(e, b, d);
	CHECK_EQUAL(3U, (unsigned int)e.dry_class);
	CHECK_EQUAL(60U, (unsigned int)e.surf_seq);
	CHECK_EQUAL(APB::dx1_em_encode(1500, 4, 3), (unsigned int)e.dive[0]);
	CHECK_EQUAL(0U, (unsigned int)e.dive[1]);
	CHECK_EQUAL(APB::dx1_em_encode(21600, 4, 3), (unsigned int)e.dive[4]);
	CHECK_EQUAL(APB::dx1_em_encode(90, 4, 3), (unsigned int)e.s_last);
	CHECK_EQUAL(APB::dx1_pct_code(5789, 86400), (unsigned int)e.pct);
	CHECK_EQUAL(APB::dx1_em_encode(60, 4, 3), (unsigned int)e.n_surf);
	CHECK_EQUAL(APB::dx1_em_encode(3480, 4, 3), (unsigned int)e.d_max);
	CHECK_EQUAL(77U, (unsigned int)e.temp);
	CHECK_EQUAL(4U, (unsigned int)e.bins[0]);
	CHECK_EQUAL(0U, (unsigned int)e.bins[1]);
	CHECK_EQUAL(1U, (unsigned int)e.phase);
	CHECK_EQUAL(0U, (unsigned int)e.dive_end);  // the caller's
}

#if DX1_DOPPLER_EXT_BUILD
TEST_GROUP(Dx1DiveTracker) {
	static constexpr uint64_t T0 = 7000000ULL;  // tracker reset at a non-zero uptime
	static constexpr uint64_t S = 1000ULL;
	static constexpr int64_t UTC0 = 1780000123;  // not on a 2-h boundary

	void setup() {
		DiveBehaviourTracker::test_simulate_boot(true);
		DiveBehaviourTracker::reset(T0);
	}
	void teardown() {
		SWSAnalogService::reset_noinit_data();
		DiveBehaviourTracker::reset(0);
	}
	using UtcFn = int64_t (*)(uint64_t);
	static int64_t utc_plain(uint64_t t) {
		return UTC0 + (int64_t)(t / 1000ULL);
	}
	static int64_t utc_jump(uint64_t t) {
		return utc_plain(t) - (t >= 10ULL * 3600ULL * S ? 1800 : 0);
	}
	static int64_t utc_none(uint64_t) {
		return 0;
	}
	static void clk(uint64_t t, int64_t utc) {
		DiveBehaviourTracker::on_clock(T0 + t, utc);
	}
	static void uw(bool state, uint64_t t, Counters c = {}, uint8_t level = 0) {
		DiveBehaviourTracker::on_uw_event(state, T0 + t, c.dto, c.fsc, c.adcf, level);
	}
	/// Mirror of dx1_ref.dive_build(): what ArgosTxService::try_send_dive_frame() builds.
	static std::string dive_hex(uint64_t t, int64_t utc, unsigned int mv, Counters c = {}) {
		DiveBehaviourTracker::on_clock(T0 + t, utc);
		APB::DiveFrameExt e;
		APB::dx1_encode_dive(e, DiveBehaviourTracker::latch(T0 + t), DiveBehaviourTracker::dive_data(T0 + t));
		e.dive_end = (uint8_t)DiveBehaviourTracker::dive_end(c.dto, c.fsc, c.adcf);
		unsigned int bits = 0;
		KineisPacket p = APB::build_dive_frame(mv, false, e, bits);
		CHECK_EQUAL(APB::DIVE_FRAME_BITS, bits);
		CHECK_TRUE(dive_crcs_ok(p));
		return hex(p);
	}
	/// Mirror of dx1_ref.dive_cycle(): a dive with a temperature sample every
	/// 60 s, a surfacing, the frame 15 s in when @p frame is given.
	static uint64_t dive_cycle(uint64_t t, unsigned int dive_s, unsigned int surf_s, int32_t q4_start, UtcFn utc,
	                           std::string *frame) {
		clk(t, utc(t));
		uw(true, t);
		for (unsigned int m = 1; m <= dive_s / 60U; m++) {
			if (m * 60U < dive_s) DiveBehaviourTracker::on_temp_sample(T0 + t + m * 60ULL * S, q4_start - (int32_t)m);
		}
		t += dive_s * S;
		clk(t, utc(t));
		uw(false, t, {}, 2);
		if (frame) {
			const uint64_t tb = t + 15ULL * S;
			*frame = dive_hex(tb, utc(tb), 3700);
		}
		return t + surf_s * S;
	}
};

TEST(Dx1DiveTracker, CyclesWithUtc) {
	struct {
		unsigned int n;
		const char *hex;
	} want[] = {
		{ 1, "006461810080000000000000008600000000000000000090" },
		{ 2, "006461810141000000046000008600000000000000000029" },
		{ 3, "0064618101C182000004600000860000000000000000004B" },
		{ 76, "00646DA5264183060C146210418628828A20A20A20A2896E" },
		{ 80, "00646DA5284183060C146210418620A20A28828828828AD3" },
		{ 82, "00646DA5294183060C146210418620A20A28828828828A05" },
	};
	uint64_t t = 0;
	unsigned int n = 0, w = 0;
	while (t < 30ULL * 3600ULL * S) {
		n++;
		std::string f;
		const bool label = (w < 6 && want[w].n == n);
		t = dive_cycle(t, 1200, 90, 112, utc_plain, label ? &f : nullptr);
		if (label) {
			STRCMP_EQUAL(want[w].hex, f.c_str());
			w++;
		}
	}
	CHECK_EQUAL(6U, w);
}

TEST(Dx1DiveTracker, ClockJumpClearsTheTimeline) {
	struct {
		unsigned int n;
		const char *hex;
	} want[] = {
		{ 27, "006461810DC183060C146000007A20A2882000000000019A" },
		{ 28, "006461810E4183060C146000007A0000000000000000002A" },
		{ 29, "006461810EC183060C146000007A00000000000000000146" },
		{ 36, "00646DA5124183060C146200007A200000000000000000B8" },
		{ 37, "00646DA512C183060C146200007A200000000000000000D3" },
	};
	uint64_t t = 0;
	unsigned int n = 0, w = 0;
	while (t < 16ULL * 3600ULL * S) {
		n++;
		std::string f;
		const bool label = (w < 5 && want[w].n == n);
		t = dive_cycle(t, 1200, 90, 100, utc_jump, label ? &f : nullptr);
		if (label) {
			STRCMP_EQUAL(want[w].hex, f.c_str());
			w++;
		}
	}
	CHECK_EQUAL(5U, w);
}

TEST(Dx1DiveTracker, ForwardClockStepClearsAndSubSecondJitterDoesNot) {
	// ClockJumpClearsTheTimeline only steps the clock BACK. A trusted clock that
	// was behind and that a fix steps FORWARD must clear the bins too; and the
	// < 1 s wobble between the RTC's whole seconds and the uptime's ms, present
	// on every board and in no test, must never clear them.
	auto known_bins = [](const std::string &h) {
		const KineisPacket p = Binascii::unhexlify(h);
		unsigned int n = 0;
		for (unsigned int k = 0; k < Dx1Dive::BINS; k++)
			n += (extract_bits(p, 111 + 6 * k, 6) != 0) ? 1U : 0U;
		return n;
	};
	// 1. +30 min from 10 h, seen at cycle 28's surface edge (t = 36 030 s).
	auto utc_fwd = [](uint64_t ms) -> int64_t { return utc_plain(ms) + (ms >= 10ULL * 3600ULL * S ? 1800 : 0); };
	uint64_t t = 0;
	std::string before, after;
	for (unsigned int n = 1; n <= 28; n++)
		t = dive_cycle(t, 1200, 90, 100, utc_fwd, n == 27 ? &before : (n == 28 ? &after : nullptr));
	CHECK(known_bins(before) >= 3U);     // 5 with these times
	CHECK_EQUAL(0U, known_bins(after));  // only the 1 215 s since the step remain

	// 2. The same cycles with the UTC - uptime offset alternating by 999 ms.
	DiveBehaviourTracker::reset(T0);
	t = 0;
	uint64_t s = 0;
	for (unsigned int n = 1; n <= 27; n++) {
		const uint64_t d = t + ((n % 2U) ? 999ULL : 0ULL);  // off the second on odd cycles
		clk(d, UTC0 + (int64_t)(d / 1000ULL));
		uw(true, d);
		s = d + 1200ULL * S;
		clk(s, UTC0 + (int64_t)(s / 1000ULL));
		uw(false, s, {}, 2);
		t += 1290ULL * S;
	}
	const uint64_t tb = s + 15ULL * S;
	CHECK(known_bins(dive_hex(tb, UTC0 + (int64_t)(tb / 1000ULL), 3700)) >= 3U);
}

TEST(Dx1DiveTracker, NoClockLeavesEveryBinUnknown) {
	const char *want[] = { "0064618101C386000004E000006C000000000000000000A6",
	                       "006461810243870C0004E000006C000000000000000000A6" };
	uint64_t t = 0;
	for (unsigned int n = 1; n <= 4; n++) {
		std::string f;
		t = dive_cycle(t, 1500, 120, 90, utc_none, n >= 3 ? &f : nullptr);
		if (n >= 3) STRCMP_EQUAL(want[n - 3], f.c_str());
	}
}

TEST(Dx1DiveTracker, VirtualClockLeavesEveryBinUnknown) {
	// dx1_ref.py dive_virtual_clock: 1 + uptime is never UTC (on_clock refuses
	// anything before 2000): dives reported, every bin and PHASE unknown.
	const char *want[] = { "00646181024E9D380004E000005200000000000000000018",
	                       "0064618102CE9D3A7004E000005200000000000000000057" };
	auto virtual_clock = [](uint64_t t) { return (int64_t)(1 + t / 1000ULL); };
	uint64_t t = 0;
	for (unsigned int n = 1; n <= 5; n++) {
		std::string f;
		t = dive_cycle(t, 3600, 120, 100, virtual_clock, n >= 4 ? &f : nullptr);
		if (n >= 4) STRCMP_EQUAL(want[n - 4], f.c_str());
	}
}

TEST(Dx1DiveTracker, ForcedSurfaceAndBlindDive) {
	uint64_t t = 0;
	t = dive_cycle(t, 1200, 90, 112, utc_plain, nullptr);
	t = dive_cycle(t, 1200, 90, 112, utc_plain, nullptr);
	// dive 3 ends in a forced surface (escalation): DEND 3, SURF_SEQ not counted
	clk(t, utc_plain(t));
	uw(true, t);
	DiveBehaviourTracker::on_temp_sample(T0 + t + 60ULL * S, 80);
	t += 21600ULL * S;
	clk(t, utc_plain(t));
	uw(false, t, { 3, 1, false }, 0);
	uint64_t tb = t + 15ULL * S;
	STRCMP_EQUAL("005A63A00162820000046000007A082180000000000001A1",
	             dive_hex(tb, utc_plain(tb), 3600, { 3, 1, false }).c_str());
	t += 30ULL * S;
	// dive 4 cut by a cooldown window: its duration is unknown at the next surfacing
	clk(t, utc_plain(t));
	uw(true, t, { 3, 1, false });
	DiveBehaviourTracker::on_blind_start(T0 + t + 10ULL * S);
	DiveBehaviourTracker::on_blind_end(T0 + t + 400ULL * S);
	t += 400ULL * S;
	clk(t, utc_plain(t));
	uw(true, t, { 3, 1, false });
	t += 800ULL * S;
	clk(t, utc_plain(t));
	uw(false, t, { 3, 1, false }, 2);
	tb = t + 15ULL * S;
	STRCMP_EQUAL("005A61AE0180C5040000000000000821800000000000012E",
	             dive_hex(tb, utc_plain(tb), 3600, { 3, 1, false }).c_str());
}

TEST(Dx1DiveTracker, SurfSeqWrapsFrom511To1) {
	// A foraging turtle passes 511 real surfacings in about a week; no test does.
	// The frame (this surfacing) and the latch (the DX1 Doppler) must read
	// 510, 511, 1, 2 -- never 512, which the 9-bit field sends as 0 = none.
	uint64_t t = 0;
	clk(t, utc_plain(t));
	uw(true, t);  // the first event only sets the state
	for (unsigned int n = 1; n <= 513; n++) {
		t += 60ULL * S;
		clk(t, utc_plain(t));
		uw(false, t, {}, 2);  // real surface edge number n
		const unsigned int want = (n - 1U) % 511U + 1U;
		if (n >= 510) CHECK_EQUAL(want, (unsigned int)DiveBehaviourTracker::dive_data(T0 + t).surf_seq);
		t += 30ULL * S;
		clk(t, utc_plain(t));
		uw(true, t);  // dive edge: surfacing n counted
		if (n >= 510) CHECK_EQUAL(want, (unsigned int)DiveBehaviourTracker::latch(T0 + t).surf_seq);
	}
}

TEST(Dx1DiveTracker, OpenSurfaceIsCreditedToTheBinItCompleted) {
	// dx1_ref.py dive_open_surface: the surface in progress crosses an even UTC
	// hour 10 min before the boundary, the frame is built 5 min after it. The
	// completed bin holds 10 min dry out of 120 (8 %), credited from the open
	// episode only -- nothing has been accounted at a dive edge yet.
	const int64_t e0 = 247223LL * 7200LL;
	const int64_t base = e0 - 10800;
	clk(0, base);
	uw(true, 0);
	const uint64_t ts = (10800ULL - 600ULL) * S;
	clk(ts, base + (int64_t)(ts / 1000ULL));
	uw(false, ts, {}, 2);
	const uint64_t tb = (10800ULL + 300ULL) * S;
	STRCMP_EQUAL("0064618100800000000000000000282000000000000001CE",
	             dive_hex(tb, base + (int64_t)(tb / 1000ULL), 3700).c_str());
}

TEST(Dx1DiveTracker, TimelineDropsPiecesOlderThanItsWindow) {
	// Defensive: intervals arrive in time order, so a piece older than the 12-bin
	// window never shows up on the tag. If one did, it must neither be stored
	// nor evict a bin inside the window (dx1_ref.py Timeline.add, same rule).
	Dx1Timeline tl;
	const uint64_t bin = Dx1Dive::BIN_MS;
	tl.add(100ULL * bin, 100ULL * bin + 3600000ULL, true);  // bin 100
	tl.add(87ULL * bin, 87ULL * bin + 3600000ULL, false);   // 13 bins older: dropped
	CHECK(tl.find(87) == nullptr);
	tl.add(88ULL * bin, 88ULL * bin + 1000ULL, false);  // 12 bins older: still in the window
	CHECK(tl.find(88) != nullptr);
	const Dx1TimelineBin *b = tl.find(100);
	CHECK(b != nullptr);
	CHECK_EQUAL(3600U, (unsigned int)b->obs_s);
	CHECK_EQUAL(3600U, (unsigned int)b->dry_s);
	tl.add(112ULL * bin, 112ULL * bin + 1000ULL, false);  // newest 112: the window is [100, 112]
	CHECK(tl.find(88) == nullptr);
	CHECK(tl.find(100) != nullptr);
	tl.add(113ULL * bin, 113ULL * bin + 1000ULL, false);  // newest 113: bin 100 leaves the window
	CHECK(tl.find(100) == nullptr);
	CHECK(tl.find(112) != nullptr);
}

TEST(Dx1DiveTracker, TemperatureOnlyUnderwaterAndPerDive) {
	clk(0, UTC0);
	uw(true, 0);
	DiveBehaviourTracker::on_temp_sample(T0 + 60 * S, 120);
	DiveBehaviourTracker::on_temp_sample(T0 + 120 * S, 116);  // the last one of the dive wins
	clk(600 * S, UTC0 + 600);
	uw(false, 600 * S, {}, 2);
	DiveBehaviourTracker::on_temp_sample(T0 + 630 * S, 200);  // at the surface: ignored
	Dx1DiveData d = DiveBehaviourTracker::dive_data(T0 + 640 * S);
	CHECK_TRUE(d.temp_valid);
	CHECK_EQUAL(116, d.temp_q4);
	clk(700 * S, UTC0 + 700);
	uw(true, 700 * S);  // a new dive forgets the previous one's reading
	d = DiveBehaviourTracker::dive_data(T0 + 710 * S);
	CHECK_FALSE(d.temp_valid);
}

// =============================================================== ArgosTx
TEST_GROUP(Dx1ArgosTx) {
	FakeBatteryMonitor *fake_battery_monitor;
	FakeConfigurationStore *fake_config_store;
	MockKineisDevice *mock_kineis;
	FakeRTC *fake_rtc;
	FakeTimer *fake_timer;
	uint64_t t = 1652105502000ULL;
	uint64_t t_boot = 0;  // t at boot(): the uptime is t - t_boot

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
		gps_device = nullptr;
		SWSAnalogService::reset_noinit_data();
		DiveBehaviourTracker::reset(0);
		ServiceManager::reset();
	}

	void set_time(uint64_t ms) {
		t = ms;
		fake_rtc->settime((std::time_t)(ms / 1000));
		fake_timer->set_counter(ms - t_boot);
	}

	/// The board booted @p up_ms ago: the uptime restarts near 0 while the RTC
	/// keeps its date. Without it the uptime is the epoch (~1.65e12 ms), past any
	/// limit the service counts from boot. Call before serv.start(): the counter
	/// steps back, and a task posted on the old one would never fire. Not 0,
	/// which the prewarm and TX-spacing stamps read as "never".
	void boot(uint64_t up_ms = 60000ULL) {
		t_boot = t - up_ms;
		set_time(t);
		DiveBehaviourTracker::reset();  // the entry into Operational, at that uptime
	}

	void uw(bool state) {
		ServiceEvent e;
		e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
		e.event_data = state;
		e.event_source = ServiceIdentifier::UW_SENSOR;
		e.event_originator_unique_id = 0x12345678;
		ServiceManager::notify_peer_event(e);
	}

	/// SWS_SENSOR_ENABLE_TX_MODE (UNP26). The host build compiles the dive info
	/// in (DIVE_INFO_MESSAGE=1): AUTO and FORCE emit DX1, OFF the legacy frames.
	void gate(unsigned int mode) {
		fake_config_store->write_param(ParamID::SWS_SENSOR_ENABLE_TX_MODE, mode);
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

	/// The module accepted the in-flight TX (KIM2 and SmdSat both notify it).
	void tx_started() {
		mock_kineis->notify(KineisEventTxStarted({}));
	}

	/// A fresh GNSS fix in the RAM cache, as GPSService leaves it.
	void cache_fix() {
		GPSLogEntry log{};
		const std::time_t now = (std::time_t)(t / 1000);
		uint16_t year;
		uint8_t month, day, hour, min, sec;
		convert_datetime_to_epoch(now, year, month, day, hour, min, sec);
		log.header.log_type = LOG_GPS;
		log.header.year = log.info.year = year;
		log.header.month = log.info.month = month;
		log.header.day = log.info.day = day;
		log.header.hours = log.info.hour = hour;
		log.header.minutes = log.info.min = min;
		log.header.seconds = log.info.sec = sec;
		log.info.schedTime = now;
		log.info.valid = 1;
		log.info.event_type = GPSEventType::FIX;
		log.info.fixType = 3;
		log.info.lat = -21.09;
		log.info.lon = 55.24;
		log.info.batt_voltage = 3700;
		configuration_store->notify_gps_location(log);
	}

	/// The in-flight TX went on air (MAC_TX_DONE / +TX): what spends a dive frame.
	void tx_done() {
		mock_kineis->notify(KineisEventTxComplete({}));
	}

	/// One short dive cycle, its ping 1 sent and completed, then @p surf_s at
	/// the surface: the next surfacing closes a dive whose start the tracker saw,
	/// so the dive frame has a known D_LAST. Without it, the first surfacing after
	/// a tracker reset has nothing to say and sends no dive frame.
	void prime(unsigned int down_s = 600, unsigned int surf_s = 90) {
		uw(true);
		set_time(t + down_s * 1000ULL);
		uw(false);
		system_scheduler->run();
		tx_started();
		tx_done();
		set_time(t + surf_s * 1000ULL);
	}

	/// Dive @p down_s, surface, ping 1 accepted, then ping 2 on the standard path.
	/// Returns false if ping 2 was not sent at all (the tests assert it).
	bool surface_and_ping2(unsigned int down_s = 1200) {
		dive_and_surface(down_s);
		tx_started();
		const unsigned int before = mock_kineis->send_count;
		next_ping();
		return mock_kineis->send_count == before + 1U;
	}

	/// The fake RTC as a GNSS-synchronised clock: the timeline trusts nothing
	/// else but an operator RTCW (dx1_trusted_utc).
	void gnss_clock() {
		fake_rtc->note_source(RtcSource::GNSS);
	}

	static bool is_dive_frame(const KineisPacket &p, unsigned int bits) {
		return bits == ArgosPacketBuilder::DIVE_FRAME_BITS && p.size() == ArgosPacketBuilder::DIVE_FRAME_BYTES
		       && (uint8_t)p[0] == 0 && extract_bits(p, 16, 2) == 1 && extract_bits(p, 18, 1) == 1
		       && (uint8_t)p[3] == CRC8::checksum(p, 24) && (uint8_t)p[23] == CRC8::checksum(p, 184);
	}
};

TEST(Dx1ArgosTx, GateOffIsLegacyOnEveryModulation) {
	struct {
		unsigned int mode;
		bool uw_en;
	} variants[] = {
		{ SwsTxMode::OFF, true },     // OFF
		{ SwsTxMode::FORCE, false },  // FORCE, but UNDERWATER_EN off
		{ 3U, true },                 // out of range (a stale stored value): OFF
		{ SwsTxMode::AUTO, false },   // AUTO, but UNDERWATER_EN off
	};
	for (KineisModulation m : { KineisModulation::LDA2, KineisModulation::LDK, KineisModulation::VLDA4 }) {
		for (const auto &v : variants) {
			gate(v.mode);
			fake_config_store->write_param(ParamID::UNDERWATER_EN, v.uw_en);
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
	gate(SwsTxMode::FORCE);
	for (const auto &c : cases) {
		DiveBehaviourTracker::reset();
		mock_kineis->test_set_current_modulation(c.m);
		ArgosTxService serv(*mock_kineis);
		serv.start();
		mock_kineis->send_count = 0;
		fake_battery_monitor->set_values(100, 3700);  // the dive-time reading every case encodes
		uw(true);
		// Prewarm path, not a fresh build: the battery moves with no peer event to
		// notice it, and the surface must still send the dive-time byte. Counting
		// samples cannot tell the two apart any more -- the monitor takes one when
		// its last is older than SAMPLE_MAX_AGE_MS, whichever path runs.
		fake_battery_monitor->set_values(100, 3600);
		set_time(t + 1200000ULL);
		uw(false);
		system_scheduler->run();
		CHECK_EQUAL(1U, mock_kineis->send_count);
		CHECK_EQUAL(c.bits, mock_kineis->last_size_bits);
		STRCMP_EQUAL(c.hex, hex(mock_kineis->last_packet).c_str());
		if (c.bits >= 32) check_crc(mock_kineis->last_packet);
		serv.stop();
	}
}

TEST(Dx1ArgosTx, PrewarmRefreshesDiveEndAfterAForcedSurface) {
	gate(SwsTxMode::FORCE);
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
	gate(SwsTxMode::FORCE);
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
	gate(SwsTxMode::FORCE);
	mock_kineis->test_set_current_modulation(KineisModulation::LDA2);
	ArgosTxService serv(*mock_kineis);
	serv.start();
	uw(true);
	CHECK_EQUAL(3700U, serv.prepared_doppler_voltage());
	fake_battery_monitor->set_values(100, 3600);  // inputs changed -> a qualifying event rebuilds
	ServiceEvent e;
	e.event_source = ServiceIdentifier::UW_SENSOR;
	e.event_originator_unique_id = 0x12345678;
	e.event_type = ServiceEventType::SERVICE_ACTIVE;
	ServiceManager::notify_peer_event(e);
	CHECK_EQUAL(3700U, serv.prepared_doppler_voltage());
	e.event_type = ServiceEventType::SERVICE_INACTIVE;  // status says surface: no rebuild
	ServiceManager::notify_peer_event(e);
	CHECK_EQUAL(3700U, serv.prepared_doppler_voltage());
	SWSAnalogService::Status st{};
	st.is_underwater = true;
	SWSAnalogService::test_set_status(st);
	ServiceManager::notify_peer_event(e);  // end of an underwater sample: rebuild
	CHECK_EQUAL(3600U, serv.prepared_doppler_voltage());
	serv.stop();
}

TEST(Dx1ArgosTx, LatchedAdcFaultSampleEndDoesNotRefresh) {
	// An invalid ADC read returns before the status push, so the status keeps
	// the last valid "underwater" while the detector forces the surface: the
	// INACTIVE of the read that latches the fault is the surfacing one.
	SWSAnalogService sws;  // outlives serv; teardown clears s_instance
	gate(SwsTxMode::FORCE);
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
	ServiceManager::notify_peer_event(e);
	CHECK_EQUAL(3600U, serv.prepared_doppler_voltage());

	// The read that latches the fault forces the surface: no rebuild.
	SWSAnalogService::test_set_consecutive_invalid_adc(SWSAnalogService::test_max_consecutive_invalid_adc());
	CHECK_TRUE(SWSAnalogService::adc_fault_active());
	// Never rebuilt from the 3500 mV reading (the surfacing may consume the packet,
	// so its absence, 0, is also an acceptable answer).
	fake_battery_monitor->set_values(100, 3500);
	ServiceManager::notify_peer_event(e);
	CHECK_FALSE(serv.prepared_doppler_voltage() == 3500U);
	// ...nor on the hourly backstop.
	set_time(t + 3600000ULL);
	ServiceManager::notify_peer_event(e);
	CHECK_FALSE(serv.prepared_doppler_voltage() == 3500U);
	serv.stop();
}

TEST(Dx1ArgosTx, ModulationMismatchSendsTheViewTheModuleFrames) {
	gate(SwsTxMode::FORCE);
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

// The Doppler was prepared while the driver said LDA2; by the time the tag
// surfaces the driver holds LDK (restore / resync / read-back). The prewarm
// send must go out as LDK -- the dive-time snapshot would hand an LDK radio an
// LDA2-tagged frame. Legacy and DX1 alike.
TEST(Dx1ArgosTx, PrewarmSendsOnTheSendTimeModulationLegacy) {
	gate(SwsTxMode::OFF);
	mock_kineis->test_set_current_modulation(KineisModulation::LDA2);  // prepared on LDA2
	ArgosTxService serv(*mock_kineis);
	serv.start();
	uw(true);
	set_time(t + 1200000ULL);
	mock_kineis->test_set_current_modulation(KineisModulation::LDK);  // module now LDK
	const unsigned int updates = fake_battery_monitor->update_calls;
	uw(false);
	system_scheduler->run();
	CHECK_EQUAL(updates, fake_battery_monitor->update_calls);  // prewarm path
	CHECK_EQUAL(1U, mock_kineis->send_count);
	CHECK_EQUAL((int)KineisModulation::LDK, (int)mock_kineis->last_mode);
	CHECK_EQUAL(24U, mock_kineis->last_size_bits);
	serv.stop();
}

TEST(Dx1ArgosTx, PrewarmSendsOnTheSendTimeModulationDx1) {
	gate(SwsTxMode::FORCE);
	mock_kineis->test_set_current_modulation(KineisModulation::LDA2);  // prepared on LDA2
	ArgosTxService serv(*mock_kineis);
	serv.start();
	uw(true);
	set_time(t + 1200000ULL);
	mock_kineis->test_set_current_modulation(KineisModulation::LDK);  // module now LDK
	const unsigned int updates = fake_battery_monitor->update_calls;
	uw(false);
	system_scheduler->run();
	CHECK_EQUAL(updates, fake_battery_monitor->update_calls);  // prewarm path
	CHECK_EQUAL(1U, mock_kineis->send_count);
	CHECK_EQUAL((int)KineisModulation::LDK, (int)mock_kineis->last_mode);
	CHECK_EQUAL(128U, mock_kineis->last_size_bits);
	CHECK_EQUAL(16U, (unsigned int)mock_kineis->last_packet.size());
	serv.stop();
}

TEST(Dx1ArgosTx, ConfigResolvesTheSwsTxGateOnEveryRegime) {
	// Not gated by GNSS, the low-battery regime or ARGOS_MODE: DX1 rides the
	// Doppler, which is only built where it already exists.
	for (unsigned int mode = 0; mode <= 3; mode++) {
		for (int uw = 0; uw < 2; uw++) {
			for (int gnss = 0; gnss < 2; gnss++) {
				for (int lb = 0; lb < 2; lb++) {
					for (BaseArgosMode am : { BaseArgosMode::LEGACY, BaseArgosMode::DOPPLER,
					                          BaseArgosMode::SURFACING_BURST, BaseArgosMode::OFF }) {
						gate(mode);
						fake_config_store->write_param(ParamID::UNDERWATER_EN, (bool)uw);
						fake_config_store->write_param(ParamID::GNSS_EN, (bool)gnss);
						fake_config_store->write_param(ParamID::LB_GNSS_EN, (bool)gnss);
						fake_config_store->write_param(ParamID::LB_EN, (bool)lb);
						fake_config_store->set_is_battery_level_low((bool)lb);
						fake_config_store->write_param(ParamID::ARGOS_MODE, am);
						fake_config_store->write_param(ParamID::LB_ARGOS_MODE, am);
						ArgosConfig cfg;
						configuration_store->get_argos_configuration(cfg);
						CHECK_EQUAL((bool)lb, cfg.is_lb);
						CHECK_EQUAL((bool)gnss, cfg.gnss_en);
						// Host builds: dive info compiled in (DIVE_INFO_MESSAGE=1).
						const bool expect = uw && (mode == SwsTxMode::FORCE || mode == SwsTxMode::AUTO);
						CHECK_EQUAL(expect, cfg.sws_tx_doppler_ext);
					}
				}
			}
		}
	}
}

TEST(Dx1ArgosTx, DefaultIsAutoAndAutoIsOnADiveInfoBuild) {
	// No release gate any more (2026-11): on a build compiled with
	// DIVE_INFO_MESSAGE, the default AUTO emits DX1 and the dive frame.
	FakeConfigurationStore fresh;
	fresh.init();
	CHECK_EQUAL(SwsTxMode::AUTO, fresh.read_param<unsigned int>(ParamID::SWS_SENSOR_ENABLE_TX_MODE));
	gate(SwsTxMode::AUTO);
	ArgosConfig cfg;
	configuration_store->get_argos_configuration(cfg);
	CHECK_TRUE(cfg.sws_tx_doppler_ext);
}

// =============================================================== SWS TX gate
// --------------------------------------------------------------- dive frame
TEST(Dx1ArgosTx, DiveFrameTakesPing2OnceAndPing1StaysCompact) {
	gate(SwsTxMode::FORCE);
	mock_kineis->test_set_current_modulation(KineisModulation::LDA2);
	ArgosTxService serv(*mock_kineis);
	serv.start();
	prime();
	mock_kineis->send_count = 0;
	dive_and_surface();
	CHECK_EQUAL(1U, mock_kineis->send_count);
	CHECK_EQUAL(32U, mock_kineis->last_size_bits);  // ping 1: the 4-byte compact view, unchanged
	tx_started();
	next_ping();
	CHECK_EQUAL(2U, mock_kineis->send_count);
	CHECK_TRUE(is_dive_frame(mock_kineis->last_packet, mock_kineis->last_size_bits));
	tx_started();
	next_ping();  // once per surfacing: ping 3 is the compact Doppler again
	CHECK_EQUAL(3U, mock_kineis->send_count);
	CHECK_EQUAL(32U, mock_kineis->last_size_bits);
	serv.stop();
}

TEST(Dx1ArgosTx, DiveFrameTakesAMessageCounterOfItsOwn) {
	// Operator rule (2026-10-12): the Dopplers of one surfacing share one MC, a
	// frame with new information takes its own. The dive frame rides the burst
	// between two Dopplers and must not borrow theirs.
	gate(SwsTxMode::FORCE);
	mock_kineis->test_set_current_modulation(KineisModulation::LDA2);
	ArgosTxService serv(*mock_kineis);
	serv.start();
	prime();
	dive_and_surface();
	CHECK_EQUAL(32U, mock_kineis->last_size_bits);  // ping 1: the surfacing's Doppler
	const uint16_t doppler_mc = mock_kineis->last_mc;
	tx_started();
	next_ping();
	CHECK_TRUE(is_dive_frame(mock_kineis->last_packet, mock_kineis->last_size_bits));
	CHECK_FALSE(mock_kineis->last_mc == doppler_mc);  // DATA: a counter of its own, never ping 1's
	tx_started();
	next_ping();
	CHECK_EQUAL(32U, mock_kineis->last_size_bits);  // ping 3: the Doppler again...
	CHECK_EQUAL(doppler_mc, mock_kineis->last_mc);  // ...back on the surfacing's counter
	serv.stop();
}

TEST(Dx1ArgosTx, DiveFrameNeverRidesTheFirstAcceptedTx) {
	gate(SwsTxMode::FORCE);
	mock_kineis->test_set_current_modulation(KineisModulation::LDA2);
	ArgosTxService serv(*mock_kineis);
	serv.start();
	prime();
	dive_and_surface();
	const unsigned int sends = mock_kineis->send_count;
	next_ping();  // ping 1 never accepted: ping 2 is the first one on air
	CHECK_EQUAL(sends + 1U, mock_kineis->send_count);
	CHECK_EQUAL(32U, mock_kineis->last_size_bits);
	tx_started();
	next_ping();
	CHECK_EQUAL(sends + 2U, mock_kineis->send_count);
	CHECK_TRUE(is_dive_frame(mock_kineis->last_packet, mock_kineis->last_size_bits));
	serv.stop();
}

TEST(Dx1ArgosTx, PingOneRefusedAfterTxStartedIsNotTheFirstOnAir) {
	// KIM2 notifies TxStarted when it writes AT+TX, before the module's +OK. A
	// ping 1 it then refuses (+ERROR: DeviceError) went nowhere: the retry is the
	// first frame on air of the surfacing, so it stays the compact Doppler, and
	// only the ping after it may carry the dive frame.
	gate(SwsTxMode::FORCE);
	mock_kineis->test_set_current_modulation(KineisModulation::LDA2);
	ArgosTxService serv(*mock_kineis);
	serv.start();
	prime();
	dive_and_surface();
	tx_started();                                     // AT+TX written...
	mock_kineis->notify(KineisEventDeviceError({}));  // ...then +ERROR: first strike, 60 s backoff
	const unsigned int sends = mock_kineis->send_count;
	for (unsigned int i = 0; i < 10 && mock_kineis->send_count == sends; i++) {
		set_time(t + 60000ULL);
		system_scheduler->run();
	}
	CHECK(mock_kineis->send_count > sends);
	CHECK_FALSE(is_dive_frame(mock_kineis->last_packet, mock_kineis->last_size_bits));
	CHECK_EQUAL(32U, mock_kineis->last_size_bits);  // the retry, first on air: the compact Doppler
	tx_started();
	next_ping();
	CHECK_TRUE(is_dive_frame(mock_kineis->last_packet, mock_kineis->last_size_bits));
	serv.stop();
}

TEST(Dx1ArgosTx, NoDiveFrameWithNothingToSay) {
	// First surfacing after a tracker reset: the dive start was never seen, no
	// bin is observed for an hour (and the fake RTC is not trusted): no frame.
	gate(SwsTxMode::FORCE);
	mock_kineis->test_set_current_modulation(KineisModulation::LDA2);
	ArgosTxService serv(*mock_kineis);
	serv.start();
	CHECK_TRUE(surface_and_ping2());
	CHECK_FALSE(is_dive_frame(mock_kineis->last_packet, mock_kineis->last_size_bits));
	CHECK_EQUAL(32U, mock_kineis->last_size_bits);
	serv.stop();
}

TEST(Dx1ArgosTx, BinUnderAnHourIsNothingToSay) {
	// First surfacing after a tracker reset, on a GNSS clock: no dive is known,
	// so only a bin can make the frame worth its slot. The dive starts obs_s
	// before a 2-h UTC edge and ping 2 comes 11 min past it: an hour less a
	// second is a code 0 like no observation at all, a full hour is a known bin.
	gate(SwsTxMode::FORCE);
	gnss_clock();
	mock_kineis->test_set_current_modulation(KineisModulation::LDA2);
	for (unsigned int obs_s : { 3599U, 3600U }) {
		set_time((t / Dx1Dive::BIN_MS + 1ULL) * Dx1Dive::BIN_MS - obs_s * 1000ULL);
		DiveBehaviourTracker::reset();  // an entry into Operational
		ArgosTxService serv(*mock_kineis);
		serv.start();
		CHECK_TRUE(surface_and_ping2(obs_s + 600));  // surfaced 10 min past the edge
		const KineisPacket p = mock_kineis->last_packet;
		if (obs_s < 3600U) {
			CHECK_FALSE(is_dive_frame(p, mock_kineis->last_size_bits));
			CHECK_EQUAL(32U, mock_kineis->last_size_bits);  // the compact Doppler keeps the slot
		} else {
			CHECK_TRUE(is_dive_frame(p, mock_kineis->last_size_bits));
			CHECK_EQUAL(0U, extract_bits(p, 41, 7));   // D_LAST: start unseen
			CHECK_EQUAL(1U, extract_bits(p, 111, 6));  // bin 0: the hour observed, 0 % dry
		}
		serv.stop();
	}
}

TEST(Dx1ArgosTx, DiveFrameOnlyOnAnLda2MasterWithTheGateOpen) {
	struct {
		unsigned int mode;
		KineisModulation m;
		bool adaptive;
		unsigned int ping2_bits;
	} cases[] = {
		{ SwsTxMode::OFF, KineisModulation::LDA2, false, 24 },    // UNP26 OFF: legacy
		{ SwsTxMode::FORCE, KineisModulation::LDK, false, 128 },  // LDK master: the 16-B DX1 view
		{ SwsTxMode::AUTO, KineisModulation::LDK, false, 128 },   // AUTO the same
		{ SwsTxMode::FORCE, KineisModulation::LDA2, true, 32 },   // adaptive modulation: never
	};
	for (const auto &c : cases) {
		gate(c.mode);
		fake_config_store->write_param(ParamID::ARGOS_ADAPTIVE_MODULATION, c.adaptive);
		mock_kineis->test_set_current_modulation(c.m);
		DiveBehaviourTracker::reset();
		ArgosTxService serv(*mock_kineis);
		serv.start();
		prime();
		CHECK_TRUE(surface_and_ping2());
		CHECK_FALSE(is_dive_frame(mock_kineis->last_packet, mock_kineis->last_size_bits));
		if (!c.adaptive) CHECK_EQUAL(c.ping2_bits, mock_kineis->last_size_bits);
		serv.stop();
	}
	fake_config_store->write_param(ParamID::ARGOS_ADAPTIVE_MODULATION, false);
}

TEST(Dx1ArgosTx, TrustedLdkMasterVetoesAnLda2Module) {
	// An LDK master whose module was escalated to LDA2 (degraded PVT, sticky on
	// SMD): the trusted read-back of the master (ART13) says LDK, so no dive frame.
	gate(SwsTxMode::FORCE);
	// Trust needs a provisioned master: any non-empty RCONF, fingerprinted with it.
	fake_config_store->write_param(ParamID::ARGOS_RADIOCONF, std::string("dive-frame-test-ldk-master"));
	fake_config_store->write_master_modulation_readback(ModulationGuard::CODE_LDK);
	CHECK_EQUAL(ModulationGuard::CODE_LDK, fake_config_store->trusted_master_modulation());
	mock_kineis->test_set_current_modulation(KineisModulation::LDA2);
	ArgosTxService serv(*mock_kineis);
	serv.start();
	prime();
	CHECK_TRUE(surface_and_ping2());
	CHECK_FALSE(is_dive_frame(mock_kineis->last_packet, mock_kineis->last_size_bits));
	serv.stop();
}

TEST(Dx1ArgosTx, DiveFrameOnlyInSurfacingBurst) {
	// DOPPLER (the turtle low-battery mode) and LEGACY with the gate open: the
	// Doppler path runs, but never the dive frame.
	for (BaseArgosMode mode : { BaseArgosMode::DOPPLER, BaseArgosMode::LEGACY }) {
		fake_config_store->write_param(ParamID::ARGOS_MODE, mode);
		fake_config_store->write_param(ParamID::GNSS_EN, false);
		gate(SwsTxMode::FORCE);
		mock_kineis->test_set_current_modulation(KineisModulation::LDA2);
		DiveBehaviourTracker::reset();
		ArgosTxService serv(*mock_kineis);
		serv.start();
		prime();
		dive_and_surface();
		const unsigned int sends0 = mock_kineis->send_count;
		unsigned int sends = sends0, dives = 0;
		for (unsigned int i = 0; i < 10; i++) {
			tx_started();
			next_ping();
			if (mock_kineis->send_count != sends) {
				sends = mock_kineis->send_count;
				if (is_dive_frame(mock_kineis->last_packet, mock_kineis->last_size_bits)) dives++;
			}
		}
		CHECK_TEXT(sends >= sends0 + 2U, mode == BaseArgosMode::DOPPLER ? "DOPPLER sent" : "LEGACY sent");
		CHECK_EQUAL(0U, dives);
		serv.stop();
	}
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::SURFACING_BURST);
	fake_config_store->write_param(ParamID::GNSS_EN, true);
}

TEST(Dx1ArgosTx, LowBatteryDuringABurstStopsTheDiveFrame) {
	// The regime switches to LB_ARGOS_MODE = DOPPLER while a surfacing burst is
	// running: the burst flags are still up, the effective mode check stops it.
	gate(SwsTxMode::FORCE);
	mock_kineis->test_set_current_modulation(KineisModulation::LDA2);
	fake_config_store->write_param(ParamID::LB_EN, true);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, 50U);
	fake_config_store->write_param(ParamID::LB_ARGOS_MODE, BaseArgosMode::DOPPLER);
	ArgosTxService serv(*mock_kineis);
	serv.start();
	prime();
	dive_and_surface();
	tx_started();
	fake_config_store->set_is_battery_level_low(true);  // SOC below LB_THRESHOLD from now on
	const unsigned int sends0 = mock_kineis->send_count;
	unsigned int sends = sends0, dives = 0;
	for (unsigned int i = 0; i < 5; i++) {
		next_ping();
		tx_started();
		if (mock_kineis->send_count != sends) {
			sends = mock_kineis->send_count;
			if (is_dive_frame(mock_kineis->last_packet, mock_kineis->last_size_bits)) dives++;
		}
	}
	ArgosConfig cfg;
	configuration_store->get_argos_configuration(cfg);
	CHECK_TRUE(cfg.is_lb);  // the low-battery regime really is the one running
	CHECK(cfg.mode == BaseArgosMode::DOPPLER);
	CHECK(sends > sends0);
	CHECK_EQUAL(0U, dives);
	serv.stop();
	fake_config_store->set_is_battery_level_low(false);
	fake_config_store->write_param(ParamID::LB_EN, false);
}

TEST(Dx1ArgosTx, LongSurfaceGetsOneDiveFrame) {
	// A basking turtle: one surfacing longer than the 2-h spacing still carries
	// a single dive frame -- the per-surfacing rule, not the spacing, stops it.
	// The rate limiter is off here: a full window would hold the heartbeat until
	// an SWS sample event this harness never sends.
	fake_config_store->write_param(ParamID::RATE_LIMIT_EN, (bool)false);
	gate(SwsTxMode::FORCE);
	mock_kineis->test_set_current_modulation(KineisModulation::LDA2);
	ArgosTxService serv(*mock_kineis);
	serv.start();
	prime();
	CHECK_TRUE(surface_and_ping2());
	CHECK_TRUE(is_dive_frame(mock_kineis->last_packet, mock_kineis->last_size_bits));
	const uint64_t t_dive = t;
	unsigned int sends = mock_kineis->send_count, dives = 0;
	uint64_t last_send = t;
	for (unsigned int i = 0; i < 150; i++) {  // 150 min at the surface
		tx_started();
		next_ping();
		if (mock_kineis->send_count != sends) {
			sends = mock_kineis->send_count;
			last_send = t;
			if (is_dive_frame(mock_kineis->last_packet, mock_kineis->last_size_bits)) dives++;
		}
	}
	CHECK(last_send - t_dive > 2ULL * 3600000ULL);  // the heartbeat still transmitted past the 2-h spacing
	CHECK_EQUAL(0U, dives);
	serv.stop();
}

TEST(Dx1ArgosTx, DiveFrameRidesAPresenceHeartbeat) {
	// Spec 5.1: burst pings AND heartbeats. SURFACING_BURST_MAX_MSG = 1 ends the
	// burst at ping 1: every later TX of the surfacing is a heartbeat
	// (m_awaiting_surfacing), a state no other dive-frame test reaches.
	fake_config_store->write_param(ParamID::SURFACING_BURST_MAX_MSG, 1U);
	fake_config_store->write_param(ParamID::RATE_LIMIT_EN, (bool)false);
	gate(SwsTxMode::FORCE);
	mock_kineis->test_set_current_modulation(KineisModulation::LDA2);
	ArgosTxService serv(*mock_kineis);
	serv.start();
	prime();
	dive_and_surface();
	CHECK_EQUAL(32U, mock_kineis->last_size_bits);  // ping 1, the whole burst
	tx_started();
	const unsigned int sends = mock_kineis->send_count;
	next_ping();  // completes ping 1; the burst is over
	for (unsigned int i = 0; i < 5 && mock_kineis->send_count == sends; i++) {
		set_time(t + 60000ULL);
		system_scheduler->run();
	}
	CHECK(mock_kineis->send_count > sends);  // a heartbeat went
	CHECK_TRUE(is_dive_frame(mock_kineis->last_packet, mock_kineis->last_size_bits));
	serv.stop();
}

TEST(Dx1ArgosTx, DiveFrameAtMostEveryTwoHours) {
	gate(SwsTxMode::FORCE);
	mock_kineis->test_set_current_modulation(KineisModulation::LDA2);
	ArgosTxService serv(*mock_kineis);
	serv.start();
	prime();
	CHECK_TRUE(surface_and_ping2());
	CHECK_TRUE(is_dive_frame(mock_kineis->last_packet, mock_kineis->last_size_bits));
	tx_started();
	tx_done();
	CHECK_TRUE(surface_and_ping2(1800));  // ~32 min later: too soon
	CHECK_FALSE(is_dive_frame(mock_kineis->last_packet, mock_kineis->last_size_bits));
	CHECK_EQUAL(32U, mock_kineis->last_size_bits);
	tx_started();
	tx_done();
	CHECK_TRUE(surface_and_ping2(5400));  // > 2 h after the dive frame went on air
	CHECK_TRUE(is_dive_frame(mock_kineis->last_packet, mock_kineis->last_size_bits));
	serv.stop();
}

TEST(Dx1ArgosTx, SpacingAndOverdueEdgesAreTwoAndSixHours) {
	// The existing tests bracket the spacing between 31 min and 2 h 02 and the
	// overdue point between 3 h 01 and 6 h 02. Pin both edges to the minute.
	gate(SwsTxMode::FORCE);
	mock_kineis->test_set_current_modulation(KineisModulation::LDA2);
	ArgosTxService serv(*mock_kineis);
	serv.start();
	prime();
	CHECK_TRUE(surface_and_ping2());
	CHECK_TRUE(is_dive_frame(mock_kineis->last_packet, mock_kineis->last_size_bits));
	tx_started();
	tx_done();  // on air: the 2 h and the 6 h run from here
	const uint64_t t1 = t;
	CHECK_TRUE(surface_and_ping2(7080));  // ping 2 at +1 h 59 min
	CHECK_EQUAL(t1 + 7140000ULL, t);
	CHECK_FALSE(is_dive_frame(mock_kineis->last_packet, mock_kineis->last_size_bits));
	tx_started();
	tx_done();
	CHECK_TRUE(surface_and_ping2(60));  // ping 2 at +2 h 01 min
	CHECK_EQUAL(t1 + 7260000ULL, t);
	CHECK_TRUE(is_dive_frame(mock_kineis->last_packet, mock_kineis->last_size_bits));
	tx_started();
	tx_done();
	const uint64_t t2 = t;
	// +5 h 58 min with a fresh fix cached: not overdue yet, the position goes.
	uw(true);
	set_time(t + 21420000ULL);
	cache_fix();
	uw(false);
	system_scheduler->run();
	tx_started();
	next_ping();
	CHECK_EQUAL(t2 + 21480000ULL, t);
	CHECK_EQUAL(ArgosPacketBuilder::SHORT_PACKET_BITS, mock_kineis->last_size_bits);
	tx_started();
	tx_done();
	// +6 h 01 min, a fresh fix cached again: overdue, ahead of the cached Short.
	uw(true);
	set_time(t + 120000ULL);
	cache_fix();
	uw(false);
	system_scheduler->run();
	tx_started();
	next_ping();
	CHECK_EQUAL(t2 + 21660000ULL, t);
	CHECK_TRUE(is_dive_frame(mock_kineis->last_packet, mock_kineis->last_size_bits));
	serv.stop();
}

TEST(Dx1ArgosTx, FirstDiveFrameAfterBootNeedsNoSpacing) {
	// The 2-h spacing runs from the last dive frame on air: before the first one
	// since boot there is nothing to wait for, even 33 min into the uptime.
	gate(SwsTxMode::FORCE);
	mock_kineis->test_set_current_modulation(KineisModulation::LDA2);
	boot();
	ArgosTxService serv(*mock_kineis);
	serv.start();
	prime();
	CHECK_TRUE(surface_and_ping2());
	CHECK(PMU::get_timestamp_ms() < Dx1Dive::MIN_SPACING_MS);  // ping 2 at 33 min of uptime
	CHECK_TRUE(is_dive_frame(mock_kineis->last_packet, mock_kineis->last_size_bits));
	serv.stop();
}

TEST(Dx1ArgosTx, DiveFrameCutBeforeAirIsNotSpent) {
	// Accepted by the module (TxStarted) but the turtle dived before RF: no
	// TxComplete, so neither the surfacing nor the 2-h spacing is used up.
	gate(SwsTxMode::FORCE);
	mock_kineis->test_set_current_modulation(KineisModulation::LDA2);
	ArgosTxService serv(*mock_kineis);
	serv.start();
	prime();
	CHECK_TRUE(surface_and_ping2());
	CHECK_TRUE(is_dive_frame(mock_kineis->last_packet, mock_kineis->last_size_bits));
	tx_started();                         // accepted, never completed: the next dive cuts it
	CHECK_TRUE(surface_and_ping2(1200));  // 20 min later, well inside 2 h
	CHECK_TRUE(is_dive_frame(mock_kineis->last_packet, mock_kineis->last_size_bits));
	serv.stop();
}

TEST(Dx1ArgosTx, DiveFrameRefusedByTheModuleIsNotSpent) {
	// Spec 5.4 counts the frames that went on air (TxComplete). One the module
	// accepted and then failed (DeviceError: SMD +ERROR, KIM2 TX FAILED) radiated
	// nothing, so the next ping of the SAME surfacing carries it again.
	gate(SwsTxMode::FORCE);
	mock_kineis->test_set_current_modulation(KineisModulation::LDA2);
	ArgosTxService serv(*mock_kineis);
	serv.start();
	prime();
	dive_and_surface();
	tx_started();
	next_ping();  // ping 2: the dive frame
	CHECK_TRUE(is_dive_frame(mock_kineis->last_packet, mock_kineis->last_size_bits));
	tx_started();
	mock_kineis->notify(KineisEventDeviceError({}));  // first strike: 60 s backoff
	const unsigned int sends = mock_kineis->send_count;
	for (unsigned int i = 0; i < 10 && mock_kineis->send_count == sends; i++) {
		set_time(t + 60000ULL);
		system_scheduler->run();
	}
	CHECK(mock_kineis->send_count > sends);
	CHECK_TRUE(is_dive_frame(mock_kineis->last_packet, mock_kineis->last_size_bits));
	serv.stop();
}

TEST(Dx1ArgosTx, DiveFrameCutByADiveIgnoresAForeignCompletion) {
	// A dive frame accepted, then cut by a dive (service_cancel, no TxComplete).
	// A DTE $SATDP chain left running shares the device: its completion while
	// underwater is not the dive frame going on air, so the next surfacing,
	// 20 min later, still gets one.
	gate(SwsTxMode::FORCE);
	mock_kineis->test_set_current_modulation(KineisModulation::LDA2);
	ArgosTxService serv(*mock_kineis);
	serv.start();
	prime();
	CHECK_TRUE(surface_and_ping2());
	CHECK_TRUE(is_dive_frame(mock_kineis->last_packet, mock_kineis->last_size_bits));
	tx_started();  // accepted, then the dive cuts it
	uw(true);
	tx_done();  // a $SATDP frame's completion, not a TX of ours
	set_time(t + 1200000ULL);
	uw(false);
	system_scheduler->run();  // ping 1
	tx_started();
	const unsigned int sends = mock_kineis->send_count;
	next_ping();
	CHECK_EQUAL(sends + 1U, mock_kineis->send_count);
	CHECK_TRUE(is_dive_frame(mock_kineis->last_packet, mock_kineis->last_size_bits));
	serv.stop();
}

TEST(Dx1ArgosTx, ServiceRestartForgetsTheSurfacing) {
	// Ping 1 accepted, then a re-entry into Operational (service_term/init) before
	// any new surfacing: whatever goes next is not a dive frame.
	gate(SwsTxMode::FORCE);
	mock_kineis->test_set_current_modulation(KineisModulation::LDA2);
	ArgosTxService serv(*mock_kineis);
	serv.start();
	prime();
	dive_and_surface();
	tx_started();
	serv.stop();
	serv.start();
	unsigned int sends = mock_kineis->send_count, dives = 0;
	for (unsigned int i = 0; i < 10; i++) {
		next_ping();
		tx_started();
		if (mock_kineis->send_count != sends) {
			sends = mock_kineis->send_count;
			if (is_dive_frame(mock_kineis->last_packet, mock_kineis->last_size_bits)) dives++;
		}
	}
	CHECK_EQUAL(0U, dives);
	serv.stop();
}

TEST(Dx1ArgosTx, RestartForgetsTheDiveFrameInFlight) {
	// A dive frame accepted, then cut by an exit from Operational (no TxComplete).
	// Back in Operational, a SATTX sent from configuration completes and the
	// device tells every listener: that is not the dive frame going on air.
	gate(SwsTxMode::FORCE);
	mock_kineis->test_set_current_modulation(KineisModulation::LDA2);
	ArgosTxService serv(*mock_kineis);
	serv.start();
	prime();
	CHECK_TRUE(surface_and_ping2());
	CHECK_TRUE(is_dive_frame(mock_kineis->last_packet, mock_kineis->last_size_bits));
	tx_started();  // accepted, then the exit cuts it
	serv.stop();
	serv.start();
	tx_done();                            // the SATTX's completion, not a TX of ours
	CHECK_TRUE(surface_and_ping2(1200));  // 20 min later, well inside 2 h
	CHECK_TRUE(is_dive_frame(mock_kineis->last_packet, mock_kineis->last_size_bits));
	serv.stop();
}

TEST(Dx1ArgosTx, PositionFirstUntilTheDiveFrameIsOverdue) {
	gate(SwsTxMode::FORCE);
	mock_kineis->test_set_current_modulation(KineisModulation::LDA2);
	ArgosTxService serv(*mock_kineis);
	serv.start();
	prime();
	// Never sent since boot: overdue, so it goes ahead of the cached re-send.
	uw(true);
	set_time(t + 1200000ULL);
	cache_fix();
	uw(false);
	system_scheduler->run();
	tx_started();
	next_ping();
	CHECK_TRUE(is_dive_frame(mock_kineis->last_packet, mock_kineis->last_size_bits));
	tx_started();
	tx_done();
	// 3 h later, a fresh fix in the cache: position first.
	uw(true);
	set_time(t + 3ULL * 3600000ULL);
	cache_fix();
	uw(false);
	system_scheduler->run();
	tx_started();
	next_ping();
	CHECK_EQUAL(ArgosPacketBuilder::SHORT_PACKET_BITS, mock_kineis->last_size_bits);  // the cached Short
	tx_started();
	tx_done();
	// 6 h after the last dive frame: overdue again, ahead of the cached Short.
	uw(true);
	set_time(t + 3ULL * 3600000ULL);
	cache_fix();
	uw(false);
	system_scheduler->run();
	tx_started();
	next_ping();
	CHECK_TRUE(is_dive_frame(mock_kineis->last_packet, mock_kineis->last_size_bits));
	serv.stop();
}

TEST(Dx1ArgosTx, FirstDiveFrameAfterBootIsOverdue) {
	// None on air since boot: overdue whatever the uptime, so it goes ahead of a
	// fresh cached fix. Ping 2 comes 3 h after boot, past the 2-h spacing and
	// short of the 6 h that would make it overdue anyway.
	gate(SwsTxMode::FORCE);
	mock_kineis->test_set_current_modulation(KineisModulation::LDA2);
	boot();
	ArgosTxService serv(*mock_kineis);
	serv.start();
	prime();
	cache_fix();  // fixed at that surfacing: 3 h old at ping 2, still fresh
	uw(true);
	set_time(t + 3ULL * 3600000ULL);
	uw(false);
	system_scheduler->run();
	tx_started();
	next_ping();
	const uint64_t up = PMU::get_timestamp_ms();
	CHECK(up > Dx1Dive::MIN_SPACING_MS && up < Dx1Dive::OVERDUE_MS);
	CHECK_TRUE(is_dive_frame(mock_kineis->last_packet, mock_kineis->last_size_bits));
	tx_started();
	next_ping();  // the cached Short it went ahead of
	CHECK_EQUAL(ArgosPacketBuilder::SHORT_PACKET_BITS, mock_kineis->last_size_bits);
	serv.stop();
}

namespace {
/// What process_doppler_burst() asks a receiver for: a CloudLocate MEAS20
/// snapshot and/or a degraded PVT. Static, so a failed CHECK cannot leave
/// gps_device dangling (teardown resets the pointer).
class Dx1HeldGps : public GPSDevice {
public:
	bool raw_ok = false;
	bool degraded_ok = false;
	GNSSRawMeasurement raw;
	void power_on(const GPSNavSettings &) override {}
	void power_off() override {}
	bool has_raw_measurement() const override { return raw_ok; }
	GNSSRawMeasurement get_raw_measurement() const override { return raw; }
	bool has_degraded_pvt() const override { return degraded_ok; }
	GNSSData get_degraded_pvt() const override {
		GNSSData d{};
		d.valid = 1;
		d.fixType = 3;
		d.lat = -21.09;
		d.lon = 55.24;
		d.hAcc = 80000;
		d.numSV = 5;
		return d;
	}
};
Dx1HeldGps dx1_held_gps;
}  // namespace

TEST(Dx1ArgosTx, CloudLocateAndDegradedFixGoBeforeAnOverdueDiveFrame) {
	// Spec 5.2-5.3: an overdue dive frame (here: none since boot) goes ahead of
	// the cached-position re-send, NEVER ahead of CloudLocate or a degraded fix.
	gate(SwsTxMode::FORCE);
	mock_kineis->test_set_current_modulation(KineisModulation::LDA2);
	ArgosTxService serv(*mock_kineis);
	serv.start();
	prime();  // the next surfacing closes a dive whose start was seen
	gps_device = &dx1_held_gps;

	// 1. A datable MEAS20 snapshot that fits the LDA2 module: CloudLocate first.
	fake_config_store->write_param(ParamID::GNSS_FASTLOC_MODE, (unsigned int)BaseFastlocMode::CLOUDLOCATE);
	fake_config_store->write_param(ParamID::GNSS_CLOUDLOCATE_FORMAT, (unsigned int)BaseCloudLocateFormat::MEAS20);
	dx1_held_gps.raw = GNSSRawMeasurement();
	dx1_held_gps.raw.has_meas20 = true;
	dx1_held_gps.raw_ok = true;
	dx1_held_gps.degraded_ok = false;
	uw(true);
	set_time(t + 1200000ULL);
	dx1_held_gps.raw.capture_time = (uint32_t)(t / 1000ULL);  // at the surfacing: 60 s old at ping 2
	uw(false);
	system_scheduler->run();  // ping 1: the prewarmed Doppler
	tx_started();
	next_ping();  // ping 2
	CHECK_FALSE(is_dive_frame(mock_kineis->last_packet, mock_kineis->last_size_bits));
	CHECK_EQUAL(ArgosPacketBuilder::cloudlocate_packet_bits((uint8_t)BaseCloudLocateFormat::MEAS20),
	            mock_kineis->last_size_bits);
	CHECK_EQUAL(7U, extract_bits(mock_kineis->last_packet, 0, 3));  // header 111: CloudLocate
	tx_started();
	tx_done();

	// 2. A degraded PVT (progressive fastloc, header 010) goes first too.
	fake_config_store->write_param(ParamID::GNSS_FASTLOC_MODE, (unsigned int)BaseFastlocMode::DEGRADED_PVT);
	dx1_held_gps.raw_ok = false;
	dx1_held_gps.degraded_ok = true;
	CHECK_TRUE(surface_and_ping2());
	CHECK_FALSE(is_dive_frame(mock_kineis->last_packet, mock_kineis->last_size_bits));
	CHECK_EQUAL(ArgosPacketBuilder::FASTLOC_PACKET_BITS, mock_kineis->last_size_bits);
	CHECK_EQUAL(2U, extract_bits(mock_kineis->last_packet, 0, 3));
	tx_started();
	tx_done();

	// 3. Nothing held: the overdue dive frame was due all along (not vacuous).
	dx1_held_gps.degraded_ok = false;
	CHECK_TRUE(surface_and_ping2());
	CHECK_TRUE(is_dive_frame(mock_kineis->last_packet, mock_kineis->last_size_bits));
	gps_device = nullptr;
	serv.stop();
}

TEST(Dx1ArgosTx, DiveFrameContentIsWhatTheTrackerSays) {
	// The frame the service actually sends, field by field, on a GNSS clock:
	// dive 1 (600 s, start unseen), 90 s at the surface, dive 2 of 9000 s, then
	// ping 2 one minute into the surface.
	gate(SwsTxMode::FORCE);
	gnss_clock();
	mock_kineis->test_set_current_modulation(KineisModulation::LDA2);
	ArgosTxService serv(*mock_kineis);
	serv.start();
	const int64_t t0_s = (int64_t)(t / 1000ULL);
	prime(600, 90);
	CHECK_TRUE(surface_and_ping2(9000));
	const KineisPacket p = mock_kineis->last_packet;
	CHECK_TRUE(is_dive_frame(p, mock_kineis->last_size_bits));
	const int64_t utc = (int64_t)(t / 1000ULL);
	CHECK_EQUAL(ArgosPacketBuilder::convert_battery_voltage(3700), extract_bits(p, 8, 7));
	CHECK_EQUAL(1U, extract_bits(p, 22, 2));                                 // DEND: normal
	CHECK_EQUAL(2U, extract_bits(p, 32, 9));                                 // SURF_SEQ: the 2nd real surfacing
	CHECK_EQUAL(APB::dx1_em_encode(9000, 4, 3), extract_bits(p, 41, 7));     // D_LAST
	CHECK_EQUAL(0U, extract_bits(p, 48, 7));                                 // D_H1: start unseen
	CHECK_EQUAL(APB::dx1_em_encode(90, 4, 3), extract_bits(p, 76, 7));       // S_LAST
	CHECK_EQUAL((unsigned int)((utc / 7200) & 1), extract_bits(p, 183, 1));  // PHASE from the RTC
	// Bin 0 = [E - 2 h, E): observed from the first event, 90 s of it dry.
	const int64_t e = (utc / 7200) * 7200;
	const int64_t from = std::max(e - 7200, t0_s);
	const int64_t dry_from = std::max(t0_s + 600, e - 7200), dry_to = std::min(t0_s + 690, e);
	const uint32_t dry = (uint32_t)std::max<int64_t>(0, dry_to - dry_from);
	CHECK_EQUAL(APB::dx1_bin_code(dry, (uint32_t)(e - from)), extract_bits(p, 111, 6));
	serv.stop();
}

TEST(Dx1ArgosTx, UntrustedClockSendsNoTimeline) {
	// The same day on a virtual (NONE) and on a RESTORED clock: dives are known,
	// every bin and PHASE are not.
	for (RtcSource src : { RtcSource::NONE, RtcSource::RESTORED }) {
		gate(SwsTxMode::FORCE);
		fake_rtc->note_source(src);
		mock_kineis->test_set_current_modulation(KineisModulation::LDA2);
		DiveBehaviourTracker::reset();
		ArgosTxService serv(*mock_kineis);
		serv.start();
		prime(600, 90);
		CHECK_TRUE(surface_and_ping2(9000));
		const KineisPacket p = mock_kineis->last_packet;
		CHECK_TRUE(is_dive_frame(p, mock_kineis->last_size_bits));
		CHECK_EQUAL(APB::dx1_em_encode(9000, 4, 3), extract_bits(p, 41, 7));
		for (unsigned int k = 0; k < 12; k++)
			CHECK_EQUAL(0U, extract_bits(p, 111 + 6 * k, 6));
		CHECK_EQUAL(0U, extract_bits(p, 183, 1));
		serv.stop();
		set_time(t + 3ULL * 3600000ULL);  // the next case starts past the 2-h spacing
	}
}

TEST(Dx1ArgosTx, ClockTrustedAtTheSurfaceCountsForThatFrame) {
	// First surfacing after a reset on a set but untrusted clock: the dive start
	// was unseen and no bin is known, so ping 2 says nothing. A PVT then sets the
	// RTC (its hAcc fails the filter: no fix, the Doppler phase goes on) with no UW
	// event before ping 3: the frame reads the clock again and the 2.5-h dive
	// fills bin 0.
	gate(SwsTxMode::FORCE);
	mock_kineis->test_set_current_modulation(KineisModulation::LDA2);
	ArgosTxService serv(*mock_kineis);
	serv.start();
	const int64_t t0_s = (int64_t)(t / 1000ULL);
	CHECK_TRUE(surface_and_ping2(9000));
	CHECK_EQUAL(32U, mock_kineis->last_size_bits);  // nothing to say yet: the compact Doppler
	gnss_clock();
	tx_started();
	next_ping();
	const KineisPacket p = mock_kineis->last_packet;
	CHECK_TRUE(is_dive_frame(p, mock_kineis->last_size_bits));
	CHECK_EQUAL(0U, extract_bits(p, 41, 7));  // D_LAST: start unseen, bin 0 alone carries the frame
	// Bin 0 = [E - 2 h, E): observed from the first event, all of it underwater.
	const int64_t e = ((int64_t)(t / 1000ULL) / 7200) * 7200;
	CHECK_EQUAL(APB::dx1_bin_code(0, (uint32_t)(e - t0_s)), extract_bits(p, 111, 6));
	serv.stop();
}

TEST(Dx1ArgosTx, ClockStepAtTheSurfaceClearsTheBinsOfThatFrame) {
	// An operator clock (RTCW) 10 min slow: a 6000-s dive and 90 s at the surface
	// fill the bin that becomes bin 1, then a 9000-s dive. Before ping 2 the first
	// PVT steps the RTC 10 min, with no UW event in between: the frame reads the
	// clock first, so the step clears the bins of the old clock and only the
	// episode in progress is credited, through the new offset.
	gate(SwsTxMode::FORCE);
	fake_rtc->note_source(RtcSource::OPERATOR);
	mock_kineis->test_set_current_modulation(KineisModulation::LDA2);
	ArgosTxService serv(*mock_kineis);
	serv.start();
	prime(6000, 90);
	dive_and_surface(9000);
	tx_started();
	tx_done();
	set_time(t + 60000ULL);
	fake_rtc->incrementtime(600);  // the RTC steps, the uptime does not
	gnss_clock();
	system_scheduler->run();  // ping 2
	const KineisPacket p = mock_kineis->last_packet;
	CHECK_TRUE(is_dive_frame(p, mock_kineis->last_size_bits));
	CHECK_EQUAL(APB::dx1_em_encode(9000, 4, 3), extract_bits(p, 41, 7));  // D_LAST
	CHECK_EQUAL(1U, extract_bits(p, 111, 6));                             // bin 0: inside the dive, 0 % dry
	CHECK_EQUAL(0U, extract_bits(p, 117, 6));                             // bin 1: seen on the old clock only, cleared
	serv.stop();
}

TEST(Dx1ArgosTx, DiveFrameCarriesTheLastUnderwaterTemperature) {
	gate(SwsTxMode::FORCE);
	mock_kineis->test_set_current_modulation(KineisModulation::LDA2);
	ArgosTxService serv(*mock_kineis);
	serv.start();
	prime();
	uw(true);
	SWSAnalogService::Status st{};
	st.is_underwater = true;
	SWSAnalogService::test_set_status(st);
	ServiceEvent e;
	e.event_source = ServiceIdentifier::UW_SENSOR;
	e.event_originator_unique_id = 0x12345678;
	e.event_type = ServiceEventType::SERVICE_INACTIVE;  // end of an underwater SWS sample
	const unsigned int reads0 = g_pmu_die_temp_reads;
	g_pmu_die_temp_ok = true;
	g_pmu_die_temp_q4 = 112;  // 28.0 °C
	set_time(t + 61000ULL);
	ServiceManager::notify_peer_event(e);
	CHECK_EQUAL(reads0 + 1U, g_pmu_die_temp_reads);
	g_pmu_die_temp_q4 = 96;  // 24.0 °C, 30 s later: rate-limited, not read
	set_time(t + 30000ULL);
	ServiceManager::notify_peer_event(e);
	CHECK_EQUAL(reads0 + 1U, g_pmu_die_temp_reads);
	set_time(t + 31000ULL);  // 61 s after the first read
	ServiceManager::notify_peer_event(e);
	CHECK_EQUAL(reads0 + 2U, g_pmu_die_temp_reads);
	st.is_underwater = false;  // the sample that saw the surface never reads
	SWSAnalogService::test_set_status(st);
	set_time(t + 70000ULL);
	ServiceManager::notify_peer_event(e);
	CHECK_EQUAL(reads0 + 2U, g_pmu_die_temp_reads);
	set_time(t + 1200000ULL);
	uw(false);
	system_scheduler->run();
	tx_started();
	next_ping();
	CHECK_TRUE(is_dive_frame(mock_kineis->last_packet, mock_kineis->last_size_bits));
	CHECK_EQUAL(APB::dx1_temp_code(true, 96), extract_bits(mock_kineis->last_packet, 104, 7));
	g_pmu_die_temp_q4 = 100;
	serv.stop();
}

TEST(Dx1ArgosTx, NoTemperatureReadWithTheGateClosed) {
	gate(SwsTxMode::OFF);
	mock_kineis->test_set_current_modulation(KineisModulation::LDA2);
	ArgosTxService serv(*mock_kineis);
	serv.start();
	uw(true);
	SWSAnalogService::Status st{};
	st.is_underwater = true;
	SWSAnalogService::test_set_status(st);
	ServiceEvent e;
	e.event_source = ServiceIdentifier::UW_SENSOR;
	e.event_originator_unique_id = 0x12345678;
	e.event_type = ServiceEventType::SERVICE_INACTIVE;
	const unsigned int reads0 = g_pmu_die_temp_reads;
	for (unsigned int i = 0; i < 5; i++) {
		set_time(t + 61000ULL);
		ServiceManager::notify_peer_event(e);
	}
	CHECK_EQUAL(reads0, g_pmu_die_temp_reads);
	serv.stop();
}

#endif  // DX1_DOPPLER_EXT_BUILD

TEST_GROUP(Dx1Gate){};

TEST(Dx1Gate, EffectiveTruthTable) {
	// The only combinations that emit DX1, written out rather than derived
	// from the function under test. No release gate since 2026-11: the build
	// (DIVE_INFO_MESSAGE + SWS analog) and UNP26 AUTO or FORCE decide.
	struct Row {
		unsigned int mode;
		bool uw, build;
	};
	const Row on[] = {
		{ SwsTxMode::FORCE, true, true },
		{ SwsTxMode::AUTO, true, true },
	};
	unsigned int n_on = 0;
	for (unsigned int mode = 0; mode <= 3; mode++) {
		for (int uw = 0; uw < 2; uw++) {
			for (int build = 0; build < 2; build++) {
				bool expect = false;
				for (const Row &r : on)
					if (r.mode == mode && r.uw == (bool)uw && r.build == (bool)build) expect = true;
				CHECK_EQUAL(expect, dx1_effective(mode, uw, build));
				n_on += expect ? 1U : 0U;
			}
		}
	}
	CHECK_EQUAL(2U, n_on);
	CHECK_FALSE(dx1_effective(0xFFFFFFFFU, true, true));  // garbage stored value = OFF
	static_assert(dx1_effective(SwsTxMode::AUTO, true, true), "AUTO emits on a dive-info build");
	static_assert(!dx1_effective(SwsTxMode::AUTO, true, false), "nothing without DIVE_INFO_MESSAGE");
	static_assert(SwsTxMode::OFF == 0 && SwsTxMode::AUTO == 1 && SwsTxMode::FORCE == 2, "DTE values are frozen");
}
