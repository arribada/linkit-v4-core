#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

#include "mock_kineis_device.hpp"
#include "fake_rtc.hpp"
#include "fake_config_store.hpp"
#include "fake_timer.hpp"
#include "fake_battery_mon.hpp"
#include "dte_protocol.hpp"
#include "binascii.hpp"
#include "timeutils.hpp"
#include "scheduler.hpp"
#include "argos_tx_service.hpp"
#include "argos_packet_builder.hpp"
#include "crc8.hpp"
#include "messages.hpp"
#include "rate_limiter.hpp"
#include "hauled_mode_service.hpp"

using namespace std::string_literals;

extern Timer *system_timer;
extern ConfigurationStore *configuration_store;
extern Scheduler *system_scheduler;
extern RTC *rtc;
extern BatteryMonitor *battery_monitor;


// ArgosTxService::DEVICE_ERROR_BACKOFF_MAX_MS is private; mirror its value
// here so the suspension test can assert "parked beyond any backoff".
static constexpr unsigned int DEVICE_ERROR_BACKOFF_MAX_MS_FOR_TEST = 600000U;

TEST_GROUP(ArgosTxService) {
	FakeBatteryMonitor *fake_battery_monitor;
	FakeConfigurationStore *fake_config_store;
	MockKineisDevice *mock_kineis;
	FakeRTC *fake_rtc;
	FakeTimer *fake_timer;
	unsigned int txco_warmup = 5U;

	void setup() {
		// Start every test from a clean global ServiceManager. Its static state
		// (registered services, id counter, cooldown/cycle bookkeeping) otherwise
		// persists across tests in the same process; a leftover registration or
		// deferred-TX continuation from a prior test could fire into this test's
		// scheduler->run() and trip a spurious mock failure (flaky full-suite).
		ServiceManager::reset();
		fake_battery_monitor = new FakeBatteryMonitor;
		battery_monitor = fake_battery_monitor;
		mock_kineis = new MockKineisDevice;
		fake_config_store = new FakeConfigurationStore;
		configuration_store = fake_config_store;
		configuration_store->init();
		fake_rtc = new FakeRTC;
		rtc = fake_rtc;
		// Set RTC time so service_is_time_known() returns true
		fake_rtc->settime(1580083200);  // 27/01/2020 00:00:00
		fake_timer = new FakeTimer;
		system_timer = fake_timer;  // linux_timer;
		system_scheduler = new Scheduler(system_timer);
		fake_timer->start();

		// Setup PP_MIN_ELEVATION for 5.0
		double min_elevation = 5.0;
		fake_config_store->write_param(ParamID::PP_MIN_ELEVATION, min_elevation);

		// Disable cooldown by default in tests (individual tests can override)
		unsigned int no_cooldown = 0U;
		fake_config_store->write_param(ParamID::MIN_SURFACE_CYCLE_INTERVAL_S, no_cooldown);
	}

	void teardown() {
		delete system_scheduler;
		delete fake_timer;
		delete fake_rtc;
		delete fake_config_store;
		delete mock_kineis;
		delete fake_battery_monitor;
		// Reset globals so the next test group can't dereference dangling
		// pointers (DTEHandler dereferences them via shared static state
		// before its own setup re-initialises everything → segfault).
		system_scheduler = nullptr;
		system_timer = nullptr;
		rtc = nullptr;
		configuration_store = nullptr;
		battery_monitor = nullptr;
	}

	GPSLogEntry make_gps_location(bool is_valid = true, double longitude = 0, double latitude = 0, std::time_t t = 0,
	                              bool is_3d_fix = false, int32_t hMSL = 0, int32_t gSpeed = 0, uint16_t batt = 4200) {
		GPSLogEntry log{};  // Zero-init to avoid UB on unset fields (e.g. headMot)
		log.info.valid = is_valid;
		log.info.lon = longitude;
		log.info.lat = latitude;
		log.info.schedTime = t;
		log.info.fixType = is_3d_fix ? 3 : 2;
		log.info.hMSL = hMSL;
		log.info.batt_voltage = batt;
		log.info.gSpeed = gSpeed;

		if (t == 0) t = rtc->gettime();

		uint16_t year;
		uint8_t month, day, hour, min, sec;
		convert_datetime_to_epoch(t, year, month, day, hour, min, sec);
		log.header.year = log.info.year = year;
		log.header.month = log.info.month = month;
		log.header.day = log.info.day = day;
		log.header.hours = log.info.hour = hour;
		log.header.minutes = log.info.min = min;
		log.header.seconds = log.info.sec = sec;
		log.info.schedTime = t;
		return log;
	}

	void inject_gps_active() {
		ServiceEvent e;
		e.event_type = ServiceEventType::SERVICE_ACTIVE;
		e.event_source = ServiceIdentifier::GNSS_SENSOR;
		e.event_originator_unique_id = 0x12345678;
		ServiceManager::notify_peer_event(e);
	}

	void inject_gps_inactive() {
		ServiceEvent e;
		e.event_type = ServiceEventType::SERVICE_INACTIVE;
		e.event_source = ServiceIdentifier::GNSS_SENSOR;
		e.event_originator_unique_id = 0x12345678;
		ServiceManager::notify_peer_event(e);
	}

	void inject_sensor_data(ServiceSensorData data, ServiceIdentifier service) {
		ServiceEvent e;
		e.event_source = service;
		e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
		e.event_data = data;
		e.event_originator_unique_id = 0x12345678;
		ServiceManager::notify_peer_event(e);
	}

	void inject_gps_location(bool is_valid = true, double longitude = 0, double latitude = 0, std::time_t t = 0,
	                         bool is_3d_fix = false, int32_t hMSL = 0, int32_t gSpeed = 0, uint16_t batt = 4200) {
		ServiceEvent e;
		GPSLogEntry log = make_gps_location(is_valid, longitude, latitude, t, is_3d_fix, hMSL, gSpeed, batt);

		e.event_source = ServiceIdentifier::GNSS_SENSOR;
		e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
		e.event_data = log;
		e.event_originator_unique_id = 0x12345678;
		ServiceManager::notify_peer_event(e);
		configuration_store->notify_gps_location(log);
	}

	void inject_gps_nofix(std::time_t t = 0) {
		ServiceEvent e;
		GPSLogEntry log = make_gps_location(false, 0, 0, t);
		log.info.event_type = GPSEventType::NO_FIX;

		e.event_source = ServiceIdentifier::GNSS_SENSOR;
		e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
		e.event_data = log;
		e.event_originator_unique_id = 0x12345678;
		ServiceManager::notify_peer_event(e);
		configuration_store->notify_gps_location(log);
	}

	void notify_underwater_state(bool state) {
		ServiceEvent e;
		e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
		e.event_data = state;
		e.event_source = ServiceIdentifier::UW_SENSOR;
		e.event_originator_unique_id = 0x12345678;
		ServiceManager::notify_peer_event(e);
	}

	// Plainest possible Argos configuration: LEGACY, a fix in the pile, no
	// jitter, no time-sync burst, no low-battery gating -- so that a schedule
	// that moves says something about the code under test and nothing else.
	// Used by the device-error tests at the end of this file.
	void inject_cloudlocate_ready() {
		ServiceEvent e;
		e.event_source = ServiceIdentifier::GNSS_SENSOR;
		e.event_type = ServiceEventType::GNSS_CLOUDLOCATE_READY;
		e.event_originator_unique_id = 0x12345678;
		ServiceManager::notify_peer_event(e);
	}

	// SURFACING_BURST with a short, readable Doppler cadence: first message
	// immediate, then 5 s, 15 s, 25 s... Used by the device-error burst tests,
	// where the whole point is that 5 s is what the backoff has to override.
	void configure_surfacing_burst(BaseArgosMode mode = BaseArgosMode::SURFACING_BURST) {
		fake_config_store->write_param(ParamID::ARGOS_MODE, mode);
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
	}

	void configure_plain_legacy(unsigned int tr_nom_s = 10) {
		fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::LEGACY);
		fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);
		fake_config_store->write_param(ParamID::ARGOS_HEXID, (unsigned int)0x01234567U);
		fake_config_store->write_param(ParamID::GNSS_EN, true);
		fake_config_store->write_param(ParamID::LB_EN, false);
		fake_config_store->write_param(ParamID::TR_NOM, tr_nom_s);
		fake_config_store->write_param(ParamID::ARGOS_TX_JITTER_EN, false);
		fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, false);
	}
};

TEST(ArgosTxService, DepthPileFillsAndEmpties) {
	DepthPile<GPSLogEntry> dp;
	std::vector<GPSLogEntry *> v;
	GPSLogEntry e;

	// Load up full depth pile with burst count of 1
	for (unsigned int i = 0; i < 24; i++) {
		e.info.day = i;
		dp.store(e, 1);
	}

	// Should have 24 eligible entries
	CHECK_EQUAL(24, dp.eligible());

	// Retrieving the latest entry (time sync burst) does NOT consume a burst
	// slot — deliberate deviation from v3 (see DepthPile::retrieve_latest)
	v = dp.retrieve_latest();
	CHECK_EQUAL(1, v.size());
	CHECK_EQUAL(24, dp.eligible());
	CHECK_EQUAL(23, v.at(0)->info.day);  // Should be most recent

	// Retrieve entire depth pile (in blocks of 3 — LDA2 long packet carries 3
	// fixes to leave room for the firmware-embedded CRC8 at byte 23)
	for (unsigned int i = 0; i < 8; i++) {
		v = dp.retrieve(24);
		CHECK_EQUAL(3, v.size());
		CHECK_EQUAL(21 - (i * 3), dp.eligible());
		for (unsigned j = 0; j < 3; j++) {
			CHECK_EQUAL(24 - ((i + 1) * 3) + j, v.at(j)->info.day);
		}
	}

	// Check depth pile returns empty vector
	v = dp.retrieve_latest();
	CHECK_EQUAL(0, v.size());
	v = dp.retrieve(24);
	CHECK_EQUAL(0, v.size());
}

TEST(ArgosTxService, DepthPile1) {
	DepthPile<GPSLogEntry> dp;
	std::vector<GPSLogEntry *> v;
	GPSLogEntry e;

	// Load up full depth pile with burst count of 1
	for (unsigned int i = 0; i < 24; i++) {
		e.info.day = i;
		dp.store(e, 1);
	}

	// Should have 24 eligible entries
	CHECK_EQUAL(24, dp.eligible());

	// Retrieving the latest entry (time sync burst) does NOT consume a burst
	// slot — deliberate deviation from v3 (see DepthPile::retrieve_latest)
	v = dp.retrieve_latest();
	CHECK_EQUAL(1, v.size());
	CHECK_EQUAL(24, dp.eligible());
	CHECK_EQUAL(23, v.at(0)->info.day);  // Should be most recent

	// Retrieve depth pile and should be most recent
	v = dp.retrieve(1);
	CHECK_EQUAL(1, v.size());
	CHECK_EQUAL(23, v.at(0)->info.day);

	// Check depth pile returns empty vector
	v = dp.retrieve_latest();
	CHECK_EQUAL(0, v.size());
	v = dp.retrieve(1);
	CHECK_EQUAL(0, v.size());
}

TEST(ArgosTxService, DutyCycleCalculation) {
	unsigned int mask;
	mask = 0xFFFFFF;
	for (unsigned int i = 0; i < 24; i++)
		CHECK_EQUAL((bool)((mask >> (23 - i)) & 1), ArgosTxScheduler::is_in_duty_cycle(i * 3600 * 1000, mask));
	mask = 0xEEEEEE;
	for (unsigned int i = 0; i < 24; i++)
		CHECK_EQUAL((bool)((mask >> (23 - i)) & 1), ArgosTxScheduler::is_in_duty_cycle(i * 3600 * 1000, mask));
	mask = 0;
	for (unsigned int i = 0; i < 24; i++)
		CHECK_EQUAL((bool)((mask >> (23 - i)) & 1), ArgosTxScheduler::is_in_duty_cycle(i * 3600 * 1000, mask));
	mask = 0x123456;
	for (unsigned int i = 0; i < 24; i++)
		CHECK_EQUAL((bool)((mask >> (23 - i)) & 1), ArgosTxScheduler::is_in_duty_cycle(i * 3600 * 1000, mask));
}

TEST(ArgosTxService, SchedulerLegacyNoJitter) {
	ArgosTxScheduler sched;
	ArgosConfig config;

	config.argos_tx_jitter_en = false;
	config.tx_interval_s = 10;
	unsigned int result;

	result = sched.schedule_legacy(config, 0);
	CHECK_EQUAL(0U, result);
	sched.notify_tx_complete();
	result = sched.schedule_legacy(config, 10);
	CHECK_EQUAL(0U, result);
	sched.notify_tx_complete();
	result = sched.schedule_legacy(config, 20);
	CHECK_EQUAL(0U, result);
	sched.notify_tx_complete();
	result = sched.schedule_legacy(config, 30);
	CHECK_EQUAL(0U, result);
	sched.notify_tx_complete();
	result = sched.schedule_legacy(config, 35);
	CHECK_EQUAL(5000U, result);
	sched.notify_tx_complete();
	result = sched.schedule_legacy(config, 59);
	CHECK_EQUAL(1000U, result);
	sched.notify_tx_complete();
}

// Sealed-device guard (audit 2026-07): the periodic 24 h search must not
// infinite-loop on a corrupt tx_interval_s==0 (period_ms==0 would spin the WDT
// -> reboot -> x3 -> factory reset). Exercised through the public schedule_legacy
// path: the guard throws, schedule_legacy catches it and returns INVALID_SCHEDULE.
// The fact that this test RETURNS at all proves there is no infinite loop.
TEST(ArgosTxService, SchedulerZeroIntervalNoHang) {
	ArgosTxScheduler sched;
	ArgosConfig config;
	config.argos_tx_jitter_en = false;

	config.tx_interval_s = 0;  // corrupt/degenerate period
	CHECK_EQUAL(ArgosTxScheduler::INVALID_SCHEDULE, sched.schedule_legacy(config, 100));

	// A valid interval still schedules normally on the same object (no regression)
	config.tx_interval_s = 60;
	sched.notify_tx_complete();
	CHECK_EQUAL(0U, sched.schedule_legacy(config, 100));
}

TEST(ArgosTxService, SchedulerLegacyNoJitterWithEarliestTxSet) {
	ArgosTxScheduler sched;
	ArgosConfig config;

	config.argos_tx_jitter_en = false;
	config.tx_interval_s = 10;
	unsigned int result;

	result = sched.schedule_legacy(config, 0);
	CHECK_EQUAL(0U, result);
	sched.notify_tx_complete();
	result = sched.schedule_legacy(config, 10);
	CHECK_EQUAL(0U, result);
	sched.notify_tx_complete();
	result = sched.schedule_legacy(config, 20);
	CHECK_EQUAL(0U, result);
	sched.notify_tx_complete();
	result = sched.schedule_legacy(config, 30);
	CHECK_EQUAL(0U, result);
	sched.notify_tx_complete();
	sched.set_earliest_schedule(41);
	result = sched.schedule_legacy(config, 35);
	CHECK_EQUAL(6000U, result);
	sched.notify_tx_complete();
}

TEST(ArgosTxService, SchedulerLegacyWithJitter) {
	ArgosTxScheduler sched;
	ArgosConfig config;

	config.argos_tx_jitter_en = true;
	config.tx_interval_s = 10;
	unsigned int result;

	result = sched.schedule_legacy(config, 0);
	CHECK_TRUE(result <= 5000);
	sched.notify_tx_complete();
	result = sched.schedule_legacy(config, 0);
	CHECK_TRUE(result <= 15000 && result >= 10000);
	sched.notify_tx_complete();
}

TEST(ArgosTxService, SchedulerDutyCycleNoJitter) {
	ArgosTxScheduler sched;
	ArgosConfig config;

	config.argos_tx_jitter_en = false;
	config.tx_interval_s = 10;
	config.duty_cycle = 0x1;

	unsigned int result = sched.schedule_duty_cycle(config, 0);
	CHECK_EQUAL(23 * 3600 * 1000, result);
	sched.notify_tx_complete();
	result = sched.schedule_duty_cycle(config, 23 * 3600);
	CHECK_EQUAL(10000U, result);
	sched.notify_tx_complete();
}

TEST(ArgosTxService, BuildLongCertificationPacket) {
	unsigned int size_bits;
	// LONG_PACKET_BYTES is 24 (LDA2 frame); certification truncates to that size.
	std::string x = ArgosPacketBuilder::build_certification_packet(
	    "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF", size_bits);
	CHECK_EQUAL(ArgosPacketBuilder::LDA2_FRAME_BITS, size_bits);
	CHECK_EQUAL(ArgosPacketBuilder::LDA2_FRAME_BYTES, x.size());
	CHECK_EQUAL("\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF"s, x);
}

TEST(ArgosTxService, BuildShortCertificationPacket) {
	unsigned int size_bits;
	// SHORT_PACKET_BYTES = 12, so input must be ≤12 bytes (24 hex chars)
	std::string x = ArgosPacketBuilder::build_certification_packet("FFFFFFFFFFFFFFFFFFFF", size_bits);  // 10 bytes
	CHECK_EQUAL(96, size_bits);                                           // SHORT_PACKET_BITS = 96
	CHECK_EQUAL("\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\x00\x00"s, x);  // Padded to 12 bytes
}

TEST(ArgosTxService, BuildShortGNSSPacket) {
	unsigned int size_bits;
	GPSLogEntry e = make_gps_location(1, 12.3, 44.4, 1652105502);
	std::vector<GPSLogEntry *> v({ &e });
	std::string x = ArgosPacketBuilder::build_gnss_packet(v, false, false, BaseDeltaTimeLoc::DELTA_T_10MIN, size_bits);
	CHECK_EQUAL("097166C6600781E00003FE58"s, Binascii::hexlify(x));

	x = ArgosPacketBuilder::build_gnss_packet(v, true, false, BaseDeltaTimeLoc::DELTA_T_10MIN, size_bits);
	CHECK_EQUAL("097166C6600781E00403FE58"s, Binascii::hexlify(x));

	x = ArgosPacketBuilder::build_gnss_packet(v, false, true, BaseDeltaTimeLoc::DELTA_T_10MIN, size_bits);
	CHECK_EQUAL("097166C6600781E00003FE5C"s, Binascii::hexlify(x));

	x = ArgosPacketBuilder::build_gnss_packet(v, true, true, BaseDeltaTimeLoc::DELTA_T_10MIN, size_bits);
	CHECK_EQUAL("097166C6600781E00403FE5C"s, Binascii::hexlify(x));
}


// Helper for LDA2 long-packet: verify size, prefix data, and self-consistent CRC8 at byte 23.
static void check_lda2_long_packet(const std::string &packet, const std::string &expected_data_prefix) {
	CHECK_EQUAL(ArgosPacketBuilder::LDA2_FRAME_BYTES, packet.size());
	std::string hex = Binascii::hexlify(packet);
	std::string prefix = hex.substr(0, expected_data_prefix.size());
	CHECK_EQUAL(expected_data_prefix, prefix);
	unsigned char expected_crc = CRC8::checksum(packet, ArgosPacketBuilder::LDA2_DATA_BITS);
	CHECK_EQUAL((unsigned int)expected_crc, (unsigned int)(unsigned char)packet[23]);
}

TEST(ArgosTxService, BuildLongGNSSPacket) {
	unsigned int size_bits;
	GPSLogEntry e = make_gps_location(1, 12.3, 44.4, 1652105502);
	// Long packet: header 000 (Type 0, shared with Short Packet, disambiguated by size 24B vs 12B).
	// Packs up to 3 fixes (down from 4) to leave 8 bits for CRC at byte 23.
	std::vector<GPSLogEntry *> v({ &e, &e });
	std::string x = ArgosPacketBuilder::build_gnss_packet(v, false, false, BaseDeltaTimeLoc::DELTA_T_10MIN, size_bits);
	CHECK_EQUAL(ArgosPacketBuilder::LDA2_FRAME_BITS, size_bits);
	check_lda2_long_packet(x, "097166C6600781E002584D8CC00F03C7FFFFFFFFFF");
	v = { &e, &e, &e };
	x = ArgosPacketBuilder::build_gnss_packet(v, false, false, BaseDeltaTimeLoc::DELTA_T_10MIN, size_bits);
	check_lda2_long_packet(x, "097166C6600781E002584D8CC00F03C1B19801E078");
	// 4 fixes provided → 4th is silently dropped (LDA2 budget = 3 fixes after CRC).
	v = { &e, &e, &e, &e };
	x = ArgosPacketBuilder::build_gnss_packet(v, false, false, BaseDeltaTimeLoc::DELTA_T_10MIN, size_bits);
	check_lda2_long_packet(x, "097166C6600781E002584D8CC00F03C1B19801E078");
	x = ArgosPacketBuilder::build_gnss_packet(v, true, false, BaseDeltaTimeLoc::DELTA_T_10MIN, size_bits);
	check_lda2_long_packet(x, "097166C6600781E006584D8CC00F03C1B19801E078");
	x = ArgosPacketBuilder::build_gnss_packet(v, false, true, BaseDeltaTimeLoc::DELTA_T_10MIN, size_bits);
	check_lda2_long_packet(x, "097166C6600781E0025C4D8CC00F03C1B19801E078");
	x = ArgosPacketBuilder::build_gnss_packet(v, true, true, BaseDeltaTimeLoc::DELTA_T_10MIN, size_bits);
	check_lda2_long_packet(x, "097166C6600781E0065C4D8CC00F03C1B19801E078");
	x = ArgosPacketBuilder::build_gnss_packet(v, true, true, BaseDeltaTimeLoc::DELTA_T_30MIN, size_bits);
	check_lda2_long_packet(x, "097166C6600781E0065CCD8CC00F03C1B19801E078");
}

// Legacy LONG format: whatever the position spacing (uniform or NOT), the frame
// keeps the plain pre-v2 layout — bits 168+ (bytes 21/22) stay zero, dating is
// the uniform delta_time_loc grid. (The 2026-06 v2 skip-field extension was
// removed 2026-07: decoders expect the plain legacy frame.)
TEST(ArgosTxService, BuildLongGNSSPacketNonUniformKeepsLegacyLayout) {
	unsigned int size_bits;
	GPSLogEntry newest = make_gps_location(1, 12.3, 44.4, 1652106702);
	GPSLogEntry mid = make_gps_location(1, 12.3, 44.4, 1652105502);     // newest - 1200 (2*600)
	GPSLogEntry oldest = make_gps_location(1, 12.3, 44.4, 1652104902);  // mid    -  600 (1*600)
	std::vector<GPSLogEntry *> v({ &oldest, &mid, &newest });           // chronological asc; builder reverses
	std::string x = ArgosPacketBuilder::build_gnss_packet(v, false, false, BaseDeltaTimeLoc::DELTA_T_10MIN, size_bits);
	CHECK_EQUAL(ArgosPacketBuilder::LDA2_FRAME_BYTES, x.size());
	// Non-uniform spacing must NOT set any trailing format/skip bits
	CHECK_EQUAL(0x00, (unsigned int)(unsigned char)x[21]);
	CHECK_EQUAL(0x00, (unsigned int)(unsigned char)x[22]);
	// CRC8 self-consistent
	unsigned char expected_crc = CRC8::checksum(x, ArgosPacketBuilder::LDA2_DATA_BITS);
	CHECK_EQUAL((unsigned int)expected_crc, (unsigned int)(unsigned char)x[23]);
}

// Legacy LONG format: uniformly-spaced positions — same zero trailing region.
TEST(ArgosTxService, BuildLongGNSSPacketUniformKeepsLegacyLayout) {
	unsigned int size_bits;
	GPSLogEntry a = make_gps_location(1, 12.3, 44.4, 1652106702);
	GPSLogEntry b = make_gps_location(1, 12.3, 44.4, 1652106102);  // a - 600 (1*600)
	GPSLogEntry c = make_gps_location(1, 12.3, 44.4, 1652105502);  // b - 600 (1*600)
	std::vector<GPSLogEntry *> v({ &c, &b, &a });                  // chronological asc; builder reverses to a,b,c
	std::string x = ArgosPacketBuilder::build_gnss_packet(v, false, false, BaseDeltaTimeLoc::DELTA_T_10MIN, size_bits);
	CHECK_EQUAL(0x00, (unsigned int)(unsigned char)x[21]);
	CHECK_EQUAL(0x00, (unsigned int)(unsigned char)x[22]);
}

// First-message gate (non-RSPB): without a valid GPS fix yet this session, the
// 2026-08 -- INTENT REVERSED. The first-message lock used to hold back EVERY
// transmission until a GPS fix had landed since power-up, presence heartbeat
// included. A beacon that restarted and never regained a fix (dead receiver,
// blocked sky) therefore vanished from the screens for good. In LEGACY the
// beacon transmits on its period and the ground segment reconstructs the
// position: its own clock plays no part in that computation, so nothing
// justifies silencing it. RSPB was already exempt.
//
// This test now asserts the opposite of its ancestor: the 0xFF heartbeat MUST
// go out even if no fix ever lands.
// === DUTY_CYCLE mode ========================================================
// This mode had ZERO coverage in this file: 20 tests exercise LEGACY, 12
// SURFACING_BURST, 6 PASS_PREDICTION, 2 DOPPLER, and none DUTY_CYCLE -- even
// though it has two distinct scheduling branches (with and without GNSS) and
// the most notorious configuration trap in the firmware.

// GNSS branch: a fix in the pile, an hour mask that enables the current hour.
// 1652056200 is 2022-05-09 00:30 UTC, i.e. hour 0.
//
// NOTE the mask convention, which is the reverse of the obvious one: ARP18 is
// MSB-first, so bit 23 (0x800000) is hour 0 and bit 0 (0x000001) is hour 23 --
// see ArgosTxScheduler::is_in_duty_cycle(). Writing 0x000001 to mean "hour 0"
// enables 23:00 instead, and the beacon looks dead for a whole day.
TEST(ArgosTxService, DutyCycleWithGnssSchedulesInsideEnabledHour) {
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::DUTY_CYCLE);
	fake_config_store->write_param(ParamID::DUTY_CYCLE, (unsigned int)0x800000);  // hour 0 only (MSB-first)
	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, (unsigned int)0x01234567U);
	fake_config_store->write_param(ParamID::GNSS_EN, true);
	fake_config_store->write_param(ParamID::LB_EN, false);
	fake_config_store->write_param(ParamID::TR_NOM, (unsigned int)10);
	fake_config_store->write_param(ParamID::ARGOS_TX_JITTER_EN, false);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, false);

	ArgosTxService serv(*mock_kineis);
	std::time_t t = 1652056200000;  // ms, 00:30 UTC
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	inject_gps_location(true, 11.8768, -33.8232, t / 1000, true);

	// Inside an enabled hour the scheduler does not park the TX for 23 h
	CHECK_COMPARE(serv.get_last_schedule(), <, 3600U * 1000U);

	mock()
	    .expectOneCall("send")
	    .onObject(mock_kineis)
	    .withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2)
	    .ignoreOtherParameters();
	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock_kineis->notify(KineisEventTxComplete({}));
	mock().checkExpectations();
}

// Position-less branch: with GNSS_EN=0, DUTY_CYCLE is one of only two modes
// that still know how to schedule (the other is LEGACY). It sends Doppler.
TEST(ArgosTxService, DutyCycleWithoutGnssStillSchedules) {
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::DUTY_CYCLE);
	fake_config_store->write_param(ParamID::DUTY_CYCLE, (unsigned int)0x800000);  // hour 0 (MSB-first)
	fake_config_store->write_param(ParamID::ARGOS_HEXID, (unsigned int)0x01234567U);
	fake_config_store->write_param(ParamID::GNSS_EN, false);
	fake_config_store->write_param(ParamID::LB_EN, false);
	fake_config_store->write_param(ParamID::TR_NOM, (unsigned int)10);
	fake_config_store->write_param(ParamID::ARGOS_TX_JITTER_EN, false);

	ArgosTxService serv(*mock_kineis);
	std::time_t t = 1652056200000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// No GPS fix is injected at all -- that is the point of this branch
	CHECK_COMPARE(serv.get_last_schedule(), !=, Service::SCHEDULE_DISABLED);
	CHECK_COMPARE(serv.get_last_schedule(), <, 3600U * 1000U);
	mock().checkExpectations();
}

// The trap: ARP18 defaults to 0x000000, so selecting ARGOS_MODE=DUTY_CYCLE and
// nothing else enables no hour at all and the beacon never transmits. Since
// 2026-08 report_duty_cycle_schedule() says so in the log instead of going
// silently dark -- but it does not override the mask, and this test pins that:
// the schedule really is disabled, and it is the mask that does it.
TEST(ArgosTxService, DutyCycleZeroMaskDisablesTxRatherThanTransmitting) {
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::DUTY_CYCLE);
	fake_config_store->write_param(ParamID::DUTY_CYCLE, (unsigned int)0x000000);  // factory default
	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, (unsigned int)0x01234567U);
	fake_config_store->write_param(ParamID::GNSS_EN, true);
	fake_config_store->write_param(ParamID::LB_EN, false);
	fake_config_store->write_param(ParamID::TR_NOM, (unsigned int)10);
	fake_config_store->write_param(ParamID::ARGOS_TX_JITTER_EN, false);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, false);

	ArgosTxService serv(*mock_kineis);
	std::time_t t = 1652056200000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	inject_gps_location(true, 11.8768, -33.8232, t / 1000, true);

	// No hour enabled -> nothing scheduled. No send() is expected, and the mock
	// would fail the test if one happened.
	CHECK_EQUAL(Service::SCHEDULE_DISABLED, serv.get_last_schedule());
	mock().checkExpectations();
}

// === Certification TX =======================================================
// CERT_TX_ENABLE is a bench mode: a fixed payload repeated on a fixed period.
// Its scheduling branch had been commented out since 747a70a7 (2025-07), so the
// flag switched the service on and silenced GNSS while nothing ever scheduled a
// certification burst. These tests pin the restored behaviour.

// The branch sits before the ARGOS_MODE dispatch on purpose: certification runs
// with ARGOS_MODE=OFF, which is precisely the case that used to return
// SCHEDULE_DISABLED and do nothing at all.
TEST(ArgosTxService, CertificationTxFiresWithArgosModeOff) {
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::OFF);
	fake_config_store->write_param(ParamID::CERT_TX_ENABLE, true);
	fake_config_store->write_param(ParamID::CERT_TX_PAYLOAD, std::string("AABBCCDD"));
	fake_config_store->write_param(ParamID::CERT_TX_MODULATION, BaseArgosModulation::A2);
	fake_config_store->write_param(ParamID::CERT_TX_REPETITION, (unsigned int)60);
	fake_config_store->write_param(ParamID::LB_EN, false);

	ArgosTxService serv(*mock_kineis);
	std::time_t t = 1652105502000;  // ms
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// First TX is immediate so the operator sees something at once
	CHECK_EQUAL(0U, serv.get_last_schedule());

	// 4 payload bytes <= SHORT_PACKET_BYTES, so a 96-bit short packet
	mock()
	    .expectOneCall("send")
	    .onObject(mock_kineis)
	    .withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2)
	    .withUnsignedIntParameter("size_bits", 96);
	system_scheduler->run();
	mock_kineis->notify(KineisEventTxComplete({}));
	mock().checkExpectations();
}

// After the first burst the period is CERT_TX_REPETITION seconds. The original
// code passed milliseconds to ArgosTxScheduler::schedule_at(), which takes a
// time_t in SECONDS -- it would have armed the next burst 1000x too late.
TEST(ArgosTxService, CertificationTxRepeatsAtConfiguredPeriod) {
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::OFF);
	fake_config_store->write_param(ParamID::CERT_TX_ENABLE, true);
	fake_config_store->write_param(ParamID::CERT_TX_PAYLOAD, std::string("AABBCCDD"));
	fake_config_store->write_param(ParamID::CERT_TX_MODULATION, BaseArgosModulation::A2);
	fake_config_store->write_param(ParamID::CERT_TX_REPETITION, (unsigned int)60);
	fake_config_store->write_param(ParamID::LB_EN, false);

	ArgosTxService serv(*mock_kineis);
	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	mock()
	    .expectOneCall("send")
	    .onObject(mock_kineis)
	    .withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2)
	    .withUnsignedIntParameter("size_bits", 96);
	system_scheduler->run();
	mock_kineis->notify(KineisEventTxComplete({}));

	// 60 s, in milliseconds -- not 60 000 s
	CHECK_EQUAL(60000U, serv.get_last_schedule());
	mock().checkExpectations();
}

// CTP04 accepts up to 0xFFFFFFFF. `repetition * 1000` overflows unsigned int
// above 4 294 967 s, which would turn a 49-day period into a near-immediate
// retransmit -- the exact inversion of the setting. Same clamp as the rate
// limiter uses a few lines above in service_next_schedule_in_ms.
TEST(ArgosTxService, CertificationTxPeriodDoesNotWrap) {
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::OFF);
	fake_config_store->write_param(ParamID::CERT_TX_ENABLE, true);
	fake_config_store->write_param(ParamID::CERT_TX_PAYLOAD, std::string("AABBCCDD"));
	fake_config_store->write_param(ParamID::CERT_TX_MODULATION, BaseArgosModulation::A2);
	fake_config_store->write_param(ParamID::CERT_TX_REPETITION, (unsigned int)0xFFFFFFFFU);
	fake_config_store->write_param(ParamID::LB_EN, false);

	ArgosTxService serv(*mock_kineis);
	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	mock()
	    .expectOneCall("send")
	    .onObject(mock_kineis)
	    .withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2)
	    .withUnsignedIntParameter("size_bits", 96);
	system_scheduler->run();
	mock_kineis->notify(KineisEventTxComplete({}));

	// Clamped, and emphatically not a near-immediate retransmit
	CHECK_EQUAL(0xFFFFFFFEU, serv.get_last_schedule());
	mock().checkExpectations();
}

// Every other burst programmes the module before transmitting; this one did not,
// so a certification frame could go out under whatever RCONF the previous
// ordinary TX had left loaded.
TEST(ArgosTxService, CertificationTxProgramsTheRequestedModulation) {
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::OFF);
	fake_config_store->write_param(ParamID::CERT_TX_ENABLE, true);
	fake_config_store->write_param(ParamID::CERT_TX_PAYLOAD, std::string("AABBCCDD"));
	fake_config_store->write_param(ParamID::CERT_TX_MODULATION, BaseArgosModulation::LDK);
	fake_config_store->write_param(ParamID::CERT_TX_REPETITION, (unsigned int)60);
	fake_config_store->write_param(ParamID::LB_EN, false);
	fake_config_store->write_param(ParamID::ARGOS_RADIOCONF_LDK, std::string("03921FB104B92859209B18ABD009DE96"));

	ArgosTxService serv(*mock_kineis);
	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	// The mock starts on LDA2, so a switch to LDK must actually be programmed
	mock_kineis->test_set_current_modulation(KineisModulation::LDA2);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	mock()
	    .expectOneCall("switch_modulation")
	    .onObject(mock_kineis)
	    .withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDK)
	    .withStringParameter("rconf", "03921FB104B92859209B18ABD009DE96");
	mock()
	    .expectOneCall("send")
	    .onObject(mock_kineis)
	    .withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDK)
	    .withUnsignedIntParameter("size_bits", 96);
	system_scheduler->run();
	mock_kineis->notify(KineisEventTxComplete({}));
	mock().checkExpectations();
}

// Regression guard for the restored branch: with CERT_TX_ENABLE off (the
// factory default), ARGOS_MODE=OFF must still disable the service outright.
TEST(ArgosTxService, CertificationDisabledLeavesArgosModeOffDisabled) {
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::OFF);
	fake_config_store->write_param(ParamID::CERT_TX_ENABLE, false);
	fake_config_store->write_param(ParamID::LB_EN, false);

	ArgosTxService serv(*mock_kineis);
	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();
	CHECK_EQUAL(Service::SCHEDULE_DISABLED, serv.get_last_schedule());
	mock().checkExpectations();
}

TEST(ArgosTxService, LegacyNoFixHeartbeatSentWithoutAnyFix) {
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::LEGACY);
	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, (unsigned int)0x01234567U);
	fake_config_store->write_param(ParamID::LB_EN, false);
	fake_config_store->write_param(ParamID::TR_NOM, (unsigned int)10);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, false);

	ArgosTxService serv(*mock_kineis);
	std::time_t t = 1652105502000;  // ms
	fake_rtc->settime(t / 1000);    // clock IS set (e.g. DTE/pseudo-RTC) but never by GPS
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// NO_FIX only, no real fix at all: the heartbeat must still be transmitted
	// (SHORT, 96 bits) -- the beacon signals that it is there.
	inject_gps_nofix(t / 1000);
	mock()
	    .expectOneCall("send")
	    .onObject(mock_kineis)
	    .withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2)
	    .withUnsignedIntParameter("size_bits", 96);
	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock_kineis->notify(KineisEventTxComplete({}));
	mock().checkExpectations();
}

// After a real GPS fix corrects the clock (unlocks TX), a subsequent no-fix
// cycle transmits the NO_FIX 0xFF heartbeat.
TEST(ArgosTxService, LegacyNoFixHeartbeatAfterFix) {
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::LEGACY);
	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, (unsigned int)0x01234567U);
	fake_config_store->write_param(ParamID::LB_EN, false);
	fake_config_store->write_param(ParamID::TR_NOM, (unsigned int)10);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, false);

	ArgosTxService serv(*mock_kineis);
	std::time_t t = 1652105502000;  // ms
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// Real fix unlocks TX and is transmitted (SHORT, 96 bits)
	inject_gps_location(1, 11.8768, -33.8232, t / 1000);
	mock()
	    .expectOneCall("send")
	    .onObject(mock_kineis)
	    .withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2)
	    .withUnsignedIntParameter("size_bits", 96);
	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock_kineis->notify(KineisEventTxComplete({}));

	// Now a no-fix cycle -> NO_FIX heartbeat cached and transmitted as SHORT 0xFF
	inject_gps_nofix(t / 1000);
	mock()
	    .expectOneCall("send")
	    .onObject(mock_kineis)
	    .withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2)
	    .withUnsignedIntParameter("size_bits", 96);
	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock().checkExpectations();
}

// v3 grid fillers: the NO_FIX entry stays in the pile even after a real fix
// arrives (no purge in LEGACY/DUTY_CYCLE/PASS_PREDICTION) so it keeps its slot
// on the delta_time_loc grid — the next packet is a LONG (192-bit) carrying
// the real fix plus the 0xFF filler.
TEST(ArgosTxService, LegacyNoFixFillerKeptAfterRealFix) {
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::LEGACY);
	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_4);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, (unsigned int)0x01234567U);
	fake_config_store->write_param(ParamID::LB_EN, false);
	fake_config_store->write_param(ParamID::TR_NOM, (unsigned int)10);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, false);

	ArgosTxService serv(*mock_kineis);
	std::time_t t = 1652105502000;  // ms
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// GPS cycle 1: no fix -> the heartbeat is cached AND transmitted (SHORT,
	// 96 bits). Before 2026-08 the first-message lock forced silence here; it
	// was removed, a beacon with no fix must keep signalling itself.
	inject_gps_nofix(t / 1000);
	mock()
	    .expectOneCall("send")
	    .onObject(mock_kineis)
	    .withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2)
	    .withUnsignedIntParameter("size_bits", 96);
	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock_kineis->notify(KineisEventTxComplete({}));
	mock().checkExpectations();

	// GPS cycle 2: real fix -> unlocks TX; the FIX purge keeps the NO_FIX filler
	// (include_no_fix=false in planned modes) -> pile = [NO_FIX, FIX] ->
	// LONG packet (192 bits) with the fix + 0xFF filler
	inject_gps_location(1, 11.8768, -33.8232, t / 1000);
	mock()
	    .expectOneCall("send")
	    .onObject(mock_kineis)
	    .withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2)
	    .withUnsignedIntParameter("size_bits", 192);
	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock_kineis->notify(KineisEventTxComplete({}));
	mock().checkExpectations();
}


TEST(ArgosTxService, TxCounterIncrements) {
	ArgosTxService serv(*mock_kineis);

	unsigned int counter;
	counter = configuration_store->read_param<unsigned int>(ParamID::TX_COUNTER);
	CHECK_EQUAL(0U, counter);

	double frequency = 900.22;
	fake_config_store->write_param(ParamID::ARGOS_FREQ, frequency);
	BaseArgosPower power = BaseArgosPower::POWER_1000_MW;
	fake_config_store->write_param(ParamID::ARGOS_POWER, power);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	mock_kineis->notify(KineisEventTxComplete({}));

	counter = configuration_store->read_param<unsigned int>(ParamID::TX_COUNTER);
	CHECK_EQUAL(1, counter);

	mock_kineis->notify(KineisEventTxComplete({}));

	counter = configuration_store->read_param<unsigned int>(ParamID::TX_COUNTER);
	CHECK_EQUAL(2, counter);
}

TEST(ArgosTxService, TimeSyncBurstPosFix) {
	double frequency = 900.22;
	BaseArgosMode mode = BaseArgosMode::LEGACY;
	BaseArgosPower power = BaseArgosPower::POWER_1000_MW;
	BaseDepthPile depth_pile = BaseDepthPile::DEPTH_PILE_1;
	unsigned int argos_hexid = 0x01234567U;
	unsigned int lb_threshold = 0U;
	bool lb_en = false;
	unsigned int tr_nom = 10;

	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, depth_pile);
	fake_config_store->write_param(ParamID::ARGOS_FREQ, frequency);
	fake_config_store->write_param(ParamID::ARGOS_MODE, mode);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, argos_hexid);
	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::ARGOS_POWER, power);
	fake_config_store->write_param(ParamID::TR_NOM, tr_nom);

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// First TX is time sync burst
	mock()
	    .expectOneCall("send")
	    .onObject(mock_kineis)
	    .withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2)
	    .withUnsignedIntParameter("size_bits", 96);

	inject_gps_location(1, 11.8768, -33.8232, t);
	system_scheduler->run();

	mock_kineis->notify(KineisEventTxComplete({}));

	mock().expectOneCall("power_off_immediate").onObject(mock_kineis);
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	serv.stop();

	// No time sync should be scheduled now
	bool time_sync_en = false;
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, time_sync_en);
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();
	inject_gps_location(1, 11.8768, -33.8232, t);
	system_scheduler->run();
}

TEST(ArgosTxService, TimeSyncBurstNoPosFix) {
	// Time-sync burst fires only after the GNSS has updated the clock this
	// session (like v3). A NO_FIX cycle does not update the clock, so the
	// first-message gate holds TX -> silence.
	double frequency = 900.22;
	BaseArgosMode mode = BaseArgosMode::LEGACY;
	BaseArgosPower power = BaseArgosPower::POWER_1000_MW;
	BaseDepthPile depth_pile = BaseDepthPile::DEPTH_PILE_1;
	unsigned int argos_hexid = 0x01234567U;
	unsigned int lb_threshold = 0U;
	bool lb_en = false;
	unsigned int tr_nom = 10;

	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, depth_pile);
	fake_config_store->write_param(ParamID::ARGOS_FREQ, frequency);
	fake_config_store->write_param(ParamID::ARGOS_MODE, mode);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, argos_hexid);
	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::ARGOS_POWER, power);
	fake_config_store->write_param(ParamID::TR_NOM, tr_nom);

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// Only NO_FIX (clock not GNSS-updated) -> time-sync held by the gate -> silence
	inject_gps_nofix(t);
	system_scheduler->run();
	mock().checkExpectations();  // no "send"

	mock().expectOneCall("power_off_immediate").onObject(mock_kineis);
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	serv.stop();
}

TEST(ArgosTxService, TimeSyncBurstNoPosOrTimeFix) {
	// No time fix and no GNSS fix: both the RTC-unknown gate and the
	// first-message gate hold TX -> silence.
	double frequency = 900.22;
	BaseArgosMode mode = BaseArgosMode::LEGACY;
	BaseArgosPower power = BaseArgosPower::POWER_1000_MW;
	BaseDepthPile depth_pile = BaseDepthPile::DEPTH_PILE_1;
	unsigned int argos_hexid = 0x01234567U;
	unsigned int lb_threshold = 0U;
	bool lb_en = false;
	unsigned int tr_nom = 10;

	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, depth_pile);
	fake_config_store->write_param(ParamID::ARGOS_FREQ, frequency);
	fake_config_store->write_param(ParamID::ARGOS_MODE, mode);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, argos_hexid);
	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::ARGOS_POWER, power);
	fake_config_store->write_param(ParamID::TR_NOM, tr_nom);

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 0;

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// Without a valid time or GPS fix, nothing is transmitted
	inject_gps_nofix(t);
	system_scheduler->run();
	mock().checkExpectations();  // silence
}

TEST(ArgosTxService, LegacyTxServiceInv) {
	double frequency = 900.22;
	BaseArgosMode mode = BaseArgosMode::LEGACY;
	BaseArgosPower power = BaseArgosPower::POWER_1000_MW;
	BaseDepthPile depth_pile = BaseDepthPile::DEPTH_PILE_4;
	unsigned int argos_hexid = 0x01234567U;
	unsigned int lb_threshold = 0U;
	bool lb_en = false;
	unsigned int tr_nom = 10;
	bool time_sync_en = false;

	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, depth_pile);
	fake_config_store->write_param(ParamID::ARGOS_FREQ, frequency);
	fake_config_store->write_param(ParamID::ARGOS_MODE, mode);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, argos_hexid);
	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::ARGOS_POWER, power);
	fake_config_store->write_param(ParamID::TR_NOM, tr_nom);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, time_sync_en);

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1665137726000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	inject_gps_location(1, -2.117964, 51.376382, t / 1000, false, 0, 162, 4424);
	//inject_gps_location(1, 11.8768, -33.8232, t);
	//inject_gps_location(1, 11.8768, -33.8232, t);
	//inject_gps_location(1, 11.8768, -33.8232, t);
	system_scheduler->run();

	// Subsequent TX will be long packets
	for (unsigned int i = 0; i < 1; i++) {
		mock()
		    .expectOneCall("send")
		    .onObject(mock_kineis)
		    .withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2)
		    .withUnsignedIntParameter("size_bits", 96);

		t += serv.get_last_schedule();
		fake_rtc->settime(t / 1000);
		fake_timer->set_counter(t);
		system_scheduler->run();

		mock_kineis->notify(KineisEventTxComplete({}));
	}
}


TEST(ArgosTxService, LegacyTxLowBattery) {
	// Relies on unlimited replay: LB_NTRY_PER_MESSAGE=0 (0 = unlimited replay in
	// LEGACY/DUTY_CYCLE/PASS_PREDICTION; the LB default of 4 would otherwise
	// bound the single LB pile entry to 4 TX).
	fake_config_store->write_param(ParamID::LB_NTRY_PER_MESSAGE, 0U);
	double frequency = 900.22;
	BaseArgosMode mode = BaseArgosMode::LEGACY;
	BaseArgosPower power = BaseArgosPower::POWER_1000_MW;
	BaseDepthPile depth_pile = BaseDepthPile::DEPTH_PILE_4;
	unsigned int argos_hexid = 0x01234567U;
	unsigned int lb_threshold = 20U;
	bool lb_en = true;
	unsigned int tr_nom = 10;
	bool time_sync_en = false;

	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, depth_pile);
	fake_config_store->write_param(ParamID::ARGOS_FREQ, frequency);
	fake_config_store->write_param(ParamID::ARGOS_MODE, mode);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, argos_hexid);
	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::ARGOS_POWER, power);
	fake_config_store->write_param(ParamID::TR_NOM, tr_nom);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, time_sync_en);

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// Force LB state
	fake_config_store->set_battery_level(10U);
	fake_config_store->set_is_battery_level_low(true);
	fake_battery_monitor->set_values(10, 4200, true, false);

	inject_gps_location(1, 11.8768, -33.8232, t);
	inject_gps_location(1, 11.8768, -33.8232, t);
	inject_gps_location(1, 11.8768, -33.8232, t);
	inject_gps_location(1, 11.8768, -33.8232, t);
	system_scheduler->run();

	// Subsequent TX will be short packets (depth pile 1 in LB mode)
	for (unsigned int i = 0; i < 10; i++) {
		mock()
		    .expectOneCall("send")
		    .onObject(mock_kineis)
		    .withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2)
		    .withUnsignedIntParameter("size_bits", 96);

		t += serv.get_last_schedule();
		fake_rtc->settime(t / 1000);
		fake_timer->set_counter(t);
		system_scheduler->run();

		mock_kineis->notify(KineisEventTxComplete({}));
	}
}

TEST(ArgosTxService, LegacyTxOutOfZone) {
	double frequency = 900.22;
	BaseArgosMode mode = BaseArgosMode::LEGACY;
	BaseArgosPower power = BaseArgosPower::POWER_1000_MW;
	BaseDepthPile depth_pile = BaseDepthPile::DEPTH_PILE_4;
	unsigned int argos_hexid = 0x01234567U;
	unsigned int lb_threshold = 0U;
	bool lb_en = false;
	unsigned int tr_nom = 10;
	bool time_sync_en = false;
	bool ooz_en = true;

	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, depth_pile);
	fake_config_store->write_param(ParamID::ARGOS_FREQ, frequency);
	fake_config_store->write_param(ParamID::ARGOS_MODE, mode);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, argos_hexid);
	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::ARGOS_POWER, power);
	fake_config_store->write_param(ParamID::TR_NOM, tr_nom);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, time_sync_en);
	fake_config_store->write_param(ParamID::ZONE_ENABLE_OUT_OF_ZONE_DETECTION_MODE, ooz_en);

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	inject_gps_location(1, 11.8768, -33.8232);
	inject_gps_location(1, 11.8768, -33.8232);
	inject_gps_location(1, 11.8768, -33.8232);
	inject_gps_location(1, 11.8768, -33.8232);
	system_scheduler->run();

	// Subsequent TX will be short packets (depth pile 1 in OOZ mode)
	for (unsigned int i = 0; i < 10; i++) {
		mock()
		    .expectOneCall("send")
		    .onObject(mock_kineis)
		    .withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2)
		    .withUnsignedIntParameter("size_bits", 96);

		t += serv.get_last_schedule();
		fake_rtc->settime(t / 1000);
		fake_timer->set_counter(t);
		system_scheduler->run();

		mock_kineis->notify(KineisEventTxComplete({}));
	}
}

TEST(ArgosTxService, TxServiceCancelledByUnderwaterBeforeTx) {
	double frequency = 900.22;
	BaseArgosMode mode = BaseArgosMode::LEGACY;
	BaseArgosPower power = BaseArgosPower::POWER_1000_MW;
	BaseDepthPile depth_pile = BaseDepthPile::DEPTH_PILE_4;
	unsigned int argos_hexid = 0x01234567U;
	unsigned int lb_threshold = 0U;
	bool lb_en = false;
	unsigned int tr_nom = 10;
	bool time_sync_en = false;

	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, depth_pile);
	fake_config_store->write_param(ParamID::ARGOS_FREQ, frequency);
	fake_config_store->write_param(ParamID::ARGOS_MODE, mode);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, argos_hexid);
	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::ARGOS_POWER, power);
	fake_config_store->write_param(ParamID::TR_NOM, tr_nom);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, time_sync_en);

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	inject_gps_location(1, 11.8768, -33.8232, t);
	inject_gps_location(1, 11.8768, -33.8232, t);
	inject_gps_location(1, 11.8768, -33.8232, t);
	inject_gps_location(1, 11.8768, -33.8232, t);
	system_scheduler->run();

	CHECK_FALSE(Service::SCHEDULE_DISABLED == serv.get_last_schedule());

	// Inject UW event
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("power_off_immediate").onObject(mock_kineis);
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	notify_underwater_state(true);

	CHECK_TRUE(Service::SCHEDULE_DISABLED == serv.get_last_schedule());

	// Inject surfaced event
	notify_underwater_state(false);

	CHECK_FALSE(Service::SCHEDULE_DISABLED == serv.get_last_schedule());

	// Next schedule should be equal to dry time before TX since the last TX was deferred
	unsigned int dry_time_before_tx = fake_config_store->read_param<unsigned int>(ParamID::DRY_TIME_BEFORE_TX);
	CHECK_EQUAL(dry_time_before_tx * 1000, serv.get_last_schedule());

	// Should now transmit (TCXO warmup skipped on first TX after submerge)
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock()
	    .expectOneCall("send")
	    .onObject(mock_kineis)
	    .withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2)
	    .withUnsignedIntParameter("size_bits", 192);
	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
}

TEST(ArgosTxService, TxServiceCancelledDuringTx) {
	double frequency = 900.22;
	BaseArgosMode mode = BaseArgosMode::LEGACY;
	BaseArgosPower power = BaseArgosPower::POWER_1000_MW;
	BaseDepthPile depth_pile = BaseDepthPile::DEPTH_PILE_4;
	unsigned int argos_hexid = 0x01234567U;
	unsigned int lb_threshold = 0U;
	bool lb_en = false;
	unsigned int tr_nom = 10;
	bool time_sync_en = false;

	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, depth_pile);
	fake_config_store->write_param(ParamID::ARGOS_FREQ, frequency);
	fake_config_store->write_param(ParamID::ARGOS_MODE, mode);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, argos_hexid);
	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::ARGOS_POWER, power);
	fake_config_store->write_param(ParamID::TR_NOM, tr_nom);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, time_sync_en);

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	inject_gps_location(1, 11.8768, -33.8232, t);
	inject_gps_location(1, 11.8768, -33.8232, t);
	inject_gps_location(1, 11.8768, -33.8232, t);
	inject_gps_location(1, 11.8768, -33.8232, t);
	system_scheduler->run();

	mock()
	    .expectOneCall("send")
	    .onObject(mock_kineis)
	    .withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2)
	    .withUnsignedIntParameter("size_bits", 192);

	// TX should start
	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();

	// Inject UW event
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("power_off_immediate").onObject(mock_kineis);
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	notify_underwater_state(true);

	// Inject surfaced event
	notify_underwater_state(false);

	// Next schedule should be equal to dry time before TX since the last TX was deferred
	unsigned int dry_time_before_tx = fake_config_store->read_param<unsigned int>(ParamID::DRY_TIME_BEFORE_TX);
	CHECK_EQUAL(dry_time_before_tx * 1000, serv.get_last_schedule());

	// Should now transmit. Depth pile has 4 fixes; with max_messages=3 the second slot
	// only carries 1 fix, so the 2nd send is a SHORT (LDK 96-bit) packet.
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock()
	    .expectOneCall("send")
	    .onObject(mock_kineis)
	    .withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2)
	    .withUnsignedIntParameter("size_bits", 96);
	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	// TX complete restores TCXO warmup
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	mock_kineis->notify(KineisEventTxComplete({}));
}


TEST(ArgosTxService, LegacyTxServiceDepthPile1) {
	double frequency = 900.22;
	BaseArgosMode mode = BaseArgosMode::LEGACY;
	BaseArgosPower power = BaseArgosPower::POWER_1000_MW;
	BaseDepthPile depth_pile = BaseDepthPile::DEPTH_PILE_1;
	unsigned int argos_hexid = 0x01234567U;
	unsigned int lb_threshold = 0U;
	bool lb_en = false;
	unsigned int tr_nom = 10;
	bool time_sync_en = false;

	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, depth_pile);
	fake_config_store->write_param(ParamID::ARGOS_FREQ, frequency);
	fake_config_store->write_param(ParamID::ARGOS_MODE, mode);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, argos_hexid);
	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::ARGOS_POWER, power);
	fake_config_store->write_param(ParamID::TR_NOM, tr_nom);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, time_sync_en);

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	inject_gps_location(1, 11.8768, -33.8232, t);
	inject_gps_location(1, 11.8768, -33.8232, t);
	inject_gps_location(1, 11.8768, -33.8232, t);
	inject_gps_location(1, 11.8768, -33.8232, t);
	system_scheduler->run();

	// Subsequent TX will be short packets
	for (unsigned int i = 0; i < 10; i++) {
		mock()
		    .expectOneCall("send")
		    .onObject(mock_kineis)
		    .withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2)
		    .withUnsignedIntParameter("size_bits", 96);

		t += serv.get_last_schedule();
		fake_rtc->settime(t / 1000);
		fake_timer->set_counter(t);
		system_scheduler->run();

		mock_kineis->notify(KineisEventTxComplete({}));
	}
}

// Regression (2026-06-17): on a KIM2 whose master RCONF is LDK (128-bit budget),
// a non-adaptive LEGACY *multi-fix* GNSS burst used to build a 192-bit LONG packet
// that KIM2 send() drops silently (no event → 30 s timeout stall, every cycle).
// The service must keep the LDK master modulation but cap the burst to a single
// fix so it ships a 96-bit SHORT packet that fits. (User policy: keep master mod,
// limit the number of fixes.)
TEST(ArgosTxService, LegacyNonAdaptiveLdkMasterCapsToSingleFix) {
	double frequency = 900.22;
	BaseArgosMode mode = BaseArgosMode::LEGACY;
	BaseArgosPower power = BaseArgosPower::POWER_1000_MW;
	BaseDepthPile depth_pile = BaseDepthPile::DEPTH_PILE_4;
	unsigned int argos_hexid = 0x01234567U;
	unsigned int lb_threshold = 0U;
	bool lb_en = false;
	unsigned int tr_nom = 10;
	bool time_sync_en = false;

	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, depth_pile);
	fake_config_store->write_param(ParamID::ARGOS_FREQ, frequency);
	fake_config_store->write_param(ParamID::ARGOS_MODE, mode);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, argos_hexid);
	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::ARGOS_POWER, power);
	fake_config_store->write_param(ParamID::TR_NOM, tr_nom);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, time_sync_en);
	fake_config_store->write_param(ParamID::ARGOS_ADAPTIVE_MODULATION, false);

	// Emulate KIM2 state_init() aligning the device to its LDK master RCONF.
	mock_kineis->test_set_current_modulation(KineisModulation::LDK);

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	inject_gps_location(1, 11.8768, -33.8232, t);
	inject_gps_location(1, 11.8768, -33.8232, t);
	inject_gps_location(1, 11.8768, -33.8232, t);
	inject_gps_location(1, 11.8768, -33.8232, t);
	system_scheduler->run();

	for (unsigned int i = 0; i < 3; i++) {
		// Capped to a single fix → 96-bit SHORT packet, still in the LDK master mod.
		// (Pre-fix this was send(LDK, 192) → silent KIM2 drop.)
		mock()
		    .expectOneCall("send")
		    .onObject(mock_kineis)
		    .withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDK)
		    .withUnsignedIntParameter("size_bits", 96);

		t += serv.get_last_schedule();
		fake_rtc->settime(t / 1000);
		fake_timer->set_counter(t);
		system_scheduler->run();

		mock_kineis->notify(KineisEventTxComplete({}));
	}
}

// Regression (2026-06-17): a non-adaptive master accidentally left on VLDA4
// (24-bit budget) can't hold even a single 96-bit GNSS fix. Instead of a silent
// KIM2 oversize drop, the service must fall back to LDA2 (when its RCONF is
// provisioned) and ship the fix there. (User policy: on overflow → LDA2.)
TEST(ArgosTxService, LegacyNonAdaptiveVlda4MasterFallsBackToLda2) {
	const std::string lda2_rconf = "00112233445566778899aabbccddeeff";  // 32 hex chars

	double frequency = 900.22;
	BaseArgosMode mode = BaseArgosMode::LEGACY;
	BaseArgosPower power = BaseArgosPower::POWER_1000_MW;
	BaseDepthPile depth_pile = BaseDepthPile::DEPTH_PILE_1;
	unsigned int argos_hexid = 0x01234567U;
	unsigned int lb_threshold = 0U;
	bool lb_en = false;
	unsigned int tr_nom = 10;
	bool time_sync_en = false;

	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, depth_pile);
	fake_config_store->write_param(ParamID::ARGOS_FREQ, frequency);
	fake_config_store->write_param(ParamID::ARGOS_MODE, mode);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, argos_hexid);
	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::ARGOS_POWER, power);
	fake_config_store->write_param(ParamID::TR_NOM, tr_nom);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, time_sync_en);
	fake_config_store->write_param(ParamID::ARGOS_ADAPTIVE_MODULATION, false);
	fake_config_store->write_param(ParamID::ARGOS_RADIOCONF_LDA2, lda2_rconf);

	// Emulate KIM2 state_init() aligning the device to a (mis-set) VLDA4 master.
	mock_kineis->test_set_current_modulation(KineisModulation::VLDA4);

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	inject_gps_location(1, 11.8768, -33.8232, t);
	inject_gps_location(1, 11.8768, -33.8232, t);
	system_scheduler->run();

	// First TX: 96-bit fix doesn't fit VLDA4 → fall back to LDA2 (switch), send there.
	mock()
	    .expectOneCall("switch_modulation")
	    .onObject(mock_kineis)
	    .withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2)
	    .withStringParameter("rconf", lda2_rconf.c_str());
	mock()
	    .expectOneCall("send")
	    .onObject(mock_kineis)
	    .withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2)
	    .withUnsignedIntParameter("size_bits", 96);

	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();

	mock_kineis->notify(KineisEventTxComplete({}));
}

TEST(ArgosTxService, UnderwaterFor24HoursBeforeTx) {
	double frequency = 900.22;
	BaseArgosMode mode = BaseArgosMode::LEGACY;
	BaseArgosPower power = BaseArgosPower::POWER_1000_MW;
	BaseDepthPile depth_pile = BaseDepthPile::DEPTH_PILE_4;
	unsigned int argos_hexid = 0x01234567U;
	unsigned int lb_threshold = 0U;
	bool lb_en = false;
	unsigned int tr_nom = 10;
	bool time_sync_en = false;

	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, depth_pile);
	fake_config_store->write_param(ParamID::ARGOS_FREQ, frequency);
	fake_config_store->write_param(ParamID::ARGOS_MODE, mode);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, argos_hexid);
	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::ARGOS_POWER, power);
	fake_config_store->write_param(ParamID::TR_NOM, tr_nom);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, time_sync_en);

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	inject_gps_location(1, 11.8768, -33.8232, t);
	inject_gps_location(1, 11.8768, -33.8232, t);
	inject_gps_location(1, 11.8768, -33.8232, t);
	inject_gps_location(1, 11.8768, -33.8232, t);
	system_scheduler->run();

	// Inject UW event
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("power_off_immediate").onObject(mock_kineis);
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	notify_underwater_state(true);

	// Keep UW for 25 hours
	t += 50 * 3600 * 1000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();

	// Inject surfaced event
	notify_underwater_state(false);

	// Next schedule should be equal to dry time before TX since the last TX was deferred
	unsigned int dry_time_before_tx = fake_config_store->read_param<unsigned int>(ParamID::DRY_TIME_BEFORE_TX);
	CHECK_EQUAL(dry_time_before_tx * 1000, serv.get_last_schedule());

	// Should now transmit (TCXO warmup skipped on first TX after submerge)
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock()
	    .expectOneCall("send")
	    .onObject(mock_kineis)
	    .withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2)
	    .withUnsignedIntParameter("size_bits", 192);
	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
}

TEST(ArgosTxService, LastTxIsUpdated) {
	double frequency = 900.22;
	BaseArgosMode mode = BaseArgosMode::LEGACY;
	BaseArgosPower power = BaseArgosPower::POWER_1000_MW;
	BaseDepthPile depth_pile = BaseDepthPile::DEPTH_PILE_1;
	unsigned int argos_hexid = 0x01234567U;
	unsigned int lb_threshold = 0U;
	bool lb_en = false;
	unsigned int tr_nom = 10;

	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, depth_pile);
	fake_config_store->write_param(ParamID::ARGOS_FREQ, frequency);
	fake_config_store->write_param(ParamID::ARGOS_MODE, mode);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, argos_hexid);
	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::ARGOS_POWER, power);
	fake_config_store->write_param(ParamID::TR_NOM, tr_nom);

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// First TX is time sync burst
	mock()
	    .expectOneCall("send")
	    .onObject(mock_kineis)
	    .withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2)
	    .withUnsignedIntParameter("size_bits", 96);

	inject_gps_location(1, 11.8768, -33.8232, t);
	system_scheduler->run();

	mock_kineis->notify(KineisEventTxComplete({}));

	std::time_t last_tx = fake_config_store->read_param<std::time_t>(ParamID::LAST_TX);
	CHECK_EQUAL(1652105502U, (unsigned int)last_tx);

	mock().expectOneCall("power_off_immediate").onObject(mock_kineis);
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	serv.stop();
}

TEST(ArgosTxService, DPHardFaultPartialDepthPile) {
	DepthPile<GPSLogEntry> dp;
	std::vector<GPSLogEntry *> v;
	GPSLogEntry e;

	// Load up full depth pile with burst count of 0
	for (unsigned int i = 0; i < 6; i++) {
		e.info.day = i;
		dp.store(e, 99999999);
	}

	// Long packet now packs up to 3 fixes (down from 4) — slot span = 3.
	v = dp.retrieve((unsigned int)BaseDepthPile::DEPTH_PILE_16);
	CHECK_EQUAL(3, v.size());
	v = dp.retrieve((unsigned int)BaseDepthPile::DEPTH_PILE_16);
	CHECK_EQUAL(3, v.size());
}

TEST(ArgosTxService, UnderwaterFor24HoursDryTimeZero) {
	// Relies on the cached fix staying transmittable across the dive
	// (NTRY_PER_MESSAGE default 0 = unlimited replay).
	double frequency = 900.22;
	BaseArgosMode mode = BaseArgosMode::LEGACY;
	BaseArgosPower power = BaseArgosPower::POWER_1000_MW;
	BaseDepthPile depth_pile = BaseDepthPile::DEPTH_PILE_1;
	unsigned int argos_hexid = 0x01234567U;
	unsigned int lb_threshold = 0U;
	bool lb_en = false;
	unsigned int tr_nom = 10;
	bool time_sync_en = false;
	unsigned int dry_time_before_tx = 0;

	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, depth_pile);
	fake_config_store->write_param(ParamID::ARGOS_FREQ, frequency);
	fake_config_store->write_param(ParamID::ARGOS_MODE, mode);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, argos_hexid);
	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::ARGOS_POWER, power);
	fake_config_store->write_param(ParamID::TR_NOM, tr_nom);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, time_sync_en);
	fake_config_store->write_param(ParamID::DRY_TIME_BEFORE_TX, dry_time_before_tx);

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	inject_gps_location(1, 11.8768, -33.8232, t);

	// Do initial transmit
	mock().expectOneCall("send").onObject(mock_kineis).ignoreOtherParameters();
	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();

	mock_kineis->notify(KineisEventTxComplete({}));

	// Inject UW event
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("power_off_immediate").onObject(mock_kineis);
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	notify_underwater_state(true);

	// Keep UW for 25 hours
	t += 25 * 3600 * 1000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();

	// Inject surfaced event
	notify_underwater_state(false);

	CHECK_EQUAL(0, serv.get_last_schedule());

	// Should now transmit (TCXO warmup skipped on first TX after submerge)
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("send").onObject(mock_kineis).ignoreOtherParameters();
	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
}

TEST(ArgosTxService, SurfacingBurstDopplerPhase) {
	// Test SURFACING_BURST mode: verify progressive Doppler intervals
	BaseArgosMode mode = BaseArgosMode::SURFACING_BURST;
	unsigned int dry_time = 1;  // 1s dry time to separate scheduling from notification

	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);
	fake_config_store->write_param(ParamID::ARGOS_MODE, mode);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, 0x01234567U);
	fake_config_store->write_param(ParamID::TR_NOM, 60U);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, (bool)false);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, (bool)true);
	fake_config_store->write_param(ParamID::ARGOS_TX_JITTER_EN, (bool)false);
	fake_config_store->write_param(ParamID::DRY_TIME_BEFORE_TX, dry_time);
	fake_config_store->write_param(ParamID::SURFACING_BURST_INIT_S, 5U);
	fake_config_store->write_param(ParamID::SURFACING_BURST_STEP_S, 10U);
	fake_config_store->write_param(ParamID::SURFACING_BURST_MAX_S, 60U);

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// Go underwater
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("power_off_immediate").onObject(mock_kineis);
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	notify_underwater_state(true);

	// Advance time underwater
	t += 60000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	// Surface — first Doppler scheduled immediately (0ms)
	notify_underwater_state(false);
	CHECK_EQUAL(0U, serv.get_last_schedule());

	// Fire first Doppler TX
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock()
	    .expectOneCall("send")
	    .onObject(mock_kineis)
	    .withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2)
	    .withUnsignedIntParameter("size_bits", 24);
	system_scheduler->run();

	// Complete TX → reschedule with INIT interval (5s)
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	mock_kineis->notify(KineisEventTxComplete({}));
	CHECK_EQUAL(5000U, serv.get_last_schedule());

	// Advance and fire second Doppler
	mock()
	    .expectOneCall("send")
	    .onObject(mock_kineis)
	    .withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2)
	    .withUnsignedIntParameter("size_bits", 24);
	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();

	mock_kineis->notify(KineisEventTxComplete({}));
	// Third message: init + 1*step = 5 + 10 = 15s
	CHECK_EQUAL(15000U, serv.get_last_schedule());
}

TEST(ArgosTxService, SurfacingBurstSwitchToGNSS) {
	// Test switch from Doppler to GNSS when GPS fix arrives
	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::SURFACING_BURST);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, 0x01234567U);
	fake_config_store->write_param(ParamID::TR_NOM, 60U);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, (bool)false);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, (bool)true);
	fake_config_store->write_param(ParamID::ARGOS_TX_JITTER_EN, (bool)false);
	fake_config_store->write_param(ParamID::DRY_TIME_BEFORE_TX, 1U);
	fake_config_store->write_param(ParamID::SURFACING_BURST_INIT_S, 5U);
	fake_config_store->write_param(ParamID::SURFACING_BURST_STEP_S, 10U);
	fake_config_store->write_param(ParamID::SURFACING_BURST_MAX_S, 60U);

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// Dive
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("power_off_immediate").onObject(mock_kineis);
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	notify_underwater_state(true);

	t += 60000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	// Surface
	notify_underwater_state(false);

	// Fire first Doppler TX
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock()
	    .expectOneCall("send")
	    .onObject(mock_kineis)
	    .withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2)
	    .withUnsignedIntParameter("size_bits", 24);
	system_scheduler->run();

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	mock_kineis->notify(KineisEventTxComplete({}));

	// Inject GPS fix → should switch to GNSS phase and reschedule
	inject_gps_location(true, 11.8768, -33.8232, t / 1000);
	CHECK_FALSE(Service::SCHEDULE_DISABLED == serv.get_last_schedule());

	// Fire TX → should be GNSS short packet (96 bits), not Doppler (24 bits)
	mock()
	    .expectOneCall("send")
	    .onObject(mock_kineis)
	    .withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2)
	    .withUnsignedIntParameter("size_bits", 96);
	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
}

TEST(ArgosTxService, SurfacingBurstResetOnDive) {
	// Test that diving resets the burst state
	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::SURFACING_BURST);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, 0x01234567U);
	fake_config_store->write_param(ParamID::TR_NOM, 60U);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, (bool)false);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, (bool)true);
	fake_config_store->write_param(ParamID::ARGOS_TX_JITTER_EN, (bool)false);
	fake_config_store->write_param(ParamID::DRY_TIME_BEFORE_TX, 1U);
	fake_config_store->write_param(ParamID::SURFACING_BURST_INIT_S, 5U);
	fake_config_store->write_param(ParamID::SURFACING_BURST_STEP_S, 10U);
	fake_config_store->write_param(ParamID::SURFACING_BURST_MAX_S, 60U);

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// First dive/surface
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("power_off_immediate").onObject(mock_kineis);
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	notify_underwater_state(true);
	t += 60000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	notify_underwater_state(false);
	CHECK_EQUAL(0U, serv.get_last_schedule());  // First burst msg immediate

	// Fire first TX
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock()
	    .expectOneCall("send")
	    .onObject(mock_kineis)
	    .withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2)
	    .withUnsignedIntParameter("size_bits", 24);
	system_scheduler->run();

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	mock_kineis->notify(KineisEventTxComplete({}));
	CHECK_EQUAL(5000U, serv.get_last_schedule());

	// Dive again before second TX
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("power_off_immediate").onObject(mock_kineis);
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	notify_underwater_state(true);
	CHECK_TRUE(Service::SCHEDULE_DISABLED == serv.get_last_schedule());

	// Surface again → burst restarts from 0
	t += 60000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	notify_underwater_state(false);
	CHECK_EQUAL(0U, serv.get_last_schedule());  // Immediate again (reset)
}

TEST(ArgosTxService, SurfacingBurstFirstGnssTxImmediate) {
	// Test that the first GNSS TX after fix is immediate (delay=0), not TR_NOM
	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::SURFACING_BURST);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, 0x01234567U);
	fake_config_store->write_param(ParamID::TR_NOM, 60U);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, (bool)false);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, (bool)true);
	fake_config_store->write_param(ParamID::ARGOS_TX_JITTER_EN, (bool)false);
	fake_config_store->write_param(ParamID::DRY_TIME_BEFORE_TX, 0U);
	fake_config_store->write_param(ParamID::SURFACING_BURST_INIT_S, 5U);
	fake_config_store->write_param(ParamID::SURFACING_BURST_STEP_S, 5U);
	fake_config_store->write_param(ParamID::SURFACING_BURST_MAX_S, 60U);

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// Dive
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("power_off_immediate").onObject(mock_kineis);
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	notify_underwater_state(true);
	t += 60000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	// Surface
	notify_underwater_state(false);
	CHECK_EQUAL(0U, serv.get_last_schedule());  // Doppler #1 immediate

	// Fire Doppler TX #1
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock()
	    .expectOneCall("send")
	    .onObject(mock_kineis)
	    .withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2)
	    .withUnsignedIntParameter("size_bits", 24);
	system_scheduler->run();

	// TX complete
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	mock_kineis->notify(KineisEventTxComplete({}));

	// Advance time past the spacing guard window (surfacing_burst_init_s = 5s)
	// so the first GNSS TX after fix can fire immediately. Without this advance
	// the spacing guard correctly defers TX#1 to maintain ≥ 5 s between any
	// pair of TX (2026-05 fix: protects TCXO + CLS rate-limit).
	t += 6000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	// Inject GPS fix → switch to GNSS phase
	inject_gps_location(true, 11.8768, -33.8232, t / 1000);

	// GNSS TX #1 fires immediately now that the spacing window has elapsed.
	CHECK_EQUAL(0U, serv.get_last_schedule());

	// Fire GNSS TX
	mock()
	    .expectOneCall("send")
	    .onObject(mock_kineis)
	    .withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2)
	    .withUnsignedIntParameter("size_bits", 96);
	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
}

TEST(ArgosTxService, SurfacingBurstDopplerMaxMsg) {
	// Test that Doppler phase stops after SURFACING_BURST_MAX_MSG
	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::SURFACING_BURST);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, 0x01234567U);
	fake_config_store->write_param(ParamID::TR_NOM, 60U);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, (bool)false);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, (bool)true);
	fake_config_store->write_param(ParamID::ARGOS_TX_JITTER_EN, (bool)false);
	fake_config_store->write_param(ParamID::DRY_TIME_BEFORE_TX, 0U);
	fake_config_store->write_param(ParamID::SURFACING_BURST_INIT_S, 5U);
	fake_config_store->write_param(ParamID::SURFACING_BURST_STEP_S, 0U);
	fake_config_store->write_param(ParamID::SURFACING_BURST_MAX_S, 60U);
	fake_config_store->write_param(ParamID::SURFACING_BURST_MAX_MSG, 2U);  // Max 2 Doppler

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// Dive and surface
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("power_off_immediate").onObject(mock_kineis);
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	notify_underwater_state(true);
	t += 60000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	notify_underwater_state(false);

	// Doppler #1 (immediate)
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("send").onObject(mock_kineis).ignoreOtherParameters();
	system_scheduler->run();
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	mock_kineis->notify(KineisEventTxComplete({}));

	// Doppler #2
	mock().expectOneCall("send").onObject(mock_kineis).ignoreOtherParameters();
	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock_kineis->notify(KineisEventTxComplete({}));

	// After 2 Doppler, burst should stop (SCHEDULE_DISABLED)
	CHECK_TRUE(Service::SCHEDULE_DISABLED == serv.get_last_schedule());
}

TEST(ArgosTxService, SurfacingBurstAdaptiveModulationPreSwitch) {
	// Test that after TX complete in adaptive mode, pre-switch to VLDA4 happens
	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::SURFACING_BURST);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, 0x01234567U);
	fake_config_store->write_param(ParamID::TR_NOM, 60U);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, (bool)false);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, (bool)true);
	fake_config_store->write_param(ParamID::ARGOS_TX_JITTER_EN, (bool)false);
	fake_config_store->write_param(ParamID::DRY_TIME_BEFORE_TX, 0U);
	fake_config_store->write_param(ParamID::SURFACING_BURST_INIT_S, 5U);
	fake_config_store->write_param(ParamID::SURFACING_BURST_STEP_S, 5U);
	fake_config_store->write_param(ParamID::SURFACING_BURST_MAX_S, 60U);
	fake_config_store->write_param(ParamID::ARGOS_ADAPTIVE_MODULATION, (bool)true);
	fake_config_store->write_param(ParamID::ARGOS_RADIOCONF_VLDA4, std::string("550b4bec21009c7a7b5bebaa937cdb41"));
	fake_config_store->write_param(ParamID::ARGOS_RADIOCONF_LDK, std::string("03921fb104b92859209b18abd009de96"));

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// Dive
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("power_off_immediate").onObject(mock_kineis);
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	notify_underwater_state(true);
	t += 60000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	// Surface → Doppler VLDA4
	notify_underwater_state(false);

	// Fire Doppler TX (VLDA4 mode)
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock()
	    .expectOneCall("switch_modulation")
	    .onObject(mock_kineis)
	    .withUnsignedIntParameter("mode", (unsigned int)KineisModulation::VLDA4)
	    .withStringParameter("rconf", "550b4bec21009c7a7b5bebaa937cdb41");
	mock()
	    .expectOneCall("send")
	    .onObject(mock_kineis)
	    .withUnsignedIntParameter("mode", (unsigned int)KineisModulation::VLDA4)
	    .withUnsignedIntParameter("size_bits", 24);
	system_scheduler->run();

	// TX complete → TCXO restored, no pre-switch needed (already VLDA4)
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	mock_kineis->notify(KineisEventTxComplete({}));

	// Inject GPS fix
	inject_gps_location(true, 11.8768, -33.8232, t / 1000);

	// Fire GNSS TX → should switch to LDK
	mock()
	    .expectOneCall("switch_modulation")
	    .onObject(mock_kineis)
	    .withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDK)
	    .withStringParameter("rconf", "03921fb104b92859209b18abd009de96");
	mock()
	    .expectOneCall("send")
	    .onObject(mock_kineis)
	    .withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDK)
	    .withUnsignedIntParameter("size_bits", 96);
	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();

	// TX complete → should pre-switch back to VLDA4
	mock()
	    .expectOneCall("switch_modulation")
	    .onObject(mock_kineis)
	    .withUnsignedIntParameter("mode", (unsigned int)KineisModulation::VLDA4)
	    .withStringParameter("rconf", "550b4bec21009c7a7b5bebaa937cdb41");
	mock_kineis->notify(KineisEventTxComplete({}));

	// Verify device is back to VLDA4
	CHECK_EQUAL((unsigned int)KineisModulation::VLDA4, (unsigned int)mock_kineis->get_current_modulation());
}

TEST(ArgosTxService, DepthPileManagerSensorTimeout) {
	bool enable = true;
	BaseSensorEnableTxMode mode = BaseSensorEnableTxMode::ONESHOT;
	fake_config_store->write_param(ParamID::ALS_SENSOR_ENABLE, enable);
	fake_config_store->write_param(ParamID::ALS_SENSOR_ENABLE_TX_MODE, mode);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	DepthPileManager man;

	ServiceEvent e;
	GPSLogEntry log;
	log.info.valid = true;
	e.event_source = ServiceIdentifier::GNSS_SENSOR;
	e.event_type = ServiceEventType::SERVICE_ACTIVE;
	man.notify_peer_event(e);

	e.event_data = log;
	e.event_source = ServiceIdentifier::GNSS_SENSOR;
	e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
	man.notify_peer_event(e);

	e.event_type = ServiceEventType::SERVICE_INACTIVE;
	man.notify_peer_event(e);

	CHECK_FALSE(man.eligible());

	// Sensor timeout
	t += 2000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();

	CHECK_TRUE(man.eligible());
}

TEST(ArgosTxService, DepthPileManagerSensorRx) {
	bool enable = true;
	BaseSensorEnableTxMode mode = BaseSensorEnableTxMode::ONESHOT;
	fake_config_store->write_param(ParamID::ALS_SENSOR_ENABLE, enable);
	fake_config_store->write_param(ParamID::ALS_SENSOR_ENABLE_TX_MODE, mode);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	DepthPileManager man;

	ServiceEvent e;
	GPSLogEntry log;
	log.info.valid = true;
	ServiceSensorData sensor;
	e.event_source = ServiceIdentifier::GNSS_SENSOR;
	e.event_type = ServiceEventType::SERVICE_ACTIVE;
	man.notify_peer_event(e);

	e.event_data = log;
	e.event_source = ServiceIdentifier::GNSS_SENSOR;
	e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
	man.notify_peer_event(e);

	e.event_data = sensor;
	e.event_source = ServiceIdentifier::ALS_SENSOR;
	e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
	man.notify_peer_event(e);

	e.event_type = ServiceEventType::SERVICE_INACTIVE;
	man.notify_peer_event(e);

	CHECK_TRUE(man.eligible());
}

TEST(ArgosTxService, DepthPileManagerTestSensorValueConversion) {
	bool enable = true;
	BaseSensorEnableTxMode mode = BaseSensorEnableTxMode::ONESHOT;
	fake_config_store->write_param(ParamID::ALS_SENSOR_ENABLE, enable);
	fake_config_store->write_param(ParamID::ALS_SENSOR_ENABLE_TX_MODE, mode);
	fake_config_store->write_param(ParamID::PH_SENSOR_ENABLE, enable);
	fake_config_store->write_param(ParamID::PH_SENSOR_ENABLE_TX_MODE, mode);
	fake_config_store->write_param(ParamID::PRESSURE_SENSOR_ENABLE, enable);
	fake_config_store->write_param(ParamID::PRESSURE_SENSOR_ENABLE_TX_MODE, mode);
	fake_config_store->write_param(ParamID::SEA_TEMP_SENSOR_ENABLE, enable);
	fake_config_store->write_param(ParamID::SEA_TEMP_SENSOR_ENABLE_TX_MODE, mode);

	DepthPileManager man;

	ServiceEvent e;
	GPSLogEntry log;
	log.info.valid = true;
	ServiceSensorData sensor;
	e.event_source = ServiceIdentifier::GNSS_SENSOR;
	e.event_type = ServiceEventType::SERVICE_ACTIVE;
	man.notify_peer_event(e);

	e.event_data = log;
	e.event_source = ServiceIdentifier::GNSS_SENSOR;
	e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
	man.notify_peer_event(e);

	sensor.port[0] = 65535;
	e.event_data = sensor;
	e.event_source = ServiceIdentifier::ALS_SENSOR;
	e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
	man.notify_peer_event(e);

	sensor.port[0] = 12.4;
	sensor.port[1] = -38.0;
	e.event_data = sensor;
	e.event_source = ServiceIdentifier::PRESSURE_SENSOR;
	e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
	man.notify_peer_event(e);

	sensor.port[0] = 12.343;
	e.event_data = sensor;
	e.event_source = ServiceIdentifier::PH_SENSOR;
	e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
	man.notify_peer_event(e);

	sensor.port[0] = -12.343;
	e.event_data = sensor;
	e.event_source = ServiceIdentifier::SEA_TEMP_SENSOR;
	e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
	man.notify_peer_event(e);

	e.event_type = ServiceEventType::SERVICE_INACTIVE;
	man.notify_peer_event(e);

	ServiceSensorData *converted;
	converted = man.retrieve_sensor_single(1, ServiceIdentifier::ALS_SENSOR);
	CHECK_FALSE(nullptr == converted);
	CHECK_EQUAL(65535, (unsigned int)converted->port[0]);
	converted = man.retrieve_sensor_single(1, ServiceIdentifier::PH_SENSOR);
	CHECK_FALSE(nullptr == converted);
	CHECK_EQUAL(12343, (unsigned int)converted->port[0]);
	converted = man.retrieve_sensor_single(1, ServiceIdentifier::PRESSURE_SENSOR);
	CHECK_FALSE(nullptr == converted);
	CHECK_EQUAL(12400, (unsigned int)converted->port[0]);
	CHECK_EQUAL(200, (unsigned int)converted->port[1]);
	converted = man.retrieve_sensor_single(1, ServiceIdentifier::SEA_TEMP_SENSOR);
	CHECK_FALSE(nullptr == converted);
	// Encoding: (°C + 126) × 100, dropped from × 1000 because the 14-bit field
	// truncated silently via PACK_BITS for every value > -109.6 °C (see comment
	// in depth_pile.cpp). For -12.343 °C: (-12.343 + 126) × 100 = 11365 (uint
	// truncation of 11365.7). Previous expected value 113657 matched the old
	// × 1000 formula and is now stale.
	CHECK_EQUAL(11365, (unsigned int)converted->port[0]);
}

// Helper: verify a built LDA2 sensor packet has the right size, the expected data prefix,
// and a self-consistent CRC8 in the last byte.
static void check_lda2_sensor_packet(const std::string &packet, const std::string &expected_data_prefix) {
	CHECK_EQUAL(ArgosPacketBuilder::LDA2_FRAME_BYTES, packet.size());
	std::string hex = Binascii::hexlify(packet);
	std::string prefix = hex.substr(0, expected_data_prefix.size());
	CHECK_EQUAL(expected_data_prefix, prefix);
	unsigned char expected_crc = CRC8::checksum(packet, ArgosPacketBuilder::LDA2_DATA_BITS);
	CHECK_EQUAL((unsigned int)expected_crc, (unsigned int)(unsigned char)packet[23]);
}

TEST(ArgosTxService, BuildSensorPacketAll) {
	unsigned int size_bits;
	GPSLogEntry e = make_gps_location(1, 12.3, 44.4, 1652105502);
	ServiceSensorData als, ph, pressure, sea_temp;
	std::string x;

	als.port[0] = 10000;        // 10000 lux
	ph.port[0] = 7000;          // 7.0 ph
	pressure.port[0] = 1000;    // 1.0 bar
	pressure.port[1] = 4000;    // 0C
	sea_temp.port[0] = 126000;  // 0C

	// LDA2 sensor packets are now always 24 bytes (192-bit frame) with header 001 (Type 1)
	// at bit 0, 5-bit sensor mask at bit 78, then sensor data, then CRC8 at byte 23.
	// Mask bits (MSB-first): ALS, PH, Pressure, SeaTemp, AXL.
	x = ArgosPacketBuilder::build_sensor_packet(&e, nullptr, nullptr, nullptr, nullptr, nullptr, false, false,
	                                            size_bits);
	CHECK_EQUAL(ArgosPacketBuilder::LDA2_FRAME_BITS, size_bits);
	check_lda2_sensor_packet(x, "297166C6600781E00258");  // mask=00000
	x = ArgosPacketBuilder::build_sensor_packet(&e, &als, &ph, &pressure, &sea_temp, nullptr, false, false, size_bits);
	CHECK_EQUAL(ArgosPacketBuilder::LDA2_FRAME_BITS, size_bits);
	check_lda2_sensor_packet(x, "297166C6600781E0025BC27106D601F41F401EC3");  // mask=11110 (all 4 non-AXL)
	x = ArgosPacketBuilder::build_sensor_packet(&e, nullptr, &ph, &pressure, &sea_temp, nullptr, false, false,
	                                            size_bits);
	CHECK_EQUAL(ArgosPacketBuilder::LDA2_FRAME_BITS, size_bits);
	check_lda2_sensor_packet(x, "297166C6600781E00259CDAC03E83E803D86");  // mask=01110 (no ALS)
	x = ArgosPacketBuilder::build_sensor_packet(&e, &als, nullptr, &pressure, &sea_temp, nullptr, false, false,
	                                            size_bits);
	CHECK_EQUAL(ArgosPacketBuilder::LDA2_FRAME_BITS, size_bits);
	check_lda2_sensor_packet(x, "297166C6600781E0025AC271007D07D007B0C0");  // mask=10110 (no PH)
	x = ArgosPacketBuilder::build_sensor_packet(&e, &als, &ph, nullptr, &sea_temp, nullptr, false, false, size_bits);
	CHECK_EQUAL(ArgosPacketBuilder::LDA2_FRAME_BITS, size_bits);
	check_lda2_sensor_packet(x, "297166C6600781E0025B427106D603D860");  // mask=11010 (no Pressure)
	x = ArgosPacketBuilder::build_sensor_packet(&e, &als, &ph, &pressure, nullptr, nullptr, false, false, size_bits);
	CHECK_EQUAL(ArgosPacketBuilder::LDA2_FRAME_BITS, size_bits);
	check_lda2_sensor_packet(x, "297166C6600781E0025B827106D601F41F40");  // mask=11100 (no SeaTemp)
}

TEST(ArgosTxService, BuildSensorPacketSeaTemp) {
	unsigned int size_bits;
	GPSLogEntry e = make_gps_location(1, 12.3, 44.4, 1652105502);
	ServiceSensorData sea_temp;
	std::string x;

	sea_temp.port[0] = 147100;  // 21.1C

	x = ArgosPacketBuilder::build_sensor_packet(&e, nullptr, nullptr, nullptr, &sea_temp, nullptr, false, false,
	                                            size_bits);
	CHECK_EQUAL(ArgosPacketBuilder::LDA2_FRAME_BITS, size_bits);
	check_lda2_sensor_packet(x, "297166C6600781E00258423E9C");  // mask=00010 (SeaTemp only)
}


TEST(ArgosTxService, DepthPileManagerTestThermistorConversion) {
	bool enable = true;
	BaseSensorEnableTxMode mode = BaseSensorEnableTxMode::ONESHOT;
	fake_config_store->write_param(ParamID::THERMISTOR_SENSOR_ENABLE, enable);
	fake_config_store->write_param(ParamID::THERMISTOR_SENSOR_ENABLE_TX_MODE, mode);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	DepthPileManager man;

	ServiceEvent e;
	GPSLogEntry log;
	log.info.valid = true;
	ServiceSensorData sensor;

	e.event_source = ServiceIdentifier::GNSS_SENSOR;
	e.event_type = ServiceEventType::SERVICE_ACTIVE;
	man.notify_peer_event(e);

	e.event_data = log;
	e.event_source = ServiceIdentifier::GNSS_SENSOR;
	e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
	man.notify_peer_event(e);

	// Thermistor: 21.1°C -> (21.1 + 40.0) * 100 = 6110
	sensor.port[0] = 21.1;
	e.event_data = sensor;
	e.event_source = ServiceIdentifier::THERMISTOR_SENSOR;
	e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
	man.notify_peer_event(e);

	e.event_type = ServiceEventType::SERVICE_INACTIVE;
	man.notify_peer_event(e);

	CHECK_TRUE(man.eligible());

	// Thermistor retrieves from sea_temp depth pile
	ServiceSensorData *converted;
	converted = man.retrieve_sensor_single(1, ServiceIdentifier::THERMISTOR_SENSOR);
	CHECK_FALSE(nullptr == converted);
	CHECK_EQUAL(6110, (unsigned int)converted->port[0]);
}

TEST(ArgosTxService, BuildSensorPacketWithAXL) {
	unsigned int size_bits;
	GPSLogEntry e = make_gps_location(1, 12.3, 44.4, 1652105502);
	ServiceSensorData axl;
	std::string x;

	// AXL: temp=6500, x=16010, y=15980, z=17000, activity=5
	axl.port[0] = 6500;
	axl.port[1] = 16010;
	axl.port[2] = 15980;
	axl.port[3] = 17000;
	axl.port[4] = 5;

	x = ArgosPacketBuilder::build_sensor_packet(&e, nullptr, nullptr, nullptr, nullptr, &axl, false, false, size_bits);
	// Should include GPS + AXL data (temp 14 bits + X 15 bits + Y 15 bits + Z 15 bits + Activity 8 bits = 67 bits)
	CHECK_TRUE(size_bits > 83);  // Must be larger than GPS-only (83 bits)
}

TEST(ArgosTxService, BuildSensorPacketOutOfZone) {
	unsigned int size_bits;
	GPSLogEntry e = make_gps_location(1, 12.3, 44.4, 1652105502);
	std::string x;

	// LDA2 sensor packets are always emitted as 192-bit frames since CRC8 lives at byte 23.
	x = ArgosPacketBuilder::build_sensor_packet(&e, nullptr, nullptr, nullptr, nullptr, nullptr, true, false,
	                                            size_bits);
	CHECK_EQUAL(ArgosPacketBuilder::LDA2_FRAME_BITS, size_bits);
	CHECK_EQUAL(ArgosPacketBuilder::LDA2_FRAME_BYTES, x.size());

	x = ArgosPacketBuilder::build_sensor_packet(&e, nullptr, nullptr, nullptr, nullptr, nullptr, false, true,
	                                            size_bits);
	CHECK_EQUAL(ArgosPacketBuilder::LDA2_FRAME_BITS, size_bits);
	CHECK_EQUAL(ArgosPacketBuilder::LDA2_FRAME_BYTES, x.size());
}

TEST(ArgosTxService, BuildDopplerPacket) {
	unsigned int size_bits;
	std::string x;

	// Battery at 4200mV, not low
	x = ArgosPacketBuilder::build_doppler_packet(4200, false, size_bits);
	CHECK_TRUE(size_bits > 0);
	CHECK_FALSE(x.empty());

	// Battery at 2800mV, low battery
	x = ArgosPacketBuilder::build_doppler_packet(2800, true, size_bits);
	CHECK_TRUE(size_bits > 0);
	CHECK_FALSE(x.empty());
}

// Regression 2026-08 -- PASS_PREDICTION ignored the configured modulation.
//
// The service does resolve `adaptive ? LDA2 : resolve_non_adaptive_modulation()`
// correctly, right before calling the scheduler -- the 2026-05-25 modulation fix
// even says so in its own comment. But schedule_prepass took m_scheduled_mode by
// REFERENCE and overwrote it unconditionally with LDA2 a handful of lines later.
// The result: a configuration of LDK with adaptive disabled went out as LDA2,
// the frame was sized for the wrong modulation, and ensure_modulation rewrote the
// master RCONF on every transmission (flash wear, plus the user's configuration
// silently undone).
//
// The output parameter is gone: choosing the modulation is a policy decision and
// it belongs to the service.
TEST(ArgosTxService, PassPredictHonoursConfiguredModulation) {
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::PASS_PREDICTION);
	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, (unsigned int)0x01234567U);
	fake_config_store->write_param(ParamID::ARGOS_ADAPTIVE_MODULATION, false);
	fake_config_store->write_param(ParamID::LB_EN, false);
	fake_config_store->write_param(ParamID::TR_NOM, (unsigned int)10);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, false);

	// The module carries LDK in its master RCONF -- that is what
	// resolve_non_adaptive_modulation() must return.
	mock_kineis->test_set_current_modulation(KineisModulation::LDK);

	// A very wide satellite pass, so the window is certain to be found.
	BasePassPredict pass_predict = { /* version_code */ 0,
	                                 1,
	                                 { { 0xA,
		                                 SAT_DNLK_ON,
		                                 SAT_UPLK_ON_ARGOS_3,
		                                 { 2020, 1, 26, 22, 59, 44 },
		                                 7195.550f,
		                                 98.5444f,
		                                 327.835f,
		                                 -25.341f,
		                                 101.3587f,
		                                 0.00f } } };
	fake_config_store->write_pass_predict(pass_predict);

	ArgosTxService serv(*mock_kineis);
	std::time_t t = 1580083200000;  // ms
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	inject_gps_location(1, 11.8768, -33.8232, t / 1000);

	// LDK and not LDA2: that is the whole point of the regression.
	mock()
	    .expectOneCall("send")
	    .onObject(mock_kineis)
	    .withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDK)
	    .ignoreOtherParameters();
	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock().checkExpectations();
}

IGNORE_TEST(ArgosTxService, PassPredictWithSensorDataPayload) {
	double frequency = 900.22;
	BaseArgosMode mode = BaseArgosMode::PASS_PREDICTION;
	BaseArgosPower power = BaseArgosPower::POWER_1000_MW;
	BaseDepthPile depth_pile = BaseDepthPile::DEPTH_PILE_4;
	unsigned int argos_hexid = 0x01234567U;
	unsigned int tr_nom = 10;
	bool time_sync_en = false;
	bool enable = true;
	BaseSensorEnableTxMode sensor_mode = BaseSensorEnableTxMode::ONESHOT;

	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, depth_pile);
	fake_config_store->write_param(ParamID::ARGOS_FREQ, frequency);
	fake_config_store->write_param(ParamID::ARGOS_MODE, mode);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, argos_hexid);
	fake_config_store->write_param(ParamID::ARGOS_POWER, power);
	fake_config_store->write_param(ParamID::TR_NOM, tr_nom);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, time_sync_en);
	fake_config_store->write_param(ParamID::ALS_SENSOR_ENABLE, enable);
	fake_config_store->write_param(ParamID::ALS_SENSOR_ENABLE_TX_MODE, sensor_mode);

	BasePassPredict pass_predict = { /* version_code */ 0,
	                                 7,
	                                 { { 0xA,
		                                 SAT_DNLK_ON,
		                                 SAT_UPLK_ON_ARGOS_3,
		                                 { 2020, 1, 26, 22, 59, 44 },
		                                 7195.550f,
		                                 98.5444f,
		                                 327.835f,
		                                 -25.341f,
		                                 101.3587f,
		                                 0.00f },
		                               { 0x9,
		                                 SAT_DNLK_OFF,
		                                 SAT_UPLK_ON_ARGOS_3,
		                                 { 2020, 1, 26, 22, 33, 39 },
		                                 7195.632f,
		                                 98.7141f,
		                                 344.177f,
		                                 -25.340f,
		                                 101.3600f,
		                                 0.00f },
		                               { 0xB,
		                                 SAT_DNLK_ON,
		                                 SAT_UPLK_ON_ARGOS_3,
		                                 { 2020, 1, 26, 23, 29, 29 },
		                                 7194.917f,
		                                 98.7183f,
		                                 330.404f,
		                                 -25.336f,
		                                 101.3449f,
		                                 0.00f },
		                               { 0x5,
		                                 SAT_DNLK_OFF,
		                                 SAT_UPLK_ON_ARGOS_3,
		                                 { 2020, 1, 26, 23, 50, 6 },
		                                 7180.549f,
		                                 98.7298f,
		                                 289.399f,
		                                 -25.260f,
		                                 101.0419f,
		                                 -1.78f },
		                               { 0x8,
		                                 SAT_DNLK_OFF,
		                                 SAT_UPLK_ON_ARGOS_3,
		                                 { 2020, 1, 26, 22, 12, 6 },
		                                 7226.170f,
		                                 99.0661f,
		                                 343.180f,
		                                 -25.499f,
		                                 102.0039f,
		                                 -1.80f },
		                               { 0xC,
		                                 SAT_DNLK_OFF,
		                                 SAT_UPLK_ON_ARGOS_3,
		                                 { 2020, 1, 26, 22, 3, 52 },
		                                 7226.509f,
		                                 99.1913f,
		                                 291.936f,
		                                 -25.500f,
		                                 102.0108f,
		                                 -1.98f },
		                               { 0xD,
		                                 SAT_DNLK_ON,
		                                 SAT_UPLK_ON_ARGOS_3,
		                                 { 2020, 1, 26, 22, 3, 53 },
		                                 7160.246f,
		                                 98.5358f,
		                                 118.029f,
		                                 -25.154f,
		                                 100.6148f,
		                                 0.00f } } };

	fake_config_store->write_pass_predict(pass_predict);

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1580083200000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	ServiceSensorData sensor_data;

	inject_gps_active();
	inject_gps_location(1, 11.8768, -33.8232, t);
	sensor_data.port[0] = 1234;
	inject_sensor_data(sensor_data, ServiceIdentifier::ALS_SENSOR);
	inject_gps_inactive();
	inject_gps_active();
	inject_gps_location(1, 11.8768, -33.8232, t);
	sensor_data.port[0] = 5678;
	inject_sensor_data(sensor_data, ServiceIdentifier::ALS_SENSOR);
	inject_gps_inactive();
	inject_gps_active();
	inject_gps_location(1, 11.8768, -33.8232, t);
	sensor_data.port[0] = 13594;
	inject_sensor_data(sensor_data, ServiceIdentifier::ALS_SENSOR);
	inject_gps_inactive();
	inject_gps_active();
	inject_gps_location(1, 11.8768, -33.8232, t);
	sensor_data.port[0] = 22782;
	inject_sensor_data(sensor_data, ServiceIdentifier::ALS_SENSOR);
	inject_gps_inactive();

	system_scheduler->run();

	// Run for 10 transmissions
	for (unsigned int i = 0; i < 10; i++) {
		mock().expectOneCall("send").onObject(mock_kineis).ignoreOtherParameters();
		t += serv.get_last_schedule();
		fake_rtc->settime(t / 1000);
		fake_timer->set_counter(t);
		system_scheduler->run();
		mock_kineis->notify(KineisEventTxComplete({}));
	}
}


// ============================================================================
// Fastloc packet builder tests
// ============================================================================

TEST(ArgosTxService, FastlocSatellitePacketEncoding) {
	// Build a fastloc GPS entry with degraded fix
	GPSLogEntry log = make_gps_location(true, -33.8232, 11.8768, 1580083200, false, 0, 5000, 3800);
	log.info.event_type = GPSEventType::FASTLOC;
	log.info.fixType = 2;  // 2D fix
	log.info.numSV = 5;
	log.info.hAcc = 500000;  // 500m in mm

	KineisPacket packet = ArgosPacketBuilder::build_fastloc_packet(&log, false);

	// Verify packet size = 12 bytes (96 bits, LDK)
	CHECK_EQUAL(ArgosPacketBuilder::FASTLOC_PACKET_BYTES, packet.size());

	// Verify header is 010 (first 3 bits of first byte)
	unsigned int header = (packet[0] >> 5) & 0x07;
	CHECK_EQUAL(ArgosPacketBuilder::FASTLOC_PACKET_HEADER, header);
}

TEST(ArgosTxService, FastlocSatellitePacketQualityFields) {
	// Test that different quality values produce different encoded packets
	GPSLogEntry log1 = make_gps_location(true, 10.0, 20.0, 1580083200, false, 0, 0, 3800);
	log1.info.event_type = GPSEventType::FASTLOC;
	log1.info.fixType = 2;
	log1.info.numSV = 4;
	log1.info.hAcc = 200000;  // 200m

	GPSLogEntry log2 = make_gps_location(true, 10.0, 20.0, 1580083200, false, 0, 0, 3800);
	log2.info.event_type = GPSEventType::FASTLOC;
	log2.info.fixType = 3;
	log2.info.numSV = 8;
	log2.info.hAcc = 1500000;  // 1500m

	KineisPacket p1 = ArgosPacketBuilder::build_fastloc_packet(&log1, false);
	KineisPacket p2 = ArgosPacketBuilder::build_fastloc_packet(&log2, false);

	// Packets should differ (different quality metadata)
	CHECK(p1 != p2);
}

TEST(ArgosTxService, FastlocPacketHeaderAndSize) {
	// Verify fastloc packet uses LDA2 format (24 bytes) with correct header
	GPSLogEntry log = make_gps_location(true, -33.8232, 11.8768, 1580083200, true, 50000, 5000, 3800);
	log.info.event_type = GPSEventType::FASTLOC;
	log.info.fixType = 2;
	log.info.numSV = 5;
	log.info.hAcc = 500000;
	KineisPacket fastloc_pkt = ArgosPacketBuilder::build_fastloc_packet(&log, false);

	// Fastloc uses LDA2: 192 bits = 24 bytes
	CHECK_EQUAL(ArgosPacketBuilder::FASTLOC_PACKET_BYTES, fastloc_pkt.size());
	CHECK_EQUAL(24U, fastloc_pkt.size());

	// Header bits = 010
	unsigned int fastloc_header = (fastloc_pkt[0] >> 5) & 0x07;
	CHECK_EQUAL(0b010, fastloc_header);

	// Compare with short packet — different size and header
	KineisPacket short_pkt = ArgosPacketBuilder::build_short_packet(&log, false, false);
	CHECK_EQUAL(12U, short_pkt.size());
	CHECK(short_pkt.size() != fastloc_pkt.size());
}

TEST(ArgosTxService, FastlocHAccEncoding16bit) {
	// Test hAcc encoding with 16-bit resolution (0-65535m)
	GPSLogEntry log = make_gps_location(true, 10.0, 20.0, 1580083200, false, 0, 0, 3800);
	log.info.event_type = GPSEventType::FASTLOC;
	log.info.fixType = 2;
	log.info.numSV = 4;

	// hAcc = 0m
	log.info.hAcc = 0;
	KineisPacket p0 = ArgosPacketBuilder::build_fastloc_packet(&log, false);

	// hAcc = 500m
	log.info.hAcc = 500000;  // 500m in mm
	KineisPacket p500 = ArgosPacketBuilder::build_fastloc_packet(&log, false);

	// hAcc = 65535m (max encodable)
	log.info.hAcc = 65535000;
	KineisPacket pmax = ArgosPacketBuilder::build_fastloc_packet(&log, false);

	// hAcc = 100000m (exceeds max) → clamped to 65535
	log.info.hAcc = 100000000;
	KineisPacket p_clamped = ArgosPacketBuilder::build_fastloc_packet(&log, false);

	// Max and clamped should be same
	CHECK(pmax == p_clamped);

	// Different hAcc values should produce different packets
	CHECK(p0 != p500);
	CHECK(p500 != pmax);
}

// NOTE: lora_tx_service.cpp is now part of the TrackerTests build, so LoRaTxService
// is compiled and linked here (and therefore warning-checked under -Werror in CI).
// It has no test cases of its own yet: the shared behaviours -- critical-battery
// gating, session TX limit, cooldown arming, device-error backoff, burst-state
// reset -- are exercised only through their Argos twins in this file.

// ============================================================================
// COOLDOWN TESTS
// ============================================================================

TEST(ArgosTxService, CooldownBlocksSecondSurfacing) {
	// After a successful TX cycle, cooldown prevents new TX on next surfacing
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::SURFACING_BURST);
	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, (bool)false);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, (bool)true);
	fake_config_store->write_param(ParamID::ARGOS_TX_JITTER_EN, (bool)false);
	fake_config_store->write_param(ParamID::DRY_TIME_BEFORE_TX, 0U);
	fake_config_store->write_param(ParamID::MIN_SURFACE_CYCLE_INTERVAL_S, 2700U);
	fake_config_store->write_param(ParamID::COOLDOWN_TRIGGER_MODE, 3U);  // AFTER_LAST_TX

	ArgosTxService serv(*mock_kineis);

	// Use a distinct epoch far from other cooldown tests to avoid static state leaks
	std::time_t t = 1700000000000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// Dive
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("power_off_immediate").onObject(mock_kineis);
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	notify_underwater_state(true);

	// Advance 60s underwater
	t += 60000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	// Surface → first burst starts
	notify_underwater_state(false);
	CHECK_EQUAL(0U, serv.get_last_schedule());

	// Fire Doppler TX
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("send").onObject(mock_kineis).ignoreOtherParameters();
	system_scheduler->run();
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	mock_kineis->notify(KineisEventTxComplete({}));

	// Dive again → cooldown starts
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("power_off_immediate").onObject(mock_kineis);
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	notify_underwater_state(true);

	// Advance only 120s (well within 2700s cooldown)
	t += 120000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	// Surface again → should be blocked by cooldown
	notify_underwater_state(false);

	// No TX should be scheduled — service is disabled during cooldown
	// Verify no send mock call is expected
	mock().checkExpectations();
}

TEST(ArgosTxService, CooldownExpiresAllowsNewBurst) {
	// After cooldown expires, a new surfacing should trigger TX
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::SURFACING_BURST);
	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, (bool)false);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, (bool)true);
	fake_config_store->write_param(ParamID::ARGOS_TX_JITTER_EN, (bool)false);
	fake_config_store->write_param(ParamID::DRY_TIME_BEFORE_TX, 0U);
	fake_config_store->write_param(ParamID::MIN_SURFACE_CYCLE_INTERVAL_S, 60U);  // Short cooldown for test
	fake_config_store->write_param(ParamID::COOLDOWN_TRIGGER_MODE, 3U);

	ArgosTxService serv(*mock_kineis);

	// Use a distinct epoch far from other cooldown tests to avoid static state leaks
	std::time_t t = 1750000000000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// Dive → surface → TX → dive (first cycle)
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("power_off_immediate").onObject(mock_kineis);
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	notify_underwater_state(true);
	t += 30000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	notify_underwater_state(false);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("send").onObject(mock_kineis).ignoreOtherParameters();
	system_scheduler->run();
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	mock_kineis->notify(KineisEventTxComplete({}));

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("power_off_immediate").onObject(mock_kineis);
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	notify_underwater_state(true);

	// Advance PAST cooldown (60s + margin)
	t += 120000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	// Surface again → cooldown expired → TX should be scheduled
	notify_underwater_state(false);
	CHECK_EQUAL(0U, serv.get_last_schedule());
}

// ============================================================================
// COOLDOWN TRIGGER MODE — REGRESSION TESTS
// Cover the 3 modes not exercised by CooldownBlocksSecondSurfacing /
// CooldownExpiresAllowsNewBurst (which both use AFTER_LAST_TX = 3).
// ============================================================================

TEST(ArgosTxService, CooldownAtSurfaceDoesNotExtendOnBounce) {
	// AT_SURFACE mode: surface → dive starts cooldown. If a passive surface
	// bounce happens during cooldown, m_cooldown_armed must NOT be re-armed
	// (otherwise the next dive would call set_cycle_complete(now) and reset
	// the cooldown timer, extending it indefinitely under repeated bounces).
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::SURFACING_BURST);
	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, (bool)false);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, (bool)true);
	fake_config_store->write_param(ParamID::ARGOS_TX_JITTER_EN, (bool)false);
	fake_config_store->write_param(ParamID::DRY_TIME_BEFORE_TX, 0U);
	fake_config_store->write_param(ParamID::MIN_SURFACE_CYCLE_INTERVAL_S, 600U);
	fake_config_store->write_param(ParamID::COOLDOWN_TRIGGER_MODE, 0U);  // AT_SURFACE

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1751000000000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// Cycle 1: dive → surface (arms AT_SURFACE) → dive (cooldown starts)
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("power_off_immediate").onObject(mock_kineis);
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	notify_underwater_state(true);
	t += 60000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	notify_underwater_state(false);  // arm AT_SURFACE

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("power_off_immediate").onObject(mock_kineis);
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	notify_underwater_state(true);  // cooldown_armed → set_cycle_complete(t_dive_1)
	CHECK_TRUE(ServiceManager::is_in_cooldown(t / 1000));
	unsigned int remaining_at_dive_1 = ServiceManager::get_cooldown_remaining_s(t / 1000);
	CHECK_EQUAL(600U, remaining_at_dive_1);  // full interval just started

	// 30s into cooldown — passive surface bounce (should NOT re-arm)
	t += 30000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	notify_underwater_state(false);

	// Re-dive 5s later. Without the fix, m_cooldown_armed would be true here
	// (re-armed by AT_SURFACE), triggering set_cycle_complete(t_dive_2) which
	// would RESET the cooldown timer to 600s remaining.
	t += 5000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("power_off_immediate").onObject(mock_kineis);
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	notify_underwater_state(true);

	// Verify cooldown anchor is still t_dive_1, not t_dive_2.
	// Remaining at this point should be 600 - (30 + 5) = 565s, NOT 600s.
	unsigned int remaining = ServiceManager::get_cooldown_remaining_s(t / 1000);
	CHECK_TRUE(remaining > 560 && remaining <= 565);

	mock().checkExpectations();
}

TEST(ArgosTxService, IsInCooldownTrueWhenSetAndCheckedSameTick) {
	// Regression: get_cooldown_remaining_s used to return 0 when
	// now == m_last_successful_cycle_time (the `<=` branch). The fix
	// (changing to `<`) ensures set_cycle_complete(now) followed by
	// is_in_cooldown(now) in the same tick reports the cooldown as active.
	fake_config_store->write_param(ParamID::MIN_SURFACE_CYCLE_INTERVAL_S, 1800U);

	std::time_t t = 1752000000;
	fake_rtc->settime(t);

	ServiceManager::set_cycle_complete(t);
	CHECK_TRUE(ServiceManager::is_in_cooldown(t));
	CHECK_EQUAL(1800U, ServiceManager::get_cooldown_remaining_s(t));

	// Sanity: 1s later still in cooldown
	CHECK_TRUE(ServiceManager::is_in_cooldown(t + 1));
	CHECK_EQUAL(1799U, ServiceManager::get_cooldown_remaining_s(t + 1));

	// And expired exactly at the interval boundary
	CHECK_FALSE(ServiceManager::is_in_cooldown(t + 1800));
	CHECK_EQUAL(0U, ServiceManager::get_cooldown_remaining_s(t + 1800));

	// RTC backward → cooldown reported expired (graceful fallback)
	CHECK_FALSE(ServiceManager::is_in_cooldown(t - 1));
}

// ============================================================================
// NTRY_PER_MESSAGE (depth pile burst counter) TESTS
// ============================================================================

TEST(ArgosTxService, NtryPerMessageLimitsGnssTx) {
	// NTRY_PER_MESSAGE=2 → each GPS fix sent exactly 2 times, then no more
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::SURFACING_BURST);
	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, (bool)false);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, (bool)true);
	fake_config_store->write_param(ParamID::ARGOS_TX_JITTER_EN, (bool)false);
	fake_config_store->write_param(ParamID::DRY_TIME_BEFORE_TX, 0U);
	fake_config_store->write_param(ParamID::TR_NOM, 10U);  // 10s for fast test
	fake_config_store->write_param(ParamID::NTRY_PER_MESSAGE, 2U);

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// Dive → surface
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("power_off_immediate").onObject(mock_kineis);
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	notify_underwater_state(true);
	t += 60000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	notify_underwater_state(false);

	// Fire initial Doppler
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("send").onObject(mock_kineis).ignoreOtherParameters();
	system_scheduler->run();
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	mock_kineis->notify(KineisEventTxComplete({}));

	// Inject GPS fix → switches to GNSS phase
	inject_gps_location(true, 55.27, -21.01, t / 1000, true);

	// GNSS TX #1
	mock().expectOneCall("send").onObject(mock_kineis).ignoreOtherParameters();
	t += 1000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock_kineis->notify(KineisEventTxComplete({}));

	// GNSS TX #2 (last allowed by NTRY=2)
	mock().expectOneCall("send").onObject(mock_kineis).ignoreOtherParameters();
	t += 10000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock_kineis->notify(KineisEventTxComplete({}));

	// GNSS TX #3 should NOT happen (burst_counter exhausted)
	t += 10000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	// No send expected — verify
	mock().checkExpectations();
}

TEST(ArgosTxService, NtryZeroSendsOnce) {
	// NTRY_PER_MESSAGE=0 → in surfacing burst mode, each fix sent exactly once
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::SURFACING_BURST);
	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, (bool)false);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, (bool)true);
	fake_config_store->write_param(ParamID::ARGOS_TX_JITTER_EN, (bool)false);
	fake_config_store->write_param(ParamID::DRY_TIME_BEFORE_TX, 0U);
	fake_config_store->write_param(ParamID::TR_NOM, 10U);
	fake_config_store->write_param(ParamID::NTRY_PER_MESSAGE, 0U);  // 0 = once in burst mode

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// Dive → surface → Doppler
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("power_off_immediate").onObject(mock_kineis);
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	notify_underwater_state(true);
	t += 60000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	notify_underwater_state(false);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("send").onObject(mock_kineis).ignoreOtherParameters();
	system_scheduler->run();
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	mock_kineis->notify(KineisEventTxComplete({}));

	// GPS fix → GNSS phase
	inject_gps_location(true, 55.27, -21.01, t / 1000, true);

	// GNSS TX #1 (only one allowed)
	mock().expectOneCall("send").onObject(mock_kineis).ignoreOtherParameters();
	t += 1000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock_kineis->notify(KineisEventTxComplete({}));

	// GNSS TX #2 should NOT happen
	t += 10000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock().checkExpectations();
}

// LEGACY (and DUTY_CYCLE/PASS_PREDICTION): NTRY_PER_MESSAGE=0 means UNLIMITED
// replay — once a valid fix is in the depth pile it keeps being retransmitted
// every TR_NOM until evicted by newer fixes. (Without a valid fix nothing is
// cached — silence.)
TEST(ArgosTxService, LegacyNtryZeroUnlimitedReplay) {
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::LEGACY);
	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, (unsigned int)0x01234567U);
	fake_config_store->write_param(ParamID::LB_EN, false);
	fake_config_store->write_param(ParamID::TR_NOM, (unsigned int)10);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, false);
	fake_config_store->write_param(ParamID::NTRY_PER_MESSAGE, 0U);

	ArgosTxService serv(*mock_kineis);
	std::time_t t = 1652105502000;  // ms
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	inject_gps_location(1, 11.8768, -33.8232, t / 1000);

	// Far more cycles than any bounded counter would allow — every one must TX
	for (unsigned int i = 0; i < 8; i++) {
		mock().expectOneCall("send").onObject(mock_kineis).ignoreOtherParameters();
		t += serv.get_last_schedule();
		fake_rtc->settime(t / 1000);
		fake_timer->set_counter(t);
		system_scheduler->run();
		mock_kineis->notify(KineisEventTxComplete({}));
	}
	mock().checkExpectations();
}

// LEGACY: NTRY_PER_MESSAGE=3 → the pile is swept exactly 3 times (each entry
// TX'd 3 times), then all entries are inert and the service goes silent until
// a new fix arrives.
TEST(ArgosTxService, LegacyNtryBoundsPileSweeps) {
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::LEGACY);
	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, (unsigned int)0x01234567U);
	fake_config_store->write_param(ParamID::LB_EN, false);
	fake_config_store->write_param(ParamID::TR_NOM, (unsigned int)10);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, false);
	fake_config_store->write_param(ParamID::NTRY_PER_MESSAGE, 3U);

	ArgosTxService serv(*mock_kineis);
	std::time_t t = 1652105502000;  // ms
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	inject_gps_location(1, 11.8768, -33.8232, t / 1000);

	// Exactly 3 transmissions of the single-entry pile
	for (unsigned int i = 0; i < 3; i++) {
		mock().expectOneCall("send").onObject(mock_kineis).ignoreOtherParameters();
		t += serv.get_last_schedule();
		fake_rtc->settime(t / 1000);
		fake_timer->set_counter(t);
		system_scheduler->run();
		mock_kineis->notify(KineisEventTxComplete({}));
	}

	// 4th cycle: burst_counter exhausted → no send
	t += 10000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock().checkExpectations();
}

// Depth pile sweep semantics behind NTRY: pile [A,B,C] with burst_counter=2 →
// retrieve() rotates newest-first through the pile (C,B,A), completes 2 full
// sweeps, then everything is inert.
TEST(ArgosTxService, DepthPileNtrySweepRotation) {
	DepthPile<GPSLogEntry> dp;
	dp.set_max_size(4);
	GPSLogEntry a{};
	a.info.day = 1;
	GPSLogEntry b{};
	b.info.day = 2;
	GPSLogEntry c{};
	c.info.day = 3;
	dp.store(a, 2);
	dp.store(b, 2);
	dp.store(c, 2);

	// retrieve(depth=4, max_messages=1): one entry per TX slot, newest-first
	int expected[6] = { 3, 2, 1, 3, 2, 1 };  // two full sweeps C,B,A
	for (unsigned int i = 0; i < 6; i++) {
		std::vector<GPSLogEntry *> v = dp.retrieve(4, 1);
		CHECK_EQUAL(1u, (unsigned int)v.size());
		CHECK_EQUAL(expected[i], (int)v.at(0)->info.day);
	}

	// Third sweep: all burst counters exhausted
	CHECK_EQUAL(0u, (unsigned int)dp.retrieve(4, 1).size());
	CHECK_EQUAL(0u, dp.eligible());
}

// === BaseGnssStrategy::REUSE_LAST helpers (Plan 1 step 1) =================

TEST(ArgosTxService, DepthPilePeekBackReturnsLatestWithoutConsuming) {
	DepthPile<GPSLogEntry> dp;
	GPSLogEntry e1{};
	e1.info.day = 1;
	GPSLogEntry e2{};
	e2.info.day = 2;
	dp.store(e1, 3);  // burst_counter = 3
	dp.store(e2, 1);  // burst_counter = 1

	// Peek returns the most recent entry…
	GPSLogEntry *peeked = dp.peek_back();
	CHECK(peeked != nullptr);
	CHECK_EQUAL(2, peeked->info.day);
	// …and does NOT consume burst_counter — eligible() must still report both.
	CHECK_EQUAL(2u, dp.eligible());

	// Now drain the latest entry's burst slot via retrieve(2,1) so its
	// burst_counter hits zero, but the entry itself remains in the deque.
	dp.retrieve(2, 1);
	CHECK_EQUAL(1u, dp.eligible());

	// peek_back must still return the same latest data even when it's no
	// longer eligible — this is the whole point: REUSE_LAST reads stale entries.
	peeked = dp.peek_back();
	CHECK(peeked != nullptr);
	CHECK_EQUAL(2, peeked->info.day);
}

TEST(ArgosTxService, DepthPilePeekBackOnEmptyReturnsNull) {
	DepthPile<GPSLogEntry> dp;
	CHECK(dp.peek_back() == nullptr);
}

TEST(ArgosTxService, ComputeGpsLogAgeSecondsNormalEntry) {
	GPSLogEntry e = make_gps_location();  // stamps header with rtc->gettime() = 1580083200
	// 90 seconds later
	unsigned int age = ArgosTxService::compute_gps_log_age_seconds(e, 1580083200 + 90);
	CHECK_EQUAL(90u, age);
}

TEST(ArgosTxService, ComputeGpsLogAgeSecondsZeroYearIsInvalid) {
	GPSLogEntry e{};  // year == 0 — cold-boot / unset RTC sentinel
	unsigned int age = ArgosTxService::compute_gps_log_age_seconds(e, 1580083200);
	CHECK_EQUAL(UINT_MAX, age);
}

TEST(ArgosTxService, ComputeGpsLogAgeSecondsFutureEntryIsInvalid) {
	GPSLogEntry e = make_gps_location();  // stamped at rtc = 1580083200
	// `now` BEFORE the entry timestamp → corruption / RTC roll-back. Must reject
	// rather than report age=0 which would falsely qualify as "fresh".
	unsigned int age = ArgosTxService::compute_gps_log_age_seconds(e, 1580083200 - 60);
	CHECK_EQUAL(UINT_MAX, age);
}

// === RateLimiter (Plan 1 step 2) ==========================================

TEST(ArgosTxService, RateLimiterDisabledNeverBlocks) {
	RateLimiter::reset_for_tests();
	// Default: RATE_LIMIT_EN = false. Cap could be exhausted yet still allowed.
	for (unsigned int i = 0; i < 20; i++)
		RateLimiter::record_tx(1000 + i);  // no-op while disabled
	unsigned int rs = 0;
	CHECK_FALSE(RateLimiter::is_blocked(2000, rs));
}

TEST(ArgosTxService, RateLimiterBlocksOnceCapReached) {
	RateLimiter::reset_for_tests();
	fake_config_store->write_param(ParamID::RATE_LIMIT_EN, (bool)true);
	fake_config_store->write_param(ParamID::RATE_LIMIT_WINDOW_S, 100U);
	fake_config_store->write_param(ParamID::RATE_LIMIT_MAX_TX, 3U);

	// Three TX within the window — cap exactly hit.
	RateLimiter::record_tx(1000);
	RateLimiter::record_tx(1010);
	RateLimiter::record_tx(1020);

	unsigned int rs = 0;
	CHECK_TRUE(RateLimiter::is_blocked(1025, rs));
	// Next allowed = oldest_in_window(1000) + window(100) = 1100. From now=1025
	// that's 75 s away.
	CHECK_EQUAL(75u, rs);
}

TEST(ArgosTxService, RateLimiterUnblocksOnceOldestRollsOut) {
	RateLimiter::reset_for_tests();
	fake_config_store->write_param(ParamID::RATE_LIMIT_EN, (bool)true);
	fake_config_store->write_param(ParamID::RATE_LIMIT_WINDOW_S, 100U);
	fake_config_store->write_param(ParamID::RATE_LIMIT_MAX_TX, 2U);

	RateLimiter::record_tx(1000);
	RateLimiter::record_tx(1050);

	unsigned int rs = 0;
	CHECK_TRUE(RateLimiter::is_blocked(1080, rs));  // both inside window

	// At now=1101 the entry @1000 has rolled out (1101 - 100 = 1001 > 1000).
	CHECK_FALSE(RateLimiter::is_blocked(1101, rs));
}

TEST(ArgosTxService, RateLimiterIgnoresFutureDatedEntries) {
	RateLimiter::reset_for_tests();
	fake_config_store->write_param(ParamID::RATE_LIMIT_EN, (bool)true);
	fake_config_store->write_param(ParamID::RATE_LIMIT_WINDOW_S, 100U);
	fake_config_store->write_param(ParamID::RATE_LIMIT_MAX_TX, 2U);

	// RTC rollback: a recorded entry sits in the future relative to `now`.
	// Such entries must be ignored (defense matching compute_gps_log_age).
	RateLimiter::record_tx(2000);  // future-dated
	RateLimiter::record_tx(1000);  // in-window entry @ now=1050

	unsigned int rs = 0;
	CHECK_FALSE(RateLimiter::is_blocked(1050, rs));  // only 1 in-window, cap=2
}

TEST(ArgosTxService, RateLimiterZeroCapDisables) {
	RateLimiter::reset_for_tests();
	fake_config_store->write_param(ParamID::RATE_LIMIT_EN, (bool)true);
	fake_config_store->write_param(ParamID::RATE_LIMIT_WINDOW_S, 100U);
	fake_config_store->write_param(ParamID::RATE_LIMIT_MAX_TX, 0U);

	for (unsigned int i = 0; i < 10; i++)
		RateLimiter::record_tx(1000 + i);  // no-op while cap = 0

	unsigned int rs = 0;
	CHECK_FALSE(RateLimiter::is_blocked(1020, rs));
}

// === HauledModeService (Plan 1 step 3) ====================================

TEST(ArgosTxService, HauledModeDisabledNeverHauls) {
	HauledModeService::reset_for_tests();
	// HAULED_DETECT_EN defaults to false. Even with a stale last_uw_event,
	// evaluate() must keep us at AT_SEA.
	HauledModeService::on_underwater_event(true, 1000);
	HauledModeService::evaluate(1000 + 100 * 3600);  // 100 hours later
	CHECK_FALSE(HauledModeService::is_hauled());
}

TEST(ArgosTxService, HauledModeHaulsAfterIdleThreshold) {
	HauledModeService::reset_for_tests();
	fake_config_store->write_param(ParamID::HAULED_DETECT_EN, (bool)true);
	fake_config_store->write_param(ParamID::HAULED_IDLE_THRESHOLD_H, 2U);  // 2h
	fake_config_store->write_param(ParamID::HAULED_RETURN_EVENTS, 3U);

	HauledModeService::on_underwater_event(true, 1000);
	// 1h59m later: still AT_SEA.
	HauledModeService::evaluate(1000 + 119 * 60);
	CHECK_FALSE(HauledModeService::is_hauled());
	// Just past 2h: HAULED engages.
	HauledModeService::evaluate(1000 + 2 * 3600 + 5);
	CHECK_TRUE(HauledModeService::is_hauled());
}

TEST(ArgosTxService, HauledModeReturnsAfterReturnEvents) {
	HauledModeService::reset_for_tests();
	fake_config_store->write_param(ParamID::HAULED_DETECT_EN, (bool)true);
	fake_config_store->write_param(ParamID::HAULED_IDLE_THRESHOLD_H, 1U);
	fake_config_store->write_param(ParamID::HAULED_RETURN_EVENTS, 3U);

	// Engage HAULED first.
	HauledModeService::on_underwater_event(true, 1000);
	HauledModeService::evaluate(1000 + 3601);
	CHECK_TRUE(HauledModeService::is_hauled());

	// 2 dive events: still HAULED (hysteresis).
	HauledModeService::on_underwater_event(true, 5000);
	CHECK_TRUE(HauledModeService::is_hauled());
	HauledModeService::on_underwater_event(true, 5100);
	CHECK_TRUE(HauledModeService::is_hauled());
	// 3rd dive: returns AT_SEA.
	HauledModeService::on_underwater_event(true, 5200);
	CHECK_FALSE(HauledModeService::is_hauled());
	CHECK_EQUAL(0u, HauledModeService::uw_events_since_hauled());
}

TEST(ArgosTxService, HauledModeOverrideAppliesToArgosConfig) {
	HauledModeService::reset_for_tests();
	fake_config_store->write_param(ParamID::HAULED_DETECT_EN, (bool)true);
	fake_config_store->write_param(ParamID::HAULED_IDLE_THRESHOLD_H, 1U);
	fake_config_store->write_param(ParamID::HAULED_ARGOS_MODE, BaseArgosMode::PASS_PREDICTION);
	fake_config_store->write_param(ParamID::HAULED_TR_NOM, 9999U);
	fake_config_store->write_param(ParamID::HAULED_GNSS_EN, (bool)false);
	fake_config_store->write_param(ParamID::HAULED_GNSS_STRAT, 1U);  // REUSE_LAST

	// Set non-hauled baseline first.
	ArgosConfig ac;
	configuration_store->get_argos_configuration(ac);
	CHECK_FALSE(ac.mode == BaseArgosMode::PASS_PREDICTION);  // base = LEGACY

	// Engage hauled.
	HauledModeService::on_underwater_event(true, 1000);
	HauledModeService::evaluate(1000 + 3601);
	CHECK_TRUE(HauledModeService::is_hauled());

	configuration_store->get_argos_configuration(ac);
	CHECK_TRUE(ac.mode == BaseArgosMode::PASS_PREDICTION);
	CHECK_EQUAL(9999u, ac.tx_interval_s);
	CHECK_FALSE(ac.gnss_en);  // HAULED_GNSS_EN=false → off
}

TEST(ArgosTxService, HauledModeGnssStratOffForcesGnssOff) {
	HauledModeService::reset_for_tests();
	fake_config_store->write_param(ParamID::HAULED_DETECT_EN, (bool)true);
	fake_config_store->write_param(ParamID::HAULED_IDLE_THRESHOLD_H, 1U);
	// HAULED_GNSS_EN says ON, but STRAT=OFF must win (Doppler-only).
	fake_config_store->write_param(ParamID::HAULED_GNSS_EN, (bool)true);
	fake_config_store->write_param(ParamID::HAULED_GNSS_STRAT, 2U);  // OFF

	HauledModeService::on_underwater_event(true, 1000);
	HauledModeService::evaluate(1000 + 3601);

	ArgosConfig ac;
	configuration_store->get_argos_configuration(ac);
	CHECK_FALSE(ac.gnss_en);

	GNSSConfig gc;
	configuration_store->get_gnss_configuration(gc);
	CHECK_FALSE(gc.enable);
}

// === BaseGnssStrategy::REUSE_LAST wiring (Plan 1 follow-up) ===============
// Verifies the REUSE_LAST dispatch routes through process_gnss_burst_from_cached
// and falls back gracefully when the depth pile has no usable entry. Each test
// sets HAULED active so the strategy field is populated by the config store.

TEST(ArgosTxService, ReuseLastEngagesGnssStrategyInHauled) {
	HauledModeService::reset_for_tests();
	fake_config_store->write_param(ParamID::HAULED_DETECT_EN, (bool)true);
	fake_config_store->write_param(ParamID::HAULED_IDLE_THRESHOLD_H, 1U);
	fake_config_store->write_param(ParamID::HAULED_GNSS_EN, (bool)true);  // ignored when STRAT=REUSE_LAST
	fake_config_store->write_param(ParamID::HAULED_GNSS_STRAT, 1U);       // REUSE_LAST

	HauledModeService::on_underwater_event(true, 1000);
	HauledModeService::evaluate(1000 + 3601);

	ArgosConfig ac;
	configuration_store->get_argos_configuration(ac);
	// REUSE_LAST keeps GPS OFF (no acquisition) but sets strategy so dispatch
	// can route to the cached-fix path.
	CHECK_FALSE(ac.gnss_en);
	CHECK_TRUE(ac.gnss_strategy == BaseGnssStrategy::REUSE_LAST);
}

TEST(ArgosTxService, ReuseLastFreshHauledStillAcquires) {
	HauledModeService::reset_for_tests();
	fake_config_store->write_param(ParamID::HAULED_DETECT_EN, (bool)true);
	fake_config_store->write_param(ParamID::HAULED_IDLE_THRESHOLD_H, 1U);
	fake_config_store->write_param(ParamID::HAULED_GNSS_EN, (bool)true);
	fake_config_store->write_param(ParamID::HAULED_GNSS_STRAT, 0U);  // FRESH

	HauledModeService::on_underwater_event(true, 1000);
	HauledModeService::evaluate(1000 + 3601);

	ArgosConfig ac;
	configuration_store->get_argos_configuration(ac);
	// FRESH preserves the existing acquire-then-TX behavior — gnss_en honors HMP12.
	CHECK_TRUE(ac.gnss_en);
	CHECK_TRUE(ac.gnss_strategy == BaseGnssStrategy::FRESH);
}

// === New mitigations (R5 auto-promote, M1b rollback recovery, K+L RTC sync) ===

TEST(ArgosTxService, HauledArgosModeAutoPromotesSurfacingBurstToLegacy) {
	// HMP10 allowed_values now excludes SURFACING_BURST (5) at DTE write,
	// but legacy configs persisted with that value must auto-promote at
	// read time to avoid a TX-less HAULED mode (no dives = no burst).
	HauledModeService::reset_for_tests();
	fake_config_store->write_param(ParamID::HAULED_DETECT_EN, (bool)true);
	fake_config_store->write_param(ParamID::HAULED_IDLE_THRESHOLD_H, 1U);
	// Force the bad value at write — bypass param-store validation in test
	// by setting directly (production would refuse PARMW HMP10=5).
	fake_config_store->write_param(ParamID::HAULED_ARGOS_MODE, BaseArgosMode::SURFACING_BURST);

	HauledModeService::on_underwater_event(true, 1000);
	HauledModeService::evaluate(1000 + 3601);

	ArgosConfig ac;
	configuration_store->get_argos_configuration(ac);
	// Auto-promoted to LEGACY despite the persisted SURFACING_BURST value.
	CHECK_TRUE(ac.mode == BaseArgosMode::LEGACY);
}

TEST(ArgosTxService, HauledRollbackReBaselinesInsteadOfFreezing) {
	// M1b (2026-05): a WDT reset puts RTC back to 1 while noinit may still
	// have a large last_uw_event_rtc from the previous session. The old
	// behavior was "return — wait it out" which leaves HAULED engagement
	// stuck. The fix re-baselines `last_uw_event_rtc = now`, so HAULED can
	// re-engage threshold_h hours later from the new RTC frame.
	HauledModeService::reset_for_tests();
	fake_config_store->write_param(ParamID::HAULED_DETECT_EN, (bool)true);
	fake_config_store->write_param(ParamID::HAULED_IDLE_THRESHOLD_H, 1U);

	// Stuff a "future" timestamp into the noinit by simulating an old session.
	HauledModeService::on_underwater_event(true, 100000);  // future from "now=1" perspective

	// Now evaluate at a tiny rtc value (mimics post-WDT virtual RTC).
	HauledModeService::evaluate(1);
	// Should NOT remain frozen: last_uw_event_rtc must have been re-baselined
	// to now (= 1).
	CHECK_EQUAL((std::time_t)1, HauledModeService::last_uw_event_rtc());
}

TEST(ArgosTxService, HauledResetForRtcSyncReAnchorsTimestamp) {
	// K (2026-05): when GPS first syncs RTC from virtual epoch to real UTC,
	// `last_uw_event_rtc` (computed in virtual frame) must be re-anchored
	// to the new real-time frame, otherwise the next evaluate() sees a
	// ~50-year elapsed and falsely engages HAULED.
	HauledModeService::reset_for_tests();
	HauledModeService::on_underwater_event(true, 3600);  // virtual RTC frame (1h uptime)

	// GPS fix arrives, RTC jumps to real epoch.
	std::time_t real_rtc = 1700000000;
	HauledModeService::reset_for_rtc_sync(real_rtc);

	CHECK_EQUAL(real_rtc, HauledModeService::last_uw_event_rtc());
}

TEST(ArgosTxService, RateLimiterResetForRtcSyncClearsRing) {
	// L (2026-05): clear ring on virtual→real RTC transition. Pre-sync ring
	// entries are in the virtual frame and meaningless post-sync.
	RateLimiter::reset_for_tests();
	fake_config_store->write_param(ParamID::RATE_LIMIT_EN, (bool)true);
	fake_config_store->write_param(ParamID::RATE_LIMIT_WINDOW_S, 100U);
	fake_config_store->write_param(ParamID::RATE_LIMIT_MAX_TX, 2U);

	RateLimiter::record_tx(150);
	RateLimiter::record_tx(180);
	CHECK_EQUAL(2u, RateLimiter::count_in_window(200, 100));  // both in [100,200]

	// Simulate RTC sync — ring should be cleared.
	RateLimiter::reset_for_rtc_sync();
	CHECK_EQUAL(0u, RateLimiter::count_in_window(200, 100));
}

TEST(ArgosTxService, NonHauledKeepsFreshStrategy) {
	HauledModeService::reset_for_tests();
	// HAULED detection disabled — config returns NORMAL branch.
	fake_config_store->write_param(ParamID::HAULED_DETECT_EN, (bool)false);
	fake_config_store->write_param(ParamID::HAULED_GNSS_STRAT, 1U);  // REUSE_LAST, but irrelevant
	fake_config_store->write_param(ParamID::GNSS_EN, (bool)true);

	ArgosConfig ac;
	configuration_store->get_argos_configuration(ac);
	// Non-HAULED path → strategy must be FRESH (byte-identical pre-Plan-1
	// behavior preserved — REUSE_LAST never accidentally engages outside HAULED).
	CHECK_TRUE(ac.gnss_strategy == BaseGnssStrategy::FRESH);
	CHECK_TRUE(ac.gnss_en);
}

// (REUSE_LAST fallback when pile is empty is covered by the helper-level
// tests ComputeGpsLogAgeSecondsZeroYearIsInvalid + DepthPilePeekBackOnEmptyReturnsNull:
// process_gnss_burst_from_cached calls read_cached_last_fix → returns false →
// falls through to process_doppler_burst — the existing Doppler path.)

TEST(ArgosTxService, HauledModeLowBatteryWins) {
	// Priority cascade per Plan 1 §5: LOW_BATTERY > HAULED. When both
	// conditions are met, HAULED must not override LB params.
	HauledModeService::reset_for_tests();
	fake_config_store->write_param(ParamID::HAULED_DETECT_EN, (bool)true);
	fake_config_store->write_param(ParamID::HAULED_IDLE_THRESHOLD_H, 1U);
	fake_config_store->write_param(ParamID::HAULED_ARGOS_MODE, BaseArgosMode::PASS_PREDICTION);
	fake_config_store->write_param(ParamID::LB_EN, (bool)true);
	fake_config_store->write_param(ParamID::LB_ARGOS_MODE, BaseArgosMode::DOPPLER);
	// Force the LB flag directly. FakeConfigurationStore::update_battery_level
	// is a no-op, so the underlying flag must be set explicitly.
	fake_config_store->set_is_battery_level_low(true);

	HauledModeService::on_underwater_event(true, 1000);
	HauledModeService::evaluate(1000 + 3601);

	ArgosConfig ac;
	configuration_store->get_argos_configuration(ac);
	CHECK_TRUE(ac.is_lb);
	// LB wins — HAULED's mode does NOT override.
	CHECK_TRUE(ac.mode == BaseArgosMode::DOPPLER);
}

// === Device-error backoff and suspension ===================================
// react(KineisEventDeviceError) has two regimes and nothing in this file
// exercised either. They recover by completely different mechanisms, and only
// one of them is the mechanism the code claims.
//
// Below DEVICE_ERROR_MAX_CONSECUTIVE the service arms a backoff and calls
// service_complete(), which reschedules. It wakes itself: no peer event is
// involved, so it recovers on a land tracker as well as a diving one, with
// GNSS on or off. At the threshold it calls service_complete(nullptr, nullptr,
// false), which does NOT reschedule -- and that is where it gets interesting.

// The self-recovering regime. One error, then no GPS session, no surface
// event and no reboot: the beacon must come back on its own.
TEST(ArgosTxService, DeviceErrorBelowThresholdReschedulesWithoutAnyExternalEvent) {
	configure_plain_legacy();

	ArgosTxService serv(*mock_kineis);
	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();
	inject_gps_location(true, 11.8768, -33.8232, t / 1000, true);

	mock().expectOneCall("send").onObject(mock_kineis).ignoreOtherParameters();
	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();

	mock().expectOneCall("stop_send").onObject(mock_kineis);
	mock_kineis->notify(KineisEventDeviceError({}));

	// DEVICE_ERROR_BACKOFF_BASE_MS. The service rearmed a schedule by itself;
	// anything else means a single device error silences the beacon until a
	// peer event happens to arrive.
	CHECK_EQUAL(60000U, serv.get_last_schedule());

	mock().expectOneCall("send").onObject(mock_kineis).ignoreOtherParameters();
	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock().checkExpectations();
}

// The suspending regime. Three strikes and the service stops transmitting --
// but it parks on a deadline rather than waiting for the world to poke it.
//
// It did neither before: service_complete(..., false) skipped reschedule(),
// which is the only path that cancels the safety-net timeout armed just before
// service_initiate(). The timeout outlived the suspension, fired, rescheduled
// at zero delay, hit the guard in service_initiate() and rearmed itself. The
// beacon woke, logged and skipped once per timeout, forever, transmitting
// nothing and writing an LFS commit each pass -- the opposite of the battery
// saving it was meant to be.
TEST(ArgosTxService, DeviceErrorAtThresholdSleepsOnTheProbeDeadline) {
	configure_plain_legacy();

	ArgosTxService serv(*mock_kineis);
	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	// Three failures and nothing after them. Counting `send` strictly is the
	// assertion; CppUTest still fails a surplus call to an expected function, so
	// a fourth transmission cannot slip through unnoticed.
	mock().expectNCalls(3, "send").onObject(mock_kineis).ignoreOtherParameters();
	mock().ignoreOtherCalls();

	serv.start();
	inject_gps_location(true, 11.8768, -33.8232, t / 1000, true);

	for (unsigned int i = 0; i < 3; i++) {
		t += serv.get_last_schedule();
		fake_rtc->settime(t / 1000);
		fake_timer->set_counter(t);
		system_scheduler->run();
		mock_kineis->notify(KineisEventDeviceError({}));
	}

	// Parked well beyond any backoff the ladder can produce -- and emphatically
	// not at zero delay, which is what the spin looked like.
	CHECK_COMPARE(serv.get_last_schedule(), >, DEVICE_ERROR_BACKOFF_MAX_MS_FOR_TEST);

	// Ten wake-ups across half an hour, all inside the probe period.
	for (unsigned int k = 0; k < 10; k++) {
		t += 180000;
		fake_rtc->settime(t / 1000);
		fake_timer->set_counter(t);
		system_scheduler->run();
	}
	mock().checkExpectations();
}

// The deadline expires and exactly one dispatch goes out as a probe. This is
// the exit that exists on every board and every configuration -- including a
// land tracker with GNSS off, which has none of the other three.
TEST(ArgosTxService, ProbeTransmitsOnceTheSuspensionElapses) {
	configure_plain_legacy();

	ArgosTxService serv(*mock_kineis);
	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectNCalls(3 + 1, "send").onObject(mock_kineis).ignoreOtherParameters();
	mock().ignoreOtherCalls();

	serv.start();
	inject_gps_location(true, 11.8768, -33.8232, t / 1000, true);

	for (unsigned int i = 0; i < 3; i++) {
		t += serv.get_last_schedule();
		fake_rtc->settime(t / 1000);
		fake_timer->set_counter(t);
		system_scheduler->run();
		mock_kineis->notify(KineisEventDeviceError({}));
	}

	// Past ARGOS_TX_ERROR_SUSPEND_S (3600 s by default).
	t += 3700000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();

	// The probe fails too, so the suspension re-arms instead of lifting: the
	// wake-ups that follow must stay silent.
	mock_kineis->notify(KineisEventDeviceError({}));
	for (unsigned int k = 0; k < 5; k++) {
		t += 180000;
		fake_rtc->settime(t / 1000);
		fake_timer->set_counter(t);
		system_scheduler->run();
	}
	mock().checkExpectations();
}

// A probe that works clears the count outright, so the beacon returns to its
// nominal cadence rather than creeping back up the backoff ladder.
TEST(ArgosTxService, SuccessfulProbeClearsTheSuspensionCompletely) {
	configure_plain_legacy();

	ArgosTxService serv(*mock_kineis);
	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectNCalls(3 + 1 + 1, "send").onObject(mock_kineis).ignoreOtherParameters();
	mock().ignoreOtherCalls();

	serv.start();
	inject_gps_location(true, 11.8768, -33.8232, t / 1000, true);

	for (unsigned int i = 0; i < 3; i++) {
		t += serv.get_last_schedule();
		fake_rtc->settime(t / 1000);
		fake_timer->set_counter(t);
		system_scheduler->run();
		mock_kineis->notify(KineisEventDeviceError({}));
	}

	t += 3700000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock_kineis->notify(KineisEventTxComplete({}));

	// Nominal period, not another suspension and not a backoff.
	CHECK_COMPARE(serv.get_last_schedule(), <, 60000U);

	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock().checkExpectations();
}

// The exit written for the 2026-06-30 field case, where a KIM2 brown-out cost
// 6.5 h of silence: a land tracker gets a fresh chance on its next GPS
// session, without waiting out the probe period. Clearing the counter alone
// would not do it -- the suspension leaves an earliest-TX floor an hour out
// that has to be dropped with it.
TEST(ArgosTxService, DeviceErrorSuspensionIsClearedByANewGpsSession) {
	configure_plain_legacy();

	ArgosTxService serv(*mock_kineis);
	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectNCalls(3 + 1, "send").onObject(mock_kineis).ignoreOtherParameters();
	mock().ignoreOtherCalls();

	serv.start();
	inject_gps_location(true, 11.8768, -33.8232, t / 1000, true);

	for (unsigned int i = 0; i < 3; i++) {
		t += serv.get_last_schedule();
		fake_rtc->settime(t / 1000);
		fake_timer->set_counter(t);
		system_scheduler->run();
		mock_kineis->notify(KineisEventDeviceError({}));
	}

	// A minute later -- deep inside the probe period -- a fix lands.
	t += 60000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	inject_gps_location(true, 11.8768, -33.8232, t / 1000, true);

	// Prompt, not an hour away.
	CHECK_COMPARE(serv.get_last_schedule(), <, 60000U);

	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock().checkExpectations();
}

// --- Is the backoff triggered for the right reasons? ------------------------

// The counter must not move for a refusal that is itself a recovery window.
// SmdSat's 30-min autofallback cooldown rejects TX on purpose; counting those
// rejections would spend the whole 3-strike budget in about three surface
// events and suspend a beacon whose radio is fine. react() checks
// cooldown_remaining_ms() BEFORE the increment for exactly this reason
// (argos_tx_service.cpp, top of react(KineisEventDeviceError)).
TEST(ArgosTxService, DeviceCooldownRejectionDoesNotBurnTheErrorBudget) {
	configure_plain_legacy();

	ArgosTxService serv(*mock_kineis);
	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();
	inject_gps_location(true, 11.8768, -33.8232, t / 1000, true);

	// Five minutes of cooldown left on the module.
	mock_kineis->test_set_cooldown_remaining_ms(300000);

	// Four rejections -- one more than DEVICE_ERROR_MAX_CONSECUTIVE, so a
	// suspension would already have happened if these were being counted.
	mock().expectNCalls(4 + 1, "send").onObject(mock_kineis).ignoreOtherParameters();
	mock().expectNCalls(4, "stop_send").onObject(mock_kineis);

	for (unsigned int i = 0; i < 4; i++) {
		t += serv.get_last_schedule();
		fake_rtc->settime(t / 1000);
		fake_timer->set_counter(t);
		system_scheduler->run();
		mock_kineis->notify(KineisEventDeviceError({}));
		// Rescheduled past the cooldown with the +1 s margin -- and NOT by the
		// exponential backoff, which would read 60 s, then 120 s.
		CHECK_EQUAL(301000U, serv.get_last_schedule());
	}

	// Cooldown lifts. The beacon must transmit, not sit in a suspension it
	// never earned.
	mock_kineis->test_set_cooldown_remaining_ms(0);
	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock().checkExpectations();
}

// A successful TX must zero the count, not merely pause it: "consecutive" has
// to mean consecutive. Without this, three failures spread across a deployment
// -- with good transmissions in between -- would eventually suspend a healthy
// beacon, because nothing else ages the counter down.
TEST(ArgosTxService, SuccessfulTxClearsTheErrorCount) {
	configure_plain_legacy();

	ArgosTxService serv(*mock_kineis);
	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();
	inject_gps_location(true, 11.8768, -33.8232, t / 1000, true);

	mock().expectNCalls(3, "send").onObject(mock_kineis).ignoreOtherParameters();
	mock().expectNCalls(2, "stop_send").onObject(mock_kineis);

	// First failure: base backoff.
	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock_kineis->notify(KineisEventDeviceError({}));
	CHECK_EQUAL(60000U, serv.get_last_schedule());

	// Then one that works.
	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock_kineis->notify(KineisEventTxComplete({}));

	// The next failure is a FIRST failure again: 60 s, not 120 s.
	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock_kineis->notify(KineisEventDeviceError({}));
	CHECK_EQUAL(60000U, serv.get_last_schedule());
	mock().checkExpectations();
}

// What the backoff is measured against, pinned because it is easy to assume
// wrong. DEVICE_ERROR_BACKOFF_BASE_MS is an absolute retry delay, not a
// multiple of TR_NOM: ArgosTxScheduler::set_earliest_schedule is a floor
// (schedule_periodic takes max(earliest, last TX anchor), argos_tx_scheduler.cpp
// around the m_earliest_schedule block), so it can never pull a TX earlier than
// the backoff -- but with a long nominal period the retry still lands well
// before the next nominal slot. That is deliberate for a retry, and the
// 3-strike suspension bounds it to two extra attempts; it is not a storm.
TEST(ArgosTxService, BackoffIsAnAbsoluteRetryDelayNotAMultipleOfTrNom) {
	configure_plain_legacy(3600);  // one hour between nominal transmissions

	ArgosTxService serv(*mock_kineis);
	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();
	inject_gps_location(true, 11.8768, -33.8232, t / 1000, true);

	mock().expectNCalls(2, "send").onObject(mock_kineis).ignoreOtherParameters();
	mock().expectNCalls(2, "stop_send").onObject(mock_kineis);

	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock_kineis->notify(KineisEventDeviceError({}));
	CHECK_EQUAL(60000U, serv.get_last_schedule());

	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock_kineis->notify(KineisEventDeviceError({}));
	CHECK_EQUAL(120000U, serv.get_last_schedule());
	mock().checkExpectations();
}

// A TX that simply never answers -- no TxComplete, no DeviceError, the module
// gone quiet -- is NOT a device error as far as this counter is concerned.
// Service::on_service_timeout cancels and reschedules without touching it, so
// there is no backoff on that path: the beacon retries at its nominal period
// indefinitely. Whether that is the right policy is a separate question; this
// test exists so the answer is visible rather than assumed.
TEST(ArgosTxService, TxTimeoutCountsNoErrorAndArmsNoBackoff) {
	configure_plain_legacy();

	ArgosTxService serv(*mock_kineis);
	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();
	inject_gps_location(true, 11.8768, -33.8232, t / 1000, true);

	mock().expectNCalls(2, "send").onObject(mock_kineis).ignoreOtherParameters();
	mock().expectNCalls(2, "stop_send").onObject(mock_kineis);

	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();

	// Silence. service_next_timeout() is 30 s on this build; step past it.
	t += 31000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();

	// No backoff was armed: the schedule is the nominal one, not 60 s.
	CHECK_COMPARE(serv.get_last_schedule(), <, 60000U);

	// And the count is still zero, so the next real error is a FIRST error.
	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock_kineis->notify(KineisEventDeviceError({}));
	CHECK_EQUAL(60000U, serv.get_last_schedule());
	mock().checkExpectations();
}

// --- The remaining two ways out of a suspension -----------------------------

// The turtle case. A diving tracker gets a second exit that a land tracker
// does not: surfacing. The UW branch clears the counter, and the base class
// reschedules through notify_underwater_state(false) ->
// service_is_triggered_on_surfaced(). This is the one recovery that does not
// depend on the wake loop, so it is the one that must keep working if that
// loop is ever removed.
TEST(ArgosTxService, SurfaceEventClearsTheSuspension) {
	configure_plain_legacy();
	fake_config_store->write_param(ParamID::DRY_TIME_BEFORE_TX, (unsigned int)0);

	ArgosTxService serv(*mock_kineis);
	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();
	inject_gps_location(true, 11.8768, -33.8232, t / 1000, true);

	mock().expectNCalls(3 + 1, "send").onObject(mock_kineis).ignoreOtherParameters();
	mock().expectNCalls(3 + 1, "stop_send").onObject(mock_kineis);
	// Diving caches TCXO=0 for the next surfacing and cuts the module; the
	// first TX after surfacing then skips the warmup, hence two zero writes.
	mock().expectNCalls(2, "set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("power_off_immediate").onObject(mock_kineis);

	for (unsigned int i = 0; i < 3; i++) {
		t += serv.get_last_schedule();
		fake_rtc->settime(t / 1000);
		fake_timer->set_counter(t);
		system_scheduler->run();
		mock_kineis->notify(KineisEventDeviceError({}));
	}

	// Down, then up.
	notify_underwater_state(true);
	t += 600000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	notify_underwater_state(false);

	// Transmitting again.
	t += 60000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock().checkExpectations();
}

// The last exit, and the only one that always exists on every board and every
// configuration: a reboot. The counter is plain RAM, cleared by service_init.
// It is also the exit the 2026-06-30 field case had to fall back on, which is
// precisely why the other three matter.
TEST(ArgosTxService, RebootClearsTheErrorCount) {
	configure_plain_legacy();

	ArgosTxService serv(*mock_kineis);
	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	mock().expectNCalls(2, "set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();
	inject_gps_location(true, 11.8768, -33.8232, t / 1000, true);

	mock().expectNCalls(3 + 1, "send").onObject(mock_kineis).ignoreOtherParameters();
	mock().expectNCalls(3 + 1, "stop_send").onObject(mock_kineis);
	// Stopping the service cuts the module, as a reboot would.
	mock().expectOneCall("power_off_immediate").onObject(mock_kineis);

	for (unsigned int i = 0; i < 3; i++) {
		t += serv.get_last_schedule();
		fake_rtc->settime(t / 1000);
		fake_timer->set_counter(t);
		system_scheduler->run();
		mock_kineis->notify(KineisEventDeviceError({}));
	}

	serv.stop();
	serv.start();
	inject_gps_location(true, 11.8768, -33.8232, t / 1000, true);

	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock().checkExpectations();
}

// KineisDevice::notify broadcasts to every subscriber, and on the KIM2 build
// ArgosRxService holds the same device object as this service. A refused
// downlink, a rejected AT+DL or an error during reception therefore arrive in
// react(KineisEventDeviceError) with no transmission of ours in flight. They
// must not move the strike counter: three failed RX windows muting a healthy
// transmitter is the failure this guard exists to prevent, and it is silent --
// with nothing pending there is no backoff and no reschedule to notice, only
// the count creeping up until service_initiate() starts refusing.
TEST(ArgosTxService, DeviceErrorWithNoTxInFlightIsNotCounted) {
	configure_plain_legacy();

	// One transmission expected, at the end. If the guard is missing, five
	// strikes suspend the service and it never happens.
	mock().expectOneCall("send").onObject(mock_kineis).ignoreOtherParameters();
	mock().ignoreOtherCalls();

	ArgosTxService serv(*mock_kineis);
	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	serv.start();
	inject_gps_location(true, 11.8768, -33.8232, t / 1000, true);
	unsigned int nominal = serv.get_last_schedule();

	// Five errors with nothing of ours in flight.
	for (unsigned int i = 0; i < 5; i++)
		mock_kineis->notify(KineisEventDeviceError({}));

	// Untouched schedule: no backoff was armed either, which is the other half
	// of "not counted".
	CHECK_EQUAL(nominal, serv.get_last_schedule());

	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock().checkExpectations();
}

// --- The backoff has to reach the burst modes too ---------------------------
// SURFACING_BURST drives itself with ArgosTxScheduler::schedule_at(), which
// writes the next TX instant directly. set_earliest_schedule -- the floor the
// backoff used to set -- is only read by schedule_periodic and schedule_prepass,
// so in the mode the turtles actually run the ladder was a log line and nothing
// else: the WARN announced 60000 ms while the next Doppler went out on the
// burst's own 5 s cadence.

TEST(ArgosTxService, BackoffSpacesTheSurfacingBurstDoppler) {
	configure_surfacing_burst();

	mock().expectOneCall("send").onObject(mock_kineis).ignoreOtherParameters();
	mock().ignoreOtherCalls();

	ArgosTxService serv(*mock_kineis);
	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	serv.start();
	inject_gps_location(true, 11.8768, -33.8232, t / 1000, true);

	// Dive, then surface: opens the Doppler burst.
	notify_underwater_state(true);
	t += 60000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	notify_underwater_state(false);

	// First Doppler goes out, and fails.
	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock_kineis->notify(KineisEventDeviceError({}));

	// SURFACING_BURST_INIT_S is 5 s. Without the hold the next Doppler lands
	// there and the announced backoff means nothing.
	CHECK_EQUAL(60000U, serv.get_last_schedule());
}

// ...and the way out is the one the mode already has. A GNSS fix promotes the
// burst to its phase 2 and clears the strike counter; it must clear the hold
// with it, or the beacon sits out the backoff with a fresh position in hand.
TEST(ArgosTxService, SurfacingBurstBackoffIsClearedByAGnssFix) {
	configure_surfacing_burst();

	mock().expectOneCall("send").onObject(mock_kineis).ignoreOtherParameters();
	mock().ignoreOtherCalls();

	ArgosTxService serv(*mock_kineis);
	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	serv.start();
	inject_gps_location(true, 11.8768, -33.8232, t / 1000, true);

	notify_underwater_state(true);
	t += 60000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	notify_underwater_state(false);

	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock_kineis->notify(KineisEventDeviceError({}));
	CHECK_EQUAL(60000U, serv.get_last_schedule());

	// A fix lands ten seconds later, well inside the hold.
	t += 10000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	inject_gps_location(true, 11.8768, -33.8232, t / 1000, true);

	CHECK_COMPARE(serv.get_last_schedule(), <, 60000U);
}

// The other exit the mode has: surfacing again.
TEST(ArgosTxService, SurfacingBurstBackoffIsClearedByASurfaceEvent) {
	configure_surfacing_burst();

	mock().expectOneCall("send").onObject(mock_kineis).ignoreOtherParameters();
	mock().ignoreOtherCalls();

	ArgosTxService serv(*mock_kineis);
	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	serv.start();
	inject_gps_location(true, 11.8768, -33.8232, t / 1000, true);

	notify_underwater_state(true);
	t += 60000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	notify_underwater_state(false);

	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock_kineis->notify(KineisEventDeviceError({}));
	CHECK_EQUAL(60000U, serv.get_last_schedule());

	// Down and up again inside the hold.
	notify_underwater_state(true);
	t += 20000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	notify_underwater_state(false);

	CHECK_COMPARE(serv.get_last_schedule(), <, 60000U);
}

// DOPPLER is exempt, deliberately, and this pins that it stays exempt. The mode
// has no surfacing event and no GNSS fix, so a hold could never be lifted
// early: it would only push messages out of the sequence with no way to bring
// them back. The strike counter still climbs and the suspension in
// service_initiate() still stops a wedged radio -- what is skipped is the
// spacing inside one sequence, which is the mode's own cadence.
TEST(ArgosTxService, DopplerModeIsDeliberatelyNotSpacedByTheBackoff) {
	configure_surfacing_burst(BaseArgosMode::DOPPLER);

	mock().expectOneCall("send").onObject(mock_kineis).ignoreOtherParameters();
	mock().ignoreOtherCalls();

	ArgosTxService serv(*mock_kineis);
	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	serv.start();
	inject_gps_location(true, 11.8768, -33.8232, t / 1000, true);

	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock_kineis->notify(KineisEventDeviceError({}));

	// The sequence cadence, not the 60 s ladder.
	CHECK_COMPARE(serv.get_last_schedule(), <, 60000U);
}

// The SmdSat autofallback cooldown is a refusal, not a fault, so it never
// touches the strike counter -- but it still has to space the burst, and on the
// same mechanism. It is SMD-only (KIM2 has no cooldown notion), which made it
// the one place where the same burst behaved differently on the two backends:
// the module was refusing for up to half an hour while the Doppler kept pinging
// every five seconds.
TEST(ArgosTxService, CooldownRejectAlsoSpacesTheSurfacingBurst) {
	configure_surfacing_burst();

	mock().expectOneCall("send").onObject(mock_kineis).ignoreOtherParameters();
	mock().ignoreOtherCalls();

	ArgosTxService serv(*mock_kineis);
	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	serv.start();
	inject_gps_location(true, 11.8768, -33.8232, t / 1000, true);

	notify_underwater_state(true);
	t += 60000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	notify_underwater_state(false);

	mock_kineis->test_set_cooldown_remaining_ms(300000);  // 5 min left

	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock_kineis->notify(KineisEventDeviceError({}));

	// Past the cooldown with the +1 s margin, not the 5 s burst cadence.
	CHECK_EQUAL(301000U, serv.get_last_schedule());
}

// DOPPLER skips the ladder, but the suspension is a different thing and it must
// still land. Without this the guard in service_initiate() reschedules through
// the burst cadence, hits the guard again seconds later and spins -- the same
// wake/log/skip loop the probe deadline exists to kill, only faster.
TEST(ArgosTxService, DopplerModeStillHonoursTheSuspension) {
	configure_surfacing_burst(BaseArgosMode::DOPPLER);

	mock().expectNCalls(3, "send").onObject(mock_kineis).ignoreOtherParameters();
	mock().ignoreOtherCalls();

	ArgosTxService serv(*mock_kineis);
	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	serv.start();
	inject_gps_location(true, 11.8768, -33.8232, t / 1000, true);

	for (unsigned int i = 0; i < 3; i++) {
		t += serv.get_last_schedule();
		fake_rtc->settime(t / 1000);
		fake_timer->set_counter(t);
		system_scheduler->run();
		mock_kineis->notify(KineisEventDeviceError({}));
	}

	// Parked on the probe deadline, far beyond any burst cadence.
	CHECK_COMPARE(serv.get_last_schedule(), >, DEVICE_ERROR_BACKOFF_MAX_MS_FOR_TEST);
}

// GNSS_CLOUDLOCATE_READY reschedules with immediate=true, which skips
// service_next_schedule_in_ms() and therefore every scheduling gate, the
// device-error hold included. That made it a third, unintended way out of a
// backoff: a CloudLocate becoming ready seconds after a failed TX transmitted
// straight through the hold. The authorised exits are the GNSS fix and the
// surface event, and this is neither.
TEST(ArgosTxService, CloudLocateReadyDoesNotBypassTheDeviceErrorHold) {
	configure_surfacing_burst();

	mock().expectOneCall("send").onObject(mock_kineis).ignoreOtherParameters();
	mock().ignoreOtherCalls();

	ArgosTxService serv(*mock_kineis);
	std::time_t t = 1652105502000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);

	serv.start();
	inject_gps_location(true, 11.8768, -33.8232, t / 1000, true);

	notify_underwater_state(true);
	t += 60000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	notify_underwater_state(false);

	t += serv.get_last_schedule();
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock_kineis->notify(KineisEventDeviceError({}));
	CHECK_EQUAL(60000U, serv.get_last_schedule());

	// A CloudLocate becomes ready inside the hold. It must not fire a TX: the
	// single `send` expected above has already been consumed, so a second one
	// fails the test.
	t += 5000;
	fake_rtc->settime(t / 1000);
	fake_timer->set_counter(t);
	inject_cloudlocate_ready();
	system_scheduler->run();
	mock().checkExpectations();
}
