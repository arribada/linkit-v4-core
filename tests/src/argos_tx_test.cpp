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


TEST_GROUP(ArgosTxService)
{
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
		fake_rtc->settime(1580083200); // 27/01/2020 00:00:00
		fake_timer = new FakeTimer;
		system_timer = fake_timer; // linux_timer;
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

	GPSLogEntry make_gps_location(bool is_valid=true, double longitude=0, double latitude=0, std::time_t t=0, bool is_3d_fix = false, int32_t hMSL=0, int32_t gSpeed=0, uint16_t batt=4200) {
		GPSLogEntry log{};  // Zero-init to avoid UB on unset fields (e.g. headMot)
		log.info.valid = is_valid;
		log.info.lon = longitude;
		log.info.lat = latitude;
		log.info.schedTime = t;
		log.info.fixType = is_3d_fix ? 3 : 2;
		log.info.hMSL = hMSL;
		log.info.batt_voltage = batt;
		log.info.gSpeed = gSpeed;

		if (t == 0)
			t = rtc->gettime();

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

	void inject_gps_location(bool is_valid=true, double longitude=0, double latitude=0, std::time_t t=0, bool is_3d_fix = false, int32_t hMSL=0, int32_t gSpeed=0, uint16_t batt=4200) {
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
};

TEST(ArgosTxService, DepthPileFillsAndEmpties)
{
	DepthPile<GPSLogEntry> dp;
	std::vector<GPSLogEntry*> v;
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
	CHECK_EQUAL(23, v.at(0)->info.day); // Should be most recent

	// Retrieve entire depth pile (in blocks of 3 — LDA2 long packet carries 3
	// fixes to leave room for the firmware-embedded CRC8 at byte 23)
	for (unsigned int i = 0; i < 8; i++) {
		v = dp.retrieve(24);
		CHECK_EQUAL(3, v.size());
		CHECK_EQUAL(21-(i*3), dp.eligible());
		for (unsigned j = 0; j < 3; j++) {
			CHECK_EQUAL(24-((i+1)*3)+j, v.at(j)->info.day);
		}
	}

	// Check depth pile returns empty vector
	v = dp.retrieve_latest();
	CHECK_EQUAL(0, v.size());
	v = dp.retrieve(24);
	CHECK_EQUAL(0, v.size());
}

TEST(ArgosTxService, DepthPile1)
{
	DepthPile<GPSLogEntry> dp;
	std::vector<GPSLogEntry*> v;
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
	CHECK_EQUAL(23, v.at(0)->info.day); // Should be most recent

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

TEST(ArgosTxService, DutyCycleCalculation)
{
	unsigned int mask;
	mask = 0xFFFFFF;
	for (unsigned int i = 0; i < 24; i++)
		CHECK_EQUAL((bool)((mask >> (23-i))&1), ArgosTxScheduler::is_in_duty_cycle(i*3600*1000, mask));
	mask = 0xEEEEEE;
	for (unsigned int i = 0; i < 24; i++)
		CHECK_EQUAL((bool)((mask >> (23-i))&1), ArgosTxScheduler::is_in_duty_cycle(i*3600*1000, mask));
	mask = 0;
	for (unsigned int i = 0; i < 24; i++)
		CHECK_EQUAL((bool)((mask >> (23-i))&1), ArgosTxScheduler::is_in_duty_cycle(i*3600*1000, mask));
	mask = 0x123456;
	for (unsigned int i = 0; i < 24; i++)
		CHECK_EQUAL((bool)((mask >> (23-i))&1), ArgosTxScheduler::is_in_duty_cycle(i*3600*1000, mask));
}

TEST(ArgosTxService, SchedulerLegacyNoJitter)
{
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
TEST(ArgosTxService, SchedulerZeroIntervalNoHang)
{
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

TEST(ArgosTxService, SchedulerLegacyNoJitterWithEarliestTxSet)
{
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

TEST(ArgosTxService, SchedulerLegacyWithJitter)
{
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

TEST(ArgosTxService, SchedulerDutyCycleNoJitter)
{
	ArgosTxScheduler sched;
	ArgosConfig config;

	config.argos_tx_jitter_en = false;
	config.tx_interval_s = 10;
	config.duty_cycle = 0x1;

	unsigned int result = sched.schedule_duty_cycle(config, 0);
	CHECK_EQUAL(23*3600*1000, result);
	sched.notify_tx_complete();
	result = sched.schedule_duty_cycle(config, 23*3600);
	CHECK_EQUAL(10000U, result);
	sched.notify_tx_complete();
}

TEST(ArgosTxService, BuildLongCertificationPacket)
{
	unsigned int size_bits;
	// LONG_PACKET_BYTES is 24 (LDA2 frame); certification truncates to that size.
	std::string x = ArgosPacketBuilder::build_certification_packet("FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF", size_bits);
	CHECK_EQUAL(ArgosPacketBuilder::LDA2_FRAME_BITS, size_bits);
	CHECK_EQUAL(ArgosPacketBuilder::LDA2_FRAME_BYTES, x.size());
	CHECK_EQUAL("\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF"s, x);
}

TEST(ArgosTxService, BuildShortCertificationPacket)
{
	unsigned int size_bits;
	// SHORT_PACKET_BYTES = 12, so input must be ≤12 bytes (24 hex chars)
	std::string x = ArgosPacketBuilder::build_certification_packet("FFFFFFFFFFFFFFFFFFFF", size_bits); // 10 bytes
	CHECK_EQUAL(96, size_bits); // SHORT_PACKET_BITS = 96
	CHECK_EQUAL("\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\x00\x00"s, x); // Padded to 12 bytes
}

TEST(ArgosTxService, BuildShortGNSSPacket)
{
	unsigned int size_bits;
	GPSLogEntry e = make_gps_location(1, 12.3, 44.4, 1652105502);
	std::vector<GPSLogEntry*> v({&e});
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
static void check_lda2_long_packet(const std::string& packet, const std::string& expected_data_prefix) {
	CHECK_EQUAL(ArgosPacketBuilder::LDA2_FRAME_BYTES, packet.size());
	std::string hex = Binascii::hexlify(packet);
	std::string prefix = hex.substr(0, expected_data_prefix.size());
	CHECK_EQUAL(expected_data_prefix, prefix);
	unsigned char expected_crc = CRC8::checksum(packet, ArgosPacketBuilder::LDA2_DATA_BITS);
	CHECK_EQUAL((unsigned int)expected_crc, (unsigned int)(unsigned char)packet[23]);
}

TEST(ArgosTxService, BuildLongGNSSPacket)
{
	unsigned int size_bits;
	GPSLogEntry e = make_gps_location(1, 12.3, 44.4, 1652105502);
	// Long packet: header 000 (Type 0, shared with Short Packet, disambiguated by size 24B vs 12B).
	// Packs up to 3 fixes (down from 4) to leave 8 bits for CRC at byte 23.
	std::vector<GPSLogEntry*> v({&e, &e});
	std::string x = ArgosPacketBuilder::build_gnss_packet(v, false, false, BaseDeltaTimeLoc::DELTA_T_10MIN, size_bits);
	CHECK_EQUAL(ArgosPacketBuilder::LDA2_FRAME_BITS, size_bits);
	check_lda2_long_packet(x, "097166C6600781E002584D8CC00F03C7FFFFFFFFFF");
	v = {&e, &e, &e};
	x = ArgosPacketBuilder::build_gnss_packet(v, false, false, BaseDeltaTimeLoc::DELTA_T_10MIN, size_bits);
	check_lda2_long_packet(x, "097166C6600781E002584D8CC00F03C1B19801E078");
	// 4 fixes provided → 4th is silently dropped (LDA2 budget = 3 fixes after CRC).
	v = {&e, &e, &e, &e};
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
TEST(ArgosTxService, BuildLongGNSSPacketNonUniformKeepsLegacyLayout)
{
	unsigned int size_bits;
	GPSLogEntry newest = make_gps_location(1, 12.3, 44.4, 1652106702);
	GPSLogEntry mid    = make_gps_location(1, 12.3, 44.4, 1652105502); // newest - 1200 (2*600)
	GPSLogEntry oldest = make_gps_location(1, 12.3, 44.4, 1652104902); // mid    -  600 (1*600)
	std::vector<GPSLogEntry*> v({&oldest, &mid, &newest}); // chronological asc; builder reverses
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
TEST(ArgosTxService, BuildLongGNSSPacketUniformKeepsLegacyLayout)
{
	unsigned int size_bits;
	GPSLogEntry a = make_gps_location(1, 12.3, 44.4, 1652106702);
	GPSLogEntry b = make_gps_location(1, 12.3, 44.4, 1652106102); // a - 600 (1*600)
	GPSLogEntry c = make_gps_location(1, 12.3, 44.4, 1652105502); // b - 600 (1*600)
	std::vector<GPSLogEntry*> v({&c, &b, &a}); // chronological asc; builder reverses to a,b,c
	std::string x = ArgosPacketBuilder::build_gnss_packet(v, false, false, BaseDeltaTimeLoc::DELTA_T_10MIN, size_bits);
	CHECK_EQUAL(0x00, (unsigned int)(unsigned char)x[21]);
	CHECK_EQUAL(0x00, (unsigned int)(unsigned char)x[22]);
}

// First-message gate (non-RSPB): without a valid GPS fix yet this session, the
// 2026-08 — INTENTION INVERSEE. Le verrou du premier message tenait TOUTE
// emission tant qu'aucun fix GPS n'etait tombe depuis la mise sous tension,
// heartbeat de presence compris. Une balise qui redemarrait sans jamais
// retrouver de fix (recepteur en panne, ciel bouche) disparaissait donc
// definitivement des ecrans. En LEGACY la balise emet a sa periode et c'est le
// segment sol qui reconstruit la position: son horloge n'entre pas dans le
// calcul, rien ne justifie de la faire taire. RSPB en etait deja exempte.
//
// Ce test verifie desormais le contraire de son ancetre: le heartbeat 0xFF
// DOIT partir meme si aucun fix n'est jamais tombe.
TEST(ArgosTxService, LegacyNoFixHeartbeatSentWithoutAnyFix)
{
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::LEGACY);
	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, (unsigned int)0x01234567U);
	fake_config_store->write_param(ParamID::LB_EN, false);
	fake_config_store->write_param(ParamID::TR_NOM, (unsigned int)10);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, false);

	ArgosTxService serv(*mock_kineis);
	std::time_t t = 1652105502000;  // ms
	fake_rtc->settime(t/1000);   // clock IS set (e.g. DTE/pseudo-RTC) but never by GPS
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// Uniquement des NO_FIX, aucun fix reel: le heartbeat doit tout de meme
	// etre emis (SHORT, 96 bits) — la balise signale sa presence.
	inject_gps_nofix(t/1000);
	mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
			withUnsignedIntParameter("size_bits", 96);
	t += serv.get_last_schedule();
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock_kineis->notify(KineisEventTxComplete({}));
	mock().checkExpectations();
}

// After a real GPS fix corrects the clock (unlocks TX), a subsequent no-fix
// cycle transmits the NO_FIX 0xFF heartbeat.
TEST(ArgosTxService, LegacyNoFixHeartbeatAfterFix)
{
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::LEGACY);
	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, (unsigned int)0x01234567U);
	fake_config_store->write_param(ParamID::LB_EN, false);
	fake_config_store->write_param(ParamID::TR_NOM, (unsigned int)10);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, false);

	ArgosTxService serv(*mock_kineis);
	std::time_t t = 1652105502000;  // ms
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// Real fix unlocks TX and is transmitted (SHORT, 96 bits)
	inject_gps_location(1, 11.8768, -33.8232, t/1000);
	mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
			withUnsignedIntParameter("size_bits", 96);
	t += serv.get_last_schedule();
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock_kineis->notify(KineisEventTxComplete({}));

	// Now a no-fix cycle -> NO_FIX heartbeat cached and transmitted as SHORT 0xFF
	inject_gps_nofix(t/1000);
	mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
			withUnsignedIntParameter("size_bits", 96);
	t += serv.get_last_schedule();
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock().checkExpectations();
}

// v3 grid fillers: the NO_FIX entry stays in the pile even after a real fix
// arrives (no purge in LEGACY/DUTY_CYCLE/PASS_PREDICTION) so it keeps its slot
// on the delta_time_loc grid — the next packet is a LONG (192-bit) carrying
// the real fix plus the 0xFF filler.
TEST(ArgosTxService, LegacyNoFixFillerKeptAfterRealFix)
{
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::LEGACY);
	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_4);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, (unsigned int)0x01234567U);
	fake_config_store->write_param(ParamID::LB_EN, false);
	fake_config_store->write_param(ParamID::TR_NOM, (unsigned int)10);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, false);

	ArgosTxService serv(*mock_kineis);
	std::time_t t = 1652105502000;  // ms
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// Cycle GPS 1: pas de fix -> le heartbeat est mis en cache ET emis (SHORT,
	// 96 bits). Avant 2026-08 le verrou du premier message imposait le silence
	// ici; il a ete supprime, une balise sans fix doit continuer a se signaler.
	inject_gps_nofix(t/1000);
	mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
			withUnsignedIntParameter("size_bits", 96);
	t += serv.get_last_schedule();
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock_kineis->notify(KineisEventTxComplete({}));
	mock().checkExpectations();

	// GPS cycle 2: real fix -> unlocks TX; the FIX purge keeps the NO_FIX filler
	// (include_no_fix=false in planned modes) -> pile = [NO_FIX, FIX] ->
	// LONG packet (192 bits) with the fix + 0xFF filler
	inject_gps_location(1, 11.8768, -33.8232, t/1000);
	mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
			withUnsignedIntParameter("size_bits", 192);
	t += serv.get_last_schedule();
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock_kineis->notify(KineisEventTxComplete({}));
	mock().checkExpectations();
}


TEST(ArgosTxService, TxCounterIncrements)
{
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

TEST(ArgosTxService, TimeSyncBurstPosFix)
{
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
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// First TX is time sync burst
	mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
			withUnsignedIntParameter("size_bits", 96);

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

TEST(ArgosTxService, TimeSyncBurstNoPosFix)
{
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
	fake_rtc->settime(t/1000);
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

TEST(ArgosTxService, TimeSyncBurstNoPosOrTimeFix)
{
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

TEST(ArgosTxService, LegacyTxServiceInv)
{
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
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	inject_gps_location(1, -2.117964, 51.376382, t/1000, false, 0, 162, 4424);
	//inject_gps_location(1, 11.8768, -33.8232, t);
	//inject_gps_location(1, 11.8768, -33.8232, t);
	//inject_gps_location(1, 11.8768, -33.8232, t);
	system_scheduler->run();

	// Subsequent TX will be long packets
	for (unsigned int i = 0; i < 1; i++) {
		mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
				withUnsignedIntParameter("size_bits", 96);

		t += serv.get_last_schedule();
		fake_rtc->settime(t/1000);
		fake_timer->set_counter(t);
		system_scheduler->run();

		mock_kineis->notify(KineisEventTxComplete({}));
	}
}


TEST(ArgosTxService, LegacyTxLowBattery)
{
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
	fake_rtc->settime(t/1000);
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
		mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
				withUnsignedIntParameter("size_bits", 96);

		t += serv.get_last_schedule();
		fake_rtc->settime(t/1000);
		fake_timer->set_counter(t);
		system_scheduler->run();

		mock_kineis->notify(KineisEventTxComplete({}));
	}
}

TEST(ArgosTxService, LegacyTxOutOfZone)
{
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
	fake_rtc->settime(t/1000);
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
		mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
				withUnsignedIntParameter("size_bits", 96);

		t += serv.get_last_schedule();
		fake_rtc->settime(t/1000);
		fake_timer->set_counter(t);
		system_scheduler->run();

		mock_kineis->notify(KineisEventTxComplete({}));
	}
}

TEST(ArgosTxService, TxServiceCancelledByUnderwaterBeforeTx)
{
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
	fake_rtc->settime(t/1000);
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
	CHECK_EQUAL(dry_time_before_tx*1000, serv.get_last_schedule());

	// Should now transmit (TCXO warmup skipped on first TX after submerge)
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
			withUnsignedIntParameter("size_bits", 192);
	t += serv.get_last_schedule();
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
}

TEST(ArgosTxService, TxServiceCancelledDuringTx)
{
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
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	inject_gps_location(1, 11.8768, -33.8232, t);
	inject_gps_location(1, 11.8768, -33.8232, t);
	inject_gps_location(1, 11.8768, -33.8232, t);
	inject_gps_location(1, 11.8768, -33.8232, t);
	system_scheduler->run();

	mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
			withUnsignedIntParameter("size_bits", 192);

	// TX should start
	t += serv.get_last_schedule();
	fake_rtc->settime(t/1000);
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
	CHECK_EQUAL(dry_time_before_tx*1000, serv.get_last_schedule());

	// Should now transmit. Depth pile has 4 fixes; with max_messages=3 the second slot
	// only carries 1 fix, so the 2nd send is a SHORT (LDK 96-bit) packet.
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
			withUnsignedIntParameter("size_bits", 96);
	t += serv.get_last_schedule();
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	// TX complete restores TCXO warmup
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	mock_kineis->notify(KineisEventTxComplete({}));
}


TEST(ArgosTxService, LegacyTxServiceDepthPile1)
{
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
	fake_rtc->settime(t/1000);
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
		mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
				withUnsignedIntParameter("size_bits", 96);

		t += serv.get_last_schedule();
		fake_rtc->settime(t/1000);
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
TEST(ArgosTxService, LegacyNonAdaptiveLdkMasterCapsToSingleFix)
{
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
	fake_rtc->settime(t/1000);
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
		mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDK).
				withUnsignedIntParameter("size_bits", 96);

		t += serv.get_last_schedule();
		fake_rtc->settime(t/1000);
		fake_timer->set_counter(t);
		system_scheduler->run();

		mock_kineis->notify(KineisEventTxComplete({}));
	}
}

// Regression (2026-06-17): a non-adaptive master accidentally left on VLDA4
// (24-bit budget) can't hold even a single 96-bit GNSS fix. Instead of a silent
// KIM2 oversize drop, the service must fall back to LDA2 (when its RCONF is
// provisioned) and ship the fix there. (User policy: on overflow → LDA2.)
TEST(ArgosTxService, LegacyNonAdaptiveVlda4MasterFallsBackToLda2)
{
	const std::string lda2_rconf = "00112233445566778899aabbccddeeff"; // 32 hex chars

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
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	inject_gps_location(1, 11.8768, -33.8232, t);
	inject_gps_location(1, 11.8768, -33.8232, t);
	system_scheduler->run();

	// First TX: 96-bit fix doesn't fit VLDA4 → fall back to LDA2 (switch), send there.
	mock().expectOneCall("switch_modulation").onObject(mock_kineis)
			.withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2)
			.withStringParameter("rconf", lda2_rconf.c_str());
	mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
			withUnsignedIntParameter("size_bits", 96);

	t += serv.get_last_schedule();
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	system_scheduler->run();

	mock_kineis->notify(KineisEventTxComplete({}));
}

TEST(ArgosTxService, UnderwaterFor24HoursBeforeTx)
{
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
	fake_rtc->settime(t/1000);
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
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	system_scheduler->run();

	// Inject surfaced event
	notify_underwater_state(false);

	// Next schedule should be equal to dry time before TX since the last TX was deferred
	unsigned int dry_time_before_tx = fake_config_store->read_param<unsigned int>(ParamID::DRY_TIME_BEFORE_TX);
	CHECK_EQUAL(dry_time_before_tx*1000, serv.get_last_schedule());

	// Should now transmit (TCXO warmup skipped on first TX after submerge)
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
			withUnsignedIntParameter("size_bits", 192);
	t += serv.get_last_schedule();
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
}

TEST(ArgosTxService, LastTxIsUpdated)
{
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
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// First TX is time sync burst
	mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
			withUnsignedIntParameter("size_bits", 96);

	inject_gps_location(1, 11.8768, -33.8232, t);
	system_scheduler->run();

	mock_kineis->notify(KineisEventTxComplete({}));

	std::time_t last_tx = fake_config_store->read_param<std::time_t>(ParamID::LAST_TX);
	CHECK_EQUAL(1652105502U, (unsigned int)last_tx);

	mock().expectOneCall("power_off_immediate").onObject(mock_kineis);
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	serv.stop();
}

TEST(ArgosTxService, DPHardFaultPartialDepthPile)
{
	DepthPile<GPSLogEntry> dp;
	std::vector<GPSLogEntry*> v;
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

TEST(ArgosTxService, UnderwaterFor24HoursDryTimeZero)
{
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
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	inject_gps_location(1, 11.8768, -33.8232, t);

	// Do initial transmit
	mock().expectOneCall("send").onObject(mock_kineis).ignoreOtherParameters();
	t += serv.get_last_schedule();
	fake_rtc->settime(t/1000);
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
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	system_scheduler->run();

	// Inject surfaced event
	notify_underwater_state(false);

	CHECK_EQUAL(0, serv.get_last_schedule());

	// Should now transmit (TCXO warmup skipped on first TX after submerge)
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("send").onObject(mock_kineis).ignoreOtherParameters();
	t += serv.get_last_schedule();
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
}

TEST(ArgosTxService, SurfacingBurstDopplerPhase)
{
	// Test SURFACING_BURST mode: verify progressive Doppler intervals
	BaseArgosMode mode = BaseArgosMode::SURFACING_BURST;
	unsigned int dry_time = 1; // 1s dry time to separate scheduling from notification

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
	fake_rtc->settime(t/1000);
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
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	// Surface — first Doppler scheduled immediately (0ms)
	notify_underwater_state(false);
	CHECK_EQUAL(0U, serv.get_last_schedule());

	// Fire first Doppler TX
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
			withUnsignedIntParameter("size_bits", 24);
	system_scheduler->run();

	// Complete TX → reschedule with INIT interval (5s)
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	mock_kineis->notify(KineisEventTxComplete({}));
	CHECK_EQUAL(5000U, serv.get_last_schedule());

	// Advance and fire second Doppler
	mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
			withUnsignedIntParameter("size_bits", 24);
	t += serv.get_last_schedule();
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	system_scheduler->run();

	mock_kineis->notify(KineisEventTxComplete({}));
	// Third message: init + 1*step = 5 + 10 = 15s
	CHECK_EQUAL(15000U, serv.get_last_schedule());
}

TEST(ArgosTxService, SurfacingBurstSwitchToGNSS)
{
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
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// Dive
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("power_off_immediate").onObject(mock_kineis);
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	notify_underwater_state(true);

	t += 60000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	// Surface
	notify_underwater_state(false);

	// Fire first Doppler TX
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
			withUnsignedIntParameter("size_bits", 24);
	system_scheduler->run();

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	mock_kineis->notify(KineisEventTxComplete({}));

	// Inject GPS fix → should switch to GNSS phase and reschedule
	inject_gps_location(true, 11.8768, -33.8232, t/1000);
	CHECK_FALSE(Service::SCHEDULE_DISABLED == serv.get_last_schedule());

	// Fire TX → should be GNSS short packet (96 bits), not Doppler (24 bits)
	mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
			withUnsignedIntParameter("size_bits", 96);
	t += serv.get_last_schedule();
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
}

TEST(ArgosTxService, SurfacingBurstResetOnDive)
{
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
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// First dive/surface
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("power_off_immediate").onObject(mock_kineis);
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	notify_underwater_state(true);
	t += 60000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	notify_underwater_state(false);
	CHECK_EQUAL(0U, serv.get_last_schedule());  // First burst msg immediate

	// Fire first TX
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
			withUnsignedIntParameter("size_bits", 24);
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
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	notify_underwater_state(false);
	CHECK_EQUAL(0U, serv.get_last_schedule());  // Immediate again (reset)
}

TEST(ArgosTxService, SurfacingBurstFirstGnssTxImmediate)
{
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
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// Dive
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("power_off_immediate").onObject(mock_kineis);
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	notify_underwater_state(true);
	t += 60000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	// Surface
	notify_underwater_state(false);
	CHECK_EQUAL(0U, serv.get_last_schedule());  // Doppler #1 immediate

	// Fire Doppler TX #1
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
			withUnsignedIntParameter("size_bits", 24);
	system_scheduler->run();

	// TX complete
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	mock_kineis->notify(KineisEventTxComplete({}));

	// Advance time past the spacing guard window (surfacing_burst_init_s = 5s)
	// so the first GNSS TX after fix can fire immediately. Without this advance
	// the spacing guard correctly defers TX#1 to maintain ≥ 5 s between any
	// pair of TX (2026-05 fix: protects TCXO + CLS rate-limit).
	t += 6000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	// Inject GPS fix → switch to GNSS phase
	inject_gps_location(true, 11.8768, -33.8232, t/1000);

	// GNSS TX #1 fires immediately now that the spacing window has elapsed.
	CHECK_EQUAL(0U, serv.get_last_schedule());

	// Fire GNSS TX
	mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
			withUnsignedIntParameter("size_bits", 96);
	t += serv.get_last_schedule();
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
}

TEST(ArgosTxService, SurfacingBurstDopplerMaxMsg)
{
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
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// Dive and surface
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("power_off_immediate").onObject(mock_kineis);
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	notify_underwater_state(true);
	t += 60000;
	fake_rtc->settime(t/1000);
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
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock_kineis->notify(KineisEventTxComplete({}));

	// After 2 Doppler, burst should stop (SCHEDULE_DISABLED)
	CHECK_TRUE(Service::SCHEDULE_DISABLED == serv.get_last_schedule());
}

TEST(ArgosTxService, SurfacingBurstAdaptiveModulationPreSwitch)
{
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
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// Dive
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("power_off_immediate").onObject(mock_kineis);
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	notify_underwater_state(true);
	t += 60000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	// Surface → Doppler VLDA4
	notify_underwater_state(false);

	// Fire Doppler TX (VLDA4 mode)
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("switch_modulation").onObject(mock_kineis)
		.withUnsignedIntParameter("mode", (unsigned int)KineisModulation::VLDA4)
		.withStringParameter("rconf", "550b4bec21009c7a7b5bebaa937cdb41");
	mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::VLDA4).
			withUnsignedIntParameter("size_bits", 24);
	system_scheduler->run();

	// TX complete → TCXO restored, no pre-switch needed (already VLDA4)
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	mock_kineis->notify(KineisEventTxComplete({}));

	// Inject GPS fix
	inject_gps_location(true, 11.8768, -33.8232, t/1000);

	// Fire GNSS TX → should switch to LDK
	mock().expectOneCall("switch_modulation").onObject(mock_kineis)
		.withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDK)
		.withStringParameter("rconf", "03921fb104b92859209b18abd009de96");
	mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDK).
			withUnsignedIntParameter("size_bits", 96);
	t += serv.get_last_schedule();
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	system_scheduler->run();

	// TX complete → should pre-switch back to VLDA4
	mock().expectOneCall("switch_modulation").onObject(mock_kineis)
		.withUnsignedIntParameter("mode", (unsigned int)KineisModulation::VLDA4)
		.withStringParameter("rconf", "550b4bec21009c7a7b5bebaa937cdb41");
	mock_kineis->notify(KineisEventTxComplete({}));

	// Verify device is back to VLDA4
	CHECK_EQUAL((unsigned int)KineisModulation::VLDA4, (unsigned int)mock_kineis->get_current_modulation());
}

TEST(ArgosTxService, DepthPileManagerSensorTimeout)
{
	bool enable = true;
	BaseSensorEnableTxMode mode = BaseSensorEnableTxMode::ONESHOT;
	fake_config_store->write_param(ParamID::ALS_SENSOR_ENABLE, enable);
	fake_config_store->write_param(ParamID::ALS_SENSOR_ENABLE_TX_MODE, mode);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t/1000);
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
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	system_scheduler->run();

	CHECK_TRUE(man.eligible());
}

TEST(ArgosTxService, DepthPileManagerSensorRx)
{
	bool enable = true;
	BaseSensorEnableTxMode mode = BaseSensorEnableTxMode::ONESHOT;
	fake_config_store->write_param(ParamID::ALS_SENSOR_ENABLE, enable);
	fake_config_store->write_param(ParamID::ALS_SENSOR_ENABLE_TX_MODE, mode);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t/1000);
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

TEST(ArgosTxService, DepthPileManagerTestSensorValueConversion)
{
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
static void check_lda2_sensor_packet(const std::string& packet, const std::string& expected_data_prefix) {
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

	als.port[0] = 10000; // 10000 lux
	ph.port[0] = 7000; // 7.0 ph
	pressure.port[0] = 1000; // 1.0 bar
	pressure.port[1] = 4000; // 0C
	sea_temp.port[0] = 126000; // 0C

	// LDA2 sensor packets are now always 24 bytes (192-bit frame) with header 001 (Type 1)
	// at bit 0, 5-bit sensor mask at bit 78, then sensor data, then CRC8 at byte 23.
	// Mask bits (MSB-first): ALS, PH, Pressure, SeaTemp, AXL.
	x = ArgosPacketBuilder::build_sensor_packet(&e, nullptr, nullptr, nullptr, nullptr, nullptr, false, false, size_bits);
	CHECK_EQUAL(ArgosPacketBuilder::LDA2_FRAME_BITS, size_bits);
	check_lda2_sensor_packet(x, "297166C6600781E00258");                              // mask=00000
	x = ArgosPacketBuilder::build_sensor_packet(&e, &als, &ph, &pressure, &sea_temp, nullptr, false, false, size_bits);
	CHECK_EQUAL(ArgosPacketBuilder::LDA2_FRAME_BITS, size_bits);
	check_lda2_sensor_packet(x, "297166C6600781E0025BC27106D601F41F401EC3");          // mask=11110 (all 4 non-AXL)
	x = ArgosPacketBuilder::build_sensor_packet(&e, nullptr, &ph, &pressure, &sea_temp, nullptr, false, false, size_bits);
	CHECK_EQUAL(ArgosPacketBuilder::LDA2_FRAME_BITS, size_bits);
	check_lda2_sensor_packet(x, "297166C6600781E00259CDAC03E83E803D86");              // mask=01110 (no ALS)
	x = ArgosPacketBuilder::build_sensor_packet(&e, &als, nullptr, &pressure, &sea_temp, nullptr, false, false, size_bits);
	CHECK_EQUAL(ArgosPacketBuilder::LDA2_FRAME_BITS, size_bits);
	check_lda2_sensor_packet(x, "297166C6600781E0025AC271007D07D007B0C0");            // mask=10110 (no PH)
	x = ArgosPacketBuilder::build_sensor_packet(&e, &als, &ph, nullptr, &sea_temp, nullptr, false, false, size_bits);
	CHECK_EQUAL(ArgosPacketBuilder::LDA2_FRAME_BITS, size_bits);
	check_lda2_sensor_packet(x, "297166C6600781E0025B427106D603D860");                // mask=11010 (no Pressure)
	x = ArgosPacketBuilder::build_sensor_packet(&e, &als, &ph, &pressure, nullptr, nullptr, false, false, size_bits);
	CHECK_EQUAL(ArgosPacketBuilder::LDA2_FRAME_BITS, size_bits);
	check_lda2_sensor_packet(x, "297166C6600781E0025B827106D601F41F40");              // mask=11100 (no SeaTemp)
}

TEST(ArgosTxService, BuildSensorPacketSeaTemp) {
	unsigned int size_bits;
	GPSLogEntry e = make_gps_location(1, 12.3, 44.4, 1652105502);
	ServiceSensorData sea_temp;
	std::string x;

	sea_temp.port[0] = 147100; // 21.1C

	x = ArgosPacketBuilder::build_sensor_packet(&e, nullptr, nullptr, nullptr, &sea_temp, nullptr, false, false, size_bits);
	CHECK_EQUAL(ArgosPacketBuilder::LDA2_FRAME_BITS, size_bits);
	check_lda2_sensor_packet(x, "297166C6600781E00258423E9C");                        // mask=00010 (SeaTemp only)
}


TEST(ArgosTxService, DepthPileManagerTestThermistorConversion)
{
	bool enable = true;
	BaseSensorEnableTxMode mode = BaseSensorEnableTxMode::ONESHOT;
	fake_config_store->write_param(ParamID::THERMISTOR_SENSOR_ENABLE, enable);
	fake_config_store->write_param(ParamID::THERMISTOR_SENSOR_ENABLE_TX_MODE, mode);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t/1000);
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
	CHECK_TRUE(size_bits > 83); // Must be larger than GPS-only (83 bits)
}

TEST(ArgosTxService, BuildSensorPacketOutOfZone) {
	unsigned int size_bits;
	GPSLogEntry e = make_gps_location(1, 12.3, 44.4, 1652105502);
	std::string x;

	// LDA2 sensor packets are always emitted as 192-bit frames since CRC8 lives at byte 23.
	x = ArgosPacketBuilder::build_sensor_packet(&e, nullptr, nullptr, nullptr, nullptr, nullptr, true, false, size_bits);
	CHECK_EQUAL(ArgosPacketBuilder::LDA2_FRAME_BITS, size_bits);
	CHECK_EQUAL(ArgosPacketBuilder::LDA2_FRAME_BYTES, x.size());

	x = ArgosPacketBuilder::build_sensor_packet(&e, nullptr, nullptr, nullptr, nullptr, nullptr, false, true, size_bits);
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

// Regression 2026-08 — PASS_PREDICTION ignorait la modulation configuree.
//
// Le service resout pourtant correctement `adaptive ? LDA2 :
// resolve_non_adaptive_modulation()` juste avant d'appeler l'ordonnanceur — le
// correctif de modulation du 2026-05-25 l'affirme meme dans son commentaire.
// Mais schedule_prepass recevait m_scheduled_mode par REFERENCE et l'ecrasait
// sans condition avec LDA2 une poignee de lignes plus loin. Resultat: une
// configuration LDK + adaptatif desactive partait en LDA2, la trame etait
// dimensionnee pour la mauvaise modulation, et ensure_modulation reecrivait le
// RCONF maitre a chaque emission (usure flash + configuration utilisateur
// silencieusement annulee).
//
// Le parametre de sortie a ete supprime: choisir la modulation est une decision
// de politique, elle appartient au service.
TEST(ArgosTxService, PassPredictHonoursConfiguredModulation)
{
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::PASS_PREDICTION);
	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, (unsigned int)0x01234567U);
	fake_config_store->write_param(ParamID::ARGOS_ADAPTIVE_MODULATION, false);
	fake_config_store->write_param(ParamID::LB_EN, false);
	fake_config_store->write_param(ParamID::TR_NOM, (unsigned int)10);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, false);

	// Le module porte LDK dans son RCONF maitre — c'est ce que
	// resolve_non_adaptive_modulation() doit renvoyer.
	mock_kineis->test_set_current_modulation(KineisModulation::LDK);

	// Un passage satellite tres large, pour que la fenetre soit forcement trouvee.
	BasePassPredict pass_predict = {
		/* version_code */ 0,
		1,
		{
			{ 0xA, 5, SAT_DNLK_ON_WITH_A3, SAT_UPLK_ON_WITH_A3,
			  { 2020, 1, 26, 22, 59, 44 }, 7195.550f, 98.5444f, 327.835f,
			  -25.341f, 101.3587f, 0.00f }
		}
	};
	fake_config_store->write_pass_predict(pass_predict);

	ArgosTxService serv(*mock_kineis);
	std::time_t t = 1580083200000;  // ms
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	inject_gps_location(1, 11.8768, -33.8232, t/1000);

	// LDK et non LDA2: c'est tout l'objet de la regression.
	mock().expectOneCall("send").onObject(mock_kineis)
		.withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDK)
		.ignoreOtherParameters();
	t += serv.get_last_schedule();
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock().checkExpectations();
}

IGNORE_TEST(ArgosTxService, PassPredictWithSensorDataPayload)
{
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

	BasePassPredict pass_predict = {
		/* version_code */ 0,
		7,
		{
		    { 0xA, 5, SAT_DNLK_ON_WITH_A3, SAT_UPLK_ON_WITH_A3, { 2020, 1, 26, 22, 59, 44 }, 7195.550f, 98.5444f, 327.835f, -25.341f, 101.3587f, 0.00f },
			{ 0x9, 3, SAT_DNLK_OFF, SAT_UPLK_ON_WITH_A3, { 2020, 1, 26, 22, 33, 39 }, 7195.632f, 98.7141f, 344.177f, -25.340f, 101.3600f, 0.00f },
			{ 0xB, 7, SAT_DNLK_ON_WITH_A3, SAT_UPLK_ON_WITH_A3, { 2020, 1, 26, 23, 29, 29 }, 7194.917f, 98.7183f, 330.404f, -25.336f, 101.3449f, 0.00f },
			{ 0x5, 0, SAT_DNLK_OFF, SAT_UPLK_ON_WITH_A2, { 2020, 1, 26, 23, 50, 6 }, 7180.549f, 98.7298f, 289.399f, -25.260f, 101.0419f, -1.78f },
			{ 0x8, 0, SAT_DNLK_OFF, SAT_UPLK_ON_WITH_A2, { 2020, 1, 26, 22, 12, 6 }, 7226.170f, 99.0661f, 343.180f, -25.499f, 102.0039f, -1.80f },
			{ 0xC, 6, SAT_DNLK_OFF, SAT_UPLK_ON_WITH_A3, { 2020, 1, 26, 22, 3, 52 }, 7226.509f, 99.1913f, 291.936f, -25.500f, 102.0108f, -1.98f },
			{ 0xD, 4, SAT_DNLK_ON_WITH_A3, SAT_UPLK_ON_WITH_A3, { 2020, 1, 26, 22, 3, 53 }, 7160.246f, 98.5358f, 118.029f, -25.154f, 100.6148f, 0.00f }
		}
	};

	fake_config_store->write_pass_predict(pass_predict);

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1580083200000;
	fake_rtc->settime(t/1000);
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
		fake_rtc->settime(t/1000);
		fake_timer->set_counter(t);
		system_scheduler->run();
		mock_kineis->notify(KineisEventTxComplete({}));
	}
}


// ============================================================================
// Fastloc packet builder tests
// ============================================================================

TEST(ArgosTxService, FastlocSatellitePacketEncoding)
{
	// Build a fastloc GPS entry with degraded fix
	GPSLogEntry log = make_gps_location(true, -33.8232, 11.8768, 1580083200, false, 0, 5000, 3800);
	log.info.event_type = GPSEventType::FASTLOC;
	log.info.fixType = 2;   // 2D fix
	log.info.numSV = 5;
	log.info.hAcc = 500000;  // 500m in mm

	KineisPacket packet = ArgosPacketBuilder::build_fastloc_packet(&log, false);

	// Verify packet size = 12 bytes (96 bits, LDK)
	CHECK_EQUAL(ArgosPacketBuilder::FASTLOC_PACKET_BYTES, packet.size());

	// Verify header is 010 (first 3 bits of first byte)
	unsigned int header = (packet[0] >> 5) & 0x07;
	CHECK_EQUAL(ArgosPacketBuilder::FASTLOC_PACKET_HEADER, header);
}

TEST(ArgosTxService, FastlocSatellitePacketQualityFields)
{
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

TEST(ArgosTxService, FastlocPacketHeaderAndSize)
{
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

TEST(ArgosTxService, FastlocHAccEncoding16bit)
{
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

// NOTE: LoRa fastloc packet tests require lora_tx_service.cpp in test build (not currently included).
// LoRa fastloc encoding is validated via firmware build (build_linkitv4_lora.sh).

// ============================================================================
// COOLDOWN TESTS
// ============================================================================

TEST(ArgosTxService, CooldownBlocksSecondSurfacing)
{
	// After a successful TX cycle, cooldown prevents new TX on next surfacing
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::SURFACING_BURST);
	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, (bool)false);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, (bool)true);
	fake_config_store->write_param(ParamID::ARGOS_TX_JITTER_EN, (bool)false);
	fake_config_store->write_param(ParamID::DRY_TIME_BEFORE_TX, 0U);
	fake_config_store->write_param(ParamID::MIN_SURFACE_CYCLE_INTERVAL_S, 2700U);
	fake_config_store->write_param(ParamID::COOLDOWN_TRIGGER_MODE, 3U); // AFTER_LAST_TX

	ArgosTxService serv(*mock_kineis);

	// Use a distinct epoch far from other cooldown tests to avoid static state leaks
	std::time_t t = 1700000000000;
	fake_rtc->settime(t/1000);
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
	fake_rtc->settime(t/1000);
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
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	// Surface again → should be blocked by cooldown
	notify_underwater_state(false);

	// No TX should be scheduled — service is disabled during cooldown
	// Verify no send mock call is expected
	mock().checkExpectations();
}

TEST(ArgosTxService, CooldownExpiresAllowsNewBurst)
{
	// After cooldown expires, a new surfacing should trigger TX
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::SURFACING_BURST);
	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, (bool)false);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, (bool)true);
	fake_config_store->write_param(ParamID::ARGOS_TX_JITTER_EN, (bool)false);
	fake_config_store->write_param(ParamID::DRY_TIME_BEFORE_TX, 0U);
	fake_config_store->write_param(ParamID::MIN_SURFACE_CYCLE_INTERVAL_S, 60U); // Short cooldown for test
	fake_config_store->write_param(ParamID::COOLDOWN_TRIGGER_MODE, 3U);

	ArgosTxService serv(*mock_kineis);

	// Use a distinct epoch far from other cooldown tests to avoid static state leaks
	std::time_t t = 1750000000000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// Dive → surface → TX → dive (first cycle)
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("power_off_immediate").onObject(mock_kineis);
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	notify_underwater_state(true);
	t += 30000;
	fake_rtc->settime(t/1000);
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
	fake_rtc->settime(t/1000);
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

TEST(ArgosTxService, CooldownAtSurfaceDoesNotExtendOnBounce)
{
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
	fake_config_store->write_param(ParamID::COOLDOWN_TRIGGER_MODE, 0U); // AT_SURFACE

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1751000000000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// Cycle 1: dive → surface (arms AT_SURFACE) → dive (cooldown starts)
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("power_off_immediate").onObject(mock_kineis);
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	notify_underwater_state(true);
	t += 60000;
	fake_rtc->settime(t/1000);
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
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	notify_underwater_state(false);

	// Re-dive 5s later. Without the fix, m_cooldown_armed would be true here
	// (re-armed by AT_SURFACE), triggering set_cycle_complete(t_dive_2) which
	// would RESET the cooldown timer to 600s remaining.
	t += 5000;
	fake_rtc->settime(t/1000);
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

TEST(ArgosTxService, IsInCooldownTrueWhenSetAndCheckedSameTick)
{
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

TEST(ArgosTxService, NtryPerMessageLimitsGnssTx)
{
	// NTRY_PER_MESSAGE=2 → each GPS fix sent exactly 2 times, then no more
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::SURFACING_BURST);
	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, (bool)false);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, (bool)true);
	fake_config_store->write_param(ParamID::ARGOS_TX_JITTER_EN, (bool)false);
	fake_config_store->write_param(ParamID::DRY_TIME_BEFORE_TX, 0U);
	fake_config_store->write_param(ParamID::TR_NOM, 10U); // 10s for fast test
	fake_config_store->write_param(ParamID::NTRY_PER_MESSAGE, 2U);

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// Dive → surface
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("power_off_immediate").onObject(mock_kineis);
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	notify_underwater_state(true);
	t += 60000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	notify_underwater_state(false);

	// Fire initial Doppler
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("send").onObject(mock_kineis).ignoreOtherParameters();
	system_scheduler->run();
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	mock_kineis->notify(KineisEventTxComplete({}));

	// Inject GPS fix → switches to GNSS phase
	inject_gps_location(true, 55.27, -21.01, t/1000, true);

	// GNSS TX #1
	mock().expectOneCall("send").onObject(mock_kineis).ignoreOtherParameters();
	t += 1000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock_kineis->notify(KineisEventTxComplete({}));

	// GNSS TX #2 (last allowed by NTRY=2)
	mock().expectOneCall("send").onObject(mock_kineis).ignoreOtherParameters();
	t += 10000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock_kineis->notify(KineisEventTxComplete({}));

	// GNSS TX #3 should NOT happen (burst_counter exhausted)
	t += 10000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	// No send expected — verify
	mock().checkExpectations();
}

TEST(ArgosTxService, NtryZeroSendsOnce)
{
	// NTRY_PER_MESSAGE=0 → in surfacing burst mode, each fix sent exactly once
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::SURFACING_BURST);
	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, (bool)false);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, (bool)true);
	fake_config_store->write_param(ParamID::ARGOS_TX_JITTER_EN, (bool)false);
	fake_config_store->write_param(ParamID::DRY_TIME_BEFORE_TX, 0U);
	fake_config_store->write_param(ParamID::TR_NOM, 10U);
	fake_config_store->write_param(ParamID::NTRY_PER_MESSAGE, 0U); // 0 = once in burst mode

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// Dive → surface → Doppler
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("power_off_immediate").onObject(mock_kineis);
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	notify_underwater_state(true);
	t += 60000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	notify_underwater_state(false);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("send").onObject(mock_kineis).ignoreOtherParameters();
	system_scheduler->run();
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	mock_kineis->notify(KineisEventTxComplete({}));

	// GPS fix → GNSS phase
	inject_gps_location(true, 55.27, -21.01, t/1000, true);

	// GNSS TX #1 (only one allowed)
	mock().expectOneCall("send").onObject(mock_kineis).ignoreOtherParameters();
	t += 1000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock_kineis->notify(KineisEventTxComplete({}));

	// GNSS TX #2 should NOT happen
	t += 10000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock().checkExpectations();
}

// LEGACY (and DUTY_CYCLE/PASS_PREDICTION): NTRY_PER_MESSAGE=0 means UNLIMITED
// replay — once a valid fix is in the depth pile it keeps being retransmitted
// every TR_NOM until evicted by newer fixes. (Without a valid fix nothing is
// cached — silence.)
TEST(ArgosTxService, LegacyNtryZeroUnlimitedReplay)
{
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::LEGACY);
	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, (unsigned int)0x01234567U);
	fake_config_store->write_param(ParamID::LB_EN, false);
	fake_config_store->write_param(ParamID::TR_NOM, (unsigned int)10);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, false);
	fake_config_store->write_param(ParamID::NTRY_PER_MESSAGE, 0U);

	ArgosTxService serv(*mock_kineis);
	std::time_t t = 1652105502000;  // ms
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	inject_gps_location(1, 11.8768, -33.8232, t/1000);

	// Far more cycles than any bounded counter would allow — every one must TX
	for (unsigned int i = 0; i < 8; i++) {
		mock().expectOneCall("send").onObject(mock_kineis).ignoreOtherParameters();
		t += serv.get_last_schedule();
		fake_rtc->settime(t/1000);
		fake_timer->set_counter(t);
		system_scheduler->run();
		mock_kineis->notify(KineisEventTxComplete({}));
	}
	mock().checkExpectations();
}

// LEGACY: NTRY_PER_MESSAGE=3 → the pile is swept exactly 3 times (each entry
// TX'd 3 times), then all entries are inert and the service goes silent until
// a new fix arrives.
TEST(ArgosTxService, LegacyNtryBoundsPileSweeps)
{
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::LEGACY);
	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, (unsigned int)0x01234567U);
	fake_config_store->write_param(ParamID::LB_EN, false);
	fake_config_store->write_param(ParamID::TR_NOM, (unsigned int)10);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, false);
	fake_config_store->write_param(ParamID::NTRY_PER_MESSAGE, 3U);

	ArgosTxService serv(*mock_kineis);
	std::time_t t = 1652105502000;  // ms
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	inject_gps_location(1, 11.8768, -33.8232, t/1000);

	// Exactly 3 transmissions of the single-entry pile
	for (unsigned int i = 0; i < 3; i++) {
		mock().expectOneCall("send").onObject(mock_kineis).ignoreOtherParameters();
		t += serv.get_last_schedule();
		fake_rtc->settime(t/1000);
		fake_timer->set_counter(t);
		system_scheduler->run();
		mock_kineis->notify(KineisEventTxComplete({}));
	}

	// 4th cycle: burst_counter exhausted → no send
	t += 10000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock().checkExpectations();
}

// Depth pile sweep semantics behind NTRY: pile [A,B,C] with burst_counter=2 →
// retrieve() rotates newest-first through the pile (C,B,A), completes 2 full
// sweeps, then everything is inert.
TEST(ArgosTxService, DepthPileNtrySweepRotation)
{
	DepthPile<GPSLogEntry> dp;
	dp.set_max_size(4);
	GPSLogEntry a{}; a.info.day = 1;
	GPSLogEntry b{}; b.info.day = 2;
	GPSLogEntry c{}; c.info.day = 3;
	dp.store(a, 2);
	dp.store(b, 2);
	dp.store(c, 2);

	// retrieve(depth=4, max_messages=1): one entry per TX slot, newest-first
	int expected[6] = { 3, 2, 1, 3, 2, 1 };  // two full sweeps C,B,A
	for (unsigned int i = 0; i < 6; i++) {
		std::vector<GPSLogEntry*> v = dp.retrieve(4, 1);
		CHECK_EQUAL(1u, (unsigned int)v.size());
		CHECK_EQUAL(expected[i], (int)v.at(0)->info.day);
	}

	// Third sweep: all burst counters exhausted
	CHECK_EQUAL(0u, (unsigned int)dp.retrieve(4, 1).size());
	CHECK_EQUAL(0u, dp.eligible());
}

// === BaseGnssStrategy::REUSE_LAST helpers (Plan 1 step 1) =================

TEST(ArgosTxService, DepthPilePeekBackReturnsLatestWithoutConsuming)
{
	DepthPile<GPSLogEntry> dp;
	GPSLogEntry e1{}; e1.info.day = 1;
	GPSLogEntry e2{}; e2.info.day = 2;
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

TEST(ArgosTxService, DepthPilePeekBackOnEmptyReturnsNull)
{
	DepthPile<GPSLogEntry> dp;
	CHECK(dp.peek_back() == nullptr);
}

TEST(ArgosTxService, ComputeGpsLogAgeSecondsNormalEntry)
{
	GPSLogEntry e = make_gps_location();  // stamps header with rtc->gettime() = 1580083200
	// 90 seconds later
	unsigned int age = ArgosTxService::compute_gps_log_age_seconds(e, 1580083200 + 90);
	CHECK_EQUAL(90u, age);
}

TEST(ArgosTxService, ComputeGpsLogAgeSecondsZeroYearIsInvalid)
{
	GPSLogEntry e{};  // year == 0 — cold-boot / unset RTC sentinel
	unsigned int age = ArgosTxService::compute_gps_log_age_seconds(e, 1580083200);
	CHECK_EQUAL(UINT_MAX, age);
}

TEST(ArgosTxService, ComputeGpsLogAgeSecondsFutureEntryIsInvalid)
{
	GPSLogEntry e = make_gps_location();  // stamped at rtc = 1580083200
	// `now` BEFORE the entry timestamp → corruption / RTC roll-back. Must reject
	// rather than report age=0 which would falsely qualify as "fresh".
	unsigned int age = ArgosTxService::compute_gps_log_age_seconds(e, 1580083200 - 60);
	CHECK_EQUAL(UINT_MAX, age);
}

// === RateLimiter (Plan 1 step 2) ==========================================

TEST(ArgosTxService, RateLimiterDisabledNeverBlocks)
{
	RateLimiter::reset_for_tests();
	// Default: RATE_LIMIT_EN = false. Cap could be exhausted yet still allowed.
	for (unsigned int i = 0; i < 20; i++)
		RateLimiter::record_tx(1000 + i);  // no-op while disabled
	unsigned int rs = 0;
	CHECK_FALSE(RateLimiter::is_blocked(2000, rs));
}

TEST(ArgosTxService, RateLimiterBlocksOnceCapReached)
{
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

TEST(ArgosTxService, RateLimiterUnblocksOnceOldestRollsOut)
{
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

TEST(ArgosTxService, RateLimiterIgnoresFutureDatedEntries)
{
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

TEST(ArgosTxService, RateLimiterZeroCapDisables)
{
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

TEST(ArgosTxService, HauledModeDisabledNeverHauls)
{
	HauledModeService::reset_for_tests();
	// HAULED_DETECT_EN defaults to false. Even with a stale last_uw_event,
	// evaluate() must keep us at AT_SEA.
	HauledModeService::on_underwater_event(true, 1000);
	HauledModeService::evaluate(1000 + 100 * 3600);  // 100 hours later
	CHECK_FALSE(HauledModeService::is_hauled());
}

TEST(ArgosTxService, HauledModeHaulsAfterIdleThreshold)
{
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

TEST(ArgosTxService, HauledModeReturnsAfterReturnEvents)
{
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

TEST(ArgosTxService, HauledModeOverrideAppliesToArgosConfig)
{
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

TEST(ArgosTxService, HauledModeGnssStratOffForcesGnssOff)
{
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

TEST(ArgosTxService, ReuseLastEngagesGnssStrategyInHauled)
{
	HauledModeService::reset_for_tests();
	fake_config_store->write_param(ParamID::HAULED_DETECT_EN, (bool)true);
	fake_config_store->write_param(ParamID::HAULED_IDLE_THRESHOLD_H, 1U);
	fake_config_store->write_param(ParamID::HAULED_GNSS_EN, (bool)true);   // ignored when STRAT=REUSE_LAST
	fake_config_store->write_param(ParamID::HAULED_GNSS_STRAT, 1U);        // REUSE_LAST

	HauledModeService::on_underwater_event(true, 1000);
	HauledModeService::evaluate(1000 + 3601);

	ArgosConfig ac;
	configuration_store->get_argos_configuration(ac);
	// REUSE_LAST keeps GPS OFF (no acquisition) but sets strategy so dispatch
	// can route to the cached-fix path.
	CHECK_FALSE(ac.gnss_en);
	CHECK_TRUE(ac.gnss_strategy == BaseGnssStrategy::REUSE_LAST);
}

TEST(ArgosTxService, ReuseLastFreshHauledStillAcquires)
{
	HauledModeService::reset_for_tests();
	fake_config_store->write_param(ParamID::HAULED_DETECT_EN, (bool)true);
	fake_config_store->write_param(ParamID::HAULED_IDLE_THRESHOLD_H, 1U);
	fake_config_store->write_param(ParamID::HAULED_GNSS_EN, (bool)true);
	fake_config_store->write_param(ParamID::HAULED_GNSS_STRAT, 0U);        // FRESH

	HauledModeService::on_underwater_event(true, 1000);
	HauledModeService::evaluate(1000 + 3601);

	ArgosConfig ac;
	configuration_store->get_argos_configuration(ac);
	// FRESH preserves the existing acquire-then-TX behavior — gnss_en honors HMP12.
	CHECK_TRUE(ac.gnss_en);
	CHECK_TRUE(ac.gnss_strategy == BaseGnssStrategy::FRESH);
}

// === New mitigations (R5 auto-promote, M1b rollback recovery, K+L RTC sync) ===

TEST(ArgosTxService, HauledArgosModeAutoPromotesSurfacingBurstToLegacy)
{
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

TEST(ArgosTxService, HauledRollbackReBaselinesInsteadOfFreezing)
{
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

TEST(ArgosTxService, HauledResetForRtcSyncReAnchorsTimestamp)
{
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

TEST(ArgosTxService, RateLimiterResetForRtcSyncClearsRing)
{
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

TEST(ArgosTxService, NonHauledKeepsFreshStrategy)
{
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

TEST(ArgosTxService, HauledModeLowBatteryWins)
{
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
