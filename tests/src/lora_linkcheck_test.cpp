/**
 * @file lora_linkcheck_test.cpp
 * @brief Cyprus boat LoRa store-and-forward: depth-pile credits follow the
 *        LinkCheck verdict carried by KineisEventTxComplete (LORA_LINKCHECK).
 *
 * Compiled empty unless LORA_LINKCHECK is defined, i.e. only in the
 * TrackerTestsCyprus host target.
 */

#if defined(LORA_LINKCHECK) && (LORA_LINKCHECK == 1)

#include <set>
#include <string>
#include <vector>

#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

#include "mock_kineis_device.hpp"
#include "fake_rtc.hpp"
#include "fake_config_store.hpp"
#include "fake_timer.hpp"
#include "fake_battery_mon.hpp"
#include "timeutils.hpp"
#include "scheduler.hpp"
#include "bitpack.hpp"
#include "lora_tx_service.hpp"
#include "lora_packet_builder.hpp"
#include "lora_rak3172_comm.hpp"
#include "axl_sensor_service.hpp"

extern Timer *system_timer;
extern ConfigurationStore *configuration_store;
extern Scheduler *system_scheduler;
extern RTC *rtc;
extern BatteryMonitor *battery_monitor;

namespace {

constexpr int8_t LINK_UNKNOWN = -1;
constexpr int8_t LINK_HEARD = 0;
constexpr int8_t LINK_NOT_HEARD = 1;

/// Every fix in these tests has its own latitude, so a latitude names a fix.
double fix_latitude(unsigned int n) {
	return 34.0 + 0.001 * n;
}
unsigned int fix_code(unsigned int n) {
	return LoRaPacketBuilder::convert_latitude(fix_latitude(n));
}

/// Latitude codes carried by a GPS frame; empty for any other frame type.
std::vector<unsigned int> frame_latitudes(const KineisPacket &p) {
	std::vector<unsigned int> lats;
	unsigned int pos = 0, type = 0, count = 0, lat = 0;
	EXTRACT_BITS(type, p, pos, LoRaPacketBuilder::BITS_PKT_TYPE);
	if (type != LoRaPacketBuilder::PKT_TYPE_GPS_SINGLE && type != LoRaPacketBuilder::PKT_TYPE_GPS_MULTI) return lats;
	pos = LoRaPacketBuilder::BITS_HEADER;
	EXTRACT_BITS(count, p, pos, LoRaPacketBuilder::BITS_GPS_COUNT);
	if (count == 0) return lats;
	pos += LoRaPacketBuilder::BITS_DAY + LoRaPacketBuilder::BITS_HOUR + LoRaPacketBuilder::BITS_MIN;
	EXTRACT_BITS(lat, p, pos, LoRaPacketBuilder::BITS_LATITUDE);
	lats.push_back(lat);
	pos = LoRaPacketBuilder::BITS_HEADER + LoRaPacketBuilder::BITS_GPS_COUNT + LoRaPacketBuilder::BITS_GPS_FULL;
	for (unsigned int i = 1; i < count; i++) {
		EXTRACT_BITS(lat, p, pos, LoRaPacketBuilder::BITS_LATITUDE);
		lats.push_back(lat);
		pos += LoRaPacketBuilder::BITS_LONGITUDE + LoRaPacketBuilder::BITS_SPEED + LoRaPacketBuilder::BITS_DELTA_T_MIN;
	}
	return lats;
}

std::set<unsigned int> fix_codes(unsigned int first, unsigned int last) {
	std::set<unsigned int> codes;
	for (unsigned int n = first; n <= last; n++) codes.insert(fix_code(n));
	return codes;
}

/// wake-ups field of the MOTION block: second of the frame's last four bytes.
unsigned int frame_motion_wakeups(const KineisPacket &p) {
	return static_cast<unsigned char>(p[p.size() - 3]);
}

}  // namespace

/// LoRaTxService in the Cyprus boat configuration: LEGACY, 24-deep pile, NTRY 3,
/// DR3, one fix every five minutes -- each fix triggers an immediate frame.
TEST_GROUP(LoRaLinkCheck) {
	FakeBatteryMonitor *fake_battery_monitor;
	FakeConfigurationStore *fake_config_store;
	MockKineisDevice *mock_device;
	FakeRTC *fake_rtc;
	FakeTimer *fake_timer;
	LoRaTxService *serv;
	std::time_t t;
	unsigned int fixes;

	void setup() {
		ServiceManager::reset();
		fake_battery_monitor = new FakeBatteryMonitor;
		battery_monitor = fake_battery_monitor;
		mock_device = new MockKineisDevice;
		fake_config_store = new FakeConfigurationStore;
		configuration_store = fake_config_store;
		configuration_store->init();
		fake_rtc = new FakeRTC;
		rtc = fake_rtc;
		fake_timer = new FakeTimer;
		system_timer = fake_timer;
		system_scheduler = new Scheduler(system_timer);
		fake_timer->start();
		fixes = 0;
		set_time(1652105502000);

		unsigned int no_cooldown = 0U;
		fake_config_store->write_param(ParamID::MIN_SURFACE_CYCLE_INTERVAL_S, no_cooldown);
		fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::LEGACY);
		fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_24);
		fake_config_store->write_param(ParamID::NTRY_PER_MESSAGE, 3U);
		fake_config_store->write_param(ParamID::GNSS_EN, true);
		fake_config_store->write_param(ParamID::LB_EN, false);
		fake_config_store->write_param(ParamID::TR_NOM, 1200U);
		fake_config_store->write_param(ParamID::ARGOS_TX_JITTER_EN, false);
		fake_config_store->write_param(ParamID::LORA_DR, 3U);
		mock().ignoreOtherCalls();

		serv = new LoRaTxService(*mock_device);
		serv->start();
	}

	void teardown() {
		delete serv;
		serv = nullptr;
		mock().clear();
		delete system_scheduler;
		delete fake_timer;
		delete fake_rtc;
		delete fake_config_store;
		delete mock_device;
		delete fake_battery_monitor;
		system_scheduler = nullptr;
		system_timer = nullptr;
		rtc = nullptr;
		configuration_store = nullptr;
		battery_monitor = nullptr;
	}

	void set_time(std::time_t ms) {
		t = ms;
		fake_rtc->settime(t / 1000);
		fake_timer->set_counter(t);
	}

	void inject_fix() {
		fixes++;
		const std::time_t now_s = t / 1000;
		GPSLogEntry log{};
		log.info.valid = true;
		log.info.lat = fix_latitude(fixes);
		log.info.lon = 33.0;
		log.info.fixType = 3;
		log.info.batt_voltage = 4000;
		log.info.schedTime = now_s;
		uint16_t year;
		uint8_t month, day, hour, min, sec;
		convert_datetime_to_epoch(now_s, year, month, day, hour, min, sec);
		log.header.year = log.info.year = year;
		log.header.month = log.info.month = month;
		log.header.day = log.info.day = day;
		log.header.hours = log.info.hour = hour;
		log.header.minutes = log.info.min = min;
		log.header.seconds = log.info.sec = sec;
		ServiceEvent e;
		e.event_source = ServiceIdentifier::GNSS_SENSOR;
		e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
		e.event_data = log;
		e.event_originator_unique_id = 0x12345678;
		ServiceManager::notify_peer_event(e);
		configuration_store->notify_gps_location(log);
	}

	/// An acquisition that ended without a position: the NO_FIX marker GPSService logs.
	void inject_no_fix() {
		const std::time_t now_s = t / 1000;
		GPSLogEntry log{};
		log.info.valid = false;
		log.info.event_type = GPSEventType::NO_FIX;
		log.info.batt_voltage = 4000;
		log.info.schedTime = now_s;
		uint16_t year;
		uint8_t month, day, hour, min, sec;
		convert_datetime_to_epoch(now_s, year, month, day, hour, min, sec);
		log.header.year = log.info.year = year;
		log.header.month = log.info.month = month;
		log.header.day = log.info.day = day;
		log.header.hours = log.info.hour = hour;
		log.header.minutes = log.info.min = min;
		log.header.seconds = log.info.sec = sec;
		ServiceEvent e;
		e.event_source = ServiceIdentifier::GNSS_SENSOR;
		e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
		e.event_data = log;
		e.event_originator_unique_id = 0x12345678;
		ServiceManager::notify_peer_event(e);
	}

	void inject_wakeups(unsigned int n) {
		for (unsigned int i = 0; i < n; i++) {
			ServiceSensorData data{};
			data.port[AXLSensorPort::WAKEUP_TRIGGERED] = 1.0;
			ServiceEvent e;
			e.event_source = ServiceIdentifier::AXL_SENSOR;
			e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
			e.event_data = data;
			e.event_originator_unique_id = 0x12345678;
			ServiceManager::notify_peer_event(e);
		}
	}

	/// Run what is due now; true if a frame was handed to the radio.
	bool dispatch() {
		const unsigned int before = mock_device->send_count;
		system_scheduler->run();
		return mock_device->send_count != before;
	}

	void complete(bool reached_air, int8_t verdict) {
		if (reached_air) mock_device->notify(KineisEventTxStarted({}));
		KineisEventTxComplete ev{};
		ev.link_check = verdict;
		mock_device->notify(ev);
	}

	/// A fix five minutes after the previous one, and the frame it triggers.
	std::vector<unsigned int> fix_and_send(int8_t verdict, bool reached_air = true) {
		set_time(t + 300000);
		inject_fix();
		CHECK_TRUE(dispatch());
		const std::vector<unsigned int> lats = frame_latitudes(mock_device->last_packet);
		complete(reached_air, verdict);
		return lats;
	}

	/// No new fix: the next TR_NOM frame.
	std::vector<unsigned int> periodic_send(int8_t verdict) {
		set_time(t + serv->get_last_schedule());
		CHECK_TRUE(dispatch());
		const std::vector<unsigned int> lats = frame_latitudes(mock_device->last_packet);
		complete(true, verdict);
		return lats;
	}
};

// Without a verdict the accounting is exactly today's: each position rides in
// NTRY consecutive frames.
TEST(LoRaLinkCheck, UnknownVerdictKeepsTheExistingRepetition) {
	CHECK_EQUAL(1U, fix_and_send(LINK_UNKNOWN).size());
	CHECK_EQUAL(2U, fix_and_send(LINK_UNKNOWN).size());
	CHECK_EQUAL(3U, fix_and_send(LINK_UNKNOWN).size());
	CHECK_EQUAL(3U, fix_and_send(LINK_UNKNOWN).size());
}

// A heard frame is spent: its positions never ride again.
TEST(LoRaLinkCheck, HeardPositionsAreNeverSentAgain) {
	for (unsigned int k = 0; k < 4; k++) {
		const std::vector<unsigned int> lats = fix_and_send(LINK_HEARD);
		CHECK_EQUAL(1U, lats.size());
		CHECK_EQUAL(fix_code(fixes), lats[0]);
	}
}

// Out of coverage nothing is spent and nothing counts as a device error: every
// frame still goes out, carrying every position taken since coverage was lost.
TEST(LoRaLinkCheck, UnheardPositionsWaitForCoverageWithoutSuspension) {
	for (unsigned int k = 1; k <= 8; k++) {
		CHECK_EQUAL(k, fix_and_send(LINK_NOT_HEARD).size());
	}
	const std::vector<unsigned int> back = fix_and_send(LINK_HEARD);
	CHECK_TRUE(std::set<unsigned int>(back.begin(), back.end()) == fix_codes(1, 9));
	CHECK_EQUAL(1U, fix_and_send(LINK_HEARD).size());
}

// A 2.5 h outage: the pile keeps the 24 newest positions, and the first two
// heard frames deliver exactly those. Nothing is left over after them.
TEST(LoRaLinkCheck, LongOutageDeliversTheWholePileOnceHeard) {
	for (unsigned int k = 0; k < 30; k++) {
		CHECK_COMPARE(fix_and_send(LINK_NOT_HEARD).size(), <=, 12U);
	}
	std::set<unsigned int> delivered;
	for (unsigned int k = 0; k < 2; k++) {
		const std::vector<unsigned int> lats = periodic_send(LINK_HEARD);
		CHECK_EQUAL(12U, lats.size());
		delivered.insert(lats.begin(), lats.end());
	}
	CHECK_TRUE(delivered == fix_codes(7, 30));
	// Pile drained: the next frame is the status heartbeat.
	CHECK_EQUAL(0U, periodic_send(LINK_HEARD).size());
}

// At DR5 more positions are pending than the 4-bit count field can carry: the
// frame holds the 15 oldest. Only those are spent by a heard frame; the rest
// keep their credits and ride in the next one.
TEST(LoRaLinkCheck, OnlyThePositionsInTheFrameAreSpent) {
	fake_config_store->write_param(ParamID::LORA_DR, 5U);
	for (unsigned int k = 1; k < 20; k++) {
		CHECK_EQUAL(std::min(k, 15U), fix_and_send(LINK_NOT_HEARD).size());
	}
	const std::vector<unsigned int> heard = fix_and_send(LINK_HEARD);
	CHECK_TRUE(std::set<unsigned int>(heard.begin(), heard.end()) == fix_codes(1, 15));
	const std::vector<unsigned int> rest = fix_and_send(LINK_HEARD);
	CHECK_TRUE(std::set<unsigned int>(rest.begin(), rest.end()) == fix_codes(16, 21));
	CHECK_EQUAL(1U, fix_and_send(LINK_HEARD).size());
}

// NTRY 1 at DR5: a heard frame carrying the 15 oldest of 20 pending positions must
// not take the only credit of the 5 it left out.
TEST(LoRaLinkCheck, PositionsLeftOutOfAHeardFrameKeepTheirLastCredit) {
	fake_config_store->write_param(ParamID::LORA_DR, 5U);
	fake_config_store->write_param(ParamID::NTRY_PER_MESSAGE, 1U);
	for (unsigned int k = 1; k < 20; k++) {
		CHECK_EQUAL(std::min(k, 15U), fix_and_send(LINK_NOT_HEARD).size());
	}
	const std::vector<unsigned int> heard = fix_and_send(LINK_HEARD);
	CHECK_TRUE(std::set<unsigned int>(heard.begin(), heard.end()) == fix_codes(1, 15));
	const std::vector<unsigned int> rest = fix_and_send(LINK_HEARD);
	CHECK_TRUE(std::set<unsigned int>(rest.begin(), rest.end()) == fix_codes(16, 21));
}

// NTRY_PER_MESSAGE 0 already means "replay until evicted": the verdict changes nothing.
TEST(LoRaLinkCheck, NtryZeroIgnoresTheVerdict) {
	fake_config_store->write_param(ParamID::NTRY_PER_MESSAGE, 0U);
	CHECK_EQUAL(1U, fix_and_send(LINK_HEARD).size());
	CHECK_EQUAL(2U, fix_and_send(LINK_HEARD).size());
	CHECK_EQUAL(3U, fix_and_send(LINK_NOT_HEARD).size());
	CHECK_EQUAL(4U, fix_and_send(LINK_HEARD).size());
}

// Sky lost during an outage: every acquisition stores a NO_FIX marker in the pile.
// The markers must not push the real positions still waiting for coverage out of
// the 24 slots -- the next valid fix deletes the markers anyway.
TEST(LoRaLinkCheck, NoFixMarkersDoNotEvictWaitingPositions) {
	for (unsigned int k = 1; k <= 12; k++) {
		CHECK_EQUAL(k, fix_and_send(LINK_NOT_HEARD).size());
	}
	for (unsigned int k = 0; k < 18; k++) {
		set_time(t + 300000);
		inject_no_fix();
		if (dispatch()) complete(true, LINK_NOT_HEARD);
	}
	std::set<unsigned int> delivered;
	const std::vector<unsigned int> back = fix_and_send(LINK_HEARD);
	delivered.insert(back.begin(), back.end());
	for (unsigned int k = 0; k < 3; k++) {
		const std::vector<unsigned int> lats = periodic_send(LINK_HEARD);
		delivered.insert(lats.begin(), lats.end());
	}
	for (unsigned int n = 1; n <= 13; n++) {
		CHECK_TRUE(delivered.count(fix_code(n)) == 1);
	}
}

// A TxComplete never preceded by TxStarted (a late event after a cancel) says
// nothing about the batch noted in flight: today's accounting applies.
TEST(LoRaLinkCheck, VerdictWithoutTxStartedIsIgnored) {
	CHECK_EQUAL(1U, fix_and_send(LINK_NOT_HEARD, false).size());
	CHECK_EQUAL(2U, fix_and_send(LINK_HEARD, false).size());
	CHECK_EQUAL(3U, fix_and_send(LINK_NOT_HEARD, false).size());
	CHECK_EQUAL(3U, fix_and_send(LINK_HEARD, false).size());
}

// A fix stored while the frame is on air may reshape the pile under the
// in-flight pointers: the verdict is not applied, one credit is spent as today.
TEST(LoRaLinkCheck, PileChangedInFlightFallsBackToOneCredit) {
	set_time(t + 300000);
	inject_fix();
	CHECK_TRUE(dispatch());
	CHECK_EQUAL(1U, frame_latitudes(mock_device->last_packet).size());
	mock_device->notify(KineisEventTxStarted({}));
	inject_fix();
	KineisEventTxComplete ev{};
	ev.link_check = LINK_HEARD;
	mock_device->notify(ev);

	// Fix 1 kept two of its three credits and rides again beside fix 2.
	CHECK_TRUE(dispatch());
	const std::vector<unsigned int> lats = frame_latitudes(mock_device->last_packet);
	CHECK_TRUE(std::set<unsigned int>(lats.begin(), lats.end()) == fix_codes(1, 2));
}

// MOTION wake-ups reported by a frame nobody heard are reported again.
TEST(LoRaLinkCheck, MotionWakeupsSurviveAnUnheardFrame) {
	inject_wakeups(5);
	set_time(t + 300000);
	inject_fix();
	CHECK_TRUE(dispatch());
	CHECK_EQUAL(5U, frame_motion_wakeups(mock_device->last_packet));
	complete(true, LINK_NOT_HEARD);

	inject_wakeups(2);
	set_time(t + 300000);
	inject_fix();
	CHECK_TRUE(dispatch());
	CHECK_EQUAL(7U, frame_motion_wakeups(mock_device->last_packet));
	complete(true, LINK_HEARD);

	set_time(t + 300000);
	inject_fix();
	CHECK_TRUE(dispatch());
	CHECK_EQUAL(0U, frame_motion_wakeups(mock_device->last_packet));
}

// ---------------------------------------------------------------------------
// +EVT:LINKCHECK parser
// ---------------------------------------------------------------------------
namespace {
struct LinkCheckProbe : public LoRaCommEventListener {
	using LoRaCommEventListener::react;
	unsigned int events = 0;
	LoRaCommEventLinkCheck last{};
	void react(const LoRaCommEventLinkCheck &e) override {
		events++;
		last = e;
	}
};

class LineFeeder : public LoRaComm {
public:
	void feed(const char *s) {
		std::string line(s);
		on_rx_line(line);
	}
};
}  // namespace

TEST_GROUP(LoRaLinkCheckParser) {
	LineFeeder comm;
	LinkCheckProbe probe;
	void setup() { comm.subscribe(probe); }
};

TEST(LoRaLinkCheckParser, AnswerCarriesMarginGatewaysRssiSnr) {
	comm.feed("+EVT:LINKCHECK:0,15,2,-61,9");
	CHECK_EQUAL(1U, probe.events);
	CHECK_EQUAL(0, probe.last.result);
	CHECK_EQUAL(15, probe.last.margin);
	CHECK_EQUAL(2, probe.last.gateways);
	CHECK_EQUAL(-61, probe.last.rssi);
	CHECK_EQUAL(9, probe.last.snr);
}

// What the RAK3172 on the bench (RUI3, 2026-09) actually prints: ':' between the fields.
TEST(LoRaLinkCheckParser, Rui3ColonSeparatedLine) {
	comm.feed("+EVT:LINKCHECK:1:0:0:0:0");
	CHECK_EQUAL(1U, probe.events);
	CHECK_EQUAL(1, probe.last.result);
	comm.feed("+EVT:LINKCHECK:0:12:1:-80:7");
	CHECK_EQUAL(2U, probe.events);
	CHECK_EQUAL(0, probe.last.result);
	CHECK_EQUAL(12, probe.last.margin);
	CHECK_EQUAL(1, probe.last.gateways);
	CHECK_EQUAL(-80, probe.last.rssi);
	CHECK_EQUAL(7, probe.last.snr);
}

TEST(LoRaLinkCheckParser, NonZeroStatusIsNotHeard) {
	comm.feed("+EVT:LINKCHECK:1,0,0,0,0");
	CHECK_EQUAL(1U, probe.events);
	CHECK_EQUAL(1, probe.last.result);
	comm.feed("+EVT:LINKCHECK:2,0,0,0,0");
	CHECK_EQUAL(2U, probe.events);
	CHECK_EQUAL(1, probe.last.result);
}

// A line cut by an RX overflow, or any shape we do not know, must never throw
// nor pass for either verdict.
TEST(LoRaLinkCheckParser, MalformedLineIsUnknown) {
	const char *const lines[] = {
		"+EVT:LINKCHECK:",
		"+EVT:LINKCHECK:x,1,1,1,1",
		"+EVT:LINKCHECK:0,15",
		"+EVT:LINKCHECK:0,15,2,-61,9,7",
		"+EVT:LINKCHECK:0,15,2,-61,9 ",
		"+EVT:LINKCHECK:0,15,2,-,9",
		"+EVT:LINKCHECK:0,123456,2,-61,9",
	};
	unsigned int n = 0;
	for (const char *line : lines) {
		comm.feed(line);
		n++;
		CHECK_EQUAL(n, probe.events);
		CHECK_EQUAL(-1, probe.last.result);
	}
}

TEST(LoRaLinkCheckParser, OtherEventsAreNotLinkCheck) {
	comm.feed("+EVT:TX_DONE");
	comm.feed("+EVT:SEND_CONFIRMED_OK");
	CHECK_EQUAL(0U, probe.events);
}

#endif
