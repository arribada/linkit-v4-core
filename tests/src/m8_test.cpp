#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"
#include "fake_logger.hpp"
#include "fake_config_store.hpp"
#include "fake_rtc.hpp"
#include "fake_timer.hpp"
#include "scheduler.hpp"
#include "bsp.hpp"
#include "nrf_libuarte_async.h"
#include "m10qasync.hpp"
#include "ubx.hpp"

namespace BSP {

const nrf_uarte_t uarte = {};
const nrf_libuarte_async_t uarte_async = {
	.p_libuarte = &uarte,
};
const UARTAsync_InitTypeDefAndInst_t UARTAsync_Inits[1] = { { .uart = &uarte_async, .config = {} } };
}

class TestGNSSListener : public GPSEventListener {
private:
	GPSDevice &m_device;

	void react(const GPSEventPowerOn &) { mock().actualCall("GPSEventPowerOn"); }
	void react(const GPSEventPowerOff &) { mock().actualCall("GPSEventPowerOff"); }
	void react(const GPSEventError &) { mock().actualCall("GPSEventError"); }
	void react(const GPSEventPVT &) { mock().actualCall("GPSEventPVT"); }
	void react(const GPSEventSatReport &) { mock().actualCall("GPSEventSatReport"); }
	void react(const GPSEventMaxNavSamples &) { mock().actualCall("GPSEventMaxNavSamples"); }
	void react(const GPSEventMaxSatSamples &) { mock().actualCall("GPSEventMaxSatSamples"); }

public:
	TestGNSSListener(GPSDevice &dev) : m_device(dev) { dev.subscribe(*this); }
	~TestGNSSListener() { m_device.unsubscribe(*this); }
};

extern ConfigurationStore *configuration_store;
extern RTC *rtc;
extern Timer *system_timer;
extern Scheduler *system_scheduler;


TEST_GROUP(M8) {
	FakeLog *logger;
	ConfigurationStore *fake_config;
	FakeRTC *fake_rtc;
	FakeTimer *fake_timer;
	uint64_t m_current_ms = 0;
	unsigned int m_iTOW = 0;

	void setup() {
		logger = new FakeLog("GNSS");
		fake_config = new FakeConfigurationStore;
		configuration_store = fake_config;
		fake_rtc = new FakeRTC;
		rtc = fake_rtc;
		fake_timer = new FakeTimer;
		system_timer = fake_timer;
		system_scheduler = new Scheduler(system_timer);
	}

	void teardown() {
		mock().clear();
		delete logger;
		delete fake_config;
		delete fake_timer;
		delete fake_rtc;
		rtc = nullptr;
	}

	void expect_power_on() {
		// Updated for 2026-05 M10Q reset-hold fix (analog of SAT_RESET BSP fix):
		// NRST is now held LOW during VDD ramp, then released after stabilize.
		// Updated again 2026-09: EXTINT gets the same treatment, driven LOW
		// before VDD rises so the pin has a defined level from the receiver's
		// first powered instant and never floats while it is awake — PMREQ wakes
		// on EITHER edge, so a floating line is a wake source.
		// Sequence: init_pin(RST) → clear(RST) → init_pin(EXTINT) →
		//           clear(EXTINT) → delay → set(PWR_EN) → delay →
		//           set(RST) → delay → GPSEventPowerOn.
		mock().expectOneCall("acquire_sensors_pwr");
		mock().expectOneCall("init_pin").withParameter("pin", BSP::GPIO::GPIO_GPS_RST);
		mock().expectOneCall("clear").withParameter("pin", BSP::GPIO::GPIO_GPS_RST);
		mock().expectOneCall("init_pin").withParameter("pin", BSP::GPIO::GPIO_GPS_EXT_INT);
		mock().expectOneCall("clear").withParameter("pin", BSP::GPIO::GPIO_GPS_EXT_INT);
		mock().expectOneCall("delay_ms").ignoreOtherParameters();
		mock().expectOneCall("set").withParameter("pin", BSP::GPIO::GPIO_GPS_PWR_EN);
		mock().expectOneCall("delay_ms").ignoreOtherParameters();
		mock().expectOneCall("set").withParameter("pin", BSP::GPIO::GPIO_GPS_RST);
		mock().expectOneCall("delay_ms").ignoreOtherParameters();
		mock().expectOneCall("GPSEventPowerOn");
	}

	void expect_power_off() {
		// Updated for 2026-05 M10Q discharge-wait fix (enter_shutdown now:
		// init_pin(RST) → clear(RST) → clear(PWR_EN) → delay(50ms) →
		// release_to_highz(RST) → release_to_highz(EXT_INT) → release_sensors).
		mock().expectOneCall("GPSEventPowerOff");
		mock().expectOneCall("init_pin").withParameter("pin", BSP::GPIO::GPIO_GPS_RST);
		mock().expectOneCall("clear").withParameter("pin", BSP::GPIO::GPIO_GPS_RST);
		mock().expectOneCall("clear").withParameter("pin", BSP::GPIO::GPIO_GPS_PWR_EN);
		mock().expectOneCall("delay_ms").ignoreOtherParameters();
		mock().expectOneCall("release_to_highz").withParameter("pin", BSP::GPIO::GPIO_GPS_RST);
		mock().expectOneCall("release_to_highz").withParameter("pin", BSP::GPIO::GPIO_GPS_EXT_INT);
		mock().expectOneCall("release_sensors_pwr");
	}

	void increment_time_ms(uint64_t ms = 0) {
		while (ms) {
			fake_timer->increment_counter(1);
			if (fake_timer->get_counter() % 1000 == 0) fake_rtc->incrementtime(1);
			system_scheduler->run();
			ms--;
			//DEBUG_TRACE("timer=%lu rtc=%lu", fake_timer->get_counter(), rtc->gettime());
		}
		system_scheduler->run();
	}

	void ubx_compute_crc(const uint8_t *const buffer, const unsigned int length, uint8_t &ck_a, uint8_t &ck_b) {
		ck_a = 0;
		ck_b = 0;
		for (unsigned int i = 0; i < length; i++) {
			ck_a = ck_a + buffer[i];
			ck_b = ck_b + ck_a;
		}
	}

	void inject_error(unsigned int flags) {
		nrf_libuarte_async_evt_t evt;
		evt.type = NRF_LIBUARTE_ASYNC_EVT_ERROR;
		evt.data.errorsrc = flags;
		nrf_libuarte_inject_event(&evt);
	}

	template <typename T> void ubx_inject_message(UBX::MessageClass cls, uint8_t id, T content) {
		unsigned int content_size;
		if constexpr (std::is_same<T, UBX::Empty>::value) {
			content_size = 0;
		} else {
			content_size = sizeof(content);
		};
		UBX::HeaderAndPayloadCRC raw;
		UBX::HeaderAndPayloadCRC *msg = &raw;
		msg->syncChars[0] = UBX::SYNC_CHAR1;
		msg->syncChars[1] = UBX::SYNC_CHAR2;
		msg->msgClass = cls;
		msg->msgId = id;
		msg->msgLength = content_size;
		std::memcpy(msg->payload, &content, content_size);
		ubx_compute_crc((const uint8_t *const)&msg->msgClass, content_size + sizeof(UBX::Header) - 2,
		                msg->payload[msg->msgLength], msg->payload[msg->msgLength + 1]);

		nrf_libuarte_async_evt_t evt;
		evt.type = NRF_LIBUARTE_ASYNC_EVT_RX_DATA;
		evt.data.rxtx.length = content_size + sizeof(UBX::Header) + 2;
		evt.data.rxtx.p_data = (uint8_t *)&raw;
		nrf_libuarte_inject_event(&evt);
	}

	void ubx_ack(UBX::MessageClass cls, uint8_t id) {
		UBX::ACK::MSG_ACK ack = { cls, id };
		ubx_inject_message(UBX::MessageClass::MSG_CLASS_ACK, UBX::ACK::ID_ACK, ack);
	}

	void ubx_nack(UBX::MessageClass cls, uint8_t id) {
		UBX::ACK::MSG_NACK nack = { cls, id };
		ubx_inject_message(UBX::MessageClass::MSG_CLASS_ACK, UBX::ACK::ID_NACK, nack);
	}
	void ubx_pvt(double lat, double lon, bool is_valid = true) {
		UBX::NAV::PVT::MSG_PVT pvt;
		pvt.iTow = m_iTOW;
		pvt.lon = lon * 1E6;
		pvt.lat = lat * 1E6;
		pvt.valid = is_valid ? (UBX::NAV::PVT::VALID::VALID_FULLY_RESOLVED | UBX::NAV::PVT::VALID_VALID_DATE
		                        | UBX::NAV::PVT::VALID_VALID_TIME)
		                     : 0;
		pvt.fixType = is_valid ? UBX::NAV::PVT::FIXTYPE_2D : UBX::NAV::PVT::FIXTYPE_NO;
		ubx_inject_message(UBX::MessageClass::MSG_CLASS_NAV, UBX::NAV::ID_PVT, pvt);
	}
	void ubx_status(bool is_valid) {
		UBX::NAV::STATUS::MSG_STATUS status;
		status.iTow = m_iTOW;
		status.fixStat = is_valid;
		ubx_inject_message(UBX::MessageClass::MSG_CLASS_NAV, UBX::NAV::ID_STATUS, status);
	}
	void ubx_dop() {
		UBX::NAV::DOP::MSG_DOP dop;
		dop.iTow = m_iTOW;
		ubx_inject_message(UBX::MessageClass::MSG_CLASS_NAV, UBX::NAV::ID_DOP, dop);
	}
	void ubx_mga_ack(bool success, unsigned int num_messages) {
		UBX::MGA::MSG_ACK ack;
		ack.infoCode = !success;
		ack.msgPayloadStart = num_messages;
		ubx_inject_message(UBX::MessageClass::MSG_CLASS_MGA, UBX::MGA::ID_ACK, ack);
	}
	void ubx_mga_dbd(unsigned int num_messages) {
		struct {
			uint8_t data[64];
		} x;
		for (unsigned int i = 0; i < num_messages; i++) {
			ubx_inject_message(UBX::MessageClass::MSG_CLASS_MGA, UBX::MGA::ID_DBD, x);
			increment_time_ms();
		}
	}
};

TEST(M8, FailedToSyncCommsError) {
	M10QAsyncReceiver m;
	TestGNSSListener listener(m);
	GPSNavSettings settings;
	settings.assistnow_autonomous_enable = false;
	settings.assistnow_offline_enable = false;
	settings.debug_enable = true;
	settings.dyn_model = BaseGNSSDynModel::PORTABLE;
	settings.fix_mode = BaseGNSSFixMode::AUTO;
	settings.hacc_filter_en = false;
	settings.hdop_filter_en = false;
	settings.max_nav_samples = 30;

	expect_power_on();
	m.power_on(settings);
	mock().expectOneCall("GPSEventError");
	// R4 (2026-08): on an unrecoverable error the driver now cuts the rail itself
	// (check_for_power_off) instead of waiting for a power_off() from the client --
	// a subscriber may perfectly well ignore the event (session already over, a
	// PWRON GNSS that subscribes nobody) and the FSM stayed frozen with the rail on.
	// The expectation must therefore be declared BEFORE the time advance that
	// triggers the error.
	expect_power_off();
	// 4 rates probed: 6 attempts on the first + 2 on each of the other three, at
	// 500 ms => exactly 6000 ms. Margin so the test does not depend on tick phase.
	increment_time_ms(7000);
	increment_time_ms();
}

// Regression 2026-08 — a permanent "M10QAsyncReceiver: failed to sync comms".
//
// setup_uart_port() writes CFG-UART1-BAUDRATE=460800 into the BBR|RAM layers. On
// a board whose V_BCKP cell or supercap holds the BBR across the window where the
// rail is off, the M10Q therefore does NOT come back to factory defaults: it
// returns at 460800, and it is completely mute at 9600 (NMEA disabled, nav
// messages in the RAM layer and so switched off). The driver only probed 9600:
// every session died on GPSEventError, and state_configure -- the only place that
// renegotiates the port and the only place that sends the CFG-RST able to erase
// the BBR -- sat behind that sync. No safety net caught this case.
TEST(M8, BootSyncFallsBackToSecondBaudWhenBbrRetained) {
	M10QAsyncReceiver m;
	TestGNSSListener listener(m);
	GPSNavSettings settings;
	settings.assistnow_autonomous_enable = false;
	settings.assistnow_offline_enable = false;
	settings.debug_enable = true;
	settings.dyn_model = BaseGNSSDynModel::PORTABLE;
	settings.fix_mode = BaseGNSSFixMode::AUTO;
	settings.hacc_filter_en = false;
	settings.hdop_filter_en = false;
	settings.max_nav_samples = 30;

	expect_power_on();
	m.power_on(settings);
	increment_time_ms();
	// Premiere sonde: 9600 (defaut usine), le recepteur reste muet.
	CHECK_EQUAL(NRF_UARTE_BAUDRATE_9600, g_fake_last_baudrate);

	// 6 x 500 ms of unanswered timeouts -> the driver must move to the next baud
	// rather than abandon the session. Margin beyond the exact 3000 ms: when the
	// 6th timeout lands depends on the scheduler tick phase.
	increment_time_ms(3500);
	CHECK_EQUAL(NRF_UARTE_BAUDRATE_460800, g_fake_last_baudrate);

	// The M10Q answers at 460800 (a NACK on the probe's invalid CFG-MSG): the
	// session must continue. No GPSEventError is expected -- the mock fails the
	// test if one arrives.
	ubx_nack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms(100);

	m.power_off();
	expect_power_off();
	increment_time_ms();
}

// The rate that answered is remembered: the next session probes it first, so a
// board with a retained BBR only pays for the wasted probes once per boot.
TEST(M8, BootSyncCachesWorkingBaudForNextSession) {
	M10QAsyncReceiver m;
	TestGNSSListener listener(m);
	GPSNavSettings settings;
	settings.assistnow_autonomous_enable = false;
	settings.assistnow_offline_enable = false;
	settings.debug_enable = true;
	settings.dyn_model = BaseGNSSDynModel::PORTABLE;
	settings.fix_mode = BaseGNSSFixMode::AUTO;
	settings.hacc_filter_en = false;
	settings.hdop_filter_en = false;
	settings.max_nav_samples = 30;

	// Session 1: muet a 9600, repond a 460800.
	expect_power_on();
	m.power_on(settings);
	increment_time_ms();
	increment_time_ms(3500);
	CHECK_EQUAL(NRF_UARTE_BAUDRATE_460800, g_fake_last_baudrate);
	ubx_nack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms(100);
	m.power_off();
	expect_power_off();
	increment_time_ms();

	// Session 2: the first probe must now go straight out at 460800.
	// Sentinel: without it the CHECK below would pass vacuously if no probe were
	// sent at all (the variable would keep session 1's value).
	g_fake_last_baudrate = NRF_UARTE_BAUDRATE_1000000;
	expect_power_on();
	m.power_on(settings);
	increment_time_ms();
	CHECK_EQUAL(NRF_UARTE_BAUDRATE_460800, g_fake_last_baudrate);

	ubx_nack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms(100);
	m.power_off();
	expect_power_off();
	increment_time_ms();
}

TEST(M8, FailedToChangeBaudRate) {
	M10QAsyncReceiver m;
	TestGNSSListener listener(m);
	GPSNavSettings settings;
	settings.assistnow_autonomous_enable = false;
	settings.assistnow_offline_enable = false;
	settings.debug_enable = true;
	settings.dyn_model = BaseGNSSDynModel::PORTABLE;
	settings.fix_mode = BaseGNSSFixMode::AUTO;
	settings.hacc_filter_en = false;
	settings.hdop_filter_en = false;
	settings.max_nav_samples = 30;

	expect_power_on();
	m.power_on(settings);
	increment_time_ms();
	ubx_nack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	mock().expectOneCall("GPSEventError");
	// R4 (2026-08): on an unrecoverable error the driver now cuts the rail itself
	// (check_for_power_off) instead of waiting for a power_off() from the client --
	// a subscriber may perfectly well ignore the event (session already over, a
	// PWRON GNSS that subscribes nobody) and the FSM stayed frozen with the rail on.
	// The expectation must therefore be declared BEFORE the time advance that
	// triggers the error.
	expect_power_off();
	increment_time_ms(3000);
	increment_time_ms();
}

// Regression: when the BBR fast path is abandoned, the UART must return to the
// rate the receiver ACTUALLY answers on. Without that, setup_uart_port() went out
// at 460800 to an M10Q listening at 9600 and the session died -- one session in
// two on a board with no backup cell.
TEST(M8, FastPathGiveUpRestoresSyncedBaud) {
	M10QAsyncReceiver m;
	TestGNSSListener listener(m);
	GPSNavSettings settings;
	settings.assistnow_autonomous_enable = false;
	settings.assistnow_offline_enable = false;
	settings.debug_enable = true;
	settings.dyn_model = BaseGNSSDynModel::PORTABLE;
	settings.fix_mode = BaseGNSSFixMode::AUTO;
	settings.hacc_filter_en = false;
	settings.hdop_filter_en = false;
	settings.max_nav_samples = 30;

	// --- Session 1: a full configure through to SEC-UNIQID, which arms
	// m_gnss_info_valid (a precondition of the fast path).
	expect_power_on();
	m.power_on(settings);
	increment_time_ms();
	ubx_nack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);  // sync 9600
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_PRT);  // step 0
	// Step 0 is fire-and-forget: it posts a task at +1000 ms before step 1 sends
	// the probe. Waiting less lets the next injection fall into the void (the
	// expectation filter is not armed yet).
	increment_time_ms(1100);
	ubx_nack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);  // step 1: sync MAX
	increment_time_ms();
	// From here the receiver has moved to 460800: the driver must have recorded
	// that, or a bridge opened in the same cycle would talk at 9600.
	CHECK_EQUAL(NRF_UARTE_BAUDRATE_460800, g_fake_last_baudrate);

	m.power_off();
	expect_power_off();
	increment_time_ms(200);
}

TEST(M8, FailedToReceivePVT) {
	M10QAsyncReceiver m;
	TestGNSSListener listener(m);
	GPSNavSettings settings;
	settings.assistnow_autonomous_enable = false;
	settings.assistnow_offline_enable = false;
	settings.debug_enable = true;
	settings.dyn_model = BaseGNSSDynModel::PORTABLE;
	settings.fix_mode = BaseGNSSFixMode::AUTO;
	settings.hacc_filter_en = false;
	settings.hdop_filter_en = false;
	settings.max_nav_samples = 30;

	// Power on
	expect_power_on();
	m.power_on(settings);
	increment_time_ms();
	ubx_nack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	// Configure
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_PRT);
	increment_time_ms(500);
	ubx_nack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_GNSS);
	increment_time_ms();
	increment_time_ms(200);
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_CFG);
	increment_time_ms();
	// !!! Soft Reset !!!
	increment_time_ms(1000);
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_ODO);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_TP5);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_TP5);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_PM2);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_RXM);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_NAV5);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_NAVX5);
	increment_time_ms();

	// Start receive
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();

	// Receive
	mock().expectOneCall("GPSEventError");
	// R4 (2026-08): on an unrecoverable error the driver now cuts the rail itself
	// (check_for_power_off) instead of waiting for a power_off() from the client --
	// a subscriber may perfectly well ignore the event (session already over, a
	// PWRON GNSS that subscribes nobody) and the FSM stayed frozen with the rail on.
	// The expectation must therefore be declared BEFORE the time advance that
	// triggers the error.
	expect_power_off();
	increment_time_ms(5000);
	increment_time_ms();
}

TEST(M8, PVTReportAfterPowerOffDemand) {
	M10QAsyncReceiver m;
	TestGNSSListener listener(m);
	GPSNavSettings settings;
	settings.assistnow_autonomous_enable = false;
	settings.assistnow_offline_enable = false;
	settings.debug_enable = true;
	settings.dyn_model = BaseGNSSDynModel::PORTABLE;
	settings.fix_mode = BaseGNSSFixMode::AUTO;
	settings.hacc_filter_en = false;
	settings.hdop_filter_en = false;
	settings.max_nav_samples = 30;

	// Power on
	expect_power_on();
	m.power_on(settings);
	increment_time_ms();
	ubx_nack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	// Configure
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_PRT);
	increment_time_ms(500);
	ubx_nack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_GNSS);
	increment_time_ms();
	increment_time_ms(200);
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_CFG);
	increment_time_ms();
	// !!! Soft Reset !!!
	increment_time_ms(1000);
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_ODO);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_TP5);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_TP5);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_PM2);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_RXM);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_NAV5);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_NAVX5);
	increment_time_ms();

	// Start receive
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();

	// Receive
	m.power_off();
	expect_power_off();
	ubx_pvt(-12, 20);
	ubx_status(true);
	ubx_dop();
	increment_time_ms();

	// Stop receiving
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms(100);
}

TEST(M8, PVTReportSuccessWithoutANO) {
	M10QAsyncReceiver m;
	TestGNSSListener listener(m);
	GPSNavSettings settings;
	settings.assistnow_autonomous_enable = false;
	settings.assistnow_offline_enable = false;
	settings.debug_enable = true;
	settings.dyn_model = BaseGNSSDynModel::PORTABLE;
	settings.fix_mode = BaseGNSSFixMode::AUTO;
	settings.hacc_filter_en = false;
	settings.hdop_filter_en = false;
	settings.max_nav_samples = 30;

	// Power on
	expect_power_on();
	m.power_on(settings);
	increment_time_ms();
	ubx_nack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	// Configure
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_PRT);
	increment_time_ms(500);
	ubx_nack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_GNSS);
	increment_time_ms();
	increment_time_ms(200);
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_CFG);
	increment_time_ms();
	// !!! Soft Reset !!!
	increment_time_ms(1000);
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_ODO);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_TP5);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_TP5);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_PM2);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_RXM);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_NAV5);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_NAVX5);
	increment_time_ms();

	// Start receive
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();

	// Receive
	mock().expectOneCall("GPSEventPVT");
	ubx_pvt(-12, 20);
	ubx_status(true);
	ubx_dop();
	increment_time_ms();

	// Stop receiving
	m.power_off();
	expect_power_off();
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms(100);
}

TEST(M8, PVTReportSuccessWithANO) {
	M10QAsyncReceiver m;
	TestGNSSListener listener(m);
	GPSNavSettings settings;
	settings.assistnow_autonomous_enable = true;
	settings.assistnow_offline_enable = false;
	settings.debug_enable = true;
	settings.dyn_model = BaseGNSSDynModel::PORTABLE;
	settings.fix_mode = BaseGNSSFixMode::AUTO;
	settings.hacc_filter_en = false;
	settings.hdop_filter_en = false;
	settings.max_nav_samples = 30;

	// Power on
	expect_power_on();
	m.power_on(settings);
	increment_time_ms();
	ubx_nack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	// Configure
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_PRT);
	increment_time_ms(500);
	ubx_nack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_GNSS);
	increment_time_ms();
	increment_time_ms(200);
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_CFG);
	increment_time_ms();
	// !!! Soft Reset !!!
	increment_time_ms(1000);
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_ODO);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_TP5);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_TP5);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_PM2);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_RXM);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_NAV5);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_NAVX5);
	increment_time_ms();

	// Start receive
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();

	// Receive
	mock().expectOneCall("GPSEventPVT");
	ubx_pvt(-12, 20);
	ubx_status(true);
	ubx_dop();
	increment_time_ms();

	// Stop receiving
	m.power_off();
	expect_power_off();
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	increment_time_ms(100);

	ubx_mga_ack(true, 56);
	ubx_mga_dbd(56);
	increment_time_ms(1000);
}

TEST(M8, PVTReportSuccessWithANOMissingDBDAck) {
	M10QAsyncReceiver m;
	TestGNSSListener listener(m);
	GPSNavSettings settings;
	settings.assistnow_autonomous_enable = true;
	settings.assistnow_offline_enable = false;
	settings.debug_enable = true;
	settings.dyn_model = BaseGNSSDynModel::PORTABLE;
	settings.fix_mode = BaseGNSSFixMode::AUTO;
	settings.hacc_filter_en = false;
	settings.hdop_filter_en = false;
	settings.max_nav_samples = 30;

	// Power on
	expect_power_on();
	m.power_on(settings);
	increment_time_ms();
	ubx_nack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	// Configure
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_PRT);
	increment_time_ms(500);
	ubx_nack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_GNSS);
	increment_time_ms();
	increment_time_ms(200);
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_CFG);
	increment_time_ms();
	// !!! Soft Reset !!!
	increment_time_ms(1000);
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_ODO);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_TP5);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_TP5);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_PM2);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_RXM);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_NAV5);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_NAVX5);
	increment_time_ms();

	// Start receive
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();

	// Receive
	mock().expectOneCall("GPSEventPVT");
	ubx_pvt(-12, 20);
	ubx_status(true);
	ubx_dop();
	increment_time_ms();

	// Stop receiving
	m.power_off();
	expect_power_off();
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms(1100);
}

TEST(M8, PVTReportSuccessWithANOMissingDBDs) {
	M10QAsyncReceiver m;
	TestGNSSListener listener(m);
	GPSNavSettings settings;
	settings.assistnow_autonomous_enable = true;
	settings.assistnow_offline_enable = false;
	settings.debug_enable = true;
	settings.dyn_model = BaseGNSSDynModel::PORTABLE;
	settings.fix_mode = BaseGNSSFixMode::AUTO;
	settings.hacc_filter_en = false;
	settings.hdop_filter_en = false;
	settings.max_nav_samples = 30;

	// Power on
	expect_power_on();
	m.power_on(settings);
	increment_time_ms();
	ubx_nack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	// Configure
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_PRT);
	increment_time_ms(500);
	ubx_nack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_GNSS);
	increment_time_ms();
	increment_time_ms(200);
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_CFG);
	increment_time_ms();
	// !!! Soft Reset !!!
	increment_time_ms(1000);
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_ODO);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_TP5);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_TP5);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_PM2);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_RXM);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_NAV5);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_NAVX5);
	increment_time_ms();

	// Start receive
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();

	// Receive
	mock().expectOneCall("GPSEventPVT");
	ubx_pvt(-12, 20);
	ubx_status(true);
	ubx_dop();
	increment_time_ms();

	// Stop receiving
	m.power_off();
	expect_power_off();
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	increment_time_ms(100);

	// Send MGA-DBD
	ubx_mga_ack(true, 56);
	increment_time_ms(1000);
	ubx_mga_ack(true, 56);
	increment_time_ms(1000);
	ubx_mga_ack(true, 56);
	increment_time_ms(1000);
}


TEST(M8, PVTReportSuccessWithANOAndStartNewReceive) {
	M10QAsyncReceiver m;
	TestGNSSListener listener(m);
	GPSNavSettings settings;
	settings.assistnow_autonomous_enable = true;
	settings.assistnow_offline_enable = false;
	settings.debug_enable = true;
	settings.dyn_model = BaseGNSSDynModel::PORTABLE;
	settings.fix_mode = BaseGNSSFixMode::AUTO;
	settings.hacc_filter_en = false;
	settings.hdop_filter_en = false;
	settings.max_nav_samples = 30;

	// Power on
	expect_power_on();
	m.power_on(settings);
	increment_time_ms();
	ubx_nack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	// Configure
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_PRT);
	increment_time_ms(500);
	ubx_nack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_GNSS);
	increment_time_ms();
	increment_time_ms(200);
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_CFG);
	increment_time_ms();
	// !!! Soft Reset !!!
	increment_time_ms(1000);
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_ODO);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_TP5);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_TP5);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_PM2);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_RXM);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_NAV5);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_NAVX5);
	increment_time_ms();

	// Start receive
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();

	// Receive
	mock().expectOneCall("GPSEventPVT");
	ubx_pvt(-12, 20);
	ubx_status(true);
	ubx_dop();
	increment_time_ms();

	// Stop receiving
	m.power_off();
	expect_power_off();
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	increment_time_ms(100);

	// Send MGA-DBD
	ubx_mga_ack(true, 56);
	increment_time_ms();
	ubx_mga_dbd(56);
	increment_time_ms(1000);


	// Power on
	expect_power_on();
	m.power_on(settings);
	increment_time_ms();
	ubx_nack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	// Configure
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_PRT);
	increment_time_ms(500);
	ubx_nack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_GNSS);
	increment_time_ms();
	increment_time_ms(200);
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_CFG);
	increment_time_ms();
	// !!! Soft Reset !!!
	increment_time_ms(1000);
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_ODO);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_TP5);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_TP5);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_PM2);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_RXM);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_NAV5);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_NAVX5);
	increment_time_ms();
	ubx_mga_ack(true, 0);
	increment_time_ms();

	for (unsigned int i = 0; i < 31; i++) {
		increment_time_ms(5);
	}
	for (unsigned int i = 0; i < 56; i++) {
		ubx_mga_ack(true, 0);
		increment_time_ms();
	}
	increment_time_ms(5);

	// Start receive
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
}


TEST(M8, FailedToStartReceive) {
	M10QAsyncReceiver m;
	TestGNSSListener listener(m);
	GPSNavSettings settings;
	settings.assistnow_autonomous_enable = false;
	settings.assistnow_offline_enable = false;
	settings.debug_enable = true;
	settings.dyn_model = BaseGNSSDynModel::PORTABLE;
	settings.fix_mode = BaseGNSSFixMode::AUTO;
	settings.hacc_filter_en = false;
	settings.hdop_filter_en = false;
	settings.max_nav_samples = 30;

	// Power on
	expect_power_on();
	m.power_on(settings);
	increment_time_ms();
	ubx_nack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	// Configure
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_PRT);
	increment_time_ms(500);
	ubx_nack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_GNSS);
	increment_time_ms();
	mock().expectOneCall("GPSEventError");
	// R4 (2026-08): on an unrecoverable error the driver now cuts the rail itself
	// (check_for_power_off) instead of waiting for a power_off() from the client --
	// a subscriber may perfectly well ignore the event (session already over, a
	// PWRON GNSS that subscribes nobody) and the FSM stayed frozen with the rail on.
	// The expectation must therefore be declared BEFORE the time advance that
	// triggers the error.
	expect_power_off();
	increment_time_ms(1000);
	increment_time_ms(1000);
	increment_time_ms(1000);
	increment_time_ms(1000);
	increment_time_ms();
}


TEST(M8, UartCommsErrorDuringReceive) {
	M10QAsyncReceiver m;
	TestGNSSListener listener(m);
	GPSNavSettings settings;
	settings.assistnow_autonomous_enable = false;
	settings.assistnow_offline_enable = false;
	settings.debug_enable = true;
	settings.dyn_model = BaseGNSSDynModel::PORTABLE;
	settings.fix_mode = BaseGNSSFixMode::AUTO;
	settings.hacc_filter_en = false;
	settings.hdop_filter_en = false;
	settings.max_nav_samples = 30;

	// Power on
	expect_power_on();
	m.power_on(settings);
	increment_time_ms();
	ubx_nack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	// Configure
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_PRT);
	increment_time_ms(500);
	ubx_nack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_GNSS);
	increment_time_ms();
	increment_time_ms(200);
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_CFG);
	increment_time_ms();
	// !!! Soft Reset !!!
	increment_time_ms(1000);
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_ODO);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_TP5);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_TP5);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_PM2);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_RXM);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_NAV5);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_NAVX5);
	increment_time_ms();

	// Start receive
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();

	// Receive
	inject_error(0x01);
	increment_time_ms();
	inject_error(0x01);
	mock().expectOneCall("GPSEventError");
	// R4 (2026-08): on an unrecoverable error the driver now cuts the rail itself
	// (check_for_power_off) instead of waiting for a power_off() from the client --
	// a subscriber may perfectly well ignore the event (session already over, a
	// PWRON GNSS that subscribes nobody) and the FSM stayed frozen with the rail on.
	// The expectation must therefore be declared BEFORE the time advance that
	// triggers the error.
	expect_power_off();
	increment_time_ms();
	inject_error(0x01);
	increment_time_ms();
	increment_time_ms();
}


TEST(M8, UartCommsErrorDuringConfig) {
	M10QAsyncReceiver m;
	TestGNSSListener listener(m);
	GPSNavSettings settings;
	settings.assistnow_autonomous_enable = false;
	settings.assistnow_offline_enable = false;
	settings.debug_enable = true;
	settings.dyn_model = BaseGNSSDynModel::PORTABLE;
	settings.fix_mode = BaseGNSSFixMode::AUTO;
	settings.hacc_filter_en = false;
	settings.hdop_filter_en = false;
	settings.max_nav_samples = 30;

	// Power on
	expect_power_on();
	m.power_on(settings);
	increment_time_ms();
	ubx_nack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	// Configure
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_PRT);
	increment_time_ms(500);
	increment_time_ms();
	inject_error(0x01);
	increment_time_ms();
	inject_error(0x01);
	increment_time_ms();
	inject_error(0x01);
	mock().expectOneCall("GPSEventError");
	// R4 (2026-08): on an unrecoverable error the driver now cuts the rail itself
	// (check_for_power_off) instead of waiting for a power_off() from the client --
	// a subscriber may perfectly well ignore the event (session already over, a
	// PWRON GNSS that subscribes nobody) and the FSM stayed frozen with the rail on.
	// The expectation must therefore be declared BEFORE the time advance that
	// triggers the error.
	expect_power_off();
	increment_time_ms();
	increment_time_ms();
}

// Regression R1 (2026-08) — the libuarte RX stayed dead after a tolerated error.
//
// handle_error() (ubx_comms.cpp) STOPS the RX so as not to loop on the error. The
// only place that restarted it was set_baudrate(). The branch that TOLERATES boot
// framing errors (below MAX_FRAMING_ERRORS_BOOT) did neither: the UART stayed
// deaf, not another byte could come back -- so the threshold of 10 was
// structurally unreachable -- and in state_configure past step 1 everything that
// followed was a cascade of TIMEOUTs (never ERROR), so the recovery branch that
// calls set_baudrate again was never taken. The first framing error during
// configure killed the session.
TEST(M8, FramingErrorDuringConfigureRestartsRx) {
	M10QAsyncReceiver m;
	TestGNSSListener listener(m);
	GPSNavSettings settings;
	settings.assistnow_autonomous_enable = false;
	settings.assistnow_offline_enable = false;
	settings.debug_enable = true;
	settings.dyn_model = BaseGNSSDynModel::PORTABLE;
	settings.fix_mode = BaseGNSSFixMode::AUTO;
	settings.hacc_filter_en = false;
	settings.hdop_filter_en = false;
	settings.max_nav_samples = 30;

	expect_power_on();
	m.power_on(settings);
	increment_time_ms();
	ubx_nack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	// Entering configure: step 0 (UART port VALSET) then a 1 s wait.
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_PRT);
	increment_time_ms(500);
	increment_time_ms();
	CHECK_TRUE(m_is_rx_enabled);

	// Framing error 0x04 = the NMEA->UBX transition, tolerated during boot.
	inject_error(0x04);
	// handle_error stopped the RX -- checked explicitly, or the test would pass
	// even if the error had never been delivered.
	CHECK_FALSE(m_is_rx_enabled);
	increment_time_ms();
	// ...
	// ...and the restart must have been posted to the scheduler (it cannot run in
	// ISR context).
	increment_time_ms(10);
	CHECK_TRUE(m_is_rx_enabled);

	// The receiver can therefore be heard again: the session continues.
	ubx_nack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms(100);

	m.power_off();
	expect_power_off();
	increment_time_ms();
}

TEST(M8, UartCommsErrorDuringConfigAndRecover) {
	M10QAsyncReceiver m;
	TestGNSSListener listener(m);
	GPSNavSettings settings;
	settings.assistnow_autonomous_enable = false;
	settings.assistnow_offline_enable = false;
	settings.debug_enable = true;
	settings.dyn_model = BaseGNSSDynModel::PORTABLE;
	settings.fix_mode = BaseGNSSFixMode::AUTO;
	settings.hacc_filter_en = false;
	settings.hdop_filter_en = false;
	settings.max_nav_samples = 30;

	// Power on
	expect_power_on();
	m.power_on(settings);
	increment_time_ms();
	ubx_nack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	// Configure
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_PRT);
	increment_time_ms(500);
	ubx_nack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	inject_error(0x01);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_GNSS);
	increment_time_ms();
	inject_error(0x01);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_GNSS);
	increment_time_ms();
}


TEST(M8, PVTReportSuccessWithMGAOverflow) {
	M10QAsyncReceiver m;
	TestGNSSListener listener(m);
	GPSNavSettings settings;
	settings.assistnow_autonomous_enable = true;
	settings.assistnow_offline_enable = false;
	settings.debug_enable = true;
	settings.dyn_model = BaseGNSSDynModel::PORTABLE;
	settings.fix_mode = BaseGNSSFixMode::AUTO;
	settings.hacc_filter_en = false;
	settings.hdop_filter_en = false;
	settings.max_nav_samples = 30;

	// Power on
	expect_power_on();
	m.power_on(settings);
	increment_time_ms();
	ubx_nack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	// Configure
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_PRT);
	increment_time_ms(500);
	ubx_nack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_GNSS);
	increment_time_ms();
	increment_time_ms(200);
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_CFG);
	increment_time_ms();
	// !!! Soft Reset !!!
	increment_time_ms(1000);
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_ODO);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_TP5);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_TP5);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_PM2);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_RXM);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_NAV5);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_NAVX5);
	increment_time_ms();

	// Start receive
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();

	// Receive
	mock().expectOneCall("GPSEventPVT");
	ubx_pvt(-12, 20);
	ubx_status(true);
	ubx_dop();
	increment_time_ms();

	m.power_off();
	expect_power_off();
	increment_time_ms();

	// Stop receiving
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	ubx_ack(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_MSG);
	increment_time_ms();
	increment_time_ms(100);

	// Fetch database
	ubx_mga_ack(true, 300);
	increment_time_ms();
	ubx_mga_dbd(300);
	increment_time_ms(1000);
}
