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
#include "filesystem.hpp"
#include "tpl_clock.hpp"
#include <vector>
#include <fstream>
#include <iterator>
#include <cstring>

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
extern FileSystem *main_filesystem;
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
		system_timer = nullptr;
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

	// ── A minimal receiver on the other end of the UART (2026-10) ──────────────
	// Reads every frame the driver transmits (fake libuarte TX log) and answers the
	// way an M10 does. The scripted tests above inject replies blind; these ones
	// need to know what was actually SENT, and in which order.
	struct TxFrame {
		uint8_t cls;
		uint8_t id;
		uint16_t len;
		uint8_t first;  // first payload byte (MGA-INI type, CFG-MSG probe class...)
	};
	std::vector<TxFrame> m_tx_frames;
	size_t m_tx_pos = 0;
	bool m_rx_auto = false;
	bool m_aop_answers = true;
	unsigned int m_aop_busy_replies = 0;
	unsigned int m_dbd_dump_msgs = 0;

	void start_fake_receiver() {
		nrf_libuarte_tx_log_clear();
		m_tx_frames.clear();
		m_tx_pos = 0;
		m_rx_auto = true;
	}

	void answer(const TxFrame &f) {
		using namespace UBX;
		if (f.cls == (uint8_t)MessageClass::MSG_CLASS_CFG) {
			if (f.id == CFG::ID_MSG && f.len >= 1 && f.first == (uint8_t)MessageClass::MSG_CLASS_BAD) {
				ubx_nack(MessageClass::MSG_CLASS_CFG, CFG::ID_MSG);  // baud probe: awake receiver NACKs
			} else if (f.id != CFG::ID_RST) {
				ubx_ack(MessageClass::MSG_CLASS_CFG, f.id);
			}
		} else if (f.cls == (uint8_t)MessageClass::MSG_CLASS_MGA) {
			if (f.id == MGA::ID_DBD && f.len == 0) {
				// Database poll: N MGA-DBD then an MGA-ACK carrying N.
				for (unsigned int i = 0; i < m_dbd_dump_msgs; i++) {
					struct {
						uint8_t data[64];
					} x = {};
					ubx_inject_message(MessageClass::MSG_CLASS_MGA, MGA::ID_DBD, x);
				}
				ubx_mga_ack(true, m_dbd_dump_msgs);
			} else if (f.id == MGA::ID_INI || f.id == MGA::ID_ANO || f.id == MGA::ID_DBD) {
				ubx_mga_ack(true, 0);
			}
		} else if (f.cls == (uint8_t)MessageClass::MSG_CLASS_NAV && f.id == NAV::ID_AOPSTATUS && f.len == 0) {
			if (!m_aop_answers) return;
			NAV::AOPSTATUS::MSG_AOPSTATUS aop = {};
			aop.aopCfg = NAV::AOPSTATUS::AOPCFG_AOP_ENABLED;
			aop.status = (m_aop_busy_replies > 0) ? 1 : 0;
			if (m_aop_busy_replies > 0) m_aop_busy_replies--;
			ubx_inject_message(MessageClass::MSG_CLASS_NAV, NAV::ID_AOPSTATUS, aop);
		} else if (f.cls == (uint8_t)MessageClass::MSG_CLASS_MON && f.id == MON::ID_VER) {
			MON::MSG_VER ver = {};
			std::strcpy(ver.swVersion, "ROM SPG 5.10");
			ubx_inject_message(MessageClass::MSG_CLASS_MON, MON::ID_VER, ver);
		} else if (f.cls == (uint8_t)MessageClass::MSG_CLASS_SEC && f.id == SEC::ID_UNIQID) {
			SEC::MSG_UNIQID uid = {};
			ubx_inject_message(MessageClass::MSG_CLASS_SEC, SEC::ID_UNIQID, uid);
		}
	}

	void poll_tx() {
		const uint8_t *log = nrf_libuarte_tx_log_data();
		const size_t size = nrf_libuarte_tx_log_size();
		while (m_tx_pos + 8 <= size) {
			if (log[m_tx_pos] != UBX::SYNC_CHAR1 || log[m_tx_pos + 1] != UBX::SYNC_CHAR2) {
				m_tx_pos++;
				continue;
			}
			const uint16_t len = (uint16_t)(log[m_tx_pos + 4] | (log[m_tx_pos + 5] << 8));
			if (m_tx_pos + 8 + len > size) break;  // rest of the frame not sent yet
			TxFrame f = { log[m_tx_pos + 2], log[m_tx_pos + 3], len, len ? log[m_tx_pos + 6] : (uint8_t)0 };
			m_tx_frames.push_back(f);
			m_tx_pos += 8 + len;
			if (m_rx_auto) answer(f);
		}
	}

	/// Advance time ms by ms, the fake receiver answering in between.
	void pump(uint64_t ms) {
		while (ms--) {
			poll_tx();
			increment_time_ms(1);
		}
		poll_tx();
	}

	int count_frames(UBX::MessageClass cls, uint8_t id, int first = -1) {
		int n = 0;
		for (auto &f : m_tx_frames)
			if (f.cls == (uint8_t)cls && f.id == id && (first < 0 || f.first == first)) n++;
		return n;
	}
	int first_index(UBX::MessageClass cls, uint8_t id, bool with_payload) {
		for (size_t i = 0; i < m_tx_frames.size(); i++)
			if (m_tx_frames[i].cls == (uint8_t)cls && m_tx_frames[i].id == id
			    && ((m_tx_frames[i].len > 0) == with_payload))
				return (int)i;
		return -1;
	}
	int last_index(UBX::MessageClass cls, uint8_t id) {
		int idx = -1;
		for (size_t i = 0; i < m_tx_frames.size(); i++)
			if (m_tx_frames[i].cls == (uint8_t)cls && m_tx_frames[i].id == id) idx = (int)i;
		return idx;
	}

	void ubx_pvt_dated(int32_t iTOW, uint16_t year, uint8_t month, uint8_t day, uint8_t hour) {
		UBX::NAV::PVT::MSG_PVT pvt = {};
		pvt.iTow = iTOW;
		pvt.year = year;
		pvt.month = month;
		pvt.day = day;
		pvt.hour = hour;
		pvt.lat = -21 * 10000000;
		pvt.lon = 55 * 10000000;
		pvt.valid = UBX::NAV::PVT::VALID::VALID_FULLY_RESOLVED | UBX::NAV::PVT::VALID_VALID_DATE
		            | UBX::NAV::PVT::VALID_VALID_TIME;
		pvt.fixType = UBX::NAV::PVT::FIXTYPE_3D;
		ubx_inject_message(UBX::MessageClass::MSG_CLASS_NAV, UBX::NAV::ID_PVT, pvt);
		UBX::NAV::STATUS::MSG_STATUS status = {};
		status.iTow = iTOW;
		status.fixStat = 1;
		ubx_inject_message(UBX::MessageClass::MSG_CLASS_NAV, UBX::NAV::ID_STATUS, status);
		UBX::NAV::DOP::MSG_DOP dop = {};
		dop.iTow = iTOW;
		ubx_inject_message(UBX::MessageClass::MSG_CLASS_NAV, UBX::NAV::ID_DOP, dop);
	}

	GPSNavSettings cold_settings(bool ana, bool ano) {
		GPSNavSettings settings;
		settings.assistnow_autonomous_enable = ana;
		settings.assistnow_offline_enable = ano;
		settings.debug_enable = false;
		settings.dyn_model = BaseGNSSDynModel::PORTABLE;
		settings.fix_mode = BaseGNSSFixMode::AUTO;
		settings.hacc_filter_en = false;
		settings.hdop_filter_en = false;
		settings.max_nav_samples = 30;
		settings.ano_stale_threshold_s = 0;
		return settings;
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


// ════════════════════════════════════════════════════════════════════════════
// 2026-10 — what the driver persists and replays around a GNSS restart.
// These tests run the driver against the fake receiver above and check the
// frames it actually sends.
// ════════════════════════════════════════════════════════════════════════════

namespace {
constexpr unsigned int RAM_BLOCK_COUNT = 64;
constexpr unsigned int RAM_BLOCK_SIZE = 4096;
constexpr unsigned int RAM_PAGE_SIZE = 256;
constexpr std::time_t T_2025_10_09_0853 = 1760000000;  // 2025-10-09 08:53:20 UTC
constexpr std::time_t T_ANO_FIXTURE_DAY = 1651665413;  // 2022-05-04, the day data/mgaoffline.ubx covers

std::vector<uint8_t> ubx_frame(UBX::MessageClass cls, uint8_t id, const uint8_t *payload, uint16_t len) {
	std::vector<uint8_t> f = { UBX::SYNC_CHAR1,       UBX::SYNC_CHAR2,    (uint8_t)cls, id,
	                           (uint8_t)(len & 0xFF), (uint8_t)(len >> 8) };
	f.insert(f.end(), payload, payload + len);
	uint8_t ck_a = 0, ck_b = 0;
	for (size_t i = 2; i < f.size(); i++) {
		ck_a = ck_a + f[i];
		ck_b = ck_b + ck_a;
	}
	f.push_back(ck_a);
	f.push_back(ck_b);
	return f;
}

void write_file(FileSystem *fs, const char *name, const std::vector<uint8_t> &bytes) {
	LFSFile file(fs, name, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);
	file.write((void *)bytes.data(), (lfs_size_t)bytes.size());
}

/// A persisted DBD as save_dbd_to_flash() writes it: timestamp, then MGA-DBD frames.
std::vector<uint8_t> dbd_file(uint32_t save_time, unsigned int frames) {
	std::vector<uint8_t> out(sizeof(save_time));
	std::memcpy(out.data(), &save_time, sizeof(save_time));
	uint8_t payload[60];
	for (unsigned int i = 0; i < frames; i++) {
		std::memset(payload, (int)i + 1, sizeof(payload));
		auto f = ubx_frame(UBX::MessageClass::MSG_CLASS_MGA, UBX::MGA::ID_DBD, payload, sizeof(payload));
		out.insert(out.end(), f.begin(), f.end());
	}
	return out;
}

struct RamFs {
	RamFlash flash{ RAM_BLOCK_SIZE, RAM_BLOCK_COUNT, RAM_PAGE_SIZE };  // (block size, block count, page size)
	LFSFileSystem fs{ &flash };
	RamFs() {
		fs.format();
		fs.mount();
		main_filesystem = &fs;
	}
	~RamFs() {
		main_filesystem = nullptr;
		fs.umount();
	}
};
}  // namespace

// ── Correction 1: the position restored from flash reaches the receiver ────────

TEST(M8, ColdStartInjectsTheRestoredPositionHint) {
	mock().ignoreOtherCalls();
	fake_rtc->settime(T_2025_10_09_0853);
	fake_rtc->note_source(RtcSource::GNSS);  // a bounded time
	GPSLogEntry hint{};
	hint.header.log_type = LOG_GPS;
	hint.info.event_type = GPSEventType::FIX;
	hint.info.valid = true;
	hint.info.lat = -21.0;
	hint.info.lon = 55.3;
	hint.info.year = 2025;
	hint.info.month = 10;
	hint.info.day = 9;
	hint.info.hour = 7;  // ~2 h old
	hint.info.hAcc = 5000;
	fake_config->set_restored_position_hint(hint);

	M10QAsyncReceiver m;
	TestGNSSListener listener(m);
	start_fake_receiver();
	m.power_on(cold_settings(false, false));
	pump(6000);

	CHECK_EQUAL(1, count_frames(UBX::MessageClass::MSG_CLASS_MGA, UBX::MGA::ID_INI, 0x10));  // MGA-INI-TIME_UTC
	CHECK_EQUAL(1, count_frames(UBX::MessageClass::MSG_CLASS_MGA, UBX::MGA::ID_INI, 0x01));  // MGA-INI-POS_LLH
	CHECK_TRUE(count_frames(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_VALSET) > 0);
}

TEST(M8, ColdStartWithoutAnyPositionSendsNoPositionAiding) {
	mock().ignoreOtherCalls();
	fake_rtc->settime(T_2025_10_09_0853);
	fake_rtc->note_source(RtcSource::GNSS);

	M10QAsyncReceiver m;
	TestGNSSListener listener(m);
	start_fake_receiver();
	m.power_on(cold_settings(false, false));
	pump(6000);

	CHECK_EQUAL(1, count_frames(UBX::MessageClass::MSG_CLASS_MGA, UBX::MGA::ID_INI, 0x10));
	CHECK_EQUAL(0, count_frames(UBX::MessageClass::MSG_CLASS_MGA, UBX::MGA::ID_INI, 0x01));
}

// ── Correction 3: the DBD is replayed even when an ANO file serves today ───────

TEST(M8, ColdStartReplaysTheDbdBeforeTheAno) {
	mock().ignoreOtherCalls();
	RamFs ram;
	std::ifstream ano("./data/mgaoffline.ubx", std::ios::binary);
	std::vector<uint8_t> ano_bytes((std::istreambuf_iterator<char>(ano)), std::istreambuf_iterator<char>());
	CHECK_TRUE(ano_bytes.size() > 0);
	write_file(&ram.fs, "gps_config.dat", ano_bytes);
	write_file(&ram.fs, "gnss_dbd.dat", dbd_file((uint32_t)(T_ANO_FIXTURE_DAY - 600), 3));
	fake_rtc->settime(T_ANO_FIXTURE_DAY);
	fake_rtc->note_source(RtcSource::GNSS);

	M10QAsyncReceiver m;
	TestGNSSListener listener(m);
	start_fake_receiver();
	m.power_on(cold_settings(false, true));
	pump(8000);

	const int first_dbd = first_index(UBX::MessageClass::MSG_CLASS_MGA, UBX::MGA::ID_DBD, true);
	const int first_ano = first_index(UBX::MessageClass::MSG_CLASS_MGA, UBX::MGA::ID_ANO, true);
	CHECK_TRUE(first_dbd >= 0);  // before 2026-10: never sent once the ANO had produced a byte
	CHECK_TRUE(first_ano >= 0);
	CHECK_TRUE(first_dbd < first_ano);
	CHECK_EQUAL(3, count_frames(UBX::MessageClass::MSG_CLASS_MGA, UBX::MGA::ID_DBD));
	CHECK_EQUAL(30, count_frames(UBX::MessageClass::MSG_CLASS_MGA, UBX::MGA::ID_ANO));
	// The chain still ends in reception.
	CHECK_TRUE(last_index(UBX::MessageClass::MSG_CLASS_CFG, UBX::CFG::ID_VALSET) > first_ano);
}

// ── Correction 4: a deferred DBD survives an immediate power-off ──────────────

TEST(M8, ImmediatePowerOffCommitsTheDeferredDbd) {
	mock().ignoreOtherCalls();
	RamFs ram;
	fake_rtc->settime(T_2025_10_09_0853);
	fake_rtc->note_source(RtcSource::GNSS);
	m_dbd_dump_msgs = 3;

	M10QAsyncReceiver m;
	TestGNSSListener listener(m);
	start_fake_receiver();
	m.power_on(cold_settings(false, false));
	pump(6000);
	ubx_pvt_dated(1000, 2025, 10, 9, 8);
	pump(10);
	m.power_off();
	pump(3000);  // stop, DBD read-out, rail cut; the flash write is deferred by 15 s

	CHECK_EQUAL(0u, M10QAsyncReceiver::persisted_dbd_time(&ram.fs));

	m.power_off_immediate();  // OffState / configuration mode / OTA path

	CHECK_TRUE(M10QAsyncReceiver::persisted_dbd_time(&ram.fs) > 0);
}

TEST(M8, FlushPendingPersistenceCommitsTheDeferredDbd) {
	mock().ignoreOtherCalls();
	RamFs ram;
	fake_rtc->settime(T_2025_10_09_0853);
	fake_rtc->note_source(RtcSource::GNSS);
	m_dbd_dump_msgs = 3;

	M10QAsyncReceiver m;
	TestGNSSListener listener(m);
	start_fake_receiver();
	m.power_on(cold_settings(false, false));
	pump(6000);
	ubx_pvt_dated(1000, 2025, 10, 9, 8);
	pump(10);
	m.power_off();
	pump(3000);
	CHECK_EQUAL(0u, M10QAsyncReceiver::persisted_dbd_time(&ram.fs));

	GPSDevice &dev = m;
	dev.flush_pending_persistence();  // what PMU::powerdown() now calls

	CHECK_TRUE(M10QAsyncReceiver::persisted_dbd_time(&ram.fs) > 0);
}

// ── Correction 5: wait for AssistNow Autonomous before the DBD read-out ───────

TEST(M8, StopWaitsForAssistNowAutonomousBeforeTheDbdReadout) {
	mock().ignoreOtherCalls();
	m_aop_busy_replies = 2;  // busy, busy, then idle
	m_dbd_dump_msgs = 2;

	M10QAsyncReceiver m;
	TestGNSSListener listener(m);
	start_fake_receiver();
	m.power_on(cold_settings(true, false));
	pump(6000);
	ubx_pvt_dated(1000, 2025, 10, 9, 8);
	pump(10);
	m.power_off();
	pump(1000);

	// Still busy: the database has not been read out yet.
	CHECK_TRUE(count_frames(UBX::MessageClass::MSG_CLASS_NAV, UBX::NAV::ID_AOPSTATUS) >= 1);
	CHECK_EQUAL(-1, first_index(UBX::MessageClass::MSG_CLASS_MGA, UBX::MGA::ID_DBD, false));

	pump(5000);

	CHECK_EQUAL(3, count_frames(UBX::MessageClass::MSG_CLASS_NAV, UBX::NAV::ID_AOPSTATUS));
	const int dbd_poll = first_index(UBX::MessageClass::MSG_CLASS_MGA, UBX::MGA::ID_DBD, false);
	CHECK_TRUE(dbd_poll > last_index(UBX::MessageClass::MSG_CLASS_NAV, UBX::NAV::ID_AOPSTATUS));
}

TEST(M8, StopStopsWaitingForAssistNowAutonomousAtTheCap) {
	mock().ignoreOtherCalls();
	m_aop_busy_replies = 1000;  // never goes idle

	M10QAsyncReceiver m;
	TestGNSSListener listener(m);
	start_fake_receiver();
	m.power_on(cold_settings(true, false));
	pump(6000);
	ubx_pvt_dated(1000, 2025, 10, 9, 8);
	pump(10);
	m.power_off();
	pump(19000);
	CHECK_EQUAL(-1, first_index(UBX::MessageClass::MSG_CLASS_MGA, UBX::MGA::ID_DBD, false));

	pump(4000);  // past AOP_WAIT_MAX_MS: powered off anyway

	const int polls = count_frames(UBX::MessageClass::MSG_CLASS_NAV, UBX::NAV::ID_AOPSTATUS);
	CHECK_TRUE(polls >= 20 && polls <= 22);
	CHECK_TRUE(first_index(UBX::MessageClass::MSG_CLASS_MGA, UBX::MGA::ID_DBD, false) >= 0);
}

TEST(M8, StopDoesNotWaitWhenAopStatusIsUnanswered) {
	mock().ignoreOtherCalls();
	m_aop_answers = false;

	M10QAsyncReceiver m;
	TestGNSSListener listener(m);
	start_fake_receiver();
	m.power_on(cold_settings(true, false));
	pump(6000);
	ubx_pvt_dated(1000, 2025, 10, 9, 8);
	pump(10);
	m.power_off();
	pump(1500);

	CHECK_EQUAL(1, count_frames(UBX::MessageClass::MSG_CLASS_NAV, UBX::NAV::ID_AOPSTATUS));
	CHECK_TRUE(first_index(UBX::MessageClass::MSG_CLASS_MGA, UBX::MGA::ID_DBD, false) >= 0);
}

TEST(M8, StopDoesNotPollAopStatusWithAnaDisabled) {
	mock().ignoreOtherCalls();

	M10QAsyncReceiver m;
	TestGNSSListener listener(m);
	start_fake_receiver();
	m.power_on(cold_settings(false, false));
	pump(6000);
	ubx_pvt_dated(1000, 2025, 10, 9, 8);
	pump(10);
	m.power_off();
	pump(1500);

	CHECK_EQUAL(0, count_frames(UBX::MessageClass::MSG_CLASS_NAV, UBX::NAV::ID_AOPSTATUS));
	CHECK_TRUE(first_index(UBX::MessageClass::MSG_CLASS_MGA, UBX::MGA::ID_DBD, false) >= 0);
}

// ── Correction 2: the time reaches the flash, and the RSPB bound is honest ────

TEST(M8, FirstGnssSyncOfTheBootIsCommittedOnce) {
	mock().ignoreOtherCalls();
	FakeConfigurationStore *store = static_cast<FakeConfigurationStore *>(fake_config);
	store->init();

	M10QAsyncReceiver m;
	TestGNSSListener listener(m);
	start_fake_receiver();
	m.power_on(cold_settings(false, false));
	pump(6000);
	const unsigned int saves_before = store->get_saved_count();
	ubx_pvt_dated(1000, 2025, 10, 9, 8);
	pump(10);

	const unsigned int t = configuration_store->read_param<unsigned int>(ParamID::LAST_TIME_SYNC);
	CHECK_TRUE(t > 0);
	CHECK_EQUAL(t, configuration_store->read_param<unsigned int>(ParamID::LAST_KNOWN_RTC));
	CHECK_EQUAL(saves_before, store->get_saved_count());  // deferred off the fix -> TX path

	pump(16000);
	CHECK_EQUAL(saves_before + 1, store->get_saved_count());

	// A second session in the same boot does not write the flash again.
	m.power_off();
	pump(3000);
	m.power_on(cold_settings(false, false));
	pump(6000);
	ubx_pvt_dated(2000, 2025, 10, 9, 9);
	pump(16000);
	CHECK_EQUAL(saves_before + 1, store->get_saved_count());
}

TEST(M8, PersistedDbdTimeReadsTheFileHeader) {
	RamFs ram;
	CHECK_EQUAL(0u, M10QAsyncReceiver::persisted_dbd_time(&ram.fs));  // no file
	CHECK_EQUAL(0u, M10QAsyncReceiver::persisted_dbd_time(nullptr));
	write_file(&ram.fs, "gnss_dbd.dat", dbd_file(1234567890u, 2));
	CHECK_EQUAL(1234567890u, M10QAsyncReceiver::persisted_dbd_time(&ram.fs));
}

TEST_GROUP(PseudoRtcBound){};

TEST(PseudoRtcBound, GrowsWithTheTimeSinceTheLastTrustedSetting) {
	// One hop of 1 h after a fix, nominal period: 10 s + 5 % of 3600 s.
	CHECK_EQUAL(190u, RTC::pseudo_uncertainty_s(1000 + 3600, 1000, TplClock::TOLERANCE_PERMILLE));
	// A day of hops without a fix: the tolerance adds up -- 10 s + 5 % of 86400 s.
	// The one-hop bound used before claimed 181 s here.
	CHECK_EQUAL(4330u, RTC::pseudo_uncertainty_s(1000 + 86400, 1000, TplClock::TOLERANCE_PERMILLE));
	// Once the resistor's real period is learnt: 10 s + 2 % of 3600 s.
	CHECK_EQUAL(82u, RTC::pseudo_uncertainty_s(1000 + 3600, 1000, TplClock::CALIBRATED_TOLERANCE_PERMILLE));
}

TEST(PseudoRtcBound, UnboundedWithoutAUsableAnchor) {
	CHECK_EQUAL(0u, RTC::pseudo_uncertainty_s(5000, 0, 50));        // never synchronised
	CHECK_EQUAL(0u, RTC::pseudo_uncertainty_s(5000, 6000, 50));     // anchor in the future
	CHECK_EQUAL(0u, RTC::pseudo_uncertainty_s(2000000, 1000, 50));  // beyond what tAccS can carry
}

// ── The TPL5111 period this board's resistor really gives (TplClock) ──────────

TEST_GROUP(TplClock){};

TEST(TplClock, ChainUsesTheNominalPeriodUntilCalibrated) {
	CHECK_EQUAL(3600u, TplClock::chain_period_s(3600, 0));
	CHECK_EQUAL(3550u, TplClock::chain_period_s(3600, 3550400));  // learnt, rounded to the second
	CHECK_EQUAL(3600u, TplClock::chain_period_s(3600, 3000000));  // -17 %: not a resistor tolerance
	CHECK_EQUAL(TplClock::TOLERANCE_PERMILLE, TplClock::tolerance_permille(3600, 0));
	CHECK_EQUAL(TplClock::CALIBRATED_TOLERANCE_PERMILLE, TplClock::tolerance_permille(3600, 3550400));
}

TEST(TplClock, MeasuresTheRealPeriodFromTheErrorAtTheFix) {
	// Anchor: a fix 300 s into the run that booted at A. Two hops later the chain
	// (3600 s per hop) says A + 7200, GNSS says the board booted at A + 7100:
	// the resistor gives 3550 s.
	const std::time_t A = 1760000000;
	CHECK_EQUAL(3550000u, TplClock::measure_period_ms(3600000, A + 7200, A + 7100, A + 300));
	// A clock running slow measures a longer period.
	CHECK_EQUAL(3660000u, TplClock::measure_period_ms(3600000, A + 7200, A + 7320, A + 300));
}

TEST(TplClock, NothingToMeasureWithoutAHop) {
	const std::time_t A = 1760000000;
	CHECK_EQUAL(0u, TplClock::measure_period_ms(3600000, A + 100, A + 90, 0));         // no anchor
	CHECK_EQUAL(0u, TplClock::measure_period_ms(3600000, A + 100, A + 90, A + 300));   // same run as the anchor
	CHECK_EQUAL(0u, TplClock::measure_period_ms(3600000, A + 900, A + 890, A + 300));  // less than half a hop
}

TEST(TplClock, CalibrationStartsSmoothsAndRestarts) {
	// First measurement: taken as is.
	CHECK_EQUAL(3550000u, TplClock::update_calibration_ms(3600, 0, 3550000));
	// Small move (temperature, supply): averaged in.
	CHECK_EQUAL(3552500u, TplClock::update_calibration_ms(3600, 3550000, 3560000));
	// A jump past 3 % from the calibration: restart from the measurement.
	CHECK_EQUAL(3450000u, TplClock::update_calibration_ms(3600, 3580000, 3450000));
	// Past 10 % from the nominal: a WAKEUP_PERIOD that does not match the resistor,
	// never learnt.
	CHECK_EQUAL(3550000u, TplClock::update_calibration_ms(3600, 3550000, 3000000));
	CHECK_EQUAL(0u, TplClock::update_calibration_ms(3600, 0, 0));
}

TEST(TplClock, LearntPeriodKeepsTheChainInsideItsBound) {
	// A board whose resistor gives 3530 s (-1.9 %, a 1 % resistor at 1 h does that),
	// configured as 3600 s, one fix every two hops. Before the first fix the chain
	// gains 70 s a hop; after two fixes it runs at the resistor's real period.
	const uint32_t nominal = 3600, real = 3530;
	const std::time_t A = 1760000000;
	uint32_t cal = 0;
	std::time_t anchor = A + 300, true_boot = A;
	for (int fix = 0; fix < 2; fix++) {
		const uint32_t chain = TplClock::chain_period_s(nominal, cal);
		const std::time_t pseudo_boot = true_boot + 2 * chain;
		true_boot += 2 * real;
		cal = TplClock::update_calibration_ms(
		    nominal, cal, TplClock::measure_period_ms(chain * 1000U, pseudo_boot, true_boot, anchor));
		anchor = true_boot + 300;
	}
	CHECK_EQUAL(real, TplClock::chain_period_s(nominal, cal));
}

#ifdef EXTERNAL_WAKEUP  // WAKEUP_PERIOD exists in ParamID on TPL5111 builds only
TEST(TplClock, ANewWakeupPeriodForgetsTheLearntOne) {
	FakeConfigurationStore store;
	store.init();
	store.write_param(ParamID::WAKEUP_PERIOD, 3600U);
	store.write_param(ParamID::WAKEUP_PERIOD_MEASURED_MS, 3550000U);
	store.write_param(ParamID::WAKEUP_PERIOD, 3600U);  // a full configuration upload rewrites it
	CHECK_EQUAL(3550000u, store.read_param<unsigned int>(ParamID::WAKEUP_PERIOD_MEASURED_MS));
	store.write_param(ParamID::WAKEUP_PERIOD, 7200U);  // another resistor
	CHECK_EQUAL(0u, store.read_param<unsigned int>(ParamID::WAKEUP_PERIOD_MEASURED_MS));
}
#endif

TEST(M8, FirstFixOfATplCycleResetsTheAnchor) {
	mock().ignoreOtherCalls();
	FakeConfigurationStore *store = static_cast<FakeConfigurationStore *>(fake_config);
	store->init();
	// GNSS truth at the fix: 2025-10-09 09:00:00. The chain, two hops after an
	// anchor taken 300 s into its run, put this boot 100 s too late.
	const std::time_t t_fix = 1760000400;
	const std::time_t up_s = 6;  // pump(6000) before the PVT
	const std::time_t pseudo_boot = t_fix - up_s + 100;
	store->write_param(ParamID::LAST_TIME_SYNC, (unsigned int)(pseudo_boot - 7200 + 300));
	fake_rtc->settime(pseudo_boot);
	fake_rtc->note_source(RtcSource::PSEUDO);
	fake_rtc->set_pseudo_uncertainty_s(370);

	M10QAsyncReceiver m;
	TestGNSSListener listener(m);
	start_fake_receiver();
	m.power_on(cold_settings(false, false));
	pump(6000);
	ubx_pvt_dated(1000, 2025, 10, 9, 9);
	pump(10);

	CHECK_EQUAL((unsigned int)t_fix, configuration_store->read_param<unsigned int>(ParamID::LAST_TIME_SYNC));
	CHECK_EQUAL((std::time_t)t_fix + 0, rtc->gettime());
#ifdef EXTERNAL_WAKEUP
	CHECK_EQUAL(3550000u, configuration_store->read_param<unsigned int>(ParamID::WAKEUP_PERIOD_MEASURED_MS));
#endif
}

TEST(PseudoRtcBound, PersistedTimeIsTheWallTimeOffTpl) {
#ifdef EXTERNAL_WAKEUP
	// TPL5111 board: the instant this run booted (the period runs wake to wake).
	CHECK_EQUAL((std::time_t)9000, RTC::persisted_time(10000, 1000000));
#else
	CHECK_EQUAL((std::time_t)10000, RTC::persisted_time(10000, 1000000));
#endif
}

TEST(M8, AFixInARunNotStartedByTheTplDoesNotAnchorTheChain) {
	mock().ignoreOtherCalls();
	FakeConfigurationStore *store = static_cast<FakeConfigurationStore *>(fake_config);
	store->init();
	store->write_param(ParamID::LAST_TIME_SYNC, 1234u);
	fake_rtc->set_chain_anchorable(false);  // magnet / watchdog / OTA boot on a TPL5111 board

	M10QAsyncReceiver m;
	TestGNSSListener listener(m);
	start_fake_receiver();
	m.power_on(cold_settings(false, false));
	pump(6000);
	ubx_pvt_dated(1000, 2025, 10, 9, 9);
	pump(10);

	// The clock is right (the fix set it), but nothing says when the next TPL wake
	// will come: no anchor.
	CHECK_EQUAL((std::time_t)1760000400, rtc->gettime());
	CHECK_EQUAL(0u, configuration_store->read_param<unsigned int>(ParamID::LAST_TIME_SYNC));
}
