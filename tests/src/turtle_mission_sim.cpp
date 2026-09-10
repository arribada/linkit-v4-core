/**
 * @file turtle_mission_sim.cpp
 * @brief Mission simulation driving the REAL ArgosTxService over months.
 *
 * turtle_simulation_test.cpp models the ENVIRONMENT only -- dives, salinity,
 * depth, an SWS ADC value -- and asserts on dive_count and max_depth. It
 * instantiates no service, so no firmware logic is exercised by it at all.
 *
 * This file closes that gap: the same kind of environment, but wired into the
 * real ArgosTxService through the real ServiceManager, so the thing under test
 * is the code that will fly. It answers the question the bench cannot reach in
 * an overnight run: over months of dive/surface cycles, does every surfacing
 * still put a POSITION on air, or does the tag quietly settle into transmitting
 * presence heartbeats that carry no position at all?
 *
 * That distinction is the whole point. A bench counter of "TX per surfacing"
 * called the degraded state healthy, because a heartbeat is a transmission.
 */

#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

#include "argos_tx_service.hpp"
#include "gps_service.hpp"
#include "sws_analog_service.hpp"
#include "nrfx_saadc.h"
#include "fake_battery_mon.hpp"
#include "fake_config_store.hpp"
#include "fake_rtc.hpp"
#include "fake_timer.hpp"
#include "scheduler.hpp"
#include "service.hpp"

#include "gentracker.hpp"
#include "ledsm.hpp"
#include "reed.hpp"
#include "dte_handler.hpp"
#include "fake_switch.hpp"
#include "fake_reed_switch.hpp"
#include "fake_rgb_led.hpp"
#include "fake_sws.hpp"
#include "fake_led.hpp"

#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"
#include "mock_ble_serv.hpp"
#include "mock_fs.hpp"
#include "mock_logger.hpp"
#include "mock_ota_file_updater.hpp"

extern Scheduler *system_scheduler;
extern Timer *system_timer;
extern ConfigurationStore *configuration_store;
extern BatteryMonitor *battery_monitor;
extern RTC *rtc;
extern FileSystem *main_filesystem;
extern Logger *system_log;
extern Logger *sensor_log;
extern OTAFileUpdater *ota_updater;
extern DTEHandler *dte_handler;
extern BLEService *ble_service;
extern RGBLed *status_led;
extern ReedSwitch *reed_switch;

// The FSM under test, booted exactly as the firmware boots it.
FSM_INITIAL_STATE(GenTracker, BootState)
using fsm_handle = GenTracker;

namespace {

/// A Kineis device that RECORDS instead of expecting. The CppUMock mock is
/// built for a handful of calls with strict expectations; a mission puts
/// thousands of frames on air and we want their content, not a verdict.
class RecordingKineisDevice : public KineisDevice {
public:
	struct Frame {
		std::time_t when;
		unsigned int size_bits;
		bool carries_position;  ///< false => 0xFF presence filler
	};
	std::vector<Frame> frames;
	bool powered_off_immediately = false;

	void send(const KineisModulation, const KineisPacket &packet, const unsigned int size_bits) override {
		// The presence heartbeat is 0xFF filler in the position field; a real
		// Doppler or GNSS frame is not. Telling them apart is the entire reason
		// this simulation exists: a bench counter of "TX per surfacing" scored
		// the heartbeat-only state as healthy, because a heartbeat IS a frame.
		// KineisPacket is a std::string of PACKED BITS, not bytes: header 3 |
		// day 5 | hour 5 | min 6 = 19, then latitude on 21 bits. A no-fix frame
		// sets those 21 bits to all ones (argos_packet_builder.cpp:129); a real
		// position does not. Reading whole bytes -- or worse, casting the
		// std::string object itself, which the first version of this file did --
		// measures nothing: it reported 0 positions out of 20 160 frames.
		auto bit_at = [&packet](unsigned int i) -> unsigned int {
			const unsigned int byte = i / 8U;
			if (byte >= packet.size()) return 0U;
			return ((unsigned char)packet[byte] >> (7U - (i % 8U))) & 1U;
		};
		bool lat_all_ones = true;
		for (unsigned int i = 19; i < 19 + 21; i++)
			if (!bit_at(i)) {
				lat_all_ones = false;
				break;
			}
		const bool carries_pos = (size_bits >= 40) && !lat_all_ones;
		frames.push_back({rtc ? rtc->gettime() : 0, size_bits, carries_pos});
	}
	void stop_send() override {}
	void power_off_immediate() override { powered_off_immediately = true; }
	void start_receive(const KineisModulation) override {}
	bool stop_receive() override { return true; }
	void set_frequency(double) override {}
	void set_tcxo_warmup_time(unsigned int) override {}
	bool switch_modulation(KineisModulation mode, const std::string &) override {
		m_mod = mode;
		modulation_switches++;
		return true;
	}
	KineisModulation get_current_modulation() const override { return m_mod; }
	void set_lpm_mode(uint8_t) override {}

	unsigned int modulation_switches = 0;

private:
	KineisModulation m_mod = KineisModulation::LDA2;
};

}  // namespace


/// A receiver that answers on a schedule instead of over a wire. power_on()
/// arms either a fix after a time-to-first-fix drawn from the environment, or
/// an acquisition timeout -- the two outcomes GPSService has to cope with.
class SimGPSDevice : public GPSDevice {
public:
	unsigned int ttff_s = 30;      ///< set by the environment before each session
	bool will_fix = true;          ///< false => the session times out
	unsigned int sessions = 0;
	unsigned int fixes = 0;
	unsigned int nofix = 0;
	bool powered = false;
	std::time_t power_on_at = 0;

	void power_on(const GPSNavSettings &) override {
		powered = true;
		sessions++;
		power_on_at = rtc ? rtc->gettime() : 0;
	}
	void power_off() override { powered = false; }

	/// Called once per simulated second by the mission loop.
	void tick() {
		if (!powered || !rtc) return;
		if ((unsigned int)(rtc->gettime() - power_on_at) < ttff_s) return;
		if (will_fix) {
			GNSSData d{};
			d.valid = 1;
			d.fixType = 3;
			d.lon = 55.24;
			d.lat = -21.09;
			d.hMSL = 0;
			d.hAcc = 5000;
			d.gSpeed = 300;  // 3 m/s: a bobbing surface fix, as the field logs show
			fixes++;
			GPSEventPVT e(d);
			notify<GPSEventPVT>(e);
		} else {
			nofix++;
			GPSEventMaxNavSamples e;
			notify<GPSEventMaxNavSamples>(e);
		}
		powered = false;
	}
};

/// The animal and its sensor, ageing.
struct Environment {
	// Dive/surface cadence of a foraging turtle.
	unsigned int dive_s = 900;
	unsigned int surface_s = 120;

	// The SWS analogue front end fouls over a deployment: the water reading
	// drifts down as growth bridges the electrodes and the air/water contrast
	// collapses. This is the ageing the bench cannot produce -- and the reason a
	// detector that looks fine on day 1 can stop separating the two states.
	uint16_t adc_air(unsigned int day) const { return (uint16_t)(50 + day / 4); }
	uint16_t adc_water(unsigned int day) const {
		unsigned int drop = day * 90;  // ~2 700 counts a month
		return (uint16_t)(15000 > drop + 2000 ? 15000 - drop : 2000);
	}
};

struct MissionStats {
	unsigned int surfacings = 0;
	unsigned int surfacings_with_position = 0;
	unsigned int surfacings_heartbeat_only = 0;
	unsigned int frames_total = 0;
	unsigned int frames_with_position = 0;
	unsigned int gnss_sessions = 0;
	unsigned int gnss_fixes = 0;
	unsigned int detector_transitions = 0;  ///< what the REAL SWS detector decided
	unsigned int first_silent_day = 0;
};

TEST_GROUP(TurtleMission) {
	RecordingKineisDevice *kineis;
	SimGPSDevice *gps_dev;
	FakeBatteryMonitor *fake_batt;
	FakeConfigurationStore *fake_cfg;
	FakeRTC *fake_rtc;
	FakeTimer *fake_timer;

	void setup() {
		ServiceManager::reset();
		fake_batt = new FakeBatteryMonitor;
		battery_monitor = fake_batt;
		fake_cfg = new FakeConfigurationStore;
		configuration_store = fake_cfg;
		configuration_store->init();
		fake_rtc = new FakeRTC;
		rtc = fake_rtc;
		fake_rtc->settime(1780000000);
		fake_timer = new FakeTimer;
		system_timer = fake_timer;
		system_scheduler = new Scheduler(system_timer);
		fake_timer->start();
		kineis = new RecordingKineisDevice;
		gps_dev = new SimGPSDevice;
		// The SWS front end calls the mocked delay_ms on every sample; a mission
		// makes hundreds of thousands of them and none is the thing under test.
		mock().ignoreOtherCalls();
	}

	void teardown() {
		mock().clear();
		delete system_scheduler;
		delete fake_timer;
		delete fake_rtc;
		delete fake_cfg;
		delete gps_dev;
		delete kineis;
		delete fake_batt;
		system_scheduler = nullptr;
		system_timer = nullptr;
		rtc = nullptr;
		configuration_store = nullptr;
		battery_monitor = nullptr;
	}

	/// The turtle profile as shipped (template_conf/turtle_gps.cfg), not a
	/// convenient bench variant. Testing anything else answers the wrong question.
	void configure_turtle_profile() {
		fake_cfg->write_param(ParamID::ARGOS_MODE, BaseArgosMode::SURFACING_BURST);
		fake_cfg->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_4);
		fake_cfg->write_param(ParamID::ARGOS_HEXID, (unsigned int)0x5A9F10U);
		fake_cfg->write_param(ParamID::NTRY_PER_MESSAGE, (unsigned int)0);
		fake_cfg->write_param(ParamID::TR_NOM, (unsigned int)60);
		fake_cfg->write_param(ParamID::UNDERWATER_EN, (bool)true);
		fake_cfg->write_param(ParamID::GNSS_EN, (bool)true);
		fake_cfg->write_param(ParamID::LB_EN, (bool)false);
		fake_cfg->write_param(ParamID::ARGOS_TX_JITTER_EN, (bool)false);
		unsigned int no_cooldown = 0U;  // as shipped: MIN_SURFACE_CYCLE_INTERVAL = 0
		fake_cfg->write_param(ParamID::MIN_SURFACE_CYCLE_INTERVAL_S, no_cooldown);
	}
};

/// Run a mission and report what actually reached the air.
static MissionStats run_mission(unsigned int days, RecordingKineisDevice *kineis, SimGPSDevice *gps_dev,
                                FakeRTC *fake_rtc, FakeTimer *fake_timer) {
	MissionStats st{};
	Environment env;

	ArgosTxService argos(*kineis);
	GPSService gps(*gps_dev, nullptr);
	SWSAnalogService sws;

	// Production starts services through ServiceManager::startall with a relay
	// callback (gentracker.cpp:681). notify_log_updated() calls ONLY that
	// callback -- it does not broadcast to peers by itself (service.cpp:317) --
	// so starting services individually leaves them deaf to each other. The
	// first version of this harness did exactly that: the SWS detector made
	// 1185 transitions and ArgosTxService heard none of them, transmitting a
	// heartbeat every 30 s for seven simulated days as if permanently surfaced.
	bool detector_underwater = true;
	unsigned int detector_transitions = 0;
	ServiceManager::startall([&](ServiceEvent &e) {
		if (e.event_source == ServiceIdentifier::UW_SENSOR && e.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
			bool st_now = std::get<bool>(e.event_data);
			if (st_now != detector_underwater) {
				detector_underwater = st_now;
				detector_transitions++;
			}
		}
		ServiceManager::notify_peer_event(e);
	});

	bool underwater = true;
	unsigned int phase_left = env.dive_s;
	size_t frames_at_surface_start = 0;
	uint64_t ms = 0;

	for (unsigned int sec = 0; sec < days * 86400U; sec++) {
		const unsigned int day = sec / 86400U;

		// The sensor, as it ages. The detector does its own filtering,
		// hysteresis and calibration on top of this raw value.
		SAADC::set_adc_value(underwater ? (int16_t)env.adc_water(day) : (int16_t)env.adc_air(day));

		if (--phase_left == 0) {
			if (underwater) {  // surfacing
				st.surfacings++;
				frames_at_surface_start = kineis->frames.size();
				underwater = false;
				phase_left = env.surface_s;
				gps_dev->ttff_s = 20 + (sec % 40);          // 20-59 s, as at sea
				gps_dev->will_fix = ((sec / 60) % 5) != 0;   // one session in five fails
			} else {  // diving
				bool got_position = false;
				for (size_t i = frames_at_surface_start; i < kineis->frames.size(); i++)
					if (kineis->frames[i].carries_position) got_position = true;
				if (got_position)
					st.surfacings_with_position++;
				else
					st.surfacings_heartbeat_only++;
				underwater = true;
				phase_left = env.dive_s;
			}
		}

		gps_dev->tick();
		ms += 1000;
		fake_rtc->settime(fake_rtc->gettime() + 1);
		fake_timer->set_counter(ms);
		// set_counter only fires timer schedules; the cooperative scheduler's own
		// task queue is drained by run(). Without this the services are registered
		// and never execute -- the first version of this harness reported 593
		// surfacings and zero frames for exactly that reason.
		system_scheduler->run();
	}

	st.frames_total = (unsigned int)kineis->frames.size();
	for (auto &f : kineis->frames)
		if (f.carries_position) st.frames_with_position++;
	st.detector_transitions = detector_transitions;
	st.gnss_sessions = gps_dev->sessions;
	st.gnss_fixes = gps_dev->fixes;
	return st;
}

TEST(TurtleMission, SevenDayMission) {
	configure_turtle_profile();
	MissionStats st = run_mission(7, kineis, gps_dev, fake_rtc, fake_timer);

	printf("\n=== MISSION 7 JOURS (profil tortue tel que livre) ===\n");
	printf("  remontees                    : %u\n", st.surfacings);
	printf("  ... avec au moins 1 position : %u\n", st.surfacings_with_position);
	printf("  ... battement seul (0 pos.)  : %u\n", st.surfacings_heartbeat_only);
	printf("  trames emises                : %u (dont %u avec position)\n", st.frames_total, st.frames_with_position);
	printf("  sessions GNSS / fixes        : %u / %u\n", st.gnss_sessions, st.gnss_fixes);
	printf("  transitions du DETECTEUR SWS : %u (attendu ~%u)\n", st.detector_transitions, st.surfacings * 2);

	CHECK(st.surfacings > 0);
}
