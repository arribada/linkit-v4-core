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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "argos_tx_service.hpp"
#include "crc8.hpp"
#include "dive_behaviour_tracker.hpp"
#include "debug.hpp"
#include "gps_service.hpp"
#include "logger.hpp"
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
extern unsigned int g_pmu_die_temp_reads;  // mock_pmu.cpp: die reads, 50 us of busy wait each
extern FileSystem *main_filesystem;
extern Logger *system_log;
extern Logger *sensor_log;
extern OTAFileUpdater *ota_updater;
extern DTEHandler *dte_handler;
extern BLEService *ble_service;
extern uint64_t g_pmu_busy_wait_us;  // tests/mocks/mock_pmu.cpp

/// Busy-wait a simulated turtle day may ask for through PMU::delay_ms/us
/// (battery samples are counted apart). Measured 0 ms on 2026-10-27; one second
/// of headroom. Blind to target-only branches: the SWS front end's per-sample
/// charge delay sits under #else of its CPPUTEST stub and never reaches here.
static constexpr uint64_t BUSY_WAIT_BUDGET_MS_PER_DAY = 1000ULL;

/// Flash-log lines a simulated turtle day may write (services only: the radio and
/// GNSS drivers are fakes here and add their own on a tag). A ratchet: 1252
/// measured on 2026-10-27 after the journal cleanup -- one line per step, the
/// sub-steps as DEBUG_DETAIL -- plus ~10 %. It was 2811 before (33.5 a
/// surfacing, ~6.5 min of flash writes a day), more than half of them TX
/// START / TX SUCCESS pairs and scheduling chatter. See
/// BatteryAndBusyWaitWithinBudget.
static constexpr unsigned int SYSLOG_LINES_RATCHET_PER_DAY = 1380U;

/// The same at SYSTEM_LOG_LEVEL=WARN: a nominal day writes no fault, and the
/// configuration warnings of a boot are the only lines left (measured on
/// 2026-10-27, see JournalAtWarnHoldsOnlyFaults).
static constexpr unsigned int SYSLOG_LINES_AT_WARN_PER_DAY = 10U;
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
		KineisPacket packet;
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
		frames.push_back({ rtc ? rtc->gettime() : 0, size_bits, carries_pos, packet });
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

/// Records like RecordingKineisDevice, and completes each transmission the way
/// a module does: TxStarted ~1 s after send(), TxComplete ~1 s later; a dive
/// (stop_send / power_off_immediate) cancels the pending completion. Without it
/// every TX ends on ArgosTxService's timeout, and the TX-complete path -- the
/// busiest single task, the costs measured on 2026-10-27 showed -- never runs.
class CompletingKineisDevice : public RecordingKineisDevice {
public:
	void send(const KineisModulation m, const KineisPacket &p, const unsigned int bits) override {
		RecordingKineisDevice::send(m, p, bits);
		const unsigned int g = ++m_generation;
		system_scheduler->post_task_prio(
		    [this, g]() {
			    if (g == m_generation) notify(KineisEventTxStarted({}));
		    },
		    "SimTxStarted", Scheduler::DEFAULT_PRIORITY, 1000);
		system_scheduler->post_task_prio(
		    [this, g]() {
			    if (g == m_generation) notify(KineisEventTxComplete({}));
		    },
		    "SimTxComplete", Scheduler::DEFAULT_PRIORITY, 2000);
	}
	void stop_send() override { m_generation++; }
	void power_off_immediate() override {
		m_generation++;
		RecordingKineisDevice::power_off_immediate();
	}

private:
	unsigned int m_generation = 0;
};

/// Counts the lines a release build commits to system.log in flash, one
/// LittleFS write each (72-480 ms, ~140 ms on an aged filesystem, measured
/// 2026-10-27). Installed as DebugLogger::system_log -- this target builds with
/// DEBUG_TO_SYSTEMLOG, as the firmware does -- it receives exactly what the
/// journal would: no DEBUG_DETAIL or TRACE, the hold window replayed, and only
/// what its SYSTEM_LOG_LEVEL lets through (set_log_level, as PreOperational
/// entry does on a tag).
class SyslogLineCounter : public Logger {
public:
	SyslogLineCounter() : Logger("SyslogLineCounter") {}
	unsigned int lines = 0;
	/// Lines by text, every run of digits folded to '#', so the 254 copies of
	/// one message count as one entry: when the ratchet trips, this says which
	/// line to look at.
	std::map<std::string, unsigned int> by_text;
	void create() override {}
	void write(void *entry) override {
		const LogEntry *e = static_cast<LogEntry *>(entry);
		const unsigned int type = e->header.log_type;
		if (type != LOG_ERROR && type != LOG_WARN && type != LOG_INFO) return;
		lines++;
		// SIM_PRINT_SYSLOG=1 prints the journal as a tag would write it, to
		// read one surfacing end to end.
		static const bool print = getenv("SIM_PRINT_SYSLOG") != nullptr;
		if (print)
			printf("SYSLOG %02u:%02u:%02u %s\n", e->header.hours, e->header.minutes, e->header.seconds,
			       reinterpret_cast<const char *>(e->data));
		std::string key;
		for (const char *s = reinterpret_cast<const char *>(e->data); *s; s++) {
			if (*s >= '0' && *s <= '9') {
				if (key.empty() || key.back() != '#') key += '#';
			} else {
				key += *s;
			}
		}
		by_text[key]++;
	}
	void print_top(unsigned int n) const {
		std::vector<std::pair<unsigned int, std::string>> v;
		for (const auto &kv : by_text)
			v.push_back({ kv.second, kv.first });
		std::sort(v.begin(), v.end(), [](const auto &a, const auto &b) { return a.first > b.first; });
		for (unsigned int i = 0; i < v.size() && i < n; i++)
			printf("  %5u  %s\n", v[i].first, v[i].second.c_str());
	}
	void read(void *, int) override {}
	unsigned int num_entries() override { return lines; }
	bool is_ready() override { return true; }
	void truncate() override {
		lines = 0;
		by_text.clear();
	}
};

}  // namespace


/// A receiver that answers on a schedule instead of over a wire. power_on()
/// arms either a fix after a time-to-first-fix drawn from the environment, or
/// an acquisition timeout -- the two outcomes GPSService has to cope with.
class SimGPSDevice : public GPSDevice {
public:
	unsigned int ttff_s = 30;  ///< set by the environment before each session
	bool will_fix = true;      ///< false => the session times out
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
	// The costs no functional assertion sees: frames still go out, later and
	// dearer. Held to a budget by BatteryAndBusyWaitWithinBudget.
	unsigned int battery_samples = 0;               ///< BatteryMonitor samples actually taken
	unsigned int max_battery_samples_to_frame = 0;  ///< worst surfacing, samples before its first frame
	uint64_t busy_wait_ms = 0;                      ///< PMU::delay_ms/us asked for, outside the battery
	std::vector<size_t> first_frames;               ///< index of the first frame of each surfacing
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
		// As shipped since 2026-10: ARGOS_ADAPTIVE_MODULATION = 0. It is also the
		// firmware default, written here so the profile does not silently follow a
		// default change. The frame then goes on the master modulation, which this
		// sim models as LDA2 (the RecordingKineisDevice's starting modulation).
		fake_cfg->write_param(ParamID::ARGOS_ADAPTIVE_MODULATION, (bool)false);
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

	// FakeConfigurationStore refreshes the battery like the real store, so these
	// count every sample a getter takes (FakeBatteryMonitor::update_calls).
	auto *batt = static_cast<FakeBatteryMonitor *>(battery_monitor);
	const unsigned int samples_at_start = batt->update_calls;
	const uint64_t busy_at_start_us = g_pmu_busy_wait_us;
	unsigned int samples_at_surfacing = 0;
	bool first_frame_pending = false;

	for (unsigned int sec = 0; sec < days * 86400U; sec++) {
		const unsigned int day = sec / 86400U;

		// The sensor, as it ages. The detector does its own filtering,
		// hysteresis and calibration on top of this raw value.
		SAADC::set_adc_value(underwater ? (int16_t)env.adc_water(day) : (int16_t)env.adc_air(day));

		if (--phase_left == 0) {
			if (underwater) {  // surfacing
				st.surfacings++;
				frames_at_surface_start = kineis->frames.size();
				samples_at_surfacing = batt->update_calls;
				first_frame_pending = true;
				underwater = false;
				phase_left = env.surface_s;
				gps_dev->ttff_s = 20 + (sec % 40);          // 20-59 s, as at sea
				gps_dev->will_fix = ((sec / 60) % 5) != 0;  // one session in five fails
			} else {                                        // diving
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

		if (first_frame_pending && kineis->frames.size() > frames_at_surface_start) {
			first_frame_pending = false;
			st.first_frames.push_back(frames_at_surface_start);
			st.max_battery_samples_to_frame =
			    std::max(st.max_battery_samples_to_frame, batt->update_calls - samples_at_surfacing);
		}
	}

	st.battery_samples = batt->update_calls - samples_at_start;
	st.busy_wait_ms = (g_pmu_busy_wait_us - busy_at_start_us) / 1000U;
	st.frames_total = (unsigned int)kineis->frames.size();
	for (auto &f : kineis->frames)
		if (f.carries_position) st.frames_with_position++;
	st.detector_transitions = detector_transitions;
	st.gnss_sessions = gps_dev->sessions;
	st.gnss_fixes = gps_dev->fixes;
	return st;
}

/// run_mission() with the counter as the flash journal and the console off
/// (the console half of every DEBUG_* line would otherwise flood the output).
static MissionStats run_mission_journaled(unsigned int days, RecordingKineisDevice *radio, SimGPSDevice *gps_dev,
                                          FakeRTC *fake_rtc, FakeTimer *fake_timer, SyslogLineCounter &journal) {
	Logger *const console = DebugLogger::console_log;
	Logger *const system = DebugLogger::system_log;
	DebugLogger::console_log = nullptr;
	DebugLogger::system_log = &journal;
	MissionStats st = run_mission(days, radio, gps_dev, fake_rtc, fake_timer);
	DebugLogger::release();  // nothing left held for the next test
	DebugLogger::console_log = console;
	DebugLogger::system_log = system;
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
	printf("  mesures batterie             : %u (pire remontee : %u avant la 1re trame)\n", st.battery_samples,
	       st.max_battery_samples_to_frame);
	printf("  attente active hors batterie : %llu ms\n", (unsigned long long)st.busy_wait_ms);

	CHECK(st.surfacings > 0);
}

/// The costs a functional assertion cannot see, held to a budget.
///
/// get_argos_configuration() sampled the battery on every call, and the services
/// call it at each scheduling step: 4023 samples a simulated day, 13 between
/// surfacing and the first frame, each a 500 ms busy-wait on KIM. Every frame
/// still went out, so every test passed -- and this file could not see it
/// either, because FakeConfigurationStore's refresh was an empty body. Three
/// budgets, so the next hidden cost fails here instead of in the field:
///  - battery samples: one per BatteryMonitor::SAMPLE_MAX_AGE_MS, none forced
///    between surfacing and the first frame beyond the one that may fall due;
///  - busy-wait through PMU::delay_ms/us: what the code measured when this test
///    was written, with headroom;
///  - lines a release build commits to the flash log, one LittleFS write each.
///    A RATCHET, not a target: today's count plus headroom, so a change adding
///    per-TX or per-surfacing lines fails here. Lower it when lines are removed.
/// Raise a ceiling only with the reason in the commit. The radio completes its
/// transmissions, so the TX-complete path runs as on a tag.
TEST(TurtleMission, BatteryAndBusyWaitWithinBudget) {
	configure_turtle_profile();
	CompletingKineisDevice radio;
	SyslogLineCounter syslog_lines;
	MissionStats st = run_mission_journaled(1, &radio, gps_dev, fake_rtc, fake_timer, syslog_lines);

	const unsigned int max_samples = (unsigned int)(86400000ULL / BatteryMonitor::SAMPLE_MAX_AGE_MS) + 1U;
	printf("\n=== BUDGET 1 JOUR (emissions completees) ===\n");
	printf("  remontees / trames           : %u / %u\n", st.surfacings, st.frames_total);
	printf("  mesures batterie             : %u (plafond %u), pire remontee : %u avant la 1re trame\n",
	       st.battery_samples, max_samples, st.max_battery_samples_to_frame);
	printf("  attente active hors batterie : %llu ms (plafond %llu)\n", (unsigned long long)st.busy_wait_ms,
	       (unsigned long long)BUSY_WAIT_BUDGET_MS_PER_DAY);
	printf("  lignes vers le journal flash : %u (plafond %u), %.1f par remontee\n", syslog_lines.lines,
	       SYSLOG_LINES_RATCHET_PER_DAY, st.surfacings ? (double)syslog_lines.lines / st.surfacings : 0.0);
	printf("  lignes les plus frequentes (chiffres -> #) :\n");
	syslog_lines.print_top(40);

	CHECK(st.surfacings > 0);
	CHECK(st.frames_total > 0);
	CHECK(st.battery_samples <= max_samples);
	CHECK(st.max_battery_samples_to_frame <= 1U);
	CHECK(st.busy_wait_ms <= BUSY_WAIT_BUDGET_MS_PER_DAY);
	CHECK(syslog_lines.lines <= SYSLOG_LINES_RATCHET_PER_DAY);
}

// The dive-frame tests need the dive info compiled in (DIVE_INFO_MESSAGE).
#if DX1_DOPPLER_EXT_BUILD
namespace {
/// Bytes on air for a payload of @p bits on LDA2 (the sim's master): 4 B steps, min 4.
unsigned int lda2_bytes(unsigned int bits) {
	unsigned int b = (bits + 7U) / 8U;
	b = (b + 3U) / 4U * 4U;
	return b < 4U ? 4U : b;
}
bool is_dive_frame(const RecordingKineisDevice::Frame &f) {
	const KineisPacket &p = f.packet;
	return f.size_bits == ArgosPacketBuilder::DIVE_FRAME_BITS && p.size() == ArgosPacketBuilder::DIVE_FRAME_BYTES
	       && (uint8_t)p[0] == 0 && (((uint8_t)p[2] >> 5) & 1U) == 1U;
}
bool dive_crcs_ok(const KineisPacket &p) {
	return (uint8_t)p[3] == CRC8::checksum(p, 24) && (uint8_t)p[23] == CRC8::checksum(p, 184);
}
}  // namespace

/// DX1 dive frame (24-byte LDA2, 2026-11) on the deployment profile: depth pile
/// 1, the same day twice, DX1 off then at AUTO. The dive frame may only REPLACE a
/// frame: same TX instants, every other frame the same size on air, never the
/// first frame of a surfacing, at most one per surfacing and one per 2 h, both
/// CRCs good. What it costs is printed against the day without it.
TEST(TurtleMission, DiveFrameOnlyReplacesAFrame) {
	configure_turtle_profile();
	fake_cfg->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);
	fake_cfg->write_param(ParamID::SWS_SENSOR_ENABLE_TX_MODE, (unsigned int)0);
	CompletingKineisDevice radio_off;
	MissionStats off = run_mission(1, &radio_off, gps_dev, fake_rtc, fake_timer);

	// A second mission in the same test inherits state from the first (it gets no
	// GNSS fix at all): rebuild the whole environment, as between two tests.
	teardown();
	setup();
	configure_turtle_profile();
	fake_cfg->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);
	fake_cfg->write_param(ParamID::SWS_SENSOR_ENABLE_TX_MODE, (unsigned int)1);  // AUTO, the template value
	DiveBehaviourTracker::reset(0);
	CompletingKineisDevice radio_on;
	MissionStats on = run_mission(1, &radio_on, gps_dev, fake_rtc, fake_timer);

	CHECK(on.surfacings > 0);
	CHECK_EQUAL(off.surfacings, on.surfacings);
	CHECK_EQUAL(off.frames_total, on.frames_total);  // the dive frame adds no TX
	const size_t n = std::min(radio_off.frames.size(), radio_on.frames.size());
	std::vector<bool> first(radio_on.frames.size(), false);
	for (size_t i : on.first_frames)
		first[i] = true;
	unsigned int dives = 0, bytes_off = 0, bytes_on = 0, moved = 0, resized = 0, first_dive = 0, bad_crc = 0;
	unsigned int too_close = 0, twice_in_one_surfacing = 0;
	std::time_t last_dive = 0;
	size_t surfacing_of_last_dive = SIZE_MAX, surfacing = 0;
	for (size_t i = 0; i < n; i++) {
		const auto &a = radio_off.frames[i];
		const auto &b = radio_on.frames[i];
		if (first[i]) surfacing++;
		bytes_off += lda2_bytes(a.size_bits);
		bytes_on += lda2_bytes(b.size_bits);
		if (a.when != b.when) moved++;
		if (!is_dive_frame(b)) {
			if (lda2_bytes(a.size_bits) != lda2_bytes(b.size_bits)) resized++;
			continue;
		}
		dives++;
		if (first[i]) first_dive++;
		if (!dive_crcs_ok(b.packet)) bad_crc++;
		if (last_dive != 0 && b.when - last_dive < 7200) too_close++;
		if (surfacing_of_last_dive == surfacing) twice_in_one_surfacing++;
		last_dive = b.when;
		surfacing_of_last_dive = surfacing;
	}
	const double extra = bytes_off ? 100.0 * ((double)bytes_on - (double)bytes_off) / (double)bytes_off : 0.0;
	printf("\n=== TRAME PLONGEE DX1, 1 JOUR, PILE 1 ===\n");
	printf("  remontees / trames           : %u / %u (DX1 coupe) ; %u / %u (DX1 actif)\n", off.surfacings,
	       off.frames_total, on.surfacings, on.frames_total);
	printf("  trames plongee               : %u (plafond 12)\n", dives);
	printf("  octets a l antenne (LDA2)    : %u -> %u (+%.1f %%)\n", bytes_off, bytes_on, extra);

	CHECK(dives > 0);
	CHECK(dives <= 12U);
	CHECK_EQUAL(0U, moved);       // every TX at the same instant
	CHECK_EQUAL(0U, resized);     // every other frame the same size on air
	CHECK_EQUAL(0U, first_dive);  // never the first frame of a surfacing
	CHECK_EQUAL(0U, bad_crc);
	CHECK_EQUAL(0U, too_close);               // one per 2 h at most
	CHECK_EQUAL(0U, twice_in_one_surfacing);  // one per surfacing at most
}

/// What the DX1 paths add to a day, against the same day with them off. The
/// host build compiles the dive info in (DIVE_INFO_MESSAGE=1): the same pile-1
/// day runs with UNP26 OFF, then at the templates' AUTO. DX1 may add its dive
/// frames and its die reads (50 µs of busy wait each), and nothing else -- not
/// one battery measurement, not one journal line.
TEST(TurtleMission, DiveFrameDayWithinBudget) {
	CompletingKineisDevice radio[2];
	SyslogLineCounter syslog_lines[2];
	MissionStats st[2];
	unsigned int reads[2] = {}, dives[2] = {};
	for (unsigned int on = 0; on < 2; on++) {
		if (on) {
			// A second mission in the same test inherits state from the first:
			// rebuild the whole environment, as between two tests.
			teardown();
			setup();
		}
		configure_turtle_profile();
		fake_cfg->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);  // the turtle deployment
		fake_cfg->write_param(ParamID::SWS_SENSOR_ENABLE_TX_MODE, on ? 1U : 0U);        // OFF, then AUTO
		DiveBehaviourTracker::reset(0);
		const unsigned int reads0 = g_pmu_die_temp_reads;
		st[on] = run_mission_journaled(1, &radio[on], gps_dev, fake_rtc, fake_timer, syslog_lines[on]);
		reads[on] = g_pmu_die_temp_reads - reads0;
		for (const auto &f : radio[on].frames)
			if (is_dive_frame(f)) dives[on]++;
	}
	const unsigned int max_samples = (unsigned int)(86400000ULL / BatteryMonitor::SAMPLE_MAX_AGE_MS) + 1U;
	const unsigned int max_reads = 24U * 60U + 1U;
	printf("\n=== BUDGET 1 JOUR, PILE 1 : DX1 COUPE -> DX1 ACTIF ===\n");
	printf("  trames plongee               : %u -> %u (plafond 12)\n", dives[0], dives[1]);
	printf("  lectures temperature         : %u -> %u (plafond %u)\n", reads[0], reads[1], max_reads);
	printf("  mesures batterie             : %u -> %u (plafond %u)\n", st[0].battery_samples, st[1].battery_samples,
	       max_samples);
	printf("  attente active hors batterie : %llu -> %llu ms (plafond %llu)\n", (unsigned long long)st[0].busy_wait_ms,
	       (unsigned long long)st[1].busy_wait_ms, (unsigned long long)BUSY_WAIT_BUDGET_MS_PER_DAY);
	printf("  lignes vers le journal flash : %u -> %u (plafond %u)\n", syslog_lines[0].lines, syslog_lines[1].lines,
	       SYSLOG_LINES_RATCHET_PER_DAY);

	CHECK_EQUAL(0U, dives[0]);  // the gate closed sends none...
	CHECK_EQUAL(0U, reads[0]);  // ...and reads no temperature
	CHECK(dives[1] > 0);        // the dive-frame path ran...
	CHECK(dives[1] <= 12U);
	CHECK(reads[1] > 0);  // ...and the underwater die read, at most one a minute
	CHECK(reads[1] <= max_reads);
	CHECK_EQUAL(st[0].frames_total, st[1].frames_total);                                // no TX added
	CHECK_EQUAL(st[0].battery_samples, st[1].battery_samples);                          // no battery measurement added
	CHECK_EQUAL(syslog_lines[0].lines, syslog_lines[1].lines);                          // no journal line added
	CHECK(st[1].busy_wait_ms <= st[0].busy_wait_ms + (reads[1] * 50U + 999U) / 1000U);  // the die reads, nothing else
	CHECK(st[1].max_battery_samples_to_frame <= 1U);
	CHECK(st[1].battery_samples <= max_samples);
	CHECK(st[1].busy_wait_ms <= BUSY_WAIT_BUDGET_MS_PER_DAY);
	CHECK(syslog_lines[1].lines <= SYSLOG_LINES_RATCHET_PER_DAY);
}
#endif  // DX1_DOPPLER_EXT_BUILD

/// The same day at SYSTEM_LOG_LEVEL=WARN (DBP02=2), the level for a tag nobody
/// will read again. A nominal mission has nothing to warn about, so the journal
/// must stay close to empty: a WARN logged for routine behaviour fails here.
/// Two did on 2026-10-27, both now INFO or DETAIL: the GNSS session cut by a
/// dive (9 a day) and the "sensor timeout" fired after every no-fix session
/// with only the GNSS bit pending, which filled nothing (17).
TEST(TurtleMission, JournalAtWarnHoldsOnlyFaults) {
	configure_turtle_profile();
	CompletingKineisDevice radio;
	SyslogLineCounter syslog_lines;
	syslog_lines.set_log_level(LOG_LEVEL_WARN);
	MissionStats st = run_mission_journaled(1, &radio, gps_dev, fake_rtc, fake_timer, syslog_lines);

	printf("\n=== JOURNAL A WARN, 1 JOUR ===\n");
	printf("  remontees / trames           : %u / %u\n", st.surfacings, st.frames_total);
	printf("  lignes vers le journal flash : %u (plafond %u)\n", syslog_lines.lines, SYSLOG_LINES_AT_WARN_PER_DAY);
	syslog_lines.print_top(20);

	CHECK(st.frames_total > 0);  // the filter changes what is written, not what the tag does
	CHECK(syslog_lines.lines <= SYSLOG_LINES_AT_WARN_PER_DAY);
}
