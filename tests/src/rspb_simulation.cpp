/**
 * @file rspb_simulation.cpp
 * @brief Long-term RSPB bird-tracker mortality simulation (the avian analogue
 *        of the 1-year TurtleSimulation).
 *
 * Drives the REAL firmware MortalityService (core/services/mortality_service.cpp)
 * over a multi-month deployment. The mission is modelled at the TPL5111 wake
 * level: the external timer re-powers the tag every WAKEUP_PERIOD seconds; on
 * each wake the firmware boot-counter/modulo decides whether to run a full
 * active cycle (GPS + sensors + mortality evaluation) or power straight back
 * down. Active wakes reload the persisted MortalityInfo from FsLog (emulating
 * the power-cut + reboot), feed AXL / thermistor / GNSS peer events exactly as
 * the sensor services would, and read back the confidence / status the service
 * computes. Because the active-wake cadence is driven by the *real*
 * BOOT_COUNTER_MODULO the MortalityService writes, the battery saving from the
 * dead-bird duty-cycle adaptation is measured, not assumed.
 *
 * Scenarios covered by the test cases below:
 *   - open-field death (fixes continue on a frozen carcass)      -> normal path
 *   - dense-canopy death (NO valid fix after death)              -> H4 fallback
 *   - live bird for the whole deployment                         -> no false positive
 *   - roosting live bird (stationary + low activity + warm)      -> no false positive
 *   - false alarm then recovery                                  -> H5 modulo restore
 *   - multi-seed robustness                                      -> deterministic detection
 *   - duty-modulo / confirm-days corner cases                    -> param edges
 *   - late death near the deployment boundary                    -> relaxed invariants
 *
 * The open-field test also writes four accessible artefacts (override the
 * output dir with RSPB_REPORT_DIR):
 *   - rspb_mortality_report.html  visual report (inline SVG charts, no assets)
 *   - rspb_mortality.csv          per-active-wake metrics for spreadsheets
 *   - rspb_mortality_log.csv      the genuine on-device MortalityService log
 *   - rspb_mission.log            human-readable per-wake event log
 *
 * Run:
 *   ./build/RSPBSimulation -v
 */

#include "CppUTest/TestHarness.h"

#include "fake_config_store.hpp"
#include "fake_rtc.hpp"
#include "fake_timer.hpp"
#include "scheduler.hpp"
#include "service_scheduler.hpp"
#include "mortality_service.hpp"
#include "fs_log.hpp"
#include "filesystem.hpp"
#include "haversine.hpp"

#include <cstdlib>
#include <cstring>
#include <cmath>
#include <ctime>
#include <random>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <algorithm>

extern ConfigurationStore *configuration_store;
extern RTC   *rtc;
extern Timer *system_timer;
extern Scheduler *system_scheduler;
extern FileSystem *main_filesystem;

namespace {

constexpr double PI = 3.14159265358979323846;

enum class DeathMode { OPEN, CANOPY, NONE };

// =====================================================================
// Simulation configuration
// =====================================================================
struct SimConfig {
	int    deploy_days      = 120;   ///< total deployment length (days)
	int    death_day        = 60;    ///< day the bird dies (0-based); ignored if NONE
	DeathMode death_mode    = DeathMode::OPEN;
	int    recover_day      = -1;    ///< >=0 and > death_day: bird revives (H5 test)
	bool   roosting         = false; ///< quiescent live bird: stationary + low act + warm
	unsigned int seed       = 20260718u;

	// --- TPL5111 wake model ---
	unsigned int wake_period_s = 6300;  ///< external-timer period (~1h45), WAKEUP_PERIOD default

	// --- Bird biology model ---
	double alive_temp_c     = 40.5;  ///< avian body temperature
	double ambient_temp_c   = 11.0;  ///< carcass cools toward this
	double temp_decay_tau_h = 2.5;   ///< exponential cooling time constant (hours)
	double base_lat         = 52.50; ///< deployment site (UK midlands)
	double base_lon         = -1.90;

	// --- Battery model (small bird tag) ---
	double capacity_mah        = 1000.0;
	double active_session_mah  = 0.55; ///< GPS(~30s@25mA)+Argos burst+sensors per active wake
	double tpl_wake_mah        = 0.02; ///< boot + modulo-check + powerdown on a skipped wake

	// --- MortalityService params (written into the config store) ---
	unsigned int activity_thresh   = 10;    ///< MTP02
	double       temp_thresh_c     = 25.0;  ///< MTP03
	unsigned int gps_dist_thresh_m = 50;    ///< MTP04
	unsigned int confirm_days      = 3;     ///< MTP05
	unsigned int duty_modulo       = 12;    ///< MTP06 dead-bird cadence
	unsigned int normal_modulo     = 4;     ///< PWP03 live cadence

	// 2026-04-01 00:00:00 UTC — fixed so the run is fully deterministic
	// (deployment ends ~2026-07-30 for the default 120-day mission).
	std::time_t  start_epoch    = 1775001600;
};

// Per active-wake record.
struct Rec {
	std::time_t epoch;
	int      day;
	bool     alive_model;   ///< true = live bird model this wake, false = dead
	bool     gps_valid;
	double   activity;
	double   body_temp;
	double   moved_m;
	int      gps_speed;
	unsigned confidence;
	unsigned consecutive_days;
	int      status;        ///< 0 ALIVE / 1 SUSPECTED / 2 CONFIRMED
	unsigned modulo;
	double   batt_pct;
};

// Per-day aggregate (drives the charts + table).
struct DayS {
	int      day;
	bool     alive_model;
	bool     had_active;
	double   avg_activity;
	double   avg_temp;
	double   avg_moved;
	int      nofix;
	int      active_wakes;
	unsigned end_confidence;
	unsigned end_consecutive;
	int      end_status;
	unsigned end_modulo;
	double   batt_pct;
};

struct SimResult {
	SimConfig cfg;
	int    detection_day   = -1;   ///< first day status became CONFIRMED
	unsigned final_confidence = 0;
	int    final_status    = 0;
	unsigned final_consecutive = 0;
	unsigned final_modulo  = 0;
	unsigned alive_max_conf = 0;   ///< max confidence during ANY alive phase
	int    alive_max_status = 0;   ///< max status during ANY alive phase
	unsigned max_conf_overall = 0;
	unsigned modulo_while_alive = 0;
	bool   ever_confirmed  = false;
	int    recovered_day   = -1;   ///< first day status left CONFIRMED after a detection
	double batt_pct = 100.0;
	double batt_pct_noadapt = 100.0;
	long   total_wakes = 0;
	long   active_wakes = 0;
	long   active_wakes_noadapt = 0;
	int    devlog_entries = 0;
};

// Expose the protected service_init() so the harness can simulate a TPL5111
// power-cut + reboot: each active wake reloads the persisted MortalityInfo from
// FsLog, exactly as the real device does when the external timer re-powers it.
class SimMortalityService : public MortalityService {
public:
	explicit SimMortalityService(Logger *l) : MortalityService(l) {}
	void sim_wake() { service_init(); }
};

// ---------------------------------------------------------------------
// Peer-event helpers — build the exact ServiceEvent a sensor service emits.
// ---------------------------------------------------------------------
static void feed_axl(SimMortalityService &svc, double activity) {
	ServiceEvent e;
	e.event_source = ServiceIdentifier::AXL_SENSOR;
	e.event_type   = ServiceEventType::SERVICE_LOG_UPDATED;
	ServiceSensorData sd;
	sd.port[4] = activity;      // AXL_PORT_ACTIVITY (see mortality_service.cpp)
	e.event_data = sd;
	svc.notify_peer_event(e);
}

static void feed_thermistor(SimMortalityService &svc, double temp_c) {
	ServiceEvent e;
	e.event_source = ServiceIdentifier::THERMISTOR_SENSOR;
	e.event_type   = ServiceEventType::SERVICE_LOG_UPDATED;
	ServiceSensorData sd;
	sd.port[0] = temp_c;
	e.event_data = sd;
	svc.notify_peer_event(e);
}

static void feed_gnss(SimMortalityService &svc, bool valid, double lat, double lon, int speed_mm_s) {
	ServiceEvent e;
	e.event_source = ServiceIdentifier::GNSS_SENSOR;
	e.event_type   = ServiceEventType::SERVICE_LOG_UPDATED;
	GPSLogEntry g;
	memset(&g, 0, sizeof(g));
	g.info.valid  = valid ? 1 : 0;
	g.info.lat    = lat;
	g.info.lon    = lon;
	g.info.gSpeed = speed_mm_s;
	e.event_data = g;
	svc.notify_peer_event(e);
}

// ---------------------------------------------------------------------
static std::string out_path(const std::string &fname) {
	const char *d = std::getenv("RSPB_REPORT_DIR");
	std::string dir = d ? d : ".";
	if (!dir.empty() && dir.back() != '/') dir += "/";
	return dir + fname;
}

static const char *status_str(int s) {
	return s == 2 ? "CONFIRMED" : s == 1 ? "SUSPECTED" : "ALIVE";
}

// =====================================================================
// The mission — TPL5111 wake loop with real boot-counter/modulo gating
// =====================================================================
static SimResult run_mission(SimConfig cfg, SimMortalityService &svc, FsLog &log,
                             std::vector<Rec> &recs, std::vector<DayS> &days) {
	SimResult R;
	R.cfg = cfg;
	if (cfg.deploy_days < 1) cfg.deploy_days = 1;
	if (cfg.wake_period_s < 60) cfg.wake_period_s = 60;
	if (cfg.normal_modulo < 2) cfg.normal_modulo = 2;

	// --- Configure the MortalityService via the config store ---
	configuration_store->write_param(ParamID::MORTALITY_ENABLE, true);
	configuration_store->write_param(ParamID::MORTALITY_ACTIVITY_THRESH, cfg.activity_thresh);
	configuration_store->write_param(ParamID::MORTALITY_TEMP_THRESH, cfg.temp_thresh_c);
	configuration_store->write_param(ParamID::MORTALITY_GPS_DISTANCE_THRESH, cfg.gps_dist_thresh_m);
	configuration_store->write_param(ParamID::MORTALITY_CONFIRM_DAYS, cfg.confirm_days);
	configuration_store->write_param(ParamID::MORTALITY_DUTY_CYCLE_MODULO, cfg.duty_modulo);
	unsigned int zero = 0u;
	configuration_store->write_param(ParamID::MORTALITY_ORIGINAL_MODULO, zero);
	configuration_store->write_param(ParamID::BOOT_COUNTER_MODULO, cfg.normal_modulo);

	std::mt19937 gen(cfg.seed);
	auto uni = [&](double a, double b) {
		return std::uniform_real_distribution<double>(a, b)(gen);
	};

	// Per-day accumulators (forward-filled after the loop).
	days.assign(cfg.deploy_days, DayS{});
	std::vector<double> day_act(cfg.deploy_days, 0), day_temp(cfg.deploy_days, 0), day_moved(cfg.deploy_days, 0);
	for (int d = 0; d < cfg.deploy_days; d++) { days[d].day = d; days[d].batt_pct = 100.0; }

	std::time_t death_epoch = cfg.start_epoch + (std::time_t)cfg.death_day * 86400;

	double cur_lat = cfg.base_lat, cur_lon = cfg.base_lon;
	double death_lat = cur_lat, death_lon = cur_lon;
	bool   death_captured = false;
	double prev_rep_lat = cur_lat, prev_rep_lon = cur_lon;
	bool   have_prev_rep = false;

	double batt_used = 0.0, batt_used_noadapt = 0.0;
	unsigned boot_counter = 0;      // persisted BOOT_COUNTER (flash-backed on device)
	unsigned boot_counter_na = 0;   // counterfactual: modulo never adapts

	// Carried "current" state so days with no active wake forward-fill cleanly.
	unsigned cur_conf = 0, cur_consec = 0, cur_modulo = cfg.normal_modulo;
	int      cur_status = 0;

	R.total_wakes = (long)cfg.deploy_days * 86400 / cfg.wake_period_s;

	for (long w = 0; w < R.total_wakes; w++) {
		std::time_t t = cfg.start_epoch + (std::time_t)w * cfg.wake_period_s;
		rtc->settime(t);
		int day = (int)((t - cfg.start_epoch) / 86400);
		if (day >= cfg.deploy_days) day = cfg.deploy_days - 1;
		double hour = std::fmod((double)(t - cfg.start_epoch) / 3600.0, 24.0);

		// --- boot-counter increment + modulo gate (mirrors config_store.hpp) ---
		unsigned modulo = configuration_store->read_param<unsigned int>(ParamID::BOOT_COUNTER_MODULO);
		if (boot_counter > modulo + 1) boot_counter = 0; else boot_counter++;
		bool active;
		if (modulo < 2) active = true;
		else if (boot_counter % modulo == 0) { active = true; boot_counter = 0; }
		else active = false;

		// Counterfactual: mortality never touches the modulo (stays normal_modulo).
		unsigned nm = cfg.normal_modulo;
		if (boot_counter_na > nm + 1) boot_counter_na = 0; else boot_counter_na++;
		bool active_na = (nm < 2) ? true : (boot_counter_na % nm == 0);
		if (active_na && nm >= 2) boot_counter_na = 0;

		// Battery: every wake pays the TPL overhead; only active wakes pay the cycle.
		batt_used         += cfg.tpl_wake_mah;
		batt_used_noadapt += cfg.tpl_wake_mah;
		if (active)    batt_used         += cfg.active_session_mah;
		if (active_na) { batt_used_noadapt += cfg.active_session_mah; R.active_wakes_noadapt++; }

		double batt_pct = 100.0 * (1.0 - batt_used / cfg.capacity_mah);
		days[day].batt_pct = batt_pct;
		days[day].day = day;

		// --- Bird state model ---
		bool dead_phase;
		if (cfg.death_mode == DeathMode::NONE) dead_phase = false;
		else dead_phase = (day >= cfg.death_day) &&
		                  !(cfg.recover_day >= 0 && day >= cfg.recover_day);
		bool alive = !dead_phase;
		days[day].alive_model = alive;

		if (!active) {
			// Device powers straight back down: carry state forward for the report.
			days[day].end_confidence = cur_conf;
			days[day].end_consecutive = cur_consec;
			days[day].end_status = cur_status;
			days[day].end_modulo = cur_modulo;
			continue;
		}

		// --- Active wake: simulated power-cut + reboot -> reload persisted state ---
		svc.sim_wake();

		// --- Sensor model ---
		double activity, body_temp;
		bool daytime = (hour >= 6.0 && hour < 20.0);
		if (alive) {
			if (cfg.roosting) {
				activity = daytime ? uni(8, 30) : uni(3, 14);   // quiescent, sometimes < thresh
			} else {
				activity = daytime ? uni(45, 185) : uni(14, 46);
				if (uni(0, 1) < 0.11) activity = uni(3, 8);      // rare deep-rest bout
			}
			body_temp = uni(39.4, 41.4);                          // warm -> temp_score 0
		} else {
			activity = uni(0, 4);                                 // carcass: still
			double dt_since_death_h = (double)(t - death_epoch) / 3600.0;
			if (dt_since_death_h < 0) dt_since_death_h = 0;
			body_temp = cfg.ambient_temp_c +
			            (cfg.alive_temp_c - cfg.ambient_temp_c) *
			                std::exp(-dt_since_death_h / cfg.temp_decay_tau_h);
		}

		// --- Position / GPS model ---
		if (alive) {
			death_captured = false;  // re-arm capture for a possible future death
			double step = cfg.roosting ? uni(0, 25) : uni(80, 2500);
			double ang  = uni(0, 2 * PI);
			cur_lat += (step * std::cos(ang)) / 111320.0;
			cur_lon += (step * std::sin(ang)) / (111320.0 * std::cos(cur_lat * PI / 180.0));
		} else if (!death_captured) {
			death_lat = cur_lat; death_lon = cur_lon; death_captured = true;
		}

		bool gps_valid;
		if (dead_phase && cfg.death_mode == DeathMode::CANOPY) gps_valid = false;
		else gps_valid = uni(0, 1) < (alive ? 0.85 : 0.90);

		double rep_lat, rep_lon;
		int gps_speed;
		if (alive) {
			rep_lat = cur_lat + uni(-3, 3) / 111320.0;
			rep_lon = cur_lon + uni(-3, 3) / (111320.0 * std::cos(cur_lat * PI / 180.0));
			gps_speed = cfg.roosting ? (int)uni(0, 80) : (int)uni(300, 3500);
		} else {
			rep_lat = death_lat + uni(-6, 6) / 111320.0;
			rep_lon = death_lon + uni(-6, 6) / (111320.0 * std::cos(death_lat * PI / 180.0));
			gps_speed = (int)uni(0, 40);
		}

		double moved_m = 0.0;
		if (have_prev_rep && gps_valid)
			moved_m = haversine_distance(prev_rep_lon, prev_rep_lat, rep_lon, rep_lat) * 1000.0;

		// --- Feed the peer events (GNSS last on the normal path -> full collection) ---
		feed_axl(svc, activity);
		feed_thermistor(svc, body_temp);
		feed_gnss(svc, gps_valid, rep_lat, rep_lon, gps_speed);

		if (gps_valid) { prev_rep_lat = rep_lat; prev_rep_lon = rep_lon; have_prev_rep = true; }
		else days[day].nofix++;

		// --- Read back what the service computed ---
		cur_conf   = svc.get_confidence();
		cur_status = (int)svc.get_status();
		cur_modulo = configuration_store->read_param<unsigned int>(ParamID::BOOT_COUNTER_MODULO);
		if (log.num_entries() > 0) {
			MortalityLogEntry le;
			log.read(&le, log.num_entries() - 1);
			cur_consec = le.info.consecutive_days;
		}

		// --- Aggregate ---
		Rec r;
		r.epoch = t; r.day = day; r.alive_model = alive; r.gps_valid = gps_valid;
		r.activity = activity; r.body_temp = body_temp; r.moved_m = moved_m; r.gps_speed = gps_speed;
		r.confidence = cur_conf; r.consecutive_days = cur_consec; r.status = cur_status;
		r.modulo = cur_modulo; r.batt_pct = batt_pct;
		recs.push_back(r);

		day_act[day] += activity; day_temp[day] += body_temp; day_moved[day] += moved_m;
		days[day].active_wakes++;
		days[day].had_active = true;
		days[day].end_confidence = cur_conf;
		days[day].end_consecutive = cur_consec;
		days[day].end_status = cur_status;
		days[day].end_modulo = cur_modulo;
		R.active_wakes++;

		if (cur_conf > R.max_conf_overall) R.max_conf_overall = cur_conf;
		if (alive) {
			if (cur_conf > R.alive_max_conf) R.alive_max_conf = cur_conf;
			if (cur_status > R.alive_max_status) R.alive_max_status = cur_status;
			if (cfg.death_mode == DeathMode::NONE || day < cfg.death_day)
				R.modulo_while_alive = cur_modulo;
		}
		if (cur_status == 2) { R.ever_confirmed = true; if (R.detection_day < 0) R.detection_day = day; }
		if (R.ever_confirmed && R.recovered_day < 0 && cur_status != 2) R.recovered_day = day;
	}

	// Forward-fill per-day averages + carried state for the charts/table.
	double last_act = 0, last_temp = 0, last_moved = 0;
	unsigned last_conf = 0, last_consec = 0, last_modulo = cfg.normal_modulo;
	int last_status = 0;
	for (int d = 0; d < cfg.deploy_days; d++) {
		if (days[d].active_wakes > 0) {
			days[d].avg_activity = day_act[d] / days[d].active_wakes;
			days[d].avg_temp     = day_temp[d] / days[d].active_wakes;
			days[d].avg_moved    = day_moved[d] / days[d].active_wakes;
			last_act = days[d].avg_activity; last_temp = days[d].avg_temp; last_moved = days[d].avg_moved;
			last_conf = days[d].end_confidence; last_consec = days[d].end_consecutive;
			last_status = days[d].end_status; last_modulo = days[d].end_modulo;
		} else {
			days[d].avg_activity = last_act; days[d].avg_temp = last_temp; days[d].avg_moved = last_moved;
			days[d].end_confidence = last_conf; days[d].end_consecutive = last_consec;
			days[d].end_status = last_status; days[d].end_modulo = last_modulo;
		}
	}

	if (!recs.empty()) {
		const Rec &fin = recs.back();
		R.final_confidence  = fin.confidence;
		R.final_status      = fin.status;
		R.final_consecutive = fin.consecutive_days;
		R.final_modulo      = fin.modulo;
	} else {
		R.final_modulo = configuration_store->read_param<unsigned int>(ParamID::BOOT_COUNTER_MODULO);
	}
	R.batt_pct         = 100.0 * (1.0 - batt_used / cfg.capacity_mah);
	R.batt_pct_noadapt = 100.0 * (1.0 - batt_used_noadapt / cfg.capacity_mah);
	R.devlog_entries   = (int)log.num_entries();
	return R;
}

// =====================================================================
// Artefact writers (all guard against a failed open)
// =====================================================================
static void write_csv(const std::vector<Rec> &recs) {
	std::ofstream f(out_path("rspb_mortality.csv"));
	if (!f.is_open()) { printf("  WARN: cannot open rspb_mortality.csv\n"); return; }
	f << "epoch,day,model,gps_valid,activity,body_temp_c,moved_m,gps_speed_mm_s,"
	     "confidence,consecutive_days,status,boot_modulo,battery_pct\r\n";
	for (const auto &r : recs) {
		f << r.epoch << ',' << r.day << ','
		  << (r.alive_model ? "ALIVE" : "DEAD") << ',' << (r.gps_valid ? 1 : 0) << ','
		  << std::fixed << std::setprecision(1) << r.activity << ','
		  << std::setprecision(2) << r.body_temp << ','
		  << std::setprecision(1) << r.moved_m << ',' << r.gps_speed << ','
		  << r.confidence << ',' << r.consecutive_days << ',' << status_str(r.status) << ','
		  << r.modulo << ',' << std::setprecision(2) << r.batt_pct << "\r\n";
	}
	printf("  wrote %s\n", out_path("rspb_mortality.csv").c_str());
}

static void write_missionlog(const std::vector<Rec> &recs, const SimConfig &cfg) {
	std::ofstream f(out_path("rspb_mission.log"));
	if (!f.is_open()) { printf("  WARN: cannot open rspb_mission.log\n"); return; }
	f << "RSPB bird-tracker mortality mission log\n";
	f << "deployment=" << cfg.deploy_days << "d  death_day=" << cfg.death_day
	  << "  confirm_days=" << cfg.confirm_days << "\n";
	f << "----------------------------------------------------------------------\n";
	for (const auto &r : recs) {
		std::time_t t = r.epoch;
		std::tm *tm = std::gmtime(&t);
		char ts[32] = "0000-00-00 00:00";
		if (tm) std::strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M", tm);
		char line[256];
		snprintf(line, sizeof(line),
		         "[%s] d%03d %-5s act=%5.1f temp=%5.2fC moved=%7.1fm gps=%-5s "
		         "-> conf=%3u%% days=%2u %-9s mod=%2u batt=%5.1f%%\n",
		         ts, r.day, r.alive_model ? "ALIVE" : "DEAD",
		         r.activity, r.body_temp, r.moved_m, r.gps_valid ? "fix" : "nofix",
		         r.confidence, r.consecutive_days, status_str(r.status),
		         r.modulo, r.batt_pct);
		f << line;
	}
	printf("  wrote %s\n", out_path("rspb_mission.log").c_str());
}

static void export_devlog(LFSFileSystem *fs, const char *logname) {
	FsLog log(fs, logname, 512 * 1024);
	log.create();  // re-open the persisted on-device log file (entries survive the object)
	MortalityLogFormatter fmt;
	std::ofstream f(out_path("rspb_mortality_log.csv"));
	if (!f.is_open()) { printf("  WARN: cannot open rspb_mortality_log.csv\n"); return; }
	f << fmt.header();
	unsigned n = log.num_entries();
	for (unsigned i = 0; i < n; i++) {
		MortalityLogEntry e;
		log.read(&e, i);
		f << fmt.log_entry(*reinterpret_cast<LogEntry *>(&e));
	}
	printf("  wrote %s (%u on-device entries)\n", out_path("rspb_mortality_log.csv").c_str(), n);
}

// --- tiny inline-SVG helpers (self-contained, no external assets) ---
static std::string svg_points(const std::vector<double> &v, double vmin, double vmax,
                              double W, double H) {
	std::ostringstream o;
	double span = (vmax - vmin) > 1e-9 ? (vmax - vmin) : 1.0;
	for (size_t i = 0; i < v.size(); i++) {
		double x = v.size() > 1 ? (W * i) / (v.size() - 1) : 0.0;
		double y = H - H * (v[i] - vmin) / span;
		if (y < 0) y = 0;
		if (y > H) y = H;
		o << std::fixed << std::setprecision(1) << x << ',' << y << ' ';
	}
	return o.str();
}

static std::string mini_chart(const std::string &title, const std::vector<double> &series,
                              double vmin, double vmax, const char *color,
                              int death_day, int detection_day, int ndays,
                              const std::string &unit) {
	const double W = 520, H = 130;
	std::ostringstream o;
	o << "<div class=\"panel\"><h3>" << title << "</h3>";
	o << "<svg viewBox=\"0 0 " << (W + 50) << " " << (H + 28)
	  << "\" preserveAspectRatio=\"xMidYMid meet\" class=\"chart\">";
	o << "<rect x=\"40\" y=\"4\" width=\"" << W << "\" height=\"" << H
	  << "\" fill=\"rgba(255,255,255,0.03)\" stroke=\"rgba(255,255,255,0.12)\"/>";
	o << "<text x=\"36\" y=\"12\" class=\"ylab\">" << std::fixed << std::setprecision(0) << vmax << "</text>";
	o << "<text x=\"36\" y=\"" << (H + 4) << "\" class=\"ylab\">" << vmin << "</text>";
	o << "<text x=\"4\" y=\"" << (H / 2) << "\" class=\"unit\">" << unit << "</text>";
	auto xat = [&](int d) { return 40.0 + (ndays > 1 ? (W * d) / (ndays - 1) : 0.0); };
	if (death_day >= 0)
		o << "<line x1=\"" << xat(death_day) << "\" y1=\"4\" x2=\"" << xat(death_day)
		  << "\" y2=\"" << (H + 4) << "\" stroke=\"#ff4757\" stroke-dasharray=\"4 3\" stroke-width=\"1.3\"/>";
	if (detection_day >= 0)
		o << "<line x1=\"" << xat(detection_day) << "\" y1=\"4\" x2=\"" << xat(detection_day)
		  << "\" y2=\"" << (H + 4) << "\" stroke=\"#00ff88\" stroke-dasharray=\"4 3\" stroke-width=\"1.3\"/>";
	o << "<g transform=\"translate(40,4)\"><polyline fill=\"none\" stroke=\"" << color
	  << "\" stroke-width=\"1.8\" points=\"" << svg_points(series, vmin, vmax, W, H) << "\"/></g>";
	o << "<text x=\"40\" y=\"" << (H + 22) << "\" class=\"xlab\">day 0</text>";
	o << "<text x=\"" << (W - 10) << "\" y=\"" << (H + 22) << "\" class=\"xlab\">day " << (ndays - 1) << "</text>";
	o << "</svg></div>";
	return o.str();
}

static void write_html(const SimResult &R, const std::vector<DayS> &days) {
	const SimConfig &cfg = R.cfg;
	std::ofstream html(out_path("rspb_mortality_report.html"));
	if (!html.is_open()) { printf("  WARN: cannot open rspb_mortality_report.html\n"); return; }

	time_t now = time(nullptr);
	char time_str[64] = "--------- --:--:--";
	std::tm *lt = localtime(&now);
	if (lt) strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", lt);

	int latency = (R.detection_day >= 0) ? (R.detection_day - cfg.death_day) : -1;
	bool detected = (R.detection_day >= 0);
	bool no_false_positive = (R.alive_max_status < 2);

	std::vector<double> conf, act, temp, moved, batt, consec;
	for (const auto &d : days) {
		conf.push_back(d.end_confidence);
		act.push_back(d.avg_activity);
		temp.push_back(d.avg_temp);
		moved.push_back(d.avg_moved);
		batt.push_back(d.batt_pct);
		consec.push_back(d.end_consecutive);
	}
	int nd = (int)days.size();

	html << R"(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>RSPB Mortality Simulation Report</title>
<style>
  * { margin:0; padding:0; box-sizing:border-box; }
  body { font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;
         background:linear-gradient(135deg,#141e30 0%,#243b55 100%); color:#e8eef5;
         padding:24px; min-height:100vh; }
  .container { max-width:1180px; margin:0 auto; }
  h1 { text-align:center; color:#7fd7ff; text-shadow:0 0 18px rgba(127,215,255,.4); margin-bottom:6px; }
  .sub { text-align:center; opacity:.65; margin-bottom:24px; font-size:13px; }
  .verdict { text-align:center; font-size:22px; font-weight:700; padding:18px; border-radius:14px;
             margin-bottom:26px; }
  .verdict.ok { background:rgba(0,255,136,.12); color:#00ff88; border:1px solid rgba(0,255,136,.3); }
  .verdict.bad{ background:rgba(255,71,87,.12); color:#ff4757; border:1px solid rgba(255,71,87,.3); }
  .summary { display:flex; flex-wrap:wrap; gap:16px; justify-content:center; margin-bottom:30px; }
  .card { background:rgba(255,255,255,.06); border:1px solid rgba(255,255,255,.08);
          border-radius:14px; padding:18px 26px; text-align:center; min-width:150px; }
  .card .v { font-size:34px; font-weight:700; }
  .card .l { font-size:12px; text-transform:uppercase; opacity:.6; margin-top:4px; letter-spacing:.5px; }
  .c-blue .v{color:#7fd7ff;} .c-green .v{color:#00ff88;} .c-red .v{color:#ff4757;}
  .c-amber .v{color:#ffb648;} .c-violet .v{color:#c39bff;}
  .charts { display:flex; flex-wrap:wrap; gap:18px; margin-bottom:30px; }
  .panel { background:rgba(255,255,255,.04); border:1px solid rgba(255,255,255,.07);
           border-radius:14px; padding:14px 16px; flex:1 1 480px; }
  .panel h3 { font-size:14px; margin-bottom:8px; color:#bcd; font-weight:600; }
  .chart { width:100%; height:auto; }
  .ylab{ fill:#9ab; font-size:9px; text-anchor:end; }
  .xlab{ fill:#9ab; font-size:9px; }
  .unit{ fill:#9ab; font-size:9px; }
  .legend { text-align:center; font-size:12px; opacity:.75; margin:-14px 0 24px; }
  .legend span{ margin:0 10px; }
  .dot{ display:inline-block; width:10px; height:10px; border-radius:2px; vertical-align:middle; margin-right:4px; }
  table { width:100%; border-collapse:collapse; background:rgba(255,255,255,.04);
          border-radius:12px; overflow:hidden; font-size:13px; }
  th { background:rgba(127,215,255,.16); padding:10px; text-align:right; position:sticky; top:0; }
  th:first-child,td:first-child{ text-align:left; }
  td { padding:7px 10px; text-align:right; border-bottom:1px solid rgba(255,255,255,.05); }
  tr.dead td { background:rgba(255,71,87,.05); }
  tr.confirmed td { background:rgba(255,71,87,.13); }
  .badge{ padding:2px 9px; border-radius:11px; font-size:11px; font-weight:700; }
  .b-alive{ background:rgba(0,255,136,.16); color:#00ff88; }
  .b-susp { background:rgba(255,182,72,.16); color:#ffb648; }
  .b-conf { background:rgba(255,71,87,.18); color:#ff4757; }
  .tablewrap{ max-height:520px; overflow:auto; border-radius:12px; margin-bottom:26px; }
  .params{ background:rgba(255,255,255,.04); border-radius:12px; padding:16px 20px; font-size:13px;
           line-height:1.9; margin-bottom:26px; }
  .params code{ background:rgba(127,215,255,.12); padding:1px 7px; border-radius:5px; color:#7fd7ff; }
  .footer{ text-align:center; opacity:.5; font-size:12px; margin-top:20px; }
  h2{ color:#9fd; font-size:17px; margin:8px 0 14px; border-left:3px solid #7fd7ff; padding-left:10px; }
</style>
</head>
<body>
<div class="container">
  <h1>&#x1F426; RSPB Bird-Tracker &mdash; Long-Term Mortality Simulation</h1>
  <div class="sub">)" << time_str << R"( &middot; drives the firmware MortalityService over a )"
	     << cfg.deploy_days << R"(-day deployment &middot; )" << R.total_wakes
	     << R"( TPL5111 wakes, )" << R.active_wakes << R"( active</div>
)";

	if (detected && no_false_positive) {
		html << "  <div class=\"verdict ok\">&#x2705; Mortality CONFIRMED " << latency
		     << " days after death (day " << R.detection_day << ") &mdash; no false positive while alive</div>\n";
	} else if (detected && !no_false_positive) {
		html << "  <div class=\"verdict bad\">&#x26A0; Mortality confirmed, but a false positive occurred while the bird was alive</div>\n";
	} else {
		html << "  <div class=\"verdict bad\">&#x274C; Mortality was never CONFIRMED over the deployment</div>\n";
	}

	html << "  <div class=\"summary\">\n";
	auto card = [&](const char *cls, const std::string &v, const char *l) {
		html << "    <div class=\"card " << cls << "\"><div class=\"v\">" << v
		     << "</div><div class=\"l\">" << l << "</div></div>\n";
	};
	card("c-blue",  std::to_string(cfg.deploy_days) + "d", "Deployment");
	card("c-red",   "day " + std::to_string(cfg.death_day), "Bird died");
	card("c-green", detected ? "day " + std::to_string(R.detection_day) : "&mdash;", "Confirmed");
	card("c-amber", detected ? std::to_string(latency) + "d" : "&mdash;", "Detection latency");
	card("c-violet", std::to_string(R.final_confidence) + "%", "Final confidence");
	{
		std::ostringstream b; b << std::fixed << std::setprecision(0) << R.batt_pct << "%";
		card("c-green", b.str(), "Battery left");
	}
	html << "  </div>\n";

	html << "  <h2>Confidence &amp; mortality state</h2>\n";
	html << "  <div class=\"legend\">"
	        "<span><span class=\"dot\" style=\"background:#ff4757\"></span>death</span>"
	        "<span><span class=\"dot\" style=\"background:#00ff88\"></span>confirmed</span>"
	        "<span>dashed 50% = SUSPECTED gate &middot; 80% = day-counter gate</span></div>\n";

	{
		const double W = 1080, H = 220;
		auto xat = [&](int d) { return 46.0 + (nd > 1 ? (W * d) / (nd - 1) : 0.0); };
		auto yat = [&](double val) { return 6.0 + H - H * val / 100.0; };
		html << "  <div class=\"panel\" style=\"flex:1 1 100%\"><h3>Mortality confidence (EMA) &amp; consecutive-day counter</h3>";
		html << "<svg viewBox=\"0 0 " << (W + 60) << " " << (H + 30)
		     << "\" preserveAspectRatio=\"xMidYMid meet\" class=\"chart\">";
		html << "<rect x=\"46\" y=\"6\" width=\"" << W << "\" height=\"" << H
		     << "\" fill=\"rgba(255,255,255,0.03)\" stroke=\"rgba(255,255,255,0.12)\"/>";
		for (int g = 0; g <= 100; g += 20) {
			html << "<line x1=\"46\" y1=\"" << yat(g) << "\" x2=\"" << (46 + W) << "\" y2=\"" << yat(g)
			     << "\" stroke=\"rgba(255,255,255,0.06)\"/>";
			html << "<text x=\"42\" y=\"" << (yat(g) + 3) << "\" class=\"ylab\">" << g << "</text>";
		}
		html << "<line x1=\"46\" y1=\"" << yat(50) << "\" x2=\"" << (46 + W) << "\" y2=\"" << yat(50)
		     << "\" stroke=\"#ffb648\" stroke-dasharray=\"5 4\" stroke-width=\"1\"/>";
		html << "<line x1=\"46\" y1=\"" << yat(80) << "\" x2=\"" << (46 + W) << "\" y2=\"" << yat(80)
		     << "\" stroke=\"#ff8ba0\" stroke-dasharray=\"5 4\" stroke-width=\"1\"/>";
		html << "<line x1=\"" << xat(cfg.death_day) << "\" y1=\"6\" x2=\"" << xat(cfg.death_day)
		     << "\" y2=\"" << (6 + H) << "\" stroke=\"#ff4757\" stroke-dasharray=\"4 3\" stroke-width=\"1.5\"/>";
		if (R.detection_day >= 0)
			html << "<line x1=\"" << xat(R.detection_day) << "\" y1=\"6\" x2=\"" << xat(R.detection_day)
			     << "\" y2=\"" << (6 + H) << "\" stroke=\"#00ff88\" stroke-dasharray=\"4 3\" stroke-width=\"1.5\"/>";
		double cmax = std::max(2.0 * cfg.confirm_days, 4.0);
		std::ostringstream cpts;
		for (int i = 0; i < nd; i++) {
			double y = 6.0 + H - H * std::min((double)consec[i], cmax) / cmax;
			cpts << std::fixed << std::setprecision(1) << xat(i) << ',' << y << ' ';
		}
		html << "<polyline fill=\"none\" stroke=\"#c39bff\" stroke-width=\"1.4\" opacity=\"0.8\" points=\""
		     << cpts.str() << "\"/>";
		std::ostringstream fpts;
		for (int i = 0; i < nd; i++) fpts << std::fixed << std::setprecision(1) << xat(i) << ',' << yat(conf[i]) << ' ';
		html << "<polyline fill=\"none\" stroke=\"#7fd7ff\" stroke-width=\"2.2\" points=\"" << fpts.str() << "\"/>";
		html << "<text x=\"46\" y=\"" << (H + 24) << "\" class=\"xlab\">day 0</text>";
		html << "<text x=\"" << (46 + W - 40) << "\" y=\"" << (H + 24) << "\" class=\"xlab\">day " << (nd - 1) << "</text>";
		html << "</svg>";
		html << "<div class=\"legend\" style=\"margin:6px 0 0\">"
		        "<span><span class=\"dot\" style=\"background:#7fd7ff\"></span>confidence %</span>"
		        "<span><span class=\"dot\" style=\"background:#c39bff\"></span>consecutive days (0.."
		     << (int)cmax << ")</span></div>";
		html << "</div>\n";
	}

	double act_max = 0, temp_max = 0, moved_max = 0;
	for (const auto &d : days) {
		act_max = std::max(act_max, d.avg_activity);
		temp_max = std::max(temp_max, d.avg_temp);
		moved_max = std::max(moved_max, d.avg_moved);
	}
	html << "  <h2>Underlying signals</h2>\n  <div class=\"charts\">\n";
	html << mini_chart("AXL activity (avg/day)", act, 0, std::max(act_max, 50.0), "#00d4ff",
	                   cfg.death_day, R.detection_day, nd, "cnt");
	html << mini_chart("Body temperature (avg/day)", temp, 0, std::max(temp_max, 42.0), "#ffb648",
	                   cfg.death_day, R.detection_day, nd, "\xC2\xB0""C");
	html << mini_chart("Displacement (avg/day)", moved, 0, std::max(moved_max, 100.0), "#c39bff",
	                   cfg.death_day, R.detection_day, nd, "m");
	html << mini_chart("Battery remaining", batt, 0, 100, "#00ff88",
	                   cfg.death_day, R.detection_day, nd, "%");
	html << "  </div>\n";

	html << "  <h2>Mission &amp; MortalityService parameters</h2>\n  <div class=\"params\">";
	html << "Board profile: <code>EXTERNAL_WAKEUP</code> (RSPB / TPL5111) &middot; wake period <code>"
	     << cfg.wake_period_s << " s</code> &middot; active-wake cadence set by the firmware boot-counter modulo<br>";
	html << "MTP01 MORTALITY_ENABLE=<code>1</code> &middot; "
	        "MTP02 ACTIVITY_THRESH=<code>" << cfg.activity_thresh << "</code> &middot; "
	        "MTP03 TEMP_THRESH=<code>" << std::fixed << std::setprecision(1) << cfg.temp_thresh_c << "&deg;C</code> &middot; "
	        "MTP04 GPS_DIST_THRESH=<code>" << cfg.gps_dist_thresh_m << " m</code><br>";
	html << "MTP05 CONFIRM_DAYS=<code>" << cfg.confirm_days << "</code> &middot; "
	        "MTP06 DUTY_CYCLE_MODULO=<code>" << cfg.duty_modulo << "</code> &middot; "
	        "PWP03 BOOT_COUNTER_MODULO (live)=<code>" << cfg.normal_modulo << "</code><br>";
	html << "Duty-cycle adaptation: modulo while alive=<code>" << R.modulo_while_alive
	     << "</code> &rarr; after CONFIRMED=<code>" << R.final_modulo << "</code> &nbsp;(active wakes: "
	     << R.active_wakes << " with adaptation vs " << R.active_wakes_noadapt << " without)<br>";
	html << "Battery @ end: with adaptation=<code>" << std::setprecision(1) << R.batt_pct
	     << "%</code> vs. without=<code>" << R.batt_pct_noadapt << "%</code> &nbsp;(&#x1F50B; fewer active wakes at the dead-bird modulo)<br>";
	html << "On-device MortalityService log: <code>" << R.devlog_entries << "</code> entries persisted to FsLog";
	html << "</div>\n";

	html << "  <h2>Per-day summary</h2>\n  <div class=\"tablewrap\"><table>\n";
	html << "    <thead><tr><th>Day</th><th>Model</th><th>Act</th><th>Temp &deg;C</th><th>Moved m</th>"
	        "<th>No-fix</th><th>Wakes</th><th>Conf %</th><th>Days</th><th>State</th><th>Modulo</th><th>Batt %</th></tr></thead>\n    <tbody>\n";
	for (const auto &d : days) {
		const char *rowcls = d.end_status == 2 ? "confirmed" : (!d.alive_model ? "dead" : "");
		const char *bcls = d.end_status == 2 ? "b-conf" : d.end_status == 1 ? "b-susp" : "b-alive";
		html << "    <tr class=\"" << rowcls << "\"><td>" << d.day << "</td><td>"
		     << (d.alive_model ? "alive" : "dead") << "</td><td>"
		     << std::fixed << std::setprecision(0) << d.avg_activity << "</td><td>"
		     << std::setprecision(1) << d.avg_temp << "</td><td>"
		     << std::setprecision(0) << d.avg_moved << "</td><td>"
		     << d.nofix << "</td><td>" << d.active_wakes << "</td><td>" << d.end_confidence << "</td><td>"
		     << d.end_consecutive << "</td><td><span class=\"badge " << bcls << "\">"
		     << status_str(d.end_status) << "</span></td><td>" << d.end_modulo << "</td><td>"
		     << std::setprecision(1) << d.batt_pct << "</td></tr>\n";
	}
	html << "    </tbody>\n  </table></div>\n";

	html << "  <div class=\"footer\">Generated by the LinkIt V4 RSPB Mortality Simulation &middot; "
	        "firmware MortalityService (core/services/mortality_service.cpp)<br>"
	        "Accessible artefacts: rspb_mortality_report.html &middot; rspb_mortality.csv &middot; "
	        "rspb_mortality_log.csv &middot; rspb_mission.log</div>\n";
	html << "</div>\n</body>\n</html>\n";
	html.close();
	printf("  wrote %s\n", out_path("rspb_mortality_report.html").c_str());
}

// ---------------------------------------------------------------------
// Scenario runner — creates a fresh FsLog + service, runs the mission.
// ---------------------------------------------------------------------
static SimResult run_scenario(SimConfig cfg, LFSFileSystem *fs, const char *logname,
                              std::vector<Rec> *out_recs = nullptr,
                              std::vector<DayS> *out_days = nullptr) {
	FsLog log(fs, logname, 512 * 1024);
	log.create();
	log.truncate();  // fresh log even if this filename was reused earlier in the same test
	SimMortalityService svc(&log);
	std::vector<Rec> recs;
	std::vector<DayS> days;
	SimResult R = run_mission(cfg, svc, log, recs, days);
	if (out_recs) *out_recs = recs;
	if (out_days) *out_days = days;
	return R;
}

}  // namespace

// =====================================================================
// Test harness
// =====================================================================
TEST_GROUP(RSPBSimulation) {
	FakeConfigurationStore *fake_cs = nullptr;
	FakeRTC   *fake_rtc_obj = nullptr;
	FakeTimer *fake_timer_obj = nullptr;
	RamFlash  *ram_flash = nullptr;
	LFSFileSystem *ram_fs = nullptr;

	void setup() {
		fake_cs = new FakeConfigurationStore;
		configuration_store = fake_cs;
		configuration_store->init();

		fake_rtc_obj = new FakeRTC;
		rtc = fake_rtc_obj;
		fake_timer_obj = new FakeTimer;
		system_timer = fake_timer_obj;
		system_scheduler = new Scheduler(system_timer);
		fake_timer_obj->start();

		ram_flash = new RamFlash(256, 64 * 1024, 256);
		ram_fs = new LFSFileSystem(ram_flash);
		ram_fs->format();
		ram_fs->mount();
		main_filesystem = ram_fs;
	}

	void teardown() {
		ram_fs->umount();
		delete ram_fs;
		delete ram_flash;
		main_filesystem = nullptr;
		delete system_scheduler;
		delete fake_timer_obj;
		delete fake_rtc_obj;
		delete fake_cs;
		system_scheduler = nullptr;
		system_timer = nullptr;
		rtc = nullptr;
		configuration_store = nullptr;
	}
};

// --- Reference scenario: death in the open, GPS fixes continue on the carcass.
//     Also the artefact-producing run (HTML/CSV/logs).
TEST(RSPBSimulation, OpenFieldDeath) {
	SimConfig cfg;  // defaults: OPEN, death_day 60, deploy 120
	std::vector<Rec> recs;
	std::vector<DayS> days;
	SimResult R = run_scenario(cfg, ram_fs, "MORTALITY", &recs, &days);

	printf("\n[rspb-sim] OPEN: %ld wakes (%ld active) | death=day%d detection=day%d "
	       "final=%s conf=%u%% modulo %u->%u | batt %.1f%% (no-adapt %.1f%%) | %d log entries\n",
	       R.total_wakes, R.active_wakes, cfg.death_day, R.detection_day,
	       status_str(R.final_status), R.final_confidence, R.modulo_while_alive, R.final_modulo,
	       R.batt_pct, R.batt_pct_noadapt, R.devlog_entries);

	write_html(R, days);
	write_csv(recs);
	export_devlog(ram_fs, "MORTALITY");
	write_missionlog(recs, cfg);

	CHECK_TEXT(R.alive_max_status < 2, "OPEN: falsely CONFIRMED while alive");
	CHECK_TEXT(R.alive_max_conf < 50, "OPEN: confidence entered SUSPECTED range while alive");
	CHECK_TEXT(R.detection_day >= 0, "OPEN: mortality never CONFIRMED");
	CHECK_TEXT(R.detection_day >= cfg.death_day, "OPEN: CONFIRMED before death");
	int latency = R.detection_day - cfg.death_day;
	CHECK_TEXT(latency >= (int)cfg.confirm_days, "OPEN: confirmed faster than confirm_days");
	CHECK_TEXT(latency <= (int)cfg.confirm_days + 8, "OPEN: detection latency too long");
	CHECK_EQUAL_TEXT(2, R.final_status, "OPEN: final status not CONFIRMED");
	CHECK_TEXT(R.final_confidence >= 80, "OPEN: final confidence below day-counter gate");
	CHECK_EQUAL_TEXT((int)cfg.normal_modulo, (int)R.modulo_while_alive, "OPEN: modulo changed while alive");
	CHECK_EQUAL_TEXT((int)cfg.duty_modulo, (int)R.final_modulo, "OPEN: modulo did not adapt after CONFIRMED");
	CHECK_TEXT(R.batt_pct > R.batt_pct_noadapt, "OPEN: duty-cycle adaptation did not save battery");
	CHECK_TEXT(R.active_wakes < R.active_wakes_noadapt, "OPEN: adaptation did not reduce active wakes");
	CHECK_TEXT(R.devlog_entries > 0, "OPEN: no state persisted to FsLog");
}

// --- Dense-canopy death: ZERO valid GPS fix after death. Exercises the H4
//     no-fix fallback. Before the H4 fix this could NOT reach CONFIRMED.
TEST(RSPBSimulation, CanopyDeathNoFix) {
	SimConfig cfg;
	cfg.death_mode = DeathMode::CANOPY;
	cfg.seed = 7u;
	SimResult R = run_scenario(cfg, ram_fs, "MORT_CAN");

	printf("[rspb-sim] CANOPY: detection=day%d final=%s conf=%u%% max_conf=%u\n",
	       R.detection_day, status_str(R.final_status), R.final_confidence, R.max_conf_overall);

	CHECK_TEXT(R.alive_max_status < 2, "CANOPY: falsely CONFIRMED while alive");
	CHECK_TEXT(R.detection_day >= cfg.death_day, "CANOPY: CONFIRMED before death");
	CHECK_TEXT(R.detection_day >= 0, "CANOPY: H4 fallback never reached CONFIRMED without any GPS fix");
	CHECK_EQUAL_TEXT(2, R.final_status, "CANOPY: final status not CONFIRMED");
	CHECK_TEXT(R.devlog_entries > 0, "CANOPY: no state persisted");
}

// --- Live bird for the whole deployment (365 days) — must NEVER confirm.
TEST(RSPBSimulation, LiveBirdNeverConfirmed) {
	SimConfig cfg;
	cfg.death_mode = DeathMode::NONE;
	cfg.deploy_days = 365;
	cfg.seed = 99u;
	SimResult R = run_scenario(cfg, ram_fs, "MORT_LIVE");

	printf("[rspb-sim] LIVE(365d): ever_confirmed=%d max_conf=%u final=%s final_modulo=%u\n",
	       (int)R.ever_confirmed, R.max_conf_overall, status_str(R.final_status), R.final_modulo);

	CHECK_TEXT(!R.ever_confirmed, "LIVE: a healthy bird was falsely CONFIRMED dead");
	CHECK_EQUAL_TEXT(0, R.final_status, "LIVE: final status not ALIVE");
	CHECK_EQUAL_TEXT((int)cfg.normal_modulo, (int)R.final_modulo, "LIVE: duty modulo changed for a live bird");
}

// --- Roosting live bird: stationary + intermittently low activity + WARM.
//     Multi-sensor fusion (warm body) must keep it out of CONFIRMED.
TEST(RSPBSimulation, RoostingLiveBirdNotConfirmed) {
	SimConfig cfg;
	cfg.death_mode = DeathMode::NONE;
	cfg.roosting = true;
	cfg.deploy_days = 120;
	cfg.seed = 123u;
	SimResult R = run_scenario(cfg, ram_fs, "MORT_ROOST");

	printf("[rspb-sim] ROOST: ever_confirmed=%d max_conf=%u final=%s\n",
	       (int)R.ever_confirmed, R.max_conf_overall, status_str(R.final_status));

	CHECK_TEXT(!R.ever_confirmed, "ROOST: a roosting live bird was falsely CONFIRMED dead");
	CHECK_EQUAL_TEXT((int)cfg.normal_modulo, (int)R.final_modulo, "ROOST: duty modulo changed for a live bird");
}

// --- False alarm then recovery: bird looks dead, CONFIRMS, then revives.
//     Status must walk back to ALIVE and the duty modulo must be RESTORED (H5).
TEST(RSPBSimulation, FalseAlarmRecovery) {
	SimConfig cfg;
	cfg.death_mode = DeathMode::OPEN;
	cfg.death_day = 40;
	cfg.recover_day = 70;
	cfg.deploy_days = 150;
	cfg.seed = 55u;
	SimResult R = run_scenario(cfg, ram_fs, "MORT_REC");

	printf("[rspb-sim] RECOVERY: detection=day%d recovered=day%d final=%s final_modulo=%u\n",
	       R.detection_day, R.recovered_day, status_str(R.final_status), R.final_modulo);

	CHECK_TEXT(R.ever_confirmed, "RECOVERY: never reached CONFIRMED before recovery");
	CHECK_TEXT(R.recovered_day > R.detection_day, "RECOVERY: status never left CONFIRMED after revival");
	CHECK_EQUAL_TEXT(0, R.final_status, "RECOVERY: final status not ALIVE after revival");
	CHECK_EQUAL_TEXT((int)cfg.normal_modulo, (int)R.final_modulo, "RECOVERY: duty modulo not restored (H5)");
}

// --- Multi-seed robustness: detection must be deterministic-ish across seeds.
TEST(RSPBSimulation, SeedRobustness) {
	const unsigned seeds[] = {1u, 2u, 3u, 7u, 42u, 100u, 271u, 999u, 2026u, 31337u, 8u, 77u};
	int runs = 0;
	for (unsigned s : seeds) {
		SimConfig cfg;
		cfg.seed = s;
		SimResult R = run_scenario(cfg, ram_fs, "MORT_SEED");
		int latency = (R.detection_day >= 0) ? R.detection_day - cfg.death_day : -1;
		char msg[128];
		snprintf(msg, sizeof(msg), "SEED %u: not confirmed / bad latency (detection=day%d)", s, R.detection_day);
		CHECK_TEXT(R.detection_day >= cfg.death_day, msg);
		CHECK_TEXT(latency >= (int)cfg.confirm_days && latency <= (int)cfg.confirm_days + 10, msg);
		snprintf(msg, sizeof(msg), "SEED %u: false positive while alive (max status %d)", s, R.alive_max_status);
		CHECK_TEXT(R.alive_max_status < 2, msg);
		CHECK_EQUAL_TEXT(2, R.final_status, msg);
		runs++;
	}
	printf("[rspb-sim] SEED robustness: %d seeds all detected, no false positives\n", runs);
}

// --- Duty-modulo corner cases: clamp (=1 -> 2) and disabled (=0 -> no change).
TEST(RSPBSimulation, DutyModuloCorners) {
	{
		// MTP06=1 must be clamped to 2 by the firmware.
		SimConfig cfg; cfg.duty_modulo = 1; cfg.seed = 5u;
		SimResult R = run_scenario(cfg, ram_fs, "MORT_M1");
		printf("[rspb-sim] DUTY=1: final_modulo=%u (expect clamp to 2)\n", R.final_modulo);
		CHECK_EQUAL_TEXT(2, R.final_modulo, "DUTY=1: not clamped to 2");
	}
	{
		// MTP06=0 disables adaptation: modulo must stay at the live value.
		SimConfig cfg; cfg.duty_modulo = 0; cfg.seed = 5u;
		SimResult R = run_scenario(cfg, ram_fs, "MORT_M0");
		printf("[rspb-sim] DUTY=0: ever_confirmed=%d final_modulo=%u (expect unchanged %u)\n",
		       (int)R.ever_confirmed, R.final_modulo, cfg.normal_modulo);
		CHECK_TEXT(R.ever_confirmed, "DUTY=0: mortality detection should still work");
		CHECK_EQUAL_TEXT((int)cfg.normal_modulo, (int)R.final_modulo, "DUTY=0: modulo changed despite disabled adaptation");
	}
}

// --- CONFIRM_DAYS extremes: fast (=1) and slow (=10).
TEST(RSPBSimulation, ConfirmDaysExtremes) {
	{
		SimConfig cfg; cfg.confirm_days = 1; cfg.seed = 11u;
		SimResult R = run_scenario(cfg, ram_fs, "MORT_CD1");
		int latency = R.detection_day - cfg.death_day;
		printf("[rspb-sim] CONFIRM=1: detection=day%d latency=%d\n", R.detection_day, latency);
		CHECK_TEXT(R.detection_day >= cfg.death_day, "CONFIRM=1: confirmed before death");
		CHECK_TEXT(latency >= 1, "CONFIRM=1: confirmed on death day (impossible)");
		CHECK_EQUAL_TEXT(2, R.final_status, "CONFIRM=1: not confirmed");
	}
	{
		SimConfig cfg; cfg.confirm_days = 10; cfg.deploy_days = 140; cfg.seed = 11u;
		SimResult R = run_scenario(cfg, ram_fs, "MORT_CD10");
		int latency = R.detection_day - cfg.death_day;
		printf("[rspb-sim] CONFIRM=10: detection=day%d latency=%d\n", R.detection_day, latency);
		CHECK_TEXT(R.alive_max_status < 2, "CONFIRM=10: false positive while alive");
		CHECK_TEXT(R.detection_day >= 0, "CONFIRM=10: never confirmed");
		CHECK_TEXT(latency >= 10, "CONFIRM=10: confirmed faster than 10 days");
	}
}

// --- Late death near the deployment boundary: confirmation legitimately may
//     not finish. Relaxed invariants only.
TEST(RSPBSimulation, LateDeathBoundary) {
	SimConfig cfg;
	cfg.death_day = cfg.deploy_days - 2;   // dies 2 days before the end
	cfg.seed = 17u;
	SimResult R = run_scenario(cfg, ram_fs, "MORT_LATE");

	printf("[rspb-sim] LATE: death=day%d detection=day%d final=%s conf=%u max_conf=%u\n",
	       cfg.death_day, R.detection_day, status_str(R.final_status), R.final_confidence, R.max_conf_overall);

	CHECK_TEXT(R.alive_max_status < 2, "LATE: false positive before death");
	CHECK_TEXT(R.final_status >= 1 || R.final_confidence > 0, "LATE: no post-death confidence trend");
	CHECK_TEXT(R.detection_day < 0 || R.detection_day >= cfg.death_day, "LATE: CONFIRMED before death");
}

// --- Death whose confirmation window straddles the New-Year boundary. Locks in
//     the monotonic eval_day_index() (the day counter must keep advancing across
//     Dec 31 -> Jan 1; bare tm_yday resets to 0 every Jan 1).
TEST(RSPBSimulation, CrossYearDeath) {
	SimConfig cfg;
	cfg.start_epoch = 1794700800;  // 2026-11-15 00:00:00 UTC
	cfg.death_day = 30;            // dies 2026-12-15; confirmation runs into 2027
	cfg.deploy_days = 90;          // ends 2027-02-13 — dead phase crosses New Year
	cfg.seed = 21u;
	SimResult R = run_scenario(cfg, ram_fs, "MORT_XYEAR");

	printf("[rspb-sim] CROSSYEAR: death=day%d detection=day%d final=%s conf=%u consec=%u\n",
	       cfg.death_day, R.detection_day, status_str(R.final_status), R.final_confidence, R.final_consecutive);

	CHECK_TEXT(R.alive_max_status < 2, "CROSSYEAR: false positive while alive");
	CHECK_TEXT(R.detection_day >= cfg.death_day, "CROSSYEAR: CONFIRMED before death");
	CHECK_TEXT(R.detection_day >= 0, "CROSSYEAR: never CONFIRMED across the year boundary");
	CHECK_EQUAL_TEXT(2, R.final_status, "CROSSYEAR: final status not CONFIRMED");
	CHECK_TEXT(R.final_consecutive >= cfg.confirm_days, "CROSSYEAR: day counter did not advance across New Year");
}
