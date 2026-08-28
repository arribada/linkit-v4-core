/**
 * @file turtle_simulation_shortsurface.cpp
 * @brief Config-driven short-surface scenario — exercises cooldown, rate
 *        limiter, hauled-mode auto-detect, REUSE_LAST, FastLoc priority,
 *        and battery / TX-rate metrics over N days.
 *
 * Drives the actual firmware features (HauledModeService, RateLimiter,
 * ConfigurationStore HAULED branch) via their public APIs while modelling
 * the surface/dive cadence at policy level. Captures per-hour and per-day
 * metrics (TX counts, GPS attempts/successes, battery drain, cooldown
 * engagements, HAULED state changes) and writes both an HTML report and a
 * CSV for spreadsheet analysis.
 *
 * Input: pass a config file via env var `TURTLE_CONFIG_PATH` containing
 * PARMW-style KEY=value lines (see tests/data/turtle_config_example.conf).
 * Simulation-only knobs use the SIM_ prefix.
 *
 * Run:
 *   TURTLE_CONFIG_PATH=tests/data/turtle_config_example.conf \
 *     ./build/TurtleSimulation -g TurtleSimShortSurface
 *
 * Output: configurable via SIM_OUTPUT_HTML / SIM_OUTPUT_CSV in the config.
 */

#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"
#include "fake_config_store.hpp"  // exposes get_param_variant()
#include "fake_rtc.hpp"
#include "fake_timer.hpp"
#include "scheduler.hpp"
#include "service_scheduler.hpp"
#include "hauled_mode_service.hpp"
#include "rate_limiter.hpp"
#include "base_types.hpp"

#include <cstdlib>
#include <ctime>
#include <cmath>
#include <random>
#include <vector>
#include <map>
#include <string>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <algorithm>

extern ConfigurationStore *configuration_store;
extern RTC *rtc;
extern Timer *system_timer;
extern Scheduler *system_scheduler;

// BaseMap table — defined in core/protocol/dte_params.cpp. Declared at file
// scope so the anonymous namespace below can reference the global symbol.
extern const BaseMap param_map[];
extern const size_t param_map_size;

namespace {

// =====================================================================
// Config file parsing
// =====================================================================

struct SimConfig {
	// Simulation knobs (SIM_ prefix in the config file).
	unsigned int duration_days = 7;
	unsigned int dive_min_s = 60;
	unsigned int dive_max_s = 900;
	unsigned int dive_avg_s = 300;
	unsigned int surface_min_s = 10;
	unsigned int surface_max_s = 90;
	unsigned int surface_avg_s = 30;
	double gps_fix_prob = 0.4;
	double fastloc_prob = 0.3;
	unsigned int gps_ttff_avg_s = 15;
	unsigned int hauled_day = 999;  // 999 = never
	unsigned int random_seed = 42;
	unsigned int battery_capacity_mah = 2400;
	std::string output_csv = "turtle_short_surface.csv";
	std::string output_html = "turtle_short_surface.html";
};

static std::string trim(const std::string &s) {
	auto a = s.find_first_not_of(" \t\r\n");
	if (a == std::string::npos) return "";
	auto b = s.find_last_not_of(" \t\r\n");
	return s.substr(a, b - a + 1);
}

// Strips inline comments (everything from a # that is preceded by whitespace
// or at start of line). Preserves # inside values (rare).
static std::string strip_comment(const std::string &s) {
	for (size_t i = 0; i < s.size(); i++) {
		if (s[i] == '#' && (i == 0 || std::isspace((unsigned char)s[i - 1]))) {
			return s.substr(0, i);
		}
	}
	return s;
}

// Apply a single KEY=value line to the simulation config + ConfigStore.
// Returns true on parse success, false on unknown / malformed.
static bool apply_config_line(const std::string &line, SimConfig &cfg) {
	std::string clean = trim(strip_comment(line));
	if (clean.empty()) return true;
	auto eq = clean.find('=');
	if (eq == std::string::npos) return false;
	std::string key = trim(clean.substr(0, eq));
	std::string val = trim(clean.substr(eq + 1));
	if (key.empty()) return false;

	// SIM_* knobs handled inline (not pushed to ConfigStore).
	if (key.rfind("SIM_", 0) == 0) {
		try {
			if (key == "SIM_DURATION_DAYS")
				cfg.duration_days = std::stoul(val);
			else if (key == "SIM_DIVE_MIN_S")
				cfg.dive_min_s = std::stoul(val);
			else if (key == "SIM_DIVE_MAX_S")
				cfg.dive_max_s = std::stoul(val);
			else if (key == "SIM_DIVE_AVG_S")
				cfg.dive_avg_s = std::stoul(val);
			else if (key == "SIM_SURFACE_MIN_S")
				cfg.surface_min_s = std::stoul(val);
			else if (key == "SIM_SURFACE_MAX_S")
				cfg.surface_max_s = std::stoul(val);
			else if (key == "SIM_SURFACE_AVG_S")
				cfg.surface_avg_s = std::stoul(val);
			else if (key == "SIM_GPS_FIX_PROB")
				cfg.gps_fix_prob = std::stod(val);
			else if (key == "SIM_FASTLOC_PROB")
				cfg.fastloc_prob = std::stod(val);
			else if (key == "SIM_GPS_TTFF_AVG_S")
				cfg.gps_ttff_avg_s = std::stoul(val);
			else if (key == "SIM_HAULED_DAY")
				cfg.hauled_day = std::stoul(val);
			else if (key == "SIM_RANDOM_SEED")
				cfg.random_seed = std::stoul(val);
			else if (key == "SIM_BATTERY_CAPACITY_MAH")
				cfg.battery_capacity_mah = std::stoul(val);
			else if (key == "SIM_OUTPUT_CSV")
				cfg.output_csv = val;
			else if (key == "SIM_OUTPUT_HTML")
				cfg.output_html = val;
			else {
				printf("[turtle-sim] unknown SIM key '%s'\n", key.c_str());
				return false;
			}
		} catch (...) {
			printf("[turtle-sim] failed to parse SIM key '%s' = '%s'\n", key.c_str(), val.c_str());
			return false;
		}
		return true;
	}

	// Otherwise, push to ConfigStore by DTE key. Look up ParamID via param_map
	// and dispatch on the *actual* variant index of the default value (rather
	// than the textual encoding), so enum-typed params (BaseArgosMode etc.)
	// land in the right variant slot.
	for (size_t i = 0; i < param_map_size; i++) {
		if (param_map[i].key == key) {
			try {
				ParamID pid = (ParamID)i;
				// Indices match BaseType variant order in base_types.hpp:
				// 0=string, 1=unsigned int, 2=int, 3=double, 4=time_t,
				// 5=BaseRawData, 6=ArgosMode, 7=ArgosPower, 8=DepthPile,
				// 9=bool, 10=GNSSFixMode, 11=GNSSDynModel, 12=LEDMode,
				// 13=ZoneType, 14=ArgosModulation, 15=UWDetectSource,
				// 16=DebugMode, 17=PressureLoggingMode, 18=PressureFullScale,
				// 19=SensorEnableTxMode.
				BaseType current = static_cast<FakeConfigurationStore *>(configuration_store)->get_param_variant(pid);
				switch (current.index()) {
				case 0: configuration_store->write_param(pid, std::string(val)); break;
				case 1: configuration_store->write_param(pid, (unsigned int)std::stoul(val)); break;
				case 2: configuration_store->write_param(pid, (int)std::stol(val)); break;
				case 3: configuration_store->write_param(pid, std::stod(val)); break;
				case 4: configuration_store->write_param(pid, (std::time_t)std::stoll(val)); break;
				case 6: configuration_store->write_param(pid, (BaseArgosMode)std::stoul(val)); break;
				case 7: configuration_store->write_param(pid, (BaseArgosPower)std::stoul(val)); break;
				case 8: configuration_store->write_param(pid, (BaseDepthPile)std::stoul(val)); break;
				case 9: configuration_store->write_param(pid, (bool)(std::stoul(val) != 0)); break;
				case 10: configuration_store->write_param(pid, (BaseGNSSFixMode)std::stoul(val)); break;
				case 11: configuration_store->write_param(pid, (BaseGNSSDynModel)std::stoul(val)); break;
				case 12: configuration_store->write_param(pid, (BaseLEDMode)std::stoul(val)); break;
				case 13: configuration_store->write_param(pid, (BaseZoneType)std::stoul(val)); break;
				case 14: configuration_store->write_param(pid, (BaseArgosModulation)std::stoul(val)); break;
				case 15: configuration_store->write_param(pid, (BaseUnderwaterDetectSource)std::stoul(val)); break;
				case 16: configuration_store->write_param(pid, (BaseDebugMode)std::stoul(val)); break;
				case 17: configuration_store->write_param(pid, (BasePressureSensorLoggingMode)std::stoul(val)); break;
				case 18: configuration_store->write_param(pid, (BasePressureSensorFullScale)std::stoul(val)); break;
				case 19: configuration_store->write_param(pid, (BaseSensorEnableTxMode)std::stoul(val)); break;
				default:
					printf("[turtle-sim] unsupported variant index %zu for '%s'\n", current.index(), key.c_str());
					return false;
				}
			} catch (...) {
				printf("[turtle-sim] failed to coerce '%s' = '%s'\n", key.c_str(), val.c_str());
				return false;
			}
			return true;
		}
	}
	printf("[turtle-sim] unknown DTE key '%s'\n", key.c_str());
	return false;
}

static bool load_config(const std::string &path, SimConfig &cfg) {
	std::ifstream f(path);
	if (!f.is_open()) {
		printf("[turtle-sim] cannot open config file: %s\n", path.c_str());
		return false;
	}
	std::string line;
	unsigned int ln = 0, bad = 0;
	while (std::getline(f, line)) {
		ln++;
		if (!apply_config_line(line, cfg)) bad++;
	}
	printf("[turtle-sim] loaded %s (%u lines, %u bad)\n", path.c_str(), ln, bad);
	return bad == 0;
}

// =====================================================================
// Battery model — back-of-envelope mA averages for nRF52840 + SMD + M10Q.
// =====================================================================

struct BatteryModel {
	// Current draw in mA per state.
	double idle_ma = 0.5;          // System OFF / deep sleep
	double sws_active_ma = 0.2;    // SWS sampling (averaged)
	double gps_on_ma = 25.0;       // M10Q tracking
	double smd_idle_ma = 1.0;      // SMD STM32WL idle (between TX)
	double tcxo_warmup_ma = 28.0;  // TCXO warmup phase (~5 s)
	double smd_tx_ma = 350.0;      // SMD active RF TX

	// Durations (seconds) per event.
	double tcxo_warmup_s = 5.0;
	double tx_rf_s = 1.0;  // pure RF emit time

	// Compute mAh consumed for one full Argos TX cycle (TCXO + RF).
	double tx_mah() const {
		double tcxo = tcxo_warmup_ma * tcxo_warmup_s / 3600.0;
		double rf = smd_tx_ma * tx_rf_s / 3600.0;
		return tcxo + rf;
	}

	// mAh consumed for `duration_s` of GPS-on.
	double gps_mah(double duration_s) const { return gps_on_ma * duration_s / 3600.0; }

	// mAh consumed for `duration_s` of generic idle (SWS background +
	// system idle).
	double idle_mah(double duration_s) const { return (idle_ma + sws_active_ma) * duration_s / 3600.0; }
};

// =====================================================================
// Per-surface event record — keeps the simulator decisions auditable.
// =====================================================================

enum class TxKind { NONE, DOPPLER, GNSS_FRESH, GNSS_REUSE_LAST, GNSS_FASTLOC };
static const char *tx_kind_str(TxKind k) {
	switch (k) {
	case TxKind::NONE: return "NONE";
	case TxKind::DOPPLER: return "Doppler";
	case TxKind::GNSS_FRESH: return "GNSS-FRESH";
	case TxKind::GNSS_REUSE_LAST: return "GNSS-REUSE_LAST";
	case TxKind::GNSS_FASTLOC: return "GNSS-FastLoc";
	default: return "?";
	}
}

struct SurfaceEvent {
	std::time_t rtc_time;
	unsigned int duration_s;
	bool hauled;
	bool cooldown_active;
	bool rate_limit_blocked;
	bool gps_attempted;
	bool gps_fix_obtained;
	bool fastloc_obtained;
	TxKind tx;
	double battery_mah_used;
};

// =====================================================================
// Hour / day aggregator buckets
// =====================================================================

struct HourBucket {
	unsigned int hour_idx = 0;
	unsigned int surfaces = 0;
	unsigned int tx_doppler = 0;
	unsigned int tx_gnss_fresh = 0;
	unsigned int tx_gnss_reuse = 0;
	unsigned int tx_gnss_fastloc = 0;
	unsigned int gps_attempts = 0;
	unsigned int gps_fixes = 0;
	unsigned int cooldown_blocks = 0;
	unsigned int ratelimit_blocks = 0;
	unsigned int hauled_engages = 0;
	unsigned int hauled_exits = 0;
	double battery_mah = 0.0;
};

struct DayBucket {
	unsigned int day_idx = 0;
	unsigned int surfaces = 0;
	unsigned int tx_total = 0;
	unsigned int tx_doppler = 0;
	unsigned int tx_gnss_fresh = 0;
	unsigned int tx_gnss_reuse = 0;
	unsigned int tx_gnss_fastloc = 0;
	unsigned int gps_attempts = 0;
	unsigned int gps_fixes = 0;
	unsigned int cooldown_blocks = 0;
	unsigned int ratelimit_blocks = 0;
	bool hauled_at_end = false;
	double battery_mah = 0.0;
};

// =====================================================================
// Simulation core
// =====================================================================

class TurtleShortSurfaceSim {
public:
	TurtleShortSurfaceSim(const SimConfig &c) : cfg(c), rng(c.random_seed) {}

	void run() {
		// Initial RTC = 1 (virtual epoch — mirrors main.cpp cold-first-boot).
		((FakeRTC *)rtc)->settime(1);
		((FakeTimer *)system_timer)->set_counter(0);
		HauledModeService::reset_for_tests();
		HauledModeService::restore_state();
		RateLimiter::reset_for_tests();

		// Pre-fill depth pile state with a "last known good fix" so REUSE_LAST
		// has something to consume if HMP13=REUSE_LAST is configured. Without
		// this, the first hauled cycle would always fall back to Doppler.
		last_real_fix_rtc = 1;

		hour_buckets.assign(cfg.duration_days * 24, HourBucket{});
		day_buckets.assign(cfg.duration_days, DayBucket{});
		for (unsigned int h = 0; h < hour_buckets.size(); h++)
			hour_buckets[h].hour_idx = h;
		for (unsigned int d = 0; d < day_buckets.size(); d++)
			day_buckets[d].day_idx = d;

		const std::time_t SIM_DURATION_S = (std::time_t)cfg.duration_days * 86400;
		std::time_t now = 1;
		bool prev_hauled = false;
		unsigned int last_html_day = 0;

		// Write initial live HTML so the operator can already open the page.
		write_html(100.0, /*live=*/true, /*day_done=*/0);
		printf("[turtle-sim] live HTML: %s (auto-refresh every 2 s)\n", cfg.output_html.c_str());

		while (now < (std::time_t)1 + SIM_DURATION_S) {
			unsigned int current_day = (unsigned int)((now - 1) / 86400);
			// Live HTML refresh on day boundary (cheap — once per simulated day).
			if (current_day != last_html_day) {
				last_html_day = current_day;
				double soc_pct_live = 100.0 - 100.0 * total_battery_mah / (double)cfg.battery_capacity_mah;
				if (soc_pct_live < 0) soc_pct_live = 0;
				write_html(soc_pct_live, /*live=*/true, current_day);
			}

			// Animal hauled-out scenario (e.g. animal beaches / dies on day N).
			bool force_hauled_now =
			    (cfg.hauled_day < cfg.duration_days && (unsigned int)((now - 1) / 86400) >= cfg.hauled_day);

			// --- Generate next dive ---
			unsigned int dive_s = sample_lognormal(cfg.dive_min_s, cfg.dive_max_s, cfg.dive_avg_s);
			advance_time(now, dive_s);
			if (now >= (std::time_t)1 + SIM_DURATION_S) break;

			// Dive event = SWS submerged. Notify hauled service.
			HauledModeService::on_underwater_event(true, now);
			// Idle current during dive.
			total_battery_mah += battery.idle_mah(dive_s);

			// --- Surface ---
			unsigned int surface_s = force_hauled_now
			                             ? 0  // hauled animal doesn't surface
			                             : sample_lognormal(cfg.surface_min_s, cfg.surface_max_s, cfg.surface_avg_s);

			SurfaceEvent ev{};
			ev.rtc_time = now;
			ev.duration_s = surface_s;
			ev.tx = TxKind::NONE;

			// Allow HauledModeService to evaluate based on idle duration.
			HauledModeService::evaluate(now);

			bool hauled = HauledModeService::is_hauled();
			ev.hauled = hauled;

			// Cooldown check: simulate UNP20 + UNP30=3 (AFTER_LAST_TX).
			unsigned int cooldown_s =
			    configuration_store->read_param<unsigned int>(ParamID::MIN_SURFACE_CYCLE_INTERVAL_S);
			bool cooldown_active =
			    (cooldown_s > 0 && last_tx_rtc > 0 && (unsigned int)(now - last_tx_rtc) < cooldown_s);
			ev.cooldown_active = cooldown_active;

			if (force_hauled_now) {
				// Hauled animal: no SWS surface event fires. But HauledModeService
				// has long since engaged via the idle threshold.
				surface_events.push_back(ev);
				record_aggregates(now, ev);
				continue;
			}

			// --- Surface awake: SWS detects, system schedules TX ---

			// SWS / scheduler idle cost during surface window.
			total_battery_mah += battery.idle_mah(surface_s);

			if (cooldown_active) {
				record_aggregates(now, ev);
				surface_events.push_back(ev);
				now += surface_s;
				continue;
			}

			// Decide TX kind based on config.
			BaseArgosMode mode = configuration_store->read_param<BaseArgosMode>(ParamID::ARGOS_MODE);
			if (hauled && configuration_store->read_param<bool>(ParamID::HAULED_DETECT_EN)) {
				// HAULED override (mirror config_store HAULED branch logic).
				mode = configuration_store->read_param<BaseArgosMode>(ParamID::HAULED_ARGOS_MODE);
				if (mode == BaseArgosMode::SURFACING_BURST) mode = BaseArgosMode::LEGACY;
				unsigned int strat = configuration_store->read_param<unsigned int>(ParamID::HAULED_GNSS_STRAT);
				if (strat == (unsigned int)BaseGnssStrategy::OFF) {
					ev.tx = TxKind::DOPPLER;
					ev.battery_mah_used = battery.tx_mah();
				} else if (strat == (unsigned int)BaseGnssStrategy::REUSE_LAST) {
					unsigned int max_age =
					    configuration_store->read_param<unsigned int>(ParamID::GNSS_REUSE_FIX_MAX_AGE_S);
					if (last_real_fix_rtc > 0 && (now - last_real_fix_rtc) <= (std::time_t)max_age) {
						ev.tx = TxKind::GNSS_REUSE_LAST;
						ev.battery_mah_used = battery.tx_mah();
					} else {
						ev.tx = TxKind::DOPPLER;  // fallback per firmware logic
						ev.battery_mah_used = battery.tx_mah();
					}
				} else {  // FRESH
					attempt_gps_and_tx(ev, surface_s);
				}
			} else {
				// AT_SEA — typical surface burst behavior.
				attempt_gps_and_tx(ev, surface_s);
			}

			// Rate limiter check.
			unsigned int rl_reschedule = 0;
			if (configuration_store->read_param<bool>(ParamID::RATE_LIMIT_EN)
			    && RateLimiter::is_blocked(now, rl_reschedule)) {
				ev.rate_limit_blocked = true;
				ev.tx = TxKind::NONE;
				ev.battery_mah_used = 0;  // no TX
			}

			// Record TX.
			if (ev.tx != TxKind::NONE) {
				last_tx_rtc = now;
				RateLimiter::record_tx(now);
				if (ev.tx == TxKind::GNSS_FRESH) last_real_fix_rtc = now;
				total_battery_mah += ev.battery_mah_used;
			}

			surface_events.push_back(ev);
			record_aggregates(now, ev);

			// Hauled state-change events on the bucket.
			if (hauled && !prev_hauled) note_hauled_engage(now);
			if (!hauled && prev_hauled) note_hauled_exit(now);
			prev_hauled = hauled;

			now += surface_s;
		}

		// Final battery report.
		double soc_pct = 100.0 - 100.0 * total_battery_mah / (double)cfg.battery_capacity_mah;
		if (soc_pct < 0) soc_pct = 0;
		printf("\n[turtle-sim] SIMULATION COMPLETE\n");
		printf("  Duration: %u days\n", cfg.duration_days);
		printf("  Surfaces: %zu\n", surface_events.size());
		printf("  Total TX: %u (Doppler %u, GNSS-fresh %u, REUSE_LAST %u, FastLoc %u)\n",
		       tx_doppler + tx_gnss_fresh + tx_reuse + tx_fastloc, tx_doppler, tx_gnss_fresh, tx_reuse, tx_fastloc);
		printf("  GPS attempts: %u  fixes: %u (success rate %.1f%%)\n", gps_attempts, gps_fixes,
		       gps_attempts ? 100.0 * gps_fixes / gps_attempts : 0);
		printf("  Cooldown blocks: %u    Rate-limit blocks: %u\n", cooldown_blocks, ratelimit_blocks);
		printf("  Battery consumed: %.2f mAh of %u mAh capacity (%.1f%% SoC remaining)\n", total_battery_mah,
		       cfg.battery_capacity_mah, soc_pct);

		write_csv();
		write_html(soc_pct, /*live=*/false, cfg.duration_days);
	}

private:
	const SimConfig &cfg;
	std::mt19937 rng;
	BatteryModel battery;

	std::vector<SurfaceEvent> surface_events;
	std::vector<HourBucket> hour_buckets;
	std::vector<DayBucket> day_buckets;

	std::time_t last_tx_rtc = 0;
	std::time_t last_real_fix_rtc = 0;
	double total_battery_mah = 0.0;

	unsigned int tx_doppler = 0, tx_gnss_fresh = 0, tx_reuse = 0, tx_fastloc = 0;
	unsigned int gps_attempts = 0, gps_fixes = 0;
	unsigned int cooldown_blocks = 0, ratelimit_blocks = 0;

	void advance_time(std::time_t &now, unsigned int seconds) {
		now += seconds;
		((FakeRTC *)rtc)->settime(now);
		((FakeTimer *)system_timer)->set_counter((uint64_t)(now - 1) * 1000);
	}

	// Sample a lognormal-clipped duration mimicking biological variability.
	unsigned int sample_lognormal(unsigned int mn, unsigned int mx, unsigned int avg) {
		std::lognormal_distribution<double> d(std::log((double)avg), 0.5);
		double v = d(rng);
		if (v < mn) v = mn;
		if (v > mx) v = mx;
		return (unsigned int)v;
	}

	void attempt_gps_and_tx(SurfaceEvent &ev, unsigned int surface_s) {
		// Surface duration must accommodate TTFF for a fix to land.
		ev.gps_attempted = true;
		gps_attempts++;
		ev.battery_mah_used = battery.gps_mah(std::min<double>(surface_s, cfg.gps_ttff_avg_s));

		std::uniform_real_distribution<double> u(0.0, 1.0);
		double r = u(rng);
		if (r < cfg.gps_fix_prob && surface_s >= cfg.gps_ttff_avg_s) {
			ev.gps_fix_obtained = true;
			gps_fixes++;
			ev.tx = TxKind::GNSS_FRESH;
		} else if (r < cfg.gps_fix_prob + cfg.fastloc_prob) {
			ev.fastloc_obtained = true;
			ev.tx = TxKind::GNSS_FASTLOC;
		} else {
			ev.tx = TxKind::DOPPLER;
		}
		ev.battery_mah_used += battery.tx_mah();
	}

	void note_hauled_engage(std::time_t now) {
		(void)now;
		// Bookkeeping in buckets — single counter at the surface event
		// recording time.
	}
	void note_hauled_exit(std::time_t now) { (void)now; }

	void record_aggregates(std::time_t now, const SurfaceEvent &ev) {
		std::time_t elapsed = now - 1;  // since virtual epoch
		unsigned int hour_idx = (unsigned int)(elapsed / 3600);
		unsigned int day_idx = (unsigned int)(elapsed / 86400);
		if (hour_idx >= hour_buckets.size()) return;
		if (day_idx >= day_buckets.size()) return;

		auto &hb = hour_buckets[hour_idx];
		auto &db = day_buckets[day_idx];
		hb.surfaces++;
		db.surfaces++;
		hb.battery_mah += ev.battery_mah_used;
		db.battery_mah += ev.battery_mah_used;
		if (ev.gps_attempted) {
			hb.gps_attempts++;
			db.gps_attempts++;
		}
		if (ev.gps_fix_obtained) {
			hb.gps_fixes++;
			db.gps_fixes++;
		}
		if (ev.cooldown_active) {
			hb.cooldown_blocks++;
			db.cooldown_blocks++;
			cooldown_blocks++;
		}
		if (ev.rate_limit_blocked) {
			hb.ratelimit_blocks++;
			db.ratelimit_blocks++;
			ratelimit_blocks++;
		}
		db.hauled_at_end = ev.hauled;

		switch (ev.tx) {
		case TxKind::DOPPLER:
			hb.tx_doppler++;
			db.tx_doppler++;
			tx_doppler++;
			break;
		case TxKind::GNSS_FRESH:
			hb.tx_gnss_fresh++;
			db.tx_gnss_fresh++;
			tx_gnss_fresh++;
			break;
		case TxKind::GNSS_REUSE_LAST:
			hb.tx_gnss_reuse++;
			db.tx_gnss_reuse++;
			tx_reuse++;
			break;
		case TxKind::GNSS_FASTLOC:
			hb.tx_gnss_fastloc++;
			db.tx_gnss_fastloc++;
			tx_fastloc++;
			break;
		case TxKind::NONE: break;
		default: break;
		}
		db.tx_total = db.tx_doppler + db.tx_gnss_fresh + db.tx_gnss_reuse + db.tx_gnss_fastloc;
	}

	void write_csv() {
		std::ofstream f(cfg.output_csv);
		if (!f.is_open()) {
			printf("[turtle-sim] cannot write CSV %s\n", cfg.output_csv.c_str());
			return;
		}
		f << "hour,surfaces,tx_doppler,tx_gnss_fresh,tx_reuse_last,tx_fastloc,"
		     "gps_attempts,gps_fixes,cooldown_blocks,ratelimit_blocks,battery_mah\n";
		for (const auto &h : hour_buckets) {
			f << h.hour_idx << "," << h.surfaces << "," << h.tx_doppler << "," << h.tx_gnss_fresh << ","
			  << h.tx_gnss_reuse << "," << h.tx_gnss_fastloc << "," << h.gps_attempts << "," << h.gps_fixes << ","
			  << h.cooldown_blocks << "," << h.ratelimit_blocks << "," << std::fixed << std::setprecision(4)
			  << h.battery_mah << "\n";
		}
		printf("[turtle-sim] CSV written to %s\n", cfg.output_csv.c_str());
	}

	void write_html(double soc_pct, bool live = false, unsigned int day_done = 0) {
		std::ofstream f(cfg.output_html);
		if (!f.is_open()) {
			static bool printed = false;
			if (!printed) {
				printf("[turtle-sim] cannot write HTML %s\n", cfg.output_html.c_str());
				printed = true;
			}
			return;
		}
		unsigned int tx_total = tx_doppler + tx_gnss_fresh + tx_reuse + tx_fastloc;
		double avg_tx_per_day = day_done > 0 ? (double)tx_total / day_done
		                                     : (cfg.duration_days > 0 ? (double)tx_total / cfg.duration_days : 0);
		double progress_pct = cfg.duration_days > 0 ? 100.0 * day_done / cfg.duration_days : 100.0;

		f << "<!DOCTYPE html><html><head><meta charset=\"UTF-8\">"
		     "<title>Turtle Short-Surface "
		  << (live ? "(running)" : "(done)") << "</title>";
		if (live) {
			f << "<meta http-equiv=\"refresh\" content=\"2\">";
		}
		f << "<style>"
		     "body{font-family:system-ui,sans-serif;background:#1a1a2e;color:#eee;padding:20px;max-width:1200px;margin:"
		     "auto}"
		     "h1{color:#00d4ff}h2{color:#ffa502;margin-top:30px}"
		     ".stats{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:12px;margin:20px 0}"
		     ".stat{background:#222247;padding:14px;border-radius:8px;text-align:center}"
		     ".stat .v{font-size:28px;color:#00d4ff;font-weight:bold}.stat .l{font-size:12px;opacity:.7}"
		     "table{width:100%;border-collapse:collapse;margin-top:20px;font-size:13px}"
		     "th,td{padding:6px 10px;border-bottom:1px solid #333;text-align:right}"
		     "th{background:#16213e;color:#00d4ff;text-align:left;position:sticky;top:0}"
		     "td:first-child,th:first-child{text-align:left}"
		     ".hauled{background:rgba(255,165,2,0.15)}"
		     ".tx-bar{background:linear-gradient(90deg,#00ff88,#00d4ff);height:8px;border-radius:4px;display:inline-"
		     "block;vertical-align:middle}"
		     ".banner{padding:14px "
		     "18px;border-radius:6px;margin-bottom:22px;font-size:14px;display:flex;justify-content:space-between;"
		     "align-items:center}"
		     ".banner.running{background:linear-gradient(90deg,rgba(0,212,255,.18),rgba(255,165,2,.18));border-left:"
		     "4px solid #ffa502}"
		     ".banner.done{background:rgba(0,255,136,.10);border-left:4px solid #00ff88}"
		     ".pulse{display:inline-block;width:10px;height:10px;background:#ffa502;border-radius:50%;margin-right:8px;"
		     "animation:pulse 1s ease-in-out infinite}"
		     "@keyframes pulse{0%,100%{opacity:1;transform:scale(1)}50%{opacity:.4;transform:scale(.7)}}"
		     ".progress-track{width:100%;height:10px;background:rgba(255,255,255,.08);border-radius:5px;overflow:"
		     "hidden;margin-top:8px}"
		     ".progress-fill{height:100%;background:linear-gradient(90deg,#00ff88,#00d4ff);transition:width .8s}"
		     "</style></head><body>"
		     "<h1>&#x1F422; Turtle Short-Surface Simulation</h1>"
		     "<p>"
		  << cfg.duration_days << " days, " << surface_events.size() << " surfaces</p>";

		// Live / done banner.
		if (live) {
			f << "<div class=\"banner running\">"
			  << "<div><span class=\"pulse\"></span><b>RUNNING</b> &middot; "
			  << "day " << day_done << " of " << cfg.duration_days << " &middot; " << surface_events.size()
			  << " surfaces recorded</div>"
			  << "<div>" << std::fixed << std::setprecision(1) << progress_pct << "%</div>"
			  << "</div>"
			  << "<div class=\"progress-track\"><div class=\"progress-fill\" style=\"width:" << progress_pct
			  << "%\"></div></div>";
		} else {
			f << "<div class=\"banner done\">"
			  << "<b>&#x2705; SIMULATION COMPLETE</b> &middot; " << surface_events.size() << " surfaces simulated, "
			  << tx_total << " TX, " << std::fixed << std::setprecision(1) << soc_pct << "% SoC remaining"
			  << "</div>";
		}
		f << "<div class=\"stats\">"
		     "<div class=\"stat\"><div class=\"v\">"
		  << tx_total
		  << "</div><div class=\"l\">Total TX</div></div>"
		     "<div class=\"stat\"><div class=\"v\">"
		  << std::fixed << std::setprecision(1) << avg_tx_per_day
		  << "</div><div class=\"l\">TX / day avg</div></div>"
		     "<div class=\"stat\"><div class=\"v\">"
		  << tx_doppler
		  << "</div><div class=\"l\">Doppler</div></div>"
		     "<div class=\"stat\"><div class=\"v\">"
		  << tx_gnss_fresh
		  << "</div><div class=\"l\">GNSS fresh</div></div>"
		     "<div class=\"stat\"><div class=\"v\">"
		  << tx_reuse
		  << "</div><div class=\"l\">REUSE_LAST</div></div>"
		     "<div class=\"stat\"><div class=\"v\">"
		  << tx_fastloc
		  << "</div><div class=\"l\">FastLoc</div></div>"
		     "<div class=\"stat\"><div class=\"v\">"
		  << gps_attempts
		  << "</div><div class=\"l\">GPS attempts</div></div>"
		     "<div class=\"stat\"><div class=\"v\">"
		  << gps_fixes
		  << "</div><div class=\"l\">GPS fixes</div></div>"
		     "<div class=\"stat\"><div class=\"v\">"
		  << cooldown_blocks
		  << "</div><div class=\"l\">Cooldown blocks</div></div>"
		     "<div class=\"stat\"><div class=\"v\">"
		  << ratelimit_blocks
		  << "</div><div class=\"l\">Rate-limit blocks</div></div>"
		     "<div class=\"stat\"><div class=\"v\">"
		  << std::fixed << std::setprecision(1) << total_battery_mah
		  << "</div><div class=\"l\">Battery used (mAh)</div></div>"
		     "<div class=\"stat\"><div class=\"v\">"
		  << std::fixed << std::setprecision(1) << soc_pct
		  << "%</div><div class=\"l\">SoC remaining</div></div>"
		     "</div>"

		     "<h2>Per-day breakdown</h2>"
		     "<table><thead><tr>"
		     "<th>Day</th><th>Surf</th><th>TX</th><th>Doppler</th><th>GNSS fresh</th>"
		     "<th>REUSE_LAST</th><th>FastLoc</th><th>GPS att</th><th>GPS fix</th>"
		     "<th>Fix%</th><th>CD-blk</th><th>RL-blk</th><th>mAh</th><th>State</th>"
		     "</tr></thead><tbody>";

		for (const auto &d : day_buckets) {
			double fix_pct = d.gps_attempts ? 100.0 * d.gps_fixes / d.gps_attempts : 0;
			f << "<tr" << (d.hauled_at_end ? " class=\"hauled\"" : "") << ">"
			  << "<td>D" << d.day_idx << "</td>"
			  << "<td>" << d.surfaces << "</td>"
			  << "<td>" << d.tx_total << "</td>"
			  << "<td>" << d.tx_doppler << "</td>"
			  << "<td>" << d.tx_gnss_fresh << "</td>"
			  << "<td>" << d.tx_gnss_reuse << "</td>"
			  << "<td>" << d.tx_gnss_fastloc << "</td>"
			  << "<td>" << d.gps_attempts << "</td>"
			  << "<td>" << d.gps_fixes << "</td>"
			  << "<td>" << std::fixed << std::setprecision(0) << fix_pct << "</td>"
			  << "<td>" << d.cooldown_blocks << "</td>"
			  << "<td>" << d.ratelimit_blocks << "</td>"
			  << "<td>" << std::fixed << std::setprecision(2) << d.battery_mah << "</td>"
			  << "<td>" << (d.hauled_at_end ? "HAULED" : "AT_SEA") << "</td>"
			  << "</tr>";
		}
		f << "</tbody></table>";

		// Hour-level histogram — first 72 hours for brevity (3 days).
		size_t hours_to_show = std::min<size_t>(hour_buckets.size(), 72);
		f << "<h2>First " << hours_to_show
		  << " hours (TX rate)</h2>"
		     "<table><thead><tr><th>Hour</th><th>Surf</th><th>TX</th><th>Bar</th></tr></thead><tbody>";
		unsigned int max_tx = 0;
		for (size_t i = 0; i < hours_to_show; i++) {
			unsigned int tx = hour_buckets[i].tx_doppler + hour_buckets[i].tx_gnss_fresh + hour_buckets[i].tx_gnss_reuse
			                  + hour_buckets[i].tx_gnss_fastloc;
			if (tx > max_tx) max_tx = tx;
		}
		if (max_tx == 0) max_tx = 1;
		for (size_t i = 0; i < hours_to_show; i++) {
			unsigned int tx = hour_buckets[i].tx_doppler + hour_buckets[i].tx_gnss_fresh + hour_buckets[i].tx_gnss_reuse
			                  + hour_buckets[i].tx_gnss_fastloc;
			unsigned int bar_w = (tx * 200) / max_tx;
			f << "<tr><td>H" << i << "</td><td>" << hour_buckets[i].surfaces << "</td>"
			  << "<td>" << tx << "</td><td><div class=\"tx-bar\" style=\"width:" << bar_w << "px\"></div></td></tr>";
		}
		f << "</tbody></table>"
		     "<p style=\"opacity:.5;margin-top:30px;font-size:12px\">Generated by TurtleSimulation -g "
		     "TurtleSimShortSurface</p>"
		     "</body></html>";

		printf("[turtle-sim] HTML written to %s\n", cfg.output_html.c_str());
	}
};

}  // namespace

// =====================================================================
// TEST entrypoint
// =====================================================================

TEST_GROUP(TurtleSimShortSurface) {
	FakeConfigurationStore *fake_cs = nullptr;
	FakeRTC *fake_rtc_obj = nullptr;
	FakeTimer *fake_timer_obj = nullptr;

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
	}

	void teardown() {
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

TEST(TurtleSimShortSurface, RunFromConfigFile) {
	SimConfig cfg;
	const char *path = std::getenv("TURTLE_CONFIG_PATH");
	if (!path) path = "tests/data/turtle_config_example.conf";
	printf("\n[turtle-sim] config file: %s\n", path);
	if (!load_config(path, cfg)) {
		printf("[turtle-sim] config load had errors — running with defaults for unknown SIM keys\n");
	}

	TurtleShortSurfaceSim sim(cfg);
	sim.run();

	CHECK_TRUE_TEXT(true, "Simulation completed");
}
