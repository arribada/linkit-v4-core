/**
 * SWS long-run simulation — post-mortem of the WWF-YanaRiba 2026 deployment.
 *
 * The seven LinkIt v4 Argos tags ran in SURFACING_BURST mode, where every
 * transmission follows a "surface" event from the analog saltwater switch.
 * Their message rate decayed toward zero after 8-12 weeks with flat battery
 * and RF. This simulation drives the real detector (sws_analog_detection.cpp,
 * unchanged) for N simulated days with:
 *
 *   - the circuit described in wiki page 13 §B.1: a 100 nF capacitor on the
 *     ADC pin charged through the medium for `sample_delay_us`, with a 1 MΩ
 *     pull-down. counts = K · Vdd · Rpd/(R+Rpd) · (1 − exp(−d/τ)),
 *     τ = (R ∥ Rpd) · C. The medium resistance R is the unknown: the repo
 *     quotes three values for seawater (1 kΩ in sws_analog_constants.hpp,
 *     10 kΩ in the wiki, ~45 kΩ to reproduce the "~3000 counts at 1 ms" seen
 *     on boards), so the scenarios are run for two water hypotheses;
 *   - progressive biofouling (logistic in time): the dry electrode becomes
 *     more conductive, the wet contact more resistive, the water film drains
 *     more slowly after each surfacing, and contact noise grows;
 *   - a sea-turtle dive/surface pattern (lognormal dive 8-50 min, surfacing
 *     30-300 s) with per-sample multiplicative noise.
 *
 * It is a documentary instrument, not a regression gate: only the clean,
 * fixed-delay control carries a pass criterion. Per-day statistics are
 * written as CSV when SWS_SIM_OUT names a directory, and a summary table is
 * printed.
 *
 *   cd tests && SWS_SIM_OUT=../.claude/wwf-yanariba/sim \
 *       ./build/TrackerTests -g SWSLongRun -v
 *
 * SWS_SIM_DAYS overrides the simulated duration (default 90).
 *
 * Reading the columns:
 *   surf   true surfacings started that day
 *   det    surfacings during which the detector flipped UW -> surface
 *          (what starts an Argos burst)
 *   cov    surfacings during which the detector believed "surface" at all
 *   false  UW -> surface flips while the animal was actually diving
 *   epis   believed-surface episodes started (burst starts)
 *   short  ... of which shorter than 3 s (an Argos TX started on them is
 *          cancelled by the return to "underwater" before it completes)
 *   dto/fsf dive timeouts / forced surfacings (UW_MAX_DIVE_TIME escalation)
 */

#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

#include "sws_analog_service.hpp"
#include "bsp.hpp"
#include "nrfx_saadc.h"
#include "fake_config_store.hpp"
#include "fake_timer.hpp"
#include "filesystem.hpp"
#include "calibration.hpp"
#include "debug.hpp"

extern Timer *system_timer;
extern ConfigurationStore *configuration_store;
extern Scheduler *system_scheduler;

namespace {

// ---------------------------------------------------------------------------
// Physical model of the RC discrimination circuit (wiki 13 §B.1)
// ---------------------------------------------------------------------------
struct Physics {
	double vdd_v = 3.0;                   // GPIO drive
	double counts_per_v = 16383.0 / 3.6;  // gain 1/6, 0.6 V reference, 14 bits
	double r_pd_ohm = 1.0e6;              // pull-down on the ADC node
	double c_f = 100e-9;                  // 100 nF
	// Medium resistance (Ω): clean and fully fouled values.
	double r_water_clean = 45.0e3;   // ~3000 counts at 1 ms (board observations)
	double r_water_fouled = 60.0e3;  // fouled contact: slightly more resistive
	double r_air_clean = 700.0e3;    // ~200 counts at 1 ms (humid salt residue)
	double r_air_fouled = 150.0e3;   // damp biofilm in air
	// Water film left on the electrodes after a surfacing: R relaxes from
	// r_water to r_air with this time constant (s).
	double film_tau_clean_s = 2.0;
	double film_tau_fouled_s = 25.0;
	// Multiplicative per-sample noise. 1 % keeps the L1 override (4 % single
	// drop) quiet on a clean electrode; biofilm contact noise raises it.
	double noise_clean = 0.01;
	double noise_fouled = 0.04;

	double counts(double r_ohm, double delay_us) const {
		double r_par = (r_ohm * r_pd_ohm) / (r_ohm + r_pd_ohm);
		double tau_s = r_par * c_f;
		double v_ss = vdd_v * r_pd_ohm / (r_ohm + r_pd_ohm);
		double v = v_ss * (1.0 - std::exp(-(delay_us * 1e-6) / tau_s));
		return v * counts_per_v;
	}
};

static double sim_mix(double a, double b, double f) {
	return a + (b - a) * f;
}
static double sim_mix_log(double a, double b, double f) {
	return std::exp(sim_mix(std::log(a), std::log(b), f));
}

// ---------------------------------------------------------------------------
// Scenario definition
// ---------------------------------------------------------------------------
struct Scenario {
	const char *name;
	Physics phys;
	double fouling_d50_days;    // logistic midpoint (days)
	double fouling_width_days;  // logistic width (days)
	double fouling_max;         // 0 = no fouling, 1 = fully fouled values
	bool conductance_drifts;    // r_air / r_water follow fouling
	bool film_grows;            // water-film retention grows with fouling
	bool noise_grows;           // contact noise grows with fouling
	unsigned delay_min_us;      // SWS_DELAY_MIN_US
	unsigned delay_max_us;      // SWS_DELAY_MAX_US
	unsigned max_dive_time_s;   // UW_MAX_DIVE_TIME
};

static double fouling_at(const Scenario &sc, double day) {
	if (sc.fouling_max <= 0.0) return 0.0;
	double x = (day - sc.fouling_d50_days) / sc.fouling_width_days;
	return sc.fouling_max / (1.0 + std::exp(-x));
}

struct DayStats {
	unsigned day = 0;
	double fouling = 0;
	unsigned true_surfacings = 0;
	unsigned detected_transitions = 0;
	unsigned covered_segments = 0;
	unsigned false_surfacings = 0;
	unsigned surface_episodes = 0;
	unsigned short_episodes = 0;
	double true_dry_s = 0;
	double believed_surface_s = 0;
	double overlap_s = 0;
	std::vector<double> latencies_s;
	unsigned dive_timeouts = 0;
	unsigned force_surfaces = 0;
	SWSAnalogService::Status status_end{};
};

struct SimResult {
	std::vector<DayStats> days;
};

}  // namespace

// ---------------------------------------------------------------------------
// Test group: production-like configuration (WWF template, May 2026)
// ---------------------------------------------------------------------------
TEST_GROUP(SWSLongRun) {
	FakeConfigurationStore *fake_config_store;
	FakeTimer *fake_timer;

	void setup() {
		fake_config_store = new FakeConfigurationStore;
		configuration_store = fake_config_store;
		fake_timer = new FakeTimer;
		system_timer = fake_timer;
		fake_timer->start();
		system_scheduler = new Scheduler(system_timer);
		configuration_store->init();

		bool uw_en = true;
		auto uw_src = BaseUnderwaterDetectSource::SWS;
		unsigned int one = 1U, gap = 1000U;
		unsigned int hyst = 4U, calib_interval = 3600U, min_surface = 5U;
		// WWF-YanaRiba deployed configuration (devices.json, 2026-05-25 snapshots):
		// UW_WET_SAMPLING 0.5 s, UW_DRY_SAMPLING 3 s, UW_PIN_SAMPLE_DELAY 500 µs,
		// SWS_DELAY 200-1000 µs, UW_MIN_SURFACE_TIME 5 s, UW_MAX_DIVE_TIME 7200 s.
		double wet_period_s = 0.5, dry_period_s = 3.0;  // UW_WET_SAMPLING / UW_DRY_SAMPLING

		configuration_store->write_param(ParamID::UNDERWATER_EN, uw_en);
		configuration_store->write_param(ParamID::UNDERWATER_DETECT_SOURCE, uw_src);
		configuration_store->write_param(ParamID::SAMPLING_UNDER_FREQ, wet_period_s);
		configuration_store->write_param(ParamID::SAMPLING_SURF_FREQ, dry_period_s);
		configuration_store->write_param(ParamID::UW_MAX_SAMPLES, one);
		configuration_store->write_param(ParamID::UW_MIN_DRY_SAMPLES, one);
		configuration_store->write_param(ParamID::UW_SAMPLE_GAP, gap);
		configuration_store->write_param(ParamID::UW_PIN_SAMPLE_DELAY_US, 500U);
		configuration_store->write_param(ParamID::SWS_ANALOG_HYSTERESIS, hyst);
		configuration_store->write_param(ParamID::SWS_ANALOG_CALIB_INTERVAL, calib_interval);
		configuration_store->write_param(ParamID::UW_MIN_SURFACE_TIME, min_surface);

		SAADC::set_adc_value(0);
		SWSAnalogService::reset_noinit_data();
		mock().ignoreOtherCalls();
	}

	void teardown() {
		delete system_scheduler;
		delete fake_timer;
		delete fake_config_store;
		mock().clear();
	}

	unsigned sim_days() {
		const char *env = std::getenv("SWS_SIM_DAYS");
		if (env && std::atoi(env) > 0) return static_cast<unsigned>(std::atoi(env));
		return 75;
	}

	SimResult run_scenario(const Scenario &sc) {
		configuration_store->write_param(ParamID::SWS_DELAY_MIN_US, sc.delay_min_us);
		configuration_store->write_param(ParamID::SWS_DELAY_MAX_US, sc.delay_max_us);
		configuration_store->write_param(ParamID::UW_MAX_DIVE_TIME, sc.max_dive_time_s);

		std::mt19937_64 rng(20260525ULL);
		std::normal_distribution<double> gauss(0.0, 1.0);
		// Olive ridley inter-nesting behaviour: median dive 20 min, median
		// surfacing 90 s. Clamped so no segment is absurd.
		std::lognormal_distribution<double> dive_min(std::log(20.0), 0.5);
		std::lognormal_distribution<double> surf_s(std::log(90.0), 0.4);

		SimResult result;
		const unsigned days = sim_days();
		result.days.resize(days);
		for (unsigned d = 0; d < days; d++) {
			result.days[d].day = d;
			result.days[d].fouling = fouling_at(sc, d + 0.5);
		}

		bool believed_uw = false;
		bool have_belief = false;
		std::vector<std::pair<uint64_t, bool>> transitions;

		SWSAnalogService s;
		s.start([&](ServiceEvent &event) {
			if (event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
				bool uw = std::get<bool>(event.event_data);
				if (!have_belief || uw != believed_uw) transitions.emplace_back(system_timer->get_counter(), uw);
				believed_uw = uw;
				have_belief = true;
			}
		});

		// Truth: start dry for one minute (tag switched on at the surface),
		// then alternate dive / surfacing.
		const uint64_t t0 = system_timer->get_counter();
		bool truth_wet = false;
		uint64_t seg_start = t0;
		uint64_t seg_end = t0 + 60 * 1000;
		bool seg_transition_seen = false;
		bool seg_covered = false;
		double seg_latency_s = -1.0;
		unsigned seg_day = 0;

		uint64_t episode_start_ms = 0;
		bool episode_open = false;
		unsigned episode_day = 0;

		// The detector traces every sample at DEBUG_LEVEL=4; over 90 days that is
		// gigabytes of console output. Silence the console logger for the run.
		Logger *saved_console = DebugLogger::console_log;
		DebugLogger::console_log = nullptr;

		SWSAnalogService::Diagnostics diag_prev = SWSAnalogService::get_diagnostics();
		const uint64_t step_ms = 250;  // detector samples every 0.5 s / 3 s
		const uint64_t day_ms = 86400ULL * 1000ULL;
		const uint64_t total_ms = static_cast<uint64_t>(days) * day_ms;
		size_t transitions_seen = 0;

		for (uint64_t t_rel = 0; t_rel < total_ms; t_rel += step_ms) {
			const uint64_t now = t0 + t_rel;
			const unsigned day = static_cast<unsigned>(t_rel / day_ms);
			DayStats &ds = result.days[day];

			// --- truth segment bookkeeping ---
			if (now >= seg_end) {
				if (!truth_wet) {
					DayStats &sd = result.days[seg_day];
					if (seg_transition_seen) {
						sd.detected_transitions++;
						sd.latencies_s.push_back(seg_latency_s);
					}
					if (seg_covered) sd.covered_segments++;
				}
				truth_wet = !truth_wet;
				seg_start = now;
				seg_day = day;
				if (truth_wet) {
					double minutes = std::clamp(dive_min(rng), 3.0, 120.0);
					seg_end = now + static_cast<uint64_t>(minutes * 60.0 * 1000.0);
				} else {
					double seconds = std::clamp(surf_s(rng), 30.0, 300.0);
					seg_end = now + static_cast<uint64_t>(seconds * 1000.0);
					ds.true_surfacings++;
					seg_transition_seen = false;
					seg_covered = false;
					seg_latency_s = -1.0;
				}
			}

			// --- physical ADC value for this instant ---
			const double f = fouling_at(sc, static_cast<double>(t_rel) / static_cast<double>(day_ms));
			const double f_cond = sc.conductance_drifts ? f : 0.0;
			const double f_film = sc.film_grows ? f : 0.0;
			const double f_noise = sc.noise_grows ? f : 0.0;
			SWSAnalogService::Status st = SWSAnalogService::get_status();
			double delay_us = st.sample_delay_us ? st.sample_delay_us : 500.0;
			double r_water = sim_mix_log(sc.phys.r_water_clean, sc.phys.r_water_fouled, f_cond);
			double r_air = sim_mix_log(sc.phys.r_air_clean, sc.phys.r_air_fouled, f_cond);
			double r_path;
			if (truth_wet) {
				r_path = r_water;
			} else {
				double film_tau = sim_mix(sc.phys.film_tau_clean_s, sc.phys.film_tau_fouled_s, f_film);
				double t_dry_s = static_cast<double>(now - seg_start) / 1000.0;
				r_path = sim_mix_log(r_air, r_water, std::exp(-t_dry_s / film_tau));
			}
			double v = sc.phys.counts(r_path, delay_us);
			double sigma = sim_mix(sc.phys.noise_clean, sc.phys.noise_fouled, f_noise);
			v *= (1.0 + sigma * gauss(rng));
			v = std::clamp(v, 0.0, 16383.0);
			SAADC::set_adc_value(static_cast<int16_t>(v));

			// --- let the detector run if its sample is due ---
			system_scheduler->run();

			// --- account new transitions ---
			while (transitions_seen < transitions.size()) {
				const auto &tr = transitions[transitions_seen++];
				if (!tr.second) {  // became "surface"
					if (truth_wet) {
						ds.false_surfacings++;
					} else if (!seg_transition_seen) {
						seg_transition_seen = true;
						seg_latency_s = static_cast<double>(tr.first - seg_start) / 1000.0;
					}
					ds.surface_episodes++;
					episode_open = true;
					episode_start_ms = tr.first;
					episode_day = day;
				} else if (episode_open) {
					if (tr.first - episode_start_ms < 3000) result.days[episode_day].short_episodes++;
					episode_open = false;
				}
			}

			// --- time accounting ---
			const double dt_s = step_ms / 1000.0;
			if (!truth_wet) ds.true_dry_s += dt_s;
			if (have_belief && !believed_uw) {
				ds.believed_surface_s += dt_s;
				if (!truth_wet) {
					ds.overlap_s += dt_s;
					seg_covered = true;
				}
			}

			// --- end of day snapshot ---
			if ((t_rel + step_ms) % day_ms == 0 || t_rel + step_ms >= total_ms) {
				SWSAnalogService::Diagnostics dg = SWSAnalogService::get_diagnostics();
				ds.dive_timeouts = dg.dive_timeout_count - diag_prev.dive_timeout_count;
				ds.force_surfaces = dg.force_surface_count - diag_prev.force_surface_count;
				diag_prev = dg;
				ds.status_end = SWSAnalogService::get_status();
			}

			fake_timer->increment_counter(step_ms);
		}

		DebugLogger::console_log = saved_console;
		s.stop();
		return result;
	}

	static double median(std::vector<double> v) {
		if (v.empty()) return -1.0;
		std::sort(v.begin(), v.end());
		return v[v.size() / 2];
	}

	void report(const Scenario &sc, const SimResult &r) {
		const char *out_dir = std::getenv("SWS_SIM_OUT");
		FILE *csv = nullptr;
		if (out_dir) {
			std::string path = std::string(out_dir) + "/" + sc.name + ".csv";
			csv = std::fopen(path.c_str(), "w");
			if (csv)
				std::fprintf(csv, "day,fouling,true_surfacings,detected_transitions,covered_segments,false_surfacings,"
				                  "surface_episodes,short_episodes,true_dry_s,believed_surface_s,overlap_s,"
				                  "median_latency_s,dive_timeouts,force_surfaces,air,water,threshold,delay_us,"
				                  "contrast_x10,uw_at_end\n");
		}
		std::printf("\n=== SWSLongRun %s (%zu days) | water %.0f k / air %.0f k clean, 1 ms -> water %.0f / air %.0f "
		            "counts ===\n",
		            sc.name, r.days.size(), sc.phys.r_water_clean / 1e3, sc.phys.r_air_clean / 1e3,
		            sc.phys.counts(sc.phys.r_water_clean, 1000.0), sc.phys.counts(sc.phys.r_air_clean, 1000.0));
		std::printf("day foul  surf  det  cov  false  epis short  dry_s  belief_s  ovl_s  lat_s  dto fsf   air  water  "
		            " thr  delay  ctr\n");
		for (const auto &d : r.days) {
			double lat = median(d.latencies_s);
			if (csv)
				std::fprintf(csv, "%u,%.3f,%u,%u,%u,%u,%u,%u,%.0f,%.0f,%.0f,%.1f,%u,%u,%u,%u,%u,%u,%u,%d\n", d.day,
				             d.fouling, d.true_surfacings, d.detected_transitions, d.covered_segments,
				             d.false_surfacings, d.surface_episodes, d.short_episodes, d.true_dry_s,
				             d.believed_surface_s, d.overlap_s, lat, d.dive_timeouts, d.force_surfaces,
				             d.status_end.threshold_air, d.status_end.threshold_water, d.status_end.threshold_current,
				             d.status_end.sample_delay_us, d.status_end.contrast_x10,
				             d.status_end.is_underwater ? 1 : 0);
			if (d.day % 10 == 0 || d.day + 1 == r.days.size())
				std::printf("%3u %.2f %5u %4u %4u %6u %5u %5u %6.0f %9.0f %6.0f %6.1f %4u %3u %5u %6u %5u %6u %4u\n",
				            d.day, d.fouling, d.true_surfacings, d.detected_transitions, d.covered_segments,
				            d.false_surfacings, d.surface_episodes, d.short_episodes, d.true_dry_s,
				            d.believed_surface_s, d.overlap_s, lat, d.dive_timeouts, d.force_surfaces,
				            d.status_end.threshold_air, d.status_end.threshold_water, d.status_end.threshold_current,
				            d.status_end.sample_delay_us, d.status_end.contrast_x10);
		}
		if (csv) std::fclose(csv);
	}

	static double detection_rate(const SimResult &r, unsigned from_day, unsigned to_day) {
		unsigned surf = 0, det = 0;
		for (unsigned d = from_day; d < to_day && d < r.days.size(); d++) {
			surf += r.days[d].true_surfacings;
			det += r.days[d].detected_transitions;
		}
		return surf ? static_cast<double>(det) / surf : 0.0;
	}

	void summary(const char *name, const SimResult &r) {
		unsigned n = static_cast<unsigned>(r.days.size());
		unsigned a = n / 3, b = 2 * n / 3;
		std::printf("%s: detection rate days 0-%u = %.2f, %u-%u = %.2f, %u-%u = %.2f\n", name, a,
		            detection_rate(r, 0, a), a, b, detection_rate(r, a, b), b, n, detection_rate(r, b, n));
	}

	// Water hypotheses: H45 reproduces the "~3000 counts at 1 ms" seen on
	// boards; H10 is the wiki's "seawater ~10 kΩ, τ = 1 ms".
	static Physics physics_h45(double noise_clean = 0.01, double noise_fouled = 0.04) {
		Physics p;
		p.noise_clean = noise_clean;
		p.noise_fouled = noise_fouled;
		return p;
	}
	static Physics physics_h10(double noise_clean = 0.01, double noise_fouled = 0.04) {
		Physics p = physics_h45(noise_clean, noise_fouled);
		p.r_water_clean = 10.0e3;
		p.r_water_fouled = 15.0e3;
		return p;
	}
};

// Deployed configuration (WWF template, build 2026-05-25): adaptive delay
// 200-5000 µs, max dive time 7200 s. Biofouling reaches the fouled values
// around day 60-75 (warm tropical water); conductance, film retention and
// contact noise all follow it.
static const Scenario kFoulingH45{ "h45_deployed_fouling",
                                   TEST_GROUP_CppUTestGroupSWSLongRun::physics_h45(),
                                   55.0,
                                   8.0,
                                   1.0,
                                   true,
                                   true,
                                   true,
                                   200U,
                                   1000U,
                                   7200U };
static const Scenario kFoulingH10{ "h10_deployed_fouling",
                                   TEST_GROUP_CppUTestGroupSWSLongRun::physics_h10(),
                                   55.0,
                                   8.0,
                                   1.0,
                                   true,
                                   true,
                                   true,
                                   200U,
                                   1000U,
                                   7200U };

// Clean electrodes for the whole deployment, adaptive delay as deployed.
static const Scenario kCleanH45{ "h45_deployed_clean",
                                 TEST_GROUP_CppUTestGroupSWSLongRun::physics_h45(),
                                 55.0,
                                 8.0,
                                 0.0,
                                 false,
                                 false,
                                 false,
                                 200U,
                                 1000U,
                                 7200U };
static const Scenario kCleanH10{ "h10_deployed_clean",
                                 TEST_GROUP_CppUTestGroupSWSLongRun::physics_h10(),
                                 55.0,
                                 8.0,
                                 0.0,
                                 false,
                                 false,
                                 false,
                                 200U,
                                 1000U,
                                 7200U };

// SHIPPED configuration, taken from the firmware defaults rather than guessed:
// config_store.hpp has SWS_DELAY_MIN_US = 200 and SWS_DELAY_MAX_US = 10000, and none
// of the three turtle templates overrides either. Every "deployed" scenario above uses
// a 200-1000 ceiling, so none of them is the configuration actually in the water --
// the real ceiling is ten times higher. These two close that gap.
static const Scenario kShippedCleanH45{ "h45_shipped_clean",
                                        TEST_GROUP_CppUTestGroupSWSLongRun::physics_h45(),
                                        55.0,
                                        8.0,
                                        0.0,
                                        false,
                                        false,
                                        false,
                                        200U,
                                        10000U,
                                        7200U };
static const Scenario kShippedFoulingH45{ "h45_shipped_fouling",
                                          TEST_GROUP_CppUTestGroupSWSLongRun::physics_h45(),
                                          55.0,
                                          8.0,
                                          1.0,
                                          true,
                                          true,
                                          true,
                                          200U,
                                          10000U,
                                          7200U };

// Charge delay pinned at 1 ms (no adaptation): isolates the adaptive-delay ratchet (R-T2).
static const Scenario kFixedCleanH45{ "h45_fixed_delay_clean",
                                      TEST_GROUP_CppUTestGroupSWSLongRun::physics_h45(),
                                      55.0,
                                      8.0,
                                      0.0,
                                      false,
                                      false,
                                      false,
                                      500U,
                                      500U,
                                      7200U };
static const Scenario kFixedFoulingH45{ "h45_fixed_delay_fouling",
                                        TEST_GROUP_CppUTestGroupSWSLongRun::physics_h45(),
                                        55.0,
                                        8.0,
                                        1.0,
                                        true,
                                        true,
                                        true,
                                        500U,
                                        500U,
                                        7200U };
static const Scenario kFixedFoulingH10{ "h10_fixed_delay_fouling",
                                        TEST_GROUP_CppUTestGroupSWSLongRun::physics_h10(),
                                        55.0,
                                        8.0,
                                        1.0,
                                        true,
                                        true,
                                        true,
                                        500U,
                                        500U,
                                        7200U };

// Conductance drift + film only, noise stays at the clean level (fixed delay).
static const Scenario kQuietFoulingH45{ "h45_fixed_delay_fouling_quiet",
                                        TEST_GROUP_CppUTestGroupSWSLongRun::physics_h45(),
                                        55.0,
                                        8.0,
                                        1.0,
                                        true,
                                        true,
                                        false,
                                        500U,
                                        500U,
                                        7200U };

// Noise growth only: electrodes keep their clean conductance and drainage (fixed delay).
static const Scenario kNoiseOnlyH45{ "h45_fixed_delay_noise_only",
                                     TEST_GROUP_CppUTestGroupSWSLongRun::physics_h45(),
                                     55.0,
                                     8.0,
                                     1.0,
                                     false,
                                     false,
                                     true,
                                     500U,
                                     500U,
                                     7200U };

// Low-noise variants: if real sample-to-sample noise at the 2 s cadence is
// well below 1 %, false L1 overrides are rare and the adaptive-delay trap
// (air inflated by false surfacings -> contrast < 5x -> delay to the floor ->
// baselines inconsistent) should develop over weeks instead of hours.
static const Scenario kCleanH45_n05{ "h45_deployed_clean_noise0.5",
                                     TEST_GROUP_CppUTestGroupSWSLongRun::physics_h45(0.005, 0.02),
                                     55.0,
                                     8.0,
                                     0.0,
                                     false,
                                     false,
                                     false,
                                     200U,
                                     1000U,
                                     7200U };
static const Scenario kFoulingH45_n05{ "h45_deployed_fouling_noise0.5",
                                       TEST_GROUP_CppUTestGroupSWSLongRun::physics_h45(0.005, 0.02),
                                       55.0,
                                       8.0,
                                       1.0,
                                       true,
                                       true,
                                       true,
                                       200U,
                                       1000U,
                                       7200U };
static const Scenario kFixedFoulingH45_n05{ "h45_fixed_delay_fouling_noise0.5",
                                            TEST_GROUP_CppUTestGroupSWSLongRun::physics_h45(0.005, 0.02),
                                            55.0,
                                            8.0,
                                            1.0,
                                            true,
                                            true,
                                            true,
                                            500U,
                                            500U,
                                            7200U };
static const Scenario kCleanH45_n03{ "h45_deployed_clean_noise0.3",
                                     TEST_GROUP_CppUTestGroupSWSLongRun::physics_h45(0.003, 0.012),
                                     55.0,
                                     8.0,
                                     0.0,
                                     false,
                                     false,
                                     false,
                                     200U,
                                     1000U,
                                     7200U };
static const Scenario kFoulingH45_n03{ "h45_deployed_fouling_noise0.3",
                                       TEST_GROUP_CppUTestGroupSWSLongRun::physics_h45(0.003, 0.012),
                                       55.0,
                                       8.0,
                                       1.0,
                                       true,
                                       true,
                                       true,
                                       200U,
                                       1000U,
                                       7200U };
static const Scenario kFixedFoulingH45_n03{ "h45_fixed_delay_fouling_noise0.3",
                                            TEST_GROUP_CppUTestGroupSWSLongRun::physics_h45(0.003, 0.012),
                                            55.0,
                                            8.0,
                                            1.0,
                                            true,
                                            true,
                                            true,
                                            500U,
                                            500U,
                                            7200U };

TEST(SWSLongRun, H45_DeployedCleanNoise05_Documentary) {
	SimResult r = run_scenario(kCleanH45_n05);
	report(kCleanH45_n05, r);
	summary(kCleanH45_n05.name, r);
}
TEST(SWSLongRun, H45_DeployedFoulingNoise05_Documentary) {
	SimResult r = run_scenario(kFoulingH45_n05);
	report(kFoulingH45_n05, r);
	summary(kFoulingH45_n05.name, r);
}
TEST(SWSLongRun, H45_FixedDelayFoulingNoise05_Documentary) {
	SimResult r = run_scenario(kFixedFoulingH45_n05);
	report(kFixedFoulingH45_n05, r);
	summary(kFixedFoulingH45_n05.name, r);
}
TEST(SWSLongRun, H45_DeployedCleanNoise03_Documentary) {
	SimResult r = run_scenario(kCleanH45_n03);
	report(kCleanH45_n03, r);
	summary(kCleanH45_n03.name, r);
}
TEST(SWSLongRun, H45_DeployedFoulingNoise03_Documentary) {
	SimResult r = run_scenario(kFoulingH45_n03);
	report(kFoulingH45_n03, r);
	summary(kFoulingH45_n03.name, r);
}
TEST(SWSLongRun, H45_FixedDelayFoulingNoise03_Documentary) {
	SimResult r = run_scenario(kFixedFoulingH45_n03);
	report(kFixedFoulingH45_n03, r);
	summary(kFixedFoulingH45_n03.name, r);
}

TEST(SWSLongRun, H45_FixedDelayClean_ControlDetectsSurfacings) {
	SimResult r = run_scenario(kFixedCleanH45);
	report(kFixedCleanH45, r);
	summary(kFixedCleanH45.name, r);
	double rate = detection_rate(r, 0, static_cast<unsigned>(r.days.size()));
	CHECK_TEXT(rate >= 0.90, "clean electrodes, fixed delay: the detector must see almost every surfacing");
}

TEST(SWSLongRun, H45_DeployedClean_Documentary) {
	SimResult r = run_scenario(kCleanH45);
	report(kCleanH45, r);
	summary(kCleanH45.name, r);
}

TEST(SWSLongRun, H10_DeployedClean_Documentary) {
	SimResult r = run_scenario(kCleanH10);
	report(kCleanH10, r);
	summary(kCleanH10.name, r);
}

TEST(SWSLongRun, H45_DeployedFouling_Documentary) {
	SimResult r = run_scenario(kFoulingH45);
	report(kFoulingH45, r);
	summary(kFoulingH45.name, r);
}

TEST(SWSLongRun, H10_DeployedFouling_Documentary) {
	SimResult r = run_scenario(kFoulingH10);
	report(kFoulingH10, r);
	summary(kFoulingH10.name, r);
}

TEST(SWSLongRun, H45_FixedDelayFouling_Documentary) {
	SimResult r = run_scenario(kFixedFoulingH45);
	report(kFixedFoulingH45, r);
	summary(kFixedFoulingH45.name, r);
}

TEST(SWSLongRun, H10_FixedDelayFouling_Documentary) {
	SimResult r = run_scenario(kFixedFoulingH10);
	report(kFixedFoulingH10, r);
	summary(kFixedFoulingH10.name, r);
}

TEST(SWSLongRun, H45_FixedDelayFoulingQuiet_Documentary) {
	SimResult r = run_scenario(kQuietFoulingH45);
	report(kQuietFoulingH45, r);
	summary(kQuietFoulingH45.name, r);
}

TEST(SWSLongRun, H45_FixedDelayNoiseOnly_Documentary) {
	SimResult r = run_scenario(kNoiseOnlyH45);
	report(kNoiseOnlyH45, r);
	summary(kNoiseOnlyH45.name, r);
}

TEST(SWSLongRun, H45_ShippedClean_Documentary) {
	SimResult r = run_scenario(kShippedCleanH45);
	report(kShippedCleanH45, r);
	summary(kShippedCleanH45.name, r);
}
TEST(SWSLongRun, H45_ShippedFouling_Documentary) {
	SimResult r = run_scenario(kShippedFoulingH45);
	report(kShippedFoulingH45, r);
	summary(kShippedFoulingH45.name, r);
}
