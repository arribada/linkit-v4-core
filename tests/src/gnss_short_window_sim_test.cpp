/**
 * @file gnss_short_window_sim_test.cpp
 * @brief Scores candidate GNSS configurations over SHORT surfacing windows.
 *
 * The question this answers is not "how fast is the receiver" but "how much of
 * the window does the receiver ever get". In Argos SURFACING_BURST on a turtle,
 * a surfacing can be a handful of seconds; the firmware spends a fixed,
 * configuration-dependent prologue talking to the SAM-M10Q before it is
 * listening at all, and whatever is left of the window is the only sky the
 * receiver sees. A configuration that shortens the prologue converts directly
 * into captured positions, and that conversion is what is counted here.
 *
 * WHAT IS EXACT AND WHAT IS ASSUMED — read this before quoting any number.
 *
 *   EXACT (taken from the firmware, by reading it; each constant names its
 *   source). The fixed `run_state_machine()` / `PMU::delay_ms()` waits on the
 *   power-on path, and the NUMBER of UART request/response round-trips each
 *   configuration costs. These are counts and literals, not measurements.
 *
 *   ASSUMED (named *_ASSUMED, swept, never reported as fact). Two things:
 *   the wall-clock cost of ONE round-trip, and the receiver's own
 *   time-to-first-output. Nobody has measured either on this board; the bench
 *   campaign that would (Lot E) has not been run.
 *
 * Because the round-trip COUNTS are exact and only their unit price is unknown,
 * the ORDERING of configurations that differ in round-trip count is robust: it
 * holds for every unit price. The test at the bottom asserts exactly that and
 * nothing stronger. The absolute hit rates printed by the documentary cases are
 * model output, and are worth only as much as the assumptions listed above.
 *
 *   cd tests && ./build/TrackerTests -g GNSSShortWindow -v
 *   cd tests && GNSS_SW_OUT=/tmp ./build/TrackerTests -g GNSSShortWindow
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

namespace {

// ─────────────────────────────────────────────────────────────────────────────
//  EXACT — the fixed waits on the power-on path.
//  ports/nrf52840/core/hardware/m10qasync/m10qasync.cpp
// ─────────────────────────────────────────────────────────────────────────────

/// exit_shutdown(): 2 ms EXTINT settle + 20 ms VDD ramp under reset + 80 ms for
/// the M10Q to boot after NRST release. Paid once per rail-up.
constexpr double RAIL_UP_FIXED_MS = 2.0 + 20.0 + 80.0;

/// power_on(), backupidle branch: the delay after pulse_extint_wake() for the
/// receiver to leave PMREQ-backup ("datasheet ~30 ms", 50 ms taken).
constexpr double EXTINT_WAKE_FIXED_MS = 50.0;

/// state_configure, the three unconditional waits of the FULL sequence:
/// step 0 -> 1000 ms for the UART reconfiguration to apply, step 3 -> 500 ms
/// for the constellation config, step 6 -> 1000 ms after the soft reset. The
/// BBR fast path jumps 0 -> 7 and pays none of them.
constexpr double CONFIGURE_FULL_FIXED_MS = 1000.0 + 500.0 + 1000.0;

// ─────────────────────────────────────────────────────────────────────────────
//  EXACT — round-trip counts, by reading each state's step list.
// ─────────────────────────────────────────────────────────────────────────────

/// state_poweron: sync_baud_rate at the first probed baud. On a board with no
/// V_BCKP cell the rail really did drop, so the receiver is back at 9600, which
/// is the first entry of BOOT_BAUD_TABLE — one probe, no failed ones. (This is
/// why GNSS_HAS_BACKUP_BATTERY must stay 0 on such a board: the flag only
/// seeds the probe order, and a wrong seed pays failed probes first.)
constexpr unsigned int RT_BOOT_SYNC = 1;

/// state_configure steps 0,1,2,4 — setup_uart_port, sync_baud_rate at
/// MAX_BAUDRATE, setup_gnss_channel_sharing, save_config. (Step 5's soft_reset
/// has no response; steps 3 and 6 are pure waits.) These four are what the BBR
/// fast path replaces.
constexpr unsigned int RT_CONFIGURE_PROLOGUE = 4;

/// state_configure fast path: FAST_VALIDATE_BAUD. FAST_BBR_VALIDATED and
/// FAST_DONE are internal bookkeeping with no message.
constexpr unsigned int RT_CONFIGURE_FAST_PROLOGUE = 1;

/// state_configure fast path, FAST_REAPPLY_CONSTELLATIONS: paid only when the
/// mask wanted this session differs from what the receiver is known to hold.
constexpr unsigned int RT_CONFIGURE_FAST_REAPPLY = 1;

/// state_configure steps 7..16, paid by BOTH paths: disable_odometer,
/// disable_timepulse_output, setup_continuous_mode,
/// setup_simple_navigation_settings, setup_expert_navigation_settings,
/// query_mon_ver, query_sec_uniqid (7), plus supply_time_assistance and
/// supply_position_assistance when the RTC is set and a last fix exists (2).
/// Step 9 is skipped in code.
constexpr unsigned int RT_CONFIGURE_TAIL = 7;
constexpr unsigned int RT_CONFIGURE_TAIL_ASSIST = 2;

/// state_startreceive: NAV-PVT, NAV-DOP, NAV-STATUS, and NAV-SAT when
/// sat_tracking is on (gps_service sets it unconditionally).
constexpr unsigned int RT_STARTRECEIVE_NAV = 4;

// ─────────────────────────────────────────────────────────────────────────────
//  ASSUMED — swept. Not measured on this board by anyone.
// ─────────────────────────────────────────────────────────────────────────────

/// Wall-clock cost of one request + ACK against the M10Q, including the
/// scheduler hop the state machine takes between steps. MEASURE THIS (Lot E,
/// E8/E12): it is the single multiplier on every count above.
const double ROUNDTRIP_MS_SWEEP[] = { 20.0, 50.0, 100.0, 200.0 };

/// Assistance upload, when it happens: ANO selected for the day plus the
/// persisted DBD, their wire time at 460800 baud and the MGA-ACK polling after
/// them (state_sendofflinedatabase polls every 50 ms, capped at 500 ms).
/// MEASURE THIS. Only the ORDER of magnitude is argued here: ~7 kB of ANO is
/// ~125 ms of wire time, and the ACK wait dominates.
constexpr double ASSISTANCE_UPLOAD_ASSUMED_MS = 700.0;

/// Receiver time-to-first-output, from the instant it is listening. Two
/// different products, and the distinction is the whole point of CloudLocate:
/// a compact RXM-MEAS snapshot needs signals, not a position fix, so it lands
/// far sooner than a PVT. Lognormal, because a latency bounded below with a
/// long tail is what acquisition looks like; sigma is a guess.
/// MEASURE THESE (Lot E, E2/E3).
struct SkyModel {
	const char *name;
	double meas_median_s;  ///< to the first usable compact RXM-MEAS
	double meas_sigma;
	double fix_median_s;  ///< to the first PVT passing the configured filters
	double fix_sigma;
};

/// Three sky models spanning what is plausible, so no conclusion rests on one
/// guess. "assisted" = ANO/DBD applied and still fresh; "unassisted" = the
/// receiver starts from nothing, which is what a rail cut on a board with no
/// V_BCKP cell produces every session unless the persisted DBD is replayed.
constexpr SkyModel SKY_ASSISTED = { "assisted", 3.0, 0.5, 6.0, 0.6 };
constexpr SkyModel SKY_MIDDLING = { "middling", 6.0, 0.6, 20.0, 0.7 };
constexpr SkyModel SKY_UNASSISTED = { "unassisted", 12.0, 0.7, 45.0, 0.8 };

// ─────────────────────────────────────────────────────────────────────────────
//  The surfacing-window distribution.
// ─────────────────────────────────────────────────────────────────────────────

/// Lognormal, clamped. The shape is taken from tests/src/sws_longrun_sim_test.cpp,
/// which models olive-ridley inter-nesting surfacings as lognormal(ln 90, 0.4)
/// seconds clamped to [30, 300]. That is the LONG-window case and is kept here
/// as a control; the short cases re-parameterise the median downwards and drop
/// the 30 s floor, which is the regime this file exists for.
///
/// NOT MEASURED. The plan this work follows records that no real distribution of
/// surfacing durations exists for these animals, only a lower bound (at least
/// 40 % of surfacings last 30 s or more). Every median below is therefore a
/// scenario, not a finding.
struct WindowModel {
	const char *name;
	double median_s;
	double sigma;
	double min_s;
	double max_s;
};

constexpr WindowModel WIN_VERY_SHORT = { "very-short-8s", 8.0, 0.6, 2.0, 60.0 };
constexpr WindowModel WIN_SHORT = { "short-15s", 15.0, 0.6, 3.0, 90.0 };
constexpr WindowModel WIN_MEDIUM = { "medium-30s", 30.0, 0.5, 5.0, 150.0 };
constexpr WindowModel WIN_SWSLONGRUN = { "swslongrun-90s", 90.0, 0.4, 30.0, 300.0 };

// ─────────────────────────────────────────────────────────────────────────────
//  A candidate configuration, in the terms the operator actually sets.
// ─────────────────────────────────────────────────────────────────────────────

struct Candidate {
	const char *name;
	bool deep_idle;           ///< GNP52 != 0 — rail kept up, receiver woken by EXTINT
	bool bbr_retained;        ///< receiver still holds its config (implied by deep_idle here)
	bool upload_assistance;   ///< ANO/DBD re-sent this session
	bool cloudlocate;         ///< GNSS_FASTLOC_MODE = 2
	unsigned int cl_formats;  ///< MSGOUT enables for compact formats: 3 before this work, 1 after
	bool cloudlocate_only;    ///< GNP53 — end the session on the first capture
	bool rtc_set;             ///< an operator-set or GNSS-synced clock exists
	bool last_fix_known;      ///< a previous valid position exists for MGA-INI-POS
	bool mask_differs;        ///< the session's constellation mask differs from the receiver's
};

/// Fixed, configuration-independent-of-sky prologue: everything between the
/// decision to power on and the receiver listening.
double prologue_ms(const Candidate &c, double rt_ms) {
	double fixed = 0.0;
	unsigned int rt = 0;

	if (c.deep_idle) {
		fixed += EXTINT_WAKE_FIXED_MS;
		rt += RT_CONFIGURE_FAST_PROLOGUE;
		if (c.mask_differs) rt += RT_CONFIGURE_FAST_REAPPLY;
	} else {
		fixed += RAIL_UP_FIXED_MS + CONFIGURE_FULL_FIXED_MS;
		rt += RT_BOOT_SYNC + RT_CONFIGURE_PROLOGUE;
	}

	// Paid by both paths: the fast path rejoins the normal sequence at step 7.
	rt += RT_CONFIGURE_TAIL;
	if (c.rtc_set) rt += 1;         // supply_time_assistance
	if (c.last_fix_known) rt += 1;  // supply_position_assistance

	if (c.upload_assistance) fixed += ASSISTANCE_UPLOAD_ASSUMED_MS;

	rt += RT_STARTRECEIVE_NAV;
	if (c.cloudlocate) rt += c.cl_formats;

	return fixed + rt_ms * (double)rt;
}

/// Round-trip count alone, for the ordering assertion (independent of price).
unsigned int prologue_roundtrips(const Candidate &c) {
	unsigned int rt = c.deep_idle ? RT_CONFIGURE_FAST_PROLOGUE : (RT_BOOT_SYNC + RT_CONFIGURE_PROLOGUE);
	if (c.deep_idle && c.mask_differs) rt += RT_CONFIGURE_FAST_REAPPLY;
	rt += RT_CONFIGURE_TAIL;
	if (c.rtc_set) rt += 1;
	if (c.last_fix_known) rt += 1;
	rt += RT_STARTRECEIVE_NAV;
	if (c.cloudlocate) rt += c.cl_formats;
	return rt;
}

struct Score {
	unsigned int surfacings = 0;
	unsigned int no_sky_at_all = 0;  ///< prologue outlasted the whole window
	unsigned int captures = 0;       ///< a usable compact RXM-MEAS landed
	unsigned int fixes = 0;          ///< a PVT landed
	unsigned int usable = 0;         ///< either — "something came back from this surfacing"
	double sky_s_total = 0.0;        ///< seconds of listening actually obtained
	double prologue_s = 0.0;
};

double pct(unsigned int n, unsigned int d) {
	return (d == 0) ? 0.0 : 100.0 * (double)n / (double)d;
}

Score run_case(const Candidate &c, const WindowModel &w, const SkyModel &sky, double rt_ms, unsigned int n_surfacings,
               uint64_t seed) {
	// One engine per stream so that changing the sky model cannot reshuffle the
	// window draws — otherwise two candidates are not compared on the same
	// surfacings, and the whole comparison is noise.
	std::mt19937_64 rng_win(seed);
	std::mt19937_64 rng_meas(seed ^ 0x9E3779B97F4A7C15ULL);
	std::mt19937_64 rng_fix(seed ^ 0xD1B54A32D192ED03ULL);

	std::lognormal_distribution<double> d_win(std::log(w.median_s), w.sigma);
	std::lognormal_distribution<double> d_meas(std::log(sky.meas_median_s), sky.meas_sigma);
	std::lognormal_distribution<double> d_fix(std::log(sky.fix_median_s), sky.fix_sigma);

	const double prologue_s = prologue_ms(c, rt_ms) / 1000.0;

	Score s;
	s.prologue_s = prologue_s;
	for (unsigned int i = 0; i < n_surfacings; i++) {
		s.surfacings++;
		const double window_s = std::clamp(d_win(rng_win), w.min_s, w.max_s);

		// Draw both latencies for every surfacing, whether or not they are used,
		// so the two RNG streams stay aligned across candidates.
		const double t_meas = d_meas(rng_meas);
		const double t_fix = d_fix(rng_fix);

		const double sky_s = window_s - prologue_s;
		if (sky_s <= 0.0) {
			s.no_sky_at_all++;
			continue;
		}
		s.sky_s_total += sky_s;

		const bool got_meas = c.cloudlocate && (t_meas <= sky_s);

		// GNSS_CLOUDLOCATE_ONLY ends the session at the first capture, so no PVT
		// can follow it. Without it the session keeps running and a fix may still
		// land inside the same window.
		const bool got_fix = (!(c.cloudlocate_only && got_meas)) && (t_fix <= sky_s);

		if (got_meas) s.captures++;
		if (got_fix) s.fixes++;
		if (got_meas || got_fix) s.usable++;
	}
	return s;
}

unsigned int sim_surfacings() {
	const char *e = std::getenv("GNSS_SW_SURFACINGS");
	if (!e) return 4000;
	long v = std::strtol(e, nullptr, 10);
	return (v > 0 && v <= 200000) ? (unsigned int)v : 4000;
}

/// The candidates, in the order they are reported.
///
///  - `current-template` is turtle_cloudlocate.cfg as committed: GNP52 absent
///    (default 0, so the rail is cut and every session is a full cold
///    configuration), all three compact formats enabled, assistance re-uploaded.
///  - `one-format` adds only the change that asks the receiver for the single
///    configured format instead of all three.
///  - `deep-idle` adds GNP52 != 0, so the receiver is woken in place. THIS IS THE
///    RECOMMENDED ONE.
///  - `deep-idle+cl-only` adds GNP53. Listed to show what it does and does not
///    buy, not as an improvement — see CloudLocateOnlyBuysNoExtraCaptures.
///  - `no-assist-upload` additionally skips re-uploading ANO/DBD when the
///    receiver's BBR survived. NOT IMPLEMENTED in firmware — it is here to size
///    the prize before anyone writes it.
std::vector<Candidate> candidates() {
	Candidate base{};
	base.name = "current-template";
	base.deep_idle = false;
	base.bbr_retained = false;
	base.upload_assistance = true;
	base.cloudlocate = true;
	base.cl_formats = 3;
	base.cloudlocate_only = false;
	base.rtc_set = true;
	base.last_fix_known = true;
	base.mask_differs = false;

	Candidate one_format = base;
	one_format.name = "one-format";
	one_format.cl_formats = 1;

	Candidate deep = one_format;
	deep.name = "deep-idle";
	deep.deep_idle = true;
	deep.bbr_retained = true;

	Candidate deep_only = deep;
	deep_only.name = "deep-idle+cl-only";
	deep_only.cloudlocate_only = true;

	Candidate deep_noassist = deep_only;
	deep_noassist.name = "no-assist-upload(*)";
	deep_noassist.upload_assistance = false;

	return { base, one_format, deep, deep_only, deep_noassist };
}

FILE *open_csv() {
	const char *dir = std::getenv("GNSS_SW_OUT");
	if (!dir) return nullptr;
	std::string path = std::string(dir) + "/gnss_short_window.csv";
	FILE *f = std::fopen(path.c_str(), "w");
	if (f) {
		std::fprintf(f, "candidate,window,sky,roundtrip_ms,roundtrips,prologue_s,surfacings,no_sky_pct,"
		                "capture_pct,fix_pct,usable_pct,mean_sky_s\n");
		std::printf("  CSV -> %s\n", path.c_str());
	}
	return f;
}

void report(const WindowModel &w, const SkyModel &sky, double rt_ms, FILE *csv) {
	const unsigned int n = sim_surfacings();
	std::printf("\n  window=%s  sky=%s  roundtrip=%.0f ms  n=%u\n", w.name, sky.name, rt_ms, n);
	std::printf("    %-20s %4s %8s %7s %8s %6s %8s\n", "candidate", "RT", "prologue", "no-sky", "capture", "fix",
	            "usable");
	for (const Candidate &c : candidates()) {
		const Score s = run_case(c, w, sky, rt_ms, n, 20261008ULL);
		std::printf("    %-20s %4u %7.2fs %6.1f%% %7.1f%% %5.1f%% %7.1f%%\n", c.name, prologue_roundtrips(c),
		            s.prologue_s, pct(s.no_sky_at_all, s.surfacings), pct(s.captures, s.surfacings),
		            pct(s.fixes, s.surfacings), pct(s.usable, s.surfacings));
		if (csv) {
			const double mean_sky =
			    (s.surfacings > s.no_sky_at_all) ? s.sky_s_total / (double)(s.surfacings - s.no_sky_at_all) : 0.0;
			std::fprintf(csv, "%s,%s,%s,%.0f,%u,%.3f,%u,%.2f,%.2f,%.2f,%.2f,%.2f\n", c.name, w.name, sky.name, rt_ms,
			             prologue_roundtrips(c), s.prologue_s, s.surfacings, pct(s.no_sky_at_all, s.surfacings),
			             pct(s.captures, s.surfacings), pct(s.fixes, s.surfacings), pct(s.usable, s.surfacings),
			             mean_sky);
		}
	}
}

}  // namespace

TEST_GROUP(GNSSShortWindow){ void setup(){ mock().ignoreOtherCalls();
}

void teardown() {
	mock().clear();
}
}
;

/// The prologue arithmetic, hand-checked, so a later edit to a constant cannot
/// quietly change what every other case in this file means.
TEST(GNSSShortWindow, PrologueArithmeticIsWhatTheCodeSays) {
	Candidate cold{};
	cold.name = "cold";
	cold.deep_idle = false;
	cold.upload_assistance = false;
	cold.cloudlocate = true;
	cold.cl_formats = 3;
	cold.rtc_set = true;
	cold.last_fix_known = true;

	// Fixed: 102 ms rail-up + 2500 ms of configure waits = 2602 ms.
	// Round-trips: 1 boot sync + 4 configure prologue + 7 tail + 2 assistance
	//              + 4 nav MSGOUT + 3 CloudLocate formats = 21.
	CHECK_EQUAL(21u, prologue_roundtrips(cold));
	DOUBLES_EQUAL(2602.0 + 21.0 * 50.0, prologue_ms(cold, 50.0), 0.001);

	// The deep-idle wake drops the 2602 ms to 50 ms and 5 of the round-trips
	// (1 boot sync + 4 configure prologue, replaced by 1 fast validation).
	Candidate warm = cold;
	warm.deep_idle = true;
	warm.bbr_retained = true;
	CHECK_EQUAL(17u, prologue_roundtrips(warm));
	DOUBLES_EQUAL(50.0 + 17.0 * 50.0, prologue_ms(warm, 50.0), 0.001);

	// Asking for one compact format instead of three removes exactly two.
	Candidate warm_one = warm;
	warm_one.cl_formats = 1;
	CHECK_EQUAL(15u, prologue_roundtrips(warm_one));
}

/// The claim that does NOT depend on the unmeasured unit price: the candidates
/// are listed in non-increasing order of prologue cost, for EVERY round-trip
/// price. A shorter prologue can only leave more sky, so this much of the
/// ranking is safe to act on whatever the bench eventually measures.
///
/// Non-increasing, not strictly decreasing: GNSS_CLOUDLOCATE_ONLY changes where
/// the session ENDS, not how it starts, so `deep-idle+cl-only` has exactly the
/// same prologue as `deep-idle`. That equality is the whole reason the parameter
/// buys no extra captures — see CloudLocateOnlyBuysNoExtraCaptures below.
TEST(GNSSShortWindow, RankingHoldsForEveryRoundTripPrice) {
	const std::vector<Candidate> cs = candidates();
	for (size_t i = 1; i < cs.size(); i++) {
		CHECK_COMPARE(prologue_roundtrips(cs[i]), <=, prologue_roundtrips(cs[i - 1]));
		for (double rt : ROUNDTRIP_MS_SWEEP) {
			CHECK_COMPARE(prologue_ms(cs[i], rt), <=, prologue_ms(cs[i - 1], rt));
		}
	}
	// And the two changes that DO shorten the prologue must each do so strictly,
	// at every price — that is the part worth configuring for.
	for (double rt : ROUNDTRIP_MS_SWEEP) {
		CHECK_COMPARE(prologue_ms(cs[1], rt), <, prologue_ms(cs[0], rt));  // one-format
		CHECK_COMPARE(prologue_ms(cs[2], rt), <, prologue_ms(cs[1], rt));  // deep-idle
	}
}

/// GNSS_CLOUDLOCATE_ONLY (GNP53) does NOT win a single extra surfacing, and
/// costs every PVT.
///
/// It is easy to assume the opposite — it is documented as the short-surface
/// parameter, and it does shorten the session. But it ends the session at the
/// FIRST raw measurement, which by definition has already been captured by then:
/// the capture count cannot change, and `usable` (capture OR fix) cannot change
/// either. What it removes is the PVT that would otherwise have landed later in
/// the same window. That PVT is not decoration: it is the only thing that
/// re-syncs the RTC and refreshes LAST_KNOWN_POS, and those two feed the ANO day
/// selection and MGA-INI-POS of every later session. Its real benefit is power
/// and time-on-surface, which this model does not score.
TEST(GNSSShortWindow, CloudLocateOnlyBuysNoExtraCaptures) {
	const std::vector<Candidate> cs = candidates();
	const Candidate &without = cs[2];  // deep-idle
	const Candidate &with = cs[3];     // deep-idle + cl-only
	const WindowModel windows[] = { WIN_VERY_SHORT, WIN_SHORT, WIN_MEDIUM, WIN_SWSLONGRUN };

	bool saw_a_fix_sacrificed = false;
	for (const WindowModel &w : windows) {
		const Score a = run_case(without, w, SKY_ASSISTED, 50.0, 2000, 20261008ULL);
		const Score b = run_case(with, w, SKY_ASSISTED, 50.0, 2000, 20261008ULL);
		CHECK_EQUAL(a.captures, b.captures);
		CHECK_EQUAL(a.usable, b.usable);
		CHECK_COMPARE(b.fixes, <=, a.fixes);
		if (b.fixes < a.fixes) saw_a_fix_sacrificed = true;
	}
	CHECK_TRUE_TEXT(saw_a_fix_sacrificed, "cl-only must be shown to cost fixes, or this test proves nothing");
}

/// The one quantitative gate. Across every combination of the swept unknowns,
/// the recommended configuration must never produce fewer usable surfacings
/// than the committed template. If a future change to the model or the constants
/// breaks this, the recommendation in the field notes is no longer supported.
TEST(GNSSShortWindow, RecommendedNeverLosesToCurrentTemplate) {
	const std::vector<Candidate> cs = candidates();
	const Candidate &current = cs.front();
	const Candidate &recommended = cs[2];  // deep-idle + one-format, CLOUDLOCATE_ONLY off
	const WindowModel windows[] = { WIN_VERY_SHORT, WIN_SHORT, WIN_MEDIUM, WIN_SWSLONGRUN };
	const SkyModel skies[] = { SKY_ASSISTED, SKY_MIDDLING, SKY_UNASSISTED };

	for (const WindowModel &w : windows) {
		for (const SkyModel &sky : skies) {
			for (double rt : ROUNDTRIP_MS_SWEEP) {
				const Score a = run_case(current, w, sky, rt, 2000, 20261008ULL);
				const Score b = run_case(recommended, w, sky, rt, 2000, 20261008ULL);
				CHECK_COMPARE(b.usable, >=, a.usable);
			}
		}
	}
}

/// Documentary: the full matrix. No assertions — this is the table to read
/// before choosing what to set, and to compare against tomorrow's measurements.
TEST(GNSSShortWindow, Matrix_Documentary) {
	FILE *csv = open_csv();
	std::printf("\n  ── GNSS short-window scoring ─────────────────────────────────────\n");
	std::printf("  RT = UART round-trips before the receiver listens (EXACT, counted in code).\n");
	std::printf("  prologue = fixed waits + RT x round-trip price (price ASSUMED, swept).\n");
	std::printf("  no-sky = surfacings the prologue outlasted entirely.\n");
	std::printf("  (*) no-assist-upload is NOT implemented in firmware — sizing only.\n");

	const WindowModel windows[] = { WIN_VERY_SHORT, WIN_SHORT, WIN_MEDIUM, WIN_SWSLONGRUN };
	for (const WindowModel &w : windows) {
		report(w, SKY_ASSISTED, 50.0, csv);
	}
	// Sensitivity: the two unknowns, on the window that matters most.
	for (double rt : ROUNDTRIP_MS_SWEEP) {
		report(WIN_SHORT, SKY_ASSISTED, rt, csv);
	}
	report(WIN_SHORT, SKY_MIDDLING, 50.0, csv);
	report(WIN_SHORT, SKY_UNASSISTED, 50.0, csv);

	if (csv) std::fclose(csv);
	CHECK_TRUE_TEXT(true, "documentary");
}
