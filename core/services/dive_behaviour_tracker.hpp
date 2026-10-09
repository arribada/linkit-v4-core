/**
 * @file dive_behaviour_tracker.hpp
 * @brief Turtle dive/surface behaviour summary for the DX1 Doppler extension.
 *
 * Fed by the single UW funnel in ServiceManager::notify_peer_event and by the
 * cooldown entry/exit, read by ArgosTxService when it builds a Doppler image.
 * All state is static RAM except the RST counter (noinit, defined in
 * service.cpp so every target that links the funnel also links the storage).
 *
 * Semantics are those of the reference model (class Tracker of dx1_ref.py,
 * DX1 v3); the scenario vectors in tests/src/dx1_doppler_test.cpp pin them.
 * Integer arithmetic only, and the same arithmetic as the model: time is
 * credited to a 6-h block as whole seconds per piece, the ring evicts ids
 * older than the block being created minus 4, and every "unknown" rule is the
 * model's. Change one and the ground decoder disagrees with the beacon.
 *
 * Latency contract: the surface edge (UW -> SURF) only stores a timestamp and
 * the counters it is handed. All accounting, the window and the latch happen
 * at the dive edge, i.e. underwater. No DEBUG line anywhere in here.
 */

#pragma once

#include <cstdint>

#include "pmu.hpp"

// Whole feature compiles out where there is no SWS analog sensor or no dive at
// all (RSPB / TPL5111 external-wakeup profile): the Doppler stays legacy there.
#if ENABLE_SWS_ANALOG && !defined(BOARD_RSPB) && !defined(EXTERNAL_WAKEUP)
#define DX1_DOPPLER_EXT_BUILD 1
#else
#define DX1_DOPPLER_EXT_BUILD 0
#endif

/// @brief Behaviour fields latched at the last dive edge, as RAW values.
/// ArgosPacketBuilder::dx1_encode_behaviour() turns them into codes. Kept
/// outside the build guard: it is plain data, and the packet builder compiles
/// on every target.
struct Dx1Behaviour {
	uint8_t latch = 0;        ///< 1 latched at a dive edge, 2 no dive edge since reset
	uint8_t blind = 0;        ///< blind interval overlaps the window or is in progress
	uint8_t partial = 0;      ///< fewer than 4 complete blocks since reset
	uint16_t surf_seq = 0;    ///< 0 none, 1..511
	uint32_t dry_s = 0;       ///< window sum; with obs_s < 12 h PCT and DRY are unknown
	uint32_t obs_s = 0;       ///< window sum
	bool n_known = false;     ///< false: N_SURF unknown (code 0)
	uint32_t n_surf = 0;      ///< counted surfacings in the window
	uint32_t s_last_s = 0;    ///< 0 unknown, else >= 1
	uint32_t d_prev_s = 0;    ///< 0 unknown, else >= 1
	uint32_t d_max_s = 0;     ///< 0 unknown or no known dive
	uint8_t surf_method = 0;  ///< 0 unknown, 1..5 L1..L5, 6 other / forced / ADC fallback
};

#if DX1_DOPPLER_EXT_BUILD

/// @brief One 6-h accounting block of the tracker ring (namespace scope: a
/// nested type with default member initialisers cannot be used by the static
/// inline members of its own enclosing class).
struct Dx1TrackerBlock {
	bool used = false;
	uint32_t id = 0;
	uint32_t dry_s = 0;
	uint32_t obs_s = 0;
	uint32_t n = 0;
	uint32_t dmax_s = 0;
	bool blind = false;
};

/// @brief RST counter record kept in .noinit across warm boots.
struct Dx1RstRecord {
	uint32_t magic;
	uint32_t count;
	uint32_t check;
};

class DiveBehaviourTracker {
public:
	static constexpr uint64_t BLOCK_MS = 21600000ULL;  ///< 6-h accounting block
	static constexpr unsigned int RING_SLOTS = 5;      ///< 4 completed blocks + the current one
	static constexpr unsigned int WINDOW_BLOCKS = 4;
	static constexpr uint16_t SURF_SEQ_MAX = 511;
	static constexpr uint8_t RST_MAX = 7;

	enum class State : uint8_t { UNKNOWN, UW, SURF };

	/// @brief Forget everything (boot, every entry into OperationalState). The
	/// first call after a boot also does the RST bookkeeping.
	static void reset() { reset(PMU::get_timestamp_ms()); }
	static void reset(uint64_t now_ms) {
		s_reset_ms = now_ms;
		s_state = State::UNKNOWN;
		s_blind = false;
		s_blind_start = 0;
		s_acct_from = now_ms;
		for (auto &b : s_ring)
			b = Block{};
		s_seq = 0;
		s_has_snap = false;
		s_dive_start = 0;
		s_dive_start_real = false;
		s_dive_blinded = false;
		s_dto0 = s_fsc0 = 0;
		s_t_s = 0;
		s_level_s = 0;
		s_dto_s = s_fsc_s = 0;
		s_adcf_s = false;
		s_surface_known = false;
		s_latched = false;
		s_latch = Dx1Behaviour{};
		if (!s_rst_done) {
			s_rst_done = true;
			update_rst_record();
		}
	}

	/// @brief UW_SENSOR SERVICE_LOG_UPDATED. Counters and flags are the SWS
	/// values at the event: Diagnostics dive_timeout/force_surface counts, the
	/// ADC-fault flag and Status.surface_level.
	static void on_uw_event(bool uw, uint64_t now, uint32_t dto, uint32_t fsc, bool adcf, uint8_t level) {
		if (s_blind) return;
		if (s_state == State::UNKNOWN) {
			// First event after a reset, a blind window or a test event: it
			// only sets the state, the episode it opens has an unknown start.
			s_state = uw ? State::UW : State::SURF;
			s_acct_from = now;
			if (uw) {
				s_has_snap = true;
				s_dive_start = now;
				s_dive_start_real = false;
				s_dive_blinded = false;
				s_dto0 = dto;
				s_fsc0 = fsc;
			} else {
				s_t_s = now;
				s_level_s = 0;
				s_dto_s = dto;
				s_fsc_s = fsc;
				s_adcf_s = adcf;
				s_surface_known = false;
			}
			return;
		}
		if ((s_state == State::UW) == uw) return;  // re-emission, not an edge
		if (!uw) {
			// UW -> SURF: STORE ONLY. This is the surfacing critical path.
			s_state = State::SURF;
			s_t_s = now;
			s_level_s = level;
			s_dto_s = dto;
			s_fsc_s = fsc;
			s_adcf_s = adcf;
			s_surface_known = true;
			return;
		}
		dive_edge(now, dto, fsc);
	}

	/// @brief UW event while SWS test mode runs: ignored, state lost, window
	/// flagged BLIND.
	static void on_test_event(uint64_t now) {
		close_open_episode(now);
		s_state = State::UNKNOWN;
		s_dive_blinded = true;
		mark_blind(now, now);
		s_acct_from = now;
	}

	/// @brief The SWS gate starts refusing (cooldown armed with interval > 0).
	static void on_blind_start(uint64_t now) {
		if (s_blind) return;
		close_open_episode(now);
		s_blind = true;
		s_blind_start = now;
		s_dive_blinded = true;
		mark_blind(now, now);
	}

	/// @brief Cooldown exit: the SWS restarts and re-emits its state, which
	/// must not count as an edge. No-op without a matching start.
	static void on_blind_end(uint64_t now) {
		if (!s_blind) return;
		mark_blind(s_blind_start, now);
		s_blind = false;
		s_state = State::UNKNOWN;
		s_acct_from = now;
	}

	/// @brief DEND: how the dive that ended at the current surface (or the one
	/// in progress) ended. 0 unknown, 1 detected surfacing, 2 at least one
	/// dive timeout, 3 forced surfacing or ADC fault. RAM only.
	static unsigned int dive_end(uint32_t dto_now, uint32_t fsc_now, bool adcf_now) {
		if (s_state == State::UNKNOWN || !s_has_snap) return 0;
		int64_t dd, fd;
		bool af;
		if (s_state == State::UW) {
			dd = (int64_t)dto_now - (int64_t)s_dto0;
			fd = (int64_t)fsc_now - (int64_t)s_fsc0;
			af = adcf_now;
		} else {
			if (!s_surface_known) return 0;
			dd = (int64_t)s_dto_s - (int64_t)s_dto0;
			fd = (int64_t)s_fsc_s - (int64_t)s_fsc0;
			af = s_adcf_s;
		}
		if (dd < 0 || fd < 0) return 0;  // cold boot or DTE clear since the snapshot
		if (fd > 0 || af) return 3;
		if (dd > 0) return 2;
		return 1;
	}

	/// @brief Behaviour fields of the last dive edge, or the "no dive edge
	/// since reset" record (LATCH 2) before the first one.
	static Dx1Behaviour latch(uint64_t now) {
		if (s_latched) return s_latch;
		Dx1Behaviour l;
		l.latch = 2;
		bool blind = s_blind;
		for (const auto &b : s_ring) {
			if (b.used && b.blind) blind = true;
		}
		l.blind = blind ? 1 : 0;
		l.partial = (bid(now) < WINDOW_BLOCKS) ? 1 : 0;
		l.surf_seq = s_seq;
		return l;
	}

	/// @brief Warm boots since the last power-on that reached Operational.
	static uint8_t rst_count() { return s_rst_valid ? s_rst.count : 0; }

#ifdef CPPUTEST
	/// Test-only: behave as if the next reset() were the first after a power-on
	/// (noinit garbage) or after a warm boot (noinit kept).
	static void test_simulate_boot(bool power_on) {
		s_rst_done = false;
		s_rst_valid = false;
		if (power_on) s_rst = RstRecord{ 0, 0xFF, 0 };
	}
	static State test_state() { return s_state; }
#endif

private:
	using Block = Dx1TrackerBlock;
	using RstRecord = Dx1RstRecord;
	static constexpr uint32_t RST_MAGIC = 0x44583152U;  // "DX1R"
	static uint32_t rst_check(uint32_t magic, uint32_t count) { return ~(magic ^ (count * 0x9E3779B1U)); }
	static void update_rst_record() {
		bool valid =
		    s_rst.magic == RST_MAGIC && s_rst.count <= RST_MAX && s_rst.check == rst_check(s_rst.magic, s_rst.count);
		uint32_t count = valid ? ((s_rst.count < RST_MAX) ? s_rst.count + 1U : (uint32_t)RST_MAX) : 0U;
		s_rst.magic = RST_MAGIC;
		s_rst.count = count;
		s_rst.check = rst_check(RST_MAGIC, count);
		s_rst_valid = true;
	}

	static uint32_t bid(uint64_t t) { return (uint32_t)(((t > s_reset_ms) ? (t - s_reset_ms) : 0ULL) / BLOCK_MS); }

	static Block *find(uint32_t id) {
		for (auto &b : s_ring) {
			if (b.used && b.id == id) return &b;
		}
		return nullptr;
	}

	/// Block @p id, created on first use; creating it evicts every id older
	/// than id - 4 (the model's dict semantics, bounded at 5 slots).
	static Block &blk(uint32_t id) {
		if (Block *b = find(id)) return *b;
		Block *slot = nullptr;
		for (auto &b : s_ring) {
			if (b.used && b.id + WINDOW_BLOCKS < id) b = Block{};
		}
		for (auto &b : s_ring) {
			if (!b.used) {
				slot = &b;
				break;
			}
		}
		if (!slot) {
			// Unreachable while blocks are created in time order; keep the
			// ring bounded anyway by recycling the oldest id.
			slot = &s_ring[0];
			for (auto &b : s_ring) {
				if (b.id < slot->id) slot = &b;
			}
		}
		*slot = Block{};
		slot->used = true;
		slot->id = id;
		return *slot;
	}

	/// Add [t0, t1) to obs (and dry), split across blocks, whole seconds per piece.
	static void account(uint64_t t0, uint64_t t1, bool dry) {
		uint64_t t = t0;
		while (t < t1) {
			uint32_t i = bid(t);
			uint64_t block_end = s_reset_ms + ((uint64_t)i + 1ULL) * BLOCK_MS;
			uint64_t end = (t1 < block_end) ? t1 : block_end;
			uint32_t s = (uint32_t)((end - t) / 1000ULL);
			Block &b = blk(i);
			b.obs_s += s;
			if (dry) b.dry_s += s;
			t = end;
		}
	}

	static void mark_blind(uint64_t t0, uint64_t t1) {
		uint32_t last = bid((t1 > t0) ? t1 : t0);
		for (uint32_t i = bid(t0); i <= last; i++)
			blk(i).blind = true;
	}

	static bool surface_forced() { return (s_has_snap && s_fsc_s > s_fsc0) || s_adcf_s; }

	static void close_open_episode(uint64_t now) {
		if (s_state == State::UW) {
			account(s_acct_from, now, false);
		} else if (s_state == State::SURF) {
			if (s_acct_from < s_t_s) account(s_acct_from, s_t_s, false);
			if (!surface_forced()) account((s_acct_from > s_t_s) ? s_acct_from : s_t_s, now, true);
		}
		s_acct_from = now;
	}

	/// SURF -> UW: all the work happens here, underwater.
	static void dive_edge(uint64_t now, uint32_t dto, uint32_t fsc) {
		const bool forced = surface_forced();
		const bool surface_real = s_surface_known && !forced;
		if (s_acct_from < s_t_s) account(s_acct_from, s_t_s, false);
		if (surface_real) account((s_acct_from > s_t_s) ? s_acct_from : s_t_s, now, true);

		Dx1Behaviour l;
		const bool dive_known = s_surface_known && s_has_snap && s_dive_start_real && !s_dive_blinded;
		if (dive_known) {
			uint64_t dive_s = (s_t_s > s_dive_start) ? (s_t_s - s_dive_start) / 1000ULL : 0ULL;
			uint32_t d = (uint32_t)((dive_s < 1ULL) ? 1ULL : (dive_s > 0xFFFFFFFFULL ? 0xFFFFFFFFULL : dive_s));
			Block &b = blk(bid(s_t_s));
			if (d > b.dmax_s) b.dmax_s = d;
			l.d_prev_s = d;
		}
		if (surface_real) {
			s_seq = (uint16_t)(s_seq % SURF_SEQ_MAX + 1U);
			blk(bid(now)).n++;
			uint64_t sl = (now - s_t_s) / 1000ULL;
			l.s_last_s = (uint32_t)((sl < 1ULL) ? 1ULL : (sl > 0xFFFFFFFFULL ? 0xFFFFFFFFULL : sl));
		}
		if (!s_surface_known) {
			l.surf_method = 0;
		} else if (forced) {
			l.surf_method = 6;
		} else {
			l.surf_method = (s_level_s >= 1 && s_level_s <= 5) ? s_level_s : 6;
		}

		// Window: the 4 completed blocks before the current one.
		const uint32_t cur = bid(now);
		bool blind = s_blind;
		uint32_t dry = 0, obs = 0, n = 0, dmax = 0;
		for (const auto &b : s_ring) {
			if (!b.used || b.id + WINDOW_BLOCKS < cur || b.id > cur) continue;
			if (b.blind) blind = true;
			if (b.id == cur) continue;
			dry += b.dry_s;
			obs += b.obs_s;
			n += b.n;
			if (b.dmax_s > dmax) dmax = b.dmax_s;
		}
		const bool partial = cur < WINDOW_BLOCKS;
		l.latch = 1;
		l.blind = blind ? 1 : 0;
		l.partial = partial ? 1 : 0;
		l.surf_seq = s_seq;
		l.dry_s = dry;
		l.obs_s = obs;
		l.n_known = !partial;
		l.n_surf = n;
		l.d_max_s = partial ? 0 : dmax;
		s_latch = l;
		s_latched = true;

		// The dive that starts now.
		s_state = State::UW;
		s_acct_from = now;
		s_has_snap = true;
		s_dive_start = now;
		s_dive_start_real = surface_real;
		s_dive_blinded = false;
		s_dto0 = dto;
		s_fsc0 = fsc;
	}

	static inline uint64_t s_reset_ms = 0;
	static inline State s_state = State::UNKNOWN;
	static inline bool s_blind = false;
	static inline uint64_t s_blind_start = 0;
	static inline uint64_t s_acct_from = 0;
	static inline Block s_ring[RING_SLOTS] = {};
	static inline uint16_t s_seq = 0;
	// current dive
	static inline bool s_has_snap = false;
	static inline uint64_t s_dive_start = 0;
	static inline bool s_dive_start_real = false;
	static inline bool s_dive_blinded = false;
	static inline uint32_t s_dto0 = 0;
	static inline uint32_t s_fsc0 = 0;
	// surface edge
	static inline uint64_t s_t_s = 0;
	static inline uint8_t s_level_s = 0;
	static inline uint32_t s_dto_s = 0;
	static inline uint32_t s_fsc_s = 0;
	static inline bool s_adcf_s = false;
	static inline bool s_surface_known = false;
	// latch
	static inline bool s_latched = false;
	static inline Dx1Behaviour s_latch = {};
	// RST: noinit record (defined in service.cpp), plus "already done this boot".
	static RstRecord s_rst __attribute__((section(".noinit")));
	static inline bool s_rst_done = false;
	static inline bool s_rst_valid = false;
};

#endif  // DX1_DOPPLER_EXT_BUILD
