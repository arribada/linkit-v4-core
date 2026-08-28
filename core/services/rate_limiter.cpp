/**
 * @file rate_limiter.cpp
 * @brief Sliding-window TX rate limiter — see rate_limiter.hpp.
 */

#include <cstddef>
#include <cstring>
#include <algorithm>

#include "rate_limiter.hpp"
#include "config_store.hpp"
#include "interrupt_lock.hpp"
#include "rtc.hpp"
#include "debug.hpp"

// Pre-deploy validation channel — see hauled_mode_service.cpp header comment.
// Enables grep-friendly [VAL-RATELIMIT] tags on block events. Default off.
#ifndef VALIDATION_LOG_ENABLE
#define VALIDATION_LOG_ENABLE 0
#endif

extern ConfigurationStore *configuration_store;
extern RTC *rtc;

// CRC16-CCITT: nRF SDK header in firmware build, inline stub in CppUTest build.
// Mirrors the conditional already present in service.cpp.
#ifndef CPPUTEST
#include "crc16.h"
#else
#include <cstdint>
static inline uint16_t crc16_compute(const uint8_t *data, uint16_t length, const uint16_t *) {
	uint16_t crc = 0xFFFF;
	for (uint16_t i = 0; i < length; i++) {
		crc ^= static_cast<uint16_t>(data[i]) << 8;
		for (uint8_t j = 0; j < 8; j++)
			crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
	}
	return crc;
}
#endif

namespace {

// Ring buffer of TX completion timestamps. `count` is the number of valid
// entries (0..MAX_CAP) and `head` points to the next write slot (mod MAX_CAP).
// We never explicitly expire old entries — `is_blocked` / `count_in_window`
// simply ignore entries outside the window. The ring naturally cycles as new
// TXes overwrite stale slots.
struct RateLimiterNoinit {
	std::time_t ring[RateLimiter::MAX_CAP];
	uint8_t head;   // next write index
	uint8_t count;  // valid entries (saturates at MAX_CAP)
	uint16_t pad;   // explicit pad keeps `crc` aligned + makes the CRC
	                // input deterministic across compilers (no padding).
	uint16_t crc;
};

#ifndef CPPUTEST
RateLimiterNoinit s_noinit __attribute__((section(".noinit")));
#else
RateLimiterNoinit s_noinit;
#endif

uint16_t noinit_crc() {
	return crc16_compute(reinterpret_cast<const uint8_t *>(&s_noinit), offsetof(decltype(s_noinit), crc), nullptr);
}

void clear_ring() {
	std::memset(s_noinit.ring, 0, sizeof(s_noinit.ring));
	s_noinit.head = 0;
	s_noinit.count = 0;
	s_noinit.pad = 0;
	s_noinit.crc = noinit_crc();
}

}  // namespace

void RateLimiter::restore_state() {
	if (s_noinit.crc == noinit_crc() && s_noinit.head < MAX_CAP && s_noinit.count <= MAX_CAP) {
		// Mitigation M1c (2026-05): if every stored timestamp is now in the
		// "future" relative to current RTC (typical after WDT reset that brought
		// RTC back to virtual 1 while noinit kept the old session's epoch
		// timestamps), the ring is effectively dead (the `ts > now` defense in
		// is_blocked would filter every entry). Detect this case explicitly and
		// clear the ring so it's reusable from boot. Keeps the noinit structure
		// itself stable across less-extreme rollbacks.
		if (rtc && rtc->is_set() && s_noinit.count > 0) {
			std::time_t now = rtc->gettime();
			bool all_future = true;
			std::time_t min_ts = s_noinit.ring[0];
			std::time_t max_ts = s_noinit.ring[0];
			for (unsigned int i = 0; i < s_noinit.count; i++) {
				if (s_noinit.ring[i] <= now) all_future = false;
				if (s_noinit.ring[i] < min_ts) min_ts = s_noinit.ring[i];
				if (s_noinit.ring[i] > max_ts) max_ts = s_noinit.ring[i];
			}
			if (all_future) {
				DEBUG_WARN("RateLimiter: all %u stored timestamps in future (now=%u) — clearing ring", s_noinit.count,
				           (unsigned int)now);
				clear_ring();
				return;
			}
			// RateLimiter MED audit fix: detect MIXED-frame ring (some entries
			// pre-rollback in virtual epoch, others post-RTC-sync). The
			// all_future check above only handles uniform-future state; mixed
			// frames can quasi-stall the limiter into long reschedules that
			// never converge. Heuristic: if max - min span > 1 year, the ring
			// straddles two RTC frames — clear to recover.
			constexpr std::time_t ONE_YEAR_S = 365LL * 24 * 3600;
			if ((max_ts - min_ts) > ONE_YEAR_S) {
				DEBUG_WARN("RateLimiter: ring spans %lld s (mixed-frame post-rollback) — clearing",
				           (long long)(max_ts - min_ts));
				clear_ring();
				return;
			}
		}
		DEBUG_INFO("RateLimiter: restored from noinit (count=%u, head=%u)", s_noinit.count, s_noinit.head);
		return;
	}
	DEBUG_TRACE("RateLimiter: noinit invalid, starting fresh");
	clear_ring();
}

void RateLimiter::record_tx(std::time_t now) {
	if (!configuration_store) return;
	if (!configuration_store->read_param<bool>(ParamID::RATE_LIMIT_EN)) return;
	unsigned int cap = configuration_store->read_param<unsigned int>(ParamID::RATE_LIMIT_MAX_TX);
	if (cap == 0) return;  // 0 = unbounded by cap; nothing to record

	InterruptLock lock;
	s_noinit.ring[s_noinit.head] = now;
	s_noinit.head = (s_noinit.head + 1) % MAX_CAP;
	if (s_noinit.count < MAX_CAP) s_noinit.count++;
	s_noinit.crc = noinit_crc();
}

unsigned int RateLimiter::count_in_window(std::time_t now, unsigned int window_s) {
	unsigned int hits = 0;
	std::time_t cutoff = now - static_cast<std::time_t>(window_s);
	for (unsigned int i = 0; i < s_noinit.count; i++) {
		if (s_noinit.ring[i] > cutoff && s_noinit.ring[i] <= now) hits++;
	}
	return hits;
}

bool RateLimiter::is_blocked(std::time_t now, unsigned int &reschedule_in_s) {
	reschedule_in_s = 0;
	if (!configuration_store) return false;
	if (!configuration_store->read_param<bool>(ParamID::RATE_LIMIT_EN)) return false;
	unsigned int cap = configuration_store->read_param<unsigned int>(ParamID::RATE_LIMIT_MAX_TX);
	unsigned int window_s = configuration_store->read_param<unsigned int>(ParamID::RATE_LIMIT_WINDOW_S);
	if (cap == 0 || window_s == 0) return false;

	// Count entries inside the window and find the oldest such entry — the
	// reschedule deadline is `oldest_in_window + window_s` (when that entry
	// rolls out of the window, freeing one slot).
	std::time_t cutoff = now - static_cast<std::time_t>(window_s);
	std::time_t oldest = 0;
	unsigned int hits = 0;
	for (unsigned int i = 0; i < s_noinit.count; i++) {
		std::time_t ts = s_noinit.ring[i];
		// Reject future-dated entries (RTC rollback / corruption) — match the
		// same defense used in compute_gps_log_age_seconds.
		if (ts > now) continue;
		if (ts > cutoff) {
			hits++;
			if (oldest == 0 || ts < oldest) oldest = ts;
		}
	}
	if (hits < cap) return false;

	// Cap reached. Compute when the oldest in-window entry expires.
	std::time_t expiry = oldest + static_cast<std::time_t>(window_s);
	reschedule_in_s = (expiry > now) ? static_cast<unsigned int>(expiry - now) : 1;
#if VALIDATION_LOG_ENABLE
	DEBUG_INFO("[VAL-RATELIMIT] block t=%u hits=%u/%u window_s=%u reschedule_in_s=%u", (unsigned int)now, hits, cap,
	           window_s, reschedule_in_s);
#endif
	return true;
}

void RateLimiter::reset_for_rtc_sync() {
	// Called by GPSService right after rtc->settime(real_t). Pre-sync ring
	// entries are in virtual-RTC frame and have no meaning post-sync; clear
	// explicitly to avoid relying on the implicit `ts > now → skip` filter.
	DEBUG_INFO("RateLimiter::reset_for_rtc_sync: clearing %u entries", s_noinit.count);
	InterruptLock lock;
	clear_ring();
}

void RateLimiter::reset_for_tests() {
	clear_ring();
}
