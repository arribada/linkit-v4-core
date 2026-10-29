/**
 * @file hauled_mode_service.cpp
 * @brief HauledModeService — see hauled_mode_service.hpp.
 */

#include <algorithm>
#include <cstddef>
#include <cstring>

#include "hauled_mode_service.hpp"
#include "config_store.hpp"
#include "interrupt_lock.hpp"
#include "rtc.hpp"
#include "debug.hpp"
#include "pmu.hpp"  // HM-4 audit fix: PMU::get_timestamp_ms() for evaluate() cache

// Pre-deploy validation channel — emit grep-friendly [VAL-HAULED] tagged
// transitions when -DVALIDATION_LOG_ENABLE=1 is set at build. Default off
// (zero overhead in deployment). See METRIC_LATENCY_LOG_ENABLE precedent.
#ifndef VALIDATION_LOG_ENABLE
#define VALIDATION_LOG_ENABLE 0
#endif

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

extern ConfigurationStore *configuration_store;
extern RTC *rtc;

// Validation threshold for stored timestamps — anything below year 2000 is a
// virtual-RTC artifact or corruption, anything above year 2100 is corruption.
static constexpr std::time_t RTC_MIN_VALID = 946684800;   // 2000-01-01
static constexpr std::time_t RTC_MAX_VALID = 4102444800;  // 2100-01-01

namespace {

// Changes whenever HauledNoinit changes shape: an image left by another layout
// (e.g. before an OTA) fails this check as well as the CRC, and is re-initialised.
constexpr uint16_t NOINIT_LAYOUT = 0x4802;  // 'H', layout 2

struct HauledNoinit {
	std::time_t last_uw_event_rtc;  // last dive; 0 = never in the water
	std::time_t dry_since_rtc;      // start of the current dry spell; 0 = in the water or unknown
	uint8_t in_hauled;              // 0 = AT_SEA, 1 = HAULED
	uint8_t uw_events_since_hauled;
	uint16_t layout;    // NOINIT_LAYOUT
	uint16_t reserved;  // zero; keeps the struct free of padding
	uint16_t crc;       // CRC16 of every byte before it
};

// Every byte is a named field, so no stale RAM can hide outside the CRC.
static_assert(sizeof(HauledNoinit) == 2 * sizeof(std::time_t) + 4 * sizeof(uint16_t),
              "HauledNoinit must not be padded");

#ifndef CPPUTEST
HauledNoinit s_noinit __attribute__((section(".noinit")));
#else
HauledNoinit s_noinit;
#endif

// RAM only, false after every reset: set by the first detector report. Until
// then the dry spell restored from .noinit is stale and cannot engage HAULED.
bool s_uw_state_known = false;

uint16_t noinit_crc() {
	return crc16_compute(reinterpret_cast<const uint8_t *>(&s_noinit), offsetof(decltype(s_noinit), crc), nullptr);
}

void clear_state() {
	std::memset(&s_noinit, 0, sizeof(s_noinit));
	s_noinit.layout = NOINIT_LAYOUT;
	s_noinit.crc = noinit_crc();
}

// Mitigation M1b (2026-05): RTC rollback detected (typically a WDT reset
// brought RTC back to 1 while noinit still has timestamps from previous
// session). The previous behavior `return` left HAULED stuck for the rest
// of the deployment. Re-baseline every future stamp to `now` so the dry
// time restarts from the new RTC frame — HAULED can engage after
// threshold_h hours from this point. The hauled flag itself is preserved
// where applicable (only called when !in_hauled, so no state loss).
// Returns true when it re-baselined: nothing more to evaluate this time.
bool rebaseline_after_rtc_rollback(std::time_t now) {
	if (now >= s_noinit.last_uw_event_rtc && now >= s_noinit.dry_since_rtc) return false;
	DEBUG_WARN("HauledModeService::evaluate: RTC rollback (now=%u < stored=%u), re-baselining", (unsigned int)now,
	           (unsigned int)std::max(s_noinit.last_uw_event_rtc, s_noinit.dry_since_rtc));
	InterruptLock lock;
	if (now < s_noinit.last_uw_event_rtc) s_noinit.last_uw_event_rtc = now;
	if (now < s_noinit.dry_since_rtc) s_noinit.dry_since_rtc = now;
	s_noinit.crc = noinit_crc();
	return true;
}

}  // namespace

void HauledModeService::restore_state() {
	if (s_noinit.layout == NOINIT_LAYOUT && s_noinit.crc == noinit_crc() && s_noinit.in_hauled <= 1) {
		// Mitigation M1c (2026-05): a future-dated last_uw_event_rtc after a
		// WDT reset (RTC reset to virtual 1, noinit kept old large value) was
		// previously handled lazily in evaluate(). With M1b that path now
		// re-baselines correctly. We still defensively reject impossible
		// values here so the in-hauled flag isn't trusted blindly when the
		// timestamp shape is suspicious.
		if (s_noinit.last_uw_event_rtc != 0 && rtc && rtc->is_set()
		    && s_noinit.last_uw_event_rtc > rtc->gettime() + RTC_MAX_VALID) {
			// Absurd future value — likely a bit-flip past the CRC. Clear.
			DEBUG_WARN("HauledModeService: noinit last_uw_event_rtc=%u absurdly future, discarding",
			           (unsigned int)s_noinit.last_uw_event_rtc);
			clear_state();
			return;
		}
		if (s_noinit.dry_since_rtc != 0 && rtc && rtc->is_set()
		    && s_noinit.dry_since_rtc > rtc->gettime() + RTC_MAX_VALID) {
			DEBUG_WARN("HauledModeService: noinit dry_since_rtc=%u absurdly future, discarding",
			           (unsigned int)s_noinit.dry_since_rtc);
			clear_state();
			return;
		}
		DEBUG_DETAIL("HauledModeService: restored from noinit (in_hauled=%u, last_uw=%u, returns=%u)",
		             s_noinit.in_hauled, (unsigned int)s_noinit.last_uw_event_rtc, s_noinit.uw_events_since_hauled);
		return;
	}
	DEBUG_TRACE("HauledModeService: noinit invalid, starting fresh (AT_SEA)");
	clear_state();
}

void HauledModeService::on_underwater_event(bool submerged, std::time_t now) {
	if (!configuration_store) return;

	InterruptLock lock;
	s_uw_state_known = true;

	if (!submerged) {
		// Surfacing starts the dry spell. A repeated dry report (detector restart,
		// reboot) keeps it, unless the RTC has gone back past its start.
		if (s_noinit.dry_since_rtc == 0 || now < s_noinit.dry_since_rtc) s_noinit.dry_since_rtc = now;
		s_noinit.crc = noinit_crc();
		return;
	}

	s_noinit.last_uw_event_rtc = now;
	s_noinit.dry_since_rtc = 0;  // back in the water: the dry spell is over

	if (s_noinit.in_hauled) {
		// Hauled → AT_SEA hysteresis: count consecutive dive events.
		unsigned int needed = configuration_store->read_param<unsigned int>(ParamID::HAULED_RETURN_EVENTS);
		if (needed == 0) needed = 1;
		// F-SWS-5 audit fix: clamp to minimum 2 to defend against a single
		// stray UW=true emit (wave splash, Vbatt sag false flap) immediately
		// exiting HAULED. With needed=1 a dead/stranded animal scenario
		// could lose its battery-throttling cadence on a single bad sample.
		// DTE allows down to 1 (HMP02 min=1) for testing/QA, but production
		// defaults should be ≥ 2 — this clamp enforces it at runtime.
		if (needed < 2) needed = 2;
		if (s_noinit.uw_events_since_hauled < 0xFF) s_noinit.uw_events_since_hauled++;
		if (s_noinit.uw_events_since_hauled >= needed) {
			DEBUG_INFO("HauledModeService: HAULED → AT_SEA after %u dive events", s_noinit.uw_events_since_hauled);
#if VALIDATION_LOG_ENABLE
			DEBUG_INFO("[VAL-HAULED] exit AT_SEA t=%u events=%u last_uw=%u", (unsigned int)now,
			           s_noinit.uw_events_since_hauled, (unsigned int)s_noinit.last_uw_event_rtc);
#endif
			s_noinit.in_hauled = 0;
			s_noinit.uw_events_since_hauled = 0;
		}
	}

	s_noinit.crc = noinit_crc();
}

void HauledModeService::evaluate() {
	if (!rtc || !rtc->is_set()) return;  // No RTC → can't compute age, stay in current state
	evaluate(rtc->gettime());
}

void HauledModeService::evaluate(std::time_t now) {
	if (!configuration_store) return;

// HM-4 audit fix: short-window cache to avoid re-reading config params and
// running the threshold subtraction on every call (evaluate is invoked
// from every get_argos_configuration / get_gnss_configuration, which fires
// many times per second). Cache TTL kept very short (500 ms) so a DTE
// HMP00 toggle takes effect within half a second. The HMP00=disabled
// fast-path below the cache check ALWAYS runs (immediate exit-from-HAULED
// semantics preserved). Only the AT_SEA→HAULED threshold check is cached.
#ifndef CPPUTEST
	static uint64_t s_last_evaluate_ms = 0;
	uint64_t now_ms = PMU::get_timestamp_ms();
	bool cache_hot = (s_last_evaluate_ms != 0) && (now_ms - s_last_evaluate_ms < 500);
#else
	bool cache_hot = false;  // tests need deterministic evaluation
#endif

	if (!configuration_store->read_param<bool>(ParamID::HAULED_DETECT_EN)) {
		// Detection disabled: ensure we never stay stuck in HAULED.
		// NOT cached: immediate response to DTE HMP00 toggle is required.
		if (s_noinit.in_hauled) {
			InterruptLock lock;
			s_noinit.in_hauled = 0;
			s_noinit.uw_events_since_hauled = 0;
			s_noinit.crc = noinit_crc();
		}
		return;
	}
	if (s_noinit.in_hauled) return;               // hysteresis: only on_underwater_event can clear
	if (s_noinit.last_uw_event_rtc == 0) return;  // never been in the water — stay AT_SEA

	// Cache hot AND nothing changed above (still AT_SEA + last_uw stamped):
	// skip the threshold check + HMP01 read. Saves a config_store lookup +
	// one std::time_t subtraction per call. Threshold is in hours so a
	// 500 ms cache window cannot meaningfully delay an AT_SEA→HAULED engage.
	if (cache_hot) return;
#ifndef CPPUTEST
	s_last_evaluate_ms = now_ms;
#endif
	if (rebaseline_after_rtc_rollback(now)) return;  // M1b, see the helper

	// Only a dry spell the detector has confirmed since boot counts: a dive
	// ends it, and a stamp restored after a reset alone proves nothing.
	if (!s_uw_state_known || s_noinit.dry_since_rtc == 0) return;

	unsigned int threshold_h = configuration_store->read_param<unsigned int>(ParamID::HAULED_IDLE_THRESHOLD_H);
	if (threshold_h == 0) return;
	const std::time_t threshold_s = (std::time_t)threshold_h * 3600;
	const std::time_t dry_s = now - s_noinit.dry_since_rtc;
	if (dry_s > threshold_s) {
		InterruptLock lock;
		s_noinit.in_hauled = 1;
		s_noinit.uw_events_since_hauled = 0;
		s_noinit.crc = noinit_crc();
		DEBUG_INFO("HauledModeService: AT_SEA → HAULED (dry for %u s, threshold %u h)", (unsigned int)dry_s,
		           threshold_h);
#if VALIDATION_LOG_ENABLE
		DEBUG_INFO("[VAL-HAULED] enter HAULED t=%u dry_s=%u threshold_h=%u", (unsigned int)now, (unsigned int)dry_s,
		           threshold_h);
#endif
	}
}

bool HauledModeService::is_hauled() {
	return s_noinit.in_hauled != 0;
}

std::time_t HauledModeService::last_uw_event_rtc() {
	return s_noinit.last_uw_event_rtc;
}

std::time_t HauledModeService::dry_since_rtc() {
	return s_noinit.dry_since_rtc;
}

unsigned int HauledModeService::uw_events_since_hauled() {
	return s_noinit.uw_events_since_hauled;
}

void HauledModeService::reset_for_rtc_sync(std::time_t now) {
	// Called by GPSService right after rtc->settime(real_t). Re-anchor the
	// stamps to the new RTC frame; keep logical state. A zero stamp means
	// "never in the water" / "not dry" and must stay zero.
	InterruptLock lock;
	const std::time_t last_uw = s_noinit.last_uw_event_rtc ? now : 0;
	DEBUG_INFO("HauledModeService::reset_for_rtc_sync: re-anchoring last_uw_event_rtc %u → %u (in_hauled=%u)",
	           (unsigned int)s_noinit.last_uw_event_rtc, (unsigned int)last_uw, s_noinit.in_hauled);
	s_noinit.last_uw_event_rtc = last_uw;
	if (s_noinit.dry_since_rtc) s_noinit.dry_since_rtc = now;
	s_noinit.crc = noinit_crc();
}

void HauledModeService::reset_for_tests() {
	clear_state();
	s_uw_state_known = false;
}

#ifdef CPPUTEST
void HauledModeService::simulate_reset_for_tests() {
	s_uw_state_known = false;
}

uint8_t *HauledModeService::noinit_bytes_for_tests(std::size_t &size) {
	size = sizeof(s_noinit);
	return reinterpret_cast<uint8_t *>(&s_noinit);
}
#endif
