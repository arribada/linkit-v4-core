/**
 * @file hauled_mode_service.cpp
 * @brief HauledModeService — see hauled_mode_service.hpp.
 */

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

struct HauledNoinit {
	std::time_t last_uw_event_rtc;
	uint8_t in_hauled;  // 0 = AT_SEA, 1 = HAULED
	uint8_t uw_events_since_hauled;
	uint16_t pad;  // keep `crc` deterministically aligned
	uint16_t crc;
};

#ifndef CPPUTEST
HauledNoinit s_noinit __attribute__((section(".noinit")));
#else
HauledNoinit s_noinit;
#endif

uint16_t noinit_crc() {
	return crc16_compute(reinterpret_cast<const uint8_t *>(&s_noinit), offsetof(decltype(s_noinit), crc), nullptr);
}

void clear_state() {
	std::memset(&s_noinit, 0, sizeof(s_noinit));
	s_noinit.crc = noinit_crc();
}

}  // namespace

void HauledModeService::restore_state() {
	if (s_noinit.crc == noinit_crc() && s_noinit.in_hauled <= 1) {
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
		DEBUG_INFO("HauledModeService: restored from noinit (in_hauled=%u, last_uw=%u, returns=%u)", s_noinit.in_hauled,
		           (unsigned int)s_noinit.last_uw_event_rtc, s_noinit.uw_events_since_hauled);
		return;
	}
	DEBUG_TRACE("HauledModeService: noinit invalid, starting fresh (AT_SEA)");
	clear_state();
}

void HauledModeService::on_underwater_event(bool submerged, std::time_t now) {
	if (!configuration_store) return;
	// Only act on dive events; surfacing transitions are not informative
	// for hauled detection (the dry counter just keeps running).
	if (!submerged) return;

	InterruptLock lock;
	s_noinit.last_uw_event_rtc = now;

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
	if (s_noinit.last_uw_event_rtc == 0) return;  // never seen UW yet — stay AT_SEA

	// Cache hot AND nothing changed above (still AT_SEA + last_uw stamped):
	// skip the threshold check + HMP01 read. Saves a config_store lookup +
	// one std::time_t subtraction per call. Threshold is in hours so a
	// 500 ms cache window cannot meaningfully delay an AT_SEA→HAULED engage.
	if (cache_hot) return;
#ifndef CPPUTEST
	s_last_evaluate_ms = now_ms;
#endif
	// Mitigation M1b (2026-05): RTC rollback detected (typically a WDT reset
	// brought RTC back to 1 while noinit still has timestamps from previous
	// session). The previous behavior `return` left HAULED stuck for the rest
	// of the deployment. Re-baseline `last_uw_event_rtc = now` so the elapsed-
	// time count restarts from the new RTC frame — HAULED can engage after
	// threshold_h hours from this point. The hauled flag itself is preserved
	// where applicable (we only reach here when !in_hauled, so no state loss).
	if (now < s_noinit.last_uw_event_rtc) {
		DEBUG_WARN("HauledModeService::evaluate: RTC rollback (now=%u < stored=%u), re-baselining", (unsigned int)now,
		           (unsigned int)s_noinit.last_uw_event_rtc);
		InterruptLock lock;
		s_noinit.last_uw_event_rtc = now;
		s_noinit.crc = noinit_crc();
		return;
	}

	unsigned int threshold_h = configuration_store->read_param<unsigned int>(ParamID::HAULED_IDLE_THRESHOLD_H);
	if (threshold_h == 0) return;
	std::time_t threshold_s = (std::time_t)threshold_h * 3600;
	if ((now - s_noinit.last_uw_event_rtc) > threshold_s) {
		InterruptLock lock;
		s_noinit.in_hauled = 1;
		s_noinit.uw_events_since_hauled = 0;
		s_noinit.crc = noinit_crc();
		DEBUG_INFO("HauledModeService: AT_SEA → HAULED (dry for %u s, threshold %u h)",
		           (unsigned int)(now - s_noinit.last_uw_event_rtc), threshold_h);
#if VALIDATION_LOG_ENABLE
		DEBUG_INFO("[VAL-HAULED] enter HAULED t=%u dry_s=%u threshold_h=%u", (unsigned int)now,
		           (unsigned int)(now - s_noinit.last_uw_event_rtc), threshold_h);
#endif
	}
}

bool HauledModeService::is_hauled() {
	return s_noinit.in_hauled != 0;
}

std::time_t HauledModeService::last_uw_event_rtc() {
	return s_noinit.last_uw_event_rtc;
}

unsigned int HauledModeService::uw_events_since_hauled() {
	return s_noinit.uw_events_since_hauled;
}

void HauledModeService::reset_for_rtc_sync(std::time_t now) {
	// Called by GPSService right after rtc->settime(real_t). Re-anchor the
	// elapsed-time baseline to the new RTC frame; keep logical state.
	InterruptLock lock;
	DEBUG_INFO("HauledModeService::reset_for_rtc_sync: re-anchoring last_uw_event_rtc %u → %u (in_hauled=%u)",
	           (unsigned int)s_noinit.last_uw_event_rtc, (unsigned int)now, s_noinit.in_hauled);
	s_noinit.last_uw_event_rtc = now;
	s_noinit.crc = noinit_crc();
}

void HauledModeService::reset_for_tests() {
	clear_state();
}
