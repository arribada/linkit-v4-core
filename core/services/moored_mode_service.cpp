/**
 * @file moored_mode_service.cpp
 * @brief MooredModeService — see moored_mode_service.hpp.
 */

#include <cstddef>
#include <cstring>

#include "moored_mode_service.hpp"
#include "config_store.hpp"
#include "haversine.hpp"
#include "interrupt_lock.hpp"
#include "rtc.hpp"
#include "debug.hpp"
#include "pmu.hpp"  // PMU::get_timestamp_ms() for the evaluate() cache

// Pre-deploy validation channel — emit grep-friendly [VAL-MOORED] tagged
// transitions when -DVALIDATION_LOG_ENABLE=1 is set at build. Default off
// (zero overhead in deployment). Mirrors HauledModeService.
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

namespace {

// Field order is chosen so the struct has NO internal padding on any ABI the
// project targets (8-byte doubles and time_t first, then the byte-sized
// counters, then an explicit pad): the CRC is computed over the raw bytes, so
// a compiler-inserted hole would make the checksum non-deterministic.
struct MooredNoinit {
	double ref_lat;  // reference anchor (degrees)
	double ref_lon;
	std::time_t last_axl_exit_rtc;  // debounce anchor for accelerometer exits
	uint8_t in_moored;              // 0 = UNDERWAY, 1 = MOORED
	uint8_t has_ref;                // 0 until the first valid fix lands
	uint8_t stationary_fixes;       // consecutive fixes inside the radius
	uint8_t motion_events;          // consecutive AXL wake-ups while MOORED
	uint16_t pad;                   // keep `crc` deterministically aligned
	uint16_t crc;
};

#ifndef CPPUTEST
MooredNoinit s_noinit __attribute__((section(".noinit")));
#else
MooredNoinit s_noinit;
#endif

uint16_t noinit_crc() {
	return crc16_compute(reinterpret_cast<const uint8_t *>(&s_noinit), offsetof(decltype(s_noinit), crc), nullptr);
}

void clear_state() {
	std::memset(&s_noinit, 0, sizeof(s_noinit));
	s_noinit.crc = noinit_crc();
}

// ConfigurationStore::read_param throws CONFIG_STORE_CORRUPTED when the store
// is invalid or a variant type does not match. This classifier is a pure
// battery optimisation reached from the peer-event broadcast — it must never be
// able to throw into the event bus and take the scheduler down with it. Every
// read goes through these two non-throwing accessors, whose fallbacks are the
// production defaults; MOORED_DETECT_EN falling back to false means "config
// unreadable -> behave exactly as if the feature did not exist".
bool read_bool(ParamID id, bool fallback) {
	if (!configuration_store) return fallback;
	try {
		return configuration_store->read_param<bool>(id);
	} catch (...) {
		return fallback;
	}
}

unsigned int read_uint(ParamID id, unsigned int fallback) {
	if (!configuration_store) return fallback;
	try {
		return configuration_store->read_param<unsigned int>(id);
	} catch (...) {
		return fallback;
	}
}

bool detect_enabled() {
	return read_bool(ParamID::MOORED_DETECT_EN, false);
}

/// Mirror the logical state into the read-only MRT01 status param so the
/// operator can read back "does this sealed tracker think it is moored?" over
/// DTE/BLE. Written on transition only: get_*_configuration() runs several
/// times per second and flash persistence is deferred to the periodic flush.
/// Never allowed to escape as an exception — this is a diagnostic mirror, not
/// a state the classifier depends on.
void publish_state(uint8_t in_moored) {
	if (!configuration_store) return;
	try {
		configuration_store->write_param(ParamID::MOORED_STATE, (unsigned int)in_moored);
	} catch (...) {
		DEBUG_WARN("MooredModeService: could not publish MOORED_STATE");
	}
}

/// Leave MOORED. Caller must hold an InterruptLock and refresh the CRC.
void enter_underway_locked() {
	s_noinit.in_moored = 0;
	s_noinit.stationary_fixes = 0;
	s_noinit.motion_events = 0;
}

}  // namespace

void MooredModeService::restore_state() {
	if (s_noinit.crc == noinit_crc() && s_noinit.in_moored <= 1 && s_noinit.has_ref <= 1) {
		DEBUG_INFO("MooredModeService: restored from noinit (moored=%u, has_ref=%u, still=%u, motion=%u)",
		           s_noinit.in_moored, s_noinit.has_ref, s_noinit.stationary_fixes, s_noinit.motion_events);
		publish_state(s_noinit.in_moored);
		return;
	}
	DEBUG_TRACE("MooredModeService: noinit invalid, starting fresh (UNDERWAY)");
	clear_state();
	publish_state(0);
}

void MooredModeService::on_gnss_fix(double lat, double lon, int32_t gspeed_mms, std::time_t now) {
	(void)now;
	if (!detect_enabled()) return;

	unsigned int radius_m = read_uint(ParamID::MOORED_RADIUS_M, 150);
	if (radius_m < MIN_RADIUS_M) radius_m = MIN_RADIUS_M;
	unsigned int enter_fixes = read_uint(ParamID::MOORED_ENTER_FIXES, 3);
	if (enter_fixes == 0) enter_fixes = 1;

	// First fix of the deployment (or after a cold boot): plant the anchor and
	// wait. Deliberately does NOT count as a stationary fix — a single point
	// carries no displacement information.
	if (!s_noinit.has_ref) {
		InterruptLock lock;
		s_noinit.ref_lat = lat;
		s_noinit.ref_lon = lon;
		s_noinit.has_ref = 1;
		s_noinit.stationary_fixes = 0;
		s_noinit.crc = noinit_crc();
		DEBUG_INFO("MooredModeService: reference anchor set (%.6f, %.6f)", lat, lon);
		return;
	}

	double distance_m = haversine_distance(s_noinit.ref_lon, s_noinit.ref_lat, lon, lat) * 1000.0;
	bool too_far = distance_m > (double)radius_m;
	bool too_fast = gspeed_mms > UNDERWAY_SPEED_MMS;

	if (too_far || too_fast) {
		// Under way. Re-plant the anchor at the new position so the next
		// stationary run is measured from where the vessel actually stopped.
		bool was_moored = s_noinit.in_moored;
		{
			InterruptLock lock;
			s_noinit.ref_lat = lat;
			s_noinit.ref_lon = lon;
			s_noinit.stationary_fixes = 0;
			if (was_moored) enter_underway_locked();
			s_noinit.crc = noinit_crc();
		}
		if (was_moored) {
			DEBUG_INFO("MooredModeService: MOORED -> UNDERWAY (%s: d=%.1f m radius=%u m gSpeed=%ld mm/s)",
			           too_far ? "displacement" : "speed", distance_m, radius_m, (long)gspeed_mms);
#if VALIDATION_LOG_ENABLE
			DEBUG_INFO("[VAL-MOORED] exit UNDERWAY t=%u d_m=%d gspeed=%ld reason=%s", (unsigned int)now,
			           (int)distance_m, (long)gspeed_mms, too_far ? "displacement" : "speed");
#endif
			publish_state(0);
		} else {
			DEBUG_TRACE("MooredModeService: still under way (d=%.1f m gSpeed=%ld mm/s)", distance_m, (long)gspeed_mms);
		}
		return;
	}

	// Stationary. The anchor stays put on purpose — see the header comment:
	// re-centring here would let a slow drift creep away unnoticed.
	if (s_noinit.in_moored) {
		DEBUG_TRACE("MooredModeService: still moored (d=%.1f m)", distance_m);
		return;
	}

	{
		InterruptLock lock;
		if (s_noinit.stationary_fixes < 0xFF) s_noinit.stationary_fixes++;
		s_noinit.crc = noinit_crc();
	}

	if (s_noinit.stationary_fixes >= enter_fixes) {
		{
			InterruptLock lock;
			s_noinit.in_moored = 1;
			s_noinit.motion_events = 0;
			s_noinit.crc = noinit_crc();
		}
		DEBUG_INFO("MooredModeService: UNDERWAY -> MOORED (%u fixes within %u m of anchor)", s_noinit.stationary_fixes,
		           radius_m);
#if VALIDATION_LOG_ENABLE
		DEBUG_INFO("[VAL-MOORED] enter MOORED t=%u fixes=%u radius_m=%u d_m=%d", (unsigned int)now,
		           s_noinit.stationary_fixes, radius_m, (int)distance_m);
#endif
		publish_state(1);
	} else {
		DEBUG_TRACE("MooredModeService: stationary %u/%u (d=%.1f m)", s_noinit.stationary_fixes, enter_fixes,
		            distance_m);
	}
}

void MooredModeService::on_motion_event(std::time_t now) {
	if (!detect_enabled()) return;

	// A wake-up while already under way carries no information — the vessel is
	// moving and the GNSS cadence is already at its fastest.
	if (!s_noinit.in_moored) {
		if (s_noinit.motion_events) {
			InterruptLock lock;
			s_noinit.motion_events = 0;
			s_noinit.crc = noinit_crc();
		}
		return;
	}

	// Anti-flapping. A moored vessel still rocks: without this hold-off, swell
	// or a passing wake would trip an exit (and therefore a GNSS acquisition
	// via GNSS_TRIGGER_ON_AXL_WAKEUP) over and over, and the whole economy
	// would evaporate. The hold-off bounds the worst case to one wake-up-driven
	// acquisition per MOORED_AXL_HOLDOFF_S however hard the accelerometer
	// chatters.
	unsigned int holdoff_s = read_uint(ParamID::MOORED_AXL_HOLDOFF_S, 900);
	if (holdoff_s && s_noinit.last_axl_exit_rtc != 0 && now >= s_noinit.last_axl_exit_rtc
	    && (now - s_noinit.last_axl_exit_rtc) < (std::time_t)holdoff_s) {
		DEBUG_TRACE("MooredModeService: motion event ignored (hold-off, %u s remaining)",
		            (unsigned int)(holdoff_s - (now - s_noinit.last_axl_exit_rtc)));
		return;
	}

	unsigned int exit_events = read_uint(ParamID::MOORED_EXIT_EVENTS, 2);
	if (exit_events == 0) exit_events = 1;

	{
		InterruptLock lock;
		if (s_noinit.motion_events < 0xFF) s_noinit.motion_events++;
		s_noinit.crc = noinit_crc();
	}

	if (s_noinit.motion_events < exit_events) {
		DEBUG_TRACE("MooredModeService: motion %u/%u", s_noinit.motion_events, exit_events);
		return;
	}

	DEBUG_INFO("MooredModeService: MOORED -> UNDERWAY (%u accelerometer wake-ups)", s_noinit.motion_events);
#if VALIDATION_LOG_ENABLE
	DEBUG_INFO("[VAL-MOORED] exit UNDERWAY t=%u events=%u reason=motion", (unsigned int)now, s_noinit.motion_events);
#endif
	{
		InterruptLock lock;
		enter_underway_locked();
		s_noinit.last_axl_exit_rtc = now;
		s_noinit.crc = noinit_crc();
	}
	publish_state(0);
	// No reschedule kick is needed here: GNSS_TRIGGER_ON_AXL_WAKEUP (GNP26)
	// already turns this same wake-up event into an immediate GNSS acquisition
	// in GPSService::service_is_triggered_on_event, and that fix then routes
	// through LoRaTxService's "GNSS fix -> reschedule for immediate TX" branch.
}

void MooredModeService::evaluate() {
	if (!rtc || !rtc->is_set()) return;  // no RTC -> can't reason about the hold-off
	evaluate(rtc->gettime());
}

void MooredModeService::evaluate(std::time_t now) {
	if (!configuration_store) return;

// Short-window cache: evaluate() is invoked from every
// get_argos_configuration / get_gnss_configuration, which fires many times
// per second. Mirrors the HM-4 fix in HauledModeService. The
// feature-disabled fast-path below is deliberately NOT cached, so a DTE
// MRP00 toggle takes effect immediately.
#ifndef CPPUTEST
	static uint64_t s_last_evaluate_ms = 0;
	uint64_t now_ms = PMU::get_timestamp_ms();
	bool cache_hot = (s_last_evaluate_ms != 0) && (now_ms - s_last_evaluate_ms < 500);
#else
	bool cache_hot = false;  // tests need deterministic evaluation
#endif

	if (!read_bool(ParamID::MOORED_DETECT_EN, false)) {
		// Detection disabled: never stay stuck in MOORED, or the operator would
		// turn the feature off and keep the slow cadence.
		if (s_noinit.in_moored) {
			{
				InterruptLock lock;
				enter_underway_locked();
				s_noinit.crc = noinit_crc();
			}
			DEBUG_INFO("MooredModeService: MRP00 cleared — forcing UNDERWAY");
			publish_state(0);
		}
		return;
	}

	if (cache_hot) return;
#ifndef CPPUTEST
	s_last_evaluate_ms = now_ms;
#endif

	// RTC rollback (typically a WDT reset that brought the RTC back to the
	// virtual epoch while noinit kept last session's real timestamps). Left
	// alone, the hold-off comparison below would be measured against a
	// timestamp ~50 years in the future and would suppress every accelerometer
	// exit for the rest of the deployment — the tracker would stay moored while
	// the boat sailed away. Re-baseline instead.
	//
	// Note this is self-healing on purpose: the analogous
	// HauledModeService::reset_for_rtc_sync / RateLimiter::reset_for_rtc_sync
	// hooks document a GPSService caller that does not exist (the RTC is set in
	// the M10Q driver and by DTE), so no such hook can be relied upon here.
	if (s_noinit.last_axl_exit_rtc != 0 && now < s_noinit.last_axl_exit_rtc) {
		DEBUG_WARN("MooredModeService::evaluate: RTC rollback (now=%u < stored=%u), re-baselining", (unsigned int)now,
		           (unsigned int)s_noinit.last_axl_exit_rtc);
		InterruptLock lock;
		s_noinit.last_axl_exit_rtc = now;
		s_noinit.crc = noinit_crc();
	}
}

bool MooredModeService::is_moored() {
	return s_noinit.in_moored != 0;
}

bool MooredModeService::has_reference() {
	return s_noinit.has_ref != 0;
}

unsigned int MooredModeService::stationary_fixes() {
	return s_noinit.stationary_fixes;
}

unsigned int MooredModeService::motion_events() {
	return s_noinit.motion_events;
}

double MooredModeService::reference_lat() {
	return s_noinit.ref_lat;
}

double MooredModeService::reference_lon() {
	return s_noinit.ref_lon;
}

double MooredModeService::distance_to_reference_m(double lat, double lon) {
	if (!s_noinit.has_ref) return -1.0;
	return haversine_distance(s_noinit.ref_lon, s_noinit.ref_lat, lon, lat) * 1000.0;
}

void MooredModeService::reset_for_tests() {
	clear_state();
}
