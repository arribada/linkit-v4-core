/**
 * @file schedule_decision.hpp
 * @brief What a service decided to do next, and why.
 */

#pragma once

#include <cstdint>
#include <type_traits>

/// @brief The outcome of a scheduling decision: when the service runs next, or
///        why it does not, and what will bring it back.
///
/// This replaces a bare `unsigned int` whose one magic value, SCHEDULE_DISABLED,
/// meant five different things: off by configuration, nothing to do right now, a
/// gate is holding me, I could not compute a slot, this mode is incompatible.
/// The schedulers' INVALID_SCHEDULE is numerically the SAME value, so a failed
/// computation was indistinguishable from a deliberate stop.
///
/// Service::reschedule() posted nothing for all of them, so a temporary
/// condition became permanent the moment no external event happened to arrive --
/// and on a tag with neither GNSS nor an underwater sensor, none ever does. That
/// is the root of a series of field defects: beacons mute for a whole
/// deployment, a GNSS service deadlocked waiting for a transmission that could
/// not happen, a suspension that suspended nothing.
///
/// The four kinds separate what the sentinel conflated:
///
///  - Run          transmit / acquire / sample in `delay_ms`.
///  - HoldUntil    nothing to do before a known instant. The framework comes
///                 back on its own at that instant.
///  - HoldForEvent waiting for something to happen, WITH a deadline after which
///                 the framework re-evaluates anyway.
///  - Off          the configuration says no. The only kind that legitimately
///                 leaves a service with nothing pending.
///
/// `hold_for_event()` has no overload without a bound, and there is no public
/// constructor. "A safety may depend on an event; it may not depend on that
/// event for ever" therefore becomes unrepresentable rather than a rule someone
/// has to remember. Every hold carries a static reason string, which reaches the
/// field log and the bench `%SCHED` report -- where today every cause collapses
/// into the single word "no-schedule".
class ScheduleDecision {
public:
	enum class Kind : uint8_t {
		Run,           ///< Run in `delay_ms()` milliseconds.
		HoldUntil,     ///< Nothing to do before `epoch_s()`.
		HoldForEvent,  ///< Waiting for an event, re-evaluate within `max_hold_s()`.
		Off,           ///< Disabled by configuration.
	};

	/// @brief Lower bound on an event hold, so `hold_for_event(0, …)` cannot spin.
	static constexpr unsigned int MIN_HOLD_S = 30;
	/// @brief Upper bound, so a hold cannot become indistinguishable from Off.
	static constexpr unsigned int MAX_HOLD_S = 24 * 3600;

	/// @brief Run in `delay_ms` milliseconds.
	static ScheduleDecision run(unsigned int delay_ms, const char *why) {
		return ScheduleDecision(Kind::Run, delay_ms, why);
	}

	/// @brief Nothing to do until `epoch_s` (RTC seconds).
	/// Seconds since the epoch as uint32_t rather than std::time_t: the latter's
	/// width depends on the newlib configuration, which would grow this type on
	/// the target for no benefit. Good until 2106.
	static ScheduleDecision hold_until(uint32_t epoch_s, const char *why) {
		return ScheduleDecision(Kind::HoldUntil, epoch_s, why);
	}

	/// @brief Waiting for an event, but never for longer than `max_hold_s`.
	/// Clamped into [MIN_HOLD_S, MAX_HOLD_S] here rather than trusted, so a
	/// caller's zero or a caller's forever both become something the framework
	/// can honour.
	static ScheduleDecision hold_for_event(unsigned int max_hold_s, const char *why) {
		if (max_hold_s < MIN_HOLD_S) max_hold_s = MIN_HOLD_S;
		if (max_hold_s > MAX_HOLD_S) max_hold_s = MAX_HOLD_S;
		return ScheduleDecision(Kind::HoldForEvent, max_hold_s, why);
	}

	/// @brief Disabled by configuration. Nothing will be posted.
	static ScheduleDecision off(const char *why) { return ScheduleDecision(Kind::Off, 0, why); }

	Kind kind() const { return m_kind; }

	/// @brief Why, as a static string. Never null; reaches the log and %SCHED.
	const char *reason() const { return m_why; }

	/// @brief Valid for Kind::Run.
	unsigned int delay_ms() const { return m_value; }
	/// @brief Valid for Kind::HoldUntil.
	uint32_t epoch_s() const { return m_value; }
	/// @brief Valid for Kind::HoldForEvent.
	unsigned int max_hold_s() const { return m_value; }

	/// @brief True when the framework will bring the service back by itself.
	bool comes_back_on_its_own() const { return m_kind != Kind::Off; }

	/// @brief Short label for logs and the bench report.
	const char *kind_name() const {
		switch (m_kind) {
		case Kind::Run: return "run";
		case Kind::HoldUntil: return "hold-until";
		case Kind::HoldForEvent: return "hold-for-event";
		case Kind::Off: return "off";
		default: return "?";
		}
	}

private:
	ScheduleDecision(Kind kind, uint32_t value, const char *why)
	    : m_kind(kind),
	      m_value(value),
	      m_why(why ? why : "unspecified") {}

	Kind m_kind;
	uint32_t m_value;
	const char *m_why;
};

// A tagged struct, deliberately not a std::variant: variant brings index
// dispatch, valueless-by-exception paths and a per-visit instantiation, several
// hundred bytes of flash for nothing here. Twelve bytes on the target (one byte
// of kind, padding, a word, a pointer), sixteen on the 64-bit host build, and
// passed in registers or through sret either way.
static_assert(std::is_trivially_copyable_v<ScheduleDecision>,
              "ScheduleDecision is returned by value on every scheduling pass; keep it trivially copyable");
static_assert(sizeof(ScheduleDecision) <= 2 * sizeof(void *) + sizeof(uint32_t) + sizeof(uint32_t),
              "ScheduleDecision has grown; it is returned by value on a hot path");
