/**
 * @file argos_tx_service.hpp
 * @brief Argos TX service — orchestrates satellite packet transmission as a Service.
 */

#pragma once

#include <ctime>
#include <optional>
#include "kineis_device.hpp"
#include "service.hpp"
#include "config_store.hpp"
#include "service_scheduler.hpp"
#include "depth_pile.hpp"
#include "argos_packet_builder.hpp"
#include "argos_tx_scheduler.hpp"
#include "messages.hpp"

/// @brief Argos satellite TX service — builds and transmits packets via KineisDevice.
class ArgosTxService : public Service, KineisEventListener {
public:
	ArgosTxService(KineisDevice &device);
	void notify_peer_event(ServiceEvent &e) override;

	// Compute the age in seconds of a GPS log entry against a reference RTC time.
	// Returns UINT_MAX when the entry timestamp is invalid (zero year) or in the
	// future relative to `now` — both indicate an entry the REUSE_LAST path must
	// not trust. Pure / static so unit tests can exercise it without an instance.
	static unsigned int compute_gps_log_age_seconds(const GPSLogEntry &entry, std::time_t now);

#ifdef BENCH_TEST
	/// @brief Bench probe: depth-pile contents as "<type>:<burst_counter>" per slot.
	/// burst_counter is what retrieve() decrements, and an entry at 0 is never
	/// eligible again -- the only way to observe a position being consumed
	/// without ever being encoded into a packet.
	std::string bench_dump_pile() { return m_depth_pile_manager.bench_dump_gps(); }
#endif

protected:
	void service_init() override;
	void service_term() override;
	bool service_is_enabled() override;
	unsigned int service_next_schedule_in_ms() override;

	/// @name Per-mode scheduling
	/// One method per ARGOS_MODE branch. service_next_schedule_in_ms() runs the
	/// gates that apply to every mode -- cooldown, rate limit, prepass, critical
	/// battery, certification -- and then dispatches here exactly once. These do
	/// NOT re-run those gates.
	/// @{
	unsigned int schedule_doppler(ArgosConfig &argos_config, std::time_t now);
	unsigned int schedule_surfacing_burst(ArgosConfig &argos_config, std::time_t now);
	unsigned int schedule_without_gnss(ArgosConfig &argos_config, std::time_t now);
	unsigned int schedule_with_gnss(ArgosConfig &argos_config, std::time_t now);
	/// @}
	void service_initiate() override;
	bool service_cancel() override;
	unsigned int service_next_timeout() override;
	bool service_is_triggered_on_surfaced(bool &immediate) override;
	bool service_is_active_on_initiate() override;

private:
	KineisDevice &m_kineis;
	DepthPileManager m_depth_pile_manager;
	ArgosTxScheduler m_sched;
	bool m_is_first_tx = true;
	bool m_is_tx_pending = false;
	// First-message gate (non-RSPB, per boot): set true once a valid GPS fix
	// (GPSEventType::FIX) has corrected the clock this power session. Until then,
	// LEGACY/DUTY_CYCLE/PASS_PREDICTION hold ALL TX (time-sync burst, NO_FIX
	// heartbeat and position) so nothing is transmitted on a DTE/pseudo-RTC clock
	// before the GPS has actually fixed in the field. RSPB is exempt at compile
	// time (#ifndef BOARD_RSPB): a boot-modulo session that ends with no fix must
	// still send an empty (0xFF) position + sensor packet. Reset per boot via the
	// member initializer (the service is constructed fresh at each power-on).
	bool m_gps_fix_corrected_clock = false;
	bool m_tcxo_skip_on_next_tx = false;
	unsigned int m_session_tx_count = 0;
	std::function<void()> m_scheduled_task;
	KineisModulation m_scheduled_mode = KineisModulation::LDA2;

	// Surfacing burst state
	bool m_is_surfacing_burst = false;
	bool m_awaiting_surfacing = false;  ///< Burst ended, waiting for next surface event
	unsigned int m_doppler_burst_count = 0;
	bool m_has_gnss_fix_since_surfacing = false;
	bool m_first_gnss_tx_sent = false;
	bool m_last_tx_had_gps = false;
	bool m_cooldown_armed = false;

	// DOPPLER burst-pattern state (2026-05). Independent from SURFACING_BURST
	// state above. Counter of messages sent in the current DOPPLER sequence;
	// reset to 0 when the sequence ends (count >= SURFACING_BURST_MAX_MSG).
	// max_msg == 0 means unbounded sequence (progressive spacing keeps growing
	// until capped at surfacing_burst_max_s — equivalent to a continuous TX
	// with progressive period).
	unsigned int m_doppler_seq_count = 0;
	// Absolute RTC time (seconds) at which the inter-sequence pause ends.
	// 0 = not in a pause. Used to protect the pause against rearming when an
	// external event (GPS log, UW surfaced) fires reschedule before the pause
	// has elapsed — without this guard, the pause would be silently reset
	// and the next sequence would start immediately.
	std::time_t m_doppler_pause_until_rtc = 0;

	// Pre-deploy validation channel — populated by process_*_burst just before
	// m_kineis.send() and consumed by the TxComplete handler to emit a
	// [VAL-TX] line with type + spacing. Cheap unconditional storage (~16 B)
	// even when VALIDATION_LOG_ENABLE is off — keeps the header stable.
	const char *m_last_val_tx_type = "none";
	std::time_t m_last_val_tx_t = 0;

	// Pre-warm of the first surfacing-burst Doppler packet. While underwater
	// (only in SURFACING_BURST mode) the battery is sampled and the Doppler
	// payload is built so the first TX at surface skips the ADC read + packet
	// build on its critical path. Refreshed if the prep is older than 1h.
	bool m_is_underwater = false;
	KineisPacket m_prepared_doppler_packet;
	unsigned int m_prepared_doppler_size_bits = 0;
	KineisModulation m_prepared_doppler_mode = KineisModulation::LDA2;
	uint64_t m_prepared_at_ms = 0;
	static constexpr uint64_t PREPARED_DOPPLER_REFRESH_MS = 3600000ULL;  ///< 1 hour

	void react(KineisEventTxStarted const &) override;
	void react(KineisEventTxComplete const &) override;
	void react(KineisEventDeviceError const &) override;

	void process_certification_burst();
	void process_time_sync_burst();
	void process_gnss_burst();
	void process_sensor_burst();
	void process_doppler_burst();
	void prepare_doppler_packet();

	// BaseGnssStrategy::REUSE_LAST dispatch (Plan 1 follow-up). Builds a GNSS
	// Argos packet from the most recent cached depth-pile fix without powering
	// the GPS. Falls back to process_doppler_burst() when no usable cached
	// fix exists (pile empty, fix too old, or not a real FIX/UPDATE entry).
	void process_gnss_burst_from_cached();

	// Spacing guard (2026-05): minimum interval between any two TX, based on
	// uptime (monotonic, immune to RTC rollback). Updated in
	// react(KineisEventTxComplete). Used by service_next_schedule_in_ms when
	// it would otherwise return an "immediate" schedule (0 ms) at transitions
	// like Doppler→GNSS or FastLoc→Doppler. Surfacing_burst_init_s acts as
	// the minimum-spacing value (≥ 5 s expected).
	uint64_t m_last_tx_uptime_ms = 0;

	// Returns the proposed_delay_ms clamped so the resulting TX time is at
	// least min_spacing_s seconds after m_last_tx_uptime_ms. If clamped,
	// also reschedules m_sched at the deferred time. No-op if no prior TX
	// has completed (m_last_tx_uptime_ms == 0).
	unsigned int apply_spacing_guard(unsigned int proposed_delay_ms, unsigned int min_spacing_s, std::time_t now);

	// Clamp a BURST schedule to the device-error backoff/suspension.
	//
	// The periodic and prepass schedulers get that delay through
	// ArgosTxScheduler::set_earliest_schedule, but SURFACING_BURST drives itself
	// with schedule_at(), which writes the next TX instant directly and never
	// reads that floor. Without this the ladder was pure logging in the mode the
	// turtles actually run: the WARN said "backoff 60000 ms" while the next
	// Doppler went out on the burst's own 5 s cadence.
	//
	// Returns the delay the caller should return, in ms: the proposed one when
	// no hold is pending or the burst is already slower than it, otherwise the
	// hold. Re-anchors the scheduler, exactly like apply_spacing_guard.
	unsigned int apply_device_error_hold(unsigned int proposed_delay_ms, std::time_t now, bool &held);

	// FastLoc priority (2026-05): peek depth pile; if the latest entry is a
	// FastLoc (or real FIX/UPDATE) less than max_age_s old, returns true and
	// the caller should route to process_gnss_burst instead of
	// process_doppler_burst. The FastLoc occupies what would have been a
	// Doppler TX slot.
	bool should_promote_doppler_to_gnss(unsigned int max_age_s);


	/// @brief Why are the AOP unusable? Three clearly distinct causes:
	/// merging them into a single message makes field diagnosis misleading
	/// ("age=0 s ... expired" means nothing).
	enum class AopStatus { USABLE, NO_RECORD, RTC_UNSET, NO_DATE, EXPIRED };

	/// @brief Are the AOP usable? (present, dated, not expired)
	/// @param[out] age_s AOP age in seconds (0 if unknown)
	/// Without valid AOP no window can be computed: the caller must then
	/// fall back to periodic transmission rather than staying silent.
	AopStatus aop_status(const ArgosConfig &config, std::time_t now, unsigned int &age_s);
	bool aop_is_usable(const ArgosConfig &config, std::time_t now, unsigned int &age_s);
	static const char *aop_status_text(AopStatus e);

	/// @brief Updates the status parameters readable by STATR.
	void refresh_prepass_status(const ArgosConfig &config, std::time_t now, std::time_t next_pass_epoch);

	// BaseGnssStrategy::REUSE_LAST plumbing: read the most recent depth-pile fix
	// if it is fresh enough per ParamID::GNSS_REUSE_FIX_MAX_AGE_S. Returns false
	// when the pile is empty, the latest entry is not a real fix, the entry is
	// older than the configured threshold, or reuse is disabled (threshold = 0).
	// Consumed by process_gnss_burst_from_cached (HAULED REUSE_LAST path).
	bool read_cached_last_fix(GPSLogEntry &out);

	// Adaptive modulation: switch RCONF if needed before TX
	bool ensure_modulation(KineisModulation target);
	std::string get_rconf_for_modulation(KineisModulation mode);

	// @brief Modulation to use when adaptive is OFF. The user's master RCONF
	// (ARGOS_RADIOCONF) is encrypted hex — we can't tell locally which
	// modulation it encodes. The device layer (KIM2) reads back AT+RCONF=? at
	// init and caches the actual modulation, exposed via get_current_modulation().
	// SMD doesn't auto-detect (m_modulation stays at LDA2 default), so SMD
	// users keep today's behavior. Falls back to LDA2 on first cold boot
	// before init has run.
	KineisModulation resolve_non_adaptive_modulation();

	/// @brief Modulation the adaptive Doppler phase transmits on.
	/// VLDA4 where the backend permits it, LDA2 where it does not (KIM2 gates
	/// VLDA4 on a 27 dBm regulatory check and refuses to transmit it). Adaptive
	/// mode selected VLDA4 unconditionally at six separate sites before this.
	KineisModulation adaptive_doppler_modulation() const;

	// @brief Bitmask of modulations whose per-mod RCONF is present (32-char hex)
	// in the config store. Used by burst processors to skip a TX cleanly when
	// the would-be fallback modulation can't hold the payload (instead of
	// hitting KIM2's silent payload-too-long drop + 30 s service timeout).
	// Computed at service_init() and on every scheduling cycle so runtime
	// PARMW edits are reflected. Bits: 0=LDK, 1=LDA2, 2=VLDA4.
	uint8_t m_modulation_avail_mask = 0;
	void refresh_modulation_availability();
	bool is_modulation_provisioned(KineisModulation mode) const;
	static bool size_fits_modulation(unsigned int payload_bits, KineisModulation mode);
	/// @brief Can this modulation carry this frame, on this backend?
	/// Size is not the only reason a modulation can be unusable -- KIM2 refuses
	/// VLDA4 outright -- and folding both into one predicate lets the existing
	/// size fallback, which reprograms the module, cover the permission case too.
	static bool can_transmit_on(KineisModulation mode, unsigned int payload_bits);
	KineisModulation m_last_preconfig_mod = KineisModulation::LDA2;
	std::optional<KineisModulation> m_modulation_preconfig;

	// Deferral applied when the device is busy receiving. Re-checked at this
	// cadence until the RX window closes; each re-check costs one scheduling
	// cycle, so keep it well above a second and well below a full RX window.
	static constexpr unsigned int ARGOS_TX_RX_BUSY_DEFER_S = 60;

	// Device error backoff
	static constexpr unsigned int DEVICE_ERROR_MAX_CONSECUTIVE = 3;
	static constexpr unsigned int DEVICE_ERROR_BACKOFF_BASE_MS = 60000;
	static constexpr unsigned int DEVICE_ERROR_BACKOFF_MAX_MS = 600000;
	unsigned int m_consecutive_device_errors = 0;

	/// @brief How long TX stays suspended after DEVICE_ERROR_MAX_CONSECUTIVE
	/// strikes, before one probe dispatch is allowed through.
	///
	/// This used to be "until something else happens": the suspension called
	/// service_complete(no reschedule), which is also the only path that
	/// cancels the safety-net timeout armed before service_initiate(). The
	/// timeout therefore outlived the suspension, fired, rescheduled at zero
	/// delay, hit the guard again and rearmed itself -- a wake/log/skip loop
	/// once per timeout that transmitted nothing and wrote an LFS commit every
	/// pass. Worse, that loop was what made the "recovers on the next GPS
	/// session" promise true, so the defect was load-bearing.
	///
	/// A land tracker (RSPB) has no surfacing events, so before this the probe
	/// did not exist and a transient module fault cost every transmission until
	/// the next GPS log. LoRaTxService has had the deadline since its own field
	/// outage; this is the same design, same default.
	/// Build-time override: ARGOS_TX_ERROR_SUSPEND_S (ports/nrf52840/CMakeLists.txt).
	/// 0 disables the suspension entirely, leaving only the capped backoff.
#ifndef ARGOS_TX_ERROR_SUSPEND_S
#define ARGOS_TX_ERROR_SUSPEND_S 3600
#endif
	static constexpr unsigned int DEVICE_ERROR_PROBE_PERIOD_S = ARGOS_TX_ERROR_SUSPEND_S;

	/// @brief Epoch second before which no TX may go out, 0 when none pending.
	/// Written by both device-error regimes -- the backoff sets it to
	/// now + the ladder value, the suspension to now + DEVICE_ERROR_PROBE_PERIOD_S
	/// -- so the guard in service_initiate() and the burst paths read one value
	/// rather than each keeping their own idea of when TX may resume. Cleared by
	/// a successful TX and by every event that clears the strike counter.
	std::time_t m_device_error_hold_until = 0;
};
