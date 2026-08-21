/**
 * @file lora_tx_service.hpp
 * @brief LoRa TX service — orchestrates LoRaWAN packet transmission as a Service.
 */

#pragma once

#include <ctime>
#include <optional>
#include <functional>
#include "kineis_device.hpp"
#include "service.hpp"
#include "config_store.hpp"
#include "service_scheduler.hpp"
#include "scheduler.hpp"
#include "depth_pile.hpp"
#include "lora_packet_builder.hpp"
#include "lora_tx_scheduler.hpp"

/// @brief LoRa TX service — builds and transmits packets via KineisDevice (LoRa backend).
class LoRaTxService : public Service, KineisEventListener {
public:
	LoRaTxService(KineisDevice& device);

	/// @brief Handle peer events (GPS fix, sensor data, underwater state).
	/// @param e  Peer service event.
	void notify_peer_event(ServiceEvent& e) override;

protected:
	void service_init() override;
	void service_term() override;
	bool service_is_enabled() override;
	unsigned int service_next_schedule_in_ms() override;
	void service_initiate() override;
	bool service_cancel() override;
	unsigned int service_next_timeout() override;
	bool service_is_triggered_on_surfaced(bool& immediate) override;
	bool service_is_active_on_initiate() override;

private:
	KineisDevice& m_device;
	DepthPileManager m_depth_pile_manager;
	LoRaTxScheduler m_sched;
	bool m_is_first_tx = true;
	bool m_is_tx_pending = false;
	unsigned int m_session_tx_count = 0;
	unsigned int m_consecutive_device_errors = 0;
	static constexpr unsigned int DEVICE_ERROR_MAX_CONSECUTIVE = 3;
	static constexpr unsigned int DEVICE_ERROR_BACKOFF_BASE_MS = 60000;   ///< 1 min
	static constexpr unsigned int DEVICE_ERROR_BACKOFF_MAX_MS  = 600000;  ///< 10 min

	/// @brief How long TX stays suspended after DEVICE_ERROR_MAX_CONSECUTIVE
	/// strikes, before one probe dispatch is allowed through.
	///
	/// This used to be forever: service_complete(no reschedule) plus a counter
	/// that only cleared on a successful TX, which could never happen because
	/// no TX was attempted. One bad patch of coverage killed the rest of the
	/// session, recoverable only by reboot — observed in the field as 18 h of
	/// "skipping TX" while GNSS kept taking fixes normally.
	///
	/// A periodic tracker (boat) has no surfacing events at all, so the probe is
	/// its only way back. One probe/h worst case is roughly one join window,
	/// which is negligible next to the GNSS duty cycle it runs alongside.
	/// Build-time override: LORA_TX_ERROR_SUSPEND_S (ports/nrf52840/CMakeLists.txt).
	/// 0 disables the suspension entirely, leaving only the capped exponential
	/// backoff — the bench/field-test setting, where an hour between retries
	/// makes iteration impossible. Host tests get the production default.
#ifndef LORA_TX_ERROR_SUSPEND_S
#define LORA_TX_ERROR_SUSPEND_S 3600
#endif
	static constexpr unsigned int DEVICE_ERROR_PROBE_PERIOD_S = LORA_TX_ERROR_SUSPEND_S;
	std::time_t m_device_error_suspend_until = 0;
	bool m_last_tx_had_gps = false;
	bool m_is_surfacing_burst = false;
	bool m_awaiting_surfacing = false;  ///< Burst ended, waiting for next surface event
	bool m_has_gnss_fix_since_surfacing = false;
	bool m_first_gnss_tx_sent = false;
	unsigned int m_status_burst_count = 0;

	/// @brief Edge-case flag for the GNSS_CLOUDLOCATE_READY event arriving
	/// while a TX is already in flight (`m_is_tx_pending == true`). In that
	/// situation we can't `service_reschedule(immediate=true)` right away
	/// (would corrupt the in-flight TX state machine). Instead we set this
	/// flag and check it in `react(KineisEventTxComplete)` — if set, we
	/// trigger the immediate reschedule there so the *next* TX is the
	/// "dans la foulée" CloudLocate the user expects, instead of waiting
	/// for the normal burst timer (init_s+step_s) to fire.
	bool m_cloudlocate_ready_pending = false;

	/// @brief Cooldown arming flag — mirrors ArgosTxService::m_cooldown_armed.
	/// Set when a condition matching COOLDOWN_TRIGGER_MODE is met during a
	/// surfacing cycle; the cooldown timer actually starts on the next dive
	/// event (notify_peer_event UW=true), keeping parity with Argos semantics.
	bool m_cooldown_armed = false;
	std::function<void()> m_scheduled_task;

	/// @brief Task handle for the delayed "cooldown-end warm-up". Scheduled
	/// on dive when the module is left off for a cooldown; fires at the end
	/// of the cooldown so the module is back in standby (configured, joined)
	/// before the next surfacing event, giving fast first-TX dispatch.
	Scheduler::TaskHandle m_cooldown_warm_up_task;

	/// @brief Task handle for the intra-burst pre-warm (lp_mode=0 only).
	/// Fires `BURST_PRE_WARM_DURATION_MS` before each scheduled SURFACING_BURST
	/// TX so the module is already booted by the time send() runs. Without
	/// this, lp_mode=0 stacks a 2.5 s boot delay on top of every burst step
	/// (interval becomes user_interval + 2.5 s). No-op when lp_mode=1
	/// (standby keeps the module ready, ~10 ms wake).
	Scheduler::TaskHandle m_burst_prewarm_task;

	/// @brief Boot budget for cold-start (power_off → power_on → configure →
	/// join check → idle → standby). Used to schedule the pre-warm task
	/// ahead of the next burst TX. Conservative: real RAK3172 cold-boot is
	/// ~2 s, we give 3 s of margin.
	static constexpr unsigned int BURST_PRE_WARM_DURATION_MS = 3000;

	/// @brief Arm / disarm the cooldown-end warm-up task. Cancels any pending
	/// warm-up task; if we're actually in a cooldown window and the module is
	/// currently off, schedules a new task at cooldown-end.
	void reschedule_cooldown_warm_up();

	/// @brief Arm the intra-burst pre-warm task for the next scheduled TX.
	/// Only schedules when lp_mode=0 and the burst is still active. Cancels
	/// any previously pending pre-warm task before re-arming.
	void schedule_burst_prewarm();

	void react(KineisEventTxStarted const&) override;
	void react(KineisEventTxComplete const&) override;
	void react(KineisEventDeviceError const&) override;

	void process_gps_burst();
	void process_sensor_burst();
	void process_status_burst();

	/// @brief Get max payload bytes for current DR setting.
	/// @return Max bytes from LoRaPayloadLimits.
	unsigned int get_max_payload_bytes();
};
