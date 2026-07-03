/**
 * @file gps_service.hpp
 * @brief GNSS acquisition service — periodic fix, cold start, fastloc, CloudLocate.
 */

#pragma once

#include <atomic>
#include <functional>

#include "gps.hpp"
#include "service.hpp"
#include "logger.hpp"
#include "timeutils.hpp"
#include "scheduler.hpp"


/// @brief CSV log formatter for GPS entries (used by DUMPD command).
class GPSLogFormatter : public LogFormatter {
public:
	const std::string header() override {
		return "log_datetime,batt_voltage,iTOW,fix_datetime,valid,onTime,ttff,fixType,flags,flags2,flags3,numSV,lon,lat,height,hMSL,hAcc,vAcc,velN,velE,velD,gSpeed,headMot,sAcc,headAcc,pDOP,vDOP,hDOP,headVeh\r\n";
	}
	const std::string log_entry(const LogEntry& e) override {
		char entry[512], d1[128], d2[128];
		const auto *gps = reinterpret_cast<const GPSLogEntry *>(&e);

		snprintf(d1, sizeof(d1), "%02hhu/%02hhu/%04hu %02hhu:%02hhu:%02hhu",
		        gps->header.day, gps->header.month, gps->header.year,
		        gps->header.hours, gps->header.minutes, gps->header.seconds);
        snprintf(d2, sizeof(d2), "%02hhu/%02hhu/%04hu %02hhu:%02hhu:%02hhu",
                gps->info.day, gps->info.month, gps->info.year,
                gps->info.hour, gps->info.min, gps->info.sec);

		// Convert to CSV
		snprintf(entry, sizeof(entry), "%s,%f,%u,%s,%u,%u,%u,%u,%u,%u,%u,%u,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n",
				d1,
				(double)gps->info.batt_voltage/1000,
				(unsigned int)gps->info.iTOW,
				d2,
				(unsigned int)gps->info.valid,
				(unsigned int)gps->info.onTime,
				(unsigned int)gps->info.ttff,
				(unsigned int)gps->info.fixType,
				(unsigned int)gps->info.flags,
				(unsigned int)gps->info.flags2,
				(unsigned int)gps->info.flags3,
				(unsigned int)gps->info.numSV,
				gps->info.lon,
				gps->info.lat,
				(double)gps->info.height / 1000,
				(double)gps->info.hMSL / 1000,
				(double)gps->info.hAcc / 1000,
				(double)gps->info.vAcc / 1000,
				(double)gps->info.velN / 1000,
				(double)gps->info.velE / 1000,
				(double)gps->info.velD / 1000,
				(double)gps->info.gSpeed / 1000,
				(double)gps->info.headMot,
				(double)gps->info.sAcc / 1000,
				(double)gps->info.headAcc,
				(double)gps->info.pDOP,
				(double)gps->info.vDOP,
				(double)gps->info.hDOP,
				(double)gps->info.headVeh);
		return std::string(entry);
	}
};

/// @brief GNSS acquisition service — periodic fixes, cold start, fastloc/CloudLocate fallback.
class GPSService : public Service, public GPSEventListener  {
public:
	GPSService(GPSDevice& device, Logger *logger) : Service(ServiceIdentifier::GNSS_SENSOR, "GNSS", logger), m_device(device) {
	    m_device.subscribe(*this);
	}
	void notify_peer_event(ServiceEvent& e) override;

	/// Manual / DTE entry-point for the V_BCKP coin-cell charge mode.
	/// duration_s > 0  → start (or extend) a charge session for the given seconds.
	/// duration_s == 0 → abort the current charge session immediately.
	/// Returns false if the request was refused (GNSS active or hardware not idle).
	bool request_backup_charge(unsigned int duration_s);

	/// True if charge is active OR pending (M10 powering off before retry).
	/// Used by gentracker's reed-switch handler to intercept events in both phases.
	bool is_backup_charge_active() const { return m_backup_active || m_pending_backup_duration_s > 0; }

	/// Set callbacks invoked on charge start and stop.
	/// on_start fires once when a new session begins (not on timer refresh).
	/// on_stop  fires on every stop (manual, timer, abort) and clears both callbacks.
	void set_backup_charge_callbacks(std::function<void()> on_start, std::function<void()> on_stop);

protected:

	// Service interface methods
	void service_init() override;
	void service_term() override;
	bool service_is_enabled() override;
	unsigned int service_next_schedule_in_ms() override;
	void service_initiate() override;
	bool service_cancel() override;
	unsigned int service_next_timeout() override;
	bool service_is_triggered_on_surfaced(bool &) override;
	bool service_is_usable_underwater() override;
	bool service_is_triggered_on_event(ServiceEvent&, bool&) override;

private:
	GPSDevice&   m_device;
	bool         m_is_first_fix_found = false;
	/// @brief One-shot request for a true cold start (CFG-RST navBbrMask=0xFFFF).
	/// Set by the GNP28 surface trigger (and any future periodic/stuck hook),
	/// consumed and cleared in service_initiate when building GPSNavSettings.
	bool         m_force_cold_start = false;
	bool         m_is_first_schedule = true;
	unsigned int m_cold_start_ntry = 0;  ///< Consecutive failed acquisitions (reset on fix or surface)
	uint64_t     m_wakeup_time = 0;
	std::time_t  m_next_schedule = 0;
	struct {
		GNSSData data;
	} m_gnss_data = {};
	unsigned int m_num_gps_fixes = 0;
	bool m_is_active = false;

	// Backup-cell (V_BCKP) charge mode bookkeeping (independent of m_is_active).
	bool m_backup_active = false;
	bool m_underwater = false;
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
	/// Gate: after surfacing, defer GNSS power-on until ArgosTxService completes
	/// its first satellite TX. Reduces CPU contention during the burst sequence.
	/// Set true on surface (in notify_peer_event), cleared on first ARGOS_TX SERVICE_INACTIVE.
	bool m_defer_gnss_until_argos_first_tx = false;
#endif
	unsigned int m_pending_backup_duration_s = 0;  ///< Set when waiting for M10 poweroff to retry
	Scheduler::TaskHandle m_backup_exit_task;     ///< Auto-exit timer once backup-charge is active
	Scheduler::TaskHandle m_backup_retry_task;    ///< Retry scheduler used while waiting for M10 poweroff
	// 2026-05 deep-idle refactor: removed m_backup_periodic_task,
	// backup_charge_schedule_next(), backup_charge_periodic_fire(). Recharge now
	// happens implicitly during the deep-idle window after each GPS session,
	// not via dedicated periodic cycles.
	std::function<void()> m_on_backup_charge_start;
	std::function<void()> m_on_backup_charge_stop;

	// 2026-05 deep-idle refactor — auto-poweroff timer after deep-idle window.
	// Armed when end-of-session enters deep-idle with a finite duration; fires
	// `m_device.power_off()` if the window elapses without intervening acquisition.
	// Cancelled at every service_initiate() so a new session re-engages cleanly.
	Scheduler::TaskHandle m_deep_idle_auto_off_task;

	// R5 robustness — timestamp (PMU::get_timestamp_ms) when the device entered
	// deep-idle. Used by service_next_schedule_in_ms to force a prophylactic
	// rail-cycle if the device has been in deep-idle > 24 h. 0 = not in deep-idle.
	uint64_t m_deep_idle_started_at_ms = 0;

	// R4 robustness gate (deep-idle plan): if the last reset was a WDT, inhibit
	// the deep-idle fast-path for the first GPS session post-boot. Cold-boot
	// proves the cold path works before re-engaging the optimization. Cleared
	// in react(GPSEventPVT) after first valid fix.
	bool m_deep_idle_inhibit_first_session = false;

	// Hard-cap timestamp for the WDT inhibit above. If a WDT reset arms the
	// inhibit but no PVT/CloudLocateReady ever fires (M10Q hardware degraded,
	// antenna issue, BBR loss), the inhibit would persist forever and disable
	// deep-idle for the rest of the deployment. After 24 h the dispatch path
	// force-clears the inhibit so V_BCKP can be recharged via the normal
	// deep-idle window. Stamped by set_deep_idle_inhibit_first_session(true),
	// reset by the same setter (with false) and by the two PVT/CloudLocate
	// clear sites. 0 = inhibit not armed.
	uint64_t m_inhibit_set_at_ms = 0;

	// Stuck-M10Q recovery: count consecutive end-of-session paths that produce
	// neither a PVT nor a CloudLocate raw measurement. After STUCK_THRESHOLD
	// dead sessions, schedule a hard rail-cycle (power_off_immediate + 30 s
	// + service_reschedule) to flush any latched M10Q hang state. The flag
	// m_stuck_recovery_in_flight prevents double-arming while a recovery is
	// already queued.
	unsigned int m_consecutive_dead_sessions = 0;
	bool m_stuck_recovery_in_flight = false;
	Scheduler::TaskHandle m_stuck_recovery_arm_task;
	Scheduler::TaskHandle m_stuck_recovery_done_task;
	static constexpr unsigned int STUCK_THRESHOLD = 20;

	// Safety net 3.5 — GPS Health WDT (no GPS event of any kind in N hours →
	// soft reset). Catches the scenario where the GPS service silently dies
	// while the scheduler is still alive (so the hardware WDT never fires).
	// Re-armed by every GPS event (PVT, degraded, CloudLocate, NO_FIX).
	// Threshold long (24h) to avoid false positives during legit long dives
	// or zone-exclusion windows where GNSS is disabled.
	Scheduler::TaskHandle m_health_wdt_task;
	static constexpr unsigned int HEALTH_WDT_HOURS = 24;

	// Safety net 3.6 — No real PVT in N days (CloudLocate doesn't count).
	// Catches the scenario where M10Q produces raw measurements (CloudLocate)
	// but never a real on-device PVT — typically when HACC/HDOP filters reject
	// every fix, leaving the tag trapped in CloudLocate-only mode forever.
	// 7-day threshold: any legit deployment should produce ≥1 real PVT/week.
	Scheduler::TaskHandle m_no_pvt_wdt_task;
	static constexpr unsigned int NO_PVT_WDT_DAYS = 7;

	// LED dispatch hint — records which branch try_enter_deep_idle_or_poweroff
	// took on its last invocation. Read by the no-fix react handlers to emit
	// GNSS_OFF_DEEP_IDLE or GNSS_OFF_POWEROFF so the LED FSM can render the
	// right pattern (double-blink red vs fast blink red). True = deep-idle
	// engaged (rail stays on, PMREQ-backup); false = full power-off.
	bool m_last_dispatch_was_deep_idle = false;
public:
	bool last_dispatch_was_deep_idle() const { return m_last_dispatch_was_deep_idle; }
#ifdef BENCH_TEST
	/// @brief Bench harness — inject a synthetic valid 3D fix at (lat,lon) straight
	/// into the post-fix pipeline (log → config-store → prepass → Argos scheduling),
	/// bypassing the M10Q entirely (no antenna / sky view required). Unlike the
	/// compile-fixed GPS_FAKE_POSITION driver hack (Saint-Paul only, auto-fires on
	/// every session), this is runtime-parameterised and fires on demand via the
	/// `%GPS` bench command. Compiled only with -DBENCH_TEST=ON; absent from
	/// production builds.
	/// @param lat      Latitude in degrees.
	/// @param lon      Longitude in degrees.
	/// @param hAcc_mm  Horizontal accuracy in mm (0 → default 2500 = 2.5 m).
	/// @param numSV    Satellites used (0 → default 8).
	void bench_inject_fix(double lat, double lon, uint32_t hAcc_mm, uint8_t numSV);
#endif
private:

	void backup_charge_stop_internal();
	void schedule_backup_charge_retry(unsigned int attempt);
	/// @brief Safety net 3.5 — (re-)arm the GPS-event health watchdog.
	/// Called from service_init() and every GPS-event callback.
	void arm_health_wdt();
	/// @brief Safety net 3.6 — (re-)arm the no-real-PVT watchdog.
	/// Called from service_init() and from gnss_data_callback() ONLY.
	void arm_no_pvt_wdt();
	/// @brief Dispatch end-of-session: enter deep-idle (with optional auto-off
	/// timer) or fall back to immediate power_off, based on
	/// GNSS_DEEP_IDLE_AFTER_OFF_S. Replaces the 7 raw `m_device.power_off()`
	/// call sites scattered through the React handlers.
	void try_enter_deep_idle_or_poweroff();

public:
	/// @brief R4 robustness — set by main.cpp on boot if reset cause was WDT.
	/// Disables deep-idle fast-path until first clean acquisition.
	/// Also stamps m_inhibit_set_at_ms so the 24h hard-cap in
	/// try_enter_deep_idle_or_poweroff() can force-clear the inhibit if no
	/// PVT/CloudLocate ever arrives. Defined inline in cpp because
	/// PMU::get_timestamp_ms() requires the pmu.hpp include.
	void set_deep_idle_inhibit_first_session(bool inhibit);

private:

    void react(const GPSEventMaxNavSamples&) override;
    void react(const GPSEventMaxSatSamples&) override;
    void react(const GPSEventPVT&) override;
    void react(const GPSEventPVTDegraded&) override;
    void react(const GPSEventRawMeasurement&) override;
    void react(const GPSEventCloudLocateReady&) override;
    void react(const GPSEventError&) override;
    void react(const GPSEventPowerOff&) override;

	// Private methods for GNSS
	void task_process_gnss_data();
	void task_process_degraded_gnss_data();
	void task_process_cloudlocate_data();
	GPSLogEntry invalid_log_entry();
	void gnss_data_callback(GNSSData data);
	void gnss_degraded_callback(GNSSData data);
	void gnss_cloudlocate_callback(GNSSRawMeasurement data);

	GNSSRawMeasurement m_raw_measurement;
};
