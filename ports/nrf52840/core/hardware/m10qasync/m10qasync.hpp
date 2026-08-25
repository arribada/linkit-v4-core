/**
 * @file m10qasync.hpp
 * @brief m10qasync driver.
 */

#pragma once

#include <cstdint>
#include "ubx_comms.hpp"
#include "gps.hpp"
#include "scheduler.hpp"

class M10QAsyncReceiver : public UBXCommsEventListener, public GPSDevice {
public:
	M10QAsyncReceiver();
	~M10QAsyncReceiver();
	void power_on(const GPSNavSettings& nav_settings) override;
	void power_off() override;
	void power_off_immediate() override;
	bool enter_backup_charge_mode() override;
	void exit_backup_charge_mode() override;
	/// @brief 2026-05 deep-idle refactor: report whether the M10Q is in PMREQ-backup
	/// with rail still on (state == backupidle or transitioning to it). Used by
	/// GPSService for the R5 hygiene check + service_initiate fast-path detection.
	bool is_in_deep_idle() const override {
		return m_state == State::backupidle || m_state == State::enterbackup;
	}
	/// @brief 2026-05 (CRITICAL #1 audit fix): arm deep-idle as the next stop
	/// disposition. Sets an intent flag that is honored in `state_poweroff()`:
	/// instead of cutting VDD via `enter_shutdown()`, the state machine routes
	/// `poweroff → enterbackup` keeping the rail on with the M10Q in PMREQ-backup.
	/// Replaces the old direct `enter_backup_charge_mode()` call from the end-of-
	/// session dispatch path which always failed (state != idle, users > 0).
	/// Must be called BEFORE `power_off()` so the flag is consumed during the
	/// power-down chain.
	void request_deep_idle_on_next_stop() override;
	/// @brief HIGH GNSS-AUDIT #2 follow-up: force-exit deep-idle by cutting rail.
	/// Used by GPSService's GNP52 auto-off timer. See gps.hpp doc.
	void poweroff_from_deep_idle() override;

private:
	GPSNavSettings m_nav_settings;
	UBXComms m_ubx_comms;
	uint8_t  m_navigation_database[16384];
	unsigned int m_ano_database_len;
    unsigned int m_ana_database_len;
	unsigned int m_expected_dbd_messages;
	unsigned int m_mga_ack_count;
	unsigned int m_step;
	unsigned int m_retries;
	unsigned int m_ano_start_pos;
	unsigned int m_uart_error_count;
	unsigned int m_num_consecutive_fixes;
	unsigned int m_num_power_on;
	unsigned int m_num_nav_samples;
	unsigned int m_num_sat_samples;
	bool m_powering_off;
	bool m_fix_was_found;
	bool m_unrecoverable_error;
	bool m_database_overflow;
	/// CRITICAL #1 fix: when true, the next `state_poweroff()` reroutes to
	/// `enterbackup` instead of calling `enter_shutdown()` (rail cut). Set via
	/// `request_deep_idle_on_next_stop()`. Single-shot: cleared upon consumption.
	bool m_deep_idle_pending = false;

	/// HIGH GNSS-AUDIT #1 follow-up: WARM-path indicator for `state_enterbackup_enter`.
	/// Set by `state_poweroff()` immediately before rerouting to `enterbackup` —
	/// signals that the rail is still powered and UART is still up, so the entry
	/// handler must SKIP the cold-reboot sequence (deinit + exit_shutdown).
	/// Previously detected via `GPIOPins::value(GPIO_GPS_PWR_EN)`, which can NEVER
	/// return non-zero on this BSP because `GPIO_GPS_PWR_EN` is configured with
	/// `NRF_GPIO_PIN_INPUT_DISCONNECT` → `nrf_gpio_pin_read()` returns 0
	/// regardless of the output drive state. Without this flag, every deep-idle
	/// dispatch fell into the COLD path, resetting the M10Q via exit_shutdown +
	/// NRST cycle, dropping us back to 9600 baud and failing PMREQ-backup
	/// sync ~50% of the time. Single-shot: cleared on consumption.
	bool m_enterbackup_warm = false;

	/// HIGH GNSS-AUDIT #3 follow-up: actual baud at which the last UBX-RXM-PMREQ
	/// backup was successfully sent — i.e. the baud the M10Q is now sleeping at
	/// and will resume at on next EXTINT wake (BBR-preserved by V_BCKP).
	///
	/// SOURCE OF TRUTH for EXTINT-wake baud pre-sync. Cannot be inferred from
	/// `m_gnss_info_valid` because the WARM enterbackup path can fall back from
	/// MAX to DEFAULT and succeed — leaving the M10Q at 9600 even when
	/// `m_gnss_info_valid==true`. Previously we hardcoded MAX_BAUDRATE at wake
	/// → mismatch → libuarte framing errors (type=0c) during the 50 ms wake
	/// window → state_configure: failed. Field log 2026-05-24 caught this in
	/// the dispatch-during-poweron/configure edge case where the M10Q was
	/// still at 9600 (cold boot baud) when enterbackup was triggered.
	///
	/// Updated on SUCCESS in `state_enterbackup` step 0/1 (whichever baud
	/// completed the sync that preceded send_pmreq_backup). Reset to DEFAULT
	/// in cold rail-cycle paths (where M10Q is forced back to factory baud).
	/// Initial value DEFAULT (9600) is the safe boot default.
	unsigned int m_pmreq_baud = 9600;

	/// 2026-08 cold-boot baud probe order. After a rail cut the M10Q can come
	/// back at EITHER baud:
	///   - 9600   : true factory POR — BBR was lost (backup cell empty).
	///   - 460800 : BBR RETAINED. `setup_uart_port()` writes CFG-UART1-BAUDRATE
	///              with layers = BBR|RAM, so once the LinkIt V4 backup cell /
	///              supercap on the GNSS rail holds BBR across the inter-session
	///              gap, every subsequent "cold" boot comes up at MAX_BAUDRATE
	///              and is SILENT at 9600 (NMEA off + nav messages off, both
	///              also persisted in BBR) — not even a framing error to hint
	///              at the mismatch.
	/// Probing 9600 only was therefore a PERMANENT GNSS death: `state_poweron`
	/// failed with "failed to sync comms" every session, and `state_configure` —
	/// the only place that renegotiates the port, and the only place that can
	/// issue the BBR-wiping CFG-RST — sits behind that sync. Field failure
	/// 2026-08-22 on the KIM tag (24 h of good fixes, then nothing).
	///
	/// Index into BOOT_BAUD_TABLE of the baud to probe FIRST; updated on every
	/// successful cold sync so a BBR-retaining board pays the failed probes
	/// only once per boot. Seeded from GNSS_HAS_BACKUP_BATTERY: that build flag
	/// is now only a HINT on the probe order (a wrong value costs ~3 s once per
	/// boot, nothing more) — never a gate on behaviour.
	unsigned int m_boot_baud_idx = 0;

	/// 2026-08: runtime proof that the M10Q kept its BBR across the last power
	/// transition. Set in `state_poweron` when the boot sync answered at
	/// MAX_BAUDRATE (the port config lives in the BBR layer, so answering there
	/// means the BBR survived), and unconditionally on the warm EXTINT wake
	/// (the rail never dropped). Cleared when the sync answered at 9600, i.e.
	/// the receiver came up on factory defaults.
	///
	/// This REPLACES `#if GNSS_HAS_BACKUP_BATTERY` as the gate for the
	/// configure fast path: whether a backup cell is fitted, charged, or dead
	/// is a per-session fact, not a build-time one — and the same firmware runs
	/// on boards with and without the cell.
	bool m_bbr_retained = false;

	/// Baud at which the M10Q is currently known to answer. Updated by the boot
	/// sync and by the warm-wake pre-sync. Used to put the UART back on a
	/// working rate when an optimistic probe (fast-path validation) fails —
	/// without it the driver kept talking at the probe's rate to a receiver
	/// that never heard it.
	unsigned int m_synced_baud = 9600;

	/// Masque de constellations effectivement accepte par le recepteur et donc
	/// present dans sa couche BBR. Compare au masque demande pour la session:
	/// le fast-path saute le step 2 (`setup_gnss_channel_sharing`), or ce masque
	/// varie par session (gps_service force |0x0F tant qu'aucun fix n'a ete
	/// obtenu). Sentinelle = on ignore ce que contient le recepteur.
	static constexpr unsigned int CONSTELLATION_MASK_UNKNOWN = 0xFFFFFFFFu;
	unsigned int m_applied_constellation_mask = CONSTELLATION_MASK_UNKNOWN;

	// GNSS device info (cached from configure phase)
	char m_gnss_sw_version[30];
	char m_gnss_hw_version[10];
	uint8_t m_gnss_unique_id[5];
	bool m_gnss_info_valid;
	/// @brief Consecutive fast-path (BBR-validation) failures at power-on. After
	/// FASTPATH_MAX_ATTEMPTS the code concludes the BBR did not retain state
	/// (dead/uncharged coin cell) and falls back to a full cold configure at 9600
	/// — see state_configure(). Reset on fast-path success / after the fallback.
	unsigned int m_fastpath_attempts = 0;

	// ISR-to-scheduler data buffers (replaces static locals to avoid data races)
	UBXCommsEventNavReport m_pending_nav;
	UBXCommsEventSatReport m_pending_sat;

	// Best degraded PVT: stores the best (lowest hAcc) 2D/3D fix that failed quality filters
	bool m_has_degraded_pvt;
	GNSSData m_degraded_pvt;

	// CloudLocate: latest raw GNSS measurement snapshot
	bool m_has_raw_measurement;
	GNSSRawMeasurement m_raw_measurement;
	/// @brief One-shot guard: set to true after we've emitted
	/// `GPSEventCloudLocateReady` for the current acquisition. Reset at the
	/// start of each new `power_on()`.
	bool m_cloudlocate_ready_notified;

	struct Timeout {
		Scheduler::TaskHandle handle;
	} m_timeout;
	Scheduler::TaskHandle m_state_machine_handle;

	enum State {
		idle,
		poweron,
		configure,
		senddatabase,
		sendofflinedatabase,
		startreceive,
		receive,
		stopreceive,
		fetchdatabase,
		poweroff,
		enterbackup,        ///< Powering on rail + sending UBX-RXM-PMREQ backup
		backupidle,         ///< Rail powered, UART torn down, M10 sleeping (~15 µA from V_BCKP)
	};

	enum OpState {
		IDLE,
		PENDING,
		SUCCESS,
		TIMEOUT,
		NACK,
		ERROR
	};

	State m_state = State::idle;
	/// 2026-08 (A1) — `volatile` obligatoire: ce champ est ecrit depuis le
	/// contexte ISR (react(SendComplete/AckNack/CfgValget/Error), appeles par le
	/// handler libuarte) et relu en boucle par les `state_*()` qui tournent en
	/// contexte principal. Sans qualificatif, rien n'interdit au compilateur de
	/// garder la valeur en registre d'une iteration de `while (true)` a l'autre:
	/// la reponse posee par l'ISR ne serait jamais vue et l'etat partirait en
	/// timeout alors que le recepteur a repondu.
	///
	/// Ce que cela ne resout PAS (documente sciemment): la sequence
	/// lecture-decision-ecriture des `state_*()` n'est pas atomique vis-a-vis de
	/// l'ISR. Une reponse tardive de l'operation precedente qui arriverait entre
	/// le passage a PENDING et l'emission suivante serait comptee pour la
	/// nouvelle operation. Le filtre d'attente (`m_expect`, desarme des qu'il a
	/// matche) rend le cas tres improbable; le corriger proprement demande de
	/// deporter les six handlers vers le scheduler comme NavReport/SatReport.
	volatile OpState m_op_state = OpState::IDLE;

	// State machine
	void state_idle_enter();
	void state_idle();
	void state_idle_exit();
	void state_poweron_enter();
	void state_poweron();
	unsigned int boot_baud_for_step(unsigned int step) const;
	static unsigned int boot_baud_retries(unsigned int step);
	void reset_session_state(const GPSNavSettings& nav_settings);
	void reset_session_counters(const GPSNavSettings& nav_settings);
	void state_poweron_exit();
	void state_configure_enter();
	void state_configure();
	void state_configure_exit();
	void state_senddatabase_enter();
	void state_senddatabase();
	void state_senddatabase_exit();
	void state_sendofflinedatabase_enter();
	void state_sendofflinedatabase();
	void state_sendofflinedatabase_exit();
	void state_startreceive_enter();
	void state_startreceive();
	void state_startreceive_exit();
	void state_receive_enter();
	void state_receive();
	void state_receive_exit();
	void state_stopreceive_enter();
	void state_stopreceive();
	void state_stopreceive_exit();
	void state_fetchdatabase_enter();
	void state_fetchdatabase();
	void state_fetchdatabase_exit();
	void state_poweroff_enter();
	void state_poweroff();
	void state_poweroff_exit();
	void state_enterbackup_enter();
	void state_enterbackup();
	void state_enterbackup_exit();
	void state_backupidle_enter();
	void state_backupidle();
	void state_backupidle_exit();
	void state_machine();
	void run_state_machine(unsigned int time_ms = 0);

	// Helpers
	void cancel_timeout();
	void initiate_timeout(unsigned int timeout_ms = 1000);
	void on_timeout();
	void save_config();
	/// @brief Efface la configuration sauvegardee en couche BBR du recepteur.
	///
	/// C'est la seule porte de sortie d'une couche BBR corrompue ou verrouillee.
	/// Jusqu'ici le firmware n'avait AUCUN moyen de l'effacer: `setup_uart_port`
	/// ecrit le debit en BBR|RAM, `save_config` recopie tout en BBR, `soft_reset`
	/// ne touche que les donnees de navigation (navBbrMask), aucun CFG-VALDEL
	/// n'etait instancie nulle part, et $FACTR n'agit pas sur le M10Q. Un debit
	/// exotique ecrit en BBR (via le bridge u-center par exemple) devenait donc
	/// un verrou definitif que la sonde de boot ne rattrape pas forcement.
	///
	/// Emis avec le CFG-RST d'un COLD START: on efface alors a la fois les
	/// donnees de navigation (navBbrMask=0xFFFF) et la configuration persistee,
	/// ce qui ramene le recepteur a un etat d'usine deterministe au prochain
	/// demarrage. La configuration RAM en cours n'est pas touchee (loadMask=0),
	/// la session continue donc normalement.
	void clear_config();
	void soft_reset();
	void setup_uart_port();
	void setup_gnss_channel_sharing();
	void setup_power_management();
	void setup_continuous_mode();
	void setup_simple_navigation_settings();
	void setup_expert_navigation_settings();
	void supply_time_assistance();
	void supply_position_assistance();
	void disable_odometer();
	void disable_timepulse_output();
	void enable_nav_pvt_message();
	void enable_nav_dop_message();
	void enable_nav_status_message();
    void enable_nav_sat_message();
	void disable_nav_pvt_message();
	void disable_nav_dop_message();
	void disable_nav_status_message();
    void disable_nav_sat_message();
	void enable_rxm_measc12_message();
	void enable_rxm_meas20_message();
	void enable_rxm_meas50_message();
	void fetch_navigation_database();
	void query_mon_ver();
	void query_sec_uniqid();
	void sync_baud_rate(unsigned int baud);
	void send_pmreq_backup();
	void pulse_extint_wake();   ///< 2026-05 deep-idle: pulse EXTINT to wake M10Q from PMREQ-backup

	// 2026-05 deep-idle robustness: counter of consecutive wake-from-deep-idle
	// failures. Incremented before each attempt, reset on first successful
	// PVT (proves the M10Q actually woke). After WAKE_FAIL_FAST_FALLBACK
	// attempts, force a cold-boot path (rail-cycle) instead of the EXTINT
	// fast-path for the rest of the session — defends against the case where
	// the EXTINT pin is somehow not waking the M10Q (HW fault, BBR lost +
	// wrong baud assumed, etc.). The counter is also session-scoped: a reset
	// or service_init clears it.
	uint8_t m_consecutive_wake_failures = 0;
	static constexpr uint8_t WAKE_FAIL_FAST_FALLBACK = 2;

	/// 2026-05-25 PMREQ-backup verification: counter for inline retry attempts
	/// when the M10Q refuses to enter PMREQ-backup (i.e. it still responds to
	/// a probe poll right after PMREQ was sent). Reset at every
	/// `state_enterbackup_enter` so each new dive cycle gets a fresh budget.
	/// Catches the silent-failure mode observed 2026-05-25 where PMREQ is sent
	/// but the M10Q stays in PSM/Sleep (~2 mA) instead of entering true backup
	/// (~10 µA).
	uint8_t m_pmreq_verify_retries = 0;

	/// 2026-05-25 wake diagnostic: timestamp (ms since boot via PMU) of the
	/// most recent transition into `backupidle`. Used by `power_on` to log how
	/// long the M10Q was supposed to be sleeping when an EXTINT wake fires —
	/// a very-short delta (e.g. <5 s within a 5-min GNP52 window) indicates a
	/// spurious wake from some caller of `power_on()` that didn't intend to
	/// cut deep-idle short. Zero = never entered backupidle this session.
	uint64_t m_backupidle_entered_ms = 0;
	void dump_navigation_database(unsigned int);
	void save_dbd_to_flash();
	bool load_dbd_from_flash();

	// Power management
	void enter_shutdown();
	void exit_shutdown();
	void check_for_power_off();
#ifdef GPS_FAKE_POSITION
	void generate_fake_fix();
#endif

	// Events
	void react(const UBXCommsEventSendComplete&) override;
	void react(const UBXCommsEventAckNack&) override;
	void react(const UBXCommsEventCfgValget&) override;
    void react(const UBXCommsEventSatReport& sat) override;
	void react(const UBXCommsEventNavReport&) override;
	void react(const UBXCommsEventMgaAck&) override;
	void react(const UBXCommsEventMgaDBD&) override;
	void react(const UBXCommsEventMonVer&) override;
	void react(const UBXCommsEventSecUniqId&) override;
	void react(const UBXCommsEventRawMeasurement&) override;
	void react(const UBXCommsEventDebug&) override;
	void react(const UBXCommsEventError&) override;

public:
	GNSSDeviceInfo get_device_info() const override;
	GNSSAlmanacStatus get_almanac_status(unsigned int ano_stale_threshold_s = 25 * 24 * 3600) const override;
	bool has_degraded_pvt() const override { return m_has_degraded_pvt; }
	GNSSData get_degraded_pvt() const override { return m_degraded_pvt; }
	bool has_raw_measurement() const override { return m_has_raw_measurement; }
	GNSSRawMeasurement get_raw_measurement() const override { return m_raw_measurement; }

	// Bridge/passthrough mode: direct USB ↔ GNSS UART access
	bool start_bridge(PassthroughCallback rx_callback) override;
	void stop_bridge() override;
	bool is_bridge_active() const override { return m_bridge_active; }
	bool is_powered() const override { return m_num_power_on > 0 || m_state != State::idle; }
	bool bridge_send(const uint8_t* data, size_t len) override;
	void bridge_process_rx() override;

private:
	bool m_bridge_active = false;
};
