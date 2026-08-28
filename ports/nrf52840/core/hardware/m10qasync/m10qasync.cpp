/**
 * @file m10qasync.cpp
 * @brief m10qasync driver.
 */

#include "m10qasync.hpp"
#include "gpio.hpp"
#include "bsp.hpp"
#include <vector>
#include "ubx_comms.hpp"
#include "ubx.hpp"
#include "pmu.hpp"
#include "debug.hpp"
#include "scheduler.hpp"
#include "error.hpp"
#include "rtc.hpp"
#include "timeutils.hpp"
#include "binascii.hpp"
#include "interrupt_lock.hpp"
#include "config_store.hpp"
#include "hauled_mode_service.hpp"
#include "rate_limiter.hpp"

using namespace UBX;

extern Scheduler *system_scheduler;
extern RTC *rtc;
extern ConfigurationStore *configuration_store;
extern FileSystem *main_filesystem;

// Required baud rates
#define DEFAULT_BAUDRATE 9600
#define MAX_BAUDRATE     460800

// Tunable parameters (named constants, see embedded-qa review C1)
static constexpr unsigned int DEFAULT_RETRIES = 3;         ///< Default retry budget for op_state machines
static constexpr unsigned int BOOT_BAUD_SYNC_RETRIES = 6;  ///< Wider retry budget on initial baud sync
static constexpr unsigned int PMREQ_VERIFY_RETRIES =
    3;  ///< 2026-05-25: PMREQ-backup verification retries (ping M10Q after PMREQ, re-send if replies)
static constexpr unsigned int PMREQ_VERIFY_TIMEOUT_MS =
    200;  ///< Wait for verification poll response (TIMEOUT = backup confirmed, SUCCESS = M10Q still awake)
static constexpr unsigned int EARLY_ABORT_SAT_REPORTS =
    20;  ///< Sat reports w/ qualityInd==0 before aborting (~20s @ 1Hz)
static constexpr unsigned int NAV_REPORT_WATCHDOG_MS =
    5000;  ///< Inter-NAV-message UART watchdog while in receive state
static constexpr unsigned int RECEIVE_TIMEOUT_FLOOR_MS = 10000;  ///< Minimum initial receive timeout (cold start floor)
static constexpr unsigned int RECEIVE_TIMEOUT_MARGIN_S =
    5;  ///< Margin added to max_nav_samples for initial receive timeout
static constexpr unsigned int MAX_FRAMING_ERRORS_BOOT = 10;  ///< Tolerance for NMEA→UBX framing errors during boot
// Cold-boot baud probe list. A rail-cycled M10Q answers at 9600 only when BBR
// was actually lost; when the backup cell / supercap on the GNSS rail holds BBR
// across the off window it comes back at the baud `setup_uart_port()` persisted
// there (MAX_BAUDRATE) and stays SILENT at 9600 — no NMEA, no nav messages, so
// not even a framing error to hint at the mismatch. Probe both, in cached order.
// Les quatre debits que `UBXComms::set_baudrate` sait produire sont couverts, et
// pas seulement les deux que le firmware programme lui-meme: le bridge u-center
// donne un acces brut au recepteur, donc un VALSET operateur peut persister
// n'importe lequel en couche BBR. Budget degressif (6 essais sur le debit mis en
// cache, 2 sur les suivants) pour que le pire cas reste ~6 s, inchange.
static constexpr unsigned int BOOT_BAUD_TABLE[] = { DEFAULT_BAUDRATE, MAX_BAUDRATE, 38400, 921600 };
static constexpr unsigned int BOOT_BAUD_COUNT = sizeof(BOOT_BAUD_TABLE) / sizeof(BOOT_BAUD_TABLE[0]);
static constexpr unsigned int BOOT_BAUD_ALT_RETRIES = 2;  ///< Budget par debit au-dela du premier
// Max age of the persisted DBD navigation database before we refuse to reload
// it on a cold power-on. Raised 12h -> 72h (2026-06): per u-blox MAX-M10S
// Integration Manual the DBD carries almanac (valid WEEKS) and AssistNow
// Autonomous predictions (valid ~3-6 days) in addition to broadcast ephemeris
// (only ~4h). Capping at 12h threw away the still-valid almanac/autonomous data
// on every >12h dive gap, forcing a cold reacquisition the receiver could not
// complete in a short surface window. The receiver itself discards the stale
// ephemeris portion on load, so restoring an aged DBD is safe and preserves the
// long-lived data that makes warm starts possible across multi-day deployments.
// 2026-08 — plafond d'age du DBD persiste, desormais fonction de la carte.
//
// Le DBD porte trois choses d'esperances de vie tres differentes: les ephemerides
// diffusees (~4 h, que le recepteur ecarte LUI-MEME au chargement), les
// predictions AssistNow Autonomous (3 a 6 jours) et l'almanach (des SEMAINES).
// Un plafond unique de 72 h jetait donc l'almanach encore valide avec les
// ephemerides perimees.
//
// Le bon plafond depend de ce que la carte sait conserver par ailleurs:
//   - BBR retenue (pile/supercap V_BCKP qui tient): le recepteur garde sa propre
//     copie vivante, le fichier n'est qu'une ceinture de secours. On reste serre.
//   - BBR perdue a chaque coupure (pas de pile): le fichier est le SEUL vecteur
//     de demarrage chaud persistant. On garde tout ce qui peut encore servir.
static constexpr unsigned int DBD_MAX_AGE_BBR_S = 72 * 3600;          ///< 3 j — carte a BBR retenue
static constexpr unsigned int DBD_MAX_AGE_NO_BBR_S = 14 * 24 * 3600;  ///< 14 j — carte sans pile V_BCKP

// State machine helper macros
#ifndef VALIDATION_LOG_ENABLE
#define VALIDATION_LOG_ENABLE 0
#endif

#if VALIDATION_LOG_ENABLE
#define VAL_GNSS_STATE_CHANGE(x, y)                                               \
	DEBUG_INFO("[VAL-GNSS] state " #x "->" #y " users=%u t=%llu", m_num_power_on, \
	           (unsigned long long)PMU::get_timestamp_ms())
#define VAL_GNSS(fmt, ...) DEBUG_INFO("[VAL-GNSS] " fmt, ##__VA_ARGS__)
#else
#define VAL_GNSS_STATE_CHANGE(x, y) \
	do {                            \
	} while (0)
#define VAL_GNSS(fmt, ...) \
	do {                   \
	} while (0)
#endif

#define STATE_CHANGE(x, y)                                             \
	do {                                                               \
		DEBUG_TRACE("M10QAsyncReceiver::STATE_CHANGE: " #x " -> " #y); \
		VAL_GNSS_STATE_CHANGE(x, y);                                   \
		m_state = y;                                                   \
		state_##x##_exit();                                            \
		state_##y##_enter();                                           \
		run_state_machine();                                           \
	} while (0)

#define STATE_EQUAL(x) (m_state == x)

#define STATE_CALL(x) \
	do {              \
		state_##x();  \
	} while (0)


M10QAsyncReceiver::M10QAsyncReceiver() {
	STATE_CHANGE(idle, idle);
	m_ana_database_len = 0;
	m_ano_database_len = 0;
	m_ano_start_pos = 0;
	m_timeout.handle = {};
	m_num_power_on = 0;
	m_powering_off = false;
	m_num_nav_samples = 0;
	m_num_sat_samples = 0;
	m_fix_was_found = false;
	m_unrecoverable_error = false;
	m_database_overflow = false;
	m_has_degraded_pvt = false;
	std::memset(&m_degraded_pvt, 0, sizeof(m_degraded_pvt));
	m_uart_error_count = 0;
	m_expected_dbd_messages = 0;
	m_mga_ack_count = 0;
	m_step = 0;
	m_retries = 0;
	m_gnss_info_valid = false;
#if GNSS_HAS_BACKUP_BATTERY
	// Le flag de build n'est plus qu'un INDICE sur l'ordre de sondage du baud au
	// boot : une carte equipee d'une pile de sauvegarde repart en general a
	// MAX_BAUDRATE, on la sonde donc en premier. Un indice faux coute ~3 s une
	// fois par boot, rien de plus — plus aucun comportement n'en depend.
	m_boot_baud_idx = 1;  // index de MAX_BAUDRATE dans BOOT_BAUD_TABLE
#endif
	std::memset(m_gnss_sw_version, 0, sizeof(m_gnss_sw_version));
	std::memset(m_gnss_hw_version, 0, sizeof(m_gnss_hw_version));
	std::memset(m_gnss_unique_id, 0, sizeof(m_gnss_unique_id));
}

M10QAsyncReceiver::~M10QAsyncReceiver() {}

/// @brief Remet a zero TOUT l'etat propre a une session et applique les
/// nouveaux reglages de navigation.
///
/// 2026-08 : ce bloc ne vivait que dans le chemin froid de `power_on()`. Le
/// reveil chaud depuis `backupidle` (deep-idle, GNP52 != 0) sortait par un
/// `return` anticipe et ne le traversait jamais, donc :
///   - `m_nav_settings` restait celui de la session precedente (cold_start,
///     cloudlocate_enable, masque de constellations, filtres hAcc/hDOP,
///     timeouts : tout fige, et une demande de cold start perdue... ou au
///     contraire rejouee a l'infini) ;
///   - `m_num_nav_samples` / `m_num_sat_samples` cumulaient d'une session a
///     l'autre, donc le budget d'acquisition et la fenetre de grace de
///     l'abort "aucun satellite" fondaient jusqu'a zero ;
///   - `m_raw_measurement` / `m_degraded_pvt` n'etaient pas purges, si bien
///     qu'une mesure CloudLocate de la session PRECEDENTE pouvait etre
///     re-emise comme si elle etait fraiche.
/// Ce qui doit survivre entre sessions (m_pmreq_baud, m_gnss_info_valid,
/// m_boot_baud_idx, m_bbr_retained, m_consecutive_wake_failures) n'est
/// deliberement pas touche ici.
/// @brief Remet a zero les compteurs et les resultats d'une session, et applique
/// les seules limites d'observation. N'ecrase PAS m_nav_settings.
///
/// Separe de `reset_session_state()` pour le cas d'un `power_on()` supplementaire
/// sur une session deja en cours (2e client: PWRON GNSS ou GNSSI depuis le mode
/// configuration). Y remplacer l'integralite des reglages changerait filtres,
/// masque de constellations et mode CloudLocate au milieu d'une acquisition, et
/// ces appelants passent un GPSNavSettings partiellement initialise. On garde
/// donc la semantique d'origine: les compteurs et les limites suivent le nouvel
/// appelant, la configuration de navigation reste celle de la session.
void M10QAsyncReceiver::reset_session_counters(const GPSNavSettings &nav_settings) {
	m_powering_off = false;
	m_deep_idle_pending = false;

	m_has_degraded_pvt = false;
	std::memset(&m_degraded_pvt, 0, sizeof(m_degraded_pvt));

	m_has_raw_measurement = false;
	m_raw_measurement = GNSSRawMeasurement();
	m_cloudlocate_ready_notified = false;

	m_num_nav_samples = 0;
	m_num_sat_samples = 0;
	m_fix_was_found = false;
	m_rtc_persisted_this_session = false;

	m_nav_settings.max_nav_samples = nav_settings.max_nav_samples;
	m_nav_settings.max_sat_samples = nav_settings.max_sat_samples;
	m_num_consecutive_fixes = m_nav_settings.num_consecutive_fixes;
}

void M10QAsyncReceiver::reset_session_state(const GPSNavSettings &nav_settings) {
	reset_session_counters(nav_settings);
	// Et, contrairement au cas du client supplementaire, on applique bien
	// l'integralite des reglages demandes pour cette session.
	m_nav_settings = nav_settings;
	m_num_consecutive_fixes = m_nav_settings.num_consecutive_fixes;
}

void M10QAsyncReceiver::power_on(const GPSNavSettings &nav_settings) {
	DEBUG_INFO("M10QAsyncReceiver::power_on");

	// 2026-05 deep-idle refactor: fast-path wake from PMREQ-backup.
	//
	// If we're in `backupidle` (M10Q in PMREQ-backup, rail still on, BBR
	// intact), pulse EXTINT to wake the M10Q in place — no rail cycle,
	// no cold boot. Sync UART at MAX_BAUDRATE (BBR retains the configured
	// baud) and transit directly to `configure` which will fast-path via
	// the BBR-validation step. TTFF should drop to warm-start range.
	//
	// If we're in `enterbackup` (mid-transition into deep-idle, e.g. the
	// PMREQ message hasn't fully been processed by the M10Q yet), it's
	// safer to fall back to the legacy synchronous teardown — we don't
	// know the M10Q state with certainty. The state machine will then
	// proceed via the normal poweron → configure path.
	if (STATE_EQUAL(backupidle)) {
		// 2026-05 robustness: after WAKE_FAIL_FAST_FALLBACK consecutive
		// failures on this boot session, the wake-from-deep-idle fast-path
		// has proven unreliable (HW or M10Q state). Force a cold rail-cycle
		// — the known-working path — instead of attempting the same fast
		// path again. Counter is reset on first successful PVT (see
		// react(UBXCommsEventNavReport) and state_configure SUCCESS path).
		if (m_consecutive_wake_failures >= WAKE_FAIL_FAST_FALLBACK) {
			DEBUG_WARN("M10QAsyncReceiver::power_on: %u consecutive wake-fail — forcing cold rail-cycle",
			           m_consecutive_wake_failures);
			// CRITICAL #2 audit fix: cancel pending state-machine tick + timeout
			// BEFORE tearing down comms. Otherwise a previously-scheduled
			// `run_state_machine()` task (e.g. from state_enterbackup) fires
			// on the freshly-reset state with stale m_step / m_op_state →
			// libuarte double-init / use-after-deinit → HardFault candidate.
			cancel_timeout();
			system_scheduler->cancel_task(m_state_machine_handle);
			VAL_GNSS("power_on abort=cold_rail_cycle (cancelled pending tasks)");
			m_ubx_comms.deinit();
			GPIOPins::clear(BSP::GPIO::GPIO_GPS_PWR_EN);
			GPIOPins::release_to_highz(BSP::GPIO::GPIO_GPS_RST);
			GPIOPins::release_to_highz(BSP::GPIO::GPIO_GPS_EXT_INT);
			GPIOPins::release_sensors_pwr();
			PMU::delay_ms(50);
			// HIGH #4 audit fix: direct m_state write is safe here ONLY because
			// we just cancelled `m_state_machine_handle` + `m_timeout` above.
			// No scheduled tick can observe a torn state machine. Do NOT
			// remove the cancel calls — they are the precondition for this
			// direct assignment instead of STATE_CHANGE().
			m_state = State::idle;
			m_num_power_on = 0;
			// HIGH GNSS-AUDIT #3 follow-up: rail was cut here, M10Q lost BBR
			// and will boot fresh at 9600 factory default. Stale m_pmreq_baud
			// (likely MAX from before) would mislead the next session.
			m_pmreq_baud = DEFAULT_BAUDRATE;
			// Fall through to the normal cold power-on path below.
		} else {
			// 2026-05-25 wake diagnostic: report how long the M10Q actually
			// slept since state_backupidle_enter. A short delta (e.g. <5 s
			// within a multi-minute GNP52 window) means someone called
			// `power_on()` unexpectedly soon — the next investigation step is
			// to identify the caller from the surrounding log context (likely
			// GPSService scheduler retry or a peer event).
			uint64_t now_ms = PMU::get_timestamp_ms();
			uint64_t slept_ms = (m_backupidle_entered_ms > 0 && now_ms >= m_backupidle_entered_ms)
			                        ? (now_ms - m_backupidle_entered_ms)
			                        : 0;
			DEBUG_INFO("M10QAsyncReceiver::power_on: wake from deep-idle via EXTINT (fail_count=%u, slept_ms=%llu)",
			           m_consecutive_wake_failures, (unsigned long long)slept_ms);
			// M10Q wakes on EXTINT rising edge. Re-init UART driver (was
			// deinit'd in state_enterbackup step 3 to save the UART block
			// power during deep-idle). Then pulse EXTINT and let the
			// M10Q boot into its post-wake state. The configure state's
			// BBR fast-path will validate at MAX_BAUDRATE.
			//
			// Pre-increment the wake-fail counter so even if we hang
			// mid-sequence (unlikely — every step has a timeout) the
			// counter still grows on next attempt. Reset to 0 on first
			// successful UBX response (configure's NAV-PVT or earlier
			// BBR validation acks). NOTE: if the M10Q does wake but the
			// configure path subsequently fails for unrelated reasons,
			// the counter is conservatively NOT reset — better to force
			// a cold-boot once than ignore a real wake issue.
			m_consecutive_wake_failures++;
			// HIGH #5 audit fix: libuarte allocator can throw
			// ErrorCode::RESOURCE_NOT_AVAILABLE on transient failure. Without
			// try/catch the exception propagates upward unwound through
			// service_initiate → scheduler → unhandled std::exception →
			// GenTracker::dispatch(ErrorEvent) → ErrorState transition. That's
			// a heavy reaction for a transient init failure. Catch + mark
			// unrecoverable + force the next iteration through the
			// m_consecutive_wake_failures >= WAKE_FAIL_FAST_FALLBACK branch
			// (cold rail-cycle path) which is the known-good recovery.
			try {
				m_ubx_comms.init();
				m_ubx_comms.subscribe(*this);
				m_ubx_comms.set_debug_enable(m_nav_settings.debug_enable);
			} catch (...) {
				DEBUG_ERROR("M10QAsyncReceiver::power_on: ubx_comms.init failed during wake — forcing cold next time");
				VAL_GNSS("ubx_init_throw fast_path -> force cold next");
				// Push the counter to threshold so the very next power_on falls
				// into the cold-rail-cycle branch.
				m_consecutive_wake_failures = WAKE_FAIL_FAST_FALLBACK;
				m_unrecoverable_error = true;
				notify(GPSEventError{});
				// Meme motif que les quatre autres sites: on est en backupidle,
				// rail ALLUME et users=0. Sans coupure autonome, si aucun abonne
				// n'agit (PWRON pendant une charge DTE, GNSS_EN=0) le rail reste
				// alimente indefiniment. m_unrecoverable_error force la
				// transition backupidle -> poweroff via le switch de dispatch.
				check_for_power_off();
				return;
			}
			// HIGH GNSS-AUDIT #3 follow-up (revised): pre-sync our UART to
			// `m_pmreq_baud` — the baud at which the last PMREQ-backup was
			// actually sent. That's what the M10Q is sleeping at, preserved
			// by V_BCKP across the deep-idle window. Using a hardcoded MAX
			// was WRONG: if the WARM enterbackup fell back from MAX to 9600
			// (e.g. dispatch during configure-in-progress where M10Q was
			// still at boot baud), the M10Q resumes at 9600 but we'd try
			// to read at MAX → garbage → framing errors (type=0c) → 50 ms
			// of noise → state_configure: failed.
			//
			// Now we track the actual baud at SUCCESS in state_enterbackup
			// (see m_pmreq_baud doc in header). If BBR was lost during a
			// long deep-idle (V_BCKP discharged), the configure fast-path
			// validation at our pre-set baud will fail and fall back to the
			// full-config (9600) path — same recovery as before.
			m_ubx_comms.set_baudrate(m_pmreq_baud);
			VAL_GNSS("extint_wake_presync baud=%u", m_pmreq_baud);
			pulse_extint_wake();
			PMU::delay_ms(50);   // give M10Q time to come out of backup (datasheet ~30 ms)
			m_num_power_on = 1;  // single user (this acquisition)
			// 2026-08 : le rail n'est jamais tombe, donc la BBR est intacte par
			// construction. On ne revendique la retention QUE si le M10Q dormait
			// deja a MAX_BAUDRATE : c'est la preuve qu'il avait bien recu notre
			// configuration complete. S'il dormait a 9600 (PMREQ envoye pendant
			// un boot, avant setup_uart_port) le rail a beau etre reste allume,
			// le recepteur n'est PAS configure -> config complete obligatoire.
			m_synced_baud = m_pmreq_baud;
			m_bbr_retained = (m_pmreq_baud == MAX_BAUDRATE);
			// 2026-08 : appliquer les reglages de CETTE session et repartir de
			// compteurs propres. Sans cet appel le reveil chaud heritait de tout
			// l'etat de la session precedente (cf. reset_session_state).
			reset_session_state(nav_settings);
			// Skip the normal poweron-state baud-sync loop (state_poweron tries
			// 9600 first which would fail since M10Q is at MAX after BBR-warm
			// wake). Jump straight to configure where the fast-path tries MAX.
			STATE_CHANGE(backupidle, configure);
			return;
		}
	} else if (STATE_EQUAL(enterbackup)) {
		DEBUG_INFO("M10QAsyncReceiver::power_on: aborting enterbackup (mid-transition) → cold reboot");
		// CRITICAL #2 audit fix: cancel pending state-machine tick + timeout
		// BEFORE tearing down comms. The enterbackup state has a queued tick
		// (`run_state_machine(100)` after `send_pmreq_backup`) that, if it
		// fires after this reset, runs on stale m_step / m_op_state →
		// libuarte double-init / use-after-deinit → HardFault candidate.
		cancel_timeout();
		system_scheduler->cancel_task(m_state_machine_handle);
		VAL_GNSS("power_on abort=enterbackup_mid_transition (cancelled pending tasks)");
		m_ubx_comms.deinit();
		GPIOPins::clear(BSP::GPIO::GPIO_GPS_PWR_EN);
		GPIOPins::release_to_highz(BSP::GPIO::GPIO_GPS_RST);
		GPIOPins::release_to_highz(BSP::GPIO::GPIO_GPS_EXT_INT);
		GPIOPins::release_sensors_pwr();
		PMU::delay_ms(50);  // brief rail-off gap before exit_shutdown re-energises it
		// HIGH #4 audit fix: direct m_state write is safe here ONLY because
		// we cancelled the state machine task + timeout above. Don't remove.
		m_state = State::idle;
		m_num_power_on = 0;
		// HIGH GNSS-AUDIT #3 follow-up: rail was just cut, M10Q will cold-boot
		// at 9600 factory default. Reset tracker so the next session reads
		// the right baud at any subsequent EXTINT wake.
		m_pmreq_baud = DEFAULT_BAUDRATE;
	}

	// Track number of power on requests
	m_num_power_on++;

	// Depuis `idle`, c'est une vraie nouvelle session: compteurs remis a zero ET
	// reglages de navigation appliques. Hors `idle` (2e client sur une session en
	// cours), on ne touche qu'aux compteurs et aux limites — voir
	// reset_session_counters().
	if (STATE_EQUAL(idle))
		reset_session_state(nav_settings);
	else
		reset_session_counters(nav_settings);

	if (STATE_EQUAL(idle)) {
#ifdef GPS_FAKE_POSITION
		// Fake GPS mode: simulate a fix after 2s delay
		DEBUG_WARN("M10QAsyncReceiver: FAKE GPS MODE - will simulate fix at Saint-Paul Reunion");
		m_fix_was_found = false;
		notify<GPSEventPowerOn>({});
		m_state_machine_handle =
		    system_scheduler->post_task_prio([this]() { generate_fake_fix(); }, "FakeGPSFix", 2000);
#else
		STATE_CHANGE(idle, poweron);
#endif
	}
}

void M10QAsyncReceiver::check_for_power_off() {
	if (m_powering_off) return;

	// On unrecoverable error, force shutdown regardless of client count —
	// the GPS hardware is in a broken state and cannot serve any client.
	// Dispatch through the same switch as the clean-shutdown path so the
	// CURRENT state's exit handler runs (STATE_CHANGE(idle, poweroff) would
	// always call state_idle_exit, skipping cleanup of receive/configure/etc.).
	if (m_unrecoverable_error) {
		m_powering_off = true;
		m_num_power_on = 0;
	}

	// Don't power off if there are still active clients (and not error path)
	if (m_num_power_on && !m_unrecoverable_error) return;

	// Try to cleanup in a way that will preserve navigation data
	// before powering off
	m_powering_off = true;

	// Try to shutdown cleanly — dispatch the exit handler matching CURRENT
	// state, not a fixed literal in STATE_CHANGE. Earlier the `else` fallback
	// at the bottom called STATE_CHANGE(idle, poweroff) which expanded to
	// state_idle_exit() regardless of m_state — leaking stopreceive's
	// cancel_timeout(), fetchdatabase's stop_dbd_filter(), etc.
	switch (m_state) {
	case idle:
		// Une intention de deep-idle armee alors qu'il n'y a rien a eteindre
		// ne doit pas survivre: elle rerouterait un power_off ULTERIEUR vers
		// enterbackup sans que personne l'ait demande — rail rallume, et sans
		// minuterie puisque le hard-cap exige m_deep_idle_started_at_ms > 0.
		m_deep_idle_pending = false;
		return;  // already idle, nothing to clean up
	case receive: STATE_CHANGE(receive, stopreceive); break;
	case poweron: STATE_CHANGE(poweron, poweroff); break;
	case configure: STATE_CHANGE(configure, poweroff); break;
	case startreceive: STATE_CHANGE(startreceive, poweroff); break;
	case senddatabase: STATE_CHANGE(senddatabase, poweroff); break;
	case sendofflinedatabase: STATE_CHANGE(sendofflinedatabase, poweroff); break;
	case backupidle: STATE_CHANGE(backupidle, poweroff); break;
	case enterbackup: STATE_CHANGE(enterbackup, poweroff); break;
	case stopreceive: STATE_CHANGE(stopreceive, poweroff); break;
	case fetchdatabase: STATE_CHANGE(fetchdatabase, poweroff); break;
	case poweroff:
		// Already in poweroff state — nothing to do
		return;
	default:
		// Should never reach here — enum is fully covered above. Guard
		// against future state additions: walk through state_idle_exit
		// so we at least power down cleanly.
		DEBUG_WARN("M10QAsyncReceiver::check_for_power_off: unhandled state %d", static_cast<int>(m_state));
		STATE_CHANGE(idle, poweroff);
		break;
	}
}

#ifdef GPS_FAKE_POSITION
void M10QAsyncReceiver::generate_fake_fix() {
	DEBUG_INFO("M10QAsyncReceiver: Generating FAKE GPS fix");
	std::time_t now = rtc->gettime();
	struct tm *t = gmtime(&now);

	GNSSData gnss_data = {};
	gnss_data.fixType = 3;   // 3D fix
	gnss_data.valid = 0x07;  // validDate | validTime | fullyResolved
	gnss_data.numSV = 8;
	gnss_data.lat = -21.0097;  // Saint-Paul de la Reunion
	gnss_data.lon = 55.2707;
	gnss_data.height = 10000;  // 10m (mm)
	gnss_data.hMSL = 10000;
	gnss_data.hAcc = 2500;  // 2.5m (mm)
	gnss_data.vAcc = 3500;  // 3.5m (mm)
	gnss_data.pDOP = 1.5f;
	gnss_data.hDOP = 1.0f;
	gnss_data.vDOP = 1.2f;
	gnss_data.ttff = 2000;  // 2s simulated TTFF
	gnss_data.year = (uint16_t)(t->tm_year + 1900);
	gnss_data.month = (uint8_t)(t->tm_mon + 1);
	gnss_data.day = (uint8_t)t->tm_mday;
	gnss_data.hour = (uint8_t)t->tm_hour;
	gnss_data.min = (uint8_t)t->tm_min;
	gnss_data.sec = (uint8_t)t->tm_sec;

	m_fix_was_found = true;
	notify(GPSEventPVT(gnss_data));
}
#endif

void M10QAsyncReceiver::power_off() {
	DEBUG_INFO("M10QAsyncReceiver::power_off");
	VAL_GNSS("power_off users=%u->%u state=%d", m_num_power_on, m_num_power_on ? m_num_power_on - 1 : 0, (int)m_state);
	// MED #7 audit fix: warn if power_off called with users already at 0 —
	// indicates an upstream double-power_off bug (would normally be silent
	// because of the `if` guard). Latent today (single client), but a future
	// bridge-mode addition could trigger this.
	if (m_num_power_on == 0) {
		DEBUG_WARN("M10QAsyncReceiver::power_off: called with users=0 (no-op, possible upstream bug)");
		VAL_GNSS("power_off_excess users=0 (upstream double-call)");
	}
	// Just decrement the number of users
	if (m_num_power_on) m_num_power_on--;
#ifdef GPS_FAKE_POSITION
	system_scheduler->cancel_task(m_state_machine_handle);
	notify(GPSEventPowerOff(m_fix_was_found));
#else
	check_for_power_off();
#endif
}

/// 2026-05-25 Fix #10 — Hard shutdown bypassing the graceful FSM cascade.
///
/// Symptom that motivated this: a user-triggered reed power-off (transit to
/// OffState) fired WHILE the M10Q was mid-boot (state=poweron/configure, UART
/// boot transition pending after a fresh rail-up). OperationalState::exit →
/// ServiceManager::stopall → GPSService::service_term → m_device.power_off()
/// → check_for_power_off() → state_stopreceive → UBX commands waiting for
/// ACKs from a half-booted M10Q that never replies properly → 15+ min hang
/// before WDT rescues. During the hang the device is in System ON with full
/// peripherals running and does NOT respond to the magnet — sealed-device UX
/// disaster (looks bricked to the user).
///
/// This method skips the entire graceful cascade and goes straight to rail-
/// cut. Trade-off acknowledged with user 2026-05-25: V_BCKP / BBR is lost
/// (next session will cold-start GNSS at 9600 baud) but the device exits
/// cleanly and the next boot starts from a known-good state.
///
/// Safety for "everything restarts cleanly next boot":
///   - Hardware: `enter_shutdown()` cuts PWR_EN, asserts NRST low for 50 ms
///     to fully discharge VDD caps, then releases NRST + EXTINT to high-Z.
///     Next power_on() sees a true power-on reset on the M10Q.
///   - Software state: PMU::powerdown() that follows this in the OffState
///     path triggers a SoC soft-reset; all static/global objects are
///     re-constructed at the next boot. We still reset the instance state
///     here defensively in case a caller invokes this without subsequently
///     soft-resetting (e.g. ConfigurationState entry, BatteryCriticalState).
void M10QAsyncReceiver::power_off_immediate() {
	DEBUG_INFO("M10QAsyncReceiver::power_off_immediate: hard rail-cut (skip graceful FSM)");
	VAL_GNSS("power_off_immediate from_state=%d users=%u", (int)m_state, m_num_power_on);

	// Cancel every pending scheduler-driven work item so no zombie task fires
	// after the rail is dead (would touch a deinit'd UART → exception →
	// ServiceManager::stopall catch → continue, but cleaner to pre-empt).
	cancel_timeout();
	system_scheduler->cancel_task(m_state_machine_handle);

	// Rail-cut + RST/EXTINT release + 50 ms VDD discharge. This is the same
	// sequence the legacy `state_poweroff()` chain ends at — we just skip
	// the asynchronous nav-message disable / fetchdatabase steps that can
	// block on a half-booted M10Q.
	enter_shutdown();

	// Reset instance state. After PMU::powerdown soft-reset these get
	// re-initialized by the constructor anyway, but ConfigurationState entry
	// and a few other paths re-use the same instance — they must see a clean
	// baseline so the next power_on() takes the cold-boot path correctly.
	m_state = State::idle;
	m_num_power_on = 0;
	m_op_state = OpState::IDLE;
	m_step = 0;
	m_powering_off = false;
	m_deep_idle_pending = false;
	m_enterbackup_warm = false;
	m_consecutive_wake_failures = 0;
	m_pmreq_verify_retries = 0;
	m_backupidle_entered_ms = 0;
	// Le rail vient d'etre coupe: la BBR n'est plus garantie et le M10Q ne dort
	// plus au debit du dernier PMREQ.
	m_pmreq_baud = DEFAULT_BAUDRATE;
	m_bbr_retained = false;
	// 2026-06 latch fix: clear the unrecoverable-error flag on a hard rail-cut
	// so the next session starts clean. Otherwise a latched error (set on a
	// receive/configure failure and never cleared on the warm backupidle->
	// configure wake path) would force power-off every session and only ever
	// recover via the 20-dead-session stuck-recovery — i.e. "GPS dies, never
	// recovers" while the device (Doppler TX) stays alive.
	m_unrecoverable_error = false;
	// m_gnss_info_valid intentionally left untouched — the cached baud
	// hint helps the next session even after a cold rail-cut (BBR may have
	// survived if the rail dropped before V_BCKP discharged).
}

void M10QAsyncReceiver::request_deep_idle_on_next_stop() {
	DEBUG_INFO("M10QAsyncReceiver::request_deep_idle_on_next_stop");
	VAL_GNSS("deep_idle_armed (next stop -> enterbackup)");
	m_deep_idle_pending = true;
}

// HIGH GNSS-AUDIT #2 follow-up: previously the GNP52 auto-off timer called
// `power_off()`, which is a no-op when `m_num_power_on == 0` (the dispatch
// already decremented users at deep-idle entry). Result: the timer fired,
// logged "auto_off_timer_fired", but the rail stayed on indefinitely — V_BCKP
// kept charging at ~µA but the M10Q itself kept burning its backup current
// (~10-12 µA) forever. This method does the actual rail teardown, allowing
// the GNP52 duration to be honored.
void M10QAsyncReceiver::poweroff_from_deep_idle() {
	if (!STATE_EQUAL(backupidle) && !STATE_EQUAL(enterbackup)) {
		DEBUG_TRACE("M10QAsyncReceiver::poweroff_from_deep_idle: not in deep-idle "
		            "(state=%d) — no-op",
		            (int)m_state);
		VAL_GNSS("poweroff_from_deep_idle skipped state=%d", (int)m_state);
		return;
	}
	DEBUG_INFO("M10QAsyncReceiver::poweroff_from_deep_idle: cutting rail (state=%d)", (int)m_state);
	VAL_GNSS("poweroff_from_deep_idle cutting_rail state=%d", (int)m_state);

	// Cancel any pending state-machine ticks. In backupidle there should be
	// none, but enterbackup-in-progress could still have a queued tick from
	// `run_state_machine(100)` after send_pmreq_backup — same hazard as
	// CRITICAL #2 audit fix in power_on cold-rail-cycle path.
	cancel_timeout();
	system_scheduler->cancel_task(m_state_machine_handle);

	// Tear down the rail. UART was deinit'd in enterbackup step 3, but
	// enter_shutdown's deinit is idempotent so a double-call is safe.
	enter_shutdown();

	notify(GPSEventPowerOff(m_fix_was_found));
	m_state = State::idle;
	m_num_power_on = 0;
	// HIGH GNSS-AUDIT #3 follow-up: rail is now off — V_BCKP will discharge
	// through the M10Q backup load over minutes/hours. Once BBR is lost,
	// the next cold boot lands at 9600 factory default. Reset the tracker
	// so we don't try to wake at a stale MAX baud.
	m_pmreq_baud = DEFAULT_BAUDRATE;
}

bool M10QAsyncReceiver::enter_backup_charge_mode() {
	DEBUG_INFO("M10QAsyncReceiver::enter_backup_charge_mode");
	// Refuse if GPS already has active clients or is not idle — backup charge
	// and normal acquisition are mutually exclusive. Caller (GPSService) must
	// gate this.
	if (m_num_power_on != 0 || !STATE_EQUAL(idle)) {
		DEBUG_WARN("M10QAsyncReceiver: backup-charge refused (state=%d, users=%u)", (int)m_state, m_num_power_on);
		VAL_GNSS("backup_charge refused state=%d users=%u", (int)m_state, m_num_power_on);
		return false;
	}
	VAL_GNSS("backup_charge accepted (idle, users=0)");
	m_powering_off = false;
	STATE_CHANGE(idle, enterbackup);
	return true;
}

void M10QAsyncReceiver::exit_backup_charge_mode() {
	DEBUG_INFO("M10QAsyncReceiver::exit_backup_charge_mode");
	VAL_GNSS("backup_charge exit (was state=%d users=%u)", (int)m_state, m_num_power_on);
	if (!STATE_EQUAL(backupidle) && !STATE_EQUAL(enterbackup)) return;
	m_num_power_on = 0;
	// Cancel any in-flight response timeout / scheduled state-machine task
	cancel_timeout();
	system_scheduler->cancel_task(m_state_machine_handle);
	// For enterbackup, UART is still alive — deinit it (libuarte tears down
	// pending TX cleanly). For backupidle, it's already deinit'd.
	if (STATE_EQUAL(enterbackup)) {
		m_ubx_comms.deinit();
	}
	// Kill the rail synchronously so callers can immediately request a normal
	// power_on (state == idle on return).
	GPIOPins::clear(BSP::GPIO::GPIO_GPS_PWR_EN);
	GPIOPins::release_to_highz(BSP::GPIO::GPIO_GPS_RST);
	GPIOPins::release_sensors_pwr();
	m_state = State::idle;
	notify(GPSEventPowerOff(false));
}

void M10QAsyncReceiver::enter_shutdown() {
	m_ubx_comms.deinit();

	// Assert NRST LOW BEFORE cutting VDD so the M10Q sees a clean reset edge
	// — without this, releasing NRST to high-Z while VDD is still up can leave
	// the chip in an indeterminate state if the next power_on happens before
	// caps fully discharge.
	GPIOPins::init_pin(BSP::GPIO::GPIO_GPS_RST);
	GPIOPins::clear(BSP::GPIO::GPIO_GPS_RST);

	// Disable the power supply for the GPS
	GPIOPins::clear(BSP::GPIO::GPIO_GPS_PWR_EN);

	// VDD cap discharge wait — without this, a rapid power_off → power_on
	// cycle (e.g. deep-idle refused → fallback power_off → next surface a few
	// seconds later) leaves residual VDD on the M10Q decoupling caps. The
	// next power_on doesn't trigger a true POR → M10Q stays in some
	// intermediate state → `failed to sync comms` cascade.
	// 50 ms matches the SMD VDD_DISCHARGE_FAST_MS pattern. Cost is amortized
	// — it happens just before sleep/idle, not on the latency-critical path.
	PMU::delay_ms(50);

	// Now release NRST so the pin doesn't sink current via the external
	// pull-up while the rail is dead.
	GPIOPins::release_to_highz(BSP::GPIO::GPIO_GPS_RST);  // Disconnect (ext pull-up)

	// 2026-05 deep-idle refactor: release EXTINT to high-Z so the OUTPUT
	// pin (BSP default direction since the refactor) doesn't sink/source
	// current into a powered-off M10Q input pin. Zero leakage between
	// sessions when GPS rail is cut.
	GPIOPins::release_to_highz(BSP::GPIO::GPIO_GPS_EXT_INT);

	// Note: m_ubx_comms.deinit() already disconnects UART TX/RX pins
	// via nrf_gpio_cfg_default() in the libuarte driver uninit path.

	// Release VSENSORS - GPS requires VSENSORS rail to be stable
	GPIOPins::release_sensors_pwr();
}

void M10QAsyncReceiver::exit_shutdown() {
	// Configure UBX comms if not already done.
	// HIGH #5 audit fix: catch libuarte init failure (RESOURCE_NOT_AVAILABLE).
	// Setting m_unrecoverable_error here cascades into state_poweron's step
	// `m_step==1` branch which already calls `notify(GPSEventError{})`.
	try {
		m_ubx_comms.init();
		m_ubx_comms.subscribe(*this);
		m_ubx_comms.set_debug_enable(m_nav_settings.debug_enable);
	} catch (...) {
		DEBUG_ERROR("M10QAsyncReceiver::exit_shutdown: ubx_comms.init failed");
		VAL_GNSS("ubx_init_throw cold_path -> unrecoverable_error");
		m_unrecoverable_error = true;
		// No further GPIO work — leave rail off so next session retries cleanly.
		return;
	}

	// Acquire VSENSORS before GPS power-on to ensure stable power rail
	GPIOPins::acquire_sensors_pwr();

	// Hold NRST asserted (LOW) BEFORE bringing VDD up — mirrors the SMD
	// state_powering_on pattern. Without this, the M10Q's NRST line gets
	// pulled HIGH by the external pull-up the moment VDD ramps, and the
	// chip starts booting at an unstable voltage. With NRST held LOW
	// through the ramp, the M10Q stays in reset until we explicitly
	// release — clean POR on release, no marginal-voltage boot.
	GPIOPins::init_pin(BSP::GPIO::GPIO_GPS_RST);  // reconfigure as OUTPUT (S0D1 open-drain)
	GPIOPins::clear(BSP::GPIO::GPIO_GPS_RST);     // drive LOW → NRST asserted (reset held)
	PMU::delay_ms(2);                             // small settle for the drive
	GPIOPins::set(BSP::GPIO::GPIO_GPS_PWR_EN);    // VDD ON with M10Q held in reset
	PMU::delay_ms(20);                            // VDD ramp + stabilize while in reset
	GPIOPins::set(BSP::GPIO::GPIO_GPS_RST);       // high-Z → ext pull-up → NRST released → POR
	PMU::delay_ms(80);                            // M10Q boot ~30ms, 80ms margin (sync_baud_rate has retries)
}

void M10QAsyncReceiver::state_machine() {
	switch (m_state) {
	case State::poweron: STATE_CALL(poweron); break;
	case State::poweroff: STATE_CALL(poweroff); break;
	case State::configure: STATE_CALL(configure); break;
	case State::senddatabase: STATE_CALL(senddatabase); break;
	case State::sendofflinedatabase: STATE_CALL(sendofflinedatabase); break;
	case State::startreceive: STATE_CALL(startreceive); break;
	case State::receive: STATE_CALL(receive); break;
	case State::stopreceive: STATE_CALL(stopreceive); break;
	case State::fetchdatabase: STATE_CALL(fetchdatabase); break;
	case State::enterbackup: STATE_CALL(enterbackup); break;
	case State::backupidle: STATE_CALL(backupidle); break;
	default:
	case State::idle: break;
	}
}

void M10QAsyncReceiver::run_state_machine(unsigned int time_ms) {
	system_scheduler->cancel_task(m_state_machine_handle);
	m_state_machine_handle =
	    system_scheduler->post_task_prio([this]() { state_machine(); }, "M10Q", Scheduler::DEFAULT_PRIORITY, time_ms);
}

void M10QAsyncReceiver::react(const UBXCommsEventSendComplete &) {
	if (m_op_state == OpState::PENDING) {
		cancel_timeout();
		m_op_state = OpState::SUCCESS;
		run_state_machine();
	}
}

void M10QAsyncReceiver::react(const UBXCommsEventAckNack &ack) {
	if (m_op_state == OpState::PENDING) {
		cancel_timeout();
		m_op_state = ack.ack ? OpState::SUCCESS : OpState::NACK;
		run_state_machine();
	}
}

void M10QAsyncReceiver::react(const UBXCommsEventCfgValget &valget) {
	unsigned int length = valget.length;

	// 2026-08 (A1) — CETTE FONCTION S'EXECUTE EN CONTEXTE ISR (handler libuarte).
	// Elle allouait auparavant un std::vector dimensionne par la longueur du
	// message recu, c'est-a-dire un malloc newlib NON REENTRANT depuis une
	// interruption, puis emettait une dizaine de DEBUG_TRACE (qui peuvent
	// descendre jusqu'a l'ecriture LFS / USB) avant meme de traiter l'etat.
	// Le decodage cle par cle n'etait que du diagnostic: il est desormais borne
	// a un tampon de pile et n'a lieu que si les traces GNSS sont demandees.
	uint8_t stack_buf[64];
	const unsigned int copy_len = (length < sizeof(stack_buf)) ? length : (unsigned int)sizeof(stack_buf);
	uint8_t *msg = stack_buf;
	std::memcpy(msg, valget.msg, copy_len);
	length = copy_len;

	// Le decodage ci-dessous est purement diagnostique — on le saute quand les
	// traces GNSS ne sont pas demandees, pour ne rien faire de couteux en ISR.
	if (!m_nav_settings.debug_enable || length < 4) {
		if (m_op_state == OpState::PENDING) {
			m_op_state = OpState::SUCCESS;
			cancel_timeout();
			run_state_machine();
		}
		return;
	}

	// Parse the fixed header fields
	[[maybe_unused]] uint8_t version = msg[0];
	[[maybe_unused]] uint8_t layer = msg[1];
	[[maybe_unused]] uint16_t position =
	    static_cast<uint16_t>(msg[2]) | (static_cast<uint16_t>(msg[3]) << 8);  // Combine bytes for position

	DEBUG_TRACE("Version: %u", version);
	DEBUG_TRACE("Layer: %u", layer);
	DEBUG_TRACE("Position: %u", position);

	// Decode the cfgData entries
	const unsigned int cfgDataStart = 4;  // cfgData starts after the 4-byte header
	unsigned int offset = cfgDataStart;   // Track the current position in cfgData

	while (offset < length) {
		// Each cfgData entry starts with a 4-byte key
		uint32_t key = static_cast<uint32_t>(msg[offset]) | (static_cast<uint32_t>(msg[offset + 1]) << 8)
		               | (static_cast<uint32_t>(msg[offset + 2]) << 16)
		               | (static_cast<uint32_t>(msg[offset + 3]) << 24);
		offset += 4;

		// Get the size of the parameter value
		uint8_t paramSize = UBX::CFG::getParameterSize(key);

		// Check for unknown keys or if remaining length is insufficient
		if (paramSize == 0 || offset + paramSize > length) {
			DEBUG_TRACE("Unknown or invalid key size for key: 0x%08X", key);
			break;
		}

		// Extract and cast the parameter value based on its size
		int64_t paramValue = 0;  // Use int64_t to handle larger int values for readability

		if (paramSize == 1) {
			paramValue = static_cast<int8_t>(msg[offset]);
		} else if (paramSize == 2) {
			paramValue = static_cast<int16_t>(msg[offset] | (msg[offset + 1] << 8));
		} else if (paramSize == 4) {
			paramValue = static_cast<int32_t>(msg[offset] | (msg[offset + 1] << 8) | (msg[offset + 2] << 16)
			                                  | (msg[offset + 3] << 24));
		} else {
			// For larger sizes or if it doesn't fit typical int sizes, treat as raw data
			paramValue = 0;
			for (uint8_t i = 0; i < paramSize; ++i) {
				paramValue |= static_cast<int64_t>(msg[offset + i]) << (8 * i);
			}
		}

		offset += paramSize;

		// Print the decoded configuration key-value pair
		DEBUG_TRACE("Config Key: 0x%08X | Value: %lld (0x%X)", key, paramValue, paramValue);
	}

	// Handle state
	if (m_op_state == OpState::PENDING) {
		m_op_state = OpState::SUCCESS;
		cancel_timeout();
		run_state_machine();
	}
}

void M10QAsyncReceiver::react(const UBXCommsEventSatReport &s) {
	if (m_nav_settings.sat_tracking) {
		// Note (QA review R10): m_pending_sat is the single ISR→task shuttle buffer.
		// If two SAT reports arrive in ISR before the scheduled task drains the
		// buffer, the earlier one is silently overwritten. At 1 Hz NAV-SAT this
		// is rare; under heavy scheduler load it may drop a report (counter
		// still increments correctly).
		{
			InterruptLock lock;
			std::memcpy(&m_pending_sat, &s, sizeof(s));
		}
		system_scheduler->post_task_prio(
		    [this]() {
			    // Copy under InterruptLock to prevent ISR overwriting mid-read
			    UBXCommsEventSatReport sat;
			    {
				    InterruptLock lock;
				    std::memcpy(&sat, &m_pending_sat, sizeof(sat));
			    }

			    if (m_nav_settings.debug_enable) {
				    DEBUG_TRACE("UBXCommsEventSatReport: numSvs=%u", (unsigned int)sat.sat.numSvs);
			    }
			    m_num_sat_samples++;
			    GPSEventSatReport e(sat.sat.numSvs, 0);
			    for (unsigned int i = 0; i < sat.sat.numSvs; i++) {
				    if (m_nav_settings.debug_enable) {
					    DEBUG_TRACE("UBXCommsEventSatReport: svInfo[%u].svId=%u", i,
						            (unsigned int)sat.sat.svInfo[i].svId);
					    DEBUG_TRACE("UBXCommsEventSatReport: svInfo[%u].qualityInd=%u", i,
						            (unsigned int)sat.sat.svInfo[i].qualityInd);
					    DEBUG_TRACE("UBXCommsEventSatReport: svInfo[%u].cno=%u", i,
						            (unsigned int)sat.sat.svInfo[i].cno);
				    }

				    if (sat.sat.svInfo[i].qualityInd > e.bestSignalQuality) {
					    e.bestSignalQuality = sat.sat.svInfo[i].qualityInd;
				    }
			    }
			    notify(e);

			    // Early abort: if after EARLY_ABORT_SAT_REPORTS sat reports no satellite
			    // is even detected (quality==0), the antenna is completely obstructed
			    // or underwater. quality >= 1 means satellites are visible — let the
			    // regular acquisition timeout handle it. Uses >= (not ==) to avoid
			    // missing the threshold if the counter ever skips a value.
			    if (m_num_sat_samples >= EARLY_ABORT_SAT_REPORTS && e.bestSignalQuality == 0) {
				    DEBUG_WARN("M10QAsyncReceiver: no satellite detected after %u sat reports | aborting",
					           m_num_sat_samples);
				    notify<GPSEventMaxSatSamples>({});
				    return;
			    }

			    // Check if max number of samples has been reached
			    if (m_nav_settings.max_sat_samples && m_num_sat_samples >= m_nav_settings.max_sat_samples) {
				    m_nav_settings.max_sat_samples = 0;
				    notify<GPSEventMaxSatSamples>({});
			    }
		    },
		    "SatReport");
	}
}

void M10QAsyncReceiver::react(const UBXCommsEventNavReport &n) {
	{
		InterruptLock lock;
		std::memcpy(&m_pending_nav, &n, sizeof(n));
	}
	system_scheduler->post_task_prio(
	    [this]() {
		    // Copy under InterruptLock to prevent ISR overwriting mid-read
		    UBXCommsEventNavReport nav;
		    {
			    InterruptLock lock;
			    std::memcpy(&nav, &m_pending_nav, sizeof(nav));
		    }

		    // 2026-05 deep-idle robustness: NAV report received → M10Q is alive
		    // and configured. If we got here via the EXTINT wake fast-path, that
		    // path is proven working for this session; reset the consecutive-
		    // wake-failure counter.
		    if (m_consecutive_wake_failures > 0) {
			    DEBUG_TRACE("M10QAsyncReceiver: NAV report received — resetting wake-fail counter");
			    m_consecutive_wake_failures = 0;
		    }

		    while (STATE_EQUAL(receive)) {
			    // Increment number of nav samples received
			    m_num_nav_samples++;

			    // Restart timeout — watchdog between NAV reports while in receive state
			    initiate_timeout(NAV_REPORT_WATCHDOG_MS);

			    if (nav.pvt.fixType != UBX::NAV::PVT::FIXTYPE_NO && nav.pvt.valid & UBX::NAV::PVT::VALID_VALID_DATE
			        && nav.pvt.valid & UBX::NAV::PVT::VALID_VALID_TIME) {
				    // Mitigations K + L (2026-05): detect first transition from
				    // virtual RTC (cold-boot fallback initialised to 1) to real
				    // UTC. Without this hook, time-based noinit state computed
				    // against the old virtual frame produces nonsense after the
				    // jump (e.g. HAULED could falsely engage with elapsed = 50
				    // years). Threshold 2000-01-01: anything below is virtual or
				    // unset, anything above is real GNSS-derived.
				    constexpr std::time_t RTC_MIN_REAL = 946684800;  // 2000-01-01
				    std::time_t prev = rtc ? rtc->gettime() : 0;
				    std::time_t now = convert_epochtime(nav.pvt.year, nav.pvt.month, nav.pvt.day, nav.pvt.hour,
					                                    nav.pvt.min, nav.pvt.sec);
				    rtc->settime(now);
				    // Mesure de la derive du quartz: `prev` (croyance de la RTC) et
				    // `now` (verite satellite) sont deja tous les deux sous la main.
				    // Aucune sonde a ajouter, une soustraction sur un chemin deja
				    // emprunte a chaque fix.
				    rtc->note_gnss_sync(prev, now);

				    // Persister l'heure juste SYNCHRONISEE, une seule fois par
				    // session. Sans cela, LAST_KNOWN_RTC n'etait rafraichi qu'au
				    // flush periodique: un reset tombant dans cet intervalle
				    // restaurait une heure ANTERIEURE a la correction GPS. Mesure au
				    // banc le 2026-08-25 — l'horloge est repartie 52 jours en
				    // arriere et l'assistance sauvegardee trois minutes plus tot a
				    // ete jetee par la garde anti-recul, pour un cold start complet.
				    if (!m_rtc_persisted_this_session && now >= RTC_MIN_REAL && configuration_store) {
					    m_rtc_persisted_this_session = true;
					    configuration_store->write_param(ParamID::LAST_KNOWN_RTC, static_cast<unsigned int>(now));
					    DEBUG_INFO("M10QAsyncReceiver: heure GNSS persistee (%u), derive mesuree %d ppm",
						           (unsigned int)now, (int)rtc->drift_ppm());
				    }

				    if (prev < RTC_MIN_REAL && now >= RTC_MIN_REAL) {
					    DEBUG_INFO(
					        "RTC: virtual→real sync detected (prev=%u → now=%u), re-anchoring noinit timekeepers",
					        (unsigned int)prev, (unsigned int)now);
					    HauledModeService::reset_for_rtc_sync(now);
					    RateLimiter::reset_for_rtc_sync();
				    }
			    }

			    if ((nav.pvt.fixType != UBX::NAV::PVT::FIXTYPE_2D && nav.pvt.fixType != UBX::NAV::PVT::FIXTYPE_3D)) {
				    break;
			    }

			    if (m_nav_settings.hacc_filter_en && (m_nav_settings.hacc_filter_threshold * 1000) < nav.pvt.hAcc) {
				    // Fix exists but fails hAcc filter — store as degraded if best so far
				    if (!m_has_degraded_pvt || nav.pvt.hAcc < m_degraded_pvt.hAcc) {
					    m_degraded_pvt = { .iTOW = nav.pvt.iTow,
						                   .year = nav.pvt.year,
						                   .month = nav.pvt.month,
						                   .day = nav.pvt.day,
						                   .hour = nav.pvt.hour,
						                   .min = nav.pvt.min,
						                   .sec = nav.pvt.sec,
						                   .valid = nav.pvt.valid,
						                   .tAcc = nav.pvt.tAcc,
						                   .nano = nav.pvt.nano,
						                   .fixType = nav.pvt.fixType,
						                   .flags = nav.pvt.flags,
						                   .flags2 = nav.pvt.flags2,
						                   .flags3 = nav.pvt.flags3,
						                   .numSV = nav.pvt.numSV,
						                   .lon = nav.pvt.lon / 10000000.0,
						                   .lat = nav.pvt.lat / 10000000.0,
						                   .height = nav.pvt.height,
						                   .hMSL = nav.pvt.hMSL,
						                   .hAcc = nav.pvt.hAcc,
						                   .vAcc = nav.pvt.vAcc,
						                   .velN = nav.pvt.velN,
						                   .velE = nav.pvt.velE,
						                   .velD = nav.pvt.velD,
						                   .gSpeed = nav.pvt.gSpeed,
						                   .headMot = nav.pvt.headMot / 100000.0f,
						                   .sAcc = nav.pvt.sAcc,
						                   .headAcc = nav.pvt.headAcc / 100000.0f,
						                   .pDOP = nav.dop.pDOP / 100.0f,
						                   .vDOP = nav.dop.vDOP / 100.0f,
						                   .hDOP = nav.dop.hDOP / 100.0f,
						                   .headVeh = nav.pvt.headVeh / 100000.0f,
						                   .ttff = nav.status.ttff };
					    m_has_degraded_pvt = true;
					    DEBUG_TRACE("M10QAsyncReceiver: degraded PVT stored (hAcc=%u mm)", nav.pvt.hAcc);
				    }
				    m_num_consecutive_fixes = m_nav_settings.num_consecutive_fixes;
				    break;
			    }

			    if (m_nav_settings.hdop_filter_en && (100 * m_nav_settings.hdop_filter_threshold) < nav.dop.hDOP) {
				    // Fix exists but fails hDOP filter — store as degraded if best so far
				    if (!m_has_degraded_pvt || nav.pvt.hAcc < m_degraded_pvt.hAcc) {
					    m_degraded_pvt = { .iTOW = nav.pvt.iTow,
						                   .year = nav.pvt.year,
						                   .month = nav.pvt.month,
						                   .day = nav.pvt.day,
						                   .hour = nav.pvt.hour,
						                   .min = nav.pvt.min,
						                   .sec = nav.pvt.sec,
						                   .valid = nav.pvt.valid,
						                   .tAcc = nav.pvt.tAcc,
						                   .nano = nav.pvt.nano,
						                   .fixType = nav.pvt.fixType,
						                   .flags = nav.pvt.flags,
						                   .flags2 = nav.pvt.flags2,
						                   .flags3 = nav.pvt.flags3,
						                   .numSV = nav.pvt.numSV,
						                   .lon = nav.pvt.lon / 10000000.0,
						                   .lat = nav.pvt.lat / 10000000.0,
						                   .height = nav.pvt.height,
						                   .hMSL = nav.pvt.hMSL,
						                   .hAcc = nav.pvt.hAcc,
						                   .vAcc = nav.pvt.vAcc,
						                   .velN = nav.pvt.velN,
						                   .velE = nav.pvt.velE,
						                   .velD = nav.pvt.velD,
						                   .gSpeed = nav.pvt.gSpeed,
						                   .headMot = nav.pvt.headMot / 100000.0f,
						                   .sAcc = nav.pvt.sAcc,
						                   .headAcc = nav.pvt.headAcc / 100000.0f,
						                   .pDOP = nav.dop.pDOP / 100.0f,
						                   .vDOP = nav.dop.vDOP / 100.0f,
						                   .hDOP = nav.dop.hDOP / 100.0f,
						                   .headVeh = nav.pvt.headVeh / 100000.0f,
						                   .ttff = nav.status.ttff };
					    m_has_degraded_pvt = true;
					    DEBUG_TRACE("M10QAsyncReceiver: degraded PVT stored (hDOP=%u)", nav.dop.hDOP);
				    }
				    m_num_consecutive_fixes = m_nav_settings.num_consecutive_fixes;
				    break;
			    }

			    if (m_num_consecutive_fixes) {
				    if (--m_num_consecutive_fixes) {
					    break;
				    }
			    }

			    GNSSData gnss_data = { .iTOW = nav.pvt.iTow,
				                       .year = nav.pvt.year,
				                       .month = nav.pvt.month,
				                       .day = nav.pvt.day,
				                       .hour = nav.pvt.hour,
				                       .min = nav.pvt.min,
				                       .sec = nav.pvt.sec,
				                       .valid = nav.pvt.valid,
				                       .tAcc = nav.pvt.tAcc,
				                       .nano = nav.pvt.nano,
				                       .fixType = nav.pvt.fixType,
				                       .flags = nav.pvt.flags,
				                       .flags2 = nav.pvt.flags2,
				                       .flags3 = nav.pvt.flags3,
				                       .numSV = nav.pvt.numSV,
				                       .lon = nav.pvt.lon / 10000000.0,
				                       .lat = nav.pvt.lat / 10000000.0,
				                       .height = nav.pvt.height,
				                       .hMSL = nav.pvt.hMSL,
				                       .hAcc = nav.pvt.hAcc,
				                       .vAcc = nav.pvt.vAcc,
				                       .velN = nav.pvt.velN,
				                       .velE = nav.pvt.velE,
				                       .velD = nav.pvt.velD,
				                       .gSpeed = nav.pvt.gSpeed,
				                       .headMot = nav.pvt.headMot / 100000.0f,
				                       .sAcc = nav.pvt.sAcc,
				                       .headAcc = nav.pvt.headAcc / 100000.0f,
				                       .pDOP = nav.dop.pDOP / 100.0f,
				                       .vDOP = nav.dop.vDOP / 100.0f,
				                       .hDOP = nav.dop.hDOP / 100.0f,
				                       .headVeh = nav.pvt.headVeh / 100000.0f,
				                       .ttff = nav.status.ttff };
			    m_fix_was_found = true;
			    notify(GPSEventPVT(gnss_data));
			    return;
		    }

		    // Check if max number of samples has been reached
		    if (STATE_EQUAL(receive) && m_nav_settings.max_nav_samples
		        && m_num_nav_samples >= m_nav_settings.max_nav_samples) {
			    m_nav_settings.max_nav_samples = 0;
			    // CloudLocate takes priority over degraded PVT
			    if (m_nav_settings.cloudlocate_enable && m_has_raw_measurement) {
				    GNSSRawMeasurement raw_copy;
				    {
					    InterruptLock lock;
					    raw_copy = m_raw_measurement;
				    }
				    DEBUG_INFO("M10QAsyncReceiver: timeout with raw measurement (CloudLocate)");
				    notify(GPSEventRawMeasurement(raw_copy));
			    } else if (m_has_degraded_pvt) {
				    // If we have a degraded fix (passed 2D/3D but failed quality filters), emit it
				    DEBUG_INFO("M10QAsyncReceiver: timeout with degraded PVT (hAcc=%u mm, fixType=%u, numSV=%u)",
					           m_degraded_pvt.hAcc, m_degraded_pvt.fixType, m_degraded_pvt.numSV);
				    notify(GPSEventPVTDegraded(m_degraded_pvt));
			    } else {
				    // Diagnostic for the "0 CloudLocate frames" symptom: session
				    // reached its sample budget with CloudLocate enabled but the
				    // receiver never produced a usable raw-measurement snapshot.
				    // Per u-blox docs this happens when fewer than the minimum
				    // satellites (~5) at sufficient C/N0 (~35 dB/Hz) were tracked
				    // in the surface window — the compact MEAS message stays empty.
				    if (m_nav_settings.cloudlocate_enable) {
					    DEBUG_WARN("M10QAsyncReceiver: timeout, CloudLocate ENABLED but NO raw measurement captured "
						           "(insufficient SVs/C-N0 in window) — 0 frames");
					    VAL_GNSS("cloudlocate_no_capture timeout num_nav=%u", m_num_nav_samples);
				    }
				    notify<GPSEventMaxNavSamples>({});
			    }
		    }
	    },
	    "NavReport");
}

void M10QAsyncReceiver::react(const UBXCommsEventRawMeasurement &meas) {
	// Called from ISR context via UBX comms filter — protect shared state
	bool emit_cloudlocate_ready = false;
	{
		InterruptLock lock;
		if (meas.has_measc12) {
			std::memcpy(m_raw_measurement.measc12, meas.measc12, sizeof(m_raw_measurement.measc12));
			m_raw_measurement.has_measc12 = true;
		}
		if (meas.has_meas20) {
			std::memcpy(m_raw_measurement.meas20, meas.meas20, sizeof(m_raw_measurement.meas20));
			m_raw_measurement.has_meas20 = true;
		}
		if (meas.has_meas50) {
			std::memcpy(m_raw_measurement.meas50, meas.meas50, sizeof(m_raw_measurement.meas50));
			m_raw_measurement.has_meas50 = true;
		}
		m_has_raw_measurement =
		    m_raw_measurement.has_measc12 || m_raw_measurement.has_meas20 || m_raw_measurement.has_meas50;
		// Stamp the capture instant (RTC epoch) so the CloudLocate packet can embed
		// the measurement time — the cloud then knows WHEN it was taken regardless
		// of any cache/TX delay. Updated on each snapshot so it reflects whichever
		// measurement is ultimately transmitted.
		if (m_has_raw_measurement && rtc && rtc->is_set()) m_raw_measurement.capture_time = (uint32_t)rtc->gettime();

		// One-shot: notify subscribers the FIRST time raw becomes available during
		// an active acquisition, so they can fire an early CloudLocate TX without
		// waiting for the cold-acq timeout. Flag set under lock; emit deferred
		// to scheduler context. The snapshot is read from m_raw_measurement at
		// task execution time (NOT captured in the lambda — keeps capture small
		// enough for stdext::inplace_function's 12-byte budget).
		if (m_has_raw_measurement && !m_cloudlocate_ready_notified && m_nav_settings.cloudlocate_enable) {
			m_cloudlocate_ready_notified = true;
			emit_cloudlocate_ready = true;
		}
	}
	// Defer the notify out of ISR context — emit from the scheduler. Subscribers
	// (e.g. LoRaTxService via GPSService) react in main loop, not interrupt.
	if (emit_cloudlocate_ready) {
		system_scheduler->post_task_prio(
		    [this]() {
			    // Re-validate raw availability — if a power_off() reset
			    // m_raw_measurement / m_has_raw_measurement between the ISR
			    // setting the flag and this scheduler task running, suppress the
			    // emit. Without this, subscribers would receive an empty
			    // GPSEventCloudLocateReady and the downstream CloudLocate TX
			    // path would fall through to status (no harm, but spurious log).
			    GNSSRawMeasurement raw_copy;
			    bool still_valid;
			    {
				    InterruptLock lock;
				    still_valid = m_has_raw_measurement;
				    if (still_valid) raw_copy = m_raw_measurement;
			    }
			    if (!still_valid) {
				    DEBUG_TRACE("M10QAsyncReceiver: CloudLocate-ready task cancelled (raw reset between ISR and task)");
				    return;
			    }
			    DEBUG_INFO("M10QAsyncReceiver: first raw measurement available — emitting GPSEventCloudLocateReady "
				           "(measc12=%u meas20=%u meas50=%u)",
				           (unsigned)raw_copy.has_measc12, (unsigned)raw_copy.has_meas20,
				           (unsigned)raw_copy.has_meas50);
			    VAL_GNSS("cloudlocate_ready measc12=%u meas20=%u meas50=%u", (unsigned)raw_copy.has_measc12,
				         (unsigned)raw_copy.has_meas20, (unsigned)raw_copy.has_meas50);
			    notify(GPSEventCloudLocateReady(raw_copy));
		    },
		    "M10QCloudLocateReady");
	}
}

void M10QAsyncReceiver::react(const UBXCommsEventMgaAck &ack) {
	if (STATE_EQUAL(fetchdatabase)) {
		m_expected_dbd_messages = ack.num_dbd_messages;
	}
}

void M10QAsyncReceiver::react(const UBXCommsEventMgaDBD &dbd) {
	if (STATE_EQUAL(fetchdatabase)) {
		cancel_timeout();
		if ((m_ana_database_len + dbd.length) < sizeof(m_navigation_database)) {
			std::memcpy(&m_navigation_database[m_ana_database_len], dbd.database, dbd.length);
			m_ana_database_len += dbd.length;
		} else
			m_database_overflow = true;
		initiate_timeout();
	} else if (STATE_EQUAL(senddatabase) || STATE_EQUAL(sendofflinedatabase)) {
		if ((m_mga_ack_count + dbd.length) <= sizeof(m_navigation_database)) {
			std::memcpy(&m_navigation_database[m_mga_ack_count], dbd.database, dbd.length);
			m_mga_ack_count += dbd.length;
		}
	}
}

void M10QAsyncReceiver::react(const UBXCommsEventDebug &e) {
	// Capture header + length by value to avoid dangling pointer to DMA buffer.
	// The full hex dump is not possible here due to inplace_function size constraints.
	uint32_t hdr = 0;
	unsigned int len = e.length;
	if (len >= 4) std::memcpy(&hdr, e.buffer, 4);
	system_scheduler->post_task_prio([hdr, len]() { DEBUG_TRACE("UBXComms<-GNSS: len=%u hdr=%08X", len, hdr); },
	                                 "Debug");
}

void M10QAsyncReceiver::react(const UBXCommsEventError &e) {
	// Framing errors (0x04) during the boot window (poweron/configure) come from
	// the NMEA->UBX baud transition. Async event delivery can land them in
	// poweron OR configure depending on scheduler timing — tolerate both.
	// Real soft bugs (alloc/overrun/buffer) keep their immediate ERROR path.
	bool is_framing = (e.error_type == 0x04);
	bool in_boot_window = STATE_EQUAL(poweron) || STATE_EQUAL(configure);

	if (is_framing && in_boot_window) {
		system_scheduler->post_task_prio(
		    [this, e]() {
			    DEBUG_WARN("UBXCommsEventError: type=%02x count=%u (boot transition, expected)", e.error_type,
				           m_uart_error_count);
		    },
		    "Debug");
		if (++m_uart_error_count >= MAX_FRAMING_ERRORS_BOOT) {
			m_uart_error_count = 0;
			cancel_timeout();
			m_op_state = OpState::ERROR;
			run_state_machine();
		} else {
			// On tolere l'erreur — mais handle_error() a ARRETE le RX. Sans cette
			// relance l'UART reste sourd: plus aucune erreur ne peut arriver (donc
			// le seuil ci-dessus est inatteignable) et la reponse attendue ne
			// remonte jamais. Depuis un contexte ISR: on differe au scheduler.
			system_scheduler->post_task_prio([this]() { m_ubx_comms.restart_rx(); }, "UBXRxRestart");
		}
	} else {
		system_scheduler->post_task_prio(
		    [this, e]() { DEBUG_ERROR("UBXCommsEventError: type=%02x count=%u", e.error_type, m_uart_error_count); },
		    "Debug");
		cancel_timeout();
		m_op_state = OpState::ERROR;
		run_state_machine();
	}
}

void M10QAsyncReceiver::initiate_timeout(unsigned int timeout_ms) {
	cancel_timeout();
	m_timeout.handle = system_scheduler->post_task_prio([this]() { on_timeout(); }, "Timeout",
	                                                    Scheduler::DEFAULT_PRIORITY, timeout_ms);
}

void M10QAsyncReceiver::on_timeout() {
	if (m_op_state == OpState::PENDING) {
		m_op_state = OpState::TIMEOUT;
		m_ubx_comms.cancel_expect();
		state_machine();
	} else if (STATE_EQUAL(receive)) {
		// No NAV/SAT information received within requested receive timeout
		DEBUG_ERROR("M10QAsyncReceiver::on_timeout: no NAV/SAT info received");
		m_unrecoverable_error = true;
		notify<GPSEventError>({});
		// Force power off to prevent GPS staying powered indefinitely
		check_for_power_off();
	}
}

void M10QAsyncReceiver::cancel_timeout() {
	system_scheduler->cancel_task(m_timeout.handle);
}

void M10QAsyncReceiver::state_poweron_enter() {
	m_step = 0;
	m_retries = BOOT_BAUD_SYNC_RETRIES;
	m_op_state = OpState::IDLE;
	m_uart_error_count = 0;
	m_fix_was_found = false;
	m_unrecoverable_error = false;


	DEBUG_INFO("M10QAsyncReceiver::state_poweron_enter");
	exit_shutdown();
	notify<GPSEventPowerOn>({});
}

/// @brief Baud to probe at boot-sync step `step`, starting from the cached hint.
/// Wraps so every entry of BOOT_BAUD_TABLE is tried exactly once per power-on.
unsigned int M10QAsyncReceiver::boot_baud_for_step(unsigned int step) const {
	return BOOT_BAUD_TABLE[(m_boot_baud_idx + step) % BOOT_BAUD_COUNT];
}

/// @brief Budget d'essais pour le debit sonde a l'etape `step`.
/// Plein sur le premier (celui mis en cache, statistiquement le bon), reduit
/// ensuite: sans cela, couvrir quatre debits quadruplerait le temps d'echec.
unsigned int M10QAsyncReceiver::boot_baud_retries(unsigned int step) {
	return (step == 0) ? BOOT_BAUD_SYNC_RETRIES : BOOT_BAUD_ALT_RETRIES;
}

void M10QAsyncReceiver::state_poweron() {
	while (true) {
		if (m_op_state == OpState::IDLE) {
			m_op_state = OpState::PENDING;
			if (m_step < BOOT_BAUD_COUNT) {
				// 2026-08 fix: probe EVERY baud the M10Q can boot at, not just
				// 9600. With BBR retained by the backup cell on the GNSS rail
				// the receiver comes back at MAX_BAUDRATE and is silent at
				// 9600 — probing 9600 alone killed GNSS permanently, because
				// state_configure (the only place that renegotiates the port
				// or wipes BBR) is only reachable through this sync.
				DEBUG_TRACE("M10QAsyncReceiver: boot sync attempt %u/%u at %u baud", m_step + 1, BOOT_BAUD_COUNT,
				            boot_baud_for_step(m_step));
				sync_baud_rate(boot_baud_for_step(m_step));
				break;
			} else {
				DEBUG_ERROR("M10QAsyncReceiver: failed to sync comms (tried %u bauds)", BOOT_BAUD_COUNT);
				m_unrecoverable_error = true;
				notify<GPSEventError>({});
				// Couper le rail nous-memes: on ne peut pas dependre d'un abonne
				// (GPSService ignore l'evenement hors session, et PWRON GNSS
				// n'abonne personne) sinon la FSM se fige en poweron rail ON.
				// Idempotent: si react() a deja lance la coupure, m_powering_off
				// est vrai et l'appel sort immediatement.
				check_for_power_off();
				break;
			}
		} else if (m_op_state == OpState::SUCCESS) {
			// Remember which baud answered: on a board whose BBR survives the
			// off window every later boot lands on the same one, so the next
			// session skips the failed probes (3 s each).
			unsigned int synced = boot_baud_for_step(m_step);
			m_synced_baud = synced;
			// PREUVE DE RETENTION BBR, a l'execution: la config du port vit dans
			// la couche BBR. Repondre a MAX_BAUDRATE apres une coupure de rail
			// signifie donc que la BBR a survecu (pile presente et chargee).
			// Repondre a 9600 = defauts usine = BBR perdue. C'est ce fait, et
			// non un flag de build, qui pilote le fast-path de state_configure.
			m_bbr_retained = (synced == MAX_BAUDRATE);
			if (!m_bbr_retained) {
				// Defauts usine: le recepteur n'a plus aucun de nos reglages, la
				// configuration complete va tout reecrire. On oublie ce qu'on
				// croyait applique pour que le prochain fast-path ne se fie pas
				// a une valeur qui n'existe plus cote recepteur.
				m_applied_constellation_mask = CONSTELLATION_MASK_UNKNOWN;
			}
			DEBUG_INFO("M10QAsyncReceiver: boot sync at %u baud — BBR %s", synced,
			           m_bbr_retained ? "RETAINED" : "lost");
			if (m_step != 0) {
				m_boot_baud_idx = (m_boot_baud_idx + m_step) % BOOT_BAUD_COUNT;
			}
			VAL_GNSS("boot_sync_ok baud=%u step=%u bbr=%u", synced, m_step, (unsigned)m_bbr_retained);
			STATE_CHANGE(poweron, configure);
			break;
		} else if (m_op_state == OpState::PENDING) {
			break;
		} else if (m_op_state == OpState::ERROR) {
			// Framing errors mean bytes ARE arriving, just at the wrong rate —
			// move straight on to the next candidate baud. Sur le DERNIER baud de
			// la table en revanche, avancer sans borne terminait le power-on en
			// "failed to sync comms" avec les 6 essais encore inutilises: une
			// rafale de framing (frontiere NMEA->UBX, parasite) suffisait a tuer
			// la session. On consomme alors le budget comme la branche timeout.
			DEBUG_TRACE("M10QAsyncReceiver: baud rate framing error detected");
			m_uart_error_count = 0;  // nouveau baud (ou nouvel essai) = budget neuf
			if (m_step + 1 < BOOT_BAUD_COUNT) {
				m_step++;
				m_retries = boot_baud_retries(m_step);
			} else if (--m_retries == 0) {
				m_step++;
				m_retries = boot_baud_retries(m_step);
			}
			m_op_state = OpState::IDLE;
			run_state_machine(100);
			break;
		} else {
			if (--m_retries == 0) {
				// Re-arm the retry budget for the next baud in the list.
				// Without this the counter underflows to UINT_MAX on the next
				// timeout and the probe loop never terminates.
				m_step++;
				m_retries = boot_baud_retries(m_step);
			}
			m_op_state = OpState::IDLE;
		}
	}
}

void M10QAsyncReceiver::state_poweron_exit() {}

void M10QAsyncReceiver::state_poweroff_enter() {
	m_step = 0;
	m_retries = DEFAULT_RETRIES;
	m_op_state = OpState::IDLE;
	m_uart_error_count = 0;
}

void M10QAsyncReceiver::state_poweroff() {
	if (!m_powering_off) {
		STATE_CHANGE(poweroff, configure);
		return;
	}

	// CRITICAL #1 audit fix: if deep-idle was armed via
	// request_deep_idle_on_next_stop() before the power-down chain started,
	// route to `enterbackup` instead of cutting the rail. The M10Q stays
	// powered with PMREQ-backup engaged so V_BCKP can charge naturally and
	// next session uses the EXTINT wake fast-path (warm-start TTFF).
	if (m_deep_idle_pending && m_unrecoverable_error) {
		// Le deep-idle suppose un recepteur qui repond: y aller apres une erreur
		// irrecuperable, c'est enchainer 12 sondes de 500 ms puis retomber ici
		// de toute facon, rail allume pendant tout ce temps.
		DEBUG_WARN("M10QAsyncReceiver: deep-idle annule (erreur irrecuperable) — coupure du rail");
		VAL_GNSS("deep_idle_cancelled reason=unrecoverable_error");
		m_deep_idle_pending = false;
	}
	if (m_deep_idle_pending) {
		m_deep_idle_pending = false;
		m_powering_off = false;  // not a true power-off anymore
		// HIGH GNSS-AUDIT #1 follow-up: signal WARM entry path to
		// state_enterbackup_enter(). Cannot detect via PWR_EN GPIO read on this
		// BSP — see m_enterbackup_warm doc in the header for the rationale.
		m_enterbackup_warm = true;
		VAL_GNSS("deep_idle_intent_consumed -> enterbackup (rail stays on, warm=1)");
		// Restore num_power_on to 1 because backup-charge state machine
		// needs the user count consistency check during the subsequent
		// `exit_backup_charge_mode()` (called by next power_on path).
		// However enter_backup_charge_mode's invariant is users==0; the
		// PMREQ-backup chain itself doesn't require users>0. Keep users=0
		// here — when power_on() is called later, it will run the wake
		// fast-path which sets m_num_power_on = 1 explicitly.
		STATE_CHANGE(poweroff, enterbackup);
		return;
	}

	enter_shutdown();
	// Persist current RTC for next boot (enables MGA-ANO assistance and faster TTFF)
	if (rtc->is_set()) {
		configuration_store->write_param(ParamID::LAST_KNOWN_RTC, static_cast<unsigned int>(rtc->gettime()));
		// RAM updated — flash deferred to periodic flush / powerdown
		DEBUG_TRACE("Updated LAST_KNOWN_RTC = %u (RAM)", static_cast<unsigned int>(rtc->gettime()));
	}
	notify(GPSEventPowerOff(m_fix_was_found));
	// HIGH GNSS-AUDIT #3 follow-up: rail just cut by enter_shutdown(). Without
	// V_BCKP the M10Q will cold-boot at 9600 next session. Tracker reset so
	// any subsequent EXTINT wake (shouldn't happen from idle, defensive) uses
	// the correct baud.
	m_pmreq_baud = DEFAULT_BAUDRATE;
	STATE_CHANGE(poweroff, idle);
}

void M10QAsyncReceiver::state_poweroff_exit() {}

void M10QAsyncReceiver::send_pmreq_backup() {
	DEBUG_TRACE("M10QAsyncReceiver::send_pmreq_backup: UBX-RXM-PMREQ backup ->");
	RXM::MSG_PMREQ pmreq = {
		.version = 0,
		.reserved1 = { 0, 0, 0 },
		.duration = 0,                     // sleep until wakeup event
		.flags = RXM::PMREQFlags::BACKUP,  // 0x02
		// 2026-05 deep-idle refactor: wake source is EXTINT0 (deepest sleep
		// mode — ~10-12 µA vs ~15 µA with UARTRX since the UART block can
		// be fully powered down). The nRF drives the EXTINT pin via
		// pulse_extint_wake() at the start of power_on() when state ==
		// backupidle.
		.wakeupSources = RXM::PMREQWakeupSources::EXTINT0,  // 0x20
	};
	m_ubx_comms.send_packet(MessageClass::MSG_CLASS_RXM, RXM::ID_PMREQ, pmreq);
	m_ubx_comms.wait_send();
}

// 2026-05 deep-idle refactor: pulse the M10Q EXTINT pin to wake it from
// PMREQ-backup. Pre-conditions:
//  - BSP::GPIO_GPS_EXT_INT is configured as OUTPUT (done in bsp.cpp 2026-05).
//  - M10Q is in PMREQ-backup state (state == backupidle).
//  - Rail (GPIO_GPS_PWR_EN) is still ON (caller's responsibility).
// The M10Q datasheet requires a pulse width >= ~100 µs to register. We
// drive HIGH for 1 ms (10× minimum spec margin) then back to LOW, then
// release to high-Z so the pin doesn't sink/source current while the
// M10Q is awake and using EXTINT as a normal logic input.
void M10QAsyncReceiver::pulse_extint_wake() {
	DEBUG_TRACE("M10QAsyncReceiver::pulse_extint_wake");
	VAL_GNSS("extint_pulse_wake from_state=%d", (int)m_state);
	GPIOPins::init_pin(BSP::GPIO::GPIO_GPS_EXT_INT);          // ensure OUTPUT direction
	GPIOPins::set(BSP::GPIO::GPIO_GPS_EXT_INT);               // edge rising → M10Q wakes
	PMU::delay_ms(1);                                         // hold (>= 100 µs spec)
	GPIOPins::clear(BSP::GPIO::GPIO_GPS_EXT_INT);             // back to LOW idle
	GPIOPins::release_to_highz(BSP::GPIO::GPIO_GPS_EXT_INT);  // no leakage
}

void M10QAsyncReceiver::state_enterbackup_enter() {
	m_step = 0;
	m_retries = BOOT_BAUD_SYNC_RETRIES;
	m_op_state = OpState::IDLE;
	m_uart_error_count = 0;
	// 2026-05-25 PMREQ verification: reset the per-dive retry budget so the
	// verification step can retry PMREQ up to PMREQ_VERIFY_RETRIES times if
	// the M10Q refuses backup on first attempt.
	m_pmreq_verify_retries = PMREQ_VERIFY_RETRIES;
	DEBUG_INFO("M10QAsyncReceiver::state_enterbackup_enter");

	// HIGH GNSS-AUDIT #1 fix: two entry paths reach enterbackup:
	//   (a) WARM — from `state_poweroff()` rerouting via `m_deep_idle_pending`
	//       (CRITICAL #1 fix). Rail still ON, UART still up, M10Q running.
	//       We just want to send the PMREQ-backup and let it sleep.
	//   (b) COLD — legacy DTE-triggered `enter_backup_charge_mode()` from
	//       idle state. Rail OFF, UART torn down, M10Q dead.
	//
	// The old code did (b) unconditionally — when reached via the new (a)
	// path, this was killing the rail + waiting 200 ms + re-energising +
	// re-booting M10Q only to immediately put it back to sleep. That wasted
	// ~400 ms of active current per end-of-session AND discharged V_BCKP
	// (defeating half the deep-idle benefit).
	//
	// Detect entry path via m_enterbackup_warm (set by state_poweroff() reroute).
	// Reading PWR_EN GPIO does NOT work here — the pin is configured with
	// NRF_GPIO_PIN_INPUT_DISCONNECT in the BSP, so `nrf_gpio_pin_read()` always
	// returns 0 regardless of the output drive state. Without the flag, every
	// deep-idle entry took the COLD path → unnecessary M10Q reset via
	// exit_shutdown() → 9600 baud sync failure → bail to poweroff → PMREQ never
	// sent → V_BCKP never charged. Field log 2026-05-24 caught this in 100%
	// of deep-idle dispatches.
	bool rail_warm = m_enterbackup_warm;
	m_enterbackup_warm = false;  // single-shot
	VAL_GNSS("enterbackup_enter rail_warm=%d", rail_warm ? 1 : 0);

	if (rail_warm) {
		// WARM path: M10Q running with UART up at MAX_BAUDRATE from the
		// previous receive session. Skip the cold-boot work; the
		// state_enterbackup() steps will sync at the cached rate
		// (m_gnss_info_valid stays true since we never lost VDD), send
		// PMREQ-backup, and the M10Q sleeps. On next EXTINT wake it comes
		// back at the same rate (preserved by V_BCKP).
		return;
	}

	// COLD path (legacy DTE-triggered backup-charge) — original behavior.
	//
	// 2026-05-23 fix: every entry to enterbackup comes from poweroff state
	// (idle → enterbackup, idle being post-poweroff). The M10Q is fresh-
	// booting and BBR is likely LOST if the V_BCKP coin cell was depleted
	// (the very reason we are running backup-charge). Force the boot baud
	// sync to start at 9600 (factory default after cold-boot).
	m_gnss_info_valid = false;

	// After a long poweroff (typical for DTE-triggered backup-charge), rail
	// caps are fully discharged. exit_shutdown's 100 ms delay is calibrated
	// for normal power-on where caps are warm from the previous session.
	// Ensure UART is dead during the power-up + boot window, then init + sync.
	m_ubx_comms.deinit();  // Tear down libuarte if it was up from a prior cycle
	exit_shutdown();       // Init UART, power rail, 100 ms settle
	PMU::delay_ms(200);    // Extra margin for cold cap recharge + M10Q boot
}

void M10QAsyncReceiver::state_enterbackup() {
	while (true) {
		if (m_op_state == OpState::IDLE) {
			DEBUG_TRACE("M10QAsyncReceiver:state_enterbackup: step:%u", m_step);
			m_op_state = OpState::PENDING;
			if (m_step == 0) {
				// After a fresh cold-boot the M10 talks at 9600, but if BBR is
				// alive (previous save_config persisted) it may have come up at
				// MAX_BAUDRATE. Mirror the configure fast-path: try the rate we
				// expect from cached info first.
				unsigned int baud = m_gnss_info_valid ? MAX_BAUDRATE : DEFAULT_BAUDRATE;
				sync_baud_rate(baud);
				break;
			} else if (m_step == 1) {
				// Fallback baud — try the other one if step 0 failed
				unsigned int baud = m_gnss_info_valid ? DEFAULT_BAUDRATE : MAX_BAUDRATE;
				sync_baud_rate(baud);
				break;
			} else if (m_step == 2) {
				// 2026-05-25 EXTINT fix: drive GPIO_GPS_EXT_INT LOW (OUTPUT)
				// BEFORE sending PMREQ-backup.
				//
				// Why: PMREQ-backup tells the M10Q to monitor EXTINT for wake
				// edges (wakeupSources=EXTINT0). After the previous wake,
				// `pulse_extint_wake` releases EXTINT to high-Z to avoid
				// leakage while the M10Q is awake — correct for the awake
				// state, but means EXTINT is floating when the next PMREQ-
				// backup fires. Floating EXTINT can pick up ambient noise or
				// settle at indeterminate levels, which the M10Q may interpret
				// as wake edges → either refuses to stay in backup or wakes
				// immediately after entering it.
				//
				// Driving LOW here gives the M10Q a clean stable idle state
				// to observe across the entire backup duration. The next
				// `pulse_extint_wake` already drives a clean LOW→HIGH→LOW
				// pulse and releases to high-Z, so this is compatible with
				// the existing wake path.
				GPIOPins::init_pin(BSP::GPIO::GPIO_GPS_EXT_INT);
				GPIOPins::clear(BSP::GPIO::GPIO_GPS_EXT_INT);

				send_pmreq_backup();
				m_op_state = OpState::IDLE;
				m_step++;
				// 2026-05-25 Fix #6: propagate delay extended 100→500 ms. Field
				// data showed M10Q routinely refusing PMREQ-backup on the first
				// probe (3 attempts in <1 s all failed in 33 % of cycles).
				// Successful cycles took 1-2 s of total settling, suggesting the
				// M10Q needs >500 ms of stability to fully commit to backup mode
				// before responding to UART. Giving the FIRST probe 500 ms head-
				// start should land most cycles on attempt #1. Cost: +400 ms on
				// the dive→backup path (non-critical, not surface→TX).
				run_state_machine(500);
				break;
			} else if (m_step == 3) {
				// 2026-05-25 PMREQ-backup verification: send an invalid CFG-MSG
				// and expect a NACK (same probe as `sync_baud_rate`). If the
				// M10Q is in PMREQ-backup, UART RX is gated off → TIMEOUT →
				// backup confirmed (good). If the M10Q is still awake (PMREQ
				// refused or never received), it replies → SUCCESS → re-send
				// PMREQ up to PMREQ_VERIFY_RETRIES.
				//
				// Why: catches the silent-failure mode observed 2026-05-25
				// where firmware happily logged "M10 sleeping" but ammeter
				// showed ~2 mA (PSM/Sleep) instead of ~10 µA (backup).
				// Wakeup-source restriction (EXTINT0 only, no UARTRX) means
				// this probe cannot accidentally wake an actually-sleeping
				// M10Q.
				{
					CFG::MSG::MSG_MSG_NORATE probe = {
						.msgClass = MessageClass::MSG_CLASS_BAD,
						.msgID = 0,
					};
					initiate_timeout(PMREQ_VERIFY_TIMEOUT_MS);
					m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_MSG, probe,
					                                    MessageClass::MSG_CLASS_ACK, ACK::ID_NACK);
				}
				break;
			} else {
				// Step 4 (formerly step 3): release UART pins (rail stays powered)
				m_ubx_comms.deinit();
				STATE_CHANGE(enterbackup, backupidle);
				break;
			}
		} else if (m_op_state == OpState::SUCCESS) {
			// HIGH GNSS-AUDIT #3 follow-up: capture the baud that actually
			// worked. This is the baud the M10Q is now responding at — and
			// critically, the baud it will preserve across PMREQ-backup
			// (V_BCKP keeps BBR alive while the rail is on). Read at next
			// EXTINT wake to pre-sync our UART without assumption.
			//   - step 0 success: primary baud worked
			//     (MAX if m_gnss_info_valid else DEFAULT)
			//   - step 1 success: fallback baud worked
			//     (DEFAULT if m_gnss_info_valid else MAX)
			// Both branches must save BEFORE we increment m_step.
			if (m_step == 0) {
				m_pmreq_baud = m_gnss_info_valid ? MAX_BAUDRATE : DEFAULT_BAUDRATE;
				m_step = 2;  // skip fallback step
			} else if (m_step == 1) {
				m_pmreq_baud = m_gnss_info_valid ? DEFAULT_BAUDRATE : MAX_BAUDRATE;
				m_step++;
			} else if (m_step == 3) {
				// 2026-05-25 PMREQ verification: SUCCESS = M10Q replied to
				// our probe = NOT in PMREQ-backup. Re-send PMREQ from step 2
				// up to PMREQ_VERIFY_RETRIES times before giving up.
				if (m_pmreq_verify_retries > 0) {
					m_pmreq_verify_retries--;
					DEBUG_WARN(
					    "M10QAsyncReceiver: M10Q replied to probe after PMREQ — backup refused, retrying (left=%u)",
					    (unsigned)m_pmreq_verify_retries);
					VAL_GNSS("pmreq_verify_retry left=%u", (unsigned)m_pmreq_verify_retries);
					m_step = 2;  // re-send PMREQ
				} else {
					// 2026-05-25 Fix #8: rail-cycle fallback. If the M10Q
					// persistently refuses PMREQ-backup after all retries,
					// continuing to backupidle leaves the M10Q awake for the
					// entire GNP52 window — measured ~2 mA × 600 s = 0.33 mAh
					// per failure cycle. Cutting the rail instead loses BBR /
					// V_BCKP (next session cold-boots at 9600 with no warm
					// ephemeris) but costs only ~0.035 mAh per cold-start —
					// roughly a 10× saving per failure. The trade is worth it
					// because PMREQ refusals are rare-but-not-vanishing
					// (~33 % of cycles in 2026-05-25 bench data) and a
					// year-long sealed deployment cannot tolerate the cumulative
					// 2-mA leak. STATE_CHANGE follows the same shape as the
					// baud-sync failure bail at the bottom of this function.
					DEBUG_ERROR("M10QAsyncReceiver: M10Q refusing PMREQ-backup after %u attempts — "
					            "cutting rail (true poweroff, BBR lost) instead of leaking ~2 mA for the GNP52 window",
					            (unsigned)PMREQ_VERIFY_RETRIES);
					VAL_GNSS("pmreq_verify_giveup_rail_cycle");
					m_powering_off = true;
					m_num_power_on = 0;
					STATE_CHANGE(enterbackup, poweroff);
					return;  // exit state_enterbackup entirely; STATE_CHANGE has redirected the FSM
				}
			} else {
				m_step++;  // step 2/4 advance — no baud change to record
			}
			VAL_GNSS("enterbackup_step_ok step=%u pmreq_baud=%u", m_step, m_pmreq_baud);
			m_retries = DEFAULT_RETRIES;
			m_op_state = OpState::IDLE;
		} else if (m_op_state == OpState::PENDING) {
			break;
		} else {
			// 2026-05-25 PMREQ verification: step 3 non-SUCCESS (TIMEOUT,
			// ERROR, unexpected NACK) = M10Q silent to probe = backup
			// confirmed. Skip the normal baud-sync retry path; advance to
			// step 4 (UART deinit + backupidle transition).
			if (m_step == 3) {
				DEBUG_INFO("M10QAsyncReceiver: PMREQ-backup verified (M10Q silent to probe)");
				VAL_GNSS("pmreq_verify_ok");
				m_step++;
				m_retries = DEFAULT_RETRIES;
				m_op_state = OpState::IDLE;
				continue;  // re-enter while loop with new step
			}
			if (--m_retries == 0) {
				if (m_step == 0) {
					// Primary baud failed all retries — try the fallback
					DEBUG_WARN("M10QAsyncReceiver: enterbackup baud sync at %s failed, trying fallback",
					           m_gnss_info_valid ? "MAX" : "9600");
					m_step = 1;
					m_retries = BOOT_BAUD_SYNC_RETRIES;
				} else {
					// 2026-05-23 (revised): bail to poweroff on sync fail.
					//
					// History:
					//   v1 (pre-2026-05-23): bail to poweroff. Cuts rail → no
					//       recharge. Bad if sync fails often.
					//   v2 (commit f49615e3): "graceful" fallback to backupidle
					//       with rail on but M10Q awake (no PMREQ sleep).
					//       Recharge happens BUT at ~30 mA × GNSS_BCKP_CHARGE_
					//       DURATION (300 s default) = 2.5 mAh per cycle.
					//       With GNSS_BCKP_CHARGE_INTERVAL=60 min → 24 cycles/day
					//       → ~60 mAh/day burned just to fight sync failures.
					//       Way worse than just skipping the recharge: a missed
					//       recharge cycle only costs ~1 mAh at the next surface
					//       (one cold-start GPS session). Awake recharge =
					//       counter-productive.
					//   v3 (this, commit 561ca588 + this): back to poweroff
					//       bail. The prerequisite fixes (9600 first via
					//       m_gnss_info_valid=false, clean UART teardown,
					//       +200 ms settling) should make sync succeed reliably.
					//       If it still fails despite that, accept the missed
					//       recharge as a ~1 mAh penalty — next interval retries.
					DEBUG_ERROR("M10QAsyncReceiver: state_enterbackup sync failed (step %u) — "
					            "bailing to poweroff (recharge skipped, will retry next cycle)",
					            m_step);
					m_powering_off = true;
					m_num_power_on = 0;
					STATE_CHANGE(enterbackup, poweroff);
					break;
				}
			}
			m_op_state = OpState::IDLE;
		}
	}
}

void M10QAsyncReceiver::state_enterbackup_exit() {}

void M10QAsyncReceiver::state_backupidle_enter() {
	// 2026-05-25 wake diagnostic: record entry timestamp so the next power_on
	// can report how long the M10Q actually got to sleep. Diagnoses the
	// "M10Q wakes after only 3 seconds in a 5-min GNP52 window" case observed
	// on the bench — see the wake log in `power_on()` below.
	m_backupidle_entered_ms = PMU::get_timestamp_ms();
	DEBUG_INFO("M10QAsyncReceiver::state_backupidle_enter: rail ON, M10 sleeping (deep-idle started, t0=%llu ms)",
	           (unsigned long long)m_backupidle_entered_ms);
	// Nothing to do — rail is powered, UART deinit'd, M10 in backup mode.
	// The state remains until exit_backup_charge_mode() is called.
}

void M10QAsyncReceiver::state_backupidle() {
	// Idle: GPSService is responsible for triggering exit_backup_charge_mode().
}

void M10QAsyncReceiver::state_backupidle_exit() {}

void M10QAsyncReceiver::state_configure_enter() {
	m_step = 0;
	m_retries = DEFAULT_RETRIES;
	m_op_state = OpState::IDLE;
	// Comme state_poweron_enter/state_poweroff_enter/state_enterbackup_enter:
	// le budget d'erreurs de framing tolerees appartient a l'etat, pas a la
	// session. Sans ce reset, le chemin chaud backupidle -> configure heritait
	// du compteur de la session precedente.
	m_uart_error_count = 0;
	m_bbr_config_cleared = false;
	// 2026-06 latch fix: clear any stale unrecoverable-error here too. The warm
	// EXTINT-wake fast-path enters configure directly (backupidle -> configure),
	// bypassing state_poweron_enter() which is the only other clear site — so a
	// flag latched in a previous session would otherwise persist across every
	// warm wake. A genuine configure failure re-sets it below (per-session), so
	// clearing it on entry just gives each session a fresh attempt.
	m_unrecoverable_error = false;
}

// m_step is a plain step counter shared by five state machines (configure,
// startreceive, stopreceive, enterbackup, fetchdatabase). Inside each of them
// 0..N is just "step N of this sequence" and needs no name.
//
// state_configure is the exception: it runs three sequences in one counter, and
// the two non-contiguous families are unreadable as bare numbers. They are named
// here. The 100+ and 200+ bases are deliberate -- `m_step >= FAST_PATH_BASE` is
// how the generic SUCCESS branch tells a fast-path step from a normal one.
namespace ConfigStep {
// Fast path, taken when the BBR survived and no cold start was requested.
static constexpr unsigned int FAST_PATH_BASE = 100;
static constexpr unsigned int FAST_VALIDATE_BAUD = 100;  ///< re-validate at the baud the M10Q actually answered on
static constexpr unsigned int FAST_BBR_VALIDATED = 101;  ///< BBR good, jump to the assistance data
static constexpr unsigned int FAST_REAPPLY_CONSTELLATIONS = 102;  ///< the one per-session setting the fast path skips
static constexpr unsigned int FAST_DONE = 103;                    ///< rejoin the normal sequence

// BBR configuration erase, on a cold start. Has its own steps because it waits
// for an ACK; SUCCESS lands on ERASE_ACKED, which returns to NORMAL_AFTER_ERASE.
static constexpr unsigned int ERASE_BBR = 200;
static constexpr unsigned int ERASE_ACKED = 201;
static constexpr unsigned int NORMAL_AFTER_ERASE = 5;
}  // namespace ConfigStep

void M10QAsyncReceiver::state_configure() {
	while (true) {
		if (m_op_state == OpState::IDLE) {
			DEBUG_TRACE("M10QAsyncReceiver:state_configure: step:%u", m_step);
			m_op_state = OpState::PENDING;
			// ── Fast path "BBR retenue" ────────────────────────────────────────
			// 2026-08 : la decision est prise a l'EXECUTION (m_bbr_retained,
			// etabli par le baud qui a repondu au boot), plus par le flag de build
			// GNSS_HAS_BACKUP_BATTERY. Motif : le meme firmware tourne sur des
			// cartes avec et sans pile de sauvegarde, et une pile peut etre
			// absente, vide ou morte — c'est un fait par session, pas un fait de
			// compilation. L'ancienne garde faisait sonder MAX_BAUDRATE a un
			// recepteur reparti a 9600, ce qui empoisonnait le baud de l'UART et
			// tuait une session sur deux sur une carte sans pile.
			//
			// Une demande de COLD START court-circuite le fast-path : le CFG-RST
			// navBbrMask=0xFFFF vit au step 5, donc sauter les steps 0-6 revenait
			// a annuler silencieusement la demande — precisement quand elle est
			// necessaire (BBR retenue, donc potentiellement perimee ou corrompue).
			if (m_bbr_retained && m_gnss_info_valid && !m_nav_settings.cold_start && m_step == 0) {
				DEBUG_INFO("M10QAsyncReceiver: BBR retenue (M10Q a %u baud) — fast path config", m_synced_baud);
				m_step = ConfigStep::FAST_VALIDATE_BAUD;  // Fast path: validate BBR
				m_op_state = OpState::IDLE;
				continue;
			}
			if (m_nav_settings.cold_start && m_step == 0) {
				DEBUG_INFO("M10QAsyncReceiver: COLD START demande — fast path desactive, config complete + CFG-RST");
				VAL_GNSS("fastpath_bypassed reason=cold_start");
			}
			// Fast path step 100: on revalide au baud auquel le M10Q a REELLEMENT
			// repondu (et non a une valeur en dur), de sorte qu'un echec ne laisse
			// jamais l'UART sur un debit que le recepteur n'ecoute pas.
			if (m_step == ConfigStep::FAST_VALIDATE_BAUD) {
				sync_baud_rate(m_synced_baud);
				break;
			}
			// Fast path step 101: BBR validated, skip to assistance data
			if (m_step == ConfigStep::FAST_BBR_VALIDATED) {
				// GNSS LOW #7 audit fix: BBR validation succeeded = the
				// M10Q is alive AND came up at MAX_BAUDRATE (warm wake).
				// This is the earliest reliable proof of wake-from-deep-idle
				// success. Reset the wake-fail counter here instead of waiting
				// for the first NAV report (which can take 5-30 s, allowing
				// the counter to incorrectly trip on a short dive cancel).
				if (m_consecutive_wake_failures > 0) {
					DEBUG_TRACE("M10QAsyncReceiver: BBR validated — resetting wake-fail counter (was %u)",
					            m_consecutive_wake_failures);
					m_consecutive_wake_failures = 0;
				}
				m_fastpath_attempts = 0;  // fast path validated → reset failure counter
				// 2026-08 : on reprend au step 7, pas au step 10. Les steps 7 et 8
				// (disable_odometer / disable_timepulse_output) ecrivent en couche
				// RAM uniquement : elle est TOUJOURS perdue a la mise sous tension,
				// meme quand la BBR survit. Les sauter laissait la sortie timepulse
				// (1 PPS, active par defaut) et l'odometre allumes a chaque session
				// sur BBR retenue. Les steps 0-6 (port UART, constellations,
				// save_config, soft_reset) restent sautes : ils sont persistes en
				// BBR ou sans objet ici — c'est la ou est le gain de temps (~2,5 s).
				// Le masque de constellations varie PAR SESSION (gps_service force
				// |0x0F tant qu'aucun fix n'a ete obtenu, puis revient au masque
				// configure). Il est ecrit au step 2, que le fast-path saute: sans
				// ce rattrapage le retrecissement post-fix ne s'appliquerait jamais
				// tant que la BBR survit, et un changement de GNSS_CONSTELLATION_MASK
				// par BLE serait ignore en silence. On ne repart PAS au step 2
				// (cela enchainerait save_config + soft_reset, ~1,5 s inutiles):
				// step 102 = le seul VALSET qui manque.
				if (m_nav_settings.constellation_mask != m_applied_constellation_mask) {
					DEBUG_INFO("M10QAsyncReceiver: fast path — masque constellations 0x%02x -> 0x%02x, reapplication",
					           m_applied_constellation_mask, m_nav_settings.constellation_mask);
					m_step = ConfigStep::FAST_REAPPLY_CONSTELLATIONS;
				} else {
					m_step = 7;
				}
				m_op_state = OpState::IDLE;
				continue;
			}
			// Etape 200: effacement de la couche de config BBR, avec ACK attendu.
			// SUCCESS amene a 201 (branche generique) qui revient au step 5.
			if (m_step == ConfigStep::ERASE_BBR) {
				clear_config();
				break;
			}
			if (m_step == ConfigStep::ERASE_ACKED) {
				m_bbr_config_cleared = true;
				DEBUG_INFO("M10QAsyncReceiver: effacement config BBR ACQUITTE par le recepteur");
				VAL_GNSS("clear_bbr_config_acked");
				m_step = ConfigStep::NORMAL_AFTER_ERASE;
				m_op_state = OpState::IDLE;
				continue;
			}
			// Fast path step 102: rattrapage du seul reglage saute qui varie par
			// session. SUCCESS amene a 103 (branche generique), traitee ci-dessous.
			if (m_step == ConfigStep::FAST_REAPPLY_CONSTELLATIONS) {
				setup_gnss_channel_sharing();
				break;
			}
			if (m_step == ConfigStep::FAST_DONE) {
				m_step = 7;
				m_op_state = OpState::IDLE;
				continue;
			}
			if (m_step == 0) {
				setup_uart_port();
				m_step++;
				m_op_state = OpState::IDLE;
				run_state_machine(1000);  // Wait for UART config to apply
				break;
			} else if (m_step == 1) {
				// A partir d'ici le M10Q parle a MAX_BAUDRATE: c'est le step 0 qui
				// vient de le reprogrammer. m_synced_baud est mis a jour dans la
				// branche SUCCESS ci-dessous, sans quoi il restait fige sur le baud
				// de boot (9600 sur carte sans pile) et le bridge u-center ouvert
				// dans le meme cycle parlait a 9600 a un recepteur passe a 460800.
				sync_baud_rate(MAX_BAUDRATE);
				break;
			} else if (m_step == 2) {
				setup_gnss_channel_sharing();
				break;
			} else if (m_step == 3) {
				// Wait 500 ms for configuration to be applied
				m_step++;
				m_op_state = OpState::IDLE;
				run_state_machine(500);
				break;
			} else if (m_step == 4) {
				//INFO : no need to save if using VALSET
				save_config();
				break;
			} else if (m_step == 5) {
				// Un COLD START efface les donnees de navigation ET la config
				// persistee: c'est la seule facon de sortir d'une couche BBR
				// verrouillee (debit exotique, cle corrompue). L'effacement passe
				// par l'etape 200 (dediee, avec attente d'ACK) et revient ici.
				if (m_nav_settings.cold_start && !m_bbr_config_cleared) {
					m_step = ConfigStep::ERASE_BBR;
					m_op_state = OpState::IDLE;
					continue;
				}
				m_step++;
				m_op_state = OpState::IDLE;
				soft_reset();  // This operation has no response
			} else if (m_step == 6) {
				m_step++;
				m_op_state = OpState::IDLE;
				run_state_machine(1000);
				break;
			} else if (m_step == 7) {
				disable_odometer();
				break;
			} else if (m_step == 8) {
				// only one timepulse to disable on M10Q
				disable_timepulse_output();
				break;
			} else if (m_step == 9) {
				// Skip - step 10 (setup_continuous_mode) overrides to FULL mode anyway
				m_step++;
				m_op_state = OpState::IDLE;
			} else if (m_step == 10) {
				setup_continuous_mode();
				break;
			} else if (m_step == 11) {
				setup_simple_navigation_settings();
				break;
			} else if (m_step == 12) {
				setup_expert_navigation_settings();
				break;
			} else if (m_step == 13) {
				if (rtc->is_set()) {
					supply_time_assistance();
					break;
				} else {
					m_op_state = OpState::IDLE;
					m_step++;
				}
			} else if (m_step == 14) {
				const auto &last_gps = configuration_store->get_last_gps_entry();
				if (last_gps.info.valid) {
					supply_position_assistance();
					break;
				} else {
					m_op_state = OpState::IDLE;
					m_step++;
				}
			} else if (m_step == 15) {
				query_mon_ver();
				break;
			} else if (m_step == 16) {
				query_sec_uniqid();
				break;
			} else {
				// Always try to send offline database as priority
				STATE_CHANGE(configure, sendofflinedatabase);
				break;
			}
		} else if (m_op_state == OpState::SUCCESS) {
			// Memoriser ce que le recepteur a reellement accepte: c'est ce que la
			// couche BBR contient desormais, et donc ce que le fast-path peut se
			// permettre de ne pas reecrire.
			if (m_step == 1 || m_step == ConfigStep::FAST_VALIDATE_BAUD)
				m_synced_baud = MAX_BAUDRATE;  // le port vient d'etre valide a MAX
			if (m_step == 2 || m_step == ConfigStep::FAST_REAPPLY_CONSTELLATIONS)
				m_applied_constellation_mask = m_nav_settings.constellation_mask;
			m_step++;
			m_retries = DEFAULT_RETRIES;
			m_op_state = OpState::IDLE;
		} else if (m_op_state == OpState::PENDING) {
			break;
		} else {
			// La validation du fast-path a echoue. On retente quelques fois (glitch
			// de sync possible) puis on conclut que la BBR n'a pas ete conservee et
			// on repart sur une configuration complete.
			//
			// 2026-08 CORRECTIF : on remet explicitement l'UART sur m_synced_baud
			// avant de repartir au step 0. Auparavant l'UART restait au debit de la
			// sonde (MAX en dur) : setup_uart_port() partait alors a 460800 vers un
			// M10Q qui ecoutait a 9600, le step 1 echouait a son tour et la session
			// entiere mourait sur "state_configure: failed" — une session sur deux
			// sur une carte sans pile. Le commentaire d'origine affirmait que le
			// baud retombait a 9600 : c'etait faux, cette selection n'existe que
			// dans state_enterbackup.
			if (m_step == ConfigStep::ERASE_BBR || m_step == ConfigStep::ERASE_ACKED) {
				// Pas d'ACK: soit le recepteur tourne un firmware ou CFG-CFG
				// n'existe plus (supprime du protocole 34.20 / SPG 5.20, remplace
				// par UBX-CFG-OTP), soit il a refuse la commande. On poursuit le
				// cold start — le CFG-RST, lui, reste valide et efface bien les
				// donnees de navigation — mais l'echappatoire de la couche de
				// CONFIGURATION n'est pas disponible sur ce module: si un debit
				// exotique s'y trouve persiste, seule la sonde multi-debits le
				// rattrapera. C'est une information de terrain, pas un detail.
				DEBUG_ERROR("M10QAsyncReceiver: effacement config BBR NON ACQUITTE — "
				            "CFG-CFG indisponible sur ce firmware recepteur (cf. SPG 5.20). "
				            "Le cold start continue, mais la couche de config BBR n'est PAS effacee");
				VAL_GNSS("clear_bbr_config_unsupported");
				m_bbr_config_cleared = true;  // ne pas boucler
				m_step = ConfigStep::NORMAL_AFTER_ERASE;
				m_retries = DEFAULT_RETRIES;
				m_op_state = OpState::IDLE;
				continue;
			}
			if (m_step >= ConfigStep::FAST_PATH_BASE) {
				constexpr unsigned int FASTPATH_MAX_ATTEMPTS = 3;
				if (++m_fastpath_attempts >= FASTPATH_MAX_ATTEMPTS) {
					DEBUG_WARN(
					    "M10QAsyncReceiver: fast path failed %u/%u — BBR non conservee, config complete a %u baud",
					    m_fastpath_attempts, FASTPATH_MAX_ATTEMPTS, m_synced_baud);
					// Seul m_bbr_retained casse la boucle de retry (c'est lui qui
					// garde le fast-path). m_gnss_info_valid n'est PAS efface : il
					// signifie "on connait le SW/HW/UID du recepteur" et sert au
					// cache de la commande DTE GNSSI ainsi qu'au choix du baud de
					// state_enterbackup — l'invalider n'avait aucun rapport.
					m_bbr_retained = false;
					m_fastpath_attempts = 0;
				} else {
					DEBUG_WARN("M10QAsyncReceiver: fast path failed (attempt %u/%u) — retry", m_fastpath_attempts,
					           FASTPATH_MAX_ATTEMPTS);
				}
				m_ubx_comms.set_baudrate(m_synced_baud);
				m_step = 0;
				m_retries = DEFAULT_RETRIES;
				m_op_state = OpState::IDLE;
				continue;
			}
			if (--m_retries == 0) {
				DEBUG_ERROR("M10QAsyncReceiver::state_configure: failed");
				m_unrecoverable_error = true;
				notify<GPSEventError>({});
				check_for_power_off();  // cf. state_poweron: ne pas dependre d'un abonne
				break;
			} else if (m_op_state == OpState::ERROR) {
				// Restart receiver on comms error
				initiate_timeout();
				m_ubx_comms.set_baudrate(MAX_BAUDRATE);
			}
			m_op_state = OpState::IDLE;
		}
	}
}

void M10QAsyncReceiver::state_configure_exit() {}

void M10QAsyncReceiver::state_startreceive_enter() {
	m_step = 0;
	m_retries = DEFAULT_RETRIES;
	m_op_state = OpState::IDLE;
}

void M10QAsyncReceiver::state_startreceive() {
	while (true) {
		if (m_op_state == OpState::IDLE) {
			m_op_state = OpState::PENDING;
			if (m_step == 0) {
				enable_nav_pvt_message();
				break;
			} else if (m_step == 1) {
				enable_nav_dop_message();
				break;
			} else if (m_step == 2) {
				enable_nav_status_message();
				break;
			} else if (m_step == 3) {
				if (m_nav_settings.sat_tracking) {
					enable_nav_sat_message();
					break;
				}
				m_op_state = OpState::IDLE;
				m_step++;
				continue;  // re-enter loop with correct m_step
			} else if (m_step == 4) {
				if (m_nav_settings.cloudlocate_enable) {
					enable_rxm_measc12_message();
					break;
				}
				m_op_state = OpState::IDLE;
				m_step++;
				continue;
			} else if (m_step == 5) {
				if (m_nav_settings.cloudlocate_enable) {
					enable_rxm_meas20_message();
					break;
				}
				m_op_state = OpState::IDLE;
				m_step++;
				continue;
			} else if (m_step == 6) {
				if (m_nav_settings.cloudlocate_enable) {
					enable_rxm_meas50_message();
					break;
				}
				m_op_state = OpState::IDLE;
				m_step++;
				continue;
			} else {
				STATE_CHANGE(startreceive, receive);
				break;
			}
		} else if (m_op_state == OpState::SUCCESS) {
			m_step++;
			m_retries = DEFAULT_RETRIES;
			m_op_state = OpState::IDLE;
		} else if (m_op_state == OpState::PENDING) {
			break;
		} else {
			if (m_retries == 0 || --m_retries == 0) {
				// CloudLocate format steps (4=MEASC12, 5=MEAS20, 6=MEAS50): a
				// single format-enable failing must NOT disable the whole
				// CloudLocate path. The three formats are independent; if e.g.
				// MEAS50 NACKs on a given M10Q FW, MEASC12/MEAS20 can still
				// produce snapshots. The previous code set cloudlocate_enable=
				// false on ANY step failure, so one NACK silently killed ALL
				// CloudLocate output (root cause of "0 CloudLocate frames" tags:
				// MEASC12 enabled fine but a later format NACK disabled
				// everything, and the emitted MEASC12 were then ignored).
				// Now: skip just the failing format and continue to the next.
				if (m_step >= 4 && m_step <= 6 && m_nav_settings.cloudlocate_enable) {
					DEBUG_WARN("M10QAsyncReceiver::state_start_receive: CloudLocate format step %u enable FAILED — "
					           "skipping this format, keeping others",
					           m_step);
					VAL_GNSS("cloudlocate_format_skip step=%u", m_step);
					m_step++;  // skip ONLY this format, keep CloudLocate enabled
					m_retries = DEFAULT_RETRIES;
					m_op_state = OpState::IDLE;
					continue;
				}
				DEBUG_ERROR("M10QAsyncReceiver::state_start_receive: failed at step %u", m_step);
				m_unrecoverable_error = true;
				notify<GPSEventError>({});
				check_for_power_off();  // cf. state_poweron: ne pas dependre d'un abonne
				break;
			} else if (m_op_state == OpState::ERROR) {
				// Restart receiver on comms error
				initiate_timeout();
				m_ubx_comms.set_baudrate(MAX_BAUDRATE);
			}
			m_op_state = OpState::IDLE;
		}
	}
}

void M10QAsyncReceiver::state_startreceive_exit() {}

void M10QAsyncReceiver::state_receive_enter() {
	// Adaptive initial timeout: use max_nav_samples as upper bound (1 sample ≈ 1 second),
	// plus RECEIVE_TIMEOUT_MARGIN_S for UART/processing overhead, with a floor for cold start.
	unsigned int timeout_ms =
	    std::max(RECEIVE_TIMEOUT_FLOOR_MS, (m_nav_settings.max_nav_samples + RECEIVE_TIMEOUT_MARGIN_S) * 1000U);
	initiate_timeout(timeout_ms);
	m_op_state = OpState::IDLE;
	m_retries = DEFAULT_RETRIES;
}

void M10QAsyncReceiver::state_receive() {
	if (m_op_state == OpState::ERROR) {
		if (m_retries > 0 && --m_retries) {
			// CommsError try to restart the receiver
			m_ubx_comms.set_baudrate(MAX_BAUDRATE);
			initiate_timeout(5000);
			m_op_state = OpState::IDLE;
		} else {
			DEBUG_ERROR("M10Receiver: repeated comms errors");
			m_unrecoverable_error = true;
			notify<GPSEventError>({});
			check_for_power_off();  // idem on_timeout(receive), deja fait ainsi
		}
	}
}

void M10QAsyncReceiver::state_receive_exit() {
	cancel_timeout();
}

void M10QAsyncReceiver::state_stopreceive_enter() {
	m_step = 0;
	m_retries = DEFAULT_RETRIES;
	m_op_state = OpState::IDLE;
}

void M10QAsyncReceiver::state_stopreceive() {
	if (!m_powering_off) {
		STATE_CHANGE(stopreceive, startreceive);
		return;
	}

	while (true) {
		if (m_op_state == OpState::IDLE) {
			m_op_state = OpState::PENDING;
			if (m_step == 0) {
				disable_nav_pvt_message();
				break;
			} else if (m_step == 1) {
				disable_nav_dop_message();
				break;
			} else if (m_step == 2) {
				disable_nav_status_message();
				break;
			} else if (m_step == 3) {
				if (m_nav_settings.sat_tracking) {
					disable_nav_sat_message();
					break;
				} else {
					m_op_state = OpState::IDLE;
					m_step++;
				}
			} else if (m_step == 4) {
				m_step++;
				m_op_state = OpState::IDLE;
				run_state_machine(100);  // Allow 100 ms to flush out any stray messages
				break;
			} else {
				STATE_CHANGE(stopreceive, fetchdatabase);
				break;
			}
		} else if (m_op_state == OpState::SUCCESS) {
			m_step++;
			m_retries = DEFAULT_RETRIES;
			m_op_state = OpState::IDLE;
		} else if (m_op_state == OpState::PENDING) {
			break;
		} else {
			if (--m_retries == 0) {
				// FSM recovers cleanly: transition to poweroff which honors
				// m_deep_idle_pending (reroutes to enterbackup → backupidle).
				// Next power_on starts fresh with fail_count=0. Sporadic NACKs
				// during NAV-message disable are expected when the M10Q is
				// shuffling its own internal state; only escalate to ERROR if
				// this turns into a chronic pattern (visible via repeated logs).
				DEBUG_WARN("M10QAsyncReceiver: stop receive failed (proceeding to poweroff)");
				STATE_CHANGE(stopreceive, poweroff);
				break;
			} else if (m_op_state == OpState::ERROR) {
				// Restart receiver on comms error
				initiate_timeout();
				m_ubx_comms.set_baudrate(MAX_BAUDRATE);
			}
			m_op_state = OpState::IDLE;
		}
	}
}

void M10QAsyncReceiver::state_stopreceive_exit() {}

void M10QAsyncReceiver::save_dbd_to_flash() {
	if (!main_filesystem || m_ana_database_len == 0) return;

	try {
		LFSFile file(main_filesystem, "gnss_dbd.dat", LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);
		// Write a 4-byte timestamp header followed by the database
		uint32_t save_time = rtc->is_set() ? (uint32_t)rtc->gettime() : 0;
		file.write(&save_time, sizeof(save_time));
		file.write(m_navigation_database, m_ana_database_len);
		DEBUG_TRACE("M10QAsyncReceiver::save_dbd_to_flash: saved %u bytes (t=%u)", m_ana_database_len, save_time);
	} catch (...) {
		DEBUG_WARN("M10QAsyncReceiver::save_dbd_to_flash: write failed");
	}
}

bool M10QAsyncReceiver::load_dbd_from_flash() {
	if (!main_filesystem) return false;

	try {
		LFSFile file(main_filesystem, "gnss_dbd.dat", LFS_O_RDONLY);
		if ((unsigned int)file.size() <= sizeof(uint32_t)) return false;

		// Read timestamp header and check freshness (max 12 hours)
		uint32_t save_time = 0;
		file.read(&save_time, sizeof(save_time));

		if (save_time > 0 && (!rtc || !rtc->is_set())) {
			// RTC not available — cannot verify freshness, reject to avoid stale data
			DEBUG_TRACE("M10QAsyncReceiver::load_dbd_from_flash: RTC not set — cannot verify freshness");
			return false;
		}
		if (rtc->is_set() && save_time > 0) {
			uint32_t now_t = (uint32_t)rtc->gettime();
			if (now_t < save_time) {
				// INFO et non TRACE: cette ligne signale la perte TOTALE de
				// l'assistance persistee. Elle se declenche des qu'un reset
				// restaure un LAST_KNOWN_RTC anterieur a la derniere synchro
				// GPS — constate au banc, l'horloge etait repartie 53 jours en
				// arriere et le DBD du jour a ete jete sans un mot.
				DEBUG_INFO(
				    "M10QAsyncReceiver::load_dbd_from_flash: RTC revenue en arriere (%u < %u) — assistance ecartee",
				    now_t, save_time);
				return false;
			}
			uint32_t age_s = now_t - save_time;
			const uint32_t max_age = m_bbr_retained ? DBD_MAX_AGE_BBR_S : DBD_MAX_AGE_NO_BBR_S;
			if (age_s > max_age) {
				DEBUG_INFO("M10QAsyncReceiver::load_dbd_from_flash: perime (%u s > %u s, BBR %s)", age_s, max_age,
				           m_bbr_retained ? "retenue" : "perdue");
				return false;
			}
			DEBUG_INFO("M10QAsyncReceiver::load_dbd_from_flash: %u octets, age %u s (plafond %u s, BBR %s)",
			           (unsigned int)(file.size() - sizeof(uint32_t)), age_s, max_age,
			           m_bbr_retained ? "retenue" : "perdue");
		}

		unsigned int data_len = (unsigned int)(file.size() - sizeof(uint32_t));
		if (data_len > sizeof(m_navigation_database)) {
			DEBUG_WARN("M10QAsyncReceiver::load_dbd_from_flash: file too large (%u)", data_len);
			return false;
		}

		lfs_ssize_t read = file.read(m_navigation_database, data_len);
		if (read != (lfs_ssize_t)data_len) {
			DEBUG_WARN("M10QAsyncReceiver::load_dbd_from_flash: short read");
			return false;
		}

		m_ana_database_len = data_len;
		DEBUG_TRACE("M10QAsyncReceiver::load_dbd_from_flash: loaded %u bytes (age=%u s)", data_len,
		            rtc->is_set() ? ((uint32_t)rtc->gettime() - save_time) : 0);
		return true;
	} catch (...) {
		return false;
	}
}

void M10QAsyncReceiver::state_fetchdatabase_enter() {
	m_step = 0;
	m_retries = DEFAULT_RETRIES;
	m_op_state = OpState::IDLE;
	m_ana_database_len = 0;
	m_expected_dbd_messages = 0;
	m_database_overflow = false;
	m_ubx_comms.start_dbd_filter();
}

void M10QAsyncReceiver::state_fetchdatabase() {
	if (!m_nav_settings.assistnow_autonomous_enable) {
		DEBUG_TRACE("M10QAsyncReceiver: fetchdatabase: ANA not enabled");
		STATE_CHANGE(fetchdatabase, poweroff);
		return;
	}
	// 2026-08 (A2) — on ne saute PLUS la recuperation quand l'ANO a servi cette
	// session. C'etait la seule raison pour laquelle gnss_dbd.dat n'etait jamais
	// rafraichi sur une unite ANO: passe le plafond d'age la copie flash
	// etait rejetee et l'unite perdait ses DEUX sources d'assistance d'un coup,
	// sans pouvoir les reconstruire. C'est le scenario "carte sans pile V_BCKP",
	// ou le DBD est le seul vecteur de demarrage chaud persistant.
	//
	// NOTE (correction de la passe precedente): on avait ajoute ici un garde
	// "pas de fix -> on ne recupere pas", au motif que save_dbd_to_flash() est
	// conditionne a m_fix_was_found. C'ETAIT FAUX: le resultat n'etait pas jete.
	// La recuperation remplit m_ana_database_len, et state_senddatabase_enter
	// rejoue ce tampon RAM a la session SUIVANTE sans relire le flash. Le garde
	// supprimait donc l'assistance precisement dans le regime qu'on cherche a
	// proteger — les series de sessions sans fix sur carte sans pile V_BCKP.
	// Seule la persistance flash reste conditionnee au fix (la fraicheur du
	// fichier ne doit pas etre blanchie par une session sterile).
	while (true) {
		if (m_op_state == OpState::IDLE) {
			m_op_state = OpState::PENDING;
			if (m_step == 0) {
				fetch_navigation_database();
				break;
			} else {
				// This will fetch any MGA-ACK from the navigation buffer so we can
				// validate if the correct number of messages are present
				m_ubx_comms.expect(MessageClass::MSG_CLASS_MGA, MGA::ID_ACK);
				m_ubx_comms.filter_buffer(m_navigation_database, m_ana_database_len);
				m_ubx_comms.cancel_expect();
				DEBUG_TRACE(
				    "M10QAsyncReceiver::state_fetchdatabase: validating database size %u bytes expected %u msgs",
				    m_ana_database_len, m_expected_dbd_messages);
				unsigned int actual_count = 0;
				if (!m_ubx_comms.is_expected_msg_count(m_navigation_database, m_ana_database_len,
				                                       m_expected_dbd_messages, actual_count,
				                                       MessageClass::MSG_CLASS_MGA, MGA::ID_DBD)) {
					if (--m_retries == 0) {
						DEBUG_WARN("M10QAsyncReceiver::state_fetch_database: failed: %u/%u msgs received", actual_count,
						           m_expected_dbd_messages);
						dump_navigation_database(m_ana_database_len);
						m_ana_database_len = 0;
						STATE_CHANGE(fetchdatabase, poweroff);
						break;
					} else {
						if (!m_database_overflow) {
							DEBUG_TRACE(
							    "M10Receiver::state_fetchdatabase: validation failed: %u/%u msgs received: retry",
							    actual_count, m_expected_dbd_messages);
							m_ana_database_len = 0;
							m_expected_dbd_messages = 0;
							m_op_state = OpState::IDLE;
							m_step = 0;
							continue;
						} else {
							DEBUG_TRACE("M10Receiver::state_fetchdatabase: validation skipped: %u/%u msgs received: "
							            "DBD buffer full",
							            actual_count, m_expected_dbd_messages);
						}
					}
				} else {
					DEBUG_TRACE("M10QAsyncReceiver::state_fetchdatabase: success");
					// Subtract the size of the MGA-ACK message from the database length
					// so it doesn't get sent out
					m_ana_database_len = m_ana_database_len - std::min(m_ana_database_len, msg_size<MGA::MSG_ACK>());
					// Persist to flash if a fix was obtained (useful data for next session)
					if (m_fix_was_found) {
						save_dbd_to_flash();
					}
				}
				STATE_CHANGE(fetchdatabase, poweroff);
				break;
			}
		} else if (m_op_state == OpState::ERROR) {
			DEBUG_TRACE("M10QAsyncReceiver::state_fetchdatabase: UART comms error");
			m_ana_database_len = 0;
			STATE_CHANGE(fetchdatabase, poweroff);
			break;
		} else if (m_op_state == OpState::SUCCESS) {
			m_step++;
			m_op_state = OpState::IDLE;
		} else if (m_op_state == OpState::TIMEOUT) {
			m_step++;
			m_op_state = OpState::IDLE;
		} else if (m_op_state == OpState::PENDING) {
			break;
		}
	}
}

void M10QAsyncReceiver::state_fetchdatabase_exit() {
	m_ubx_comms.stop_dbd_filter();
}

void M10QAsyncReceiver::state_senddatabase_enter() {
	m_step = 0;
	m_op_state = OpState::IDLE;
	m_mga_ack_count = 0;

	// If no ANA data in RAM and ANO is not in use, try loading persisted DBD from flash
	if (m_ana_database_len == 0 && m_ano_database_len == 0 && m_nav_settings.assistnow_autonomous_enable) {
		load_dbd_from_flash();
	}

	m_ubx_comms.start_dbd_filter();
	// INFO et non TRACE: c'est la SEULE trace qui dit si une assistance a
	// reellement ete injectee au recepteur. Sans elle, il faut deduire la
	// reponse du temps passe dans l'etat — ce qu'on a du faire au banc le
	// 2026-08-25 faute de mieux.
	DEBUG_INFO("M10QAsyncReceiver::state_senddatabase: injection de %u octets d'assistance", m_ana_database_len);
}

void M10QAsyncReceiver::state_senddatabase() {
	if (!m_nav_settings.assistnow_autonomous_enable) {
		DEBUG_TRACE("M10QAsyncReceiver: senddatabase: ANA not enabled");
		STATE_CHANGE(senddatabase, startreceive);
		return;
	}
	while (true) {
		if (m_op_state == OpState::IDLE) {
			m_op_state = OpState::PENDING;
			if (m_step < m_ana_database_len) {
				// Send buffer contents in chunks raw and notify when sent
				unsigned int sz = std::min(128U, (m_ana_database_len - m_step));
				m_ubx_comms.send(&m_navigation_database[m_step], sz, true, true);
				m_step += sz;
				break;
			} else {
				unsigned int actual_count = 0;
				[[maybe_unused]] bool success =
				    m_ubx_comms.is_expected_msg_count(m_navigation_database, m_mga_ack_count, m_expected_dbd_messages,
					                                  actual_count, MessageClass::MSG_CLASS_MGA, MGA::ID_ACK);
				DEBUG_TRACE("M10QAsyncReceiver::state_senddatabase: %s: %u/%u acks recieved",
				            success ? "success" : "missing MGA-ACK", actual_count, m_expected_dbd_messages);
				STATE_CHANGE(senddatabase, startreceive);
				break;
			}
		} else if (m_op_state == OpState::SUCCESS) {
			// Skip to next record
			m_op_state = OpState::IDLE;
			run_state_machine(5);  // Wait 5 ms delay between transmitted data
			break;
		} else if (m_op_state == OpState::PENDING) {
			break;
		} else {
			DEBUG_WARN("M10QAsyncReceiver::state_send_database: failed");
			if (m_op_state == OpState::ERROR) {
				// Restart receiver on comms error
				m_ubx_comms.set_baudrate(MAX_BAUDRATE);
			}
			STATE_CHANGE(senddatabase, startreceive);
			break;
		}
	}
}

void M10QAsyncReceiver::state_senddatabase_exit() {
	m_ubx_comms.stop_dbd_filter();
	// 2026-08 (A2) — react(UBXCommsEventMgaDBD) recopie les MGA-ACK dans
	// m_navigation_database a l'offset 0 pendant l'emission: le contenu ne vaut
	// plus rien apres coup. Sans cette remise a zero, une session qui n'atteint
	// pas fetchdatabase (le seul autre site de reset) laissait un tampon sali
	// avec une longueur non nulle, et toutes les sessions suivantes injectaient
	// ces octets sans jamais recharger la copie flash valide.
	m_ana_database_len = 0;
}

void M10QAsyncReceiver::state_sendofflinedatabase_enter() {
	m_op_state = OpState::IDLE;
	m_step = 0;
	m_mga_ack_count = 0;
	m_ano_database_len = 0;

	if (!m_nav_settings.assistnow_offline_enable) {
		DEBUG_TRACE("M10QAsyncReceiver: sendofflinedatabase: ANO not enabled");
		VAL_GNSS("ano_skip reason=disabled");
		return;
	}

	if (!rtc->is_set()) {
		DEBUG_TRACE("M10QAsyncReceiver: sendofflinedatabase: time not yet set");
		VAL_GNSS("ano_skip reason=no_rtc");  // ANO needs a valid date to pick the day's records
		return;
	}

	// 2026-08 (A2) — m_navigation_database a TROIS producteurs: le DBD recu, la
	// capture des MGA-ACK, et le tampon ANO ci-dessous. copy_mga_ano_to_buffer
	// ecrit dans ce tampon au fil de son balayage MEME quand elle finit par tout
	// rejeter (fichier perime -> 0 octet retenu). On sortait alors avec
	// m_ano_database_len == 0 mais un tampon dont la tete etait de l'ANO, et
	// m_ana_database_len valant encore la longueur du DBD de la session
	// precedente: state_senddatabase_enter voyait donc "j'ai deja un DBD en RAM",
	// ne rechargeait PAS la copie flash valide, et emettait vers le recepteur des
	// octets corrompus. Le DBD en RAM est mort des l'instant ou l'on touche au
	// tampon: on le declare tel quel ici.
	m_ana_database_len = 0;

	try {
		LFSFile file(main_filesystem, "gps_config.dat", LFS_O_RDONLY);
		unsigned int file_sz = (unsigned int)file.size();
		// 2026-06 latch fix: always re-scan the ANO file from the start for the
		// current day. copy_mga_ano_to_buffer used m_ano_start_pos as a
		// forward-only cursor that, once it hit a stale day (or a time jump),
		// latched to file.size() (EOF) and returned 0 records FOREVER — ANO
		// silently stopped applying and never recovered. The full re-scan is
		// bounded (one pass of the offline file) and runs only on cold sessions.
		m_ano_start_pos = 0;
		m_ubx_comms.copy_mga_ano_to_buffer(file, m_navigation_database, sizeof(m_navigation_database), rtc->gettime(),
		                                   m_ano_database_len, m_expected_dbd_messages, m_ano_start_pos,
		                                   m_nav_settings.ano_stale_threshold_s);
		m_ubx_comms.start_dbd_filter();
		// Field-visible ANO diagnostic: how many MGA-ANO msgs/bytes were actually
		// selected for TODAY from the offline file. 0 here = the 5-week file is
		// present but has no record matching the current date within the stale
		// window (wrong/old RTC, file doesn't cover today, or file truncated) →
		// the receiver gets NO offline aiding this session despite ANO "enabled".
		DEBUG_INFO(
		    "M10QAsyncReceiver::sendofflinedatabase: ANO file=%uB -> selected %u msgs / %u bytes for today (t=%u)",
		    file_sz, m_expected_dbd_messages, m_ano_database_len, (unsigned)rtc->gettime());
		VAL_GNSS("ano_sent file=%uB msgs=%u bytes=%u t=%u", file_sz, m_expected_dbd_messages, m_ano_database_len,
		         (unsigned)rtc->gettime());
	} catch (...) {
		DEBUG_ERROR("M10QAsyncReceiver::state_sendofflinedatabase: error opening MGA ANO file");
		VAL_GNSS("ano_skip reason=file_error");
		m_op_state = OpState::ERROR;
	}
}

void M10QAsyncReceiver::state_sendofflinedatabase() {
	if (!m_nav_settings.assistnow_offline_enable) {
		STATE_CHANGE(sendofflinedatabase, senddatabase);
		return;
	}

	if (m_ano_database_len == 0) {
		STATE_CHANGE(sendofflinedatabase, senddatabase);
		return;
	}

	// Adaptive MGA-ACK polling: a fixed 100 ms wait missed 1-2 late ACKs in
	// ~1-5 % of cold starts (log 94/95, 93/95). Poll every 50 ms and exit as
	// soon as the expected count is reached. Nominal case still completes in
	// one 50 ms tick; worst case stops at MGA_ACK_POLL_COUNT × MGA_ACK_POLL_MS.
	static constexpr unsigned int MGA_ACK_POLL_MS = 50;
	static constexpr unsigned int MGA_ACK_POLL_COUNT = 10;  // 500 ms cap

	while (true) {
		if (m_op_state == OpState::IDLE) {
			m_op_state = OpState::PENDING;
			if (m_step < m_ano_database_len) {
				// Send buffer contents in chunks raw and notify when sent
				unsigned int sz = std::min(512U, (m_ano_database_len - m_step));
				m_ubx_comms.send(&m_navigation_database[m_step], sz, true, true);
				m_step += sz;
				break;
			} else if (m_step == m_ano_database_len) {
				// All data sent — schedule async wait for MGA-ACK confirmations
				m_step++;  // Advance past m_ano_database_len to enter ACK-check state
				m_op_state = OpState::IDLE;
				run_state_machine(MGA_ACK_POLL_MS);  // First poll (adaptive ACK wait)
				break;
			} else {
				// ACK polling phase. m_step is reused as poll counter:
				// value (m_ano_database_len + N) means N polls have completed.
				unsigned int actual_count = 0;
				bool all_received =
				    m_ubx_comms.is_expected_msg_count(m_navigation_database, m_mga_ack_count, m_expected_dbd_messages,
					                                  actual_count, MessageClass::MSG_CLASS_MGA, MGA::ID_ACK);
				unsigned int poll_idx = m_step - m_ano_database_len;  // 1..MGA_ACK_POLL_COUNT
				if (all_received || poll_idx >= MGA_ACK_POLL_COUNT) {
					if (!all_received)
						DEBUG_WARN("M10QAsyncReceiver::state_sendofflinedatabase missing MGA-ACK: %u/%u acks received "
						           "(after %u ms)",
						           actual_count, m_expected_dbd_messages, poll_idx * MGA_ACK_POLL_MS);
					STATE_CHANGE(sendofflinedatabase, startreceive);
				} else {
					m_step++;
					m_op_state = OpState::IDLE;
					run_state_machine(MGA_ACK_POLL_MS);
				}
				break;
			}
		} else if (m_op_state == OpState::SUCCESS) {
			// Skip to next record
			m_retries = 1;
			m_op_state = OpState::IDLE;
			run_state_machine(1);  // Require 1 ms delay between transmitted data
			break;
		} else if (m_op_state == OpState::PENDING) {
			break;
		} else if (m_op_state == OpState::ERROR) {
			// handle_error() a arrete le RX. Dans startreceive, seuls les
			// op-states ERROR refont un set_baudrate (qui le relance): une
			// cascade de TIMEOUT y mene a l'echec de session sans jamais
			// ressusciter la reception. Meme classe que R1.
			m_ubx_comms.restart_rx();
			STATE_CHANGE(sendofflinedatabase, startreceive);
			break;
		}
	}
}

void M10QAsyncReceiver::state_sendofflinedatabase_exit() {
	m_ubx_comms.stop_dbd_filter();
}

void M10QAsyncReceiver::state_idle_enter() {
	m_nav_settings.max_nav_samples = 0;
	m_nav_settings.max_sat_samples = 0;
}

void M10QAsyncReceiver::state_idle() {}

void M10QAsyncReceiver::state_idle_exit() {}

void M10QAsyncReceiver::sync_baud_rate(unsigned int baud) {
	DEBUG_TRACE("M10QAsyncReceiver::sync_baud_rate: Syncing baud rate to %u", baud);

	m_ubx_comms.set_baudrate(baud);

	// Test configuration by sending a known invalid message and expecting a NACK
	CFG::MSG::MSG_MSG_NORATE cfg_msg_invalid = {
		.msgClass = MessageClass::MSG_CLASS_BAD,
		.msgID = 0,
	};

	initiate_timeout(500);
	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_MSG, cfg_msg_invalid,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_NACK);
}


void M10QAsyncReceiver::save_config() {
	DEBUG_TRACE("M10QAsyncReceiver::save_config: GPS CFG-CFG ->");
	CFG::CFG::MSG_CFG cfg_msg_cfg_cfg = {
		.clearMask = 0,
		.saveMask = 0xFFFFFFFF,
		// CFG::CFG::CLEARMASK_IOPORT   |
		// 			   CFG::CFG::CLEARMASK_MSGCONF  |
		// 			   CFG::CFG::CLEARMASK_INFMSG   |
		// 			   CFG::CFG::CLEARMASK_NAVCONF  |
		// 			   CFG::CFG::CLEARMASK_RXMCONF  |
		// 			   CFG::CFG::CLEARMASK_SENCONF  |
		// 			   CFG::CFG::CLEARMASK_RINVCONF |
		// 			   CFG::CFG::CLEARMASK_ANTCONF  |
		// 			   CFG::CFG::CLEARMASK_LOGCONF  |
		// 			   CFG::CFG::CLEARMASK_FTSCONF,
		.loadMask = 0,
		//.deviceMask  = CFG::CFG::DEVMASK_SPIFLASH, //BBR changed to SPIFLASH for not realy saving but go to next step.
		.deviceMask = CFG::CFG::DEVMASK_BBR,  //BBR changed to SPIFLASH for not realy saving but go to next step.
	};

	initiate_timeout();
	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_CFG, cfg_msg_cfg_cfg);
}

void M10QAsyncReceiver::clear_config() {
	DEBUG_INFO("M10QAsyncReceiver::clear_config: effacement de la config BBR du recepteur (CFG-CFG clear)");
	VAL_GNSS("clear_bbr_config");
	CFG::CFG::MSG_CFG cfg_msg_cfg_cfg = {
		.clearMask = 0xFFFFFFFF,
		.saveMask = 0,
		.loadMask = 0,  // pas de rechargement: la config RAM en cours reste active
		.deviceMask = CFG::CFG::DEVMASK_BBR,
	};
	// ACQUITTEMENT ATTENDU. La regle generale de la classe UBX-CFG (interface
	// description M10 SPG 5.10, §3.10) est "acquitte par ACK-ACK si traite,
	// ACK-NAK sinon", et CFG-CFG ne porte AUCUNE clause derogatoire — contrairement
	// a CFG-RST, dont la spec dit explicitement de ne pas attendre de reponse.
	// Emettre sans attendre revenait donc a jeter la seule preuve que
	// l'effacement a eu lieu.
	//
	// Enjeu concret: CFG-CFG a ete SUPPRIME du protocole 34.20 (SPG 5.20), ou il
	// est remplace par UBX-CFG-OTP. Sur un module approvisionne avec ce firmware,
	// l'effacement echouerait en silence et l'echappatoire BBR n'existerait plus.
	// L'absence d'ACK est donc l'information a remonter, pas un detail.
	initiate_timeout();
	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_CFG, cfg_msg_cfg_cfg);
}

void M10QAsyncReceiver::soft_reset() {
	// navBbrMask selects what survives the GNSS-only software reset:
	//   0x0000 = hot start  (keep ephemeris+almanac+pos+time+clock+aiding)
	//   0xFFFF = cold start (wipe everything in BBR)
	// A cold start is requested via GPSNavSettings.cold_start. It is issued
	// here, BEFORE supply_time_assistance (step 13), supply_position_assistance
	// (step 14) and the offline-database (ANO) send in the same configure
	// sequence — so the receiver is wiped clean and immediately re-seeded with
	// fresh time + position + ANO. RESETMODE_SOFTWARE_RESET_GNSS_ONLY keeps the
	// UART/port config, so no baud re-sync is needed.
	const uint16_t bbr_mask = m_nav_settings.cold_start ? 0xFFFF : 0x0000;
	DEBUG_TRACE("M10QAsyncReceiver::soft_reset: GPS CFG-RST-> navBbrMask=0x%04X (%s)", bbr_mask,
	            m_nav_settings.cold_start ? "COLD" : "hot");
	VAL_GNSS("soft_reset navBbrMask=0x%04X cold=%u", bbr_mask, (unsigned)m_nav_settings.cold_start);
	CFG::RST::MSG_RST cfg_msg_cfg_rst = {
		.navBbrMask = bbr_mask,
		.resetMode = CFG::RST::RESETMODE_SOFTWARE_RESET_GNSS_ONLY,
		//.resetMode = CFG::RST::RESETMODE_HARDWARE_RESET_IMMEDIATE,
		.reserved1 = 0,
	};
	m_ubx_comms.send_packet(MessageClass::MSG_CLASS_CFG, CFG::ID_RST, cfg_msg_cfg_rst);
	m_ubx_comms.wait_send();
	// 2026-08 : la demande de cold start est consommee ICI, une fois le CFG-RST
	// reellement emis. Sans cela le drapeau restait colle dans m_nav_settings et,
	// sur le chemin de reveil chaud (qui ne rafraichissait pas les reglages),
	// chaque session suivante re-effacait la BBR — l'exact inverse du but du
	// deep-idle. Le log ci-dessus est la trace terrain que le wipe a bien eu lieu.
	if (m_nav_settings.cold_start) {
		DEBUG_INFO("M10QAsyncReceiver: COLD START applique (BBR effacee) — demande consommee");
		m_nav_settings.cold_start = false;
	}
}

void M10QAsyncReceiver::setup_uart_port() {
	DEBUG_TRACE("M10QAsyncReceiver::setup_uart_port: Configuring UART1 with VALSET ->");

	// 8N1 at MAX_BAUDRATE (460800). The receiver keeps this in BBR, which is why
	// state_poweron has to probe every rate in BOOT_BAUD_TABLE rather than
	// assume the 9600 of a factory-fresh part.
	CFG::UART1::BAUDRATE.set_value(MAX_BAUDRATE);
	CFG::UART1::STOPBITS.set_value(CFG::UART1::StopBits::ONE);
	CFG::UART1::DATABITS.set_value(CFG::UART1::DataBits::EIGHT);
	CFG::UART1::PARITY.set_value(CFG::UART1::Parity::NONE);
	CFG::UART1::ENABLED.set_value(1);

	// UBX in and out, NMEA off in both directions: the parser only speaks UBX,
	// and NMEA sentences would just burn UART time and current.
	CFG::UART1::INPROT_UBX.set_value(1);
	CFG::UART1::OUTPROT_UBX.set_value(1);
	CFG::UART1::INPROT_NMEA.set_value(0);
	CFG::UART1::OUTPROT_NMEA.set_value(0);

	// TX-ready line unused: we read the UART continuously, so there is no pin to
	// assert and no threshold to reach. All five values are therefore 0.
	CFG::UART1::TXREADY_ENABLED.set_value(0);
	CFG::UART1::TXREADY_POLARITY.set_value(0);
	CFG::UART1::TXREADY_PIN.set_value(0);
	CFG::UART1::TXREADY_THRESHOLD.set_value(0);
	CFG::UART1::TXREADY_INTERFACE.set_value(0);

	// Collect all parameters in a vector
	std::vector<UBX::CFG::UBXParameter> uart1_config = {
		CFG::UART1::ENABLED,           CFG::UART1::BAUDRATE,         CFG::UART1::STOPBITS,
		CFG::UART1::DATABITS,          CFG::UART1::PARITY,           CFG::UART1::INPROT_UBX,
		CFG::UART1::OUTPROT_UBX,       CFG::UART1::INPROT_NMEA,      CFG::UART1::OUTPROT_NMEA,
		CFG::UART1::TXREADY_ENABLED,   CFG::UART1::TXREADY_POLARITY, CFG::UART1::TXREADY_PIN,
		CFG::UART1::TXREADY_THRESHOLD, CFG::UART1::TXREADY_INTERFACE
	};
	uint8_t layers = CFG::VALSET::LAYERS::BBR | CFG::VALSET::LAYERS::RAM;
	//DEBUG_TRACE("M10QAsyncReceiver::setup_uart_port: save %x", layers);
	//uint8_t layers = CFG::VALSET::LAYERS::RAM;
	// Create the VALSET message with dynamically sized parameters
	alignas(CFG::VALSET::MSG_VALSET)
	    uint8_t uart1_valset_msg_storage[sizeof(CFG::VALSET::MSG_VALSET) + CFG::VALSET::MSG_VALSET_MAX_CFG];
	auto &uart1_valset_msg = *new (uart1_valset_msg_storage) CFG::VALSET::MSG_VALSET(0x00, layers, uart1_config);
	size_t cfgDataSize = uart1_valset_msg.get_cfgData_size(uart1_config);
	initiate_timeout();

	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_VALSET, uart1_valset_msg,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_ACK, cfgDataSize);
	m_ubx_comms.wait_send();
}

// UBX configuration is written to a DELIBERATE storage layer per setting, and
// the choice is not cosmetic. As written today:
//   BBR|RAM  setup_power_management, setup_continuous_mode,
//            setup_simple_navigation_settings, setup_gnss_channel_sharing
//            -- survives a reset, and a rail cut when V_BCKP holds it.
//   RAM      everything else, including every NAV/RXM message rate and the
//            expert navigation settings -- re-applied on every session, so a
//            stale value can never outlive a firmware change.
// Persisting the wrong thing has already cost us a receiver: a baud rate held
// in BBR that the boot probe did not cover left the M10Q permanently mute (see
// BOOT_BAUD_TABLE at the top of this file). Each call below used to carry a
// commented-out alternative layer beside it; those have been removed. Changing
// a layer is a reviewed decision, not a line to uncomment.
void M10QAsyncReceiver::setup_power_management() {
	DEBUG_TRACE("M10QAsyncReceiver::setup_power_management: Configuring power management with VALSET ->");

	// Set power management parameters
	CFG::PM::OPERATEMODE.set_value(CFG::PM::OPERATEMODE_VALUES::PSMCT);  // Set to cyclic tracking mode
	CFG::PM::POSUPDATEPERIOD.set_value(1000);                            // Set position update period to 1000 seconds
	CFG::PM::ACQPERIOD.set_value(10000);  // Set acquisition period to 10000 seconds if failed
	CFG::PM::GRIDOFFSET.set_value(0);     // No offset for GPS week alignment
	CFG::PM::ONTIME.set_value(1);         // Time in Tracking state (1 second)
	CFG::PM::MINACQTIME.set_value(300);   // Minimum acquisition time (300 ms)
	CFG::PM::MAXACQTIME.set_value(0);     // Maximum acquisition time (no limit)
	CFG::PM::DONOTENTEROFF.set_value(1);  // Do not enter off state if fix fails
	CFG::PM::WAITTIMEFIX.set_value(0);    // No need to wait for time fix
	CFG::PM::UPDATEEPH.set_value(1);      // Regular ephemeris updates

	// Collect all parameters for VALSET
	std::vector<CFG::UBXParameter> pm_config = {
		CFG::PM::OPERATEMODE, CFG::PM::POSUPDATEPERIOD, CFG::PM::ACQPERIOD,  CFG::PM::GRIDOFFSET,
		CFG::PM::ONTIME,      CFG::PM::MINACQTIME,      CFG::PM::MAXACQTIME, CFG::PM::DONOTENTEROFF,
		CFG::PM::WAITTIMEFIX, CFG::PM::UPDATEEPH,
	};

	// Create the VALSET message with power management configuration
	alignas(CFG::VALSET::MSG_VALSET)
	    uint8_t pm_valset_msg_storage[sizeof(CFG::VALSET::MSG_VALSET) + CFG::VALSET::MSG_VALSET_MAX_CFG];
	auto &pm_valset_msg = *new (pm_valset_msg_storage) CFG::VALSET::MSG_VALSET(
	    0x00, CFG::VALSET::LAYERS::BBR | CFG::VALSET::LAYERS::RAM, pm_config);
	size_t cfgDataSize = pm_valset_msg.get_cfgData_size(pm_config);
	initiate_timeout();

	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_VALSET, pm_valset_msg,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_ACK, cfgDataSize);
}

void M10QAsyncReceiver::setup_continuous_mode() {
	DEBUG_TRACE("M10QAsyncReceiver::setup_continuous_mode: Configuring continuous mode with VALSET ->");

	// Set power management to FULL mode for continuous operation
	CFG::PM::OPERATEMODE.set_value(CFG::PM::FULL);  // Disable all power-saving modes

	// Collect parameters in a vector for the VALSET message
	std::vector<CFG::UBXParameter> pm_config = { CFG::PM::OPERATEMODE };

	// Create the VALSET message for power management configuration
	alignas(CFG::VALSET::MSG_VALSET)
	    uint8_t pm_valset_msg_storage[sizeof(CFG::VALSET::MSG_VALSET) + CFG::VALSET::MSG_VALSET_MAX_CFG];
	auto &pm_valset_msg = *new (pm_valset_msg_storage) CFG::VALSET::MSG_VALSET(
	    0x00, CFG::VALSET::LAYERS::BBR | CFG::VALSET::LAYERS::RAM, pm_config);
	size_t cfgDataSize = pm_valset_msg.get_cfgData_size(pm_config);
	initiate_timeout();

	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_VALSET, pm_valset_msg,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_ACK, cfgDataSize);
}

void M10QAsyncReceiver::setup_simple_navigation_settings() {
	DEBUG_TRACE("M10QAsyncReceiver::setup_simple_navigation_settings: Configuring NAVSPG with VALSET ->");

	// Set the navigation parameters for standard precision
	CFG::NAVSPG::FIXMODE.set_value(
	    static_cast<CFG::NAVSPG::FIXMODE_VALUES>(m_nav_settings.fix_mode));  // Set position fix mode dynamically
	CFG::NAVSPG::DYNMODEL.set_value(
	    static_cast<CFG::NAVSPG::DYNMODEL_VALUES>(m_nav_settings.dyn_model));  // Set dynamic platform model dynamically
	CFG::NAVSPG::UTCSTANDARD.set_value(CFG::NAVSPG::UTCSTANDARD_VALUES::UTC_AUTO);  // UTC standard auto
	CFG::NAVSPG::OUTFIL_PDOP.set_value(250);                                        // Position DOP threshold (25.0)
	CFG::NAVSPG::OUTFIL_TDOP.set_value(250);                                        // Time DOP threshold (25.0)
	CFG::NAVSPG::OUTFIL_PACC.set_value(100);                                        // Position accuracy mask
	CFG::NAVSPG::OUTFIL_TACC.set_value(350);                                        // Time accuracy mask
	CFG::NAVSPG::CONSTR_ALT.set_value(0);                                           // Fixed altitude for 2D fix mode
	CFG::NAVSPG::CONSTR_ALTVAR.set_value(10000);                                    // Fixed altitude variance
	CFG::NAVSPG::INFIL_MINELEV.set_value(m_nav_settings.min_elev);                  // Minimum elevation angle [deg]
	CFG::NAVSPG::CONSTR_DGNSSTO.set_value(60);                                      // DGNSS timeout

	// Collect all parameters in a vector for VALSET
	std::vector<CFG::UBXParameter> navspg_config = {
		CFG::NAVSPG::FIXMODE,       CFG::NAVSPG::DYNMODEL,      CFG::NAVSPG::UTCSTANDARD,    CFG::NAVSPG::OUTFIL_PDOP,
		CFG::NAVSPG::OUTFIL_TDOP,   CFG::NAVSPG::OUTFIL_PACC,   CFG::NAVSPG::OUTFIL_TACC,    CFG::NAVSPG::CONSTR_ALT,
		CFG::NAVSPG::CONSTR_ALTVAR, CFG::NAVSPG::INFIL_MINELEV, CFG::NAVSPG::CONSTR_DGNSSTO,
	};

	// Create the VALSET message with the navigation parameters
	alignas(CFG::VALSET::MSG_VALSET)
	    uint8_t nav_valset_msg_storage[sizeof(CFG::VALSET::MSG_VALSET) + CFG::VALSET::MSG_VALSET_MAX_CFG];
	auto &nav_valset_msg = *new (nav_valset_msg_storage) CFG::VALSET::MSG_VALSET(
	    0x00, CFG::VALSET::LAYERS::BBR | CFG::VALSET::LAYERS::RAM, navspg_config);
	size_t cfgDataSize = nav_valset_msg.get_cfgData_size(navspg_config);
	initiate_timeout();

	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_VALSET, nav_valset_msg,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_ACK, cfgDataSize);
}

void M10QAsyncReceiver::setup_expert_navigation_settings() {
	DEBUG_TRACE("M10QAsyncReceiver::setup_expert_navigation_settings: Configuring NAVSPG and ANA with VALSET ->");

	// NAVSPG Configuration (Standard Precision Navigation)
	CFG::NAVSPG::INFIL_MINSVS.set_value(3);                       // Minimum satellites for navigation
	CFG::NAVSPG::INFIL_MAXSVS.set_value(32);                      // Maximum satellites for navigation
	CFG::NAVSPG::INFIL_MINCNO.set_value(m_nav_settings.min_cno);  // Minimum satellite signal level [dBHz]
	CFG::NAVSPG::INIFIX3D.set_value(0);                           // Do not require initial 3D fix
	//CFG::NAVSPG::WKNROLLOVER.set_value(0);                          // Default GPS week rollover min value allowed is 1 - default is 2148, do not update
	CFG::NAVSPG::ACKAIDING.set_value(1);  // Acknowledge assistance messages
	CFG::NAVSPG::SIGATTCOMP.set_value(static_cast<uint8_t>(
	    CFG::NAVSPG::SIGATTCOMP_VALUES::SIGCOMP_AUTO));  // Auto signal attenuation compensation (patch antenna)

	// ANA (AssistNow Autonomous) Configuration
	CFG::ANA::USE_ANA.set_value(m_nav_settings.assistnow_autonomous_enable
	                                ? static_cast<uint8_t>(CFG::ANA::USE_ANA_VALUES::ANA_ENABLED)
									: static_cast<uint8_t>(CFG::ANA::USE_ANA_VALUES::ANA_DISABLED));
	CFG::ANA::ORBMAXERR.set_value(m_nav_settings.orbmaxerr);  // Maximum orbit error for AssistNow Autonomous [m]

	// Collect parameters into a vector
	std::vector<CFG::UBXParameter> navspg_ana_config = { CFG::NAVSPG::INFIL_MINSVS, CFG::NAVSPG::INFIL_MAXSVS,
	                                                     CFG::NAVSPG::INFIL_MINCNO, CFG::NAVSPG::INIFIX3D,
	                                                     //CFG::NAVSPG::WKNROLLOVER,
	                                                     CFG::NAVSPG::ACKAIDING, CFG::NAVSPG::SIGATTCOMP,
	                                                     CFG::ANA::USE_ANA, CFG::ANA::ORBMAXERR };

	// Create and send VALSET message
	alignas(CFG::VALSET::MSG_VALSET)
	    uint8_t nav_ana_valset_msg_storage[sizeof(CFG::VALSET::MSG_VALSET) + CFG::VALSET::MSG_VALSET_MAX_CFG];
	auto &nav_ana_valset_msg =
	    *new (nav_ana_valset_msg_storage) CFG::VALSET::MSG_VALSET(0x00, CFG::VALSET::LAYERS::RAM, navspg_ana_config);
	size_t cfgDataSize = nav_ana_valset_msg.get_cfgData_size(navspg_ana_config);
	initiate_timeout();

	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_VALSET, nav_ana_valset_msg,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_ACK, cfgDataSize);
}

void M10QAsyncReceiver::supply_time_assistance() {
	// Le recepteur qui a garde sa BBR a sa propre base de temps GNSS, forcement
	// meilleure que la notre. On ne lui apprend rien, on ne risque que de le
	// contraindre a tort.
	if (m_bbr_retained) {
		DEBUG_INFO("M10QAsyncReceiver::supply_time_assistance: BBR retenue — le recepteur a sa propre heure, injection "
		           "inutile");
		m_step++;
		m_op_state = OpState::IDLE;
		run_state_machine();
		return;
	}

	// Incertitude REELLE sur notre heure. Zero = provenance non bornable
	// (heure restauree du flash, ou horloge virtuelle): dans ce cas le champ
	// tAccS ne peut pas etre rempli honnetement, et une heure fausse annoncee
	// comme sure est pire que pas d'heure du tout — elle restreint la fenetre
	// de recherche du recepteur autour d'une valeur erronee.
	const unsigned int tacc = rtc->time_accuracy_s();
	if (tacc == 0) {
		DEBUG_INFO("M10QAsyncReceiver::supply_time_assistance: heure de provenance non bornable (%s) — pas d'injection",
		           rtc->source() == RtcSource::RESTORED ? "restauree du flash" : "jamais synchronisee");
		m_step++;
		m_op_state = OpState::IDLE;
		run_state_machine();
		return;
	}

	DEBUG_INFO("M10QAsyncReceiver::supply_time_assistance: MGA-INI-TIME tAccS=%u s (age %u s, derive %d ppm)", tacc,
	           rtc->age_s(), (int)rtc->drift_ppm());
	uint16_t year;
	uint8_t month, day, hour, min, sec;

	convert_datetime_to_epoch(rtc->gettime(), year, month, day, hour, min, sec);

	MGA::MSG_INI_TIME_UTC cfg_msg_ini_time_utc = {
		.type = 0x10,
		.version = 0x00,
		.ref = 0x00,
		.leapSecs = -128,  // Number of leap seconds unknown
		.year = year,
		.month = month,
		.day = day,
		.hour = hour,
		.minute = min,
		.second = sec,
		.reserved1 = 0,
		.ns = 0,
		.tAccS = (uint16_t)tacc,  // incertitude mesuree, plus une constante optimiste
		.reserved2 = { 0 },
		.tAccNs = 0
	};

	initiate_timeout();
	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_MGA, MGA::ID_INI_TIME_UTC, cfg_msg_ini_time_utc,
	                                    MessageClass::MSG_CLASS_MGA, MGA::ID_ACK);
}

void M10QAsyncReceiver::supply_position_assistance() {
	const auto &last_gps = configuration_store->get_last_gps_entry();
	if (!last_gps.info.valid) {
		DEBUG_TRACE("M10QAsyncReceiver::supply_position_assistance: no valid last position");
		return;
	}

	// L'age de la position ne se calcule que si l'heure courante est fiable.
	// Avec une heure restauree du flash, l'ecart peut valoir des semaines et
	// l'age serait une fiction — on prefere ne rien injecter.
	if (rtc->time_accuracy_s() == 0) {
		DEBUG_INFO("M10QAsyncReceiver::supply_position_assistance: heure non fiable — age de la position incalculable, "
		           "pas d'injection");
		return;
	}

	// Age de la derniere position connue.
	const std::time_t fix_t = convert_epochtime(last_gps.info.year, last_gps.info.month, last_gps.info.day,
	                                            last_gps.info.hour, last_gps.info.min, last_gps.info.sec);
	const std::time_t now_t = rtc->gettime();
	const unsigned long age_s = (now_t > fix_t) ? (unsigned long)(now_t - fix_t) : 0UL;

	// Vitesse de reference: celle MESUREE au dernier fix (gSpeed, mm/s), bornee.
	// Le plancher couvre l'animal immobile au moment du fix mais parti depuis;
	// le plafond evite qu'une valeur aberrante fasse exploser le rayon.
	static constexpr unsigned long POS_SPEED_FLOOR_CM_S = 50;    // 0,5 m/s
	static constexpr unsigned long POS_SPEED_CEIL_CM_S = 500;    // 5 m/s
	static constexpr unsigned long POS_ACC_MAX_CM = 30000000UL;  // 300 km
	unsigned long speed_cm_s = (last_gps.info.gSpeed > 0) ? (unsigned long)last_gps.info.gSpeed / 10UL : 0UL;
	if (speed_cm_s < POS_SPEED_FLOOR_CM_S) speed_cm_s = POS_SPEED_FLOOR_CM_S;
	if (speed_cm_s > POS_SPEED_CEIL_CM_S) speed_cm_s = POS_SPEED_CEIL_CM_S;

	// Rayon d'incertitude honnete: precision du fix + marge + ce que la bete a
	// pu parcourir depuis. Sans ce terme, on annoncait au recepteur une position
	// vieille de plusieurs jours avec la precision qu'elle avait a l'instant du
	// fix — une contrainte fausse qu'il utilise pour restreindre sa recherche.
	const unsigned long acc_cm = (unsigned long)last_gps.info.hAcc / 10UL + 100UL + age_s * speed_cm_s;
	if (acc_cm > POS_ACC_MAX_CM) {
		DEBUG_INFO("M10QAsyncReceiver::supply_position_assistance: position trop vieille (age %lu s, rayon %lu km) — "
		           "pas d'injection",
		           age_s, acc_cm / 100000UL);
		return;
	}

	DEBUG_INFO(
	    "M10QAsyncReceiver::supply_position_assistance: MGA-INI-POS lat=%f lon=%f rayon=%lu m (age %lu s, %lu cm/s)",
	    last_gps.info.lat, last_gps.info.lon, acc_cm / 100UL, age_s, speed_cm_s);

	MGA::MSG_INI_POS_LLH msg = {
		.type = 0x01,
		.version = 0x00,
		.reserved1 = { 0 },
		.lat = static_cast<int32_t>(last_gps.info.lat * 1e7),
		.lon = static_cast<int32_t>(last_gps.info.lon * 1e7),
		.alt = last_gps.info.height / 10,  // mm -> cm
		.posAcc = (uint32_t)acc_cm,
	};

	initiate_timeout();
	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_MGA, MGA::ID_INI_TIME_UTC, msg,
	                                    MessageClass::MSG_CLASS_MGA, MGA::ID_ACK);
}

void M10QAsyncReceiver::disable_odometer() {
	DEBUG_TRACE("M10QAsyncReceiver::disable_odometer: Disabling odometer with VALSET ->");


	// Set the odometer parameters to the desired values for disabling
	CFG::ODO::USE_ODO.set_value(0x00);                    // Disable odometer usage
	CFG::ODO::USE_COG.set_value(0x00);                    // Disable low-speed course over ground filter
	CFG::ODO::OUTLPVEL.set_value(0x00);                   // Disable low-pass filtered velocity output
	CFG::ODO::OUTLPCOG.set_value(0x00);                   // Disable low-pass filtered course over ground output
	CFG::ODO::PROFILE.set_value(CFG::ODO::Profile::RUN);  // Set profile to CUSTOM (disabled state)
	CFG::ODO::COG_MAXSPEED.set_value(1);    // Set minimal max speed for course over ground filter (example value)
	CFG::ODO::COG_MAXPOSACC.set_value(50);  // Set example value for max position accuracy
	CFG::ODO::VEL_LPGAIN.set_value(153);    // Set velocity low-pass filter gain
	CFG::ODO::COG_LPGAIN.set_value(76);     // Set course over ground low-pass filter gain

	// Collect all parameters in a vector for MSG_VALSET
	std::vector<CFG::UBXParameter> odo_config = { CFG::ODO::USE_ODO,       CFG::ODO::USE_COG,    CFG::ODO::OUTLPVEL,
	                                              CFG::ODO::OUTLPCOG,      CFG::ODO::PROFILE,    CFG::ODO::COG_MAXSPEED,
	                                              CFG::ODO::COG_MAXPOSACC, CFG::ODO::VEL_LPGAIN, CFG::ODO::COG_LPGAIN };

	// Create the VALSET message with the odometer parameters
	alignas(CFG::VALSET::MSG_VALSET)
	    uint8_t odo_valset_msg_storage[sizeof(CFG::VALSET::MSG_VALSET) + CFG::VALSET::MSG_VALSET_MAX_CFG];
	auto &odo_valset_msg =
	    *new (odo_valset_msg_storage) CFG::VALSET::MSG_VALSET(0x00, CFG::VALSET::LAYERS::RAM, odo_config);
	size_t cfgDataSize = odo_valset_msg.get_cfgData_size(odo_config);
	initiate_timeout();

	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_VALSET, odo_valset_msg,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_ACK, cfgDataSize);
}

void M10QAsyncReceiver::disable_timepulse_output() {
	DEBUG_TRACE("M10QAsyncReceiver::disable_timepulse_output: Configuring timepulse output with VALSET ->");
	// Minimal configuration for disabling time pulse with 1 Hz setting
	CFG::TP::PULSE_DEF.set_value(CFG::TP::PULSE_DEF::PERIOD);  // Set pulse mode to period
	CFG::TP::PERIOD_TP1.set_value(1000000);                    // Set period to 1 second (1 Hz) in microseconds
	CFG::TP::PERIOD_LOCK_TP1.set_value(1000000);               // Set locked period to 1 second in microseconds
	CFG::TP::LEN_TP1.set_value(0);                             // Set pulse length to 0 (no pulse)
	CFG::TP::TP1_ENA.set_value(0);                             // Disable the time pulse
	CFG::TP::POL_TP1.set_value(0);                             // Set time pulse polarity to default (falling edge)

	// Collect only necessary parameters in the configuration vector
	std::vector<CFG::UBXParameter> tp_config = { CFG::TP::PULSE_DEF, CFG::TP::PERIOD_TP1, CFG::TP::PERIOD_LOCK_TP1,
	                                             CFG::TP::LEN_TP1,   CFG::TP::POL_TP1,    CFG::TP::TP1_ENA };

	// Create the VALSET message with the timepulse parameters
	alignas(CFG::VALSET::MSG_VALSET)
	    uint8_t tp_valset_msg_storage[sizeof(CFG::VALSET::MSG_VALSET) + CFG::VALSET::MSG_VALSET_MAX_CFG];
	auto &tp_valset_msg =
	    *new (tp_valset_msg_storage) CFG::VALSET::MSG_VALSET(0x00, CFG::VALSET::LAYERS::RAM, tp_config);
	size_t cfgDataSize = tp_valset_msg.get_cfgData_size(tp_config);
	initiate_timeout();

	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_VALSET, tp_valset_msg,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_ACK, cfgDataSize);
}

void M10QAsyncReceiver::enable_nav_pvt_message() {
	DEBUG_TRACE("M10QAsyncReceiver::enable_nav_pvt_message: Configuring NAV PVT output on UART1 with VALSET ->");

	// Set output rate for UBX-NAV-PVT message on UART1 port
	CFG::MSGOUT::NAV_PVT_UART1.set_value(1);  // Set rate to 1 (enables message output on UART1)

	// Collect the NAV PVT UART1 configuration parameter in a vector
	std::vector<CFG::UBXParameter> nav_pvt_config = { CFG::MSGOUT::NAV_PVT_UART1 };

	// Create and send the VALSET message with the NAV PVT configuration
	alignas(CFG::VALSET::MSG_VALSET)
	    uint8_t nav_pvt_valset_msg_storage[sizeof(CFG::VALSET::MSG_VALSET) + CFG::VALSET::MSG_VALSET_MAX_CFG];
	auto &nav_pvt_valset_msg =
	    *new (nav_pvt_valset_msg_storage) CFG::VALSET::MSG_VALSET(0x00, CFG::VALSET::LAYERS::RAM, nav_pvt_config);
	size_t cfgDataSize = nav_pvt_valset_msg.get_cfgData_size(nav_pvt_config);
	initiate_timeout();

	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_VALSET, nav_pvt_valset_msg,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_ACK, cfgDataSize);
}

void M10QAsyncReceiver::disable_nav_pvt_message() {
	DEBUG_TRACE("M10QAsyncReceiver::disable_nav_pvt_message: Disabling NAV-PVT message with VALSET ->");

	CFG::MSGOUT::NAV_PVT_UART1.set_value(0);  // Disable NAV-PVT message on UART1

	std::vector<CFG::UBXParameter> nav_pvt_config = { CFG::MSGOUT::NAV_PVT_UART1 };
	alignas(CFG::VALSET::MSG_VALSET)
	    uint8_t nav_pvt_valset_msg_storage[sizeof(CFG::VALSET::MSG_VALSET) + CFG::VALSET::MSG_VALSET_MAX_CFG];
	auto &nav_pvt_valset_msg =
	    *new (nav_pvt_valset_msg_storage) CFG::VALSET::MSG_VALSET(0x00, CFG::VALSET::LAYERS::RAM, nav_pvt_config);

	size_t cfgDataSize = nav_pvt_valset_msg.get_cfgData_size(nav_pvt_config);
	initiate_timeout();

	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_VALSET, nav_pvt_valset_msg,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_ACK, cfgDataSize);
}

void M10QAsyncReceiver::enable_nav_dop_message() {
	DEBUG_TRACE("M10QAsyncReceiver::enable_nav_dop_message: Enabling NAV-DOP message with VALSET ->");

	CFG::MSGOUT::NAV_DOP_UART1.set_value(1);  // Enable NAV-DOP message on UART1

	std::vector<CFG::UBXParameter> nav_dop_config = { CFG::MSGOUT::NAV_DOP_UART1 };
	alignas(CFG::VALSET::MSG_VALSET)
	    uint8_t nav_dop_valset_msg_storage[sizeof(CFG::VALSET::MSG_VALSET) + CFG::VALSET::MSG_VALSET_MAX_CFG];
	auto &nav_dop_valset_msg =
	    *new (nav_dop_valset_msg_storage) CFG::VALSET::MSG_VALSET(0x00, CFG::VALSET::LAYERS::RAM, nav_dop_config);

	size_t cfgDataSize = nav_dop_valset_msg.get_cfgData_size(nav_dop_config);
	initiate_timeout();

	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_VALSET, nav_dop_valset_msg,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_ACK, cfgDataSize);
}

void M10QAsyncReceiver::disable_nav_dop_message() {
	DEBUG_TRACE("M10QAsyncReceiver::disable_nav_dop_message: Disabling NAV-DOP message with VALSET ->");

	CFG::MSGOUT::NAV_DOP_UART1.set_value(0);  // Disable NAV-DOP message on UART1

	std::vector<CFG::UBXParameter> nav_dop_config = { CFG::MSGOUT::NAV_DOP_UART1 };
	alignas(CFG::VALSET::MSG_VALSET)
	    uint8_t nav_dop_valset_msg_storage[sizeof(CFG::VALSET::MSG_VALSET) + CFG::VALSET::MSG_VALSET_MAX_CFG];
	auto &nav_dop_valset_msg =
	    *new (nav_dop_valset_msg_storage) CFG::VALSET::MSG_VALSET(0x00, CFG::VALSET::LAYERS::RAM, nav_dop_config);

	size_t cfgDataSize = nav_dop_valset_msg.get_cfgData_size(nav_dop_config);
	initiate_timeout();

	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_VALSET, nav_dop_valset_msg,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_ACK, cfgDataSize);
}

void M10QAsyncReceiver::enable_nav_status_message() {
	DEBUG_TRACE("M10QAsyncReceiver::enable_nav_status_message: Enabling NAV-STATUS message with VALSET ->");

	CFG::MSGOUT::NAV_STATUS_UART1.set_value(1);  // Enable NAV-STATUS message on UART1

	std::vector<CFG::UBXParameter> nav_status_config = { CFG::MSGOUT::NAV_STATUS_UART1 };
	alignas(CFG::VALSET::MSG_VALSET)
	    uint8_t nav_status_valset_msg_storage[sizeof(CFG::VALSET::MSG_VALSET) + CFG::VALSET::MSG_VALSET_MAX_CFG];
	auto &nav_status_valset_msg =
	    *new (nav_status_valset_msg_storage) CFG::VALSET::MSG_VALSET(0x00, CFG::VALSET::LAYERS::RAM, nav_status_config);

	size_t cfgDataSize = nav_status_valset_msg.get_cfgData_size(nav_status_config);
	initiate_timeout();

	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_VALSET, nav_status_valset_msg,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_ACK, cfgDataSize);
}

void M10QAsyncReceiver::disable_nav_status_message() {
	DEBUG_TRACE("M10QAsyncReceiver::disable_nav_status_message: Disabling NAV-STATUS message with VALSET ->");

	CFG::MSGOUT::NAV_STATUS_UART1.set_value(0);  // Disable NAV-STATUS message on UART1

	std::vector<CFG::UBXParameter> nav_status_config = { CFG::MSGOUT::NAV_STATUS_UART1 };
	alignas(CFG::VALSET::MSG_VALSET)
	    uint8_t nav_status_valset_msg_storage[sizeof(CFG::VALSET::MSG_VALSET) + CFG::VALSET::MSG_VALSET_MAX_CFG];
	auto &nav_status_valset_msg =
	    *new (nav_status_valset_msg_storage) CFG::VALSET::MSG_VALSET(0x00, CFG::VALSET::LAYERS::RAM, nav_status_config);

	size_t cfgDataSize = nav_status_valset_msg.get_cfgData_size(nav_status_config);
	initiate_timeout();

	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_VALSET, nav_status_valset_msg,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_ACK, cfgDataSize);
}

void M10QAsyncReceiver::enable_nav_sat_message() {
	DEBUG_TRACE("M10QAsyncReceiver::enable_nav_sat_message: Enabling NAV-SAT message with VALSET ->");

	CFG::MSGOUT::NAV_SAT_UART1.set_value(1);  // Enable NAV-SAT message on UART1

	std::vector<CFG::UBXParameter> nav_sat_config = { CFG::MSGOUT::NAV_SAT_UART1 };
	alignas(CFG::VALSET::MSG_VALSET)
	    uint8_t nav_sat_valset_msg_storage[sizeof(CFG::VALSET::MSG_VALSET) + CFG::VALSET::MSG_VALSET_MAX_CFG];
	auto &nav_sat_valset_msg =
	    *new (nav_sat_valset_msg_storage) CFG::VALSET::MSG_VALSET(0x00, CFG::VALSET::LAYERS::RAM, nav_sat_config);

	size_t cfgDataSize = nav_sat_valset_msg.get_cfgData_size(nav_sat_config);
	initiate_timeout();

	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_VALSET, nav_sat_valset_msg,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_ACK, cfgDataSize);
}

void M10QAsyncReceiver::disable_nav_sat_message() {
	DEBUG_TRACE("M10QAsyncReceiver::disable_nav_sat_message: Disabling NAV-SAT message with VALSET ->");

	CFG::MSGOUT::NAV_SAT_UART1.set_value(0);  // Disable NAV-SAT message on UART1

	std::vector<CFG::UBXParameter> nav_sat_config = { CFG::MSGOUT::NAV_SAT_UART1 };
	alignas(CFG::VALSET::MSG_VALSET)
	    uint8_t nav_sat_valset_msg_storage[sizeof(CFG::VALSET::MSG_VALSET) + CFG::VALSET::MSG_VALSET_MAX_CFG];
	auto &nav_sat_valset_msg =
	    *new (nav_sat_valset_msg_storage) CFG::VALSET::MSG_VALSET(0x00, CFG::VALSET::LAYERS::RAM, nav_sat_config);

	size_t cfgDataSize = nav_sat_valset_msg.get_cfgData_size(nav_sat_config);
	initiate_timeout();

	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_VALSET, nav_sat_valset_msg,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_ACK, cfgDataSize);
}

void M10QAsyncReceiver::enable_rxm_measc12_message() {
	DEBUG_TRACE("M10QAsyncReceiver::enable_rxm_measc12_message");
	CFG::MSGOUT::RXM_MEASC12_UART1.set_value(1);
	std::vector<CFG::UBXParameter> config = { CFG::MSGOUT::RXM_MEASC12_UART1 };
	alignas(CFG::VALSET::MSG_VALSET)
	    uint8_t valset_msg_storage[sizeof(CFG::VALSET::MSG_VALSET) + CFG::VALSET::MSG_VALSET_MAX_CFG];
	auto &valset_msg = *new (valset_msg_storage) CFG::VALSET::MSG_VALSET(0x00, CFG::VALSET::LAYERS::RAM, config);
	size_t cfgDataSize = valset_msg.get_cfgData_size(config);
	initiate_timeout();
	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_VALSET, valset_msg,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_ACK, cfgDataSize);
}

void M10QAsyncReceiver::enable_rxm_meas20_message() {
	DEBUG_TRACE("M10QAsyncReceiver::enable_rxm_meas20_message");
	CFG::MSGOUT::RXM_MEAS20_UART1.set_value(1);
	std::vector<CFG::UBXParameter> config = { CFG::MSGOUT::RXM_MEAS20_UART1 };
	alignas(CFG::VALSET::MSG_VALSET)
	    uint8_t valset_msg_storage[sizeof(CFG::VALSET::MSG_VALSET) + CFG::VALSET::MSG_VALSET_MAX_CFG];
	auto &valset_msg = *new (valset_msg_storage) CFG::VALSET::MSG_VALSET(0x00, CFG::VALSET::LAYERS::RAM, config);
	size_t cfgDataSize = valset_msg.get_cfgData_size(config);
	initiate_timeout();
	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_VALSET, valset_msg,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_ACK, cfgDataSize);
}

void M10QAsyncReceiver::enable_rxm_meas50_message() {
	DEBUG_TRACE("M10QAsyncReceiver::enable_rxm_meas50_message");
	CFG::MSGOUT::RXM_MEAS50_UART1.set_value(1);
	std::vector<CFG::UBXParameter> config = { CFG::MSGOUT::RXM_MEAS50_UART1 };
	alignas(CFG::VALSET::MSG_VALSET)
	    uint8_t valset_msg_storage[sizeof(CFG::VALSET::MSG_VALSET) + CFG::VALSET::MSG_VALSET_MAX_CFG];
	auto &valset_msg = *new (valset_msg_storage) CFG::VALSET::MSG_VALSET(0x00, CFG::VALSET::LAYERS::RAM, config);
	size_t cfgDataSize = valset_msg.get_cfgData_size(config);
	initiate_timeout();
	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_VALSET, valset_msg,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_ACK, cfgDataSize);
}

void M10QAsyncReceiver::setup_gnss_channel_sharing() {
	DEBUG_TRACE("M10QAsyncReceiver::setup_gnss_channel_sharing: Configuring GNSS signals with VALSET (mask=0x%02X) ->",
	            m_nav_settings.constellation_mask);

	// Decode constellation bitmask: bit0=GPS, bit1=GAL, bit2=GLO, bit3=BDS, bit4=QZSS, bit5=SBAS
	uint8_t gps_en = (m_nav_settings.constellation_mask & 0x01) ? 1 : 0;
	uint8_t gal_en = (m_nav_settings.constellation_mask & 0x02) ? 1 : 0;
	uint8_t glo_en = (m_nav_settings.constellation_mask & 0x04) ? 1 : 0;
	uint8_t bds_en = (m_nav_settings.constellation_mask & 0x08) ? 1 : 0;
	uint8_t qzss_en = (m_nav_settings.constellation_mask & 0x10) ? 1 : 0;
	uint8_t sbas_en = (m_nav_settings.constellation_mask & 0x20) ? 1 : 0;

	CFG::SIGNAL::GPS_ENA.set_value(gps_en);
	CFG::SIGNAL::GPS_L1CA_ENA.set_value(gps_en);
	CFG::SIGNAL::GAL_ENA.set_value(gal_en);
	CFG::SIGNAL::GAL_E1_ENA.set_value(gal_en);
	CFG::SIGNAL::GLO_ENA.set_value(glo_en);
	CFG::SIGNAL::GLO_L1_ENA.set_value(glo_en);
	CFG::SIGNAL::BDS_ENA.set_value(bds_en);
	CFG::SIGNAL::BDS_B1C_ENA.set_value(bds_en);  // M10Q uses B1C (not B1I)
	CFG::SIGNAL::BDS_B1_ENA.set_value(0);        // B1I disabled on M10Q
	CFG::SIGNAL::QZSS_ENA.set_value(qzss_en);
	CFG::SIGNAL::QZSS_L1CA_ENA.set_value(qzss_en);
	CFG::SIGNAL::QZSS_L1S_ENA.set_value(0);
	CFG::SIGNAL::SBAS_ENA.set_value(sbas_en);
	CFG::SIGNAL::SBAS_L1CA_ENA.set_value(sbas_en);

	// Collect all parameters in a vector for MSG_VALSET
	std::vector<UBX::CFG::UBXParameter> gnss_signal_config = {
		CFG::SIGNAL::GPS_ENA,     CFG::SIGNAL::GPS_L1CA_ENA, CFG::SIGNAL::SBAS_ENA,      CFG::SIGNAL::SBAS_L1CA_ENA,
		CFG::SIGNAL::GAL_ENA,     CFG::SIGNAL::GAL_E1_ENA,   CFG::SIGNAL::BDS_ENA,       CFG::SIGNAL::BDS_B1_ENA,
		CFG::SIGNAL::BDS_B1C_ENA, CFG::SIGNAL::QZSS_ENA,     CFG::SIGNAL::QZSS_L1CA_ENA, CFG::SIGNAL::QZSS_L1S_ENA,
		CFG::SIGNAL::GLO_ENA,     CFG::SIGNAL::GLO_L1_ENA
	};

	// Create the VALSET message for GNSS signal configuration
	alignas(CFG::VALSET::MSG_VALSET)
	    uint8_t gnss_valset_msg_storage[sizeof(CFG::VALSET::MSG_VALSET) + CFG::VALSET::MSG_VALSET_MAX_CFG];
	auto &gnss_valset_msg = *new (gnss_valset_msg_storage) CFG::VALSET::MSG_VALSET(
	    0x00, CFG::VALSET::LAYERS::BBR | CFG::VALSET::LAYERS::RAM, gnss_signal_config);
	size_t cfgDataSize = gnss_valset_msg.get_cfgData_size(gnss_signal_config);
	initiate_timeout();

	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_VALSET, gnss_valset_msg,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_ACK, cfgDataSize);

	m_ubx_comms.wait_send();
}

void M10QAsyncReceiver::fetch_navigation_database() {
	UBX::Empty msg_dbd = {};
	initiate_timeout();
	m_ubx_comms.send_packet(MessageClass::MSG_CLASS_MGA, MGA::ID_DBD, msg_dbd);
}

void M10QAsyncReceiver::dump_navigation_database(unsigned int len) {
	for (unsigned int i = 0; i < len; i++)
		printf("%02X", m_navigation_database[i]);
	printf("\n");
}

void M10QAsyncReceiver::query_mon_ver() {
	DEBUG_TRACE("M10QAsyncReceiver::query_mon_ver");
	UBX::Empty msg = {};
	initiate_timeout();
	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_MON, MON::ID_VER, msg, MessageClass::MSG_CLASS_MON,
	                                    MON::ID_VER);
}

void M10QAsyncReceiver::query_sec_uniqid() {
	DEBUG_TRACE("M10QAsyncReceiver::query_sec_uniqid");
	UBX::Empty msg = {};
	initiate_timeout();
	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_SEC, SEC::ID_UNIQID, msg, MessageClass::MSG_CLASS_SEC,
	                                    SEC::ID_UNIQID);
}

void M10QAsyncReceiver::react(const UBXCommsEventMonVer &ver) {
	if (m_op_state == OpState::PENDING) {
		cancel_timeout();
		std::memcpy(m_gnss_sw_version, ver.swVersion, sizeof(m_gnss_sw_version));
		std::memcpy(m_gnss_hw_version, ver.hwVersion, sizeof(m_gnss_hw_version));
		m_gnss_sw_version[sizeof(m_gnss_sw_version) - 1] = '\0';
		m_gnss_hw_version[sizeof(m_gnss_hw_version) - 1] = '\0';
		DEBUG_INFO("GNSS SW: %s HW: %s", m_gnss_sw_version, m_gnss_hw_version);
		m_op_state = OpState::SUCCESS;
		run_state_machine();
	}
}

void M10QAsyncReceiver::react(const UBXCommsEventSecUniqId &uid) {
	if (m_op_state == OpState::PENDING) {
		cancel_timeout();
		std::memcpy(m_gnss_unique_id, uid.uniqueId, sizeof(m_gnss_unique_id));
		m_gnss_info_valid = true;
		DEBUG_INFO("GNSS UID: %02X%02X%02X%02X%02X", m_gnss_unique_id[0], m_gnss_unique_id[1], m_gnss_unique_id[2],
		           m_gnss_unique_id[3], m_gnss_unique_id[4]);
		notify(GPSEventDeviceInfoReady{});
		m_op_state = OpState::SUCCESS;
		run_state_machine();
	}
}

GNSSDeviceInfo M10QAsyncReceiver::get_device_info() const {
	GNSSDeviceInfo info = {};
	if (m_gnss_info_valid) {
		std::memcpy(info.swVersion, m_gnss_sw_version, sizeof(info.swVersion));
		std::memcpy(info.hwVersion, m_gnss_hw_version, sizeof(info.hwVersion));
		std::memcpy(info.uniqueId, m_gnss_unique_id, sizeof(info.uniqueId));
		info.valid = true;
	}
	return info;
}

GNSSAlmanacStatus M10QAsyncReceiver::get_almanac_status(unsigned int ano_stale_threshold_s) const {
	GNSSAlmanacStatus status = {};

	if (!main_filesystem) {
		status.stale = true;
		return status;
	}

	try {
		LFSFile file(main_filesystem, "gps_config.dat", LFS_O_RDONLY);
		status.file_present = true;
		status.file_size = (unsigned int)file.size();

		// Parse UBX messages to count ANO records and check dates.
		// Only the 6-byte header and the first 8 bytes of an ANO payload
		// (up through the date fields) are inspected, so we keep two tiny
		// stack buffers and seek past the rest. The previous 256-byte stack
		// buffer (MAX_PACKET_LEN) was a stack-pressure risk on large almanac
		// files parsed synchronously from the DTE handler context.
		uint8_t header_buf[sizeof(Header)];
		uint8_t ano_head_buf[8];  // type, version, svId, gnssId, year, month, day, reserved1
		unsigned int total_records = 0;
		unsigned int valid_records = 0;
		std::time_t deltatime = (std::time_t)0xFFFFFFFF;
		// Mitigation M2 (2026-05): treat a virtual RTC (cold-first-boot
		// initialised to 1 by main.cpp's fallback) as "not available" for ANO
		// staleness. Before this guard the previous abs() math happened to be
		// safe (deltatime ~50 years → always > threshold → marked stale), but
		// that was accidental. Anything below year 2000 is virtual or unset.
		constexpr std::time_t RTC_MIN_REAL_FOR_ANO = 946684800;  // 2000-01-01
		bool rtc_available = rtc && rtc->is_set() && rtc->gettime() >= RTC_MIN_REAL_FOR_ANO;
		std::time_t now = rtc_available ? rtc->gettime() : 0;

		// Hard cap on iterations: defense in depth against a corrupted LFS file
		// that could otherwise loop forever if every read returns sizeof(Header)
		// without advancing. Real-world ANO files have a few hundred records;
		// 20000 is well above any plausible legitimate count.
		constexpr unsigned int MAX_ITERATIONS = 20000;
		unsigned int iter = 0;

		while (iter++ < MAX_ITERATIONS) {
			// Periodic watchdog kick — parsing a multi-KB almanac file does
			// hundreds of synchronous LFS reads from the DTE handler context.
			if ((iter & 0x3F) == 0)  // every 64 iterations
				PMU::kick_watchdog();

			lfs_ssize_t sz = file.read(header_buf, sizeof(Header));
			if (sz != (lfs_ssize_t)(sizeof(Header))) break;

			Header *hdr = (Header *)header_buf;
			unsigned int payload_len = hdr->msgLength;
			// Payload + 2-byte CRC must fit a reasonable UBX packet size;
			// reject garbage that would seek us way past EOF or wrap LFS.
			if (payload_len + 2 > MAX_PACKET_LEN) break;

			bool is_ano = (hdr->msgClass == MessageClass::MSG_CLASS_MGA && hdr->msgId == MGA::ID_ANO);

			if (is_ano && rtc_available && payload_len >= sizeof(ano_head_buf)) {
				sz = file.read(ano_head_buf, sizeof(ano_head_buf));
				if (sz != (lfs_ssize_t)sizeof(ano_head_buf)) break;
				// Skip the rest of the payload (data[64] + reserved2[4]) plus the 2-byte CRC.
				lfs_soff_t off = file.seek((lfs_soff_t)(payload_len - sizeof(ano_head_buf) + 2), LFS_SEEK_CUR);
				if (off < 0) break;

				total_records++;

				// ano_head_buf layout: [0]=type [1]=version [2]=svId [3]=gnssId
				//                      [4]=year [5]=month  [6]=day [7]=reserved1
				std::time_t ano_time =
				    convert_epochtime(2000 + ano_head_buf[4], ano_head_buf[5], ano_head_buf[6], 12, 0, 0);
				std::time_t timediff = std::abs(ano_time - now);

				if (timediff < deltatime) {
					deltatime = timediff;
					valid_records = 1;
				} else if (timediff == deltatime) {
					valid_records++;
				}
			} else {
				// Non-ANO message, or ANO without RTC: skip payload + CRC entirely.
				lfs_soff_t off = file.seek((lfs_soff_t)(payload_len + 2), LFS_SEEK_CUR);
				if (off < 0) break;
				if (is_ano) total_records++;
			}
		}

		status.total_records = total_records;
		status.valid_records = valid_records;

		if (!rtc_available || (ano_stale_threshold_s > 0 && deltatime >= ano_stale_threshold_s)) {
			status.stale = true;
			status.valid_records = 0;
		}

	} catch (...) {
		status.file_present = false;
		status.stale = true;
	}

	return status;
}

bool M10QAsyncReceiver::start_bridge(PassthroughCallback rx_callback) {
	if (m_bridge_active) return true;

	// En deep-idle l'UART est deinit et ses canaux PPI sont liberes: la suite de
	// cette fonction n'appelle exit_shutdown() que depuis `idle`, puis touche
	// l'UART -> canal PPI libere -> APP_ERROR_CHECK_BOOL(false) -> reset du SoC.
	if (STATE_EQUAL(backupidle) || STATE_EQUAL(enterbackup)) {
		DEBUG_WARN("M10QAsyncReceiver::start_bridge: refuse — GNSS en deep-idle, "
		           "sortir d'abord avec $GNSSBCKP#001;0");
		return false;
	}

	// Cancel any pending state machine task
	cancel_timeout();
	system_scheduler->cancel_task(m_state_machine_handle);

	// Power on GNSS module and init UART (if not already on)
	const bool was_idle = (m_state == State::idle);
	if (was_idle) {
		exit_shutdown();
	}

	// Se mettre au debit auquel le M10Q repond REELLEMENT — mais lequel depend de
	// ce qu'on vient de faire au recepteur:
	//   - on arrivait de `idle`: exit_shutdown() ci-dessus a coupe puis reallume
	//     le rail, donc le M10Q vient de faire un POR et repart au debit de BOOT
	//     (9600 si la BBR est perdue, MAX si elle est retenue) — c'est l'indice
	//     de sondage qui le sait, pas m_synced_baud;
	//   - sinon le recepteur est deja alimente et configure: il parle a
	//     m_synced_baud.
	// Constate au banc: forcer m_synced_baud (460800, herite de la derniere
	// session) sur un recepteur fraichement redemarre a 9600 rendait le bridge
	// muet — le defaut meme que ce correctif etait cense supprimer, mais dans
	// l'autre sens.
	const unsigned int bridge_baud = was_idle ? boot_baud_for_step(0) : m_synced_baud;
	m_ubx_comms.set_baudrate(bridge_baud);

	// Enable passthrough: raw UART RX goes to callback
	m_ubx_comms.set_passthrough(true, rx_callback);
	m_bridge_active = true;

	DEBUG_INFO("M10QAsyncReceiver: bridge mode ACTIVE (%u baud, %s)", bridge_baud,
	           was_idle ? "recepteur redemarre" : "recepteur deja configure");
	return true;
}

void M10QAsyncReceiver::stop_bridge() {
	if (!m_bridge_active) return;

	m_ubx_comms.set_passthrough(false);
	m_bridge_active = false;

	DEBUG_INFO("M10QAsyncReceiver: bridge mode STOPPED");

	// Power off the GNSS and return to idle
	enter_shutdown();
	// Remettre la MEME base que power_off_immediate(): le bridge a prempte tous
	// les clients (il annule les taches et coupe le rail sans passer par la FSM).
	// Sans cela m_num_power_on restait a 1 apres un PWRON GNSS suivi d'un bridge,
	// et plus AUCUNE session suivante ne pouvait couper le rail: ~25-30 mA en
	// continu jusqu'au prochain reset.
	m_state = State::idle;
	m_num_power_on = 0;
	m_powering_off = false;
	m_op_state = OpState::IDLE;
	m_step = 0;
	m_unrecoverable_error = false;
	m_pmreq_baud = DEFAULT_BAUDRATE;  // le rail vient d'etre coupe
	m_bbr_retained = false;
	// La liste doit rester alignee sur power_off_immediate(): ces cinq-la
	// manquaient, et un drapeau de deep-idle ou un compteur de reveil herite du
	// bridge se serait applique a la premiere session suivante.
	m_deep_idle_pending = false;
	m_enterbackup_warm = false;
	m_consecutive_wake_failures = 0;
	m_pmreq_verify_retries = 0;
	m_backupidle_entered_ms = 0;
}

bool M10QAsyncReceiver::bridge_send(const uint8_t *data, size_t len) {
	if (!m_bridge_active) return false;
	return m_ubx_comms.send_raw(data, len);
}

void M10QAsyncReceiver::bridge_process_rx() {
	if (m_bridge_active) m_ubx_comms.process_passthrough_rx();
}
