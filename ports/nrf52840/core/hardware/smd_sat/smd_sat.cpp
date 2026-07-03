/**
 * @file smd_sat.cpp
 * @brief SMD satellite device driver — state machine, TX, credentials, DFU.
 */

#include <cstdint>
#include <cstring>
#include <cmath>

#include "bsp.hpp"
#include "error.hpp"
#include "debug.hpp"
#include "gpio.hpp"
#include "scheduler.hpp"
#include "timer.hpp"
#include "binascii.hpp"
#include "smd_sat.hpp"
#include "pmu.hpp"

#include "nrf_delay.h"
#include "nrf_gpio.h"

extern Scheduler *system_scheduler;
extern Timer *system_timer;

#include "config_store.hpp"
extern ConfigurationStore *configuration_store;

#if SMD_UART
// True when the BLIND MAC profile is active: ARGOS_BLIND_EN set AND the mode is
// not SURFACING_BURST (surfacing runs its own progressive Doppler cascade; a
// module-owned retx burst would double-transmit). Fills clamped retx_nb (1..127)
// and retx_period_s. NOTE: reads the raw ARGOS_MODE — an LB/OoZ regime that
// overrides the mode to SURFACING_BURST is not resolved here; do not pair BLIND
// with a surfacing regime.
static bool smd_blind_active(unsigned int& retx_nb, unsigned int& retx_period_s) {
	if (!configuration_store) return false;
	// get_argos_configuration resolves the EFFECTIVE mode (LB/OoZ/HAULED) and
	// clears blind_en for SURFACING_BURST/DOPPLER. evaluate() inside is cached
	// (500 ms) and already runs many times per second, so this is cheap.
	ArgosConfig cfg;
	configuration_store->get_argos_configuration(cfg);
	if (!cfg.blind_en) return false;
	retx_nb = cfg.blind_retx_nb;
	if (retx_nb < 1) retx_nb = 1; else if (retx_nb > 127) retx_nb = 127;
	retx_period_s = cfg.blind_retx_period_s;
	return true;
}
#endif

// Runtime degraded-mode flag shared with smd_sat_cmd_spi.cpp via the inline
// accessors in smd_sat_registers.hpp. Default false — the autofallback path
// flips it when SMD_MAX_CONSECUTIVE_ERRORS is reached. Touched only by the
// SmdSat instance, but in scope here so SPI-level delays in cmd_spi can read
// it without needing a pointer back to SmdSat.
bool g_smdsat_use_safe_timings = false;

// Validation log helper — guarded by build-time flag (see scripts/build_*.sh
// --validation). Lets us tag state transitions / cooldown / degraded-mode
// events with a grep-friendly `[VAL-SAT]` prefix for post-deploy log analysis.
#ifndef VALIDATION_LOG_ENABLE
#define VALIDATION_LOG_ENABLE 0
#endif
#if VALIDATION_LOG_ENABLE
#define VAL_SAT(fmt, ...) DEBUG_INFO("[VAL-SAT] " fmt, ##__VA_ARGS__)
#else
#define VAL_SAT(fmt, ...) do {} while (0)
#endif

// SMD state-machine task priority — bumped above GNSS (M10Q uses DEFAULT_PRIORITY=7)
// so satellite TX path is not preempted by GPS init/fix work during surfacing burst.
// Lower number = higher priority (0 = highest, 7 = default).
static constexpr unsigned int SMD_SCHEDULER_PRIORITY = 4;

// TX latency instrumentation — call sites kept across the driver to allow
// re-enabling for future timing investigations. Currently NO-OP.
// To re-enable: uncomment the body and choose between console-only (fast, RTT/UART)
// or DEBUG_INFO (visible in system_log via DTE — but inflates total TX time by LFS commits;
// ping_ms isolated measurements remain accurate since t0 is captured after any prior log).
#define TXTRACE(fmt, ...) \
    do { \
        (void)fmt; \
        /* DEBUG_INFO("[TXTRACE +%u ms] " fmt, \
            static_cast<unsigned>(m_tx_trace_start_ms ? (PMU::get_timestamp_ms() - m_tx_trace_start_ms) : 0), \
            ##__VA_ARGS__); */ \
    } while (0)

#define MODSWITCH_LOG(delta_ms, fmt, ...) \
    do { \
        (void)(delta_ms); (void)fmt; \
        /* DEBUG_INFO("[MODSWITCH +%u ms] " fmt, static_cast<unsigned>(delta_ms), ##__VA_ARGS__); */ \
    } while (0)

// ============================================================================
// Modulation conversion helpers (used by send() and switch_modulation())
// ============================================================================

static SmdArgosModulation kineis_to_smd_mod(KineisModulation mode) {
	switch (mode) {
		case KineisModulation::LDK:  return ARGOS_MOD_LDK;
		case KineisModulation::VLDA4: return ARGOS_MOD_VLDA4;
		case KineisModulation::LDA2:
		default:                      return ARGOS_MOD_LDA2;
	}
}

static KineisModulation smd_to_kineis_mod(SmdArgosModulation mode) {
	switch (mode) {
		case ARGOS_MOD_LDK:  return KineisModulation::LDK;
		case ARGOS_MOD_VLDA4: return KineisModulation::VLDA4;
		case ARGOS_MOD_LDA2:
		default:              return KineisModulation::LDA2;
	}
}

// ============================================================================
// Constructor / Destructor
// ============================================================================

SmdSat::SmdSat(SmdSatCmd& cmd, unsigned int idle_shutdown_ms)
	: m_cmd(cmd)
{
	DEBUG_TRACE("SmdSat::%s",__func__);
	set_idle_timeout(idle_shutdown_ms);
	m_packet_buffer.clear();
	// Load cached modulation from config — set by the credentials-dirty path
	// after the last master RCONF write. Avoids defaulting to LDA2 every boot
	// (which would silently desync m_modulation from the actual programmed
	// RCONF when the user's master encodes LDK or VLDA4). Default 0 = LDA2.
	m_modulation = ARGOS_MOD_LDA2;
	if (configuration_store) {
		unsigned int cached = configuration_store->read_param<unsigned int>(ParamID::ARGOS_CACHED_MODULATION);
		switch (cached) {
			case 0:  m_modulation = ARGOS_MOD_LDA2;  break;
			case 1:  m_modulation = ARGOS_MOD_LDK;   break;
			case 2:  m_modulation = ARGOS_MOD_VLDA4; break;
			default: m_modulation = ARGOS_MOD_LDA2;  break;
		}
		DEBUG_INFO("SmdSat::ctor: loaded cached modulation=%u (-> enum=%d)",
		           cached, static_cast<int>(m_modulation));
	}
	m_state = SmdSatState::stopped;
	m_stopping = false;
	m_state_counter = 0;
	m_next_delay = 0;
	m_tcxo_warmup_time = DEFAULT_TCXO_WARMUP_TIME_SECONDS;
	m_lpm_mode = 0x01;  // NONE by default
	m_tx_power = 0;
	m_tx_freq = 0.0;
	m_is_first_tx = true;
	this->shutdown();
	is_kmac_profil_loaded = false;
}

SmdSat::~SmdSat() {
	power_off_immediate();
}

// ============================================================================
// Power management
// ============================================================================

void SmdSat::shutdown(void) {
	GPIOPins::init_pin(SAT_RESET);
	GPIOPins::clear(SAT_RESET);  // Hold RESET LOW
	nrf_delay_ms(SMDSAT_DELAY_RST_MS);
	GPIOPins::clear(SAT_PWR_EN);  // Cut power
#ifdef SMD_VPA_PIN
	GPIOPins::drive_low(SMD_VPA_PIN);
#endif
	// Wait for VDD caps to discharge — ensures a true POR on next power_on.
	// Without this, the STM32WL stays alive on residual VDD (~10-50ms) and
	// resumes from its previous SPI sequence number, causing INVALID_CMD
	// when the nRF restarts with seq=0 after a soft reset.
	nrf_delay_ms(smdsat_vdd_discharge_ms());
	// NOTE: this is a static method — cannot stamp m_last_power_off_ms here.
	// The instance-level timestamp is set in power_off_immediate() instead.
}

void SmdSat::power_on_blocking() {
	DEBUG_INFO("SmdSat::%s: synchronous boot sequence", __func__);

	GPIOPins::acquire_sensors_pwr();

	// F-SAT-7 audit fix: honor m_last_power_off_ms like state_powering_on does.
	// Without this, a DTE-initiated path (SATVF / dfu_enter / cw_start /
	// get_firmware_version / smd_spi_test) called shortly after a soft reset
	// hits the same INVALID_CMD cascade that CLAUDE.md §6.2 warns about —
	// STM32WL holds residual state for up to ~50 ms after the previous reset.
	// Run the discharge wait if last power-off was too recent. Same logic
	// as state_powering_on, kept inline since power_on_blocking is a static
	// method indirectly invoked.
	{
		uint64_t now = PMU::get_timestamp_ms();
		bool vdd_already_decayed = (m_last_power_off_ms != 0) &&
		                            ((now - m_last_power_off_ms) >= VDD_NATURAL_DISCHARGE_MS);
		if (!vdd_already_decayed) {
			GPIOPins::init_pin(SAT_RESET);
			GPIOPins::clear(SAT_RESET);                          // hold reset during discharge
			GPIOPins::clear(SAT_PWR_EN);                         // make sure rail is down
#ifdef SMD_VPA_PIN
			GPIOPins::drive_low(SMD_VPA_PIN);
#endif
			nrf_delay_ms(smdsat_vdd_discharge_ms());             // FAST=50ms / SAFE=100ms
		}
	}

	// Hold RESET LOW during power ramp
	GPIOPins::init_pin(SAT_RESET);
	GPIOPins::clear(SAT_RESET);

#ifdef SAT_EXTWAKEUP
	GPIOPins::init_pin(SAT_EXTWAKEUP);
	GPIOPins::set(SAT_EXTWAKEUP);
#endif

	// Power ON — VPA stays LOW (driven by nRF) to prevent PA regulator spike
	GPIOPins::set(SAT_PWR_EN);
	PMU::kick_watchdog();
	nrf_delay_ms(smdsat_delay_power_on_ms());

	// Release RESET — STM32WL boots
	GPIOPins::release_to_highz(SAT_RESET);
	PMU::kick_watchdog();
	nrf_delay_ms(smdsat_delay_power_on_ms());

	// Init SPI AFTER boot (avoid MISO backfeed during power ramp)
	m_cmd.init();

	// Wait for STM32WL to respond
	bool ready = false;
	for (uint8_t attempt = 0; attempt < 10; attempt++) {
		PMU::kick_watchdog();
		nrf_delay_ms(smdsat_delay_power_on_ms() / 2);
		if (m_cmd.ping()) {
			ready = true;
			break;
		}
	}

	// VPA stays LOW until TX — released in state_transmit_pending, driven LOW again after TX
	if (!ready) {
		DEBUG_ERROR("SmdSat::%s: SMD did not respond after boot", __func__);
	}
}

void SmdSat::power_off() {
	DEBUG_TRACE("SmdSat::%s",__func__);
	if (!SMD_STATE_EQUAL(stopped)) {
		m_stopping = true;
	}
}

void SmdSat::power_on() {
	DEBUG_TRACE("SmdSat::%s",__func__);

#if SMDSAT_AUTOFALLBACK_ENABLED
	// First power_on after boot: restore persisted SAFE/FAST choice from
	// config_store so a watchdog reset in degraded mode keeps the SAFE
	// timings on the next session.
	degraded_mode_load_if_needed();
#endif

	if (m_state != SmdSat::stopped) {
		m_stopping = false;
		TXTRACE("power_on(): already running (state=%d) — TX rides existing session",
		        static_cast<int>(m_state));
		return;
	}

	m_stopping = false;
	TXTRACE("power_on(): cold start, entering state machine");
	SMD_STATE_CHANGE(stopped, starting);
	state_machine();
}


void SmdSat::power_off_immediate()
{
	DEBUG_TRACE("SmdSat::%s",__func__);

	if (!SMD_STATE_EQUAL(stopped)) {
#if SMDSAT_AUTOFALLBACK_ENABLED
		// Detect TX cascade pattern BEFORE we overwrite m_state. If state was
		// transmitting and we burned through >25% of the polling budget without
		// success, the dive (or other external cancellation) just masked an
		// SPI-cascade failure that would have hit state_error timeout — count
		// it toward the autofallback so flapping devices don't slip through.
		degraded_mode_note_tx_cancelled_during_cascade();
#endif
		system_scheduler->cancel_task(m_task);
		switch (m_state) {
		case SmdSatState::starting:         state_starting_exit(); break;
		case SmdSatState::powering_on:      state_powering_on_exit(); break;
		case SmdSatState::load_kmac:        state_load_kmac_exit(); break;
		case SmdSatState::idle_pending:     state_idle_pending_exit(); break;
		case SmdSatState::idle:             state_idle_exit(); break;
		case SmdSatState::transmit_pending: state_transmit_pending_exit(); break;
		case SmdSatState::transmitting:     state_transmitting_exit(); break;
		case SmdSatState::error:            state_error_exit(); break;
		case SmdSatState::stopped:          break;
		default: break;
		}
		m_state = SmdSatState::stopped;
		state_stopped_enter();
	}

	// Clean up transport in case state was already stopped
	m_cmd.deinit();
}

// ============================================================================
// State machine
// ============================================================================

void SmdSat::state_machine(bool use_scheduler) {
	m_next_delay = 0;

	switch (m_state) {
	case SmdSat::starting:
		SMD_STATE_CALL(starting);
		break;
	case SmdSat::powering_on:
		SMD_STATE_CALL(powering_on);
		break;
	case SmdSat::load_kmac:
		SMD_STATE_CALL(load_kmac);
		break;
	case SmdSat::idle_pending:
		SMD_STATE_CALL(idle_pending);
		break;
	case SmdSat::idle:
		SMD_STATE_CALL(idle);
		break;
	case SmdSat::transmit_pending:
		SMD_STATE_CALL(transmit_pending);
		break;
	case SmdSat::transmitting:
		SMD_STATE_CALL(transmitting);
		break;
	case SmdSat::error:
		SMD_STATE_CALL(error);
		break;
	case SmdSat::stopped:
		SMD_STATE_CALL(stopped);
		break;
	default:
		break;
	}

	if (use_scheduler && !SMD_STATE_EQUAL(stopped)) {
		if (m_next_delay > 100) {
			TXTRACE("state_machine: rescheduling tick in %u ms (state=%d)",
			        m_next_delay, static_cast<int>(m_state));
		}
		system_scheduler->cancel_task(m_task);
		m_task = system_scheduler->post_task_prio([this]() {
			state_machine();
		}, "SmdReceiverStateMachine", SMD_SCHEDULER_PRIORITY, m_next_delay);
	}
}

void SmdSat::state_starting_enter() {}
void SmdSat::state_starting_exit() {}

void SmdSat::state_starting()
{
	m_is_first_tx = true;
	is_kmac_profil_loaded = false;  // Force entry to state_load_kmac for boot config (TCXO, LPM)
	// m_needs_explicit_kmac_load is NOT reset here — it persists across power cycles.
	// Only set true when RCONF/credentials actually change (steps 1a/1b in load_kmac).
	// When false, STM32 auto-initializes MAC from flash at POR — no SPI command needed.
	m_credentials_written = false;
	m_rconf_recovery_attempted = false;
	// SPI init is deferred to state_idle_pending (after power-on + reset release).
	// Initializing SPI here would configure MISO as input without pulldown while
	// the STM32WL is booting — risk of MISO backfeed via nRF ESD diode.
	m_state_counter = 3;
	m_next_delay = 0;
	SMD_STATE_CHANGE(starting, powering_on);
}

void SmdSat::state_error_enter() {
	// Force KMAC reload on next boot — SMD may be in inconsistent state.
	// Do NOT attempt SPI commands here — the bus is likely desynchronized
	// (INVALID_CMD cascade). Any command would fail and waste time.
	is_kmac_profil_loaded = false;
	m_error_count++;

	if (m_error_count >= SMD_MAX_CONSECUTIVE_ERRORS) {
		// Too many consecutive errors — enter 30 min cooldown to stop SPI spam
		// and save battery.  SMD will be retried after cooldown expires.
		m_cooldown_until = PMU::get_timestamp_ms() + SMD_ERROR_COOLDOWN_MS;
		DEBUG_ERROR("SmdSat: %u consecutive errors — cooldown for %u min",
			m_error_count, SMD_ERROR_COOLDOWN_MS / 60000);
		VAL_SAT("cooldown_engaged reason=state_error duration_min=%u",
		        SMD_ERROR_COOLDOWN_MS / 60000);
		m_error_count = 0;  // Reset for next attempt after cooldown
#if SMDSAT_AUTOFALLBACK_ENABLED
		// Autofallback: engage SAFE timings (or double the trust window if
		// we were already in SAFE and a FAST retest just failed).
		degraded_mode_engage();
#endif
	}

	notify(KineisEventDeviceError({}));
	SMD_STATE_CHANGE(error, stopped);
}

void SmdSat::state_error_exit() {}

/// @brief Defensive no-op: in normal flow this body never executes because
/// state_error_enter() always calls SMD_STATE_CHANGE(error, stopped) before
/// returning, so m_state is already 'stopped' by the time state_machine()
/// dispatches its next tick. Kept as a safety net: if state_error_enter()
/// is ever modified to return without transitioning (e.g. mid-refactor that
/// adds an async path), we still drop to 'stopped' here instead of looping
/// in error forever. The WARN log will surface the bug in field traces.
void SmdSat::state_error() {
	DEBUG_WARN("SmdSat::state_error: body reached — state_error_enter did not transit (regression?)");
	SMD_STATE_CHANGE(error, stopped);
}

#if SMDSAT_AUTOFALLBACK_ENABLED
/// @brief Load persisted SMD_DEGRADED_MODE flag on first SmdSat tick.
/// Reads ParamID::SMD_DEGRADED_MODE so a watchdog or power-cycle that
/// occurred while in SAFE mode keeps the SAFE timings on the next boot.
///
/// KNOWN LIMITATION (audit 2026-05 F1): only the SAFE/FAST flag is persisted —
/// m_safe_trust_window_hours always resets to 1 on boot. Consequence on a
/// sealed 1-year deployment: every WDT/POR reset while in SAFE mode triggers
/// a FAST retest 1 h post-boot regardless of where the doubling chain had
/// reached before (could have been 8 h, 16 h, 24 h). Cost ≈ 3 wasted TXes per
/// failed FAST retest. If reboots are rare (annual deployment, well-tested
/// firmware) the impact is bounded; if reboots become frequent the FAST/SAFE
/// chain effectively collapses to "always retest at 1 h", which is the
/// same behavior as a fresh device entering SAFE for the first time.
/// Fix path if needed: encode the window in SMD_DEGRADED_MODE itself
/// (0=FAST, N>0=SAFE/N-hours) and persist on every doubling.
///
/// KNOWN LIMITATION (audit 2026-05 F2): m_safe_mode_since_ms is reset to
/// PMU::get_timestamp_ms() at load time, i.e. boot timestamp — not the real
/// UTC moment when SAFE was engaged. degraded_mode_note_success() therefore
/// measures "hours in SAFE since this boot" rather than "hours since SAFE
/// was actually engaged". A boot that happens 23 h into SAFE waits a full
/// new trust window (24 h potentially) instead of retesting promptly.
/// Fix path if needed: persist RTC UTC timestamp at engage time, compare
/// against RTC on load.
void SmdSat::degraded_mode_load_if_needed() {
	if (m_degraded_mode_loaded) return;
	m_degraded_mode_loaded = true;
	unsigned int persisted = configuration_store->read_param<unsigned int>(ParamID::SMD_DEGRADED_MODE);
	if (persisted != 0) {
		g_smdsat_use_safe_timings = true;
		m_safe_mode_since_ms = PMU::get_timestamp_ms();
		m_safe_mode_tx_count = 0;
		DEBUG_INFO("SmdSat: degraded mode restored from flash — SAFE timings active");
		VAL_SAT("degraded_mode_load persisted=1 -> SAFE timings active");
	} else {
		VAL_SAT("degraded_mode_load persisted=0 -> FAST timings active");
	}
}

/// @brief Flip to SAFE timings on consecutive-error threshold.
/// First engagement: persist the flag so a reboot keeps us safe.
/// Re-entry while already in SAFE: a FAST retest just failed, so double the
/// trust window (cap 24h) before the next retest attempt.
void SmdSat::degraded_mode_engage() {
	if (!g_smdsat_use_safe_timings) {
		g_smdsat_use_safe_timings = true;
		m_safe_mode_since_ms = PMU::get_timestamp_ms();
		m_safe_mode_tx_count = 0;
		m_safe_trust_window_hours = 1;
		configuration_store->write_param(ParamID::SMD_DEGRADED_MODE, 1U);
		DEBUG_ERROR("SmdSat: degraded mode ENGAGED — SAFE timings, retest in %u h",
		            m_safe_trust_window_hours);
		VAL_SAT("degraded_mode_engage trust_window_h=%u (first engage)",
		        m_safe_trust_window_hours);
	} else {
		// We were already in SAFE and just hit the threshold again — the
		// previous FAST retest failed, double the window.
		if (m_safe_trust_window_hours < SAFE_TRUST_WINDOW_MAX_H) {
			m_safe_trust_window_hours = m_safe_trust_window_hours * 2;
			if (m_safe_trust_window_hours > SAFE_TRUST_WINDOW_MAX_H)
				m_safe_trust_window_hours = SAFE_TRUST_WINDOW_MAX_H;
		}
		m_safe_mode_since_ms = PMU::get_timestamp_ms();
		m_safe_mode_tx_count = 0;
		DEBUG_WARN("SmdSat: FAST retest failed — next retest in %u h",
		           m_safe_trust_window_hours);
		VAL_SAT("degraded_mode_engage trust_window_h=%u (retest_failed)",
		        m_safe_trust_window_hours);
	}
}

/// @brief Count successful TXes while in SAFE; retest FAST once the trust
/// window has elapsed AND enough TXes have succeeded.
void SmdSat::degraded_mode_note_success() {
	if (!g_smdsat_use_safe_timings) return;
	m_safe_mode_tx_count++;
	uint64_t hours_in_safe = (PMU::get_timestamp_ms() - m_safe_mode_since_ms) / (3600ULL * 1000ULL);
	if (m_safe_mode_tx_count >= SAFE_RETEST_MIN_TX &&
	    hours_in_safe >= m_safe_trust_window_hours) {
		g_smdsat_use_safe_timings = false;
		configuration_store->write_param(ParamID::SMD_DEGRADED_MODE, 0U);
		DEBUG_INFO("SmdSat: retesting FAST timings (was SAFE %lu h, %u successful TX)",
		           static_cast<unsigned long>(hours_in_safe), m_safe_mode_tx_count);
		VAL_SAT("retest_fast hours_in_safe=%lu tx_count=%u",
		        static_cast<unsigned long>(hours_in_safe), m_safe_mode_tx_count);
	}
}

/// @brief Cascade-failure detector: TX cancelled externally (dive event or
/// service_cancel) while state_transmitting had consumed a significant
/// fraction of its polling budget. Treats it as 1 consecutive error so the
/// autofallback can trip without waiting for the natural ~13-15s timeout —
/// matters because dive/surface flapping pre-empts that timeout in practice.
/// Idempotent: marks the snapshot consumed so a repeated stop_send/power_off
/// in the same TX cycle doesn't double-count.
void SmdSat::degraded_mode_note_tx_cancelled_during_cascade() {
	if (m_state != transmitting || m_initial_tx_state_counter == 0) return;
	unsigned int consumed = m_initial_tx_state_counter - m_state_counter;
	bool cascade = (consumed * TX_CASCADE_FAILURE_DENOMINATOR) >=
	               (m_initial_tx_state_counter * TX_CASCADE_FAILURE_NUMERATOR);
	m_initial_tx_state_counter = 0;  // Idempotent — consumed.
	if (!cascade) return;
	m_error_count++;
	DEBUG_WARN("SmdSat: TX cancelled after %u/%u failed polls — counting toward autofallback (m_error_count=%u/%u)",
	           consumed, consumed + m_state_counter, m_error_count, SMD_MAX_CONSECUTIVE_ERRORS);
	if (m_error_count >= SMD_MAX_CONSECUTIVE_ERRORS) {
		m_cooldown_until = PMU::get_timestamp_ms() + SMD_ERROR_COOLDOWN_MS;
		DEBUG_ERROR("SmdSat: %u consecutive cascade errors — cooldown for %u min",
		            m_error_count, SMD_ERROR_COOLDOWN_MS / 60000);
		m_error_count = 0;
		degraded_mode_engage();
	}
}
#endif  // SMDSAT_AUTOFALLBACK_ENABLED

void SmdSat::state_stopped_enter() {
	m_cmd.deinit();

	// Release wakeup pin LOW so SMD can enter deep sleep (STANDBY/SHUTDOWN)
	// before power is cut. This allows the STM32 to enter LPM immediately.
#ifdef SAT_EXTWAKEUP
	GPIOPins::clear(SAT_EXTWAKEUP);
#endif

	this->shutdown();
	GPIOPins::release_sensors_pwr();
	nrf_gpio_cfg_default(BSP::GPIO_Inits[SAT_RESET].pin_number);

	// Stamp power-off timestamp so the next state_powering_on can skip the
	// redundant VDD discharge wait if the dive was long enough for natural
	// decay. Saves 50 ms (FAST) / 100 ms (SAFE) on the first Doppler at
	// surface for the sealed turtle case (dive duration minutes).
	m_last_power_off_ms = PMU::get_timestamp_ms();

	// Force entry to state_load_kmac on next boot for TCXO/LPM writes (RAM registers).
	// The explicit load_kmac_profil SPI command is only sent when RCONF changed
	// (m_needs_explicit_kmac_load) — otherwise STM32 auto-inits MAC from flash at POR.
	is_kmac_profil_loaded = false;

	m_packet_buffer.clear();

	notify(KineisEventPowerOff({}));
}

void SmdSat::state_stopped_exit() {}
void SmdSat::state_stopped() {}

void SmdSat::state_powering_on_enter() {}

void SmdSat::state_powering_on_exit() {
	m_next_delay = smdsat_spi_boot_delay_ms();
}

void SmdSat::state_powering_on() {
	TXTRACE("state_powering_on: starting power sequence (100ms discharge + %u ms VDD)",
	        smdsat_delay_power_on_ms());

	GPIOPins::acquire_sensors_pwr();

	// Step 0: Force a clean power cycle — kill any residual power from previous session.
	// After a nRF soft reset, SAT_PWR_EN may have been HIGH and the STM32WL
	// is still alive on VDD decoupling caps with a stale SPI sequence counter.
	// Without this, the first 2-phase write fails with INVALID_CMD.
	GPIOPins::init_pin(SAT_RESET);
	GPIOPins::clear(SAT_RESET);
	GPIOPins::clear(SAT_PWR_EN);
#ifdef SMD_VPA_PIN
	GPIOPins::drive_low(SMD_VPA_PIN);
#endif
	// Skip the redundant VDD discharge wait if the previous power-off already
	// allowed enough time for natural decay. The sealed turtle case (dive
	// duration minutes) always satisfies this, saving 50 ms (FAST) / 100 ms
	// (SAFE) on the FIRST Doppler at surface — the most latency-sensitive TX
	// of the burst. m_last_power_off_ms==0 means we've never powered off
	// (first cold boot), so keep the discharge to be safe.
	uint64_t now = PMU::get_timestamp_ms();
	bool vdd_already_decayed = (m_last_power_off_ms != 0) &&
	                           ((now - m_last_power_off_ms) >= VDD_NATURAL_DISCHARGE_MS);
	if (!vdd_already_decayed) {
		nrf_delay_ms(smdsat_vdd_discharge_ms());  // VDD cap discharge — FAST=50ms / SAFE=100ms.
	} else {
		TXTRACE("state_powering_on: VDD discharge skipped (%llu ms since last off)",
		        now - m_last_power_off_ms);
	}
	// power_off_immediate() in the surfacing-burst flow already drives VDD low;
	// 50ms is enough for the LinkIt V4 cap to discharge before the next POR.

	// Step 1: Assert RESET LOW before powering on — hold STM32WL in reset
	// during VDD ramp-up to prevent undefined behavior at low voltage.
	GPIOPins::clear(SAT_RESET);

	// Step 2: Assert wakeup pin HIGH (LinkIt only) before power-on
#ifdef SAT_EXTWAKEUP
	GPIOPins::init_pin(SAT_EXTWAKEUP);
	GPIOPins::set(SAT_EXTWAKEUP);
#endif

	// Step 3: Power ON — VPA stays LOW (PA regulator not needed until TX)
	GPIOPins::set(SAT_PWR_EN);
#ifdef SMD_VPA_PIN
	GPIOPins::drive_low(SMD_VPA_PIN);
#endif
	DEBUG_TRACE("SmdSat::%s: SAT_PWR_EN=HIGH, RESET=LOW, VPA=LOW", __func__);

	// Step 4: Wait for VDD stabilization (150ms >> 5ms required)
	nrf_delay_ms(smdsat_delay_power_on_ms());

	// Step 5: Release RESET — pull-up on SMD VDD rail pulls HIGH → STM32WL boots
	GPIOPins::release_to_highz(SAT_RESET);
	DEBUG_TRACE("SmdSat::%s: RESET released, VPA held LOW until ping OK", __func__);

	TXTRACE("state_powering_on: RESET released, scheduling idle_pending in %u ms",
	        smdsat_spi_boot_delay_ms());
	SMD_STATE_CHANGE(powering_on, idle_pending);
}

void SmdSat::state_load_kmac_enter() {
	m_state_counter = 20;  // MAC poll iterations before timeout
	// Do NOT set m_next_delay here — the first call writes RCONF/credentials
	// (steps 1-2) and must execute promptly after ping OK.  The 500ms delay
	// is only appropriate for MAC polling retries (step 3) and is set there.
}
void SmdSat::state_load_kmac_exit() {}

void SmdSat::state_load_kmac() {
	TXTRACE("state_load_kmac: tick (pending_rconf=%u creds_written=%u explicit_kmac=%u poll=%u)",
	        !m_pending_rconf.empty(), m_credentials_written, m_needs_explicit_kmac_load, m_state_counter);

	// Sequence per firmware spec:
	// 1. Write RCONF/credentials if changed (flash operations — don't need MAC_OK)
	// 2. Load KMAC only if RCONF changed (otherwise STM32 auto-inits MAC from flash at POR)
	// 3. Poll READ_SPIMAC_STATE (0x27) until MAC_OK
	// 4. Write TCXO warmup + LPM (RAM registers, lost on every power cycle)

	// Step 1a: Apply deferred RCONF from switch_modulation() while stopped.
	if (!m_pending_rconf.empty()) {
		TXTRACE("state_load_kmac: applying deferred RCONF (set+save+wait ~120ms)");
		DEBUG_INFO("SmdSat::%s: applying deferred RCONF for modulation %d", __func__, static_cast<int>(m_modulation));
		auto wait_cmd = []() { nrf_delay_ms(SMDSAT_DELAY_CMD_MS * 2); };  // 60ms (v4.0.9 timing)
		std::string rconf_bin = Binascii::unhexlify(m_pending_rconf);
		smd_uint8_array_t rconf_struct = {static_cast<uint16_t>(rconf_bin.size()),
		                                  reinterpret_cast<uint8_t *>(rconf_bin.data())};
		// F-SAT-3 audit fix: track whether BOTH set + save succeeded. If save
		// fails, the RAM RCONF on STM32 is half-written (next power cycle reverts
		// to flash) — but we MUST retry. Previously we unconditionally cleared
		// m_pending_rconf in the catch path AND in the success path, so a
		// transient SPI error during save_radio_conf would silently leave the
		// device on the wrong modulation for the rest of the deployment
		// (m_modulation in RAM does not match flash).
		bool save_succeeded = false;
		try {
			m_cmd.set_radio_conf(&rconf_struct);
			wait_cmd();
			m_cmd.save_radio_conf();
			wait_cmd();
			save_succeeded = true;
			m_credentials_written = true;
			m_needs_explicit_kmac_load = true;  // RCONF changed — must reload MAC
			DEBUG_INFO("SmdSat::%s: deferred RCONF applied OK", __func__);
			VAL_SAT("deferred_rconf_applied modulation=%d", (int)m_modulation);
		} catch (...) {
			DEBUG_ERROR("SmdSat::%s: failed to apply deferred RCONF — retry on next session", __func__);
			VAL_SAT("deferred_rconf_throw modulation=%d (retry pending)", (int)m_modulation);
			// Also mark KMAC reload pending so next session re-fires the path
			m_needs_explicit_kmac_load = true;
		}
		if (save_succeeded) {
			m_pending_rconf.clear();
		}
		// else: keep m_pending_rconf so next state_load_kmac retries
	}

	// Step 1b: Write credentials if changed via DTE.
	if (!m_credentials_written) {
		m_credentials_written = true;
		if (configuration_store && configuration_store->is_credentials_dirty()) {
			TXTRACE("state_load_kmac: credentials dirty -> write_credentials_from_config (~480ms)");
			DEBUG_INFO("SmdSat::%s: credentials dirty, writing to SMD", __func__);
			if (!write_credentials_from_config()) {
				DEBUG_ERROR("SmdSat::%s: credential write failed", __func__);
				SMD_STATE_CHANGE(load_kmac, error);
				return;
			}
			configuration_store->clear_credentials_dirty();
			m_needs_explicit_kmac_load = true;  // Credentials changed — must reload MAC
		}
	}

	// Step 2: Send load_kmac_profil only when RCONF/credentials changed.
	// When unchanged, the STM32 auto-initializes MAC from flash at POR —
	// we just wait for MAC_OK in step 3.
#if SMD_UART
	// BLIND MAC profile (SMD-UART only). When ARGOS_BLIND_EN, push AT+KMAC=2
	// with the packed burst config every session (the ctx must reach the MAC —
	// don't rely on flash auto-init). Graceful: if the module rejects profile 2,
	// fall back to BASIC. When blind is off, keep the flash-auto-init optimization.
	// SMD-UART: push the KMAC profile EVERY session (blind or basic). A blind
	// toggle (ARGOS_BLIND_EN) is not signaled by the RCONF-dirty flag, so relying
	// on the STM32 flash auto-init would leave a disabled tag stuck in blind.
	unsigned int rn = 0, period = 0;
	bool blind_en = smd_blind_active(rn, period);
	{
		try {
			if (blind_en) {
				// KNS_MAC_BLIND_usrCfg_t packed, little-endian:
				// {int8 retx_nb, uint8 nb_parallel_msg=1, uint32 retx_period_s, int8 per_offset=0}
				uint8_t ctx[7] = {
					(uint8_t)rn, 1,
					(uint8_t)(period & 0xFF), (uint8_t)((period >> 8) & 0xFF),
					(uint8_t)((period >> 16) & 0xFF), (uint8_t)((period >> 24) & 0xFF),
					0 };
				try {
					m_cmd.load_kmac_profil(2, ctx, sizeof(ctx));
					DEBUG_INFO("SmdSat::%s: BLIND KMAC loaded (retx_nb=%u period=%us)", __func__, rn, period);
				} catch (...) {
					DEBUG_WARN("SmdSat::%s: BLIND KMAC (profile 2) rejected — falling back to BASIC", __func__);
					m_cmd.load_kmac_profil(1);
				}
			} else {
				TXTRACE("state_load_kmac: sending explicit load_kmac_profil(1)");
				m_cmd.load_kmac_profil(1);
			}
			m_needs_explicit_kmac_load = false;
		} catch (...) {
			DEBUG_ERROR("SmdSat::%s: KMAC load failed", __func__);
			SMD_STATE_CHANGE(load_kmac, error);
			return;
		}
	}
#else
	if (m_needs_explicit_kmac_load) {
		TXTRACE("state_load_kmac: sending explicit load_kmac_profil(1)");
		try {
			m_cmd.load_kmac_profil(1);
			m_needs_explicit_kmac_load = false;
			DEBUG_TRACE("SmdSat::%s: explicit KMAC load sent (RCONF changed)", __func__);
		} catch (...) {
			DEBUG_ERROR("SmdSat::%s: KMAC load failed", __func__);
			SMD_STATE_CHANGE(load_kmac, error);
			return;
		}
	} else {
		TXTRACE("state_load_kmac: skip explicit KMAC (STM32 auto-inits from flash)");
	}
#endif

	// Step 3: Poll READ_SPIMAC_STATE (0x27) for MAC ready.
	// After explicit load_kmac_profil: MAC resets to MAC_OK (0x01).
	// After STM32 auto-init from flash at POR: MAC reports last session status
	// (e.g. MAC_TX_DONE after a successful TX). Any state except UNKNOWN (not
	// initialized), ERROR (init failed), and TX_IN_PROGRESS (busy) means the
	// MAC has initialized from flash and is ready to accept TX commands.
	uint8_t spi_st = 0, mac_st = 0;
	try { m_cmd.read_spimac_state(&spi_st, &mac_st); } catch (...) {}
	bool mac_ready = (mac_st != MAC_UNKNOWN && mac_st != MAC_ERROR && mac_st != MAC_TX_IN_PROGRESS);
	if (mac_ready) {
		is_kmac_profil_loaded = true;
		TXTRACE("state_load_kmac: MAC READY (mac=%u spi=%u) -> writing TCXO+LPM, then idle", mac_st, spi_st);
		DEBUG_TRACE("SmdSat::%s: MAC ready (mac=%u, explicit_load=%u)", __func__, mac_st, m_needs_explicit_kmac_load);

		// Step 4: Write TCXO warmup — RAM register on STM32, lost on power cycle.
		// Force 0 on first TX after power-on: TCXO was just powered, no warmup needed.
		// Configured value is rewritten in state_transmitting_exit after the first TX.
		// Safety: if die temperature is below TCXO_SKIP_TEMP_THRESHOLD_C, fall back to
		// the configured warmup so a cold TCXO (deep dive → quick surface in cold
		// water) doesn't TX off-frequency. Affects only the first TX of the burst;
		// subsequent TXes already use the configured warmup.
		int die_temp_c = PMU::get_die_temperature_c();
		bool tcxo_skip = m_is_first_tx && (die_temp_c >= TCXO_SKIP_TEMP_THRESHOLD_C);
		uint32_t warmup_ms = tcxo_skip ? 0 : (m_tcxo_warmup_time * 1000);
		if (m_is_first_tx && !tcxo_skip) {
			DEBUG_INFO("SmdSat::%s: cold die temp (%d °C < %d) — keeping TCXO warmup",
			           __func__, die_temp_c, TCXO_SKIP_TEMP_THRESHOLD_C);
		}
		try {
			m_cmd.write_tcxo_warmup(warmup_ms);
			DEBUG_TRACE("SmdSat::%s: TCXO warmup written: %u ms (first_tx=%u)", __func__, warmup_ms, m_is_first_tx);
		} catch (...) {
			DEBUG_WARN("SmdSat::%s: failed to write TCXO warmup", __func__);
		}

		// Write LPM mode if not NONE — RAM register on STM32.
		if (m_lpm_mode != 0x01) {
			try {
				m_cmd.write_lpm(&m_lpm_mode);
				DEBUG_TRACE("SmdSat::%s: LPM mode written: 0x%02X", __func__, m_lpm_mode);
			} catch (...) {
				DEBUG_WARN("SmdSat::%s: failed to write LPM mode", __func__);
			}
		}

		// First-TX shortcut (2026-05 optim Cible #2): if a packet is already
		// queued (typical surfacing-burst path where send() set m_packet_buffer
		// before triggering power_on), skip state_idle entirely and go straight
		// to transmit_pending. Saves ~40-100 ms of scheduler latency (10 ms wait
		// after state_idle_enter + 30 ms wait after state_idle_exit + a scheduler
		// context switch). Falls back to the regular idle transition when no
		// packet is pending (e.g. DTE-initiated set_credentials boot).
		if (m_packet_buffer.length() > 0) {
			m_tx_buffer = m_packet_buffer;
			m_packet_buffer.clear();
			SMD_STATE_CHANGE(load_kmac, transmit_pending);
		} else {
			// Go directly to idle — SPI bus already confirmed working by
			// read_spimac_state + write_tcxo above. No need for a second ping.
			SMD_STATE_CHANGE(load_kmac, idle);
		}
	} else if (mac_st == MAC_ERROR && !m_rconf_recovery_attempted) {
		// Recovery: MAC_ERROR means RCONF in STM32 flash is corrupted or missing.
		// Force-write the master RCONF, save to flash, then re-attempt KMAC load.
		// NOTE: the "attempted" latch is set only AFTER a successful write (below),
		// not here — otherwise a throw mid-recovery would permanently disable the
		// retry and leave the SMD stuck in error→cooldown on a recoverable glitch.
		DEBUG_WARN("SmdSat::%s: MAC_ERROR — attempting RCONF recovery", __func__);
		if (configuration_store) {
			std::string radioconf = configuration_store->read_param<std::string>(ParamID::ARGOS_RADIOCONF);
			if (!radioconf.empty() && radioconf.size() >= 2 && (radioconf.size() % 2) == 0) {
				auto wait_cmd = []() { nrf_delay_ms(150); };
				std::string rconf_bin = Binascii::unhexlify(radioconf);
				smd_uint8_array_t rconf_struct = {static_cast<uint16_t>(rconf_bin.size()),
				                                  reinterpret_cast<uint8_t *>(rconf_bin.data())};
				try {
					m_cmd.set_radio_conf(&rconf_struct);
					wait_cmd();
					m_cmd.save_radio_conf();
					wait_cmd();
					m_needs_explicit_kmac_load = true;  // Force KMAC reload after recovery
					is_kmac_profil_loaded = false;
					m_state_counter = 10;  // Reset poll counter for MAC_OK after recovery
					m_rconf_recovery_attempted = true;  // mark done only on success
					DEBUG_INFO("SmdSat::%s: RCONF recovery written — retrying KMAC", __func__);
					m_next_delay = SMDSAT_DELAY_STATE_TICK_MS;
					return;  // Re-enter state_load_kmac on next tick
				} catch (...) {
					DEBUG_ERROR("SmdSat::%s: RCONF recovery write failed", __func__);
				}
			}
		}
		SMD_STATE_CHANGE(load_kmac, error);
	} else {
		if (--m_state_counter == 0) {
			DEBUG_ERROR("SmdSat::%s: MAC not OK after KMAC load (spi=%u mac=%u)", __func__, spi_st, mac_st);
			// Auto-init failed — force explicit KMAC load on next boot to recover.
			// Without this, m_needs_explicit_kmac_load stays false and we'd loop
			// through the same timeout on every subsequent boot attempt.
			m_needs_explicit_kmac_load = true;
			SMD_STATE_CHANGE(load_kmac, error);
		} else {
			TXTRACE("state_load_kmac: MAC NOT ready (mac=%u spi=%u) retry in %u ms (left=%u)",
			        mac_st, spi_st, smdsat_delay_load_kmac_ms(), m_state_counter);
			m_next_delay = smdsat_delay_load_kmac_ms();
		}
	}
}

void SmdSat::state_idle_pending_enter() {
	TXTRACE("state_idle_pending_enter: init SPI, will ping STM32");
	// Init SPI here (after power-on + reset release) to avoid MISO backfeed.
	// STM32WL has booted and its GPIOs are in a defined state now.
	m_cmd.init();
	m_next_delay = SMDSAT_DELAY_STATE_TICK_MS;
	m_state_counter = 10;
}

void SmdSat::state_idle_pending_exit() {
	m_next_delay = SMDSAT_DELAY_STATE_TICK_MS;
}

void SmdSat::state_idle_pending() {
	TXTRACE("state_idle_pending: tick entered, calling ping() now");
	// Capture t0 AFTER the log above so the LFS commit time isn't counted in ping_ms.
	[[maybe_unused]] uint64_t ping_t0 = PMU::get_timestamp_ms();
	bool ping_ok = m_cmd.ping();
	[[maybe_unused]] unsigned int ping_ms = static_cast<unsigned>(PMU::get_timestamp_ms() - ping_t0);
	TXTRACE("state_idle_pending: ping() returned %s after %u ms",
	        ping_ok ? "OK" : "FAIL", ping_ms);
	if (ping_ok) {
		TXTRACE("state_idle_pending: ping OK (retries_left=%u) -> %s",
		        m_state_counter, is_kmac_profil_loaded ? "idle" : "load_kmac");
		// Small delay after ping ACK for STM32 DMA re-arm.
		// MAC readiness is verified in state_load_kmac (poll MAC_OK).
		nrf_delay_ms(10);
		// VPA stays LOW — only released just before TX (state_transmit_pending)
		if (!is_kmac_profil_loaded) {
			SMD_STATE_CHANGE(idle_pending, load_kmac);
		} else {
			SMD_STATE_CHANGE(idle_pending, idle);
		}
		return;
	}
	if (--m_state_counter == 0) {
		DEBUG_ERROR("SmdSat::%s: failed to enter IDLE state",__func__);
		SMD_STATE_CHANGE(idle_pending, error);
	} else {
		TXTRACE("state_idle_pending: ping FAIL, retry in %u ms (counter=%u)",
		        SMDSAT_DELAY_STATE_TICK_MS, m_state_counter);
		m_next_delay = SMDSAT_DELAY_STATE_TICK_MS;
	}
}

void SmdSat::state_idle_enter() {
	m_next_delay = SMDSAT_DELAY_TICK_INTERRUPT_MS;
	// Clamp ≥1: integer division of m_idle_timeout_ms / TICK_MS gives 0 when
	// idle_timeout < TICK_MS (10 ms). 0 then underflows on the first
	// --m_state_counter to UINT_MAX → idle effectively never times out, so
	// state_idle keeps re-scheduling until something external (packet arrival
	// or m_stopping) breaks the loop. Real callers pass 1 s default or
	// 35 s burst, so this is defense-in-depth against a future caller
	// passing a small or zero value.
	m_state_counter = m_idle_timeout_ms / SMDSAT_DELAY_TICK_INTERRUPT_MS;
	if (m_state_counter == 0) m_state_counter = 1;

	// If LPM allows STANDBY/SHUTDOWN, release wakeup pin LOW so SMD can
	// enter deep sleep while waiting for next TX in this session.
	//
	// IMPORTANT: in the normal send() path, m_packet_buffer is filled BEFORE
	// power_on() is called, so state_idle is transient (1 tick) — we go
	// straight to transmit_pending. Dropping WKUP here would let the STM32
	// start entering STANDBY, and raising it 10 ms later triggers a POR wake
	// (reset + firmware re-init ~100-200 ms) while we send SPI TX REQ
	// immediately → TX REQ fails. So only drop WKUP if the buffer is empty,
	// i.e. a real idle period (warm_up_for_tx-style pre-boot).
#ifdef SAT_EXTWAKEUP
	m_wkup_lowered = false;
	if ((m_lpm_mode & 0x18) && !m_packet_buffer.length()) {
		GPIOPins::clear(SAT_EXTWAKEUP);
		m_wkup_lowered = true;
		DEBUG_TRACE("SmdSat::%s: WKUP LOW (LPM=0x%02X, idle lingering)", __func__, m_lpm_mode);
	}
#endif
}

void SmdSat::state_idle_exit() {
	// Only raise WKUP + wait if we actually lowered it in state_idle_enter.
	// Avoids a pointless 50 ms delay (and spurious wake-up POR) in the
	// common transient-idle case where WKUP was never dropped.
#ifdef SAT_EXTWAKEUP
	if (m_wkup_lowered) {
		GPIOPins::set(SAT_EXTWAKEUP);
		nrf_delay_ms(50);  // STM32 wakeup from STANDBY/SHUTDOWN takes ~50ms (STANDBY).
		                   // SHUTDOWN needs more — user should not enable 0x10 without
		                   // also increasing this and re-running state_load_kmac (TCXO/LPM
		                   // are RAM registers lost on SHUTDOWN wake = full POR).
		m_wkup_lowered = false;
		DEBUG_TRACE("SmdSat::%s: WKUP HIGH (wakeup from LPM)", __func__);
	}
#endif
	m_next_delay = SMDSAT_DELAY_STATE_TICK_MS;
}

void SmdSat::state_idle() {
	if (m_packet_buffer.length()) {
		TXTRACE("state_idle: packet pending -> transmit_pending");
		m_tx_buffer = m_packet_buffer;
		m_packet_buffer.clear();
		SMD_STATE_CHANGE(idle, transmit_pending);
	} else if (m_stopping) {
		SMD_STATE_CHANGE(idle, stopped);
	} else {
		m_next_delay = SMDSAT_DELAY_TICK_INTERRUPT_MS;
		if (--m_state_counter == 0) {
			DEBUG_TRACE("SmdSat::%s: idle timeout elapsed",__func__);
			SMD_STATE_CHANGE(idle, stopped);
		}
		return;
	}

	// Mirror the clamp from state_idle_enter — same UINT_MAX underflow risk
	// when transitioning back to idle from a TX with a small idle_timeout.
	m_state_counter = m_idle_timeout_ms / SMDSAT_DELAY_TICK_INTERRUPT_MS;
	if (m_state_counter == 0) m_state_counter = 1;
}

void SmdSat::state_transmit_pending_enter() {
	m_state_counter = 3;
}

void SmdSat::state_transmit_pending_exit() {
	// First TX after power-on: warmup forced to 0 ONLY if the die is warm enough.
	// Cold die (deep cold dive then quick surface) keeps the configured warmup
	// to avoid an off-frequency first TX. Must match the gate logic in
	// state_load_kmac so the SMD's actual warmup matches our poll horizon.
	bool tcxo_skip = m_is_first_tx && (PMU::get_die_temperature_c() >= TCXO_SKIP_TEMP_THRESHOLD_C);
	unsigned int effective_warmup = tcxo_skip ? 0 : m_tcxo_warmup_time;
	unsigned int base_delay = (effective_warmup > 0) ? SMDSAT_DELAY_CMD_TX : smdsat_first_tx_base_delay_ms();
	m_next_delay = base_delay + (effective_warmup * 1000);
	TXTRACE("state_transmit_pending_exit: scheduling 1st TX poll in %u ms (first_tx=%u warmup=%u s)",
	        m_next_delay, m_is_first_tx, effective_warmup);
}

// Keep every retransmission of one logical message on the same Argos MAC message
// counter (MC). The SMD auto-increments MC after each TX, so without this each
// repeat of the same payload would carry a different MC. Policy (ALL modes —
// LEGACY / pass-prediction / duty-cycle / doppler / surface-doppler — because
// this is the single choke point for every SMD TX):
//   - NEW message (payload differs from the previous TX): do NOT touch the MC.
//     No SPI op is issued, so the first-TX timing is preserved (critical for
//     surface-doppler). The message goes out with the SMD's natural MC.
//   - REPEAT (identical payload): decrement MC by 1 (mod 512) so this copy is
//     transmitted with the same counter as the previous identical copy. Every
//     repeat lands back on that first MC.
// Optional / graceful: if the module firmware lacks the MC command, the read or
// write returns false, we latch "unsupported" and leave the SMD to auto-manage
// its MC — nothing else changes.
void SmdSat::apply_message_counter_hold()
{
	if (m_mc_support == McSupport::NO)
		return;  // module lacks the MC command — leave the SMD's MC auto-managed

	const bool is_repeat = !m_mc_last_payload.empty() &&
	                       (m_tx_buffer == m_mc_last_payload);

	if (!is_repeat) {
		// New message — leave the MC untouched (no SPI op → first-TX timing
		// preserved). Just remember the payload to detect future repeats.
		m_mc_last_payload = m_tx_buffer;
		return;
	}

	// Repeat of the same message — decrement the MC by 1 (mod 512) so this copy
	// reuses the counter of the previous identical transmission.
	uint16_t cur = 0;
	if (!m_cmd.read_message_counter(&cur)) {
		DEBUG_INFO("SmdSat::%s: read MC unsupported — MC hold disabled (SMD auto-manages MC)", __func__);
		m_mc_support = McSupport::NO;
		return;
	}
	uint16_t dec = (cur == 0) ? 511 : static_cast<uint16_t>(cur - 1);
	if (m_cmd.set_message_counter(dec)) {
		m_mc_support = McSupport::YES;
		DEBUG_TRACE("SmdSat::%s: repeat — MC %u -> %u (held)", __func__, cur, dec);
	} else {
		DEBUG_INFO("SmdSat::%s: set MC unsupported — MC hold disabled", __func__);
		m_mc_support = McSupport::NO;
	}
}

void SmdSat::state_transmit_pending() {
	if (m_tx_buffer.size()) {
		TXTRACE("state_transmit_pending: calling initiate_tx (%u bytes)",
		        static_cast<unsigned>(m_tx_buffer.size()));
		// Release VPA just before TX — PA regulator needs to be enabled for RF output
#ifdef SMD_VPA_PIN
		GPIOPins::release_to_highz(SMD_VPA_PIN);
		DEBUG_TRACE("SmdSat::%s: VPA released for TX", __func__);
#endif
		// Keep every repeat of the same message on one Argos MC (RSPB).
		apply_message_counter_hold();
		if (!m_cmd.initiate_tx(m_tx_buffer)) {
			DEBUG_ERROR("SmdSat::%s: initiate_tx failed, aborting TX", __func__);
			m_tx_buffer.clear();
			SMD_STATE_CHANGE(transmit_pending, error);
			return;
		}
		TXTRACE("state_transmit_pending: initiate_tx OK, TX started on STM32");
		notify(KineisEventTxStarted({}));
		SMD_STATE_CHANGE(transmit_pending, transmitting);
	} else if (--m_state_counter == 0) {
		DEBUG_ERROR("SmdSat::%s: failed accept SEND command",__func__);
		SMD_STATE_CHANGE(transmit_pending, error);
	} else {
		m_next_delay = SMDSAT_DELAY_STATE_TICK_MS;
	}
}

void SmdSat::state_transmitting_enter() {
	DEBUG_TRACE("SmdSat::%s", __func__);
	uint32_t total_timeout_ms = (m_tcxo_warmup_time * 1000) + 5000;
#if SMD_UART
	// BLIND: the module transmits retx_nb copies retx_period_s apart before it
	// reports +TX — extend the poll window to cover the whole burst.
	unsigned int rn = 0, period = 0;
	if (smd_blind_active(rn, period)) {
		// uint64 intermediate: rn(<=127) * period(<=65535) * 1000 overflows uint32.
		// Cap at 2 h so an extreme config can't park the service indefinitely.
		uint64_t burst_ms = (uint64_t)rn * (uint64_t)period * 1000ULL;
		if (burst_ms > 7200000ULL) burst_ms = 7200000ULL;
		total_timeout_ms += (uint32_t)burst_ms;
		DEBUG_INFO("SmdSat::%s: BLIND — TX poll window +%u ms (burst)", __func__, (uint32_t)burst_ms);
	}
#endif
	m_state_counter = (total_timeout_ms / smdsat_timing_tx_poll_ms()) + 1;
#if SMDSAT_AUTOFALLBACK_ENABLED
	// Snapshot for cascade-failure detection in stop_send / power_off_immediate.
	m_initial_tx_state_counter = m_state_counter;
#endif
	DEBUG_TRACE("SmdSat::%s: poll timeout=%ums | counter=%u", __func__, total_timeout_ms, m_state_counter);
}

void SmdSat::state_transmitting_exit() {
	// First TX done: restore configured TCXO warmup on STM32 for subsequent TX
	// in THIS session (RAM register, lost on power cycle). This does NOT touch the
	// fast first-TX path (state_load_kmac forces warmup=0 when the TCXO is already
	// warm) — it only makes sure every *subsequent* in-session TX gets the real
	// warmup, so the carrier doesn't drift off-frequency and lose the Argos pass.
	//
	// The STM32WL is not ready for a fresh SPI command in the few ms right after
	// MAC_TX_DONE: the field log showed REQ 0x29 → "IDLE pattern, no response
	// ready" on all 3 instant auto-retries, leaving the warmup stuck at 0 for the
	// rest of the burst. Give it a settle window and a few spaced attempts so the
	// restore actually lands.
	if (m_is_first_tx && m_tcxo_warmup_time > 0) {
		static constexpr uint8_t  TCXO_RESTORE_ATTEMPTS  = 3;
		static constexpr uint32_t TCXO_RESTORE_SETTLE_MS = 50;  // STM32WL post-TX settle
		bool restored = false;
		for (uint8_t attempt = 0; attempt < TCXO_RESTORE_ATTEMPTS && !restored; attempt++) {
			nrf_delay_ms(TCXO_RESTORE_SETTLE_MS);
			try {
				m_cmd.write_tcxo_warmup(m_tcxo_warmup_time * 1000);
				DEBUG_TRACE("SmdSat::%s: restored TCXO warmup: %u s (attempt %u)",
				            __func__, m_tcxo_warmup_time, attempt + 1);
				restored = true;
			} catch (...) {
				DEBUG_WARN("SmdSat::%s: TCXO warmup restore attempt %u/%u failed",
				           __func__, attempt + 1, TCXO_RESTORE_ATTEMPTS);
			}
		}
	}
	// Clear unconditionally: the first TX of every session is re-armed by
	// state_starting() (m_is_first_tx=true on each power_on), so this never
	// affects the fast first-TX path — it only governs in-session subsequent TX.
	m_is_first_tx = false;
	// Drive VPA LOW after TX — PA regulator no longer needed
#ifdef SMD_VPA_PIN
	GPIOPins::drive_low(SMD_VPA_PIN);
	DEBUG_TRACE("SmdSat::%s: VPA driven LOW after TX", __func__);
#endif
}

void SmdSat::state_transmitting() {
	if (m_cmd.is_tx_finished()) {
		if (m_tx_buffer.size()) {
			TXTRACE("state_transmitting: TX FINISHED — total elapsed");
			m_tx_buffer.clear();
			// is_tx_finished() returns true for MAC_TX_DONE (real success),
			// MAC_TX_TIMEOUT (no satellite reception), AND MAC_ERROR (hard
			// failure). Only MAC_TX_DONE should fire KineisEventTxComplete —
			// otherwise AFTER_LAST_TX cooldown arms on a failed TX and locks
			// out the whole surface cycle even though no packet was sent.
			bool tx_success = m_cmd.is_tx_successful();
			if (!tx_success) {
				TXTRACE("state_transmitting: TX FAILED (timeout/error) — notifying DeviceError");
				notify(KineisEventDeviceError({}));
				SMD_STATE_CHANGE(transmitting, stopped);
				return;
			}
			m_error_count = 0;  // TX success — reset consecutive error counter
#if SMDSAT_AUTOFALLBACK_ENABLED
			// Autofallback: track successful TXes while in SAFE so the
			// trust window can close and we retest FAST.
			degraded_mode_note_success();
#endif
			DEBUG_TRACE("SmdSat::%s: notify KineisEventTxComplete", __func__);
			notify(KineisEventTxComplete({}));
		}
		SMD_STATE_CHANGE(transmitting, stopped);
	} else {
		if (!m_tx_buffer.size()) {
			SMD_STATE_CHANGE(transmitting, stopped);
		} else if (--m_state_counter == 0) {
			DEBUG_ERROR("SmdSat::%s: TX timeout after polling",__func__);
			SMD_STATE_CHANGE(transmitting, error);
		} else {
			TXTRACE("state_transmitting: not finished, poll again in %u ms (left=%u)",
			        smdsat_timing_tx_poll_ms(), m_state_counter);
			m_next_delay = smdsat_timing_tx_poll_ms();
		}
	}
}

// ============================================================================
// KineisDevice interface
// ============================================================================

void SmdSat::stop_send() {
	DEBUG_TRACE("SmdSat::%s",__func__);
#if SMDSAT_AUTOFALLBACK_ENABLED
	// Soft-cancel path (service_cancel timeout / similar) — apply the same
	// cascade detector as power_off_immediate so cancellations during an
	// ongoing SPI poll cascade still count toward the autofallback.
	degraded_mode_note_tx_cancelled_during_cascade();
#endif
	m_packet_buffer.clear();
	m_tx_buffer.clear();
}

void SmdSat::start_receive(const KineisModulation mode) {
	(void)mode;
	DEBUG_TRACE("SmdSat::%s: Not supported",__func__);
}

bool SmdSat::stop_receive() {
	DEBUG_TRACE("SmdSat::%s: Not supported",__func__);
	return false;
}

void SmdSat::set_frequency(double freq_mhz) {
	DEBUG_TRACE("SmdSat::%s",__func__);
	m_tx_freq = freq_mhz;
}

unsigned int SmdSat::cooldown_remaining_ms() const {
	if (m_cooldown_until == 0) return 0;
	uint64_t now = PMU::get_timestamp_ms();
	return (now < m_cooldown_until)
	       ? static_cast<unsigned int>(m_cooldown_until - now)
	       : 0;
}

void SmdSat::send(const KineisModulation mode, const KineisPacket& user_payload, const unsigned int payload_length)
{
	m_tx_trace_start_ms = PMU::get_timestamp_ms();
	TXTRACE("send() entered: mode=%d state=%d first_tx=%u tcxo_warmup=%u s",
	        static_cast<int>(mode), static_cast<int>(m_state), m_is_first_tx, m_tcxo_warmup_time);

#if SMDSAT_AUTOFALLBACK_ENABLED
	// Lazy sync of runtime flag from persisted param. Lets a DTE PARMW SMP00=0
	// (operator-triggered manual clear after a confirmed root-cause fix) take
	// effect at the next TX without requiring a reboot. The runtime flag
	// `g_smdsat_use_safe_timings` is otherwise only set at boot via
	// `degraded_mode_load_if_needed()` and by `degraded_mode_engage()`. The
	// reverse direction (param=1 while flag=false) cannot happen because DTE
	// PARMW rejects SMP00=1 (see dte_handler.cpp::PARMW_REQ); only
	// degraded_mode_engage() can set param=1, and it also sets the flag.
	if (g_smdsat_use_safe_timings && configuration_store &&
	    configuration_store->read_param<unsigned int>(ParamID::SMD_DEGRADED_MODE) == 0) {
		DEBUG_INFO("SmdSat::send: DTE-cleared degraded mode — re-engaging FAST timings");
		VAL_SAT("degraded_mode_cleared_via_dte -> FAST timings");
		g_smdsat_use_safe_timings = false;
		m_safe_mode_tx_count = 0;
		m_safe_trust_window_hours = 1;
		m_safe_mode_since_ms = 0;
	}
#endif

	// Reject operations during error cooldown — prevents SPI spam when SMD is unresponsive
	if (m_cooldown_until > 0 && PMU::get_timestamp_ms() < m_cooldown_until) {
		unsigned int remaining_ms = static_cast<unsigned int>(m_cooldown_until - PMU::get_timestamp_ms());
		DEBUG_WARN("SmdSat::%s: in cooldown (%u min remaining) — TX rejected",
			__func__, remaining_ms / 60000);
		VAL_SAT("send_rejected_cooldown remaining_ms=%u", remaining_ms);
		notify(KineisEventDeviceError({}));
		return;
	}
	if (m_cooldown_until > 0) {
		VAL_SAT("cooldown_expired_naturally");
	}
	m_cooldown_until = 0;  // Cooldown expired — allow operation

	DEBUG_TRACE("SmdSat::%s: length %u mode=%d current=%d", __func__, payload_length, static_cast<int>(mode), static_cast<int>(m_modulation));
	SmdArgosModulation requested = kineis_to_smd_mod(mode);
	if (requested != m_modulation) {
		DEBUG_WARN("SmdSat::%s: TX mode %d != current modulation %d — call switch_modulation() first",
		           __func__, static_cast<int>(requested), static_cast<int>(m_modulation));
	}

	unsigned int max_payload_size = 0;
	switch(m_modulation) {
		case ARGOS_MOD_LDA2:
			max_payload_size = ARGOS_TX_LDA2_PAYLOAD_BYTE_SIZE;
			break;
		case ARGOS_MOD_LDK:
			max_payload_size = ARGOS_TX_LDK_PAYLOAD_BYTE_SIZE;
			break;
		case ARGOS_MOD_VLDA4:
			max_payload_size = ARGOS_TX_VLDA4_PAYLOAD_BYTE_SIZE;
			break;
		default:
			DEBUG_TRACE("SmdSat::%s: Unknown modulation type %d", __func__, m_modulation);
			return;
	}

	unsigned int effective_payload_length = std::min(payload_length,
	                                                 static_cast<unsigned int>(user_payload.size()));
	if (effective_payload_length > max_payload_size) {
		DEBUG_ERROR("SmdSat::%s: Payload truncated from %u to %u bytes",
		            __func__, effective_payload_length, max_payload_size);
		effective_payload_length = max_payload_size;
	}

	unsigned int padded_size = max_payload_size;
	if (m_modulation == ARGOS_MOD_LDA2) {
		padded_size = ((effective_payload_length + ARGOS_TX_LDA2_SIZE_STEP - 1) /
		                ARGOS_TX_LDA2_SIZE_STEP) * ARGOS_TX_LDA2_SIZE_STEP;
		if (padded_size < ARGOS_TX_LDA2_MIN_BYTE_SIZE)
			padded_size = ARGOS_TX_LDA2_MIN_BYTE_SIZE;
		if (padded_size > max_payload_size)
			padded_size = max_payload_size;
		DEBUG_TRACE("SmdSat::%s: LDA2 padded %u -> %u bytes", __func__, effective_payload_length, padded_size);
	}

	m_packet_buffer.assign(padded_size, 0);
	std::copy(user_payload.begin(), user_payload.begin() + effective_payload_length, m_packet_buffer.begin());

	DEBUG_TRACE("SmdSat::%s: raw data[%u]=%s", __func__,
	            effective_payload_length,
	            Binascii::hexlify(m_packet_buffer).c_str());

	DEBUG_TRACE("SmdSat::%s::Packet size %u", __func__, static_cast<unsigned int>(m_packet_buffer.size()));

	TXTRACE("send() calling power_on() (payload built, %u bytes)",
	        static_cast<unsigned>(m_packet_buffer.size()));

	power_on();
}

void SmdSat::set_tcxo_warmup_time(unsigned int time_s) {
	m_tcxo_warmup_time = time_s;
}

void SmdSat::set_lpm_mode(uint8_t lpm_bitmap) {
	// Sanitize: STANDBY(0x08) and SHUTDOWN(0x10) require SAT_EXTWAKEUP pin
#ifndef SAT_EXTWAKEUP
	if (lpm_bitmap & 0x18) {
		DEBUG_WARN("SmdSat::%s: STANDBY/SHUTDOWN disabled (no SAT_WKUP pin), masking to 0x%02X",
		           __func__, lpm_bitmap & 0x07);
		lpm_bitmap &= 0x07;  // Keep only NONE|SLEEP|STOP
	}
#endif
	if (lpm_bitmap == 0) lpm_bitmap = 0x01;  // Fallback to NONE
	m_lpm_mode = lpm_bitmap;
}

void SmdSat::set_tx_power(unsigned int power) {
	m_tx_power = power;
}

// ============================================================================
// Auto-write credentials from config store at boot
// ============================================================================

bool SmdSat::write_credentials_from_config() {
	if (!configuration_store) return false;

	// Flash write operations need ~100ms for persistence.
	// Previous delay (SMDSAT_DELAY_CMD_MS*2 = 60ms) was insufficient.
	//
	// KNOWN LIMITATION (audit 2026-05 F5): no PMU::kick_watchdog() between the
	// per-cmd wait_cmd() calls. Total worst case here is ~7 SPI writes × ~150ms
	// = ~1.1 s blocking. The 15-min WDT budget swallows this comfortably, but
	// if a future SPI cmd grows internal retries (e.g. cascade backoff) the
	// margin shrinks. Add a kick mid-routine if you ever extend this sequence
	// past ~5 s of blocking work. NOT a current bug — flagged for awareness.
	auto wait_cmd = []() { nrf_delay_ms(120); };

	unsigned int dec_id = configuration_store->read_param<unsigned int>(ParamID::ARGOS_DECID);
	unsigned int address = configuration_store->read_param<unsigned int>(ParamID::ARGOS_HEXID);
	std::string seckey = configuration_store->read_param<std::string>(ParamID::ARGOS_SECKEY);
	std::string radioconf = configuration_store->read_param<std::string>(ParamID::ARGOS_RADIOCONF);

	// Skip if credentials are not configured yet
	if (seckey.empty() || radioconf.empty()) {
		DEBUG_WARN("SmdSat::%s: credentials not configured (seckey=%u rconf=%u) | skipping",
		           __func__, static_cast<unsigned>(seckey.size()), static_cast<unsigned>(radioconf.size()));
		return true;  // Not an error, just nothing to write
	}

	// Validate credential sizes before writing (seckey = 16 bytes = 32 hex chars)
	if (seckey.size() != 32) {
		DEBUG_ERROR("SmdSat::%s: invalid seckey length %u (expected 32 hex chars)", __func__, static_cast<unsigned>(seckey.size()));
		return false;
	}
	if (radioconf.size() < 2 || (radioconf.size() % 2) != 0) {
		DEBUG_ERROR("SmdSat::%s: invalid radioconf length %u (must be even hex string)", __func__, static_cast<unsigned>(radioconf.size()));
		return false;
	}

	DEBUG_INFO("SmdSat::%s: writing credentials from config store (id=%u addr=0x%08X)",
	           __func__, dec_id, address);

	// Set ID
	try { m_cmd.set_id(dec_id); } catch (...) {
		DEBUG_ERROR("SmdSat::%s: failed to set ID", __func__); return false;
	}
	wait_cmd();

	// Set Address
	{
		uint8_t address_data[4] = {
			static_cast<uint8_t>((address >> 24) & 0xFF),
			static_cast<uint8_t>((address >> 16) & 0xFF),
			static_cast<uint8_t>((address >> 8) & 0xFF),
			static_cast<uint8_t>(address & 0xFF)
		};
		smd_uint8_array_t address_val = {SMDSAT_CMD_WRITE_ADDR_LEN-1, address_data};
		try { m_cmd.set_address(&address_val); } catch (...) {
			DEBUG_ERROR("SmdSat::%s: failed to set address", __func__); return false;
		}
	}
	wait_cmd();

	// Set Security Key
	{
		std::string seckey_bin = Binascii::unhexlify(seckey);
		smd_uint8_array_t seckey_struct = {static_cast<uint16_t>(seckey_bin.size()),
		                                   reinterpret_cast<uint8_t *>(seckey_bin.data())};
		try { m_cmd.set_seckey(&seckey_struct); } catch (...) {
			DEBUG_ERROR("SmdSat::%s: failed to set seckey", __func__); return false;
		}
	}
	wait_cmd();

	// Set Radio Configuration.
	// When adaptive modulation is ON, per-modulation RCONFs are managed by
	// ensure_modulation()/switch_modulation(). However, the master RCONF must
	// still be written on first boot or when credentials are dirty (chiperase,
	// PARMW on ARGOS_RADIOCONF, SATVF with force) — otherwise the STM32 flash
	// may contain invalid RCONF data and KMAC init will fail with MAC_ERROR.
	// After the first successful write, ensure_modulation() takes over.
	if (!radioconf.empty()) {
		bool adaptive = false;
		if (configuration_store) {
			adaptive = configuration_store->read_param<bool>(ParamID::ARGOS_ADAPTIVE_MODULATION);
		}
		if (!adaptive || configuration_store->is_credentials_dirty()) {
			if (adaptive) {
				DEBUG_INFO("SmdSat::%s: adaptive ON but credentials dirty — writing master RCONF as baseline",
				           __func__);
			}
			std::string rconf_bin = Binascii::unhexlify(radioconf);
			smd_uint8_array_t rconf_struct = {static_cast<uint16_t>(rconf_bin.size()),
			                                  reinterpret_cast<uint8_t *>(rconf_bin.data())};
			try { m_cmd.set_radio_conf(&rconf_struct); } catch (...) {
				DEBUG_ERROR("SmdSat::%s: failed to set radio conf", __func__); return false;
			}
			wait_cmd();

			try {
				if (!m_cmd.save_radio_conf()) {
					DEBUG_ERROR("SmdSat::%s: failed to save RCONF", __func__); return false;
				}
			} catch (...) {
				DEBUG_ERROR("SmdSat::%s: save_radio_conf exception", __func__); return false;
			}
			wait_cmd();
		} else {
			DEBUG_INFO("SmdSat::%s: adaptive modulation ON — skipping master RCONF write", __func__);
		}
	}

	// If adaptive modulation is active and we just wrote the master RCONF as baseline,
	// re-apply the RCONF for the current modulation so the next TX uses the right config.
	{
		bool is_adaptive = false;
		if (configuration_store)
			is_adaptive = configuration_store->read_param<bool>(ParamID::ARGOS_ADAPTIVE_MODULATION);
		if (is_adaptive && m_modulation != ARGOS_MOD_LDA2) {
			std::string active_rconf;
			ArgosConfig ac;
			configuration_store->get_argos_configuration(ac);
			KineisModulation km = smd_to_kineis_mod(m_modulation);
			if (km == KineisModulation::VLDA4) active_rconf = ac.radioconf_vlda4;
			else if (km == KineisModulation::LDK) active_rconf = ac.radioconf_ldk;

			if (!active_rconf.empty() && active_rconf.size() == 32) {
				DEBUG_INFO("SmdSat::%s: restoring RCONF for active modulation %d", __func__, static_cast<int>(m_modulation));
				std::string rconf_bin = Binascii::unhexlify(active_rconf);
				smd_uint8_array_t rconf_struct = {static_cast<uint16_t>(rconf_bin.size()),
				                                  reinterpret_cast<uint8_t *>(rconf_bin.data())};
				try {
					m_cmd.set_radio_conf(&rconf_struct);
					wait_cmd();
					m_cmd.save_radio_conf();
					wait_cmd();
					m_cmd.load_kmac_profil(1);
					wait_cmd();
				} catch (...) {
					DEBUG_WARN("SmdSat::%s: failed to restore active RCONF", __func__);
				}
			}
		}
	}

	// Read back the actual modulation programmed in STM32WL flash and persist
	// it. The master RCONF (ARGOS_RADIOCONF) is encrypted hex — we can't decode
	// which modulation it encodes locally. After the credentials-dirty path
	// just wrote it, READ_RCONF gives us the truth. Cache to config_store so
	// subsequent boots load the correct modulation at SmdSat construction
	// without needing a runtime SPI readback on the surface-burst critical
	// path. Failure here is non-fatal — m_modulation keeps its prior value.
	try {
		SmdArgosModulation reported = m_modulation;
		m_cmd.read_radio_conf(&reported);
		if (reported != m_modulation) {
			DEBUG_INFO("SmdSat::%s: cached modulation update %d -> %d (master RCONF readback)",
			           __func__, static_cast<int>(m_modulation), static_cast<int>(reported));
			m_modulation = reported;
		}
		if (configuration_store) {
			configuration_store->write_param(ParamID::ARGOS_CACHED_MODULATION,
			                                 static_cast<unsigned int>(m_modulation));
		}
	} catch (...) {
		DEBUG_WARN("SmdSat::%s: read_radio_conf failed — cached modulation not refreshed", __func__);
	}

	DEBUG_INFO("SmdSat::%s: credentials written successfully", __func__);
	return true;
}

// ============================================================================
// Credentials (legacy API, kept for direct hardware access if needed)
// ============================================================================

void SmdSat::set_credentials(unsigned int dec_id, unsigned int address, const std::string& seckey, const std::string& radioconf) {
	DEBUG_TRACE("SmdSat::%s",__func__);
	bool was_stopped = (m_state == SmdSatState::stopped);

	auto wait_between_commands = []() -> bool {
		nrf_delay_ms(SMDSAT_DELAY_CMD_MS * 2);
		return true;
	};

	if (was_stopped) {
		power_on_blocking();
	}

	for (uint8_t retry = 0; retry < SMDSAT_SPI_MAX_RETRIES; retry++) {
		try {
			m_cmd.set_id(dec_id);
			break;
		} catch (...) {
			if (retry == SMDSAT_SPI_MAX_RETRIES - 1) {
				DEBUG_ERROR("SmdSat::%s: Failed to set ID", __func__);
				goto cleanup;
			}
			nrf_delay_ms(smdsat_spi_retry_delay_ms());
		}
	}

	if (!wait_between_commands()) goto cleanup;

	{
		uint8_t address_data[4] = {
			static_cast<uint8_t>((address >> 24) & 0xFF),
			static_cast<uint8_t>((address >> 16) & 0xFF),
			static_cast<uint8_t>((address >> 8) & 0xFF),
			static_cast<uint8_t>(address & 0xFF)
		};
		smd_uint8_array_t address_val = {SMDSAT_CMD_WRITE_ADDR_LEN-1, address_data};

		for (uint8_t retry = 0; retry < SMDSAT_SPI_MAX_RETRIES; retry++) {
			try {
				m_cmd.set_address(&address_val);
				break;
			} catch (...) {
				if (retry == SMDSAT_SPI_MAX_RETRIES - 1) {
					DEBUG_ERROR("SmdSat::%s: Failed to set address", __func__);
					goto cleanup;
				}
				nrf_delay_ms(smdsat_spi_retry_delay_ms());
			}
		}
	}

	if (!wait_between_commands()) goto cleanup;

	{
		std::string seckey_val = Binascii::unhexlify(seckey);
		smd_uint8_array_t seckey_struct = {static_cast<uint16_t>(seckey_val.size()),
		                               reinterpret_cast<uint8_t *>(seckey_val.data())};

		for (uint8_t retry = 0; retry < SMDSAT_SPI_MAX_RETRIES; retry++) {
			try {
				m_cmd.set_seckey(&seckey_struct);
				break;
			} catch (...) {
				if (retry == SMDSAT_SPI_MAX_RETRIES - 1) {
					DEBUG_ERROR("SmdSat::%s: Failed to set seckey", __func__);
					goto cleanup;
				}
				nrf_delay_ms(smdsat_spi_retry_delay_ms());
			}
		}
	}

	if (!wait_between_commands()) goto cleanup;

	{
		std::string radioconf_val = Binascii::unhexlify(radioconf);
		smd_uint8_array_t radioconf_struct = {static_cast<uint16_t>(radioconf_val.size()),
		                                  reinterpret_cast<uint8_t *>(radioconf_val.data())};

		for (uint8_t retry = 0; retry < SMDSAT_SPI_MAX_RETRIES; retry++) {
			try {
				m_cmd.set_radio_conf(&radioconf_struct);
				break;
			} catch (...) {
				if (retry == SMDSAT_SPI_MAX_RETRIES - 1) {
					DEBUG_ERROR("SmdSat::%s: Failed to set radio conf", __func__);
					goto cleanup;
				}
				nrf_delay_ms(smdsat_spi_retry_delay_ms());
			}
		}
	}

	wait_between_commands();

	for (uint8_t retry = 0; retry < SMDSAT_SPI_MAX_RETRIES; retry++) {
		try {
			if (m_cmd.save_radio_conf()) break;
			if (retry == SMDSAT_SPI_MAX_RETRIES - 1) goto cleanup;
		} catch (...) {
			if (retry == SMDSAT_SPI_MAX_RETRIES - 1) goto cleanup;
		}
		nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
	}

	wait_between_commands();

	for (uint8_t retry = 0; retry < SMDSAT_SPI_MAX_RETRIES; retry++) {
		try {
			m_cmd.load_kmac_profil(1);
			break;
		} catch (...) {
			if (retry == SMDSAT_SPI_MAX_RETRIES - 1) goto cleanup;
			nrf_delay_ms(smdsat_spi_retry_delay_ms());
		}
	}

	wait_between_commands();

	{
		uint8_t rconf_raw[SMDSAT_CMD_READ_RCONF_RAW_LEN] = {0};
		uint16_t rconf_raw_len = 0;
		try {
			m_cmd.read_rconf_raw(rconf_raw, &rconf_raw_len);
			char hex[SMDSAT_CMD_READ_RCONF_RAW_LEN * 2 + 1] = {0};
			for (uint16_t i = 0; i < rconf_raw_len && i < SMDSAT_CMD_READ_RCONF_RAW_LEN; i++) {
				uint8_t hi = rconf_raw[i] >> 4;
				uint8_t lo = rconf_raw[i] & 0x0F;
				hex[i * 2]     = hi < 10 ? ('0' + hi) : ('a' + hi - 10);
				hex[i * 2 + 1] = lo < 10 ? ('0' + lo) : ('a' + lo - 10);
			}
			DEBUG_INFO("SmdSat::%s: RCONF_RAW readback OK: %s", __func__, hex);
		} catch (...) {
			DEBUG_WARN("SmdSat::%s: Failed to read back raw radio config", __func__);
		}
	}

	DEBUG_INFO("SmdSat::%s: Credentials set successfully", __func__);

cleanup:
	if (was_stopped) {
		m_cmd.deinit();
		shutdown();
		GPIOPins::release_sensors_pwr();
		nrf_gpio_cfg_default(BSP::GPIO_Inits[SAT_RESET].pin_number);
	}
}

void SmdSat::read_credentials(unsigned int *dec_id, unsigned int *address, std::string *seckey, std::string *radioconf) {
	DEBUG_TRACE("SmdSat::%s",__func__);
	bool was_stopped = (m_state == SmdSatState::stopped);

	if (was_stopped) {
		power_on_blocking();
	}

	PMU::kick_watchdog();
	nrf_delay_ms(smdsat_delay_power_on_ms());

	// Write pending credentials to SMD before reading them back.
	// PARMW saves credentials to the nRF config store but does NOT push
	// them to the SMD. Without this, SATVF reads stale/empty flash.
	if (configuration_store && configuration_store->is_credentials_dirty()) {
		DEBUG_INFO("SmdSat::%s: credentials dirty — writing to SMD before verify", __func__);
		if (write_credentials_from_config()) {
			configuration_store->clear_credentials_dirty();
			// KMAC reload so the MAC picks up the new RCONF
			try {
				m_cmd.load_kmac_profil(1);
				nrf_delay_ms(smdsat_delay_load_kmac_ms());
			} catch (...) {
				DEBUG_WARN("SmdSat::%s: KMAC reload failed after credential write", __func__);
			}
		} else {
			DEBUG_ERROR("SmdSat::%s: credential write failed before verify", __func__);
		}
	}

	try {
		if (dec_id) {
			uint32_t dec_id_val = 0;
			m_cmd.read_id(&dec_id_val);
			*dec_id = static_cast<unsigned int>(dec_id_val);
			nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
		}

		if (address) {
			uint8_t address_data[SMDSAT_CMD_READ_ADDR_LEN] = {0};
			smd_uint8_array_t address_value = {SMDSAT_CMD_READ_ADDR_LEN, address_data};
			m_cmd.read_address(&address_value);
			nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
			*address  = (static_cast<uint32_t>(address_value.p_data[0]) << 24) |
			            (static_cast<uint32_t>(address_value.p_data[1]) << 16) |
			            (static_cast<uint32_t>(address_value.p_data[2]) << 8)  |
			            (address_value.p_data[3]);
		}

		if (seckey) {
			uint8_t seckey_data[SMDSAT_CMD_READ_SECKEY_LEN] = {0};
			smd_uint8_array_t seckey_value = {SMDSAT_CMD_READ_SECKEY_LEN, seckey_data};
			m_cmd.read_seckey(&seckey_value);
			nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
			*seckey = Binascii::hexlify(std::string(reinterpret_cast<char *>(seckey_data), SMDSAT_CMD_READ_SECKEY_LEN));
		}

		if (radioconf) {
			uint8_t rconf_raw[SMDSAT_CMD_READ_RCONF_RAW_LEN] = {0};
			uint16_t rconf_raw_len = 0;
			m_cmd.read_rconf_raw(rconf_raw, &rconf_raw_len);
			nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
			*radioconf = Binascii::hexlify(std::string(reinterpret_cast<char *>(rconf_raw),
			                               rconf_raw_len > 0 ? rconf_raw_len : SMDSAT_CMD_READ_RCONF_RAW_LEN));
			DEBUG_INFO("SmdSat::%s: RCONF_RAW: %s", __func__, radioconf->c_str());
		}
	} catch (...) {
		DEBUG_ERROR("SmdSat::%s: error during credential read", __func__);
		if (was_stopped) {
			m_cmd.deinit();
			shutdown();
			GPIOPins::release_sensors_pwr();
			nrf_gpio_cfg_default(BSP::GPIO_Inits[SAT_RESET].pin_number);
		}
		throw;
	}

	if (was_stopped) {
		m_cmd.deinit();
		shutdown();
		GPIOPins::release_sensors_pwr();
		nrf_gpio_cfg_default(BSP::GPIO_Inits[SAT_RESET].pin_number);
	}
}

// ============================================================================
// Runtime modulation switching
// ============================================================================

bool SmdSat::switch_modulation(KineisModulation mode, const std::string& rconf_hex) {
	// modswitch_t0/elapsed used only by MODSWITCH_LOG (currently no-op) — keep
	// the anchor in place so re-enabling the macro requires no other edits.
	[[maybe_unused]] uint64_t modswitch_t0 = PMU::get_timestamp_ms();
	[[maybe_unused]] auto modswitch_elapsed = [&]() -> unsigned {
		return static_cast<unsigned>(PMU::get_timestamp_ms() - modswitch_t0);
	};
	SmdArgosModulation target = kineis_to_smd_mod(mode);
	if (target == m_modulation) {
		MODSWITCH_LOG(0, "already in target modulation %d — no-op",
		              static_cast<int>(target));
		return true;
	}

	MODSWITCH_LOG(0, "switching %d -> %d (state=%d)",
	              static_cast<int>(m_modulation), static_cast<int>(target), static_cast<int>(m_state));

	if (rconf_hex.size() != 32) {
		DEBUG_ERROR("SmdSat::%s: invalid RCONF hex length %u (expected 32)", __func__, static_cast<unsigned>(rconf_hex.size()));
		return false;
	}

	// If SMD is stopped, defer the switch — RCONF will be written on next power-on
	if (m_state == SmdSatState::stopped) {
		MODSWITCH_LOG(modswitch_elapsed(), "SMD stopped, DEFERRED switch to %d (no SPI)",
		              static_cast<int>(target));
		m_modulation = target;
		m_pending_rconf = rconf_hex;
		// Persist so the next boot loads this modulation directly.
		if (configuration_store) {
			configuration_store->write_param(ParamID::ARGOS_CACHED_MODULATION,
			                                 static_cast<unsigned int>(m_modulation));
		}
		// Force state_load_kmac on next boot so the deferred RCONF gets applied.
		// Without this, idle_pending skips to idle and the RCONF is never written.
		is_kmac_profil_loaded = false;
		return true;
	}

	// Timing: set_radio_conf is a 2-phase write (~30ms SPI) but the STM32 needs
	// time to process it. save_radio_conf triggers a flash persistence (~100ms).
	// load_kmac_profil needs the MAC to re-initialize with the new RCONF.
	// Old timing (60ms everywhere) caused TX REQ failures because the MAC was
	// not ready after KMAC reload. Increase progressively:
	// - After RCONF write: 80ms (STM32 processes + small margin)
	// - After RCONF save: 120ms (flash write ~100ms + margin)
	// - After KMAC load:  200ms (MAC re-init, less than clean-repo's 1000ms)

	// 1. Write RCONF
	std::string rconf_bin = Binascii::unhexlify(rconf_hex);
	smd_uint8_array_t rconf_struct = {static_cast<uint16_t>(rconf_bin.size()),
	                                  reinterpret_cast<uint8_t *>(rconf_bin.data())};
	try {
		m_cmd.set_radio_conf(&rconf_struct);
	} catch (...) {
		DEBUG_ERROR("SmdSat::%s: failed to write RCONF", __func__);
		return false;
	}
	MODSWITCH_LOG(modswitch_elapsed(), "set_radio_conf OK, waiting 80ms");

	nrf_delay_ms(80);  // STM32 RCONF processing

	// 2. Save RCONF to flash
	try {
		if (!m_cmd.save_radio_conf()) {
			DEBUG_ERROR("SmdSat::%s: failed to save RCONF", __func__);
			return false;
		}
	} catch (...) {
		DEBUG_ERROR("SmdSat::%s: save_radio_conf exception", __func__);
		return false;
	}
	MODSWITCH_LOG(modswitch_elapsed(), "save_radio_conf OK (flash), waiting 120ms");

	nrf_delay_ms(120);  // Flash persistence (~100ms) + margin

	// 3. Reload KMAC profile 1
	try {
		m_cmd.load_kmac_profil(1);
	} catch (...) {
		DEBUG_ERROR("SmdSat::%s: failed to reload KMAC", __func__);
		return false;
	}
	MODSWITCH_LOG(modswitch_elapsed(), "load_kmac_profil(1) OK, waiting 200ms");

	nrf_delay_ms(200);  // MAC re-init after KMAC reload

	// Update cached modulation
	m_modulation = target;
	is_kmac_profil_loaded = true;
	// Persist so the next boot loads this modulation directly.
	if (configuration_store) {
		configuration_store->write_param(ParamID::ARGOS_CACHED_MODULATION,
		                                 static_cast<unsigned int>(m_modulation));
	}

	MODSWITCH_LOG(modswitch_elapsed(), "modulation switched to %d OK — TOTAL",
	              static_cast<int>(target));
	return true;
}

KineisModulation SmdSat::get_current_modulation() const {
	return smd_to_kineis_mod(m_modulation);
}

// ============================================================================
// DFU (delegates to command layer)
// ============================================================================

bool SmdSat::dfu_enter() {
	DEBUG_TRACE("SmdSat::%s", __func__);

	bool was_stopped = (m_state == SmdSatState::stopped);

	if (was_stopped) {
		GPIOPins::acquire_sensors_pwr();
		GPIOPins::set(SAT_PWR_EN);
		PMU::kick_watchdog();
		nrf_delay_ms(smdsat_delay_power_on_ms());
		PMU::kick_watchdog();
		nrf_delay_ms(smdsat_delay_power_on_ms());
#ifdef SMD_VPA_PIN
		GPIOPins::release_to_highz(SMD_VPA_PIN);
#endif

		// Only init SPI if it wasn't already running
		m_cmd.init();
	}
	// If SPI is already active (not stopped), do NOT re-init — dfu_enter() handles protocol reset

	PMU::kick_watchdog();
	nrf_delay_ms(smdsat_delay_power_on_ms() / 2);

	bool result = m_cmd.dfu_enter();

	if (!result) {
		if (was_stopped) {
			m_cmd.deinit();
			shutdown();
			GPIOPins::release_sensors_pwr();
			nrf_gpio_cfg_default(BSP::GPIO_Inits[SAT_RESET].pin_number);
		}
	}

	return result;
}

bool SmdSat::dfu_exit() {
	return m_cmd.dfu_exit();
}

bool SmdSat::dfu_get_bootloader_info(SmdDfuInfo *info) {
	return m_cmd.dfu_get_bootloader_info(info);
}

SmdDfuResponse SmdSat::firmware_update(const uint8_t *firmware, size_t size,
                                       void (*progress_callback)(uint8_t percent)) {
	DEBUG_INFO("SmdSat::%s: Starting firmware update | size=%u bytes", __func__, static_cast<unsigned>(size));

	if (firmware == nullptr || size == 0) {
		DEBUG_ERROR("SmdSat::%s: Invalid firmware data", __func__);
		return DFU_RSP_ERROR;
	}

	SmdDfuResponse result;

	if (!m_cmd.dfu_supported()) {
		DEBUG_ERROR("SmdSat::%s: DFU not supported by command layer", __func__);
		return DFU_RSP_ERROR;
	}

	// DFU failure cleanup: exit DFU mode so SMD returns to normal operation
	auto dfu_fail = [this](SmdDfuResponse r) -> SmdDfuResponse {
		m_cmd.dfu_exit();
		return r;
	};

	// Enter DFU mode
	if (!dfu_enter()) {
		DEBUG_ERROR("SmdSat::%s: Failed to enter DFU mode", __func__);
		return DFU_RSP_NOT_READY;
	}

	if (progress_callback) progress_callback(5);

	// Get bootloader info
	SmdDfuInfo dfu_info;
	if (!m_cmd.dfu_get_bootloader_info(&dfu_info)) {
		DEBUG_ERROR("SmdSat::%s: Failed to get bootloader info", __func__);
		return dfu_fail(DFU_RSP_ERROR);
	}

	if (size > dfu_info.app_max_size) {
		DEBUG_ERROR("SmdSat::%s: Firmware too large (%u > %u)", __func__,
		            static_cast<unsigned>(size), dfu_info.app_max_size);
		return dfu_fail(DFU_RSP_SIZE_ERROR);
	}

	if (progress_callback) progress_callback(15);

	// Erase
	result = m_cmd.dfu_erase();
	if (result != DFU_RSP_OK) {
		DEBUG_ERROR("SmdSat::%s: Flash erase failed", __func__);
		return dfu_fail(result);
	}

	if (progress_callback) progress_callback(25);

	// Write in chunks
	uint32_t addr = dfu_info.app_start_addr;
	size_t remaining = size;
	size_t offset = 0;
	uint8_t last_progress = 25;

	while (remaining > 0) {
		uint16_t chunk_size = (remaining > SMDSAT_DFU_CHUNK_SIZE) ?
		                      SMDSAT_DFU_CHUNK_SIZE : static_cast<uint16_t>(remaining);

		PMU::kick_watchdog();

		result = m_cmd.dfu_write_chunk(addr, &firmware[offset], chunk_size);
		if (result != DFU_RSP_OK) {
			DEBUG_ERROR("SmdSat::%s: Write failed at offset %u", __func__, static_cast<unsigned>(offset));
			return dfu_fail(result);
		}

		addr += chunk_size;
		offset += chunk_size;
		remaining -= chunk_size;

		if (progress_callback) {
			uint8_t progress = 25 + static_cast<uint8_t>((offset * 60) / size);
			if (progress > last_progress) {
				progress_callback(progress);
				last_progress = progress;
			}
		}

		nrf_delay_ms(5);
	}

	if (progress_callback) progress_callback(90);

	// Verify CRC32
	PMU::kick_watchdog();
	uint32_t crc = m_cmd.calculate_crc32(firmware, size);
	result = m_cmd.dfu_verify(crc);
	if (result != DFU_RSP_OK) {
		DEBUG_ERROR("SmdSat::%s: CRC verification failed", __func__);
		return dfu_fail(result);
	}

	if (progress_callback) progress_callback(95);

	// Jump to application
	m_cmd.dfu_jump();

	// Full power cycle — GPIO reset alone won't clear SRAM DFU magic
	DEBUG_INFO("SmdSat::%s: Power cycling SMD for clean app boot...", __func__);
	m_cmd.deinit();
	shutdown();
	PMU::kick_watchdog();
	nrf_delay_ms(500);
	PMU::kick_watchdog();

	power_on_blocking();

	m_new_firmware_version = get_firmware_version();
	if (!m_new_firmware_version.empty()) {
		DEBUG_INFO("SmdSat::%s: New firmware: %s", __func__, m_new_firmware_version.c_str());
	} else {
		DEBUG_WARN("SmdSat::%s: App not responding after power cycle", __func__);
	}

	if (progress_callback) progress_callback(100);

	DEBUG_INFO("SmdSat::%s: Firmware update completed!", __func__);
	return DFU_RSP_OK;
}

SmdDfuResponse SmdSat::firmware_update(File *file, size_t size, uint32_t stm32_crc32,
                                       void (*progress_callback)(uint8_t percent)) {
	DEBUG_INFO("SmdSat::%s: Starting streamed firmware update | size=%u bytes | CRC32=0x%08X",
	           __func__, static_cast<unsigned>(size), stm32_crc32);

	if (file == nullptr || size == 0) {
		return DFU_RSP_ERROR;
	}

	if (!m_cmd.dfu_supported()) {
		return DFU_RSP_ERROR;
	}

	SmdDfuResponse result;

	// DFU failure cleanup: exit DFU mode so SMD returns to normal operation
	auto dfu_fail = [this](SmdDfuResponse r) -> SmdDfuResponse {
		m_cmd.dfu_exit();
		return r;
	};

	if (!dfu_enter()) {
		return DFU_RSP_NOT_READY;
	}

	if (progress_callback) progress_callback(5);

	SmdDfuInfo dfu_info;
	if (!m_cmd.dfu_get_bootloader_info(&dfu_info)) {
		return dfu_fail(DFU_RSP_ERROR);
	}

	if (size > dfu_info.app_max_size) {
		return dfu_fail(DFU_RSP_SIZE_ERROR);
	}

	if (progress_callback) progress_callback(15);

	result = m_cmd.dfu_erase();
	if (result != DFU_RSP_OK) return dfu_fail(result);

	if (progress_callback) progress_callback(25);

	uint32_t addr = dfu_info.app_start_addr;
	size_t remaining = size;
	size_t offset = 0;
	uint8_t last_progress = 25;
	uint8_t chunk_buf[SMDSAT_DFU_CHUNK_SIZE];

	while (remaining > 0) {
		uint16_t chunk_size = (remaining > SMDSAT_DFU_CHUNK_SIZE) ?
		                      SMDSAT_DFU_CHUNK_SIZE : static_cast<uint16_t>(remaining);

		PMU::kick_watchdog();

		lfs_ssize_t bytes_read = file->read(chunk_buf, chunk_size);
		if (bytes_read != static_cast<lfs_ssize_t>(chunk_size)) {
			return dfu_fail(DFU_RSP_ERROR);
		}

		result = m_cmd.dfu_write_chunk(addr, chunk_buf, chunk_size);
		if (result != DFU_RSP_OK) return dfu_fail(result);

		addr += chunk_size;
		offset += chunk_size;
		remaining -= chunk_size;

		if (progress_callback) {
			uint8_t progress = 25 + static_cast<uint8_t>((offset * 60) / size);
			if (progress > last_progress) {
				progress_callback(progress);
				last_progress = progress;
			}
		}

		nrf_delay_ms(5);
	}

	if (progress_callback) progress_callback(90);

	result = m_cmd.dfu_verify(stm32_crc32);
	if (result != DFU_RSP_OK) return dfu_fail(result);

	if (progress_callback) progress_callback(95);

	m_cmd.dfu_jump();

	// Full power cycle after JUMP to ensure clean app boot.
	// A GPIO reset alone would NOT clear SRAM, so the DFU magic (0x4446554D)
	// at 0x2000FFF8 would survive and cause the bootloader to re-enter DFU.
	// Power cycle cuts VCC → SRAM is lost → clean boot into new app.
	DEBUG_INFO("SmdSat::%s: Power cycling SMD for clean app boot...", __func__);
	m_cmd.deinit();
	shutdown();  // SAT_PWR_EN OFF + SAT_RESET LOW
	PMU::kick_watchdog();
	nrf_delay_ms(500);  // Capacitors discharge, SRAM loses state
	PMU::kick_watchdog();

	power_on_blocking();

	m_new_firmware_version = get_firmware_version();
	if (!m_new_firmware_version.empty()) {
		DEBUG_INFO("SmdSat::%s: New firmware: %s", __func__, m_new_firmware_version.c_str());
	} else {
		DEBUG_WARN("SmdSat::%s: App not responding after power cycle", __func__);
	}

	if (progress_callback) progress_callback(100);
	return DFU_RSP_OK;
}

SmdDfuResponse SmdSat::firmware_update_from_file(const std::string& filepath,
                                                 void (*progress_callback)(uint8_t percent)) {
	(void)filepath;
	(void)progress_callback;
	DEBUG_ERROR("SmdSat::%s: Not implemented", __func__);
	return DFU_RSP_ERROR;
}

bool SmdSat::cw_start(uint32_t freq_hz, uint16_t power_dbm, uint16_t duration_s) {
	DEBUG_INFO("SmdSat::cw_start: %lu Hz, %u dBm, %u s",
	           (unsigned long)freq_hz, power_dbm, duration_s);

	bool was_stopped = (m_state == SmdSatState::stopped);
	if (was_stopped) {
		power_on_blocking();
	}

	uint8_t payload[9];
	payload[0] = 0x01;  // mode = start
	payload[1] = static_cast<uint8_t>(freq_hz & 0xFF);
	payload[2] = static_cast<uint8_t>((freq_hz >> 8) & 0xFF);
	payload[3] = static_cast<uint8_t>((freq_hz >> 16) & 0xFF);
	payload[4] = static_cast<uint8_t>((freq_hz >> 24) & 0xFF);
	payload[5] = static_cast<uint8_t>(power_dbm & 0xFF);
	payload[6] = static_cast<uint8_t>((power_dbm >> 8) & 0xFF);
	size_t len = 7;
	if (duration_s > 0) {
		payload[7] = static_cast<uint8_t>(duration_s & 0xFF);
		payload[8] = static_cast<uint8_t>((duration_s >> 8) & 0xFF);
		len = 9;
	}

	try {
		m_cmd.write_cw(payload, static_cast<uint16_t>(len));
		return true;
	} catch (...) {
		DEBUG_ERROR("SmdSat::cw_start: write_cw threw");
		return false;
	}
}

bool SmdSat::cw_stop() {
	DEBUG_INFO("SmdSat::cw_stop");
	if (m_state == SmdSatState::stopped) return true;

	uint8_t payload[7] = { 0x00, 0, 0, 0, 0, 0, 0 };  // mode = stop
	try {
		m_cmd.write_cw(payload, sizeof(payload));
		return true;
	} catch (...) {
		DEBUG_ERROR("SmdSat::cw_stop: write_cw threw");
		return false;
	}
}

std::string SmdSat::get_firmware_version() {
	DEBUG_TRACE("SmdSat::%s", __func__);

	bool was_stopped = (m_state == SmdSatState::stopped);

	if (was_stopped) {
		power_on_blocking();
	}

	std::string version = "";

	try {
		bool smd_ready = !was_stopped || m_cmd.ping();  // Already pinged in power_on_blocking

		if (smd_ready) {
			uint8_t ver[64] = {0};
			m_cmd.read_version(ver);
			size_t len = 0;
			while (len < sizeof(ver) && ver[len] != 0 && ver[len] >= 0x20 && ver[len] < 0x7F) {
				len++;
			}
			version = std::string((char*)ver, len);
			DEBUG_INFO("SmdSat::%s: Firmware version: %s", __func__, version.c_str());
		}
	} catch (...) {
		DEBUG_ERROR("SmdSat::%s: Failed to read firmware version", __func__);
	}

	if (was_stopped) {
		m_cmd.deinit();
		shutdown();
		GPIOPins::release_sensors_pwr();
		nrf_gpio_cfg_default(BSP::GPIO_Inits[SAT_RESET].pin_number);
	}

	return version;
}

std::string SmdSat::smd_spi_test() {
	bool was_stopped = (m_state == SmdSatState::stopped);

	if (was_stopped) {
		DEBUG_INFO("SmdSat::%s: Powering on SMD for test...", __func__);
		power_on_blocking();
	}

	std::string result = m_cmd.run_command_test();

	if (was_stopped) {
		DEBUG_INFO("SmdSat::%s: Powering off SMD after test", __func__);
		m_cmd.deinit();
		shutdown();
		GPIOPins::release_sensors_pwr();
		nrf_gpio_cfg_default(BSP::GPIO_Inits[SAT_RESET].pin_number);
	}

	return result;
}
