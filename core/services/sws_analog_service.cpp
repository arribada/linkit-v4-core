/**
 * @file sws_analog_service.cpp
 * @brief SWS analog — static members, service lifecycle, test mode, guided calibration, ADC read.
 */

#include <cstdint>
#include "sws_analog_service.hpp"
#include "sws_analog_constants.hpp"
#include "debug.hpp"
#include "gpio.hpp"
#include "pmu.hpp"
#include "nrfx_saadc.h"
#include "nrf_peripheral_power.hpp"
#ifndef CPPUTEST
#include "nrf_gpio.h"
#endif
#include "rgb_led.hpp"
#include "rtc.hpp"

// CRC16 and constants are in sws_analog_constants.hpp

// LED for test mode visual feedback
extern RGBLed *status_led;
extern RTC *rtc;

// Tuning constants (ADC, detection levels, adaptive delay, guided calibration)
// all live in sws_analog_constants.hpp, which this file includes above. They
// used to be redefined here as well -- 46 macros, same values, no #ifndef. That
// is legal C++ only while the two copies agree; the day one drifted, this
// translation unit and sws_analog_detection.cpp would have compiled cleanly and
// behaved differently.

// Initialize static members
SWSAnalogService::CalibrationData SWSAnalogService::m_calib;
uint16_t SWSAnalogService::m_observed_peak_adc;
uint16_t SWSAnalogService::m_observed_peak_crc;
uint32_t SWSAnalogService::m_sample_delay_us_noinit;
uint16_t SWSAnalogService::m_sample_delay_us_crc;
SWSAnalogService::Diagnostics SWSAnalogService::m_diag;
uint16_t SWSAnalogService::m_diag_crc;
SWSAnalogService::Status SWSAnalogService::m_status = {};
SWSAnalogService* SWSAnalogService::s_instance = nullptr;
bool SWSAnalogService::m_test_mode = false;
uint64_t SWSAnalogService::m_test_mode_start_time = 0;
uint64_t SWSAnalogService::m_test_timeout_ms = SWSAnalogService::TEST_TIMEOUT_DEFAULT_MS;
uint32_t SWSAnalogService::m_heartbeat_warn_uw_sec = SWSAnalogService::HEARTBEAT_WARN_UW_DEFAULT_SEC;
uint32_t SWSAnalogService::m_heartbeat_warn_surf_sec = SWSAnalogService::HEARTBEAT_WARN_SURF_DEFAULT_SEC;
uint64_t SWSAnalogService::m_last_heartbeat_warn_time = 0;
SWSAnalogService::CalibPhase SWSAnalogService::m_calib_phase = SWSAnalogService::CalibPhase::IDLE;
uint32_t SWSAnalogService::m_calib_sum = 0;
uint8_t SWSAnalogService::m_calib_count = 0;
uint16_t SWSAnalogService::m_calib_air_result = 0;
uint16_t SWSAnalogService::m_calib_water_result = 0;
uint8_t SWSAnalogService::m_calib_stable_count = 0;
uint16_t SWSAnalogService::m_calib_prev_value = 0;
uint16_t SWSAnalogService::m_calib_timeout_ticks = 0;
bool SWSAnalogService::m_calib_water_success = false;
std::function<void(const SWSAnalogService::CalibResult&)> SWSAnalogService::m_calib_notify;
std::function<void(const SWSAnalogService::Status&)> SWSAnalogService::m_status_notify;
std::function<void()> SWSAnalogService::m_on_test_stop;
#if ENABLE_SWS_LOG
Logger *SWSAnalogService::m_sws_logger = nullptr;
#endif

/// @brief Get current SWS status snapshot (for DTE SWSST command).
/// @return Status struct with all diagnostic fields.
SWSAnalogService::Status SWSAnalogService::get_status() {
    return m_status;
}

// ═══════════════════════════════════════════════════════
//  PERSISTENT DIAGNOSTIC COUNTERS (audit 2026-05 R-MON-01)
// ═══════════════════════════════════════════════════════

/// @brief Read current persistent counters (for DTE SWSSTATS).
SWSAnalogService::Diagnostics SWSAnalogService::get_diagnostics() {
    return m_diag;
}

/// @brief Reset all counters to zero (DTE-callable).
void SWSAnalogService::clear_diagnostics() {
    memset(&m_diag, 0, sizeof(m_diag));
    update_diagnostics_crc();
    DEBUG_INFO("SWSAnalog: Diagnostics cleared");
}

/// @brief Validate noinit Diagnostics via CRC16. Zero on corruption.
/// Called from service_init() after the calibration validation pass.
void SWSAnalogService::validate_diagnostics() {
    uint16_t calc = crc16_compute((const uint8_t *)&m_diag, sizeof(m_diag), nullptr);
    if (calc != m_diag_crc) {
        memset(&m_diag, 0, sizeof(m_diag));
        update_diagnostics_crc();
        DEBUG_INFO("SWSAnalog: Diagnostics CRC invalid — cleared");
    } else {
        DEBUG_INFO("SWSAnalog: Diagnostics restored from noinit "
                   "stuck=%u coh=%u div=%u force=%u spike=%u peak=%u saadc=%u",
                   m_diag.stuck_recovery_count, m_diag.coherence_recalib_count,
                   m_diag.dive_timeout_count, m_diag.force_surface_count,
                   m_diag.spike_reject_count, m_diag.peak_incoherent_count,
                   m_diag.saadc_init_retry_count);
    }
}

/// @brief Recompute m_diag_crc — call after any counter mutation.
void SWSAnalogService::update_diagnostics_crc() {
    m_diag_crc = crc16_compute((const uint8_t *)&m_diag, sizeof(m_diag), nullptr);
}

/// @brief Saturating increment of a Diagnostics counter (no wrap).
void SWSAnalogService::inc_diag(uint16_t &counter) {
    if (counter < UINT16_MAX) {
        counter++;
        update_diagnostics_crc();
    }
}

/// @brief Start SWS test mode — force-enable service, LED feedback on state changes.
void SWSAnalogService::start_test_mode() {
    m_test_mode = true;
    // Capture wall-clock start so detector_state() can auto-stop after
    // m_test_timeout_ms — protects against forgotten test mode draining
    // battery on a deployed unit (DTE SWSTST,1 without a follow-up SWSTST,0).
    m_test_mode_start_time = PMU::get_timestamp_ms();
    if (s_instance) {
        DEBUG_INFO("SWSAnalog: Test mode started");
        s_instance->start();
        // Set initial LED to reflect current state.
        // BLUE = underwater, GREEN = surface (convention shared with ledsm).
        if (status_led) {
            if (s_instance->m_current_state)
                status_led->set(RGBLedColor::BLUE);    // UNDERWATER
            else
                status_led->set(RGBLedColor::GREEN);   // SURFACE
        }
    }
}

/// @brief Configure test-mode auto-stop timeout (0 = disabled).
void SWSAnalogService::set_test_timeout_ms(uint64_t ms) {
    m_test_timeout_ms = ms;
}

/// @brief Configure heartbeat WARN thresholds (audit 2026-05 R-CODE-04).
/// @param uw_sec    Seconds without state change in UW before WARN (0 = disabled)
/// @param surf_sec  Seconds without state change in SURF before WARN (0 = disabled)
void SWSAnalogService::set_heartbeat_thresholds_sec(uint32_t uw_sec, uint32_t surf_sec) {
    m_heartbeat_warn_uw_sec = uw_sec;
    m_heartbeat_warn_surf_sec = surf_sec;
}

/// @brief Stop SWS test mode — turn off LED, invoke on_test_stop callback.
void SWSAnalogService::stop_test_mode() {
    if (s_instance) {
        s_instance->stop();
        DEBUG_INFO("SWSAnalog: Test mode stopped");
    }
    m_test_mode = false;
    // Restore normal LED behavior
    if (m_on_test_stop)
        m_on_test_stop();
}

/// @brief Check if test mode is currently active.
/// @return true if test mode started and instance exists.
bool SWSAnalogService::is_test_running() {
    return m_test_mode;
}

// ═══════════════════════════════════════════════════════
//  GUIDED CALIBRATION — LED-assisted air/water measurement
//
//  GREEN flashing  → place in AIR  → samples → GREEN solid
//  BLUE  flashing  → place in WATER → samples → BLUE solid
//  WHITE flash     → done (success)
//  RED flash       → done (failure)
// ═══════════════════════════════════════════════════════


/// @brief Start guided calibration — GREEN=air phase, BLUE=water phase, async completion notify.
void SWSAnalogService::start_guided_calibration() {
    if (!s_instance) return;

    m_calib_phase = CalibPhase::AIR_WAITING;
    m_calib_timeout_ticks = 0;
    m_calib_sum = 0;
    m_calib_count = 0;
    m_calib_air_result = 0;
    m_calib_water_result = 0;
    m_calib_stable_count = 0;
    // Sentinel (outside valid 14-bit ADC range) marks "no prior sample yet"
    // — distinguishes the first iteration from a legitimate raw=0 reading.
    m_calib_prev_value = UINT16_MAX;

    // Start the SWS service if not already running
    m_test_mode = true;
    s_instance->start();

    DEBUG_INFO("SWSAnalog: Guided calibration started — place device in AIR");
    if (status_led) status_led->flash(RGBLedColor::GREEN, 500);
}

/// @brief Cancel in-progress guided calibration and notify failure.
/// No-op when IDLE (not started) or DONE (already completed) — without the
/// DONE guard, a SWSCAL,0 sent after a successful run would emit a spurious
/// status=3 notify that the GUI could interpret as a retroactive failure.
void SWSAnalogService::cancel_guided_calibration() {
    if (m_calib_phase == CalibPhase::IDLE ||
        m_calib_phase == CalibPhase::DONE) return;

    m_calib_phase = CalibPhase::IDLE;
    m_test_mode = false;
    if (s_instance) s_instance->stop();

    if (status_led) status_led->flash(RGBLedColor::RED, 200);
    DEBUG_INFO("SWSAnalog: Guided calibration cancelled");

    if (m_calib_notify) {
        CalibResult r = {3, 0, 0};  // status=cancelled
        m_calib_notify(r);
    }
}

/// @brief True only while the state machine is actively progressing.
/// DONE is a sticky terminal state (results stay readable via
/// get_guided_calibration_result()), but the run is not active anymore —
/// otherwise a GUI polling this flag would never see the run finish.
bool SWSAnalogService::is_guided_calibration_running() {
    return m_calib_phase != CalibPhase::IDLE &&
           m_calib_phase != CalibPhase::DONE;
}

SWSAnalogService::CalibResult SWSAnalogService::get_guided_calibration_result() {
    if (m_calib_phase == CalibPhase::DONE) {
        return {1, m_calib_air_result, m_calib_water_result};  // success
    } else if (m_calib_phase == CalibPhase::IDLE && m_calib_air_result > 0) {
        return {1, m_calib_air_result, m_calib_water_result};  // already done
    }
    return {0, 0, 0};  // in progress
}

void SWSAnalogService::set_guided_calib_notify(std::function<void(const CalibResult&)> fn) {
    m_calib_notify = fn;
}

void SWSAnalogService::clear_guided_calib_notify() {
    m_calib_notify = nullptr;
}

void SWSAnalogService::set_status_notify(std::function<void(const Status&)> fn) {
    m_status_notify = fn;
}

void SWSAnalogService::clear_status_notify() {
    m_status_notify = nullptr;
}

void SWSAnalogService::set_on_test_stop(std::function<void()> fn) {
    m_on_test_stop = fn;
}

void SWSAnalogService::clear_on_test_stop() {
    m_on_test_stop = nullptr;
}

// SAADC event handler (required but not used for blocking mode)
static void nrfx_saadc_event_handler_sws(nrfx_saadc_evt_t const *p_event)
{
    (void)p_event;
}

// ═══════════════════════════════════════════════════════
//  MANUAL CALIBRATION (via DTE $SCALW/$SCALR, persisted in SWS.CAL)
//  Offset 0 = expected water ADC
//  Offset 1 = expected air ADC
// ═══════════════════════════════════════════════════════


/// @brief Init: load config params, validate/restore calibration, set adaptive delay.
void SWSAnalogService::service_init() {
    UWDetectorService::service_init();

    // SWS analog has its own internal filtering (MA2, 5-level detection, consecutive
    // samples). Multi-sample batching is redundant and wastes CPU in sample_gap waits.
    m_max_samples = 1;

    // One-time SAADC offset calibration — retained until power reset.
    // Without this, readings can be offset by hundreds of ADC counts.
#ifndef CPPUTEST
    nrfx_saadc_init(&BSP::ADC_Inits.config, nrfx_saadc_event_handler_sws);
    nrfx_saadc_calibrate_offset();
    // Audit 2026-05 R-CODE-05: timeout-guarded busy wait. Nominal calibration
    // completes in <1ms; the 100ms ceiling catches a hung SAADC peripheral
    // (HW failure, brown-out) without bricking the boot. We proceed anyway —
    // a less-accurate offset is far better than a boot loop.
    {
        uint64_t saadc_wait_start = PMU::get_timestamp_ms();
        constexpr uint64_t SAADC_CALIB_TIMEOUT_MS = 100;
        while (nrfx_saadc_is_busy()) {
            if ((PMU::get_timestamp_ms() - saadc_wait_start) >= SAADC_CALIB_TIMEOUT_MS) {
                DEBUG_ERROR("SWSAnalog: SAADC offset calibration timeout (%llums) — continuing without precise offset",
                            (unsigned long long)SAADC_CALIB_TIMEOUT_MS);
                break;
            }
        }
    }
    nrfx_saadc_uninit();
    nrf_peripheral_power_reset(NRF_SAADC_BASE_ADDR);  // Errata 241: prevent 400 µA idle leak
#endif

    // Load configuration
    m_hysteresis_percent = service_read_param<unsigned int>(ParamID::SWS_ANALOG_HYSTERESIS);
    m_calib_interval_sec = service_read_param<unsigned int>(ParamID::SWS_ANALOG_CALIB_INTERVAL);
    m_max_dive_time_sec = service_read_param<unsigned int>(ParamID::UW_MAX_DIVE_TIME);
    m_min_surface_time_sec = service_read_param<unsigned int>(ParamID::UW_MIN_SURFACE_TIME);
    // Defense in depth: UNP25=0 disables the L-override lockout entirely, which
    // combined with a short SAMPLING_SURF_FREQ allows rapid flap. The
    // OVERRIDE_MIN_TIME_SEC=1 backstop covers most of this, but the lockout
    // floor here is a cleaner guarantee. Clamp without warning since 0 is in
    // the valid DTE range — operators may have set it intentionally.
    if (m_min_surface_time_sec < 1) {
        m_min_surface_time_sec = 1;
    }

    if (m_hysteresis_percent > 50) {
        m_hysteresis_percent = DEFAULT_HYSTERESIS_PERCENT;
    }

    m_threshold_ratio_percent = DEFAULT_THRESHOLD_RATIO_PERCENT;
    m_alpha_percent = DEFAULT_ALPHA_PERCENT;

    // Load configurable delay bounds (UNP09/UNP10)
    m_delay_min_us = service_read_param<unsigned int>(ParamID::SWS_DELAY_MIN_US);
    m_delay_max_us = service_read_param<unsigned int>(ParamID::SWS_DELAY_MAX_US);
    if (m_delay_min_us < 50) m_delay_min_us = 50;
    if (m_delay_max_us < m_delay_min_us) m_delay_max_us = m_delay_min_us;

    // Initialize adaptive sample delay from config (already in µs — UNP08
    // was renamed from UW_PIN_SAMPLE_DELAY (ms) to UW_PIN_SAMPLE_DELAY_US
    // for consistency with UNP09/UNP10).
    //
    // F-SWS-7 audit fix: restore from .noinit if CRC valid and within bounds.
    // The converged delay can drift over weeks as biofouling progresses —
    // a soft reset (WDT) would otherwise reset it to UNP08 default and
    // require re-convergence (minutes of misreport during the window).
    uint16_t delay_crc = crc16_compute((const uint8_t *)&m_sample_delay_us_noinit,
                                        sizeof(m_sample_delay_us_noinit), nullptr);
    if (delay_crc == m_sample_delay_us_crc &&
        m_sample_delay_us_noinit >= m_delay_min_us &&
        m_sample_delay_us_noinit <= m_delay_max_us) {
        m_sample_delay_us = m_sample_delay_us_noinit;
        DEBUG_INFO("SWSAnalog: sample delay restored from noinit = %u µs", m_sample_delay_us);
    } else {
        m_sample_delay_us = m_enable_sample_delay;
        if (m_sample_delay_us < m_delay_min_us) m_sample_delay_us = m_delay_min_us;
        if (m_sample_delay_us > m_delay_max_us) m_sample_delay_us = m_delay_max_us;
        DEBUG_INFO("SWSAnalog: sample delay from UNP08 = %u µs (noinit invalid)", m_sample_delay_us);
    }

    // Validate observed peak ADC from noinit RAM
    uint16_t peak_crc = crc16_compute((const uint8_t *)&m_observed_peak_adc,
                                       sizeof(m_observed_peak_adc), nullptr);
    if (peak_crc != m_observed_peak_crc || m_observed_peak_adc > ADC_INVALID_MAX) {
        m_observed_peak_adc = 0;  // Will be learned from live readings
        DEBUG_INFO("SWSAnalog: Observed peak ADC reset (invalid noinit)");
    } else {
        DEBUG_INFO("SWSAnalog: Observed peak ADC=%u (from noinit)", m_observed_peak_adc);
    }

    // Validate persistent diagnostic counters (audit 2026-05 R-MON-01)
    validate_diagnostics();

    // Validate or initialize calibration from noinit RAM
    if (!validate_calibration_data()) {
        DEBUG_INFO("SWSAnalog: Calibration invalid | trying flash backup");
        memset(&m_calib, 0, sizeof(m_calib));
        if (!load_calibration_from_flash()) {
            DEBUG_INFO("SWSAnalog: No flash backup | clearing and recalibrating");
            calibrate_air_baseline();
        }
    } else {
        DEBUG_INFO("SWSAnalog: Calibration valid - air=%u water=%u thresh=%u",
                   m_calib.threshold_air, m_calib.threshold_water, m_calib.threshold_current);
    }

    // Initialize state tracking
    m_last_state_change_time = 0;
    m_time_in_current_state = 0;
    m_consecutive_samples = 0;
    // Surface readings buffer
    m_surface_readings_idx = 0;
    m_surface_readings_count = 0;
    for (int i = 0; i < SURFACE_BUFFER_SIZE; i++)
        m_surface_readings[i] = 0;

    // Level 1 & 2: fast drop
    m_prev_raw = 0;
    m_drop_reference = 0;
    m_consecutive_raw_drops = 0;
    m_l4_consecutive_below = 0;

    // Level 3: trend MA3
    m_trend_buffer_idx = 0;
    m_trend_buffer_count = 0;
    m_prev_ma3 = 0;
    m_ma3_trend_start = 0;
    m_ma3_trend_count = 0;
    for (int i = 0; i < TREND_MA_SIZE; i++)
        m_trend_buffer[i] = 0;

    // Level 4 & 5
    m_peak_adc_since_underwater = 0;

    // Safety
    m_surface_lockout_remaining = 0;
    m_consecutive_dive_timeouts = 0;

    // First-sample coherence check
    m_first_sample_done = false;
    m_fast_convergence_count = 0;
    m_coherence_high_count = 0;
    m_air_collapse_count = 0;

    DEBUG_INFO("SWSAnalog: Init - hyst=%u%% ratio=%u%%",
               m_hysteresis_percent, m_threshold_ratio_percent);
}

/// @brief Enabled if UNDERWATER_EN=true and source is SWS, or test mode active.
/// @return true if SWS detection should run.
bool SWSAnalogService::service_is_enabled() {
    if (m_test_mode) return true;
    return service_read_param<bool>(ParamID::UNDERWATER_EN);
}

/// @brief Schedule: 1s during guided calibration, fast period in test mode, otherwise UNP03/UNP04.
/// @return Delay in ms until next sample.
unsigned int SWSAnalogService::service_next_schedule_in_ms() {
    // Fast 1s sampling during guided calibration for responsive user experience
    if (m_calib_phase != CalibPhase::IDLE && m_calib_phase != CalibPhase::DONE) {
        return CALIB_SAMPLE_INTERVAL_MS;
    }
    // SWSTST,1 (test mode): fixed fast polling, bypasses UNP03/UNP04 for bench/cable
    // testing. Auto-stop (m_test_timeout_ms, default 1h) prevents battery drain on
    // a deployed unit if SWSTST,0 is forgotten.
    if (m_test_mode) {
        return SWS_TEST_MODE_SAMPLE_MS;
    }
    // Cooldown gate (2026-05): defer SWS sampling while
    // MIN_SURFACE_CYCLE_INTERVAL_S is still running. Without this guard the
    // boot path (Service::start → reschedule → here) samples within seconds
    // of POR even though cooldown is active — emits a phantom surface event
    // that triggers the passive-surfacing path and one full SWS measurement
    // cycle just to be silenced again. The exit_cooldown_sleep() wake timer
    // (set in enter_cooldown_sleep) re-emits state when cooldown expires.
    // Skipped during guided calibration / test mode (handled above) so DTE
    // diagnostics never get blocked.
    if (rtc && rtc->is_set() && ServiceManager::is_in_cooldown(service_current_time())) {
        DEBUG_TRACE("SWSAnalogService::service_next_schedule_in_ms: cooldown active — SCHEDULE_DISABLED");
        return Service::SCHEDULE_DISABLED;
    }
    // Normal: read configured surface/underwater period (seconds, supports fractions ≥ 0.1)
    double period_s = m_current_state ?
        service_read_param<double>(ParamID::SAMPLING_UNDER_FREQ) :
        service_read_param<double>(ParamID::SAMPLING_SURF_FREQ);
    unsigned int period = static_cast<unsigned int>(period_s * 1000.0);
    return period > 0 ? period : 1000;
}

/// @brief Read one ADC sample from SWS channel with configurable RC charge delay.
/// @return Raw 14-bit ADC value (0-16383), or ADC_READ_ERROR on failure.
///
/// Power-rail dependency (audit 2026-05 R-DOC-01 / R-08 neutralized):
///   The SAADC and the SWS RC charge circuit require VSENSORS to be powered.
///   The main loop in ports/nrf52840/main.cpp:1077-1086 guarantees that
///   PMU::restore_power_rails() is called BEFORE system_scheduler->run() any
///   time the scheduler is about to fire a task (idle threshold 5s). Since
///   SWSAnalogService is invoked via the scheduler, this function always runs
///   with VSENSORS powered up — there is no risk of reading garbage from a
///   floating analog frontend during deep idle.
uint16_t SWSAnalogService::read_analog_sws() {
#ifdef CPPUTEST
    // Test stub — drive the SAADC fake so SAADC::set_adc_value(NNN) is observable
    // by detector_state(). Mirrors the production path (init → channel_init →
    // sample_convert) so the fake returns the most recent injected value.
    nrfx_saadc_init(nullptr, nullptr);
    nrfx_saadc_channel_init(0, nullptr);
    nrf_saadc_value_t raw = 0;
    nrfx_saadc_sample_convert(0, &raw);
    nrfx_saadc_channel_uninit(0);
    nrfx_saadc_uninit();
    return static_cast<uint16_t>(raw < 0 ? 0 : raw);
#else
    // Disconnect digital input buffer on SWS pin to prevent analog loading.
    uint32_t sws_pin = BSP::GPIO_Inits[SWS_SAMPLE_PIN].pin_number;
    nrf_gpio_cfg(sws_pin, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_DISCONNECT,
                 NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE);

    // RC discrimination: 100nF cap charges through water resistance.
    GPIOPins::set(SWS_ENABLE_PIN);
    PMU::delay_us(m_sample_delay_us);

    nrfx_err_t init_err = nrfx_saadc_init(&BSP::ADC_Inits.config, nrfx_saadc_event_handler_sws);
    if (init_err == NRFX_ERROR_INVALID_STATE) {
        inc_diag(m_diag.saadc_init_retry_count);
        nrfx_saadc_uninit();
        nrf_peripheral_power_reset(NRF_SAADC_BASE_ADDR);
        init_err = nrfx_saadc_init(&BSP::ADC_Inits.config, nrfx_saadc_event_handler_sws);
    }
    if (init_err != NRFX_SUCCESS) {
        GPIOPins::clear(SWS_ENABLE_PIN);
        return ADC_READ_ERROR;
    }
    nrfx_saadc_channel_init(SWS_ADC, &BSP::ADC_Inits.channel_config[SWS_ADC]);

    // 2026-05-24 REVERT of F-SWS-1 (double-sample reject) — field test in
    // real saltwater showed the natural sample-to-sample variation in the
    // RC discrimination circuit (100 nF cap charging through seawater) is
    // routinely >12% at 500 µs intervals due to electrode contact noise,
    // micro-bubbles, biofouling, conductivity gradients. The hypothesised
    // Vbatt-sag-during-SMD-power-on was not the dominant noise source.
    // F-SWS-1 was rejecting ~95% of legitimate samples → SWS could never
    // produce a valid reading → state stuck. The base L1-L5 / hysteresis /
    // coherence / peak-validation / M1-M12 safety mesh in the detector
    // already handles real-world noise correctly.
    nrf_saadc_value_t raw = 0;
    nrfx_err_t err = nrfx_saadc_sample_convert(SWS_ADC, &raw);
    if (err != NRFX_SUCCESS) {
        DEBUG_ERROR("SWSAnalog: ADC conversion failed %d", err);
        nrfx_saadc_uninit();
    nrf_peripheral_power_reset(NRF_SAADC_BASE_ADDR);  // Errata 241: prevent 400 µA idle leak
        GPIOPins::clear(SWS_ENABLE_PIN);
        return ADC_READ_ERROR;
    }

    nrfx_saadc_uninit();
    nrf_peripheral_power_reset(NRF_SAADC_BASE_ADDR);  // Errata 241: prevent 400 µA idle leak
    GPIOPins::clear(SWS_ENABLE_PIN);

    return static_cast<uint16_t>(raw < 0 ? 0 : raw);
#endif // CPPUTEST
}
