#pragma once

#include "bsp.hpp"
#include "uwdetector_service.hpp"
#include "calibration.hpp"
#include "logger.hpp"
#include "messages.hpp"
#include "timeutils.hpp"

#if ENABLE_SWS_LOG

struct __attribute__((packed)) SWSLogEntry {
	LogHeader header;
	union {
		struct __attribute__((packed)) {
			uint16_t raw_adc;          // Raw ADC reading
			uint16_t filtered_adc;     // Filtered (MA2) ADC reading
			uint16_t threshold;        // Current active threshold
			uint16_t hysteresis;       // Hysteresis value (ADC counts)
			uint16_t air;              // Air baseline ADC
			uint16_t water;            // Water baseline ADC
			uint8_t calibrated;        // Calibration valid flag
			uint8_t underwater;        // Current state (1=underwater, 0=surface)
			uint16_t time_in_state;    // Seconds in current state (max 65535s = 18h)
			uint8_t surface_level;     // Detection level (0=none, 1-5=L1-L5)
			uint16_t contrast_x10;     // Water/air contrast ratio x10
			uint16_t observed_peak;    // Highest ADC value observed
			uint16_t sample_delay_us;  // Adaptive RC charge delay (µs, max 5000)
		};
		uint8_t data[MAX_LOG_PAYLOAD];
	};
};
static_assert(sizeof(SWSLogEntry) - sizeof(LogHeader) <= MAX_LOG_PAYLOAD, "SWSLogEntry payload too large");

class SWSLogFormatter : public LogFormatter {
public:
	const std::string header() override {
		return "log_datetime,raw_adc,filtered_adc,threshold,hysteresis,air,water,calibrated,underwater,time_in_state,"
		       "surface_level,contrast_x10,observed_peak,sample_delay_us\r\n";
	}
	const std::string log_entry(const LogEntry &e) override {
		char entry[512], d1[128];
		const auto *log = reinterpret_cast<const SWSLogEntry *>(&e);
		std::time_t t;
		std::tm *tm;

		t = convert_epochtime(log->header.year, log->header.month, log->header.day, log->header.hours,
		                      log->header.minutes, log->header.seconds);
		tm = std::gmtime(&t);
		std::strftime(d1, sizeof(d1), "%d/%m/%Y %H:%M:%S", tm);

		snprintf(entry, sizeof(entry), "%s,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",  // NOLINT
		         d1, log->raw_adc, log->filtered_adc, log->threshold, log->hysteresis, log->air, log->water,
		         log->calibrated, log->underwater, log->time_in_state, log->surface_level, log->contrast_x10,
		         log->observed_peak, log->sample_delay_us);
		return std::string(entry);
	}
};

#endif  // ENABLE_SWS_LOG

/**
 * @brief SWS Analog Surface Detection — fast, adaptive, biofouling-resistant
 *
 * RC charge discrimination via 100nF cap + adaptive delay (100-5000µs).
 * 5-level surface override (L1-L5) for fast detection even with degraded electrodes.
 * Auto-calibration of air/water baselines with noinit RAM persistence.
 *
 * Test mode (DTE SWSTST): LED BLUE=underwater, GREEN=surface. Async status push.
 */
class SWSAnalogService : public UWDetectorService, public Calibratable {
public:
	// Detection method IDs (for IHM visualization)
	enum DetectMethod : uint8_t {
		DETECT_NONE = 0,       // No transition this sample
		DETECT_THRESHOLD = 1,  // Simple threshold crossing
		DETECT_RAPID_T1 = 2,   // Rapid Tier 1 - single sharp drop
		DETECT_RAPID_T2 = 3,   // Rapid Tier 2 - confirmed moderate drop
		DETECT_RAPID_T3 = 4,   // Rapid Tier 3 - trend-based small drop
		DETECT_RAPID_T4 = 5,   // Rapid Tier 4 - sliding window gradual drop
		DETECT_TREND = 6,      // Trend/variance biofouling override
		DETECT_SAFETY = 7,     // Safety timeout override
	};

	// Status snapshot for DTE SWSST command (read-only diagnostic)
	struct Status {
		uint16_t threshold_air;      // Current air baseline ADC
		uint16_t threshold_water;    // Current water baseline ADC
		uint16_t threshold_current;  // Active threshold ADC
		uint16_t hysteresis;         // Hysteresis value (ADC counts)
		uint16_t last_raw_adc;       // Last raw ADC reading
		uint16_t last_filtered_adc;  // Last filtered ADC reading
		bool is_calibrated;          // Calibration valid
		bool is_underwater;          // Current state (true=underwater)
		uint32_t time_in_state_sec;  // Seconds in current state
		uint8_t surface_level;       // Last detection level (0=none, 1-5=L1-L5)
		uint16_t contrast_x10;       // Water/air contrast ratio x10 (e.g. 47 = 4.7x)
		uint16_t observed_peak;      // Highest ADC value actually observed
		uint32_t sample_delay_us;    // Current adaptive RC charge delay (µs)
	};

	static Status get_status();

	/**
     * @brief Persistent diagnostic counters — track defense-in-depth firings.
     *
     * Stored in noinit RAM with CRC. Survives soft reset (WDT, brown-out) but
     * NOT cold reset / power-on. Saturate at UINT16_MAX (no wrap) so a
     * recovered tracker exposes a true minimum count.
     *
     * Read via DTE SWSSTATS, reset via clear_diagnostics() (DTE-callable).
     * Use these to assess post-deployment health (e.g. abnormally high
     * stuck_recovery_count signals electrode degradation).
     */
	struct Diagnostics {
		uint16_t stuck_recovery_count;     // Path B6: air-collapse forced recalibrations
		uint16_t coherence_recalib_count;  // First-sample + continuous coherence recalibs
		uint16_t dive_timeout_count;       // UW_MAX_DIVE_TIME elapsed without surface
		uint16_t force_surface_count;      // Escalation force-surfaces (3 consec timeouts)
		uint16_t spike_reject_count;       // Peak resets after 10 consecutive rejects
		uint16_t peak_incoherent_count;    // Peak > water*5 coherence resets
		uint16_t saadc_init_retry_count;   // SAADC re-init after NRFX_ERROR_INVALID_STATE
	};
	static Diagnostics get_diagnostics();
	static void clear_diagnostics();

	/**
     * @brief Test mode API: start/stop SWS independently of config (DTE SWSTST command)
     *
     * start_test_mode() forces the service enabled and begins sampling.
     * stop_test_mode() halts sampling and turns off the status LED.
     * During test mode, detector_state() sets the RGB LED on state transitions:
     * BLUE = underwater, YELLOW = surface.
     */
	static void start_test_mode();
	static void stop_test_mode();
	static bool is_test_running();

	/// Default test-mode auto-stop timeout (1h). Acts as a battery-drain
	/// safety net if the operator forgets to send SWSTST,0 before deployment.
	static constexpr uint64_t TEST_TIMEOUT_DEFAULT_MS = 3600UL * 1000UL;

	/**
     * @brief Configure auto-stop timeout for test mode (DTE SWSTST,1).
     * @param ms  Timeout in milliseconds. 0 = disabled (no auto-stop).
     *
     * Default is TEST_TIMEOUT_DEFAULT_MS (1h). The configured timeout takes
     * effect immediately for the current test session (the start time is
     * captured at start_test_mode()).
     */
	static void set_test_timeout_ms(uint64_t ms);

	/// Default heartbeat WARNING thresholds (audit 2026-05 R-CODE-04).
	/// A turtle can legitimately stay submerged for hours and at the surface
	/// for days, so these are generous; the WARN is observability-only.
	static constexpr uint32_t HEARTBEAT_WARN_UW_DEFAULT_SEC = 6UL * 3600UL;     // 6h
	static constexpr uint32_t HEARTBEAT_WARN_SURF_DEFAULT_SEC = 48UL * 3600UL;  // 48h
	/// Minimum interval between repeated heartbeat WARN emissions for the
	/// same state period. Avoids log flooding.
	static constexpr uint32_t HEARTBEAT_WARN_REPEAT_SEC = 3600UL;  // 1h

	/**
     * @brief Configure heartbeat WARNING thresholds.
     * @param uw_sec   Seconds without state change in UW before WARN (0 = disabled)
     * @param surf_sec Seconds without state change in SURF before WARN (0 = disabled)
     *
     * Pure observability — no automatic action is taken. Emitted via
     * DEBUG_WARN so the warning ends up in the persistent system.log for
     * post-deployment forensics.
     */
	static void set_heartbeat_thresholds_sec(uint32_t uw_sec, uint32_t surf_sec);

	/**
     * @brief Guided calibration: LED-assisted air/water measurement
     *
     * Phase 1: GREEN flashing → user places device in AIR → samples CALIB_NUM_SAMPLES readings → GREEN solid
     * Phase 2: BLUE flashing  → user places device in WATER → samples CALIB_NUM_SAMPLES readings → BLUE solid
     * Phase 3: Save to SWS.CAL + flash. WHITE flash = success, RED flash = failure.
     *
     * Runs asynchronously via detector_state() ticks. Async notify pushes completion.
     */
	enum class CalibPhase : uint8_t {
		IDLE = 0,
		AIR_WAITING,             // GREEN flashing — waiting for stable air readings
		AIR_SAMPLING,            // GREEN fast flash — sampling air
		AIR_DONE_PAUSE,          // GREEN solid — brief pause before water phase
		WATER_WAITING,           // BLUE flashing — waiting for stable water readings
		WATER_SAMPLING,          // BLUE fast flash — sampling water
		WAIT_RESURFACE_FOR_ACK,  // BLUE solid — sampling done, waiting for resurface
		                         //   before firing GUI notify (BLE is blocked by water,
		                         //   so the ACK must be deferred until raw drops back).
		COMPLETION_PAUSE,        // WHITE/RED flash — result feedback, brief pause
		DONE,                    // Calibration complete
	};

	struct CalibResult {
		uint8_t status;  // 0=in progress, 1=success, 2=failed, 3=cancelled
		uint16_t air;
		uint16_t water;
	};

	static void start_guided_calibration();
	static void cancel_guided_calibration();
	static bool is_guided_calibration_running();
	static CalibResult get_guided_calibration_result();
	static void set_guided_calib_notify(std::function<void(const CalibResult &)> fn);
	static void clear_guided_calib_notify();

	// Async notify: push SWSST status on each sample during test mode
	static void set_status_notify(std::function<void(const Status &)> fn);
	static void clear_status_notify();

	// Callback invoked when test mode stops (to restore normal LED behavior)
	static void set_on_test_stop(std::function<void()> fn);
	static void clear_on_test_stop();

#if ENABLE_SWS_LOG
	static void set_sws_logger(Logger *logger) { m_sws_logger = logger; }
#endif

#ifdef CPPUTEST
	/// Test-only: simulate a post-reboot timestamp regression by writing
	/// directly into m_calib.last_calibration_time. Used by R-TEST-06
	/// (audit 2026-05 R-CODE-06 underflow guard verification).
	/// CRC is intentionally NOT recomputed — should_recalibrate() does not
	/// validate the CRC, and a subsequent service stop+start would invalidate
	/// the noinit data anyway.
	static void test_set_last_calibration_time(uint64_t t_sec) { m_calib.last_calibration_time = t_sec; }
	/// Test-only: expose should_recalibrate() result.
	static bool test_should_recalibrate() { return s_instance ? s_instance->should_recalibrate() : false; }

	// Reset static calibration data for test isolation. Covers ALL static
	// members so a previous test's leftovers cannot pollute the next test
	// (FlashPersistence and CoherenceCheck regress without this).
	static void reset_noinit_data() {
		memset(&m_calib, 0, sizeof(m_calib));
		m_observed_peak_adc = 0;
		m_observed_peak_crc = 0;
		memset(&m_diag, 0, sizeof(m_diag));
		m_diag_crc = 0;
		m_status = {};
		s_instance = nullptr;
		m_test_mode = false;
		m_test_mode_start_time = 0;
		m_test_timeout_ms = TEST_TIMEOUT_DEFAULT_MS;
		m_heartbeat_warn_uw_sec = HEARTBEAT_WARN_UW_DEFAULT_SEC;
		m_heartbeat_warn_surf_sec = HEARTBEAT_WARN_SURF_DEFAULT_SEC;
		m_last_heartbeat_warn_time = 0;
		m_status_notify = nullptr;
		m_on_test_stop = nullptr;
		m_calib_phase = CalibPhase::IDLE;
		m_calib_sum = 0;
		m_calib_count = 0;
		m_calib_air_result = 0;
		m_calib_water_result = 0;
		m_calib_water_success = false;
		m_calib_stable_count = 0;
		m_calib_prev_value = UINT16_MAX;
		m_calib_timeout_ticks = 0;
		m_calib_notify = nullptr;
	}
#endif

	// Calibratable interface: manual calibration via DTE $SCALW/$SCALR
	// Offset 0 = expected water ADC, offset 1 = expected air ADC
	void calibration_write(const double value, const unsigned int offset) override;
	void calibration_read(double &value, const unsigned int offset) override;
	void calibration_save(bool force) override;

	// F-SWS-2 audit fix: extend the base UWDetectorService reset to also clear
	// SWS-specific detection state. Previously, after a long cooldown the
	// first sample compared against stale m_prev_raw / m_prev_ma3 / peak
	// history from before the cooldown — could produce a spurious surface
	// emit while still wet, triggering a new cycle (battery drain).
	//
	// Preserved (intentional learning across cooldowns):
	//   - m_calib (calibration baselines)
	//   - m_observed_peak_adc (lifetime peak)
	//   - m_consecutive_dive_timeouts (safety escalation counter)
	//   - m_last_state_change_time (UW duration estimate for max-dive timeout)
	void reset_state_for_cooldown_exit() override {
		UWDetectorService::reset_state_for_cooldown_exit();  // base class iter counters
		m_prev_raw = 0;
		m_prev_ma3 = 0;
		m_recent_peak = 0;
		m_peak_adc_since_underwater = 0;
		m_l4_consecutive_below = 0;
		m_consecutive_raw_drops = 0;
		m_ma3_trend_count = 0;
		m_trend_buffer_count = 0;
		m_trend_buffer_idx = 0;
		m_consecutive_samples = 0;
		m_consecutive_invalid_adc = 0;
		m_consecutive_spike_rejects = 0;
		m_air_collapse_count = 0;
		m_surface_lockout_remaining = 0;
		m_first_sample_done = false;
		m_coherence_high_count = 0;
		m_fast_convergence_count = 0;
	}

	SWSAnalogService() : UWDetectorService("SWSAnalog"), Calibratable("SWS"), m_manual_calib("SWS") {
		s_instance = this;
		m_adc_history_idx = 0;
		m_adc_history_count = 0;
		m_last_state_change_time = 0;
		m_time_in_current_state = 0;
		m_consecutive_samples = 0;
		m_surface_readings_idx = 0;
		m_surface_readings_count = 0;
		m_prev_raw = 0;
		m_drop_reference = 0;
		m_consecutive_raw_drops = 0;
		m_l4_consecutive_below = 0;
		m_trend_buffer_idx = 0;
		m_trend_buffer_count = 0;
		m_prev_ma3 = 0;
		m_ma3_trend_start = 0;
		m_ma3_trend_count = 0;
		m_peak_adc_since_underwater = 0;
		m_recent_peak = 0;
		m_surface_lockout_remaining = 0;
		m_consecutive_dive_timeouts = 0;
		m_first_sample_done = false;
		m_fast_convergence_count = 0;
		m_coherence_high_count = 0;
		m_air_collapse_count = 0;
		m_contrast_x10 = 0;
		for (int i = 0; i < ADC_HISTORY_SIZE; i++) {
			m_adc_history[i] = 0;
		}
		for (int i = 0; i < TREND_MA_SIZE; i++) {
			m_trend_buffer[i] = 0;
		}
		for (int i = 0; i < SURFACE_BUFFER_SIZE; i++) {
			m_surface_readings[i] = 0;
		}
	}

private:
	// Live status snapshot (updated each detector_state() call)
	static Status m_status;
	// Test mode support
	static SWSAnalogService *s_instance;
	static bool m_test_mode;
	static uint64_t m_test_mode_start_time;  // PMU::get_timestamp_ms() at start_test_mode()
	static uint64_t m_test_timeout_ms;       // Auto-stop after this many ms (0 = disabled)
	// Heartbeat WARN (R-CODE-04) — pure observability, no auto-action
	static uint32_t m_heartbeat_warn_uw_sec;
	static uint32_t m_heartbeat_warn_surf_sec;
	static uint64_t m_last_heartbeat_warn_time;  // PMU sec, 0 = no warn yet for this period
	static std::function<void(const Status &)> m_status_notify;
	static std::function<void()> m_on_test_stop;
#if ENABLE_SWS_LOG
	static Logger *m_sws_logger;
#endif
	// Manual calibration data (persisted in SWS.CAL via Calibration class)
	Calibration m_manual_calib;

	// Guided calibration state
	static CalibPhase m_calib_phase;
	static uint32_t m_calib_sum;
	static uint8_t m_calib_count;
	static uint16_t m_calib_air_result;
	static uint16_t m_calib_water_result;
	static uint8_t m_calib_stable_count;
	static uint16_t m_calib_prev_value;
	static uint16_t m_calib_timeout_ticks;  // Timeout counter for guided calibration
	// Result captured at WATER_SAMPLING completion, used by WAIT_RESURFACE_FOR_ACK
	// to fire the GUI notify only after the device is back in air (BLE link up).
	static bool m_calib_water_success;
	static std::function<void(const CalibResult &)> m_calib_notify;

	// Calibration data structure (stored in noinit RAM to survive resets)
	struct CalibrationData {
		uint16_t threshold_air;          // Baseline ADC value in air (low conductivity)
		uint16_t threshold_water;        // Baseline ADC value in water (high conductivity)
		uint16_t threshold_current;      // Current active threshold
		uint16_t hysteresis_value;       // Hysteresis margin (in ADC counts)
		uint64_t last_calibration_time;  // Timestamp of last calibration
		bool is_calibrated;              // Whether initial calibration is complete
		uint16_t crc;                    // CRC for data integrity
	};

	// Static calibration data in noinit RAM section (survives reset)
	static CalibrationData m_calib __attribute__((section(".noinit")));

	// Dynamic peak ADC tracker: highest ADC value actually observed (noinit for persistence)
	// Used to cap water baseline estimates when calibrating from air with wet electrodes
	static uint16_t m_observed_peak_adc __attribute__((section(".noinit")));
	static uint16_t m_observed_peak_crc __attribute__((section(".noinit")));

	// F-SWS-7 audit fix: persist the converged adaptive sample delay across
	// soft resets. Without this, every WDT/POR reboot resets the delay to
	// UNP08 default — on a months-deployed turtle with biofouled electrode
	// (delay may have ratcheted down significantly), the first dozen
	// post-reboot samples run at the wrong delay → contrast misreports →
	// L1/L2 misfires during the convergence window.
	static uint32_t m_sample_delay_us_noinit __attribute__((section(".noinit")));
	static uint16_t m_sample_delay_us_crc __attribute__((section(".noinit")));

	// Diagnostic counters in noinit RAM (audit 2026-05 R-MON-01).
	// Separate from CalibrationData to avoid touching the existing CRC scheme.
	static Diagnostics m_diag __attribute__((section(".noinit")));
	static uint16_t m_diag_crc __attribute__((section(".noinit")));

	// Validate / initialize m_diag on service init (CRC check, zero on corruption).
	static void validate_diagnostics();
	// Recompute and store m_diag_crc — call after every counter mutation.
	static void update_diagnostics_crc();
	// Saturating increment (no wrap at UINT16_MAX) + CRC refresh.
	static void inc_diag(uint16_t &counter);

	// History buffer for moving average filter
	// Optimized: smaller buffer = faster response
	static constexpr int ADC_HISTORY_SIZE = 2;  // Was 5, now 2 for fast response
	uint16_t m_adc_history[ADC_HISTORY_SIZE];
	uint8_t m_adc_history_idx;
	uint8_t m_adc_history_count;

	// Timing tracking for safety timeouts
	uint64_t m_last_state_change_time;
	uint64_t m_time_in_current_state;
	uint64_t m_last_flash_save_time = 0;
	static constexpr uint64_t FLASH_SAVE_MIN_INTERVAL_SEC = 60;

	// Sample-cadence instrumentation (2026-05 investigation: real vs test-mode
	// detection latency). m_last_sample_ms = timestamp of previous detector_state()
	// entry. Used to log scheduler-cadence deltas so we can spot CPU contention
	// (other tasks blocking the SWS sample) vs. nominal SAMPLING_UNDER/SURF_FREQ.
	uint64_t m_last_sample_ms = 0;

	// Sample confirmation counters (for robust detection)
	uint8_t m_consecutive_samples;            // Consecutive samples in same direction
	uint8_t m_consecutive_spike_rejects = 0;  // Consecutive peak spike rejections

	// ADC silicon/frontend failure detector. Increments on every invalid
	// raw read, resets on the first valid one. If the SAADC or SWS frontend
	// permanently fails while the device is underwater, the "return
	// m_current_state" early-out in detector_state() would otherwise pin
	// the device in state=underwater forever — no TX ever. After this many
	// consecutive invalids we force state=surface and emit a WARN so the
	// device at least keeps transmitting.
	uint16_t m_consecutive_invalid_adc = 0;
	static constexpr uint16_t MAX_CONSECUTIVE_INVALID_ADC = 60;  // ~1 min at 1 Hz

	// Surface readings buffer for adaptive air baseline
	static constexpr int SURFACE_BUFFER_SIZE = 10;
	uint16_t m_surface_readings[SURFACE_BUFFER_SIZE];
	uint8_t m_surface_readings_idx;
	uint8_t m_surface_readings_count;

	// === MULTI-LEVEL SURFACE DETECTION STATE ===

	// Level 1 & 2: Fast raw drop detection
	uint16_t m_prev_raw;               // Previous raw ADC value
	uint16_t m_drop_reference;         // Raw value when consecutive drops started
	uint16_t m_consecutive_raw_drops;  // Count of consecutive raw drops

	// Level 4: require 2 consecutive samples below water * (1 - L4_DROP_PERCENT/100)
	// before firing — protects against biofouling stage transitions where the
	// water baseline is stale (one sample past the threshold isn't enough
	// to claim surfacing if the baseline hasn't adapted yet).
	uint8_t m_l4_consecutive_below;

	// Level 3: Trend MA3 detection
	static constexpr int TREND_MA_SIZE = 3;
	uint16_t m_trend_buffer[TREND_MA_SIZE];  // Ring buffer for MA3 computation
	uint8_t m_trend_buffer_idx;
	uint8_t m_trend_buffer_count;
	uint16_t m_prev_ma3;         // Previous 3-sample moving average
	uint16_t m_ma3_trend_start;  // MA3 value when trend started
	uint16_t m_ma3_trend_count;  // Consecutive MA3 decreases

	// Level 4 & 5: Relative and cumulative drop
	uint16_t m_peak_adc_since_underwater;

	// Level 1: Recent peak for fast drop detection (decays with drift)
	uint16_t m_recent_peak;  // Tracks recent max, slowly decays to follow drift

	// Fast convergence: aggressive alpha for first N water samples when water is estimated
	uint8_t m_fast_convergence_count;  // Counts samples during fast convergence phase

	// Safety
	uint32_t m_surface_lockout_remaining;
	uint8_t m_consecutive_dive_timeouts;  // Escalation: force surface after N consecutive timeouts

	// First-sample coherence check
	bool m_first_sample_done;

	// Continuous coherence: consecutive samples exceeding water×2 before adapting
	uint8_t m_coherence_high_count;

	// Stuck-state recovery: count consecutive surface samples with air baseline
	// at/below AIR_BASELINE_FLOOR. Triggers a forced recalibration when the
	// periodic Air recalib path collapses air to noise level (dry electrodes).
	uint8_t m_air_collapse_count;

	// Configuration parameters (loaded from config store)
	uint16_t m_hysteresis_percent;
	uint32_t m_calib_interval_sec;
	uint32_t m_max_dive_time_sec;
	uint32_t m_min_surface_time_sec;

	// New optimized parameters for fast surface detection
	uint8_t m_threshold_ratio_percent;  // Position of threshold (35% = closer to air)
	uint8_t m_alpha_percent;            // EMA factor for water baseline (19 = 0.19)

	// Adaptive sample delay (in µs) — auto-adjusted based on contrast
	uint32_t m_sample_delay_us;  // Current delay
	uint32_t m_delay_min_us;     // Configurable floor (UNP09)
	uint32_t m_delay_max_us;     // Configurable ceiling (UNP10)

	// Cached contrast ratio (water/air × 10), updated once per detector_state() call
	uint16_t m_contrast_x10;

	/**
     * @brief Read analog ADC value from SWS channel
     * @return Raw ADC value (0-4095 for 12-bit, 0-16383 for 14-bit)
     */
	uint16_t read_analog_sws();

	/**
     * @brief Initialize or validate calibration data from noinit RAM
     * @return true if calibration data is valid, false if needs initialization
     */
	bool validate_calibration_data();

	/**
     * @brief Perform initial calibration in air (baseline)
     * Called on first boot or after calibration data corruption
     */
	void calibrate_air_baseline();

	/**
     * @brief Update water baseline when underwater is confirmed
     * @param value Current ADC reading
     */
	void calibrate_water_baseline(uint16_t value);

	/**
     * @brief Update dynamic threshold based on current calibration
     * Uses midpoint between air and water with hysteresis
     */
	void update_dynamic_threshold();

	/**
     * @brief Add value to history buffer and get filtered result
     * @param value New ADC reading to add
     * @return Filtered value (moving average)
     */
	uint16_t add_to_history_and_filter(uint16_t value);

	/**
     * @brief Check if ADC value is within valid range
     * @param value ADC reading to validate
     * @return true if value is valid (not saturated or out of range)
     */
	bool is_value_valid(uint16_t value) const;

	/**
     * @brief Adjust sample delay based on current contrast ratio.
     * Reduces delay when biofouling degrades contrast, increases when clean.
     */
	void adjust_sample_delay();

	/**
     * @brief Check if recalibration is needed based on time interval
     * @return true if calibration interval has elapsed
     */
	bool should_recalibrate() const;

	/**
     * @brief Check safety timeout conditions
     * @param current_state Current underwater state
     * @return true if timeout override is needed
     */
	bool check_safety_timeouts(bool current_state);

	/**
     * @brief Save calibration data to flash (survives hard resets)
     * Called on state transitions and significant calibration changes.
     */
	void save_calibration_to_flash();
	void save_calibration_to_flash_debounced();

	/**
     * @brief Load calibration data from flash
     * @return true if valid calibration was loaded from flash
     */
	bool load_calibration_from_flash();

protected:
	/**
     * @brief Detect current state (underwater or surface)
     * @return true if underwater detected, false if at surface
     */
	bool detector_state() override;

	/**
     * @brief Check if service is enabled in configuration
     * @return true if SWS analog detection is enabled
     */
	bool service_is_enabled() override;

	/**
     * @brief Override scheduling: 1s during guided calibration, normal otherwise
     */
	unsigned int service_next_schedule_in_ms() override;

	/**
     * @brief Service initialization - load config and validate calibration
     */
	void service_init() override;
};
