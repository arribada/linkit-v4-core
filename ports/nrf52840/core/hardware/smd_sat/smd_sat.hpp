/**
 * @file smd_sat.hpp
 * @brief SMD satellite device driver — KineisDevice interface + state machine.
 */

#pragma once

#include <cstdint>

#include "smd_sat_cmd.hpp"
#include "kineis_device.hpp"
#include "scheduler.hpp"
#include "filesystem.hpp"

#define SMD_STATE_CHANGE(x, y)                       \
	do {                                             \
		DEBUG_TRACE("SmdSat::SMD_STATE_CHANGE: " #x " -> " #y ); \
		m_state = y;                                 \
		state_ ## x ##_exit();                       \
		state_ ## y ##_enter();                      \
	} while (0)

#define SMD_STATE_EQUAL(x) \
	(m_state == x)

#define SMD_STATE_CALL(x)   \
	do {                    \
		state_ ## x();      \
	} while (0)


class SmdSat : public KineisDevice {

private:
	enum SmdSatState {
		stopped,
		starting,
		powering_on,
		load_kmac,
		idle_pending,
		idle,
		transmit_pending,
		transmitting,
		error
	};

	// Command layer (SPI or AT, selected at compile time)
	SmdSatCmd& m_cmd;

	// Top-level state
	Scheduler::TaskHandle m_task;
	SmdSatState m_state;
	unsigned int m_state_counter;
	unsigned int m_next_delay;
	bool m_stopping;
	bool is_kmac_profil_loaded = false;       ///< Routes idle_pending → load_kmac for boot config (TCXO, LPM)
	bool m_needs_explicit_kmac_load = true;   ///< True when RCONF changed and explicit load_kmac_profil SPI command is needed.
	bool m_kmac_blind_pushed = false;         ///< True while the last pushed KMAC profile was BLIND — forces a BASIC push when blind is turned off within a session.
	                                          ///< False when RCONF unchanged — STM32 auto-inits MAC from flash at POR.
	bool m_credentials_written = false;
	bool m_rconf_recovery_attempted = false;

	// (m_modulation is now loaded from ParamID::ARGOS_CACHED_MODULATION at
	//  construction and updated only in the credentials-dirty path. No runtime
	//  readback is needed on the TX critical path.)

	/// @brief Consecutive error count — after MAX errors, enter long cooldown to prevent SPI spam.
	/// Aligned with ArgosTxService::DEVICE_ERROR_MAX_CONSECUTIVE (3) so the SmdSat-level
	/// autofallback engages at the same threshold as the ArgosTx session suspension. If
	/// SmdSat threshold > ArgosTx threshold, ArgosTx stops calling send() and SmdSat
	/// never reaches its own threshold, so the SAFE flag is never persisted.
	unsigned int m_error_count = 0;
	static constexpr unsigned int SMD_MAX_CONSECUTIVE_ERRORS = 3;
	static constexpr unsigned int SMD_ERROR_COOLDOWN_MS = 30 * 60 * 1000;  ///< 30 min cooldown
	uint64_t m_cooldown_until = 0;  ///< Timestamp (ms) until which SMD operations are blocked

	/// @brief Die-temperature threshold (°C) below which we keep the configured
	/// TCXO warmup on the first TX instead of forcing 0 ms. Deep cold dives
	/// followed by quick surfacing can leave the TCXO too cold for instant
	/// frequency stability — better to spend one warmup than waste an off-frequency
	/// TX. nRF52840 die temp sensor has ±5 °C accuracy, so this is a coarse gate.
	static constexpr int TCXO_SKIP_TEMP_THRESHOLD_C = 5;

	// =====================================================================
	// Degraded-mode (autofallback to SAFE timings) state.
	// Only emitted into the binary when SMDSAT_AUTOFALLBACK_ENABLED == 1.
	// On SMD_MAX_CONSECUTIVE_ERRORS the driver flips g_smdsat_use_safe_timings
	// to true and persists the state via ParamID::SMD_DEGRADED_MODE. After
	// SAFE_RETEST_MIN_TX successful TXes AND at least m_safe_trust_window_hours
	// hours in SAFE, FAST is retested; if it re-fails the window doubles
	// (cap 24h) so the device stops flapping under sustained marginal
	// conditions.
	// =====================================================================
#if SMDSAT_AUTOFALLBACK_ENABLED
	uint64_t m_safe_mode_since_ms = 0;
	unsigned int m_safe_mode_tx_count = 0;
	unsigned int m_safe_trust_window_hours = 1;
	bool m_degraded_mode_loaded = false;

	// Saved at state_transmitting_enter so we can quantify how much of the
	// polling budget was consumed by SPI cascade failures before an external
	// cancellation (dive / service_cancel) pre-empted the natural timeout.
	unsigned int m_initial_tx_state_counter = 0;

	static constexpr unsigned int SAFE_RETEST_MIN_TX = 20;
	static constexpr unsigned int SAFE_TRUST_WINDOW_MAX_H = 24;
	/// @brief Fraction of polling budget consumed during state_transmitting
	/// without success that we treat as evidence of an SPI cascade failure
	/// when the TX is cancelled externally. 25% of initial counter — with
	/// the FAST profile counter ≈ 67, this is ~17 failed polls (~3-5 s of
	/// WARN cascade) before counting as 1 autofallback error. Tighter than
	/// the natural state_transmitting timeout (counter == 0, ~13-15 s).
	// F-SAT-6 audit fix: bumped 1/4 (25%) → 1/2 (50%). The 25% threshold
	// was too lax for turtle behavior — a dive event 3-5 s into a TX poll
	// (very common on shallow rest dives) was counted as a cascade failure,
	// tripping autofallback to SAFE timings unnecessarily. With 50%, the
	// gate requires ~6-8 s of failed polling before counting — clearly
	// distinguishes a real SPI stall from a normal dive cancellation.
	static constexpr unsigned int TX_CASCADE_FAILURE_NUMERATOR = 1;
	static constexpr unsigned int TX_CASCADE_FAILURE_DENOMINATOR = 2;

	void degraded_mode_load_if_needed();
	void degraded_mode_engage();
	void degraded_mode_note_success();
	void degraded_mode_note_tx_cancelled_during_cascade();
#endif

	// TX state
	KineisPacket m_tx_buffer;
	KineisPacket m_packet_buffer;
	SmdArgosModulation m_modulation;
	std::string m_pending_rconf;  // Deferred RCONF for next power-on
	unsigned int m_tx_power;
	double      m_tx_freq;
	bool        m_is_first_tx;
	uint32_t    m_tcxo_warmup_time;
	uint64_t    m_tx_trace_start_ms = 0;  ///< Anchor for [TXTRACE +N ms] timing logs; reset in send()
	uint8_t     m_lpm_mode;  // SMD LPM bitmap written at every boot
	bool        m_wkup_lowered = false;  // True if state_idle_enter dropped WKUP (=> idle_exit must re-raise + wait for wake)

	/// @brief Timestamp (ms) of the last full power-off (shutdown). Used by
	/// state_powering_on to skip the redundant VDD discharge wait when the
	/// dive lasted long enough for natural decay (>5 s). Saves ~50 ms FAST /
	/// ~100 ms SAFE on the first Doppler at surface — the typical sealed
	/// turtle case where dives are minutes long. 0 = never powered off.
	uint64_t    m_last_power_off_ms = 0;
	static constexpr uint64_t VDD_NATURAL_DISCHARGE_MS = 5000;  ///< Dive >5 s → VDD fully decayed

	// New firmware version (cached after DFU)
	std::string m_new_firmware_version;

	// ── Argos MAC message counter (MC) hold across message repetitions (RSPB) ──
	// Keep every retransmission of the SAME message on one MC. Policy: leave MC
	// untouched on a NEW message (no SPI op → first-TX timing preserved, critical
	// for surface-doppler); on a REPEAT (identical payload) decrement MC by 1 so
	// the copy reuses the previous identical TX's counter. The tri-state latch
	// avoids re-probing a module firmware that lacks the command (graceful
	// degradation — MC then left auto-managed by the SMD).
	enum class McSupport : uint8_t { UNKNOWN, YES, NO };
	McSupport    m_mc_support = McSupport::UNKNOWN;
	KineisPacket m_mc_last_payload;     // payload of the previous TX (repeat detection)
	void apply_message_counter_hold();  // called before initiate_tx in state_transmit_pending

	void power_off();
	void power_on();
	void power_on_blocking();  // Synchronous boot: power + reset + SPI init + wait for ping + VPA release
	bool write_credentials_from_config();

	// State machine functionality
	void state_machine(bool use_scheduler=true);
	void state_starting();
	void state_starting_enter();
	void state_starting_exit();
	void state_stopped();
	void state_stopped_enter();
	void state_stopped_exit();
	void state_error();
	void state_error_enter();
	void state_error_exit();
	void state_powering_on();
	void state_powering_on_enter();
	void state_powering_on_exit();
	void state_load_kmac_enter();
	void state_load_kmac_exit();
	void state_load_kmac();
	void state_idle_pending();
	void state_idle_pending_enter();
	void state_idle_pending_exit();
	void state_idle();
	void state_idle_enter();
	void state_idle_exit();
	void state_transmit_pending();
	void state_transmit_pending_enter();
	void state_transmit_pending_exit();
	void state_transmitting();
	void state_transmitting_enter();
	void state_transmitting_exit();

public:
	SmdSat(SmdSatCmd& cmd, unsigned int idle_shutdown_timeout_ms = 1000);
	~SmdSat();
	void send(const KineisModulation mode, const KineisPacket& packet, const unsigned int size_bits) override;
	void stop_send() override;
	void start_receive(const KineisModulation mode) override;
	bool stop_receive() override;
	void set_frequency(double freq_mhz) override;
	void set_tcxo_warmup_time(unsigned int time) override;
	void set_tx_power(unsigned int power) override;
	void set_lpm_mode(uint8_t lpm_bitmap);

	void set_credentials(unsigned int dec_id, unsigned int address,
	                     const std::string& seckey, const std::string& radioconf) override;
	void read_credentials(unsigned int *dec_id, unsigned int *address,
	                      std::string *seckey, std::string *radioconf) override;
	static void shutdown(void);

	// DFU Public API
	void power_off_immediate();

	bool dfu_enter();
	bool dfu_exit();
	bool is_dfu_mode() const { return m_cmd.is_dfu_mode(); }

	bool dfu_get_bootloader_info(SmdDfuInfo *info);

	SmdDfuResponse firmware_update(const uint8_t *firmware, size_t size,
	                               void (*progress_callback)(uint8_t percent) = nullptr);

	SmdDfuResponse firmware_update(File *file, size_t size, uint32_t stm32_crc32,
	                               void (*progress_callback)(uint8_t percent) = nullptr);

	SmdDfuResponse firmware_update_from_file(const std::string& filepath,
	                                         void (*progress_callback)(uint8_t percent) = nullptr);

	std::string get_firmware_version();
	std::string get_new_firmware_version() const { return m_new_firmware_version; }
	std::string smd_spi_test();

	/// @brief Start Continuous Wave transmission for RF testing / certification.
	/// @param freq_hz    Carrier frequency in Hz (must be in a licensed SMD band).
	/// @param power_dbm  TX power in dBm.
	/// @param duration_s Duration in seconds (0 = continuous until cw_stop()).
	/// @return true on success, false if module not ready or AT+CW failed.
	bool cw_start(uint32_t freq_hz, uint16_t power_dbm, uint16_t duration_s = 0);

	/// @brief Stop any active Continuous Wave transmission.
	bool cw_stop();

	// Runtime modulation switching (RCONF + save + KMAC reload, no power cycle)
	bool switch_modulation(KineisModulation mode, const std::string& rconf_hex) override;
	KineisModulation get_current_modulation() const override;
	unsigned int cooldown_remaining_ms() const override;
};
