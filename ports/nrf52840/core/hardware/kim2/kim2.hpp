#pragma once

/**
 * @file kim2.hpp
 * @brief KIM2 satellite module device driver (AT commands over UART).
 *
 * Implements the KineisDevice interface for the CLS KIM2 module.
 * State machine: power_off → power_on → init (ping, ID, RCONF, KMAC) → idle → transmit.
 * Power-managed: module is powered off between TX sessions to save battery.
 */

#include "kim2_comm.hpp"
#include "kineis_device.hpp"
#include "nrfx_uarte.h"
#include "scheduler.hpp"
#include <atomic>

class KIM2Device : public KIM2CommEventListener, public KineisDevice {
private:
	KIM2Comm m_kim2_comm;  ///< UART AT command layer

public:
	/// @throws ErrorCode if module does not respond to ping at first power-on.
	KIM2Device();
	~KIM2Device();

	/// @brief Pack payload, stuff bits, queue for TX.  Powers on module if needed.
	/// @param mode       Modulation type (LDK, LDA2, VLDA4).
	/// @param packet     Raw payload bytes.
	/// @param size_bits  Payload size in bits.
	void send(const KineisModulation mode, const KineisPacket& packet, const unsigned int size_bits) override;

	/// @brief Cancel any pending TX.
	void stop_send() override;

	/// @brief Not implemented on KIM2.
	void start_receive(const KineisModulation mode) override;

	/// @brief Not implemented on KIM2.
	/// @return Always false.
	bool stop_receive() override;

	/// @brief No-op — KIM2 frequency is set via RCONF.
	void set_frequency(double freq_mhz) override;

	/// @brief No-op — KIM2 manages TCXO warmup internally.
	void set_tcxo_warmup_time(unsigned int ms) override;

	// Bridge/passthrough mode: direct USB ↔ UART access for raw AT commands
	bool start_bridge(KIM2Comm::PassthroughCallback rx_callback);
	void stop_bridge();
	bool is_bridge_active() const { return m_bridge_active; }
	bool bridge_send(const uint8_t* data, size_t len);
	void bridge_process_rx();  // Call periodically to pump UART RX

	/// @brief VLDA4 regulatory gate — KIM2 only accepts VLDA4 at 27 dBm.
	/// Set false by state_init / switch_modulation when the module reports
	/// VLDA4 at a different rf_level. While false, any VLDA4 TX or switch is
	/// rejected to prevent out-of-spec emissions. Reset to true on each boot.
	bool is_vlda4_allowed() const { return m_vlda4_allowed; }

	/// @brief Read KIM2 module identifier via AT+ID=?.
	/// The KIM2 RUI does not expose a dedicated firmware-version command,
	/// so we return the hardware ID string as the identity reported to the host.
	/// Module must be powered on. Returns empty string on failure.
	std::string get_firmware_version();

private:
	/// @brief KIM2 state machine states.
	enum KIM2ManagerState {
		power_off,
		power_on,
		init,
		idle,
		transmit,
		error
	};

	/// @name Top-level state
	/// @{
	Scheduler::TaskHandle m_task;           ///< Scheduler task for state machine ticks
	KIM2ManagerState      m_state;          ///< Current state
	bool                  m_stopping;       ///< True when stop requested
	std::atomic<bool>     m_cmd_is_ok;      ///< Set by ISR on +OK response
	std::atomic<bool>     m_is_error;       ///< Set by ISR on +ERROR or timeout
	struct Timeout {
		Scheduler::TaskHandle handle;
	} m_timeout;
	/// @}

	/// @brief Bridge mode active flag — pauses state machine, routes raw UART.
	bool m_bridge_active = false;

	/// @brief VLDA4 regulatory gate (see is_vlda4_allowed()). Reset to true at
	///        every state_init entry so a newly uploaded compliant RCONF is
	///        re-tried on the next boot instead of being latched off forever.
	bool m_vlda4_allowed = true;

	/// @name TX state
	/// @{
	KineisPacket m_tx_buffer;                ///< Hex string queued for AT+TX
	KineisPacket m_packet_buffer;            ///< Incoming packet from send()
	KineisModulation m_tx_mode;              ///< Modulation for current TX
	KineisModulation m_current_rconf_mode;   ///< Last RCONF modulation written
	std::atomic<bool> m_tx_done;             ///< Set by ISR on +TX= response
	unsigned int m_tx_poll_counter;          ///< Remaining TX poll ticks before timeout

	/// @brief Two-phase async AT+TX progress (replaces the old 5 s busy-wait for +OK).
	///        AWAIT_ACK: AT+TX sent, state_transmit() polls for the +OK ACK
	///        (m_cmd_is_ok), bounded by m_tx_ack_deadline_ms (KIM2_TX_ACK_TIMEOUT_MS).
	///        AWAIT_TX : +OK seen (command accepted, NOT yet emitted), polling for
	///        +TX=<status> (m_tx_done), bounded by the existing 60 s initiate_timeout.
	enum class TxPhase { AWAIT_ACK, AWAIT_TX };
	TxPhase  m_tx_phase = TxPhase::AWAIT_ACK;  ///< Current AT+TX poll phase
	uint64_t m_tx_ack_deadline_ms = 0;         ///< Wall-clock deadline (ms) for the +OK ACK

	/// @brief True while SAT_EXTWAKEUP has been released for the duration of a
	///        BLIND burst, so the resume path knows it is the one holding the
	///        pin down and no other path raises it behind our back.
	bool m_tx_wkup_lowered = false;
	uint64_t m_tx_started_ms = 0;  ///< horodatage de l'AT+TX
	void release_wkup_for_burst();
	void resume_wkup_after_burst(bool wait_for_wake, const char *reason);
	/// @}

	/// @name State machine
	/// @{
	void state_machine();                              ///< Dispatch to current state handler
	void run_state_machine(uint16_t delay_ms = 100);   ///< Schedule next state machine tick
	// State handlers: each state has enter/exit/tick functions
	void state_power_off();       void state_power_off_enter();   void state_power_off_exit();
	void state_power_on();        void state_power_on_enter();    void state_power_on_exit();
	void state_init();            void state_init_enter();        void state_init_exit();
	void state_idle();            void state_idle_enter();        void state_idle_exit();
	void state_transmit();        void state_transmit_enter();    void state_transmit_exit();
	void state_error();           void state_error_enter();       void state_error_exit();
	/// @}

	/// @name UART event handlers (ISR context → atomic flags)
	/// @{
	void react(const KIM2CommEventRespOk&) override;
	void react(const KIM2CommEventTxDone&) override;
	void react(const KIM2CommEventRespError&) override;
	void react(const KIM2CommEventUartError&) override;
	/// @}

	/// @name Runtime modulation switching (KineisDevice interface)
	/// @{

	/// @brief Switch RCONF + KMAC for a different modulation.
	/// @param mode       Target modulation.
	/// @param rconf_hex  32-char hex RCONF string.
	/// @return true on success.
	bool switch_modulation(KineisModulation mode, const std::string& rconf_hex) override;

	/// @return Current RCONF modulation.
	KineisModulation get_current_modulation() const override;

	/// @brief Return the last ID/ADDR read from the KIM2 module during
	///        state_init, plus the last decoded RCONF info from AT+RCONF=?.
	/// @note  KIM2 has no per-module SECKEY AT command (unlike SMD), so
	///        @p seckey is always empty. @p radioconf carries the decoded
	///        "freq_min,freq_max,mod_type,rf_level" string from the module
	///        (diagnostic use — not the encrypted hex written via AT+RCONF=).
	void read_credentials(unsigned int *dec_id, unsigned int *address,
	                      std::string *seckey, std::string *radioconf) override;
	/// @brief Program the configured RCONF, read it back, and re-seed the
	/// modulation cache (ARGOS_CACHED_MODULATION). Synchronous power-cycle if the
	/// module is off. Called on a master-RCONF PARMW edit / force-SATVF so the
	/// first TX is sized for the right modulation (no aborted first cycle).
	bool resync_rconf_cache() override;
	/// @}

	/// @name Internal helpers
	/// @{

	/// @brief Send AT command and busy-wait for response (up to timeout_ms).
	/// @param cmd         AT command type.
	/// @param params      Optional parameter string.
	/// @param timeout_ms  Max wait in ms (default 1000).
	/// @return true if +OK received, false on timeout or +ERROR.
	// Load the KMAC MAC profile honoring ARGOS_BLIND_EN: BLIND (AT+KMAC=2,<ctx>)
	// with graceful fallback to BASIC (AT+KMAC=1) if the module rejects it.
	bool load_kmac();
	bool send_AT(KIM2::ATCmd cmd, const std::optional<std::string>& params = std::nullopt,
	             uint16_t timeout_ms = 1000);

	/// @brief Power on module and start state machine (no-op if already running).
	void start_device();

	/// @brief Immediate power off — cancel all tasks, uninit UART, cut power.
	void power_off_immediate();

	void cancel_timeout();                              ///< Cancel pending AT response timeout
	void initiate_timeout(unsigned int timeout_ms = 1000); ///< Schedule AT response timeout
	void on_timeout();                                   ///< Timeout handler — sets m_is_error

	/// @brief Read the RCONF configured for a modulation from ConfigStore.
	/// @note  Honors ARGOS_ADAPTIVE_MODULATION: when ON, reads per-mode RCONF
	///        (ARGOS_RADIOCONF_LDK / _LDA2 / _VLDA4); when OFF, reads the master
	///        ARGOS_RADIOCONF. Returns empty if nothing is configured.
	std::string load_rconf_for_mode(KineisModulation mode);

	/// @brief Write @p rconf_hex, read back via AT+RCONF=?, then enforce the
	///        KIM2 regulatory constraint: VLDA4 is only allowed at 27 dBm.
	///        Updates @c m_vlda4_allowed. Module must already be powered on
	///        and UART subscribed.
	/// @param rconf_hex       32-char hex RCONF to program.
	/// @param expected_mode   Modulation the caller thinks this RCONF encodes.
	///                        Used for diagnostic logging only.
	/// @param out_decoded     Optional: receives the decoded +RCONF= fields.
	/// @return true if RCONF applied and (when VLDA4) at 27 dBm. False on AT
	///         failure or VLDA4-at-wrong-power (caller must fall back).
	bool write_and_validate_rconf(const std::string& rconf_hex,
	                              KineisModulation expected_mode,
	                              KIM2::RConfDecoded* out_decoded = nullptr);

	/// @note The modulation-name decode + read-back validation moved to the pure,
	///       host-testable KIM2::mod_from_name / KIM2::verify_modulation in
	///       kim2_modulation.hpp.

	/// @brief Persist m_current_rconf_mode to ParamID::ARGOS_CACHED_MODULATION so
	///        the next cold boot loads the real modulation instead of defaulting
	///        to LDA2 (which made the first scheduling cycle build a wrong-mod
	///        packet on an LDK/VLDA4 master). Mirrors SmdSat's cached-modulation
	///        persistence. No-op if configuration_store is unavailable.
	void cache_current_modulation();
	/// @}
};
