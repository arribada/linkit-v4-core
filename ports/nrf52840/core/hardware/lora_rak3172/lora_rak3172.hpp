/**
 * @file lora_rak3172.hpp
 * @brief LoRa RAK3172-SiP device driver — KineisDevice interface + state machine.
 */

#pragma once

#include "lora_rak3172_comm.hpp"
#include "kineis_device.hpp"
#include "scheduler.hpp"
#include <string>

/**
 * LoRa RAK3172-SiP device driver.
 *
 * Implements the KineisDevice interface to integrate with ArgosTxService.
 * Communicates via UART AT commands (RUI3 firmware) with the RAK3172-SiP module.
 *
 * State machine:
 *   power_off → power_on → configure → joining → idle → transmit → idle → standby
 *                                                  ↑                         │
 *                                                  └── wake (~10ms) ─────────┘
 *                                                         ↓
 *                                                       error → power_off
 *
 * After first TX cycle, the module stays in standby (LPM Stop2, ~1.7µA)
 * instead of full power-off. Wake from standby via UART is ~10ms vs ~2.5s reboot.
 */
class LoRaDevice : public LoRaCommEventListener, public KineisDevice {
private:
	LoRaComm m_lora_comm;

public:
	LoRaDevice();
	~LoRaDevice();

	// KineisDevice interface
	void send(const KineisModulation mode, const KineisPacket &packet, const unsigned int size_bits) override;
	void stop_send() override;
	void start_receive(const KineisModulation mode) override;
	bool stop_receive() override;
	void set_frequency(double freq_mhz) override;
	void set_tcxo_warmup_time(unsigned int ms) override;

	/// @brief Hard shutdown — cut SAT_PWR_EN and deinit UART. Next send() goes
	/// through the full power_on → configure path (≈2 s wake vs 10 ms standby).
	/// Overrides KineisDevice::power_off_immediate() for service-driven shutdown
	/// (e.g. LORA_POWER_OFF_UNDERWATER compile flag).
	void power_off_immediate() override;

	/// @brief Pre-boot the RAK3172 so the next send() dispatches with only a
	/// ~10 ms wake from Stop2 standby instead of ~3 s boot+configure. No-op
	/// if the module is already running. Used by LoRaTxService to warm up
	/// the module during an underwater phase when no cooldown is active.
	void warm_up_for_tx() override;

	// LoRa-specific public API
	bool is_joined() const { return m_joined; }

	/// @brief Read credentials from RAK3172 module for verification.
	struct LoRaCredentials {
		std::string deveui;
		std::string appeui;
		std::string appkey;
		std::string devaddr;
		uint8_t njm = 0;
		bool read_ok = false;
	};
	LoRaCredentials read_lora_credentials();

	/// @brief Write credentials from the config store to the RAK3172 module.
	/// Sends AT_SET_NJM, DEVEUI, APPEUI/APPKEY (OTAA) or DEVADDR (ABP).
	/// Module must be powered on (idle/standby/configure state).
	/// @return true on success, false if any AT write fails or module is off.
	bool write_credentials_from_config();

	/// @brief Read RAK3172 firmware version (AT+VER=?).
	/// Module must be powered on. Returns empty string on failure.
	std::string get_firmware_version();

	/// @brief Re-read all LORA_* params from the config store and, if any
	/// differ from the cached `m_config`, schedule a full module reconfigure
	/// at the next idle entry. Called on every PARMW completion (`CONFIG_UPDATED`
	/// action) so that runtime credential / band / DR changes propagate without
	/// requiring a device reset.
	///
	/// Safety: if module is currently busy (transmit/joining/configure/power_on/error),
	/// the flag is recorded and applied at the next idle entry — the in-flight
	/// operation is never interrupted. Returns true if a change was detected.
	bool reload_config_if_changed();

	/// @brief Start Continuous Wave transmission for RF testing / certification.
	/// @param freq_hz    Carrier frequency in Hz (must be in a licensed LoRa band).
	/// @param power_dbm  TX power in dBm (0-22 for RAK3172).
	/// @param duration_s Duration in seconds (1-65535, RUI3 requires non-zero).
	/// @return true if AT+CW accepted.
	bool cw_start(uint32_t freq_hz, uint16_t power_dbm, uint16_t duration_s);

	/// @brief Stop active Continuous Wave — RUI3 stops when duration expires;
	/// issuing ATZ (software reset) is the reliable way to abort immediately.
	bool cw_stop();

	// Bridge/passthrough mode: direct USB ↔ UART access for RUI3 AT commands
	bool start_bridge(LoRaComm::PassthroughCallback rx_callback);
	void stop_bridge();
	bool is_bridge_active() const { return m_bridge_active; }
	bool bridge_send(const uint8_t *data, size_t len);
	void bridge_process_rx();  // Call periodically to pump UART RX

	// LoRa configuration (set before power_on or during idle)
	struct LoRaConfig {
		std::string deveui;                   // 16 hex chars - empty = read from module
		std::string appeui;                   // 16 hex chars
		std::string appkey;                   // 32 hex chars
		std::string devaddr;                  // 8 hex chars (ABP only)
		std::string appskey;                  // 32 hex chars (ABP only)
		std::string nwkskey;                  // 32 hex chars (ABP only)
		uint8_t njm = LoRa::DEFAULT_NJM;      // 0=ABP, 1=OTAA
		uint8_t band = LoRa::DEFAULT_BAND;    // 4=EU868
		uint8_t dr = LoRa::DEFAULT_DR;        // Data rate
		uint8_t adr = LoRa::DEFAULT_ADR;      // ADR enable
		uint8_t txp = LoRa::DEFAULT_TXP;      // TX power index
		uint8_t cfm = LoRa::DEFAULT_CFM;      // 0=unconfirmed, 1=confirmed
		uint8_t fport = LoRa::DEFAULT_FPORT;  // Application port
		uint8_t device_class = 'A';           // A, B, or C
		uint8_t lp_mode =
		    1;  // 0=shutdown (0µA, 2.5s wake), 1=standby (Stop1 ~3µA, ~10 ms wake — Stop2 would be ~1.7µA but UART1 cannot wake from it on this PCB, see configure step 13)
	};

	LoRaConfig m_config;

private:
	// State machine
	enum State {
		power_off,
		power_on,
		configure,
		joining,
		idle,
		standby,  // Module powered, LPM Stop2 (~1.7µA), wake via UART ~10ms
		transmit,
		error
	};

	// Top-level state
	Scheduler::TaskHandle m_task;
	State m_state;
	volatile bool m_cmd_is_ok;
	volatile bool m_is_error;
	volatile bool m_joined;
	volatile bool m_join_failed;
	volatile bool m_tx_done;

	struct Timeout {
		Scheduler::TaskHandle handle;
	} m_timeout;

	// TX state
	std::string m_packet_buffer;  // Hex-encoded payload waiting to be sent
	bool m_join_attempted;        // A join was already tried for the packet currently buffered

	// Join window sizing. The module runs the whole retry cycle itself once
	// AT+JOIN is accepted, so the firmware timeout must OUTLAST that cycle or it
	// pre-empts the very procedure it supervises.
	//
	// Per attempt the module spends: JoinRequest airtime (~0.4 s at DR3, up to
	// ~1.5 s at DR0) + RX1 at JOIN_ACCEPT_DELAY1 = 5 s + RX2 at
	// JOIN_ACCEPT_DELAY2 = 6 s. So an attempt is not free until ~7 s, and
	// JOIN_INTERVAL_S is the gap BETWEEN attempts, not the attempt budget.
	// Cycle = attempts * 7 s + (attempts - 1) * interval.
	// With 8 and 10 that is 56 + 70 = ~126 s, which the old flat 90 ms timeout
	// could not cover: it fired mid-cycle, the FSM re-entered joining, and the
	// module answered the second AT+JOIN with AT_ERROR because it was still
	// busy. That is the "error response type=1" seen in the field.
	static constexpr unsigned int JOIN_ATTEMPTS = 8;
	static constexpr unsigned int JOIN_INTERVAL_S = 10;
	static constexpr unsigned int JOIN_ATTEMPT_COST_S = 7;     ///< airtime + RX1 + RX2
	static constexpr unsigned int JOIN_TIMEOUT_MARGIN_S = 20;  ///< module bookkeeping + UART latency
	static constexpr unsigned int JOIN_WINDOW_MS =
	    ((JOIN_ATTEMPTS * JOIN_ATTEMPT_COST_S) + ((JOIN_ATTEMPTS - 1) * JOIN_INTERVAL_S) + JOIN_TIMEOUT_MARGIN_S)
	    * 1000;

	// How often, while waiting, to ask the module for the truth with AT+NJS=?
	// instead of trusting the single +EVT:JOINED we may never see.
	static constexpr unsigned int JOIN_NJS_POLL_TICKS = 20;  ///< 20 * 500 ms = 10 s
	unsigned int m_join_wait_ticks;

	// Configuration tracking
	unsigned int m_config_step;
	bool m_is_configured;           // true after first successful full configuration
	unsigned int m_power_on_retry;  // AT ping retry counter during power_on
	bool m_nwm_reboot_pending;      // Waiting for module reboot after AT+NWM=1

	// Error retry: allow transient errors (RF interference, UART glitch) before power_off
	static constexpr uint8_t MAX_CONSECUTIVE_ERRORS = 2;
	uint8_t m_consecutive_errors;

	// Set by reload_config_if_changed() / write_credentials_from_config() when
	// the persisted LoRa config has been edited at runtime. state_idle picks it
	// up and forces a power_off → power_on → full configure walk so the new
	// values are pushed to the module. Cleared once acted on.
	volatile bool m_config_reload_pending;

	// State machine methods
	void state_machine();
	void run_state_machine(uint16_t delay_ms = 100);

	void state_power_off_enter();
	void state_power_off();
	void state_power_off_exit();

	void state_power_on_enter();
	void state_power_on();
	void state_power_on_exit();

	void state_configure_enter();
	void state_configure();
	void state_configure_exit();

	/// @brief True when this build is OTAA but the join credentials are unset.
	/// Always false for ABP (njm == 0), which activates from DEVADDR + session
	/// keys and never touches APPEUI/APPKEY.
	bool otaa_credentials_missing() const;

	/// @brief Ask the module whether it is joined (AT+NJS=?). Local UART query,
	/// no RF and no duty-cycle cost.
	bool query_join_state();

	void state_joining_enter();
	void state_joining();
	void state_joining_exit();

	void state_idle_enter();
	void state_idle();
	void state_idle_exit();

	void state_standby_enter();
	void state_standby();
	void state_standby_exit();

	void state_transmit_enter();
	void state_transmit();
	void state_transmit_exit();

	void state_error_enter();
	void state_error();
	void state_error_exit();

	// Event handlers from LoRaComm
	void react(const LoRaCommEventRespOk &) override;
	void react(const LoRaCommEventRespError &) override;
	void react(const LoRaCommEventJoined &) override;
	void react(const LoRaCommEventJoinFailed &) override;
	void react(const LoRaCommEventTxDone &) override;
	void react(const LoRaCommEventRxData &) override;
	void react(const LoRaCommEventUartError &) override;

	// Bridge state
	bool m_bridge_active = false;

	// Helpers
	bool send_AT(LoRa::ATCmd cmd, const std::optional<std::string> &params = std::nullopt, uint16_t timeout_ms = 2000);
	void start_device();
	/// @brief Synchronous wake: if the module is powered off, trigger power-on
	/// and poll until it responds to AT_TEST (or timeout). Does NOT wait for
	/// LoRaWAN join — only UART + AT command channel readiness. Used by DTE
	/// helpers (SATVF, get_firmware_version, cw_start) so the GUI can query
	/// the module without having to issue a TX first.
	/// @param timeout_ms  Max wait in ms (default 3500 ≈ 2 s boot + ping retries).
	/// @return true if the module responded to AT within timeout, false otherwise.
	bool ensure_module_awake(unsigned int timeout_ms = 3500);
	void cancel_timeout();
	void initiate_timeout(unsigned int timeout_ms = 1000);
	void on_timeout();
	void load_config_from_store();
};
