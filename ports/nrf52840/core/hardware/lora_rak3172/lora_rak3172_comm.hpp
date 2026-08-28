/**
 * @file lora_rak3172_comm.hpp
 * @brief RAK3172 UART AT command layer — types, events, comm class.
 */

#pragma once

#include "nrf_uart_async.hpp"
#include "events.hpp"
#include <cstdint>
#include <string>
#include <string_view>
#include <optional>
#include <functional>

namespace LoRa {

enum ATCmd {
	AT_TEST = 0,     // AT             - connection test
	AT_RESET,        // ATZ            - MCU reset
	AT_GET_VER,      // AT+VER=?       - firmware version
	AT_SET_NWM,      // AT+NWM=        - network working mode (0=P2P, 1=LoRaWAN)
	AT_GET_NWM,      // AT+NWM=?       - get network working mode
	AT_SET_NJM,      // AT+NJM=        - join mode (0=ABP, 1=OTAA)
	AT_GET_NJM,      // AT+NJM=?       - get join mode
	AT_SET_BAND,     // AT+BAND=       - frequency band
	AT_GET_BAND,     // AT+BAND=?      - get frequency band
	AT_SET_DEVEUI,   // AT+DEVEUI=     - device EUI
	AT_GET_DEVEUI,   // AT+DEVEUI=?    - get device EUI
	AT_SET_APPEUI,   // AT+APPEUI=     - application EUI
	AT_GET_APPEUI,   // AT+APPEUI=?    - get application EUI
	AT_SET_APPKEY,   // AT+APPKEY=     - application key
	AT_GET_APPKEY,   // AT+APPKEY=?    - get application key
	AT_SET_DEVADDR,  // AT+DEVADDR=    - device address (ABP)
	AT_GET_DEVADDR,  // AT+DEVADDR=?   - get device address
	AT_SET_APPSKEY,  // AT+APPSKEY=    - application session key (ABP)
	AT_SET_NWKSKEY,  // AT+NWKSKEY=    - network session key (ABP)
	AT_SET_CLASS,    // AT+CLASS=      - device class (A/B/C)
	AT_GET_CLASS,    // AT+CLASS=?     - get device class
	AT_SET_DR,       // AT+DR=         - data rate
	AT_GET_DR,       // AT+DR=?        - get data rate
	AT_SET_ADR,      // AT+ADR=        - adaptive data rate
	AT_GET_ADR,      // AT+ADR=?       - get ADR status
	AT_SET_TXP,      // AT+TXP=        - transmit power index
	AT_GET_TXP,      // AT+TXP=?       - get TX power
	AT_SET_CFM,      // AT+CFM=        - confirmed/unconfirmed mode
	AT_GET_CFM,      // AT+CFM=?       - get confirm mode
	AT_SET_RETY,     // AT+RETY=       - confirmed uplink retry count
	AT_JOIN,         // AT+JOIN=       - join network
	AT_GET_NJS,      // AT+NJS=?       - get join status
	AT_SEND,         // AT+SEND=       - send data
	AT_GET_RECV,     // AT+RECV=?      - get last received data
	AT_GET_RSSI,     // AT+RSSI=?      - last RSSI
	AT_GET_SNR,      // AT+SNR=?       - last SNR
	AT_SET_DCS,      // AT+DCS=        - duty cycle setting
	AT_SET_MASK,     // AT+MASK=       - channel mask
	AT_SET_CHS,      // AT+CHS=        - single channel mode
	AT_SET_RX1DL,    // AT+RX1DL=      - RX1 delay
	AT_SET_RX2DR,    // AT+RX2DR=      - RX2 data rate
	AT_SET_PNM,      // AT+PNM=        - public network mode
	AT_SET_LPM,      // AT+LPM=        - low power mode (auto-sleep after AT)
	AT_SET_LPMLVL,   // AT+LPMLVL=     - low power level (1=Stop1, 2=Stop2) — RAK3172 only
	AT_SET_SLEEP,    // AT+SLEEP=      - enter sleep for N ms
	AT_SLEEP_NOW,    // AT+SLEEP       - continuous sleep (no param), wakes on UART RX byte
	AT_SET_CW,       // AT+CW=         - continuous wave (RF test)
	// Readback queries for low-power state — appended to the end to keep
	// existing ATCmd enum values stable (AT_SLEEP_NOW must remain 45 so
	// existing field logs / runbook references stay valid).
	AT_GET_LPM,     // AT+LPM=?       - read back current LPM mode
	AT_GET_LPMLVL,  // AT+LPMLVL=?    - read back current LPM level
	AT_UNKNOWN
};

enum RespType {
	RESP_OK = 0,
	RESP_ERROR,
	RESP_PARAM_ERROR,
	RESP_BUSY_ERROR,
	RESP_NO_NETWORK,
	RESP_VALUE,  // Value response (before OK)
	RESP_EVT_JOINED,
	RESP_EVT_JOIN_FAILED,
	RESP_EVT_TX_DONE,
	RESP_EVT_SEND_CONFIRMED_OK,
	RESP_EVT_SEND_CONFIRMED_FAILED,
	RESP_EVT_RX,  // Downlink data received
	RESP_UNKNOWN
};

// RAK3172 response strings (constexpr string_view — no heap allocation)
constexpr std::string_view OK_RESP = "OK";
constexpr std::string_view ERR_RESP = "AT_ERROR";
constexpr std::string_view PARAM_ERR_RESP = "AT_PARAM_ERROR";
constexpr std::string_view BUSY_ERR_RESP = "AT_BUSY_ERROR";
constexpr std::string_view NO_NET_RESP = "AT_NO_NETWORK_JOINED";

// Async event prefixes (constexpr string_view — no heap allocation)
constexpr std::string_view EVT_PREFIX = "+EVT:";
constexpr std::string_view EVT_JOINED = "+EVT:JOINED";
constexpr std::string_view EVT_JOIN_FAILED = "+EVT:JOIN FAILED";
constexpr std::string_view EVT_TX_DONE = "+EVT:TX_DONE";
constexpr std::string_view EVT_SEND_CONF_OK = "+EVT:SEND CONFIRMED OK";
constexpr std::string_view EVT_SEND_CONF_FAIL = "+EVT:SEND CONFIRMED FAILED";
constexpr std::string_view EVT_RX_PREFIX = "+EVT:RX_";

// Default LoRaWAN configuration (optimized for marine GPS tracker)
static constexpr uint8_t DEFAULT_NWM = 1;    // LoRaWAN mode
static constexpr uint8_t DEFAULT_NJM = 1;    // OTAA
static constexpr uint8_t DEFAULT_BAND = 4;   // EU868
static constexpr uint8_t DEFAULT_DR = 3;     // SF9/125kHz — best speed/range for marine
static constexpr uint8_t DEFAULT_ADR = 0;    // ADR OFF — mandatory for mobile devices
static constexpr uint8_t DEFAULT_TXP = 0;    // Max TX power (+16 dBm EIRP)
static constexpr uint8_t DEFAULT_CFM = 0;    // Unconfirmed — lower power, no downlink dependency
static constexpr uint8_t DEFAULT_FPORT = 2;  // Application port
static constexpr uint8_t DEFAULT_RETY = 0;   // No retries (unconfirmed mode)
static constexpr uint8_t DEFAULT_LPM = 1;    // Low power mode (Stop2, ~1.7uA sleep)

static constexpr uint8_t END_CHAR_CR = '\r';
static constexpr uint8_t END_CHAR_LF = '\n';
}

// Events emitted by LoRaComm
struct LoRaCommEventRespOk {};
struct LoRaCommEventRespError {
	LoRa::RespType error_type;
	LoRaCommEventRespError() : error_type(LoRa::RESP_ERROR) {}
	LoRaCommEventRespError(LoRa::RespType t) : error_type(t) {}
};
struct LoRaCommEventJoined {};
struct LoRaCommEventJoinFailed {};
struct LoRaCommEventTxDone {};
struct LoRaCommEventRxData {
	int port;
	std::string payload;  // hex-encoded
	LoRaCommEventRxData() : port(0) {}
	LoRaCommEventRxData(int p, const std::string &d) : port(p), payload(d) {}
};
struct LoRaCommEventUartError {
	unsigned int error_type;
	LoRaCommEventUartError(unsigned int a) : error_type(a) {}
};

class LoRaCommEventListener {
public:
	virtual ~LoRaCommEventListener() {}
	virtual void react(const LoRaCommEventRespOk &) {}
	virtual void react(const LoRaCommEventRespError &) {}
	virtual void react(const LoRaCommEventJoined &) {}
	virtual void react(const LoRaCommEventJoinFailed &) {}
	virtual void react(const LoRaCommEventTxDone &) {}
	virtual void react(const LoRaCommEventRxData &) {}
	virtual void react(const LoRaCommEventUartError &) {}
};

/**
 * @brief LoRa RAK3172 UART AT command layer.
 *
 * Inherits NrfUartAsync for UART lifecycle + deferred RX.
 * Adds: AT command table, RUI3 response parsing, bridge/passthrough mode.
 */
class LoRaComm : public NrfUartAsync, public EventEmitter<LoRaCommEventListener> {
public:
	std::string m_last_value;  // Last value response from a read command
	int m_last_rx_port = 0;    // Last RX event port number

	LoRaComm(unsigned int libuarte_async_instance = 1);

	/// @brief Init UART at the given baudrate (default 115200 = RAK3172 RUI3 factory).
	/// @param baudrate_override NRF_UARTE_BAUDRATE_* value. 0 keeps BSP default.
	void init(uint32_t baudrate_override = NRF_UARTE_BAUDRATE_115200);

	/// @brief Deinit UART.
	void deinit();

	/// @brief Send AT command.
	bool send(LoRa::ATCmd cmd, const std::optional<std::string> &params = std::nullopt);

	/// @brief Send raw bytes (bridge mode or direct).
	bool send_raw_data(const uint8_t *data, size_t len);

	/// @brief Process ISR-buffered RX data. Call periodically from main context.
	void process_rx();

	// Bridge/passthrough mode: forward raw UART RX to callback instead of parsing
	using PassthroughCallback = std::function<void(const uint8_t *, size_t)>;
	void set_passthrough(bool active, PassthroughCallback callback = nullptr);
	bool is_passthrough() const { return m_passthrough_active; }

protected:
	/// @brief Parse a complete RX line and emit events (RUI3 protocol).
	void on_rx_line(std::string &line) override;

	/// @brief Handle UART error — emit event.
	void on_rx_error(unsigned int error_type) override;

private:
	bool m_passthrough_active = false;
	PassthroughCallback m_passthrough_callback;

	bool send_at_cmd(LoRa::ATCmd cmd, const std::optional<std::string> &params = std::nullopt);
	LoRa::RespType parse_rx_line_protocol(std::string &line);
};
