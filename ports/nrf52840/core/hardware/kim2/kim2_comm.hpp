/**
 * @file kim2_comm.hpp
 * @brief KIM2 satellite module UART AT command communication layer.
 *
 * Handles UART TX/RX with the CLS KIM2 module via AT commands.
 * Response parsing supports +OK, +ID=, +ADDR=, +TX=, +ERROR= formats.
 * Events are emitted to listeners (KIM2Device) via the EventEmitter pattern.
 *
 * UART lifecycle and deferred RX are handled by NrfUartAsync base class.
 */

#pragma once

#include "nrf_uart_async.hpp"
#include "events.hpp"
#include <cstdint>
#include <string>
#include <optional>
#include <functional>

namespace KIM2 {

/// @brief AT command types supported by the KIM2 module.
/// @note  Per KIM2 Integration Manual v0.8: RCONF is kept in RAM and
///        reapplied on every power-on, so SAVE_RCONF is not used. KMAC=1
///        (basic MAC profile) must be called after RCONF and before any AT+TX.
/// @note  CORRECTION 2026-08-25: this comment used to claim LPM was not a
///        supported command. That is WRONG -- AT+LPM appears in the official AT
///        command table (KnsStack_AtCmds_APIs) from v0.4 through V1.0, for read
///        (AT+LPM=?) as well as write (AT+LPM=0x<bitmap>), and the handler is
///        present in the module binary. The claim hid the only lever we have for
///        bounding how deeply the module sleeps during a BLIND burst.
enum ATCmd {
	AT_PING = 0,
	AT_GET_ID,
	AT_GET_ADDR,
	AT_SET_RCONF,
	AT_GET_RCONF,
	AT_SET_KMAC_BASIC,
	AT_SET_KMAC_BLIND,
	AT_TX,
	AT_GET_FW,    ///< AT+FW=? — firmware build string, gates RX (kim2_rx.hpp)
	AT_DL_START,  ///< AT+DL=1[,<window_s>] — start reception (runtime mode)
	AT_DL_STOP,   ///< AT+DL=0 — stop reception
	AT_UNKNOWN
};

/// @brief Response types from the KIM2 module.
enum RespType {
	RESP_OK = 0,
	RESP_ERROR,
	RESP_CONFIG,
	RESP_TX_STATUS,
	RESP_ALLCAST,  ///< +DL_ALLCAST= — decoded allcast message (AOP / constellation status)
	RESP_RX_END,   ///< empty +RX= / +DL= — the RX window actually ended
	RESP_UNKNOWN
};

static constexpr uint8_t SYNC_CHAR = '+';
static constexpr uint8_t END_CHAR_1 = '\r';
static constexpr uint8_t END_CHAR_2 = '\n';

/// @name Response prefix strings
/// @{
static constexpr const char *OK_RESPONSE = "+OK";
static constexpr const char *ID_RESPONSE = "+ID=";
static constexpr const char *ADDR_RESPONSE = "+ADDR=";
static constexpr const char *RCONF_RESPONSE = "+RCONF=";
static constexpr const char *TX_RESPONSE = "+TX=";
static constexpr const char *ERR_RESPONSE = "+ERROR=";
static constexpr const char *HDLR_RESPONSE = "+HDLR=";  // new-stack AT+TX immediate ack (before +OK)
/// @brief Version banner the module prints SPONTANEOUSLY at every start-up. The
///        integration manual allows for lines like this (§3.A: "Spontaneous
///        notifications can also be sent from the module with the format
///        +CMD=<parameter>"), but we did not recognise it: it fell into
///        RESP_UNKNOWN and was discarded. Yet it is the only evidence of a module
///        RESTART -- measured 2026-08-25: with the pin released during a BLIND
///        burst, the module resets itself on its own retransmission deadline and
///        reprints this line, with no way for the firmware to see it. We
///        recognise it so we can act on it.
static constexpr const char *FW_RESPONSE = "+FW=";
/// @name RX-side unsolicited lines (firmware built with USE_RX_STACK)
/// The driver drives reception in RUNTIME mode (AT+DL), where the module only
/// reports decoded downlink messages. The test-mode lines (raw +RX= frames and
/// the detection diagnostics) are still recognised so a manual AT+RX=1 issued
/// through the bridge does not flood the log with unknown lines.
/// @{
static constexpr const char *DL_ALLCAST_RESPONSE = "+DL_ALLCAST=";  ///< decoded allcast frame
static constexpr const char *DL_USERBC_RESPONSE = "+DL_USERBC=";    ///< decoded beacon command
static constexpr const char *DL_RESPONSE = "+DL=";                  ///< empty = window end (runtime mode)
static constexpr const char *RX_RESPONSE = "+RX=";                  ///< raw frame, or empty = window end (test mode)
static constexpr const char *SATDET_RESPONSE = "+SATDET=";          ///< satellite detected (test mode)
static constexpr const char *SATLOST_RESPONSE = "+SATLOST=";        ///< satellite lost (test mode)
static constexpr const char *N0_RESPONSE = "+N0=";                  ///< noise density report (test mode)
/// @}
/// @}

static constexpr uint8_t ID_SIZE = 6;    ///< Decimal ID string length
static constexpr uint8_t ADDR_SIZE = 8;  ///< Hex address string length

/// @brief Decoded payload of an AT+RCONF=? response.
/// Format per KIM2 Integration Manual v0.8:
///   +RCONF=<min_freq_Hz>,<max_freq_Hz>,<rf_level_dBm>,<modulation_name>
/// where modulation_name is one of: LDA2, LDA2L, VLDA4, HDA4, LDK, UNKNOWN.
struct RConfDecoded {
	bool valid = false;  ///< True if all 4 fields parsed successfully
	unsigned int min_freq_hz = 0;
	unsigned int max_freq_hz = 0;
	int rf_level_dbm = 0;
	std::string modulation;  ///< Upper-case modulation name from the module
};

/// @brief Parse the trailing payload of a "+RCONF=..." response into its 4 fields.
/// @param info  Payload portion (already stripped of the "+RCONF=" prefix).
/// @return Decoded struct. On parse failure, RConfDecoded::valid is false.
RConfDecoded parse_rconf_info(const std::string &info);

}  // namespace KIM2

/// @name KIM2 communication events
/// @{
struct KIM2CommEventTxDone {};
/// @brief A +DL_ALLCAST= line was received. @c hex is the raw payload as printed
///        by the module: the KIM2 emits ceil(size_bits/4) nibbles, so the length
///        may be ODD (75 nibbles for a 297-bit message). Converting it to bytes
///        without completing the last nibble drops the final FCS bit and the
///        frame is then rejected downstream — see KIM2Device::state_receive().
struct KIM2CommEventAllcast {
	std::string hex;
};
/// @brief The RX window actually ended (explicit stop or window expiry), as
///        opposed to the solicited +OK acknowledging the stop command itself.
struct KIM2CommEventRxWindowEnd {};
struct KIM2CommEventRespOk {};
struct KIM2CommEventRespError {};
struct KIM2CommEventUartError {
	unsigned int error_type;
	KIM2CommEventUartError(unsigned int a) : error_type(a) {}
};
/// @}

class KIM2CommEventListener {
public:
	virtual ~KIM2CommEventListener() = default;
	virtual void react(const KIM2CommEventTxDone &) {}
	virtual void react(const KIM2CommEventAllcast &) {}
	virtual void react(const KIM2CommEventRxWindowEnd &) {}
	virtual void react(const KIM2CommEventRespOk &) {}
	virtual void react(const KIM2CommEventRespError &) {}
	virtual void react(const KIM2CommEventUartError &) {}
};

/**
 * @brief UART AT command layer for the KIM2 satellite module.
 *
 * Inherits NrfUartAsync for UART lifecycle + deferred RX.
 * Adds: AT command table, KIM2 response parsing.
 */
class KIM2Comm : public NrfUartAsync, public EventEmitter<KIM2CommEventListener> {
public:
	unsigned int m_kineis_id = 0;   ///< Device ID from +ID= response
	unsigned int m_hex_addr = 0;    ///< Device address from +ADDR= response
	uint16_t m_tx_status = 0xFFFF;  ///< Last TX status from +TX= response
	// KIM +TX response format is FIRMWARE-VERSION dependent, NOT MAC-profile
	// dependent:
	//   old stack (any profile): +TX=<status>[,<data>]           (status FIRST)
	//   new stack blind/satdet:  +TX=<handler>,<status>[,<data>]  (handler FIRST)
	// The new stack ALWAYS emits a "+HDLR=<handler>" ack just before +OK for the
	// AT+TX, so the presence of that ack in THIS TX cycle — not whether we loaded
	// AT+KMAC=2 — is the correct discriminator. Older KIM firmware runs BLIND
	// (AT+KMAC=2 accepted) yet still replies with the old status-first +TX; keying
	// off m_blind_active there misreads the frame-data field as the status (e.g.
	// "+TX=0,03387334..." -> stoi("03387334")&0xFFFF = 44998 "failure" on a real
	// SUCCESS). m_hdlr_seen makes both stacks parse correctly. Reset at each AT+TX.
	bool m_blind_active = false;
	bool m_hdlr_seen = false;
	void set_blind_active(bool active) { m_blind_active = active; }
	std::string m_rconf_info;  ///< Last +RCONF=? response payload (diag)
	/// @brief Version announced by the last +FW= banner received.
	std::string m_module_banner;

	/// @param libuarte_async_instance  BSP UART instance index (default 1).
	KIM2Comm(unsigned int libuarte_async_instance = 1);

	/// @brief Init UART and start RX.
	void init();

	/// @brief Deinit UART.
	void deinit();

	/// @brief Send an AT command (non-blocking).
	bool send(KIM2::ATCmd cmd, const std::optional<std::string> &params = std::nullopt);

	/// @brief Send raw bytes (bridge mode).
	bool send_raw_data(const uint8_t *data, size_t len);

	/// @brief Process ISR-buffered RX data. Call periodically from main context.
	void process_rx();

	// Bridge/passthrough mode: forward raw UART RX to callback instead of parsing
	using PassthroughCallback = std::function<void(const uint8_t *, size_t)>;
	void set_passthrough(bool active, PassthroughCallback callback = nullptr);
	bool is_passthrough() const { return m_passthrough_active; }

protected:
	/// @brief Parse a complete RX line and emit events (KIM2 protocol).
	void on_rx_line(std::string &line) override;

	/// @brief Handle UART error — emit event.
	void on_rx_error(unsigned int error_type) override;

private:
	bool m_passthrough_active = false;
	PassthroughCallback m_passthrough_callback;

	bool send_at_cmd(KIM2::ATCmd cmd, const std::optional<std::string> &params = std::nullopt);
	KIM2::RespType parse_rx_line_protocol(const std::string &line);
};
