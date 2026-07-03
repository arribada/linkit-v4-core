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
///        reapplied on every power-on, so SAVE_RCONF is not used. LPM is
///        not a supported AT command. KMAC=1 (basic MAC profile) must be
///        called after RCONF and before any AT+TX.
enum ATCmd {
	AT_PING = 0,
	AT_GET_ID,
	AT_GET_ADDR,
	AT_SET_RCONF,
	AT_GET_RCONF,
	AT_SET_KMAC_BASIC,
	AT_SET_KMAC_BLIND,
	AT_TX,
	AT_UNKNOWN
};

/// @brief Response types from the KIM2 module.
enum RespType {
	RESP_OK = 0,
	RESP_ERROR,
	RESP_CONFIG,
	RESP_TX_STATUS,
	RESP_UNKNOWN
};

static constexpr uint8_t SYNC_CHAR  = '+';
static constexpr uint8_t END_CHAR_1 = '\r';
static constexpr uint8_t END_CHAR_2 = '\n';

/// @name Response prefix strings
/// @{
static constexpr const char *OK_RESPONSE   = "+OK";
static constexpr const char *ID_RESPONSE   = "+ID=";
static constexpr const char *ADDR_RESPONSE = "+ADDR=";
static constexpr const char *RCONF_RESPONSE = "+RCONF=";
static constexpr const char *TX_RESPONSE   = "+TX=";
static constexpr const char *ERR_RESPONSE  = "+ERROR=";
/// @}

static constexpr uint8_t ID_SIZE   = 6;   ///< Decimal ID string length
static constexpr uint8_t ADDR_SIZE = 8;   ///< Hex address string length

/// @brief Decoded payload of an AT+RCONF=? response.
/// Format per KIM2 Integration Manual v0.8:
///   +RCONF=<min_freq_Hz>,<max_freq_Hz>,<rf_level_dBm>,<modulation_name>
/// where modulation_name is one of: LDA2, LDA2L, VLDA4, HDA4, LDK, UNKNOWN.
struct RConfDecoded {
	bool         valid        = false;  ///< True if all 4 fields parsed successfully
	unsigned int min_freq_hz  = 0;
	unsigned int max_freq_hz  = 0;
	int          rf_level_dbm = 0;
	std::string  modulation;            ///< Upper-case modulation name from the module
};

/// @brief Parse the trailing payload of a "+RCONF=..." response into its 4 fields.
/// @param info  Payload portion (already stripped of the "+RCONF=" prefix).
/// @return Decoded struct. On parse failure, RConfDecoded::valid is false.
RConfDecoded parse_rconf_info(const std::string& info);

} // namespace KIM2

/// @name KIM2 communication events
/// @{
struct KIM2CommEventTxDone {};
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
	virtual void react(const KIM2CommEventTxDone&) {}
	virtual void react(const KIM2CommEventRespOk&) {}
	virtual void react(const KIM2CommEventRespError&) {}
	virtual void react(const KIM2CommEventUartError&) {}
};

/**
 * @brief UART AT command layer for the KIM2 satellite module.
 *
 * Inherits NrfUartAsync for UART lifecycle + deferred RX.
 * Adds: AT command table, KIM2 response parsing.
 */
class KIM2Comm : public NrfUartAsync, public EventEmitter<KIM2CommEventListener> {
public:
	unsigned int m_kineis_id = 0;      ///< Device ID from +ID= response
	unsigned int m_hex_addr = 0;       ///< Device address from +ADDR= response
	uint16_t m_tx_status = 0xFFFF;     ///< Last TX status from +TX= response
	std::string m_rconf_info;          ///< Last +RCONF=? response payload (diag)

	/// @param libuarte_async_instance  BSP UART instance index (default 1).
	KIM2Comm(unsigned int libuarte_async_instance = 1);

	/// @brief Init UART and start RX.
	void init();

	/// @brief Deinit UART.
	void deinit();

	/// @brief Send an AT command (non-blocking).
	bool send(KIM2::ATCmd cmd, const std::optional<std::string>& params = std::nullopt);

	/// @brief Send raw bytes (bridge mode).
	bool send_raw_data(const uint8_t* data, size_t len);

	/// @brief Process ISR-buffered RX data. Call periodically from main context.
	void process_rx();

	// Bridge/passthrough mode: forward raw UART RX to callback instead of parsing
	using PassthroughCallback = std::function<void(const uint8_t*, size_t)>;
	void set_passthrough(bool active, PassthroughCallback callback = nullptr);
	bool is_passthrough() const { return m_passthrough_active; }

protected:
	/// @brief Parse a complete RX line and emit events (KIM2 protocol).
	void on_rx_line(std::string& line) override;

	/// @brief Handle UART error — emit event.
	void on_rx_error(unsigned int error_type) override;

private:
	bool m_passthrough_active = false;
	PassthroughCallback m_passthrough_callback;

	bool send_at_cmd(KIM2::ATCmd cmd, const std::optional<std::string>& params = std::nullopt);
	KIM2::RespType parse_rx_line_protocol(const std::string& line);
};
