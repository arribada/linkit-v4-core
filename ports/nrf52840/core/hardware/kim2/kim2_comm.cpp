/**
 * @file kim2_comm.cpp
 * @brief KIM2 UART AT command layer — response parsing.
 *
 * UART lifecycle and deferred RX are handled by NrfUartAsync base class.
 * This file implements only: AT command table and KIM2 response parsing.
 */

#include "kim2_comm.hpp"
#include "debug.hpp"
#include <cstdlib>
#include <cstring>

using namespace KIM2;

// ============================================================================
// RCONF response parser (namespace-scope helper — reusable by KIM2Device)
// ============================================================================

KIM2::RConfDecoded KIM2::parse_rconf_info(const std::string& info)
{
	// Expected: "<min_freq>,<max_freq>,<rf_level>,<modulation>"
	// e.g. "401625000,401635000,27,VLDA4"
	RConfDecoded out;

	const size_t c1 = info.find(',');
	if (c1 == std::string::npos) return out;
	const size_t c2 = info.find(',', c1 + 1);
	if (c2 == std::string::npos) return out;
	const size_t c3 = info.find(',', c2 + 1);
	if (c3 == std::string::npos) return out;

	char* end = nullptr;
	const std::string s_min = info.substr(0, c1);
	const std::string s_max = info.substr(c1 + 1, c2 - c1 - 1);
	const std::string s_rf  = info.substr(c2 + 1, c3 - c2 - 1);
	const std::string s_mod = info.substr(c3 + 1);

	const unsigned long min_hz = strtoul(s_min.c_str(), &end, 10);
	if (end == s_min.c_str()) return out;
	const unsigned long max_hz = strtoul(s_max.c_str(), &end, 10);
	if (end == s_max.c_str()) return out;
	const long rf = strtol(s_rf.c_str(), &end, 10);
	if (end == s_rf.c_str()) return out;

	out.min_freq_hz  = static_cast<unsigned int>(min_hz);
	out.max_freq_hz  = static_cast<unsigned int>(max_hz);
	out.rf_level_dbm = static_cast<int>(rf);
	out.modulation   = s_mod;
	// Strip any trailing whitespace/CR left over (base class should have stripped CRLF)
	while (!out.modulation.empty() &&
	       (out.modulation.back() == '\r' || out.modulation.back() == '\n' ||
	        out.modulation.back() == ' '  || out.modulation.back() == '\t')) {
		out.modulation.pop_back();
	}
	out.valid = !out.modulation.empty();
	return out;
}

// ============================================================================
// Constructor / Init / Deinit — delegate to NrfUartAsync
// ============================================================================

KIM2Comm::KIM2Comm(unsigned int instance)
	: NrfUartAsync(instance)
{
}

void KIM2Comm::init()
{
	NrfUartAsync::init();  // Uses BSP default baudrate
}

void KIM2Comm::deinit()
{
	NrfUartAsync::deinit();
}

// ============================================================================
// AT command send
// ============================================================================

bool KIM2Comm::send(ATCmd cmd, const std::optional<std::string>& params)
{
	if (m_is_send_busy) {
		DEBUG_ERROR("KIM2Comm: already busy");
		return false;
	}
	// Note: m_is_send_busy is set by NrfUartAsync::send_string(), not here.
	return send_at_cmd(cmd, params);
}

/// @brief AT command table (static — no heap allocation).
struct ATCmdEntry {
	ATCmd type;
	const char *prefix;
	bool expects_params;
};

static constexpr ATCmdEntry cmd_table[] = {
	{ AT_PING,           "AT+PING=?\r\n",    false },
	{ AT_GET_ID,         "AT+ID=?\r\n",       false },
	{ AT_GET_ADDR,       "AT+ADDR=?\r\n",     false },
	{ AT_SET_RCONF,      "AT+RCONF=",          true },
	{ AT_GET_RCONF,      "AT+RCONF=?\r\n",    false },
	{ AT_SET_KMAC_BASIC, "AT+KMAC=1\r\n",     false },
	{ AT_SET_KMAC_BLIND, "AT+KMAC=2,",         true  },
	{ AT_TX,             "AT+TX=",              true },
	{ AT_GET_FW,         "AT+FW=?\r\n",         false },
	{ AT_DL_START,       "AT+DL=",              true  },
	{ AT_DL_STOP,        "AT+DL=0\r\n",         false },
};

bool KIM2Comm::send_at_cmd(ATCmd cmd, const std::optional<std::string>& params)
{
	if (cmd >= AT_UNKNOWN || cmd >= static_cast<ATCmd>(std::size(cmd_table))) {
		return false;
	}
	const auto& entry = cmd_table[cmd];

	// Fresh AT+TX cycle: forget any prior +HDLR ack so the delayed +TX status is
	// parsed against THIS transmission's stack format (see m_hdlr_seen).
	if (cmd == AT_TX)
		m_hdlr_seen = false;

	m_tx_buffer = entry.prefix;
	if (entry.expects_params && params.has_value()) {
		m_tx_buffer.append(params.value());
		m_tx_buffer += "\r\n";
	}

	return send_string(m_tx_buffer);
}

// ============================================================================
// process_rx — delegate to base class
// ============================================================================

void KIM2Comm::process_rx()
{
	NrfUartAsync::process_rx();
}

// ============================================================================
// Bridge/passthrough mode — raw UART access for DTE KIMBR command
// ============================================================================

bool KIM2Comm::send_raw_data(const uint8_t* data, size_t len)
{
	return NrfUartAsync::send_raw(data, len);
}

void KIM2Comm::set_passthrough(bool active, PassthroughCallback callback)
{
	m_passthrough_active = active;
	m_passthrough_callback = callback;
}

// ============================================================================
// Protocol parsing — NrfUartAsync callbacks
// ============================================================================

void KIM2Comm::on_rx_line(std::string& line)
{
	// Bridge/passthrough mode: forward raw line (with CRLF) to callback, skip AT parsing.
	// Base class strips CRLF so we reconstitute for terminal-style display on USB.
	if (m_passthrough_active) {
		if (m_passthrough_callback && !line.empty()) {
			std::string framed = line + "\r\n";
			m_passthrough_callback(reinterpret_cast<const uint8_t*>(framed.data()),
			                        framed.size());
		}
		return;
	}

	// Diagnostic: trace every line received from the module so we can see
	// +OK / +RCONF= / +ERROR= / +TX= etc. during debugging. Elevated to INFO on
	// the bench so the raw module responses (incl. the exact +TX= format, which
	// varies by KIM2 firmware version) are visible without a TRACE-level build.
#ifdef BENCH_TEST
	DEBUG_INFO("KIM2 RX: %s", line.c_str());
#else
	DEBUG_TRACE("KIM2 RX: %s", line.c_str());
#endif

	RespType msg = parse_rx_line_protocol(line);

	if (msg == RESP_OK) {
		notify<KIM2CommEventRespOk>({});
	} else if (msg == RESP_TX_STATUS) {
		notify<KIM2CommEventTxDone>({});
	} else if (msg == RESP_ALLCAST) {
		// Payload carried by the event: several allcast lines can arrive inside
		// a single process_rx() burst, so a shared member would be clobbered
		// before the listener had a chance to copy it.
		notify(KIM2CommEventAllcast{ line.substr(strlen(DL_ALLCAST_RESPONSE)) });
	} else if (msg == RESP_RX_END) {
		notify<KIM2CommEventRxWindowEnd>({});
	} else if (msg == RESP_ERROR) {
		DEBUG_INFO("KIM2: error response: %s", line.c_str());
		notify<KIM2CommEventRespError>({});
	}
	// RESP_CONFIG and RESP_UNKNOWN: no event, values stored in members
}

void KIM2Comm::on_rx_error(unsigned int error_type)
{
	notify(KIM2CommEventUartError(error_type));
}

// ============================================================================
// KIM2 response line parser
// ============================================================================

/// @brief Parse a single line and extract response type + data.
/// @note Line is already stripped of CR/LF by NrfUartAsync.
KIM2::RespType KIM2Comm::parse_rx_line_protocol(const std::string& line)
{
	if (line.empty() || line[0] != SYNC_CHAR)
		return RESP_UNKNOWN;

	size_t ok_len    = strlen(OK_RESPONSE);
	size_t id_len    = strlen(ID_RESPONSE);
	size_t addr_len  = strlen(ADDR_RESPONSE);
	size_t rconf_len = strlen(RCONF_RESPONSE);
	size_t tx_len    = strlen(TX_RESPONSE);
	size_t err_len   = strlen(ERR_RESPONSE);

	// +OK
	if (line.compare(0, ok_len, OK_RESPONSE) == 0 && line.size() == ok_len)
		return RESP_OK;

	// +ID=<6 decimal digits>
	if (line.compare(0, id_len, ID_RESPONSE) == 0 && line.size() >= id_len + ID_SIZE) {
		char ascii_id[ID_SIZE + 1] = {};
		std::memcpy(ascii_id, line.c_str() + id_len, ID_SIZE);
		char *end = nullptr;
		long id_val = strtol(ascii_id, &end, 10);
		m_kineis_id = (end != ascii_id && *end == '\0') ? static_cast<unsigned int>(id_val) : 0;
		return RESP_CONFIG;
	}

	// +ADDR=<8 hex digits>
	if (line.compare(0, addr_len, ADDR_RESPONSE) == 0 && line.size() >= addr_len + ADDR_SIZE) {
		char ascii_addr[ADDR_SIZE + 1] = {};
		std::memcpy(ascii_addr, line.c_str() + addr_len, ADDR_SIZE);
		char *end = nullptr;
		unsigned long addr_val = strtoul(ascii_addr, &end, 16);
		m_hex_addr = (end != ascii_addr) ? static_cast<unsigned int>(addr_val) : 0;
		return RESP_CONFIG;
	}

	// +RCONF=<freq_min>,<freq_max>,<mod_type>,<rf_level>
	// Store the payload portion for diagnostics (e.g. verify mod_type).
	if (line.compare(0, rconf_len, RCONF_RESPONSE) == 0) {
		m_rconf_info = line.substr(rconf_len);
		return RESP_CONFIG;
	}

	// +FW=<build string>
	if (line.compare(0, strlen(FW_RESPONSE), FW_RESPONSE) == 0) {
		m_module_banner = line.substr(line.find(FW_RESPONSE) + strlen(FW_RESPONSE));
		return RESP_CONFIG;
	}

	// ---- RX-side unsolicited lines (AT+DL runtime mode) ---------------------
	// Order matters: "+DL_ALLCAST=" and "+DL_USERBC=" must be tested before the
	// bare "+DL=" end-of-window marker.

	// +DL_ALLCAST=<hex data> — the only line we act on: allcast AOP / CS update.
	if (line.compare(0, strlen(DL_ALLCAST_RESPONSE), DL_ALLCAST_RESPONSE) == 0)
		return RESP_ALLCAST;

	// +DL_USERBC=<hex data>,<bc_mc> — decoded beacon command, not handled here.
	if (line.compare(0, strlen(DL_USERBC_RESPONSE), DL_USERBC_RESPONSE) == 0) {
		DEBUG_TRACE("KIM2: user beacon command ignored: %s", line.c_str());
		return RESP_CONFIG;
	}

	// +DL= with an EMPTY payload: the reception window actually ended (explicit
	// AT+DL=0 or window expiry), as opposed to the solicited +OK acknowledging
	// the stop command itself.
	if (line.compare(0, strlen(DL_RESPONSE), DL_RESPONSE) == 0)
		return (line.size() == strlen(DL_RESPONSE)) ? RESP_RX_END : RESP_CONFIG;

	// Test-mode lines, never emitted in runtime mode: an empty +RX= closes a
	// window started by a manual AT+RX=1, a populated one is a raw frame we
	// deliberately ignore — the decoded +DL_ALLCAST= carries the same content
	// already framed.
	if (line.compare(0, strlen(RX_RESPONSE), RX_RESPONSE) == 0)
		return (line.size() == strlen(RX_RESPONSE)) ? RESP_RX_END : RESP_CONFIG;

	// Detection diagnostics (test mode): traced, never acted upon.
	if (line.compare(0, strlen(SATDET_RESPONSE), SATDET_RESPONSE) == 0 ||
	    line.compare(0, strlen(SATLOST_RESPONSE), SATLOST_RESPONSE) == 0 ||
	    line.compare(0, strlen(N0_RESPONSE), N0_RESPONSE) == 0) {
		DEBUG_TRACE("KIM2 RX diag: %s", line.c_str());
		return RESP_CONFIG;
	}

	// +ERROR=...
	if (line.compare(0, err_len, ERR_RESPONSE) == 0)
		return RESP_ERROR;

	// +HDLR=<TX handler id> — new-stack immediate ack, sent just before +OK.
	// Informational: we advance on +OK, so recognise and ignore it (a benign
	// type avoids "unknown line" noise on the new stack). Its PRESENCE flags that
	// this firmware puts the handler first in the delayed +TX (see m_hdlr_seen).
	if (line.compare(0, strlen(HDLR_RESPONSE), HDLR_RESPONSE) == 0) {
		m_hdlr_seen = true;
		return RESP_CONFIG;
	}

	// +TX=<status>[,<data>]            (old stack — status FIRST, any MAC profile)
	// +TX=<handler>,<status>[,<data>]  (new stack blind/satdet — handler FIRST)
	// Discriminate on whether THIS TX cycle produced a +HDLR ack (new stack), NOT
	// on m_blind_active: old KIM firmware runs BLIND but keeps the status-first
	// format, and dropping a field there parses the frame-data hex as the status.
	if (line.compare(0, tx_len, TX_RESPONSE) == 0) {
		std::string resp = line.substr(tx_len);
		if (m_hdlr_seen) {
			// New stack: drop the leading handler field so <status> parses.
			size_t h = resp.find(',');
			if (h != std::string::npos)
				resp = resp.substr(h + 1);
		}
		size_t comma = resp.find(',');
		std::string status_str = (comma != std::string::npos) ? resp.substr(0, comma) : resp;
		try {
			m_tx_status = static_cast<uint16_t>(std::stoi(status_str));
		} catch (...) {
			m_tx_status = 0xFFFF;
		}
		return RESP_TX_STATUS;
	}

	return RESP_UNKNOWN;
}
