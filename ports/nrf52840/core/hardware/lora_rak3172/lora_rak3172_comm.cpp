/**
 * @file lora_rak3172_comm.cpp
 * @brief RAK3172 UART AT command layer — RUI3 protocol parsing.
 *
 * UART lifecycle and deferred RX are handled by NrfUartAsync base class.
 * This file implements only: AT command table, RUI3 response parsing, bridge mode.
 */

#include "lora_rak3172_comm.hpp"
#include "debug.hpp"
#include "interrupt_lock.hpp"
#include <cstring>

using namespace LoRa;

// ============================================================================
// Constructor / Init / Deinit — delegate to NrfUartAsync
// ============================================================================

LoRaComm::LoRaComm(unsigned int instance)
    : NrfUartAsync(instance)
{
}

void LoRaComm::init(uint32_t baudrate_override)
{
    m_last_value.reserve(64);
    NrfUartAsync::init(baudrate_override);
}

void LoRaComm::deinit()
{
    NrfUartAsync::deinit();
}

// ============================================================================
// AT command send
// ============================================================================

bool LoRaComm::send(ATCmd cmd, const std::optional<std::string>& params)
{
    if (m_is_send_busy) {
        DEBUG_ERROR("LoRaComm: already busy");
        return false;
    }
    // Note: m_is_send_busy is set by NrfUartAsync::send_string(), not here.
    // Setting it here would cause send_string() to see it as "already busy".
    return send_at_cmd(cmd, params);
}

bool LoRaComm::send_raw_data(const uint8_t* data, size_t len)
{
    return NrfUartAsync::send_raw(data, len);
}

/// @brief AT command table entry (static — no heap allocation).
struct ATCmdEntry {
    ATCmd type;
    const char *prefix;
    bool expects_params;
};

/// @brief AT command string table for RAK3172 RUI3.
/// Indexed by ATCmd enum value for O(1) lookup.
static constexpr ATCmdEntry cmd_table[] = {
    { AT_TEST,       "AT\r\n",          false },
    { AT_RESET,      "ATZ\r\n",         false },
    { AT_GET_VER,    "AT+VER=?\r\n",    false },
    { AT_SET_NWM,    "AT+NWM=",         true  },
    { AT_GET_NWM,    "AT+NWM=?\r\n",    false },
    { AT_SET_NJM,    "AT+NJM=",         true  },
    { AT_GET_NJM,    "AT+NJM=?\r\n",    false },
    { AT_SET_BAND,   "AT+BAND=",        true  },
    { AT_GET_BAND,   "AT+BAND=?\r\n",   false },
    { AT_SET_DEVEUI, "AT+DEVEUI=",      true  },
    { AT_GET_DEVEUI, "AT+DEVEUI=?\r\n", false },
    { AT_SET_APPEUI, "AT+APPEUI=",      true  },
    { AT_GET_APPEUI, "AT+APPEUI=?\r\n", false },
    { AT_SET_APPKEY, "AT+APPKEY=",      true  },
    { AT_GET_APPKEY, "AT+APPKEY=?\r\n", false },
    { AT_SET_DEVADDR,"AT+DEVADDR=",     true  },
    { AT_GET_DEVADDR,"AT+DEVADDR=?\r\n",false },
    { AT_SET_APPSKEY,"AT+APPSKEY=",     true  },
    { AT_SET_NWKSKEY,"AT+NWKSKEY=",     true  },
    { AT_SET_CLASS,  "AT+CLASS=",       true  },
    { AT_GET_CLASS,  "AT+CLASS=?\r\n",  false },
    { AT_SET_DR,     "AT+DR=",          true  },
    { AT_GET_DR,     "AT+DR=?\r\n",     false },
    { AT_SET_ADR,    "AT+ADR=",         true  },
    { AT_GET_ADR,    "AT+ADR=?\r\n",    false },
    { AT_SET_TXP,    "AT+TXP=",         true  },
    { AT_GET_TXP,    "AT+TXP=?\r\n",    false },
    { AT_SET_CFM,    "AT+CFM=",         true  },
    { AT_GET_CFM,    "AT+CFM=?\r\n",    false },
    { AT_SET_RETY,   "AT+RETY=",        true  },
    { AT_JOIN,       "AT+JOIN=",        true  },
    { AT_GET_NJS,    "AT+NJS=?\r\n",    false },
    { AT_SEND,       "AT+SEND=",        true  },
    { AT_GET_RECV,   "AT+RECV=?\r\n",   false },
    { AT_GET_RSSI,   "AT+RSSI=?\r\n",   false },
    { AT_GET_SNR,    "AT+SNR=?\r\n",    false },
    { AT_SET_DCS,    "AT+DCS=",         true  },
    { AT_SET_MASK,   "AT+MASK=",        true  },
    { AT_SET_CHS,    "AT+CHS=",         true  },
    { AT_SET_RX1DL,  "AT+RX1DL=",      true  },
    { AT_SET_RX2DR,  "AT+RX2DR=",      true  },
    { AT_SET_PNM,    "AT+PNM=",         true  },
    { AT_SET_LPM,    "AT+LPM=",         true  },
    { AT_SET_LPMLVL, "AT+LPMLVL=",      true  },
    { AT_SET_SLEEP,  "AT+SLEEP=",       true  },
    { AT_SLEEP_NOW,  "AT+SLEEP\r\n",    false },
    { AT_SET_CW,     "AT+CW=",          true  },
    // Readback queries — see header note about enum stability.
    { AT_GET_LPM,    "AT+LPM=?\r\n",    false },
    { AT_GET_LPMLVL, "AT+LPMLVL=?\r\n", false },
};

bool LoRaComm::send_at_cmd(ATCmd cmd, const std::optional<std::string>& params)
{
    // O(1) lookup: ATCmd enum values match cmd_table indices
    if (cmd >= AT_UNKNOWN || cmd >= static_cast<ATCmd>(std::size(cmd_table))) {
        return false;
    }
    const auto& entry = cmd_table[cmd];

    // Build AT command string in m_tx_buffer (inherited from NrfUartAsync)
    m_tx_buffer = entry.prefix;
    if (entry.expects_params && params.has_value()) {
        m_tx_buffer.append(params.value());
        m_tx_buffer += "\r\n";
    }

    DEBUG_TRACE("LoRaComm::send> %.*s", static_cast<int>(m_tx_buffer.size()) - 2, m_tx_buffer.c_str());

    return send_string(m_tx_buffer);
}

// ============================================================================
// Bridge/passthrough mode
// ============================================================================

void LoRaComm::set_passthrough(bool active, PassthroughCallback callback)
{
    m_passthrough_active = active;
    m_passthrough_callback = callback;
}

// ============================================================================
// process_rx — wraps base class, adds passthrough support
// ============================================================================

void LoRaComm::process_rx()
{
    // Delegate to base class. on_rx_line() below checks m_passthrough_active
    // first and, when set, forwards the raw line (+CRLF) to the callback
    // instead of running the RUI3 AT parser. This is how bridge mode pipes
    // RAK3172 output back to the USB/BLE host untouched.
    NrfUartAsync::process_rx();
}

// ============================================================================
// Protocol parsing — NrfUartAsync callbacks
// ============================================================================

void LoRaComm::on_rx_line(std::string& line)
{
    // Bridge / passthrough mode: forward every RAK3172 line unparsed to the
    // host (with CR+LF re-added — base class strips terminators). The AT
    // protocol parser is skipped entirely so the host sees exactly what the
    // module says (RUI3 "OK", "+EVT:...", banner text, etc.).
    if (m_passthrough_active) {
        if (m_passthrough_callback && !line.empty()) {
            std::string framed = line + "\r\n";
            m_passthrough_callback(reinterpret_cast<const uint8_t*>(framed.data()),
                                   framed.size());
        }
        return;
    }

    LoRa::RespType resp = parse_rx_line_protocol(line);

    DEBUG_TRACE("LoRaComm::rx< %s", line.c_str());

    switch (resp) {
        case RESP_OK:
            notify<LoRaCommEventRespOk>({});
            break;
        case RESP_ERROR:
        case RESP_PARAM_ERROR:
        case RESP_BUSY_ERROR:
        case RESP_NO_NETWORK:
            notify(LoRaCommEventRespError(resp));
            break;
        case RESP_EVT_JOINED:
            notify<LoRaCommEventJoined>({});
            break;
        case RESP_EVT_JOIN_FAILED:
            notify<LoRaCommEventJoinFailed>({});
            break;
        case RESP_EVT_TX_DONE:
        case RESP_EVT_SEND_CONFIRMED_OK:
            notify<LoRaCommEventTxDone>({});
            break;
        case RESP_EVT_SEND_CONFIRMED_FAILED:
            notify(LoRaCommEventRespError(RESP_EVT_SEND_CONFIRMED_FAILED));
            break;
        case RESP_EVT_RX:
            notify(LoRaCommEventRxData(m_last_rx_port, m_last_value));
            break;
        case RESP_VALUE:
            // Value stored in m_last_value, OK will follow
            break;
        case RESP_UNKNOWN:
        default:
            break;
    }
}

void LoRaComm::on_rx_error(unsigned int error_type)
{
    notify(LoRaCommEventUartError(error_type));
}

// ============================================================================
// RUI3 response line parser
// ============================================================================

/// @brief Parse a single line from the RAK3172 response.
LoRa::RespType LoRaComm::parse_rx_line_protocol(std::string& line)
{
    if (line.empty())
        return RESP_UNKNOWN;

    // Check for OK
    if (line == OK_RESP)
        return RESP_OK;

    // Check for errors
    if (line == ERR_RESP)
        return RESP_ERROR;
    if (line == PARAM_ERR_RESP)
        return RESP_PARAM_ERROR;
    if (line == BUSY_ERR_RESP)
        return RESP_BUSY_ERROR;
    if (line == NO_NET_RESP)
        return RESP_NO_NETWORK;

    // Check for async events
    if (line.compare(0, EVT_PREFIX.size(), EVT_PREFIX.data(), EVT_PREFIX.size()) == 0)
    {
        if (line == EVT_JOINED)
            return RESP_EVT_JOINED;
        if (line == EVT_JOIN_FAILED)
            return RESP_EVT_JOIN_FAILED;
        if (line == EVT_TX_DONE)
            return RESP_EVT_TX_DONE;
        if (line == EVT_SEND_CONF_OK)
            return RESP_EVT_SEND_CONFIRMED_OK;
        if (line == EVT_SEND_CONF_FAIL)
            return RESP_EVT_SEND_CONFIRMED_FAILED;

        // Check for RX data: +EVT:RX_1:<rssi>:<snr>:UNICAST:<port>:<payload>
        if (line.compare(0, EVT_RX_PREFIX.size(), EVT_RX_PREFIX.data(), EVT_RX_PREFIX.size()) == 0)
        {
            size_t last_colon = line.rfind(':');
            if (last_colon != std::string::npos && last_colon > 0) {
                size_t prev_colon = line.rfind(':', last_colon - 1);
                if (prev_colon != std::string::npos) {
                    m_last_value.assign(line, last_colon + 1, std::string::npos);
                    m_last_rx_port = 0;
                    for (size_t i = prev_colon + 1; i < last_colon; i++) {
                        if (line[i] >= '0' && line[i] <= '9')
                            m_last_rx_port = m_last_rx_port * 10 + (line[i] - '0');
                    }
                    DEBUG_TRACE("LoRaComm: RX port=%d payload=%s", m_last_rx_port, m_last_value.c_str());
                }
            }
            return RESP_EVT_RX;
        }

        DEBUG_TRACE("LoRaComm: unknown EVT: %s", line.c_str());
        return RESP_UNKNOWN;
    }

    // Filter out RAK3172 boot banner / unsolicited notifications so they
    // don't pollute m_last_value and race with the next AT query.
    // Typical boot output: "Current Work Mode: LoRaWAN.", "RAKwireless...",
    // "Welcome...", "LoRaWAN stack has been initialized".
    if (line.compare(0, 6, "Curren") == 0 ||    // "Current Work Mode: ..."
        line.compare(0, 11, "RAKwireless") == 0 ||
        line.compare(0, 7, "Welcome") == 0 ||
        line.compare(0, 7, "LoRaWAN") == 0 ||
        line.compare(0, 6, "Region") == 0) {
        DEBUG_TRACE("LoRaComm: banner line ignored: %s", line.c_str());
        return RESP_UNKNOWN;
    }

    // Value response: RUI3 echoes the query as "AT+XXX=<value>" or "+XXX:<value>".
    // Strip the prefix so callers get the bare value (e.g. "1" instead of
    // "AT+NJM=1", hex string instead of "AT+DEVEUI=xxxx"). Without this,
    // std::stoul() on NJM and string comparison on credentials would break.
    std::string value = line;
    if (line.size() > 3 && line.compare(0, 3, "AT+") == 0) {
        size_t eq = line.find('=');
        if (eq != std::string::npos && eq + 1 < line.size())
            value = line.substr(eq + 1);
    } else if (line.size() > 1 && line[0] == '+') {
        size_t colon = line.find(':');
        if (colon != std::string::npos && colon + 1 < line.size()) {
            size_t v_start = colon + 1;
            while (v_start < line.size() && line[v_start] == ' ') v_start++;
            value = line.substr(v_start);
        }
    }

    m_last_value.assign(value);
    return RESP_VALUE;
}
