/**
 * @file kim2.cpp
 * @brief KIM2 satellite module — state machine, AT command flow, TX management.
 */

#include "kim2.hpp"
#include "kim2_comm.hpp"
#include "kim2_modulation.hpp"
#include "kim2_rx.hpp"
#include "bsp.hpp"
#include "gpio.hpp"
#include "pmu.hpp"
#include "debug.hpp"
#include "error.hpp"
#include "bitpack.hpp"
#include "binascii.hpp"
#include "config_store.hpp"
#include "ledsm.hpp"
#include <cstdint>
#include <string>

using namespace KIM2;

/// @name Payload size limits per modulation (bits)
/// @{
static constexpr uint16_t LDK_MAX_LENGTH_BITS  = 16 * 8;
static constexpr uint16_t LDA2_MAX_LENGTH_BITS  = 24 * 8;
static constexpr uint16_t VLDA4_MAX_LENGTH_BITS  = 3 * 8;
/// @}

/// @name Timing constants (ms)
/// @{
static constexpr uint16_t KIM2_DELAY_POWER_ON_MS = 500;   ///< Wait after power-on before ping
static constexpr uint16_t KIM2_DELAY_POLL_MS     = 100;   ///< TX poll interval
static constexpr uint32_t KIM2_TX_TIMEOUT_MS     = 60000; ///< Max time for TX complete
static constexpr uint16_t KIM2_IDLE_TICK_MS      = 100;   ///< Idle state poll interval
/// @brief Delay after AT+RCONF= so the module can process the new radio config
///        (analogous to SMD's 80ms STM32 RCONF processing window).
static constexpr uint16_t KIM2_DELAY_AFTER_RCONF_MS = 80;
/// @brief Delay after AT+KMAC=1 so the MAC layer can re-initialize with the
///        new RCONF. Sending AT+TX before the MAC is ready causes the module
///        to reject the frame (observed as +ERROR=5 / KNS_STATUS_BAD_LEN).
///        SMD uses 200 ms here; same value reused for KIM2.
static constexpr uint16_t KIM2_DELAY_AFTER_KMAC_MS  = 200;
/// @brief Regulatory power level required by the KIM2 module for VLDA4 TX.
///        The default stored VLDA4 RCONF encodes 22 dBm, which is not
///        permitted — a compliant RCONF must be programmed at 27 dBm.
static constexpr int      KIM2_VLDA4_REQUIRED_DBM   = 27;
/// @brief Modulation name reported by +RCONF= for VLDA4 (KIM2 firmware string).
static constexpr const char *KIM2_MOD_NAME_VLDA4 = "VLDA4";
/// @brief Timeout for the synchronous +OK ACK of AT+TX. The KIM2 firmware
///        can take up to ~3 s to emit +OK (even though the +TX=<status>
///        completion arrives later via the async path). The 1 s default is
///        too short and causes a spurious "send_AT: timeout" warning while
///        the TX is actually succeeding. Match SMD AT's 5 s budget.
static constexpr uint16_t KIM2_TX_ACK_TIMEOUT_MS    = 5000;
/// @brief Delay after raising SAT_EXTWAKEUP to bring the module back from the
///        low-power mode it entered during a BLIND burst. The SMD driver budgets
///        50 ms for STANDBY and warns in state_idle_exit() that SHUTDOWN needs
///        more; the KIM2 firmware shipped as "LPM_SHDWN" takes the deeper path,
///        so budget its full power-on figure rather than the STANDBY one. Only
///        paid when the burst ended without the module talking to us.
static constexpr uint16_t KIM2_DELAY_WKUP_RESUME_MS  = 500;
/// @brief How early SAT_EXTWAKEUP is raised before the LAST retransmission of a
///        BLIND burst.
///
///        Measured on the bench 2026-08-25: with the pin released for the whole
///        burst, the closing +TX= NEVER arrives -- the 240 s window expires, the
///        TX is counted as failed and the module rail is cut. A framing error
///        (type=04) shows up 57 s after the release, that is when the module
///        wakes for its second transmission: our RX loses sync when the module's
///        TX line swings as it enters and leaves low power.
///        Raising the pin before the last transmission keeps the module awake
///        with a stable UART at the moment it emits the closing message, while
///        still letting it sleep through the earlier intervals -- which is where
///        nearly all the saving is.
/// @brief Poll interval of the receive state. The module emits about one
///        allcast message per second during a pass, so a 100 ms tick drains
///        the queue with a wide margin.
static constexpr uint16_t KIM2_RX_TICK_MS          = 100;
/// @brief Timeout for the asynchronous +OK of AT+DL=1. Per the AT API, this
///        +OK is the STACK's reply once reception has actually been started,
///        not a parser acknowledgement, so it can be preceded or interleaved by
///        unsolicited lines and takes longer than a plain command.
static constexpr uint16_t KIM2_RX_START_TIMEOUT_MS  = 3000;
/// @brief Per-attempt timeout of AT+DL=0, and number of attempts. While
///        receiving, the module has far less CPU left for the AT parser and a
///        single command can simply be missed, so a short timeout with retries
///        beats one long wait.
static constexpr uint16_t KIM2_RX_STOP_TIMEOUT_MS   = 1500;
static constexpr uint8_t  KIM2_RX_STOP_ATTEMPTS     = 5;
/// @brief Settle time before re-sending a stop the module did not accept. A
///        silent attempt has already burnt its timeout; a rejected one comes
///        back immediately and would otherwise hammer the parser.
static constexpr uint16_t KIM2_RX_STOP_RETRY_GAP_MS = 300;
/// @}
/// @brief Safety window passed to AT+DL=1,<window_s>. Reception is normally
///        bounded by the service, which calls stop_receive(); this second
///        barrier makes the MODULE stop by itself should the host never issue
///        AT+DL=0 (lock-up, reset). Generous compared to a satellite pass.
static constexpr unsigned int KIM2_RX_SAFETY_WINDOW_S = 1800;
/// @brief Cap on the allcast backlog kept between two state machine ticks.
static constexpr size_t KIM2_RX_QUEUE_MAX = 16;
/// @}

#define KIM2_STATE_CHANGE(x, y)                     \
	do {                                             \
		DEBUG_TRACE("KIM2::KIM2_STATE_CHANGE: " #x " -> " #y ); \
		m_state = y;                                 \
		state_ ## x ##_exit();                       \
		state_ ## y ##_enter();                      \
		run_state_machine();						 \
	} while (0)

#define KIM2_STATE_EQUAL(x) \
	(m_state == x)

#define KIM2_STATE_CALL(x) \
	do {                    \
		state_ ## x();      \
	} while (0)

extern Scheduler *system_scheduler;
extern ConfigurationStore *configuration_store;

// ============================================================================
// ARGOS_CACHED_MODULATION mapping
// The param uses the SmdArgosModulation convention (0=LDA2, 1=LDK, 2=VLDA4),
// documented in config_store.hpp and shared with SmdSat. KineisModulation uses
// a DIFFERENT order (LDK=0, LDA2=1, VLDA4=2), so we map explicitly — never cast.
// ============================================================================
static unsigned int kineis_mod_to_cached(KineisModulation m) {
    switch (m) {
        case KineisModulation::LDK:   return 1;
        case KineisModulation::VLDA4: return 2;
        case KineisModulation::LDA2:
        default:                      return 0;
    }
}

static KineisModulation cached_to_kineis_mod(unsigned int v) {
    switch (v) {
        case 1:  return KineisModulation::LDK;
        case 2:  return KineisModulation::VLDA4;
        case 0:
        default: return KineisModulation::LDA2;
    }
}

// ============================================================================
// Constructor / Destructor
// ============================================================================

KIM2Device::KIM2Device()
{
    m_tx_buffer.clear();
    m_packet_buffer.clear();
    m_state = KIM2ManagerState::power_off;
    m_tx_mode = KineisModulation::LDA2;
    // Load the last-known modulation from config instead of blindly defaulting
    // to LDA2. On a non-adaptive unit whose master RCONF encodes LDK/VLDA4, the
    // LDA2 default made the FIRST scheduling cycle after a cold boot build a
    // wrong-modulation packet — the service reads get_current_modulation()
    // BEFORE the module is powered/queried, so it would size the frame for LDA2
    // while the radio is LDK (12-byte frame on a 16-byte-fixed LDK radio →
    // +ERROR=5, one aborted TX per boot). state_init still re-confirms from
    // AT+RCONF=? and re-persists. Mirrors SmdSat's ARGOS_CACHED_MODULATION load.
    m_current_rconf_mode = KineisModulation::LDA2;
    if (configuration_store) {
        m_current_rconf_mode = cached_to_kineis_mod(
            configuration_store->read_param<unsigned int>(ParamID::ARGOS_CACHED_MODULATION));
    }
    m_timeout = {};
    m_stopping = false;
    m_cmd_is_ok = false;
    m_is_error = false;
    m_tx_done = false;
    m_tx_poll_counter = 0;
    m_rx_window_ended = false;
    m_rx_allcast_queue.clear();
    // Start in power_off — device will power on when send() is called
}

KIM2Device::~KIM2Device()
{
    stop_bridge();
    power_off_immediate();
}

// ============================================================================
// Bridge/passthrough mode — raw UART access for DTE KIMBR command
// ============================================================================

bool KIM2Device::start_bridge(KIM2Comm::PassthroughCallback rx_callback)
{
    if (m_bridge_active)
        return true;

    // Cancel any pending state machine task / timeout — bridge owns the UART
    cancel_timeout();
    system_scheduler->cancel_task(m_task);

    // Power on module hardware if not already on (required for UART communication).
    // If already powered (state=init/idle/transmit), we reuse the current GPIO state.
    bool was_off = (m_state == KIM2ManagerState::power_off);
    if (was_off) {
        GPIOPins::set(SAT_PWR_EN);
        // USR_NRST (broche 2): "Should not be left floating" (datasheet v0.4
        // §4.2). There is no external pull-up on this board: the reset is
        // released onto the INTERNAL pull-up, which gives a defined high level
        // while still letting an SWD probe drive reset without fighting it.
        GPIOPins::release_to_pullup(SAT_RESET);
        GPIOPins::set(SAT_EXTWAKEUP);
    }

    // Ensure UART is initialized (may be off if no TX happened yet)
    m_kim2_comm.init();

    // Wait for module boot if we just powered it on
    if (was_off) {
        PMU::delay_ms(KIM2_DELAY_POWER_ON_MS);
    }

    // Enable passthrough: raw UART RX goes to callback (line-framed with CRLF)
    m_kim2_comm.set_passthrough(true, rx_callback);
    m_bridge_active = true;

    DEBUG_INFO("KIM2Device: bridge mode ACTIVE (was_off=%u)", was_off ? 1 : 0);
    return true;
}

void KIM2Device::stop_bridge()
{
    if (!m_bridge_active)
        return;

    m_kim2_comm.set_passthrough(false);
    m_bridge_active = false;

    DEBUG_INFO("KIM2Device: bridge mode STOPPED");

    // Force power_off — bridge usage may have left the module in an arbitrary
    // AT protocol state. Next send() will do a clean power-on cycle via
    // start_device(). This avoids resuming into stale idle/transmit states.
    cancel_timeout();
    system_scheduler->cancel_task(m_task);
    m_kim2_comm.unsubscribe(*this);
    m_state = KIM2ManagerState::power_off;
    state_power_off_enter();  // Deinit UART, cut GPIOs, clear buffers
}

bool KIM2Device::bridge_send(const uint8_t* data, size_t len)
{
    if (!m_bridge_active)
        return false;
    return m_kim2_comm.send_raw_data(data, len);
}

void KIM2Device::bridge_process_rx()
{
    if (m_bridge_active)
        m_kim2_comm.process_rx();
}

// ============================================================================
// KineisDevice interface
// ============================================================================

/// @brief Pack payload with modulation-specific stuffing bits, queue for TX.
/// @param mode            Modulation (LDK, LDA2, VLDA4).
/// @param user_payload    Raw payload bytes.
/// @param payload_length  Payload length in bits.
void KIM2Device::send(const KineisModulation mode, const KineisPacket& user_payload, const unsigned int payload_length)
{
    // Reject TX while bridge is active — bridge owns the UART exclusively.
    // Stop the bridge first via stop_bridge() if a TX is needed.
    if (m_bridge_active) {
        DEBUG_WARN("KIM2Device::send: rejected — bridge mode active");
        notify(KineisEventDeviceError({}));
        return;
    }

    // Regulatory gate: VLDA4 was observed decoded at != 27 dBm on a previous
    // probe. Refuse to TX at a non-compliant power level. Caller (argos_tx_service)
    // should have already fallen back to LDK/LDA2 via ensure_modulation().
    if (mode == KineisModulation::VLDA4 && !m_vlda4_allowed) {
        DEBUG_ERROR("KIM2Device::send: VLDA4 disabled on this unit — required %d dBm not met",
                    KIM2_VLDA4_REQUIRED_DBM);
        notify(KineisEventDeviceError({}));
        return;
    }

    KineisPacket packet;
    uint16_t total_bits;
    uint16_t modulo;
    uint16_t stuffing_bits = 0;

    switch (mode)
    {
        case KineisModulation::LDK:
            if (payload_length > LDK_MAX_LENGTH_BITS)
            {
                DEBUG_ERROR("KIM2Device::send: LDK payload too long: %d bits (max %d)",
                    payload_length, LDK_MAX_LENGTH_BITS);
                // Signal the caller instead of dropping silently — otherwise the
                // service waits out its 30 s TX timeout with no event. The
                // ArgosTxService size guards should make this unreachable, but a
                // DeviceError lets any future caller reschedule with backoff.
                notify(KineisEventDeviceError({}));
                return;
            }
            stuffing_bits = LDK_MAX_LENGTH_BITS - payload_length;
            break;

        case KineisModulation::LDA2:
            if (payload_length > LDA2_MAX_LENGTH_BITS)
            {
                DEBUG_ERROR("KIM2Device::send: LDA2 payload too long: %d bits (max %d)",
                    payload_length, LDA2_MAX_LENGTH_BITS);
                notify(KineisEventDeviceError({}));
                return;
            }
            modulo = payload_length % 32;
            stuffing_bits = modulo ? (32 - modulo) : 0;
            break;

        case KineisModulation::VLDA4:
            if (payload_length > VLDA4_MAX_LENGTH_BITS)
            {
                DEBUG_ERROR("KIM2Device::send: VLDA4 payload too long: %d bits (max %d)",
                    payload_length, VLDA4_MAX_LENGTH_BITS);
                notify(KineisEventDeviceError({}));
                return;
            }
            stuffing_bits = VLDA4_MAX_LENGTH_BITS - payload_length;
            break;

        default:
            DEBUG_ERROR("KIM2Device::send: Unknown modulation type");
            // L1: notify like the sized branches above so the service reschedules
            // with backoff instead of silently waiting out its 30 s TX timeout.
            notify(KineisEventDeviceError({}));
            return;
    }
    DEBUG_TRACE("KIM2Device::send: adding %u stuffing bits for alignment", stuffing_bits);

    total_bits = payload_length + stuffing_bits;
    packet.assign(total_bits/8, 0);
    packet.replace(0, user_payload.size(), user_payload);

    // Add any stuffing bits
    unsigned int payload_bits_remaining = stuffing_bits;
    unsigned int op_offset = payload_length;
    while (payload_bits_remaining) {
        unsigned int bits = std::min(8U, payload_bits_remaining);
        payload_bits_remaining -= bits;
        PACK_BITS(0, packet, op_offset, bits);
    }

    // Store as hex string for AT+TX command
    DEBUG_TRACE("KIM2Device::send: data[%u]=%s", total_bits, Binascii::hexlify(packet).c_str());
    m_packet_buffer = Binascii::hexlify(packet).c_str();
    m_tx_mode = mode;

    // Request power on (if not already running)
    start_device();
}

void KIM2Device::stop_send() {
    DEBUG_TRACE("KIM2Device::stop_send");
    m_packet_buffer.clear();
    m_tx_buffer.clear();
}

/// @brief Arm continuous downlink reception. The AT+DL=1 itself is issued by
///        state_receive_enter() so the command always runs in state machine
///        context, never from the caller's.
/// @note @p mode is ignored: the downlink frequency is chosen by the module's
///       stack (observed around 400.64 MHz), while the RCONF band configured
///       for TX covers the uplink only. Nothing to re-program before an RX.
void KIM2Device::start_receive(const KineisModulation mode)
{
    (void)mode;

    m_rx_stop_requested = false;
    if (m_rx_requested) {
        DEBUG_TRACE("KIM2Device::start_receive: already requested");
        start_device();
        return;
    }
    m_rx_requested = true;
    DEBUG_INFO("KIM2Device::start_receive: requested");

    // Request power on (if not already running)
    start_device();
}

bool KIM2Device::stop_receive()
{
    if (!m_rx_requested && !m_rx_active) {
        DEBUG_TRACE("KIM2Device::stop_receive: no reception pending");
        return false;
    }

    DEBUG_INFO("KIM2Device::stop_receive");
    m_rx_requested = false;
    m_rx_stop_requested = true;

    // Stop the radio NOW rather than on the next state machine tick. The
    // caller is typically the RX service committing its AOP table: it goes on
    // to write the database and the parameters to flash, then a full pass
    // prediction is recomputed on service_complete(). That is seconds during
    // which the state machine never runs while the module keeps streaming
    // about one message per second into a buffer nobody drains.
    if (KIM2_STATE_EQUAL(receive))
        perform_rx_stop();
    else
        finish_rx_session();   // never actually started, close the books now

    return true;
}

void KIM2Device::set_frequency(double freq_mhz)
{
    (void)freq_mhz;
    // KIM2 frequency is configured via RCONF, not a separate command
}

void KIM2Device::set_tcxo_warmup_time(unsigned int ms)
{
    (void)ms;
    // KIM2 has no configurable TCXO AT command — warmup is handled
    // internally by the module firmware. The observed ~3 s delay between
    // AT+TX and the +OK ACK is the module's internal TCXO warmup + actual
    // RF transmission, which is why KIM2_TX_ACK_TIMEOUT_MS is set to 5 s.
}

// ============================================================================
// Runtime modulation switching
// ============================================================================

/// @brief Runtime modulation switch: write RCONF, validate, then reload KMAC.
/// @param mode       Target modulation.
/// @param rconf_hex  32-char hex RCONF string for the target modulation.
/// @return true on success, false on AT failure or VLDA4 gated off.
/// @note Programming a VLDA4 RCONF at != 27 dBm is rejected here — the flag
///       @c m_vlda4_allowed is cleared and the KMAC reload is skipped so the
///       module is not left in a non-compliant armed state.
bool KIM2Device::switch_modulation(KineisModulation mode, const std::string& rconf_hex) {
    if (mode == m_current_rconf_mode) {
        DEBUG_TRACE("KIM2Device::%s: already in target modulation %d", __func__, static_cast<int>(mode));
        return true;
    }

    DEBUG_INFO("KIM2Device::%s: switching %d -> %d", __func__, static_cast<int>(m_current_rconf_mode), static_cast<int>(mode));

    if (rconf_hex.size() != 32) {
        DEBUG_ERROR("KIM2Device::%s: invalid RCONF hex length %u", __func__, static_cast<unsigned>(rconf_hex.size()));
        return false;
    }

    // Regulatory gate: a previous probe already flagged VLDA4 as non-compliant.
    // Refuse without ever touching the RF config so the module stays on the
    // current (compliant) modulation.
    if (mode == KineisModulation::VLDA4 && !m_vlda4_allowed) {
        DEBUG_WARN("KIM2Device::%s: VLDA4 gated off (non-27dBm) — refusing switch", __func__);
        return false;
    }

    // If device is powered off, just store the target mode.
    // state_init() will write the correct RCONF at next power-on.
    if (m_state == KIM2ManagerState::power_off) {
        DEBUG_INFO("KIM2Device::%s: device OFF, deferring to next init", __func__);
        m_current_rconf_mode = mode;
        return true;
    }

    // Write RCONF + read back, enforcing the VLDA4-at-27dBm rule. On rejection
    // m_vlda4_allowed is cleared and we bail out before KMAC is reloaded.
    KIM2::RConfDecoded sw_decoded;
    if (!write_and_validate_rconf(rconf_hex, mode, &sw_decoded)) {
        DEBUG_ERROR("KIM2Device::%s: RCONF rejected (mode=%d)", __func__, static_cast<int>(mode));
        return false;
    }

    // Trust the module's read-back modulation, not the requested tag. A
    // mislabeled RCONF (e.g. an LDK config stored in the LDA2 slot) would leave
    // the radio on a different modulation than we cache, so the next send()
    // would zero-pad the frame to the WRONG length (only LDA2 is variable; LDK
    // and VLDA4 are fixed) → AT+TX rejected with +ERROR=5 (BAD_LEN). Refuse the
    // switch and cache the actual modulation so the caller can react/fall back.
    KIM2::ModulationVerdict sw_v = KIM2::verify_modulation(mode, sw_decoded.modulation);
    if (sw_v.mismatch) {
        DEBUG_ERROR("KIM2Device::%s: RCONF for mode %d physically encodes %s (mode %d) — refusing switch (check provisioning)",
            __func__, static_cast<int>(mode), sw_decoded.modulation.c_str(), static_cast<int>(sw_v.actual));
        m_current_rconf_mode = sw_v.actual;
        cache_current_modulation();
        return false;
    }

    if (!load_kmac()) {
        DEBUG_ERROR("KIM2Device::%s: failed to reload KMAC", __func__);
        return false;
    }
    PMU::delay_ms(KIM2_DELAY_AFTER_KMAC_MS);

    m_current_rconf_mode = mode;
    cache_current_modulation();
    DEBUG_INFO("KIM2Device::%s: modulation switched OK", __func__);
    return true;
}

KineisModulation KIM2Device::get_current_modulation() const {
    return m_current_rconf_mode;
}

void KIM2Device::cache_current_modulation() {
    if (configuration_store) {
        configuration_store->write_param(ParamID::ARGOS_CACHED_MODULATION,
                                         kineis_mod_to_cached(m_current_rconf_mode));
    }
}

// ============================================================================
// Credential read-back (SATVF)
// ============================================================================

/// @brief Actively read ID, address and decoded RCONF from the KIM2 module.
///
/// When the device is stopped, this does a synchronous power-on / ping /
/// power-off cycle so SATVF always returns fresh values (not stale cache).
/// If the state machine is currently running (init/idle/transmit) we reuse
/// the live UART session and leave power state untouched.
///
/// @note KIM2 exposes no AT+SECKEY command (unlike SMD), so @p seckey is
///       always cleared. @p radioconf carries the decoded
///       "freq_min,freq_max,mod_type,rf_level" string from AT+RCONF=? —
///       useful for SATVF diagnostics (e.g. confirming the active modulation).
void KIM2Device::read_credentials(unsigned int *dec_id, unsigned int *address,
                                   std::string *seckey, std::string *radioconf)
{
    DEBUG_TRACE("KIM2Device::read_credentials");

    // Zero outputs up-front so callers see a clean state on early return.
    if (dec_id)    *dec_id = 0;
    if (address)   *address = 0;
    if (seckey)    seckey->clear();
    if (radioconf) radioconf->clear();

    // Bridge owns the UART — can't issue AT reads. Fall back to cached values.
    if (m_bridge_active) {
        DEBUG_WARN("KIM2Device::read_credentials: bridge active, returning cached values");
        if (dec_id)    *dec_id    = m_kim2_comm.m_kineis_id;
        if (address)   *address   = m_kim2_comm.m_hex_addr;
        if (radioconf) *radioconf = m_kim2_comm.m_rconf_info;
        return;
    }

    const bool was_off = (m_state == KIM2ManagerState::power_off);

    if (was_off) {
        // Freeze the state machine so it doesn't race with our synchronous reads.
        cancel_timeout();
        system_scheduler->cancel_task(m_task);

        GPIOPins::set(SAT_PWR_EN);
        // USR_NRST (broche 2): "Should not be left floating" (datasheet v0.4
        // §4.2). There is no external pull-up on this board: the reset is
        // released onto the INTERNAL pull-up, which gives a defined high level
        // while still letting an SWD probe drive reset without fighting it.
        GPIOPins::release_to_pullup(SAT_RESET);
        GPIOPins::set(SAT_EXTWAKEUP);
        m_kim2_comm.init();
        m_kim2_comm.subscribe(*this);
        PMU::delay_ms(KIM2_DELAY_POWER_ON_MS);

        if (!send_AT(AT_PING)) {
            DEBUG_WARN("KIM2Device::read_credentials: ping failed — module not responding");
            m_kim2_comm.unsubscribe(*this);
            m_kim2_comm.deinit();
            GPIOPins::clear(SAT_EXTWAKEUP);
            // Reset asserted BEFORE cutting the supply, and held low afterwards: a
            // floating reset line on a powered-down module is what the datasheet
            // proscrit explicitement.
            GPIOPins::init_pin(SAT_RESET);
            GPIOPins::clear(SAT_RESET);
            GPIOPins::clear(SAT_PWR_EN);
            return;
        }
    }

    if (send_AT(AT_GET_ID)) {
        if (dec_id) *dec_id = m_kim2_comm.m_kineis_id;
    } else {
        DEBUG_WARN("KIM2Device::read_credentials: AT+ID=? failed");
    }

    if (send_AT(AT_GET_ADDR)) {
        if (address) *address = m_kim2_comm.m_hex_addr;
    } else {
        DEBUG_WARN("KIM2Device::read_credentials: AT+ADDR=? failed");
    }

    if (send_AT(AT_GET_RCONF)) {
        if (radioconf) *radioconf = m_kim2_comm.m_rconf_info;

        // SATVF-time regulatory check: if the module currently reports VLDA4
        // at anything other than 27 dBm, update the gate so subsequent TX /
        // switch_modulation calls refuse to emit on non-compliant power.
        KIM2::RConfDecoded decoded = KIM2::parse_rconf_info(m_kim2_comm.m_rconf_info);
        if (decoded.valid &&
            decoded.modulation == KIM2_MOD_NAME_VLDA4 &&
            decoded.rf_level_dbm != KIM2_VLDA4_REQUIRED_DBM) {
            DEBUG_WARN("KIM2Device::read_credentials: VLDA4 at %d dBm (required %d) — gating VLDA4 off",
                       decoded.rf_level_dbm, KIM2_VLDA4_REQUIRED_DBM);
            m_vlda4_allowed = false;
        }

        // Align the cached modulation from this live read-back. We don't write
        // credentials on KIM2 (ID/ADDR are burned in, RCONF is re-applied each
        // power-on), so SATVF is the natural place to seed the cache: running it
        // at provisioning/verification persists the module's ACTUAL modulation
        // into ARGOS_CACHED_MODULATION (and the in-RAM tag), so the next cold
        // boot starts on the right modulation with no aborted first TX. Mirrors
        // state_init's re-tag. Skip a VLDA4 that was just gated off — caching a
        // non-compliant modulation would only force a fallback next boot.
        std::optional<KineisModulation> actual = KIM2::mod_from_name(decoded.modulation);
        if (decoded.valid && actual.has_value() &&
            actual.value() != m_current_rconf_mode &&
            !(actual.value() == KineisModulation::VLDA4 && !m_vlda4_allowed)) {
            DEBUG_INFO("KIM2Device::read_credentials: aligning cached modulation to %s (was %d)",
                       decoded.modulation.c_str(), static_cast<int>(m_current_rconf_mode));
            m_current_rconf_mode = actual.value();
            cache_current_modulation();
        }
    } else {
        DEBUG_WARN("KIM2Device::read_credentials: AT+RCONF=? failed");
    }

    if (was_off) {
        m_kim2_comm.unsubscribe(*this);
        m_kim2_comm.deinit();
        GPIOPins::clear(SAT_EXTWAKEUP);
        // Reset asserted BEFORE cutting the supply, and held low afterwards: a
        // floating reset line on a powered-down module is what the datasheet
        // proscrit explicitement.
        GPIOPins::init_pin(SAT_RESET);
        GPIOPins::clear(SAT_RESET);
        GPIOPins::clear(SAT_PWR_EN);
    }
}

/// @brief Program the configured RCONF, read it back, and re-seed the modulation
/// cache. Best-effort synchronous power-cycle if the module is off.
///
/// Unlike read_credentials() which only READS the module's power-on RCONF (a
/// hardware default, not necessarily the configured master), this WRITES the
/// configured master/per-mode RCONF first, then reads it back — so the cache
/// reflects the modulation a TX session will actually program. Fixes the
/// wrong-modulation first TX after a fresh config / RCONF edit. RCONF lives in
/// the module's RAM and is lost on the trailing power-off; only the persistent
/// cache (ARGOS_CACHED_MODULATION) is the goal here.
bool KIM2Device::resync_rconf_cache()
{
    if (m_bridge_active) {
        DEBUG_WARN("KIM2Device::resync_rconf_cache: bridge active — skipped");
        return false;
    }

    const bool was_off = (m_state == KIM2ManagerState::power_off);
    if (was_off) {
        cancel_timeout();
        system_scheduler->cancel_task(m_task);
        GPIOPins::set(SAT_PWR_EN);
        // USR_NRST (broche 2): "Should not be left floating" (datasheet v0.4
        // §4.2). There is no external pull-up on this board: the reset is
        // released onto the INTERNAL pull-up, which gives a defined high level
        // while still letting an SWD probe drive reset without fighting it.
        GPIOPins::release_to_pullup(SAT_RESET);
        GPIOPins::set(SAT_EXTWAKEUP);
        m_kim2_comm.init();
        m_kim2_comm.subscribe(*this);
        PMU::delay_ms(KIM2_DELAY_POWER_ON_MS);
        if (!send_AT(AT_PING)) {
            DEBUG_WARN("KIM2Device::resync_rconf_cache: ping failed — module not responding");
            m_kim2_comm.unsubscribe(*this);
            m_kim2_comm.deinit();
            GPIOPins::clear(SAT_EXTWAKEUP);
            // Reset asserted BEFORE cutting the supply, and held low afterwards: a
            // floating reset line on a powered-down module is what the datasheet
            // proscrit explicitement.
            GPIOPins::init_pin(SAT_RESET);
            GPIOPins::clear(SAT_RESET);
            GPIOPins::clear(SAT_PWR_EN);
            return false;
        }
    }

    bool ok = false;
    std::string rconf = load_rconf_for_mode(m_current_rconf_mode);
    if (rconf.size() == 32) {
        KIM2::RConfDecoded decoded;
        if (write_and_validate_rconf(rconf, m_current_rconf_mode, &decoded) && decoded.valid) {
            std::optional<KineisModulation> actual = KIM2::mod_from_name(decoded.modulation);
            if (actual.has_value() &&
                !(actual.value() == KineisModulation::VLDA4 && !m_vlda4_allowed)) {
                if (actual.value() != m_current_rconf_mode)
                    DEBUG_INFO("KIM2Device::resync_rconf_cache: cache %d -> %d (%s)",
                               static_cast<int>(m_current_rconf_mode),
                               static_cast<int>(actual.value()), decoded.modulation.c_str());
                m_current_rconf_mode = actual.value();
                cache_current_modulation();
                ok = true;
            }
        } else {
            DEBUG_WARN("KIM2Device::resync_rconf_cache: RCONF program/verify failed");
        }
    } else {
        DEBUG_WARN("KIM2Device::resync_rconf_cache: no RCONF configured (len=%u)",
                   static_cast<unsigned>(rconf.size()));
    }

    if (was_off) {
        m_kim2_comm.unsubscribe(*this);
        m_kim2_comm.deinit();
        GPIOPins::clear(SAT_EXTWAKEUP);
        // Reset asserted BEFORE cutting the supply, and held low afterwards: a
        // floating reset line on a powered-down module is what the datasheet
        // proscrit explicitement.
        GPIOPins::init_pin(SAT_RESET);
        GPIOPins::clear(SAT_RESET);
        GPIOPins::clear(SAT_PWR_EN);
    }
    return ok;
}

// ============================================================================
// KIM2Comm event handlers (ISR context)
// ============================================================================

void KIM2Device::react(const KIM2CommEventRespOk&) {
    m_cmd_is_ok = true;
}

void KIM2Device::react(const KIM2CommEventTxDone&) {
    m_tx_done = true;
}

void KIM2Device::react(const KIM2CommEventRespError&) {
    m_is_error = true;
}

void KIM2Device::react(const KIM2CommEventAllcast& e) {
    // process_rx() context (possibly inside a blocking send_AT()): queue only,
    // the dispatch to listeners happens on the next state machine tick.
    if (m_rx_allcast_queue.size() >= KIM2_RX_QUEUE_MAX) {
        DEBUG_WARN("KIM2Device: allcast queue full, dropping oldest");
        m_rx_allcast_queue.pop_front();
    }
    m_rx_allcast_queue.push_back(e.hex);
}

void KIM2Device::react(const KIM2CommEventRxWindowEnd&) {
    m_rx_window_ended = true;
}

void KIM2Device::react(const KIM2CommEventUartError& err) {
    // Snapshot the state at error time (ISR context) — the deferred log task below
    // may run after the state machine has advanced past the boot window.
    KIM2ManagerState st = m_state;
    system_scheduler->post_task_prio([err, st]() {
        // 0x04 = UART framing error. During the power-on/init window the KIM TX
        // line/baud settles and produces a one-shot framing glitch on our RX; the
        // AT comm recovers immediately (next RCONF read / AT+TX succeed). That case
        // is benign, so log it at TRACE — hidden at the normal DEBUG_LEVEL=3 (shown
        // only at level 4), so it no longer perturbs the user. A UART error OUTSIDE
        // the boot window is unexpected and stays visible at WARN.
        bool boot_window = (st == power_on || st == init);
        if (err.error_type == 0x04 && boot_window)
            DEBUG_TRACE("KIM2CommEventUartError: type=04 (boot transition, expected)");
        else
            DEBUG_WARN("KIM2CommEventUartError: type=%02x (state=%d)", err.error_type, static_cast<int>(st));
    }, "Debug");
}

// ============================================================================
// AT command helper
// ============================================================================

/// @brief Send AT command and busy-wait for +OK or +ERROR response.
/// @param cmd         AT command type.
/// @param params      Optional parameter string.
/// @param timeout_ms  Max wait time in ms (default 1000).
/// @return true if +OK received before timeout.
/// @note Enforces the KIM2 Integration Manual v0.8 timing constraint:
///       "User shall wait at minimum 10ms before sending a new command
///       after previous is completed." The 10ms gap is applied after the
///       response is received so the next send_AT() call is already clear
///       to transmit.
bool KIM2Device::send_AT(ATCmd cmd, const std::optional<std::string>& params, uint16_t timeout_ms)
{
    // Drain whatever is still in the UART buffer BEFORE arming the flags: a
    // late reply to the previous command (the module answers a command issued
    // while it was busy several seconds later) would otherwise be parsed during
    // this command's wait and taken for its own verdict. 
    m_kim2_comm.process_rx();

    m_cmd_is_ok = false;
    m_is_error = false;

    m_kim2_comm.send(cmd, params);

    // Busy-wait for UART response — blocking but bounded.
    // AT protocol is synchronous; async would require full state machine rewrite.
    while (!m_cmd_is_ok && !m_is_error && timeout_ms != 0) {
        PMU::delay_ms(1);
        m_kim2_comm.process_rx();  // Drain ISR buffer → parse → notify
        timeout_ms--;
    }

    if(timeout_ms == 0) {
        DEBUG_WARN("KIM2Device::send_AT: timeout (cmd=%d)", static_cast<int>(cmd));
    }

    // Mandatory inter-command gap (KIM2 manual v0.8, §3.A timing constraints).
    PMU::delay_ms(10);

    return m_cmd_is_ok && !m_is_error;
}

// ============================================================================
// Timeout management
// ============================================================================

// True when the BLIND MAC profile is active: ARGOS_BLIND_EN set AND the effective
// mode is not SURFACING_BURST (which runs its own progressive Doppler cascade —
// a module-owned retx burst would double-transmit). Fills clamped retx_nb (1..127)
// and retx_period_s. Reads the raw ARGOS_MODE (LB/OoZ overrides to surfacing are
// not resolved here — do not pair BLIND with a surfacing regime).
static bool kim2_blind_active(unsigned int& retx_nb, unsigned int& retx_period_s) {
    if (!configuration_store) return false;
    // get_argos_configuration resolves the EFFECTIVE mode (LB/OoZ/HAULED) and
    // clears blind_en for SURFACING_BURST/DOPPLER. evaluate() is cached (500 ms).
    ArgosConfig cfg;
    configuration_store->get_argos_configuration(cfg);
    if (!cfg.blind_en) return false;
    retx_nb = cfg.blind_retx_nb;
    if (retx_nb < 1) retx_nb = 1; else if (retx_nb > 127) retx_nb = 127;
    retx_period_s = cfg.blind_retx_period_s;
    return true;
}

bool KIM2Device::load_kmac() {
    unsigned int rn = 0, period = 0;
    if (kim2_blind_active(rn, period)) {
        // KNS_MAC_BLIND_usrCfg_t packed LE: {int8 retx_nb, uint8 nb_parallel=1, uint32 retx_period_s, int8 per_offset=0}
        uint8_t ctx[7] = {
            (uint8_t)rn, 1,
            (uint8_t)(period & 0xFF), (uint8_t)((period >> 8) & 0xFF),
            (uint8_t)((period >> 16) & 0xFF), (uint8_t)((period >> 24) & 0xFF),
            0 };
        std::string hex = Binascii::hexlify(std::string(reinterpret_cast<const char*>(ctx), sizeof(ctx)));
        if (send_AT(AT_SET_KMAC_BLIND, hex)) {
            DEBUG_INFO("KIM2Device::load_kmac: BLIND loaded (retx_nb=%u period=%us)", rn, period);
            m_kim2_comm.set_blind_active(true);   // +TX will be handler-prefixed
            return true;
        }
        DEBUG_WARN("KIM2Device::load_kmac: BLIND (AT+KMAC=2) rejected — falling back to BASIC");
    }
    m_kim2_comm.set_blind_active(false);          // basic +TX (no handler prefix)
    return send_AT(AT_SET_KMAC_BASIC);
}

void KIM2Device::initiate_timeout(unsigned int timeout_ms) {
	cancel_timeout();
	m_timeout.handle = system_scheduler->post_task_prio([this]() {
		on_timeout();
	}, "Timeout", Scheduler::DEFAULT_PRIORITY, timeout_ms);
}

void KIM2Device::on_timeout() {
    DEBUG_ERROR("KIM2Device::on_timeout");
    m_is_error = true;
}

void KIM2Device::cancel_timeout() {
	system_scheduler->cancel_task(m_timeout.handle);
}

// ============================================================================
// Power management
// ============================================================================

/// @brief Power on module and start state machine (no-op if already running).
void KIM2Device::start_device()
{
    if (m_state != KIM2ManagerState::power_off) {
        m_stopping = false;
        DEBUG_TRACE("KIM2Device::start: already running in state=%u", static_cast<unsigned int>(m_state));
        run_state_machine(0);
        return;
    }

    m_stopping = false;
    KIM2_STATE_CHANGE(power_off, power_on);
    // Schedule the module boot-settle (was a blocking 500 ms PMU::delay_ms in
    // state_power_on_enter) instead of freezing the scheduler. The macro above
    // reposts state_power_on() at 100 ms; override it to KIM2_DELAY_POWER_ON_MS
    // so the first AT+PING only fires once the module has finished booting.
    run_state_machine(KIM2_DELAY_POWER_ON_MS);
}

/// @brief Immediate power off — cancel tasks, uninit UART, cut GPIO power.
void KIM2Device::power_off_immediate(void)
{
    DEBUG_TRACE("KIM2Device::power_off_immediate");

    if (!KIM2_STATE_EQUAL(power_off)) {
        system_scheduler->cancel_task(m_task);
        cancel_timeout();
        // Powering off while receiving: the module dies with the rail, so no
        // AT+DL=0 is possible — just close the session so the service still
        // gets its KineisEventRxStopped and its RX-on time accounting.
        finish_rx_session();
        m_rx_requested = false;
        KIM2_STATE_CHANGE(idle, power_off);
    }
}

// ============================================================================
// State machine
// ============================================================================

/// @brief Dispatch to the current state handler.
void KIM2Device::state_machine(void)
{
	switch (m_state) {
	case KIM2ManagerState::power_off:
		KIM2_STATE_CALL(power_off);
		break;
	case KIM2ManagerState::power_on:
		KIM2_STATE_CALL(power_on);
		break;
	case KIM2ManagerState::init:
		KIM2_STATE_CALL(init);
		break;
	case KIM2ManagerState::idle:
		KIM2_STATE_CALL(idle);
		break;
	case KIM2ManagerState::transmit:
		KIM2_STATE_CALL(transmit);
		break;
	case KIM2ManagerState::receive:
		KIM2_STATE_CALL(receive);
		break;
	case KIM2ManagerState::error:
		KIM2_STATE_CALL(error);
		break;
	default:
		break;
	}
}

/// @brief Schedule the next state machine tick after delay_ms.
/// @param delay_ms  Delay before next tick (default 100 ms).
void KIM2Device::run_state_machine(uint16_t delay_ms)
{
    system_scheduler->cancel_task(m_task);
    m_task = system_scheduler->post_task_prio([this]() {
        state_machine();
    }, "KIM2StateMachine", Scheduler::DEFAULT_PRIORITY, delay_ms);
}

// ============================================================================
// State: power_off
// ============================================================================

/// @brief Power off: uninit UART, cut GPIO power, clear buffers.
void KIM2Device::state_power_off_enter()
{
    DEBUG_INFO("KIM2Device::state_power_off_enter");
    m_kim2_comm.deinit();
    GPIOPins::clear(SAT_EXTWAKEUP);
    // Reset asserted BEFORE cutting the supply, and held low afterwards: a
    // floating reset line on a powered-down module is exactly what the datasheet
    // forbids.
    GPIOPins::init_pin(SAT_RESET);
    GPIOPins::clear(SAT_RESET);
    GPIOPins::clear(SAT_PWR_EN);
    m_tx_buffer.clear();
    m_packet_buffer.clear();

    notify(KineisEventPowerOff({}));
}

void KIM2Device::state_power_off()
{
    ;
}

void KIM2Device::state_power_off_exit()
{
    ;
}

// ============================================================================
// State: power_on
// ============================================================================

/// @brief Power on: enable SAT_PWR_EN + EXTWAKEUP, init UART. The module
///        boot-settle (KIM2_DELAY_POWER_ON_MS) is SCHEDULED by start_device
///        (run_state_machine), not busy-waited, so the scheduler stays free.
void KIM2Device::state_power_on_enter()
{
    GPIOPins::set(SAT_PWR_EN);
    // USR_NRST (broche 2): "Should not be left floating" (datasheet v0.4
    // §4.2). There is no external pull-up on this board: the reset is released
    // onto the INTERNAL pull-up, which gives a defined high level while still
    // letting an SWD probe drive reset without fighting it.
    GPIOPins::release_to_pullup(SAT_RESET);
    GPIOPins::set(SAT_EXTWAKEUP);
    m_kim2_comm.init();
    m_kim2_comm.subscribe(*this);
}

void KIM2Device::state_power_on()
{
    DEBUG_INFO("KIM2Device::state_power_on");
    if(send_AT(AT_PING))
    {
        KIM2_STATE_CHANGE(power_on, init);
    }
    else
    {
        KIM2_STATE_CHANGE(power_on, error);
    }
}

void KIM2Device::state_power_on_exit()
{
    ;
}

// ============================================================================
// State: init
// ============================================================================

void KIM2Device::state_init_enter()
{
    ;
}

/// @brief Init: read ID/ADDR, write RCONF + KMAC → transition to idle.
/// @note Per KIM2 Integration Manual v0.8, RCONF is kept in RAM and must be
///       reapplied after every power-on (SAVE_RCONF is discouraged for
///       normal use). KMAC=1 (basic MAC profile) must be set after RCONF
///       and before any AT+TX.
/// @note Regulatory gate: VLDA4 is only permitted at 27 dBm on KIM2. If the
///       decoded RCONF reports VLDA4 at a lower level, we fall back to
///       LDK → LDA2 (adaptive and non-adaptive both), trigger a red error
///       LED, and refuse to enter idle when no compliant fallback exists.
void KIM2Device::state_init()
{
    // Fresh session: re-enable VLDA4 so a newly uploaded compliant RCONF is
    // re-probed on this boot instead of staying latched off from a prior boot.
    m_vlda4_allowed = true;


    // Read credentials from module if not already known
    if(m_kim2_comm.m_kineis_id == 0 && m_kim2_comm.m_hex_addr == 0)
    {
        if(!send_AT(AT_GET_ID))
        {
            DEBUG_ERROR("KIM2Device::state_init: can not read ID");
            KIM2_STATE_CHANGE(init, error);
            return;
        }
        DEBUG_TRACE("KIM2Device::state_init ID:%d", m_kim2_comm.m_kineis_id);
        configuration_store->write_param(ParamID::ARGOS_DECID, m_kim2_comm.m_kineis_id);

        if(!send_AT(AT_GET_ADDR))
        {
            DEBUG_ERROR("KIM2Device::state_init: can not read ADDR");
            KIM2_STATE_CHANGE(init, error);
            return;
        }
        DEBUG_TRACE("KIM2Device::state_init ADDR:%x", m_kim2_comm.m_hex_addr);
        configuration_store->write_param(ParamID::ARGOS_HEXID, m_kim2_comm.m_hex_addr);
    }

    // Read the firmware build string once per session and evaluate the RX gate.
    check_rx_firmware_support();

    // Configure RCONF for the target modulation, then start basic MAC profile.
    // When adaptive modulation is ON, use the per-modulation RCONF matching
    // m_current_rconf_mode (set by switch_modulation() while device was OFF).
    // When adaptive is OFF, use the master RCONF entered by the user.
    bool adaptive = configuration_store->read_param<bool>(ParamID::ARGOS_ADAPTIVE_MODULATION);
    KineisModulation target_mode = m_current_rconf_mode;
    std::string rconf = load_rconf_for_mode(target_mode);
    if (adaptive) {
        DEBUG_INFO("KIM2Device::state_init: adaptive ON, using RCONF for mode %d", static_cast<int>(target_mode));
    }
    if (rconf.empty()) {
        rconf = "03921fb104b92859209b18abd009de96"; // Default: ESS4 - LDK - 27dBm
        target_mode = KineisModulation::LDK;         // default matches LDK
        m_current_rconf_mode = target_mode;
        DEBUG_WARN("KIM2Device::state_init: RCONF empty, using default LDK fallback");
    }

    // Program + validate. Returns false on AT error OR on VLDA4-not-27dBm
    // rejection (in which case m_vlda4_allowed is now false).
    KIM2::RConfDecoded init_decoded;
    if (!write_and_validate_rconf(rconf, target_mode, &init_decoded)) {
        // Try to recover before giving up: if the user configured a compliant
        // non-VLDA4 fallback (LDK or LDA2, in that order), program it instead
        // and red-blink so the failure is visible. Applies to both adaptive
        // and non-adaptive — in adaptive mode VLDA4 is simply dropped from
        // the rotation for this session.
        DEBUG_WARN("KIM2Device::state_init: primary RCONF rejected — attempting fallback");
        LEDState::dispatch<SetLEDError>({});

        const KineisModulation fallback_chain[] = {
            KineisModulation::LDK,
            KineisModulation::LDA2,
        };
        bool recovered = false;
        for (auto fb : fallback_chain) {
            if (fb == target_mode) continue;  // already tried
            std::string fb_rconf = load_rconf_for_mode(fb);
            if (fb_rconf.size() != 32) {
                DEBUG_WARN("KIM2Device::state_init: no stored RCONF for fallback mode %d", static_cast<int>(fb));
                continue;
            }
            if (write_and_validate_rconf(fb_rconf, fb, &init_decoded)) {
                target_mode = fb;
                rconf = fb_rconf;
                m_current_rconf_mode = fb;
                DEBUG_INFO("KIM2Device::state_init: fallback to mode %d OK", static_cast<int>(fb));
                recovered = true;
                break;
            }
        }

        if (!recovered) {
            DEBUG_ERROR("KIM2Device::state_init: no compliant RCONF available — entering error");
            KIM2_STATE_CHANGE(init, error);
            return;
        }
    }

    // Non-adaptive mode: the master RCONF (ARGOS_RADIOCONF) is opaque/encrypted
    // hex — we can't tell locally which modulation it encodes. The module's
    // AT+RCONF=? readback reports the actual modulation, so we cache it here so
    // that the service (via get_current_modulation()) builds the next packet
    // at the correct size. Client bug otherwise: master encodes LDK but the
    // service hardcodes LDA2 → AT+TX 24 B to LDK module → +ERROR=5.
    if (!adaptive && init_decoded.valid) {
        auto actual = KIM2::mod_from_name(init_decoded.modulation);
        if (actual.has_value() && actual.value() != m_current_rconf_mode) {
            DEBUG_INFO("KIM2Device::state_init: non-adaptive master RCONF encodes %s (was tracking %d) — aligning",
                       init_decoded.modulation.c_str(), static_cast<int>(m_current_rconf_mode));
            m_current_rconf_mode = actual.value();
        }
    }

    if(!load_kmac())
    {
        DEBUG_ERROR("KIM2Device::state_init: can not set KMAC");
        KIM2_STATE_CHANGE(init, error);
        return;
    }
    // KMAC settle (KIM2_DELAY_AFTER_KMAC_MS) is scheduled after the transition to
    // idle (see run_state_machine at the end) instead of blocking the scheduler here.
    DEBUG_TRACE("KIM2Device::state_init RCONF set and KMAC=1 activated");
    if (!adaptive) {
        configuration_store->write_param(ParamID::ARGOS_RADIOCONF, rconf);
    }
    // Persist the confirmed modulation so the next cold boot starts on the right
    // one (no LDA2-default first-cycle desync). m_current_rconf_mode here is the
    // value validated above (master readback re-tag, fallback, or default-LDK).
    cache_current_modulation();

    KIM2_STATE_CHANGE(init, idle);
    // Give the MAC layer KIM2_DELAY_AFTER_KMAC_MS to re-initialize before idle's
    // first tick can launch an AT+TX (sending too early → +ERROR=5 BAD_LEN). The
    // macro above reposts at 100 ms; override to the 200 ms KMAC settle.
    run_state_machine(KIM2_DELAY_AFTER_KMAC_MS);
}

void KIM2Device::state_init_exit()
{
    ;
}

/// @brief Return KIM2 module identifier (AT+ID=?) as a hex string.
/// KIM2 firmware does not expose a firmware-version command, so we report the
/// hardware Kineis ID instead — it is unique per module and sufficient for the
/// GUI to identify the communication module.
std::string KIM2Device::get_firmware_version()
{
    if (m_state == power_off) {
        DEBUG_WARN("KIM2Device::get_firmware_version: module is powered off");
        return "";
    }
    if (!send_AT(AT_GET_ID)) {
        DEBUG_WARN("KIM2Device::get_firmware_version: AT+ID=? failed");
        return "";
    }
    char buf[16];
    snprintf(buf, sizeof(buf), "KIM2-%08X", m_kim2_comm.m_kineis_id);
    return std::string(buf);
}

// ============================================================================
// State: idle (with configurable timeout like SMD)
// ============================================================================

void KIM2Device::state_idle_enter()
{
    if (m_idle_timeout_ms > 0) {
        m_tx_poll_counter = m_idle_timeout_ms / KIM2_IDLE_TICK_MS;
    } else {
        m_tx_poll_counter = 0; // No idle timeout — power off immediately if no packet
    }
}

/// @brief Idle: check for pending TX, or power off after timeout.
void KIM2Device::state_idle()
{
    // Check for pending TX
	if (m_packet_buffer.length()) {
		m_tx_buffer = m_packet_buffer;
		m_packet_buffer.clear();
		KIM2_STATE_CHANGE(idle, transmit);
	}
    else if (m_rx_requested) {
        if (!m_rx_supported) {
            // Refused: report it once and let the service close its window
            // through its KineisEventDeviceError handler rather than waiting
            // for a reception that will never start.
            DEBUG_ERROR("KIM2Device::state_idle: RX unavailable — module firmware "
                        "\"%s\" is older than %04u-%02u, downlink reception refused",
                        m_fw_version.c_str(),
                        static_cast<unsigned>(KIM2::KIM2_RX_MIN_FW_YEAR),
                        static_cast<unsigned>(KIM2::KIM2_RX_MIN_FW_MONTH));
            m_rx_requested = false;
            notify(KineisEventDeviceError({}));
            run_state_machine(KIM2_IDLE_TICK_MS);
        } else {
            KIM2_STATE_CHANGE(idle, receive);
        }
    }
    else if (m_stopping) {
        KIM2_STATE_CHANGE(idle, power_off);
    }
    else if (m_idle_timeout_ms == 0) {
        // No idle timeout configured — power off immediately
        KIM2_STATE_CHANGE(idle, power_off);
    }
    else if (m_tx_poll_counter == 0) {
        DEBUG_TRACE("KIM2Device::state_idle: idle timeout elapsed");
        KIM2_STATE_CHANGE(idle, power_off);
    }
    else {
        m_tx_poll_counter--;
        run_state_machine(KIM2_IDLE_TICK_MS);
    }
}

void KIM2Device::state_idle_exit()
{
    ;
}

// ============================================================================
// State: transmit
// ============================================================================

/// @brief Start TX: send AT+TX, set timeout, start polling for +TX= response.
void KIM2Device::state_transmit_enter()
{
    DEBUG_INFO("KIM2Device::state_transmit_enter: mode=%u", static_cast<unsigned int>(m_tx_mode));

    // Auto-recovery: if the module's RCONF doesn't match the requested TX
    // modulation, rewrite it before AT+TX. Otherwise KIM rejects the TX with
    // +ERROR=5 (KNS_STATUS_BAD_LEN, per v0.8) because the payload length no
    // longer matches the active modulation's allowed sizes. Happens when
    // ensure_modulation() wasn't called by the service or when state_init
    // fell back to a default RCONF for a different mode than the caller wants.
    if (m_tx_mode != m_current_rconf_mode) {
        DEBUG_WARN("KIM2Device::state_transmit_enter: TX mode %u != RCONF mode %u, realigning",
            static_cast<unsigned int>(m_tx_mode), static_cast<unsigned int>(m_current_rconf_mode));

        // Non-adaptive guard: master RCONF dictates the actual modulation
        // (load_rconf_for_mode() returns the master regardless of m_tx_mode in
        // non-adaptive). Realignment is futile — rewriting the same RCONF
        // doesn't change the modulation on the air, but ALSO the existing code
        // below clobbers m_current_rconf_mode = m_tx_mode (line ~927),
        // overwriting the truth that state_init read back. With the wrong
        // modulation tag cached, get_current_modulation() lies on the next
        // scheduling cycle and the service keeps building wrong-size packets.
        // Abort cleanly: service.react(DeviceError) reschedules, and at the
        // next cycle get_current_modulation() returns the value state_init set
        // (LDK, etc.) so the rebuilt packet matches the master modulation.
        bool adaptive = configuration_store->read_param<bool>(ParamID::ARGOS_ADAPTIVE_MODULATION);
        if (!adaptive) {
            DEBUG_ERROR("KIM2Device::state_transmit_enter: non-adaptive — service requested mode %u but module is configured for mode %u — aborting TX (next cycle will rebuild at correct mode via get_current_modulation())",
                static_cast<unsigned int>(m_tx_mode),
                static_cast<unsigned int>(m_current_rconf_mode));
            m_tx_buffer.clear();
            KIM2_STATE_CHANGE(transmit, error);
            return;
        }

        // Regulatory gate: VLDA4 already probed off on this boot — never
        // reprogram it even if a caller slipped through.
        if (m_tx_mode == KineisModulation::VLDA4 && !m_vlda4_allowed) {
            DEBUG_ERROR("KIM2Device::state_transmit_enter: VLDA4 gated off — aborting TX");
            m_tx_buffer.clear();
            KIM2_STATE_CHANGE(transmit, error);
            return;
        }

        std::string rconf = load_rconf_for_mode(m_tx_mode);
        if (rconf.size() != 32) {
            DEBUG_ERROR("KIM2Device::state_transmit_enter: no valid RCONF for mode %u (len=%u) — aborting TX",
                static_cast<unsigned int>(m_tx_mode), static_cast<unsigned int>(rconf.size()));
            m_tx_buffer.clear();
            KIM2_STATE_CHANGE(transmit, error);
            return;
        }
        // Write + read back + enforce VLDA4-at-27dBm. If this trips on VLDA4,
        // m_vlda4_allowed is cleared so future switches are refused upstream.
        KIM2::RConfDecoded realign_decoded;
        if (!write_and_validate_rconf(rconf, m_tx_mode, &realign_decoded)) {
            DEBUG_ERROR("KIM2Device::state_transmit_enter: RCONF rejected — aborting TX");
            m_tx_buffer.clear();
            KIM2_STATE_CHANGE(transmit, error);
            return;
        }

        // The payload in m_tx_buffer was zero-padded for m_tx_mode in send():
        // LDK/VLDA4 to their FIXED lengths, LDA2 to the next 32-bit step (the
        // only variable-size modulation). If the RCONF for m_tx_mode physically
        // encodes a DIFFERENT modulation (mislabeled per-mod RCONF, or a
        // non-adaptive master that decodes to another mod), the radio is now on
        // that other modulation and our frame length no longer matches — the
        // module rejects AT+TX with +ERROR=5 (BAD_LEN), e.g. a 12-byte LDA2
        // frame on an LDK radio (LDK length is fixed at 16 B). Trust the
        // read-back: cache the ACTUAL modulation and abort this TX so the
        // service rebuilds a correctly-framed packet next cycle (it reads the
        // truth via get_current_modulation()), instead of TXing a bad-length
        // frame and latching on repeated +ERROR=5.
        KIM2::ModulationVerdict realign_v = KIM2::verify_modulation(m_tx_mode, realign_decoded.modulation);
        if (realign_v.mismatch) {
            DEBUG_ERROR("KIM2Device::state_transmit_enter: RCONF for mode %u physically encodes %s (mode %d) — aborting TX (check RCONF provisioning), caching actual modulation",
                static_cast<unsigned int>(m_tx_mode), realign_decoded.modulation.c_str(),
                static_cast<int>(realign_v.actual));
            m_current_rconf_mode = realign_v.actual;
            cache_current_modulation();
            m_tx_buffer.clear();
            KIM2_STATE_CHANGE(transmit, error);
            return;
        }

        if (!load_kmac()) {
            DEBUG_ERROR("KIM2Device::state_transmit_enter: KMAC reload failed — aborting TX");
            m_tx_buffer.clear();
            KIM2_STATE_CHANGE(transmit, error);
            return;
        }
        PMU::delay_ms(KIM2_DELAY_AFTER_KMAC_MS);
        m_current_rconf_mode = m_tx_mode;
        cache_current_modulation();
        DEBUG_INFO("KIM2Device::state_transmit_enter: RCONF realigned to mode %u",
            static_cast<unsigned int>(m_tx_mode));
    }

    m_tx_done    = false;
    m_is_error   = false;
    m_cmd_is_ok  = false;   // phase-1 +OK ACK flag (reset like send_AT would)
    DEBUG_INFO("KIM2Device::state_transmit_enter: TX START — RCONF=[%s] mode=%d, payload=%u bytes",
        m_kim2_comm.m_rconf_info.c_str(),
        static_cast<int>(m_tx_mode),
        static_cast<unsigned>(m_tx_buffer.size() / 2));
    DEBUG_INFO("KIM2Device::state_transmit_enter: AT+TX=%s)", m_tx_buffer.c_str());

    // Fire AT+TX NON-BLOCKING, then poll the two AT+TX responses in state_transmit()
    // instead of busy-waiting up to 5 s for the +OK (which froze the scheduler):
    //   phase AWAIT_ACK : +OK   = command accepted by the module (NOT yet emitted),
    //   phase AWAIT_TX  : +TX=  = async emission verdict (success/failure).
    if (!m_kim2_comm.send(AT_TX, m_tx_buffer)) {
        DEBUG_ERROR("KIM2Device::state_transmit_enter: AT+TX send rejected (UART busy) — aborting TX");
        m_tx_buffer.clear();
        KIM2_STATE_CHANGE(transmit, error);
        return;
    }
    m_tx_phase             = TxPhase::AWAIT_ACK;
    m_tx_wkup_lowered        = false;
    m_tx_started_ms          = PMU::get_timestamp_ms();
    m_tx_ack_deadline_ms = PMU::get_timestamp_ms() + KIM2_TX_ACK_TIMEOUT_MS;  // 5 s, preserved

    notify(KineisEventTxStarted({}));

    // BLIND: the module bursts retx_nb copies retx_period_s apart before +TX —
    // extend the completion window to cover the whole burst.
    unsigned int tx_timeout_ms = KIM2_TX_TIMEOUT_MS;
    unsigned int rn = 0, period = 0;
    if (kim2_blind_active(rn, period)) {
        // uint64 intermediate: rn(<=127) * period(<=65535) * 1000 overflows uint32.
        // Cap at 2 h so an extreme config can't park the service indefinitely.
        uint64_t burst_ms = (uint64_t)rn * (uint64_t)period * 1000ULL;
        if (burst_ms > 7200000ULL) burst_ms = 7200000ULL;
        tx_timeout_ms += (uint32_t)burst_ms;
        DEBUG_INFO("KIM2Device::state_transmit_enter: BLIND — TX window %u ms", tx_timeout_ms);
    }
    initiate_timeout(tx_timeout_ms);

    // Poll counter for TX completion (60 s backstop, extended in blind)
    m_tx_poll_counter = tx_timeout_ms / KIM2_DELAY_POLL_MS;
}

/// @brief Poll TX: check for +TX= done, error, or poll timeout.
void KIM2Device::state_transmit()
{
    m_kim2_comm.process_rx();  // Drain ISR buffer for async TX events

    // ---- Phase 1: await the +OK ACK for AT+TX (replaces the old 5 s busy-wait) ----
    // +OK only means the module ACCEPTED the command — not that the message was
    // emitted. The emission verdict arrives later as +TX= (phase 2 below).
    if (m_tx_phase == TxPhase::AWAIT_ACK)
    {
        if (m_is_error)
        {
            // +ERROR= to the AT+TX itself (e.g. +ERROR=5 BAD_LEN) — fail fast.
            DEBUG_ERROR("KIM2Device::state_transmit: AT+TX rejected (+ERROR)");
            m_tx_buffer.clear();
            KIM2_STATE_CHANGE(transmit, error);
            return;
        }
        if (m_cmd_is_ok)
        {
            // Command accepted → now wait for the async emission completion.
            m_tx_phase = TxPhase::AWAIT_TX;
            // From here the module owns the sequence and expects no further AT
            // command until the single +TX= that closes it — the point the
            // integration sequence describes as "the wake-up pin is then pulled
            // low because no additional AT command needs to be sent".
            release_wkup_for_burst();
            // fall through into phase-2 polling this same tick
        }
        else if (PMU::get_timestamp_ms() >= m_tx_ack_deadline_ms)
        {
            DEBUG_WARN("KIM2Device::state_transmit: AT+TX +OK ACK timeout (%u ms)",
                static_cast<unsigned>(KIM2_TX_ACK_TIMEOUT_MS));
            m_tx_buffer.clear();
            KIM2_STATE_CHANGE(transmit, error);
            return;
        }
        else
        {
            run_state_machine(KIM2_DELAY_POLL_MS);  // keep waiting for +OK, scheduler free
            return;
        }
    }

    // ---- Phase 2: await +TX=<status> emission completion (existing logic) ----

    if(m_tx_done)
    {
        m_tx_done = false;
        // The module just spoke to us, so it is demonstrably out of low-power
        // mode: raise the pin without paying the wake-up delay.
        resume_wkup_after_burst(false, "fin de salve (+TX= recu)");
        DEBUG_TRACE("KIM2Device::state_transmit: TX status %d", m_kim2_comm.m_tx_status);
        if (m_tx_buffer.size()) {
            m_tx_buffer.clear();
            if (m_kim2_comm.m_tx_status == 0) {
                DEBUG_INFO("KIM2Device::state_transmit: TX EMITTED OK — Argos message sent (mode=%d, RCONF=[%s])",
                    static_cast<int>(m_tx_mode), m_kim2_comm.m_rconf_info.c_str());
                notify(KineisEventTxComplete({}));
            } else {
                DEBUG_WARN("KIM2Device::state_transmit: TX failed status=%d", m_kim2_comm.m_tx_status);
                notify(KineisEventDeviceError({}));
            }
        }
        KIM2_STATE_CHANGE(transmit, idle);
    }
    else if(m_is_error)
    {
        DEBUG_ERROR("KIM2Device::state_transmit: error during TX");
        m_tx_buffer.clear();
        KIM2_STATE_CHANGE(transmit, error);
    }
    else
    {
        if (--m_tx_poll_counter == 0) {
            DEBUG_ERROR("KIM2Device::state_transmit: TX poll timeout");
            m_tx_buffer.clear();
            KIM2_STATE_CHANGE(transmit, error);
        } else {
            run_state_machine(KIM2_DELAY_POLL_MS);
        }
    }
}

void KIM2Device::state_transmit_exit()
{
    cancel_timeout();
    // Net de securite: whatever ended the burst — +ERROR, an ACK timeout, a poll
    // timeout, or a caller cancelling the TX — the pin must be back up before the
    // next AT command. No-op when the +TX= path already raised it.
    resume_wkup_after_burst(true, "sortie de l'etat transmit");
}

/// @brief Release SAT_EXTWAKEUP for the duration of a module-paced BLIND burst.
///
/// Holding the pin asserted keeps the module's STM32 awake for the whole
/// sequence — measured at the bench, 122,6 s awake to emit three messages
/// spaced 60 s apart. Once AT+TX is acknowledged the module paces the burst
/// itself and expects nothing from us until the closing +TX=, so the pin can be
/// dropped and the module allowed into its low-power mode in between.
///
/// Guarded by the KIM2_BLIND_WKUP_RELEASE build flag: the default build keeps
/// the pin asserted exactly as before.
void KIM2Device::release_wkup_for_burst()
{
#if defined(KIM2_BLIND_WKUP_RELEASE) && (KIM2_BLIND_WKUP_RELEASE == 1)
    unsigned int rn = 0, period = 0;
    if (m_tx_wkup_lowered || !kim2_blind_active(rn, period))
        return;
    // A single transmission: the +TX= follows immediately, there is no interval
    // to sleep through. Releasing the pin would gain nothing and cost a wake-up.
    if (rn <= 1)
        return;

    // Transmissions land at t+0, t+period, ... t+(rn-1)*period. We hand control
    // back to the module for the intermediate intervals and take the pin back
    // before the LAST one, the transmission followed by the closing +TX=.
    // Once AT+TX is acknowledged the module paces its own retransmissions and
    // wakes itself for its own deadlines -- the integration manual is explicit:
    // "will be able to wakeup automatically when some frame scheduling/processing
    // will be needed by KIM2 itself. Its RTC is still running, and any RAM
    // configuration is kept in this low power mode". So we hand it back until the
    // closing +TX=, pacing nothing ourselves.
    GPIOPins::clear(SAT_EXTWAKEUP);
    m_tx_wkup_lowered = true;
    DEBUG_INFO("KIM2Device: SAT_EXTWAKEUP LOW - module paces its own burst (retx_nb=%u period=%us)",
               rn, period);
#endif
}

/// @brief Raise SAT_EXTWAKEUP again at the end of a BLIND burst.
///
/// @param wait_for_wake blocks for KIM2_DELAY_WKUP_RESUME_MS after raising the
///        pin. Set it when an AT command may follow immediately and the module
///        state is unknown (exit on error or on timeout). Not needed when we are
///        only listening -- an early raise before the last retransmission, or a
///        close on a +TX= that already proves the module is awake.
/// @param reason label carried in the trace, to tell the three paths apart.
void KIM2Device::resume_wkup_after_burst(bool wait_for_wake, const char *reason)
{
    if (!m_tx_wkup_lowered)
        return;
    m_tx_wkup_lowered = false;
    GPIOPins::set(SAT_EXTWAKEUP);
    if (wait_for_wake)
        PMU::delay_ms(KIM2_DELAY_WKUP_RESUME_MS);
    DEBUG_INFO("KIM2Device: SAT_EXTWAKEUP HIGH — %s", reason);
}


// ============================================================================
// State: receive
// ============================================================================

/// @brief Start reception in runtime mode: AT+DL=1,<safety window>.
/// @note Runtime mode (AT+DL) is used rather than test mode (AT+RX): the module
///       then reports only decoded downlink messages — no raw frames, no
///       +SATDET/+SATLOST/+N0 diagnostics — which is all this driver dispatches
///       and keeps the UART quiet during a pass.
/// @note The +OK of AT+DL is asynchronous — the stack answers once reception is
///       really started, so unsolicited lines may already be interleaved before
///       it. send_AT() copes: the line parser dispatches on the prefix and only
///       a bare +OK sets m_cmd_is_ok, allcast lines seen meanwhile are queued
///       by react(KIM2CommEventAllcast) and drained by the first tick.
void KIM2Device::state_receive_enter()
{
    char params[16];

    m_rx_window_ended = false;
    m_rx_allcast_queue.clear();
    m_rx_stop_requested = false;

    snprintf(params, sizeof(params), "1,%u", KIM2_RX_SAFETY_WINDOW_S);
    if (!send_AT(AT_DL_START, std::string(params), KIM2_RX_START_TIMEOUT_MS)) {
        DEBUG_ERROR("KIM2Device::state_receive_enter: AT+DL=1 rejected");
        m_rx_requested = false;
        m_rx_active = false;
        KIM2_STATE_CHANGE(receive, error);
        return;
    }

    m_rx_active = true;
    m_rx_start_ms = PMU::get_timestamp_ms();
    DEBUG_INFO("KIM2Device::state_receive_enter: reception started (safety window %us)",
               KIM2_RX_SAFETY_WINDOW_S);
    notify(KineisEventRxStarted({}));
}

/// @brief Receive tick: drain the UART, dispatch allcast messages, honour a
///        stop request or the end of the module's window.
void KIM2Device::state_receive()
{
    m_kim2_comm.process_rx();   // Drain ISR buffer → parse → queue allcast lines

    dispatch_allcast_queue();

    // The module reports the real end of the window (explicit stop or safety
    // window expiry) with an empty +RX= line.
    if (m_rx_window_ended) {
        DEBUG_INFO("KIM2Device::state_receive: RX window ended%s",
                   m_rx_stop_requested ? " (stop confirmed)" : "");
        m_rx_window_ended = false;
        m_rx_stop_requested = false;
        m_rx_requested = false;
        KIM2_STATE_CHANGE(receive, idle);
        return;
    }

    if (m_rx_stop_requested) {
        perform_rx_stop();
        return;
    }

    if (m_is_error) {
        DEBUG_ERROR("KIM2Device::state_receive: error during reception");
        m_is_error = false;
        m_rx_requested = false;
        KIM2_STATE_CHANGE(receive, error);
        return;
    }

    run_state_machine(KIM2_RX_TICK_MS);
}

/**
 * @brief Take the module out of DL mode, right now, in the caller's context.
 *
 * Called both from the receive state and directly from stop_receive(). The
 * direct path matters: the RX service asks for the stop while committing its
 * AOP table, then spends seconds writing flash and recomputing a pass
 * prediction before the state machine gets a chance to run. Deferring the
 * command to the next tick left the module streaming into a UART buffer nobody
 * was draining — measured at 4 s in the field, ending in an RX overflow and a
 * rejected command.
 *
 * Safe to call from outside the state machine: allcast lines arriving while we
 * wait are only queued by react(), never dispatched, so no listener runs from
 * inside this call.
 */
void KIM2Device::perform_rx_stop()
{
    bool stopped = false;

    m_rx_stop_requested = false;
    DEBUG_TRACE("KIM2Device::state_receive: stopping reception");

    // send_AT() drains the UART every millisecond while it waits, so the
    // backlog the module is still streaming keeps being parsed and the
    // allcast messages queued behind the reply are not lost — they are
    // dispatched right after the loop.
    for (uint8_t attempt = 1; attempt <= KIM2_RX_STOP_ATTEMPTS; attempt++) {
        if (send_AT(AT_DL_STOP, std::nullopt, KIM2_RX_STOP_TIMEOUT_MS)) {
            stopped = true;
            break;
        }

        const bool rejected = m_is_error;

        // Neither +OK nor an expired window: the stop did not take effect,
        // whether the module stayed silent (command swallowed) or answered
        // +ERROR (it did not parse what reached it). Drain, check the
        // safety window, let the parser settle, and send it again.
        m_kim2_comm.process_rx();
        dispatch_allcast_queue();
        if (m_rx_window_ended) {
            stopped = true;
            break;
        }

        DEBUG_WARN("KIM2Device::state_receive: AT+DL=0 %s (attempt %u/%u), retrying",
                   rejected ? "rejected" : "unanswered",
                   static_cast<unsigned int>(attempt),
                   static_cast<unsigned int>(KIM2_RX_STOP_ATTEMPTS));

        if (rejected)
            PMU::delay_ms(KIM2_RX_STOP_RETRY_GAP_MS);
    }

    m_kim2_comm.process_rx();
    dispatch_allcast_queue();
    m_rx_window_ended = false;

    if (!stopped) {
        // The module never confirmed it left DL mode: it would reject the
        // next AT+TX. Cut the rail instead — the next transmission powers it
        // back on and reconfigures it from scratch.
        //
        // state_power_off_enter() clears m_packet_buffer, so a transmission
        // queued while we were receiving would be silently dropped here.
        // Carry it across the power cycle and re-arm the module for it.
        const std::string pending_tx = m_packet_buffer;
        const KineisModulation pending_mode = m_tx_mode;

        DEBUG_ERROR("KIM2Device::state_receive: reception could not be stopped, "
                    "powering the module off to clear DL mode");
        m_rx_requested = false;
        KIM2_STATE_CHANGE(receive, power_off);

        if (pending_tx.length()) {
            DEBUG_INFO("KIM2Device::state_receive: re-arming the TX queued during "
                       "reception (%u bytes) across the power cycle",
                       static_cast<unsigned int>(pending_tx.length() / 2));
            m_packet_buffer = pending_tx;
            m_tx_mode = pending_mode;
            start_device();
        }
        return;
    }

    KIM2_STATE_CHANGE(receive, idle);
    return;

}

void KIM2Device::state_receive_exit()
{
    // Late lines can still be sitting in the UART buffer.
    m_kim2_comm.process_rx();
    dispatch_allcast_queue();
    finish_rx_session();
}

// ============================================================================
// RX helpers
// ============================================================================

/// @brief Query AT+FW=? once per power-on and evaluate the RX capability gate.
/// @note  Never fails the init: an unsupported (or unreadable) firmware only
///        disables reception, transmission is unaffected.
void KIM2Device::check_rx_firmware_support()
{
    if (m_fw_checked)
        return;
    m_fw_checked = true;

    m_kim2_comm.m_module_banner.clear();
    if (!send_AT(AT_GET_FW)) {
        DEBUG_WARN("KIM2Device::check_rx_firmware_support: AT+FW=? failed");
        m_fw_version.clear();
        m_rx_supported = false;
        return;
    }

    m_fw_version = m_kim2_comm.m_module_banner;
    m_rx_supported = KIM2::fw_supports_rx(m_fw_version);

    const KIM2::FwBuildDate build = KIM2::parse_fw_build_date(m_fw_version);

    if (m_rx_supported) {
        DEBUG_INFO("KIM2Device: firmware \"%s\" (build %04u-%02u-%02u) — RX available",
                   m_fw_version.c_str(), build.year, build.month, build.day);
    } else if (build.valid) {
        DEBUG_ERROR("KIM2Device: firmware \"%s\" built %04u-%02u-%02u is older than "
                    "%04u-%02u — downlink reception disabled, please update the module",
                    m_fw_version.c_str(), build.year, build.month, build.day,
                    static_cast<unsigned>(KIM2::KIM2_RX_MIN_FW_YEAR),
                    static_cast<unsigned>(KIM2::KIM2_RX_MIN_FW_MONTH));
    } else {
        DEBUG_ERROR("KIM2Device: no build date in firmware string \"%s\" — downlink "
                    "reception disabled", m_fw_version.c_str());
    }
}

/// @brief Convert every queued allcast payload and hand it to the listeners.
void KIM2Device::dispatch_allcast_queue()
{
    while (!m_rx_allcast_queue.empty()) {
        const std::string hex = m_rx_allcast_queue.front();
        m_rx_allcast_queue.pop_front();

        unsigned int size_bits = 0;
        const std::string packet = KIM2::allcast_to_bytes(hex, size_bits);
        if (packet.empty()) {
            DEBUG_WARN("KIM2Device: malformed allcast payload: %s", hex.c_str());
            continue;
        }

        notify(KineisEventRxPacket({ packet, size_bits }));
    }
}

void KIM2Device::finish_rx_session()
{
    if (!m_rx_active)
        return;

    m_rx_active = false;
    m_rx_stop_requested = false;

    const uint64_t now = PMU::get_timestamp_ms();
    const unsigned int rx_time = (now > m_rx_start_ms)
                               ? static_cast<unsigned int>(now - m_rx_start_ms) : 0;

    DEBUG_INFO("KIM2Device: reception stopped after %u ms", rx_time);
    notify(KineisEventRxStopped({ rx_time }));
}

// ============================================================================
// State: error
// ============================================================================

void KIM2Device::state_error_enter()
{
    ;
}

/// @brief Error: notify listener and transition to power_off.
void KIM2Device::state_error()
{
    DEBUG_ERROR("KIM2Device::state_error");
    notify(KineisEventDeviceError({}));
    KIM2_STATE_CHANGE(error, power_off);
}

void KIM2Device::state_error_exit()
{
    ;
}

// ============================================================================
// Internal helpers
// ============================================================================

// mod_from_name() moved to the pure, host-testable KIM2::mod_from_name in
// kim2_modulation.hpp (alongside KIM2::verify_modulation).

std::string KIM2Device::load_rconf_for_mode(KineisModulation mode)
{
    bool adaptive = configuration_store->read_param<bool>(ParamID::ARGOS_ADAPTIVE_MODULATION);
    if (!adaptive) {
        return configuration_store->read_param<std::string>(ParamID::ARGOS_RADIOCONF);
    }
    switch (mode) {
        case KineisModulation::LDK:
            return configuration_store->read_param<std::string>(ParamID::ARGOS_RADIOCONF_LDK);
        case KineisModulation::VLDA4:
            return configuration_store->read_param<std::string>(ParamID::ARGOS_RADIOCONF_VLDA4);
        case KineisModulation::LDA2:
        default:
            return configuration_store->read_param<std::string>(ParamID::ARGOS_RADIOCONF_LDA2);
    }
}

/// @brief Write RCONF to the module, then read it back and enforce the
///        KIM2 VLDA4-at-27dBm regulatory constraint.
/// @note Does NOT reload KMAC — that is the caller's responsibility once the
///       RCONF is accepted. Skipping the KMAC reload on rejection keeps the
///       radio MAC tied to the previously-compliant config.
bool KIM2Device::write_and_validate_rconf(const std::string& rconf_hex,
                                          KineisModulation expected_mode,
                                          KIM2::RConfDecoded* out_decoded)
{
    if (rconf_hex.size() != 32) {
        DEBUG_ERROR("KIM2Device::write_and_validate_rconf: invalid RCONF length %u",
                    static_cast<unsigned>(rconf_hex.size()));
        return false;
    }

    if (!send_AT(AT_SET_RCONF, rconf_hex)) {
        DEBUG_ERROR("KIM2Device::write_and_validate_rconf: AT+RCONF= failed");
        return false;
    }
    PMU::delay_ms(KIM2_DELAY_AFTER_RCONF_MS);

    if (!send_AT(AT_GET_RCONF)) {
        DEBUG_WARN("KIM2Device::write_and_validate_rconf: AT+RCONF=? failed — cannot verify compliance");
        // Be conservative: without a read-back we cannot prove VLDA4 is at 27 dBm.
        if (expected_mode == KineisModulation::VLDA4) {
            m_vlda4_allowed = false;
            return false;
        }
        return true;  // Non-VLDA4 path — accept even without verification
    }

    DEBUG_INFO("KIM2Device::write_and_validate_rconf: module RCONF=%s (expected mode=%d)",
               m_kim2_comm.m_rconf_info.c_str(), static_cast<int>(expected_mode));

    KIM2::RConfDecoded decoded = KIM2::parse_rconf_info(m_kim2_comm.m_rconf_info);
    if (out_decoded) *out_decoded = decoded;

    if (!decoded.valid) {
        DEBUG_WARN("KIM2Device::write_and_validate_rconf: could not decode +RCONF= payload");
        if (expected_mode == KineisModulation::VLDA4) {
            m_vlda4_allowed = false;
            return false;
        }
        return true;
    }

    // KIM2 rule: VLDA4 TX is only authorized at 27 dBm. If the module reports
    // VLDA4 at any other power level, disable VLDA4 for the rest of this
    // session. Upstream (ArgosTxService via ensure_modulation()) then sticks
    // to LDK/LDA2 even if adaptive modulation is on.
    if (decoded.modulation == KIM2_MOD_NAME_VLDA4 &&
        decoded.rf_level_dbm != KIM2_VLDA4_REQUIRED_DBM) {
        DEBUG_ERROR("KIM2Device::write_and_validate_rconf: VLDA4 rejected — rf_level=%d dBm (required %d dBm)",
                    decoded.rf_level_dbm, KIM2_VLDA4_REQUIRED_DBM);
        m_vlda4_allowed = false;
        return false;
    }

    return true;
}
