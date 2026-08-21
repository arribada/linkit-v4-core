/**
 * @file lora_rak3172.cpp
 * @brief LoRa RAK3172-SiP device driver — state machine, TX, join, config.
 */

#include "lora_rak3172.hpp"
#include "lora_rak3172_comm.hpp"
#include "bsp.hpp"
#include "gpio.hpp"
#include "pmu.hpp"
#include "debug.hpp"
#include "error.hpp"
#include "binascii.hpp"
#include "config_store.hpp"
#include "nrf_gpio.h"
#include <cstdint>
#include <string>

// Park unused satellite-slot pins in high-impedance on the LoRa board variant:
//   - SAT_INT (P0.29): BSP default has INPUT_CONNECT (input buffer ON). On the
//     LoRa PCB this pin is not wired to anything the RAK3172 drives, so the
//     enabled buffer floats and leaks through the Schmitt trigger. Switch to
//     INPUT_DISCONNECT.
//   - SAT_EXTWAKEUP (P0.30): not connected on LoRa. BSP leaves it as an output;
//     driving an unconnected pin is 0 A but toggling during state changes is
//     wasted bus activity. Park as INPUT_DISCONNECT.
//   - SAT_RESET (P0.31): BSP already sets INPUT_DISCONNECT for probe flashing,
//     which coincidentally is also the lowest-leakage config — leave as is.
//   - SAT_PWR_EN (P1.15): THE one pin we keep actively driving (module power).
//   - KIM_PWR_ON (P0.5): legacy KIM2 power-enable pin, never used in LoRa code.
//     BSP leaves it as OUTPUT (default LOW after reset). If the LoRa PCB
//     physically routes this pin to a RAK3172-SiP GPIO that has an internal
//     pull-up enabled by RUI3 firmware, ~80 µA leaks: RAK VDD → pull-up →
//     pin → nRF P0.5 (LOW) → GND. Parking as INPUT_DISCONNECT breaks the
//     current path. Hypothesis to validate: standby ~130 µA → ~50 µA after
//     this fix.
static inline void lora_park_unused_sat_pins()
{
    nrf_gpio_cfg(NRF_GPIO_PIN_MAP(0, 29),   // SAT_INT
                 NRF_GPIO_PIN_DIR_INPUT,
                 NRF_GPIO_PIN_INPUT_DISCONNECT,
                 NRF_GPIO_PIN_NOPULL,
                 NRF_GPIO_PIN_S0S1,
                 NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_cfg(NRF_GPIO_PIN_MAP(0, 30),   // SAT_EXTWAKEUP
                 NRF_GPIO_PIN_DIR_INPUT,
                 NRF_GPIO_PIN_INPUT_DISCONNECT,
                 NRF_GPIO_PIN_NOPULL,
                 NRF_GPIO_PIN_S0S1,
                 NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_cfg(NRF_GPIO_PIN_MAP(0, 5),    // KIM_PWR_ON (unused on LoRa)
                 NRF_GPIO_PIN_DIR_INPUT,
                 NRF_GPIO_PIN_INPUT_DISCONNECT,
                 NRF_GPIO_PIN_NOPULL,
                 NRF_GPIO_PIN_S0S1,
                 NRF_GPIO_PIN_NOSENSE);
}

using namespace LoRa;

#define LORA_STATE_CHANGE(x, y)                                 \
    do {                                                        \
        DEBUG_TRACE("LoRa::STATE_CHANGE: " #x " -> " #y);      \
        m_state = State::y;                                     \
        state_ ## x ##_exit();                                  \
        state_ ## y ##_enter();                                 \
        run_state_machine();                                    \
    } while (0)

#define LORA_STATE_EQUAL(x) \
    (m_state == State::x)

#define LORA_STATE_CALL(x) \
    do {                   \
        state_ ## x();     \
    } while (0)

extern Scheduler *system_scheduler;
extern ConfigurationStore *configuration_store;

// ============================================================================
// TODO / WARNING — Power consumption when LoRa service is NOT used at runtime
// ============================================================================
// The constructor below immediately kicks the state machine from power_off →
// power_on. This means the RAK3172 module is BOOTED, CONFIGURED, and (in OTAA)
// JOINED **unconditionally** as soon as the LoRaDevice is instantiated, even
// when the LoRaTxService is disabled at runtime (e.g. ARGOS_MODE = OFF or any
// configuration where service_is_enabled() returns false and service_initiate()
// is never called).
//
// Observed symptom (measured 13/05/2026, LinkIt V4 LoRa, PPK) :
//   - LoRa service ENABLED  (Argos Doppler / surface burst running) :
//       inter-cycle idle ~25 µA  (RAK3172 cleanly in Stop2 standby ~1.7 µA,
//       UARTE1 deinit'd, all paths optimised)
//   - LoRa service DISABLED (ARGOS_MODE = OFF, no service ever runs) :
//       inter-cycle idle ~150 µA (RAK3172 sits in `idle` after configure with
//       UARTE1 still init'd; nobody ever transitions it to standby because
//       state_idle → standby fires only after a TX cycle. ~120 µA wasted.)
//
// Why it works in production : every field deployment has at least one TX mode
// active, so the service eventually fires a TX → state_transmit → state_idle
// → state_standby chain, and the module settles in Stop2 (1.7 µA). The wasted
// current only shows up in bench tests with the LoRa stack disabled but the
// module compiled in.
//
// Fix idea (not done — low priority, doesn't impact field deployments) :
//   - On boot, defer the power_on transition until either:
//       a) service_initiate() is first called (lazy init), or
//       b) write_credentials_from_config() / read_lora_credentials() is invoked
//          (covers the SATVF / DTE diagnostic paths)
//   - Alternatively, post a one-shot "if no TX scheduled within 30 s, transit
//     to power_off" task at boot.
//
// Until that refactor, accept that bench tests with LoRa disabled show ~120 µA
// extra idle conso. This is a TEST-ONLY artefact, not a field issue.
// ============================================================================
LoRaDevice::LoRaDevice()
{
    m_packet_buffer.clear();
    m_join_attempted = false;
    m_join_wait_ticks = 0;
    m_state = State::power_off;
    m_joined = false;
    m_join_failed = false;
    m_tx_done = false;
    m_config_step = 0;
    m_is_configured = false;
    m_power_on_retry = 0;
    m_nwm_reboot_pending = false;
    m_consecutive_errors = 0;
    m_config_reload_pending = false;
    load_config_from_store();
    LORA_STATE_CHANGE(power_off, power_on);
}

LoRaDevice::~LoRaDevice()
{
    stop_bridge();
    power_off_immediate();
}

bool LoRaDevice::start_bridge(LoRaComm::PassthroughCallback rx_callback)
{
    if (m_bridge_active)
        return true;

    // Cancel any pending state machine task
    cancel_timeout();
    system_scheduler->cancel_task(m_task);

    // Ensure UART is initialized
    m_lora_comm.init();

    // Enable passthrough: raw UART RX goes to callback
    m_lora_comm.set_passthrough(true, rx_callback);
    m_bridge_active = true;

    DEBUG_INFO("LoRaDevice: bridge mode ACTIVE");
    return true;
}

void LoRaDevice::stop_bridge()
{
    if (!m_bridge_active)
        return;

    m_lora_comm.set_passthrough(false);
    m_bridge_active = false;

    DEBUG_INFO("LoRaDevice: bridge mode STOPPED");

    // Restart state machine from current state
    run_state_machine();
}

bool LoRaDevice::bridge_send(const uint8_t* data, size_t len)
{
    if (!m_bridge_active)
        return false;
    return m_lora_comm.send_raw_data(data, len);
}

void LoRaDevice::bridge_process_rx()
{
    if (m_bridge_active)
        m_lora_comm.process_rx();
}

/**
 * Send a packet via LoRaWAN.
 *
 * The modulation parameter is ignored (LoRa uses its own DR settings).
 * The packet data is hex-encoded and sent on the configured FPort.
 */
void LoRaDevice::send(const KineisModulation mode, const KineisPacket& user_payload, const unsigned int payload_length)
{
    (void)mode;  // LoRa handles modulation via DR setting

    // Reject if already transmitting to avoid overwriting in-flight packet
    if (m_state == State::transmit) {
        DEBUG_WARN("LoRaDevice::send: already transmitting, ignoring");
        return;
    }

    // Defense in depth: reject oversized payloads before handing to the module.
    // Service layer should already clamp to LoRaPayloadLimits::max_payload_for_dr,
    // but a stale config cache or programming error must not be able to wedge the
    // module with an AT_PARAM_ERROR that falls through the error retry path.
    unsigned int payload_bytes = (payload_length + 7) / 8;
    static constexpr unsigned int dr_limits[] = { 51, 51, 51, 115, 222, 222 };
    unsigned int max_bytes = (m_config.dr < 6) ? dr_limits[m_config.dr] : dr_limits[0];
    if (payload_bytes > max_bytes) {
        DEBUG_ERROR("LoRaDevice::send: payload %u bytes > DR%u limit %u — rejecting",
                    payload_bytes, m_config.dr, max_bytes);
        notify(KineisEventDeviceError({}));
        return;
    }

    KineisPacket packet(user_payload.begin(), user_payload.begin() + payload_bytes);
    m_packet_buffer = Binascii::hexlify(packet);
    // New packet, new join budget: state_idle may spend at most one join window
    // on it (see the guard there).
    m_join_attempted = false;

    DEBUG_TRACE("LoRaDevice::send: payload[%u bits]=%s", payload_length, m_packet_buffer.c_str());

    // Request power on (if not already running)
    start_device();
}

void LoRaDevice::stop_send() {
    DEBUG_TRACE("LoRaDevice::stop_send");
    m_packet_buffer.clear();
}

// ========================================================================
// Event handlers from LoRaComm
// ========================================================================

void LoRaDevice::react(const LoRaCommEventRespOk&) {
    m_cmd_is_ok = true;
}

void LoRaDevice::react(const LoRaCommEventRespError& err) {
    DEBUG_INFO("LoRaDevice: error response type=%d", static_cast<int>(err.error_type));
    // AT_NO_NETWORK_JOINED is the module telling us the OTAA session is gone —
    // server-side re-registration, session expiry, or a module-side reset that
    // dropped it. Nothing else in this driver ever clears m_joined (it is only
    // set false on power-on, on entering joining, and on a credential change),
    // so without this the flag stays stale-true for as long as the module stays
    // powered, and the re-join guard in state_idle can never observe the very
    // condition it exists for.
    if (err.error_type == LoRa::RESP_NO_NETWORK)
        m_joined = false;
    m_is_error = true;
}

void LoRaDevice::react(const LoRaCommEventJoined&) {
    DEBUG_INFO("LoRaDevice: JOINED");
    m_joined = true;
}

void LoRaDevice::react(const LoRaCommEventJoinFailed&) {
    DEBUG_INFO("LoRaDevice: JOIN FAILED");
    m_join_failed = true;
}

void LoRaDevice::react(const LoRaCommEventTxDone&) {
    DEBUG_TRACE("LoRaDevice: TX_DONE");
    m_tx_done = true;
}

void LoRaDevice::react(const LoRaCommEventRxData& rx) {
    DEBUG_INFO("LoRaDevice: RX port=%d data=%s", rx.port, rx.payload.c_str());
    // Downlink data received - could be forwarded to a service if needed
}

void LoRaDevice::react(const LoRaCommEventUartError& err) {
    DEBUG_INFO("LoRaDevice: UART error type=%02x", err.error_type);
    m_is_error = true;
}

// ========================================================================
// Timeout management
// ========================================================================

void LoRaDevice::initiate_timeout(unsigned int timeout_ms) {
    cancel_timeout();
    m_timeout.handle = system_scheduler->post_task_prio([this]() {
        on_timeout();
    }, "LoRaTimeout", Scheduler::DEFAULT_PRIORITY, timeout_ms);
}

void LoRaDevice::on_timeout() {
    DEBUG_ERROR("LoRaDevice::on_timeout");
    m_is_error = true;
}

void LoRaDevice::cancel_timeout() {
    system_scheduler->cancel_task(m_timeout.handle);
}

// ========================================================================
// State machine infrastructure
// ========================================================================

void LoRaDevice::start_device()
{
    if (m_state == State::standby) {
        // state_standby_enter deinit'd the nRF UARTE1 for power savings. We
        // MUST reinit it before any AT command — sending on a deinit'd UART
        // silently fails (nrf_libuarte_async_tx no-ops) and leaves
        // m_is_send_busy stuck at true, producing "already busy" on every
        // subsequent call. init() is idempotent (guarded by m_is_init).
        m_lora_comm.init();

        // Quick wake from LPM Stop2 via AT ping (~10ms).
        // RAK3172 wakes on first UART byte, responds to AT within a few ms.
        // Retry up to 3 times before power cycling (module may be slow to wake).
        bool wake_ok = false;
        for (int attempt = 0; attempt < 3 && !wake_ok; attempt++) {
            if (send_AT(AT_TEST))
                wake_ok = true;
        }
        if (wake_ok) {
            DEBUG_TRACE("LoRa: wake from standby OK");
            // Fast path: skip state_idle when a packet is queued and no
            // deferred config reload is pending. Saves one FSM tick
            // (~100 ms, run_state_machine default delay) on the surface
            // first-TX critical path. Mirrors the Argos optimization
            // "skip state_idle on warm-packet path" (§5 priority 3,
            // CLAUDE.md). state_transmit_enter() is robust to entry from
            // any state — it only resets m_tx_done/m_is_error and fires
            // AT+SEND synchronously. The config_reload_pending guard
            // preserves the slow path when a PARMW landed between dive
            // warm-up and surface TX (would otherwise TX with stale config).
            if (m_packet_buffer.length() && !m_config_reload_pending) {
                DEBUG_TRACE("LoRa: standby wake fast-path → transmit (packet queued)");
                LORA_STATE_CHANGE(standby, transmit);
            } else {
                LORA_STATE_CHANGE(standby, idle);
            }
        } else {
            // Module not responding after 3 attempts — full power cycle
            DEBUG_WARN("LoRa: standby wake failed after 3 attempts, power cycling");
            std::string saved_packet = m_packet_buffer;
            power_off_immediate();  // This clears m_packet_buffer
            m_packet_buffer = saved_packet;
            LORA_STATE_CHANGE(power_off, power_on);
        }
        return;
    }

    if (m_state != State::power_off) {
        DEBUG_TRACE("LoRa::start: already running state=%u", static_cast<unsigned>(m_state));
        run_state_machine(0);
        return;
    }
    LORA_STATE_CHANGE(power_off, power_on);
}

void LoRaDevice::power_off_immediate(void)
{
    DEBUG_TRACE("LoRaDevice::power_off_immediate");
    if (!LORA_STATE_EQUAL(power_off)) {
        system_scheduler->cancel_task(m_task);
        cancel_timeout();
        m_state = State::power_off;
        state_power_off_enter();
    }
}

/// @brief Warm up the module for a fast next-TX dispatch. The FSM runs
/// power_on → configure → join → idle → standby asynchronously. State
/// machine is left in standby with empty packet buffer so the next send()
/// goes through the fast wake path (state_standby → start_device reinit
/// UART + AT ping ~10 ms).
void LoRaDevice::warm_up_for_tx()
{
    if (m_state != State::power_off) {
        // Already running — nothing to do. start_device would just log and
        // re-tick the FSM.
        return;
    }
    DEBUG_INFO("LoRaDevice::warm_up_for_tx: pre-booting module for next surface TX");
    LORA_STATE_CHANGE(power_off, power_on);
}

/// @brief Blocking send of an AT command with polling for OK response.
/// @param cmd        AT command type.
/// @param params     Optional parameter string.
/// @param timeout_ms Maximum wait time in ms.
/// @return true on success (OK received), false on error/timeout.
bool LoRaDevice::send_AT(ATCmd cmd, const std::optional<std::string>& params, uint16_t timeout_ms)
{
    m_cmd_is_ok = false;
    m_is_error = false;

    m_lora_comm.send(cmd, params);

    while (!m_cmd_is_ok && !m_is_error && timeout_ms != 0) {
        PMU::delay_ms(1);
        m_lora_comm.process_rx();
        timeout_ms--;
    }

    if (timeout_ms == 0) {
        // AT_SLEEP_NOW runs on a deliberately short, non-fatal timeout: the
        // RAK3172 may already be in Stop2 or transition into it before
        // emitting OK, and state_standby_enter proceeds with UART deinit
        // either way (wake-on-RX brings it back). Demote to TRACE so we don't
        // spam WARN on every standby cycle; keep WARN for any other command
        // where a missing OK is a real protocol failure.
        if (cmd == AT_SLEEP_NOW)
            DEBUG_TRACE("LoRaDevice::send_AT: AT_SLEEP_NOW no-ack (expected — module entering Stop2)");
        else
            DEBUG_WARN("LoRaDevice::send_AT: timeout (cmd=%d)", static_cast<int>(cmd));
    }

    return m_cmd_is_ok && !m_is_error;
}

void LoRaDevice::state_machine(void)
{
    switch (m_state) {
    case State::power_off:   LORA_STATE_CALL(power_off);  break;
    case State::power_on:    LORA_STATE_CALL(power_on);   break;
    case State::configure:   LORA_STATE_CALL(configure);  break;
    case State::joining:     LORA_STATE_CALL(joining);    break;
    case State::idle:        LORA_STATE_CALL(idle);       break;
    case State::standby:     LORA_STATE_CALL(standby);    break;
    case State::transmit:    LORA_STATE_CALL(transmit);   break;
    case State::error:       LORA_STATE_CALL(error);      break;
    default: break;
    }
}

void LoRaDevice::run_state_machine(uint16_t delay_ms)
{
    system_scheduler->cancel_task(m_task);
    m_task = system_scheduler->post_task_prio([this]() {
        state_machine();
    }, "LoRaStateMachine", Scheduler::DEFAULT_PRIORITY, delay_ms);
}

// ========================================================================
// State: power_off
// ========================================================================

void LoRaDevice::state_power_off_enter()
{
    DEBUG_INFO("LoRaDevice::state_power_off_enter");
    m_lora_comm.deinit();
    GPIOPins::clear(SAT_PWR_EN);
    // Park all non-essential nRF pins that touch (or might touch) the LoRa
    // slot into high-impedance input-disconnect so the board truly draws 0 A
    // through the satellite interface when SAT_PWR_EN is low.
    lora_park_unused_sat_pins();
    m_packet_buffer.clear();
}

void LoRaDevice::state_power_off() {
    ;
}

void LoRaDevice::state_power_off_exit() {
    ;
}

// ========================================================================
// State: power_on
//   - Enable power to module
//   - Init UART at 115200 baud
//   - Wait for module boot
//   - Verify communication with AT ping
// ========================================================================

void LoRaDevice::state_power_on_enter()
{
    GPIOPins::set(SAT_PWR_EN);
    // Park unused SAT_* slot pins in input-disconnect. Runs on every power-on
    // so first-boot (before any power_off_enter has fired) still gets the
    // low-leakage pin config instead of the BSP default (input buffer ON).
    lora_park_unused_sat_pins();
    m_lora_comm.init();  // RAK3172 RUI3 factory default 115200 8N1
    m_lora_comm.subscribe(*this);
    m_joined = false;
    m_join_failed = false;
    m_power_on_retry = 0;
    // Non-blocking: schedule first state_power_on() after module boot delay
}

void LoRaDevice::state_power_on()
{
    // Boot-wait + ping budget (tuned for fast first-TX in ABP):
    //   - Initial wait 1000 ms (was 2000) — RAK3172 RUI3 boots in ~1.2-1.5 s
    //     typical, so the first ping may fail on cold/slow boots; the retry
    //     loop below covers that.
    //   - AT_TEST per-attempt timeout 400 ms (was 2000) — AT_TEST is a 4-byte
    //     command; a healthy RAK answers in <50 ms. Tight timeout makes the
    //     retry mechanism react fast when the module isn't ready yet.
    //   - 8 retries at 300 ms gap (was 4 at 500 ms) — total ping window is
    //     1000 + 7*(400+300) ≈ 5.9 s, similar to the old worst case (~11 s
    //     thanks to old 2 s timeouts) but converges in ~1.1 s in the nominal
    //     case (saves ~1 s on every cold boot vs the old 2.1 s minimum).
    if (m_power_on_retry == 0) {
        // First call: wait for RAK3172 boot (typically ~1.2 s)
        m_power_on_retry = 1;
        run_state_machine(1000);
        return;
    }

    // Drain boot banner / noise before the first AT ping. The RAK3172 prints a
    // welcome banner at power-on which would otherwise trigger a spurious
    // framing error and confuse the next send_AT's polling loop.
    if (m_power_on_retry == 1)
        m_lora_comm.process_rx();

    DEBUG_TRACE("LoRaDevice::state_power_on retry=%u", m_power_on_retry);

    // Try AT ping with tight per-attempt timeout (default 2000 ms is way too
    // generous for AT_TEST — it just blocks the FSM when the module is silent).
    if (send_AT(AT_TEST, std::nullopt, 400))
    {
        DEBUG_TRACE("LoRaDevice: AT ping OK (attempt %u)", m_power_on_retry);
        LORA_STATE_CHANGE(power_on, configure);
        return;
    }

    if (m_power_on_retry < 8) {
        m_power_on_retry++;
        run_state_machine(300);  // Tighter retry gap
        return;
    }

    DEBUG_ERROR("LoRaDevice: AT ping failed after %u attempts — check wiring/module", m_power_on_retry);
    LORA_STATE_CHANGE(power_on, error);
}

void LoRaDevice::state_power_on_exit() {
    ;
}

// ========================================================================
// State: configure
//   - First boot: full configuration (all LoRaWAN parameters via AT commands)
//   - Subsequent boots: fast path — RAK3172 persists config to flash,
//     so we only read DEVEUI and check join status
// ========================================================================

void LoRaDevice::state_configure_enter()
{
    m_nwm_reboot_pending = false;
    if (m_is_configured) {
        // Fast path: RAK3172 persists config to flash, skip to DEVEUI read + join check
        m_config_step = 100;
        DEBUG_INFO("LoRaDevice: fast config (module already configured)");
    } else {
        m_config_step = 0;
        DEBUG_INFO("LoRaDevice: full config (first boot)");
    }
}

void LoRaDevice::state_configure()
{
    DEBUG_TRACE("LoRaDevice::state_configure step=%u", m_config_step);
    bool at_error = false;

    switch (m_config_step)
    {
        // ==== Full configuration (first boot only) ====

        case 0:
        {
            // Ensure LoRaWAN network mode (AT+NWM=1)
            if (m_nwm_reboot_pending) {
                m_nwm_reboot_pending = false;
                at_error = !send_AT(AT_TEST);
                if (at_error) {
                    DEBUG_ERROR("LoRaDevice: module not responding after NWM reboot");
                }
                break;
            }
            at_error = !send_AT(AT_GET_NWM);
            if (!at_error && m_lora_comm.m_last_value == "1") {
                DEBUG_TRACE("LoRaDevice: already in LoRaWAN mode");
                break;
            }
            DEBUG_INFO("LoRaDevice: setting NWM=1 (module will reboot)");
            if (!m_lora_comm.send(AT_SET_NWM, "1")) {
                DEBUG_ERROR("LoRaDevice: failed to issue AT+NWM=1 (UART busy/deinit)");
                at_error = true;
                break;
            }
            m_nwm_reboot_pending = true;
            run_state_machine(3000);  // Wait for module autoreboot after NWM change
            return;
        }

        case 1:
            // Set join mode (OTAA or ABP)
            at_error = !send_AT(AT_SET_NJM, std::to_string(m_config.njm));
            break;

        case 2:
            // Set frequency band
            at_error = !send_AT(AT_SET_BAND, std::to_string(m_config.band));
            break;

        case 3:
            // Set Device EUI. Only relevant for OTAA — ABP activation uses
            // DEVADDR + NWKSKEY + APPSKEY (written at step 5). RAK3172 rejects
            // AT+DEVEUI writes in certain ABP states with PARAM_ERROR type=2.
            if (m_config.njm == 0) {
                DEBUG_TRACE("LoRaDevice: ABP mode — skipping DEVEUI set");
                break;
            }
            if (m_config.deveui.empty()) {
                at_error = !send_AT(AT_GET_DEVEUI);
                if (!at_error && !m_lora_comm.m_last_value.empty() &&
                    m_lora_comm.m_last_value != "0000000000000000") {
                    m_config.deveui = m_lora_comm.m_last_value;
                    DEBUG_INFO("LoRaDevice: module DEVEUI=%s", m_config.deveui.c_str());
                } else {
                    // Generate from nRF52840 FICR unique device ID
                    char buf[17];
#ifdef NRF52840_XXAA
                    snprintf(buf, sizeof(buf), "%08lX%08lX",
                             (unsigned long)NRF_FICR->DEVICEID[1],
                             (unsigned long)NRF_FICR->DEVICEID[0]);
#else
                    snprintf(buf, sizeof(buf), "%08lX%08lX",
                             (unsigned long)0, (unsigned long)PMU::device_identifier());
#endif
                    m_config.deveui = std::string(buf);
                    DEBUG_INFO("LoRaDevice: generated DEVEUI=%s", m_config.deveui.c_str());
                    at_error = !send_AT(AT_SET_DEVEUI, m_config.deveui);
                }
                // Persist to config store so user can read via PARMR (LRP01)
                configuration_store->write_param(ParamID::LORA_DEVEUI, m_config.deveui);
            } else {
                at_error = !send_AT(AT_SET_DEVEUI, m_config.deveui);
            }
            break;

        case 4:
            // Set Application EUI (OTAA only)
            if (m_config.njm == 1) {
                if (m_config.appeui.empty()) {
                    // Was a silent skip: the module then joined with whatever
                    // APPEUI sat in its own flash. Say so, loudly.
                    DEBUG_ERROR("LoRaDevice: LORA_APPEUI (LRP02) is UNSET — module keeps its stored JoinEUI");
                    break;
                }
                if (m_config.appeui.size() != 16) {
                    DEBUG_ERROR("LoRaDevice: invalid APPEUI length %u (expected 16 hex chars)", static_cast<unsigned>(m_config.appeui.size()));
                    at_error = true;
                    break;
                }
                at_error = !send_AT(AT_SET_APPEUI, m_config.appeui);
                if (!at_error)
                    DEBUG_INFO("LoRaDevice: APPEUI written to module (%s)", m_config.appeui.c_str());
            }
            break;

        case 5:
            // Set Application Key (OTAA) or session keys (ABP)
            if (m_config.njm == 1) {
                if (m_config.appkey.empty()) {
                    // Same silent-skip trap as APPEUI, and the more damaging of
                    // the two: a wrong AppKey fails the server-side MIC check,
                    // so the JoinRequest is discarded without any reply at all.
                    DEBUG_ERROR("LoRaDevice: LORA_APPKEY (LRP03) is UNSET — module keeps its stored AppKey; "
                                "the server will drop the JoinRequest on MIC");
                    break;
                }
                if (m_config.appkey.size() != 32) {
                    DEBUG_ERROR("LoRaDevice: invalid APPKEY length %u (expected 32 hex chars)", static_cast<unsigned>(m_config.appkey.size()));
                    at_error = true;
                    break;
                }
                at_error = !send_AT(AT_SET_APPKEY, m_config.appkey);
                if (!at_error)
                    DEBUG_INFO("LoRaDevice: APPKEY written to module (32 hex chars, not logged)");
            } else if (m_config.njm == 0) {
                if (!m_config.devaddr.empty()) {
                    if (m_config.devaddr.size() != 8) {
                        DEBUG_ERROR("LoRaDevice: invalid DEVADDR length %u (expected 8 hex chars)", static_cast<unsigned>(m_config.devaddr.size()));
                        at_error = true;
                        break;
                    }
                    at_error = !send_AT(AT_SET_DEVADDR, m_config.devaddr);
                }
                if (!at_error && !m_config.nwkskey.empty()) {
                    if (m_config.nwkskey.size() != 32) {
                        DEBUG_ERROR("LoRaDevice: invalid NWKSKEY length %u (expected 32 hex chars)", static_cast<unsigned>(m_config.nwkskey.size()));
                        at_error = true;
                        break;
                    }
                    at_error = !send_AT(AT_SET_NWKSKEY, m_config.nwkskey);
                }
                if (!at_error && !m_config.appskey.empty()) {
                    if (m_config.appskey.size() != 32) {
                        DEBUG_ERROR("LoRaDevice: invalid APPSKEY length %u (expected 32 hex chars)", static_cast<unsigned>(m_config.appskey.size()));
                        at_error = true;
                        break;
                    }
                    at_error = !send_AT(AT_SET_APPSKEY, m_config.appskey);
                }
            }
            break;

        case 6:
            // Set device class (AT+CLASS=A/B/C)
            at_error = !send_AT(AT_SET_CLASS, std::string(1, static_cast<char>(m_config.device_class)));
            break;

        case 7:
            // Set data rate
            at_error = !send_AT(AT_SET_DR, std::to_string(m_config.dr));
            break;

        case 8:
            // Set ADR
            at_error = !send_AT(AT_SET_ADR, std::to_string(m_config.adr));
            break;

        case 9:
            // Set TX power
            at_error = !send_AT(AT_SET_TXP, std::to_string(m_config.txp));
            break;

        case 10:
            // Set confirmed/unconfirmed mode
            at_error = !send_AT(AT_SET_CFM, std::to_string(m_config.cfm));
            break;

        case 11:
            // Set retry count (only relevant if CFM=1)
            if (m_config.cfm) {
                at_error = !send_AT(AT_SET_RETY, std::to_string(LoRa::DEFAULT_RETY));
            }
            break;

        case 12:
            // Apply user-configured low-power mode (LORA_LP_MODE / LRP15):
            //   0 = shutdown (0 µA idle, ~2.5 s reboot on next TX)
            //   1 = standby Stop2 (~1.7 µA idle, ~10 ms wake)
            at_error = !send_AT(AT_SET_LPM, std::to_string(m_config.lp_mode));
            break;

        case 13:
            // Select the deepest sleep level compatible with our hardware.
            //
            // The LinkIt V4 LoRa PCB routes the AT-command UART to the RAK3172
            // *UART1* pins (STM32 USART1 = PA9/PA10) — see bsp.cpp:360-361.
            //
            // Per the RUI3 AT manual: "Stop2 Mode ... will not allow you to
            // wake up using UART1. On Stop1 Mode, both UART1 and UART2 can
            // wake up the device." So LPMLVL=2 is incompatible with this PCB
            // — the module enters Stop2 but cannot be woken via our UART,
            // and RUI3's fallback (RTC-wake duty cycle) results in ~130 µA
            // observed instead of the datasheet 1.7 µA.
            //
            // LPMLVL=1 (Stop1) supports UART1 wake and gives ~3 µA per
            // STM32WLE5 datasheet — acceptable for our use case (turtle tag
            // burst gaps of 30+ s).
            //
            // Tolerated as non-fatal: older RUI3 firmware versions (pre 4.0.x)
            // don't support AT+LPMLVL at all and return ERROR. In that case
            // AT+LPM=1 (set in step 12) is the best we can do — the module
            // uses its default sleep level. Don't fail the whole configure
            // walk over a missing optional optimisation.
            if (!send_AT(AT_SET_LPMLVL, std::to_string(1))) {
                DEBUG_WARN("LoRaDevice: AT+LPMLVL not supported by this RUI3 version — continuing with LPM=1 default");
            }

            // Belt-and-suspenders verification: query LPM mode + level back from
            // the module so we know what sleep state it will actually enter at
            // idle. Pure diagnostic — failures don't fail the configure walk,
            // they just leave "?" in the log. The only ground truth for sleep
            // current is still an ammeter on SAT_PWR_EN (~3 µA at Stop1, ~400 µA
            // if auto-sleep is off), but a matching LPM=1 + LPMLVL=1 readback is
            // strong indirect evidence that the RAK accepted and applied the
            // commands. Runs once per cold boot, not per standby cycle.
            {
                std::string lpm_val = "?";
                std::string lpmlvl_val = "?";
                if (send_AT(AT_GET_LPM))
                    lpm_val = m_lora_comm.m_last_value;
                if (send_AT(AT_GET_LPMLVL))
                    lpmlvl_val = m_lora_comm.m_last_value;
                DEBUG_INFO("LoRaDevice: low-power readback: LPM=%s LPMLVL=%s (configured LPM=%u; LPMLVL=1 → Stop1 ~3uA wake-on-UART1)",
                           lpm_val.c_str(), lpmlvl_val.c_str(),
                           static_cast<unsigned>(m_config.lp_mode));
            }
            break;

        case 14:
            // Duty Cycle Setting — compile-time flag LORA_DCS_ENABLE.
            //   1 (default) : RAK3172 enforces EU868 1%/0.1%/10% limits (ETSI legal)
            //   0           : DCS off — test builds only, **illegal in EU deployment**
            // We always set this explicitly so the module does not keep a stale
            // value written by an earlier firmware flash.
            at_error = !send_AT(AT_SET_DCS, std::to_string(LORA_DCS_ENABLE));
            break;

        case 15:
            // Full config done — mark configured. ABP short-circuit: no join,
            // no NJS read needed, so transition straight to idle and skip the
            // two no-op scheduler ticks the common path (case 100 → 101) would
            // cost. Saves ~40 ms on the cold-boot → first-TX critical path
            // for ABP. OTAA still takes the common path to verify join state
            // via AT_GET_NJS / launch joining if needed.
            m_is_configured = true;
            if (m_config.njm == 0) {
                DEBUG_INFO("LoRaDevice: ABP — configure done, going to idle");
                m_joined = true;
                LORA_STATE_CHANGE(configure, idle);
                return;
            }
            m_config_step = 99;  // Will be incremented to 100 (OTAA path)
            break;

        // ==== Common path (runs every boot) ====

        case 100:
            // Config store (LRP01) is the source of truth for DEVEUI — never
            // overwrite it from the module. Bootstrap (empty store) is handled
            // in case 3 for OTAA. ABP keeps whatever the user wrote for later
            // analysis even though it has no LoRaWAN function.
            break;

        case 101:
            // Check join status / initiate join
            if (m_config.njm == 0) {
                DEBUG_INFO("LoRaDevice: ABP mode - no join required");
                m_joined = true;
                LORA_STATE_CHANGE(configure, idle);
                return;
            }
            at_error = !send_AT(AT_GET_NJS);
            if (!at_error && m_lora_comm.m_last_value == "1") {
                DEBUG_INFO("LoRaDevice: already joined from previous session");
                m_joined = true;
                LORA_STATE_CHANGE(configure, idle);
                return;
            }
            LORA_STATE_CHANGE(configure, joining);
            return;

        default:
            LORA_STATE_CHANGE(configure, error);
            return;
    }

    if (at_error)
    {
        DEBUG_ERROR("LoRaDevice::state_configure: failed at step %u (fatal — no internal retry)", m_config_step);
        // Configure failures (bad credentials, module rejecting AT command) do
        // NOT self-heal on retry. Skip the state_error retry loop, notify the
        // service layer immediately, and cut power. The service's global error
        // counter (DEVICE_ERROR_MAX_CONSECUTIVE) is the single source of truth
        // for when to give up.
        m_consecutive_errors = 0;
        notify(KineisEventDeviceError({}));
        LORA_STATE_CHANGE(configure, power_off);
        return;
    }

    m_config_step++;
    // Inter-step delay: 20 ms (was 50 ms). Each step's send_AT() blocks ~50-100 ms
    // already, which is plenty for the scheduler to tick other tasks. The 50 ms
    // gap was a safety margin that costs ~390 ms over the 13 configure steps
    // for nothing measurable. Keep some delay (not 0) so background tasks
    // (BLE, sensors, USB) get airtime between AT round-trips.
    run_state_machine(20);  // Continue configuration
}

void LoRaDevice::state_configure_exit() {
    ;
}

// ========================================================================
// State: joining
//   - Initiate LoRaWAN join procedure
//   - Wait for +EVT:JOINED or +EVT:JOIN FAILED
//   - Timeout after 90 seconds
// ========================================================================

bool LoRaDevice::otaa_credentials_missing() const
{
    // ABP activates from DEVADDR + NWKSKEY + APPSKEY and never uses these two,
    // so it can never be "missing credentials" by this definition.
    return m_config.njm == 1 && (m_config.appeui.empty() || m_config.appkey.empty());
}

bool LoRaDevice::query_join_state()
{
    // AT+NJS=? is a local UART query: no RF, no duty-cycle cost, safe to poll.
    if (!send_AT(AT_GET_NJS))
        return false;
    return m_lora_comm.m_last_value == "1";
}

void LoRaDevice::state_joining_enter()
{
    DEBUG_INFO("LoRaDevice::state_joining_enter: NJM=%u", m_config.njm);

    // ABP mode should never reach here, but handle gracefully
    if (m_config.njm == 0) {
        DEBUG_INFO("LoRaDevice: ABP mode - skipping join");
        m_joined = true;
        return;
    }

    // Refuse to burn a join window with credentials the server cannot possibly
    // accept. With LORA_NJM defaulting to 1 (OTAA) and LORA_APPEUI/LORA_APPKEY
    // both defaulting to "", an unprovisioned unit used to reach here, skip the
    // AT+APPEUI/AT+APPKEY writes silently in configure, and join with whatever
    // keys were left in the module's own flash. The server drops those on MIC
    // and the only symptom is a join that never completes.
    if (otaa_credentials_missing()) {
        DEBUG_ERROR("LoRaDevice: OTAA join refused — LORA_APPEUI(LRP02)=%s LORA_APPKEY(LRP03)=%s; "
                    "provision them (PARMW) then force a write (SATVF force=1)",
                    m_config.appeui.empty() ? "UNSET" : "set",
                    m_config.appkey.empty() ? "UNSET" : "set");
        m_is_error = true;
        return;
    }

    m_joined = false;
    m_join_failed = false;
    m_is_error = false;
    m_join_wait_ticks = 0;
    // Consume this packet's single join allowance, whoever initiated the join
    // (configure case 101 on a cold dispatch, or the state_idle guard).
    m_join_attempted = true;

    // AT+JOIN=<start>:<auto_join>:<interval>:<attempts>
    // start=1, auto_join=0 (no persistent rejoin on failure — saves battery).
    // Attempt count and interval are unchanged from the previous behaviour, so
    // the RF footprint is identical; only the supervising timeout is corrected.
    const std::string join_args = "1:0:" + std::to_string(JOIN_INTERVAL_S) +
                                  ":" + std::to_string(JOIN_ATTEMPTS);
    if (!send_AT(AT_JOIN, join_args)) {
        // A module still running its previous join cycle answers AT_ERROR /
        // AT_BUSY_ERROR here. That is not a device fault and must not cost an
        // error strike: ask it directly whether it is already joined, and
        // otherwise just keep supervising the cycle it is still running.
        if (query_join_state()) {
            DEBUG_INFO("LoRaDevice: AT+JOIN refused but module reports joined (NJS=1)");
            m_joined = true;
            return;
        }
        DEBUG_WARN("LoRaDevice: AT+JOIN refused (module busy in a previous cycle) — supervising it");
        initiate_timeout(JOIN_WINDOW_MS);
        return;
    }

    DEBUG_INFO("LoRaDevice: join started (%u attempts, %u s apart, window %u s)",
               JOIN_ATTEMPTS, JOIN_INTERVAL_S, JOIN_WINDOW_MS / 1000);
    initiate_timeout(JOIN_WINDOW_MS);
}

void LoRaDevice::state_joining()
{
    m_lora_comm.process_rx();  // Process any ISR-buffered async events

    if (m_joined)
    {
        DEBUG_INFO("LoRaDevice: network joined successfully");
        LORA_STATE_CHANGE(joining, idle);
    }
    else if (m_join_failed || m_is_error)
    {
        // Before declaring failure, ask the module directly. +EVT:JOINED is a
        // single asynchronous line: if the cooperative scheduler was starved
        // while the GNSS receiver streamed (observed: 3.5 min with no LoRa
        // tick), process_rx() was not pumping and the event can be dropped on
        // ISR-buffer overflow — after which the join has actually SUCCEEDED and
        // the firmware would never know. AT+NJS=? is the module's own state and
        // costs no RF, so it is the safer source of truth on this edge.
        if (query_join_state()) {
            DEBUG_WARN("LoRaDevice: join event missed (timeout/error) but module reports joined (NJS=1) — recovering");
            m_joined = true;
            LORA_STATE_CHANGE(joining, idle);
            return;
        }
        DEBUG_ERROR("LoRaDevice: join failed");
        LORA_STATE_CHANGE(joining, error);
    }
    else
    {
        // Still waiting. Poll NJS periodically for the same reason as above:
        // it turns a missed event into at most JOIN_NJS_POLL_TICKS * 500 ms of
        // extra waiting instead of a whole wasted window.
        if (++m_join_wait_ticks >= JOIN_NJS_POLL_TICKS) {
            m_join_wait_ticks = 0;
            if (query_join_state()) {
                DEBUG_INFO("LoRaDevice: joined (detected by NJS poll)");
                m_joined = true;
                LORA_STATE_CHANGE(joining, idle);
                return;
            }
        }
        run_state_machine(500);
    }
}

void LoRaDevice::state_joining_exit()
{
    cancel_timeout();
}

// ========================================================================
// State: idle
//   - Check for pending TX packets
//   - Power off if nothing to send
// ========================================================================

void LoRaDevice::state_idle_enter() {
    ;
}

void LoRaDevice::state_idle()
{
    // Deferred config reload triggered by a PARMW while the FSM was busy.
    // Drop pending packet (if any) since it may have been encoded with stale
    // credentials/keys and re-route through power_off so the next start_device
    // cold-boots through the full configure walk with the new values. The
    // service layer's depth-pile keeps the data; this only sacrifices one TX
    // dispatch to converge on the fresh config.
    if (m_config_reload_pending) {
        DEBUG_INFO("LoRaDevice::state_idle: applying deferred config reload");
        m_config_reload_pending = false;
        if (m_packet_buffer.length()) {
            DEBUG_WARN("LoRaDevice::state_idle: dropping pending packet to apply new config");
            m_packet_buffer.clear();
            notify(KineisEventDeviceError({}));
        }
        LORA_STATE_CHANGE(idle, power_off);
        return;
    }

    if (m_packet_buffer.length()) {
        // OTAA session guard, for the case the module was already up and lost
        // its session: send() is dispatched by the service layer with no
        // knowledge of join state, so the packet would go out as AT+SEND on an
        // unjoined module, fail with AT_NO_NETWORK_JOINED, and burn an error
        // strike for nothing. Re-joining here keeps the packet buffered —
        // state_joining returns to idle on success and this branch transmits it.
        //
        // m_join_attempted caps this at ONE join window per buffered packet,
        // and that bound is load-bearing. state_error sends the FSM back to
        // idle for its first MAX_CONSECUTIVE_ERRORS strikes, and nothing on
        // that path clears m_packet_buffer, so an unbounded guard would re-fire
        // on every strike and turn each one into a fresh 90 s join. Out of
        // gateway coverage — the normal state of a terrestrial tracker between
        // passes — that trades a ~50 ms local rejection for ~270 s of join
        // requests at LORA_TXP=0, every dispatch, for a gateway that isn't
        // there. Battery is priority 2 in this firmware; the guard must not
        // make the no-coverage case expensive to buy back the rare
        // lost-session case.
        //
        // The flag is set by state_joining_enter (so a join already spent by
        // the configure walk counts too) and cleared by send() for each new
        // packet.
        //
        // ABP is unaffected twice over: njm == 0, and the configure walk has
        // already set m_joined (case 15 / case 101).
        if (m_config.njm == 1 && !m_joined && !m_join_attempted) {
            DEBUG_INFO("LoRaDevice::state_idle: TX pending but not joined — re-joining first");
            LORA_STATE_CHANGE(idle, joining);
            return;
        }
        LORA_STATE_CHANGE(idle, transmit);
    } else if (m_config.lp_mode == 1) {
        // Standby: module stays powered. Configured LPMLVL=1 → Stop1
        // (~3 µA per STM32WLE5 datasheet), NOT Stop2 (~1.7 µA): Stop2 cannot
        // wake from UART1 on this PCB — see state_configure step 13 rationale.
        LORA_STATE_CHANGE(idle, standby);
    } else {
        // Shutdown: full power off = 0µA, but 2.5s reboot on next TX
        LORA_STATE_CHANGE(idle, power_off);
    }
}

void LoRaDevice::state_idle_exit() {
    ;
}

// ========================================================================
// State: standby
//   - Module powered, UART initialized, Stop1 LPM active (~3 µA per
//     STM32WLE5 datasheet — Stop2 ~1.7 µA is incompatible with our UART1
//     wake path, so configure pins LPMLVL=1).
//   - RAK3172 auto-sleeps via LPM=1 when no UART activity
//   - Wake on UART RX (any byte wakes the MCU from Stop1 in ~15µs)
//   - Waiting for send() to be called
// ========================================================================

void LoRaDevice::state_standby_enter() {
    // Deinitialize the nRF UARTE1 peripheral while the RAK3172 sleeps in
    // its own Stop1 LPM. Without this, UARTE1 stays clocked and burns
    // ~300-500 µA even when idle — that's the difference between the
    // LoRa board measuring 0.7 mA vs SMD measuring 27 µA.
    //
    // The RAK3172 itself continues to receive power via SAT_PWR_EN so it
    // retains LoRaWAN session state. We only detach the nRF-side UART;
    // the module's internal Stop1 wake-on-UART is still active on its
    // side, but we no longer clock our peripheral while waiting for a
    // send() call from the service layer.
    // Demoted to TRACE: fires on every TX cycle (idle → standby) on the
    // surfacing-burst hot path. LFS commit overhead (~50-300 ms) compounds
    // across burst pings. The transition is observable via the standby_exit
    // INFO log on next TX wake (if needed).
    DEBUG_TRACE("LoRaDevice: standby (RAK Stop1 ~3uA + UARTE1 deinit)");

    // Explicit AT+SLEEP before UART deinit — workaround for AT+LPM=1 not
    // reliably triggering Stop2 auto-sleep on RUI 4.0.6. The continuous-sleep
    // form (no param) wakes on any UART RX byte, which is exactly what
    // start_device() sends to resume from standby (AT ping).
    // Short timeout (200 ms) to avoid blocking the scheduler if the module is
    // already asleep / unresponsive — we're transitioning to standby anyway,
    // a missed OK isn't fatal.
    send_AT(AT_SLEEP_NOW, std::nullopt, 200);

    m_lora_comm.deinit();
    // Re-assert the unused-pin park — defensive in case some other code path
    // reconfigured them between power_on and here.
    lora_park_unused_sat_pins();
}

void LoRaDevice::state_standby() {
    // Nothing to do - module is sleeping, waiting for send() call
}

void LoRaDevice::state_standby_exit() {
    // Re-initialize UARTE1 so we can talk to the RAK3172 again. This is
    // the mirror of state_standby_enter's deinit. The module is still
    // powered (SAT_PWR_EN high), so the first UART byte we send (typically
    // an AT ping from start_device's wake path, or the AT+SEND from
    // state_transmit_enter) will wake it from Stop2 in ~15 µs.
    DEBUG_TRACE("LoRaDevice: standby exit — reinit UART");
    m_lora_comm.init();
}

// ========================================================================
// State: transmit
//   - Send hex payload via AT+SEND=<fport>:<hex_data>
//   - Wait for +EVT:TX_DONE
// ========================================================================

void LoRaDevice::state_transmit_enter()
{
    // Demoted to TRACE: fires on every TX. AT+SEND itself is the canonical
    // marker (visible in module UART traces) and the service-layer log
    // ("process_status_burst" or equivalent) already names the TX.
    DEBUG_TRACE("LoRaDevice::state_transmit_enter");

    m_tx_done = false;
    m_is_error = false;

    // Format: AT+SEND=<port>:<hex_payload>
    std::string send_params = std::to_string(m_config.fport) + ":" + m_packet_buffer;
    if (!send_AT(AT_SEND, send_params)) {
        DEBUG_ERROR("LoRaDevice: AT+SEND command failed");
        m_is_error = true;
        return;
    }

    notify(KineisEventTxStarted({}));

    // TX timeout based on data rate — low DRs have very long air times
    // EU868 worst case: DR0 (SF12/125kHz) ~2.5s per byte, 51 bytes max = ~130s
    // Add margin for RX windows (1s + 2s) and processing
    static constexpr unsigned int tx_timeout_ms[] = {
        150000,  // DR0 (SF12) — ~130s air time + RX windows
        80000,   // DR1 (SF11) — ~65s air time
        45000,   // DR2 (SF10) — ~35s air time
        25000,   // DR3 (SF9)  — ~18s air time
        15000,   // DR4 (SF8)  — ~10s air time
        10000,   // DR5 (SF7)  — ~5s air time
    };
    unsigned int dr = m_config.dr;
    unsigned int timeout = (dr < 6) ? tx_timeout_ms[dr] : 10000;
    initiate_timeout(timeout);
}

void LoRaDevice::state_transmit()
{
    m_lora_comm.process_rx();  // Process any ISR-buffered async events

    if (m_tx_done)
    {
        m_tx_done = false;
        m_consecutive_errors = 0;  // Reset error counter on successful TX
        DEBUG_TRACE("LoRaDevice::state_transmit: TX complete");
        m_packet_buffer.clear();
        notify(KineisEventTxComplete({}));
        LORA_STATE_CHANGE(transmit, idle);
    }
    else if (m_is_error)
    {
        DEBUG_ERROR("LoRaDevice::state_transmit: TX error");
        LORA_STATE_CHANGE(transmit, error);
    }
    else
    {
        run_state_machine(100);
    }
}

void LoRaDevice::state_transmit_exit()
{
    cancel_timeout();
}

// ========================================================================
// State: error
// ========================================================================

void LoRaDevice::state_error_enter() {
    ;
}

void LoRaDevice::state_error()
{
    m_consecutive_errors++;
    if (m_consecutive_errors <= MAX_CONSECUTIVE_ERRORS) {
        // Transient error — retry from idle (module may still be responsive)
        DEBUG_WARN("LoRaDevice::state_error: retry %u/%u", m_consecutive_errors, MAX_CONSECUTIVE_ERRORS);
        m_is_error = false;
        LORA_STATE_CHANGE(error, idle);
    } else {
        // Persistent error — power cycle
        DEBUG_ERROR("LoRaDevice::state_error: max retries reached, powering off");
        m_consecutive_errors = 0;
        notify(KineisEventDeviceError({}));
        LORA_STATE_CHANGE(error, power_off);
    }
}

void LoRaDevice::state_error_exit() {
    ;
}

// ========================================================================
// Config store integration
// ========================================================================

/// @brief Load LoRaWAN configuration from ConfigStore.
void LoRaDevice::load_config_from_store()
{
    m_config.deveui  = configuration_store->read_param<std::string>(ParamID::LORA_DEVEUI);
    m_config.appeui  = configuration_store->read_param<std::string>(ParamID::LORA_APPEUI);
    m_config.appkey  = configuration_store->read_param<std::string>(ParamID::LORA_APPKEY);
    m_config.devaddr = configuration_store->read_param<std::string>(ParamID::LORA_DEVADDR);
    m_config.appskey = configuration_store->read_param<std::string>(ParamID::LORA_APPSKEY);
    m_config.nwkskey = configuration_store->read_param<std::string>(ParamID::LORA_NWKSKEY);
    m_config.njm     = static_cast<uint8_t>(configuration_store->read_param<unsigned int>(ParamID::LORA_NJM));
    m_config.band    = static_cast<uint8_t>(configuration_store->read_param<unsigned int>(ParamID::LORA_BAND));
    m_config.dr      = static_cast<uint8_t>(configuration_store->read_param<unsigned int>(ParamID::LORA_DR));
    m_config.adr     = configuration_store->read_param<bool>(ParamID::LORA_ADR) ? 1 : 0;
    m_config.txp     = static_cast<uint8_t>(configuration_store->read_param<unsigned int>(ParamID::LORA_TXP));
    m_config.cfm     = configuration_store->read_param<bool>(ParamID::LORA_CFM) ? 1 : 0;
    m_config.fport   = static_cast<uint8_t>(configuration_store->read_param<unsigned int>(ParamID::LORA_FPORT));
    m_config.lp_mode = static_cast<uint8_t>(configuration_store->read_param<unsigned int>(ParamID::LORA_LP_MODE));

    // Class: stored as 0/1/2 → convert to 'A'/'B'/'C'
    unsigned int lora_class = configuration_store->read_param<unsigned int>(ParamID::LORA_CLASS);
    m_config.device_class = 'A' + static_cast<uint8_t>(lora_class);

    // Compute the largest packet the current config will ever produce, and bump
    // LORA_DR to the minimum rate that guarantees it fits. This replaces the
    // previous "silent skip" behavior: every packet the service emits is now
    // transmittable. Only applies to packet types actually enabled by config.
    //
    // EU868 max MAC payload: DR0-2 = 51 B, DR3 = 115 B, DR4-5 = 222 B.
    unsigned int max_packet_bytes = 2;  // Status packet floor (14 bits)

    // CloudLocate MEAS50 is only emitted when FASTLOC_MODE=CLOUDLOCATE + fmt=MEAS50
    unsigned int fastloc_mode = configuration_store->read_param<unsigned int>(ParamID::GNSS_FASTLOC_MODE);
    unsigned int cl_format    = configuration_store->read_param<unsigned int>(ParamID::GNSS_CLOUDLOCATE_FORMAT);
    if (fastloc_mode == (unsigned int)BaseFastlocMode::CLOUDLOCATE &&
        cl_format    == (unsigned int)BaseCloudLocateFormat::MEAS50) {
        // type(3)+fmt(2)+flags(4)+volt(7)+50B = 16+400 = 416 bits = 52 B
        if (52 > max_packet_bytes) max_packet_bytes = 52;
    }

    // Sensor packet: header + mask + always-present GPS_FULL + enabled sensors
    unsigned int sensor_bits = 14 + 6 + 86;  // header + mask + GPS_FULL
    if (fastloc_mode >= (unsigned int)BaseFastlocMode::DEGRADED_PVT)
        sensor_bits += 60;                   // BITS_FASTLOC_QUALITY
#if ENABLE_ALS_SENSOR
    if (configuration_store->read_param<bool>(ParamID::ALS_SENSOR_ENABLE))
        sensor_bits += 17;
#endif
#if ENABLE_PH_SENSOR
    if (configuration_store->read_param<bool>(ParamID::PH_SENSOR_ENABLE))
        sensor_bits += 14;
#endif
#if ENABLE_PRESSURE_SENSOR
    if (configuration_store->read_param<bool>(ParamID::PRESSURE_SENSOR_ENABLE))
        sensor_bits += 29;                   // pressure + press_temp
#endif
    // SEA_TEMP and THERMISTOR share the same 14-bit slot (mutually exclusive at build)
#if ENABLE_SEA_TEMP_SENSOR
    if (configuration_store->read_param<bool>(ParamID::SEA_TEMP_SENSOR_ENABLE))
        sensor_bits += 14;
#endif
#if ENABLE_THERMISTOR_SENSOR
    if (configuration_store->read_param<bool>(ParamID::THERMISTOR_SENSOR_ENABLE))
        sensor_bits += 14;
#endif
#if ENABLE_AXL_SENSOR
    if (configuration_store->read_param<bool>(ParamID::AXL_SENSOR_ENABLE))
        sensor_bits += 67;                   // AXL temp + 3 axis + act
#endif
    unsigned int sensor_bytes = (sensor_bits + 7) / 8;
    if (sensor_bytes > max_packet_bytes) max_packet_bytes = sensor_bytes;

    // GPS multi is always size-clamped by max_gps_entries(), so it never
    // exceeds the current DR — no contribution to max_packet_bytes.

    // Map byte threshold → min required DR
    uint8_t min_dr = 0;
    if      (max_packet_bytes > 115) min_dr = 4;  // DR4 (222 B)
    else if (max_packet_bytes > 51)  min_dr = 3;  // DR3 (115 B)
    else                             min_dr = 0;  // any DR

    if (m_config.dr < min_dr) {
        DEBUG_WARN("LoRaDevice: max packet %u B requires DR≥%u — bumping LORA_DR %u → %u",
                   max_packet_bytes, min_dr, m_config.dr, min_dr);
        m_config.dr = min_dr;
    }

    DEBUG_INFO("LoRaDevice: config NJM=%u BAND=%u DR=%u ADR=%u TXP=%u CFM=%u FPORT=%u CLASS=%c LP=%u",
               m_config.njm, m_config.band, m_config.dr, m_config.adr,
               m_config.txp, m_config.cfm, m_config.fport, m_config.device_class, m_config.lp_mode);
    if (!m_config.deveui.empty())
        DEBUG_INFO("LoRaDevice: DEVEUI=%s", m_config.deveui.c_str());
    if (!m_config.appeui.empty())
        DEBUG_INFO("LoRaDevice: APPEUI=%s", m_config.appeui.c_str());
}

/// @brief Re-read LoRa config from store and detect drift vs cached `m_config`.
/// On drift: invalidate the "already configured" fast path, raise the deferred
/// reload flag, and (if the FSM is currently parked) trigger a power_off so the
/// next start_device walks the full configure chain with fresh values.
///
/// Busy states (transmit/joining/configure/power_on/error) are left alone — the
/// flag is consumed at the next idle entry. This guarantees we never abort an
/// in-flight TX or Join just because a PARMW was issued.
bool LoRaDevice::reload_config_if_changed()
{
    LoRaConfig old_cfg = m_config;
    load_config_from_store();

    bool changed =
        (old_cfg.deveui       != m_config.deveui)       ||
        (old_cfg.appeui       != m_config.appeui)       ||
        (old_cfg.appkey       != m_config.appkey)       ||
        (old_cfg.devaddr      != m_config.devaddr)      ||
        (old_cfg.appskey      != m_config.appskey)      ||
        (old_cfg.nwkskey      != m_config.nwkskey)      ||
        (old_cfg.njm          != m_config.njm)          ||
        (old_cfg.band         != m_config.band)         ||
        (old_cfg.dr           != m_config.dr)           ||
        (old_cfg.adr          != m_config.adr)          ||
        (old_cfg.txp          != m_config.txp)          ||
        (old_cfg.cfm          != m_config.cfm)          ||
        (old_cfg.fport        != m_config.fport)        ||
        (old_cfg.lp_mode      != m_config.lp_mode)      ||
        (old_cfg.device_class != m_config.device_class);

    if (!changed)
        return false;

    DEBUG_INFO("LoRaDevice::reload_config_if_changed: LoRa config drift detected, scheduling reconfigure");
    m_is_configured        = false;  // Force full configure walk on next entry
    m_config_reload_pending = true;

    // Bridge mode (LORABR=1) gives the host direct UART access to the RAK3172.
    // Calling power_off_immediate() would deinit UARTE1 and kill the bridge
    // mid-session — definitely not what the user wants while typing AT
    // commands in a terminal. Defer; stop_bridge() → run_state_machine() will
    // hit state_idle which consumes the flag cleanly.
    if (m_bridge_active) {
        DEBUG_INFO("LoRaDevice::reload_config_if_changed: bridge active, deferring apply");
        return true;
    }

    // Apply immediately if FSM is parked. For busy states (transmit/joining/
    // configure/power_on/error), state_idle will catch the flag on the way back.
    if (m_state == State::idle || m_state == State::standby || m_state == State::power_off) {
        DEBUG_TRACE("LoRaDevice::reload_config_if_changed: applying now (state=%u)", static_cast<unsigned>(m_state));
        power_off_immediate();
        m_config_reload_pending = false;  // Next start_device() cold-boots → full configure
    }
    return true;
}

/// @brief Read LoRa credentials from RAK3172 module via AT commands.
/// Module must be powered on (idle/standby/configure state).
LoRaDevice::LoRaCredentials LoRaDevice::read_lora_credentials()
{
    LoRaCredentials creds;
    creds.read_ok = false;

    // Auto-wake: if the module is off (LORA_LP_MODE=0 shutdown, or
    // LORA_POWER_OFF_UNDERWATER during a dive), boot it up just for this read.
    // Restore to power_off on exit so the caller's idle state is preserved.
    bool was_off = (m_state == State::power_off);
    if (!ensure_module_awake()) {
        DEBUG_ERROR("LoRaDevice::read_lora_credentials: module wake-up failed");
        return creds;
    }

    // Read NJM — defensive parse: only accept "0" or "1", anything else is
    // likely leftover banner/noise that slipped past the comm filter.
    if (send_AT(AT_GET_NJM)) {
        const std::string& v = m_lora_comm.m_last_value;
        if (v == "0") creds.njm = 0;
        else if (v == "1") creds.njm = 1;
        else {
            DEBUG_WARN("LoRaDevice::read_lora_credentials: unexpected NJM value '%s' — treating as read failure", v.c_str());
            if (was_off) power_off_immediate();
            return creds;
        }
    } else {
        DEBUG_WARN("LoRaDevice::read_lora_credentials: failed to read NJM");
        if (was_off) power_off_immediate();
        return creds;
    }

    // Read DEVEUI
    if (send_AT(AT_GET_DEVEUI)) {
        creds.deveui = m_lora_comm.m_last_value;
    }

    if (creds.njm == 1) {
        // OTAA: read APPEUI + APPKEY
        if (send_AT(AT_GET_APPEUI)) {
            creds.appeui = m_lora_comm.m_last_value;
        }
        if (send_AT(AT_GET_APPKEY)) {
            creds.appkey = m_lora_comm.m_last_value;
        }
    } else {
        // ABP: read DEVADDR (NWKSKEY/APPSKEY are write-only on RAK3172)
        if (send_AT(AT_GET_DEVADDR)) {
            creds.devaddr = m_lora_comm.m_last_value;
        }
    }

    creds.read_ok = true;

    if (was_off) {
        DEBUG_TRACE("LoRaDevice::read_lora_credentials: restoring power-off");
        power_off_immediate();
    }
    return creds;
}

/// @brief Write LoRa credentials from the config store to the RAK3172 module.
/// Module must be powered on (idle/standby/configure state). Caller must reload
/// the module config (e.g. via read_lora_credentials()) afterwards to refresh cache.
/// @note Changing NJM triggers a RAK3172 reboot; subsequent AT writes in the same
///       call will fail. Callers should only change NJM outside this path or
///       re-invoke after the reboot completes.
bool LoRaDevice::write_credentials_from_config()
{
    bool was_off = (m_state == State::power_off);
    if (!ensure_module_awake()) {
        DEBUG_ERROR("LoRaDevice::write_credentials_from_config: wake-up failed");
        return false;
    }

    // Capture the module's CURRENT NJM before we overwrite it. AT+SET_NJM
    // triggers a RAK3172 software reboot when the value actually changes;
    // any AT writes issued in that ~2 s window are silently dropped (no
    // response, banner printed, etc.). Detect-then-pause avoids the cascade
    // of "DEVADDR not set / no network joined" failures previously observed
    // when switching OTAA ↔ ABP from SATVF=1.
    uint8_t module_njm = 0xFF;  // 0xFF = unknown / no readback
    if (send_AT(AT_GET_NJM)) {
        const std::string& v = m_lora_comm.m_last_value;
        if      (v == "0") module_njm = 0;
        else if (v == "1") module_njm = 1;
    }

    // Load latest values from config store into m_config (keeps local cache coherent)
    load_config_from_store();
    DEBUG_INFO("write_credentials: njm=%u deveui='%s' (len=%u) devaddr='%s' (len=%u) "
               "appeui_len=%u appkey_len=%u nwkskey_len=%u appskey_len=%u",
               m_config.njm,
               m_config.deveui.c_str(),  static_cast<unsigned>(m_config.deveui.size()),
               m_config.devaddr.c_str(), static_cast<unsigned>(m_config.devaddr.size()),
               static_cast<unsigned>(m_config.appeui.size()),
               static_cast<unsigned>(m_config.appkey.size()),
               static_cast<unsigned>(m_config.nwkskey.size()),
               static_cast<unsigned>(m_config.appskey.size()));

    bool njm_changing = (module_njm != 0xFF && module_njm != m_config.njm);

    // Set join mode first — module may reboot if NJM changes
    DEBUG_TRACE("write_credentials: step NJM");
    if (!send_AT(AT_SET_NJM, std::to_string(m_config.njm))) {
        DEBUG_ERROR("write_credentials: AT_SET_NJM failed");
        return false;
    }

    // If NJM actually changed, the RAK3172 is rebooting right now. Wait for
    // the boot to complete (~2 s typical, give 3 s margin), drain the boot
    // banner so it does not pollute the next response, and re-probe with
    // AT to confirm the module is responsive again BEFORE writing any
    // credentials. Without this, the credential writes that follow vanish
    // into the UART void and leave the module in a broken half-provisioned
    // state.
    if (njm_changing) {
        DEBUG_INFO("write_credentials: NJM changed (%u → %u), waiting for module reboot",
                   module_njm, m_config.njm);
        PMU::delay_ms(3000);
        m_lora_comm.process_rx();  // drain boot banner
        if (!send_AT(AT_TEST)) {
            DEBUG_ERROR("write_credentials: module not responsive after NJM-triggered reboot");
            return false;
        }
    }

    // DEVEUI: OTAA only. ABP activation uses DEVADDR + session keys; writing
    // DEVEUI in ABP triggers RAK3172 PARAM_ERROR type=2 in some states.
    if (m_config.njm == 1 &&
        !m_config.deveui.empty() && m_config.deveui != "0000000000000000") {
        if (m_config.deveui.size() != 16) {
            DEBUG_ERROR("write_credentials: invalid DEVEUI length %u",
                        static_cast<unsigned>(m_config.deveui.size()));
            return false;
        }
        DEBUG_TRACE("write_credentials: step DEVEUI");
        if (!send_AT(AT_SET_DEVEUI, m_config.deveui)) {
            DEBUG_ERROR("write_credentials: AT_SET_DEVEUI failed");
            return false;
        }
    } else if (m_config.njm == 0) {
        DEBUG_TRACE("write_credentials: ABP mode — skipping DEVEUI");
    }

    if (m_config.njm == 1) {
        // OTAA: APPEUI + APPKEY. This is the SATVF force=1 path, i.e. an
        // operator explicitly asking "push my credentials to the module". It
        // used to skip an unset key silently and still report success, so the
        // operator had no way to tell a real write from a no-op. Report both
        // outcomes at a level that survives DEBUG_LEVEL=3.
        if (m_config.appeui.empty()) {
            DEBUG_ERROR("write_credentials: LORA_APPEUI (LRP02) UNSET — NOT written, module keeps its stored JoinEUI");
        } else {
            if (m_config.appeui.size() != 16) {
                DEBUG_ERROR("write_credentials: invalid APPEUI length %u",
                            static_cast<unsigned>(m_config.appeui.size()));
                return false;
            }
            if (!send_AT(AT_SET_APPEUI, m_config.appeui)) {
                DEBUG_ERROR("write_credentials: AT_SET_APPEUI failed");
                return false;
            }
            DEBUG_INFO("write_credentials: APPEUI written (%s)", m_config.appeui.c_str());
        }
        if (m_config.appkey.empty()) {
            DEBUG_ERROR("write_credentials: LORA_APPKEY (LRP03) UNSET — NOT written, module keeps its stored AppKey");
        } else {
            if (m_config.appkey.size() != 32) {
                DEBUG_ERROR("write_credentials: invalid APPKEY length %u",
                            static_cast<unsigned>(m_config.appkey.size()));
                return false;
            }
            if (!send_AT(AT_SET_APPKEY, m_config.appkey)) {
                DEBUG_ERROR("write_credentials: AT_SET_APPKEY failed");
                return false;
            }
            DEBUG_INFO("write_credentials: APPKEY written (32 hex chars, not logged)");
        }
    } else {
        // ABP: DEVADDR + NWKSKEY + APPSKEY all required for a functional session.
        // (Previously only DEVADDR was written — session keys were skipped with a
        // misleading "write-only" comment. They ARE write-only at the RAK3172 read
        // side but MUST be written for ABP activation.)
        if (!m_config.devaddr.empty()) {
            if (m_config.devaddr.size() != 8) {
                DEBUG_ERROR("write_credentials: invalid DEVADDR length %u",
                            static_cast<unsigned>(m_config.devaddr.size()));
                return false;
            }
            DEBUG_TRACE("write_credentials: step DEVADDR");
            if (!send_AT(AT_SET_DEVADDR, m_config.devaddr)) {
                DEBUG_ERROR("write_credentials: AT_SET_DEVADDR failed");
                return false;
            }
        }
        if (!m_config.nwkskey.empty()) {
            if (m_config.nwkskey.size() != 32) {
                DEBUG_ERROR("write_credentials: invalid NWKSKEY length %u",
                            static_cast<unsigned>(m_config.nwkskey.size()));
                return false;
            }
            DEBUG_TRACE("write_credentials: step NWKSKEY");
            if (!send_AT(AT_SET_NWKSKEY, m_config.nwkskey)) {
                DEBUG_ERROR("write_credentials: AT_SET_NWKSKEY failed");
                return false;
            }
        }
        if (!m_config.appskey.empty()) {
            if (m_config.appskey.size() != 32) {
                DEBUG_ERROR("write_credentials: invalid APPSKEY length %u",
                            static_cast<unsigned>(m_config.appskey.size()));
                return false;
            }
            DEBUG_TRACE("write_credentials: step APPSKEY");
            if (!send_AT(AT_SET_APPSKEY, m_config.appskey)) {
                DEBUG_ERROR("write_credentials: AT_SET_APPSKEY failed");
                return false;
            }
        }
    }

    DEBUG_INFO("write_credentials: credentials written OK (njm=%u)", m_config.njm);

    // Credentials just changed → the cached "module already configured"
    // shortcut is no longer valid, and any prior OTAA session is dead.
    // Force the FSM to walk the full configure chain on its next entry, and
    // re-Join in OTAA. Without this, the driver would AT+SEND with stale
    // session keys and rely on the state_error path to recover (1-2 lost
    // dispatches per credential change).
    m_is_configured = false;
    if (m_config.njm == 1) {
        m_joined = false;
    }

    if (was_off) {
        DEBUG_TRACE("LoRaDevice::write_credentials_from_config: restoring power-off");
        power_off_immediate();
        // power_off_immediate already resets the cycle; next start_device
        // walks power_on → configure(full) → joining (OTAA) cleanly.
    } else {
        // Module is still on (caller had it awake). Mark the deferred reload
        // so the next idle entry forces a power-cycle and clean reconfigure.
        m_config_reload_pending = true;
    }
    return true;
}

/// @brief Start RF Continuous Wave on the RAK3172 (AT+CW=freq,pwr,duration).
/// Auto-wakes if needed — CW is a test command so we do NOT restore power-off
/// afterwards: the CW itself takes over the radio for its `duration_s` window,
/// and the caller typically pairs with cw_stop (ATZ) which brings the driver
/// back to power_off anyway.
bool LoRaDevice::cw_start(uint32_t freq_hz, uint16_t power_dbm, uint16_t duration_s)
{
    if (!ensure_module_awake()) {
        DEBUG_WARN("LoRaDevice::cw_start: module wake-up failed");
        return false;
    }
    if (duration_s == 0) duration_s = 60;   // RUI3 requires a duration
    std::string params = std::to_string(freq_hz) + "," +
                         std::to_string(power_dbm) + "," +
                         std::to_string(duration_s);
    DEBUG_INFO("LoRaDevice::cw_start: %s", params.c_str());
    return send_AT(AT_SET_CW, params);
}

/// @brief Stop CW — RUI3 does not expose AT+CW stop; issue ATZ. The module
/// resets without sending "OK", so we dispatch the write without waiting and
/// force our own state machine back to power_off. The next send() will bring
/// the module up cleanly via the normal configure path.
bool LoRaDevice::cw_stop()
{
    DEBUG_INFO("LoRaDevice::cw_stop (ATZ reset)");
    if (m_state == State::power_off) return true;  // Already stopped

    // If the driver is in standby, UARTE1 is deinit'd (see state_standby_enter).
    // Sending AT_RESET on an unclocked UART would bus-fault. Re-init first.
    if (m_state == State::standby) {
        m_lora_comm.init();
    }

    // Fire-and-forget: RUI3 silently resets on ATZ, so waiting for OK would
    // always time out and falsely report failure.
    m_lora_comm.send(AT_RESET);
    PMU::delay_ms(10);  // Let the bytes drain to UART before we tear down the state machine.

    // Match internal state to the fact that the module is rebooting.
    power_off_immediate();
    return true;
}

/// @brief Synchronous wake-up helper — auto power-on + wait for AT readiness.
/// Does NOT block on configure/join so DTE callers (SATVF, get_firmware_version,
/// COMCW) get a fast turnaround. Returns as soon as AT_TEST succeeds.
bool LoRaDevice::ensure_module_awake(unsigned int timeout_ms)
{
    if (m_state == State::standby) {
        // Module is in RAK3172 Stop2 LPM and — importantly — our nRF UARTE1
        // was deinit'd by state_standby_enter for power savings. Sending on
        // an unclocked UARTE peripheral causes a bus fault (HARDFAULT / MCU
        // reboot). Re-init the UART and send an AT ping to wake the module.
        DEBUG_INFO("LoRaDevice::ensure_module_awake: module standby — reinit UART + wake ping");
        m_lora_comm.init();
        bool wake_ok = false;
        for (int attempt = 0; attempt < 3 && !wake_ok; attempt++) {
            if (send_AT(AT_TEST, std::nullopt, 500))
                wake_ok = true;
        }
        if (wake_ok) {
            m_state = State::idle;
            return true;
        }
        // Wake failed — fall through to power cycle path below.
        DEBUG_WARN("LoRaDevice::ensure_module_awake: standby wake failed — power cycling");
        power_off_immediate();
        // continue into the power_off branch
    }

    if (m_state != State::power_off) {
        return true;  // Already powered (idle, configure, joining, transmit)
    }

    DEBUG_INFO("LoRaDevice::ensure_module_awake: module off — booting");

    // Kick off the state machine (power_off → power_on). The async state machine
    // will run configure + join in the background after we return.
    GPIOPins::set(SAT_PWR_EN);
    GPIOPins::set(SAT_EXTWAKEUP);
    m_lora_comm.init();
    m_lora_comm.subscribe(*this);

    // Wait for the RAK3172 boot banner to settle (~1.5 s typical) and drain it.
    const unsigned int boot_delay_ms = 2000;
    for (unsigned int i = 0; i < boot_delay_ms && timeout_ms > boot_delay_ms; i += 50) {
        PMU::delay_ms(50);
    }
    if (timeout_ms <= boot_delay_ms) {
        DEBUG_WARN("LoRaDevice::ensure_module_awake: timeout < boot delay, skipping wait");
    } else {
        timeout_ms -= boot_delay_ms;
    }
    m_lora_comm.process_rx();  // drain boot banner

    // Probe the module with AT. First ping after cold boot sometimes needs retry.
    while (timeout_ms > 0) {
        if (send_AT(AT_TEST, std::nullopt, 500)) {
            // Transition the driver state to idle so the normal FSM path resumes.
            // configure/join will run asynchronously on the next scheduler tick.
            DEBUG_INFO("LoRaDevice::ensure_module_awake: module responded in < %ums", 3500 - timeout_ms);
            m_state = State::idle;
            run_state_machine(50);
            return true;
        }
        if (timeout_ms < 500) break;
        timeout_ms -= 500;
    }

    DEBUG_ERROR("LoRaDevice::ensure_module_awake: module did not respond to AT within timeout");
    return false;
}

/// @brief Read RAK3172 RUI3 firmware version. Auto-wakes if needed, restores
/// the initial power state (off) on exit so idle power draw is unchanged by
/// DTE queries. Retries once if the first response looks like lingering boot
/// banner text instead of a version string.
std::string LoRaDevice::get_firmware_version()
{
    bool was_off = (m_state == State::power_off);
    if (!ensure_module_awake()) {
        DEBUG_WARN("LoRaDevice::get_firmware_version: module wake-up failed");
        return "";
    }

    // Valid RUI3 version strings start with "RUI_" (e.g. "RUI_4.0.6_RAK3272-SiP").
    // If the first response is anything else (typically leftover boot banner),
    // drain RX and retry once — the comm filter may have missed a late banner.
    std::string version;
    for (int attempt = 0; attempt < 2; attempt++) {
        if (!send_AT(AT_GET_VER)) {
            DEBUG_WARN("LoRaDevice::get_firmware_version: AT+VER=? failed (attempt %d)", attempt + 1);
            break;
        }
        const std::string& v = m_lora_comm.m_last_value;
        if (v.compare(0, 4, "RUI_") == 0 || v.compare(0, 4, "RAK") == 0) {
            version = v;
            break;
        }
        DEBUG_WARN("LoRaDevice::get_firmware_version: unexpected response '%s', draining and retrying", v.c_str());
        PMU::delay_ms(200);
        m_lora_comm.process_rx();
    }

    if (was_off) {
        DEBUG_TRACE("LoRaDevice::get_firmware_version: restoring power-off");
        power_off_immediate();
    }
    return version;
}

// ========================================================================
// KineisDevice interface - unused methods for LoRa
// ========================================================================

void LoRaDevice::start_receive(const KineisModulation mode)
{
    (void)mode;
    DEBUG_WARN("LoRaDevice::start_receive: not implemented (use Class C for continuous RX)");
}

bool LoRaDevice::stop_receive()
{
    DEBUG_WARN("LoRaDevice::stop_receive: not implemented");
    return false;
}

void LoRaDevice::set_frequency(double freq_mhz)
{
    (void)freq_mhz;
    DEBUG_TRACE("LoRaDevice::set_frequency: %.3f MHz (ignored - LoRa uses band/channel config)", freq_mhz);
}

void LoRaDevice::set_tcxo_warmup_time(unsigned int ms)
{
    (void)ms;
    DEBUG_TRACE("LoRaDevice::set_tcxo_warmup_time: %u ms (managed by RAK3172)", ms);
}
