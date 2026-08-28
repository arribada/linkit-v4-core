/**
 * @file nrf_uart_async.cpp
 * @brief Common async UART driver — deferred RX pattern implementation.
 */

#include "nrf_uart_async.hpp"
#include "nrf_peripheral_power.hpp"
#include "bsp.hpp"
#include "error.hpp"
#include "debug.hpp"
#include "interrupt_lock.hpp"
#include "nrf_gpio.h"
#include <cstring>

// ============================================================================
// Static ISR dispatcher — routes to the correct NrfUartAsync instance
// ============================================================================

static void nrf_uart_async_evt_handler(void *context, nrf_libuarte_async_evt_t *p_evt) {
	auto *obj = static_cast<NrfUartAsync *>(context);
	switch (p_evt->type) {
	case NRF_LIBUARTE_ASYNC_EVT_TX_DONE: obj->isr_handle_tx_done(); break;
	case NRF_LIBUARTE_ASYNC_EVT_RX_DATA:
		obj->isr_handle_rx_data(p_evt->data.rxtx.p_data, static_cast<uint16_t>(p_evt->data.rxtx.length));
		break;
	case NRF_LIBUARTE_ASYNC_EVT_ERROR: obj->isr_handle_error(static_cast<unsigned int>(p_evt->data.errorsrc)); break;
	case NRF_LIBUARTE_ASYNC_EVT_ALLOC_ERROR: obj->isr_handle_error(0x100); break;
	case NRF_LIBUARTE_ASYNC_EVT_OVERRUN_ERROR: obj->isr_handle_error(0x200); break;
	default: break;
	}
}

// ============================================================================
// Constructor / Destructor
// ============================================================================

NrfUartAsync::NrfUartAsync(unsigned int uart_instance)
    : m_uart_instance(uart_instance),
      m_is_rx_started(false),
      m_is_send_busy(false),
      m_is_init(false),
      m_uart_config{},
      m_isr_buf_len(0),
      m_isr_error(false),
      m_isr_error_type(0),
      m_isr_overflow(false) {}

NrfUartAsync::~NrfUartAsync() {
	deinit();
}

// ============================================================================
// Init / Deinit
// ============================================================================

void NrfUartAsync::init(uint32_t baudrate_override) {
	if (m_is_init) return;

	// Pre-reserve string buffers to avoid heap reallocation during processing
	m_rx_buffer.reserve(256);
	m_tx_buffer.reserve(128);
	m_line_buffer.reserve(128);

	// Copy BSP config, optionally override baudrate
	m_uart_config = BSP::UARTAsync_Inits[m_uart_instance].config;
	if (baudrate_override != 0) m_uart_config.baudrate = static_cast<nrf_uarte_baudrate_t>(baudrate_override);

	if (nrf_libuarte_async_init(BSP::UARTAsync_Inits[m_uart_instance].uart, &m_uart_config, nrf_uart_async_evt_handler,
	                            static_cast<void *>(this))
	    != NRF_SUCCESS) {
		DEBUG_ERROR("NrfUartAsync: UART%u init failed", m_uart_instance);
		throw ErrorCode::RESOURCE_NOT_AVAILABLE;
	}

	nrf_libuarte_async_start_rx(BSP::UARTAsync_Inits[m_uart_instance].uart);
	m_is_init = true;
	m_is_rx_started = true;
	m_isr_buf_len = 0;
	m_isr_error = false;
	m_isr_overflow = false;
	m_rx_buffer.clear();
}

/// @note Includes Nordic SDK workaround: UARTE HF clock and DMA bus not properly
///       released on uninit — force POWER register toggle to fully shut down.
///       Also pins the TX line HIGH and disconnects RX so the UART peer sees a
///       stable idle signal and can enter its deepest LPM without being woken
///       by spurious transitions on a floating line (measured impact: ~150 µA
///       saved on the RAK3172 Stop2 path).
void NrfUartAsync::deinit() {
	if (!m_is_init) return;

	// Capture pin assignments BEFORE uninit — nrfx_uarte_uninit resets them.
	uint32_t tx_pin = BSP::UARTAsync_Inits[m_uart_instance].config.tx_pin;
	uint32_t rx_pin = BSP::UARTAsync_Inits[m_uart_instance].config.rx_pin;

	nrf_libuarte_async_uninit(BSP::UARTAsync_Inits[m_uart_instance].uart);
	m_is_init = false;

	// Any in-flight TX is aborted with the peripheral. Clear the busy flag so
	// the next init+send() can proceed — TX_DONE will never fire for the
	// aborted transfer. Also reset RX state so the next start_rx is clean.
	m_is_send_busy = false;
	m_is_rx_started = false;

	// Nordic SDK bug: UARTE HF clock and DMA bus not released on uninit.
	// UARTE0=0x40002000, UARTE1=0x40028000
	static constexpr uint32_t uarte_base[] = { 0x40002000, 0x40028000 };
	if (m_uart_instance < 2) nrf_peripheral_power_reset(uarte_base[m_uart_instance]);

	// After uninit, nrfx drops the UART pins to input-disconnect (floating).
	// A floating line into the peer's RX pin can look like start bits and
	// keep the peer from entering deep LPM. Explicitly park TX as an output
	// driven HIGH (UART idle state) so the peer sees a stable, noise-free
	// signal. Leave RX as input-disconnect (high-Z) — the peer drives it
	// when awake, and when sleeping either floats or holds HIGH; nRF input
	// leakage is negligible either way.
	nrf_gpio_cfg(tx_pin, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1,
	             NRF_GPIO_PIN_NOSENSE);
	nrf_gpio_pin_set(tx_pin);
	nrf_gpio_cfg(rx_pin, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1,
	             NRF_GPIO_PIN_NOSENSE);
}

// ============================================================================
// TX
// ============================================================================

bool NrfUartAsync::send_raw(const uint8_t *data, size_t len) {
	if (!m_is_init || len == 0) return false;

	// DMA TX is async: the peripheral keeps reading from the buffer after this
	// returns. The caller typically passes a stack buffer, so we copy into
	// m_tx_buffer which is held until TX_DONE clears m_is_send_busy.
	if (m_is_send_busy) {
		DEBUG_ERROR("NrfUartAsync::send_raw: UART%u already busy", m_uart_instance);
		return false;
	}

	if (!m_is_rx_started) {
		nrf_libuarte_async_start_rx(BSP::UARTAsync_Inits[m_uart_instance].uart);
		m_is_rx_started = true;
	}

	m_tx_buffer.assign(reinterpret_cast<const char *>(data), len);
	m_is_send_busy = true;

	ret_code_t ret = nrf_libuarte_async_tx(BSP::UARTAsync_Inits[m_uart_instance].uart,
	                                       reinterpret_cast<uint8_t *>(m_tx_buffer.data()), m_tx_buffer.length());

	if (ret != NRF_SUCCESS) {
		m_is_send_busy = false;
		DEBUG_ERROR("NrfUartAsync::send_raw: UART%u TX failed (0x%08X)", m_uart_instance, static_cast<unsigned>(ret));
	}
	return (ret == NRF_SUCCESS);
}

bool NrfUartAsync::send_string(const std::string &str) {
	if (m_is_send_busy) {
		DEBUG_ERROR("NrfUartAsync: UART%u already busy", m_uart_instance);
		return false;
	}
	m_is_send_busy = true;

	if (!m_is_rx_started) {
		nrf_libuarte_async_start_rx(BSP::UARTAsync_Inits[m_uart_instance].uart);
		m_is_rx_started = true;
	}

	// Store in member buffer so it stays valid during async DMA transfer
	m_tx_buffer = str;

	ret_code_t ret = nrf_libuarte_async_tx(BSP::UARTAsync_Inits[m_uart_instance].uart,
	                                       reinterpret_cast<uint8_t *>(m_tx_buffer.data()), m_tx_buffer.length());

	if (ret != NRF_SUCCESS) {
		m_is_send_busy = false;
		DEBUG_ERROR("NrfUartAsync: UART%u TX failed (0x%08X)", m_uart_instance, static_cast<unsigned>(ret));
	}

	return (ret == NRF_SUCCESS);
}

// ============================================================================
// ISR callbacks — minimal work, no heap allocation
// ============================================================================

void NrfUartAsync::isr_handle_tx_done() {
	m_is_send_busy = false;
}

/// @brief ISR: copy raw data to ISR buffer and free DMA buffer.
/// No string operations or notifications — all processing deferred to process_rx().
void NrfUartAsync::isr_handle_rx_data(uint8_t *buffer, uint16_t length) {
	uint16_t cur_len = m_isr_buf_len;
	uint16_t space = ISR_BUF_SIZE - cur_len;
	uint16_t copy_len = (length < space) ? length : space;
	memcpy(m_isr_buf + cur_len, buffer, copy_len);
	m_isr_buf_len = static_cast<uint16_t>(cur_len + copy_len);
	if (copy_len < length) m_isr_overflow = true;

	nrf_libuarte_async_rx_free(BSP::UARTAsync_Inits[m_uart_instance].uart, buffer, length);
}

/// @brief ISR: set error flag for deferred handling.
void NrfUartAsync::isr_handle_error(unsigned int error_type) {
	nrf_libuarte_async_stop_rx(BSP::UARTAsync_Inits[m_uart_instance].uart);
	m_is_rx_started = false;
	m_isr_error = true;
	m_isr_error_type = error_type;
}

// ============================================================================
// Deferred RX processing (main context only)
// ============================================================================

void NrfUartAsync::process_rx() {
	// Check for UART error (deferred from ISR)
	if (m_isr_error) {
		unsigned int err_type = m_isr_error_type;
		m_isr_error = false;
		m_rx_buffer.clear();
		m_isr_buf_len = 0;

		// isr_handle_error() a coupe la reception. Elle n'etait relancee que par
		// send_raw()/send_string(), donc UNIQUEMENT a la prochaine emission — on
		// restait sourd jusque-la. Mesure au banc le 2026-08-25 sur une salve
		// Argos BLIND: le module cadence lui-meme ses retransmissions et on ne lui
		// envoie plus rien pendant toute la salve; une erreur de trame survenue a
		// son reveil nous rendait sourds pour de bon, et le +TX= de cloture etait
		// perdu — fenetre expiree, TX compte en echec, rail du module coupe.
		// Aucun des on_rx_error() (KIM2, SMD-AT, LoRa) ne relancait la reception,
		// donc personne ne dependait de cet etat: on restaure l'invariant "port
		// ouvert => reception armee" ici, avant de remonter l'erreur.
		if (!m_is_rx_started) {
			nrf_libuarte_async_start_rx(BSP::UARTAsync_Inits[m_uart_instance].uart);
			m_is_rx_started = true;
		}

		on_rx_error(err_type);
		return;
	}

	// Check for ISR RX buffer overflow
	if (m_isr_overflow) {
		m_isr_overflow = false;
		DEBUG_ERROR("NrfUartAsync: UART%u ISR RX overflow — data lost", m_uart_instance);
	}

	// Nothing to process
	if (m_isr_buf_len == 0) return;

	// Copy ISR buffer under interrupt lock to prevent race
	uint8_t local_buf[ISR_BUF_SIZE];
	uint16_t local_len;
	{
		InterruptLock lock;
		local_len = m_isr_buf_len;
		memcpy(local_buf, m_isr_buf, local_len);
		m_isr_buf_len = 0;
	}

	// Guard against unbounded growth if data arrives without line terminators
	if (m_rx_buffer.size() + local_len > MAX_RX_BUFFER_SIZE) {
		DEBUG_ERROR("NrfUartAsync: UART%u RX buffer overflow (%u bytes) — flushing", m_uart_instance,
		            static_cast<unsigned>(m_rx_buffer.size() + local_len));
		m_rx_buffer.clear();
	}

	// Append to line accumulator and process complete lines
	m_rx_buffer.append(reinterpret_cast<const char *>(local_buf), local_len);
	process_rx_lines();
}

/// @brief Extract complete lines from m_rx_buffer and call on_rx_line().
void NrfUartAsync::process_rx_lines() {
	size_t pos;
	while ((pos = m_rx_buffer.find('\n')) != std::string::npos) {
		// Reuse m_line_buffer — assign() within reserved capacity is just memcpy
		m_line_buffer.assign(m_rx_buffer, 0, pos + 1);
		m_rx_buffer.erase(0, pos + 1);

		// Strip trailing CR/LF
		while (!m_line_buffer.empty() && (m_line_buffer.back() == '\r' || m_line_buffer.back() == '\n'))
			m_line_buffer.pop_back();

		if (m_line_buffer.empty()) continue;

		on_rx_line(m_line_buffer);
	}
}
