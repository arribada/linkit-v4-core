/**
 * @file nrf_usb.cpp
 * @brief USB CDC ACM implementation — debug output and DTE command input.
 */

#include "nrf_usb.hpp"
#include "nrf_drv_clock.h"
#include "app_usbd.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"
#include "app_util_platform.h"  // CRITICAL_REGION_ENTER/EXIT
#include "pmu.hpp"
#include "debug.hpp"
#include <cstring>

/// @brief Process all pending USB events from the queue.
static inline void usb_queue_process() {
	while (app_usbd_event_queue_process()) {}
}

#define CDC_ACM_COMM_INTERFACE 0
#define CDC_ACM_COMM_EPIN      NRF_DRV_USBD_EPIN2
#define CDC_ACM_DATA_INTERFACE 1
#define CDC_ACM_DATA_EPIN      NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT     NRF_DRV_USBD_EPOUT1

static constexpr size_t USB_CDC_READ_SIZE = 64;  ///< Max bytes per USB CDC read transfer

// Static member initialization
bool NrfUSB::m_port_open = false;
char NrfUSB::m_rx_buffer[NRF_USB_RX_BUFFER_SIZE];
volatile size_t NrfUSB::m_rx_write_idx = 0;
volatile size_t NrfUSB::m_rx_read_idx = 0;
volatile bool NrfUSB::m_line_ready = false;

/// @brief Temporary buffer for USB CDC reads (single transfer).
static char s_usb_rx_temp[USB_CDC_READ_SIZE];

static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const *p_inst, app_usbd_cdc_acm_user_event_t event);

extern "C" {
APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm, cdc_acm_user_ev_handler, CDC_ACM_COMM_INTERFACE, CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN, CDC_ACM_DATA_EPIN, CDC_ACM_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_AT_V250);
}

// TX path. app_usbd_cdc_acm_write() does not copy: the USB peripheral reads the
// caller's buffer packet by packet, whenever the host polls, and a write made
// while a transfer is in flight is refused (NRF_ERROR_BUSY) and was silently
// dropped. Callers pass stack buffers (printf) and temporaries (DTE replies), so
// a slow host read the transfer out of reused memory: a DTE reply came out as a
// fragment, RAM bytes and a later log line (bench, 2026-10-10). Every write is
// now copied into this ring and sent from a static chunk, one transfer at a time,
// the next one started on TX_DONE.
static constexpr size_t USB_TX_RING_SIZE = 4096;
static constexpr size_t USB_TX_CHUNK_SIZE = 256;
static char s_tx_ring[USB_TX_RING_SIZE];
static volatile size_t s_tx_head = 0;  ///< next byte written
static volatile size_t s_tx_tail = 0;  ///< next byte sent
static char s_tx_chunk[USB_TX_CHUNK_SIZE] __attribute__((aligned(4)));
static volatile bool s_tx_busy = false;
static volatile uint32_t s_tx_dropped = 0;  ///< bytes lost to a full ring (host gone or not reading)

/// @brief Start the next transfer if none is in flight (thread context only).
static void tx_pump() {
	if (s_tx_busy || !NrfUSB::is_port_open()) return;
	size_t n = 0;
	{
		CRITICAL_REGION_ENTER();
		while (n < USB_TX_CHUNK_SIZE && s_tx_tail != s_tx_head) {
			s_tx_chunk[n++] = s_tx_ring[s_tx_tail];
			s_tx_tail = (s_tx_tail + 1) % USB_TX_RING_SIZE;
		}
		CRITICAL_REGION_EXIT();
	}
	if (n == 0) return;
	s_tx_busy = true;
	if (app_usbd_cdc_acm_write(&m_app_cdc_acm, s_tx_chunk, n) != NRF_SUCCESS) {
		s_tx_busy = false;
		s_tx_dropped = s_tx_dropped + n;  // port not ready: nothing sensible to retry with
	}
}

static void tx_reset() {
	CRITICAL_REGION_ENTER();
	s_tx_head = 0;
	s_tx_tail = 0;
	s_tx_busy = false;
	CRITICAL_REGION_EXIT();
}

/// @brief Kick off a new async USB CDC read if port is open.
static void start_usb_read() {
	if (NrfUSB::is_port_open()) app_usbd_cdc_acm_read_any(&m_app_cdc_acm, s_usb_rx_temp, USB_CDC_READ_SIZE);
}

/// @brief CDC ACM user event handler — manages port state and RX data.
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const *p_inst, app_usbd_cdc_acm_user_event_t event) {
	switch (event) {
	case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
		NrfUSB::set_port_open(true);
		start_usb_read();
		break;
	case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
		NrfUSB::set_port_open(false);
		tx_reset();
		break;
	case APP_USBD_CDC_ACM_USER_EVT_RX_DONE: {
		size_t rx_size = app_usbd_cdc_acm_rx_size(&m_app_cdc_acm);
		if (rx_size > 0) NrfUSB::on_rx_data(s_usb_rx_temp, rx_size);
		start_usb_read();
		break;
	}
	case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
		s_tx_busy = false;
		tx_pump();
		break;
	default: break;
	}
}

/// @brief USBD state machine handler — enable/disable/start/stop on power events.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wswitch-enum"
static void usbd_user_ev_handler(app_usbd_event_type_t event) {
	switch (event) {
	case APP_USBD_EVT_STOPPED: app_usbd_disable(); break;
	case APP_USBD_EVT_POWER_DETECTED:
		if (!nrf_drv_usbd_is_enabled()) app_usbd_enable();
		break;
	case APP_USBD_EVT_POWER_REMOVED: app_usbd_stop(); break;
	case APP_USBD_EVT_POWER_READY: app_usbd_start(); break;
	default: break;
	}
}
#pragma GCC diagnostic pop


// ═══════════════════════════════════════════════════════
//  Init
// ═══════════════════════════════════════════════════════

void NrfUSB::init() {
	m_rx_write_idx = 0;
	m_rx_read_idx = 0;
	m_line_ready = false;

	static const app_usbd_config_t usbd_config = {
		.ev_isr_handler = nullptr,
		.ev_state_proc = usbd_user_ev_handler,
		.enable_sof = false,
	};

	nrf_drv_clock_init();
	nrf_drv_clock_lfclk_request(nullptr);

	// Wait for LFCLK to start (bounded — crystal startup is typically < 1 ms)
	constexpr unsigned int LFCLK_TIMEOUT_MS = 100;
	for (unsigned int ms = 0; ms < LFCLK_TIMEOUT_MS; ms++) {
		if (nrf_drv_clock_lfclk_is_running()) break;
		PMU::delay_ms(1);
	}
	if (!nrf_drv_clock_lfclk_is_running()) {
		DEBUG_ERROR("NrfUSB: LFCLK failed to start");
		return;
	}

	app_usbd_serial_num_generate();

	ret_code_t ret = app_usbd_init(&usbd_config);
	if (ret != NRF_SUCCESS) {
		DEBUG_ERROR("NrfUSB: app_usbd_init failed (0x%08X)", ret);
		return;
	}

	app_usbd_class_inst_t const *class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
	ret = app_usbd_class_append(class_cdc_acm);
	if (ret != NRF_SUCCESS) {
		DEBUG_ERROR("NrfUSB: class_append failed (0x%08X)", ret);
		return;
	}

	ret = app_usbd_power_events_enable();
	if (ret != NRF_SUCCESS) {
		DEBUG_ERROR("NrfUSB: power_events_enable failed (0x%08X)", ret);
		return;
	}

	// Poll at init in case USB is already plugged
	for (unsigned int ms = 0; ms < 500; ms++) {
		usb_queue_process();
		if (m_port_open) break;
		PMU::delay_ms(1);
	}
}


// ═══════════════════════════════════════════════════════
//  TX / RX
// ═══════════════════════════════════════════════════════

/// @brief Copy into the TX ring, then send what the USB can take now.
/// In interrupt context the data is only queued: the USB event queue belongs to
/// the main loop. In thread context a full ring is drained for up to 50 ms (a
/// large DTE reply such as a full PARMR exceeds the ring) before bytes are
/// dropped -- never blocking for long on a host that stopped reading.
int NrfUSB::write(char *ptr, int len) {
	if (!m_port_open) return len;
	const bool in_isr = (__get_IPSR() != 0);
	int i = 0;
	uint32_t waited_ms = 0;
	while (i < len) {
		bool stored = false;
		{
			CRITICAL_REGION_ENTER();
			const size_t next = (s_tx_head + 1) % USB_TX_RING_SIZE;
			if (next != s_tx_tail) {
				s_tx_ring[s_tx_head] = ptr[i];
				s_tx_head = next;
				stored = true;
			}
			CRITICAL_REGION_EXIT();
		}
		if (stored) {
			i++;
			continue;
		}
		if (in_isr || waited_ms >= 50) {
			s_tx_dropped = s_tx_dropped + (len - i);
			break;
		}
		usb_queue_process();
		tx_pump();
		PMU::delay_ms(1);
		waited_ms++;
	}
	if (!in_isr) {
		usb_queue_process();
		tx_pump();
	}
	return len;
}

uint32_t NrfUSB::tx_dropped() {
	return s_tx_dropped;
}

/// @brief Store received data in ring buffer (called from USB ISR).
void NrfUSB::on_rx_data(const char *data, size_t len) {
	for (size_t i = 0; i < len; i++) {
		size_t next_write = (m_rx_write_idx + 1) % NRF_USB_RX_BUFFER_SIZE;
		if (next_write == m_rx_read_idx) break;  // Buffer full — discard rest

		m_rx_buffer[m_rx_write_idx] = data[i];
		m_rx_write_idx = next_write;

		if (data[i] == '\r' || data[i] == '\n') m_line_ready = true;
	}
}

/// @brief True if the ring buffer contains unread data.
bool NrfUSB::has_data() {
	return m_rx_write_idx != m_rx_read_idx;
}

/// @brief Read raw bytes from the ring buffer.
int NrfUSB::read(char *ptr, int max_len) {
	int count = 0;
	while (m_rx_read_idx != m_rx_write_idx && count < max_len) {
		ptr[count++] = m_rx_buffer[m_rx_read_idx];
		m_rx_read_idx = (m_rx_read_idx + 1) % NRF_USB_RX_BUFFER_SIZE;
	}
	return count;
}

/// @brief Read one complete line from the ring buffer, or return empty string.
std::string NrfUSB::read_line() {
	std::string result;

	usb_queue_process();

	if (!m_line_ready) return result;

	// Extract characters until line terminator
	while (m_rx_read_idx != m_rx_write_idx) {
		char c = m_rx_buffer[m_rx_read_idx];
		m_rx_read_idx = (m_rx_read_idx + 1) % NRF_USB_RX_BUFFER_SIZE;

		if (c == '\r' || c == '\n') {
			// Skip trailing \n or \r (handle \r\n and \n\r)
			if (m_rx_read_idx != m_rx_write_idx) {
				char next = m_rx_buffer[m_rx_read_idx];
				if ((next == '\r' || next == '\n') && next != c)
					m_rx_read_idx = (m_rx_read_idx + 1) % NRF_USB_RX_BUFFER_SIZE;
			}
			break;
		}
		result += c;
	}

	// Check if another complete line remains in the buffer
	m_line_ready = false;
	size_t idx = m_rx_read_idx;
	while (idx != m_rx_write_idx) {
		if (m_rx_buffer[idx] == '\r' || m_rx_buffer[idx] == '\n') {
			m_line_ready = true;
			break;
		}
		idx = (idx + 1) % NRF_USB_RX_BUFFER_SIZE;
	}

	return result;
}

/// @brief Process pending USB events — call from main loop.
void NrfUSB::process() {
#ifdef BENCH_TEST
	// Bench harness runs over WSL2/usbip, where the host CDC DTR assertion
	// (SET_CONTROL_LINE_STATE) that normally raises APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN
	// is not reliably forwarded. Without PORT_OPEN the RX is never armed and every
	// TX (logs AND '%' console replies) is gated off — the link looks dead. Once the
	// USB device is enumerated/enabled, force the port open and arm RX so the bench
	// console works regardless of host DTR behaviour. Idempotent; harmless if a real
	// PORT_OPEN later fires. BENCH_TEST builds only — production keeps strict gating.
	if (!m_port_open && nrf_drv_usbd_is_enabled()) {
		set_port_open(true);
		start_usb_read();
	}
#endif
	usb_queue_process();
	tx_pump();
}

/// @brief Update port state and reset ring buffer on close.
void NrfUSB::set_port_open(bool is_open) {
	m_port_open = is_open;
	if (!is_open) {
		m_rx_write_idx = 0;
		m_rx_read_idx = 0;
		m_line_ready = false;
	}
}
