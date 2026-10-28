/**
 * @file runcam_uart_link.cpp
 * @brief RunCamLink on LinkIt V4 — see runcam_uart_link.hpp.
 */

#include "runcam_uart_link.hpp"

#include <cstdio>
#include <cstring>

#include "bsp.hpp"
#include "debug.hpp"
#include "gpio.hpp"

NrfRunCamLink::NrfRunCamLink() : NrfUartAsync(CAM_UART_INSTANCE) {}

/// @brief The camera rail. GPIOPins::set() also raises the VSYS interlock
/// (is_cam_rail_active): the camera's 3.3 V TX must never meet a 2.3 V nRF.
void NrfRunCamLink::set_rail(bool on) {
	if (on)
		GPIOPins::set(CAM_PWR_EN);
	else
		GPIOPins::clear(CAM_PWR_EN);
}

bool NrfRunCamLink::open() {
	try {
		NrfUartAsync::init();  // BSP baud rate: 115200, fixed by the protocol
	} catch (...) {
		DEBUG_ERROR("NrfRunCamLink: UART%u init failed", static_cast<unsigned int>(CAM_UART_INSTANCE));
		return false;
	}
	m_rx_len = 0;
	return true;
}

/// @brief deinit() leaves TX parked HIGH, right for a powered peer; the rail is
/// about to be cut, so drive it LOW instead (the KIM2 43 µA leak, 34ebb4a1).
/// Also called on a link that never opened: TX then goes from high-Z to LOW.
void NrfRunCamLink::close() {
	NrfUartAsync::deinit();
	NrfUartAsync::park_tx_for_power_off();
	m_rx_len = 0;
}

bool NrfRunCamLink::is_open() const {
	return NrfUartAsync::is_init();
}

bool NrfRunCamLink::send(const uint8_t *data, size_t len) {
	return NrfUartAsync::send_raw(data, len);
}

size_t NrfRunCamLink::receive(uint8_t *buf, size_t max) {
	// Pull the next ISR batch in only once the previous one is handed over: at
	// most ISR_BUF_SIZE bytes, which RX_BUF_SIZE holds whole.
	if (m_rx_len == 0 && NrfUartAsync::is_init()) NrfUartAsync::process_rx();
	const size_t n = (m_rx_len < max) ? m_rx_len : max;
	if (n == 0) return 0;
	memcpy(buf, m_rx, n);
	memmove(m_rx, m_rx + n, m_rx_len - n);
	m_rx_len -= n;
	return n;
}

void NrfRunCamLink::on_rx_data(const uint8_t *data, size_t len) {
	const size_t room = RX_BUF_SIZE - m_rx_len;
	const size_t n = (len < room) ? len : room;
	memcpy(m_rx + m_rx_len, data, n);
	m_rx_len += n;
	if (n < len) m_rx_errors++;  // cannot happen while receive() refills only when empty
}

void NrfRunCamLink::on_rx_error(unsigned int error_type) {
	m_rx_errors++;
	DEBUG_TRACE("NrfRunCamLink: UART error 0x%X", error_type);
}

/// @brief One line of driver and peripheral state, for the bench (%CAM LINK):
/// what to look at when the link has gone deaf.
void NrfRunCamLink::describe(char *out, size_t out_size) const {
	const nrf_libuarte_async_t *a = BSP::UARTAsync_Inits[CAM_UART_INSTANCE].uart;
	const nrf_libuarte_drv_ctrl_blk_t *d = a->p_libuarte->ctrl_blk;
	const NRF_UARTE_Type *u = a->p_libuarte->uarte;
	const uint32_t rx = BSP::UARTAsync_Inits[CAM_UART_INSTANCE].config.rx_pin;
	const uint32_t tx = BSP::UARTAsync_Inits[CAM_UART_INSTANCE].config.tx_pin;
	snprintf(out, out_size,
	         "init=%u rx_started=%u isr_err=%u isr_len=%u rx_len=%u busy=%u | async en=%u rx_en=%u halted=%u alloc=%d "
	         "| drv en=%u cur=%u next=%u tx=%u | uarte EN=%lu RXD=%lu TXD=%lu ERR=0x%lX AMOUNT=%lu RXSTARTED=%lu "
	         "ENDRX=%lu RXTO=%lu ERROR=%lu INTEN=0x%lX | rxpin cnf=0x%lX in=%lu txpin cnf=0x%lX out=%lu",
	         is_init() ? 1u : 0u, m_is_rx_started ? 1u : 0u, isr_error_pending() ? 1u : 0u,
	         (unsigned int)isr_pending_bytes(), (unsigned int)m_rx_len, m_is_send_busy ? 1u : 0u,
	         a->p_ctrl_blk->enabled ? 1u : 0u, a->p_ctrl_blk->rx_enabled ? 1u : 0u, a->p_ctrl_blk->rx_halted ? 1u : 0u,
	         (int)a->p_ctrl_blk->alloc_cnt, d->enabled ? 1u : 0u, d->p_cur_rx ? 1u : 0u, d->p_next_rx ? 1u : 0u,
	         d->p_tx ? 1u : 0u, (unsigned long)u->ENABLE, (unsigned long)u->PSEL.RXD, (unsigned long)u->PSEL.TXD,
	         (unsigned long)u->ERRORSRC, (unsigned long)u->RXD.AMOUNT, (unsigned long)u->EVENTS_RXSTARTED,
	         (unsigned long)u->EVENTS_ENDRX, (unsigned long)u->EVENTS_RXTO, (unsigned long)u->EVENTS_ERROR,
	         (unsigned long)u->INTEN, (unsigned long)NRF_P0->PIN_CNF[rx & 0x1F],
	         (unsigned long)((NRF_P0->IN >> (rx & 0x1F)) & 1), (unsigned long)NRF_P0->PIN_CNF[tx & 0x1F],
	         (unsigned long)((NRF_P0->OUT >> (tx & 0x1F)) & 1));
}
