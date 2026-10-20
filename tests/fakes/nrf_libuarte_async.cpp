#include <stdbool.h>
#include <vector>
#include "nrf_libuarte_async.h"
#include "debug.hpp"

/* Every byte handed to the UART, in order, so a test can check what the driver
 * actually sent to the receiver (frame order, polls, assistance). */
static std::vector<uint8_t> s_tx_log;

void nrf_libuarte_tx_log_clear(void) {
	s_tx_log.clear();
}
size_t nrf_libuarte_tx_log_size(void) {
	return s_tx_log.size();
}
const uint8_t *nrf_libuarte_tx_log_data(void) {
	return s_tx_log.data();
}

static nrf_libuarte_async_evt_handler_t m_evt_callback;
static void *m_context;
/* Etat RX expose aux tests: `handle_error()` arrete le RX et seul un
 * an explicit restart brings it back -- the invariant checked by the
 * FramingErrorDuringConfigureRestartsRx regression test. */
bool m_is_rx_enabled = false;

ret_code_t nrf_libuarte_async_init(const nrf_libuarte_async_t *const p_libuarte,
                                   nrf_libuarte_async_config_t const *p_config,
                                   nrf_libuarte_async_evt_handler_t evt_handler, void *context) {
	(void)p_libuarte;
	(void)p_config;
	m_evt_callback = evt_handler;
	m_context = context;
	DEBUG_TRACE("nrf_libuarte_async_init");
	return 0;
}

void nrf_libuarte_async_uninit(const nrf_libuarte_async_t *const p_libuarte) {
	(void)p_libuarte;
	DEBUG_TRACE("nrf_libuarte_async_uninit");
}

void nrf_libuarte_async_start_rx(const nrf_libuarte_async_t *const p_libuarte) {
	(void)p_libuarte;
	m_is_rx_enabled = true;
	DEBUG_TRACE("nrf_libuarte_async_start_rx");
}

void nrf_libuarte_async_stop_rx(const nrf_libuarte_async_t *const p_libuarte) {
	(void)p_libuarte;
	m_is_rx_enabled = false;
	DEBUG_TRACE("nrf_libuarte_async_stop_rx");
}

void nrf_libuarte_async_set_timeout(const nrf_libuarte_async_t *const p_libuarte, unsigned int timeout_us) {
	(void)p_libuarte;
	(void)timeout_us;
	DEBUG_TRACE("nrf_libuarte_async_set_timeout");
}

ret_code_t nrf_libuarte_async_tx(const nrf_libuarte_async_t *const p_libuarte, uint8_t *p_data, size_t length) {
	(void)p_libuarte;
	if (p_data && length) s_tx_log.insert(s_tx_log.end(), p_data, p_data + length);
	DEBUG_TRACE("nrf_libuarte_async_tx");
	nrf_libuarte_async_evt_t evt;
	evt.type = NRF_LIBUARTE_ASYNC_EVT_TX_DONE;
	m_evt_callback(m_context, &evt);
	return 0;
}

void nrf_libuarte_async_rx_free(const nrf_libuarte_async_t *const p_libuarte, uint8_t *p_data, size_t length) {
	(void)p_libuarte;
	(void)p_data;
	(void)length;
	DEBUG_TRACE("nrf_libuarte_async_rx_free");
}

void nrf_libuarte_inject_event(nrf_libuarte_async_evt_t *evt) {
	if (m_is_rx_enabled) m_evt_callback(m_context, evt);
}
