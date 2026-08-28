#include "nrf_uarte.h"

/* Dernier debit demande au peripherique. Expose pour que les tests puissent
 * check the baud probe sequence at M10Q start-up (the 2026-08 "failed to sync
 * comms" regression: the receiver comes back at 460800 when the BBR survived, and
 * the driver only probed 9600). Deliberately a plain variable rather than a mock:
 * an actualCall would fail every existing test that does not expect it. */
nrf_uarte_baudrate_t g_fake_last_baudrate = NRF_UARTE_BAUDRATE_9600;

void nrf_uarte_baudrate_set(NRF_UARTE_Type *p_reg, nrf_uarte_baudrate_t baudrate) {
	(void)p_reg;
	g_fake_last_baudrate = baudrate;
}
