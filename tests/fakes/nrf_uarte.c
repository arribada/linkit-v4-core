#include "nrf_uarte.h"

/* Dernier debit demande au peripherique. Expose pour que les tests puissent
 * verifier la sequence de sondage du baud au demarrage du M10Q (regression
 * "failed to sync comms" du 2026-08 : le recepteur repart a 460800 quand la
 * BBR a survecu, et le driver ne sondait que 9600). Volontairement une simple
 * variable et non un mock : un actualCall ferait echouer tous les tests
 * existants qui ne l'attendent pas. */
nrf_uarte_baudrate_t g_fake_last_baudrate = NRF_UARTE_BAUDRATE_9600;

void nrf_uarte_baudrate_set(NRF_UARTE_Type *p_reg, nrf_uarte_baudrate_t baudrate) {
	(void)p_reg;
	g_fake_last_baudrate = baudrate;
}
