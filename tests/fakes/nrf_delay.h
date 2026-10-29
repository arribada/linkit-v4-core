#pragma once

#include <stdint.h>

// Fake Nordic SDK busy-wait delays for test builds: no time passes on the host.
static inline void nrf_delay_ms(uint32_t ms_time) {
	(void)ms_time;
}
static inline void nrf_delay_us(uint32_t us_time) {
	(void)us_time;
}
