#pragma once

#include <stdint.h>

// Fake NRF_POWER peripheral for test builds
typedef struct {
	volatile uint32_t GPREGRET;
	volatile uint32_t GPREGRET2;
} NRF_POWER_Type;

static NRF_POWER_Type nrf_power_fake_instance;
#define NRF_POWER (&nrf_power_fake_instance)
