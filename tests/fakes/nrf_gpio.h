/**
 * @file nrf_gpio.h (test fake)
 * @brief Minimal host-side stub for Nordic SDK nrf_gpio.h. Drivers that
 *        directly call nrf_gpio_* (e.g. nrf_uart_async, lora_rak3172) can
 *        link cleanly in the host test binary without pulling in the SDK.
 */

#ifndef TESTS_FAKES_NRF_GPIO_H
#define TESTS_FAKES_NRF_GPIO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NRF_GPIO_PIN_MAP(port, pin) (((port) << 5) | ((pin) & 0x1F))

typedef enum {
	NRF_GPIO_PIN_DIR_INPUT = 0,
	NRF_GPIO_PIN_DIR_OUTPUT = 1,
} nrf_gpio_pin_dir_t;

typedef enum {
	NRF_GPIO_PIN_INPUT_CONNECT = 0,
	NRF_GPIO_PIN_INPUT_DISCONNECT = 1,
} nrf_gpio_pin_input_t;

typedef enum {
	NRF_GPIO_PIN_NOPULL = 0,
	NRF_GPIO_PIN_PULLDOWN = 1,
	NRF_GPIO_PIN_PULLUP = 3,
} nrf_gpio_pin_pull_t;

typedef enum {
	NRF_GPIO_PIN_S0S1 = 0,
	NRF_GPIO_PIN_H0S1 = 1,
	NRF_GPIO_PIN_S0H1 = 2,
	NRF_GPIO_PIN_H0H1 = 3,
	NRF_GPIO_PIN_D0S1 = 4,
	NRF_GPIO_PIN_D0H1 = 5,
	NRF_GPIO_PIN_S0D1 = 6,
	NRF_GPIO_PIN_H0D1 = 7,
} nrf_gpio_pin_drive_t;

typedef enum {
	NRF_GPIO_PIN_NOSENSE = 0,
	NRF_GPIO_PIN_SENSE_LOW = 2,
	NRF_GPIO_PIN_SENSE_HIGH = 3,
} nrf_gpio_pin_sense_t;

static inline void nrf_gpio_cfg(uint32_t pin_number, nrf_gpio_pin_dir_t dir, nrf_gpio_pin_input_t input,
                                nrf_gpio_pin_pull_t pull, nrf_gpio_pin_drive_t drive, nrf_gpio_pin_sense_t sense) {
	(void)pin_number;
	(void)dir;
	(void)input;
	(void)pull;
	(void)drive;
	(void)sense;
}

static inline void nrf_gpio_pin_set(uint32_t pin_number) {
	(void)pin_number;
}
static inline void nrf_gpio_pin_clear(uint32_t pin_number) {
	(void)pin_number;
}
static inline uint32_t nrf_gpio_pin_read(uint32_t pin_number) {
	(void)pin_number;
	return 0;
}

#ifdef __cplusplus
}
#endif

#endif /* TESTS_FAKES_NRF_GPIO_H */
