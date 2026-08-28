/**
 * @file fake_bsp.cpp
 * @brief Fake BSP definitions for unit tests
 */

#include "bsp.hpp"

namespace BSP {
const RTC_InitTypeDefAndInst_t RTC_Inits[RTC_TOTAL_NUMBER] = {
	{ .rtc = {}, .irq_priority = 0 },
	{ .rtc = {}, .irq_priority = 0 },
	{ .rtc = {}, .irq_priority = 0 },
};

const GPIO_InitTypeDefAndInst_t GPIO_Inits[GPIO_TOTAL_NUMBER] = {
	{ .pin_number = 0, .gpiote_in_config = nullptr }, { .pin_number = 1, .gpiote_in_config = nullptr },
	{ .pin_number = 2, .gpiote_in_config = nullptr }, { .pin_number = 3, .gpiote_in_config = nullptr },
	{ .pin_number = 4, .gpiote_in_config = nullptr }, { .pin_number = 5, .gpiote_in_config = nullptr },
	{ .pin_number = 6, .gpiote_in_config = nullptr },
};

const ADC_InitTypeDefAndInst_t ADC_Inits = { .config = {}, .channel_config = {} };

const WDT_InitTypeDefAndInst_t WDT_Inits[WDT_TOTAL_NUMBER] = { { .config = { .reload_value = 60000 } } };

static nrf_libuarte_async_t fake_uart = {};
const UARTAsync_InitTypeDefAndInst_t UARTAsync_Inits[1] = { { .uart = &fake_uart, .config = {} } };
}
