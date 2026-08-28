#pragma once

#include <cstdint>
#include "nrfx_rtc.h"
#include "nrfx_saadc.h"
#include "nrf_libuarte_async.h"

#define RTC_TIMER      BSP::RTC::RTC_1
#define SWS_ENABLE_PIN BSP::GPIO::GPIO_SLOW_SWS_SEND
#define SWS_SAMPLE_PIN BSP::GPIO::GPIO_SLOW_SWS_RX
#define SWS_ADC        BSP::ADC::ADC_CHANNEL_1
#define BATTERY_ADC    BSP::ADC::ADC_CHANNEL_0
#define GPS_POWER      BSP::GPIO::GPIO_GPS_PWR_EN
#define GPS_RST        BSP::GPIO::GPIO_GPS_RST
#define SAT_PWR_EN     BSP::GPIO::GPIO_SAT_EN
#define SAT_EXTWAKEUP  BSP::GPIO::GPIO_SAT_WKUP

#define ADC_GAIN       (1.0f / 6.0f)  // 1/6 gain
#define V_DIV_GAIN     2.0f
#define RP506_ADC_GAIN 4.0f

namespace BSP {
enum RTC {
	RTC_RESERVED,  // Reserved by the softdevice
	RTC_1,
	RTC_2,
	RTC_TOTAL_NUMBER
};

typedef struct {
	nrfx_rtc_t rtc;
	uint8_t irq_priority;
} RTC_InitTypeDefAndInst_t;

extern const RTC_InitTypeDefAndInst_t RTC_Inits[RTC_TOTAL_NUMBER];

enum GPIO {
	GPIO_SWITCH,
	GPIO_POWER_CONTROL,
	GPIO_SLOW_SWS_SEND,
	GPIO_SLOW_SWS_RX,
	GPIO_GPS_PWR_EN,
	GPIO_GPS_RST,
	GPIO_GPS_EXT_INT,
	GPIO_SAT_EN,
	GPIO_SAT_WKUP,
	GPIO_TOTAL_NUMBER
};

typedef struct {
	int pin_number;
	void *gpiote_in_config;
} GPIO_InitTypeDefAndInst_t;

extern const GPIO_InitTypeDefAndInst_t GPIO_Inits[GPIO_TOTAL_NUMBER];

enum ADC {
	ADC_CHANNEL_0,  // Battery voltage
	ADC_CHANNEL_1,  // SWS analog
	ADC_TOTAL_CHANNELS
};

typedef struct {
	nrfx_saadc_config_t config;
	nrf_saadc_channel_config_t channel_config[ADC_TOTAL_CHANNELS];
} ADC_InitTypeDefAndInst_t;

extern const ADC_InitTypeDefAndInst_t ADC_Inits;

enum WDT { WDT, WDT_TOTAL_NUMBER };

typedef struct {
	struct {
		unsigned int reload_value;
	} config;
} WDT_InitTypeDefAndInst_t;

extern const WDT_InitTypeDefAndInst_t WDT_Inits[WDT_TOTAL_NUMBER];

typedef struct {
	const nrf_libuarte_async_t *uart;
	const nrf_libuarte_async_config_t config;
} UARTAsync_InitTypeDefAndInst_t;

extern const UARTAsync_InitTypeDefAndInst_t UARTAsync_Inits[1];
}
