#include "bsp.hpp"

namespace BSP
{
	///////////////////////////////// GPIO definitions ////////////////////////////////
	const GPIO_InitTypeDefAndInst_t GPIO_Inits[GPIO_TOTAL_NUMBER] =
	{
		// pin number, direction, input, pull, drive sense
		/* GPIO_POWER_CONTROL   */ {NRF_GPIO_PIN_MAP(0,  4), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_D0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_DEBUG           */ {NRF_GPIO_PIN_MAP(0, 11), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_SLOW_SWS_RX     */ {NRF_GPIO_PIN_MAP(0,  9), NRF_GPIO_PIN_DIR_INPUT,  NRF_GPIO_PIN_INPUT_CONNECT,    NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_SLOW_SWS_SEND   */ {NRF_GPIO_PIN_MAP(0, 10), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_SENSORS_PWR     */ {NRF_GPIO_PIN_MAP(0, 25), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_TEMP_ANALOG     */ {NRF_GPIO_PIN_MAP(0, 28), NRF_GPIO_PIN_DIR_INPUT,  NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_SAT_RESET       */ {NRF_GPIO_PIN_MAP(0, 31), NRF_GPIO_PIN_DIR_INPUT,  NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0D1, NRF_GPIO_PIN_NOSENSE, {}},  // INPUT for probe flashing - change back to OUTPUT for normal operation
		// Push-pull outputs carry no pull: see the top of the table in
		// linkitv4_v1.0/bsp.cpp (a pull-down on a HIGH output conducts ~0.25 mA).
		/* GPIO_SAT_EN          */ {NRF_GPIO_PIN_MAP(1, 15), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_GPS_RST         */ {NRF_GPIO_PIN_MAP(1, 14), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0D1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_MCU_DONE        */ {NRF_GPIO_PIN_MAP(1, 13), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_D0S1, NRF_GPIO_PIN_NOSENSE, {}},
		// GPIO_GPS_EXT_INT: nRF drives M10Q EXTINT to wake from PMREQ-backup
		// (deep-idle refactor 2026-05). See linkitv4_v1.0/bsp.cpp for rationale.
		/* GPIO_GPS_EXT_INT     */ {NRF_GPIO_PIN_MAP(1, 11), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {NRF_GPIOTE_POLARITY_HITOLO, NRF_GPIO_PIN_NOPULL, false, true, false}},
		/* GPIO_LED_GREEN       */ {NRF_GPIO_PIN_MAP(1, 10), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0D1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_LED_RED         */ {NRF_GPIO_PIN_MAP(1,  7), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0D1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_LED_BLUE        */ {NRF_GPIO_PIN_MAP(1,  4), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0D1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_GPS_PWR_EN      */ {NRF_GPIO_PIN_MAP(1,  6), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
	    /* GPIO_SMD_BUSY        */ {NRF_GPIO_PIN_MAP(1,  5), NRF_GPIO_PIN_DIR_INPUT,  NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},  // Unused — pulldown to prevent float
		/* GPIO_SMD_RFRESET     */ {NRF_GPIO_PIN_MAP(0, 30), NRF_GPIO_PIN_DIR_INPUT,  NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},  // Unused — pulldown to prevent float
		// REED_SW and the accelerometer INT1 (GPIO_INT1_AG) are GPIOTE inputs in IN (event) mode:
		// hi_accuracy = true, the 4th field of the last brace. Nordic documents that
		// mode as costing sleep current while any IN channel is on, ~20 to ~250 uA
		// by source; PORT (SENSE) events cost ~1 uA. Kept as is on purpose (audit
		// point 9, 2026-10-28): the reed is the only way back into a sealed tag, and
		// the real cost was never measured on a sound board. It is armed for the
		// whole mission (UW_DIVE_MODE_ENABLE, which pauses it underwater, is off by
		// default), so whatever it costs, it costs all day.
		// To move both to PORT later, all three, or an input dies:
		// - hi_accuracy = false on BOTH rows (one left in IN mode keeps the clock);
		// - NRFX_GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS and the legacy
		//   GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS >= 2 in sdk_config.h: at 1 the
		//   second nrfx_gpiote_in_init fails and that input never fires;
		// - no other pin left with a SENSE field (PORT events share one DETECT
		//   line; the bootloader arms P0.11): %SENSE must list none.
		// Measure first, bench build: %GPIOTE (registers, reed edges), %GPIOTEAB
		// (A/B of every IN channel for a PPK, USB unplugged).
		/* GPIO_REED_SW         */ {NRF_GPIO_PIN_MAP(1,  3), NRF_GPIO_PIN_DIR_INPUT,  NRF_GPIO_PIN_INPUT_CONNECT,    NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIO_PIN_PULLDOWN, false, true, false}},
		/* GPIO_INT1_AG         */ {NRF_GPIO_PIN_MAP(1,  2), NRF_GPIO_PIN_DIR_INPUT,  NRF_GPIO_PIN_INPUT_CONNECT,    NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {NRF_GPIOTE_POLARITY_LOTOHI, NRF_GPIO_PIN_NOPULL, false, true, false}},
		/* GPIO_INT2_AG         */ {NRF_GPIO_PIN_MAP(0, 13), NRF_GPIO_PIN_DIR_INPUT,  NRF_GPIO_PIN_INPUT_CONNECT,    NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_PRESS_INT       */ {NRF_GPIO_PIN_MAP(0,  3), NRF_GPIO_PIN_DIR_INPUT,  NRF_GPIO_PIN_INPUT_CONNECT,    NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_SMD_OPT         */ {NRF_GPIO_PIN_MAP(0, 20), NRF_GPIO_PIN_DIR_INPUT,  NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},  // Unused — pulldown to prevent float
		/* GPIO_SMD_OPT2        */ {NRF_GPIO_PIN_MAP(0, 16), NRF_GPIO_PIN_DIR_INPUT,  NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},  // Unused — pulldown to prevent float
		/* GPIO_SMD_VPA         */ {NRF_GPIO_PIN_MAP(1,  1), NRF_GPIO_PIN_DIR_INPUT,  NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},  // VPA regulator control - INPUT (HIGH-Z) when SMD ON, OUTPUT LOW when SMD OFF
    };

/////////////////////////////////// UART definitions ////////////////////////////////

    const UART_InitTypeDefAndInst UART_Inits[UART_TOTAL_NUMBER] =
    {
    #if NRFX_UARTE0_ENABLED
        {
            // UART_0 = GPS (M10Q UBX protocol)
            .uarte = NRFX_UARTE_INSTANCE(0),
            .config = {
                .pseltxd = NRF_GPIO_PIN_MAP(0, 26),
                .pselrxd = NRF_GPIO_PIN_MAP(0, 14),
                .pselcts = NRF_UARTE_PSEL_DISCONNECTED,
                .pselrts = NRF_UARTE_PSEL_DISCONNECTED,
                .p_context = NULL,
                .hwfc = NRF_UARTE_HWFC_DISABLED,
                .parity = NRF_UARTE_PARITY_EXCLUDED,
                .baudrate = NRF_UARTE_BAUDRATE_9600,
                .interrupt_priority = INTERRUPT_PRIORITY_UART_0,
            }
        },
    #endif
    #if NRFX_UARTE1_ENABLED
        {
            .uarte = NRFX_UARTE_INSTANCE(1),
            .config = {
                .pseltxd = NRF_GPIO_PIN_MAP(0, 11),
                .pselrxd = NRF_GPIO_PIN_MAP(1, 12),
                .pselcts = NRF_UARTE_PSEL_DISCONNECTED,
                .pselrts = NRF_UARTE_PSEL_DISCONNECTED,
                .p_context = NULL,
                .hwfc = NRF_UARTE_HWFC_DISABLED,
                .parity = NRF_UARTE_PARITY_EXCLUDED,
                .baudrate = NRF_UARTE_BAUDRATE_460800,
                .interrupt_priority = INTERRUPT_PRIORITY_UART_1,
            }
        }
    #endif
    };

    /////////////////////////////////// QSPI definitions ////////////////////////////////

    const QSPI_InitTypeDefAndInst QSPI_Inits[QSPI_TOTAL_NUMBER] =
    {
    #ifdef NRFX_QSPI_ENABLED
        {
            .config =
            {
                .xip_offset = 0,
                .pins = {
                    .sck_pin = NRF_GPIO_PIN_MAP(1, 0),
                    .csn_pin = NRF_GPIO_PIN_MAP(0, 24),
                    .io0_pin = NRF_GPIO_PIN_MAP(0, 21),
                    .io1_pin = NRF_GPIO_PIN_MAP(0, 23),
                    .io2_pin = NRF_GPIO_PIN_MAP(0, 22),
                    .io3_pin = NRF_GPIO_PIN_MAP(0, 19),
                },
                .prot_if = {
                    .readoc = NRF_QSPI_READOC_READ4IO,
                    .writeoc = NRF_QSPI_WRITEOC_PP4O,
                    .addrmode = NRF_QSPI_ADDRMODE_24BIT,
                    .dpmconfig = false,
                },
                .phy_if = {
                    .sck_delay = 1,
                    .dpmen = false,
                    .spi_mode = NRF_QSPI_MODE_0,
                    .sck_freq = NRF_QSPI_FREQ_32MDIV2,
                },
                .irq_priority = INTERRUPT_PRIORITY_QSPI_0
            }
        }
    #endif
    };

    ////////////////////////////////// RTC definitions /////////////////////////////////
    const RTC_InitTypeDefAndInst_t RTC_Inits[RTC_TOTAL_NUMBER] =
    {
#if APP_TIMER_V2_RTC0_ENABLED
		{
			.rtc = DRV_RTC_INSTANCE(0),
			.irq_priority = INTERRUPT_PRIORITY_RTC_0
		},
#endif
#if APP_TIMER_V2_RTC1_ENABLED
		{
			.rtc = DRV_RTC_INSTANCE(1),
			.irq_priority = INTERRUPT_PRIORITY_RTC_1
		},
#endif
#if APP_TIMER_V2_RTC2_ENABLED
		{
			.rtc = DRV_RTC_INSTANCE(2),
			.irq_priority = INTERRUPT_PRIORITY_RTC_2
		}
#endif
    };

    ////////////////////////////////// SPI definitions /////////////////////////////////
    const SPI_InitTypeDefAndInst_t SPI_Inits[SPI_TOTAL_NUMBER] =
    {
    #if NRFX_SPIM0_ENABLED
        {
            .spim = NRFX_SPIM_INSTANCE(0),
            .config = {
                .sck_pin  = NRFX_SPIM_PIN_NOT_USED,
                .mosi_pin = NRFX_SPIM_PIN_NOT_USED,
                .miso_pin = NRFX_SPIM_PIN_NOT_USED,
                .ss_pin   = NRFX_SPIM_PIN_NOT_USED,
                .ss_active_high = false,
                .irq_priority = INTERRUPT_PRIORITY_SPI_0,
                .orc = 0xFF,
                .frequency = NRF_SPIM_FREQ_4M,
                .mode = NRF_SPIM_MODE_0,
                .bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST
            }
        },
    #endif
    #if NRFX_SPIM1_ENABLED
        {
            .spim = NRFX_SPIM_INSTANCE(1),
            .config = {
                .sck_pin  = NRFX_SPIM_PIN_NOT_USED,
                .mosi_pin = NRFX_SPIM_PIN_NOT_USED,
                .miso_pin = NRFX_SPIM_PIN_NOT_USED,
                .ss_pin   = NRFX_SPIM_PIN_NOT_USED,
                .ss_active_high = false,
                .irq_priority = INTERRUPT_PRIORITY_SPI_1,
                .orc = 0xFF,
                .frequency = NRF_SPIM_FREQ_4M,
                .mode = NRF_SPIM_MODE_0,
                .bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST
            }
        },
    #endif
    #if NRFX_SPIM2_ENABLED
        {
            .spim = NRFX_SPIM_INSTANCE(2),
            .config = {
                .sck_pin  = NRF_GPIO_PIN_MAP(0, 8),
                .mosi_pin = NRF_GPIO_PIN_MAP(0, 6),
                .miso_pin = NRF_GPIO_PIN_MAP(0, 7),
                .ss_pin   = NRF_GPIO_PIN_MAP(0, 5),
                .ss_active_high = false,
                .irq_priority = INTERRUPT_PRIORITY_SPI_2,
                .orc = 0xFF,
                .frequency = NRF_SPIM_FREQ_2M,
                .mode = NRF_SPIM_MODE_0,
                .bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST
            }
        },
    #endif
    #if NRFX_SPIM3_ENABLED
        {
            .spim = NRFX_SPIM_INSTANCE(3),
            .config = {
                .sck_pin  = NRF_GPIO_PIN_MAP(0, 19),
                .mosi_pin = NRF_GPIO_PIN_MAP(0, 21),
                .miso_pin = NRF_GPIO_PIN_MAP(0, 23),
                .ss_pin   = NRF_GPIO_PIN_MAP(0, 24),
                .ss_active_high = false,
                .irq_priority = INTERRUPT_PRIORITY_SPI_3,
                .orc = 0xFF,
                .frequency = NRF_SPIM_FREQ_32M,
                .mode = NRF_SPIM_MODE_0,
                .bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST
            }
        }
    #endif
    };

    ////////////////////////////////// ADC definitions /////////////////////////////////
    const ADC_InitTypeDefAndInst_t ADC_Inits =
    {
        .config = {
            .resolution = NRF_SAADC_RESOLUTION_14BIT,
            .oversample = NRF_SAADC_OVERSAMPLE_DISABLED,
            .interrupt_priority = INTERRUPT_PRIORITY_ADC,
            .low_power_mode = false
        },
        .channel_config = {
            {
                // ADC_CHANNEL_0 - Thermistor
                .resistor_p = NRF_SAADC_RESISTOR_DISABLED,
                .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
                .gain = NRF_SAADC_GAIN1_6,
                .reference = NRF_SAADC_REFERENCE_INTERNAL,
                .acq_time = NRF_SAADC_ACQTIME_20US,
                .mode = NRF_SAADC_MODE_SINGLE_ENDED,
                .burst = NRF_SAADC_BURST_DISABLED,
                .pin_p = NRF_SAADC_INPUT_AIN4,
                .pin_n = NRF_SAADC_INPUT_DISABLED
            }
        }
    };

    ///////////////////////////////// I2C definitions /////////////////////////////////
    const I2C_InitTypeDefAndInst_t I2C_Inits[I2C_TOTAL_NUMBER] =
    {
    #if NRFX_TWIM0_ENABLED
        {
            .twim = NRFX_TWIM_INSTANCE(0),
			.twim_config =
            {
                .scl = NRF_GPIO_PIN_MAP(0, 15),
                .sda = NRF_GPIO_PIN_MAP(0, 27),
				.frequency = NRF_TWIM_FREQ_400K,
                .interrupt_priority = INTERRUPT_PRIORITY_I2C_0,
                .hold_bus_uninit = false,
            }
        },
    #endif
    #if NRFX_TWIM1_ENABLED
        {
            .twim = NRFX_TWIM_INSTANCE(1),
			.twim_config =
            {
                .scl = NRF_GPIO_PIN_MAP(0, 15),
                .sda = NRF_GPIO_PIN_MAP(0, 27),
                .frequency = NRF_TWIM_FREQ_400K,
                .interrupt_priority = INTERRUPT_PRIORITY_I2C_1,
                .hold_bus_uninit = false,
            }
        }
    #endif
    };

    ///////////////////////////////// WDT definitions /////////////////////////////////
    const WDT_InitTypeDefAndInst_t WDT_Inits[WDT_TOTAL_NUMBER] =
    {
#if NRFX_WDT_ENABLED
    	{
			.config =
			{
				.behaviour = NRF_WDT_BEHAVIOUR_RUN_SLEEP_HALT,
				.reload_value = 15 * 60 * 1000,   // 15 minutes
				.interrupt_priority = INTERRUPT_PRIORITY_WDT
			}
    	}
#endif
    };

    // Define UARTE async instance using library supplied macro
    NRF_LIBUARTE_ASYNC_DEFINE(async_uarte_0, 0, 1,\
            NRF_LIBUARTE_PERIPHERAL_NOT_USED, 2,\
            255, 4);

    const UARTAsync_InitTypeDefAndInst_t UARTAsync_Inits[1] =
    {
        {
            .uart = &async_uarte_0,
            .config = {
                .rx_pin = NRF_GPIO_PIN_MAP(1, 8),
                .tx_pin = NRF_GPIO_PIN_MAP(1, 9),
                .cts_pin = NRF_UARTE_PSEL_DISCONNECTED,
                .rts_pin = NRF_UARTE_PSEL_DISCONNECTED,
                .timeout_us = 250,
                .flush_on_timeout = true,
                .hwfc = NRF_UARTE_HWFC_DISABLED,
                .parity = NRF_UARTE_PARITY_EXCLUDED,
                .baudrate = NRF_UARTE_BAUDRATE_460800,
                .pullup_rx = false,
                .int_prio = INTERRUPT_PRIORITY_UART_0,
            }
        }
    };
}
