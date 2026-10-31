#include "bsp.hpp"

namespace BSP
{
	///////////////////////////////// GPIO definitions ////////////////////////////////
	const GPIO_InitTypeDefAndInst_t GPIO_Inits[GPIO_TOTAL_NUMBER] =
	{
		// pin number, direction, input, pull, drive sense
		//
		// A push-pull output carries NO pull. While it is driven, the pin sets the
		// level, so the resistor either does nothing (same level) or conducts VDD
		// over ~13 kOhm (opposite level). Measured on a LinkIt 2026-10-27 with the
		// %PULLAB bench probe: the pull-down of a HIGH GPS_PWR_EN drew current for
		// the whole GNSS deep-idle window, ~0.12 mA seen by a PPK sharing the supply
		// with USB, ~0.25 mA expected from 3.3 V / 13 kOhm. Nor does it hold anything
		// through a reset, which returns PIN_CNF to a disconnected input with no
		// pull: a level that must survive a reset needs a resistor on the board.
		// tools/check_gpio_output_pulls.py enforces this in CI.
		/* GPIO_POWER_CONTROL   */ {NRF_GPIO_PIN_MAP(0,  4), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
        /* GPIO_VSYS_SEL        */ {NRF_GPIO_PIN_MAP(1, 14), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0D1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_DEBUG           */ {NRF_GPIO_PIN_MAP(0, 11), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_SWS             */ {NRF_GPIO_PIN_MAP(0,  2), NRF_GPIO_PIN_DIR_INPUT,  NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_SLOW_SWS_SEND   */ {NRF_GPIO_PIN_MAP(0, 10), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_ACC_PWR         */ {NRF_GPIO_PIN_MAP(0, 25), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
        /* GPIO_ACC_INT1        */ {NRF_GPIO_PIN_MAP(1,  2), NRF_GPIO_PIN_DIR_INPUT,  NRF_GPIO_PIN_INPUT_CONNECT,    NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {NRF_GPIOTE_POLARITY_LOTOHI, NRF_GPIO_PIN_NOPULL, false, true, false}},
		// SAT_RESET: open-drain output (S0D1). nRF can pull NRST LOW to reset
		// STM32WL but never drives HIGH — external pull-up handles release. A
		// SWD probe wired to NRST can still assert reset at any time (both ends
		// open-drain → no contention), so STM32WL firmware flashing is unaffected.
		/* GPIO_SAT_RESET       */ {NRF_GPIO_PIN_MAP(0, 31), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0D1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_SAT_EN          */ {NRF_GPIO_PIN_MAP(1, 15), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
        // KIM2 external wake-up. It used to carry a pull-down, meant to keep the
        // module asleep while the nRF is in reset or System OFF. It did neither:
        // a reset clears the pull with the rest of PIN_CNF, and System OFF keeps
        // the driven level. It only ever conducted, while the line was HIGH.
        /* GPIO_SAT_WKUP        */ {NRF_GPIO_PIN_MAP(0, 30), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
        /* GPIO_KIM_PWR_ON      */ {NRF_GPIO_PIN_MAP(0,  5), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
        // KIM_INT (module pin 5) -- an output of the module into nRF P0.29, so an
        // INPUT here. Pull-down: the module releases it when powered off, and a
        // floating input both burns current and picks up noise. Unused so far.
        /* GPIO_SAT_INT         */ {NRF_GPIO_PIN_MAP(0, 29), NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_GPS_PWR_EN      */ {NRF_GPIO_PIN_MAP(1,  6), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_GPS_RST         */ {NRF_GPIO_PIN_MAP(1, 13), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0D1, NRF_GPIO_PIN_NOSENSE, {}},
		// GPIO_GPS_EXT_INT: nRF drives the M10Q EXTINT input pin to wake it from
		// PMREQ-backup deep-sleep (2026-05 deep-idle refactor). Was DIR_INPUT
		// historically (unused defensive config); now OUTPUT so we can pulse.
		// Driver releases to high-Z between pulses to avoid leakage into a
		// powered-off M10Q. Sense fields kept for legacy compat (ignored when
		// configured as OUTPUT).
		/* GPIO_GPS_EXT_INT     */ {NRF_GPIO_PIN_MAP(1, 11), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {NRF_GPIOTE_POLARITY_HITOLO, NRF_GPIO_PIN_NOPULL, false, true, false}},
        // GPOI_GPS_PPS: nothing in the whole firmware consumes it (checked
        // 2026-08). The input buffer was CONNECT with no pull, on a pin the M10Q
        // only drives while its rail is powered -- that is, a floating input with
        // a connected buffer the rest of the time, so permanent shoot-through
        // current on a sealed device. It was the only GPS pin that was not
        // INPUT_DISCONNECT. With no consumer, disconnecting it is safe whether
        // the track is routed or not.
        /* GPOI_GPS_PPS         */ {NRF_GPIO_PIN_MAP(1,  5), NRF_GPIO_PIN_DIR_INPUT,  NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_LED_GREEN       */ {NRF_GPIO_PIN_MAP(1, 10), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_LED_RED         */ {NRF_GPIO_PIN_MAP(1,  7), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_LED_BLUE        */ {NRF_GPIO_PIN_MAP(1,  4), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		// REED_SW and the accelerometer INT1 are GPIOTE inputs in IN (event) mode:
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
        /* GPIO_ADC_EN          */ {NRF_GPIO_PIN_MAP(0, 12), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
        /* GPIO_ADC_ALERT_READY */ {NRF_GPIO_PIN_MAP(1,  1), NRF_GPIO_PIN_DIR_INPUT,  NRF_GPIO_PIN_INPUT_CONNECT,    NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
        /* GPIO_BAT_READ_EN     */ {NRF_GPIO_PIN_MAP(0, 28), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_EXT_GPIO1       */ {NRF_GPIO_PIN_MAP(0,  6), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_EXT_GPIO2       */ {NRF_GPIO_PIN_MAP(0,  7), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_EXT_GPIO3       */ {NRF_GPIO_PIN_MAP(0,  8), NRF_GPIO_PIN_DIR_INPUT,  NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_EXT_GPIO4       */ {NRF_GPIO_PIN_MAP(0,  9), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},  // CAM_PWR_EN (CAM_CONTROL=BUTTON) / SMD SPI CS
		/* GPIO_EXT_GPIO5       */ {NRF_GPIO_PIN_MAP(0, 16), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},  // CAM_PWR_BUTT / CAM_PWR_EN (CAM_CONTROL=UART) / BUZZER
    };

/////////////////////////////////// UART definitions ////////////////////////////////

    // Supported baudrates:
    // NRF_UARTE_BAUDRATE_1200
    // NRF_UARTE_BAUDRATE_2400
    // NRF_UARTE_BAUDRATE_4800
    // NRF_UARTE_BAUDRATE_9600
    // NRF_UARTE_BAUDRATE_14400
    // NRF_UARTE_BAUDRATE_19200
    // NRF_UARTE_BAUDRATE_28800
    // NRF_UARTE_BAUDRATE_31250
    // NRF_UARTE_BAUDRATE_38400
    // NRF_UARTE_BAUDRATE_56000
    // NRF_UARTE_BAUDRATE_57600
    // NRF_UARTE_BAUDRATE_76800
    // NRF_UARTE_BAUDRATE_115200
    // NRF_UARTE_BAUDRATE_230400
    // NRF_UARTE_BAUDRATE_250000
    // NRF_UARTE_BAUDRATE_460800
    // NRF_UARTE_BAUDRATE_921600
    // NRF_UARTE_BAUDRATE_1000000

    const UART_InitTypeDefAndInst UART_Inits[UART_TOTAL_NUMBER] =
    {
    #if NRFX_UARTE0_ENABLED
        {
            .uarte = NRFX_UARTE_INSTANCE(0),
            .config = {
                .pseltxd = NRF_GPIO_PIN_MAP(1, 9),
                .pselrxd = NRF_GPIO_PIN_MAP(1, 8),
                .pselcts = NRF_UARTE_PSEL_DISCONNECTED,
                .pselrts = NRF_UARTE_PSEL_DISCONNECTED,
                .p_context = NULL, // Context passed to interrupt handler
                .hwfc = NRF_UARTE_HWFC_DISABLED,
                .parity = NRF_UARTE_PARITY_EXCLUDED,
                .baudrate = NRF_UARTE_BAUDRATE_9600, // See table above
                .interrupt_priority = INTERRUPT_PRIORITY_UART_0,
            }
        },
    #endif
    #if NRFX_UARTE1_ENABLED
        {
            .uarte = NRFX_UARTE_INSTANCE(1),
            .config = {
                .pseltxd = NRF_GPIO_PIN_MAP(0, 11),
                .pselrxd = NRF_GPIO_PIN_MAP(0, 14),
                .pselcts = NRF_UARTE_PSEL_DISCONNECTED,
                .pselrts = NRF_UARTE_PSEL_DISCONNECTED,
                .p_context = NULL, // Context passed to interrupt handler
                .hwfc = NRF_UARTE_HWFC_DISABLED,
                .parity = NRF_UARTE_PARITY_EXCLUDED,
                .baudrate = NRF_UARTE_BAUDRATE_460800, // See table above
                .interrupt_priority = INTERRUPT_PRIORITY_UART_1,
            }
        }
    #endif
    };

    /////////////////////////////////// QSPI definitions ////////////////////////////////

    // Supported frequencies:
    // NRF_QSPI_FREQ_32MDIV1    32MHz/1
    // NRF_QSPI_FREQ_32MDIV2    32MHz/2
    // NRF_QSPI_FREQ_32MDIV3    32MHz/3
    // NRF_QSPI_FREQ_32MDIV4    32MHz/4
    // NRF_QSPI_FREQ_32MDIV5    32MHz/5
    // NRF_QSPI_FREQ_32MDIV6    32MHz/6
    // NRF_QSPI_FREQ_32MDIV7    32MHz/7
    // NRF_QSPI_FREQ_32MDIV8    32MHz/8
    // NRF_QSPI_FREQ_32MDIV9    32MHz/9
    // NRF_QSPI_FREQ_32MDIV10   32MHz/10
    // NRF_QSPI_FREQ_32MDIV11   32MHz/11
    // NRF_QSPI_FREQ_32MDIV12   32MHz/12
    // NRF_QSPI_FREQ_32MDIV13   32MHz/13
    // NRF_QSPI_FREQ_32MDIV14   32MHz/14
    // NRF_QSPI_FREQ_32MDIV15   32MHz/15
    // NRF_QSPI_FREQ_32MDIV16   32MHz/16

    const QSPI_InitTypeDefAndInst QSPI_Inits[QSPI_TOTAL_NUMBER] =
    {
    #ifdef NRFX_QSPI_ENABLED
        {
            .config = 
            {
                .xip_offset = 0, // Address offset in the external memory for Execute in Place operation
                .pins = {
                    .sck_pin = NRF_GPIO_PIN_MAP(0, 19),
                    .csn_pin = NRF_GPIO_PIN_MAP(0, 24),
                    .io0_pin = NRF_GPIO_PIN_MAP(0, 21),
                    .io1_pin = NRF_GPIO_PIN_MAP(0, 23),
                    .io2_pin = NRF_GPIO_PIN_MAP(0, 22),
                    .io3_pin = NRF_GPIO_PIN_MAP(1,  0),
                },
                .prot_if = {
                    .readoc = NRF_QSPI_READOC_READ4IO, // Number of data lines and opcode used for reading
                    .writeoc = NRF_QSPI_WRITEOC_PP4O,  // Number of data lines and opcode used for writing
                    .addrmode = NRF_QSPI_ADDRMODE_24BIT,
                    .dpmconfig = false, // Deep power-down mode enable
                },
                .phy_if = {
                    .sck_delay = 1, // SCK delay in units of 62.5 ns  <0-255>
                    .dpmen = false, // Deep power-down mode enable
                    .spi_mode = NRF_QSPI_MODE_0,
                    .sck_freq = NRF_QSPI_FREQ_32MDIV2, // See table above
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
                .orc = 0xFF, // Over-run character
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
                .orc = 0xFF, // Over-run character
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
                .ss_pin   = NRF_GPIO_PIN_MAP(0, 9),  // NFC1 pin used as SPI CS for SMD satellite
                .ss_active_high = false,
                .irq_priority = INTERRUPT_PRIORITY_SPI_2,
                .orc = 0xFF, // Over-run character
                .frequency = NRF_SPIM_FREQ_2M,  // Application link; the bootloader DFU runs at SMDSAT_DFU_SPI_FREQUENCY
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
                .orc = 0xFF, // Over-run character
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
                // ADC_CHANNEL_0: Battery voltage on AIN1 (P0.03)
                .resistor_p = NRF_SAADC_RESISTOR_DISABLED,
                .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
// Keep in step with ADC_GAIN in bsp.hpp — same flag, same reason.
#if defined(BATTERY_ADC_HIGH_RANGE) && (BATTERY_ADC_HIGH_RANGE == 1)
                .gain = NRF_SAADC_GAIN1_6,
#else
                .gain = NRF_SAADC_GAIN1_5,
#endif
                .reference = NRF_SAADC_REFERENCE_INTERNAL,
                .acq_time = NRF_SAADC_ACQTIME_40US,
                .mode = NRF_SAADC_MODE_SINGLE_ENDED,
                .burst = NRF_SAADC_BURST_DISABLED,
                .pin_p = NRF_SAADC_INPUT_AIN1,
                .pin_n = NRF_SAADC_INPUT_DISABLED
            },
            {
                // ADC_CHANNEL_1: SWS analog on AIN0 (P0.02)
                // Gain 1/6: V_max = 0.6V/0.167 = 3.6V (full VDD range)
                .resistor_p = NRF_SAADC_RESISTOR_DISABLED,
                .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
                .gain = NRF_SAADC_GAIN1_6,
                .reference = NRF_SAADC_REFERENCE_INTERNAL,
                .acq_time = NRF_SAADC_ACQTIME_40US,
                .mode = NRF_SAADC_MODE_SINGLE_ENDED,
                .burst = NRF_SAADC_BURST_DISABLED,
                .pin_p = NRF_SAADC_INPUT_AIN0,
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
                .scl = NRF_GPIO_PIN_MAP(0, 27),
				.sda = NRF_GPIO_PIN_MAP(0, 15),
				.frequency = NRF_TWIM_FREQ_250K,
                .interrupt_priority = INTERRUPT_PRIORITY_I2C_0,
                .hold_bus_uninit = 0, // Hold pull up state on gpio pins after uninit <0 = Disabled, 1 = Enabled>
            }
        },
    #endif
    #if NRFX_TWIM1_ENABLED
        {
            .twim = NRFX_TWIM_INSTANCE(1),
			.twim_config =
            {
                .scl = NRF_GPIO_PIN_MAP(0, 13),
                .sda = NRF_GPIO_PIN_MAP(0, 17),
                .frequency = NRF_TWIM_FREQ_400K,
                .interrupt_priority = INTERRUPT_PRIORITY_I2C_1,
                .hold_bus_uninit = 0, // Hold pull up state on gpio pins after uninit <0 = Disabled, 1 = Enabled>
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

    NRF_LIBUARTE_ASYNC_DEFINE(async_uarte_1, 1, 3,\
            NRF_LIBUARTE_PERIPHERAL_NOT_USED, 4,\
            255, 4);

    const UARTAsync_InitTypeDefAndInst_t UARTAsync_Inits[2] =
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
        },
#if defined(CAM_CONTROL_UART) && (CAM_CONTROL_UART == 1)
        // RunCam camera, RunCam Device Protocol (-DCAM_CONTROL=UART). CMake allows
        // it on an SMD SPI build only, where the satellite slot no longer needs
        // this UART. nRF TX P0.14 -> camera RX, nRF RX P0.26 <- camera TX: the
        // orientation of the SMD UART it replaces.
        //
        // HARDWARE: the SMD module must no longer be on these two lines. Its
        // LPUART1 debug TX lands on P0.26 (bench 2026-09-02): left connected it
        // drives against the camera's TX, and the camera's idle-high TX feeds an
        // SMD whose rail is cut through its ESD clamp. %SATLOG with the camera
        // off tells whether something still talks on P0.26.
        {
            .uart = &async_uarte_1,
            .config = {
                .rx_pin = NRF_GPIO_PIN_MAP(0, 26),
                .tx_pin = NRF_GPIO_PIN_MAP(0, 14),
                .cts_pin = NRF_UARTE_PSEL_DISCONNECTED,
                .rts_pin = NRF_UARTE_PSEL_DISCONNECTED,
                // ~6 byte times at 115200: a 5-byte answer is handed over 0.5 ms
                // after its last byte instead of waiting for a full buffer.
                .timeout_us = 500,
                .flush_on_timeout = true,
                .hwfc = NRF_UARTE_HWFC_DISABLED,
                .parity = NRF_UARTE_PARITY_EXCLUDED,
                .baudrate = NRF_UARTE_BAUDRATE_115200,  // fixed by the protocol
                // Only while the link is up, which is from one second after the
                // rail to just before it is cut (RunCamUartDevice): it never feeds
                // an unpowered camera, and it holds the line idle while the
                // camera's own TX is not driven yet during its boot.
                .pullup_rx = true,
                .int_prio = INTERRUPT_PRIORITY_UART_1,
            }
        }
#else
        {
            .uart = &async_uarte_1,
            .config = {
#if (defined(LORA_RAK3172) && (LORA_RAK3172 == 1)) || (defined(SMD_UART) && (SMD_UART == 1))
                // LinkIt V4 LoRa: PCB routes nRF P0.14 → RAK3172 UART_RX (PA10)
                // and nRF P0.26 ← RAK3172 UART_TX (PA9). In Nordic async config,
                // rx_pin = nRF RX input, tx_pin = nRF TX output.
                //
                // The SMD (STM32WL) shares this orientation: the satellite slot
                // is wired like the LoRa one. Proven on the bench 2026-09-02 --
                // with the KIM2 orientation below, every AT command timed out
                // because we drove P0.26 (the module's own TX output) and
                // listened on P0.14 (the module's RX input, silent by
                // construction). Swapping them made PING, credentials, KMAC, LPM
                // and TX all work first try. This is also why %SATLOG found the
                // module's LPUART1 debug log silent on P0.14: wrong line.
                .rx_pin = NRF_GPIO_PIN_MAP(0, 26),
                .tx_pin = NRF_GPIO_PIN_MAP(0, 14),
#else
                // KIM2 on the same slot -- opposite orientation to the LoRa and
                // SMD modules above. Do not "unify" these: the two are different
                // modules and each is verified on hardware as it stands.
                .rx_pin = NRF_GPIO_PIN_MAP(0, 14),
                .tx_pin = NRF_GPIO_PIN_MAP(0, 26),
#endif
                .cts_pin = NRF_UARTE_PSEL_DISCONNECTED,
                .rts_pin = NRF_UARTE_PSEL_DISCONNECTED,
                .timeout_us = 1250,
                .flush_on_timeout = true,
                .hwfc = NRF_UARTE_HWFC_DISABLED,
                .parity = NRF_UARTE_PARITY_EXCLUDED,
                .baudrate = NRF_UARTE_BAUDRATE_9600,
                // Pull-up on the receive line. The satellite module releases its
                // TX pin as soon as it drops into low power (KIM2 in STANDBY
                // during a BLIND burst, SMD in STANDBY/SHUTDOWN): with no pull the
                // line FLOATS, the UART idle state is no longer guaranteed, and
                // the nRF receiver raises framing errors then fails to resync when
                // the module wakes. Measured on the bench 2026-08-25: a type=04
                // error at every wake-up, then total silence -- the end-of-burst
                // +TX= never arrived. The internal pull-up (~13 kOhm) holds the
                // line idle while the module sleeps. nrf_libuarte_drv.c:466.
                .pullup_rx = true,
                .int_prio = INTERRUPT_PRIORITY_UART_1,
            }
        }
#endif
    };
}
