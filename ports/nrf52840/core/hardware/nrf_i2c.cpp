/**
 * @file nrf_i2c.cpp
 * @brief nRF52840 TWIM driver — async transfers, timeout, multi-level bus recovery.
 */

#include "nrf_i2c.hpp"
#include "nrf_peripheral_power.hpp"
#include "nrfx_twim.h"
#include "nrfx_twi_twim.h"
#include "nrf_twim.h"
#include "error.hpp"
#include "debug.hpp"
#include "gpio.hpp"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "pmu.hpp"
#include "timer.hpp"

extern Timer *system_timer;

// ═══════════════════════════════════════════════════════
//  TWIM event handlers (C-style callbacks for nrfx)
// ═══════════════════════════════════════════════════════

static bool is_error_event(nrfx_twim_evt_type_t type) {
	return type == NRFX_TWIM_EVT_ADDRESS_NACK ||
	       type == NRFX_TWIM_EVT_DATA_NACK    ||
	       type == NRFX_TWIM_EVT_OVERRUN       ||
	       type == NRFX_TWIM_EVT_BUS_ERROR;
}

#if NRFX_TWIM0_ENABLED
static void twim0_event_handler(nrfx_twim_evt_t const *p_event, void *) {
	NrfI2C::event_handler(0, is_error_event(p_event->type));
}
#endif

#if NRFX_TWIM1_ENABLED
static void twim1_event_handler(nrfx_twim_evt_t const *p_event, void *) {
	NrfI2C::event_handler(1, is_error_event(p_event->type));
}
#endif

static nrfx_twim_evt_handler_t get_event_handler(uint8_t bus) {
	switch (bus) {
#if NRFX_TWIM0_ENABLED
	case 0: return twim0_event_handler;
#endif
#if NRFX_TWIM1_ENABLED
	case 1: return twim1_event_handler;
#endif
	default: return nullptr;
	}
}


// ═══════════════════════════════════════════════════════
//  Bus recovery (3 levels: stuck check → clock stretch → full reset)
// ═══════════════════════════════════════════════════════

/// @brief Helper — read SCL/SDA pin pair for a bus.
static void get_bus_pins(uint8_t bus, uint32_t &scl, uint32_t &sda) {
	scl = BSP::I2C_Inits[bus].twim_config.scl;
	sda = BSP::I2C_Inits[bus].twim_config.sda;
}

/// @brief Generate an I2C STOP condition on the bus via bit-bang.
static void bitbang_stop(uint32_t scl, uint32_t sda) {
	nrf_gpio_cfg_output(sda);
	nrf_gpio_pin_clear(sda);
	PMU::delay_us(5);
	nrf_gpio_pin_set(scl);
	PMU::delay_us(5);
	nrf_gpio_pin_set(sda);
}

bool NrfI2C::is_bus_stuck(uint8_t bus) {
	if (bus >= BSP::I2C_TOTAL_NUMBER) return false;

	uint32_t scl, sda;
	get_bus_pins(bus, scl, sda);

	nrf_gpio_cfg_input(scl, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(sda, NRF_GPIO_PIN_PULLUP);
	PMU::delay_us(50);

	bool scl_low = (nrf_gpio_pin_read(scl) == 0);
	bool sda_low = (nrf_gpio_pin_read(sda) == 0);

	if (scl_low || sda_low) {
		DEBUG_WARN("I2C bus %u stuck: SCL=%s SDA=%s",
			bus, scl_low ? "LOW" : "HIGH", sda_low ? "LOW" : "HIGH");
		return true;
	}
	return false;
}

bool NrfI2C::clock_stretch_recovery(uint8_t bus) {
	if (bus >= BSP::I2C_TOTAL_NUMBER) return false;

	uint32_t scl, sda;
	get_bus_pins(bus, scl, sda);

	nrf_gpio_cfg_output(scl);
	nrf_gpio_cfg_input(sda, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_pin_set(scl);
	PMU::delay_us(10);

	// Clock pulses to release SDA
	for (unsigned int cycle = 0; cycle < I2C_BUS_RECOVERY_CYCLES * 2; cycle++) {
		if (nrf_gpio_pin_read(sda) == 1) break;
		nrf_gpio_pin_clear(scl);
		PMU::delay_us(5);
		nrf_gpio_pin_set(scl);
		PMU::delay_us(5);
	}

	bitbang_stop(scl, sda);
	PMU::delay_us(10);

	nrf_gpio_cfg_input(sda, NRF_GPIO_PIN_PULLUP);
	PMU::delay_us(10);
	return (nrf_gpio_pin_read(sda) == 1);
}

bool NrfI2C::reinit_bus(uint8_t bus) {
	nrfx_twim_evt_handler_t handler = get_event_handler(bus);
	nrfx_err_t err = nrfx_twim_init(&BSP::I2C_Inits[bus].twim,
	                                 &BSP::I2C_Inits[bus].twim_config,
	                                 handler, nullptr);
	if (err != NRFX_SUCCESS) {
		DEBUG_ERROR("I2C bus %u reinit failed (0x%08X)", bus, err);
		m_is_enabled[bus] = false;
		return false;
	}
	nrfx_twim_enable(&BSP::I2C_Inits[bus].twim);
	m_is_enabled[bus] = true;
	return true;
}

/// @brief Pin-level bus reset — drive lines high, START, clock out stuck bits, STOP.
/// Bit-bang only: safe to call before the TWIM instance is initialised (boot path).
static void bitbang_bus_reset(uint32_t scl, uint32_t sda) {
	// Drive both lines high, then generate START
	nrf_gpio_cfg_output(scl);
	nrf_gpio_cfg_output(sda);
	nrf_gpio_pin_set(scl);
	nrf_gpio_pin_set(sda);
	PMU::delay_ms(1);

	// START condition
	nrf_gpio_pin_clear(sda);
	PMU::delay_us(10);

	// Clock out stuck data
	nrf_gpio_cfg_input(sda, NRF_GPIO_PIN_PULLUP);
	for (unsigned int i = 0; i < I2C_BUS_RECOVERY_CYCLES + 3; i++) {
		nrf_gpio_pin_clear(scl);
		PMU::delay_us(5);
		nrf_gpio_pin_set(scl);
		PMU::delay_us(5);
	}

	// STOP condition
	bitbang_stop(scl, sda);
	PMU::delay_ms(1);
}

bool NrfI2C::full_bus_reset(uint8_t bus) {
	if (bus >= BSP::I2C_TOTAL_NUMBER) return false;

	DEBUG_WARN("I2C bus %u full reset", bus);

	uint32_t scl, sda;
	get_bus_pins(bus, scl, sda);

	nrfx_twim_disable(&BSP::I2C_Inits[bus].twim);
	nrfx_twim_uninit(&BSP::I2C_Inits[bus].twim);

	bitbang_bus_reset(scl, sda);

	// Verify bus is free
	nrf_gpio_cfg_input(scl, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(sda, NRF_GPIO_PIN_PULLUP);
	PMU::delay_us(50);

	if (nrf_gpio_pin_read(scl) == 0 || nrf_gpio_pin_read(sda) == 0) {
		DEBUG_ERROR("I2C bus %u still stuck after full reset", bus);
		m_is_enabled[bus] = false;
		return false;
	}

	if (!reinit_bus(bus)) return false;

	DEBUG_INFO("I2C bus %u recovered (full reset)", bus);
	return true;
}

bool NrfI2C::recover_bus(uint8_t bus) {
	if (bus >= BSP::I2C_TOTAL_NUMBER) return false;

	DEBUG_WARN("I2C bus %u recovery", bus);
	m_stats[bus].bus_recoveries++;

	nrfx_twim_disable(&BSP::I2C_Inits[bus].twim);
	nrfx_twim_uninit(&BSP::I2C_Inits[bus].twim);

	for (unsigned int attempt = 0; attempt < I2C_RECOVERY_MAX_ATTEMPTS; attempt++) {
		if (clock_stretch_recovery(bus) && reinit_bus(bus)) {
			DEBUG_INFO("I2C bus %u recovered (attempt %u)", bus, attempt + 1);
			return true;
		}
		PMU::delay_ms(5);
	}

	return full_bus_reset(bus);
}


// ═══════════════════════════════════════════════════════
//  Initialisation
// ═══════════════════════════════════════════════════════

void NrfI2C::init(void) {
	for (unsigned int i = 0; i < BSP::I2C_TOTAL_NUMBER; i++) {
		if (m_is_enabled[i]) continue;

		// Pre-configure I2C pins with pull-ups before bus-stuck check
		nrf_gpio_cfg(BSP::I2C_Inits[i].twim_config.scl, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_CONNECT,
		             NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_S0D1, NRF_GPIO_PIN_NOSENSE);
		nrf_gpio_cfg(BSP::I2C_Inits[i].twim_config.sda, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_CONNECT,
		             NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_S0D1, NRF_GPIO_PIN_NOSENSE);
		PMU::delay_us(100);

		if (is_bus_stuck(i)) {
			DEBUG_WARN("I2C bus %u stuck at init | attempting recovery", i);
			// Match the runtime recover_bus() strength: several clock-stretch
			// attempts instead of the single one this path used to get.
			bool freed = false;
			for (unsigned int attempt = 0; attempt < I2C_RECOVERY_MAX_ATTEMPTS && !freed; attempt++) {
				freed = clock_stretch_recovery(i);
				if (!freed)
					PMU::delay_ms(5);
			}
			if (!freed) {
				// Last chance before booting degraded: pin-level full reset
				// (START + clock-out + STOP). Runtime recovery escalates to
				// full_bus_reset(), but that needs an initialised TWIM — this
				// boot path never had one, so a slave that only resyncs on a
				// START (e.g. an STC3117 on VBAT left armed mid-transfer by a
				// power cut) was previously never offered one at boot.
				uint32_t scl, sda;
				get_bus_pins(i, scl, sda);
				bitbang_bus_reset(scl, sda);
				freed = !is_bus_stuck(i);
			}
			if (!freed) {
				// Unrecoverable (e.g. STC3117 on VBAT holding SCL low — only a full
				// board power cycle clears that). Disable the bus and boot degraded
				// rather than freezing: consumers fast-fail and the gauge falls back
				// to its conservative defaults.
				DEBUG_ERROR("I2C bus %u recovery failed | booting degraded (bus disabled)", i);
				m_is_enabled[i] = false;
				continue;
			}
		}

		m_transfer_done[i] = false;
		m_transfer_error[i] = false;

		if (!reinit_bus(i)) continue;

		m_stats[i] = {};
		DEBUG_TRACE("I2C bus %u OK", i);
	}
}

void NrfI2C::uninit(void) {
	for (unsigned int i = 0; i < BSP::I2C_TOTAL_NUMBER; i++) {
		if (m_is_enabled[i]) {
			nrfx_twim_disable(&BSP::I2C_Inits[i].twim);
			nrfx_twim_uninit(&BSP::I2C_Inits[i].twim);
			// Errata 89: toggle POWER register to prevent 400 µA idle current leak
			nrf_peripheral_power_reset(reinterpret_cast<uint32_t>(BSP::I2C_Inits[i].twim.p_twim));
			m_is_enabled[i] = false;
		}
	}
}

void NrfI2C::disable(uint8_t bus) {
	if (bus < BSP::I2C_TOTAL_NUMBER && m_is_enabled[bus]) {
		nrfx_twim_disable(&BSP::I2C_Inits[bus].twim);
		nrfx_twim_uninit(&BSP::I2C_Inits[bus].twim);
		// Errata 89: toggle POWER register to prevent 400 µA idle current leak
		nrf_peripheral_power_reset(reinterpret_cast<uint32_t>(BSP::I2C_Inits[bus].twim.p_twim));
		m_is_enabled[bus] = false;
	}
}


// ═══════════════════════════════════════════════════════
//  Transfer engine (async with timeout + retry)
// ═══════════════════════════════════════════════════════

void NrfI2C::event_handler(uint8_t bus, bool error) {
	if (bus >= BSP::I2C_TOTAL_NUMBER) return;
	m_transfer_error[bus] = error;
	m_transfer_done[bus] = true;
}

bool NrfI2C::wait_for_transfer(uint8_t bus, uint32_t timeout_ms) {
	if (bus >= BSP::I2C_TOTAL_NUMBER) return false;

	// Fallback: bounded busy-wait when system_timer is not yet available (early init)
	if (!system_timer) {
		constexpr uint32_t FALLBACK_TIMEOUT_US = 200000;  // 200 ms hard limit
		uint32_t elapsed = 0;
		while (!m_transfer_done[bus] && elapsed < FALLBACK_TIMEOUT_US) {
			nrf_delay_us(50);
			elapsed += 50;
		}
		return m_transfer_done[bus] && !m_transfer_error[bus];
	}

	uint64_t start_time = system_timer->get_counter();
	uint64_t next_wdt_kick = 500;

	while (!m_transfer_done[bus]) {
		uint64_t elapsed = system_timer->get_counter() - start_time;
		if (elapsed >= timeout_ms) {
			DEBUG_WARN("I2C bus %u transfer timeout (%u ms)", bus, timeout_ms);
			m_stats[bus].timeouts++;

			// Force stop and clear TWIM events
			NRF_TWIM_Type *p_twim = BSP::I2C_Inits[bus].twim.p_twim;
			nrf_twim_task_trigger(p_twim, NRF_TWIM_TASK_STOP);
			nrf_delay_us(100);
			nrf_twim_event_clear(p_twim, NRF_TWIM_EVENT_STOPPED);
			nrf_twim_event_clear(p_twim, NRF_TWIM_EVENT_ERROR);
			return false;
		}

		// Safety net for longer per-op timeouts. Unreachable at the current
		// I2C_OPERATION_TIMEOUT_MS (100ms << the 500ms first-kick, << the 15-min
		// WDT), kept so a future longer timeout still kicks. (2026-07 audit note.)
		if (elapsed >= next_wdt_kick) {
			PMU::kick_watchdog();
			next_wdt_kick = elapsed + 500;
		}

		nrf_delay_us(50);
	}

	return !m_transfer_error[bus];
}

/**
 * @brief Common transfer retry loop for read_safe/write_safe.
 *
 * Starts an async RX or TX, waits with timeout, retries with bus
 * recovery on failure.  Factored to avoid duplicating the retry logic.
 */
bool NrfI2C::transfer_with_retry(uint8_t bus, uint8_t address,
		uint8_t *buffer, unsigned int length,
		bool is_read, bool no_stop, const char *op_name)
{
	if (bus >= BSP::I2C_TOTAL_NUMBER || !m_is_enabled[bus])
		return false;

	m_stats[bus].total_operations++;

	for (uint8_t retry = 0; retry < I2C_MAX_RETRIES; retry++) {
		m_transfer_done[bus] = false;
		m_transfer_error[bus] = false;

		nrfx_err_t err;
		if (is_read)
			err = nrfx_twim_rx(&BSP::I2C_Inits[bus].twim, address, buffer, length);
		else
			err = nrfx_twim_tx(&BSP::I2C_Inits[bus].twim, address, buffer, length, no_stop);

		if (err == NRFX_SUCCESS && wait_for_transfer(bus, I2C_OPERATION_TIMEOUT_MS)) {
			m_stats[bus].successful_operations++;
			return true;
		}

		if (err != NRFX_SUCCESS) {
			DEBUG_WARN("I2C %s(0x%02X) start err=0x%08X retry %u/%u",
			           op_name, address, err, retry + 1, I2C_MAX_RETRIES);
		}

		m_stats[bus].errors++;

		if (retry < I2C_MAX_RETRIES - 1) {
			recover_bus(bus);
			PMU::delay_ms(5);
			// recover_bus() disables the bus (m_is_enabled=false, TWIM left
			// uninitialised) when clock-stretch + full reset both fail. Re-issuing
			// nrfx_twim_rx/tx on it then drives an uninitialised peripheral
			// (nrfx only NRFX_ASSERTs, compiled out in release). Abort instead.
			if (!m_is_enabled[bus]) {
				DEBUG_ERROR("I2C %s(0x%02X): bus disabled after recovery — aborting", op_name, address);
				break;
			}
		}
	}

	DEBUG_ERROR("I2C %s(0x%02X) failed after %u retries", op_name, address, I2C_MAX_RETRIES);
	return false;
}

bool NrfI2C::read_safe(uint8_t bus, uint8_t address, uint8_t *buffer, unsigned int length) {
	return transfer_with_retry(bus, address, buffer, length, true, false, "read");
}

bool NrfI2C::write_safe(uint8_t bus, uint8_t address, const uint8_t *buffer, unsigned int length, bool no_stop) {
	return transfer_with_retry(bus, address, const_cast<uint8_t *>(buffer), length, false, no_stop, "write");
}

void NrfI2C::read(uint8_t bus, uint8_t address, uint8_t *buffer, unsigned int length) {
	if (!read_safe(bus, address, buffer, length))
		throw ErrorCode::I2C_COMMS_ERROR;
}

void NrfI2C::write(uint8_t bus, uint8_t address, const uint8_t *buffer, unsigned int length, bool no_stop) {
	if (!write_safe(bus, address, buffer, length, no_stop))
		throw ErrorCode::I2C_COMMS_ERROR;
}


// ═══════════════════════════════════════════════════════
//  Utility
// ═══════════════════════════════════════════════════════

uint8_t NrfI2C::num_buses(void) {
	return BSP::I2C::I2C_TOTAL_NUMBER;
}

bool NrfI2C::is_enabled(uint8_t bus) {
	return (bus < BSP::I2C_TOTAL_NUMBER) && m_is_enabled[bus];
}

const I2CStats& NrfI2C::get_stats(uint8_t bus) {
	if (bus >= BSP::I2C_TOTAL_NUMBER) {
		static I2CStats dummy = {};
		return dummy;
	}
	return m_stats[bus];
}

void NrfI2C::reset_stats(uint8_t bus) {
	if (bus < BSP::I2C_TOTAL_NUMBER)
		m_stats[bus] = {};
}
