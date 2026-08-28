/**
 * @file nrf_irq.cpp
 * @brief GPIOTE interrupt wrapper — maps pin events to NrfIRQ objects via static table.
 */

#include <functional>

#include "error.hpp"
#include "debug.hpp"
#include "bsp.hpp"
#include "nrf_irq.hpp"
#include "gpio.hpp"
#include "nrfx_gpiote.h"
#include "scheduler.hpp"
#include "interrupt_lock.hpp"

extern Scheduler *system_scheduler;

// ═══════════════════════════════════════════════════════
//  Pin → NrfIRQ lookup (replaces std::map to avoid heap + ISR-unsafe access)
// ═══════════════════════════════════════════════════════

// nRF52840 has pins P0.00..P0.31 and P1.00..P1.15 = 48 pins max.
// The GPIOTE ISR callback only provides the raw pin number, so we
// need a fast O(1) lookup from pin number to NrfIRQ*.
static constexpr unsigned int MAX_NRF_PINS = 48;
static NrfIRQ *s_pin_map[MAX_NRF_PINS] = {};

static void pin_map_add(uint32_t pin, NrfIRQ *ref) {
	if (pin < MAX_NRF_PINS) s_pin_map[pin] = ref;
}

static void pin_map_remove(uint32_t pin) {
	if (pin < MAX_NRF_PINS) s_pin_map[pin] = nullptr;
}

static NrfIRQ *pin_map_get(uint32_t pin) {
	return (pin < MAX_NRF_PINS) ? s_pin_map[pin] : nullptr;
}


// ═══════════════════════════════════════════════════════
//  GPIOTE ISR handler
// ═══════════════════════════════════════════════════════

static void nrfx_gpiote_in_event_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
	if (action != NRF_GPIOTE_POLARITY_LOTOHI && action != NRF_GPIOTE_POLARITY_HITOLO) return;

	NrfIRQ *obj = pin_map_get(static_cast<uint32_t>(pin));
	if (obj) obj->process_event();
}


// ═══════════════════════════════════════════════════════
//  NrfIRQ implementation
// ═══════════════════════════════════════════════════════

/// @brief Register a GPIOTE input channel for the given BSP pin.
NrfIRQ::NrfIRQ(int pin) : m_pin(pin), m_task{} {
	if (!nrfx_gpiote_is_init()) {
		nrfx_err_t err = nrfx_gpiote_init();
		if (err != NRFX_SUCCESS) {
			DEBUG_ERROR("GPIOTE init failed: 0x%08X", err);
			throw ErrorCode::RESOURCE_NOT_AVAILABLE;
		}
	}

	uint32_t hw_pin = BSP::GPIO_Inits[m_pin].pin_number;
	if (nrfx_gpiote_in_init(hw_pin, &BSP::GPIO_Inits[m_pin].gpiote_in_config, nrfx_gpiote_in_event_handler)
	    != NRFX_SUCCESS) {
		throw ErrorCode::RESOURCE_NOT_AVAILABLE;
	}

	pin_map_add(hw_pin, this);
}

NrfIRQ::~NrfIRQ() {
	disable();
	uint32_t hw_pin = BSP::GPIO_Inits[m_pin].pin_number;
	nrfx_gpiote_in_uninit(hw_pin);
	pin_map_remove(hw_pin);
}

/// @brief Enable the GPIOTE interrupt and set the callback.
void NrfIRQ::enable(std::function<void()> func) {
	InterruptLock lock;
	system_scheduler->cancel_task(m_task);
	m_func = func;
	nrfx_gpiote_in_event_enable(BSP::GPIO_Inits[m_pin].pin_number, true);
}

/// @brief Disable the GPIOTE interrupt and clear the callback.
void NrfIRQ::disable() {
	InterruptLock lock;
	m_func = nullptr;
	nrfx_gpiote_in_event_disable(BSP::GPIO_Inits[m_pin].pin_number);
	system_scheduler->cancel_task(m_task);
}

/// @brief Post the user callback to the scheduler (called from GPIOTE ISR).
void NrfIRQ::process_event() {
	if (m_func) m_task = system_scheduler->post_task_prio([this]() { m_func(); }, "NrfIRQTask");
}

/// @brief Read current pin level, returns true if the active polarity matches.
bool NrfIRQ::poll() {
	if (BSP::GPIO_Inits[m_pin].gpiote_in_config.sense == NRF_GPIOTE_POLARITY_LOTOHI) return GPIOPins::value(m_pin);
	return !GPIOPins::value(m_pin);
}
