/**
 * @file nrf_switch.cpp
 * @brief nRF52840 debounced switch — GPIOTE edge detect + timer debounce.
 */

#include <functional>

#include "nrfx_gpiote.h"
#include "gpio.hpp"
#include "switch.hpp"
#include "nrf_switch.hpp"
#include "bsp.hpp"
#include "timer.hpp"
#include "error.hpp"
#include "debug.hpp"

extern Timer *system_timer;

// ═══════════════════════════════════════════════════════
//  Pin → NrfSwitch lookup (replaces std::map — ISR-safe, no heap)
// ═══════════════════════════════════════════════════════

static constexpr unsigned int MAX_NRF_PINS = 48;  // P0.00..P0.31 + P1.00..P1.15
static NrfSwitch *s_pin_map[MAX_NRF_PINS] = {};

static void pin_map_add(uint32_t pin, NrfSwitch *ref) {
	if (pin < MAX_NRF_PINS) s_pin_map[pin] = ref;
}

static void pin_map_remove(uint32_t pin) {
	if (pin < MAX_NRF_PINS) s_pin_map[pin] = nullptr;
}

static NrfSwitch *pin_map_get(uint32_t pin) {
	return (pin < MAX_NRF_PINS) ? s_pin_map[pin] : nullptr;
}


// ═══════════════════════════════════════════════════════
//  GPIOTE ISR handler
// ═══════════════════════════════════════════════════════

void nrfx_gpiote_switch_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
	NrfSwitch *obj = pin_map_get(static_cast<uint32_t>(pin));
	if (!obj) return;

	switch (action) {
	case NRF_GPIOTE_POLARITY_LOTOHI: obj->process_event(true); break;
	case NRF_GPIOTE_POLARITY_HITOLO: obj->process_event(false); break;
	case NRF_GPIOTE_POLARITY_TOGGLE: obj->process_event(obj->get_state()); break;
	default: break;
	}
}


// ═══════════════════════════════════════════════════════
//  NrfSwitch implementation
// ═══════════════════════════════════════════════════════

/// @brief Init GPIOTE library if needed.  Does not register the pin yet (call start/resume).
NrfSwitch::NrfSwitch(int pin, unsigned int hysteresis_time_ms, bool active_state)
    : Switch(pin, hysteresis_time_ms, active_state) {
	if (!nrfx_gpiote_is_init()) {
		nrfx_err_t err = nrfx_gpiote_init();
		if (err != NRFX_SUCCESS) {
			DEBUG_ERROR("GPIOTE init failed: 0x%08X", err);
			throw ErrorCode::RESOURCE_NOT_AVAILABLE;
		}
	}
	m_is_paused = true;
}

NrfSwitch::~NrfSwitch() {
	stop();
}

/// @brief Register callback and enable GPIOTE events.
void NrfSwitch::start(std::function<void(bool)> func) {
	Switch::start(func);
	resume();
}

/// @brief Read current physical pin state (true = active).
bool NrfSwitch::get_state() {
	return GPIOPins::value(m_pin) == m_active_state;
}

/// @brief Clear callback and disable GPIOTE events.
void NrfSwitch::stop() {
	Switch::stop();
	pause();
}

/// @brief Apply new debounced state and fire callback if changed.
void NrfSwitch::update_state(bool state) {
	if (state != static_cast<bool>(m_current_state)) {
		DEBUG_TRACE("NrfSwitch: %u -> %u", m_current_state, state);
		m_current_state = state;
		if (m_state_change_handler) m_state_change_handler(state);
	}
}

/// @brief ISR entry — disable GPIOTE (suppress bounces) and schedule a readback after debounce.
void NrfSwitch::process_event(bool /* state */) {
	// On first edge: disable GPIOTE to suppress all bounce interrupts,
	// then schedule a single readback after the hysteresis period.
	if (m_debouncing) return;

	m_debouncing = true;
	nrfx_gpiote_in_event_disable(BSP::GPIO_Inits[m_pin].pin_number);

	uint64_t now = system_timer->get_counter();
	m_timer_handle = system_timer->add_schedule(
	    [this]() {
		    // Read the actual settled pin state after debounce
		    bool settled = (GPIOPins::value(m_pin) == m_active_state);
		    update_state(settled);
		    // Re-enable GPIOTE for next edge
		    m_debouncing = false;
		    nrfx_gpiote_in_event_enable(BSP::GPIO_Inits[m_pin].pin_number, true);
	    },
	    now + m_hysteresis_time_ms);

	// If schedule list was full, the debounce callback won't fire.
	// Recover immediately to avoid permanent reed switch lockout.
	if (!m_timer_handle.has_value()) {
		m_debouncing = false;
		nrfx_gpiote_in_event_enable(BSP::GPIO_Inits[m_pin].pin_number, true);
	}
}

/// @brief Disable GPIOTE, cancel pending debounce timer, force state to off.
void NrfSwitch::pause() {
	if (m_is_paused) return;
	system_timer->cancel_schedule(m_timer_handle);
	m_debouncing = false;  // Reset debounce state — critical for resume() to work
	nrfx_gpiote_in_event_disable(BSP::GPIO_Inits[m_pin].pin_number);
	nrfx_gpiote_in_uninit(BSP::GPIO_Inits[m_pin].pin_number);
	pin_map_remove(BSP::GPIO_Inits[m_pin].pin_number);
	m_is_paused = true;
	update_state(false);
}

/// @brief Re-register GPIOTE channel, read initial state, and re-enable events.
void NrfSwitch::resume() {
	if (!m_is_paused) return;
	pin_map_add(BSP::GPIO_Inits[m_pin].pin_number, this);
	nrfx_err_t err = nrfx_gpiote_in_init(BSP::GPIO_Inits[m_pin].pin_number, &BSP::GPIO_Inits[m_pin].gpiote_in_config,
	                                     nrfx_gpiote_switch_handler);
	if (err != NRFX_SUCCESS) {
		pin_map_remove(BSP::GPIO_Inits[m_pin].pin_number);
		DEBUG_ERROR("NrfSwitch: GPIOTE resume failed pin %u (0x%08X) — switch inactive", m_pin, err);
		return;  // Degrade gracefully — switch stays paused, no throw
	}
	m_debouncing = false;  // Ensure clean state on resume
	// Read actual pin state BEFORE enabling events to suppress spurious initial event
	m_current_state = (GPIOPins::value(m_pin) == m_active_state) ? 1 : 0;
	nrfx_gpiote_in_event_enable(BSP::GPIO_Inits[m_pin].pin_number, true);
	m_is_paused = false;
}
