#include "gpio.hpp"

#include "CppUTestExt/MockSupport.h"

uint8_t GPIOPins::m_sensors_pwr_refcount = 0;
bool GPIOPins::m_gnss_uart_active = false;

void GPIOPins::initialise() {
	mock().actualCall("initialise");
}

void GPIOPins::init_pin(uint32_t pin) {
	mock().actualCall("init_pin").withParameter("pin", pin);
}

void GPIOPins::set(uint32_t pin) {
	mock().actualCall("set").withParameter("pin", pin);
}

void GPIOPins::clear(uint32_t pin) {
	mock().actualCall("clear").withParameter("pin", pin);
}

uint32_t GPIOPins::value(uint32_t pin) {
	return mock().actualCall("value").withParameter("pin", pin).returnUnsignedLongIntValue();
}

void GPIOPins::toggle(uint32_t pin) {
	mock().actualCall("toggle").withParameter("pin", pin);
}

void GPIOPins::enable(uint32_t pin) {
	mock().actualCall("enable").withParameter("pin", pin);
}

void GPIOPins::disable(uint32_t pin) {
	mock().actualCall("disable").withParameter("pin", pin);
}

void GPIOPins::acquire_sensors_pwr() {
	mock().actualCall("acquire_sensors_pwr");
	m_sensors_pwr_refcount++;
}

void GPIOPins::release_sensors_pwr() {
	mock().actualCall("release_sensors_pwr");
	if (m_sensors_pwr_refcount > 0) m_sensors_pwr_refcount--;
}

bool GPIOPins::get_sensors_pwr_state() {
	return m_sensors_pwr_refcount > 0;
}

// Verrou "UART GNSS en service" (2026-08): pas de mock().actualCall ici, il
// serait pose/retire a chaque init/deinit d'UBXComms et ferait echouer tous les
// tests existants qui ne l'attendent pas. Simple etat lisible par les tests.
void GPIOPins::set_gnss_uart_active(bool active) {
	m_gnss_uart_active = active;
}

bool GPIOPins::is_gnss_uart_active() {
	return m_gnss_uart_active;
}

uint8_t GPIOPins::get_sensors_pwr_refcount() {
	return m_sensors_pwr_refcount;
}

void GPIOPins::pulse_low_then_release(uint32_t pin, uint32_t duration_ms) {
	mock().actualCall("pulse_low_then_release").withParameter("pin", pin).withParameter("duration_ms", duration_ms);
}

void GPIOPins::drive_low(uint32_t pin) {
	mock().actualCall("drive_low").withParameter("pin", pin);
}

void GPIOPins::release_to_highz(uint32_t pin) {
	mock().actualCall("release_to_highz").withParameter("pin", pin);
}

void GPIOPins::disconnect_sensor_pins() {}
void GPIOPins::reconnect_sensor_pins() {}
