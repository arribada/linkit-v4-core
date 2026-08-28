/**
 * @file irq.hpp
 * @brief Abstract GPIO interrupt interface — enable/disable with callback.
 */

#pragma once

#include <functional>

/// @brief Abstract GPIO interrupt — wraps GPIOTE for edge-triggered callbacks.
class IRQ {
protected:
	std::function<void()> m_func;

public:
	IRQ() : m_func(nullptr) {}
	virtual ~IRQ() {}
	virtual void enable(std::function<void()> func) { m_func = func; }
	virtual void disable() { m_func = nullptr; }
};
