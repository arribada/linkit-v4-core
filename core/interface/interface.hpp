/**
 * @file interface.hpp
 * @brief Abstract DTE communication interface — read/write string lines.
 */

#pragma once

#include <string>

/// @brief Abstract DTE interface — implemented by USB CDC and BLE NUS.
class Interface {
public:
	virtual std::string read_line() = 0;
	virtual void write(std::string str) = 0;
};