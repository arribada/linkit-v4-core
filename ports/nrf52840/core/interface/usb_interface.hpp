#pragma once

#include <string>
#include "nrf_usb.hpp"

/**
 * USB DTE Interface - provides DTE command interface over USB CDC
 * Can be used in parallel with BLE interface
 */
class UsbInterface {
public:
	static UsbInterface &get_instance() {
		static UsbInterface instance;
		return instance;
	}

	void init() {
		// NrfUSB::init() is called separately in main.cpp
	}

	bool is_connected() { return NrfUSB::is_port_open(); }

	bool has_data() {
		NrfUSB::process();  // Process USB events
		return NrfUSB::has_data();
	}

	std::string read_line() { return NrfUSB::read_line(); }

	bool write(const std::string &str) {
		if (!NrfUSB::is_port_open()) {
			return false;
		}
		NrfUSB::write(const_cast<char *>(str.c_str()), str.length());
		return true;
	}

private:
	UsbInterface() {}
	UsbInterface(UsbInterface const &) = delete;
	void operator=(UsbInterface const &) = delete;
};
