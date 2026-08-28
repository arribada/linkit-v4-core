#pragma once

/**
 * @file nrf_rtc.hpp
 * @brief nRF52840 RTC-based wall-clock time (epoch seconds from a low-power 8 Hz counter).
 *
 * Uses a 24-bit RTC peripheral at 8 Hz with overflow tracking to provide
 * a 64-bit tick count.  Wall-clock time = uptime + offset (set by GNSS fix).
 * Singleton — only one date/time RTC instance exists.
 */

#include "rtc.hpp"

class NrfRTC final : public RTC {
public:
	static NrfRTC &get_instance() {
		static NrfRTC instance;
		return instance;
	}

	void init();
	void uninit();

	std::time_t gettime() override;
	void settime(std::time_t time) override;
	bool is_set() override;

private:
	bool m_is_set;
	NrfRTC() : m_is_set(false) {}
	NrfRTC(NrfRTC const &) = delete;
	void operator=(NrfRTC const &) = delete;

	/// @brief Seconds since boot (derived from 64-bit tick count / 8 Hz).
	int64_t getuptime();
};
