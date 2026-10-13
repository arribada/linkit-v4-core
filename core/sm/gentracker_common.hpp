/**
 * @file gentracker_common.hpp
 * @brief Includes and extern context shared by the GenTracker translation units.
 *
 * gentracker.cpp and gentracker_config_state.cpp are two halves of one state
 * machine, and both need the same set of includes, the same CRC16 shim and the
 * same extern singletons. That preamble had been copied verbatim into both --
 * 75 identical lines, including a full crc16_compute implementation under
 * `#ifdef CPPUTEST`. Two copies of a conditional implementation is one copy too
 * many: a fix applied to one of them is invisible in the other, and neither
 * file says the other exists.
 *
 * Nothing here is logic of its own. Adding an include or an extern used by both
 * halves belongs in this file; anything needed by only one stays in that one.
 */

#pragma once

#include "scheduler.hpp"
#include "service.hpp"
#include "dte_handler.hpp"
#include "filesystem.hpp"
#include "config_store.hpp"
#include "error.hpp"
#include "timer.hpp"
#include "debug.hpp"
#include "switch.hpp"
#include "reed.hpp"
#include "ledsm.hpp"
#include "buzzm.hpp"
#include "battery.hpp"
#include "rgb_led.hpp"
#include "rtc.hpp"

extern RTC *rtc;
#include "led.hpp"
#include "gps.hpp"
#include "gps_service.hpp"
#include "ble_service.hpp"
#include "sws_analog_service.hpp"
// CRC16-CCITT: nRF SDK header in firmware build, inline stub in CppUTest build.
// Mirrors the conditional in sws_analog_constants.hpp so tests can build this
// file (the SDK header is not on the test include path).
#ifndef CPPUTEST
#include "crc16.h"
#else
#include <cstdint>
static inline uint16_t crc16_compute(const uint8_t *data, uint16_t length, const uint16_t *) {
	uint16_t crc = 0xFFFF;
	for (uint16_t i = 0; i < length; i++) {
		crc ^= static_cast<uint16_t>(data[i]) << 8;
		for (uint8_t j = 0; j < 8; j++)
			crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
	}
	return crc;
}
#endif
#include "gentracker.hpp"

// USB DTE interface (platform-specific)
#ifdef USB_DTE_ENABLED
#include "usb_interface.hpp"
#endif
#ifdef BENCH_TEST
#include "bench_console.hpp"
#endif

// LoRa device instance (for bridge mode in process_usb_data)
#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
#include "lora_rak3172.hpp"
extern LoRaDevice *lora_device_instance;
#endif

// KIM2 device instance (for bridge mode in process_usb_data)
#if !(defined(LORA_RAK3172) && (LORA_RAK3172 == 1)) && !(defined(ARGOS_SMD) && (ARGOS_SMD == 1))
#include "kim2.hpp"
extern KIM2Device *kim2_device_instance;
#endif

// LED hardware access for forced LED off before powerdown
extern RGBLed *status_led;

// These contexts must be created before the FSM is initialised
extern FileSystem *main_filesystem;
extern Scheduler *system_scheduler;
extern Timer *system_timer;
extern ConfigurationStore *configuration_store;
extern BLEService *ble_service;
extern OTAFileUpdater *ota_updater;
extern DTEHandler *dte_handler;
extern ReedSwitch *reed_switch;
extern BatteryMonitor *battery_monitor;
extern BaseDebugMode g_debug_mode;
