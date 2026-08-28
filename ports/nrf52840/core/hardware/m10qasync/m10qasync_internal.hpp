#pragma once

/**
 * @file m10qasync_internal.hpp
 * @brief Definitions shared between m10qasync.cpp and m10qasync_config.cpp.
 *
 * Private to the M10Q driver. Nothing outside this folder should include it:
 * these are the four things the UBX configuration helpers still need from the
 * state machine after the two were split into separate translation units.
 */

#include "debug.hpp"

// The two rates the firmware itself programmes. The receiver keeps its rate in
// BBR, which is why state_poweron probes BOOT_BAUD_TABLE instead of assuming
// either of them.
#define DEFAULT_BAUDRATE 9600
#define MAX_BAUDRATE     460800

// Pre-deploy validation channel — grep-friendly [VAL-GNSS] tags. Default off,
// zero overhead. Enabled by the CMake option of the same name.
#ifndef VALIDATION_LOG_ENABLE
#define VALIDATION_LOG_ENABLE 0
#endif

#if VALIDATION_LOG_ENABLE
#define VAL_GNSS(fmt, ...) DEBUG_INFO("[VAL-GNSS] " fmt, ##__VA_ARGS__)
#else
#define VAL_GNSS(fmt, ...) \
	do {                   \
	} while (0)
#endif

// Reads m_state from the enclosing M10QAsyncReceiver method.
#define STATE_EQUAL(x) (m_state == x)
