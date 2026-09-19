/**
 * @file cam.hpp
 * @brief Abstract camera device interface — power on/off, capture count.
 */

#pragma once

#include <map>
#include <cstdint>
#include "base_types.hpp"
#include "events.hpp"
/* 
struct CAMSettings {
	unsigned int time_on;
	unsigned int time_off;
} */
/* 
struct CAMData {
	bool state;
	uint16_t counter
};

 */
/// @brief Time the power button must be held for the camera to register a
/// press (ms). On power-off the module closes its SD card file during this
/// window — cutting the rail before the end of it risks corrupting the card.
static constexpr unsigned int PWR_BUTT_DELAY = 2000;
/// @brief Power rail settling time around a button press (ms).
static constexpr unsigned int PWR_DELAY = 100;

struct CAMEventError {};
struct CAMEventPowerOn {};
struct CAMEventPowerOff {};

class CAMEventListener {
public:
	virtual ~CAMEventListener() {}
	virtual void react(const CAMEventPowerOn &) {}
	virtual void react(const CAMEventPowerOff &) {}
	virtual void react(const CAMEventError &) {}
};

/// @brief Abstract camera device — implemented by board-specific driver.
///
/// power_on() and power_off() are ASYNCHRONOUS: they start a power sequence
/// lasting PWR_DELAY + PWR_BUTT_DELAY ms and return at once, so the cooperative
/// scheduler keeps serving GNSS / TX / SWS meanwhile. Completion is signalled
/// by CAMEventPowerOn / CAMEventPowerOff. Both calls are idempotent: asking
/// for the state the device is already in, or for a sequence already running,
/// does nothing. is_powered_on() only reports true once a power-on sequence
/// has completed.
class CAMDevice : public EventEmitter<CAMEventListener> {
public:
	virtual ~CAMDevice() {}
	// These methods are specific to the chipset and should be implemented by device-specific subclass
	virtual void power_off() = 0;
	virtual void power_on() = 0;
	/// @brief Synchronous power-off, for the shutdown path ONLY.
	///
	/// Blocks for the whole button sequence, from whatever step an asynchronous
	/// sequence had reached, so the camera has really closed its file before
	/// the caller cuts power or enters System OFF. Emits CAMEventPowerOff when
	/// the device was not already off. Everything else must use power_off().
	virtual void power_off_blocking() = 0;
	virtual bool is_powered_on() = 0;
	virtual unsigned int get_num_captures() = 0;
};
