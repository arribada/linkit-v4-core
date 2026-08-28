#pragma once

/**
 * @file rgb_led.hpp
 * @brief Abstract RGB LED interface with flash and alternate-flash support.
 */

enum class RGBLedColor {
	BLACK,
	RED,
	GREEN,
	BLUE,
	CYAN,
	MAGENTA,
	YELLOW,
	WHITE,
};

class RGBLed {
public:
	static const char *color_to_string(RGBLedColor color) {
		switch (color) {
		case RGBLedColor::BLACK: return "BLACK";
		case RGBLedColor::RED: return "RED";
		case RGBLedColor::GREEN: return "GREEN";
		case RGBLedColor::BLUE: return "BLUE";
		case RGBLedColor::CYAN: return "CYAN";
		case RGBLedColor::MAGENTA: return "MAGENTA";
		case RGBLedColor::YELLOW: return "YELLOW";
		case RGBLedColor::WHITE: return "WHITE";
		default: return "UNKNOWN";
		}
	}

	RGBLed() = default;
	virtual ~RGBLed() = default;
	virtual void off() = 0;                                                   ///< Turn LED off (set to BLACK)
	virtual RGBLedColor get_state() = 0;                                      ///< Current color (BLACK if off)
	virtual void set(RGBLedColor color) = 0;                                  ///< Set solid color, stop flashing
	virtual void flash(RGBLedColor color, unsigned int period_ms = 500) = 0;  ///< Flash color on/off
	virtual void flash_alternate(RGBLedColor color1, RGBLedColor color2,
	                             unsigned int period_ms = 250) = 0;  ///< Alternate between two colors
	virtual bool is_flashing() = 0;                                  ///< True if currently in flash mode
};
