/**
 * @file runcam_protocol.hpp
 * @brief RunCam Device Protocol v1.0 — frame encoding and response decoding.
 *
 * The serial protocol of RunCam's split cameras (the Split 4 among them), at
 * 115200 bauds 8N1. A frame is `0xCC | command | payload | crc`, the CRC being
 * CRC-8/DVB-S2 (polynomial 0xD5, initial value 0) over every byte from the
 * header on, so a whole valid frame — its own CRC included — checks to 0.
 *
 * Only some commands are answered. CAMERA_CONTROL, the one that presses the
 * camera's buttons, never is: the camera does not say whether it acted on it.
 *
 * Sources, in order of authority: RunCam's "RunCam Device Protocol" article
 * (support.runcam.com, 360014537794 — the live page answers 403, read from its
 * 2025-03 web archive copy), Betaflight io/rcdevice.c (tag v3.3.0 for the
 * chunked settings answers, removed from later versions) and ArduPilot
 * AP_RunCam.cpp. Pure code, no hardware access: host-tested.
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace runcam {

static constexpr uint8_t HEADER = 0xCC;
/// @brief Largest frame either side sends: header, command or chunk count,
/// length, 60 data bytes and the CRC (Betaflight's 64-byte packet buffer).
static constexpr size_t MAX_FRAME = 64;

enum class Command : uint8_t {
	GET_DEVICE_INFO = 0x00,      ///< answer: CC version features_lo features_hi crc
	CAMERA_CONTROL = 0x01,       ///< one ControlAction byte; NO answer
	KEY_PRESS = 0x02,            ///< 5-key OSD remote, one key byte; answer CC crc
	KEY_RELEASE = 0x03,          ///< 5-key OSD remote; answer CC crc
	KEY_CONNECTION = 0x04,       ///< 5-key session open (1) / close (2); answer CC (op<<4|ok) crc
	GET_SETTINGS = 0x10,         ///< parent setting id, chunk index; chunked answer
	READ_SETTING_DETAIL = 0x11,  ///< setting id, chunk index; chunked answer
	WRITE_SETTING = 0x13,        ///< setting id, value; answer CC result update crc
};

/// @brief The CAMERA_CONTROL action byte.
enum class ControlAction : uint8_t {
	/// The Split 4 has no Wi-Fi: this is its Mode button. Three presses within
	/// 2 s in standby FACTORY-RESET the camera and switch it off (manual).
	WIFI_BUTTON = 0x00,
	/// A short press of Power/Shutter. In video mode it starts or stops a
	/// recording — a TOGGLE — and inside the OSD menu it moves to the next item.
	/// Power on/off is a LONG press, which the protocol cannot simulate.
	POWER_BUTTON = 0x01,
	CHANGE_MODE = 0x02,  ///< video <-> OSD settings menu
	START_RECORDING = 0x03,
	STOP_RECORDING = 0x04,
};

/// @brief GET_DEVICE_INFO feature bits. The Split 4 is reported at 0x0077
/// (ArduPilot forum, camera firmware 1.1.5): power, Wi-Fi, mode, settings,
/// DisplayPort and START — but not STOP, and ArduPilot found START unreliable
/// enough to drive that camera with power-button presses only.
namespace feature {
static constexpr uint16_t POWER_BUTTON = 1u << 0;
static constexpr uint16_t WIFI_BUTTON = 1u << 1;
static constexpr uint16_t CHANGE_MODE = 1u << 2;
static constexpr uint16_t FIVE_KEY_OSD = 1u << 3;
static constexpr uint16_t SETTINGS_ACCESS = 1u << 4;
static constexpr uint16_t DISPLAY_PORT = 1u << 5;
static constexpr uint16_t START_RECORDING = 1u << 6;
static constexpr uint16_t STOP_RECORDING = 1u << 7;
static constexpr uint16_t FC_ATTITUDE = 1u << 9;
}  // namespace feature

/// @brief The reserved setting ids (READ_SETTING_DETAIL / WRITE_SETTING), as
/// the Split 4 answers them on the bench (2026-10-26).
namespace setting {
static constexpr uint8_t SDCARD_CAPACITY = 3;  ///< INFO "free/total" in MB, live while recording
static constexpr uint8_t REMAINING_TIME = 4;   ///< INFO seconds of recording the card still takes, live
static constexpr uint8_t RESOLUTION = 5;       ///< TEXT_SELECTION, numbered from 0; taken up at the next boot
static constexpr uint8_t CAMERA_TIME = 6;      ///< STRING "YYYYMMDDTHHMMSS.0", writable
static constexpr uint8_t TYPE_TEXT_SELECTION = 9;
static constexpr uint8_t TYPE_STRING = 10;
static constexpr uint8_t TYPE_INFO = 12;
}  // namespace setting

/// @brief One CRC-8/DVB-S2 step.
uint8_t crc8_dvb_s2(uint8_t crc, uint8_t byte);
/// @brief CRC-8/DVB-S2 of a buffer, from 0.
uint8_t crc8_dvb_s2(const uint8_t *data, size_t len);

/// @brief Build `HEADER | command | payload | crc` into @p out.
/// @return The frame length, or 0 if it does not fit in @p out_size or MAX_FRAME.
size_t encode(Command command, const uint8_t *payload, size_t payload_len, uint8_t *out, size_t out_size);
/// @brief Build a CAMERA_CONTROL frame (always 4 bytes).
size_t encode_control(ControlAction action, uint8_t *out, size_t out_size);

/// @brief A whole GET_DEVICE_INFO answer, as decoded.
struct DeviceInfo {
	uint8_t protocol_version = 0;
	uint16_t features = 0;
};

/// @brief Read a COMPLETE GET_DEVICE_INFO answer (5 bytes, CRC already checked).
bool parse_device_info(const uint8_t *frame, size_t len, DeviceInfo &out);

/// @brief Build a READ_SETTING_DETAIL request for @p setting_id, chunk 0.
size_t encode_read_setting(uint8_t setting_id, uint8_t *out, size_t out_size);
/// @brief Build a WRITE_SETTING request carrying @p text (STRING setting), NUL included.
size_t encode_write_text(uint8_t setting_id, const char *text, uint8_t *out, size_t out_size);

/// @brief The text of a COMPLETE READ_SETTING_DETAIL answer for an INFO or a
/// STRING setting: `CC remaining length type text\0 ... crc`. False for any
/// other type, a missing terminator, or a text longer than @p out_size - 1.
bool parse_setting_text(const uint8_t *frame, size_t len, char *out, size_t out_size);
/// @brief "15049/15079" -> 15049 and 15079 (MB free / total).
bool parse_sd_capacity(const char *text, unsigned int &free_mb, unsigned int &total_mb);
/// @brief A decimal integer and nothing else ("2681").
bool parse_uint(const char *text, unsigned int &value);
/// @brief A WRITE_SETTING answer, `CC result update crc`: true if result is 0.
bool write_succeeded(const uint8_t *frame, size_t len);

/// @brief Build a WRITE_SETTING request carrying one byte: a UINT8, or the
/// index of a TEXT_SELECTION option.
size_t encode_write_u8(uint8_t setting_id, uint8_t value, uint8_t *out, size_t out_size);
/// @brief A COMPLETE READ_SETTING_DETAIL answer for a TEXT_SELECTION setting:
/// `CC remaining length 9 value options\0 crc`, the options separated by ';'.
/// False for any other type, a missing terminator, or options longer than
/// @p options_size - 1.
bool parse_setting_selection(const uint8_t *frame, size_t len, uint8_t &value, char *options, size_t options_size);
/// @brief The position (from 0) of @p label among the ';'-separated @p options, or -1.
int selection_position(const char *options, const char *label);
/// @brief The option at @p position (from 0). False if there is none or it does not fit @p out_size.
bool selection_label(const char *options, unsigned int position, char *out, size_t out_size);

/// @brief Space-separated names of the set feature bits ("POWER WIFI ..."), for
/// logs and the bench console. Always NUL-terminated when @p out_size > 0.
void describe_features(uint16_t features, char *out, size_t out_size);

/// @brief Reassembles the camera's answer to the last request, byte by byte.
///
/// Armed by expect() for one command; bytes before the 0xCC header are skipped,
/// so the decoder resynchronises on its own after line noise (a camera still
/// booting, a framing error). Once armed it counts — a 0xCC inside the payload
/// or as a CRC is a data byte, not a new header. The chunked settings answers
/// carry their own length (`CC remaining length data crc`); every other answer
/// has a fixed one. Bytes arriving while nothing is expected are dropped, which
/// is also what becomes of a frame the camera sends on its own (it may ask for
/// the flight controller's attitude, command 0x50).
class ResponseDecoder {
public:
	enum class Status {
		IDLE,         ///< not armed: the byte was dropped
		IN_PROGRESS,  ///< armed, the answer is not complete yet
		COMPLETE,     ///< a whole answer with a valid CRC is in frame()
		CORRUPT,      ///< a whole answer that fails its CRC, or a length that cannot fit
	};

	/// @brief Wait for the answer to @p command.
	/// @return false for a command that is never answered (CAMERA_CONTROL).
	bool expect(Command command);
	void cancel() { m_armed = false; }
	bool is_armed() const { return m_armed; }
	Status feed(uint8_t byte);

	Command command() const { return m_command; }
	const uint8_t *frame() const { return m_frame; }
	size_t length() const { return m_length; }

private:
	static constexpr size_t CHUNKED = 0xFF;  ///< length read from the frame itself

	uint8_t m_frame[MAX_FRAME] = {};
	size_t m_length = 0;
	size_t m_expected = 0;
	Command m_command = Command::GET_DEVICE_INFO;
	bool m_armed = false;

	static size_t answer_length(Command command);
};

}  // namespace runcam
