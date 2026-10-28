#include "CppUTest/TestHarness.h"

#include <cstring>
#include <vector>

#include "runcam_protocol.hpp"

using namespace runcam;

/*
 * RunCam Device Protocol codec. The frames below were computed independently
 * (Python, CRC-8/DVB-S2, poly 0xD5, init 0) before the code existed; the one
 * also worked out by hand in the plan -- CC 01 01 E7, the power-button press --
 * is among them. A wrong polynomial or a CRC taken from the wrong byte would
 * change every one of them, and the camera would ignore every command in
 * silence: CAMERA_CONTROL is never answered.
 */
TEST_GROUP(RunCamProtocol){
	ResponseDecoder::Status feed_all(ResponseDecoder & decoder, const std::vector<uint8_t> &bytes){
	    ResponseDecoder::Status st = ResponseDecoder::Status::IDLE;
for (uint8_t b : bytes) {
	st = decoder.feed(b);
	if (st == ResponseDecoder::Status::COMPLETE || st == ResponseDecoder::Status::CORRUPT) break;
}
return st;
}

std::vector<uint8_t> frame_of(Command command, std::vector<uint8_t> payload) {
	uint8_t out[MAX_FRAME];
	size_t len = encode(command, payload.data(), payload.size(), out, sizeof(out));
	return std::vector<uint8_t>(out, out + len);
}

std::vector<uint8_t> control_frame(ControlAction action) {
	uint8_t out[4];
	size_t len = encode_control(action, out, sizeof(out));
	return std::vector<uint8_t>(out, out + len);
}
}
;

TEST(RunCamProtocol, Crc8DvbS2CheckValue) {
	// The catalogued check value of CRC-8/DVB-S2.
	const uint8_t data[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
	CHECK_EQUAL(0xBC, crc8_dvb_s2(data, sizeof(data)));
}

TEST(RunCamProtocol, EncodesGetDeviceInfo) {
	CHECK(frame_of(Command::GET_DEVICE_INFO, {}) == (std::vector<uint8_t>{ 0xCC, 0x00, 0x60 }));
}

TEST(RunCamProtocol, EncodesEveryCameraControlAction) {
	CHECK(control_frame(ControlAction::WIFI_BUTTON) == (std::vector<uint8_t>{ 0xCC, 0x01, 0x00, 0x32 }));
	CHECK(control_frame(ControlAction::POWER_BUTTON) == (std::vector<uint8_t>{ 0xCC, 0x01, 0x01, 0xE7 }));
	CHECK(control_frame(ControlAction::CHANGE_MODE) == (std::vector<uint8_t>{ 0xCC, 0x01, 0x02, 0x4D }));
	CHECK(control_frame(ControlAction::START_RECORDING) == (std::vector<uint8_t>{ 0xCC, 0x01, 0x03, 0x98 }));
	CHECK(control_frame(ControlAction::STOP_RECORDING) == (std::vector<uint8_t>{ 0xCC, 0x01, 0x04, 0xCC }));
}

TEST(RunCamProtocol, EncodesSettingsAndKeyCommands) {
	CHECK(frame_of(Command::GET_SETTINGS, { 0x00, 0x00 }) == (std::vector<uint8_t>{ 0xCC, 0x10, 0x00, 0x00, 0x13 }));
	CHECK(frame_of(Command::READ_SETTING_DETAIL, { 0x03, 0x00 })
	      == (std::vector<uint8_t>{ 0xCC, 0x11, 0x03, 0x00, 0x8D }));
	CHECK(frame_of(Command::KEY_PRESS, { 0x01 }) == (std::vector<uint8_t>{ 0xCC, 0x02, 0x01, 0xFA }));
	CHECK(frame_of(Command::KEY_RELEASE, {}) == (std::vector<uint8_t>{ 0xCC, 0x03, 0xCA }));
	CHECK(frame_of(Command::KEY_CONNECTION, { 0x01 }) == (std::vector<uint8_t>{ 0xCC, 0x04, 0x01, 0xC0 }));
}

TEST(RunCamProtocol, EncodeRefusesWhatDoesNotFit) {
	uint8_t payload[MAX_FRAME] = {};
	uint8_t out[MAX_FRAME];
	CHECK_EQUAL(0U, encode(Command::WRITE_SETTING, payload, MAX_FRAME - 2, out, sizeof(out)));  // 65 bytes
	CHECK_EQUAL(MAX_FRAME, encode(Command::WRITE_SETTING, payload, MAX_FRAME - 3, out, sizeof(out)));
	uint8_t small[3];
	CHECK_EQUAL(0U, encode_control(ControlAction::POWER_BUTTON, small, sizeof(small)));
}

TEST(RunCamProtocol, EveryEncodedFrameChecksToZero) {
	CHECK_EQUAL(0, crc8_dvb_s2(control_frame(ControlAction::POWER_BUTTON).data(), 4));
	std::vector<uint8_t> f = frame_of(Command::WRITE_SETTING, { 0x06, 'a', 'b', 0x00 });
	CHECK_EQUAL(0, crc8_dvb_s2(f.data(), f.size()));
}

TEST(RunCamProtocol, DecodesDeviceInfoAsReportedForTheSplit4) {
	ResponseDecoder decoder;
	CHECK_TRUE(decoder.expect(Command::GET_DEVICE_INFO));
	CHECK(ResponseDecoder::Status::COMPLETE == feed_all(decoder, { 0xCC, 0x01, 0x77, 0x00, 0x02 }));
	DeviceInfo info;
	CHECK_TRUE(parse_device_info(decoder.frame(), decoder.length(), info));
	CHECK_EQUAL(1, info.protocol_version);
	CHECK_EQUAL(0x0077, info.features);
	CHECK_TRUE(info.features & feature::POWER_BUTTON);
	CHECK_TRUE(info.features & feature::START_RECORDING);
	CHECK_FALSE(info.features & feature::STOP_RECORDING);
	CHECK_FALSE(info.features & feature::FIVE_KEY_OSD);
	CHECK_FALSE(decoder.is_armed());
}

TEST(RunCamProtocol, SkipsNoiseBeforeTheHeader) {
	// A camera still booting, or a framing error, leaves bytes on the line.
	ResponseDecoder decoder;
	decoder.expect(Command::GET_DEVICE_INFO);
	CHECK(ResponseDecoder::Status::COMPLETE
	      == feed_all(decoder, { 0x00, 0xFF, 0x55, 0x0A, 0xCC, 0x01, 0x7F, 0x00, 0x5A }));
	CHECK_EQUAL(5U, decoder.length());
}

TEST(RunCamProtocol, HeaderByteInsideTheFrameIsData) {
	// features low byte 0xCC: counted, not taken for a new header.
	ResponseDecoder decoder;
	decoder.expect(Command::GET_DEVICE_INFO);
	CHECK(ResponseDecoder::Status::COMPLETE == feed_all(decoder, { 0xCC, 0x01, 0xCC, 0x00, 0xE9 }));
	DeviceInfo info;
	CHECK_TRUE(parse_device_info(decoder.frame(), decoder.length(), info));
	CHECK_EQUAL(0x00CC, info.features);
}

TEST(RunCamProtocol, LineFeedAndCarriageReturnAreData) {
	// What NrfUartAsync's line framing would have eaten: 0x0A / 0x0D inside a frame.
	ResponseDecoder decoder;
	decoder.expect(Command::GET_DEVICE_INFO);
	std::vector<uint8_t> f = { 0xCC, 0x0A, 0x0D, 0x00 };
	f.push_back(crc8_dvb_s2(f.data(), f.size()));
	CHECK(ResponseDecoder::Status::COMPLETE == feed_all(decoder, f));
}

TEST(RunCamProtocol, BadCrcIsCorrupt) {
	ResponseDecoder decoder;
	decoder.expect(Command::GET_DEVICE_INFO);
	CHECK(ResponseDecoder::Status::CORRUPT == feed_all(decoder, { 0xCC, 0x01, 0x77, 0x00, 0x03 }));
	CHECK_FALSE(decoder.is_armed());
}

TEST(RunCamProtocol, CameraControlIsNeverAnswered) {
	ResponseDecoder decoder;
	CHECK_FALSE(decoder.expect(Command::CAMERA_CONTROL));
	CHECK(ResponseDecoder::Status::IDLE == decoder.feed(0xCC));
}

TEST(RunCamProtocol, BytesWithNothingExpectedAreDropped) {
	// An unsolicited frame from the camera -- it may ask for an attitude (0x50).
	ResponseDecoder decoder;
	CHECK(ResponseDecoder::Status::IDLE == feed_all(decoder, { 0xCC, 0x50, 0x00 }));
	decoder.expect(Command::GET_DEVICE_INFO);
	CHECK(ResponseDecoder::Status::COMPLETE == feed_all(decoder, { 0xCC, 0x01, 0x77, 0x00, 0x02 }));
}

TEST(RunCamProtocol, FixedLengthAnswers) {
	ResponseDecoder decoder;
	decoder.expect(Command::KEY_PRESS);
	CHECK(ResponseDecoder::Status::COMPLETE == feed_all(decoder, { 0xCC, 0xA5 }));
	decoder.expect(Command::KEY_CONNECTION);
	CHECK(ResponseDecoder::Status::COMPLETE == feed_all(decoder, { 0xCC, 0x11, 0xE7 }));  // open, success
	std::vector<uint8_t> w = { 0xCC, 0x00, 0x01 };
	w.push_back(crc8_dvb_s2(w.data(), w.size()));
	decoder.expect(Command::WRITE_SETTING);
	CHECK(ResponseDecoder::Status::COMPLETE == feed_all(decoder, w));
	CHECK_EQUAL(4U, decoder.length());
}

TEST(RunCamProtocol, ChunkedSettingsAnswerCarriesItsLength) {
	// CC remaining=0 length=3 "abc" crc
	ResponseDecoder decoder;
	decoder.expect(Command::GET_SETTINGS);
	CHECK(ResponseDecoder::Status::COMPLETE == feed_all(decoder, { 0xCC, 0x00, 0x03, 'a', 'b', 'c', 0xDB }));
	CHECK_EQUAL(7U, decoder.length());
}

TEST(RunCamProtocol, ChunkedLengthThatCannotFitIsCorrupt) {
	ResponseDecoder decoder;
	decoder.expect(Command::READ_SETTING_DETAIL);
	CHECK(ResponseDecoder::Status::CORRUPT == feed_all(decoder, { 0xCC, 0x00, 61 }));  // 61 + 4 > 64
	CHECK_FALSE(decoder.is_armed());
}

TEST(RunCamProtocol, ParseDeviceInfoRejectsOtherFrames) {
	DeviceInfo info;
	const uint8_t short_frame[] = { 0xCC, 0xA5 };
	CHECK_FALSE(parse_device_info(short_frame, sizeof(short_frame), info));
	const uint8_t no_header[] = { 0x00, 0x01, 0x77, 0x00, 0x02 };
	CHECK_FALSE(parse_device_info(no_header, sizeof(no_header), info));
}

TEST(RunCamProtocol, DescribesFeatures) {
	char out[96];
	describe_features(0x0077, out, sizeof(out));
	STRCMP_EQUAL("POWER WIFI MODE SETTINGS DISPLAYPORT START", out);
	describe_features(0, out, sizeof(out));
	STRCMP_EQUAL("", out);
	char tiny[8];
	describe_features(0x0077, tiny, sizeof(tiny));
	STRCMP_EQUAL("POWER", tiny);  // truncated on a name boundary, still terminated
}

/*
 * The settings the driver reads and writes, on frames captured on the bench
 * from a Split 4 (2026-10-26): the requests are the bytes that went out, the
 * answers the bytes that came back, CRC included.
 */
TEST(RunCamProtocol, EncodesTheSettingsRequestsAsSentOnTheBench) {
	uint8_t out[MAX_FRAME];
	size_t len = encode_read_setting(setting::SDCARD_CAPACITY, out, sizeof(out));
	CHECK(std::vector<uint8_t>(out, out + len) == (std::vector<uint8_t>{ 0xCC, 0x11, 0x03, 0x00, 0x8D }));
	len = encode_write_text(setting::CAMERA_TIME, "20261026T211852.0", out, sizeof(out));
	CHECK(std::vector<uint8_t>(out, out + len)
	      == (std::vector<uint8_t>{ 0xCC, 0x13, 0x06, 0x32, 0x30, 0x32, 0x36, 0x31, 0x30, 0x32, 0x36,
	                                0x54, 0x32, 0x31, 0x31, 0x38, 0x35, 0x32, 0x2E, 0x30, 0x00, 0x02 }));
	char too_long[MAX_FRAME];
	memset(too_long, 'x', sizeof(too_long) - 1);
	too_long[sizeof(too_long) - 1] = '\0';
	CHECK_EQUAL(0U, encode_write_text(setting::CAMERA_TIME, too_long, out, sizeof(out)));
}

TEST(RunCamProtocol, ReadsTheBenchCardAnswers) {
	ResponseDecoder decoder;
	char text[24];
	unsigned int free_mb = 0, total_mb = 0;
	decoder.expect(Command::READ_SETTING_DETAIL);
	CHECK(ResponseDecoder::Status::COMPLETE
	      == feed_all(decoder, { 0xCC, 0x00, 0x0D, 0x0C, 0x31, 0x34, 0x36, 0x38, 0x33, 0x2F, 0x31, 0x35, 0x30, 0x37,
	                             0x39, 0x00, 0x04 }));
	CHECK_TRUE(parse_setting_text(decoder.frame(), decoder.length(), text, sizeof(text)));
	STRCMP_EQUAL("14683/15079", text);
	CHECK_TRUE(parse_sd_capacity(text, free_mb, total_mb));
	CHECK_EQUAL(14683U, free_mb);
	CHECK_EQUAL(15079U, total_mb);

	unsigned int remaining = 0;
	decoder.expect(Command::READ_SETTING_DETAIL);
	CHECK(ResponseDecoder::Status::COMPLETE
	      == feed_all(decoder, { 0xCC, 0x00, 0x06, 0x0C, 0x32, 0x36, 0x38, 0x31, 0x00, 0x37 }));
	CHECK_TRUE(parse_setting_text(decoder.frame(), decoder.length(), text, sizeof(text)));
	CHECK_TRUE(parse_uint(text, remaining));
	CHECK_EQUAL(2681U, remaining);
}

TEST(RunCamProtocol, ReadsTheCameraClockStringAndItsMaxLength) {
	// A STRING answer carries its maximum length after the terminator (0x14).
	ResponseDecoder decoder;
	char text[24];
	decoder.expect(Command::READ_SETTING_DETAIL);
	CHECK(ResponseDecoder::Status::COMPLETE
	      == feed_all(decoder, { 0xCC, 0x00, 0x14, 0x0A, 0x32, 0x30, 0x32, 0x31, 0x30, 0x39, 0x30, 0x37,
	                             0x54, 0x30, 0x31, 0x33, 0x37, 0x30, 0x30, 0x2E, 0x30, 0x00, 0x14, 0xAE }));
	CHECK_TRUE(parse_setting_text(decoder.frame(), decoder.length(), text, sizeof(text)));
	STRCMP_EQUAL("20210907T013700.0", text);
	char small[8];
	CHECK_FALSE(parse_setting_text(decoder.frame(), decoder.length(), small, sizeof(small)));  // never truncated
}

TEST(RunCamProtocol, SettingTextRefusesOtherTypesAndUnterminatedText) {
	char text[24];
	const uint8_t uint8_setting[] = { 0xCC, 0x00, 0x03, 0x00, 0x05, 0x00, 0xA6 };
	CHECK_FALSE(parse_setting_text(uint8_setting, sizeof(uint8_setting), text, sizeof(text)));
	const uint8_t unterminated[] = { 0xCC, 0x00, 0x03, 0x0C, 0x31, 0x32, 0x00 };
	CHECK_FALSE(parse_setting_text(unterminated, sizeof(unterminated), text, sizeof(text)));
	const uint8_t short_frame[] = { 0xCC, 0x00, 0x0D, 0x0C, 0x31, 0x00 };
	CHECK_FALSE(parse_setting_text(short_frame, sizeof(short_frame), text, sizeof(text)));
}

TEST(RunCamProtocol, NumbersAreWholeDecimalsThatFit) {
	unsigned int v = 0;
	CHECK_TRUE(parse_uint("0", v));
	CHECK_EQUAL(0U, v);
	CHECK_TRUE(parse_uint("4294967295", v));
	CHECK_EQUAL(4294967295U, v);
	CHECK_FALSE(parse_uint("4294967296", v));
	CHECK_FALSE(parse_uint("", v));
	CHECK_FALSE(parse_uint("12a", v));
	CHECK_FALSE(parse_uint("-1", v));
	unsigned int free_mb = 0, total_mb = 0;
	CHECK_FALSE(parse_sd_capacity("15049", free_mb, total_mb));
	CHECK_FALSE(parse_sd_capacity("/15079", free_mb, total_mb));
	CHECK_FALSE(parse_sd_capacity("15049/", free_mb, total_mb));
	CHECK_FALSE(parse_sd_capacity("a/b", free_mb, total_mb));
}

TEST(RunCamProtocol, WriteAnswerSaysWhetherTheValueWasTaken) {
	ResponseDecoder decoder;
	decoder.expect(Command::WRITE_SETTING);
	CHECK(ResponseDecoder::Status::COMPLETE == feed_all(decoder, { 0xCC, 0x00, 0x00, 0x39 }));  // the bench answer
	CHECK_TRUE(write_succeeded(decoder.frame(), decoder.length()));
	decoder.expect(Command::WRITE_SETTING);
	CHECK(ResponseDecoder::Status::COMPLETE == feed_all(decoder, { 0xCC, 0x01, 0x00, 0x32 }));
	CHECK_FALSE(write_succeeded(decoder.frame(), decoder.length()));
}

TEST(RunCamProtocol, ReadsTheBenchResolutionSelection) {
	// READ_SETTING_DETAIL 5 as the Split 4 answered it on the bench (2026-10-27).
	const std::vector<uint8_t> answer = {
		0xCC, 0x00, 0x2D, 0x09, 0x01, 0x31, 0x30, 0x38, 0x30, 0x50, 0x36, 0x30, 0x46, 0x50, 0x53, 0x3B, 0x31,
		0x30, 0x38, 0x30, 0x50, 0x35, 0x30, 0x46, 0x50, 0x53, 0x3B, 0x31, 0x30, 0x38, 0x30, 0x50, 0x33, 0x30,
		0x46, 0x50, 0x53, 0x3B, 0x37, 0x32, 0x30, 0x50, 0x36, 0x30, 0x46, 0x50, 0x53, 0x00, 0x53,
	};
	ResponseDecoder decoder;
	decoder.expect(Command::READ_SETTING_DETAIL);
	CHECK(ResponseDecoder::Status::COMPLETE == feed_all(decoder, answer));
	uint8_t value = 0;
	char options[MAX_FRAME];
	CHECK_TRUE(parse_setting_selection(decoder.frame(), decoder.length(), value, options, sizeof(options)));
	CHECK_EQUAL(1, value);
	STRCMP_EQUAL("1080P60FPS;1080P50FPS;1080P30FPS;720P60FPS", options);
	CHECK_EQUAL(0, selection_position(options, "1080P60FPS"));
	CHECK_EQUAL(3, selection_position(options, "720P60FPS"));
	CHECK_EQUAL(-1, selection_position(options, "4K30FPS"));
	CHECK_EQUAL(-1, selection_position(options, "720P60"));  // whole names only
	char label[16];
	CHECK_TRUE(selection_label(options, 2, label, sizeof(label)));
	STRCMP_EQUAL("1080P30FPS", label);
	CHECK_TRUE(selection_label(options, 3, label, sizeof(label)));
	STRCMP_EQUAL("720P60FPS", label);
	CHECK_FALSE(selection_label(options, 4, label, sizeof(label)));
	char small[6];
	CHECK_FALSE(selection_label(options, 0, small, sizeof(small)));  // never truncated
	// Not a TEXT_SELECTION: the free-space answer is refused.
	const uint8_t info[] = { 0xCC, 0x00, 0x06, 0x0C, 0x32, 0x36, 0x38, 0x31, 0x00, 0x37 };
	CHECK_FALSE(parse_setting_selection(info, sizeof(info), value, options, sizeof(options)));
}

TEST(RunCamProtocol, EncodesTheResolutionWritesAsSentOnTheBench) {
	uint8_t out[MAX_FRAME];
	size_t len = encode_write_u8(setting::RESOLUTION, 3, out, sizeof(out));
	CHECK(std::vector<uint8_t>(out, out + len) == (std::vector<uint8_t>{ 0xCC, 0x13, 0x05, 0x03, 0xCE }));
	len = encode_write_u8(setting::RESOLUTION, 1, out, sizeof(out));
	CHECK(std::vector<uint8_t>(out, out + len) == (std::vector<uint8_t>{ 0xCC, 0x13, 0x05, 0x01, 0xB1 }));
	len = encode_write_u8(setting::RESOLUTION, 2, out, sizeof(out));
	CHECK(std::vector<uint8_t>(out, out + len) == (std::vector<uint8_t>{ 0xCC, 0x13, 0x05, 0x02, 0x1B }));
}
