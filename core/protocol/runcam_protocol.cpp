/**
 * @file runcam_protocol.cpp
 * @brief RunCam Device Protocol v1.0 — see runcam_protocol.hpp.
 */

#include "runcam_protocol.hpp"

#include <cstdio>
#include <cstring>

namespace runcam {

uint8_t crc8_dvb_s2(uint8_t crc, uint8_t byte) {
	crc ^= byte;
	for (int i = 0; i < 8; i++) {
		crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0xD5) : static_cast<uint8_t>(crc << 1);
	}
	return crc;
}

uint8_t crc8_dvb_s2(const uint8_t *data, size_t len) {
	uint8_t crc = 0;
	for (size_t i = 0; i < len; i++)
		crc = crc8_dvb_s2(crc, data[i]);
	return crc;
}

size_t encode(Command command, const uint8_t *payload, size_t payload_len, uint8_t *out, size_t out_size) {
	const size_t length = payload_len + 3;
	if (length > out_size || length > MAX_FRAME) return 0;
	out[0] = HEADER;
	out[1] = static_cast<uint8_t>(command);
	if (payload_len) memcpy(&out[2], payload, payload_len);
	out[length - 1] = crc8_dvb_s2(out, length - 1);
	return length;
}

size_t encode_control(ControlAction action, uint8_t *out, size_t out_size) {
	const uint8_t payload = static_cast<uint8_t>(action);
	return encode(Command::CAMERA_CONTROL, &payload, 1, out, out_size);
}

size_t encode_read_setting(uint8_t setting_id, uint8_t *out, size_t out_size) {
	const uint8_t payload[2] = { setting_id, 0 };  // chunk 0: these answers fit in one
	return encode(Command::READ_SETTING_DETAIL, payload, sizeof(payload), out, out_size);
}

size_t encode_write_text(uint8_t setting_id, const char *text, uint8_t *out, size_t out_size) {
	uint8_t payload[MAX_FRAME];
	const size_t text_len = strlen(text);
	if (text_len + 2 > sizeof(payload)) return 0;
	payload[0] = setting_id;
	memcpy(&payload[1], text, text_len + 1);  // the NUL is part of the value
	return encode(Command::WRITE_SETTING, payload, text_len + 2, out, out_size);
}

bool parse_setting_text(const uint8_t *frame, size_t len, char *out, size_t out_size) {
	if (len < 6 || frame[0] != HEADER || out_size == 0) return false;
	const size_t data_len = frame[2];
	if (data_len + 4 != len || data_len < 2) return false;
	const uint8_t *data = &frame[3];
	if (data[0] != setting::TYPE_INFO && data[0] != setting::TYPE_STRING) return false;
	for (size_t i = 1; i < data_len; i++) {
		if (data[i] == 0) {
			if (i - 1 >= out_size) return false;
			memcpy(out, &data[1], i - 1);
			out[i - 1] = '\0';
			return true;
		}
	}
	return false;  // no terminator
}

bool parse_uint(const char *text, unsigned int &value) {
	if (!text || !*text) return false;
	uint64_t v = 0;
	for (const char *p = text; *p; p++) {
		if (*p < '0' || *p > '9') return false;
		v = v * 10 + static_cast<uint64_t>(*p - '0');
		if (v > 0xFFFFFFFFull) return false;
	}
	value = static_cast<unsigned int>(v);
	return true;
}

bool parse_sd_capacity(const char *text, unsigned int &free_mb, unsigned int &total_mb) {
	if (!text) return false;
	const char *slash = strchr(text, '/');
	if (!slash || slash == text) return false;
	char first[12];
	const size_t n = static_cast<size_t>(slash - text);
	if (n >= sizeof(first)) return false;
	memcpy(first, text, n);
	first[n] = '\0';
	return parse_uint(first, free_mb) && parse_uint(slash + 1, total_mb);
}

bool write_succeeded(const uint8_t *frame, size_t len) {
	return len == 4 && frame[0] == HEADER && frame[1] == 0;
}

size_t encode_write_u8(uint8_t setting_id, uint8_t value, uint8_t *out, size_t out_size) {
	const uint8_t payload[2] = { setting_id, value };
	return encode(Command::WRITE_SETTING, payload, sizeof(payload), out, out_size);
}

bool parse_setting_selection(const uint8_t *frame, size_t len, uint8_t &value, char *options, size_t options_size) {
	if (len < 7 || frame[0] != HEADER || options_size == 0) return false;
	const size_t data_len = frame[2];
	if (data_len + 4 != len || data_len < 3) return false;
	const uint8_t *data = &frame[3];
	if (data[0] != setting::TYPE_TEXT_SELECTION) return false;
	for (size_t i = 2; i < data_len; i++) {
		if (data[i] == 0) {
			const size_t n = i - 2;
			if (n >= options_size) return false;
			memcpy(options, &data[2], n);
			options[n] = '\0';
			value = data[1];
			return true;
		}
	}
	return false;  // no terminator
}

int selection_position(const char *options, const char *label) {
	if (!options || !label || !*label) return -1;
	const size_t label_len = strlen(label);
	int position = 0;
	for (const char *p = options;; position++) {
		const char *end = strchr(p, ';');
		const size_t n = end ? static_cast<size_t>(end - p) : strlen(p);
		if (n == label_len && memcmp(p, label, n) == 0) return position;
		if (!end) return -1;
		p = end + 1;
	}
}

bool selection_label(const char *options, unsigned int position, char *out, size_t out_size) {
	if (!options || out_size == 0) return false;
	const char *p = options;
	for (unsigned int i = 0; i < position; i++) {
		p = strchr(p, ';');
		if (!p) return false;
		p++;
	}
	const char *end = strchr(p, ';');
	const size_t n = end ? static_cast<size_t>(end - p) : strlen(p);
	if (n == 0 || n >= out_size) return false;
	memcpy(out, p, n);
	out[n] = '\0';
	return true;
}

bool parse_device_info(const uint8_t *frame, size_t len, DeviceInfo &out) {
	if (len != 5 || frame[0] != HEADER) return false;
	out.protocol_version = frame[1];
	out.features = static_cast<uint16_t>(frame[2] | (frame[3] << 8));
	return true;
}

void describe_features(uint16_t features, char *out, size_t out_size) {
	static const struct {
		uint16_t bit;
		const char *name;
	} names[] = {
		{ feature::POWER_BUTTON, "POWER" },       { feature::WIFI_BUTTON, "WIFI" },
		{ feature::CHANGE_MODE, "MODE" },         { feature::FIVE_KEY_OSD, "5KEY" },
		{ feature::SETTINGS_ACCESS, "SETTINGS" }, { feature::DISPLAY_PORT, "DISPLAYPORT" },
		{ feature::START_RECORDING, "START" },    { feature::STOP_RECORDING, "STOP" },
		{ feature::FC_ATTITUDE, "ATTITUDE" },
	};
	if (out_size == 0) return;
	out[0] = '\0';
	size_t used = 0;
	for (const auto &n : names) {
		if (!(features & n.bit)) continue;
		const size_t need = strlen(n.name) + (used ? 1 : 0);
		if (used + need >= out_size) return;  // stop on a name boundary rather than print half a name
		snprintf(out + used, out_size - used, "%s%s", used ? " " : "", n.name);
		used += need;
	}
}

size_t ResponseDecoder::answer_length(Command command) {
	switch (command) {
	case Command::GET_DEVICE_INFO: return 5;
	case Command::KEY_PRESS:
	case Command::KEY_RELEASE: return 2;
	case Command::KEY_CONNECTION: return 3;
	case Command::WRITE_SETTING: return 4;
	case Command::GET_SETTINGS:
	case Command::READ_SETTING_DETAIL: return CHUNKED;
	case Command::CAMERA_CONTROL: return 0;
	default: return 0;
	}
}

bool ResponseDecoder::expect(Command command) {
	m_expected = answer_length(command);
	m_command = command;
	m_length = 0;
	m_armed = (m_expected != 0);
	return m_armed;
}

ResponseDecoder::Status ResponseDecoder::feed(uint8_t byte) {
	if (!m_armed) return Status::IDLE;
	if (m_length == 0 && byte != HEADER) return Status::IN_PROGRESS;  // not synchronised yet
	m_frame[m_length++] = byte;

	size_t expected = m_expected;
	if (expected == CHUNKED) {
		// CC remaining_chunks data_length data[data_length] crc
		if (m_length < 3) return Status::IN_PROGRESS;
		expected = static_cast<size_t>(m_frame[2]) + 4;
		if (expected > MAX_FRAME) {
			m_armed = false;
			return Status::CORRUPT;
		}
	}
	if (m_length < expected) return Status::IN_PROGRESS;

	m_armed = false;
	return (crc8_dvb_s2(m_frame, m_length) == 0) ? Status::COMPLETE : Status::CORRUPT;
}

}  // namespace runcam
