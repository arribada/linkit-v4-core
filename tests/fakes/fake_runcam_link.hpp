#pragma once

#include <algorithm>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

#include "pmu.hpp"
#include "runcam_protocol.hpp"
#include "runcam_uart_device.hpp"

/// @brief RunCamLink double. Records the rail, the link and every frame sent,
/// in order, and plays the camera: bytes a test queues in `rx` are what the
/// device reads back, and `info_answer`, when set, is queued in reply to each
/// GET_DEVICE_INFO the way a booted camera would answer it.
///
/// With `camera` set it also plays a Split 4 as measured on the bench
/// (2026-10-26): POWER_BUTTON toggles the recording, START/STOP are ignored
/// unless told otherwise, the card fills while it records — from a moment
/// after the start to a moment after the stop — and its free space, remaining
/// time and clock are answered to READ_SETTING_DETAIL / WRITE_SETTING.
class FakeRunCamLink : public RunCamLink {
public:
	bool rail = false;
	bool link_open = false;
	bool open_fails = false;
	/// The next this-many CAMERA_CONTROL frames are refused, as a busy or dead UART would.
	unsigned int fail_next_controls = 0;
	std::vector<std::vector<uint8_t>> sent;
	std::deque<uint8_t> rx;
	std::vector<uint8_t> info_answer;
	/// Rail and link transitions in order: the TX-low-before-rail rule is read here.
	std::vector<std::string> events;

	/// @name The camera
	/// @{
	bool camera = false;
	bool settings_answer = true;    ///< false: settings advertised, never answered
	bool auto_record = false;       ///< starts recording with the rail
	bool honours_start = false;     ///< START_RECORDING (the Split 4 ignores it)
	bool honours_stop = false;      ///< STOP_RECORDING (not advertised by the Split 4)
	bool deaf = false;              ///< no press ever does anything
	unsigned int lost_presses = 0;  ///< the next this-many presses arrive and do nothing
	unsigned int lost_answers = 0;  ///< the next this-many settings answers never come back
	int lose_answer_to = -1;        ///< the next answer to this setting id never comes back
	double free_mb = 14707;
	unsigned int total_mb = 15079;
	double mb_per_s = 4.5;
	int remaining_s = -1;  ///< answered as is when >= 0, else derived from free_mb
	unsigned int start_delay_ms = 1500;
	unsigned int stop_delay_ms = 1000;
	bool recording = false;
	bool recording_at_cut = false;  ///< what the camera was doing when the rail went
	std::string clock;              ///< the last CAMERA_TIME written
	unsigned int clock_writes = 0;
	/// RESOLUTION (id 5): the option list, the value stored (read back, written)
	/// and the one in force, taken up from the stored one at each power-on — the
	/// bench Split 4's rule. Numbered from 0, as it numbers them; a value past
	/// the list is refused.
	std::string resolution_options = "1080P60FPS;1080P50FPS;1080P30FPS;720P60FPS";
	uint8_t resolution_stored = 0;
	uint8_t resolution_active = 0;
	bool refuses_resolution = false;  ///< answers a write with a failure
	bool forgets_resolution = false;  ///< accepts a write, keeps the old value
	unsigned int resolution_writes = 0;
	unsigned int rail_ons = 0;
	unsigned int cuts_while_recording = 0;  ///< rail cuts on a recording, whichever cut
	/// @}

	void set_rail(bool on) override {
		advance();
		if (on) {
			rail_ons++;
			resolution_active = resolution_stored;
		}
		if (on && auto_record) start_recording();
		if (!on) {
			recording_at_cut = recording;
			if (recording) cuts_while_recording++;
			recording = false;
			m_writing = false;
		}
		rail = on;
		events.push_back(on ? "rail_on" : "rail_off");
	}
	bool open() override {
		if (open_fails) return false;
		link_open = true;
		events.push_back("open");
		return true;
	}
	void close() override {
		link_open = false;
		events.push_back("close");
	}
	bool is_open() const override { return link_open; }
	bool send(const uint8_t *data, size_t len) override {
		if (!link_open) return false;
		if (len == 4 && data[1] == 0x01 && fail_next_controls) {
			fail_next_controls--;
			return false;
		}
		sent.emplace_back(data, data + len);
		if (!info_answer.empty() && len == 3 && data[1] == 0x00)
			rx.insert(rx.end(), info_answer.begin(), info_answer.end());
		if (camera) play(data, len);
		return true;
	}
	size_t receive(uint8_t *buf, size_t max) override {
		size_t n = 0;
		while (n < max && !rx.empty()) {
			buf[n++] = rx.front();
			rx.pop_front();
		}
		return n;
	}

	/// @brief How many GET_DEVICE_INFO requests went out.
	unsigned int info_requests() const {
		unsigned int n = 0;
		for (const auto &f : sent)
			if (f.size() == 3 && f[1] == 0x00) n++;
		return n;
	}
	/// @brief How many CAMERA_CONTROL frames with @p action went out.
	unsigned int controls(uint8_t action) const {
		unsigned int n = 0;
		for (const auto &f : sent)
			if (f.size() == 4 && f[1] == 0x01 && f[2] == action) n++;
		return n;
	}
	/// @brief Every CAMERA_CONTROL frame, whatever the action.
	unsigned int all_controls() const {
		unsigned int n = 0;
		for (const auto &f : sent)
			if (f.size() == 4 && f[1] == 0x01) n++;
		return n;
	}
	/// @brief How many settings requests (READ_SETTING_DETAIL or WRITE_SETTING) went out.
	unsigned int setting_requests() const {
		unsigned int n = 0;
		for (const auto &f : sent)
			if (f.size() >= 4 && (f[1] == 0x11 || f[1] == 0x13)) n++;
		return n;
	}
	/// @brief The camera's free space now, as it would answer it.
	unsigned int free_now() {
		advance();
		return (free_mb > 0) ? static_cast<unsigned int>(free_mb) : 0;
	}

private:
	bool m_writing = false;
	uint64_t m_write_from_ms = 0;
	uint64_t m_write_until_ms = 0;  ///< 0 while recording
	uint64_t m_last_ms = 0;

	/// @brief Bring the card up to now.
	void advance() {
		const uint64_t now = PMU::get_timestamp_ms();
		if (m_writing && now > m_last_ms) {
			const uint64_t from = (m_last_ms > m_write_from_ms) ? m_last_ms : m_write_from_ms;
			const uint64_t to = (m_write_until_ms && m_write_until_ms < now) ? m_write_until_ms : now;
			if (to > from) free_mb -= static_cast<double>(to - from) * mb_per_s / 1000.0;
			if (free_mb < 0) free_mb = 0;
			if (m_write_until_ms && now >= m_write_until_ms) m_writing = false;
		}
		m_last_ms = now;
	}
	unsigned int remaining() {
		if (remaining_s >= 0) return static_cast<unsigned int>(remaining_s);
		return static_cast<unsigned int>(free_now() / 5.6);  // 15079 MB = 2681 s on the bench card
	}
	void start_recording() {
		advance();
		if (recording || remaining() == 0) return;
		recording = true;
		m_writing = true;
		m_write_from_ms = PMU::get_timestamp_ms() + start_delay_ms;
		m_write_until_ms = 0;
	}
	void stop_recording() {
		advance();
		if (!recording) return;
		recording = false;
		m_write_until_ms = PMU::get_timestamp_ms() + stop_delay_ms;
	}
	void press(uint8_t action) {
		if (deaf) return;
		if (lost_presses) {
			lost_presses--;
			return;
		}
		if (action == 0x01) {
			if (recording) {
				stop_recording();
			} else {
				start_recording();
			}
		} else if (action == 0x03 && honours_start) {
			start_recording();
		} else if (action == 0x04 && honours_stop) {
			stop_recording();
		}
	}
	void answer(uint8_t type, const std::string &text, bool with_max) {
		std::vector<uint8_t> f = { 0xCC, 0x00, 0x00, type };
		f.insert(f.end(), text.begin(), text.end());
		f.push_back(0x00);
		if (with_max) f.push_back(0x14);
		f[2] = static_cast<uint8_t>(f.size() - 3);
		f.push_back(runcam::crc8_dvb_s2(f.data(), f.size()));
		rx.insert(rx.end(), f.begin(), f.end());
	}
	/// @brief A settings answer the test wants lost, consumed as it is asked for.
	bool loses(uint8_t setting_id) {
		if (lose_answer_to == setting_id) {
			lose_answer_to = -1;
			return true;
		}
		if (lost_answers) {
			lost_answers--;
			return true;
		}
		return false;
	}
	void answer_selection(uint8_t value, const std::string &options) {
		std::vector<uint8_t> f = { 0xCC, 0x00, 0x00, runcam::setting::TYPE_TEXT_SELECTION, value };
		f.insert(f.end(), options.begin(), options.end());
		f.push_back(0x00);
		f[2] = static_cast<uint8_t>(f.size() - 3);
		f.push_back(runcam::crc8_dvb_s2(f.data(), f.size()));
		rx.insert(rx.end(), f.begin(), f.end());
	}
	void play(const uint8_t *data, size_t len) {
		if (len == 4 && data[1] == 0x01) {
			press(data[2]);
		} else if (len == 5 && data[1] == 0x11 && settings_answer && !loses(data[2])) {
			if (data[2] == runcam::setting::SDCARD_CAPACITY) {
				answer(runcam::setting::TYPE_INFO, std::to_string(free_now()) + "/" + std::to_string(total_mb), false);
			} else if (data[2] == runcam::setting::REMAINING_TIME) {
				answer(runcam::setting::TYPE_INFO, std::to_string(remaining()), false);
			} else if (data[2] == runcam::setting::CAMERA_TIME) {
				answer(runcam::setting::TYPE_STRING, clock, true);
			} else if (data[2] == runcam::setting::RESOLUTION) {
				answer_selection(resolution_stored, resolution_options);
			}
		} else if (len == 5 && data[1] == 0x13 && data[2] == runcam::setting::RESOLUTION && settings_answer
		           && !loses(data[2])) {
			resolution_writes++;
			const bool in_list = data[3] < 1 + std::count(resolution_options.begin(), resolution_options.end(), ';');
			const bool refused_now = refuses_resolution || !in_list;
			if (!forgets_resolution && !refused_now) resolution_stored = data[3];
			const uint8_t ok[] = { 0xCC, 0x00, 0x00, 0x39 };
			const uint8_t refused[] = { 0xCC, 0x01, 0x00, 0x32 };
			if (refused_now) {
				rx.insert(rx.end(), refused, refused + sizeof(refused));
			} else {
				rx.insert(rx.end(), ok, ok + sizeof(ok));
			}
		} else if (len >= 5 && data[1] == 0x13 && data[2] == runcam::setting::CAMERA_TIME && settings_answer
		           && !loses(data[2])) {
			clock.assign(reinterpret_cast<const char *>(&data[3]),
			             strnlen(reinterpret_cast<const char *>(&data[3]), len - 4));
			clock_writes++;
			const uint8_t ok[] = { 0xCC, 0x00, 0x00, 0x39 };
			rx.insert(rx.end(), ok, ok + sizeof(ok));
		}
	}
};
