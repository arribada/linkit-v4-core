/**
 * @file runcam_uart_device.cpp
 * @brief RunCam camera over its UART — see runcam_uart_device.hpp.
 */

#include "runcam_uart_device.hpp"

#include <climits>
#include <cstdio>
#include <cstring>
#include <ctime>

#include "config_store.hpp"
#include "debug.hpp"
#include "pmu.hpp"
#include "rtc.hpp"

extern Scheduler *system_scheduler;
extern ConfigurationStore *configuration_store;
extern RTC *rtc;

using runcam::Command;
using runcam::ControlAction;
using runcam::ResponseDecoder;

/// @brief The camera's clock as it reads it back, "YYYYMMDDTHHMMSS.0".
static bool format_camera_time(std::time_t t, char *out, size_t out_size) {
	const std::tm *tm = std::gmtime(&t);
	if (!tm) return false;
	const int n = snprintf(out, out_size, "%04d%02d%02dT%02d%02d%02d.0", tm->tm_year + 1900, tm->tm_mon + 1,
	                       tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);
	return n > 0 && static_cast<size_t>(n) < out_size;
}

static int to_int(unsigned int v) {
	return (v > static_cast<unsigned int>(INT_MAX)) ? INT_MAX : static_cast<int>(v);
}

/// @brief -1 (never read this session) as the status keys' "unread".
static unsigned int unread_if_negative(int v) {
	return (v < 0) ? 0xFFFFFFFFU : static_cast<unsigned int>(v);
}

static const char *const RESOLUTION_LABELS[RunCamUartDevice::RESOLUTION_COUNT] = {
	"", "1080P60FPS", "1080P50FPS", "1080P30FPS", "720P60FPS",
};

const char *RunCamUartDevice::resolution_label(Resolution resolution) {
	const unsigned int i = static_cast<unsigned int>(resolution);
	return (i < RESOLUTION_COUNT) ? RESOLUTION_LABELS[i] : "";
}

RunCamUartDevice::Resolution RunCamUartDevice::resolution_of(const char *label) {
	for (unsigned int i = 1; i < RESOLUTION_COUNT; i++)
		if (strcmp(label, RESOLUTION_LABELS[i]) == 0) return static_cast<Resolution>(i);
	return Resolution::KEEP;
}

/// @brief CAP07, read at every power-on like CAP06. Never throws: a store that
/// cannot be read leaves the camera's resolution alone.
RunCamUartDevice::Resolution RunCamUartDevice::read_resolution_param() {
	try {
		if (configuration_store) {
			const unsigned int v = configuration_store->read_param<unsigned int>(ParamID::CAM_RESOLUTION);
			if (v < RESOLUTION_COUNT) return static_cast<Resolution>(v);
		}
	} catch (...) {}
	return Resolution::KEEP;
}

RunCamUartDevice::~RunCamUartDevice() {
	power_off_blocking();
}

/// @brief CAP06, read at every power-on so a PARMW takes effect at the next
/// session. Never throws: a store that cannot be read gives TOGGLE, the default.
RunCamUartDevice::RecordMode RunCamUartDevice::read_record_mode() {
	try {
		if (configuration_store) {
			const unsigned int mode = configuration_store->read_param<unsigned int>(ParamID::CAM_REC_MODE);
			if (mode <= static_cast<unsigned int>(RecordMode::COMMANDS)) return static_cast<RecordMode>(mode);
		}
	} catch (...) {}
	return RecordMode::TOGGLE;
}

/// @brief Power on: rail, then poll GET_DEVICE_INFO until the camera answers.
void RunCamUartDevice::power_on() {
	if (m_state != State::POWERED_OFF) return;  // already on, a sequence is running, or the bench holds it

	m_mode = read_record_mode();
	m_wanted = read_resolution_param();
	DEBUG_TRACE("RunCamUart: power_on mode=%u resolution=%u", static_cast<unsigned int>(m_mode),
	            static_cast<unsigned int>(m_wanted));
	m_state = State::BOOTING;
	m_step = Step::NONE;
	m_verify = Verify::START;
	m_resolution = Resolution::KEEP;
	m_restarted = false;
	m_restart_stop_sent = false;
	m_answered = false;
	m_recording = false;
	m_verified = false;
	m_stop_verified = false;
	m_cut_recording = false;
	m_clock_set = false;
	m_failure = Failure::NONE;
	m_stop_sent = false;
	m_stop_by_power = false;
	m_start_attempts = 0;
	m_stop_attempts = 0;
	m_free_mb = -1;
	m_total_mb = -1;
	m_remaining_s = -1;
	m_decoder.cancel();
	m_rail_on_ms = PMU::get_timestamp_ms();
	m_link.set_rail(true);
	m_task = system_scheduler->post_task_prio([this]() { boot_poll(); }, "RunCamUartBoot", Scheduler::DEFAULT_PRIORITY,
	                                          POLL_MS);
}

/// @brief One boot poll: flush what came in since the last one, then decide —
/// or ask again and read the answer REPLY_WAIT_MS later.
void RunCamUartDevice::boot_poll() {
	if (!m_link.is_open() && !m_link.open()) {
		boot_failed(Failure::LINK, "UART would not open");
		return;
	}
	drain();
	if (boot_decide()) return;
	request(Command::GET_DEVICE_INFO);
	m_task = system_scheduler->post_task_prio([this]() { boot_check(); }, "RunCamUartReply",
	                                          Scheduler::DEFAULT_PRIORITY, REPLY_WAIT_MS);
}

void RunCamUartDevice::boot_check() {
	drain();
	if (boot_decide()) return;
	m_task = system_scheduler->post_task_prio([this]() { boot_poll(); }, "RunCamUartBoot", Scheduler::DEFAULT_PRIORITY,
	                                          POLL_MS - REPLY_WAIT_MS);
}

/// @brief Start, wait for BOOT_MIN_MS, or give up. Returns true when it took
/// the next step itself, false when the caller keeps polling.
bool RunCamUartDevice::boot_decide() {
	const uint64_t now = PMU::get_timestamp_ms();
	const uint64_t elapsed = now - m_rail_on_ms;
	// AUTO: an auto-recording camera is recording from the end of its boot,
	// whether or not its answer reaches us — the TX line may be the broken one.
	// From then on a shutdown stops it rather than cut a file being written.
	if (m_mode == RecordMode::AUTO && !m_recording && (m_answered || elapsed >= BOOT_MIN_MS)) {
		m_recording = true;
		m_record_ms = now;
	}
	if (m_answered) {
		if (elapsed >= BOOT_MIN_MS) {
			prepare();
		} else {
			m_task = system_scheduler->post_task_prio([this]() { boot_poll(); }, "RunCamUartBoot",
			                                          Scheduler::DEFAULT_PRIORITY,
			                                          static_cast<unsigned int>(BOOT_MIN_MS - elapsed));
		}
		return true;
	}
	if (elapsed >= BOOT_TIMEOUT_MS) {
		boot_failed(Failure::NO_ANSWER, "no answer to GET_DEVICE_INFO -- camera absent, unpowered or miswired");
		return true;
	}
	return false;
}

/// @brief The camera is up. One that gives access to its settings has its card
/// checked and its clock set first; one that does not is started at once, the
/// sequence as it was before the card checks.
void RunCamUartDevice::prepare() {
	m_state = State::STARTING;
	if (has_settings()) {
		read_setting(runcam::setting::REMAINING_TIME, Step::CARD_CHECK);
	} else {
		press_start();
	}
}

/// @brief REMAINING_TIME is the recording time the camera itself still gives its
/// card. At 0 — full, missing or unusable — the session fails rather than log an
/// ON for a camera that films nothing. Unread, the start goes all the same.
void RunCamUartDevice::on_card_check() {
	unsigned int remaining = 0;
	if (!m_reply_ok || !runcam::parse_uint(m_setting_text, remaining)) {
		DEBUG_WARN("RunCamUart: remaining recording time unread, card unchecked");
		read_resolution();
		return;
	}
	m_remaining_s = to_int(remaining);
	if (remaining == 0) {
		m_recording = false;  // under AUTO too: a camera with no room left records nothing
		boot_failed(Failure::NO_SPACE, "no recording time left on the SD card");
		return;
	}
	read_resolution();
}

void RunCamUartDevice::read_resolution() {
	read_setting(runcam::setting::RESOLUTION, Step::RESOLUTION_READ);
}

/// @brief The camera's resolution, and CAP07 when it differs: written, then
/// taken up by a restart — the camera's own rule (bench, 2026-10-27). At most
/// one restart a session: a camera that reads back otherwise after it is left
/// as it is rather than restarted in a loop. Unread, nothing is changed.
void RunCamUartDevice::on_resolution_read() {
	m_resolution = Resolution::KEEP;
	if (!m_reply_ok) {
		set_clock();
		return;
	}
	char label[16];
	const int position = static_cast<int>(m_selection_value) - static_cast<int>(SELECTION_INDEX_BASE);
	if (position >= 0 && runcam::selection_label(m_options, static_cast<unsigned int>(position), label, sizeof(label)))
		m_resolution = resolution_of(label);
	if (m_wanted == Resolution::KEEP || m_wanted == m_resolution) {
		set_clock();
		return;
	}
	if (m_restarted) {
		DEBUG_WARN("RunCamUart: resolution still not %s after the restart, left as it is", resolution_label(m_wanted));
		set_clock();
		return;
	}
	const int wanted = runcam::selection_position(m_options, resolution_label(m_wanted));
	if (wanted < 0) {
		DEBUG_WARN("RunCamUart: the camera offers no %s, resolution left as it is", resolution_label(m_wanted));
		set_clock();
		return;
	}
	uint8_t frame[6];
	const size_t len = runcam::encode_write_u8(runcam::setting::RESOLUTION,
	                                           static_cast<uint8_t>(wanted + static_cast<int>(SELECTION_INDEX_BASE)),
	                                           frame, sizeof(frame));
	exchange(Command::WRITE_SETTING, frame, len, Step::RESOLUTION_WRITE);
}

/// @brief Written: restart the camera to take it up. An auto-recording camera is
/// already filming by now (the bench Split 4 is), and the rail cut would end
/// that file without its index: the card says first.
void RunCamUartDevice::on_resolution_written() {
	if (!m_reply_ok) {
		DEBUG_WARN("RunCamUart: the camera refused resolution %s", resolution_label(m_wanted));
		set_clock();
		return;
	}
	DEBUG_INFO("RunCamUart: resolution %s written, restarting the camera to take it up", resolution_label(m_wanted));
	m_restart_stop_sent = false;
	start_verify(Verify::RESTART);
}

/// @brief Recording on its own: one stop, checked, then the restart. Whatever the
/// card says the second time the camera is restarted — once.
void RunCamUartDevice::on_restart_measured(bool filling) {
	if (filling && !m_restart_stop_sent) {
		m_restart_stop_sent = true;
		m_recording = true;      // measured: it records on its own
		m_stop_by_power = true;  // whatever CAP06 says, POWER_BUTTON is what stops it
		if (send_stop()) {
			start_verify(Verify::RESTART);
			return;
		}
	}
	if (filling) DEBUG_WARN("RunCamUart: camera still recording, restarted all the same");
	restart_camera();
}

/// @brief Rail off RESTART_OFF_MS (link down, TX low, first), then a new boot:
/// the session carries on from there — card, resolution, clock, start.
void RunCamUartDevice::restart_camera() {
	m_restarted = true;
	cut_power();
	m_state = State::BOOTING;
	m_answered = false;
	m_task = system_scheduler->post_task_prio([this]() { power_back_on(); }, "RunCamUartRestart",
	                                          Scheduler::DEFAULT_PRIORITY, RESTART_OFF_MS);
}

void RunCamUartDevice::power_back_on() {
	m_rail_on_ms = PMU::get_timestamp_ms();
	m_link.set_rail(true);
	m_task = system_scheduler->post_task_prio([this]() { boot_poll(); }, "RunCamUartBoot", Scheduler::DEFAULT_PRIORITY,
	                                          POLL_MS);
}

/// @brief Stamp the session's files in UTC: the RTC's time, less the offset the
/// camera adds back when it reads its clock. Not while the RTC is unset — the
/// camera keeps its own clock rather than take 1970 — nor under AUTO, where the
/// file is already open by the time the camera answers.
void RunCamUartDevice::set_clock() {
	char text[32];
	if (m_mode == RecordMode::AUTO || !rtc || !rtc->is_set()
	    || !format_camera_time(rtc->gettime() - CAMERA_CLOCK_OFFSET_S, text, sizeof(text))) {
		press_start();
		return;
	}
	uint8_t frame[runcam::MAX_FRAME];
	const size_t len = runcam::encode_write_text(runcam::setting::CAMERA_TIME, text, frame, sizeof(frame));
	exchange(Command::WRITE_SETTING, frame, len, Step::CLOCK_SET);
}

void RunCamUartDevice::on_clock_set() {
	m_clock_set = m_reply_ok;
	if (!m_clock_set) DEBUG_WARN("RunCamUart: camera clock not set, its files keep the camera's own time");
	press_start();
}

/// @brief The start, or a repeat after the card said the camera is not
/// recording. A repeat is POWER_BUTTON whatever CAP06 says: the camera was just
/// measured in standby, and that is the press the Split 4 honours.
void RunCamUartDevice::press_start() {
	const bool repeat = (m_start_attempts > 0);
	m_start_attempts++;
	bool sent = true;
	if (repeat || m_mode == RecordMode::TOGGLE) {
		warn_if_unsupported(runcam::feature::POWER_BUTTON, "POWER_BUTTON");
		sent = control(ControlAction::POWER_BUTTON);
	} else if (m_mode == RecordMode::COMMANDS) {
		warn_if_unsupported(runcam::feature::START_RECORDING, "START_RECORDING");
		sent = control(ControlAction::START_RECORDING);
	}
	// AUTO, first attempt: recording since the end of its own boot (boot_decide).
	if (!sent) {
		// The camera never got the start: it is in standby, nothing to stop.
		// Believing it recording would make the stop press START a file the rail
		// cut then corrupts.
		boot_failed(Failure::LINK, "start command not sent");
		return;
	}
	if (repeat || m_mode != RecordMode::AUTO) {
		m_recording = true;
		m_record_ms = PMU::get_timestamp_ms();
	}
	if (has_settings()) {
		start_verify(Verify::START);
		return;
	}
	m_task = system_scheduler->post_task_prio([this]() { finish_power_on(); }, "RunCamUartStarted",
	                                          Scheduler::DEFAULT_PRIORITY,
	                                          (m_mode == RecordMode::AUTO) ? 0 : RECORD_SETTLE_MS);
}

void RunCamUartDevice::finish_power_on() {
	m_state = State::POWERED_ON;
	m_num_captures++;
	if (m_verified)
		DEBUG_INFO("RunCamUart: recording (seen on the card after %u start%s, %d MB free)", m_start_attempts,
		           (m_start_attempts > 1) ? "s" : "", m_free_mb);
	notify_power_on();
}

/// @brief Power off: stop the recording, let the camera close its file, cut the rail.
void RunCamUartDevice::power_off() {
	if (m_state != State::POWERED_ON) return;  // already off, or a sequence is running

	DEBUG_TRACE("RunCamUart: power_off");
	m_state = State::STOPPING;
	m_stop_attempts = 0;
	press_stop();
}

/// @brief One stop, then the card read — or, blind, the stop's settle.
void RunCamUartDevice::press_stop() {
	if (!send_stop()) {
		DEBUG_WARN("RunCamUart: stop not sent (attempt %u), retrying in %u ms", m_stop_attempts + 1, STOP_SETTLE_MS);
		m_task = system_scheduler->post_task_prio([this]() { finish_power_off(); }, "RunCamUartStopped",
		                                          Scheduler::DEFAULT_PRIORITY, STOP_SETTLE_MS);
		return;
	}
	if (has_settings()) {
		start_verify(Verify::STOP);
		return;
	}
	m_task = system_scheduler->post_task_prio([this]() { finish_power_off(); }, "RunCamUartStopped",
	                                          Scheduler::DEFAULT_PRIORITY, STOP_SETTLE_MS);
}

void RunCamUartDevice::finish_power_off() {
	// A stop that did not leave: the camera is still recording, and cutting
	// now corrupts its file. Try again before giving up on it.
	if (m_recording && m_stop_attempts < STOP_RETRIES) {
		m_stop_attempts++;
		press_stop();
		return;
	}
	if (m_recording) {
		DEBUG_WARN("RunCamUart: camera still recording after every stop, rail cut on a recording");
		m_cut_recording = true;
	}
	cut_power();
	m_state = State::POWERED_OFF;
	publish_status();
	notify_power_off();
}

/// @brief Two free-space reads, VERIFY_FIRST_MS after the press and VERIFY_GAP_MS apart.
void RunCamUartDevice::start_verify(Verify purpose) {
	m_verify = purpose;
	m_free_first_mb = -1;
	m_task = system_scheduler->post_task_prio(
	    [this]() { read_setting(runcam::setting::SDCARD_CAPACITY, Step::VERIFY_FIRST); }, "RunCamUartVerify",
	    Scheduler::DEFAULT_PRIORITY, VERIFY_FIRST_MS);
}

void RunCamUartDevice::on_verify_first() {
	if (!read_free_mb(m_free_first_mb)) {
		verify_unavailable();
		return;
	}
	m_task = system_scheduler->post_task_prio(
	    [this]() { read_setting(runcam::setting::SDCARD_CAPACITY, Step::VERIFY_SECOND); }, "RunCamUartVerify",
	    Scheduler::DEFAULT_PRIORITY, VERIFY_GAP_MS);
}

void RunCamUartDevice::on_verify_second() {
	int free_mb;
	if (!read_free_mb(free_mb)) {
		verify_unavailable();
		return;
	}
	const bool filling = (m_free_first_mb - free_mb) >= static_cast<int>(VERIFY_MIN_DROP_MB);
	switch (m_verify) {
	case Verify::START: on_start_measured(filling); break;
	case Verify::STOP: on_stop_measured(filling); break;
	case Verify::RESTART: on_restart_measured(filling); break;
	default: break;
	}
}

void RunCamUartDevice::on_start_measured(bool filling) {
	if (filling) {
		m_recording = true;
		m_verified = true;
		// Started by a POWER_BUTTON repeat: under COMMANDS the camera has just
		// shown it ignores START, so it is stopped with POWER_BUTTON too -- on the
		// blocking path above all, which has no card to check a STOP against.
		if (m_start_attempts > 1) m_stop_by_power = true;
		finish_power_on();
		return;
	}
	m_recording = false;  // measured: the camera is in standby
	if (m_start_attempts < START_ATTEMPTS) {
		DEBUG_WARN("RunCamUart: card not filling after start %u (CAP06=%u), pressing POWER", m_start_attempts,
		           static_cast<unsigned int>(m_mode));
		press_start();
		return;
	}
	boot_failed(Failure::NOT_RECORDING, "camera does not record, its SD card is not filling");
}

void RunCamUartDevice::on_stop_measured(bool filling) {
	if (!filling) {
		m_stop_verified = true;
		finish_power_off();  // m_recording is false since the stop left; the settle is long past
		return;
	}
	m_recording = true;  // measured: the camera missed the stop
	m_stop_by_power = true;
	if (m_stop_attempts < STOP_RETRIES) {
		m_stop_attempts++;
		DEBUG_WARN("RunCamUart: card still filling after stop %u, pressing POWER", m_stop_attempts);
		press_stop();
		return;
	}
	finish_power_off();
}

/// @brief The card could not be read: the press is believed, as without
/// settings access. Never repeated on a guess — a repeat is a toggle.
void RunCamUartDevice::verify_unavailable() {
	DEBUG_WARN("RunCamUart: SD capacity unread, %s unverified", (m_verify == Verify::START)  ? "start"
	                                                            : (m_verify == Verify::STOP) ? "stop"
	                                                                                         : "restart");
	if (m_verify == Verify::START) {
		finish_power_on();  // VERIFY_FIRST_MS is past RECORD_SETTLE_MS
		return;
	}
	if (m_verify == Verify::RESTART) {
		restart_camera();  // nothing tells it is recording: what was said last stands
		return;
	}
	const uint64_t settled = m_stop_ms + STOP_SETTLE_MS;
	const uint64_t now = PMU::get_timestamp_ms();
	m_task = system_scheduler->post_task_prio([this]() { finish_power_off(); }, "RunCamUartStopped",
	                                          Scheduler::DEFAULT_PRIORITY,
	                                          (settled > now) ? static_cast<unsigned int>(settled - now) : 0);
}

/// @brief Blocking power-off for the shutdown path (see CAMDevice).
///
/// Whatever step the asynchronous sequence had reached, the camera must have
/// closed its file before the rail goes. Unlike the button there is no "clean
/// edge" to restart from: a press is a toggle, so a second stop after the one
/// already sent would START a new recording, which the rail cut then corrupts.
/// What the card said last is in m_recording; nothing new is read here.
void RunCamUartDevice::power_off_blocking() {
	if (system_scheduler) system_scheduler->cancel_task(m_task);
	m_step = Step::NONE;
	if (m_state == State::POWERED_OFF) return;

	DEBUG_TRACE("RunCamUart: power_off (blocking) from %s", state_name());
	const bool session = (m_state != State::BENCH);  // a bench hold is not a session to report
	if (m_recording) {
		// A start just sent goes through first: a stop that lands while the
		// camera is still opening its file is lost.
		wait_until(m_record_ms + RECORD_SETTLE_MS);
		if (!send_stop()) {
			PMU::delay_ms(STOP_RETRY_MS);
			send_stop();
		}
	}
	if (m_stop_sent) wait_until(m_stop_ms + STOP_SETTLE_MS);
	cut_power();
	m_state = State::POWERED_OFF;
	if (session) publish_status();
	notify_power_off();
}

bool RunCamUartDevice::is_powered_on() {
	return m_state == State::POWERED_ON;
}

unsigned int RunCamUartDevice::get_num_captures() {
	return m_num_captures;
}

/// @brief The session cannot go on. A camera that never answered, never got
/// its start or was measured in standby can have its rail cut at once. Under
/// AUTO, unmeasured, it may be recording all the same, so it is stopped first
/// — blind, since its answer never came, but it may only be its TX that is lost.
void RunCamUartDevice::boot_failed(Failure failure, const char *why) {
	m_failure = failure;
	DEBUG_WARN("RunCamUart: %s, after %u ms (%u corrupt answers)", why,
	           static_cast<unsigned int>(PMU::get_timestamp_ms() - m_rail_on_ms), m_corrupt);
	m_stop_attempts = 0;
	stop_before_failing();
}

/// @brief The blind stop of boot_failed(), tried again STOP_RETRIES times,
/// STOP_RETRY_MS apart, when it does not leave: the rail is only cut on a
/// recording once every attempt has failed, and that is then recorded.
void RunCamUartDevice::stop_before_failing() {
	if (!m_recording) {
		finish_failed();
		return;
	}
	if (send_stop()) {
		m_state = State::STOPPING;
		m_task = system_scheduler->post_task_prio([this]() { finish_failed(); }, "RunCamUartFailed",
		                                          Scheduler::DEFAULT_PRIORITY, STOP_SETTLE_MS);
		return;
	}
	if (m_stop_attempts < STOP_RETRIES) {
		m_stop_attempts++;
		m_task = system_scheduler->post_task_prio([this]() { stop_before_failing(); }, "RunCamUartFailed",
		                                          Scheduler::DEFAULT_PRIORITY, STOP_RETRY_MS);
		return;
	}
	DEBUG_WARN("RunCamUart: blind stop never sent, rail cut on a recording");
	m_cut_recording = true;
	finish_failed();
}

/// @brief The listener (CAMService) closes the session on the error.
void RunCamUartDevice::finish_failed() {
	cut_power();
	m_state = State::POWERED_OFF;
	publish_status();
	notify_error();
}

RunCamUartDevice::Outcome RunCamUartDevice::outcome() const {
	switch (m_failure) {
	case Failure::NO_ANSWER: return Outcome::NO_ANSWER;
	case Failure::LINK: return Outcome::LINK_ERROR;
	case Failure::NO_SPACE: return Outcome::NO_SPACE;
	case Failure::NOT_RECORDING: return Outcome::NOT_RECORDING;
	case Failure::NONE: break;
	default: break;
	}
	if (m_cut_recording) return Outcome::STOP_FAILED;
	if (!m_verified || !m_stop_verified) return Outcome::UNVERIFIED;
	return Outcome::OK;
}

/// @brief CAT01-CAT06, what the session found, for STATR: in RAM, persisted by
/// the store's periodic flush. A diagnostic mirror the session does not depend
/// on, so nothing here may escape.
void RunCamUartDevice::publish_status() {
	if (!configuration_store) return;
	try {
		configuration_store->write_param(ParamID::CAM_LAST_OUTCOME, static_cast<unsigned int>(outcome()));
		configuration_store->write_param(ParamID::CAM_SD_FREE_MB, unread_if_negative(m_free_mb));
		configuration_store->write_param(ParamID::CAM_SD_TOTAL_MB, unread_if_negative(m_total_mb));
		configuration_store->write_param(ParamID::CAM_REC_TIME_LEFT_S, unread_if_negative(m_remaining_s));
		configuration_store->write_param(ParamID::CAM_RESOLUTION_READ, static_cast<unsigned int>(m_resolution));
		configuration_store->write_param(ParamID::CAM_FEATURES,
		                                 m_answered ? static_cast<unsigned int>(m_info.features) : 0U);
	} catch (...) {
		DEBUG_WARN("RunCamUart: could not publish the camera status");
	}
}

/// @brief Stop the recording the camera is believed to make. The belief only
/// changes when the press really left. @return false if it did not.
bool RunCamUartDevice::send_stop() {
	if (!m_recording) return true;
	bool sent;
	if (m_mode == RecordMode::COMMANDS && !m_stop_by_power) {
		warn_if_unsupported(runcam::feature::STOP_RECORDING, "STOP_RECORDING");
		sent = control(ControlAction::STOP_RECORDING);
	} else {
		sent = control(ControlAction::POWER_BUTTON);
	}
	if (!sent) return false;
	m_recording = false;
	m_stop_sent = true;
	m_stop_ms = PMU::get_timestamp_ms();
	return true;
}

/// @brief A camera that answered without advertising what is about to be sent.
/// The command still goes: CAP06 is the operator's choice, and the Split 4 is
/// reported to advertise START while handling it badly — the bench decides.
void RunCamUartDevice::warn_if_unsupported(uint16_t needed, const char *what) {
	if (m_answered && !(m_info.features & needed))
		DEBUG_WARN("RunCamUart: %s not advertised (features 0x%04X), sent anyway", what,
		           static_cast<unsigned int>(m_info.features));
}

bool RunCamUartDevice::has_settings() const {
	return m_answered && (m_info.features & runcam::feature::SETTINGS_ACCESS);
}

void RunCamUartDevice::read_setting(uint8_t setting_id, Step step) {
	uint8_t frame[5];
	const size_t len = runcam::encode_read_setting(setting_id, frame, sizeof(frame));
	exchange(Command::READ_SETTING_DETAIL, frame, len, step);
}

/// @brief Send a settings request; on_step() looks for its answer every
/// SETTING_POLL_MS and moves on when it is in, or once the request has gone
/// unanswered twice.
void RunCamUartDevice::exchange(Command command, const uint8_t *frame, size_t len, Step step) {
	m_step = step;
	m_request_command = command;
	m_request_len = (len <= sizeof(m_request)) ? len : 0;
	if (m_request_len) memcpy(m_request, frame, m_request_len);
	m_request_repeated = false;
	send_request();
}

void RunCamUartDevice::send_request() {
	drain();  // nothing received before the request is its answer
	m_replied = false;
	m_reply_ok = false;
	m_answer_complete = false;
	m_setting_text[0] = '\0';
	m_request_ms = PMU::get_timestamp_ms();
	m_decoder.expect(m_request_command);
	if (!m_request_len || !send_frame(m_request, m_request_len)) {
		m_decoder.cancel();
		m_replied = true;  // nothing is coming: the step goes on without it
	}
	m_task = system_scheduler->post_task_prio([this]() { on_step(); }, "RunCamUartStep", Scheduler::DEFAULT_PRIORITY,
	                                          SETTING_POLL_MS);
}

void RunCamUartDevice::on_step() {
	drain();
	if (!m_replied && PMU::get_timestamp_ms() - m_request_ms < SETTING_TIMEOUT_MS) {
		m_task = system_scheduler->post_task_prio([this]() { on_step(); }, "RunCamUartStep",
		                                          Scheduler::DEFAULT_PRIORITY, SETTING_POLL_MS);
		return;
	}
	// Lost or corrupt, the request goes once more; an answer that came whole is
	// final, a refusal included — the camera would only say it again.
	if (!m_answer_complete && !m_request_repeated) {
		m_request_repeated = true;  // a settings request changes nothing it has not already: safe to repeat
		send_request();
		return;
	}
	m_decoder.cancel();  // an answer later than this is dropped, not taken for the next one's
	const Step step = m_step;
	m_step = Step::NONE;
	switch (step) {
	case Step::CARD_CHECK: on_card_check(); break;
	case Step::RESOLUTION_READ: on_resolution_read(); break;
	case Step::RESOLUTION_WRITE: on_resolution_written(); break;
	case Step::CLOCK_SET: on_clock_set(); break;
	case Step::VERIFY_FIRST: on_verify_first(); break;
	case Step::VERIFY_SECOND: on_verify_second(); break;
	case Step::NONE:
	default: break;
	}
}

/// @brief The free space of the SDCARD_CAPACITY answer just in, "free/total" MB.
bool RunCamUartDevice::read_free_mb(int &free_mb) {
	unsigned int free_now = 0, total = 0;
	if (!m_reply_ok || !runcam::parse_sd_capacity(m_setting_text, free_now, total)) return false;
	free_mb = m_free_mb = to_int(free_now);
	m_total_mb = to_int(total);
	return true;
}

bool RunCamUartDevice::request(Command command) {
	uint8_t frame[3];
	const size_t len = runcam::encode(command, nullptr, 0, frame, sizeof(frame));
	m_decoder.expect(command);
	return send_frame(frame, len);
}

bool RunCamUartDevice::control(ControlAction action) {
	uint8_t frame[4];
	const size_t len = runcam::encode_control(action, frame, sizeof(frame));
	return send_frame(frame, len);
}

bool RunCamUartDevice::send_frame(const uint8_t *frame, size_t len) {
	if (!m_link.is_open() && !m_link.open()) {
		DEBUG_WARN("RunCamUart: link down, command 0x%02X not sent", frame[1]);
		return false;
	}
	if (m_link.send(frame, len)) return true;
	DEBUG_WARN("RunCamUart: command 0x%02X not sent", frame[1]);
	return false;
}

void RunCamUartDevice::drain() {
	uint8_t buf[32];
	size_t n;
	while ((n = m_link.receive(buf, sizeof(buf))) > 0) {
		for (size_t i = 0; i < n; i++) {
			switch (m_decoder.feed(buf[i])) {
			case ResponseDecoder::Status::COMPLETE: on_answer(); break;
			case ResponseDecoder::Status::CORRUPT:
				m_corrupt++;
				if (m_step != Step::NONE) m_replied = true;  // answered, unusably: no point waiting
				break;
			case ResponseDecoder::Status::IDLE:
			case ResponseDecoder::Status::IN_PROGRESS: break;
			default: break;
			}
		}
	}
}

void RunCamUartDevice::on_answer() {
	const uint8_t *frame = m_decoder.frame();
	const size_t len = m_decoder.length();
	const Command command = m_decoder.command();
	if (command == Command::READ_SETTING_DETAIL) {
		m_replied = true;
		m_answer_complete = true;
		m_reply_ok = (m_step == Step::RESOLUTION_READ)
		                 ? runcam::parse_setting_selection(frame, len, m_selection_value, m_options, sizeof(m_options))
		                 : runcam::parse_setting_text(frame, len, m_setting_text, sizeof(m_setting_text));
		return;
	}
	if (command == Command::WRITE_SETTING) {
		m_replied = true;
		m_answer_complete = true;
		m_reply_ok = runcam::write_succeeded(frame, len);
		return;
	}
	if (command != Command::GET_DEVICE_INFO) return;

	runcam::DeviceInfo info;
	if (!runcam::parse_device_info(frame, len, info)) return;
	m_info = info;
	if (m_answered) return;
	m_answered = true;

	char names[80];
	runcam::describe_features(info.features, names, sizeof(names));
	DEBUG_INFO("RunCamUart: answered after %u ms, protocol %u, features 0x%04X (%s)",
	           static_cast<unsigned int>(PMU::get_timestamp_ms() - m_rail_on_ms),
	           static_cast<unsigned int>(info.protocol_version), static_cast<unsigned int>(info.features), names);
}

/// @brief Link down with TX driven low, THEN the rail: never the other way round.
void RunCamUartDevice::cut_power() {
	m_decoder.cancel();
	m_link.close();
	m_link.set_rail(false);
	m_recording = false;
}

void RunCamUartDevice::wait_until(uint64_t deadline_ms) {
	const uint64_t now = PMU::get_timestamp_ms();
	if (deadline_ms > now) PMU::delay_ms(static_cast<unsigned int>(deadline_ms - now));
}

const char *RunCamUartDevice::state_name() const {
	switch (m_state) {
	case State::POWERED_OFF: return "OFF";
	case State::BOOTING: return "BOOTING";
	case State::STARTING: return "STARTING";
	case State::POWERED_ON: return "ON";
	case State::STOPPING: return "STOPPING";
	case State::BENCH: return "BENCH";
	default: return "?";
	}
}

bool RunCamUartDevice::bench_power(bool on) {
	if (on) {
		if (m_state != State::POWERED_OFF) return false;
		m_state = State::BENCH;
		m_answered = false;
		m_recording = false;
		m_verified = false;
		m_stop_verified = false;
		m_cut_recording = false;
		m_clock_set = false;
		m_failure = Failure::NONE;
		m_resolution = Resolution::KEEP;
		m_restarted = false;
		m_stop_sent = false;
		m_free_mb = -1;
		m_total_mb = -1;
		m_remaining_s = -1;
		m_rail_on_ms = PMU::get_timestamp_ms();
		m_link.set_rail(true);
		return true;
	}
	if (m_state != State::BENCH) return false;
	cut_power();
	m_state = State::POWERED_OFF;
	return true;
}

int RunCamUartDevice::bench_exchange(const uint8_t *frame, size_t len, uint8_t *out, size_t out_size,
                                     unsigned int window_ms) {
	if (m_state != State::POWERED_ON && m_state != State::BENCH) return -1;
	if (!m_link.is_open() && !m_link.open()) return -1;

	uint8_t stale[32];
	while (m_link.receive(stale, sizeof(stale)) > 0) {}
	if (!m_link.send(frame, len)) return -1;

	size_t got = 0;
	const uint64_t deadline = PMU::get_timestamp_ms() + window_ms;
	while (got < out_size && PMU::get_timestamp_ms() < deadline) {
		PMU::delay_ms(5);
		got += m_link.receive(out + got, out_size - got);
	}
	return static_cast<int>(got);
}

int RunCamUartDevice::bench_receive(uint8_t *out, size_t out_size) {
	if ((m_state != State::POWERED_ON && m_state != State::BENCH) || !m_link.is_open()) return -1;
	size_t got = 0, n;
	while (got < out_size && (n = m_link.receive(out + got, out_size - got)) > 0)
		got += n;
	return static_cast<int>(got);
}

/// @brief Deliver the completion events from inside a scheduler task.
///
/// Same barrier as the button driver: the listener (CAMService) reschedules
/// itself in react(), that path reads the config store, which can throw, and
/// we are outside Service::run_scheduled_task()'s try/catch. The device state is
/// already final here; the listener owns its own recovery.
void RunCamUartDevice::notify_power_on() {
	try {
		notify<CAMEventPowerOn>({});
	} catch (...) {
		DEBUG_ERROR("RunCamUart: exception in power-on listener — ignored");
	}
}

void RunCamUartDevice::notify_power_off() {
	try {
		notify<CAMEventPowerOff>({});
	} catch (...) {
		DEBUG_ERROR("RunCamUart: exception in power-off listener — ignored");
	}
}

void RunCamUartDevice::notify_error() {
	try {
		notify<CAMEventError>({});
	} catch (...) {
		DEBUG_ERROR("RunCamUart: exception in error listener — ignored");
	}
}
