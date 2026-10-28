/**
 * @file runcam_uart_device.hpp
 * @brief RunCam camera driven over its UART (RunCam Device Protocol) — portable logic.
 *
 * The CAM_CONTROL=UART counterpart of the button driver (ports/nrf52840/core/
 * hardware/run_cam/runcam.cpp), behind the same asynchronous CAMDevice contract.
 * The rail stays the one way to power the camera: the protocol presses buttons
 * but cannot hold them, and power on/off is a long press. What goes over the
 * UART is the recording, as CAM_REC_MODE (CAP06) says.
 *
 * On the Split 4 recording is a TOGGLE, exactly as with the button, and the
 * camera never answers a CAMERA_CONTROL. What it does answer is the state of
 * its card, refreshed live: its free space drops every second while it records
 * and not at all once it has stopped (bench, 2026-10-26: 4-5 MB/s at 1080p60,
 * writing visible 1 to 3.5 s after the start, settled 1.4 s after the stop).
 * So when the camera advertises settings access, every press is checked
 * against its card:
 *
 *  - Before the start, a card the camera itself gives no recording time to
 *    (full, missing, unusable) fails the session instead of logging an ON for
 *    a camera that films nothing.
 *  - After a start, two free-space reads VERIFY_GAP_MS apart must show the card
 *    filling. If not, POWER_BUTTON is pressed once more — which also puts right
 *    a camera whose auto-record disagrees with CAP06 — then the session fails.
 *  - After a stop, the card must have stopped filling before the rail is cut;
 *    a stop the camera missed is pressed again rather than turned into a file
 *    the rail cut corrupts.
 *
 * A camera that does not advertise settings access gets the sequence as it was
 * before these checks. One that advertises it but leaves a read unanswered
 * gets that press believed: a press is only ever repeated on a measurement.
 * The checks read the card as a recording would fill it — the camera's loop
 * recording must be OFF, or a full card reads as unable to record.
 *
 * The session also sets the camera's clock from the tag's RTC, when the RTC is
 * set, so that its files are stamped in UTC, and applies CAP07, the recording
 * resolution: the camera only takes a new one up at its next boot (bench,
 * 2026-10-27), so a session that finds it different writes it and restarts the
 * camera through its rail — once, after checking on the card that it is not
 * recording on its own. What the session found ends up in CAT01-CAT06.
 *
 * No command goes out before the camera has answered GET_DEVICE_INFO and
 * BOOT_MIN_MS have passed — a command sent to a camera still booting is
 * silently lost. Sequences are scheduler tasks, as in the button driver, so
 * nothing waits on the camera except power_off_blocking(), the shutdown path,
 * which stays blind: it is the path that cannot afford to wait for answers.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "cam.hpp"
#include "runcam_protocol.hpp"
#include "scheduler.hpp"

/// @brief The board side of the camera: its power rail and a byte link to its UART.
class RunCamLink {
public:
	virtual ~RunCamLink() {}
	virtual void set_rail(bool on) = 0;
	/// @brief Bring the UART up; the rail is already on. false if the peripheral refused.
	virtual bool open() = 0;
	/// @brief Take the UART down and drive its TX LOW. Called before the rail is cut,
	/// so an unpowered camera is not fed through its RX pin. Safe when not open.
	virtual void close() = 0;
	virtual bool is_open() const = 0;
	/// @brief Queue a frame, non-blocking. false if the link is down or still busy.
	virtual bool send(const uint8_t *data, size_t len) = 0;
	/// @brief Hand over the bytes received since the last call, at most @p max.
	virtual size_t receive(uint8_t *buf, size_t max) = 0;
	/// @brief UART errors since boot, for diagnostics.
	virtual unsigned int rx_errors() const { return 0; }
	/// @brief Bench diagnostics: the link's driver and peripheral state as text.
	virtual void describe(char *out, size_t out_size) const {
		if (out_size) out[0] = '\0';
	}
};

class RunCamUartDevice : public CAMDevice {
public:
	/// @brief CAP06 CAM_REC_MODE: how the first start (and the first stop) is sent.
	/// With the card checks a mismatch with the camera's auto-record setting
	/// costs one more press, not the session; a press repeated on a measurement
	/// is always POWER_BUTTON, the one action the Split 4 honours.
	enum class RecordMode : unsigned int {
		TOGGLE = 0,    ///< camera auto-record OFF: one POWER_BUTTON press to start, one to stop
		AUTO = 1,      ///< camera auto-record ON: nothing to start, one POWER_BUTTON press to stop
		COMMANDS = 2,  ///< START_RECORDING / STOP_RECORDING first (the Split 4 ignores both)
	};

	/// @brief CAP07 CAM_RESOLUTION and CAT05: the recording resolution, by the name
	/// the camera gives it. The Split 4 lists these four over its UART (bench,
	/// 2026-10-26), not the 4K / 2.7K modes of its manual. KEEP leaves the camera's
	/// own setting alone; read back, it means "unknown".
	enum class Resolution : unsigned int {
		KEEP = 0,
		R1080P60 = 1,
		R1080P50 = 2,
		R1080P30 = 3,
		R720P60 = 4,
	};
	static constexpr unsigned int RESOLUTION_COUNT = 5;
	/// @brief The camera's name for each Resolution, as its option list spells it.
	static const char *resolution_label(Resolution resolution);

	/// @brief How a session ended: CAMTST's status and CAT01. Values are a DTE contract.
	enum class Outcome : unsigned int {
		OK = 0,             ///< the recording, then its stop, both seen on the card
		NO_ANSWER = 1,      ///< no answer to GET_DEVICE_INFO
		NO_SPACE = 2,       ///< no recording time left on the card
		NOT_RECORDING = 3,  ///< the card did not fill
		UNVERIFIED = 4,     ///< the session ran, the card was not read to check it
		STOP_FAILED = 5,    ///< still recording after every stop: cut on a recording
		LINK_ERROR = 6,     ///< the UART would not open, or refused the start
	};

	/// @brief Why the last session failed (the camera test of the configuration tool reports it).
	enum class Failure : unsigned int {
		NONE = 0,
		NO_ANSWER,      ///< no answer to GET_DEVICE_INFO: absent, unpowered or miswired
		LINK,           ///< the UART would not open, or refused the start
		NO_SPACE,       ///< REMAINING_TIME 0: card full, missing or unusable
		NOT_RECORDING,  ///< the card did not fill after START_ATTEMPTS starts
	};

	/// @brief Poll period while booting. The link opens at the first poll, not with
	/// the rail: its RX pull-up would feed the camera's TX pin while the camera's
	/// own rails are still ramping.
	static constexpr unsigned int POLL_MS = 1000;
	/// @brief How long after a GET_DEVICE_INFO its answer is read. The link is
	/// drained just before each request and read again this soon after, so the
	/// answer is among the first bytes received, whatever the camera chatters
	/// around it — the UART's ISR buffer keeps the oldest bytes when it fills.
	static constexpr unsigned int REPLY_WAIT_MS = 200;
	/// @brief A settings read or write is looked for this often after its request...
	static constexpr unsigned int SETTING_POLL_MS = 100;
	/// @brief ...and sent once more if nothing valid came back within this, or
	/// after a corrupt answer; then the step goes on without it. One answer lost
	/// to line noise or a busy main loop does not cost the measurement.
	static constexpr unsigned int SETTING_TIMEOUT_MS = 1500;
	/// @brief No command before this after the rail, answered or not (ArduPilot's
	/// RunCam boot delay).
	static constexpr unsigned int BOOT_MIN_MS = 7000;
	/// @brief Still no answer to GET_DEVICE_INFO by then: camera reported absent.
	static constexpr unsigned int BOOT_TIMEOUT_MS = 15000;
	/// @brief After a start, before any other command (ArduPilot: twice its 800 ms
	/// mode delay). Waited out explicitly by the blind sequence; a verified one
	/// reads the card later than this anyway.
	static constexpr unsigned int RECORD_SETTLE_MS = 1600;
	/// @brief After the stop, before the rail is cut: the camera finalises its MP4,
	/// whose index is written last. A verified stop waits longer than this and
	/// sees the card settle; NOT MEASURED on the Split 4 with a real rail cut yet.
	static constexpr unsigned int STOP_SETTLE_MS = 3000;
	/// @brief A stop that did not leave the UART, or that the card says the camera
	/// missed, is pressed again this many times before the rail is cut regardless.
	static constexpr unsigned int STOP_RETRIES = 2;
	/// @brief Second attempt at a stop on the blocking path.
	static constexpr unsigned int STOP_RETRY_MS = 100;
	/// @brief First free-space read after a press. A stop has settled on the card
	/// by then (1.4 s on the bench); a start may take up to 3.5 s to reach it,
	/// which the second read still comes well after.
	static constexpr unsigned int VERIFY_FIRST_MS = 2500;
	/// @brief The second read, this much after the first answer.
	static constexpr unsigned int VERIFY_GAP_MS = 3000;
	/// @brief Recording = at least this much written between the two reads. The
	/// slowest Split 4 mode (1080p30) still writes ~7 MB in VERIFY_GAP_MS; the
	/// reads are whole megabytes.
	static constexpr unsigned int VERIFY_MIN_DROP_MB = 2;
	/// @brief Starts (the first, plus repeats on a measurement) before the session fails.
	static constexpr unsigned int START_ATTEMPTS = 2;
	/// @brief The Split 4 reads its clock back 8 h ahead of what was written (bench,
	/// 2026-10-26: its factory time zone), so UTC minus this is written for it to
	/// stamp its files in UTC.
	static constexpr long CAMERA_CLOCK_OFFSET_S = 8L * 3600L;
	/// @brief TEXT_SELECTION options are numbered from 0, as the RunCam spec and
	/// Betaflight number them: the bench Split 4 takes 0-3 and refuses 4
	/// (2026-10-27). Its bitrate does not tell its 1080p modes apart (all three
	/// ~5 MB/s; 720P60 ~1.9 MB/s), and its GET_SETTINGS listing always says
	/// 1080P60FPS: only the detail value is to be believed.
	static constexpr unsigned int SELECTION_INDEX_BASE = 0;
	/// @brief Rail off this long to restart the camera for a new resolution, then
	/// the session boots it again.
	static constexpr unsigned int RESTART_OFF_MS = 2000;

	explicit RunCamUartDevice(RunCamLink &link) : m_link(link) {}
	~RunCamUartDevice();

	void power_on() override;
	void power_off() override;
	void power_off_blocking() override;
	bool is_powered_on() override;
	unsigned int get_num_captures() override;

	/// @name Diagnostics, read by the bench console
	/// @{
	const char *state_name() const;
	bool has_answered() const { return m_answered; }
	const runcam::DeviceInfo &device_info() const { return m_info; }
	RecordMode record_mode() const { return m_mode; }
	/// @brief What the camera was last told to do, corrected by what its card
	/// shows whenever it is read. The camera never confirms a button press.
	bool is_recording_assumed() const { return m_recording; }
	/// @brief This session's recording was seen filling the card.
	bool is_recording_verified() const { return m_verified; }
	/// @brief This session set the camera's clock.
	bool is_clock_set() const { return m_clock_set; }
	/// @brief Last values read from the card this session, -1 if none was.
	int last_free_mb() const { return m_free_mb; }
	int last_total_mb() const { return m_total_mb; }
	int last_remaining_s() const { return m_remaining_s; }
	unsigned int corrupt_answers() const { return m_corrupt; }
	unsigned int link_errors() const { return m_link.rx_errors(); }
	void describe_link(char *out, size_t out_size) const { m_link.describe(out, out_size); }
	/// @brief Starts sent this session: 2 when the camera's auto-record disagrees with CAP06.
	unsigned int start_attempts() const { return m_start_attempts; }
	/// @brief The last stop was seen on the card: it stopped filling before the cut.
	bool is_stop_verified() const { return m_stop_verified; }
	/// @brief The rail was cut while the card still said recording, every stop spent.
	bool was_cut_recording() const { return m_cut_recording; }
	Failure last_failure() const { return m_failure; }
	/// @brief How this session ended — meaningful once it has.
	Outcome outcome() const;
	/// @brief The camera's resolution as last read this session (KEEP = unknown).
	Resolution resolution_read() const { return m_resolution; }
	/// @brief This session restarted the camera to apply CAP07.
	bool was_restarted() const { return m_restarted; }
	/// @brief Off and free: no session running, not held by the bench.
	bool is_idle() const { return m_state == State::POWERED_OFF; }
	/// @}

	/// @brief Bench only: power the camera with no recording command at all, for
	/// manual exploration. Only from OFF; power_on() / power_off() ignore the
	/// camera meanwhile. Turning it back off cuts the rail without a stop.
	bool bench_power(bool on);
	/// @brief Bench only, BLOCKING: send @p frame and collect whatever comes back
	/// within @p window_ms. Only when nothing else uses the link — the camera
	/// recording (ON) or held by bench_power().
	/// @return The number of bytes collected, or -1 if the link cannot be used now.
	int bench_exchange(const uint8_t *frame, size_t len, uint8_t *out, size_t out_size, unsigned int window_ms);
	/// @brief Bench only: what the link holds now, nothing sent, nothing dropped.
	/// @return The number of bytes, or -1 if the link cannot be used now.
	int bench_receive(uint8_t *out, size_t out_size);

private:
	enum class State { POWERED_OFF, BOOTING, STARTING, POWERED_ON, STOPPING, BENCH };
	/// @brief What the settings exchange in flight is for.
	enum class Step { NONE, CARD_CHECK, RESOLUTION_READ, RESOLUTION_WRITE, CLOCK_SET, VERIFY_FIRST, VERIFY_SECOND };
	/// @brief What the free-space reads in flight decide: a start, a stop, or
	/// whether the camera records on its own before a restart cuts its rail.
	enum class Verify { START, STOP, RESTART };

	RunCamLink &m_link;
	State m_state = State::POWERED_OFF;
	Step m_step = Step::NONE;
	RecordMode m_mode = RecordMode::TOGGLE;
	runcam::ResponseDecoder m_decoder;
	runcam::DeviceInfo m_info;
	bool m_answered = false;
	bool m_recording = false;
	bool m_verified = false;
	bool m_stop_verified = false;
	bool m_cut_recording = false;
	bool m_clock_set = false;
	Failure m_failure = Failure::NONE;
	Verify m_verify = Verify::START;             ///< what the verification in flight decides
	Resolution m_wanted = Resolution::KEEP;      ///< CAP07, read at power-on
	Resolution m_resolution = Resolution::KEEP;  ///< the camera's, as read this session
	bool m_restarted = false;                    ///< restarted this session for CAP07: never twice
	bool m_restart_stop_sent = false;            ///< a stop went before the restart in flight
	uint8_t m_selection_value = 0;               ///< the raw TEXT_SELECTION value just read
	char m_options[runcam::MAX_FRAME] = {};      ///< the option list just read
	bool m_stop_by_power = false;       ///< the camera was seen to ignore START or STOP: POWER_BUTTON from now on
	bool m_stop_sent = false;           ///< a stop left this session: its settle is owed before the cut
	bool m_replied = false;             ///< the settings exchange in flight is over, answered or not
	bool m_reply_ok = false;            ///< ...and its answer was a valid one
	bool m_answer_complete = false;     ///< a whole answer came, CRC good, whatever it said
	unsigned int m_start_attempts = 0;  ///< starts sent this session (the first, plus repeats)
	unsigned int m_stop_attempts = 0;   ///< stops repeated this session, for a send failure or a measurement
	unsigned int m_num_captures = 0;    ///< Power-on count since boot
	unsigned int m_corrupt = 0;         ///< Answers that failed their CRC since boot
	int m_free_mb = -1;
	int m_total_mb = -1;
	int m_free_first_mb = -1;  ///< the first read of the verification in flight
	int m_remaining_s = -1;
	char m_setting_text[24] = {};               ///< text of the last settings answer
	uint8_t m_request[runcam::MAX_FRAME] = {};  ///< the settings request in flight, for its one repeat
	size_t m_request_len = 0;
	runcam::Command m_request_command = runcam::Command::READ_SETTING_DETAIL;
	bool m_request_repeated = false;
	uint64_t m_rail_on_ms = 0;
	uint64_t m_request_ms = 0;     ///< the settings exchange in flight was sent
	uint64_t m_record_ms = 0;      ///< start sent (AUTO: recording assumed) — recording from here
	uint64_t m_stop_ms = 0;        ///< stop sent
	Scheduler::TaskHandle m_task;  ///< Pending step of the running sequence, if any

	void boot_poll();
	void boot_check();
	bool boot_decide();
	void prepare();
	void on_card_check();
	void read_resolution();
	void on_resolution_read();
	void on_resolution_written();
	void on_restart_measured(bool filling);
	void restart_camera();
	void power_back_on();
	void set_clock();
	void on_clock_set();
	void press_start();
	void press_stop();
	void start_verify(Verify purpose);
	void on_verify_first();
	void on_verify_second();
	void on_start_measured(bool filling);
	void on_stop_measured(bool filling);
	void verify_unavailable();
	void finish_power_on();
	void finish_power_off();
	void boot_failed(Failure failure, const char *why);
	void stop_before_failing();
	void finish_failed();
	void publish_status();
	bool send_stop();
	bool has_settings() const;
	void read_setting(uint8_t setting_id, Step step);
	void exchange(runcam::Command command, const uint8_t *frame, size_t len, Step step);
	void send_request();
	void on_step();
	bool read_free_mb(int &free_mb);
	bool request(runcam::Command command);
	bool control(runcam::ControlAction action);
	bool send_frame(const uint8_t *frame, size_t len);
	void drain();
	void on_answer();
	void cut_power();
	void warn_if_unsupported(uint16_t needed, const char *what);
	static RecordMode read_record_mode();
	static Resolution read_resolution_param();
	static Resolution resolution_of(const char *label);
	static void wait_until(uint64_t deadline_ms);
	void notify_power_on();
	void notify_power_off();
	void notify_error();
};
