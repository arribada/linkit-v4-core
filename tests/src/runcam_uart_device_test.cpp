#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

#include <vector>

#include "cam_service.hpp"
#include "dte_handler.hpp"
#include "fake_battery_mon.hpp"
#include "fake_config_store.hpp"
#include "fake_logger.hpp"
#include "fake_rtc.hpp"
#include "fake_runcam_link.hpp"
#include "fake_timer.hpp"
#include "runcam_uart_device.hpp"
#include "scheduler.hpp"

extern Timer *system_timer;
extern ConfigurationStore *configuration_store;
extern Scheduler *system_scheduler;
extern RTC *rtc;
extern BatteryMonitor *battery_monitor;
extern RunCamUartDevice *runcam_uart_device;

namespace {
/// The Split 4 as it answers on the bench (and as ArduPilot reported it).
const std::vector<uint8_t> SPLIT4_INFO = { 0xCC, 0x01, 0x77, 0x00, 0x02 };
/// The same features less SETTINGS_ACCESS: a camera whose card cannot be read,
/// so the sequence is the blind one, press believed, nothing verified.
const std::vector<uint8_t> BLIND_INFO = { 0xCC, 0x01, 0x67, 0x00, 0xB2 };
constexpr uint8_t POWER = 0x01;
constexpr uint8_t START = 0x03;
constexpr uint8_t STOP = 0x04;

struct CamEvents : public CAMEventListener {
	unsigned int on = 0;
	unsigned int off = 0;
	unsigned int error = 0;
	void react(const CAMEventPowerOn &) override { on++; }
	void react(const CAMEventPowerOff &) override { off++; }
	void react(const CAMEventError &) override { error++; }
};
}  // namespace

/*
 * RunCamUartDevice against a fake link, for a camera whose card cannot be read
 * (BLIND_INFO). It never answers a button press either, so what the device
 * believes it is doing rests entirely on WHEN it presses and HOW OFTEN. What
 * is checked: nothing goes out before the camera has answered and BOOT_MIN_MS
 * have passed; recording is pressed exactly as often as CAP06 says -- one
 * press too many is a toggle, a recording started just before the rail cut
 * corrupts it; and every path that cuts the rail takes the link down first.
 */
TEST_GROUP(RunCamUartDevice) {
	FakeTimer *fake_timer;
	FakeConfigurationStore *fake_config_store;
	FakeRunCamLink *link;
	RunCamUartDevice *cam;
	CamEvents events;

	void setup() {
		fake_timer = new FakeTimer;
		system_timer = fake_timer;
		fake_config_store = new FakeConfigurationStore;
		configuration_store = fake_config_store;
		configuration_store->init();
		system_scheduler = new Scheduler(system_timer);
		fake_timer->start();
		link = new FakeRunCamLink;
		cam = new RunCamUartDevice(*link);
		cam->subscribe(events);
	}

	void teardown() {
		mock().checkExpectations();
		mock().clear();
		mock().ignoreOtherCalls();  // the destructor's blocking power-off, if a test left the camera on
		cam->unsubscribe(events);
		delete cam;
		delete link;
		delete system_scheduler;
		delete fake_timer;
		delete fake_config_store;
		system_scheduler = nullptr;
		system_timer = nullptr;
		configuration_store = nullptr;
		mock().clear();
	}

	/// @brief Advance the fake clock one millisecond at a time, running the
	/// scheduler after each tick, so a task posted by a task fires on time.
	void run_for(unsigned int ms) {
		for (unsigned int i = 0; i < ms; i++) {
			fake_timer->increment_counter(1);
			system_scheduler->run();
		}
	}

	void set_mode(RunCamUartDevice::RecordMode mode) {
		configuration_store->write_param(ParamID::CAM_REC_MODE, static_cast<unsigned int>(mode));
	}

	/// @brief Power on a camera that answers, through to the end of its start.
	void bring_on() {
		link->info_answer = BLIND_INFO;
		cam->power_on();
		run_for(RunCamUartDevice::BOOT_MIN_MS + RunCamUartDevice::RECORD_SETTLE_MS);
		CHECK_TRUE(cam->is_powered_on());
	}
};

TEST(RunCamUartDevice, LinkWaitsOnePollAfterTheRail) {
	link->info_answer = BLIND_INFO;
	cam->power_on();
	CHECK_TRUE(link->rail);
	CHECK_FALSE(link->link_open);  // no RX pull-up on a camera whose rails are still ramping
	CHECK_EQUAL(0U, link->sent.size());
	run_for(RunCamUartDevice::POLL_MS - 1);
	CHECK_FALSE(link->link_open);
	run_for(1);
	CHECK_TRUE(link->link_open);
	CHECK_EQUAL(1U, link->info_requests());
}

TEST(RunCamUartDevice, TogglePressesOnceAfterBootMinThenReportsOn) {
	link->info_answer = BLIND_INFO;
	cam->power_on();
	run_for(RunCamUartDevice::BOOT_MIN_MS - 1);
	CHECK_TRUE(cam->has_answered());
	CHECK_EQUAL(0U, link->all_controls());   // answered at 2 s, still nothing before BOOT_MIN_MS
	CHECK_EQUAL(1U, link->info_requests());  // and no second GET_DEVICE_INFO once answered
	run_for(1);
	CHECK_EQUAL(1U, link->controls(POWER));
	CHECK(link->sent.back() == (std::vector<uint8_t>{ 0xCC, 0x01, 0x01, 0xE7 }));
	CHECK_FALSE(cam->is_powered_on());
	run_for(RunCamUartDevice::RECORD_SETTLE_MS - 1);
	CHECK_FALSE(cam->is_powered_on());
	CHECK_EQUAL(0U, events.on);
	run_for(1);
	CHECK_TRUE(cam->is_powered_on());
	CHECK_EQUAL(1U, events.on);
	CHECK_EQUAL(1U, cam->get_num_captures());
	CHECK_TRUE(cam->is_recording_assumed());
	CHECK_EQUAL(0x0067, cam->device_info().features);
}

TEST(RunCamUartDevice, StopIsOnePressThenTheRailAfterTheSettle) {
	bring_on();
	const size_t before = link->events.size();
	cam->power_off();
	CHECK_EQUAL(2U, link->controls(POWER));  // the start, and this stop
	CHECK_TRUE(link->rail);
	run_for(RunCamUartDevice::STOP_SETTLE_MS - 1);
	CHECK_TRUE(link->rail);  // the camera is closing its file
	CHECK_EQUAL(0U, events.off);
	run_for(1);
	CHECK_FALSE(link->rail);
	CHECK_EQUAL(1U, events.off);
	CHECK_FALSE(cam->is_recording_assumed());
	// The link goes down (TX driven low) BEFORE the rail.
	CHECK_EQUAL(before + 2, link->events.size());
	STRCMP_EQUAL("close", link->events[before].c_str());
	STRCMP_EQUAL("rail_off", link->events[before + 1].c_str());
}

TEST(RunCamUartDevice, SilentCameraIsAnErrorAndIsNeverPressed) {
	cam->power_on();
	run_for(RunCamUartDevice::BOOT_TIMEOUT_MS - 1);
	CHECK_TRUE(link->rail);
	CHECK_EQUAL(0U, events.error);
	run_for(1);
	CHECK_FALSE(link->rail);
	CHECK_FALSE(link->link_open);
	CHECK_EQUAL(1U, events.error);
	CHECK_EQUAL(0U, events.on);
	CHECK_EQUAL(0U, link->all_controls());
	CHECK_EQUAL(14U, link->info_requests());  // one a second, from the first poll to the last before the timeout
	CHECK_FALSE(cam->is_powered_on());
	cam->power_on();  // the next session tries again
	CHECK_TRUE(link->rail);
}

TEST(RunCamUartDevice, LateAnswerPastBootMinStartsAtOnce) {
	cam->power_on();
	run_for(8500);
	CHECK_FALSE(cam->has_answered());
	link->info_answer = BLIND_INFO;  // the camera comes up late
	run_for(500);                    // 9 s: this poll's request is answered
	CHECK_EQUAL(0U, link->all_controls());
	run_for(RunCamUartDevice::REPLY_WAIT_MS);  // the answer is read, BOOT_MIN_MS is long past
	CHECK_EQUAL(1U, link->controls(POWER));
}

TEST(RunCamUartDevice, StartThatDidNotLeaveIsAnErrorNotARecording) {
	// Believing it recording would make the stop press START a file the rail
	// cut then corrupts.
	link->info_answer = BLIND_INFO;
	link->fail_next_controls = 1;
	cam->power_on();
	run_for(RunCamUartDevice::BOOT_MIN_MS);
	CHECK_EQUAL(0U, link->all_controls());
	CHECK_FALSE(cam->is_recording_assumed());
	CHECK_FALSE(link->rail);  // in standby: nothing to stop, cut at once
	CHECK_EQUAL(1U, events.error);
	CHECK_EQUAL(0U, events.on);
}

TEST(RunCamUartDevice, StopThatDidNotLeaveIsRetriedBeforeTheCut) {
	bring_on();
	link->fail_next_controls = 1;
	cam->power_off();
	CHECK_EQUAL(1U, link->controls(POWER));  // the stop did not leave
	CHECK_TRUE(cam->is_recording_assumed());
	run_for(RunCamUartDevice::STOP_SETTLE_MS);
	CHECK_EQUAL(2U, link->controls(POWER));  // tried again
	CHECK_TRUE(link->rail);                  // and its own settle is waited out
	run_for(RunCamUartDevice::STOP_SETTLE_MS - 1);
	CHECK_TRUE(link->rail);
	run_for(1);
	CHECK_FALSE(link->rail);
	CHECK_EQUAL(1U, events.off);
}

TEST(RunCamUartDevice, StopThatNeverLeavesEndsInACutAnyway) {
	bring_on();
	link->fail_next_controls = 1 + RunCamUartDevice::STOP_RETRIES;
	cam->power_off();
	run_for((1 + RunCamUartDevice::STOP_RETRIES) * RunCamUartDevice::STOP_SETTLE_MS);
	CHECK_EQUAL(1U, link->controls(POWER));  // only the start ever left
	CHECK_FALSE(link->rail);                 // the battery is not held hostage
	CHECK_EQUAL(1U, events.off);
}

TEST(RunCamUartDevice, AutoModeSilentCameraIsStoppedBlindBeforeTheCut) {
	// An auto-recording camera records whether or not its answer reaches us:
	// a broken TX line must not cost the file.
	set_mode(RunCamUartDevice::RecordMode::AUTO);
	cam->power_on();
	run_for(RunCamUartDevice::BOOT_TIMEOUT_MS);
	CHECK_EQUAL(1U, link->controls(POWER));
	CHECK_TRUE(link->rail);
	CHECK_EQUAL(0U, events.error);
	run_for(RunCamUartDevice::STOP_SETTLE_MS);
	CHECK_FALSE(link->rail);
	CHECK_EQUAL(1U, events.error);
	CHECK_EQUAL(0U, events.on);
}

TEST(RunCamUartDevice, AutoModeBlindStopThatDidNotLeaveIsRetried) {
	set_mode(RunCamUartDevice::RecordMode::AUTO);
	link->fail_next_controls = 1;
	cam->power_on();
	run_for(RunCamUartDevice::BOOT_TIMEOUT_MS);
	CHECK_EQUAL(0U, link->controls(POWER));  // refused by the link
	CHECK_TRUE(link->rail);
	run_for(RunCamUartDevice::STOP_RETRY_MS);
	CHECK_EQUAL(1U, link->controls(POWER));  // tried again
	CHECK_TRUE(link->rail);
	run_for(RunCamUartDevice::STOP_SETTLE_MS);
	CHECK_FALSE(link->rail);
	CHECK_EQUAL(1U, events.error);
	CHECK_FALSE(cam->was_cut_recording());
}

TEST(RunCamUartDevice, AutoModeBlindStopThatNeverLeavesIsACutOnARecording) {
	set_mode(RunCamUartDevice::RecordMode::AUTO);
	link->fail_next_controls = 1 + RunCamUartDevice::STOP_RETRIES;
	cam->power_on();
	run_for(RunCamUartDevice::BOOT_TIMEOUT_MS + RunCamUartDevice::STOP_RETRIES * RunCamUartDevice::STOP_RETRY_MS);
	CHECK_EQUAL(0U, link->controls(POWER));
	CHECK_FALSE(link->rail);  // the battery is not held hostage
	CHECK_EQUAL(1U, events.error);
	CHECK_TRUE(cam->was_cut_recording());
}

TEST(RunCamUartDevice, AutoModeBlockingOffEarlyInTheBootCutsWithoutAPress) {
	set_mode(RunCamUartDevice::RecordMode::AUTO);
	cam->power_on();
	run_for(RunCamUartDevice::BOOT_MIN_MS / 2);  // no answer, and too early to be recording
	cam->power_off_blocking();
	CHECK_EQUAL(0U, link->all_controls());
	CHECK_FALSE(link->rail);
}

TEST(RunCamUartDevice, CorruptAnswerIsCountedAndPollingGoesOn) {
	link->info_answer = { 0xCC, 0x01, 0x77, 0x00, 0x03 };  // CRC off by one
	cam->power_on();
	run_for(3000);
	CHECK_FALSE(cam->has_answered());
	CHECK_TRUE(cam->corrupt_answers() >= 1U);
	link->info_answer = BLIND_INFO;
	run_for(RunCamUartDevice::BOOT_MIN_MS + RunCamUartDevice::RECORD_SETTLE_MS);
	CHECK_TRUE(cam->is_powered_on());
	CHECK_EQUAL(1U, link->controls(POWER));
}

TEST(RunCamUartDevice, AutoModeSendsNothingToStartAndOnePressToStop) {
	set_mode(RunCamUartDevice::RecordMode::AUTO);
	bring_on();
	CHECK_EQUAL(0U, link->all_controls());
	CHECK_TRUE(cam->is_recording_assumed());
	cam->power_off();
	CHECK_EQUAL(1U, link->controls(POWER));
	run_for(RunCamUartDevice::STOP_SETTLE_MS);
	CHECK_FALSE(link->rail);
}

TEST(RunCamUartDevice, CommandsModeUsesStartAndStop) {
	set_mode(RunCamUartDevice::RecordMode::COMMANDS);
	bring_on();
	CHECK_EQUAL(1U, link->controls(START));
	CHECK_EQUAL(0U, link->controls(POWER));
	cam->power_off();
	CHECK_EQUAL(1U, link->controls(STOP));
	CHECK(link->sent.back() == (std::vector<uint8_t>{ 0xCC, 0x01, 0x04, 0xCC }));
	run_for(RunCamUartDevice::STOP_SETTLE_MS);
	CHECK_FALSE(link->rail);
}

TEST(RunCamUartDevice, UnknownRecordModeFallsBackToToggle) {
	configuration_store->write_param(ParamID::CAM_REC_MODE, 7U);
	bring_on();
	CHECK(RunCamUartDevice::RecordMode::TOGGLE == cam->record_mode());
	CHECK_EQUAL(1U, link->controls(POWER));
}

TEST(RunCamUartDevice, RecordModeIsReadAtEachPowerOn) {
	bring_on();  // TOGGLE, the default
	cam->power_off();
	run_for(RunCamUartDevice::STOP_SETTLE_MS);
	set_mode(RunCamUartDevice::RecordMode::COMMANDS);
	bring_on();
	CHECK_EQUAL(1U, link->controls(START));
	CHECK_EQUAL(2U, cam->get_num_captures());
}

TEST(RunCamUartDevice, BlockingOffFromOnStopsOnceAndWaitsTheSettle) {
	bring_on();
	mock().expectOneCall("delay_ms").withParameter("ms", RunCamUartDevice::STOP_SETTLE_MS);
	cam->power_off_blocking();
	CHECK_EQUAL(2U, link->controls(POWER));
	CHECK_FALSE(link->rail);
	CHECK_EQUAL(1U, events.off);
	STRCMP_EQUAL("close", link->events[link->events.size() - 2].c_str());
	STRCMP_EQUAL("rail_off", link->events.back().c_str());
}

TEST(RunCamUartDevice, BlockingOffWhileStoppingNeverPressesTwice) {
	bring_on();
	cam->power_off();
	run_for(1000);
	mock().expectOneCall("delay_ms").withParameter("ms", RunCamUartDevice::STOP_SETTLE_MS - 1000);
	cam->power_off_blocking();
	CHECK_EQUAL(2U, link->controls(POWER));  // a third press would start a recording the rail cut corrupts
	CHECK_FALSE(link->rail);
	CHECK_EQUAL(1U, events.off);
	run_for(RunCamUartDevice::STOP_SETTLE_MS);  // the cancelled asynchronous step does not fire again
	CHECK_EQUAL(1U, events.off);
}

TEST(RunCamUartDevice, BlockingOffWhileBootingCutsWithoutAPress) {
	link->info_answer = BLIND_INFO;
	cam->power_on();
	run_for(3000);              // answered, still before BOOT_MIN_MS
	cam->power_off_blocking();  // no delay_ms expected: nothing is recording
	CHECK_EQUAL(0U, link->all_controls());
	CHECK_FALSE(link->rail);
	CHECK_EQUAL(1U, events.off);
	run_for(RunCamUartDevice::BOOT_TIMEOUT_MS);
	CHECK_EQUAL(0U, link->all_controls());  // and the boot polling is over
	CHECK_FALSE(link->rail);
}

TEST(RunCamUartDevice, BlockingOffJustAfterTheStartLetsItSettleFirst) {
	link->info_answer = BLIND_INFO;
	cam->power_on();
	run_for(RunCamUartDevice::BOOT_MIN_MS + 500);  // the start went out 500 ms ago
	mock().expectOneCall("delay_ms").withParameter("ms", RunCamUartDevice::RECORD_SETTLE_MS - 500);
	mock().expectOneCall("delay_ms").withParameter("ms", RunCamUartDevice::STOP_SETTLE_MS);
	cam->power_off_blocking();
	CHECK_EQUAL(2U, link->controls(POWER));
	CHECK_FALSE(link->rail);
}

TEST(RunCamUartDevice, AutoModeBlockingOffAfterTheAnswerStopsTheRecording) {
	set_mode(RunCamUartDevice::RecordMode::AUTO);
	link->info_answer = BLIND_INFO;
	cam->power_on();
	run_for(4000);  // answered at 2 s: an auto-recording camera records from the end of its boot
	CHECK_TRUE(cam->is_recording_assumed());
	mock().expectOneCall("delay_ms").withParameter("ms", RunCamUartDevice::STOP_SETTLE_MS);
	cam->power_off_blocking();
	CHECK_EQUAL(1U, link->controls(POWER));
	CHECK_FALSE(link->rail);
}

TEST(RunCamUartDevice, LinkThatWillNotOpenIsAnError) {
	link->open_fails = true;
	cam->power_on();
	run_for(RunCamUartDevice::POLL_MS);
	CHECK_EQUAL(1U, events.error);
	CHECK_FALSE(link->rail);
}

TEST(RunCamUartDevice, PowerRequestsAreIdempotent) {
	link->info_answer = BLIND_INFO;
	cam->power_on();
	cam->power_on();
	cam->power_off();  // ignored: not ON yet
	unsigned int rails_on = 0;
	for (const auto &e : link->events)
		if (e == "rail_on") rails_on++;
	CHECK_EQUAL(1U, rails_on);
	run_for(RunCamUartDevice::BOOT_MIN_MS + RunCamUartDevice::RECORD_SETTLE_MS);
	CHECK_EQUAL(1U, link->controls(POWER));
	cam->power_on();  // ignored: already ON
	CHECK_EQUAL(1U, link->controls(POWER));
}

TEST(RunCamUartDevice, BenchPowerKeepsTheDriverAway) {
	CHECK_TRUE(cam->bench_power(true));
	CHECK_TRUE(link->rail);
	STRCMP_EQUAL("BENCH", cam->state_name());
	cam->power_on();  // the bench holds the camera
	run_for(RunCamUartDevice::BOOT_TIMEOUT_MS);
	CHECK_EQUAL(0U, link->sent.size());
	CHECK_FALSE(cam->bench_power(true));
	CHECK_TRUE(cam->bench_power(false));
	CHECK_FALSE(link->rail);
	STRCMP_EQUAL("OFF", cam->state_name());
	CHECK_EQUAL(0U, events.on + events.off + events.error);
}

/*
 * The card checks, against a camera played the way the Split 4 behaves on the
 * bench: POWER toggles, START is ignored, the card fills from 1.5 s after a
 * start to 1 s after a stop, and its free space is answered live. What is
 * checked: a session is ON only once the card is seen filling; a press the
 * camera missed is pressed again, exactly once for a start; the rail is cut
 * only once the card has stopped filling; a camera that cannot record fails
 * its session; and a camera that leaves its reads unanswered gets the blind
 * sequence, never a press repeated on a guess.
 */
TEST_GROUP(RunCamUartCard) {
	FakeTimer *fake_timer;
	FakeConfigurationStore *fake_config_store;
	FakeRTC *fake_rtc;
	FakeRunCamLink *link;
	RunCamUartDevice *cam;
	CamEvents events;

	/// From the rail to ON, every answer prompt and the RTC unset: the boot, the
	/// card check, then the start and its two reads.
	/// From the rail to the start: the boot, the card check, the resolution read.
	static constexpr unsigned int PRESS_MS = RunCamUartDevice::BOOT_MIN_MS + 2 * RunCamUartDevice::SETTING_POLL_MS;
	static constexpr unsigned int VERIFIED_ON_MS = PRESS_MS + RunCamUartDevice::VERIFY_FIRST_MS
	                                               + RunCamUartDevice::SETTING_POLL_MS + RunCamUartDevice::VERIFY_GAP_MS
	                                               + RunCamUartDevice::SETTING_POLL_MS;
	/// From the stop to the cut, every answer prompt.
	static constexpr unsigned int VERIFIED_OFF_MS =
	    RunCamUartDevice::VERIFY_FIRST_MS + RunCamUartDevice::SETTING_POLL_MS + RunCamUartDevice::VERIFY_GAP_MS
	    + RunCamUartDevice::SETTING_POLL_MS;

	void setup() {
		fake_timer = new FakeTimer;
		system_timer = fake_timer;
		fake_config_store = new FakeConfigurationStore;
		configuration_store = fake_config_store;
		configuration_store->init();
		fake_rtc = new FakeRTC;
		rtc = fake_rtc;
		system_scheduler = new Scheduler(system_timer);
		fake_timer->start();
		link = new FakeRunCamLink;
		link->info_answer = SPLIT4_INFO;
		link->camera = true;
		cam = new RunCamUartDevice(*link);
		cam->subscribe(events);
	}

	void teardown() {
		mock().checkExpectations();
		mock().clear();
		mock().ignoreOtherCalls();
		cam->unsubscribe(events);
		delete cam;
		delete link;
		delete system_scheduler;
		delete fake_timer;
		delete fake_config_store;
		delete fake_rtc;
		system_scheduler = nullptr;
		system_timer = nullptr;
		configuration_store = nullptr;
		rtc = nullptr;
		mock().clear();
	}

	void run_for(unsigned int ms) {
		for (unsigned int i = 0; i < ms; i++) {
			fake_timer->increment_counter(1);
			system_scheduler->run();
		}
	}

	/// @brief Run until the session is ON or failed, at most @p limit_ms.
	void run_until_settled(unsigned int limit_ms) {
		for (unsigned int i = 0; i < limit_ms && !cam->is_powered_on() && events.error == 0; i++)
			run_for(1);
	}

	/// @brief Run until the rail is cut, at most @p limit_ms.
	void run_until_cut(unsigned int limit_ms) {
		for (unsigned int i = 0; i < limit_ms && link->rail; i++)
			run_for(1);
	}

	void set_mode(RunCamUartDevice::RecordMode mode) {
		configuration_store->write_param(ParamID::CAM_REC_MODE, static_cast<unsigned int>(mode));
	}

	void bring_on() {
		cam->power_on();
		run_until_settled(90000);
		CHECK_TRUE(cam->is_powered_on());
	}

	unsigned int status(ParamID id) {
		return configuration_store->read_param<unsigned int>(id);
	}
};

TEST(RunCamUartCard, StartIsOnOnlyOnceTheCardIsSeenFilling) {
	cam->power_on();
	run_for(RunCamUartDevice::BOOT_MIN_MS + RunCamUartDevice::SETTING_POLL_MS - 1);
	CHECK_EQUAL(0U, link->all_controls());  // the card is checked before anything is pressed
	CHECK_EQUAL(1U, link->setting_requests());
	run_for(1);
	CHECK_EQUAL(2U, link->setting_requests());  // then the resolution is read
	CHECK_EQUAL(0U, link->all_controls());
	run_for(PRESS_MS - RunCamUartDevice::BOOT_MIN_MS - RunCamUartDevice::SETTING_POLL_MS);
	CHECK_EQUAL(1U, link->controls(POWER));
	CHECK_TRUE(link->recording);
	run_for(VERIFIED_ON_MS - PRESS_MS - 1);
	CHECK_FALSE(cam->is_powered_on());  // pressed, not yet seen on the card
	CHECK_EQUAL(0U, events.on);
	run_for(1);
	CHECK_TRUE(cam->is_powered_on());
	CHECK_EQUAL(1U, events.on);
	CHECK_TRUE(cam->is_recording_verified());
	CHECK_EQUAL(1U, link->controls(POWER));
	CHECK_EQUAL(4U, link->setting_requests());  // remaining time, resolution, then two free-space reads
	CHECK(RunCamUartDevice::Resolution::R1080P60 == cam->resolution_read());
	CHECK_EQUAL(0U, link->resolution_writes);  // CAP07 0: the camera's own resolution is left alone
	CHECK_EQUAL(1U, link->rail_ons);
	CHECK_TRUE(cam->last_free_mb() < 14707);
	CHECK_TRUE(cam->last_remaining_s() > 0);
}

TEST(RunCamUartCard, StartTheCameraMissedIsPressedOnceMore) {
	link->lost_presses = 1;
	bring_on();
	CHECK_EQUAL(2U, link->controls(POWER));
	CHECK_TRUE(link->recording);
	CHECK_TRUE(cam->is_recording_verified());
	CHECK_EQUAL(0U, events.error);
}

TEST(RunCamUartCard, AutoRecordingCameraUnderToggleIsPutRight) {
	// CAP06 says TOGGLE but the camera records on its own: the first press
	// stops it, the card says so, the second starts it again.
	link->auto_record = true;
	bring_on();
	CHECK_EQUAL(2U, link->controls(POWER));
	CHECK_TRUE(link->recording);
	CHECK_TRUE(cam->is_recording_verified());
}

TEST(RunCamUartCard, AutoModeOnACameraThatDoesNotAutoRecordGetsOnePress) {
	set_mode(RunCamUartDevice::RecordMode::AUTO);
	bring_on();
	CHECK_EQUAL(1U, link->controls(POWER));
	CHECK_TRUE(link->recording);
}

TEST(RunCamUartCard, AutoModeOnAnAutoRecordingCameraPressesNothingToStart) {
	set_mode(RunCamUartDevice::RecordMode::AUTO);
	link->auto_record = true;
	bring_on();
	CHECK_EQUAL(0U, link->all_controls());
	CHECK_TRUE(cam->is_recording_verified());
}

TEST(RunCamUartCard, CommandsModeFallsBackToPowerAndStopsWithIt) {
	set_mode(RunCamUartDevice::RecordMode::COMMANDS);
	bring_on();
	CHECK_EQUAL(1U, link->controls(START));  // ignored, as by the Split 4
	CHECK_EQUAL(1U, link->controls(POWER));
	cam->power_off();
	// The camera has shown it ignores START: no STOP is wasted on it.
	CHECK_EQUAL(0U, link->controls(STOP));
	CHECK_EQUAL(2U, link->controls(POWER));
	run_until_cut(30000);
	CHECK_EQUAL(2U, link->controls(POWER));
	CHECK_FALSE(link->recording_at_cut);
	CHECK_TRUE(cam->is_stop_verified());
	CHECK_EQUAL(1U, events.off);
}

TEST(RunCamUartCard, CommandsModeBlockingOffAfterAPowerStartStopsWithPower) {
	// The shutdown path checks nothing on the card: a STOP the camera ignores
	// would leave the rail cut on a live recording.
	set_mode(RunCamUartDevice::RecordMode::COMMANDS);
	bring_on();
	mock().expectOneCall("delay_ms").withParameter("ms", RunCamUartDevice::STOP_SETTLE_MS);
	cam->power_off_blocking();
	CHECK_EQUAL(0U, link->controls(STOP));
	CHECK_EQUAL(2U, link->controls(POWER));
	CHECK_FALSE(link->recording_at_cut);
}

TEST(RunCamUartCard, CommandsModeOnACameraThatHonoursThemNeverPressesPower) {
	set_mode(RunCamUartDevice::RecordMode::COMMANDS);
	link->honours_start = true;
	link->honours_stop = true;
	bring_on();
	cam->power_off();
	run_until_cut(30000);
	CHECK_EQUAL(1U, link->controls(START));
	CHECK_EQUAL(1U, link->controls(STOP));
	CHECK_EQUAL(0U, link->controls(POWER));
	CHECK_FALSE(link->recording_at_cut);
}

TEST(RunCamUartCard, CameraThatNeverRecordsFailsItsSession) {
	link->deaf = true;
	cam->power_on();
	run_until_settled(40000);
	CHECK_EQUAL(1U, events.error);
	CHECK_EQUAL(0U, events.on);
	CHECK_EQUAL(RunCamUartDevice::START_ATTEMPTS, link->controls(POWER));  // and no stop: it is measured in standby
	CHECK_FALSE(link->rail);
	CHECK_FALSE(link->link_open);
}

TEST(RunCamUartCard, FullCardFailsTheSessionWithoutAPress) {
	link->remaining_s = 0;
	cam->power_on();
	run_for(RunCamUartDevice::BOOT_MIN_MS + RunCamUartDevice::SETTING_POLL_MS);
	CHECK_EQUAL(1U, events.error);
	CHECK_EQUAL(0U, link->all_controls());
	CHECK_FALSE(link->rail);
	CHECK_EQUAL(0, cam->last_remaining_s());
}

TEST(RunCamUartCard, FullCardUnderAutoIsCutWithoutAStop) {
	set_mode(RunCamUartDevice::RecordMode::AUTO);
	link->remaining_s = 0;
	cam->power_on();
	run_for(RunCamUartDevice::BOOT_MIN_MS + RunCamUartDevice::SETTING_POLL_MS);
	CHECK_EQUAL(1U, events.error);
	CHECK_EQUAL(0U, link->all_controls());  // a press would only try to start
	CHECK_FALSE(link->rail);
}

TEST(RunCamUartCard, UnansweredReadsAreBelievedNeverRepeated) {
	link->settings_answer = false;
	bring_on();
	CHECK_EQUAL(1U, link->controls(POWER));
	CHECK_FALSE(cam->is_recording_verified());
	CHECK_EQUAL(-1, cam->last_free_mb());
	cam->power_off();
	run_until_cut(30000);
	CHECK_EQUAL(2U, link->controls(POWER));
	CHECK_FALSE(link->recording_at_cut);
	CHECK_EQUAL(1U, events.off);
}

TEST(RunCamUartCard, StopIsCutOnlyOnceTheCardHasStopped) {
	bring_on();
	cam->power_off();
	CHECK_EQUAL(2U, link->controls(POWER));
	run_for(VERIFIED_OFF_MS - 1);
	CHECK_TRUE(link->rail);
	CHECK_EQUAL(0U, events.off);
	run_for(1);
	CHECK_FALSE(link->rail);
	CHECK_FALSE(link->recording_at_cut);
	CHECK_EQUAL(1U, events.off);
	CHECK_TRUE(VERIFIED_OFF_MS >= RunCamUartDevice::STOP_SETTLE_MS);  // the file had its time to close
	STRCMP_EQUAL("close", link->events[link->events.size() - 2].c_str());
	STRCMP_EQUAL("rail_off", link->events.back().c_str());
}

TEST(RunCamUartCard, StopTheCameraMissedIsPressedAgainBeforeTheCut) {
	bring_on();
	link->lost_presses = 1;
	cam->power_off();
	run_until_cut(30000);
	CHECK_EQUAL(3U, link->controls(POWER));  // the start, the missed stop, the stop
	CHECK_FALSE(link->recording_at_cut);
	CHECK_EQUAL(1U, events.off);
}

TEST(RunCamUartCard, CameraThatIgnoresEveryStopIsCutAfterTheRetries) {
	bring_on();
	link->deaf = true;
	cam->power_off();
	run_until_cut(60000);
	CHECK_EQUAL(1U + 1U + RunCamUartDevice::STOP_RETRIES, link->controls(POWER));
	CHECK_FALSE(link->rail);  // the battery is not held hostage
	CHECK_TRUE(link->recording_at_cut);
	CHECK_EQUAL(1U, events.off);
}

TEST(RunCamUartCard, ClockIsSetToUtcLessTheCameraOffset) {
	fake_rtc->settime(1761510000);  // 2025-10-26 20:20:00 UTC
	bring_on();
	CHECK_EQUAL(1U, link->clock_writes);
	STRCMP_EQUAL("20251026T122000.0", link->clock.c_str());  // the camera adds its 8 h back
	CHECK_TRUE(cam->is_clock_set());
	// Written before the start, so the file is named with it.
	size_t write_at = 0, press_at = 0;
	for (size_t i = 0; i < link->sent.size(); i++) {
		if (link->sent[i][1] == 0x13) write_at = i;
		if (link->sent[i].size() == 4 && link->sent[i][1] == 0x01 && !press_at) press_at = i;
	}
	CHECK_TRUE(write_at < press_at);
}

TEST(RunCamUartCard, ClockIsLeftAloneWhileTheRtcIsUnset) {
	bring_on();
	CHECK_EQUAL(0U, link->clock_writes);
	CHECK_FALSE(cam->is_clock_set());
}

TEST(RunCamUartCard, AutoModeLeavesTheClockAlone) {
	set_mode(RunCamUartDevice::RecordMode::AUTO);
	link->auto_record = true;
	fake_rtc->settime(1761510000);
	bring_on();
	CHECK_EQUAL(0U, link->clock_writes);  // the file is already open
}

TEST(RunCamUartCard, ClockTheCameraRefusesStillStartsTheSession) {
	fake_rtc->settime(1761510000);
	link->settings_answer = false;
	bring_on();
	CHECK_FALSE(cam->is_clock_set());
	CHECK_EQUAL(1U, link->controls(POWER));
}

TEST(RunCamUartCard, BlockingOffDuringTheStartCheckStopsTheRecording) {
	cam->power_on();
	run_for(PRESS_MS + 200);  // pressed 200 ms ago
	CHECK_EQUAL(1U, link->controls(POWER));
	mock().expectOneCall("delay_ms").withParameter("ms", RunCamUartDevice::RECORD_SETTLE_MS - 200);
	mock().expectOneCall("delay_ms").withParameter("ms", RunCamUartDevice::STOP_SETTLE_MS);
	cam->power_off_blocking();
	CHECK_EQUAL(2U, link->controls(POWER));
	CHECK_FALSE(link->rail);
	CHECK_FALSE(link->recording_at_cut);
	run_for(VERIFIED_ON_MS);  // the cancelled verification does not come back
	CHECK_EQUAL(2U, link->controls(POWER));
	CHECK_EQUAL(0U, events.on);
}

TEST(RunCamUartCard, BlockingOffDuringTheStopCheckNeverPressesAgain) {
	bring_on();
	cam->power_off();
	run_for(1000);
	mock().expectOneCall("delay_ms").withParameter("ms", RunCamUartDevice::STOP_SETTLE_MS - 1000);
	cam->power_off_blocking();
	CHECK_EQUAL(2U, link->controls(POWER));
	CHECK_FALSE(link->rail);
	CHECK_EQUAL(1U, events.off);
	run_for(VERIFIED_OFF_MS);
	CHECK_EQUAL(1U, events.off);
}

TEST(RunCamUartCard, ResolutionChangeIsWrittenThenTakenUpByARestart) {
	configuration_store->write_param(ParamID::CAM_RESOLUTION, 4U);  // 720P60
	bring_on();
	CHECK_EQUAL(1U, link->resolution_writes);
	CHECK_EQUAL(3U, (unsigned int)link->resolution_stored);  // its 4th option, numbered from 0
	CHECK_EQUAL(3U, (unsigned int)link->resolution_active);  // taken up at the restart
	CHECK_EQUAL(2U, link->rail_ons);
	CHECK_TRUE(cam->was_restarted());
	CHECK(RunCamUartDevice::Resolution::R720P60 == cam->resolution_read());
	CHECK_TRUE(cam->is_recording_verified());
	CHECK_EQUAL(1U, link->controls(POWER));  // nothing to stop before the restart, one start after
	CHECK_EQUAL(0U, link->cuts_while_recording);
	// The restart takes the link down (TX low) before the rail, like any cut.
	size_t first_cut = 0;
	while (first_cut < link->events.size() && link->events[first_cut] != "rail_off")
		first_cut++;
	CHECK_TRUE(first_cut > 0 && first_cut < link->events.size());
	STRCMP_EQUAL("close", link->events[first_cut - 1].c_str());
}

TEST(RunCamUartCard, ResolutionAlreadySetIsNotTouched) {
	configuration_store->write_param(ParamID::CAM_RESOLUTION, 1U);
	bring_on();
	CHECK_EQUAL(0U, link->resolution_writes);
	CHECK_EQUAL(1U, link->rail_ons);
	CHECK_FALSE(cam->was_restarted());
}

TEST(RunCamUartCard, ResolutionTheCameraDoesNotOfferIsLeftAlone) {
	link->resolution_options = "4K30FPS;2.7K60FPS";  // another model's list
	configuration_store->write_param(ParamID::CAM_RESOLUTION, 4U);
	bring_on();
	CHECK_EQUAL(0U, link->resolution_writes);
	CHECK_EQUAL(1U, link->rail_ons);
	CHECK(RunCamUartDevice::Resolution::KEEP == cam->resolution_read());  // not one of ours: unknown
	CHECK_TRUE(cam->is_recording_verified());
}

TEST(RunCamUartCard, AutoRecordingCameraIsStoppedBeforeItsRestart) {
	// The bench Split 4 starts recording on its own at boot: the restart must
	// not end that file without its index.
	link->auto_record = true;
	configuration_store->write_param(ParamID::CAM_RESOLUTION, 3U);
	bring_on();
	CHECK_EQUAL(0U, link->cuts_while_recording);
	CHECK_EQUAL(2U, link->rail_ons);
	CHECK_EQUAL(2U, (unsigned int)link->resolution_active);  // 1080P30, its 3rd option
	CHECK_TRUE(cam->is_recording_verified());
	// A stop before the restart; after it the camera records on its own again,
	// so the TOGGLE start stops it and the card has it pressed once more.
	CHECK_EQUAL(3U, link->controls(POWER));
}

TEST(RunCamUartCard, CameraRestartedOncePerSessionEvenIfItForgets) {
	link->forgets_resolution = true;  // accepts the write, keeps the old value
	configuration_store->write_param(ParamID::CAM_RESOLUTION, 2U);
	bring_on();
	CHECK_EQUAL(1U, link->resolution_writes);
	CHECK_EQUAL(2U, link->rail_ons);  // one restart, not a loop
	CHECK(RunCamUartDevice::Resolution::R1080P60 == cam->resolution_read());
	CHECK_TRUE(cam->is_recording_verified());
}

TEST(RunCamUartCard, RefusedResolutionIsNotARestart) {
	link->refuses_resolution = true;
	configuration_store->write_param(ParamID::CAM_RESOLUTION, 2U);
	bring_on();
	CHECK_EQUAL(1U, link->resolution_writes);
	CHECK_EQUAL(1U, link->rail_ons);
	CHECK_FALSE(cam->was_restarted());
	CHECK_TRUE(cam->is_recording_verified());
}

TEST(RunCamUartCard, SessionStatusIsPublishedWhenItEnds) {
	CHECK_EQUAL(255U, status(ParamID::CAM_LAST_OUTCOME));  // no session yet
	bring_on();
	cam->power_off();
	run_until_cut(30000);
	CHECK_EQUAL(0U, status(ParamID::CAM_LAST_OUTCOME));  // OK
	CHECK_EQUAL((unsigned int)cam->last_free_mb(), status(ParamID::CAM_SD_FREE_MB));
	CHECK_EQUAL(15079U, status(ParamID::CAM_SD_TOTAL_MB));
	CHECK_TRUE(status(ParamID::CAM_REC_TIME_LEFT_S) > 0 && status(ParamID::CAM_REC_TIME_LEFT_S) != 0xFFFFFFFFU);
	CHECK_EQUAL(1U, status(ParamID::CAM_RESOLUTION_READ));
	CHECK_EQUAL(0x0077U, status(ParamID::CAM_FEATURES));
}

TEST(RunCamUartCard, FullCardStatusIsPublished) {
	link->remaining_s = 0;
	cam->power_on();
	run_for(RunCamUartDevice::BOOT_MIN_MS + RunCamUartDevice::SETTING_POLL_MS);
	CHECK_EQUAL(2U, status(ParamID::CAM_LAST_OUTCOME));  // NO_SPACE
	CHECK_EQUAL(0U, status(ParamID::CAM_REC_TIME_LEFT_S));
	CHECK_EQUAL(0xFFFFFFFFU, status(ParamID::CAM_SD_FREE_MB));  // never read
	CHECK_EQUAL(0U, status(ParamID::CAM_RESOLUTION_READ));      // failed before it was read
}

TEST(RunCamUartCard, SilentCameraStatusIsPublished) {
	link->info_answer.clear();
	cam->power_on();
	run_for(RunCamUartDevice::BOOT_TIMEOUT_MS);
	CHECK_EQUAL(1U, status(ParamID::CAM_LAST_OUTCOME));  // NO_ANSWER
	CHECK_EQUAL(0U, status(ParamID::CAM_FEATURES));
	CHECK_EQUAL(0xFFFFFFFFU, status(ParamID::CAM_SD_TOTAL_MB));
}

TEST(RunCamUartCard, BenchHoldPublishesNothing) {
	CHECK_TRUE(cam->bench_power(true));
	cam->power_off_blocking();
	CHECK_FALSE(link->rail);
	CHECK_EQUAL(255U, status(ParamID::CAM_LAST_OUTCOME));
}

TEST(RunCamUartCard, CorruptSettingsAnswerIsAskedAgainAtOnce) {
	// A frame that fails its CRC ends the wait at once: the request goes again
	// without waiting out its timeout.
	cam->power_on();
	run_for(RunCamUartDevice::BOOT_MIN_MS - 1);
	link->lost_answers = 1;
	const uint8_t bad[] = { 0xCC, 0x00, 0x03, 0x0C, 0x39, 0x00, 0x00 };  // CRC wrong
	run_for(1);                                                          // the remaining-time request is out
	CHECK_EQUAL(1U, link->setting_requests());
	link->rx.insert(link->rx.end(), bad, bad + sizeof(bad));
	run_for(RunCamUartDevice::SETTING_POLL_MS);
	CHECK_TRUE(cam->corrupt_answers() >= 1U);
	CHECK_EQUAL(2U, link->setting_requests());       // asked again, and this time answered
	run_for(2 * RunCamUartDevice::SETTING_POLL_MS);  // then the resolution read
	CHECK_EQUAL(1U, link->controls(POWER));
	CHECK_TRUE(cam->last_remaining_s() > 0);
}

TEST(RunCamUartCard, LostCardCheckAnswerIsAskedOnceMore) {
	link->lose_answer_to = runcam::setting::REMAINING_TIME;
	cam->power_on();
	run_for(RunCamUartDevice::BOOT_MIN_MS + RunCamUartDevice::SETTING_TIMEOUT_MS - 1);
	CHECK_EQUAL(1U, link->setting_requests());
	run_for(1);
	CHECK_EQUAL(2U, link->setting_requests());  // the repeat
	run_until_settled(40000);
	CHECK_TRUE(cam->is_recording_verified());
	CHECK_TRUE(cam->last_remaining_s() > 0);  // the card was checked after all
	CHECK_EQUAL(1U, link->controls(POWER));
}

TEST(RunCamUartCard, LostVerificationAnswerIsAskedOnceMoreNotGuessed) {
	link->lose_answer_to = runcam::setting::SDCARD_CAPACITY;
	bring_on();
	CHECK_TRUE(cam->is_recording_verified());  // measured, from the repeated read
	CHECK_EQUAL(1U, link->controls(POWER));
	CHECK_EQUAL(5U, link->setting_requests());  // remaining, resolution, free (lost), free again, free
}

TEST(RunCamUartCard, LostStopVerificationAnswerStillEndsInAVerifiedStop) {
	bring_on();
	link->lose_answer_to = runcam::setting::SDCARD_CAPACITY;
	cam->power_off();
	run_until_cut(30000);
	CHECK_TRUE(cam->is_stop_verified());
	CHECK_EQUAL(2U, link->controls(POWER));
	CHECK_FALSE(link->recording_at_cut);
}

/*
 * The same device under the real CAMService: the two outcomes of a session.
 * A camera that never answers must still close its session, with an OFF
 * entry and the rail cut; one that answers runs the ON / OFF cycle.
 */
TEST_GROUP(RunCamUartService) {
	static constexpr unsigned int PERIOD_ON_S = 60;
	static constexpr unsigned int PERIOD_OFF_S = 300;
	// The button sequence the service charges to every period (cam_service.cpp).
	static constexpr unsigned int SERVICE_SEQ_MS = PWR_DELAY + PWR_BUTT_DELAY;

	FakeConfigurationStore *fake_config_store;
	FakeTimer *fake_timer;
	FakeLog *fake_logger;
	FakeRTC *fake_rtc;
	FakeBatteryMonitor *fake_battery_monitor;
	FakeRunCamLink *link;
	RunCamUartDevice *cam;

	void setup() {
		ServiceManager::reset();
		fake_timer = new FakeTimer;
		system_timer = fake_timer;
		fake_rtc = new FakeRTC;
		rtc = fake_rtc;
		fake_rtc->settime(1580083200);  // 27/01/2020 00:00:00
		fake_config_store = new FakeConfigurationStore;
		configuration_store = fake_config_store;
		configuration_store->init();
		fake_logger = new FakeLog;
		fake_logger->create();
		fake_battery_monitor = new FakeBatteryMonitor;
		battery_monitor = fake_battery_monitor;
		fake_battery_monitor->set_values(/*level*/ 100, /*mv*/ 4100, /*is_low*/ false);
		system_scheduler = new Scheduler(system_timer);
		fake_timer->start();
		link = new FakeRunCamLink;
		cam = new RunCamUartDevice(*link);

		configuration_store->write_param(ParamID::CAM_ENABLE, true);
		configuration_store->write_param(ParamID::CAM_PERIOD_ON, PERIOD_ON_S);
		configuration_store->write_param(ParamID::CAM_PERIOD_OFF, PERIOD_OFF_S);
		configuration_store->write_param(ParamID::CAM_TRIGGER_ON_SURFACED, false);
		configuration_store->write_param(ParamID::CAM_TRIGGER_ON_AXL_WAKEUP, false);
		configuration_store->write_param(ParamID::LB_EN, false);
	}

	void teardown() {
		mock().checkExpectations();
		mock().clear();
		mock().ignoreOtherCalls();
		delete cam;
		delete link;
		delete system_scheduler;
		delete fake_timer;
		delete fake_config_store;
		delete fake_logger;
		delete fake_rtc;
		delete fake_battery_monitor;
		system_scheduler = nullptr;
		system_timer = nullptr;
		rtc = nullptr;
		configuration_store = nullptr;
		battery_monitor = nullptr;
		mock().clear();
	}

	void run_for(unsigned int ms) {
		for (unsigned int i = 0; i < ms; i++) {
			fake_timer->increment_counter(1);
			system_scheduler->run();
		}
	}

	CAMLogEntry log_entry(unsigned int index) {
		CAMLogEntry entry;
		fake_logger->read(&entry, (int)index);
		return entry;
	}
};

TEST(RunCamUartService, SilentCameraClosesTheSessionWithAnOffEntry) {
	CAMService s(*cam, fake_logger);
	s.start([](ServiceEvent &) {});
	run_for(PERIOD_OFF_S * 1000 + SERVICE_SEQ_MS);  // the cycle starts with the OFF period
	CHECK_TRUE(link->rail);
	CHECK_EQUAL(0, fake_logger->num_entries());
	run_for(RunCamUartDevice::BOOT_TIMEOUT_MS);
	CHECK_FALSE(link->rail);
	CHECK_EQUAL(1, fake_logger->num_entries());
	CHECK_EQUAL((unsigned int)CAMEventType::OFF, (unsigned int)log_entry(0).info.event_type);
	CHECK_EQUAL(0U, link->all_controls());
	s.stop();
	cam->unsubscribe(s);  // s dies here, cam in teardown
}

TEST(RunCamUartService, AnsweringCameraRunsTheCycle) {
	link->info_answer = BLIND_INFO;
	CAMService s(*cam, fake_logger);
	s.start([](ServiceEvent &) {});
	run_for(PERIOD_OFF_S * 1000 + SERVICE_SEQ_MS);
	run_for(RunCamUartDevice::BOOT_MIN_MS + RunCamUartDevice::RECORD_SETTLE_MS);
	CHECK_TRUE(cam->is_powered_on());
	CHECK_EQUAL(1, fake_logger->num_entries());
	CHECK_EQUAL((unsigned int)CAMEventType::ON, (unsigned int)log_entry(0).info.event_type);
	CHECK_EQUAL(1U, link->controls(POWER));

	run_for(PERIOD_ON_S * 1000 + SERVICE_SEQ_MS);  // the ON period, then the stop
	CHECK_EQUAL(2U, link->controls(POWER));
	CHECK_TRUE(link->rail);
	run_for(RunCamUartDevice::STOP_SETTLE_MS);
	CHECK_FALSE(link->rail);
	CHECK_EQUAL(2, fake_logger->num_entries());
	CHECK_EQUAL((unsigned int)CAMEventType::OFF, (unsigned int)log_entry(1).info.event_type);
	s.stop();
	cam->unsubscribe(s);
}

TEST(RunCamUartService, CardCheckedCameraRunsTheCycle) {
	link->info_answer = SPLIT4_INFO;
	link->camera = true;
	CAMService s(*cam, fake_logger);
	s.start([](ServiceEvent &) {});
	run_for(PERIOD_OFF_S * 1000 + SERVICE_SEQ_MS);
	CHECK_TRUE(link->rail);
	for (unsigned int i = 0; i < 40000 && !cam->is_powered_on(); i++)
		run_for(1);
	CHECK_TRUE(cam->is_powered_on());
	CHECK_TRUE(cam->is_recording_verified());
	STRCMP_EQUAL("20200126T160000.0", link->clock.c_str());  // the RTC's 2020-01-27 00:00 UTC, less 8 h
	CHECK_EQUAL(1, fake_logger->num_entries());
	CHECK_EQUAL((unsigned int)CAMEventType::ON, (unsigned int)log_entry(0).info.event_type);

	run_for(PERIOD_ON_S * 1000 + SERVICE_SEQ_MS);
	CHECK_EQUAL(2U, link->controls(POWER));
	for (unsigned int i = 0; i < 40000 && link->rail; i++)
		run_for(1);
	CHECK_FALSE(link->rail);
	CHECK_FALSE(link->recording_at_cut);
	CHECK_EQUAL(2, fake_logger->num_entries());
	CHECK_EQUAL((unsigned int)CAMEventType::OFF, (unsigned int)log_entry(1).info.event_type);
	s.stop();
	cam->unsubscribe(s);
}

TEST(RunCamUartService, FullCardClosesTheSessionWithAnOffEntry) {
	link->info_answer = SPLIT4_INFO;
	link->camera = true;
	link->remaining_s = 0;
	CAMService s(*cam, fake_logger);
	s.start([](ServiceEvent &) {});
	run_for(PERIOD_OFF_S * 1000 + SERVICE_SEQ_MS);
	run_for(RunCamUartDevice::BOOT_MIN_MS + RunCamUartDevice::SETTING_POLL_MS);
	CHECK_FALSE(link->rail);
	CHECK_EQUAL(0U, link->all_controls());
	CHECK_EQUAL(1, fake_logger->num_entries());
	CHECK_EQUAL((unsigned int)CAMEventType::OFF, (unsigned int)log_entry(0).info.event_type);
	s.stop();
	cam->unsubscribe(s);
}

/*
 * CAMTST, the configuration tool's camera test: the real session run from a
 * DTE request, answered when the camera is off again with what its card said.
 * What is checked: nothing is answered at once but a refusal; the answer
 * carries the outcome AND the measurements, a failing camera included; the
 * camera is always off when it comes; and a test cut short by leaving the
 * configuration state answers nothing more.
 */
TEST_GROUP(RunCamUartDte) {
	FakeTimer *fake_timer;
	FakeConfigurationStore *fake_config_store;
	FakeRTC *fake_rtc;
	FakeRunCamLink *link;
	RunCamUartDevice *cam;
	DTEHandler *dte;
	std::vector<std::string> answers;

	void setup() {
		fake_timer = new FakeTimer;
		system_timer = fake_timer;
		fake_config_store = new FakeConfigurationStore;
		configuration_store = fake_config_store;
		configuration_store->init();
		fake_rtc = new FakeRTC;
		rtc = fake_rtc;
		fake_rtc->settime(1761510000);
		system_scheduler = new Scheduler(system_timer);
		fake_timer->start();
		link = new FakeRunCamLink;
		link->info_answer = SPLIT4_INFO;
		link->camera = true;
		cam = new RunCamUartDevice(*link);
		runcam_uart_device = cam;
		dte = new DTEHandler();
		answers.clear();
		dte->set_async_write([this](const std::string &msg) { answers.push_back(msg); });
	}

	void teardown() {
		mock().checkExpectations();
		mock().clear();
		mock().ignoreOtherCalls();
		delete dte;
		delete cam;
		runcam_uart_device = nullptr;
		delete link;
		delete system_scheduler;
		delete fake_timer;
		delete fake_config_store;
		delete fake_rtc;
		system_scheduler = nullptr;
		system_timer = nullptr;
		configuration_store = nullptr;
		rtc = nullptr;
		mock().clear();
	}

	void run_for(unsigned int ms) {
		for (unsigned int i = 0; i < ms; i++) {
			fake_timer->increment_counter(1);
			system_scheduler->run();
		}
	}

	std::string request() {
		std::string resp;
		dte->handle_dte_message(DTEEncoder::encode(DTECommand::CAMTST_REQ), resp);
		return resp;
	}

	/// @brief Run until the test answers, at most @p limit_ms.
	void run_until_answered(unsigned int limit_ms) {
		for (unsigned int i = 0; i < limit_ms && answers.empty(); i++)
			run_for(1);
	}

	/// @brief The fields of a CAMTST answer, "$O;CAMTST#LEN;a,b,c\r" -> {a, b, c}.
	static std::vector<std::string> fields(const std::string &answer) {
		std::vector<std::string> out;
		STRCMP_EQUAL("$O;CAMTST#", answer.substr(0, 10).c_str());
		std::string body = answer.substr(answer.find(';', 3) + 1);
		if (!body.empty() && body.back() == '\r') body.pop_back();
		size_t start = 0;
		for (size_t comma; (comma = body.find(',', start)) != std::string::npos; start = comma + 1)
			out.push_back(body.substr(start, comma - start));
		out.push_back(body.substr(start));
		return out;
	}
};

TEST(RunCamUartDte, TestRunsTheRealSessionAndAnswersOnceTheCameraIsOff) {
	CHECK_TRUE(request().empty());  // answered later
	CHECK_TRUE(link->rail);
	run_until_answered(90000);
	CHECK_EQUAL(1U, answers.size());
	const std::vector<std::string> f = fields(answers[0]);
	CHECK_EQUAL(11U, f.size());
	STRCMP_EQUAL("0", f[0].c_str());                             // status OK
	STRCMP_EQUAL("119", f[1].c_str());                           // features 0x0077
	CHECK_TRUE(std::stoi(f[2]) > 0 && std::stoi(f[2]) < 14707);  // free, after a few seconds of video
	STRCMP_EQUAL("15079", f[3].c_str());
	CHECK_TRUE(std::stoi(f[4]) > 0);
	STRCMP_EQUAL("1", f[5].c_str());  // recording seen on the card
	STRCMP_EQUAL("1", f[6].c_str());  // and its stop
	STRCMP_EQUAL("1", f[7].c_str());  // clock set
	STRCMP_EQUAL("1", f[8].c_str());  // one start: CAP06 matches the camera
	CHECK_TRUE(std::stoul(f[9]) > RunCamUartDevice::BOOT_MIN_MS);
	STRCMP_EQUAL("1", f[10].c_str());  // the camera's resolution: 1080P60
	CHECK_FALSE(link->rail);
	CHECK_FALSE(link->recording_at_cut);
	STRCMP_EQUAL("20251026T122000.0", link->clock.c_str());
}

TEST(RunCamUartDte, TestAppliesTheResolutionAndReportsIt) {
	configuration_store->write_param(ParamID::CAM_RESOLUTION, 3U);  // 1080P30
	request();
	run_until_answered(180000);
	const std::vector<std::string> f = fields(answers.at(0));
	STRCMP_EQUAL("0", f[0].c_str());
	STRCMP_EQUAL("3", f[10].c_str());
	CHECK_EQUAL(2U, link->rail_ons);                         // the restart that took it up
	CHECK_EQUAL(2U, (unsigned int)link->resolution_active);  // 1080P30, its 3rd option
	CHECK_FALSE(link->rail);
}

TEST(RunCamUartDte, FullCardIsReportedWithWhatWasRead) {
	link->remaining_s = 0;
	request();
	run_until_answered(90000);
	const std::vector<std::string> f = fields(answers.at(0));
	STRCMP_EQUAL("2", f[0].c_str());  // NO_SPACE
	STRCMP_EQUAL("0", f[4].c_str());  // remaining 0, as the camera said
	STRCMP_EQUAL("0", f[5].c_str());
	STRCMP_EQUAL("0", f[8].c_str());  // never pressed
	CHECK_EQUAL(0U, link->all_controls());
	CHECK_FALSE(link->rail);
}

TEST(RunCamUartDte, SilentCameraIsNoAnswerAndNothingWasRead) {
	link->info_answer.clear();
	request();
	run_until_answered(90000);
	const std::vector<std::string> f = fields(answers.at(0));
	STRCMP_EQUAL("1", f[0].c_str());  // NO_ANSWER
	STRCMP_EQUAL("0", f[1].c_str());
	STRCMP_EQUAL("-1", f[2].c_str());
	STRCMP_EQUAL("-1", f[3].c_str());
	STRCMP_EQUAL("-1", f[4].c_str());
	CHECK_FALSE(link->rail);
}

TEST(RunCamUartDte, CameraThatDoesNotRecordIsReportedAfterTwoStarts) {
	link->deaf = true;
	request();
	run_until_answered(90000);
	const std::vector<std::string> f = fields(answers.at(0));
	STRCMP_EQUAL("3", f[0].c_str());  // NOT_RECORDING
	STRCMP_EQUAL("2", f[8].c_str());
	CHECK_FALSE(link->rail);
}

TEST(RunCamUartDte, CameraThatAutoRecordsAgainstCap06IsOkWithTwoStarts) {
	link->auto_record = true;  // CAP06 says TOGGLE
	request();
	run_until_answered(90000);
	const std::vector<std::string> f = fields(answers.at(0));
	STRCMP_EQUAL("0", f[0].c_str());
	STRCMP_EQUAL("2", f[8].c_str());  // the hint for the operator: set CAP06=1 or the camera's auto-record off
	CHECK_FALSE(link->recording_at_cut);
}

TEST(RunCamUartDte, UnreadableCardIsUnverified) {
	link->settings_answer = false;
	request();
	run_until_answered(90000);
	const std::vector<std::string> f = fields(answers.at(0));
	STRCMP_EQUAL("4", f[0].c_str());  // UNVERIFIED
	STRCMP_EQUAL("0", f[5].c_str());
	STRCMP_EQUAL("0", f[6].c_str());
	CHECK_FALSE(link->rail);
}

TEST(RunCamUartDte, BusyCameraIsRefusedAtOnce) {
	CHECK_TRUE(cam->bench_power(true));
	const std::string resp = request();
	CHECK_FALSE(resp.empty());
	STRCMP_EQUAL("7", fields(resp)[0].c_str());  // BUSY
	CHECK_TRUE(answers.empty());
	CHECK_TRUE(cam->bench_power(false));

	CHECK_TRUE(request().empty());                    // a test now runs...
	STRCMP_EQUAL("7", fields(request())[0].c_str());  // ...and a second one is refused
	run_until_answered(90000);
	CHECK_EQUAL(1U, answers.size());
}

TEST(RunCamUartDte, LeavingConfigurationCutsTheTestAndAnswersNothingMore) {
	request();
	run_for(RunCamUartDevice::BOOT_MIN_MS + 1000);  // started, recording
	CHECK_TRUE(link->recording);
	// Pressed at 7300 ms (card check, resolution, clock): its settle, then the stop's.
	mock().expectOneCall("delay_ms").withParameter("ms", 7300 + RunCamUartDevice::RECORD_SETTLE_MS - 8000);
	mock().expectOneCall("delay_ms").withParameter("ms", RunCamUartDevice::STOP_SETTLE_MS);
	dte->abort_camera_test();
	CHECK_FALSE(link->rail);
	CHECK_FALSE(link->recording_at_cut);  // stopped before the cut
	run_for(90000);
	CHECK_TRUE(answers.empty());
}
