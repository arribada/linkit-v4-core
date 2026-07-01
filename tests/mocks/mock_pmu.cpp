#include "pmu.hpp"

#include <chrono>
#include "CppUTestExt/MockSupport.h"
#include "timer.hpp"

// In the test build, PMU's millisecond timestamp tracks the (controllable)
// system timer so simulated time advances deterministically with the test's
// FakeTimer — no real sleeps needed for time-based logic (SWS dive timeout,
// calibration interval, heartbeats). Under LinuxTimer the counter advances in
// real milliseconds, so behaviour is unchanged. Falls back to wall-clock only
// before any timer is wired up.
extern Timer *system_timer;


void PMU::reset(bool dfu_mode) {
	mock().actualCall("reset").withParameter("dfu_mode", dfu_mode);
}

void PMU::powerdown() {
	mock().actualCall("powerdown");
}

void PMU::delay_ms(unsigned ms) {
	mock().actualCall("delay_ms").withParameter("ms", ms);
}

void PMU::delay_us(unsigned us) {
	mock().actualCall("delay_us").withParameter("us", us);
}

void PMU::start_watchdog() {
	mock().actualCall("start_watchdog");
}

void PMU::kick_watchdog() {
	// No-op: watchdog kick is a side-effect irrelevant to test assertions.
	// Previously mocked, but caused spurious failures when state transitions
	// (ErrorState, BatteryCriticalState) added extra kick_watchdog() calls.
}

void PMU::print_stack() {
}

void PMU::save_stack(PMULogType type) {
	// No-op in host tests — real PMU logs the crash type to .noinit RAM
	// for post-mortem retrieval; not useful when the test process is short-
	// lived. Mock provided so safety-net code calling save_stack() before
	// PMU::reset() links cleanly.
	(void)type;
}

ResetCause PMU::reset_cause() {
	return ResetCause::POWER_ON;
}

const char* PMU::reset_cause_str() {
	return "UNKNOWN";
}

const char* PMU::hardware_version() {
	return "simulator";
}

uint32_t PMU::device_identifier() {
	return 0x12345678;
}

uint64_t PMU::get_timestamp_ms() {
	if (system_timer)
		return system_timer->get_counter();
	static auto start = std::chrono::steady_clock::now();
	auto now = std::chrono::steady_clock::now();
	return std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
}

bool PMU::was_firmware_updated() {
	return false;
}

void PMU::set_firmware_updated_flag() {
}

void PMU::reduce_power_rails() {
}

void PMU::restore_power_rails() {
}

int PMU::get_die_temperature_c() {
	return 25;  // Mock sentinel — matches the real PMU error fallback.
}
