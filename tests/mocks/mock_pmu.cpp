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

void PMU::print_stack() {}

const char *PMU::last_crash_str() {
	return "none";
}

void PMU::save_stack(PMULogType type) {
	// No-op in host tests — real PMU logs the crash type to .noinit RAM
	// for post-mortem retrieval; not useful when the test process is short-
	// lived. Mock provided so safety-net code calling save_stack() before
	// PMU::reset() links cleanly.
	(void)type;
}

// Parameterizable so a test can exercise the boot paths that branch on the
// reset cause. It used to hardcode POWER_ON — which is exactly the value that
// hid the storage-wake bug, since a System OFF wake also decoded as POWER_ON.
static ResetCause s_mock_reset_cause = ResetCause::POWER_ON;
static uint32_t s_mock_resetreas = 0;
static uint32_t s_mock_gpregret = 0;

void mock_pmu_set_reset_cause(ResetCause cause, uint32_t resetreas, uint32_t gpregret) {
	s_mock_reset_cause = cause;
	s_mock_resetreas = resetreas;
	s_mock_gpregret = gpregret;
}

ResetCause PMU::reset_cause() {
	return s_mock_reset_cause;
}

uint32_t PMU::reset_reason_raw() {
	return s_mock_resetreas;
}

uint32_t PMU::boot_gpregret() {
	return s_mock_gpregret;
}

const char *PMU::reset_cause_str() {
	switch (s_mock_reset_cause) {
	case ResetCause::HARD_RESET: return "Hard Reset";
	case ResetCause::WDT_RESET: return "WDT Reset";
	case ResetCause::SOFT_RESET: return "Soft Reset";
	case ResetCause::PSEUDO_POWER_ON: return "Pseudo Power On Reset";
	case ResetCause::POWER_ON: return "Power On Reset";
	default: return "UNKNOWN";
	}
}

const char *PMU::hardware_version() {
	return "simulator";
}

uint32_t PMU::device_identifier() {
	return 0x12345678;
}

uint64_t PMU::get_timestamp_ms() {
	if (system_timer) return system_timer->get_counter();
	static auto start = std::chrono::steady_clock::now();
	auto now = std::chrono::steady_clock::now();
	return std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
}

bool PMU::was_firmware_updated() {
	return false;
}

void PMU::set_firmware_updated_flag() {}

void PMU::reduce_power_rails() {}

void PMU::restore_power_rails() {}

int PMU::get_die_temperature_c() {
	return 25;  // Mock sentinel — matches the real PMU error fallback.
}
