/**
 * @file rspb_main.cpp
 * @brief Entry point for the RSPB bird-tracker long-term mortality simulation.
 *
 * Mirrors turtle_main.cpp: defines the firmware globals the linked core needs
 * and runs the CppUTest registry. The *rich* mission report (HTML + CSV +
 * exported on-device mortality log) is written by rspb_simulation.cpp itself —
 * this file only wires up the runtime and prints a short console summary.
 *
 * Run:
 *   ./build/RSPBSimulation -v
 *   RSPB_REPORT_DIR=/some/dir ./build/RSPBSimulation   # redirect artefacts
 */

#include "filesystem.hpp"
#include "console_log.hpp"
#include "timer.hpp"
#include "config_store.hpp"
#include "service_scheduler.hpp"
#include "dte_handler.hpp"
#include "scheduler.hpp"
#include "logger.hpp"
#include "ble_service.hpp"
#include "ota_file_updater.hpp"
#include "rgb_led.hpp"
#include "led.hpp"
#include "switch.hpp"
#include "memory_access.hpp"
#include "rtc.hpp"
#include "battery.hpp"
#include "debug.hpp"
#include "gpio_buzzer.hpp"
#include "gps.hpp"
#include "lora_rak3172.hpp"

#include "CppUTest/CommandLineTestRunner.h"
#include "CppUTest/TestRegistry.h"
#include "CppUTestExt/MockSupportPlugin.h"

#include <cstdio>

// Global contexts (same set turtle_main.cpp defines — the linked firmware core
// resolves these symbols at link time even though the mortality mission only
// drives a subset).
FileSystem *main_filesystem;
Timer *system_timer;
ConfigurationStore *configuration_store;
ServiceScheduler *comms_scheduler;
DTEHandler *dte_handler;
Scheduler *system_scheduler;
BLEService *ble_service;
OTAFileUpdater *ota_updater;
Switch *reed_switch;
RGBLed *status_led;
MemoryAccess *memory_access;
RTC *rtc;
BatteryMonitor *battery_monitor;
GPSDevice *gps_device;
GPSService *gps_service = nullptr;
KineisDevice *kineis_device_instance = nullptr;
LoRaDevice *lora_device_instance = nullptr;
BaseDebugMode g_debug_mode;
Buzzer *buzzer_ctl;

MockSupportPlugin mockPlugin;

int main(int argc, char **argv) {
	ConsoleLog con_log;
	DebugLogger::console_log = &con_log;

	TestRegistry::getCurrentRegistry()->installPlugin(&mockPlugin);

	printf("\n");
	printf("=====================================================\n");
	printf("   RSPB BIRD-TRACKER — LONG-TERM MORTALITY SIMULATION\n");
	printf("=====================================================\n");

	int exit_code = CommandLineTestRunner::RunAllTests(argc, argv);

	printf("\n");
	printf("=====================================================\n");
	printf("  Artefacts written to the working directory:\n");
	printf("    rspb_mortality_report.html  (visual mission report)\n");
	printf("    rspb_mortality.csv          (per-session metrics)\n");
	printf("    rspb_mortality_log.csv      (on-device MortalityService log)\n");
	printf("    rspb_mission.log            (human-readable event log)\n");
	printf("  Override the output directory with RSPB_REPORT_DIR=<dir>\n");
	printf("=====================================================\n");

	return exit_code;
}
