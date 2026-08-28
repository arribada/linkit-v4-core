/**
 * @file turtle_main.cpp
 * @brief Point d'entrée pour la simulation tortue avec génération de rapport HTML
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
#include "CppUTest/TestOutput.h"

#include <fstream>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <vector>
#include <string>
#include <cstring>

// Global contexts
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
GPSService *gps_service = nullptr;  // Forward-declared in dte_handler.hpp; null in tests (used by GNSSBCKP DTE handler)
KineisDevice *kineis_device_instance = nullptr;
LoRaDevice *lora_device_instance = nullptr;
BaseDebugMode g_debug_mode;
Buzzer *buzzer_ctl;

MockSupportPlugin mockPlugin;

/**
 * @brief Structure to store test results for HTML report
 */
struct TestResultInfo {
	std::string group_name;
	std::string test_name;
	bool passed;
	std::string failure_message;
	double duration_ms;
};

// Global storage for HTML results (accessible from custom output)
static std::vector<TestResultInfo> g_test_results;
static std::string g_current_group;
static std::string g_current_test;
static std::string g_current_failure;
static clock_t g_test_start_time;
static int g_total_tests = 0;
static int g_passed_tests = 0;
static int g_failed_tests = 0;
static bool g_test_failed = false;

/**
 * @brief Custom test plugin to capture results for HTML report
 */
class HtmlReportPlugin : public TestPlugin {
public:
	HtmlReportPlugin(const SimpleString &name) : TestPlugin(name) {}

	void preTestAction(UtestShell &test, TestResult & /*result*/) override {
		g_current_group = test.getGroup().asCharString();
		g_current_test = test.getName().asCharString();
		g_current_failure.clear();
		g_test_failed = false;
		g_test_start_time = clock();
	}

	void postTestAction(UtestShell & /*test*/, TestResult &result) override {
		double duration = (double)(clock() - g_test_start_time) / CLOCKS_PER_SEC * 1000.0;

		// Check if this specific test failed
		bool test_passed = !g_test_failed;

		TestResultInfo tr;
		tr.group_name = g_current_group;
		tr.test_name = g_current_test;
		tr.passed = test_passed;
		tr.failure_message = g_current_failure;
		tr.duration_ms = duration;
		g_test_results.push_back(tr);

		g_total_tests++;
		if (test_passed)
			g_passed_tests++;
		else
			g_failed_tests++;

		// Suppress unused parameter warning
		(void)result;
	}
};

/**
 * @brief Custom test output to capture failures
 */
class HtmlTestOutput : public ConsoleTestOutput {
public:
	void printFailure(const TestFailure &failure) override {
		g_test_failed = true;
		std::ostringstream oss;
		oss << failure.getFileName().asCharString() << ":" << failure.getFailureLineNumber() << " - "
		    << failure.getMessage().asCharString();
		g_current_failure = oss.str();
		ConsoleTestOutput::printFailure(failure);
	}
};

/**
 * @brief Generate HTML report from test results
 */
void generate_html_report(const std::string &filename) {
	std::ofstream html(filename);
	if (!html.is_open()) {
		printf("ERROR: Cannot create HTML report file: %s\n", filename.c_str());
		return;
	}

	// Get current time
	time_t now = time(nullptr);
	char time_str[64];
	strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", localtime(&now));

	double pass_rate = g_total_tests > 0 ? (100.0 * g_passed_tests / g_total_tests) : 0.0;

	html << R"(<!DOCTYPE html>
<html lang="fr">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Turtle Simulation Report</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
            min-height: 100vh;
            color: #eee;
            padding: 20px;
        }
        .container { max-width: 1200px; margin: 0 auto; }
        h1 {
            text-align: center;
            margin-bottom: 30px;
            color: #00d4ff;
            text-shadow: 0 0 20px rgba(0,212,255,0.5);
        }
        .summary {
            display: flex;
            justify-content: center;
            gap: 30px;
            margin-bottom: 40px;
            flex-wrap: wrap;
        }
        .stat-card {
            background: rgba(255,255,255,0.1);
            border-radius: 15px;
            padding: 25px 40px;
            text-align: center;
            backdrop-filter: blur(10px);
            border: 1px solid rgba(255,255,255,0.1);
            transition: transform 0.3s;
        }
        .stat-card:hover { transform: translateY(-5px); }
        .stat-value {
            font-size: 48px;
            font-weight: bold;
            margin-bottom: 5px;
        }
        .stat-label { font-size: 14px; text-transform: uppercase; opacity: 0.7; }
        .passed .stat-value { color: #00ff88; }
        .failed .stat-value { color: #ff4757; }
        .total .stat-value { color: #00d4ff; }
        .rate .stat-value { color: #ffa502; }

        .progress-bar {
            width: 100%;
            height: 20px;
            background: rgba(255,255,255,0.1);
            border-radius: 10px;
            margin-bottom: 40px;
            overflow: hidden;
        }
        .progress-fill {
            height: 100%;
            background: linear-gradient(90deg, #00ff88, #00d4ff);
            border-radius: 10px;
            transition: width 1s ease;
        }

        .tests-table {
            width: 100%;
            border-collapse: collapse;
            background: rgba(255,255,255,0.05);
            border-radius: 15px;
            overflow: hidden;
        }
        .tests-table th {
            background: rgba(0,212,255,0.2);
            padding: 15px;
            text-align: left;
            font-weight: 600;
        }
        .tests-table td {
            padding: 12px 15px;
            border-bottom: 1px solid rgba(255,255,255,0.05);
        }
        .tests-table tr:hover { background: rgba(255,255,255,0.05); }

        .status-badge {
            display: inline-block;
            padding: 5px 15px;
            border-radius: 20px;
            font-size: 12px;
            font-weight: bold;
        }
        .status-pass { background: rgba(0,255,136,0.2); color: #00ff88; }
        .status-fail { background: rgba(255,71,87,0.2); color: #ff4757; }

        .duration { color: #888; font-size: 12px; }
        .failure-msg {
            color: #ff4757;
            font-size: 12px;
            margin-top: 5px;
            font-family: monospace;
            white-space: pre-wrap;
            max-width: 600px;
            word-wrap: break-word;
        }

        .footer {
            text-align: center;
            margin-top: 40px;
            opacity: 0.5;
            font-size: 12px;
        }

        .turtle-icon { font-size: 40px; margin-right: 10px; }
    </style>
</head>
<body>
    <div class="container">
        <h1><span class="turtle-icon">&#x1F422;</span>Turtle Simulation Report</h1>
        <p style="text-align:center; margin-bottom:30px; opacity:0.7;">)"
	     << time_str << R"(</p>

        <div class="summary">
            <div class="stat-card total">
                <div class="stat-value">)"
	     << g_total_tests << R"(</div>
                <div class="stat-label">Total Tests</div>
            </div>
            <div class="stat-card passed">
                <div class="stat-value">)"
	     << g_passed_tests << R"(</div>
                <div class="stat-label">Passed</div>
            </div>
            <div class="stat-card failed">
                <div class="stat-value">)"
	     << g_failed_tests << R"(</div>
                <div class="stat-label">Failed</div>
            </div>
            <div class="stat-card rate">
                <div class="stat-value">)"
	     << std::fixed << std::setprecision(1) << pass_rate << R"(%</div>
                <div class="stat-label">Pass Rate</div>
            </div>
        </div>

        <div class="progress-bar">
            <div class="progress-fill" style="width: )"
	     << pass_rate << R"(%;"></div>
        </div>

        <table class="tests-table">
            <thead>
                <tr>
                    <th>Test Group</th>
                    <th>Test Name</th>
                    <th>Status</th>
                    <th>Duration</th>
                </tr>
            </thead>
            <tbody>
)";

	for (const auto &test : g_test_results) {
		html << "                <tr>\n"
		     << "                    <td>" << test.group_name << "</td>\n"
		     << "                    <td>" << test.test_name;
		if (!test.passed && !test.failure_message.empty()) {
			html << "<div class=\"failure-msg\">" << test.failure_message << "</div>";
		}
		html << "</td>\n"
		     << "                    <td><span class=\"status-badge "
		     << (test.passed ? "status-pass\">PASS" : "status-fail\">FAIL") << "</span></td>\n"
		     << "                    <td class=\"duration\">" << std::fixed << std::setprecision(1) << test.duration_ms
		     << " ms</td>\n"
		     << "                </tr>\n";
	}

	html << R"(            </tbody>
        </table>

        <div class="footer">
            Generated by LinkIt V4 Turtle Simulation Test Suite<br>
            CLS GenTracker Firmware
        </div>
    </div>
</body>
</html>
)";

	html.close();
	printf("\n=== HTML Report generated: %s ===\n", filename.c_str());
}

int main(int argc, char **argv) {
	ConsoleLog con_log;
	DebugLogger::console_log = &con_log;

	// Install plugins
	HtmlReportPlugin htmlPlugin("HtmlReport");
	TestRegistry::getCurrentRegistry()->installPlugin(&mockPlugin);
	TestRegistry::getCurrentRegistry()->installPlugin(&htmlPlugin);

	// Run tests
	int exit_code = CommandLineTestRunner::RunAllTests(argc, argv);

	// Generate HTML report
	std::string report_path = "turtle_simulation_report.html";

	// Check if a custom path was provided via environment variable
	const char *custom_path = getenv("TURTLE_REPORT_PATH");
	if (custom_path != nullptr) {
		report_path = custom_path;
	}

	generate_html_report(report_path);

	// Print summary to console
	printf("\n");
	printf("========================================\n");
	printf("       TURTLE SIMULATION SUMMARY\n");
	printf("========================================\n");
	printf("  Total:  %d tests\n", g_total_tests);
	printf("  Passed: %d tests\n", g_passed_tests);
	printf("  Failed: %d tests\n", g_failed_tests);
	printf("  Rate:   %.1f%%\n", g_total_tests > 0 ? (100.0 * g_passed_tests / g_total_tests) : 0.0);
	printf("========================================\n");
	printf("  Report: %s\n", report_path.c_str());
	printf("========================================\n");

	return exit_code;
}
