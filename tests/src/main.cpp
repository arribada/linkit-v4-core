/**
 * @file main.cpp
 * @brief TrackerTests entry point + HTML regression report generator.
 *
 * After every CommandLineTestRunner run, captures per-test results and
 * writes an HTML report to `trackertests_report.html` (overridable via
 * env `TRACKERTESTS_REPORT_PATH`). Report header carries firmware version
 * (`FW_APP_VERSION_STR_C`), git commit short SHA, branch, `git describe`
 * output, and build timestamp — all baked in at CMake configure time so
 * the report is unambiguous about which commit was tested.
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

#include "CppUTest/CommandLineTestRunner.h"
#include "CppUTest/TestRegistry.h"
#include "CppUTest/TestPlugin.h"
#include "CppUTest/TestResult.h"
#include "CppUTestExt/MockSupportPlugin.h"

#include <fstream>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <vector>
#include <string>
#include <map>
#include <algorithm>
#include <cstdlib>

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
GPSService *gps_service = nullptr;
KineisDevice *kineis_device_instance = nullptr;
LoRaDevice *lora_device_instance = nullptr;
#if ENABLE_MORTALITY_SENSOR
#include "mortality_service.hpp"
MortalityService *mortality_service = nullptr;
#endif
BaseDebugMode g_debug_mode;
Buzzer *buzzer_ctl;

MockSupportPlugin mockPlugin;

// === HTML report — per-test result capture =============================
namespace {

struct TestRecord {
	std::string group;
	std::string name;
	bool passed = true;
	std::string failure_message;
	double duration_ms = 0.0;
};

std::vector<TestRecord> g_results;
std::string g_current_group;
std::string g_current_test;
std::string g_current_failure;
bool g_current_failed = false;
clock_t g_current_start = 0;
clock_t g_run_start = 0;
std::string g_report_path;
std::string g_log_path;  // sidecar TSV log of per-test results
bool g_run_completed = false;

// Forward decl — defined further down.
void write_html_report(const std::string &path, bool live);

// Append a single record to the TSV log file. CppUTest's `-p` (process
// isolation) mode forks per test, so the plugin's g_results vector in the
// parent stays empty. The log file is shared filesystem state that bridges
// the fork: each child appends its record, then everyone reads the full
// log to rebuild the HTML. With `-p` runs being sequential, no race.
void append_log_record(const TestRecord &r) {
	if (g_log_path.empty()) return;
	std::ofstream f(g_log_path, std::ios::app);
	if (!f.is_open()) return;
	// Format: group<TAB>test<TAB>passed(0/1)<TAB>duration_ms
	f << r.group << "\t" << r.name << "\t" << (r.passed ? 1 : 0) << "\t" << std::fixed << std::setprecision(3)
	  << r.duration_ms << "\n";
}

// Load all records from the log file into g_results (replaces existing).
void load_log_records() {
	g_results.clear();
	if (g_log_path.empty()) return;
	std::ifstream f(g_log_path);
	if (!f.is_open()) return;
	std::string line;
	while (std::getline(f, line)) {
		TestRecord r;
		auto t1 = line.find('\t');
		auto t2 = line.find('\t', t1 + 1);
		auto t3 = line.find('\t', t2 + 1);
		if (t1 == std::string::npos || t2 == std::string::npos || t3 == std::string::npos) continue;
		r.group = line.substr(0, t1);
		r.name = line.substr(t1 + 1, t2 - t1 - 1);
		r.passed = (line.substr(t2 + 1, t3 - t2 - 1) == "1");
		try {
			r.duration_ms = std::stod(line.substr(t3 + 1));
		} catch (...) {
			r.duration_ms = 0;
		}
		g_results.push_back(r);
	}
}

class HtmlReportPlugin : public TestPlugin {
	size_t m_failures_before = 0;

public:
	HtmlReportPlugin() : TestPlugin("HtmlReport") {}
	void preTestAction(UtestShell &test, TestResult &result) override {
		g_current_group = test.getGroup().asCharString();
		g_current_test = test.getName().asCharString();
		g_current_failure.clear();
		g_current_failed = false;
		g_current_start = clock();
		m_failures_before = result.getFailureCount();
		// Live update: write report so a browser refresh shows the
		// currently-running test before it finishes.
		write_html_report(g_report_path, /*live=*/true);
	}
	void postTestAction(UtestShell &, TestResult &result) override {
		TestRecord r;
		r.group = g_current_group;
		r.name = g_current_test;
		// New failures since pre-test → this test failed.
		r.passed = (result.getFailureCount() == m_failures_before);
		r.duration_ms = (double)(clock() - g_current_start) / CLOCKS_PER_SEC * 1000.0;
		// Append to shared log so -p forked children survive into the
		// parent's view via the log file.
		append_log_record(r);
		// Reload from log so live HTML shows ALL tests done so far,
		// including ones run in previous forked children.
		load_log_records();
		write_html_report(g_report_path, /*live=*/true);
	}
};

// Escape minimal HTML special chars in failure messages.
std::string html_escape(const std::string &s) {
	std::string out;
	out.reserve(s.size());
	for (char c : s) {
		switch (c) {
		case '<': out += "&lt;"; break;
		case '>': out += "&gt;"; break;
		case '&': out += "&amp;"; break;
		case '"': out += "&quot;"; break;
		default: out += c;
		}
	}
	return out;
}

void write_html_report(const std::string &path, bool live) {
	std::ofstream f(path);
	if (!f.is_open()) {
		// Don't spam on every live update — print once and continue.
		static bool printed = false;
		if (!printed) {
			printf("[trackertests-report] cannot open %s\n", path.c_str());
			printed = true;
		}
		return;
	}

	// Build metadata (CMake-baked defines; fall back to "unknown" if absent).
	const char *git_short =
#ifdef TEST_BUILD_GIT_SHORT
	    TEST_BUILD_GIT_SHORT;
#else
	    "unknown";
#endif
	const char *git_full =
#ifdef TEST_BUILD_GIT_FULL
	    TEST_BUILD_GIT_FULL;
#else
	    "unknown";
#endif
	const char *git_branch =
#ifdef TEST_BUILD_GIT_BRANCH
	    TEST_BUILD_GIT_BRANCH;
#else
	    "unknown";
#endif
	const char *git_describe =
#ifdef TEST_BUILD_GIT_DESCRIBE
	    TEST_BUILD_GIT_DESCRIBE;
#else
	    "unknown";
#endif
	const char *build_ts =
#ifdef TEST_BUILD_TIMESTAMP
	    TEST_BUILD_TIMESTAMP;
#else
	    "unknown";
#endif
	const char *fw_version =
#ifdef FW_APP_VERSION_STR_C
	    FW_APP_VERSION_STR_C;
#else
	    "unknown";
#endif

	// Tally per-group + totals.
	std::map<std::string, std::pair<unsigned, unsigned>> by_group;  // group → {passed, failed}
	unsigned total = g_results.size(), passed = 0, failed = 0;
	double total_ms = 0;
	for (const auto &r : g_results) {
		auto &p = by_group[r.group];
		if (r.passed) {
			p.first++;
			passed++;
		} else {
			p.second++;
			failed++;
		}
		total_ms += r.duration_ms;
	}
	double pass_rate = total ? 100.0 * passed / total : 0.0;

	time_t now = time(nullptr);
	char run_ts[64];
	strftime(run_ts, sizeof(run_ts), "%Y-%m-%d %H:%M:%S", localtime(&now));

	// Detect "in-progress" by looking at g_current_test (only set during a
	// running test) vs the completed-run state.
	bool in_progress = live && !g_run_completed && !g_current_test.empty();
	double elapsed_s = (clock() - g_run_start) / (double)CLOCKS_PER_SEC;

	f << "<!DOCTYPE html><html lang=\"en\"><head><meta charset=\"UTF-8\">"
	     "<title>TrackerTests "
	  << (in_progress ? "(running)" : "(done)") << " — " << git_short << "</title>";
	// Auto-refresh every 2 s while the run is in progress so the operator
	// can keep the page open and watch progress. No refresh on the final
	// write (run completed).
	if (live) {
		f << "<meta http-equiv=\"refresh\" content=\"2\">";
	}
	f << "<style>"
	     "*{margin:0;padding:0;box-sizing:border-box}"
	     "body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;"
	     "background:linear-gradient(135deg,#1a1a2e,#16213e);min-height:100vh;color:#eee;padding:20px}"
	     ".container{max-width:1300px;margin:0 auto}"
	     "h1{color:#00d4ff;text-align:center;margin-bottom:6px;text-shadow:0 0 18px rgba(0,212,255,.4)}"
	     "h2{color:#ffa502;margin-top:36px;margin-bottom:12px}"
	     ".meta{text-align:center;font-size:13px;opacity:.75;margin-bottom:30px}"
	     ".meta code{background:rgba(255,255,255,.08);padding:2px 8px;border-radius:4px;font-size:12px}"
	     ".stats{display:grid;grid-template-columns:repeat(auto-fit,minmax(160px,1fr));gap:14px;margin:20px 0 30px}"
	     ".stat{background:rgba(255,255,255,.06);padding:18px;border-radius:10px;text-align:center;"
	     "border:1px solid rgba(255,255,255,.08)}"
	     ".stat .v{font-size:36px;font-weight:700}.stat .l{font-size:12px;opacity:.7;text-transform:uppercase}"
	     ".v.ok{color:#00ff88}.v.ko{color:#ff4757}.v.info{color:#00d4ff}.v.warn{color:#ffa502}"
	     ".progress{width:100%;height:14px;background:rgba(255,255,255,.08);border-radius:7px;overflow:hidden;margin-"
	     "bottom:20px}"
	     ".progress div{height:100%;background:linear-gradient(90deg,#00ff88,#00d4ff);transition:width .8s}"
	     "table{width:100%;border-collapse:collapse;background:rgba(255,255,255,.04);border-radius:10px;overflow:"
	     "hidden;font-size:13px}"
	     "th{background:rgba(0,212,255,.18);padding:11px 14px;text-align:left;font-weight:600;position:sticky;top:0}"
	     "td{padding:9px 14px;border-bottom:1px solid rgba(255,255,255,.04)}"
	     "tr:hover{background:rgba(255,255,255,.04)}"
	     ".badge{display:inline-block;padding:3px 10px;border-radius:12px;font-size:11px;font-weight:700}"
	     ".pass{background:rgba(0,255,136,.18);color:#00ff88}.fail{background:rgba(255,71,87,.18);color:#ff4757}"
	     ".dur{color:#888;font-size:11px}"
	     ".fmsg{color:#ff4757;font-size:11px;margin-top:6px;font-family:'SF Mono',Menlo,monospace;"
	     "white-space:pre-wrap;word-break:break-all;max-width:700px}"
	     ".group-row td{background:rgba(255,165,2,.05);font-weight:600;color:#ffa502}"
	     "details{margin-top:8px}summary{cursor:pointer;opacity:.8}summary:hover{opacity:1}"
	     ".running-banner{background:linear-gradient(90deg,rgba(0,212,255,.18),rgba(255,165,2,.18));"
	     "border-left:4px solid #ffa502;padding:14px 18px;border-radius:6px;margin-bottom:22px;"
	     "display:flex;justify-content:space-between;align-items:center;font-size:14px}"
	     ".running-banner .pulse{display:inline-block;width:10px;height:10px;background:#ffa502;"
	     "border-radius:50%;margin-right:8px;animation:pulse 1s ease-in-out infinite}"
	     "@keyframes pulse{0%,100%{opacity:1;transform:scale(1)}50%{opacity:.4;transform:scale(.7)}}"
	     ".done-banner{background:rgba(0,255,136,.10);border-left:4px solid #00ff88;padding:14px 18px;"
	     "border-radius:6px;margin-bottom:22px;font-size:14px}"
	     ".done-banner.fail{background:rgba(255,71,87,.10);border-left-color:#ff4757}"
	     ".footer{text-align:center;margin-top:40px;opacity:.45;font-size:11px}"
	     "</style></head><body><div class=\"container\">"
	     "<h1>&#x1F50D; TrackerTests Regression Report</h1>"
	     "<div class=\"meta\">"
	     "FW <code>"
	  << html_escape(fw_version)
	  << "</code> &middot; "
	     "branch <code>"
	  << html_escape(git_branch)
	  << "</code> &middot; "
	     "commit <code title=\""
	  << html_escape(git_full) << "\">" << html_escape(git_short)
	  << "</code> &middot; "
	     "describe <code>"
	  << html_escape(git_describe)
	  << "</code><br>"
	     "built "
	  << html_escape(build_ts) << " &middot; run " << run_ts << "</div>";

	// Live status banner.
	if (in_progress) {
		f << "<div class=\"running-banner\">"
		  << "<div><span class=\"pulse\"></span><b>RUNNING</b> &middot; "
		  << "currently: <code>" << html_escape(g_current_group) << "::" << html_escape(g_current_test)
		  << "</code></div>"
		  << "<div>" << total << " test" << (total == 1 ? "" : "s") << " done &middot; "
		  << "elapsed " << std::fixed << std::setprecision(1) << elapsed_s << "s</div>"
		  << "</div>";
	} else if (g_run_completed) {
		bool any_fail = failed > 0;
		f << "<div class=\"done-banner" << (any_fail ? " fail" : "") << "\">"
		  << "<b>" << (any_fail ? "&#x274C; FAILED" : "&#x2705; PASSED") << "</b> &middot; " << passed << "/" << total
		  << " tests passed in " << std::fixed << std::setprecision(1) << elapsed_s << "s"
		  << "</div>";
	}

	f << "<div class=\"stats\">"
	     "<div class=\"stat\"><div class=\"v info\">"
	  << total
	  << "</div><div class=\"l\">Total tests</div></div>"
	     "<div class=\"stat\"><div class=\"v ok\">"
	  << passed
	  << "</div><div class=\"l\">Passed</div></div>"
	     "<div class=\"stat\"><div class=\"v ko\">"
	  << failed
	  << "</div><div class=\"l\">Failed</div></div>"
	     "<div class=\"stat\"><div class=\"v warn\">"
	  << std::fixed << std::setprecision(1) << pass_rate
	  << "%</div><div class=\"l\">Pass rate</div></div>"
	     "<div class=\"stat\"><div class=\"v info\">"
	  << by_group.size()
	  << "</div><div class=\"l\">Groups</div></div>"
	     "<div class=\"stat\"><div class=\"v info\">"
	  << std::fixed << std::setprecision(1) << total_ms / 1000.0
	  << "s</div><div class=\"l\">Total time</div></div>"
	     "</div>";

	f << "<div class=\"progress\"><div style=\"width:" << pass_rate << "%\"></div></div>";

	// Per-group summary.
	f << "<h2>Per-group summary</h2><table><thead><tr>"
	     "<th>Group</th><th style=\"text-align:right\">Passed</th><th style=\"text-align:right\">Failed</th>"
	     "<th style=\"text-align:right\">Total</th><th>Status</th></tr></thead><tbody>";
	for (const auto &kv : by_group) {
		unsigned gp = kv.second.first, gf = kv.second.second;
		const char *badge = gf ? "fail" : "pass";
		const char *txt = gf ? "FAIL" : "PASS";
		f << "<tr><td>" << html_escape(kv.first) << "</td>"
		  << "<td style=\"text-align:right\">" << gp << "</td>"
		  << "<td style=\"text-align:right\">" << gf << "</td>"
		  << "<td style=\"text-align:right\">" << (gp + gf) << "</td>"
		  << "<td><span class=\"badge " << badge << "\">" << txt << "</span></td></tr>";
	}
	f << "</tbody></table>";

	// Failures section (always shown if any).
	if (failed > 0) {
		f << "<h2>Failures (" << failed
		  << ")</h2><table><thead><tr>"
		     "<th>Group</th><th>Test</th><th>Message</th><th style=\"text-align:right\">Duration</th>"
		     "</tr></thead><tbody>";
		for (const auto &r : g_results) {
			if (r.passed) continue;
			f << "<tr><td>" << html_escape(r.group) << "</td>"
			  << "<td>" << html_escape(r.name) << "</td>"
			  << "<td><div class=\"fmsg\">" << html_escape(r.failure_message) << "</div></td>"
			  << "<td class=\"dur\" style=\"text-align:right\">" << std::fixed << std::setprecision(1) << r.duration_ms
			  << " ms</td></tr>";
		}
		f << "</tbody></table>";
	}

	// Full per-test list (in <details> to keep page short by default).
	f << "<h2>All tests (" << total
	  << ")</h2>"
	     "<details><summary>Click to expand the full list</summary>"
	     "<table><thead><tr><th>Group</th><th>Test</th><th>Status</th>"
	     "<th style=\"text-align:right\">Duration</th></tr></thead><tbody>";
	for (const auto &r : g_results) {
		const char *badge = r.passed ? "pass" : "fail";
		const char *txt = r.passed ? "PASS" : "FAIL";
		f << "<tr><td>" << html_escape(r.group) << "</td>"
		  << "<td>" << html_escape(r.name);
		if (!r.passed && !r.failure_message.empty()) {
			f << "<div class=\"fmsg\">" << html_escape(r.failure_message) << "</div>";
		}
		f << "</td><td><span class=\"badge " << badge << "\">" << txt << "</span></td>"
		  << "<td class=\"dur\" style=\"text-align:right\">" << std::fixed << std::setprecision(1) << r.duration_ms
		  << " ms</td></tr>";
	}
	f << "</tbody></table></details>";

	f << "<div class=\"footer\">"
	     "Generated by tests/src/main.cpp &middot; linkit-v4-core"
	     "</div></div></body></html>";

	// Only print on the final (non-live) write — live updates would spam.
	if (!live) {
		printf("\n=== TrackerTests HTML Report: %s ===\n", path.c_str());
	}
}

}  // namespace

int main(int argc, char **argv) {
	ConsoleLog con_log;
	DebugLogger::console_log = &con_log;

	// Output path: env var override, else default. Captured into a global
	// so HtmlReportPlugin can rewrite the file after each test (live mode).
	const char *env_path = std::getenv("TRACKERTESTS_REPORT_PATH");
	g_report_path = env_path ? env_path : "trackertests_report.html";
	g_log_path = g_report_path + ".log";  // sidecar TSV for fork-survival

	// Reset the log so a previous run's records don't pollute this report.
	std::ofstream(g_log_path, std::ios::trunc).close();

	// Write initial "starting up" report so the operator can already open
	// the page in a browser before any test has fired.
	g_run_start = clock();
	write_html_report(g_report_path, /*live=*/true);
	printf("[trackertests-report] live report: %s (auto-refresh every 2 s)\n", g_report_path.c_str());

	// Install plugins
	HtmlReportPlugin html_plugin;
	TestRegistry::getCurrentRegistry()->installPlugin(&mockPlugin);
	TestRegistry::getCurrentRegistry()->installPlugin(&html_plugin);

	int exit_code = CommandLineTestRunner::RunAllTests(argc, argv);

	// Final write (no auto-refresh, "done" banner).
	g_run_completed = true;
	g_current_test.clear();
	g_current_group.clear();
	// Reload from log to pick up ALL records, including those from -p
	// forked children whose g_results never made it back here.
	load_log_records();
	write_html_report(g_report_path, /*live=*/false);

	// Console summary
	unsigned passed = 0, failed = 0;
	for (const auto &r : g_results) {
		if (r.passed)
			passed++;
		else
			failed++;
	}
	printf("========================================\n");
	printf("  TrackerTests summary: %u/%u passed, %u failed\n", passed, (unsigned)g_results.size(), failed);
	printf("  Report: %s\n", g_report_path.c_str());
	printf("========================================\n");

	return exit_code;
}
