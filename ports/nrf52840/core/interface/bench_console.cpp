/**
 * @file bench_console.cpp
 * @brief Bench-test serial console implementation (see bench_console.hpp).
 *
 * BENCH_TEST-only. Zero footprint in production builds.
 */

#ifdef BENCH_TEST

#include "bench_console.hpp"
#include "usb_interface.hpp"
#include "gentracker.hpp"
#include "gps_service.hpp"
#include "reed.hpp"
#include "scheduler.hpp"
#include "debug.hpp"

#include <cstdio>
#include <cstdlib>

extern Scheduler  *system_scheduler;
extern GPSService *gps_service;

namespace {

constexpr unsigned int BENCH_POLL_MS = 50;
Scheduler::TaskHandle  s_poll_task;

void reply(const std::string& s) {
    UsbInterface::get_instance().write(s + "\r\n");
}

const char* state_name() {
    if (GenTracker::is_in_state<ConfigurationState>())   return "CONFIG";
    if (GenTracker::is_in_state<OperationalState>())     return "OPERATIONAL";
    if (GenTracker::is_in_state<PreOperationalState>())  return "PREOP";
    if (GenTracker::is_in_state<OffState>())             return "OFF";
    if (GenTracker::is_in_state<BatteryCriticalState>()) return "BATT_CRIT";
    if (GenTracker::is_in_state<ErrorState>())           return "ERROR";
    if (GenTracker::is_in_state<BootState>())            return "BOOT";
    return "UNKNOWN";
}

/// @brief Synthesise the reed confirmation gesture (SHORT_HOLD → RELEASE → ENGAGE)
/// and dispatch it from a fresh scheduler task (NOT inline). The FSM interprets it
/// as ENTER_CONFIG (from Operational) or EXIT_CONFIG (from Config) — byte-for-byte
/// the same path as a fast real magnet gesture (see GenTracker::react(ReedSwitchEvent)).
///
/// Deferring via post_task_prio is deliberate: the transit<> it triggers must NOT
/// run nested inside process_usb_data()/bench_poll() (which would re-arm a poll task
/// for a state we just left). Mirrors how the real reed callback posts tasks.
void synth_confirm_gesture() {
    system_scheduler->post_task_prio([]() {
        ReedSwitchEvent e;
        e.state = ReedSwitchGesture::SHORT_HOLD; GenTracker::dispatch(e);
        e.state = ReedSwitchGesture::RELEASE;    GenTracker::dispatch(e);
        e.state = ReedSwitchGesture::ENGAGE;     GenTracker::dispatch(e);
    }, "BenchGesture");
}

void cmd_gps(const std::string& line) {
    if (!GenTracker::is_in_state<OperationalState>()) {
        reply("%GPS ERR not-operational (use %OP first)");
        return;
    }
    if (!gps_service) {
        reply("%GPS ERR no-gps-service (no M10Q detected)");
        return;
    }
    double lat = 0.0, lon = 0.0;
    unsigned int hacc = 0, numsv = 0;
    int n = sscanf(line.c_str(), "%%GPS %lf %lf %u %u", &lat, &lon, &hacc, &numsv);
    if (n < 2) {
        reply("%GPS ERR usage: %GPS <lat> <lon> [hAcc_mm] [numSV]");
        return;
    }
    gps_service->bench_inject_fix(lat, lon, (uint32_t)hacc, (uint8_t)numsv);
    char buf[96];
    snprintf(buf, sizeof(buf), "%%GPS OK lat=%.6f lon=%.6f", lat, lon);
    reply(buf);
}

}  // namespace

bool bench::handle_line(const std::string& raw) {
    // Trim trailing CR/LF/space.
    std::string line = raw;
    while (!line.empty() &&
           (line.back() == '\r' || line.back() == '\n' || line.back() == ' '))
        line.pop_back();

    if (line.empty() || line[0] != '%')
        return false;

    const std::string cmd = line.substr(0, line.find(' '));

    if (cmd == "%PING") {
        reply(std::string("%BENCH OK state=") + state_name());
    } else if (cmd == "%STATE") {
        reply(std::string("%STATE ") + state_name());
    } else if (cmd == "%CFG") {
        if (GenTracker::is_in_state<ConfigurationState>()) {
            reply("%CFG OK already-config");
        } else {
            reply("%CFG OK entering-config");
            synth_confirm_gesture();
        }
    } else if (cmd == "%OP") {
        if (!GenTracker::is_in_state<ConfigurationState>()) {
            reply("%OP OK not-in-config");
        } else {
            reply("%OP OK leaving-config");
            synth_confirm_gesture();
        }
    } else if (cmd == "%GPS") {
        cmd_gps(line);
    } else {
        reply(std::string("%ERR unknown-cmd ") + cmd);
    }
    return true;
}

namespace {

void bench_poll() {
    // While in ConfigurationState the config USB poller owns the RX and routes
    // '%' lines into handle_line(); reading here too would steal DTE bytes and
    // race on the CDC buffer. Everywhere else, we are the sole USB reader.
    if (!GenTracker::is_in_state<ConfigurationState>()) {
        auto& usb = UsbInterface::get_instance();
        if (usb.has_data()) {
            const std::string line = usb.read_line();
            if (line.size())
                bench::handle_line(line);
        }
    }
    s_poll_task = system_scheduler->post_task_prio(
        bench_poll, "BenchPoll", Scheduler::DEFAULT_PRIORITY, BENCH_POLL_MS);
}

}  // namespace

void bench::start_poll() {
    reply("%BENCH ready");
    DEBUG_WARN("bench::start_poll: BENCH_TEST console active ('%%' commands over USB-CDC)");
    s_poll_task = system_scheduler->post_task_prio(
        bench_poll, "BenchPoll", Scheduler::DEFAULT_PRIORITY, BENCH_POLL_MS);
}

#endif  // BENCH_TEST
