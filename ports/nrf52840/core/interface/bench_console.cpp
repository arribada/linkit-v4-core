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
#include "ble_service.hpp"
#include "service.hpp"
#include "reed.hpp"
#include "scheduler.hpp"
#include "debug.hpp"
#include "is25_flash.hpp"
#include "config_store.hpp"
#include "ota_file_updater.hpp"
#include "crc32.hpp"

extern OTAFileUpdater *ota_updater;

extern Is25Flash *bench_flash;

#include <cstdio>
#include <cstdlib>

extern Scheduler  *system_scheduler;
extern GPSService *gps_service;
extern BLEService *ble_service;

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

void cmd_fastloc(const std::string& line) {
    if (!GenTracker::is_in_state<OperationalState>()) {
        reply("%GPSFL ERR not-operational (use %OP first)"); return;
    }
    if (!gps_service) { reply("%GPSFL ERR no-gps-service"); return; }
    double lat = 0.0, lon = 0.0;
    unsigned int hacc = 0, numsv = 0;
    int n = sscanf(line.c_str(), "%%GPSFL %lf %lf %u %u", &lat, &lon, &hacc, &numsv);
    if (n < 2) { reply("%GPSFL ERR usage: %GPSFL <lat> <lon> [hAcc_mm] [numSV]"); return; }
    gps_service->bench_inject_fastloc(lat, lon, (uint32_t)hacc, (uint8_t)numsv);
    reply("%GPSFL OK fastloc injected");
}

}  // namespace


/// @brief Deterministic filler byte for the %OTA harness.
static inline uint8_t bench_ota_byte(unsigned int i) { return (uint8_t)(i & 0xFF); }

/// Bench-only OTA driver. It exercises the updater directly rather than over
/// BLE, because the interesting behaviour is not the transport: it is whether
/// leaving configuration mode aborts an UNFINISHED transfer (it must, or the
/// file handle leaks and every later transfer is refused) while leaving a
/// FINISHED one alone (it must, or the staged image is destroyed before
/// apply_file_update() runs). Uses OTAFileIdentifier::GPS_CONFIG, which is a
/// plain LittleFS file -- never the firmware region, and never a reset.
static unsigned int s_ota_total = 0;
static unsigned int s_ota_sent  = 0;

static void bench_ota(const std::string& line) {
    if (!ota_updater) { reply("%OTA ERR no-updater"); return; }

    if (line.find(" START") != std::string::npos) {
        unsigned int size = 0;
        if (sscanf(line.c_str(), "%%OTA START %u", &size) != 1 || size == 0 || (size & 3)) {
            reply("%OTA ERR usage: %OTA START <size, multiple of 4>"); return;
        }
        uint32_t crc = 0xFFFFFFFF;
        for (unsigned int i = 0; i < size; i++) {
            uint8_t b = bench_ota_byte(i);
            CRC32::checksum_update(&b, 1, crc);
        }
        CRC32::checksum_finalize(crc);
        try {
            ota_updater->start_file_transfer(OTAFileIdentifier::GPS_CONFIG, size, crc);
        } catch (ErrorCode e) {
            char buf[64]; snprintf(buf, sizeof(buf), "%%OTA ERR start=%u", (unsigned)e);
            reply(buf); return;
        }
        s_ota_total = size; s_ota_sent = 0;
        char buf[80]; snprintf(buf, sizeof(buf), "%%OTA OK start size=%u crc=%08lX",
                               size, (unsigned long)crc);
        reply(buf);
    } else if (line.find(" DATA") != std::string::npos) {
        unsigned int n = 0;
        if (sscanf(line.c_str(), "%%OTA DATA %u", &n) != 1 || n == 0) {
            reply("%OTA ERR usage: %OTA DATA <bytes>"); return;
        }
        if (s_ota_sent + n > s_ota_total) n = s_ota_total - s_ota_sent;
        uint8_t chunk[64];
        try {
            while (n) {
                unsigned int k = n > sizeof(chunk) ? (unsigned int)sizeof(chunk) : n;
                for (unsigned int j = 0; j < k; j++) chunk[j] = bench_ota_byte(s_ota_sent + j);
                ota_updater->write_file_data(chunk, k);
                s_ota_sent += k; n -= k;
            }
        } catch (ErrorCode e) {
            char buf[64]; snprintf(buf, sizeof(buf), "%%OTA ERR data=%u", (unsigned)e);
            reply(buf); return;
        }
        char buf[80]; snprintf(buf, sizeof(buf), "%%OTA OK data sent=%u/%u", s_ota_sent, s_ota_total);
        reply(buf);
    } else if (line.find(" END") != std::string::npos) {
        try {
            ota_updater->complete_file_transfer();
        } catch (ErrorCode e) {
            char buf[64]; snprintf(buf, sizeof(buf), "%%OTA ERR end=%u", (unsigned)e);
            reply(buf); return;
        }
        reply("%OTA OK end");
    } else if (line.find(" ABORT") != std::string::npos) {
        try { ota_updater->abort_file_transfer(); }
        catch (ErrorCode e) {
            char buf[64]; snprintf(buf, sizeof(buf), "%%OTA ERR abort=%u", (unsigned)e);
            reply(buf); return;
        }
        s_ota_total = s_ota_sent = 0;
        reply("%OTA OK abort");
    } else {
        char buf[96];
        snprintf(buf, sizeof(buf), "%%OTA incomplete=%d sent=%u/%u",
                 ota_updater->is_transfer_incomplete() ? 1 : 0, s_ota_sent, s_ota_total);
        reply(buf);
    }
}

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
    } else if (cmd == "%BLE") {
        // %BLE        -> etat de l'advertising
        // %BLE DISC   -> rejoue une deconnexion BLE (sans telephone)
        if (!ble_service) {
            reply("%BLE ERR no-ble-service");
        } else if (line.find(" PROBE") != std::string::npos) {
            char buf[80];
            snprintf(buf, sizeof(buf), "%%BLE PROBE rc=%u (0=advertissait, 8=non)",
                     ble_service->bench_probe_advertising());
            reply(buf);
        } else if (line.find(" DISC") != std::string::npos) {
            ble_service->bench_inject_disconnect();
            char buf[96];
            snprintf(buf, sizeof(buf), "%%BLE OK disconnect-injected wanted=%d mode=%d",
                     ble_service->bench_is_advertising() ? 1 : 0,
                     ble_service->bench_adv_mode());
            reply(buf);
        } else {
            char buf[96];
            snprintf(buf, sizeof(buf), "%%BLE wanted=%d mode=%d state=%s",
                     ble_service->bench_is_advertising() ? 1 : 0,
                     ble_service->bench_adv_mode(), state_name());
            reply(buf);
        }
    } else if (cmd == "%FLASH") {
        // %FLASH        -> statut du composant (WIP/QE) + etat d'init
        // %FLASH SWRST  -> exerce RSTEN+RST sur silicium reel, non destructif
        if (!bench_flash) {
            reply("%FLASH ERR no-flash");
        } else if (line.find(" SWRST") != std::string::npos) {
            uint8_t jedec[3] = { 0, 0, 0 };
            uint8_t status = 0;
            bool ok = bench_flash->bench_software_reset(jedec, status);
            char buf[128];
            snprintf(buf, sizeof(buf),
                     "%%FLASH SWRST %s jedec=%02X%02X%02X status=%02X qe=%d wip=%d",
                     ok ? "OK" : "ERR", jedec[0], jedec[1], jedec[2], status,
                     (status & 0x40) ? 1 : 0, (status & 0x01) ? 1 : 0);
            reply(buf);
        } else {
            uint8_t status = bench_flash->bench_status();
            char buf[128];
            snprintf(buf, sizeof(buf), "%%FLASH init=%d status=%02X qe=%d wip=%d",
                     bench_flash->is_init() ? 1 : 0, status,
                     (status & 0x40) ? 1 : 0, (status & 0x01) ? 1 : 0);
            reply(buf);
        }
    } else if (cmd == "%OTA") {
        bench_ota(line);
    } else if (cmd == "%LB") {
        // Consistency of the two battery thresholds. The matching DEBUG_WARN goes
        // to system.log (console logs are deliberately silent during a DTE
        // exchange), so a test cannot observe it on the console: we expose the
        // verdict here, emitted by the same branch as the warning.
        char buf[96];
        snprintf(buf, sizeof(buf), "%%LB coherent=%d lb=%u crit=%u",
                 configuration_store->check_battery_thresholds() ? 1 : 0,
                 configuration_store->read_param<unsigned int>(ParamID::LB_THRESHOLD),
                 configuration_store->read_param<unsigned int>(ParamID::LB_CRITICAL_THRESH));
        reply(buf);
    } else if (cmd == "%SCHED") {
        // Schedule of EVERY registered service, straight from the decision point
        // in Service::reschedule(). Needed because the only log line that proves
        // scheduling is DEBUG_TRACE, i.e. compiled out at DEBUG_LEVEL=3 — a test
        // watching the console for it can never see it, and a looser pattern just
        // matches unrelated services and passes without verifying anything.
        reply(std::string("%SCHED ") + ServiceManager::bench_schedule_report());
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
    } else if (cmd == "%GPSFL") {
        cmd_fastloc(line);
    } else if (cmd == "%GPSCL") {
        if (!GenTracker::is_in_state<OperationalState>())
            reply("%GPSCL ERR not-operational (use %OP first)");
        else if (!gps_service)
            reply("%GPSCL ERR no-gps-service");
        else { gps_service->bench_inject_cloudlocate(); reply("%GPSCL OK cloudlocate injected"); }
    } else if (cmd == "%NOFIX") {
        if (!GenTracker::is_in_state<OperationalState>())
            reply("%NOFIX ERR not-operational (use %OP first)");
        else if (!gps_service)
            reply("%NOFIX ERR no-gps-service");
        else { gps_service->bench_inject_nofix(); reply("%NOFIX OK"); }
    } else if (cmd == "%DIVE" || cmd == "%SURFACE") {
        // Simulate the saltwater switch: broadcast the same UW_SENSOR event the SWS
        // service emits (source=UW_SENSOR, type=SERVICE_LOG_UPDATED, data=bool). Each
        // Service::notify_peer_event routes it to notify_underwater_state(). %DIVE =
        // wet (underwater=true), %SURFACE = dry (surfaced=false) — required to drive
        // SURFACING_BURST's progressive Doppler cascade and underwater gating.
        ServiceEvent e;
        e.event_source = ServiceIdentifier::UW_SENSOR;
        e.event_type   = ServiceEventType::SERVICE_LOG_UPDATED;
        e.event_data   = (cmd == "%DIVE");   // true = underwater, false = surfaced
        ServiceManager::notify_peer_event(e);
        reply(cmd == "%DIVE" ? "%DIVE OK underwater" : "%SURFACE OK surfaced");
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

namespace {

Scheduler::TaskHandle s_autoinject_task;

// RSPB auto-inject: once per boot, when Operational + gps_service is up, inject a
// synthetic fix so the satellite TX pipeline runs with no antenna. Statics reset
// on the soft reset that ends each simulated TPL cycle, so every boot injects once.
void auto_inject_fire() {
    static bool s_done = false;
    static unsigned int s_tries = 0;
    if (s_done)
        return;
    if (GenTracker::is_in_state<OperationalState>() && gps_service) {
        s_done = true;
        DEBUG_WARN("bench::auto_inject: injecting synthetic fix (RSPB duty-cycle)");
        gps_service->bench_inject_fix(-21.0097, 55.2707, 2500, 8);
        return;
    }
    if (++s_tries > 60) {   // ~60 s: give up (no GNSS service on this build)
        DEBUG_WARN("bench::auto_inject: gps_service not ready after 60 s — TX will rely on heartbeat");
        return;
    }
    s_autoinject_task = system_scheduler->post_task_prio(
        auto_inject_fire, "BenchAutoInject", Scheduler::DEFAULT_PRIORITY, 1000);
}

}  // namespace

void bench::start_auto_inject() {
    DEBUG_WARN("bench::start_auto_inject: RSPB compressed duty-cycle — auto-inject fix once Operational");
    s_autoinject_task = system_scheduler->post_task_prio(
        auto_inject_fire, "BenchAutoInject", Scheduler::DEFAULT_PRIORITY, 5000);
}

#endif  // BENCH_TEST
