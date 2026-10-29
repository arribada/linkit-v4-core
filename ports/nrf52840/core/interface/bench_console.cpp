/**
 * @file bench_console.cpp
 * @brief Bench-test serial console implementation (see bench_console.hpp).
 *
 * BENCH_TEST-only. Zero footprint in production builds.
 */

#ifdef BENCH_TEST

#include "uwdetector_service.hpp"
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
#include "filesystem.hpp"
#include "ledsm.hpp"
#include "rgb_led.hpp"

extern RGBLed *status_led;
using led_handle = LEDState;  // meme alias que gentracker.cpp / ledsm.cpp
#include "argos_tx_service.hpp"
extern ArgosTxService *argos_tx_service_instance;
#if defined(LORA_LINKCHECK) && (LORA_LINKCHECK == 1)
#include "lora_tx_service.hpp"
extern LoRaTxService *lora_tx_service_instance;
#endif
#if defined(LORA_MOTION_EXT) && (LORA_MOTION_EXT == 1)
#include "bma400.hpp"
#endif
#include "moored_mode_service.hpp"
#include "hauled_mode_service.hpp"
#include "rtc.hpp"
#include "m10qasync.hpp"
#include "ota_file_updater.hpp"
#include "crc32.hpp"
#include "nrf_i2c.hpp"
#include "pmu.hpp"
#include "bsp.hpp"
#include "gpio.hpp"
#include "nrf_gpio.h"
#include "memmang.hpp"

extern OTAFileUpdater *ota_updater;

extern Is25Flash *bench_flash;
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
#include "smd_sat.hpp"
extern SmdSat *smd_sat_instance;
#endif
#if ENABLE_CAM_SENSOR && defined(CAM_CONTROL_UART) && (CAM_CONTROL_UART == 1)
#include "binascii.hpp"
#include "runcam_uart_device.hpp"
extern RunCamUartDevice *runcam_uart_device;
#endif

#include <cstdio>
#include <cstdlib>

extern Scheduler *system_scheduler;
extern Timer *system_timer;
extern GPSService *gps_service;
extern RTC *rtc;
extern FileSystem *main_filesystem;
#ifdef BENCH_TEST
void bench_bootfail_read(unsigned int &, unsigned int &);
extern uint32_t g_bench_vsys_restore_settled;  // nrf_pmu.cpp
extern uint32_t g_bench_vsys_restore_skipped;
#endif
extern BLEService *ble_service;

namespace {

constexpr unsigned int BENCH_POLL_MS = 50;
Scheduler::TaskHandle s_poll_task;

void reply(const std::string &s) {
	UsbInterface::get_instance().write(s + "\r\n");
}

#ifndef BOARD_RSPB
/// @brief Stream the satellite-slot UART to the console — the SMD module's own
/// debug log.
///
/// On a SPI build nothing opens UART instance 1, so its peripheral is free and
/// its pins are parked (nrf_gpio.cpp releases both to default) -- except the
/// camera of a CAM_CONTROL=UART build, which is why the probe refuses to run
/// while the camera rail is on. With the camera off it listens on the camera's
/// RX line, P0.26 there: anything heard is the SMD still wired to it. The slot is the
/// one the KIM2 uses for AT, and on the SMD variant the nRF RECEIVES on P0.14
/// (bsp.cpp) — which is where the module's LPUART1 TX (PA2) lands.
///
/// Driven straight at the UARTE1 registers rather than through libuarte or
/// nrfx: NRFX_UARTE1_ENABLED is 0 and nothing else claims the peripheral on this
/// build, so there is no driver to fight, and this way the probe cannot disturb
/// the SPI path it exists to diagnose.
///
/// The module logs at 115200 in SPI mode (usart.c: 9600 is the STDLN/GUI-UART
/// case only). Bench only.
static void cmd_satlog(const std::string &line) {
	unsigned int secs = 10;
	unsigned int baud = 115200;
	unsigned int a = 0, b = 0, c = 0;
	int argc = sscanf(line.c_str(), "%%SATLOG %u %u %u", &a, &b, &c);
	if (argc >= 1 && a > 0 && a <= 120) secs = a;
	if (argc >= 2 && b) baud = b;

	// Register values for the rates the module can be built at (usart.c: 115200
	// in SPI mode, 9600 for STDLN/GUI-UART). The others are here so a sweep can
	// settle the question without a reflash between tries.
	uint32_t baud_reg;
	switch (baud) {
	case 9600: baud_reg = 0x00275000; break;
	case 19200: baud_reg = 0x004EA000; break;
	case 38400: baud_reg = 0x009D5000; break;
	case 57600: baud_reg = 0x00EBF000; break;
	case 460800: baud_reg = 0x03AFB000; break;  // the M10Q's rate, for checking the probe itself
	default:
		baud = 115200;
		baud_reg = 0x01D7E000;
		break;
	}

	NRF_UARTE_Type *u = NRF_UARTE1;
	// Default to the BSP's RX for this build, but let the caller name a pin: the
	// satellite slot is P0.14 / P0.26 and the BSP assigns the direction per
	// variant (LoRa receives on 26, KIM2/SMD on 14). Which way a given module
	// drives it is a board fact, not a build fact, so the probe should be able to
	// listen to either without a reflash.
	uint32_t rx_pin = BSP::UARTAsync_Inits[1].config.rx_pin;
	// PSEL encoding is port * 32 + pin, so 0..31 is P0 and 32..63 is P1. Being
	// able to name a P1 pin is what makes the probe checkable: pointed at the
	// GNSS UART RX (P1.08 = 40) while the M10Q is running, it must return
	// bytes. A probe that has only ever returned zero proves nothing about the
	// line it was pointed at.
	if (argc >= 3 && c <= 63) rx_pin = c;

	nrf_gpio_cfg_input(rx_pin, NRF_GPIO_PIN_NOPULL);
	u->PSEL.RXD = rx_pin;
	u->PSEL.TXD = 0xFFFFFFFF;
	u->PSEL.CTS = 0xFFFFFFFF;
	u->PSEL.RTS = 0xFFFFFFFF;
	u->BAUDRATE = baud_reg;
	u->CONFIG = 0;  // 8N1, no flow control
	u->ENABLE = 8;

	reply("%SATLOG start (P" + std::to_string(rx_pin / 32) + "." + std::to_string(rx_pin % 32) + " @"
	      + std::to_string(baud) + ", " + std::to_string(secs) + "s)");

	static uint8_t buf[64];
	const uint64_t deadline = PMU::get_timestamp_ms() + (uint64_t)secs * 1000u;
	unsigned int total = 0;

	while (PMU::get_timestamp_ms() < deadline) {
		PMU::kick_watchdog();
		u->EVENTS_ENDRX = 0;
		u->RXD.PTR = (uint32_t)buf;
		u->RXD.MAXCNT = sizeof(buf);
		u->TASKS_STARTRX = 1;

		// Give the DMA a window, then close it so RXD.AMOUNT is valid even on a
		// partial buffer — the module talks in short bursts, waiting for a full
		// 64 bytes would hold most lines back until the next one arrived.
		uint64_t slice = PMU::get_timestamp_ms() + 250;
		while (!u->EVENTS_ENDRX && PMU::get_timestamp_ms() < slice) { /* spin */
		}
		if (!u->EVENTS_ENDRX) {
			u->EVENTS_ENDRX = 0;
			u->TASKS_STOPRX = 1;
			uint64_t stop_deadline = PMU::get_timestamp_ms() + 20;
			while (!u->EVENTS_ENDRX && PMU::get_timestamp_ms() < stop_deadline) { /* spin */
			}
		}

		unsigned int n = u->RXD.AMOUNT;
		if (n) {
			total += n;
			UsbInterface::get_instance().write(std::string((const char *)buf, n));
		}
	}

	u->TASKS_STOPRX = 1;
	u->ENABLE = 0;
	u->PSEL.RXD = 0xFFFFFFFF;
	nrf_gpio_cfg_default(rx_pin);

	reply("");
	reply("%SATLOG done bytes=" + std::to_string(total));
}
#endif  // !BOARD_RSPB: LinkIt's UARTE1 slot; RSPB has one async UART, and this probe kept its bench build from compiling


const char *state_name() {
	if (GenTracker::is_in_state<ConfigurationState>()) return "CONFIG";
	if (GenTracker::is_in_state<OperationalState>()) return "OPERATIONAL";
	if (GenTracker::is_in_state<PreOperationalState>()) return "PREOP";
	if (GenTracker::is_in_state<OffState>()) return "OFF";
	if (GenTracker::is_in_state<BatteryCriticalState>()) return "BATT_CRIT";
	if (GenTracker::is_in_state<ErrorState>()) return "ERROR";
	if (GenTracker::is_in_state<BootState>()) return "BOOT";
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
	system_scheduler->post_task_prio(
	    []() {
		    ReedSwitchEvent e;
		    e.state = ReedSwitchGesture::SHORT_HOLD;
		    GenTracker::dispatch(e);
		    e.state = ReedSwitchGesture::RELEASE;
		    GenTracker::dispatch(e);
		    e.state = ReedSwitchGesture::ENGAGE;
		    GenTracker::dispatch(e);
	    },
	    "BenchGesture");
}

void cmd_gps(const std::string &line) {
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

void cmd_fastloc(const std::string &line) {
	if (!GenTracker::is_in_state<OperationalState>()) {
		reply("%GPSFL ERR not-operational (use %OP first)");
		return;
	}
	if (!gps_service) {
		reply("%GPSFL ERR no-gps-service");
		return;
	}
	double lat = 0.0, lon = 0.0;
	unsigned int hacc = 0, numsv = 0;
	int n = sscanf(line.c_str(), "%%GPSFL %lf %lf %u %u", &lat, &lon, &hacc, &numsv);
	if (n < 2) {
		reply("%GPSFL ERR usage: %GPSFL <lat> <lon> [hAcc_mm] [numSV]");
		return;
	}
	gps_service->bench_inject_fastloc(lat, lon, (uint32_t)hacc, (uint8_t)numsv);
	reply("%GPSFL OK fastloc injected");
}

}  // namespace


/// @brief Deterministic filler byte for the %OTA harness.
static inline uint8_t bench_ota_byte(unsigned int i) {
	return (uint8_t)(i & 0xFF);
}

/// Bench-only OTA driver. It exercises the updater directly rather than over
/// BLE, because the interesting behaviour is not the transport: it is whether
/// leaving configuration mode aborts an UNFINISHED transfer (it must, or the
/// file handle leaks and every later transfer is refused) while leaving a
/// FINISHED one alone (it must, or the staged image is destroyed before
/// apply_file_update() runs). Uses OTAFileIdentifier::GPS_CONFIG, which is a
/// plain LittleFS file -- never the firmware region, and never a reset.
static unsigned int s_ota_total = 0;
static unsigned int s_ota_sent = 0;

static void bench_ota(const std::string &line) {
	if (!ota_updater) {
		reply("%OTA ERR no-updater");
		return;
	}

	if (line.find(" START") != std::string::npos) {
		unsigned int size = 0;
		if (sscanf(line.c_str(), "%%OTA START %u", &size) != 1 || size == 0 || (size & 3)) {
			reply("%OTA ERR usage: %OTA START <size, multiple of 4>");
			return;
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
			char buf[64];
			snprintf(buf, sizeof(buf), "%%OTA ERR start=%u", (unsigned)e);
			reply(buf);
			return;
		}
		s_ota_total = size;
		s_ota_sent = 0;
		char buf[80];
		snprintf(buf, sizeof(buf), "%%OTA OK start size=%u crc=%08lX", size, (unsigned long)crc);
		reply(buf);
	} else if (line.find(" DATA") != std::string::npos) {
		unsigned int n = 0;
		if (sscanf(line.c_str(), "%%OTA DATA %u", &n) != 1 || n == 0) {
			reply("%OTA ERR usage: %OTA DATA <bytes>");
			return;
		}
		if (s_ota_sent + n > s_ota_total) n = s_ota_total - s_ota_sent;
		uint8_t chunk[64];
		try {
			while (n) {
				unsigned int k = n > sizeof(chunk) ? (unsigned int)sizeof(chunk) : n;
				for (unsigned int j = 0; j < k; j++)
					chunk[j] = bench_ota_byte(s_ota_sent + j);
				ota_updater->write_file_data(chunk, k);
				s_ota_sent += k;
				n -= k;
			}
		} catch (ErrorCode e) {
			char buf[64];
			snprintf(buf, sizeof(buf), "%%OTA ERR data=%u", (unsigned)e);
			reply(buf);
			return;
		}
		char buf[80];
		snprintf(buf, sizeof(buf), "%%OTA OK data sent=%u/%u", s_ota_sent, s_ota_total);
		reply(buf);
	} else if (line.find(" END") != std::string::npos) {
		try {
			ota_updater->complete_file_transfer();
		} catch (ErrorCode e) {
			char buf[64];
			snprintf(buf, sizeof(buf), "%%OTA ERR end=%u", (unsigned)e);
			reply(buf);
			return;
		}
		reply("%OTA OK end");
	} else if (line.find(" ABORT") != std::string::npos) {
		try {
			ota_updater->abort_file_transfer();
		} catch (ErrorCode e) {
			char buf[64];
			snprintf(buf, sizeof(buf), "%%OTA ERR abort=%u", (unsigned)e);
			reply(buf);
			return;
		}
		s_ota_total = s_ota_sent = 0;
		reply("%OTA OK abort");
	} else {
		char buf[96];
		snprintf(buf, sizeof(buf), "%%OTA incomplete=%d sent=%u/%u", ota_updater->is_transfer_incomplete() ? 1 : 0,
		         s_ota_sent, s_ota_total);
		reply(buf);
	}
}


/// @brief Scan an I2C bus for responding addresses.
///
/// Answers a question no amount of code reading can settle: what is actually
/// FITTED on this board. The BSP declares devices that may or may not be
/// populated, and it also declares ADS1115_DEVICE / ADS1115_ADDRESS (0x48) for
/// which no driver exists anywhere in the tree -- so a positive response there
/// means the part is on the bus and simply unused by the firmware.
///
/// Uses read_safe(), which does not throw, and a 1-byte read: a device that
/// ACKs its address answers, everything else is skipped. Reserved ranges
/// (0x00-0x07, 0x78-0x7F) are not probed.
static void bench_i2c_scan(const std::string &line) {
	// RSPB has a single bus and no EXT_I2C_BUS: since 8f72340f this line stopped
	// every RSPB bench build from compiling.
#ifdef EXT_I2C_BUS
	const bool ext = line.find(" EXT") != std::string::npos;
	unsigned int bus = ext ? (unsigned int)EXT_I2C_BUS : (unsigned int)ONBOARD_I2C_BUS;
#else
	const bool ext = false;
	unsigned int bus = (unsigned int)ONBOARD_I2C_BUS;
#endif
	std::string found;
	char tmp[8];
	unsigned int n = 0;

	// read_safe() retries three times and logs an ERROR line per failure, so a
	// full sweep would bury the answer under 112 error lines and take about
	// 30 s. Mute the console for the duration and feed the watchdog: the scan
	// runs in a scheduler task and nothing else gets a turn while it does.
	auto *saved_log = DebugLogger::console_log;
	DebugLogger::console_log = nullptr;
	for (unsigned int addr = 0x08; addr <= 0x77; addr++) {
		uint8_t b = 0;
		if (NrfI2C::read_safe((uint8_t)bus, (uint8_t)addr, &b, 1)) {
			snprintf(tmp, sizeof(tmp), "%02X ", addr);
			found += tmp;
			n++;
		}
		PMU::kick_watchdog();
	}
	DebugLogger::console_log = saved_log;
	char buf[160];
	snprintf(buf, sizeof(buf), "%%I2C bus=%s found=%u addrs=%s", ext ? "EXT" : "ONBOARD", n, n ? found.c_str() : "-");
	reply(buf);
}


/// @brief Bench probe: which LED state is active, and what the LED is showing.
///
/// Exists because ledsm.cpp defines NO ::exit() at all, so its eight deferred
/// transit<>() calls are never cancelled: one armed by a finishing GNSS session
/// fires 500 ms later whatever state the FSM has reached by then, and on the
/// nominal deployment sequence the Argos TX starts inside that window. Nothing
/// else can observe which LED state is current, so the orphan is invisible from
/// outside.
static const char *bench_etat_led() {
	if (LEDState::is_in_state<LEDOff>()) return "Off";
	if (LEDState::is_in_state<LEDBoot>()) return "Boot";
	if (LEDState::is_in_state<LEDError>()) return "Error";
	if (LEDState::is_in_state<LEDConfigNotConnected>()) return "ConfigNotConnected";
	if (LEDState::is_in_state<LEDConfigConnected>()) return "ConfigConnected";
	if (LEDState::is_in_state<LEDGNSSOn>()) return "GNSSOn";
	if (LEDState::is_in_state<LEDGNSSOffWithFix>()) return "GNSSOffWithFix";
	if (LEDState::is_in_state<LEDGNSSOffWithoutFix>()) return "GNSSOffWithoutFix";
	if (LEDState::is_in_state<LEDGNSSDeepIdle>()) return "GNSSDeepIdle";
	if (LEDState::is_in_state<LEDGNSSPowerOff>()) return "GNSSPowerOff";
	if (LEDState::is_in_state<LEDGNSSCloudLocateReady>()) return "GNSSCloudLocateReady";
	if (LEDState::is_in_state<LEDArgosTX>()) return "ArgosTX";
	if (LEDState::is_in_state<LEDArgosTXComplete>()) return "ArgosTXComplete";
	if (LEDState::is_in_state<LEDBatteryCritical>()) return "BatteryCritical";
	if (LEDState::is_in_state<LEDSurfaceDetected>()) return "SurfaceDetected";
	if (LEDState::is_in_state<LEDDiveDetected>()) return "DiveDetected";
	return "autre";
}

static void bench_led(const std::string &line) {
	// %LED EVT <name> injects a LED event without going through the hardware.
	if (line.find(" EVT") != std::string::npos) {
		auto a = [&](const char *n) { return line.find(n) != std::string::npos; };
		if (a("GNSSON"))
			led_handle::dispatch<SetLEDGNSSOn>({});
		else if (a("GNSSFIX"))
			led_handle::dispatch<SetLEDGNSSOffWithFix>({});
		else if (a("GNSSNOFIX"))
			led_handle::dispatch<SetLEDGNSSOffWithoutFix>({});
		else if (a("GNSSIDLE"))
			led_handle::dispatch<SetLEDGNSSDeepIdle>({});
		else if (a("GNSSPOWEROFF"))
			led_handle::dispatch<SetLEDGNSSPowerOff>({});
		else if (a("CLREADY"))
			led_handle::dispatch<SetLEDGNSSCloudLocateReady>({});
		else if (a("ARGOSTX"))
			led_handle::dispatch<SetLEDArgosTX>({});
		else if (a("OFF"))
			led_handle::dispatch<SetLEDOff>({});
		else {
			reply("%LED ERR evt inconnu (GNSSON|GNSSFIX|GNSSNOFIX|GNSSIDLE|"
			      "GNSSPOWEROFF|CLREADY|ARGOSTX|OFF)");
			return;
		}
	}
	char buf[96];
	snprintf(buf, sizeof(buf), "%%LED etat=%s couleur=%u clignote=%u", bench_etat_led(),
	         status_led ? (unsigned)status_led->get_state() : 99u,
	         status_led ? (unsigned)(status_led->is_flashing() ? 1 : 0) : 0u);
	reply(buf);
}

/// @name handle_line, split by command family
/// One chain of 35 `cmd ==` comparisons used to live in handle_line: 484 lines
/// and a cyclomatic complexity of 118, the second worst in the repository. The
/// branches are unchanged and still in their original order -- each function
/// below holds a contiguous run of them, moved verbatim. The split is by family
/// so that a reader looking for one command reads ~70 lines instead of 484.
///
/// Each returns true if it handled `cmd`, false to let the next family try, and
/// handle_line keeps the single "unknown-cmd" answer so there is still exactly
/// one place that decides a command does not exist. They take `line` as well as
/// `cmd` because several branches parse their arguments out of it.
/// @{

/// @brief liveness, FSM state, boot forensics, BLE and IS25 flash
static bool handle_state_cmd(const std::string &cmd, const std::string &line) {
	if (cmd == "%PING") {
		reply(std::string("%BENCH OK state=") + state_name());
	} else if (cmd == "%STATE") {
		reply(std::string("%STATE ") + state_name());
	} else if (cmd == "%BOOT") {
		// Motif du dernier reset + temps de fonctionnement, pour distinguer une
		// perte de lien USB (l'hote reconnecte, uptime continue de croitre) d'un
		// vrai redemarrage (uptime repart de zero, et le motif dit pourquoi).
		// Les compteurs %PMREQ etant des statiques, seul un redemarrage les
		// remet a zero : %BOOT donne la cause quand cela arrive.
		char buf[96];
		snprintf(buf, sizeof(buf), "%%BOOT cause=%s crash=%s uptime_ms=%llu", PMU::reset_cause_str(),
		         PMU::last_crash_str(), system_timer ? (unsigned long long)system_timer->get_counter() : 0ULL);
		reply(buf);
	} else if (cmd == "%BLE") {
		// %BLE        -> advertising state
		// %BLE DISC   -> replays a BLE disconnection (with no phone)
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
			         ble_service->bench_is_advertising() ? 1 : 0, ble_service->bench_adv_mode());
			reply(buf);
		} else {
			char buf[96];
			snprintf(buf, sizeof(buf), "%%BLE wanted=%d mode=%d state=%s", ble_service->bench_is_advertising() ? 1 : 0,
			         ble_service->bench_adv_mode(), state_name());
			reply(buf);
		}
	} else if (cmd == "%FLASH") {
		// %FLASH        -> part status (WIP/QE) plus the init state
		// %FLASH SWRST  -> exercises RSTEN+RST on real silicon, non destructive
		if (!bench_flash) {
			reply("%FLASH ERR no-flash");
		} else if (line.find(" SWRST") != std::string::npos) {
			uint8_t jedec[3] = { 0, 0, 0 };
			uint8_t status = 0;
			bool ok = bench_flash->bench_software_reset(jedec, status);
			char buf[128];
			snprintf(buf, sizeof(buf), "%%FLASH SWRST %s jedec=%02X%02X%02X status=%02X qe=%d wip=%d",
			         ok ? "OK" : "ERR", jedec[0], jedec[1], jedec[2], status, (status & 0x40) ? 1 : 0,
			         (status & 0x01) ? 1 : 0);
			reply(buf);
		} else {
			uint8_t status = bench_flash->bench_status();
			char buf[128];
			snprintf(buf, sizeof(buf), "%%FLASH init=%d status=%02X qe=%d wip=%d", bench_flash->is_init() ? 1 : 0,
			         status, (status & 0x40) ? 1 : 0, (status & 0x01) ? 1 : 0);
			reply(buf);
		}
	} else {
		return false;
	}
	return true;
}

/// @brief Argos/TX configuration, depth pile, LED and scheduler queues
static bool handle_argos_cmd(const std::string &cmd, const std::string &line) {
	if (cmd == "%TXPAR") {
		// %TXPAR <N> [retx_nb] [period_s] — the BLIND parallel-message question,
		// settled on real silicon instead of guessed. Loads the KMAC BLIND context
		// with nb_parallel=N (production pins it to 1) and fires N payloads back to
		// back without waiting, reporting what the MAC answered to each. If the
		// second one is refused, parallel BLIND is unreachable with this command
		// set and the whole feature can be closed with evidence.
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
		if (!smd_sat_instance) {
			reply("%TXPAR ERR no-smd");
		} else {
			unsigned int n = 2, rn = 2, per = 60;
			const char *args = line.c_str() + cmd.size();
			sscanf(args, "%u %u %u", &n, &rn, &per);
			char buf[160];
			buf[0] = 0;
			smd_sat_instance->bench_parallel_probe(n, rn, per, buf, sizeof(buf));
			reply(buf[0] ? buf : "%TXPAR ERR no-report");
		}
#else
		reply("%TXPAR ERR not-an-smd-build");
#endif
	} else if (cmd == "%PILE") {
		// Contents of the depth pile: type and burst counter per entry. This
		// is the only way to watch the rotation at all -- retrieve() decrements
		// before the burst decides what to encode, and an entry at 0 is never
		// eligible again.
#if defined(LORA_LINKCHECK) && (LORA_LINKCHECK == 1)
		if (lora_tx_service_instance) {
			std::string d = lora_tx_service_instance->bench_dump_pile();
			reply(std::string("%PILE ") + (d.empty() ? "empty" : d));
			return true;
		}
#endif
		if (!argos_tx_service_instance) {
			reply("%PILE ERR no-service");
		} else {
			std::string d = argos_tx_service_instance->bench_dump_pile();
			reply(std::string("%PILE ") + (d.empty() ? "empty" : d));
		}
#if defined(LORA_MOTION_EXT) && (LORA_MOTION_EXT == 1)
	} else if (cmd == "%AXLFAIL") {
		// Make the next BMA400 read throw like an I2C failure, after the chip was
		// switched to NORMAL: the path that used to leave the wake interrupt latched
		// until reboot. The next wake-up (motion, or a low AXP03) does the read.
		BMA400LL::bench_fail_next_read();
		reply("%AXLFAIL OK next read fails");
#endif
	} else if (cmd == "%LED") {
		bench_led(line);
	} else if (cmd == "%ARGOSCFG") {
		// EFFECTIVE Argos configuration (after the LB/OoZ/HAULED cascade). It
		// is what proves sensor_tx_enable really picks up the AXL bit: that
		// mask is what selects process_sensor_burst() over process_gnss_burst(),
		// and there is no other way to observe it.
		ArgosConfig ac;
		configuration_store->get_argos_configuration(ac);
		char buf[176];
		snprintf(buf, sizeof(buf),
		         "%%ARGOSCFG mode=%u sensor_tx=0x%08X depth=%u ntry=%u tr_nom=%u duty=0x%06X lb=%d prepass=%d",
		         (unsigned)ac.mode, ac.sensor_tx_enable, (unsigned)ac.depth_pile, ac.ntry_per_message, ac.tx_interval_s,
		         ac.duty_cycle & 0xFFFFFF, ac.is_lb ? 1 : 0, ac.prepass_en ? 1 : 0);
		reply(buf);
	} else if (cmd.rfind("%LPM", 0) == 0) {
		// Set/read the SMD low-power mode WITHOUT a configuration round trip.
		// Going through DTE means %CFG then %OP, i.e. two reed-confirmation
		// gesture sequences per mode; sweeping the five modes that way put the
		// board into a state where the console went silent and only a debugger
		// reset brought it back. This writes the same parameter and pushes it
		// straight into the live driver.
		//   %LPM        -> report
		//   %LPM <host bitmap>  1=NONE 2=SLEEP 4=STOP 8=STANDBY 16=SHUTDOWN
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
		unsigned int v = 0;
		char buf[128];
		if (sscanf(line.c_str(), "%%LPM %u", &v) == 1) {
			if (v != 1 && v != 2 && v != 4 && v != 8 && v != 16) {
				reply("%LPM ERR usage: %LPM <1=NONE|2=SLEEP|4=STOP|8=STANDBY|16=SHUTDOWN>");
				return true;
			}
			configuration_store->write_param(ParamID::SMD_LPM_MODE, v);
			if (smd_sat_instance) smd_sat_instance->set_lpm_mode(static_cast<uint8_t>(v));
		}
		unsigned int cur = configuration_store->read_param<unsigned int>(ParamID::SMD_LPM_MODE);
		unsigned int mod = cur >> 1;
		snprintf(buf, sizeof(buf), "%%LPM host=0x%02X module=0x%02X mask=0x%02X degraded=%u", cur, mod,
		         mod ? ((mod << 1) - 1) : 0, configuration_store->read_param<unsigned int>(ParamID::SMD_DEGRADED_MODE));
		reply(buf);
#else
		reply("%LPM ERR not-an-smd-build");
#endif
	} else if (cmd.rfind("%BLIND", 0) == 0) {
		// Toggle BLIND without a configuration round trip -- same reason as %LPM.
		// Needed to exercise the two halves of the message-counter policy: BLIND
		// owns its repetitions and the host must not touch the MC, while without
		// BLIND the host sends each repeat itself and pins them to one MC.
		//   %BLIND        -> report
		//   %BLIND 0|1
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
		unsigned int v = 0;
		char buf[128];
		if (sscanf(line.c_str(), "%%BLIND %u", &v) == 1) {
			if (v > 1) {
				reply("%BLIND ERR usage: %BLIND <0|1>");
				return true;
			}
			configuration_store->write_param(ParamID::ARGOS_BLIND_EN, v != 0);
		}
		ArgosConfig ac;
		configuration_store->get_argos_configuration(ac);
		snprintf(buf, sizeof(buf), "%%BLIND en=%u retx_nb=%u period=%us ntry=%u", ac.blind_en ? 1u : 0u,
		         ac.blind_retx_nb, ac.blind_retx_period_s, ac.ntry_per_message);
		reply(buf);
#else
		reply("%BLIND ERR not-an-smd-build");
#endif
	} else if (cmd.rfind("%SATKEEP", 0) == 0) {
		// Hold the SMD powered (parked in its own LPM) for N seconds after a TX
		// instead of cutting its rail. This is what a "wake and transmit"
		// measurement needs: with the rail up and the module in STANDBY at
		// ~0.93 uA, a surface event only has to raise WKUP and send AT+TX.
		//   %SATKEEP <seconds>   (0 restores the 1 s default)
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
		unsigned int sec = 0;
		char buf[96];
		if (sscanf(line.c_str(), "%%SATKEEP %u", &sec) != 1) {
			reply("%SATKEEP ERR usage: %SATKEEP <seconds>");
			return true;
		}
		if (!smd_sat_instance) {
			reply("%SATKEEP ERR no-smd");
			return true;
		}
		smd_sat_instance->set_idle_timeout(sec ? sec * 1000 : 1000);
		snprintf(buf, sizeof(buf), "%%SATKEEP OK idle=%u ms", sec ? sec * 1000 : 1000);
		reply(buf);
#else
		reply("%SATKEEP ERR not-an-smd-build");
#endif
	} else if (cmd.rfind("%DRY", 0) == 0) {
		// DRY_TIME_BEFORE_TX without a configuration round trip. It is not just an
		// antenna-drying delay: ArgosTxScheduler::schedule_periodic takes
		// m_earliest_schedule as the TX time when it is in the FUTURE, and falls
		// back to the periodic grid when it is not. set_earliest_schedule() is fed
		// now + dry_time in whole seconds, so dry=0 always lands in the past and
		// the first TX after surfacing waits for the next grid slot -- up to a
		// full TR_NOM. Any non-zero value pins it instead.
		//   %DRY <seconds>
		unsigned int v = 0;
		char buf[96];
		if (sscanf(line.c_str(), "%%DRY %u", &v) == 1) configuration_store->write_param(ParamID::DRY_TIME_BEFORE_TX, v);
		snprintf(buf, sizeof(buf), "%%DRY dry_time_before_tx=%u s",
		         configuration_store->read_param<unsigned int>(ParamID::DRY_TIME_BEFORE_TX));
		reply(buf);
	} else if (cmd.rfind("%AMODE", 0) == 0) {
		// ARGOS_MODE without a configuration round trip.
		//   %AMODE <0=OFF 1=PASS_PREDICTION 2=LEGACY 3=DUTY_CYCLE 4=DOPPLER 5=SURFACING_BURST>
		unsigned int v = 0;
		char buf[96];
		if (sscanf(line.c_str(), "%%AMODE %u", &v) == 1) {
			if (v > 5) {
				reply("%AMODE ERR usage: %AMODE <0..5>");
				return true;
			}
			configuration_store->write_param(ParamID::ARGOS_MODE, static_cast<BaseArgosMode>(v));
		}
		ArgosConfig ac;
		configuration_store->get_argos_configuration(ac);
		snprintf(buf, sizeof(buf), "%%AMODE mode=%u burst_max=%u s", (unsigned)ac.mode, ac.surfacing_burst_max_s);
		reply(buf);
	} else if (cmd == "%SCHEDQ") {
		// Scheduler queue occupancy. It is what proves a self-rescheduling task
		// does not clone itself: a round trip through configuration and back to
		// operational must leave deferred= unchanged.
		char buf[128];
		snprintf(buf, sizeof(buf), "%%SCHEDQ imm=%u/%u(hw=%u) deferred=%u/%u(hw=%u)", system_scheduler->tasks_size(),
		         Scheduler::capacity(), system_scheduler->tasks_high_water(), system_scheduler->deferred_size(),
		         Scheduler::capacity(), system_scheduler->deferred_high_water());
		reply(buf);
	} else if (cmd == "%I2C") {
		// %I2C        -> scan du bus interne
		// %I2C EXT    -> scan du bus externe
		bench_i2c_scan(line);
	} else {
		return false;
	}
	return true;
}

/// @name %PULLAB -- A/B probe of a pull-down on GPS_PWR_EN (audit point 1, 2026-10-27)
/// GPS_PWR_EN is a push-pull output, driven HIGH while the GNSS rail is on, and
/// a pull-down there conducts VDD over ~13 kOhm (the BSP used to configure one).
/// This alternates the pin between pull-down and no pull every period, from the
/// BSP setting and leaving the level alone, so a PPK on the battery input shows
/// what the pull-down costs as a square wave -- with no debugger (APPROTECT
/// re-arms at every reset on these boards) and no USB (it would supply the
/// board). Started over USB, it keeps running once unplugged.
/// @{
static Scheduler::TaskHandle s_pullab_task;
static unsigned int s_pullab_left = 0;
static unsigned int s_pullab_period_ms = 0;
static bool s_pullab_pulldown = true;

static void pullab_apply(bool pulldown) {
	const auto &g = BSP::GPIO_Inits[BSP::GPIO::GPIO_GPS_PWR_EN];
	nrf_gpio_cfg(g.pin_number, g.dir, g.input, pulldown ? NRF_GPIO_PIN_PULLDOWN : NRF_GPIO_PIN_NOPULL, g.drive,
	             g.sense);
	s_pullab_pulldown = pulldown;
}

static void pullab_tick() {
	if (s_pullab_left == 0) {
		GPIOPins::init_pin(BSP::GPIO::GPIO_GPS_PWR_EN);  // back to the BSP pull
		return;
	}
	s_pullab_left--;
	pullab_apply(!s_pullab_pulldown);
	s_pullab_task =
	    system_scheduler->post_task_prio(pullab_tick, "PullAB", Scheduler::DEFAULT_PRIORITY, s_pullab_period_ms);
}
/// @}

/// @name %GPIOTEAB -- A/B probe of the GPIOTE IN channels (audit point 9, 2026-10-28)
/// The reed and the accelerometer interrupt are GPIOTE inputs in IN (event)
/// mode, "hi_accuracy" in the BSP. Nordic documents that mode as keeping a
/// clock that costs sleep current while any IN channel is on, from ~20 uA to
/// ~250 uA depending on the source; PORT (SENSE) events cost ~1 uA. This
/// alternates every IN channel between event mode and disabled, straight in
/// the GPIOTE registers so it catches every driver's channel at once (one left
/// on would keep the clock requested and hide the difference), and restores
/// them at the end. Edges are missed while disabled: bench only. Started over
/// USB, it keeps running once unplugged, like %PULLAB.
/// @{
static Scheduler::TaskHandle s_gpioteab_task;
static unsigned int s_gpioteab_left = 0;
static unsigned int s_gpioteab_period_ms = 0;
static bool s_gpioteab_off = false;
static uint32_t s_gpioteab_mask = 0;  ///< channels found in event mode at the start

static uint32_t gpiote_event_channels() {
	uint32_t mask = 0;
	for (unsigned int i = 0; i < GPIOTE_CH_NUM; i++)
		if ((NRF_GPIOTE->CONFIG[i] & GPIOTE_CONFIG_MODE_Msk) == (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos))
			mask |= 1U << i;
	return mask;
}

static void gpioteab_apply(bool off) {
	for (unsigned int i = 0; i < GPIOTE_CH_NUM; i++) {
		if (!(s_gpioteab_mask & (1U << i))) continue;
		const uint32_t config = NRF_GPIOTE->CONFIG[i] & ~GPIOTE_CONFIG_MODE_Msk;  // a register: no compound assignment
		NRF_GPIOTE->CONFIG[i] = off ? config : (config | (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos));
	}
	s_gpioteab_off = off;
}

static void gpioteab_tick() {
	if (s_gpioteab_left == 0) {
		gpioteab_apply(false);  // every channel back in event mode
		return;
	}
	s_gpioteab_left--;
	gpioteab_apply(!s_gpioteab_off);
	s_gpioteab_task =
	    system_scheduler->post_task_prio(gpioteab_tick, "GpioteAB", Scheduler::DEFAULT_PRIORITY, s_gpioteab_period_ms);
}
/// @}

/// @name The cost audit's probes (2026-10), one function per command
/// @{
#ifdef VSYS_SEL
static void bench_rails() {
	// Main-loop rail restores since boot: "settled" found VSYS at 2.3 V and
	// waited 2 ms, "skipped" found it at 3.3 V -- GNSS rail up, LED lit, flash
	// busy -- and did nothing. vsys_sel is the OUT latch now. LinkIt only:
	// RSPB has no VSYS_SEL.
	char buf[96];
	snprintf(buf, sizeof(buf), "%%RAILS settled=%lu skipped=%lu vsys_sel=%u",
	         (unsigned long)g_bench_vsys_restore_settled, (unsigned long)g_bench_vsys_restore_skipped,
	         (unsigned int)nrf_gpio_pin_out_read(BSP::GPIO_Inits[VSYS_SEL].pin_number));
	reply(buf);
}
#endif

static void bench_satrst(const std::string &line) {
	// The satellite module's NRST, by hand: 0 holds it in reset (the boot
	// state), Z releases it to the module-side pull-up (what SmdSat leaves
	// after a shutdown). Bench diagnostics for a module still wired to the
	// camera's UART lines and back-powered through them.
	char arg[4] = { 0 };
	sscanf(line.c_str(), "%%SATRST %3s", arg);
	const uint32_t pin = BSP::GPIO_Inits[SAT_RESET].pin_number;
	if (arg[0] == '0') {
		nrf_gpio_pin_clear(pin);
		nrf_gpio_cfg(pin, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL,
		             NRF_GPIO_PIN_S0D1, NRF_GPIO_PIN_NOSENSE);
		reply("%SATRST OK low");
	} else if (arg[0] == 'Z' || arg[0] == 'z') {
		nrf_gpio_cfg_default(pin);
		reply("%SATRST OK high-z");
	} else {
		reply("%SATRST ERR usage: %SATRST <0|Z>");
	}
}

static void bench_pullab(const std::string &line) {
	unsigned int period_s = 0, toggles = 0;
	system_scheduler->cancel_task(s_pullab_task);
	if (line.find("STOP") != std::string::npos) {
		s_pullab_left = 0;
		GPIOPins::init_pin(BSP::GPIO::GPIO_GPS_PWR_EN);
		reply("%PULLAB OK stopped (BSP pull restored)");
	} else if (sscanf(line.c_str(), "%%PULLAB %u %u", &period_s, &toggles) == 2 && period_s >= 5 && toggles >= 1
	           && toggles <= 200) {
		GPIOPins::init_pin(BSP::GPIO::GPIO_GPS_PWR_EN);
		s_pullab_pulldown = BSP::GPIO_Inits[BSP::GPIO::GPIO_GPS_PWR_EN].pull == NRF_GPIO_PIN_PULLDOWN;
		s_pullab_left = toggles;
		s_pullab_period_ms = period_s * 1000U;
		s_pullab_task =
		    system_scheduler->post_task_prio(pullab_tick, "PullAB", Scheduler::DEFAULT_PRIORITY, s_pullab_period_ms);
		char buf[112];
		snprintf(buf, sizeof(buf), "%%PULLAB OK period_s=%u toggles=%u bsp_pull=%s rail=%u t0_ms=%llu", period_s,
		         toggles, s_pullab_pulldown ? "down" : "none", GPIOPins::is_gnss_rail_active() ? 1U : 0U,
		         (unsigned long long)PMU::get_timestamp_ms());
		reply(buf);
	} else {
		reply("%PULLAB ERR usage: %PULLAB <period_s 5..> <toggles 1..200> | %PULLAB STOP");
	}
}

static void bench_gpiote() {
	// The raw GPIOTE state behind %GPIOTEAB: every channel's CONFIG, the
	// interrupt enables, the reed pin's PIN_CNF (SENSE = PORT mode), and how
	// many edges the reed ISR got since boot.
	extern unsigned int g_bench_switch_edges, g_bench_switch_debounces;
	extern unsigned int g_bench_switch_gap_min_ms, g_bench_switch_gap_max_ms, g_bench_switch_gap_n;
	extern uint64_t g_bench_switch_gap_sum_ms;
	char buf[112];
	snprintf(buf, sizeof(buf), "%%GPIOTE reed_edges=%u debounces=%u uptime_ms=%llu", g_bench_switch_edges,
	         g_bench_switch_debounces, (unsigned long long)PMU::get_timestamp_ms());
	reply(buf);
	snprintf(buf, sizeof(buf), "%%GPIOTE reenable_to_edge n=%u min_ms=%u max_ms=%u mean_ms=%llu", g_bench_switch_gap_n,
	         g_bench_switch_gap_n ? g_bench_switch_gap_min_ms : 0U, g_bench_switch_gap_max_ms,
	         (unsigned long long)(g_bench_switch_gap_n ? g_bench_switch_gap_sum_ms / g_bench_switch_gap_n : 0U));
	reply(buf);
	const uint32_t reed = BSP::GPIO_Inits[BSP::GPIO::GPIO_REED_SW].pin_number;
	NRF_GPIO_Type *port = (reed < 32) ? NRF_P0 : NRF_P1;
	snprintf(buf, sizeof(buf), "%%GPIOTE intenset=0x%08lx reed=P%u.%02u pin_cnf=0x%08lx in=%lu latch=0x%08lx",
	         (unsigned long)NRF_GPIOTE->INTENSET, reed < 32 ? 0U : 1U, (unsigned int)(reed & 31),
	         (unsigned long)port->PIN_CNF[reed & 31], (unsigned long)((port->IN >> (reed & 31)) & 1U),
	         (unsigned long)port->LATCH);
	reply(buf);
	for (unsigned int i = 0; i < GPIOTE_CH_NUM; i++) {
		snprintf(buf, sizeof(buf), "%%GPIOTE ch=%u config=0x%08lx", i, (unsigned long)NRF_GPIOTE->CONFIG[i]);
		reply(buf);
	}
}

static void bench_gpioteab(const std::string &line) {
	unsigned int period_s = 0, toggles = 0;
	system_scheduler->cancel_task(s_gpioteab_task);
	if (line.find("STOP") != std::string::npos) {
		s_gpioteab_left = 0;
		if (s_gpioteab_off) gpioteab_apply(false);
		reply("%GPIOTEAB OK stopped (IN channels back in event mode)");
	} else if (sscanf(line.c_str(), "%%GPIOTEAB %u %u", &period_s, &toggles) == 2 && period_s >= 5 && toggles >= 1
	           && toggles <= 200) {
		if (s_gpioteab_off) gpioteab_apply(false);
		s_gpioteab_mask = gpiote_event_channels();
		s_gpioteab_left = toggles;
		s_gpioteab_period_ms = period_s * 1000U;
		s_gpioteab_task = system_scheduler->post_task_prio(gpioteab_tick, "GpioteAB", Scheduler::DEFAULT_PRIORITY,
		                                                   s_gpioteab_period_ms);
		char buf[112];
		snprintf(buf, sizeof(buf), "%%GPIOTEAB OK period_s=%u toggles=%u in_channels=0x%02x t0_ms=%llu", period_s,
		         toggles, (unsigned int)s_gpioteab_mask, (unsigned long long)PMU::get_timestamp_ms());
		reply(buf);
	} else {
		reply("%GPIOTEAB ERR usage: %GPIOTEAB <period_s 5..> <toggles 1..200> | %GPIOTEAB STOP");
	}
}

/// @brief The regular files at the root of the file system, space separated.
static std::string bench_root_files() {
	if (!main_filesystem) return "no-fs";
	main_filesystem->power_up();
	lfs_t *lfs = static_cast<lfs_t *>(main_filesystem->get_private_data());
	std::string out;
	lfs_dir_t dir;
	if (lfs_dir_open(lfs, &dir, "/") == 0) {
		struct lfs_info info;
		while (lfs_dir_read(lfs, &dir, &info) > 0)
			if (info.type == LFS_TYPE_REG) out += std::string(info.name) + " ";
		lfs_dir_close(lfs, &dir);
	}
	main_filesystem->power_down();
	return out;
}

/// @brief %RECOVERY: the boot-fail recovery's factory reset now, then a reboot --
/// what the FSM does after five failed boots (and, on LinkIt, 30 min of backoff),
/// on demand, to watch it on a board. Identity and safety are kept, the state goes;
/// the files at the root are listed before and after.
static void bench_recovery() {
	reply("%RECOVERY before " + bench_root_files());
	try {
		configuration_store->factory_reset_for_recovery();
	} catch (...) {
		reply("%RECOVERY ERR the factory reset threw");
	}
	reply("%RECOVERY after " + bench_root_files());
	PMU::delay_ms(100);  // let the replies leave
	PMU::reset(false);
}

static void bench_mem() {
	// The two watermarks MemoryMonitorService only journals every 12 h, on demand:
	// the stack peak since boot (stack painting) against the reserved size, and
	// the heap's lowest free since boot. To check a __STACK_SIZE or
	// configTOTAL_HEAP_SIZE change after a session that exercised the board.
	const HeapStats_t heap = MEMMANG::heap_stats();
	char buf[112];
	snprintf(buf, sizeof(buf), "%%MEM stack_max=%lu stack_size=%u heap_min_free=%u heap_free=%u heap_size=%u",
	         (unsigned long)MEMMANG::max_stack_usage(), (unsigned int)__STACK_SIZE,
	         (unsigned int)heap.xMinimumEverFreeBytesRemaining, (unsigned int)heap.xAvailableHeapSpaceInBytes,
	         (unsigned int)configTOTAL_HEAP_SIZE);
	reply(buf);
}

static void bench_swsraw(const std::string &line) {
	// Feed the SWS analog detector a raw reading of our choice. It goes through
	// the whole real path -- filters, levels, calibration, the UW detector and
	// its broadcast -- which %DIVE / %SURFACE skip: a surfacing without water.
	// The detector decides again (%DIVE / %SURFACE hold released).
	unsigned int v = 0;
	char buf[48];
	if (line.find("OFF") != std::string::npos) {
		UWDetectorService::bench_raw_override = -1;
		reply("%SWSRAW OK off (electrode)");
	} else if (sscanf(line.c_str(), "%%SWSRAW %u", &v) == 1 && v <= 16383) {
		UWDetectorService::bench_forced_state = -1;
		UWDetectorService::bench_raw_override = (int32_t)v;
		snprintf(buf, sizeof(buf), "%%SWSRAW OK raw=%u", v);
		reply(buf);
	} else {
		reply("%SWSRAW ERR usage: %SWSRAW <0..16383> | %SWSRAW OFF");
	}
}

/// @brief rails, pull-downs, GPIOTE, the satellite reset line, a raw SWS reading, memory,
/// the boot-fail recovery on demand
static bool handle_probe_cmd(const std::string &cmd, const std::string &line) {
	if (cmd == "%SATRST") {
		bench_satrst(line);
	} else if (cmd == "%PULLAB") {
		bench_pullab(line);
	} else if (cmd == "%GPIOTE") {
		bench_gpiote();
	} else if (cmd == "%GPIOTEAB") {
		bench_gpioteab(line);
	} else if (cmd == "%SWSRAW") {
		bench_swsraw(line);
	} else if (cmd == "%MEM") {
		bench_mem();
	} else if (cmd == "%RECOVERY") {
		bench_recovery();
#ifdef VSYS_SEL
	} else if (cmd == "%RAILS") {
		bench_rails();
#endif
	} else {
		return false;
	}
	return true;
}
/// @}

static void bench_satlog(const std::string &line) {
#if ENABLE_CAM_SENSOR && defined(CAM_CONTROL_UART) && (CAM_CONTROL_UART == 1)
	// UARTE1 is the camera's on this build, and the probe drives its
	// registers directly: never under a link that may be open.
	if (GPIOPins::is_cam_rail_active()) {
		reply("%SATLOG ERR uarte1-in-use-by-camera (%CAM OFF or %CAM PWR 0 first)");
		return;
	}
#endif
#ifndef BOARD_RSPB
	cmd_satlog(line);
#else
	(void)line;
	reply("%SATLOG ERR not-on-rspb (no satellite-slot UART)");
#endif
}

static void bench_hauled(const std::string &line) {
	// The hauled state lives in .noinit and SURVIVES a reboot, which makes
	// every case that drives it non-idempotent: the second run finds the
	// board already HAULED, no AT_SEA -> HAULED line is emitted, and the
	// case blames firmware that did nothing wrong. That is exactly what
	// happened on 2026-08-27, and there was no way to even SEE the state.
	//
	// HMP00=0 does clear in_hauled (evaluate() resets it when detection is
	// disabled), but it leaves the dry spell standing -- so the next enable
	// can engage HAULED immediately. RESET clears all of it, which is what
	// a case needs to start from a known AT_SEA.
	//
	// %HAULED       -> classifier state, plus the dry time it is comparing
	//                  against HMP01 (0 while in the water, -1 without RTC).
	//                  A fresh board reads last_uw=0, and evaluate()
	//                  deliberately stays AT_SEA there: never having been
	//                  in the water is not being hauled out.
	// %HAULED RESET -> clear the state so a case can start from AT_SEA.
	if (line.find(" RESET") != std::string::npos) {
		HauledModeService::reset_for_tests();
		reply("%HAULED OK reset");
	} else {
		char buf[176];
		long dry = -1;
		const std::time_t dry_since = HauledModeService::dry_since_rtc();
		if (rtc && rtc->is_set()) dry = dry_since ? (long)(rtc->gettime() - dry_since) : 0;
		snprintf(buf, sizeof(buf), "%%HAULED en=%u state=%s last_uw=%u dry_s=%ld returns=%u/%u threshold_h=%u",
		         (unsigned)configuration_store->read_param<bool>(ParamID::HAULED_DETECT_EN),
		         HauledModeService::is_hauled() ? "HAULED" : "AT_SEA", (unsigned)HauledModeService::last_uw_event_rtc(),
		         dry, HauledModeService::uw_events_since_hauled(),
		         configuration_store->read_param<unsigned int>(ParamID::HAULED_RETURN_EVENTS),
		         configuration_store->read_param<unsigned int>(ParamID::HAULED_IDLE_THRESHOLD_H));
		reply(buf);
	}
}

/// @brief OTA, satellite log, battery, scheduling, moored and hauled modes
static bool handle_services_cmd(const std::string &cmd, const std::string &line) {
	if (cmd == "%OTA") {
		bench_ota(line);
	} else if (cmd == "%SATLOG") {
		bench_satlog(line);
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
	} else if (cmd == "%MOORED") {
		// Everything needed to tune MOORED_RADIUS_M and the AXP03/AXP04
		// accelerometer threshold without guessing: the classifier's own view
		// plus the live distance from the reference anchor. `d` is -1 until the
		// first valid FIX plants the anchor.
		char buf[192];
		double d = -1.0;
		if (MooredModeService::has_reference() && gps_service) {
			const GPSLogEntry &last = configuration_store->get_last_gps_entry();
			if (last.info.valid) d = MooredModeService::distance_to_reference_m(last.info.lat, last.info.lon);
		}
		snprintf(buf, sizeof(buf),
		         "%%MOORED en=%u state=%s ref=%d still=%u/%u motion=%u/%u "
		         "radius=%u m holdoff=%u s d_last_fix=%.1f m",
		         (unsigned)configuration_store->read_param<bool>(ParamID::MOORED_DETECT_EN),
		         MooredModeService::is_moored() ? "MOORED" : "UNDERWAY", (int)MooredModeService::has_reference(),
		         MooredModeService::stationary_fixes(),
		         configuration_store->read_param<unsigned int>(ParamID::MOORED_ENTER_FIXES),
		         MooredModeService::motion_events(),
		         configuration_store->read_param<unsigned int>(ParamID::MOORED_EXIT_EVENTS),
		         configuration_store->read_param<unsigned int>(ParamID::MOORED_RADIUS_M),
		         configuration_store->read_param<unsigned int>(ParamID::MOORED_AXL_HOLDOFF_S), d);
		reply(buf);
	} else if (cmd == "%HAULED") {
		bench_hauled(line);
	} else if (cmd == "%BOOTFAIL") {
		// Own command name: under "%BOOT" this branch was shadowed by the
		// reset-cause branch above and could never be reached.
		// Boot-failure counter, straight out of .noinit RAM. Three consecutive
		// failed boots trigger a factory reset, which wipes the Argos
		// credentials -- the sealed-device brick. A counter that stops clearing
		// on a healthy boot is therefore a silent countdown, and this is the
		// only way to see it.
		unsigned int echecs = 0, usine = 0;
		bench_bootfail_read(echecs, usine);
		char buf[112];
		snprintf(buf, sizeof(buf), "%%BOOTFAIL failures=%u factory_attempted=%u", echecs, usine);
		reply(buf);
	} else {
		return false;
	}
	return true;
}

/// @brief configuration mode, synthetic GPS fixes and GNSS power requests
static bool handle_gps_cmd(const std::string &cmd, const std::string &line) {
	if (cmd == "%CFG") {
		if (GenTracker::is_in_state<ConfigurationState>()) {
			reply("%CFG OK already-config");
		} else if (GenTracker::is_in_state<BootState>() || GenTracker::is_in_state<PreOperationalState>()) {
			// Accepted here, the gesture was consumed by boot, which then left
			// configuration 0.3 s later on its own: every DTE command that
			// followed landed in PreOp/Operational and was ignored (bench
			// 2026-10-10/11, after a flash and after --pinreset). Refuse so the
			// host retries once the board has settled.
			reply("%CFG ERR booting (retry when OPERATIONAL)");
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
		else {
			gps_service->bench_inject_cloudlocate();
			reply("%GPSCL OK cloudlocate injected");
		}
	} else if (cmd == "%PMREQ") {
		// A/B a chaud du flag FORCE de UBX-RXM-PMREQ (SAM-M10Q IM §2.6.3.2 :
		// « The "force" flag must be set ... to enter software standby mode »).
		//   %PMREQ            -> etat + compteurs
		//   %PMREQ <n>        -> pose flags=n ET remet les compteurs a zero
		// BACKUP=2 (valeur historique), BACKUP|FORCE=6 (valeur exigee par le manuel).
		unsigned int f = 0;
		char buf[190];
		if (sscanf(line.c_str(), "%%PMREQ SETTLE %u", &f) == 1) {
			M10QAsyncReceiver::bench_pmreq_settle_ms = f;
			M10QAsyncReceiver::bench_pmreq_reset_stats();
			snprintf(buf, sizeof(buf), "%%PMREQ OK settle=%u ms stats-remis-a-zero", f);
			reply(buf);
		} else if (sscanf(line.c_str(), "%%PMREQ %u", &f) == 1) {
			M10QAsyncReceiver::bench_pmreq_flags = f;
			M10QAsyncReceiver::bench_pmreq_reset_stats();
			snprintf(buf, sizeof(buf), "%%PMREQ OK flags=%u stats-remis-a-zero", f);
			reply(buf);
		} else {
			snprintf(buf, sizeof(buf), "%%PMREQ flags=%u settle=%u seq=%u probes=%u first_ok=%u giveup=%u",
			         (unsigned)M10QAsyncReceiver::bench_pmreq_flags, M10QAsyncReceiver::bench_pmreq_settle_ms,
			         M10QAsyncReceiver::bench_pmreq_seq, M10QAsyncReceiver::bench_pmreq_probes,
			         M10QAsyncReceiver::bench_pmreq_first_ok, M10QAsyncReceiver::bench_pmreq_giveup);
			reply(buf);
		}
	} else if (cmd == "%NOFIX") {
		if (!GenTracker::is_in_state<OperationalState>())
			reply("%NOFIX ERR not-operational (use %OP first)");
		else if (!gps_service)
			reply("%NOFIX ERR no-gps-service");
		else {
			gps_service->bench_inject_nofix();
			reply("%NOFIX OK");
		}
	} else {
		return false;
	}
	return true;
}

/// @brief dive/surface events, reset cause, reed, SENSE pins and storage entry
static bool handle_sensors_cmd(const std::string &cmd, const std::string &line) {
	if (cmd == "%DIVE" || cmd == "%SURFACE") {
		// Simulate the saltwater switch: broadcast the same UW_SENSOR event the SWS
		// service emits (source=UW_SENSOR, type=SERVICE_LOG_UPDATED, data=bool). Each
		// Service::notify_peer_event routes it to notify_underwater_state(). %DIVE =
		// wet (underwater=true), %SURFACE = dry (surfaced=false) — required to drive
		// SURFACING_BURST's progressive Doppler cascade and underwater gating.
		// Hold the simulated state in the detector too, until the next
		// %DIVE / %SURFACE or %UWREAL: otherwise the real electrode (dry on the
		// bench) reports "surfaced" on its next sample, ~6 s later.
		UWDetectorService::bench_forced_state = (cmd == "%DIVE") ? 1 : 0;
		ServiceEvent e;
		e.event_source = ServiceIdentifier::UW_SENSOR;
		e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
		e.event_data = (cmd == "%DIVE");  // true = underwater, false = surfaced
		ServiceManager::notify_peer_event(e);
		{
			// Absolute uptime on both ends (here and in SmdSat's TXSTART) so a host
			// stopwatch can split surface-to-air into service latency and driver
			// latency without another firmware trace.
			char b[80];
			snprintf(b, sizeof(b), "%s uptime=%llu", cmd == "%DIVE" ? "%DIVE OK underwater" : "%SURFACE OK surfaced",
			         (unsigned long long)PMU::get_timestamp_ms());
			reply(b);
		}
	} else if (cmd == "%UWREAL") {
		// Hand the underwater state back to the real detector.
		UWDetectorService::bench_forced_state = -1;
		reply("%UWREAL OK detector=real");
	} else if (cmd == "%RST") {
		// Boot forensics: decoded cause plus the raw registers behind it. The
		// decisive datum for the storage work is bit 16 (OFF) of RESETREAS —
		// whether it survives the bootloader — and the GPREGRET marker.
		char buf[96];
		snprintf(buf, sizeof(buf), "%%RST OK cause=%s RESETREAS=0x%08x GPREGRET=0x%02x", PMU::reset_cause_str(),
		         (unsigned int)PMU::reset_reason_raw(), (unsigned int)PMU::boot_gpregret());
		reply(buf);
	} else if (cmd == "%REED") {
		char buf[64];
		snprintf(buf, sizeof(buf), "%%REED OK pin=%u engaged=%u", (unsigned int)GPIOPins::value(BSP::GPIO_REED_SW),
		         (unsigned int)(GPIOPins::value(BSP::GPIO_REED_SW) == REED_SWITCH_ACTIVE_STATE));
		reply(buf);
	} else if (cmd == "%SENSE") {
		// Every pin left with a SENSE field armed is a System OFF wake source.
		// Only the reed should appear here; anything else (P0.11 armed by the
		// bootloader's DFU button, a sensor INT...) is a spurious-wake vector.
		char buf[64];
		unsigned int found = 0;
		for (unsigned int pin = 0; pin < 48; pin++) {
			NRF_GPIO_Type *port = (pin < 32) ? NRF_P0 : NRF_P1;
			uint32_t cnf = port->PIN_CNF[pin & 31];
			uint32_t sense = (cnf & GPIO_PIN_CNF_SENSE_Msk) >> GPIO_PIN_CNF_SENSE_Pos;
			if (sense) {
				snprintf(buf, sizeof(buf), "%%SENSE pin=P%u.%02u sense=%s", pin < 32 ? 0u : 1u, pin & 31,
				         sense == GPIO_PIN_CNF_SENSE_High ? "HIGH" : "LOW");
				reply(buf);
				found++;
			}
		}
		snprintf(buf, sizeof(buf), "%%SENSE OK count=%u", found);
		reply(buf);
	} else if (cmd == "%STORE") {
		// Drive a real storage entry without a magnet. Requires the firmware to
		// be built with BENCH_ALLOW_POWERDOWN, otherwise powerdown() is a no-op
		// under BENCH_TEST and the board just stays alive.
#ifdef BENCH_ALLOW_POWERDOWN
		reply("%STORE OK powering-down (USB link will drop)");
		system_scheduler->post_task_prio([]() { PMU::powerdown(); }, "BenchStore");
#else
		reply("%STORE ERR rebuild with -DBENCH_ALLOW_POWERDOWN=ON");
#endif
	} else {
		return false;
	}
	return true;
}

#if ENABLE_CAM_SENSOR && defined(CAM_CONTROL_UART) && (CAM_CONTROL_UART == 1)
static std::string cam_hex(const uint8_t *data, size_t len) {
	return Binascii::hexlify(std::string((const char *)data, len));
}

/// @brief "features=0x0077 names=POWER,WIFI,..." -- one token per value, the
/// way the bench harness splits every % reply.
static std::string cam_features(uint16_t features) {
	char names[96];
	runcam::describe_features(features, names, sizeof(names));
	for (char *c = names; *c; c++)
		if (*c == ' ') *c = ',';
	char buf[128];
	snprintf(buf, sizeof(buf), "features=0x%04X names=%s", (unsigned int)features, names[0] ? names : "-");
	return buf;
}

/// @brief Send one frame and report it, the raw bytes that came back and, when
/// they decode as the answer to @p command, that answer.
static void cam_exchange(const char *tag, runcam::Command command, const uint8_t *frame, size_t len,
                         unsigned int window_ms) {
	uint8_t rx[runcam::MAX_FRAME];
	const int n = runcam_uart_device->bench_exchange(frame, len, rx, sizeof(rx), window_ms);
	if (n < 0) {
		reply(std::string("%CAM ") + tag + " ERR link-unavailable state=" + runcam_uart_device->state_name()
		      + " (%CAM PWR 1, or %CAM ON and wait for ON)");
		return;
	}
	std::string out = std::string("%CAM ") + tag + " tx=" + cam_hex(frame, len)
	                  + " rx=" + (n ? cam_hex(rx, (size_t)n) : std::string("none"));
	runcam::ResponseDecoder decoder;
	if (decoder.expect(command)) {
		std::string verdict = " answer=none";
		for (int i = 0; i < n; i++) {
			const runcam::ResponseDecoder::Status st = decoder.feed(rx[i]);
			if (st == runcam::ResponseDecoder::Status::CORRUPT) {
				verdict = " answer=corrupt";
				break;
			}
			if (st != runcam::ResponseDecoder::Status::COMPLETE) continue;
			verdict = " answer=ok";
			runcam::DeviceInfo info;
			if (command == runcam::Command::GET_DEVICE_INFO
			    && runcam::parse_device_info(decoder.frame(), decoder.length(), info))
				verdict += " proto=" + std::to_string(info.protocol_version) + " " + cam_features(info.features);
			break;
		}
		out += verdict;
	}
	reply(out);
}

/// @brief %CAM — the UART-driven camera (CAM_CONTROL=UART builds).
///
///   %CAM [STATE]      driver state, what the camera answered, CAP06, the
///                     recording the driver believes in and whether the card
///                     confirmed it, the card last read (-1: never this
///                     session), the clock set, the resolution read (CAP07
///                     codes) and whether CAP07 restarted the camera, captures,
///                     CRC errors
///   %CAM LINK         the UART link's driver and peripheral state, raw
///   %CAM ON | OFF     the real sequences (power_on / power_off), CAP06 included
///   %CAM PWR 1 | 0    rail only, no recording command: manual exploration. 0
///                     cuts without a stop -- a recording in progress is lost
///   %CAM INFO         GET_DEVICE_INFO, decoded
///   %CAM CTRL <0..4>  CAMERA_CONTROL: 0 Wi-Fi/Mode (x3 in 2 s in standby =
///                     FACTORY RESET), 1 Power (record toggle), 2 change mode,
///                     3 START_RECORDING, 4 STOP_RECORDING
///   %CAM RAW <cmd> [byte..]  hex; header and CRC are added, 1 s of answer kept
///
/// INFO, CTRL and RAW need the camera ON or held by PWR 1, where the driver
/// leaves the link alone. Set CAP01=0 first for manual work: an enabled
/// CAMService would otherwise run its own sessions underneath.
///
/// One function per subcommand below, bench_cam() only dispatches.
static void cam_state() {
	const runcam::DeviceInfo &info = runcam_uart_device->device_info();
	char buf[320];
	snprintf(buf, sizeof(buf),
	         "%%CAM state=%s rail=%u answered=%u proto=%u %s mode=%u rec=%u verified=%u "
	         "free_mb=%d remaining_s=%d clock=%u res=%u restarted=%u captures=%u corrupt=%u rx_errors=%u",
	         runcam_uart_device->state_name(), GPIOPins::is_cam_rail_active() ? 1u : 0u,
	         runcam_uart_device->has_answered() ? 1u : 0u, (unsigned int)info.protocol_version,
	         cam_features(info.features).c_str(), (unsigned int)runcam_uart_device->record_mode(),
	         runcam_uart_device->is_recording_assumed() ? 1u : 0u,
	         runcam_uart_device->is_recording_verified() ? 1u : 0u, runcam_uart_device->last_free_mb(),
	         runcam_uart_device->last_remaining_s(), runcam_uart_device->is_clock_set() ? 1u : 0u,
	         (unsigned int)runcam_uart_device->resolution_read(), runcam_uart_device->was_restarted() ? 1u : 0u,
	         runcam_uart_device->get_num_captures(), runcam_uart_device->corrupt_answers(),
	         runcam_uart_device->link_errors());
	reply(buf);
}

static void cam_sniff() {
	// GET_DEVICE_INFO through the driver's link, then both lines watched at
	// the GPIO level for 60 ms, edges counted: whether our TX moved and the
	// camera answered, whatever libuarte makes of it. Bench diagnostics.
	uint8_t frame[3];
	const size_t len = runcam::encode(runcam::Command::GET_DEVICE_INFO, nullptr, 0, frame, sizeof(frame));
	const uint32_t rx = BSP::UARTAsync_Inits[1].config.rx_pin & 0x1F;
	const uint32_t tx = BSP::UARTAsync_Inits[1].config.tx_pin & 0x1F;
	const uint32_t tx_cnf = NRF_P0->PIN_CNF[tx];
	NRF_P0->PIN_CNF[tx] = tx_cnf & ~GPIO_PIN_CNF_INPUT_Msk;  // connect its input buffer: read what is driven
	uint8_t sink[runcam::MAX_FRAME];
	if (runcam_uart_device->bench_exchange(frame, len, sink, sizeof(sink), 0) < 0) {
		NRF_P0->PIN_CNF[tx] = tx_cnf;
		reply("%CAM SNIFF ERR link-unavailable (%CAM PWR 1 first)");
		return;
	}
	unsigned int rx_edges = 0, tx_edges = 0, rx_low = 0;
	uint32_t last = NRF_P0->IN;
	const uint64_t end = PMU::get_timestamp_ms() + 60;
	while (PMU::get_timestamp_ms() < end) {
		const uint32_t in = NRF_P0->IN;
		const uint32_t diff = in ^ last;
		if (diff & (1u << rx)) rx_edges++;
		if (diff & (1u << tx)) tx_edges++;
		if (!(in & (1u << rx))) rx_low++;
		last = in;
	}
	NRF_P0->PIN_CNF[tx] = tx_cnf;
	const int got = runcam_uart_device->bench_receive(sink, sizeof(sink));
	reply("%CAM SNIFF tx_edges=" + std::to_string(tx_edges) + " rx_edges=" + std::to_string(rx_edges)
	      + " rx_low_samples=" + std::to_string(rx_low) + " libuarte_bytes=" + std::to_string(got < 0 ? 0 : got));
}

static void cam_pwr(const char *args) {
	unsigned int on = 2;
	sscanf(args, "%u", &on);
	if (on > 1) {
		reply("%CAM ERR usage: %CAM PWR <1|0>");
	} else if (!runcam_uart_device->bench_power(on == 1)) {
		reply(std::string("%CAM PWR ERR state=") + runcam_uart_device->state_name());
	} else {
		reply(std::string("%CAM PWR OK state=") + runcam_uart_device->state_name());
	}
}

static void cam_ctrl(const char *args) {
	unsigned int action = 99;
	sscanf(args, "%u", &action);
	if (action > static_cast<unsigned int>(runcam::ControlAction::STOP_RECORDING)) {
		reply("%CAM ERR usage: %CAM CTRL <0..4>");
		return;
	}
	uint8_t frame[4];
	const size_t len = runcam::encode_control(static_cast<runcam::ControlAction>(action), frame, sizeof(frame));
	cam_exchange("CTRL", runcam::Command::CAMERA_CONTROL, frame, len, 300);
}

static void cam_raw(const char *args) {
	// Every token must be one byte, or nothing is sent: a frame cut short at a
	// typo would still go out with a valid CRC -- a WRITE_SETTING missing its
	// value, say.
	uint8_t bytes[runcam::MAX_FRAME];
	size_t count = 0;
	const char *p = args;
	bool bad = false;
	while (*p) {
		while (*p == ' ')
			p++;
		if (!*p) break;
		char *end = nullptr;
		const unsigned long v = strtoul(p, &end, 16);
		if (end == p || v > 0xFF || (*end && *end != ' ') || count == sizeof(bytes)) {
			bad = true;
			break;
		}
		bytes[count++] = (uint8_t)v;
		p = end;
	}
	const runcam::Command command = static_cast<runcam::Command>(count ? bytes[0] : 0);
	uint8_t frame[runcam::MAX_FRAME];
	const size_t len = (count && !bad) ? runcam::encode(command, bytes + 1, count - 1, frame, sizeof(frame)) : 0;
	if (len == 0) {
		reply("%CAM ERR usage: %CAM RAW <cmd> [byte..] (hex, at most 61 payload bytes)");
		return;
	}
	cam_exchange("RAW", command, frame, len, 1000);
}

static void bench_cam(const std::string &line) {
	if (!runcam_uart_device) {
		reply("%CAM ERR no-camera");
		return;
	}
	char sub[8] = { 0 };
	sscanf(line.c_str(), "%%CAM %7s", sub);
	const std::string s(sub);
	const size_t sub_at = s.empty() ? std::string::npos : line.find(s, 4);
	const char *args = line.c_str() + ((sub_at == std::string::npos) ? line.size() : sub_at + s.size());

	if (s.empty() || s == "STATE") {
		cam_state();
	} else if (s == "SNIFF") {
		cam_sniff();
	} else if (s == "LINK") {
		char buf[480];
		runcam_uart_device->describe_link(buf, sizeof(buf));
		reply(std::string("%CAM LINK ") + buf);
	} else if (s == "ON") {
		runcam_uart_device->power_on();
		reply(std::string("%CAM OK on-requested state=") + runcam_uart_device->state_name());
	} else if (s == "OFF") {
		runcam_uart_device->power_off();
		reply(std::string("%CAM OK off-requested state=") + runcam_uart_device->state_name());
	} else if (s == "PWR") {
		cam_pwr(args);
	} else if (s == "INFO") {
		uint8_t frame[3];
		const size_t len = runcam::encode(runcam::Command::GET_DEVICE_INFO, nullptr, 0, frame, sizeof(frame));
		cam_exchange("INFO", runcam::Command::GET_DEVICE_INFO, frame, len, 500);
	} else if (s == "CTRL") {
		cam_ctrl(args);
	} else if (s == "RAW") {
		cam_raw(args);
	} else {
		reply("%CAM ERR usage: %CAM [STATE|LINK|ON|OFF|PWR <1|0>|INFO|CTRL <0..4>|RAW <cmd> [byte..]]");
	}
}
#endif

/// @brief the UART-driven camera (CAM_CONTROL=UART builds only)
static bool handle_camera_cmd(const std::string &cmd, const std::string &line) {
	if (cmd != "%CAM") return false;
#if ENABLE_CAM_SENSOR && defined(CAM_CONTROL_UART) && (CAM_CONTROL_UART == 1)
	bench_cam(line);
#else
	(void)line;
	reply("%CAM ERR not-a-camera-uart-build");
#endif
	return true;
}
/// @}

bool bench::handle_line(const std::string &raw) {
	// Trim trailing CR/LF/space.
	std::string line = raw;
	while (!line.empty() && (line.back() == '\r' || line.back() == '\n' || line.back() == ' '))
		line.pop_back();

	if (line.empty() || line[0] != '%') return false;

	const std::string cmd = line.substr(0, line.find(' '));

	if (handle_state_cmd(cmd, line)) return true;
	if (handle_argos_cmd(cmd, line)) return true;
	if (handle_services_cmd(cmd, line)) return true;
	if (handle_gps_cmd(cmd, line)) return true;
	if (handle_sensors_cmd(cmd, line)) return true;
	if (handle_camera_cmd(cmd, line)) return true;
	if (handle_probe_cmd(cmd, line)) return true;

	reply(std::string("%ERR unknown-cmd ") + cmd);
	return true;
}

namespace {

void bench_poll() {
	// While in ConfigurationState the config USB poller owns the RX and routes
	// '%' lines into handle_line(); reading here too would steal DTE bytes and
	// race on the CDC buffer. Everywhere else, we are the sole USB reader.
	if (!GenTracker::is_in_state<ConfigurationState>()) {
		auto &usb = UsbInterface::get_instance();
		if (usb.has_data()) {
			const std::string line = usb.read_line();
			if (line.size()) bench::handle_line(line);
		}
	}
	s_poll_task = system_scheduler->post_task_prio(bench_poll, "BenchPoll", Scheduler::DEFAULT_PRIORITY, BENCH_POLL_MS);
}

}  // namespace

void bench::start_poll() {
	reply("%BENCH ready");
	DEBUG_WARN("bench::start_poll: BENCH_TEST console active ('%%' commands over USB-CDC)");
	s_poll_task = system_scheduler->post_task_prio(bench_poll, "BenchPoll", Scheduler::DEFAULT_PRIORITY, BENCH_POLL_MS);
}

namespace {

Scheduler::TaskHandle s_autoinject_task;

// RSPB auto-inject: once per boot, when Operational + gps_service is up, inject a
// synthetic fix so the satellite TX pipeline runs with no antenna. Statics reset
// on the soft reset that ends each simulated TPL cycle, so every boot injects once.
void auto_inject_fire() {
	static bool s_done = false;
	static unsigned int s_tries = 0;
	if (s_done) return;
	if (GenTracker::is_in_state<OperationalState>() && gps_service) {
		s_done = true;
		DEBUG_WARN("bench::auto_inject: injecting synthetic fix (RSPB duty-cycle)");
		gps_service->bench_inject_fix(-21.0097, 55.2707, 2500, 8);
		return;
	}
	if (++s_tries > 60) {  // ~60 s: give up (no GNSS service on this build)
		DEBUG_WARN("bench::auto_inject: gps_service not ready after 60 s — TX will rely on heartbeat");
		return;
	}
	s_autoinject_task =
	    system_scheduler->post_task_prio(auto_inject_fire, "BenchAutoInject", Scheduler::DEFAULT_PRIORITY, 1000);
}

}  // namespace

void bench::start_auto_inject() {
	DEBUG_WARN("bench::start_auto_inject: RSPB compressed duty-cycle — auto-inject fix once Operational");
	s_autoinject_task =
	    system_scheduler->post_task_prio(auto_inject_fire, "BenchAutoInject", Scheduler::DEFAULT_PRIORITY, 5000);
}

#endif  // BENCH_TEST
