/**
 * @file gentracker_config_state.cpp
 * @brief GenTracker — ConfigurationState.
 *
 * Split out of gentracker.cpp, where this one state was 588 of 1 483 lines: BLE
 * advertising and events, the OTA transfer, the USB-CDC poll loop and the DTE
 * handler, all in the middle of a file whose other six states average forty
 * lines each.
 *
 * Nothing in gentracker.hpp changed -- these are member functions already
 * declared there.
 *
 * NOTE FOR VERIFICATION: the USB_DTE_ENABLED block at the bottom is not compiled
 * by any host test target (the macro is only defined by the firmware build), so
 * a change in here is only proved by an actual firmware build.
 */

#include "bsp.hpp"
#include "gpio.hpp"
#include "pmu.hpp"
#include "ota_file_updater.hpp"
#include "logger.hpp"
#include "config_store.hpp"
#include "service_scheduler.hpp"
#include "scheduler.hpp"
#include "service.hpp"
#include "dte_handler.hpp"
#include "filesystem.hpp"
#include "config_store.hpp"
#include "error.hpp"
#include "timer.hpp"
#include "debug.hpp"
#include "switch.hpp"
#include "reed.hpp"
#include "ledsm.hpp"
#include "buzzm.hpp"
#include "battery.hpp"
#include "rgb_led.hpp"
#include "rtc.hpp"

extern RTC *rtc;
#include "led.hpp"
#include "gps.hpp"
#include "gps_service.hpp"
#include "ble_service.hpp"
#include "sws_analog_service.hpp"
// CRC16-CCITT: nRF SDK header in firmware build, inline stub in CppUTest build.
// Mirrors the conditional in sws_analog_constants.hpp so tests can build this
// file (the SDK header is not on the test include path).
#ifndef CPPUTEST
#include "crc16.h"
#else
#include <cstdint>
static inline uint16_t crc16_compute(const uint8_t *data, uint16_t length, const uint16_t *) {
	uint16_t crc = 0xFFFF;
	for (uint16_t i = 0; i < length; i++) {
		crc ^= static_cast<uint16_t>(data[i]) << 8;
		for (uint8_t j = 0; j < 8; j++)
			crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
	}
	return crc;
}
#endif
#include "gentracker.hpp"

// USB DTE interface (platform-specific)
#ifdef USB_DTE_ENABLED
#include "usb_interface.hpp"
#endif
#ifdef BENCH_TEST
#include "bench_console.hpp"
#endif

// LoRa device instance (for bridge mode in process_usb_data)
#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
#include "lora_rak3172.hpp"
extern LoRaDevice *lora_device_instance;
#endif

// KIM2 device instance (for bridge mode in process_usb_data)
#if !(defined(LORA_RAK3172) && (LORA_RAK3172 == 1)) && !(defined(ARGOS_SMD) && (ARGOS_SMD == 1))
#include "kim2.hpp"
extern KIM2Device *kim2_device_instance;
#endif

// LED hardware access for forced LED off before powerdown
extern RGBLed *status_led;

// These contexts must be created before the FSM is initialised
extern FileSystem *main_filesystem;
extern Scheduler *system_scheduler;
extern Timer *system_timer;
extern ConfigurationStore *configuration_store;
extern BLEService *ble_service;
extern OTAFileUpdater *ota_updater;
extern DTEHandler *dte_handler;
extern ReedSwitch *reed_switch;
extern BatteryMonitor *battery_monitor;
extern BaseDebugMode g_debug_mode;


using led_handle = LEDState;
using buzz_handle = BuzzState;

/// @brief Config entry — start BLE advertising, USB polling, DTE handler.
void ConfigurationState::entry() {
	DEBUG_INFO("entry: ConfigurationState");

	// Hold VSYS at 3.3 V for the whole session: the scheduler is idle here, so
	// deep idle would otherwise drop the rail to 2.3 V under a live BLE/DTE
	// exchange -- and an OTA write, dispatched from interrupt context, would
	// program the IS25 below its rated minimum.
	GPIOPins::set_config_mode_active(true);

	// Flash the blue LED to indicate we have started BLE and we are
	// waiting for a connection
	led_handle::dispatch<SetLEDConfigNotConnected>({});
	buzz_handle::dispatch<SetBuzzConfiguration>({});

	m_backup_charge_mode = false;

	set_ble_device_name();
	ble_service->start([this](BLEServiceEvent &event) -> int { return on_ble_event(event); });
	restart_inactivity_timeout();

	if (gps_service) {
		gps_service->set_backup_charge_callbacks(
		    [this]() {
			    // Charge started: cut BLE after 200ms (lets the $O; response TX complete).
			    m_backup_charge_mode = true;
			    // MED #6 audit fix: store the task handle so `exit()` can cancel
			    // it if the FSM transits out during the 200 ms window. Without
			    // this, the lambda fires with a dangling `this` → HardFault.
			    m_backup_charge_stop_ble_task = system_scheduler->post_task_prio(
			        [this]() {
				        ble_service->stop();
				        system_scheduler->cancel_task(m_ble_inactivity_timeout_task);
				        led_handle::dispatch<SetLEDOff>({});
				        // Start the visual heartbeat: yellow flash every 10 s while charging.
				        m_backup_charge_blink_task = system_scheduler->post_task_prio(
				            std::bind(&ConfigurationState::backup_charge_blink_fire, this), "BackupChargeBlink",
				            Scheduler::DEFAULT_PRIORITY, 10000);
			        },
			        "BackupChargeStopBLE", Scheduler::DEFAULT_PRIORITY, 200);
		    },
		    [this]() {
			    // Charge ended (reed, timer, or abort): resume normal operation.
			    m_backup_charge_mode = false;
			    transit<OperationalState>();
		    });
	}

#ifdef USB_DTE_ENABLED
	// Default async writer → USB CDC. BLE overrides this on CONNECT
	// (see on_ble_event CONNECTED). Restored on BLE DISCONNECTED.
	// This lets bridges (GNSSBR/KIMBR/LORABR) started over USB route
	// raw UART RX back to the USB host via the DTEHandler async_write.
	dte_handler->set_async_write([](const std::string &msg) { UsbInterface::get_instance().write(msg); });

	// Start USB DTE polling (runs in parallel with BLE)
	DEBUG_TRACE("ConfigurationState: Starting DTE polling");
	schedule_usb_poll();
#endif
}

#ifdef USB_DTE_ENABLED
static void sync_bridge_log_silencing();  // Forward decl — defined near process_usb_data
#endif

void ConfigurationState::exit() {
	DEBUG_INFO("exit: ConfigurationState");

	// Released FIRST, so the rail is freed even if the shutdown below throws.
	GPIOPins::set_config_mode_active(false);
	// Shut the BLE down FIRST, and under protection. This call used to be the LAST
	// instruction of exit() and had no try/catch: if stop_bridge() or
	// power_off_immediate() threw, we never got there and the beacon went back to
	// operation with the BLE radio advertising permanently — on a sealed device,
	// an invisible current leak. OperationalState::exit() was already protecting
	// its own the same way.
	try {
		ble_service->stop();
	} catch (...) {
		DEBUG_ERROR("exit: ConfigurationState: ble_service->stop failed");
	}

	// Abandon any OTA still in flight, HERE rather than in the BLE DISCONNECTED
	// handler. That handler used to be the only place calling this, which was
	// fragile twice over: it needed a DISCONNECTED event to actually arrive
	// (leaving via the reed gesture with no phone attached never produced one),
	// and since stop() now deregisters the callback it would not run at all.
	// Without the abort, OTAFlashFileUpdater keeps m_file_size != 0 and an open
	// LittleFS file handle: the handle leaks and every later transfer is
	// refused until the next reboot. Cheap no-op when nothing is in progress
	// (m_file_size == 0); can throw on the MCU_FIRMWARE erase path, hence the
	// guard.
	// Only a transfer that is genuinely unfinished. A fully received image is
	// waiting for apply_file_update(), which for MCU_FIRMWARE is posted on the
	// scheduler and therefore has NOT run yet: aborting there would erase the
	// staged firmware header and lose a perfectly good update.
	try {
		if (ota_updater && ota_updater->is_transfer_incomplete()) {
			DEBUG_WARN("exit: ConfigurationState — aborting an unfinished OTA transfer");
			ota_updater->abort_file_transfer();
		}
	} catch (...) {
		DEBUG_ERROR("exit: ConfigurationState: abort_file_transfer failed");
	}

#ifdef USB_DTE_ENABLED
	// Drop the async writer back onto USB. It may still be holding the lambda
	// installed on BLE CONNECT, whose link is now gone — same reasoning as
	// above: the DISCONNECTED handler used to do this and no longer runs.
	dte_handler->set_async_write([](const std::string &msg) { UsbInterface::get_instance().write(msg); });
#endif

	system_scheduler->cancel_task(m_ble_inactivity_timeout_task);
	system_scheduler->cancel_task(m_backup_charge_blink_task);
	// MED #6 audit fix: cancel the BackupChargeStopBLE task too — its lambda
	// captures `this` and would HardFault if it fires after our destruction.
	system_scheduler->cancel_task(m_backup_charge_stop_ble_task);
	if (gps_service) gps_service->set_backup_charge_callbacks(nullptr, nullptr);
	m_backup_charge_mode = false;
#ifdef USB_DTE_ENABLED
	system_scheduler->cancel_task(m_usb_poll_task);
#endif
	// Ensure any active bridge is stopped before leaving configuration mode —
	// otherwise the bridge would prevent services from using the underlying UART.
	if (gps_device && gps_device->is_bridge_active()) gps_device->stop_bridge();
#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
	if (lora_device_instance && lora_device_instance->is_bridge_active()) lora_device_instance->stop_bridge();
#endif
#if !(defined(LORA_RAK3172) && (LORA_RAK3172 == 1)) && !(defined(ARGOS_SMD) && (ARGOS_SMD == 1))
	if (kim2_device_instance && kim2_device_instance->is_bridge_active()) kim2_device_instance->stop_bridge();
#endif
#ifdef USB_DTE_ENABLED
	// Restore USB console logs after stopping bridges — otherwise logs stay
	// suppressed until next process_usb_data tick (which is cancelled above).
	sync_bridge_log_silencing();
#endif
	// Safety net against a `PWRON GNSS`/`PWRON ALL` left dangling: the DTE command
	// powers the receiver up with no timer and only `PWRON OFF` decrements it.
	// Without this catch-up, the client counter never gets back to zero: every
	// later service session then does +1/-1 on top of the leaked count, the driver
	// stays in reception permanently (~25-30 mA) and the VSYS latch holds the
	// shared rail at 3.3 V — battery drained in a few days on a sealed device,
	// with NO way at all of noticing it. We only cut power if something is
	// really powered on, so as not to release VSENSORS without having taken it.
	if (gps_device && gps_device->is_powered()) {
		DEBUG_WARN("exit: ConfigurationState — GNSS still powered (PWRON not cancelled) | cutting the rail");
		gps_device->power_off_immediate();
	}
	led_handle::dispatch<SetLEDOff>({});
}

/// @brief BLE event callback — handle connect/disconnect, DTE data, OTA transfers.
/// @param event  BLE service event (CONNECTED, DISCONNECTED, DTE_DATA, OTA_*).
/// @return 0 on success.
int ConfigurationState::on_ble_event(BLEServiceEvent &event) {
	int rc = 0;

	switch (event.event_type) {
	case BLEServiceEventType::CONNECTED:
		DEBUG_TRACE("ConfigurationState::on_ble_event: CONNECTED");
		// Indicate DTE connection is made
		dte_handler->reset_state();
		dte_handler->set_async_write([](const std::string &msg) {
			if (ble_service) ble_service->write(msg);
		});
		led_handle::dispatch<SetLEDConfigConnected>({});
		restart_inactivity_timeout();
		break;
	case BLEServiceEventType::DISCONNECTED:
		DEBUG_TRACE("ConfigurationState::on_ble_event: DISCONNECTED");
		ota_updater->abort_file_transfer();
#ifdef USB_DTE_ENABLED
		// Restore USB writer so future DTE async responses / USB-started
		// bridges route to USB. Note: any bridge still active that captured
		// the (now stale) BLE writer will silently drop RX until stopped.
		dte_handler->set_async_write([](const std::string &msg) { UsbInterface::get_instance().write(msg); });
#endif
		// During backup charge the device is silent — keep LED off instead of
		// returning to the "waiting for BLE connection" blink.
		if (m_backup_charge_mode)
			led_handle::dispatch<SetLEDOff>({});
		else
			led_handle::dispatch<SetLEDConfigNotConnected>({});
		break;
	case BLEServiceEventType::DTE_DATA_RECEIVED:
		DEBUG_TRACE("ConfigurationState::on_ble_event: DTE_DATA_RECEIVED");
		restart_inactivity_timeout();
		system_scheduler->post_task_prio(std::bind(&ConfigurationState::process_received_data, this),
		                                 "BLEProcessReceivedData");
		break;
	case BLEServiceEventType::OTA_START:
		DEBUG_INFO("ConfigurationState::on_ble_event: OTA_START");
		restart_inactivity_timeout();
		ota_updater->start_file_transfer((OTAFileIdentifier)event.file_id, event.file_size, event.crc32);
		led_handle::dispatch<SetLEDDFUUpdate>({});
		break;
	case BLEServiceEventType::OTA_END:
		DEBUG_INFO("ConfigurationState::on_ble_event: OTA_END");
		restart_inactivity_timeout();
		ota_updater->complete_file_transfer();
		led_handle::dispatch<SetLEDConfigConnected>({});
		// Wrap in try/catch — apply_file_update can throw (CRC validation
		// failure, SPI/UART comms error during SMD DFU streaming, LFS read
		// error). Without the wrapper, a throw escapes the scheduler task
		// runner and surfaces at the main-loop catch, which dispatches an
		// ErrorEvent that transits to ErrorState → OffState — bricking a
		// sealed device on a recoverable OTA glitch. Log + swallow keeps
		// the device in Configuration state so the operator can retry.
		system_scheduler->post_task_prio(
		    []() {
			    try {
				    ota_updater->apply_file_update();
			    } catch (ErrorCode e) {
				    DEBUG_ERROR("ConfigurationState: apply_file_update ErrorCode=%d — staying in Config for retry",
					            (int)e);
			    } catch (const std::exception &ex) {
				    DEBUG_ERROR(
				        "ConfigurationState: apply_file_update std::exception: %s — staying in Config for retry",
				        ex.what());
			    } catch (...) {
				    DEBUG_ERROR(
				        "ConfigurationState: apply_file_update unknown exception — staying in Config for retry");
			    }
		    },
		    "BLEApplyOTAFileUpdate");
		break;
	case BLEServiceEventType::OTA_ABORT:
		DEBUG_INFO("ConfigurationState::on_ble_event: OTA_ABORT");
		restart_inactivity_timeout();
		ota_updater->abort_file_transfer();
		led_handle::dispatch<SetLEDConfigConnected>({});
		break;
	case BLEServiceEventType::OTA_FILE_DATA:
		//DEBUG_TRACE("ConfigurationState::on_ble_event: OTA_FILE_DATA");
		restart_inactivity_timeout();
		ota_updater->write_file_data(event.data, event.length);
		break;
	default: break;
	}

	return rc;
}


/// @brief BLE inactivity timeout after 20 min idle in config.
/// Sealed-device hardening (audit 2026-07): this used to transit<OffState>,
/// which on LinkIt means System OFF with the reed magnet as the only wake
/// source — an operator who configured a tag and sealed/released the animal
/// without the explicit exit gesture would get a device that silently powers
/// itself OFF 20 minutes later (a dead deployment at sea). Mirror the NORMAL
/// config-exit gesture (EXIT_CONFIG -> PreOperationalState -> Operational)
/// so the tag resumes its mission. A deliberate power-off stays available via
/// the reed LONG_HOLD (POWEROFF) gesture and the DTE command — both untouched.
void ConfigurationState::on_ble_inactivity_timeout() {
	// Defence in depth: transit<>() leaves whatever state the FSM is CURRENTLY
	// in, not the one that armed the timer. A stale BLE callback re-arming this
	// from OperationalState therefore yanked a running mission back through
	// PreOperationalState — restarting every service — for no reason. The
	// callback leak is fixed in BleInterface::stop(); this guard makes the
	// timer harmless even if anything else ever re-arms it out of context.
	if (!is_in_state<ConfigurationState>()) {
		DEBUG_WARN("BLE Inactivity Timeout fired outside configuration mode — ignored");
		return;
	}
	DEBUG_INFO("BLE Inactivity Timeout — returning to operation (as if exit gesture)");
	transit<PreOperationalState>();
}

/// @brief Visual heartbeat during DTE-triggered backup charge: brief YELLOW flash
/// (~200 ms) then back to off, re-scheduled every 10 s. Lets the operator see the
/// device is still alive in the otherwise-silent charging state. Cancelled on exit
/// or on m_backup_charge_mode = false.
void ConfigurationState::backup_charge_blink_fire() {
	if (!m_backup_charge_mode) return;  // charge ended between schedule and fire
	status_led->set(RGBLedColor::YELLOW);
	system_scheduler->post_task_prio(
	    [this]() {
		    if (!m_backup_charge_mode) return;
		    status_led->off();
		    m_backup_charge_blink_task =
		        system_scheduler->post_task_prio(std::bind(&ConfigurationState::backup_charge_blink_fire, this),
				                                 "BackupChargeBlink", Scheduler::DEFAULT_PRIORITY, 10000);
	    },
	    "BackupChargeBlinkOff", Scheduler::DEFAULT_PRIORITY, 200);
}

/// @brief Reset the BLE inactivity timeout (called on every BLE activity).
void ConfigurationState::restart_inactivity_timeout() {
	//DEBUG_TRACE("Restart BLE inactivity timeout: %lu", system_timer->get_counter());
	system_scheduler->cancel_task(m_ble_inactivity_timeout_task);
	m_ble_inactivity_timeout_task = system_scheduler->post_task_prio(
	    std::bind(&ConfigurationState::on_ble_inactivity_timeout, this), "BLEInactivityTimeout",
	    Scheduler::DEFAULT_PRIORITY, BLE_INACTIVITY_TIMEOUT_MS);
}

/// @brief Process BLE DTE command — parse, dispatch to DTEHandler, send response.
void ConfigurationState::process_received_data() {
	auto req = ble_service->read_line();

	if (req.size()) {
		// Bridge mode: forward raw BLE bytes straight to the UART, bypassing
		// the DTE parser. Mirrors the USB bridge path in process_usb_data().
		// BLE NUS RX is line-buffered (\r triggers flush), so AT-based modules
		// (KIM2/LoRa) work cleanly; GNSS binary UBX over BLE is limited by
		// this line buffering — host should prefer USB for u-center.
		// Exit sequence: user sends "+++\r" over BLE.
		auto bridge_forward = [&req, this](auto *device) -> bool {
			if (!device || !device->is_bridge_active()) return false;

			PMU::kick_watchdog();

			// BLE read_line() keeps the trailing \r. Strip before inspection.
			std::string line = req;
			while (!line.empty() && (line.back() == '\r' || line.back() == '\n'))
				line.pop_back();

			if (line == "+++") {
				device->stop_bridge();
				ble_service->write("\r\n[BRIDGE OFF]\r\n");
				return true;
			}

			// Re-append CRLF for AT framing (safe no-op for payloads that
			// already end in it, since we stripped above).
			std::string data = line + "\r\n";
			device->bridge_send(reinterpret_cast<const uint8_t *>(data.c_str()), data.size());
			return true;
		};

		if (bridge_forward(gps_device)) return;
#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
		if (bridge_forward(lora_device_instance)) return;
#endif
#if !(defined(LORA_RAK3172) && (LORA_RAK3172 == 1)) && !(defined(ARGOS_SMD) && (ARGOS_SMD == 1))
		if (bridge_forward(kim2_device_instance)) return;
#endif

		DEBUG_TRACE("received %u bytes:", req.size());
#if defined(DEBUG_ENABLE) && DEBUG_LEVEL >= 4
		printf("%s\n", req.c_str());
#endif

		std::string resp;
		DTEAction action;

		do {
			action = dte_handler->handle_dte_message(req, resp);

			// Kick watchdog every iteration (not just when resp produced)
			PMU::kick_watchdog();

			if (resp.size()) {
				DEBUG_TRACE("responding: %s", resp.c_str());
				if (!ble_service->write(resp)) {
					dte_handler->reset_state();
					break;
				}

				// Reset inactivity timeout whenever we send a response
				// This is important during a command sequences that can take
				// a long time to complete (eg DUMPD)
				restart_inactivity_timeout();
			}

			if (action == DTEAction::FACTR) {
				DEBUG_INFO("Perform factory reset of configuration store");
				configuration_store->factory_reset();

				// After formatting the filesystem we must do a system reset so that
				// the boot up procedure can repopulate files
				PMU::reset(false);
			} else if (action == DTEAction::RESET) {
				DEBUG_INFO("Perform device reset");

				// Execute this after 3 seconds to allow time for the BLE response to be sent
				system_scheduler->post_task_prio([]() { PMU::reset(false); }, "DTEActionPMUReset",
				                                 Scheduler::DEFAULT_PRIORITY, 3000);
			} else if (action == DTEAction::SECUR) {
				// TODO: add secure procedure
				DEBUG_INFO("Perform secure procedure");
			} else if (action == DTEAction::CONFIG_UPDATED) {
				// Propagate runtime LoRa param changes to the RAK3172 module.
				// reload_config_if_changed() is a no-op if no LORA_* param
				// actually changed (cheap diff against the cached LoRaConfig),
				// so it is safe to call after every PARMW. Busy states (TX /
				// joining) defer the apply to the next idle entry.
#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
				if (lora_device_instance) {
					lora_device_instance->reload_config_if_changed();
				}
#endif
			}

		} while (action == DTEAction::AGAIN);
	}
}

#ifdef USB_DTE_ENABLED
void ConfigurationState::schedule_usb_poll() {
	m_usb_poll_task = system_scheduler->post_task_prio(std::bind(&ConfigurationState::process_usb_data, this),
	                                                   "USBDTEPoll", Scheduler::DEFAULT_PRIORITY, USB_POLL_INTERVAL_MS);
}

/// @brief Suppress USB console logs while any bridge is active, restore when stopped.
/// Debug output would pollute the raw UART passthrough stream sent to the host
/// (u-center, AT terminal, etc.). Call this from any context where bridge state
/// may have changed (process_usb_data tick, FSM exit).
static void sync_bridge_log_silencing() {
	static Logger *s_saved_console_log = nullptr;
	bool bridge_active = (gps_device && gps_device->is_bridge_active());
#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
	bridge_active = bridge_active || (lora_device_instance && lora_device_instance->is_bridge_active());
#endif
#if !(defined(LORA_RAK3172) && (LORA_RAK3172 == 1)) && !(defined(ARGOS_SMD) && (ARGOS_SMD == 1))
	bridge_active = bridge_active || (kim2_device_instance && kim2_device_instance->is_bridge_active());
#endif
	if (bridge_active && DebugLogger::console_log != nullptr) {
		s_saved_console_log = DebugLogger::console_log;
		DebugLogger::console_log = nullptr;
	} else if (!bridge_active && s_saved_console_log != nullptr) {
		DebugLogger::console_log = s_saved_console_log;
		s_saved_console_log = nullptr;
	}
}

void ConfigurationState::process_usb_data() {
	auto &usb = UsbInterface::get_instance();

	sync_bridge_log_silencing();

	// GNSS UART bridge mode: forward USB ↔ GNSS UART directly (binary UBX)
	if (gps_device && gps_device->is_bridge_active()) {
		// Process UART RX → USB (via passthrough callback)
		gps_device->bridge_process_rx();

		// Process USB → UART (raw bytes, no line parsing — UBX is binary)
		if (usb.has_data()) {
			char buf[256];
			int n = NrfUSB::read(buf, sizeof(buf));
			if (n > 0) {
				restart_inactivity_timeout();
				PMU::kick_watchdog();

				// Check for exit sequence "+++" (text typed in terminal)
				if (n == 3 && buf[0] == '+' && buf[1] == '+' && buf[2] == '+') {
					gps_device->stop_bridge();
					usb.write("\r\n[BRIDGE OFF]\r\n");
					schedule_usb_poll();
					return;
				}
				// Also check with line ending
				if (n >= 3 && buf[0] == '+' && buf[1] == '+' && buf[2] == '+'
				    && (n == 3 || buf[3] == '\r' || buf[3] == '\n')) {
					gps_device->stop_bridge();
					usb.write("\r\n[BRIDGE OFF]\r\n");
					schedule_usb_poll();
					return;
				}

				gps_device->bridge_send(reinterpret_cast<const uint8_t *>(buf), n);
			}
		}

		schedule_usb_poll();
		return;
	}

#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
	// LoRa UART bridge mode: forward USB ↔ RAK3172 UART directly
	if (lora_device_instance && lora_device_instance->is_bridge_active()) {
		// Process UART RX → USB (via passthrough callback)
		lora_device_instance->bridge_process_rx();

		// Process USB → UART
		if (usb.has_data()) {
			auto line = usb.read_line();
			if (line.size()) {
				restart_inactivity_timeout();
				PMU::kick_watchdog();

				// Check for exit sequence "+++"
				if (line == "+++") {
					lora_device_instance->stop_bridge();
					usb.write("\r\n[BRIDGE OFF]\r\n");
					schedule_usb_poll();
					return;
				}

				// Forward to RAK3172 UART with \r\n termination
				std::string data = line + "\r\n";
				lora_device_instance->bridge_send(reinterpret_cast<const uint8_t *>(data.c_str()), data.size());
			}
		}

		schedule_usb_poll();
		return;
	}
#endif

#if !(defined(LORA_RAK3172) && (LORA_RAK3172 == 1)) && !(defined(ARGOS_SMD) && (ARGOS_SMD == 1))
	// KIM2 UART bridge mode: forward USB ↔ KIM2 UART directly (AT commands)
	if (kim2_device_instance && kim2_device_instance->is_bridge_active()) {
		// Process UART RX → USB (via passthrough callback)
		kim2_device_instance->bridge_process_rx();

		// Process USB → UART
		if (usb.has_data()) {
			auto line = usb.read_line();
			if (line.size()) {
				restart_inactivity_timeout();
				PMU::kick_watchdog();

				// Check for exit sequence "+++"
				if (line == "+++") {
					kim2_device_instance->stop_bridge();
					usb.write("\r\n[BRIDGE OFF]\r\n");
					schedule_usb_poll();
					return;
				}

				// Forward to KIM2 UART with \r\n termination (AT command framing)
				std::string data = line + "\r\n";
				kim2_device_instance->bridge_send(reinterpret_cast<const uint8_t *>(data.c_str()), data.size());
			}
		}

		schedule_usb_poll();
		return;
	}
#endif

	// Check if USB has data
	if (usb.has_data()) {
		auto req = usb.read_line();

		if (req.size()) {
#ifdef BENCH_TEST
			// Bench '%' commands share the CDC stream with DTE '$...#'. Route
			// them to the bench console (e.g. %OP to leave config, %STATE) so
			// they work while in ConfigurationState too. Never fed to the DTE
			// parser.
			if (req[0] == '%') {
				bench::handle_line(req);
				schedule_usb_poll();
				return;
			}
#endif
			// DTE protocol expects trailing \r which read_line() strips
			req += '\r';
			// Suppress console debug logs during DTE exchange to avoid
			// polluting the USB DTE response stream
			auto *saved_log = DebugLogger::console_log;
			DebugLogger::console_log = nullptr;

			std::string resp;
			DTEAction action;

			do {
				action = dte_handler->handle_dte_message(req, resp);

				// Kick watchdog every iteration (not just when resp produced)
				PMU::kick_watchdog();

				if (resp.size()) {
					usb.write(resp);

					// Reset inactivity timeout on USB activity too
					restart_inactivity_timeout();
				}

				if (action == DTEAction::FACTR) {
					DEBUG_INFO("Perform factory reset of configuration store (USB)");
					configuration_store->factory_reset();
					PMU::reset(false);
				} else if (action == DTEAction::RESET) {
					DEBUG_INFO("Perform device reset (USB)");
					system_scheduler->post_task_prio([]() { PMU::reset(false); }, "DTEActionPMUReset",
					                                 Scheduler::DEFAULT_PRIORITY, 3000);
				} else if (action == DTEAction::SECUR) {
					DEBUG_INFO("Perform secure procedure (USB)");
				}

			} while (action == DTEAction::AGAIN);

			// Restore console logging
			DebugLogger::console_log = saved_log;
		}
	}

	// Schedule next poll
	schedule_usb_poll();
}
#else
// Empty implementations when USB DTE is disabled
void ConfigurationState::schedule_usb_poll() {}
void ConfigurationState::process_usb_data() {}
#endif
