/**
 * @file gps.hpp
 * @brief Abstract GNSS device interface — PVT, sat tracking, almanac, bridge mode.
 */

#pragma once

#include <map>
#include <cstdint>
#include <cstring>
#include <functional>
#include "base_types.hpp"
#include "events.hpp"

struct GPSNavSettings {
	BaseGNSSFixMode fix_mode;
	BaseGNSSDynModel dyn_model;
	bool assistnow_autonomous_enable;
	bool assistnow_offline_enable;
	bool hdop_filter_en;
	unsigned int hdop_filter_threshold;
	bool hacc_filter_en;
	unsigned int hacc_filter_threshold;
	bool debug_enable = false;
	bool sat_tracking = false;
	unsigned int num_consecutive_fixes = 1;
	unsigned int max_nav_samples = 0;
	unsigned int max_sat_samples = 0;
	unsigned int constellation_mask = 0x0F;  // bit0=GPS, bit1=GAL, bit2=GLO, bit3=BDS, bit4=QZSS, bit5=SBAS
	unsigned int orbmaxerr = 300;
	unsigned int min_cno = 10;
	unsigned int min_elev = 10;
	unsigned int ano_stale_threshold_s = 25 * 24 * 3600;  // ANO staleness threshold in seconds (default: 25 days)
	bool cloudlocate_enable = false;                      // Enable MEAS message capture for CloudLocate
	bool cold_start =
	    false;  // Force a true cold start: CFG-RST navBbrMask=0xFFFF (wipe ephemeris/almanac/pos/time/clock + aiding) at power-on, then re-inject time/pos + ANO. Default false = hot start (keep BBR).
};

struct GNSSData {
	uint32_t iTOW;
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	uint8_t valid;
	uint32_t tAcc;
	int32_t nano;
	uint8_t fixType;
	uint8_t flags;
	uint8_t flags2;
	uint8_t flags3;
	uint8_t numSV;
	double lon;      // Degrees
	double lat;      // Degrees
	int32_t height;  // mm
	int32_t hMSL;    // mm
	uint32_t hAcc;   // mm
	uint32_t vAcc;   // mm
	int32_t velN;    // mm
	int32_t velE;    // mm
	int32_t velD;    // mm
	int32_t gSpeed;  // mm/s
	float headMot;   // Degrees
	uint32_t sAcc;   // mm/s
	float headAcc;   // Degrees
	float pDOP;
	float vDOP;
	float hDOP;
	float headVeh;  // Degrees
	uint32_t ttff;  // ms
};

struct GNSSDeviceInfo {
	char swVersion[30];
	char hwVersion[10];
	uint8_t uniqueId[5];
	bool valid;
};

struct GNSSAlmanacStatus {
	bool file_present;
	unsigned int file_size;
	unsigned int total_records;
	unsigned int valid_records;
	bool stale;
};

struct GNSSRawMeasurement {
	uint8_t measc12[12];
	uint8_t meas20[20];
	uint8_t meas50[50];
	bool has_measc12;
	bool has_meas20;
	bool has_meas50;
	uint32_t
	    capture_time;  // RTC epoch (s) at snapshot capture; 0 = unknown. Embedded in the CloudLocate packet so the cloud knows the measurement instant regardless of TX delay.
	GNSSRawMeasurement() : has_measc12(false), has_meas20(false), has_meas50(false), capture_time(0) {
		std::memset(measc12, 0, sizeof(measc12));
		std::memset(meas20, 0, sizeof(meas20));
		std::memset(meas50, 0, sizeof(meas50));
	}
};

struct GPSEventMaxNavSamples {};
struct GPSEventMaxSatSamples {};

struct GPSEventSatReport {
	unsigned int numSvs;
	unsigned int bestSignalQuality;
	GPSEventSatReport(unsigned int a, unsigned int b) : numSvs(a), bestSignalQuality(b) {}
};

struct GPSEventError {};
struct GPSEventPowerOn {};
struct GPSEventDeviceInfoReady {};
struct GPSEventPowerOff {
	bool fix_found;
	GPSEventPowerOff(bool a) : fix_found(a) {}
};
struct GPSEventPVT {
	GNSSData &data;
	GPSEventPVT(GNSSData &a) : data(a) {}
};
struct GPSEventPVTDegraded {
	GNSSData &data;
	GPSEventPVTDegraded(GNSSData &a) : data(a) {}
};
struct GPSEventRawMeasurement {
	GNSSRawMeasurement &data;
	GPSEventRawMeasurement(GNSSRawMeasurement &a) : data(a) {}
};

/// @brief Emitted once per acquisition the FIRST time a raw CloudLocate
/// measurement becomes available. Unlike GPSEventRawMeasurement (which is
/// terminal and triggers GPS power-off at timeout), this event is just a
/// signal — GPS continues running, looking for a full PVT fix. Subscribers
/// can use the embedded snapshot to fire an early CloudLocate transmission
/// (e.g. LoRaTxService 1st surface ping).
struct GPSEventCloudLocateReady {
	GNSSRawMeasurement data;
	GPSEventCloudLocateReady(const GNSSRawMeasurement &a) : data(a) {}
};

class GPSEventListener {
public:
	virtual ~GPSEventListener() {}
	virtual void react(const GPSEventPowerOn &) {}
	virtual void react(const GPSEventPowerOff &) {}
	virtual void react(const GPSEventError &) {}
	virtual void react(const GPSEventPVT &) {}
	virtual void react(const GPSEventSatReport &) {}
	virtual void react(const GPSEventMaxNavSamples &) {}
	virtual void react(const GPSEventPVTDegraded &) {}
	virtual void react(const GPSEventRawMeasurement &) {}
	virtual void react(const GPSEventCloudLocateReady &) {}
	virtual void react(const GPSEventMaxSatSamples &) {}
	virtual void react(const GPSEventDeviceInfoReady &) {}
};

/// @brief Abstract GNSS device — implemented by M10QAsync (u-blox M10).
class GPSDevice : public EventEmitter<GPSEventListener> {
public:
	using PassthroughCallback = std::function<void(const uint8_t *, size_t)>;

	virtual ~GPSDevice() {}
	// These methods are specific to the chipset and should be implemented by device-specific subclass
	virtual void power_off() = 0;
	virtual void power_on(const GPSNavSettings &nav_settings) = 0;

	/// 2026-05-25 Fix #10 — Hard shutdown, no graceful teardown.
	/// Cancels all pending FSM tasks and cuts the rail immediately. Used by
	/// service_term() on the OffState / ConfigurationState / Error /
	/// BatteryCritical paths where:
	///  - Graceful power_off() can hang for >15 min if the receiver is mid-
	///    boot (UART transactions pending, FSM in poweron/configure with no
	///    response yet). The hang bricks the device from user perspective
	///    until WDT fires.
	///  - V_BCKP / BBR preservation does NOT matter because the device is
	///    about to soft-reset for System OFF (LinkIt) or cold-boot anyway.
	///    Next session does a cold-start GNSS acquisition — acceptable cost.
	/// Default impl falls back to power_off() so non-M10 backends keep
	/// compiling without behavioral change.
	virtual void power_off_immediate() { power_off(); }

	// Backup-cell charge mode: rail powered, receiver in deep sleep (UBX-RXM-PMREQ backup).
	// Default no-op so non-M10 backends keep compiling. Caller is responsible for scheduling exit.
	//
	// 2026-05 deep-idle refactor: this same mechanism is the canonical "deep-idle"
	// state. Aliased below via `enter_deep_idle()` / `exit_deep_idle()` for the
	// GPSService deep-idle dispatch path. The DTE GNSSBCKP_REQ command still uses
	// the original names. Both call paths converge on the same M10Q state machine.
	virtual bool enter_backup_charge_mode() { return false; }
	virtual void exit_backup_charge_mode() {}
	/// @brief Alias for `enter_backup_charge_mode()` — see deep-idle plan §2.
	virtual bool enter_deep_idle() { return enter_backup_charge_mode(); }
	/// @brief Alias for `exit_backup_charge_mode()` — see deep-idle plan §2.
	virtual void exit_deep_idle() { exit_backup_charge_mode(); }
	/// @brief Is the M10Q currently in deep-idle (PMREQ-backup with rail on)?
	/// Used by GPSService::service_next_schedule_in_ms for the R5 hygiene check
	/// (force rail-cycle if device has been in deep-idle > 24 h).
	virtual bool is_in_deep_idle() const { return false; }
	/// @brief Arm deep-idle as the next stop disposition (CRITICAL #1 audit fix).
	/// When set, the next `power_off()` chain routes to `enterbackup` instead of
	/// cutting the rail. Default no-op for backends without deep-idle support.
	/// Must be called BEFORE `power_off()` so the intent is consumed during the
	/// power-down chain. Single-shot: flag is cleared after consumption.
	virtual void request_deep_idle_on_next_stop() {}
	/// @brief Force-exit deep-idle by cutting the rail (HIGH GNSS-AUDIT #2 follow-up).
	/// Distinct from `power_off()`: when the device is in PMREQ-backup with
	/// users==0, `power_off()` is a no-op (nothing to decrement). This method
	/// unconditionally tears down the rail from the backup state. Used by the
	/// GNP52 auto-off timer to actually terminate deep-idle after the configured
	/// duration. No-op if the device is not currently in deep-idle.
	virtual void poweroff_from_deep_idle() {}
	virtual GNSSDeviceInfo get_device_info() const { return {}; }
	virtual GNSSAlmanacStatus get_almanac_status(unsigned int ano_stale_threshold_s = 25 * 24 * 3600) const {
		(void)ano_stale_threshold_s;
		return {};
	}

	// Fastloc: query best degraded PVT accumulated during current acquisition
	virtual bool has_degraded_pvt() const { return false; }
	virtual GNSSData get_degraded_pvt() const { return {}; }

	// CloudLocate: query latest raw GNSS measurement snapshot
	virtual bool has_raw_measurement() const { return false; }
	virtual GNSSRawMeasurement get_raw_measurement() const { return {}; }

	// Bridge/passthrough mode (default: not supported)
	virtual bool start_bridge(PassthroughCallback) { return false; }
	virtual void stop_bridge() {}
	virtual bool is_bridge_active() const { return false; }

	/// @brief Is the receiver powered / the state machine out of idle?
	///
	/// Used to recover from a DTE `PWRON GNSS` left dangling: that command
	/// increments the client count with no timer at all, and only a `PWRON OFF`
	/// decrements it. Forgotten, it never returns to zero -- every service session
	/// then does +1/-1 ABOVE the leaked count and nothing cuts the rail any more
	/// (~25-30 mA continuously on a sealed device).
	virtual bool is_powered() const { return false; }
	virtual bool bridge_send(const uint8_t *, size_t) { return false; }
	virtual void bridge_process_rx() {}
};
