/**
 * @file ble_service.hpp
 * @brief Abstract BLE service interface — DTE data, OTA events, connection lifecycle.
 */

#pragma once

#include <functional>
#include <string>

enum class BLEServiceEventType {
	CONNECTED,
	DISCONNECTED,
	DTE_DATA_RECEIVED,
	OTA_START,
	OTA_FILE_DATA,
	OTA_ABORT,
	OTA_END
};

struct BLEServiceEvent {
	BLEServiceEventType event_type;
	union {
		struct {
			unsigned int file_id;
			unsigned int file_size;
			unsigned int crc32;
		};
		struct {
			void *data;
			unsigned int length;
		};
	};
};

class BLEService {
public:
	virtual ~BLEService() {}
	virtual void init() {}
	virtual void start(std::function<int(BLEServiceEvent &event)> on_event) = 0;
	virtual void stop() = 0;
	virtual bool write(std::string str) = 0;
	virtual bool write_best_effort(std::string str) { return write(str); }
	virtual std::string read_line() = 0;
	virtual void set_device_name(const std::string &) = 0;
#ifdef BENCH_TEST
	/// @brief Bench only: is advertising still wanted/active?
	/// Lets you VERIFY on the board that after leaving configuration mode
	/// the radio really does stay off, instead of inferring it from the code.
	virtual bool bench_is_advertising() { return false; }
	/// @brief Bench: advertising mode as seen by the SDK module (0 = IDLE).
	/// WARNING: sd_ble_gap_adv_stop() does NOT reset this field to IDLE — it stays
	/// stuck on FAST after a stop. So on its own it proves nothing.
	virtual int bench_adv_mode() { return -1; }
	/// @brief Bench: AUTHORITATIVE probe of the SoftDevice.
	/// Returns the return code of sd_ble_gap_adv_stop(): 0 = we were REALLY
	/// advertising (and we have just stopped it), NRF_ERROR_INVALID_STATE (8) =
	/// we were not advertising. Accepted side effect: it stops advertising.
	virtual unsigned int bench_probe_advertising() { return 0xFFFFFFFF; }
	/// @brief Bench only: injects a synthetic BLE disconnection event
	/// to replay, with no phone, the sequence that left advertising switched back on.
	virtual void bench_inject_disconnect() {}
#endif
};
