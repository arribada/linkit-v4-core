/**
 * @file cam_service.hpp
 * @brief Camera service — periodic on/off cycling, capture count logging.
 */

#pragma once

#include "cam.hpp"
#include "service.hpp"
#include "logger.hpp"
#include "timeutils.hpp"
#include "scheduler.hpp"

/// @brief CSV log formatter for camera entries (used by DUMPD command).
class CAMLogFormatter : public LogFormatter {
public:
	const std::string header() override { return "log_datetime,id,batt_voltage,state\r\n"; }
	const std::string log_entry(const LogEntry &e) override {
		char entry[512], d1[128];
		const auto *cam = reinterpret_cast<const CAMLogEntry *>(&e);
		std::time_t t;
		std::tm *tm;

		t = convert_epochtime(cam->header.year, cam->header.month, cam->header.day, cam->header.hours,
		                      cam->header.minutes, cam->header.seconds);
		tm = std::gmtime(&t);
		std::strftime(d1, sizeof(d1), "%d/%m/%Y %H:%M:%S", tm);

		// Convert to CSV
		snprintf(entry, sizeof(entry), "%s,%d,%f,%d\r\n", d1, (unsigned int)cam->info.counter,
		         (double)cam->info.batt_voltage / 1000, (unsigned int)cam->info.event_type);
		return std::string(entry);
	};
};

/// @brief Camera service — periodic power on/off cycling with capture logging.
///
/// The device's power sequences are asynchronous (see CAMDevice): a scheduled
/// session starts one in service_initiate() and completes in react() when the
/// device reports the new state. The camera is usable underwater, so the
/// framework never deschedules it on a dive, and never tells it about a
/// resurfacing either — the CAM_TRIGGER_ON_SURFACED edge is picked up in
/// notify_peer_event() instead.
class CAMService : public Service, public CAMEventListener {
public:
	CAMService(CAMDevice &device, Logger *logger)
	    : Service(ServiceIdentifier::CAM_SENSOR, "CAM", logger),
	      m_device(device) {
		m_device.subscribe(*this);
	}
	void notify_peer_event(ServiceEvent &event) override;

protected:
	// Service interface methods
	void service_init() override;
	void service_term() override;
	bool service_is_enabled() override;
	unsigned int service_next_schedule_in_ms() override;
	void service_initiate() override;
	bool service_cancel() override;
	unsigned int service_next_timeout() override;
	bool service_is_usable_underwater() override;
	bool service_is_triggered_on_event(ServiceEvent &, bool &) override;

private:
	/// @brief Delay before retrying a schedule after the completion path threw.
	/// Same value as the framework's own Service::EXCEPTION_RETRY_MS.
	static constexpr unsigned int RECOVERY_RETRY_MS = 5000;
	/// @brief Safety-net timeout on a session (see service_next_timeout).
	static constexpr unsigned int SEQUENCE_TIMEOUT_MS = 30000;

	CAMDevice &m_device;
	std::time_t m_next_schedule = 0;  ///< UTC instant of the toggle last armed (logged as schedTime)
	bool m_is_active = false;
	bool m_is_pwr_on = false;
	Scheduler::TaskHandle m_task_recovery;

	void react(const CAMEventError &) override;
	void react(const CAMEventPowerOn &) override;
	void react(const CAMEventPowerOff &) override;

	void task_process_cam_data(bool state);
	void complete_with_log(bool state);
	void populate_cam_log_with_time(CAMLogEntry &entry, std::time_t time);
	CAMLogEntry invalid_log_entry();
};
