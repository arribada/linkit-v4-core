/**
 * @file argos_rx_service.hpp
 * @brief Argos RX service — schedules downlink RX windows for AOP updates via PREVIPASS.
 */

#pragma once

#include "kineis_device.hpp"
#include "service.hpp"

/// @brief Argos RX scheduler — computes next downlink RX window using PREVIPASS.
class ArgosRxScheduler {
private:
	struct Location {
		double longitude;
		double latitude;
		Location(double x, double y) : longitude(x), latitude(y) {}
	};
	std::time_t m_earliest_schedule = 0;
	std::optional<Location> m_location;

	static constexpr unsigned int SECONDS_PER_MINUTE    = 60;
	static constexpr unsigned int MINUTES_PER_HOUR      = 60;
	static constexpr unsigned int HOURS_PER_DAY         = 24;
	static constexpr unsigned int MSECS_PER_SECOND      = 1000;
	static constexpr unsigned int SECONDS_PER_HOUR      = MINUTES_PER_HOUR * SECONDS_PER_MINUTE;
	static constexpr unsigned int SECONDS_PER_DAY       = HOURS_PER_DAY * SECONDS_PER_HOUR;
	static constexpr unsigned int ARGOS_RX_MARGIN_MSECS = 0;

public:
	static constexpr unsigned int INVALID_SCHEDULE = static_cast<unsigned int>(-1);

	ArgosRxScheduler();

	/// @brief Find next downlink RX window using PREVIPASS.
	/// @param config          Argos configuration (prepass params, AOP update period).
	/// @param pass_predict    AOP satellite database.
	/// @param now             Current RTC time.
	/// @param[out] timeout    RX window duration in ms.
	/// @param[out] scheduled_mode  Modulation for the RX window.
	/// @return Delay in ms until RX start, or SCHEDULE_DISABLED if no window found.
	unsigned int schedule(ArgosConfig& config, BasePassPredict& pass_predict,
	                      std::time_t now, unsigned int &timeout, KineisModulation& scheduled_mode);

	/// @brief Set earliest allowed RX time.
	/// @param t  Earliest epoch time (seconds). Only advances forward (max of current and t).
	void set_earliest_schedule(std::time_t t);

	/// @brief Update last known GPS position for PREVIPASS computation.
	/// @param lon  Longitude in degrees.
	/// @param lat  Latitude in degrees.
	void set_location(double lon, double lat);
};

/// @brief Argos downlink RX service — receives AOP data to update pass prediction database.
class ArgosRxService : public Service, KineisEventListener {
public:
	ArgosRxService(KineisDevice& device);

	/// @brief Handle peer events (GPS fix → update location, UW → set earliest schedule).
	/// @param e  Peer service event.
	void notify_peer_event(ServiceEvent& e);

protected:
	void service_init() override;
	void service_term() override;
	bool service_is_enabled() override;
	unsigned int service_next_schedule_in_ms() override;
	void service_initiate() override;
	bool service_cancel() override;
	unsigned int service_next_timeout() override;
	bool service_is_triggered_on_surfaced(bool&) override;

private:
	KineisDevice& m_kineis;
	ArgosRxScheduler m_sched;
	unsigned int m_timeout = 0;
	KineisModulation m_mode = KineisModulation::LDK;
	std::map<uint8_t, AopSatelliteEntry_t> m_orbit_params_map;
	std::map<uint8_t, AopSatelliteEntry_t> m_constellation_status_map;
	unsigned int m_cumulative_rx_time = 0;

	void react(KineisEventRxPacket const&) override;
	void react(KineisEventDeviceError const&) override;
	void react(KineisEventPowerOff const&) override;
	void react(KineisEventRxStopped const&) override;

	/// @brief Merge new AOP records into existing pass predict database and persist.
	/// @param new_pass_predict  Decoded AOP records from downlink packet.
	void update_pass_predict(BasePassPredict& new_pass_predict);
};
