#pragma once

#include "debug.hpp"
#include "logger.hpp"
#include "gps.hpp"
#include "timeutils.hpp"

class MockM10Q : public GPSDevice {
public:
    void notify_gnss_data(std::time_t time, double lat=0, double lon=0, double hdop = 0, double hacc = 0, bool valid = 1) {
    	// Value-initialised, like notify_degraded_gnss_data below. Without the
    	// braces every field this helper does not set (gSpeed, numSV, velN/E/D,
    	// ttff, fixType, flags...) is stack garbage, which stayed harmless only
    	// as long as no test read one of them.
    	GNSSData gnss_data{};
    	gnss_data.lat = lat;
    	gnss_data.lon = lon;
    	gnss_data.valid = valid;
    	gnss_data.hDOP = hdop;
    	gnss_data.hAcc = hacc;
    	convert_datetime_to_epoch(time, gnss_data.year, gnss_data.month, gnss_data.day, gnss_data.hour, gnss_data.min, gnss_data.sec);
    	notify(GPSEventPVT(gnss_data));
    }

    void notify_degraded_gnss_data(std::time_t time, double lat=0, double lon=0,
    		double hdop = 0, double hacc = 0, uint8_t fixType = 2, uint8_t numSV = 3) {
    	GNSSData gnss_data{};
    	gnss_data.lat = lat;
    	gnss_data.lon = lon;
    	gnss_data.valid = 1;
    	gnss_data.hDOP = hdop;
    	gnss_data.hAcc = hacc;
    	gnss_data.fixType = fixType;
    	gnss_data.numSV = numSV;
    	convert_datetime_to_epoch(time, gnss_data.year, gnss_data.month, gnss_data.day, gnss_data.hour, gnss_data.min, gnss_data.sec);
    	notify(GPSEventPVTDegraded(gnss_data));
    }

    void notify_max_nav_samples() {
    	notify<GPSEventMaxNavSamples>({});
    }

    void notify_max_sat_samples() {
    	notify<GPSEventMaxSatSamples>({});
    }

    void notify_sat_report(unsigned int qual=3, unsigned int nSv=1) {
    	notify(GPSEventSatReport(nSv, qual));
    }

    void notify_error() {
    	notify<GPSEventError>({});
    }

    void notify_power_off(bool fix_found ) {
        notify(GPSEventPowerOff(fix_found));
    }

    void power_on(const GPSNavSettings& nav_settings) override {
    	DEBUG_TRACE("MockM10Q::power_on()");
        mock().actualCall("power_on").onObject(this).withParameterOfType("const GPSNavSettings&", "nav_settings", &nav_settings);
    }

    void power_off() override {
    	DEBUG_TRACE("MockM10Q::power_off()");
        mock().actualCall("power_off").onObject(this);
    }
};
