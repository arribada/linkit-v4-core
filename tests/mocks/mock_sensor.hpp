#pragma once

#include "sensor.hpp"

#include "CppUTestExt/MockSupport.h"

class MockSensor : public Sensor {
public:
	MockSensor(const char *name = "Mock") : Sensor(name) {}

	void calibration_write(const double value, const unsigned int offset) override {
		mock()
		    .actualCall("calibration_write")
		    .onObject(this)
		    .withParameter("value", value)
		    .withParameter("offset", offset);
	}

	void calibration_read(double &value, const unsigned int offset) override {
		value =
		    mock().actualCall("calibration_read").onObject(this).withParameter("offset", offset).returnDoubleValue();
	}

	double read(unsigned int port) override {
		return mock().actualCall("read").onObject(this).withParameter("port", port).returnDoubleValue();
	}
};
