#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

#include <iostream>

#include "axl_sensor_service.hpp"
#include "mock_sensor.hpp"
#include "fake_config_store.hpp"
#include "fake_logger.hpp"
#include "fake_rtc.hpp"
#include "fake_timer.hpp"
#include "scheduler.hpp"


extern Timer *system_timer;
extern ConfigurationStore *configuration_store;
extern Scheduler *system_scheduler;
extern RTC *rtc;


TEST_GROUP(AXLSensor) {
	FakeConfigurationStore *fake_config_store;
	FakeTimer *fake_timer;
	FakeLog *fake_logger;
	Logger *logger;
	FakeRTC *fake_rtc;

	void setup() {
		fake_timer = new FakeTimer;
		system_timer = fake_timer;
		fake_rtc = new FakeRTC;
		rtc = fake_rtc;
		fake_config_store = new FakeConfigurationStore;
		configuration_store = fake_config_store;
		fake_logger = new FakeLog;
		logger = fake_logger;
		fake_logger->create();
		system_scheduler = new Scheduler(system_timer);
		configuration_store->init();
	}

	void teardown() {
		mock().clear();
		delete system_scheduler;
		delete fake_timer;
		delete fake_config_store;
		delete fake_logger;
		delete fake_rtc;
		// Reset globals so the next test group cannot dereference dangling
		// pointers (DTEHandler segfaulted on the still-cached pointers).
		system_scheduler = nullptr;
		system_timer = nullptr;
		rtc = nullptr;
		configuration_store = nullptr;
	}

	void expect_calibration_writes(int count = 5) {
		// The AXL service writes calibration values at start - ignore these
		// 5 basic calls: g_force, power_mode, x, y, z calibration
		// +2 for wakeup threshold and duration if configured
		mock().expectNCalls(count, "calibration_write").ignoreOtherParameters();
	}
};


TEST(AXLSensor, SensorDisabled) {
	MockSensor drv;
	AXLSensorService s(drv, logger);
	unsigned int num_callbacks = 0;

	system_timer->start();

	unsigned int period = 10;
	bool sensor_en = false;

	configuration_store->write_param(ParamID::AXL_SENSOR_ENABLE, sensor_en);
	configuration_store->write_param(ParamID::AXL_SENSOR_PERIODIC, period);

	expect_calibration_writes();
	s.start([&num_callbacks](ServiceEvent &event) {
		if (event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
			num_callbacks++;
		}
	});

	// Sampling should happen every 10
	for (unsigned int i = 0; i < 5; i++) {
		fake_timer->increment_counter(period * 1000);
		system_scheduler->run();
	}

	CHECK_EQUAL(0, num_callbacks);
	CHECK_EQUAL(0, logger->num_entries());

	s.stop();
}

TEST(AXLSensor, SchedulingPeriodic) {
	MockSensor drv;
	AXLSensorService s(drv, logger);
	unsigned int num_callbacks = 0;

	system_timer->start();

	unsigned int period = 10;
	bool sensor_en = true;

	configuration_store->write_param(ParamID::AXL_SENSOR_ENABLE, sensor_en);
	configuration_store->write_param(ParamID::AXL_SENSOR_PERIODIC, period);

	expect_calibration_writes();
	s.start([&num_callbacks](ServiceEvent &event) {
		if (event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
			num_callbacks++;
		}
	});

	// Sampling should happen every 10
	for (unsigned int i = 0; i < 5; i++) {
		mock().expectOneCall("read").onObject(&drv).withUnsignedIntParameter("port", 1).andReturnValue((double)i);
		mock().expectOneCall("read").onObject(&drv).withUnsignedIntParameter("port", 2).andReturnValue((double)i + 1);
		mock().expectOneCall("read").onObject(&drv).withUnsignedIntParameter("port", 3).andReturnValue((double)i + 2);
		mock().expectOneCall("read").onObject(&drv).withUnsignedIntParameter("port", 0).andReturnValue((double)i + 3);
		mock().expectOneCall("read").onObject(&drv).withUnsignedIntParameter("port", 4).andReturnValue(
		    (double)(i ? 1 : 0));
		mock().expectOneCall("read").onObject(&drv).withUnsignedIntParameter("port", 5).andReturnValue(
		    (double)(i ? 1 : 0));
		fake_timer->increment_counter(period * 1000);
		system_scheduler->run();
	}

	CHECK_EQUAL(5, num_callbacks);
	CHECK_EQUAL(5, logger->num_entries());

	// Validate log entries
	for (unsigned int i = 0; i < 5; i++) {
		AXLLogEntry e;
		logger->read(&e, i);
		CHECK_EQUAL((double)i, e.x);
		CHECK_EQUAL((double)i + 1, e.y);
		CHECK_EQUAL((double)i + 2, e.z);
		CHECK_EQUAL((double)i + 3, e.temperature);
		CHECK_EQUAL((bool)i & 1, e.wakeup_triggered);
	}

	s.stop();
}


TEST(AXLSensor, SchedulingNoPeriodic) {
	MockSensor drv;
	AXLSensorService s(drv, logger);
	unsigned int num_callbacks = 0;

	system_timer->start();

	unsigned int period = 0;
	bool sensor_en = true;

	configuration_store->write_param(ParamID::AXL_SENSOR_ENABLE, sensor_en);
	configuration_store->write_param(ParamID::AXL_SENSOR_PERIODIC, period);

	expect_calibration_writes();
	s.start([&num_callbacks](ServiceEvent &event) {
		if (event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
			num_callbacks++;
		}
	});

	// Sampling should happen every 10
	for (unsigned int i = 0; i < 5; i++) {
		fake_timer->increment_counter(period * 1000);
		system_scheduler->run();
	}

	CHECK_EQUAL(0, num_callbacks);
	CHECK_EQUAL(0, logger->num_entries());

	s.stop();
}
