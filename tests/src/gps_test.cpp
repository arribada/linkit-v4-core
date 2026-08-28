#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

#include "gps_service.hpp"
#include "mock_m10q.hpp"
#include "fake_rtc.hpp"
#include "fake_config_store.hpp"
#include "fake_logger.hpp"
#include "fake_timer.hpp"
#include "fake_battery_mon.hpp"
#include "dte_protocol.hpp"
#include "mock_comparators.hpp"
#include "axl_sensor_service.hpp"
#include "moored_mode_service.hpp"


extern Timer *system_timer;
extern ConfigurationStore *configuration_store;
extern Scheduler *system_scheduler;
extern RTC *rtc;
extern BatteryMonitor *battery_monitor;

#define FIRST_AQPERIOD (30)


TEST_GROUP(GPSService) {
	FakeBatteryMonitor *fake_battery_mon;
	FakeConfigurationStore *fake_config_store;
	FakeRTC *fake_rtc;
	FakeTimer *fake_timer;
	FakeLog *fake_log;
	MockM10Q *mock_m10q;
	MockStdFunctionVoidComparator m_comparator_std_func;
	MockGPSNavSettingsComparator m_comparator_nav;

	void setup() {
		mock().installComparator("std::function<void()>", m_comparator_std_func);
		mock().installComparator("const GPSNavSettings&", m_comparator_nav);
		fake_log = new FakeLog("GPS");
		mock_m10q = new MockM10Q;
		fake_config_store = new FakeConfigurationStore;
		configuration_store = fake_config_store;
		fake_battery_mon = new FakeBatteryMonitor;
		battery_monitor = fake_battery_mon;
		fake_rtc = new FakeRTC;
		rtc = fake_rtc;
		fake_timer = new FakeTimer;
		system_timer = fake_timer;
		system_scheduler = new Scheduler(system_timer);
		m_current_ms = 0;

		// Initialise configuration store (applies defaults)
		configuration_store->init();
		// The moored classifier is static and consulted by every
		// get_gnss_configuration(); a state leaked from another test would
		// silently rewrite dloc_arg_nom here.
		MooredModeService::reset_for_tests();
	}

	void teardown() {
		MooredModeService::reset_for_tests();
		delete system_scheduler;
		delete fake_timer;
		delete fake_rtc;
		rtc = nullptr;
		delete fake_config_store;
		delete fake_battery_mon;
		delete mock_m10q;
		delete fake_log;
	}

	void increment_time_ms(uint64_t ms) {
		while (ms) {
			m_current_ms++;
			if (m_current_ms % 1000 == 0) fake_rtc->incrementtime(1);
			fake_timer->increment_counter(1);

			system_scheduler->run();

			ms--;
		}
	}

	void increment_time_s(uint64_t s) {
		increment_time_ms(s * 1000);
	}

	void increment_time_min(uint64_t min) {
		increment_time_ms(min * 60 * 1000);
	}

	void notify_underwater_state(bool state) {
		ServiceEvent e;
		e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
		e.event_data = state;
		e.event_source = ServiceIdentifier::UW_SENSOR;
		e.event_originator_unique_id = 0x12345678;
		ServiceManager::notify_peer_event(e);
	}

	uint64_t m_current_ms;
};


TEST(GPSService, GNSSDisabled) {
	bool lb_en = false;
	unsigned int lb_threshold = 0U;
	bool gnss_en = false;
	unsigned int dloc_arg_nom = 10 * 60;
	unsigned int gnss_acq_timeout = 0;
	unsigned int gnss_acq_timeout_cold_start = 0;
	bool gnss_hdopfilt_en = false;
	unsigned int gnss_hdopfilt_thres = 0;
	bool underwater_en = false;

	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::GNSS_EN, gnss_en);
	fake_config_store->write_param(ParamID::DLOC_ARG_NOM, dloc_arg_nom);
	fake_config_store->write_param(ParamID::GNSS_ACQ_TIMEOUT, gnss_acq_timeout);
	fake_config_store->write_param(ParamID::GNSS_COLD_ACQ_TIMEOUT, gnss_acq_timeout_cold_start);
	fake_config_store->write_param(ParamID::GNSS_HDOPFILT_EN, gnss_hdopfilt_en);
	fake_config_store->write_param(ParamID::GNSS_HDOPFILT_THR, gnss_hdopfilt_thres);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, underwater_en);
	BaseGNSSFixMode fix_mode = BaseGNSSFixMode::FIX_2D;
	fake_config_store->write_param(ParamID::GNSS_FIX_MODE, fix_mode);
	BaseGNSSDynModel dyn_model = BaseGNSSDynModel::SEA;
	fake_config_store->write_param(ParamID::GNSS_DYN_MODEL, dyn_model);

	GPSService s(*mock_m10q, fake_log);
	s.start();

	increment_time_min(60);
}

TEST(GPSService, GNSSEnabled10MinutesDloc) {
	bool lb_en = false;
	unsigned int lb_threshold = 0U;
	bool gnss_en = true;
	unsigned int dloc_arg_nom = 10 * 60;
	unsigned int gnss_acq_timeout = 60;
	unsigned int gnss_acq_timeout_cold_start = 60;
	bool gnss_hdopfilt_en = false;
	unsigned int gnss_hdopfilt_thres = 0;
	bool underwater_en = false;

	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::GNSS_EN, gnss_en);
	fake_config_store->write_param(ParamID::DLOC_ARG_NOM, dloc_arg_nom);
	fake_config_store->write_param(ParamID::GNSS_ACQ_TIMEOUT, gnss_acq_timeout);
	fake_config_store->write_param(ParamID::GNSS_COLD_ACQ_TIMEOUT, gnss_acq_timeout_cold_start);
	fake_config_store->write_param(ParamID::GNSS_HDOPFILT_EN, gnss_hdopfilt_en);
	fake_config_store->write_param(ParamID::GNSS_HDOPFILT_THR, gnss_hdopfilt_thres);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, underwater_en);
	BaseGNSSFixMode fix_mode = BaseGNSSFixMode::FIX_2D;
	fake_config_store->write_param(ParamID::GNSS_FIX_MODE, fix_mode);
	BaseGNSSDynModel dyn_model = BaseGNSSDynModel::SEA;
	fake_config_store->write_param(ParamID::GNSS_DYN_MODEL, dyn_model);

	fake_rtc->settime(1580083200);  // 27/01/2020 00:00:00

	GPSService s(*mock_m10q, fake_log);
	s.start();
	unsigned int offset = FIRST_AQPERIOD;

	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(offset);

	// Send dummy fix
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_gnss_data(fake_rtc->gettime(), 10, 10);

	int iterations = 3;
	for (int i = 0; i < iterations; ++i) {
		mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
		increment_time_s(dloc_arg_nom - offset);
		mock().expectOneCall("power_off").onObject(mock_m10q);
		mock_m10q->notify_max_nav_samples();
		offset = 0;
	}
}

TEST(GPSService, GNSSEnabled15MinutesDloc) {
	bool lb_en = false;
	unsigned int lb_threshold = 0U;
	bool gnss_en = true;
	unsigned int dloc_arg_nom = 15 * 60;
	unsigned int gnss_acq_timeout = 60;
	unsigned int gnss_acq_timeout_cold_start = 60;
	bool gnss_hdopfilt_en = false;
	unsigned int gnss_hdopfilt_thres = 0;
	bool underwater_en = false;

	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::GNSS_EN, gnss_en);
	fake_config_store->write_param(ParamID::DLOC_ARG_NOM, dloc_arg_nom);
	fake_config_store->write_param(ParamID::GNSS_ACQ_TIMEOUT, gnss_acq_timeout);
	fake_config_store->write_param(ParamID::GNSS_COLD_ACQ_TIMEOUT, gnss_acq_timeout_cold_start);
	fake_config_store->write_param(ParamID::GNSS_HDOPFILT_EN, gnss_hdopfilt_en);
	fake_config_store->write_param(ParamID::GNSS_HDOPFILT_THR, gnss_hdopfilt_thres);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, underwater_en);
	BaseGNSSFixMode fix_mode = BaseGNSSFixMode::FIX_2D;
	fake_config_store->write_param(ParamID::GNSS_FIX_MODE, fix_mode);
	BaseGNSSDynModel dyn_model = BaseGNSSDynModel::SEA;
	fake_config_store->write_param(ParamID::GNSS_DYN_MODEL, dyn_model);

	fake_rtc->settime(1580083200);  // 27/01/2020 00:00:00

	GPSService s(*mock_m10q, fake_log);
	s.start();
	unsigned int offset = FIRST_AQPERIOD;

	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(offset);

	// Send dummy fix
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_gnss_data(fake_rtc->gettime(), 10, 10);

	int iterations = 3;
	for (int i = 0; i < iterations; ++i) {
		mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
		increment_time_s(dloc_arg_nom - offset);
		mock().expectOneCall("power_off").onObject(mock_m10q);
		mock_m10q->notify_max_nav_samples();
		offset = 0;
	}
}

TEST(GPSService, GNSSEnabled30MinutesDloc) {
	bool lb_en = false;
	unsigned int lb_threshold = 0U;
	bool gnss_en = true;
	unsigned int dloc_arg_nom = 30 * 60;
	unsigned int gnss_acq_timeout = 60;
	unsigned int gnss_acq_timeout_cold_start = 60;
	bool gnss_hdopfilt_en = false;
	unsigned int gnss_hdopfilt_thres = 0;
	bool underwater_en = false;

	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::GNSS_EN, gnss_en);
	fake_config_store->write_param(ParamID::DLOC_ARG_NOM, dloc_arg_nom);
	fake_config_store->write_param(ParamID::GNSS_ACQ_TIMEOUT, gnss_acq_timeout);
	fake_config_store->write_param(ParamID::GNSS_COLD_ACQ_TIMEOUT, gnss_acq_timeout_cold_start);
	fake_config_store->write_param(ParamID::GNSS_HDOPFILT_EN, gnss_hdopfilt_en);
	fake_config_store->write_param(ParamID::GNSS_HDOPFILT_THR, gnss_hdopfilt_thres);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, underwater_en);
	BaseGNSSFixMode fix_mode = BaseGNSSFixMode::FIX_2D;
	fake_config_store->write_param(ParamID::GNSS_FIX_MODE, fix_mode);
	BaseGNSSDynModel dyn_model = BaseGNSSDynModel::SEA;
	fake_config_store->write_param(ParamID::GNSS_DYN_MODEL, dyn_model);

	fake_rtc->settime(1580083200);  // 27/01/2020 00:00:00

	GPSService s(*mock_m10q, fake_log);
	s.start();
	unsigned int offset = FIRST_AQPERIOD;

	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(offset);

	// Send dummy fix
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_gnss_data(fake_rtc->gettime(), 10, 10);

	int iterations = 3;
	for (int i = 0; i < iterations; ++i) {
		mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
		increment_time_s(dloc_arg_nom - offset);
		mock().expectOneCall("power_off").onObject(mock_m10q);
		mock_m10q->notify_max_nav_samples();
		offset = 0;
	}
}

TEST(GPSService, GNSSEnabled60MinutesDloc) {
	bool lb_en = false;
	unsigned int lb_threshold = 0U;
	bool gnss_en = true;
	unsigned int dloc_arg_nom = 60 * 60;
	unsigned int gnss_acq_timeout = 60;
	unsigned int gnss_acq_timeout_cold_start = 60;
	bool gnss_hdopfilt_en = false;
	unsigned int gnss_hdopfilt_thres = 0;
	bool underwater_en = false;

	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::GNSS_EN, gnss_en);
	fake_config_store->write_param(ParamID::DLOC_ARG_NOM, dloc_arg_nom);
	fake_config_store->write_param(ParamID::GNSS_ACQ_TIMEOUT, gnss_acq_timeout);
	fake_config_store->write_param(ParamID::GNSS_COLD_ACQ_TIMEOUT, gnss_acq_timeout_cold_start);
	fake_config_store->write_param(ParamID::GNSS_HDOPFILT_EN, gnss_hdopfilt_en);
	fake_config_store->write_param(ParamID::GNSS_HDOPFILT_THR, gnss_hdopfilt_thres);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, underwater_en);
	BaseGNSSFixMode fix_mode = BaseGNSSFixMode::FIX_2D;
	fake_config_store->write_param(ParamID::GNSS_FIX_MODE, fix_mode);
	BaseGNSSDynModel dyn_model = BaseGNSSDynModel::SEA;
	fake_config_store->write_param(ParamID::GNSS_DYN_MODEL, dyn_model);

	fake_rtc->settime(1580083200);  // 27/01/2020 00:00:00

	GPSService s(*mock_m10q, fake_log);
	s.start();
	unsigned int offset = FIRST_AQPERIOD;

	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(offset);

	// Send dummy fix
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_gnss_data(fake_rtc->gettime(), 10, 10);

	int iterations = 3;
	for (int i = 0; i < iterations; ++i) {
		mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
		increment_time_s(dloc_arg_nom - offset);
		mock().expectOneCall("power_off").onObject(mock_m10q);
		mock_m10q->notify_max_nav_samples();
		offset = 0;
	}
}

TEST(GPSService, GNSSEnabled120MinutesDloc) {
	bool lb_en = false;
	unsigned int lb_threshold = 0U;
	bool gnss_en = true;
	unsigned int dloc_arg_nom = 120 * 60;
	unsigned int gnss_acq_timeout = 60;
	unsigned int gnss_acq_timeout_cold_start = 60;
	bool gnss_hdopfilt_en = false;
	unsigned int gnss_hdopfilt_thres = 0;
	bool underwater_en = false;

	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::GNSS_EN, gnss_en);
	fake_config_store->write_param(ParamID::DLOC_ARG_NOM, dloc_arg_nom);
	fake_config_store->write_param(ParamID::GNSS_ACQ_TIMEOUT, gnss_acq_timeout);
	fake_config_store->write_param(ParamID::GNSS_COLD_ACQ_TIMEOUT, gnss_acq_timeout_cold_start);
	fake_config_store->write_param(ParamID::GNSS_HDOPFILT_EN, gnss_hdopfilt_en);
	fake_config_store->write_param(ParamID::GNSS_HDOPFILT_THR, gnss_hdopfilt_thres);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, underwater_en);
	BaseGNSSFixMode fix_mode = BaseGNSSFixMode::FIX_2D;
	fake_config_store->write_param(ParamID::GNSS_FIX_MODE, fix_mode);
	BaseGNSSDynModel dyn_model = BaseGNSSDynModel::SEA;
	fake_config_store->write_param(ParamID::GNSS_DYN_MODEL, dyn_model);

	fake_rtc->settime(1580083200);  // 27/01/2020 00:00:00

	GPSService s(*mock_m10q, fake_log);
	s.start();
	unsigned int offset = FIRST_AQPERIOD;

	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(offset);

	// Send dummy fix
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_gnss_data(fake_rtc->gettime(), 10, 10);

	int iterations = 3;
	for (int i = 0; i < iterations; ++i) {
		mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
		increment_time_s(dloc_arg_nom - offset);
		mock().expectOneCall("power_off").onObject(mock_m10q);
		mock_m10q->notify_max_nav_samples();
		offset = 0;
	}
}

TEST(GPSService, GNSSEnabledColdStartTimeoutAndRetryCheck) {
	bool lb_en = false;
	unsigned int lb_threshold = 0U;
	bool gnss_en = true;
	unsigned int dloc_arg_nom = 10 * 60;
	unsigned int gnss_acq_timeout = 60;
	unsigned int gnss_acq_timeout_cold_start = 120;
	unsigned int gnss_cold_start_retry_period = 60;
	bool gnss_hdopfilt_en = false;
	unsigned int gnss_hdopfilt_thres = 0;
	bool underwater_en = false;

	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::GNSS_EN, gnss_en);
	fake_config_store->write_param(ParamID::DLOC_ARG_NOM, dloc_arg_nom);
	fake_config_store->write_param(ParamID::GNSS_ACQ_TIMEOUT, gnss_acq_timeout);
	fake_config_store->write_param(ParamID::GNSS_COLD_ACQ_TIMEOUT, gnss_acq_timeout_cold_start);
	fake_config_store->write_param(ParamID::GNSS_HDOPFILT_EN, gnss_hdopfilt_en);
	fake_config_store->write_param(ParamID::GNSS_HDOPFILT_THR, gnss_hdopfilt_thres);
	fake_config_store->write_param(ParamID::GNSS_COLD_START_RETRY_PERIOD, gnss_cold_start_retry_period);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, underwater_en);
	BaseGNSSFixMode fix_mode = BaseGNSSFixMode::FIX_2D;
	fake_config_store->write_param(ParamID::GNSS_FIX_MODE, fix_mode);
	BaseGNSSDynModel dyn_model = BaseGNSSDynModel::SEA;
	fake_config_store->write_param(ParamID::GNSS_DYN_MODEL, dyn_model);

	fake_rtc->settime(1580083200);  // 27/01/2020 00:00:00

	GPSService s(*mock_m10q, fake_log);
	s.start();

	// We're expecting the device to turn on at 27/01/2020 00:00:30

	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(FIRST_AQPERIOD);

	// Should power down after cold start timeout
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_max_nav_samples();

	// Repeat to make sure cold start repeats after retry period (on UTC 60 seconds)
	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(gnss_cold_start_retry_period - FIRST_AQPERIOD);

	// Make a fix to exit cold start mode
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_gnss_data(fake_rtc->gettime(), 10, 10);

	// Next schedule should be back to dloc_arg_nom
	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(dloc_arg_nom - (m_current_ms / 1000) + 1);
}

TEST(GPSService, GNSSEnabledNominalTimeoutAfterFirstFix) {
	bool lb_en = false;
	unsigned int lb_threshold = 0U;
	bool gnss_en = true;
	unsigned int dloc_arg_nom = 10 * 60;
	unsigned int gnss_acq_timeout = 60;
	unsigned int gnss_acq_timeout_cold_start = 120;
	bool gnss_hdopfilt_en = false;
	unsigned int gnss_hdopfilt_thres = 0;
	bool underwater_en = false;

	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::GNSS_EN, gnss_en);
	fake_config_store->write_param(ParamID::DLOC_ARG_NOM, dloc_arg_nom);
	fake_config_store->write_param(ParamID::GNSS_ACQ_TIMEOUT, gnss_acq_timeout);
	fake_config_store->write_param(ParamID::GNSS_COLD_ACQ_TIMEOUT, gnss_acq_timeout_cold_start);
	fake_config_store->write_param(ParamID::GNSS_HDOPFILT_EN, gnss_hdopfilt_en);
	fake_config_store->write_param(ParamID::GNSS_HDOPFILT_THR, gnss_hdopfilt_thres);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, underwater_en);
	BaseGNSSFixMode fix_mode = BaseGNSSFixMode::FIX_2D;
	fake_config_store->write_param(ParamID::GNSS_FIX_MODE, fix_mode);
	BaseGNSSDynModel dyn_model = BaseGNSSDynModel::SEA;
	fake_config_store->write_param(ParamID::GNSS_DYN_MODEL, dyn_model);

	fake_rtc->settime(1580083200);  // 27/01/2020 00:00:00

	GPSService s(*mock_m10q, fake_log);
	s.start();

	// We're expecting the device to turn on at 27/01/2020 00:00:30
	increment_time_s(FIRST_AQPERIOD - 1);

	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(1);

	// Should not power down yet
	increment_time_s(gnss_acq_timeout);

	// Send a dummy GNSS data event to mark the first fix as being made
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_gnss_data(fake_rtc->gettime());

	printf("rtc = %llu\n", fake_rtc->gettime());

	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(530);

	printf("rtc = %llu\n", fake_rtc->gettime());

	// Should now power down at nominal timeout
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_max_nav_samples();
}

TEST(GPSService, GNSSInterruptedByUnderwaterEvent) {
	bool lb_en = false;
	unsigned int lb_threshold = 0U;
	bool gnss_en = true;
	unsigned int dloc_arg_nom = 10 * 60;
	unsigned int gnss_acq_timeout = 60;
	unsigned int gnss_acq_timeout_cold_start = 120;
	bool gnss_hdopfilt_en = false;
	bool gnss_haccfilt_en = false;
	bool underwater_en = true;

	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::GNSS_EN, gnss_en);
	fake_config_store->write_param(ParamID::DLOC_ARG_NOM, dloc_arg_nom);
	fake_config_store->write_param(ParamID::GNSS_ACQ_TIMEOUT, gnss_acq_timeout);
	fake_config_store->write_param(ParamID::GNSS_COLD_ACQ_TIMEOUT, gnss_acq_timeout_cold_start);
	fake_config_store->write_param(ParamID::GNSS_HDOPFILT_EN, gnss_hdopfilt_en);
	fake_config_store->write_param(ParamID::GNSS_HACCFILT_EN, gnss_haccfilt_en);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, underwater_en);
	BaseGNSSFixMode fix_mode = BaseGNSSFixMode::FIX_2D;
	fake_config_store->write_param(ParamID::GNSS_FIX_MODE, fix_mode);
	BaseGNSSDynModel dyn_model = BaseGNSSDynModel::SEA;
	fake_config_store->write_param(ParamID::GNSS_DYN_MODEL, dyn_model);

	fake_rtc->settime(1580083200);  // 27/01/2020 00:00:00

	GPSService s(*mock_m10q, fake_log);
	s.start();

	// We're expecting the device to turn on at 27/01/2020 00:00:30
	increment_time_s(FIRST_AQPERIOD - 1);

	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(1);

	// Now fire an underwater event before we get GPS lock
	mock().expectOneCall("power_off").onObject(mock_m10q);
	notify_underwater_state(true);
}


TEST(GPSService, GNSSIgnoredAfterUnderwaterEvent) {
	bool lb_en = false;
	unsigned int lb_threshold = 0U;
	bool gnss_en = true;
	unsigned int dloc_arg_nom = 10 * 60;
	unsigned int gnss_acq_timeout = 60;
	bool gnss_hdopfilt_en = false;
	bool gnss_haccfilt_en = false;
	bool underwater_en = true;

	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::GNSS_EN, gnss_en);
	fake_config_store->write_param(ParamID::DLOC_ARG_NOM, dloc_arg_nom);
	fake_config_store->write_param(ParamID::GNSS_ACQ_TIMEOUT, gnss_acq_timeout);
	fake_config_store->write_param(ParamID::GNSS_HDOPFILT_EN, gnss_hdopfilt_en);
	fake_config_store->write_param(ParamID::GNSS_HACCFILT_EN, gnss_haccfilt_en);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, underwater_en);
	BaseGNSSFixMode fix_mode = BaseGNSSFixMode::FIX_2D;
	fake_config_store->write_param(ParamID::GNSS_FIX_MODE, fix_mode);
	BaseGNSSDynModel dyn_model = BaseGNSSDynModel::SEA;
	fake_config_store->write_param(ParamID::GNSS_DYN_MODEL, dyn_model);

	fake_rtc->settime(1580083200);  // 27/01/2020 00:00:00

	GPSService s(*mock_m10q, fake_log);
	s.start();

	// Now fire an underwater event before we schedule
	notify_underwater_state(true);

	// We're expecting the device to turn on at 27/01/2020 00:00:30 - remain off
	increment_time_s(FIRST_AQPERIOD);

	// Next schedule attempt will be at 00:01:00 - remain off
	increment_time_s(30);

	// Next schedule attempt will be at 00:02:00 - remain off
	increment_time_s(60);

	// Now fire a surfaced event - next time will power on
	notify_underwater_state(false);

	// Next schedule attempt will be at 00:03:00 - power on
	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(60);
}


TEST(GPSService, GNSSNoPeriodicTriggerOnSurfaceEvent) {
	bool lb_en = false;
	unsigned int lb_threshold = 0U;
	bool gnss_en = true;
	unsigned int dloc_arg_nom = 0;
	unsigned int gnss_acq_timeout = 60;
	unsigned int gnss_acq_timeout_cold_start = 60;
	bool gnss_hdopfilt_en = false;
	unsigned int gnss_hdopfilt_thres = 0;
	bool underwater_en = true;
	bool trigger_surface_en = true;

	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::GNSS_EN, gnss_en);
	fake_config_store->write_param(ParamID::DLOC_ARG_NOM, dloc_arg_nom);
	fake_config_store->write_param(ParamID::GNSS_ACQ_TIMEOUT, gnss_acq_timeout);
	fake_config_store->write_param(ParamID::GNSS_COLD_ACQ_TIMEOUT, gnss_acq_timeout_cold_start);
	fake_config_store->write_param(ParamID::GNSS_HDOPFILT_EN, gnss_hdopfilt_en);
	fake_config_store->write_param(ParamID::GNSS_HDOPFILT_THR, gnss_hdopfilt_thres);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, underwater_en);
	BaseGNSSFixMode fix_mode = BaseGNSSFixMode::FIX_2D;
	fake_config_store->write_param(ParamID::GNSS_FIX_MODE, fix_mode);
	BaseGNSSDynModel dyn_model = BaseGNSSDynModel::SEA;
	fake_config_store->write_param(ParamID::GNSS_DYN_MODEL, dyn_model);
	fake_config_store->write_param(ParamID::GNSS_TRIGGER_ON_SURFACED, trigger_surface_en);

	fake_rtc->settime(0);

	GPSService s(*mock_m10q, fake_log);
	s.start();

	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(FIRST_AQPERIOD);

	// Send dummy fix
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_gnss_data(fake_rtc->gettime(), 10, 10);

	// Now fire a surfaced event - next time will power on
	notify_underwater_state(false);
	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(1);

	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_gnss_data(fake_rtc->gettime(), 10, 10);
}


TEST(GPSService, GNSSNoPeriodicTriggerOnAXLWakeupEvent) {
	bool lb_en = false;
	unsigned int lb_threshold = 0U;
	bool gnss_en = true;
	unsigned int dloc_arg_nom = 0;
	unsigned int gnss_acq_timeout = 60;
	unsigned int gnss_acq_timeout_cold_start = 60;
	bool gnss_hdopfilt_en = false;
	unsigned int gnss_hdopfilt_thres = 0;
	bool underwater_en = true;
	bool trigger_axl_en = true;

	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::GNSS_EN, gnss_en);
	fake_config_store->write_param(ParamID::DLOC_ARG_NOM, dloc_arg_nom);
	fake_config_store->write_param(ParamID::GNSS_ACQ_TIMEOUT, gnss_acq_timeout);
	fake_config_store->write_param(ParamID::GNSS_COLD_ACQ_TIMEOUT, gnss_acq_timeout_cold_start);
	fake_config_store->write_param(ParamID::GNSS_HDOPFILT_EN, gnss_hdopfilt_en);
	fake_config_store->write_param(ParamID::GNSS_HDOPFILT_THR, gnss_hdopfilt_thres);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, underwater_en);
	BaseGNSSFixMode fix_mode = BaseGNSSFixMode::FIX_2D;
	fake_config_store->write_param(ParamID::GNSS_FIX_MODE, fix_mode);
	BaseGNSSDynModel dyn_model = BaseGNSSDynModel::SEA;
	fake_config_store->write_param(ParamID::GNSS_DYN_MODEL, dyn_model);
	fake_config_store->write_param(ParamID::GNSS_TRIGGER_ON_AXL_WAKEUP, trigger_axl_en);

	fake_rtc->settime(0);

	GPSService s(*mock_m10q, fake_log);
	s.start();

	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(FIRST_AQPERIOD);

	// Send dummy fix
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_gnss_data(fake_rtc->gettime(), 10, 10);

	// Now fire an AXL event - next time will power on
	ServiceEvent e;
	e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
	ServiceSensorData sensor_data = {};
	sensor_data.port[AXLSensorPort::WAKEUP_TRIGGERED] = 1.0;
	e.event_data = sensor_data;
	e.event_source = ServiceIdentifier::AXL_SENSOR;
	e.event_originator_unique_id = 0x12345678;
	s.notify_peer_event(e);
	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(1);

	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_gnss_data(fake_rtc->gettime(), 10, 10);
}


TEST(GPSService, GNSSInterruptedByErrorEvent) {
	bool lb_en = false;
	unsigned int lb_threshold = 0U;
	bool gnss_en = true;
	unsigned int dloc_arg_nom = 10 * 60;
	unsigned int gnss_acq_timeout = 60;
	unsigned int gnss_acq_timeout_cold_start = 120;
	bool gnss_hdopfilt_en = false;
	bool gnss_haccfilt_en = false;
	bool underwater_en = true;

	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::GNSS_EN, gnss_en);
	fake_config_store->write_param(ParamID::DLOC_ARG_NOM, dloc_arg_nom);
	fake_config_store->write_param(ParamID::GNSS_ACQ_TIMEOUT, gnss_acq_timeout);
	fake_config_store->write_param(ParamID::GNSS_COLD_ACQ_TIMEOUT, gnss_acq_timeout_cold_start);
	fake_config_store->write_param(ParamID::GNSS_HDOPFILT_EN, gnss_hdopfilt_en);
	fake_config_store->write_param(ParamID::GNSS_HACCFILT_EN, gnss_haccfilt_en);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, underwater_en);
	BaseGNSSFixMode fix_mode = BaseGNSSFixMode::FIX_2D;
	fake_config_store->write_param(ParamID::GNSS_FIX_MODE, fix_mode);
	BaseGNSSDynModel dyn_model = BaseGNSSDynModel::SEA;
	fake_config_store->write_param(ParamID::GNSS_DYN_MODEL, dyn_model);

	fake_rtc->settime(1580083200);  // 27/01/2020 00:00:00

	GPSService s(*mock_m10q, fake_log);
	s.start();

	// We're expecting the device to turn on at 27/01/2020 00:00:30
	increment_time_s(FIRST_AQPERIOD - 1);

	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(1);

	// Now fire an underwater event before we get GPS lock
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_error();
}


TEST(GPSService, GNSSNoPeriodicColdStartOnSurfaceEvent) {
	bool lb_en = false;
	unsigned int lb_threshold = 0U;
	bool gnss_en = true;
	unsigned int dloc_arg_nom = 0;
	unsigned int gnss_acq_timeout = 60;
	unsigned int gnss_acq_timeout_cold_start = 120;
	bool gnss_hdopfilt_en = false;
	unsigned int gnss_hdopfilt_thres = 0;
	bool underwater_en = true;
	bool trigger_surface_en = true;

	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::GNSS_EN, gnss_en);
	fake_config_store->write_param(ParamID::DLOC_ARG_NOM, dloc_arg_nom);
	fake_config_store->write_param(ParamID::GNSS_ACQ_TIMEOUT, gnss_acq_timeout);
	fake_config_store->write_param(ParamID::GNSS_COLD_ACQ_TIMEOUT, gnss_acq_timeout_cold_start);
	fake_config_store->write_param(ParamID::GNSS_HDOPFILT_EN, gnss_hdopfilt_en);
	fake_config_store->write_param(ParamID::GNSS_HDOPFILT_THR, gnss_hdopfilt_thres);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, underwater_en);
	BaseGNSSFixMode fix_mode = BaseGNSSFixMode::FIX_2D;
	fake_config_store->write_param(ParamID::GNSS_FIX_MODE, fix_mode);
	BaseGNSSDynModel dyn_model = BaseGNSSDynModel::SEA;
	fake_config_store->write_param(ParamID::GNSS_DYN_MODEL, dyn_model);
	fake_config_store->write_param(ParamID::GNSS_TRIGGER_ON_SURFACED, trigger_surface_en);
	fake_config_store->write_param(ParamID::GNSS_TRIGGER_COLD_START_ON_SURFACED, trigger_surface_en);

	fake_rtc->settime(0);

	GPSService s(*mock_m10q, fake_log);
	s.start();

	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(FIRST_AQPERIOD);

	// Send dummy fix (this is the initial cold start fix)
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_gnss_data(fake_rtc->gettime(), 10, 10);

	// Now fire a surfaced event - next time will power on
	notify_underwater_state(false);
	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(1);
}

// ============================================================================
// FASTLOC (DEGRADED PVT) TESTS
// ============================================================================

TEST(GPSService, FastlocDegradedPVTLoggedWhenEnabled) {
	// GNP45=1 (DEGRADED_PVT): degraded fix should produce FASTLOC log entry
	fake_config_store->write_param(ParamID::GNSS_EN, (bool)true);
	fake_config_store->write_param(ParamID::GNSS_FASTLOC_MODE, 1U);  // 1=DEGRADED_PVT
	fake_config_store->write_param(ParamID::DLOC_ARG_NOM, 600U);
	fake_config_store->write_param(ParamID::GNSS_ACQ_TIMEOUT, 60U);
	fake_config_store->write_param(ParamID::GNSS_COLD_ACQ_TIMEOUT, 60U);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, (bool)false);
	fake_config_store->write_param(ParamID::GNSS_HDOPFILT_EN, (bool)false);
	fake_config_store->write_param(ParamID::GNSS_FIX_MODE, BaseGNSSFixMode::AUTO);
	fake_config_store->write_param(ParamID::GNSS_DYN_MODEL, BaseGNSSDynModel::SEA);

	fake_rtc->settime(1580083200);

	fake_log->create();  // Reset log index
	GPSService s(*mock_m10q, fake_log);
	s.start();

	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(FIRST_AQPERIOD);

	// GPS sends degraded PVT (timeout, fix failed quality filters)
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_degraded_gnss_data(fake_rtc->gettime(), -21.01, 55.27, 8.0, 150000, 2, 3);

	// The service should have completed with a FASTLOC log entry
	CHECK_EQUAL(1U, fake_log->num_entries());
}

TEST(GPSService, FastlocDisabledTreatsAsNoFix) {
	// GNP45=0 (OFF): degraded fix should be treated as no fix
	fake_config_store->write_param(ParamID::GNSS_EN, (bool)true);
	fake_config_store->write_param(ParamID::GNSS_FASTLOC_MODE, 0U);  // 0=OFF
	fake_config_store->write_param(ParamID::DLOC_ARG_NOM, 600U);
	fake_config_store->write_param(ParamID::GNSS_ACQ_TIMEOUT, 60U);
	fake_config_store->write_param(ParamID::GNSS_COLD_ACQ_TIMEOUT, 60U);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, (bool)false);
	fake_config_store->write_param(ParamID::GNSS_HDOPFILT_EN, (bool)false);
	fake_config_store->write_param(ParamID::GNSS_FIX_MODE, BaseGNSSFixMode::AUTO);
	fake_config_store->write_param(ParamID::GNSS_DYN_MODEL, BaseGNSSDynModel::SEA);

	fake_rtc->settime(1580083200);

	fake_log->create();  // Reset log index
	GPSService s(*mock_m10q, fake_log);
	s.start();

	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(FIRST_AQPERIOD);

	// GPS sends degraded PVT but fastloc is disabled
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_degraded_gnss_data(fake_rtc->gettime(), -21.01, 55.27, 8.0, 150000, 2, 3);

	// Should still log (as invalid/no-fix), but event_type should not be FASTLOC
	CHECK_EQUAL(1U, fake_log->num_entries());
}

TEST(GPSService, FastlocDoesNotSetFirstFixFound) {
	// Degraded fix should NOT set m_is_first_fix_found — cold start retry period used
	unsigned int cold_retry = 60U;  // GNSS_COLD_START_RETRY_PERIOD default
	fake_config_store->write_param(ParamID::GNSS_EN, (bool)true);
	fake_config_store->write_param(ParamID::GNSS_FASTLOC_MODE, 1U);  // 1=DEGRADED_PVT
	fake_config_store->write_param(ParamID::DLOC_ARG_NOM, 600U);
	fake_config_store->write_param(ParamID::GNSS_ACQ_TIMEOUT, 60U);
	fake_config_store->write_param(ParamID::GNSS_COLD_ACQ_TIMEOUT, 120U);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, (bool)false);
	fake_config_store->write_param(ParamID::GNSS_HDOPFILT_EN, (bool)false);
	fake_config_store->write_param(ParamID::GNSS_FIX_MODE, BaseGNSSFixMode::AUTO);
	fake_config_store->write_param(ParamID::GNSS_DYN_MODEL, BaseGNSSDynModel::SEA);

	fake_rtc->settime(1580083200);

	fake_log->create();  // Reset log index
	GPSService s(*mock_m10q, fake_log);
	s.start();

	// First session at FIRST_AQPERIOD
	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(FIRST_AQPERIOD);

	// Degraded PVT (not a real fix — first_fix_found stays false)
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_degraded_gnss_data(fake_rtc->gettime(), -21.01, 55.27);
	CHECK_EQUAL(1U, fake_log->num_entries());

	// Next session: cold_start_retry_period (60s) since first_fix_found is false
	// Advance exactly to the schedule point, send a max_nav event to avoid timeout
	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(cold_retry);
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_max_nav_samples();

	// 2 log entries total: degraded + no-fix from max_nav
	CHECK_EQUAL(2U, fake_log->num_entries());
}

TEST(GPSService, NormalFixAfterDegradedSetsFirstFix) {
	// After degraded-only → normal fix → first_fix_found should be set
	unsigned int cold_retry = 60U;
	fake_config_store->write_param(ParamID::GNSS_EN, (bool)true);
	fake_config_store->write_param(ParamID::GNSS_FASTLOC_MODE, 1U);  // 1=DEGRADED_PVT
	fake_config_store->write_param(ParamID::DLOC_ARG_NOM, 600U);
	fake_config_store->write_param(ParamID::GNSS_ACQ_TIMEOUT, 60U);
	fake_config_store->write_param(ParamID::GNSS_COLD_ACQ_TIMEOUT, 120U);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, (bool)false);
	fake_config_store->write_param(ParamID::GNSS_HDOPFILT_EN, (bool)false);
	fake_config_store->write_param(ParamID::GNSS_FIX_MODE, BaseGNSSFixMode::AUTO);
	fake_config_store->write_param(ParamID::GNSS_DYN_MODEL, BaseGNSSDynModel::SEA);

	fake_rtc->settime(1580083200);

	fake_log->create();  // Reset log index
	GPSService s(*mock_m10q, fake_log);
	s.start();

	// First session: degraded PVT (does NOT set first_fix_found)
	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(FIRST_AQPERIOD);
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_degraded_gnss_data(fake_rtc->gettime(), -21.01, 55.27);
	CHECK_EQUAL(1U, fake_log->num_entries());

	// Second session (cold_retry=60s): send real fix → sets first_fix_found
	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(cold_retry);
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_gnss_data(fake_rtc->gettime(), -21.01, 55.27, 1.5, 5000);
	CHECK_EQUAL(2U, fake_log->num_entries());

	// Verify: first_fix_found is now true (normal fix sets it).
	// Degraded fix alone did not set it (test FastlocDoesNotSetFirstFixFound covers that).
	mock().checkExpectations();
}

TEST(GPSService, MaxSatSamplesNoFix) {
	// MaxSatSamples event (no signal at all) should produce no-fix log entry
	fake_config_store->write_param(ParamID::GNSS_EN, (bool)true);
	fake_config_store->write_param(ParamID::GNSS_FASTLOC_MODE, 1U);  // 1=DEGRADED_PVT
	fake_config_store->write_param(ParamID::DLOC_ARG_NOM, 600U);
	fake_config_store->write_param(ParamID::GNSS_ACQ_TIMEOUT, 60U);
	fake_config_store->write_param(ParamID::GNSS_COLD_ACQ_TIMEOUT, 60U);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, (bool)false);
	fake_config_store->write_param(ParamID::GNSS_HDOPFILT_EN, (bool)false);
	fake_config_store->write_param(ParamID::GNSS_FIX_MODE, BaseGNSSFixMode::AUTO);
	fake_config_store->write_param(ParamID::GNSS_DYN_MODEL, BaseGNSSDynModel::SEA);

	fake_rtc->settime(1580083200);

	fake_log->create();  // Reset log index
	GPSService s(*mock_m10q, fake_log);
	s.start();

	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(FIRST_AQPERIOD);

	// No signal at all → early abort
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_max_sat_samples();

	CHECK_EQUAL(1U, fake_log->num_entries());
}

TEST(GPSService, SingleFixModeStopsAfterFirstFix) {
	// GNP30=1: GPS should not reschedule after first valid fix
	fake_config_store->write_param(ParamID::GNSS_EN, (bool)true);
	fake_config_store->write_param(ParamID::GNSS_SESSION_SINGLE_FIX, (bool)true);
	fake_config_store->write_param(ParamID::DLOC_ARG_NOM, 600U);
	fake_config_store->write_param(ParamID::GNSS_ACQ_TIMEOUT, 60U);
	fake_config_store->write_param(ParamID::GNSS_COLD_ACQ_TIMEOUT, 60U);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, (bool)false);
	fake_config_store->write_param(ParamID::GNSS_HDOPFILT_EN, (bool)false);
	fake_config_store->write_param(ParamID::GNSS_FIX_MODE, BaseGNSSFixMode::AUTO);
	fake_config_store->write_param(ParamID::GNSS_DYN_MODEL, BaseGNSSDynModel::SEA);

	fake_rtc->settime(1580083200);

	fake_log->create();  // Reset log index
	GPSService s(*mock_m10q, fake_log);
	s.start();

	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(FIRST_AQPERIOD);

	// Normal fix
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_gnss_data(fake_rtc->gettime(), -21.01, 55.27, 1.5, 5000);

	// Wait well past DLOC period — GPS should NOT power on again
	increment_time_s(1200);
	mock().checkExpectations();  // No power_on expected
}


// === MOORED mode end-to-end (2026-08) =====================================
//
// The unit tests in moored_mode_test.cpp prove the classifier's logic and
// config_store_test.cpp proves the parameter substitution, but neither shows
// that a stationary vessel actually gets a slower GNSS cadence. This one does,
// through the REAL path: a fix broadcast on the peer bus, picked up by the
// funnel in ServiceManager::notify_peer_event, feeding the classifier, whose
// state is then read back by get_gnss_configuration() inside the very same
// service_complete() that reschedules.
//
// It also pins the timing: service_complete() calls service_log() (which
// broadcasts, hence updates the classifier) BEFORE reschedule(), so the new
// cadence takes effect on the same session that engages MOORED — not one
// session later.
TEST(GPSService, MooredModeStretchesAcquisitionPeriodEndToEnd) {
	const unsigned int dloc_under_way = 600;  // 10 min
	const unsigned int dloc_moored = 3600;    // 1 h

	fake_config_store->write_param(ParamID::LB_EN, false);
	fake_config_store->write_param(ParamID::GNSS_EN, true);
	fake_config_store->write_param(ParamID::DLOC_ARG_NOM, dloc_under_way);
	fake_config_store->write_param(ParamID::GNSS_ACQ_TIMEOUT, 60U);
	fake_config_store->write_param(ParamID::GNSS_COLD_ACQ_TIMEOUT, 60U);
	fake_config_store->write_param(ParamID::GNSS_HDOPFILT_EN, false);
	fake_config_store->write_param(ParamID::GNSS_HACCFILT_EN, false);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, false);
	BaseGNSSFixMode fix_mode = BaseGNSSFixMode::FIX_2D;
	fake_config_store->write_param(ParamID::GNSS_FIX_MODE, fix_mode);
	BaseGNSSDynModel dyn_model = BaseGNSSDynModel::SEA;
	fake_config_store->write_param(ParamID::GNSS_DYN_MODEL, dyn_model);

	fake_config_store->write_param(ParamID::MOORED_DETECT_EN, true);
	fake_config_store->write_param(ParamID::MOORED_ENTER_FIXES, 2U);
	fake_config_store->write_param(ParamID::MOORED_RADIUS_M, 150U);
	fake_config_store->write_param(ParamID::MOORED_DLOC, dloc_moored);
	fake_config_store->write_param(ParamID::MOORED_GNSS_EN, true);

	// 27/01/2020 00:00:00 — divisible by 30, 600 and 3600, so every UTC-aligned
	// schedule below is exact arithmetic rather than an approximation.
	fake_rtc->settime(1580083200);

	GPSService s(*mock_m10q, fake_log);
	// Start with the production notification callback so the fix actually
	// travels the peer bus and reaches the moored funnel. s.start() with no
	// argument leaves m_data_notification_callback null and nothing is
	// broadcast at all — the funnel would never run.
	s.start([](ServiceEvent &e) { ServiceManager::notify_peer_event(e); });

	// --- Session 1: first schedule, plants the reference anchor -------------
	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(FIRST_AQPERIOD);
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_gnss_data(fake_rtc->gettime(), 10, 10);
	CHECK_TRUE(MooredModeService::has_reference());
	CHECK_FALSE(MooredModeService::is_moored());

	// --- Session 2: one stationary fix, still under way ---------------------
	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(dloc_under_way - FIRST_AQPERIOD);  // UTC-aligned: 570 s
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_gnss_data(fake_rtc->gettime(), 10, 10);
	CHECK_EQUAL(1U, MooredModeService::stationary_fixes());
	CHECK_FALSE(MooredModeService::is_moored());

	// --- Session 3: second stationary fix -> MOORED engages -----------------
	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(dloc_under_way);
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_gnss_data(fake_rtc->gettime(), 10, 10);
	CHECK_TRUE(MooredModeService::is_moored());

	// The reschedule that just ran must have used MOORED_DLOC. Time is now
	// 1580084400, i.e. 1200 s past the 3600 s UTC boundary, so the next
	// acquisition is due at +2400 s — NOT at the 600 s under-way cadence.
	//
	// No expectation is armed here on purpose: if power_on fires inside this
	// window CppUTest reports it as an unexpected call and the test fails. That
	// is the actual assertion — "the old cadence is gone".
	increment_time_s(dloc_under_way);
	mock().checkExpectations();
	CHECK_TRUE(MooredModeService::is_moored());

	// And it does fire once the moored period elapses.
	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(1800);  // 600 + 1800 = 2400
	mock().checkExpectations();
}

// Same setup, master switch off: the cadence must stay at DLOC_ARG_NOM however
// stationary the vessel is. This is the non-regression guarantee for every
// deployment that never enables the feature.
TEST(GPSService, MooredModeDisabledLeavesAcquisitionPeriodUntouched) {
	const unsigned int dloc_under_way = 600;

	fake_config_store->write_param(ParamID::LB_EN, false);
	fake_config_store->write_param(ParamID::GNSS_EN, true);
	fake_config_store->write_param(ParamID::DLOC_ARG_NOM, dloc_under_way);
	fake_config_store->write_param(ParamID::GNSS_ACQ_TIMEOUT, 60U);
	fake_config_store->write_param(ParamID::GNSS_COLD_ACQ_TIMEOUT, 60U);
	fake_config_store->write_param(ParamID::GNSS_HDOPFILT_EN, false);
	fake_config_store->write_param(ParamID::GNSS_HACCFILT_EN, false);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, false);
	BaseGNSSFixMode fix_mode = BaseGNSSFixMode::FIX_2D;
	fake_config_store->write_param(ParamID::GNSS_FIX_MODE, fix_mode);
	BaseGNSSDynModel dyn_model = BaseGNSSDynModel::SEA;
	fake_config_store->write_param(ParamID::GNSS_DYN_MODEL, dyn_model);

	// MOORED_DETECT_EN left at its false default; MOORED_DLOC deliberately set
	// to a value that would be glaringly visible if it ever leaked through.
	fake_config_store->write_param(ParamID::MOORED_DLOC, 3600U);
	fake_config_store->write_param(ParamID::MOORED_ENTER_FIXES, 2U);

	fake_rtc->settime(1580083200);

	GPSService s(*mock_m10q, fake_log);
	s.start([](ServiceEvent &e) { ServiceManager::notify_peer_event(e); });

	mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
	increment_time_s(FIRST_AQPERIOD);
	mock().expectOneCall("power_off").onObject(mock_m10q);
	mock_m10q->notify_gnss_data(fake_rtc->gettime(), 10, 10);

	unsigned int offset = FIRST_AQPERIOD;
	for (int i = 0; i < 3; i++) {
		mock().expectOneCall("power_on").onObject(mock_m10q).ignoreOtherParameters();
		increment_time_s(dloc_under_way - offset);
		mock().expectOneCall("power_off").onObject(mock_m10q);
		mock_m10q->notify_gnss_data(fake_rtc->gettime(), 10, 10);
		offset = 0;
	}
	CHECK_FALSE(MooredModeService::is_moored());
}
