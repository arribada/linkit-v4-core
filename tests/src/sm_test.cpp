#include "gentracker.hpp"
#include "ledsm.hpp"
#include "debug.hpp"

#include "fake_switch.hpp"
#include "fake_reed_switch.hpp"
#include "fake_rgb_led.hpp"
#include "fake_timer.hpp"
#include "fake_sws.hpp"
#include "fake_led.hpp"

#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

#include "mock_ble_serv.hpp"
#include "mock_config_store.hpp"
#include "mock_fs.hpp"
#include "mock_location.hpp"
#include "mock_argos_tx.hpp"
#include "mock_logger.hpp"
#include "mock_timer.hpp"
#include "fake_battery_mon.hpp"
#include "mock_ota_file_updater.hpp"
#include "scheduler.hpp"
#include "dte_handler.hpp"
#include "fake_timer.hpp"
#include "bsp.hpp"
#include "reed.hpp"

#define BLOCK_COUNT (256)
#define BLOCK_SIZE  (64 * 1024)
#define PAGE_SIZE   (256)

// BSP settings
namespace BSP {
const WDT_InitTypeDefAndInst_t WDT_Inits[WDT_TOTAL_NUMBER] = { { .config = { .reload_value = 10 * 60 * 1000 } } };
}

// These are defined by main.cpp
extern FileSystem *main_filesystem;
extern Timer *system_timer;
extern ConfigurationStore *configuration_store;
extern DTEHandler *dte_handler;
extern Scheduler *system_scheduler;
extern BLEService *ble_service;
extern OTAFileUpdater *ota_updater;
extern RGBLed *status_led;
extern Switch *saltwater_switch;
extern ReedSwitch *reed_switch;
extern BatteryMonitor *battery_monitor;

// FSM initial state -> BootState
FSM_INITIAL_STATE(GenTracker, BootState)

using fsm_handle = GenTracker;

TEST_GROUP(Sm) {
	FakeReedSwitch *fake_reed_switch;
	FakeSWS *fake_saltwater_switch;
	FakeSwitch *dummy_switch;
	FakeRGBLed *fake_status_led;
	FakeTimer *fake_timer;
	FakeBatteryMonitor *fake_battery_monitor;
	MockOTAFileUpdater *mock_ota_file_updater;
	MockBLEService *mock_ble_service;
	MockLog *sensor_log;
	MockLog *system_log;
	MockLocationScheduler *location_scheduler;
	MockArgosTxService *comms_scheduler;
	MockConfigurationStore *mock_config_store;

	void setup() {
		main_filesystem = new MockFileSystem;
		system_log = new MockLog("System");
		sensor_log = new MockLog("GPS");
		fake_timer = new FakeTimer;
		system_timer = fake_timer;
		mock_config_store = new MockConfigurationStore;
		configuration_store = mock_config_store;
		location_scheduler = new MockLocationScheduler;
		comms_scheduler = new MockArgosTxService;
		mock_ota_file_updater = new MockOTAFileUpdater;
		ota_updater = mock_ota_file_updater;
		dte_handler = new DTEHandler;
		system_scheduler = new Scheduler(system_timer);
		mock_ble_service = new MockBLEService;
		ble_service = mock_ble_service;
		fake_battery_monitor = new FakeBatteryMonitor;
		battery_monitor = fake_battery_monitor;
		fake_status_led = new FakeRGBLed("STATUS");
		status_led = fake_status_led;
		fake_saltwater_switch = new FakeSWS;
		dummy_switch = new FakeSwitch();
		fake_reed_switch = new FakeReedSwitch(*dummy_switch);
		reed_switch = fake_reed_switch;
	}

	void teardown() {
		delete main_filesystem;
		main_filesystem = nullptr;
		delete location_scheduler;
		// ~ReedSwitch::stop() calls system_scheduler->cancel_task() and m_switch.stop(),
		// so the reed switch must be torn down while BOTH the scheduler and dummy_switch
		// are still alive — destroy it first to avoid a use-after-free (SIGSEGV in
		// Scheduler::cancel_task walking the freed scheduler's task list).
		delete fake_reed_switch;
		delete system_scheduler;
		delete comms_scheduler;
		delete mock_config_store;
		delete mock_ble_service;
		delete mock_ota_file_updater;
		delete fake_battery_monitor;
		delete fake_status_led;
		delete dummy_switch;
		delete fake_saltwater_switch;
		delete fake_timer;
		delete sensor_log;
		delete system_log;
	}
};

TEST(Sm, CheckBootFileSystemMountOk) {
	mock().expectOneCall("create").onObject(sensor_log);
	mock().expectOneCall("create").onObject(system_log);
	mock().expectOneCall("num_entries").onObject(sensor_log).andReturnValue(0);
	mock().expectOneCall("num_entries").onObject(system_log).andReturnValue(0);
	mock().expectOneCall("init").onObject(configuration_store);
	mock().expectOneCall("is_mounted").onObject(main_filesystem).andReturnValue(false);
	mock().expectOneCall("mount").onObject(main_filesystem).andReturnValue(0);
	fsm_handle::start();
	CHECK_TRUE(fsm_handle::is_in_state<BootState>());
	CHECK_EQUAL((int)RGBLedColor::WHITE, (int)fake_status_led->get_state());
	CHECK_TRUE(status_led->is_flashing());
}

TEST(Sm, CheckBootFileSystemFirstMountFail) {
	mock().expectOneCall("create").onObject(sensor_log);
	mock().expectOneCall("create").onObject(system_log);
	mock().expectOneCall("num_entries").onObject(sensor_log).andReturnValue(0);
	mock().expectOneCall("num_entries").onObject(system_log).andReturnValue(0);
	mock().expectOneCall("init").onObject(configuration_store);
	mock().expectOneCall("is_mounted").onObject(main_filesystem).andReturnValue(false);
	mock().expectOneCall("mount").onObject(main_filesystem).andReturnValue(-1);
	mock().expectOneCall("format").onObject(main_filesystem).andReturnValue(0);
	mock().expectOneCall("mount").onObject(main_filesystem).andReturnValue(0);
	fsm_handle::start();
	CHECK_TRUE(fsm_handle::is_in_state<BootState>());
}

TEST(Sm, CheckBootFileSystemSecondMountFailAndEnterErrorState) {
	mock().expectOneCall("is_mounted").onObject(main_filesystem).andReturnValue(false);
	mock().expectOneCall("mount").onObject(main_filesystem).andReturnValue(-1);
	mock().expectOneCall("format").onObject(main_filesystem).andReturnValue(0);
	mock().expectOneCall("mount").onObject(main_filesystem).andReturnValue(-1);
	fsm_handle::start();
	CHECK_TRUE(fsm_handle::is_in_state<BootState>());
	system_scheduler->run();
	CHECK_TRUE(fsm_handle::is_in_state<ErrorState>());
}

TEST(Sm, CheckBootFileSystemFormatFailAndEnterErrorState) {
	mock().expectOneCall("is_mounted").onObject(main_filesystem).andReturnValue(false);
	mock().expectOneCall("mount").onObject(main_filesystem).andReturnValue(-1);
	mock().expectOneCall("format").onObject(main_filesystem).andReturnValue(-1);
	fsm_handle::start();
	CHECK_TRUE(fsm_handle::is_in_state<BootState>());
	system_scheduler->run();
	CHECK_TRUE(fsm_handle::is_in_state<ErrorState>());
}

TEST(Sm, CheckTransitionToPreOperationalState) {
	mock().expectOneCall("create").onObject(sensor_log);
	mock().expectOneCall("create").onObject(system_log);
	mock().expectOneCall("num_entries").onObject(sensor_log).andReturnValue(0);
	mock().expectOneCall("num_entries").onObject(system_log).andReturnValue(0);
	mock().expectOneCall("init").onObject(configuration_store);
	mock().expectOneCall("is_mounted").onObject(main_filesystem).andReturnValue(false);
	mock().expectOneCall("mount").onObject(main_filesystem).andReturnValue(0);
	fsm_handle::start();

	fake_timer->set_counter(1000);
	system_scheduler->run();
	CHECK_TRUE(fsm_handle::is_in_state<PreOperationalState>());
	CHECK_FALSE(fake_saltwater_switch->is_started());
}

TEST(Sm, CheckTransitionToOperationalConfigValid) {
	mock().disable();
	fsm_handle::start();

	mock().enable();


	fake_timer->set_counter(1000);
	system_scheduler->run();
	CHECK_TRUE(fsm_handle::is_in_state<PreOperationalState>());
	CHECK_EQUAL((int)RGBLedColor::GREEN, (int)status_led->get_state());
	CHECK_TRUE(status_led->is_flashing());

	// After 5 seconds, transition to operational
	fake_timer->set_counter(6000);
	system_scheduler->run();
	CHECK_TRUE(fsm_handle::is_in_state<OperationalState>());
	CHECK_EQUAL((int)RGBLedColor::BLACK, (int)status_led->get_state());
	CHECK_FALSE(status_led->is_flashing());
	CHECK_TRUE(location_scheduler->is_started());
	CHECK_TRUE(comms_scheduler->is_started());
}


TEST(Sm, CheckTransitionToOperationalConfigValidBatteryLow) {
	mock().disable();
	fsm_handle::start();

	mock().enable();


	fake_battery_monitor->set_values(10, 3300, true, false);
	fake_timer->set_counter(1000);
	system_scheduler->run();
	CHECK_TRUE(fsm_handle::is_in_state<PreOperationalState>());
	CHECK_EQUAL((int)RGBLedColor::YELLOW, (int)status_led->get_state());
	CHECK_TRUE(status_led->is_flashing());

	// After 5 seconds, transition to operational with yellow LED flashing
	fake_timer->set_counter(6000);
	system_scheduler->run();
	CHECK_TRUE(fsm_handle::is_in_state<OperationalState>());
	CHECK_EQUAL((int)RGBLedColor::BLACK, (int)status_led->get_state());
	CHECK_FALSE(status_led->is_flashing());
	CHECK_TRUE(location_scheduler->is_started());
	CHECK_TRUE(comms_scheduler->is_started());
}


TEST(Sm, CheckTransitionToErrorConfigInvalid) {
	mock().disable();

	fsm_handle::start();

	mock().enable();
	mock_config_store->set_is_valid(false);


	fake_timer->set_counter(1000);
	system_scheduler->run();
	CHECK_TRUE(fsm_handle::is_in_state<PreOperationalState>());
	CHECK_TRUE(status_led->is_flashing());
	CHECK_EQUAL((int)RGBLedColor::RED, (int)status_led->get_state());

	// After 5 seconds, transition to error with red LED flashing
	fake_timer->set_counter(6000);
	system_scheduler->run();
	CHECK_TRUE(fsm_handle::is_in_state<ErrorState>());
	CHECK_TRUE(status_led->is_flashing());
	CHECK_EQUAL((int)RGBLedColor::RED, (int)status_led->get_state());

	// Red LED should go off after 5 seconds and then transition to off state with white LED flashing
	fake_timer->set_counter(11000);
	system_scheduler->run();
	CHECK_TRUE(fsm_handle::is_in_state<OffState>());
	CHECK_TRUE(status_led->is_flashing());
	CHECK_EQUAL((int)RGBLedColor::WHITE, (int)status_led->get_state());
}

TEST(Sm, CheckTransitionToConfigurationState) {
	mock().disable();

	fsm_handle::start();
	fake_timer->set_counter(1000);
	system_scheduler->run();
	CHECK_TRUE(fsm_handle::is_in_state<PreOperationalState>());

	mock().enable();
	mock().expectOneCall("set_device_name").onObject(mock_ble_service).withParameter("name", "LinkIt V4 0");
	mock().expectOneCall("start").onObject(mock_ble_service).ignoreOtherParameters();
	// New confirmation gesture: SHORT_HOLD → RELEASE → re-ENGAGE within 3s
	fake_reed_switch->invoke_gesture(ReedSwitchGesture::SHORT_HOLD);
	fake_reed_switch->invoke_gesture(ReedSwitchGesture::RELEASE);
	fake_reed_switch->invoke_gesture(ReedSwitchGesture::ENGAGE);
	system_scheduler->run();
	CHECK_TRUE(fsm_handle::is_in_state<ConfigurationState>());
	CHECK_TRUE(status_led->is_flashing());
	CHECK_EQUAL((int)RGBLedColor::BLUE, (int)status_led->get_state());
}

TEST(Sm, MagnetPresentAtBootArmsHoldGesture) {
	// Regression: a magnet held continuously across boot must arm the
	// SHORT_HOLD/LONG_HOLD gesture timers, otherwise config / power-off cannot
	// be requested without first removing and re-applying the magnet.
	// prime_if_engaged() (used by BootState when the magnet is present at boot)
	// runs the full engage path; the old direct dispatch(ENGAGE) armed nothing.
	mock().disable();

	int engage = 0, short_hold = 0, long_hold = 0, release = 0;
	dummy_switch->set_state(true);  // magnet present at boot
	fake_reed_switch->start([&](ReedSwitchGesture g) {
		switch (g) {
		case ReedSwitchGesture::ENGAGE: engage++; break;
		case ReedSwitchGesture::SHORT_HOLD: short_hold++; break;
		case ReedSwitchGesture::LONG_HOLD: long_hold++; break;
		case ReedSwitchGesture::RELEASE: release++; break;
		default: break;
		}
	});

	// ENGAGE is delivered immediately; the hold timers are now armed.
	fake_reed_switch->prime_if_engaged();
	system_scheduler->run();
	CHECK_EQUAL(1, engage);
	CHECK_EQUAL(0, short_hold);
	CHECK_EQUAL(0, long_hold);

	// SHORT_HOLD fires at 3 s — proof the hold timer was armed by prime.
	fake_timer->set_counter(3000);
	system_scheduler->run();
	CHECK_EQUAL(1, short_hold);
	CHECK_EQUAL(0, long_hold);

	// LONG_HOLD fires at 6 s.
	fake_timer->set_counter(6000);
	system_scheduler->run();
	CHECK_EQUAL(1, long_hold);

	(void)release;
}

TEST(Sm, MagnetPresentAtBootCanEnterConfigViaArmedGesture) {
	// End-to-end: magnet present at boot → BootState primes the gesture → holding
	// to SHORT_HOLD then release + re-engage enters config, WITHOUT the operator
	// first removing and re-applying the magnet. Fails with the old direct
	// dispatch(ENGAGE) in BootState (no hold timer armed → SHORT_HOLD never fires
	// → confirmation never becomes ENTER_CONFIG → stays out of config).
	mock().disable();

	dummy_switch->set_state(true);  // magnet present at boot
	fsm_handle::start();
	fake_timer->set_counter(1000);
	system_scheduler->run();  // BootState → PreOperationalState (Operational transit at 6 s)
	CHECK_TRUE(fsm_handle::is_in_state<PreOperationalState>());

	mock().enable();
	mock().expectOneCall("set_device_name").onObject(mock_ble_service).withParameter("name", "LinkIt V4 0");
	mock().expectOneCall("start").onObject(mock_ble_service).ignoreOtherParameters();

	// Armed SHORT_HOLD fires at 3 s (still PreOp); release + re-engage confirms.
	fake_timer->set_counter(3000);
	system_scheduler->run();
	fake_reed_switch->invoke_gesture(ReedSwitchGesture::RELEASE);
	fake_reed_switch->invoke_gesture(ReedSwitchGesture::ENGAGE);
	system_scheduler->run();
	CHECK_TRUE(fsm_handle::is_in_state<ConfigurationState>());
}

TEST(Sm, ConfirmBlinkNotInterruptedByTransientLed) {
	// 2026-07 regression: while a reed confirmation gesture is pending the LED
	// fast-blinks (BLUE=enter-config, period 50 ms) awaiting the operator's 2nd
	// gesture. That prompt must survive transient/background LED events firing in
	// the same window — the boot white (config triggered right at startup was
	// interrupted by the startup white), a GNSS session/​end flash, an Argos
	// TX, etc. Before the fix any of these stomped the confirm blink and the
	// operator lost the visual cue mid-gesture.
	mock().disable();
	fsm_handle::start();
	fake_timer->set_counter(1000);
	system_scheduler->run();
	CHECK_TRUE(fsm_handle::is_in_state<PreOperationalState>());
	fake_timer->set_counter(6000);
	system_scheduler->run();
	CHECK_TRUE(fsm_handle::is_in_state<OperationalState>());

	// A GNSS session is running in the background (steady CYAN flash).
	ServiceEvent e;
	e.event_source = ServiceIdentifier::GNSS_SENSOR;
	e.event_type = ServiceEventType::SERVICE_ACTIVE;
	e.event_originator_unique_id = 0x12345678;
	ServiceManager::inject_event(e);
	CHECK_EQUAL((int)RGBLedColor::CYAN, (int)status_led->get_state());

	// Operator holds the magnet to enter config → SHORT_HOLD → confirm blink:
	// fast BLUE (period 50 ms), pending the release + re-engage.
	fake_reed_switch->invoke_gesture(ReedSwitchGesture::SHORT_HOLD);
	system_scheduler->run();
	CHECK_TRUE(fsm_handle::is_confirmation_gesture_pending());
	CHECK_EQUAL((int)RGBLedColor::BLUE, (int)status_led->get_state());
	CHECK_TRUE(status_led->is_flashing());
	CHECK_EQUAL(50, (int)fake_status_led->m_period);

	// (1) A background GNSS end-of-session indicator must NOT stomp the blink.
	e.event_type = ServiceEventType::GNSS_OFF_POWEROFF;
	ServiceManager::inject_event(e);
	CHECK_EQUAL((int)RGBLedColor::BLUE, (int)status_led->get_state());
	CHECK_EQUAL(50, (int)fake_status_led->m_period);

	// (2) The literal "boot white" (SetLEDBoot) must NOT stomp it either.
	LEDState::dispatch<SetLEDBoot>({});
	CHECK_EQUAL((int)RGBLedColor::BLUE, (int)status_led->get_state());
	CHECK_EQUAL(50, (int)fake_status_led->m_period);

	// (3) An Argos TX (MAGENTA) must NOT stomp it either.
	LEDState::dispatch<SetLEDArgosTX>({});
	CHECK_EQUAL((int)RGBLedColor::BLUE, (int)status_led->get_state());
	CHECK_EQUAL(50, (int)fake_status_led->m_period);

	// The gesture still resolves normally: release + re-engage enters config,
	// clearing the pending flag so the resolved-state LED (ConfigNotConnected,
	// BLUE flash, default 500 ms period) IS shown — proof the gate re-opens.
	mock().enable();
	mock().expectOneCall("set_device_name").onObject(mock_ble_service).withParameter("name", "LinkIt V4 0");
	mock().expectOneCall("start").onObject(mock_ble_service).ignoreOtherParameters();
	fake_reed_switch->invoke_gesture(ReedSwitchGesture::RELEASE);
	fake_reed_switch->invoke_gesture(ReedSwitchGesture::ENGAGE);
	system_scheduler->run();
	CHECK_TRUE(fsm_handle::is_in_state<ConfigurationState>());
	CHECK_FALSE(fsm_handle::is_confirmation_gesture_pending());
	CHECK_EQUAL((int)RGBLedColor::BLUE, (int)status_led->get_state());
	CHECK_EQUAL(500, (int)fake_status_led->m_period);
}

TEST(Sm, CheckTransitionToOffState) {
	mock().disable();
	fsm_handle::start();
	fake_timer->set_counter(1000);
	system_scheduler->run();
	CHECK_TRUE(fsm_handle::is_in_state<PreOperationalState>());

	// New confirmation gesture: LONG_HOLD → RELEASE → re-ENGAGE within 3s
	fake_reed_switch->invoke_gesture(ReedSwitchGesture::LONG_HOLD);
	fake_reed_switch->invoke_gesture(ReedSwitchGesture::RELEASE);
	fake_reed_switch->invoke_gesture(ReedSwitchGesture::ENGAGE);
	system_scheduler->run();

	CHECK_TRUE(fsm_handle::is_in_state<OffState>());
	CHECK_EQUAL((int)RGBLedColor::WHITE, (int)status_led->get_state());
	CHECK_TRUE(status_led->is_flashing());
	CHECK_EQUAL(50, fake_status_led->m_period);

	// Continue to hold for 5 more seconds
	mock().enable();
	mock().expectOneCall("powerdown");
	fake_timer->set_counter(16000);
	system_scheduler->run();
	CHECK_EQUAL((int)RGBLedColor::BLACK, (int)status_led->get_state());
}

TEST(Sm, CheckOffStateCanBeCancelled) {}

TEST(Sm, CheckBLEInactivityTimeout) {
	mock().disable();
	fsm_handle::start();
	fake_timer->set_counter(1000);
	system_scheduler->run();
	CHECK_TRUE(fsm_handle::is_in_state<PreOperationalState>());

	// Confirmation gesture: SHORT_HOLD → RELEASE → re-ENGAGE within 3s
	mock().enable();
	mock().expectOneCall("set_device_name").onObject(mock_ble_service).withParameter("name", "LinkIt V4 0");
	mock().expectOneCall("start").onObject(mock_ble_service).ignoreOtherParameters();
	fake_reed_switch->invoke_gesture(ReedSwitchGesture::SHORT_HOLD);
	fake_reed_switch->invoke_gesture(ReedSwitchGesture::RELEASE);
	fake_reed_switch->invoke_gesture(ReedSwitchGesture::ENGAGE);
	system_scheduler->run();
	CHECK_TRUE(fsm_handle::is_in_state<ConfigurationState>());
	mock().disable();

	// Wait until BLE inactivity timeout (20 minutes = 1,200,000 ms).
	// Sealed-device hardening (audit 2026-07): the timeout now returns to
	// operation (via PreOperationalState, exactly like the normal exit-config
	// gesture) instead of OffState — transiting to OffState would System-OFF a
	// sealed LinkIt whose operator forgot the exit gesture before release.
	fake_timer->set_counter(1201000);
	system_scheduler->run();
	CHECK_TRUE(fsm_handle::is_in_state<PreOperationalState>());
}


TEST(Sm, CheckTransitionToConfigurationStateAndVerifyOTAUpdateEvents) {
	mock().disable();

	fsm_handle::start();

	mock().enable();
	mock_config_store->set_is_valid(false);


	fake_timer->set_counter(1000);
	system_scheduler->run();
	CHECK_TRUE(fsm_handle::is_in_state<PreOperationalState>());

	mock().expectOneCall("set_device_name").onObject(mock_ble_service).withParameter("name", "LinkIt V4 0");
	mock().expectOneCall("start").onObject(mock_ble_service).ignoreOtherParameters();
	fake_reed_switch->invoke_gesture(ReedSwitchGesture::SHORT_HOLD);
	fake_reed_switch->invoke_gesture(ReedSwitchGesture::RELEASE);
	fake_reed_switch->invoke_gesture(ReedSwitchGesture::ENGAGE);
	system_scheduler->run();

	// Trigger BLE connected event
	BLEServiceEvent event;
	event.event_type = BLEServiceEventType::CONNECTED;
	mock_ble_service->invoke_event(event);
	CHECK_EQUAL((int)RGBLedColor::BLUE, (int)status_led->get_state());
	CHECK_FALSE(status_led->is_flashing());

	// Trigger OTA_START
	mock().enable();
	event.event_type = BLEServiceEventType::OTA_START;
	event.file_size = 0x12345678;
	event.crc32 = 0x87654321;
	event.file_id = (unsigned int)OTAFileIdentifier::MCU_FIRMWARE;
	mock()
	    .expectOneCall("start_file_transfer")
	    .onObject(mock_ota_file_updater)
	    .withUnsignedIntParameter("file_id", event.file_id)
	    .withUnsignedIntParameter("length", event.file_size)
	    .withUnsignedIntParameter("crc32", event.crc32);
	mock_ble_service->invoke_event(event);

	// Trigger OTA_FILE_DATA
	event.event_type = BLEServiceEventType::OTA_FILE_DATA;
	event.data = (void *)0x12345678;
	event.length = 0x87654321;
	mock()
	    .expectOneCall("write_file_data")
	    .onObject(mock_ota_file_updater)
	    .withParameter("data", event.data)
	    .withParameter("length", event.length);
	mock_ble_service->invoke_event(event);

	// Trigger OTA_ABORT
	event.event_type = BLEServiceEventType::OTA_ABORT;
	mock().expectOneCall("abort_file_transfer");
	mock_ble_service->invoke_event(event);

	// Trigger OTA_END
	event.event_type = BLEServiceEventType::OTA_END;
	mock().expectOneCall("complete_file_transfer").onObject(mock_ota_file_updater);
	mock_ble_service->invoke_event(event);
	mock().expectOneCall("apply_file_update").onObject(mock_ota_file_updater);
	system_scheduler->run();
}


TEST(Sm, CheckSWSEventsDispatchedInOperationalState) {
	mock().disable();
	fsm_handle::start();

	mock().enable();


	fake_timer->set_counter(1000);
	system_scheduler->run();
	CHECK_TRUE(fsm_handle::is_in_state<PreOperationalState>());
	CHECK_EQUAL((int)RGBLedColor::GREEN, (int)status_led->get_state());
	CHECK_TRUE(status_led->is_flashing());

	// After 5 seconds, transition to pre-operational with green LED flashing
	fake_timer->set_counter(6000);
	system_scheduler->run();
	CHECK_TRUE(fsm_handle::is_in_state<OperationalState>());
	CHECK_EQUAL((int)RGBLedColor::BLACK, (int)status_led->get_state());
	CHECK_FALSE(status_led->is_flashing());
	CHECK_TRUE(location_scheduler->is_started());
	CHECK_TRUE(comms_scheduler->is_started());

	// Notify an SWS event and make sure it is dispatched into the schedulers
	fake_saltwater_switch->set_state(true);
	CHECK_TRUE(comms_scheduler->is_underwater_deferred());
	CHECK_TRUE(location_scheduler->is_underwater_deferred());
}


TEST(Sm, CheckGNSSWithFixLedTransitions) {
	mock().disable();
	fsm_handle::start();

	mock().enable();


	fake_timer->set_counter(1000);
	system_scheduler->run();
	CHECK_TRUE(fsm_handle::is_in_state<PreOperationalState>());
	CHECK_EQUAL((int)RGBLedColor::GREEN, (int)status_led->get_state());
	CHECK_TRUE(status_led->is_flashing());

	// After 5 seconds, transition to pre-operational with green LED flashing
	fake_timer->set_counter(6000);
	system_scheduler->run();
	CHECK_TRUE(fsm_handle::is_in_state<OperationalState>());
	CHECK_EQUAL((int)RGBLedColor::BLACK, (int)status_led->get_state());
	CHECK_FALSE(status_led->is_flashing());
	CHECK_TRUE(location_scheduler->is_started());
	CHECK_TRUE(comms_scheduler->is_started());

	// Notify GNSS active
	ServiceEvent e;
	e.event_source = ServiceIdentifier::GNSS_SENSOR;
	e.event_type = ServiceEventType::SERVICE_ACTIVE;
	e.event_originator_unique_id = 0x12345678;
	ServiceManager::inject_event(e);
	CHECK_EQUAL((int)RGBLedColor::CYAN, (int)status_led->get_state());
	CHECK_TRUE(status_led->is_flashing());

	// Notify GNSS logged (valid fix). 2026-05 LED refactor: SERVICE_LOG_UPDATED
	// only latches fix validity — the LED stays CYAN (still acquiring). The
	// GREEN/RED indicator is rendered later by the end-of-session GNSS_OFF_* event.
	GPSLogEntry log;
	log.info.valid = 1;
	e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
	e.event_data = log;
	ServiceManager::inject_event(e);
	CHECK_EQUAL((int)RGBLedColor::CYAN, (int)status_led->get_state());
	CHECK_TRUE(status_led->is_flashing());

	// End of session (full power-off): fix → GREEN fast-blink for ~500 ms.
	e.event_type = ServiceEventType::GNSS_OFF_POWEROFF;
	ServiceManager::inject_event(e);
	CHECK_EQUAL((int)RGBLedColor::GREEN, (int)status_led->get_state());
	CHECK_TRUE(status_led->is_flashing());

	// After the ~500 ms indicator window, the LED returns off.
	fake_timer->set_counter(9000);
	CHECK_EQUAL((int)RGBLedColor::BLACK, (int)status_led->get_state());
	CHECK_FALSE(status_led->is_flashing());
}

TEST(Sm, CheckGNSSWithoutFixLedTransitions) {
	mock().disable();
	fsm_handle::start();

	mock().enable();


	fake_timer->set_counter(1000);
	system_scheduler->run();
	CHECK_TRUE(fsm_handle::is_in_state<PreOperationalState>());
	CHECK_EQUAL((int)RGBLedColor::GREEN, (int)status_led->get_state());
	CHECK_TRUE(status_led->is_flashing());

	// After 5 seconds, transition to pre-operational with green LED flashing
	fake_timer->set_counter(6000);
	system_scheduler->run();
	CHECK_TRUE(fsm_handle::is_in_state<OperationalState>());
	CHECK_EQUAL((int)RGBLedColor::BLACK, (int)status_led->get_state());
	CHECK_FALSE(status_led->is_flashing());
	CHECK_TRUE(location_scheduler->is_started());
	CHECK_TRUE(comms_scheduler->is_started());

	// Notify GNSS active
	ServiceEvent e;
	e.event_source = ServiceIdentifier::GNSS_SENSOR;
	e.event_type = ServiceEventType::SERVICE_ACTIVE;
	e.event_originator_unique_id = 0x12345678;
	ServiceManager::inject_event(e);
	CHECK_EQUAL((int)RGBLedColor::CYAN, (int)status_led->get_state());
	CHECK_TRUE(status_led->is_flashing());

	// Notify GNSS logged (no fix). 2026-05 LED refactor: SERVICE_LOG_UPDATED only
	// latches fix validity — the LED stays CYAN (still acquiring). The GREEN/RED
	// indicator is rendered later by the end-of-session GNSS_OFF_* event.
	GPSLogEntry log;
	log.info.valid = 0;
	e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
	e.event_data = log;
	ServiceManager::inject_event(e);
	CHECK_EQUAL((int)RGBLedColor::CYAN, (int)status_led->get_state());
	CHECK_TRUE(status_led->is_flashing());

	// End of session (full power-off): no fix → RED fast-blink for ~500 ms.
	e.event_type = ServiceEventType::GNSS_OFF_POWEROFF;
	ServiceManager::inject_event(e);
	CHECK_EQUAL((int)RGBLedColor::RED, (int)status_led->get_state());
	CHECK_TRUE(status_led->is_flashing());

	// After the ~500 ms indicator window, the LED returns off.
	fake_timer->set_counter(9000);
	CHECK_EQUAL((int)RGBLedColor::BLACK, (int)status_led->get_state());
	CHECK_FALSE(status_led->is_flashing());
}


TEST(Sm, CheckArgosTXLedTransitions) {
	mock().disable();
	fsm_handle::start();

	mock().enable();


	fake_timer->set_counter(1000);
	system_scheduler->run();
	CHECK_TRUE(fsm_handle::is_in_state<PreOperationalState>());
	CHECK_EQUAL((int)RGBLedColor::GREEN, (int)status_led->get_state());
	CHECK_TRUE(status_led->is_flashing());

	// After 5 seconds, transition to pre-operational with green LED flashing
	fake_timer->set_counter(6000);
	system_scheduler->run();
	CHECK_TRUE(fsm_handle::is_in_state<OperationalState>());
	CHECK_EQUAL((int)RGBLedColor::BLACK, (int)status_led->get_state());
	CHECK_FALSE(status_led->is_flashing());
	CHECK_TRUE(location_scheduler->is_started());
	CHECK_TRUE(comms_scheduler->is_started());

	// Notify Argos TX active
	ServiceEvent e;
	e.event_source = ServiceIdentifier::ARGOS_TX;
	e.event_type = ServiceEventType::SERVICE_ACTIVE;
	e.event_originator_unique_id = 0x12345678;
	ServiceManager::inject_event(e);
	CHECK_EQUAL((int)RGBLedColor::MAGENTA, (int)status_led->get_state());
	CHECK_FALSE(status_led->is_flashing());

	// Notify Argos TX inactive
	e.event_type = ServiceEventType::SERVICE_INACTIVE;
	ServiceManager::inject_event(e);
	CHECK_EQUAL((int)RGBLedColor::BLACK, (int)status_led->get_state());
	CHECK_FALSE(status_led->is_flashing());
}

TEST(Sm, TriggerBatteryCriticalStateInOperationalState) {
	mock().disable();
	fsm_handle::start();

	mock().enable();


	fake_timer->set_counter(1000);
	system_scheduler->run();
	CHECK_TRUE(fsm_handle::is_in_state<PreOperationalState>());
	CHECK_EQUAL((int)RGBLedColor::GREEN, (int)status_led->get_state());
	CHECK_TRUE(status_led->is_flashing());

	// After 5 seconds, transition to pre-operational with green LED flashing
	fake_timer->set_counter(6000);
	system_scheduler->run();
	CHECK_TRUE(fsm_handle::is_in_state<OperationalState>());
	CHECK_EQUAL((int)RGBLedColor::BLACK, (int)status_led->get_state());
	CHECK_FALSE(status_led->is_flashing());
	CHECK_TRUE(location_scheduler->is_started());
	CHECK_TRUE(comms_scheduler->is_started());

	// Set battery critical
	fake_battery_monitor->set_values(5, 2000, true, true);
	battery_monitor->update();
	fake_timer->set_counter(7000);
	system_scheduler->run();
	CHECK_TRUE(fsm_handle::is_in_state<BatteryCriticalState>());
	CHECK_EQUAL((int)RGBLedColor::YELLOW, (int)status_led->get_state());
	CHECK_FALSE(status_led->is_flashing());

	// Wait 2 minutes for transition to off state
	fake_timer->set_counter(7000 + 120000);
	system_scheduler->run();
	CHECK_TRUE(fsm_handle::is_in_state<OffState>());
}

TEST(Sm, TriggerBatteryCriticalStateInPreOperationalState) {
	mock().disable();
	fsm_handle::start();

	mock().enable();


	fake_battery_monitor->set_values(5, 2000, true, true);
	fake_timer->set_counter(1000);
	system_scheduler->run();
	CHECK_TRUE(fsm_handle::is_in_state<BatteryCriticalState>());
	CHECK_EQUAL((int)RGBLedColor::YELLOW, (int)status_led->get_state());
	CHECK_FALSE(status_led->is_flashing());
}
