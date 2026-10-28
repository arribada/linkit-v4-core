/**
 * @file rspb_recovery_test.cpp
 * @brief The TPL5111 halves of two July-audit fixes (fix-investigation, ported
 * 2026-10-28). Compiled into RSPBSimulation only: the one host build with
 * EXTERNAL_WAKEUP, where the duty-cycle parameters exist.
 */

#include "config_store_fs.hpp"
#include "fake_battery_mon.hpp"
#include "fake_config_store.hpp"

#include "CppUTest/TestHarness.h"

extern FileSystem *main_filesystem;
extern BatteryMonitor *battery_monitor;

namespace {
constexpr unsigned int RAM_BLOCK_COUNT = 256;
constexpr unsigned int RAM_BLOCK_SIZE = 64 * 1024;
constexpr unsigned int RAM_PAGE_SIZE = 256;
}  // namespace

TEST_GROUP(RSPBRecovery) {
	RamFlash *ram_flash;
	LFSFileSystem *fs;
	FakeBatteryMonitor *battery;

	void setup() {
		battery = new FakeBatteryMonitor;
		battery_monitor = battery;
		ram_flash = new RamFlash(RAM_BLOCK_COUNT, RAM_BLOCK_SIZE, RAM_PAGE_SIZE);
		fs = new LFSFileSystem(ram_flash);
		fs->format();
		fs->mount();
		main_filesystem = fs;
	}

	void teardown() {
		fs->umount();
		delete fs;
		main_filesystem = nullptr;
		delete ram_flash;
		battery_monitor = nullptr;
		delete battery;
	}
};

// C2b: below 2 there is no duty cycle, every wake runs. The check said "allowing
// boot" and returned false, which powered the board down.
TEST(RSPBRecovery, ModuloBelowTwoRunsEveryWake) {
	FakeConfigurationStore store;
	store.init();
	for (unsigned int modulo = 0; modulo < 2; modulo++) {
		store.write_param(ParamID::BOOT_COUNTER_MODULO, modulo);
		for (unsigned int counter = 1; counter <= 4; counter++)
			CHECK_TRUE(store.boot_count_check_modulo(counter));
	}
	// A real modulo still skips the wakes that are not its turn.
	unsigned int modulo = 4;
	store.write_param(ParamID::BOOT_COUNTER_MODULO, modulo);
	CHECK_FALSE(store.boot_count_check_modulo(1));
	CHECK_TRUE(store.boot_count_check_modulo(4));
}

// H6: the boot-fail recovery keeps the duty cycle and the wake limit; FACTR does not.
TEST(RSPBRecovery, RecoveryResetKeepsTheDutyCycle) {
	LFSConfigurationStore *store = new LFSConfigurationStore(*fs);
	store->init();
	const unsigned int def_timer = store->read_param<unsigned int>(ParamID::SHUTDOWN_TIMER);
	const unsigned int def_modulo = store->read_param<unsigned int>(ParamID::BOOT_COUNTER_MODULO);
	const unsigned int def_period = store->read_param<unsigned int>(ParamID::WAKEUP_PERIOD);
	const unsigned int timer = def_timer + 600U, modulo = def_modulo + 2U, period = def_period + 900U;

	for (int recovery = 1; recovery >= 0; recovery--) {
		store->write_param(ParamID::SHUTDOWN_TIMER, timer);
		store->write_param(ParamID::BOOT_COUNTER_MODULO, modulo);
		store->write_param(ParamID::WAKEUP_PERIOD, period);
		store->save_params();
		if (recovery)
			store->factory_reset_for_recovery();
		else
			store->factory_reset();
		delete store;
		store = new LFSConfigurationStore(*fs);
		store->init();

		const bool kept = recovery != 0;
		CHECK_EQUAL(kept ? timer : def_timer, store->read_param<unsigned int>(ParamID::SHUTDOWN_TIMER));
		CHECK_EQUAL(kept ? modulo : def_modulo, store->read_param<unsigned int>(ParamID::BOOT_COUNTER_MODULO));
		CHECK_EQUAL(kept ? period : def_period, store->read_param<unsigned int>(ParamID::WAKEUP_PERIOD));
	}
	delete store;
}
