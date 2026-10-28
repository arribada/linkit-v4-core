// The host's copy of the SMD module's TCXO warm-up register: when the driver
// must write it before a transmission, and when it may skip the write.

#include "smd_sat/smd_tcxo_register.hpp"

#include "CppUTest/TestHarness.h"

TEST_GROUP(SmdTcxoRegister){};

// Nothing is known before the first write: the module boots with its own
// default (2000 ms in its firmware), which the host must never assume.
TEST(SmdTcxoRegister, UnknownUntilWritten) {
	SmdTcxoRegister r;
	CHECK_FALSE(r.known());
	CHECK_TRUE(r.needs(0));
	CHECK_TRUE(r.needs(2000));
}

// A powered module that already holds the value needs no write; another value does.
TEST(SmdTcxoRegister, SkipsTheWriteOnlyForTheValueHeld) {
	SmdTcxoRegister r;
	r.written(2000);
	CHECK_FALSE(r.needs(2000));
	CHECK_TRUE(r.needs(0));
	CHECK_EQUAL(2000U, r.value_ms());
}

// A power-up, a reset or a STANDBY exit erases the module's RAM: write again.
TEST(SmdTcxoRegister, ForgetMakesTheNextTxWrite) {
	SmdTcxoRegister r;
	r.written(2000);
	r.forget();
	CHECK_TRUE(r.needs(2000));
}

// A surfacing as the driver lives it, the rail cut after every TX: the first
// TX after the dive at 0, the next ones at the configured 2 s. Each follows a
// power-up, so each writes -- and nothing is written after a TX. With the
// module kept powered (LPM), only a change of value writes.
TEST(SmdTcxoRegister, SurfacingWritesOncePerPowerUpAndPerChange) {
	SmdTcxoRegister r;
	unsigned int writes = 0;
	auto tx = [&](uint32_t want_ms) {
		if (r.needs(want_ms)) {
			r.written(want_ms);
			writes++;
		}
	};

	r.forget();
	tx(0);  // power-up, first TX of the surfacing: warm-up 0
	r.forget();
	tx(2000);  // rail cut, power-up: the configured warm-up
	r.forget();
	tx(2000);  // cut again: the module lost it
	CHECK_EQUAL(3U, writes);

	tx(2000);  // kept powered, same value: no write
	CHECK_EQUAL(3U, writes);
	tx(0);  // kept powered, first TX after the next dive: the value changes
	CHECK_EQUAL(4U, writes);
}
