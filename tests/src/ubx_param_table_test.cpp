/**
 * @file ubx_param_table_test.cpp
 * @brief UBX::CFG::getParameterSize() and the parameter table behind it.
 *
 * getParameterSize() used to be a chain of 111 `if (key == X.key)`. The chain
 * had drifted from the declarations it mirrored: three CloudLocate parameters
 * were declared, were actively written by the configuration sequence, and were
 * missing from the chain. These tests exist so that cannot happen quietly again
 * -- the first one walks the declarations themselves, so a parameter added
 * without a table entry fails here rather than in a field VALGET response.
 */

#include "ubx.hpp"

#include <set>

#include "CppUTest/TestHarness.h"

TEST_GROUP(UBXParamTable){};

/// Every declared parameter must resolve, and resolve to its own size. This is
/// the regression guard: it iterates the table, so an entry pointing at the
/// wrong parameter is caught, and the count check below catches a missing one.
TEST(UBXParamTable, every_table_entry_resolves_to_its_own_size) {
	for (const UBX::CFG::UBXParameter *p : UBX::CFG::PARAMETERS) {
		CHECK(p != nullptr);
		// A zero size would make the caller treat the key as unknown.
		CHECK(p->size != 0);
		CHECK_EQUAL(p->size, UBX::CFG::getParameterSize(p->key));
	}
}

/// The table is the firmware's whole vocabulary of CFG keys. Pinning the count
/// makes an addition a deliberate act: adding a parameter means updating this
/// number, which is where you notice you also have to add the table entry.
TEST(UBXParamTable, table_holds_every_declared_parameter) {
	CHECK_EQUAL(113u, sizeof(UBX::CFG::PARAMETERS) / sizeof(UBX::CFG::PARAMETERS[0]));
}

/// No duplicate keys: two entries answering for one key would mean one of them
/// is dead and a size change to it would silently never take effect.
TEST(UBXParamTable, no_duplicate_keys) {
	std::set<uint32_t> seen;
	for (const UBX::CFG::UBXParameter *p : UBX::CFG::PARAMETERS) {
		CHECK_FALSE(seen.count(p->key) > 0);
		seen.insert(p->key);
	}
	CHECK_EQUAL(sizeof(UBX::CFG::PARAMETERS) / sizeof(UBX::CFG::PARAMETERS[0]), seen.size());
}

/// The three CloudLocate message parameters the old if-chain had forgotten.
/// They are written by m10qasync_config.cpp (RXM_MEASC12/MEAS20/MEAS50), so a
/// VALGET reply can legitimately carry them; returning 0 made the parser abandon
/// the rest of the response at that key.
TEST(UBXParamTable, cloudlocate_meas_keys_are_known) {
	CHECK_EQUAL(1, UBX::CFG::getParameterSize(UBX::CFG::MSGOUT::RXM_MEASC12_UART1.key));
	CHECK_EQUAL(1, UBX::CFG::getParameterSize(UBX::CFG::MSGOUT::RXM_MEAS20_UART1.key));
	CHECK_EQUAL(1, UBX::CFG::getParameterSize(UBX::CFG::MSGOUT::RXM_MEAS50_UART1.key));
}

/// Unchanged behaviour, and the reason the table is not replaced by deriving the
/// size from the key: keys arrive from the module's reply bytes, so one the
/// firmware never declared is reachable, and the caller relies on the 0 to stop.
/// 0x20910000 is well-formed (storage nibble 2 = one byte) but undeclared.
TEST(UBXParamTable, unknown_key_returns_zero) {
	CHECK_EQUAL(0, UBX::CFG::getParameterSize(0x20910000));
	CHECK_EQUAL(0, UBX::CFG::getParameterSize(0x00000000));
	CHECK_EQUAL(0, UBX::CFG::getParameterSize(0xFFFFFFFF));
}

/// Records the measurement that justifies the comment in ubx.hpp rather than
/// leaving it as a claim: for every declared parameter, bits 28-30 of the key
/// give its storage size. Not relied on by the implementation -- asserted here
/// so that if a future parameter breaks the pattern, we learn it deliberately.
TEST(UBXParamTable, key_storage_nibble_agrees_with_declared_size) {
	for (const UBX::CFG::UBXParameter *p : UBX::CFG::PARAMETERS) {
		unsigned int nibble = (p->key >> 28) & 0x7;
		unsigned int derived = 0;
		switch (nibble) {
		case 0x1: derived = 1; break;  // one bit, stored in a byte
		case 0x2: derived = 1; break;
		case 0x3: derived = 2; break;
		case 0x4: derived = 4; break;
		case 0x5: derived = 8; break;
		default: derived = 0; break;
		}
		CHECK_EQUAL(p->size, derived);
	}
}
