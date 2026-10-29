/**
 * @file nrf_memory_access_test.cpp
 * @brief The RAM window DUMPM may read (NrfMemoryAccess) and its end test
 * (July audit M3, ported 2026-10-29). The DTE tests use a fake window; this
 * one checks the real boundary.
 */

#include "nrf_memory_access.hpp"

#include "CppUTest/TestHarness.h"

TEST_GROUP(NrfMemoryAccess) {
	NrfMemoryAccess access;
	const unsigned int dumpm_max = 0x500;  // DUMPM's length limit
};

// Every range inside the 256 KB of RAM is handed back as it is, up to its last byte.
TEST(NrfMemoryAccess, RangesInsideRamAreAccepted) {
	POINTERS_EQUAL(reinterpret_cast<void *>(RAM_START_ADDR), access.get_physical_address(RAM_START_ADDR, 16));
	POINTERS_EQUAL(reinterpret_cast<void *>(RAM_END_ADDR - dumpm_max),
	               access.get_physical_address(RAM_END_ADDR - dumpm_max, dumpm_max));
}

// An address near 0xFFFFFFFF made addr + length wrap below the end of RAM: the
// old test passed it and DUMPM handed the encoder a pointer to unmapped memory.
TEST(NrfMemoryAccess, ARangeThatWrapsOrLeavesRamIsRefused) {
	CHECK_THROWS(ErrorCode, access.get_physical_address(0xFFFFFC00u, dumpm_max));
	CHECK_THROWS(ErrorCode, access.get_physical_address(RAM_END_ADDR - dumpm_max + 1, dumpm_max));
	CHECK_THROWS(ErrorCode, access.get_physical_address(RAM_START_ADDR - 1, 1));
	CHECK_THROWS(ErrorCode, access.get_physical_address(RAM_START_ADDR, 0));
}
