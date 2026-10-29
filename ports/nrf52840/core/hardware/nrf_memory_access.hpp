#pragma once

/**
 * @file nrf_memory_access.hpp
 * @brief nRF52840 RAM bounds-checked memory access for DTE DUMPD command.
 *
 * Validates that the requested address range falls entirely within the
 * nRF52840 256 KB RAM region (0x20000000..0x2003FFFF) before returning
 * a physical pointer.  Used by the DTE handler to safely read device memory.
 */

#include <cstdint>
#include "error.hpp"
#include "memory_access.hpp"

/// @name nRF52840 RAM boundaries
/// @{
static constexpr uint32_t RAM_START_ADDR = 0x20000000;
static constexpr uint32_t RAM_END_ADDR = 0x20040000;  ///< 256 KB RAM
/// @}

class NrfMemoryAccess : public MemoryAccess {
public:
	void *get_physical_address(unsigned int addr, unsigned int length) override {
		// Guard against integer overflow and out-of-range access. The end is
		// tested by subtraction: addr + length wraps for an address near
		// 0xFFFFFFFF (DUMPM takes any 32-bit address) and passed the old test.
		// length is bounded by the RAM size first, so the subtraction cannot
		// underflow (2026-07 audit M3).
		if (addr < RAM_START_ADDR || length == 0 || length > RAM_END_ADDR - RAM_START_ADDR
		    || addr > RAM_END_ADDR - length) {
			throw ILLEGAL_MEMORY_ADDRESS;
		}
		return reinterpret_cast<void *>(addr);
	}
};
