/**
 * @file smd_tcxo_register.hpp
 * @brief What the SMD module's TCXO warm-up register holds, as far as the host knows.
 */

#pragma once

#include <cstdint>

/// @brief The host's copy of the SMD module's TCXO warm-up register.
///
/// The module keeps the warm-up in RAM (argos-smd-at-kineis-firmware,
/// mcu_misc.c: tcxo_warmup_time_ms, 2000 ms at boot) and waits it, blocking,
/// before EVERY transmission. A power-up, a reset or a STANDBY exit (a cold
/// reset of the STM32WL) brings back that default. So the host writes the
/// value the next TX needs after any of them, and otherwise only when that
/// value changes -- the first TX of a surfacing goes out at 0, the next ones at
/// ARGOS_TCXO_WARMUP_TIME.
///
/// Until 2026-10 the driver also wrote the configured value back at the end of
/// the first TX of each power-up, then cut the rail: ~0.3 s of host time per
/// TX (a 50 ms busy-wait and an SPI write) for a value the cut erased at once.
class SmdTcxoRegister {
public:
	/// @brief The module lost its RAM: power-up, reset, STANDBY exit, error.
	void forget() { m_known = false; }

	/// @brief True when the module must be written before a TX needing want_ms.
	bool needs(uint32_t want_ms) const { return !m_known || m_value_ms != want_ms; }

	/// @brief The module acknowledged want_ms.
	void written(uint32_t ms) {
		m_value_ms = ms;
		m_known = true;
	}

	bool known() const { return m_known; }
	uint32_t value_ms() const { return m_value_ms; }

private:
	bool m_known = false;
	uint32_t m_value_ms = 0;
};
