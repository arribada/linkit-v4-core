#pragma once

/**
 * @file nrf_battery_mon.hpp
 * @brief nRF52840 SAADC-based battery voltage monitor.
 *
 * Reads battery voltage through an analog voltage divider via the nRF SAADC,
 * converts to millivolts using a BSP-defined gain, and maps to a 0-100% SOC
 * via a per-chemistry lookup table.  SOC hysteresis prevents rapid state
 * toggling near the critical/low thresholds.
 *
 * Filtered voltage and SOC are persisted in .noinit RAM (CRC-protected) so
 * they survive a soft reset — this avoids false "battery OK" readings on
 * EXTERNAL_WAKEUP boards that reboot frequently.
 */

#include "battery.hpp"

/// @brief Supported cell chemistries — each has its own voltage range and discharge LUT.
/// Selected at compile time via -DBATTERY_CHEMISTRY=<enum_name> (default NCR18650).
enum BatteryChemistry {
	BATT_CHEM_S18650_2600,         ///< Sony/Murata US18650VTC5 (Li-ion, 3.2-4.2 V)
	BATT_CHEM_CGR18650_2250,       ///< Panasonic CGR18650 (Li-ion, 3.2-4.2 V)
	BATT_CHEM_NCR18650_3100_3400,  ///< Panasonic NCR18650B (Li-ion, 3.2-4.2 V) — default
	BATT_CHEM_LS17500_2P,          ///< 2× Saft LS17500 in parallel (Li-SOCl2, 2.7-3.7 V)
	BATT_CHEM_ALKALINE_3S2P        ///< 6× Energizer EN95 (LR20 / D) in 3S2P (alkaline, 2.7-4.7 V)
};

class NrfBatteryMonitor : public BatteryMonitor {
private:
	BatteryChemistry m_chem;
	uint8_t m_adc_channel;
	bool m_is_init;

	/// @brief Sample the SAADC and return raw voltage in mV (before divider correction).
	float sample_adc();

	/// @brief Apply voltage divider gain to raw ADC millivolts.
	uint16_t convert_voltage(float adc_mv);

	/// @brief Map millivolts to 0-100% SOC via the chemistry-specific LUT.
	uint8_t convert_level(uint16_t mv);

	void internal_update() override;

	/// @brief Is this reading physically possible for the fitted chemistry?
	///
	/// Guards against a failed conversion (0 mV, which would otherwise be read
	/// as a flat battery and trip the critical shutdown) and against a rail
	/// glitch during a TX burst. Deliberately generous: the point is to reject
	/// the impossible, not to second-guess a battery that is genuinely dying.
	bool is_plausible(uint16_t mv) const;

	/// @brief Consecutive samples rejected by is_plausible().
	///
	/// A rejected sample leaves the previously reported voltage in place, which
	/// is right for a one-off glitch and wrong for a broken ADC — the tag would
	/// report a healthy battery for ever with nothing in the log after the first
	/// line. Counted so the warning escalates instead of repeating.
	unsigned int m_consecutive_rejects = 0;

	/// @brief Rejections before the warning is escalated to an error.
	static constexpr unsigned int MAX_CONSECUTIVE_REJECTS = 5;

public:
	NrfBatteryMonitor(uint8_t adc_channel, BatteryChemistry chem = BATT_CHEM_LS17500_2P, uint8_t critical_level = 5,
	                  uint8_t low_level = 10);
};
