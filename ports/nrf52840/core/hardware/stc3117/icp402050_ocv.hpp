#pragma once

/**
 * @file icp402050_ocv.hpp
 * @brief OCV curve of the RSPB cell and the verdicts drawn from it.
 *
 * No hardware here, so that the host tests (RSPBSimulation) check the same
 * verdicts the gauge driver takes.
 */

#include <cstdint>

// ─────────────────────────────────────────────────────────────────────────
// RSPB battery cell — Renata ICP402050 (402050, 3.7 V 420 mAh LiPo)
//
// SINGLE SOURCE OF TRUTH for the OCV curve. Every consumer derives from these
// two arrays so they can never drift:
//   - the STC3117 chip OCV table  (GasGauge_DefaultInit, CUSTOM_BATTERY_OCV)
//   - the voltage-based estimate  (estimate_soc_from_voltage below), used by the
//     sanity check on the gauge's SOC and by the critical verdict
// Rested-OCV deduced from a standard 4.2 V LiCoO2 shape anchored to this cell's
// endpoints (datasheet ICP402050PR rev V02: 4.2 V charge / 3.0 V cutoff @0.2C;
// the datasheet has no discharge graph). Refine ICP402050_OCV_MV[] with measured
// rested-OCV points if you characterize the cell. Cell params (capacity, Rint,
// Rsense, voltages) live in stc311x_BatteryInfo.h.
// ─────────────────────────────────────────────────────────────────────────
static constexpr unsigned int ICP402050_OCV_POINTS = 16;
static const uint16_t ICP402050_OCV_MV[ICP402050_OCV_POINTS] = {
	3300, 3440, 3510, 3560, 3600, 3630, 3655, 3675, 3700, 3740, 3800, 3835, 3875, 3955, 4060, 4180,
};
static const uint8_t ICP402050_OCV_SOC[ICP402050_OCV_POINTS] = {
	0, 3, 6, 10, 15, 20, 25, 30, 40, 50, 60, 65, 70, 80, 90, 100,
};

// STC3117 OCV register LSB = 0.55 mV → reg = round(mV / 0.55) = round(mV * 100 / 55).
static inline uint16_t stc3117_mv_to_ocv_reg(uint16_t mv) {
	return static_cast<uint16_t>((static_cast<uint32_t>(mv) * 100u + 27u) / 55u);
}

// Voltage-based SOC estimate from the single-source ICP402050 OCV curve above.
// Overrides an implausibly high SOC from the STC3117 after a power cycle, and
// vetoes an implausibly low one (soc_and_voltage_critical below). Tracks the
// CUSTOM_BATTERY_OCV profile (the RSPB default).
static inline uint8_t estimate_soc_from_voltage(uint16_t mv) {
	if (mv <= ICP402050_OCV_MV[0]) return 0;
	if (mv >= ICP402050_OCV_MV[ICP402050_OCV_POINTS - 1]) return 100;
	for (unsigned int i = 1; i < ICP402050_OCV_POINTS; i++) {
		if (mv <= ICP402050_OCV_MV[i]) {
			uint32_t range_mv = ICP402050_OCV_MV[i] - ICP402050_OCV_MV[i - 1];
			uint32_t range_soc = ICP402050_OCV_SOC[i] - ICP402050_OCV_SOC[i - 1];
			return ICP402050_OCV_SOC[i - 1]
			       + static_cast<uint8_t>((mv - ICP402050_OCV_MV[i - 1]) * range_soc / range_mv);
		}
	}
	return 100;
}

// Critical needs BOTH the gauge's SOC and the cell voltage below the critical
// level (H3, 2026-07 audit). On RSPB a critical verdict powers the wake down at
// once, and the SOC filtering starts afresh after every TPL5111 power cut, so a
// single low SOC used to be enough. The gauge can report one on a healthy cell
// (OCV not settled at a cold power-up, a SOC restored from its RAM); if it did
// so at every wake the tag went mute. The voltage vetoes that case and still
// lets a genuinely empty cell through: both read low. The reading is taken
// under load, below the rested OCV, so the veto errs towards critical.
static inline bool soc_and_voltage_critical(uint8_t soc, uint16_t mv, uint8_t critical_level) {
	return soc < critical_level && estimate_soc_from_voltage(mv) < critical_level;
}
