#pragma once

/**
 * @file bma400_math.hpp
 * @brief The BMA400's arithmetic: activity metric and calibration average.
 *
 * No hardware here, so that the host tests check what the driver computes.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>

/// Below this total magnitude (g), every axis reads about 0. A sensor at rest
/// always senses ~1 g: this is a stuck bus or a missing sensor, not a still animal.
static constexpr double BMA400_STUCK_G = 0.1;

/// @brief True when the reading cannot come from a working sensor.
static inline bool bma400_reads_stuck(double x, double y, double z) {
	return std::sqrt(x * x + y * y + z * z) < BMA400_STUCK_G;
}

/// @brief Activity metric, 0-255: the deviation of |g| from 1 g, scaled to the range.
///
/// A stuck reading gives 255, stated explicitly (2026-07 audit M3): the mortality
/// design requires that a failed sensor never push a live animal toward a false
/// death. Before, |0 - 1| = 1 g gave 255 at 2 g only by accident of the scaling,
/// and 17 at 16 g, which a raised MORTALITY_ACTIVITY_THRESH would count as still.
static inline uint8_t bma400_activity(double x, double y, double z, uint8_t g_range) {
	if (bma400_reads_stuck(x, y, z)) return 255;
	double activity_g = std::abs(std::sqrt(x * x + y * y + z * z) - 1.0);
	const double max_dev = std::max(1.0, static_cast<double>(g_range) - 1.0);
	if (activity_g > max_dev) activity_g = max_dev;
	return static_cast<uint8_t>((activity_g / max_dev) * 255.0);
}

/// @brief Average of a calibration run, over the samples actually read.
///
/// The driver used to divide by the number of samples asked for: with half the
/// reads timed out, every offset came out halved -- Z at rest then read 1.5 g --
/// and was saved to AXL.CAL. A run that read fewer than half its samples is
/// refused (decided with the user on 2026-10-29): the outputs are left alone, and
/// the caller keeps the previous calibration.
/// @return true if the run is accepted.
static inline bool bma400_calibration_average(double sum_x, double sum_y, double sum_z, unsigned int n_read,
                                              unsigned int n_asked, double &offset_x, double &offset_y,
                                              double &offset_z) {
	if (n_read == 0 || 2 * n_read < n_asked) return false;
	offset_x = sum_x / n_read;
	offset_y = sum_y / n_read;
	offset_z = sum_z / n_read;
	return true;
}
