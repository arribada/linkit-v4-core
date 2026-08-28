/**
 * @file dte_params.hpp
 * @brief DTE parameter map — maps ParamID to name, key, encoding, constraints.
 */

#pragma once

#include "base_types.hpp"

/// @brief Parameter value with its ParamID (used by PARMW command).
struct ParamValue {
	ParamID param;
	BaseType value;
};

extern const BaseMap param_map[];
extern const size_t param_map_size;
