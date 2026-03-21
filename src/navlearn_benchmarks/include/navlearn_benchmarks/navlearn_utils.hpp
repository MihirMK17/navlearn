// Copyright 2026 NavLearn Contributors
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cmath>
#include <sstream>
#include <iomanip>
#include "unique_identifier_msgs/msg/uuid.hpp"

namespace navlearn {

/// ref_source values for TrajectoryMetric.msg
constexpr uint8_t REF_NONE = 0;
constexpr uint8_t REF_GROUND_TRUTH = 1;

/// Wrap angle to [-π, π]
inline double wrap_to_pi(double angle)
{
  constexpr double kPi = 3.14159265358979323846;
  while (angle >  kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

/// Convert UUID bytes to readable lowercase hex string (e.g., "a1b2c3d4...")
inline std::string uuid_to_string(const unique_identifier_msgs::msg::UUID & uuid)
{
  std::ostringstream ss;
  ss << std::hex << std::setfill('0');
  for (auto b : uuid.uuid) {
    ss << std::setw(2) << static_cast<int>(b);
  }
  return ss.str();
}

}  // namespace navlearn
