// Copyright 2026 NavLearn Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
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
