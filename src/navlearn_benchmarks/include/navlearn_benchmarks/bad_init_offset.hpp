// Copyright 2026 Mihir Kulkarni
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

#ifndef NAVLEARN_BENCHMARKS__BAD_INIT_OFFSET_HPP_
#define NAVLEARN_BENCHMARKS__BAD_INIT_OFFSET_HPP_

#include <cmath>
#include <cstdint>
#include <random>

namespace navlearn
{

/// A perturbation applied to a known-good pose before handing it to the localizer.
struct PoseOffset
{
  double dx = 0.0;
  double dy = 0.0;
  double dyaw = 0.0;
};

/// Legacy bad-init draw: dx and dy independent and uniform over [-range, range].
///
/// Kept because the categorical TTC levels ran against it and their results must stay
/// reproducible. Note what its nominal range means: it bounds each AXIS, not the
/// displacement, so a "0.5 m" level reaches 0.707 m on the diagonal and the realised
/// severity depends on the direction the error happens to point. Acceptable for a
/// categorical level whose label is a name; not acceptable for a sweep whose independent
/// variable is the magnitude itself.
inline PoseOffset squareOffset(uint64_t seed, double lin_range_m, double yaw_range_rad)
{
  std::mt19937_64 gen(seed);
  std::uniform_real_distribution<double> lin(-lin_range_m, lin_range_m);
  std::uniform_real_distribution<double> yaw(-yaw_range_rad, yaw_range_rad);

  PoseOffset out;
  out.dx = lin(gen);
  out.dy = lin(gen);
  out.dyaw = yaw(gen);
  return out;
}

/// Continuous-sweep bad-init draw: exactly `magnitude_m` at a uniformly random bearing.
///
/// The realised displacement equals the commanded one by construction, which is what lets
/// the magnitude be the regression's independent variable rather than a label attached to
/// it. Bearing is uniform over the full circle so severity is not confounded with the
/// direction the error points -- in a map with asymmetric free space, a biased bearing
/// would make "more severe" partly mean "pointing at the open room".
///
/// Orientation error keeps its own configured range and does NOT scale with magnitude, so
/// the sweep moves exactly one variable: the curve is a curve in translation error, with
/// orientation error held fixed. If a different reading is wanted -- one severity dial
/// driving both -- that is a deliberate design choice and belongs in PROTOCOL.md, not in
/// a default.
inline PoseOffset polarOffset(uint64_t seed, double magnitude_m, double yaw_range_rad)
{
  std::mt19937_64 gen(seed);
  std::uniform_real_distribution<double> bearing(-M_PI, M_PI);
  std::uniform_real_distribution<double> yaw(-yaw_range_rad, yaw_range_rad);

  const double theta = bearing(gen);

  PoseOffset out;
  // Guarded so the bottom of a linear sweep is an exact no-op in translation rather than
  // a floating-point smear on a condition the design calls unperturbed.
  if (magnitude_m != 0.0) {
    out.dx = magnitude_m * std::cos(theta);
    out.dy = magnitude_m * std::sin(theta);
  }
  out.dyaw = yaw(gen);
  return out;
}

}  // namespace navlearn

#endif  // NAVLEARN_BENCHMARKS__BAD_INIT_OFFSET_HPP_
