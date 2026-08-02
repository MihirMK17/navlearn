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

#ifndef NAVLEARN_BENCHMARKS__KIDNAP_YAW_HPP_
#define NAVLEARN_BENCHMARKS__KIDNAP_YAW_HPP_

namespace navlearn
{

/// Heading the robot is given by a kidnap teleport.
///
/// The kidnap moves the robot in position only; its heading is carried through unchanged.
/// PROTOCOL.md (frozen 2026-07-30) records the design as "Whether yaw error scales with
/// magnitude, or stays fixed | currently fixed -- one variable moves", and displacement is
/// the sweep's declared independent variable.
///
/// The sampler previously drew this from a uniform distribution over the full circle,
/// independent of the commanded displacement. That made the experiment measure something
/// nobody had declared. On the 2026-08-02 leg 2 rpp arm (120 goals):
///
///     success vs |yaw change|   0-30 deg 100.0%   30-90 deg 35.3%   90-180 deg 26.8%
///     success vs displacement   0-5 cm    26.5%   5-50 cm   44.4%   0.5-3 m   38.7%
///
/// Yaw explained the outcome and distance explained nothing, while every record in the
/// campaign labelled the goal by its distance. Claim 2 -- that recovery is governed by
/// where the robot lands rather than how far it moved -- would have been confirmed by an
/// uncontrolled rotation rather than by landing geometry.
///
/// Kept as a named function rather than an inline literal so the policy has one place to
/// live, one test, and one line to change if the yaw question in PROTOCOL.md is ever
/// reopened with a dated amendment.
///
/// \param reference_yaw the robot's heading immediately before the teleport, radians.
/// \return the heading to teleport to.
inline double kidnapYaw(double reference_yaw)
{
  return reference_yaw;
}

}  // namespace navlearn

#endif  // NAVLEARN_BENCHMARKS__KIDNAP_YAW_HPP_
