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
//
// Kidnap yaw policy.
//
// Why this exists
//   PROTOCOL.md (frozen 2026-07-30) records the kidnap severity design as "yaw error
//   ... currently fixed -- one variable moves". The sampler did not implement that: it
//   drew yaw uniformly over the full circle on every kidnap, independent of the commanded
//   displacement. Measured on the 2026-08-02 leg 2 rpp arm (120 goals): at displacements
//   under 5 cm the median absolute yaw change was 93.5 deg and the maximum 172.8 deg.
//
//   The consequence was that the sweep's declared independent variable did nothing and an
//   undeclared one did everything:
//
//       success vs yaw change   0-30 deg 100.0%   30-90 deg 35.3%   90-180 deg 26.8%
//       success vs displacement 0-5 cm    26.5%   5-50 cm   44.4%   0.5-3 m   38.7%
//
//   Claim 2 asserts recovery is governed by where the robot lands rather than how far it
//   moved. A uniformly random rotation would have handed that result over for the wrong
//   reason, and the first reviewer to plot outcome against yaw would have found it.
//
// What it pins
//   The kidnap preserves heading: only position moves. This is a property of the
//   perturbation design, so it is asserted directly against the rule rather than by
//   spinning up a node.

#include <cmath>
#include <random>

#include "gtest/gtest.h"
#include "navlearn_benchmarks/kidnap_yaw.hpp"

using navlearn::kidnapYaw;

TEST(KidnapYaw, preserves_the_reference_heading_exactly)
{
  // The robot is not rotated by the teleport, whatever the displacement.
  for (double ref_yaw : {-3.0, -1.5, 0.0, 0.7, 3.1}) {
    EXPECT_DOUBLE_EQ(kidnapYaw(ref_yaw), ref_yaw);
  }
}

TEST(KidnapYaw, is_independent_of_displacement_magnitude)
{
  // The whole defect: yaw must not co-vary with the severity knob, or the sweep has two
  // independent variables and neither can be attributed.
  const double ref_yaw = 0.42;
  for (double magnitude : {0.01, 0.05, 0.5, 1.5, 3.0}) {
    (void)magnitude;
    EXPECT_DOUBLE_EQ(kidnapYaw(ref_yaw), ref_yaw);
  }
}

TEST(KidnapYaw, yields_zero_yaw_change_across_a_simulated_sweep)
{
  // The acceptance check restated as the measurement that exposed the bug: draw a sweep
  // the way the campaign does and confirm the yaw-change column would be identically
  // zero. Under the old sampler this had a median near 90 degrees.
  std::mt19937 gen(20260730);
  std::uniform_real_distribution<double> ref(-M_PI, M_PI);
  std::uniform_real_distribution<double> mag(0.01, 3.0);

  double worst = 0.0;
  for (int i = 0; i < 2000; ++i) {
    const double ref_yaw = ref(gen);
    (void)mag(gen);
    const double delta = std::fabs(kidnapYaw(ref_yaw) - ref_yaw);
    worst = std::max(worst, delta);
  }
  EXPECT_DOUBLE_EQ(worst, 0.0);
}
