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
// Tests for how a bad-initialization offset is drawn.
//
// The TTC leg became a continuous sweep in displacement magnitude, which only works if
// the offset a goal receives actually HAS the commanded magnitude. The legacy sampler
// draws dx and dy independently over a square, so its nominal range is a bound rather
// than a displacement and the realised magnitude depends on direction. That is fine for a
// categorical level and wrong for a curve, so both samplers exist and these tests pin the
// difference.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "navlearn_benchmarks/bad_init_offset.hpp"

using navlearn::PoseOffset;
using navlearn::polarOffset;
using navlearn::squareOffset;

namespace
{
constexpr double kYawRange = 0.26;
}

// ------------------------------------------------------------------ legacy square draw

TEST(SquareOffset, ComponentsStayWithinTheConfiguredRange)
{
  for (uint64_t s = 0; s < 500; ++s) {
    const PoseOffset o = squareOffset(s, 0.5, kYawRange);
    EXPECT_LE(std::fabs(o.dx), 0.5);
    EXPECT_LE(std::fabs(o.dy), 0.5);
    EXPECT_LE(std::fabs(o.dyaw), kYawRange);
  }
}

TEST(SquareOffset, RealisedMagnitudeExceedsTheNominalRangeDiagonally)
{
  // Documents the reason the curve leg cannot use this sampler. A "0.5 m" level reaches
  // 0.707 m on the diagonal, so the nominal number is a bound on each axis rather than a
  // displacement, and severity varies with direction.
  double worst = 0.0;
  for (uint64_t s = 0; s < 20000; ++s) {
    const PoseOffset o = squareOffset(s, 0.5, kYawRange);
    worst = std::max(worst, std::hypot(o.dx, o.dy));
  }
  EXPECT_GT(worst, 0.5) << "square sampler never exceeded its nominal range";
  EXPECT_LE(worst, 0.5 * std::sqrt(2.0) + 1e-9);
}

TEST(SquareOffset, IsDeterministicForASeed)
{
  const PoseOffset a = squareOffset(9876, 0.5, kYawRange);
  const PoseOffset b = squareOffset(9876, 0.5, kYawRange);
  EXPECT_DOUBLE_EQ(a.dx, b.dx);
  EXPECT_DOUBLE_EQ(a.dy, b.dy);
  EXPECT_DOUBLE_EQ(a.dyaw, b.dyaw);
}

// ------------------------------------------------------------------- polar curve draw

TEST(PolarOffset, RealisedMagnitudeEqualsTheCommandedMagnitude)
{
  // The property the curve rests on. If the realised displacement did not equal the
  // commanded one, the regression's independent variable would be a label rather than a
  // measurement, and the fitted onset of degradation would be at the wrong place.
  for (uint64_t s = 0; s < 500; ++s) {
    const double m = 0.3 + 0.005 * static_cast<double>(s % 100);
    const PoseOffset o = polarOffset(s, m, kYawRange);
    EXPECT_NEAR(std::hypot(o.dx, o.dy), m, 1e-12) << "seed " << s;
  }
}

TEST(PolarOffset, BearingsCoverTheFullCircle)
{
  // A sampler biased toward one direction would confound severity with the direction the
  // error points, which in a map with asymmetric free space is a real confound.
  const int kBins = 8;
  std::vector<int> counts(kBins, 0);
  const int kDraws = 4000;
  for (int i = 0; i < kDraws; ++i) {
    const PoseOffset o = polarOffset(static_cast<uint64_t>(i), 1.0, kYawRange);
    double bearing = std::atan2(o.dy, o.dx);           // (-pi, pi]
    int bin = static_cast<int>((bearing + M_PI) / (2.0 * M_PI) * kBins);
    bin = std::min(std::max(bin, 0), kBins - 1);
    counts[static_cast<size_t>(bin)]++;
  }
  const double expected = static_cast<double>(kDraws) / kBins;
  for (int b = 0; b < kBins; ++b) {
    EXPECT_NEAR(counts[static_cast<size_t>(b)], expected, expected * 0.25)
      << "bearing bin " << b << " unevenly covered";
  }
}

TEST(PolarOffset, YawStaysWithinItsOwnRangeIndependentOfMagnitude)
{
  // Orientation error is held at its configured range while translation sweeps, so the
  // curve moves exactly one variable. If PROTOCOL.md later chooses to scale yaw with
  // magnitude instead, that is a deliberate change to what the sweep varies -- and this
  // test is what will catch it happening by accident.
  for (uint64_t s = 0; s < 300; ++s) {
    for (double m : {0.3, 1.0, 3.0}) {
      EXPECT_LE(std::fabs(polarOffset(s, m, kYawRange).dyaw), kYawRange);
    }
  }
}

TEST(PolarOffset, IsDeterministicForASeed)
{
  const PoseOffset a = polarOffset(4242, 1.5, kYawRange);
  const PoseOffset b = polarOffset(4242, 1.5, kYawRange);
  EXPECT_DOUBLE_EQ(a.dx, b.dx);
  EXPECT_DOUBLE_EQ(a.dy, b.dy);
  EXPECT_DOUBLE_EQ(a.dyaw, b.dyaw);
}

TEST(PolarOffset, DistinctSeedsGiveDistinctOffsets)
{
  const PoseOffset a = polarOffset(1, 1.0, kYawRange);
  const PoseOffset b = polarOffset(2, 1.0, kYawRange);
  EXPECT_FALSE(a.dx == b.dx && a.dy == b.dy);
}

TEST(PolarOffset, ZeroMagnitudeIsAnExactNoOpInTranslation)
{
  // The bottom of a linear sweep. It must not leak a floating-point smear into a
  // condition the design says is unperturbed in translation.
  const PoseOffset o = polarOffset(77, 0.0, kYawRange);
  EXPECT_DOUBLE_EQ(o.dx, 0.0);
  EXPECT_DOUBLE_EQ(o.dy, 0.0);
}
