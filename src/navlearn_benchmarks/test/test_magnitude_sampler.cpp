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
// Tests for continuous perturbation-magnitude sampling.
//
// The campaign replaced four categorical perturbation levels with a continuous magnitude,
// so the curve is the result rather than a discarded calibration input. That removes the
// "why these levels?" question entirely, but only if the magnitude a goal receives is
// drawn from the campaign seed rather than configured per cell. These tests pin the three
// properties the design rests on: every goal in the sweep draws a distinct magnitude, the
// draw is reproducible from (campaign_seed, run, goal) alone, and coverage of the range is
// even in the space the regression is fitted in.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <set>
#include <vector>

#include "navlearn_benchmarks/magnitude_sampler.hpp"
#include "navlearn_benchmarks/navlearn_seed.hpp"

using navlearn::MagnitudeSampler;
using navlearn::MagnitudeScale;
using navlearn::seed::Stream;

namespace
{
constexpr uint64_t kCampaign = 424242ULL;

double draw(const MagnitudeSampler & s, uint64_t run, uint64_t goal)
{
  return s.sample(navlearn::seed::derive(kCampaign, Stream::PERTURBATION_MAGNITUDE,
                                         run, goal));
}
}  // namespace

TEST(MagnitudeSampler, DrawsLieWithinTheRequestedRange)
{
  MagnitudeSampler s(0.3, 3.0, MagnitudeScale::LOG);
  for (uint64_t run = 0; run < 12; ++run) {
    for (uint64_t goal = 0; goal < 10; ++goal) {
      const double m = draw(s, run, goal);
      EXPECT_GE(m, 0.3);
      EXPECT_LE(m, 3.0);
    }
  }
}

TEST(MagnitudeSampler, SameSeedGivesTheSameMagnitude)
{
  // Reproducibility is the whole point of deriving the magnitude rather than configuring
  // it: a rerun of a cell must perturb every goal exactly as the first run did.
  MagnitudeSampler s(0.3, 3.0, MagnitudeScale::LOG);
  for (uint64_t goal = 0; goal < 5; ++goal) {
    EXPECT_DOUBLE_EQ(draw(s, 2, goal), draw(s, 2, goal));
  }
}

TEST(MagnitudeSampler, DistinctGoalsDrawDistinctMagnitudes)
{
  // The defect this replaces: hardcoded perturbation seeds meant a pooled cell was a
  // handful of unique setups repeated, which invalidated every confidence interval
  // computed from it. A curve fitted on repeated magnitudes has the same disease.
  MagnitudeSampler s(0.3, 3.0, MagnitudeScale::LOG);
  std::set<double> seen;
  for (uint64_t run = 0; run < 12; ++run) {
    for (uint64_t goal = 0; goal < 10; ++goal) {
      seen.insert(draw(s, run, goal));
    }
  }
  EXPECT_EQ(seen.size(), 120u) << "magnitudes repeated across goals";
}

TEST(MagnitudeSampler, LogScaleCoversTheRangeEvenlyInLogSpace)
{
  // A displacement sweep spans an order of magnitude, and the regression is fitted on
  // log-displacement. Sampling uniformly in metres would put most goals in the top half of
  // the range and leave the onset of degradation, which is the part the curve exists to
  // locate, sparsely covered.
  MagnitudeSampler s(0.3, 3.0, MagnitudeScale::LOG);
  const double lo = std::log(0.3), hi = std::log(3.0);
  const int kBins = 5;
  std::vector<int> counts(kBins, 0);
  const int kDraws = 2000;

  for (int i = 0; i < kDraws; ++i) {
    const double m = draw(s, static_cast<uint64_t>(i / 10), static_cast<uint64_t>(i % 10));
    int bin = static_cast<int>((std::log(m) - lo) / (hi - lo) * kBins);
    bin = std::min(std::max(bin, 0), kBins - 1);
    counts[static_cast<size_t>(bin)]++;
  }

  const double expected = static_cast<double>(kDraws) / kBins;
  for (int b = 0; b < kBins; ++b) {
    EXPECT_NEAR(counts[static_cast<size_t>(b)], expected, expected * 0.25)
      << "log-space bin " << b << " is unevenly covered";
  }
}

TEST(MagnitudeSampler, LinearScaleCoversTheRangeEvenlyInMetres)
{
  MagnitudeSampler s(0.0, 1.0, MagnitudeScale::LINEAR);
  const int kBins = 5;
  std::vector<int> counts(kBins, 0);
  const int kDraws = 2000;

  for (int i = 0; i < kDraws; ++i) {
    const double m = draw(s, static_cast<uint64_t>(i / 10), static_cast<uint64_t>(i % 10));
    int bin = static_cast<int>(m * kBins);
    bin = std::min(std::max(bin, 0), kBins - 1);
    counts[static_cast<size_t>(bin)]++;
  }

  const double expected = static_cast<double>(kDraws) / kBins;
  for (int b = 0; b < kBins; ++b) {
    EXPECT_NEAR(counts[static_cast<size_t>(b)], expected, expected * 0.25)
      << "linear bin " << b << " is unevenly covered";
  }
}

TEST(MagnitudeSampler, DegenerateRangeReturnsTheSingleValue)
{
  // min == max is how a curve cell is pinned to one magnitude, which is what the fixed
  // sigma_hit ablation leg needs. It must not divide by a zero span.
  MagnitudeSampler s(1.0, 1.0, MagnitudeScale::LOG);
  EXPECT_DOUBLE_EQ(draw(s, 0, 0), 1.0);
  EXPECT_DOUBLE_EQ(draw(s, 3, 7), 1.0);
}

TEST(MagnitudeSampler, RejectsAnInvertedRange)
{
  EXPECT_THROW(MagnitudeSampler(3.0, 0.3, MagnitudeScale::LOG), std::invalid_argument);
}

TEST(MagnitudeSampler, RejectsNonPositiveBoundsOnALogScale)
{
  // log(0) is not a magnitude. A silently clamped zero would put a cluster of goals at the
  // bottom of the range that the sweep never asked for.
  EXPECT_THROW(MagnitudeSampler(0.0, 3.0, MagnitudeScale::LOG), std::invalid_argument);
  EXPECT_THROW(MagnitudeSampler(-1.0, 3.0, MagnitudeScale::LOG), std::invalid_argument);
}

TEST(MagnitudeSampler, MagnitudeStreamIsIndependentOfTheOtherDraws)
{
  // Magnitude must not correlate with where the goal is or where the teleport lands, or
  // the sweep would confound severity with position and the curve would not be a curve in
  // severity alone.
  const uint64_t a = navlearn::seed::derive(kCampaign, Stream::PERTURBATION_MAGNITUDE, 1, 1);
  const uint64_t b = navlearn::seed::derive(kCampaign, Stream::KIDNAP_TARGET, 1, 1);
  const uint64_t c = navlearn::seed::derive(kCampaign, Stream::GOAL_POSITION, 1, 1);
  EXPECT_NE(a, b);
  EXPECT_NE(a, c);
}
