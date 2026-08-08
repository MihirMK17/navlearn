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
// Tests for the scan-rate decimation rule.
//
// The rule decides which scans a starved-sensor cell delivers to the stack. Two properties
// carry the experiment. It must be a pure function of the arrival index, so that a rerun of
// the same seed drops the same scans and a cell is reproducible; and the rate it actually
// delivers must be reported rather than assumed, so a cell can never silently run at a rate
// nobody asked for. Both are asserted here rather than inferred from a live run.

#include <gtest/gtest.h>

#include "navlearn_benchmarks/scan_decimator.hpp"

using navlearn::ScanDecimator;

TEST(ScanDecimator, KeepsEveryScanWhenTargetEqualsNative)
{
  ScanDecimator d(10.0, 10.0);
  EXPECT_EQ(d.keep_every(), 1u);
  EXPECT_DOUBLE_EQ(d.achieved_rate_hz(), 10.0);
  for (uint64_t i = 0; i < 20; ++i) {
    EXPECT_TRUE(d.shouldKeep(i)) << "scan " << i << " dropped at pass-through rate";
  }
}

TEST(ScanDecimator, CampaignRatesDivideTheNativeRateExactly)
{
  // The four rates leg 6 runs. All divide 10 Hz, so each is delivered exactly.
  const struct { double target; unsigned expect_n; } cases[] = {
    {10.0, 1u}, {5.0, 2u}, {2.0, 5u}, {1.0, 10u},
  };
  for (const auto & c : cases) {
    ScanDecimator d(10.0, c.target);
    EXPECT_EQ(d.keep_every(), c.expect_n) << "target " << c.target;
    EXPECT_DOUBLE_EQ(d.achieved_rate_hz(), c.target) << "target " << c.target;
    EXPECT_FALSE(d.rate_is_approximate()) << "target " << c.target;
  }
}

TEST(ScanDecimator, KeepsTheFirstScanAndThenEveryNth)
{
  ScanDecimator d(10.0, 2.0);          // keep every 5th
  const bool expected[12] = {true, false, false, false, false,
                             true, false, false, false, false,
                             true, false};
  for (uint64_t i = 0; i < 12; ++i) {
    EXPECT_EQ(d.shouldKeep(i), expected[i]) << "scan index " << i;
  }
}

TEST(ScanDecimator, DecisionDependsOnIndexAloneNotOnCallOrder)
{
  // Determinism is the reproducibility guarantee: the same index must give the same
  // answer however many times it is asked, and in whatever order. A stateful counter
  // would pass a sequential test and still desynchronise after a dropped callback.
  ScanDecimator d(10.0, 2.0);
  const uint64_t probes[] = {7, 0, 5, 5, 100, 7, 0};
  for (uint64_t i : probes) {
    EXPECT_EQ(d.shouldKeep(i), (i % 5 == 0)) << "index " << i;
  }
}

TEST(ScanDecimator, NonDivisorTargetReportsTheRateItCanActuallyDeliver)
{
  // 10 Hz cannot be decimated to 3 Hz. Rounding to every 3rd scan delivers 3.333 Hz.
  // The number the campaign records must be the delivered one, and the mismatch must be
  // visible rather than rounded away in a log line nobody reads.
  ScanDecimator d(10.0, 3.0);
  EXPECT_EQ(d.keep_every(), 3u);
  EXPECT_NEAR(d.achieved_rate_hz(), 10.0 / 3.0, 1e-9);
  EXPECT_TRUE(d.rate_is_approximate());
  EXPECT_NEAR(d.rate_error_hz(), 10.0 / 3.0 - 3.0, 1e-9);
}

TEST(ScanDecimator, RoundsToTheNearestAchievableRateNotDown)
{
  // 10 -> 4 Hz: every 2nd gives 5 Hz (error 1.0), every 3rd gives 3.33 Hz (error 0.67).
  // Nearest wins, so the delivered rate is the closest one available.
  ScanDecimator d(10.0, 4.0);
  EXPECT_EQ(d.keep_every(), 3u);
  EXPECT_NEAR(d.achieved_rate_hz(), 10.0 / 3.0, 1e-9);
}

TEST(ScanDecimator, RejectsATargetAboveTheNativeRate)
{
  // Decimation cannot create measurements. Asking for more than the sensor produces is a
  // configuration error, and a cell that quietly ran at the native rate instead would be
  // pooled with genuinely starved ones.
  EXPECT_THROW(ScanDecimator(10.0, 12.0), std::invalid_argument);
}

TEST(ScanDecimator, RejectsNonPositiveRates)
{
  EXPECT_THROW(ScanDecimator(10.0, 0.0), std::invalid_argument);
  EXPECT_THROW(ScanDecimator(10.0, -1.0), std::invalid_argument);
  EXPECT_THROW(ScanDecimator(0.0, 1.0), std::invalid_argument);
}

TEST(ScanDecimator, DeliveredCountOverAWindowMatchesTheAchievedRate)
{
  // End-to-end arithmetic check: over 1000 native scans at 10 Hz (100 s), a 1 Hz cell
  // must deliver 100 scans. An off-by-one in the keep rule shows up here as a rate error.
  ScanDecimator d(10.0, 1.0);
  unsigned kept = 0;
  for (uint64_t i = 0; i < 1000; ++i) {
    if (d.shouldKeep(i)) { ++kept; }
  }
  EXPECT_EQ(kept, 100u);
  EXPECT_DOUBLE_EQ(kept / 100.0, d.achieved_rate_hz());
}
