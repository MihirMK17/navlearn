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

/// Tests for benchmark seed derivation.
///
/// The two invariants asserted here are what make the campaign's sample sizes and its
/// statistical tests mean what they claim:
///
///   Uniqueness across (run, goal) — n episodes must be n distinct conditions. The scheme
///   this replaces combined a fixed constant with the goal index and ignored the run
///   index, so a 25-episode cell contained 5 conditions and a pooled n=75 contained the
///   same 5.
///
///   Equality across controllers — RPP, DWB and MPPI must face identical goals and
///   identical perturbations, which is the precondition for the paired tests (McNemar)
///   the analysis plan specifies.
///
/// The golden vectors pin the algorithm itself. splitmix64's constants are load-bearing:
/// changing them silently invalidates the reproducibility of every campaign run made
/// before the change, and a campaign that cannot be reproduced is not evidence. The same
/// vectors are asserted from Python in test_seed_reference.py, so the analysis scripts
/// are proven to re-derive exactly what the C++ drew.

#include <gtest/gtest.h>

#include <set>
#include <vector>

#include "navlearn_benchmarks/navlearn_seed.hpp"

using navlearn::seed::Stream;
using navlearn::seed::derive;

TEST(NavlearnSeed, GoldenVectorsPinTheAlgorithm)
{
  EXPECT_EQ(derive(42, Stream::GOAL_POSITION, 0, 0), 11244777001763019517ULL);
  EXPECT_EQ(derive(42, Stream::INITIAL_POSE, 0, 0), 4228068372608098218ULL);
  EXPECT_EQ(derive(42, Stream::INITIAL_POSE, 1, 0), 11275064635349857629ULL);
  EXPECT_EQ(derive(42, Stream::INITIAL_POSE, 0, 1), 3350397314887148275ULL);
  EXPECT_EQ(derive(42, Stream::KIDNAP_TARGET, 4, 4), 10896568657172245899ULL);
  EXPECT_EQ(derive(43, Stream::INITIAL_POSE, 0, 0), 10156579724707918578ULL);
}

TEST(NavlearnSeed, DistinctForEveryRunAndGoal)
{
  // 100 runs x 25 goals is far beyond the campaign's 5 x 5, deliberately: the property
  // must hold with margin, not just at the sizes currently planned.
  std::set<uint64_t> seen;
  for (uint64_t run = 0; run < 100; ++run) {
    for (uint64_t goal = 0; goal < 25; ++goal) {
      seen.insert(derive(42, Stream::INITIAL_POSE, run, goal));
    }
  }
  EXPECT_EQ(seen.size(), 2500u)
    << "collisions mean two episodes shared a perturbation, so the effective sample size "
       "is smaller than the reported one";
}

TEST(NavlearnSeed, RunIndexActuallyParticipates)
{
  // The precise defect being regression-tested: the old scheme ignored the run index, so
  // goal 0 of every run drew the same perturbation.
  std::set<uint64_t> per_run;
  for (uint64_t run = 0; run < 50; ++run) {
    per_run.insert(derive(42, Stream::INITIAL_POSE, run, 0));
  }
  EXPECT_EQ(per_run.size(), 50u)
    << "goal 0 must differ between runs; if it does not, the run index is being ignored "
       "exactly as it was before this scheme replaced the old one";
}

TEST(NavlearnSeed, IdenticalAcrossControllers)
{
  // Controllers do not appear in the signature, so this asserts the consequence: two
  // nodes running different controllers, given the same campaign seed and indices,
  // derive the same conditions. If a controller argument is ever added, this test still
  // passes but the signature change would be visible in review — which is why the header
  // documents the omission as deliberate rather than incidental.
  for (uint64_t run = 0; run < 5; ++run) {
    for (uint64_t goal = 0; goal < 5; ++goal) {
      const uint64_t as_rpp = derive(42, Stream::INITIAL_POSE, run, goal);
      const uint64_t as_dwb = derive(42, Stream::INITIAL_POSE, run, goal);
      const uint64_t as_mppi = derive(42, Stream::INITIAL_POSE, run, goal);
      EXPECT_EQ(as_rpp, as_dwb);
      EXPECT_EQ(as_rpp, as_mppi);
    }
  }
}

TEST(NavlearnSeed, StreamsDoNotCollide)
{
  const std::vector<Stream> streams = {
    Stream::GOAL_POSITION, Stream::GOAL_ORIENTATION, Stream::INITIAL_POSE,
    Stream::KIDNAP_TARGET, Stream::KIDNAP_DELAY, Stream::ATTEMPT_ID,
  };
  std::set<uint64_t> seen;
  for (const auto stream : streams) {
    seen.insert(derive(42, stream, 3, 7));
  }
  EXPECT_EQ(seen.size(), streams.size())
    << "two purposes sharing a seed would correlate the TTC displacement with the goal "
       "placement, confounding the perturbation with where the robot was sent";
}

TEST(NavlearnSeed, AdjacentCampaignSeedsDecorrelate)
{
  // Replication cells re-run at neighbouring campaign seeds. If adjacent seeds produced
  // overlapping condition sets, a replication would partly repeat the original rather
  // than sampling anew.
  std::set<uint64_t> combined;
  size_t total = 0;
  for (uint64_t campaign = 42; campaign <= 44; ++campaign) {
    for (uint64_t goal = 0; goal < 25; ++goal) {
      combined.insert(derive(campaign, Stream::INITIAL_POSE, 0, goal));
      ++total;
    }
  }
  EXPECT_EQ(combined.size(), total)
    << "replication seeds must not reproduce the original campaign's conditions";
}

TEST(NavlearnSeed, LargeIndicesRemainDistinct)
{
  // Guards the boundary where a narrower integer type would wrap. The old scheme cast to
  // uint32_t; nothing here should be sensitive to that.
  std::set<uint64_t> seen;
  for (uint64_t run : {0ULL, 1ULL, 1000ULL, 65535ULL, 65536ULL, 4294967295ULL,
                       4294967296ULL})
  {
    seen.insert(derive(42, Stream::KIDNAP_TARGET, run, 3));
  }
  EXPECT_EQ(seen.size(), 7u);
}
