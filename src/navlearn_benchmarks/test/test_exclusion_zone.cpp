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
// Exclusion zones: the rectangles barred from goal placement and kidnap sampling.
//
// Why this is tested rather than eyeballed
//   Until 2026-08-05 this was one rectangle hardcoded for small_house, and every world
//   inherited it. The first test below is the regression that matters: the default must
//   reproduce those exact bounds, because legs 1-6 and the yaw leg were all collected
//   under them and a shifted zone would make the new data quietly incomparable with the
//   old. The rest pin the behaviour the warehouse run depends on -- an empty list
//   excluding nothing, and multiple zones each being honoured.

#include <vector>

#include "gtest/gtest.h"
#include "navlearn_benchmarks/exclusion_zone.hpp"

using navlearn::inAnyExclusionZone;

namespace
{
// The rectangle the value was hardcoded to, and the node's default.
const std::vector<double> kSmallHouseDefault{-2.4, 0.7, 3.3, 5.4};
}  // namespace

TEST(ExclusionZone, default_reproduces_the_hardcoded_small_house_rectangle)
{
  // Inside.
  EXPECT_TRUE(inAnyExclusionZone(kSmallHouseDefault, -1.0, 4.0));
  EXPECT_TRUE(inAnyExclusionZone(kSmallHouseDefault, 0.6, 3.4));
  // Outside on each side.
  EXPECT_FALSE(inAnyExclusionZone(kSmallHouseDefault, -2.5, 4.0));
  EXPECT_FALSE(inAnyExclusionZone(kSmallHouseDefault, 0.8, 4.0));
  EXPECT_FALSE(inAnyExclusionZone(kSmallHouseDefault, -1.0, 3.2));
  EXPECT_FALSE(inAnyExclusionZone(kSmallHouseDefault, -1.0, 5.5));
}

TEST(ExclusionZone, edges_are_exclusive_exactly_as_before)
{
  // The original used strict inequalities on all four bounds. A point on an edge was
  // not excluded, and that must not drift: it would move which goals are legal.
  EXPECT_FALSE(inAnyExclusionZone(kSmallHouseDefault, -2.4, 4.0));
  EXPECT_FALSE(inAnyExclusionZone(kSmallHouseDefault, 0.7, 4.0));
  EXPECT_FALSE(inAnyExclusionZone(kSmallHouseDefault, -1.0, 3.3));
  EXPECT_FALSE(inAnyExclusionZone(kSmallHouseDefault, -1.0, 5.4));
}

TEST(ExclusionZone, an_empty_list_excludes_nothing)
{
  // What the warehouse cells pass. A world that inherits another world's rectangle has
  // a hole in it that appears in no config and no log.
  const std::vector<double> none{};
  EXPECT_FALSE(inAnyExclusionZone(none, 0.0, 0.0));
  EXPECT_FALSE(inAnyExclusionZone(none, -1.0, 4.0));
  EXPECT_FALSE(inAnyExclusionZone(none, 1e6, -1e6));
}

TEST(ExclusionZone, every_zone_in_a_multi_zone_list_is_honoured)
{
  const std::vector<double> zones{-2.4, 0.7, 3.3, 5.4, 10.0, 12.0, -1.0, 1.0};
  EXPECT_TRUE(inAnyExclusionZone(zones, -1.0, 4.0));   // first
  EXPECT_TRUE(inAnyExclusionZone(zones, 11.0, 0.0));   // second
  EXPECT_FALSE(inAnyExclusionZone(zones, 5.0, 0.0));   // between them
}

TEST(ExclusionZone, a_trailing_partial_group_is_ignored_not_half_applied)
{
  // The node refuses to start on a list that is not a multiple of four, so this is
  // defence in depth: a truncated group must never be interpreted with guessed bounds.
  const std::vector<double> ragged{-2.4, 0.7, 3.3, 5.4, 10.0, 12.0};
  EXPECT_TRUE(inAnyExclusionZone(ragged, -1.0, 4.0));
  EXPECT_FALSE(inAnyExclusionZone(ragged, 11.0, 0.0));
}

TEST(ExclusionZone, an_inverted_rectangle_excludes_nothing_rather_than_everything)
{
  // xmin > xmax cannot be satisfied, so it bars nothing. Worth pinning: the opposite
  // convention would silently bar the entire map and produce a campaign of zero goals.
  const std::vector<double> inverted{5.0, -5.0, 5.0, -5.0};
  EXPECT_FALSE(inAnyExclusionZone(inverted, 0.0, 0.0));
}
