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

#ifndef NAVLEARN_BENCHMARKS__NAVLEARN_SEED_HPP_
#define NAVLEARN_BENCHMARKS__NAVLEARN_SEED_HPP_

#include <cstdint>

/// Deterministic seed derivation for benchmark randomness.
///
/// THE PROBLEM THIS REPLACES
///
/// Perturbation seeds were fixed constants combined with the goal index by addition:
/// `mt19937(initial_pose_seed + goal_index)` with `initial_pose_seed` pinned at 1337 in a
/// YAML default that nothing ever overrode. The run index never entered the calculation,
/// so goal 0 of run 1 and goal 0 of run 5 drew byte-identical perturbations. A cell of
/// 25 episodes contained 5 distinct displacements, and pooling three cells did not help
/// because they repeated the same five. Reported sample sizes were roughly an order of
/// magnitude larger than the number of distinct experimental conditions actually tested.
///
/// Additive offsets are also poor practice independently: `seed + index` produces
/// adjacent seeds, and adjacent seeds are not guaranteed to produce independent streams.
///
/// THE TWO INVARIANTS
///
/// Everything here exists to hold two properties at once:
///
///   1. DISTINCT across (run, goal, purpose). Every episode draws its own perturbation,
///      so n episodes means n conditions.
///
///   2. IDENTICAL across controllers. The controller under test is deliberately NOT an
///      input to the derivation, so RPP, DWB and MPPI face exactly the same goals and
///      exactly the same perturbations. This is the precondition for the paired
///      statistics the analysis plan calls for (McNemar); without it the arms are
///      independent samples and the weaker unpaired tests apply.
///
/// Both are asserted in test/test_navlearn_seed.cpp. Neither is safe to assume.
///
/// WHY SPLITMIX64
///
/// splitmix64 (Steele, Lea and Flood, 2014) is the standard finalizer for exactly this
/// job: it avalanches sequential inputs into well-separated outputs, so campaign seeds
/// 42 and 43, or runs 3 and 4, produce uncorrelated streams. It is stateless and
/// trivially reproducible in any language, which matters because the analysis scripts
/// must be able to re-derive a run's conditions from its manifest without linking C++.
namespace navlearn::seed
{

/// Distinct random streams. Two draws that must not correlate need different tags.
///
/// The values are arbitrary odd 64-bit constants; only their distinctness matters. Never
/// reuse or renumber a tag once a campaign has run against it — the tag is part of what
/// makes a result reproducible, so changing one silently changes the experiment.
enum class Stream : uint64_t
{
  GOAL_POSITION     = 0x243F6A8885A308D3ULL,  ///< where goals are placed
  GOAL_ORIENTATION  = 0x13198A2E03707344ULL,  ///< goal yaw
  INITIAL_POSE      = 0xA4093822299F31D0ULL,  ///< TTC bad-initialization offset
  KIDNAP_TARGET     = 0x082EFA98EC4E6C89ULL,  ///< TTR teleport destination
  KIDNAP_DELAY      = 0x452821E638D01377ULL,  ///< delay before the kidnap fires
  ATTEMPT_ID        = 0xBE5466CF34E90C6CULL,  ///< identifiers, not experimental conditions
  /// Continuous perturbation severity, once the campaign made the curve the result rather
  /// than four categorical levels. Its own stream because magnitude must not correlate
  /// with where the goal is or where the teleport lands: a sweep in which severity tracked
  /// position would confound the two, and the curve would no longer be a curve in severity.
  PERTURBATION_MAGNITUDE = 0xC0AC29B7C97C50DDULL,
  /// Continuous bad-initialization severity. Separate from PERTURBATION_MAGNITUDE, which
  /// drives the kidnap sweep: a cell that enabled both would otherwise draw the same
  /// value for each, correlating the two perturbations by construction.
  BAD_INIT_MAGNITUDE = 0x9216D5D98979FB1BULL,
  /// Continuous kidnap yaw severity (the yaw-curve leg: displacement pinned to zero, the
  /// rotation is the swept variable). Its own stream for the same reason as
  /// PERTURBATION_MAGNITUDE: the drawn angle must not correlate with goal placement,
  /// delay, or any other draw.
  KIDNAP_YAW_MAGNITUDE = 0x3F84D5B5B5470917ULL,
  /// Direction of the yaw-curve rotation. Separate from KIDNAP_YAW_MAGNITUDE so the sign
  /// is independent of the angle; deriving both from one draw would tie the two.
  KIDNAP_YAW_SIGN = 0xB8E1AFED6A267E96ULL,
};

/// splitmix64 mixing function.
///
/// Fixed algorithm — do not "improve" it. The constants are load-bearing, and any change
/// silently invalidates the reproducibility of every campaign run before the change.
inline uint64_t splitmix64(uint64_t x) noexcept
{
  x += 0x9E3779B97F4A7C15ULL;
  uint64_t z = x;
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
  return z ^ (z >> 31);
}

/// Derive the seed for one draw.
///
/// \param campaign_seed  Single value identifying the whole campaign. Changing it
///                       regenerates every condition; holding it fixed makes the campaign
///                       reproducible end to end.
/// \param stream         What is being drawn. Different purposes must not correlate.
/// \param run_index      Zero-based episode index within the cell.
/// \param goal_index     Zero-based goal index within the episode.
///
/// The controller, planner and localizer are deliberately absent from this signature.
/// Adding one would break the paired-comparison invariant, which is the entire reason
/// this function takes the arguments it does and no others.
inline uint64_t derive(
  uint64_t campaign_seed, Stream stream, uint64_t run_index, uint64_t goal_index) noexcept
{
  // Chained rather than combined with XOR in one step: each application avalanches the
  // accumulated state, so no single component can cancel another. Plain
  // `campaign_seed ^ run ^ goal` would collide for (run 1, goal 2) and (run 2, goal 1).
  uint64_t h = splitmix64(campaign_seed);
  h = splitmix64(h ^ static_cast<uint64_t>(stream));
  h = splitmix64(h ^ run_index);
  h = splitmix64(h ^ goal_index);
  return h;
}

}  // namespace navlearn::seed

#endif  // NAVLEARN_BENCHMARKS__NAVLEARN_SEED_HPP_
