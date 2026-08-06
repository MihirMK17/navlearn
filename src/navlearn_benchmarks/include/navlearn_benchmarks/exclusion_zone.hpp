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

#ifndef NAVLEARN_BENCHMARKS__EXCLUSION_ZONE_HPP_
#define NAVLEARN_BENCHMARKS__EXCLUSION_ZONE_HPP_

#include <cstddef>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace navlearn
{

/// Is (x, y) inside any barred rectangle?
///
/// Zones arrive as flat groups of four, [xmin, xmax, ymin, ymax, ...], because a ROS 2
/// parameter cannot carry a list of structs and a flat vector of doubles is the shape
/// that survives the round trip through YAML and the command line unchanged.
///
/// Why this is a parameter and not a constant
///     It used to be one rectangle hardcoded in episode_manager.cpp, chosen for
///     small_house. Every world inherited it. Pointed at a warehouse the same numbers
///     carve an arbitrary three-by-two metre patch out of goal placement and kidnap
///     sampling — a hole in the experiment that appears in no config, no log and no
///     record, and that nobody reading the results could reconstruct. The campaign has
///     already been invalidated twice by exactly that shape of invisible constant.
///
/// A trailing partial group is ignored rather than half-applied; the node refuses to
/// start on a list whose length is not a multiple of four, so reaching here with one is
/// already a bug, and guessing at the missing bounds would be worse than dropping it.
///
/// Bounds are exclusive on both sides, preserving the original comparison exactly: a
/// point on a zone edge is not excluded.
inline bool inAnyExclusionZone(const std::vector<double> & zones, double x, double y)
{
  for (std::size_t i = 0; i + 3 < zones.size(); i += 4) {
    if (x > zones[i] && x < zones[i + 1] && y > zones[i + 2] && y < zones[i + 3]) {
      return true;
    }
  }
  return false;
}

/// Parse the `exclusion_zones` parameter: "xmin,xmax,ymin,ymax, ..." or empty for none.
///
/// Why a string and not a double array
///     "no zones at all" is a legitimate and expected setting for any world that is not
///     small_house, and ROS 2 cannot represent it as a typed array: an empty list has no
///     inferable element type, arrives as PARAMETER_NOT_SET, and aborts the node with
///     "No parameter value set" whether the declaration is statically or dynamically
///     typed. Both were tried. A string has no such hole — "" is unambiguously empty —
///     and it round-trips through YAML, the command line and the run record unchanged.
///
/// Surrounding brackets are optional so both `[]` and `` and `[1,2,3,4]` are accepted;
/// commas and whitespace both separate.
///
/// Throws std::invalid_argument on a value that is not a multiple of four, or on a
/// token that is not a number. Both are configuration errors, and a half-read zone list
/// would bar a region nobody asked for while looking like it had worked.
inline std::vector<double> parseExclusionZones(const std::string & spec)
{
  std::string cleaned;
  cleaned.reserve(spec.size());
  for (const char c : spec) {
    cleaned.push_back((c == '[' || c == ']' || c == ',') ? ' ' : c);
  }

  std::vector<double> out;
  std::istringstream stream(cleaned);
  std::string token;
  while (stream >> token) {
    try {
      size_t consumed = 0;
      const double value = std::stod(token, &consumed);
      if (consumed != token.size()) {
        throw std::invalid_argument("trailing characters");
      }
      out.push_back(value);
    } catch (const std::exception &) {
      throw std::invalid_argument("exclusion_zones: '" + token + "' is not a number");
    }
  }

  if (out.size() % 4 != 0) {
    throw std::invalid_argument(
            "exclusion_zones needs groups of four [xmin, xmax, ymin, ymax]");
  }
  return out;
}

}  // namespace navlearn

#endif  // NAVLEARN_BENCHMARKS__EXCLUSION_ZONE_HPP_
