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

#ifndef NAVLEARN_BENCHMARKS__MAGNITUDE_SAMPLER_HPP_
#define NAVLEARN_BENCHMARKS__MAGNITUDE_SAMPLER_HPP_

#include <cmath>
#include <cstdint>
#include <random>
#include <stdexcept>

namespace navlearn
{

/// How magnitudes are spread across the requested range.
enum class MagnitudeScale
{
  /// Uniform in the value itself. For a range spanning well under an order of magnitude.
  LINEAR,
  /// Uniform in log(value). For a range spanning an order of magnitude or more, which is
  /// also the space the recovery regression is fitted in.
  LOG,
};

/// Draws one perturbation magnitude per goal, continuously across a range.
///
/// Why continuous rather than four levels
///     With categorical levels a reviewer can always ask whether the conclusion holds at
///     magnitudes that were not chosen, and the honest answer is "not reported". Making
///     the magnitude continuous and the curve the reported result removes the question
///     instead of answering it, and it costs the same machine time: the sweep IS the
///     experiment, so the calibration runs that would have chosen the levels disappear.
///
/// Why the magnitude is derived from the campaign seed rather than configured per cell
///     A cell configured with one magnitude produces goals that all share it. Pooling such
///     cells is what invalidated the retired campaign's confidence intervals — the
///     hardcoded perturbation seeds meant a nominal n was a handful of unique setups
///     repeated. Deriving from (campaign_seed, run, goal) gives every goal its own
///     magnitude while keeping the whole sweep reproducible from one number.
///
/// Why log by default for displacement
///     A displacement sweep spans an order of magnitude. Sampling uniformly in metres puts
///     most goals in the top half of the range and leaves the onset of degradation — the
///     part the curve exists to locate — sparsely covered.
class MagnitudeSampler
{
public:
  /// Throws std::invalid_argument for an inverted range, or a non-positive bound on a log
  /// scale, where log(0) is not a magnitude and a silent clamp would pile goals at the
  /// bottom of the range that the sweep never asked for.
  MagnitudeSampler(double min_value, double max_value, MagnitudeScale scale)
  : min_(min_value), max_(max_value), scale_(scale)
  {
    if (max_value < min_value) {
      throw std::invalid_argument("magnitude max is below magnitude min");
    }
    if (scale == MagnitudeScale::LOG && min_value <= 0.0) {
      throw std::invalid_argument("log-scale magnitude range requires a positive minimum");
    }
  }

  /// One magnitude for the goal whose derived seed is `goal_seed`.
  ///
  /// Takes the seed rather than the indices so the caller cannot accidentally draw from
  /// the wrong stream: the stream tag is chosen once, at the call site that owns it.
  double sample(uint64_t goal_seed) const
  {
    // A pinned range is how a cell holds severity fixed (the ablation leg needs this), and
    // it must not divide by a zero span.
    if (max_ <= min_) { return min_; }

    std::mt19937_64 gen(goal_seed);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    const double u = unit(gen);

    if (scale_ == MagnitudeScale::LOG) {
      const double lo = std::log(min_);
      const double hi = std::log(max_);
      return std::exp(lo + u * (hi - lo));
    }
    return min_ + u * (max_ - min_);
  }

  double min_value() const { return min_; }
  double max_value() const { return max_; }
  MagnitudeScale scale() const { return scale_; }

  /// True when the range pins severity to a single value.
  bool is_fixed() const { return max_ <= min_; }

private:
  double min_;
  double max_;
  MagnitudeScale scale_;
};

}  // namespace navlearn

#endif  // NAVLEARN_BENCHMARKS__MAGNITUDE_SAMPLER_HPP_
