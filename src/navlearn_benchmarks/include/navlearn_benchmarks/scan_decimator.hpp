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

#ifndef NAVLEARN_BENCHMARKS__SCAN_DECIMATOR_HPP_
#define NAVLEARN_BENCHMARKS__SCAN_DECIMATOR_HPP_

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace navlearn
{

/// Decides which scans a starved-sensor cell delivers, and at what rate.
///
/// Why index-based decimation rather than a timer
///     A timer resampling the stream would drop a set of scans that depends on scheduler
///     jitter, so two runs of the same seed would starve the filter differently and the
///     cell would not be reproducible. Keeping every Nth arrival makes the decision a pure
///     function of the arrival index: the same seed drops the same scans on every rerun.
///
/// Why the delivered rate is computed rather than assumed
///     Only integer decimation factors exist, so a target that does not divide the native
///     rate cannot be delivered. Reporting the achievable rate lets the caller refuse the
///     cell instead of recording a requested rate the run never actually ran at.
class ScanDecimator
{
public:
  /// Throws std::invalid_argument for non-positive rates or a target above the native
  /// rate. Decimation cannot create measurements: a cell asking for more than the sensor
  /// produces would quietly run at the native rate and pool with genuinely starved cells.
  ScanDecimator(double native_rate_hz, double target_rate_hz)
  : native_rate_hz_(native_rate_hz), target_rate_hz_(target_rate_hz)
  {
    if (!(native_rate_hz > 0.0)) {
      throw std::invalid_argument("native_rate_hz must be positive");
    }
    if (!(target_rate_hz > 0.0)) {
      throw std::invalid_argument("target_rate_hz must be positive");
    }
    if (target_rate_hz > native_rate_hz * (1.0 + kRateEpsilon)) {
      throw std::invalid_argument(
              "target_rate_hz exceeds native_rate_hz; decimation cannot create scans");
    }

    // Nearest achievable factor, not the floor: for a 10 Hz sensor asked for 4 Hz, every
    // 3rd scan (3.33 Hz) is closer than every 2nd (5 Hz), and the closest available rate
    // is the honest interpretation of the request.
    const double ideal = native_rate_hz / target_rate_hz;
    auto n = static_cast<uint64_t>(std::llround(ideal));
    keep_every_ = (n < 1u) ? 1u : n;
  }

  /// True when the scan at this arrival index is delivered. Pure: same index, same answer,
  /// in any order and however many times asked.
  bool shouldKeep(uint64_t index) const { return (index % keep_every_) == 0u; }

  uint64_t keep_every() const { return keep_every_; }

  /// The rate this decimation actually delivers, which is what the run record must carry.
  double achieved_rate_hz() const
  {
    return native_rate_hz_ / static_cast<double>(keep_every_);
  }

  /// Signed difference between what is delivered and what was asked for.
  double rate_error_hz() const { return achieved_rate_hz() - target_rate_hz_; }

  /// True when the target could not be delivered exactly.
  bool rate_is_approximate() const
  {
    return std::fabs(rate_error_hz()) > kRateEpsilon * target_rate_hz_;
  }

  double native_rate_hz() const { return native_rate_hz_; }
  double target_rate_hz() const { return target_rate_hz_; }

private:
  // Relative slack for float comparison of rates. Well below any rate difference the
  // campaign distinguishes (the closest pair is 1 Hz against 2 Hz).
  static constexpr double kRateEpsilon = 1e-9;

  double native_rate_hz_;
  double target_rate_hz_;
  uint64_t keep_every_;
};

}  // namespace navlearn

#endif  // NAVLEARN_BENCHMARKS__SCAN_DECIMATOR_HPP_
