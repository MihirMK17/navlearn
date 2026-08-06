#!/usr/bin/env python3
# Copyright 2026 Mihir Kulkarni
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for the leg 6 confident-and-wrong mechanism analysis.

The risk this guards against is specific. The analysis compares cells that differ in
sample density by a factor of ten, so any function that degrades quietly when samples
are sparse would produce a difference between rates out of nothing at all -- and that
difference would look exactly like the mechanism being tested. Every helper is therefore
checked on a deliberately thin series as well as a dense one.
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))

from rate_mechanism import (  # noqa: E402
    CONFIDENT_FACTOR,
    analyse_goal,
    baseline_cov,
    first_crossing,
    interp_xy,
)


def gt_series(t0, t1, hz=30.0, x=0.0, y=0.0):
    n = int((t1 - t0) * hz) + 1
    return [(t0 + k / hz, x, y) for k in range(n)]


class TestInterpXy:
    def test_interpolates_inside_a_small_gap(self):
        gt = [(0.0, 0.0, 0.0), (0.2, 1.0, 2.0)]
        x, y = interp_xy(gt, 0.1)
        assert abs(x - 0.5) < 1e-12 and abs(y - 1.0) < 1e-12

    def test_returns_none_when_stale(self):
        assert interp_xy([(0.0, 0.0, 0.0)], 0.31) is None

    def test_returns_none_before_first_sample(self):
        assert interp_xy([(1.0, 0.0, 0.0)], 0.5) is None


class TestBaselineCov:
    def test_median_of_the_pre_kidnap_window(self):
        amcl = [(t, 0.0, 0.0, 0.10) for t in [5.1, 5.5, 6.0, 6.5]]
        amcl += [(t, 0.0, 0.0, 9.0) for t in [7.5, 8.0]]  # after the kidnap, ignored
        assert baseline_cov(amcl, kidnap_t=7.0) == 0.10

    def test_excludes_samples_older_than_the_window(self):
        amcl = [(0.0, 0.0, 0.0, 99.0)] + [(t, 0.0, 0.0, 0.2) for t in [6.0, 6.4, 6.8]]
        assert baseline_cov(amcl, kidnap_t=7.0, window_s=5.0) == 0.2

    def test_returns_none_when_history_is_too_thin(self):
        # The 1 Hz cells have a tenth of the samples; a silent fallback here would
        # manufacture a rate difference. Too thin must mean "unknown", not a guess.
        amcl = [(6.5, 0.0, 0.0, 0.2), (6.9, 0.0, 0.0, 0.2)]
        assert baseline_cov(amcl, kidnap_t=7.0) is None

    def test_three_samples_is_enough(self):
        amcl = [(6.1, 0.0, 0.0, 0.2), (6.5, 0.0, 0.0, 0.4), (6.9, 0.0, 0.0, 0.6)]
        assert baseline_cov(amcl, kidnap_t=7.0) == 0.4


class TestFirstCrossing:
    def test_finds_the_first_sample_at_or_below(self):
        assert first_crossing([(1.0, 5.0), (2.0, 3.0), (3.0, 1.0)], 3.0) == (2.0, 3.0)

    def test_none_when_never_crossed(self):
        assert first_crossing([(1.0, 5.0), (2.0, 4.0)], 1.0) is None

    def test_takes_the_first_not_the_lowest(self):
        # A later, deeper sample must not win: the question is WHEN confidence arrived.
        assert first_crossing([(1.0, 2.9), (2.0, 0.1)], 3.0) == (1.0, 2.9)


class TestAnalyseGoal:
    def test_confident_and_wrong_is_detected(self):
        # Covariance collapses at t=3 while the estimate is 4 m from truth: the exact
        # signature the depletion account predicts for the fast filter.
        gt = gt_series(0.0, 20.0)
        amcl = [(t, 0.0, 0.0, 0.05) for t in [0.2, 0.6, 1.0, 1.4]]  # pre-kidnap baseline
        amcl += [(t, 4.0, 0.0, 5.0) for t in [2.0, 2.5]]            # lost, uncertain
        amcl += [(t, 4.0, 0.0, 0.06) for t in [3.0, 4.0, 5.0]]      # sure, still wrong
        r = analyse_goal(amcl, gt, kidnap_t=1.8, end_t=6.0)
        assert r["baseline_cov"] == 0.05
        assert abs(r["t_confident"] - (3.0 - 1.8)) < 1e-9
        assert abs(r["err_at_confident"] - 4.0) < 1e-6

    def test_confident_and_right(self):
        gt = gt_series(0.0, 20.0)
        amcl = [(t, 0.0, 0.0, 0.05) for t in [0.2, 0.6, 1.0, 1.4]]
        amcl += [(t, 4.0, 0.0, 5.0) for t in [2.0, 2.5]]
        amcl += [(t, 0.02, 0.0, 0.06) for t in [3.0, 4.0]]
        r = analyse_goal(amcl, gt, kidnap_t=1.8, end_t=5.0)
        assert r["err_at_confident"] < 0.05

    def test_never_confident_leaves_the_fields_none(self):
        gt = gt_series(0.0, 20.0)
        amcl = [(t, 0.0, 0.0, 0.05) for t in [0.2, 0.6, 1.0, 1.4]]
        amcl += [(t, 4.0, 0.0, 9.0) for t in [2.0, 3.0, 4.0]]
        r = analyse_goal(amcl, gt, kidnap_t=1.8, end_t=5.0)
        assert r["t_confident"] is None and r["err_at_confident"] is None
        assert r["min_err"] is not None  # still measured

    def test_threshold_is_relative_to_this_goals_own_baseline(self):
        # A filter whose converged covariance is ten times larger must not be judged
        # confident at the other's absolute value. Same series, scaled: same verdict.
        gt = gt_series(0.0, 20.0)
        for scale in (1.0, 10.0):
            amcl = [(t, 0.0, 0.0, 0.05 * scale) for t in [0.2, 0.6, 1.0, 1.4]]
            amcl += [(t, 4.0, 0.0, 5.0 * scale) for t in [2.0, 2.5]]
            amcl += [(t, 4.0, 0.0, 0.06 * scale) for t in [3.0, 4.0]]
            r = analyse_goal(amcl, gt, kidnap_t=1.8, end_t=5.0)
            assert abs(r["t_confident"] - 1.2) < 1e-9, f"scale {scale}"

    def test_update_count_and_rate_reflect_the_window_only(self):
        gt = gt_series(0.0, 30.0)
        amcl = [(t, 0.0, 0.0, 0.05) for t in [0.2, 0.6, 1.0, 1.4]]
        amcl += [(2.0 + k, 0.0, 0.0, 0.05) for k in range(10)]
        amcl += [(50.0, 0.0, 0.0, 0.05)]  # past the goal end, must not count
        r = analyse_goal(amcl, gt, kidnap_t=1.8, end_t=11.8)
        assert r["n_updates"] == 10
        assert abs(r["update_hz"] - 1.0) < 1e-9

    def test_sparse_and_dense_series_agree_on_confidence_timing(self):
        # The whole comparison is 10 Hz against 1 Hz. If sample density alone shifted
        # t_confident, the analysis would find a mechanism in its own arithmetic.
        # Identical underlying history, sampled at 1 Hz and at 10 Hz. Confidence arrives
        # at the same wall-clock instant in both; only the sampling differs. The 1 Hz
        # answer may be up to one sample period late, and no more.
        gt = gt_series(0.0, 40.0)
        kidnap_t, truth_t = 10.0, 13.0
        results = {}
        for hz in (1.0, 10.0):
            step = 1.0 / hz

            def at(t0, t1, x, cov, step=step):
                n = int(round((t1 - t0) / step))
                return [(t0 + k * step, x, 0.0, cov) for k in range(n)]
            series = (at(5.0, kidnap_t, 0.0, 0.05)          # converged, 5 s of baseline
                      + at(kidnap_t, truth_t, 3.0, 5.0)      # lost and uncertain
                      + at(truth_t, 20.0, 3.0, 0.06))        # confident, still 3 m out
            results[hz] = analyse_goal(series, gt, kidnap_t=kidnap_t, end_t=20.0)

        for hz in (1.0, 10.0):
            assert results[hz]["baseline_cov"] == 0.05, f"{hz} Hz lost its baseline"
            assert results[hz]["t_confident"] is not None, f"{hz} Hz found no confidence"
            assert abs(results[hz]["err_at_confident"] - 3.0) < 1e-6
        expected = truth_t - kidnap_t
        for hz, tol in ((1.0, 1.0), (10.0, 0.1)):
            assert abs(results[hz]["t_confident"] - expected) <= tol, f"{hz} Hz"

    def test_a_1hz_goal_kidnapped_early_is_excluded_not_guessed(self):
        # At 1 Hz a 5 s baseline window holds ~5 samples, so a kidnap firing soon after
        # the goal starts leaves too little history. Such goals must drop out visibly
        # rather than borrow a threshold -- and because this bites the starved cells
        # hardest, the report prints the usable fraction per cell.
        gt = gt_series(0.0, 20.0)
        amcl = [(1.0, 0.0, 0.0, 0.05), (2.0, 0.0, 0.0, 0.05)]  # 2 samples, 1 Hz
        amcl += [(t, 5.0, 0.0, 0.06) for t in [4.0, 5.0, 6.0]]
        r = analyse_goal(amcl, gt, kidnap_t=3.0, end_t=8.0)
        assert r["baseline_cov"] is None
        assert r["t_confident"] is None

    def test_missing_baseline_disables_the_confidence_fields(self):
        gt = gt_series(0.0, 20.0)
        amcl = [(2.0, 4.0, 0.0, 5.0), (3.0, 0.0, 0.0, 0.01)]  # no pre-kidnap history
        r = analyse_goal(amcl, gt, kidnap_t=1.8, end_t=5.0)
        assert r["baseline_cov"] is None
        assert r["t_confident"] is None

    def test_confident_factor_is_applied(self):
        gt = gt_series(0.0, 20.0)
        base = 0.10
        just_over = CONFIDENT_FACTOR * base * 1.01
        just_under = CONFIDENT_FACTOR * base * 0.99
        amcl = [(t, 0.0, 0.0, base) for t in [0.2, 0.6, 1.0, 1.4]]
        amcl += [(2.0, 4.0, 0.0, just_over), (3.0, 4.0, 0.0, just_under)]
        r = analyse_goal(amcl, gt, kidnap_t=1.8, end_t=5.0)
        assert abs(r["t_confident"] - (3.0 - 1.8)) < 1e-9

    def test_goal_with_no_ground_truth_coverage_yields_no_errors(self):
        gt = gt_series(0.0, 1.0)
        amcl = [(t, 0.0, 0.0, 0.05) for t in [0.2, 0.4, 0.6]]
        amcl += [(t, 0.0, 0.0, 0.05) for t in [10.0, 11.0]]
        r = analyse_goal(amcl, gt, kidnap_t=9.0, end_t=12.0)
        assert r["min_err"] is None
        assert not math.isnan(r["window_s"])
