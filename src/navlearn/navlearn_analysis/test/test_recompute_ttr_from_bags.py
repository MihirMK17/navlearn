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

"""Tests for the offline TTR recompute.

Every quantity the recompute reports is checked against series whose answers are known
by construction: touch-and-drift must not count as recovered, the sustained entry time
must be the FINAL entry (not the first touch), and the hold-style number must reproduce
what the node would have said given an uncapped timeout. The whole point of the script
is to replace censored numbers; a bug here silently rewrites a paper table.
"""

import math

from navlearn_analysis.claims.recompute_ttr_from_bags import (
    binned_recovery,
    evaluate_window,
    interp_gt,
    log_bin_edges,
    sample_ok,
    wrap_to_pi,
    yaw_from_quat,
)


def flat_gt(t0, t1, x=0.0, y=0.0, yaw=0.0, hz=30.0):
    """Stationary ground truth sampled at hz over [t0, t1]."""
    n = int((t1 - t0) * hz) + 1
    return [(t0 + k / hz, x, y, yaw) for k in range(n)]


def amcl_at(times, x=0.0, y=0.0, yaw=0.0):
    return [(t, x, y, yaw) for t in times]


class TestAngles:
    def test_wrap_to_pi(self):
        assert abs(wrap_to_pi(3 * math.pi) - math.pi) < 1e-12
        assert abs(wrap_to_pi(-3 * math.pi) - math.pi) < 1e-12
        assert wrap_to_pi(0.5) == 0.5

    def test_yaw_from_quat_pure_z(self):
        # 90 degrees about z
        assert (
            abs(
                yaw_from_quat(0, 0, math.sin(0.25 * math.pi), math.cos(0.25 * math.pi))
                - 0.5 * math.pi
            )
            < 1e-12
        )


class TestSampleOk:
    def test_boundary_is_exclusive_like_the_node(self):
        assert not sample_ok(0.20, 0.0, 0.0)
        assert not sample_ok(0.0, 0.0, 0.10)
        assert sample_ok(0.19, 0.0, 0.0)

    def test_hypot_catches_diagonal(self):
        # each axis inside, hypot outside -- the node's third clause
        assert not sample_ok(0.15, 0.15, 0.0)

    def test_yaw_wraps_before_compare(self):
        assert sample_ok(0.0, 0.0, 2.0 * math.pi + 0.05)


class TestInterpGt:
    GT = [(0.0, 0.0, 0.0, 0.0), (1.0, 1.0, 2.0, 0.2)]

    def test_stale_beyond_030_returns_none(self):
        assert interp_gt([(0.0, 0.0, 0.0, 0.0)], 0.31) is None

    def test_before_first_sample_returns_none(self):
        assert interp_gt(self.GT, -0.1) is None

    def test_gap_above_030_holds_earlier_sample(self):
        # samples 1 s apart: no interpolation, hold gt[0] (query 0.2 s after it)
        x, y, yaw = interp_gt(self.GT, 0.2)
        assert (x, y, yaw) == (0.0, 0.0, 0.0)

    def test_linear_interpolation_inside_gap(self):
        gt = [(0.0, 0.0, 0.0, 0.0), (0.2, 1.0, 2.0, 0.2)]
        x, y, yaw = interp_gt(gt, 0.1)
        assert abs(x - 0.5) < 1e-12
        assert abs(y - 1.0) < 1e-12
        assert abs(yaw - 0.1) < 1e-12

    def test_yaw_interpolates_across_pi(self):
        gt = [(0.0, 0.0, 0.0, math.pi - 0.05), (0.2, 0.0, 0.0, -math.pi + 0.05)]
        _, _, yaw = interp_gt(gt, 0.1)
        assert abs(abs(yaw) - math.pi) < 1e-9  # midway through the wrap, not 0


class TestEvaluateWindow:
    def test_never_converges(self):
        gt = flat_gt(0.0, 20.0)
        amcl = amcl_at([1.0 * k for k in range(1, 20)], x=5.0)
        r = evaluate_window(amcl, gt, 0.0, 20.0)
        assert r["recovered_sustained"] == 0
        assert r["first_touch_s"] is None
        assert r["ttr_sustained_s"] is None
        assert r["ttr_hold_s"] is None

    def test_clean_recovery_all_three_agree(self):
        gt = flat_gt(0.0, 20.0)
        bad = amcl_at([1.0, 2.0, 3.0], x=5.0)
        good = amcl_at([4.0 + k for k in range(16)])
        r = evaluate_window(bad + good, gt, 0.0, 20.0)
        assert r["recovered_sustained"] == 1
        assert abs(r["first_touch_s"] - 4.0) < 1e-9
        assert abs(r["ttr_sustained_s"] - 4.0) < 1e-9
        assert abs(r["ttr_hold_s"] - 4.0) < 1e-9

    def test_touch_and_drift_is_not_recovered(self):
        # inside threshold 4-8 s, diverges after: PROTOCOL says NOT recovered,
        # but hold-style (2 s) says recovered at 4 s. That gap is the censoring bug.
        gt = flat_gt(0.0, 20.0)
        series = (
            amcl_at([1.0, 2.0, 3.0], x=5.0)
            + amcl_at([4.0, 5.0, 6.0, 7.0, 8.0])
            + amcl_at([9.0 + k for k in range(11)], x=5.0)
        )
        r = evaluate_window(series, gt, 0.0, 20.0)
        assert r["recovered_sustained"] == 0
        assert abs(r["first_touch_s"] - 4.0) < 1e-9
        assert r["ttr_sustained_s"] is None
        assert abs(r["ttr_hold_s"] - 4.0) < 1e-9

    def test_sustained_entry_is_final_entry_not_first_touch(self):
        # touch at 4-5 s, diverge, re-enter at 10 s and stay: recovered, ttr = 10
        gt = flat_gt(0.0, 20.0)
        series = (
            amcl_at([1.0, 2.0], x=5.0)
            + amcl_at([4.0, 5.0])
            + amcl_at([6.0, 7.0, 8.0], x=5.0)
            + amcl_at([10.0 + k for k in range(10)])
        )
        r = evaluate_window(series, gt, 0.0, 20.0)
        assert r["recovered_sustained"] == 1
        assert abs(r["first_touch_s"] - 4.0) < 1e-9
        assert abs(r["ttr_sustained_s"] - 10.0) < 1e-9

    def test_yaw_alone_blocks_recovery(self):
        gt = flat_gt(0.0, 10.0)
        amcl = [(t, 0.0, 0.0, 0.3) for t in [2.0, 3.0, 4.0, 5.0, 6.0]]
        r = evaluate_window(amcl, gt, 0.0, 10.0)
        assert r["recovered_sustained"] == 0

    def test_samples_outside_window_ignored(self):
        gt = flat_gt(0.0, 30.0)
        # perfect samples before kidnap and after end must not count
        series = amcl_at([1.0, 2.0]) + amcl_at([6.0, 7.0], x=5.0) + amcl_at([25.0])
        r = evaluate_window(series, gt, 5.0, 20.0)
        assert r["n_samples"] == 2
        assert r["recovered_sustained"] == 0

    def test_no_gt_coverage_counts_skips(self):
        gt = flat_gt(0.0, 5.0)
        amcl = amcl_at([10.0, 11.0])  # > 0.30 s past last gt
        r = evaluate_window(amcl, gt, 9.0, 12.0)
        assert r["n_samples"] == 0
        assert r["n_skipped_no_gt"] == 2
        assert r["recovered_sustained"] == 0

    def test_single_ok_sample_at_end_recovers_with_ttr_but_no_hold(self):
        gt = flat_gt(0.0, 10.0)
        series = amcl_at([1.0, 2.0], x=5.0) + amcl_at([9.5])
        r = evaluate_window(series, gt, 0.0, 10.0)
        assert r["recovered_sustained"] == 1
        assert abs(r["ttr_sustained_s"] - 9.5) < 1e-9
        assert r["ttr_hold_s"] is None  # 0.5 s of evidence, not 2 s
        assert abs(r["last_sample_gap_s"] - 0.5) < 1e-9


class TestBinning:
    def test_log_edges_match_campaign_analysis(self):
        edges = log_bin_edges(0.01, 3.0, 4)
        assert abs(edges[0] - 0.01) < 1e-12
        assert abs(edges[-1] - 3.0) < 1e-9
        ratios = [edges[i + 1] / edges[i] for i in range(4)]
        assert max(ratios) - min(ratios) < 1e-9

    def test_last_bin_closed(self):
        edges = [0.01, 0.1, 1.0]
        rows = [{"magnitude_m": 1.0, "recovered_sustained": 1}]
        out = binned_recovery(rows, edges, "recovered_sustained")
        assert out[1][2] == 1  # the hi-edge goal lands in the last bin
