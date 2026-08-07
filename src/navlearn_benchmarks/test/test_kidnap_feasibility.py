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

"""Tests for offline kidnap-feasibility estimation.

Why it exists
    The Gate-0 positive control lost 2 of 10 kidnaps to "no valid pose in the ring": at
    larger radii a thin annulus can miss free space entirely in a cluttered map. The
    predictor leg needs a fixed number of APPLIED kidnaps per map, so the attrition rate
    sets how many attempts to budget and how wide the band must be. Both are PROTOCOL.md
    numbers, and measuring them from the map costs seconds where measuring them from
    simulation costs hours.

What it must get right
    It replicates the node's own feasibility rule -- free space plus a clearance disc --
    so a "feasible" answer here means the sampler would actually have found a pose. If the
    two rules drifted, the budget would be built on a different experiment than the one
    that runs.
"""

import math
import os
import sys

import pytest

np = pytest.importorskip("numpy", reason="numpy not available")
pytest.importorskip("scipy", reason="scipy not available")

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))

from kidnap_feasibility import feasible_fraction, valid_pose_mask  # noqa: E402

RES = 0.05


def _open_field(size_m=30.0, res=RES):
    """A large empty room with a one-cell wall."""
    n = int(round(size_m / res))
    occ = np.zeros((n, n), dtype=bool)
    occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = True
    return occ


def test_clearance_mask_excludes_cells_too_close_to_a_wall():
    """The node requires a free disc of goal_min_clearance_m; so must this.

    A mask that merely tested "free" would call poses feasible that the sampler rejects,
    and the attempt budget derived from it would be optimistic in exactly the cluttered
    places where the sampler struggles.
    """
    occ = _open_field(4.0)
    mask = valid_pose_mask(occ, RES, clearance_m=0.75)
    edge = int(round(0.75 / RES))
    assert not mask[:edge, :].any(), "cells within the clearance radius of a wall passed"
    assert mask[edge + 2:-edge - 2, edge + 2:-edge - 2].all()


def test_open_field_is_fully_feasible():
    """Deep in open space every bearing works, so no attempt should be lost."""
    occ = _open_field(30.0)
    result = feasible_fraction(occ, RES, (0.0, 0.0), radius_m=1.5, band=0.05,
                               clearance_m=0.75, n_refs=80, seed=1)
    assert result["feasible_fraction"] == pytest.approx(1.0)


def test_radius_larger_than_the_map_is_never_feasible():
    """A ring outside the world has no valid pose; the answer is zero, not an error."""
    occ = _open_field(6.0)
    result = feasible_fraction(occ, RES, (0.0, 0.0), radius_m=20.0, band=0.05,
                               clearance_m=0.75, n_refs=40, seed=2)
    assert result["feasible_fraction"] == 0.0


def test_widening_the_band_never_reduces_feasibility():
    """The property that justifies the band as a knob at all.

    A wider annulus is a superset of a narrower one at the same radius, so feasibility is
    monotonic in the band. If it were not, trading precision for attempts would not be a
    trade -- it would be a gamble.
    """
    occ = _open_field(12.0)
    # A corridor cut through it, so feasibility is genuinely partial rather than saturated.
    occ[100:140, :] = True
    fractions = [
        feasible_fraction(occ, RES, (0.0, 0.0), radius_m=2.0, band=b,
                          clearance_m=0.75, n_refs=120, seed=3)["feasible_fraction"]
        for b in (0.05, 0.10, 0.20, 0.40)
    ]
    for tighter, wider in zip(fractions, fractions[1:]):
        assert wider >= tighter - 1e-12, f"widening the band lost feasibility: {fractions}"


def test_sparse_ring_is_reported_as_likely_missed_not_merely_possible():
    """Existence of a valid pose is not the same as the sampler finding it.

    The node draws a bounded number of random poses from the annulus and gives up if none
    land in free space, so a ring that is a fraction of a percent valid is usually missed
    even though it is not empty. Verified against the Gate-0 run: a goal whose ring was
    4 valid out of 3240 was reported unapplied, which this model puts at roughly a 16%
    miss -- while a pure existence test would have called it feasible outright and the
    attempt budget built on it would have been wrong.
    """
    occ = _open_field(12.0)
    occ[100:140, :] = True
    common = dict(radius_m=2.0, band=0.05, clearance_m=0.75, n_refs=120, seed=11)

    generous = feasible_fraction(occ, RES, (0.0, 0.0), max_sample_tries=10 ** 9, **common)
    realistic = feasible_fraction(occ, RES, (0.0, 0.0), max_sample_tries=1500, **common)

    assert realistic["expected_applied_fraction"] <= generous["expected_applied_fraction"]
    assert realistic["expected_applied_fraction"] <= realistic["feasible_fraction"] + 1e-12
    assert 0.0 <= realistic["fraction_marginal"] <= 1.0


def test_attempts_budget_uses_the_achievable_yield_not_the_upper_bound():
    """Budgeting on 'a pose exists' would under-provision every cluttered cell."""
    occ = _open_field(12.0)
    occ[100:140, :] = True
    r = feasible_fraction(occ, RES, (0.0, 0.0), radius_m=2.5, band=0.05,
                          clearance_m=0.75, n_refs=120, seed=12)
    if r["expected_applied_fraction"] > 0.0:
        assert r["attempts_per_applied"] == pytest.approx(
            1.0 / r["expected_applied_fraction"], rel=1e-9)


def test_result_is_deterministic_for_a_seed():
    """A budget that changes between runs of the same map is not a budget."""
    occ = _open_field(12.0)
    occ[100:140, :] = True
    kwargs = dict(radius_m=2.0, band=0.05, clearance_m=0.75, n_refs=60, seed=7)
    first = feasible_fraction(occ, RES, (0.0, 0.0), **kwargs)
    second = feasible_fraction(occ, RES, (0.0, 0.0), **kwargs)
    assert first == second


def test_reports_the_attempts_needed_for_a_target_yield():
    """The number PROTOCOL.md actually needs: attempts to budget per applied kidnap."""
    occ = _open_field(30.0)
    result = feasible_fraction(occ, RES, (0.0, 0.0), radius_m=1.5, band=0.05,
                               clearance_m=0.75, n_refs=80, seed=4)
    assert result["attempts_per_applied"] == pytest.approx(1.0, abs=1e-6)
    assert math.isinf(feasible_fraction(
        _open_field(6.0), RES, (0.0, 0.0), radius_m=20.0, band=0.05,
        clearance_m=0.75, n_refs=20, seed=6)["attempts_per_applied"])
