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

"""Tests for the map-derived destination ambiguity measure.

What is being measured
    For a destination pose p, the ambiguity is the entropy of the posterior a perfectly
    informed global localizer would hold after observing the scan expected at p, under a
    uniform prior over free space. High entropy means the observation does not pin the
    pose down: many places in the map look the same from there.

Why it is tested against hand-checkable geometry rather than a golden file
    The measure only earns its place in the paper if it means what it claims to mean. A
    golden-value test would pin the implementation without ever checking that a corridor
    scores as ambiguous, and the failure mode that matters is a measure that is
    self-consistent and wrong. Every assertion here is a geometric fact about the test
    map that would hold for any correct implementation.
"""

import math
import os
import sys

import pytest

np = pytest.importorskip("numpy", reason="numpy not available")
pytest.importorskip("PIL", reason="Pillow not available; needed to read the map image")

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))

from map_ambiguity import (  # noqa: E402
    AmbiguityField,
    LikelihoodFieldAmbiguity,
    load_occupancy,
    resample_occupancy,
    spread_report,
)

RES = 0.1


def _blank(width_m, height_m, res=RES):
    """An enclosed rectangular room: free interior, one cell of wall all round."""
    cols = int(round(width_m / res))
    rows = int(round(height_m / res))
    occupied = np.zeros((rows, cols), dtype=bool)
    occupied[0, :] = True
    occupied[-1, :] = True
    occupied[:, 0] = True
    occupied[:, -1] = True
    return occupied


def _carved(width_m, height_m, rects, res=RES):
    """An all-occupied grid with the given world rectangles carved out as free space."""
    cols = int(round(width_m / res))
    rows = int(round(height_m / res))
    occupied = np.ones((rows, cols), dtype=bool)
    for x0, y0, x1, y1 in rects:
        occupied[int(round(y0 / res)):int(round(y1 / res)),
                 int(round(x0 / res)):int(round(x1 / res))] = False
    return occupied


def _field(occupied, **kwargs):
    """Build an AmbiguityField over a grid whose origin is at (0, 0)."""
    params = dict(pose_spacing_m=0.5, yaw_bins=8, n_beams=36,
                  max_range_m=6.0, sigma_m=0.15)
    params.update(kwargs)
    return AmbiguityField(occupied, resolution=RES, origin=(0.0, 0.0), **params)


# --------------------------------------------------------------------------- raycasting


def test_raycast_measures_distance_to_wall():
    """Beam 0 points along the pose's heading; its range is the distance to the wall.

    A 4 m x 4 m room with 0.1 m walls: from (2.0, 2.0) facing +x the inner wall face is
    at x = 3.9, so the range is 1.9 m, within one cell.
    """
    field = _field(_blank(4.0, 4.0))
    scan = field.expected_scan(2.0, 2.0, 0.0)
    assert scan[0] == pytest.approx(1.9, abs=RES)


def test_raycast_follows_the_heading():
    """Rotating the pose rotates the observation; a long room proves it."""
    field = _field(_blank(8.0, 3.0))
    ahead = field.expected_scan(4.0, 1.5, 0.0)[0]          # down the long axis
    across = field.expected_scan(4.0, 1.5, math.pi / 2)[0]  # into the near wall
    assert ahead == pytest.approx(3.9, abs=RES)
    assert across == pytest.approx(1.4, abs=RES)


def test_raycast_saturates_at_max_range():
    """A beam that reaches nothing within max_range reports max_range, not infinity."""
    field = _field(_blank(20.0, 20.0), max_range_m=2.0)
    scan = field.expected_scan(10.0, 10.0, 0.0)
    assert np.all(scan <= 2.0 + 1e-9)
    assert scan[0] == pytest.approx(2.0, abs=1e-9)


# ---------------------------------------------------------------------------- ambiguity


def test_entropy_is_bounded_by_log2_of_the_pose_count():
    """Entropy of a distribution over N poses cannot exceed log2(N), or it is not one."""
    field = _field(_blank(4.0, 4.0))
    h = field.ambiguity(2.0, 2.0, 0.0)
    assert 0.0 <= h <= math.log2(len(field.poses)) + 1e-9


def test_corridor_is_more_ambiguous_than_a_distinctive_corner():
    """The hand-checkable case: a featureless corridor tells a localizer almost nothing.

    Both queries are scored in ONE map, against ONE pose set. Comparing entropies from
    two separate maps would compare two different ceilings — log2(N) differs with the
    number of free poses — and a corridor could then score lower in bits while being more
    ambiguous, which would make the test pass or fail for the wrong reason.

    Mid-corridor, with the sensor unable to reach either end, every position along the
    run produces the same observation. The room corner sees two walls meeting at a fixed
    distance, a pattern only the corners satisfy.
    """
    l_shaped = _carved(14.0, 6.0, [
        (0.1, 0.1, 10.0, 1.1),   # long featureless corridor
        (10.0, 0.1, 13.9, 5.9),  # room at the far end
    ])
    field = _field(l_shaped, max_range_m=2.0)

    corridor_h = field.ambiguity(5.0, 0.6, 0.0)
    corner_h = field.ambiguity(13.6, 5.6, math.pi / 4)

    assert corridor_h > corner_h, (
        f"corridor {corridor_h:.3f} bits should exceed corner {corner_h:.3f} bits")


def test_adding_a_landmark_reduces_ambiguity():
    """Same map, same query pose, one distinctive feature added: ambiguity must fall.

    A differential test needs no absolute calibration, so it holds whatever the units or
    the discretisation. If cutting a unique alcove into an otherwise featureless corridor
    does not make the pose beside it easier to identify, the measure is not measuring
    identifiability.

    Sensor range is capped below the corridor length so the baseline is genuinely
    ambiguous; with the ends in view, distance-to-end already identifies the pose and the
    alcove would have nothing left to contribute.
    """
    corridor = [(0.1, 0.1, 11.9, 1.1)]
    plain = _carved(12.0, 2.0, corridor)
    notched = _carved(12.0, 2.0, corridor + [(5.0, 1.1, 5.4, 1.6)])

    query = (5.2, 0.6, math.pi / 2)  # facing the alcove
    assert (_field(notched, max_range_m=2.0).ambiguity(*query)
            < _field(plain, max_range_m=2.0).ambiguity(*query))


def test_ambiguity_is_deterministic():
    """No RNG anywhere: the same map and pose give bit-identical answers."""
    occupied = _blank(5.0, 5.0)
    first = _field(occupied).ambiguity(2.5, 2.5, 0.0)
    second = _field(occupied).ambiguity(2.5, 2.5, 0.0)
    assert first == second


def test_batch_matches_single_queries():
    """The vectorised path used on the campaign's destinations must not diverge.

    Ambiguity is scored for a couple of hundred kidnap targets at once; if that path
    disagreed with the single-pose one, the paper's numbers would come from code no test
    ever checked.
    """
    field = _field(_blank(5.0, 5.0))
    queries = np.array([
        [2.5, 2.5, 0.0],
        [1.0, 1.0, math.pi / 2],
        [4.0, 2.0, math.pi],
    ])
    batch = field.ambiguity_many(queries)
    singles = [field.ambiguity(x, y, yaw) for x, y, yaw in queries]
    assert np.allclose(batch, singles, atol=1e-12)


def test_pose_outside_free_space_is_refused():
    """A destination in a wall is a bug upstream, not a zero-ambiguity pose.

    Returning a number here would let a mis-transformed kidnap target enter the
    regression as an ordinary observation.
    """
    field = _field(_blank(4.0, 4.0))
    with pytest.raises(ValueError):
        field.ambiguity(0.0, 0.0, 0.0)


def test_pose_set_excludes_occupied_cells():
    """The uniform prior is over free space; walls are not candidate poses."""
    field = _field(_blank(4.0, 4.0))
    for x, y, _ in field.poses:
        assert not field.is_occupied(x, y), f"pose ({x}, {y}) is inside an obstacle"


def test_entropy_is_never_negative():
    """Rounding may drive a near-delta posterior slightly below zero; a bit count cannot be.

    Reported ambiguities of -0.000 are harmless to read and corrosive to analyse: they
    survive a log transform as NaN and a sign test as a negative.
    """
    field = _field(_blank(6.0, 6.0), sigma_m=0.05)
    values = field.ambiguity_many(field.poses[:64])
    assert np.all(values >= 0.0)


def test_higher_random_measurement_weight_increases_ambiguity():
    """z_rand is a floor on how badly one beam can condemn a pose, so it raises entropy.

    AMCL's likelihood is a mixture: a Gaussian around the expected range plus a uniform
    random-measurement term. Without the uniform part, a single grossly mismatched beam
    drives a pose's likelihood to zero and the posterior collapses to a delta regardless
    of how similar the map actually looks. Modelling only the Gaussian would make every
    destination score as unambiguous by construction.
    """
    occupied = _carved(12.0, 2.0, [(0.1, 0.1, 11.9, 1.1)])
    query = (5.2, 0.6, 0.0)
    tight = _field(occupied, max_range_m=2.0, z_rand=0.01).ambiguity(*query)
    loose = _field(occupied, max_range_m=2.0, z_rand=0.40).ambiguity(*query)
    assert loose > tight


# ------------------------------------------------------- AMCL likelihood-field ambiguity
#
# The AmbiguityField above scores poses by comparing full expected scans — the information
# available to an IDEAL localizer. AMCL does not do that. Its likelihood-field model
# projects the measured beam endpoints from each hypothesis and scores only the distance
# from each endpoint to the NEAREST obstacle, capped at laser_likelihood_max_dist, and it
# discards beams at max range entirely. Both steps throw information away, so a map can be
# unambiguous to the ideal model and still confusing to AMCL. The RCA question is exactly
# that gap, so the deployed model needs its own measure.


def _lf(occupied, **kwargs):
    """Build a LikelihoodFieldAmbiguity over a grid whose origin is at (0, 0)."""
    params = dict(pose_spacing_m=0.5, yaw_bins=8, n_beams=36,
                  max_range_m=6.0, sigma_m=0.1, z_hit=0.85, z_rand=0.05,
                  likelihood_max_dist_m=2.0)
    params.update(kwargs)
    return LikelihoodFieldAmbiguity(occupied, resolution=RES, origin=(0.0, 0.0), **params)


def test_lf_all_max_range_beams_carry_no_information():
    """AMCL discards beams at max range; a scan that is all discards constrains nothing.

    From the centre of a room much larger than the sensor range, every beam maxes out,
    every hypothesis scores identically, and the posterior is the prior: entropy must sit
    at the log2(N) ceiling. This is the measurable meaning of 'the lidar fails in open
    space' — not noise, an observation with zero content under the deployed model.
    """
    field = _lf(_blank(30.0, 30.0), max_range_m=5.0)
    result = field.ambiguity(15.0, 15.0, 0.0)
    assert result["max_range_fraction"] == pytest.approx(1.0)
    assert result["entropy_bits"] == pytest.approx(math.log2(len(field.poses)), abs=0.01)


def test_lf_max_range_fraction_is_zero_when_walls_are_in_reach():
    """A pose whose every beam returns must report zero discarded beams."""
    field = _lf(_blank(4.0, 4.0), max_range_m=6.0)
    assert field.ambiguity(2.0, 2.0, 0.0)["max_range_fraction"] == pytest.approx(0.0)


def test_lf_corridor_is_more_ambiguous_than_a_distinctive_corner():
    """The corridor ordering must survive the change of measurement model.

    Sliding a hypothesis along the corridor keeps every projected endpoint on or near a
    wall, so the likelihood field cannot tell positions along the run apart — same
    geometry as the ideal model, different mechanism (endpoint distances, not ranges).
    """
    l_shaped = _carved(14.0, 6.0, [
        (0.1, 0.1, 10.0, 1.1),
        (10.0, 0.1, 13.9, 5.9),
    ])
    field = _lf(l_shaped, max_range_m=2.0)
    corridor = field.ambiguity(5.0, 0.6, 0.0)["entropy_bits"]
    corner = field.ambiguity(13.6, 5.6, math.pi / 4)["entropy_bits"]
    assert corridor > corner


def test_lf_local_ambiguity_only_considers_nearby_hypotheses():
    """The local variant models a filter whose belief is already concentrated.

    Post-kidnap AMCL is not a global localizer: its particles sit in a tight, wrong
    cluster, and recovery hinges on whether hypotheses NEAR the truth out-score each
    other once injected. The competition set is a disc, so the entropy ceiling is the
    log2 of the hypotheses inside the disc, not of the whole map.
    """
    field = _lf(_blank(20.0, 20.0), max_range_m=6.0)
    result = field.ambiguity(10.0, 10.0, 0.0, radius_m=2.0)
    n_local = result["n_hypotheses"]
    assert n_local < len(field.poses)
    assert result["entropy_bits"] <= math.log2(n_local) + 1e-9

    # Every hypothesis in the disc is genuinely within it.
    assert result["n_hypotheses"] == int(np.sum(
        np.hypot(field.poses[:, 0] - 10.0, field.poses[:, 1] - 10.0) <= 2.0))


def test_lf_refuses_a_pose_outside_free_space():
    """Same contract as the ideal measure: a destination in a wall is an upstream bug."""
    field = _lf(_blank(4.0, 4.0))
    with pytest.raises(ValueError):
        field.ambiguity(0.0, 0.0, 0.0)


# -------------------------------------------------------------------- degeneracy guard


def test_spread_report_flags_a_degenerate_measure():
    """A predictor with no variance must announce itself, not be silently regressed on.

    If every destination scores the same, a model containing that predictor cannot
    distinguish anything — and the null result would look like evidence against the
    hypothesis rather than what it is: a measure with nothing to say on this map.
    """
    report = spread_report(np.zeros(200))
    assert report["degenerate"] is True

    varied = spread_report(np.linspace(0.0, 8.0, 200))
    assert varied["degenerate"] is False
    assert varied["median"] == pytest.approx(4.0, abs=0.1)
    assert varied["iqr"] > 0.0


def test_spread_report_flags_a_measure_that_is_almost_all_zero():
    """Near-degenerate counts too: a handful of non-zero scores is not usable variance."""
    values = np.zeros(200)
    values[:2] = 5.0
    assert spread_report(values)["degenerate"] is True


# ------------------------------------------------------------------ resolution normalising
#
# The three campaign maps ship at different resolutions (small_house 0.05 m, warehouse
# 0.02 m, bookstore 0.05 m). Ambiguity is computed over a discretised pose set and a
# raycast stepped in cells, so resolution changes the number. Comparing maps scored at
# different resolutions would confound "this map is more ambiguous" with "this map was
# sampled more finely", and the campaign's whole map axis rests on that comparison.


def test_resample_is_identity_at_the_same_resolution():
    """A no-op resample must not perturb the grid."""
    occupied = _blank(4.0, 4.0)
    out = resample_occupancy(occupied, RES, RES)
    assert out.shape == occupied.shape
    assert np.array_equal(out, occupied)


def test_resample_coarsens_by_the_integer_factor():
    """0.05 m from 0.025 m halves each axis."""
    occupied = np.zeros((40, 60), dtype=bool)
    out = resample_occupancy(occupied, 0.025, 0.05)
    assert out.shape == (20, 30)


def test_resample_is_conservative_about_obstacles():
    """A coarse cell is occupied if ANY of its sub-cells is.

    Averaging or majority voting would erase thin walls: a 0.05 m wall in a 0.02 m grid is
    a single cell, outvoted in every block it belongs to. A map whose walls dissolve reads
    as wide-open free space, which would score as maximally ambiguous for reasons that have
    nothing to do with the building.
    """
    fine = np.zeros((4, 4), dtype=bool)
    fine[0, 0] = True                      # one occupied sub-cell in the first 2x2 block
    out = resample_occupancy(fine, 0.025, 0.05)
    assert out.shape == (2, 2)
    assert out[0, 0], "a wall thinner than the coarse cell was erased"
    assert not out[0, 1] and not out[1, 0] and not out[1, 1]


def test_resample_preserves_a_thin_wall_across_a_whole_grid():
    """End-to-end version of the above on a wall one fine-cell thick."""
    fine = np.zeros((20, 20), dtype=bool)
    fine[10, :] = True
    out = resample_occupancy(fine, 0.025, 0.05)
    assert out[5, :].all(), "thin wall did not survive coarsening"


def test_resample_refuses_to_upsample():
    """Coarse to fine would invent detail the map never had."""
    with pytest.raises(ValueError):
        resample_occupancy(np.zeros((4, 4), dtype=bool), 0.05, 0.025)


def test_resample_handles_a_non_integer_factor():
    """0.02 m to 0.05 m is a factor of 2.5; the grid must still coarsen conservatively."""
    fine = np.zeros((10, 10), dtype=bool)
    fine[3, 3] = True
    out = resample_occupancy(fine, 0.02, 0.05)
    assert out.shape == (4, 4)
    assert out.sum() == 1, "one obstacle cell became more or fewer than one"


# --------------------------------------------------------------------------- map loading


def _campaign_map():
    """Path to the map the campaign actually runs on."""
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.normpath(os.path.join(
        here, "..", "..", "bumperbot_mapping", "maps", "small_house", "map.yaml"))
    if not os.path.isfile(path):
        pytest.skip(f"campaign map not found at {path}")
    return path


def test_campaign_map_loads_with_declared_geometry():
    """The real map parses, and its metadata survives the load.

    Guards the axis convention: a ROS occupancy grid's first row is the map origin, while
    the PGM stores the image top-down. Getting that backwards mirrors the map vertically,
    which produces a perfectly plausible ambiguity field for the wrong world.
    """
    occ = load_occupancy(_campaign_map())
    assert occ.resolution == pytest.approx(0.05)
    assert occ.origin == pytest.approx((-12.5, -12.5))
    assert occ.occupied.dtype == bool
    assert occ.occupied.shape == occ.free.shape
    assert occ.free.any(), "map has no free space; the thresholds are inverted"
    assert not (occ.free & occ.occupied).any(), "a cell cannot be both free and occupied"


def test_campaign_map_origin_maps_to_grid_corner():
    """World-to-grid must place the declared origin at cell (0, 0), x to col and y to row.

    Queried at cell centres, not cell edges. A boundary coordinate is not exactly
    representable in binary — -12.5 + 0.05 * 7 evaluates just below the edge and floors
    into the cell beneath — so asserting there tests floating-point representation rather
    than the axis convention this is meant to pin down.
    """
    occ = load_occupancy(_campaign_map())
    assert occ.world_to_cell(-12.5 + 0.025, -12.5 + 0.025) == (0, 0)
    row, col = occ.world_to_cell(-12.5 + 0.05 * 3.5, -12.5 + 0.05 * 7.5)
    assert (row, col) == (7, 3), "x must index columns and y must index rows"
