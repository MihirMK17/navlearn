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

"""Map-derived destination ambiguity for the kidnapped-robot recovery analysis.

Purpose
    Quantify how much a laser observation taken at a pose constrains that pose, using
    the static map alone. This is the candidate predictor of kidnap recovery that the
    Paper 1 secondary claim tests against displacement distance.

Definition
    Discretise free space into poses (x, y, theta). Raycast the scan each pose would
    expect to see. For a destination p, score every pose q by the likelihood of observing
    scan(p) at q, normalise over q, and take the entropy of the result. That is the
    posterior a perfectly informed global localizer would hold under a uniform prior:
    low entropy means the observation identifies the pose, high entropy means many places
    in the map look the same from there.

Why entropy of a map-derived likelihood field rather than a geometric proxy
    It measures the quantity the claim is about — how much the observation constrains the
    pose — instead of correlating with it. It is computed before any run, so it cannot be
    contaminated by outcomes. And it is localizer-independent: the resulting claim
    generalises past AMCL rather than describing one filter's tuning. Geometric proxies
    (free area within range, range-profile variance, fraction of max-range returns) are
    cheaper but must stay secondary and pre-declared, since choosing whichever proxy fits
    the data best after the fact is p-hacking with extra steps.

Related work
    Ambiguity Grid Map (Li et al., Sensors 2019) computes a comparable per-cell ambiguity
    from observation-likelihood confusion between poses, but applies it to correcting
    accumulated tracking error while moving through ambiguous regions. The measure here
    is used differently — scored at a teleport destination, before the fact, as a
    predictor of recovery — and that distinction belongs in the paper.

Cost
    One pass over the pose set per map, cached; queries are then a single matrix
    operation. For the campaign map at 0.5 m and 45 degrees this is order 10^8 vectorised
    operations, seconds in numpy, and is evaluated only at destinations actually used.

Usage
    python3 map_ambiguity.py <map.yaml> --targets targets.csv --out ambiguity.csv
"""

import argparse
import csv
import math
import os
import sys

import numpy as np
import yaml

# Ray sampling step as a fraction of a map cell. Half a cell cannot step over a
# single-cell wall, which a full-cell step can when the ray runs near a diagonal.
_STEP_CELLS = 0.5

# Cap on elements materialised per raycast chunk. The intermediate is
# (poses x beams x steps) float64; the campaign map at full resolution would otherwise
# allocate close to a gigabyte in one array.
_CHUNK_ELEMENTS = 8_000_000


class OccupancyMap:
    """A static occupancy grid in the conventions ROS map_server uses.

    Attributes
        occupied: bool array (rows, cols); row 0 is the map origin, y increases with row.
        free: bool array of the same shape. A cell is neither when it is unknown.
        resolution: metres per cell.
        origin: (x, y) of cell (0, 0)'s lower-left corner, in the map frame.
    """

    def __init__(self, occupied, free, resolution, origin):
        """Store the grid and its geometry."""
        self.occupied = occupied
        self.free = free
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))

    @property
    def shape(self):
        """Grid shape as (rows, cols)."""
        return self.occupied.shape

    def world_to_cell(self, x, y):
        """Map a world coordinate to its (row, col) cell index."""
        col = int(math.floor((x - self.origin[0]) / self.resolution))
        row = int(math.floor((y - self.origin[1]) / self.resolution))
        return row, col


def load_occupancy(map_yaml_path):
    """Load a ROS map_server YAML/image pair into an OccupancyMap.

    The image is stored top-down while an occupancy grid's first row is the map origin,
    so the loaded array is flipped vertically. Getting that backwards mirrors the world
    and yields a perfectly plausible ambiguity field for a map that does not exist.
    """
    from PIL import Image

    with open(map_yaml_path) as handle:
        meta = yaml.safe_load(handle)

    image_path = meta["image"]
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(os.path.abspath(map_yaml_path)),
                                  image_path)

    pixels = np.asarray(Image.open(image_path).convert("L"), dtype=np.float64)
    pixels = np.flipud(pixels)

    # map_server's occupancy convention: p = (255 - pixel) / 255, inverted when negate.
    if int(meta.get("negate", 0)):
        occupancy = pixels / 255.0
    else:
        occupancy = (255.0 - pixels) / 255.0

    occupied = occupancy > float(meta.get("occupied_thresh", 0.65))
    free = occupancy < float(meta.get("free_thresh", 0.196))

    return OccupancyMap(occupied, free, meta["resolution"], meta["origin"])


def resample_occupancy(occupied, from_resolution, to_resolution):
    """Coarsen an occupancy grid to a target resolution, conservatively.

    Why this exists
        The campaign's three maps ship at different resolutions. Ambiguity is computed
        over a discretised pose set with a raycast stepped in cells, so the resolution is
        part of the measurement. Scoring maps at their native resolutions would confound
        "this map is more ambiguous" with "this map was sampled more finely", and the map
        axis of the campaign rests entirely on that comparison being clean.

    Why any-occupied rather than averaging or majority vote
        A wall thinner than the coarse cell would be outvoted in every block it belongs
        to and would vanish. A map whose walls dissolve reads as open free space and
        scores as maximally ambiguous for a reason that has nothing to do with the
        building. Over-reporting obstacles is the safe direction here.

    Upsampling is refused: it would invent detail the survey never captured.
    """
    occupied = np.asarray(occupied, dtype=bool)
    if to_resolution < from_resolution * (1.0 - 1e-9):
        raise ValueError(
            f"cannot resample {from_resolution} m to a finer {to_resolution} m; "
            "upsampling would invent detail the map does not contain")

    factor = to_resolution / from_resolution
    if abs(factor - 1.0) < 1e-9:
        return occupied.copy()

    rows, cols = occupied.shape
    out_rows = int(math.floor(rows / factor))
    out_cols = int(math.floor(cols / factor))
    if out_rows < 1 or out_cols < 1:
        raise ValueError("target resolution coarser than the whole map")

    # Block boundaries in fine-cell indices. Computed per output cell rather than by
    # reshaping, so a non-integer factor (0.02 m to 0.05 m is 2.5) is handled exactly
    # instead of being rounded into drift that accumulates across the grid.
    row_edges = np.floor(np.arange(out_rows + 1) * factor).astype(np.int64)
    col_edges = np.floor(np.arange(out_cols + 1) * factor).astype(np.int64)

    # Cumulative counts give each block's occupied total in constant time per cell.
    integral = np.zeros((rows + 1, cols + 1), dtype=np.int64)
    integral[1:, 1:] = np.cumsum(np.cumsum(occupied.astype(np.int64), axis=0), axis=1)

    r0, r1 = row_edges[:-1, None], row_edges[1:, None]
    c0, c1 = col_edges[None, :-1], col_edges[None, 1:]
    block_sum = (integral[r1, c1] - integral[r0, c1]
                 - integral[r1, c0] + integral[r0, c0])
    return block_sum > 0


class AmbiguityField:
    """Entropy of the map-derived likelihood field, evaluated at arbitrary poses.

    Parameters
        occupied: bool array (rows, cols), row 0 at the origin.
        resolution: metres per cell.
        origin: (x, y) of cell (0, 0).
        free: bool array; defaults to the complement of `occupied`. Pass it explicitly
            for a map with unknown cells, which are candidate poses for neither.
        pose_spacing_m: lattice spacing of the candidate pose set (default 0.5).
        yaw_bins: headings per position (default 8, i.e. 45 degrees).
        n_beams: beams per simulated scan (default 36). Beam 0 points along the heading.
        max_range_m: sensor range; beams reaching nothing report exactly this.
        sigma_m: range noise the likelihood assumes, in metres (AMCL's sigma_hit).
        z_hit, z_rand: mixture weights of the per-beam likelihood (AMCL's z_hit, z_rand).
            The uniform z_rand term is not optional decoration. It floors how far a single
            mismatched beam can drive a pose's likelihood; with only the Gaussian term, one
            bad beam sends a pose to zero probability and the posterior collapses to a
            delta whatever the map looks like, so every destination scores as perfectly
            unambiguous by construction.
    """

    def __init__(self, occupied, resolution, origin=(0.0, 0.0), *, free=None,
                 pose_spacing_m=0.5, yaw_bins=8, n_beams=36,
                 max_range_m=12.0, sigma_m=0.20, z_hit=0.85, z_rand=0.05):
        """Build the candidate pose set; expected scans are computed on first use."""
        self.occupied = np.asarray(occupied, dtype=bool)
        self.free = (~self.occupied) if free is None else np.asarray(free, dtype=bool)
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))
        self.pose_spacing_m = float(pose_spacing_m)
        self.yaw_bins = int(yaw_bins)
        self.n_beams = int(n_beams)
        self.max_range_m = float(max_range_m)
        self.sigma_m = float(sigma_m)
        self.z_hit = float(z_hit)
        self.z_rand = float(z_rand)

        if self.sigma_m <= 0.0:
            raise ValueError("sigma_m must be positive")
        if self.z_hit <= 0.0 or self.z_rand < 0.0:
            raise ValueError("z_hit must be positive and z_rand non-negative")
        if self.yaw_bins < 1 or self.n_beams < 1:
            raise ValueError("yaw_bins and n_beams must be positive")

        self._beam_angles = np.linspace(0.0, 2.0 * math.pi, self.n_beams, endpoint=False)
        self.poses = self._build_poses()
        self._scans = None

    # ------------------------------------------------------------------ pose set

    def _build_poses(self):
        """Lattice of free positions crossed with the heading bins."""
        rows, cols = self.free.shape
        span_x = cols * self.resolution
        span_y = rows * self.resolution

        # Half-spacing inset so a lattice point never sits exactly on a cell boundary,
        # where floor() would make the sampled cell depend on floating-point noise.
        xs = np.arange(self.origin[0] + self.pose_spacing_m / 2.0,
                       self.origin[0] + span_x, self.pose_spacing_m)
        ys = np.arange(self.origin[1] + self.pose_spacing_m / 2.0,
                       self.origin[1] + span_y, self.pose_spacing_m)

        grid_x, grid_y = np.meshgrid(xs, ys, indexing="xy")
        flat_x = grid_x.ravel()
        flat_y = grid_y.ravel()

        keep = ~self._occupied_mask(flat_x, flat_y)
        flat_x, flat_y = flat_x[keep], flat_y[keep]

        yaws = np.linspace(0.0, 2.0 * math.pi, self.yaw_bins, endpoint=False)
        n_pos = flat_x.size
        poses = np.empty((n_pos * self.yaw_bins, 3), dtype=np.float64)
        poses[:, 0] = np.repeat(flat_x, self.yaw_bins)
        poses[:, 1] = np.repeat(flat_y, self.yaw_bins)
        poses[:, 2] = np.tile(yaws, n_pos)
        return poses

    def _cell_indices(self, x, y):
        """Vectorised world-to-cell, with an in-bounds mask."""
        col = np.floor((np.asarray(x) - self.origin[0]) / self.resolution).astype(np.int64)
        row = np.floor((np.asarray(y) - self.origin[1]) / self.resolution).astype(np.int64)
        rows, cols = self.free.shape
        inside = (row >= 0) & (row < rows) & (col >= 0) & (col < cols)
        return row, col, inside

    def _occupied_mask(self, x, y):
        """True where the world point is a wall or outside the known map.

        Out of bounds counts as unavailable rather than free: a pose there has no map to
        be localized against, so it is not a candidate.
        """
        row, col, inside = self._cell_indices(x, y)
        result = np.ones(np.shape(row), dtype=bool)
        if inside.any():
            safe_row = np.where(inside, row, 0)
            safe_col = np.where(inside, col, 0)
            result[inside] = ~self.free[safe_row[inside], safe_col[inside]]
        return result

    def is_occupied(self, x, y):
        """True when (x, y) is not free space."""
        return bool(self._occupied_mask(np.array([x]), np.array([y]))[0])

    # ---------------------------------------------------------------- raycasting

    def _raycast(self, poses):
        """Expected ranges for each pose, shape (n_poses, n_beams).

        Rays are marched at half-cell steps and stop at the first occupied cell. A ray
        that leaves the grid keeps going: outside the map is unknown, not a wall, and
        treating it as a return would invent a surface that is not in the map.
        """
        poses = np.atleast_2d(np.asarray(poses, dtype=np.float64))
        step = self.resolution * _STEP_CELLS
        n_steps = max(1, int(math.ceil(self.max_range_m / step)))
        distances = (np.arange(1, n_steps + 1, dtype=np.float64) * step)
        distances = np.minimum(distances, self.max_range_m)

        rows, cols = self.occupied.shape
        out = np.empty((poses.shape[0], self.n_beams), dtype=np.float64)

        per_pose = self.n_beams * n_steps
        chunk = max(1, _CHUNK_ELEMENTS // max(1, per_pose))

        for start in range(0, poses.shape[0], chunk):
            block = poses[start:start + chunk]
            angles = block[:, 2, None] + self._beam_angles[None, :]      # (m, B)
            px = block[:, 0, None, None] + np.cos(angles)[:, :, None] * distances
            py = block[:, 1, None, None] + np.sin(angles)[:, :, None] * distances

            col = np.floor((px - self.origin[0]) / self.resolution).astype(np.int64)
            row = np.floor((py - self.origin[1]) / self.resolution).astype(np.int64)
            inside = (row >= 0) & (row < rows) & (col >= 0) & (col < cols)

            np.clip(row, 0, rows - 1, out=row)
            np.clip(col, 0, cols - 1, out=col)
            hit = self.occupied[row, col] & inside                       # (m, B, S)

            any_hit = hit.any(axis=2)
            first = np.argmax(hit, axis=2)
            out[start:start + chunk] = np.where(any_hit, distances[first],
                                                self.max_range_m)
        return out

    @property
    def scans(self):
        """Expected scans for the candidate pose set, computed once and cached."""
        if self._scans is None:
            self._scans = self._raycast(self.poses)
        return self._scans

    def expected_scan(self, x, y, yaw):
        """The scan a noiseless sensor would return at this pose."""
        return self._raycast(np.array([[x, y, yaw]]))[0]

    # ----------------------------------------------------------------- ambiguity

    def _entropy_from_scans(self, query_scans):
        """Entropy in bits of the normalised likelihood over the pose set."""
        # Per-beam mixture, matching AMCL's beam model: a Gaussian around the expected
        # range plus a uniform random-measurement term. The uniform term is what keeps
        # this from being a delta — see the class docstring on z_rand.
        peak = self.z_hit / (math.sqrt(2.0 * math.pi) * self.sigma_m)
        floor = self.z_rand / self.max_range_m

        diff = query_scans[:, None, :] - self.scans[None, :, :]
        beam = peak * np.exp(-0.5 * (diff * diff) / (self.sigma_m ** 2)) + floor
        loglik = np.sum(np.log(beam), axis=2)

        loglik -= loglik.max(axis=1, keepdims=True)
        weights = np.exp(loglik)
        weights /= weights.sum(axis=1, keepdims=True)

        # 0 log 0 is 0; guard the log rather than the product so no NaN can propagate.
        safe = np.where(weights > 0.0, weights, 1.0)
        entropy = -np.sum(weights * np.log2(safe), axis=1)

        # A near-delta posterior rounds to a small negative. A bit count cannot be
        # negative, and one that is survives a log transform as NaN.
        return np.maximum(entropy, 0.0)

    def ambiguity(self, x, y, yaw):
        """Entropy in bits of the posterior over poses, given the scan expected at (x, y, yaw).

        Raises ValueError when the pose is not in free space: a destination inside a wall
        is an upstream bug, and returning a number would let a mis-transformed kidnap
        target enter the regression as an ordinary observation.
        """
        if self.is_occupied(x, y):
            raise ValueError(
                f"pose ({x:.3f}, {y:.3f}) is not in free space; "
                "check the map frame and the kidnap target transform")
        return float(self._entropy_from_scans(self.expected_scan(x, y, yaw)[None, :])[0])

    def ambiguity_many(self, poses):
        """Ambiguity for many poses at once, shape (M,). Same result as looping."""
        poses = np.atleast_2d(np.asarray(poses, dtype=np.float64))
        bad = self._occupied_mask(poses[:, 0], poses[:, 1])
        if bad.any():
            raise ValueError(
                f"{int(bad.sum())} of {poses.shape[0]} poses are not in free space; "
                f"first offender: {poses[bad][0][:2]}")
        return self._entropy_from_scans(self._raycast(poses))


class LikelihoodFieldAmbiguity(AmbiguityField):
    """Destination ambiguity under AMCL's own measurement model, not an ideal one.

    The parent class compares full expected scans — the information available to a
    perfectly informed localizer. AMCL's likelihood-field model uses strictly less:

      * it projects the measured beam endpoints from each hypothesis and scores each by
        the distance to the NEAREST obstacle, discarding which obstacle it is, so an
        endpoint near any wall looks as good as an endpoint near the right wall;
      * that distance is capped at laser_likelihood_max_dist, so once an endpoint is in
        deep free space it stops mattering how deep;
      * beams that return max range are discarded entirely, so a destination whose scan
        mostly maxes out is scored on the few beams that remain.

    A map can therefore be unambiguous to the ideal model and still confusing to AMCL.
    The gap between the two measures at the same pose is not noise — it is the
    information AMCL's model throws away, and it is where the filter actually fails.

    Extra parameters
        likelihood_max_dist_m: AMCL's laser_likelihood_max_dist (deployed: 6.0 tuned,
            3.0 default).
    """

    def __init__(self, occupied, resolution, origin=(0.0, 0.0), *, free=None,
                 pose_spacing_m=0.5, yaw_bins=8, n_beams=36,
                 max_range_m=12.0, sigma_m=0.20, z_hit=0.85, z_rand=0.05,
                 likelihood_max_dist_m=2.0):
        """Build the pose set and the distance-to-obstacle field AMCL scores against."""
        super().__init__(occupied, resolution, origin, free=free,
                         pose_spacing_m=pose_spacing_m, yaw_bins=yaw_bins,
                         n_beams=n_beams, max_range_m=max_range_m, sigma_m=sigma_m,
                         z_hit=z_hit, z_rand=z_rand)
        from scipy.ndimage import distance_transform_edt

        self.likelihood_max_dist_m = float(likelihood_max_dist_m)
        if self.likelihood_max_dist_m <= 0.0:
            raise ValueError("likelihood_max_dist_m must be positive")

        # Distance from every free cell to the nearest occupied cell, in metres, capped
        # exactly as AMCL caps it. Endpoints outside the grid score the cap: there is
        # nothing known to be near them, which is the same statement the cap makes.
        edt = distance_transform_edt(~self.occupied) * self.resolution
        self._clamped_edt = np.minimum(edt, self.likelihood_max_dist_m)

    def ambiguity(self, x, y, yaw, radius_m=None):
        """Entropy of AMCL's likelihood over hypotheses, given the scan expected here.

        radius_m restricts the hypothesis set to a disc around the destination. That is
        the post-kidnap regime: the filter's belief is already concentrated, recovery
        hinges on whether hypotheses near the truth out-score each other once particles
        land there, and the competition is local, not map-wide. None means the full pose
        set — the global-relocalization regime.

        Returns a dict:
            entropy_bits: ambiguity of the destination under this model.
            max_range_fraction: fraction of beams AMCL would discard at this pose. 1.0
                means the observation has no content at all under the deployed model.
            n_hypotheses: size of the competition set the entropy is over.
        """
        if self.is_occupied(x, y):
            raise ValueError(
                f"pose ({x:.3f}, {y:.3f}) is not in free space; "
                "check the map frame and the kidnap target transform")

        candidates = self.poses
        if radius_m is not None:
            keep = np.hypot(candidates[:, 0] - x, candidates[:, 1] - y) <= float(radius_m)
            candidates = candidates[keep]
            if candidates.shape[0] == 0:
                raise ValueError(f"no candidate poses within {radius_m} m of ({x}, {y})")

        scan = self.expected_scan(x, y, yaw)
        valid = scan < (self.max_range_m - 1e-9)
        max_range_fraction = float(1.0 - valid.mean())

        n = candidates.shape[0]
        if not valid.any():
            # Every beam discarded: the posterior is the prior, exactly.
            return {"entropy_bits": float(math.log2(n)),
                    "max_range_fraction": max_range_fraction,
                    "n_hypotheses": n}

        ranges = scan[valid]
        angles = candidates[:, 2, None] + self._beam_angles[valid][None, :]
        ex = candidates[:, 0, None] + ranges[None, :] * np.cos(angles)
        ey = candidates[:, 1, None] + ranges[None, :] * np.sin(angles)

        rows, cols = self.occupied.shape
        col = np.floor((ex - self.origin[0]) / self.resolution).astype(np.int64)
        row = np.floor((ey - self.origin[1]) / self.resolution).astype(np.int64)
        inside = (row >= 0) & (row < rows) & (col >= 0) & (col < cols)
        np.clip(row, 0, rows - 1, out=row)
        np.clip(col, 0, cols - 1, out=col)

        d = np.where(inside, self._clamped_edt[row, col], self.likelihood_max_dist_m)

        peak = self.z_hit
        floor = self.z_rand / self.max_range_m
        beam = peak * np.exp(-0.5 * (d * d) / (self.sigma_m ** 2)) + floor
        loglik = np.sum(np.log(beam), axis=1)

        loglik -= loglik.max()
        weights = np.exp(loglik)
        weights /= weights.sum()

        safe = np.where(weights > 0.0, weights, 1.0)
        entropy = float(max(0.0, -np.sum(weights * np.log2(safe))))

        return {"entropy_bits": entropy,
                "max_range_fraction": max_range_fraction,
                "n_hypotheses": n}


# --------------------------------------------------------------------- degeneracy guard

# A predictor is unusable below these. The thresholds are properties of the measure, not
# of any outcome, and are fixed here so the check cannot be relaxed after seeing results.
_MIN_IQR_BITS = 0.05
_MAX_ZERO_FRACTION = 0.90
_ZERO_BITS = 0.01


def spread_report(values):
    """Summarise a set of ambiguity scores and say whether they can predict anything.

    A predictor with no variance cannot distinguish anything, and a model containing it
    will return a null result that looks like evidence against the hypothesis when it is
    really a statement that the measure had nothing to say on this map. That distinction
    is invisible once the numbers reach a regression table, so it is made here.
    """
    values = np.asarray(values, dtype=np.float64).ravel()
    if values.size == 0:
        raise ValueError("no values to summarise")

    q25, q50, q75 = np.percentile(values, [25, 50, 75])
    iqr = float(q75 - q25)
    zero_fraction = float(np.mean(values < _ZERO_BITS))

    return {
        "count": int(values.size),
        "min": float(values.min()),
        "p25": float(q25),
        "median": float(q50),
        "p75": float(q75),
        "max": float(values.max()),
        "iqr": iqr,
        "zero_fraction": zero_fraction,
        "degenerate": bool(iqr < _MIN_IQR_BITS or zero_fraction > _MAX_ZERO_FRACTION),
    }


# --------------------------------------------------------------------------- entry point


def _read_targets(path):
    """Read kidnap destinations from a metrics CSV or a bare x,y,yaw file.

    Only applied kidnaps are scored. An attempt that never moved the robot has no
    destination the robot ever observed, and scoring it would put a pose the experiment
    never visited into the regression.
    """
    with open(path) as handle:
        rows = list(csv.DictReader(handle))

    targets = []
    for index, row in enumerate(rows):
        if "Kidnap Target_X (m)" in row:
            if row.get("Kidnap Applied", "0").strip() != "1":
                continue
            x = float(row["Kidnap Target_X (m)"])
            y = float(row["Kidnap Target_Y (m)"])
            yaw = math.radians(float(row["Kidnap Target_Yaw (deg)"]))
        else:
            x, y, yaw = float(row["x"]), float(row["y"]), float(row.get("yaw", 0.0))
        targets.append((index, x, y, yaw))
    return targets


def main(argv=None):
    """Score every kidnap destination in a metrics CSV against a map."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("map_yaml", help="ROS map_server YAML for the campaign map")
    parser.add_argument("--targets", required=True,
                        help="metrics CSV (uses the Kidnap Target columns), or x,y,yaw CSV")
    parser.add_argument("--out", required=True, help="output CSV path")
    parser.add_argument("--pose-spacing", type=float, default=0.5)
    parser.add_argument("--yaw-bins", type=int, default=8)
    parser.add_argument("--beams", type=int, default=36)
    parser.add_argument("--max-range", type=float, default=12.0,
                        help="metres; match the sensor the campaign ran (RPLidar A1)")
    parser.add_argument("--sigma", type=float, default=0.20, help="range sigma, metres")
    parser.add_argument("--z-hit", type=float, default=0.85)
    parser.add_argument("--z-rand", type=float, default=0.05)
    args = parser.parse_args(argv)

    occ = load_occupancy(args.map_yaml)
    field = AmbiguityField(
        occ.occupied, occ.resolution, occ.origin, free=occ.free,
        pose_spacing_m=args.pose_spacing, yaw_bins=args.yaw_bins,
        n_beams=args.beams, max_range_m=args.max_range, sigma_m=args.sigma,
        z_hit=args.z_hit, z_rand=args.z_rand)

    targets = _read_targets(args.targets)
    if not targets:
        print("no applied kidnap targets found", file=sys.stderr)
        return 1

    print(f"pose set: {len(field.poses)} poses "
          f"({len(field.poses) // args.yaw_bins} positions x {args.yaw_bins} headings)",
          file=sys.stderr)

    scored = field.ambiguity_many(np.array([[x, y, yaw] for _, x, y, yaw in targets]))

    report = spread_report(scored)
    print("ambiguity (bits): min {min:.3f}  p25 {p25:.3f}  median {median:.3f}  "
          "p75 {p75:.3f}  max {max:.3f}  IQR {iqr:.3f}".format(**report), file=sys.stderr)

    with open(args.out, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["row_index", "target_x", "target_y", "target_yaw_rad",
                         "ambiguity_bits", "max_entropy_bits"])
        ceiling = math.log2(len(field.poses))
        for (index, x, y, yaw), value in zip(targets, scored):
            writer.writerow([index, x, y, yaw, value, ceiling])

    print(f"wrote {len(targets)} rows to {args.out}", file=sys.stderr)

    if report["degenerate"]:
        print(
            "\nDEGENERATE: this predictor has no usable variance on this map "
            f"(IQR {report['iqr']:.4f} bits, {report['zero_fraction']:.1%} of targets "
            f"below {_ZERO_BITS} bits).\n"
            "A regression on it cannot distinguish anything, and the null result it "
            "produces is NOT evidence against the ambiguity hypothesis — it means the "
            "map does not vary in the quantity being tested. Do not report it as a "
            "finding. Either score destinations on a map with genuine perceptual "
            "aliasing, or drop the claim.",
            file=sys.stderr)
        return 2

    return 0


if __name__ == "__main__":
    sys.exit(main())
