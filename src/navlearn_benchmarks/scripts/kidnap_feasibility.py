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

"""How often a kidnap of a given displacement can actually be applied, per map.

Why
    The Gate-0 positive control lost 2 of 10 kidnaps to "no valid pose in the ring". That
    is not a bug: at larger radii a thin annulus around the robot can miss free space
    entirely in a cluttered map, and the sampler correctly reports the attempt as
    unapplied rather than quietly moving the robot somewhere else. But the predictor leg
    needs a fixed number of APPLIED kidnaps per map, so the attrition rate decides how
    many attempts to budget and how wide the magnitude band has to be.

    Both are PROTOCOL.md numbers. Measuring them from the occupancy grid takes seconds;
    measuring them from simulation takes hours and burns the campaign's own goals doing it.

How
    Replicates the node's feasibility rule exactly: a candidate pose must be free space
    AND have a clear disc of `goal_min_clearance_m` around it, which is a distance
    transform test. A reference pose is "feasible at radius r" when at least one pose in
    the annulus [r(1-band), r(1+band)] passes. If the two rules ever drift, the budget
    would describe a different experiment than the one that runs -- hence the tests.

KNOWN LIMITATION -- read before budgeting from the map-level average
    Reference poses are drawn uniformly from valid free space, but a navigating robot's
    poses are not uniform: it spends its time on routes between goals, and those include
    confined rooms where a large-radius ring is mostly wall.

    Measured on the Gate-0 run: this tool reports 1.000 expected yield for small_house at
    every radius and band, while the run itself lost 2 of 10 kidnaps. Checked at the two
    failing reference poses directly, the tool is correct -- their rings held 0 and 4
    valid targets out of 3240 candidates respectively, so the sampler was right to refuse
    and the 4/3240 case is a ~16% miss under 1500 draws. Both poses were in the same
    confined east room.

    So the per-pose computation is sound and the map-level AVERAGE is optimistic. Budget
    attempts from observed attrition, or from this tool evaluated at the actual reference
    poses a pilot recorded (the `Kidnap Reference_X/Y` columns exist for exactly this),
    rather than from the uniform-sample figure.

Usage
    python3 kidnap_feasibility.py --map NAME=path/to/map.yaml [--map ...] \
        --radii 0.3,0.5,1.0,1.5,2.0,2.5,3.0 --bands 0.05,0.10,0.20
"""

import argparse
import json
import math
import os
import sys

import numpy as np

from map_ambiguity import load_occupancy

# Matches episode_manager's goal_min_clearance_m default. Stated as a constant rather than
# inferred, so a change on either side is a visible edit rather than a silent divergence.
DEFAULT_CLEARANCE_M = 0.75


def valid_pose_mask(occupied, resolution, clearance_m=DEFAULT_CLEARANCE_M, free=None):
    """Cells the node would accept: free, and with a clear disc of `clearance_m`.

    Out-of-bounds counts as blocked. The node returns false when the clearance disc would
    leave the grid, so a pose near the map edge is not a candidate there either.
    """
    from scipy.ndimage import distance_transform_edt

    occupied = np.asarray(occupied, dtype=bool)
    free = (~occupied) if free is None else np.asarray(free, dtype=bool)

    # Distance from each cell to the nearest blocked cell, in metres. Cells outside the
    # grid are treated as blocked by padding, matching the node's bounds rejection.
    padded = np.pad(occupied | ~free, 1, mode="constant", constant_values=True)
    edt = distance_transform_edt(~padded) * resolution
    return (edt[1:-1, 1:-1] >= clearance_m) & free


def feasible_fraction(occupied, resolution, origin, *, radius_m, band,
                      clearance_m=DEFAULT_CLEARANCE_M, free=None,
                      n_refs=200, n_bearings=72, n_radii=5, seed=0,
                      max_sample_tries=1500):
    """Fraction of reference poses for which a kidnap at `radius_m` can be applied.

    Reference poses are drawn from the valid set, because that is where the robot is when
    a kidnap fires: sampling them from arbitrary free space would include places the robot
    could never have been and would bias the estimate.
    """
    occupied = np.asarray(occupied, dtype=bool)
    free = (~occupied) if free is None else np.asarray(free, dtype=bool)
    valid = valid_pose_mask(occupied, resolution, clearance_m, free)

    rows, cols = valid.shape
    candidates = np.flatnonzero(valid)
    if candidates.size == 0:
        raise ValueError("map has no valid poses at this clearance")

    rng = np.random.default_rng(seed)
    take = min(n_refs, candidates.size)
    chosen = rng.choice(candidates, size=take, replace=False)
    ref_row, ref_col = np.unravel_index(chosen, valid.shape)
    ref_x = origin[0] + (ref_col + 0.5) * resolution
    ref_y = origin[1] + (ref_row + 0.5) * resolution

    bearings = np.linspace(0.0, 2.0 * math.pi, n_bearings, endpoint=False)
    radii = np.linspace(radius_m * (1.0 - band), radius_m * (1.0 + band), n_radii)

    # (refs, bearings, radii) candidate targets, tested in one pass.
    tx = ref_x[:, None, None] + radii[None, None, :] * np.cos(bearings)[None, :, None]
    ty = ref_y[:, None, None] + radii[None, None, :] * np.sin(bearings)[None, :, None]

    col = np.floor((tx - origin[0]) / resolution).astype(np.int64)
    row = np.floor((ty - origin[1]) / resolution).astype(np.int64)
    inside = (row >= 0) & (row < rows) & (col >= 0) & (col < cols)
    np.clip(row, 0, rows - 1, out=row)
    np.clip(col, 0, cols - 1, out=col)

    ok = valid[row, col] & inside
    per_ref = ok.any(axis=(1, 2))
    fraction = float(per_ref.mean())

    # "A valid pose exists" is not the same as "the sampler finds it". The node draws
    # `max_sample_tries` random poses from the annulus and gives up if none land in free
    # space, so a ring that is 0.1% valid is very likely to be missed even though it is
    # not empty. Verified against the Gate-0 run: a goal whose ring was 4/3240 valid was
    # reported unapplied, which this model puts at a 16% miss -- consistent, where the
    # existence test called it feasible outright.
    valid_share = ok.mean(axis=(1, 2))
    hit_probability = 1.0 - np.power(1.0 - valid_share, max_sample_tries)
    expected_yield = float(hit_probability.mean())

    return {
        "radius_m": float(radius_m),
        "band": float(band),
        "refs_tested": int(take),
        # Fraction of poses where a valid target EXISTS. An upper bound on what the
        # sampler can achieve, not a prediction of what it will.
        "feasible_fraction": fraction,
        # Fraction of poses where the sampler is expected to actually find one. This is
        # the number to budget with.
        "expected_applied_fraction": expected_yield,
        "median_valid_share": float(np.median(valid_share)),
        # Poses whose ring is so sparse the sampler will usually fail even though a pose
        # exists -- the gap between "possible" and "practical".
        "fraction_marginal": float(np.mean((valid_share > 0.0) & (hit_probability < 0.9))),
        "attempts_per_applied": (
            (1.0 / expected_yield) if expected_yield > 0.0 else float("inf")),
    }


def main(argv=None):
    """Report feasibility across radii and bands for each candidate map."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--map", action="append", required=True, metavar="NAME=PATH")
    parser.add_argument("--radii", default="0.3,0.5,0.8,1.2,1.6,2.0,2.5,3.0")
    parser.add_argument("--bands", default="0.05,0.10,0.20")
    parser.add_argument("--clearance", type=float, default=DEFAULT_CLEARANCE_M)
    parser.add_argument("--refs", type=int, default=200)
    parser.add_argument("--seed", type=int, default=20260730)
    parser.add_argument("--out", help="optional JSON output path")
    args = parser.parse_args(argv)

    radii = [float(v) for v in args.radii.split(",")]
    bands = [float(v) for v in args.bands.split(",")]
    report = []

    for spec in args.map:
        name, path = spec.split("=", 1)
        if not os.path.isfile(path):
            parser.error(f"{name}: no such map yaml: {path}")
        occ = load_occupancy(path)
        print(f"\n=== {name} (clearance {args.clearance} m) ===")
        header = "  radius " + "".join(f"  band={b:<6.2f}" for b in bands)
        print(header)
        for r in radii:
            row = f"  {r:6.2f} "
            for b in bands:
                res = feasible_fraction(
                    occ.occupied, occ.resolution, occ.origin, radius_m=r, band=b,
                    clearance_m=args.clearance, free=occ.free, n_refs=args.refs,
                    seed=args.seed)
                res["map"] = name
                report.append(res)
                row += f"   {res['feasible_fraction']:6.3f}    "
            print(row)

    print("\n(values are the fraction of robot poses at which a kidnap of that "
          "displacement can be applied)")

    if args.out:
        os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
        with open(args.out, "w") as handle:
            json.dump({"clearance_m": args.clearance, "seed": args.seed,
                       "results": report}, handle, indent=2)
        print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
