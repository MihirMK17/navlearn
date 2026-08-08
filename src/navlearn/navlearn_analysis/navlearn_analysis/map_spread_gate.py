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

"""Pre-registered map-selection gate for the three-map campaign.

What this decides
    Whether a candidate map can carry the secondary claim at all. The claim compares
    predictors of kidnap recovery, so a map whose predictor takes the same value
    everywhere contributes nothing: a regression on it returns null by construction, and
    that null is a statement about the map rather than evidence about the hypothesis.

Why running it BEFORE any simulation is the point
    The selection criterion is measured spread of the predictor over free space, computed
    from the occupancy grid alone. No outcome data is involved, so the criterion is
    orthogonal to the hypothesis under test and can be pre-registered. This is the
    distinction that matters for Paper 1: choosing conditions by a criterion independent
    of the result is legitimate design; choosing them to make an effect visible is not.

Why all maps are scored on one grid
    The candidate maps ship at different resolutions. Ambiguity is computed over a
    discretised pose set with a cell-stepped raycast, so resolution is part of the
    measurement. Scoring maps natively would confound "more ambiguous" with "sampled more
    finely" -- the exact comparison the campaign's map axis rests on.

Predictors scored
    ideal_entropy   full expected-scan entropy; what a perfect localizer could infer.
    amcl_entropy    the same under AMCL's likelihood-field model, which scores endpoint
                    distance to the nearest obstacle, clamps it, and drops max-range
                    beams -- strictly less information, so a map can be unambiguous to the
                    ideal model and still confusing to the deployed one.
    amcl_local      as above with hypotheses restricted to a disc, modelling the
                    post-kidnap regime where the filter's belief is already concentrated.
    survivability   mean per-beam AMCL likelihood of a pose's scan evaluated at
                    ring-offset poses: how well the pre-kidnap belief still explains the
                    post-kidnap scan. Low means no gradient home.

Usage
    python3 map_spread_gate.py --map NAME=path/to/map.yaml [--map ...] --out DIR
"""

import argparse
import json
import math
import os
import sys

import numpy as np

from navlearn_analysis.map_ambiguity import (
    AmbiguityField,
    LikelihoodFieldAmbiguity,
    load_occupancy,
    resample_occupancy,
    spread_report,
)

# Deployed AMCL measurement model (amcl_tuned.yaml). Fixed here rather than tuned, so the
# gate describes the stack the campaign actually runs.
AMCL_SIGMA_HIT = 0.1
AMCL_Z_HIT = 0.85
AMCL_Z_RAND = 0.05
AMCL_LIKELIHOOD_MAX_DIST = 6.0
LASER_MAX_RANGE = 12.0

# Kidnap displacement ring the campaign's mid band uses, for the survivability probe.
RING_MIN_M = 0.8
RING_MAX_M = 1.2


def survivability(
    field, poses, ring_min=RING_MIN_M, ring_max=RING_MAX_M, n_offsets=12, n_yaws=8
):
    """Mean per-beam AMCL likelihood of each pose's scan, seen from ring-offset poses.

    Models what the filter experiences at the instant of a kidnap: the belief is still at
    the old pose while the scan now comes from the new one. A high value means the stale
    belief still partially explains the new scan, so weights degrade gracefully and
    resampling has a gradient to follow home. A low value means the belief is in a flat
    region where nothing distinguishes a correct correction from a wrong one.
    """
    out = np.empty(len(poses), dtype=np.float64)
    peak, floor = AMCL_Z_HIT, AMCL_Z_RAND / LASER_MAX_RANGE
    rows, cols = field.occupied.shape

    for i, (px, py, pyaw) in enumerate(poses):
        scan = field.expected_scan(px, py, pyaw)
        valid = scan < (field.max_range_m - 1e-9)
        if not valid.any():
            out[i] = float(floor)
            continue
        ranges = scan[valid]
        beam_angles = field._beam_angles[valid]

        vals = []
        for k in range(n_offsets):
            radius = ring_min + (ring_max - ring_min) * (k % 3) / 2.0
            bearing = 2.0 * math.pi * k / n_offsets
            qx = px + radius * math.cos(bearing)
            qy = py + radius * math.sin(bearing)
            if field.is_occupied(qx, qy):
                continue
            for j in range(n_yaws):
                angles = (2.0 * math.pi * j / n_yaws) + beam_angles
                ex = qx + ranges * np.cos(angles)
                ey = qy + ranges * np.sin(angles)
                col = np.floor((ex - field.origin[0]) / field.resolution).astype(
                    np.int64
                )
                row = np.floor((ey - field.origin[1]) / field.resolution).astype(
                    np.int64
                )
                inside = (row >= 0) & (row < rows) & (col >= 0) & (col < cols)
                np.clip(row, 0, rows - 1, out=row)
                np.clip(col, 0, cols - 1, out=col)
                d = np.where(
                    inside, field._clamped_edt[row, col], field.likelihood_max_dist_m
                )
                vals.append(
                    float(
                        np.mean(
                            peak * np.exp(-0.5 * (d * d) / (AMCL_SIGMA_HIT**2)) + floor
                        )
                    )
                )
        out[i] = float(np.mean(vals)) if vals else float(floor)
    return out


def score_map(
    name,
    yaml_path,
    *,
    resolution,
    pose_spacing,
    yaw_bins,
    beams,
    sample,
    local_radius,
    rng_seed,
):
    """Score one map on the common grid and return its gate report."""
    occ = load_occupancy(yaml_path)
    occupied = resample_occupancy(occ.occupied, occ.resolution, resolution)
    free = ~resample_occupancy(~occ.free, occ.resolution, resolution)

    ideal = AmbiguityField(
        occupied,
        resolution,
        occ.origin,
        free=free,
        pose_spacing_m=pose_spacing,
        yaw_bins=yaw_bins,
        n_beams=beams,
        max_range_m=LASER_MAX_RANGE,
        sigma_m=AMCL_SIGMA_HIT,
        z_hit=AMCL_Z_HIT,
        z_rand=AMCL_Z_RAND,
    )
    amcl = LikelihoodFieldAmbiguity(
        occupied,
        resolution,
        occ.origin,
        free=free,
        pose_spacing_m=pose_spacing,
        yaw_bins=yaw_bins,
        n_beams=beams,
        max_range_m=LASER_MAX_RANGE,
        sigma_m=AMCL_SIGMA_HIT,
        z_hit=AMCL_Z_HIT,
        z_rand=AMCL_Z_RAND,
        likelihood_max_dist_m=AMCL_LIKELIHOOD_MAX_DIST,
    )

    n_poses = len(ideal.poses)
    if n_poses == 0:
        raise ValueError(f"{name}: no free poses; check the map thresholds")

    rng = np.random.default_rng(rng_seed)
    take = min(sample, n_poses)
    idx = rng.choice(n_poses, size=take, replace=False)
    probe = ideal.poses[idx]

    ideal_h = ideal.ambiguity_many(probe)
    amcl_h = np.array(
        [amcl.ambiguity(x, y, yaw)["entropy_bits"] for x, y, yaw in probe]
    )
    amcl_local = np.array(
        [
            amcl.ambiguity(x, y, yaw, radius_m=local_radius)["entropy_bits"]
            for x, y, yaw in probe
        ]
    )
    surv = survivability(amcl, probe)

    predictors = {
        "ideal_entropy": ideal_h,
        "amcl_entropy": amcl_h,
        "amcl_local_entropy": amcl_local,
        "survivability": surv,
    }

    report = {
        "map": name,
        "yaml": os.path.abspath(yaml_path),
        "native_resolution_m": occ.resolution,
        "scored_resolution_m": resolution,
        "grid_cells": [int(occupied.shape[0]), int(occupied.shape[1])],
        "free_positions": int(n_poses // yaw_bins),
        "pose_set_size": int(n_poses),
        "entropy_ceiling_bits": float(math.log2(n_poses)),
        "probed": int(take),
        "predictors": {},
    }

    for key, values in predictors.items():
        stats = spread_report(values)
        # The degeneracy rule in spread_report is written for entropies in bits.
        # Survivability is a likelihood in different units, so its usable-variance test is
        # a relative one: an IQR smaller than 5% of the median cannot separate anything.
        if key == "survivability":
            median = float(np.median(values))
            rel_iqr = stats["iqr"] / median if median > 0 else 0.0
            stats["relative_iqr"] = rel_iqr
            stats["degenerate"] = bool(rel_iqr < 0.05)
        report["predictors"][key] = stats

    report["usable_predictors"] = sorted(
        k for k, v in report["predictors"].items() if not v["degenerate"]
    )
    report["passes_gate"] = bool(report["usable_predictors"])
    return report


def main(argv=None):
    """Score every candidate map and write the gate decision."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument(
        "--map",
        action="append",
        required=True,
        metavar="NAME=PATH",
        help="candidate map, repeatable",
    )
    parser.add_argument("--out", required=True, help="output directory")
    parser.add_argument(
        "--resolution",
        type=float,
        default=0.05,
        help="common grid all maps are scored on, metres",
    )
    parser.add_argument("--pose-spacing", type=float, default=0.5)
    parser.add_argument("--yaw-bins", type=int, default=8)
    parser.add_argument("--beams", type=int, default=36)
    parser.add_argument("--sample", type=int, default=200, help="poses probed per map")
    parser.add_argument("--local-radius", type=float, default=2.0)
    parser.add_argument(
        "--seed", type=int, default=20260730, help="fixed so the gate is reproducible"
    )
    args = parser.parse_args(argv)

    os.makedirs(args.out, exist_ok=True)
    reports = []

    for spec in args.map:
        if "=" not in spec:
            parser.error(f"--map expects NAME=PATH, got {spec!r}")
        name, path = spec.split("=", 1)
        if not os.path.isfile(path):
            parser.error(f"{name}: no such map yaml: {path}")
        print(f"scoring {name} ...", file=sys.stderr)
        reports.append(
            score_map(
                name,
                path,
                resolution=args.resolution,
                pose_spacing=args.pose_spacing,
                yaw_bins=args.yaw_bins,
                beams=args.beams,
                sample=args.sample,
                local_radius=args.local_radius,
                rng_seed=args.seed,
            )
        )

    out_path = os.path.join(args.out, "map_spread_gate.json")
    with open(out_path, "w") as handle:
        json.dump(
            {"resolution_m": args.resolution, "seed": args.seed, "maps": reports},
            handle,
            indent=2,
        )

    header = (
        f"{'map':<14}{'free pos':>9}{'ceil':>7}"
        f"{'ideal':>9}{'amcl':>9}{'amcl_loc':>10}{'surv IQR/med':>14}  gate"
    )
    print("\n" + header)
    print("-" * len(header))
    for r in reports:
        p = r["predictors"]
        print(
            f"{r['map']:<14}{r['free_positions']:>9}"
            f"{r['entropy_ceiling_bits']:>7.2f}"
            f"{p['ideal_entropy']['iqr']:>9.3f}"
            f"{p['amcl_entropy']['iqr']:>9.3f}"
            f"{p['amcl_local_entropy']['iqr']:>10.3f}"
            f"{p['survivability'].get('relative_iqr', 0.0):>14.3f}"
            f"  {'PASS' if r['passes_gate'] else 'FAIL'}"
            f" [{', '.join(r['usable_predictors']) or 'none'}]"
        )
    print("\n(entropy columns are IQR in bits; survivability is IQR/median, unitless)")
    print(f"wrote {out_path}")

    if not any(r["passes_gate"] for r in reports):
        print(
            "\nNO CANDIDATE MAP CARRIES A USABLE PREDICTOR — the secondary claim is not "
            "testable on this map set. Do not collect data against it.",
            file=sys.stderr,
        )
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
