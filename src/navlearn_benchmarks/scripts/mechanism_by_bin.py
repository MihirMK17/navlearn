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

"""Confidence-versus-truth, split by perturbation magnitude rather than by cell.

`rate_mechanism.py` asks whether a filter arrives at confidence while still far from
ground truth, and reports it per cell because leg 6's variable IS the cell. The yaw leg's
variable is inside the cell: every goal has its own |yaw change|. This regroups the same
per-goal quantities by magnitude bin so the question can be put to rotation.

The question it puts
    Above 45 degrees the yaw leg recovers almost never. There are two very different
    reasons a filter fails to recover, and they call for opposite fixes:

      it never becomes confident   -- the scan matches nothing, the cloud stays diffuse,
                                      the filter knows it is lost;
      it becomes confident and is  -- the rotated scan matches somewhere else in the map,
      wrong                           the cloud collapses onto that place, and the filter
                                      believes it has recovered.

    The second is perceptual aliasing under rotation, and it would connect the yaw result
    to the ambiguity hypothesis that was rejected in its map-derived, position-only form
    (amendment A2). It is not the same claim and is not asserted by A3; this reports the
    evidence either way.

Usage:
    python3 mechanism_by_bin.py \
        --mechanism results/leg_yaw_mechanism/per_goal_mechanism.csv \
        --magnitudes results/leg_yaw_recompute/per_goal_ttr_recomputed.csv \
        --edges 0,45,90,135,180 --unit deg --out results/leg_yaw_mechanism/by_angle.md
"""

import argparse
import csv
import pathlib
import statistics


def med(values):
    vals = [v for v in values if v is not None]
    return statistics.median(vals) if vals else float("nan")


def as_float(text):
    try:
        return float(text)
    except (TypeError, ValueError):
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--mechanism", required=True)
    parser.add_argument("--magnitudes", required=True,
                        help="recompute per-goal CSV; supplies magnitude_m and the "
                             "recovery outcome")
    parser.add_argument("--edges", required=True, help="comma-separated bin edges")
    parser.add_argument("--unit", default="")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    edges = [float(v) for v in args.edges.split(",")]
    mag = {}
    for r in csv.DictReader(open(args.magnitudes)):
        mag[r["goal_id"]] = (float(r["magnitude_m"]),
                             int(r["recovered_sustained"]))

    rows = []
    unmatched = 0
    for r in csv.DictReader(open(args.mechanism)):
        if r["goal_id"] not in mag:
            unmatched += 1
            continue
        m, rec = mag[r["goal_id"]]
        rows.append({
            "magnitude": m,
            "recovered": rec,
            "t_confident": as_float(r["t_confident"]),
            "err_at_confident": as_float(r["err_at_confident"]),
            "min_err": as_float(r["min_err"]),
            "usable": as_float(r["baseline_cov"]) is not None,
        })

    lines = [
        "# Confidence versus truth, by perturbation magnitude",
        "",
        "For each bin: how often the filter reached its own pre-perturbation confidence "
        "level at all, and how far it was from ground truth at that instant. A filter "
        "that becomes confident while far from truth has locked onto the wrong place; "
        "one that never becomes confident knows it is lost.",
        "",
        f"| bin ({args.unit}) | n | usable | reached confidence | "
        f"err at confidence (m) | recovered | min err (m) |",
        "|---|---|---|---|---|---|---|",
    ]
    for i in range(len(edges) - 1):
        lo, hi = edges[i], edges[i + 1]
        last = i == len(edges) - 2
        sel = [r for r in rows
               if lo <= r["magnitude"] < hi or (last and r["magnitude"] == hi)]
        if not sel:
            lines.append(f"| {lo:g}-{hi:g} | 0 | - | - | - | - | - |")
            continue
        usable = [r for r in sel if r["usable"]]
        conf = [r for r in usable if r["t_confident"] is not None]
        lines.append(
            f"| {lo:g}-{hi:g} | {len(sel)} | {len(usable)}/{len(sel)} | "
            f"{len(conf)}/{len(usable)} | "
            f"{med(r['err_at_confident'] for r in conf):.3f} | "
            f"{100.0 * sum(r['recovered'] for r in sel) / len(sel):.1f}% | "
            f"{med(r['min_err'] for r in usable):.3f} |")

    lines.append("")
    # The decisive split: among goals that did NOT recover, did the filter nonetheless
    # reach confidence? That is the aliasing signature.
    lines += ["## Non-recovered goals only: did the filter still become confident?", "",
              f"| bin ({args.unit}) | non-recovered n | of those, reached confidence | "
              f"err at confidence (m) |", "|---|---|---|---|"]
    for i in range(len(edges) - 1):
        lo, hi = edges[i], edges[i + 1]
        last = i == len(edges) - 2
        sel = [r for r in rows
               if (lo <= r["magnitude"] < hi or (last and r["magnitude"] == hi))
               and not r["recovered"] and r["usable"]]
        conf = [r for r in sel if r["t_confident"] is not None]
        if not sel:
            lines.append(f"| {lo:g}-{hi:g} | 0 | - | - |")
            continue
        lines.append(
            f"| {lo:g}-{hi:g} | {len(sel)} | {len(conf)} "
            f"({100.0 * len(conf) / len(sel):.0f}%) | "
            f"{med(r['err_at_confident'] for r in conf):.3f} |")
    lines += ["",
              "A high confidence rate among non-recovered goals, at a large error, is "
              "the filter locking onto the wrong place -- aliasing. A low one is the "
              "filter correctly remaining unsure.",
              "",
              f"- goals in the mechanism file with no magnitude match: {unmatched}"]

    text = "\n".join(lines)
    out = pathlib.Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(text)
    print(text)


if __name__ == "__main__":
    main()
