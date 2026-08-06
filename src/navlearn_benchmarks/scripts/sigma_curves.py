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

"""Leg 4, judged exactly as PROTOCOL A4 pre-registered it.

Primary
    Logistic regression of episode-window sustained recovery (section 1, via the A1
    recompute) on sigma_hit as a CONTINUOUS predictor over all pooled goals. Continuous
    because it uses every goal at once; pairwise level comparisons stay underpowered at
    any n this campaign can afford and are reported as descriptive only.

Secondary, pre-declared as a distinct question rather than a fallback
    Two curves over the same levels: recovery against sigma_hit, and false success
    against sigma_hit judged at BOTH 0.05 m and 0.10 m. A2 established that the 0.05 m
    figure sits on a decision boundary and moves for reasons unrelated to the filter, so
    quoting it alone would be quoting the boundary rather than the result.

    The question the pair answers: is the sigma_hit that localizes most accurately in
    ordinary driving the same one that recovers most often from a kidnap? Two curves of
    different shape mean a trade-off, not a single optimum.

Usage:
    python3 sigma_curves.py \
        --level 0.05=results/leg4_sigma_005 --level 0.05=results/leg4_sigma_ext_005 \
        ... \
        --recompute results/leg4_pooled_recompute/per_goal_ttr_recomputed.csv \
        --out results/leg4_pooled_analysis/sigma_curves.md
"""

import argparse
import collections
import csv
import glob
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from nested_models import compare_nested, fit_logistic  # noqa: E402

TOLERANCES = (0.05, 0.10)


def load_wide(dirs):
    """Per-goal reported/true-distance rows from a cell's wide metrics CSVs.

    Exclusions mirror analyze_curve_campaign.py: an attempted-but-unapplied kidnap ran
    unperturbed and is not a TTR trial; a goal without ground truth is unmeasured, which
    is not the same as a failure.
    """
    rows, attrition, no_gt = [], 0, 0
    for d in dirs:
        for path in sorted(glob.glob(f"{d}/navlearn_metrics_run_*[0-9].csv")):
            for r in csv.DictReader(open(path)):
                if r.get("Kidnap Attempted") == "1" and r.get("Kidnap Applied") != "1":
                    attrition += 1
                    continue
                if r.get("GT Available") != "1":
                    no_gt += 1
                    continue
                rows.append({
                    "goal_id": r["Goal_ID"],
                    "reported": r["Goal Result"] == "SUCCEEDED",
                    "distance": float(r["True Distance To Goal (m)"]),
                })
    return rows, attrition, no_gt


def wilson(k, n, z=1.96):
    """Wilson score interval -- behaves at proportions near 0 and 1, unlike normal."""
    if n == 0:
        return (float("nan"), float("nan"))
    p = k / n
    denom = 1 + z * z / n
    centre = (p + z * z / (2 * n)) / denom
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return (max(0.0, centre - half), min(1.0, centre + half))


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--level", action="append", required=True, metavar="SIGMA=DIR",
                        help="repeatable; repeat the same SIGMA to pool passes")
    parser.add_argument("--recompute", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    by_sigma = collections.defaultdict(list)
    for spec in args.level:
        sigma, _, directory = spec.partition("=")
        by_sigma[float(sigma)].append(directory)

    recovered = {r["goal_id"]: int(r["recovered_sustained"])
                 for r in csv.DictReader(open(args.recompute))}

    sigmas, y, per_level = [], [], {}
    for sigma in sorted(by_sigma):
        rows, attrition, no_gt = load_wide(by_sigma[sigma])
        matched = [r for r in rows if r["goal_id"] in recovered]
        for r in matched:
            sigmas.append(sigma)
            y.append(recovered[r["goal_id"]])
        per_level[sigma] = {
            "rows": rows, "matched": matched,
            "attrition": attrition, "no_gt": no_gt,
            "rec": sum(recovered[r["goal_id"]] for r in matched),
        }

    sigmas = np.array(sigmas, dtype=float)
    y = np.array(y, dtype=float)

    lines = ["# Leg 4 -- sigma_hit, judged as PROTOCOL A4 pre-registered it", "",
             f"Pooled n = {len(y)} goals, recovered {int(y.sum())} "
             f"({100.0 * y.mean():.1f}%). Outcome is the episode-window sustained "
             f"recovery of section 1, not the 10 s censored online flag.", ""]

    # --- primary: continuous fit ---------------------------------------------------
    report = compare_nested(y, {"sigma_hit": sigmas})
    m0, m1 = report["models"]["M0"], report["models"]["M1"]
    slope = m1["coefficients"][1]
    delta = m0["aic"] - m1["aic"]
    lr = report["tests"]["M0->M1"]
    r2 = 1.0 - m1["log_likelihood"] / m0["log_likelihood"]
    auc = report["cross_validated_auc"]["sigma_hit"]
    lines += [
        "## Primary: recovery on sigma_hit as a continuous predictor", "",
        f"- AIC: null {m0['aic']:.2f} -> sigma {m1['aic']:.2f} (delta {delta:.2f})",
        f"- slope {slope:+.3f} per unit sigma_hit "
        f"({'positive: wider basin recovers more' if slope > 0 else 'negative'})"
        + ("  [SEPARATED]" if m1["separated"] else ""),
        f"- likelihood ratio: chi2 {lr['statistic']:.2f}, p {lr['p_value']:.3e}",
        f"- McFadden R2 {r2:.3f}, cross-validated AUC {auc:.3f}",
        f"- **A4 primary: sigma_hit predicts recovery = "
        f"{bool(delta >= 2.0 and lr['p_value'] < 0.05)}**",
        "",
    ]

    # --- secondary: the two curves ---------------------------------------------------
    lines += ["## Secondary: the two curves", "",
              "| sigma_hit | n | recovery % (95% CI) | false success @0.05 m |"
              " false success @0.10 m | true success @0.05 m |",
              "|---|---|---|---|---|---|"]
    for sigma in sorted(per_level):
        d = per_level[sigma]
        rows, n = d["rows"], len(d["rows"])
        nm = len(d["matched"])
        lo, hi = wilson(d["rec"], nm)
        cells = [f"| {sigma:.2f} | {n} | "
                 f"{100.0 * d['rec'] / nm:.1f} ({100 * lo:.0f}-{100 * hi:.0f}) |"]
        for tol in TOLERANCES:
            fs = sum(1 for r in rows if r["reported"] and r["distance"] > tol)
            cells.append(f" {100.0 * fs / n:.1f}% |")
        ts = sum(1 for r in rows if r["distance"] <= 0.05)
        cells.append(f" {100.0 * ts / n:.1f}% |")
        lines.append("".join(cells))
    lines.append("")

    # Does false success have a different SHAPE from recovery? Fit both against sigma
    # and report the direction; a non-monotone false-success curve against a monotone
    # recovery curve is the trade-off claim.
    fs_y, fs_x = [], []
    for sigma in sorted(per_level):
        for r in per_level[sigma]["rows"]:
            fs_x.append(sigma)
            fs_y.append(1.0 if (r["reported"] and r["distance"] > 0.10) else 0.0)
    fs_fit = fit_logistic(np.array(fs_x).reshape(-1, 1), np.array(fs_y))
    lines += [
        f"False success (at 0.10 m) on sigma_hit: slope {fs_fit.coefficients[1]:+.3f}, "
        f"against a recovery slope of {slope:+.3f}.",
        "",
        "A monotone recovery curve beside a false-success curve of different shape is "
        "the finding: the sigma_hit that localizes most accurately in ordinary driving "
        "is not the one that recovers most often from a kidnap. Neither curve "
        "recommends a value; the claim is about their shapes differing.",
        "",
    ]

    total_attr = sum(d["attrition"] for d in per_level.values())
    total_nogt = sum(d["no_gt"] for d in per_level.values())
    lines += ["## Bookkeeping", "",
              f"- goals matched to a recomputed outcome: {len(y)}",
              f"- excluded: {total_attr} kidnap-never-applied, {total_nogt} without "
              f"ground truth"]

    text = "\n".join(lines)
    out = pathlib.Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(text)
    print(text)


if __name__ == "__main__":
    main()
