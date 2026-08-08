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

"""A5 secondary judgment: is the yaw cliff steeper in one environment than another?

A5 pre-registered (2026-08-05, re-registered for the bookstore 2026-08-06) that an
environment of repeating shelf aisles offers less distinguishing structure under
rotation than a house of irregular rooms, so its logistic recovery-vs-|yaw| slope
should be MORE negative. The amendment also fixed how the prediction is judged:
"by comparing the two slopes with their confidence intervals" — this script is that
comparison, committed so the judgment can be re-run.

Method: per environment, the same M1 fit run_yaw_claim.py uses (logistic recovery on
|yaw change| in degrees, episode-window outcome). Wald standard errors from the
observed information matrix at the fit, in original units; the two campaigns are
independent samples, so the slope difference is tested with
z = (b_A - b_B) / sqrt(se_A^2 + se_B^2).

Usage:
    python3 compare_yaw_slopes.py \
        --env small_house=results/leg_yaw_recompute/per_goal_ttr_recomputed.csv:rpp \
        --env bookstore=results/leg7_bookstore_recompute_yaw/per_goal_ttr_recomputed.csv \
        --steeper bookstore \
        --out results/leg7_bookstore_analysis_yaw/slope_comparison.md

The optional :arm suffix filters a multi-arm CSV to one arm, so a pooled small_house
file can be compared like-for-like against a single-controller replication.
"""

import argparse
import csv
import math
import pathlib
import sys

import numpy as np

from navlearn_analysis.nested_models import fit_logistic


def wald_se(fit, x):
    """Wald standard errors for [intercept, slope] in original units.

    Observed information I = X^T W X with W = diag(p(1-p)) evaluated at the fitted
    coefficients on the ORIGINAL-units design, so no de-standardisation of the
    covariance is needed.
    """
    x = np.asarray(x, dtype=np.float64).reshape(-1)
    design = np.column_stack([np.ones(x.size), x])
    eta = design @ fit.coefficients
    p = 1.0 / (1.0 + np.exp(-eta))
    w = p * (1.0 - p)
    info = design.T @ (design * w[:, None])
    cov = np.linalg.inv(info)
    return np.sqrt(np.diag(cov))


def load_env(spec):
    """LABEL=CSV[:arm] -> (label, |yaw| array, outcome array)."""
    label, _, rest = spec.partition("=")
    path, _, arm = rest.partition(":")
    rows = list(csv.DictReader(open(path)))
    if arm:
        rows = [r for r in rows if r.get("arm") == arm]
        if not rows:
            sys.exit(f"FATAL: no rows with arm '{arm}' in {path}")
    yaw = np.array([abs(float(r["magnitude_m"])) for r in rows])
    y = np.array([int(r["recovered_sustained"]) for r in rows], dtype=float)
    return label, yaw, y


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--env",
        action="append",
        required=True,
        metavar="LABEL=CSV[:ARM]",
        help="exactly two environments",
    )
    parser.add_argument(
        "--steeper",
        required=True,
        help="label the pre-registered prediction says is steeper",
    )
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    if len(args.env) != 2:
        sys.exit("FATAL: exactly two --env specs are required")

    fits = []
    for spec in args.env:
        label, yaw, y = load_env(spec)
        fit = fit_logistic(yaw, y)
        if not fit.converged or fit.separated:
            sys.exit(
                f"FATAL: {label} fit did not converge cleanly "
                f"(converged={fit.converged}, separated={fit.separated})"
            )
        se = wald_se(fit, yaw)
        fits.append(
            {
                "label": label,
                "n": int(y.size),
                "recovered": int(y.sum()),
                "slope": float(fit.coefficients[1]),
                "se": float(se[1]),
            }
        )

    if args.steeper not in {f["label"] for f in fits}:
        sys.exit(f"FATAL: --steeper '{args.steeper}' matches neither environment")

    a, b = fits
    diff = a["slope"] - b["slope"]
    se_diff = math.hypot(a["se"], b["se"])
    z = diff / se_diff
    # Two-sided p from the normal, via erfc.
    p = math.erfc(abs(z) / math.sqrt(2.0))

    predicted = next(f for f in fits if f["label"] == args.steeper)
    other = next(f for f in fits if f["label"] != args.steeper)
    direction_holds = predicted["slope"] < other["slope"]

    lines = [
        "# A5 secondary: slope comparison across environments",
        "",
        "Logistic recovery on |yaw change| (deg), episode-window outcome; Wald 95% CIs.",
        "",
        "| environment | n | recovered | slope (per deg) | 95% CI |",
        "|---|---|---|---|---|",
    ]
    for f in fits:
        lo, hi = f["slope"] - 1.96 * f["se"], f["slope"] + 1.96 * f["se"]
        lines.append(
            f"| {f['label']} | {f['n']} | {f['recovered']} "
            f"| {f['slope']:.4f} | [{lo:.4f}, {hi:.4f}] |"
        )
    lines += [
        "",
        f"- pre-registered prediction: **{args.steeper}** is steeper (more negative)",
        f"- observed direction: {'as predicted' if direction_holds else 'OPPOSITE'} — "
        f"{predicted['label']} {predicted['slope']:.4f} vs "
        f"{other['label']} {other['slope']:.4f}",
        f"- slope difference {diff:+.4f}, z = {z:.2f}, two-sided p = {p:.3g}",
        "",
    ]
    if direction_holds and p < 0.05:
        verdict = "prediction SUPPORTED: steeper in the predicted environment."
    elif (not direction_holds) and p < 0.05:
        verdict = (
            "prediction FAILS: the cliff is significantly steeper in the OTHER "
            "environment. Reported at full prominence per A5."
        )
    else:
        verdict = (
            "INCONCLUSIVE at this sample: the slope difference is not resolved "
            "(p >= 0.05). Neither steeper nor shallower is claimed."
        )
    lines.append(f"**Judgment: {verdict}**")
    lines.append("")

    text = "\n".join(lines)
    out = pathlib.Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(text)
    print(text)


if __name__ == "__main__":
    main()
