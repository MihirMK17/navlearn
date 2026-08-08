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

"""PROTOCOL A3 judgment: does recovery degrade with |yaw change| at zero displacement?

Pre-registered test (A3, dated 2026-08-03, before any yaw data existed): the logistic
fit of the episode-window recovery outcome on |yaw change| must beat the intercept-only
model by delta AIC >= 2 with a negative slope. Also reported: McFadden pseudo-R2,
cross-validated AUC, and the same fit per arm.

Input is the yaw leg's recompute output (recompute_ttr_from_bags.py with
--magnitude-source yaw_change_deg), so the outcome is PROTOCOL section 1 sustained
recovery over the full episode, not the censored online flag.

Usage:
    python3 run_yaw_claim.py \
        --per-goal results/leg_yaw_recompute/per_goal_ttr_recomputed.csv \
        --out results/leg_yaw_recompute/yaw_claim.md
"""

import argparse
import csv
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from nested_models import compare_nested  # noqa: E402


def judge(rows, label, lines):
    y = np.array([int(r["recovered_sustained"]) for r in rows], dtype=float)
    yaw = np.array([float(r["magnitude_m"]) for r in rows])  # degrees, |yaw change|
    report = compare_nested(y, {"yaw_deg": yaw})
    m0, m1 = report["models"]["M0"], report["models"]["M1"]
    slope = m1["coefficients"][1]
    delta = m0["aic"] - m1["aic"]
    lr = report["tests"]["M0->M1"]
    auc = report["cross_validated_auc"]["yaw_deg"]
    r2 = 1.0 - m1["log_likelihood"] / m0["log_likelihood"]
    holds = delta >= 2.0 and slope < 0.0
    lines += [
        f"## {label} -- n={len(rows)}, recovered {int(y.sum())} ({100.0 * y.mean():.1f}%)",
        "",
        f"- AIC: M0 {m0['aic']:.2f} -> M1(yaw) {m1['aic']:.2f}  (delta {delta:.2f})",
        f"- slope: {slope:.4f} per degree ({'negative' if slope < 0 else 'NON-NEGATIVE'})"
        + ("  [SEPARATED — magnitude not interpretable]" if m1["separated"] else ""),
        f"- LR M0->M1: chi2 {lr['statistic']:.2f}, p {lr['p_value']:.2e}",
        f"- McFadden R2 {r2:.3f}, cross-validated AUC {auc:.3f}",
        f"- **A3 holds: {holds}**",
        "",
    ]
    return holds


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--per-goal", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    rows = list(csv.DictReader(open(args.per_goal)))
    lines = [
        "# A3 -- recovery vs |yaw change| at zero displacement",
        "",
        "Outcome: recovered_sustained (episode window). Predictor: |yaw change| deg.",
        "",
    ]
    judge(rows, "Pooled (primary)", lines)
    for arm in sorted({r["arm"] for r in rows}):
        judge([r for r in rows if r["arm"] == arm], f"{arm} (robustness)", lines)

    text = "\n".join(lines)
    pathlib.Path(args.out).write_text(text)
    print(text)


if __name__ == "__main__":
    main()
