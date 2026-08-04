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

"""Assemble the claim 2 dataset and run the nested model comparison.

Joins the episode-window recovery outcome (recompute_ttr_from_bags.py) with the
map-derived landing ambiguity (map_ambiguity.py) and hands the result to
nested_models.compare_nested. The join is by goal_id via the targets CSV that fed the
ambiguity scoring, so a row cannot silently pair an outcome with someone else's landing.

PROTOCOL section 6: claim 2 holds if M2 (ambiguity) beats M1 (distance) by delta AIC >= 2;
strongest form adds that M3 does not improve on M2. The predictor correlation is reported
unconditionally. Distance enters as log magnitude -- the campaign drew magnitudes
log-uniformly, so log is the scale on which the design is balanced; the raw-scale fit is
reported alongside as a robustness line, not a second hypothesis.

Usage:
    python3 run_claim2_models.py \
        --per-goal results/leg2_ttr_recompute/per_goal_ttr_recomputed.csv \
        --targets results/leg2_ttr_recompute/kidnap_targets.csv \
        --ambiguity results/leg2_ttr_recompute/ambiguity.csv \
        --out results/leg2_ttr_recompute/claim2_models.md
"""

import argparse
import csv
import json
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from nested_models import compare_nested  # noqa: E402


def load_dataset(per_goal_path, targets_path, ambiguity_path):
    """One row per usable goal: (goal_id, arm, recovered, magnitude_m, ambiguity_bits)."""
    outcomes = {r["goal_id"]: r for r in csv.DictReader(open(per_goal_path))}

    targets = list(csv.DictReader(open(targets_path)))
    scores = list(csv.DictReader(open(ambiguity_path)))
    if len(targets) != len(scores):
        raise RuntimeError(
            f"targets ({len(targets)}) and ambiguity ({len(scores)}) row counts differ")

    rows = []
    for i, (t, s) in enumerate(zip(targets, scores)):
        if int(s["row_index"]) != i:
            raise RuntimeError(f"ambiguity row_index {s['row_index']} != position {i}")
        # The ambiguity file carries the coordinates it scored; they must be the
        # coordinates of the goal we are about to attach the score to.
        if abs(float(s["target_x"]) - float(t["x"])) > 1e-9 or \
           abs(float(s["target_y"]) - float(t["y"])) > 1e-9:
            raise RuntimeError(f"row {i}: scored pose differs from target pose")
        o = outcomes[t["goal_id"]]
        rows.append({
            "goal_id": t["goal_id"],
            "arm": t["arm"],
            "recovered": int(o["recovered_sustained"]),
            "magnitude_m": float(o["magnitude_m"]),
            "ambiguity_bits": float(s["ambiguity_bits"]),
        })
    return rows


def run(rows, distance_transform, label, lines):
    y = np.array([r["recovered"] for r in rows], dtype=float)
    dist = np.array([distance_transform(r["magnitude_m"]) for r in rows])
    amb = np.array([r["ambiguity_bits"] for r in rows])
    report = compare_nested(y, {"distance": dist, "ambiguity": amb})

    lines.append(f"## {label} -- n={len(rows)}, recovered {int(y.sum())} "
                 f"({100.0 * y.mean():.1f}%)")
    lines.append("")
    if report["any_separated"]:
        lines.append("**WARNING: separation detected in at least one model -- "
                     "coefficients there are not effect sizes.**")
        lines.append("")
    lines.append("| model | AIC | BIC | McFadden R2 |")
    lines.append("|---|---|---|---|")
    null_ll = report["models"]["M0"]["log_likelihood"]
    for name in ("M0", "M1", "M2", "M3"):
        m = report["models"][name]
        r2 = 1.0 - m["log_likelihood"] / null_ll if name != "M0" else 0.0
        lines.append(f"| {name} | {m['aic']:.2f} | {m['bic']:.2f} | {r2:.3f} |")
    lines.append("")
    for pair, t in report["tests"].items():
        lines.append(f"- LR {pair}: chi2 {t['statistic']:.2f}, p {t['p_value']:.2e}")
    cv = report["cross_validated_auc"]
    lines.append(f"- cross-validated AUC: distance {cv['distance']:.3f}, "
                 f"ambiguity {cv['ambiguity']:.3f}")
    corr = report["predictor_correlation"]
    lines.append(f"- predictor correlation: pearson {corr['pearson']:.3f}, "
                 f"spearman {corr['spearman']:.3f} (p {corr['spearman_p']:.2e})")
    v = report["verdict"]
    lines.append(f"- delta AIC (M1 - M2): {v['delta_aic_M1_minus_M2']:.2f}; "
                 f"best model {v['best_model_by_aic']}")
    lines.append(f"- **M2 beats M1: {v['candidate_beats_incumbent']}** | "
                 f"distance adds nothing on top: {v['incumbent_adds_nothing_on_top']} | "
                 f"ambiguity subsumes distance: {v['ambiguity_subsumes_distance']}")
    lines.append("")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--per-goal", required=True)
    parser.add_argument("--targets", required=True)
    parser.add_argument("--ambiguity", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    rows = load_dataset(args.per_goal, args.targets, args.ambiguity)

    lines = ["# Claim 2 -- nested model comparison (PROTOCOL section 6)", "",
             "Outcome: recovered_sustained (episode window). "
             "M1 distance, M2 landing ambiguity, M3 both.", ""]

    primary = run(rows, math.log, "Pooled, log distance (primary)", lines)
    run(rows, lambda m: m, "Pooled, raw distance (robustness)", lines)
    for arm in sorted({r["arm"] for r in rows}):
        run([r for r in rows if r["arm"] == arm], math.log,
            f"{arm} only, log distance (robustness)", lines)

    text = "\n".join(lines)
    pathlib.Path(args.out).write_text(text)
    print(text)
    with open(pathlib.Path(args.out).with_suffix(".json"), "w") as f:
        json.dump(primary, f, indent=2, default=float)


if __name__ == "__main__":
    main()
