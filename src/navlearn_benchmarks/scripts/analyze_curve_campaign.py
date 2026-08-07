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
"""Analysis for the curve campaigns: per-goal outcomes, binned tables, one report.

Why it exists
    The leg 1 tables were computed ad hoc and the method left the machine with the session
    that produced them. These are paper numbers; the analysis has to be a committed,
    tested artefact or it cannot be re-run, and an analysis that cannot be re-run cannot
    be defended. This script IS the method.

Definitions (from docs/paper1/PROTOCOL.md, frozen 2026-07-30; node behaviour verified in
navlearn_localization_eval source)

    true success    ground truth within the stack's xy_goal_tolerance at episode end.
                    The tolerance is read from each run's own stack spec, never assumed.
    false success   Nav2 reported SUCCEEDED while ground truth was outside tolerance.
    reported        Nav2 said SUCCEEDED. Derivable from the two above; shown because it
                    is the number being demonstrated unreliable.
    recovered       TTR Outcome == 1: estimate within threshold at goal end, sustained --
                    the node demotes RECOVERED back to RECOVERING on a threshold break.
    TTR             seconds to first sustained convergence; only defined when recovered.
    magnitude       the REALISED displacement ("Kidnap Displacement (m)"), not the
                    commanded band: the band is a label, the robot moved by the distance.

    Goals whose kidnap was attempted but never applied ran unperturbed and are excluded
    as attrition (counted, reported). Goals without ground truth are excluded and counted:
    without ground truth, true/false success is unknowable, not zero.

Usage:
    python3 analyze_curve_campaign.py \
        --arm rpp=results/leg2_final_rpp --arm dwb=results/leg2_final_dwb \
        --arm mppi=results/leg2_final_mppi \
        --range 0.01:3.0 --expect-goals 120 --output-dir results/leg2_analysis
"""

import argparse
import collections
import csv
import dataclasses
import json
import math
import pathlib
import statistics
import sys


class ArmGoals(list):
    """An arm's usable goals, carrying its exclusion counts with it.

    A list subclass rather than a wrapper so every consumer that treats an arm as a plain
    sequence (bins, filters, tests) keeps working; the counts ride along instead of being
    smuggled through module state, which would make every arm report whichever loaded last.
    """

    def __init__(self, goals=(), attrition=0, no_gt=0):
        super().__init__(goals)
        self.attrition = attrition
        self.no_gt = no_gt


@dataclasses.dataclass(frozen=True)
class Goal:
    """One perturbed goal with every outcome the paper's claims are judged on."""

    goal_id: str
    magnitude: float  # realised displacement, metres
    reported: bool  # Nav2 said SUCCEEDED
    true_success: bool  # ground truth within tolerance at episode end
    false_success: bool  # reported and not truly there
    true_distance_m: float
    recovered: bool  # TTR Outcome == 1 (sustained)
    ttr_s: float  # -1.0 when undefined
    nav_time_s: float


def _read_localization(paths) -> dict:
    """Fold the long-format localization CSVs into {goal_id: {metric: value}}."""
    metrics = collections.defaultdict(dict)
    for path in paths:
        with open(path) as handle:
            for row in csv.DictReader(handle):
                try:
                    metrics[row["GoalID"]][row["Metric Name"]] = float(
                        row["Metric Value"]
                    )
                except (KeyError, ValueError):
                    continue
    return metrics


def load_arm(arm_dir, expect_goals=None, magnitude_field="displacement") -> list:
    """Load one arm directory into per-goal records, refusing partial data.

    A short arm silently analysed is how a crashed campaign becomes a result, which is
    the exact defect validate_run_output exists to stop at collection time; the same rule
    is enforced again here because analysis may run on directories the harness never
    validated.
    """
    arm = pathlib.Path(arm_dir)

    spec_files = sorted(arm.glob("navlearn_stack_spec_run_*.json"))
    if not spec_files:
        sys.exit(f"FATAL: no stack spec in {arm}; the tolerance cannot be established")
    tolerance = json.loads(spec_files[0].read_text())["identity"]["xy_goal_tolerance"]

    wide_files = sorted(
        p
        for p in arm.glob("navlearn_metrics_run_*.csv")
        if not p.name.endswith("_localization.csv")
    )
    loc = _read_localization(
        sorted(arm.glob("navlearn_metrics_run_*_localization.csv"))
    )

    goals, attrition, no_gt = [], 0, 0
    for path in wide_files:
        with open(path) as handle:
            for row in csv.DictReader(handle):
                if (
                    row.get("Kidnap Attempted") == "1"
                    and row.get("Kidnap Applied") != "1"
                ):
                    attrition += 1
                    continue
                if row.get("GT Available") != "1":
                    no_gt += 1
                    continue

                distance = float(row["True Distance To Goal (m)"])
                reported = row["Goal Result"] == "SUCCEEDED"
                # True success is about where the robot IS, not what Nav2 said: a FAILED
                # goal with the robot truly at the goal is a true success (the MPPI
                # signature -- leg 1's true 73.3% against reported 38.3% exists only
                # under this reading, and PROTOCOL.md defines it this way).
                truly_there = distance <= tolerance
                ttr = loc.get(row["Goal_ID"], {})
                # The yaw-curve leg's independent variable is the rotation, not the
                # displacement (which is pinned to zero by design). |value|: the sign is
                # a direction, the severity is the angle.
                if magnitude_field == "yaw_change":
                    magnitude = abs(float(row["Kidnap Yaw Change (deg)"]))
                else:
                    magnitude = float(row["Kidnap Displacement (m)"])
                goals.append(
                    Goal(
                        goal_id=row["Goal_ID"],
                        magnitude=magnitude,
                        reported=reported,
                        true_success=truly_there,
                        false_success=reported and not truly_there,
                        true_distance_m=distance,
                        recovered=ttr.get("TTR Outcome") == 1.0,
                        ttr_s=ttr.get("TTR", -1.0),
                        nav_time_s=float(row["Nav Time (sec)"]),
                    )
                )

    if expect_goals is not None and len(goals) + attrition + no_gt != expect_goals:
        sys.exit(
            f"FATAL: {arm} holds {len(goals)} usable goals "
            f"(+{attrition} attrition, +{no_gt} without ground truth) "
            f"but {expect_goals} were expected. A partial arm is a crashed campaign, "
            "not a smaller sample; re-run the cell instead of analysing it."
        )
    return ArmGoals(goals, attrition=attrition, no_gt=no_gt)


def summarise(goals) -> dict:
    """Overall rates for one set of goals."""
    n = len(goals)
    recovered_ttrs = [g.ttr_s for g in goals if g.recovered and g.ttr_s >= 0.0]
    return {
        "n": n,
        "attrition": getattr(goals, "attrition", 0),
        "no_gt": getattr(goals, "no_gt", 0),
        "reported_pct": (
            100.0 * sum(g.reported for g in goals) / n if n else float("nan")
        ),
        "true_pct": (
            100.0 * sum(g.true_success for g in goals) / n if n else float("nan")
        ),
        "false_pct": (
            100.0 * sum(g.false_success for g in goals) / n if n else float("nan")
        ),
        "recovered_pct": (
            100.0 * sum(g.recovered for g in goals) / n if n else float("nan")
        ),
        "ttr_median_s": (
            statistics.median(recovered_ttrs) if recovered_ttrs else float("nan")
        ),
        "nav_time_median_s": (
            statistics.median(g.nav_time_s for g in goals) if n else float("nan")
        ),
    }


def log_bin_edges(lo: float, hi: float, bins: int) -> list:
    """Bin edges uniform in log space, the same construction as the leg 1 tables."""
    ratio = (hi / lo) ** (1.0 / bins)
    return [lo * ratio**k for k in range(bins + 1)]


def linear_bin_edges(lo: float, hi: float, bins: int) -> list:
    """Bin edges uniform in the value. The yaw-curve leg samples linearly and its
    range starts at zero, which has no logarithm."""
    step = (hi - lo) / bins
    return [lo + step * k for k in range(bins + 1)]


def binned(goals, edges) -> list:
    """Per-bin summaries. Half-open [lo, hi) bins; the last bin closed so hi is kept."""
    out = []
    for i in range(len(edges) - 1):
        last = i == len(edges) - 2
        members = [
            g
            for g in goals
            if edges[i] <= g.magnitude < edges[i + 1]
            or (last and math.isclose(g.magnitude, edges[i + 1]))
            or (last and g.magnitude >= edges[i + 1])  # realised may overshoot the band
        ]
        row = summarise(members) if members else {"n": 0}
        row["lo"], row["hi"] = edges[i], edges[i + 1]
        # Off-range magnitudes below the sweep floor land in bin 0.
        if i == 0:
            extra = [g for g in goals if g.magnitude < edges[0]]
            if extra:
                row = summarise(members + extra)
                row["lo"], row["hi"] = edges[0], edges[1]
        out.append(row)
    return out


def _fmt(value, width=6, digits=1):
    if isinstance(value, float) and math.isnan(value):
        return "-".rjust(width)
    return f"{value:.{digits}f}".rjust(width)


def build_report(
    arms: dict, lo: float, hi: float, bins: int = 4, bin_scale: str = "log"
) -> str:
    """One self-describing markdown report for every arm."""
    edges = (linear_bin_edges if bin_scale == "linear" else log_bin_edges)(lo, hi, bins)
    lines = [
        "# Curve campaign analysis",
        "",
        "Definitions are docs/paper1/PROTOCOL.md (frozen 2026-07-30):",
        "true success = ground truth within the stack's xy_goal_tolerance at episode end;",
        "false success = SUCCEEDED reported while ground truth was outside tolerance;",
        "recovered = TTR Outcome == 1 (sustained to goal end);",
        "magnitude = realised kidnap displacement, not the commanded band.",
        "Attrition (kidnap never applied) and missing-ground-truth goals are excluded and "
        "counted.",
        "",
    ]

    lines.append("## Overall, per arm")
    lines.append("")
    lines.append(
        "| arm | n | reported | true | false | recovered | TTR med (s) "
        "| nav med (s) | attrition | no GT |"
    )
    lines.append("|---|---|---|---|---|---|---|---|---|---|")
    for name, goals in arms.items():
        s = summarise(goals)
        lines.append(
            f"| {name} | {s['n']} |{_fmt(s['reported_pct'])}% |{_fmt(s['true_pct'])}% "
            f"|{_fmt(s['false_pct'])}% |{_fmt(s['recovered_pct'])}% "
            f"|{_fmt(s['ttr_median_s'], digits=2)} |{_fmt(s['nav_time_median_s'])} "
            f"| {s['attrition']} | {s['no_gt']} |"
        )
    lines.append("")

    for name, goals in arms.items():
        lines.append(f"## {name} by realised magnitude (log-quartile bins)")
        lines.append("")
        lines.append("| kidnap (m) | n | true | false | recovered | TTR med (s) |")
        lines.append("|---|---|---|---|---|---|")
        for row in binned(goals, edges):
            label = f"{row['lo']:.2f}-{row['hi']:.2f}"
            if row["n"] == 0:
                lines.append(f"| {label} | 0 | - | - | - | - |")
                continue
            lines.append(
                f"| {label} | {row['n']} |{_fmt(row['true_pct'])}% "
                f"|{_fmt(row['false_pct'])}% |{_fmt(row['recovered_pct'])}% "
                f"|{_fmt(row['ttr_median_s'], digits=2)} |"
            )
        lines.append("")

    return "\n".join(lines)


def write_per_goal_csv(arms: dict, path):
    """The joined per-goal table, for the regression stage and for anyone re-deriving."""
    with open(path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "arm",
                "goal_id",
                "magnitude_m",
                "reported",
                "true_success",
                "false_success",
                "true_distance_m",
                "recovered",
                "ttr_s",
                "nav_time_s",
            ]
        )
        for name, goals in arms.items():
            for g in goals:
                writer.writerow(
                    [
                        name,
                        g.goal_id,
                        f"{g.magnitude:.6f}",
                        int(g.reported),
                        int(g.true_success),
                        int(g.false_success),
                        f"{g.true_distance_m:.6f}",
                        int(g.recovered),
                        f"{g.ttr_s:.3f}",
                        f"{g.nav_time_s:.2f}",
                    ]
                )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Analyze a NavLearn curve campaign",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--arm",
        action="append",
        required=True,
        metavar="NAME=DIR",
        help="Controller arm as name=directory (repeatable)",
    )
    parser.add_argument(
        "--range",
        required=True,
        metavar="LO:HI",
        help="Commanded magnitude range of the sweep, e.g. 0.01:3.0",
    )
    parser.add_argument("--bins", type=int, default=4)
    parser.add_argument(
        "--magnitude-field",
        choices=("displacement", "yaw_change"),
        default="displacement",
        help="Which column is the sweep's independent variable: the kidnap displacement "
        "(legs 1/2) or |Kidnap Yaw Change (deg)| (the yaw-curve leg, PROTOCOL A3)",
    )
    parser.add_argument(
        "--bin-scale",
        choices=("log", "linear"),
        default="log",
        help="Bin spacing. The yaw leg samples linearly from zero, which has no log",
    )
    parser.add_argument(
        "--expect-goals",
        type=int,
        default=None,
        help="Total goals per arm incl. attrition; refuses a partial arm when set",
    )
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()

    lo, hi = (float(x) for x in args.range.split(":", 1))
    if args.bin_scale == "log" and lo <= 0.0:
        sys.exit(
            "FATAL: log bins need a positive range minimum; use --bin-scale linear"
        )
    arms = {}
    for spec in args.arm:
        name, _, directory = spec.partition("=")
        if not directory:
            sys.exit(f"--arm expects NAME=DIR, got {spec!r}")
        arms[name] = load_arm(
            directory,
            expect_goals=args.expect_goals,
            magnitude_field=args.magnitude_field,
        )

    out = pathlib.Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)
    report = build_report(arms, lo=lo, hi=hi, bins=args.bins, bin_scale=args.bin_scale)
    (out / "summary.md").write_text(report + "\n")
    write_per_goal_csv(arms, out / "per_goal.csv")
    print(report)
    print(f"\nWritten: {out / 'summary.md'} and {out / 'per_goal.csv'}")


if __name__ == "__main__":
    main()
