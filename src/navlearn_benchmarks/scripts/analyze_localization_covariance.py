#!/usr/bin/env python3
# Copyright 2026 NavLearn Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
"""Localization-covariance reduction + false-success mechanism analysis.

NavLearn already LOGS the AMCL covariance diagonal as a ~10 Hz time-series
(``Localization Quality / AMCLPose / {CovXX, CovYY, CovYawYaw}``) but never
reports it.  This script turns that raw series into per-goal summary statistics
and then tests the central mechanism question behind the false-success gap:

    At episodes Nav2 marks SUCCEEDED but where AMCL never converged on the
    true pose (TTC Outcome == 0, i.e. a *false success*), is AMCL's own
    position-covariance trace elevated relative to true successes?

If yes, the covariance trace is a *deployable* false-success detector: it is
AMCL's self-reported confidence and needs no ground-truth pose, unlike the
TTC predicate (which compares against the simulator's GT pose).  That would
upgrade the false-success finding from a GT-only inference to an online,
field-usable signal.

Per-goal reduction of the position-covariance trace ``tr = CovXX + CovYY``:
  - ``trace_mean``   : mean over the episode
  - ``trace_median`` : robust central value
  - ``trace_peak``   : max (worst-case diffusion)
  - ``trace_final``  : mean of the last 10% of samples (steady-state confidence)
plus heading covariance (``CovYawYaw`` mean/peak) and pose-jump stability
(``Localization Stability / TF / {Jump Linear, Jump Angular}`` count + max).

Episode classification mirrors ``false_success_analysis.py`` EXACTLY so the
buckets reconcile with the paper's 38% headline:
  - true_success : Goal Result == 1  AND  TTC Outcome == 1
  - false_success: Goal Result == 1  AND  TTC Outcome == 0
  - true_fail    : Goal Result == 0  AND  TTC Outcome == 0
  - amcl_only    : Goal Result == 0  AND  TTC Outcome == 1

Reads the mixed long/wide navlearn_metrics CSVs (long 6-col rows only).
Writes JSON to results/phase_comparison/localization_covariance_analysis.json
and prints a publication-oriented table to stdout.

Usage::

    python3 analyze_localization_covariance.py
    python3 analyze_localization_covariance.py --run-sets phase2_ttc phase3_mppi_baseline_ttc
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import pathlib
import sys
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy.stats import mannwhitneyu

WS = pathlib.Path("/home/mihirmk/robot_ws")
RESULTS_DIR = WS / "results"
OUT_PATH = RESULTS_DIR / "phase_comparison" / "localization_covariance_analysis.json"

LEVELS: Tuple[str, ...] = ("easy", "medium", "hard", "extreme")

# TTC run-sets that carry the AMCL covariance time-series (verified: phase1_ttc,
# sensorrate, and plain-nav do NOT log it).  These are where the false-success
# gap is defined, so they are the headline targets.
DEFAULT_RUN_SETS: Tuple[str, ...] = (
    "phase2_ttc",
    "phase3_mppi_baseline_ttc",
    "phase3_mppi_aggressive_ttc",
)

# Fraction of trailing samples used for the steady-state ("final") reduction.
FINAL_FRAC = 0.10
# Pose jump (m) above which a TF discontinuity counts as a relocalization event.
JUMP_EPS_M = 0.05

CLASS_ORDER = ("true_success", "false_success", "true_fail", "amcl_only")


# ---------------------------------------------------------------------------
# Per-goal record
# ---------------------------------------------------------------------------


class GoalRecord:
    """Accumulates one goal's episode-summary fields + covariance series."""

    __slots__ = (
        "level",
        "goal_result",
        "ttc_outcome",
        "ttc_s",
        "nav_time",
        "cov_xx",
        "cov_yy",
        "cov_yaw",
        "jump_lin",
    )

    def __init__(self, level: Optional[str]) -> None:
        self.level = level
        self.goal_result: Optional[str] = None
        self.ttc_outcome: Optional[str] = None
        self.ttc_s: Optional[float] = None
        self.nav_time: Optional[float] = None
        self.cov_xx: List[float] = []
        self.cov_yy: List[float] = []
        self.cov_yaw: List[float] = []
        self.jump_lin: List[float] = []

    def classify(self) -> str:
        """Bucket per the false_success_analysis.py rule; '' if undetermined."""
        if self.goal_result is None or self.ttc_outcome is None:
            return ""
        nav_ok = self.goal_result.strip() == "1"
        ttc_ok = self.ttc_outcome.strip() == "1"
        if nav_ok and ttc_ok:
            return "true_success"
        if nav_ok and not ttc_ok:
            return "false_success"
        if not nav_ok and not ttc_ok:
            return "true_fail"
        return "amcl_only"

    def reduce(self) -> Optional[Dict[str, float]]:
        """Reduce the covariance series to per-goal scalars; None if no samples."""
        n = min(len(self.cov_xx), len(self.cov_yy))
        if n == 0:
            return None
        trace = np.asarray(self.cov_xx[:n]) + np.asarray(self.cov_yy[:n])
        k = max(1, int(math.ceil(n * FINAL_FRAC)))
        yaw = np.asarray(self.cov_yaw) if self.cov_yaw else np.asarray([np.nan])
        jumps = np.asarray(self.jump_lin) if self.jump_lin else np.asarray([])
        return {
            "n_cov_samples": float(n),
            "trace_mean": float(np.mean(trace)),
            "trace_median": float(np.median(trace)),
            "trace_peak": float(np.max(trace)),
            "trace_final": float(np.mean(trace[-k:])),
            "yaw_mean": float(np.nanmean(yaw)),
            "yaw_peak": float(np.nanmax(yaw)),
            "n_jumps": float(int(np.sum(jumps > JUMP_EPS_M))) if jumps.size else 0.0,
            "max_jump": float(np.max(jumps)) if jumps.size else 0.0,
            "ttc_s": float(self.ttc_s) if self.ttc_s is not None else float("nan"),
            "nav_time": (
                float(self.nav_time) if self.nav_time is not None else float("nan")
            ),
        }


# ---------------------------------------------------------------------------
# CSV parsing
# ---------------------------------------------------------------------------


def _to_float(value: str) -> Optional[float]:
    """Parse a metric value to float, returning None on failure."""
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def parse_csv(
    path: pathlib.Path, level: Optional[str], goals: Dict[str, GoalRecord]
) -> None:
    """Stream one CSV, routing long-format (6-col) rows into per-goal records.

    Mutates ``goals`` in place (keyed by GoalID).  Wide-format rows (30 cols)
    and the header row are skipped.
    """
    try:
        with path.open(newline="") as fh:
            for row in csv.reader(fh):
                if len(row) != 6 or row[0] == "GoalID":
                    continue
                gid, mclass, _msource, mname, mval, _ts = row
                rec = goals.get(gid)
                if rec is None:
                    rec = GoalRecord(level)
                    goals[gid] = rec

                if mclass == "Localization Quality":
                    if mname == "CovXX":
                        v = _to_float(mval)
                        if v is not None:
                            rec.cov_xx.append(v)
                    elif mname == "CovYY":
                        v = _to_float(mval)
                        if v is not None:
                            rec.cov_yy.append(v)
                    elif mname == "CovYawYaw":
                        v = _to_float(mval)
                        if v is not None:
                            rec.cov_yaw.append(v)
                elif mclass == "Localization Stability":
                    if mname == "Jump Linear":
                        v = _to_float(mval)
                        if v is not None:
                            rec.jump_lin.append(v)
                elif mname == "Goal Result":
                    rec.goal_result = mval
                elif mname == "TTC Outcome":
                    rec.ttc_outcome = mval
                elif mname == "TTC":
                    rec.ttc_s = _to_float(mval)
                elif mname == "Navigation Time":
                    rec.nav_time = _to_float(mval)
    except OSError as exc:
        print(f"WARN: cannot read {path}: {exc}", file=sys.stderr)


def load_run_set(run_set: str) -> Dict[str, GoalRecord]:
    """Load all goals for a run-set, walking its {level}/ subdirectories."""
    base = RESULTS_DIR / run_set
    goals: Dict[str, GoalRecord] = {}
    if not base.is_dir():
        return goals
    for level in LEVELS:
        level_dir = base / level
        if not level_dir.is_dir():
            continue
        for csv_path in sorted(level_dir.glob("navlearn_metrics_run_*.csv")):
            parse_csv(csv_path, level, goals)
    return goals


# ---------------------------------------------------------------------------
# Aggregation + the headline test
# ---------------------------------------------------------------------------


def _summ(values: List[float]) -> Dict[str, float]:
    """median / mean / n summary for a list (NaN-safe, empties -> NaN)."""
    arr = np.asarray([v for v in values if not math.isnan(v)], dtype=float)
    if arr.size == 0:
        return {"n": 0, "median": float("nan"), "mean": float("nan")}
    return {
        "n": int(arr.size),
        "median": float(np.median(arr)),
        "mean": float(np.mean(arr)),
    }


def headline_test(true_vals: List[float], false_vals: List[float]) -> Dict[str, object]:
    """Mann-Whitney U of false- vs true-success trace + rank-AUC separability.

    AUC = U / (n_true * n_false) is the probability a random false-success goal
    has a higher trace than a random true-success goal (0.5 = no separation,
    1.0 = covariance perfectly flags false successes without ground truth).
    """
    t = np.asarray([v for v in true_vals if not math.isnan(v)], dtype=float)
    f = np.asarray([v for v in false_vals if not math.isnan(v)], dtype=float)
    out: Dict[str, object] = {
        "n_true": int(t.size),
        "n_false": int(f.size),
        "median_true": float(np.median(t)) if t.size else float("nan"),
        "median_false": float(np.median(f)) if f.size else float("nan"),
        "p_mannwhitney": float("nan"),
        "auc_false_gt_true": float("nan"),
    }
    if t.size >= 1 and f.size >= 1 and (t.size + f.size) >= 3:
        # alternative='greater': is false-success trace stochastically larger?
        u_stat, p = mannwhitneyu(f, t, alternative="greater")
        out["p_mannwhitney"] = float(p)
        out["auc_false_gt_true"] = float(u_stat / (t.size * f.size))
    return out


def analyze_run_set(run_set: str) -> Dict[str, object]:
    """Build the per-level + pooled covariance/class summary for one run-set."""
    goals = load_run_set(run_set)

    # Per-goal reduced records tagged with class + level.
    reduced: List[Dict[str, object]] = []
    for rec in goals.values():
        cls = rec.classify()
        red = rec.reduce()
        if not cls:
            continue
        reduced.append({"level": rec.level, "class": cls, "red": red})

    def traces(level: Optional[str], cls: str) -> List[float]:
        return [
            r["red"]["trace_final"]
            for r in reduced
            if r["class"] == cls
            and r["red"] is not None
            and (level is None or r["level"] == level)
        ]

    per_level: Dict[str, object] = {}
    for level in LEVELS:
        counts = {
            c: sum(1 for r in reduced if r["level"] == level and r["class"] == c)
            for c in CLASS_ORDER
        }
        n_cov = sum(1 for r in reduced if r["level"] == level and r["red"] is not None)
        if sum(counts.values()) == 0:
            continue
        cls_trace = {
            c: _summ(
                [
                    r["red"]["trace_final"]
                    for r in reduced
                    if r["level"] == level and r["class"] == c and r["red"] is not None
                ]
            )
            for c in CLASS_ORDER
        }
        per_level[level] = {
            "counts": counts,
            "n_with_cov": n_cov,
            "trace_final_by_class": cls_trace,
            "headline": headline_test(
                traces(level, "true_success"), traces(level, "false_success")
            ),
        }

    def vals(cls: str, key: str) -> List[float]:
        return [
            r["red"][key] for r in reduced if r["class"] == cls and r["red"] is not None
        ]

    # Sweep the candidate per-goal reductions to see which (if any) separates
    # true- from false-success.  trace_final can wash out the signal if AMCL
    # re-collapses (overconfident) by episode end, so peak/mean are checked too.
    reduction_sweep = {
        key: headline_test(vals("true_success", key), vals("false_success", key))
        for key in ("trace_mean", "trace_peak", "trace_final", "yaw_mean", "n_jumps")
    }

    pooled = {
        "counts": {c: sum(1 for r in reduced if r["class"] == c) for c in CLASS_ORDER},
        "trace_final_by_class": {
            c: _summ(
                [
                    r["red"]["trace_final"]
                    for r in reduced
                    if r["class"] == c and r["red"] is not None
                ]
            )
            for c in CLASS_ORDER
        },
        "headline": headline_test(
            traces(None, "true_success"), traces(None, "false_success")
        ),
        "reduction_sweep": reduction_sweep,
    }
    return {"n_goals": len(goals), "per_level": per_level, "pooled": pooled}


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------


def _fmt(x: float, w: int = 7, p: int = 3) -> str:
    return f"{'--':>{w}}" if (x is None or math.isnan(x)) else f"{x:>{w}.{p}f}"


def _fmt_p(p: float) -> str:
    if math.isnan(p):
        return "  --   "
    return "<0.001 " if p < 0.001 else f"{p:6.3f} "


def print_report(result: Dict[str, object]) -> None:
    """Render the stdout summary."""
    for run_set, data in result["run_sets"].items():
        print("=" * 92)
        print(f"{run_set}   (goals={data['n_goals']})")
        print("=" * 92)
        hdr = (
            f"{'level':8s} | {'true_s':>6s} {'FALSE_s':>7s} {'fail':>5s} {'amcl':>5s} | "
            f"{'tr_true':>8s} {'tr_FALSE':>8s} | {'AUC':>5s} {'MWU p':>7s}"
        )
        print(hdr)
        print("-" * len(hdr))
        for level in LEVELS:
            lv = data["per_level"].get(level)
            if lv is None:
                continue
            c = lv["counts"]
            tf = lv["trace_final_by_class"]
            h = lv["headline"]
            print(
                f"{level:8s} | {c['true_success']:>6d} {c['false_success']:>7d} "
                f"{c['true_fail']:>5d} {c['amcl_only']:>5d} | "
                f"{_fmt(tf['true_success']['median'], 8):>8s} "
                f"{_fmt(tf['false_success']['median'], 8):>8s} | "
                f"{_fmt(h['auc_false_gt_true'], 5, 2):>5s} {_fmt_p(h['p_mannwhitney']):>7s}"
            )
        p = data["pooled"]
        c, tf, h = p["counts"], p["trace_final_by_class"], p["headline"]
        print("-" * len(hdr))
        print(
            f"{'POOLED':8s} | {c['true_success']:>6d} {c['false_success']:>7d} "
            f"{c['true_fail']:>5d} {c['amcl_only']:>5d} | "
            f"{_fmt(tf['true_success']['median'], 8):>8s} "
            f"{_fmt(tf['false_success']['median'], 8):>8s} | "
            f"{_fmt(h['auc_false_gt_true'], 5, 2):>5s} {_fmt_p(h['p_mannwhitney']):>7s}"
        )
        print(
            f"  trace_final median: true-success={_fmt(tf['true_success']['median'], 6)}  "
            f"false-success={_fmt(tf['false_success']['median'], 6)}  "
            f"(AUC={_fmt(h['auc_false_gt_true'], 4, 2)} = P[cov flags false success w/o GT])"
        )
        sweep = p.get("reduction_sweep", {})
        if sweep:
            print("  reduction sweep (false vs true success separability):")
            print(
                f"    {'reduction':12s} {'med_true':>9s} {'med_false':>9s} "
                f"{'AUC':>5s} {'MWU p':>7s}"
            )
            for key, s in sweep.items():
                print(
                    f"    {key:12s} {_fmt(s['median_true'], 9):>9s} "
                    f"{_fmt(s['median_false'], 9):>9s} {_fmt(s['auc_false_gt_true'], 5, 2):>5s} "
                    f"{_fmt_p(s['p_mannwhitney']):>7s}"
                )
        print()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main() -> int:
    """CLI entry point."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--run-sets",
        nargs="+",
        default=list(DEFAULT_RUN_SETS),
        help="results/ subdirectory names (with {level}/ children)",
    )
    args = parser.parse_args()

    result: Dict[str, object] = {
        "false_success_definition": "Goal Result==1 AND TTC Outcome==0",
        "trace_definition": "CovXX + CovYY (AMCL position-covariance diagonal trace)",
        "final_frac": FINAL_FRAC,
        "run_sets": {},
    }
    for run_set in args.run_sets:
        result["run_sets"][run_set] = analyze_run_set(run_set)

    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUT_PATH.write_text(json.dumps(result, indent=2))

    print_report(result)
    print(f"Wrote: {OUT_PATH}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
