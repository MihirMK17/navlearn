#!/usr/bin/env python3
# Copyright 2026 NavLearn Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
"""Covariance-by-zone + spatial covariance heatmap (existing data, no runs).

Extends the verified covariance reduction (analyze_localization_covariance.py)
into the spatial dimension to probe the NARROW-vs-OPEN tension flagged in the
walkthrough (success-based binning says near-wall goal-starts fail more, but the
open-space inf-collapse hypothesis predicts open areas degrade localization):

  1. Bins each goal by start-pose distance-to-nearest-wall (NARROW / MID / OPEN),
     reusing spatial_zone_fisher's wall-distance transform, and compares per-goal
     AMCL covariance (CovXX, CovYY, trace = XX+YY, CovYawYaw) across zones, split
     by true/false success.
  2. Renders a spatial heatmap of the AMCL position-covariance trace over the
     map, from the logged AMCL mean-pose (MeanX/MeanY) + covariance time-series,
     showing WHERE the filter loses confidence (open rooms vs narrow corridors).

HONESTY / SCOPE:
  - The spatial bin and heatmap use the AMCL ESTIMATED pose, not ground truth
    (GT pose trajectory is not logged in existing CSVs). Where the estimate
    diverges, the placement is the believed (wrong) pose — usable as a first
    probe, not a definitive true-pose attribution.
  - Definitive true-pose mid/end binning, valid-beam-fraction by zone, and the
    lidar-availability heatmap all need the deferred re-run (with GT-trajectory
    + valid-beam logging). This script does the existing-data half.

Writes:
  results/phase_comparison/covariance_by_zone.csv
  results/phase_comparison/covariance_trace_heatmap.png

Usage::

    python3 analyze_covariance_by_zone.py
    python3 analyze_covariance_by_zone.py --run-sets phase2_ttc phase3_mppi_baseline_ttc
"""

from __future__ import annotations

import argparse
import csv
import math
import pathlib
import sys
from typing import Dict, List, Optional, Tuple

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from scipy.stats import mannwhitneyu  # noqa: E402

SCRIPT_DIR = pathlib.Path(__file__).parent.resolve()
sys.path.insert(0, str(SCRIPT_DIR))
from spatial_zone_fisher import (  # type: ignore  # noqa: E402
    load_map_and_wall_distance,
    world_to_rowcol,
)

WS = pathlib.Path("/home/mihirmk/robot_ws")
RESULTS_DIR = WS / "results"
OUT_CSV = RESULTS_DIR / "phase_comparison" / "covariance_by_zone.csv"
OUT_PNG = RESULTS_DIR / "phase_comparison" / "covariance_trace_heatmap.png"

LEVELS: Tuple[str, ...] = ("easy", "medium", "hard", "extreme")
DEFAULT_RUN_SETS: Tuple[str, ...] = (
    "phase2_ttc",
    "phase3_mppi_baseline_ttc",
    "phase3_mppi_aggressive_ttc",
)
QUARTILE_LOW, QUARTILE_HIGH = 0.25, 0.75


# ---------------------------------------------------------------------------
# Parsing: per-goal start pose (wide) + class + covariance series (long)
# ---------------------------------------------------------------------------


class Goal:
    """Per-goal start pose, success class, and aligned AMCL pose/cov series."""

    __slots__ = ("level", "goal_result", "ttc_outcome", "start_x", "start_y",
                 "mean_x", "mean_y", "cov_xx", "cov_yy", "cov_yaw", "jump_lin")

    def __init__(self, level: Optional[str]) -> None:
        self.level = level
        self.goal_result: Optional[str] = None
        self.ttc_outcome: Optional[str] = None
        self.start_x: Optional[float] = None
        self.start_y: Optional[float] = None
        self.mean_x: List[float] = []
        self.mean_y: List[float] = []
        self.cov_xx: List[float] = []
        self.cov_yy: List[float] = []
        self.cov_yaw: List[float] = []
        self.jump_lin: List[float] = []

    def classify(self) -> str:
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


def _f(v: str) -> Optional[float]:
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def parse_csv(path: pathlib.Path, level: Optional[str], goals: Dict[str, Goal]) -> None:
    """Stream one CSV; collect wide-row start pose + long-row class/cov series."""
    try:
        with path.open(newline="") as fh:
            for row in csv.reader(fh):
                if not row or row[0] in ("GoalID", "Goal_ID"):
                    continue
                # Wide row (30 cols, frame == 'map'): grab start pose.
                if len(row) == 30 and row[1] == "map":
                    g = goals.setdefault(row[0], Goal(level))
                    g.start_x, g.start_y = _f(row[2]), _f(row[3])
                    continue
                # Long row (6 cols): class + covariance/pose/stability series.
                if len(row) != 6:
                    continue
                gid, mclass, _src, mname, mval, _ts = row
                g = goals.setdefault(gid, Goal(level))
                if mclass == "Localization Quality":
                    target = {"CovXX": g.cov_xx, "CovYY": g.cov_yy,
                              "CovYawYaw": g.cov_yaw, "MeanX": g.mean_x,
                              "MeanY": g.mean_y}.get(mname)
                    if target is not None:
                        v = _f(mval)
                        if v is not None:
                            target.append(v)
                elif mclass == "Localization Stability" and mname == "Jump Linear":
                    v = _f(mval)
                    if v is not None:
                        g.jump_lin.append(v)
                elif mname == "Goal Result":
                    g.goal_result = mval
                elif mname == "TTC Outcome":
                    g.ttc_outcome = mval
    except OSError as exc:
        print(f"WARN: cannot read {path}: {exc}", file=sys.stderr)


def load_goals(run_sets: List[str]) -> Dict[str, Goal]:
    """Load every goal across the requested run-sets / levels."""
    goals: Dict[str, Goal] = {}
    for rs in run_sets:
        for level in LEVELS:
            d = RESULTS_DIR / rs / level
            if not d.is_dir():
                continue
            for c in sorted(d.glob("navlearn_metrics_run_*.csv")):
                parse_csv(c, level, goals)
    return goals


# ---------------------------------------------------------------------------
# Per-goal reductions
# ---------------------------------------------------------------------------


def reduce_goal(g: Goal) -> Optional[Dict[str, float]]:
    """Episode-mean covariance reductions (the verified-interpretable choice)."""
    n = min(len(g.cov_xx), len(g.cov_yy))
    if n == 0:
        return None
    xx = np.asarray(g.cov_xx[:n])
    yy = np.asarray(g.cov_yy[:n])
    trace = xx + yy
    yaw = np.asarray(g.cov_yaw) if g.cov_yaw else np.asarray([np.nan])
    jumps = np.asarray(g.jump_lin) if g.jump_lin else np.asarray([0.0])
    return {
        "covxx_mean": float(np.mean(xx)),
        "covyy_mean": float(np.mean(yy)),
        "trace_mean": float(np.mean(trace)),
        "trace_peak": float(np.max(trace)),
        "yaw_mean": float(np.nanmean(yaw)),
        "n_jumps": float(int(np.sum(jumps > 0.05))),
    }


# ---------------------------------------------------------------------------
# Zone analysis
# ---------------------------------------------------------------------------


def build_frame(goals: Dict[str, Goal], dist_wall_m, res, ox, oy, shape) -> pd.DataFrame:
    """One row per goal: pose, zone distance, class, covariance reductions."""
    rows: List[Dict[str, object]] = []
    for g in goals.values():
        red = reduce_goal(g)
        cls = g.classify()
        if red is None or not cls or g.start_x is None or g.start_y is None:
            continue
        r, c = world_to_rowcol(np.asarray([g.start_x]), np.asarray([g.start_y]),
                               res, ox, oy, shape)
        rows.append({
            "level": g.level, "class": cls,
            "start_x": g.start_x, "start_y": g.start_y,
            "d_wall_m": float(dist_wall_m[r[0], c[0]]),
            **red,
        })
    return pd.DataFrame(rows)


def _mwu(a: np.ndarray, b: np.ndarray) -> float:
    """One-sided MWU: is a stochastically greater than b? NaN if too small."""
    a = a[~np.isnan(a)]
    b = b[~np.isnan(b)]
    if a.size < 1 or b.size < 1 or (a.size + b.size) < 3:
        return float("nan")
    return float(mannwhitneyu(a, b, alternative="greater")[1])


def zone_table(df: pd.DataFrame, q_lo: float, q_hi: float) -> List[Dict[str, object]]:
    """Compare covariance reductions across NARROW / MID / OPEN zones."""
    def zone(d: float) -> str:
        if d <= q_lo:
            return "NARROW"
        if d >= q_hi:
            return "OPEN"
        return "MID"

    df = df.assign(zone=df["d_wall_m"].apply(zone))
    out: List[Dict[str, object]] = []
    for metric in ("trace_mean", "covxx_mean", "covyy_mean", "yaw_mean", "n_jumps"):
        rec: Dict[str, object] = {"metric": metric}
        for z in ("NARROW", "MID", "OPEN"):
            vals = df.loc[df["zone"] == z, metric].to_numpy(dtype=float)
            vals = vals[~np.isnan(vals)]
            rec[f"{z}_n"] = int(vals.size)
            rec[f"{z}_median"] = float(np.median(vals)) if vals.size else float("nan")
        rec["p_open_gt_narrow"] = _mwu(
            df.loc[df["zone"] == "OPEN", metric].to_numpy(dtype=float),
            df.loc[df["zone"] == "NARROW", metric].to_numpy(dtype=float),
        )
        out.append(rec)
    return out


def render_heatmap(goals: Dict[str, Goal], img, extent, dist_wall_m,
                   res, ox, oy, shape, out_path: pathlib.Path) -> None:
    """Spatial heatmap of mean cov-trace, binned over the AMCL estimated pose."""
    xs, ys, tr = [], [], []
    for g in goals.values():
        n = min(len(g.mean_x), len(g.mean_y), len(g.cov_xx), len(g.cov_yy))
        for i in range(n):
            xs.append(g.mean_x[i])
            ys.append(g.mean_y[i])
            tr.append(g.cov_xx[i] + g.cov_yy[i])
    if not xs:
        print("WARN: no pose/cov samples for heatmap", file=sys.stderr)
        return
    xs = np.asarray(xs)
    ys = np.asarray(ys)
    tr = np.asarray(tr)

    x0, x1, y0, y1 = extent
    nb = 60
    xe = np.linspace(x0, x1, nb + 1)
    ye = np.linspace(y0, y1, nb + 1)
    sum_tr, _, _ = np.histogram2d(xs, ys, bins=[xe, ye], weights=tr)
    cnt, _, _ = np.histogram2d(xs, ys, bins=[xe, ye])
    with np.errstate(invalid="ignore", divide="ignore"):
        mean_tr = np.where(cnt > 0, sum_tr / cnt, np.nan)

    fig, ax = plt.subplots(figsize=(11, 9))
    ax.imshow(img, extent=extent, origin="lower", cmap="gray", alpha=0.55, vmin=0, vmax=255)
    # 95th-percentile clip so a few divergence spikes don't wash out the field.
    vmax = float(np.nanpercentile(mean_tr, 95)) if np.isfinite(mean_tr).any() else 1.0
    im = ax.imshow(mean_tr.T, extent=extent, origin="lower", cmap="inferno",
                   alpha=0.75, vmin=0, vmax=max(vmax, 1e-6))
    cbar = plt.colorbar(im, ax=ax, fraction=0.04, pad=0.04)
    cbar.set_label("mean AMCL position-cov trace  CovXX+CovYY (m^2)")
    ax.set_xlabel("x (m, map frame)")
    ax.set_ylabel("y (m, map frame)")
    ax.set_title("Where AMCL loses confidence — cov-trace over estimated pose\n"
                 "(open rooms vs narrow corridors; estimated-pose-based)", fontsize=10)
    plt.tight_layout()
    plt.savefig(out_path, dpi=140)
    plt.close()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--run-sets", nargs="+", default=list(DEFAULT_RUN_SETS))
    args = parser.parse_args()

    print("Loading map + wall-distance transform...")
    img, extent, dist_wall_m, res, ox, oy = load_map_and_wall_distance()

    print(f"Loading goals from: {', '.join(args.run_sets)}")
    goals = load_goals(args.run_sets)
    df = build_frame(goals, dist_wall_m, res, ox, oy, img.shape)
    print(f"  goals with cov + start-pose: {len(df)}")

    # Zone thresholds from pooled extreme-TTC d_wall (mirrors spatial_zone_fisher).
    ext = df[df["level"] == "extreme"]
    base = ext if len(ext) >= 8 else df
    q_lo = float(base["d_wall_m"].quantile(QUARTILE_LOW))
    q_hi = float(base["d_wall_m"].quantile(QUARTILE_HIGH))
    print(f"  zone thresholds (n={len(base)}): NARROW <= {q_lo:.2f} m | OPEN >= {q_hi:.2f} m")

    all_rows: List[Dict[str, object]] = []
    for scope_name, scope_df in (("extreme_ttc", ext), ("all_levels", df)):
        if scope_df.empty:
            continue
        print(f"\n=== covariance by zone — {scope_name} (n={len(scope_df)}) ===")
        print(f"  {'metric':11s} | {'NARROW (n)':>14s} | {'MID (n)':>14s} | "
              f"{'OPEN (n)':>14s} | {'p(open>narrow)':>14s}")
        for rec in zone_table(scope_df, q_lo, q_hi):
            rec["scope"] = scope_name
            all_rows.append(rec)
            p = rec["p_open_gt_narrow"]
            ps = "  --   " if (isinstance(p, float) and math.isnan(p)) else f"{p:6.3f}"
            print(f"  {rec['metric']:11s} | "
                  f"{rec['NARROW_median']:8.4f} ({rec['NARROW_n']:>3d}) | "
                  f"{rec['MID_median']:8.4f} ({rec['MID_n']:>3d}) | "
                  f"{rec['OPEN_median']:8.4f} ({rec['OPEN_n']:>3d}) | {ps:>14s}")

    # False- vs true-success cov split WITHIN each zone (extreme-TTC).
    if not ext.empty:
        print("\n=== trace_mean: false vs true success, within zone (extreme_ttc) ===")
        ext_z = ext.assign(zone=ext["d_wall_m"].apply(
            lambda d: "NARROW" if d <= q_lo else ("OPEN" if d >= q_hi else "MID")))
        for z in ("NARROW", "MID", "OPEN"):
            zt = ext_z[(ext_z["zone"] == z) & (ext_z["class"] == "true_success")]["trace_mean"]
            zf = ext_z[(ext_z["zone"] == z) & (ext_z["class"] == "false_success")]["trace_mean"]
            tmed = float(np.median(zt)) if len(zt) else float("nan")
            fmed = float(np.median(zf)) if len(zf) else float("nan")
            print(f"  {z:7s}: true n={len(zt):>3d} med={tmed:8.4f}   "
                  f"false n={len(zf):>3d} med={fmed:8.4f}")

    OUT_CSV.parent.mkdir(parents=True, exist_ok=True)
    pd.DataFrame(all_rows).to_csv(OUT_CSV, index=False)
    render_heatmap(goals, img, extent, dist_wall_m, res, ox, oy, img.shape, OUT_PNG)
    print(f"\nWrote: {OUT_CSV}")
    print(f"Wrote: {OUT_PNG}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
