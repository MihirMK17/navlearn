#!/usr/bin/env python3
"""§9 verification (b): nav_time distribution baseline vs high_tolerance.

Reads results/spatial_bin_analysis/all_goals_pose_outcome.csv (already
extracts Nav Time (sec) per goal with profile/mode/level/seed labels)
and compares the nav_time distribution for mppi_baseline vs
mppi_baseline_high_tolerance under TTC extreme.

Two questions answered:
  1. Among SUCCEEDED goals only, does high_tolerance reach the goal
     faster, slower, or equivalently to baseline? (Mann-Whitney U)
  2. Counting only the SUCCEEDED subset, what is the central tendency
     and spread of nav_time?

Writes results/phase_comparison/nav_time_distribution.json + console
summary.
"""

from __future__ import annotations

import json
import pathlib

import numpy as np
import pandas as pd
from scipy.stats import mannwhitneyu

WS = pathlib.Path("/home/mihirmk/robot_ws")
SRC = WS / "results" / "spatial_bin_analysis" / "all_goals_pose_outcome.csv"
OUT = WS / "results" / "phase_comparison" / "nav_time_distribution.json"

PROFILES = ["mppi_baseline", "mppi_baseline_high_tolerance"]
MODE = "ttc"
LEVEL = "extreme"


def summarize(arr: np.ndarray) -> dict:
    if arr.size == 0:
        return {"n": 0}
    return {
        "n": int(arr.size),
        "mean": float(np.mean(arr)),
        "median": float(np.median(arr)),
        "std": float(np.std(arr, ddof=1)) if arr.size > 1 else 0.0,
        "min": float(np.min(arr)),
        "max": float(np.max(arr)),
        "q25": float(np.quantile(arr, 0.25)),
        "q75": float(np.quantile(arr, 0.75)),
    }


def main() -> int:
    df = pd.read_csv(SRC)
    df = df[(df["mode"] == MODE) & (df["level"] == LEVEL)]
    df["Nav Time (sec)"] = pd.to_numeric(df["Nav Time (sec)"], errors="coerce")

    result = {
        "source": str(SRC.relative_to(WS)),
        "filter": {"mode": MODE, "level": LEVEL},
        "profiles": {},
        "succeeded_only": {},
        "all_outcomes": {},
        "mannwhitney_succeeded": {},
    }

    for prof in PROFILES:
        sub = df[df["profile"] == prof]
        succ = sub[sub["Goal Result"].astype(str).str.upper() == "SUCCEEDED"]
        all_t = sub["Nav Time (sec)"].dropna().to_numpy()
        succ_t = succ["Nav Time (sec)"].dropna().to_numpy()

        result["profiles"][prof] = {
            "n_goals": int(len(sub)),
            "n_succeeded": int(len(succ)),
            "succ_rate_pct": (
                (100.0 * len(succ) / len(sub)) if len(sub) else float("nan")
            ),
            "seeds_present": sorted(sub["seed"].unique().tolist()),
        }
        result["all_outcomes"][prof] = summarize(all_t)
        result["succeeded_only"][prof] = summarize(succ_t)

    # Mann-Whitney U on succeeded subset (non-parametric, no normality assumption)
    b_succ = (
        df[
            (df["profile"] == "mppi_baseline")
            & (df["Goal Result"].astype(str).str.upper() == "SUCCEEDED")
        ]["Nav Time (sec)"]
        .dropna()
        .to_numpy()
    )
    h_succ = (
        df[
            (df["profile"] == "mppi_baseline_high_tolerance")
            & (df["Goal Result"].astype(str).str.upper() == "SUCCEEDED")
        ]["Nav Time (sec)"]
        .dropna()
        .to_numpy()
    )

    if b_succ.size > 0 and h_succ.size > 0:
        u, p_two = mannwhitneyu(b_succ, h_succ, alternative="two-sided")
        _, p_h_lt_b = mannwhitneyu(h_succ, b_succ, alternative="less")
        _, p_h_gt_b = mannwhitneyu(h_succ, b_succ, alternative="greater")
        result["mannwhitney_succeeded"] = {
            "U_statistic": float(u),
            "p_two_sided": float(p_two),
            "p_h_lt_b_one_sided": float(p_h_lt_b),
            "p_h_gt_b_one_sided": float(p_h_gt_b),
            "n_baseline_succ": int(b_succ.size),
            "n_high_tol_succ": int(h_succ.size),
        }

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(result, indent=2))

    # Console summary
    print("=" * 76)
    print(f"§9 (b) nav_time distribution — mode={MODE} level={LEVEL}")
    print("=" * 76)
    for prof in PROFILES:
        p = result["profiles"][prof]
        s = result["succeeded_only"][prof]
        a = result["all_outcomes"][prof]
        print(f"\n--- {prof} ---")
        print(
            f"  goals: n={p['n_goals']}  succ={p['n_succeeded']}  "
            f"succ_rate={p['succ_rate_pct']:.1f}%  seeds={p['seeds_present']}"
        )
        if s.get("n", 0):
            print("  succeeded nav_time (s):")
            print(
                f"    n={s['n']}  mean={s['mean']:.2f}  median={s['median']:.2f}  "
                f"std={s['std']:.2f}  IQR=[{s['q25']:.2f}, {s['q75']:.2f}]  "
                f"range=[{s['min']:.2f}, {s['max']:.2f}]"
            )
        if a.get("n", 0):
            print("  all outcomes nav_time (s):")
            print(
                f"    n={a['n']}  mean={a['mean']:.2f}  median={a['median']:.2f}  "
                f"std={a['std']:.2f}"
            )

    mw = result["mannwhitney_succeeded"]
    if mw:
        print("\n--- Mann-Whitney U (succeeded only, baseline vs high_tolerance) ---")
        print(
            f"  n_baseline_succ={mw['n_baseline_succ']}  n_high_tol_succ={mw['n_high_tol_succ']}"
        )
        print(f"  U = {mw['U_statistic']:.1f}")
        print(f"  two-sided p = {mw['p_two_sided']:.4f}")
        print(
            f"  one-sided p (high_tol nav_time < baseline) = {mw['p_h_lt_b_one_sided']:.4f}"
        )
        print(
            f"  one-sided p (high_tol nav_time > baseline) = {mw['p_h_gt_b_one_sided']:.4f}"
        )

    print(f"\nWrote {OUT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
