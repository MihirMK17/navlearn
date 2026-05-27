#!/usr/bin/env python3
"""Phase 3 xy_goal_tolerance single-variable ablation analyzer.

Cells (6): mppi_baseline_high_tolerance x extreme x {ttc, ttr} x seeds {42, 43, 44}
Each cell expected n=25 (5 episodes x 5 goals).

Compares against pre-computed pooled benchmarks from
results/phase_comparison/seed_replication_analysis.json:
  mppi_baseline pooled extreme TTC : 31.1% (23/74)
  mppi_baseline pooled extreme TTR :  9.3% (7/75)
  mppi_aggressive pooled extreme TTC: 60.0% (45/75)
  mppi_aggressive pooled extreme TTR: 16.0% (12/75)

Verdict logic:
  If high_tol ~ aggressive AND high_tol >> baseline -> xy_goal_tolerance DOMINANT.
  If high_tol ~ baseline   AND high_tol << aggressive -> xy_goal_tolerance NOT dominant.
  Else PARTIAL.

Writes JSON to results/phase_comparison/ablation_tolerance_analysis.json.
"""

from __future__ import annotations

import json
import math
import pathlib
import sys
from typing import Dict, List

# Reuse helpers from compare_phase2_phase3.py
SCRIPT_DIR = pathlib.Path(__file__).parent.resolve()
sys.path.insert(0, str(SCRIPT_DIR))
from compare_phase2_phase3 import (  # type: ignore
    _extract_wide_rows,
    cell_stats,
    fisher_p,
    wilson_ci,
)

WS = pathlib.Path("/home/mihirmk/robot_ws")
ABL_DIR = WS / "results" / "phase3_ablation_tolerance"
OUT_PATH = WS / "results" / "phase_comparison" / "ablation_tolerance_analysis.json"

# Pooled benchmarks (from seed_replication_analysis.json)
BENCHMARK = {
    "ttc": {
        "baseline":   {"n": 74, "n_succ": 23, "pct": 31.1},
        "aggressive": {"n": 75, "n_succ": 45, "pct": 60.0},
    },
    "ttr": {
        "baseline":   {"n": 75, "n_succ":  7, "pct":  9.3},
        "aggressive": {"n": 75, "n_succ": 12, "pct": 16.0},
    },
}

SEEDS = [42, 43, 44]
MODES = ["ttc", "ttr"]


def load_cell(mode: str, seed: int) -> Dict[str, object]:
    """Load all CSVs for one (mode, seed) cell, return cell_stats + raw n."""
    cell_dir = ABL_DIR / f"mppi_baseline_high_tolerance_{mode}_extreme_seed{seed}"
    csvs = sorted(cell_dir.glob("*metrics*.csv"))
    if not csvs:
        return {"n": 0, "n_succ": 0, "csv_count": 0}
    import pandas as pd
    frames = [_extract_wide_rows(p) for p in csvs]
    frames = [f for f in frames if not f.empty]
    if not frames:
        return {"n": 0, "n_succ": 0, "csv_count": len(csvs)}
    df = pd.concat(frames, ignore_index=True)
    stats = cell_stats(df)
    stats["csv_count"] = len(csvs)
    return stats


def verdict(high_pct: float, base_pct: float, agg_pct: float, p_vs_base: float, p_vs_agg: float) -> str:
    """Return DOMINANT / PARTIAL / NOT_DOMINANT verdict."""
    near_agg = (p_vs_agg >= 0.05) and (high_pct >= agg_pct - 15)
    diff_from_base = (p_vs_base < 0.05) and (high_pct >= base_pct + 15)
    if near_agg and diff_from_base:
        return "DOMINANT"
    if (not diff_from_base) and (high_pct < base_pct + 10):
        return "NOT_DOMINANT"
    return "PARTIAL"


def main() -> int:
    if not ABL_DIR.is_dir():
        print(f"ERROR: {ABL_DIR} not found", file=sys.stderr)
        return 1

    result: Dict[str, object] = {
        "benchmark": BENCHMARK,
        "ablation_profile": "mppi_baseline_high_tolerance",
        "single_variable_changed": "xy_goal_tolerance: 0.05 -> 0.10",
        "per_mode": {},
    }

    for mode in MODES:
        per_seed: List[Dict[str, object]] = []
        total_n = 0
        total_succ = 0
        for seed in SEEDS:
            stats = load_cell(mode, seed)
            n = int(stats.get("n", 0))
            s = int(stats.get("n_succ", 0))
            per_seed.append({
                "seed": seed,
                "n": n,
                "n_succ": s,
                "pct": (100.0 * s / n) if n else float("nan"),
                "ate_mean": stats.get("ate_mean", float("nan")),
                "ate_median": stats.get("ate_median", float("nan")),
                "nav_mean": stats.get("nav_mean", float("nan")),
                "csv_count": stats.get("csv_count", 0),
            })
            total_n += n
            total_succ += s

        pooled_pct = (100.0 * total_succ / total_n) if total_n else float("nan")
        lo, hi = wilson_ci(total_succ, total_n)
        base = BENCHMARK[mode]["baseline"]
        agg  = BENCHMARK[mode]["aggressive"]
        p_vs_base = fisher_p(total_succ, total_n, base["n_succ"], base["n"])
        p_vs_agg  = fisher_p(total_succ, total_n, agg["n_succ"],  agg["n"])
        v = verdict(pooled_pct, base["pct"], agg["pct"], p_vs_base, p_vs_agg)

        result["per_mode"][mode] = {
            "per_seed": per_seed,
            "pooled": {
                "n": total_n,
                "n_succ": total_succ,
                "pct": pooled_pct,
                "wilson_lo": lo,
                "wilson_hi": hi,
            },
            "fisher_vs_baseline": p_vs_base,
            "fisher_vs_aggressive": p_vs_agg,
            "verdict": v,
        }

    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUT_PATH.write_text(json.dumps(result, indent=2))

    # Pretty print
    print("=" * 76)
    print("Phase 3 xy_goal_tolerance Ablation — mppi_baseline_high_tolerance (= mppi_baseline + xy_tol 0.10)")
    print("=" * 76)
    for mode in MODES:
        r = result["per_mode"][mode]
        print(f"\n--- {mode.upper()} extreme ---")
        for ps in r["per_seed"]:
            print(f"  seed={ps['seed']}: n={ps['n']} succ={ps['n_succ']} pct={ps['pct']:5.1f}%  "
                  f"ate_mean={ps.get('ate_mean', float('nan')):7.2f}  nav_mean={ps.get('nav_mean', float('nan')):6.2f}")
        p = r["pooled"]
        print(f"  POOLED: n={p['n']} succ={p['n_succ']} pct={p['pct']:5.1f}% "
              f"[Wilson {100*p['wilson_lo']:4.1f}%, {100*p['wilson_hi']:4.1f}%]")
        b = BENCHMARK[mode]["baseline"]; a = BENCHMARK[mode]["aggressive"]
        print(f"  vs mppi_baseline ({b['pct']:.1f}%):   Fisher p = {r['fisher_vs_baseline']:.4f}")
        print(f"  vs mppi_aggressive ({a['pct']:.1f}%): Fisher p = {r['fisher_vs_aggressive']:.4f}")
        print(f"  VERDICT: {r['verdict']}")
    print(f"\nWrote: {OUT_PATH}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
