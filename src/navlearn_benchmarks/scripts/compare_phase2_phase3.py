#!/usr/bin/env python3
# Copyright 2026 NavLearn Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
"""
Compare Phase 2 (baseline AMCL + RPP) vs Phase 3 (MPPI baseline + aggressive) results.

For each (perturbation level, harness mode) cell, computes:
  - Success rate + Wilson 95% CI for each profile
  - Fisher's exact p-value for Phase 2 baseline vs each Phase 3 profile
  - Mean / std of nav_time, path_length, ATE_RMSE, RPE, control_energy
    (restricted to successful episodes only)

Reads the mixed long/wide-format navlearn_metrics CSVs produced by metrics_compiler.
Only the wide-format rows (one per completed goal, NF==30, second column == 'map')
are used to extract per-episode quantitative metrics; the long-format AMCLEval /
Episode Event rows are filtered out.

Writes a tidy CSV to results/phase_comparison/phase2_vs_phase3_summary.csv and
prints a publication-ready table to stdout.  Idempotent — safe to re-run while
Phase 3 is still in progress; missing levels are skipped with a notice.

Usage::

    python3 compare_phase2_phase3.py
    python3 compare_phase2_phase3.py --watch         # re-run every 5 min
    python3 compare_phase2_phase3.py --interval 60   # custom watch interval (s)
"""

from __future__ import annotations

import argparse
import csv
import logging
import math
import pathlib
import sys
import time
from typing import Dict, List, Optional, Tuple

import numpy as np
import pandas as pd
from scipy.stats import fisher_exact, norm

# ---------------------------------------------------------------------------
# Layout constants
# ---------------------------------------------------------------------------

REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
RESULTS_DIR = REPO_ROOT / "results"
OUT_DIR = RESULTS_DIR / "phase_comparison"
OUT_CSV = OUT_DIR / "phase2_vs_phase3_summary.csv"

MODES: Tuple[str, ...] = ("ttc", "ttr")
LEVELS: Tuple[str, ...] = ("easy", "medium", "hard", "extreme")

# (profile_label, results subdir prefix)
PROFILES: List[Tuple[str, str]] = [
    ("phase2_baseline", "phase2"),
    ("phase3_mppi_baseline", "phase3_mppi_baseline"),
    ("phase3_mppi_aggressive", "phase3_mppi_aggressive"),
]

# Wide-format columns of interest (per metrics_compiler header row)
WIDE_HEADER_COL0 = "Goal_ID"
WIDE_NCOLS = 30
NUMERIC_COLS = {
    "nav_time": "Nav Time (sec)",
    "path_length": "Path Length (m)",
    "ate_rmse": "Absolute Path Error RMS (m)",
    "rpe": "Relative Pose Error (Drift)",
    "control_energy": "Control Energy",
}
SUCCESS_COL = "Goal Result"

LOG = logging.getLogger("compare_phase2_phase3")


# ---------------------------------------------------------------------------
# CSV discovery + parsing
# ---------------------------------------------------------------------------


def find_csv_files(profile_prefix: str, mode: str, level: str) -> List[pathlib.Path]:
    """Return all navlearn_metrics CSVs for one (profile, mode, level) cell.

    Mirrors find_csv_files() in analyze_ttc_ttr.py: expects layout
    ``results/{profile_prefix}_{mode}/{level}/*metrics*.csv``.
    """
    subdir = RESULTS_DIR / f"{profile_prefix}_{mode}" / level
    if not subdir.is_dir():
        return []
    return sorted(subdir.glob("*metrics*.csv"))


def _extract_wide_rows(csv_path: pathlib.Path) -> pd.DataFrame:
    """Return a DataFrame of wide-format metrics_compiler rows from one CSV.

    The CSV commingles long-format (6 cols, AMCLEval / Episode Event / TF jitter
    rows) and wide-format (30 cols, one per completed goal) records.  We detect
    the wide rows by parsing the file in two passes:

      1. Find the Goal_ID header line — captures the wide column names.
      2. Stream all lines and keep only rows with WIDE_NCOLS fields whose
         second column equals 'map' (the reference frame).

    Returns an empty DataFrame if no wide rows are present.
    """
    header: Optional[List[str]] = None
    data_rows: List[List[str]] = []

    try:
        with csv_path.open(newline="") as fh:
            reader = csv.reader(fh)
            for row in reader:
                if not row:
                    continue
                if (
                    header is None
                    and row[0] == WIDE_HEADER_COL0
                    and len(row) == WIDE_NCOLS
                ):
                    header = row
                    continue
                if len(row) == WIDE_NCOLS and len(row) > 1 and row[1] == "map":
                    data_rows.append(row)
    except OSError as exc:
        LOG.warning("Cannot read %s: %s", csv_path, exc)
        return pd.DataFrame()

    if header is None or not data_rows:
        return pd.DataFrame()

    df = pd.DataFrame(data_rows, columns=header)
    # Coerce known numeric columns; leave Goal Result as string ("SUCCEEDED"/etc.)
    for canonical, col_name in NUMERIC_COLS.items():
        if col_name in df.columns:
            df[col_name] = pd.to_numeric(df[col_name], errors="coerce")
    return df


def load_cell(profile_prefix: str, mode: str, level: str) -> pd.DataFrame:
    """Concatenate wide-format episodes for one (profile, mode, level) cell."""
    csvs = find_csv_files(profile_prefix, mode, level)
    if not csvs:
        return pd.DataFrame()
    frames = [_extract_wide_rows(p) for p in csvs]
    frames = [f for f in frames if not f.empty]
    if not frames:
        return pd.DataFrame()
    return pd.concat(frames, ignore_index=True)


# ---------------------------------------------------------------------------
# Statistics (mirrors statistical_tests.py)
# ---------------------------------------------------------------------------


def wilson_ci(successes: int, total: int, alpha: float = 0.05) -> Tuple[float, float]:
    """Wilson score confidence interval for a proportion."""
    if total == 0:
        return (float("nan"), float("nan"))
    z = norm.ppf(1 - alpha / 2)
    p = successes / total
    denom = 1 + z**2 / total
    centre = (p + z**2 / (2 * total)) / denom
    margin = z * math.sqrt(p * (1 - p) / total + z**2 / (4 * total**2)) / denom
    return (max(0.0, centre - margin), min(1.0, centre + margin))


def fisher_p(s1: int, n1: int, s2: int, n2: int) -> float:
    """Two-sided Fisher's exact p-value; NaN if either group is empty."""
    if n1 == 0 or n2 == 0:
        return float("nan")
    table = [[s1, n1 - s1], [s2, n2 - s2]]
    _, p_value = fisher_exact(table, alternative="two-sided")
    return p_value


def _is_success(value: object) -> bool:
    """Map the wide-format Goal Result column to a boolean success flag."""
    s = str(value).strip().upper()
    return s in {"SUCCEEDED", "SUCCESS", "1", "TRUE"}


def cell_stats(df: pd.DataFrame) -> Dict[str, object]:
    """Compute n / n_succ / per-metric mean+std for one cell DataFrame."""
    if df.empty:
        return {"n": 0, "n_succ": 0}
    if SUCCESS_COL in df.columns:
        successes_mask = df[SUCCESS_COL].apply(_is_success)
    else:
        successes_mask = pd.Series([False] * len(df))
    n = len(df)
    n_succ = int(successes_mask.sum())

    out: Dict[str, object] = {"n": n, "n_succ": n_succ}
    succ_df = df[successes_mask]
    for canonical, col_name in NUMERIC_COLS.items():
        if col_name not in succ_df.columns or succ_df[col_name].dropna().empty:
            out[f"{canonical}_mean"] = float("nan")
            out[f"{canonical}_std"] = float("nan")
        else:
            vals = succ_df[col_name].dropna().to_numpy()
            out[f"{canonical}_mean"] = float(np.mean(vals))
            out[f"{canonical}_std"] = float(
                np.std(vals, ddof=1) if len(vals) > 1 else 0.0
            )
    return out


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------


def _fmt_pct(x: float) -> str:
    return "  --  " if math.isnan(x) else f"{100*x:5.1f}%"


def _fmt_ci(lo: float, hi: float) -> str:
    if math.isnan(lo):
        return "[ -- ,  -- ]"
    return f"[{100*lo:4.0f}%,{100*hi:4.0f}%]"


def _fmt_p(p: float) -> str:
    if math.isnan(p):
        return "  --   "
    if p < 0.001:
        return "<0.001*"
    return f"{p:6.3f}"


def _fmt_mean_std(mean: float, std: float) -> str:
    if math.isnan(mean):
        return "    --     "
    return f"{mean:6.2f}±{std:5.2f}"


def collect_all() -> List[Dict[str, object]]:
    """Walk every (mode, level, profile) cell and build summary rows."""
    rows: List[Dict[str, object]] = []
    skipped: List[str] = []

    for mode in MODES:
        for level in LEVELS:
            cells: Dict[str, Tuple[pd.DataFrame, Dict[str, object]]] = {}
            for label, prefix in PROFILES:
                df = load_cell(prefix, mode, level)
                if df.empty and label.startswith("phase3"):
                    skipped.append(f"{label}/{mode}/{level}")
                cells[label] = (df, cell_stats(df))

            # Phase 2 baseline reference values for Fisher tests
            base_stats = cells["phase2_baseline"][1]
            s_base = int(base_stats.get("n_succ", 0))
            n_base = int(base_stats.get("n", 0))

            for label, _ in PROFILES:
                df, stats = cells[label]
                n = int(stats["n"])
                s = int(stats["n_succ"])
                rate = (s / n) if n else float("nan")
                lo, hi = wilson_ci(s, n)

                if label == "phase2_baseline":
                    p_fisher = float("nan")
                else:
                    p_fisher = fisher_p(s_base, n_base, s, n)

                row: Dict[str, object] = {
                    "level": level,
                    "mode": mode,
                    "profile": label,
                    "n": n,
                    "n_succ": s,
                    "rate": rate,
                    "wilson_lo": lo,
                    "wilson_hi": hi,
                    "p_fisher_vs_ph2": p_fisher,
                }
                for canonical in NUMERIC_COLS:
                    row[f"{canonical}_mean"] = stats.get(
                        f"{canonical}_mean", float("nan")
                    )
                    row[f"{canonical}_std"] = stats.get(
                        f"{canonical}_std", float("nan")
                    )
                rows.append(row)

    if skipped:
        # Group skip notices by profile for compactness
        by_profile: Dict[str, List[str]] = {}
        for s in skipped:
            prof = s.split("/", 1)[0]
            by_profile.setdefault(prof, []).append(s.split("/", 1)[1])
        for prof, cells_list in by_profile.items():
            LOG.info(
                "Phase 3 data not yet available for %s: %s", prof, ", ".join(cells_list)
            )

    return rows


def write_summary_csv(rows: List[Dict[str, object]], path: pathlib.Path) -> None:
    """Write the full per-(level,mode,profile) summary to CSV."""
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        LOG.warning("No rows to write to %s", path)
        return
    fieldnames = list(rows[0].keys())
    with path.open("w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        for r in rows:
            # Stringify floats with reasonable precision; keep NaN as empty
            out_row = {}
            for k, v in r.items():
                if isinstance(v, float):
                    out_row[k] = "" if math.isnan(v) else f"{v:.6g}"
                else:
                    out_row[k] = v
            writer.writerow(out_row)
    LOG.info("Wrote %d rows to %s", len(rows), path)


def print_table(rows: List[Dict[str, object]]) -> None:
    """Render a stdout summary table grouped by (mode, level)."""
    by_cell: Dict[Tuple[str, str], Dict[str, Dict[str, object]]] = {}
    for r in rows:
        by_cell.setdefault((r["mode"], r["level"]), {})[r["profile"]] = r

    header = (
        f"{'mode':4s} {'level':7s} {'profile':24s} "
        f"{'n':>4s} {'succ':>5s} {'rate':>6s} {'95% CI':>13s} {'p vs Ph2':>8s} "
        f"{'nav_t (s)':>12s} {'path (m)':>12s} {'ATE (m)':>12s} {'RPE':>12s} {'ctrl_E':>12s}"
    )
    print()
    print("=" * len(header))
    print("NavLearn Phase 2 vs Phase 3 (MPPI) Comparison")
    print("=" * len(header))
    print(header)
    print("-" * len(header))

    for mode in MODES:
        for level in LEVELS:
            cell = by_cell.get((mode, level), {})
            if not cell:
                continue
            for label, _ in PROFILES:
                r = cell.get(label)
                if r is None:
                    continue
                line = (
                    f"{r['mode']:4s} {r['level']:7s} {r['profile']:24s} "
                    f"{r['n']:>4d} {r['n_succ']:>5d} {_fmt_pct(r['rate']):>6s} "
                    f"{_fmt_ci(r['wilson_lo'], r['wilson_hi']):>13s} "
                    f"{_fmt_p(r['p_fisher_vs_ph2']):>8s} "
                    f"{_fmt_mean_std(r['nav_time_mean'], r['nav_time_std']):>12s} "
                    f"{_fmt_mean_std(r['path_length_mean'], r['path_length_std']):>12s} "
                    f"{_fmt_mean_std(r['ate_rmse_mean'], r['ate_rmse_std']):>12s} "
                    f"{_fmt_mean_std(r['rpe_mean'], r['rpe_std']):>12s} "
                    f"{_fmt_mean_std(r['control_energy_mean'], r['control_energy_std']):>12s}"
                )
                print(line)
            print("-" * len(header))
    print()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def run_once() -> None:
    """Single pass: collect data, print table, write CSV."""
    rows = collect_all()
    print_table(rows)
    write_summary_csv(rows, OUT_CSV)


def main() -> int:
    """CLI entry point with optional --watch loop."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument(
        "--watch",
        action="store_true",
        help="Re-run continuously while Phase 3 is in progress (^C to exit)",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=300.0,
        help="Watch interval in seconds (default: 300 = 5 minutes)",
    )
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)s %(name)s: %(message)s",
    )

    if not args.watch:
        run_once()
        return 0

    LOG.info("Watch mode: re-running every %.0fs. Ctrl-C to exit.", args.interval)
    try:
        while True:
            run_once()
            LOG.info("Sleeping %.0fs ...", args.interval)
            time.sleep(args.interval)
    except KeyboardInterrupt:
        LOG.info("Interrupted by user — exiting watch loop.")
        return 0


if __name__ == "__main__":
    sys.exit(main())
