#!/usr/bin/env python3
# Copyright 2026 NavLearn Contributors
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
"""
Aggregate NavLearn benchmark results across multiple runs.

Reads all navlearn_run_report_run_*.json files from an input directory,
computes per-metric statistics (mean, std, percentiles, outliers),
and optionally computes SPL.

Usage:
    python3 aggregate_runs.py --input-dir benchmark_reports/runs/baseline
    python3 aggregate_runs.py --input-dir /path/to/runs --output-format csv
"""

import argparse
import json
import logging
import math
import pathlib
import sys
from typing import Any, Dict, List, Optional, Tuple


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Aggregate NavLearn multi-run benchmark results",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--input-dir",
        type=str,
        required=True,
        help="Directory containing navlearn_run_report_run_*.json files",
    )
    parser.add_argument(
        "--output-format",
        type=str,
        choices=["text", "csv", "json"],
        default="text",
        help="Output format for the summary table",
    )
    parser.add_argument(
        "--precision",
        type=int,
        default=3,
        help="Decimal places for floating-point values",
    )
    return parser.parse_args()


def load_run_summaries(directory: pathlib.Path) -> List[Tuple[str, Dict[str, Any]]]:
    """Load all run JSON summaries from directory, sorted by filename."""
    summaries = []
    for path in sorted(directory.glob("navlearn_run_report_run_*.json")):
        with open(path, "r") as f:
            data = json.load(f)
        summaries.append((path.name, data))
    return summaries


def safe_float(value: Any, fallback: float = float("nan")) -> float:
    """Convert value to float, returning fallback on failure."""
    try:
        return float(value)
    except (TypeError, ValueError):
        return fallback


def percentile(data: List[float], p: float) -> float:
    """Compute the p-th percentile of data (0-100) using linear interpolation."""
    if not data:
        return float("nan")
    sorted_data = sorted(data)
    n = len(sorted_data)
    index = (p / 100.0) * (n - 1)
    lower = int(index)
    upper = min(lower + 1, n - 1)
    frac = index - lower
    return sorted_data[lower] + frac * (sorted_data[upper] - sorted_data[lower])


def compute_stats(values: List[float]) -> Dict[str, float]:
    """Compute full statistics for a list of float values."""
    valid = [v for v in values if not math.isnan(v)]
    if not valid:
        nan = float("nan")
        return {
            "mean": nan,
            "std": nan,
            "min": nan,
            "max": nan,
            "p25": nan,
            "p50": nan,
            "p75": nan,
            "p95": nan,
            "count": 0,
        }
    n = len(valid)
    mean = sum(valid) / n
    variance = sum((x - mean) ** 2 for x in valid) / n if n > 1 else 0.0
    std = math.sqrt(variance)
    return {
        "mean": mean,
        "std": std,
        "min": min(valid),
        "max": max(valid),
        "p25": percentile(valid, 25),
        "p50": percentile(valid, 50),
        "p75": percentile(valid, 75),
        "p95": percentile(valid, 95),
        "count": n,
    }


def detect_outliers(values: List[float], threshold: float = 2.0) -> List[int]:
    """Return 0-based indices of values that are > threshold std devs from mean."""
    valid = [v for v in values if not math.isnan(v)]
    if len(valid) < 3:
        return []
    mean = sum(valid) / len(valid)
    std = math.sqrt(sum((x - mean) ** 2 for x in valid) / len(valid))
    if std == 0.0:
        return []
    return [
        i
        for i, v in enumerate(values)
        if not math.isnan(v) and abs(v - mean) > threshold * std
    ]


def compute_spl(
    success: bool, optimal_path_m: float, actual_path_m: float
) -> Optional[float]:
    """
    Compute Success weighted by inverse Path Length (SPL).

    Returns None if optimal_path_m is missing (-1) or zero.
    """
    if optimal_path_m <= 0 or actual_path_m < 0:
        return None
    if not success:
        return 0.0
    return optimal_path_m / max(actual_path_m, optimal_path_m)


def aggregate_summaries(
    summaries: List[Tuple[str, Dict[str, Any]]],
    precision: int = 3,
) -> Dict[str, Any]:
    """
    Aggregate all run summaries into per-metric statistics.

    Note: min_clearance_m is available per-goal in the per-run CSV output
    (written by MetricsCompiler), but is not yet included in the run summary
    JSON files. To aggregate min_clearance_m across goals/runs, read the CSV
    files directly. This will be added to the JSON summary in a future pass.
    """
    nav_times: List[float] = []
    path_lengths: List[float] = []
    ctrl_energies: List[float] = []
    spl_values: List[float] = []
    total_goals = 0
    total_succeeded = 0
    total_failed = 0
    total_canceled = 0

    for filename, d in summaries:
        goals = int(d.get("goals_total", 0))
        succ = int(d.get("goals_succeeded", 0))
        fail = int(d.get("goals_failed", 0))
        canceled = int(d.get("goals_canceled", 0))
        nav_mean = safe_float(d.get("nav_time_mean"))
        path_mean = safe_float(d.get("path_length_mean"))
        ctrl_mean = safe_float(d.get("control_energy_mean"))
        optimal_path = safe_float(d.get("optimal_path_mean", -1.0), -1.0)

        total_goals += goals
        total_succeeded += succ
        total_failed += fail
        total_canceled += canceled

        nav_times.append(nav_mean)
        path_lengths.append(path_mean)
        ctrl_energies.append(ctrl_mean)

        # SPL per run: requires optimal_path in JSON (may not exist in older runs)
        spl = compute_spl(succ == goals and goals > 0, optimal_path, path_mean)
        if spl is not None:
            spl_values.append(spl)

    success_rate = total_succeeded / total_goals if total_goals > 0 else 0.0

    result = {
        "runs": len(summaries),
        "total_goals": total_goals,
        "total_succeeded": total_succeeded,
        "total_failed": total_failed,
        "total_canceled": total_canceled,
        "success_rate": round(success_rate, precision),
        "nav_time_s": compute_stats(nav_times),
        "path_length_m": compute_stats(path_lengths),
        "control_energy": compute_stats(ctrl_energies),
        "nav_time_outliers": detect_outliers(nav_times),
        "path_length_outliers": detect_outliers(path_lengths),
    }

    if spl_values:
        result["spl"] = compute_stats(spl_values)

    return result


def format_stats(name: str, stats: Dict[str, float], precision: int = 3) -> str:
    """Format a stats dict into a human-readable string."""
    fmt = f".{precision}f"
    return (
        f"  {name}:\n"
        f"    mean={stats['mean']:{fmt}}  std={stats['std']:{fmt}}\n"
        f"    min={stats['min']:{fmt}}  p25={stats['p25']:{fmt}}  "
        f"p50={stats['p50']:{fmt}}  p75={stats['p75']:{fmt}}  "
        f"p95={stats['p95']:{fmt}}  max={stats['max']:{fmt}}\n"
        f"    count={stats['count']}"
    )


def print_text_report(agg: Dict[str, Any], precision: int = 3) -> None:
    """Print aggregated results as a formatted text report."""
    print(f"\n{'='*60}")
    print(f"NavLearn Benchmark Aggregation — {agg['runs']} runs")
    print(f"{'='*60}")
    print(
        f"  Goals:     {agg['total_goals']} total  "
        f"({agg['total_succeeded']} succeeded, "
        f"{agg['total_failed']} failed, "
        f"{agg['total_canceled']} canceled)"
    )
    print(f"  Success rate: {agg['success_rate']:.1%}")

    print(format_stats("Nav Time [s]", agg["nav_time_s"], precision))
    print(format_stats("Path Length [m]", agg["path_length_m"], precision))
    print(format_stats("Control Energy", agg["control_energy"], precision))

    if "spl" in agg:
        print(format_stats("SPL", agg["spl"], precision))

    if agg["nav_time_outliers"]:
        print(f"  Nav-time outlier runs (>2σ): {agg['nav_time_outliers']}")
    if agg["path_length_outliers"]:
        print(f"  Path-length outlier runs (>2σ): {agg['path_length_outliers']}")

    print(f"{'='*60}\n")


def print_csv_report(agg: Dict[str, Any], precision: int = 3) -> None:
    """Print aggregated results in CSV format."""
    metrics = ["nav_time_s", "path_length_m", "control_energy"]
    if "spl" in agg:
        metrics.append("spl")

    header = [
        "metric",
        "mean",
        "std",
        "min",
        "p25",
        "p50",
        "p75",
        "p95",
        "max",
        "count",
    ]
    print(",".join(header))
    fmt = f".{precision}f"
    for m in metrics:
        s = agg[m]
        row = [
            m,
            f"{s['mean']:{fmt}}",
            f"{s['std']:{fmt}}",
            f"{s['min']:{fmt}}",
            f"{s['p25']:{fmt}}",
            f"{s['p50']:{fmt}}",
            f"{s['p75']:{fmt}}",
            f"{s['p95']:{fmt}}",
            f"{s['max']:{fmt}}",
            str(s["count"]),
        ]
        print(",".join(row))


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%H:%M:%S",
    )

    args = parse_args()
    directory = pathlib.Path(args.input_dir)

    if not directory.exists():
        logging.error("Directory does not exist: %s", directory)
        sys.exit(1)

    summaries = load_run_summaries(directory)
    if not summaries:
        logging.error("No run summaries found in %s", directory)
        sys.exit(1)

    logging.info("Loaded %d run summaries from %s", len(summaries), directory)

    agg = aggregate_summaries(summaries, precision=args.precision)

    if args.output_format == "text":
        print_text_report(agg, precision=args.precision)
    elif args.output_format == "csv":
        print_csv_report(agg, precision=args.precision)
    elif args.output_format == "json":
        import json as _json

        print(_json.dumps(agg, indent=2, default=str))


if __name__ == "__main__":
    main()
