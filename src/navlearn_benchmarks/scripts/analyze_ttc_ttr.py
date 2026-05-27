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
r"""
Analyze TTC/TTR localization experiment results and produce degradation curves.

Reads localization CSV files from structured experiment directories and produces:
  - Degradation curves (success rate vs perturbation level, PNG)
  - Recovery time box plots per mode and level (PNG)
  - Phase 1 vs Phase 2 comparison table (Markdown)
  - Tidy aggregated CSV (phase, mode, level, success_rate_pct, ...)

Expected directory structure::

    {base_dir}_{mode}/{level}/  (e.g. results/phase1_ttc/easy/)
        *metrics*.csv            (localization_metrics_*.csv or navlearn_metrics_*.csv)

CSV format: long/tidy with columns GoalID, Metric Class, Metric Source, Metric Name,
Metric Value, Timestamp.  Relevant metric names (case-sensitive):
    {MODE} Outcome -- int, 1 = converged/success, 0 = timeout, 2 = ended early, 3 = not armed
    {MODE}         -- float, convergence/recovery time in seconds

Usage::

    python3 analyze_ttc_ttr.py --phase1-dir results/phase1 --output-dir out/
    python3 analyze_ttc_ttr.py \\
        --phase1-dir results/phase1 \\
        --phase2-dir results/phase2 \\
        --output-dir out/
"""

import argparse
import csv
import logging
import pathlib
import sys
import warnings
from typing import Dict, List, Optional, Tuple

# ---------------------------------------------------------------------------
# Optional matplotlib — graceful degradation for headless / CI environments
# ---------------------------------------------------------------------------
try:
    import matplotlib  # noqa: F401

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    warnings.warn(
        "matplotlib not found — all plot outputs will be skipped. "
        "Install with: pip install matplotlib",
        stacklevel=2,
    )

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
MODES: Tuple[str, ...] = ("ttc", "ttr")
LEVELS: Tuple[str, ...] = ("easy", "medium", "hard", "extreme")
LEVEL_ORDER: Dict[str, int] = {lvl: i for i, lvl in enumerate(LEVELS)}

LOG = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments for the TTC/TTR analysis script."""
    parser = argparse.ArgumentParser(
        description="Analyze TTC/TTR localization degradation curves",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--phase1-dir",
        required=True,
        type=str,
        help="Root directory for Phase 1 experiment results",
    )
    parser.add_argument(
        "--phase2-dir",
        required=False,
        default=None,
        type=str,
        help="Root directory for Phase 2 experiment results (optional overlay)",
    )
    parser.add_argument(
        "--phase2b-dir",
        required=False,
        default=None,
        type=str,
        help="Root directory for Phase 2b ablation results (optional)",
    )
    parser.add_argument(
        "--phase3-dirs",
        required=False,
        default=None,
        nargs="+",
        type=str,
        help="Root directories for Phase 3 MPPI results (one per profile, optional)",
    )
    parser.add_argument(
        "--phase3-labels",
        required=False,
        default=None,
        nargs="+",
        type=str,
        help="Labels for --phase3-dirs entries (must match count; defaults to phase3_0, phase3_1…)",
    )
    parser.add_argument(
        "--output-dir",
        required=True,
        type=str,
        help="Directory for output plots, markdown table, and tidy CSV",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable debug-level logging",
    )
    return parser.parse_args()


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------


def find_csv_files(base_dir: pathlib.Path, mode: str, level: str) -> List[pathlib.Path]:
    """
    Return all localization/metrics CSV files for the given mode and level.

    Checks two directory conventions:
      1. {base_dir}_{mode}/{level}/  (e.g. results/phase1_ttc/easy/)
      2. {base_dir}/{mode}_{level}/ (legacy flat layout)
    """
    candidates = [
        base_dir.parent / f"{base_dir.name}_{mode}" / level,
        base_dir / f"{mode}_{level}",
    ]
    for subdir in candidates:
        if subdir.is_dir():
            files = sorted(subdir.glob("*metrics*.csv"))
            LOG.debug("Found %d CSV file(s) in %s", len(files), subdir)
            return files

    LOG.debug(
        "No subdirectory found for mode=%s level=%s under %s", mode, level, base_dir
    )
    return []


def parse_localization_csv(
    csv_path: pathlib.Path, mode: str
) -> Tuple[List[int], List[Optional[float]]]:
    """
    Parse a single localization CSV and extract outcomes and recovery times.

    The CSV is in long (tidy) format written by ``localization_metrics``::

        GoalID,Metric Class,Metric Source,Metric Name,Metric Value,Timestamp

    Metric names of interest (case-insensitive match):
      - ``TTC Outcome`` / ``TTR Outcome`` — integer outcome code (1 = success)
      - ``TTC`` / ``TTR`` — recovery/convergence time in seconds

    Parameters
    ----------
    csv_path:
        Path to the localization CSV file.
    mode:
        Either ``"ttc"`` or ``"ttr"``.

    Returns
    -------
    outcomes:
        List of integer outcome values (1 = success).
    times:
        List of recovery time values in seconds; ``None`` if not available.

    """
    mode_upper = mode.upper()
    outcome_metric = f"{mode_upper} Outcome"
    time_metric = mode_upper  # "TTC" or "TTR"

    # Collect per-goal data: {goal_id: {"outcome": int, "time": float|None}}
    goal_data: Dict[str, Dict[str, object]] = {}

    with csv_path.open(newline="") as fh:
        reader = csv.DictReader(fh)
        if reader.fieldnames is None:
            LOG.warning("CSV has no header: %s", csv_path)
            return [], []

        # Verify this is the expected long-format schema
        required = {"GoalID", "Metric Name", "Metric Value"}
        if not required.issubset(set(reader.fieldnames)):
            LOG.warning(
                "CSV missing required columns %s in %s — file skipped",
                required - set(reader.fieldnames),
                csv_path,
            )
            return [], []

        for row in reader:
            metric_name = row.get("Metric Name", "").strip()
            goal_id = row.get("GoalID", "").strip()
            raw_value = row.get("Metric Value", "").strip()

            if not goal_id or not raw_value:
                continue

            if goal_id not in goal_data:
                goal_data[goal_id] = {}

            if metric_name == outcome_metric:
                try:
                    goal_data[goal_id]["outcome"] = int(float(raw_value))
                except ValueError:
                    pass
            elif metric_name == time_metric:
                try:
                    goal_data[goal_id]["time"] = float(raw_value)
                except ValueError:
                    pass

    outcomes: List[int] = []
    times: List[Optional[float]] = []

    for gid in sorted(goal_data.keys()):
        entry = goal_data[gid]
        if "outcome" in entry:
            outcomes.append(entry["outcome"])
            times.append(entry.get("time"))

    return outcomes, times


def collect_phase_data(
    base_dir: pathlib.Path,
) -> Dict[str, Dict[str, Dict[str, object]]]:
    """
    Walk the structured directory and aggregate outcomes/times per mode/level.

    Returns a nested dict::

        {mode: {level: {"outcomes": [...], "times": [...]}}}
    """
    data: Dict[str, Dict[str, Dict[str, object]]] = {}

    for mode in MODES:
        data[mode] = {}
        for level in LEVELS:
            csv_files = find_csv_files(base_dir, mode, level)
            all_outcomes: List[int] = []
            all_times: List[float] = []

            for csv_path in csv_files:
                outcomes, times = parse_localization_csv(csv_path, mode)
                all_outcomes.extend(outcomes)
                all_times.extend(t for t in times if t is not None)

            data[mode][level] = {
                "outcomes": all_outcomes,
                "times": all_times,
            }
            LOG.info(
                "mode=%s level=%s  trials=%d  time_samples=%d",
                mode,
                level,
                len(all_outcomes),
                len(all_times),
            )

    return data


# ---------------------------------------------------------------------------
# Statistics helpers
# ---------------------------------------------------------------------------


def compute_success_rate(outcomes: List[int]) -> Optional[float]:
    """Return success rate as a percentage (1 = success outcome), or None if no data."""
    if not outcomes:
        return None
    successes = sum(1 for o in outcomes if o == 1)
    return 100.0 * successes / len(outcomes)


def _mean(values: List[float]) -> Optional[float]:
    """Return arithmetic mean or None for empty lists."""
    return sum(values) / len(values) if values else None


# ---------------------------------------------------------------------------
# Output generators
# ---------------------------------------------------------------------------


def write_tidy_csv(
    phases: List[Tuple[str, Dict]],
    output_dir: pathlib.Path,
) -> pathlib.Path:
    """Write a tidy aggregated CSV with one row per (phase, mode, level)."""
    out_path = output_dir / "ttc_ttr_aggregated.csv"
    fieldnames = [
        "phase",
        "mode",
        "level",
        "success_rate_pct",
        "successes",
        "total",
        "mean_recovery_time_s",
    ]

    with out_path.open("w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()

        for phase_label, phase_data in phases:
            for mode in MODES:
                for level in LEVELS:
                    cell = phase_data[mode][level]
                    outcomes: List[int] = cell["outcomes"]
                    times: List[float] = cell["times"]

                    successes = sum(1 for o in outcomes if o == 1)
                    total = len(outcomes)
                    sr = (100.0 * successes / total) if total > 0 else ""
                    mean_t = _mean(times) if times else ""

                    writer.writerow(
                        {
                            "phase": phase_label,
                            "mode": mode,
                            "level": level,
                            "success_rate_pct": f"{sr:.2f}" if sr != "" else "",
                            "successes": successes,
                            "total": total,
                            "mean_recovery_time_s": (
                                f"{mean_t:.4f}" if mean_t != "" else ""
                            ),
                        }
                    )

    LOG.info("Tidy CSV written: %s", out_path)
    return out_path


def write_comparison_table(
    phases: List[Tuple[str, Dict]],
    output_dir: pathlib.Path,
) -> pathlib.Path:
    """Write a Markdown comparison table: success rates across all phases with deltas vs phase1."""
    out_path = output_dir / "ttc_ttr_comparison.md"

    phase_labels = [label for label, _ in phases]
    # Header: Mode | Level | P1 | P2 | Delta(P2-P1) | P3 | Delta(P3-P1) ...
    header_cols = ["Mode", "Level"]
    sep_cols = ["------|", "-------|"]
    for i, label in enumerate(phase_labels):
        header_cols.append(f"{label.upper()} (%)")
        sep_cols.append("--------------|")
        if i > 0:
            header_cols.append(f"Δ vs {phase_labels[0].upper()} (pp)")
            sep_cols.append("------------|")

    lines = [
        "# TTC/TTR Phase Comparison\n",
        "| " + " | ".join(header_cols) + " |",
        "|" + "".join(sep_cols),
    ]

    baseline_data = phases[0][1]
    for mode in MODES:
        for level in LEVELS:
            sr_baseline = compute_success_rate(baseline_data[mode][level]["outcomes"])
            row = [mode.upper(), level.capitalize()]
            for i, (label, phase_data) in enumerate(phases):
                outcomes = phase_data[mode][level]["outcomes"]
                sr = compute_success_rate(outcomes)
                row.append(f"{sr:.1f}" if sr is not None else "—")
                if i > 0:
                    if sr is not None and sr_baseline is not None:
                        delta = sr - sr_baseline
                        sign = "+" if delta >= 0 else ""
                        row.append(f"{sign}{delta:.1f}")
                    else:
                        row.append("—")
            lines.append("| " + " | ".join(row) + " |")

    out_path.write_text("\n".join(lines) + "\n")
    LOG.info("Comparison table written: %s", out_path)
    return out_path


_PLOT_COLORS = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b"]
_PLOT_MARKERS = ["o", "s", "^", "D", "v", "P"]
_PLOT_STYLES = ["-", "--", "-.", ":", "-", "--"]


def _plot_degradation_curves(
    phases: List[Tuple[str, Dict]],
    output_dir: pathlib.Path,
) -> None:
    """Plot success rate vs perturbation level for all phases and modes."""
    if not HAS_MATPLOTLIB:
        LOG.warning("Skipping degradation curve plots — matplotlib unavailable")
        return

    x_labels = list(LEVELS)
    x_positions = list(range(len(LEVELS)))

    for mode in MODES:
        fig, ax = plt.subplots(figsize=(7, 4))

        for i, (label, phase_data) in enumerate(phases):
            sr_vals = []
            for level in LEVELS:
                sr = compute_success_rate(phase_data[mode][level]["outcomes"])
                sr_vals.append(sr if sr is not None else float("nan"))

            ax.plot(
                x_positions,
                sr_vals,
                marker=_PLOT_MARKERS[i % len(_PLOT_MARKERS)],
                linewidth=2,
                linestyle=_PLOT_STYLES[i % len(_PLOT_STYLES)],
                label=label.upper(),
                color=_PLOT_COLORS[i % len(_PLOT_COLORS)],
            )

        ax.set_xticks(x_positions)
        ax.set_xticklabels([lbl.capitalize() for lbl in x_labels])
        ax.set_xlabel("Perturbation Level")
        ax.set_ylabel("Success Rate (%)")
        ax.set_ylim(-5, 105)
        ax.set_title(f"{mode.upper()} Degradation Curve")
        ax.legend()
        ax.grid(axis="y", linestyle="--", alpha=0.5)
        fig.tight_layout()

        out_path = output_dir / f"{mode}_degradation.png"
        fig.savefig(out_path, dpi=150)
        plt.close(fig)
        LOG.info("Degradation curve saved: %s", out_path)


def _plot_recovery_box_plots(
    phases: List[Tuple[str, Dict]],
    output_dir: pathlib.Path,
) -> None:
    """Plot recovery time box plots per mode, grouped by perturbation level, one box per phase."""
    if not HAS_MATPLOTLIB:
        LOG.warning("Skipping box plot output — matplotlib unavailable")
        return

    n_phases = len(phases)
    for mode in MODES:
        fig, ax = plt.subplots(figsize=(8, 5))

        n_levels = len(LEVELS)
        width = 0.8 / n_phases
        offsets = [((j - (n_phases - 1) / 2) * width) for j in range(n_phases)]

        for i, level in enumerate(LEVELS):
            for j, (label, phase_data) in enumerate(phases):
                times: List[float] = phase_data[mode][level]["times"]
                pos = i + offsets[j]
                color = _PLOT_COLORS[j % len(_PLOT_COLORS)]

                if times:
                    bp = ax.boxplot(
                        times,
                        positions=[pos],
                        widths=width * 0.9,
                        patch_artist=True,
                        boxprops=dict(facecolor=color, alpha=0.6),
                        medianprops=dict(color="black", linewidth=1.5),
                        whiskerprops=dict(linestyle="--"),
                        showfliers=True,
                        manage_ticks=False,
                    )
                    if i == 0:
                        bp["boxes"][0].set_label(label.upper())

        ax.set_xticks(range(n_levels))
        ax.set_xticklabels([lvl.capitalize() for lvl in LEVELS])
        ax.set_xlabel("Perturbation Level")
        ax.set_ylabel("Recovery Time (s)")
        ax.set_title(f"{mode.upper()} Recovery Times")
        if n_phases > 1:
            ax.legend()
        ax.grid(axis="y", linestyle="--", alpha=0.4)
        fig.tight_layout()

        out_path = output_dir / f"{mode}_recovery_times.png"
        fig.savefig(out_path, dpi=150)
        plt.close(fig)
        LOG.info("Recovery time box plot saved: %s", out_path)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main() -> int:
    """Run the TTC/TTR analysis pipeline and return an exit code."""
    args = parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)s %(name)s: %(message)s",
    )

    output_dir = pathlib.Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    # Build ordered phases list: phase1 always first
    phase1_dir = pathlib.Path(args.phase1_dir).expanduser().resolve()
    if not phase1_dir.is_dir():
        LOG.error("Phase 1 directory not found: %s", phase1_dir)
        return 1

    LOG.info("Loading Phase 1 data from: %s", phase1_dir)
    all_phases: List[Tuple[str, Dict]] = [("phase1", collect_phase_data(phase1_dir))]

    if args.phase2_dir:
        p2 = pathlib.Path(args.phase2_dir).expanduser().resolve()
        if not p2.is_dir():
            LOG.error("Phase 2 directory not found: %s", p2)
            return 1
        LOG.info("Loading Phase 2 data from: %s", p2)
        all_phases.append(("phase2", collect_phase_data(p2)))

    if args.phase2b_dir:
        p2b = pathlib.Path(args.phase2b_dir).expanduser().resolve()
        if not p2b.is_dir():
            LOG.warning("Phase 2b directory not found (skipping): %s", p2b)
        else:
            LOG.info("Loading Phase 2b data from: %s", p2b)
            all_phases.append(("phase2b", collect_phase_data(p2b)))

    if args.phase3_dirs:
        labels = args.phase3_labels or [
            f"phase3_{i}" for i in range(len(args.phase3_dirs))
        ]
        if len(labels) != len(args.phase3_dirs):
            LOG.error(
                "--phase3-labels count (%d) != --phase3-dirs count (%d)",
                len(labels),
                len(args.phase3_dirs),
            )
            return 1
        for p3_path_str, p3_label in zip(args.phase3_dirs, labels):
            p3 = pathlib.Path(p3_path_str).expanduser().resolve()
            if not p3.is_dir():
                LOG.warning("Phase 3 directory not found (skipping): %s", p3)
                continue
            LOG.info("Loading %s data from: %s", p3_label, p3)
            all_phases.append((p3_label, collect_phase_data(p3)))

    # Generate outputs
    write_tidy_csv(all_phases, output_dir)
    write_comparison_table(all_phases, output_dir)
    _plot_degradation_curves(all_phases, output_dir)
    _plot_recovery_box_plots(all_phases, output_dir)

    LOG.info(
        "Analysis complete. %d phases. Outputs written to: %s",
        len(all_phases),
        output_dir,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
