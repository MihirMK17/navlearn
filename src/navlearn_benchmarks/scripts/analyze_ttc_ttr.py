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

    {base_dir}/{mode}_{level}/  (e.g. ttc_easy/, ttr_medium/)
        *localization*.csv

CSV columns expected (per row / trial):
    {mode}_outcome   -- int, 0 = success
    {mode}_time_sec  -- float, recovery time in seconds

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
    """Return all localization CSV files under {base_dir}/{mode}_{level}/."""
    subdir = base_dir / f"{mode}_{level}"
    if not subdir.is_dir():
        LOG.debug("Subdirectory not found: %s", subdir)
        return []
    files = sorted(subdir.glob("*localization*.csv"))
    LOG.debug("Found %d CSV file(s) in %s", len(files), subdir)
    return files


def parse_localization_csv(
    csv_path: pathlib.Path, mode: str
) -> Tuple[List[int], List[Optional[float]]]:
    """
    Parse a single localization CSV and extract outcomes and recovery times.

    Parameters
    ----------
    csv_path:
        Path to the localization CSV file.
    mode:
        Either ``"ttc"`` or ``"ttr"``.

    Returns
    -------
    outcomes:
        List of integer outcome values (0 = success).
    times:
        List of recovery time values in seconds; ``None`` if column absent or blank.

    """
    outcome_col = f"{mode}_outcome"
    time_col = f"{mode}_time_sec"

    outcomes: List[int] = []
    times: List[Optional[float]] = []

    with csv_path.open(newline="") as fh:
        reader = csv.DictReader(fh)
        if reader.fieldnames is None:
            LOG.warning("CSV has no header: %s", csv_path)
            return outcomes, times

        has_outcome = outcome_col in reader.fieldnames
        has_time = time_col in reader.fieldnames

        if not has_outcome:
            LOG.warning(
                "Column '%s' missing in %s — file skipped", outcome_col, csv_path
            )
            return outcomes, times

        for row in reader:
            raw_outcome = row.get(outcome_col, "").strip()
            if raw_outcome == "":
                continue
            try:
                outcomes.append(int(raw_outcome))
            except ValueError:
                LOG.debug("Non-integer outcome '%s' in %s", raw_outcome, csv_path)
                continue

            if has_time:
                raw_time = row.get(time_col, "").strip()
                try:
                    times.append(float(raw_time) if raw_time else None)
                except ValueError:
                    times.append(None)
            else:
                times.append(None)

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
    """Return success rate as a percentage (0 = success outcome), or None if no data."""
    if not outcomes:
        return None
    successes = sum(1 for o in outcomes if o == 0)
    return 100.0 * successes / len(outcomes)


def _mean(values: List[float]) -> Optional[float]:
    """Return arithmetic mean or None for empty lists."""
    return sum(values) / len(values) if values else None


# ---------------------------------------------------------------------------
# Output generators
# ---------------------------------------------------------------------------


def write_tidy_csv(
    phase1_data: Dict,
    phase2_data: Optional[Dict],
    output_dir: pathlib.Path,
) -> pathlib.Path:
    """
    Write a tidy aggregated CSV with one row per (phase, mode, level).

    Columns: phase, mode, level, success_rate_pct, successes, total,
             mean_recovery_time_s
    """
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

    phases = [("phase1", phase1_data)]
    if phase2_data is not None:
        phases.append(("phase2", phase2_data))

    with out_path.open("w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()

        for phase_label, phase_data in phases:
            for mode in MODES:
                for level in LEVELS:
                    cell = phase_data[mode][level]
                    outcomes: List[int] = cell["outcomes"]
                    times: List[float] = cell["times"]

                    successes = sum(1 for o in outcomes if o == 0)
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
    phase1_data: Dict,
    phase2_data: Optional[Dict],
    output_dir: pathlib.Path,
) -> pathlib.Path:
    """
    Write a Markdown comparison table: Phase 1 vs Phase 2 success rates with delta.

    Columns: Mode | Level | P1 Success% | P2 Success% | Delta
    """
    out_path = output_dir / "ttc_ttr_comparison.md"

    lines = [
        "# TTC/TTR Phase Comparison\n",
        "| Mode | Level | P1 Success (%) | P2 Success (%) | Delta (pp) |",
        "|------|-------|---------------|----------------|-----------|",
    ]

    for mode in MODES:
        for level in LEVELS:
            outcomes1 = phase1_data[mode][level]["outcomes"]
            sr1 = compute_success_rate(outcomes1)
            sr1_str = f"{sr1:.1f}" if sr1 is not None else "—"

            sr2_str = "—"
            delta_str = "—"
            if phase2_data is not None:
                outcomes2 = phase2_data[mode][level]["outcomes"]
                sr2 = compute_success_rate(outcomes2)
                sr2_str = f"{sr2:.1f}" if sr2 is not None else "—"
                if sr1 is not None and sr2 is not None:
                    delta = sr2 - sr1
                    sign = "+" if delta >= 0 else ""
                    delta_str = f"{sign}{delta:.1f}"

            lines.append(
                f"| {mode.upper()} | {level.capitalize()} | {sr1_str} | {sr2_str} | {delta_str} |"
            )

    out_path.write_text("\n".join(lines) + "\n")
    LOG.info("Comparison table written: %s", out_path)
    return out_path


def _plot_degradation_curves(
    phase1_data: Dict,
    phase2_data: Optional[Dict],
    output_dir: pathlib.Path,
) -> None:
    """
    Plot success rate vs perturbation level for TTC and TTR.

    Generates one PNG per mode (ttc_degradation.png, ttr_degradation.png).
    Phase 1 is always plotted; Phase 2 is overlaid when available.
    """
    if not HAS_MATPLOTLIB:
        LOG.warning("Skipping degradation curve plots — matplotlib unavailable")
        return

    x_labels = list(LEVELS)
    x_positions = list(range(len(LEVELS)))

    for mode in MODES:
        fig, ax = plt.subplots(figsize=(7, 4))

        # Phase 1
        sr1_vals = []
        for level in LEVELS:
            sr = compute_success_rate(phase1_data[mode][level]["outcomes"])
            sr1_vals.append(sr if sr is not None else float("nan"))

        ax.plot(
            x_positions,
            sr1_vals,
            marker="o",
            linewidth=2,
            label="Phase 1",
            color="#1f77b4",
        )

        # Phase 2 overlay
        if phase2_data is not None:
            sr2_vals = []
            for level in LEVELS:
                sr = compute_success_rate(phase2_data[mode][level]["outcomes"])
                sr2_vals.append(sr if sr is not None else float("nan"))

            ax.plot(
                x_positions,
                sr2_vals,
                marker="s",
                linewidth=2,
                linestyle="--",
                label="Phase 2",
                color="#ff7f0e",
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
    phase1_data: Dict,
    phase2_data: Optional[Dict],
    output_dir: pathlib.Path,
) -> None:
    """
    Plot recovery time box plots per mode, grouped by perturbation level.

    Generates one PNG per mode (ttc_recovery_times.png, ttr_recovery_times.png).
    """
    if not HAS_MATPLOTLIB:
        LOG.warning("Skipping box plot output — matplotlib unavailable")
        return

    for mode in MODES:
        fig, ax = plt.subplots(figsize=(8, 5))

        n_levels = len(LEVELS)
        width = 0.35
        has_p2 = phase2_data is not None

        for i, level in enumerate(LEVELS):
            times1: List[float] = phase1_data[mode][level]["times"]

            if has_p2:
                times2: List[float] = phase2_data[mode][level]["times"]  # type: ignore[index]
                pos1 = i - width / 2
                pos2 = i + width / 2
                datasets = [
                    (times1, pos1, "Phase 1", "#1f77b4"),
                    (times2, pos2, "Phase 2", "#ff7f0e"),
                ]
            else:
                datasets = [(times1, i, "Phase 1", "#1f77b4")]

            for times, pos, label, color in datasets:
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
                    # Label only once per phase
                    if i == 0:
                        bp["boxes"][0].set_label(label)

        ax.set_xticks(range(n_levels))
        ax.set_xticklabels([lvl.capitalize() for lvl in LEVELS])
        ax.set_xlabel("Perturbation Level")
        ax.set_ylabel("Recovery Time (s)")
        ax.set_title(f"{mode.upper()} Recovery Times")
        if has_p2:
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

    phase1_dir = pathlib.Path(args.phase1_dir).expanduser().resolve()
    phase2_dir = (
        pathlib.Path(args.phase2_dir).expanduser().resolve()
        if args.phase2_dir
        else None
    )
    output_dir = pathlib.Path(args.output_dir).expanduser().resolve()

    # Validate inputs
    if not phase1_dir.is_dir():
        LOG.error("Phase 1 directory not found: %s", phase1_dir)
        return 1
    if phase2_dir is not None and not phase2_dir.is_dir():
        LOG.error("Phase 2 directory not found: %s", phase2_dir)
        return 1

    output_dir.mkdir(parents=True, exist_ok=True)

    # Load data
    LOG.info("Loading Phase 1 data from: %s", phase1_dir)
    phase1_data = collect_phase_data(phase1_dir)

    phase2_data = None
    if phase2_dir is not None:
        LOG.info("Loading Phase 2 data from: %s", phase2_dir)
        phase2_data = collect_phase_data(phase2_dir)

    # Generate outputs
    write_tidy_csv(phase1_data, phase2_data, output_dir)
    write_comparison_table(phase1_data, phase2_data, output_dir)
    _plot_degradation_curves(phase1_data, phase2_data, output_dir)
    _plot_recovery_box_plots(phase1_data, phase2_data, output_dir)

    LOG.info("Analysis complete. Outputs written to: %s", output_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
