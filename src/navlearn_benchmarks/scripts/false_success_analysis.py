#!/usr/bin/env python3
"""Quantify the false-success gap from NavLearn TTC localization experiment CSVs.

False success: Nav2 reports SUCCEEDED while AMCL pose error never fell below
the convergence threshold (TTC = -1, i.e., TTC Outcome = 0).

Usage:
    python3 false_success_analysis.py --phase1-ttc results/phase1_ttc
    python3 false_success_analysis.py --phase1-ttc results/phase1_ttc \\
        --phase2-ttc results/phase2_ttc
    python3 false_success_analysis.py --phase1-ttc results/phase1_ttc \\
        --csv localization_evaluation.csv
"""

import argparse
import csv
import pathlib
import sys
from typing import Dict, List

LEVELS = ["easy", "medium", "hard", "extreme"]


def load_episodes_from_dir(base_dir: pathlib.Path) -> Dict[str, Dict[str, str]]:
    """Load TTC Outcome + Goal Result per GoalID from all level subdirectories."""
    episodes: Dict[str, Dict[str, str]] = {}
    for level in LEVELS:
        level_dir = base_dir / level
        if not level_dir.exists():
            continue
        patterns = ["localization_metrics_run_*.csv", "navlearn_metrics_run_*.csv"]
        csv_files = []
        for pat in patterns:
            csv_files.extend(level_dir.glob(pat))
        for csv_file in sorted(csv_files):
            with csv_file.open() as f:
                reader = csv.DictReader(f)
                if "GoalID" not in (reader.fieldnames or []):
                    continue  # skip old trajectory-format CSVs
                for row in reader:
                    gid = row["GoalID"]
                    name = row["Metric Name"]
                    val = row["Metric Value"]
                    episodes.setdefault(gid, {"level": level})
                    if name == "TTC Outcome":
                        episodes[gid]["ttc_outcome"] = val
                    elif name == "Goal Result":
                        episodes[gid]["goal_result"] = val
                    elif name == "Navigation Time":
                        episodes[gid]["nav_time"] = val
                    elif name == "TTC":
                        episodes[gid]["ttc_s"] = val
    return episodes


def load_episodes_from_csv(csv_path: pathlib.Path) -> Dict[str, Dict[str, str]]:
    """Load episodes from a flat localization_evaluation.csv (no level info)."""
    episodes: Dict[str, Dict[str, str]] = {}
    with csv_path.open() as f:
        for row in csv.DictReader(f):
            gid = row["GoalID"]
            name = row["Metric Name"]
            val = row["Metric Value"]
            episodes.setdefault(gid, {})
            if name == "TTC Outcome":
                episodes[gid]["ttc_outcome"] = val
            elif name == "Goal Result":
                episodes[gid]["goal_result"] = val
            elif name == "Navigation Time":
                episodes[gid]["nav_time"] = val
            elif name == "TTC":
                episodes[gid]["ttc_s"] = val
    return episodes


def classify_episodes(episodes: Dict) -> Dict[str, List]:
    """Classify episodes into true_success, false_success, true_fail, amcl_only."""
    buckets: Dict[str, List] = {
        "true_success": [],
        "false_success": [],
        "true_fail": [],
        "amcl_only": [],
        "unknown": [],
    }
    for gid, ep in episodes.items():
        goal = ep.get("goal_result", "")
        ttc = ep.get("ttc_outcome", "")
        if not goal or not ttc:
            buckets["unknown"].append(gid)
            continue
        nav_ok = goal.strip() == "1"
        ttc_ok = ttc.strip() == "1"
        if nav_ok and ttc_ok:
            buckets["true_success"].append(gid)
        elif nav_ok and not ttc_ok:
            buckets["false_success"].append(gid)
        elif not nav_ok and not ttc_ok:
            buckets["true_fail"].append(gid)
        else:
            buckets["amcl_only"].append(gid)
    return buckets


def mean_nav_time(gids: List[str], episodes: Dict) -> float:
    """Compute mean navigation time for a list of GoalIDs."""
    times = []
    for gid in gids:
        val = episodes.get(gid, {}).get("nav_time", "")
        try:
            times.append(float(val))
        except (ValueError, TypeError):
            pass
    return sum(times) / len(times) if times else float("nan")


def report(label: str, episodes: Dict, buckets: Dict) -> None:
    """Print full false-success gap report."""
    total = sum(len(v) for k, v in buckets.items() if k != "unknown")
    unk = len(buckets["unknown"])
    fs = len(buckets["false_success"])
    ts = len(buckets["true_success"])
    tf = len(buckets["true_fail"])
    ao = len(buckets["amcl_only"])
    nav_ok = ts + fs

    print(f"\n=== False-Success Gap Analysis: {label} ===")
    print(f"Total episodes: {total} (+ {unk} with missing data)")
    print(f"  True success  (nav OK + AMCL conv):    {ts:3d} ({100*ts/max(total,1):.1f}%)")
    print(f"  FALSE SUCCESS (nav OK + AMCL NOT conv): {fs:3d} ({100*fs/max(total,1):.1f}%)")
    print(f"  True failure  (nav fail + no AMCL):    {tf:3d} ({100*tf/max(total,1):.1f}%)")
    print(f"  AMCL-only OK  (nav fail + AMCL conv):  {ao:3d} ({100*ao/max(total,1):.1f}%)")
    if nav_ok > 0:
        print(f"\n  Of {nav_ok} Nav2 successes: {fs} ({100*fs/nav_ok:.1f}%) are false successes")

    mt_true = mean_nav_time(buckets["true_success"], episodes)
    mt_false = mean_nav_time(buckets["false_success"], episodes)
    if not (mt_true != mt_true) and not (mt_false != mt_false):
        print(f"  Mean nav_time true successes:  {mt_true:.1f}s")
        print(f"  Mean nav_time false successes: {mt_false:.1f}s  "
              f"({100*(mt_false-mt_true)/mt_true:+.1f}%)")

    # Per-level breakdown (if level info available)
    levels_present = {ep.get("level") for ep in episodes.values() if "level" in ep}
    if levels_present:
        print("\n  Per-level breakdown:")
        print(f"  {'Level':8s}  {'Total':>5s}  {'NavOK':>5s}  {'TrueS':>5s}  "
              f"{'FalseS':>6s}  {'FalseS/NavOK':>12s}")
        for level in LEVELS:
            level_eps = {g: e for g, e in episodes.items() if e.get("level") == level}
            lb = classify_episodes(level_eps)
            l_nav = len(lb["true_success"]) + len(lb["false_success"])
            l_fs = len(lb["false_success"])
            l_ts = len(lb["true_success"])
            l_tot = sum(len(v) for k, v in lb.items() if k != "unknown")
            ratio = f"{100*l_fs/l_nav:.0f}%" if l_nav > 0 else "—"
            print(f"  {level:8s}  {l_tot:5d}  {l_nav:5d}  {l_ts:5d}  {l_fs:6d}  {ratio:>12s}")
    print()


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(description="Quantify NavLearn false-success gap")
    parser.add_argument("--phase1-ttc", type=pathlib.Path,
                        default=pathlib.Path("results/phase1_ttc"),
                        help="Phase 1 TTC results directory")
    parser.add_argument("--phase2-ttc", type=pathlib.Path,
                        help="Phase 2 TTC results directory (optional)")
    parser.add_argument("--csv", type=pathlib.Path,
                        help="Flat localization_evaluation.csv (optional, overrides dirs)")
    return parser.parse_args()


def main() -> None:
    """Run false-success analysis."""
    args = parse_args()

    if args.csv and args.csv.exists():
        eps = load_episodes_from_csv(args.csv)
        buckets = classify_episodes(eps)
        report("localization_evaluation.csv", eps, buckets)
    else:
        if not args.phase1_ttc.exists():
            print(f"ERROR: {args.phase1_ttc} not found", file=sys.stderr)
            sys.exit(1)
        eps1 = load_episodes_from_dir(args.phase1_ttc)
        b1 = classify_episodes(eps1)
        report(f"Phase 1 TTC ({args.phase1_ttc})", eps1, b1)

        if args.phase2_ttc and args.phase2_ttc.exists():
            eps2 = load_episodes_from_dir(args.phase2_ttc)
            b2 = classify_episodes(eps2)
            report(f"Phase 2 TTC ({args.phase2_ttc})", eps2, b2)


if __name__ == "__main__":
    main()
