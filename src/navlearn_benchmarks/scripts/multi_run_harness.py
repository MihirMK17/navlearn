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
Multi-run benchmark harness for NavLearn.

Runs N episodes of the NavLearn benchmarking pipeline, each as a separate
ros2 launch invocation. Each run gets a unique seed derived from BASE_SEED.

Usage:
    python3 multi_run_harness.py --episodes 10 --goals 5 --seed 42 \
        --output-dir /path/to/runs/baseline --profile baseline

    # Dry-run (print commands without executing):
    python3 multi_run_harness.py --episodes 2 --goals 3 --seed 42 \
        --output-dir /tmp/test --profile aggressive --dry-run
"""

import argparse
import logging
import pathlib
import subprocess
import sys
import time


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="NavLearn multi-run benchmark harness",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--episodes", type=int, default=10, help="Number of benchmark runs to execute"
    )
    parser.add_argument("--goals", type=int, default=5, help="Number of goals per run")
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Base RNG seed; each run gets seed + run_index",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        required=True,
        help="Directory to write CSV/JSON output files",
    )
    parser.add_argument(
        "--profile",
        type=str,
        required=True,
        choices=["baseline", "aggressive"],
        help="Nav2 parameter profile to benchmark",
    )
    parser.add_argument(
        "--goal-source",
        type=str,
        default="map_random",
        help="Goal source: map_random | fixed | stress",
    )
    parser.add_argument(
        "--sleep",
        type=float,
        default=5.0,
        help="Seconds to sleep between runs (allows ROS cleanup)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print launch commands without executing them",
    )
    parser.add_argument(
        "--extra-arg",
        action="append",
        default=[],
        help="Additional launch arg passed to benchmarks.launch.py (repeatable), "
             "e.g. --extra-arg bad_init_test:=true --extra-arg perturbation_level:=easy",
    )
    return parser.parse_args()


def run_benchmark(
    episode_id: int,
    args: argparse.Namespace,
    report_dir: pathlib.Path,
) -> None:
    """Launch one benchmark run and exit with non-zero status on failure."""
    import time as _time

    stamp = _time.strftime("%Y%m%d_%H%M%S")
    seed = args.seed + (episode_id - 1)

    csv_path = report_dir / f"navlearn_metrics_run_{episode_id}_{stamp}.csv"
    json_path = report_dir / f"navlearn_run_report_run_{episode_id}_{stamp}.json"

    logging.info("=== RUN %d / %d ===", episode_id, args.episodes)
    logging.info("Profile  : %s", args.profile)
    logging.info("Seed     : %d", seed)
    logging.info("CSV  --> %s", csv_path)
    logging.info("JSON --> %s", json_path)

    cmd = [
        "ros2",
        "launch",
        "navlearn_benchmarks",
        "benchmarks.launch.py",
        f"goals_num:={args.goals}",
        f"goal_source:={args.goal_source}",
        f"goal_seed:={seed}",
        f"nav2_profile:={args.profile}",
        f"csv_path:={csv_path}",
        f"json_path:={json_path}",
    ]
    cmd.extend(args.extra_arg)

    if args.dry_run:
        logging.info("[DRY RUN] Would execute: %s", " ".join(cmd))
        return

    result = subprocess.run(cmd)

    if result.returncode != 0:
        logging.error(
            "Run %d/%d failed with exit code %d. Aborting harness.",
            episode_id,
            args.episodes,
            result.returncode,
        )
        sys.exit(result.returncode)

    logging.info("Run %d/%d completed successfully.", episode_id, args.episodes)


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%H:%M:%S",
    )

    args = parse_args()
    report_dir = pathlib.Path(args.output_dir)

    if not args.dry_run:
        report_dir.mkdir(parents=True, exist_ok=True)
        logging.info("Output directory: %s", report_dir)

    logging.info(
        "Starting harness: %d episodes × %d goals, seed=%d, profile=%s",
        args.episodes,
        args.goals,
        args.seed,
        args.profile,
    )

    for i in range(args.episodes):
        run_benchmark(i + 1, args, report_dir)
        if i < args.episodes - 1:
            if not args.dry_run:
                logging.info("Sleeping %.1f s before next run...", args.sleep)
                time.sleep(args.sleep)

    logging.info("Harness complete: %d/%d runs finished.", args.episodes, args.episodes)


if __name__ == "__main__":
    main()
