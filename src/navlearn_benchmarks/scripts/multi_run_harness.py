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
ros2 launch invocation.

Randomness
    --seed sets the campaign seed, held constant across every episode and every
    controller. Each episode is distinguished by its run_index, and every draw (goal
    placement, TTC displacement, TTR target, kidnap delay) is derived from
    (campaign_seed, run_index, goal_index) through splitmix64. The controller is
    deliberately not an input, so all arms face identical conditions and the paired
    statistics the analysis plan calls for remain valid.

Stack provenance
    This harness does not choose the navigation stack and does not claim to know it.
    The stack is selected when the bringup is launched, and ``navigation.launch.py``
    records what it actually composed to a JSON spec. The harness reads that spec,
    verifies the process that wrote it is still alive, and copies it into every run
    directory alongside the metrics.

    This replaces the retired ``--profile`` flag, which named a profile that nothing
    verified: it was forwarded to ``benchmarks.launch.py``, which declared the argument
    and never consumed it. A run could therefore be labelled ``mppi_baseline`` while the
    bringup in the other terminal was running something else entirely, with no way to
    tell afterwards. Recorded configuration and actual configuration are now the same
    object.

Usage:
    # Terminal 1 - bringup selects the stack
    ros2 launch bumperbot_bringup simulated_robot.launch.py \
        world_name:=small_house controller:=mppi planner:=smac2d localizer:=amcl_tuned

    # Terminal 2 - harness reads what terminal 1 composed
    python3 multi_run_harness.py --episodes 10 --goals 5 --seed 42 \
        --output-dir /path/to/runs/mppi_clean

    # Dry-run (print commands without executing; skips the liveness check):
    python3 multi_run_harness.py --episodes 2 --goals 3 --seed 42 \
        --output-dir /tmp/test --dry-run
"""

import argparse
import json
import logging
import os
import pathlib
import subprocess
import sys
import time

DEFAULT_STACK_SPEC = os.path.join(
    os.path.expanduser("~"), ".navlearn", "current_stack_spec.json"
)


def read_stack_spec(path: str, require_live: bool = True) -> dict:
    """Load the provenance record written by navigation.launch.py.

    Args:
        path: location of the JSON spec.
        require_live: verify the launch process that wrote the spec is still running.
            A spec left behind by a previous bringup would otherwise silently label this
            campaign with the wrong stack.

    Returns:
        The parsed spec.

    Raises:
        SystemExit: if the spec is missing, malformed, or stale. Every one of these
            means the run's configuration cannot be established, and a benchmark result
            whose configuration is unknown is not worth the machine time to produce.
    """
    spec_path = pathlib.Path(path)
    if not spec_path.is_file():
        sys.exit(
            f"FATAL: no stack spec at {spec_path}.\n"
            "The bringup writes it when nav2 launches. Either the bringup is not running, "
            "or it was started with stack_spec_out set to an empty value.\n"
            "Start it with, for example:\n"
            "  ros2 launch bumperbot_bringup simulated_robot.launch.py "
            "world_name:=small_house controller:=rpp planner:=smac2d localizer:=amcl_tuned"
        )

    try:
        spec = json.loads(spec_path.read_text())
    except json.JSONDecodeError as exc:
        sys.exit(f"FATAL: stack spec at {spec_path} is not valid JSON: {exc}")

    for key in ("schema", "selection", "identity", "fragments"):
        if key not in spec:
            sys.exit(f"FATAL: stack spec at {spec_path} has no '{key}' field")

    if require_live:
        pid = spec.get("launch_pid")
        if pid is None:
            sys.exit(f"FATAL: stack spec at {spec_path} has no launch_pid to verify")
        try:
            os.kill(int(pid), 0)
        except (OSError, ValueError):
            sys.exit(
                f"FATAL: stack spec at {spec_path} was written by PID {pid}, which is no "
                "longer running.\n"
                "This spec is left over from an earlier bringup and does not describe the "
                "stack now running. Restart the bringup, or point --stack-spec at the "
                "correct file."
            )

    return spec


def _log_compute_verdict(prefix: pathlib.Path) -> None:
    """Surface the run's ROBOT-class compute cost and any thermal throttling.

    Throttling is reported at the console as it happens rather than discovered during
    analysis, because a throttled cell may need re-running and that is far cheaper to
    notice on the night it occurs than after the campaign has finished.
    """
    summary_path = pathlib.Path(f"{prefix}_summary.json")
    if not summary_path.is_file():
        logging.warning("No compute summary at %s", summary_path)
        return

    try:
        summary = json.loads(summary_path.read_text())
    except json.JSONDecodeError:
        logging.warning("Compute summary at %s is unreadable", summary_path)
        return

    robot = summary.get("by_class", {}).get("ROBOT")
    if robot:
        logging.info(
            "Compute  : ROBOT mean %.1f%% of a core, peak %.1f%%, %.1f CPU-s, peak PSS %s kB",
            robot.get("cpu_percent_of_core_mean") or 0.0,
            robot.get("cpu_percent_of_core_peak") or 0.0,
            robot.get("cpu_seconds_total") or 0.0,
            robot.get("peak_pss_kb"),
        )

    thermal = summary.get("thermal", {})
    if thermal.get("throttled"):
        logging.warning("THERMAL: %s", thermal.get("verdict"))
    else:
        logging.info(
            "Thermal  : clean, %.0f MHz mean, %.0f C max",
            thermal.get("cpu_freq_mhz_mean") or 0.0,
            thermal.get("package_temp_c_max") or 0.0,
        )


def _log_rate_verdict(prefix: pathlib.Path) -> None:
    """Surface the run's achieved rates and real-time factor.

    A scan rate below the requested one means the cell did not test the condition its
    label claims, which matters most for the sensor-starve leg where the rate *is* the
    independent variable.
    """
    path = pathlib.Path(f"{prefix}_rates.json")
    if not path.is_file():
        logging.warning("No rate record at %s", path)
        return
    try:
        rates = json.loads(path.read_text())
    except json.JSONDecodeError:
        logging.warning("Rate record at %s is unreadable", path)
        return

    logging.info(
        "Rates    : RTF=%s scan=%s Hz cmd_vel=%s Hz",
        rates.get("real_time_factor"),
        rates.get("scan", {}).get("rate_hz_median"),
        rates.get("cmd_vel", {}).get("rate_hz_median"),
    )
    if rates.get("scan_rate_met") is False:
        logging.error("RATE: %s", rates.get("verdict"))


def validate_run_output(
    episode_id: int, args: argparse.Namespace, csv_path: pathlib.Path,
    log_path: pathlib.Path,
) -> None:
    """Halt the campaign if a run produced fewer goal rows than it was asked for.

    The retired orchestrators handled a short cell by re-running it and topping up the
    directory until the expected row count appeared. That converts a systematic failure —
    a controller that cannot complete under this perturbation, a sim that dies partway —
    into a data set that looks complete, with the surviving rows biased toward whatever
    conditions happened to succeed.

    A row is written for every terminated goal regardless of outcome, so a FAILED goal
    still produces a row. Fewer rows than goals therefore means the episode did not finish,
    which is a defect rather than a result, and the campaign stops so it can be diagnosed
    while the evidence is fresh.
    """
    if not csv_path.is_file():
        logging.error(
            "Run %d produced no CSV at %s. The metrics pipeline did not complete. "
            "Node output: %s", episode_id, csv_path, log_path,
        )
        sys.exit(2)

    with open(csv_path) as handle:
        rows = [line for line in handle.read().splitlines() if line.strip()]
    data_rows = max(len(rows) - 1, 0)  # minus header

    if data_rows < args.goals:
        logging.error(
            "Run %d wrote %d goal rows but %d were requested. The episode did not "
            "complete. Not re-running to top up the count: that would hide a systematic "
            "failure behind a directory that looks full. Node output: %s",
            episode_id, data_rows, args.goals, log_path,
        )
        sys.exit(3)

    logging.info("Validate : %d/%d goal rows present", data_rows, args.goals)


def git_sha(workspace: pathlib.Path) -> str:
    """Return the short git SHA of the workspace, or 'unknown' outside a repository."""
    try:
        result = subprocess.run(
            ["git", "-C", str(workspace), "rev-parse", "--short", "HEAD"],
            capture_output=True,
            text=True,
            timeout=10,
        )
    except (OSError, subprocess.SubprocessError):
        return "unknown"
    return result.stdout.strip() if result.returncode == 0 else "unknown"


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
        "--stack-spec",
        type=str,
        default=DEFAULT_STACK_SPEC,
        help="Provenance record written by the bringup; copied into every run directory",
    )
    parser.add_argument(
        "--compute-interval",
        type=float,
        default=0.2,
        help="Compute sampler interval in seconds; 0 disables compute profiling",
    )
    parser.add_argument(
        "--expected-scan-hz",
        type=float,
        default=10.0,
        help="LiDAR rate this cell is meant to test; the monitor flags under-delivery",
    )
    parser.add_argument(
        "--no-rate-monitor",
        action="store_true",
        help="Disable RTF and topic-rate measurement (not advised for campaign runs)",
    )
    parser.add_argument(
        "--profile",
        type=str,
        default=None,
        help=argparse.SUPPRESS,  # Retired. Accepted only to fail with an explanation.
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
    args = parser.parse_args()

    if args.profile is not None:
        parser.error(
            "--profile has been retired. It named a Nav2 profile that nothing verified: "
            "the argument was forwarded to benchmarks.launch.py, which declared it and "
            "never consumed it, so the recorded profile and the running stack could "
            "differ with no way to detect it.\n\n"
            "The stack is now selected on the bringup and recorded automatically:\n"
            "  ros2 launch bumperbot_bringup simulated_robot.launch.py \\\n"
            "      world_name:=small_house controller:=mppi planner:=smac2d "
            "localizer:=amcl_tuned\n\n"
            "Then run this harness with no stack argument at all."
        )

    return args


def run_benchmark(
    episode_id: int,
    args: argparse.Namespace,
    report_dir: pathlib.Path,
    stack_spec: dict,
) -> None:
    """Launch one benchmark run and exit with non-zero status on failure."""
    import time as _time

    stamp = _time.strftime("%Y%m%d_%H%M%S")
    # The campaign seed is constant across every episode and every controller; the run
    # index is what makes episodes differ. Previously the harness folded the episode into
    # the seed itself, which varied goal placement but never reached the perturbation
    # seeds — those were fixed constants in a YAML default, so all 25 episodes of a cell
    # drew the same five displacements.
    run_index = episode_id - 1

    csv_path = report_dir / f"navlearn_metrics_run_{episode_id}_{stamp}.csv"
    json_path = report_dir / f"navlearn_run_report_run_{episode_id}_{stamp}.json"
    spec_path = report_dir / f"navlearn_stack_spec_run_{episode_id}_{stamp}.json"

    logging.info("=== RUN %d / %d ===", episode_id, args.episodes)
    logging.info("Stack    : %s", stack_spec["selection"])
    logging.info("Seed     : campaign=%d run_index=%d", args.seed, run_index)
    logging.info("CSV  --> %s", csv_path)
    logging.info("JSON --> %s", json_path)
    logging.info("SPEC --> %s", spec_path)

    cmd = [
        "ros2",
        "launch",
        "navlearn_benchmarks",
        "benchmarks.launch.py",
        f"goals_num:={args.goals}",
        f"goal_source:={args.goal_source}",
        f"campaign_seed:={args.seed}",
        f"run_index:={run_index}",
        f"csv_path:={csv_path}",
        f"json_path:={json_path}",
        # Forwarded so control_metric resolves its velocity ceiling from the same spec the
        # harness validated, rather than from whatever happens to be at the default path.
        f"stack_spec:={args.stack_spec}",
    ]
    cmd.extend(args.extra_arg)

    if args.dry_run:
        logging.info("[DRY RUN] Would execute: %s", " ".join(cmd))
        return

    # Written before the run, not after: if the run crashes, the record of what was being
    # attempted must still exist. It is what makes an aborted cell diagnosable.
    run_record = dict(stack_spec)
    run_record["run"] = {
        "episode": episode_id,
        "episodes_total": args.episodes,
        "campaign_seed": args.seed,
        "run_index": run_index,
        "goals": args.goals,
        "goal_source": args.goal_source,
        "extra_args": list(args.extra_arg),
        "csv_path": str(csv_path),
        "json_path": str(json_path),
        "captured_at": _time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "workspace_git_sha": args.git_sha,
        "launch_command": cmd,
    }
    spec_path.write_text(json.dumps(run_record, indent=2, sort_keys=True) + "\n")

    # Instrumentation spans exactly the launch, so its samples cover the navigation work
    # and nothing else. Started here rather than inside the launch file so that a crashed
    # or hung ROS graph still leaves usable traces behind.
    prefix = report_dir / f"navlearn_{{}}_run_{episode_id}_{stamp}"
    here = pathlib.Path(__file__).parent
    monitors = []

    if args.compute_interval > 0:
        monitors.append(subprocess.Popen([
            sys.executable, str(here / "compute_sampler.py"),
            "--output-prefix", str(prefix).format("compute"),
            "--interval", str(args.compute_interval),
        ]))

    if not args.no_rate_monitor:
        monitors.append(subprocess.Popen([
            sys.executable, str(here / "rate_monitor.py"),
            "--output-prefix", str(prefix).format("rates"),
            "--expected-scan-hz", str(args.expected_scan_hz),
        ]))

    # Node stdout and stderr go to the run directory, not to the terminal and not to
    # /tmp. They are the only record of why a cell failed, and /tmp does not survive a
    # reboot — which is exactly when an overnight campaign's failures get investigated.
    log_path = pathlib.Path(str(prefix).format("nodes")).with_suffix(".log")
    logging.info("LOG  --> %s", log_path)

    try:
        with open(log_path, "w") as log_handle:
            log_handle.write(f"# {' '.join(cmd)}\n")
            log_handle.flush()
            result = subprocess.run(cmd, stdout=log_handle, stderr=subprocess.STDOUT)
    finally:
        # Stop instrumentation on every path, including a failed or interrupted run.
        # SIGTERM is each monitor's cue to write its record, so they must be given the
        # chance to exit cleanly or the run's traces are lost.
        for monitor in monitors:
            if monitor.poll() is None:
                monitor.terminate()
                try:
                    monitor.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    logging.warning("Monitor pid %d ignored SIGTERM; killing.", monitor.pid)
                    monitor.kill()
                    monitor.wait(timeout=5)

    if result.returncode != 0:
        logging.error(
            "Run %d/%d failed with exit code %d. Node output: %s",
            episode_id, args.episodes, result.returncode, log_path,
        )
        sys.exit(result.returncode)

    _log_compute_verdict(pathlib.Path(str(prefix).format("compute")))
    _log_rate_verdict(pathlib.Path(str(prefix).format("rates")))
    validate_run_output(episode_id, args, csv_path, log_path)
    logging.info("Run %d/%d completed successfully.", episode_id, args.episodes)


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%H:%M:%S",
    )

    args = parse_args()
    report_dir = pathlib.Path(args.output_dir)

    # Establish what is actually running before spending machine time on it. A stale or
    # missing spec aborts here rather than producing a directory of unattributable CSVs.
    stack_spec = read_stack_spec(args.stack_spec, require_live=not args.dry_run)
    args.git_sha = git_sha(pathlib.Path(__file__).resolve().parents[3])

    if not args.dry_run:
        report_dir.mkdir(parents=True, exist_ok=True)
        logging.info("Output directory: %s", report_dir)

    logging.info(
        "Starting harness: %d episodes × %d goals, seed=%d",
        args.episodes,
        args.goals,
        args.seed,
    )
    logging.info("Stack spec  : %s", args.stack_spec)
    logging.info("Selection   : %s", stack_spec["selection"])
    for key, value in stack_spec["identity"].items():
        logging.info("  %-34s %s", key, value)
    logging.info("Workspace   : %s", args.git_sha)

    for i in range(args.episodes):
        run_benchmark(i + 1, args, report_dir, stack_spec)
        if i < args.episodes - 1:
            if not args.dry_run:
                logging.info("Sleeping %.1f s before next run...", args.sleep)
                time.sleep(args.sleep)

    logging.info("Harness complete: %d/%d runs finished.", args.episodes, args.episodes)


if __name__ == "__main__":
    main()
