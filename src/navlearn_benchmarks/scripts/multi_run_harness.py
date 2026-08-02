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
import importlib.util
import json
import logging
import os
import pathlib
import resource
import signal
import subprocess
import sys
import time

DEFAULT_STACK_SPEC = os.path.join(
    os.path.expanduser("~"), ".navlearn", "current_stack_spec.json"
)


def _load_sibling(name: str):
    """Import a module from this scripts directory by path.

    The scripts directory is not an installed Python package -- the harness is executed as
    a file, not imported -- so a plain ``import`` would resolve against whatever happens to
    be on sys.path.
    """
    spec = importlib.util.spec_from_file_location(
        name, str(pathlib.Path(__file__).parent / f"{name}.py")
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


run_watchdog = _load_sibling("run_watchdog")
run_forensics = _load_sibling("run_forensics")


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


def _log_costmap_verdict(prefix: pathlib.Path) -> None:
    """Surface how far the global costmap drifted from the static map during this run."""
    path = pathlib.Path(f"{prefix}_costmap.json")
    if not path.is_file():
        logging.warning("No costmap record at %s", path)
        return
    try:
        rec = json.loads(path.read_text())
    except json.JSONDecodeError:
        logging.warning("Costmap record at %s is unreadable", path)
        return

    peak = rec.get("peak", {})
    logging.info(
        "Costmap : peak phantom %s cells (%s permanent), erased wall %s cells",
        peak.get("phantom_clearable"), peak.get("phantom_permanent"),
        peak.get("erased_wall"),
    )
    verdict = rec.get("verdict", "")
    if verdict.startswith("CORRUPTED"):
        logging.warning("COSTMAP: %s", verdict)


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


def _parse_magnitude_curve(spec: str) -> tuple:
    """Parse a MIN:MAX magnitude range, refusing anything a sweep cannot run.

    Validated here rather than left to the node so a mistyped range fails before a bringup
    is launched, instead of after the sim has spun up. An inverted or non-positive range
    is a configuration error, not a severity: a log sweep has no lower bound at zero, and
    silently clamping one would pile goals at the bottom of the range the sweep never
    asked for.
    """
    try:
        lo_text, hi_text = spec.split(":", 1)
        lo, hi = float(lo_text), float(hi_text)
    except ValueError:
        raise SystemExit(
            f"--magnitude-curve expects MIN:MAX in metres, e.g. 0.3:3.0; got {spec!r}")
    if lo <= 0.0:
        raise SystemExit(
            f"--magnitude-curve minimum must be positive, got {lo}; a zero displacement "
            "is not a perturbation and has no place on a log scale")
    if hi < lo:
        raise SystemExit(f"--magnitude-curve maximum {hi} is below its minimum {lo}")
    return lo, hi


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
        "--perturbation",
        choices=("clean", "ttc", "ttr"),
        required=True,
        help="Experimental condition. Passed explicitly to the launch so it appears in "
             "every run record; never defaulted, because a defaulted independent variable "
             "is an unlabelled experiment",
    )
    parser.add_argument(
        "--perturbation-level",
        choices=("easy", "medium", "hard", "extreme"),
        default="medium",
        help="Severity preset for ttc/ttr; ignored for clean and in curve mode",
    )
    parser.add_argument(
        "--magnitude-curve",
        metavar="MIN:MAX",
        help="Draw each goal's perturbation displacement from a continuous range instead "
             "of a categorical level, e.g. 0.3:3.0. Applies to the kidnap for --perturbation "
             "ttr and to the bad initialization for ttc. The magnitude comes from the "
             "campaign seed, so every goal in the sweep gets its own severity and the whole "
             "sweep still reproduces from one number. Overrides --perturbation-level",
    )
    parser.add_argument(
        "--magnitude-scale",
        choices=("log", "linear"),
        default="log",
        help="Spacing of the curve draws. Log by default: a displacement sweep spans an "
             "order of magnitude, and uniform-in-metres leaves the onset of degradation "
             "-- the part the curve exists to locate -- sparsely covered",
    )
    parser.add_argument(
        "--expected-scan-hz",
        type=float,
        default=10.0,
        help="LiDAR rate this cell is meant to test; the monitor flags under-delivery",
    )
    parser.add_argument(
        "--costmap-interval",
        type=float,
        default=2.0,
        help="Global-costmap vs static-map comparison interval in seconds",
    )
    parser.add_argument(
        "--no-costmap-monitor",
        action="store_true",
        help="Disable costmap corruption measurement (not advised for campaign runs)",
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
    # Watchdog. The slowest goal in the leg 1 campaign (n=360) took 205.8 s, median 30.5 s,
    # p99 176.3 s. Both budgets clear that by a wide margin on purpose: the slowest goals
    # are the hardest ones, and a watchdog that culls them would bias every rate in the
    # paper toward the conditions that happen to run fast. The budget exists to bound a
    # hang, not to police a slow cell.
    parser.add_argument(
        "--goal-timeout",
        type=float,
        default=600.0,
        help="Seconds without a completed goal before the run is declared hung. Measured "
             "from the previous goal's completion, not from the start of the run",
    )
    parser.add_argument(
        "--startup-timeout",
        type=float,
        default=900.0,
        help="Seconds allowed for node startup plus the first goal, which is not a goal "
             "cycle and must not be timed as one",
    )
    parser.add_argument(
        "--watchdog-interval",
        type=float,
        default=5.0,
        help="Seconds between watchdog polls",
    )
    parser.add_argument(
        "--no-forensics",
        dest="forensics",
        action="store_false",
        help="Disable per-run forensic capture: node logs into the run directory, crash "
             "reports harvested from the apport spool, stack state at the moment of an "
             "abort, and an environment record. On by default; a campaign that has to be "
             "re-run because the evidence was discarded costs more than the disk",
    )
    parser.add_argument(
        "--crash-dir",
        type=str,
        default=run_forensics.DEFAULT_CRASH_DIR,
        help="Apport spool to harvest crash reports from after each episode",
    )
    parser.add_argument(
        "--no-rosbag",
        dest="rosbag",
        action="store_false",
        help="Do not record a rosbag per episode. On by default: the bag is the only "
             "artefact that lets a finished run be re-examined for a question nobody "
             "thought to ask while it was running",
    )
    parser.add_argument(
        "--rosbag-compression",
        type=str,
        default=None,
        choices=("zstd",),
        help="Compress each bag file at close. Off by default; it costs CPU and wall clock "
             "between episodes, and disk is the resource this machine has most of",
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

    # A non-positive budget would read as "watchdog off", which is exactly the
    # configuration that cost 8 h 14 m on 2026-08-01. There is no supported way to
    # disable it; a cell that needs longer says so with a larger number.
    for flag, value in (("--goal-timeout", args.goal_timeout),
                        ("--startup-timeout", args.startup_timeout),
                        ("--watchdog-interval", args.watchdog_interval)):
        if value <= 0:
            parser.error(
                f"{flag} must be positive, got {value}. The watchdog cannot be disabled: "
                "an unsupervised run that hangs produces no event for anything to notice, "
                "which is how a 24-episode cell became an 8-hour wait."
            )

    return args


def launch_supervised(cmd, log_handle, watchdog, poll_interval_s=5.0, grace_s=20.0,
                      env=None, on_abort=None):
    """Run one launch under the watchdog.

    Returns ``(returncode, abort)``. ``abort`` is ``None`` when the launch ended on its own,
    in which case the return code is the launch's and the behaviour is identical to the
    ``subprocess.run`` call this replaced.

    ``start_new_session`` puts the launch in its own process group. That is what makes the
    abort safe to escalate: the watchdog signals the group, so it reaches the nodes
    ``ros2 launch`` spawned, and stops short of the harness that owns the campaign.
    """
    process = subprocess.Popen(
        cmd, stdout=log_handle, stderr=subprocess.STDOUT, start_new_session=True, env=env,
    )
    abort = run_watchdog.supervise(
        process, watchdog, poll_interval_s=poll_interval_s, grace_s=grace_s,
        on_abort=on_abort,
    )
    if abort is not None and process.returncode == 0:
        # A launch killed by signal can still report 0 if it exited between the abort
        # decision and the signal. Reporting success for a cell we chose to kill would put
        # a truncated run into the campaign as a complete one.
        return abort.code, abort
    return process.returncode, abort


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
        # The experimental condition, always stated explicitly rather than left to launch
        # defaults. It therefore appears verbatim in launch_command inside every run
        # record, so a cell's directory name can be checked against what actually ran.
        # kidnap_enabled defaulted to true, which silently turned a 'clean' pilot into a
        # TTR cell; nothing in the record showed it.
        f"bad_init_test:={'true' if args.perturbation == 'ttc' else 'false'}",
        f"kidnap_enabled:={'true' if args.perturbation == 'ttr' else 'false'}",
        f"perturbation_level:={args.perturbation_level}",
    ]

    # Continuous severity, stated explicitly for the same reason as the condition above:
    # it is the cell's independent variable and must appear verbatim in the run record.
    #
    # Routed to whichever perturbation the cell actually runs. Setting both sweeps from
    # one flag would make a ttc cell carry a kidnap range it never uses, which reads in
    # the record as a condition that was configured and is a lie about what ran.
    if args.magnitude_curve:
        lo, hi = _parse_magnitude_curve(args.magnitude_curve)
        prefix = "kidnap" if args.perturbation == "ttr" else "bad_init"
        cmd += [
            f"{prefix}_magnitude_mode:=curve",
            f"{prefix}_magnitude_min_m:={lo}",
            f"{prefix}_magnitude_max_m:={hi}",
            f"{prefix}_magnitude_scale:={args.magnitude_scale}",
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
        "perturbation": args.perturbation,
        "perturbation_level": args.perturbation_level,
        "magnitude_curve": args.magnitude_curve,
        "magnitude_scale": args.magnitude_scale if args.magnitude_curve else None,
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
    # (process, stop signal). The signal is per-monitor because rosbag2 finalises its
    # metadata only on SIGINT; the samplers write their records on SIGTERM.
    monitors = []

    if args.compute_interval > 0:
        monitors.append((subprocess.Popen([
            sys.executable, str(here / "compute_sampler.py"),
            "--output-prefix", str(prefix).format("compute"),
            "--interval", str(args.compute_interval),
        ]), signal.SIGTERM))

    if not args.no_costmap_monitor:
        monitors.append((subprocess.Popen([
            sys.executable, str(here / "costmap_corruption_monitor.py"),
            "--output-prefix", str(prefix).format("costmap"),
            "--interval", str(args.costmap_interval),
        ]), signal.SIGTERM))

    if not args.no_rate_monitor:
        monitors.append((subprocess.Popen([
            sys.executable, str(here / "rate_monitor.py"),
            "--output-prefix", str(prefix).format("rates"),
            "--expected-scan-hz", str(args.expected_scan_hz),
        ]), signal.SIGTERM))

    # The bag. Started before the launch so the first messages of the episode -- the goal
    # that is about to be sent, the TF tree settling -- are inside it rather than lost to
    # recorder startup. Its own stdout goes to a file so a recorder that fails says why.
    if args.rosbag:
        bag_path = run_forensics.rosbag_dir(report_dir, episode_id, stamp)
        bag_log = pathlib.Path(str(prefix).format("rosbag")).with_suffix(".log")
        recorder = run_forensics.start_recorder(
            bag_path,
            compression=args.rosbag_compression,
            log_handle=open(bag_log, "w"),
        )
        if recorder is not None:
            monitors.append((recorder, signal.SIGINT))
            logging.info("BAG  --> %s", bag_path)
        else:
            logging.error("No rosbag for run %d: the recorder would not start.", episode_id)

    # Node stdout and stderr go to the run directory, not to the terminal and not to
    # /tmp. They are the only record of why a cell failed, and /tmp does not survive a
    # reboot — which is exactly when an overnight campaign's failures get investigated.
    log_path = pathlib.Path(str(prefix).format("nodes")).with_suffix(".log")
    logging.info("LOG  --> %s", log_path)

    # The supervisor. Progress is read from the metrics CSV this run is writing; liveness
    # from the nav2 server processes plus the bringup that owns them. Both are external to
    # the launch on purpose -- a launch that has stopped working is exactly the thing that
    # cannot be asked whether it is still working.
    watchdog = run_watchdog.RunWatchdog(
        progress=lambda: run_watchdog.completed_goals(csv_path),
        liveness=lambda: run_watchdog.dead_processes(
            run_watchdog.REQUIRED_STACK_PROCESSES,
            launch_pid=stack_spec.get("launch_pid"),
        ),
        goal_timeout_s=args.goal_timeout,
        startup_timeout_s=args.startup_timeout,
        goals_total=args.goals,
    )

    # Every node's rcl log goes into this episode's own directory rather than the shared
    # ~/.ros/log, so a log found later is attributable to a cell by where it is rather than
    # by matching timestamps. episode_start bounds the crash harvest below: reports older
    # than it belong to an earlier run and are not evidence about this one.
    episode_start = _time.time()
    episode_env = None
    on_abort = None
    forensic_dir = report_dir / f"forensics_run_{episode_id}_{stamp}"
    if args.forensics:
        episode_env = run_forensics.episode_environment(
            run_forensics.run_log_dir(report_dir, episode_id, stamp)
        )

        def on_abort(verdict):
            """Photograph the stack while it is still standing.

            Crash reports first: they are file operations that cannot block, and a core
            dump is the artefact that actually solved the last failure. The stack probes
            come second because they are the ones that can wait out a discovery timeout.
            """
            crashes = run_forensics.harvest_crash_reports(
                forensic_dir, since=episode_start, crash_dir=args.crash_dir,
            )
            for crash in crashes:
                logging.error("Forensics: crash report -> %s", crash)
            state = run_forensics.capture_stack_state(forensic_dir, reason=verdict.reason)
            logging.error("Forensics: stack state -> %s", state)

    try:
        with open(log_path, "w") as log_handle:
            log_handle.write(f"# {' '.join(cmd)}\n")
            log_handle.flush()
            returncode, abort = launch_supervised(
                cmd, log_handle, watchdog, poll_interval_s=args.watchdog_interval,
                env=episode_env, on_abort=on_abort,
            )
    finally:
        # Stop instrumentation on every path, including a failed or interrupted run. The
        # signal is each monitor's cue to write its record, so they must be given the chance
        # to exit cleanly or the run's traces are lost. The recorder gets SIGINT, which is
        # the only path on which rosbag2 writes metadata.yaml; without it the bag exists on
        # disk and no reader will open it.
        for monitor, sig in monitors:
            run_forensics.stop_process(monitor, sig=sig, grace_s=30.0)

    # After every episode, not only aborted ones. On 2026-08-01 planner_server segfaulted
    # during run 4's final goal; all five rows were already written, so the harness called
    # that run successful and moved on. The crash was the cause of everything that followed
    # and nothing in the run's own directory recorded it.
    if args.forensics:
        crashes = run_forensics.harvest_crash_reports(
            forensic_dir, since=episode_start, crash_dir=args.crash_dir,
        )
        for crash in crashes:
            logging.error(
                "CRASH: a process died during run %d and left %s. This run's data is "
                "suspect even if its rows are complete.", episode_id, crash,
            )

    if abort is not None:
        # Named, non-zero, and immediate. The failure this replaces was silent: the launch
        # stayed up with a dead stack behind it and the harness waited on it for hours.
        logging.error(
            "Run %d/%d ABORTED BY WATCHDOG after %d/%d goals: %s",
            episode_id, args.episodes, watchdog.completed, args.goals, abort.reason,
        )
        logging.error("Node output: %s", log_path)
        logging.error(
            "The cell is incomplete and its runs must not be analysed as a finished cell."
        )
        sys.exit(abort.code)

    if returncode != 0:
        logging.error(
            "Run %d/%d failed with exit code %d. Node output: %s",
            episode_id, args.episodes, returncode, log_path,
        )
        sys.exit(returncode)

    _log_compute_verdict(pathlib.Path(str(prefix).format("compute")))
    _log_rate_verdict(pathlib.Path(str(prefix).format("rates")))
    _log_costmap_verdict(pathlib.Path(str(prefix).format("costmap")))
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
        if args.forensics:
            env_path = run_forensics.write_environment(report_dir)
            logging.info("Environment: %s", env_path)
            if resource.getrlimit(resource.RLIMIT_CORE)[0] == 0:
                logging.warning(
                    "CORE LIMIT IS 0: a segfaulting node will leave no core dump. The "
                    "2026-08-01 planner_server crash was only diagnosable because one "
                    "was written. Run the campaign from a shell with 'ulimit -c unlimited'."
                )

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
