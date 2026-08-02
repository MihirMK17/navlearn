#!/usr/bin/env python3
# Copyright 2026 Mihir Kulkarni
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
"""Per-run forensic capture: make the run directory the whole story.

Why it exists
    The 2026-08-01 planner_server segfault was solved from a core dump nobody had arranged
    to keep. Apport caught it by default and it survived only because no second planner
    crash overwrote it -- apport stores one report per executable path and skips writing
    when one is already present. A bt_navigator crash from the previous night was found in
    the same directory by accident, three days late, with nothing tying it to the run that
    produced it.

    Everything that ran during an episode now lands beside that episode's metrics:

      ros_logs/run_N_<stamp>/   every node's rcl log, via a per-episode ROS_LOG_DIR
      crash/                    apport reports produced during the run, moved out of
                                /var/crash so the next crash has somewhere to land
      stack_state_*.txt         the process table and lifecycle state at the instant the
                                watchdog fired, captured before the kill destroys it
      environment.json          kernel, ROS distro, nav2 package versions, limits

    Recording too much costs disk. Recording too little costs a re-run of the campaign.
"""

import json
import os
import pathlib
import platform
import resource
import shutil
import subprocess
import time

DEFAULT_CRASH_DIR = "/var/crash"

#: Packages whose versions decide whether a result reproduces. Kept short deliberately:
#: this is the stack under test, not an inventory of the machine.
NAV2_PACKAGES = (
    "ros-humble-nav2-planner",
    "ros-humble-nav2-smac-planner",
    "ros-humble-nav2-controller",
    "ros-humble-nav2-costmap-2d",
    "ros-humble-nav2-bt-navigator",
    "ros-humble-nav2-amcl",
    "ros-humble-nav2-regulated-pure-pursuit-controller",
    "ros-humble-nav2-mppi-controller",
    "ros-humble-nav2-dwb-controller",
)


def run_log_dir(report_dir, episode_id: int, stamp: str) -> pathlib.Path:
    """Return (and create) the per-episode ROS log directory inside the run directory."""
    target = pathlib.Path(report_dir) / "ros_logs" / f"run_{episode_id}_{stamp}"
    target.mkdir(parents=True, exist_ok=True)
    return target


def episode_environment(ros_log_dir, base_env=None) -> dict:
    """Return an environment that sends this episode's node logs into the run directory.

    ``~/.ros/log`` is shared by every run on the machine, so a log found there cannot be
    attributed to a cell without parsing timestamps out of the launch output. Pointing
    ROS_LOG_DIR at the run directory makes the attribution structural.
    """
    target = pathlib.Path(ros_log_dir)
    target.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ if base_env is None else base_env)
    env["ROS_LOG_DIR"] = str(target)
    return env


def harvest_crash_reports(dest, since: float, crash_dir=DEFAULT_CRASH_DIR) -> list:
    """Move apport reports produced since ``since`` into ``dest``.

    Moved rather than copied, because apport writes at most one report per executable path
    and skips the write entirely when a report is already there. Leaving the first report
    in place silently discards the second crash of the same node -- which is the one that
    says whether the fault is deterministic.

    The original is removed only after the copy is known to have succeeded. Deleting
    evidence while failing to save it would destroy the only artefact of the failure being
    investigated.
    """
    source_dir = pathlib.Path(crash_dir)
    destination = pathlib.Path(dest)
    if not source_dir.is_dir():
        return []

    taken = []
    for report in sorted(source_dir.glob("*.crash")):
        try:
            if report.stat().st_mtime < since:
                continue
        except OSError:
            continue

        target = destination / report.name
        try:
            destination.mkdir(parents=True, exist_ok=True)
            shutil.copy2(report, target)
        except OSError:
            continue  # original deliberately left in place

        try:
            report.unlink()
        except OSError:
            pass  # copied safely; a read-only crash dir is not a failure
        taken.append(target)

    return taken


def _command(argv, timeout=10) -> str:
    """Run a probe, returning its output or a note about why there is none."""
    try:
        result = subprocess.run(argv, capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.SubprocessError) as exc:
        return f"<{' '.join(argv)} unavailable: {exc}>"
    return (result.stdout or "") + (result.stderr or "")


def environment_snapshot() -> dict:
    """Record what the stack under test was built from.

    "It worked last month" is unanswerable without the versions it worked with, and the
    core limit belongs here because a campaign that segfaults without producing a core is
    a campaign that has to be run twice.
    """
    versions = {}
    for package in NAV2_PACKAGES:
        out = _command(["dpkg-query", "-W", "-f=${Version}", package], timeout=10).strip()
        if out and not out.startswith("<") and "no packages found" not in out:
            versions[package] = out

    soft, hard = resource.getrlimit(resource.RLIMIT_CORE)
    return {
        "captured_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "kernel": platform.release(),
        "platform": platform.platform(),
        "python": platform.python_version(),
        "ros_distro": os.environ.get("ROS_DISTRO", "<unset>"),
        "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION", "<default>"),
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "<default>"),
        "nav2_versions": versions,
        "cpu_count": os.cpu_count(),
        "core_limit": {"soft": soft, "hard": hard},
        "core_pattern": _read("/proc/sys/kernel/core_pattern"),
    }


def _read(path) -> str:
    try:
        return pathlib.Path(path).read_text().strip()
    except OSError:
        return "<unreadable>"


#: Seconds capture_stack_state may spend on ROS probes. The launch is still alive while it
#: runs, so this is time the campaign stays stuck. Querying a stack that has just died is
#: the case where every probe waits out its full discovery timeout: six in series held an
#: abort for 69 s in testing.
DEFAULT_FORENSIC_BUDGET_S = 25.0


def capture_stack_state(dest, reason: str, budget_s=DEFAULT_FORENSIC_BUDGET_S
                        ) -> pathlib.Path:
    """Write what every process was doing at the moment the run was declared failed.

    Called before the abort kills the launch. Teardown is what makes a hang unreproducible:
    once the group is signalled, the state that would have explained it is gone.

    Every probe is best-effort and its failure is recorded rather than raised -- this runs
    on the failure path, and a diagnostic that can itself fail is a second outage.
    """
    destination = pathlib.Path(dest)
    destination.mkdir(parents=True, exist_ok=True)
    path = destination / f"stack_state_{time.strftime('%Y%m%d_%H%M%S')}.txt"

    deadline = time.monotonic() + float(budget_s)

    def within_budget(label):
        """Expensive probes run only while there is budget; a skip is recorded, not hidden."""
        if time.monotonic() >= deadline:
            return f"<skipped: forensic budget of {budget_s:.0f} s exhausted>"
        return None

    # Cheap reads first and unconditionally. They are the ones that always survive, and on
    # a machine that is thrashing they are also the ones that explain it.
    sections = [
        ("reason", reason),
        ("captured_at", time.strftime("%Y-%m-%dT%H:%M:%S%z")),
        ("loadavg", _read("/proc/loadavg")),
        ("meminfo", "\n".join(_read("/proc/meminfo").splitlines()[:8])),
        ("processes", _command(
            ["ps", "-eo", "pid,ppid,stat,pcpu,pmem,rss,etime,comm,args", "--sort=-pcpu"],
            timeout=10)),
    ]

    skipped = within_budget("ros2 node list")
    sections.append(
        ("ros2 node list", skipped or _command(["ros2", "node", "list"], timeout=8)))

    lifecycle = []
    for node in ("/planner_server", "/controller_server", "/bt_navigator",
                 "/amcl", "/map_server"):
        skipped = within_budget(node)
        body = skipped or _command(["ros2", "lifecycle", "get", node], timeout=3)
        lifecycle.append(f"--- {node} ---\n{body}")
    sections.append(("nav2 lifecycle", "\n".join(lifecycle)))

    sections.append(("disk", _command(["df", "-h", str(destination)], timeout=5)))

    with open(path, "w") as handle:
        for title, body in sections:
            handle.write(f"\n===== {title} =====\n{body}\n")
    return path


def write_environment(dest) -> pathlib.Path:
    """Write :func:`environment_snapshot` beside the run's metrics."""
    destination = pathlib.Path(dest)
    destination.mkdir(parents=True, exist_ok=True)
    path = destination / "environment.json"
    path.write_text(json.dumps(environment_snapshot(), indent=2, sort_keys=True) + "\n")
    return path
