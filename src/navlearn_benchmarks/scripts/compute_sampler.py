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

"""Compute-cost sampler for NavLearn benchmark runs.

What it measures and why each piece is here
    Per-process CPU
        Read as deltas of ``utime + stime`` from ``/proc/<pid>/stat`` between samples.
        The retired campaign polled ``ps -o %cpu``, which reports a *cumulative average
        over the process lifetime*, not instantaneous load. Averaging that column, as
        ``phase3_compute_profile.sh`` did, produces a number dominated by process startup
        that cannot show a transient spike no matter how fast it is sampled. Deltas of the
        raw counters are the only correct way to get instantaneous CPU from /proc.

    Process classification
        Every sampled process is assigned to ROBOT, SIMULATION, MEASUREMENT or INFRA.
        A single machine-wide CPU figure is meaningless for a paper about robot compute:
        Gazebo dominates it, and the benchmark's own metric nodes contribute too. Only the
        ROBOT class is a claim about what the navigation stack would cost on a real robot.

    CPU-seconds, not just percent
        Percent-of-a-core is specific to this i5-11260H. CPU-seconds per goal and per metre
        travelled are what transfer to a Jetson or an industrial PC, so the integral is
        recorded alongside the rate.

    Memory: RSS and PSS
        PSS (from ``smaps_rollup``) apportions shared library pages across the processes
        sharing them. With a dozen ROS nodes all mapping the same libraries, summing RSS
        double-counts heavily. Peak PSS is what determines whether a stack fits in a target
        board's memory.

    Frequency and thermal state
        This is a laptop: the i5-11260H scales 800-4400 MHz and will throttle over a
        multi-hour campaign. If one controller's cells run on a hot chassis and another's
        run cold, a compute difference between them is unattributable. Per-core frequency,
        package temperature and the kernel's throttle counters are recorded every sample,
        and the summary reports whether any throttling occurred *during this run*. A cell
        that throttled is a cell whose compute numbers carry an asterisk.

    GPU
        Recorded as explicitly absent when no driver is present, rather than omitted. As of
        2026-07-28 this machine has an RTX 3050 Mobile with no NVIDIA driver installed, and
        Nav2's MPPI links no CUDA, so navigation is entirely CPU-bound. Stating that is a
        methods fact; leaving it out invites the reviewer question. The fields populate
        automatically if a driver appears, which is the path to the real-robot paper.

Outputs
    ``<prefix>_samples.csv``   one row per process per sample, plus system rows
    ``<prefix>_summary.json``  per-class and per-process aggregates, thermal verdict

Usage
    compute_sampler.py --output-prefix DIR/run_1 [--interval 0.2] [--duration SECONDS]

    Runs until SIGINT/SIGTERM or --duration elapses, then writes the summary. The harness
    starts one per benchmark run and stops it when the run's launch exits.
"""

import argparse
import glob
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time

CLK_TCK = os.sysconf("SC_CLK_TCK")
PAGE_SIZE = os.sysconf("SC_PAGE_SIZE")
CPU_COUNT = os.cpu_count() or 1

# Process classification. Order matters: the first pattern that matches wins, so the
# specific measurement and simulation names are tested before the broad robot ones.
#
# ROBOT is the only class that constitutes a claim about navigation compute cost.
CLASSES = (
    (
        "SIMULATION",
        re.compile(
            r"gz[\s_-]?sim|gzserver|gzclient|ign[\s_-]gazebo|ruby|ros_gz|parameter_bridge"
            r"|rviz2|gz_set_pose",
            re.I,
        ),
    ),
    (
        "MEASUREMENT",
        re.compile(
            r"episode_manager|metrics_compiler|control_metric|trajectory_metric"
            r"|localization_metrics|ground_truth_publisher|collision_",
            re.I,
        ),
    ),
    (
        "ROBOT",
        re.compile(
            r"controller_server|planner_server|bt_navigator|behavior_server"
            r"|smoother_server|amcl|map_server|waypoint_follower|velocity_smoother"
            r"|lifecycle_manager",
            re.I,
        ),
    ),
    (
        "INFRA",
        re.compile(
            r"robot_state_publisher|controller_manager|spawner|twist_mux|twist_relay"
            r"|joy_node|joy_teleop|joystick|scan_sanitizer|noisy_controller|safety_stop",
            re.I,
        ),
    ),
)


# Interpreters that host a ROS node whose identity is only visible in the command line.
# Everything else is classified on the process name alone.
INTERPRETERS = re.compile(r"^(python3?(\.\d+)?|ruby[\d.]*)$", re.I)

# Shells and process-inspection tools are never part of the stack, and their command lines
# routinely quote node names — an orchestrator script, the harness itself, or a `grep
# controller_server` would otherwise be classified as the thing it merely mentions and
# counted as a near-idle member of that class, deflating its mean.
NEVER = re.compile(r"^(bash|sh|dash|zsh|grep|awk|sed|ps|xargs|tee|timeout|env|sudo)$", re.I)


def classify(name, cmdline):
    """Assign a process to a compute class, or None if it is not part of the stack.

    Matching is on the process name, not the command line. Only interpreters are resolved
    through their command line, because a Python ROS node reports ``python3`` as its name
    and is otherwise unidentifiable.
    """
    if NEVER.match(name):
        return None

    for label, pattern in CLASSES:
        if pattern.search(name):
            return label

    if INTERPRETERS.match(name):
        # Inspect only the script path and arguments, never the interpreter's own name.
        for label, pattern in CLASSES:
            if pattern.search(cmdline):
                return label
    return None


def read_proc_stat(pid):
    """Return (name, utime+stime ticks, num_threads) for a pid, or None if it vanished.

    The comm field is parenthesised and may itself contain spaces and parentheses, so the
    split is anchored on the last ')' rather than on whitespace.
    """
    try:
        with open(f"/proc/{pid}/stat") as handle:
            raw = handle.read()
    except (OSError, ValueError):
        return None

    try:
        close = raw.rindex(")")
        name = raw[raw.index("(") + 1: close]
        fields = raw[close + 2:].split()
        # Fields after comm and state: utime is index 11, stime 12, num_threads 17
        # (0-based within this slice, per proc(5) counting from field 3).
        utime, stime = int(fields[11]), int(fields[12])
        threads = int(fields[17])
    except (ValueError, IndexError):
        return None
    return name, utime + stime, threads


def read_memory(pid):
    """Return (rss_kb, pss_kb) for a pid; pss is None when smaps_rollup is unreadable."""
    rss = pss = None
    try:
        with open(f"/proc/{pid}/statm") as handle:
            rss = int(handle.read().split()[1]) * PAGE_SIZE // 1024
    except (OSError, ValueError, IndexError):
        return None, None

    try:
        with open(f"/proc/{pid}/smaps_rollup") as handle:
            for line in handle:
                if line.startswith("Pss:"):
                    pss = int(line.split()[1])
                    break
    except (OSError, ValueError, IndexError):
        pss = None
    return rss, pss


def read_cmdline(pid):
    """Return a pid's command line with NULs replaced by spaces, or '' if unreadable."""
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as handle:
            return handle.read().replace(b"\0", b" ").decode("utf-8", "replace").strip()
    except OSError:
        return ""


def cpu_frequencies_khz():
    """Return the current scaling frequency of every core, in kHz."""
    freqs = []
    for path in sorted(glob.glob("/sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_cur_freq")):
        try:
            with open(path) as handle:
                freqs.append(int(handle.read().strip()))
        except (OSError, ValueError):
            continue
    return freqs


def package_temp_c():
    """Return the CPU package temperature in Celsius, or None if unavailable."""
    for path in glob.glob("/sys/class/thermal/thermal_zone*"):
        try:
            with open(os.path.join(path, "type")) as handle:
                if handle.read().strip() not in ("TCPU", "x86_pkg_temp", "acpitz"):
                    continue
            with open(os.path.join(path, "temp")) as handle:
                return int(handle.read().strip()) / 1000.0
        except (OSError, ValueError):
            continue
    return None


def throttle_counters():
    """Return summed core and package throttle counts and total throttled milliseconds."""
    totals = {"core_throttle_count": 0, "package_throttle_count": 0,
              "core_throttle_total_time_ms": 0, "package_throttle_total_time_ms": 0}
    for key in totals:
        for path in glob.glob(f"/sys/devices/system/cpu/cpu[0-9]*/thermal_throttle/{key}"):
            try:
                with open(path) as handle:
                    totals[key] += int(handle.read().strip())
            except (OSError, ValueError):
                continue
    return totals


def gpu_state():
    """Describe GPU availability, so its absence is recorded rather than merely implied."""
    state = {"present_in_hardware": False, "driver_available": False,
             "used_by_navigation": False, "devices": [], "note": ""}

    try:
        listing = subprocess.run(
            ["lspci"], capture_output=True, text=True, timeout=10
        ).stdout
        state["devices"] = [
            line.strip() for line in listing.splitlines()
            if re.search(r"VGA compatible controller|3D controller", line)
        ]
        state["present_in_hardware"] = bool(state["devices"])
    except (OSError, subprocess.SubprocessError):
        state["note"] = "lspci unavailable"

    if shutil.which("nvidia-smi"):
        try:
            result = subprocess.run(
                ["nvidia-smi", "--query-gpu=name,utilization.gpu,memory.used",
                 "--format=csv,noheader,nounits"],
                capture_output=True, text=True, timeout=10,
            )
            if result.returncode == 0:
                state["driver_available"] = True
                state["nvidia_smi"] = result.stdout.strip()
        except (OSError, subprocess.SubprocessError):
            pass

    if not state["driver_available"]:
        state["note"] = (
            "No NVIDIA driver present. Nav2's MPPI controller links no CUDA, so navigation "
            "is CPU-bound regardless; a GPU would accelerate simulator rendering only, "
            "which headless operation removes. Recorded as a methods fact."
        )
    return state


def sample_processes(previous, now, elapsed):
    """Produce one sample across all classified processes.

    Args:
        previous: mapping of pid -> (cpu_ticks, name, klass) from the last sample.
        now: wall-clock timestamp of this sample.
        elapsed: seconds since the previous sample.

    Returns:
        (rows, current) where rows are per-process dicts and current seeds the next call.
    """
    rows, current = [], {}

    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        pid = int(entry)

        stat = read_proc_stat(pid)
        if stat is None:
            continue
        name, ticks, threads = stat

        cmdline = read_cmdline(pid)
        klass = classify(name, cmdline)
        if klass is None:
            continue

        current[pid] = (ticks, name, klass)

        # A process seen for the first time has no delta yet; it contributes from the next
        # sample onward. Reporting its lifetime average here would repeat the ps %cpu bug.
        if pid not in previous or elapsed <= 0:
            continue
        prev_ticks = previous[pid][0]
        if ticks < prev_ticks:  # pid reused
            continue

        cpu_seconds = (ticks - prev_ticks) / CLK_TCK
        rss, pss = read_memory(pid)
        rows.append({
            "timestamp": round(now, 3),
            "kind": "process",
            "pid": pid,
            "name": name,
            "class": klass,
            "cpu_percent_of_core": round(100.0 * cpu_seconds / elapsed, 2),
            "cpu_percent_of_machine": round(100.0 * cpu_seconds / elapsed / CPU_COUNT, 3),
            "cpu_seconds_delta": round(cpu_seconds, 6),
            "threads": threads,
            "rss_kb": rss,
            "pss_kb": pss,
        })

    return rows, current


def system_row(now):
    """Produce one system-level sample: frequency, temperature, throttle state, load."""
    freqs = cpu_frequencies_khz()
    counters = throttle_counters()
    try:
        load1 = os.getloadavg()[0]
    except OSError:
        load1 = None

    return {
        "timestamp": round(now, 3),
        "kind": "system",
        "pid": None,
        "name": "system",
        "class": "SYSTEM",
        "cpu_freq_mean_mhz": round(sum(freqs) / len(freqs) / 1000.0, 1) if freqs else None,
        "cpu_freq_min_mhz": round(min(freqs) / 1000.0, 1) if freqs else None,
        "cpu_freq_max_mhz": round(max(freqs) / 1000.0, 1) if freqs else None,
        "package_temp_c": package_temp_c(),
        "load_avg_1min": load1,
        **counters,
    }


def summarise(process_rows, system_rows, throttle_start, throttle_end, started, ended):
    """Aggregate samples into the figures a results table needs.

    Reports per class and per process name. The thermal verdict is explicit: a run during
    which the kernel recorded a throttle event has compute numbers that are not comparable
    with an unthrottled run, and that must be visible in the data rather than inferred.
    """
    def percentile(values, fraction):
        if not values:
            return None
        ordered = sorted(values)
        index = min(int(round(fraction * (len(ordered) - 1))), len(ordered) - 1)
        return round(ordered[index], 2)

    by_class, by_name = {}, {}
    for row in process_rows:
        by_class.setdefault(row["class"], []).append(row)
        by_name.setdefault((row["class"], row["name"]), []).append(row)

    def aggregate(rows):
        cpu = [r["cpu_percent_of_core"] for r in rows]
        pss = [r["pss_kb"] for r in rows if r["pss_kb"] is not None]
        rss = [r["rss_kb"] for r in rows if r["rss_kb"] is not None]
        return {
            "samples": len(rows),
            "cpu_percent_of_core_mean": round(sum(cpu) / len(cpu), 2) if cpu else None,
            "cpu_percent_of_core_p95": percentile(cpu, 0.95),
            "cpu_percent_of_core_peak": round(max(cpu), 2) if cpu else None,
            "cpu_seconds_total": round(sum(r["cpu_seconds_delta"] for r in rows), 3),
            "peak_rss_kb": max(rss) if rss else None,
            "peak_pss_kb": max(pss) if pss else None,
            "max_threads": max((r["threads"] for r in rows), default=None),
        }

    throttled = {
        key: throttle_end.get(key, 0) - throttle_start.get(key, 0) for key in throttle_end
    }
    any_throttle = any(
        value > 0 for key, value in throttled.items() if key.endswith("_count")
    )

    freqs = [r["cpu_freq_mean_mhz"] for r in system_rows if r["cpu_freq_mean_mhz"]]
    temps = [r["package_temp_c"] for r in system_rows if r["package_temp_c"]]

    return {
        "schema": "navlearn.compute_summary/1",
        "wall_seconds": round(ended - started, 3),
        "cpu_count": CPU_COUNT,
        "by_class": {name: aggregate(rows) for name, rows in sorted(by_class.items())},
        "by_process": {
            f"{klass}/{name}": aggregate(rows)
            for (klass, name), rows in sorted(by_name.items())
        },
        "thermal": {
            "throttle_events_during_run": throttled,
            "throttled": any_throttle,
            "verdict": (
                "THROTTLED — compute figures for this run are not comparable with "
                "unthrottled runs and must be excluded or flagged in analysis."
                if any_throttle else
                "clean — no kernel throttle event recorded during this run"
            ),
            "cpu_freq_mhz_mean": round(sum(freqs) / len(freqs), 1) if freqs else None,
            "cpu_freq_mhz_min": min(freqs) if freqs else None,
            "cpu_freq_mhz_max": max(freqs) if freqs else None,
            "package_temp_c_mean": round(sum(temps) / len(temps), 1) if temps else None,
            "package_temp_c_max": max(temps) if temps else None,
        },
        "gpu": gpu_state(),
        "notes": {
            "cpu_method": (
                "utime+stime deltas from /proc/<pid>/stat over the sample interval. "
                "Not ps %cpu, which is a lifetime cumulative average."
            ),
            "robot_class_meaning": (
                "Only the ROBOT class represents navigation compute. SIMULATION is "
                "Gazebo, MEASUREMENT is this benchmark's own instrumentation, and neither "
                "would exist on a real robot."
            ),
            "energy": (
                "Intel RAPL energy counters require root and were not read. Joules per "
                "goal is the metric that matters for a battery-powered platform and is "
                "the natural extension for real-robot work."
            ),
        },
    }


def main():
    """Sample until stopped, then write the per-sample CSV and the summary JSON."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--output-prefix", required=True,
                        help="Path prefix; _samples.csv and _summary.json are appended")
    parser.add_argument("--interval", type=float, default=0.2,
                        help="Seconds between samples")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="Stop after this many seconds; 0 means run until signalled")
    args = parser.parse_args()

    running = {"value": True}

    def stop(_signum, _frame):
        running["value"] = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    os.makedirs(os.path.dirname(os.path.abspath(args.output_prefix)), exist_ok=True)
    samples_path = f"{args.output_prefix}_samples.csv"
    summary_path = f"{args.output_prefix}_summary.json"

    throttle_start = throttle_counters()
    started = time.monotonic()
    started_wall = time.time()

    process_rows, system_rows = [], []
    previous, last = {}, started

    while running["value"]:
        time.sleep(args.interval)
        now = time.monotonic()
        elapsed = now - last

        rows, previous = sample_processes(previous, now - started, elapsed)
        process_rows.extend(rows)
        system_rows.append(system_row(now - started))
        last = now

        if args.duration and (now - started) >= args.duration:
            break

    ended = time.monotonic()
    throttle_end = throttle_counters()

    columns = [
        "timestamp", "kind", "pid", "name", "class",
        "cpu_percent_of_core", "cpu_percent_of_machine", "cpu_seconds_delta",
        "threads", "rss_kb", "pss_kb",
        "cpu_freq_mean_mhz", "cpu_freq_min_mhz", "cpu_freq_max_mhz",
        "package_temp_c", "load_avg_1min",
        "core_throttle_count", "package_throttle_count",
        "core_throttle_total_time_ms", "package_throttle_total_time_ms",
    ]
    with open(samples_path, "w") as handle:
        handle.write(",".join(columns) + "\n")
        for row in sorted(process_rows + system_rows, key=lambda r: r["timestamp"]):
            handle.write(
                ",".join("" if row.get(c) is None else str(row.get(c)) for c in columns) + "\n"
            )

    summary = summarise(
        process_rows, system_rows, throttle_start, throttle_end, started, ended
    )
    summary["started_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime(started_wall))
    summary["sample_interval_s"] = args.interval
    with open(summary_path, "w") as handle:
        json.dump(summary, handle, indent=2, sort_keys=True)
        handle.write("\n")

    print(f"compute_sampler: {len(process_rows)} process samples, "
          f"{len(system_rows)} system samples -> {samples_path}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
