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

"""Tests for per-run forensic capture.

Why it exists
    The 2026-08-01 planner_server segfault was diagnosed from a core dump nobody had
    arranged to keep. Apport happened to catch it, and it survived only because no second
    planner crash overwrote it -- apport stores exactly one report per executable path and
    skips writing when one is already there. The bt_navigator crash from the night before
    was found by accident in the same directory, three days late, with nothing tying it to
    the run that produced it.

    A campaign that has to be re-run because the evidence was thrown away costs more than
    a campaign that records too much. So the run directory becomes the whole story: node
    logs, crash reports, the state of the stack at the instant the watchdog fired, and the
    versions everything was built from.

What it must get right
    Evidence must never be destroyed in the act of collecting it. The harvester moves crash
    reports out of /var/crash so the next crash has somewhere to land, which means a failed
    copy that still deleted the original would lose the only artefact of the failure being
    investigated. That case gets its own test.
"""

import os
import pathlib
import signal
import subprocess
import time

import pytest

TEST_DIR = os.path.dirname(os.path.abspath(__file__))


@pytest.fixture(scope="module")
def fx():
    from navlearn_analysis.harness import run_forensics

    return run_forensics


@pytest.fixture(scope="module")
def harness():
    from navlearn_analysis.harness import multi_run_harness

    return multi_run_harness


def _crash(directory, name, mtime):
    path = directory / name
    path.write_bytes(b"fake apport report")
    os.utime(path, (mtime, mtime))
    return path


# --- crash harvesting --------------------------------------------------------------


def test_harvest_takes_only_reports_from_this_run(fx, tmp_path):
    """A crash from last week is not evidence about tonight's cell."""
    crash_dir = tmp_path / "crash"
    crash_dir.mkdir()
    dest = tmp_path / "run"
    dest.mkdir()
    now = time.time()
    _crash(crash_dir, "old.crash", now - 86400)
    _crash(crash_dir, "new.crash", now)

    taken = fx.harvest_crash_reports(dest, since=now - 60, crash_dir=crash_dir)

    assert [p.name for p in taken] == ["new.crash"]
    assert (dest / "new.crash").is_file()
    assert not (dest / "old.crash").exists()


def test_harvest_clears_the_slot_so_a_second_crash_is_recorded(fx, tmp_path):
    """Apport keeps one report per executable and skips writing when one exists.

    Leaving the first report in place means the second crash of the same node -- the one
    that shows whether the fault is deterministic -- is silently never written.
    """
    crash_dir = tmp_path / "crash"
    crash_dir.mkdir()
    dest = tmp_path / "run"
    dest.mkdir()
    original = _crash(crash_dir, "planner.crash", time.time())

    fx.harvest_crash_reports(dest, since=0, crash_dir=crash_dir)

    assert not original.exists()
    assert (dest / "planner.crash").is_file()


def test_harvest_keeps_the_original_when_the_copy_fails(fx, tmp_path):
    """Never destroy evidence in the act of failing to save it."""
    crash_dir = tmp_path / "crash"
    crash_dir.mkdir()
    original = _crash(crash_dir, "planner.crash", time.time())
    # A destination that cannot be made into a directory — the shape a full disk or a
    # permission problem takes at this call site.
    blocked = tmp_path / "not_a_directory"
    blocked.write_text("this is a file")

    taken = fx.harvest_crash_reports(blocked, since=0, crash_dir=crash_dir)

    assert taken == []
    assert original.exists(), "the only copy of the crash was deleted"


def test_harvest_is_quiet_when_there_is_no_crash_directory(fx, tmp_path):
    """Machines without apport must not fail a healthy cell."""
    assert (
        fx.harvest_crash_reports(tmp_path, since=0, crash_dir=tmp_path / "nope") == []
    )


# --- environment -------------------------------------------------------------------


def test_environment_snapshot_records_what_the_stack_was_built_from(fx):
    """'It worked last month' is unanswerable without the versions it worked with."""
    snapshot = fx.environment_snapshot()

    for key in ("kernel", "ros_distro", "nav2_versions", "cpu_count", "core_limit"):
        assert key in snapshot, f"missing {key}"
    assert snapshot["nav2_versions"], "no nav2 package versions captured"


def test_environment_snapshot_records_overridden_packages(fx):
    """A dpkg version is a lie when a workspace overlay shadows the installed package.

    nav2_smac_planner is vendored and patched in this workspace. environment.json reporting
    only 'ros-humble-nav2-smac-planner 1.1.18-1jammy' would describe a stack that is not the
    one running, and every result taken against it would be unreproducible by anyone
    reading the record.
    """
    snapshot = fx.environment_snapshot()

    assert "overlays" in snapshot
    assert "nav2_smac_planner" in snapshot["overlays"]
    location = snapshot["overlays"]["nav2_smac_planner"]
    assert location, "the resolved location of an overridden package was not recorded"


def test_environment_snapshot_is_json_serialisable(fx, tmp_path):
    """It is written to disk beside the metrics; a non-serialisable value loses all of it."""
    import json

    path = tmp_path / "env.json"
    path.write_text(json.dumps(fx.environment_snapshot(), indent=2, sort_keys=True))

    assert json.loads(path.read_text())["nav2_versions"]


# --- state at the moment of failure ------------------------------------------------


def test_capture_stack_state_records_the_live_processes(fx, tmp_path):
    """The watchdog names what died; this says what everything else was doing."""
    path = fx.capture_stack_state(tmp_path, reason="test")

    assert path.is_file()
    text = path.read_text()
    assert str(os.getpid()) in text, "the process table was not captured"
    assert "test" in text, "the abort reason was not recorded alongside the state"


def test_capture_stack_state_survives_a_missing_ros_environment(fx, tmp_path):
    """Best-effort probes must not turn a diagnosis into a second failure."""
    assert fx.capture_stack_state(tmp_path, reason="no ros here").is_file()


def test_capture_stack_state_bounds_how_long_it_blocks_the_abort(fx, tmp_path):
    """The launch is still running while this executes.

    Querying a stack that has just died is exactly the case where every ROS probe waits
    out its full discovery timeout. Six of them in series held the abort for 69 s in the
    replay -- 69 s in which the hung launch was still alive and the campaign still stuck.
    The cheap reads must survive the cap, and a skipped probe must say so rather than look
    like a clean empty result.
    """
    start = time.monotonic()
    path = fx.capture_stack_state(tmp_path, reason="bounded", budget_s=0.0)
    elapsed = time.monotonic() - start

    assert elapsed < 5.0, f"forensics blocked the abort for {elapsed:.1f} s"
    text = path.read_text()
    assert str(os.getpid()) in text, "the cheap process snapshot was dropped too"
    assert "budget" in text, "a skipped probe must be recorded, not silently omitted"


# --- rosbag recording ---------------------------------------------------------------


def test_rosbag_records_every_topic(fx, tmp_path):
    """A bag that omits a topic cannot answer the question nobody thought to ask.

    There are no image or point-cloud topics on this robot -- 68 topics, the heaviest being
    two OccupancyGrids at 1 Hz -- so recording everything is affordable and is the only
    option that does not require predicting which signal a future diagnosis will need.
    """
    argv = fx.rosbag_command(tmp_path / "rosbag_run_1")

    assert argv[:3] == ["ros2", "bag", "record"]
    assert "--all" in argv or "-a" in argv
    assert str(tmp_path / "rosbag_run_1") in argv


def test_rosbag_directory_is_not_created_in_advance(fx, tmp_path):
    """`ros2 bag record -o DIR` refuses to start when DIR already exists."""
    target = fx.rosbag_dir(tmp_path, episode_id=2, stamp="20260801_235959")

    assert not target.exists()
    assert target.parent == pathlib.Path(tmp_path)
    assert "2" in target.name and "20260801_235959" in target.name


def test_recorder_is_stopped_with_sigint_so_the_bag_is_finalised(fx, tmp_path):
    """rosbag2 writes metadata.yaml on SIGINT.

    rclcpp installs a SIGINT handler; SIGTERM kills the recorder outright and leaves a
    directory with a .db3 in it and no metadata.yaml, which every rosbag2 reader refuses to
    open. A whole campaign of unreadable bags looks exactly like a campaign of good ones
    until someone tries to use them.
    """
    marker = tmp_path / "clean_exit"
    process = subprocess.Popen(
        [
            "bash",
            "-c",
            f'trap "touch {marker}; exit 0" INT; while true; do sleep 0.1; done',
        ],
        start_new_session=True,
    )
    time.sleep(0.5)

    fx.stop_process(process, sig=signal.SIGINT, grace_s=5)

    assert (
        marker.exists()
    ), "the recorder was not given SIGINT; the bag would be unreadable"
    assert process.poll() is not None


def test_stop_process_does_not_signal_the_callers_own_process_group(fx, tmp_path):
    """Signalling a process group is only safe when the target leads its own group.

    The three samplers are started without ``start_new_session``, so they share the
    harness's process group. An unconditional ``killpg`` therefore signals the harness
    itself and whatever shell launched it. Observed for real: the samplers wrote their
    records, the harness died partway through its own cleanup loop, and the rosbag recorder
    -- the one child in its own session, and the last entry in the list -- was never
    stopped, leaving a bag with no metadata.yaml.

    Two children in the caller's group: stopping one must not touch the other.
    """
    victim = subprocess.Popen(["bash", "-c", "while true; do sleep 0.1; done"])
    bystander = subprocess.Popen(["bash", "-c", "while true; do sleep 0.1; done"])
    try:
        time.sleep(0.4)
        assert (
            os.getpgid(victim.pid) == os.getpgid(bystander.pid) == os.getpgid(0)
        ), "test precondition: both children must share the caller's group"

        fx.stop_process(victim, sig=signal.SIGTERM, grace_s=5)

        assert victim.poll() is not None, "the target was not stopped"
        time.sleep(0.3)
        assert (
            bystander.poll() is None
        ), "a sibling in the same process group was signalled too"
    finally:
        for p in (victim, bystander):
            if p.poll() is None:
                p.kill()
            p.wait()


def test_stop_process_escalates_when_the_signal_is_ignored(fx, tmp_path):
    """A recorder wedged on a full disk must not hold the campaign open forever."""
    process = subprocess.Popen(
        ["bash", "-c", 'trap "" INT; while true; do sleep 0.1; done'],
        start_new_session=True,
    )
    time.sleep(0.5)

    fx.stop_process(process, sig=signal.SIGINT, grace_s=1)

    assert process.poll() is not None, "an unresponsive recorder was left running"


def test_a_recorder_that_cannot_start_does_not_abort_the_episode(fx, tmp_path):
    """Losing the bag is bad; losing the run because the bag failed is worse."""
    started = fx.start_recorder(tmp_path / "bag", executable="navlearn_no_such_binary")

    assert started is None


# --- node logs into the run directory ----------------------------------------------


def test_episode_environment_points_ros_logs_at_the_run_directory(fx, tmp_path):
    """~/.ros/log is shared by every run; a per-episode dir is what ties logs to a cell."""
    target = tmp_path / "ros_logs" / "run_3"

    env = fx.episode_environment(target, base_env={"PATH": "/usr/bin"})

    assert env["ROS_LOG_DIR"] == str(target)
    assert env["PATH"] == "/usr/bin", "the rest of the environment must be preserved"
    assert target.is_dir(), "rcl does not create the directory itself"


def test_launch_supervised_passes_the_environment_to_the_launch(harness, tmp_path):
    """Setting ROS_LOG_DIR is pointless if the launch never receives it."""

    class Never:
        def poll(self):
            return None

    log = tmp_path / "out.log"
    with open(log, "w") as handle:
        harness.launch_supervised(
            ["sh", "-c", "echo LOGDIR=$ROS_LOG_DIR"],
            handle,
            Never(),
            poll_interval_s=0.05,
            env={"ROS_LOG_DIR": "/tmp/navlearn-probe", "PATH": "/bin:/usr/bin"},
        )

    assert "LOGDIR=/tmp/navlearn-probe" in log.read_text()


def test_harness_exposes_a_forensics_switch(harness, monkeypatch):
    """Campaign runs keep everything; a quick local probe may not want the overhead."""
    import sys

    monkeypatch.setattr(
        sys,
        "argv",
        [
            "multi_run_harness.py",
            "--perturbation",
            "clean",
            "--output-dir",
            "/tmp/unused",
        ],
    )
    args = harness.parse_args()

    assert args.forensics is True, "forensic capture must be on by default"


def test_run_log_directory_is_created_under_the_run_directory(fx, tmp_path):
    """Everything about one episode lives in one place, or it will not be found later."""
    target = fx.run_log_dir(tmp_path, episode_id=7, stamp="20260801_083325")

    assert target.parent.parent == pathlib.Path(tmp_path)
    assert "7" in target.name and "20260801_083325" in target.name
