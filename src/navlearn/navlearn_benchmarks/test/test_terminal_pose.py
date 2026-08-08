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

"""Integration tests for the terminal pose report reaching the metrics CSV.

Why these run without a simulator
    MetricsCompiler emits a row once it has an END event, a ControlMetric and a
    TrajectoryMetric sharing a goal id. Nothing about that requires Gazebo, so the whole
    path from message contract to CSV column can be exercised directly by publishing the
    three messages. The alternative — discovering a column misalignment during the pilot
    cell — costs a sim run to find and another to confirm.

What they protect
    The terminal block is the campaign's headline measurement. Two failure modes would be
    silent and destructive: a header and row that disagree in column count, which shifts
    every downstream column by one and is invisible in a spreadsheet; and a missing
    ground-truth value written as 0.0, which reads as a perfect result rather than as an
    absent measurement.
"""

import csv
import itertools
import os
import subprocess
import time

import pytest

rclpy = pytest.importorskip("rclpy", reason="ROS 2 Python runtime not available")

from builtin_interfaces.msg import Duration  # noqa: E402
from navlearn_msgs.msg import (  # noqa: E402
    ControlMetric,
    EpisodeEvent,
    KidnapEvent,
    TrajectoryMetric,
)
from unique_identifier_msgs.msg import UUID  # noqa: E402

TERMINAL_COLUMNS = [
    "GT Available",
    "Estimate Available",
    "True Pose_X (m)",
    "True Pose_Y (m)",
    "True Pose_Yaw (deg)",
    "Estimated Pose_X (m)",
    "Estimated Pose_Y (m)",
    "Estimated Pose_Yaw (deg)",
    "True Distance To Goal (m)",
    "Estimated Distance To Goal (m)",
    "Localization Error (m)",
    "True Yaw Error (deg)",
    "Covariance_XX (m2)",
    "Covariance_YY (m2)",
    "Covariance_Yaw (rad2)",
    "Filter Converged",
    "Convergence Threshold (m2)",
    "Estimate Age (sec)",
    "False Success",
]


def _uuid(seed):
    """Build a deterministic UUID message from a single byte."""
    return UUID(uuid=[seed] * 16)


def _episode_end(goal_id, *, result, true_xy, est_xy, goal_xy, gt_available=True):
    """Build a terminated EpisodeEvent with a populated terminal report."""
    ev = EpisodeEvent()
    ev.state = EpisodeEvent.END
    ev.result = result
    ev.goal_id = goal_id
    ev.goal_pose.pose.position.x, ev.goal_pose.pose.position.y = goal_xy
    ev.goal_pose.pose.orientation.w = 1.0
    ev.start_pose.pose.orientation.w = 1.0
    ev.optimal_path_m = 5.0
    ev.nav_time = Duration(sec=10, nanosec=0)

    term = ev.terminal
    term.gt_available = gt_available
    term.estimate_available = True
    if gt_available:
        term.true_pose.pose.position.x, term.true_pose.pose.position.y = true_xy
        term.true_pose.pose.orientation.w = 1.0
        term.true_distance_to_goal_m = (
            (true_xy[0] - goal_xy[0]) ** 2 + (true_xy[1] - goal_xy[1]) ** 2
        ) ** 0.5
        term.true_yaw_error_rad = 0.0
        term.localization_error_m = (
            (true_xy[0] - est_xy[0]) ** 2 + (true_xy[1] - est_xy[1]) ** 2
        ) ** 0.5
    else:
        term.true_distance_to_goal_m = -1.0
        term.true_yaw_error_rad = -1.0
        term.localization_error_m = -1.0

    term.estimated_pose.pose.position.x, term.estimated_pose.pose.position.y = est_xy
    term.estimated_pose.pose.orientation.w = 1.0
    term.estimated_distance_to_goal_m = (
        (est_xy[0] - goal_xy[0]) ** 2 + (est_xy[1] - goal_xy[1]) ** 2
    ) ** 0.5
    term.covariance_xx = 0.01
    term.covariance_yy = 0.01
    term.covariance_yaw = 0.02
    term.filter_converged = True
    term.convergence_threshold_m2 = 0.25
    term.estimate_age_sec = 0.35
    return ev


def _compiler_executable():
    """Locate the metrics_compiler binary in the installed workspace."""
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        candidate = os.path.join(
            prefix, "lib", "navlearn_benchmarks", "metrics_compiler"
        )
        if os.path.isfile(candidate):
            return candidate
    pytest.skip("metrics_compiler executable not found in AMENT_PREFIX_PATH")


_node_counter = itertools.count()


def _run_compiler(tmp_path, events, kidnaps=()):
    """Run metrics_compiler against a set of episodes and return the parsed CSV rows.

    `kidnaps` are published before the episodes they belong to. The compiler folds a
    KidnapEvent into an episode by goal id but does not wait for one — clean and TTC
    cells produce none — so a kidnap arriving after the flush would be dropped silently.

    The node binary is executed directly rather than through `ros2 run`. `ros2 run` is a
    wrapper process: terminating it leaves the node it spawned running, and a surviving
    compiler from an earlier test steals the next test's messages while still holding the
    previous test's output path. That produced four spurious failures before it was found.
    """
    csv_path = tmp_path / "metrics.csv"
    json_path = tmp_path / "report.json"
    node_name = f"metrics_compiler_test_{next(_node_counter)}"

    proc = subprocess.Popen(
        [
            _compiler_executable(),
            "--ros-args",
            "-r",
            f"__node:={node_name}",
            "-p",
            f"csv_path:={csv_path}",
            "-p",
            f"json_path:={json_path}",
            "-p",
            "false_success_threshold_m:=0.10",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

    rclpy.init()
    node = rclpy.create_node("terminal_pose_test_publisher")
    ep_pub = node.create_publisher(EpisodeEvent, "/navlearn/episode_event", 10)
    cm_pub = node.create_publisher(ControlMetric, "/navlearn/control_metric", 10)
    tm_pub = node.create_publisher(TrajectoryMetric, "/navlearn/trajectory_metric", 10)
    kd_pub = node.create_publisher(KidnapEvent, "/navlearn/kidnap_event", 10)

    try:
        # Wait for the compiler's subscriptions to appear rather than sleeping a fixed
        # interval; discovery time varies and a fixed wait is how these tests turn flaky.
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            if (
                ep_pub.get_subscription_count() > 0
                and cm_pub.get_subscription_count() > 0
                and tm_pub.get_subscription_count() > 0
                and kd_pub.get_subscription_count() > 0
            ):
                break
            rclpy.spin_once(node, timeout_sec=0.1)
        else:
            pytest.fail("metrics_compiler never subscribed; is the workspace sourced?")

        for kd in kidnaps:
            kd_pub.publish(kd)
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.05)

        for ev in events:
            cm = ControlMetric(goal_id=ev.goal_id, samples=100)
            cm.tracking_rms_v = 0.05
            tm = TrajectoryMetric(goal_id=ev.goal_id, samples=100)
            tm.path_length_m = 5.5
            tm.min_clearance_m = 0.4

            ep_pub.publish(ev)
            cm_pub.publish(cm)
            tm_pub.publish(tm)
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.05)

        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            if csv_path.is_file():
                with open(csv_path) as handle:
                    rows = list(csv.DictReader(handle))
                if len(rows) >= len(events):
                    return rows, csv_path
            rclpy.spin_once(node, timeout_sec=0.1)
        pytest.fail(f"metrics_compiler wrote no complete CSV at {csv_path}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()


@pytest.fixture(scope="module")
def sourced():
    """Skip the module unless the built workspace is on the environment."""
    if "navlearn_benchmarks" not in os.environ.get("AMENT_PREFIX_PATH", ""):
        pytest.skip("workspace not sourced; run after 'source install/setup.bash'")


def test_header_and_row_column_counts_match(sourced, tmp_path):
    """A header/row mismatch shifts every column and is invisible in a spreadsheet."""
    rows, csv_path = _run_compiler(
        tmp_path,
        [
            _episode_end(
                _uuid(1),
                result=EpisodeEvent.RESULT_SUCCEEDED,
                true_xy=(1.0, 0.0),
                est_xy=(0.02, 0.0),
                goal_xy=(0.0, 0.0),
            ),
        ],
    )

    with open(csv_path) as handle:
        lines = [line for line in handle.read().splitlines() if line.strip()]
    header_cols = lines[0].count(",") + 1
    for line in lines[1:]:
        assert (
            line.count(",") + 1 == header_cols
        ), f"row has {line.count(',') + 1} columns, header has {header_cols}"
    assert rows, "no rows parsed"


def test_terminal_columns_present(sourced, tmp_path):
    """Every terminal field reaches the CSV under its documented name."""
    rows, _ = _run_compiler(
        tmp_path,
        [
            _episode_end(
                _uuid(2),
                result=EpisodeEvent.RESULT_SUCCEEDED,
                true_xy=(0.03, 0.0),
                est_xy=(0.02, 0.0),
                goal_xy=(0.0, 0.0),
            ),
        ],
    )
    for column in TERMINAL_COLUMNS:
        assert column in rows[0], f"missing column: {column}"


def test_false_success_is_flagged(sourced, tmp_path):
    """A SUCCEEDED goal whose ground truth is far from the goal is marked.

    This is the measurement the campaign exists to produce: Nav2 reports arrival from the
    localization estimate, so a converged-but-displaced filter yields a success the robot
    did not achieve.
    """
    rows, _ = _run_compiler(
        tmp_path,
        [
            _episode_end(
                _uuid(3),
                result=EpisodeEvent.RESULT_SUCCEEDED,
                true_xy=(1.20, 0.0),
                est_xy=(0.02, 0.0),
                goal_xy=(0.0, 0.0),
            ),
        ],
    )
    row = rows[0]
    assert row["Goal Result"] == "SUCCEEDED"
    assert float(row["True Distance To Goal (m)"]) == pytest.approx(1.20, abs=1e-6)
    assert float(row["Estimated Distance To Goal (m)"]) == pytest.approx(0.02, abs=1e-6)
    assert float(row["Localization Error (m)"]) == pytest.approx(1.18, abs=1e-6)
    assert row["False Success"] == "1"


def test_genuine_success_is_not_flagged(sourced, tmp_path):
    """A goal actually reached must not be marked as a false success."""
    rows, _ = _run_compiler(
        tmp_path,
        [
            _episode_end(
                _uuid(4),
                result=EpisodeEvent.RESULT_SUCCEEDED,
                true_xy=(0.03, 0.0),
                est_xy=(0.02, 0.0),
                goal_xy=(0.0, 0.0),
            ),
        ],
    )
    assert rows[0]["False Success"] == "0"


def test_missing_ground_truth_is_not_a_clean_success(sourced, tmp_path):
    """Absent ground truth reports -1, never 0.

    Zero would read as a perfect result. An unmeasured goal must be distinguishable from
    a perfectly executed one, or the campaign's headline statistic is biased by exactly
    the goals where measurement failed.
    """
    rows, _ = _run_compiler(
        tmp_path,
        [
            _episode_end(
                _uuid(5),
                result=EpisodeEvent.RESULT_SUCCEEDED,
                true_xy=(0.0, 0.0),
                est_xy=(0.02, 0.0),
                goal_xy=(0.0, 0.0),
                gt_available=False,
            ),
        ],
    )
    row = rows[0]
    assert row["GT Available"] == "0"
    assert float(row["True Distance To Goal (m)"]) == -1.0
    assert row["False Success"] == "-1"
