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

"""Integration tests for the scan-rate governor node.

What it is for
    Leg 6 of the campaign starves the LiDAR to 10, 5, 2 and 1 Hz and asks whether recovery
    from a kidnap degrades faster than tracking does. That prediction only means anything
    if the starving itself is exact and reproducible, and if the scans that do get through
    are the ones the sensor produced rather than something the governor rebuilt.

What these protect that the unit tests cannot
    `test_scan_decimator.cpp` proves the keep RULE. These prove the NODE: that it is wired
    to the right topics, that it forwards the message unmodified — including the header
    stamp, which AMCL uses for its TF lookup, so a rewritten stamp would silently shift
    every scan in time — and that a misconfigured rate is refused at startup instead of
    producing a cell that ran at a rate nobody asked for.
"""

import itertools
import os
import subprocess
import time

import pytest

rclpy = pytest.importorskip("rclpy", reason="ROS 2 Python runtime not available")

from sensor_msgs.msg import LaserScan  # noqa: E402

IN_TOPIC = "/navlearn_test/scan_in"
OUT_TOPIC = "/navlearn_test/scan_out"

_node_counter = itertools.count()


@pytest.fixture(scope="module")
def sourced():
    """Skip the module unless the built workspace is on the environment."""
    if "navlearn_benchmarks" not in os.environ.get("AMENT_PREFIX_PATH", ""):
        pytest.skip("workspace not sourced; run after 'source install/setup.bash'")


def _governor_executable():
    """Locate the scan_rate_governor binary in the installed workspace."""
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        candidate = os.path.join(
            prefix, "lib", "navlearn_benchmarks", "scan_rate_governor"
        )
        if os.path.isfile(candidate):
            return candidate
    pytest.skip("scan_rate_governor executable not found in AMENT_PREFIX_PATH")


def _scan(index):
    """Build a LaserScan whose contents identify it uniquely.

    range_min carries the index so a received scan can be traced back to the one that was
    sent. Reusing a real field rather than adding a sidecar keeps the test honest about
    what the governor forwards.
    """
    msg = LaserScan()
    msg.header.frame_id = "test_laser"
    msg.header.stamp.sec = 1000 + index
    msg.header.stamp.nanosec = 12345 * (index + 1)
    msg.angle_min = -1.5
    msg.angle_max = 1.5
    msg.angle_increment = 0.1
    msg.time_increment = 0.001
    msg.scan_time = 0.1
    msg.range_min = float(index)
    msg.range_max = 12.0
    msg.ranges = [1.0 + 0.5 * index, 2.0, 3.0]
    msg.intensities = [10.0, 20.0, 30.0]
    return msg


def _run_governor(native_hz, target_hz, n_scans, extra_params=()):
    """Publish n_scans into the governor and return the scans it forwarded."""
    node_name = f"scan_rate_governor_test_{next(_node_counter)}"
    args = [
        _governor_executable(),
        "--ros-args",
        "-r",
        f"__node:={node_name}",
        "-p",
        f"input_topic:={IN_TOPIC}",
        "-p",
        f"output_topic:={OUT_TOPIC}",
        "-p",
        f"native_rate_hz:={native_hz}",
        "-p",
        f"target_rate_hz:={target_hz}",
    ]
    for key, value in extra_params:
        args += ["-p", f"{key}:={value}"]

    proc = subprocess.Popen(
        args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
    )

    rclpy.init()
    node = rclpy.create_node(f"{node_name}_driver")
    pub = node.create_publisher(LaserScan, IN_TOPIC, 10)
    received = []
    node.create_subscription(LaserScan, OUT_TOPIC, received.append, 10)

    try:
        # Wait for the governor's subscription and publication to appear rather than
        # sleeping a fixed interval; discovery time varies and a fixed wait is how these
        # tests turn flaky.
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            if (
                pub.get_subscription_count() > 0
                and node.count_publishers(OUT_TOPIC) > 0
            ):
                break
            if proc.poll() is not None:
                return None, proc
            rclpy.spin_once(node, timeout_sec=0.1)
        else:
            pytest.fail("governor never connected; is the workspace sourced?")

        for i in range(n_scans):
            pub.publish(_scan(i))
            for _ in range(4):
                rclpy.spin_once(node, timeout_sec=0.02)

        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        return received, proc
    finally:
        node.destroy_node()
        rclpy.shutdown()
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()


def test_pass_through_forwards_every_scan(sourced):
    """A cell at the native rate must not drop anything."""
    received, _ = _run_governor(10.0, 10.0, 12)
    assert [int(m.range_min) for m in received] == list(range(12))


def test_decimation_keeps_every_nth_scan(sourced):
    """A 2 Hz cell on a 10 Hz sensor forwards scans 0, 5, 10 and nothing else."""
    received, _ = _run_governor(10.0, 2.0, 12)
    assert [int(m.range_min) for m in received] == [0, 5, 10]


def test_one_hz_cell_keeps_one_scan_in_ten(sourced):
    """The most starved campaign rate, checked end to end through the node."""
    received, _ = _run_governor(10.0, 1.0, 21)
    assert [int(m.range_min) for m in received] == [0, 10, 20]


def test_forwarded_scan_is_bit_identical_to_the_input(sourced):
    """The governor selects scans; it must never author one.

    The header stamp is the sharp edge: AMCL looks up the sensor-to-map transform at that
    stamp, so a governor that restamped on forward would shift every measurement in time
    and quietly change the experiment it was built to run.
    """
    received, _ = _run_governor(10.0, 5.0, 4)
    assert received, "nothing forwarded"
    first, expected = received[0], _scan(0)
    assert first.header.stamp.sec == expected.header.stamp.sec
    assert first.header.stamp.nanosec == expected.header.stamp.nanosec
    assert first.header.frame_id == expected.header.frame_id
    assert first.angle_min == pytest.approx(expected.angle_min)
    assert first.angle_increment == pytest.approx(expected.angle_increment)
    assert first.scan_time == pytest.approx(expected.scan_time)
    assert first.range_max == pytest.approx(expected.range_max)
    assert list(first.ranges) == pytest.approx(list(expected.ranges))
    assert list(first.intensities) == pytest.approx(list(expected.intensities))


def test_target_above_native_rate_refuses_to_start(sourced):
    """Decimation cannot create scans, so the cell must die instead of running unstarved.

    Silently delivering 10 Hz for a cell labelled 20 Hz would pool an unstarved run with
    genuinely starved ones, and nothing downstream could tell.
    """
    result = subprocess.run(
        [
            _governor_executable(),
            "--ros-args",
            "-p",
            f"input_topic:={IN_TOPIC}",
            "-p",
            f"output_topic:={OUT_TOPIC}",
            "-p",
            "native_rate_hz:=10.0",
            "-p",
            "target_rate_hz:=20.0",
        ],
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode != 0
    assert "exceeds" in (result.stdout + result.stderr).lower()


def test_unachievable_rate_refuses_to_start_by_default(sourced):
    """10 Hz cannot deliver 3 Hz; without an explicit opt-in the cell must not run.

    The delivered rate would be 3.33 Hz. Recording that cell as '3 Hz' puts a number in a
    results table that the run never ran at, so the default is to stop.
    """
    result = subprocess.run(
        [
            _governor_executable(),
            "--ros-args",
            "-p",
            f"input_topic:={IN_TOPIC}",
            "-p",
            f"output_topic:={OUT_TOPIC}",
            "-p",
            "native_rate_hz:=10.0",
            "-p",
            "target_rate_hz:=3.0",
        ],
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode != 0
    combined = (result.stdout + result.stderr).lower()
    assert "3.33" in combined or "achievable" in combined


def test_unachievable_rate_runs_when_explicitly_allowed(sourced):
    """With allow_approximate_rate, the run proceeds and the delivered rate is announced."""
    received, proc = _run_governor(
        10.0, 3.0, 10, extra_params=[("allow_approximate_rate", "true")]
    )
    assert received is not None, "governor exited despite allow_approximate_rate"
    assert [int(m.range_min) for m in received] == [0, 3, 6, 9]
