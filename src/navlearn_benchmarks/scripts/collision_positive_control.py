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

"""Positive control for collision detection: drive into a wall on purpose.

Why this exists
    Collision count is reported per goal, and across the retired campaign it was
    frequently zero. Zero is indistinguishable between "the robot never hit anything" and
    "the detector never worked", and nothing in that campaign ever established which. A
    safety metric that has never been observed to fire is not evidence of safety.

    This drives the robot forward into an obstacle deliberately and asserts the count
    increments. It is the run that makes every subsequent zero meaningful.

What it exercises
    The whole path, not just the threshold arithmetic: it publishes an episode START so
    trajectory_metric begins accumulating, drives until the LiDAR minimum range crosses
    the configured threshold, publishes an episode END, and reads collision_count off the
    resulting TrajectoryMetric message. A failure anywhere in that chain fails the test.

Safety
    Drives at a low fixed speed and stops on the first threshold crossing, on timeout, or
    on any exception. This intentionally collides a simulated robot with a wall; do not
    run it on hardware.

Usage
    Terminal 1:  ros2 launch bumperbot_bringup simulated_robot.launch.py \\
                     world_name:=small_house controller:=rpp planner:=smac2d \\
                     localizer:=amcl_tuned
    Terminal 2:  ros2 run navlearn_benchmarks collision_positive_control.py

Exit status
    0 if the detector fired, 1 otherwise. Prints a verdict either way.
"""

import argparse
import math
import sys
import time
import uuid

import rclpy
from geometry_msgs.msg import TwistStamped
from navlearn_msgs.msg import EpisodeEvent, TrajectoryMetric
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from unique_identifier_msgs.msg import UUID


def sensor_qos(depth=10):
    """QoS matching a sensor publisher: best-effort, volatile, keep-last."""
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


class CollisionPositiveControl(Node):
    """Drives forward until the collision threshold trips, then reports what was counted."""

    def __init__(self, args):
        super().__init__("collision_positive_control")
        self.args = args
        self.min_range = math.inf
        self.scan_seen = False
        self.metric = None
        self.goal_id = UUID(uuid=list(uuid.uuid4().bytes))

        self.create_subscription(LaserScan, args.scan_topic, self._on_scan, sensor_qos())
        self.create_subscription(
            TrajectoryMetric, args.trajectory_metric_topic, self._on_metric, 10
        )
        self.cmd_pub = self.create_publisher(TwistStamped, args.cmd_vel_topic, 10)
        self.episode_pub = self.create_publisher(EpisodeEvent, args.episode_topic, 10)

    def _on_scan(self, msg):
        valid = [
            r for r in msg.ranges
            if not math.isnan(r) and not math.isinf(r) and msg.range_min <= r <= msg.range_max
        ]
        if valid:
            self.min_range = min(valid)
            self.scan_seen = True

    def _on_metric(self, msg):
        if bytes(msg.goal_id.uuid) == bytes(self.goal_id.uuid):
            self.metric = msg

    def _drive(self, speed):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = speed
        self.cmd_pub.publish(msg)

    def _publish_episode(self, state, result=EpisodeEvent.RESULT_NA):
        ev = EpisodeEvent()
        ev.header.stamp = self.get_clock().now().to_msg()
        ev.state = state
        ev.result = result
        ev.goal_id = self.goal_id
        ev.start_pose.pose.orientation.w = 1.0
        ev.goal_pose.pose.orientation.w = 1.0
        self.episode_pub.publish(ev)

    def _spin(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def run(self):
        """Execute the control and return True if the detector fired."""
        print("Waiting for LiDAR...")
        deadline = time.monotonic() + 30
        while not self.scan_seen and time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.scan_seen:
            print(f"FAIL: no messages on {self.args.scan_topic}. Is the sim running?")
            return False

        print(f"  initial minimum range: {self.min_range:.3f} m")
        if self.min_range <= self.args.threshold:
            print(
                f"FAIL: already within {self.args.threshold} m of an obstacle. Move the "
                "robot to open space first, or this proves nothing about driving into one."
            )
            return False

        self._publish_episode(EpisodeEvent.START)
        self._spin(1.0)

        print(f"Driving forward at {self.args.speed} m/s until range <= "
              f"{self.args.threshold} m...")
        tripped = False
        deadline = time.monotonic() + self.args.timeout
        try:
            while time.monotonic() < deadline and rclpy.ok():
                self._drive(self.args.speed)
                rclpy.spin_once(self, timeout_sec=0.05)
                if self.min_range <= self.args.threshold:
                    tripped = True
                    print(f"  threshold crossed at {self.min_range:.3f} m")
                    break
        finally:
            # Stop on every path, including exceptions and timeout.
            for _ in range(10):
                self._drive(0.0)
                self._spin(0.05)

        if not tripped:
            print(
                f"FAIL: drove for {self.args.timeout} s without the range falling to "
                f"{self.args.threshold} m (closest {self.min_range:.3f} m). The robot may "
                "be facing open space, or cmd_vel is not reaching it."
            )
            self._publish_episode(EpisodeEvent.END, EpisodeEvent.RESULT_FAILED)
            return False

        self._publish_episode(EpisodeEvent.END, EpisodeEvent.RESULT_FAILED)
        print("Waiting for the trajectory metric...")
        deadline = time.monotonic() + 15
        while self.metric is None and time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.metric is None:
            print(
                f"FAIL: no TrajectoryMetric on {self.args.trajectory_metric_topic}. The "
                "threshold was crossed but the metrics pipeline produced nothing, so a "
                "real collision would also go unrecorded."
            )
            return False

        count = self.metric.collision_count
        print(f"  reported collision_count = {count}")
        print(f"  reported min_clearance_m = {self.metric.min_clearance_m:.3f}")

        if count < 1:
            print(
                "FAIL: the robot demonstrably reached the collision threshold and the "
                "counter stayed at zero. Every zero in the campaign would be meaningless."
            )
            return False

        print("PASS: collision detector fires on a deliberate contact. Zeros elsewhere in "
              "the campaign now carry information.")
        return True


def main():
    """Parse arguments, run the control, and report a verdict."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--cmd-vel-topic", default="/bumperbot_controller/cmd_vel")
    parser.add_argument("--episode-topic", default="/navlearn/episode_event")
    parser.add_argument("--trajectory-metric-topic", default="/navlearn/trajectory_metric")
    parser.add_argument("--threshold", type=float, default=0.15,
                        help="Must match trajectory_metric's collision_scan_threshold_m")
    parser.add_argument("--speed", type=float, default=0.10, help="Forward speed [m/s]")
    parser.add_argument("--timeout", type=float, default=60.0)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = CollisionPositiveControl(args)
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
