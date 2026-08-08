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

"""Collision positive control with the metric node it needs.

The control asserts that a deliberate wall contact increments collision_count, which
requires trajectory_metric to be running — and trajectory_metric belongs to
benchmarks.launch.py, not to the robot bringup. Running the script against a bare bringup
fails with "no TrajectoryMetric", which is the script correctly reporting that the
pipeline under test was absent.

The full benchmark launch cannot be used instead: it also starts episode_manager, which
would issue its own navigation goals and fight the control for the robot.

So this launch brings up exactly the node under test and nothing else, then runs the
control against it. Parameters mirror what benchmarks.launch.py passes, so a pass here is
evidence about the configuration the campaign will actually use.

Usage
    Terminal 1:  ros2 launch bumperbot_bringup simulated_robot.launch.py \\
                     world_name:=small_house controller:=rpp planner:=smac2d \\
                     localizer:=amcl_tuned headless:=true use_rviz:=false
    Terminal 2:  ros2 launch navlearn_analysis collision_positive_control.launch.py

The robot ends up against a wall. Restart the bringup before running a benchmark cell so
the campaign starts from a clean spawn.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import EmitEvent
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start trajectory_metric, then drive the robot into a wall and check the count."""
    threshold_arg = DeclareLaunchArgument(
        "collision_scan_threshold_m",
        default_value="0.15",
        description="Must match the value benchmarks.launch.py uses.",
    )
    threshold = LaunchConfiguration("collision_scan_threshold_m")

    speed_arg = DeclareLaunchArgument("speed", default_value="0.10")
    timeout_arg = DeclareLaunchArgument("timeout", default_value="60.0")

    trajectory_metric = Node(
        package="navlearn_benchmarks",
        executable="trajectory_metric",
        name="trajectory_metric",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("navlearn_benchmarks"),
                "config",
                "trajectory_metric.yaml",
            ),
            {
                "use_sim_time": True,
                "collision_scan_threshold_m": threshold,
                # Scan is the campaign default and the detector this control exercises.
                "collision_source": "scan",
            },
        ],
    )

    control = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "navlearn_analysis",
            "collision_positive_control",
            "--threshold",
            threshold,
            "--speed",
            LaunchConfiguration("speed"),
            "--timeout",
            LaunchConfiguration("timeout"),
            "--ros-args",
            "-p",
            "use_sim_time:=true",
        ],
        output="screen",
    )

    # The control's exit status is the verdict, so the launch ends when it does rather
    # than leaving trajectory_metric running for someone to notice and clean up.
    stop_when_done = RegisterEventHandler(
        OnProcessExit(
            target_action=control,
            on_exit=[EmitEvent(event=Shutdown(reason="positive control finished"))],
        )
    )

    return LaunchDescription(
        [
            threshold_arg,
            speed_arg,
            timeout_arg,
            trajectory_metric,
            control,
            stop_when_done,
        ]
    )
