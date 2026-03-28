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

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    TimerAction,
    RegisterEventHandler,
    EmitEvent,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def apply_perturbation_presets(context, *args, **kwargs):
    """
    Map perturbation_level string to concrete parameter values.

    Always overrides kidnap_distance_m, kidnap_max_distance_m,
    bad_init_lin_range_m, and bad_init_yaw_range_rad based on the
    perturbation_level. If a user wants custom values, they should
    NOT set perturbation_level and instead pass params directly.
    """
    level = context.launch_configurations.get("perturbation_level", "medium")

    presets = {
        "easy": {
            "bad_init_lin_range_m": "0.25",
            "bad_init_yaw_range_rad": "0.087",
            "kidnap_distance_m": "0.3",
            "kidnap_max_distance_m": "0.5",
        },
        "medium": {
            "bad_init_lin_range_m": "0.50",
            "bad_init_yaw_range_rad": "0.262",
            "kidnap_distance_m": "0.8",
            "kidnap_max_distance_m": "1.2",
        },
        "hard": {
            "bad_init_lin_range_m": "1.00",
            "bad_init_yaw_range_rad": "0.524",
            "kidnap_distance_m": "1.5",
            "kidnap_max_distance_m": "2.5",
        },
        "extreme": {
            "bad_init_lin_range_m": "2.00",
            "bad_init_yaw_range_rad": "0.785",
            "kidnap_distance_m": "2.5",
            "kidnap_max_distance_m": "4.0",
        },
    }

    if level in presets:
        for key, val in presets[level].items():
            context.launch_configurations[key] = val

    return []


def generate_launch_description():
    episode_start_delay_arg = DeclareLaunchArgument(
        "episode_start_delay", default_value="2.0"
    )
    episode_start_delay = LaunchConfiguration("episode_start_delay")

    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="True")
    use_sim_time = LaunchConfiguration("use_sim_time")

    goal_seed_arg = DeclareLaunchArgument("goal_seed", default_value="42")
    goal_seed = LaunchConfiguration("goal_seed")

    goals_num_arg = DeclareLaunchArgument("goals_num", default_value="15")
    goals_num = LaunchConfiguration("goals_num")

    goal_source_arg = DeclareLaunchArgument("goal_source", default_value="map_random")
    goal_source = LaunchConfiguration("goal_source")

    csv_path_arg = DeclareLaunchArgument(
        "csv_path",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("navlearn_benchmarks"),
                "benchmark_reports",
                "navlearn_metrics.csv",
            ]
        ),
    )
    csv_path = LaunchConfiguration("csv_path")

    json_path_arg = DeclareLaunchArgument(
        "json_path",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("navlearn_benchmarks"),
                "benchmark_reports",
                "navlearn_run_report.json",
            ]
        ),
    )
    json_path = LaunchConfiguration("json_path")

    episode_manager_config_arg = DeclareLaunchArgument(
        "episode_manager_config", default_value="episode_manager_1mSquare.yaml"
    )
    episode_manager_config = LaunchConfiguration("episode_manager_config")

    bad_init_test_arg = DeclareLaunchArgument("bad_init_test", default_value="false")
    bad_init_test = LaunchConfiguration("bad_init_test")

    kidnap_enabled_arg = DeclareLaunchArgument("kidnap_enabled", default_value="true")
    kidnap_test = LaunchConfiguration("kidnap_enabled")

    kidnap_max_distance_m_arg = DeclareLaunchArgument(
        "kidnap_max_distance_m", default_value="1.1"
    )
    kidnap_max_distance_m = LaunchConfiguration("kidnap_max_distance_m")

    kidnap_distance_m_arg = DeclareLaunchArgument(
        "kidnap_distance_m", default_value="0.20"
    )
    kidnap_distance_m = LaunchConfiguration("kidnap_distance_m")

    bad_init_lin_range_m_arg = DeclareLaunchArgument(
        "bad_init_lin_range_m",
        default_value="0.50",
        description="TTC position perturbation half-range [m]",
    )
    bad_init_lin_range_m = LaunchConfiguration("bad_init_lin_range_m")

    bad_init_yaw_range_rad_arg = DeclareLaunchArgument(
        "bad_init_yaw_range_rad",
        default_value="0.262",
        description="TTC yaw perturbation half-range [rad]",
    )
    bad_init_yaw_range_rad = LaunchConfiguration("bad_init_yaw_range_rad")

    world_name_arg = DeclareLaunchArgument(
        "world_name", default_value="small_house", description="Gazebo world name"
    )

    nav2_profile_arg = DeclareLaunchArgument(
        "nav2_profile",
        default_value="aggressive",
        description="Nav2 parameter profile: baseline | aggressive",
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="ROS 2 log level: debug | info | warn | error",
    )

    collision_scan_threshold_m_arg = DeclareLaunchArgument(
        "collision_scan_threshold_m",
        default_value="0.15",
        description="LiDAR range threshold for scan-based collision detection [m]. "
        "Default 0.15. Set to 0.55 for collision counter integration test.",
    )
    collision_scan_threshold_m = LaunchConfiguration("collision_scan_threshold_m")

    perturbation_level_arg = DeclareLaunchArgument(
        "perturbation_level",
        default_value="medium",
        description="Perturbation preset for TTC/TTR: easy | medium | hard | extreme",
    )

    goal_min_distance_m_arg = DeclareLaunchArgument(
        "goal_min_distance_m",
        default_value="0.0",
        description="Min Euclidean distance between consecutive goals [m]. "
        "Set to 4.0 for TTC/TTR experiments.",
    )
    goal_min_distance_m = LaunchConfiguration("goal_min_distance_m")

    ttc_timeout_sec_arg = DeclareLaunchArgument(
        "ttc_timeout_sec",
        default_value="10.0",
        description="TTC convergence timeout passed to localization_metrics [s]",
    )
    ttc_timeout_sec = LaunchConfiguration("ttc_timeout_sec")

    ttr_timeout_sec_arg = DeclareLaunchArgument(
        "ttr_timeout_sec",
        default_value="10.0",
        description="TTR recovery timeout passed to localization_metrics [s]",
    )
    ttr_timeout_sec = LaunchConfiguration("ttr_timeout_sec")

    recovery_timeout_sec_arg = DeclareLaunchArgument(
        "recovery_timeout_sec",
        default_value="15.0",
        description="Recovery convergence timeout for episode_manager [s]",
    )
    recovery_timeout_sec = LaunchConfiguration("recovery_timeout_sec")

    episode_manager_config_path = PathJoinSubstitution(
        [FindPackageShare("navlearn_benchmarks"), "config", episode_manager_config]
    )

    localization_metrics_yaml = PathJoinSubstitution(
        [
            FindPackageShare("navlearn_localization_eval"),
            "config",
            "localization_metrics.yaml",
        ]
    )

    gz_set_pose_yaml = PathJoinSubstitution(
        [
            FindPackageShare("navlearn_localization_eval"),
            "config",
            "gz_set_pose_server.yaml",
        ]
    )

    compiler = Node(
        package="navlearn_benchmarks",
        executable="metrics_compiler",
        name="metrics_compiler",
        output="log",
        parameters=[
            os.path.join(
                get_package_share_directory("navlearn_benchmarks"),
                "config",
                "metrics_compiler.yaml",
            ),
            {
                "use_sim_time": use_sim_time,
                "csv_path": csv_path,
                "json_path": json_path,
            },
        ],
    )

    control_metric = Node(
        package="navlearn_benchmarks",
        executable="control_metric",
        name="control_metric",
        output="log",
        parameters=[
            os.path.join(
                get_package_share_directory("navlearn_benchmarks"),
                "config",
                "control_metric.yaml",
            ),
            {"use_sim_time": use_sim_time},
        ],
    )

    trajectory_metric = Node(
        package="navlearn_benchmarks",
        executable="trajectory_metric",
        name="trajectory_metric",
        output="log",
        parameters=[
            os.path.join(
                get_package_share_directory("navlearn_benchmarks"),
                "config",
                "trajectory_metric.yaml",
            ),
            {
                "use_sim_time": use_sim_time,
                "collision_scan_threshold_m": collision_scan_threshold_m,
            },
        ],
    )

    episode_manager = Node(
        package="navlearn_benchmarks",
        executable="episode_manager_node",
        name="episode_manager",
        output="log",
        parameters=[
            episode_manager_config_path,
            {
                "use_sim_time": use_sim_time,
                "goal_seed": goal_seed,
                "goals_num": goals_num,
                "goal_source": goal_source,
                "bad_init_test": bad_init_test,
                "kidnap_enabled": kidnap_test,
                "kidnap_max_distance_m": kidnap_max_distance_m,
                "kidnap_distance_m": kidnap_distance_m,
                "goal_min_distance_m": goal_min_distance_m,
                "recovery_timeout_sec": recovery_timeout_sec,
                "bad_init_lin_range_m": bad_init_lin_range_m,
                "bad_init_yaw_range_rad": bad_init_yaw_range_rad,
                "kidnap_max_distance_m": kidnap_max_distance_m,
            },
        ],
    )

    world_name = LaunchConfiguration("world_name")

    # ros_gz_bridge: bridge Ignition contact sensor → ROS 2 ros_gz_interfaces/msg/Contacts
    # Topic format: /world/{world}/model/bumperbot/link/base_link/sensor/contact_sensor/contact
    contact_bridge_topic = [
        TextSubstitution(text="/world/"),
        world_name,
        TextSubstitution(
            text="/model/bumperbot/link/base_link/sensor/contact_sensor/contact"
            "@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts"
        ),
    ]

    collision_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="collision_contact_bridge",
        arguments=[contact_bridge_topic],
        output="log",
    )

    collision_monitor = Node(
        package="navlearn_benchmarks",
        executable="collision_monitor",
        name="collision_monitor",
        output="log",
        parameters=[
            os.path.join(
                get_package_share_directory("navlearn_benchmarks"),
                "config",
                "collision_monitor.yaml",
            ),
            # Override contacts_topic to match the current world_name at runtime
            {
                "contacts_topic": [
                    TextSubstitution(text="/world/"),
                    world_name,
                    TextSubstitution(
                        text="/model/bumperbot/link/base_link/sensor/contact_sensor/contact"
                    ),
                ]
            },
        ],
    )

    ground_truth_publisher = Node(
        package="navlearn_localization_eval",
        executable="ground_truth_publisher",
        name="ground_truth_publisher",
        output="log",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("navlearn_localization_eval"),
                    "config",
                    "ground_truth_publisher.yaml",
                ]
            ),
            {"use_sim_time": use_sim_time},
        ],
    )

    localization_metrics_node = Node(
        condition=IfCondition(
            PythonExpression(
                ["'", bad_init_test, "' == 'true' or '", kidnap_test, "' == 'true'"]
            )
        ),
        package="navlearn_localization_eval",
        executable="localization_metrics",
        name="localization_metrics",
        output="log",
        parameters=[
            localization_metrics_yaml,
            {
                "use_sim_time": use_sim_time,
                "bad_init_test": bad_init_test,
                "kidnap_test": kidnap_test,
                "ttc_timeout_sec": ttc_timeout_sec,
                "ttr_timeout_sec": ttr_timeout_sec,
            },
        ],
    )

    gz_set_pose_server_node = Node(
        condition=IfCondition(PythonExpression(["'", kidnap_test, "' == 'true'"])),
        package="navlearn_localization_eval",
        executable="gz_set_pose_server",
        name="gz_set_pose_server",
        output="log",
        parameters=[
            gz_set_pose_yaml,
            {"use_sim_time": use_sim_time, "world_name": world_name},
        ],
    )

    delayed_episode_manager = TimerAction(
        period=episode_start_delay,
        actions=[episode_manager],
    )

    shutdown_on_episode_done = RegisterEventHandler(
        OnProcessExit(
            target_action=episode_manager,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )

    return LaunchDescription(
        [
            bad_init_test_arg,
            kidnap_enabled_arg,
            kidnap_max_distance_m_arg,
            kidnap_distance_m_arg,
            bad_init_lin_range_m_arg,
            bad_init_yaw_range_rad_arg,
            episode_start_delay_arg,
            use_sim_time_arg,
            goal_seed_arg,
            goals_num_arg,
            goal_source_arg,
            csv_path_arg,
            json_path_arg,
            episode_manager_config_arg,
            world_name_arg,
            nav2_profile_arg,
            log_level_arg,
            collision_scan_threshold_m_arg,
            perturbation_level_arg,
            OpaqueFunction(function=apply_perturbation_presets),
            goal_min_distance_m_arg,
            ttc_timeout_sec_arg,
            ttr_timeout_sec_arg,
            recovery_timeout_sec_arg,
            compiler,
            control_metric,
            trajectory_metric,
            collision_bridge,
            collision_monitor,
            ground_truth_publisher,
            localization_metrics_node,
            gz_set_pose_server_node,
            delayed_episode_manager,
            shutdown_on_episode_done,
        ]
    )
