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
    DeclareLaunchArgument, TimerAction, RegisterEventHandler, EmitEvent)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    episode_start_delay_arg = DeclareLaunchArgument(
        "episode_start_delay", default_value="2.0")
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
        default_value=PathJoinSubstitution([
            FindPackageShare("navlearn_benchmarks"),
            "benchmark_reports",
            "navlearn_metrics.csv"
        ])
    )
    csv_path = LaunchConfiguration("csv_path")

    json_path_arg = DeclareLaunchArgument(
        "json_path",
        default_value=PathJoinSubstitution([
            FindPackageShare("navlearn_benchmarks"),
            "benchmark_reports",
            "navlearn_run_report.json"
        ])
    )
    json_path = LaunchConfiguration("json_path")

    episode_manager_config_arg = DeclareLaunchArgument(
        "episode_manager_config",
        default_value="episode_manager_1mSquare.yaml")
    episode_manager_config = LaunchConfiguration("episode_manager_config")

    bad_init_test_arg = DeclareLaunchArgument('bad_init_test', default_value='false')
    bad_init_test = LaunchConfiguration('bad_init_test')

    kidnap_enabled_arg = DeclareLaunchArgument('kidnap_enabled', default_value='true')
    kidnap_test = LaunchConfiguration('kidnap_enabled')

    kidnap_max_distance_m_arg = DeclareLaunchArgument(
        'kidnap_max_distance_m', default_value='1.1')
    kidnap_max_distance_m = LaunchConfiguration('kidnap_max_distance_m')

    kidnap_distance_m_arg = DeclareLaunchArgument(
        'kidnap_distance_m', default_value='0.20')
    kidnap_distance_m = LaunchConfiguration('kidnap_distance_m')

    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value="small_house",
        description="Gazebo world name")

    nav2_profile_arg = DeclareLaunchArgument(
        "nav2_profile",
        default_value="aggressive",
        description="Nav2 parameter profile: baseline | aggressive")

    localization_eval_enabled_arg = DeclareLaunchArgument(
        "localization_eval_enabled",
        default_value="false",
        description="Enable localization_eval node alongside benchmarks")

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="ROS 2 log level: debug | info | warn | error")

    episode_manager_config_path = PathJoinSubstitution([
        FindPackageShare("navlearn_benchmarks"),
        "config",
        episode_manager_config
    ])

    compiler = Node(
        package='navlearn_benchmarks',
        executable='metrics_compiler',
        name='metrics_compiler',
        output='log',
        parameters=[
            os.path.join(
                get_package_share_directory("navlearn_benchmarks"),
                "config",
                "metrics_compiler.yaml"
            ),
            {"use_sim_time": use_sim_time,
             "csv_path": csv_path,
             "json_path": json_path}
        ]
    )

    control_metric = Node(
        package='navlearn_benchmarks',
        executable='control_metric',
        name='control_metric',
        output='log',
        parameters=[
            os.path.join(
                get_package_share_directory("navlearn_benchmarks"),
                "config",
                "control_metric.yaml"
            ),
            {"use_sim_time": use_sim_time}
        ]
    )

    trajectory_metric = Node(
        package='navlearn_benchmarks',
        executable='trajectory_metric',
        name='trajectory_metric',
        output='log',
        parameters=[
            os.path.join(
                get_package_share_directory("navlearn_benchmarks"),
                "config",
                "trajectory_metric.yaml"
            ),
            {"use_sim_time": use_sim_time}
        ]
    )

    episode_manager = Node(
        package='navlearn_benchmarks',
        executable='episode_manager_node',
        name='episode_manager',
        output='log',
        parameters=[
            episode_manager_config_path,
            {"use_sim_time": use_sim_time,
             "goal_seed": goal_seed,
             "goals_num": goals_num,
             "goal_source": goal_source,
             "bad_init_test": bad_init_test,
             "kidnap_enabled": kidnap_test,
             "kidnap_max_distance_m": kidnap_max_distance_m,
             "kidnap_distance_m": kidnap_distance_m}
        ]
    )

    world_name = LaunchConfiguration("world_name")

    # ros_gz_bridge: bridge Ignition contact sensor → ROS 2 ros_gz_interfaces/msg/Contacts
    # Topic format: /world/{world}/model/bumperbot/link/base_link/sensor/contact_sensor/contact
    contact_bridge_topic = [
        TextSubstitution(text='/world/'),
        world_name,
        TextSubstitution(
            text='/model/bumperbot/link/base_link/sensor/contact_sensor/contact'
                 '@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'),
    ]

    collision_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='collision_contact_bridge',
        arguments=[contact_bridge_topic],
        output='log',
    )

    collision_monitor = Node(
        package='navlearn_benchmarks',
        executable='collision_monitor',
        name='collision_monitor',
        output='log',
        parameters=[
            os.path.join(
                get_package_share_directory("navlearn_benchmarks"),
                "config",
                "collision_monitor.yaml"
            ),
            # Override contacts_topic to match the current world_name at runtime
            {"contacts_topic": [
                TextSubstitution(text='/world/'),
                world_name,
                TextSubstitution(
                    text='/model/bumperbot/link/base_link/sensor/contact_sensor/contact'),
            ]},
        ]
    )

    localization_eval_enabled = LaunchConfiguration("localization_eval_enabled")

    ground_truth_publisher = Node(
        package='navlearn_localization_eval',
        executable='ground_truth_publisher',
        name='ground_truth_publisher',
        output='log',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('navlearn_localization_eval'),
                'config',
                'ground_truth_publisher.yaml'
            ]),
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(localization_eval_enabled),
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

    return LaunchDescription([
        bad_init_test_arg,
        kidnap_enabled_arg,
        kidnap_max_distance_m_arg,
        kidnap_distance_m_arg,
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
        localization_eval_enabled_arg,
        log_level_arg,
        compiler,
        control_metric,
        trajectory_metric,
        collision_bridge,
        collision_monitor,
        ground_truth_publisher,
        delayed_episode_manager,
        shutdown_on_episode_done,
    ])
