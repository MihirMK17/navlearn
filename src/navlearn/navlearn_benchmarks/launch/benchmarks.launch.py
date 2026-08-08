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

import json
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
from launch.actions import SetLaunchConfiguration, LogInfo
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

DEFAULT_STACK_SPEC = os.path.join(
    os.path.expanduser("~"), ".navlearn", "current_stack_spec.json"
)


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


def resolve_speed_limit(context, *args, **kwargs):
    """Set control_metric's v_max from the stack the bringup actually composed.

    Saturation fraction is the share of commands at the controller's velocity ceiling, so
    it is only meaningful against the ceiling that was actually in force. control_metric
    defaulted to 0.25 m/s and no launch file ever overrode it, while the profiles ran at
    0.26 to 0.45 — so the metric was computed against a limit six of seven profiles did
    not use, and the ablation that deliberately raises the speed to 0.45 would have been
    scored against 0.25.

    The value is read from the stack spec rather than hardcoded here because it depends on
    which controller and which ablation the bringup composed, and each controller plugin
    names its speed limit differently. Reading it back means the metric cannot disagree
    with the stack even if a fragment changes.

    A missing or unreadable spec is fatal. A silently wrong ceiling produces a
    plausible-looking saturation number that is simply incorrect, which is precisely the
    class of defect the rebuild exists to eliminate.
    """
    spec_path = os.path.expanduser(
        context.launch_configurations.get("stack_spec", DEFAULT_STACK_SPEC)
    )

    def fail(message):
        return [
            LogInfo(msg=f"[benchmarks] PREFLIGHT FAILED: {message}"),
            EmitEvent(event=Shutdown(reason=f"benchmarks preflight: {message}")),
        ]

    if not os.path.isfile(spec_path):
        return fail(
            f"no stack spec at {spec_path}. control_metric cannot determine the velocity "
            "ceiling, so saturation fraction would be computed against a guess. Start the "
            "bringup first, or pass stack_spec:=<path>."
        )

    try:
        with open(spec_path) as handle:
            spec = json.load(handle)
    except (OSError, json.JSONDecodeError) as exc:
        return fail(f"stack spec at {spec_path} is unreadable: {exc}")

    v_max = spec.get("identity", {}).get("max_linear_velocity")
    if v_max is None:
        return fail(
            f"stack spec at {spec_path} carries no max_linear_velocity. It was written by "
            "an older navigation.launch.py; restart the bringup to regenerate it."
        )

    return [
        SetLaunchConfiguration("resolved_v_max", str(float(v_max))),
        LogInfo(
            msg=f"[benchmarks] control_metric v_max={v_max} m/s "
                f"(from {spec['selection'].get('controller', '?')}"
                f"/{spec['selection'].get('ablation', '?')})"
        ),
    ]


def generate_launch_description():
    episode_start_delay_arg = DeclareLaunchArgument(
        "episode_start_delay", default_value="2.0"
    )

    stack_spec_arg = DeclareLaunchArgument(
        "stack_spec",
        default_value=DEFAULT_STACK_SPEC,
        description="Stack provenance record written by the bringup; supplies v_max.",
    )
    episode_start_delay = LaunchConfiguration("episode_start_delay")

    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="True")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # One seed for the whole campaign; run_index distinguishes episodes within it.
    # These replace goal_seed, which seeded only goal placement — the perturbation
    # seeds were fixed constants that no launch argument ever reached.
    campaign_seed_arg = DeclareLaunchArgument("campaign_seed", default_value="42")
    campaign_seed = LaunchConfiguration("campaign_seed")
    run_index_arg = DeclareLaunchArgument("run_index", default_value="0")
    run_index = LaunchConfiguration("run_index")

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

    # localization_metrics writes a LONG-format CSV (metric name/value rows) and was
    # handed the same csv_path as metrics_compiler's WIDE format. Nothing noticed for a
    # month because the node was segfaulting at startup in every bad-init cell; the
    # 2026-07-31 rebuild revived it and the two formats interleaved into one file
    # (leg1 mppi_v2, salvaged by column count). Distinct file, derived from csv_path so
    # the pair always lands in the same run directory.
    loc_csv_path = PythonExpression(
        ["'", csv_path, "'.replace('.csv', '_localization.csv')"])

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

    # Perturbation is OPT-IN. This defaulted to true, so a run launched without thinking
    # about it got TTR kidnap injection — and the campaign's clean leg is three cells whose
    # entire purpose is the absence of perturbation. A default that silently applies the
    # independent variable is how a cell ends up labelled 'clean' while being a TTR cell,
    # which is precisely the class of mislabelling the 2026-07-25 audit was cleaning up.
    # Verified on the first pilot: a cell run as 'clean' had kidnap active throughout.
    kidnap_enabled_arg = DeclareLaunchArgument("kidnap_enabled", default_value="false")
    kidnap_test = LaunchConfiguration("kidnap_enabled")

    kidnap_max_distance_m_arg = DeclareLaunchArgument(
        "kidnap_max_distance_m", default_value="1.1"
    )
    kidnap_max_distance_m = LaunchConfiguration("kidnap_max_distance_m")

    kidnap_distance_m_arg = DeclareLaunchArgument(
        "kidnap_distance_m", default_value="0.20"
    )
    kidnap_distance_m = LaunchConfiguration("kidnap_distance_m")

    # Continuous severity. In "curve" mode each goal draws its own displacement from the
    # campaign seed over [min, max], so the sweep is the experiment and the curve is the
    # reported result rather than a discarded calibration input. The banded presets above
    # are ignored in this mode. Default "fixed" so no existing cell changes meaning.
    kidnap_magnitude_mode_arg = DeclareLaunchArgument(
        "kidnap_magnitude_mode", default_value="fixed",
        choices=["fixed", "curve"],
    )
    kidnap_magnitude_mode = LaunchConfiguration("kidnap_magnitude_mode")

    kidnap_magnitude_min_m_arg = DeclareLaunchArgument(
        "kidnap_magnitude_min_m", default_value="0.3"
    )
    kidnap_magnitude_min_m = LaunchConfiguration("kidnap_magnitude_min_m")

    kidnap_magnitude_max_m_arg = DeclareLaunchArgument(
        "kidnap_magnitude_max_m", default_value="3.0"
    )
    kidnap_magnitude_max_m = LaunchConfiguration("kidnap_magnitude_max_m")

    # Log by default: a displacement sweep spans an order of magnitude, and uniform-in-
    # metres would leave the onset of degradation sparsely covered.
    kidnap_magnitude_scale_arg = DeclareLaunchArgument(
        "kidnap_magnitude_scale", default_value="log", choices=["log", "linear"],
    )
    kidnap_magnitude_scale = LaunchConfiguration("kidnap_magnitude_scale")

    kidnap_magnitude_band_arg = DeclareLaunchArgument(
        "kidnap_magnitude_band", default_value="0.05"
    )
    kidnap_magnitude_band = LaunchConfiguration("kidnap_magnitude_band")

    # The yaw-curve leg (PROTOCOL A3): displacement pinned to zero, rotation swept.
    # These MUST be declared here and forwarded below — a launch argument this file does
    # not declare is silently discarded by ros2 launch, the node falls back to its
    # "preserve" default, and the campaign collects unrotated goals under a yaw-curve
    # label. The 2026-08-04 yaw pilot caught exactly that.
    kidnap_yaw_mode_arg = DeclareLaunchArgument(
        "kidnap_yaw_mode", default_value="preserve", choices=["preserve", "curve"],
    )
    kidnap_yaw_mode = LaunchConfiguration("kidnap_yaw_mode")

    # Rectangles barred from goal placement and kidnap sampling, as flat groups of four
    # [xmin, xmax, ymin, ymax]. World-specific by nature: the default is the small_house
    # rectangle this was hardcoded to before 2026-08-05, which every world inherited.
    # A campaign in another world passes its own list, or '[]' for none.
    exclusion_zones_arg = DeclareLaunchArgument(
        "exclusion_zones", default_value="-2.4, 0.7, 3.3, 5.4",
        description="Barred rectangles as 'xmin,xmax,ymin,ymax,...'; empty for none. "
                    "A string rather than a list because ROS 2 cannot carry an empty "
                    "typed array, and 'no zones' is a normal setting.",
    )
    exclusion_zones = LaunchConfiguration("exclusion_zones")

    kidnap_yaw_min_rad_arg = DeclareLaunchArgument(
        "kidnap_yaw_min_rad", default_value="0.0"
    )
    kidnap_yaw_min_rad = LaunchConfiguration("kidnap_yaw_min_rad")

    kidnap_yaw_max_rad_arg = DeclareLaunchArgument(
        "kidnap_yaw_max_rad", default_value="3.141592653589793"
    )
    kidnap_yaw_max_rad = LaunchConfiguration("kidnap_yaw_max_rad")

    # The TTC leg sweeps the same way, over bad-initialization displacement. Separate
    # range and seed stream from the kidnap sweep, so a cell can sweep one perturbation
    # without inheriting the other's bounds and enabling both cannot correlate them.
    bad_init_magnitude_mode_arg = DeclareLaunchArgument(
        "bad_init_magnitude_mode", default_value="fixed", choices=["fixed", "curve"],
    )
    bad_init_magnitude_mode = LaunchConfiguration("bad_init_magnitude_mode")

    bad_init_magnitude_min_m_arg = DeclareLaunchArgument(
        "bad_init_magnitude_min_m", default_value="0.05"
    )
    bad_init_magnitude_min_m = LaunchConfiguration("bad_init_magnitude_min_m")

    bad_init_magnitude_max_m_arg = DeclareLaunchArgument(
        "bad_init_magnitude_max_m", default_value="2.0"
    )
    bad_init_magnitude_max_m = LaunchConfiguration("bad_init_magnitude_max_m")

    bad_init_magnitude_scale_arg = DeclareLaunchArgument(
        "bad_init_magnitude_scale", default_value="log", choices=["log", "linear"],
    )
    bad_init_magnitude_scale = LaunchConfiguration("bad_init_magnitude_scale")

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

    # NOTE: there is deliberately no nav2_profile argument here.
    #
    # One was declared until 2026-07-28 and never consumed — this launch file starts the
    # metric nodes, not the navigation stack, so the value was accepted and discarded.
    # multi_run_harness.py forwarded `--profile` into it and logged the name as though it
    # described the run, which meant a result could be labelled with one stack while a
    # different one was actually running in the bringup terminal, undetectably.
    #
    # The stack is now selected on the bringup and recorded by navigation.launch.py into a
    # JSON spec that the harness copies into each run directory. See
    # bumperbot_navigation/launch/stack_spec.py.

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

    # Exactly one collision detector may count. Both used to increment the same counter,
    # so a collision that tripped the scan threshold and the contact sensor was counted
    # twice. "scan" is the campaign default: no contact sensor required, identical on any
    # simulator and on hardware, and it is what collision_positive_control.py exercises.
    collision_source_arg = DeclareLaunchArgument(
        "collision_source",
        default_value="scan",
        description="Which detector counts collisions: scan | topic",
    )
    collision_source = LaunchConfiguration("collision_source")
    use_contact_sensor = PythonExpression(["'", collision_source, "' == 'topic'"])

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
            FindPackageShare("navlearn_benchmarks"),
            "config",
            "localization_metrics.yaml",
        ]
    )

    gz_set_pose_yaml = PathJoinSubstitution(
        [
            FindPackageShare("navlearn_benchmarks"),
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
            # Injected last so it wins over control_metric.yaml. Resolved from the stack
            # the bringup actually composed — see resolve_speed_limit for why a static
            # value here is wrong.
            {
                "v_max": ParameterValue(
                    LaunchConfiguration("resolved_v_max"), value_type=float
                )
            },
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
                "collision_source": collision_source,
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
                "campaign_seed": campaign_seed,
                "run_index": run_index,
                "goals_num": goals_num,
                "goal_source": goal_source,
                "bad_init_test": bad_init_test,
                "kidnap_enabled": kidnap_test,
                "kidnap_max_distance_m": kidnap_max_distance_m,
                "kidnap_distance_m": kidnap_distance_m,
                "kidnap_magnitude_mode": kidnap_magnitude_mode,
                "kidnap_magnitude_min_m": ParameterValue(
                    kidnap_magnitude_min_m, value_type=float),
                "kidnap_magnitude_max_m": ParameterValue(
                    kidnap_magnitude_max_m, value_type=float),
                "kidnap_magnitude_scale": kidnap_magnitude_scale,
                "kidnap_magnitude_band": ParameterValue(
                    kidnap_magnitude_band, value_type=float),
                "kidnap_yaw_mode": kidnap_yaw_mode,
                "exclusion_zones": ParameterValue(exclusion_zones, value_type=str),
                "kidnap_yaw_min_rad": ParameterValue(
                    kidnap_yaw_min_rad, value_type=float),
                "kidnap_yaw_max_rad": ParameterValue(
                    kidnap_yaw_max_rad, value_type=float),
                "bad_init_magnitude_mode": bad_init_magnitude_mode,
                "bad_init_magnitude_min_m": ParameterValue(
                    bad_init_magnitude_min_m, value_type=float),
                "bad_init_magnitude_max_m": ParameterValue(
                    bad_init_magnitude_max_m, value_type=float),
                "bad_init_magnitude_scale": bad_init_magnitude_scale,
                "goal_min_distance_m": goal_min_distance_m,
                "recovery_timeout_sec": recovery_timeout_sec,
                "bad_init_lin_range_m": bad_init_lin_range_m,
                "bad_init_yaw_range_rad": bad_init_yaw_range_rad,
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
        condition=IfCondition(use_contact_sensor),
    )

    collision_monitor = Node(
        package="navlearn_benchmarks",
        executable="collision_monitor",
        name="collision_monitor",
        output="log",
        condition=IfCondition(use_contact_sensor),
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
        package="navlearn_benchmarks",
        executable="ground_truth_publisher",
        name="ground_truth_publisher",
        output="log",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("navlearn_benchmarks"),
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
        package="navlearn_benchmarks",
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
                "csv_path": loc_csv_path,
            },
        ],
    )

    gz_set_pose_server_node = Node(
        condition=IfCondition(PythonExpression(["'", kidnap_test, "' == 'true'"])),
        package="navlearn_benchmarks",
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
            kidnap_magnitude_mode_arg,
            kidnap_magnitude_min_m_arg,
            kidnap_magnitude_max_m_arg,
            kidnap_magnitude_scale_arg,
            kidnap_magnitude_band_arg,
            kidnap_yaw_mode_arg,
            exclusion_zones_arg,
            kidnap_yaw_min_rad_arg,
            kidnap_yaw_max_rad_arg,
            bad_init_magnitude_mode_arg,
            bad_init_magnitude_min_m_arg,
            bad_init_magnitude_max_m_arg,
            bad_init_magnitude_scale_arg,
            bad_init_lin_range_m_arg,
            bad_init_yaw_range_rad_arg,
            episode_start_delay_arg,
            use_sim_time_arg,
            campaign_seed_arg,
            run_index_arg,
            stack_spec_arg,
            # Must precede control_metric: it sets the v_max that node reads.
            OpaqueFunction(function=resolve_speed_limit),
            goals_num_arg,
            goal_source_arg,
            csv_path_arg,
            json_path_arg,
            episode_manager_config_arg,
            world_name_arg,
            log_level_arg,
            collision_scan_threshold_m_arg,
            collision_source_arg,
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
