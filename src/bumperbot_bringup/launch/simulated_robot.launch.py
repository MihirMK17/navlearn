import importlib.util
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import UnlessCondition, IfCondition
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# Shared with bumperbot_navigation, which owns config/nav2_stack/. Loaded by path because
# launch files are not importable as a package. Legal localizer names come from the
# filesystem, so this file holds no list that could drift from what is on disk.
_stack_spec_path = os.path.join(
    get_package_share_directory("bumperbot_navigation"), "launch", "stack_spec.py"
)
_spec = importlib.util.spec_from_file_location("navlearn_stack_spec", _stack_spec_path)
stack_spec = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(stack_spec)


def _preflight_failure(message):
    """Return launch actions that log a preflight failure and shut the launch down."""
    return [
        LogInfo(msg=f"[nav2_stack] PREFLIGHT FAILED: {message}"),
        EmitEvent(event=Shutdown(reason=f"nav2_stack preflight: {message}")),
    ]


def _resolve_localizer(context, *args, **kwargs):
    """Turn the localizer name into the amcl_config path the localization launch expects.

    AMCL is launched by bumperbot_localization, which takes a full YAML path rather than a
    fragment name, so the nav2_stack/localizers/ fragment is resolved here and handed over.
    An explicitly supplied amcl_config always wins, which keeps the escape hatch open for
    reproducing an archived run against its original config.
    """
    if LaunchConfiguration("amcl_config").perform(context):
        return []

    stack = stack_spec.stack_dir(get_package_share_directory("bumperbot_navigation"))
    localizer = LaunchConfiguration("localizer").perform(context)

    problem = stack_spec.validate_selection(stack, "localizer", localizer)
    if problem:
        return _preflight_failure(problem)

    return [
        SetLaunchConfiguration(
            "amcl_config", stack_spec.fragment_path(stack, "localizer", localizer)
        )
    ]


def generate_launch_description():

    use_slam_arg = DeclareLaunchArgument(
        name="use_slam",
        default_value="false"
    )

    use_slam = LaunchConfiguration("use_slam")

    # Stack composition. See bumperbot_navigation/config/nav2_stack/ and
    # bumperbot_navigation/launch/navigation.launch.py for what each slot loads.
    controller_arg = DeclareLaunchArgument(
        "controller",
        default_value="rpp",
        description="Local controller: rpp | dwb | mppi"
    )
    controller_name = LaunchConfiguration("controller")

    planner_arg = DeclareLaunchArgument(
        "planner",
        default_value="smac2d",
        description="Global planner: smac2d | navfn | thetastar"
    )
    planner_name = LaunchConfiguration("planner")

    ablation_arg = DeclareLaunchArgument(
        "ablation",
        default_value="none",
        description="Single-variable override: none | high_tolerance | fixed_bt | high_vx"
    )
    ablation_name = LaunchConfiguration("ablation")

    localizer_arg = DeclareLaunchArgument(
        "localizer",
        default_value="amcl_tuned",
        description="Localizer fragment; legal values are the files in nav2_stack/localizers/"
    )

    stack_spec_out_arg = DeclareLaunchArgument(
        "stack_spec_out",
        default_value=os.path.join(
            os.path.expanduser("~"), ".navlearn", "current_stack_spec.json"
        ),
        description=(
            "Path for the JSON provenance record of the composed stack. The benchmark "
            "harness reads it and copies it into each run directory."
        )
    )
    stack_spec_out = LaunchConfiguration("stack_spec_out")

    world_name_arg = DeclareLaunchArgument("world_name", default_value="small_house")
    world_name = LaunchConfiguration("world_name")

    amcl_config_arg = DeclareLaunchArgument(
        name="amcl_config",
        default_value="",
        description=(
            "Explicit path to an AMCL config YAML. Leave empty to derive it from the "
            "localizer argument; set it only to reproduce an archived run against its "
            "original config."
        )
    )
    amcl_config = LaunchConfiguration("amcl_config")
    localizer_resolver = OpaqueFunction(function=_resolve_localizer)

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_description"),
            "launch",
            "gazebo.launch.py"
        ),
        launch_arguments={"world_name": world_name}.items(),
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "False"
        }.items(),
    )
    
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True"
        }.items()
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        launch_arguments={"amcl_config": amcl_config}.items(),
        condition=UnlessCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(use_slam)
    )

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_navigation"),
            "launch",
            "navigation.launch.py"
        ),
        launch_arguments={
            "controller": controller_name,
            "planner": planner_name,
            "ablation": ablation_name,
            "stack_spec_out": stack_spec_out,
        }.items()
    )

    safety_stop = Node(
        package="bumperbot_utils",
        executable="safety_stop",
        output="screen"
    )

    rviz_localization = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
            get_package_share_directory("bumperbot_localization"),
            "rviz",
            "global_localization.rviz"
        )],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=UnlessCondition(use_slam)
    )

    rviz_slam = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
            get_package_share_directory("bumperbot_mapping"),
            "rviz",
            "slam.rviz"
        )],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_slam)
    )

    return LaunchDescription([
        use_slam_arg,
        controller_arg,
        planner_arg,
        ablation_arg,
        localizer_arg,
        stack_spec_out_arg,
        world_name_arg,
        amcl_config_arg,
        localizer_resolver,   # must precede `localization` — it sets amcl_config
        gazebo,
        controller,
        joystick,
        # safety_stop,
        localization,
        slam,
        navigation,
        rviz_localization,
        rviz_slam
    ])
