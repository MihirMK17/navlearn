import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bumperbot_description = get_package_share_directory("bumperbot_description")

    model_arg = DeclareLaunchArgument(
        name="model", default_value=os.path.join(
                bumperbot_description, "urdf", "bumperbot.urdf.xacro"
            ),
        description="Absolute path to robot urdf file"
    )

    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="empty")

    # Sensor-rate leg. Starves the LiDAR stream to a commanded rate by decimating what is
    # published, rather than by lowering the simulated sensor's own rate: that would also
    # change what Gazebo simulates and how the bridge is loaded, so a rate cell would
    # differ from baseline in more than one variable. Decimating leaves the simulation
    # identical and varies exactly one thing — how many scans reach the stack.
    #
    # Default 0.0 means the governor is not launched at all and the scan path is
    # byte-identical to what it was before this argument existed.
    scan_rate_hz_arg = DeclareLaunchArgument(
        name="scan_rate_hz", default_value="0.0",
        description="Starve the LiDAR to this rate for the sensor-rate leg. "
                    "0 disables the governor and leaves the scan path unchanged.",
    )
    scan_rate_hz = LaunchConfiguration("scan_rate_hz")

    # Headless operation. Gazebo has always been launched with `-r` (run) and no `-s`
    # (server only), so every benchmark run so far rendered a GUI — roughly 29 hours of
    # simulation drawing frames on an Intel iGPU that nobody was watching, competing for
    # CPU with the stack being measured.
    #
    # Default is false so existing behaviour is unchanged and the setting is opt-in; the
    # campaign passes headless:=true. The speedup is measured on the pilot cell rather
    # than assumed, and whichever mode a run used is recorded, because rendering load
    # affects the compute numbers and cells run in different modes are not comparable.
    headless_arg = DeclareLaunchArgument(
        name="headless",
        default_value="false",
        description="Run Gazebo without the GUI (adds -s). Campaign runs use true.",
    )

    # PRIME render offload. prime-select on-demand keeps the display on the Intel iGPU
    # and a process is given the discrete NVIDIA card only if it asks for it, so without
    # these variables the GPU LiDAR renders on integrated graphics regardless of what
    # nvidia-smi reports — which is how every campaign before 2026-08-06 was collected
    # (warehouse 0.23 Hz, bookstore 3.3-6.2 Hz vs the sensor's 10 Hz). They are set on
    # the launch process rather than the caller's shell so they reach the `ign gazebo`
    # child that ros_gz_sim spawns; the GLX pair covers the GUI path and the EGL one the
    # headless path. gpu:=false reproduces the Intel backend for A/B comparison runs.
    gpu_arg = DeclareLaunchArgument(
        name="gpu",
        default_value="true",
        description="Render on the NVIDIA dGPU via PRIME offload. Set false to "
                    "reproduce the Intel-iGPU backend pre-2026-08-06 legs used.",
    )
    gpu_on = IfCondition(PythonExpression(
        ["'", LaunchConfiguration("gpu"), "'.lower() in ('true', '1', 'yes')"]))
    gpu_env = [
        SetEnvironmentVariable("__NV_PRIME_RENDER_OFFLOAD", "1", condition=gpu_on),
        SetEnvironmentVariable("__GLX_VENDOR_LIBRARY_NAME", "nvidia", condition=gpu_on),
        SetEnvironmentVariable(
            "__EGL_VENDOR_LIBRARY_FILENAMES",
            "/usr/share/glvnd/egl_vendor.d/10_nvidia.json",
            condition=gpu_on,
        ),
    ]

    # World lookup searches this package's worlds/ first, then every entry on
    # GZ_SIM_RESOURCE_PATH for worlds/<name>.world — so an asset package (e.g.
    # navlearn_assets, which registers itself on that variable through an ament
    # environment hook) can contribute worlds without this package knowing it exists.
    # A name that resolves nowhere shuts the launch down by name instead of handing
    # Gazebo a path that does not exist.
    def _resolve_world(context, *args, **kwargs):
        name = LaunchConfiguration("world_name").perform(context)
        candidates = [os.path.join(bumperbot_description, "worlds", f"{name}.world")]
        for entry in os.environ.get("GZ_SIM_RESOURCE_PATH", "").split(pathsep):
            if entry:
                candidates.append(os.path.join(entry, "worlds", f"{name}.world"))
        for candidate in candidates:
            if os.path.isfile(candidate):
                return [SetLaunchConfiguration("world_file", candidate)]
        return [
            LogInfo(msg=f"[gazebo.launch] world '{name}.world' not found in "
                        f"{len(candidates)} search locations"),
            EmitEvent(event=Shutdown(reason=f"unknown world '{name}'")),
        ]

    world_resolver = OpaqueFunction(function=_resolve_world)
    world_path = LaunchConfiguration("world_file")

    model_path = str(Path(bumperbot_description).parent.resolve())
    model_path += pathsep + os.path.join(get_package_share_directory("bumperbot_description"), 'models')

    # APPEND to the ambient value rather than overwrite it. Overwriting silently
    # discarded whatever the environment already carried — exactly the channel an
    # asset package's environment hook uses to make its models resolvable.
    ambient_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    if ambient_resource_path:
        model_path += pathsep + ambient_resource_path

    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        model_path
        )

    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"

    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=",
            is_ignition
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments={
                    # -s is appended only when headless is requested. Written as a
                    # conditional expression rather than two launch entries so the world
                    # path and verbosity cannot drift between the two modes.
                    "gz_args": PythonExpression([
                        "'", world_path, " -v 4 -r'",
                        " + (' -s' if '", LaunchConfiguration("headless"),
                        "'.lower() in ('true', '1', 'yes') else '')",
                    ])
                }.items()
             )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "bumperbot"],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/world/default/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        ],
        remappings=[
            ('/imu', '/imu/out'),
            ('/scan', '/scan_unfiltered'),
            ("/world/default/dynamic_pose/info", "/tf_gt"),
        ],
        parameters=[{'use_sim_time': True}],
        output="screen"
    )

    # The scan-rate governor that used to sit between the bridge and the sanitizer is
    # composed by bumperbot_bringup now: it is a navlearn_benchmarks node, and a robot
    # description package that launches benchmark instrumentation has its layering
    # backwards. The scan_rate_hz argument stays declared here because the sanitizer's
    # input-topic switch below still keys on it.
    scan_sanitizer_node = Node(
        package="bumperbot_description",
        executable="scan_sanitizer",
        parameters=[{
            "use_sim_time" : True,
            # Reads the governed stream when the governor is running, the raw one
            # otherwise. Everything downstream of the sanitizer is untouched either way.
            "input_topic": PythonExpression([
                "'scan_governed' if float('", scan_rate_hz, "') > 0.0 "
                "else 'scan_unfiltered'"
            ]),
            "output_topic": "scan",
            "min_valid_range": 0.12,
            "max_valid_range": 12.0,
            "mode": "to_inf"
        }],
        output="screen"
    )

    return LaunchDescription([  
        model_arg,
        world_name_arg,
        scan_rate_hz_arg,
        headless_arg,
        gpu_arg,
        *gpu_env,     # must precede `gazebo` — the env has to exist before ign spawns
        world_resolver,   # must precede `gazebo` — it sets world_file
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
        scan_sanitizer_node
    ])
