import os 

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo, EmitEvent
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.events import Shutdown
from ament_index_python.packages import get_package_share_directory

def _validate_nav2_profile(context, *args, **kwargs):
    profile = LaunchConfiguration("nav2_profile").perform(context)

    profiles_root = os.path.join(
        get_package_share_directory("bumperbot_navigation"),
        "config", "nav2_profiles"
    )
    profile_dir = os.path.join(profiles_root, profile)

    if not os.path.isdir(profile_dir):
        return [
            LogInfo(msg=f"[nav2] invalid nav2_profile='{profile}'. Not found: {profile_dir}"),
            EmitEvent(event=Shutdown(reason="Invalid nav2_profile"))
        ]
    return []

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    
    nav2_profile_arg = DeclareLaunchArgument("nav2_profile", default_value="baseline")
    nav2_profile = LaunchConfiguration("nav2_profile")

    nav2_profiles_directory_guard = OpaqueFunction(function=_validate_nav2_profile)

    lifecycle_nodes = ["controller_server", "planner_server", "smoother_server", "bt_navigator", "behavior_server"]

    controller_yaml = PathJoinSubstitution([
        FindPackageShare("bumperbot_navigation"),
        "config", "nav2_profiles", nav2_profile,
        "controller_server.yaml"
        ]
    )

    planner_yaml = PathJoinSubstitution([
        FindPackageShare("bumperbot_navigation"),
        "config", "nav2_profiles", nav2_profile,
        "planner_server.yaml"
        ]
    )

    smoother_yaml = PathJoinSubstitution([
        FindPackageShare("bumperbot_navigation"),
        "config", "nav2_profiles", nav2_profile,
        "smoother_server.yaml"
        ]
    )

    bt_navigator_yaml = PathJoinSubstitution([
        FindPackageShare("bumperbot_navigation"),
        "config", "nav2_profiles", nav2_profile,
        "bt_navigator.yaml"
        ]
    )

    behavior_yaml = PathJoinSubstitution([
        FindPackageShare("bumperbot_navigation"),
        "config", "nav2_profiles", nav2_profile,
        "behavior_server.yaml"
        ]
    )

    nav2_controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[
            controller_yaml,
            {"use_sim_time": use_sim_time}
        ]
    )

    nav2_planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[
            planner_yaml,
            {"use_sim_time": use_sim_time}
        ]
    )

    nav2_smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[
            smoother_yaml,
            {"use_sim_time": use_sim_time}
        ]
    )

    nav2_bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[
            bt_navigator_yaml,
            {"use_sim_time": use_sim_time}
        ]
    )

    nav2_behaviors = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[
            behavior_yaml,
            {"use_sim_time": use_sim_time}
        ]
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        nav2_profile_arg,
        nav2_profiles_directory_guard,
        nav2_controller_server,
        nav2_planner_server,
        nav2_smoother_server,
        nav2_lifecycle_manager,
        nav2_bt_navigator,
        nav2_behaviors
    ])