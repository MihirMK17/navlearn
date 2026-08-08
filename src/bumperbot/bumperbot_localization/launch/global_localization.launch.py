from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    
    use_sim_time_arg= DeclareLaunchArgument(
        name="use_sim_time", 
        default_value="true"
    )

    map_name_arg = DeclareLaunchArgument(
        name="map_name",
        default_value="small_house"
    )

    amcl_config_arg = DeclareLaunchArgument(
        name="amcl_config",
        default_value= os.path.join(
            get_package_share_directory("bumperbot_localization"),
            "config", 
            "amcl.yaml"
        )
    )

    map_name = LaunchConfiguration("map_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    amcl_config = LaunchConfiguration("amcl_config")

    lifeycle_nodes = ["map_server", "amcl"]

    map_path = PathJoinSubstitution([
        get_package_share_directory("bumperbot_mapping"),
        "maps",
        map_name,
        "map.yaml"
    ])

    # map_yaml takes a full path, so a campaign can point at a map that does not live in
    # bumperbot_mapping -- the environments beyond small_house are campaign assets and
    # are versioned with the analysis that consumes them, not with the robot.
    #
    # Its default IS the path map_name used to build, so every existing invocation
    # resolves to exactly the file it did before and nothing that does not pass this
    # argument can change behaviour.
    map_yaml_arg = DeclareLaunchArgument(
        name="map_yaml",
        default_value=map_path,
        description="Full path to a map_server YAML. Overrides map_name."
    )
    map_yaml = LaunchConfiguration("map_yaml")

    nav2_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        parameters=[
            {"yaml_filename": map_yaml},
            {"use_sim_time": use_sim_time}
        ]
    )

    nav2_amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[
            amcl_config,
            {"use_sim_time": use_sim_time}]
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"node_names": lifeycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        amcl_config_arg,
        map_name_arg,
        map_yaml_arg,
        nav2_map_server,
        nav2_amcl,
        nav2_lifecycle_manager
    ])