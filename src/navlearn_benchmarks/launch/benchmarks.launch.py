import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    
    episode_start_delay_arg = DeclareLaunchArgument(
        "episode_start_delay",
        default_value="2.0"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True"
    )

    episode_start_delay = LaunchConfiguration("episode_start_delay")

    use_sim_time = LaunchConfiguration("use_sim_time")

    compiler = Node(
        package='navlearn_benchmarks',
        executable='metrics_compiler',
        name='metrics_compiler',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory("navlearn_benchmarks"), 
                "config", 
                "metrics_compiler.yaml"
            ),
            {"use_sim_time": use_sim_time}
        ]
    )

    control_metric = Node(
        package='navlearn_benchmarks',
        executable='control_metric',
        name='control_metric',
        output='screen',
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
        output='screen',
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
        executable='episode_manager_node',   # rclcpp_components EXECUTABLE
        name='episode_manager',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory("navlearn_benchmarks"), 
                "config", 
                "episode_manager_stressGoals.yaml"
            ),
            {"use_sim_time": use_sim_time}
        ]
    )

    delayed_episode_manager = TimerAction(
        period=episode_start_delay,
        actions=[episode_manager],
    )

    return LaunchDescription([
        episode_start_delay_arg,
        use_sim_time_arg,
        compiler,            # 1) compiler first
        control_metric,      # 2) metric nodes
        trajectory_metric,
        delayed_episode_manager,  # 3) episode manager last (delayed)
    ])
