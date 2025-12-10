import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, RegisterEventHandler, EmitEvent
from launch.substitutions import LaunchConfiguration
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    
    episode_start_delay_arg = DeclareLaunchArgument("episode_start_delay", default_value="2.0")
    episode_start_delay = LaunchConfiguration("episode_start_delay")

    use_sim_time_arg = DeclareLaunchArgument("use_sim_time",default_value="True")
    use_sim_time = LaunchConfiguration("use_sim_time")

    goals_num_arg = DeclareLaunchArgument("goals_num",default_value="4")
    goals_num = LaunchConfiguration("goals_num")

    goal_source_arg = DeclareLaunchArgument("goal_source",default_value="map_random")
    goal_source = LaunchConfiguration("goal_source")

    csv_path_arg = DeclareLaunchArgument("csv_path", default_value="/home/mihirmk/robot_ws/src/navlearn_benchmarks/benchmark_reports/runs")
    csv_path = LaunchConfiguration("csv_path")

    json_path_arg = DeclareLaunchArgument("json_path", default_value="/home/mihirmk/robot_ws/src/navlearn_benchmarks/benchmark_reports/runs")
    json_path = LaunchConfiguration("json_path")

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
            {"use_sim_time": use_sim_time,
             "csv_path" : csv_path,
             "json_path" : json_path}
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
        executable='episode_manager_node',   
        name='episode_manager',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory("navlearn_benchmarks"), 
                "config", 
                "episode_manager_customGoals.yaml"
            ),
            {"use_sim_time": use_sim_time,
             "goals_num" : goals_num,
             "goal_source" : goal_source}
        ]
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
        episode_start_delay_arg,
        use_sim_time_arg,
        goals_num_arg, 
        goal_source_arg,
        csv_path_arg,
        json_path_arg,
        compiler,            
        control_metric,      
        trajectory_metric,
        delayed_episode_manager,
        shutdown_on_episode_done,
    ])
