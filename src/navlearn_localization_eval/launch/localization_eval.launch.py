from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    use_sim_time = LaunchConfiguration('use_sim_time')

    csv_path_arg = DeclareLaunchArgument('csv_path', default_value='/home/mihirmk/robot_ws/src/navlearn_localization_eval/evaluations/tuning/localization_evaluation_test_tune_t3.csv')
    csv_path = LaunchConfiguration('csv_path')

    bad_init_test_arg = DeclareLaunchArgument('bad_init_test', default_value='false')
    bad_init_test = LaunchConfiguration('bad_init_test')

    kidnap_test_arg = DeclareLaunchArgument('kidnap_test', default_value='true')
    kidnap_test = LaunchConfiguration('kidnap_test')

    log_level_arg = DeclareLaunchArgument('log_level', default_value='info')
    log_level = LaunchConfiguration('log_level')

    world_name_arg = DeclareLaunchArgument('world_name', default_value='default')
    world_name = LaunchConfiguration('world_name')

    metrics_yaml = PathJoinSubstitution([
        FindPackageShare('navlearn_localization_eval'),
        'config',
        'localization_metrics.yaml'
    ])

    set_pose_yaml = PathJoinSubstitution([
        FindPackageShare('navlearn_localization_eval'),
        'config',
        'gz_set_pose_server.yaml'
    ])

    ground_truth_yaml = PathJoinSubstitution([
        FindPackageShare('navlearn_localization_eval'),
        'config',
        'ground_truth_publisher.yaml'
    ])

    gt_node = Node(
        package='navlearn_localization_eval',
        executable='ground_truth_publisher',
        name='ground_truth_publisher',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[
            ground_truth_yaml,
            {'use_sim_time': use_sim_time}
        ],
    )

    set_pose_node = Node(
        package='navlearn_localization_eval',
        executable='gz_set_pose_server',
        name='gz_set_pose_server',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[
            set_pose_yaml,
            {'use_sim_time': use_sim_time,
             'world_name': world_name}
        ],
    )

    metrics_node = Node(
        package='navlearn_localization_eval',
        executable='localization_metrics',
        name='localization_metrics',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[
            metrics_yaml,
            {'use_sim_time': use_sim_time,
             'bad_init_test': bad_init_test,
             'kidnap_test': kidnap_test,
             'csv_path' : csv_path}
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        csv_path_arg,
        bad_init_test_arg,
        kidnap_test_arg,
        log_level_arg,
        world_name_arg,
        gt_node,
        set_pose_node,
        metrics_node,
    ])
