import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():

    bumperbot_description = get_package_share_directory("bumperbot_description")

    robot_description = ParameterValue(Command([
            "xacro ",
            os.path.join(bumperbot_description, "urdf", "bumperbot.urdf.xacro"),
            " is_sim:=False"
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description,
             "use_sim_time": False},
             os.path.join(
                 get_package_share_directory("bumperbot_controller"),
                 "config",
                 "bumperbot_controllers.yaml"
             )
        ]
    )

    imu_driver_node = Node(
        package="bumperbot_firmware",
        executable="mpu6050_driver.py"
    )
        
    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager,
        imu_driver_node
    ])