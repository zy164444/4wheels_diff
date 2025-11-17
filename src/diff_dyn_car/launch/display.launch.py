from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():

    pkg_path = get_package_share_directory("botlab_diff_description")
    xacro_file = os.path.join(pkg_path, "urdf", "diff_drive_car.urdf.xacro")

    # 处理 xacro → URDF
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description}]
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher"
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2"
        )
    ])
