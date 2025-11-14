from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import os


def generate_launch_description():
    pkg_name = "botlab_diff_description"  # <<< 如果你的包名不是这个，这里要改
    pkg_share = get_package_share_directory(pkg_name)

    # 1. 用 xacro 生成 robot_description
    urdf_file = os.path.join(pkg_share, "urdf", "diff_drive_car.urdf.xacro")
    robot_description = Command(["xacro ", urdf_file])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    # 2. 启动 Gazebo 空世界
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gazebo.launch.py",
            )
        )
    )

    # 3. 把机器人从 robot_description 这个 topic 里 spawn 出来
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "diff_car",
            "-z",
            "0.04",  # 大约等于轮半径，保证轮子落地不埋/不悬空
        ],
        output="screen",
    )

    # 4. 起两个 ros2_control 控制器
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    wheel_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["wheel_velocity_controller"],
        output="screen",
    )

    return LaunchDescription(
        [
            gazebo,
            robot_state_publisher,
            spawn_entity,
            joint_state_broadcaster_spawner,
            wheel_velocity_controller_spawner,
        ]
    )
