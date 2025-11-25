from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = FindPackageShare('diff_dyn_car')
    pkg_path = get_package_share_directory('diff_dyn_car')

    # ================== world ==================
    empty_world = os.path.join(
        get_package_share_directory('diff_dyn_car'),
        'worlds',
        'empty_with_state.world'
    )
    world = LaunchConfiguration('world')

    declare_world = DeclareLaunchArgument(
        'world',
        default_value=empty_world,
        description='Gazebo world file (empty world with ROS state plugin)'
    )

    # ================== robot description ==================
    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([
            pkg_share,
            'urdf',
            'diff_dyn_car.urdf.xacro'
        ])
    ])

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # ================== gazebo ==================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={'world': world}.items()
    )

    # ================== spawn robot ==================
    spawn_car = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'diff_dyn_car',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.05',
            '-Y', '0.0'
        ],
        output='screen'
    )

    # ================== ros2_control ==================
    spawner_jsb = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager'
        ],
        output='screen'
    )

    spawner_effort = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=[
            'wheel_effort_controller',
            '--controller-manager', '/controller_manager'
        ],
        output='screen'
    )

    # ================== 加你自己的两个节点 ==================

    # 力矩控制（手柄→力矩）
    udp_joy_to_effort = Node(
        package='diff_dyn_car',
        executable='udp_joy_to_effort',
        name='udp_joy_to_effort',
        output='screen'
    )

    # 记录轨迹（CSV logger）
    car_state_logger = Node(
        package='diff_dyn_car',
        executable='car_state_logger',
        name='car_state_logger',
        output='screen'
    )

    # ================== return ==================
    return LaunchDescription([
        declare_world,
        gazebo,
        rsp_node,
        spawn_car,
        spawner_jsb,
        spawner_effort,

        # ★ 新加的两个节点
        udp_joy_to_effort,
        car_state_logger,
    ])
