from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 包路径
    pkg_share = FindPackageShare('diff_dyn_car')
    pkg_path = get_package_share_directory('diff_dyn_car')

    # ================== world：空世界（带 ROS 插件） ==================
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


    # ================== 机器人描述 ==================
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

    # ================== gazebo 本体 ==================
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

    # ================== spawn 小车（固定在原点） ==================
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

    # ================== ros2_control 控制器 ==================
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

    # 你要是想顺便起手柄力矩节点，可以在这里再加一个 udp_joy_to_effort 的 Node

    return LaunchDescription([
        declare_world,
        gazebo,
        rsp_node,
        spawn_car,
        spawner_jsb,
        spawner_effort,
    ])
