from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = FindPackageShare('diff_dyn_car')
    pkg_path = get_package_share_directory('diff_dyn_car')

    # ⚠️ 这里用 pkg_path（真实字符串路径），不能用 pkg_share
    obstacle_sdf = os.path.join(pkg_path, 'models', 'obstacle_box.sdf')

    # world 参数
    world = LaunchConfiguration('world')

    # ========== 1. xacro → robot_description ==========
    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([
            pkg_share,
            'urdf',
            'diff_dyn_car.urdf.xacro'
        ])
    ])

    # ========== 2. robot_state_publisher ==========
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }]
    )

    # ========== 3. spawn 机器人 ==========
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'diff_dyn_car',
            '-topic', 'robot_description'
            '-x', '1.0', '-y', '0.0', '-z', '0.035',
        ],
        output='screen',
    )

    # ========== 3.5 spawn 障碍物 box ==========
    spawn_obstacle = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'obstacle_box',
            '-file',   obstacle_sdf,
            '-x', '1.0', '-y', '0.0', '-z', '0.035',
        ],
        output='screen'
    )

    # ========== 4. 你的 effort 控制节点 ==========
    diff_effort_controller = Node(
        package='diff_dyn_car',
        executable='diff_effort_controller',
        name='diff_effort_controller',
        output='screen'
    )

    # ========== 5. gazebo 本体 ==========
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

    # ========== 6. ros2_control 控制器 ==========
    spawner_jsb = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    spawner_effort = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['wheel_effort_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # world 声明
    declare_world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            get_package_share_directory('gazebo_ros'),
            'worlds',
            'empty.world'
        ),
        description='Gazebo world file'
    )

    return LaunchDescription([
        declare_world,
        gazebo,
        rsp_node,
        spawn,
        spawn_obstacle,
        spawner_jsb,
        spawner_effort,
        diff_effort_controller
    ])
