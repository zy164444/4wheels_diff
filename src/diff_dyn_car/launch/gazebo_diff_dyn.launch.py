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

    # world 参数
    world = LaunchConfiguration('world')

    # ========== 1. xacro → robot_description ==========
    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([
            pkg_share,
            'urdf',
            'diff_dyn_car.urdf.xacro'   # 按你的文件名修改
        ])
    ])

    # ========== 2. Node: robot_state_publisher ==========
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }]
    )

    # ========== 3. spawn gazebo 里的机器人 ==========
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'diff_dyn_car',
            '-topic', 'robot_description'
        ],
        output='screen',
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

    # ========== 6. 自动 spawn ros2_control 控制器 ==========
    # joint_state_broadcaster
    spawner_jsb = Node(
        package='controller_manager',
        executable='spawner.py',   # 在你机器上是 spawner.py
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # wheel_effort_controller
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

    # return
    return LaunchDescription([
        declare_world,
        gazebo,
        rsp_node,
        spawn,
        spawner_jsb,          # ⭐ 自动加载 joint_state_broadcaster
        spawner_effort,       # ⭐ 自动加载 wheel_effort_controller
        diff_effort_controller
    ])
