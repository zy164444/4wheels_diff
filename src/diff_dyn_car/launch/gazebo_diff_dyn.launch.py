from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import random
import math


def generate_launch_description():
    # 包路径
    pkg_share = FindPackageShare('diff_dyn_car')
    pkg_path = get_package_share_directory('diff_dyn_car')

    obstacle_sdf = os.path.join(pkg_path, 'models', 'obstacle_box.sdf')
    goal_sdf = os.path.join(pkg_path, 'models', 'goal_frame.sdf')

    # world 文件
    world = LaunchConfiguration('world')

    # ================== 随机初始化部分 ==================

    # 在 [-1, 1] 范围内生成随机坐标
    def random_xy():
        return random.uniform(-1.0, 1.0), random.uniform(-1.0, 1.0)

    # 随机 yaw
    def random_yaw():
        return random.uniform(-math.pi, math.pi)

    SAFEDIST = 0.5

    # --- 先随机一个方块位置 ---
    box_x, box_y = random_xy()
    box_yaw = random_yaw()

    # --- 再随机小车：只要求与方块距离 > SAFEDIST ---
    while True:
        car_x, car_y = random_xy()
        d_car_box = math.hypot(car_x - box_x, car_y - box_y)
        if d_car_box > SAFEDIST:
            break
    car_yaw = random_yaw()

    # --- 再随机目标点：只要求与方块距离 > SAFEDIST ---
    while True:
        goal_x, goal_y = random_xy()
        d_goal_box = math.hypot(goal_x - box_x, goal_y - box_y)
        if d_goal_box > SAFEDIST:
            break
    goal_yaw = random_yaw()

    # 变成字符串给 spawn_entity.py 用
    car_x_str = f"{car_x:.2f}"
    car_y_str = f"{car_y:.2f}"
    car_z_str = "0.05"
    car_yaw_str = f"{car_yaw:.3f}"

    box_x_str = f"{box_x:.2f}"
    box_y_str = f"{box_y:.2f}"
    box_z_str = "0.045"
    box_yaw_str = f"{box_yaw:.3f}"

    goal_x_str = f"{goal_x:.2f}"
    goal_y_str = f"{goal_y:.2f}"
    goal_z_str = "0.03"
    goal_yaw_str = f"{goal_yaw:.3f}"

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

    # ================== spawn 小车 ==================
    spawn_car = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'diff_dyn_car',
            '-topic', 'robot_description',
            '-x', car_x_str,
            '-y', car_y_str,
            '-z', car_z_str,
            '-Y', car_yaw_str
        ],
        output='screen'
    )

    # ================== spawn 方块（立即） ==================
    spawn_box = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'obstacle_box',
            '-file', obstacle_sdf,
            '-x', box_x_str,
            '-y', box_y_str,
            '-z', box_z_str,
            '-Y', box_yaw_str
        ],
        output='screen'
    )
    
    spawn_goal = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'goal_frame',
            '-file',   goal_sdf,
            '-x', goal_x_str,
            '-y', goal_y_str,
            '-z', goal_z_str,
            '-Y', goal_yaw_str,
        ],
        output='screen'
    )



    # ================== effort 控制器 ==================
    diff_effort_controller = Node(
        package='diff_dyn_car',
        executable='diff_effort_controller',
        name='diff_effort_controller',
        output='screen'
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

    # ================== world ==================
    declare_world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            get_package_share_directory('diff_dyn_car'),
            'worlds',
            'push_box_world.world'
        ),
        description='Gazebo world file'
    )


    # ================== 返回 LaunchDescription ==================
    return LaunchDescription([
        declare_world,
        gazebo,
        rsp_node,
        spawn_car,     # ✔ 随机且不重叠的小车
        spawn_box,     # ✔ 随机且不重叠的方块（无延时）
        spawn_goal,   
        spawner_jsb,
        spawner_effort,
        diff_effort_controller
    ])
