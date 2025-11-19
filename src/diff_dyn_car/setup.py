from setuptools import setup
import os
from glob import glob

package_name = 'diff_dyn_car'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', ['urdf/diff_dyn_car.urdf.xacro']),
        ('share/' + package_name + '/config', ['config/diff_dyn_car_ros2_control.yaml']),
        ('share/' + package_name + '/launch', ['launch/gazebo_diff_dyn.launch.py']),
        ('share/' + package_name + '/launch', ['launch/display.launch.py']),
        (os.path.join('share', package_name, 'models'),glob(os.path.join('models', '*.sdf'))),
        (os.path.join('share', package_name, 'worlds'),glob(os.path.join('worlds', '*.world'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Differential drive car dynamics (effort control) simulation',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'diff_effort_controller = diff_dyn_car.diff_effort_controller:main',
            'wheel_odom_monitor = diff_dyn_car.wheel_odom_monitor:main',
        ],
    },
)
