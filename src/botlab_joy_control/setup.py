from setuptools import setup

package_name = 'botlab_joy_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dp',
    maintainer_email='2417457465@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_diff_drive_cmdvel = botlab_joy_control.joy_diff_drive_cmdvel:main',
            'joy_server_ros2 = botlab_joy_control.joy_server_ros2:main',
        ],
    },
)
