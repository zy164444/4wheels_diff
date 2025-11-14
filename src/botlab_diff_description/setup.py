from setuptools import setup
import os
from glob import glob

package_name = 'botlab_diff_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # package.xml 安装
        ('share/' + package_name, ['package.xml']),

        # 安装 launch 文件
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),

        # 安装 URDF（注意路径改成相对包名目录）
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join(package_name, 'urdf', '*.xacro'))),

        # 安装 config（yaml）
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join(package_name, 'config', '*.yaml'))),

        # ⭐ 安装 models 目录里的 sdf 文件
        (os.path.join('share', package_name, 'models'),
            glob(os.path.join('models', '*.sdf'))),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dp',
    maintainer_email='2417457465@qq.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_keyboard_teleop = botlab_diff_description.wheel_keyboard_teleop:main',
        ],
    },
)
