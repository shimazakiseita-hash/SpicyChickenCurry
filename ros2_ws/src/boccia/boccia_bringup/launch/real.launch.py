"""実機の RealSense を使って全体を動かす.

ロボット側は別に起動しておくこと (manipulation.launch.py の説明を参照).

  ros2 launch boccia_bringup real.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory('boccia_bringup')
    perception = get_package_share_directory('boccia_perception')
    return LaunchDescription([
        DeclareLaunchArgument('with_arm', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(bringup, 'launch', 'camera.launch.py'))),
        Node(package='boccia_perception', executable='ball_detector_node', name='ball_detector',
             output='screen',
             parameters=[os.path.join(perception, 'config', 'ball_detector.yaml')]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'core.launch.py')),
            launch_arguments={'with_arm': LaunchConfiguration('with_arm')}.items()),
    ])
