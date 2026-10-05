"""【L0】シナリオファイルのボール位置を検出結果として使う (カメラ・検出ノードなし).

  ros2 launch boccia_bringup dummy_l0.launch.py
  ros2 launch boccia_bringup dummy_l0.launch.py scenario:=/path/to/scenario.yaml with_arm:=false
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
    sim = get_package_share_directory('boccia_sim')
    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario', default_value=os.path.join(sim, 'config', 'scenario_default.yaml')),
        DeclareLaunchArgument('with_arm', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='boccia_sim', executable='fake_ball_publisher', output='screen',
             parameters=[{'scenario': LaunchConfiguration('scenario'),
                          'use_sim_time': LaunchConfiguration('use_sim_time')}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'core.launch.py')),
            launch_arguments={'with_arm': LaunchConfiguration('with_arm'),
                              'use_sim_time': LaunchConfiguration('use_sim_time')}.items()),
    ])
