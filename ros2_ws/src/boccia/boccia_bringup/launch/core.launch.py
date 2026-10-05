"""どのモードでも共通のノード: 得点計算、試合進行、アーム動作.

dummy_l0 / dummy_l1 / real の各 launch から読み込まれる.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory('boccia_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'with_arm', default_value='true',
            description='false にするとアーム動作 (MoveIt) を起動しない (検出だけ試すとき)'),
        Node(package='boccia_game', executable='scorer_node', output='screen',
             parameters=[{'use_sim_time': use_sim_time}]),
        Node(package='boccia_game', executable='game_manager_node', output='screen',
             parameters=[{
                 'use_sim_time': use_sim_time,
                 'court_config': os.path.join(bringup, 'config', 'court.yaml'),
             }]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'manipulation.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
            condition=IfCondition(LaunchConfiguration('with_arm')),
        ),
    ])
