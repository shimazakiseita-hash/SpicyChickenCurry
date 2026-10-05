"""Gazebo 上で CRANE-X7 とボールを出し、試合進行・得点計算・アーム動作まで起動する.

  ros2 launch boccia_bringup gazebo.launch.py
  ros2 launch boccia_bringup gazebo.launch.py scenario:=/path/to/scenario.yaml

起動したら、別のターミナルから 1 球ずつ動かせる (README の「Gazebo で動かす」を参照):
  ros2 service call /boccia/play_once std_srvs/srv/Trigger
  ros2 topic echo /boccia/score

構成:
  crane_x7_gazebo の crane_x7_with_table.launch.py (Gazebo + コントローラ + MoveIt + RViz)
  → 起動を待ってから gazebo_ball_spawner でボールを置く (木のブロックは取り除く)
  → gazebo_ball_publisher (Gazebo の中のボールの本当の位置を /boccia/balls に出す. カメラの代わり)
  → core.launch.py (得点計算・試合進行・アーム動作、use_sim_time:=true)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory('boccia_bringup')
    sim = get_package_share_directory('boccia_sim')
    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('crane_x7_gazebo'), 'launch', 'crane_x7_with_table.launch.py')))
    court_config = os.path.join(bringup, 'config', 'court.yaml')
    spawner = Node(
        package='boccia_sim', executable='gazebo_ball_spawner', output='screen',
        parameters=[{'scenario': LaunchConfiguration('scenario'),
                     'court_config': court_config}])
    ball_publisher = Node(
        package='boccia_sim', executable='gazebo_ball_publisher', output='screen',
        parameters=[{'court_config': court_config, 'use_sim_time': True}])
    core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'core.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items())

    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario', default_value=os.path.join(sim, 'config', 'scenario_default.yaml')),
        DeclareLaunchArgument(
            'startup_delay', default_value='15.0',
            description='Gazebo とコントローラの起動を待つ時間 [s] (遅い PC では増やす)'),
        gazebo,
        TimerAction(period=LaunchConfiguration('startup_delay'),
                    actions=[spawner, ball_publisher, core]),
    ])
