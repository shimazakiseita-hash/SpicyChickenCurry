"""【L1】シナリオのボール位置から合成したカメラ画像で、検出を含めた全体を動かす.

  ros2 launch boccia_bringup dummy_l1.launch.py
  ros2 launch boccia_bringup dummy_l1.launch.py with_arm:=false   # 検出だけ試す
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
    perception = get_package_share_directory('boccia_perception')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario', default_value=os.path.join(sim, 'config', 'scenario_default.yaml')),
        DeclareLaunchArgument('with_arm', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(bringup, 'launch', 'camera_tf.launch.py'))),
        Node(package='boccia_sim', executable='synthetic_camera_node', output='screen',
             parameters=[{'scenario': LaunchConfiguration('scenario'),
                          'court_config': os.path.join(bringup, 'config', 'court.yaml'),
                          'use_sim_time': use_sim_time}]),
        Node(package='boccia_perception', executable='ball_detector_node', name='ball_detector',
             output='screen',
             parameters=[os.path.join(perception, 'config', 'ball_detector.yaml'),
                         {'use_sim_time': use_sim_time}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'core.launch.py')),
            launch_arguments={'with_arm': LaunchConfiguration('with_arm'),
                              'use_sim_time': use_sim_time}.items()),
    ])
