"""MoveBall アクションサーバ (boccia_manipulation) を MoveIt の設定つきで起動する.

ロボット側 (コントローラ + MoveIt) は別に起動しておくこと:
  実機:         ros2 launch crane_x7_examples demo.launch.py port_name:=/dev/ttyUSB0
  仮想モーター: ros2 launch crane_x7_examples demo.launch.py use_mock_components:=true
  Gazebo:       ros2 launch crane_x7_gazebo crane_x7_with_table.launch.py  (use_sim_time:=true)
"""

import os

from ament_index_python.packages import get_package_share_directory
from crane_x7_description.robot_description_loader import RobotDescriptionLoader
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    share = get_package_share_directory('boccia_manipulation')
    moveit_config = (
        MoveItConfigsBuilder('crane_x7')
        # OMPL (自由な移動) と Pilz (直線移動) の両方を読み込む
        .planning_pipelines(pipelines=['ompl', 'pilz_industrial_motion_planner'])
        .moveit_cpp(file_path=os.path.join(share, 'config', 'moveit_py.yaml'))
        .to_moveit_configs()
    )
    moveit_config.robot_description = {'robot_description': RobotDescriptionLoader().load()}
    # crane_x7_examples_py と同じく、ここで use_sim_time を足す
    # (https://github.com/moveit/moveit2/issues/2940#issuecomment-2401302214)
    params = moveit_config.to_dict()
    params.update({'use_sim_time': LaunchConfiguration('use_sim_time')})

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='boccia_manipulation',
            executable='move_ball_server',
            output='screen',
            parameters=[params, os.path.join(share, 'config', 'move_ball.yaml'),
                        {'court_config': os.path.join(
                            get_package_share_directory('boccia_bringup'), 'config', 'court.yaml')}],
        ),
    ])
