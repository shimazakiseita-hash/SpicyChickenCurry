"""学習した到達方策を動かす.

Gazebo (crane_x7_with_table.launch.py を別途起動しておく):
    ros2 launch crane_x7_rl reach_demo.launch.py policy_path:=<policy.npz のパス>

実機 (crane_x7_examples の demo.launch.py を別途起動しておく):
    ros2 launch crane_x7_rl reach_demo.launch.py policy_path:=<policy.npz のパス> use_sim_time:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('policy_path', description='mujoco/policies/... の policy.npz のパス'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Gazebo なら true、実機なら false (実機には /clock が無いので動かなくなる)'),
        DeclareLaunchArgument(
            'show_in_gazebo', default_value=LaunchConfiguration('use_sim_time'),
            description='目標の球を Gazebo に表示する (既定は use_sim_time と同じ)'),
        Node(
            package='crane_x7_rl',
            executable='reach_policy_node',
            output='screen',
            parameters=[{
                'policy_path': LaunchConfiguration('policy_path'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'show_in_gazebo': LaunchConfiguration('show_in_gazebo'),
            }],
        ),
    ])
