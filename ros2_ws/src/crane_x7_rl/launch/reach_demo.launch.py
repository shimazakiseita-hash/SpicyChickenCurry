"""学習した到達方策を動かす (Gazebo は別途 crane_x7_with_table.launch.py で起動しておく).

    ros2 launch crane_x7_rl reach_demo.launch.py policy_path:=<policy.npz のパス>
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('policy_path', description='mujoco/runs/.../policy.npz のパス'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='crane_x7_rl',
            executable='reach_policy_node',
            output='screen',
            parameters=[{
                'policy_path': LaunchConfiguration('policy_path'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
