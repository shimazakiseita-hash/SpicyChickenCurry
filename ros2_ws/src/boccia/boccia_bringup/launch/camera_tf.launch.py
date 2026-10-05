"""固定カメラの位置 (config/camera_extrinsics.yaml) を TF に流す.

camera.launch.py (実機) と dummy_l1.launch.py (合成画像) の両方から読み込まれる.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    path = os.path.join(
        get_package_share_directory('boccia_bringup'), 'config', 'camera_extrinsics.yaml')
    with open(path) as f:
        cfg = yaml.safe_load(f)
    t, r = cfg['translation'], cfg['rotation_rpy']
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_extrinsics_tf',
            arguments=[
                '--x', str(t['x']), '--y', str(t['y']), '--z', str(t['z']),
                '--roll', str(r['roll']), '--pitch', str(r['pitch']), '--yaw', str(r['yaw']),
                '--frame-id', cfg['parent_frame'], '--child-frame-id', cfg['child_frame'],
            ],
        ),
    ])
