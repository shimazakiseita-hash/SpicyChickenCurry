"""RealSense D435 (実機) を起動し、固定カメラの位置を TF に流す.

トピック名を /camera/color/image_raw などにするため、camera_namespace を空にしている
(realsense2_camera 4.x の既定だと /camera/camera/... になる).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')),
        launch_arguments={
            'camera_namespace': '',
            'camera_name': 'camera',
            'align_depth.enable': 'true',
            'rgb_camera.color_profile': '640,480,30',
            'depth_module.depth_profile': '640,480,30',
        }.items(),
    )
    camera_tf = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('boccia_bringup'), 'launch', 'camera_tf.launch.py')))
    return LaunchDescription([realsense, camera_tf])
