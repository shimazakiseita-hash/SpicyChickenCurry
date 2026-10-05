from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'boccia_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*'))),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='SpicyChickenCurry',
    maintainer_email='shimazakiseita@gmail.com',
    description='実機が無くても動作確認するためのダミーデータ (L0: 固定座標, L1: 合成カメラ画像, L2: Gazebo)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fake_ball_publisher = boccia_sim.fake_ball_publisher:main',
            'synthetic_camera_node = boccia_sim.synthetic_camera_node:main',
            'gazebo_ball_spawner = boccia_sim.gazebo_ball_spawner:main',
            'gazebo_ball_publisher = boccia_sim.gazebo_ball_publisher:main',
        ],
    },
)
