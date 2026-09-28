from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'crane_x7_rl'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SpicyChickenCurry',
    maintainer_email='shimazakiseita@gmail.com',
    description='MuJoCo で学習した方策を CRANE-X7 (Gazebo / 実機) で動かす',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'reach_policy_node = crane_x7_rl.reach_policy_node:main',
        ],
    },
)
