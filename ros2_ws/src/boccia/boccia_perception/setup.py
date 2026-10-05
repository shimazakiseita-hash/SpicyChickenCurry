from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'boccia_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SpicyChickenCurry',
    maintainer_email='shimazakiseita@gmail.com',
    description='固定カメラ (RealSense D435) のカラー・深度画像からボールの 3 次元位置を検出する',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ball_detector_node = boccia_perception.ball_detector_node:main',
        ],
    },
)
