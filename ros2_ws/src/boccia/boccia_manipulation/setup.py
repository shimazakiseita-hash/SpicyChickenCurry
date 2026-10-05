from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'boccia_manipulation'

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
    description='MoveIt で CRANE-X7 を動かし、ボールをつかんで置く・押し出す (MoveBall アクションサーバ)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'move_ball_server = boccia_manipulation.move_ball_server:main',
        ],
    },
)
