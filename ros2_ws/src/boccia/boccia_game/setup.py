from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'boccia_game'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SpicyChickenCurry',
    maintainer_email='shimazakiseita@gmail.com',
    description='試合の進行 (どのボールをどこへ動かすか) と得点計算 (ジャックとの距離)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'game_manager_node = boccia_game.game_manager_node:main',
            'scorer_node = boccia_game.scorer_node:main',
        ],
    },
)
