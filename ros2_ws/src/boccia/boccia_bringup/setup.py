from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'boccia_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SpicyChickenCurry',
    maintainer_email='shimazakiseita@gmail.com',
    description='ミニボッチャ一式の起動ファイルと共通設定 (コート寸法、カメラ位置、ボールの色)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
        ],
    },
)
