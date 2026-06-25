import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'acroba_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'worlds'),
            glob(os.path.join('worlds', '*.sdf'))),
        (os.path.join('share', package_name, 'models', 'acroba_bot'),
            glob(os.path.join('models', 'acroba_bot', '*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'rviz'),
            glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ibrahima NIASSE',
    maintainer_email='ibrahimaniasse@github.com',
    description='Launch files, simulation worlds, and robot model for the Acroba framework',
    license='MIT',
)
