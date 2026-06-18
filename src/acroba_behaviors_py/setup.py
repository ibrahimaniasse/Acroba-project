from setuptools import find_packages, setup

package_name = 'acroba_behaviors_py'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ibrahima NIASSE',
    maintainer_email='ibrahimaniasse@github.com',
    description='Python-based modular behaviors for the Acroba framework',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_to_waypoint = acroba_behaviors_py.move_to_waypoint:main',
            'rotate_in_place = acroba_behaviors_py.rotate_in_place:main',
            'patrol_loop = acroba_behaviors_py.patrol_loop:main',
            'return_to_home = acroba_behaviors_py.return_to_home:main',
            'wait_for_trigger = acroba_behaviors_py.wait_for_trigger:main',
            'search_pattern = acroba_behaviors_py.search_pattern:main',
            'follow_path = acroba_behaviors_py.follow_path:main',
            'dock_at_station = acroba_behaviors_py.dock_at_station:main',
            'formation_hold = acroba_behaviors_py.formation_hold:main',
        ],
    },
)
