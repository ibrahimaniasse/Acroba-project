from setuptools import find_packages, setup

package_name = 'acroba_manager'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='Ibrahima NIASSE',
    maintainer_email='ibrahimaniasse@github.com',
    description='Central behavior orchestration node for the Acroba framework',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'behavior_manager = acroba_manager.behavior_manager_node:main',
        ],
    },
)
