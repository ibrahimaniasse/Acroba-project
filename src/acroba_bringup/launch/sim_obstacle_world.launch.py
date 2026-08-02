"""Launch Gazebo simulation with the acroba_bot in the obstacle field.

Usage:
    ros2 launch acroba_bringup sim_obstacle_world.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('acroba_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(pkg_bringup, 'worlds', 'obstacle_field.sdf')
    model_path = os.path.join(pkg_bringup, 'models')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
        }.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'acroba_bot',
            '-file', os.path.join(model_path, 'acroba_bot', 'model.sdf'),
            '-x', '-8.0',
            '-y', '-8.0',
            '-z', '0.15',
        ],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
        ],
        output='screen',
    )

    return LaunchDescription([
        gz_sim,
        spawn_robot,
        bridge,
    ])
