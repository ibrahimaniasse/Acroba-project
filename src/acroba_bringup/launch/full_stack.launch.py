"""Launch the full Acroba framework stack.

This launch file brings up:
  1. Gazebo simulation (configurable world)
  2. ROS-Gz bridge for sensor topics
  3. Behavior manager node
  4. All behavior nodes

Usage:
    ros2 launch acroba_bringup full_stack.launch.py
    ros2 launch acroba_bringup full_stack.launch.py world:=obstacle_field
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('acroba_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Arguments
    world_arg = DeclareLaunchArgument(
        'world', default_value='empty_arena',
        description='World name: empty_arena or obstacle_field'
    )
    use_sim_arg = DeclareLaunchArgument(
        'use_sim', default_value='true',
        description='Launch Gazebo simulation'
    )

    world_name = LaunchConfiguration('world')
    behaviors_config = os.path.join(pkg_bringup, 'config', 'behaviors.yaml')

    # Gazebo simulation
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', pkg_bringup, '/worlds/', world_name, '.sdf'],
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_sim')),
    )

    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'acroba_bot',
            '-file', os.path.join(pkg_bringup, 'models', 'acroba_bot', 'model.sdf'),
            '-x', '0.0', '-y', '0.0', '-z', '0.15',
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_sim')),
    )

    # ROS-Gz bridge
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
        condition=IfCondition(LaunchConfiguration('use_sim')),
    )

    # Behavior manager
    behavior_manager = Node(
        package='acroba_manager',
        executable='behavior_manager',
        name='behavior_manager',
        output='screen',
        parameters=[{
            'tick_rate_hz': 20.0,
            'behaviors_config': behaviors_config,
        }],
    )

    # C++ behavior nodes
    avoid_obstacle = Node(
        package='acroba_behaviors_cpp',
        executable='avoid_obstacle',
        name='avoid_obstacle',
        output='screen',
    )

    follow_wall = Node(
        package='acroba_behaviors_cpp',
        executable='follow_wall',
        name='follow_wall',
        output='screen',
    )

    emergency_stop = Node(
        package='acroba_behaviors_cpp',
        executable='emergency_stop',
        name='emergency_stop',
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        use_sim_arg,
        gz_sim,
        spawn_robot,
        bridge,
        behavior_manager,
        avoid_obstacle,
        follow_wall,
        emergency_stop,
    ])
