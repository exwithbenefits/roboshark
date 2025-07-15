#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg = 'articubot_one'

    # Allow overriding sim time (we'll default to 'false' since this is real-world)
    use_sim_time = LaunchConfiguration('use_sim_time')
    sim_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    # Robot State Publisher (TF from URDF → base_link → laser_frame)
    rsp_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(pkg),
                'launch',
                'rsp.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_ros2_control': 'false'
        }.items()
    )

    # RPLIDAR driver node
    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'laser_frame',
            'angle_compensate': True,
            'scan_mode': 'Standard'
        }]
    )

    # SLAM Toolbox (online async) → publishes /map & map→odom TF
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory(pkg),
                'config',
                'mapper_params_online_async.yaml'
            ),
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        sim_arg,
        rsp_include,
        lidar_node,
        slam_node,
    ])
