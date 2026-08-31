#!/usr/bin/env python3
# Launches the LiDAR + static transform + rf2o odometry pipeline together.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # 1. RPLIDAR driver
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/rover_lidar',
                'serial_baudrate': 115200,
                'frame_id': 'laser_frame',
                'angle_compensate': True,
            }],
        ),

        # 2. Static transform: base_link -> laser_frame (laser 20cm up)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            output='screen',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0.20',
                '--frame-id', 'base_link',
                '--child-frame-id', 'laser_frame',
            ],
        ),

        # 3. rf2o laser odometry: odom -> base_link
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic': '/scan',
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'publish_tf': True,
                'freq': 10.0,
            }],
        ),
    ])
