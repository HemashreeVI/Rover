#!/usr/bin/env python3
# Master bringup: LiDAR + odometry + SLAM + cmd_bridge + heatmap + A* + follower.
# (detection_node runs separately — needs the YOLO venv python)
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rover_control_share = get_package_share_directory('rover_control')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')

    slam_params = os.path.join(rover_control_share, 'config', 'slam_params.yaml')

    return LaunchDescription([

        # 1. LiDAR driver
        Node(
            package='rplidar_ros', executable='rplidar_node', name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/rover_lidar',
                'serial_baudrate': 115200,
                'frame_id': 'laser_frame',
                'angle_compensate': True,
            }],
        ),

        # 2. Static transform base_link -> laser_frame
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='base_to_laser', output='screen',
            arguments=['--x', '0', '--y', '0', '--z', '0.20',
                       '--frame-id', 'base_link', '--child-frame-id', 'laser_frame'],
        ),

        # 3. rf2o laser odometry
        Node(
            package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry', output='screen',
            parameters=[{
                'laser_scan_topic': '/scan',
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'publish_tf': True,
                'freq': 10.0,
            }],
        ),

        # 4. slam_toolbox (delayed 5s so odometry is up first)
        TimerAction(period=5.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    slam_toolbox_share, 'launch', 'online_async_launch.py')),
                launch_arguments={'slam_params_file': slam_params}.items(),
            ),
        ]),

        # 5. cmd_bridge (motors + gas + IMU)
        Node(package='rover_control', executable='cmd_bridge',
             name='cmd_bridge', output='screen'),

        # 6. risk heatmap (delayed 6s so map/TF exist)
        TimerAction(period=6.0, actions=[
            Node(package='rover_control', executable='risk_heatmap',
                 name='risk_heatmap', output='screen'),
        ]),

        # 7. A* planner (delayed 7s)
        TimerAction(period=7.0, actions=[
            Node(package='rover_control', executable='astar_planner',
                 name='astar_planner', output='screen'),
        ]),

        # 8. path follower (delayed 7s)
        TimerAction(period=7.0, actions=[
            Node(package='rover_control', executable='path_follower',
                 name='path_follower', output='screen'),
        ]),
    ])
