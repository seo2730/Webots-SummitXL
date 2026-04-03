from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import yaml


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='namespace', default_value='',
            description='Top-level namespace'
        ),
        ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', '-r', '10',
                '--qos-profile', 'sensor_data',
                [namespace, '/scan'],
                'sensor_msgs/msg/LaserScan', yaml.dump({
                    'header': {'frame_id': 'scan'}, 'angle_min': -1.0,
                    'angle_max': 1.0, 'angle_increment': 0.1, 'range_max': 10.0,
                    'ranges': [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
                })
            ],
            name='scan_publisher'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace=namespace,
            name='static_transform_publisher',
            arguments='0 0 0 0 0 0 1 map scan'
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='laserscan_to_pointcloud_node',
            namespace=namespace,
            name='laserscan_to_pointcloud',
            remappings=[('scan_in', 'scan'),
                        ('cloud', 'cloud')],
            parameters=[{'target_frame': 'scan', 'transform_tolerance': 0.01}]
        ),
    ])
