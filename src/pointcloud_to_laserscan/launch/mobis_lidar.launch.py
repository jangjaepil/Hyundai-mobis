from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import yaml


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="sick_safetyscanners2",
            executable="sick_safetyscanners2_node",
            name="front_sick_safetyscanners2_node",
            namespace="front",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"frame_id": "front_lidar_frame",
                 "sensor_ip": "192.168.0.11",
                 "host_ip": "192.168.0.200",
                 "interface_ip": "0.0.0.0",
                 "host_udp_port": 0,
                 "channel": 0,
                 "channel_enabled": True,
                 "skip": 0,
                 "angle_start": 0.0,
                 "angle_end": 0.0,
                 "time_offset": 0.0,
                 "general_system_state": True,
                 "derived_settings": True,
                 "measurement_data": True,
                 "intrusion_data": True,
                 "application_io_data": True,
                 "use_persistent_config": False,
                 "min_intensities": 0.0}
            ]
        ),
        Node(
            package="sick_safetyscanners2",
            executable="sick_safetyscanners2_node",
            name="rear_sick_safetyscanners2_node",
            namespace="rear",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"frame_id": "rear_lidar_frame",
                 "sensor_ip": "192.168.0.12",
                 "host_ip": "192.168.0.200",
                 "interface_ip": "0.0.0.0",
                 "host_udp_port": 0,
                 "channel": 0,
                 "channel_enabled": True,
                 "skip": 0,
                 "angle_start": 0.0,
                 "angle_end": 0.0,
                 "time_offset": 0.0,
                 "general_system_state": True,
                 "derived_settings": True,
                 "measurement_data": True,
                 "intrusion_data": True,
                 "application_io_data": True,
                 "use_persistent_config": False,
                 "min_intensities": 0.0}
            ]
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='laserscan_to_pointcloud_node',
            name='laserscan_to_pointcloud_front',
            remappings=[('scan_in', '/front/scan'),
                        ('cloud', '/front/cloud')],
            parameters=[{'target_frame': 'base_frame', 'transform_tolerance': 0.01}]
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='laserscan_to_pointcloud_node',
            name='laserscan_to_pointcloud_rear',
            remappings=[('scan_in', '/rear/scan'),
                        ('cloud', '/rear/cloud')],
            parameters=[{'target_frame': 'base_frame', 'transform_tolerance': 0.01}]
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_merge_node',
            name='pointcloud_merge_node',
            parameters=[{'target_frame': 'base_frame', 'transform_tolerance': 0.01}]
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/merged_cloud'),
                        ('scan', '/merged_scan')],
            parameters=[{'target_frame': 'base_frame', 'transform_tolerance': 0.01}]
        ),
        Node(
            package='tf_test',
            executable='tf_test',
            name='tf_test',
        ),
        Node(
            package='obstacle_detection',
            executable='obstacle_deteciont_node',
            name='obstacle_deteciont_node',
        )

    ])