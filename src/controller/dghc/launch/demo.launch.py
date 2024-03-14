import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    #package_dir = get_package_share_directory('ur_robot_driver')

    # Specify the path to the launch files you want to include
    #ur_driver_launch_file = os.path.join(package_dir, 'launch','ur5e.launch.py')

    return LaunchDescription([
        # Launch the nodes
        Node(
            package='iahrs_driver',
            executable='iahrs_driver',
            name='iahrs_driver_node'
        ),
        Node(
            package='serial_comm',
            executable='serial_comm',
            name='serial_comm_node'
        ),
        Node(
            package='dghc',
            executable='trajectory',
            name='trajectoroy_node'
        ),
        Node(
            package='dghc',
            executable='dghc_demo',
            name='dghc_node',
            output="screen"
        ),
        
        # Launch the UR robot driver
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(ur_driver_launch_file),
        #    launch_arguments={'ur_type': 'ur5e', 'robot_ip': '192.168.0.102', 'launch_rviz': 'true', 'initial_joint_controller': 'forward_velocity_controller'}.items(),
        #)
    ])
