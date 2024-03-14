from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='obstacle_detection',
            executable='obstacle_deteciont_node.py',
            name='obstacle_deteciont_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                LaunchConfiguration('topics_file')
            ]
        ),
        DeclareLaunchArgument(
            'params_file',
            description='/home/lpigeon/ros2_ws/src/obstacle_detection/obstacle_detection/config/dr_spaam_ros.yaml',
            default_value=[LaunchConfiguration('params_file', default='$(find obstacle_detection)/config/dr_spaam_ros.yaml')]
        ),
        DeclareLaunchArgument(
            'topics_file',
            description='/home/lpigeon/ros2_ws/src/obstacle_detection/obstacle_detection/config/topics.yaml',
            default_value=[LaunchConfiguration('topics_file', default='$(find obstacle_detection)/config/topics.yaml')]
        )
    ])
