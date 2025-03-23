import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_ros2_gps_ackermann',
            executable='gps_ackermann_node',
            name='gps_ackermann_node',
            output='screen'
        ),
    ])
