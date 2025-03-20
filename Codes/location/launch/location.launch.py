#!/usr/bin/env python
import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='location',
            executable='recorder_node',  # This runs location/recorder.py
            name='recorder_node',
            output='screen',
            parameters=[],  # You can add parameters here if needed
        ),
        Node(
            package='location',
            executable='detection_node',  # This runs location/detection.py
            name='detection_node',
            output='screen',
            parameters=[],  # You can add parameters here if needed
        ),
    ])
