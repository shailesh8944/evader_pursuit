import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gnc',  # Replace with your package name
            executable='nav',  # Replace with your script name
            name='nav'
        ),
    ])