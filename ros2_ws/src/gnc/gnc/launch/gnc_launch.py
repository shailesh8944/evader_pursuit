import os
from launch import LaunchDescription
from launch.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gnc',  # Replace with your package name
            executable='start_gnc.py',  # Replace with your script name
            name='GNC'
        )
    ])