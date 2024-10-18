import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mav_simulator',  # Replace with your package name
            executable='simulate',  # Replace with your script name
            name='MAVSIM'
        ),
        Node(
            package='gnc',
            executable='gnc',
            name='GNC'
        ),
    ])
