import os
from launch import LaunchDescription
from launch.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mav_simulator',  # Replace with your package name
            executable='simulate.py',  # Replace with your script name
            name='MAVSIM'
        ),
        Node(
            package='gnc',
            executable='start_gnc.py',
            name='GNC'
        ),
    ])
