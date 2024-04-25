import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[os.path.join(os.getcwd(), 'mav_simulator', 'simulate.py')],
            output='screen',
        )
    ])
