import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[os.path.join(os.getcwd(), 'gnc', 'start_gnc.py')],
            output='screen',
        )
    ])