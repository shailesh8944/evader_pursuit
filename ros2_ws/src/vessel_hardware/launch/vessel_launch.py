import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch_ros.actions import  PushRosNamespace
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from mav_simulator.class_world import World

def generate_launch_description():
    asv_id = LaunchConfiguration('vessel_id') 
    world = World('/workspaces/mavlab/inputs/simulation_input.yml')
    vessels = world.vessels
    asv_name = f"{vessels[int(asv_id)].vessel_name}_{int(asv_id):02d}"

    return LaunchDescription([
        Node(
            package='uwb_driver',
            executable='uwb',
            name= 'uwb', 
            parameters=[{'vessel_id': asv_id}]
            ) , 
        Node(
            package='vessel_hardware',
            executable='arduino',
            parameters=[{'vessel_id': asv_id}]
            ),
        GroupAction([
            PushRosNamespace(asv_name),  # Ensure namespace applies
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(FindPackageShare("sbg_driver").find("sbg_driver"), "launch", "sbg_device_launch.py")
                )
            )
        ]),        
    ])
