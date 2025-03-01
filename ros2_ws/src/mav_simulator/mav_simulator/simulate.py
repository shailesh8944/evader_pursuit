#!/usr/bin/env python3

from class_world import World
import threading
import rclpy
from class_world_node_ros2 import World_Node
import module_kinematics as kin

def ros_thread(node):
    rclpy.spin(node)
    rclpy.shutdown()

def main():
    # Creates an object of class 'World'
    rclpy.init()
    world = World('/workspaces/mavlab/inputs/mavymini/simulation_input.yml')
    world_node = World_Node(world_rate=1/world.dt)
  
    world.node = world_node
    world.start_vessel_ros_nodes(world_node)

    # Run ROS on a separate thread
    ros_thread_instance = threading.Thread(target=ros_thread, args=(world_node,))
    ros_thread_instance.start()

if __name__ == '__main__':
    main()