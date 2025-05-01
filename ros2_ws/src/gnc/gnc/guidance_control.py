"""
File: guidance_control.py
Description: This script initializes and runs the Guidance and Control (GC) ROS2 node(s) for vessels 
             defined in a simulation configuration file. It leverages the `GuidanceControl` class 
             and a multi-threaded executor to manage potentially multiple vessel control nodes.

Author: MAV GNC Team
"""

import rclpy
from class_guidance_control import GuidanceControl
from mav_simulator.class_world import World # Used to load vessel configurations
from rclpy.executors import MultiThreadedExecutor # For running multiple ROS2 nodes

def main():
    """Main function to set up and execute the GuidanceControl nodes."""
    # Load simulation setup, including vessel definitions, from a YAML file
    world = World('/workspaces/mavlab/inputs/simulation_input.yml')
    vessels = world.vessels  # Get the list of vessel objects from the world setup
    # llh0 = world.gps_datum # GPS datum (origin) - loaded but not used in this script
    
    # Initialize the ROS2 Python client library
    rclpy.init()

    # Use a MultiThreadedExecutor to handle callbacks for multiple nodes concurrently
    executor = MultiThreadedExecutor()
    gcs = [] # List to keep track of the GuidanceControl node instances

    # Create a GuidanceControl node for each vessel defined in the world
    for vessel in vessels:
        gc_node = GuidanceControl(vessel) # Instantiate the node, passing vessel-specific config
        gcs.append(gc_node)
        executor.add_node(gc_node) # Add the newly created node to the executor
        print(f"Added GuidanceControl node for {vessel.vessel_name}")
    
    try: 
        print("Spinning GuidanceControl nodes...")
        # Start the ROS2 executor loop. This blocks until rclpy.shutdown() is called.
        # It waits for and executes callbacks (e.g., timer triggers, subscription messages) for all added nodes.
        executor.spin()

    finally:
        # This block executes on shutdown (e.g., Ctrl+C)
        print("Shutting down GuidanceControl nodes...")
        # Stop the executor
        executor.shutdown()
        
        # Explicitly destroy each created node
        for gc in gcs:
            print(f"Destroying node {gc.get_name()}")
            gc.destroy_node()
        
        # Shutdown the ROS2 client library
        rclpy.shutdown()

if __name__ == '__main__':
    # Standard Python entry point: ensures main() runs only when the script is executed directly
    main()