import numpy as np
from class_navigation import Navigation
from class_ekf import EKF
from mav_simulator.class_world import World
from rclpy.executors import MultiThreadedExecutor
import rclpy


def main():
    world = World('/workspaces/mavlab/inputs/simulation_input.yml')
    vessels = world.vessels
    ekfs = []
    llh0 = world.gps_datum
    for vessel in vessels:
        ekfs.append(EKF(1/vessel.dt, n_states=15, n_inp=1, pro_noise_cov=np.eye(6)))

    rclpy.init()

    # ROS Multi Thread Execution
    executor = MultiThreadedExecutor()
    navs = []
    for vessel, ekf in zip(vessels, ekfs):
        navs.append(Navigation(vessel, ekf, llh0))
        executor.add_node(navs[-1])
    
    try: 
        executor.spin()

    finally:
        executor.shutdown()
        
        for nav in navs:
            nav.node.destroy_node()
        
        rclpy.shutdown()

if __name__ == '__main__':
    main()











