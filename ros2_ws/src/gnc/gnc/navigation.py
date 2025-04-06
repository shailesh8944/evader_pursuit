import numpy as np
from class_navigation import Navigation
from class_ekf import EKF
from mav_simulator.class_world import World
from rclpy.executors import MultiThreadedExecutor
import rclpy

def calculate_threshold(dt):
        # Threshold for correction
        th = np.full(15, np.inf)
                
        # Thresholds for positions
        th[0] = np.sqrt(th[6]**2 + th[7]**2) * dt
        th[1] = np.sqrt(th[6]**2 + th[7]**2) * dt
        th[2] = 1 / 1000000

        th[3:6] = np.linalg.norm(th[9:12]) * dt

        th[6:9] = th[12:15] * dt
        th[9:12] = np.inf * dt
        th[12:15] = np.inf * dt

        return th

def main():
    world = World('/workspaces/mavlab/inputs/simulation_input.yml')
    vessels = world.vessels
    ekfs = []
    llh0 = world.gps_datum
    for vessel in vessels:        
        ekfs.append(EKF(
             1/vessel.dt, 
             n_states=15, 
             n_inp=1, 
             pro_noise_cov=1e2*np.diag([1, 1, 1, 1, 1, 1])
        ))

    rclpy.init()

    # ROS Multi Thread Execution
    executor = MultiThreadedExecutor()
    navs = []
    for vessel, ekf in zip(vessels, ekfs):
        th = calculate_threshold(vessel.dt)
        navs.append(Navigation(vessel, ekf, llh0, th=th))
        executor.add_node(navs[-1])
    
    try: 
        executor.spin()

    finally:
        executor.shutdown()
        
        for nav in navs:
            nav.destroy_node()
        
        rclpy.shutdown()

if __name__ == '__main__':
    main()











