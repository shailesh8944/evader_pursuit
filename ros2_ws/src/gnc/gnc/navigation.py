import numpy as np
from class_navigation import Navigation
from gnc.class_ekf import EKF
from mav_simulator.class_world import World
from rclpy.executors import MultiThreadedExecutor
import rclpy

def calculate_threshold(dt):
        # Threshold for correction
        th = np.full(15, np.inf)

        acc_der_std = 10
        omg_der_std = 10

        acc_der_var = acc_der_std**2
        omg_der_var = omg_der_std**2

        cov = np.diag([1, 1, 1, 1, 1, 1])
        cov[0:3, 0:3] = omg_der_var * np.eye(3)
        cov[3:6, 3:6] = acc_der_var * np.eye(3)

        th[9:12] = np.sqrt(omg_der_var/3) * dt
        th[12:15] = np.sqrt(acc_der_var/3) * dt

        th[6:9] = 2.0 * np.linalg.norm(th[12:15]) * dt
        th[3:6] = 2.0 * np.linalg.norm(th[9:12]) * dt

        # Thresholds for positions
        th[0] = 2.0 * np.linalg.norm(th[6:9]) * dt
        th[1] = 2.0 * np.linalg.norm(th[6:9]) * dt
        th[2] = 0.0

        return th, cov

def main():
    world = World('/workspaces/mavlab/inputs/simulation_input.yml')
    vessels = world.vessels
    ekfs = []
    llh0 = world.gps_datum
    for vessel in vessels:        
        _, cov = calculate_threshold(vessel.dt)
        ekfs.append(EKF(
             1/vessel.dt, 
             n_states=15, 
             n_inp=1, 
             pro_noise_cov=cov
        ))

    rclpy.init()

    # ROS Multi Thread Execution
    executor = MultiThreadedExecutor()
    navs = []
    for vessel, ekf in zip(vessels, ekfs):
        th, _ = calculate_threshold(vessel.dt)        
        navs.append(Navigation(vessel, ekf, llh0, th=None))
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











