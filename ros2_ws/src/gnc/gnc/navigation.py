"""
File: navigation.py
Description: This script sets up and runs the navigation node for one or more vessels. 
             It initializes Extended Kalman Filters (EKFs) for state estimation and the main Navigation class 
             for each vessel, handling sensor data processing, state updates, and communication within a ROS2 environment.
             The script uses a multi-threaded executor to manage multiple navigation nodes concurrently.

Author: MAV GNC Team
"""
import numpy as np
from class_navigation import Navigation
from gnc.class_ekf import EKF
from mav_simulator.class_world import World
from rclpy.executors import MultiThreadedExecutor
import rclpy

def calculate_threshold(dt):
        """Calculates state estimation correction thresholds and process noise covariance.

        This function determines thresholds used within the EKF to decide when to apply corrections 
        based on sensor measurements. It also calculates the process noise covariance matrix (Q) 
        representing the uncertainty in the state prediction model.

        The thresholds and covariance are derived based on assumed standard deviations 
        of the derivatives of angular velocity (omega) and linear acceleration (acc).

        Args:
            dt (float): Time step in seconds.

        Returns:
            tuple: 
                - np.ndarray: Array of thresholds for state variables [pos, vel, att, acc_bias, gyro_bias].
                - np.ndarray: Process noise covariance matrix (Q) for the EKF.
        """
        # Threshold for correction - initialized to infinity (no correction initially)
        th = np.full(15, np.inf)

        # Assumed standard deviations for the rate of change (jerk/jounce) of measurements
        acc_der_std = 10  # Standard deviation for acceleration derivative (m/s^3)
        omg_der_std = 10  # Standard deviation for angular velocity derivative (rad/s^2)

        # Calculate variances
        acc_der_var = acc_der_std**2
        omg_der_var = omg_der_std**2

        # Initialize process noise covariance matrix (Q) - Diagonal initially
        # Order: [omega_dot (3), acc_dot (3)] - assuming noise affects derivatives
        cov = np.diag([1, 1, 1, 1, 1, 1]) 
        cov[0:3, 0:3] = omg_der_var * np.eye(3) # Noise in angular acceleration
        cov[3:6, 3:6] = acc_der_var * np.eye(3) # Noise in linear jerk

        # Calculate thresholds based on noise propagation over one time step (dt)
        # These represent the expected change/uncertainty in bias estimates
        th[9:12] = np.sqrt(omg_der_var/3) * dt   # Gyro bias threshold (rad) - Indices 9, 10, 11
        th[12:15] = np.sqrt(acc_der_var/3) * dt  # Accelerometer bias threshold (m/s) - Indices 12, 13, 14

        # Thresholds for velocity and attitude - propagating bias uncertainty
        # Factor of 2.0 is potentially heuristic or based on specific system tuning needs
        th[6:9] = 2.0 * np.linalg.norm(th[12:15]) * dt # Velocity threshold (m) - Indices 6, 7, 8
        th[3:6] = 2.0 * np.linalg.norm(th[9:12]) * dt  # Attitude threshold (rad) - Indices 3, 4, 5

        # Thresholds for position - propagating velocity uncertainty
        # Factor of 2.0 is potentially heuristic
        th[0] = 2.0 * np.linalg.norm(th[6:9]) * dt # Position X threshold (m) - Index 0
        th[1] = 2.0 * np.linalg.norm(th[6:9]) * dt # Position Y threshold (m) - Index 1
        th[2] = 0.0 # Position Z threshold (m) - Assuming Z position is fixed or highly certain? Index 2

        return th, cov

def main():
    """Main function to initialize and run the navigation system for all vessels."""
    # Load simulation configuration using the World class from mav_simulator
    # This likely contains vessel parameters, simulation time, etc.
    world = World('/workspaces/mavlab/inputs/simulation_input.yml')
    vessels = world.vessels # Get the list of vessel objects
    ekfs = []             # List to hold EKF instances
    llh0 = world.gps_datum # Get the GPS datum (origin) for coordinate conversions

    # Initialize an EKF for each vessel
    for vessel in vessels:        
        # Calculate process noise covariance based on vessel's time step
        _, cov = calculate_threshold(vessel.dt) 
        ekfs.append(EKF(
             frequency=1/vessel.dt, # EKF update frequency
             n_states=15,          # Number of states (pos, att, vel, bias_acc, bias_gyro)
             n_inp=1,              # Number of inputs (usually none used directly in state transition matrix for EKF)
             pro_noise_cov=cov     # Process noise covariance matrix (Q)
        ))

    # Initialize ROS2
    rclpy.init()

    # Use a MultiThreadedExecutor to handle multiple ROS2 nodes (one per vessel)
    executor = MultiThreadedExecutor()
    navs = [] # List to hold Navigation node instances

    # Create and configure a Navigation node for each vessel
    for vessel, ekf in zip(vessels, ekfs):
        # Calculate thresholds for this specific vessel's dt
        th, _ = calculate_threshold(vessel.dt)        
        # Create the Navigation node instance
        # Passing th=None suggests thresholds might be handled internally or optional
        navs.append(Navigation(vessel, ekf, llh0, th=None)) 
        # Add the node to the executor
        executor.add_node(navs[-1])
    
    try: 
        print("Spinning Navigation nodes...")
        # Run the ROS2 event loop, processing callbacks for all added nodes
        executor.spin()

    finally:
        # Ensure clean shutdown
        print("Shutting down Navigation nodes...")
        executor.shutdown()
        
        # Destroy each navigation node explicitly
        for nav in navs:
            nav.destroy_node()
        
        # Shutdown ROS2 communications
        rclpy.shutdown()

if __name__ == '__main__':
    # Standard Python entry point guard
    main()











