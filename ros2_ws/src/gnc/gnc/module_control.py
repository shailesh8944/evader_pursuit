"""
File: module_control.py
Description: This module provides control functions for vessel steering, primarily focusing on waypoint following 
             using rudder control. It includes helper functions for angle normalization and value clipping, 
             and implements a PID-based control strategy for path following.

Author: MAV GNC Team
"""
import numpy as np
import warnings

"""
Rudder control module for vessel simulation.

This module provides different rudder control strategies for vessel steering:
"""

def ssa(ang, deg=False):
    """Smallest signed angle normalization.

    Calculates the smallest signed angle equivalent to the input angle, ensuring the result lies between -pi and +pi radians 
    (or -180 and +180 degrees if deg=True).

    Args:
        ang (float): Input angle.
        deg (bool, optional): If True, assumes input `ang` is in degrees and returns degrees. 
                              If False (default), assumes radians and returns radians.

    Returns:
        float: The normalized angle in the range [-pi, pi] or [-180, 180].
    """
    # Normalize angle based on whether it's in degrees or radians
    if deg:
        # Normalize degrees to [-180, 180)
        ang = (ang + 180.0) % 360.0 - 180.0
    else:
        # Normalize radians to [-pi, pi)
        ang = (ang + np.pi) % (2.0 * np.pi) - np.pi
    
    return ang

def clip(val, min_val, max_val):
    """Clips a value to a specified range.

    Ensures that the input value `val` does not exceed the bounds defined by `min_val` and `max_val`.

    Args:
        val (float or int): The value to clip.
        min_val (float or int): The minimum allowed value.
        max_val (float or int): The maximum allowed value.

    Returns:
        float or int: The clipped value.
    """
    return max(min_val, min(val, max_val))


def pid_control(t, state, waypoints, waypoint_idx, ye_int=0.0):
    """PID-based Line-of-Sight (LOS) guidance controller for waypoint following.

    This function calculates the required rudder angle command (`delta_c`) to steer the vessel 
    towards the current target waypoint using a PID control strategy based on cross-track error (ye).
    It also handles waypoint switching logic.
    
    Note: This implementation appears to be closer to a LOS guidance law feeding into a 
          heading controller rather than a pure PID on cross-track error.

    Args:
        t (float): Current simulation time [s]. (Currently unused in the function logic)
        state (np.ndarray): Current vessel state vector. Expected order includes: 
                           [u, v, w, p, q, r, x, y, z, phi, theta, psi, ...]
                           Indices used: 3(u), 4(v), 5(r), 6(x), 7(y), 11(psi).
        waypoints (np.ndarray): Array of waypoints, where each row is [x, y, z].
        waypoint_idx (int): Index of the current target waypoint in the `waypoints` array.
        ye_int (float, optional): Integral of the cross-track error. Defaults to 0.0. (Currently unused)
        
    Returns:
        tuple: 
            - float: Commanded rudder angle in radians (negated due to convention).
            - float: Current cross-track error (ye).
            - int: Index of the next target waypoint.
    """

    # Check if the final waypoint has already been reached
    if waypoint_idx >= len(waypoints):
        warnings.warn(f"Final waypoint reached or invalid index {waypoint_idx}. Holding rudder at 0.")
        return 0.0, 0.0, waypoint_idx # Return zero command and error

    # Extract relevant states
    # Body-fixed velocities: surge (u), sway (v), yaw rate (r)
    u, v, r = state[3], state[4], state[5]
    # Inertial position (x, y) and yaw angle (psi)
    x, y, psi = state[6], state[7], state[11]
    
    # Get current target waypoint and the previous one (origin of the current leg)
    wp_xn, wp_yn, _ = waypoints[waypoint_idx]       # Target waypoint (Next)
    wp_xn1, wp_yn1, _ = waypoints[waypoint_idx - 1] # Previous waypoint (n-1)

    # --- Path Following Logic --- 
    
    # Calculate the unit vector pointing along the desired path segment
    wp_vec = np.array([wp_xn - wp_xn1, wp_yn - wp_yn1, 0.0])
    # Handle potential zero-length segment (e.g., consecutive identical waypoints)
    norm_wp_vec = np.linalg.norm(wp_vec)
    if norm_wp_vec < 1e-6: # Avoid division by zero
        warnings.warn(f"Waypoint segment {waypoint_idx-1} to {waypoint_idx} has zero length. Holding rudder.")
        return 0.0, 0.0, waypoint_idx 
    wp_unit_vec = wp_vec / norm_wp_vec
    
    # Calculate the desired path angle (course angle)
    pi_p = np.arctan2(wp_unit_vec[1], wp_unit_vec[0])

    # Calculate the vector from the previous waypoint to the vessel's current position
    r_vec = np.array([x - wp_xn1, y - wp_yn1, 0.0])

    # Calculate cross-track error (ye): perpendicular distance from the vessel to the desired path
    # Positive ye means the vessel is to the starboard side of the path
    ye = np.cross(wp_unit_vec, r_vec)[2]
    
    # Calculate the rate of change of the cross-track error (ye_dot)
    # First, calculate inertial velocities (NED frame)
    x_dot = u * np.cos(psi) - v * np.sin(psi) 
    y_dot = u * np.sin(psi) + v * np.cos(psi)
    # Project inertial velocity onto the cross-track direction
    ye_dot = np.cross(wp_unit_vec, np.array([x_dot, y_dot, 0.0]))[2]

    # --- Controller Calculation --- 
    # Outer loop (Guidance Law - LOS): Determine desired heading based on cross-track error
    # Kp_out = 1.0  # Proportional gain for outer loop (LOS lookahead distance relates to this)
    # Ki_out = 0.0  # Integral gain for outer loop (not used effectively)
    
    # Lookahead distance (Delta) based on Kp_out - classic LOS guidance parameter
    # Delta = 1/Kp_out # If Kp_out = 1, Delta = 1 meter. Might need tuning.
    Delta = max(1.0, 2*np.linalg.norm([u,v])) # Example: Lookahead distance based on speed (e.g., 2x vessel lengths if L=1)
    
    # Calculate desired heading (psi_des) using the LOS guidance law
    psi_des = pi_p - np.arctan2(ye, Delta) # atan2 handles signs correctly

    # Desired yaw rate (r_des) - Currently set to zero, suggesting a heading controller focus
    # r_des = Kp_out * ye_dot + Ki_out * ye # Original calculation, likely unused
    r_des = 0.0

    # Inner loop (Heading Controller - P controller): Determine rudder angle based on heading error
    Kp_in = -1.0  # Proportional gain for heading error (Negative sign likely due to rudder convention)
    # Kd_in = 0.0  # Derivative gain for damping yaw rate error (Could be added: Kd_in * (r_des - r))
    
    # Calculate heading error using smallest signed angle
    heading_error = ssa(psi_des - psi)
    
    # Calculate commanded rudder angle (proportional control on heading error)
    delta_c = Kp_in * heading_error # + Kd_in * (r_des - r) # Potential PD control
    
    # Print debug information (optional)
    # warnings.warn(f"psi des: {psi_des*180/np.pi:.2f} ye :{ye:.2f} psi: {psi*180/np.pi:.2f} delta_c {delta_c*180/np.pi:.2f}")

    # --- Output and Waypoint Switching --- 
    
    # Limit the rudder command to physical constraints (e.g., +/- 35 degrees)
    max_rudder_angle = 35.0 * np.pi / 180.0
    delta_c = clip(delta_c, -max_rudder_angle, max_rudder_angle)

    # Check for waypoint arrival: Calculate distance to the target waypoint
    wp_dist = np.linalg.norm(np.array([x - wp_xn, y - wp_yn])) # Only consider horizontal distance
    acceptance_radius = 5.0 # Radius within which the waypoint is considered reached (tune this) 
    
    # If within acceptance radius, switch to the next waypoint
    if wp_dist < acceptance_radius:
        waypoint_idx += 1
        # Reset integral term if it were used
        # ye_int = 0.0 
        print(f"Waypoint {waypoint_idx-1} reached. Switching to waypoint {waypoint_idx}.")

    # Return the commanded rudder angle (negated), cross-track error, and next waypoint index
    # The negative sign might be specific to how the vessel model interprets rudder angle.
    return -delta_c, ye, waypoint_idx