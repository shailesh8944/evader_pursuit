import numpy as np

"""
Rudder control module for vessel simulation.

This module provides different rudder control strategies for vessel steering:
"""

def ssa(ang, deg=False):
    """
    Smallest signed angle that lies between -pi and pi.
    If deg is True, the angle is assumed to be in degrees and is converted to radians.
    If deg is False, the angle is assumed to be in radians.

    Args:
        ang (float): Angle in degrees or radians
        deg (bool): If True, the angle is assumed to be in degrees

    Returns:
        float: Smallest signed angle between -pi and pi. The angle is in radians if deg is False, and in degrees if deg is True.
    """

    #===========================================================================
    if deg:
        ang = (ang + 180) % (360.0) - 180.0
    else:
        ang = (ang + np.pi) % (2 * np.pi) - np.pi
    #===========================================================================

    return ang

def clip(val, min_val, max_val):
    """
    Clip a value to a range.
    """
    return max(min_val, min(val, max_val))


def pid_control(t, state, waypoints, waypoint_idx, ye_int=0.0):
    """
    Implement a PID control strategy to follow the waypoints. 
    
    Args:
        t (float): Current simulation time [s]
        state (ndarray): Current vessel state vector
        waypoints (ndarray): Waypoints array
        waypoint_idx (int): Current waypoint index
        ye_int (float): Integral term of cross-track error
        
    Returns:
        float: Commanded rudder angle in radians
        int: Next waypoint index
    """

    # Check if we have reached the last waypoint
    if waypoint_idx == len(waypoints):
        return 0.0, waypoint_idx

    # Current state
    u, v, r = state[3], state[4], state[5]
    x, y, psi = state[6], state[7], state[11]
    
    # Current and previous waypoints
    wp_xn, wp_yn, _ = waypoints[waypoint_idx]
    wp_xn1, wp_yn1, _ = waypoints[waypoint_idx - 1]

    delta_c = 0.0
    ye = 0.0
    
    #===========================================================================
    
    # Unit vector of waypoint line
    wp_unit_vec = np.array([wp_xn - wp_xn1, wp_yn - wp_yn1, 0.0])
    wp_unit_vec = wp_unit_vec / np.linalg.norm(wp_unit_vec)

    # Vector from previous waypoint to current position
    r_vec = np.array([x - wp_xn1, y - wp_yn1, 0.0])

    # Along-track and cross-track errors
    xe = np.dot(r_vec, wp_unit_vec)
    ye = np.cross(wp_unit_vec, r_vec)[2]

    # Derivative of cross-track error
    x_dot, y_dot = u * np.cos(psi) - v * np.sin(psi), u * np.sin(psi) + v * np.cos(psi)
    ye_dot = np.cross(wp_unit_vec, np.array([x_dot, y_dot, 0.0]))[2]

    # PID controller parameters
    Kp_out = 0.5  # Proportional gain for outer loop
    Ki_out = 0.1  # Integral gain for outer loop
    
    psi_des = Kp_out * (0.0 - ye) + Ki_out * (0.0 - ye_int)
    r_des = Kp_out * ye_dot + Ki_out * ye
    r_des = 0.0

    # psi_des = 90.0 * np.pi / 180.0;

    Kp_in = 0.5  # Proportional gain for inner loop
    Kd_in = 0.2  # Derivative gain for inner loop

    delta_c = Kp_in * ssa(psi_des - psi) + Kd_in * (r_des - r)

    #===========================================================================
    
    # Clip the rudder angle
    delta_c = clip(delta_c, -35*np.pi/180, 35*np.pi/180)

    # Distance to waypoint
    wp_dist = np.linalg.norm(np.array([x - wp_xn, y - wp_yn, 0.0]))
    if wp_dist < 0.5:
        waypoint_idx += 1

    # Do not change the following line (the negative sign is due to the sign convention of the rudder angle)
    return -delta_c, ye, waypoint_idx