"""
File: module_control.py
Description: This file provides control strategies for marine autonomous vessels in the simulator.
             The module implements various control functions for:
             
             - Rudder/control surface angle commands:
               * Fixed angle control for maintaining constant rudder angles
               * Switching angle control for alternating between different angles
               
             - Thruster/propeller control:
               * Fixed RPM control for constant thrust
               * Variable thrust patterns
               
             These control functions can be used as building blocks for more complex
             control strategies or as simple test inputs for vessel dynamics validation.
             The functions take the current simulation time and vessel state as inputs
             and return appropriate control commands.
             
Author: MAV Simulator Team
"""

import numpy as np

"""
Control module for vessel simulation.

This module provides different control strategies for vessel steering:
- Fixed angle control for control surfaces
- Switching (alternating) angle control for control surfaces
- Fixed RPM control for thrusters
- Switching RPM control for thrusters
"""

def fixed_rudder(t, state, n_control_surfaces=1, rudder_angle=15.0):
    """Return constant control surface angle commands.
    
    Args:
        t (float): Current simulation time [s]
        state (ndarray): Current vessel state vector
        n_control_surfaces (int): Number of control surfaces
        rudder_angle (float): Commanded angle in degrees
        
    Returns:
        ndarray: Commanded control surface angles in radians
        
    Example:
        >>> fixed_rudder(10.0, state_vector, 2)
        array([0.2618, 0.2618])  # 15 degrees in radians for each surface
    """
    delta_c = np.ones(n_control_surfaces) * (rudder_angle * np.pi / 180.0)
    return delta_c

def switching_rudder(t, state, n_control_surfaces=1):
    """Return alternating control surface angle commands that switch every 10 seconds.
    
    Args:
        t (float): Current simulation time [s]
        state (ndarray): Current vessel state vector
        n_control_surfaces (int): Number of control surfaces
        
    Returns:
        ndarray: Commanded control surface angles in radians
        
    Example:
        >>> switching_rudder(5.0, state_vector, 2)
        array([0.2618, 0.2618])  # +15 degrees for all surfaces
        >>> switching_rudder(15.0, state_vector, 2)
        array([-0.2618, -0.2618])  # -15 degrees for all surfaces
    """
    period = t % 20  # 20 sec for complete cycle (10 sec each direction)
    if period < 10:
        angle = 15.0
    else:
        angle = -15.0
        
    delta_c = np.ones(n_control_surfaces) * (angle * np.pi / 180.0)
    return delta_c

def fixed_thrust(t, state, n_thrusters=1, rpm=1000.0):
    """Return constant thruster RPM commands.
    
    Args:
        t (float): Current simulation time [s]
        state (ndarray): Current vessel state vector
        n_thrusters (int): Number of thrusters
        rpm (float): Commanded RPM
        
    Returns:
        ndarray: Commanded thruster RPMs
        
    Example:
        >>> fixed_thrust(10.0, state_vector, 2, 1000.0)
        array([1000.0, 1000.0])
    """
    n_c = np.ones(n_thrusters) * rpm
    return n_c
