"""
File: class_vessel.py
Description: This file defines the Vessel class which is the core component of the MAV simulator.
             It models a marine autonomous vessel with complete 6-DOF dynamics including:
             - Mass and inertia properties
             - Hydrodynamic forces (added mass, damping)
             - Control forces (rudder, thruster)
             - Gravitational and buoyancy forces
             - Coriolis and centripetal effects
             
             The class provides methods for:
             - Initializing vessel parameters
             - Simulating vessel dynamics through ODE integration
             - Computing various force components
             - Handling vessel state and control inputs
             - Supporting ROS2 integration for distributed simulation
             
Author: MAV Simulator Team
"""

from typing import Dict, Optional, List
import numpy as np
from scipy.integrate import solve_ivp
import pandas as pd
import sys
from mav_simulator.module_kinematics import Smat, clip, eul_to_rotm, eul_rate_matrix, eul_to_quat, ssa
import mav_simulator.module_control as con
from mav_simulator.calculate_hydrodynamics import CalculateHydrodynamics
from mav_simulator.terminalMessages import print_debug, print_info, print_warning, print_error

class Vessel:
    """A class representing a marine vessel with its dynamics.
    
    Attributes:
        g (float): Gravitational acceleration
        rho (float): Water density
        L (float): Vessel length
        U (float): Forward speed
        mass (float): Vessel mass
        cg (np.ndarray): Center of gravity coordinates
        ...
    
    Example:
        >>> vessel_params = {...}  # vessel parameters
        >>> hydro_data = {...}     # hydrodynamic coefficients
        >>> vessel = Vessel(vessel_params, hydro_data)
        >>> vessel.simulate()
    """

    vessel_node = None
    delta_c = None
    n_c = None
    
    def __init__(self, vessel_params: Dict, hydrodynamic_data: Dict, vessel_id: int):
        """Initialize vessel with parameters and hydrodynamic data.
        
        Args:
            vessel_params: Dictionary containing vessel parameters
            hydrodynamic_data: Dictionary containing hydrodynamic coefficients
        """

        self.active_dof = vessel_params['active_dof']
        self.maintain_speed = vessel_params['maintain_speed']
        
        self.vessel_config = vessel_params
        self.vessel_name = vessel_params['name']
        self.vessel_id = vessel_id

        self.gps_datum = vessel_params['gps_datum']

        # Force direction mapping
        self.force_indices = {'X': 0, 'Y': 1, 'Z': 2, 'K': 3, 'M': 4, 'N': 5}

        # Base vessel parameters
        self.g = vessel_params['gravity']
        self.rho = vessel_params['density'] 
        self.L = vessel_params['geometry']['length']
        self.B = vessel_params['geometry']['breadth'] 
        self.D = vessel_params['geometry']['depth']
        self.U = vessel_params.get('U', 0.0)  # Default to 0 if not specified
        

        self.coriolis_flag = hydrodynamic_data.get('coriolis_flag',False)
        
        # Mass parameters
        self.mass = vessel_params['inertia']['mass']
        self.CG = vessel_params['geometry']['CG']['position']
        self.gyration = vessel_params['geometry']['gyration']
        
        if vessel_params['inertia']['inertia_matrix'] == "None":
            hydrodynamics = CalculateHydrodynamics()
            self.mass_matrix = hydrodynamics._generate_mass_matrix(self.CG,self.mass,self.gyration)
            print_info(f"Dimensionalized mass matrix: {self.mass_matrix}")
        else:
            self.mass_matrix = vessel_params['inertia']['inertia_matrix']

        if vessel_params['inertia']['added_mass_matrix'] == "None":
            hydrodynamics = CalculateHydrodynamics()
            dim_A = hydrodynamics.calculate_added_mass_from_hydra(hydrodynamic_data['hydra_file'])
            nonDim_A = dim_A.copy()
            nonDim_A[0:3][:, 0:3] = nonDim_A[0:3][:, 0:3] / (0.5 * self.rho * ((self.L ) ** 3))
            nonDim_A[0:3][:, 3:6] = nonDim_A[0:3][:, 3:6] / (0.5 * self.rho * ((self.L ) ** 4))
            nonDim_A[3:6][:, 0:3] = nonDim_A[3:6][:, 0:3] / (0.5 * self.rho * ((self.L ) ** 4))
            nonDim_A[3:6][:, 3:6] = nonDim_A[3:6][:, 3:6] / (0.5 * self.rho * ((self.L ) ** 5))
            print_info(f"Non-dimensionalized added mass matrix: {nonDim_A}")
            print_info(f"Dimensionalized added mass matrix: {dim_A}")
            if self.coriolis_flag==False:
               print_warning("Coriolis Matrix will not be used for Dynamics since flag set False")
            self.added_mass_matrix = dim_A
        else:
            self.added_mass_matrix = vessel_params['inertia']['added_mass_matrix']


        # Buoyancy parameters
        self.W = self.mass * self.g  # Weight
        self.buoyancy_mass = vessel_params['inertia']['buoyancy_mass']  # Buoyancy force, default to neutral
        self.B = self.buoyancy_mass * self.g  # Buoyancy force, default to neutral
        self.CB = vessel_params['geometry']['CB']['position']  # Center of buoyancy location relative to body frame

        # Dimensionalization flag
        self.dim_flag = hydrodynamic_data.get('dim_flag', False)

       
        # Initialize hydrodynamics dictionary
        self.hydrodynamics = {}
        
        # Extract hydrodynamic coefficients
        # Dynamically set hydrodynamic coefficients from data
        for coeff_name, coeff_value in hydrodynamic_data.items():
            if coeff_name != 'dim_flag' and coeff_name != 'hydra_file':  # Skip the dimensionalization flag and hydra file
                # Store in the hydrodynamics dictionary
                self.hydrodynamics[coeff_name] = coeff_value
                # Also set as attribute for backward compatibility
                setattr(self, coeff_name, coeff_value)

        # Dimensionalize if needed
        if not self.dim_flag:
            self._dimensionalize_coefficients(self.rho, self.L, self.U)
        
      
        # Load NACA airfoil data
        try:
            self.naca_data = pd.read_csv(vessel_params['control_surfaces']['naca_file'])
        except:
            print_warning("NACA airfoil data file path not specified in vessel parameters, will use control surface hydrodynamics if specified")
            pass
        
        # Determine state vector size based on attitude representation and actuators
        self.use_quaternion = vessel_params['initial_conditions'].get('use_quaternion', False)
        attitude_size = 4 if self.use_quaternion else 3        


        self.n_control_surfaces = 0
        self.n_thrusters = 0
        
        try:
            if vessel_params['control_surfaces'] is not None:
                self.control_surfaces = vessel_params['control_surfaces']
                self.n_control_surfaces = len(self.control_surfaces['control_surfaces']) if 'control_surfaces' in self.control_surfaces else 0
                print_info(f"Control surfaces for vessel {self.vessel_name}: {self.n_control_surfaces}")
            else:
                self.n_control_surfaces = 0
                print_warning(" No control surfaces specified in vessel parameters")
        except:
            print_warning(" No control surfaces specified in vessel parameters")
            self.n_control_surfaces = 0

        try:
            if vessel_params['thrusters'] is not None:
                self.thrusters = vessel_params['thrusters']
                self.n_thrusters = len(self.thrusters['thrusters']) if 'thrusters' in self.thrusters else 0
                print_info(f"Thrusters for vessel {self.vessel_name}: {self.n_thrusters}")
            else:
                self.n_thrusters = 0
                print_warning(" No thrusters specified in vessel parameters")
        except:
            print_warning(" No thrusters specified in vessel parameters")
            self.n_thrusters = 0

        # Build initial state vector
        initial_velocity = vessel_params['initial_conditions']['start_velocity']
        initial_position = vessel_params['initial_conditions']['start_location']
        initial_orientation = vessel_params['initial_conditions']['start_orientation']
        if self.use_quaternion:
            # Convert Euler angles to quaternion if needed
            initial_orientation = eul_to_quat(initial_orientation)
            
        initial_control = np.zeros(self.n_control_surfaces)  # Initial control surface angles
        initial_thrust = np.zeros(self.n_thrusters)  # Initial thruster states
        
        self.current_state = np.concatenate([
            initial_velocity,
            initial_position,
            initial_orientation,
            initial_control,
            initial_thrust
        ])
        
        self.initial_state = self.current_state.copy()
        
        # Initialize state derivative vector
        self.current_state_der = np.zeros_like(self.current_state)
        
        # Store total state size for history allocation
        self.state_size = len(self.current_state)
                
        self.Tmax = vessel_params['sim_time']
        self.dt = vessel_params['time_step']
        self.t = 0.0
        
        # Initialize commanded values
        self.delta_c = np.zeros(self.n_control_surfaces)
        self.n_c = np.zeros(self.n_thrusters)

        # Initialize history with pre-allocated array based on simulation time
        num_timesteps = int(self.Tmax / self.dt) + 2
        self.history = np.zeros((num_timesteps, self.state_size))  # Dynamic state size
        self.history[0, :] = self.current_state  # Store initial state
        self.time_index = 1  # Index to track position in history array

    def _dimensionalize_coefficients(self, rho, L, U):
        """Convert non-dimensional coefficients to dimensional form.
        
        This function processes all hydrodynamic coefficients and applies the appropriate
        dimensionalization factors based on the coefficient type.
        
        Args:
            rho (float): Water density
            L (float): Characteristic length (vessel length)
            U (float): Characteristic velocity
        """
        # Skip if coefficients are already dimensional
        if self.dim_flag:
            return
            
        # Process each coefficient in the hydrodynamics dictionary
        for coeff_name, coeff_value in self.hydrodynamics.items():
            # Skip if coefficient is zero or not a valid name format
            if coeff_value == 0 or '_' not in coeff_name:
                continue
                
            # Split into force direction and components
            parts = coeff_name.split('_')
            if len(parts) < 2:  # Need at least force direction and one component
                continue
                
            force_dir = parts[0]  # X, Y, Z, K, M, or N
            components = parts[1:]  # remaining components
            
            # Skip if not a valid force direction
            if force_dir not in self.force_indices:
                continue
                
            # Determine the dimensionalization factor based on the components
            factor = 0.5 * rho
            
            # Base L power depends on force direction (X,Y,Z vs K,M,N)
            if force_dir in ['X', 'Y', 'Z']:
                L_power = 2
            else:  # K, M, N
                L_power = 3
            
            # Add additional L power based on components
            for comp in components[1:]:
                if comp in ['p', 'q', 'r']:
                    L_power += 1
                
            # Apply L^n factor
            factor *= L**L_power
            
            # Apply U factor based on velocity components
            U_power = 0
            for comp in components:
                if comp in ['u', 'v', 'w', 'p', 'q', 'r']:
                    U_power += 1
                    
            factor *= U**U_power
            
            # Apply the dimensionalization factor to both the attribute and dictionary
            dimensionalized_value = coeff_value * factor
            print_info(f"Dimensionalized {coeff_name}: {dimensionalized_value}")
            
            # Update both the attribute and the dictionary entry
            setattr(self, coeff_name, dimensionalized_value)
            self.hydrodynamics[coeff_name] = dimensionalized_value

    def vessel_ode(self, t, state):
        """Vessel ordinary differential equations.
        
        The state vector is structured as follows:
        - state[0:6]: Velocities [u, v, w, p, q, r]
        - state[6:9]: Positions [x, y, z]
        - state[9:12]: Euler angles [phi, theta, psi] OR
        - state[9:13]: Quaternions [q0, q1, q2, q3]
        - state[control_start:thruster_start]: Control surface angles
        - state[thruster_start:]: Thruster states (RPM)
        
        Args:
            t (float): Current time
            state (np.ndarray): Current state vector
            
        Returns:
            np.ndarray: State derivatives
        """
        # Determine state vector structure based on quaternion vs euler
        use_quaternion = hasattr(self, 'use_quaternion') and self.use_quaternion
        attitude_size = 4 if use_quaternion else 3
        attitude_end = 9 + attitude_size
        
        # Calculate indices for control surfaces and thrusters
        control_start = attitude_end
        thruster_start = control_start + self.n_control_surfaces
        
        state[0:6] = state[0:6] * self.active_dof
        state[9:12] = state[9:12] * self.active_dof[3:6]
        # Extract state components
        vel = state[0:6]  # [u, v, w, p, q, r]
        pos = state[6:9]  # [x, y, z]
        
        if use_quaternion:
            quat = state[9:13]
            angles = eul_to_rotm(quat)
        else:
            angles = state[9:attitude_end]  # [phi, theta, psi]
        
      
        # Initialize state derivative vector
        state_dot = np.zeros_like(state)
        
        # Calculate forces and moments
        F_hyd = self.hydrodynamic_forces(vel)
        if self.n_control_surfaces > 0:
            F_control = self.control_forces(state[control_start:thruster_start])
        else:
            F_control = np.zeros(6)
        if self.n_thrusters > 0:
            F_thrust = self.thruster_forces(state[thruster_start:])
        else:
            F_thrust = np.zeros(6)
        F_g = self.gravitational_forces(angles[0], angles[1])  # Add gravitational forces        
        
        # Calculate mass matrices
        M_RB = self.mass_matrix
        M_A = self.added_mass_matrix
        if self.coriolis_flag:
          C_RB, C_A = self.calculate_coriolis_matrices(vel)
        
        # Check if added mass is too large compared to rigid body mass
        if np.any(np.abs(M_A) > 2 * np.abs(M_RB)):
            M = M_RB  # Ignore added mass if too large
            if self.coriolis_flag:
              C_A = np.zeros_like(C_A)  # Ignore Coriolis added mass if too large
        else:
            M = M_RB + M_A

        # Calculate Coriolis forces
        F_C = np.zeros_like(F_g)
        if self.coriolis_flag:
         F_C = (C_RB + C_A) @ vel
      
        # Calculate total force vector
        F = F_hyd + F_control + F_thrust - F_g - F_C

        # Calculate velocity derivatives

        M = M_RB
        state_dot[0:6] = np.linalg.inv(M) @ F
        
        # Calculate position derivatives
        if use_quaternion:
            state_dot[6:9] = eul_to_rotm(quat) @ vel[0:3]
            state_dot[9:13] = eul_rate_matrix(angles) @ vel[3:6]
        else:
            state_dot[6:9] = eul_to_rotm(angles) @ vel[0:3]
            state_dot[9:attitude_end] = eul_rate_matrix(angles) @ vel[3:6]
        
        # Calculate control surface derivatives using individual time constants
        if self.n_control_surfaces > 0:
            delta_c = np.array(self.delta_c) if isinstance(self.delta_c, (list, np.ndarray)) else np.array([self.delta_c])
            for i in range(self.n_control_surfaces):
                # Get time constant and limits for this control surface
                T = self.control_surfaces['control_surfaces'][i]['control_surface_T']
                delta_max = self.control_surfaces['control_surfaces'][i]['control_surface_delta_max']
                deltad_max = self.control_surfaces['control_surfaces'][i]['control_surface_deltad_max']
                
                # Limit commanded angle
                delta_c[i] = np.clip(delta_c[i], -delta_max, delta_max)
                
                # Calculate derivative
                state_dot[control_start + i] = (delta_c[i] - state[control_start + i]) / T
                
                # Apply rate limiting
                state_dot[control_start + i] = np.clip(state_dot[control_start + i], 
                                                     -deltad_max, 
                                                     deltad_max)
        # Calculate thruster derivatives
        if self.n_thrusters > 0:
            n_c = self.n_c if hasattr(self, 'n_c') else np.zeros(self.n_thrusters)
            for i in range(self.n_thrusters):
                state_dot[thruster_start + i] = (n_c[i] - state[thruster_start + i]) / 1.0 # TODO: Add time constant
                # Apply rate limiting
                if hasattr(self, 'nd_max'):
                    state_dot[thruster_start + i] = np.clip(state_dot[thruster_start + i], 
                                                          -self.nd_max, 
                                                          self.nd_max)
        

        state_dot[0:6] = state_dot[0:6] * self.active_dof
        state_dot[9:12] = state_dot[9:12] * self.active_dof[3:6]
        
        return state_dot
    
    def hydrodynamic_forces(self, vel):
        """Calculate hydrodynamic forces and moments
        
        Args:
            vel (array): Velocity vector [u, v, w, p, q, r]
            vel_dot (array, optional): Acceleration vector [u_dot, v_dot, w_dot, p_dot, q_dot, r_dot]
            
        Returns:
            array: Forces and moments vector [X, Y, Z, K, M, N]
        """
        # Extract velocities
        u, v, w, p, q, r = vel
        
        # Initialize forces vector
        F = np.zeros(6)
                    
        # Dictionary mapping velocity components to values
        if self.maintain_speed:
            vel_map = {'u': u - self.U, 'v': v, 'w': w, 'p': p, 'q': q, 'r': r}
        else:
            vel_map = {'u': u, 'v': v, 'w': w, 'p': p, 'q': q, 'r': r}
            
       
        # Process each hydrodynamic coefficient from the vessel's hydrodynamics dictionary
        for coeff_name, coeff_value in self.hydrodynamics.items():
            # Skip zero coefficients since they won't contribute to forces
            if coeff_value == 0:
                continue
                
            # Split coefficient name into force direction and velocity components
            parts = coeff_name.split('_')
            if len(parts) < 2:
                continue
                
            # Extract force direction (X,Y,Z,K,M,N)
            force_dir = parts[0]
            # Skip if not a valid force direction
            if force_dir not in self.force_indices:
                continue
                
            # Calculate the force component by multiplying coefficient with velocity terms
            force = coeff_value
            
            # Handle velocity and absolute value components
            for i in range(1, len(parts)):
                vel_char = parts[i]
                
                # Handle absolute multiplication (e.g., X_u_au -> u*abs(u))
                if vel_char.startswith('a') and len(vel_char) > 1 and vel_char[1:] in vel_map:
                    # Extract the velocity component without the 'a' prefix
                    v_char = vel_char[1:]
                    # Multiply by absolute value of velocity
                    force *= abs(vel_map[v_char])
                # Handle regular velocity components
                elif vel_char in vel_map:
                    force *= vel_map[vel_char]
            
            # Add the calculated force to the appropriate component in the force vector
            F[self.force_indices[force_dir]] += force
                
        return F

    def control_forces(self, delta, stall_angle=15, Cl=None, Cd=None):
        """Calculate control forces and moments
        
        Args:
            delta (float): Rudder angle in radians
            
        Returns:
            array: Forces and moments vector [X, Y, Z, K, M, N]
        """
        # Initialize control forces vector
        tau = np.zeros(6)

        # Extract velocities from current state
        u = self.current_state[0]
        v = self.current_state[1] 
        w = self.current_state[2]
        p = self.current_state[3]
        q = self.current_state[4]
        r = self.current_state[5]


        # Then calculate forces from control surfaces

        for surface in self.control_surfaces['control_surfaces']:

            max_delta = np.deg2rad(surface['control_surface_delta_max'])

            # If control surface has hydrodynamic coefficients, calculate forces from them
            if surface['control_surface_hydrodynamics']!= 'None':
                    delta = clip(ssa(delta[surface['control_surface_id'] - 1]),max_delta)
                    for coeff_name, coeff_value in surface['control_surface_hydrodynamics'].items():
                        if 'delta' in coeff_name:
                             if not self.dim_flag:   
                                force_pow = {'X':2,'Y':2,'Z':2,'K':3,'M':3,'N':3}
                                factor = 0.5 * self.rho * self.L**force_pow[coeff_name.split('_')[0]] * self.U**2
                                coeff_value = coeff_value * factor
                        force_dir = coeff_name.split('_')[0]
                        if force_dir in self.force_indices:
                            tau[self.force_indices[force_dir]] += coeff_value * delta

            else:
                # If control surface has no hydrodynamic coefficients, calculate forces from Aerofoil data
                
                # Get surface parameters
                sd = surface['control_surface_location']  # [x,y,z] coordinates in body frame
                area = surface['control_surface_area']
                
                # Create rotation matrix from Euler angles
                phi, theta, psi = surface['control_surface_orientation']
                cphi = np.cos(phi)
                sphi = np.sin(phi)
                cth = np.cos(theta)
                sth = np.sin(theta)
                cpsi = np.cos(psi)
                spsi = np.sin(psi)
                
                # Rotation matrix from surface frame to body frame
                R = np.array([
                    [cth*cpsi, -cth*spsi, sth],
                    [sphi*sth*cpsi + cphi*spsi, -sphi*sth*spsi + cphi*cpsi, -sphi*cth],
                    [-cphi*sth*cpsi + sphi*spsi, cphi*sth*spsi + sphi*cpsi, cphi*cth]
                ])

                # Calculate local velocities at control surface
                netU = u
                netV = v + r*sd[0] + p*sd[2]  # yaw and roll effects on sway
                netW = w + p*sd[1] - q*sd[0]  # roll and pitch effects on heave
            
                # Transform velocities to surface frame
                V_surface = R.T @ np.array([netU, netV, netW])
            
                # Get corresponding delta for this control surface based on surface ID
                surface_id = surface['control_surface_id']
                surface_delta = delta = clip(ssa(delta[surface['control_surface_id'] - 1]),max_delta) # Subtract 1 since IDs start at 1
                
                # Calculate effective angle of attack
                alpha = np.arctan2(V_surface[2], V_surface[0]) + surface_delta
                V_mag = np.sqrt(V_surface[0]**2 + V_surface[2]**2)
                
                # Calculate dynamic pressure
                q_dyn = 0.5 * self.rho * V_mag**2 * area
                
                # Check if angle of attack is near vertical
                # stall condition
                if abs(np.rad2deg(alpha)) > stall_angle:
                    L = 0.0 #Lift is zero at stall
                    D = q_dyn * 0.1  # Use a high drag coefficient (~1.2) for stalled flat plate
                else:
                    # Normal operation - calculate both lift and drag
                    if Cl is None or Cd is None:
                        alpha_deg = np.clip(np.rad2deg(alpha), 
                                        self.naca_data['Alpha'].min(), 
                                        self.naca_data['Alpha'].max())
                        Cl = np.interp(alpha_deg, self.naca_data['Alpha'], self.naca_data['Cl'])
                        Cd = np.interp(alpha_deg, self.naca_data['Alpha'], self.naca_data['Cd'])
                    L = q_dyn * Cl
                    D = q_dyn * Cd
                # Force vector in surface frame
                F_surface = np.array([-D, 0.0, -L])  

                # Transform forces to body frame
                F_body = R @ F_surface
                
                # Add forces
                tau[0:3] += F_body
                
                # Calculate and add moments
                tau[3:6] += np.cross(sd, F_body)

        return tau

    def thruster_forces(self, n_prop):
        """Calculate thruster forces and moments
        
        Args:
            n_prop (ndarray): Array of propeller RPMs
            
        Returns:
            array: Forces and moments vector [X, Y, Z, K, M, N]
        """
        # Initialize thruster forces vector
        tau = np.zeros(6)

        # Extract velocities from current state
        u = self.current_state[0]
        v = self.current_state[1] 
        w = self.current_state[2]
        p = self.current_state[3]
        q = self.current_state[4]
        r = self.current_state[5]

        # Convert input RPM to RPS
        n_prop = n_prop/60

        # For each thruster in the configuration
        for i, thruster in enumerate(self.thrusters['thrusters']):

            n_max = thruster['n_max']/60  # Convert to RPS
                 
            # Calculate thrust coefficient at zero advance ratio (J=0)
            KT_at_J0 = thruster['KT_at_J0']

            # Get thruster location and orientation
            td = thruster['thruster_location']  # [x,y,z] coordinates in body frame
            
            # Create rotation matrix from Euler angles
            phi, theta, psi = thruster['thruster_orientation']
            cphi = np.cos(phi)
            sphi = np.sin(phi)
            cth = np.cos(theta)
            sth = np.sin(theta)
            cpsi = np.cos(psi)
            spsi = np.sin(psi)
            
            # Rotation matrix from thruster frame to body frame
            R = np.array([
                [cth*cpsi, -cth*spsi, sth],
                [sphi*sth*cpsi + cphi*spsi, -sphi*sth*spsi + cphi*cpsi, -sphi*cth],
                [-cphi*sth*cpsi + sphi*spsi, cphi*sth*spsi + sphi*cpsi, cphi*cth]
            ])
            
            # Calculate local velocities at thruster
            netU = u
            netV = v + r*td[0] + p*td[2]  # yaw and roll effects on sway
            netW = w + p*td[1] - q*td[0]  # roll and pitch effects on heave
            
            # Transform velocities to thruster frame
            V_thruster = R.T @ np.array([netU, netV, netW])
            
            # Use the axial velocity component for advance ratio calculation
            u_thruster = V_thruster[0]
            
            # Calculate advance ratio J with protection against division by zero
            denominator = n_prop[i] * thruster['D_prop']
            if abs(denominator) > 1e-6:  # Check if denominator is not too close to zero
                J = u_thruster / denominator
            else:
                J = 0.0  # Set a default value when propeller is not rotating
                
            # Linear approximation of thrust coefficient
            KT = KT_at_J0 * (1 - J)
            
            # Calculate propeller thrust
            X_prop = KT * self.rho * thruster['D_prop']**4 * np.abs(n_prop[i]) * n_prop[i]
            
            # Apply thrust deduction if available
            if 'tp' in thruster:
                X_prop *= (1 - thruster['tp'])

            # Force vector in thruster frame (thrust acts along x-axis of thruster)
            F_thruster = np.array([X_prop, 0.0, 0.0])
            
            # Transform forces to body frame
            F_body = R @ F_thruster
            
            # Add forces
            tau[0:3] += F_body
            
            # Calculate and add moments
            tau[3:6] += np.cross(td, F_body)

        return tau
    
    def step(self):
        """Step the vessel forward in time"""        
        sol = solve_ivp(self.vessel_ode, [self.t, self.t + self.dt], self.current_state, method='RK45')        
        self.current_state = sol.y[:, -1]
        
        # Calculate state derivative at new state
        self.current_state_der = self.vessel_ode(self.t + self.dt, self.current_state)
        
        self.t = sol.t[-1]

        # print_debug(f"Current state at time {self.t:.2f}: {np.round(self.current_state, 1)}")

    def reset(self):
        """Reset the vessel to the initial state"""
        self.current_state = self.initial_state
        self.t = 0.0
    
    def simulate(self):
        """Simulate the vessel"""
        
        self.reset()
        while self.t < self.Tmax:
            self.step()
        
        # Trim history array to actual size
        self.history = self.history[:self.time_index, :]

    def gravitational_forces(self, phi, theta):
        """Calculate gravitational and buoyancy forces.
        
        Args:
            phi (float): Roll angle in radians
            theta (float): Pitch angle in radians
            
        Returns:
            np.ndarray: 6x1 vector of gravitational and buoyancy forces
        """
        sth = np.sin(theta)
        cth = np.cos(theta)
        sphi = np.sin(phi)
        cphi = np.cos(phi)
        
        # Calculate forces and moments
        g = np.array([
            (self.W - self.B) * sth,
            -(self.W - self.B) * cth * sphi,
            -(self.W - self.B) * cth * cphi,
            -(self.CG[1]*self.W - self.CB[1]*self.B) * cth * cphi + 
             (self.CG[2]*self.W - self.CB[2]*self.B) * cth * sphi,
            (self.CG[2]*self.W - self.CB[2]*self.B) * sth + 
            (self.CG[0]*self.W - self.CB[0]*self.B) * cth * cphi,
            -(self.CG[0]*self.W - self.CB[0]*self.B) * cth * sphi - 
             (self.CG[1]*self.W - self.CB[1]*self.B) * sth
        ])

        return g

    def calculate_coriolis_matrices(self, vel):
        """Calculate rigid-body and added mass Coriolis-centripetal matrices.
        
        Args:
            vel (np.ndarray): 6x1 velocity vector [u, v, w, p, q, r]
            
        Returns:
            tuple: (C_RB, C_A) where:
                - C_RB is the 6x6 rigid-body Coriolis-centripetal matrix
                - C_A is the 6x6 added mass Coriolis-centripetal matrix
        """
        # Split velocity vector into linear and angular components
        v1 = vel[0:3]  # Linear velocities [u, v, w]
        v2 = vel[3:6]  # Angular velocities [p, q, r]
        
        # Split mass matrix into blocks
        M_RB = self.mass_matrix
        M11  = M_RB[0:3, 0:3]
        M12  = M_RB[0:3, 3:6]
        M21  = M_RB[3:6, 0:3]
        M22  = M_RB[3:6, 3:6]
        
        # Calculate rigid-body Coriolis matrix
        C_RB = np.zeros((6, 6))
        C_RB[0:3, 3:6] = -Smat(M11 @ v1 + M12 @ v2)
        C_RB[3:6, 0:3] = -Smat(M11 @ v1 + M12 @ v2)
        C_RB[3:6, 3:6] = -Smat(M21 @ v1 + M22 @ v2)
        
        # Split added mass matrix into blocks
        M_A = self.added_mass_matrix
        A11 = M_A[0:3, 0:3]
        A12 = M_A[0:3, 3:6]
        A21 = M_A[3:6, 0:3]
        A22 = M_A[3:6, 3:6]
        
        # Calculate added mass Coriolis matrix
        C_A = np.zeros((6, 6))
        C_A[0:3, 3:6] = -Smat(A11 @ v1 + A12 @ v2)
        C_A[3:6, 0:3] = -Smat(A11 @ v1 + A12 @ v2)
        C_A[3:6, 3:6] = -Smat(A21 @ v1 + A22 @ v2)
        
        return C_RB, C_A