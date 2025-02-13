from typing import Dict, Optional, List
import numpy as np
from scipy.integrate import solve_ivp
import pandas as pd
import sys
sys.path.append('/workspaces/mavlab/')
from module_kinematics import Smat, eul_to_rotm, eul_rate_matrix, eul_to_quat
import module_control as con

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
    
    def __init__(self, vessel_params: Dict, hydrodynamic_data: Dict, ros_flag: bool = False):
        """Initialize vessel with parameters and hydrodynamic data.
        
        Args:
            vessel_params: Dictionary containing vessel parameters
            hydrodynamic_data: Dictionary containing hydrodynamic coefficients
        """

        # ROS flag
        self.ros_flag = ros_flag

        # Force direction mapping
        self.force_indices = {'X': 0, 'Y': 1, 'Z': 2, 'K': 3, 'M': 4, 'N': 5}

        # Base vessel parameters
        self.g = vessel_params['gravity']
        self.rho = vessel_params['density'] 
        self.L = vessel_params['geometry']['length']
        self.B = vessel_params['geometry']['breadth'] 
        self.D = vessel_params['geometry']['depth']
        self.U = vessel_params.get('U', 0.0)  # Default to 0 if not specified
        
        # Mass parameters
        self.mass = vessel_params['mass']
        self.CG = vessel_params['cg']
        self.gyration = vessel_params['gyration']
        
        # Buoyancy parameters
        self.W = self.mass * self.g  # Weight
        self.buoyancy_mass = vessel_params.get('buoyancy_mass', self.W)  # Buoyancy force, default to neutral
        self.B = self.buoyancy_mass * self.g  # Buoyancy force, default to neutral
        self.CB = vessel_params['cb']  # Center of buoyancy location relative to body frame

        # Generate mass matrix
        self._generate_mass_matrix()

        # Dimensionalization flag
        self.dim_flag = hydrodynamic_data.get('dim_flag', False)

        # Initialize hydrodynamics dictionary
        self.hydrodynamics = {}
        
        # Extract hydrodynamic coefficients
        # Dynamically set hydrodynamic coefficients from data
        for coeff_name, coeff_value in hydrodynamic_data.items():
            if coeff_name != 'dim_flag':  # Skip the dimensionalization flag
                # Store in the hydrodynamics dictionary
                self.hydrodynamics[coeff_name] = coeff_value
                # Also set as attribute for backward compatibility
                setattr(self, coeff_name, coeff_value)

        # Dimensionalize if needed
        if not self.dim_flag:
            self._dimensionalize_coefficients(self.rho, self.L, self.U)
        
        # Load NACA airfoil data
        if 'naca_file' in vessel_params:
            self.naca_data = pd.read_csv(vessel_params['naca_file'])
        else:
            raise ValueError("NACA airfoil data file path not specified in vessel parameters")
        
        # Determine state vector size based on attitude representation and actuators
        self.use_quaternion = vessel_params.get('use_quaternion', False)
        attitude_size = 4 if self.use_quaternion else 3

        self.control_surfaces = vessel_params.get('control_surfaces', [])
        n_control_surfaces = len(self.control_surfaces) if hasattr(self, 'control_surfaces') else 0

        self.thrusters = vessel_params.get('thrusters', [])
        n_thrusters = len(self.thrusters) if hasattr(self, 'thrusters') else 0
        
        # Build initial state vector
        initial_velocity = vessel_params['initial_velocity']
        initial_position = vessel_params['initial_position']
        initial_attitude = vessel_params['initial_attitude']
        if self.use_quaternion:
            # Convert Euler angles to quaternion if needed
            initial_attitude = eul_to_quat(initial_attitude)
            
        initial_control = np.zeros(n_control_surfaces)  # Initial control surface angles
        initial_thrust = np.zeros(n_thrusters)  # Initial thruster states
        
        self.current_state = np.concatenate([
            initial_velocity,
            initial_position,
            initial_attitude,
            initial_control,
            initial_thrust
        ])
        
        self.initial_state = self.current_state.copy()
        
        # Store total state size for history allocation
        self.state_size = len(self.current_state)
                
        self.Tmax = vessel_params['sim_time']
        self.dt = vessel_params['time_step']
        self.t = 0.0
        self.control_type = vessel_params['control_type']
        
        # Initialize commanded values
        self.delta_c = np.zeros(n_control_surfaces)
        self.n_c = np.zeros(n_thrusters)

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
            
        The dimensionalization follows standard naval architecture practices:
        - Linear velocity dependent terms: 0.5 * rho * L^2 * U
        - Angular velocity dependent terms: 0.5 * rho * L^3 * U
        - Linear acceleration dependent terms: 0.5 * rho * L^3
        - Angular acceleration dependent terms: 0.5 * rho * L^4
        - Control surface dependent terms: 0.5 * rho * L^2 * U^2
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
            force_dir, components = coeff_name.split('_')
            
            # Skip if not a valid force direction
            if force_dir not in self.force_indices:
                continue
                
            # Determine the dimensionalization factor based on the components
            # and force direction
            factor = 0.5 * rho
            
            # Count the number of velocity components (not counting 'a' for absolute)
            vel_components = sum(1 for c in components if c in 'uvwpqr')
            
            # Base factor on force direction (X,Y,Z vs K,M,N)
            # Moments (K,M,N) get an extra L factor
            base_L_power = 2 if force_dir in ['X', 'Y', 'Z'] else 3
            
            # Add L factors based on components
            # Each velocity component adds 1 to L power
            # Each acceleration component (d) adds 1 more to L power
            L_power = base_L_power
            L_power += vel_components
            if 'd' in components:  # Acceleration term
                L_power += 1
                
            # Add L^n factor
            factor *= L**L_power
            
            # Add U factor for velocity-dependent terms (not acceleration)
            if 'd' not in components:
                # Each velocity component contributes one U factor
                factor *= U**vel_components
                
            # Special case for control surface coefficients (delta)
            if 'delta' in components:
                factor = 0.5 * rho * L**base_L_power * U**2
                
            # Apply the dimensionalization factor
            setattr(self, coeff_name, coeff_value * factor)

    def _generate_mass_matrix(self):
    
        """Generate the mass matrix"""
        # Calculate gyration tensor about vessel frame
        self.xprdct2 = np.diag(self.gyration)**2 - Smat(self.CG)@Smat(self.CG)
        
        # Generate the inertia matrix (3 x 3)using radii of gyration
        self.inertia_matrix = self.xprdct2 * self.mass

        # Generate the mass matrix
        self.mass_matrix = np.zeros((6,6))
        self.mass_matrix[0:3][:, 0:3] = self.mass * np.eye(3)
        self.mass_matrix[3:6][:, 3:6] = self.inertia_matrix
        self.mass_matrix[0:3][:, 3:6] = -Smat(self.CG) * self.mass
        self.mass_matrix[3:6][:, 0:3] = Smat(self.CG) * self.mass

        #========================================================================

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
        n_control_surfaces = len(self.control_surfaces) if hasattr(self, 'control_surfaces') else 0
        thruster_start = control_start + n_control_surfaces
        n_thrusters = len(self.thrusters) if hasattr(self, 'thrusters') else 0
        
        # Extract state components
        vel = state[0:6]  # [u, v, w, p, q, r]
        pos = state[6:9]  # [x, y, z]
        
        if use_quaternion:
            quat = state[9:13]
            angles = eul_to_rotm(quat)
        else:
            angles = state[9:attitude_end]  # [phi, theta, psi]
            
        # Extract control surface angles and thruster states
        control_angles = state[control_start:thruster_start] if n_control_surfaces > 0 else np.array([])
        thruster_states = state[thruster_start:] if n_thrusters > 0 else np.array([])
        
        # Get commanded values
        if not self.ros_flag:
            if self.control_type == 'fixed_rudder':
                self.delta_c = con.fixed_rudder(t, state, n_control_surfaces, 5.0)
            elif self.control_type == 'switching_rudder':
                self.delta_c = con.switching_rudder(t, state, n_control_surfaces)
            else:
                raise ValueError(f"Invalid control type: {self.control_type}")
        
        # Initialize state derivative vector
        state_dot = np.zeros_like(state)
        
        # Calculate forces and moments
        F_hyd = self.hydrodynamic_forces(vel)
        F_control = self.control_forces(control_angles) if n_control_surfaces > 0 else np.zeros(6)
        #F_thrust = self.thruster_forces(thruster_states) if n_thrusters > 0 else np.zeros(6)
        F_g = self.gravitational_forces(angles[0], angles[1])  # Add gravitational forces        
        
        # Calculate mass matrices
        M_RB = self.mass_matrix
        M_A = self.added_mass_matrix()
        M = M_RB + M_A
        
        # Calculate total force vector
        #F = F_hyd + F_control + F_thrust + F_g
        F = F_hyd + F_g + F_control
        print(F_control)
        
        # Calculate velocity derivatives
        state_dot[0:6] = np.linalg.inv(M) @ F
        
        # Calculate position derivatives
        if use_quaternion:
            state_dot[6:9] = eul_to_rotm(quat) @ vel[0:3]
            state_dot[9:13] = eul_rate_matrix(angles) @ vel[3:6]
        else:
            state_dot[6:9] = eul_to_rotm(angles) @ vel[0:3]
            state_dot[9:attitude_end] = eul_rate_matrix(angles) @ vel[3:6]
        
        # Calculate control surface derivatives using individual time constants
        if n_control_surfaces > 0:
            delta_c = np.array(self.delta_c) if isinstance(self.delta_c, (list, np.ndarray)) else np.array([self.delta_c])
            for i in range(n_control_surfaces):
                # Get time constant and limits for this control surface
                T = self.control_surfaces[i]['control_surface_T']
                delta_max = self.control_surfaces[i]['control_surface_delta_max']
                deltad_max = self.control_surfaces[i]['control_surface_deltad_max']
                
                # Limit commanded angle
                delta_c[i] = np.clip(delta_c[i], -delta_max, delta_max)
                
                # Calculate derivative
                state_dot[control_start + i] = (delta_c[i] - control_angles[i]) / T
                
                # Apply rate limiting
                state_dot[control_start + i] = np.clip(state_dot[control_start + i], 
                                                     -deltad_max, 
                                                     deltad_max)
        # Calculate thruster derivatives
        if n_thrusters > 0:
            n_c = self.n_c if hasattr(self, 'n_c') else np.zeros(n_thrusters)
            for i in range(n_thrusters):
                state_dot[thruster_start + i] = (n_c[i] - thruster_states[i]) / self.T_prop
                # Apply rate limiting
                if hasattr(self, 'nd_max'):
                    state_dot[thruster_start + i] = np.clip(state_dot[thruster_start + i], 
                                                          -self.nd_max, 
                                                          self.nd_max)
        
        return state_dot

    def added_mass_matrix(self):
        """Calculate the added mass matrix"""
        # Initialize added mass matrix
        M_A = np.zeros((6, 6))

        # Calculate added mass matrix
        M_A[0, 0] = -self.X_ud
        M_A[1, 1] = -self.Y_vd
        M_A[1, 5] = -self.Y_rd
        M_A[5, 1] = -self.N_vd
        M_A[5, 5] = -self.N_rd

        return M_A
    
    def hydrodynamic_forces(self, vel):
        """Calculate hydrodynamic forces and moments
        
        Args:
            vel (array): Velocity vector [u, v, w, p, q, r]
            
        Returns:
            array: Forces and moments vector [X, Y, Z, K, M, N]
        """
        # Extract velocities
        u, v, w, p, q, r = vel
        
        # Initialize forces vector
        F = np.zeros(6)
                    
        # Dictionary mapping velocity components to values
        vel_map = {'u': u, 'v': v, 'w': w, 'p': p, 'q': q, 'r': r}
        
        # Process each hydrodynamic coefficient from the vessel's hydrodynamics dictionary
        # Example coefficients: X_uau (X-force dependent on u*|u|), Y_vav (Y-force dependent on v*|v|)
        for coeff_name, coeff_value in self.hydrodynamics.items():
            # Skip zero coefficients since they won't contribute to forces
            if coeff_value == 0:
                continue
                
            # Split coefficient name into force direction and velocity components
            # e.g., X_uau splits into ['X', 'uau'] where:
            # - X is the force direction 
            # - uau indicates multiplication of u * |u|
            parts = coeff_name.split('_')
            if len(parts) != 2:
                continue
                
            # Extract force direction (X,Y,Z,K,M,N) and velocity components
            force_dir, vel_components = parts
            # Skip if not a valid force direction
            if force_dir not in self.force_indices:
                continue
                
            # Calculate the force component by multiplying coefficient with velocity terms
            force = coeff_value
            for vel_char in vel_components:
                if vel_char in vel_map:
                    # Multiply by the velocity component (u,v,w,p,q,r)
                    force *= vel_map[vel_char]
                elif vel_char == 'a':
                    # 'a' indicates absolute value term
                    # e.g., in X_uau, multiply by |u| where u is the first velocity component
                    force *= abs(vel_map[vel_components[0]])
            
            # Add the calculated force to the appropriate component in the force vector
            # using the mapping from force direction to vector index
            F[self.force_indices[force_dir]] += force
                
        return F

    def control_forces(self, delta):
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

        # First calculate forces from hydrodynamic coefficients if they exist
        if hasattr(self, 'hydrodynamics'):
            for coeff_name, coeff_value in self.hydrodynamics.items():
                if 'delta' in coeff_name and coeff_value != 0:
                    force_dir = coeff_name.split('_')[0]
                    if force_dir in self.force_indices:
                        tau[self.force_indices[force_dir]] += coeff_value * delta

        # Then calculate forces from control surfaces
        for surface in self.control_surfaces:
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
        
            # Calculate angle of attack in surface frame
            # Get corresponding delta for this control surface based on surface index
            surface_idx = self.control_surfaces.index(surface)
            surface_delta = delta[surface_idx]
            
            # Calculate effective angle of attack
            alpha = np.arctan2(V_surface[2], V_surface[0]) + surface_delta
            V_mag = np.sqrt(np.sum(V_surface**2))
            
            # Calculate dynamic pressure
            q_dyn = 0.5 * self.rho * V_mag**2 * area
            
            # Check if angle of attack is near vertical
            if abs(np.rad2deg(alpha)) > 35:
                # For near-vertical motion, only consider lift force
                alpha_deg = np.clip(np.rad2deg(alpha), 
                                  self.naca_data['Alpha'].min(), 
                                  self.naca_data['Alpha'].max())
                Cl = np.interp(alpha_deg, self.naca_data['Alpha'], self.naca_data['Cl'])
                L = 0.0
                D = 0.0  # Ignore drag at high angles of attack
            else:
                # Normal operation - calculate both lift and drag
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
            n_prop (float): Propeller RPM
            
        Returns:
            array: Forces and moments vector [X, Y, Z, K, M, N]
        """
        # Initialize thruster forces vector
        tau = np.zeros(6)

        # Extract forward velocity from current state
        u = self.current_state[0]

        # Maximum RPM from T200 thruster specs
        n_max = 2668/60  # Convert to RPS

        # Calculate thrust coefficient at zero advance ratio (J=0)
        # 28.73 N thrust at 2668 RPM at 12V (from T200 datasheet)
        KT_at_J0 = 28.73/(1024 * self.D_prop**4 * n_max**2)

        # Calculate advance ratio J with protection against division by zero
        denominator = n_prop * self.D_prop
        if abs(denominator) > 1e-6:  # Check if denominator is not too close to zero
            J = u / denominator
        else:
            J = 0.0  # Set a default value when propeller is not rotating
            
        # Linear approximation of thrust coefficient
        KT = KT_at_J0 * (1 - J)
        
        # Calculate propeller thrust
        X_prop = KT * 1024 * self.D_prop**4 * np.abs(n_prop) * n_prop

        # Apply thrust deduction if available
        if hasattr(self, 'tp'):
            X_prop *= (1 - self.tp)

        # Add thrust force to surge direction
        tau[0] = X_prop

        # For each thruster in the configuration
        for thruster in self.thrusters:
            # Get thruster location relative to body frame
            pos = thruster['thruster_location']
            
            # Calculate moments due to thrust
            # Only X-direction force is considered as per the mavymini implementation
            tau[3:6] += np.cross(pos, np.array([X_prop, 0, 0]))

        return tau
    
    def step(self):
        """Step the vessel forward in time"""        
        sol = solve_ivp(self.vessel_ode, [self.t, self.t + self.dt], self.current_state, method='RK45')        
        self.current_state = sol.y[:, -1]
        self.t = sol.t[-1]

        # Store current state in history
        if not self.ros_flag:
            self.history[self.time_index, :] = self.current_state
            self.time_index += 1

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
