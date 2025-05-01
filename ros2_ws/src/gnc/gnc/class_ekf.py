"""
File: class_ekf.py
Description: Implements a standard Extended Kalman Filter (EKF) class for state estimation.
             Provides methods for initialization, prediction, and correction steps.
             Handles both linear and non-linear system models (using Jacobians or provided functions).
             Includes options for system discretization (Euler or matrix exponential).

Author: MAV GNC Team (adapted from standard EKF implementations)
"""

import numpy as np
import scipy.linalg # Used for matrix exponential and inverse
import warnings
from mav_simulator.module_kinematics import clip, ssa # Utility functions

class EKF():
    """Extended Kalman Filter Implementation.

    Handles state prediction and correction using sensor measurements.
    Assumes a state-space model structure, potentially non-linear.

    State dynamics (continuous-time, non-linear):
        x_dot = f(t, x, u) + E w,  where w ~ N(0, Q)
    Measurement model (discrete-time, non-linear):
        y_k = h(x_k) + v_k,        where v_k ~ N(0, R_k)

    Linearized versions used in EKF:
        x_dot ≈ A x + B u + E w   (A = df/dx, B = df/du)
        y_k ≈ C_k x_k + v_k       (C_k = dh/dx at x_k)

    Attributes:
        sampling_rate (float): Filter update rate in Hz.
        dt (float): Time step (1 / sampling_rate).
        t (float): Current filter time.
        n_states (int): Number of states in the state vector x.
        n_inp (int): Number of control inputs in the input vector u.
        n_pro_noise (int): Dimension of the process noise vector w.
        A (np.ndarray): State transition matrix (or Jacobian F) (n_states x n_states).
        B (np.ndarray): Control input matrix (or Jacobian G) (n_states x n_inp).
        E (np.ndarray): Process noise input matrix (n_states x n_pro_noise).
        Q (np.ndarray): Process noise covariance matrix (n_pro_noise x n_pro_noise).
        x (np.ndarray): Current state estimate vector (n_states x 1).
        P (np.ndarray): Current state estimate covariance matrix (n_states x n_states).
        Ad (np.ndarray): Discretized A matrix.
        Bd (np.ndarray): Discretized B matrix.
        Ed (np.ndarray): Discretized E matrix.
    """
    def __init__(
            self, 
            frequency, # Renamed sampling_rate to frequency for clarity 
            n_states=15, 
            n_inp=1, 
            pro_noise_cov=np.eye(6), 
        ):
        """Initializes the EKF.

        Args:
            frequency (float): The operating frequency of the filter in Hz.
            n_states (int, optional): Dimension of the state vector. Defaults to 15.
            n_inp (int, optional): Dimension of the control input vector. Defaults to 1.
            pro_noise_cov (np.ndarray, optional): Process noise covariance matrix (Q). 
                                                 Defaults to a 6x6 identity matrix. 
                                                 The size must match n_pro_noise inferred from E.
        """
        if frequency <= 0:
            raise ValueError("EKF frequency must be positive.")
            
        self.sampling_rate = frequency # Store original frequency if needed
        self.dt = 1.0 / frequency
        self.t = 0.0 # Initial time
        
        self.n_states = n_states
        self.n_inp = n_inp
        # Infer the dimension of process noise from the provided Q matrix
        if pro_noise_cov.ndim != 2 or pro_noise_cov.shape[0] != pro_noise_cov.shape[1]:
            raise ValueError("Process noise covariance Q must be a square matrix.")
        self.n_pro_noise = pro_noise_cov.shape[0]

        # Initialize system matrices (will often be updated in predict step)
        self.A = np.eye(self.n_states)
        self.B = np.zeros((self.n_states, self.n_inp))
        self.E = np.zeros((self.n_states, self.n_pro_noise))
        self.Q = pro_noise_cov # Process noise covariance
        
        # Initialize state and covariance
        self.x0 = np.zeros((self.n_states, 1)) # Initial state guess (zero)
        # Initial covariance: Large values indicate high initial uncertainty
        self.P0 = 10000.0 * np.eye(self.n_states) 
        
        self.x = self.x0.copy()
        self.P = self.P0.copy()

        # Discretized matrices (initialized later)
        self.Ad = np.eye(self.n_states)
        self.Bd = np.zeros_like(self.B)
        self.Ed = np.zeros_like(self.E)

        self.debug = False # Flag for printing debug info
        print(f"EKF initialized with dt={self.dt:.4f}s, n_states={self.n_states}, n_inp={self.n_inp}, n_pro_noise={self.n_pro_noise}")

    def discretize(self, discrete_flag=False):
        """Discretizes the continuous-time system matrices A, B, E.

        Uses either Zero-Order Hold (ZOH) via matrix exponential (if A is invertible)
        or first-order Euler approximation.

        Args:
            discrete_flag (bool, optional): If True, assumes A, B, E are already discrete. 
                                          Defaults to False.
        """
        if discrete_flag:
            # Assume A, B, E provided are already discrete Ad, Bd, Ed
            self.Ad = self.A
            self.Bd = self.B
            self.Ed = self.E
        else:
            # Check if A is singular or nearly singular (determinant close to zero)
            # Use Euler if singular, otherwise use matrix exponential for better accuracy
            if np.abs(np.linalg.det(self.A)) < 1e-6:
                # Forward Euler approximation
                # Ad = I + A*dt
                # Bd = B*dt
                # Ed = E*dt
                warnings.warn("Matrix A is singular or near-singular. Using Euler discretization.", RuntimeWarning)
                self.Ad = np.eye(self.n_states) + self.A * self.dt
                self.Bd = self.B * self.dt
                self.Ed = self.E * self.dt
            else:
                # ZOH discretization using matrix exponential
                # Ad = expm(A*dt)
                # Bd = A^-1 * (Ad - I) * B
                # Ed = A^-1 * (Ad - I) * E
                try:
                    self.Ad = scipy.linalg.expm(self.A * self.dt)
                    A_inv = np.linalg.inv(self.A)
                    Ad_minus_I = self.Ad - np.eye(self.n_states)
                    self.Bd = A_inv @ Ad_minus_I @ self.B
                    self.Ed = A_inv @ Ad_minus_I @ self.E
                except np.linalg.LinAlgError:
                    warnings.warn("Matrix A inversion failed during discretization. Falling back to Euler.", RuntimeWarning)
                    self.Ad = np.eye(self.n_states) + self.A * self.dt
                    self.Bd = self.B * self.dt
                    self.Ed = self.E * self.dt
            
    def jacobian(self, fun, x0):
        """Numerically computes the Jacobian of a vector function using central differences.

        Args:
            fun (callable): The vector function (e.g., plant_model or meas_model) 
                          taking state vector x as input and returning a vector.
            x0 (np.ndarray): The state vector point at which to compute the Jacobian.

        Returns:
            np.ndarray: The computed Jacobian matrix (df/dx).
        """
        x0_flat = x0.flatten() # Ensure x0 is 1D for iteration
        f0 = fun(x0_flat) # Evaluate function at x0
        n_out = np.size(f0)
        n_in = np.size(x0_flat)
        jacob = np.zeros((n_out, n_in))
        eps = 1e-6 # Step size for numerical differentiation

        for i in range(n_in):
            # Perturb state i slightly in both directions
            x2 = x0_flat.copy()
            x2[i] += eps

            x1 = x0_flat.copy()
            x1[i] -= eps

            # Evaluate function at perturbed points
            f1 = fun(x1)
            f2 = fun(x2)
            
            # Central difference approximation for the i-th column of the Jacobian
            jacob[:, i] = (f2 - f1) / (2.0 * eps)
        
        return jacob
    
    def predict(self, u, A=None, B=None, E=None, plant_model=None, discrete_flag=False, threshold=None):
        """Performs the EKF prediction (time update) step.

        Updates the state estimate `x` and covariance `P` based on the system model.

        Args:
            u (np.ndarray): Control input vector at the current time step.
            A (np.ndarray, optional): State transition matrix (or Jacobian F). If None and plant_model 
                                    is provided, Jacobian is computed numerically. Defaults to None.
            B (np.ndarray, optional): Control input matrix (or Jacobian G). Defaults to None (assumes no input).
            E (np.ndarray, optional): Process noise input matrix. Defaults to None (uses previous self.E).
            plant_model (callable, optional): Non-linear state transition function x_dot = f(t, x, u).
                                           Used for RK4 integration if provided. Defaults to None.
            discrete_flag (bool, optional): If True, assumes A, B, E are already discrete. Defaults to False.
            threshold (np.ndarray, optional): Optional clipping threshold for state change. Defaults to None.
        """
        # --- Update System Matrices --- 
        # Update A if provided, otherwise compute Jacobian if plant_model is given
        if A is not None:
            if A.shape == self.A.shape:
                self.A = A
            else:
                raise ValueError(f"Inconsistent size of matrix A: Expected {self.A.shape} but found {A.shape}")
        elif plant_model is not None:
            # Numerically compute Jacobian F = df/dx around current state x
            self.A = self.jacobian(lambda x_eval: plant_model(self.t, x_eval, u), self.x)
        # else: use self.A from previous step or initialization
        
        # Update B if provided
        if B is not None:
            if B.shape == self.B.shape:
                self.B = B
            else:
                raise ValueError(f"Inconsistent size of matrix B: Expected {self.B.shape} but found {B.shape}")
        else:
            # If B is not given, assume zero input influence
            # Ensure u matches the zero B matrix dimension if u was provided
            self.B = np.zeros((self.n_states, self.n_inp))
            # u should be compatible, e.g., np.zeros((self.n_inp, 1)) or scalar if n_inp=1
            if isinstance(u, np.ndarray) and u.shape[0] != self.n_inp:
                 warnings.warn(f"Input u shape {u.shape} inconsistent with n_inp={self.n_inp} when B is None. Assuming zero input.")
            # Force u to be compatible scalar/vector for potential later use (though B is zero now)
            u = np.zeros((self.n_inp, 1)) if self.n_inp > 1 else 0.0

        # Update E if provided
        if E is not None:
            if E.shape == self.E.shape:
                self.E = E
            else:
                raise ValueError(f"Inconsistent size of matrix E: Expected {self.E.shape} but found {E.shape}")
        # else: use self.E from previous step or initialization
        
        # --- Discretize System --- 
        self.discretize(discrete_flag=discrete_flag)

        # --- Predict State --- 
        if plant_model is None:
            # Linear prediction: x_k+1 = Ad * x_k + Bd * u_k
            # Ensure correct dimensions for u multiplication
            if self.n_inp == 1 and isinstance(u, (int, float)):
                 x_next = self.Ad @ self.x + self.Bd * u # u is scalar
            elif self.n_inp > 0:
                 # Ensure u is a column vector (n_inp x 1)
                 u_col = np.asarray(u).reshape((self.n_inp, 1)) 
                 x_next = self.Ad @ self.x + self.Bd @ u_col
            else: # n_inp == 0
                 x_next = self.Ad @ self.x
            change = x_next - self.x
        else:
            # Non-linear prediction using 4th order Runge-Kutta (RK4) integration
            # x_k+1 = x_k + dt * f_avg(t_k, x_k, u_k)
            # Ensure plant_model receives 1D state array and compatible u
            x_flat = self.x.flatten()
            u_arg = u.flatten() if isinstance(u, np.ndarray) else u # Pass u appropriately
            
            k1 = plant_model(self.t, x_flat, u_arg)
            k2 = plant_model(self.t + self.dt/2.0, x_flat + self.dt/2.0 * k1, u_arg) 
            k3 = plant_model(self.t + self.dt/2.0, x_flat + self.dt/2.0 * k2, u_arg)
            k4 = plant_model(self.t + self.dt, x_flat + self.dt * k3, u_arg)
            
            x_next_flat = x_flat + self.dt/6.0 * (k1 + 2.0*k2 + 2.0*k3 + k4)
            change = x_next_flat[:, np.newaxis] - self.x            
        
        # Optional clipping of the state change (useful for stability)
        if threshold is not None:
            if len(threshold) != self.n_states:
                raise ValueError(f"Threshold length {len(threshold)} must match n_states {self.n_states}")
            # Clip each element of the change vector
            # Ensure threshold is applied correctly to the change, not the state itself
            clipped_change = np.array([clip(ch[0], -abs(th), abs(th)) for ch, th in zip(change, threshold)])
            change = clipped_change[:, np.newaxis]
                
        # Update state estimate
        self.x = self.x + change
        
        # --- Predict Covariance --- 
        # P_k+1|k = Ad * P_k|k * Ad^T + Ed * Q * Ed^T
        self.P = self.Ad @ self.P @ self.Ad.T + self.Ed @ self.Q @ self.Ed.T
        
        # Update time
        self.t = self.t + self.dt
        
        # Apply state constraints or normalizations (e.g., wrap angles)
        # Assuming states 3, 4, 5 are Euler angles (roll, pitch, yaw)
        for i in range(3, 6): 
            if i < self.n_states:
                 self.x[i] = ssa(self.x[i]) # Normalize angles to [-pi, pi]

    def correct(self, y, Cd, R, meas_model=None, threshold=None, imu_ssa=False):
        """Performs the EKF correction (measurement update) step.

        Updates the state estimate `x` and covariance `P` based on a measurement `y`.

        Args:
            y (np.ndarray): Measurement vector (m x 1).
            Cd (np.ndarray): Measurement matrix (or Jacobian H) (m x n_states).
            R (np.ndarray): Measurement noise covariance matrix (m x m).
            meas_model (callable, optional): Non-linear measurement function y = h(x). 
                                           If provided, Cd is ignored (Jacobian H=dh/dx should be 
                                           computed or passed as Cd). Defaults to None (linear model y=Cd*x assumed).
                                           **Note**: Current implementation uses provided Cd even if meas_model is given. 
                                           Consider computing Jacobian H from meas_model if non-linear.
            threshold (np.ndarray, optional): Optional clipping threshold for state correction. Defaults to None.
            imu_ssa (bool, optional): If True, applies smallest-signed-angle (ssa) to the first 3 elements 
                                    of the innovation (y - h(x)), assuming they are angular errors. Defaults to False.
        """        
        m = y.shape[0] # Measurement dimension
        I = np.eye(self.n_states)
        
        # Calculate innovation covariance S = H * P * H^T + R
        S = Cd @ self.P @ Cd.T + R

        # Check if S is invertible before proceeding
        if np.abs(np.linalg.det(S)) > 1e-9: # Use a small tolerance for singularity check
            # Calculate Kalman Gain K = P * H^T * S^-1
            try:
                 S_inv = np.linalg.inv(S)
                 K = self.P @ Cd.T @ S_inv
            except np.linalg.LinAlgError:
                warnings.warn("Singular matrix S encountered during Kalman gain calculation. Skipping correction.", RuntimeWarning)
                return

            # Calculate innovation (measurement residual) 
            # inov = y - h(x) or inov = y - Cd * x
            if meas_model is not None:
                # Use non-linear measurement model h(x)
                # Ensure meas_model returns a column vector matching y
                h_x = meas_model(self.x.flatten())[:, np.newaxis]
                innovation = y - h_x
                # **Note**: If meas_model is non-linear, Cd should ideally be the Jacobian dh/dx evaluated at self.x.
                # The current code uses the provided Cd regardless. This might be incorrect if h(x) is non-linear.
                # Consider adding Jacobian calculation: Cd = self.jacobian(meas_model, self.x)
            else:
                # Use linear measurement model: y = Cd * x
                innovation = y - Cd @ self.x

            # Apply SSA normalization to angular components of innovation if requested
            if imu_ssa:
                # Assuming first 3 elements are angular errors (e.g., roll, pitch, yaw error)
                for i in range(min(3, m)): # Apply only up to measurement dimension
                    innovation[i] = ssa(innovation[i])
            
            # Calculate state correction: K * innovation
            change = K @ innovation
            
            # Optional clipping of the state correction
            if threshold is not None:
                if len(threshold) != self.n_states:
                   raise ValueError(f"Threshold length {len(threshold)} must match n_states {self.n_states}")
                # Clip each element of the correction vector
                clipped_change = np.array([clip(ch[0], -abs(th), abs(th)) for ch, th in zip(change, threshold)])
                change = clipped_change[:, np.newaxis]
            
            # --- Update State Estimate --- 
            # x_k|k = x_k|k-1 + K * innovation
            self.x = self.x + change
            
            # --- Update Covariance Estimate --- 
            # P_k|k = (I - K * H) * P_k|k-1 * (I - K * H)^T + K * R * K^T (Joseph form for numerical stability)
            # Or P_k|k = (I - K * H) * P_k|k-1 
            I_minus_KC = I - K @ Cd
            # Joseph form:
            self.P = I_minus_KC @ self.P @ I_minus_KC.T + K @ R @ K.T 
            # Simpler form (less stable if K is large or P is ill-conditioned):
            # self.P = I_minus_KC @ self.P 

            # Apply state constraints/normalizations after correction (e.g., wrap angles)
            # Assuming states 3, 4, 5 are Euler angles
            for i in range(3, 6):
                if i < self.n_states:
                    self.x[i] = ssa(self.x[i])
            
            # Debugging print statement (optional)
            # warnings.warn(f"Innovation: {innovation.flatten()}, Change: {change.flatten()}")

        else:
            # Singularity detected in innovation covariance S
            warnings.warn(f"WARNING: Singular innovation covariance matrix S! Correction step skipped.", RuntimeWarning)

        if self.debug:
            print(f"--- Correction Step (t={self.t:.2f}) ---")
            print(f"Measurement y: {y.flatten()}")
            if meas_model: print(f"Predicted meas h(x): {h_x.flatten()}")
            else: print(f"Predicted meas Cd*x: {(Cd @ self.x).flatten()}")
            print(f"Innovation: {innovation.flatten()}")
            print(f"Kalman Gain K (norm): {np.linalg.norm(K)}")
            print(f"State change: {change.flatten()}")
            print(f"Corrected x: {self.x.flatten()}")
            print(f"Corrected P (diag): {np.diag(self.P)}")
            
            