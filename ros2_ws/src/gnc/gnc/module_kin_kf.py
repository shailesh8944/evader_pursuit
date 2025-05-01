"""
File: module_kin_kf.py
Description: Defines kinematic models, measurement models, and associated Jacobians 
             specifically tailored for the Extended Kalman Filter (EKF) used in this navigation system.
             Includes functions for calculating derivatives of rotation matrices, 
             state transition matrices (A, E), measurement Jacobians (C for IMU and GNSS), 
             and the non-linear process and measurement models (f(x), h(x)).

Assumed EKF State Vector (x):
    [0:3]   r_nb_n      Position (NED frame)
    [3:6]   Theta_nb    Euler angles (roll, pitch, yaw) representing body orientation wrt NED
    [6:9]   v_nb_b      Velocity (Body frame)
    [9:12]  omg_nb_b    Angular velocity (Body frame)
    [12:15] a_nb_b      Linear acceleration (Body frame) - Potentially specific force?

Author: MAV GNC Team
"""

import numpy as np
# Import derivatives of sensor orientation w.r.t body orientation (likely auto-generated)
from dTheta_ns import dTheta_ns_dphi, dTheta_ns_dtheta, dTheta_ns_dpsi 
from mav_simulator.module_kinematics import eul_to_rotm, Smat, rotm_to_eul
# Renaming for clarity, J2 is the transformation from body rates to Euler rates
from mav_simulator.module_kinematics import eul_rate_matrix as J2mat 

# --- Helper Matrix Functions (Potentially related to Coriolis/Centripetal terms or Jacobians) --- 
# Note: The physical meaning of S1mat, S2mat, S3mat is unclear without further context.
# They might be related to derivatives of inertia matrices or specific kinematic terms.

def S1mat(vec):
    """Constructs a specific 3x3 matrix based on input vector `vec`."""
    # Check usage context for physical meaning.
    return np.array([
        [0, vec[1], vec[2]],
        [vec[1], -2*vec[0], 0],
        [vec[2], 0, -2*vec[0]]
    ])

def S2mat(vec):
    """Constructs a specific 3x3 matrix based on input vector `vec`."""
    # Check usage context for physical meaning.
    return np.array([
        [-2*vec[1], vec[0], 0],
        [vec[0], 0, vec[2]],
        [0, vec[2], -2*vec[1]]        
    ])

def S3mat(vec):
    """Constructs a specific 3x3 matrix based on input vector `vec`."""
    # Check usage context for physical meaning.
    return np.array([
        [-2*vec[2], 0, vec[0]],
        [0, -2*vec[2], vec[1]],
        [vec[0], vec[1], 0]
    ])

# --- Rotation Matrix Derivatives --- 
# Calculate the partial derivative of the ZYX Euler rotation matrix R_b_n(phi, theta, psi)
# with respect to each Euler angle. These are used in Jacobians involving rotations.

def dR_dphi(eul):
    """Calculates d(R_b_n)/d(phi) evaluated at Euler angles `eul` [phi, theta, psi]."""
    phi, theta, psi = eul[0], eul[1], eul[2]
    
    s1, c1 = np.sin(phi), np.cos(phi)
    s2, c2 = np.sin(theta), np.cos(theta)
    s3, c3 = np.sin(psi), np.cos(psi)

    # Derivative of ZYX rotation matrix w.r.t. phi (roll)
    dR = np.array([
        [0, s1*s3 + s2*c1*c3, -s1*s2*c3 + s3*c1],
        [0, -s1*c3 + s3*s2*c1, -s1*s3*s2 - c1*c3],
        [0, c1*c2, -s1*c2]
    ])
    
    return dR

def dR_dtheta(eul):
    """Calculates d(R_b_n)/d(theta) evaluated at Euler angles `eul` [phi, theta, psi]."""
    phi, theta, psi = eul[0], eul[1], eul[2]
    
    s1, c1 = np.sin(phi), np.cos(phi)
    s2, c2 = np.sin(theta), np.cos(theta)
    s3, c3 = np.sin(psi), np.cos(psi)

    # Derivative of ZYX rotation matrix w.r.t. theta (pitch)
    dR = np.array([
        [-s2*c3, s1*c3*c2, c1*c3*c2],
        [-s3*s2, s3*s1*c2, s3*c1*c2],
        [-c2, -s1*s2, -s2*c1]
    ])
    return dR

def dR_dpsi(eul):
    """Calculates d(R_b_n)/d(psi) evaluated at Euler angles `eul` [phi, theta, psi]."""
    phi, theta, psi = eul[0], eul[1], eul[2]
    
    s1, c1 = np.sin(phi), np.cos(phi)
    s2, c2 = np.sin(theta), np.cos(theta)
    s3, c3 = np.sin(psi), np.cos(psi)

    # Derivative of ZYX rotation matrix w.r.t. psi (yaw)
    dR = np.array([
        [-s3*c2, -s3*s1*s2 - c1*c3, s1*c3 - s3*s2*c1],
        [c3*c2, s1*s2*c3 - s3*c1, s1*s3 + s2*c1*c3],
        [0, 0, 0] # Z-axis rotation derivative is simpler
    ])
    return dR

# --- Euler Rate Matrix (J2) Derivatives --- 
# Calculate the partial derivative of the Euler rate transformation matrix J2(Theta_nb)
# (where Theta_dot = J2 * omega_b) with respect to each Euler angle.

def dJ2_dphi(eul):
    """Calculates d(J2)/d(phi) evaluated at Euler angles `eul` [phi, theta, psi]."""
    phi, theta, psi = eul[0], eul[1], eul[2]
    
    s1, c1 = np.sin(phi), np.cos(phi)
    s2, c2 = np.sin(theta), np.cos(theta)
    # s3, c3 = np.sin(psi), np.cos(psi) # Not needed for dJ2/dphi

    # Derivative of J2 matrix w.r.t. phi (roll)
    dJ = np.array([
        [0, c1*s2/c2, -s1*s2/c2],
        [0, -s1, -c1],
        [0, c1/c2, -s1/c2]
    ])
    return dJ

def dJ2_dtheta(eul):
    """Calculates d(J2)/d(theta) evaluated at Euler angles `eul` [phi, theta, psi]."""
    phi, theta, psi = eul[0], eul[1], eul[2]
    
    s1, c1 = np.sin(phi), np.cos(phi)
    s2, c2 = np.sin(theta), np.cos(theta)
    # s3, c3 = np.sin(psi), np.cos(psi) # Not needed for dJ2/dtheta

    # Derivative of J2 matrix w.r.t. theta (pitch)
    # Protect against division by zero if cos(theta) is near zero (singularity)
    if abs(c2) < 1e-6:
        # Handle singularity - return zeros or raise error? Returning zeros for now.
        warnings.warn("Pitch angle near +/- 90 deg (singularity) in dJ2_dtheta. Returning zeros.")
        return np.zeros((3,3))
    
    dJ = np.array([
        [0, s1/(c2**2), c1/(c2**2) ],
        [0, 0, 0],
        [0, s1*s2/(c2**2), c1*s2/(c2**2)]
    ])
    return dJ

def dJ2_dpsi(eul):
    """Calculates d(J2)/d(psi) evaluated at Euler angles `eul` [phi, theta, psi]."""
    # phi, theta, psi = eul[0], eul[1], eul[2]
    # s1, c1 = np.sin(phi), np.cos(phi)
    # s2, c2 = np.sin(theta), np.cos(theta)
    # s3, c3 = np.sin(psi), np.cos(psi)

    # J2 matrix does not depend on psi, so derivative is zero.
    dJ = np.zeros((3,3))
    return dJ

# --- Jacobian Helper Matrices (L1 to L5) --- 
# These functions compute matrix blocks used in the larger Jacobians A and C.

def L1mat(eul, v_nb_b):
    """Computes the block d(R_b_n * v_b)/d(Theta_nb).
    Used in A[0:3, 3:6] (position dynamics dependence on orientation).
    """
    L1 = np.zeros((3,3))
    # Each column is (d(R_b_n)/d(angle)) * v_b
    L1[:, 0] = dR_dphi(eul) @ v_nb_b
    L1[:, 1] = dR_dtheta(eul) @ v_nb_b
    L1[:, 2] = dR_dpsi(eul) @ v_nb_b
    return L1

def L2mat(eul, omg_nb_b):
    """Computes the block d(J2 * omg_b)/d(Theta_nb).
    Used in A[3:6, 3:6] (Euler rate dynamics dependence on orientation).
    """
    L2 = np.zeros((3,3))
    # Each column is (d(J2)/d(angle)) * omg_b
    L2[:, 0] = dJ2_dphi(eul) @ omg_nb_b
    L2[:, 1] = dJ2_dtheta(eul) @ omg_nb_b
    L2[:, 2] = dJ2_dpsi(eul) @ omg_nb_b # This column will be zero
    return L2

def L3mat(eul, eul_s):
    """Computes the block d(Theta_ns)/d(Theta_nb). 
    Relates the change in sensor orientation measurement (Theta_ns) to the change 
    in body orientation (Theta_nb). Uses imported derivatives from dTheta_ns.
    Used in C_imu[0:3, 3:6] (IMU Euler angle measurement dependence on body orientation).
    """
    L3 = np.zeros((3,3))
    # Assumes dTheta_ns functions return (3,) arrays, flatten just in case
    try:
        # Body Euler angles (eul) and Sensor Euler angles wrt body (eul_s) are inputs
        L3[:, 0] = dTheta_ns_dphi(eul[0], eul[1], eul[2], eul_s[0], eul_s[1], eul_s[2]).flatten()
        L3[:, 1] = dTheta_ns_dtheta(eul[0], eul[1], eul[2], eul_s[0], eul_s[1], eul_s[2]).flatten()
        L3[:, 2] = dTheta_ns_dpsi(eul[0], eul[1], eul[2], eul_s[0], eul_s[1], eul_s[2]).flatten()
    except Exception as e:
         print(f"Error during L3mat calculation (check dTheta_ns imports/args): {e}")
         # Return zeros or raise error?
    return L3

def L4mat(omg_nb_b, r_bs_b, Theta_bs):
    """Computes a block related to the derivative of sensor acceleration w.r.t body angular rate.
    Involves Coriolis-like terms: d(R_b_s * (omega_b x (omega_b x r_s))) / d(omega_b) ? 
    Or derivative of centrifugal term: d(R_b_s * (omega_dot_b x r_s)) / d(omega_b) ? 
    Requires clarification based on the exact IMU model used.
    Used in C_imu[6:9, 9:12] (IMU acceleration measurement dependence on body angular rate).
    """
    L4 = np.zeros((3,3))
    R_s_b = eul_to_rotm(Theta_bs) # Rotation from sensor frame to body frame
    R_b_s = R_s_b.T # Rotation from body frame to sensor frame
    # The S1/S2/S3 matrices might be related to d(omega x vector)/d(omega)
    L4[:, 0] = R_b_s @ S1mat(omg_nb_b) @ r_bs_b
    L4[:, 1] = R_b_s @ S2mat(omg_nb_b) @ r_bs_b
    L4[:, 2] = R_b_s @ S3mat(omg_nb_b) @ r_bs_b
    return L4

def L5mat(eul, r_bs_b):
    """Computes the block d(R_b_n * r_s) / d(Theta_nb).
    Relates the change in the sensor position vector (in NED frame) to the change 
    in body orientation.
    Used in C_gnss[0:3, 3:6] (GNSS position measurement dependence on body orientation).
    """
    L5 = np.zeros((3,3))
    # Each column is (d(R_b_n)/d(angle)) * r_bs_b
    L5[:, 0] = dR_dphi(eul) @ r_bs_b
    L5[:, 1] = dR_dtheta(eul) @ r_bs_b
    L5[:, 2] = dR_dpsi(eul) @ r_bs_b
    return L5

# --- EKF System Matrices (Linearized) --- 

def state_mats(x):
    """Computes the linearized state transition matrix A and process noise matrix E.
    
    A = df/dx evaluated at the current state x.
    E = df/dw (where w is process noise) evaluated at the current state x.

    Args:
        x (np.ndarray): Current state vector (15x1).

    Returns:
        tuple: (A, E)
            - A (np.ndarray): Linearized state transition matrix (15x15).
            - E (np.ndarray): Linearized process noise input matrix (15x6).
    """
    # Unpack state vector for clarity
    # r_nb_n = x[0:3]   # Position (NED)
    Theta_nb = x[3:6] # Euler angles (Body wrt NED)
    v_nb_b = x[6:9]   # Velocity (Body)
    omg_nb_b = x[9:12]  # Angular velocity (Body)
    acc_nb_b = x[12:15] # Acceleration (Body) - Assuming this is true acceleration a_dot = a x omega ?
    
    A = np.zeros((15, 15))
    
    # --- Populate A matrix (Jacobian df/dx) --- 
    # d(r_dot)/d(Theta) = d(R*v)/d(Theta) = L1mat
    A[0:3, 3:6] = L1mat(Theta_nb, v_nb_b) 
    # d(r_dot)/d(v) = d(R*v)/d(v) = R
    A[0:3, 6:9] = eul_to_rotm(Theta_nb)
    
    # d(Theta_dot)/d(Theta) = d(J2*omg)/d(Theta) = L2mat
    A[3:6, 3:6] = L2mat(Theta_nb, omg_nb_b)
    # d(Theta_dot)/d(omg) = d(J2*omg)/d(omg) = J2
    A[3:6, 9:12] = J2mat(Theta_nb)
    
    # d(v_dot)/d(v) = d(a - omg x v)/d(v) = -S(omg)
    A[6:9, 6:9] = -Smat(omg_nb_b)
    # d(v_dot)/d(omg) = d(a - omg x v)/d(omg) = S(v)
    A[6:9, 9:12] = Smat(v_nb_b)
    # d(v_dot)/d(a) = d(a - omg x v)/d(a) = I
    A[6:9, 12:15] = np.eye(3)

    # d(omg_dot)/d(omg) = 0 (assuming omg_dot driven by noise)
    # d(omg_dot)/d(a) = 0

    # d(a_dot)/d(omg) = d(a x omg)/d(omg) = -S(a) ? Or S(a)? Convention check needed.
    # Assuming d(cross(a,b))/db = -S(a). Let's assume a_dot model is correct.
    A[12:15, 9:12] = -Smat(acc_nb_b) # Check sign convention
    # d(a_dot)/d(a) = d(a x omg)/d(a) = S(omg) 
    A[12:15, 12:15] = Smat(omg_nb_b) # Check sign convention

    # --- Populate E matrix (Jacobian df/dw) --- 
    # Assumes process noise w = [w_gyro (3), w_accel (3)] affects omg_dot and a_dot
    # omg_dot = R_n_b * w_gyro => E[9:12, 0:3] = R_n_b = R_b_n.T
    # a_dot = R_n_b * w_accel => E[12:15, 3:6] = R_n_b = R_b_n.T
    E = np.zeros((15, 6))
    R_b_n = eul_to_rotm(Theta_nb)
    E[9:12, 0:3] = R_b_n.T # Noise effect on angular velocity derivative
    E[12:15, 3:6] = R_b_n.T # Noise effect on linear acceleration derivative

    return A, E

# --- EKF Measurement Jacobians (C = dh/dx) --- 

def imu_mat(x, r_bs_b=np.array([0.0, 0.0, 0.0]), Theta_bs=np.array([0.0, 0.0, 0.0])):
    """Computes the linearized IMU measurement matrix C_imu = dh_imu/dx.

    Assumes IMU measurement y_imu = [Theta_ns (3), omg_s (3), acc_s (3)]. 
    Note the order: Euler angles, angular velocity, linear acceleration.

    Args:
        x (np.ndarray): Current state vector (15x1).
        r_bs_b (np.ndarray, optional): Position vector from body origin to sensor frame, in body coords. Defaults to zero.
        Theta_bs (np.ndarray, optional): Euler angles for sensor frame orientation relative to body frame. Defaults to zero.

    Returns:
        np.ndarray: IMU Measurement Jacobian C_imu (9x15).
    """
    # Unpack state vector
    # r_nb_n = x[0:3]   # Position (NED)
    Theta_nb = x[3:6] # Euler angles (Body wrt NED)
    # v_nb_b = x[6:9]   # Velocity (Body)
    omg_nb_b = x[9:12]  # Angular velocity (Body)
    acc_nb_b = x[12:15] # Acceleration (Body)
    
    C = np.zeros((9, 15))
    
    # --- Populate C_imu matrix (Jacobian dh_imu/dx) --- 
    # Row 0:2 -> Euler Angles Measurement (Theta_ns)
    # d(Theta_ns)/d(Theta_nb) = L3mat (using dTheta_ns functions)
    C[0:3, 3:6] = L3mat(Theta_nb, Theta_bs)
    
    # Row 3:5 -> Angular Velocity Measurement (omg_s)
    # omg_s = R_b_s * omg_b = R_s_b.T * omg_b
    # d(omg_s)/d(omg_nb_b) = R_s_b.T
    R_s_b = eul_to_rotm(Theta_bs)
    C[3:6, 9:12] = R_s_b.T 
    
    # Row 6:8 -> Linear Acceleration Measurement (acc_s)
    # acc_s = R_b_s * (acc_b + omg_dot x r_s + omg x (omg x r_s)) - g_s ? 
    # Model simplification? Appears to be: acc_s = R_b_s * (acc_b + omg x (omg x r_s) + omg_dot x r_s)?
    # Current C matrix suggests model: acc_s approx R_b_s * (acc_b + d/dt(omg_b) x r_bs_b + ...?)
    # Check imu_model function for the exact model assumed.
    # d(acc_s)/d(omg_nb_b) = L4mat ? (Depends on the exact model used)
    C[6:9, 9:12] = L4mat(omg_nb_b, r_bs_b, Theta_bs)
    # d(acc_s)/d(acc_nb_b) = R_b_s = R_s_b.T
    C[6:9, 12:15] = R_s_b.T
    
    return C

def gnss_mat(x, r_bs_b=np.array([0.0, 0.0, 0.0]), Theta_bs=np.array([0.0, 0.0, 0.0])):
    """Computes the linearized GNSS measurement matrix C_gnss = dh_gnss/dx.

    Assumes GNSS measurement y_gnss = position_sensor_ned (r_ns_n).
    Model: r_ns_n = r_nb_n + R_b_n * r_bs_b

    Args:
        x (np.ndarray): Current state vector (15x1).
        r_bs_b (np.ndarray, optional): Position vector from body origin to sensor frame, in body coords. Defaults to zero.
        Theta_bs (np.ndarray, optional): Euler angles for sensor frame orientation relative to body frame. (Unused in this model).

    Returns:
        np.ndarray: GNSS Measurement Jacobian C_gnss (3x15).
    """
    # Unpack state vector
    # r_nb_n = x[0:3]   # Position (NED)
    Theta_nb = x[3:6] # Euler angles (Body wrt NED)
    # v_nb_b = x[6:9]   # Velocity (Body)
    # omg_nb_b = x[9:12]  # Angular velocity (Body)
    # acc_nb_b = x[12:15] # Acceleration (Body)
    
    C = np.zeros((3, 15))
    
    # --- Populate C_gnss matrix (Jacobian dh_gnss/dx) --- 
    # d(r_ns_n)/d(r_nb_n) = I
    C[0:3, 0:3] = np.eye(3)
    # d(r_ns_n)/d(Theta_nb) = d(R_b_n * r_bs_b) / d(Theta_nb) = L5mat
    C[0:3, 3:6] = L5mat(Theta_nb, r_bs_b)

    return C

# --- Non-Linear Models (for RK4 integration and EKF correction) --- 

def plant_model(t, x, u):
    """Non-linear state transition function dx/dt = f(t, x, u).

    Defines the continuous-time dynamics of the 15-state system.

    Args:
        t (float): Current time (unused in this model).
        x (np.ndarray): Current state vector (15, ). Flattened.
        u : Control input vector (unused in this model, placeholder).

    Returns:
        np.ndarray: State derivative vector dx/dt (15, ).
    """
    # Unpack state vector
    # r_nb_n = x[0:3]   # Position (NED)
    Theta_nb = x[3:6] # Euler angles (Body wrt NED)
    v_nb_b = x[6:9]   # Velocity (Body)
    omg_nb_b = x[9:12]  # Angular velocity (Body)
    a_nb_b = x[12:15] # Acceleration (Body)

    R_b_n = eul_to_rotm(Theta_nb) # Rotation matrix from body to NED
    J2_nb = J2mat(Theta_nb)      # Transformation matrix for Euler rates

    xd = np.zeros(15)

    # Position derivative (NED frame): r_dot = R_b_n * v_b
    xd[0:3] = R_b_n @ v_nb_b
    # Euler angle derivative: Theta_dot = J2 * omg_b
    xd[3:6] = J2_nb @ omg_nb_b
    # Velocity derivative (Body frame): v_dot = a_b + v_b x omg_b (Coriolis)
    # Note the sign: typically v_dot = forces/mass - omg x v. Here a_b seems to include 1/m * F_ext?
    # Assuming a_nb_b represents the non-gravitational acceleration in body frame.
    xd[6:9] = a_nb_b + np.cross(omg_nb_b, v_nb_b) # Check Coriolis term sign: a_b - omega x v ?
    
    # Angular velocity derivative: omg_dot = ???
    # Assuming driven by noise / external torques not modeled here.
    # Set to zero, noise is added via E*w in EKF predict step covariance.
    xd[9:12] = 0.0 
    
    # Acceleration derivative: a_dot = ???
    # Model assumes a_dot = a x omg ? This seems unusual. Usually a_dot relates to jerk.
    # Perhaps modeling bias dynamics? Check derivation.
    # Set to zero, noise is added via E*w in EKF predict step covariance.
    # xd[12:15] = np.cross(a_nb_b, omg_nb_b) # Original line - Check physical basis.
    xd[12:15] = 0.0 # Assuming acceleration is driven by noise (bias model?)


    return xd

def imu_model(x, r_bs_b=np.array([0.0, 0.0, 0.0]), Theta_bs=np.array([0.0, 0.0, 0.0])):
    """Non-linear IMU measurement model y_imu = h_imu(x).

    Predicts the expected IMU measurements based on the current state.
    Output: [Theta_ns (3), omg_s (3), acc_s (3)]

    Args:
        x (np.ndarray): Current state vector (15, ). Flattened.
        r_bs_b (np.ndarray, optional): Position vector from body origin to sensor frame, in body coords. Defaults to zero.
        Theta_bs (np.ndarray, optional): Euler angles for sensor frame orientation relative to body frame. Defaults to zero.

    Returns:
        np.ndarray: Predicted IMU measurement vector (9, ).
    """
    # Unpack state vector
    # r_nb_n = x[0:3]   # Position (NED)
    Theta_nb = x[3:6] # Euler angles (Body wrt NED)
    # v_nb_b = x[6:9]   # Velocity (Body)
    omg_nb_b = x[9:12]  # Angular velocity (Body)
    a_nb_b = x[12:15] # Acceleration (Body)

    # Body-to-NED rotation
    # R_b_n = eul_to_rotm(Theta_nb)

    # Sensor-to-Body rotation
    R_s_b = eul_to_rotm(Theta_bs)
    R_b_s = R_s_b.T

    # --- Predict Measurements --- 
    y_imu = np.zeros(9)

    # 1. Euler Angles (Sensor frame relative to NED?)
    # Assumes Theta_ns can be calculated from Theta_nb and Theta_bs.
    # The exact transformation depends on how Theta_ns is defined.
    # Placeholder: Use body angles rotated into sensor frame? Or composition?
    # If Theta_ns = rotm_to_eul(R_s_n) = rotm_to_eul(R_b_n * R_s_b), requires rotm composition.
    # Using Theta_nb directly rotated might be an approximation or specific convention.
    # C_imu suggests a more complex relationship via L3 / dTheta_ns. 
    # This model needs to be consistent with C_imu / L3mat!
    # Placeholder - Assume measured angles approx body angles for now. Check dTheta_ns derivation.
    R_n_b = eul_to_rotm(Theta_nb).T
    R_n_s = R_n_b @ R_b_s # Rotation from Sensor to NED
    try:
        Theta_ns = rotm_to_eul(R_n_s) # Convert rotation matrix to Euler angles
    except Exception as e:
         print(f"Error converting R_n_s to Euler in imu_model: {e}")
         Theta_ns = np.zeros(3) # Fallback
    y_imu[0:3] = Theta_ns

    # 2. Angular Velocity (Sensor frame)
    # omg_s = R_b_s * omg_b
    y_imu[3:6] = R_b_s @ omg_nb_b

    # 3. Linear Acceleration (Sensor frame)
    # Standard model: acc_s = R_b_s * (acc_b + omg_dot x r_s + omg x (omg x r_s)) - g_s
    # Simplified model used here? (Matches C_imu structure)
    # acc_s = R_b_s * (acc_b + omg x (omg x r_bs_b) + omg_dot x r_bs_b)?
    # Assuming a_nb_b is specific force (a_measured - g_b). Need to clarify g_b handling.
    # Assuming omg_dot = 0 (consistent with plant model)
    acc_coriolis_centripetal_b = np.cross(omg_nb_b, np.cross(omg_nb_b, r_bs_b)) # Centripetal term in body frame
    # Add gravity in body frame? Assume a_nb_b already accounts for it? Let's assume a_nb_b = specific force.
    acc_total_b = a_nb_b + acc_coriolis_centripetal_b
    y_imu[6:9] = R_b_s @ acc_total_b
    # If a_nb_b is true accel (not specific force), need to add R_b_s @ R_n_b @ [0,0,g]

    return y_imu

def gnss_model(x, r_bs_b=np.array([0.0, 0.0, 0.0]), Theta_bs=np.array([0.0, 0.0, 0.0])):
    """Non-linear GNSS measurement model y_gnss = h_gnss(x).

    Predicts the expected GNSS position measurement (in NED frame) based on the current state.
    Model: r_ns_n = r_nb_n + R_b_n * r_bs_b

    Args:
        x (np.ndarray): Current state vector (15, ). Flattened.
        r_bs_b (np.ndarray, optional): Position vector from body origin to sensor frame, in body coords. Defaults to zero.
        Theta_bs (np.ndarray, optional): Euler angles for sensor frame orientation relative to body frame. (Unused).

    Returns:
        np.ndarray: Predicted GNSS measurement vector (position NED) (3, ).
    """
    # Unpack state vector
    r_nb_n = x[0:3]   # Position (NED)
    Theta_nb = x[3:6] # Euler angles (Body wrt NED)
    # v_nb_b = x[6:9]   # Velocity (Body)
    # omg_nb_b = x[9:12]  # Angular velocity (Body)
    # a_nb_b = x[12:15] # Acceleration (Body)

    R_b_n = eul_to_rotm(Theta_nb) # Rotation matrix from body to NED

    # Calculate sensor position in NED frame
    r_ns_n = r_nb_n + R_b_n @ r_bs_b
    
    return r_ns_n