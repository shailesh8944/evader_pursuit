import numpy as np
from dTheta_ns import dTheta_ns_dphi, dTheta_ns_dtheta, dTheta_ns_dpsi
from mav_simulator.module_kinematics import eul_to_rotm, Smat, rotm_to_eul
from mav_simulator.module_kinematics import eul_rate_matrix as J2mat

def S1mat(vec):
    return np.array([
        [0, vec[1], vec[2]],
        [vec[1], -2*vec[0], 0],
        [vec[2], 0, -2*vec[0]]
    ])

def S2mat(vec):
    return np.array([
        [-2*vec[1], vec[0], 0],
        [vec[0], 0, vec[2]],
        [0, vec[2], -2*vec[1]]        
    ])

def S3mat(vec):
    return np.array([
        [-2*vec[2], 0, vec[0]],
        [0, -2*vec[2], vec[1]],
        [vec[0], vec[1], 0]
    ])

def dR_dphi(eul):
    phi = eul[0]
    theta = eul[1]
    psi = eul[2]
    
    s1 = np.sin(phi); c1 = np.cos(phi)
    s2 = np.sin(theta); c2 = np.cos(theta)
    s3 = np.sin(psi); c3 = np.cos(psi)

    dR = np.array([
        [0, s1*s3 + s2*c1*c3, -s1*s2*c3 + s3*c1],
        [0, -s1*c3 + s3*s2*c1, -s1*s3*s2 - c1*c3],
        [0, c1*c2, -s1*c2]
    ])
    
    return dR

def dR_dtheta(eul):
    phi = eul[0]
    theta = eul[1]
    psi = eul[2]
    
    s1 = np.sin(phi); c1 = np.cos(phi)
    s2 = np.sin(theta); c2 = np.cos(theta)
    s3 = np.sin(psi); c3 = np.cos(psi)

    dR = np.array([
        [-s2*c3, s1*c3*c2, c1*c3*c2],
        [-s3*s2, s3*s1*c2, s3*c1*c2],
        [-c2, -s1*s2, -s2*c1]
    ])
    return dR

def dR_dpsi(eul):
    phi = eul[0]
    theta = eul[1]
    psi = eul[2]
    
    s1 = np.sin(phi); c1 = np.cos(phi)
    s2 = np.sin(theta); c2 = np.cos(theta)
    s3 = np.sin(psi); c3 = np.cos(psi)

    dR = np.array([
        [-s3*c2, -s3*s1*s2 - c1*c3, s1*c3 - s3*s2*c1],
        [c3*c2, s1*s2*c3 - s3*c1, s1*s3 + s2*c1*c3],
        [0, 0, 0]
    ])
    return dR

def dJ2_dphi(eul):
    phi = eul[0]
    theta = eul[1]
    psi = eul[2]
    
    s1 = np.sin(phi); c1 = np.cos(phi)
    s2 = np.sin(theta); c2 = np.cos(theta)
    s3 = np.sin(psi); c3 = np.cos(psi)

    dJ = np.array([
        [0, c1*s2/c2, -s1*s2/c2],
        [0, -s1, -c1],
        [0, c1/c2, -s1/c2]
    ])
    return dJ

def dJ2_dtheta(eul):
    phi = eul[0]
    theta = eul[1]
    psi = eul[2]
    
    s1 = np.sin(phi); c1 = np.cos(phi)
    s2 = np.sin(theta); c2 = np.cos(theta)
    s3 = np.sin(psi); c3 = np.cos(psi)

    dJ = np.array([
        [0, s1/(c2**2), c1/(c2**2) ],
        [0, 0, 0],
        [0, s1*s2/(c2**2), c1*s2/(c2**2)]
    ])
    return dJ

def dJ2_dpsi(eul):
    phi = eul[0]
    theta = eul[1]
    psi = eul[2]
    
    s1 = np.sin(phi); c1 = np.cos(phi)
    s2 = np.sin(theta); c2 = np.cos(theta)
    s3 = np.sin(psi); c3 = np.cos(psi)

    dJ = np.zeros((3,3))
    return dJ

def L1mat(eul, v_nb_b):
    L1 = np.zeros((3,3))
    L1[:, 0] = dR_dphi(eul) @ v_nb_b
    L1[:, 1] = dR_dtheta(eul) @ v_nb_b
    L1[:, 2] = dR_dpsi(eul) @ v_nb_b
    return L1

def L2mat(eul, omg_nb_b):
    L2 = np.zeros((3,3))
    L2[:, 0] = dJ2_dphi(eul) @ omg_nb_b
    L2[:, 1] = dJ2_dtheta(eul) @ omg_nb_b
    L2[:, 2] = dJ2_dpsi(eul) @ omg_nb_b
    return L2

def L3mat(eul, eul_s):
    L3 = np.zeros((3,3))
    L3[:, 0] = dTheta_ns_dphi(eul[0], eul[1], eul[2], eul_s[0], eul_s[1], eul_s[2]).flatten()
    L3[:, 1] = dTheta_ns_dtheta(eul[0], eul[1], eul[2], eul_s[0], eul_s[1], eul_s[2]).flatten()
    L3[:, 2] = dTheta_ns_dpsi(eul[0], eul[1], eul[2], eul_s[0], eul_s[1], eul_s[2]).flatten()
    return L3

def L4mat(omg_nb_b, r_bs_b, Theta_bs):
    L4 = np.zeros((3,3))
    R_s_b = eul_to_rotm(Theta_bs)
    R_b_s = R_s_b.T
    L4[:, 0] = R_b_s @ S1mat(omg_nb_b) @ r_bs_b
    L4[:, 1] = R_b_s @ S2mat(omg_nb_b) @ r_bs_b
    L4[:, 2] = R_b_s @ S3mat(omg_nb_b) @ r_bs_b
    return L4

def L5mat(eul, r_bs_b):
    L5 = np.zeros((3,3))
    L5[:, 0] = dR_dphi(eul) @ r_bs_b
    L5[:, 1] = dR_dtheta(eul) @ r_bs_b
    L5[:, 2] = dR_dpsi(eul) @ r_bs_b
    return L5

def state_mats(x):
    
    r_nb_n = x[0:3]
    Theta_nb = x[3:6]
    v_nb_b = x[6:9]
    omg_nb_b = x[9:12]
    acc_nb_b = x[12:15]
    
    A = np.zeros((15,15))
    
    A[0:3][:, 3:6] = L1mat(Theta_nb, v_nb_b)
    A[0:3][:, 6:9] = eul_to_rotm(Theta_nb)
    
    A[3:6][:, 3:6] = L2mat(Theta_nb, omg_nb_b)
    A[3:6][:, 9:12] = J2mat(Theta_nb)
    
    A[6:9][:, 6:9] = -Smat(omg_nb_b)
    A[6:9][:, 9:12] = Smat(v_nb_b)
    A[6:9][:, 12:15] = np.eye(3)

    A[12:15][:, 9:12] = Smat(acc_nb_b)
    A[12:15][:, 12:15] = -Smat(omg_nb_b)

    E = np.zeros((15,6))

    E[9:12][:, 0:3] = eul_to_rotm(Theta_nb).T
    E[12:15][:, 3:6] = eul_to_rotm(Theta_nb).T

    return A, E

def imu_mat(x, r_bs_b=np.array([0.0, 0.0, 0.0]), Theta_bs=np.array([0.0, 0.0, 0.0])):
    
    r_nb_n = x[0:3]
    Theta_nb = x[3:6]
    v_nb_b = x[6:9]
    omg_nb_b = x[9:12]
    acc_nb_b = x[12:15]
    
    C = np.zeros((9, 15))
    
    C[0:3][:, 3:6] = L3mat(Theta_nb, Theta_bs)
    C[3:6][:, 9:12] = eul_to_rotm(Theta_bs).T
    C[6:9][:, 9:12] = L4mat(omg_nb_b, r_bs_b, Theta_bs)
    C[6:9][:, 12:15] = eul_to_rotm(Theta_bs).T
    
    return C

def gnss_mat(x, r_bs_b=np.array([0.0, 0.0, 0.0]), Theta_bs=np.array([0.0, 0.0, 0.0])):

    r_nb_n = x[0:3]
    Theta_nb = x[3:6]
    v_nb_b = x[6:9]
    omg_nb_b = x[9:12]
    acc_nb_b = x[12:15]
    
    C = np.zeros((3, 15))
    C[0:3][:, 0:3] = np.eye(3)
    C[0:3][:, 3:6] = L5mat(Theta_nb, r_bs_b)

    return C

def plant_model(t, x, u):

    r_nb_n = x[0:3]
    Theta_nb = x[3:6]
    v_nb_b = x[6:9]
    omg_nb_b = x[9:12]
    a_nb_b = x[12:15]

    R_b_n = eul_to_rotm(Theta_nb)
    J2_nb = J2mat(Theta_nb)

    xd = np.zeros(15)

    xd[0:3] = R_b_n @ v_nb_b
    xd[3:6] = J2_nb @ omg_nb_b
    xd[6:9] = a_nb_b + np.cross(v_nb_b, omg_nb_b)
    xd[9:12] = 0.0
    xd[12:15] = np.cross(a_nb_b, omg_nb_b)

    return xd

def imu_model(x, r_bs_b=np.array([0.0, 0.0, 0.0]), Theta_bs=np.array([0.0, 0.0, 0.0])):

    r_nb_n = x[0:3]
    Theta_nb = x[3:6]
    v_nb_b = x[6:9]
    omg_nb_b = x[9:12]
    a_nb_b = x[12:15]

    R_b_n = eul_to_rotm(Theta_nb)

    R_s_b = eul_to_rotm(Theta_bs)
    R_b_s = R_s_b.T

    y = np.zeros(9)
    y[0:3] = rotm_to_eul(R_b_n @ R_s_b)
    y[3:6] = R_b_s @ omg_nb_b
    y[6:9] = R_b_s @ a_nb_b + R_b_s @ np.cross(omg_nb_b, np.cross(omg_nb_b, r_bs_b))

    return y

def gnss_model(x, r_bs_b=np.array([0.0, 0.0, 0.0]), Theta_bs=np.array([0.0, 0.0, 0.0])):

    r_nb_n = x[0:3]
    Theta_nb = x[3:6]

    R_b_n = eul_to_rotm(Theta_nb)

    y = r_nb_n + R_b_n @ r_bs_b

    return y