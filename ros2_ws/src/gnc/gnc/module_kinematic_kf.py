import numpy as np

def d_by_dTheta_nb_R_b_n_v_nb_b(Theta_nb, v_nb_b):

    phi = Theta_nb[0]
    theta = Theta_nb[1]
    psi = Theta_nb[2]

    s1 = np.sin(phi); c1 = np.cos(phi)
    s2 = np.sin(theta); c2 = np.cos(theta)
    s3 = np.sin(psi); c3 = np.cos(psi)

    # R_b_n = np.array([
    #     [c2*c3, -c1*s3+s1*s2*c3, s1*s3+c1*s2*c3],
    #     [c2*s3, c1*c3+s1*s2*s3, -s1*c3+c1*s2*s3],
    #     [-s2, s1*c2, c1*c2]
    # ])

    d_by_dphi_R_b_n = np.array([
        [0, s1*s3+c1*s2*c3, c1*s3-s1*s2*c3],
        [0, -s1*c3+c1*s2*s3, -c1*c3-s1*s2*s3],
        [0, c1*c2, -s1*c2]
    ])

    d_by_dtheta_R_b_n = np.array([
        [-s2*c3, s1*c2*c3, c1*c2*c3],
        [-s2*s3, s1*c2*s3, c1*c2*s3],
        [-c2, -s1*s2, -c1*s2]
    ])

    d_by_dpsi_R_b_n = np.array([
        [-c2*s3, -c1*c3-s1*s2*s3, s1*c3-c1*s2*s3],
        [c2*c3, -c1*s3+s1*s2*c3, s1*s3+c1*s2*c3],
        [0, 0, 0]
    ])

    derivative_mat = np.zeros((3,3))
    derivative_mat[:, 0] = d_by_dphi_R_b_n @ v_nb_b
    derivative_mat[:, 1] = d_by_dtheta_R_b_n @ v_nb_b
    derivative_mat[:, 2] = d_by_dpsi_R_b_n @ v_nb_b

    return derivative_mat

def d_by_dTheta_nb_J2_w_nb_b(Theta_nb, w_nb_b):
    
    phi = Theta_nb[0]
    theta = Theta_nb[1]
    psi = Theta_nb[2]

    s1 = np.sin(phi); c1 = np.cos(phi)
    s2 = np.sin(theta); c2 = np.cos(theta)
    s3 = np.sin(psi); c3 = np.cos(psi)

    # J2 = np.array([
    #     [1, s1*s2/c2, c1*s2/c2],
    #     [0, c1, -s1],
    #     [0, s1/c2, c1/c2]
    # ])

    d_by_dphi_J2 = np.array([
        [0, c1*s2/c2, -s1*s2/c2],
        [0, -s1, -c1],
        [0, c1/c2, -s1/c2]
    ])

    d_by_dtheta_J2 = np.array([
        [0, s1/(c2**2), c1/(c2**2)],
        [0, 0, 0],
        [0, s1*s2/(c2**2), c1*s2/(c2**2)]
    ])

    d_by_dpsi_J2 = np.zeros((3,3))

    derivative_mat = np.zeros((3,3))
    derivative_mat[:, 0] = d_by_dphi_J2 @ w_nb_b
    derivative_mat[:, 1] = d_by_dtheta_J2 @ w_nb_b
    derivative_mat[:, 2] = d_by_dpsi_J2 @ w_nb_b

    return derivative_mat

def d_by_dq_b_n_R_b_n_v_nb_b(q_b_n, v_nb_b):
    eta = q_b_n[0]
    e1 = q_b_n[1]
    e2 = q_b_n[2]
    e3 = q_b_n[3]

    d_by_deta_R_b_n = np.array([
        [0, -2*e3, 2*e2],
        [2*e3, 0, -2*e1],
        [-2*e2, 2*e1, 0]
    ])

    d_by_deps1_R_b_n = np.array([
        [0, 2*e2, 2*e3],
        [2*e2, -4*e1, 2*eta],
        [2*e3, 2*eta, -4*e1]
    ])

    d_by_deps2_R_b_n = np.array([
        [-4*e2, 2*e1, 2*eta],
        [2*e1, 0, 2*e3],
        [-2*eta, 2*e3, -4*e2]
    ])

    d_by_deps3_R_b_n = np.array([
        [-4*e3, -2*eta, 2*e1],
        [2*eta, -4*e3, 2*e2],
        [2*e1, 2*e2, 0]
    ])

    derivative_mat = np.zeros((3, 4))
    derivative_mat[:, 0] = d_by_deta_R_b_n @ v_nb_b
    derivative_mat[:, 1] = d_by_deps1_R_b_n @ v_nb_b
    derivative_mat[:, 2] = d_by_deps2_R_b_n @ v_nb_b
    derivative_mat[:, 3] = d_by_deps3_R_b_n @ v_nb_b

    return derivative_mat

def d_by_dq_b_n_J2_w_nb_b(q_b_n, w_nb_b):
    
    d_by_deta_J2 = 0.5 * np.array([
        [0,0,0],
        [1,0,0],
        [0,1,0],
        [0,0,1]
    ])

    d_by_deps1_J2 = 0.5 * np.array([
        [-1,0,0],
        [0,0,0],
        [0,0,-1],
        [0,1,0]
    ])

    d_by_deps2_J2 = 0.5 * np.array([
        [0,-1,0],
        [0,0,1],
        [0,0,0],
        [-1,0,0]
    ])

    d_by_deps3_J2 = 0.5 * np.array([
        [0,0,-1],
        [0,-1,0],
        [1,0,0],
        [0,0,0]
    ])

    derivative_mat = np.zeros((4,4))
    derivative_mat[:, 0] = d_by_deta_J2 @ w_nb_b
    derivative_mat[:, 1] = d_by_deps1_J2 @ w_nb_b
    derivative_mat[:, 2] = d_by_deps2_J2 @ w_nb_b
    derivative_mat[:, 3] = d_by_deps3_J2 @ w_nb_b

    return derivative_mat



    
