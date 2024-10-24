import numpy as np
import module_kinematics as kin

def rotate_vec_by_quat(vec_a, q_a_b):
    q_vec_a = np.zeros(4)
    q_vec_a[1:] = vec_a
    vec_b = kin.quat_multiply(kin.quat_multiply(q_a_b, q_vec_a), kin.quat_conjugate(q_a_b))[1:]
    return vec_b

def quat_quat_rotate(q1, q2):
    return kin.quat_multiply(kin.quat_multiply(q2, q1), kin.quat_conjugate(q2))

    

vec_gcs = np.array([1.0, 0.0, 0.0])

Theta_nb = np.array([np.pi/3, 0, np.pi/6])

q_b_n = kin.eul_to_quat(Theta_nb)

R_b_n = kin.eul_to_rotm(Theta_nb)
R_n_b = R_b_n.T

q_n_b = kin.rotm_to_quat(R_n_b)

print(q_b_n)
print(q_n_b)

vec_bcs = R_n_b @ vec_gcs

vec_bcs_new = rotate_vec_by_quat(vec_gcs, q_n_b)

print(vec_gcs)
print(vec_bcs)
print(vec_bcs_new)


q_a_b = kin.eul_to_quat(np.array([np.pi/3, np.pi/6, np.pi/4]))
q_b_c = kin.eul_to_quat(np.array([np.pi/4, np.pi/3, np.pi/6]))

q_a_c = kin.quat_multiply(q_b_c, q_a_b)

v1 = np.array([1.3, 3.4, 7.8])

print(rotate_vec_by_quat(rotate_vec_by_quat(v1, q_a_b), q_b_c))
print(rotate_vec_by_quat(v1, q_a_c))

print(kin.quat_multiply(kin.quat_conjugate(q_a_c), q_a_c))