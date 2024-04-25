import numpy as np
import warnings

# Coordinate frames:
#
# 1. Earth Centered Inertial (ECI) frame - {i}
# 2. Earth Centered Earth Fixed (ECEF) frame - {e}
# 3. North-East-Down (NED) frame - {n}
# 4. BODY frame - {b}
# 5. FLOW frame - {f}
#
# You may further assume the following:
# 
# eul = [phi, theta, psi] 
# 
# with the order of rotation being ZYX and
#   phi being angle about x-axis
#   theta being angle about y-axis
#   psi being angle about z-axis
#
# We assume ZYX rotation order in this file. For example,
# this means that the NED frame is rotated about 
# its z-axis by angle psi followed by a rotation about 
# the resultant y-axis by angle theta followed by a 
# rotation about the resultant x-axis by angle phi
# to reach the BODY frame.
#
# quat = [qw, qx, qy, qz] is considered to be an unit quaternion
#
# rotm = 3 x 3 matrix
# 
# The rotation matrix rotm when pre-multiplied by a vector
# in BODY frame will yield a vector in NED frame

# Compute the rotation matrix from Euler angles
def eul_to_rotm(eul, order='ZYX', deg=False):
    rotm = np.eye(3, dtype=float)

    if order != 'ZYX':
        raise ValueError('Any order other than ZYX is not currently available!')

    # Write your code here

    if order == 'ZYX':
        
        if deg:
            phi = eul[0] * np.pi / 180
            theta = eul[1] * np.pi / 180
            psi = eul[2] * np.pi / 180
        else:
            phi = eul[0]
            theta = eul[1]
            psi = eul[2]

        c1 = np.cos(phi); s1 = np.sin(phi)
        c2 = np.cos(theta); s2 = np.sin(theta)
        c3 = np.cos(psi); s3 = np.sin(psi)

        rotm[0, 0] = c2 * c3
        rotm[0, 1] = -c1 * s3 + s1 * s2 * c3
        rotm[0, 2] = s1 * s3 + c1 * s2 * c3
        rotm[1, 0] = c2 * s3
        rotm[1, 1] = c1 * c3 + s1 * s2 * s3
        rotm[1, 2] = -s1 * c3 + c1 * s2 * s3
        rotm[2, 0] = -s2
        rotm[2, 1] = s1 * c2
        rotm[2, 2] = c1 * c2

    return rotm

# Compute Euler angles from rotation matrix
def rotm_to_eul(rotm, order='ZYX', prev_eul=None, deg=False):
    eul = np.zeros(3, dtype=float)
    
    if order != 'ZYX':
        raise ValueError('Any order other than ZYX is not currently available!')

    # Write your code here

    if order == 'ZYX':
    
        theta1 = np.arcsin(-rotm[2, 0])
        if theta1 > 0:
            theta2 = np.pi - theta1
        else:
            theta2 = -np.pi - theta1
        
        if theta1 == np.pi/2:
            phi1 = np.arctan2(rotm[0,1], rotm[1,1])
            psi1 = 0

            phi2 = phi1
            psi2 = psi1        

        elif theta1 == -np.pi/2:
            phi1 = np.arctan2(-rotm[0,1], rotm[1,1])
            psi1 = 0

            phi2 = phi1
            psi2 = psi1

        else:
            phi1 = np.arctan2(rotm[2,1], rotm[2,2])
            phi2 = np.arctan2(-rotm[2,1], -rotm[2,2])

            psi1 = np.arctan2(rotm[1,0], rotm[0,0])
            psi2 = np.arctan2(-rotm[1,0], -rotm[0,0])
        
        eul1 = np.array([phi1, theta1, psi1])
        eul2 = np.array([phi2, theta2, psi2])

        if prev_eul is not None:
            if np.linalg.norm(eul1 - prev_eul) >= np.linalg.norm(eul2 - prev_eul):
                eul = eul1
            else:
                eul = eul2
        else:
            warnings.warn(f'Both ({eul1[0]*180/np.pi:.2f}, {eul1[1]*180/np.pi:.2f}, {eul1[2]*180/np.pi:.2f}) and ({eul2[0]*180/np.pi:.2f}, {eul2[1]*180/np.pi:.2f}, {eul2[2]*180/np.pi:.2f}) are possible. But the first set is chosen!')
            eul = eul1
        
        if deg:
            eul = eul * 180 / np.pi

    return eul

# Compute quaternion from euler angles
def eul_to_quat(eul, order='ZYX', deg=False):
    quat = np.zeros(4, dtype=float)

    if order != 'ZYX':
        raise ValueError('Any order other than ZYX is not currently available!')

    # Write your code here

    if order == 'ZYX':
        
        if deg:
            phi = eul[0] * np.pi / 180
            theta = eul[1] * np.pi / 180
            psi = eul[2] * np.pi / 180
        else:
            phi = eul[0]
            theta = eul[1]
            psi = eul[2]

        quat[0] = np.cos(psi/2) * np.cos(theta/2) * np.cos(phi/2) + np.sin(psi/2) * np.sin(theta/2) * np.sin(phi/2)
        quat[1] = np.cos(psi/2) * np.cos(theta/2) * np.sin(phi/2) - np.sin(psi/2) * np.sin(theta/2) * np.cos(phi/2)
        quat[2] = np.sin(psi/2) * np.cos(theta/2) * np.sin(phi/2) + np.cos(psi/2) * np.sin(theta/2) * np.cos(phi/2)
        quat[3] = np.sin(psi/2) * np.cos(theta/2) * np.cos(phi/2) - np.cos(psi/2) * np.sin(theta/2) * np.sin(phi/2)

        quat = quat / np.linalg.norm(quat)

    return quat

# Compute Euler angles from quaternion
def quat_to_eul(quat, order='ZYX', deg=False):
    eul = np.zeros(3, dtype=float)
    
    if order != 'ZYX':
        raise ValueError('Any order other than ZYX is not currently available!')

    # Write your code here

    if order == 'ZYX':
        qw = quat[0]
        qx = quat[1]
        qy = quat[2]
        qz = quat[3]

        phi = np.arctan2(2 * (qy * qz + qx * qw), 1 - 2 * (qx ** 2 + qy ** 2))
        theta = np.arcsin(2 * (qy * qw - qx * qz))
        psi = np.arctan2(2 * (qx * qy + qz * qw), 1 - 2 * (qy ** 2 + qz ** 2))

        eul[0] = phi
        eul[1] = theta
        eul[2] = psi

    if deg:
        eul = eul * 180 / np.pi

    return eul

# Compute the rotation matrix from quaternion
def quat_to_rotm(quat):
    rotm = np.eye(3, dtype=float)

    # Write your code here

    qw = quat[0]
    qx = quat[1]
    qy = quat[2]
    qz = quat[3]

    # Calculate the rotation matrix elements
    r11 = 1 - 2*qy**2 - 2*qz**2
    r12 = 2*qx*qy - 2*qz*qw
    r13 = 2*qx*qz + 2*qy*qw
    r21 = 2*qx*qy + 2*qz*qw
    r22 = 1 - 2*qx**2 - 2*qz**2
    r23 = 2*qy*qz - 2*qx*qw
    r31 = 2*qx*qz - 2*qy*qw
    r32 = 2*qy*qz + 2*qx*qw
    r33 = 1 - 2*qx**2 - 2*qy**2
    
    # Create the rotation matrix
    rotm = np.array([[r11, r12, r13],
                    [r21, r22, r23],
                    [r31, r32, r33]])

    return rotm

# Compute quaternion from rotation matrix
def rotm_to_quat(rotm):
    quat = np.zeros(4, dtype=float)

    # Write your code here

    r11 = rotm[0, 0]
    r12 = rotm[0, 1]
    r13 = rotm[0, 2]

    r21 = rotm[1, 0]
    r22 = rotm[1, 1]
    r23 = rotm[1, 2]

    r31 = rotm[2, 0]
    r32 = rotm[2, 1]
    r33 = rotm[2, 2]

    qw = np.sqrt(r11 + r22 + r33 + 1) / 2
    qx = (r32 - r23) / (2 * qw)
    qy = (r13 - r31) / (2 * qw)
    qz = (r21 - r12) / (2 * qw)

    quat[0] = qw
    quat[1] = qx
    quat[2] = qy
    quat[3] = qz

    quat = quat / np.linalg.norm(quat)

    return quat

# Compute T matrix (or J2 matrix) from Euler angles
def eul_rate_matrix(eul, order='ZYX', deg=False):
    Tmat = np.zeros((3, 3))

    if order != 'ZYX':
        raise ValueError('Any order other than ZYX is not currently available!')

    # Write your code here

    if order == 'ZYX':

        if deg:
            phi = eul[0] * np.pi / 180
            theta = eul[1] * np.pi / 180
            psi = eul[2] * np.pi / 180
        else:
            phi = eul[0]
            theta = eul[1]
            psi = eul[2]

        Tmat[0, 0] = 1
        Tmat[0, 1] = np.sin(phi) * np.tan(theta)
        Tmat[0, 2] = np.cos(phi) * np.tan(theta)

        Tmat[1, 1] = np.cos(phi)
        Tmat[1, 2] = -np.sin(phi)

        Tmat[2, 1] = np.sin(phi) / np.cos(theta)
        Tmat[2, 2] = np.cos(phi) / np.cos(theta)
    
    return Tmat

# Compute T matrix (or J2 matrix) from quaternion
def quat_rate_matrix(quat):
    Tmat = np.zeros((4, 3))

    qw = quat[0]
    qx = quat[1]
    qy = quat[2]
    qz = quat[3]

    Tmat[0, 0] = -qx
    Tmat[0, 1] = -qy
    Tmat[0, 2] = -qz
    Tmat[1, 0] = qw
    Tmat[1, 1] = -qz
    Tmat[1, 2] = qy
    Tmat[2, 0] = qz
    Tmat[2, 1] = qw
    Tmat[2, 2] = -qx
    Tmat[3, 0] = -qy
    Tmat[3, 1] = qx
    Tmat[3, 2] = qw

    return 0.5 * Tmat

# Compute Euler rate from angular velocity
def eul_rate(eul, w, order='ZYX'):
    deul = np.zeros(3)

    if order != 'ZYX':
        raise ValueError('Any order other than ZYX is not currently available!')

    # Write your code here

    if order == 'ZYX':
        Tmat = eul_rate_matrix(eul, order=order)
        deul = Tmat @ w
    
    return deul

# Compute quaternion rate from angular velocity
def quat_rate(quat, w):
    dquat = np.zeros(4)

    # Write your code here

    Tmat = quat_rate_matrix(quat)
    dquat = Tmat @ w
    
    return dquat

def ssa(ang, deg=False):
    if deg:
        ang = (ang + 180) % (360.0) - 180.0
    else:
        ang = (ang + np.pi) % (2 * np.pi) - np.pi
    return ang

def ned_to_llh(ned, llh0):
    xn = ned[0]
    yn = ned[1]
    zn = ned[2]

    mu0 = llh0[0] * np.pi / 180
    l0 = llh0[1] * np.pi / 180
    h0 = llh0[2]

    re = 6378137.0
    rp = 6356752.314245
    e = np.sqrt(1 - (rp/re) ** 2)

    RN = re / np.sqrt(1 - (e * np.sin(mu0))**2)
    RM = re * (1 - e ** 2) / (1 - (e * np.sin(mu0))**2)

    dmu = xn * np.arctan2(1, RN)
    dl = yn * np.arctan2(1, RM * np.cos(mu0))

    mu = ssa(mu0 + dmu) * 180 / np.pi
    l = ssa(l0 + dl) * 180 / np.pi
    h = h0 - zn

    llh = np.array([mu, l, h])
    return llh

def llh_to_ned(llh, llh0):

    mu0 = llh0[0] * np.pi / 180
    l0 = llh0[1] * np.pi / 180
    h0 = llh0[2]

    dllh = llh - llh0
    dmu = dllh[0] * np.pi / 180
    dl = dllh[1] * np.pi / 180
    dh = dllh[2]

    re = 6378137.0
    rp = 6356752.314245
    e = np.sqrt(1 - (rp/re) ** 2)

    RN = re / np.sqrt(1 - (e * np.sin(mu0))**2)
    RM = re * (1 - e ** 2) / (1 - (e * np.sin(mu0))**2)

    xn = ssa(dmu) / np.arctan2(1, RN)
    yn = ssa(dl) / np.arctan2(1, RM * np.cos(mu0))
    zn = -dh

    ned = np.array([xn, yn, zn])
    return ned

def generate_waypoints():

    # NED tangent point location - At the sluez gate in the lake
    mu_dat = 12.993012 
    l_dat = 80.239142
    h_dat = 0.0
    llh_dat = np.array([mu_dat, l_dat, h_dat])

    # Location of Water Intake Structure in Lake
    mu0 = 12.993660
    l0 = 80.239398
    h0 = 0.0
    llh0 = np.array([mu0, l0, h0])
    
    # Position vector of water intake structure in NED frame
    rn0 = llh_to_ned(llh0, llh_dat)

    # Waypoints to be tracked
    L = 40
    # wp0 = ned_to_llh(rn0 + np.array([-55, -L/2, 0]), llh_dat)
    wp1 = ned_to_llh(rn0 + np.array([-L/2, L/2, 0]), llh_dat)
    wp2 = ned_to_llh(rn0 + np.array([-L/2, -L/2, 0]), llh_dat)
    wp3 = ned_to_llh(rn0 + np.array([L/2, -L/2, 0]), llh_dat)
    wp4 = ned_to_llh(rn0 + np.array([L/2, L/2, 0]), llh_dat)
    wp5 = ned_to_llh(rn0 + np.array([-L/2, L/2, 0]), llh_dat)
    # wp6 = ned_to_llh(rn0 + np.array([-55, -L/2, 0]), llh_dat)

    wp = []
    # wp.append(wp0.tolist())
    wp.append(wp1.tolist())
    wp.append(wp2.tolist())
    wp.append(wp3.tolist())
    wp.append(wp4.tolist())
    wp.append(wp5.tolist())
    # wp.append(wp6.tolist())

    wp = np.array(wp)
    return wp

def rotm_ned_to_ecef(llh):
    mu = llh[0] * np.pi / 180
    l = llh[1] * np.pi / 180
    h = llh[2]

    R1 = np.zeros((3,3))
    R2 = np.zeros((3,3))

    R1[0, 0] = np.cos(l)
    R1[0, 1] = -np.sin(l)
    R1[1, 0] = np.sin(l)
    R1[1, 1] = np.cos(l)
    R1[2, 2] = 1

    R2[0, 0] = np.cos(-mu-np.pi/2)
    R2[0, 2] = np.sin(-mu-np.pi/2)
    R2[2, 0] = -np.sin(-mu-np.pi/2)
    R2[2, 2] = np.cos(-mu-np.pi/2)
    R2[1, 1] = 1

    rotm = R1 @ R2
    return rotm

def Smat(vec):
    S = np.zeros((3,3))
    S[0, 1] = -vec[2]
    S[0, 2] = vec[1]
    S[1, 2] = -vec[0]
    return S - S.T

if __name__ == "__main__":
    wp = generate_waypoints()
    print(wp)