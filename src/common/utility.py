import numpy as np

_FLOAT_EPS = np.finfo(np.float64).eps
_EPS4 = _FLOAT_EPS * 4.0

def rotation(angle, axis):
    c, s = np.cos(angle), np.sin(angle)
    if axis == 0:
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    elif axis == 1:
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    elif axis == 2:
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    raise ValueError('axis must be 0, 1, or 2')

def quat2rotation(q):
    q0, q1, q2, q3 = q
    return np.array([
        [q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3),           2*(q1*q3+q0*q2)],
        [2*(q1*q2+q0*q3),         q0**2-q1**2+q2**2-q3**2,   2*(q2*q3-q0*q1)],
        [2*(q1*q3-q0*q2),         2*(q2*q3+q0*q1),           q0**2-q1**2-q2**2+q3**2]])

def rotation2quat(R):
    q0 = np.sqrt(max(0.0, 1+R[0,0]+R[1,1]+R[2,2]))/2
    q1 = np.sqrt(max(0.0, 1+R[0,0]-R[1,1]-R[2,2]))/2
    q2 = np.sqrt(max(0.0, 1-R[0,0]+R[1,1]-R[2,2]))/2
    q3 = np.sqrt(max(0.0, 1-R[0,0]-R[1,1]+R[2,2]))/2
    if q0 >= max(q1, q2, q3):
        q1 = (R[2,1]-R[1,2])/(4*q0)
        q2 = (R[0,2]-R[2,0])/(4*q0)
        q3 = (R[1,0]-R[0,1])/(4*q0)
    elif q1 >= max(q0, q2, q3):
        q0 = (R[2,1]-R[1,2])/(4*q1)
        q2 = (R[1,0]+R[0,1])/(4*q1)
        q3 = (R[0,2]+R[2,0])/(4*q1)
    elif q2 >= max(q0, q1, q3):
        q0 = (R[0,2]-R[2,0])/(4*q2)
        q1 = (R[1,0]+R[0,1])/(4*q2)
        q3 = (R[2,1]+R[1,2])/(4*q2)
    else:
        q0 = (R[1,0]-R[0,1])/(4*q3)
        q1 = (R[0,2]+R[2,0])/(4*q3)
        q2 = (R[2,1]+R[1,2])/(4*q3)
    return np.array([q0, q1, q2, q3])

def quat2axisangle(quat):
    q0, qx, qy, qz = quat
    angle = 2*np.arccos(np.clip(q0, -1, 1))
    sh = np.sin(angle/2)
    axis = np.array([1, 0, 0]) if sh < 1e-6 else np.array([qx, qy, qz])/sh
    return axis, angle

def euler2rotation(euler):
    return rotation(euler[0], 0) @ rotation(euler[1], 1) @ rotation(euler[2], 2)

def rotation2euler(R):
    theta = np.arcsin(np.clip(R[0,2], -1, 1))
    ct = np.cos(theta)
    psi = np.arcsin(np.clip(-R[0,1]/ct, -1, 1))
    phi = np.arcsin(np.clip(-R[1,2]/ct, -1, 1))
    return np.array([phi, theta, psi])

def quat2euler(q):
    return rotation2euler(quat2rotation(q))

def euler2quat(euler):
    return rotation2quat(euler2rotation(euler))

def quat_conjugate(q):
    c = -q.copy()
    c[0] = q[0]
    return c

def quat_product(q, p):
    q0, p0 = q[0], p[0]
    qv, pv = q[1:4], p[1:4]
    return np.r_[q0*p0-qv@pv, q0*pv+p0*qv+np.cross(qv, pv)]

def quat_normalize(q):
    return q / np.linalg.norm(q, axis=-1, keepdims=True)

def vec2skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])

def mat2quat(R):
    tr = R[0,0] + R[1,1] + R[2,2]
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (R[2,1] - R[1,2]) / S
        qy = (R[0,2] - R[2,0]) / S
        qz = (R[1,0] - R[0,1]) / S
    elif (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
        S = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
        qw = (R[2,1] - R[1,2]) / S
        qx = 0.25 * S
        qy = (R[0,1] + R[1,0]) / S
        qz = (R[0,2] + R[2,0]) / S
    elif R[1,1] > R[2,2]:
        S = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
        qw = (R[0,2] - R[2,0]) / S
        qx = (R[0,1] + R[1,0]) / S
        qy = 0.25 * S
        qz = (R[1,2] + R[2,1]) / S
    else:
        S = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
        qw = (R[1,0] - R[0,1]) / S
        qx = (R[0,2] + R[2,0]) / S
        qy = (R[1,2] + R[2,1]) / S
        qz = 0.25 * S
    return np.array([qw, qx, qy, qz])

def quat2mat(quat):
    quat = np.asarray(quat, dtype=np.float64)
    w, x, y, z = quat[..., 0], quat[..., 1], quat[..., 2], quat[..., 3]
    Nq = np.sum(quat*quat, axis=-1)
    s = 2.0/Nq
    X, Y, Z = x*s, y*s, z*s
    wX, wY, wZ = w*X, w*Y, w*Z
    xX, xY, xZ = x*X, x*Y, x*Z
    yY, yZ, zZ = y*Y, y*Z, z*Z
    mat = np.empty(quat.shape[:-1]+(3,3), dtype=np.float64)
    mat[...,0,0]=1.0-(yY+zZ); mat[...,0,1]=xY-wZ;       mat[...,0,2]=xZ+wY
    mat[...,1,0]=xY+wZ;       mat[...,1,1]=1.0-(xX+zZ); mat[...,1,2]=yZ-wX
    mat[...,2,0]=xZ-wY;       mat[...,2,1]=yZ+wX;       mat[...,2,2]=1.0-(xX+yY)
    return np.where((Nq>_FLOAT_EPS)[...,np.newaxis,np.newaxis], mat, np.eye(3))

def quat2bryant(quat):
    return mat2bryant(quat2mat(quat))

def bryant2quat(euler):
    euler = np.asarray(euler, dtype=np.float64)
    ai, aj, ak = euler[..., 2]/2, -euler[..., 1]/2, euler[..., 0]/2
    si, sj, sk = np.sin(ai), np.sin(aj), np.sin(ak)
    ci, cj, ck = np.cos(ai), np.cos(aj), np.cos(ak)
    cc, cs, sc, ss = ci*ck, ci*sk, si*ck, si*sk
    quat = np.empty(euler.shape[:-1]+(4,), dtype=np.float64)
    quat[..., 0]=cj*cc+sj*ss; quat[..., 1]=cj*cs-sj*sc
    quat[..., 2]=-(cj*ss+sj*cc); quat[..., 3]=cj*sc-sj*cs
    return quat

def mat2bryant(mat):
    mat = np.asarray(mat, dtype=np.float64)
    cy = np.sqrt(mat[..., 2, 2]**2 + mat[..., 1, 2]**2)
    cond = cy > _EPS4
    euler = np.empty(mat.shape[:-1], dtype=np.float64)
    euler[..., 2] = np.where(cond, -np.arctan2(mat[...,0,1],mat[...,0,0]),
                                  -np.arctan2(-mat[...,1,0],mat[...,1,1]))
    euler[..., 1] = -np.arctan2(-mat[...,0,2], cy)
    euler[..., 0] = np.where(cond, -np.arctan2(mat[...,1,2],mat[...,2,2]), 0.0)
    return euler

def bryant2mat(euler):
    euler = np.asarray(euler, dtype=np.float64)
    ai, aj, ak = -euler[..., 2], -euler[..., 1], -euler[..., 0]
    si, sj, sk = np.sin(ai), np.sin(aj), np.sin(ak)
    ci, cj, ck = np.cos(ai), np.cos(aj), np.cos(ak)
    cc, cs, sc, ss = ci*ck, ci*sk, si*ck, si*sk
    mat = np.empty(euler.shape[:-1]+(3,3), dtype=np.float64)
    mat[...,2,2]=cj*ck; mat[...,2,1]=sj*sc-cs; mat[...,2,0]=sj*cc+ss
    mat[...,1,2]=cj*sk; mat[...,1,1]=sj*ss+cc; mat[...,1,0]=sj*cs-sc
    mat[...,0,2]=-sj;   mat[...,0,1]=cj*si;    mat[...,0,0]=cj*ci
    return mat

def quat2angvelBody(quat, quatd):
    qc = quat_conjugate(quat)
    qc = qc/np.linalg.norm(qc)
    return 2*quat_product(qc, quatd)

def quat2angvelWorld(quat, quatd):
    qc = quat_conjugate(quat)
    qc = qc/np.linalg.norm(qc)
    return 2*quat_product(quatd, qc)

def quat2angaccBody(quat, quatd, quatdd):
    qc  = quat_conjugate(quat)
    qc  = qc /np.linalg.norm(qc)
    qdc = quat_conjugate(quatd)
    qdc = qdc/np.linalg.norm(qdc)
    return 2*quat_product(qc, quatdd)+2*quat_product(qdc, quatd)

def quat2angaccWorld(quat, quatd, quatdd):
    qc  = quat_conjugate(quat)
    qc  = qc /np.linalg.norm(qc)
    qdc = quat_conjugate(quatd)
    qdc = qdc/np.linalg.norm(qdc)
    return 2*quat_product(quatdd, qc)+2*quat_product(qdc, quatd)
