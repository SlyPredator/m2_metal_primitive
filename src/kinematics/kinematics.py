from types import SimpleNamespace

import numpy as np
from common import globals
from common import utility as ram
from common.parameters import parms
from common.robot_data import robot


def forward_kinematics_leg(q, leg_no):
    L1 = 0.1915
    L2 = 0.1915
    w = 0.0811 if leg_no in (0, 1) else 0.0796
    if leg_no in (0, 2): 
        w = -w
    dx = 0.0
    
    c1, s1 = np.cos(q[0]), np.sin(q[0])
    c2, s2 = np.cos(q[1]), np.sin(q[1])
    c3, s3 = np.cos(q[2]), np.sin(q[2])
    x3, y3, z3 = 0.0, 0.0, -L2
    
    # Analytical local propagation
    x2 = c3 * x3 + s3 * z3
    y2 = y3
    z2 = -s3 * x3 + c3 * z3 - L1
    x1 = c2 * x2 + s2 * z2 + dx
    y1 = y2 + w
    z1 = -s2 * x2 + c2 * z2
    eff = np.array([
        x1,
        c1 * y1 - s1 * z1,
        s1 * y1 + c1 * z1
    ])
    
    # Pre-calculated knee and hip vectors in world/hip-relative frame
    o01 = np.array([0.0, 0.0, 0.0])
    o02 = np.array([dx, w * c1, w * s1])
    o03 = np.array([dx - L1 * s2, w * c1 + L1 * s1 * c2, w * s1 - L1 * c1 * c2])
    
    return SimpleNamespace(end_eff_pos=eff, o01=o01, o02=o02, o03=o03, c1=c1, s1=s1)

def jac_end_effector_leg(q, leg_no):
    sol = forward_kinematics_leg(q, leg_no)
    e0  = sol.end_eff_pos
    o01 = sol.o01
    o02 = sol.o02
    o03 = sol.o03
    
    v1 = e0 - o01
    v2 = e0 - o02
    v3 = e0 - o03
    
    c1, s1 = sol.c1, sol.s1
    
    col1 = np.array([0.0, -v1[2], v1[1]])
    col2 = np.array([c1 * v2[2] - s1 * v2[1], s1 * v2[0], -c1 * v2[0]])
    col3 = np.array([c1 * v3[2] - s1 * v3[1], s1 * v3[0], -c1 * v3[0]])
    
    return np.column_stack([col1, col2, col3])

def inverse_kinematics_analytic(X_ref, leg_no=0):
    L1 = 0.1915
    L2 = 0.1915
    lx, ly, lz = X_ref
    l = np.sqrt(lx**2 + ly**2 + lz**2)
    q_a = np.arcsin(np.clip(ly / l, -1, 1))
    q_k = -np.pi + np.arccos(np.clip((2*L1**2 - l**2) / (2*L1**2), -1, 1))
    q_h = -0.5 * q_k + np.arcsin(np.clip(-lx / l, -1, 1))
    return np.array([q_a, q_h, q_k])

def forward_kinematics_robot(q):
    eff_local = robot.params.end_eff_pos_local
    q_trunk   = q[:7]
    q_legs    = q[7:]
    
    if not hasattr(robot.body[1], 'H_global'):
        for i in range(1, len(robot.body) + 1):
            robot.body[i].H_global = np.eye(4)
        for i in range(2, len(robot.body) + 1):
            robot.body[i].R_nominal = ram.quat2rotation(robot.body[i].quat)
            robot.body[i].H_local = np.eye(4)
            robot.body[i].H_local[:3, 3] = robot.body[i].pos
    j = 0
    for i in range(2, len(robot.body) + 1):
        axis_id = int(np.argmax(np.abs(robot.body[i].joint_axis)))
        
        c, s = np.cos(q_legs[j]), np.sin(q_legs[j])
        if axis_id == 0:
            R_q = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
        elif axis_id == 1:
            R_q = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
        else:
            R_q = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        j += 1
        
        robot.body[i].R_local = robot.body[i].R_nominal @ R_q
        robot.body[i].H_local[:3, :3] = robot.body[i].R_local
    pos_t = q_trunk[:3]
    R_t   = ram.quat2rotation(q_trunk[3:])
    robot.body[1].H_global[:3, :3] = R_t
    robot.body[1].H_global[:3, 3] = pos_t
    i = 2
    for _ in range(4):
        tmp = robot.body[1].H_global
        for _ in range(3):
            robot.body[i].H_global = tmp @ robot.body[i].H_local
            tmp = robot.body[i].H_global
            i += 1
    in_sho  = np.array([robot.body[1+k*3-2].H_global[:3, 3] for k in range(1,5)])
    out_sho = np.array([robot.body[1+k*3-1].H_global[:3, 3] for k in range(1,5)])
    elbow   = np.array([robot.body[1+k*3  ].H_global[:3, 3] for k in range(1,5)])
    
    end_eff = np.empty((4, 3))
    for k in range(1, 5):
        H = robot.body[1+k*3].H_global
        end_eff[k-1] = H[:3, :3] @ eff_local + H[:3, 3]
        
    com_tmp = robot.body[1].H_global[:3, :3] @ robot.body[1].ipos + robot.body[1].H_global[:3, 3]
    return robot, SimpleNamespace(
        in_shoulder_pos  = in_sho,
        out_shoulder_pos = out_sho,
        elbow_pos        = elbow,
        end_eff_pos      = end_eff,
        trunk_com_pos    = com_tmp)
