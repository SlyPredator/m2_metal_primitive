import numpy as np
from common import globals
from common import utility as ram
from common.parameters import parms
from kinematics.kinematics import (forward_kinematics_robot,
                                   jac_end_effector_leg)


def stance_force(leg_no):
    q_act          = globals.q_act.copy()            # joint angles (12,)
    pos_quat_trunk = globals.pos_quat_trunk.copy()   # [x,y,z, qw,qx,qy,qz]
    vel_angvel     = globals.vel_angvel_trunk.copy() # [xdot,ydot,zdot, wx,wy,wz]

    # Yaw decoupling: express velocity in robot-heading frame
    quat   = pos_quat_trunk[3:]
    euler  = ram.quat2bryant(quat)                   # [roll, pitch, yaw]
    Rz     = ram.rotation(euler[2], 2)               # rotation about z by yaw angle
    R      = ram.quat2mat(quat)
    R_body = Rz.T @ R                                # yaw-decoupled body rotation
    vel_b  = Rz.T @ vel_angvel[:3]                   # body-frame linear velocity

    # Run FK (with yaw removed) to get foot positions and trunk COM
    pqt_ = np.concatenate((pos_quat_trunk[:3], ram.mat2quat(R_body)))
    q    = np.concatenate((pqt_, q_act))
    _, sol = forward_kinematics_robot(q)
    eff  = sol.end_eff_pos     # foot positions in world frame, shape (4, 3)
    com  = sol.trunk_com_pos   # trunk COM position in world frame

    # Assemble A from lever arms (foot position relative to COM)
    I3 = np.eye(3)
    if leg_no == 0:            # diagonal pair A: FR (leg 0) + RL (leg 3)
        r_L, r_R = eff[0] - com, eff[3] - com
    else:                      # diagonal pair B: FL (leg 1) + RR (leg 2)
        r_L, r_R = eff[1] - com, eff[2] - com
    A = np.block([[I3,                 I3              ],
                  [ram.vec2skew(r_L),  ram.vec2skew(r_R)]])

    # Desired wrench b from PD on trunk state
    z      = pos_quat_trunk[2]
    z_ref  = -parms.lz0
    zdot   = vel_angvel[2]
    omega  = vel_angvel[3:]
    xdot   = vel_b[0]
    ydot   = vel_b[1]
    psidot = vel_angvel[5]
    
    b = np.array([
        100 * (globals.xdot_ref - xdot),                        # fwd velocity error
        100 * (globals.ydot_ref - ydot),                        # lat velocity error
        50  * (-10*(z - z_ref) - 1*zdot) + parms.mass*parms.gravity,  # height PD + gravity ff
        50  * (-10*euler[0] - 0.5*omega[0]),                    # roll -> 0
        50  * (-10*euler[1] - 0.5*omega[1]),                    # pitch -> 0
        10  * (globals.psidot_ref - psidot)])                   # yaw rate tracking

    # Solve A @ F = b via pseudo-inverse; swing legs get zero force
    F = np.linalg.pinv(A, rcond=1e-10) @ b

    if leg_no == 0:
        return F[:3], np.zeros(3), np.zeros(3), F[3:6]   # FR, _, _, RL
    else:
        return np.zeros(3), F[:3], F[3:6], np.zeros(3)   # _, FL, RR, _

def joint_control():
    for leg_no in range(4):
        fsm = globals.fsm[leg_no]
        qa  = globals.q_act[3*leg_no : 3*leg_no+3]   # actual joint angles
        ua  = globals.u_act[3*leg_no : 3*leg_no+3]   # actual joint velocities
        qr  = globals.q_ref[3*leg_no : 3*leg_no+3]   # reference joint angles
        ur  = globals.u_ref[3*leg_no : 3*leg_no+3]   # reference joint velocities

        g  = 10                                        # gain scale (Kp=100, Kd=10)
        pd = g * (-10*(qa - qr) - 1*(ua - ur))        # PD control in joint space

        if fsm == parms.fsm_stance:
            if leg_no in (0, 1):
                F0, F1, F2, F3 = stance_force(leg_no) # wrench solve for this diagonal pair
            F = [F0, F1, F2, F3][leg_no]              # this leg's contact force
            J  = jac_end_effector_leg(qr, leg_no)
            pd = pd + (-J.T @ F)                      # -J^T F: virtual work feedforward

        globals.trq[3*leg_no : 3*leg_no+3] = pd
