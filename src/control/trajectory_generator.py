import numpy as np
from common import globals
from common.parameters import parms
from kinematics.kinematics import (inverse_kinematics_analytic,
                                   jac_end_effector_leg)


def quintic_poly(t, t0, tf, q0, qf):
    t = np.clip(t, t0, tf)
    T = tf - t0
    if abs(T) < 1e-9:
        return q0, 0.0, 0.0
    s = (t - t0) / T
    dq = qf - q0
    
    q     = q0 + dq * (10*s**3 - 15*s**4 + 6*s**5)
    qdot  = (dq / T) * (30*s**2 - 60*s**3 + 30*s**4)
    qddot = (dq / T**2) * (60*s - 180*s**2 + 120*s**3)
    return q, qdot, qddot

def cartesian_traj():
    time = globals.time
    for leg_no in range(4):
        fsm = globals.fsm[leg_no]
        dt  = time - globals.t_fsm[leg_no]          # elapsed time in current FSM phase
        ti  = globals.t_i[leg_no]
        tf  = globals.t_f[leg_no]

        if fsm in (parms.fsm_stand, parms.fsm_stance):
            # foot planted in x,y; only height interpolated
            lz, lzd, _ = quintic_poly(dt, ti, tf, globals.lz_i[leg_no], globals.lz_f[leg_no])
            globals.lz_ref[leg_no]    = lz
            globals.lzdot_ref[leg_no] = lzd
            globals.lx_ref[leg_no]    = 0.0
            globals.lxdot_ref[leg_no] = 0.0
            globals.ly_ref[leg_no]    = 0.0
            globals.lydot_ref[leg_no] = 0.0

        elif fsm == parms.fsm_swing:
            # fore-aft and lateral follow Raibert arc set by state_machine()
            lx, lxd, _ = quintic_poly(dt, ti, tf, globals.lx_i[leg_no], globals.lx_f[leg_no])
            ly, lyd, _ = quintic_poly(dt, ti, tf, globals.ly_i[leg_no], globals.ly_f[leg_no])
            globals.lx_ref[leg_no] = lx
            globals.lxdot_ref[leg_no] = lxd
            globals.ly_ref[leg_no] = ly
            globals.lydot_ref[leg_no] = lyd
            # two-segment lz: up in first half, back down in second half
            half = 0.5 * tf
            if dt <= half:
                lz, lzd, _ = quintic_poly(dt, ti, half,
                                          globals.lz_i[leg_no], globals.lz_f[leg_no])
            else:
                lz, lzd, _ = quintic_poly(dt, half, tf,
                                          globals.lz_f[leg_no], globals.lz_i[leg_no])
            globals.lz_ref[leg_no] = lz
            globals.lzdot_ref[leg_no] = lzd

def joint_traj():
    for leg_no in range(4):
        X  = np.array([globals.lx_ref[leg_no],
                       globals.ly_ref[leg_no],
                       globals.lz_ref[leg_no]])       # Cartesian foot reference
        Xd = np.array([globals.lxdot_ref[leg_no],
                       globals.lydot_ref[leg_no],
                       globals.lzdot_ref[leg_no]])     # Cartesian foot velocity reference

        q_leg = inverse_kinematics_analytic(X, leg_no)         # q_ref = IK(l_ref)
        globals.q_ref[3*leg_no : 3*leg_no+3] = q_leg

        J = jac_end_effector_leg(q_leg, leg_no)
        globals.u_ref[3*leg_no : 3*leg_no+3] = np.linalg.inv(J) @ Xd  # qdot_ref = J^-1 * ldot_ref
