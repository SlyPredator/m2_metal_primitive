import numpy as np

from .parameters import parms


def init():
    global time, fsm, t_fsm, t_i, t_f, lx_ref, ly_ref, lz_ref
    global lxdot_ref, lydot_ref, lzdot_ref, lx_i, lx_f, ly_i, ly_f, lz_i, lz_f
    global q_ref, u_ref, q_act, u_act, trq, xdot_ref, ydot_ref, psidot_ref, step, prev_step
    global pos_quat_trunk, vel_angvel_trunk
    
    time = 0.0
    fsm = np.array([parms.fsm_stand]*4)
    t_fsm = np.zeros(4)

    t_i = np.zeros(4)
    t_f = np.array([parms.t_stand]*4)

    lx_ref = np.zeros(4)
    ly_ref = np.zeros(4)
    lz_ref = np.zeros(4)
    lxdot_ref = np.zeros(4)
    lydot_ref = np.zeros(4)
    lzdot_ref = np.zeros(4)
    
    lx_i = np.zeros(4)
    lx_f = np.zeros(4)
    ly_i = np.zeros(4)
    ly_f = np.zeros(4)
    lz_i = np.array([parms.lz0]*4)
    lz_f = np.array([parms.lz0]*4)

    q_ref = np.zeros(12)
    u_ref = np.zeros(12)
    q_act = np.zeros(12)
    u_act = np.zeros(12)
    trq = np.zeros(12)

    xdot_ref = 0.0
    ydot_ref = 0.0
    psidot_ref = 0.0
    step = 0
    prev_step = 0

    pos_quat_trunk = np.zeros(7)
    vel_angvel_trunk = np.zeros(6)


init()

