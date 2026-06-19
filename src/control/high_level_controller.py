import numpy as np
from common import globals
from common.parameters import parms


def set_command_step(cmd_des, cmd_curr, cmd_min, cmd_max, cmd_rate):
    step = np.sign(cmd_des - cmd_curr) * min(abs(cmd_des - cmd_curr), cmd_rate)
    return float(np.clip(cmd_curr + step, cmd_min, cmd_max))


def high_level_control(vx_des=0.0, vy_des=0.0, omega_des=0.0):
    if globals.prev_step < globals.step:
        globals.prev_step = globals.step
        globals.xdot_ref   = set_command_step(vx_des, globals.xdot_ref,
                                 parms.vx_min, parms.vx_max, parms.dvx)
        globals.ydot_ref   = set_command_step(vy_des, globals.ydot_ref,
                                 parms.vy_min, parms.vy_max, parms.dvy)
        globals.psidot_ref = set_command_step(omega_des, globals.psidot_ref,
                                 parms.omega_min, parms.omega_max, parms.domega)
