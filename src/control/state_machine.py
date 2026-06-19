from common import globals
from common.parameters import parms


def state_machine():
    time = globals.time
    c    = 0.183   # half hip-to-hip spacing (m) -- yaw coupling lever arm

    for leg_no in range(4):
        # Stand -> Swing (legs 0,3) or Stance (legs 1,2) after t_stand
        # Let the robot settle before starting the gait
        if time >= globals.t_fsm[leg_no]+parms.t_stand and globals.fsm[leg_no]==parms.fsm_stand:
            if leg_no in (0, 3):                         # FR, RL -> swing first
                globals.fsm[leg_no]   = parms.fsm_swing
                globals.lz_f[leg_no]  = parms.lz0 + parms.hcl  # lift to clearance height
            else:                                        # FL, RR -> stance first
                globals.fsm[leg_no]   = parms.fsm_stance
                globals.lz_f[leg_no]  = parms.lz0               # stay at ground level
            globals.t_fsm[leg_no] = time
            globals.lz_i[leg_no]  = parms.lz0
            globals.t_i[leg_no]   = 0
            globals.t_f[leg_no]   = parms.t_step

        # Stance -> Swing: set liftoff->touchdown arc endpoints (Raibert)
        # Foot travels from behind hip (-lx_i) to ahead of hip (+lx_f)
        if time >= globals.t_fsm[leg_no]+parms.t_step and globals.fsm[leg_no]==parms.fsm_stance:
            globals.fsm[leg_no]   = parms.fsm_swing
            globals.t_fsm[leg_no] = time
            globals.lz_i[leg_no]  = parms.lz0
            globals.lz_f[leg_no]  = parms.lz0 + parms.hcl      # lift during swing
            # fore-aft: swing from behind to ahead of hip
            globals.lx_i[leg_no]  = -0.5 * globals.xdot_ref * parms.t_step
            globals.lx_f[leg_no]  =  0.5 * globals.xdot_ref * parms.t_step
            # lateral: symmetric arc + yaw coupling
            globals.ly_i[leg_no]  = -0.5 * globals.ydot_ref * parms.t_step
            globals.ly_f[leg_no]  =  0.5 * globals.ydot_ref * parms.t_step
            if leg_no in (0, 1):  # front legs: positive yaw -> foot steps outward
                globals.ly_i[leg_no] -= 0.5 * c * globals.psidot_ref * parms.t_step
                globals.ly_f[leg_no] += 0.5 * c * globals.psidot_ref * parms.t_step
            else:                 # rear legs: opposite yaw coupling
                globals.ly_i[leg_no] += 0.5 * c * globals.psidot_ref * parms.t_step
                globals.ly_f[leg_no] -= 0.5 * c * globals.psidot_ref * parms.t_step

        # Swing -> Stance: set touchdown->liftoff arc endpoints (mirror of above)
        # Foot starts ahead (+lx_i) and slides backward to behind (-lx_f)
        # as the body moves forward over the planted foot during stance.
        if time >= globals.t_fsm[leg_no]+parms.t_step and globals.fsm[leg_no]==parms.fsm_swing:
            if leg_no in (0, 1):
                globals.step += 1                        # count full gait cycles
            globals.fsm[leg_no]   = parms.fsm_stance
            globals.t_fsm[leg_no] = time
            globals.lz_i[leg_no]  = parms.lz0
            globals.lz_f[leg_no]  = parms.lz0           # foot stays at ground level
            globals.lx_i[leg_no]  =  0.5 * globals.xdot_ref * parms.t_step
            globals.lx_f[leg_no]  = -0.5 * globals.xdot_ref * parms.t_step
            globals.ly_i[leg_no]  =  0.5 * globals.ydot_ref * parms.t_step
            globals.ly_f[leg_no]  = -0.5 * globals.ydot_ref * parms.t_step
            if leg_no in (0, 1):
                globals.ly_i[leg_no] += 0.5 * c * globals.psidot_ref * parms.t_step
                globals.ly_f[leg_no] -= 0.5 * c * globals.psidot_ref * parms.t_step
            else:
                globals.ly_i[leg_no] -= 0.5 * c * globals.psidot_ref * parms.t_step
                globals.ly_f[leg_no] += 0.5 * c * globals.psidot_ref * parms.t_step
