class parameters:
    def __init__(self):
        self.fsm_stand = 1
        self.fsm_stance = 2
        self.fsm_swing = 3
        self.t_stand = 0.1
        self.t_step = 0.15
        self.lz0 = -0.32
        self.hcl = 0.075
        self.mass = 10.057
        self.gravity = 9.81
        self.vx_min = -2.0
        self.vx_max = 2.0
        self.dvx = 0.1
        self.vy_min = -1.0
        self.vy_max = 1.0
        self.dvy = 0.05
        self.omega_min = -2.0
        self.omega_max = 2.0
        self.domega = 0.1
        self.sim_duration = 10.0
        self.trail_length = 100
        self.foot_colors = [
            [1.0, 0.1, 0.1, 1.0],  # FL: Red
            [0.1, 1.0, 0.1, 1.0],  # FR: Green
            [0.1, 0.5, 1.0, 1.0],  # RL: Blue
            [1.0, 0.6, 0.1, 1.0],  # RR: Orange
        ]

parms = parameters()
