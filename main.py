import os
import sys
import time
from collections import deque

# Ensure the src directory is in python path
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'src'))

import mujoco as mj
import mujoco.viewer
import numpy as np
import pandas as pd
from common import globals
from common.parameters import parms
from common.plotter import save_and_show_plots
from common.robot_data import robot
from control.high_level_controller import high_level_control
from control.joint_controller import joint_control
from control.state_machine import state_machine
from control.trajectory_generator import cartesian_traj, joint_traj

# Resolve XML path dynamically relative to this script
XML_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), 'robot_description/xml/m2_metal.xml'))
print(f'Loading model from: {XML_PATH}')

model = mj.MjModel.from_xml_path(XML_PATH)
data  = mj.MjData(model)

# Initialize foot trails for visualization
foot_geom_ids = [mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, name) for name in robot.params.foot_geom_names]
foot_trails = [deque(maxlen=parms.trail_length) for _ in range(4)]
step_counter = 0

print(f'lz0 = {parms.lz0:.6f}')

# Initialise global state
globals.init()

def _run_planners_and_trajectories(sim_time):
    """Layer 1 & 2: sync time, run gait planner and trajectory generators."""
    globals.time = sim_time
    state_machine()    # update FSM; set Raibert foot arc endpoints
    cartesian_traj()   # smooth foot references [lx,ly,lz] via quintic polynomial
    joint_traj()       # convert foot references to joint angles (IK) and velocities (J^-1)


def _read_robot_state(data):
    """Read current robot state from MuJoCo."""
    globals.q_act            = data.qpos[7:].copy()   # 12 leg joint angles
    globals.u_act            = data.qvel[6:].copy()   # 12 leg joint velocities
    globals.pos_quat_trunk   = data.qpos[:7].copy()   # trunk [x,y,z, qw,qx,qy,qz]
    globals.vel_angvel_trunk = data.qvel[:6].copy()   # trunk [xdot,ydot,zdot, wx,wy,wz]


def _run_controllers():
    """Layer 3: compute joint torques and update velocity commands."""
    joint_control()      # PD + (-J^T F) feedforward -> stores torques in globals.trq
    high_level_control(vx_des=0.8, vy_des=0.0, omega_des=0.0) # once per gait cycle: ramp xdot_ref toward target speed


def _step_simulation(model, data):
    """Send control and integrate dynamics one timestep."""
    data.ctrl[:] = globals.trq
    mj.mj_step(model, data)   # integrate dynamics one timestep


def _record_foot_trails(data, foot_geom_ids, foot_trails):
    """Record foot positions for trajectory trails."""
    for i, fid in enumerate(foot_geom_ids):
        if fid >= 0:
            foot_trails[i].append(data.geom_xpos[fid].copy())


def _log_state(data, log):
    """Log state for plotting."""
    log.append({
        'time'      : data.time,
        'x'         : data.qpos[0], 'y': data.qpos[1], 'z': data.qpos[2],
        'vx'        : data.qvel[0], 'vy': data.qvel[1], 'vz': data.qvel[2],
        'fsm0'      : int(globals.fsm[0]), 'fsm1': int(globals.fsm[1]),
        'fsm2'      : int(globals.fsm[2]), 'fsm3': int(globals.fsm[3]),
        **{f'trq{k}': float(globals.trq[k]) for k in range(12)},
        'xdot_ref'  : float(globals.xdot_ref),
        'lx_ref_0'  : float(globals.lx_ref[0]),
        'lz_ref_0'  : float(globals.lz_ref[0]),
    })


def _render_trails(viewer, foot_trails):
    """Sync viewer and draw foot trails."""
    viewer.user_scn.ngeom = 0  # clear previous user geoms
    for i, trail in enumerate(foot_trails):
        if len(trail) < 2:
            continue
        color = np.array(parms.foot_colors[i])
        for j in range(len(trail) - 1):
            if viewer.user_scn.ngeom >= viewer.user_scn.maxgeom:
                break
            geom = viewer.user_scn.geoms[viewer.user_scn.ngeom]
            mj.mjv_initGeom(
                geom,
                type=mj.mjtGeom.mjGEOM_LINE,
                size=np.zeros(3),
                pos=np.zeros(3),
                mat=np.zeros(9),
                rgba=color
            )
            mj.mjv_connector(
                geom,
                mj.mjtGeom.mjGEOM_LINE,
                2,
                trail[j],
                trail[j+1]
            )
            viewer.user_scn.ngeom += 1
    viewer.sync()


def _pace_realtime(sim_time, sim_start_time, wall_start_time, current_wall_time):
    """Real-time control rate limiter (accumulated wall-clock sync)."""
    sim_elapsed = sim_time - sim_start_time
    wall_elapsed = current_wall_time - wall_start_time
    time_to_sleep = sim_elapsed - wall_elapsed
    if time_to_sleep > 0.002:
        time.sleep(time_to_sleep)


# Set initial joint positions in MuJoCo using robot nominal configs
pos   = robot.params.q_base[:3]
quat  = robot.params.q_base[3:]
qleg  = robot.params.q_leg_nominal
data.qpos[:] = np.concatenate((pos, quat, qleg, qleg, qleg, qleg))
mj.mj_forward(model, data)                  # propagate initial state through MuJoCo

log    = []
print(f'Simulating {parms.sim_duration} s ...')

# Launch passive viewer for real-time visualization
with mj.viewer.launch_passive(model, data) as viewer:
    sim_start_time = data.time
    wall_start_time = time.time()
    last_viewer_sync = 0.0
    while data.time < parms.sim_duration and viewer.is_running():
        # Layer 1 & 2: sync time, run gait planner and trajectory generators
        _run_planners_and_trajectories(data.time)
        
        # Read current robot state from MuJoCo
        _read_robot_state(data)
        
        # Layer 3: compute joint torques and update velocity commands
        _run_controllers()
        
        # Send control to MuJoCo
        _step_simulation(model, data)
        step_counter += 1
        
        # Record foot positions for trajectory trails
        if step_counter % 10 == 0:
            _record_foot_trails(data, foot_geom_ids, foot_trails)
                    
        # Log state for plotting
        _log_state(data, log)
        
        # Sync viewer at ~60 Hz
        current_wall_time = time.time()
        if current_wall_time - last_viewer_sync >= 1.0 / 60.0:
            _render_trails(viewer, foot_trails)
            last_viewer_sync = current_wall_time
            
        # Real-time control rate limiter (accumulated wall-clock sync)
        _pace_realtime(data.time, sim_start_time, wall_start_time, current_wall_time)

df = pd.DataFrame(log)
print(f'Done. {len(df)} timesteps logged.')

# Save plots to a folder named 'plots' in the project root directory
project_root = os.path.dirname(os.path.abspath(__file__))
plots_dir = os.path.join(project_root, './plots')
# save_and_show_plots(df, plots_dir, show=False)

