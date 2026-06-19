import os

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D


def save_and_show_plots(df, output_dir, show=True):
    os.makedirs(output_dir, exist_ok=True)
    
    # Plot 1 — Trunk Position vs. Time
    fig, ax = plt.subplots(figsize=(10, 4))
    for col, label in [('x', 'X'), ('y', 'Y'), ('z', 'Z')]:
        ax.plot(df['time'], df[col], label=f'trunk {label}')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position (m)')
    ax.set_title('Trunk x, y, z position')
    ax.legend()
    ax.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'trunk_position.png'))
    if show:
        plt.show()
    else:
        plt.close()

    # Plot 2 — Trunk Forward Velocity vs. Time
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(df['time'], df['vx'], label='vx actual', color='tab:blue')
    ax.plot(df['time'], df['xdot_ref'], '--', label='xdot_ref', color='tab:orange')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Velocity (m/s)')
    ax.set_title('Trunk forward velocity')
    ax.legend()
    ax.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'trunk_velocity.png'))
    if show:
        plt.show()
    else:
        plt.close()

    # Plot 3: FSM State per Leg vs. Time
    FSM_COLORS = {1: 'gray', 2: 'steelblue', 3: 'darkorange'}
    LEG_NAMES  = ['FR (0)', 'FL (1)', 'RR (2)', 'RL (3)']
    fig, axes = plt.subplots(4, 1, figsize=(10, 6), sharex=True)
    for idx, (ax, name) in enumerate(zip(axes, LEG_NAMES)):
        col = f'fsm{idx}'
        t   = df['time'].values
        fsm = df[col].values
        changes = np.where(fsm[:-1] != fsm[1:])[0]
        indices = np.concatenate(([0], changes + 1, [len(t) - 1]))
        for i in range(len(indices) - 1):
            idx_start = indices[i]
            idx_end = indices[i+1]
            ax.fill_between([t[idx_start], t[idx_end]], idx+0.1, idx+0.9,
                            color=FSM_COLORS.get(fsm[idx_start], 'white'), alpha=0.85)
        ax.set_xlim(t[0], t[-1])
        ax.set_ylim(idx, idx+1)
        ax.set_yticks([])
        ax.set_ylabel(name, fontsize=8)
        ax.grid(axis='x', linestyle='--', alpha=0.4)
    axes[-1].set_xlabel('Time (s)')
    fig.suptitle('FSM state per leg (gray=stand blue=stance orange=swing)')
    legend_els = [Line2D([0], [0], color='gray', lw=6, label='stand'),
                  Line2D([0], [0], color='steelblue', lw=6, label='stance'),
                  Line2D([0], [0], color='darkorange', lw=6, label='swing')]
    fig.legend(handles=legend_els, loc='upper right', fontsize=8)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'leg_fsm.png'))
    if show:
        plt.show()
    else:
        plt.close()

    # Plot 4: Joint Torques vs. Time (2x2 layout by leg)
    LEG_LABELS  = ['FR', 'FL', 'RR', 'RL']
    JOINT_LABELS= ['abduction', 'hip', 'knee']
    COLORS      = ['tab:blue', 'tab:orange', 'tab:green']
    fig, axes = plt.subplots(2, 2, figsize=(12, 6), sharex=True)
    for leg_idx, (ax, label) in enumerate(zip(axes.flat, LEG_LABELS)):
        for j_idx, (jlabel, col) in enumerate(zip(JOINT_LABELS, COLORS)):
            trq_col = f'trq{3*leg_idx+j_idx}'
            ax.plot(df['time'], df[trq_col], color=col, label=jlabel, lw=0.8)
        ax.set_title(f'Leg {label}')
        ax.set_ylabel('Torque (N·m)')
        ax.legend(fontsize=7)
        ax.grid(True)
    axes[1,0].set_xlabel('Time (s)')
    axes[1,1].set_xlabel('Time (s)')
    fig.suptitle('Joint torques — 2×2 layout by leg')
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'joint_torques.png'))
    if show:
        plt.show()
    else:
        plt.close()

    print(f"Plots saved to {output_dir}")
