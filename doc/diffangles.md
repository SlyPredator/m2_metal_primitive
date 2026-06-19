Ran command: `python main.py`
Viewed main.py:23-63
Viewed globals.py:1-81

To set different angles for each leg, you can define the joint angles for each leg individually and then concatenate them. 

The joint order in MuJoCo's `data.qpos` is grouped by leg in the following order: **FL (Front Left), FR (Front Right), RL (Rear Left), and RR (Rear Right)**. Each leg has 3 joints in the order: **[hip (abduction), pitch (thigh), knee (calf)]**.

Here are the two cleanest ways to do this in `main.py`:

### Option 1: Using a 2D NumPy array (Recommended)
This approach is clean and lets you easily see and modify each leg's angles:

```python
# Set initial joint positions in MuJoCo
pos   = np.array([0.0, 0.0, 0.32])     # trunk starts 0.32 m above ground
quat  = np.array([1.0, 0.0, 0.0, 0.0]) # level orientation

# Define [hip, pitch, knee] for each leg in order: FL, FR, RL, RR
q_legs = np.array([
    [0.0,  0.7, -1.4],  # Front Left (FL)
    [0.1,  0.6, -1.2],  # Front Right (FR)
    [-0.1, 0.8, -1.5],  # Rear Left (RL)
    [0.0,  0.7, -1.4]   # Rear Right (RR)
])

# Use .flatten() to convert the 4x3 array to a 12-element 1D array
data.qpos[:] = np.concatenate((pos, quat, q_legs.flatten()))
mj.mj_forward(model, data)
```

---

### Option 2: Defining separate leg arrays
If you prefer explicit variable names for each leg, you can define them as individual 1D arrays:

```python
# Set initial joint positions in MuJoCo
pos   = np.array([0.0, 0.0, 0.32])     # trunk starts 0.32 m above ground
quat  = np.array([1.0, 0.0, 0.0, 0.0]) # level orientation

# Define joint configurations individually
q_fl = np.array([0.0,  0.7, -1.4]) # FL joints
q_fr = np.array([0.1,  0.6, -1.2]) # FR joints
q_rl = np.array([-0.1, 0.8, -1.5]) # RL joints
q_rr = np.array([0.0,  0.7, -1.4]) # RR joints

# Concatenate all of them together
data.qpos[:] = np.concatenate((pos, quat, q_fl, q_fr, q_rl, q_rr))
mj.mj_forward(model, data)
```