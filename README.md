# M2 Quadruped Robot Simulator & Web Deployment

This repository contains the simulation, control, and web deployment workspace for the **M2 Quadruped Robot** ("M2 Metal"). It features both a high-fidelity Python simulator powered by **MuJoCo** and a lightweight, fast-compiling WebAssembly (WASM) / native desktop target using **Raylib** with a custom C physics solver.

---

## Workspace Structure

```text
├──                   # Main project directory
│   ├── main.py                # Main entry point for the MuJoCo Python simulation
│   ├── src/                   # Python modules for control, planning, and kinematics
│   │   ├── common/            # Parameters, globals, logging, and plotting
│   │   ├── control/           # FSM state machine, joint control, high level control
│   │   └── kinematics/        # Analytical inverse/forward kinematics solver
│   ├── robot_description/     # Robot model description
│   │   ├── xml/               # m2_metal.xml (MuJoCo XML definition)
│   │   └── meshes/            # STL/OBJ meshes of the physical links
│   ├── nb/                    # Interactive Jupyter notebooks for trot experiments
│   ├── doc/                   # Controls guidelines and math documentation
│   ├── plots/                 # Simulation analysis plots (torques, trajectories, FSM states)
│   │
│   └── m2_web_dep/            # WebAssembly & Native C deployment project
│       ├── web/               # Raylib visual simulator and UI source (m2_demo.c, m2_host.c)
│       ├── assets/            # Fonts and static assets for the web player
│       ├── tools/             # Python tools to sync MuJoCo XML with the C simulation
│       ├── vendor/            # Precompiled Raylib static libraries (Linux & WASM)
│       └── build/             # Output directory for compiled binaries and HTML
├── .gitignore                 # Root level gitignore
└── README.md                  # This file
```

---

## 1. Python Simulation (MuJoCo)

The Python simulation runs the robot using the MuJoCo physics engine, executing the high-level gait planners and joint controllers, and plotting the logged telemetry on exit.

### Prerequisites
Install the required packages in your Python environment:
```bash
pip install mujoco numpy pandas matplotlib
```

### Running the Simulator
Run the main script from the root directory:
```bash
python3 main.py
```
* **Controls**: Click and drag to rotate the camera.
* **Foot Trails**: Red, Green, Blue, and Orange trails track the foot positions in real-time.
* **Logging & Plotting**: When you close the viewer window, telemetry charts are saved under `plots/`.

---

## 2. WebAssembly & Native Desktop Simulator (Raylib)

The web deployment folder `m2_web_dep/` contains a lightweight visual simulator written in C using Raylib. It is optimized for web browser playback and can also be run as a standalone desktop application.

### Prerequisites

* **Native Desktop Build**: A standard C compiler (like `gcc`) and OpenGL development libraries (X11, GL, math).
* **Web/WASM Build**: Emscripten SDK (`emsdk`). Ensure `emcc` is in your shell path:
  ```bash
  source /path/to/emsdk/emsdk_env.sh
  ```

### Build Instructions

Navigate to the deployment directory or compile directly from the root workspace using the centralized build script:

#### A. Build and Run Natively (Linux Desktop)
To compile a native desktop executable:
```bash
./m2_web_dep/web/build_m2_demo.sh
```
This builds and outputs a binary to `m2_web_dep/build/m2demo`. Run it via:
```bash
./m2_web_dep/build/m2demo
```

#### B. Build for Web (WebAssembly)
To compile to HTML5/WASM:
```bash
./m2_web_dep/web/build_m2_demo.sh --web
```
This outputs compiled assets into `m2_web_dep/build/web/index.html`.

To run the web version locally, serve the directory using a web server:
```bash
python3 -m http.server -d m2_web_dep/build/web 8080
```
Then visit `http://localhost:8080` in your web browser.

---

## Asset Pipeline & Synchronization

If you modify the physical parameters, joint limits, or 3D meshes in `robot_description/xml/m2_metal.xml` (or `m2_web_dep/assets/m2_metal.xml`), you must synchronize those changes with the C simulator using the helper scripts under `tools/`:

1. **Synchronize Model Constants (`m2_model_const.h`)**:
   ```bash
   python3 m2_web_dep/tools/dump_m2_model_const.py
   ```
   This generates the C header containing the physics parameters needed by the C solver.

2. **Synchronize 3D Meshes (`m2_meshes.bin`)**:
   ```bash
   python3 m2_web_dep/tools/dump_m2_meshes.py
   ```
   This packs the updated 3D link meshes into a compressed binary format readable by the Raylib renderer.
