# Biped Locomotion

A learning project for classical locomotion implementations for bipedal robots using Rust.

## Overview

This project implements fundamental concepts in biped locomotion control:
- **Zero Moment Point (ZMP)** calculations for stability analysis
- **Inverse Kinematics (IK)** for leg positioning
- **Stability Controllers** including PD and Linear Inverted Pendulum Model (LIPM)
- Foundation for **Differential Dynamic Programming (DDP)** trajectory generation

## Project Structure

```
src/
├── main.rs         # Entry point
├── biped.rs        # Robot state and physical parameters
├── zmp.rs          # Zero Moment Point calculations
├── ik.rs           # Inverse kinematics solvers
└── stability.rs    # Stability controllers (PD, LIPM)
```

## Dependencies

### Core Libraries
- **nalgebra** (0.33) - Linear algebra and math operations
- **argmin** & **argmin-math** - Optimization libraries for DDP trajectory generation
- **serde** - Serialization for saving/loading robot states

### Optional (Commented Out)
- **mujoco-rs** - MuJoCo physics simulator bindings (requires MuJoCo installation)
- **k** - Kinematics library (has dependency issues, using custom IK for now)

## Building the Project

```bash
# Build the project
cargo build

# Run the project
cargo run

# Run tests
cargo test
```

## Key Concepts Implemented

### 1. Zero Moment Point (ZMP)

The ZMP is a fundamental stability criterion for bipedal locomotion. For a robot to be statically stable, the ZMP must remain inside the support polygon (the convex hull of contact points with the ground).

**Implementation:** `src/zmp.rs:16`

The ZMP is calculated using:
```
x_zmp = x_com - (z_com / g) * x_com_ddot
y_zmp = y_com - (z_com / g) * y_com_ddot
```

Where:
- `(x_com, y_com, z_com)` is the Center of Mass position
- `(x_com_ddot, y_com_ddot)` is the CoM acceleration in the ground plane
- `g` is gravity (9.81 m/s²)

### 2. Inverse Kinematics (IK)

IK solves for joint angles given a desired end-effector (foot) position.

**Implementation:** `src/ik.rs:14`

Currently implements:
- Analytical 2D IK for planar 2-link chains (hip-knee or knee-ankle)
- Based on law of cosines for exact solutions
- Reachability checking

### 3. Stability Controllers

#### PD Controller (`src/stability.rs:14`)
A simple Proportional-Derivative controller that generates CoM accelerations to keep the ZMP within the support polygon.

#### LIPM Controller (`src/stability.rs:74`)
The Linear Inverted Pendulum Model treats the robot as an inverted pendulum with constant CoM height. This simplification is widely used in bipedal locomotion and follows the dynamics:

```
x_ddot = ω² * (x - x_zmp)
```

Where `ω = sqrt(g/h)` is the natural frequency, and `h` is the constant CoM height.

## Next Steps

### Short Term
1. **Test the implementations** - Run unit tests and verify calculations
2. **Create example simulations** - Simple walking scenarios without full physics
3. **Integrate optimization** - Use argmin for DDP trajectory generation

### Medium Term
1. **Install MuJoCo** - Set up the physics simulator
   - Download from: https://mujoco.org/
   - Set `PKG_CONFIG_PATH` environment variable
   - Uncomment `mujoco-rs` in `Cargo.toml`
2. **Create URDF model** - Define a simple biped robot
3. **Integrate IK with full robot** - Replace simple 2D IK with full 3D solutions
4. **Implement gait patterns** - Walking, running trajectories

### Long Term
1. **DDP Trajectory Optimization** - Generate optimal trajectories between footsteps
2. **Adaptive controllers** - Handle disturbances and uneven terrain
3. **Model Predictive Control (MPC)** - Preview control for walking
4. **Learning-based enhancements** - Combine classical methods with learned components

## Learning Resources

### Papers & Theory
- **ZMP & Stability**
  - Vukobratović & Borovac (2004) - "Zero-Moment Point — Thirty Five Years of its Life"
  - Goswami (1999) - "Postural Stability of Biped Robots and the Foot-Rotation Indicator (FRI) Point"

- **LIPM**
  - Kajita et al. (2001) - "The 3D Linear Inverted Pendulum Mode: A simple modeling for a biped walking pattern generation"

- **DDP for Locomotion**
  - Tassa et al. (2012) - "Synthesis and stabilization of complex behaviors through online trajectory optimization"

### Rust Robotics Ecosystem (2026)
- **OpenRR** - Open Rust Robotics platform
- **nalgebra** - Foundation for all robotics math
- **ik-geo** - Fast analytical IK solver (alternative to `k`)
- **rusty_mujoco** - Alternative MuJoCo bindings

See the following for more information:
- [Rust Robotics Ecosystem](https://lib.rs/science/robotics)
- [OpenRR Project](https://github.com/openrr/openrr)
- [IK-Geo Implementation](https://github.com/rpiRobotics/ik-geo)

## Installing MuJoCo

MuJoCo is a physics simulator commonly used in robotics research. This project requires MuJoCo to be installed for full functionality.

### macOS (Recommended)

**Download and Install:**
```bash
# 1. Download MuJoCo from the official releases
# Visit: https://github.com/google-deepmind/mujoco/releases
# Download: mujoco-3.3.7-macos-universal2.dmg (or latest version)

# 2. Mount the DMG (double-click or):
hdiutil attach ~/Downloads/mujoco-3.3.7-macos-universal2.dmg

# 3. Copy the framework to your user frameworks directory
cp -r "/Volumes/MuJoCo/mujoco.framework" ~/Library/Frameworks/

# 4. Create a symlink for the dylib (required by mujoco-rs)
cd ~/Library/Frameworks/Versions/Current
ln -s libmujoco.3.3.7.dylib libmujoco.dylib

# 5. (Optional) Install the GUI app
cp -r "/Volumes/MuJoCo/MuJoCo.app" /Applications/

# 6. Unmount the DMG
hdiutil detach "/Volumes/MuJoCo"
```

**Update Configuration:**
The `.cargo/config.toml` file is already configured to point to `~/Library/Frameworks/`. If you installed elsewhere, update the `MUJOCO_DYNAMIC_LINK_DIR` path in that file.

**Verify Installation:**
```bash
cargo build  # Should compile without errors
```

### Linux

```bash
# 1. Download and extract MuJoCo
wget https://github.com/google-deepmind/mujoco/releases/download/3.3.7/mujoco-3.3.7-linux-x86_64.tar.gz
tar -xzf mujoco-3.3.7-linux-x86_64.tar.gz

# 2. Install to user directory (no sudo needed)
mkdir -p ~/.local/lib
mv mujoco-3.3.7/lib/* ~/.local/lib/

# 3. Update .cargo/config.toml
# Change MUJOCO_DYNAMIC_LINK_DIR to: "/home/yourusername/.local/lib"

# OR: Install system-wide (requires sudo)
sudo mv mujoco-3.3.7 /usr/local/
# Then use: MUJOCO_DYNAMIC_LINK_DIR = "/usr/local/mujoco-3.3.7/lib"
```

### Troubleshooting

**Build fails with "library not found":**
- Check that `libmujoco.dylib` exists in your configured directory
- Verify the path in `.cargo/config.toml` is correct
- Try: `ls -la $MUJOCO_DYNAMIC_LINK_DIR` to verify

**Runtime errors:**
```bash
# Check dynamic library loading
otool -L target/debug/biped_locomotion | grep mujoco  # macOS
ldd target/debug/biped_locomotion | grep mujoco       # Linux
```

## Contributing

This is a personal learning project. Feel free to fork and experiment!

## License

See LICENSE file.
