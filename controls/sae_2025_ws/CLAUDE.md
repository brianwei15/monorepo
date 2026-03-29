# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS 2 workspace for autonomous UAV control, targeting SAE/IARC competitions and swarm operations. It uses **ROS 2 Humble**, **Gazebo Harmonic** for simulation, and **PX4 Autopilot** as the flight controller.

## Build Commands

```bash
# Source ROS 2 environment first
source /opt/ros/humble/setup.bash
export GZ_VERSION=harmonic  # Required for ros_gz builds

# Install dependencies
rosdep install -r --from-paths src -i -y --rosdistro humble

# Build entire workspace
colcon build --symlink-install

# Build a specific package
colcon build --symlink-install --packages-select uav

# Source workspace after building
source install/setup.bash
```

## Running

```bash
# Launch UAV mission node (configure src/uav/launch/launch_params.yaml first)
ros2 launch uav main.launch.py

# Launch simulation (configure src/sim/launch/launch_params.yaml first)
ros2 launch sim sim.launch.py

# Launch integration dashboard
cd src/integration && ./launch.sh
# Backend: http://localhost:8080, Frontend: http://localhost:3000
```

## Tests and Linting

```bash
# Run all tests (linting/style only - no unit tests exist)
colcon test
colcon test --packages-select uav   # single package
colcon test-result --verbose

# Ruff linting (excludes submodules)
ruff check src/
ruff format src/
```

Tests are style checks only (flake8, pep257, copyright headers). No functional unit tests exist.

## Architecture

### Node Architecture

**ModeManager** (`src/uav/uav/ModeManager.py`) is the main ROS 2 node. It:
1. Instantiates a UAV object (VTOL or Multicopter based on airframe)
2. Loads a mission YAML from `src/uav/uav/missions/`
3. Runs a state machine, advancing through `Mode` instances
4. Calls vision service nodes as needed during modes

**UAV base class** (`src/uav/uav/UAV.py`) manages all PX4 communication via `px4_msgs` topics:
- Publishers: `OffboardControlMode`, `TrajectorySetpoint`, `VehicleCommand`
- Subscribers: `VehicleStatus`, `VehicleAttitude`, GPS topics

### Mission System

Missions are YAML files in `src/uav/uav/missions/`. Each defines a sequence of mode names with parameters. Mode parameters are Python literal strings (e.g., `coordinates: '[((5, 0, -5), 1, "LOCAL"),]'`). Modes are dynamically imported from `src/uav/uav/autonomous_modes/`.

Each `Mode` subclass implements `on_enter()`, `on_exit()`, and an `update()` loop called at ~20Hz. Transitions are defined in the mission YAML.

### Vision Pipeline

Vision nodes (`src/uav/uav/vision_nodes/`) are separate ROS 2 nodes that expose services (defined in `src/uav_interfaces/`). Modes call these services to get tracking results. Computer vision algorithms live in `src/uav/uav/cv/` using OpenCV.

### Simulation

The `sim` package generates Gazebo SDF world files programmatically. `sim.launch.py` selects a world generator based on the `competition` parameter in `launch_params.yaml` (0=in_house, 1=iarc, 2=custom, 3=sae, 4=swarm).

### Integration Dashboard

FastAPI backend (`src/integration/backend/`) + React frontend (`src/integration/frontend/`). The backend subscribes to ROS 2 camera topics and streams MJPEG over HTTP. It also manages SSH connections to Raspberry Pis for deployment.

### Key Configuration Files

| File | Purpose |
|------|---------|
| `src/uav/launch/launch_params.yaml` | Airframe, mission name, camera, IDs, sim mode |
| `src/sim/launch/launch_params.yaml` | Competition type, scoring toggle |
| `.ruff.toml` | Ruff linting config (excludes submodule dirs) |

### Submodules

`src/px4_msgs/`, `src/actuator_msgs/`, and `src/ros_gz/` are git submodules — don't edit them directly.
