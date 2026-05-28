# MFJA 3rd Floor Gazebo — Room 315 Kinematic Shuttle

This repository contains the Gazebo Harmonic / ROS 2 Jazzy simulation assets for the MFJA 3rd floor, focusing on the **Room 315 flexible rail system**.

## 📖 General Overview

The current project state is a **kinematic-first shuttle simulation**. Instead of relying on complex contact dynamics and physics (like wheel friction), the shuttles move kinematically. They follow arc-length paths generated from a calibrated CSV rail geometry and an explicit rail graph. The system updates the Gazebo model poses directly via the `/world/<world_name>/set_pose` bridge.

This allows for reliable, highly controllable testing of routing logic, switch control, multiple shuttles, and station interactions without physical simulation instability.

---

## 🛠️ Installation Guide

The project requires **Ubuntu 24.04** and **ROS 2 Jazzy**. The repository acts as a meta-repository and must be built inside a colcon workspace.

### 1. Install Prerequisites
Make sure ROS 2 Jazzy is installed, along with essential build tools:
```bash
sudo apt update
sudo apt install -y build-essential cmake git ninja-build pkg-config \
  python3-colcon-common-extensions python3-rosdep python3-yaml \
  ros-jazzy-desktop ros-jazzy-robot-state-publisher ros-jazzy-ros-gz

# Initialize rosdep if you haven't already
sudo rosdep init || true
rosdep update
```

### 2. Clone the Repository
Create a workspace and clone this meta-repository inside its `src/` folder:
```bash
export MFJA_WS=~/mfja_ws
mkdir -p "$MFJA_WS/src"
cd "$MFJA_WS/src"
git clone https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz.git
```

### 3. Build and Source
Install ROS dependencies, build the workspace, and source it:
```bash
cd "$MFJA_WS"
source /opt/ros/jazzy/setup.bash

# Install dependencies defined in package.xml files
rosdep install --from-paths src/mfja_3rd_floor_gz -y --ignore-src --rosdistro jazzy

# Build the workspace
colcon build --symlink-install --base-paths src/mfja_3rd_floor_gz

# Source the installed environment
source install/setup.bash
```
*(Note: You must run `source install/setup.bash` in every new terminal you open.)*

---

## ⚡ Basic Commands & Quick Start

Here are the essential commands you need to launch and interact with the simulation.

### 1. Launching the Simulation
You can launch just Room 315. The following command launches Gazebo with the rail system, UI markers, and prepares 1 shuttle on each rail (waiting for the `ON` command):
```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true \
  enable_room315_kinematic_shuttles:=true \
  room315_right_shuttle_count:=1 \
  room315_left_shuttle_count:=1 \
  room315_shuttles_start_enabled:=false
```

### 2. Controlling Shuttles (ON/OFF/RESET)
Turn on the right-rail shuttle so it starts moving along its path:
```bash
ros2 topic pub --once /room_315/rails/right/shuttles/command \
  mfja_rail_interfaces/msg/ShuttleCommand \
  "{name: 'room315_right_shuttle_1', command: 'ON'}"
```
*You can also send `OFF` to pause it, or `RESET` to return it to its start slot.*

### 3. Controlling Switches
Switches define the route. You can set them to `INTERIOR` or `EXTERIOR`. To switch all right-rail switches to the exterior branch:
```bash
ros2 topic pub --once /room_315/rails/right/switches/command \
  mfja_rail_interfaces/msg/SwitchCommand \
  "{switches: [{name: 'ALL', state: 'EXTERIOR'}]}"
```

### 4. Controlling Stoppers
Stoppers block a shuttle from entering a switch. `1` means STOP/CLOSED, `0` means PASS/OPEN:
```bash
ros2 topic pub --once /room_315/rails/right/stoppers/command \
  mfja_rail_interfaces/msg/StopperCommand \
  "{stoppers: [{name: 'A1', state: '1'}]}"
```

---

## 📂 Repository Layout

*   `mfja_3rd_floor_description/`: Gazebo worlds, models, meshes, and URDF/SDF assets.
*   `mfja_rail_interfaces/`: Custom ROS 2 interfaces for commands, states, and sensors.
*   `mfja_robot_control_config/`: Shuttle/switch scripts, bridge configurations, and rail kinematic settings.
*   `mfja_3rd_floor_bringup/`: Centralized launch entry points for Room 315, full floor, and isolated robot configurations.

---

## 📚 Detailed Documentation

For a deep dive into advanced features, please refer to our dedicated documentation files:

*   **[Detailed Feature & API Guide (DETAILED_GUIDE.md)](DETAILED_GUIDE.md)**: Includes step-by-step guides for adding shuttles dynamically, reading sensor feedback, collision avoidance, testing industrial robots, and troubleshooting.
*   **[Room 315 Kinematic Rail Network Specs](mfja_robot_control_config/config/room_315_kinematics/README.md)**: Technical details about segment directions, device YAMLs, and sensor cookbook testing.
*   **[HTML Runbook](runbook.html)**: A focused visualization and operational guide.
