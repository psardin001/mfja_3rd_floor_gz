# MFJA 3rd Floor Gazebo — Room 315 Kinematic Shuttle

This repository contains the Gazebo Harmonic / ROS 2 Jazzy simulation assets for the MFJA 3rd floor, with the focus on the Room 315 flexible rail system.

The current project state is a **kinematic-first shuttle simulation**. Shuttles move along arc-length paths generated from calibrated CSV rail geometry and an explicit rail graph, updating Gazebo model poses via the `/world/<world_name>/set_pose` bridge.

---

## 📖 Quick Links & Documentation

To keep this main README concise, detailed guides have been moved to dedicated files:

*   **[Detailed Feature & API Guide (DETAILED_GUIDE.md)](DETAILED_GUIDE.md)**:
    *   Step-by-step feature guides (ON/OFF control, switches, stoppers, sensors)
    *   Typed interfaces, ROS 2 topics, and service details
    *   Multi-shuttle setup, collision avoidance, and troubleshooting
*   **[Room 315 Kinematic Rail Network Specs](mfja_robot_control_config/config/room_315_kinematics/README.md)**: Low-level details about segment directions, device YAMLs, and sensor cookbook testing.
*   **[HTML Runbook](runbook.html)**: A focused visualization and operational guide.

---

## 📂 Repository Layout

This git repository is organized as a **meta-repository**:

*   `mfja_3rd_floor_description/`: Gazebo worlds, models, meshes, and URDF/SDF assets.
*   `mfja_rail_interfaces/`: Custom ROS 2 interfaces for commands, states, and sensors.
*   `mfja_robot_control_config/`: Shuttle/switch scripts, bridge configurations, and rail kinematic settings.
*   `mfja_3rd_floor_bringup/`: Centralized launch entry points for Room 315, full floor, and isolated robot configurations.

---

## ⚡ Quick Start

### 1. Build the Workspace

From your colcon workspace root:

```bash
# Install dependencies
rosdep install --from-paths src/mfja_3rd_floor_gz -y --ignore-src --rosdistro jazzy

# Build
colcon build --symlink-install --base-paths src/mfja_3rd_floor_gz

# Source
source install/setup.bash
```

### 2. Launch the Simulation (Room 315 Only)

Start Room 315 with one right-rail shuttle and one left-rail shuttle waiting for commands:

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

### 3. Basic Shuttle Control

Turn on the right shuttle:

```bash
ros2 topic pub --once /room_315/rails/right/shuttles/command \
  mfja_rail_interfaces/msg/ShuttleCommand \
  "{name: 'room315_right_shuttle_1', command: 'ON'}"
```

Control switches on the right rail:

```bash
ros2 topic pub --once /room_315/rails/right/switches/command \
  mfja_rail_interfaces/msg/SwitchCommand \
  "{switches: [{name: 'ALL', state: 'EXTERIOR'}]}"
```

Control stoppers on the right rail:

```bash
ros2 topic pub --once /room_315/rails/right/stoppers/command \
  mfja_rail_interfaces/msg/StopperCommand \
  "{stoppers: [{name: 'A1', state: '1'}]}"
```

---

*For advanced scenarios, dataset collection/recording, Nix environments, collision parameters, and troubleshooting, please refer to **[DETAILED_GUIDE.md](DETAILED_GUIDE.md)**.*
