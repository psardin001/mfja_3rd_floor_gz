# mfja_staubli_demos

Straight Cartesian `tool0` lines for the Room 315 Staubli TX2-60L, planned by
HPP (Humanoid Path Planner) and executed in the MFJA Gazebo simulation.

Every run plans fresh: the current robot configuration is read from
`/staubli1/joint_states`, the goal is projected with HPP constraints, and HPP
computes a constrained, continuously collision-checked path against the Room
315 cell meshes, including the glass. Nothing is precomputed; if a motion would
touch the cell, planning fails with a collision report instead of moving the
robot.

## Layout

```
mfja_staubli_demos/
├── hpp/
│   ├── room315_hpp_line.py    # planner and executor
│   ├── staubli_tx2_60l.srdf   # adjacent-link self-collision disables
│   ├── room315_cell.urdf      # six Room 315 cell meshes as HPP obstacles
│   └── room315_cell.srdf
├── launch/
│   └── room_315_staubli_cartesian_demo.launch.py
└── scripts/
    ├── room315_demo.sh        # start the simulation
    ├── room315_hpp_line.sh    # run the planner in the hpp-exec container
    └── room315_env.sh         # source ROS + the MFJA workspace cleanly
```

## Requirements

Host side:

- Ubuntu/ROS 2 environment compatible with ROS 2 Jazzy
- Gazebo `gz sim` 8
- `colcon`, `git`, and the ROS/Gazebo dependencies used by this repository
- Docker, if you use the recommended `hpp-exec` container path

Planning side:

- `hpp-exec` with `pyhpp`, `rclpy`, and the
  `hpp_exec.read_current_configuration` helper.

## Install MFJA

Clone the MFJA repository into a colcon workspace and build it:

```bash
mkdir -p ~/mfja_ws/src
cd ~/mfja_ws/src
git clone https://github.com/psardin001/mfja_3rd_floor_gz.git
cd ~/mfja_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

The scripts are meant to be run from a source checkout, not only from the
installed package. The HPP planner resolves the robot, SRDF, and cell meshes
through `package://` URLs, and the Docker wrapper mounts the source checkout
inside the container.

If your workspace is not `~/mfja_ws`, export one of these before running the
scripts:

```bash
export MFJA_WS=/path/to/mfja_ws
# or
export MFJA_SETUP=/path/to/mfja_ws/install/setup.bash
```

## Install HPP

The recommended path is the `hpp-exec` Docker container. MFJA and Gazebo run on
the host; HPP planning runs in this container.

Clone `hpp-exec` from the HPP organization and start the container once:

```bash
git clone -b devel https://github.com/humanoid-path-planner/hpp-exec.git ~/hpp-exec
cd ~/hpp-exec
./run.sh
```

Inside the container, build the HPP Python stack the first time:

```bash
cd ~/devel/src
make hpp-python.install
exit
```

`make all` also builds `hpp-gepetto-viewer`; it is useful for HPP visualization
work, but the Staubli Gazebo demo does not need it.

Tell the MFJA wrapper where your `hpp-exec` checkout lives:

```bash
export HPP_EXEC_DIR=~/hpp-exec
```

For a persistent machine-local setting, create
`mfja_staubli_demos/scripts/room315_local_env.sh` with the same export. That
file is ignored by Git.

If an `hpp-exec` container is already running, stop it before using this demo
unless it was started by `mfja_staubli_demos/scripts/room315_hpp_line.sh`; the
wrapper must mount the MFJA source checkout into the container:

```bash
docker rm -f hpp-exec
```

## First Run

Open a terminal in the MFJA source checkout:

```bash
cd ~/mfja_ws/src/mfja_3rd_floor_gz
export MFJA_WS=~/mfja_ws
export HPP_EXEC_DIR=~/hpp-exec
```

Run a planning-only smoke test before starting Gazebo:

```bash
mfja_staubli_demos/scripts/room315_hpp_line.sh --plan-only
```

Expected output includes a line like:

```text
max straight-line deviation: 0.0000xx m
```

Start the simulation:

```bash
mfja_staubli_demos/scripts/room315_demo.sh
```

In another terminal, from the same source checkout, move the robot from the
spawn pose to the working pose:

```bash
export MFJA_WS=~/mfja_ws
export HPP_EXEC_DIR=~/hpp-exec
mfja_staubli_demos/scripts/room315_hpp_line.sh --goto-start
```

Run the default line: straight up from one quarter to three quarters of the
robot height, 0.5 m in front of the base, inside the glass:

```bash
mfja_staubli_demos/scripts/room315_hpp_line.sh
```

Move back down:

```bash
mfja_staubli_demos/scripts/room315_hpp_line.sh --line 0 0 -0.6475
```

Try another line in the Staubli base frame, in meters:

```bash
mfja_staubli_demos/scripts/room315_hpp_line.sh --line 0 0.2 0 --duration 8
```

The line starts at the current tool position. After a line up, run the opposite
line or `--goto-start` before asking for another line that is not reachable from
the current pose.

## Local HPP Instead Of Docker

The Docker wrapper is only a convenience. A local HPP environment works if this
command succeeds:

```bash
python3 -c 'import pyhpp, hpp_exec, rclpy'
```

Source ROS, the MFJA workspace, and your HPP environment, then run the planner
directly:

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/mfja_ws/install/setup.bash
# source /path/to/hpp/env.sh, or otherwise set PYTHONPATH and LD_LIBRARY_PATH
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-7}
export ROS_PACKAGE_PATH=/path/to/mfja_3rd_floor_gz${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}
python3 /path/to/mfja_3rd_floor_gz/mfja_staubli_demos/hpp/room315_hpp_line.py --plan-only
```

For live execution, use the same environment while the simulation is running.
The process must see the same `ROS_DOMAIN_ID` as Gazebo.

## Options

| Option | Default | Meaning |
|---|---|---|
| `--line DX DY DZ` | `0 0 0.6475` | Line displacement in the Staubli base frame, meters. |
| `--duration` | `5.0` | Time to traverse the line, seconds. |
| `--samples` | `80` | Trajectory waypoints along the line. |
| `--q-start j1..j6` | working pose | Start configuration for `--plan-only` and target of `--goto-start`. |
| `--goto-start` | off | Collision-checked joint motion to `--q-start` instead of a line. Also recovers from collided states. |
| `--plan-only` | off | Plan and check only, no ROS publishing. |
| `--robot-name` | `staubli1` | Topic namespace. |

## How It Works

```
mfja_staubli_demos/scripts/room315_demo.sh
        |
room_315_staubli_cartesian_demo
.launch.py: gz server, bridges,
robot spawn, GUI as separate
process
        |
   ros_gz bridge
        |
gz JointTrajectory plugin moves the robot

mfja_staubli_demos/scripts/room315_hpp_line.sh
        |
    hpp-exec container (pyhpp + rclpy,
    host network, ROS_DOMAIN_ID=7,
    /dev/shm shared for Fast DDS)
        |
mfja_staubli_demos/hpp/room315_hpp_line.py
```

Each invocation of `room315_hpp_line.py`:

1. Builds the HPP problem: the Staubli URDF and `room315_cell.urdf` are loaded
   into one HPP `Device`. The cell meshes are at their world poses and are
   placed with the inverse of the robot world pose so everything lives in the
   robot base frame.
2. Adds `CollisionValidation` and `JointBoundValidation`, uses `Straight` as
   the steering method, and uses `Dichotomy` for continuous path validation.
3. Reads the live configuration from `/staubli1/joint_states`. This is the
   start of the plan, so the controller does not receive a step input. The
   script refuses to run if several simulations publish on the topic.
4. Projects the goal with an HPP `ConfigProjector` at `start + line`, keeping
   the start orientation, and validates it.
5. Installs a line `ConfigProjector` with `problem.setConstraints`: the
   `Position` constraint frees only the along-line axis and the `Orientation`
   constraint locks the start orientation. `directPath` then projects every
   interpolated configuration onto the line while `Dichotomy` checks the swept
   robot body against the cell.
6. Publishes once on `/staubli1/joint_trajectory`. The first point is held for
   one second so the controller settles.

`--goto-start` plans the same way but without the line constraints: a
collision-checked joint-space `directPath` to the working pose. If the current
configuration is already in collision, it retreats slowly without validation;
that is only meant to recover from a bad state.

## Troubleshooting

- `HPP_EXEC_DIR is not set`: export `HPP_EXEC_DIR=/path/to/hpp-exec` or create
  `mfja_staubli_demos/scripts/room315_local_env.sh`.
- `hpp-exec run.sh not found`: check that `HPP_EXEC_DIR` points to an
  `hpp-exec` checkout with `run.sh`.
- `MFJA workspace setup not found`: set `MFJA_WS` or `MFJA_SETUP`.
- `No module named pyhpp`: build HPP inside the `hpp-exec` container with
  `cd ~/devel/src && make hpp-python.install`, or source your local HPP
  environment.
- `No module named hpp_exec` or missing `read_current_configuration`: use an
  `hpp-exec` checkout that contains that helper.
- No ROS communication between the planner and Gazebo: make sure both sides use
  the same `ROS_DOMAIN_ID`; the wrapper defaults to `7`.
- Two simulations running at once: stop the extra one. Both publish the same
  robot topics and would interleave joint states.
