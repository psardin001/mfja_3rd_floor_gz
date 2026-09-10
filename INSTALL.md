# Gears: install, plan and execute

Ubuntu 24.04, ROS 2 Jazzy, Python 3.12. HPP is built from pinned sources.
The installation includes the MFJA models, Viser and the Stäubli driver.

## Install

```bash
sudo apt update
sudo apt install -y git
mkdir -p "$HOME/mfja-gears"
cd "$HOME/mfja-gears"
git clone --recurse-submodules https://github.com/psardin001/mfja_3rd_floor_gz.git
cd mfja_3rd_floor_gz
sudo bash installation/dependencies.sh
/usr/bin/python3 installation/install.py "$HOME/mfja-gears"
```

Builds use one job. Logs are in `~/mfja-gears/logs/`.
To resume an interrupted installation, repeat the last command.

In each terminal:

```bash
source "$HOME/mfja-gears/setup.bash"
python "$MFJA_ROOT/installation/check.py"
cd "$HPP_TUTORIAL_DIR/mfja"
```

## Plan and view

```bash
python -i one_gear.py
# Or:
python -i two_gears.py
```

In Python:

```python
v = Viewer(robot)
v.initViewer(open=True, loadModel=True)
v.setProblem(problem)
v.setGraph(graph)
v.loadPath(p2)
```

## Prepare the robot

Install the matching [VAL3 programs](https://github.com/psardin001/Staubli_ROS2/blob/18af18f7c4a34270c09dd4a83bcf8885da52d2c6/README.md#installing-the-val3-components)
from `Staubli_ROS2/staubli_val3_driver/val3` on the CS9 controller.

Verify the cell, fixture poses, TCP and gripper. Update `environment.py` for
measured fixture poses, then set `calibrated: true` in
`two_gears_execution.yaml` and generate a new plan.

The driver uses TOPPRA velocities with nominal joint speeds as the VAL3
reference. Physical tracking still depends on the controller and its settings.
Use the same ROS domain in both terminals (Room 315 uses domain 7).

Terminal 1, using the cell's ROS domain:

```bash
source "$HOME/mfja-gears/setup.bash"
export ROS_DOMAIN_ID=7
read -r -p "Controller IP: " ROBOT_IP
ros2 launch mfja_staubli_manipulation_demos room_315_staubli_hardware.launch.py \
  robot_ip:="$ROBOT_IP" joint_config:="$MFJA_ROOT/installation/staubli_gears.yaml"
```

## Execute one gear

Terminal 2:

```bash
source "$HOME/mfja-gears/setup.bash"
export ROS_DOMAIN_ID=7
cd "$HPP_TUTORIAL_DIR/mfja"
python one_gear_execute.py --execute --all
```

Place the gear at its initial location and leave the gripper empty. The script
plans from the measured arm position, validates the path and executes the cycle.
Omit `--all` to choose the next segment, the remaining cycle, or quit.

## Execute two gears

In terminal 2, place both gears at their initial locations and leave the gripper
empty. Run the same workflow for two gears:

```bash
python two_gears_execute.py --execute --all --seed 3
```

The installation recipe is grouped in [installation/](installation/).
`sources.repos` pins HPP and the tutorial; this MFJA checkout pins the driver
through its Git submodule. Calibration values remain local.
