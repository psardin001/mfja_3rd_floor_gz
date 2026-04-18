Gazebo simulation for the 3rd floor of the Maison de la Formation Jacqueline Auriol (MFJA)
=================================================================================================================


# Introduction
The aim of this repository is to provide the world, models, and links towards other repositories to simulate robots at the third floor of the Maison de la Formation Jacqueline Auriol.

# Install

```
mkdir -p ~/mfja_ws/src
cd ~/mfja_ws/src
git clone https://github.com/olivier-stasse/mfja_3rd_floor_gz.git
cd ..
colcon build --symlink-install
```

# Simulation

## Walls only


```
ros2 launch mfja_3rd_floor_gz mfja_3rdf.launch.py world_name:=mfja_3rd_floor
```

## Room 315 kinematic shuttle

The detailed usage guide for the Room 315 kinematic shuttle, including room-only
and full-floor launch commands, multi-shuttle control, switch commands, runtime
shuttle spawning, collision avoidance, and validation tools is here:

`mfja_robot_control_config/config/room_315_kinematics/README.md`
