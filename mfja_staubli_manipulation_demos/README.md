# mfja_staubli_manipulation_demos

First scaffold for a Room 315 manipulation problem with:

- the Staubli TX2-60L,
- a fixed suction TCP mounted on the Staubli wrist,
- one stopped right-rail shuttle near the robot,
- a small payload box with an HPP handle and bottom contact surface,
- a shuttle deck HPP contact surface,
- a Staubli-table HPP drop-zone contact surface.

The initial scenario is deliberately quasi-static: the shuttle delivers the box
and waits at `slot_3`; HPP plans with the shuttle pose as a fixed support.

## Layout

```text
mfja_staubli_manipulation_demos/
├── config/
│   └── robots_room315_suction.yaml
├── hpp/
│   ├── room315_shuttle_manipulation.py
│   ├── room315_payload_box.urdf/.srdf
│   ├── room315_shuttle_deck.urdf/.srdf
│   ├── room315_staubli_table_drop_zone.urdf/.srdf
│   ├── room315_cell.urdf/.srdf
│   └── staubli_tx2_60l_manipulation.srdf
├── launch/
│   └── room_315_staubli_shuttle_manipulation_demo.launch.py
├── models/
│   └── staubli_tx2_60l_suction/model.sdf
├── scripts/
    ├── room315_demo.sh
    ├── room315_env.sh
    └── room315_hpp_manipulation.sh
└── urdf/
    └── staubli_tx2_60l_suction.urdf
```

## Run The Scene

Build or source the MFJA workspace, then launch:

```bash
mfja_staubli_manipulation_demos/scripts/room315_demo.sh
```

The launch starts Room 315 with only `staubli1`, enables the right shuttle rail,
creates one stopped shuttle at `slot_3`, and leaves the left rail disabled.

## HPP Scene Smoke Test

Use the hpp-exec wrapper:

```bash
mfja_staubli_manipulation_demos/scripts/room315_hpp_manipulation.sh --build-only
```

Plan the first pick-from-shuttle, place-on-table manipulation:

```bash
mfja_staubli_manipulation_demos/scripts/room315_hpp_manipulation.sh --plan-only
```

`--plan-only` also previews the sampled execution phases. The executor follows
the same idea as HPP tutorial 7: approach to pregrasp, run the semantic grasp
action, transfer toward the table, run the semantic release action, then
retreat.

Execute the plan in the running Room 315 Gazebo scene:

```bash
mfja_staubli_manipulation_demos/scripts/room315_hpp_manipulation.sh --execute
```

The execution uses the semantic HPP gripper `staubli/tool0_gripper`, now tied to
the fixed `suction_tcp` link on the visible Staubli suction tool. At the semantic
grasp/release boundaries it publishes:

```bash
/staubli1/suction_gripper/command  # std_msgs/msg/Bool, true=close, false=open
```

There is no detachable Gazebo payload joint yet; the visible box is spawned and
moved kinematically from the HPP object freeflyer pose through Gazebo
`/world/room_315_only/create`, `/set_pose`, and `/remove`.
The Gazebo box model is static, so it should not drift under physics. It is
held fixed on the shuttle during the approach phase, follows the HPP object pose
during the semantic grasp-transfer phase, and is fixed on the table during the
release-retreat phase.

By default the executor does not remove an existing box first, which avoids a
noisy Gazebo warning on a fresh scene. If a previous run left a box behind, use:

```bash
mfja_staubli_manipulation_demos/scripts/room315_hpp_manipulation.sh --execute --replace-box
```

The initial move to the HPP start pose is intentionally slow and includes a
short hold before the motion starts. Manipulation is then published as three
coarse phase trajectories on `/staubli1/joint_trajectory`, because the Room 315
Gazebo launch exposes a trajectory topic rather than the
`FollowJointTrajectory` action used by tutorial 7's `execute_segments()`.
The executor waits on `/staubli1/joint_states` after each phase to recover the
same "segment completed before next action" behavior.

## Real Robot Notes

The HPP problem already includes the payload and suction tool in the collision
model, with a small security margin to the Room 315 cell. The current execution
layer is still Gazebo-specific: the payload follows by repeated `set_pose`
calls, not by a physical attachment state. Before running on the real Staubli,
map `/staubli1/suction_gripper/command` to the real vacuum output, keep
`--plan-only` as the planning check, and send only the arm trajectory through
the real robot controller after validating speeds and clearances.

## Contact Surfaces

The payload box has:

- `box/top_handle`
- `box/bottom_surface`

The environment has:

- `shuttle/top_surface`
- `staubli_table/drop_zone`

`staubli_table/drop_zone` is a small patch on the real Staubli table top, away
from the robot base. It gives HPP a placement surface without turning the whole
robot support table into a new collision obstacle on the first pass.
