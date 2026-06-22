#!/usr/bin/python3
"""Plan and execute the Room 315 Staubli shuttle payload manipulation demo."""

import argparse
import time
from dataclasses import dataclass

import numpy as np
import pinocchio as pin
import rclpy
from geometry_msgs.msg import Pose
from hpp_exec import configs_to_joint_trajectory
from pyhpp.manipulation import Device, Graph, Problem, TransitionPlanner, urdf
from pyhpp.manipulation.constraint_graph_factory import ConstraintGraphFactory
from pyhpp.manipulation.security_margins import SecurityMargins
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity, SetEntityPose, SpawnEntity
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory

ROOM315_ROBOT_POSE = (-15.1622, -6.0, 1.0, 0.0, 0.0, 1.57)
DEFAULT_SHUTTLE_SLOT3_POSE = (-15.240, -5.536, 0.839, 0.0, 0.0, 0.0)
TABLE_DROP_ZONE_POSE = (-14.65, -5.84, 1.003, 0.0, 0.0, 0.0)
GRAPH_NAME = "room315_staubli_shuttle_box"

JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]
DEFAULT_Q_START = np.array(
    [-1.56136443, 0.47307870, 2.04964315, -0.00130315, -0.32991444, 0.00524110]
)
BOX_SIZE = (0.10, 0.08, 0.08)
BOX_HEIGHT = BOX_SIZE[2]
SHUTTLE_CONTACT_Z = 0.085
BOX_ENTITY_NAME = "room315_payload_box"
WORLD_NAME = "room_315_only"
BOX_ROOM315_MARGIN = 0.03
STAUBLI_ROOM315_MARGIN = 0.02

ROBOT_URDF = (
    "package://mfja_staubli_manipulation_demos/urdf/"
    "staubli_tx2_60l_suction.urdf"
)
ROBOT_SRDF = (
    "package://mfja_staubli_manipulation_demos/hpp/"
    "staubli_tx2_60l_manipulation.srdf"
)
CELL_URDF = "package://mfja_staubli_manipulation_demos/hpp/room315_cell.urdf"
CELL_SRDF = "package://mfja_staubli_manipulation_demos/hpp/room315_cell.srdf"
BOX_URDF = "package://mfja_staubli_manipulation_demos/hpp/room315_payload_box.urdf"
BOX_SRDF = "package://mfja_staubli_manipulation_demos/hpp/room315_payload_box.srdf"
SHUTTLE_URDF = (
    "package://mfja_staubli_manipulation_demos/hpp/room315_shuttle_deck.urdf"
)
SHUTTLE_SRDF = (
    "package://mfja_staubli_manipulation_demos/hpp/room315_shuttle_deck.srdf"
)
TABLE_URDF = (
    "package://mfja_staubli_manipulation_demos/hpp/"
    "room315_staubli_table_drop_zone.urdf"
)
TABLE_SRDF = (
    "package://mfja_staubli_manipulation_demos/hpp/"
    "room315_staubli_table_drop_zone.srdf"
)

PICK_TRANSITIONS = [
    "staubli/tool0_gripper > box/top_handle | f_01",
    "staubli/tool0_gripper > box/top_handle | f_12",
    "staubli/tool0_gripper > box/top_handle | f_23",
    "staubli/tool0_gripper > box/top_handle | f_34",
]
TRANSFER_TRANSITION = "Loop | 0-0"
RELEASE_TRANSITIONS = [
    "staubli/tool0_gripper < box/top_handle | 0-0_43",
    "staubli/tool0_gripper < box/top_handle | 0-0_32",
    "staubli/tool0_gripper < box/top_handle | 0-0_21",
    "staubli/tool0_gripper < box/top_handle | 0-0_10",
]

PAYLOAD_BOX_SDF = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{BOX_ENTITY_NAME}">
    <static>true</static>
    <link name="base_link">
      <inertial>
        <mass>0.2</mass>
        <inertia>
          <ixx>0.0002</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.0002</iyy>
          <iyz>0</iyz>
          <izz>0.0002</izz>
        </inertia>
      </inertial>
      <visual name="payload_visual">
        <geometry>
          <box>
            <size>{BOX_SIZE[0]} {BOX_SIZE[1]} {BOX_SIZE[2]}</size>
          </box>
        </geometry>
        <material>
          <ambient>0.05 0.35 0.95 1</ambient>
          <diffuse>0.05 0.35 0.95 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
        </material>
      </visual>
      <collision name="payload_collision">
        <geometry>
          <box>
            <size>{BOX_SIZE[0]} {BOX_SIZE[1]} {BOX_SIZE[2]}</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
"""


@dataclass
class PlannedSegment:
    transition_name: str
    path: object
    q_start: np.ndarray
    q_goal: np.ndarray


@dataclass
class ExecutionPhase:
    name: str
    planned_segments: list[PlannedSegment]
    payload_mode: str
    configs: list[np.ndarray]
    payload_configs: list[np.ndarray]
    times: list[float]


class JointStateTracker:
    def __init__(self, node, topic):
        self.node = node
        self.topic = topic
        self.configuration = None
        self.last_update = None
        self.subscription = node.create_subscription(
            JointState, topic, self.update, 10
        )

    def update(self, message):
        positions = {
            name.split("::")[-1]: value
            for name, value in zip(message.name, message.position)
        }
        try:
            self.configuration = np.array([positions[joint] for joint in JOINT_NAMES])
            self.last_update = time.monotonic()
        except KeyError:
            return

    def wait(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.configuration is not None:
                return self.configuration.copy()
        return None

    def current(self):
        if self.configuration is None:
            return None
        return self.configuration.copy()

    def is_stale(self, timeout):
        return (
            self.last_update is None
            or time.monotonic() - self.last_update > timeout
        )


def se3_from_pose(pose):
    x, y, z, roll, pitch, yaw = pose
    return pin.SE3(pin.rpy.rpyToMatrix(roll, pitch, yaw), np.array([x, y, z]))


def world_pose_in_robot_frame(world_pose):
    return se3_from_pose(ROOM315_ROBOT_POSE).inverse() * se3_from_pose(world_pose)


def pose_msg_from_se3(placement):
    quat = pin.Quaternion(placement.rotation).coeffs()
    pose = Pose()
    pose.position.x = float(placement.translation[0])
    pose.position.y = float(placement.translation[1])
    pose.position.z = float(placement.translation[2])
    pose.orientation.x = float(quat[0])
    pose.orientation.y = float(quat[1])
    pose.orientation.z = float(quat[2])
    pose.orientation.w = float(quat[3])
    return pose


def build_problem(shuttle_pose):
    robot = Device("room315_staubli_manipulation")

    urdf.loadModel(
        robot, 0, "staubli", "anchor", ROBOT_URDF, ROBOT_SRDF, pin.SE3.Identity()
    )
    urdf.loadModel(
        robot,
        0,
        "room315",
        "anchor",
        CELL_URDF,
        CELL_SRDF,
        se3_from_pose(ROOM315_ROBOT_POSE).inverse(),
    )
    urdf.loadModel(
        robot,
        0,
        "shuttle",
        "anchor",
        SHUTTLE_URDF,
        SHUTTLE_SRDF,
        world_pose_in_robot_frame(shuttle_pose),
    )
    urdf.loadModel(
        robot,
        0,
        "staubli_table",
        "anchor",
        TABLE_URDF,
        TABLE_SRDF,
        world_pose_in_robot_frame(TABLE_DROP_ZONE_POSE),
    )
    urdf.loadModel(
        robot, 0, "box", "freeflyer", BOX_URDF, BOX_SRDF, pin.SE3.Identity()
    )
    robot.setJointBounds(
        "box/root_joint",
        [
            -1.2,
            1.2,
            -1.0,
            1.2,
            -0.4,
            0.8,
            -float("inf"),
            float("inf"),
            -float("inf"),
            float("inf"),
            -float("inf"),
            float("inf"),
            -float("inf"),
            float("inf"),
        ],
    )

    problem = Problem(robot)
    problem.addConfigValidation("CollisionValidation")
    problem.addConfigValidation("JointBoundValidation")

    graph = Graph(GRAPH_NAME, robot, problem)
    graph.maxIterations(40)
    graph.errorThreshold(1e-5)

    factory = ConstraintGraphFactory(graph)
    factory.setGrippers(["staubli/tool0_gripper"])
    factory.setObjects(
        ["box"],
        [["box/top_handle"]],
        [["box/bottom_surface"]],
    )
    factory.environmentContacts(["shuttle/top_surface", "staubli_table/drop_zone"])
    factory.generate()

    margins = SecurityMargins(
        problem,
        factory,
        ["staubli", "box", "room315", "shuttle", "staubli_table"],
        robot,
    )
    margins.setSecurityMarginBetween("box", "room315", BOX_ROOM315_MARGIN)
    margins.setSecurityMarginBetween("staubli", "room315", STAUBLI_ROOM315_MARGIN)
    margins.apply()

    graph.initialize()
    return robot, problem, graph


def mapping_names(mapping):
    if hasattr(mapping, "keys"):
        return sorted(mapping.keys())
    return sorted(entry.key() for entry in mapping)


def box_rank(robot):
    return robot.rankInConfiguration["box/root_joint"]


def box_world_pose(robot, q):
    rank = box_rank(robot)
    quat = pin.Quaternion(np.asarray(q[rank + 3 : rank + 7]))
    box_in_robot = pin.SE3(quat.matrix(), np.asarray(q[rank : rank + 3]))
    return se3_from_pose(ROOM315_ROBOT_POSE) * box_in_robot


def box_world_pose_msg(robot, q):
    return pose_msg_from_se3(box_world_pose(robot, q))


def box_configuration_from_world_pose(q_arm, world_pose):
    box_pose = world_pose_in_robot_frame(world_pose)
    return np.r_[q_arm, box_pose.translation, pin.Quaternion(box_pose.rotation).coeffs()]


def shuttle_box_world_pose(shuttle_pose):
    x, y, z, roll, pitch, yaw = shuttle_pose
    return (x, y, z + SHUTTLE_CONTACT_Z + 0.5 * BOX_HEIGHT, roll, pitch, yaw)


def table_box_world_pose():
    x, y, z, roll, pitch, yaw = TABLE_DROP_ZONE_POSE
    return (x, y, z + 0.5 * BOX_HEIGHT, roll, pitch, yaw)


def project_free_configuration(problem, graph, q, label):
    ok, q_projected, error = graph.applyStateConstraints(graph.getState("free"), q)
    if not ok:
        raise RuntimeError(f"failed to project {label} on free state: {error:.3g}")
    q_projected = np.asarray(q_projected).flatten()
    valid, report = problem.isConfigValid(q_projected)
    if not valid:
        raise RuntimeError(f"{label} configuration is invalid: {report}")
    return q_projected


def make_goal_matrix(robot, q_goal):
    q_goals = np.zeros((1, robot.configSize()), order="F")
    q_goals[0, :] = q_goal
    return q_goals


def validate_transition_config(transition, q, label):
    valid, report = transition.pathValidation().validateConfiguration(q)
    if not valid:
        raise RuntimeError(f"{label} target is invalid: {report}")


def seeded_target(shooter, q_free, rank, attempt, preferred=None):
    q_seed = np.asarray(shooter.shoot()).flatten()
    q_seed[rank : rank + 7] = q_free[rank : rank + 7]
    if preferred is not None and attempt % 3 == 0:
        q_seed[:6] = preferred[:6]
    elif attempt % 3 == 1:
        q_seed[:6] = q_free[:6]
    return q_seed


def arm_chain_score(q_free, chain, preferred=None):
    reference = q_free if preferred is None else preferred
    configs = [q_free] + chain
    motion = sum(
        float(np.max(np.abs(current[:6] - previous[:6])))
        for previous, current in zip(configs[:-1], configs[1:])
    )
    posture = float(np.max(np.abs(chain[-1][:6] - reference[:6])))
    wrist_wrap = float(np.sum(np.maximum(0.0, np.abs(chain[-1][:6]) - np.pi)))
    return motion + 0.5 * posture + 0.5 * wrist_wrap


def generate_pick_chain(robot, problem, graph, q_free, attempts, label, preferred=None):
    shooter = problem.configurationShooter()
    rank = box_rank(robot)
    best_chain = None
    best_attempt = 0
    best_score = float("inf")

    for attempt in range(attempts):
        seed = seeded_target(shooter, q_free, rank, attempt, preferred)
        source = q_free
        chain = []

        for index, transition_name in enumerate(PICK_TRANSITIONS):
            transition = graph.getTransition(transition_name)
            initializer = seed if index == 0 else source
            ok, q_next, error = graph.generateTargetConfig(
                transition, source, initializer
            )
            if not ok:
                break

            q_next = np.asarray(q_next).flatten()
            try:
                validate_transition_config(
                    transition, q_next, f"{label} {transition_name}"
                )
            except RuntimeError:
                break

            chain.append(q_next)
            source = q_next

        if len(chain) != len(PICK_TRANSITIONS):
            continue

        score = arm_chain_score(q_free, chain, preferred)
        if score < best_score:
            best_chain = chain
            best_attempt = attempt + 1
            best_score = score

    if best_chain is not None:
        print(
            f"{label} pick chain selected from {attempts} attempt(s) "
            f"(best attempt {best_attempt}, score {best_score:.3f})",
            flush=True,
        )
        return best_chain

    raise RuntimeError(f"failed to generate {label} pick chain after {attempts} attempts")


def plan_transition(robot, planner, graph, transition_name, q_start, q_goal):
    transition = graph.getTransition(transition_name)
    validate_transition_config(transition, q_goal, transition_name)
    planner.setEdge(transition)
    success, path, report = planner.directPath(q_start, q_goal, True)
    if success:
        return PlannedSegment(transition_name, path, q_start, q_goal)

    try:
        path = planner.planPath(q_start, make_goal_matrix(robot, q_goal), True)
    except Exception as exc:
        raise RuntimeError(
            f"failed to plan transition {transition_name}: {report}"
        ) from exc
    return PlannedSegment(transition_name, path, q_start, q_goal)


def plan_manipulation(
    robot,
    problem,
    graph,
    q_shuttle,
    q_table,
    *,
    target_attempts,
    transition_iterations,
    transition_timeout,
):
    problem.constraintGraph(graph)
    planner = TransitionPlanner(problem)
    planner.maxIterations(transition_iterations)
    planner.timeOut(transition_timeout)

    shuttle_pick = generate_pick_chain(
        robot, problem, graph, q_shuttle, target_attempts, "shuttle"
    )
    table_pick = generate_pick_chain(
        robot,
        problem,
        graph,
        q_table,
        target_attempts,
        "table",
        preferred=shuttle_pick[-1],
    )

    segments = []
    current = q_shuttle
    for transition_name, target in zip(PICK_TRANSITIONS, shuttle_pick):
        segment = plan_transition(robot, planner, graph, transition_name, current, target)
        segments.append(segment)
        current = target

    segment = plan_transition(
        robot, planner, graph, TRANSFER_TRANSITION, current, table_pick[-1]
    )
    segments.append(segment)
    current = table_pick[-1]

    release_targets = [table_pick[2], table_pick[1], table_pick[0], q_table]
    for transition_name, target in zip(RELEASE_TRANSITIONS, release_targets):
        segment = plan_transition(robot, planner, graph, transition_name, current, target)
        segments.append(segment)
        current = target

    return segments


def sample_path(path, samples):
    length = float(path.length())
    if samples < 2:
        samples = 2
    if length <= 1e-9:
        q, ok = path(0.0)
        if not ok:
            raise RuntimeError("HPP failed to evaluate a zero-length path")
        config = np.asarray(q).flatten()
        return [config, config.copy()]

    configs = []
    for index in range(samples):
        q, ok = path(index / (samples - 1) * length)
        if not ok:
            raise RuntimeError(f"HPP failed to evaluate path sample {index}")
        configs.append(np.asarray(q).flatten())
    return configs


def format_plan(segments):
    rows = []
    total = 0.0
    for index, segment in enumerate(segments):
        length = float(segment.path.length())
        total += length
        rows.append((index, segment.transition_name, length))

    print("planned manipulation transitions:")
    for index, name, length in rows:
        print(f"  {index:02d}  {length:8.3f}  {name}")
    print(f"total HPP path parameter length: {total:.3f}")


def path_sample_count(path, samples_per_path_unit, min_segment_samples):
    return max(min_segment_samples, int(float(path.length()) * samples_per_path_unit) + 1)


def retime_joint_configs(configs, *, max_joint_speed, min_sample_dt, initial_hold):
    times = [0.0]
    if len(configs) > 1:
        times.append(initial_hold)

    for previous, current in zip(configs[1:-1], configs[2:]):
        delta = float(np.max(np.abs(current[:6] - previous[:6])))
        times.append(times[-1] + max(min_sample_dt, delta / max_joint_speed))
    return times


def execution_config(robot, arm_config, payload_config):
    q = np.asarray(arm_config).copy()
    rank = box_rank(robot)
    q[rank : rank + 7] = payload_config[rank : rank + 7]
    return normalize_box_quaternion(robot, q)


def append_execution_sample(robot, arm_configs, payload_configs, arm_config, payload_config):
    rank = box_rank(robot)
    arm_config = np.asarray(arm_config).flatten()
    payload_config = np.asarray(payload_config).flatten()
    same_arm = (
        arm_configs
        and np.max(np.abs(arm_config[:6] - arm_configs[-1][:6])) < 1e-8
    )
    same_payload = (
        payload_configs
        and np.max(
            np.abs(
                payload_config[rank : rank + 7]
                - payload_configs[-1][rank : rank + 7]
            )
        )
        < 1e-8
    )
    if same_arm and same_payload:
        arm_configs[-1] = arm_config
        payload_configs[-1] = payload_config
    else:
        arm_configs.append(arm_config)
        payload_configs.append(payload_config)


def build_execution_phase(
    robot,
    graph,
    name,
    planned_segments,
    payload_mode,
    fixed_payload,
    args,
):
    configs = []
    payload_configs = []
    transition_names = []

    for segment_index, segment in enumerate(planned_segments):
        transition = graph.getTransition(segment.transition_name)
        transition_names.append(segment.transition_name)
        samples = path_sample_count(
            segment.path, args.samples_per_path_unit, args.min_segment_samples
        )
        segment_configs = sample_path(segment.path, samples)

        if payload_mode == "follow":
            segment_payload = segment_configs
        else:
            segment_payload = [fixed_payload.copy() for _ in segment_configs]

        for sample_index, (arm_config, payload_config) in enumerate(
            zip(segment_configs, segment_payload)
        ):
            arm_config = np.asarray(arm_config).flatten()
            payload_config = np.asarray(payload_config).flatten()
            q = execution_config(robot, arm_config, payload_config)
            valid, report = transition.pathValidation().validateConfiguration(q)
            if not valid:
                raise RuntimeError(
                    f"execution phase {name} segment {segment_index} "
                    f"sample {sample_index} is invalid: {report}"
                )
            append_execution_sample(
                robot, configs, payload_configs, arm_config, payload_config
            )

    if configs:
        configs.insert(1, configs[0].copy())
        payload_configs.insert(1, payload_configs[0].copy())

    times = retime_joint_configs(
        configs,
        max_joint_speed=args.max_joint_speed,
        min_sample_dt=args.min_sample_dt,
        initial_hold=args.phase_start_hold,
    )
    validate_sampled_configs(robot, configs, payload_configs, times)
    print(
        f"execution phase {name}: {payload_mode}, "
        f"{len(configs)} points, {times[-1]:.1f} s",
        flush=True,
    )
    for transition_name in transition_names:
        print(f"  {transition_name}", flush=True)
    return ExecutionPhase(
        name,
        planned_segments,
        payload_mode,
        configs,
        payload_configs,
        times,
    )


def build_execution_phases(robot, graph, segments, q_shuttle, q_table, args):
    grasp_index = next(
        index
        for index, segment in enumerate(segments)
        if segment.transition_name == PICK_TRANSITIONS[2]
    )
    release_index = next(
        index
        for index, segment in enumerate(segments)
        if segment.transition_name == RELEASE_TRANSITIONS[2]
    )

    phases = [
        build_execution_phase(
            robot,
            graph,
            "approach-pregrasp",
            segments[:grasp_index],
            "shuttle-fixed",
            q_shuttle,
            args,
        ),
        build_execution_phase(
            robot,
            graph,
            "grasp-transfer",
            segments[grasp_index:release_index],
            "follow",
            q_shuttle,
            args,
        ),
        build_execution_phase(
            robot,
            graph,
            "release-retreat",
            segments[release_index:],
            "table-fixed",
            q_table,
            args,
        ),
    ]
    total_points = sum(len(phase.configs) for phase in phases)
    total_duration = sum(phase.times[-1] for phase in phases)
    print(
        f"execution preview: {len(phases)} phases, "
        f"{total_points} points, {total_duration:.1f} s",
        flush=True,
    )
    return phases


def validate_sampled_configs(robot, arm_configs, payload_configs, times):
    if not (len(arm_configs) == len(payload_configs) == len(times)):
        raise RuntimeError(
            "internal execution sampling error: arm, payload, and time lengths differ"
        )
    if len(arm_configs) < 2:
        raise RuntimeError("internal execution sampling error: empty trajectory")
    for index, (arm, payload) in enumerate(zip(arm_configs, payload_configs)):
        if arm.shape[0] != robot.configSize() or payload.shape[0] != robot.configSize():
            raise RuntimeError(f"internal execution sample {index} has wrong size")


def publish_trajectory(node, publisher, topic, trajectory):
    if publisher.get_subscription_count() == 0:
        node.get_logger().warning(f"no subscriber detected on {topic}")
    publisher.publish(trajectory)
    rclpy.spin_once(node, timeout_sec=0.05)


def timed_joint_trajectory(configs, times):
    return configs_to_joint_trajectory(configs, times, JOINT_NAMES)


def wait_for_subscriber(node, publisher, topic, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and publisher.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if publisher.get_subscription_count() == 0:
        node.get_logger().warning(f"no subscriber detected on {topic}")


def call_service(node, client, request, label, timeout=3.0, require_success=True):
    if not client.wait_for_service(timeout_sec=timeout):
        raise RuntimeError(f"{label} service is unavailable")

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        raise RuntimeError(f"{label} service call timed out")

    result = future.result()
    if result is None:
        raise RuntimeError(f"{label} service returned no result")
    if require_success and hasattr(result, "success") and not result.success:
        raise RuntimeError(f"{label} service failed: {result}")
    return result


def delete_payload(node, client, entity_name):
    request = DeleteEntity.Request()
    request.entity.name = entity_name
    request.entity.type = Entity.MODEL
    try:
        call_service(
            node,
            client,
            request,
            f"delete {entity_name}",
            timeout=2.0,
            require_success=False,
        )
    except RuntimeError as exc:
        node.get_logger().warning(str(exc))


def spawn_payload(node, spawn_client, entity_name, pose):
    request = SpawnEntity.Request()
    request.entity_factory.name = entity_name
    request.entity_factory.allow_renaming = False
    request.entity_factory.sdf = PAYLOAD_BOX_SDF
    request.entity_factory.pose = pose
    request.entity_factory.relative_to = "world"
    try:
        call_service(node, spawn_client, request, f"spawn {entity_name}", timeout=5.0)
        return True
    except RuntimeError as exc:
        node.get_logger().warning(str(exc))
        return False


def set_payload_pose(node, pose_client, entity_name, pose, timeout=1.0):
    request = SetEntityPose.Request()
    request.entity.name = entity_name
    request.entity.type = Entity.MODEL
    request.pose = pose
    call_service(
        node,
        pose_client,
        request,
        f"set pose for {entity_name}",
        timeout=timeout,
    )


def sleep_with_spin(node, duration):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())))


def normalize_box_quaternion(robot, q):
    q = np.asarray(q).copy()
    rank = box_rank(robot)
    quat = q[rank + 3 : rank + 7]
    norm = np.linalg.norm(quat)
    if norm > 1e-12:
        q[rank + 3 : rank + 7] = quat / norm
    return q


def interpolate_indexed_config(robot, configs, progress):
    if progress <= 0:
        return configs[0]
    if progress >= len(configs) - 1:
        return configs[-1]

    lower = int(np.floor(progress))
    upper = lower + 1
    alpha = progress - lower
    q = (1.0 - alpha) * configs[lower] + alpha * configs[upper]
    return normalize_box_quaternion(robot, q)


def nearest_arm_progress(current, arm_positions, progress, lookahead):
    if len(arm_positions) < 2:
        return 0.0, float(np.max(np.abs(current - arm_positions[0])))

    first = max(0, int(np.floor(progress)) - 1)
    last = min(len(arm_positions) - 2, int(np.floor(progress)) + lookahead)
    best_progress = progress
    best_error = float("inf")

    for index in range(first, last + 1):
        start = arm_positions[index]
        end = arm_positions[index + 1]
        delta = end - start
        norm2 = float(delta @ delta)
        if norm2 <= 1e-12:
            alpha = 0.0
            closest = start
        else:
            alpha = float(np.clip(((current - start) @ delta) / norm2, 0.0, 1.0))
            closest = start + alpha * delta
        error = float(np.max(np.abs(current - closest)))
        candidate = index + alpha
        if error < best_error:
            best_error = error
            best_progress = candidate

    return max(progress, best_progress), best_error


def payload_pose_changed(robot, previous, current, threshold):
    if previous is None:
        return True
    rank = box_rank(robot)
    return (
        np.max(np.abs(current[rank : rank + 3] - previous[rank : rank + 3]))
        > threshold
        or np.max(np.abs(current[rank + 3 : rank + 7] - previous[rank + 3 : rank + 7]))
        > threshold
    )


def wait_for_arm_configuration(node, tracker, target, timeout, tolerance):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        current = tracker.current()
        if current is not None:
            error = float(np.max(np.abs(current - target)))
            if error <= tolerance:
                return True
        rclpy.spin_once(node, timeout_sec=0.05)
    return False


def wait_for_phase_end(node, tracker, phase, args):
    timeout = args.execution_timeout_scale * phase.times[-1] + 5.0
    if wait_for_arm_configuration(
        node, tracker, phase.configs[-1][:6], timeout, args.segment_tolerance
    ):
        return True

    current = tracker.current()
    error = (
        float(np.max(np.abs(current - phase.configs[-1][:6])))
        if current is not None
        else float("inf")
    )
    raise RuntimeError(
        f"Staubli did not finish phase {phase.name} within {timeout:.1f} s "
        f"(error {error:.3f} rad)"
    )


def follow_payload(
    node,
    pose_client,
    tracker,
    robot,
    entity_name,
    arm_configs,
    payload_configs,
    times,
    args,
):
    arm_positions = np.asarray([config[:6] for config in arm_configs])
    period = 1.0 / args.box_rate
    start = time.monotonic()
    deadline = start + args.execution_timeout_scale * times[-1] + 30.0
    next_tick = start
    progress = 0.0
    last_payload_config = None
    last_report = start

    while True:
        now = time.monotonic()
        if tracker.is_stale(args.joint_state_stale_timeout):
            raise RuntimeError(
                f"no fresh joint state on {tracker.topic} for "
                f"{args.joint_state_stale_timeout:.1f} s"
            )

        current = tracker.current()
        if current is not None:
            candidate, error = nearest_arm_progress(
                current,
                arm_positions,
                progress,
                args.payload_sync_lookahead,
            )
            if error <= args.payload_sync_error:
                progress = candidate
            elif now - last_report >= args.payload_sync_report_period:
                print(
                    f"payload sync waiting: progress={progress:.1f}/"
                    f"{len(arm_configs) - 1}, nearest error={error:.3f} rad",
                    flush=True,
                )
                last_report = now

        q = interpolate_indexed_config(robot, payload_configs, progress)
        if payload_pose_changed(robot, last_payload_config, q, args.payload_pose_epsilon):
            set_payload_pose(
                node,
                pose_client,
                entity_name,
                box_world_pose_msg(robot, q),
                timeout=0.5,
            )
            last_payload_config = q

        if progress >= len(arm_configs) - 1:
            break
        if now >= deadline:
            final_snap_start = len(arm_configs) - 1 - args.payload_final_snap_samples
            if progress >= final_snap_start:
                set_payload_pose(
                    node,
                    pose_client,
                    entity_name,
                    box_world_pose_msg(robot, payload_configs[-1]),
                    timeout=0.5,
                )
                print(
                    f"payload sync final snap: progress={progress:.1f}/"
                    f"{len(arm_configs) - 1}",
                    flush=True,
                )
                break
            raise RuntimeError(
                f"payload sync timed out at progress {progress:.1f}/"
                f"{len(arm_configs) - 1}"
            )
        next_tick += period
        sleep_with_spin(node, max(0.0, next_tick - time.monotonic()))


def set_payload_config(node, pose_client, robot, entity_name, config):
    set_payload_pose(
        node,
        pose_client,
        entity_name,
        box_world_pose_msg(robot, config),
    )


def publish_gripper_command(node, publisher, topic, closed):
    message = Bool()
    message.data = closed
    publisher.publish(message)
    rclpy.spin_once(node, timeout_sec=0.05)
    action = "close" if closed else "open"
    print(f"suction gripper {action}: {topic}={closed}", flush=True)


def semantic_grasp(
    node,
    gripper_publisher,
    gripper_topic,
    pose_client,
    robot,
    entity_name,
    phase,
):
    publish_gripper_command(node, gripper_publisher, gripper_topic, True)
    print("semantic grasp: payload follows suction TCP pose")
    set_payload_config(node, pose_client, robot, entity_name, phase.payload_configs[0])
    return True


def semantic_release(
    node,
    gripper_publisher,
    gripper_topic,
    pose_client,
    robot,
    entity_name,
    phase,
):
    publish_gripper_command(node, gripper_publisher, gripper_topic, False)
    print("semantic release: payload fixed on table drop-zone")
    set_payload_config(node, pose_client, robot, entity_name, phase.payload_configs[0])
    return True


def execute_phase(
    node,
    publisher,
    topic,
    pose_client,
    tracker,
    robot,
    entity_name,
    phase,
    args,
):
    trajectory = timed_joint_trajectory(phase.configs, phase.times)
    print(
        f"publishing phase {phase.name}: "
        f"{len(phase.configs)} points, {phase.times[-1]:.1f} s",
        flush=True,
    )
    publish_trajectory(node, publisher, topic, trajectory)

    if phase.payload_mode == "follow":
        follow_payload(
            node,
            pose_client,
            tracker,
            robot,
            entity_name,
            phase.configs,
            phase.payload_configs,
            phase.times,
            args,
        )
        wait_for_phase_end(node, tracker, phase, args)
    else:
        wait_for_phase_end(node, tracker, phase, args)


def move_to_start(node, publisher, topic, tracker, args, q_start):
    current = tracker.wait(args.joint_state_timeout)
    if current is None:
        raise RuntimeError(f"could not read /{args.robot_name}/joint_states")

    delta = float(np.max(np.abs(current - q_start[:6])))
    if delta < 0.02:
        return

    duration = max(args.min_start_duration, delta / args.start_joint_speed)
    n_samples = max(3, int(duration * args.start_samples_per_second) + 1)
    start_configs = [
        (1.0 - alpha) * current + alpha * q_start[:6]
        for alpha in np.linspace(0.0, 1.0, n_samples)
    ]
    trajectory = timed_joint_trajectory(
        [current] + start_configs,
        [0.0]
        + np.linspace(args.initial_hold, args.initial_hold + duration, n_samples).tolist(),
    )
    print(f"moving Staubli to the planned start ({duration:.1f} s)")
    publish_trajectory(node, publisher, topic, trajectory)
    timeout = args.execution_timeout_scale * (args.initial_hold + duration) + 5.0
    if not wait_for_arm_configuration(
        node, tracker, q_start[:6], timeout, args.start_tolerance
    ):
        current = tracker.current()
        error = (
            float(np.max(np.abs(current - q_start[:6])))
            if current is not None
            else float("inf")
        )
        raise RuntimeError(
            f"Staubli did not reach the planned start within {timeout:.1f} s "
            f"(error {error:.3f} rad)"
        )


def execute_plan(robot, graph, segments, q_shuttle, q_table, args):
    rclpy.init()
    node = Node("room315_hpp_manipulation")
    try:
        trajectory_topic = f"/{args.robot_name}/joint_trajectory"
        joint_state_topic = f"/{args.robot_name}/joint_states"
        gripper_topic = args.gripper_command_topic or (
            f"/{args.robot_name}/suction_gripper/command"
        )
        publisher = node.create_publisher(JointTrajectory, trajectory_topic, 10)
        gripper_publisher = node.create_publisher(Bool, gripper_topic, 10)
        tracker = JointStateTracker(node, joint_state_topic)
        wait_for_subscriber(node, publisher, trajectory_topic, args.subscriber_timeout)

        service_prefix = f"/world/{args.world_name}"
        spawn_client = node.create_client(SpawnEntity, f"{service_prefix}/create")
        delete_client = node.create_client(DeleteEntity, f"{service_prefix}/remove")
        pose_client = node.create_client(SetEntityPose, f"{service_prefix}/set_pose")

        if args.replace_box:
            delete_payload(node, delete_client, args.box_entity_name)
        spawned = spawn_payload(
            node, spawn_client, args.box_entity_name, box_world_pose_msg(robot, q_shuttle)
        )
        if not spawned:
            node.get_logger().info(
                f"using existing Gazebo entity {args.box_entity_name}"
            )
        set_payload_pose(
            node,
            pose_client,
            args.box_entity_name,
            box_world_pose_msg(robot, q_shuttle),
        )

        move_to_start(node, publisher, trajectory_topic, tracker, args, q_shuttle)

        phases = build_execution_phases(
            robot, graph, segments, q_shuttle, q_table, args
        )
        set_payload_config(
            node,
            pose_client,
            robot,
            args.box_entity_name,
            q_shuttle,
        )

        execute_phase(
            node,
            publisher,
            trajectory_topic,
            pose_client,
            tracker,
            robot,
            args.box_entity_name,
            phases[0],
            args,
        )
        semantic_grasp(
            node,
            gripper_publisher,
            gripper_topic,
            pose_client,
            robot,
            args.box_entity_name,
            phases[1],
        )
        execute_phase(
            node,
            publisher,
            trajectory_topic,
            pose_client,
            tracker,
            robot,
            args.box_entity_name,
            phases[1],
            args,
        )
        semantic_release(
            node,
            gripper_publisher,
            gripper_topic,
            pose_client,
            robot,
            args.box_entity_name,
            phases[2],
        )
        execute_phase(
            node,
            publisher,
            trajectory_topic,
            pose_client,
            tracker,
            robot,
            args.box_entity_name,
            phases[2],
            args,
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-name", default="staubli1")
    parser.add_argument("--world-name", default=WORLD_NAME)
    parser.add_argument("--box-entity-name", default=BOX_ENTITY_NAME)
    parser.add_argument(
        "--gripper-command-topic",
        default=None,
        help=(
            "Bool topic used for suction open/close commands. Defaults to "
            "/<robot-name>/suction_gripper/command."
        ),
    )
    parser.add_argument(
        "--shuttle-pose",
        nargs=6,
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
        type=float,
        default=DEFAULT_SHUTTLE_SLOT3_POSE,
        help="Gazebo/world pose of the shuttle model at the pickup slot.",
    )
    parser.add_argument(
        "--q-start",
        nargs=6,
        metavar=tuple(JOINT_NAMES),
        default=DEFAULT_Q_START,
        type=float,
        help="Staubli joint configuration used to seed the shuttle and table placements.",
    )
    parser.add_argument("--build-only", action="store_true")
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--target-attempts", type=int, default=200)
    parser.add_argument("--transition-iterations", type=int, default=1000)
    parser.add_argument("--transition-timeout", type=float, default=25.0)
    parser.add_argument("--samples-per-path-unit", type=int, default=30)
    parser.add_argument("--min-segment-samples", type=int, default=8)
    parser.add_argument("--max-joint-speed", type=float, default=0.50)
    parser.add_argument("--min-sample-dt", type=float, default=0.03)
    parser.add_argument("--phase-start-hold", type=float, default=0.2)
    parser.add_argument("--box-rate", type=float, default=10.0)
    parser.add_argument("--joint-state-timeout", type=float, default=10.0)
    parser.add_argument("--joint-state-stale-timeout", type=float, default=5.0)
    parser.add_argument("--subscriber-timeout", type=float, default=5.0)
    parser.add_argument("--start-joint-speed", type=float, default=0.15)
    parser.add_argument("--start-samples-per-second", type=int, default=30)
    parser.add_argument("--min-start-duration", type=float, default=5.0)
    parser.add_argument("--initial-hold", type=float, default=1.5)
    parser.add_argument("--start-tolerance", type=float, default=0.04)
    parser.add_argument("--segment-tolerance", type=float, default=0.08)
    parser.add_argument("--execution-timeout-scale", type=float, default=6.0)
    parser.add_argument("--payload-sync-error", type=float, default=0.50)
    parser.add_argument("--payload-sync-lookahead", type=int, default=80)
    parser.add_argument("--payload-sync-report-period", type=float, default=5.0)
    parser.add_argument("--payload-final-snap-samples", type=int, default=6)
    parser.add_argument("--payload-pose-epsilon", type=float, default=1e-4)
    parser.add_argument(
        "--replace-box",
        action="store_true",
        help="Remove an existing Gazebo payload entity before spawning a fresh one.",
    )
    args = parser.parse_args()

    if args.plan_only and args.execute:
        parser.error("--plan-only and --execute are mutually exclusive")

    robot, problem, graph = build_problem(tuple(args.shuttle_pose))
    print("HPP manipulation scene initialized")
    print(f"config size: {robot.configSize()}")
    print(f"grippers: {mapping_names(robot.grippers())}")
    print(f"handles: {mapping_names(robot.handles())}")
    print(f"contact surfaces: {mapping_names(robot.contactSurfaces())}")
    print(f"graph: {GRAPH_NAME}")
    if args.build_only:
        return 0

    q_arm = np.asarray(args.q_start, dtype=float)
    q_shuttle_guess = box_configuration_from_world_pose(
        q_arm, shuttle_box_world_pose(tuple(args.shuttle_pose))
    )
    q_table_guess = box_configuration_from_world_pose(q_arm, table_box_world_pose())
    q_shuttle = project_free_configuration(problem, graph, q_shuttle_guess, "shuttle")
    q_table = project_free_configuration(problem, graph, q_table_guess, "table")

    segments = plan_manipulation(
        robot,
        problem,
        graph,
        q_shuttle,
        q_table,
        target_attempts=args.target_attempts,
        transition_iterations=args.transition_iterations,
        transition_timeout=args.transition_timeout,
    )
    format_plan(segments)

    if args.execute:
        execute_plan(robot, graph, segments, q_shuttle, q_table, args)
    else:
        build_execution_phases(robot, graph, segments, q_shuttle, q_table, args)
        print("pass --execute to publish the phases to Gazebo")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
