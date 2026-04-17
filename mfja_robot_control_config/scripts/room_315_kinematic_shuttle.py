#!/usr/bin/env python3

import argparse
import csv
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import yaml


MOVING = 'MOVING'
WAITING = 'WAITING'
FALLING = 'FALLING'


@dataclass(frozen=True)
class Point3D:
    x: float
    y: float
    z: float

    def distance_to(self, other: 'Point3D') -> float:
        return math.dist((self.x, self.y, self.z), (other.x, other.y, other.z))


@dataclass
class ShuttleState:
    current_segment: str
    s: float
    speed: float
    mode: str = MOVING


@dataclass(frozen=True)
class ShuttlePose:
    x: float
    y: float
    z: float
    yaw: float
    current_segment: str
    s: float
    mode: str


class SegmentGeometry:
    def __init__(self, name: str, points: Sequence[Point3D]) -> None:
        if len(points) < 2:
            raise ValueError(f'{name} must contain at least two points')
        self.name = name
        self.points = list(points)
        self.arc_lengths = self._compute_arc_lengths(self.points)
        if self.length <= 0.0:
            raise ValueError(f'{name} has zero total length')

    @staticmethod
    def _compute_arc_lengths(points: Sequence[Point3D]) -> List[float]:
        arc_lengths = [0.0]
        total = 0.0
        for previous, current in zip(points, points[1:]):
            total += previous.distance_to(current)
            arc_lengths.append(total)
        return arc_lengths

    @property
    def length(self) -> float:
        return self.arc_lengths[-1]

    def sample(self, s: float) -> tuple[Point3D, float]:
        clamped_s = max(0.0, min(s, self.length))
        for index in range(1, len(self.arc_lengths)):
            previous_s = self.arc_lengths[index - 1]
            current_s = self.arc_lengths[index]
            if clamped_s > current_s and index < len(self.arc_lengths) - 1:
                continue

            previous = self.points[index - 1]
            current = self.points[index]
            edge_length = current_s - previous_s
            ratio = 0.0 if edge_length == 0.0 else (clamped_s - previous_s) / edge_length
            x = previous.x + ratio * (current.x - previous.x)
            y = previous.y + ratio * (current.y - previous.y)
            z = previous.z + ratio * (current.z - previous.z)
            yaw = math.atan2(current.y - previous.y, current.x - previous.x)
            return Point3D(x=x, y=y, z=z), yaw

        previous = self.points[-2]
        current = self.points[-1]
        yaw = math.atan2(current.y - previous.y, current.x - previous.x)
        return current, yaw


class RailNetwork:
    def __init__(self, network_path: Path, config: dict, segments: Dict[str, SegmentGeometry]) -> None:
        self.network_path = network_path
        self.config = config
        self.segments = segments
        self.routing_table = config.get('routing_table', {})
        self.switches = config.get('switches', {})
        self.valid_switch_states = set(config.get('switch_state_space', {}).get('values', []))

    @classmethod
    def from_yaml(cls, network_path: Path) -> 'RailNetwork':
        network_path = network_path.resolve()
        with network_path.open() as handle:
            config = yaml.safe_load(handle)

        segments = {
            name: SegmentGeometry(
                name=name,
                points=_read_csv_points(_resolve_path(segment_config['csv'], network_path)),
            )
            for name, segment_config in config['segments'].items()
        }
        return cls(network_path=network_path, config=config, segments=segments)

    def default_switch_states(self) -> Dict[str, str]:
        return {
            switch_name: switch_config.get('states', ['G'])[0]
            for switch_name, switch_config in self.switches.items()
        }

    def normalized_switch_state(self, raw_state: str) -> str:
        state = raw_state.strip().upper()
        if state not in self.valid_switch_states:
            raise ValueError(
                f'Unknown switch state {raw_state!r}; expected one of {sorted(self.valid_switch_states)}'
            )
        return state

    def resolve_successor(
        self,
        current_segment: str,
        switch_states: Dict[str, str],
    ) -> Optional[str]:
        rule = self.routing_table.get(current_segment)
        if rule is None:
            return None

        rule_type = rule.get('type')
        if rule_type == 'fixed':
            next_segment = rule.get('next_segment')
            return None if next_segment == FALLING else next_segment

        switch_name = rule.get('switch')
        switch_state = switch_states.get(switch_name)
        if switch_state is None:
            return None

        normalized_state = self.normalized_switch_state(switch_state)
        next_segment = rule.get('by_state', {}).get(normalized_state, rule.get('on_unknown_state'))
        if next_segment == FALLING:
            return None
        return next_segment


class KinematicShuttleCore:
    def __init__(self, network: RailNetwork, initial_state: ShuttleState) -> None:
        if initial_state.current_segment not in network.segments:
            raise ValueError(f'Unknown initial segment: {initial_state.current_segment}')
        self.network = network
        self.state = initial_state

    def pose(self) -> ShuttlePose:
        segment = self.network.segments[self.state.current_segment]
        point, yaw = segment.sample(self.state.s)
        return ShuttlePose(
            x=point.x,
            y=point.y,
            z=point.z,
            yaw=yaw,
            current_segment=self.state.current_segment,
            s=self.state.s,
            mode=self.state.mode,
        )

    def step(self, dt: float, switch_states: Optional[Dict[str, str]] = None) -> ShuttlePose:
        if dt < 0.0:
            raise ValueError('dt must be non-negative')

        switch_states = switch_states or self.network.default_switch_states()
        if self.state.mode == FALLING:
            return self.pose()

        if self.state.speed <= 0.0:
            self.state.mode = WAITING
            return self.pose()

        if self.state.mode == WAITING:
            self.state.mode = MOVING

        remaining_distance = self.state.speed * dt
        while remaining_distance > 0.0 and self.state.mode == MOVING:
            segment = self.network.segments[self.state.current_segment]
            distance_to_end = max(0.0, segment.length - self.state.s)

            if remaining_distance < distance_to_end:
                self.state.s += remaining_distance
                remaining_distance = 0.0
                break

            remaining_distance -= distance_to_end
            self.state.s = segment.length
            successor = self.network.resolve_successor(self.state.current_segment, switch_states)
            if successor is None or successor not in self.network.segments:
                self.state.mode = FALLING
                break

            self.state.current_segment = successor
            self.state.s = 0.0

        return self.pose()


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _default_network_path() -> Path:
    return (
        _repo_root()
        / 'mfja_robot_control_config'
        / 'config'
        / 'room_315_kinematics'
        / 'rail_network.yaml'
    )


def _resolve_path(raw_path: str, network_path: Path) -> Path:
    path = Path(raw_path)
    if path.is_absolute():
        return path

    repo_relative = _repo_root() / path
    if repo_relative.exists():
        return repo_relative
    return network_path.parent / path


def _read_csv_points(csv_path: Path) -> List[Point3D]:
    with csv_path.open(newline='') as handle:
        reader = csv.DictReader(handle)
        fieldnames = set(reader.fieldnames or [])
        required = {'index', 'x', 'y', 'z'}
        missing = required - fieldnames
        if missing:
            raise ValueError(f'{csv_path} is missing required columns: {sorted(missing)}')
        return [
            Point3D(x=float(row['x']), y=float(row['y']), z=float(row['z']))
            for row in reader
        ]


def _parse_switch_states(network: RailNetwork, raw_values: Sequence[str]) -> Dict[str, str]:
    switch_states = network.default_switch_states()
    for raw_value in raw_values:
        for token in raw_value.replace(',', ' ').split():
            if '=' not in token:
                raise ValueError(f'Switch assignment must look like A1=G, got {token!r}')
            switch_name, raw_state = token.split('=', 1)
            switch_name = switch_name.strip().upper()
            if switch_name not in network.switches:
                raise ValueError(f'Unknown switch {switch_name!r}; expected one of {sorted(network.switches)}')
            switch_states[switch_name] = network.normalized_switch_state(raw_state)
    return switch_states


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Run the room 315 one-shuttle kinematic core without ROS or Gazebo.'
    )
    parser.add_argument(
        '--network',
        type=Path,
        default=_default_network_path(),
        help='rail_network.yaml path.',
    )
    parser.add_argument(
        '--segment',
        default='A14',
        help='Initial segment name.',
    )
    parser.add_argument(
        '--s',
        type=float,
        default=0.0,
        help='Initial arc-length progress on the segment.',
    )
    parser.add_argument(
        '--speed',
        type=float,
        default=0.25,
        help='Shuttle speed in m/s.',
    )
    parser.add_argument(
        '--duration',
        type=float,
        default=10.0,
        help='Simulation duration in seconds.',
    )
    parser.add_argument(
        '--dt',
        type=float,
        default=0.05,
        help='Simulation time step in seconds.',
    )
    parser.add_argument(
        '--switch',
        action='append',
        default=[],
        help='Switch assignment such as A1=G. Can be repeated or comma/space separated.',
    )
    parser.add_argument(
        '--trace-every',
        type=float,
        default=1.0,
        help='Print one pose sample every N seconds. Use 0 for final state only.',
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    network = RailNetwork.from_yaml(args.network)
    switch_states = _parse_switch_states(network, args.switch)
    core = KinematicShuttleCore(
        network=network,
        initial_state=ShuttleState(
            current_segment=args.segment,
            s=args.s,
            speed=args.speed,
            mode=MOVING,
        ),
    )

    elapsed = 0.0
    next_trace_time = 0.0
    while elapsed < args.duration:
        pose = core.step(args.dt, switch_states=switch_states)
        elapsed += args.dt
        if args.trace_every > 0.0 and elapsed >= next_trace_time:
            print(json.dumps({'t': elapsed, **asdict(pose)}, sort_keys=True))
            next_trace_time += args.trace_every
        if pose.mode == FALLING:
            break

    final_pose = core.pose()
    print(json.dumps({'final': True, 't': elapsed, **asdict(final_pose)}, sort_keys=True))
    return 0 if final_pose.mode != FALLING else 2


if __name__ == '__main__':
    raise SystemExit(main())
