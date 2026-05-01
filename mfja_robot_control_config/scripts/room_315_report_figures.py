#!/usr/bin/env python3

import argparse
import math
from dataclasses import dataclass
from pathlib import Path
import re
from typing import Dict, Iterable, Sequence

import matplotlib

matplotlib.use('Agg')
import matplotlib.patheffects as path_effects
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Rectangle
import yaml

from room_315_kinematic_shuttle import (
    CUBIC_HERMITE_PATH_BACKEND,
    POLYLINE_PATH_BACKEND,
    Point3D,
    RailNetwork,
)


EXTERIOR_SEGMENTS = {'A1G', 'A2G', 'A3G', 'A4G', 'A12E', 'A34E'}
INTERIOR_SEGMENTS = {'A1S', 'A2S', 'A3S', 'A4S', 'A12I', 'A34I'}
TRUNK_SEGMENTS = {'A14', 'A23'}

EXTERIOR_COLOR = '#f28e2b'
INTERIOR_COLOR = '#edc948'
TRUNK_COLOR = '#2f4858'
RAW_POINT_COLOR = '#222222'
NETWORK_MUTED_COLOR = '#8b95a1'
STOPPER_COLOR = '#3366cc'
DZI_COLOR = '#d62728'
DA_MAIN_COLOR = '#2ca02c'
SWITCH_HALO_COLOR = '#c7d9bf'


@dataclass(frozen=True)
class PoseTransform:
    a: float = -0.893249246800
    b: float = 0.005839516878
    tx: float = -26.921427375871
    c: float = 0.001889497475
    d: float = 1.308619216904
    ty: float = 0.666926143808
    scale_x: float = 1.0
    scale_y: float = 1.0
    origin_x: float = -15.855195431322
    origin_y: float = -4.525523413467
    offset_x: float = 0.0
    offset_y: float = 0.0
    z_offset: float = 0.0

    def point(self, x: float, y: float, z: float = 0.0) -> tuple[float, float, float]:
        base_x = self.a * x + self.b * y + self.tx
        base_y = self.c * x + self.d * y + self.ty
        return (
            self.origin_x + (base_x - self.origin_x) * self.scale_x + self.offset_x,
            self.origin_y + (base_y - self.origin_y) * self.scale_y + self.offset_y,
            z + self.z_offset,
        )


SENSOR_LABEL_OFFSETS = {
    'DA1R': (-0.30, -0.10),
    'DA1GR': (-0.26, -0.12),
    'DA1SR': (-0.13, -0.18),
    'DA2R': (0.14, -0.10),
    'DA2GR': (0.15, -0.18),
    'DA2SR': (-0.14, 0.10),
    'DA3R': (0.42, 0.06),
    'DA3GR': (0.28, 0.11),
    'DA3SR': (0.40, -0.16),
    'DA4R': (-0.30, 0.10),
    'DA4GR': (-0.30, 0.07),
    'DA4SR': (-0.05, -0.11),
}

SENSOR_CONNECTION_STYLES = {
    'DA3R': 'arc3,rad=-0.18',
    'DA3SR': 'arc3,rad=0.18',
}

STOPPER_LABEL_OFFSETS = {
    ('A1', 'A14'): (-0.18, -0.10),
    ('A2', 'A12E'): (0.10, -0.12),
    ('A2', 'A12I'): (0.18, -0.10),
    ('A3', 'A23'): (-0.28, 0.24),
    ('A4', 'A34E'): (-0.17, 0.12),
    ('A4', 'A34I'): (-0.16, 0.12),
}

STOPPER_CONNECTION_STYLES = {
    ('A3', 'A23'): 'arc3,rad=-0.22',
}

SLOT_LABEL_OFFSETS = {
    '1': (-0.08, 0.12),
    '2': (0.07, 0.12),
    '3': (0.10, -0.12),
    '4': (-0.08, -0.12),
}

REPORT_MIRROR_X = True

DISPLAY_SWITCH_LABELS = {}

DISPLAY_SLOT_LABELS = {
    '1': '3',
    '2': '4',
    '3': '1',
    '4': '2',
}


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


def _default_output_dir() -> Path:
    return (
        _repo_root()
        / 'mfja_robot_control_config'
        / 'config'
        / 'room_315_kinematics'
        / 'report_figures'
    )


def _segment_color(segment_name: str) -> str:
    if segment_name in EXTERIOR_SEGMENTS:
        return EXTERIOR_COLOR
    if segment_name in INTERIOR_SEGMENTS:
        return INTERIOR_COLOR
    return TRUNK_COLOR


def _display_slot_label(slot_name: str) -> str:
    return DISPLAY_SLOT_LABELS.get(slot_name, slot_name)


def _display_switch_label(switch_name: str) -> str:
    return DISPLAY_SWITCH_LABELS.get(switch_name, switch_name)


def _display_sensor_label(sensor_name: str) -> str:
    match = re.fullmatch(r'(DA)([1-4])(.*)', sensor_name)
    if not match:
        return sensor_name
    prefix, index, suffix = match.groups()
    return f"{prefix}{_display_switch_label(f'A{index}')[1:]}{suffix}"


def _display_stopper_label(stopper_name: str) -> str:
    return stopper_name


DISPLAY_STOPPER_LABELS = {
    ('A1', 'A14'): 'A1',
    ('A2', 'A12E'): 'A2E',
    ('A2', 'A12I'): 'A2I',
    ('A3', 'A23'): 'A3',
    ('A4', 'A34E'): 'A4E',
    ('A4', 'A34I'): 'A4I',
}


def _display_segment_label(segment_name: str) -> str:
    match = re.fullmatch(r'A([1-4]+)(.*)', segment_name)
    if not match:
        return segment_name
    digits, suffix = match.groups()
    mapped_digits = ''.join(_display_switch_label(f'A{digit}')[1:] for digit in digits)
    return f'A{mapped_digits}{suffix}'


def _sample_segment_xy(
    network: RailNetwork,
    segment_name: str,
    transform: PoseTransform,
    step_m: float,
) -> list[tuple[float, float]]:
    segment = network.segments[segment_name]
    if segment.length <= 0.0:
        return []

    sample_count = max(2, int(math.ceil(segment.length / max(step_m, 1e-3))) + 1)
    sampled = []
    for index in range(sample_count):
        s = segment.length * index / (sample_count - 1)
        point, _yaw = segment.sample(s)
        x, y, _z = transform.point(point.x, point.y, point.z)
        sampled.append((x, y))
    return sampled


def _raw_segment_xy(
    network: RailNetwork,
    segment_name: str,
    transform: PoseTransform,
) -> list[tuple[float, float]]:
    points = []
    for point in network.segments[segment_name].points:
        x, y, _z = transform.point(point.x, point.y, point.z)
        points.append((x, y))
    return points


def _closest_network_position(
    network: RailNetwork,
    transform: PoseTransform,
    pose_xyz: Sequence[float],
) -> tuple[str, float, tuple[float, float]]:
    target_x, target_y, target_z = pose_xyz[:3]
    best_segment = ''
    best_s = 0.0
    best_distance = math.inf
    best_xy = (target_x, target_y)

    for segment_name, segment in network.segments.items():
        transformed_points = [
            transform.point(point.x, point.y, point.z)
            for point in segment.points
        ]
        for index, (previous, current) in enumerate(
            zip(transformed_points, transformed_points[1:])
        ):
            p0x, p0y, p0z = previous
            p1x, p1y, p1z = current
            vx = p1x - p0x
            vy = p1y - p0y
            vz = p1z - p0z
            edge_length_sq = vx * vx + vy * vy + vz * vz
            if edge_length_sq <= 1e-12:
                continue

            wx = target_x - p0x
            wy = target_y - p0y
            wz = target_z - p0z
            ratio = max(0.0, min(1.0, (wx * vx + wy * vy + wz * vz) / edge_length_sq))
            projected_x = p0x + ratio * vx
            projected_y = p0y + ratio * vy
            projected_z = p0z + ratio * vz
            distance = math.dist(
                (target_x, target_y, target_z),
                (projected_x, projected_y, projected_z),
            )
            if distance < best_distance:
                best_distance = distance
                best_segment = segment_name
                best_xy = (projected_x, projected_y)
                previous_s = segment.arc_lengths[index]
                current_s = segment.arc_lengths[index + 1]
                best_s = previous_s + ratio * (current_s - previous_s)

    if not best_segment:
        raise RuntimeError('Could not snap slot pose to the room 315 rail network.')
    return best_segment, best_s, best_xy


def _sensor_point_xy(
    sensor_name: str,
    sensor_config: dict,
    hermite_network: RailNetwork,
    transform: PoseTransform,
    start_slots: dict,
) -> tuple[float, float]:
    if 'slot' in sensor_config:
        slot = str(sensor_config['slot']).strip()
        slot_pose = start_slots[slot]['pose']
        segment_name, sensor_s, _xy = _closest_network_position(
            hermite_network,
            transform,
            slot_pose,
        )
        point, _yaw = hermite_network.segments[segment_name].sample(sensor_s)
        x, y, _z = transform.point(point.x, point.y, point.z)
        return x, y

    segment_name = str(sensor_config['segment']).strip()
    segment = hermite_network.segments[segment_name]
    if 's' in sensor_config:
        sensor_s = float(sensor_config['s'])
    else:
        offset_m = float(sensor_config.get('offset_m', 0.0))
        reference = str(sensor_config.get('reference', 'start')).strip().lower()
        if reference in {'start', 'begin', 'from_start'}:
            sensor_s = offset_m
        elif reference in {'end', 'finish', 'from_end', 'before_end'}:
            sensor_s = segment.length - offset_m
        else:
            raise ValueError(
                f'Position sensor {sensor_name} uses unsupported reference {reference!r}.'
            )
    sensor_s = max(0.0, min(sensor_s, segment.length))
    point, _yaw = segment.sample(sensor_s)
    x, y, _z = transform.point(point.x, point.y, point.z)
    return x, y


def _stopper_points(
    network_config: dict,
    hermite_network: RailNetwork,
    transform: PoseTransform,
    *,
    reverse_direction: bool = False,
) -> list[dict]:
    stopper_points: list[dict] = []
    for stopper_name, raw_config in (network_config.get('stoppers') or {}).items():
        default_stop_offset_m = float(raw_config.get('stop_offset_m', 0.08))
        raw_stop_points = raw_config.get('stop_points')
        if raw_stop_points is None:
            raw_stop_points = [
                {'segment': segment_name}
                for segment_name in raw_config.get('segments', [])
            ]

        for raw_stop_point in raw_stop_points:
            segment_name = str(raw_stop_point['segment']).strip()
            segment = hermite_network.segments[segment_name]
            if 's' in raw_stop_point:
                stop_s = float(raw_stop_point['s'])
                if reverse_direction:
                    stop_s = segment.length - stop_s
            else:
                stop_offset_m = float(
                    raw_stop_point.get('stop_offset_m', default_stop_offset_m)
                )
                if reverse_direction:
                    stop_s = stop_offset_m
                else:
                    stop_s = segment.length - stop_offset_m
            stop_s = max(0.0, min(stop_s, segment.length))
            point, _yaw = segment.sample(stop_s)
            x, y, _z = transform.point(point.x, point.y, point.z)
            stopper_points.append(
                {
                    'stopper': str(stopper_name).strip().upper(),
                    'segment': segment_name,
                    'x': x,
                    'y': y,
                }
            )
    return stopper_points


def _text_with_halo(axis, x: float, y: float, text: str, **kwargs) -> None:
    artist = axis.text(x, y, text, **kwargs)
    artist.set_path_effects(
        [path_effects.withStroke(linewidth=3.0, foreground='white', alpha=0.9)]
    )


def _style_axis(
    axis,
    bounds: tuple[float, float, float, float],
    *,
    padding_x: float = 0.18,
    padding_top: float = 0.18,
    padding_bottom: float = 0.18,
    mirror_x: bool = False,
) -> None:
    min_x, max_x, min_y, max_y = bounds
    axis.set_xlim(min_x - padding_x, max_x + padding_x)
    axis.set_ylim(min_y - padding_bottom, max_y + padding_top)
    if mirror_x:
        axis.invert_xaxis()
    axis.set_aspect('equal', adjustable='box')
    axis.set_xticks([])
    axis.set_yticks([])
    for spine in axis.spines.values():
        spine.set_visible(False)


def _draw_direction_arrows(
    axis,
    hermite_network: RailNetwork,
    transform: PoseTransform,
    segment_names: Iterable[str],
    color: str = '#111111',
    alpha: float = 0.7,
    reverse: bool = False,
) -> None:
    for segment_name in segment_names:
        segment = hermite_network.segments[segment_name]
        center_s = 0.5 * segment.length
        half_step = min(0.10, 0.25 * segment.length)
        start_point, _ = segment.sample(max(0.0, center_s - half_step))
        end_point, _ = segment.sample(min(segment.length, center_s + half_step))
        if reverse:
            start_point, end_point = end_point, start_point
        x0, y0, _ = transform.point(start_point.x, start_point.y, start_point.z)
        x1, y1, _ = transform.point(end_point.x, end_point.y, end_point.z)
        axis.annotate(
            '',
            xy=(x1, y1),
            xytext=(x0, y0),
            arrowprops={
                'arrowstyle': '->',
                'color': color,
                'lw': 1.3,
                'alpha': alpha,
                'shrinkA': 0.0,
                'shrinkB': 0.0,
            },
            zorder=4,
        )


def _plot_geometry_panel(
    axis,
    network_config: dict,
    polyline_network: RailNetwork,
    hermite_network: RailNetwork,
    transform: PoseTransform,
    bounds: tuple[float, float, float, float],
) -> None:
    for segment_name in network_config['segments']:
        raw_xy = _raw_segment_xy(polyline_network, segment_name, transform)
        hermite_xy = _sample_segment_xy(hermite_network, segment_name, transform, 0.01)
        color = _segment_color(segment_name)
        axis.plot(
            [point[0] for point in hermite_xy],
            [point[1] for point in hermite_xy],
            color=color,
            linewidth=2.8,
            alpha=0.95,
            solid_capstyle='round',
            zorder=2,
        )
        axis.plot(
            [point[0] for point in raw_xy],
            [point[1] for point in raw_xy],
            color=color,
            linewidth=0.9,
            alpha=0.22,
            linestyle='--',
            zorder=1,
        )
        axis.scatter(
            [point[0] for point in raw_xy],
            [point[1] for point in raw_xy],
            s=10,
            color=RAW_POINT_COLOR,
            alpha=0.75,
            zorder=3,
        )

        label_point = hermite_xy[len(hermite_xy) // 2]
        _text_with_halo(
            axis,
            label_point[0],
            label_point[1],
            _display_segment_label(segment_name),
            fontsize=8,
            weight='bold',
            color=color,
            ha='center',
            va='center',
            zorder=6,
        )

    for node_name, raw_node in network_config['nodes'].items():
        if not node_name.endswith('_C'):
            continue
        x, y, _z = transform.point(*raw_node['xyz'])
        axis.add_patch(
            Circle(
                (x, y),
                radius=0.11,
                facecolor=SWITCH_HALO_COLOR,
                edgecolor='none',
                alpha=0.5,
                zorder=0,
            )
        )
        _text_with_halo(
            axis,
            x,
            y + 0.01,
            _display_switch_label(node_name.split('_')[0]),
            fontsize=13,
            weight='bold',
            color='#324b2b',
            ha='center',
            va='center',
            zorder=7,
        )

    for slot_name, raw_slot in network_config['start_slots'].items():
        x, y, _z = raw_slot['pose'][:3]
        axis.add_patch(
            Rectangle(
                (x - 0.08, y - 0.022),
                width=0.16,
                height=0.044,
                facecolor='none',
                edgecolor='#2f4f9f',
                linewidth=2.1,
                zorder=5,
            )
        )
        dx, dy = SLOT_LABEL_OFFSETS[slot_name]
        display_slot = _display_slot_label(slot_name)
        _text_with_halo(
            axis,
            x + dx,
            y + dy,
            f'slot {display_slot}',
            fontsize=10,
            weight='bold',
            color='#2f4f9f',
            ha='center',
            va='center',
            zorder=7,
        )

    _draw_direction_arrows(
        axis,
        hermite_network,
        transform,
        ['A14', 'A23', 'A12E', 'A34E'],
        reverse=REPORT_MIRROR_X,
    )
    _style_axis(axis, bounds, mirror_x=REPORT_MIRROR_X)
    axis.set_title('A. Raw CSV points and cubic Hermite path', fontsize=14, weight='bold')

    legend_handles = [
        Line2D([0], [0], color=TRUNK_COLOR, lw=3, label='Shared trunk'),
        Line2D([0], [0], color=EXTERIOR_COLOR, lw=3, label='Exterior family'),
        Line2D([0], [0], color=INTERIOR_COLOR, lw=3, label='Interior family'),
        Line2D(
            [0],
            [0],
            marker='o',
            color='none',
            markerfacecolor=RAW_POINT_COLOR,
            markersize=5,
            label='Measured CSV points',
        ),
        Line2D([0], [0], color='#2f4f9f', lw=2, label='Start slots'),
    ]
    axis.legend(
        handles=legend_handles,
        loc='lower center',
        bbox_to_anchor=(0.5, -0.06),
        ncol=3,
        frameon=False,
        fontsize=9,
    )


def _plot_sensor_panel(
    axis,
    network_config: dict,
    hermite_network: RailNetwork,
    transform: PoseTransform,
    bounds: tuple[float, float, float, float],
) -> None:
    for segment_name in network_config['segments']:
        hermite_xy = _sample_segment_xy(hermite_network, segment_name, transform, 0.01)
        axis.plot(
            [point[0] for point in hermite_xy],
            [point[1] for point in hermite_xy],
            color=NETWORK_MUTED_COLOR,
            linewidth=2.0,
            alpha=0.35,
            zorder=0,
        )

    for node_name, raw_node in network_config['nodes'].items():
        if not node_name.endswith('_C'):
            continue
        x, y, _z = transform.point(*raw_node['xyz'])
        axis.add_patch(
            Circle(
                (x, y),
                radius=0.13,
                facecolor=SWITCH_HALO_COLOR,
                edgecolor='none',
                alpha=0.55,
                zorder=1,
            )
        )
        _text_with_halo(
            axis,
            x,
            y + 0.01,
            _display_switch_label(node_name.split('_')[0]),
            fontsize=13,
            weight='bold',
            color='#35512f',
            ha='center',
            va='center',
            zorder=6,
        )

    _draw_direction_arrows(
        axis,
        hermite_network,
        transform,
        ['A14', 'A23', 'A12E', 'A34E'],
        color='#333333',
        alpha=0.55,
        reverse=REPORT_MIRROR_X,
    )

    start_slots = network_config['start_slots']
    for sensor_name, sensor_config in (network_config.get('position_sensors') or {}).items():
        sensor_x, sensor_y = _sensor_point_xy(
            sensor_name,
            sensor_config,
            hermite_network,
            transform,
            start_slots,
        )
        kind = str(sensor_config.get('kind', '')).strip().lower()
        branch = str(sensor_config.get('branch', '')).strip().upper()
        aliases = sensor_config.get('aliases', []) or []

        if kind == 'indexing_zone':
            slot = str(sensor_config['slot']).strip()
            slot_x, slot_y, _slot_z = start_slots[slot]['pose'][:3]
            axis.add_patch(
                Rectangle(
                    (slot_x - 0.075, slot_y - 0.02),
                    width=0.15,
                    height=0.04,
                    facecolor='none',
                    edgecolor='#2f4f9f',
                    linewidth=2.0,
                    zorder=4,
                )
            )
            axis.scatter(
                [sensor_x],
                [sensor_y],
                s=78,
                marker='s',
                color=DZI_COLOR,
                edgecolors='white',
                linewidths=0.8,
                zorder=5,
            )
            dx, dy = SLOT_LABEL_OFFSETS[slot]
            display_slot = _display_slot_label(slot)
            _text_with_halo(
                axis,
                slot_x + dx,
                slot_y + dy,
                f'slot {display_slot}\nDZI{display_slot}R',
                fontsize=9,
                weight='bold',
                color=DZI_COLOR,
                ha='center',
                va='center',
                zorder=7,
            )
            continue

        if kind == 'switch_main':
            color = DA_MAIN_COLOR
        elif branch == 'G':
            color = EXTERIOR_COLOR
        else:
            color = INTERIOR_COLOR

        axis.scatter(
            [sensor_x],
            [sensor_y],
            s=70,
            marker='o',
            color=color,
            edgecolors='white',
            linewidths=0.9,
            zorder=5,
        )
        offset_x, offset_y = SENSOR_LABEL_OFFSETS[sensor_name]
        label = _display_sensor_label(sensor_name)
        if 'P' in aliases:
            label = f'{_display_sensor_label(sensor_name)}\n(alias {aliases[-1]})'
        axis.annotate(
            label,
            xy=(sensor_x, sensor_y),
            xytext=(sensor_x + offset_x, sensor_y + offset_y),
            fontsize=8.5,
            weight='bold',
            color=color,
            ha='center',
            va='center',
            bbox={
                'boxstyle': 'round,pad=0.18',
                'fc': 'white',
                'ec': color,
                'alpha': 0.92,
                'lw': 0.8,
            },
            arrowprops={
                'arrowstyle': '-',
                'color': color,
                'lw': 1.0,
                'alpha': 0.9,
                'connectionstyle': SENSOR_CONNECTION_STYLES.get(sensor_name, 'arc3,rad=0.0'),
            },
            zorder=7,
        )

    for stopper_point in _stopper_points(
        network_config,
        hermite_network,
        transform,
        reverse_direction=REPORT_MIRROR_X,
    ):
        x = stopper_point['x']
        y = stopper_point['y']
        axis.scatter(
            [x],
            [y],
            s=65,
            marker='D',
            color=STOPPER_COLOR,
            edgecolors='white',
            linewidths=0.9,
            zorder=5,
        )
        dx, dy = STOPPER_LABEL_OFFSETS[(stopper_point['stopper'], stopper_point['segment'])]
        display_stopper = DISPLAY_STOPPER_LABELS[
            (stopper_point['stopper'], stopper_point['segment'])
        ]
        stopper_label = f'STP {display_stopper}'

        axis.annotate(
            stopper_label,
            xy=(x, y),
            xytext=(x + dx, y + dy),
            fontsize=7.6,
            color=STOPPER_COLOR,
            weight='bold',
            ha='center',
            va='center',
            bbox={
                'boxstyle': 'round,pad=0.12',
                'fc': 'white',
                'ec': STOPPER_COLOR,
                'alpha': 0.90,
                'lw': 0.7,
            },
            arrowprops={
                'arrowstyle': '-',
                'color': STOPPER_COLOR,
                'lw': 0.85,
                'alpha': 0.85,
                'connectionstyle': STOPPER_CONNECTION_STYLES.get(
                    (stopper_point['stopper'], stopper_point['segment']),
                    'arc3,rad=0.0',
                ),
            },
            zorder=7,
        )

    _style_axis(axis, bounds, padding_bottom=0.40, mirror_x=REPORT_MIRROR_X)
    axis.set_title('B. Sensors, stoppers, and operational points', fontsize=14, weight='bold')

    legend_handles = [
        Line2D([0], [0], marker='s', color='none', markerfacecolor=DZI_COLOR, markersize=8, label='DZI*R indexing sensor'),
        Line2D([0], [0], marker='o', color='none', markerfacecolor=DA_MAIN_COLOR, markersize=8, label='DA*R main detector'),
        Line2D([0], [0], marker='o', color='none', markerfacecolor=EXTERIOR_COLOR, markersize=8, label='DA*GR exterior branch'),
        Line2D([0], [0], marker='o', color='none', markerfacecolor=INTERIOR_COLOR, markersize=8, label='DA*SR interior branch'),
        Line2D([0], [0], marker='D', color='none', markerfacecolor=STOPPER_COLOR, markersize=7, label='Stopper stop point'),
        Line2D([0], [0], color='#2f4f9f', lw=2, label='Corrected start-slot numbering'),
    ]
    axis.legend(
        handles=legend_handles,
        loc='lower center',
        bbox_to_anchor=(0.5, -0.02),
        ncol=2,
        frameon=False,
        fontsize=8.8,
    )


def _compute_bounds(
    network_config: dict,
    hermite_network: RailNetwork,
    transform: PoseTransform,
) -> tuple[float, float, float, float]:
    xs: list[float] = []
    ys: list[float] = []
    for segment_name in network_config['segments']:
        for x, y in _sample_segment_xy(hermite_network, segment_name, transform, 0.01):
            xs.append(x)
            ys.append(y)
    return min(xs), max(xs), min(ys), max(ys)


def _save_figure(figure, output_dir: Path, stem: str) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    for suffix in ('.png', '.svg', '.pdf'):
        figure.savefig(output_dir / f'{stem}{suffix}', dpi=240, bbox_inches='tight')


def _load_network_config(network_path: Path) -> dict:
    with network_path.open() as handle:
        return yaml.safe_load(handle)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Generate report-ready figures for the room 315 rail geometry and sensors.'
    )
    parser.add_argument(
        '--network',
        type=Path,
        default=_default_network_path(),
        help='Path to rail_network.yaml.',
    )
    parser.add_argument(
        '--output-dir',
        type=Path,
        default=_default_output_dir(),
        help='Directory where the report figures will be written.',
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    network_config = _load_network_config(args.network)
    transform = PoseTransform()
    polyline_network = RailNetwork.from_yaml(
        args.network,
        path_backend=POLYLINE_PATH_BACKEND,
    )
    hermite_network = RailNetwork.from_yaml(
        args.network,
        path_backend=CUBIC_HERMITE_PATH_BACKEND,
        arc_length_samples_per_edge=16,
    )
    bounds = _compute_bounds(network_config, hermite_network, transform)

    summary_figure, axes = plt.subplots(
        1,
        2,
        figsize=(16.5, 8.8),
        constrained_layout=True,
    )
    _plot_geometry_panel(
        axes[0],
        network_config,
        polyline_network,
        hermite_network,
        transform,
        bounds,
    )
    _plot_sensor_panel(
        axes[1],
        network_config,
        hermite_network,
        transform,
        bounds,
    )
    summary_figure.suptitle(
        'Room 315 shuttle report figure: geometry, Hermite interpolation, and right-rail sensors',
        fontsize=16,
        weight='bold',
    )
    _save_figure(summary_figure, args.output_dir, 'room_315_report_summary')
    plt.close(summary_figure)

    sensor_figure, sensor_axis = plt.subplots(
        1,
        1,
        figsize=(10.4, 9.4),
        constrained_layout=True,
    )
    _plot_sensor_panel(
        sensor_axis,
        network_config,
        hermite_network,
        transform,
        bounds,
    )
    sensor_axis.set_title(
        'Room 315 right-rail sensor and stopper layout',
        fontsize=16,
        weight='bold',
    )
    _save_figure(sensor_figure, args.output_dir, 'room_315_report_sensor_layout')
    plt.close(sensor_figure)

    print(f'Wrote report figures to: {args.output_dir}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
