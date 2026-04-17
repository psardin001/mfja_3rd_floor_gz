Room 315 kinematic shuttle artifacts live here.

Contents:
- `rail_network.yaml`: directed topology with explicit nodes, switches, fixed transitions, and routing rules.
- `normalized_segments/`: per-segment normalized CSV files generated from the raw `CSV/` directory.
- `segment_summary.yaml`: preprocessing summary with counts, endpoints, lengths, and bounding boxes.
- `validation_report.yaml`: network validation report with lengths, snap distances, gaps, and tangent checks.
- `debug_plots/`: visual plots for inspecting the raw segments and the extracted network.

Phase 1 preprocessing:
- Read raw per-segment CSV geometry.
- Remove consecutive duplicate points.
- Re-index points.
- Compute cumulative arc length and local tangent/yaw per point.

Phase 2 network extraction:
- Encode the rail network as an explicit directed graph.
- Use the 12 nodes `A1_C`, `A1_G`, `A1_S`, `A2_C`, `A2_G`, `A2_S`,
  `A3_C`, `A3_G`, `A3_S`, `A4_C`, `A4_G`, and `A4_S`.
- Snap segment endpoints to nodes with `snap_tolerance_m: 0.05`.
- Assume one-way motion on every segment from CSV index `0` to the last row.

Phase 3 validation:
- Generate `validation_report.yaml`.
- Generate `debug_plots/network_validation.png` with segment and node labels.

Phase 4 kinematic core:
- Run one shuttle on the explicit graph without ROS or Gazebo.
- If a segment has no valid successor, the shuttle enters `FALLING`.

Phase 5 ROS 2 first node:
- `room_315_kinematic_shuttle_node.py` publishes a kinematic pose and state.
- Switch states are commanded as text messages such as `A1=G A2=S`.

Useful commands:

```bash
python3 mfja_robot_control_config/scripts/room_315_csv_preprocessor.py
python3 mfja_robot_control_config/scripts/room_315_network_validator.py
python3 mfja_robot_control_config/scripts/room_315_kinematic_shuttle.py --switch A1=G --switch A2=G --switch A3=G --switch A4=G
```
