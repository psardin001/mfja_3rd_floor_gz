"""Check the installed gears modules and models without opening a robot connection."""

import inspect
import os
import sys
from pathlib import Path

import hpp_exec
import pyhpp_toppra
import pyhpp_viser
from ament_index_python.packages import get_package_share_path
from industrial_msgs.msg import RobotStatus
from pinocchio import SE3
from pyhpp.core import InterpolatedPath, Progressive
from pyhpp.manipulation import Device, urdf
from staubli_msgs.srv import WriteSingleIO

sys.path.insert(0, str(Path(os.environ["HPP_TUTORIAL_DIR"]) / "mfja"))
from environment import initial_configuration, load_scene

assert sys.version_info[:2] == (3, 12)
assert "positions_only" in inspect.signature(hpp_exec.execute_segments).parameters
assert "wait_for_completion" in inspect.signature(hpp_exec.execute_segments).parameters
assert "velocities" in inspect.signature(hpp_exec.execute_segments).parameters
assert Progressive.timeOut and InterpolatedPath.interpolationPoints
assert pyhpp_toppra.Toppra and pyhpp_viser.Viewer and WriteSingleIO
assert "trajectory_complete" in RobotStatus.get_fields_and_field_types()
robot = Device("installation_check")
load_scene(robot)
for name in ("gear_42_1", "gear_42_2"):
    urdf.loadModel(
        robot,
        0,
        name,
        "freeflyer",
        "package://mfja_3rd_floor_description/urdf/gear_42.urdf",
        "package://mfja_3rd_floor_description/srdf/gear_42.srdf",
        SE3.Identity(),
    )
assert len(initial_configuration(robot)) == 20
print(f"Python {sys.version.split()[0]}: HPP, TOPPRA, Viser and hpp-exec ready")
print(f"MFJA models: {get_package_share_path('mfja_3rd_floor_description')}")
print(f"Stäubli driver: {get_package_share_path('staubli_val3_driver')}")
print("Room 315 and both gears loaded; no robot connection opened.")
