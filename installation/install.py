#!/usr/bin/env python3
"""Install the pinned HPP and Stäubli stack for the gears tutorials."""

import argparse
import os
import shlex
import shutil
import subprocess
import sys
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
MFJA = HERE.parent

OPTIONS = {
    "coal": [
        "-DBUILD_PYTHON_INTERFACE=ON",
        "-DCOAL_PYTHON_NANOBIND=OFF",
        "-DCOAL_HAS_QHULL=ON",
        "-DCOAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL=ON",
    ],
    "pinocchio": [
        "-DBUILD_PYTHON_INTERFACE=ON",
        "-DBUILD_WITH_COLLISION_SUPPORT=ON",
        "-DBUILD_EXAMPLES=OFF",
    ],
    "proxsuite": [
        "-DBUILD_WITH_VECTORIZATION_SUPPORT=OFF",
        "-DBUILD_PYTHON_INTERFACE=OFF",
    ],
    "example-robot-data": ["-DBUILD_PYTHON_INTERFACE=OFF"],
    "hpp-constraints": ["-DUSE_QPOASES=OFF"],
    "toppra": ["-DBUILD_TESTS=OFF", "-DPYTHON_BINDINGS=OFF"],
    "hpp-gepetto-viewer": ["-DUSE_HPP_PYTHON=ON", "-DUSE_CORBA=OFF"],
}
ROS_PACKAGES = [
    "industrial_msgs",
    "simple_message",
    "urdf_extention",
    "industrial_utils",
    "industrial_robot_client",
    "staubli_msgs",
    "staubli_tx2_60l_description",
    "staubli_val3_driver",
    "mfja_3rd_floor_description",
    "mfja_robot_control_config",
    "mfja_staubli_manipulation_demos",
]


def run(*args, **kwargs):
    subprocess.run([str(arg) for arg in args], check=True, **kwargs)


def git(path, *args):
    return subprocess.check_output(["git", "-C", str(path), *args], text=True).strip()


def install(workspace):
    workspace.mkdir(parents=True, exist_ok=True)
    source = workspace / "hpp/src"
    prefix = workspace / "hpp/install"
    logs = workspace / "logs"
    source.mkdir(parents=True, exist_ok=True)
    logs.mkdir(exist_ok=True)
    repositories = yaml.safe_load((HERE / "sources.repos").read_text())["repositories"]
    for name, repository in repositories.items():
        path = source / name
        if not path.exists():
            run("git", "init", "-q", path)
            run("git", "-C", path, "remote", "add", "origin", repository["url"])
        head = subprocess.run(
            ["git", "-C", str(path), "rev-parse", "--verify", "HEAD"],
            capture_output=True,
        )
        if head.returncode:
            run(
                "git", "-C", path, "fetch", "--depth=1", "origin", repository["version"]
            )
            run("git", "-C", path, "checkout", "--detach", repository["version"])
        if git(path, "rev-parse", "HEAD") != repository["version"]:
            raise RuntimeError(f"{path}: revision differs from sources.repos")
        if git(path, "status", "--porcelain", "--untracked-files=no"):
            raise RuntimeError(
                f"{path}: preserve local source changes before installing"
            )
        if name in ("coal", "pinocchio", "hpp-exec", "proxsuite"):
            module = "cmake-module" if name == "proxsuite" else "cmake"
            run("git", "-C", path, "submodule", "update", "--init", module)

    driver = MFJA / "Staubli_ROS2"
    expected = git(MFJA, "ls-tree", "HEAD", "Staubli_ROS2").split()[2]
    if not (driver / ".git").exists():
        run("git", "-C", MFJA, "submodule", "update", "--init", "--recursive")
    if git(driver, "rev-parse", "HEAD") != expected or git(
        driver, "status", "--porcelain"
    ):
        raise RuntimeError("Staubli_ROS2 differs from the pinned submodule")

    python = workspace / ".venv/bin/python"
    if not python.exists():
        run(
            "/usr/bin/python3",
            "-m",
            "venv",
            "--system-site-packages",
            workspace / ".venv",
        )
    run(python, "-m", "pip", "install", "-r", HERE / "requirements.txt")
    setup = (
        (HERE / "setup.bash.in")
        .read_text()
        .replace("@WORKSPACE@", shlex.quote(str(workspace)))
        .replace("@MFJA_ROOT@", shlex.quote(str(MFJA)))
    )
    (workspace / "setup.bash").write_text(setup)
    env = dict(os.environ)
    env.update(
        dict(
            line.split("=", 1)
            for line in subprocess.check_output(
                [
                    "bash",
                    "-ec",
                    'source "$1" >/dev/null; env -0',
                    "bash",
                    str(workspace / "setup.bash"),
                ]
            )
            .decode()
            .strip("\0")
            .split("\0")
        )
    )
    env["CMAKE_BUILD_PARALLEL_LEVEL"] = "1"
    env["MAKEFLAGS"] = "-j1 -l1"
    common = [
        "-DCMAKE_BUILD_TYPE=Release",
        "-DCMAKE_CXX_FLAGS_RELEASE=-O2 -DNDEBUG",
        f"-DCMAKE_INSTALL_PREFIX={prefix}",
        f"-DPYTHON_EXECUTABLE={python}",
        f"-DPython_EXECUTABLE={python}",
        f"-DPython3_EXECUTABLE={python}",
        "-DPYTHON_STANDARD_LAYOUT=ON",
        "-DBUILD_TESTING=OFF",
        "-DBUILD_DOCUMENTATION=OFF",
        "-DINSTALL_DOCUMENTATION=OFF",
        "-DGENERATE_PYTHON_STUBS=OFF",
        "-DENABLE_PYTHON_DOXYGEN_AUTODOC=OFF",
    ]
    for name in repositories:
        if name == "hpp_tutorial":
            continue
        print(f"Building {name} (logs/{name}.log)", flush=True)
        directory = source / name / "cpp" if name == "toppra" else source / name
        build = workspace / "hpp/build" / name
        with (logs / f"{name}.log").open("w") as log:
            run(
                "cmake",
                "-S",
                directory,
                "-B",
                build,
                *common,
                *OPTIONS.get(name, []),
                env=env,
                stdout=log,
                stderr=subprocess.STDOUT,
            )
            run(
                "cmake",
                "--build",
                build,
                "--parallel",
                "1",
                "--target",
                "install",
                env=env,
                stdout=log,
                stderr=subprocess.STDOUT,
            )

    print("Building MFJA resources and Stäubli driver", flush=True)
    ros = workspace / "ros"
    ros.mkdir(exist_ok=True)
    paths = [
        driver / name if (driver / name).exists() else MFJA / name
        for name in ROS_PACKAGES
    ]
    run(
        "colcon",
        "--log-base",
        ros / "log",
        "build",
        "--base-paths",
        *paths,
        "--build-base",
        ros / "build",
        "--install-base",
        ros / "install",
        "--merge-install",
        "--symlink-install",
        "--executor",
        "sequential",
        "--packages-select",
        *ROS_PACKAGES,
        "--cmake-args",
        "-DBUILD_TESTING=OFF",
        "-DPython3_EXECUTABLE=/usr/bin/python3",
        "-DPYTHON_EXECUTABLE=/usr/bin/python3",
        env=env,
        cwd=ros,
    )
    run(
        "bash",
        "-ec",
        'source "$1"; python "$2"',
        "bash",
        workspace / "setup.bash",
        HERE / "check.py",
        env=env,
    )
    shutil.copyfile(HERE / "sources.repos", workspace / "sources.repos")
    (workspace / "mfja-revision.txt").write_text(git(MFJA, "rev-parse", "HEAD") + "\n")
    print(f"Ready: source {shlex.quote(str(workspace / 'setup.bash'))}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("workspace", type=Path, help="new installation directory")
    args = parser.parse_args()
    if sys.version_info[:2] != (3, 12):
        parser.error("Python 3.12 required")
    # Build against ROS Jazzy and this prefix, independent of sourced overlays.
    if os.environ.get("MFJA_GEARS_INSTALL_ENV") != "1":
        env = {
            key: os.environ[key]
            for key in ("HOME", "USER", "LANG", "TERM")
            if key in os.environ
        }
        env.update(
            PATH="/usr/bin:/bin", PYTHONNOUSERSITE="1", MFJA_GEARS_INSTALL_ENV="1"
        )
        os.execve(
            "/usr/bin/python3",
            ["python3", str(Path(__file__).resolve()), str(args.workspace.resolve())],
            env,
        )
    install(args.workspace.resolve())


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, subprocess.CalledProcessError) as error:
        raise SystemExit(str(error)) from error
