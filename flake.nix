{
  description = "Development shell for the MFJA 3rd floor ROS 2 Jazzy / Gazebo Harmonic simulation";

  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
  };

  outputs = { nix-ros-overlay, nixpkgs, ... }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };

        ros = pkgs.rosPackages.jazzy;

        rosEnv = with ros; buildEnv {
          paths = [
            ros-base
            ros2launch

            ament-cmake
            ament-index-python
            ament-lint-auto
            ament-lint-common
            rosidl-default-generators
            rosidl-default-runtime

            rcl-interfaces
            rclpy
            geometry-msgs
            nav-msgs
            rosgraph-msgs
            sensor-msgs
            std-msgs
            tf2-msgs
            tf2-ros
            trajectory-msgs

            robot-state-publisher
            ros-gz
            ros-gz-bridge
            ros-gz-interfaces
            ros-gz-sim

            rmw-cyclonedds-cpp
          ];
        };

        pythonEnv = pkgs.python3.withPackages (ps: with ps; [
          pyyaml
          setuptools
        ]);
      in
      {
        devShells.default = pkgs.mkShell {
          name = "mfja-ros2-jazzy-gz-harmonic";

          packages = [
            pkgs.cmake
            pkgs.colcon
            pkgs.gcc
            pkgs.git
            pkgs.gnumake
            pkgs.ninja
            pkgs.pkg-config
            pythonEnv
            rosEnv
          ];

          shellHook = ''
            export ROS_DISTRO=jazzy

            echo "Entered MFJA ROS 2 Jazzy / Gazebo Harmonic Nix shell."
            echo "Next steps: colcon build --symlink-install && source install/setup.bash"
          '';
        };
      });

  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [
      "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo="
    ];
  };
}
