# ros2_robotiq_gripper

This repository contains the ROS 2 driver, controller and description packages for working with a Robotiq Gripper.

The goal is to support multiple Robotiq Grippers.

Initially this repo supported only the 2f-85 however we want to also support the e-pick and pull requests are welcome for other grippers.
- https://github.com/PickNikRobotics/ros2_epick_gripper


## Build status

Currently the `main` branch is used for current releases after Humble: Kilted, Jazzy, and Rolling.
As this is not a core ROS 2 package API/ABI breakage is not guaranteed, it is done as best effort and takes into account maintenance costs.
This is not sponsored or maintained by Robotiq we try to keep everything on main to reduce maintenance overhead.

ROS2 Distro | Branch | Build status | Documentation | Released packages
:---------: | :----: | :----------: | :-----------: | :---------------:
**Rolling** | [`main`](https://github.com/PickNikRobotics/ros2_robotiq_gripper/tree/main) | [![Rolling Binary Build](https://github.com/PickNikRobotics/ros2_robotiq_gripper/actions/workflows/rolling-binary-build-main.yml/badge.svg?branch=main)](https://github.com/PickNikRobotics/ros2_robotiq_gripper/actions/workflows/rolling-binary-build-main.yml?branch=main) <br> [![Rolling Semi-Binary Build](https://github.com/PickNikRobotics/ros2_robotiq_gripper/actions/workflows/rolling-semi-binary-build-main.yml/badge.svg?branch=main)](https://github.com/PickNikRobotics/ros2_robotiq_gripper/actions/workflows/rolling-semi-binary-build-main.yml?branch=main) <br> [![build.ros2.org](https://build.ros2.org/buildStatus/icon?job=Rdev__ros2_robotiq_gripper__ubuntu_noble_amd64&subject=build.ros2.org)](https://build.ros2.org/job/Rdev__ros2_robotiq_gripper__ubuntu_noble_amd64/) | | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Rbin_uN64__ros2_robotiq_gripper__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Rbin_uN64__ros2_robotiq_gripper__ubuntu_noble_amd64__binary/)
**Kilted** | [`main`](https://github.com/PickNikRobotics/ros2_robotiq_gripper/tree/main) |  | | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Kbin_uN64__ros2_robotiq_gripper__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Kbin_uN64__ros2_robotiq_gripper__ubuntu_noble_amd64__binary/)
**Jazzy** | [`main`](https://github.com/PickNikRobotics/ros2_robotiq_gripper/tree/main) |  | | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Jbin_uN64__ros2_robotiq_gripper__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Jbin_uN64__ros2_robotiq_gripper__ubuntu_noble_amd64__binary/)
**Humble** | [`humble`](https://github.com/PickNikRobotics/ros2_robotiq_gripper/tree/humble) |  | | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__ros2_robotiq_gripper__ubuntu_jammy_amd64__binary)](https://build.ros2.org/job/Hbin_uJ64__ros2_robotiq_gripper__ubuntu_jammy_amd64__binary/)

### Explanation of different build types

**NOTE**: There are three build stages checking current and future compatibility of the package.

[Detailed build status](.github/workflows/README.md)

1. Binary builds - against released packages (main and testing) in ROS distributions. Shows that direct local build is possible.

   Uses repos file: `$NAME$-not-released.<ros-distro>.repos`

1. Semi-binary builds - against released core ROS packages (main and testing), but the immediate dependencies are pulled from source.
   Shows that local build with dependencies is possible and if fails there we can expect that after the next package sync we will not be able to build.

   Uses repos file: `$NAME$.repos`

1. Source build - also core ROS packages are build from source. It shows potential issues in the mid future.
