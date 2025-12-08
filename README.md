# ros2_robotiq_gripper

This repository contains the ROS 2 driver, controller and description packages for working with a Robotiq Gripper.

The goal is to support multiple Robotiq Grippers.

Initially this repo supported only the 2f-85 however we want to also support the e-pick and pull requests are welcome for other grippers.
- https://github.com/PickNikRobotics/ros2_epick_gripper


## Build status

Currently the `main` branch is used for all current releases: Humble, Iron and Rolling.
The `jazzy` branch adds support for ROS 2 Jazzy Jalisco (Ubuntu 24.04).
As this is not a core ROS 2 package API/ABI breakage is not guaranteed, it is done as best effort and takes into account maintenance costs.
This is not sponsored or maintained by Robotiq we try to keep everything on main to reduce maintenance overhead.


ROS2 Distro | Branch | Build status | Documentation | Released packages
:---------: | :----: | :----------: | :-----------: | :---------------:
**Rolling** | [`main`](https://github.com/PickNikRobotics/ros2_robotiq_gripper/tree/main) | [![Rolling Binary Build](https://github.com/PickNikRobotics/ros2_robotiq_gripper/actions/workflows/rolling-binary-build-main.yml/badge.svg?branch=main)](https://github.com/PickNikRobotics/ros2_robotiq_gripper/actions/workflows/rolling-binary-build-main.yml?branch=main) <br /> [![Rolling Semi-Binary Build](https://github.com/PickNikRobotics/ros2_robotiq_gripper/actions/workflows/rolling-semi-binary-build-main.yml/badge.svg?branch=main)](https://github.com/PickNikRobotics/ros2_robotiq_gripper/actions/workflows/rolling-semi-binary-build-main.yml?branch=main) | | [ros2_robotiq_gripper](https://index.ros.org/p/ros2_robotiq_gripper/github-PickNikRobotics-ros2_robotiq_grippper/#rolling)


ROS2 Distro | Branch | Build status | Documentation | Released packages
:---------: | :----: | :----------: | :-----------: | :---------------:
**Humble** | [`main`](https://github.com/PickNikRobotics/ros2_robotiq_gripper/tree/main) | [![Humble Binary Build](https://github.com/PickNikRobotics/ros2_robotiq_gripper/actions/workflows/humble-binary-build-main.yml/badge.svg?branch=main)](https://github.com/PickNikRobotics/ros2_robotiq_gripper/actions/workflows/humble-binary-build-main.yml?branch=main) <br /> [![Humble Semi-Binary Build](https://github.com/PickNikRobotics/ros2_robotiq_gripper/actions/workflows/humble-semi-binary-build-main.yml/badge.svg?branch=main)](https://github.com/PickNikRobotics/ros2_robotiq_gripper/actions/workflows/humble-semi-binary-build-main.yml?branch=main) | | [ros2_robotiq_gripper](https://index.ros.org/p/ros2_robotiq_gripper/github-PickNikRobotics-ros2_robotiq_grippper/#humble)


ROS2 Distro | Branch | Build status | Documentation | Released packages
:---------: | :----: | :----------: | :-----------: | :---------------:
**Iron** | [`main`](https://github.com/PickNikRobotics/ros2_robotiq_gripper/tree/main) | [![Iron Binary Build](https://github.com/PickNikRobotics/ros2_robotiq_gripper/actions/workflows/iron-binary-build-main.yml/badge.svg?branch=main)](https://github.com/PickNikRobotics/ros2_robotiq_gripper/actions/workflows/iron-binary-build-main.yml?branch=main) <br /> [![Iron Semi-Binary Build](https://github.com/PickNikRobotics/ros2_robotiq_gripper/actions/workflows/iron-semi-binary-build-main.yml/badge.svg?branch=main)](https://github.com/PickNikRobotics/ros2_robotiq_gripper/actions/workflows/iron-semi-binary-build-main.yml?branch=main) | | [ros2_robotiq_gripper](https://index.ros.org/p/ros2_robotiq_gripper/github-PickNikRobotics-ros2_robotiq_grippper/#iron)


ROS2 Distro | Branch | Build status | Documentation | Released packages
:---------: | :----: | :----------: | :-----------: | :---------------:
**Jazzy** | [`jazzy`](#) | N/A | | Not yet released

### Explanation of different build types

**NOTE**: There are three build stages checking current and future compatibility of the package.

[Detailed build status](.github/workflows/README.md)

1. Binary builds - against released packages (main and testing) in ROS distributions. Shows that direct local build is possible.

   Uses repos file: `$NAME$-not-released.<ros-distro>.repos`

1. Semi-binary builds - against released core ROS packages (main and testing), but the immediate dependencies are pulled from source.
   Shows that local build with dependencies is possible and if fails there we can expect that after the next package sync we will not be able to build.

   Uses repos file: `$NAME$.repos`

1. Source build - also core ROS packages are build from source. It shows potential issues in the mid future.

---

## ROS 2 Jazzy Jalisco Support

This fork adds support for **ROS 2 Jazzy Jalisco** (Ubuntu 24.04). The Jazzy branch includes API compatibility updates required for the newer ROS 2 release.

### Installation (Jazzy)

```bash
# Create workspace
mkdir -p ~/ws_robotiq/src
cd ~/ws_robotiq/src

# Clone this repository (jazzy branch)
git clone -b jazzy https://github.com/bryceag11/ros2_robotiq_gripper.git

# Import dependencies (gets the serial package)
vcs import . < ros2_robotiq_gripper/ros2_robotiq_gripper.rolling.repos

# Build
cd ~/ws_robotiq
colcon build --symlink-install

# Source
source install/setup.bash
```

### Changes from Humble/Iron/Rolling

The Jazzy port includes the following updates for API compatibility:

1. **Hardware Interface API Updates**
   - Updated `ResourceManager` constructor to use new Jazzy signature with `rclcpp::Clock` and `rclcpp::Logger`
   - Implemented new `on_init(const HardwareComponentInterfaceParams&)` method alongside deprecated signature
   - Changed from `rclcpp::ClockType::SYSTEM_TIME` to `RCL_SYSTEM_TIME` constant

2. **Controller Interface Updates**
   - Updated to check return values from `LoanedCommandInterface::set_value()` (now marked with `[[nodiscard]]`)
   - Replaced deprecated `get_value()` with `get_optional<double>()` for safer value retrieval
   - Added proper error handling for command interface operations

3. **CMake Updates**
   - Added CMake policy CMP0060 to suppress harmless runtime search path warnings
   - Fixed format string warnings in logging statements

4. **Dependencies**
   - Requires the `serial` package from [tylerjw/serial](https://github.com/tylerjw/serial) (imported via `.repos` file)
   - All other dependencies remain the same as Humble/Iron/Rolling

### Tested Environment

- Ubuntu 24.04 LTS (Noble Numbat)
- ROS 2 Jazzy Jalisco
- All packages build successfully without errors

### Contributing

This Jazzy port was created to enable compatibility with Ubuntu 24.04 systems. If you encounter issues or have improvements, please open an issue or pull request.
