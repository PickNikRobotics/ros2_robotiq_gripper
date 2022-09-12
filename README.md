# ros2_robotiq_gripper

## Building

#### Create ROS2 workspace
```
export COLCON_WS=~/workspace/
mkdir -p $COLCON_WS/src

```

#### Pull relevant packages, install dependencies, compile, and source the workspace by using

```
cd $COLCON_WS
git clone https://github.com/PickNikRobotics/ros2_robotiq_gripper.git src/ros2_robotiq_gripper.git
vcs import src --skip-existing --input src/ros2_robotiq_gripper/ros2_robotiq_gripper.repos
rosdep update
rosdep install --ignore-src --from-paths src -y -r
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

```

## Launch

#### With real hardware

```
ros2 launch robotiq_driver robotiq_control.launch.py

```

#### With ros2_control fake hardware

```
ros2 launch robotiq_driver robotiq_control.launch.py use_fake_hardware:=true
```

#### With Ignition simulation

```
ros2 launch robotiq_driver robotiq_simulation.launch.py
```

## Usage

```
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.80, max_effort: 50.0}}"
```
