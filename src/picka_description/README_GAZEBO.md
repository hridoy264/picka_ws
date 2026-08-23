# Picka Arm — ROS 2 Humble and Gazebo Fortress

This package targets:

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Gazebo Fortress (Ignition Gazebo 6)
- `ros_gz_sim` and `gz_ros2_control`
- Bash workspace: `~/ros2_humble`

Ubuntu 22.02 is not an Ubuntu release; use Ubuntu 22.04.

## What is included

- Fixed-base Picka arm description with CAD visual meshes
- Primitive collision boxes derived from the scaled STL bounds
- Position interfaces for five commanded joints
- Passive mirrored left gripper finger
- `joint_state_broadcaster`
- `joint_trajectory_controller` named `arm_controller`
- Fortress-compatible empty world and `/clock` bridge
- RViz-only and Gazebo launch files

## Install dependencies

Install ROS 2 Humble first, then:

```bash
sudo apt update
sudo apt install \
  ros-humble-ros-gz \
  ros-humble-gz-ros2-control \
  ros-humble-controller-manager \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher \
  ros-humble-ros2controlcli \
  ros-humble-rviz2 \
  ros-humble-xacro
```

The supported Humble/Fortress control package is
`ros-humble-gz-ros2-control`. Do not install or use
`gazebo_ros2_control`, which is for Gazebo Classic.

## Put the package in the workspace

```bash
mkdir -p ~/ros2_humble/src
cd ~/ros2_humble/src
unzip ~/Downloads/picka_description_gazebo_ready_humble.zip
```

The result must be:

```text
~/ros2_humble/src/picka_description/package.xml
```

## Clean build

```bash
cd ~/ros2_humble
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
rm -rf build/picka_description install/picka_description
colcon build --symlink-install --packages-select picka_description
source install/setup.bash
ros2 pkg prefix picka_description
```

The last command should print a path under
`~/ros2_humble/install/picka_description`.

## Launch

Normal Gazebo server and GUI:

```bash
cd ~/ros2_humble
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch picka_description gazebo.launch.py
```

Headless server:

```bash
ros2 launch picka_description gazebo.launch.py gui:=false
```

Gazebo plus RViz:

```bash
ros2 launch picka_description gazebo.launch.py use_rviz:=true
```

RViz description check without Gazebo:

```bash
ros2 launch picka_description display.launch.py
```

Optional spawn arguments are `entity_name`, `x`, `y`, `z`, and `yaw`:

```bash
ros2 launch picka_description gazebo.launch.py entity_name:=picka_1 x:=0.2 yaw:=0.5
```

## Verify simulation and controllers

Run these in a second sourced terminal:

```bash
cd ~/ros2_humble
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 node list
ros2 topic list
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states --once
```

Expected active controllers:

```text
joint_state_broadcaster
arm_controller
```

Expected command interfaces are:

```text
base_joint/position
shoulder_joint/position
elbow_joint/position
wrist_joint/position
right_gripper_joint/position
```

`left_gripper_joint` is passive and follows:

```text
left_gripper_joint = -1 × right_gripper_joint
```

## Safe motion test

The following small goal stays inside all configured limits:

```bash
ros2 action send_goal \
  /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [base_joint, shoulder_joint, elbow_joint, wrist_joint, right_gripper_joint], points: [{positions: [0.20, 0.15, 0.20, -0.15, 0.10], time_from_start: {sec: 4}}]}}"
```

Watch feedback:

```bash
ros2 topic echo /joint_states
```

Do not begin with large or fast commands. First confirm that every joint rotates
around the intended physical shaft and that the fingers move symmetrically.

## Stop and restart

Press `Ctrl+C` in the launch terminal. Wait for Gazebo and the controller manager
to exit, then run the launch command again. If source files change, rebuild and
source the workspace again before relaunching.

## Model assumptions and important changes

- The original CAD joint origins, axes, visual transforms, masses, and joint
  position limits were preserved.
- Original velocity limits of `100 rad/s` were reduced to conservative simulation
  limits between `1.0` and `2.0 rad/s`.
- The two zero-inertia horn links use solid-box inertia approximations calculated
  from their STL bounding boxes and original masses.
- Detailed STL collision meshes were replaced with per-link axis-aligned boxes.
  Visual meshes remain unchanged.
- The base uses the Humble/Fortress `world` fixed-link pattern. A `-0.02 m`
  mounting offset places the base collision bottom at ground level.
- The gripper multiplier of `-1` is inferred from the mirrored joint limits:
  right `[0, +0.523599]`, left `[-0.523599, 0]`.
- The uploaded files did not identify a real maintainer or license. Update
  `package.xml` and `setup.py` before public distribution.
- The ZIP contained no reference screenshots. All six revolute origins lie inside
  both connected-link mesh bounds, and a moved-pose kinematic check remained
  connected, but final shaft/horn concentricity must still be visually confirmed
  in Gazebo on the target Ubuntu system.

See `VALIDATION_REPORT.md` for the exact checks that were and were not run.

## Compatibility references

- [gz_ros2_control Humble branch](https://github.com/ros-controls/gz_ros2_control/tree/humble)
- [ROS–Gazebo compatibility](https://github.com/gazebosim/ros_gz)
- [ROS 2 controller manager](https://control.ros.org/humble/doc/ros2_control/controller_manager/doc/userdoc.html)
