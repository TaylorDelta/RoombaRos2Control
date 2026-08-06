setup

ros2 launch robot_bringup robot_x_v2.launch.py   namespace:=robot1   prefix:=robot1/ controllers_file:=robot_x_controllers_robot1.yaml


ros2 launch robot_bringup robot_x_v2.launch.py   namespace:=robot2   prefix:=robot2/ controllers_file:=robot_x_controllers_robot2.yaml



# Roomba Hardware Interface and ROS 2 Controller

This repository provides a ros2_control hardware interface for iRobot® Roomba® using the SCI (Serial Command Interface) and a custom diff-drive-like controller plugin (roomba_controller). It includes bringup, URDF, and a small Twist→TwistStamped bridge for Foxglove control.

Make sure to use the right branch for your model series. Currently available branches: 500er and 600er.

## Features

- ros2_control hardware plugin that talks SCI over a serial connection
- Custom controller plugin roomba_controller for differential drive (TwistStamped in, odom/tf out)
- Bringup and controller configuration via a single launch file
- Optional Foxglove control via a simple twist_bridge node

## Supported platforms

- ROS 2 Humble (and newer). Earlier distros like Foxy/Galactic are not supported.
- Hardware: iRobot® Roomba® with SCI support and a serial/USB interface
- OS: Linux recommended for hardware. The serial device is expected at /dev/roomba (configurable via udev rule).

## Repository layout

- robot_hardware_interface: ros2_control SystemInterface plugin that implements SCI communication
- roomba_controller: Controller plugin that accepts TwistStamped and publishes odometry and tf
- robot_bringup: Launch file and controller configuration
- robot_description: URDF/Xacro and RViz config used for bringup/visualization
- twist_bridge: Converts un-stamped Twist (e.g., from Foxglove) to TwistStamped for the controller

## Quick start

1) Create or use an existing ROS 2 workspace and clone this repo into src

```
mkdir -p <your_ws>/src
cd <your_ws>/src
git clone https://github.com/TaylorDelta/RoombaRos2Control.git
```

2) Install dependencies (system) and resolve the rest with rosdep

```
sudo apt update && sudo apt install -y \
  ros-${ROS_DISTRO}-ros2-control \
  ros-${ROS_DISTRO}-ros2-controllers \
  ros-${ROS_DISTRO}-controller-manager \
  ros-${ROS_DISTRO}-xacro \
  ros-${ROS_DISTRO}-robot-state-publisher \
  ros-${ROS_DISTRO}-rviz2 \
  ros-${ROS_DISTRO}-foxglove-bridge

cd <your_ws>
rosdep update
rosdep install --from-paths src -y --ignore-src
```

3) Build and source

```
cd <your_ws>
colcon build
source install/setup.bash
```

4) Launch bringup

```
ros2 launch robot_bringup robot_x.launch.py
```

Useful launch arguments:
- use_mock_hardware:=true to run without a real robot (mirrors commands to state)
- robot_controller:=roomba_controller|forward_velocity_controller|joint_trajectory_controller

RViz is not started by default. To visualize, you can run:

```
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix robot_description)/share/robot_description/rviz/robot_x.rviz
```

## Sending commands

The roomba_controller subscribes to the private topic ~/cmd_vel, which resolves to the absolute topic /roomba_controller/cmd_vel. It expects geometry_msgs/TwistStamped by default.

Example: drive a slow circle (about 1 m diameter) at 0.1 m/s

```
ros2 topic pub /roomba_controller/cmd_vel geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, twist: {linear: {x: 0.1}, angular: {z: 0.2}}}" -r 10
```

Stop by publishing zeros or interrupting the publisher (Ctrl+C).

Notes:
- The controller publishes odometry on /roomba_controller/odom and a transform on /tf (odom→base_link) when enabled in its parameters.
- The command interface type can be changed to un-stamped Twist via controller parameters if desired.

## Foxglove (optional web control)

Install the bridge:

```
sudo apt install ros-${ROS_DISTRO}-foxglove-bridge
```

Start the ROS 2 ↔ Foxglove WebSocket bridge:

```
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

Run the twist bridge (subscribes to /cmd_vel_fg as Twist and republishes TwistStamped to the controller topic):

```
ros2 run twist_bridge twist_to_stamped
```

In Foxglove, publish geometry_msgs/Twist to /cmd_vel_fg.

## Hardware and serial notes

- The hardware interface uses a serial device at /dev/roomba. Create a stable symlink with a udev rule, e.g.:

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", SYMLINK+="roomba"
```

Then reload rules and replug the adapter:

```
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Ensure your user has serial permissions:

```
sudo usermod -a -G dialout $USER
newgrp dialout
```

## Troubleshooting

- Controller not moving: verify you publish to /roomba_controller/cmd_vel and that roomba_controller is the active controller.
- No odom/tf: check controller parameters and that the node is active; verify /roomba_controller/odom and /tf.
- Serial errors: confirm the device exists at /dev/roomba, udev rule is applied, and user is in the dialout group.

## License

This project is licensed under the Apache 2.0 License – see the LICENSE file for details.

## Acknowledgments

- iRobot® Roomba® Serial Command Interface (SCI) specification
- ROS 2 and the ROS community
- Templates from RosTeamWorkspace

This project uses code from [RosTeamWorkspace](https://github.com/b-robotized/ros_team_workspace), licensed under the Apache License 2.0.



