# Roomba Hardware Interface and ROS2 Controller

This project implements a ROS2 hardware interface and controller for the iRobot® Roomba® using the Serial Command Interface (SCI) specification. It provides a seamless way to control Roomba's movement via the Joint Trajectory Controller in ROS2.

## Features

Interface to control Roomba's velocity using ROS2 messages.

Compatible with ROS2-based systems.

Can be used to send joint trajectory commands to Roomba for movement control.

Utilizes the iRobot® Roomba® Serial Command Interface (SCI) for communication.

## Requirements

ROS2 (e.g., Foxy, Galactic, or Humble)

iRobot® Roomba® with SCI support

A working serial connection between the host computer and Roomba.

## Setup and Installation
1. Clone the repository
git clone https://github.com/your-username/roomba_ros2_controller.git
cd roomba_ros2_controller

2. Install dependencies

Install necessary ROS2 dependencies for your workspace:

sudo apt update
sudo apt install ros-<ros2-distro>-serial

3. Build the workspace

From your ROS2 workspace, build the project:

colcon build

4. Source the workspace

After building, source the workspace to ensure that ROS2 can find the packages:

source install/setup.bash

Launch the Roomba Controller

To launch the hardware interface and controller, run the following command:

ros2 launch robot_bringup robot_x.launch.py


This will start the necessary ROS2 nodes for controlling your Roomba robot.

## Sending Commands to Roomba

You can use the Joint Trajectory Controller to send movement commands to Roomba. To send a trajectory command, run the following command in a separate terminal:

ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'base_link'
joint_names: ['linear_velocity_joint', 'angular_velocity_joint']
points:
  - positions: [0.0, 0.0]
    velocities: [200.0, 0.0]
    accelerations: [0.0, 0.0]
    time_from_start:
      sec: 2
      nanosec: 0"

Explanation of Command:

positions: The position command for the joints. In this example, 200.0 for linear_velocity_joint corresponds to the desired linear velocity in mm/s, and 0.0 for angular_velocity_joint indicates no rotational movement.

velocities: Specifies the desired velocity for each joint.

accelerations: Specifies the desired acceleration for each joint.

time_from_start: The time duration for the trajectory to reach the desired position.

Example Behavior:

In the example above, the command will move the Roomba forward at a speed of 200 mm/s for 2 seconds. You can modify the velocity values to control the Roomba's velocity and direction.

## Troubleshooting

Ensure that the serial connection between the computer and Roomba is properly established.

Verify that the correct serial port is being used in your launch files.

If commands are not working, check the Roomba's logs to see if there are any communication issues.

## License

This project is licensed under the MIT License - see the LICENSE
 file for details.

## Acknowledgments

The iRobot® Roomba® Serial Command Interface (SCI) Specification

ROS2 and the ROS community for providing an open-source platform for robotics development.

Feel free to modify the README as needed for your specific project! If you have further questions or need more details, let me know!

