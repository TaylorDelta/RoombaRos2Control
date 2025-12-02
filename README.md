# Roomba Hardware Interface and ROS2 Controller

This project implements a ROS2 hardware interface and controller for the iRobot® Roomba® using the Serial Command Interface (SCI) specification. It provides a seamless way to control Roomba's movement via the Forward Velocity Controller in ROS2.

## Features

Interface to control Roomba's velocity using ROS2 messages.

Compatible with ROS2-based systems.

Can be used to send commands to Roomba for movement control. Different controllers can be used.

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
```md
sudo apt update
sudo apt install ros-<ros2-distro>-serial
```
3. Build the workspace

From your ROS2 workspace, build the project:
```md
colcon build
```
4. Source the workspace

After building, source the workspace to ensure that ROS2 can find the packages:
```md
source install/setup.bash
```
Launch the Roomba Controller

To launch the hardware interface and controller, run the following command:
```md
ros2 launch robot_bringup robot_x.launch.py
```

This will start the necessary ROS2 nodes for controlling your Roomba robot.

## Sending Commands to Roomba

You can use the Joint Trajectory Controller to send movement commands to Roomba. To send a trajectory command, run the following command in a separate terminal:
```md
ros2 topic pub /forward_velocity_controller/commands std_msgs/msg/Float64MultiArray "data: [Velocity, Radius]" --once
```
### Explanation of Command
- **Velocity**: 
  - Defines the speed of the robot in millimeters per second.
  - The valid range for velocity is **-500.0 to 500.0 mm/s**:
    - A positive value indicates **forward motion**.
    - A negative value indicates **reverse motion**.
- **Radius**: 
  - Defines the turning radius of the robot in millimeters.
  - The valid range for radius is **-2000.0 to 2000.0 mm**:
    - A positive value results in a **counter-clockwise** turn (left turn).
    - A negative value results in a **clockwise** turn (right turn).
    - A radius of `0` corresponds to **moving in a straight line**.

- **Special Cases** for Turning in Place:
  - **Turn in place clockwise**: Set **radius = -1**.
  - **Turn in place counter-clockwise**: Set **radius = 1**.


## Troubleshooting

Ensure that the serial connection between the computer and Roomba is properly established.

Verify that the correct serial port is being used in your launch files.

If commands are not working, check the Roomba's logs to see if there are any communication issues.

## License

This project is licensed under the Apache 2.0 License - see the LICENSE
 file for details.

## Acknowledgments

The iRobot® Roomba® Serial Command Interface (SCI) Specification

ROS2 and the ROS community for providing an open-source platform for robotics development.

Feel free to modify the README as needed for your specific project! If you have further questions or need more details, let me know!

