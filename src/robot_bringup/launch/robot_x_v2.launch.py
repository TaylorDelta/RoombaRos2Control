# Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# Source of this file are templates in
# [RosTeamWorkspace](https://github.com/StoglRobotics/ros_team_workspace) repository.
#
# Author: Dr. Denis
#
"""
Launch sequence:

1. Read launch arguments
2. Generate robot_description from xacro
3. Start ros2_control_node
4. Start robot_state_publisher
5. Wait for ros2_control to come up
6. Spawn joint_state_broadcaster
7. Spawn main robot controller
8. Spawn additional controllers

Good places to add things:
- Additional launch arguments
- Additional ROS nodes
- Additional controllers
- Additional startup dependencies
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import (
    OnProcessStart,
    OnProcessExit,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():

    # ============================================================
    # 1. Launch arguments
    # ============================================================
    #
    # These can be overridden:
    #
    # ros2 launch ... use_robot:=right
    #
    declared_arguments = [

        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Robot namespace",
        ),

        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="TF / URDF frame prefix (e.g. robot1_)",
        ),


        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="robot_bringup",
        ),

        DeclareLaunchArgument(
            "controllers_file",
            default_value="robot_x_controllers.yaml",
        ),


        DeclareLaunchArgument(
            "description_package",
            default_value="robot_description",
        ),

        DeclareLaunchArgument(
            "description_file",
            default_value="robot_x.urdf.xacro",
        ),

        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
        ),

        DeclareLaunchArgument(
            "robot_controller",
            default_value="roomba_controller",
            choices=[
                "roomba_controller",
                "forward_velocity_controller",
                "forward_position_controller",
                "joint_trajectory_controller",
            ],
        ),
    ]

    # ============================================================
    # 2. Resolve launch arguments
    # ============================================================


    namespace = LaunchConfiguration(
        "namespace"
    )

    prefix = LaunchConfiguration(
        "prefix"
    )

    runtime_config_package = LaunchConfiguration(
        "runtime_config_package"
    )

    controllers_file = LaunchConfiguration(
        "controllers_file"
    )

    description_package = LaunchConfiguration(
        "description_package"
    )

    description_file = LaunchConfiguration(
        "description_file"
    )

    use_mock_hardware = LaunchConfiguration(
        "use_mock_hardware"
    )

    robot_controller = LaunchConfiguration(
        "robot_controller"
    )


    # ============================================================
    # 3. Build robot_description
    # ============================================================
    #
    # Equivalent terminal command:
    #
    # xacro robot_x.urdf.xacro use_robot:=left ...
    #

    robot_description_content = Command([
        PathJoinSubstitution([
            FindExecutable(name="xacro")
        ]),

        " ",

        PathJoinSubstitution([
            FindPackageShare(description_package),
            "urdf",
            description_file,
        ]),

        " ",
        "use_mock_hardware:=", use_mock_hardware,
        " ",
        "prefix:=", prefix,
    ])

    robot_description = {
        "robot_description": robot_description_content
    }

    # ============================================================
    # 4. Config file paths
    # ============================================================

    robot_controllers = PathJoinSubstitution([
        FindPackageShare(runtime_config_package),
        "config",
        controllers_file,
    ])


    # ============================================================
    # 5. Core ROS nodes
    # ============================================================

    # ros2_control manager
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        namespace=namespace,

        parameters=[
            robot_description, #takes info from the topic
            robot_controllers,
        ],
    )

    # TF publisher from URDF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,

        parameters=[
            robot_description
        ],

        output="screen",
    )

    # ============================================================
    # 6. Additional nodes
    # ============================================================
    #
    # Add your own nodes here.
    #
    # Example:

    # lidar_node = Node(...)
    #
    # camera_node = Node(...)
    #


    twist_to_stamped_node = Node(
        package="twist_bridge",
        executable="twist_to_stamped",
        output="screen",
        namespace=namespace,

    )


    # ============================================================
    # 7. Controller spawners
    # ============================================================

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            PathJoinSubstitution(["/", namespace, "controller_manager"]),
        ]
    )

    # Controllers that should become ACTIVE

    active_controllers = [
        robot_controller,
        "gpio_command_controller",

        # Add new controllers here:
        #
        # "diff_drive_controller",
        # "imu_broadcaster",
        #
    ]

    active_controller_spawners = []

    for controller in active_controllers:

        active_controller_spawners.append(

            Node(
                package="controller_manager",
                executable="spawner",
                namespace=namespace,

                arguments=[
                    controller,
                    "--controller-manager",
                    PathJoinSubstitution(["/", namespace, "controller_manager"]),
                ]
            )
        )

    # ============================================================
    # 8. Startup ordering
    # ============================================================

    #
    # ros2_control_node
    #      ↓
    # wait 3 sec
    #      ↓
    # joint_state_broadcaster
    #      ↓
    # robot_controller
    #      ↓
    # gpio_command_controller
    #

    start_joint_state_broadcaster = RegisterEventHandler(
        OnProcessStart(

            target_action=control_node,

            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[
                        joint_state_broadcaster
                    ],
                )
            ],
        )
    )

    start_active_controllers = []

    for i, controller in enumerate(
        active_controller_spawners
    ):

        previous_controller = (
            active_controller_spawners[i - 1]
            if i > 0
            else joint_state_broadcaster
        )

        start_active_controllers.append(

            RegisterEventHandler(

                OnProcessExit(

                    target_action=previous_controller,

                    on_exit=[
                        controller
                    ],
                )
            )
        )

    # ============================================================
    # 9. Launch everything
    # ============================================================

    launch_items = [

        # Core
        control_node,
        robot_state_publisher,
        twist_to_stamped_node,

        # Custom nodes
        # lidar_node,
        # camera_node,

        # Controller startup sequence
        start_joint_state_broadcaster,

    ] + start_active_controllers

    return LaunchDescription(
        declared_arguments + launch_items
    )