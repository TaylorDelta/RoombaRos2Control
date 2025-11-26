from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[{
                "robot_description": Command(['xacro ', '/home/ubuntu/test_ws/src/pacr_robot_description/urdf/pacr.urdf.xacro'])
            }],
            output="screen"
        )
    ])
