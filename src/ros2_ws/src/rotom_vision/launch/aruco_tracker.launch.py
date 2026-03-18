from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="rotom_vision",
                executable="aruco_tracker",
                name="aruco_tracker",
                output="screen",
            ),
        ]
    )
