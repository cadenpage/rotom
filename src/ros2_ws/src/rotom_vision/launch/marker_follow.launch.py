from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="rotom_vision",
                executable="marker_follower",
                name="marker_follower",
                output="screen",
            ),
        ]
    )
