from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="rotom_vision",
                executable="image_monitor",
                name="view_aruco",
                output="screen",
                parameters=[
                    {
                        "image_topic": "/aruco/debug_image",
                        "window_name": "Rotom ArUco Debug",
                        "max_width": 1200,
                        "reliability": "reliable",
                    }
                ],
            ),
        ]
    )
