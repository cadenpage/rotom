from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="rotom_vision",
                executable="image_monitor",
                name="view_camera",
                output="screen",
                parameters=[
                    {
                        "image_topic": "/camera/selected/image_raw",
                        "window_name": "Rotom Camera",
                        "max_width": 1200,
                    }
                ],
            ),
        ]
    )
