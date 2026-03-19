import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("rotom_teleop")
    config_file = os.path.join(package_share, "config", "mediapipe_teleop.yaml")

    return LaunchDescription(
        [
            Node(
                package="rotom_teleop",
                executable="mediapipe_teleop",
                name="mediapipe_teleop",
                output="screen",
                emulate_tty=True,
                additional_env={"PYTHONUNBUFFERED": "1"},
                parameters=[config_file],
            )
        ]
    )
