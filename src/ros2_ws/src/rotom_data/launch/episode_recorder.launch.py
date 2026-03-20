import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    data_share = get_package_share_directory("rotom_data")
    recorder_config = os.path.join(data_share, "config", "episode_recorder.yaml")

    return LaunchDescription(
        [
            Node(
                package="rotom_data",
                executable="episode_recorder",
                name="episode_recorder",
                output="screen",
                emulate_tty=True,
                additional_env={"PYTHONUNBUFFERED": "1"},
                parameters=[recorder_config],
            ),
        ]
    )
