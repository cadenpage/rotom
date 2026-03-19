import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    task_share = get_package_share_directory("rotom_task_control")
    mediapipe_config = os.path.join(task_share, "config", "mediapipe_task_input.yaml")

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(task_share, "launch", "task_control_hw.launch.py")),
            ),
            Node(
                package="rotom_task_control",
                executable="mediapipe_task_input",
                name="mediapipe_task_input",
                output="screen",
                emulate_tty=True,
                additional_env={"PYTHONUNBUFFERED": "1"},
                parameters=[mediapipe_config],
            ),
        ]
    )
