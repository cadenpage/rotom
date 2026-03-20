import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    data_share = get_package_share_directory("rotom_data")
    task_share = get_package_share_directory("rotom_task_control")
    vision_share = get_package_share_directory("rotom_vision")

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(task_share, "launch", "task_control_hw.launch.py")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(vision_share, "launch", "camera_stream.launch.py")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(data_share, "launch", "episode_recorder.launch.py")),
            ),
        ]
    )
