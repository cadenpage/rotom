import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    moveit_share = get_package_share_directory("rotom_moveit_config")
    task_share = get_package_share_directory("rotom_task_control")
    controller_config = os.path.join(task_share, "config", "planar_task_controller.yaml")

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(moveit_share, "launch", "rsp.launch.py")),
            ),
            Node(
                package="rotom_control",
                executable="rotom_motor_bridge",
                name="rotom_motor_bridge",
                output="screen",
                parameters=[{"publish_rate_hz": 90.0}],
            ),
            Node(
                package="rotom_task_control",
                executable="planar_task_controller",
                name="planar_task_controller",
                output="screen",
                emulate_tty=True,
                additional_env={"PYTHONUNBUFFERED": "1"},
                parameters=[controller_config],
            ),
        ]
    )
