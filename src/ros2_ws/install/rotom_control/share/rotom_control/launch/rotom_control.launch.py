from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("rotom_control")
    params_path = os.path.join(pkg_share, "config", "rotom_control.yaml")

    return LaunchDescription(
        [
            Node(
                package="rotom_control",
                executable="rotom_motor_bridge",
                name="rotom_motor_bridge",
                output="screen",
                parameters=[params_path],
            ),
        ]
    )
