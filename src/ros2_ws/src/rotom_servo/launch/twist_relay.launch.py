from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="rotom_servo",
                executable="twist_relay",
                name="twist_relay",
                output="screen",
                emulate_tty=True,
                additional_env={"PYTHONUNBUFFERED": "1"},
            ),
        ]
    )
