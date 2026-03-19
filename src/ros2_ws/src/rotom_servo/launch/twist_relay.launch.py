from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    command_frame = LaunchConfiguration("command_frame")
    return LaunchDescription(
        [
            DeclareLaunchArgument("command_frame", default_value="tool0"),
            Node(
                package="rotom_servo",
                executable="twist_relay",
                name="twist_relay",
                output="screen",
                emulate_tty=True,
                additional_env={"PYTHONUNBUFFERED": "1"},
                parameters=[
                    {
                        "command_frame": command_frame,
                    }
                ],
            ),
        ]
    )
