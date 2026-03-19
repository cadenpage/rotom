import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    moveit_share = get_package_share_directory("rotom_moveit_config")
    servo_share = get_package_share_directory("rotom_servo")
    teleop_share = get_package_share_directory("rotom_teleop")

    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(moveit_share, "launch", "servo_hw.launch.py")),
                launch_arguments={"use_rviz": use_rviz}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(servo_share, "launch", "twist_relay.launch.py")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(teleop_share, "launch", "mediapipe_teleop.launch.py")),
            ),
        ]
    )
