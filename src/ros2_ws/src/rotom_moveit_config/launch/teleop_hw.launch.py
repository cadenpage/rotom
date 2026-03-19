import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    moveit_share = get_package_share_directory("rotom_moveit_config")
    servo_share = get_package_share_directory("rotom_servo")
    teleop_share = get_package_share_directory("rotom_teleop")

    use_rviz = LaunchConfiguration("use_rviz")
    table_slide_mode = LaunchConfiguration("table_slide_mode")
    include_teleop = LaunchConfiguration("include_teleop")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("table_slide_mode", default_value="false"),
            DeclareLaunchArgument("include_teleop", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(moveit_share, "launch", "servo_hw.launch.py")),
                launch_arguments={"use_rviz": use_rviz}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(servo_share, "launch", "twist_relay.launch.py")),
                launch_arguments={"command_frame": "ground"}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(teleop_share, "launch", "mediapipe_teleop.launch.py")),
                launch_arguments={"table_slide_mode": table_slide_mode}.items(),
                condition=IfCondition(include_teleop),
            ),
            TimerAction(
                period=4.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "service",
                            "call",
                            "/servo_node/change_control_dimensions",
                            "moveit_msgs/srv/ChangeControlDimensions",
                            (
                                "{control_x_translation: true, control_y_translation: true, "
                                "control_z_translation: false, control_x_rotation: false, "
                                "control_y_rotation: false, control_z_rotation: false}"
                            ),
                        ],
                        output="screen",
                    )
                ],
                condition=IfCondition(table_slide_mode),
            ),
        ]
    )
