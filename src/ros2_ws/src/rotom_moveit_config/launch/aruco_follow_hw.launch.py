import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("rotom", package_name="rotom_moveit_config")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )
    launch_package_path = moveit_config.package_path / "launch"
    servo_params_file = moveit_config.package_path / "config" / "moveit_servo.yaml"

    vision_share = get_package_share_directory("rotom_vision")
    servo_share = get_package_share_directory("rotom_servo")

    vision_launch = os.path.join(vision_share, "launch", "vision_pipeline.launch.py")
    twist_relay_launch = os.path.join(servo_share, "launch", "twist_relay.launch.py")

    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(launch_package_path / "rsp.launch.py")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(launch_package_path / "move_group.launch.py")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(launch_package_path / "moveit_rviz.launch.py")),
                condition=IfCondition(use_rviz),
            ),
            Node(
                package="rotom_control",
                executable="rotom_motor_bridge",
                name="rotom_motor_bridge",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "trajectory_command_topic": "/servo_node/joint_trajectory",
                    }
                ],
                additional_env={"PYTHONUNBUFFERED": "1"},
            ),
            Node(
                package="moveit_servo",
                executable="servo_node_main",
                name="servo_node",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    str(servo_params_file),
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(twist_relay_launch),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(vision_launch),
                launch_arguments={"enable_follower_output": "true"}.items(),
            ),
            TimerAction(
                period=3.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "service",
                            "call",
                            "/servo_node/start_servo",
                            "std_srvs/srv/Trigger",
                            "{}",
                        ],
                        output="screen",
                    )
                ],
            ),
        ]
    )
