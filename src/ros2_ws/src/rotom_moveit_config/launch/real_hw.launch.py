from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("rotom", package_name="rotom_moveit_config").to_moveit_configs()
    launch_package_path = moveit_config.package_path / "launch"

    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
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
            ),
        ]
    )
