import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _workspace_override(relative_path: str, fallback: str) -> str:
    workspace_root = os.environ.get("ROTOM_ROOT", "").strip()
    if workspace_root:
        candidate = os.path.join(workspace_root, relative_path)
        if os.path.exists(candidate):
            return candidate
    return fallback


def generate_launch_description():
    vision_share = get_package_share_directory("rotom_vision")
    description_share = get_package_share_directory("rotom_description")
    control_share = get_package_share_directory("rotom_control")

    description_launch = os.path.join(description_share, "launch", "view_robot.launch.py")
    dragger_rviz = os.path.join(description_share, "rviz", "servo_dragger.rviz")

    use_rviz = LaunchConfiguration("use_rviz")
    start_control = LaunchConfiguration("start_control")
    enable_servo = LaunchConfiguration("enable_servo")
    params_file = LaunchConfiguration("params_file")
    control_params_file = LaunchConfiguration("control_params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("start_control", default_value="true"),
            DeclareLaunchArgument(
                "enable_servo",
                default_value="false",
                description="Set true to let the RViz dragger move the physical arm.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=_workspace_override(
                    "src/ros2_ws/src/rotom_vision/config/vision_pipeline.yaml",
                    os.path.join(vision_share, "config", "vision_pipeline.yaml"),
                ),
            ),
            DeclareLaunchArgument(
                "control_params_file",
                default_value=_workspace_override(
                    "src/ros2_ws/src/rotom_control/config/rotom_control.yaml",
                    os.path.join(control_share, "config", "rotom_control.yaml"),
                ),
            ),
            LogInfo(
                condition=IfCondition(enable_servo),
                msg="Drag servo mode: LIVE (RViz dragger drives the custom damped Jacobian servo).",
            ),
            LogInfo(
                condition=UnlessCondition(enable_servo),
                msg="Drag servo mode: DRY (RViz dragger computes commands but arm motion stays disabled).",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(description_launch),
                launch_arguments={
                    "use_rviz": use_rviz,
                    "use_jsp": "false",
                    "use_jsp_gui": "false",
                    "rviz_config": dragger_rviz,
                }.items(),
            ),
            Node(
                condition=IfCondition(start_control),
                package="rotom_control",
                executable="rotom_motor_bridge",
                name="rotom_motor_bridge",
                output="screen",
                parameters=[control_params_file],
            ),
            Node(
                package="rotom_vision",
                executable="drag_target_servo",
                name="drag_target_servo",
                output="screen",
                parameters=[
                    params_file,
                    {"enable_twist_output": ParameterValue(enable_servo, value_type=bool)},
                ],
            ),
            Node(
                condition=IfCondition(enable_servo),
                package="rotom_vision",
                executable="visual_servo",
                name="visual_servo",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
