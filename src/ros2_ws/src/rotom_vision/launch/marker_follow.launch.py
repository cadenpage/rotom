import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


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

    vision_launch = os.path.join(vision_share, "launch", "vision_pipeline.launch.py")
    description_launch = os.path.join(description_share, "launch", "view_robot.launch.py")

    use_rviz = LaunchConfiguration("use_rviz")
    start_camera = LaunchConfiguration("start_camera")
    start_control = LaunchConfiguration("start_control")
    enable_servo = LaunchConfiguration("enable_servo")
    params_file = LaunchConfiguration("params_file")
    calibration_file = LaunchConfiguration("calibration_file")
    control_params_file = LaunchConfiguration("control_params_file")
    camera_device = LaunchConfiguration("camera_device")
    camera_pixel_format = LaunchConfiguration("camera_pixel_format")
    camera_image_size = LaunchConfiguration("camera_image_size")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Set true to start RViz with the robot model alongside marker following.",
            ),
            DeclareLaunchArgument(
                "start_camera",
                default_value="true",
                description="Set true to start the v4l2 camera node.",
            ),
            DeclareLaunchArgument(
                "start_control",
                default_value="true",
                description="Set true to start the motor bridge so /joint_states and /joint_command are live.",
            ),
            DeclareLaunchArgument(
                "enable_servo",
                default_value="false",
                description="Set true only after marker tracking looks correct and you are ready for live motion.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=_workspace_override(
                    "src/ros2_ws/src/rotom_vision/config/vision_pipeline.yaml",
                    os.path.join(vision_share, "config", "vision_pipeline.yaml"),
                ),
            ),
            DeclareLaunchArgument(
                "calibration_file",
                default_value=_workspace_override(
                    "src/ros2_ws/src/rotom_vision/config/camera_calibration.yaml",
                    os.path.join(vision_share, "config", "camera_calibration.yaml"),
                ),
            ),
            DeclareLaunchArgument(
                "control_params_file",
                default_value=_workspace_override(
                    "src/ros2_ws/src/rotom_control/config/rotom_control.yaml",
                    os.path.join(
                        control_share,
                        "config",
                        "rotom_control.yaml",
                    ),
                ),
            ),
            DeclareLaunchArgument(
                "camera_device",
                default_value="/dev/v4l/by-id/usb-3D_USB_Camera_3D_USB_Camera_01.00.00-video-index0",
            ),
            DeclareLaunchArgument(
                "camera_pixel_format",
                default_value="YUYV",
            ),
            DeclareLaunchArgument(
                "camera_image_size",
                default_value="[1600, 600]",
            ),
            LogInfo(
                condition=IfCondition(enable_servo),
                msg="Marker follow mode: LIVE (custom damped Jacobian visual servo enabled).",
            ),
            LogInfo(
                condition=UnlessCondition(enable_servo),
                msg="Marker follow mode: DRY (marker_follower computes twists but arm motion stays disabled).",
            ),
            LogInfo(
                condition=IfCondition(use_rviz),
                msg="Marker follow RViz enabled.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(description_launch),
                launch_arguments={
                    "use_rviz": use_rviz,
                    "use_jsp": "false",
                    "use_jsp_gui": "false",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(vision_launch),
                launch_arguments={
                    "params_file": params_file,
                    "start_camera": start_camera,
                    "start_control": start_control,
                    "camera_device": camera_device,
                    "camera_pixel_format": camera_pixel_format,
                    "camera_image_size": camera_image_size,
                    "calibration_file": calibration_file,
                    "control_params_file": control_params_file,
                    "enable_servo": enable_servo,
                }.items(),
            ),
        ]
    )
