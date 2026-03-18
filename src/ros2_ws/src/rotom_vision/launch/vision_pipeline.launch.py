import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
    pkg_share = get_package_share_directory("rotom_vision")
    default_params = _workspace_override(
        "src/ros2_ws/src/rotom_vision/config/vision_pipeline.yaml",
        os.path.join(pkg_share, "config", "vision_pipeline.yaml"),
    )
    default_calibration = _workspace_override(
        "src/ros2_ws/src/rotom_vision/config/camera_calibration.yaml",
        os.path.join(pkg_share, "config", "camera_calibration.yaml"),
    )
    default_calibration = default_calibration if os.path.exists(default_calibration) else ""

    params_file = LaunchConfiguration("params_file")
    start_camera = LaunchConfiguration("start_camera")
    camera_device = LaunchConfiguration("camera_device")
    camera_pixel_format = LaunchConfiguration("camera_pixel_format")
    camera_image_size = LaunchConfiguration("camera_image_size")
    camera_output_encoding = LaunchConfiguration("camera_output_encoding")
    calibration_file = LaunchConfiguration("calibration_file")
    enable_follower_output = LaunchConfiguration("enable_follower_output")

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument(
                "camera_device",
                default_value="/dev/v4l/by-id/usb-3D_USB_Camera_3D_USB_Camera_01.00.00-video-index0",
            ),
            DeclareLaunchArgument("camera_pixel_format", default_value="YUYV"),
            DeclareLaunchArgument("camera_image_size", default_value="[1600, 600]"),
            DeclareLaunchArgument("camera_output_encoding", default_value="mono8"),
            DeclareLaunchArgument("calibration_file", default_value=default_calibration),
            DeclareLaunchArgument("enable_follower_output", default_value="false"),
            Node(
                condition=IfCondition(start_camera),
                package="v4l2_camera",
                executable="v4l2_camera_node",
                name="v4l2_camera",
                output="screen",
                respawn=True,
                respawn_delay=1.0,
                parameters=[
                    {
                        "video_device": camera_device,
                        "pixel_format": camera_pixel_format,
                        "image_size": ParameterValue(camera_image_size, value_type=List[int]),
                        "output_encoding": camera_output_encoding,
                    }
                ],
            ),
            Node(
                package="rotom_vision",
                executable="stereo_splitter",
                name="stereo_splitter",
                output="screen",
                emulate_tty=True,
                parameters=[
                    params_file,
                    {"calibration_file": ParameterValue(calibration_file, value_type=str)},
                ],
                additional_env={"PYTHONUNBUFFERED": "1"},
            ),
            Node(
                package="rotom_vision",
                executable="aruco_tracker",
                name="aruco_tracker",
                output="screen",
                emulate_tty=True,
                parameters=[params_file],
                additional_env={"PYTHONUNBUFFERED": "1"},
            ),
            Node(
                package="rotom_vision",
                executable="marker_follower",
                name="marker_follower",
                output="screen",
                emulate_tty=True,
                parameters=[
                    params_file,
                    {"enable_output": ParameterValue(enable_follower_output, value_type=bool)},
                ],
                additional_env={"PYTHONUNBUFFERED": "1"},
            ),
        ]
    )
