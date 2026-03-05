import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("rotom_vision")
    default_params = os.path.join(pkg_share, "config", "vision_pipeline.yaml")

    params_file = LaunchConfiguration("params_file")
    start_camera = LaunchConfiguration("start_camera")
    camera_device = LaunchConfiguration("camera_device")
    camera_pixel_format = LaunchConfiguration("camera_pixel_format")
    enable_servo = LaunchConfiguration("enable_servo")

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("start_camera", default_value="false"),
            DeclareLaunchArgument(
                "camera_device",
                default_value="/dev/v4l/by-id/usb-3D_USB_Camera_3D_USB_Camera_01.00.00-video-index0",
            ),
            DeclareLaunchArgument("camera_pixel_format", default_value="YUYV"),
            DeclareLaunchArgument(
                "enable_servo",
                default_value="false",
                description="Set true to enable visual_servo and marker_follower twist output.",
            ),
            Node(
                condition=IfCondition(start_camera),
                package="v4l2_camera",
                executable="v4l2_camera_node",
                name="v4l2_camera",
                output="screen",
                parameters=[
                    {
                        "video_device": camera_device,
                        "pixel_format": camera_pixel_format,
                        "image_size": [1280, 480],
                        "output_encoding": "bgr8",
                    }
                ],
            ),
            Node(
                package="rotom_vision",
                executable="stereo_splitter",
                name="stereo_splitter",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="rotom_vision",
                executable="aruco_tracker",
                name="aruco_tracker",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="rotom_vision",
                executable="marker_follower",
                name="marker_follower",
                output="screen",
                parameters=[
                    params_file,
                    {"enable_twist_output": enable_servo},
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
