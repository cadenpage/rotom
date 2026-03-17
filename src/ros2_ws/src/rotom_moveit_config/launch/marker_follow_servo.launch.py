import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def _workspace_override(relative_path: str, fallback: str) -> str:
    workspace_root = os.environ.get("ROTOM_ROOT", "").strip()
    if workspace_root:
        candidate = os.path.join(workspace_root, relative_path)
        if os.path.exists(candidate):
            return candidate
    return fallback


def generate_launch_description():
    moveit_share = get_package_share_directory("rotom_moveit_config")
    vision_share = get_package_share_directory("rotom_vision")
    control_share = get_package_share_directory("rotom_control")

    default_params = _workspace_override(
        "src/ros2_ws/src/rotom_vision/config/vision_pipeline.yaml",
        os.path.join(vision_share, "config", "vision_pipeline.yaml"),
    )
    default_calibration = _workspace_override(
        "src/ros2_ws/src/rotom_vision/config/camera_calibration.yaml",
        os.path.join(vision_share, "config", "camera_calibration.yaml"),
    )
    default_control_params = _workspace_override(
        "src/ros2_ws/src/rotom_control/config/rotom_control.yaml",
        os.path.join(control_share, "config", "rotom_control.yaml"),
    )
    default_servo_params = _workspace_override(
        "src/ros2_ws/src/rotom_moveit_config/config/moveit_servo.yaml",
        os.path.join(moveit_share, "config", "moveit_servo.yaml"),
    )

    moveit_config = (
        MoveItConfigsBuilder("rotom", package_name="rotom_moveit_config")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    use_rviz = LaunchConfiguration("use_rviz")
    start_camera = LaunchConfiguration("start_camera")
    start_control = LaunchConfiguration("start_control")
    enable_servo = LaunchConfiguration("enable_servo")
    translation_only = LaunchConfiguration("translation_only")
    publish_debug_image = LaunchConfiguration("publish_debug_image")
    params_file = LaunchConfiguration("params_file")
    calibration_file = LaunchConfiguration("calibration_file")
    control_params_file = LaunchConfiguration("control_params_file")
    servo_params_file = LaunchConfiguration("servo_params_file")
    camera_device = LaunchConfiguration("camera_device")
    camera_pixel_format = LaunchConfiguration("camera_pixel_format")
    camera_image_size = LaunchConfiguration("camera_image_size")

    rsp_launch = os.path.join(moveit_share, "launch", "rsp.launch.py")
    static_tf_launch = os.path.join(moveit_share, "launch", "static_virtual_joint_tfs.launch.py")
    rviz_launch = os.path.join(moveit_share, "launch", "moveit_rviz.launch.py")
    workspace_setup = os.path.abspath(
        os.path.join(moveit_share, os.pardir, os.pardir, os.pardir, "setup.bash")
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("start_control", default_value="true"),
            DeclareLaunchArgument(
                "enable_servo",
                default_value="false",
                description="Set true to let marker_follower commands reach MoveIt Servo.",
            ),
            DeclareLaunchArgument(
                "translation_only",
                default_value="true",
                description="Set true to force MoveIt Servo into translation-only Cartesian control.",
            ),
            DeclareLaunchArgument(
                "publish_debug_image",
                default_value="false",
                description="Set true only when you explicitly want /aruco/debug_image during bringup.",
            ),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("calibration_file", default_value=default_calibration),
            DeclareLaunchArgument("control_params_file", default_value=default_control_params),
            DeclareLaunchArgument("servo_params_file", default_value=default_servo_params),
            DeclareLaunchArgument(
                "camera_device",
                default_value="/dev/v4l/by-id/usb-3D_USB_Camera_3D_USB_Camera_01.00.00-video-index0",
            ),
            DeclareLaunchArgument("camera_pixel_format", default_value="YUYV"),
            DeclareLaunchArgument("camera_image_size", default_value="[1600, 600]"),
            LogInfo(
                condition=IfCondition(enable_servo),
                msg="Marker follow mode: LIVE (marker_follower twist output enabled, MoveIt Servo started).",
            ),
            LogInfo(
                condition=IfCondition(translation_only),
                msg="MoveIt Servo control dimensions: translation only with free orientation drift.",
            ),
            LogInfo(
                condition=UnlessCondition(enable_servo),
                msg="Marker follow mode: DRY (marker_follower computes twists but will not move the arm).",
            ),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(rsp_launch)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(static_tf_launch)),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rviz_launch),
                condition=IfCondition(use_rviz),
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
                        "image_size": ParameterValue(camera_image_size, value_type=List[int]),
                        "output_encoding": "bgr8",
                    }
                ],
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
                executable="stereo_splitter",
                name="stereo_splitter",
                output="screen",
                parameters=[
                    params_file,
                    {"calibration_file": ParameterValue(calibration_file, value_type=str)},
                ],
            ),
            Node(
                package="rotom_vision",
                executable="aruco_tracker",
                name="aruco_tracker",
                output="screen",
                parameters=[
                    params_file,
                    {"publish_debug_image": ParameterValue(publish_debug_image, value_type=bool)},
                ],
            ),
            Node(
                condition=UnlessCondition(enable_servo),
                package="rotom_vision",
                executable="marker_follower",
                name="marker_follower",
                output="screen",
                parameters=[
                    params_file,
                    {"enable_twist_output": ParameterValue(enable_servo, value_type=bool)},
                ],
            ),
            TimerAction(
                period=7.0,
                condition=IfCondition(enable_servo),
                actions=[
                    Node(
                        package="rotom_vision",
                        executable="marker_follower",
                        name="marker_follower",
                        output="screen",
                        parameters=[
                            params_file,
                            {"enable_twist_output": True},
                        ],
                    )
                ],
            ),
            Node(
                condition=IfCondition(enable_servo),
                package="moveit_servo",
                executable="servo_node_main",
                name="servo_node",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    servo_params_file,
                ],
            ),
            TimerAction(
                period=3.0,
                condition=IfCondition(enable_servo),
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "bash",
                            "-lc",
                            (
                                f"source '{workspace_setup}' && "
                                "ros2 service call /servo_node/start_servo std_srvs/srv/Trigger '{}'"
                            ),
                        ],
                        output="screen",
                    )
                ],
            ),
            TimerAction(
                period=4.0,
                condition=IfCondition(enable_servo),
                actions=[
                    ExecuteProcess(
                        condition=IfCondition(translation_only),
                        cmd=[
                            "bash",
                            "-lc",
                            (
                                f"source '{workspace_setup}' && "
                                "ros2 service call /servo_node/change_control_dimensions "
                                "moveit_msgs/srv/ChangeControlDimensions "
                                "'{control_x_translation: true, control_y_translation: true, "
                                "control_z_translation: true, control_x_rotation: false, "
                                "control_y_rotation: false, control_z_rotation: false}'"
                            ),
                        ],
                        output="screen",
                    )
                ],
            ),
            TimerAction(
                period=5.0,
                condition=IfCondition(enable_servo),
                actions=[
                    ExecuteProcess(
                        condition=IfCondition(translation_only),
                        cmd=[
                            "bash",
                            "-lc",
                            (
                                f"source '{workspace_setup}' && "
                                "ros2 service call /servo_node/change_drift_dimensions "
                                "moveit_msgs/srv/ChangeDriftDimensions "
                                "'{drift_x_translation: false, drift_y_translation: false, "
                                "drift_z_translation: false, drift_x_rotation: true, "
                                "drift_y_rotation: true, drift_z_rotation: true, "
                                "transform_jog_frame_to_drift_frame: {translation: {x: 0.0, y: 0.0, z: 0.0}, "
                                "rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'"
                            ),
                        ],
                        output="screen",
                    )
                ],
            ),
            TimerAction(
                period=6.0,
                condition=IfCondition(enable_servo),
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "bash",
                            "-lc",
                            (
                                f"source '{workspace_setup}' && "
                                "ros2 service call /servo_node/reset_servo_status std_srvs/srv/Empty '{}'"
                            ),
                        ],
                        output="screen",
                    )
                ],
            ),
        ]
    )
