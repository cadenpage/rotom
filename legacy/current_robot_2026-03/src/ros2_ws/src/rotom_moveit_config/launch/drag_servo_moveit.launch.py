import os

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
    description_share = get_package_share_directory("rotom_description")
    control_share = get_package_share_directory("rotom_control")

    default_params = _workspace_override(
        "src/ros2_ws/src/rotom_vision/config/vision_pipeline.yaml",
        os.path.join(vision_share, "config", "vision_pipeline.yaml"),
    )
    default_control_params = _workspace_override(
        "src/ros2_ws/src/rotom_control/config/rotom_control.yaml",
        os.path.join(control_share, "config", "rotom_control.yaml"),
    )
    default_servo_params = _workspace_override(
        "src/ros2_ws/src/rotom_moveit_config/config/moveit_servo.yaml",
        os.path.join(moveit_share, "config", "moveit_servo.yaml"),
    )
    dragger_rviz = os.path.join(description_share, "rviz", "servo_dragger.rviz")

    moveit_config = (
        MoveItConfigsBuilder("rotom", package_name="rotom_moveit_config")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    use_rviz = LaunchConfiguration("use_rviz")
    start_control = LaunchConfiguration("start_control")
    enable_servo = LaunchConfiguration("enable_servo")
    translation_only = LaunchConfiguration("translation_only")
    params_file = LaunchConfiguration("params_file")
    control_params_file = LaunchConfiguration("control_params_file")
    servo_params_file = LaunchConfiguration("servo_params_file")

    rsp_launch = os.path.join(moveit_share, "launch", "rsp.launch.py")
    static_tf_launch = os.path.join(moveit_share, "launch", "static_virtual_joint_tfs.launch.py")
    rviz_launch = os.path.join(moveit_share, "launch", "moveit_rviz.launch.py")
    workspace_setup = os.path.abspath(
        os.path.join(moveit_share, os.pardir, os.pardir, os.pardir, "setup.bash")
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("start_control", default_value="true"),
            DeclareLaunchArgument(
                "enable_servo",
                default_value="false",
                description="Set true to let the RViz dragger drive MoveIt Servo and the physical arm.",
            ),
            DeclareLaunchArgument(
                "translation_only",
                default_value="true",
                description="Set true to force MoveIt Servo into translation-only Cartesian control.",
            ),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("control_params_file", default_value=default_control_params),
            DeclareLaunchArgument("servo_params_file", default_value=default_servo_params),
            LogInfo(
                condition=IfCondition(enable_servo),
                msg="Drag servo mode: LIVE (RViz dragger drives MoveIt Servo).",
            ),
            LogInfo(
                condition=IfCondition(translation_only),
                msg="MoveIt Servo control dimensions: translation only with free orientation drift.",
            ),
            LogInfo(
                condition=UnlessCondition(translation_only),
                msg="MoveIt Servo control dimensions: full 6-DOF pose tracking.",
            ),
            LogInfo(
                condition=UnlessCondition(enable_servo),
                msg="Drag servo mode: DRY (RViz dragger target updates without moving the arm).",
            ),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(rsp_launch)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(static_tf_launch)),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rviz_launch),
                condition=IfCondition(use_rviz),
                launch_arguments={"rviz_config": dragger_rviz}.items(),
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
