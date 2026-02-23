from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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

    launch_package_path = moveit_config.package_path

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("db", default_value="false"))
    ld.add_action(DeclareLaunchArgument("use_rviz", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_ros2_control", default_value="false"))

    virtual_joints_launch = launch_package_path / "launch/static_virtual_joint_tfs.launch.py"
    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_package_path / "launch/rsp.launch.py")),
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_package_path / "launch/move_group.launch.py")),
            launch_arguments={"with_static_tf": "false"}.items(),
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_package_path / "launch/moveit_rviz.launch.py")),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_package_path / "launch/warehouse_db.launch.py")),
            condition=IfCondition(LaunchConfiguration("db")),
        )
    )

    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                str(moveit_config.package_path / "config/ros2_controllers.yaml"),
            ],
            condition=IfCondition(LaunchConfiguration("use_ros2_control")),
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_package_path / "launch/spawn_controllers.launch.py")),
            condition=IfCondition(LaunchConfiguration("use_ros2_control")),
        )
    )

    return ld
