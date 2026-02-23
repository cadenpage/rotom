from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("rotom", package_name="rotom_moveit_config")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("with_static_tf", default_value="true"))

    static_tf_launch = moveit_config.package_path / "launch/static_virtual_joint_tfs.launch.py"
    if static_tf_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(static_tf_launch)),
                condition=IfCondition(LaunchConfiguration("with_static_tf")),
            )
        )

    move_group_ld = generate_move_group_launch(moveit_config)
    for entity in move_group_ld.entities:
        ld.add_action(entity)

    return ld
