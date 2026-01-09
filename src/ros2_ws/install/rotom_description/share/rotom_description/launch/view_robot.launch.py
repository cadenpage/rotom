from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory
import os




def generate_launch_description():
    # 1) Find the installed "share" directory for your package.
    #    This is how ROS2 safely locates files after colcon installs your package.
    pkg_share = get_package_share_directory('rotom_description')

    # 2) Build the full path to your URDF file inside that package.
    #    Assumes you have: rotom_description/urdf/rotom.urdf
    urdf_path = os.path.join(pkg_share, 'urdf', 'rotom.urdf.xacro')

    



    # 3) Read the URDF file into a string.
    #    robot_state_publisher expects the actual XML contents in the parameter.
    # with open(urdf_path, 'r') as f:
    #     robot_description = f.read()

    # Alternative: use xacro to process the file on-the-fly.
    robot_description = ParameterValue(
    Command(['xacro ', urdf_path]), value_type=str)

    # 4) Start robot_state_publisher.
    #    This node publishes TF transforms for every link/joint in the URDF.
    #    RViz uses TF + robot_description to draw the robot.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }]
    )

    # 5) Start RViz.
    #    With no config, it opens default RViz; you then add "RobotModel"
    #    and set Fixed Frame (often base_link).
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    # 6) LaunchDescription is the list of things to start.
    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
    ])
