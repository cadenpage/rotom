from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory('rotom_description')

    # --- Build robot_description from xacro (no .urdf file needed) ---
    xacro_path = os.path.join(pkg_share, 'urdf', 'rotom.urdf.xacro')
    robot_description_xml = xacro.process_file(xacro_path).toxml()

    # --- Launch args ---
    use_rviz = LaunchConfiguration('use_rviz')
    use_jsp = LaunchConfiguration('use_jsp')
    use_jsp_gui = LaunchConfiguration('use_jsp_gui')

    # --- robot_state_publisher (publishes TF from robot_description + joint_states) ---
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_xml}],
    )

    # --- joint_state_publisher (headless, no sliders) ---
    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        condition=IfCondition(use_jsp),
        parameters=[{'robot_description': robot_description_xml}],
    )

    # --- joint_state_publisher_gui (sliders) ---
    joint_state_pub_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(use_jsp_gui),
        parameters=[{'robot_description': robot_description_xml}],
    )

    # --- RViz ---
    rviz_config = os.path.join(pkg_share, 'rviz', 'view_robot.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Whether to start RViz.'
        ),
        DeclareLaunchArgument(
            'use_jsp', default_value='false',
            description='Whether to start joint_state_publisher (no GUI).'
        ),
        DeclareLaunchArgument(
            'use_jsp_gui', default_value='false',
            description='Whether to start joint_state_publisher_gui (sliders).'
        ),
        robot_state_publisher_node,
        joint_state_pub,
        joint_state_pub_gui,
        rviz_node,
    ])
