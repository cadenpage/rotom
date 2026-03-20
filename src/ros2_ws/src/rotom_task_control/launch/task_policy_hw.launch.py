import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    task_share = get_package_share_directory('rotom_task_control')
    vision_share = get_package_share_directory('rotom_vision')
    policy_config = os.path.join(task_share, 'config', 'policy_inference.yaml')
    wrapper = '/home/caden/Documents/rotom/scripts/ros2_repo_python_env.sh'

    policy_repo_id_arg = DeclareLaunchArgument(
        'policy_repo_id',
        default_value='caden-ut/rotom_task_teleop_diffusion',
        description='Preferred Hugging Face Hub policy repo id.',
    )
    local_policy_path_arg = DeclareLaunchArgument(
        'local_policy_path',
        default_value='/home/caden/Documents/rotom/policies/rotom_task_teleop_diffusion/pretrained_model',
        description='Optional local fallback path for the policy checkpoint.',
    )
    start_enabled_arg = DeclareLaunchArgument(
        'start_enabled',
        default_value='false',
        description='Whether to start policy inference enabled immediately.',
    )
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/v4l/by-id/usb-3D_USB_Camera_3D_USB_Camera_01.00.00-video-index0',
        description='Video device for the policy camera stream.',
    )
    camera_pixel_format_arg = DeclareLaunchArgument(
        'camera_pixel_format',
        default_value='YUYV',
        description='Pixel format for the policy camera stream.',
    )
    camera_image_size_arg = DeclareLaunchArgument(
        'camera_image_size',
        default_value='[1280,480]',
        description='Stereo camera image size for the policy stream.',
    )
    camera_output_encoding_arg = DeclareLaunchArgument(
        'camera_output_encoding',
        default_value='mono8',
        description='Output encoding for the policy camera stream.',
    )

    return LaunchDescription(
        [
            policy_repo_id_arg,
            local_policy_path_arg,
            start_enabled_arg,
            camera_device_arg,
            camera_pixel_format_arg,
            camera_image_size_arg,
            camera_output_encoding_arg,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(task_share, 'launch', 'task_control_hw.launch.py')),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(vision_share, 'launch', 'camera_stream.launch.py')),
                launch_arguments={
                    'camera_device': LaunchConfiguration('camera_device'),
                    'camera_pixel_format': LaunchConfiguration('camera_pixel_format'),
                    'camera_image_size': LaunchConfiguration('camera_image_size'),
                    'camera_output_encoding': LaunchConfiguration('camera_output_encoding'),
                }.items(),
            ),
            ExecuteProcess(
                cmd=[
                    wrapper,
                    '-m',
                    'rotom_task_control.policy_inference_node',
                    '--ros-args',
                    '--params-file',
                    policy_config,
                    '-p',
                    ['policy_repo_id:=', LaunchConfiguration('policy_repo_id')],
                    '-p',
                    ['local_policy_path:=', LaunchConfiguration('local_policy_path')],
                    '-p',
                    ['start_enabled:=', LaunchConfiguration('start_enabled')],
                ],
                output='screen',
                sigterm_timeout='5',
                sigkill_timeout='5',
            ),
        ]
    )
