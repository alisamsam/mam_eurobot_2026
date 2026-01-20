import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mam_eurobot_2026')
    default_params = os.path.join(pkg_share, 'config', 'gripper', 'gripper_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Gripper controller parameters file',
        ),
        Node(
            package='mam_eurobot_2026',
            executable='gripper_controller',
            name='gripper_controller',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
