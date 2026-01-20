from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    nav2_bringup_dir = '/opt/ros/humble/share/nav2_bringup'
    pkg_share = get_package_share_directory('mam_eurobot_2026')
    map_yaml = os.path.join(pkg_share, 'my_arena_map.yaml')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2', 'nav2_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_yaml,
                'params_file': nav2_params,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }.items()
        )
    ])
