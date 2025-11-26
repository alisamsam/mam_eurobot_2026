from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('mam_eurobot_2026')

    arena_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'arena.launch.py'))
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'slam.launch.py'))
    )

    rviz_config = os.path.join(pkg_share, 'config', 'slam_rviz.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        remappings=[('/scan', '/scan')],
    )

    # Start the arena immediately, then start slam after a short delay so the simulation is up.
    # Start RViz after slam is ready
    return LaunchDescription([
        arena_launch,
        TimerAction(period=5.0, actions=[slam_launch]),
        TimerAction(period=10.0, actions=[rviz_node])
    ])
