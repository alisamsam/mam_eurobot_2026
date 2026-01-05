import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Package share directory
    pkg_share = get_package_share_directory('mam_eurobot_2026')

    # Arguments de lancement
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_name = LaunchConfiguration('robot_name', default='mecanum_robot')
    
    # Chemin vers le fichier XACRO
    xacro_file = os.path.join(pkg_share, 'models', 'simple_robot', 'URDF', 'model.xacro')

    # Traiter le fichier XACRO pour obtenir la description du robot en XML
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    # Nœud pour publier l'état du robot (transformations TF)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}],
        arguments=[robot_description]
    )

    # Nœud pour spawner le robot dans Gazebo
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_description,
                   '-name', robot_name,
                   '-allow_renaming', 'true',
                   '-z', '0.1']
    )

    return LaunchDescription([
        robot_state_publisher_node,
        spawn_entity_node
    ])