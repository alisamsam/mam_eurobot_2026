import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    # Package share directory
    pkg_share = get_package_share_directory('mam_eurobot_2026')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Paths
    xacro_file = os.path.join(pkg_share, 'models', 'simple_robot', 'URDF', 'model.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'arena_world.sdf')

    # Process xacro to get robot description
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    # 1. Launch Ignition Gazebo
    # On lance Ignition avec le fichier monde
    launch_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_file],
        output='screen'
    )

    # 2. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}]
    )

    # 3. Spawn Robot (Méthode Ignition)
    # Utilise ros_gz_sim pour créer l'entité
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_description,
                   '-name', 'mecanum_robot',
                   '-allow_renaming', 'true',
                   '-z', '0.3'] # On le met un peu en hauteur pour éviter qu'il ne passe sous le sol
    )

    # 4. Bridge ROS <-> Ignition
    # C'est CRUCIAL : Le pont permet de synchroniser l'horloge (/clock)
    # Sans ça, les contrôleurs attendent indéfiniment le temps
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )

    # 5. Spawners des contrôleurs
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "60"],
        output="screen",
    )

    mecanum_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_drive_controller", "--controller-manager-timeout", "60"],
        output="screen",
    )

    # 6. Orchestration (Event Handlers)
    # On attend que le robot soit spawn pour lancer les contrôleurs
    load_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    load_mecanum_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[mecanum_drive_controller_spawner],
        )
    )

    return LaunchDescription([
        launch_gazebo,
        robot_state_publisher_node,
        spawn_entity_node,
        bridge_node, # Ne pas oublier d'ajouter le bridge ici
        load_joint_state_broadcaster,
        load_mecanum_controller,
    ])