import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = FindPackageShare('mam_eurobot_2026')

    # --- 1. CONFIGURATION ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_slam = LaunchConfiguration('launch_slam')
    launch_rviz = LaunchConfiguration('launch_rviz')
    gazebo_headless = LaunchConfiguration('gazebo_headless')

    # --- 2. CHEMINS DES RESSOURCES (LA CORRECTION EST ICI) ---
    # On ajoute le dossier 'models' au chemin de recherche Gazebo
    # Gazebo cherchera dans install/share/mam_eurobot_2026 ET install/share/mam_eurobot_2026/models
    models_path = PathJoinSubstitution([pkg_path, 'models'])
    
    gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        [pkg_path, ':', models_path]
    )

    # --- 3. SLAM TOOLBOX ---
    slam_params_file = os.path.join(
        get_package_share_directory('mam_eurobot_2026'),
        'config', 'slam', 'mapper_params_online_async.yaml'
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file, 
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(launch_slam),
    )

    # --- 4. ENTITÉS DYNAMIQUES ---
    # Le terrain et les caisses sont déjà dans arena_world.sdf
    world_entities = [
        # Mur de test (Optionnel)
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_sim", "create",
                "-file", "file://models/wall_test/wall.sdf",
                "-name", "wall",
                "-x", "0", "-y", "0", "-z", "0.0"
            ],
            output="screen"
        ),
        # Robot
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_sim", "create",
                "-file", "file://models/simple_robot",
                "-name", "simple_robot",
                "-x", "0", "-y", "0", "-z", "0.05", "-Y", "1.57075"
            ],
            output="screen"
        ),
    ]

    # --- 5. BRIDGES & TF ---
    bridges_and_tf = [
        # A. Horloge Gazebo -> ROS2
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            output='screen',
            arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        ),
        
        # B. Commandes Robot
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='cmd_vel_bridge',
            output='screen',
            arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist']
        ),

        # C. Caméras
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_bridges',
            output='screen',
            arguments=[
                '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            ],
            condition=IfCondition(LaunchConfiguration('launch_camera')),
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='left_camera_bridges',
            output='screen',
            arguments=[
                '/left_cam/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                '/left_cam/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            ],
            condition=IfCondition(LaunchConfiguration('launch_camera')),
        ),

        # D. LiDAR
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='lidar_bridge',
            output='screen',
            arguments=['/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'],
            remappings=[('/lidar', '/scan')],
            condition=IfCondition(LaunchConfiguration('launch_lidar')),
        ),
        
        # E. Odométrie & TF Global
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='odom_bridge',
            output='screen',
            arguments=['/model/simple_robot/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry'],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='tf_bridge',
            output='screen',
            arguments=['/model/simple_robot/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'],
            remappings=[('/model/simple_robot/tf', '/tf')]
        ),

        # F. TF Statiques
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='chassis_to_base_scan',
            arguments=['0', '0', '0.10', '0', '0', '0', 'simple_robot/chassis', 'simple_robot/base_scan'],
            condition=IfCondition(LaunchConfiguration('launch_lidar')),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_scan_to_lidar',
            arguments=['0', '0', '0.01', '0', '0', '0', 'simple_robot/base_scan', 'simple_robot/base_scan/hls_lfcd_lds'],
            condition=IfCondition(LaunchConfiguration('launch_lidar')),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='chassis_to_camera_link',
            arguments=['-0.085', '0', '0.08', '0', '0', '0', 'simple_robot/chassis', 'simple_robot/camera_link'],
            condition=IfCondition(LaunchConfiguration('launch_camera')),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_link_to_sensor',
            arguments=['0.035', '0', '0.12', '0', '0.5236', '0', 'simple_robot/camera_link', 'simple_robot/camera_sensor'],
            condition=IfCondition(LaunchConfiguration('launch_camera')),
        ),

        # G. RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(launch_rviz),
        ),
    ]

    # --- 6. DESCRIPTION DU LANCEMENT ---
    return LaunchDescription(
        [
            # Arguments
            DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
            DeclareLaunchArgument("world", default_value=PathJoinSubstitution([pkg_path, 'worlds', 'arena_world.sdf'])),
            DeclareLaunchArgument("launch_rviz", default_value='false'),
            DeclareLaunchArgument("launch_slam", default_value='false'),
            DeclareLaunchArgument("launch_camera", default_value='false'),
            DeclareLaunchArgument("launch_lidar", default_value='false'),
            DeclareLaunchArgument("gazebo_headless", default_value='false'),

            # Variables d'environnement
            SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
            SetEnvironmentVariable('QT_X11_NO_MITSHM', '1'),
            SetEnvironmentVariable('GDK_BACKEND', 'x11'),
            SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
            
            # Application de la variable GZ_SIM_RESOURCE_PATH modifiée
            gz_resource_path,

            # Lancement de Gazebo
            ExecuteProcess(
                condition=UnlessCondition(gazebo_headless),
                cmd=["ign", "gazebo", "-r", LaunchConfiguration("world")],
                output="screen"
            ),
            ExecuteProcess(
                condition=IfCondition(gazebo_headless),
                cmd=["ign", "gazebo", "-r", "-s", LaunchConfiguration("world")],
                output="screen"
            ),
        ]
        + world_entities
        + bridges_and_tf
        + [slam_node]
    )