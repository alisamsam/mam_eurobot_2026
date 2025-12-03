from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory  # <-- ajouté
import os  # <-- ajouté


def crate_create_cmd(name, x, y, z=0.05, pitch=None, yaw=None):
    cmd = [
        "ros2", "run", "ros_gz_sim", "create",
        "-file", "file://models/crate",
        "-name", name,
        "-x", str(x), "-y", str(y), "-z", str(z),
    ]
    if pitch is not None:
        cmd += ["-P", pitch]
    if yaw is not None:
        cmd += ["-Y", yaw]
    return cmd


def generate_launch_description():
    # Substitution pour Gazebo,
    pkg_path = FindPackageShare('mam_eurobot_2026')

    # Paramètres SLAM Toolbox
    slam_params_file = os.path.join(
        get_package_share_directory('mam_eurobot_2026'),
        'config', 'slam', 'mapper_params_online_async.yaml'
    )

    # Node SLAM Toolbox (async, pour la simu)
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': True}],
    )

    crates = [
        {'name': 'crate1',  'x': -0.675, 'y': -1.30, 'pitch': "3.1415", 'yaw': "1.5708"},
        {'name': 'crate2',  'x': -0.625, 'y': -1.30, 'yaw': "1.5708"},
        {'name': 'crate3',  'x': -0.575, 'y': -1.30, 'pitch': "3.1415", 'yaw': "1.5708"},
        {'name': 'crate4',  'x': -0.525, 'y': -1.30, 'yaw': "1.5708"},
        {'name': 'crate5',  'x':  0.125, 'y': -1.30, 'yaw': "1.5708"},
        {'name': 'crate6',  'x':  0.175, 'y': -1.30, 'yaw': "1.5708"},
        {'name': 'crate7',  'x':  0.225, 'y': -1.30, 'pitch': "3.1415", 'yaw': "1.5708"},
        {'name': 'crate8',  'x':  0.275, 'y': -1.30, 'pitch': "3.1415", 'yaw': "1.5708"},
        {'name': 'crate9',  'x': -0.2,   'y': -0.275},
        {'name': 'crate10', 'x': -0.2,   'y': -0.325, 'pitch': "3.1415"},
        {'name': 'crate11', 'x': -0.2,   'y': -0.375},
        {'name': 'crate12', 'x': -0.2,   'y': -0.425, 'pitch': "3.1415"},
        {'name': 'crate13', 'x': -0.2,   'y':  0.275},
        {'name': 'crate14', 'x': -0.2,   'y':  0.325, 'pitch': "3.1415"},
        {'name': 'crate15', 'x': -0.2,   'y':  0.375, 'pitch': "3.1415"},
        {'name': 'crate16', 'x': -0.2,   'y':  0.425},
        {'name': 'crate17', 'x': -0.825, 'y': -0.325, 'pitch': "3.1415"},
        {'name': 'crate18', 'x': -0.825, 'y': -0.375, 'pitch': "3.1415"},
        {'name': 'crate19', 'x': -0.825, 'y': -0.425},
        {'name': 'crate20', 'x': -0.825, 'y': -0.475},
    ]

    spawn_crates = [
        ExecuteProcess(cmd=crate_create_cmd(**crate), output="screen")
        for crate in crates
    ]

    beacons = [
        # y = -1.55 (bord bas)
        {'name': 'beaconB1', 'file': 'beacons/beaconB.sdf', 'x':  0.95, 'y': -1.55, 'z': 0.5},
        {'name': 'beaconB2', 'file': 'beacons/beaconB.sdf', 'x': -0.95, 'y': -1.55, 'z': 0.5},
        {'name': 'beaconY1', 'file': 'beacons/beaconY.sdf', 'x':  0.0,  'y': -1.55, 'z': 0.5},

        # y = 1.55 (bord haut)
        {'name': 'beaconB3', 'file': 'beacons/beaconB.sdf', 'x':  0.0,  'y':  1.55, 'z': 0.5},
        {'name': 'beaconY2', 'file': 'beacons/beaconY.sdf', 'x': -0.95, 'y':  1.55, 'z': 0.5},
        {'name': 'beaconY3', 'file': 'beacons/beaconY.sdf', 'x':  0.95, 'y':  1.55, 'z': 0.5},
    ]

    spawn_beacons = [
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_sim", "create",
                "-file", f"file://models/{b['file']}",
                "-name", b['name'],
                "-x", str(b['x']), "-y", str(b['y']), "-z", str(b['z']),
            ],
            output="screen"
        )
        for b in beacons
    ]

    # globalisation
    world_entities = spawn_crates + spawn_beacons + [

        # Mur de test
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_sim", "create",
                "-file", "file://models/wall_test/wall.sdf",
                "-name", "wall",
                "-x", "0", "-y", "0", "-z", "0.0"
            ],
            output="screen"
        ),

        # Robot (0,0) monde -> (1000mm, 1500mm) sur le tapis
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_sim", "create",
                "-file", "file://models/simple_robot",
                "-name", "simple_robot",
                "-x", "0", "-y", "0", "-z", "0.05", "-Y", "3.1415"
            ],
            output="screen"
        ),
    ]

    use_rviz = LaunchConfiguration('launch_rviz')
    gazebo_headless = LaunchConfiguration('gazebo_headless')

    # ======================
    #   BRIDGES, TF, RVIZ
    # ======================
    bridges_and_tf = [

        # Horloge Gazebo -> ROS2 (use_sim_time)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            output='screen',
            arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        ),

        # cmd_vel
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='cmd_vel_bridge',
            output='screen',
            arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist']
        ),

        # Camera frontale
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_bridges',
            output='screen',
            arguments=[
                '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            ],
        ),

        # Seconde caméra (left corner)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='left_camera_bridges',
            output='screen',
            arguments=[
                '/left_cam/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                '/left_cam/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            ],
        ),

        # Lidar
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='lidar_bridge',
            output='screen',
            arguments=['/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'],
            remappings=[('/lidar', '/scan')]
        ),

        # Odométrie
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='odom_bridge',
            output='screen',
            arguments=['/model/simple_robot/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry'],
        ),

        # TF depuis Gazebo
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='tf_bridge',
            output='screen',
            arguments=['/model/simple_robot/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'],
            remappings=[('/model/simple_robot/tf', '/tf')]
        ),

        # TF statiques pour le lidar
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='chassis_to_base_scan',
            arguments=['0', '0', '0.10', '0', '0', '0',
                       'simple_robot/chassis', 'simple_robot/base_scan']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_scan_to_lidar',
            arguments=['0', '0', '0.01', '0', '0', '0',
                       'simple_robot/base_scan', 'simple_robot/base_scan/hls_lfcd_lds']
        ),
    

        # TF statiques pour la caméra frontale
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='chassis_to_camera_link',
            arguments=['-0.085', '0', '0.08', '0', '0', '0',
                       'simple_robot/chassis', 'simple_robot/camera_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_link_to_sensor',
            arguments=['0.035', '0', '0.12', '0', '0.5236', '0',
                       'simple_robot/camera_link', 'simple_robot/camera_sensor']
        ),

        # RViz (optionnel)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(use_rviz),
        ),
    ]

    # ======================
    #   LAUNCH DESCRIPTION
    # ======================
    return LaunchDescription(
        [

            # Forcer X11 pour les GUI
            SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
            SetEnvironmentVariable('QT_X11_NO_MITSHM', '1'),
            SetEnvironmentVariable('GDK_BACKEND', 'x11'),
            SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),

            # Ressources Gazebo (models, worlds) depuis le package
            SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', pkg_path),

            # Argument "world"
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [pkg_path, 'worlds', 'arena_world.sdf']
                )
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value='false'
            ),
            DeclareLaunchArgument(
                "gazebo_headless",
                default_value='false',
                description='Run Gazebo without the GUI'
            ),

            # Lancer Gazebo avec le monde
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
        + [slam_node]          # <-- SLAM est lancé avec le reste
    )
