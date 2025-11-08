from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_path = FindPackageShare('mam_eurobot_2026')
    
    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            pkg_path
        ),
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution([pkg_path, 'worlds', 'arena_world.sdf'])
        ),

        # Lancer Gazebo avec le monde
        ExecuteProcess(
            cmd=["ign", "gazebo", "-r", LaunchConfiguration("world")],
            output="screen"
        ),

        # === 8 crates depuis (700mm, 2800mm) -> (-0.30, 1.30) m, pas 0.10 m sur X, Y constant ===
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_sim", "create",
                "-file", "file://models/crate",
                "-name", "crate1",
                "-x", "-0.675", "-y", "-1.30", "-z", "0.05",
                "-P", "3.1415",
                "-Y", "1.5708"
            ],
            output="screen"
        ),
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_sim", "create",
                "-file", "file://models/crate",
                "-name", "crate2",
                "-x", "-0.625", "-y", "-1.30", "-z", "0.05",
                "-Y", "1.5708"
            ],
            output="screen"
        ),
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_sim", "create",
                "-file", "file://models/crate",
                "-name", "crate3",
                "-x", "-0.575", "-y", "-1.30", "-z", "0.05",
                "-P", "3.1415",
                "-Y", "1.5708"
            ],
            output="screen"
        ),
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_sim", "create",
                "-file", "file://models/crate",
                "-name", "crate4",
                "-x", "-0.525", "-y", "-1.30", "-z", "0.05",
                "-Y", "1.5708"
            ],
            output="screen"
        ),
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_sim", "create",
                "-file", "file://models/crate",
                "-name", "crate5",
                "-x", "0.125", "-y", "-1.30", "-z", "0.05",
                "-Y", "1.5708"
            ],
            output="screen"
        ),
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_sim", "create",
                "-file", "file://models/crate",
                "-name", "crate6",
                "-x", "0.175", "-y", "-1.30", "-z", "0.05",
                "-Y", "1.5708"
            ],
            output="screen"
        ),
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_sim", "create",
                "-file", "file://models/crate",
                "-name", "crate7",
                "-x", "0.225", "-y", "-1.30", "-z", "0.05",
                "-P", "3.1415",
                "-Y", "1.5708"
            ],
            output="screen"
        ),
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_sim", "create",
                "-file", "file://models/crate",
                "-name", "crate8",
                "-x", "0.275", "-y", "-1.30", "-z", "0.05",
                "-P", "3.1415",
                "-Y", "1.5708"
            ],
            output="screen"
        ),
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_sim", "create",
                "-file", "file://models/crate",
                "-name", "crate9",
                "-x", "-0.2", "-y", "-0.275", "-z", "0.05",
            ],
            output="screen"
        ),
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_sim", "create",
                "-file", "file://models/crate",     
                "-name", "crate10",
                "-x", "-0.2", "-y", "-0.325", "-z", "0.05",
                "-P", "3.1415",
            ],
            output="screen"
        ),
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_sim", "create",
                "-file", "file://models/crate",
                "-name", "crate11",
                "-x", "-0.2", "-y", "-0.375", "-z", "0.05",
            ],
            output="screen"
        ),
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_sim", "create",
                "-file", "file://models/crate",
                "-name", "crate12",
                "-x", "-0.2", "-y", "-0.425", "-z", "0.05",
                "-P", "3.1415",
            ],
            output="screen"
        ),
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_sim", "create",
                "-file", "file://models/crate",
                "-name", "crate13",
                "-x", "-0.2", "-y", "0.275", "-z", "0.05",
            ],
            output="screen"
        ),
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_sim", "create",
                "-file", "file://models/crate",     
                "-name", "crate14",
                "-x", "-0.2", "-y", "0.325", "-z", "0.05",
                "-P", "3.1415",
            ],
            output="screen"
        ),
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_sim", "create",
                "-file", "file://models/crate",
                "-name", "crate15",
                "-x", "-0.2", "-y", "0.375", "-z", "0.05",
                "-P", "3.1415",
            ],
            output="screen"
        ),
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_sim", "create",
                "-file", "file://models/crate",
                "-name", "crate16",
                "-x", "-0.2", "-y", "0.425", "-z", "0.05",
            ],
            output="screen"
        ),
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_sim", "create",
                "-file", "file://models/crate",
                "-name", "crate17",
                "-x", "-0.825", "-y", "-0.325", "-z", "0.05",
                "-P", "3.1415",
            ],
            output="screen"
        ),
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_sim", "create",
                "-file", "file://models/crate",     
                "-name", "crate18",
                "-x", "-0.825", "-y", "-0.375", "-z", "0.05",
                "-P", "3.1415",
            ],
            output="screen"
        ),
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_sim", "create",
                "-file", "file://models/crate",
                "-name", "crate19",
                "-x", "-0.825", "-y", "-0.425", "-z", "0.05",
            ],
            output="screen"
        ),
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_sim", "create",
                "-file", "file://models/crate",
                "-name", "crate20",
                "-x", "-0.825", "-y", "-0.475", "-z", "0.05",
            ],
            output="screen"
        ),

        #beacons
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_sim", "create",
                "-file", "file://models/beacons/beaconY.sdf",
                "-name", "beaconY",
                "-x", "-1.5", "-y", "-1.0", "-z", "0.0"
            ],
            output="screen"
        ),
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_sim", "create",
                "-file", "file://models/beacons/beaconB.sdf",
                "-name", "beaconB",
                "-x", "1.5", "-y", "1.0", "-z", "0.0"
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

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='cmd_vel_bridge',
            output='screen',
            arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist']
        ),
        
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_image_bridge',
            output='screen',
            arguments=['/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'],
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_info_bridge',
            output='screen',
            arguments=['/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
        ),
        #second camera (left corner)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_bridges',
            output='screen',
            arguments=[
                '/left_cam/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                '/left_cam/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            ],
        ),
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        )
    ])
