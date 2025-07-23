#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Get package directory
    pkg_share = FindPackageShare(package='tadeo_ecar_description').find('tadeo_ecar_description')
    
    # File paths
    urdf_file = os.path.join(pkg_share, 'urdf', 'tadeo_ecar.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'empty_world_ignition.sdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'tadeo_ecar.rviz')
    
    # Process the URDF file
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    return LaunchDescription([
        # Gazebo Ignition con GUI
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', '-v', '4', world_file, '--force-version', '6'],
            name='ign_gazebo',
            output='screen',
        ),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }],
        ),
        
        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True
            }],
        ),
        
        # Spawn robot in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_tadeo_ecar',
            arguments=[
                '-name', 'tadeo_ecar',
                '-topic', 'robot_description',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.1'
            ],
            output='screen',
        ),
        
        # ROS-Gazebo Bridge for control and sensors
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                '/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
                '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
                '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            ],
            parameters=[{
                'use_sim_time': True
            }],
            output='screen',
        ),
        
        # Bridge for camera sensor
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_bridge',
            arguments=[
                '/camera@sensor_msgs/msg/Image@ignition.msgs.Image',
                '/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            ],
            parameters=[{
                'use_sim_time': True
            }],
            output='screen',
        ),
        
        # RViz2 with sensors visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{
                'use_sim_time': True
            }],
        ),
    ])