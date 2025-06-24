#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get package directory
    pkg_share = FindPackageShare(package='tadeo_ecar_perception').find('tadeo_ecar_perception')
    
    # Configuration files
    robot_config = os.path.join(
        FindPackageShare('tadeo_ecar_config').find('tadeo_ecar_config'),
        'config', 'robot_params.yaml'
    )
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='true',
        description='Enable camera processing'
    )
    
    enable_lidar_arg = DeclareLaunchArgument(
        'enable_lidar',
        default_value='true',
        description='Enable LiDAR processing'
    )
    
    enable_imu_arg = DeclareLaunchArgument(
        'enable_imu',
        default_value='true',
        description='Enable IMU processing'
    )
    
    enable_fusion_arg = DeclareLaunchArgument(
        'enable_fusion',
        default_value='true',
        description='Enable sensor fusion'
    )
    
    # Perception nodes group
    perception_nodes = GroupAction([
        # Camera Processor Node
        Node(
            package='tadeo_ecar_perception',
            executable='camera_processor_node',
            name='camera_processor',
            output='screen',
            condition=lambda: LaunchConfiguration('enable_camera').perform() == 'true',
            parameters=[
                robot_config,
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'processing.frequency': 10.0,
                    'processing.enable_object_detection': True,
                    'processing.enable_lane_detection': True,
                    'processing.min_object_confidence': 0.5
                }
            ],
            remappings=[
                ('camera/image_raw', 'camera/image_raw'),
                ('camera/processed_image', 'camera/processed_image'),
                ('perception/detected_objects', 'perception/detected_objects'),
                ('perception/lane_center', 'perception/lane_center'),
            ],
        ),
        
        # LiDAR Processor Node
        Node(
            package='tadeo_ecar_perception',
            executable='lidar_processor_node',
            name='lidar_processor',
            output='screen',
            condition=lambda: LaunchConfiguration('enable_lidar').perform() == 'true',
            parameters=[
                robot_config,
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'processing.frequency': 10.0,
                    'filtering.min_range': 0.1,
                    'filtering.max_range': 30.0,
                    'filtering.voxel_size': 0.05,
                    'clustering.min_cluster_size': 10,
                    'clustering.max_cluster_size': 500,
                    'clustering.cluster_tolerance': 0.2
                }
            ],
            remappings=[
                ('scan', 'scan'),
                ('points', 'points'),
                ('scan_filtered', 'scan_filtered'),
                ('points_filtered', 'points_filtered'),
                ('perception/obstacles', 'perception/obstacles'),
                ('perception/free_space', 'perception/free_space'),
                ('perception/closest_obstacle', 'perception/closest_obstacle'),
            ],
        ),
        
        # IMU Processor Node
        Node(
            package='tadeo_ecar_perception',
            executable='imu_processor_node',
            name='imu_processor',
            output='screen',
            condition=lambda: LaunchConfiguration('enable_imu').perform() == 'true',
            parameters=[
                robot_config,
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'processing.frequency': 50.0,
                    'filtering.enable_calibration': True,
                    'filtering.calibration_samples': 1000,
                    'filtering.enable_lowpass': True,
                    'filtering.lowpass_alpha': 0.8,
                    'filtering.enable_highpass': True,
                    'filtering.highpass_alpha': 0.95
                }
            ],
            remappings=[
                ('imu/data_raw', 'imu/data_raw'),
                ('imu/data', 'imu/data'),
                ('imu/euler_angles', 'imu/euler_angles'),
                ('imu/linear_acceleration', 'imu/linear_acceleration'),
                ('imu/angular_velocity', 'imu/angular_velocity'),
            ],
        ),
        
        # Sensor Fusion Node
        Node(
            package='tadeo_ecar_perception',
            executable='sensor_fusion_node',
            name='sensor_fusion',
            output='screen',
            condition=lambda: LaunchConfiguration('enable_fusion').perform() == 'true',
            parameters=[
                robot_config,
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'fusion.frequency': 20.0,
                    'fusion.max_association_distance': 1.0,
                    'fusion.obstacle_timeout': 2.0,
                    'fusion.min_confidence': 0.3,
                    'fusion.track_threshold': 0.7,
                    'weights.imu': 1.0,
                    'weights.lidar': 0.8,
                    'weights.camera': 0.6
                }
            ],
            remappings=[
                ('imu/data', 'imu/data'),
                ('scan', 'scan'),
                ('camera/image_raw', 'camera/image_raw'),
                ('perception/obstacles', 'perception/obstacles'),
                ('perception/detected_objects', 'perception/detected_objects'),
                ('perception/fused_obstacles', 'perception/fused_obstacles'),
                ('perception/nearest_obstacle', 'perception/nearest_obstacle'),
            ],
        ),
    ])
    
    return LaunchDescription([
        use_sim_time_arg,
        enable_camera_arg,
        enable_lidar_arg,
        enable_imu_arg,
        enable_fusion_arg,
        perception_nodes,
    ])